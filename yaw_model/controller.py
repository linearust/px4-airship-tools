#!/usr/bin/env python3
"""Discrete model of the airship yaw rate loop as it runs in gz SITL, from the
yaw stick to the normalized yaw torque, plus the part of the airship allocator
that feeds the loop's anti-windup.

Source: PX4-Autopilot branch airship/dev at HEAD 4e1baea97c (2026-10-01). The
port was written against 1965de11ab and the 2026-09-27 working tree; the
2026-09-28 architecture pass moved the yaw path into AirshipYawRateLoop (YRL)
and d2647379af added AS_YAWRATE_FF, which reaches RateControl through
setGains() (YRL:58-69, called from AAC:104-105). Checked against HEAD on
2026-10-01: the loop law, the integrator and its anti-windup gate (YRL:76-82,
RC:88-118), the reset on opening (YRL:108-117) and the allocator (EA, POD)
are the same arithmetic as before; FF is the one addition. The parameter
defaults are read from the yaml files at import (DEFAULT_PARAMS below), and a
gz target's own set-defaults from its posix airframe chain (head_params()).

Paths below are relative to the PX4-Autopilot root:
  AAC  = src/modules/airship_att_control/airship_att_control_main.cpp
  AAH  = src/modules/airship_att_control/airship_att_control.hpp
  AMI  = src/modules/airship_att_control/airship_manual_input.hpp
  YRL  = src/modules/airship_att_control/airship_yaw_rate_loop.hpp
  RC   = src/lib/rate_control/rate_control.cpp
  RCH  = src/lib/rate_control/rate_control.hpp
  EA   = src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessAirship.cpp
  EAH  = src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessAirship.hpp
  POD  = src/modules/control_allocator/VehicleActuatorEffectiveness/AirshipPod.cpp
  PODH = src/modules/control_allocator/VehicleActuatorEffectiveness/AirshipPod.hpp
  CA   = src/modules/control_allocator/ControlAllocator.cpp
  VAV  = src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp

What one cycle is, in lockstep gz SITL (the built px4_sitl_default defines
ENABLE_LOCKSTEP_SCHEDULER: build/px4_sitl_default/compile_commands.json):
  1. gz steps 4 ms (Tools/simulation/gz/worlds/lta.sdf:8 max_step_size 0.004)
     and the IMU publishes at 250 Hz (models/cloudship/model.sdf:242,
     models/airship_base/model.sdf:217 update_rate 250), gyro noise
     0.0008726646 rad/s (cloudship/model.sdf:261, airship_base/model.sdf:236).
  2. gz_bridge imuCallback -> sensor_gyro (src/modules/simulation/gz_bridge/
     GZBridge.cpp:436-459), timestamp_sample = the IMU header stamp capped at
     the current time (GZBridge.cpp:438-441).
  3. VehicleAngularVelocity filters every sample with a 2nd-order Butterworth
     at IMU_GYRO_CUTOFF 40 Hz (imu_gyro_parameters.yaml:83; VAV:770-771),
     subtracts the EKF gyro bias (VAV:939) and publishes. The static notches
     are off (IMU_GYRO_NF0_FRQ / NF1_FRQ default 0, imu_gyro_parameters.yaml:17,
     :51) and so is the dynamic notch: SITL starts gyro_fft
     (init.d-posix/rcS:172, 362-365) but IMU_GYRO_DNF_EN defaults to 0
     (imu_gyro_parameters.yaml:141), which VAV:676 requires for the FFT notch.
     IMU_GYRO_RATEMAX 400
     (imu_gyro_parameters.yaml:106) against a 250 Hz sensor gives
     round(2500/4000) = 1 sample per publish and a 500 us minimum interval
     (VAV:134-150), so vehicle_angular_velocity is 250 Hz.
  4. vehicle_angular_velocity triggers AirshipAttitudeControl::Run()
     (AAC:78 registerCallback, AAH:130), dt from timestamp_sample, clamped to
     [0.125 ms, 20 ms] (AAC:186-188). While the loop is closed it reads the
     latest control_allocator_status (AAC:159-166, YRL:76-82), and it
     publishes thrust and torque together (AAC:224-226, publishWrench
     AAC:123-137).
  5. vehicle_torque_setpoint triggers ControlAllocator::Run() (CA:96, CA:387),
     same rate_ctrl work queue (CA:57, AAC:55), so it runs after the
     controller in the same cycle. It publishes control_allocator_status only
     when now - last >= 5 ms (CA:464), i.e. every second 4 ms cycle, so the
     controller at cycle k sees the allocation of cycle k-1 or k-2.
The plant side (gz_bridge outputs, gz motor/servo models, when gz applies a
command) is not modelled here.

Checked against the gate runs' flight logs (ulog_check.py; build/px4_sitl_default/
rootfs/fs/log/2026-09-25/14_34_05, 14_39_53, 14_45_54, 14_50_29 and 2026-09-27/
08_31_54, 08_36_30): every logged vehicle_angular_velocity, torque and actuator
timestamp_sample is on the 4 ms grid with timestamp == timestamp_sample;
control_allocator_status is published at one fixed phase of an 8 ms period
per run (0 or 4000 mod 8000 us), i.e. every second cycle (one log shows the
phase shift by 4 ms twice and one gyro sample delivered a cycle late
in 260 s: rare SITL hiccups this model does not reproduce); a mode change is
acted on in the cycle whose time equals the vehicle_control_mode timestamp.
Driven by the logged rate, sticks and modes, SitlYawChain reproduces the
logged torque and integrator to p99 <= 1.3e-4, the tilt servos and motors,
and the logged yaw saturation sign on 8946 of 8948 status samples. The two
misses are the last status sample of a pod swing, whose end the model puts
20-50 ms after SITL's: once from the 20 ms resolution of the stick
reconstruction in Manual, once where the demand sat at the 0.02 steer
threshold and the swing start is that sensitive.
"""
import functools
import glob
import math
import os
import re

import yaml

FLT_EPSILON = 1.1920929e-07
PX4 = os.environ.get('PX4_ROOT', os.path.expanduser('~/dev/PX4-Autopilot'))

# ---------------------------------------------------------------------------
# Parameters (defaults and the two airframes)
# ---------------------------------------------------------------------------

# Every default the model uses, read from the module yaml that defines it, so
# the model is the firmware it reads: until 2026-10-01 this table restated
# AS_YAWRATE_P 0.5 a week after d2647379af made it 0.7, and had no FF at all.
# A replay of a record flown with other gains passes them explicitly
# (record_gains.py); nothing historical may lean on these.
AS_YAML = 'src/modules/airship_att_control/airship_att_control_params.yaml'
PARAM_FILES = {
    'AS_YAWRATE_P': AS_YAML,
    'AS_YAWRATE_I': AS_YAML,
    'AS_YR_INT_LIM': AS_YAML,
    'AS_YAWRATE_FF': AS_YAML,
    'AS_YAWRATE_MAX': AS_YAML,       # deg/s
    'MAN_DEADZONE': 'src/modules/manual_control/manual_control_params.yaml',
    'IMU_GYRO_CUTOFF': 'src/modules/sensors/vehicle_angular_velocity/imu_gyro_parameters.yaml',     # Hz
}
_YAML_CACHE = {}


def yaml_default(relpath, name):
    """The default of one parameter in a module yaml (parameters: - group:
    definitions: NAME: default). Raises when the file or the name is not
    there: a default that cannot be read is not a default of 0."""
    if relpath not in _YAML_CACHE:
        with open(os.path.join(PX4, relpath), encoding='utf-8') as handle:
            _YAML_CACHE[relpath] = yaml.safe_load(handle)

    for group in _YAML_CACHE[relpath].get('parameters', []):
        spec = group.get('definitions', {}).get(name)

        if spec is not None:
            if 'default' not in spec:
                raise LookupError('%s: %s has no default' % (relpath, name))
            return spec['default']

    raise LookupError('%s does not define %s' % (relpath, name))


DEFAULT_PARAMS = {name: float(yaml_default(path, name)) for name, path in PARAM_FILES.items()}

# ---------------------------------------------------------------------------
# The posix airframe chain of a gz target: what its set-defaults make of the
# yaml defaults. A file sources another with `. ${R}etc/init.d-posix/...`
# (2527 sources 2520 after setting PX4_SIM_MODEL, then sets AS_YAWRATE_FF),
# and a later `param set-default` of the same name wins, as in the shell.
# ---------------------------------------------------------------------------

ROMFS = os.path.join(PX4, 'ROMFS/px4fmu_common')
POSIX_AIRFRAMES = os.path.join(ROMFS, 'init.d-posix/airframes')
_SOURCE = re.compile(r'^\s*\.\s+\$\{R\}etc/(\S+)\s*$')
_SET_DEFAULT = re.compile(r'^\s*param set-default (\w+) (\S+)')
_SIM_MODEL = re.compile(r'^\s*PX4_SIM_MODEL=\$\{PX4_SIM_MODEL:=(\S+)\}')


def airframe_file(number):
    """The posix airframe file of one SYS_AUTOSTART number, exactly one."""
    found = glob.glob(os.path.join(POSIX_AIRFRAMES, '%d_*' % int(number)))

    if len(found) != 1:
        raise LookupError('posix airframe %s: %d files (%s)' % (number, len(found), found))

    return found[0]


def airframe_walk(path, _depth=0):
    """(kind, name, value, file, line) in shell order through every sourced
    file: kind 'default' for param set-default, 'model' for the
    PX4_SIM_MODEL default."""
    if _depth > 8:
        raise RecursionError('airframe sourcing deeper than 8 at %s' % path)

    with open(path, encoding='utf-8') as handle:
        for number, line in enumerate(handle, 1):
            m = _SOURCE.match(line)

            if m:
                yield from airframe_walk(os.path.join(ROMFS, m.group(1)), _depth + 1)
                continue

            m = _SET_DEFAULT.match(line)

            if m:
                yield 'default', m.group(1), m.group(2), path, number
                continue

            m = _SIM_MODEL.match(line)

            if m:
                yield 'model', 'PX4_SIM_MODEL', m.group(1), path, number


def airframe_defaults(number):
    """{name: (value string, file, line)} after the whole chain: the last
    set-default of a name is the one the parameter keeps."""
    return dict(_airframe_defaults(int(number)))


@functools.lru_cache(maxsize=None)
def _airframe_defaults(number):
    out = {}

    for kind, name, value, path, line in airframe_walk(airframe_file(number)):
        if kind == 'default':
            out[name] = (value, os.path.relpath(path, PX4), line)

    return out


def airframe_model(number):
    """The gz model the airframe spawns: the first PX4_SIM_MODEL default in
    shell order, since ${PX4_SIM_MODEL:=...} keeps the first one set."""
    for kind, _, value, _, _ in airframe_walk(airframe_file(number)):
        if kind == 'model':
            return value

    raise LookupError('posix airframe %s sets no PX4_SIM_MODEL' % number)


@functools.lru_cache(maxsize=None)
def airframe_for_model(model):
    """SYS_AUTOSTART of the posix airframe that spawns gz `model` (no gz_
    prefix), exactly one. Cached per process: the files are read once."""
    hits, unread = [], []

    for path in sorted(glob.glob(os.path.join(POSIX_AIRFRAMES, '[0-9]*_*'))):
        number = int(os.path.basename(path).split('_')[0])

        # Some airframes source a build-generated defaults file before they
        # name their model (rc.rover_defaults is not in ROMFS); they are
        # counted, not guessed at, and named if the match comes out wrong
        try:
            if airframe_model(number) == model:
                hits.append(number)
        except (LookupError, OSError):
            unread.append(number)

    if len(hits) != 1:
        raise LookupError('gz model %s: %d posix airframes spawn it (%s); %d airframes unreadable: %s'
                          % (model, len(hits), hits, len(unread), unread))

    return hits[0]


def head_params(model):
    """The controller parameters gz `model` flies at HEAD: the yaml defaults
    with its airframe chain's set-defaults on top (only the names this model
    reads; 2527, for one, sets AS_YAWRATE_FF 1.0), and THR_MDL_FAC, which the
    gate reads. Raises when the airframe or a value cannot be read."""
    number = airframe_for_model(model)
    defaults = airframe_defaults(number)
    params = dict(DEFAULT_PARAMS)

    for name in params:
        if name in defaults:
            params[name] = float(defaults[name][0])

    if 'THR_MDL_FAC' not in defaults:
        raise LookupError('posix airframe %d does not set THR_MDL_FAC' % number)

    params['THR_MDL_FAC'] = float(defaults['THR_MDL_FAC'][0])
    params['SYS_AUTOSTART'] = number
    return params


LOOP_HZ = 250.0
DT_US = 4000

# How the firmware applies AS_YAWRATE_FF. 'quadratic' (airship_att_control since
# 2026-10-12): the gain RateControl multiplies the setpoint by is FF0 |sp| / r_max
# plus the speed schedule's term (airship_yaw_schedule::feedforwardGain), since
# the hull's yaw drag grows with the square of the rate; the schedule's cap is the
# positive root of FF0 r^2 / r_max + FF_u r = A - rho. 'linear' (before): FF0 as
# the gain, FF0 + FF_u from the schedule, the cap (A - rho) / FF. The scripts that
# replay records flown before the change set 'linear'.
FF_LAW = 'quadratic'

# Allocator configuration per plant (ROMFS/px4fmu_common/init.d-posix/airframes).
# The _am_fin_drag variants (2517, 2527) only change PX4_SIM_MODEL and source
# these files (2517_gz_cloudship_am_fin_drag:11-13,
# 2527_gz_airship_vectored_independent_am_fin_drag:11-13).
AIRFRAMES = {
    # 2520_gz_airship_vectored_independent:53-57: CA_AIRFRAME 16, GRP 1,
    # TLMIN -180, TLMAX 180, TLT_R 120. CA_AIRSHIP_TAIL not set -> 0
    # (control_allocator/module.yaml:654); CA_SV_CS_COUNT not set -> 0
    # (module.yaml:268); CA_AIRSHIP_CS_K not set -> 1.0 (module.yaml:728);
    # CA_R_REV not set -> 0 (module.yaml:73).
    'vectored': dict(CA_AIRFRAME=16, CA_AIRSHIP_GRP=1, CA_AIRSHIP_TAIL=False,
                     CA_AIRSHIP_TLMIN=-180.0, CA_AIRSHIP_TLMAX=180.0, CA_AIRSHIP_TLT_R=120.0,
                     CA_AIRSHIP_CS_K=1.0, CA_R_REV=0, yaw_surfaces=False),
    # 2510_gz_cloudship:79-86: CA_AIRFRAME 16, GRP 0, TAIL 1, TLMIN -90,
    # TLMAX 90, TLT_R 120, CA_R_REV 4 (motor 3 = the tail, reversible).
    'cloudship': dict(CA_AIRFRAME=16, CA_AIRSHIP_GRP=0, CA_AIRSHIP_TAIL=True,
                      CA_AIRSHIP_TLMIN=-90.0, CA_AIRSHIP_TLMAX=90.0, CA_AIRSHIP_TLT_R=120.0,
                      CA_AIRSHIP_CS_K=1.0, CA_R_REV=4, yaw_surfaces=False),
}
PLANT_AIRFRAME = {
    'gz_airship_vectored_independent': 'vectored',
    'gz_airship_vectored_independent_am_fin_drag': 'vectored',
    'gz_cloudship': 'cloudship',
    'gz_cloudship_am_fin_drag': 'cloudship',
}
CA_YAML = 'src/modules/control_allocator/module.yaml'


def check_allocator_config(key, number):
    """AIRFRAMES[key] against airframe `number`'s chain, the names it leaves
    unset against control_allocator/module.yaml. Returns the mismatches, []
    when the table is the file. yaw_surfaces stands for hasYawSurfaces()
    (AMI:132-145): False needs CA_SV_CS_COUNT 0."""
    defaults = airframe_defaults(number)
    bad = []

    def value(name):
        if name in defaults:
            return float(defaults[name][0])
        return float(yaml_default(CA_YAML, name))

    for name, want in AIRFRAMES[key].items():
        if name == 'yaw_surfaces':
            got = value('CA_SV_CS_COUNT') > 0
        else:
            got = value(name)

        if float(got) != float(want):
            bad.append('%s: table %s, airframe %d %s' % (name, want, number, got))

    return bad

# ---------------------------------------------------------------------------
# Flight modes -> vehicle_control_mode flags the loop gate reads
# ---------------------------------------------------------------------------
# The airship is MAV_TYPE 7 (rc.airship_defaults:11), which is neither rotary,
# fixed-wing nor ground (commander_helper.cpp:92-124), so vehicle_type stays
# VEHICLE_TYPE_UNSPECIFIED (Commander.cpp:816, 1935-1950). Hence Manual maps to
# ThrustAndTorque (ModeUtil/control_mode.cpp:52-60, setpoint_types.cpp:150-151:
# allocation only, rates OFF), Acro to Rates (control_mode.cpp:94-96,
# setpoint_types.cpp:85-88) and Stabilized to Attitude (control_mode.cpp:64-66,
# setpoint_types.cpp:90-94), both with manual and rates ON.
# (flag_control_manual_enabled, flag_control_rates_enabled)
MODE_FLAGS = {
    'manual': (True, False),
    'acro': (True, True),
    'stabilized': (True, True),
    'altctl': (True, True),     # control_mode.cpp:69-75, Trajectory: setpoint_types.cpp:75-83
    'posctl': (True, True),     # control_mode.cpp:77-81
    'auto': (False, True),      # control_mode.cpp:83-92: no manual flag -> loop open
    'termination': (False, False),
}


# ---------------------------------------------------------------------------
# Small PX4 helpers
# ---------------------------------------------------------------------------

def constrain(x, lo, hi):
    """math::constrain: (x < lo) ? lo : ((x > hi) ? hi : x); NaN passes through."""
    return lo if x < lo else (hi if x > hi else x)


def isfinite(x):
    return x == x and x not in (float('inf'), float('-inf'))


def finite_or(value, fallback):
    """AMI:57-60."""
    return value if isfinite(value) else fallback


def sign(x):
    """matrix::sign (lib/matrix/matrix/helper_functions.hpp:147-150)."""
    return (0.0 < x) - (x < 0.0)


def deadzone(value, dz):
    """math::deadzone (lib/mathlib/math/Functions.hpp:125-133)."""
    x = constrain(value, -1.0, 1.0)
    dzc = constrain(dz, 0.0, 0.99)
    out = (x - sign(x) * dzc) / (1.0 - dzc)
    return out * (1.0 if abs(x) > dzc else 0.0)


def yaw_rate_setpoint(stick, dz, max_rate_rad):
    """AMI:91-94: deadzone first, then linear scaling; NaN stick reads as 0."""
    return deadzone(finite_or(stick, 0.0), dz) * max_rate_rad


def manual_thrust(throttle, pitch):
    """AMI:70-73, the thrust setpoint per body axis (FRD): the throttle stick
    forward, (throttle + 1) / 2, a NaN throttle read as -1 (no thrust); the
    pitch stick vertical."""
    return ((finite_or(throttle, -1.0) + 1.0) * 0.5, 0.0, finite_or(pitch, 0.0))


def manual_torque(roll, pitch, yaw):
    """AMI:80-83, the torque passthrough per body axis (FRD)."""
    return (finite_or(roll, 0.0), -finite_or(pitch, 0.0), finite_or(yaw, 0.0))


def throttle_stick(fraction, quantize=True):
    """manual_control_setpoint.throttle for a thrust `fraction` (0..1): the
    receiver reads the MANUAL_CONTROL z as z / 500 - 1
    (src/modules/mavlink/mavlink_receiver.cpp:2379), so the thrust fraction
    PX4 flies is z / 1000. What z is:
      True        sitl_link.Sticks given the fraction itself: it sends
                  z = int(clamp(fraction, 0, 1) * 1000) (bench sitl_link.py:
                  307, the int truncates).
      'midpoint'  what the 0b check flies (bench forward_yaw_check.quantize):
                  it rounds the fraction to the nearest count, counts =
                  int(round(clamp(fraction, 0, 1) * 1000)) (Python's round,
                  half to even), and hands Sticks that count's midpoint
                  (counts + 0.5) / 1000, which the truncation turns into
                  z = counts.
      False       the fraction exactly, z = fraction * 1000 (not an integer):
                  a recorded thrust setpoint, or a sweep that wants the
                  speed it is labelled with."""
    f = max(0.0, min(1.0, fraction))

    if quantize == 'midpoint':
        counts = int(round(f * 1000.0))
        z = int(((counts + 0.5) / 1000.0) * 1000)       # Sticks' truncation of the midpoint: counts
    else:
        z = int(f * 1000) if quantize else f * 1000.0

    return z / 500.0 - 1.0


def propulsive_yaw(allocator, pod_grouping, tail, yaw_surfaces, surface_credit):
    """AMI:176-184 (yaw_surfaces: hasYawSurfaces(), AMI:132-145)."""
    if allocator != 16:     # kCaAirframeAirship (AirshipAirframe.hpp)
        return True
    return (pod_grouping > 0 or bool(tail)) and not (yaw_surfaces and surface_credit >= 1.0)


def yaw_rate_loop_active(mode, manual_input_usable, propulsive):
    """AMI:196-201."""
    manual_enabled, rates_enabled = MODE_FLAGS[mode]
    return manual_input_usable and propulsive and manual_enabled and rates_enabled


class LowPassFilter2p:
    """math::LowPassFilter2p (lib/mathlib/math/filter/LowPassFilter2p.hpp:61-109),
    Direct Form II, coefficients from tan(pi / (fs / fc))."""

    def __init__(self, sample_freq, cutoff_freq):
        self.d1 = 0.0
        self.d2 = 0.0

        if sample_freq <= 0 or cutoff_freq <= 0 or cutoff_freq >= sample_freq / 2:
            self.b0, self.b1, self.b2, self.a1, self.a2 = 1.0, 0.0, 0.0, 0.0, 0.0
            return

        fr = sample_freq / max(cutoff_freq, sample_freq * 0.001)
        ohm = math.tan(math.pi / fr)
        c = 1.0 + 2.0 * math.cos(math.pi / 4.0) * ohm + ohm * ohm
        self.b0 = ohm * ohm / c
        self.b1 = 2.0 * self.b0
        self.b2 = self.b0
        self.a1 = 2.0 * (ohm * ohm - 1.0) / c
        self.a2 = (1.0 - 2.0 * math.cos(math.pi / 4.0) * ohm + ohm * ohm) / c

    def apply(self, sample):
        d0 = sample - self.d1 * self.a1 - self.d2 * self.a2
        out = d0 * self.b0 + self.d1 * self.b1 + self.d2 * self.b2
        self.d2, self.d1 = self.d1, d0
        return out

    def reset(self, sample):
        """LowPassFilter2p.hpp:128-144."""
        k = 1.0 + self.a1 + self.a2
        self.d1 = self.d2 = sample / k if abs(k) > FLT_EPSILON else sample
        return self.apply(sample)


# ---------------------------------------------------------------------------
# RateControl, yaw axis only
# ---------------------------------------------------------------------------

class RateControlYaw:
    """lib/rate_control, the yaw component.

    update(): torque = P*e + I_state - D*accel + FF*sp with the integrator state
    from BEFORE this cycle (RC:75-78), then the integrator is advanced (RC:80-83,
    88-118). The airship passes landed = false always (YRL:96-97), D = 0 (YRL:61
    passes Vector3f() for D) and accel = 0 (YRL:96-97). FF is
    AS_YAWRATE_FF, set with the gains by setGains() (YRL:58-69 ->
    setFeedForwardGain, RCH:72) on every parameter update (AAC:104-105).
    FF enters the torque only: updateIntegral() integrates the rate error,
    clipped to one sign by the saturation flags (RC:91-99), and never sees
    the feedforward. It reaches the anti-windup only through the allocator,
    whose shortfall is taken on the whole torque it was handed. Nothing clips
    the returned torque (the "[-1,1]" of RCH:94 is a comment only).
    """

    I_FACTOR_RATE = math.radians(400.0)     # RC:107

    def __init__(self, p, i, int_lim, ff=0.0):
        self.p = p
        self.i = i
        self.int_lim = int_lim
        self.ff = ff
        self.integ = 0.0
        self.sat_pos = False
        self.sat_neg = False

    def update(self, rate, rate_sp, dt, landed=False):
        e = rate_sp - rate                                       # RC:75
        torque = self.p * e + self.integ + self.ff * rate_sp     # RC:78
        if not landed:                                           # RC:81
            self._update_integral(e, dt)
        return torque

    def _update_integral(self, e, dt):
        if self.sat_pos:                                         # RC:92-94
            e = min(e, 0.0)
        if self.sat_neg:                                         # RC:97-99
            e = max(e, 0.0)
        f = e / self.I_FACTOR_RATE                               # RC:107-108
        f = max(0.0, 1.0 - f * f)
        rate_i = self.integ + f * self.i * e * dt                # RC:111
        if isfinite(rate_i):                                     # RC:114-116
            self.integ = constrain(rate_i, -self.int_lim, self.int_lim)

    def set_saturation(self, positive, negative):               # RC:57-69, YRL:80-81
        self.sat_pos = positive
        self.sat_neg = negative

    def set_gains(self, p, i, int_lim, ff):                      # YRL:58-69
        """setPidGains, setIntegratorLimit and setFeedForwardGain together;
        the integrator state is left as it is (RC has no reset there)."""
        self.p = p
        self.i = i
        self.int_lim = int_lim
        self.ff = ff

    def reset(self):                                             # RCH:103, YRL:115
        self.integ = 0.0


class Topic:
    """A uORB topic as the subscriber sees it: the latest message and a
    generation counter; Subscription::update() returns true once per new
    generation."""

    def __init__(self):
        self.msg = None
        self.gen = 0

    def publish(self, msg):
        self.msg = msg
        self.gen += 1


class AirshipYawController:
    """AirshipAttitudeControl::Run() for the yaw axis, stepped once per
    vehicle_angular_velocity sample (250 Hz in gz SITL).

    Inputs per step: the sample time [us], the raw gz gyro z [rad/s] (or the
    already filtered rate with filter_gyro=False), the yaw stick [-1, 1], the
    flight mode name, armed, sticks valid, and the control_allocator_status
    Topic. Output: the normalized yaw torque the controller publishes in
    vehicle_torque_setpoint.xyz[2].

    Policy primitives for the D1 0c comparison, none in HEAD's firmware, all
    off at construction (off, step() is bit-identical to the controller
    without them): set_feedforward() (the per-cycle FF gain),
    set_rate_cap() (the yaw-rate cap as the stick's full scale and as a
    clamp on the loop's setpoint), set_restore() (options/D's restore rule).
    A per-cycle caller (forward.run_leg's hook) sets them between cycles.
    """

    def __init__(self, airframe='vectored', params=None, filter_gyro=True, gyro_bias=0.0):
        self.params = dict(DEFAULT_PARAMS)
        self.params.update(params or {})
        self.airframe = dict(AIRFRAMES[airframe]) if isinstance(airframe, str) else dict(airframe)
        self.rc = RateControlYaw(0.0, 0.0, 0.0)
        self.filter_gyro = filter_gyro
        self.gyro_bias = gyro_bias
        self._lpf = LowPassFilter2p(LOOP_HZ, self.params['IMU_GYRO_CUTOFF'])
        self._last_run = 0            # AAH:140 _last_run{0}
        self._was_active = False      # YRL:124 _closed{false}
        self._status_gen_seen = 0
        self.parameters_updated()
        # The D1 plan's policy primitives (d1_universal_architecture_20261001.md
        # "2) 구조" 다, and "남은 결정" 1), for the 0c comparison. All off here;
        # off, step() runs exactly the arithmetic it ran before they existed.
        self._ff_speed = 0.0          # the schedule's speed term, FF_LAW 'quadratic' (set_feedforward)
        self.rate_cap = None          # rad/s; None: no cap (HEAD). set_rate_cap()
        self.rate_cap_neg = None      # rad/s, the negative stick's cap where it differs; None: rate_cap both ways
        self.cap_stick = True         # the cap is the stick's full scale (AAC:206-207's max_rate)
        self.cap_clamp = True         # the cap clamps the setpoint handed to the loop (AAC:208)
        self.restore = None           # None, or options/D's restore rule with its edge. set_restore()
        self._restore_sp_prev = 0.0   # the setpoint of the last closed cycle (options/D _sp_prev)
        self._restore_trim = 0.0      # the integrator saved when the setpoint left 0 (options/D _trim)
        self.restore_events = []      # (t_us, value put back), one per restore
        # last-cycle diagnostics
        self.rate = 0.0
        self.rate_sp = 0.0
        self.loop_active = False
        self.dt = 0.0

    def parameters_updated(self):
        """AAC:101-121: setGains(P, I, INT_LIM, FF) (YRL:58-69) and the stick's
        full scale. Does NOT touch the integrator: a changed AS_YAWRATE_I stops
        or rescales integration but keeps what is accumulated. In the firmware
        the update is polled at the END of Run() through a 1 s
        SubscriptionInterval (AAC:228, AAH:125), so a param_set lands up to
        1 s later and takes effect the cycle after."""
        p = self.params
        self.rc.set_gains(p['AS_YAWRATE_P'], p['AS_YAWRATE_I'], p['AS_YR_INT_LIM'], p['AS_YAWRATE_FF'])
        self.yaw_rate_max = math.radians(p['AS_YAWRATE_MAX'])
        a = self.airframe
        self.propulsive_yaw = propulsive_yaw(a['CA_AIRFRAME'], a['CA_AIRSHIP_GRP'], a['CA_AIRSHIP_TAIL'],
                                             a['yaw_surfaces'], a['CA_AIRSHIP_CS_K'])

    def set_param(self, name, value):
        self.params[name] = value
        self.parameters_updated()

    # -- policy primitives (off by default) ------------------------------------

    def set_feedforward(self, ff):
        """The speed schedule's feedforward term for this cycle on, normalized
        torque per rad/s, finite, >= 0 (the yaml's minimum). FF_LAW 'quadratic':
        the term the firmware's schedule adds to AS_YAWRATE_FF's share of the
        setpoint (airship_yaw_schedule::feedforwardGain, applied in step()).
        FF_LAW 'linear' (the firmware before 2026-10-12): the whole gain,
        RateControl::setFeedForwardGain on the yaw axis from this cycle on
        (RCH:72), as fixed-wing sets it every cycle
        (FixedwingRateControl.cpp:389-392). FF only: P, I, the integrator limit
        and the integrator are untouched. A parameter update (set_param) puts
        AS_YAWRATE_FF back as the gain, as setGains() does in the firmware
        (YRL:58-69)."""
        if not (isfinite(ff) and ff >= 0.0):
            raise ValueError('feedforward %r: not a finite gain >= 0' % (ff,))

        if FF_LAW == 'quadratic':
            self._ff_speed = float(ff)
        else:
            self.rc.ff = float(ff)

    def set_rate_cap(self, cap_deg_s, stick=True, clamp=True, cap_neg_deg_s=None):
        """The yaw-rate cap of the plan (2.다 상한, applied in two places):
        stick, the stick's full scale (yawRateSetpoint's max_rate, AAC:206-207,
        so the whole stick spans the cap); clamp, a constrain of the setpoint
        handed to the rate loop (AAC:208, the rover's placement:
        AckermannRateControl.cpp:71-74, which limits any setpoint source). The
        proposal uses both; each can be left out to compare the two
        placements. cap_deg_s: deg/s, finite, >= 0 (AS_YAWRATE_MAX's unit);
        None removes the cap. A cap equal to AS_YAWRATE_MAX, both places, is
        bit-identical to no cap: the same max_rate, and the constrain returns
        its input (|deadzone()| <= 1).

        cap_neg_deg_s: the negative (nose left) stick's cap where the plan's
        per-direction authority makes it differ (2.나: a tail without its
        reverse bit); the stick's full scale is then cap_deg_s for a positive
        and cap_neg_deg_s for a negative deadzoned stick, and the clamp is
        [-cap_neg, cap]. None (default): cap_deg_s both ways, the arithmetic
        unchanged."""
        if cap_deg_s is None:
            self.rate_cap = None
            self.rate_cap_neg = None
            return

        if not (isfinite(cap_deg_s) and cap_deg_s >= 0.0):
            raise ValueError('rate cap %r deg/s: not a finite rate >= 0' % (cap_deg_s,))

        if cap_neg_deg_s is not None and not (isfinite(cap_neg_deg_s) and cap_neg_deg_s >= 0.0):
            raise ValueError('negative-stick rate cap %r deg/s: not a finite rate >= 0' % (cap_neg_deg_s,))

        self.rate_cap = math.radians(cap_deg_s)
        self.rate_cap_neg = None if cap_neg_deg_s is None else math.radians(cap_neg_deg_s)
        self.cap_stick = bool(stick)
        self.cap_clamp = bool(clamp)

    def set_restore(self, edge='zero_or_sign'):
        """options/D's 'restore' rule (yaw_model/options/D/option_d.py
        OptionD, kind 'restore'), a candidate safeguard of "남은 결정" 1: on
        the closed cycle the setpoint leaves zero (the stick leaves the
        deadzone) the integrator is saved; on the cycle it returns to zero, or
        reverses sign without passing zero (edge 'zero_or_sign', MAN_DEADZONE >
        0), the saved value is put back. 'zero' takes only the first trigger
        (options/D's edge='zero', 'restore0'). The put-back is setIntegral: the
        integrator is assigned, before RateControl::update of that cycle, so
        that cycle's torque already uses it; no clamp at the assignment. The
        saved value and the previous setpoint are cleared when the loop
        closes, as OptionD does. None switches it off."""
        if edge not in (None, 'zero_or_sign', 'zero'):
            raise ValueError('restore edge %r: None, zero_or_sign or zero' % (edge,))

        self.restore = edge

    def _setpoint(self, stick_yaw):
        """AAC:206-207 yawRateSetpoint, and with a cap set, the plan's two
        applications of it (set_rate_cap)."""
        if self.rate_cap is None:
            return yaw_rate_setpoint(stick_yaw, self.params['MAN_DEADZONE'], self.yaw_rate_max)

        cap = self.rate_cap

        if self.rate_cap_neg is None:
            sp = yaw_rate_setpoint(stick_yaw, self.params['MAN_DEADZONE'], cap if self.cap_stick else self.yaw_rate_max)
            return constrain(sp, -cap, cap) if self.cap_clamp else sp

        neg = self.rate_cap_neg
        d = deadzone(finite_or(stick_yaw, 0.0), self.params['MAN_DEADZONE'])
        sp = d * ((cap if d >= 0.0 else neg) if self.cap_stick else self.yaw_rate_max)
        return constrain(sp, -neg, cap) if self.cap_clamp else sp

    def _restore_edge(self, sp):
        """OptionD._edge: the setpoint returns to zero, or (edge
        'zero_or_sign' with a deadzone) reverses sign in one cycle."""
        prev = self._restore_sp_prev

        if self.restore == 'zero':
            return sp == 0.0 and prev != 0.0

        return (sp == 0.0 and prev != 0.0) or (self.params['MAN_DEADZONE'] > 0.0 and sp * prev < 0.0)

    def _update_saturation(self, status_topic):
        """AAC:159-166 -> YRL:76-82: only the sign test on the yaw element
        matters here, gated by torque_setpoint_achieved. The flags persist
        until the next status message is read."""
        if status_topic is None or status_topic.gen == self._status_gen_seen:
            return
        self._status_gen_seen = status_topic.gen
        achieved, unallocated = status_topic.msg['torque_setpoint_achieved'], status_topic.msg['unallocated_torque']
        pos = neg = False
        if not achieved:
            u = unallocated[2]
            if u > FLT_EPSILON:
                pos = True
            elif u < -FLT_EPSILON:
                neg = True
        self.rc.set_saturation(pos, neg)

    def step(self, t_us, gyro_z, stick_yaw, mode='acro', armed=True, sticks_valid=True, status_topic=None):
        # vehicle_angular_velocity (VAV:770-771, 939)
        rate = (self._lpf.apply(gyro_z) if self.filter_gyro else gyro_z) - self.gyro_bias
        self.rate = rate
        # AAC:186-188
        dt = constrain((t_us - self._last_run) * 1e-6, 0.000125, 0.02)
        self._last_run = t_us
        self.dt = dt
        # AAC:195-198: torque passthrough unless the loop takes the yaw axis
        # (AMI:102-105 usable, AMI:80-83 torque)
        usable = bool(armed) and bool(sticks_valid)
        torque = finite_or(stick_yaw, 0.0) if usable else 0.0
        # AAC:201-202
        active = yaw_rate_loop_active(mode, usable, self.propulsive_yaw)

        if active:
            self._update_saturation(status_topic)                  # AAC:205
            sp = self._setpoint(stick_yaw)                         # AAC:206-207 (and a cap, if set)
            self.rate_sp = sp

            if not self._was_active:                               # OptionD: cleared when the loop closes
                self._restore_sp_prev = 0.0
                self._restore_trim = 0.0

            if self.restore is not None:                           # OptionD._before_update, kind 'restore'
                if self._restore_sp_prev == 0.0 and sp != 0.0:
                    self._restore_trim = self.rc.integ
                elif self._restore_edge(sp):
                    self.rc.integ = self._restore_trim
                    self.restore_events.append((t_us, self._restore_trim))

            if FF_LAW == 'quadratic':
                # AYS feedforwardGain: AS_YAWRATE_FF holds AS_YAWRATE_MAX against the hull's
                # drag, quadratic in the rate, so the setpoint gets its share of it, plus the
                # schedule's speed term (AAC: setFeedForward every closed cycle)
                share = abs(sp) / self.yaw_rate_max if self.yaw_rate_max > 0.0 else 1.0
                self.rc.ff = self.params['AS_YAWRATE_FF'] * share + self._ff_speed

            t = self.rc.update(rate, sp, dt, landed=False)         # AAC:208, YRL:96-97
            torque = t if isfinite(t) else 0.0                     # YRL:99
            self._restore_sp_prev = sp
        elif self._was_active:
            self.rc.reset()                                        # AAC:216, YRL:108-117
            self.rate_sp = 0.0
            self._restore_sp_prev = 0.0
        else:
            self.rate_sp = 0.0

        self._was_active = active                                  # YRL:90, :114
        self.loop_active = active
        return torque

    @property
    def integ(self):
        return self.rc.integ


# ---------------------------------------------------------------------------
# The allocator's side of the anti-windup: ActuatorEffectivenessAirship
# ---------------------------------------------------------------------------

STEERING, HOLDING, RELEASED, PARKED = 'steering', 'holding', 'released', 'parked'


class AirshipPod:
    """AirshipPod (PODH, POD), float64."""
    K_STEER_ENGAGE = 0.02       # PODH:54
    K_STEER_RELEASE = 0.01      # PODH:55
    K_END_SWITCH_MARGIN = 0.05  # PODH:60
    K_MIN_TILT_SPAN = 1e-3      # PODH:112
    K_TILT_SETTLED = 1e-3       # PODH:113

    def __init__(self):
        self.tilt = 0.0             # SlewRate state (lib/slew_rate/SlewRate.hpp:91)
        self.target = 0.0           # PODH:133
        self.tilt_min = 0.0
        self.tilt_max = 0.0
        self.slew_rate = 0.0
        self.slew_limited = False
        self.mode = PARKED          # PODH:137

    def set_tilt_range(self, tmin, tmax):           # POD:41-47
        self.tilt_min = tmin
        self.tilt_max = max(tmax, tmin)
        self.tilt = constrain(self.tilt, self.tilt_min, self.tilt_max)
        self.target = constrain(self.target, self.tilt_min, self.tilt_max)

    def set_tilt_slew_rate(self, rate):             # PODH:66
        self.slew_limited = rate > 0.0
        self.slew_rate = rate

    def can_tilt(self):                             # PODH:68
        return self.tilt_max - self.tilt_min > self.K_MIN_TILT_SPAN

    @staticmethod
    def realized(d, tilt):                          # PODH:128
        return d[0] * math.cos(tilt) + d[1] * math.sin(tilt)

    def steer(self, d, dt):                         # POD:49-70
        mag = math.hypot(d[0], d[1])
        engaged = self.mode in (STEERING, HOLDING)
        if mag > self.K_STEER_ENGAGE:
            self.mode = STEERING
            self.target = self._steer_target(d, mag)
        elif engaged and mag > self.K_STEER_RELEASE:
            self.mode = HOLDING
        else:
            self.mode = RELEASED
            self.target = self.tilt
        self._slew(dt)

    def park(self, dt):                             # POD:72-77
        self.mode = PARKED
        self.target = constrain(0.0, self.tilt_min, self.tilt_max)
        self._slew(dt)

    def _slew(self, dt):                            # POD:79-87, SlewRate.hpp:79-87
        if self.slew_limited:
            step = self.slew_rate * dt
            self.tilt += constrain(self.target - self.tilt, -step, step)
        else:
            self.tilt = self.target

    def _steer_target(self, d, mag):                # POD:89-173
        pi = math.pi
        tilt = math.atan2(d[1], d[0])
        margin = max(self.K_END_SWITCH_MARGIN * mag, self.K_STEER_RELEASE)

        if d[0] < 0.0 and abs(d[1]) < margin:
            rear_hi = constrain(pi, self.tilt_min, self.tilt_max)
            rear_lo = constrain(-pi, self.tilt_min, self.tilt_max)
            rev_hi = -math.cos(rear_hi)
            rev_lo = -math.cos(rear_lo)
            if abs(rev_hi - rev_lo) > FLT_EPSILON:
                tilt = rear_hi if rev_hi > rev_lo else rear_lo
            else:
                tilt = rear_hi if self.target >= 0.0 else rear_lo
            if self.realized(d, tilt) <= margin:
                tilt = self.target
        elif tilt < self.tilt_min or tilt > self.tilt_max:
            p_hi = max(0.0, self.realized(d, self.tilt_max))
            p_lo = max(0.0, self.realized(d, self.tilt_min))
            if max(p_hi, p_lo) <= FLT_EPSILON:
                tilt = self.target
            else:
                committed_hi = self.target - self.tilt_min > self.tilt_max - self.target
                if committed_hi:
                    tilt = self.tilt_min if p_lo > p_hi + margin else self.tilt_max
                else:
                    tilt = self.tilt_max if p_hi > p_lo + margin else self.tilt_min

        if abs(tilt - self.target) > pi:
            if self.realized(d, tilt) <= max(0.0, self.realized(d, self.target)) + margin:
                tilt = self.target

        return constrain(tilt, self.tilt_min, self.tilt_max)

    def servo_setpoint(self):                       # POD:175-178
        return -1.0 + 2.0 * (self.tilt - self.tilt_min) / (self.tilt_max - self.tilt_min)

    def set_servo_setpoint(self, sp):               # POD:180-183
        self.tilt = self.tilt_min + (sp + 1.0) * 0.5 * (self.tilt_max - self.tilt_min)

    def held_by_choice(self):                       # POD:195-200
        return (self.can_tilt() and self.mode in (HOLDING, RELEASED)
                and abs(self.tilt - self.target) < self.K_TILT_SETTLED)

    def drive(self, d, out_min, out_max):           # POD:185-193
        ax = (math.cos(self.tilt), math.sin(self.tilt))
        thrust = constrain(d[0] * ax[0] + d[1] * ax[1], max(out_min, 0.0), out_max)
        achieved = (ax[0] * thrust, ax[1] * thrust)
        withheld = (d[0] - achieved[0], d[1] - achieved[1]) if self.held_by_choice() else (0.0, 0.0)
        return thrust, achieved, withheld


def pod_wrench(s, p):
    """EA:155-166: [roll, pitch, yaw, thrust_x, thrust_y, thrust_z]."""
    return [0.5 * (p[1] - s[1]), 0.0, 0.5 * (p[0] - s[0]),
            0.5 * (s[0] + p[0]), 0.0, -(0.5 * (s[1] + p[1]))]


class AirshipAllocator:
    """ActuatorEffectivenessAirship::updateSetpoint() and getUnallocatedControl()
    for an airframe with no control surfaces (both plants here): what the pods
    and the tail deliver, in normalized units, and the unallocated torque the
    allocator publishes, which is all the rate loop's anti-windup reads.

    Limits (CA:521-548): pod motors [0, 1], a CA_R_REV motor [-1, 1], servos
    [-1, 1]. No CA_R*_SLEW / CA_SV*_SLEW is set, so applySlewRateLimit() does not
    run (CA:449-451), and clipActuatorSetpoint() (CA:453) changes nothing the
    model has not already clamped.
    """

    def __init__(self, airframe='vectored'):
        a = dict(AIRFRAMES[airframe]) if isinstance(airframe, str) else dict(airframe)
        self.a = a
        self.independent = a['CA_AIRSHIP_GRP'] > 0                  # EA:52
        self.has_tail = bool(a['CA_AIRSHIP_TAIL'])                   # EA:53
        self.credit = a['CA_AIRSHIP_CS_K']
        self.pods = [AirshipPod(), AirshipPod()]
        for pod in self.pods:                                        # EA:59-62
            pod.set_tilt_range(math.radians(a['CA_AIRSHIP_TLMIN']), math.radians(a['CA_AIRSHIP_TLMAX']))
            pod.set_tilt_slew_rate(math.radians(a['CA_AIRSHIP_TLT_R']))
        self.num_tilt_servos = (2 if self.independent else 1) if self.pods[0].can_tilt() else 0  # EA:93-95
        self.tail_min = -1.0 if (a['CA_R_REV'] & (1 << 2)) else 0.0  # motor 3 is index 2 (CA:525-527)
        self.demand = [0.0] * 6
        self.achieved = [0.0] * 6
        self.held = [0.0] * 6
        self.motors = [0.0, 0.0, 0.0]
        self.servos = [0.0, 0.0]

    def update(self, control_sp, dt, armed=True):
        """control_sp = [roll, pitch, yaw, thrust_x, thrust_y, thrust_z] as the
        controller publishes them (AMI:70-83 for the passthrough axes). dt is
        the allocator's own cycle time (CA:377, clamped 0.2..20 ms), handed to
        the pods by allocateAuxilaryControls() (EAH:85). Returns the delivered
        normalized yaw."""
        demand = list(control_sp)                                    # EA:177-181, no surfaces
        common = (demand[3], -demand[5])                             # EA:183
        diff = (demand[2], demand[0]) if self.independent else (0.0, 0.0)  # EA:184-185
        pod_demand = [(common[0] - diff[0], common[1] - diff[1]),    # EA:186
                      (common[0] + diff[0], common[1] + diff[1])]

        for pod, d in zip(self.pods, pod_demand):                    # EA:190-197
            if armed:
                pod.steer(d, dt)
            else:
                pod.park(dt)

        # writeTiltServos (EA:233-256, called at EA:199)
        if self.num_tilt_servos and self.pods[0].can_tilt():
            for i in range(self.num_tilt_servos):
                sp = constrain(self.pods[i].servo_setpoint(), -1.0, 1.0)
                self.servos[i] = sp
                self.pods[i].set_servo_setpoint(sp)
            if not self.independent:
                self.pods[1].set_servo_setpoint(self.servos[0])

        drives = [pod.drive(d, 0.0, 1.0) for pod, d in zip(self.pods, pod_demand)]  # EA:208-211
        self.motors[0], self.motors[1] = drives[0][0], drives[1][0]
        achieved = pod_wrench(drives[0][1], drives[1][1])            # EA:213

        if self.has_tail:                                            # EA:215-222
            self.motors[2] = constrain(demand[2] - achieved[2], self.tail_min, 1.0)
            achieved[2] += self.motors[2]

        self.demand = demand
        self.achieved = achieved
        self.held = pod_wrench(drives[0][2], drives[1][2])           # EA:229
        return achieved[2]

    def shortfall(self, asked, axis):                                # EA:258-269
        raw = asked - self.achieved[axis]
        held = self.held[axis]
        if raw * held > 0.0:
            return math.copysign(max(abs(raw) - abs(held), 0.0), raw)
        return raw

    @staticmethod
    def saturation_sign(x):                                          # EA:271-283, EAH:135
        return 1.0 if x > 1e-4 else (-1.0 if x < -1e-4 else 0.0)

    def status(self):
        """ControlAllocator::publish_control_allocator_status() (CA:661-691)
        with the airship override (EA:285-319): no torque axis is surface-served
        here, so each publishes saturationSign(shortfall)."""
        u = [self.saturation_sign(self.shortfall(self.demand[ax], ax)) for ax in range(3)]
        return {'unallocated_torque': u,
                'torque_setpoint_achieved': (u[0] ** 2 + u[1] ** 2 + u[2] ** 2) < 1e-6}


# ---------------------------------------------------------------------------
# The chain: gyro -> controller -> allocator -> status, one gz step per call
# ---------------------------------------------------------------------------

class SitlYawChain:
    """One 250 Hz cycle of the SITL chain, in firmware order:
    controller (reads the status the allocator published on an earlier cycle)
    -> allocator (same cycle) -> status publish when >= 5 ms since the last
    (CA:464-472; _last_status_pub starts at 0, CA.hpp:248).

    step() returns (torque_sp_yaw, delivered_yaw): the normalized torque the
    controller asked for and the normalized yaw the pods + tail are commanded
    to produce this cycle (EA's achieved wrench). Turning delivered_yaw into
    N m (54 N m vectored, 4.478 N m Cloudship per unit: gz_model.yaw_authority),
    the gz actuator dynamics and the command-to-physics delay belong to the
    plant model, not here.
    """

    STATUS_MIN_INTERVAL_US = 5000

    def __init__(self, airframe='vectored', params=None, filter_gyro=True, t0_us=10_000_000):
        self.ctrl = AirshipYawController(airframe, params, filter_gyro=filter_gyro)
        self.alloc = AirshipAllocator(airframe)
        self.status_topic = Topic()
        self.t_us = t0_us
        self._alloc_last_run = 0          # CA.hpp:246
        self._last_status_pub = 0         # CA.hpp:248
        self.throttle = -1.0              # sitl_link.Sticks: z = 0 -> throttle -1 -> thrust_x 0
        self.pitch = 0.0
        self.roll = 0.0

    def step(self, gyro_z, stick_yaw, mode='acro', armed=True, sticks_valid=True):
        self.t_us += DT_US
        tq = self.ctrl.step(self.t_us, gyro_z, stick_yaw, mode, armed, sticks_valid, self.status_topic)
        usable = armed and sticks_valid
        # AMI:70-83 passthrough axes, zeroed when not usable (AAC:197-198)
        if usable:
            thrust_x, _, thrust_z = manual_thrust(self.throttle, self.pitch)
            roll_t, pitch_t, _ = manual_torque(self.roll, self.pitch, 0.0)
        else:
            thrust_x = thrust_z = roll_t = pitch_t = 0.0
        control_sp = [roll_t, pitch_t, tq, thrust_x, 0.0, thrust_z]
        dt_alloc = constrain((self.t_us - self._alloc_last_run) / 1e6, 0.0002, 0.02)   # CA:377
        self._alloc_last_run = self.t_us
        delivered = self.alloc.update(control_sp, dt_alloc, armed=armed)
        if self.t_us - self._last_status_pub >= self.STATUS_MIN_INTERVAL_US:            # CA:464
            self.status_topic.publish(self.alloc.status())
            self._last_status_pub = self.t_us
        return tq, delivered


# ---------------------------------------------------------------------------
# Adapters for gate.py's compose(controller, actuation, plant)
# ---------------------------------------------------------------------------

class GateController:
    """AirshipYawController behind gate.compose()'s controller interface:
    update(dt, rate_meas, stick, mode, armed, sticks_valid, unallocated),
    param(), set_param(), integ (None until rate_ctrl_status is first
    published: AAC:214 while closed, AAC:219 once on opening).

    `unallocated` is the yaw element of the allocation compose() ran on the
    previous tick. It becomes a new control_allocator_status only on the ticks
    the allocator would have published (>= 5 ms since the last, CA:464), so
    the controller sees an allocation 1 or 2 ticks old, as in SITL. The yaw
    flag needs torque_setpoint_achieved false (CA:686-688, norm^2 < 1e-6 over
    the three torque axes); only yaw is passed in, so roll and pitch count as
    zero, which is exact for the sign-publishing airframes here (a yaw of
    +-1 fails the gate on its own; a 0 sets no flag either way).

    filter_gyro: apply VehicleAngularVelocity's 40 Hz Butterworth to
    rate_meas. Set False if the plant's measured_rate() already includes it.
    other_params: firmware parameters the gate reads that the controller does
    not (THR_MDL_FAC); closed_loop.build() passes the airframe's own. A name
    nobody gave reads as None, as param_show of an absent parameter does.
    """

    def __init__(self, airframe='vectored', params=None, filter_gyro=True, t0_us=10_000_000, other_params=None):
        self.ctrl = AirshipYawController(airframe, params, filter_gyro=filter_gyro)
        self.status_topic = Topic()
        self.t_us = t0_us
        self._last_status_pub = 0
        self._published = False
        self._first = True
        self._other_params = dict(other_params or {})

    def param(self, name):
        if name in self.ctrl.params:
            return self.ctrl.params[name]
        return self._other_params.get(name)

    def set_param(self, name, value):
        # AAC:228 polls parameter_update at the end of Run() through a 1 s
        # SubscriptionInterval (AAH:125): the change applies from the next
        # cycle, or up to 1 s later if another update was taken in the last
        # second. Modelled as the next cycle.
        if name in self.ctrl.params:
            self.ctrl.set_param(name, value)
        else:
            self._other_params[name] = value

    @property
    def integ(self):
        return self.ctrl.integ if self._published else None

    def update(self, dt, rate_meas, stick, mode, armed, sticks_valid, unallocated):
        prev_alloc_t = self.t_us                      # the allocator ran at the previous tick's time
        if not self._first and prev_alloc_t - self._last_status_pub >= SitlYawChain.STATUS_MIN_INTERVAL_US:
            u = float(unallocated)
            self.status_topic.publish({'unallocated_torque': [0.0, 0.0, u],
                                       'torque_setpoint_achieved': u * u < 1e-6})
            self._last_status_pub = prev_alloc_t
        self._first = False
        self.t_us += int(round(dt * 1e6))
        was = self.ctrl._was_active
        tq = self.ctrl.step(self.t_us, rate_meas, stick, mode, armed, sticks_valid, self.status_topic)
        if self.ctrl.loop_active or was:
            self._published = True
        return tq


class AllocatorActuation:
    """AirshipAllocator behind gate.compose()'s actuation interface:
    step(dt, torque_sp) -> (yaw moment N m FRD, unallocated yaw).

    The moment is the allocator's commanded normalized yaw times the full
    command authority (54.0 N m vectored, 4.478 N m Cloudship:
    gz_model.yaw_authority), applied at once. That is the allocator's own
    belief, i.e. IDEAL actuators: the gz tilt joint's tracking, the motor
    models, THR_MDL_FAC's curve and the command delay are the plant part's.
    Throttle and pitch are the gate's (sitl_link.Sticks: z = 0, x = 0):
    thrust 0, so the pod demands are the yaw couple alone.
    """

    def __init__(self, airframe, authority_nm, armed=lambda: True):
        self.alloc = AirshipAllocator(airframe)
        self.authority = authority_nm
        self.armed = armed

    def step(self, dt, torque_sp):
        delivered = self.alloc.update([0.0, -0.0, torque_sp, 0.0, 0.0, 0.0], dt, armed=self.armed())
        return delivered * self.authority, self.alloc.status()['unallocated_torque'][2]
