#!/usr/bin/env python3
"""Yaw actuation of the independently vectored gz airship (2520 family):
normalized yaw torque u(t) -> physical yaw moment N(t) on the hull [N m, FRD].

    python3 vectored_actuation.py            # sources check + step responses
    python3 vectored_actuation.py --csv      # also writes steps_*.csv here

Importable for a closed-loop model:

    from vectored_actuation import VectoredActuation, load_params
    act = VectoredActuation(load_params())      # armed, pods parked at 0 deg
    row = act.step(u)                           # one 4 ms tick
    row['N']            yaw moment applied to the hull during this tick [N m]
    row['sat_seen']     yaw saturation sign the rate controller reads on the
                        NEXT tick (+1/-1/0), exactly as airship_att_control
                        derives saturation_positive/negative from it

Everything below is read from the sources; file:line citations are relative
to ~/dev/PX4-Autopilot unless marked (bench) = this repository,
(gz-sim) = gz-sim 8.15.0 (installed: libgz-sim8 8.15.0-1~noble) or
(gz-math) = gz-math 7.7.0 (installed libgz-math7 7.7.0). The gz sources were
fetched at those tags into ./gz_src/ next to this file.

Signal chain, one PX4 control cycle = one gz physics step = 4 ms:

  u = vehicle_torque_setpoint.xyz[2]
    -> ControlAllocator (runs on the torque setpoint, ControlAllocator.cpp:387-442)
       ActuatorEffectivenessAirship::updateSetpoint (Independent, no tail,
       no surfaces on 2520): pod demand (forward, up) = common -/+ (u, roll),
       common = (thrust_x, -thrust_z)                    ActuatorEffectivenessAirship.cpp:181-184
       AirshipPod::steer: steer band, end choice, 120 deg/s tilt slew
                                                          AirshipPod.cpp:49-70, 79-87, 89-173
       tilt servo written, clamped and read back          ActuatorEffectivenessAirship.cpp:231-252
       AirshipPod::drive: motor = clamp(demand . axis(tilt), 0, 1)
                                                          AirshipPod.cpp:185-193
    -> mixer: THR_MDL_FAC 1 -> sqrt, [0,1]->[-1,1], [-1,1]->[SIM_GZ_EC_MIN, MAX]
                                                          FunctionMotors.hpp:83-117, mixer_module.cpp:565
       servo [-1,1] -> [0,1000] -> angle over SIM_GZ_SV_MINA..MAXA
                                                          mixer_module.cpp:565, GZMixingInterfaceServo.cpp:144-148
    -> gz: JointPositionController PD on the tilt joint   (gz-sim) JointPositionController.cc:434-435, 472
           MulticopterMotorModel first-order rotor filter, thrust = kF w^2
                                                          (gz-sim) MulticopterMotorModel.cc:537-539, 566-591, 683-690
    -> N = 1.5 m x (T_port cos th_port - T_stbd cos th_stbd)
           + reaction torque (momentConstant) + rotor angular-momentum term

The gate (sitl_check.py) holds throttle 0, pitch 0 and roll 0: (bench)
sitl_link.py:276-279 initialises them to 0 and :297-298 sends them; PX4 reads
z = 0 as throttle -1 (src/modules/mavlink/mavlink_receiver.cpp:2379), which
is thrust_x = (throttle + 1) / 2 = 0 (airship_manual_input.hpp:70); thrust_z
= pitch stick = 0 (same line) and roll torque = roll stick = 0 (:80). So the
pods get pure differential demand: starboard (-u, 0), port (+u, 0). There is
no forward thrust to lean on: yaw is made by pointing one pod forward and
the other straight back (tilt +-180), each at |u| of full thrust.
"""
import argparse
import csv
import math
import os
import re
import struct
import sys
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = os.path.dirname(HERE)
sys.path.insert(0, BENCH)
import gz_model  # noqa: E402  (bench) pure SDF reader, no simulator

PX4 = gz_model.ROOT
FLT_EPSILON = 2.0 ** -23        # FLT_EPSILON (float.h), exact


def to_f32(x):
    """Round to IEEE single, as the firmware's float arithmetic does."""
    return struct.unpack('f', struct.pack('f', x))[0]


M_PI_F = to_f32(math.pi)


def radians_f(deg):
    # src/lib/mathlib/math/Limits.hpp:139-142: degrees * (T(MATH_PI) / T(180)) in float
    return to_f32(to_f32(deg) * to_f32(M_PI_F / to_f32(180.0)))


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


# ---------------------------------------------------------------------------
# Parameters, read from the files and asserted against the cited values
# ---------------------------------------------------------------------------

AIRFRAME = 'ROMFS/px4fmu_common/init.d-posix/airframes/2520_gz_airship_vectored_independent'
AIRFRAME_PATH = gz_model.airframe('airship_vectored_independent')[1]     # that file, found over the homes (overlay, tree)
WORLD = 'Tools/simulation/gz/worlds/lta.sdf'

# Values the citations promise. load_params() re-reads every one from the file
# and refuses to run on a mismatch, so a changed airframe or model cannot be
# modelled silently with stale numbers.
CITED = {
    # airframe 2520 (param set-default, line)
    'SIM_GZ_EC_MIN1': (0.0, 32), 'SIM_GZ_EC_MIN2': (0.0, 33),
    'SIM_GZ_EC_MAX1': (300.0, 34), 'SIM_GZ_EC_MAX2': (300.0, 35),
    'THR_MDL_FAC': (1.0, 39),
    'SIM_GZ_SV_MINA1': (-180.0, 46), 'SIM_GZ_SV_MAXA1': (180.0, 47),
    'SIM_GZ_SV_MINA2': (-180.0, 48), 'SIM_GZ_SV_MAXA2': (180.0, 49),
    'CA_AIRFRAME': (16.0, 53), 'CA_AIRSHIP_GRP': (1.0, 54),
    'CA_AIRSHIP_TLMIN': (-180.0, 55), 'CA_AIRSHIP_TLMAX': (180.0, 56),
    'CA_AIRSHIP_TLT_R': (120.0, 57),
}
# Parameters 2520 leaves at their defaults (src/modules/control_allocator/module.yaml):
#   CA_AIRSHIP_TAIL 0 (:654), CA_SV_CS_COUNT 0 (:268), CA_R_REV 0 (:73),
#   CA_R<i>_SLEW 0 (:99), CA_SV<i>_SLEW 0 (:117) -> no tail, no surfaces, pods
#   non-reversible (actuator_min 0, ControlAllocator.cpp:531), no generic slew.
# gz bridge servo output range: SIM_GZ_SV_MIN default 0, MAX default 1000
#   (src/modules/simulation/gz_bridge/module.yaml:28-29); there is no
#   SIM_GZ_SV_CENT param (not in build/px4_sitl_default/parameters.xml), so
#   the mapping is the linear interpolate at mixer_module.cpp:565.


def airframe_params(path):
    """{name: (value, line)} of the `param set-default` lines written in
    this one file (not the ones it sources: the CITED lines are its own)."""
    return {name: (float(value), line) for kind, name, value, where, line in gz_model.airframe_settings(path)
            if kind == 'default' and where == path}


def _text(elem, path, default=None):
    found = elem.find(path)
    return default if found is None else found.text.strip()


def plant_params(model_name):
    """Pod geometry, tilt servo and motor constants from the merged gz model.

    airship_vectored_independent/model.sdf (rigid) lines:
      pod poses y -1.5 / +1.5 :19 / :92, prop poses :55 / :128,
      tilt joint axis (0 -1 0) :44 / :117, limits +-3.141593 :46-47 / :119-120,
      joint damping 0.01 :50 / :123, pod inertia 0.001 :21-26 / :94-99,
      prop inertia 0.001 :57-62 / :130-135,
      JointPositionController p 2.0 i 0.0 d 0.1 :164-179 (no cmd_max: default 1000),
      MulticopterMotorModel :189-222: ccw/cw :192/:210, timeConstantUp 0.0125
      :194/:211, timeConstantDown 0.025 :195/:212, maxRotVelocity 300 :196/:213,
      motorConstant 2e-4 :197/:214, momentConstant 0.016 :198/:215,
      rotorVelocitySlowdownSim 10 :203/:220.
    The complete plant (…_am_fin_drag) reaches its pods through
    airship_vectored_independent_am, whose pod/servo/motor block is textually
    identical from <link name="pod_starboard"> to the end (checked with diff);
    load_params() re-verifies that the two read the same.
    """
    model = gz_model.merged(model_name)
    family = gz_model.family(model_name)
    _, com = gz_model.centre_of_mass(family)
    plugins = model.findall('plugin')
    pods = {}

    for side in ('starboard', 'port'):
        prop = gz_model.named(model, 'link', 'prop_' + side)
        pod = gz_model.named(model, 'link', 'pod_' + side)
        joint = gz_model.named(model, 'joint', 'tilt_%s_joint' % side)
        servo = [p for p in plugins if 'joint-position-controller' in p.get('filename', '')
                 and _text(p, 'joint_name') == joint.get('name')]
        assert len(servo) == 1, side
        servo = servo[0]
        motor = gz_model.motor_plugin(model, 'prop_' + side)
        axis = [float(v) for v in _text(joint, 'axis/xyz').split()]
        pods[side] = {
            'y_lever': gz_model.pose(prop, 1) - com[1],     # FLU y about the family CoM
            'pod_y': gz_model.pose(pod, 1),
            'axis': axis,
            'lower': float(_text(joint, 'axis/limit/lower')),
            'upper': float(_text(joint, 'axis/limit/upper')),
            'damping': float(_text(joint, 'axis/dynamics/damping', '0')),
            # rotational inertia the tilt joint turns: pod iyy + prop transverse
            # inertia (prop link pitched 90 deg, all diagonal terms equal)
            'J': float(_text(pod, 'inertial/inertia/iyy')) + float(_text(prop, 'inertial/inertia/ixx')),
            'I_spin': float(_text(prop, 'inertial/inertia/izz')),
            'p': float(_text(servo, 'p_gain', '1')),        # defaults: JointPositionController.cc:149-156
            'i': float(_text(servo, 'i_gain', '0.1')),
            'd': float(_text(servo, 'd_gain', '0.01')),
            'cmd_max': float(_text(servo, 'cmd_max', '1000')),
            'turning': _text(motor, 'turningDirection'),
            'tau_up': float(_text(motor, 'timeConstantUp')),
            'tau_down': float(_text(motor, 'timeConstantDown')),
            'max_rot': float(_text(motor, 'maxRotVelocity')),
            'kF': float(_text(motor, 'motorConstant')),
            'kM': float(_text(motor, 'momentConstant')),
            'slowdown': float(_text(motor, 'rotorVelocitySlowdownSim', '10')),
            'motor_number': int(_text(motor, 'motorNumber')),
            'motor_type': _text(motor, 'motorType', 'velocity'),
        }

    return {'pods': pods, 'com': com, 'model': model_name}


def world_step(path):
    with open(path) as handle:
        for number, line in enumerate(handle, 1):
            match = re.search(r'<max_step_size>([0-9.eE+-]+)</max_step_size>', line)
            if match:
                return float(match.group(1)), number
    raise LookupError(path)


def load_params(model_name='airship_vectored_independent', check=True, airframe=None):
    """The actuation's parameters: the allocator's from the posix airframe
    (AIRFRAME, 2520's, whose cited values are re-read, or `airframe`, another
    family's bare airframe path relative to the PX4 tree, taken as it is) and
    the plant's from the gz model; check=True also holds the plant to the
    10.97 m anchor's cited constants."""
    af = airframe_params(AIRFRAME_PATH if airframe is None else os.path.join(PX4, airframe))
    problems = []

    for name, (value, line) in (CITED if airframe is None else {}).items():
        got = af.get(name)
        if got is None or abs(got[0] - value) > 1e-9 or got[1] != line:
            problems.append('%s: cited %g at :%d, file has %s' % (name, value, line, got))

    plant = plant_params(model_name)
    step, step_line = world_step(os.path.join(PX4, WORLD))

    if check:
        expect = {'y_lever': None, 'lower': -3.141593, 'upper': 3.141593, 'damping': 0.01,
                  'J': 0.002, 'I_spin': 0.001, 'p': 2.0, 'i': 0.0, 'd': 0.1, 'cmd_max': 1000.0,
                  'tau_up': 0.0125, 'tau_down': 0.025, 'max_rot': 300.0, 'kF': 2e-4,
                  'kM': 0.016, 'slowdown': 10.0, 'motor_type': 'velocity'}
        for side, sign, turning, number in (('starboard', -1, 'ccw', 0), ('port', 1, 'cw', 1)):
            pod = plant['pods'][side]
            if abs(pod['y_lever'] - sign * 1.5) > 1e-9:
                problems.append('%s lever %.6f, cited %+.1f' % (side, pod['y_lever'], sign * 1.5))
            if pod['axis'] != [0.0, -1.0, 0.0]:
                problems.append('%s tilt axis %s' % (side, pod['axis']))
            if pod['turning'] != turning or pod['motor_number'] != number:
                problems.append('%s motor %s #%d' % (side, pod['turning'], pod['motor_number']))
            for key, value in expect.items():
                if value is None:
                    continue
                if isinstance(value, str):
                    ok = pod[key] == value
                else:
                    ok = abs(pod[key] - value) <= 1e-9 * max(1.0, abs(value))
                if not ok:
                    problems.append('%s %s = %s, cited %s' % (side, key, pod[key], value))
        if abs(step - 0.004) > 1e-12 or step_line != 8:
            problems.append('world step %g at :%d, cited 0.004 at :8' % (step, step_line))
        if model_name != 'airship_vectored_independent_am_fin_drag':
            other = plant_params('airship_vectored_independent_am_fin_drag')['pods']
            for side in ('starboard', 'port'):
                for key, value in plant['pods'][side].items():
                    if key == 'y_lever':
                        continue
                    if other[side][key] != value:
                        problems.append('complete plant differs: %s %s %s vs %s'
                                        % (side, key, other[side][key], value))

    if problems:
        raise SystemExit('source mismatch, refusing to model stale numbers:\n  ' + '\n  '.join(problems))

    def a(name):
        return af[name][0]

    return {
        'dt': step,                        # PX4 cycle == gz step (IMU 250 Hz: airship_base/model.sdf:217)
        'tilt_min': radians_f(a('CA_AIRSHIP_TLMIN')),   # ActuatorEffectivenessAirship.cpp:56
        'tilt_max': radians_f(a('CA_AIRSHIP_TLMAX')),   # :57
        'slew': radians_f(a('CA_AIRSHIP_TLT_R')),       # :58 -> AirshipPod::setTiltSlewRate
        'thr_mdl_fac': a('THR_MDL_FAC'),
        'ec_min': [a('SIM_GZ_EC_MIN1'), a('SIM_GZ_EC_MIN2')],
        'ec_max': [a('SIM_GZ_EC_MAX1'), a('SIM_GZ_EC_MAX2')],
        'sv_mina': [a('SIM_GZ_SV_MINA1'), a('SIM_GZ_SV_MINA2')],
        'sv_maxa': [a('SIM_GZ_SV_MAXA1'), a('SIM_GZ_SV_MAXA2')],
        'sv_min': 0.0, 'sv_max': 1000.0,
        'plant': plant,
        # PX4 index 0 = starboard (MotorIndex STARBOARD, ActuatorEffectivenessAirship.hpp:62)
        # -> Motor1/Servo1 (2520:30, :44) -> ESC 0 / servo_0 (model.sdf:167, :200)
        'sides': ('starboard', 'port'),
    }


# ---------------------------------------------------------------------------
# PX4: AirshipPod and the Independent branch of ActuatorEffectivenessAirship
# ---------------------------------------------------------------------------

PARKED, STEERING, HOLDING, RELEASED = 'parked', 'steering', 'holding', 'released'


class Pod:
    """Line-for-line port of src/modules/control_allocator/VehicleActuatorEffectiveness/AirshipPod.cpp."""
    ENGAGE = 0.02        # AirshipPod.hpp:54 kSteerEngage
    RELEASE = 0.01       # AirshipPod.hpp:55 kSteerRelease
    MARGIN = 0.05        # AirshipPod.hpp:60 kEndSwitchMargin
    MIN_SPAN = 1e-3      # AirshipPod.hpp:112
    SETTLED = 1e-3       # AirshipPod.hpp:113

    def __init__(self, tilt_min, tilt_max, slew_rate):
        self.tilt = 0.0                  # SlewRate _value{} (SlewRate.hpp:91)
        self.target = 0.0
        self.mode = PARKED
        self.tmin, self.tmax = tilt_min, max(tilt_max, tilt_min)          # AirshipPod.cpp:43-44
        self.tilt = clamp(self.tilt, self.tmin, self.tmax)                # :45
        self.target = clamp(self.target, self.tmin, self.tmax)            # :46
        self.slew = slew_rate
        self.slew_limited = slew_rate > 0.0                               # AirshipPod.hpp:66

    def can_tilt(self):
        return self.tmax - self.tmin > self.MIN_SPAN

    @staticmethod
    def realized(d, tilt):                                                # AirshipPod.hpp:128
        return d[0] * math.cos(tilt) + d[1] * math.sin(tilt)

    def steer(self, d, dt):                                               # AirshipPod.cpp:49-70
        magnitude = math.hypot(d[0], d[1])
        engaged = self.mode in (STEERING, HOLDING)

        if magnitude > self.ENGAGE:
            self.mode = STEERING
            self.target = self.steer_target(d, magnitude)
        elif engaged and magnitude > self.RELEASE:
            self.mode = HOLDING
        else:
            self.mode = RELEASED
            self.target = self.tilt

        self.slew_to_target(dt)

    def park(self, dt):                                                   # :72-77
        self.mode = PARKED
        self.target = clamp(0.0, self.tmin, self.tmax)
        self.slew_to_target(dt)

    def slew_to_target(self, dt):                                         # :79-87, SlewRate.hpp:79-87
        if self.slew_limited:
            step = self.slew * dt
            self.tilt += clamp(self.target - self.tilt, -step, step)
        else:
            self.tilt = self.target

    def steer_target(self, d, magnitude):                                 # :89-173
        tilt = math.atan2(d[1], d[0])
        margin = max(self.MARGIN * magnitude, self.RELEASE)

        if d[0] < 0.0 and abs(d[1]) < margin:
            rear_hi = clamp(M_PI_F, self.tmin, self.tmax)
            rear_lo = clamp(-M_PI_F, self.tmin, self.tmax)
            reverse_hi, reverse_lo = -math.cos(rear_hi), -math.cos(rear_lo)

            if abs(reverse_hi - reverse_lo) > FLT_EPSILON:
                tilt = rear_hi if reverse_hi > reverse_lo else rear_lo
            else:
                tilt = rear_hi if self.target >= 0.0 else rear_lo

            if self.realized(d, tilt) <= margin:
                tilt = self.target

        elif tilt < self.tmin or tilt > self.tmax:
            p_hi = max(0.0, self.realized(d, self.tmax))
            p_lo = max(0.0, self.realized(d, self.tmin))

            if max(p_hi, p_lo) <= FLT_EPSILON:
                tilt = self.target
            else:
                committed_hi = self.target - self.tmin > self.tmax - self.target
                if committed_hi:
                    tilt = self.tmin if p_lo > p_hi + margin else self.tmax
                else:
                    tilt = self.tmax if p_hi > p_lo + margin else self.tmin

        if abs(tilt - self.target) > M_PI_F:
            if self.realized(d, tilt) <= max(0.0, self.realized(d, self.target)) + margin:
                tilt = self.target

        return clamp(tilt, self.tmin, self.tmax)

    def servo_setpoint(self):                                             # :175-178
        return -1.0 + 2.0 * (self.tilt - self.tmin) / (self.tmax - self.tmin)

    def set_servo_setpoint(self, servo_sp):                               # :180-183
        self.tilt = self.tmin + (servo_sp + 1.0) * 0.5 * (self.tmax - self.tmin)

    def held_by_choice(self):                                             # :195-200
        return (self.can_tilt() and self.mode in (HOLDING, RELEASED)
                and abs(self.tilt - self.target) < self.SETTLED)

    def drive(self, d, out_min, out_max):                                 # :185-193
        axis = (math.cos(self.tilt), math.sin(self.tilt))
        thrust = clamp(d[0] * axis[0] + d[1] * axis[1], max(out_min, 0.0), out_max)
        achieved = (axis[0] * thrust, axis[1] * thrust)
        withheld = (d[0] - achieved[0], d[1] - achieved[1]) if self.held_by_choice() else (0.0, 0.0)
        return thrust, achieved, withheld


def pod_wrench(starboard, port):
    """ActuatorEffectivenessAirship.cpp:154-164 -> (roll, pitch, yaw, Tx, Ty, Tz)."""
    return (0.5 * (port[1] - starboard[1]), 0.0, 0.5 * (port[0] - starboard[0]),
            0.5 * (starboard[0] + port[0]), 0.0, -(0.5 * (starboard[1] + port[1])))


class Allocator:
    """ActuatorEffectivenessAirship::updateSetpoint / getUnallocatedControl for
    CA_AIRSHIP_GRP 1, CA_AIRSHIP_TAIL 0, CA_SV_CS_COUNT 0 (the 2520 family)."""
    SHORTFALL_DEADBAND = 1e-4   # ActuatorEffectivenessAirship.hpp:131

    def __init__(self, p):
        self.pods = [Pod(p['tilt_min'], p['tilt_max'], p['slew']) for _ in range(2)]
        self.unallocated = [0.0, 0.0, 0.0]

    def update(self, torque, thrust, dt, armed=True):
        demand = (torque[0], torque[1], torque[2], thrust[0], thrust[1], thrust[2])
        # no surfaces: surface torque 0, demand = control_sp (:171-179)
        common = (demand[3], -demand[5])                                  # :181
        diff = (demand[2], demand[0])                                     # :182-183 (Independent)
        pod_demand = ((common[0] - diff[0], common[1] - diff[1]),
                      (common[0] + diff[0], common[1] + diff[1]))         # :184

        for pod, d in zip(self.pods, pod_demand):                         # :188-195
            if armed:
                pod.steer(d, dt)
            else:
                pod.park(dt)

        servo = []
        for pod in self.pods:                                             # :243-247
            s = clamp(pod.servo_setpoint(), -1.0, 1.0)
            pod.set_servo_setpoint(s)
            servo.append(s)

        # actuator_min 0 (CA_R_REV 0, ControlAllocator.cpp:531), max 1 (:551)
        drives = [pod.drive(d, 0.0, 1.0) for pod, d in zip(self.pods, pod_demand)]   # :206-209
        achieved = pod_wrench(drives[0][1], drives[1][1])                 # :211
        held = pod_wrench(drives[0][2], drives[1][2])                     # :227

        def shortfall(asked, axis):                                       # :254-265
            raw = asked - achieved[axis]
            if raw * held[axis] > 0.0:
                return math.copysign(max(abs(raw) - abs(held[axis]), 0.0), raw)
            return raw

        def sign(x):                                                      # :267-279
            return 1.0 if x > self.SHORTFALL_DEADBAND else -1.0 if x < -self.SHORTFALL_DEADBAND else 0.0

        # no surface serves any axis -> every torque axis publishes a sign (:297-303)
        self.unallocated = [sign(shortfall(demand[a], a)) for a in range(3)]
        self.achieved_yaw = achieved[2]
        return servo, [d[0] for d in drives]


def esc_output(c, fac, out_min, out_max):
    """Motor command [0,1] -> rotor speed command [rad/s] on the gz ESC topic."""
    v = c
    if fac > FLT_EPSILON and fac <= 1.0:                                  # FunctionMotors.hpp:83-107
        a, b = fac, 1.0 - fac
        tmp1, tmp2 = b / (2.0 * a), b * b / (4.0 * a * a)
        if c > FLT_EPSILON:
            v = -tmp1 + math.sqrt(tmp2 + c / a)
        elif c < -FLT_EPSILON:
            v = tmp1 - math.sqrt(tmp2 - c / a)
    if v < -FLT_EPSILON:                                                  # :110-112 non-reversible -> NaN
        return out_min                                                    # NaN -> disarmed value 0 (mixer_module.cpp:543-545)
    v = v * 2.0 - 1.0                                                     # :116
    return out_min + (clamp(v, -1.0, 1.0) + 1.0) * 0.5 * (out_max - out_min)   # mixer_module.cpp:565


def servo_angle(s, mina_deg, maxa_deg, sv_min, sv_max):
    """Servo [-1,1] -> joint position command [rad] (GZMixingInterfaceServo.cpp:144-145)."""
    out = sv_min + (clamp(s, -1.0, 1.0) + 1.0) * 0.5 * (sv_max - sv_min)   # mixer_module.cpp:565
    amin = radians_f(mina_deg)                                            # GZMixingInterfaceServo.cpp:103
    arange = radians_f(maxa_deg) - amin                                   # :69, :123
    return amin + arange * (out - sv_min) / (sv_max - sv_min)


# ---------------------------------------------------------------------------
# gz: tilt joint under JointPositionController, rotor under MulticopterMotorModel
# ---------------------------------------------------------------------------

class GzTiltJoint:
    """JointPositionController (gz-sim 8.15.0) PD force on a revolute joint of
    inertia J with viscous damping c, integrated as DART does it (semi-implicit
    Euler, damping implicit) and stopped hard at the SDF limits.

    force = cmdOffset - p e - iErr - d (e - e_last)/dt, e = pos - cmd
      (gz-sim) JointPositionController.cc:434-435, 472; (gz-math) PID.cc:164-165, 184-205.
    """

    def __init__(self, pod, dt):
        self.p, self.i, self.d = pod['p'], pod['i'], pod['d']
        self.cmd_max = pod['cmd_max']
        self.J, self.c = pod['J'], pod['damping']
        self.lower, self.upper = pod['lower'], pod['upper']
        self.dt = dt
        self.theta = 0.0
        self.omega = 0.0
        self.err_last = 0.0
        self.i_err = 0.0
        self.cmd = 0.0                   # no <initial_position>: jointPosCmd starts at 0

    def pre_update(self, cmd):
        if cmd is not None:
            self.cmd = cmd
        err = self.theta - self.cmd
        rate = (err - self.err_last) / self.dt
        self.err_last = err
        self.i_err = clamp(self.i_err + self.i * self.dt * err, -1.0, 1.0)   # i_max/i_min defaults
        self.tau = clamp(-self.p * err - self.i_err - self.d * rate, -self.cmd_max, self.cmd_max)

    def physics(self):
        self.omega = (self.omega + self.dt * self.tau / self.J) / (1.0 + self.dt * self.c / self.J)
        self.theta += self.dt * self.omega
        if self.theta > self.upper:
            self.theta, self.omega = self.upper, min(self.omega, 0.0)
        elif self.theta < self.lower:
            self.theta, self.omega = self.lower, max(self.omega, 0.0)


class GzRotor:
    """MulticopterMotorModel (gz-sim 8.15.0), motorType velocity.

    PreUpdate k: ref = min(cmd, maxRotVelocity) (.cc:537-539); thrust from the
    joint velocity the physics reached in step k-1 (.cc:566-581); then the
    first-order filter (.cc:76-93, tau_up when rising, tau_down when falling)
    sets the joint velocity command td * w / slowdown (.cc:683-690), which the
    velocity-commanded joint reaches in step k (assumed exact).
    """

    def __init__(self, pod, dt):
        self.kF, self.kM = pod['kF'], pod['kM']
        self.max_rot, self.slowdown = pod['max_rot'], pod['slowdown']
        self.tau_up, self.tau_down = pod['tau_up'], pod['tau_down']
        self.td = 1 if pod['turning'] == 'ccw' else -1                    # .cc:111-112, 307-310
        self.dt = dt
        self.ref = 0.0
        self.filtered = 0.0             # previousState = refMotorInput 0 (.cc:185, 370-373)
        self.joint_vel = 0.0            # signed prop joint velocity [rad/s], = td * w / slowdown
        self.joint_vel_cmd = 0.0

    def pre_update(self, cmd):
        if cmd is not None:
            self.ref = min(cmd, self.max_rot)
        real = self.joint_vel * self.slowdown
        sign = (real > 0) - (real < 0)
        self.thrust = self.td * sign * real * real * self.kF             # >= 0 by construction
        tau = self.tau_up if self.ref > self.filtered else self.tau_down
        alpha = math.exp(-self.dt / tau)
        self.filtered = alpha * self.filtered + (1.0 - alpha) * self.ref
        self.joint_vel_cmd = self.td * self.filtered / self.slowdown

    def physics(self):
        self.joint_vel = self.joint_vel_cmd


# ---------------------------------------------------------------------------
# The chain
# ---------------------------------------------------------------------------

class VectoredActuation:
    """u -> N, one 4 ms tick per step().

    transport_ticks: ticks between the allocator computing a command and gz
    applying it. 0 (default) = the command computed from the IMU sample that
    closed step k-1 acts in step k: gz publishes the IMU in PostUpdate and
    paces steps to real time (lta.sdf:8-10, 4 ms at RTF 1), and PX4's
    sensor->controller->allocator->bridge path takes well under one step. Use
    1 for a bridge that misses the step.
    """

    def __init__(self, p, transport_ticks=0, armed=True):
        self.p = p
        self.dt = p['dt']
        self.alloc = Allocator(p)
        pods = [p['plant']['pods'][s] for s in p['sides']]
        self.joints = [GzTiltJoint(pod, self.dt) for pod in pods]
        self.rotors = [GzRotor(pod, self.dt) for pod in pods]
        self.lever = [pod['y_lever'] for pod in pods]        # FLU y: starboard -1.5, port +1.5
        self.I_spin = [pod['I_spin'] for pod in pods]
        self.queue = deque([(None, None)] * transport_ticks)
        self.armed = armed
        self.tick = 0
        self.last_status_tick = None
        self.published = [0.0, 0.0, 0.0]

    def _Hz(self):
        # FLU z angular momentum of the two spinning props: I * signed joint rate * sin(tilt)
        return sum(I * r.joint_vel * math.sin(j.theta) for I, r, j in zip(self.I_spin, self.rotors, self.joints))

    def step(self, u, thrust=(0.0, 0.0, 0.0), roll=0.0):
        p = self.p
        # PX4 (airship_manual_input.hpp:80: pitch torque = -pitch stick = -0.0 in the gate)
        servo, motor = self.alloc.update((roll, -0.0, u), thrust, self.dt, self.armed)
        angle_cmd = [servo_angle(s, p['sv_mina'][i], p['sv_maxa'][i], p['sv_min'], p['sv_max'])
                     for i, s in enumerate(servo)]
        omega_cmd = [esc_output(c, p['thr_mdl_fac'], p['ec_min'][i], p['ec_max'][i])
                     for i, c in enumerate(motor)]

        # allocator status every >= 5 ms (ControlAllocator.cpp:464): every 2nd 4 ms tick;
        # the controller reads it at the start of its next cycle
        # (airship_att_control_main.cpp:235 before :238)
        if self.last_status_tick is None or (self.tick - self.last_status_tick) * self.dt >= 0.005 - 1e-9:
            self.published = list(self.alloc.unallocated)
            self.last_status_tick = self.tick
        yaw = self.published[2]
        # airship_att_control_main.cpp:181-190: achieved iff |unallocated|^2 < 1e-6
        # (ControlAllocator.cpp:686-688); yaw is a sign here, so the gate reduces to its sign
        achieved = sum(x * x for x in self.published) < 1e-6
        sat_seen = 0.0 if achieved else (1.0 if yaw > FLT_EPSILON else -1.0 if yaw < -FLT_EPSILON else 0.0)

        self.queue.append((angle_cmd, omega_cmd))
        vis_angle, vis_omega = self.queue.popleft()

        # gz step: forces from the pose and rotor state at the start of the step
        theta = [j.theta for j in self.joints]
        H0 = self._Hz()
        for i in range(2):
            self.joints[i].pre_update(None if vis_angle is None else vis_angle[i])
            self.rotors[i].pre_update(None if vis_omega is None else vis_omega[i])
        T = [r.thrust for r in self.rotors]

        # lever moment, FRD yaw = -(FLU z) = sum y_FLU * T cos(theta)
        N_lever = sum(y * t * math.cos(th) for y, t, th in zip(self.lever, T, theta))
        # rotor reaction: (0,0,-td T kM) along the thrust axis (cos th, 0, sin th) FLU
        # (MulticopterMotorModel.cc:655-662) -> FRD yaw = +td T kM sin th
        N_react = sum(r.td * t * r.kM * math.sin(th) for r, t, th in zip(self.rotors, T, theta))
        # up force and FRD roll, for information
        F_up = sum(t * math.sin(th) for t, th in zip(T, theta))
        roll_m = sum(y * t * math.sin(th) for y, t, th in zip(self.lever, T, theta))

        for i in range(2):
            self.joints[i].physics()
            self.rotors[i].physics()
        # hull takes -dH/dt of the props (internal torques); FRD yaw = +dHz_FLU/dt
        N_gyro = (self._Hz() - H0) / self.dt

        row = {
            't': self.tick * self.dt, 'u': u,
            'N': N_lever + N_react + N_gyro, 'N_lever': N_lever, 'N_react': N_react, 'N_gyro': N_gyro,
            'yaw_alloc': self.alloc.achieved_yaw,          # what the allocator believes it delivered [normalized]
            'tilt_px4_s': math.degrees(self.alloc.pods[0].tilt), 'tilt_px4_p': math.degrees(self.alloc.pods[1].tilt),
            'tilt_gz_s': math.degrees(theta[0]), 'tilt_gz_p': math.degrees(theta[1]),
            'mode_s': self.alloc.pods[0].mode, 'mode_p': self.alloc.pods[1].mode,
            'c_s': motor[0], 'c_p': motor[1], 'T_s': T[0], 'T_p': T[1],
            'F_up': F_up, 'roll': roll_m,
            'unalloc_yaw': self.alloc.unallocated[2], 'sat_seen': sat_seen,
        }
        self.tick += 1
        return row


# ---------------------------------------------------------------------------
# Runs
# ---------------------------------------------------------------------------

def run(act, u_of_t, seconds):
    rows = []
    t0 = act.tick * act.dt
    for _ in range(int(round(seconds / act.dt))):
        t = act.tick * act.dt - t0
        rows.append(act.step(u_of_t(t)))
        rows[-1]['t'] = t
    return rows


def prepared(p, u_pre=None, pre_s=3.0, rest_s=0.0, transport_ticks=0):
    """An armed actuation; optionally held at u_pre for pre_s, then u = 0 for rest_s."""
    act = VectoredActuation(p, transport_ticks)
    run(act, lambda t: 0.0, 0.2)                 # armed at rest: pods Released at 0 deg
    if u_pre is not None:
        run(act, lambda t: u_pre, pre_s)
        if rest_s > 0:
            run(act, lambda t: 0.0, rest_s)
    return act


def first_time(rows, predicate):
    for row in rows:
        if predicate(row):
            return row['t']
    return float('nan')


def summarize(rows, start_N):
    """final value; first times N reaches 10/50/90/99 % of the final value
    (of the final value itself, not of the swing, so a reversal's times are
    on the positive side); impulse-equivalent pure delay D: an ideal switch
    from start_N to final at t = D loses the same impulse."""
    final = sum(r['N'] for r in rows[-100:]) / 100.0
    frac = lambda f: first_time(rows, lambda r: r['N'] >= f * final)
    dt = rows[1]['t'] - rows[0]['t']
    dead = sum((final - r['N']) for r in rows) * dt / (final - start_N)
    return final, frac(0.1), frac(0.5), frac(0.9), frac(0.99), dead


PRINT_TIMES = [0.0, 0.004, 0.008, 0.012, 0.016, 0.02, 0.03, 0.05, 0.1, 0.25, 0.5, 0.7, 0.74, 0.76, 0.78,
               0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.48, 1.5, 1.52, 1.54, 1.56, 1.6, 1.8, 2.0, 3.0]


def at(rows, t):
    return min(rows, key=lambda r: abs(r['t'] - t))


def print_table(title, runs):
    print('\n' + title)
    header = '   t [s] ' + ''.join('| u=%-4g N [N m]  N/54u  tilt_s tilt_p  ' % u for u, _ in runs)
    print(header)
    for t in PRINT_TIMES:
        line = '  %6.3f ' % t
        for u, rows in runs:
            r = at(rows, t)
            line += '| %14.3f %6.3f %6.1f %6.1f  ' % (r['N'], r['N'] / (54.0 * u), r['tilt_gz_s'], r['tilt_gz_p'])
        print(line)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv', action='store_true')
    parser.add_argument('--transport', type=int, default=0)
    args = parser.parse_args()

    p = load_params()
    pods = p['plant']['pods']
    print('sources verified: airframe 2520 values at their cited lines, rigid and complete '
          'plants carry identical pods, world step %.3f s' % p['dt'])
    print('  tilt range %.7f .. %.7f rad (float radians), slew %.4f rad/s = %.1f deg/s -> 180 deg in %.3f s'
          % (p['tilt_min'], p['tilt_max'], p['slew'], math.degrees(p['slew']), M_PI_F / p['slew']))
    for side in p['sides']:
        pod = pods[side]
        full = pod['kF'] * pod['max_rot'] ** 2
        wn = math.sqrt(pod['p'] / pod['J'])
        print('  %-9s lever %+.4f m, full thrust %.2f N (kF %.1e x %.0f^2), %s, tau up/down %.4f/%.3f s; '
              'tilt PD p %.1f d %.2f on J %.4f + c %.2f: wn %.1f rad/s zeta %.2f, ramp lag c R/p = %.2f deg'
              % (side, pod['y_lever'], full, pod['kF'], pod['max_rot'], pod['turning'], pod['tau_up'],
                 pod['tau_down'], pod['p'], pod['d'], pod['J'], pod['damping'], wn,
                 (pod['d'] + pod['damping']) / (2 * math.sqrt(pod['p'] * pod['J'])),
                 math.degrees(pod['damping'] * p['slew'] / pod['p'])))
    authority = sum(pods[s]['kF'] * pods[s]['max_rot'] ** 2 * abs(pods[s]['y_lever']) for s in p['sides'])
    print('  full-command authority 2 x %.1f N x %.2f m = %.2f N m (sitl_check.py:50 uses 54.0)'
          % (pods['port']['kF'] * pods['port']['max_rot'] ** 2, abs(pods['port']['y_lever']), authority))

    steps = (0.1, 0.3, 1.0)
    fresh = {u: run(prepared(p, transport_ticks=args.transport), lambda t, u=u: u, 3.0) for u in steps}
    # aligned: pods already pointed for +u (held +0.05 for 3 s), then u = 0 for 1 s (rotors stop)
    aligned = {u: run(prepared(p, 0.05, 3.0, 1.0, args.transport), lambda t, u=u: u, 3.0) for u in steps}
    # reversal: held at -u for 3 s (N = -54u), then +u
    reverse = {u: run(prepared(p, -u, 3.0, 0.0, args.transport), lambda t, u=u: u, 3.0) for u in steps}

    print_table('A. Step from rest just after arming (both pods at 0 deg, rotors stopped), gate throttle 0:',
                [(u, fresh[u]) for u in steps])
    print_table('B. Step with the pods already pointed for +u (starboard 180, port 0), rotors stopped:',
                [(u, aligned[u]) for u in steps])
    print_table('C. Reversal: held at -u for 3 s (N = -54u), then +u at t = 0 (both pods swap ends):',
                [(u, reverse[u]) for u in steps])

    print('\nSummary (final = mean of the last 0.4 s of 3 s; tNN = first time N >= NN % of final; '
          'dead = impulse-equivalent pure delay of the whole change)')
    for label, table, start in (('A fresh', fresh, lambda u: 0.0), ('B aligned', aligned, lambda u: 0.0),
                                ('C reversal', reverse, lambda u: -54.0 * u)):
        for u in steps:
            final, t10, t50, t90, t99, dead = summarize(table[u], start(u))
            print('  %-10s u=%-4g final %8.4f N m = %.5f x 54u  N(0.5 s) = %+.3f x 54u   t10 %.3f t50 %.3f '
                  't90 %.3f t99 %.3f s   dead %.3f s'
                  % (label, u, final, final / (54.0 * u), at(table[u], 0.5)['N'] / (54.0 * u),
                     t10, t50, t90, t99, dead))
        if label.startswith('C'):
            for u in steps:
                rows = reverse[u]
                zero = first_time(rows, lambda r: r['N'] > -1e-3 * 54 * u)
                pos = first_time(rows, lambda r: r['N'] > 1e-3 * 54 * u)
                print('  %-10s u=%-4g |N| < 0.1%% of 54u from %.3f s to %.3f s: zero authority while both pods swing'
                      % ('', u, zero, pos))

    # symmetry: a negative step from rest swings the port pod instead
    mirror = run(prepared(p, transport_ticks=args.transport), lambda t: -0.3, 3.0)
    worst = max(abs(a['N'] + b['N']) for a, b in zip(fresh[0.3], mirror))
    print('  symmetry: fresh u=-0.3 is the mirror of u=+0.3 to %.2e N m (port swings to +180 instead)' % worst)
    for ticks in (1,):
        rows = run(prepared(p, transport_ticks=ticks), lambda t: 0.3, 3.0)
        final, t10, t50, t90, t99, dead = summarize(rows, 0.0)
        print('  transport %d tick instead of %d: fresh u=0.3 t10 %.3f t90 %.3f dead %.3f s'
              % (ticks, args.transport, t10, t90, dead))

    # the band: below kSteerEngage a pod does not swing
    print('\nSteer band (AirshipPod.hpp:54-55: engage 0.02, release 0.01):')
    for label, act, u in (('fresh, u=+0.015', prepared(p), 0.015),
                          ('fresh, u=+0.025', prepared(p), 0.025),
                          ('pods set for -u, u=+0.015', prepared(p, -0.05, 3.0, 1.0), 0.015),
                          ('pods set for -u, u=+0.025', prepared(p, -0.05, 3.0, 1.0), 0.025)):
        rows = run(act, lambda t: u, 4.0)
        r = rows[-1]
        print('  %-27s N after 4 s %7.4f N m = %.3f x 54u (tilts %6.1f / %6.1f deg, modes %s/%s, '
              'yaw unallocated %+g)' % (label, r['N'], r['N'] / (54 * u), r['tilt_gz_s'], r['tilt_gz_p'],
                                       r['mode_s'], r['mode_p'], r['unalloc_yaw']))
    rows = run(prepared(p), lambda t: 1.2, 3.0)
    print('  saturation: u=1.2 from rest -> %.3f N m after 3 s, yaw unallocated %+g (published sign)'
          % (rows[-1]['N'], rows[-1]['unalloc_yaw']))

    # what the other terms contribute
    worst = max((abs(r['N_react']) + abs(r['N_gyro']), u, r['t']) for u in steps for r in fresh[u] + reverse[u])
    print('\nNon-lever terms (rotor reaction + rotor angular momentum): worst |sum| %.4f N m at u=%g t=%.3f s'
          % worst)
    for u in steps:
        imp = sum(r['N_react'] + r['N_gyro'] for r in fresh[u]) * p['dt']
        lev = sum(r['N_lever'] for r in fresh[u]) * p['dt']
        print('  fresh u=%g: their impulse over 3 s %.4f N m s against %.2f from the lever (%.3f%%)'
              % (u, imp, lev, 100 * imp / lev))

    # The saturation flag during a sweep (what anti-windup sees)
    for name, rows in (('reversal u=-0.3 -> +0.3', reverse[0.3]), ('fresh step u=+0.3', fresh[0.3])):
        print('\nYaw saturation the rate controller reads on the next cycle, %s:' % name)
        last = first_time(rows[::-1], lambda r: r['sat_seen'] != 0.0)
        for t in (0.0, 0.1, 0.74, 0.8, 1.4, 1.488, 1.492, 1.496, 1.5, 1.6):
            r = at(rows, t)
            print('  t %.3f  allocator believes yaw %+.4f  unallocated sign %+g  controller reads %+g  modes %s/%s'
                  % (r['t'], r['yaw_alloc'], r['unalloc_yaw'], r['sat_seen'], r['mode_s'], r['mode_p']))
        print('  last tick with a nonzero flag: t %.3f s (positive integration frozen until then)' % last)

    validation(p)

    if args.csv:
        for name, table in (('fresh', fresh), ('aligned', aligned), ('reversal', reverse)):
            for u in steps:
                path = os.path.join(HERE, 'steps_%s_u%g.csv' % (name, u))
                with open(path, 'w', newline='') as handle:
                    writer = csv.DictWriter(handle, fieldnames=list(table[u][0].keys()))
                    writer.writeheader()
                    writer.writerows(table[u])
        print('\nCSV written to %s/steps_*.csv' % HERE)


def validation(p):
    """Checks against the recorded rigid gate (bench) sitl_out/chain7_20260925.txt."""
    print('\nAgainst the recorded rigid-plant gate, (bench) sitl_out/chain7_20260925.txt:')
    # :73 wind 1 N m, I on: last 2 s |rate| 0.10 deg/s, integrator 0.0179 (needed 0.0185)
    # :75 wind 0.4 N m: |rate| 0.06 deg/s, integrator 0.0073 (needed 0.0074)
    P = 0.5   # AS_YAWRATE_P (task context)
    for torque, integ, rate, line in ((1.0, 0.0179, 0.10, 73), (0.4, 0.0073, 0.06, 75)):
        lo = integ - P * math.radians(rate)
        hi = integ + P * math.radians(rate)
        print('  :%d  wind %.1f N m held by u = %.4f +- %.4f (I +- P|r|) -> %.1f .. %.1f N m per unit u '
              '(model steady gain 54.0 with both pods set; one pod would be 27.0)'
              % (line, torque, integ, P * math.radians(rate), torque / hi, torque / lo))
    # :66 wind check: 1 N m for a 5 s wall window moved -0.00 -> -2.09 deg/s; rigid Izz 129.2 predicts 2.22
    izz = 129.2
    window = 5.0 * 2.09 / 2.22
    print('  :66  1 N m for a nominal 5 s window gave 2.09 of the 2.22 deg/s Izz %.1f predicts: the '
          'effective window is %.2f s (%.0f%%)' % (izz, window, 100 * window / 5.0))
    # :67 Manual: stick torque 0.5 for a nominal 3 s window, -2.7 -> 23.5 deg/s
    rows = run(prepared(p), lambda t: 0.5, 3.2)
    omega, integral = 0.0, [0.0]
    for r in rows:
        omega += r['N'] / izz * p['dt']
        integral.append(math.degrees(omega))
    eff = 3.0 * window / 5.0
    def delta(t0, t1):
        return integral[int(round(t1 / p['dt']))] - integral[int(round(t0 / p['dt']))]
    print('  :67  Manual, u = 0.5 from rest (pods at 0): measured 26.2 deg/s over the nominal 3 s window; '
          'model %.1f over [0, %.2f] s (the :66 window ratio), %.1f over [0, 3.0], %.1f over [0.1, 2.8]; '
          'with no sweep (54u from t=0) %.1f, one pod only %.1f over [0, %.2f]'
          % (delta(0, eff), eff, delta(0, 3.0), delta(0.1, 2.8), math.degrees(27.0 / izz * eff),
             math.degrees(13.5 / izz * eff), eff))


if __name__ == '__main__':
    main()
