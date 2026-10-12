#!/usr/bin/env python3
"""Forward flight on the yaw model: one gz target in straight flight at u0,
a constant throttle, the HEAD controller, and a yaw stick leg.

    python3 forward.py gz_airship_vectored_independent_am_fin_drag --u0 4 --stick 1
    python3 forward.py gz_cloudship_am_fin_drag --u0 2 --stick 0.5 --param AS_YAWRATE_FF=0 --csv leg.csv
    from forward import run_leg, trim_throttle

The leg, on the simulator's 4 ms step with the surge free (planar
RollPlanarPlant: surge, sway, roll, yaw; added mass, Munk moment, hull drag
at the link origin, the lift-drag fin replica), every number from the model
files through the existing parts:

  warm-up   warmup_s with the plant held: the controller in Manual, the
            throttle on, so the rotors and the allocator's pods are at their
            steady state when the hull starts to move (the hull's entry state
            is exactly u0, v0, r0, p0, phi0)
  Manual    manual_s, loop open, integrator clear, yaw stick centred
  Acro      at t = 0 the loop closes; centred_s with the stick centred
  stick     stick_s at the yaw stick s (positive: nose right)
  release   release_s with the stick centred

Frames: FRD in everything a caller gives or reads, inputs and outputs alike:
u forward, v to starboard, r and the heading positive nose right, p and phi
positive right wing down, all at the hull link origin (the model origin,
base_link: what forward_yaw_check records). The plant integrates in FLU
(planar.py); the conversion is here and nowhere else (v_FLU = -v, r_FLU = -r;
roll is the same axis in both). Until 2026-10-01 v0 and the reported v were
FLU (port positive); both are FRD now.

Throttle. A fraction 0..1 of full thrust (throttle 'trim' is trim_throttle():
gz_model.trim_thrust, the formula forward_yaw_check flies, behind the checks
that make it the trim). It reaches the firmware through the MANUAL_CONTROL z
(controller.throttle_stick): throttle = z / 500 - 1
(mavlink_receiver.cpp:2379), thrust_x = (throttle + 1) / 2
(airship_manual_input.hpp:70-73), into the allocator ports with the loop's
yaw: the vectored pods share it with the yaw couple, a pod whose share goes
negative swings through its tilt (AirshipPod.cpp:185-193 clamps its thrust at
0), and the Cloudship's side motors take it while the tail takes the yaw.
What z is (quantize):
  'midpoint' (default)  as the 0b check flies it: forward_yaw_check.quantize
             rounds the fraction to the nearest count and hands sitl_link the
             count's midpoint (counts + 0.5) / 1000, which its int()
             truncates to the count, so PX4 flies round(fraction * 1000) / 1000
             (2527 at 2 m/s: trim 0.03054 flies 31/1000; 2517 at 2.0 m/s:
             0.0119 flies 12/1000)
  True       sitl_link given the fraction itself, z = int(fraction * 1000)
             (what forward.py flew until 2026-10-01: 30 and 11 counts there)
  False      the fraction exactly: a replay of a recorded thrust setpoint (a
             mean of float32 values such as 0.0309999995, which True would
             cut to 30 counts), or a sweep that wants the speed it is labelled
             with

Reversals, two different quantities under one word, kept apart:
  pod swings (vectored): a pod's thrust axis flipping between forward and
    back, on the allocator's own pods (the tilt PX4 commands), counted by a
    Schmitt trigger on cos(tilt) at +-FIN_BAND (forward inside 60 deg, back
    beyond 120 deg), so a pod passing through the vertical counts once and a
    dither at 90 deg does not count; per pod, into the rear and back forward,
    and the time spent in the rear. A counted swing takes the tilt servo
    through the vertical, 60 deg of travel at the least.
  tail flips (Cloudship): the tail command's sign. The tail is commanded
    continuously through zero (ActuatorEffectivenessAirship.cpp:215-222, its
    reverse range from CA_R_REV bit 2) and gz's tail is linear with no
    deadband and no lag (cloudship_actuation.TailChain: F = 2.3544 N per unit
    command, minCommand 0, no time constant), so physically a flip is any sign
    change of its thrust and costs nothing in gz. TAIL_BAND is a counting
    convention, not a firmware or actuator threshold: a direction counts once
    the command leaves +-TAIL_BAND (2 % of the tail's authority), and the
    count at band 0 on the thrust gz applies is kept beside it.
'rev_stick' / 'rev_release' are the vehicle's own count (pod swings summed
over the pods, or tail flips at TAIL_BAND), counted on every 4 ms step. The
record samples the commands more coarsely (actuator_servos and
actuator_motors at 100 ms on SITL's logger profile); reversals_at() counts
the model's commands at a log's own sample times, with forward_yaw_check's
own counting functions, for a count against a record.

Fins. Each vertical fin's angle of attack as the plugin computes it
(planar.lift_drag_alpha) against its alpha_stall: 'stalled' past it, where
the lift-drag curve takes its post-stall branch, 'no flow' where the plugin
applies nothing (the flow at its cp comes from behind: a hull spinning or
sliding backward). The plugin itself has no hysteresis; the closed loop does
(airship_tail_design.md:113-121): a stalled tail damps less, the rate grows
and keeps it stalled. fin_names() names a record's alpha_fin<n> columns the
same way; fin_alphas() computes the plugin's angle from a recorded state.

Timing. Everything is per 4 ms tick, deterministic unless noise=True (gz's
gyro noise, seeded). A row (series, trace, entry) at time t holds the plant
at t and the controller's and allocator's outputs of the cycle that ran at
t - DT (the cycle whose commands drove the step that ended at t): the
integrator, the torque, the setpoint, the flags, the pods' tilts, the tail
command. A log samples the same outputs at the cycle's own time, 4 ms
earlier; command_at() and reversals_at() take that into account.

Metrics a 0b record has too, on the record's own windows (forward_yaw_check
leg_metrics): u as 0.5 s means at the stick's start, its end and the
release's end; r as 1 s means ending at the release marks, of the measured
rate (the gyro the loop reads: the 40 Hz filter, gz's noise if on), against
the record's gyro means; the stick's peak as the extreme of a 0.5 s moving
mean of the measured rate; the integrator at the marks. A mark past the leg's
end is None, or the end itself when it is no more than forward_yaw_check's
DURATION_TOL_S past it (its mark_time()). The instantaneous metrics of
before stay as they were (r_rel_*, u_stick_*, r_peak).

Entry state and the start (a replay of a record starts from either recorded
instant, replay_kwargs()): start 'acro' (default) runs the timeline above;
start 'release' begins at the stick's release with the loop already closed
and runs release_s centred. pods0, motors0, integ0, sat0 and gyro_settled set
the allocator's pods (tilt, target, mode) and the gz tilt joints, the rotors,
the integrator, the saturation flags and the IMU filter at the start; none of
them given, the leg is the one it always was.

Policy primitives for the 0c comparison (all off by default; off, the leg is
bit-identical to the leg without them): hook, called every cycle before the
controller with (t, plant, controller, allocator), may set the feedforward
gain and the yaw-rate cap (controller.set_feedforward, set_rate_cap: the cap
both as the stick's full scale and as a clamp on the loop's setpoint) and
reads the true air-relative and the ground-relative surge speed (PlantView);
restore switches on options/D's restore rule; wind, a steady uniform wind,
reaches only the ground-speed reading (it does not change the air-relative
dynamics); tilt_range overrides CA_AIRSHIP_TLMIN/TLMAX of the vectored pods
(with the bridge's servo angle range, so the joint goes where the allocator
believes), e.g. (-90, 90) for pods that cannot point backward, A = 0.5.
"""
import argparse
import array
import bisect
import contextlib
import csv
import functools
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import closed_loop as cl  # noqa: E402
import controller as ctl  # noqa: E402
import forward_yaw_check as fyc  # noqa: E402  (bench root, on sys.path through gate; read only)

gz_model = cl.gate.gz_model
DT = cl.DT
# Counting conventions, not thresholds of the firmware or the actuators (the
# module docstring); forward_yaw_check counts its record with the same two
FIN_BAND = 0.5          # Schmitt on cos(tilt): forward above +0.5, back below -0.5
TAIL_BAND = 0.02        # a tail command inside +-0.02 is neither direction
SAMPLE_HZ = 10.0
POD_NAMES = ('stbd', 'port')                    # the allocator's pods 0 and 1
POD_MODES = (ctl.STEERING, ctl.HOLDING, ctl.RELEASED, ctl.PARKED)
MARKS_S = (10.0, 60.0)                          # release marks besides the end (forward_yaw_check's)
MEAN_U_S = 0.5                                  # forward_yaw_check's u windows
MEAN_R_S = 1.0                                  # its r window ending at a mark
PEAK_WINDOW_S = 0.5                             # its smoothed peak's window


def linear_thrust_check(model):
    """The airframe that flies `model`, after checking what makes a thrust
    fraction linear in thrust on its two pod motors, which trim_throttle's
    division assumes: THR_MDL_FAC 1 in its posix airframe chain
    (FunctionMotors' square root then cancels the rotor's square), and on each
    pod motor SIM_GZ_EC_MIN 0 and SIM_GZ_EC_MAX equal to the motor plugin's
    maxRotVelocity (gz output n is the plugin's motorNumber n - 1), so a
    fraction of 1 is the plugin's full rotor speed and 0 a stopped rotor.
    Raises on any other value, or on a value that cannot be read."""
    number = ctl.airframe_for_model(model)
    defaults = ctl.airframe_defaults(number)

    def read(name):
        if name not in defaults:
            raise LookupError('posix airframe %d (%s) does not set %s' % (number, model, name))

        return float(defaults[name][0])

    if read('THR_MDL_FAC') != 1.0:
        raise ValueError('%d: THR_MDL_FAC %s, not 1: thrust is not linear in the fraction'
                         % (number, defaults['THR_MDL_FAC'][0]))

    merged = gz_model.merged(model)

    for link in ('prop_starboard', 'prop_port'):
        plugin = gz_model.motor_plugin(merged, link)

        if plugin is None or gz_model.MULTICOPTER not in plugin.get('filename', ''):
            raise LookupError('%s: %s is not driven by a MulticopterMotorModel' % (model, link))

        n = int(plugin.find('motorNumber').text) + 1
        top = float(plugin.find('maxRotVelocity').text)
        low, high = read('SIM_GZ_EC_MIN%d' % n), read('SIM_GZ_EC_MAX%d' % n)

        if low != 0.0 or high != top:
            raise ValueError('%d: %s (motor %d) maps the fraction onto %g..%g rad/s against maxRotVelocity %g: '
                             'thrust is not the fraction times full' % (number, link, n, low, high, top))

    return number


def trim_throttle(model, u):
    """(fraction, drag N, full thrust N): the thrust fraction that holds `u`
    in straight flight on `model` (gz_model.trim_thrust: the surge drag
    |xUabsU| u^2 of its Hydrodynamics block over pod_thrust(), the full
    forward thrust of the two pods; 0 on a plant without a drag block, whose
    fins have no drag at zero incidence), after linear_thrust_check()."""
    linear_thrust_check(model)
    fraction = gz_model.trim_thrust(model, u)
    full = gz_model.pod_thrust(model)
    return fraction, fraction * full, full


def flown_fraction(fraction, quantize='midpoint'):
    """The thrust fraction PX4 flies for a commanded `fraction`
    (controller.throttle_stick's z / 1000)."""
    return (ctl.throttle_stick(fraction, quantize) + 1.0) * 0.5


def default_legs(target):
    """[(u0 m/s, stick)] of the 0b check's default legs for a gz target
    (forward_yaw_check.default_legs, read from the check itself)."""
    return list(fyc.default_legs(cl.ALL_TARGETS[target][0]))


class Schmitt:
    """A +-1 state that changes only past +-band, and its flips: to_neg (into
    -1: a pod into the rear, the tail to starboard thrust) and to_pos."""

    def __init__(self, band):
        self.band = band
        self.state = None
        self.flips = 0
        self.to_neg = 0
        self.to_pos = 0

    def update(self, x):
        new = 1 if x > self.band else (-1 if x < -self.band else self.state)

        if new is not None and self.state is not None and new != self.state:
            self.flips += 1
            self.to_neg += new < 0
            self.to_pos += new > 0

        self.state = new


@contextlib.contextmanager
def _held(plant):
    """The plant does not move: the actuators and the controller run on."""
    step = plant.step_actuated
    plant.step_actuated = lambda *args, **kw: None

    try:
        yield
    finally:
        plant.step_actuated = step


class PlantView:
    """What run_leg's per-cycle hook reads of the plant, FRD, at the hull
    link origin:
      u_air, v_air   m/s, the velocity relative to the air: the plant's own
                     state (planar.py integrates the air-relative motion; a
                     steady uniform wind leaves it unchanged)
      u_ground, v_ground  m/s, relative to the ground, body axes: the air
                     velocity plus the wind, the wind rotated into the hull's
                     current heading. Equal to u_air, v_air without wind
      r              deg/s, true yaw rate (no filter, no noise)
      heading        deg, since the leg's start, positive nose right
      raw            the planar plant itself (FLU: nu = u, v, p, r)
    wind: (speed m/s, toward deg) as run_leg takes it, or None."""

    def __init__(self, plant, wind=None):
        self.raw = plant
        self.wind = None if wind is None else (float(wind[0]), math.radians(float(wind[1])))

    @property
    def u_air(self):
        return self.raw.nu[0]

    @property
    def v_air(self):
        return -self.raw.nu[1]

    @property
    def r(self):
        return -math.degrees(self.raw.nu[3])

    @property
    def heading(self):
        return -math.degrees(self.raw.psi)

    def wind_body(self):
        """(x, y) of the wind in body axes, FRD, m/s."""
        if self.wind is None:
            return 0.0, 0.0

        speed, toward = self.wind
        rel = toward + self.raw.psi                 # toward, minus the heading (FRD heading = -psi)
        return speed * math.cos(rel), speed * math.sin(rel)

    @property
    def u_ground(self):
        return self.u_air + self.wind_body()[0]

    @property
    def v_ground(self):
        return self.v_air + self.wind_body()[1]


def _check_wind(wind):
    if wind is None:
        return None

    speed, toward = wind

    if not (ctl.isfinite(speed) and speed >= 0.0 and ctl.isfinite(toward)):
        raise ValueError('wind %r: (speed m/s >= 0, toward deg), both finite' % (wind,))

    return float(speed), float(toward)


def _pod_parts(loop, vectored):
    """(allocator pods, gz tilt joints, gz rotors, (THR_MDL_FAC, EC_MIN, EC_MAX),
    joint angle the allocator's servo commands for pod i) of the actuation."""
    act = loop.actuation

    if vectored:
        a = act.act
        p = a.p
        angle = lambda i, pod: act.va.servo_angle(pod.servo_setpoint(), p['sv_mina'][i], p['sv_maxa'][i],
                                                  p['sv_min'], p['sv_max'])
        return a.alloc.pods, a.joints, a.rotors, (p['thr_mdl_fac'], p['ec_min'], p['ec_max']), angle

    pods = act.pods
    angle = lambda i, pod: pods.va.servo_angle(pod.servo_setpoint(), pods.sv_mina, pods.sv_maxa, pods.sv_min,
                                               pods.sv_max)
    return pods.alloc.pods, [pods.joint], pods.rotors, (pods.fac, pods.ec_min, pods.ec_max), angle


def _range(pod):
    """(min, max) tilt of either pod port, rad."""
    return (pod.tmin, pod.tmax) if hasattr(pod, 'tmin') else (pod.tilt_min, pod.tilt_max)


def actuator_state(loop, vectored):
    """The actuators as pods0 / motors0 take them: 'pods', per allocator pod
    (starboard, port) {'tilt', 'target' deg (the allocator's), 'mode', 'gz'
    deg (the gz tilt joint's angle; the Cloudship's one joint for both),
    'gz_rate' deg/s}; 'motors', the two pod motor commands of the last cycle
    (0..1, actuator_motors control[0..1]); 'tail', the Cloudship's tail
    command (None on the vectored airship)."""
    alloc_pods, joints, rotors, _, _ = _pod_parts(loop, vectored)
    out = []

    for i, pod in enumerate(alloc_pods):
        joint = joints[i if vectored else 0]
        out.append({'tilt': math.degrees(pod.tilt), 'target': math.degrees(pod.target), 'mode': pod.mode,
                    'gz': math.degrees(joint.theta), 'gz_rate': math.degrees(joint.omega)})

    act = loop.actuation

    if vectored:
        row = act.row or {}
        return {'pods': out, 'motors': [row.get('c_s', 0.0), row.get('c_p', 0.0)], 'tail': None}

    return {'pods': out, 'motors': list(act.pods.motors), 'tail': act.stage['cmd'] if act.stage else 0.0}


def _pod_specs(pods0, alloc_pods, vectored):
    """pods0 checked and in radians: [(tilt, target, mode, gz or None, gz_rate)]."""
    if len(pods0) != len(alloc_pods):
        raise ValueError('pods0: %d entries for %d pods (starboard, port)' % (len(pods0), len(alloc_pods)))

    specs = []

    for i, (spec, pod) in enumerate(zip(pods0, alloc_pods)):
        if 'tilt' not in spec or 'mode' not in spec:
            raise ValueError('pods0[%d] %r: needs tilt and mode (the log carries no mode; the model does not '
                             'guess one)' % (i, spec))

        if spec['mode'] not in POD_MODES:
            raise ValueError('pods0[%d] mode %r: one of %s' % (i, spec['mode'], POD_MODES))

        tilt = spec['tilt']
        target = spec.get('target', tilt)
        gz = spec.get('gz')
        gz_rate = spec.get('gz_rate', 0.0)
        values = [tilt, target, gz_rate] + ([] if gz is None else [gz])

        if not all(isinstance(x, (int, float)) and ctl.isfinite(x) for x in values):
            raise ValueError('pods0[%d] %r: a tilt, target, gz or gz_rate that is not a finite number' % (i, spec))

        lo, hi = _range(pod)

        for name, x in (('tilt', tilt), ('target', target)):
            if not lo - 1e-6 <= math.radians(x) <= hi + 1e-6:
                raise ValueError('pods0[%d] %s %g deg outside the tilt range %g..%g deg'
                                 % (i, name, x, math.degrees(lo), math.degrees(hi)))

        specs.append((ctl.constrain(math.radians(tilt), lo, hi), ctl.constrain(math.radians(target), lo, hi),
                      spec['mode'], None if gz is None else math.radians(gz), math.radians(gz_rate)))

    if not vectored and (specs[0][:3] != specs[1][:3] or specs[0][3:] != specs[1][3:]):
        raise ValueError('pods0 %r: the Cloudship\'s pods are collective (one tilt servo, one joint): the two '
                         'entries must be equal' % (pods0,))

    return specs


@contextlib.contextmanager
def _pinned(alloc_pods, specs):
    """The allocator's pods keep the given tilt, target and mode whatever
    steer() or park() would make of the demand: during the warm-up, so the
    rotors settle on the drive command of the pinned tilt."""
    for pod, (tilt, target, mode, _, _) in zip(alloc_pods, specs):
        def keep(*args, pod=pod, tilt=tilt, target=target, mode=mode):
            pod.tilt, pod.target, pod.mode = tilt, target, mode

        pod.steer = keep        # instance attributes over the class's methods
        pod.park = keep

    try:
        yield
    finally:
        for pod in alloc_pods:
            del pod.steer
            del pod.park


def _impose_actuators(loop, vectored, specs, motors0):
    """The pods (specs, from _pod_specs) and the rotors (motors0) as given,
    after the warm-up: the allocator's pod state through its servo round trip
    (writeTiltServos, EA:233-256), the gz joints settled at the angle the servo
    commands unless 'gz' says otherwise, each rotor at the steady state of its
    motor command (THR_MDL_FAC, SIM_GZ_EC_* to the rotor speed, the motor
    model's filter at it), and the reports the leg reads made consistent."""
    alloc_pods, joints, rotors, (fac, ec_min, ec_max), angle_of = _pod_parts(loop, vectored)
    act = loop.actuation

    if specs is not None:
        for pod, (tilt, target, mode, _, _) in zip(alloc_pods, specs):
            pod.tilt, pod.target, pod.mode = tilt, target, mode

        if vectored:
            for pod in alloc_pods:
                pod.set_servo_setpoint(ctl.constrain(pod.servo_setpoint(), -1.0, 1.0))
        else:
            servo = ctl.constrain(alloc_pods[0].servo_setpoint(), -1.0, 1.0)

            for pod in alloc_pods:
                pod.set_servo_setpoint(servo)

            act.pods.alloc.servos[0] = servo
            act.pods.servo = servo

        for i, joint in enumerate(joints):
            _, _, _, gz, gz_rate = specs[i]
            cmd = angle_of(i, alloc_pods[i])
            joint.cmd = cmd
            joint.theta = cmd if gz is None else gz
            joint.omega = gz_rate
            joint.err_last = joint.theta - cmd
            joint.i_err = 0.0

    if motors0 is not None:
        if len(motors0) not in ((2,) if vectored else (2, 3)):
            raise ValueError('motors0 %r: the two pod motors (and on the Cloudship optionally the tail)' % (motors0,))

        if len(motors0) == 3:
            tail = motors0[2]

            if not (isinstance(tail, (int, float)) and ctl.isfinite(tail) and act.chain.u_min <= tail <= act.chain.u_max):
                raise ValueError('motors0[2] %r: the tail command, finite, %g..%g'
                                 % (tail, act.chain.u_min, act.chain.u_max))

            act.stage = act.chain.stages(float(tail))       # the tail's last command (a static map, no state)

        va = act.va if vectored else act.pods.va

        for i, (rotor, c) in enumerate(zip(rotors, motors0[:2])):
            if not (isinstance(c, (int, float)) and ctl.isfinite(c) and 0.0 <= c <= 1.0):
                raise ValueError('motors0[%d] %r: a pod motor command is a finite 0..1' % (i, c))

            ref = min(va.esc_output(float(c), fac, ec_min[i], ec_max[i]), rotor.max_rot)
            rotor.ref = rotor.filtered = ref
            rotor.joint_vel = rotor.joint_vel_cmd = rotor.td * ref / rotor.slowdown
            rotor.thrust = ref * ref * rotor.kF

    if vectored:
        row = dict(act.row or {})
        row.update(tilt_px4_s=math.degrees(alloc_pods[0].tilt), tilt_px4_p=math.degrees(alloc_pods[1].tilt),
                   tilt_gz_s=math.degrees(joints[0].theta), tilt_gz_p=math.degrees(joints[1].theta),
                   mode_s=alloc_pods[0].mode, mode_p=alloc_pods[1].mode,
                   T_s=rotors[0].thrust, T_p=rotors[1].thrust)

        if motors0 is not None:
            row.update(c_s=float(motors0[0]), c_p=float(motors0[1]))

        act.row = row
        act._hx_prev = act._hx()        # the props' momentum memory, for the roll term's derivative
    else:
        act.pods.thrust = [r.thrust for r in rotors]
        act.pods.theta = joints[0].theta

        if motors0 is not None:
            act.pods.motors = [float(c) for c in motors0[:2]]


def _override_tilt_range(loop, tilt_range):
    """CA_AIRSHIP_TLMIN/TLMAX of the vectored pods, deg, before anything has
    stepped: the allocator's range as load_params reads it (float radians,
    ActuatorEffectivenessAirship.cpp:56-57) and the bridge's servo angle range
    SIM_GZ_SV_MINA/MAXA with it, so the servo's -1..1 maps onto the angles
    the allocator means (GZMixingInterfaceServo.cpp:144-145)."""
    act = loop.actuation

    if not isinstance(act, cl.VectoredAct):
        raise ValueError('tilt_range: the vectored pods only (the Cloudship\'s collective tilt carries no yaw)')

    lo, hi = tilt_range

    if not (ctl.isfinite(lo) and ctl.isfinite(hi) and -180.0 <= lo < hi <= 180.0):
        raise ValueError('tilt_range %r: -180 <= min < max <= 180 deg' % (tilt_range,))

    p = dict(act.p)
    p['tilt_min'], p['tilt_max'] = act.va.radians_f(lo), act.va.radians_f(hi)
    p['sv_mina'], p['sv_maxa'] = [float(lo), float(lo)], [float(hi), float(hi)]
    act.p = p
    act.act = act.va.VectoredActuation(p, 0)


def _check_flags(sat0):
    if sat0 is None:
        return False, False

    pos, neg = sat0

    if not (isinstance(pos, bool) and isinstance(neg, bool)) or (pos and neg):
        raise ValueError('sat0 %r: (positive, negative), two bools, not both' % (sat0,))

    return pos, neg


def run_leg(target, u0, stick, throttle='trim', params=None, noise=False, seed=1, hold_u=None, quantize='midpoint',
            warmup_s=1.0, manual_s=2.0, centred_s=3.0, stick_s=20.0, release_s=120.0, sample_hz=SAMPLE_HZ, v0=0.0,
            r0=0.0, stick_in_manual=False, phi0=0.0, p0=0.0, start='acro', pods0=None, motors0=None, integ0=None,
            sat0=None, gyro_settled=False, hook=None, restore=None, restore_trim0=None, wind=None, tilt_range=None):
    """One forward leg. Returns a dict: 'metrics', 'entry' (the state at each
    segment's start, with the full actuator state and p), 'series'
    (sample_hz rows), 'trace' (every 4 ms step, arrays), 'setup'.

    The leg (FRD, deg and deg/s for angles and rates, m/s for speeds):
      target     a closed_loop.ALL_TARGETS name ('gz_' + gz model)
      u0, v0     the entry's surge and sway at the link origin [m/s, FRD: v0
                 to starboard]. Until 2026-10-01 v0 was FLU (port positive)
      r0, p0     the entry's yaw and roll rates [deg/s, FRD]
      phi0       the entry's roll [deg, FRD]
      stick      the yaw stick of the stick segment, -1..1 (positive nose
                 right); with start 'release', the stick held before it
      throttle   'trim' (trim_throttle) or a thrust fraction 0..1; quantize
                 'midpoint' (default, as 0b flies it), True or False: module
                 docstring
      params     overrides on top of the target's HEAD gains
                 (controller.head_params), e.g. {'AS_YAWRATE_FF': 0.0}
      hold_u     hold the surge at that speed (the reference review's
                 fixed-speed model); None, a free surge
      warmup_s, manual_s, centred_s, stick_s, release_s  segment lengths [s],
                 taken to the nearest 4 ms step (the grid the loop runs on)
      stick_in_manual  start already turning with the stick held, Manual
                 passing it through as torque (a hull put on the fins'
                 stalled branch before the loop closes,
                 airship_tail_design.md:113-121); centred_s is then 0
      noise, seed  gz's gyro noise (closed_loop W2), seeded
      sample_hz  the series' rate

    Entry state (none given: the leg it always was):
      start      'acro' (default): t = 0 is the Acro switch, the timeline
                 above. 'release': t = 0 is the stick's release; the loop is
                 already closed (no reset on closing, the restore rule's
                 memory kept), the stick centred for release_s; manual_s,
                 centred_s and stick_s are 0 and the stick metrics None; the
                 reversal counters carry in the directions the pods (and the
                 Cloudship's tail command, motors0[2]) have at the start.
                 Needs integ0, pods0 and motors0 (the record's at-release
                 values: a turn's pods and rotors are not the warm-up's).
      pods0      per allocator pod (starboard, port; on the Cloudship the
                 collective pair, two equal entries) a dict: 'tilt' deg (the
                 allocator's tilt: what actuator_servos records through
                 CA_AIRSHIP_TLMIN/TLMAX), 'mode' ('steering', 'holding',
                 'released' or 'parked': the log does not carry it, so it is
                 required), 'target' deg (default the tilt), 'gz' deg (the gz
                 joint's angle, default the angle the servo command maps to,
                 a settled joint), 'gz_rate' deg/s (default 0). The pods are
                 pinned to it through the warm-up (the rotors settle on the
                 drive command of that tilt), then set exactly. The form of
                 entry[...]['actuators']['pods'].
      motors0    the two pod motor commands (0..1, actuator_motors
                 control[0..1]): each rotor starts at its steady state. On
                 the Cloudship a third value is the tail's last command
                 (control[2], -1..1; no dynamics, it only sets the reported
                 tail and, at a release start, the direction the tail's
                 reversal count carries in). Default: the warm-up's.
      integ0     the integrator (rate_ctrl_status yawspeed_integ,
                 normalized torque) at the start; within +-AS_YR_INT_LIM (a
                 float32 value a hair over it is taken at the limit).
                 Default: as built, 0.
      sat0       (positive, negative) saturation flags the controller holds
                 at the start (AirshipYawRateLoop::setSaturation's), until it
                 reads a status of the leg's own allocation (the warm-up's
                 count as read, a status on the first cycle carries the same
                 sign). Default None: an Acro start reads Manual's last status
                 on its first closed cycle, as the firmware does; a release
                 start holds (False, False).
      gyro_settled  the IMU filter starts settled at the entry's yaw rate (a
                 hull that has been turning at r0) instead of at 0. Default
                 False, as before; a replay wants True.

    Policy primitives (all off by default, and off the leg is bit-identical):
      hook       hook(t, plant, controller, allocator), called every cycle
                 of the leg (not the warm-up) before the controller runs:
                 t s since the Acro switch (the release with start
                 'release'), plant a PlantView (u_air, u_ground, ...),
                 controller the controller.AirshipYawController
                 (set_feedforward(ff), set_rate_cap(deg/s, stick, clamp),
                 integ, rate_sp [rad/s], params), allocator the allocator
                 the pods run in (vectored: vectored_actuation.Allocator;
                 Cloudship: the side pods' controller.AirshipAllocator; the
                 tail is a static map, cloudship_actuation.TailChain).
      restore    None, True or 'zero_or_sign' (options/D's restore rule), or
                 'zero' (its zero-only edge): controller.set_restore
      restore_trim0  with start 'release' and restore on: the integrator
                 saved when the stick left the deadzone (required)
      wind       (speed m/s, toward deg): a steady uniform wind the air moves
                 with, toward that direction measured from the hull's heading
                 at the leg's start, positive to the right (180: a headwind
                 at the start). It changes only PlantView.u_ground/v_ground
                 and the series' 'u_g', 'v_g'
      tilt_range (min deg, max deg): CA_AIRSHIP_TLMIN/TLMAX of the vectored
                 pods, with SIM_GZ_SV_MINA/MAXA; (-90, 90): pods that cannot
                 point backward"""
    model, airframe = cl.ALL_TARGETS[target]
    vectored = airframe == 'vectored'

    if start not in ('acro', 'release'):
        raise ValueError('start %r: acro or release' % (start,))

    if start == 'release' and (integ0 is None or pods0 is None or motors0 is None or stick_in_manual):
        raise ValueError('start release: needs integ0, pods0 and motors0 (the at-release state), and no '
                         'stick_in_manual')

    restore = 'zero_or_sign' if restore is True else (None if restore is False else restore)

    if start == 'release' and restore is not None and restore_trim0 is None:
        raise ValueError('start release with the restore rule: restore_trim0 (the saved integrator) is required')

    wind = _check_wind(wind)
    pos0, neg0 = _check_flags(sat0)
    gains = ctl.head_params(model)
    gains.update(params or {})
    trim, drag, full = trim_throttle(model, u0)
    fraction = trim if throttle == 'trim' else float(throttle)
    stick_thr = ctl.throttle_stick(fraction, quantize)
    flown = (stick_thr + 1.0) * 0.5
    loop = cl.build(target, params=gains, noise=noise, seed=seed, hold_u=hold_u)

    if tilt_range is not None:
        _override_tilt_range(loop, tilt_range)

    plant, act, ctrl = loop.plant, loop.actuation, loop.controller
    yc = ctrl.ctrl

    if restore is not None:
        yc.set_restore(restore)

    if stick_in_manual:
        centred_s = 0.0

    if start == 'release':
        manual_s = centred_s = stick_s = 0.0

    grid = lambda x: round(round(x / DT) * DT, 6)
    manual_s, centred_s, stick_s, release_s = (grid(x) for x in (manual_s, centred_s, stick_s, release_s))
    alloc_pods = _pod_parts(loop, vectored)[0]
    specs = None if pods0 is None else _pod_specs(pods0, alloc_pods, vectored)

    if integ0 is not None:
        lim = gains['AS_YR_INT_LIM']

        if not (isinstance(integ0, (int, float)) and ctl.isfinite(integ0) and abs(integ0) <= lim * (1 + 1e-6)):
            raise ValueError('integ0 %r: a finite integrator within +-AS_YR_INT_LIM %g' % (integ0, lim))

    # FRD -> the plant's FLU: roll is the same axis, sway and yaw flip
    plant.set_state(u=u0, v=-v0, p=math.radians(p0), r=-math.radians(r0), phi=math.radians(phi0))
    loop.arm()
    loop.set_stick(stick if stick_in_manual else 0.0)
    loop.set_sticks(throttle=stick_thr)
    loop.set_mode('manual')

    with _held(plant), (_pinned(alloc_pods, specs) if specs is not None else contextlib.nullcontext()):
        loop.advance(warmup_s)

    if specs is not None or motors0 is not None:
        _impose_actuators(loop, vectored, specs, motors0)

    if integ0 is not None:
        lim = gains['AS_YR_INT_LIM']
        yc.rc.integ = ctl.constrain(float(integ0), -lim, lim)

    if sat0 is not None or start == 'release':
        # the controller holds these flags until it reads a status of the
        # leg's own: the warm-up's statuses (a Manual allocation) count as
        # read, and a status published on the first cycle carries the same
        # sign. Not done on a plain Acro start, where reading Manual's last
        # status on the first closed cycle is what the firmware does.
        yc.rc.set_saturation(pos0, neg0)
        yc._status_gen_seen = ctrl.status_topic.gen
        loop._unallocated = 1.0 if pos0 else (-1.0 if neg0 else 0.0)

    if gyro_settled:
        plant.meas = plant.lpf.reset(-plant.nu[3])

    if start == 'release':
        # the loop has been closed through the stick: no reset on closing, the
        # restore memory as it was when the stick came off
        yc._was_active = True
        ctrl._published = True
        yc._restore_sp_prev = yc._setpoint(stick)
        yc._restore_trim = float(restore_trim0) if restore_trim0 is not None else 0.0

    view = PlantView(plant, wind)
    allocator = act.act.alloc if vectored else act.pods.alloc
    us0 = ctrl.t_us                     # the controller's clock at the last warm-up cycle
    from_acro = start == 'acro'
    t_stick, t_release = centred_s, grid(centred_s + stick_s)
    t_end = grid(t_release + release_s)
    every = max(1, int(round(1.0 / (sample_hz * DT))))
    pods = [Schmitt(FIN_BAND) for _ in POD_NAMES] if vectored else []
    tails = [] if vectored else [Schmitt(TAIL_BAND), Schmitt(0.0)]     # the command at the band; the thrust at 0

    if start == 'release':
        # the stick's directions carried into the release, as a leg from the
        # Acro switch carries them: from the actuators at the start
        side = lambda x, band: 1 if x > band else (-1 if x < -band else None)

        for f, pod in zip(pods, act.act.alloc.pods if vectored else ()):
            f.state = side(math.cos(pod.tilt), FIN_BAND)

        if tails and act.stage:
            tails[0].state, tails[1].state = side(act.stage['cmd'], TAIL_BAND), side(act.stage['F'], 0.0)

    swings = {seg: [{'into_rear': 0, 'back_forward': 0, 'rear_s': 0.0} for _ in pods] for seg in ('stick', 'release')}
    flips = {seg: [0 for _ in tails] for seg in ('stick', 'release')}
    entry, series = {}, []
    fin_max = {'stick': {}, 'release': {}}
    stalled_ticks = {'stick': 0, 'release': 0}
    n_seg = {'stick': 0, 'release': 0}
    keys = ['t', 'r', 'r_meas', 'u', 'v', 'heading', 'integ', 'torque', 'sp']
    keys += ['tilt_s', 'tilt_p'] if vectored else ['tail', 'tilt']
    keys += ['u_g'] if wind is not None else []
    trace = {key: array.array('d') for key in keys}

    def state(t, full=False):
        u, v, p, r = plant.nu
        rc = yc.rc
        out = {'t': round(t, 4), 'u': u, 'v': -v, 'r': -math.degrees(r), 'r_meas': math.degrees(plant.measured_rate()),
               'phi': math.degrees(plant.phi), 'heading': -math.degrees(plant.psi), 'integ': rc.integ,
               'torque': loop.read_torque(), 'sat_pos': rc.sat_pos, 'sat_neg': rc.sat_neg,
               'unalloc': loop._unallocated, 'mode': loop._mode, 'stick': loop._stick}

        if vectored:
            pod = act.act.alloc.pods
            row = act.row or {}
            out.update(tilt_s=math.degrees(pod[0].tilt), tilt_p=math.degrees(pod[1].tilt),
                       T_s=row.get('T_s', 0.0), T_p=row.get('T_p', 0.0), mode_s=pod[0].mode, mode_p=pod[1].mode)
        else:
            out.update(tail=act.stage['cmd'] if act.stage else 0.0, F_tail=act.stage['F'] if act.stage else 0.0,
                       T_pods=sum(act.pods.thrust), tilt=math.degrees(act.pods.theta))

        for name, alpha, stall in plant.fin_state():
            out['alpha_' + name] = math.degrees(alpha) if alpha is not None else None
            out['stalled_' + name] = alpha is not None and abs(alpha) > stall
            out['branch_' + name] = 'no flow' if alpha is None else ('stalled' if abs(alpha) > stall else 'attached')

        if wind is not None:
            out['u_g'], out['v_g'] = view.u_ground, view.v_ground

        if full:
            out['p'] = math.degrees(p)
            out['actuators'] = actuator_state(loop, vectored)
            out['restore_trim'] = yc._restore_trim

        return out

    t = -manual_s
    entry['manual'] = state(t, True)
    k = 0

    while t < t_end - 1e-9:
        if abs(t - 0.0) < 1e-9:
            loop.set_mode('acro')
            entry['acro'] = state(t, True)
        if from_acro and abs(t - t_stick) < 1e-9:
            loop.set_stick(stick)
            entry['stick'] = state(t, True)
        if abs(t - t_release) < 1e-9:
            loop.set_stick(0.0)
            entry['release'] = state(t, True)

        if hook is not None:
            hook(t, view, yc, allocator)

        loop.advance(DT)
        t = round(t + DT, 6)
        k += 1
        u, v, p, r = plant.nu
        seg = 'stick' if t_stick < t <= t_release + 1e-9 else ('release' if t > t_release else None)

        for i, (f, c) in enumerate(zip(pods, [math.cos(pod.tilt) for pod in act.act.alloc.pods] if vectored else [])):
            was, neg, pos = f.state, f.to_neg, f.to_pos
            f.update(c)

            if seg:
                d = swings[seg][i]
                d['into_rear'] += f.to_neg - neg
                d['back_forward'] += f.to_pos - pos
                d['rear_s'] += DT if was == -1 else 0.0         # the state held through this tick

        for i, (f, x) in enumerate(zip(tails, (act.stage['cmd'], act.stage['F']) if not vectored else ())):
            before = f.flips
            f.update(x)

            if seg:
                flips[seg][i] += f.flips - before

        if seg:
            n_seg[seg] += 1
            stalled = False

            for name, alpha, stall in plant.fin_state():
                if alpha is not None:
                    ratio = abs(alpha) / stall
                    fin_max[seg][name] = max(fin_max[seg].get(name, 0.0), ratio)
                    stalled = stalled or ratio > 1.0

            stalled_ticks[seg] += stalled

        trace['t'].append(t)
        trace['r'].append(-math.degrees(r))
        trace['r_meas'].append(math.degrees(plant.measured_rate()))
        trace['u'].append(u)
        trace['v'].append(-v)
        trace['heading'].append(-math.degrees(plant.psi))
        trace['integ'].append(yc.rc.integ)
        trace['torque'].append(loop.read_torque())
        trace['sp'].append(math.degrees(yc.rate_sp))

        if vectored:
            trace['tilt_s'].append(math.degrees(act.act.alloc.pods[0].tilt))
            trace['tilt_p'].append(math.degrees(act.act.alloc.pods[1].tilt))
        else:
            trace['tail'].append(act.stage['cmd'])
            trace['tilt'].append(math.degrees(act.pods.alloc.pods[0].tilt))

        if wind is not None:
            trace['u_g'].append(view.u_ground)

        if k % every == 0:
            series.append(state(t))

    entry['end'] = state(t, True)
    T, R, RM, U, INTEG = trace['t'], trace['r'], trace['r_meas'], trace['u'], trace['integ']

    def pick(series_, time):
        """The value at the step nearest `time` (_nearest); past the end by
        no more than forward_yaw_check's DURATION_TOL_S, the end's, as its
        mark_time() takes a mark; further past it, None."""
        if time > t_end + 1e-9:
            if time - t_end > fyc.DURATION_TOL_S + 1e-9:
                return None

            time = t_end

        i = _nearest(T, time)
        return None if i is None else series_[i]

    def mean(series_, a, b, closed=False):
        return _window_mean(T, series_, a, b, closed)

    def mark(seconds):
        """forward_yaw_check's mark_time() on the leg's clock: the release
        plus `seconds`, the end when past it by DURATION_TOL_S at most, else
        None."""
        when = t_release + seconds

        if when <= t_end + 1e-9:
            return when

        return t_end if when - t_end <= fyc.DURATION_TOL_S + 1e-9 else None

    def pod_count(seg):
        return sum(d['into_rear'] + d['back_forward'] for d in swings[seg])

    stick_idx = [i for i in range(len(T)) if t_stick < T[i] <= t_release + 1e-9] if from_acro else []
    last5 = [R[i] for i in stick_idx if T[i] > t_release - 5.0 + 1e-9]
    sgn = 1.0 if stick >= 0 else -1.0
    end_stick, end = entry['release'], entry['end']
    fins_end = {name: end['branch_' + name] for name in ('dorsal', 'ventral') if 'branch_' + name in end}
    fins_stick = {name: end_stick['branch_' + name] for name in ('dorsal', 'ventral') if 'branch_' + name in end_stick}
    marks = {s: mark(s) for s in MARKS_S}
    # the stick's 0.5 s moving mean of the measured rate, forward_yaw_check's
    # r_peak_smoothed_deg_s (its stick rows: onset <= t < release)
    peak_rows = [RM[i] for i in range(len(T)) if t_stick - 1e-9 <= T[i] < t_release - 1e-9] if from_acro else []
    n_win = int(round(PEAK_WINDOW_S / DT))
    smoothed = None

    if len(peak_rows) >= n_win:
        run = sum(peak_rows[:n_win])
        best = run

        for i in range(n_win, len(peak_rows)):
            run += peak_rows[i] - peak_rows[i - n_win]
            best = max(best, run) if sgn > 0 else min(best, run)

        smoothed = best / n_win

    m = {
        'r_mean_last5': sum(last5) / len(last5) if last5 else None,
        'r_peak': sgn * max(sgn * R[i] for i in stick_idx) if stick_idx else None,
        'integ_stick_end': end_stick['integ'], 'torque_stick_end': end_stick['torque'],
        'unalloc_stick_end': end_stick['unalloc'], 'sat_pos_stick_end': end_stick['sat_pos'],
        'sat_neg_stick_end': end_stick['sat_neg'],
        'heading_stick': end_stick['heading'] - entry['stick']['heading'] if from_acro else None,
        'heading_release': end['heading'] - end_stick['heading'],
        'r_rel_10': pick(R, t_release + 10.0), 'r_rel_60': pick(R, t_release + 60.0), 'r_rel_120': pick(R, t_end),
        'integ_rel_120': end['integ'],
        # the vehicle's own count (module docstring), and per kind
        'rev_stick': (pod_count('stick') if vectored else flips['stick'][0]) if from_acro else None,
        'rev_release': pod_count('release') if vectored else flips['release'][0],
        'pod_swings_stick': {n: dict(d) for n, d in zip(POD_NAMES, swings['stick'])} if vectored and from_acro else None,
        'pod_swings_release': {n: dict(d) for n, d in zip(POD_NAMES, swings['release'])} if vectored else None,
        'tail_flips_stick': None if vectored or not from_acro else flips['stick'][0],
        'tail_flips_release': None if vectored else flips['release'][0],
        'tail_thrust_signs_stick': None if vectored or not from_acro else flips['stick'][1],
        'tail_thrust_signs_release': None if vectored else flips['release'][1],
        'u_stick_start': entry['stick']['u'] if from_acro else None, 'u_stick_end': end_stick['u'],
        'u_release_end': end['u'],
        'fins_branch_stick_end': fins_stick, 'fins_branch_release_end': fins_end,
        'fin_max_alpha_ratio_stick': fin_max['stick'], 'fin_max_alpha_ratio_release': fin_max['release'],
        'stalled_fraction_stick': stalled_ticks['stick'] / max(1, n_seg['stick']) if from_acro else None,
        'stalled_fraction_release': stalled_ticks['release'] / max(1, n_seg['release']),
        'phi_stick_end': end_stick['phi'], 'v_stick_end': end_stick['v'],
        # as a 0b record has them (module docstring: windows and the measured rate)
        'r_peak_smoothed': smoothed,
        'r_mean1s_rel_10': None if marks[10.0] is None else mean(RM, marks[10.0] - MEAN_R_S, marks[10.0]),
        'r_mean1s_rel_60': None if marks[60.0] is None else mean(RM, marks[60.0] - MEAN_R_S, marks[60.0]),
        'r_mean1s_rel_end': mean(RM, t_end - MEAN_R_S, t_end),
        'integ_rel_10': None if marks[10.0] is None else pick(INTEG, marks[10.0]),
        'integ_rel_60': None if marks[60.0] is None else pick(INTEG, marks[60.0]),
        'u_mean05_stick_start': mean(U, t_stick, t_stick + MEAN_U_S) if from_acro else None,
        'u_mean05_stick_end': mean(U, t_release - MEAN_U_S, t_release) if from_acro else None,
        'u_mean05_release_end': mean(U, t_end - MEAN_U_S, t_end, closed=True),
    }
    setup = {'target': target, 'model': model, 'airframe': gains.get('SYS_AUTOSTART'), 'u0': u0, 'stick': stick,
             'throttle_asked': throttle, 'fraction': fraction, 'flown': flown, 'trim': trim, 'drag_N': drag,
             'full_thrust_N': full, 'quantize': quantize, 'hold_u': hold_u, 'noise': noise, 'seed': seed,
             'P': gains['AS_YAWRATE_P'], 'I': gains['AS_YAWRATE_I'], 'INT_LIM': gains['AS_YR_INT_LIM'],
             'FF': gains['AS_YAWRATE_FF'], 'MAX': gains['AS_YAWRATE_MAX'],
             'sp': math.degrees(ctl.yaw_rate_setpoint(stick, gains['MAN_DEADZONE'], math.radians(gains['AS_YAWRATE_MAX']))),
             'times': (-manual_s, 0.0, t_stick, t_release, t_end), 'vectored': vectored,
             'tail_band_N': None if vectored else TAIL_BAND * act.chain.full_thrust,
             'tail_band_Nm': None if vectored else TAIL_BAND * act.chain.K,
             'start': start, 'frame': 'FRD', 'v0': v0, 'r0': r0, 'p0': p0, 'phi0': phi0, 'params': dict(gains),
             'entry_given': {'pods0': pods0 is not None, 'motors0': motors0 is not None, 'integ0': integ0,
                             'sat0': sat0, 'gyro_settled': gyro_settled},
             'policy': {'hook': hook is not None, 'restore': restore, 'wind': wind, 'tilt_range': tilt_range},
             # (the cycle's time, s since t = 0; the integrator put back)
             'restore_events': [(round(-manual_s + (t_us - us0 - ctl.DT_US) / 1e6, 6), value)
                                for t_us, value in yc.restore_events]}
    return {'metrics': m, 'entry': entry, 'series': series, 'trace': trace, 'setup': setup}


def _nearest(T, time):
    """The index of the step of T nearest `time` (the earlier on a tie), or
    None when none is within one step (DT) of it."""
    i = bisect.bisect_left(T, time)
    js = [j for j in (i - 1, i) if 0 <= j < len(T)]

    if not js:
        return None

    i = min(js, key=lambda j: abs(T[j] - time))
    return i if abs(T[i] - time) <= DT + 1e-9 else None


def _window_mean(T, X, a, b, closed=False):
    """Mean of X over the steps a <= t < b (t <= b when closed), as
    forward_yaw_check's windows; None when empty or not finite."""
    lo = bisect.bisect_left(T, a - 1e-9)
    hi = bisect.bisect_right(T, b + 1e-9) if closed else bisect.bisect_left(T, b - 1e-9)
    xs = X[lo:hi]
    return sum(xs) / len(xs) if len(xs) and all(ctl.isfinite(x) for x in xs) else None


def trace_at(leg, key, time):
    """A leg's trace[key] at the step nearest `time` [s since t = 0], None
    when no step is within 4 ms of it (past the end, before the start)."""
    i = _nearest(leg['trace']['t'], time)
    return None if i is None else leg['trace'][key][i]


def trace_mean(leg, key, a, b, closed=False):
    """A leg's trace[key] averaged over a <= t < b [s since t = 0] (t <= b
    when closed), the windows forward_yaw_check averages a record over (its
    r_mean1s at a mark w: (w - 1, w); u_start: (on, on + 0.5); release u_end:
    (end - 0.5, end], closed). Use 'r_meas' against the record's gyro."""
    return _window_mean(leg['trace']['t'], leg['trace'][key], a, b, closed)


# ---------------------------------------------------------------------------
# A leg against a record: the record's own sample times, counts and fins
# ---------------------------------------------------------------------------

COMMANDS = ('torque', 'integ', 'sp', 'tilt_s', 'tilt_p', 'tail', 'tilt')


def command_at(leg, key, times_s):
    """The model's `key` as a log topic stamped at each of `times_s` [s since
    the leg's t = 0] holds it, zero-order: a controller or allocator output
    (COMMANDS) from the latest cycle at or before that time (a trace row
    ending at t carries the cycle at t - DT), a plant quantity from the latest
    step that ended at or before it. None before the trace begins."""
    T, X = leg['trace']['t'], leg['trace'][key]
    lag = DT if key in COMMANDS else 0.0
    out = []

    for ts in times_s:
        i = bisect.bisect_right(T, ts + lag + 1e-9) - 1
        out.append(X[i] if i >= 0 else None)

    return out


def reversals_at(leg, times_s, a_s, b_s, source=None):
    """The model's reversals between a_s and b_s [s since t = 0], counted as
    forward_yaw_check counts a record: its own pod_reversals() and
    sign_reversals() on the model's commands sampled at `times_s` (the log's
    actuator_servos or actuator_motors times, s since its Acro switch;
    command_at). source: the Cloudship's counted command, 'tail' (default,
    actuator_motors control[2]) or 'torque' (vehicle_torque_setpoint, its
    sign and band the tail's: the tail command is clamp(torque),
    cloudship_actuation.py:201-202). Returns the record's form: vectored
    {'stbd': {into_rear, back_forward, rear_s}, 'port': {...}, 'flips'};
    Cloudship {'tilt_min_deg', 'tilt_max_deg', 'tail_flips', 'flips'}."""
    t_us = [int(round(ts * 1e6)) for ts in times_s]
    a, b = int(round(a_s * 1e6)), int(round(b_s * 1e6))

    if leg['setup']['vectored']:
        out, total = {}, 0

        for name, key in zip(POD_NAMES, ('tilt_s', 'tilt_p')):
            values = command_at(leg, key, times_s)
            keep = [(ti, x) for ti, x in zip(t_us, values) if x is not None]
            into, back, rear = fyc.pod_reversals([ti for ti, _ in keep], [x for _, x in keep], a, b)
            out[name] = {'into_rear': int(into), 'back_forward': int(back), 'rear_s': round(float(rear), 3)}
            total += into + back

        out['flips'] = int(total)
        return out

    key = source or 'tail'
    values = command_at(leg, key, times_s)
    keep = [(ti, x) for ti, x in zip(t_us, values) if x is not None]
    tilts = [x for ti, x in zip(t_us, command_at(leg, 'tilt', times_s)) if x is not None and a <= ti < b]
    n = fyc.sign_reversals([ti for ti, _ in keep], [x for _, x in keep], a, b)
    return {'tilt_min_deg': min(tilts) if tilts else None, 'tilt_max_deg': max(tilts) if tilts else None,
            'tail_flips': int(n), 'flips': int(n)}


def fin_names(model):
    """The vertical fins of gz `model` in file order, as the 0b record
    numbers them (gz_model.vertical_fins: its alpha_fin<n>_deg, n from 1),
    named as the model names them (planar.RollPlanarPlant): 'dorsal' when
    the fin's cp is above the family's centre of mass, else 'ventral'."""
    _, com = gz_model.centre_of_mass(gz_model.family(model))
    return ['dorsal' if f['cp'][2] > com[2] else 'ventral' for f in gz_model.vertical_fins(model)]


@functools.lru_cache(maxsize=None)
def _vertical_fins(target):
    import planar
    model = cl.ALL_TARGETS[target][0]
    return tuple(f for f in planar.RollPlanarPlant(target, model=model).fins if f['vertical'])


def fin_alphas(target, u, v, p, r, phi):
    """{fin name: angle of attack deg, or None for no flow} of the vertical
    fins at a state, as the lift-drag plugin computes it
    (planar.lift_drag_alpha: signed by the fin's own upward vector, folded
    into +-90 deg, None where the plugin returns early: the flow at the cp
    from behind or below 0.01 m/s) and as the leg reports it
    (RollPlanarPlant.fin_state). FRD inputs at the link origin: u, v m/s,
    p, r deg/s, phi deg; the body pitch rate a heading change has while
    rolled, q = r tan(phi), as the plant has it."""
    import planar
    v_l, p_l, r_l, phi_l = -v, math.radians(p), -math.radians(r), math.radians(phi)
    w = (p_l, r_l * math.tan(phi_l), r_l)
    out = {}

    for f in _vertical_fins(target):
        vcp = planar._cross(w, f['cp'])
        alpha = planar.lift_drag_alpha((u + vcp[0], v_l + vcp[1], vcp[2]), f)
        out[f['name']] = None if alpha is None else math.degrees(alpha)

    return out


# ---------------------------------------------------------------------------
# Replay inputs: a 0b record's leg, or a model leg's own release
# ---------------------------------------------------------------------------

def infer_pod_mode(demand, entering_from_above=True):
    """The AirshipPod mode a pod has when its demand magnitude has been
    `demand` for a while (AirshipPod.cpp:49-70: engage above 0.02, release
    below 0.01, holding between them only for a pod that was engaged).
    entering_from_above: the demand came down into the band from above it,
    as the 0b check's throttle steps down from its acceleration throttle."""
    if demand > ctl.AirshipPod.K_STEER_ENGAGE:
        return ctl.STEERING

    if demand > ctl.AirshipPod.K_STEER_RELEASE and entering_from_above:
        return ctl.HOLDING

    return ctl.RELEASED


def replay_kwargs(leg, model, start='acro', gains=None):
    """run_leg's keyword arguments that replay one leg of a 0b record
    (forward_yaw_check's summary: doc['legs'][i], doc['model']) from its Acro
    switch ('acro') or its stick's release ('release'):

      target 'gz_' + model; params the record's gains (record_gains.GAINS,
      each the shortest decimal of its float32, record_gains.f32) unless
      `gains` is given; throttle the recorded mean thrust setpoint, exact
      (quantize False); the record's timeline (manual_s 0, centred_s, stick_s,
      release_s from replay.stick and replay.end_s); the entry state of that
      instant (u, v FRD, r, p deg/s, roll deg); gyro_settled True; and the
      actuators as recorded (pods' tilts from actuator_servos, motors from
      actuator_motors), each pod's mode inferred from its demand
      (infer_pod_mode: the collective throttle and the recorded torque
      setpoint of that instant shared as the allocator shares them), target =
      tilt, the gz joint settled. From the Acro switch the integrator starts
      cleared, as the firmware clears it when the loop opens in Manual (the
      record's replay.integrator, the first rate_ctrl_status sample after
      the switch, is a check of that, not an input). 'release' adds integ0
      (the rate_ctrl_status sample at or before the release, up to 200 ms
      old: release.entry.integrator_age_s) and sat0 (the flags at the
      stick's end).

    Raises on any value the record does not carry (None): a replay does not
    start from a guess. Checks the record's preconditions (its verdicts,
    SYS_AUTOSTART, the allocator parameters against the airframe files) are
    the caller's."""
    import record_gains as rg
    target = 'gz_' + model
    _, airframe = cl.ALL_TARGETS[target]
    vectored = airframe == 'vectored'
    rp = leg['replay']

    if start not in ('acro', 'release'):
        raise ValueError('start %r: acro or release' % (start,))

    state = rp['state'] if start == 'acro' else rp['state_at_release']
    acts = rp['actuators'] if start == 'acro' else rp['actuators_at_release']
    (_, _), (on, stick), (off, _) = rp['stick']
    need = {'u': state.get('u'), 'v': state.get('v'), 'p_deg_s': state.get('p_deg_s'),
            'r_deg_s': state.get('r_deg_s'), 'roll_deg': state.get('roll_deg'), 'torque_z': state.get('torque_z'),
            'throttle': rp.get('throttle'), 'stick': stick, 'on': on, 'off': off, 'end_s': rp.get('end_s'),
            'motor1': acts.get('motor1'), 'motor2': acts.get('motor2')}

    if not vectored:
        need['motor3'] = acts.get('motor3')         # the tail (CA_AIRSHIP_TAIL: the motor after the pods)
    tilt_keys = ('tilt_stbd_deg', 'tilt_port_deg') if vectored else ('tilt_deg', 'tilt_deg')
    need.update({k: acts.get(k) for k in tilt_keys})

    if gains is None:
        need.update({k: rp['params'].get(k) for k in rg.GAINS})

    if start == 'release':
        need['integrator_at_release'] = rp.get('integrator_at_release')
        need['flag_pos_end'] = leg['stick_phase'].get('flag_pos_end')
        need['flag_neg_end'] = leg['stick_phase'].get('flag_neg_end')

    missing = [k for k, x in need.items() if x is None]

    if missing:
        raise LookupError('the record leaves %s unread: no replay from a guess' % ', '.join(missing))

    c, tau = float(rp['throttle']), float(state['torque_z'])
    demand = [abs(c - tau), abs(c + tau)] if vectored else [abs(c), abs(c)]
    pods0 = [{'tilt': float(acts[key]), 'mode': infer_pod_mode(d)} for key, d in zip(tilt_keys, demand)]
    motors0 = [float(acts['motor1']), float(acts['motor2'])] + ([] if vectored else [float(acts['motor3'])])
    kw = dict(target=target, u0=float(state['u']), stick=float(stick), throttle=c, quantize=False,
              params=dict(gains) if gains is not None else {k: rg.f32(rp['params'][k]) for k in rg.GAINS},
              v0=float(state['v']), r0=float(state['r_deg_s']), p0=float(state['p_deg_s']),
              phi0=float(state['roll_deg']), pods0=pods0, motors0=motors0, gyro_settled=True, start=start)

    if start == 'acro':
        kw.update(manual_s=0.0, centred_s=float(on), stick_s=float(off) - float(on),
                  release_s=float(rp['end_s']) - float(off))
    else:
        kw.update(release_s=float(rp['end_s']) - float(off), integ0=float(rp['integrator_at_release']),
                  sat0=(bool(leg['stick_phase']['flag_pos_end']), bool(leg['stick_phase']['flag_neg_end'])))

    return kw


def release_kwargs(leg, **over):
    """run_leg's keyword arguments that start a leg at another model leg's
    release, from that leg's own state there (entry['release']: the plant,
    the actuators, the integrator and the flags); `over` replaces any. The
    plant's heading starts again at 0."""
    s, e = leg['setup'], leg['entry']['release']
    policy = s['policy']
    motors = list(e['actuators']['motors']) + ([] if s['vectored'] else [e['actuators']['tail']])
    kw = dict(target=s['target'], u0=e['u'], stick=s['stick'], throttle=s['fraction'], quantize=s['quantize'],
              params=dict(s['params']), hold_u=s['hold_u'], v0=e['v'], r0=e['r'], p0=e['p'], phi0=e['phi'],
              start='release', pods0=e['actuators']['pods'], motors0=motors, integ0=e['integ'],
              sat0=(bool(e['sat_pos']), bool(e['sat_neg'])), gyro_settled=True,
              release_s=s['times'][4] - s['times'][3], restore=policy['restore'],
              restore_trim0=e['restore_trim'] if policy['restore'] else None, wind=policy['wind'],
              tilt_range=policy['tilt_range'])
    kw.update(over)
    return kw


def write_csv(leg, path):
    rows = leg['series']
    keys = list(rows[0].keys())

    with open(path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=keys, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)


def fmt(x, spec='%+.2f'):
    """A number for the text, '-' where it is None (a mark past the end, a
    stick segment a release start does not have)."""
    return '-' if x is None else spec % x


def reversal_text(leg, seg):
    """One segment's reversals in words, by kind."""
    m = leg['metrics']

    if leg['setup']['vectored']:
        if m['pod_swings_' + seg] is None:
            return '-'

        return 'pod swings (cos(tilt) past +-%g) %s' % (FIN_BAND, ', '.join(
            '%s into the rear %d, back forward %d (%.1f s in the rear)' % (n, d['into_rear'], d['back_forward'], d['rear_s'])
            for n, d in m['pod_swings_' + seg].items()))

    if m['tail_flips_' + seg] is None:
        return '-'

    return 'tail flips %d outside +-%g (%.3f N, %.3f N m), %d sign changes of its thrust' % (
        m['tail_flips_' + seg], TAIL_BAND, leg['setup']['tail_band_N'], leg['setup']['tail_band_Nm'],
        m['tail_thrust_signs_' + seg])


def describe(leg):
    """Text lines for one leg."""
    s, m = leg['setup'], leg['metrics']
    out = ['%s (%s), u0 %.1f m/s, stick %+.2f (setpoint %.2f deg/s), throttle %s -> fraction %.4f flown %.4f '
           '(trim %.4f: drag %.3f N / full %.2f N)%s; P %g I %g LIM %g FF %g'
           % (s['target'], s['airframe'], s['u0'], s['stick'], s['sp'], s['throttle_asked'], s['fraction'], s['flown'],
              s['trim'], s['drag_N'], s['full_thrust_N'], '; u held at %g' % s['hold_u'] if s['hold_u'] is not None else '',
              s['P'], s['I'], s['INT_LIM'], s['FF'])]
    out.append('  stick: r mean last 5 s %s, peak %s deg/s; at its end integ %+.4f, torque %+.4f, unallocated %+g, '
               'sat +%d/-%d; heading %s deg in the stick'
               % (fmt(m['r_mean_last5']), fmt(m['r_peak']), m['integ_stick_end'], m['torque_stick_end'],
                  m['unalloc_stick_end'], m['sat_pos_stick_end'], m['sat_neg_stick_end'],
                  fmt(m['heading_stick'], '%+.1f')))
    out.append('  release: heading %+.1f deg; r at +10/+60/+120 s %s / %s / %s deg/s; integ at +120 s %+.4f'
               % (m['heading_release'], fmt(m['r_rel_10']), fmt(m['r_rel_60']), fmt(m['r_rel_120']),
                  m['integ_rel_120']))
    out.append('  stick: %s; release: %s' % (reversal_text(leg, 'stick'), reversal_text(leg, 'release')))
    out.append('  u %s -> %s -> %.2f m/s (stick start, stick end, release end)'
               % (fmt(m['u_stick_start'], '%.2f'), fmt(m['u_stick_end'], '%.2f'), m['u_release_end']))
    out.append('  fins: max |alpha|/alpha_stall stick %s, release %s; stalled %s %% of the stick; at stick end %s, '
               'at release end %s'
               % (', '.join('%s %.2f' % kv for kv in sorted(m['fin_max_alpha_ratio_stick'].items())) or '-',
                  ', '.join('%s %.2f' % kv for kv in sorted(m['fin_max_alpha_ratio_release'].items())) or '-',
                  fmt(None if m['stalled_fraction_stick'] is None else 100 * m['stalled_fraction_stick'], '%.0f'),
                  ', '.join('%s %s' % kv for kv in sorted(m['fins_branch_stick_end'].items())) or '-',
                  ', '.join('%s %s' % kv for kv in sorted(m['fins_branch_release_end'].items())) or '-'))
    out.append('  as a 0b record has them: r peak (0.5 s moving mean) %s deg/s; r 1 s means at +10/+60/end %s / %s / %s '
               'deg/s; integ at +10/+60 %s / %s; u 0.5 s means %s -> %s -> %s m/s (stick start, stick end, release end)'
               % (fmt(m['r_peak_smoothed']), fmt(m['r_mean1s_rel_10']), fmt(m['r_mean1s_rel_60']),
                  fmt(m['r_mean1s_rel_end']), fmt(m['integ_rel_10'], '%+.4f'), fmt(m['integ_rel_60'], '%+.4f'),
                  fmt(m['u_mean05_stick_start'], '%.2f'), fmt(m['u_mean05_stick_end'], '%.2f'),
                  fmt(m['u_mean05_release_end'], '%.2f')))
    return out


def main():
    ap = argparse.ArgumentParser(description='one forward leg on the yaw model')
    ap.add_argument('target', choices=sorted(cl.ALL_TARGETS))
    ap.add_argument('--u0', type=float, required=True)
    ap.add_argument('--stick', type=float, required=True)
    ap.add_argument('--throttle', default='trim', help="'trim' or a thrust fraction 0..1")
    ap.add_argument('--param', action='append', default=[], help='NAME=VALUE over the HEAD gains')
    ap.add_argument('--hold-u', action='store_true', help='hold the surge at u0 (ablation)')
    ap.add_argument('--exact-throttle', action='store_true', help='fly the fraction exactly (quantize False)')
    ap.add_argument('--truncate', action='store_true', help='sitl_link given the fraction itself (quantize True)')
    ap.add_argument('--noise', action='store_true')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--csv', default=None)
    args = ap.parse_args()
    params = {}

    for item in args.param:
        name, value = item.split('=', 1)
        params[name] = float(value)

    quantize = False if args.exact_throttle else (True if args.truncate else 'midpoint')
    leg = run_leg(args.target, args.u0, args.stick, args.throttle, params, args.noise, args.seed,
                  args.u0 if args.hold_u else None, quantize)

    for line in describe(leg):
        print(line)

    if args.csv:
        write_csv(leg, args.csv)
        print('written', args.csv)


if __name__ == '__main__':
    main()
