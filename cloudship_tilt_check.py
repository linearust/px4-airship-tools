#!/usr/bin/env python3
"""(The 14/14 baseline of 2026-09-18 belongs to the plant with hull drag,
which since 2026-09-23 is the gz_cloudship_drag target; on the undamped
variants the burst inversion runs with zero drag, which is exact there.)
Does the Cloudship's one tilt actually vector its thrust?

    GZ_TARGET=gz_cloudship ./gz_run.sh ./cloudship_tilt_check.py

This is the vehicle's defining mechanism and the one no simulator has ever
run. CA_AIRSHIP_GRP 0 puts both thrusters on one collective, CA_AIRSHIP_TAIL 1
gives the yaw to the tail, and the tilt is what turns forward thrust into
climb. The SITL gate exercises the yaw axis; nothing else touches this one, and
a model whose rod turns without carrying the props, or whose servo sign is
inverted against the allocator's, would pass every other check here.

  1. the rod reaches the angles the airframe says it can
  2. the thrust turns with it, in magnitude as well as direction
  3. the two props counter-rotate, so a collective burst does not roll the hull

Getting (2) right took four attempts and every wrong one produced a correct
ANGLE. That is why the magnitude is reported and asserted: it is what caught
each of them.

  - Body-frame coefficients were applied to NED velocities. Sway drag is 35x
    surge, so a hull running three metres a second on a heading that is not
    north had 29 N of sway drag credited to it and the level burst recovered
    16.13 N where the pods deliver 7.95. Rotating through vehicle_attitude.q
    first fixed it, and made the expected answer the commanded tilt itself
    rather than a difference between tilts.
  - The drag term was evaluated at the mean speed instead of averaged over the
    window. `F*span = m*dv + integral of c*v|v|`, and v|v| is convex, so the
    two differ - by 27% on both vertical bursts and by nothing on the level
    one, which is the signature: vertically the hull reaches terminal inside
    the window, m*dv goes to nothing, and the whole thrust comes out of that
    integral.
  - A coasting baseline was subtracted AND the drag added back, counting it
    twice; and the elapsed time was the nominal window rather than the
    measured one.

(3) used to be a roll-rate threshold, and it could not see the property it
named. At +-90 the propellers' reaction couple is not on the roll axis at all -
the pair is vertical, so a co-rotating pair makes a pure YAW couple - and even
at tilt 0 the couple is 0.13 N m, a quarter of the threshold it was judged
against, sampled below Nyquist for the 0.61 Hz roll mode. The directions are
declared in the model, so they are asserted from the model; the roll reading is
kept under the name of what it can actually catch, which is a dead or
mismatched prop at +-90.
"""
import math
import sys
import time

import gz_model
from report import note, summary, verdict
from sitl_link import (MODEL_NAME, actuator_hold, actuator_release, commander,
                       field, listener, vector)
# The rod's arrival tolerance is deliberately the one drive_to polls to:
# two constants governing one event is two chances to disagree.
from tilt_range_check import TOLERANCE_DEG, drive_to, near

ROD = 'thruster_rod'
PODS = ('prop_starboard', 'prop_port')
TILT_FUNCTION = 201          # SIM_GZ_SV_FUNC1
MOTOR_FUNCTIONS = (101, 102)  # starboard, port
BURST_VALUE = 0.25           # of full thrust each, linear through THR_MDL_FAC
BURST_S = 3.0
HOLD_S = 25                  # actuator_test timeout, shared by every function
VECTOR_TOL_DEG = 8.0
ROLL_TOL_DPS = 3.0
THRUST_TOL = 0.20            # of the pods' own full-command thrust
MIN_CLIMB_M = 6.0            # height gained before pointing the thrust down


def plant():
    """Mass, the three drag coefficients and one pod's full thrust.

    Every number this check turns an acceleration into a force with belongs to
    the model, so every one of them is read from it.
    """
    model = gz_model.merged(MODEL_NAME)          # the family, see cloudship_first_light
    thrust = gz_model.full_thrust(model, PODS[0])
    return (gz_model.centre_of_mass(gz_model.family(MODEL_NAME))[0],
            gz_model.translational_drag(model),
            thrust[0] if thrust else 0.0,
            gz_model.turning(model))


def state():
    """(velocity NED, height gained, body roll rate deg/s, attitude) or None.

    Every field is fatal. The roll rate used to fall back to exactly 0.0 when
    vehicle_angular_velocity did not read, which is the reading the roll
    verdict is judged on: a single timed-out listener produced a green '롤 최대
    0.00 deg/s' indistinguishable from a genuinely quiet hull.
    """
    pos = listener('vehicle_local_position')
    v = [field(pos, axis) for axis in ('vx', 'vy', 'vz')]
    z = field(pos, 'z')
    xyz = vector(listener('vehicle_angular_velocity'), 'xyz')
    q = vector(listener('vehicle_attitude'), 'q')

    if any(n is None for n in v) or z is None or xyz is None or q is None or len(q) != 4:
        return None

    return v, -z, math.degrees(xyz[0]), q


def to_body(q, v):
    """NED into body FRD, through the attitude quaternion.

    This has to happen before any drag coefficient is applied. Sway drag is
    35 times surge, so mistaking one axis for the other is not a small error.

    PX4's vehicle_attitude.q takes body to NED, so this is its transpose.
    """
    w, x, y, z = q
    rows = ((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
            (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
            (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)))
    return [sum(rows[j][i] * v[j] for j in range(3)) for i in range(3)]


def hold(tilt_value, motors=False):
    """Re-assert the rod and, optionally, both thrusters in one test session.

    ActuatorTest keeps one override per output but a single timeout, reset by
    whichever command arrives last, and clears every override the moment it
    leaves test mode. So they have to be issued together and often enough that
    the session never lapses between them.
    """
    pairs = [(TILT_FUNCTION, tilt_value)]

    if motors:
        pairs += [(f, BURST_VALUE) for f in MOTOR_FUNCTIONS]

    actuator_hold(pairs, timeout=HOLD_S)


def burst(tilt_value, mass, drag):
    """Hold the rod, run both thrusters, and return the thrust in the body frame.

    Not the acceleration. Heave drag is 35 times surge here, so a burst still
    accelerating horizontally has already gone terminal vertically; the thrust
    is what the hull felt plus what the air took, at the speed it was doing
    while it took it.

    The drag put back is the hull's only. The propellers' own air drag
    (rotorDragCoefficient, zero on these pods) and the rod's are not modelled
    by anything here, so a few percent of the residual belongs to terms this
    inversion does not carry.
    """
    hold(tilt_value)
    time.sleep(6)                       # let the rod get there

    hold(tilt_value, motors=True)
    before = state()
    began = time.time()

    if before is None:
        return None

    roll_peak = 0.0
    samples = []
    deadline = began + BURST_S

    while time.time() < deadline:
        hold(tilt_value, motors=True)
        now = state()

        if now is None:
            actuator_release(MOTOR_FUNCTIONS)
            return None

        roll_peak = max(roll_peak, abs(now[2]))
        samples.append(to_body(now[3], now[0]))

    after = state()
    # Timed, not assumed. One pass through the loop above is four subprocess
    # calls, so the window overruns BURST_S by most of a pass, and dividing by
    # the nominal figure inflated the acceleration by a third.
    span = time.time() - began
    actuator_release(MOTOR_FUNCTIONS)

    if after is None or span <= 0.0:
        return None

    v0, v1 = to_body(before[3], before[0]), to_body(after[3], after[0])

    # Momentum over the window: F*span = m*dv + integral of the drag, so the
    # drag term is the TIME AVERAGE of c*v|v|, not c*v|v| at the average speed.
    # Equal weights over the samples plus the two endpoints, which is a
    # trapezoid with its ends over-counted by half a step - a percent or two at
    # this sample count, and in the same direction at every tilt.
    accel = [(a - b) / span for a, b in zip(v1, v0)]
    trail = [v0] + samples + [v1]
    quad = [sum(row[i] * abs(row[i]) for row in trail) / len(trail) for i in range(3)]
    thrust = [mass * a + c * q for a, c, q in zip(accel, drag, quad)]

    # In the body frame the answer is the commanded angle itself: the rod is at
    # tilt, the thrust is along it, and the hull's own 2.6 degree nose-down hang
    # is not in the way because it is not the frame this is measured in.
    elevation = math.degrees(math.atan2(-thrust[2], thrust[0]))
    return (elevation, math.sqrt(sum(t * t for t in thrust)), roll_peak,
            before[1], thrust)


def main():
    if not commander('status'):
        note('px4 에 붙지 못했다')
        return 3

    mass, drag, full, turning = plant()
    want_thrust = 2.0 * BURST_VALUE * full

    # 3. the two props counter-rotate. Declared in the model, asserted from the
    #    model: at +-90 their couple leaves the roll axis entirely, so nothing
    #    measured on the hull can see them co-rotate where it matters.
    directions = [turning.get(link) for link in PODS]
    verdict('두 프로펠러가 반대로 돈다',
            all(directions) and directions[0] != directions[1],
            '%s: %s, %s: %s' % (PODS[0], directions[0], PODS[1], directions[1]))

    # 1. the rod reaches both ends and the middle. Centre last, so it is
    #    reached from an end stop rather than from the rest position a tilt
    #    joint sits at anyway.
    reached = []

    for want in (90.0, -90.0, 0.0):
        got, owed = drive_to(ROD, TILT_FUNCTION, want / 90.0, want)
        ok = got is not None and near(got, want) <= TOLERANCE_DEG
        reached.append(ok)
        verdict('틸트 %+.0f deg' % want, ok,
                '실제 %s%s' % ('읽지 못함' if got is None else '%+.1f deg' % got,
                              '' if owed else ' (명령 전부터 그 자세였다)'))

    # A rod that cannot be put where it is asked makes every thrust direction
    # below a measurement of some other angle, so stop rather than report them.
    if not all(reached):
        note('  로드가 명령한 각도에 가지 못했다. 추력 방향 측정은 의미가 없어 건너뛴다')
        actuator_release([TILT_FUNCTION])
        return summary()

    # 2. and the thrust turns with it. +90 first: it climbs, which is what
    #    buys the clearance the -90 burst needs on top of the spawn height.
    elevation = {}

    for tilt in (90.0, 0.0, -90.0):
        if tilt < 0:
            here = state()

            if here is None or here[1] < MIN_CLIMB_M:
                verdict('틸트 %+.0f 에서 추력 방향' % tilt, False,
                        '상승 %s m, 아래로 밀기에는 낮다'
                        % ('읽기실패' if here is None else '%.1f' % here[1]))
                continue

        got = burst(tilt / 90.0, mass, drag)

        if got is None:
            verdict('틸트 %+.0f 에서 추력 방향' % tilt, False, '속도나 자세를 읽지 못했다')
            continue

        elevation[tilt], speed, roll_peak, climb, parts = got
        note('    틸트 %+5.0f: 상승각 %+6.1f deg, 추력 %.3f N '
             '(동체 %+.2f, %+.2f, %+.2f), 롤 최대 %.2f deg/s, 상승 %.1f m'
             % (tilt, elevation[tilt], speed, parts[0], parts[1], parts[2],
                roll_peak, climb))
        verdict('틸트 %+.0f 에서 추력이 로드를 따른다' % tilt,
                near(elevation[tilt], tilt) <= VECTOR_TOL_DEG,
                '동체 기준 상승각 %+.1f deg, 명령 %+.0f deg' % (elevation[tilt], tilt))
        verdict('틸트 %+.0f 에서 추력 크기' % tilt,
                abs(speed - want_thrust) <= THRUST_TOL * want_thrust,
                '%.3f N, 모델의 %.3f N 대비 %+.0f %%'
                % (speed, want_thrust, 100.0 * (speed / want_thrust - 1.0)))

        # What the roll rate CAN catch: a dead or mismatched thruster. At +-90
        # the two thrusts are vertical at y = +-0.45 m, so one of them missing
        # is 1.8 N m of roll - 11 deg/s - against this 3.0 deg/s threshold.
        if abs(tilt) > 45.0:
            verdict('틸트 %+.0f 에서 두 추진기의 추력이 같다' % tilt,
                    roll_peak <= ROLL_TOL_DPS,
                    '버스트 중 롤 각속도 최대 %.2f deg/s' % roll_peak)

    for high, low in ((90.0, 0.0), (0.0, -90.0)):
        if high not in elevation or low not in elevation:
            continue

        turned = elevation[high] - elevation[low]
        verdict('틸트 %+.0f -> %+.0f 가 추력을 90 도 돌린다' % (low, high),
                abs(turned - 90.0) <= VECTOR_TOL_DEG,
                '추력이 %+.1f deg 돌았다 (%.1f -> %.1f)' % (turned, elevation[low], elevation[high]))

    actuator_release([TILT_FUNCTION])
    return summary()


if __name__ == '__main__':
    sys.exit(main())
