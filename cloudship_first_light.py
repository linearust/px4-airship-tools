#!/usr/bin/env python3
"""First light for the gz Cloudship. Run it through gz_run.sh.

    GZ_TARGET=gz_cloudship ./gz_run.sh ./cloudship_first_light.py

This airframe is the collective-thrust, tilting-rod, reversible-tail airship
that the flight side already flies as 2507 and that no simulator has ever
covered: the posix 2507 is upstream's gazebo-classic one, on the older
CA_AIRFRAME 9 rotor geometry, and gazebo-classic is not installed here. So
these are the questions a first run has to answer before any of the yaw-loop
checks mean anything, in the order a failure would make the rest meaningless:

  1. the model is in the world at all, under the name PX4 spawns it with
  2. it floats: neutral buoyancy is a property of the collision volumes and
     the declared mass, and a model that quietly sinks invalidates everything
  3. the allocator recognised the airframe and claimed its four outputs
  4. the tail can actually reverse, and by as much as the model says. It is
     the vehicle's only yaw authority
  5. a yaw stick turns the hull, and turns it the way it was asked to

Three things this check used to get wrong, all of them the same mistake in
different clothes - a reading taken where the quantity was not.

The buoyancy drift was a sim-time displacement divided by a wall-clock window.
poses() parses the simulator's stamps and the old code threw them away, so on a
sim running at 0.6x the implied imbalance came out 0.36x too small. It now
divides by the stamp span and refuses the measurement if that span is not close
to the window it asked for.

The tail's reverse was commanded once and read once after a ten second sleep.
actuator_test's override lapses on the simulator's clock while the sleep runs
on this machine's, so a tail that pushed and was then let go read exactly like
one that never pushed. It is now re-issued every pass and the extreme over the
window is the statistic.

And nothing checked how HARD the tail reverses. TAIL_YAW_MIN was a flat
10 deg/s, which is 4.6% of nominal: the reverse coefficient could be wrong by a
factor of twenty and still pass. Both floors are now derived from the model's
own thrust curves, forward and reverse, so the check scales with the vehicle it
is pointed at.
"""
import math
import re
import sys
import time

import gz_link
import gz_model
from report import note, summary, verdict
from sitl_link import (MODEL, MODEL_NAME, Sticks, actuator_hold, actuator_release,
                       angular_rate, commander, field, listener, px4, settle_rate,
                       vector, wait_for_ekf)

SETTLE_S = 20.0              # let the spawn transient go before measuring
WINDOW_S = 20.0              # and then measure over this long
CLOCK_TOL = 0.25             # how far the sim's stamps may drift from WINDOW_S
BUOYANCY_TOL = 0.01          # implied imbalance, as a fraction of the weight
MIN_HEIGHT_M = 5.0           # a hull below this is resting on the ground, not floating
TAIL_FUNCTION = 103          # SIM_GZ_EC_FUNC3
TAIL_INDEX = TAIL_FUNCTION - 101
TAIL_LINK = 'prop_tail'
TAIL_SPIN_S = 10.0           # long enough for the hull to reverse its turn
TAIL_SPIN_FREE_S = 2.0       # on an undamped plant: 2 s of 4.48 N m on 5.8 kg m^2 is already 88 deg/s
TAIL_YAW_FRACTION = 0.6      # of the terminal rate the model's thrust implies
YAW_STICK = 0.5


def hull():
    """Everything this check needs to know about the plant, from the model.

    Read rather than restated. The drift tolerance, the tail's floors and the
    weight are all properties of the vehicle, and a copy of one of them is how
    a tolerance silently stops describing the vehicle it is applied to.
    """
    # The family merged, not the one file: a physics variant carries no motor
    # plugin and no mass of its own, and reading it alone made the tail read
    # as absent and the vehicle as weightless.
    model = gz_model.merged(MODEL_NAME)
    terms = gz_model.hydrodynamics(model)
    added = gz_model.added_mass(model)
    family = gz_model.family(MODEL_NAME)
    mass, com = gz_model.centre_of_mass(family)
    return {
        'mass': mass,
        # None on a plant with no drag block: the physics variants without
        # _drag have nothing that bounds a drift or a spin, and the two
        # verdicts that used to divide by these take a different route there.
        'drag_z': abs(terms['zWabsW']) if 'zWabsW' in terms else None,
        'drag_yaw': abs(terms['nRabsR']) if 'nRabsR' in terms else None,
        'added_z': added.get('zz', 0.0),
        'izz': gz_model.yaw_inertia(family) + added.get('rr', 0.0),
        'thrust': gz_model.full_thrust(model, TAIL_LINK),
        'duty': gz_model.duty_range(model, TAIL_LINK),
        # The yaw lever, taken from the model so a tail that moves takes its
        # authority with it - which it did: moving the prop to the ventral fin
        # root took this from 1.670 m to 1.902.
        'arm': abs(gz_model.pose(gz_model.named(model, 'link', TAIL_LINK), 0) - com[0]),
    }



def stamped_yaw_rate():
    """(simulator time in s, yaw rate in deg/s) from one angular-velocity
    message, or None. The simulator's clock, because on an undamped plant
    the measurement is an acceleration and a wall-clock window over a
    simulator that runs at whatever rate it runs at is not a duration."""
    text = listener('vehicle_angular_velocity')
    t, xyz = field(text, 'timestamp'), vector(text, 'xyz')
    return None if t is None or xyz is None else (t / 1e6, math.degrees(xyz[2]))


def yaw_acceleration(samples):
    """Least-squares slope of yaw rate against simulator time, deg/s^2."""
    n = len(samples)
    mt, mr = sum(t for t, _ in samples) / n, sum(r for _, r in samples) / n
    stt = sum((t - mt) ** 2 for t, _ in samples)
    return sum((t - mt) * (r - mr) for t, r in samples) / stt if stt > 0 else float('nan')


def hold_for_sim_seconds(value, seconds, samples=None):
    """Keep one tail command up until the simulator's clock has advanced
    `seconds`, re-issued as actuator_hold requires, collecting stamped rates
    on the way. Bounded on the wall at three times that."""
    first = None
    deadline = time.time() + 3.0 * seconds + 5.0

    while time.time() < deadline:
        actuator_hold([(TAIL_FUNCTION, value)], timeout=int(seconds) + 3)
        sample = stamped_yaw_rate()

        if sample is None:
            continue

        first = first or sample[0]

        if samples is not None:
            samples.append(sample)

        if sample[0] - first >= seconds:
            return True

    return False


def terminal_yaw_deg(torque, drag_yaw):
    """Steady yaw rate for a constant couple against quadratic damping."""
    return math.degrees(math.sqrt(torque / drag_yaw)) if torque > 0 and drag_yaw > 0 else 0.0


def main():
    if not commander('status'):
        note('px4 에 붙지 못했다')
        return 3

    plant = hull()

    # 1. the model is in the world, under the name px4 spawns it with
    rows = gz_link.poses(MODEL)
    verdict('모델이 월드에 있다', rows is not None and len(rows) >= 10,
            '%s 의 자세 표본 %s 개' % (MODEL, '읽기실패' if rows is None else len(rows)))

    if not rows or len(rows) < 10:
        return summary()

    # 2. and it floats. Measured as a rate against the SIMULATOR's clock, then
    #    turned into the force that would hold it: at this drift the hull is at
    #    terminal velocity, so the drag holding it is the imbalance. The force
    #    is the quantity the check is about, it is comparable to the weight,
    #    and it separates the failures worth catching - buoyancy not enabled is
    #    71.7 N, a tenth of the volume missing is 7.2 N - from a hull balanced
    #    to a gram by hand. One gram settles at 5.5 cm/s on this vehicle, which
    #    is 1.1 m in the window: a fixed displacement tolerance could not be
    #    both meaningful and passable.
    time.sleep(SETTLE_S)
    first = gz_link.poses(MODEL)
    time.sleep(WINDOW_S)
    second = gz_link.poses(MODEL)
    third = None

    if plant['drag_z'] is None and first and second:
        # Undamped: a constant imbalance accelerates the hull instead of
        # settling it, so the force is (m + added heave mass) times the
        # acceleration, and an acceleration needs a third point.
        time.sleep(WINDOW_S)
        third = gz_link.poses(MODEL)

    if not first or not second or (plant['drag_z'] is None and not third):
        verdict('중성부력으로 뜬다', False, '자세를 읽지 못했다')
        return summary()

    t0, (_, _, z0), _ = first[-1]
    t1, (_, _, z1), _ = second[-1]
    span = t1 - t0
    honest = abs(span - WINDOW_S) <= CLOCK_TOL * WINDOW_S
    # A hull that has reached the ground drifts at exactly zero and reads as
    # balanced by either route, so the height is asserted first: below
    # MIN_HEIGHT_M the drift is the ground's, not the buoyancy's.
    lowest = min(z for r in (first, second, third or []) for _, (_, _, z), _ in r)

    if lowest < MIN_HEIGHT_M:
        verdict('중성부력으로 뜬다', False,
                '선체가 %.1f m 까지 내려왔다 (바닥 %.0f m): 표류가 아니라 지면이 멈춘 것이다' % (lowest, MIN_HEIGHT_M))
    elif not honest:
        verdict('중성부력으로 뜬다', False,
                '시뮬레이터 시계가 %.1f 초 흐르는 동안 벽시계는 %.1f 초 — 정착 가정이 깨졌다'
                % (span, WINDOW_S))
    elif plant['drag_z'] is not None:
        rate = (z1 - z0) / span
        weight = plant['mass'] * gz_model.G
        implied = math.copysign(plant['drag_z'] * rate * rate, rate)
        verdict('중성부력으로 뜬다', abs(implied) < BUOYANCY_TOL * weight,
                '%.0f 초 정착 뒤 시뮬레이터 시각 %.1f 초 동안 %+.3f m (%.4f m/s) '
                '= 불균형 %+.3f N, 무게 %.1f N 의 %.2f %%'
                % (SETTLE_S, span, z1 - z0, rate, implied, weight,
                   100.0 * abs(implied) / weight))
    else:
        t2, (_, _, z2), _ = third[-1]
        span2 = t2 - t1
        weight = plant['mass'] * gz_model.G

        if abs(span2 - WINDOW_S) > CLOCK_TOL * WINDOW_S:
            verdict('중성부력으로 뜬다', False,
                    '두 번째 창: 시뮬레이터 시계 %.1f 초 대 벽시계 %.1f 초' % (span2, WINDOW_S))
        else:
            accel = 2.0 * ((z2 - z1) / span2 - (z1 - z0) / span) / (t2 - t0)
            implied = (plant['mass'] + plant['added_z']) * accel
            verdict('중성부력으로 뜬다', abs(implied) < BUOYANCY_TOL * weight,
                    '감쇠 없는 플랜트: %.0f 초 정착 뒤 %.1f + %.1f 초 동안 %+.3f, %+.3f m, 가속도 %+.4f m/s^2 '
                    '= 불균형 %+.3f N (질량 %.2f + 부가 %.2f kg), 무게 %.1f N 의 %.2f %%'
                    % (SETTLE_S, span, span2, z1 - z0, z2 - z1, accel, implied,
                       plant['mass'], plant['added_z'], weight, 100.0 * abs(implied) / weight))

    # 3. the allocator recognised the airframe
    status = px4('control_allocator', 'status') or ''
    configured = re.findall(r'Configured actuators:\s*(\d+)', status)
    count = int(configured[0]) if configured else -1
    verdict('할당기가 네 개의 출력을 잡았다', count == 4,
            '할당기가 보고한 출력 %s' % (count if count >= 0 else '없음'))

    # 4. the tail has to go both ways, by as much as the model says, and the
    #    hull is what has to say so. FROM REST, before anything has pushed the
    #    hull: on the added-mass plants a hull that is translating carries the
    #    Munk moment -(m_y - m_x) u v, 4.9 N m per (m/s)^2 against the tail's
    #    4.5, and with no drag the translation the duty test used to leave
    #    behind never stopped; the tail then read +2 and -14 deg/s^2 where
    #    the same command from rest reads +-21.
    #     hull is what has to say so. The command cannot: the duty range is
    #     non-negative by construction and the sign lives in the model's thrust
    #     polynomials, so full reverse and full forward are both positive.
    #
    #     Re-issued every pass and sampled as it goes, and each leg starts from
    #     rest and reports where it started, because a leg that begins inside
    #     the previous one's wake measures the wake.
    forward, reverse = plant['thrust'] or (0.0, 0.0)
    damped = plant['drag_yaw'] is not None
    spin_s = TAIL_SPIN_S if damped else TAIL_SPIN_FREE_S

    if damped:
        floors = {1.0: TAIL_YAW_FRACTION * terminal_yaw_deg(forward * plant['arm'], plant['drag_yaw']),
                  -1.0: TAIL_YAW_FRACTION * terminal_yaw_deg(reverse * plant['arm'], plant['drag_yaw'])}
    else:
        # Nothing bounds the rate on an undamped plant, so the model's claim
        # is the angular ACCELERATION the tail produces, tau / I, read as the
        # slope of the yaw rate against the simulator's clock while the
        # command is held. That is independent of how long the leg lasts
        # and of the rate it starts at, which matters twice over: the sim
        # runs slower than the wall with the fin plugins on, and each leg is
        # braked by the opposite command rather than settled, since on this
        # plant a settling never comes.
        floors = {1.0: TAIL_YAW_FRACTION * math.degrees(forward * plant['arm'] / plant['izz']),
                  -1.0: TAIL_YAW_FRACTION * math.degrees(reverse * plant['arm'] / plant['izz'])}

    # A zero floor is a figure the model did not give (no thrust polynomial,
    # no reverse thrust, no yaw drag), and it turns the verdict below into a
    # sign test.
    floored = all(f > 0 for f in floors.values())
    spin, began = {}, {}

    for value in (1.0, -1.0):
        began[value] = settle_rate(limit=5.0 if damped else 15.0, deadline=80.0 if damped else 20.0)

        if damped:
            reach = 0.0
            deadline = time.time() + spin_s

            while time.time() < deadline:
                actuator_hold([(TAIL_FUNCTION, value)], timeout=int(spin_s))
                rate = angular_rate()

                if rate is not None and abs(rate[2]) > abs(reach):
                    reach = rate[2]

            spin[value] = reach
        else:
            samples = []
            hold_for_sim_seconds(value, spin_s, samples)
            spin[value] = yaw_acceleration(samples) if len(samples) >= 3 else float('nan')
            # The record, because one leg on cloudship_am_fin read a quarter
            # of the other and nothing in a verdict line says why.
            note('    %+.0f: %s; 꼬리 회전수 %s' % (value, ' '.join('%.2f:%+.1f' % (t - samples[0][0], r) for t, r in samples),
                                                  gz_link.motor_speed(TAIL_INDEX)))
            hold_for_sim_seconds(-value, spin_s)

        actuator_release([TAIL_FUNCTION])

    # Rest is a precondition only where the measurement is absolute. On the
    # undamped plant it is a change from wherever the leg began, which the
    # note reports, and a brake that is a little over or under leaves the
    # hull turning at a rate nothing here will remove.
    rested = damped and all(v is not None for v in began.values()) or not damped
    verdict('꼬리 추진기가 모델이 말한 만큼 양방향으로 민다',
            floored and rested and spin[1.0] > floors[1.0] and spin[-1.0] < -floors[-1.0],
            '명령 +1 -> 요 %s %+.1f %s (바닥 %+.1f), -1 -> %+.1f (바닥 %+.1f), '
            '모델 추력 %.3f / %.3f N%s'
            % ('최대' if damped else '각가속도 (감쇠 없음, 시뮬레이터 시계)', spin[1.0],
               'deg/s' if damped else 'deg/s^2', floors[1.0], spin[-1.0], -floors[-1.0],
               forward, reverse,
               ', 바닥이 0 이다: 모델이 추력 또는 요 항력을 주지 않았다' if not floored
               else ('' if damped else ', 시작 %s / %s deg/s' % tuple('?' if began[v] is None else '%+.1f' % began[v] for v in (1.0, -1.0)))
               if rested else ', 시작 각속도가 가라앉지 않았다'))

    # and the hull has to stop turning before a yaw stick means anything;
    # on the undamped plant it will not, so the stick verdict below is a
    # change from the rate it starts at rather than an absolute.
    if damped and settle_rate() is None:
        note('  경고: 꼬리 시험 뒤 선체가 멈추지 않았다')
    elif not damped:
        settle_rate(limit=15.0, deadline=20.0)

    # 5. a yaw stick turns it, and the right way
    if not wait_for_ekf():
        verdict('EKF 가 유효해진다', False, '위치 추정이 유효해지지 않았다')
        return summary()

    sticks = Sticks()
    sticks.start()
    time.sleep(2)
    commander('mode', 'manual')
    time.sleep(1)
    commander('arm')
    time.sleep(3)
    armed = field(listener('vehicle_control_mode'), 'flag_armed') == 1.0
    verdict('무장한다', armed, 'Manual 에서 %s' % ('무장됨' if armed else '무장 거부'))

    if not armed:
        sticks.stop = True
        return summary()

    before = angular_rate()
    sticks.yaw = YAW_STICK
    time.sleep(8)
    rate = angular_rate()
    sticks.yaw = 0
    time.sleep(1)
    commander('disarm', '-f')
    sticks.stop = True
    # Manual is open loop on this vehicle (the gate's first check), so on an
    # undamped plant the stick adds rate to whatever the hull already had.
    start = 0.0 if damped or before is None else before[2]
    verdict('오른쪽 요 스틱이 기수를 오른쪽으로 돌린다',
            rate is not None and (damped or before is not None) and rate[2] - start > 0.5,
            '8 초 뒤 요 각속도 %s%s'
            % ('읽기실패' if rate is None else '%+.2f deg/s' % rate[2],
               '' if damped else ' (시작 %s)' % ('읽기실패' if before is None else '%+.2f' % before[2])))
    time.sleep(2)

    # 6. the airframe's SIM_GZ_EC_MIN3/MAX3 have to be the model's duty range.
    #     The hull test below passes with the centre anywhere between the two
    #     ends, and a centre that is not the centre is exactly how the uint16
    #     truncation hid: it put neutral at 32768, full forward, with the stick
    #     at rest. Only the wire can see that. Last, and disarmed again by
    #     then: it reads a topic, not the hull, so the spin and the drift the
    #     earlier tests leave behind cost it nothing, whereas its own pushes
    #     would have cost them everything on an added-mass plant (below).

    if plant['duty']:
        low, high = plant['duty']
        want = {-1.0: low, 0.0: (low + high) / 2.0, 1.0: high}
        reach = {}

        for value in (-1.0, 0.0, 1.0):
            actuator_hold([(TAIL_FUNCTION, value)], timeout=12)
            time.sleep(4)
            reach[value] = gz_link.motor_speed(TAIL_INDEX)

        actuator_release([TAIL_FUNCTION])
        show = lambda v: '읽기실패' if reach[v] is None else '%.0f' % reach[v]
        verdict('꼬리 명령이 모델의 듀티 범위에 맞는다',
                all(reach[v] is not None and abs(reach[v] - want[v]) <= 1.0 for v in want),
                '명령 -1/0/+1 -> %s / %s / %s, 기대 %.0f / %.0f / %.0f'
                % (show(-1.0), show(0.0), show(1.0), want[-1.0], want[0.0], want[1.0]))
    else:
        verdict('꼬리 명령이 모델의 듀티 범위에 맞는다', False,
                '모델에 양방향 듀티 범위가 없다')


    return summary()


if __name__ == '__main__':
    sys.exit(main())
