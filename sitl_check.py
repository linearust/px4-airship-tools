#!/usr/bin/env python3
"""Simulator check for the airship yaw axis. Run it through gz_run.sh.

    GZ_TARGET=gz_airship_vectored_independent ./gz_run.sh ./sitl_check.py

Two plants, picked by GZ_TARGET. On the plain model the hull is a free-spinning
mass: a torque keeps accelerating it and nothing damps a rotation. That is
harsher than the real airship, which makes it a good place to see whether the
loop itself stops the vehicle. On the hull-aerodynamics model the air damps the
rotation, which is kinder in flight and harsher here, because the yaw integral
then has to hold the damping torque and unwinds slowly when the stick is
released. The two disagree on one check each, deliberately; the tools README
records which and why. Wind is injected as a constant torque on the hull.

Every check here turns the hull roughly in place. On the plants without fins
nothing in it is sensitive to surge or sway. On the _fin plants the fins see
whatever flow the rotation and any drift make, and the lift-drag plugin gives
nothing when that flow comes from behind, so their verdicts depend on how the
hull drifts; the wind expectation leaves the fins out. The hull's translational
drag coefficients, and the cross terms that carry their moments, are invisible
to this gate; aero_moment_check.py is what covers those.

Each tracking step starts clean: Acro first brings the hull to rest (bounded),
then a pass through Manual clears the integrator, since the controller resets
it whenever the loop opens. Without that the step inherited what the brake or
the previous release left, and its overshoot was the brake's, not the loop's.
"""
import collections
import csv
import math
import os
import sys
import time

from gz_link import TRACE, wrench, wrench_clear
from report import note, results, summary, verdict
import gz_model
from sitl_link import (MODEL_NAME, Sticks, commander, field, finite, listener,
                       param_set, param_show, vector, wait_for_ekf)

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sitl_out')

# The vehicle's own yaw authority, which the wind cases are written against,
# and whether it has a tilt band at all. A gust only means something as a
# fraction of the authority that has to hold it, so the torques below scale
# with it: the 1 N m that is 1.9% of the vectored airship's 54 would be 22% of
# the Cloudship's 4.48. The tilt-band case is about how the vectored
# allocation discounts a demand no pod can turn far enough to meet, which a
# collective tilt with a tail thruster does not do, so it is skipped there.
def vehicle_files():
    """{name: (authority N m, True)} of every vectored vehicle file in
    vehicles/: two pods of pods.thrust_n at +-hull.pod_y."""
    import glob
    import json
    out = {}

    for path in sorted(glob.glob(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'vehicles', '*.json'))):
        with open(path) as handle:
            v = json.load(handle)

        if 'pods' in v and 'hull' in v:
            out[v['name']] = (2.0 * v['pods']['thrust_n'] * v['hull']['pod_y'], True)

    return out


VEHICLES = dict({'cloudship': (4.48, False)},          # 2.3544 N tail x 1.902 m
                **vehicle_files())
# The authority the cases below were written against: 1 N m of wind on the
# 54 N m vectored airship of 2026-09 (18 N pods at 1.5 m). Every torque is
# scaled from it, not from whatever the generic vehicle is today
CASES_AUTHORITY_NM = 54.0
# A physics variant (cloudship_am_fin_drag, airship_vectored_independent_drag)
# is the same vehicle on another plant: same actuators, same authority. The
# key is the name with its scheme tokens stripped, not the include chain's
# root: the vectored airship's root is airship_base, a body and not a
# vehicle, and keying by it exited this check at import.
VEHICLE = gz_model.vehicle(MODEL_NAME)

# No default. An unlisted vehicle used to fall back to the 49 kg airship's
# authority, which for a 7 kg one turns every wind case into a gale and for a
# bigger one into a breeze - and either way the verdicts stop being about the
# vehicle under test while still reading as verdicts about it.
if VEHICLE not in VEHICLES:
    raise SystemExit(
        '%s 는 VEHICLES 에 없다. 요 권한을 모르면 바람 사례는 이 기체에 대한 것이 아니다.\n'
        '아는 기체: %s' % (MODEL_NAME, ', '.join(sorted(VEHICLES))))

YAW_AUTHORITY_NM, HAS_TILT_BAND = VEHICLES[VEHICLE]
# Every torque this check applies is a share of the vehicle's own authority:
# the 1 N m the vectored airship's cases were written with, scaled. On a
# 0.18 kg m^2 hull the unscaled 1 N m spun the hull to 165 deg/s and the
# engine diverged (2026-10-07)
STRONG_NM = YAW_AUTHORITY_NM / CASES_AUTHORITY_NM
# The table keeps the wind cases' names and torques fixed; the model is what
# the simulator flies. If a pod's thrust or station moves, the gusts must not
# silently stop scaling with the vehicle.
_MODEL_AUTHORITY = gz_model.yaw_authority(MODEL_NAME)

if abs(YAW_AUTHORITY_NM - _MODEL_AUTHORITY) > 0.01 * max(YAW_AUTHORITY_NM, _MODEL_AUTHORITY):
    raise SystemExit('%s 의 요 권한이 VEHICLES 에는 %.2f N m, 모델에는 %.3f N m 이다. 표를 모델에 맞춘다.'
                     % (MODEL_NAME, YAW_AUTHORITY_NM, _MODEL_AUTHORITY))

STICK = 0.5
# how long clean_start() waits for the hull to come to rest in Acro
CLEAN_BOUND_S = 60.0
# the band a released hull has settled into, held for 1.5 s
SETTLED_DEG_S = 1.0

samples = []


T0 = time.time()


def sample(phase, seconds, period=0.3):
    rows = []
    end = time.time() + seconds

    while time.time() < end:
        rate = vector(listener('vehicle_angular_velocity'), 'xyz')
        torque = vector(listener('vehicle_torque_setpoint'), 'xyz')
        integral = field(listener('rate_ctrl_status'), 'yawspeed_integ')
        row = {'t': round(time.time() - T0, 2), 'phase': phase,
               'rate': math.degrees(rate[2]) if rate else float('nan'),
               'torque': torque[2] if torque else float('nan'),
               'integ': integral if integral is not None else float('nan')}
        rows.append(row)
        samples.append(row)
        time.sleep(period)

    return rows


# The hull's translational drag, from the files: a plant with none keeps the
# drift the earlier phases gave it for good, so waiting for it would only run
# the bound out; its steps start capped and the tracking verdict says so
HAS_DRAG = any(gz_model.translational_drag(gz_model.merged(MODEL_NAME)))
# The vertical fins' yaw damping per m/s of drift, N m s/rad per m/s, from the
# files (0 without fins): a hull drifting at u is damped by this times u on
# top of P, and a plant without drag drifts for good (the indoor _fin plant at
# 0.14 m/s: 0.08 N m s/rad against 0.03 from P, so the I-off push drifted at
# 0.36 deg/s where P alone would give 1.36, 2026-10-12)
FIN_DAMPING = gz_model.fin_yaw_damping(MODEL_NAME)


def drift_speed():
    """The EKF's horizontal speed now, m/s (nan when unread)."""
    pos = listener('vehicle_local_position')
    vx, vy = field(pos, 'vx'), field(pos, 'vy')
    return math.hypot(vx, vy) if vx is not None and vy is not None else float('nan')


Windows = collections.namedtuple('Windows', 'tau_s integral_time_s swing_s push_s release_s settle_limit_s')


def loop_windows(izz, yaw_p, yaw_i, tilt_rate_deg, setpoint_deg):
    """The gate's time windows from the plant's own loop, in seconds.

    tau_s is the P-only decay time constant of THIS plant's yaw inertia (the
    rigid one, or with the added inertia of an _am member) over N P;
    integral_time_s is P / I, over which the integrator unwinds what it
    gathered during a rise; swing_s is one pod reversal at the tilt slew rate,
    for the pods to point the other way when the stick is released. The
    tracking window push_s is four of the slower of the first two (on an
    undamped plant the integral time is the longer: the indoor rigid plant,
    tau 6.1 s, P / I 7.8 s, was still 27 % over at 4 tau = 25 s and inside the
    tolerance at 35 s). The released hull has to settle inside
    settle_limit_s, the P-only decay from the setpoint to the settled band,
    tau ln(sp / SETTLED_DEG_S), with a quarter in hand, plus the reversal (a
    damped hull settles sooner, an undamped one takes exactly this); release_s
    is the window that holds it. Until 2026-10-12 these were the anchor's
    constants, 35 s and 10 s (its complete plant's 8.5 s, helped by its drag,
    plus 1.5 s), and the JAXA plant's 60 deg/s servos read as a loop defect at
    10.1 s."""
    tau_s = izz / (YAW_AUTHORITY_NM * yaw_p)
    integral_time_s = yaw_p / yaw_i if yaw_i else 0.0
    swing_s = math.pi / math.radians(tilt_rate_deg) if tilt_rate_deg and tilt_rate_deg > 0.0 else 0.0
    settle_limit_s = 1.25 * tau_s * math.log(setpoint_deg / SETTLED_DEG_S) + swing_s
    push_s = max(10.0, math.ceil(4.0 * max(tau_s, integral_time_s)))
    release_s = math.ceil(settle_limit_s + 1.5 + 1.0)
    return Windows(tau_s, integral_time_s, swing_s, push_s, release_s, settle_limit_s)


def rest_drift(slope, source, int_lim, setpoint_deg):
    """The drift, m/s, a step may start with: below the ramp the schedule's
    cap is reserve / (k u), so the hull has to be slower than the drift at
    which that cap clips the step's setpoint, with a tenth in hand (0.23 m/s
    on the 7 m plant, 0.08 on the indoor one whose k / N is 15.4); infinite
    with the schedule off. 2026-10-12: the JAXA and 7 m plants started their
    Acro steps capped at 2-4 deg/s on 0.2-0.5 m/s left by the brake phase,
    and a flat 0.1 m/s then ran the 7 m plant's wait out."""
    reserve = 0.5 * min(int_lim, 0.5)
    return reserve / (1.1 * slope * math.radians(setpoint_deg)) if slope > 0.0 and source != 0.0 else float('inf')


def clean_start(mode, drift_m_s=float('inf'), rest_deg_s=0.5, hold_s=1.0, bound_s=CLEAN_BOUND_S):
    """Bring the hull to rest in Acro, and where the hull has drag give its
    drift the rest of the bound to fall below drift_m_s, clear the integrator
    through Manual, enter `mode`. Returns (seconds it took the yaw rate to
    settle or None, the rate and the integrator as the step starts)."""
    commander('mode', 'acro')
    begin = time.time()
    quiet_since = None
    settled_after = None
    drift = float('nan')
    drifting = False

    # The premise is the yaw rate at rest; the drift gets the rest of the
    # bound to decay, and a drift that does not is written down, not held
    # against the loop (the indoor plant's 2 m hull loses 0.2 m/s slowly:
    # 2026-10-12 it ran the bound out once, the tracking itself fine)
    while time.time() - begin < bound_s:
        rows = sample('정지 대기', 0.3)
        row = rows[-1] if rows else {'rate': float('nan')}
        drift = drift_speed()
        drifting = HAS_DRAG and not (finite(drift) and drift < drift_m_s)

        if finite(row['rate']) and abs(row['rate']) < rest_deg_s:
            quiet_since = quiet_since or time.time()

            if time.time() - quiet_since >= hold_s:
                settled_after = settled_after or time.time() - begin

                if not drifting:
                    break
        else:
            quiet_since = None
            settled_after = None

    note('  %s 출발 전 표류 %.2f m/s%s' % (mode, drift, ' (항력 없는 플랜트: 표류를 기다리지 않음)' if not HAS_DRAG
                                       else ' (허용 %.2f 를 %g 초 안에 못 넘겼다: 상한이 깎으면 판정이 적는다)' % (drift_m_s, bound_s) if drifting
                                       else ''))

    commander('mode', 'manual')
    time.sleep(1.0)
    commander('mode', mode)
    time.sleep(1.0)
    rows = sample('출발', 0.3)
    row = rows[-1] if rows else {'rate': float('nan'), 'integ': float('nan')}
    return settled_after, row['rate'], row['integ']


def published_setpoint(stick_deg):
    """(the yaw rate setpoint the loop was just handed, deg/s; a note when
    it is less than the stick's own). With a speed source the schedule caps
    the stick's full scale, so a hull still drifting from the earlier phases
    (a drag-free plant keeps its drift for good) is asked for less than
    STICK x AS_YAWRATE_MAX; the loop is judged on what it was asked, and the
    cap is written down (2026-10-12: the JAXA plants sat at 2-3 deg/s for 25 s
    on 0.46 m/s of drift, the _fin plant at 1.6 deg/s on 0.55 m/s for good).
    An unread setpoint reads as the stick's own."""
    sp = field(listener('vehicle_rates_setpoint'), 'yaw')

    if sp is None:
        return stick_deg, ''

    asked = math.degrees(sp)

    if asked >= 0.95 * stick_deg:
        return asked, ''

    speed = listener('airship_speed')
    cap = field(speed, 'yaw_rate_cap')
    return asked, '; 스케줄 상한 %s deg/s (u %s..%s m/s) 이 스틱의 %.2f 를 %.2f 로 깎았다' % (
        '?' if cap is None else '%.2f' % math.degrees(cap), field(speed, 'u_lo'), field(speed, 'u_hi'), stick_deg, asked)


def tail_mean(rows, seconds=2.0, key='rate', signed=False):
    """Mean of the last `seconds` of one column. |value| unless signed.

    The tracking check used to hand-roll the signed form inline, twice over,
    with the window filter written out both times.
    """
    if not rows:
        return float('nan')

    last = rows[-1]['t']
    values = [row[key] if signed else abs(row[key])
              for row in rows if row['t'] >= last - seconds]
    return sum(values) / len(values) if values else float('nan')


def all_finite(rows, key='rate'):
    """Every sample in the window is a number.

    The guards below are thresholds, and every comparison against NaN is
    False, so a threshold alone cannot tell a quiet vehicle from an unread
    one. sample() writes NaN whenever a listener does not answer.
    """
    return bool(rows) and all(finite(row[key]) for row in rows)


def settled(rows, threshold=SETTLED_DEG_S, hold=1.5):
    start = None

    for row in rows:
        if abs(row['rate']) < threshold:
            start = row['t'] if start is None else start
            if row['t'] - start >= hold:
                return start - rows[0]['t']
        else:
            start = None

    return None


def still_falling(rows, tail=3.0):
    """Slope of |rate| over the last `tail` seconds, in deg/s^2.

    A rate that has not reached the threshold inside the window is not the
    same thing as a rate that is not coming down, and on a damped hull the
    difference is the whole verdict: a wound-up integral unwinds slowly and a
    quadratic damping term is weakest exactly where the check looks. So report
    which one happened instead of calling both "it never stopped".
    """
    last = [r for r in rows if r['t'] >= rows[-1]['t'] - tail]

    if len(last) < 4:
        return None

    t0 = last[0]['t']
    xs = [r['t'] - t0 for r in last]
    ys = [abs(r['rate']) for r in last]
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    var = sum((x - mx) ** 2 for x in xs)

    if var <= 0:
        return None

    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / var


def loop_is_closed(rows):
    """Ask the firmware whether the yaw stick is still torque.

    The parameters exist several lessons before anything reads them, so
    their presence proves nothing about the firmware's behaviour. What
    separates the two is what happens in Acro with the stick centred and
    the vehicle turning: on the passthrough the torque IS the stick, so
    it stays at zero, and a closed loop pushes back against the rotation.
    """
    body = rows[1:]

    if not body or not all_finite(body, 'torque'):
        return None    # not "no loop": no reading. The caller must tell them apart

    return any(abs(row['torque']) > 0.02 for row in body)


def main():
    os.makedirs(OUT, exist_ok=True)

    # AS_YAWRATE_MAX and MAN_DEADZONE are firmware parameters, and this used to
    # restate their defaults - 15.0 and 0.1 - as module constants. They agreed,
    # but they are one number written twice in two repositories, and the gate
    # would have gone on asserting 6.67 deg/s against an airframe that changed
    # either. AS_YAWRATE_I is read for the same reason and because the I-off
    # case below has to put back what it found, not the default it assumed.
    if TRACE:
        print('gz_publish T0 %.2f (samples.csv t = wall - T0)' % T0, flush=True)

    rate_max_deg = param_show('AS_YAWRATE_MAX')
    deadzone = param_show('MAN_DEADZONE')
    yaw_i = param_show('AS_YAWRATE_I')
    yaw_p = param_show('AS_YAWRATE_P')
    tilt_rate_deg = param_show('CA_AIRSHIP_TLT_R')

    if rate_max_deg is None or deadzone is None or not yaw_p:
        print('AS_YAWRATE_MAX / MAN_DEADZONE / AS_YAWRATE_P 를 읽지 못했다. '
              '지시 각속도와 루프의 시정수를 모르면 추종 검사는 무엇에 대한 것도 아니다.')
        return 2

    setpoint_deg = (STICK - deadzone) / (1 - deadzone) * rate_max_deg
    # the windows and the rest drift from the plant's own loop (loop_windows,
    # rest_drift), not from the anchor's constants
    model = gz_model.merged(MODEL_NAME)
    izz = gz_model.yaw_inertia(gz_model.family(MODEL_NAME)) + gz_model.added_mass(model).get('rr', 0.0)
    w = loop_windows(izz, yaw_p, yaw_i, tilt_rate_deg, setpoint_deg)
    note('루프 시정수 I/(N P) %.1f s (요 관성 %.3g kg m^2), 적분 시간 P/I %.1f s, 포드 반전 %.1f s: 추종 창 %g s, 놓음 정지 허용 %.1f s'
         % (w.tau_s, izz, w.integral_time_s, w.swing_s, w.push_s, w.settle_limit_s))
    drift_m_s = rest_drift(param_show('AS_YR_FF_SLP') or 0.0, param_show('AS_SPD_SRC') or 0.0,
                           param_show('AS_YR_INT_LIM') or 0.3, setpoint_deg)
    note('출발 전 허용 표류 %s m/s (스케줄 %s)' % ('%.2f' % drift_m_s if math.isfinite(drift_m_s) else '무한',
                                             '켬: 상한 reserve/(k u) 가 설정의 1.1 배에 닿는 속도' if math.isfinite(drift_m_s) else '꺼짐'))
    # P and FF are only recorded: the verdicts do not depend on them, but a run
    # that cannot say which gains it flew cannot be compared with another one
    note('기체 %s, 요 권한 %.2f N m, 지시 각속도 %.2f deg/s '
         '(AS_YAWRATE_MAX %.1f, MAN_DEADZONE %.2f, AS_YAWRATE_P %s, AS_YAWRATE_I %s, AS_YAWRATE_FF %s)'
         % (MODEL_NAME, YAW_AUTHORITY_NM, setpoint_deg, rate_max_deg, deadzone,
            param_show('AS_YAWRATE_P'), yaw_i, param_show('AS_YAWRATE_FF')))

    linearized = (param_show('THR_MDL_FAC') or 0.0) > 0.5

    if not linearized:
        print('THR_MDL_FAC 가 0 이다: 레슨 04 전이라 추력이 제곱 특성이고 응답이 느리게 보인다.')

    print('EKF 를 기다린다...', flush=True)

    if not wait_for_ekf():
        print('EKF 가 유효해지지 않았다'); return 2

    time.sleep(10)                    # arming needs a settled estimator
    param_set('MAN_ARM_GESTURE', 0)   # a yaw-right stick must not arm the vehicle

    sticks = Sticks()
    sticks.start()
    time.sleep(2)
    commander('mode', 'manual')
    time.sleep(1)
    commander('arm')
    time.sleep(3)

    if field(listener('vehicle_control_mode'), 'flag_armed') != 1.0:
        print('무장하지 못했다'); return 2

    print('무장 완료 (Manual)\n')

    # 0. Every number below is a change from where the vehicle starts, so
    # check it starts from rest. A hull already turning (a wrench left over
    # from a killed run, a world that never settled) shifts every delta and
    # the checks would still print a verdict.
    if not wrench_clear():
        note('  렌치 해제 발행 실패: 이전 토크가 남아 있을 수 있다')
    settle = sample('정지 확인', 3)

    # The whole window, not its last sample, and only if every sample is a
    # number. One instantaneous reading can land on a zero crossing of a hull
    # that is swinging, and one unreadable listener makes `abs(nan) > 2.0`
    # False, which is the same as passing.
    if not all_finite(settle) or tail_mean(settle, 3.0) > 2.0:
        print('시작 상태를 확신할 수 없다: 마지막 3 초 |각속도| %.2f deg/s, 표본 %d 개%s'
              % (tail_mean(settle, 3.0), len(settle),
                 '' if all_finite(settle) else ', 읽지 못한 표본이 있다'))
        sticks.stop = True
        commander('disarm', '-f')
        return 2

    # the wind injection must actually reach the hull. What 1 N m for 5 s does
    # to THIS plant, from the files: the rigid yaw inertia plus the added yaw
    # inertia of an _am member, against the yaw damping of a _drag member. A
    # fixed 1.5 deg/s floor was the rigid hull's number, and the added-mass
    # plants, whose inertia is 2.4 x, reached 1.04 and were called unpushed; an
    # undamped expectation then failed the Cloudship's drag plant, whose rate
    # saturates at 24 deg/s in the window.
    terms = gz_model.hydrodynamics(model)
    linear, quadratic = abs(terms.get('nR', 0.0)), abs(terms.get('nRabsR', 0.0))

    def expected_push(u_drift):
        # the fins damp the push by FIN_DAMPING x the drift the hull has
        fin = FIN_DAMPING * u_drift if finite(u_drift) else 0.0
        r, dt = 0.0, 0.001

        for _ in range(int(5.0 / dt)):
            r += (STRONG_NM - (linear + fin) * r - quadratic * r * abs(r)) / izz * dt

        return math.degrees(r)

    # `gz topic -p` reports success when the simulator drops the message, so
    # a push that did not move the hull is published once more before the
    # check reads as a plant that the wind cannot push (2026-10-12: four of
    # seventeen runs lost the first push, all five of their later ones acted)
    for attempt in range(2):
        before = sample('바람 확인 기준' if attempt == 0 else '바람 확인 기준 (재시도)', 2)[-1]['rate']
        u_drift = drift_speed()
        expected = expected_push(u_drift)
        wrench(STRONG_NM)
        after = sample('바람 확인' if attempt == 0 else '바람 확인 (재시도)', 5)

        if not wrench_clear():
            note('  렌치 해제 발행 실패: 이전 토크가 남아 있을 수 있다')

        pushed = finite(before, after[-1]['rate']) and abs(after[-1]['rate'] - before) > 0.5 * expected

        if pushed:
            break

        note('  %g N m 가 걸리지 않았다 (%.2f -> %.2f deg/s): 한 번 더 건다' % (STRONG_NM, before, after[-1]['rate']))
        sample('바람 확인 재시도 전', 5)

    verdict('바람(외부 토크)이 기체를 민다', pushed,
            '5 초 동안 %.2f -> %.2f deg/s (%g N m 이 요 관성 %.3g kg m^2 와 요 감쇠 %.3g, 표류 %.2f m/s 의 핀 감쇠 %.3g 에 주는 %.2f 의 절반 이상)%s'
            % (before, after[-1]['rate'], STRONG_NM, izz, quadratic, u_drift, FIN_DAMPING * u_drift if finite(u_drift) else 0.0,
               expected, '' if attempt == 0 else '; 두 번째 발행'))

    # 1. Manual is the passthrough: unchanged by this increment
    sticks.yaw = STICK
    manual = sample('Manual 통과', 3)
    sticks.yaw = 0
    # skip the first half: the stick takes a moment to reach the FMU
    torques = [row['torque'] for row in manual[len(manual) // 2:]]
    verdict('Manual 에서 스틱이 곧 토크', all(abs(t - STICK) < 0.02 for t in torques),
            '토크 %.3f, 각속도가 %.1f -> %.1f deg/s 로 계속 오른다'
            % (torques[-1], manual[0]['rate'], manual[-1]['rate']))

    # This phase is also the liveness proof. Manual is the passthrough on
    # every firmware in this branch, so the torque here must be the stick.
    # If it sat at zero the sticks never reached the FMU, and every verdict
    # below would be measuring silence — including "no loop", which is the
    # most believable wrong answer this script can give.
    if not (abs(torques[-1]) >= 0.02):
        # Written as a negated >= so that NaN - an unread torque - takes this
        # branch. `abs(nan) < 0.02` is False, so the old form walked a dead
        # simulator straight through to "this firmware has no yaw loop".
        print('\n스틱이 FMU 에 닿지 않았다 (Manual 에서 토크가 %s).'
              % ('0 이다' if finite(torques[-1]) else '읽히지 않는다'))
        print('아래를 재지 않고 멈춘다. 다시 돌린다.')
        sticks.stop = True
        commander('disarm', '-f')
        return 2

    # 2. Acro brakes the rotation Manual left behind.
    # These same samples answer "which firmware is this?". Manual just left
    # the vehicle turning fast, and the stick is centred: on the passthrough
    # the torque IS the stick and stays at zero, while a closed loop pushes
    # back hard. The parameters exist several lessons before anything reads
    # them, so their presence would answer the wrong question.
    commander('mode', 'acro')
    brake = sample('Acro 제동', 15)
    has_loop = loop_is_closed(brake)
    print('\n요 각속도 루프: %s (요 토크 %.3f, 각속도 %.1f deg/s)'
          % ({True: '있음', False: '없음 (통과 경로)', None: '판단 불가'}[has_loop],
             brake[1]['torque'] if len(brake) > 1 else float('nan'),
             brake[1]['rate'] if len(brake) > 1 else float('nan')))

    if has_loop is None:
        print('제동 창에서 요 토크를 읽지 못했다. 루프의 유무를 말할 수 없다.')
        sticks.stop = True
        commander('disarm', '-f')
        return 2

    if not has_loop:
        print('레슨 11 이후에 다시 돌린다.')
        print('(요 루프가 없으면 아래 검사는 잴 것이 없다: 통과가 아니라 미실행이다.)')
        sticks.stop = True
        commander('disarm', '-f')
        return 3	# not a pass: nothing below was measured

    verdict('Acro 가 남은 회전을 잡는다', abs(brake[-1]['rate']) < 6.0,
            '%.1f -> %.1f deg/s' % (brake[0]['rate'], brake[-1]['rate']))

    # 3. and 4. the loop tracks the stick and returns to zero, in both modes
    for mode, label in (('acro', 'Acro'), ('stabilized', 'Stabilized')):
        settled_after, start_rate, start_integ = clean_start(mode, drift_m_s)
        # The clean start is the step's precondition, so it gates the
        # verdict rather than annotating it. A hull still turning with the
        # stick lowers the peak and can pass an overshooting loop; one
        # turning against it raises the peak. An unread start rate or
        # integrator is a start nobody saw, which is not a clean one.
        clean = settled_after is not None and finite(start_rate, start_integ)
        start = '출발 %.2f deg/s, 적분기 %.3f' % (start_rate, start_integ)

        if settled_after is None:
            start += '; 정지 전제가 깨졌다 (%g 초 안에 서지 않았다)' % CLEAN_BOUND_S
        elif not clean:
            start += '; 정지 전제가 깨졌다 (출발 상태를 읽지 못했다)'

        sticks.yaw = STICK
        # The hull has a lot of inertia and little yaw damping, so the first
        # rise overshoots and the integral takes tens of seconds to walk it
        # back. Judging a 12 s window measures the overshoot, not the
        # tracking, and lands right on the tolerance.
        push = sample('%s 추종' % label, w.push_s)
        asked, capped = published_setpoint(setpoint_deg)
        sticks.yaw = 0
        release = sample('%s 놓음' % label, w.release_s)
        mean = tail_mean(push, 3.0, signed=True)
        peak = max(row['rate'] for row in push) if all_finite(push) else float('nan')
        verdict('%s: 스틱이 지시한 각속도를 따라간다' % label,
                clean and all_finite(push)
                and abs(mean - asked) < 0.25 * asked
                and peak < 1.4 * asked,
                '지시 %.2f, 마지막 3 초 평균 %.2f, 최대 %.2f deg/s; %s%s' % (asked, mean, peak, start, capped))
        rest = settled(release)
        slope = still_falling(release)

        # There has to have been a rotation to stop. settled() walks from
        # rows[0], so a window that was already quiet returns 0.0 - not None,
        # and under 10.0 - and the check credited '0.0 초 만에 정지' to a vehicle
        # that never turned. It measured "the hull is quiet", not "the loop
        # brought the hull to rest".
        was_turning = all_finite(release) and abs(release[0]['rate']) > 0.5 * asked

        if not was_turning:
            how = '놓을 때 이미 %.1f deg/s 였다: 세울 회전이 없었다' % release[0]['rate']
        elif rest is not None:
            how = '%.1f 초 만에 정지' % rest
        elif slope is not None and slope < -0.02:
            how = '창이 끝날 때까지 계속 내려간다 (마지막 3 초 %.2f deg/s^2)' % slope
        else:
            how = '멈추지 않음'

        verdict('%s: 스틱을 놓으면 선다' % label,
                was_turning and rest is not None and rest < w.settle_limit_s,
                '%.1f -> %.1f deg/s, %s (허용 %.1f s)' % (release[0]['rate'], release[-1]['rate'], how, w.settle_limit_s))

    # 5. wind: what the integral term is for
    commander('mode', 'acro')
    time.sleep(2)

    def wind_case(name, torque_nm, expect_hold, seconds=40):
        # `gz topic -p` reports success even when the simulator drops the
        # message, and a dropped push reads as a hull the loop holds still
        # (integrator 0.0000 against a verdict that wants 0.0185: 2026-09-27,
        # the rigid Cloudship). So judge by effect, at the end of the window
        # where the verdict reads: the +z couple turns the FRD hull negative,
        # so a loop carrying it ends the window commanding a positive torque
        # of more than half the push (about +needed with the integrator on,
        # P times the rate with it off). A peak of |torque| cannot tell,
        # because the first rows still hold what the previous case left in the
        # loop (2527, chain12b of 2026-09-28: row 0 read -0.0154 while the
        # push was lost). If the loop does not carry it, publish once more; if
        # it still does not, the case is unmeasured, not a failure of the
        # loop, and the verdict says so.
        needed = torque_nm / YAW_AUTHORITY_NM
        acted = False
        u_drift = drift_speed()
        fin = FIN_DAMPING * u_drift if finite(u_drift) else 0.0

        for attempt in range(2):
            # False is "not confirmed", not "absent": the push is published once
            # (gz_link.wrench), and whether it acted is judged from the torque below
            if not wrench(torque_nm):
                note('  렌치 발행 확인 실패 (%g N m): 걸렸는지는 토크로 판정한다' % torque_nm)

            rows = sample(name if attempt == 0 else name + ' (재시도)', seconds)
            if not wrench_clear():
                note('  렌치 해제 발행 실패: 이전 토크가 남아 있을 수 있다')
            held = tail_mean(rows, 2.0, 'torque', signed=True)
            # with the integral on the loop ends up carrying the whole push; with it
            # off, P carries its share beside the fins at the drift the hull has
            share = 1.0 if expect_hold else YAW_AUTHORITY_NM * yaw_p / (YAW_AUTHORITY_NM * yaw_p + fin)
            acted = all_finite(rows, 'torque') and held > 0.5 * needed * share

            if acted:
                break

            note('  %g N m 가 걸리지 않았다 (마지막 2 초 토크 %.4f, %.4f 미만): 한 번 더 건다' % (torque_nm, held, 0.5 * needed * share))
            sample(name + ' 재시도 전', 5)

        # Signed: a loop holding the push holds +needed. The magnitude alone
        # passed an integrator that had wound to -0.0073 against a 0.0074 push
        # (2520, chain14 of 2026-09-29: both weak pushes lost, a +0.05 deg/s
        # drift wound it the other way for 95 s)
        integ = tail_mean(rows, 2.0, 'integ', signed=True)
        rate = tail_mean(rows, 2.0, 'rate')
        # With the integral off, P and the fins at the hull's drift hold the
        # push: the hull drifts toward torque / (N P + k_fin u), less where the
        # hull's own damping takes a share
        drift_floor = 0.5 * math.degrees(torque_nm / (YAW_AUTHORITY_NM * yaw_p + fin))
        ok = acted and all_finite(rows) and all_finite(rows, 'integ') and (
            (rate < 1.0 and abs(integ - needed) < 0.5 * needed) if expect_hold else rate > drift_floor)
        verdict(name, ok, '마지막 2 초 |각속도| %.2f deg/s, 적분기 %.4f (필요한 토크 %.4f)%s%s'
                % (rate, integ, needed,
                   '' if expect_hold else '; P 와 표류 %.2f m/s 의 핀 감쇠로 평형 표류 %.2f deg/s 의 절반 이상' % (u_drift, 2.0 * drift_floor),
                   '' if acted else '; 렌치가 걸리지 않았다 (미측정)'))
        sample(name + ' 회복', 10)

    strong = STRONG_NM                           # 1 N m on the vectored airship
    weak = 0.4 * strong                          # below its tilt band
    wind_case('바람 %g N m, I 켬: 적분기가 받아낸다' % strong, strong, True)

    # AS_YAWRATE_I = 0 stops the integral growing; it does not empty what is
    # already in it, because the accumulator holds the gain-scaled sum. Go
    # out to Manual and back to make the loop drop it (레슨 13), or the "I
    # off" case starts with the previous case's integral still pushing.
    if yaw_i is None:
        note('AS_YAWRATE_I 을 읽지 못했다. 되돌릴 수 없는 이득을 끄지 않는다: I 끔 사례를 건너뛴다')
    else:
        param_set('AS_YAWRATE_I', 0.0)
        commander('mode', 'manual')
        time.sleep(2)
        commander('mode', 'acro')
        time.sleep(1)
        wind_case('바람 %g N m, I 끔: 계속 밀린다' % strong, strong, False, seconds=25)
        # Back to what was found, not to a restated default. The run measures
        # one more wind case after this, on whatever gain is in the vehicle.
        param_set('AS_YAWRATE_I', yaw_i)
        time.sleep(1)

    if HAS_TILT_BAND:
        wind_case('바람 %g N m (틸트 밴드 아래), I 켬' % weak, weak, True, seconds=50)

    sticks.stop = True
    commander('disarm', '-f')

    print('\n=== 정리 ===')

    for name, ok in results():
        print(('통과  ' if ok else '실패  ') + name)

    return summary()


def write_samples():
    """The CSV, on every exit route.

    It used to be written only on the success path, so an early return left
    the PREVIOUS run's file sitting there with a current timestamp - the one
    artefact anyone opens to find out why a run stopped, showing a different
    run.
    """
    path = os.path.join(OUT, 'samples.csv')

    if not samples:
        if os.path.exists(path):
            os.remove(path)     # better absent than stale
        return

    os.makedirs(OUT, exist_ok=True)

    with open(path, 'w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(samples[0].keys()))
        writer.writeheader()
        writer.writerows(samples)

    print('\n표본 %d 개를 %s 에 남겼다' % (len(samples), path), flush=True)


if __name__ == '__main__':
    try:
        status = main()
    finally:
        write_samples()

    sys.exit(status)
