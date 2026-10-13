#!/usr/bin/env python3
"""Short test of controller.py. Plain python, no pytest needed:

    python3 test_controller.py            # unit tests + replay of the recorded SITL samples
    python3 test_controller.py --demo     # also the toy closed-loop brake (informational only)

Three kinds of check:
  1. the controller law and gates, against the C++ it models;
  2. the allocator port, against the expectations of the firmware's own gtest
     cases (ActuatorEffectivenessAirshipTest.cpp), same inputs, same dt;
  3. replay: the recorded gz_cloudship_am_fin_drag gate run (sitl_out/samples.csv,
     2026-09-25 23:54, snapshot in data/) must satisfy torque = P*(sp - rate) + I
     and dI = I_gain * e * dt; the fit of the second gives sim seconds per wall
     second (the gate's windows are wall clock).
  4. the throttle the 0b check flies (forward_yaw_check.quantize's count), and
     the D1 0c policy primitives: set_feedforward and set_rate_cap
     bit-identical where they must be (the same gain every cycle; a cap at
     AS_YAWRATE_MAX), the cap's two placements, and the restore rule cycle for
     cycle against options/D's OptionD and against options/D's own record
     (beyond.jsonl, relwind+ on two plants).
"""
import csv
import glob
import hashlib
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import controller as C  # noqa: E402
import record_gains  # noqa: E402

FAILS = []
SKIPS = []      # checks that could not run on this tree (the re-anchoring, absent flight logs), reported beside the failures


def run_of(path):
    """(registered run, gz target) of a samples file: the replays below are of
    records flown at P 0.5 (chain7, chain9), and they say so through
    record_gains rather than through the model's defaults, which are HEAD's."""
    import re
    base = os.path.basename(path)
    m = re.match(r'samples_(gz_\w+?)_(chain7|chain9|20260925)\.csv$', base)

    if not m:
        raise LookupError('%s: no registered run' % base)

    run = {'chain7': 'chain7_20260925', '20260925': 'chain7_20260925', 'chain9': 'chain9_20260927'}[m.group(2)]
    return run, m.group(1)


def gains_of(path):
    return record_gains.gains(*run_of(path))[0]


def check(name, ok, detail=''):
    print('%s  %s%s' % ('PASS' if ok else 'FAIL', name, ('  -- ' + detail) if detail else ''))
    if not ok:
        FAILS.append(name)


def near(a, b, tol=1e-6):
    return abs(a - b) <= tol


# ---------------------------------------------------------------------------
# 1. controller
# ---------------------------------------------------------------------------

def test_setpoint():
    rmax = math.radians(15.0)
    sp = C.yaw_rate_setpoint(0.5, 0.1, rmax)
    check('stick 0.5 -> 6.667 deg/s (the gate\'s setpoint)', near(math.degrees(sp), 6.6666667, 1e-6),
          '%.6f deg/s' % math.degrees(sp))
    check('stick inside the deadzone -> 0', C.yaw_rate_setpoint(0.1, 0.1, rmax) == 0.0
          and C.yaw_rate_setpoint(-0.05, 0.1, rmax) == 0.0)
    check('full stick -> AS_YAWRATE_MAX', near(C.yaw_rate_setpoint(-1.0, 0.1, rmax), -rmax, 1e-12))
    check('NaN stick -> 0', C.yaw_rate_setpoint(float('nan'), 0.1, rmax) == 0.0)


def test_gate():
    for name in ('vectored', 'cloudship'):
        a = C.AIRFRAMES[name]
        pv = C.propulsive_yaw(a['CA_AIRFRAME'], a['CA_AIRSHIP_GRP'], a['CA_AIRSHIP_TAIL'], a['yaw_surfaces'],
                              a['CA_AIRSHIP_CS_K'])
        check('%s: propulsiveYaw' % name, pv)
    check('collective without tail keeps the loop open', not C.propulsive_yaw(16, 0, False, False, 1.0))
    checks = {m: C.yaw_rate_loop_active(m, True, True) for m in C.MODE_FLAGS}
    check('loop closes in Acro/Stabilized/Altctl/Posctl, open in Manual/auto',
          checks['acro'] and checks['stabilized'] and checks['altctl'] and checks['posctl']
          and not checks['manual'] and not checks['auto'], str(checks))
    check('disarmed or sticks lost -> loop open', not C.yaw_rate_loop_active('acro', False, True))


def test_rate_control_law():
    rc = C.RateControlYaw(0.5, 0.09, 0.3)
    t = rc.update(0.0, 0.2, 0.004)
    check('torque uses the integrator from before the step', near(t, 0.1, 1e-12) and rc.integ > 0)
    f = 1 - (0.2 / math.radians(400)) ** 2
    check('I step = i_factor * I * e * dt', near(rc.integ, f * 0.09 * 0.2 * 0.004, 1e-15),
          'i_factor %.6f' % f)
    rc.integ = 0.0
    for _ in range(100000):
        rc.update(0.0, 1.0, 0.004)
    check('integrator clamps at AS_YR_INT_LIM', near(rc.integ, 0.3, 1e-12))
    rc.integ = 0.1
    rc.set_saturation(True, False)
    rc.update(0.0, 0.5, 0.004)
    check('positive saturation freezes growth', near(rc.integ, 0.1, 1e-15))
    rc.update(0.5, 0.0, 0.004)
    check('positive saturation still lets it fall', rc.integ < 0.1)
    rc.set_saturation(False, True)
    i0 = rc.integ
    rc.update(0.5, 0.0, 0.004)
    check('negative saturation freezes falling', near(rc.integ, i0, 1e-15))
    rc2 = C.RateControlYaw(0.5, 0.09, 0.3)
    check('no output clipping', near(rc2.update(-3.0, 0.0, 0.004), 1.5, 1e-12))


def test_feedforward():
    """AS_YAWRATE_FF as HEAD has it: RateControl::update adds FF * sp to the
    torque (rate_control.cpp:78) and updateIntegral integrates the rate error
    alone (:88-118); setGains passes it with P, I and the limit on every
    parameter update (airship_yaw_rate_loop.hpp:58-69,
    airship_att_control_main.cpp:104-105); the saturation flags come from the
    allocator's verdict on the whole torque (airship_yaw_rate_loop.hpp:76-82)."""
    import random
    rng = random.Random(7)
    a, b = C.RateControlYaw(0.7, 0.09, 0.3, ff=1.3), C.RateControlYaw(0.7, 0.09, 0.3, ff=0.0)
    worst_t, worst_i = 0.0, 0.0
    for _ in range(5000):
        rate, sp = rng.uniform(-0.5, 0.5), rng.uniform(-0.3, 0.3)
        ta = a.update(rate, sp, 0.004)
        tb = b.update(rate, sp, 0.004)
        worst_t = max(worst_t, abs(ta - tb - 1.3 * sp))
        worst_i = max(worst_i, abs(a.integ - b.integ))
    check('FF adds FF * setpoint to the torque and nothing else', worst_t < 1e-15, 'worst %.1e' % worst_t)
    check('FF never enters the integrator (same rate error, same integral)', worst_i == 0.0)
    rc = C.RateControlYaw(0.7, 0.09, 0.3)
    rc.integ = 0.123
    rc.set_gains(0.6, 0.0, 0.3, 2.0)
    check('set_gains (setGains) keeps the integrator', rc.integ == 0.123 and rc.ff == 2.0)

    for name in ('AS_YAWRATE_P', 'AS_YAWRATE_I', 'AS_YR_INT_LIM', 'AS_YAWRATE_FF', 'AS_YAWRATE_MAX'):
        check('DEFAULT_PARAMS %s is the yaml default (%g)' % (name, C.DEFAULT_PARAMS[name]),
              C.DEFAULT_PARAMS[name] == float(C.yaml_default(C.AS_YAML, name)))

    import re
    for model in ('airship_vectored_independent', 'airship_vectored_independent_fin',
                  'airship_vectored_independent_am_fin_drag', 'cloudship', 'cloudship_fin', 'cloudship_am_fin_drag'):
        number = C.airframe_for_model(model)
        with open(C.airframe_file(number), encoding='utf-8') as handle:
            own = re.findall(r'^param set-default AS_YAWRATE_FF (\S+)', handle.read(), re.M)
        want = float(own[-1]) if own else C.DEFAULT_PARAMS['AS_YAWRATE_FF']
        check('%d (%s): AS_YAWRATE_FF %g, from its own file or the yaml' % (number, model, C.head_params(model)['AS_YAWRATE_FF']),
              C.head_params(model)['AS_YAWRATE_FF'] == want)
        check('%s: the scheme\'s airframe (gz_model.airframe) is the walker\'s (airframe_for_model), %d'
              % (model, number), C.gz_model.airframe(model)[0] == number)

    c = C.AirshipYawController('vectored', params={'AS_YAWRATE_FF': 1.0}, filter_gyro=False)
    c.set_param('AS_YAWRATE_I', 0.0)
    check('a parameter update keeps FF (setGains passes all four)', c.rc.ff == 1.0)
    c.set_param('AS_YAWRATE_FF', 0.4)
    check('AS_YAWRATE_FF changes at the next update', c.rc.ff == 0.4)

    # FF reaches the anti-windup only through the allocator: full stick
    # (15 deg/s) with FF 10 asks for 2.6 of torque, the pods saturate and the
    # positive flag freezes the integrator; FF 0 at the same rate does not
    def run(ff):
        ch = C.SitlYawChain('vectored', params={'AS_YAWRATE_FF': ff}, filter_gyro=False)
        for _ in range(1000):           # 4 s: the starboard pod's swing (flagged) is over by 1.5 s
            ch.step(0.0, 1.0, 'acro')
        return ch.ctrl
    hi, lo = run(10.0), run(0.0)
    check('FF large enough to saturate the pods sets the flag that stops the integrator',
          hi.rc.sat_pos and hi.integ < 1e-3 and not lo.rc.sat_pos and lo.integ > 10 * max(hi.integ, 1e-4),
          'FF 10: sat %s integ %.4f; FF 0: sat %s integ %.4f' % (hi.rc.sat_pos, hi.integ, lo.rc.sat_pos, lo.integ))


def run_modes(ctrl, schedule, gyro=0.02, t0=10_000_000):
    out = []
    t = t0
    for mode, stick, n in schedule:
        for _ in range(n):
            t += C.DT_US
            out.append((mode, ctrl.step(t, gyro, stick, mode), ctrl.integ))
    return out


def test_mode_switches():
    c = C.AirshipYawController('vectored', filter_gyro=False)
    run_modes(c, [('acro', 0.5, 250)])
    i_acro = c.integ
    run_modes(c, [('stabilized', 0.5, 1)])
    check('Acro -> Stabilized keeps the integrator (loop stays closed)', c.integ > i_acro > 0)
    run_modes(c, [('manual', 0.5, 1)])
    check('-> Manual resets it on the first open cycle', c.integ == 0.0)
    out = run_modes(c, [('manual', 0.5, 3)])
    check('Manual torque is the stick', all(o[1] == 0.5 for o in out))
    run_modes(c, [('acro', 0.5, 250)])
    t = c._last_run + C.DT_US
    c.step(t, 0.02, 0.5, 'acro', armed=False)
    check('disarm resets it', c.integ == 0.0)
    run_modes(c, [('acro', 0.5, 10)])
    t = c._last_run + C.DT_US
    c.step(t, 0.02, 0.5, 'acro', sticks_valid=False)
    check('invalid sticks reset it', c.integ == 0.0)

    a = C.AirshipYawController('cloudship', filter_gyro=False)
    b = C.AirshipYawController('cloudship', filter_gyro=False)
    oa = run_modes(a, [('acro', 0.5, 500), ('acro', 0.0, 500)])
    ob = run_modes(b, [('stabilized', 0.5, 500), ('stabilized', 0.0, 500)])
    check('Acro and Stabilized are the same loop', all(x[1] == y[1] for x, y in zip(oa, ob)))

    d = C.AirshipYawController('cloudship', filter_gyro=False)
    d.step(12345678, 0.0, 0.0, 'acro')
    check('first cycle dt clamps to 20 ms (_last_run starts at 0)', near(d.dt, 0.02))
    d.step(12345678 + 4000, 0.0, 0.0, 'acro')
    check('then dt = 4 ms', near(d.dt, 0.004))


def test_status_cadence():
    ch = C.SitlYawChain('cloudship', filter_gyro=False)
    gens = []
    for _ in range(10):
        ch.step(0.0, 0.0, 'acro')
        gens.append(ch.status_topic.gen)
    steps = [b - a for a, b in zip(gens, gens[1:])]
    check('status published every second 4 ms cycle (>= 5 ms rule)', steps == [0, 1] * 4 + [0][:len(steps) - 8],
          str(steps))

    # the saturation the controller acts on is one or two cycles old
    ch = C.SitlYawChain('vectored', filter_gyro=False)
    flags = []
    for k in range(6):
        ch.step(0.0, 1.0, 'acro')       # full stick from rest: starboard pod must swing to the rear
        flags.append(ch.ctrl.rc.sat_pos)
    check('vectored: swing shortfall reaches the controller on the cycle after', flags[0] is False and all(flags[1:]),
          str(flags))


# ---------------------------------------------------------------------------
# 2. allocator port vs the firmware's gtest expectations
# ---------------------------------------------------------------------------

def gtest_airframe(grp=1, tail=False, tlmin=-180.0, tlmax=180.0, rate=0.0, rev=0):
    """resetAirshipParams() of ActuatorEffectivenessAirshipTest.cpp:92-110:
    independent, +-180, no tail, credit 1, no slew; the tail is reversible only
    where a test passes its own limits."""
    return dict(CA_AIRFRAME=16, CA_AIRSHIP_GRP=grp, CA_AIRSHIP_TAIL=tail, CA_AIRSHIP_TLMIN=tlmin,
                CA_AIRSHIP_TLMAX=tlmax, CA_AIRSHIP_TLT_R=rate, CA_AIRSHIP_CS_K=1.0, CA_R_REV=rev,
                yaw_surfaces=False)


def sp6(yaw=0.0, tx=0.0, tz=0.0, roll=0.0):
    return [roll, 0.0, yaw, tx, 0.0, tz]


KDT = 0.02      # ActuatorEffectivenessAirshipTest.cpp:78


def test_allocator_port():
    eng, rel = C.AirshipPod.K_STEER_ENGAGE, C.AirshipPod.K_STEER_RELEASE

    a = C.AirshipAllocator(gtest_airframe())
    a.update(sp6(yaw=1.0), KDT)
    check('FullYawIsTheExactCouple (:222)', near(a.motors[0], 1) and near(a.motors[1], 1)
          and near(a.servos[0], 1) and near(a.servos[1], 0) and a.status()['unallocated_torque'][2] == 0)

    a = C.AirshipAllocator(gtest_airframe())
    a.update(sp6(yaw=1.0, tx=1.0), KDT)
    check('SaturationClampsAndReports (:272)', near(a.motors[0], 0) and near(a.motors[1], 1)
          and a.status()['unallocated_torque'][2] == 1.0)

    a = C.AirshipAllocator(gtest_airframe(rate=90.0))
    a.update(sp6(yaw=1.0), KDT)
    check('SwingShortfallIsSaturation (:1686)', 0 < a.servos[0] < 0.2 and a.status()['unallocated_torque'][2] == 1.0)

    a = C.AirshipAllocator(gtest_airframe(rate=90.0))
    a.update(sp6(tz=-1.0), KDT)
    s1 = a.servos[0]
    a.update(sp6(tz=-1.0), KDT)
    check('TiltRateLimited (:1559)', near(s1, 0.01, 1e-6) and near(a.servos[0], 0.02, 1e-6))

    a = C.AirshipAllocator(gtest_airframe())
    a.update(sp6(yaw=0.75 * eng), KDT)
    ok = near(a.servos[0], 0) and near(a.motors[0], 0) and near(a.motors[1], 0.75 * eng) \
        and a.status()['unallocated_torque'][2] == 0.0
    a.update(sp6(yaw=2 * eng), KDT)
    check('SteerBandShortfallIsNotSaturation (:1599)', ok and near(a.servos[0], 1)
          and a.status()['unallocated_torque'][2] == 0.0)

    a = C.AirshipAllocator(gtest_airframe())
    a.update(sp6(yaw=1.0), KDT)
    a.update(sp6(yaw=-0.5 * (eng + rel)), KDT)
    check('HysteresisBandReversalIsNotSaturation (:1708)', near(a.servos[0], 1) and a.motors[0] == 0
          and a.motors[1] == 0 and a.status()['unallocated_torque'][2] == 0.0)

    a = C.AirshipAllocator(gtest_airframe(rate=90.0))
    a.update(sp6(yaw=1.0), KDT)
    first = a.servos[0]
    a.update(sp6(yaw=0.5 * (eng + rel)), KDT)
    check('SwingContinuedInBandIsSaturation (:1735)', a.servos[0] > first
          and a.status()['unallocated_torque'][2] == 1.0)

    a = C.AirshipAllocator(gtest_airframe())
    a.update(sp6(tx=0.3, yaw=0.3 + 0.5 * eng), KDT)
    check('OnePodHeldInBandIsNotSaturation (:1780)', near(a.servos[0], 0) and a.motors[0] == 0
          and near(a.motors[1], 0.6 + 0.5 * eng, 1e-6) and a.status()['unallocated_torque'][2] == 0.0)

    a = C.AirshipAllocator(gtest_airframe(grp=0))
    a.update(sp6(yaw=0.5 * eng), KDT)
    check('CollectiveYawShortfallStandsInSteerBand (:1823)', a.status()['unallocated_torque'][2] == 1.0)

    a = C.AirshipAllocator(gtest_airframe(grp=0, tail=True))
    a.update(sp6(yaw=1.0), KDT)
    ok = a.motors[0] == 0 and a.motors[1] == 0 and near(a.motors[2], 1) and a.status()['unallocated_torque'][2] == 0
    a.update(sp6(yaw=-1.0), KDT)
    ok2 = a.motors[2] == 0 and a.status()['unallocated_torque'][2] == -1.0
    b = C.AirshipAllocator(gtest_airframe(grp=0, tail=True, rev=4))
    b.update(sp6(yaw=-1.0), KDT)
    check('TailServesCollectiveYaw / TailReverseNeedsConfiguration (:688, :710)',
          ok and ok2 and near(b.motors[2], -1) and b.status()['unallocated_torque'][2] == 0.0)

    # the rule, stated for the two gz airframes (CA_R_REV 4 on the Cloudship)
    cl = C.AirshipAllocator('cloudship')
    res = []
    for y in (0.5, 0.999, 1.0, 1.0002, -1.0002, 3.0):
        cl.update(sp6(yaw=y), 0.004)
        res.append(cl.status()['unallocated_torque'][2])
    check('Cloudship: tail saturates only past |yaw| > 1 + 1e-4', res == [0, 0, 0, 1, -1, 1], str(res))


def test_gate_adapter():
    """GateController + AllocatorActuation stepped the way gate.compose()
    steps them give the same torques as SitlYawChain on the same gyro series."""
    for frame in ('vectored', 'cloudship'):
        chain = C.SitlYawChain(frame)
        gc = C.GateController(frame)
        act = C.AllocatorActuation(frame, 1.0)
        unalloc = 0.0
        same = True
        for k in range(3000):
            mode, stick = ('manual', 0.0) if k < 10 else ('acro', 0.5 if k < 1500 else -0.3)
            gyro = 0.2 * math.sin(k * 0.004 * 0.7)
            t1, _ = chain.step(gyro, stick, mode)
            t2 = gc.update(0.004, gyro, stick, mode, True, True, unalloc)
            _, unalloc = act.step(0.004, t2)
            same &= t1 == t2 and chain.ctrl.integ == gc.ctrl.integ
        check('%s: gate.compose() adapter == SitlYawChain, 3000 ticks' % frame, same)
    gc = C.GateController('cloudship')
    ok = gc.integ is None
    gc.update(0.004, 0.0, 0.0, 'manual', True, True, 0.0)
    ok &= gc.integ is None
    gc.update(0.004, 0.0, 0.0, 'acro', True, True, 0.0)
    check('integ reads None until the loop first closes', ok and gc.integ == 0.0)


# ---------------------------------------------------------------------------
# 3. replay of the recorded SITL time series
# ---------------------------------------------------------------------------

# Phase -> (mode, stick), from sitl_check.py: main() 277-326 (Manual, stick 0
# except 'Manual 통과' 0.5), 351 brake (Acro, 0), clean_start 112-134 ('정지 대기'
# Acro 0, '출발' in the step's mode 0), 390-397 (추종 0.5, 놓음 0), 429-471 wind
# cases (Acro, 0).
def phase_mode_stick(phase):
    if phase in ('정지 확인', '바람 확인 기준', '바람 확인'):
        return 'manual', 0.0
    if phase == 'Manual 통과':
        return 'manual', 0.5
    if phase in ('Acro 제동', '정지 대기', '출발', 'Acro 놓음', 'Stabilized 놓음'):
        return 'acro', 0.0
    if phase in ('Acro 추종', 'Stabilized 추종'):
        return 'acro', 0.5
    if phase.startswith('바람 '):
        return 'acro', 0.0
    return None, None


def load(path):
    rows = []
    with open(path) as fh:
        for r in csv.DictReader(fh):
            rows.append({'t': float(r['t']), 'phase': r['phase'], 'rate': float(r['rate']),
                         'torque': float(r['torque']), 'integ': float(r['integ'])})
    return rows


def replay(path, P, I, rate_max, dz, FF=0.0, saturated_rows=frozenset()):
    """The gains are the record's (record_gains). saturated_rows: row times
    whose preceding interval had a yaw saturation flag set in the
    rate-driven model; those intervals are left out of the integrator fit,
    where anti-windup legitimately stops dI."""
    rows = load(path)
    name = os.path.basename(path)
    sp_of = lambda s: C.yaw_rate_setpoint(s, dz, math.radians(rate_max))
    res, manual_ok, manual_n = [], 0, 0
    xs, ys = [], []
    for k, r in enumerate(rows):
        mode, stick = phase_mode_stick(r['phase'])
        first = k == 0 or rows[k - 1]['phase'] != r['phase']
        if mode is None or first or not all(map(C.isfinite, (r['rate'], r['torque']))):
            continue
        i_gain = 0.0 if 'I 끔' in r['phase'] else I
        if mode == 'manual':
            manual_n += 1
            manual_ok += abs(r['torque'] - stick) < 1e-6
            continue
        if not C.isfinite(r['integ']):
            continue
        e = sp_of(stick) - math.radians(r['rate'])
        res.append(r['torque'] - (P * e + r['integ'] + FF * sp_of(stick)))
        # integrator increment over the next interval, same phase, not the first row
        if k + 1 < len(rows) and rows[k + 1]['phase'] == r['phase'] and i_gain > 0 \
                and rows[k + 1]['t'] not in saturated_rows \
                and C.isfinite(rows[k + 1]['integ']) and abs(rows[k + 1]['integ']) < 0.3 - 1e-3:
            e1 = sp_of(stick) - math.radians(rows[k + 1]['rate'])
            dt_wall = rows[k + 1]['t'] - r['t']
            xs.append(i_gain * 0.5 * (e + e1) * dt_wall)
            ys.append(rows[k + 1]['integ'] - r['integ'])

    res_abs = sorted(abs(x) for x in res)
    p95 = res_abs[int(0.95 * (len(res_abs) - 1))] if res_abs else float('nan')
    rtf = sum(x * y for x, y in zip(xs, ys)) / sum(x * x for x in xs) if xs else float('nan')
    fit_res = [y - rtf * x for x, y in zip(xs, ys)]
    rms = math.sqrt(sum(f * f for f in fit_res) / len(fit_res)) if fit_res else float('nan')
    ymag = math.sqrt(sum(y * y for y in ys) / len(ys)) if ys else float('nan')
    print('  %s: %d closed-loop rows, |torque - (P e + I)| median %.5f p95 %.5f max %.5f; '
          'Manual rows torque == stick %d/%d; integrator fit over %d intervals: '
          'sim/wall = %.4f, residual rms %.2e vs step rms %.2e'
          % (name, len(res), res_abs[len(res_abs) // 2] if res_abs else float('nan'), p95,
             res_abs[-1] if res_abs else float('nan'), manual_ok, manual_n, len(xs), rtf, rms, ymag))
    return len(res), p95, manual_ok, manual_n, rtf, rms, ymag


def rate_driven(path, frame, params, sim_per_wall=1.0, switch_delay=0.0):
    """Drive SitlYawChain (controller + allocator + status cadence) with the
    RECORDED rate, linearly interpolated between the 0.3 s rows, and the
    gate's stick/mode/I schedule; no plant model at all. Compare the model's
    integrator and torque with the recorded ones at every row. This checks
    the anti-windup path (pod swings freeze the integrator) against SITL.

    Schedule (sitl_check.py): a phase's stick/mode takes effect switch_delay
    after its first row is read; first rows can still show the old state
    (the brake's first torque 0.5000 in the 2026-09-25 file), so they are left
    out of the comparison, and 0 fits the other rows best (integrator p95
    0.00045/0.00052 at 0 against 0.00106/0.00071 at 50 ms on the two Cloudship
    files); the clean start
    goes to Manual 0.3 s after its last '정지 대기' row and into the step's
    mode 1.0 s before '출발' (lines 118-134); the I-off case is preceded by
    AS_YAWRATE_I 0, Manual 2 s, Acro 1 s (lines 459-463) and its recovery
    rows keep I = 0 (param restored at line 467, after wind_case returns).
    Returns per-row (t, phase, integ_rec, integ_model, torque_rec, torque_model, first_row,
    saturated: a yaw flag was set on some cycle since the previous row)."""
    rows = [r for r in load(path) if phase_mode_stick(r['phase'])[0] is not None]
    I = params['AS_YAWRATE_I']

    def settings(phase):
        mode, stick = phase_mode_stick(phase)
        return mode, stick, (0.0 if 'I 끔' in phase else I)

    # (start_wall_time, mode, stick, I) segments
    segs = [(rows[0]['t'] - 1.0,) + settings(rows[0]['phase'])]
    for k in range(1, len(rows)):
        r0, r1 = rows[k - 1], rows[k]
        if r1['phase'] == r0['phase']:
            continue
        s1 = settings(r1['phase'])
        if r1['phase'] == '출발' or ('I 끔' in r1['phase'] and 'I 끔' not in r0['phase']):
            segs.append((r0['t'] + 0.3, 'manual', 0.0, s1[2]))
            segs.append((r1['t'] - 1.0,) + s1)
        else:
            segs.append((r1['t'] + switch_delay,) + s1)

    chain = C.SitlYawChain(frame, params=params, filter_gyro=False)
    t = rows[0]['t']
    si = 0
    out = []
    step_wall = 0.004 / sim_per_wall
    for k, r in enumerate(rows):
        if k == 0:
            continue
        ra = rows[k - 1]
        sat = False
        while t + step_wall <= r['t'] + 1e-9:
            t += step_wall
            while si + 1 < len(segs) and segs[si + 1][0] <= t:
                si += 1
            _, mode, stick, i_gain = segs[si]
            if chain.ctrl.params['AS_YAWRATE_I'] != i_gain:
                chain.ctrl.set_param('AS_YAWRATE_I', i_gain)
            frac = (t - ra['t']) / (r['t'] - ra['t'])
            rate = math.radians(ra['rate'] + frac * (r['rate'] - ra['rate']))
            tq, _ = chain.step(rate, stick, mode)
            sat |= chain.ctrl.loop_active and (chain.ctrl.rc.sat_pos or chain.ctrl.rc.sat_neg)
        first = r['phase'] != ra['phase']
        out.append((r['t'], r['phase'], r['integ'], chain.ctrl.integ, r['torque'], tq, first, sat))
    return out


def test_rate_driven():
    found = sorted(glob.glob(os.path.join(HERE, 'data', 'samples_*.csv')))
    found += sorted(glob.glob(os.path.join(HERE, 'records', 'samples_*.csv')))      # the kept copies of the chain runs
    found += sorted(glob.glob(os.path.join(BENCH, 'sitl_out', 'samples_*_chain9.csv')))
    seen = set()
    for p in found:
        with open(p, 'rb') as fh:
            digest = hashlib.sha256(fh.read()).hexdigest()
        if digest in seen:
            continue
        seen.add(digest)
        base = os.path.basename(p)
        frame = 'cloudship' if 'cloudship' in base else ('vectored' if 'vectored' in base else None)
        if frame is None:
            continue
        out = rate_driven(p, frame, gains_of(p))
        errs = [abs(im - ir) for (_, ph, ir, im, _, _, first, _) in out
                if not first and C.isfinite(ir) and phase_mode_stick(ph)[0] != 'manual']
        terrs = [abs(tm - tr) for (_, ph, _, _, tr, tm, first, _) in out if not first and C.isfinite(tr)]
        scale = max(abs(ir) for (_, _, ir, _, _, _, _, _) in out if C.isfinite(ir))
        nsat = sum(1 for o in out if o[7])
        errs.sort()
        terrs.sort()
        p95 = errs[int(0.95 * (len(errs) - 1))]
        tp95 = terrs[int(0.95 * (len(terrs) - 1))]
        print('  rate-driven %s (%s): integrator |model - SITL| median %.5f p95 %.5f max %.5f (|I| up to %.4f); '
              'torque p95 %.5f max %.5f over %d rows; %d row intervals with a yaw saturation flag'
              % (base, frame, errs[len(errs) // 2], p95, errs[-1], scale, tp95, terrs[-1], len(terrs), nsat))
        check('rate-driven %s: integrator within 0.1 x its range on 95 %% of rows' % base,
              p95 < 0.1 * scale, 'p95 %.5f vs range %.4f' % (p95, scale))


def test_replay():
    found = sorted(glob.glob(os.path.join(HERE, 'data', 'samples_*.csv')))
    found += sorted(glob.glob(os.path.join(HERE, 'records', 'samples_*.csv')))      # the kept copies of the chain runs
    # per-plant series from the running chain, if any appeared
    found += sorted(glob.glob(os.path.join(BENCH, 'sitl_out', 'samples_*_chain9.csv')))
    paths, seen = [], set()
    for p in found:                     # the same run can sit here under two names
        with open(p, 'rb') as fh:
            digest = hashlib.sha256(fh.read()).hexdigest()
        if digest not in seen:
            seen.add(digest)
            paths.append(p)
    if not paths:
        check('replay data present', False, 'no samples file')
        return
    for p in paths:
        base = os.path.basename(p)
        frame = 'cloudship' if 'cloudship' in base else 'vectored'
        g = gains_of(p)
        sat_rows = frozenset(o[0] for o in rate_driven(p, frame, g) if o[7])
        n, p95, mok, mn, rtf, rms, ymag = replay(p, g['AS_YAWRATE_P'], g['AS_YAWRATE_I'], g['AS_YAWRATE_MAX'],
                                                 g['MAN_DEADZONE'], g['AS_YAWRATE_FF'], saturated_rows=sat_rows)
        # Tolerances: the three listener reads of one row are ~5 ms apart and
        # the filtered gyro noise is ~5e-4 rad/s, so P*noise*sqrt(2) ~ 4e-4;
        # 0.003 is the 95 % bound that still rejects a wrong P or I sign.
        check('replay %s: torque = P*(sp - rate) + I on 95 %% of rows' % os.path.basename(p),
              n > 50 and p95 < 0.003, 'p95 %.5f' % p95)
        check('replay %s: Manual is the passthrough' % os.path.basename(p), mn > 0 and mok == mn)
        check('replay %s: dI = I*e*dt with sim/wall within 0.9..1.1' % os.path.basename(p),
              0.9 < rtf < 1.1 and rms < 0.5 * ymag, 'sim/wall %.4f' % rtf)


def test_rate_driven_sensitivity():
    """The vectored replay must be able to tell the anti-windup model from
    wrong ones: each mutation below has to do at least 10x worse at p95."""
    path = os.path.join(HERE, 'data', 'samples_gz_airship_vectored_independent_chain9.csv')
    if not os.path.exists(path):
        check('vectored samples present for the sensitivity test', False, path)
        return

    gains = gains_of(path)

    def p95():
        e = sorted(abs(im - ir) for (_, ph, ir, im, _, _, f, _) in rate_driven(path, 'vectored', gains)
                   if not f and C.isfinite(ir) and phase_mode_stick(ph)[0] != 'manual')
        return e[int(0.95 * (len(e) - 1))]

    base = p95()
    results = {}
    orig = C.AirshipYawController._update_saturation
    C.AirshipYawController._update_saturation = lambda self, topic: None
    results['no anti-windup'] = p95()
    C.AirshipYawController._update_saturation = orig
    orig_sf = C.AirshipAllocator.shortfall
    C.AirshipAllocator.shortfall = lambda self, asked, axis: asked - self.achieved[axis]
    results['held-by-choice reported as saturation'] = p95()
    C.AirshipAllocator.shortfall = orig_sf
    rate = C.AIRFRAMES['vectored']['CA_AIRSHIP_TLT_R']
    C.AIRFRAMES['vectored']['CA_AIRSHIP_TLT_R'] = 60.0
    results['tilt slew 60 deg/s instead of 120'] = p95()
    C.AIRFRAMES['vectored']['CA_AIRSHIP_TLT_R'] = rate
    detail = 'model %.5f; ' % base + ', '.join('%s %.5f' % kv for kv in results.items())
    check('vectored replay rejects every anti-windup mutation (>= 10x worse)',
          all(v >= 10 * base for v in results.values()), detail)


def test_ulogs():
    """Cycle-level: ulog_check.py on the gate runs' flight logs, if present."""
    try:
        import ulog_check as U
    except ImportError as exc:
        check('ulog check available', False, str(exc))
        return
    for rel, frame in U.DEFAULT_LOGS + U.HEAD_FF_LOGS:
        path = os.path.join(U.LOGDIR, rel)
        if not os.path.exists(path):
            print('  (no %s)' % path)
            SKIPS.append('ulog %s' % rel)
            continue
        r = U.check_log(path, frame, verbose=False)
        ei = sorted(abs(x) for x in r['e_i'])
        et = sorted(abs(x) for x in r['e_tq'])
        p99i, p99t = ei[int(0.99 * (len(ei) - 1))], et[int(0.99 * (len(et) - 1))]
        check('ulog %s: integrator p99 %.1e, torque p99 %.1e (< 1e-3); yaw saturation sign %d/%d'
              % (rel, p99i, p99t, r['agree'], r['total']),
              p99i < 1e-3 and p99t < 1e-3 and r['agree'] >= 0.995 * r['total'] and r['total'] > 1000)


# ---------------------------------------------------------------------------
# informational: the chain on a toy rigid plant (NOT a plant model)
# ---------------------------------------------------------------------------

def demo():
    """Rigid hull, ideal actuators, no delay: J r' = K * delivered_yaw [+ drag].
    The numbers come from gz_model (yaw_inertia, yaw_authority, added mass,
    nRabsR) via the bench tools. Informational only: the gz tilt servo, the
    motor models and the command delay are the plant part's."""
    sys.path.insert(0, BENCH)
    import gz_model as g
    import closed_loop as cl
    for model, target in (('airship_vectored_independent', 'gz_airship_vectored_independent'),
                          ('airship_vectored_independent_am_fin_drag', 'gz_airship_vectored_independent_am_fin_drag'),
                          ('cloudship', 'gz_cloudship'), ('cloudship_am_fin_drag', 'gz_cloudship_am_fin_drag')):
        m = g.merged(model)
        J = g.yaw_inertia(g.family(model)) + g.added_mass(m).get('rr', 0.0)
        K = g.yaw_authority(model)
        q = abs(g.hydrodynamics(m).get('nRabsR', 0.0))
        ch = C.SitlYawChain(cl.ALL_TARGETS[target][1], params=C.head_params(model))
        # Manual, stick 0.5 for 3 s, from rest (the gate's 'Manual 통과'), then 0
        r = 0.0
        for mode, stick, secs in (('manual', 0.5, 3.0), ('manual', 0.0, 0.3)):
            for _ in range(int(secs * C.LOOP_HZ)):
                _, u = ch.step(r, stick, mode)
                r += (K * u - q * r * abs(r)) / J * 0.004
        r0 = r
        # Acro brake, 15 s
        for _ in range(int(15 * C.LOOP_HZ)):
            _, u = ch.step(r, 0.0, 'acro')
            r += (K * u - q * r * abs(r)) / J * 0.004
        print('  toy %s: J %.1f K %.2f q %.1f: Manual left %.1f deg/s, Acro brake after 15 s %.2f deg/s, I %.4f'
              % (target, J, K, q, math.degrees(r0), math.degrees(r), ch.ctrl.integ))


# ---------------------------------------------------------------------------
# 4. the throttle the 0b check flies, and the D1 0c policy primitives
# ---------------------------------------------------------------------------

def test_throttle_midpoint():
    """'midpoint' is forward_yaw_check.quantize's count (Python's round of
    fraction x 1000) after sitl_link's int() of the count's midpoint."""
    import closed_loop  # noqa: F401  (through gate: the bench tools on sys.path)
    import forward_yaw_check as fyc
    z = lambda f, q: int(round((C.throttle_stick(f, q) + 1.0) * 500.0))
    fracs = [k / 20000.0 for k in range(0, 20001, 7)] + [0.0305, 0.03054, 0.0119, 0.0186, 0.0125, 0.0135, 0.9995]
    worst = [f for f in fracs if z(f, 'midpoint') != fyc.quantize(f)[0]]
    check('midpoint flies forward_yaw_check.quantize\'s count on %d fractions (half counts to even, as round)'
          % len(fracs), not worst, str(worst[:5]))
    check('the plan\'s examples: 2527 at 2 m/s 0.03054 flies 31 counts (truncation 30), 2517 at 2.0 m/s 0.0119 '
          'flies 12 (11), 2517 at 2.5 m/s 0.0186 flies 19 (18)',
          [z(f, 'midpoint') for f in (0.03054, 0.0119, 0.0186)] == [31, 12, 19]
          and [z(f, True) for f in (0.03054, 0.0119, 0.0186)] == [30, 11, 18])
    rec = 0.0309999995      # a recorded float32 thrust setpoint
    check('a recorded thrust passes exactly with quantize False and loses a count truncated',
          abs((C.throttle_stick(rec, False) + 1.0) * 0.5 - rec) < 1e-15 and z(rec, True) == 30)


def stepped(ctrl, sticks, gyro, mode='acro', t0=10_000_000):
    """Drive a controller cycle by cycle: sticks and gyro per cycle (or a
    constant gyro), the mode per cycle or one; returns [(torque, integ,
    rate_sp)]."""
    out = []
    t = t0
    for k, s in enumerate(sticks):
        t += C.DT_US
        g = gyro[k] if isinstance(gyro, (list, tuple)) else gyro
        m = mode[k] if isinstance(mode, (list, tuple)) else mode
        out.append((ctrl.step(t, g, s, m), ctrl.integ, ctrl.rate_sp))
    return out


def random_drive(seed, n=3000):
    """Sticks that enter and leave the deadzone and reverse, a noisy gyro,
    and mode switches that open and close the loop."""
    import random
    rng = random.Random(seed)
    sticks, gyro, modes = [], [], []
    s, m = 0.0, 'acro'
    for _ in range(n):
        if rng.random() < 0.01:
            s = rng.choice([0.0, 0.05, 0.12, 0.3, 0.5, 1.0, -0.4, -1.0, 0.1])
        if rng.random() < 0.002:
            m = rng.choice(['acro', 'stabilized', 'manual'])
        sticks.append(s)
        gyro.append(rng.uniform(-0.3, 0.3))
        modes.append(m)
    return sticks, gyro, modes


def test_feedforward_law():
    """FF_LAW 'quadratic' (the firmware since 2026-10-12): the gain this cycle is
    AS_YAWRATE_FF |sp| / r_max plus the schedule's speed term
    (airship_yaw_schedule::feedforwardGain), so the parameter adds
    FF sp |sp| / r_max to the torque and never enters the integrator."""
    sticks, gyro, modes = random_drive(3, 1500)
    a = C.AirshipYawController('vectored', params={'AS_YAWRATE_FF': 1.3}, filter_gyro=False)
    b = C.AirshipYawController('vectored', params={'AS_YAWRATE_FF': 0.0}, filter_gyro=False)
    ra, rb = stepped(a, sticks, gyro, modes), stepped(b, sticks, gyro, modes)
    r_max = math.radians(C.DEFAULT_PARAMS['AS_YAWRATE_MAX'])
    d_t = max(abs((x[0] - y[0]) - 1.3 * x[2] * abs(x[2]) / r_max) for x, y in zip(ra, rb))
    check('AS_YAWRATE_FF adds FF sp |sp| / r_max to the torque (the setpoint\'s share of the maximum)', d_t < 1e-12,
          'worst %.1e' % d_t)
    check('and never enters the integrator', all(x[1] == y[1] for x, y in zip(ra, rb)))
    full = [x for x in ra if abs(abs(x[2]) - r_max) < 1e-12]
    check('at full stick the gain is the parameter itself (%d full-stick cycles)' % len(full),
          len(full) > 0 and all(abs((x[0] - y[0]) - 1.3 * x[2]) < 1e-12 for x, y in zip(ra, rb) if abs(abs(x[2]) - r_max) < 1e-12))


def test_feedforward_setter():
    sticks, gyro, modes = random_drive(3, 1500)
    a = C.AirshipYawController('vectored', params={'AS_YAWRATE_FF': 1.0}, filter_gyro=False)
    b = C.AirshipYawController('vectored', params={'AS_YAWRATE_FF': 1.0}, filter_gyro=False)
    ra = stepped(a, sticks, gyro, modes)
    rb = []
    t = 10_000_000
    for s, g, m in zip(sticks, gyro, modes):
        t += C.DT_US
        b.set_feedforward(0.0)          # a per-cycle hook's call: no speed term, as without a hook
        rb.append((b.step(t, g, s, m), b.integ, b.rate_sp))
    check('set_feedforward(0) every cycle is bit-identical to no hook (the speed term is 0 by default)', ra == rb)
    c = C.AirshipYawController('vectored', params={'AS_YAWRATE_FF': 1.0}, filter_gyro=False)
    c.set_feedforward(1.5)
    p, i, lim = c.rc.p, c.rc.i, c.rc.int_lim
    rc = stepped(c, sticks, gyro, modes)
    d_t = max(abs((x[0] - y[0]) - 1.5 * x[2]) for x, y in zip(rc, ra))
    check('set_feedforward(1.5), the speed term, adds 1.5 x setpoint to the torque and leaves the integrator, P, I and the '
          'limit alone', d_t < 1e-12 and all(x[1] == y[1] for x, y in zip(rc, ra))
          and (c.rc.p, c.rc.i, c.rc.int_lim) == (p, i, lim), 'worst %.1e' % d_t)
    c.set_param('AS_YAWRATE_I', 0.09)
    check('a parameter update puts AS_YAWRATE_FF back as the gain (setGains); step() then scales it', c.rc.ff == 1.0)
    bad = 0
    for value in (-0.1, float('nan'), float('inf')):
        try:
            c.set_feedforward(value)
        except ValueError:
            bad += 1
    check('set_feedforward refuses a negative or non-finite gain', bad == 3)


def test_rate_cap():
    sticks, gyro, modes = random_drive(5, 3000)
    a = C.AirshipYawController('vectored', filter_gyro=False)
    b = C.AirshipYawController('vectored', filter_gyro=False)
    b.set_rate_cap(a.params['AS_YAWRATE_MAX'])          # 15 deg/s, both places
    check('a cap equal to AS_YAWRATE_MAX in both places is bit-identical to no cap (3000 random cycles)',
          stepped(a, sticks, gyro, modes) == stepped(b, sticks, gyro, modes))

    def sp(yaw_stick, **cap):
        c = C.AirshipYawController('vectored', filter_gyro=False)
        if cap.pop('on', True):
            c.set_rate_cap(5.0, **cap)
        c.step(10_004_000, 0.0, yaw_stick, 'acro')
        return math.degrees(c.rate_sp)

    check('the stick\'s full scale alone: half stick (0.5 - 0.1) / 0.9 x 5 = 2.222 deg/s, full 5',
          near(sp(0.5, clamp=False), 0.4 / 0.9 * 5.0, 1e-9) and near(sp(1.0, clamp=False), 5.0, 1e-12))
    check('the clamp alone (the rover\'s placement): full stick 15 -> 5, half 6.667 -> 5, quarter 2.5 stays',
          near(sp(1.0, stick=False), 5.0, 1e-12) and near(sp(0.5, stick=False), 5.0, 1e-12)
          and near(sp(0.25, stick=False), 0.15 / 0.9 * 15.0, 1e-9))
    check('both (the plan): the stick spans the cap, half 2.222, full +5 / -5 deg/s; no cap: half 6.667',
          near(sp(0.5), 0.4 / 0.9 * 5.0, 1e-9) and near(sp(1.0), 5.0, 1e-12) and near(sp(-1.0), -5.0, 1e-12)
          and near(sp(0.5, on=False), 0.4 / 0.9 * 15.0, 1e-9))
    c = C.AirshipYawController('vectored', params={'AS_YAWRATE_FF': 1.0}, filter_gyro=False)
    c.set_rate_cap(5.0, stick=False)
    tq = c.step(10_004_000, 0.01, 1.0, 'acro')
    check('the loop is handed the capped setpoint: torque = P (cap - r) + FF cap (cap / r_max), the cap\'s share of the maximum',
          near(tq, 0.7 * (math.radians(5.0) - 0.01) + 1.0 * math.radians(5.0) * (5.0 / 15.0), 1e-12), '%.6f' % tq)
    bad = 0
    for value in (-1.0, float('nan')):
        try:
            c.set_rate_cap(value)
        except ValueError:
            bad += 1
    c.set_rate_cap(None)
    check('set_rate_cap refuses a negative or NaN cap; None removes it', bad == 2 and c.rate_cap is None)


def test_restore_rule():
    """options/D's restore rule (OptionD kind 'restore'), stepped beside
    OptionD itself on the same inputs, and its behaviour on a turn."""
    sys.path.insert(0, os.path.join(HERE, 'options/D'))
    import option_d as od
    for edge, rule in (('zero_or_sign', {'kind': 'restore'}), ('zero', {'kind': 'restore', 'edge': 'zero'})):
        for seed in (11, 12):
            sticks, gyro, modes = random_drive(seed, 4000)
            a = od.OptionD('vectored', filter_gyro=False, rule=rule)
            b = C.AirshipYawController('vectored', filter_gyro=False)
            b.set_restore(edge)
            ra, rb = stepped(a, sticks, gyro, modes), stepped(b, sticks, gyro, modes)
            check('restore %s, seed %d: bit-identical to OptionD over 4000 random cycles (%d restores, with mode '
                  'switches and sign reversals)' % (edge, seed, len(b.restore_events)),
                  ra == rb and len(b.restore_events) > 0)
    # a turn: a trim built with the stick centred, saved at the stick, put back on release
    c = C.AirshipYawController('vectored', filter_gyro=False)
    c.set_restore()
    stepped(c, [0.0] * 500, -0.02)                      # 2 s at -0.02 rad/s: the integrator winds up positive
    trim = c.integ
    out = stepped(c, [0.5] * 1000 + [0.0] * 3, 0.05, t0=c._last_run)
    release_tq = out[1000][0]
    check('the integrator saved as the stick leaves the deadzone is put back on release (setIntegral before that '
          'cycle\'s update: its torque is P (0 - r) + trim)', len(c.restore_events) == 1
          and c.restore_events[0][1] == trim and near(release_tq, 0.7 * (0.0 - 0.05) + trim, 1e-12),
          'trim %.5f, torque at the release %.5f' % (trim, release_tq))
    d = C.AirshipYawController('vectored', filter_gyro=False)
    d.set_restore()
    stepped(d, [0.0] * 200, -0.02)
    stepped(d, [0.5] * 300 + [0.0] * 5, 0.0, t0=d._last_run)
    saved = d._restore_trim
    stepped(d, [0.0] * 5, 0.0, mode='manual', t0=d._last_run)
    opened = d._restore_sp_prev
    stepped(d, [0.0], 0.0, t0=d._last_run)
    check('opening the loop clears the previous setpoint, closing it again the saved value (as OptionD)',
          saved > 0.0 and opened == 0.0 and d._restore_trim == 0.0, 'saved %.5f' % saved)
    bad = 0
    try:
        d.set_restore('both')
    except ValueError:
        bad = 1
    check('set_restore refuses an unknown edge', bad == 1)


def test_restore_recorded_scenario():
    """The rule against options/D's own record: beyond.py's relwind+ (a wind
    trim held by the integrator, half stick for 40 s, released for 40 s,
    P 0.7, gyro noise seed 1) on two plants, the closed loop built as options/D
    builds it but with this controller's restore switch in place of OptionD,
    every one of the scenario's recorded numbers (options/D/beyond.jsonl)."""
    import json
    sys.path.insert(0, os.path.join(HERE, 'options/D'))
    import beyond
    import closed_loop as cl
    record = {}
    with open(os.path.join(HERE, 'options/D/beyond.jsonl')) as handle:
        for line in handle:
            r = json.loads(line)
            record[r['id']] = r

    def build(target, rule=None, params=None, noise=False, seed=1, slew_deg=None, tail_tau=None, trace=False):
        assert rule == {'kind': 'restore'} and slew_deg is None and tail_tau is None
        loop = cl.build(target, params=params, noise=noise, trace=trace, **({'seed': seed} if noise else {}))
        loop.controller.ctrl.set_restore('zero_or_sign')
        return loop

    real = beyond.od.build
    beyond.od.build = build
    try:
        for target in ('gz_cloudship', 'gz_airship_vectored_independent_am_fin_drag'):
            rec = record['restore|P0.7|%s|relwind+|1|None' % target]
            try:
                got = beyond.run_one({k: v for k, v in rec.items() if k != 'm'})['m']
            except SystemExit as refusal:
                # vectored_actuation refuses a tree whose pods are not the
                # model's cited 10.97 m anchor; the record was flown on that
                # anchor and cannot be replayed on another plant.
                print('SKIP  restore reproduces options/D\'s recorded relwind+ on %s: %s'
                      % (target, str(refusal).splitlines()[0]))
                SKIPS.append('restore reproduces options/D\'s recorded relwind+ on %s' % target)
                continue
            diff = [k for k in rec['m'] if got.get(k) != rec['m'][k]]
            check('restore reproduces options/D\'s recorded relwind+ on %s: %d of %d numbers equal (trim %.5f, '
                  'integrator at release %.4f, at the end %.5f)'
                  % (target, len(rec['m']) - len(diff), len(rec['m']), got['trim_before'], got['integ_at_release'],
                     got['integ_end']), not diff, str(diff[:4]))
    finally:
        beyond.od.build = real


if __name__ == '__main__':
    test_setpoint()
    test_gate()
    test_rate_control_law()
    test_feedforward()
    test_mode_switches()
    test_status_cadence()
    test_allocator_port()
    test_gate_adapter()
    test_replay()
    test_rate_driven()
    test_rate_driven_sensitivity()
    test_ulogs()
    test_throttle_midpoint()
    test_feedforward_setter()
    test_feedforward_law()
    test_rate_cap()
    test_restore_rule()
    test_restore_recorded_scenario()
    if '--demo' in sys.argv:
        demo()
    print('\n%d failed, %d skipped' % (len(FAILS), len(SKIPS)) + (': ' + ', '.join(FAILS) if FAILS else '')
          + (' (skipped: ' + ', '.join(SKIPS) + ')' if SKIPS else ''))
    sys.exit(1 if FAILS else 0)
