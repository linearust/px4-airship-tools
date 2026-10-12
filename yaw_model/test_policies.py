#!/usr/bin/env python3
"""Tests of the step 0c policies (policies.py) and the comparison's own
arithmetic (policy_compare.py). Plain python:

    python3 test_policies.py

The PX4 helpers as ported (interpolate, AlphaFilter's alpha); A from the
tilt range and the tail's reverse bit; the schedule off and at rest exactly
(AS_YAWRATE_FF, AS_YAWRATE_MAX), each of its terms, rho's two cases, the
ramp's ends, monotonicity, a finite cap; the selector's intervals,
intersection and pitot rejection, the switch and the return hold, the
filter, the all-lost branch with the upper bound held only while the loop
is closed; the source cases; k and c from the files; the hooks on legs: at
rest bit-identical to HEAD, k = 0 bit-identical at speed, FF and the cap set
as the schedule says, the stall term, the forward-gated restore; and the
identity the "to rest" number stands on (the heading over the release is
(I at release - I at the end) / K_I while no flag clips the integrator); the
matrix's shape; the disturbance probe's trimmed entry and its rest of the
turn (the integrator ends at the couple's trim, not at 0).

Added after the 0c review (2026-10-02): the supplied-FF budget (SCHED_CB)
and its promise, the w(u_lo) blend, the ramp classes; A per direction (the
controller's per-direction cap, bit-identical when symmetric; a tail without
its reverse bit keeps the positive stick's cap); the pitot band read from the
speed document and the SDP3x check case equal to the 1.05 Pa case; the held
surge cells and their records; the allocator's 1:1 knee and the asymptote's
flag stop; gate 1's verdicts (dead sticks); the Cloudship's force balance;
the pod-swing kick.
"""
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import controller as C  # noqa: E402
import forward as F  # noqa: E402
import policies as P  # noqa: E402
import policy_compare as PC  # noqa: E402

FAILS = []
V27, V22 = 'gz_airship_vectored_independent_am_fin_drag', 'gz_airship_vectored_independent_fin'


def check(name, ok, detail=''):
    print('%s  %s%s' % ('PASS' if ok else 'FAIL', name, ('  -- ' + detail) if detail else ''))
    if not ok:
        FAILS.append(name)


def near(a, b, tol=1e-12):
    return abs(a - b) <= tol


def test_helpers():
    f = P.interpolate
    check('interpolate: y_low at and below x_low, y_high above x_high, linear between (Functions.hpp:152-166)',
          f(1.5, 1.5, 2.5, 0.0, 1.0) == 0.0 and f(-3.0, 1.5, 2.5, 0.0, 1.0) == 0.0
          and f(2.5000001, 1.5, 2.5, 0.0, 1.0) == 1.0 and near(f(2.0, 1.5, 2.5, 0.0, 1.0), 0.5)
          and near(f(2.5, 1.5, 2.5, 0.0, 1.0), 1.0))
    check('interpolate with x_low == x_high divides by nothing: a step', f(2.0, 2.0, 2.0, 0.0, 1.0) == 0.0
          and f(2.1, 2.0, 2.0, 0.0, 1.0) == 1.0)
    check('AlphaFilter alpha = dt / (tau + dt) (AlphaFilter.hpp:144-153)', near(P.alpha_of(0.004, 1.0), 0.004 / 1.004))


def test_authority():
    check('A: pods over +-180 deg 1, over +-90 deg 0.5, over +-120 deg 0.75, over 0..180 deg 1',
          P.tilt_authority(-180.0, 180.0) == 1.0 and near(P.tilt_authority(-90.0, 90.0), 0.5, 1e-15)
          and near(P.tilt_authority(-120.0, 120.0), 0.75, 1e-12) and P.tilt_authority(0.0, 180.0) == 1.0)
    check('A: the Cloudship\'s tail is reversible (CA_R_REV 4: motor 3), 1 both ways', P.tail_authority('cloudship') ==
          (1.0, 1.0))
    bad = 0
    for args in ((90.0, -90.0), (-200.0, 0.0), (0.0, 181.0)):
        try:
            P.tilt_authority(*args)
        except ValueError:
            bad += 1
    check('A refuses a range outside -180..180 or upside down', bad == 3)
    cfgs = {c.name: c for c in P.plant_configs()}
    check('the plant configs: 2527 and 2522 A 1, their +-90 twins A 0.5, the Cloudships A 1; 2517 not gated',
          [cfgs[n].A for n in ('2527', '2522', '2527/A0.5', '2522/A0.5', '2512', '2517')] == [1.0, 1.0, 0.5, 0.5, 1.0, 1.0]
          and not cfgs['2517'].gated and all(cfgs[n].gated for n in ('2527', '2522', '2527/A0.5', '2522/A0.5', '2512')))


def test_schedule():
    S = P.schedule
    ff, cap, info = S(4.0, 4.0, 0.0, 1.0, 15.0, 0.3, 1.0)
    check('k 0 is off: exactly (AS_YAWRATE_FF, AS_YAWRATE_MAX), whatever the speed', (ff, cap) == (1.0, 15.0)
          and info['off'])
    rest = [S(u, u, 4.0124, ff0, 15.0, 0.3, 1.0)[:2] for u in (0.0, 0.7, 1.5) for ff0 in (0.0, 0.3, 1.0, 10.0)]
    check('at rest and up to AS_SPD_LO the schedule is exactly (FF0, MAX) for any k > 0 (w = 0, r_lim <= MAX)',
          all(c == 15.0 for _, c in rest) and [f for f, _ in rest] == [0.0, 0.3, 1.0, 10.0] * 3)
    k, u = 4.0124, 4.0
    ff, cap, info = S(u, u, k, 1.0, 15.0, 0.3, 1.0)
    check('2527 at 4 m/s, a one-point interval: FF = 1 + k u, cap = (A - rho) / (FF0 + k u) in deg/s (no interval '
          'term), rho 0.3 (%.4f, %.3f deg/s)' % (ff, cap),
          near(ff, 1.0 + k * u) and near(cap, math.degrees(0.7 / (1.0 + k * u)), 1e-12) and info['rho'] == 0.3
          and 'interval' not in info['terms'] and info['binding'] == 'authority')
    ff, cap, info = S(0.0, 4.0, k, 1.0, 15.0, 0.3, 1.0)
    check('a wide interval [0, 4]: FF stays FF0 (w(u_lo) 0), the interval term rho / (k (u_hi - u_lo)) binds '
          '(%.3f deg/s)' % cap, ff == 1.0 and near(cap, math.degrees(0.3 / (k * 4.0)), 1e-12)
          and info['binding'] == 'interval')
    _, _, info = S(4.0, 4.0, k, 1.0, 15.0, 1.0, 0.5)
    check('rho = min(AS_YR_INT_LIM, A / 2): INT_LIM 1.0 on A 0.5 gives 0.25, A - rho 0.25 > 0 (no zero cap)',
          info['rho'] == 0.25 and info['terms']['authority'] > 0.0)
    _, c2, info2 = S(2.0, 2.0, k, 1.0, 15.0, 0.3, 1.0)
    check('inside the ramp (2 m/s, w 0.5) the applied cap is halfway from MAX to r_lim (%.3f deg/s)' % c2,
          near(c2, 15.0 + 0.5 * (info2['r_lim'] - 15.0), 1e-12) and near(info2['w_hi'], 0.5, 1e-15))
    c = 0.0292
    _, cs, infos = S(2.0, 2.0, k, 1.0, 15.0, 0.3, 1.0, c_stall=c)
    _, c3, info3 = S(3.0, 3.0, k, 1.0, 15.0, 0.3, 1.0, c_stall=c)
    check('the stall term c u_lo joins the min: at 2 m/s it binds (%.3f deg/s under the authority term %.3f) and '
          'the ramp lifts the cap halfway to MAX (%.3f); at 3 m/s it does not (%.3f over %.3f)'
          % (infos['terms']['stall'], infos['terms']['authority'], cs, info3['terms']['stall'],
             info3['terms']['authority']),
          near(infos['terms']['stall'], math.degrees(c * 2.0), 1e-12) and infos['binding'] == 'stall'
          and near(cs, 15.0 + 0.5 * (infos['terms']['stall'] - 15.0), 1e-12) and info3['binding'] == 'authority'
          and near(c3, info3['terms']['authority'], 1e-12))
    _, cz, _ = S(0.0, 4.0, k, 1.0, 15.0, 0.3, 1.0, c_stall=c)
    check('the stall term on an interval with u_lo 0 and u_hi past the ramp is 0: the stick does nothing (cap %g)'
          % cz, cz == 0.0)
    ok = True
    for _ in range(3000):
        lo = random.uniform(0.0, 7.0)
        hi = lo + random.uniform(0.0, 4.0)
        ff_a, cap_a, _ = S(lo, hi, k, 0.3, 15.0, 0.3, 1.0)
        ff_b, cap_b, _ = S(lo, hi + 0.01, k, 0.3, 15.0, 0.3, 1.0)
        ff_c, _, _ = S(lo + 0.01, hi + 0.01, k, 0.3, 15.0, 0.3, 1.0)
        ok = ok and math.isfinite(cap_a) and 0.0 <= cap_a <= 15.0 and cap_b <= cap_a + 1e-12 and ff_c >= ff_a - 1e-12
    check('3000 random intervals: the cap is finite in 0..MAX, never rises with u_hi, FF never falls with u_lo', ok)
    lo_side = [S(u, u, k, 1.0, 15.0, 0.3, 1.0)[:2] for u in (1.5, 1.5 + 1e-9)]
    hi_side = [S(u, u, k, 1.0, 15.0, 0.3, 1.0)[:2] for u in (2.5, 2.5 + 1e-9)]
    check('the ramp is continuous at both ends (AS_SPD_LO, AS_SPD_HI)',
          abs(lo_side[0][0] - lo_side[1][0]) < 1e-6 and abs(lo_side[0][1] - lo_side[1][1]) < 1e-6
          and abs(hi_side[0][0] - hi_side[1][0]) < 1e-6 and abs(hi_side[0][1] - hi_side[1][1]) < 1e-6)


def test_selector():
    dq = P.DQ
    check('one count of each sensor read from the speed document: SDP3x %g Pa, MS4525DO %g Pa'
          % (dq['SDP3x'], dq['MS4525DO']), dq == {'SDP3x': 0.05, 'MS4525DO': 1.05})
    lo, hi = P.pitot_interval(4.0, 1.05)
    q = 0.5 * P.RHO * 16.0
    check('pitot interval: the sample\'s q +- dq back to TAS, 4 m/s +- 1.05 Pa is %.3f..%.3f m/s' % (lo, hi),
          near(0.5 * P.RHO * lo * lo, q - 1.05, 1e-9) and near(0.5 * P.RHO * hi * hi, q + 1.05, 1e-9))
    check('pitot interval at rest is [0, sqrt(2 dq / rho)] (%.3f m/s for 1.05 Pa): wider at low speed by itself'
          % P.pitot_interval(0.0, 1.05)[1], P.pitot_interval(0.0, 1.05)[0] == 0.0)
    check('ground interval [max(u_g - W, 0), max(u_g + W, 0)]', P.ground_interval(4.0, 2.0) == (2.0, 6.0)
          and P.ground_interval(1.0, 2.0) == (0.0, 3.0) and P.ground_interval(-3.0, 2.0) == (0.0, 0.0))
    s = P.SpeedSelector(2.0, 0.05)
    lo, hi, src = s.update(0.004, 4.0, 4.5, True)
    p = P.pitot_interval(4.0, 0.05)
    check('first cycle: converged on the raw bounds; both usable: the intersection, AIRSPEED',
          (lo, hi) == (max(p[0], 2.5), min(p[1], 6.5)) and src == P.AIRSPEED)
    s = P.SpeedSelector(1.0, 0.05)
    s.update(0.004, 4.0, 4.0, True)
    lo, hi, src = s.update(0.004, 4.0, 7.0, True)
    check('an empty intersection drops the pitot for that cycle: GROUND, one reject', src == P.GROUND
          and s.rejects == 1)
    s = P.SpeedSelector(2.0, 0.05)
    s.update(0.004, 4.0, 4.0, True)
    srcs = [s.update(0.004, None if 10 <= i < 20 else 4.0, 4.0, True)[2] for i in range(400)]
    back = srcs.index(P.AIRSPEED, 20)
    check('the pitot lost: GROUND that same cycle; back to AIRSPEED only after %g s usable without a break (cycle '
          '%d, %d cycles after it returned)' % (P.SELECTOR_HOLD_S, back, back - 20),
          srcs[10] == P.GROUND and all(x == P.GROUND for x in srcs[10:back])
          and abs((back - 20 + 1) * 0.004 - P.SELECTOR_HOLD_S) < 0.0041)
    s = P.SpeedSelector(0.0, 0.05)
    s.update(0.004, None, 4.0, True)
    for _ in range(250):
        lo, hi, _ = s.update(0.004, None, 6.0, True)
    check('a step of the raw interval reaches 1 - (1 - alpha)^250 of it in 1 s (%.4f; e^-1 left)' % ((lo - 4) / 2),
          near((lo - 4.0) / 2.0, 1.0 - (1.0 - P.alpha_of(0.004, 1.0)) ** 250, 1e-12))
    s = P.SpeedSelector(0.0, 0.05)
    s.update(0.004, None, 4.0, True)
    for _ in range(250):
        lo, hi, src = s.update(0.004, None, None, True)
    held = hi
    for _ in range(250):
        lo2, hi2, _ = s.update(0.004, None, None, False)
    check('every source lost: NONE, the lower bound decays to 0 (%.3f after 1 s), the upper held while the loop is '
          'closed (%.3f), decaying once it opens (%.3f after 1 s)' % (lo, held, hi2),
          src == P.NONE and near(lo, 4.0 * (1.0 - P.alpha_of(0.004, 1.0)) ** 250, 1e-12) and held == 4.0
          and near(hi2, 4.0 * (1.0 - P.alpha_of(0.004, 1.0)) ** 250, 1e-12))
    ok = True
    s = P.SpeedSelector(2.0, 1.05)
    for _ in range(20000):
        tas = None if random.random() < 0.1 else random.uniform(0.0, 8.0)
        ug = None if random.random() < 0.1 else random.uniform(-2.0, 9.0)
        lo, hi, _ = s.update(0.004, tas, ug, random.random() < 0.8)
        ok = ok and 0.0 <= lo <= hi + 1e-12
    check('20000 random cycles (losses, rejects, open and closed loop): 0 <= u_lo <= u_hi always', ok)


class _View:
    def __init__(self, u_air, u_ground):
        self.u_air, self.u_ground = u_air, u_ground


def test_sources():
    S = P.SOURCES
    check('the 0c source cases: perfect, ground W0, ground W2/W3 x head/tail/90, two pitot biases, dropout, stuck',
          list(S) == ['perfect', 'ground_W0', 'ground_W2_head', 'ground_W2_tail', 'ground_W2_x90', 'ground_W3_head',
                      'ground_W3_tail', 'ground_W3_x90', 'pitot_bias_sdp', 'pitot_bias_ms', 'pitot_dropout',
                      'pitot_stuck'])
    check('the winds: (W, 180) a headwind at the start, (W, 0) a tailwind, (W, 90) from the left',
          S['ground_W2_head'].wind == (2.0, 180.0) and S['ground_W3_tail'].wind == (3.0, 0.0)
          and S['ground_W2_x90'].wind == (2.0, 90.0) and S['ground_W0'].wind is None)
    read = S['pitot_bias_sdp'].reader(3.0)
    tas, ug = read(0.0, _View(4.0, 4.2))
    check('the pitot bias: TAS from q + 1 Pa (%.4f m/s at 4 m/s), the ground speed as the plant has it' % tas,
          near(0.5 * P.RHO * tas * tas, 0.5 * P.RHO * 16.0 + 1.0, 1e-12) and ug == 4.2)
    read = S['pitot_dropout'].reader(3.0)
    check('the dropout: the pitot read until the stick onset, lost from it on',
          read(2.996, _View(4.0, 4.0))[0] is not None and read(3.0, _View(4.0, 4.0))[0] is None)
    read = S['pitot_stuck'].reader(3.0)
    first = read(-2.0, _View(4.0, 4.0))[0]
    check('stuck: the first reading for ever', read(50.0, _View(2.0, 2.0))[0] == first and near(first, 4.0, 1e-12))
    check('perfect has no selector; calm() keeps the configuration and drops the wind',
          S['perfect'].selector() is None and S['ground_W2_head'].calm().wind is None
          and S['ground_W2_head'].calm().w_max == 2.0)


def test_k_and_c():
    fs = P.fin_slope('airship_vectored_independent_am_fin_drag')
    runs = [K for _, K in fs['runs']]
    check('k of 2527: the engine\'s fin damping (mean of %s N m s/rad per m/s) over its %g N m: %.4f per rad/s per '
          'm/s' % (', '.join('%g' % x for x in runs), fs['authority'], fs['k']),
          near(fs['k'], sum(runs) / len(runs) / 54.0, 1e-12) and fs['member'] == 'airship_vectored_independent_fin')
    check('k beside the files\' sum %.4f and the replica\'s %.4f (6 %% stiff, G3)' % (fs['k_files'], fs['k_linear']),
          fs['k_files'] > fs['k'] and fs['k_linear'] > fs['k'])
    check('a model without vertical fins has no k (the schedule is off)', P.fin_slope('airship_vectored_independent')
          is None and P.stall_slope('cloudship') is None)
    st = P.stall_slope('airship_vectored_independent_am_fin_drag')
    check('c of 2527: %g x alpha_stall %.4f / arm %.4f m = %.5f rad/s per m/s (the dorsal fin, the lower stall angle)'
          % (st['margin'], st['alpha_stall'], st['arm'], st['c']),
          near(st['c'], 0.8 * st['alpha_stall'] / st['arm'], 1e-15) and st['alpha_stall'] == min(
              a for _, a, _ in st['fins']))
    cs = P.fin_slope('cloudship_am_fin_drag')
    check('the Cloudship\'s k comes from its own engine run (%s): %.4f' % (cs['runs'][0][0], cs['k']),
          len(cs['runs']) == 1 and cs['member'] == 'cloudship_fin')


def test_hooks_on_legs():
    cfg = PC.config('2527')
    short = dict(centred_s=1.0, stick_s=8.0, release_s=10.0)
    head = F.run_leg(V27, 0.0, 1.0, **short)
    same = []

    for policy in ('SCHED', 'SCHED_STALL', 'SCHED+RESTORE_FWD'):
        for src in ('perfect', 'ground_W0', 'pitot_bias_sdp', 'pitot_stuck'):
            kw, _ = P.leg_kwargs(policy, cfg, P.SOURCES[src].calm(), 0.0, 1.0, **short)
            same.append(PC._same(F.run_leg(**kw)['trace'], head['trace']))

    check('at rest in still air every scheduled policy with a source whose interval stays under AS_SPD_LO is HEAD, '
          'trace for trace (%d of %d)' % (sum(same), len(same)), all(same))
    kw, _ = P.leg_kwargs('SCHED', cfg, P.SOURCES['ground_W2_head'].calm(), 0.0, 1.0, **short)
    rest_w2 = F.run_leg(**kw)
    check('at rest, ground only with W 2: the interval [0, 2] puts w(u_hi) at 0.5 and the cap is on (sp %.2f deg/s '
          'against HEAD\'s 15)' % F.trace_at(rest_w2, 'sp', 5.0), F.trace_at(rest_w2, 'sp', 5.0) < 14.0)
    base = F.run_leg(V27, 4.0, 1.0, **short)
    off = P.PlantConfig('off', V27, k_override=0.0)
    kw, _ = P.leg_kwargs('SCHED', off, P.SOURCES['perfect'], 4.0, 1.0, **short)
    check('k 0 at 4 m/s: the hook sets (FF0, MAX) every cycle, bit-identical to HEAD', PC._same(F.run_leg(**kw)['trace'],
                                                                                         base['trace']))
    kw, hook = P.leg_kwargs('SCHED', cfg, P.SOURCES['perfect'], 4.0, 1.0, **short)
    leg = F.run_leg(**kw)
    t_end_stick = round(1.0 + 8.0 - F.DT, 6)
    row = hook.at['stick_end']
    want_ff, want_cap, _ = P.schedule(row[1], row[2], cfg.k, 1.0, 15.0, 0.3, 1.0)
    check('SCHED at 4 m/s: the loop flew the schedule\'s FF %.3f and cap %.3f deg/s (the setpoint of the stick\'s '
          'last cycle %.3f)' % (row[4], row[5], F.trace_at(leg, 'sp', 9.0)),
          row[0] == t_end_stick and (row[4], row[5]) == (want_ff, want_cap)
          and near(F.trace_at(leg, 'sp', 9.0), row[5], 1e-9))
    kw, hook = P.leg_kwargs('SCHED_STALL', cfg, P.SOURCES['perfect'], 2.0, 1.0, **short)
    F.run_leg(**kw)
    bind = hook.summary()['binding_stick']
    check('SCHED_STALL at 2 m/s on 2527: the stall term binds r_lim at the stick\'s onset and on most of its cycles '
          '(%s; the free surge takes the hull past 2.33 m/s, where the authority term is the lower)' % bind,
          hook.at['stick'][6] == 'stall' and bind.get('stall', 0) > 0.5 * sum(bind.values()))
    a = F.run_leg(**P.leg_kwargs('SCHED+RESTORE', cfg, P.SOURCES['perfect'], 4.0, 1.0, **short)[0])
    b = F.run_leg(**P.leg_kwargs('SCHED+RESTORE_FWD', cfg, P.SOURCES['perfect'], 4.0, 1.0, **short)[0])
    check('SCHED+RESTORE_FWD in forward flight (w(u_lo) 1 at the release) is SCHED+RESTORE, trace for trace, one '
          'put-back each', PC._same(a['trace'], b['trace']) and len(a['setup']['restore_events']) == 1
          and len(b['setup']['restore_events']) == 1)
    kw, hook = P.leg_kwargs('SCHED+RESTORE_FWD', cfg, P.SOURCES['perfect'], 0.0, 1.0, **short)
    leg = F.run_leg(**kw)
    check('SCHED+RESTORE_FWD at rest: the edge fired (%d) and nothing was put back' % hook.gated_edges,
          hook.gated_edges == 1 and not leg['setup']['restore_events'])
    kw, _ = P.leg_kwargs('HEAD', cfg, P.SOURCES['ground_W2_head'], 4.0, 1.0, **short)
    check('HEAD and RESTORE take no wind and no hook (the source cannot touch them)', kw['wind'] is None
          and kw['hook'] is None and P.leg_kwargs('RESTORE', cfg, None, 4.0, 1.0)[0]['restore'] == 'zero_or_sign')


def test_rest_identity():
    """The number "to rest" stands on: while no flag clips the integrator and
    it stays inside its limit, each release cycle adds K_I e dt with e = -r
    (the measured rate), so the heading over a window is (I at its start - I
    at its end) / K_I. On HEAD's 2527 4 m/s leg, over the 120 s release."""
    leg = F.run_leg(V27, 4.0, 1.0)
    m, s = leg['metrics'], leg['setup']
    ki = s['params']['AS_YAWRATE_I']
    pred = math.degrees((m['integ_stick_end'] - m['integ_rel_120']) / ki)
    sat = any(r['sat_pos'] or r['sat_neg'] for r in leg['series'] if r['t'] > s['times'][3])
    check('the release turns (I_release - I_end) / K_I: %.3f deg predicted, %.3f flown, no flag in the release'
          % (pred, m['heading_release']), abs(pred - m['heading_release']) < 0.1 and not sat)
    spec = dict(group='main', config='2527', policy='HEAD', source='none', k_factor=1.0, u0=4.0, stick=1.0, hold=20.0,
                release=120.0, seed=0, k_override=None)
    rec = PC.run_one(spec)['result']
    check('policy_compare\'s "to rest": heading at 120 s %+.1f + I_end / K_I = %+.1f deg (I parked 0.3 / K_I 0.09 = '
          '%.1f deg)' % (rec['drift_120'], rec['drift_rest'], math.degrees(0.3 / 0.09)),
          rec['rest_valid'] and abs(rec['drift_rest'] - math.degrees(0.3 / 0.09)) < 1.0)


def test_matrix_and_disturbance():
    specs = PC.matrix(12.7)
    ids = [s['id'] for s in specs]
    main = [s for s in specs if s['group'] == 'main']
    base = {(s['config'], s['u0'], s['stick'], s['hold']) for s in main}
    per = {}
    for s in main:
        key = (s['config'], s['u0'], s['stick'], s['hold'], bool(s.get('held')))
        per[key] = per.get(key, 0) + 1
    # a primary cell: HEAD, RESTORE, 4 scheduled policies x (12 sources + 3 k errors), SCHED_CB x 15 on the gate's
    # legs; a held cell (2522's 300 s) keeps its 62 free-surge runs and flies 77 held
    ok = all(n == (77 if (not h and not PC.held_cell(c, hold)) or h else 62)
             for (c, _, _, hold, h), n in per.items() if c in PC.PRIMARY)
    check('the matrix: %d runs, every id unique; 6 plants x 4 speeds x 3 sticks = %d base cases; a primary cell flies '
          'HEAD, RESTORE, 4 scheduled policies x (12 sources + 3 k errors) and SCHED_CB* x 15 on the gate\'s legs '
          '(77), a held cell keeps its 62 free runs beside them' % (len(specs), len(base)),
          len(set(ids)) == len(ids) and len(base) == 72 and ok)
    held = {(s['config'], s['hold']) for s in specs if s.get('held') and s['group'] == 'main'}
    check('the held cells: the gliders\' 300 s holds (2522, 2512) and every A 0.5 cell; SCHED_CB* flies only on the '
          'gate\'s legs of the gated plants',
          held == {('2522', 300.0), ('2512', 300.0)} | {(c, h) for c in PC.A05 for h in (20.0, 300.0)}
          and not any(s['policy'] == PC.CB and s['group'] == 'main'
                      and (s['config'] == '2517' or (PC.held_cell(s['config'], s['hold']) and not s.get('held')))
                      for s in specs)
          and all(s['id'].endswith('|held') == bool(s.get('held')) for s in specs))
    spec = dict(group='dist', config='2527', policy='HEAD', source='none', k_factor=1.0, u0=4.0, stick=1.0, hold=20.0,
                release=120.0, seed=0, k_override=None, variant='dist_const')
    rec = PC.run_one(spec)
    r = rec.get('result', {})
    ok = 'error' not in rec
    check('the disturbance probe: the trimmed integrator meets the couple at the Acro switch (r at the stick\'s onset '
          '%s deg/s); "to rest" ends the integrator at the trim the couple needs, I_inf %s: %s = %s + (I_end - I_inf) / '
          'K_I' % (PC.fnum(r.get('entry_stick', [0, 0, None])[2], '%+.5f'), PC.fnum((r.get('dist') or {}).get('I_inf'),
                                                                                   '%+.3f'),
                   PC.fnum(r.get('drift_rest'), '%+.2f'), PC.fnum(r.get('drift_120'), '%+.2f')),
          ok and abs(r['entry_stick'][2]) < 1e-3 and r['dist']['I_inf'] == -PC.DIST_SHARE
          and near(r['drift_rest'], r['drift_120'] + math.degrees((r['I_end'] + PC.DIST_SHARE) / 0.09), 1e-9),
          rec.get('error', ''))


def test_budget():
    """The supplied-FF interval term (SCHED_CB, review finding 1) and the ramp classes."""
    S, k = P.schedule, 4.0124
    same = []
    for lo, hi in ((2.5, 4.0), (3.0, 6.5), (0.0, 4.0), (0.0, 1.2), (4.0, 4.0), (1.0, 1.3)):
        same.append(S(lo, hi, k, 1.0, 15.0, 0.3, 1.0)[:2] == S(lo, hi, k, 1.0, 15.0, 0.3, 1.0, budget='supplied')[:2])
    check('the supplied-FF term is the plan\'s wherever w(u_lo) = 1, wherever u_lo = 0, and wherever u_hi is under '
          'the ramp (w(u_hi) = 0: the cap is MAX either way)', all(same))
    ff, cap_p, ip = S(1.68, 2.5, 3.895, 0.3, 15.0, 0.3, 1.0)
    _, cap_s, isup = S(1.68, 2.5, 3.895, 0.3, 15.0, 0.3, 1.0, budget='supplied')
    need = lambda cap: 3.895 * (2.5 - ip['w_lo'] * 1.68) * math.radians(cap)
    check('a straddling interval [1.68, 2.5] (the review\'s 2522 pitot_bias_ms end state): the plan\'s cap %.2f '
          'deg/s needs %.3f of the integrator at u_hi, over rho 0.3; the supplied term\'s %.2f deg/s needs %.3f'
          % (cap_p, need(cap_p), cap_s, need(cap_s)), need(cap_p) > 0.3 + 1e-9 and need(cap_s) <= 0.3 + 1e-12)
    ok = True
    for _ in range(3000):
        lo = random.uniform(0.0, 6.0)
        hi = max(lo, random.uniform(P.AS_SPD_HI, 8.0))
        ff, cap, info = S(lo, hi, k, 1.0, 15.0, 0.3, 1.0, budget='supplied')
        for u in (lo, 0.5 * (lo + hi), hi):
            ok = ok and k * (u - info['w_lo'] * lo) * math.radians(cap) <= info['rho'] + 1e-12
    check('3000 random intervals with u_hi past the ramp: under the supplied term a turn at the cap needs at most rho '
          'of the integrator at any speed in the interval (k right)', ok)
    lo_side = [S(u, 4.0, k, 1.0, 15.0, 0.3, 1.0, budget='supplied')[1] for u in (1.5, 1.5 + 1e-9)]
    check('the supplied term is continuous at AS_SPD_LO', abs(lo_side[0] - lo_side[1]) < 1e-6)
    _, c_hi, _ = S(2.0, 4.0, k, 1.0, 15.0, 0.3, 1.0)
    _, c_lo, _ = S(2.0, 4.0, k, 1.0, 15.0, 0.3, 1.0, blend='lo')
    check('blending the cap by w(u_lo) (%.2f deg/s) lifts it toward MAX over the w(u_hi) blend (%.2f) whenever u_lo '
          'is in the ramp' % (c_lo, c_hi), c_lo > c_hi)
    bad = 0
    for kw in ({'budget': 'x'}, {'blend': 'mid'}):
        try:
            S(2.0, 4.0, k, 1.0, 15.0, 0.3, 1.0, **kw)
        except ValueError:
            bad += 1
    check('schedule refuses an unknown budget or blend', bad == 2)
    rc = P.ramp_class
    check('ramp classes: off, u_lo past the ramp, straddling, u_hi inside it',
          rc(S(4.0, 4.0, 0.0, 1.0, 15.0, 0.3, 1.0)[2]) == 'off' and rc(S(3.0, 5.0, k, 1.0, 15.0, 0.3, 1.0)[2]) == 'w_lo=1'
          and rc(S(1.0, 4.0, k, 1.0, 15.0, 0.3, 1.0)[2]) == 'straddle'
          and rc(S(0.0, 2.0, k, 1.0, 15.0, 0.3, 1.0)[2]) == 'w_hi<1')
    check('SCHED_CB is SCHED with the supplied budget (the hook\'s budget), no restore',
          P.make_hook('SCHED_CB', PC.config('2527'), P.SOURCES['perfect'])[0].budget == 'supplied'
          and P.make_hook('SCHED_CB', PC.config('2527'), P.SOURCES['perfect'])[1] is None
          and P.make_hook('SCHED', PC.config('2527'), P.SOURCES['perfect'])[0].budget == 'plan')


class _Ctrl:
    """A controller stand-in that records what a hook sets."""
    def __init__(self):
        self.loop_active, self.calls, self.ff = True, [], None

    def set_feedforward(self, ff):
        self.ff = ff

    def set_rate_cap(self, cap, stick=True, clamp=True, cap_neg_deg_s=None):
        self.calls.append((cap, cap_neg_deg_s))


class _Plant:
    def __init__(self, u):
        self.u_air = self.u_ground = u


def test_direction():
    """A per direction (review finding 5)."""
    a = C.AirshipYawController('vectored', params={'AS_YAWRATE_MAX': 15.0})
    b = C.AirshipYawController('vectored', params={'AS_YAWRATE_MAX': 15.0})
    a.set_rate_cap(5.0)
    b.set_rate_cap(5.0, cap_neg_deg_s=5.0)
    sticks = [x / 50.0 for x in range(-50, 51)] + [float('nan')]
    check('a per-direction cap equal both ways is the one-number cap, bit for bit, over every stick',
          all(a._setpoint(s) == b._setpoint(s) for s in sticks))
    b.set_rate_cap(5.0, cap_neg_deg_s=2.0)
    pos, neg = b._setpoint(1.0), b._setpoint(-1.0)
    check('a per-direction cap: full stick right %.3f deg/s, full stick left %.3f deg/s (the stick\'s full scale per '
          'direction, the clamp [-cap-, cap+])' % (math.degrees(pos), math.degrees(neg)),
          near(math.degrees(pos), 5.0, 1e-12) and near(math.degrees(neg), -2.0, 1e-12))
    b.set_rate_cap(5.0, clamp=False, stick=False, cap_neg_deg_s=2.0)
    check('without the stick scaling and the clamp the per-direction cap does nothing',
          near(math.degrees(b._setpoint(-1.0)), -15.0, 1e-12))
    bad = 0
    for value in (-1.0, float('nan')):
        try:
            b.set_rate_cap(5.0, cap_neg_deg_s=value)
        except ValueError:
            bad += 1
    b.set_rate_cap(None)
    check('the per-direction cap refuses a negative or NaN rate; None clears both', bad == 2 and b.rate_cap is None
          and b.rate_cap_neg is None)
    af = C.AIRFRAMES['cloudship']
    rev = af['CA_R_REV']
    try:
        af['CA_R_REV'] = 0
        cfg = P.PlantConfig('norev', 'gz_cloudship_fin')
    finally:
        af['CA_R_REV'] = rev
    check('a tail without its reverse bit: A+ 1, A- 0 (the plan\'s per-direction A), A = A+ (every 0c stick is '
          'positive)', cfg.A_dir == (1.0, 0.0) and cfg.A == 1.0)
    hook = P.Scheduled(P.SOURCES['perfect'], 0.7884, 0.0, 15.0, 0.3, cfg.A_dir)
    ctrl = _Ctrl()
    hook(0.0, _Plant(4.0), ctrl, None)
    hook(0.004, _Plant(0.0), ctrl, None)
    sym = P.Scheduled(P.SOURCES['perfect'], 0.7884, 0.0, 15.0, 0.3, (1.0, 1.0))
    ctrl2 = _Ctrl()
    sym(0.0, _Plant(4.0), ctrl2, None)
    want_pos = P.schedule(4.0, 4.0, 0.7884, 0.0, 15.0, 0.3, 1.0)[1]
    check('the hook with (A+, A-) = (1, 0) at 4 m/s: the positive stick keeps its cap %.2f deg/s, the negative stick '
          'gets %.2f; at rest both are MAX; with A equal both ways it is the one-number call'
          % (ctrl.calls[0][0], ctrl.calls[0][1]),
          ctrl.calls[0] == (want_pos, 0.0) and ctrl.calls[1] == (15.0, 15.0) and ctrl2.calls == [(want_pos, None)])


def test_doc_and_check_source():
    check('the SDP3x\'s CMODEL 0 offset read from the speed document: %g Pa' % P.SDP_OFFSET, P.SDP_OFFSET == 1.05)
    a, b = P.source('pitot_bias_sdp_dq'), P.SOURCES['pitot_bias_ms']
    same = True
    for tas in (0.0, 1.0, 2.0, 2.5, 4.0, 6.0):
        sa, sb = a.selector(), b.selector()
        ra, rb = a.reader(3.0), b.reader(3.0)
        for t in (0.0, 1.0, 5.0):
            va, vb = ra(t, _View(tas, tas + 0.3)), rb(t, _View(tas, tas + 0.3))
            same = same and va == vb and sa.update(0.004, va[0], va[1], True) == sb.update(0.004, vb[0], vb[1], True)
    check('the SDP3x with dq at its documented offset is the 1.05 Pa case: the same readings and intervals '
          '(a pitot is its bias and its band in the model)', same and P.source('perfect') is P.SOURCES['perfect'])
    check('the pitot cases say what their band is: below the zero error (a misconfiguration), covering the bias',
          'misconfiguration' in P.SOURCES['pitot_bias_sdp'].describe()
          and 'covering the bias' in P.SOURCES['pitot_bias_ms'].describe())


def test_knee_and_asymptote():
    """The allocator's 1:1 knee (review finding 8, and the same on +-180 deg) and the asymptote that uses it."""
    t = 0.2748
    under = PC.alloc_delivery((-180.0, 180.0), t, 0.70)
    over = PC.alloc_delivery((-180.0, 180.0), t, 0.80)
    check('+-180 deg pods at T %.4f: 1:1 under the knee 1 - T (tau 0.70 -> %.4f, surge %.4f, no flag); past it half '
          'gain, the surge dropping and the shortfall flagged (tau 0.80 -> %.4f, surge %.4f)'
          % (t, under['yaw'], under['surge'], over['yaw'], over['surge']),
          near(under['yaw'], 0.70, 1e-9) and near(under['surge'], t, 1e-9) and under['unalloc'] == 0
          and near(over['yaw'], 0.5 * (1.0 + 0.80 - t), 1e-9) and over['unalloc'] > 0)
    t = 0.1221
    under = PC.alloc_delivery((-90.0, 90.0), t, 0.10)
    over = PC.alloc_delivery((-90.0, 90.0), t, 0.25)
    check('+-90 deg pods at T %.4f: 1:1 under the knee T (tau 0.10 -> %.4f); past it (T + tau) / 2 with the surge '
          'the same (tau 0.25 -> %.4f, surge %.4f), flagged' % (t, under['yaw'], over['yaw'], over['surge']),
          near(under['yaw'], 0.10, 1e-9) and near(over['yaw'], 0.5 * (t + 0.25), 1e-9)
          and near(over['surge'], 0.5 * (t + 0.25), 1e-9) and over['unalloc'] > 0)


def test_gate1_verdicts():
    """Gate 1's verdicts on hand-made records (review finding 6)."""
    def leg(d120, rest, heading, valid=True):
        return {'drift_120': d120, 'drift_rest': rest, 'rest_valid': valid, 'heading_stick': heading, 'rest_why': ''}

    class _B:
        def __init__(self, r):
            self.r = r

        def gate(self, cfg, policy, *a, **kw):
            return leg(100.0, 190.0, 40.0) if policy == 'HEAD' else self.r

    def verdict(r):
        out = PC.gate1(_B(r), ('2527',), ('SCHED',), lambda c, h: ['perfect'])
        return {x['verdict'] for x in out}

    check('gate 1: smaller on merit; within the floor with a live stick; a dead stick (under %g %% of HEAD\'s heading '
          'change) is no pass whatever its release; a pass under %g %% is counted slow; larger is not smaller'
          % (100 * PC.DEAD_SHARE, 100 * PC.SLOW_SHARE),
          verdict(leg(10.0, 20.0, 30.0)) == {'smaller'} and verdict(leg(0.5, 0.8, 30.0)) == {'smaller'}
          and verdict(leg(0.5, 0.8, 1.9)) == {'dead'} and verdict(leg(150.0, 191.0, 30.0)) == {'not'}
          and verdict(leg(10.0, 20.0, 1.9)) == {'dead'} and verdict(leg(10.0, 20.0, 9.9)) == {'smaller'}
          and PC.gate1_counts(PC.gate1(_B(leg(10.0, 20.0, 9.9)), ('2527',), ('SCHED',),
                                       lambda c, h: ['perfect']))['slow'] == 12)


def test_closed_form_and_kick():
    rec = PC.load_records()
    cf = PC.cloudship_closed_form(PC.Book(rec))
    if cf is None:
        check('the Cloudship\'s force balance (needs the hold records)', False)
        return
    check('the Cloudship\'s steady turn as a force balance: m |x_t| |x_f| / (x_f - x_t) / authority = %.2f per rad/s '
          'per m/s about the centre of mass, %.2f from the link origin, against %.2f measured'
          % (cf['k_closed'], cf['k_origin'], cf['k_meas']),
          near(cf['k_closed'], cf['m'] * abs(cf['x_t']) * abs(cf['x_f']) / (cf['x_f'] - cf['x_t']) / cf['authority'],
               1e-12) and abs(cf['k_closed'] / cf['k_meas'] - 1.0) < 0.15)
    spec = dict(group='kick', config='2522', policy='SCHED', source='perfect', k_factor=1.0, u0=2.0, stick=1.0,
                hold=20.0, release=10.0, seed=0, k_override=None)
    k = PC.run_one(spec)['result']['kick']
    check('the pod-swing kick on 2522 at 2 m/s: the swing ends %.1f s after the onset with u %.3f (from %.3f), Fx '
          'up to %.1f N during it, %+.3f N after' % (k['swing_end_s'], k['u_swing_end'], k['u_onset'], k['fx_max_N'],
                                                    k['fx_after_N']),
          k['u_swing_end'] > k['u_onset'] + 0.2 and k['fx_max_N'] > 10.0 and abs(k['fx_after_N']) < 0.5)


if __name__ == '__main__':
    random.seed(7)
    test_helpers()
    test_authority()
    test_schedule()
    test_selector()
    test_sources()
    test_k_and_c()
    test_hooks_on_legs()
    test_rest_identity()
    test_matrix_and_disturbance()
    test_budget()
    test_direction()
    test_doc_and_check_source()
    test_knee_and_asymptote()
    test_gate1_verdicts()
    test_closed_form_and_kick()
    print('\n%d failed' % len(FAILS) + (': ' + ', '.join(FAILS) if FAILS else ''))
    sys.exit(1 if FAILS else 0)
