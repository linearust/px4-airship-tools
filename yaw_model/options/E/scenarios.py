#!/usr/bin/env python3
"""What the gate does not check, on the vectored plants: stick steps, wind
rejection (steady, reversing, slowly varying), torque-command noise (gyro
noise scaled up as a vibration proxy) and the tilt servo speed.

    python3 scenarios.py                  # -> scenarios.jsonl
    python3 scenarios.py --report         # tables from scenarios.jsonl

Driven directly on the closed loop (no gate harness): armed in Manual, then
Acro with the stick centred for 5 s, then the scenario. Wind is a body FRD
yaw moment (the gate's world-z couple on a level hull). All times sim s.
"""
import collections
import json
import math
import multiprocessing as mp
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True

DEG = 180.0 / math.pi
GYRO_NOISE = 0.0008726646
VECTORED = ['gz_airship_vectored_independent', 'gz_airship_vectored_independent_am_fin_drag',
            'gz_airship_vectored_independent_fin']
BANDS = ['base', 'report_band', 'report_all', 'deliver']
GAINS = {'current': (0.5, 0.09), 'A': (0.7, 0.09)}


def stick_for(rate_deg):
    """Stick that asks for rate_deg (MAN_DEADZONE 0.1, AS_YAWRATE_MAX 15)."""
    s = 0.1 + 0.9 * abs(rate_deg) / 15.0
    return math.copysign(s, rate_deg)


# (name, [(duration s, stick, wind FRD N m or callable(t)->N m)])
SCEN = {
    'step_big': [(60, stick_for(6.6667), 0.0), (60, 0.0, 0.0)],
    'step_small': [(60, stick_for(1.0), 0.0), (60, 0.0, 0.0)],
    'reverse': [(40, stick_for(6.6667), 0.0), (40, stick_for(-6.6667), 0.0), (40, 0.0, 0.0)],
    'wind_strong': [(80, 0.0, -1.0), (40, 0.0, 0.0)],
    'wind_weak': [(80, 0.0, -0.4), (40, 0.0, 0.0)],
    'wind_weak_rev': [(80, 0.0, -0.4), (80, 0.0, 0.4), (40, 0.0, 0.0)],
    'wind_sine': [(150, 0.0, 'sine')],
    'hover': [(120, 0.0, 0.0)],
}


def run(job):
    target, band, gname, scen, noise_scale, seed, slew, phys = job
    import e_model as em
    import planar
    planar.GYRO_NOISE = GYRO_NOISE * noise_scale
    P, I = GAINS[gname]
    loop = em.build(target, band, params={'AS_YAWRATE_P': P, 'AS_YAWRATE_I': I}, noise=noise_scale > 0,
                    seed=seed, trace=True, slew_deg=slew, phys_slew_deg=phys)
    loop.set_stick(0.0)
    loop.set_mode('manual')
    loop.arm()
    loop.advance(1.0)
    loop.set_mode('acro')
    loop.advance(5.0)
    t_start = loop.sim
    k_start = len(loop.trace)
    segs = []
    for dur, stick, wind in SCEN[scen]:
        t0 = loop.sim
        loop.set_stick(stick)
        if wind == 'sine':
            steps = int(round(dur / 0.004))
            for _ in range(steps):
                loop.set_wind(-0.5 * math.sin(2 * math.pi * (loop.sim - t0) / 30.0))
                loop.advance(0.004)
        else:
            loop.set_wind(wind)
            loop.advance(dur)
        segs.append((t0 - t_start, loop.sim - t_start, stick, wind))
    tr = loop.trace[k_start:]
    return metrics(job, tr, t_start, segs)


def held_after(ts, ok, t_from, hold):
    """First t >= t_from from which ok() holds for >= hold s (None if never)."""
    start = None
    for t, good in zip(ts, ok):
        if t < t_from:
            continue
        if good:
            start = t if start is None else start
            if t - start >= hold:
                return start
        else:
            start = None
    return None


def metrics(job, tr, t_start, segs):
    target, band, gname, scen, noise_scale, seed, slew, phys = job
    ts = [r[0] - t_start for r in tr]
    rate = [r[14] * DEG for r in tr]              # true FRD rate, deg/s
    torque = [r[4] for r in tr]
    integ = [r[5] for r in tr]
    swings = [(r[15] or 0) for r in tr]
    swinging = [(r[10] == 'steering' or r[11] == 'steering') for r in tr]
    tilt = [(r[12], r[13]) for r in tr]
    out = {'target': target, 'band': band, 'gains': gname, 'scen': scen, 'noise': noise_scale, 'seed': seed,
           'slew': slew, 'phys': phys, 'swings': swings[-1] - swings[0], 'segs': []}
    sw0 = swings[0]
    for i, (a, b, stick, wind) in enumerate(segs):
        idx = [k for k, t in enumerate(ts) if a <= t < b]
        if not idx:
            continue
        seg_t = [ts[k] - a for k in idx]
        seg_r = [rate[k] for k in idx]
        m = {'t': (a, b), 'stick': stick, 'wind': wind}
        sp = 0.0 if abs(stick) <= 0.1 else math.copysign((abs(stick) - 0.1) / 0.9 * 15.0, stick)
        m['sp'] = sp
        m['swings'] = swings[idx[-1]] - (swings[idx[0] - 1] if idx[0] > 0 else sw0)
        m['swinging_frac'] = sum(1 for k in idx if swinging[k]) / len(idx)
        prev_sp = segs[i - 1][2] if i > 0 else 0.0
        if sp != 0.0:
            sgn = 1 if sp > 0 else -1
            peak = max(sgn * r for r in seg_r)
            m['overshoot_pct'] = 100.0 * (peak - abs(sp)) / abs(sp)
            t90 = next((t for t, r in zip(seg_t, seg_r) if sgn * r >= 0.9 * abs(sp)), None)
            m['t90'] = t90
            band_ok = [abs(r - sp) <= 0.1 * abs(sp) for r in seg_r]
            last_bad = max((t for t, g in zip(seg_t, band_ok) if not g), default=None)
            m['t_settle10'] = None if last_bad is None else (last_bad if last_bad < seg_t[-1] - 1.0 else None)
            m['mean_last10'] = sum(r for t, r in zip(seg_t, seg_r) if t >= seg_t[-1] - 10) / max(
                1, sum(1 for t in seg_t if t >= seg_t[-1] - 10))
        elif prev_sp != 0.0 and wind == 0.0:
            # release: time to |r| < 0.5 deg/s held 2 s, and the undershoot
            sgn = 1 if prev_sp > 0 else -1
            rest = held_after(seg_t, [abs(r) < 0.5 for r in seg_r], 0.0, 2.0)
            m['t_rest05'] = rest
            m['t_rest10'] = held_after(seg_t, [abs(r) < 1.0 for r in seg_r], 0.0, 1.5)
            m['undershoot'] = max(0.0, max(-sgn * r for r in seg_r))
            m['end_rate'] = seg_r[-1]
        if wind not in (0.0,):
            m['max_abs_rate'] = max(abs(r) for r in seg_r)
            m['t_recover02'] = held_after(seg_t, [abs(r) < 0.2 for r in seg_r], 0.0, 3.0)
            last = [k for k in idx if ts[k] >= b - 10]
            m['rms_last10'] = math.sqrt(sum(rate[k] ** 2 for k in last) / len(last))
            m['integ_end'] = integ[idx[-1]]
            if isinstance(wind, float):
                m['needed'] = -wind / 54.0
        elif sp == 0.0 and prev_sp == 0.0:
            # hover / after-wind segments: noise and drift
            m['max_abs_rate'] = max(abs(r) for r in seg_r)
            m['rms_rate'] = math.sqrt(sum(r * r for r in seg_r) / len(seg_r))
            tq = [torque[k] for k in idx]
            mu = sum(tq) / len(tq)
            m['torque_std'] = math.sqrt(sum((x - mu) ** 2 for x in tq) / len(tq))
            m['torque_mean'] = mu
            m['t_recover02'] = held_after(seg_t, [abs(r) < 0.2 for r in seg_r], 0.0, 3.0)
            m['integ_end'] = integ[idx[-1]]
        out['segs'].append(m)
    # whole-run heading change, deg
    return out


def jobs():
    out = []
    for target in VECTORED:
        for band in BANDS:
            for g in GAINS:
                for scen in SCEN:
                    out.append((target, band, g, scen, 1.0, 1, None, None))
                # torque-command noise: gyro noise x k (vibration proxy)
                for k in (10.0, 30.0, 60.0):
                    out.append((target, band, g, 'hover', k, 1, None, None))
                    out.append((target, band, g, 'wind_weak', k, 1, None, None))
                # tilt servo speed: CA_AIRSHIP_TLT_R and the servo together, and a
                # servo slower than the firmware assumes
                for slew, phys in ((60.0, None), (240.0, None), (None, 60.0)):
                    for scen in ('step_big', 'reverse', 'wind_weak_rev'):
                        out.append((target, band, g, scen, 1.0, 1, slew, phys))
    # longest first
    out.sort(key=lambda j: -sum(d for d, _, _ in SCEN[j[3]]))
    return out


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--workers', type=int, default=6)
    ap.add_argument('--report', action='store_true')
    args = ap.parse_args()
    path = os.path.join(HERE, 'scenarios.jsonl')
    if args.report:
        report(path)
        return
    todo = jobs()
    with mp.Pool(args.workers) as pool, open(path, 'w') as fh:
        for i, r in enumerate(pool.imap_unordered(run, todo, chunksize=1)):
            fh.write(json.dumps(r) + '\n')
            fh.flush()
            if (i + 1) % 50 == 0:
                print('%d/%d' % (i + 1, len(todo)), flush=True)
    print('written', path, len(todo))


def report(path):
    rows = [json.loads(line) for line in open(path)]
    by = {}
    for r in rows:
        by[(r['target'], r['band'], r['gains'], r['scen'], r['noise'], r['slew'], r['phys'])] = r
    short = {'gz_airship_vectored_independent': 'rigid', 'gz_airship_vectored_independent_am_fin_drag': 'complete',
             'gz_airship_vectored_independent_fin': '_fin'}

    def f(x, fmt='%.1f', none='never'):
        return none if x is None else fmt % x

    def get(t, b, g, sc, k=1.0, slew=None, phys=None):
        return by.get((t, b, g, sc, k, slew, phys))

    print('%d runs' % len(rows))
    print('\n== Stick steps (gyro noise x1). step: sp 6.67 deg/s 60 s then 0; small: sp 1.0 deg/s; '
          'reverse: +6.67 -> -6.67 at 40 s. os = overshoot %%, ts = settle into +-10%% [s], '
          'rest = release to |r|<0.5 held 2 s [s], us = release undershoot [deg/s], sw = target reversals >90 deg')
    for t in VECTORED:
        for g in GAINS:
            for b in BANDS:
                a = get(t, b, g, 'step_big'); c = get(t, b, g, 'step_small'); d = get(t, b, g, 'reverse')
                if not (a and c and d):
                    continue
                a0, a1 = a['segs'][0], a['segs'][1]
                c0, c1 = c['segs'][0], c['segs'][1]
                d1, d2 = d['segs'][1], d['segs'][2]
                print('  %-8s %-7s %-12s step os %5.1f ts %-5s rest %-5s us %.2f sw %3d | small os %6.1f ts %-5s rest %-5s us %.2f sw %3d'
                      ' | reverse os %5.1f ts %-5s rest %-5s sw %3d'
                      % (short[t], g, b, a0['overshoot_pct'], f(a0['t_settle10']), f(a1.get('t_rest05')),
                         a1.get('undershoot', 0), a['swings'],
                         c0['overshoot_pct'], f(c0['t_settle10']), f(c1.get('t_rest05')), c1.get('undershoot', 0),
                         c['swings'], d1['overshoot_pct'], f(d1['t_settle10']), f(d2.get('t_rest05')), d['swings']))
    print('\n== Wind rejection (gyro noise x1): step wind for 80 s. maxr = peak |rate| [deg/s], rec = to |r|<0.2 '
          'held 3 s [s], rms = last 10 s [deg/s], I/need = integrator / needed torque; weak_rev: +0.4 then -0.4 N m '
          '(second half shown); sine: 0.5 N m, 30 s period, 150 s')
    for t in VECTORED:
        for g in GAINS:
            for b in BANDS:
                ws = get(t, b, g, 'wind_strong'); ww = get(t, b, g, 'wind_weak'); wr = get(t, b, g, 'wind_weak_rev')
                sn = get(t, b, g, 'wind_sine')
                if not (ws and ww and wr and sn):
                    continue
                def w(seg):
                    return 'maxr %.2f rec %-5s rms %.3f I/need %.2f' % (
                        seg['max_abs_rate'], f(seg['t_recover02']), seg['rms_last10'],
                        seg['integ_end'] / seg['needed'] if seg.get('needed') else float('nan'))
                s0 = sn['segs'][0]
                print('  %-8s %-7s %-12s strong %s sw %d | weak %s sw %d | weak_rev %s sw %d | sine maxr %.2f rms %.3f sw %d'
                      % (short[t], g, b, w(ws['segs'][0]), ws['swings'], w(ww['segs'][0]), ww['swings'],
                         w(wr['segs'][1]), wr['swings'], s0['max_abs_rate'], s0['rms_last10'], sn['swings']))
    print('\n== Torque-command noise: gyro noise x k (SITL x1 = 8.7e-4 rad/s), hover 120 s stick 0 and the '
          '0.4 N m weak wind; tq = torque command std, rms = rate rms [deg/s], sw = target reversals >90 deg')
    for t in VECTORED:
        for g in GAINS:
            for b in BANDS:
                parts = []
                for k in (1.0, 10.0, 30.0, 60.0):
                    h = get(t, b, g, 'hover', k)
                    ww = get(t, b, g, 'wind_weak', k)
                    if not (h and ww):
                        parts.append('x%-3g ?' % k)
                        continue
                    hs = h['segs'][0]
                    parts.append('x%-3g tq %.4f rms %.3f sw %3d / wind rms %.3f sw %3d'
                                 % (k, hs['torque_std'], hs['rms_rate'], h['swings'], ww['segs'][0]['rms_last10'],
                                    ww['swings']))
                print('  %-8s %-7s %-12s %s' % (short[t], g, b, ' | '.join(parts)))
    print('\n== Tilt servo speed (gyro noise x1): CA_AIRSHIP_TLT_R and servo 60 / 240 deg/s, and a 60 deg/s '
          'servo under the firmware\'s 120 (phys60)')
    for t in VECTORED:
        for g in GAINS:
            for b in BANDS:
                parts = []
                for label, slew, phys in (('120', None, None), ('60', 60.0, None), ('240', 240.0, None),
                                          ('phys60', None, 60.0)):
                    a = get(t, b, g, 'step_big', 1.0, slew, phys)
                    d = get(t, b, g, 'reverse', 1.0, slew, phys)
                    wr = get(t, b, g, 'wind_weak_rev', 1.0, slew, phys)
                    if not (a and d and wr):
                        parts.append('%s ?' % label)
                        continue
                    parts.append('%s: step os %.1f rest %s | rev os %.1f ts %s | wrev maxr %.2f rec %s sw %d'
                                 % (label, a['segs'][0]['overshoot_pct'], f(a['segs'][1].get('t_rest05')),
                                    d['segs'][1]['overshoot_pct'], f(d['segs'][1]['t_settle10']),
                                    wr['segs'][1]['max_abs_rate'], f(wr['segs'][1]['t_recover02']), wr['swings']))
                print('  %-8s %-7s %-12s %s' % (short[t], g, b, ' || '.join(parts)))


if __name__ == '__main__':
    main()
