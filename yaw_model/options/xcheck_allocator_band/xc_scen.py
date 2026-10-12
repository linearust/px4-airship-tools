#!/usr/bin/env python3
"""Beyond-gate scenarios for the option-E cross-check, on xc_model (own
driver and metrics, not options/E/scenarios.py).

    python3 xc_scen.py --out xc_scen.jsonl [--workers 4]
    python3 xc_scen.py --report xc_scen.jsonl

Driver: stick 0, Manual, arm, 1 s; Acro, 5 s; then the segments. The
recorded rate is the plant's TRUE body yaw rate (FRD, deg/s), not the gyro.
Wind: a body FRD yaw moment (the gate's world-z couple on a level hull).
Gyro noise x1 (SITL, 8.7e-4 rad/s) unless the job says otherwise.
"""
import argparse
import json
import math
import multiprocessing as mp
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True

DEG = 180.0 / math.pi
GYRO_NOISE = 0.0008726646
VEC = ['gz_airship_vectored_independent', 'gz_airship_vectored_independent_am_fin_drag',
       'gz_airship_vectored_independent_fin']
CS = ['gz_cloudship', 'gz_cloudship_am_fin_drag']
GAINS = {'P0.5': (0.5, 0.09), 'P0.7': (0.7, 0.09)}


def stick_for(rate_deg):
    return math.copysign(0.1 + 0.9 * abs(rate_deg) / 15.0, rate_deg) if rate_deg else 0.0


SP = 6.6667
SCEN = {
    'step_big': [(60, SP, 0.0), (60, 0.0, 0.0)],
    'step_small': [(60, 1.0, 0.0), (60, 0.0, 0.0)],
    'reverse': [(40, SP, 0.0), (40, -SP, 0.0), (40, 0.0, 0.0)],
    'wind_strong': [(80, 0.0, -1.0), (40, 0.0, 0.0)],
    'wind_weak': [(80, 0.0, -0.4), (40, 0.0, 0.0)],
    'wind_weak_rev': [(80, 0.0, -0.4), (80, 0.0, 0.4), (40, 0.0, 0.0)],
    'hover': [(120, 0.0, 0.0)],
}


def run(job):
    target, variant, gname, scen, opts = job
    import xc_model as xm
    import planar
    planar.GYRO_NOISE = GYRO_NOISE * opts.get('gyro_x', 1.0)
    P, I = GAINS[gname]
    loop = xm.build(target, variant, params={'AS_YAWRATE_P': P, 'AS_YAWRATE_I': I}, noise=True,
                    seed=opts.get('seed', 11), slew_deg=opts.get('slew'), cloud_tau=opts.get('tau'),
                    torque_noise=opts.get('tq_noise', 0.0))
    loop.set_stick(0.0)
    loop.set_mode('manual')
    loop.arm()
    loop.advance(1.0)
    loop.set_mode('acro')
    loop.advance(5.0)
    pods = loop.actuation.act.alloc.pods if hasattr(loop.actuation, 'act') else None
    rec = []   # (t, rate_deg, torque, integ, jumps)
    segs = []
    t = 0.0
    for dur, sp, wind in SCEN[scen]:
        loop.set_stick(stick_for(sp))
        loop.set_wind(wind)
        segs.append((t, t + dur, sp, wind))
        for _ in range(int(round(dur / 0.004))):
            loop.advance(0.004)
            t += 0.004
            rec.append((t, loop.plant.rate * DEG, loop._torque, loop.controller.ctrl.integ,
                        sum(p.target_jumps for p in pods) if pods else 0))
    return {'target': target, 'variant': variant, 'gains': gname, 'scen': scen, 'opts': opts,
            'segs': [seg_metrics(rec, a, b, sp, wind, (segs[i - 1][2], segs[i - 1][3]) if i else (0.0, 0.0))
                     for i, (a, b, sp, wind) in enumerate(segs)],
            'jumps': rec[-1][4] - (rec[0][4] if rec else 0)}


def first_hold(ts, ok, hold):
    start = None
    for t, g in zip(ts, ok):
        if g:
            start = t if start is None else start
            if t - start >= hold:
                return start
        else:
            start = None
    return None


def seg_metrics(rec, a, b, sp, wind, prev):
    rows = [r for r in rec if a < r[0] <= b + 1e-9]
    ts = [r[0] - a for r in rows]
    rr = [r[1] for r in rows]
    tq = [r[2] for r in rows]
    m = {'a': a, 'b': b, 'sp': sp, 'wind': wind, 'jumps': rows[-1][4] - rows[0][4],
         'end_rate': rr[-1], 'integ_end': rows[-1][3]}
    last10 = [r for t, r in zip(ts, rr) if t >= ts[-1] - 10.0]
    m['mean_last10'] = sum(last10) / len(last10)
    m['rms_last10'] = math.sqrt(sum(x * x for x in last10) / len(last10))
    if sp:
        s = 1.0 if sp > 0 else -1.0
        m['overshoot_pct'] = 100.0 * (max(s * r for r in rr) - abs(sp)) / abs(sp)
        out = [t for t, r in zip(ts, rr) if abs(r - sp) > 0.1 * abs(sp)]
        m['t_settle10'] = out[-1] if out and out[-1] < ts[-1] - 1.0 else (0.0 if not out else None)
    elif prev[0] and not wind:
        s = 1.0 if prev[0] > 0 else -1.0
        m['t_rest05'] = first_hold(ts, [abs(r) < 0.5 for r in rr], 2.0)
        m['undershoot'] = max(0.0, max(-s * r for r in rr))
    if wind or prev[1]:
        m['max_abs'] = max(abs(r) for r in rr)
        # recovery measured AFTER the peak (a window that is quiet before the
        # disturbance has built up the rate is not a recovery)
        k_pk = max(range(len(rr)), key=lambda k: abs(rr[k]))
        m['t_peak'] = ts[k_pk]
        m['t_rec02'] = first_hold(ts[k_pk:], [abs(r) < 0.2 for r in rr[k_pk:]], 3.0)
        m['t_rec02_from0'] = first_hold(ts, [abs(r) < 0.2 for r in rr], 3.0)
    if not sp and not wind:
        mu = sum(tq) / len(tq)
        m['tq_std'] = math.sqrt(sum((x - mu) ** 2 for x in tq) / len(tq))
        m['rms'] = math.sqrt(sum(x * x for x in rr) / len(rr))
    return m


def jobs():
    j = []
    core = ['step_big', 'step_small', 'reverse', 'wind_strong', 'wind_weak', 'wind_weak_rev']
    for t in VEC:
        for g in GAINS:
            for v in ('base', 'report_band', 'deliver'):
                for s in core:
                    j.append((t, v, g, s, {}))
            j.append((t, 'report_all', g, 'wind_weak', {}))
            # tilt slew 60 / 240 (firmware + servo)
            for slew in (60.0, 240.0):
                for s in ('step_big', 'reverse', 'wind_weak_rev'):
                    j.append((t, 'base', g, s, {'slew': slew}))
                j.append((t, 'deliver', g, 'wind_weak_rev', {'slew': slew}))
            # noise: gyro x30 and direct torque-command noise
            for v in ('base', 'deliver'):
                j.append((t, v, g, 'hover', {'gyro_x': 30.0}))
                j.append((t, v, g, 'hover', {'tq_noise': 0.004}))
                j.append((t, v, g, 'hover', {'tq_noise': 0.008}))
    j.append(('gz_airship_vectored_independent_am_fin_drag', 'report_band', 'P0.5', 'reverse', {'slew': 60.0}))
    j.append(('gz_airship_vectored_independent_am_fin_drag', 'base', 'P0.5', 'reverse', {'slew': 60.0}))
    # Cloudship: step and wind with and without a tail lag (E: not applicable)
    for t in CS:
        for g in GAINS:
            for tau in (None, 0.05, 0.1):
                for s in ('step_big', 'wind_strong'):
                    j.append((t, 'base', g, s, {'tau': tau} if tau else {}))
    return j


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(HERE, 'xc_scen.jsonl'))
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--report', default=None)
    args = ap.parse_args()
    if args.report:
        report(args.report)
        return
    todo = jobs()
    todo.sort(key=lambda x: -sum(d for d, _, _ in SCEN[x[3]]))
    start = time.time()
    print('%d runs' % len(todo), flush=True)
    with mp.Pool(args.workers) as pool, open(args.out, 'w') as fh:
        for i, r in enumerate(pool.imap_unordered(run, todo, chunksize=1)):
            fh.write(json.dumps(r) + '\n')
            fh.flush()
            if (i + 1) % 10 == 0:
                print('%d/%d %.0f s' % (i + 1, len(todo), time.time() - start), flush=True)
    print('done %.0f s' % (time.time() - start), flush=True)


def report(path):
    rows = [json.loads(line) for line in open(path)]
    short = {'gz_airship_vectored_independent': 'rigid', 'gz_airship_vectored_independent_am_fin_drag': 'complete',
             'gz_airship_vectored_independent_fin': '_fin', 'gz_cloudship': 'cs rigid',
             'gz_cloudship_am_fin_drag': 'cs complete'}

    def f(x, fmt='%.1f'):
        return 'never' if x is None else fmt % x

    for r in sorted(rows, key=lambda r: (r['scen'], r['target'], r['gains'], r['variant'], json.dumps(r['opts']))):
        s = r['segs']
        head = '%-13s %-11s %-4s %-14s %-22s jumps %4d |' % (r['scen'], short[r['target']], r['gains'], r['variant'],
                                                           json.dumps(r['opts']), r['jumps'])
        parts = []
        for m in s:
            p = []
            if 'overshoot_pct' in m:
                p.append('os %.1f%% ts10 %s' % (m['overshoot_pct'], f(m['t_settle10'])))
            if 't_rest05' in m:
                p.append('rest %s us %.2f end %.2f' % (f(m['t_rest05']), m['undershoot'], m['end_rate']))
            if 'max_abs' in m:
                p.append('max %.2f@%.1f rec %s (from0 %s) mean10 %.2f rms10 %.3f' % (m['max_abs'], m['t_peak'], f(m['t_rec02']), f(m['t_rec02_from0']), m['mean_last10'], m['rms_last10']))
            if 'tq_std' in m and 'max_abs' not in m and 't_rest05' not in m:
                p.append('tq_std %.4f rms %.3f' % (m['tq_std'], m['rms']))
            p.append('I %.4f j %d' % (m['integ_end'], m['jumps']))
            parts.append(' '.join(p))
        print(head + ' || '.join(parts))


if __name__ == '__main__':
    main()
