#!/usr/bin/env python3
"""Run sitl_check's gate (the shared gate.py) on an Option D controller and
turn each verdict into a number with a margin to its threshold.

Margins are computed from the gate's own sample rows with sitl_check.py's
own formulas (settled :164-175, tail_mean :139-150, the verdict
expressions :371, :398-404, :413-426, :432-446), and each recomputed verdict
is asserted equal to the gate's printed verdict. Positive margin = inside
(passing side), negative = outside.

Undecided: a verdict that flips when the rate the check reads moves by the
model's stated uncertainty BAND[target] (deg/s): the rigid vectored peak and
brake 0.3-0.4 -> 0.4 (applied to every rate there); the Cloudship-complete
0.1-0.15 -> 0.15; the others 0.15 (the 0-0.12 noise-free peak bias, which
the noisy runs here remove, rounded up). Integrator checks: the model
matched SITL's wind integrators to 4e-4 (reproduction_planar.csv), so an
integ margin under 5e-4 is undecided. For the release check the band is
applied to the 1.0 deg/s settle threshold (settled() at 1.0 -/+ band).
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True

import option_d as od  # noqa: E402

BAND = {'gz_airship_vectored_independent': 0.4,
        'gz_airship_vectored_independent_am_fin_drag': 0.15,
        'gz_cloudship': 0.15,
        'gz_cloudship_am_fin_drag': 0.15,
        'gz_airship_vectored_independent_fin': 0.15}
INTEG_BAND = 5e-4

# gain_sweep.py's ENSEMBLE, minus its single noisy member: every member here
# runs with gyro noise on, over the seeds given
TIMING = [
    ('nominal', {}),
    ('stick 0', {'stick_delay_s': 0.0}),
    ('stick 50ms', {'stick_delay_s': 0.05}),
    ('rtf .98', {'rtf': 0.98}),
    ('rtf 1.02', {'rtf': 1.02}),
    ('P .3153', {'listener_s': (0.3153 - 0.3) / 3}),
    ('P .3177', {'listener_s': (0.3177 - 0.3) / 3}),
    ('cmd 0/30ms', {'command_delay_s': 0.03}),
]

SP = (0.5 - 0.1) / (1 - 0.1) * 15.0      # sitl_check :240 at AS_YAWRATE_MAX 15, MAN_DEADZONE 0.1


def settled(rows, threshold=1.0, hold=1.5):                     # sitl_check :164-175
    start = None
    for row in rows:
        if abs(row['rate']) < threshold:
            start = row['t'] if start is None else start
            if row['t'] - start >= hold:
                return start - rows[0]['t']
        else:
            start = None
    return None


def tail_mean(rows, seconds=2.0, key='rate', signed=False):     # sitl_check :139-150
    last = rows[-1]['t']
    v = [r[key] if signed else abs(r[key]) for r in rows if r['t'] >= last - seconds]
    return sum(v) / len(v)


def phase_rows(samples, pred):
    return [r for r in samples if pred(r['phase'])]


def metrics(res, target):
    import reproduce as rp
    band = BAND[target]
    parsed = rp.parse(res.gate_lines)
    s = res.samples
    out = {}

    def rec(key, ok, margins, undecided, **vals):
        gate_ok = parsed[key]['ok'] if key in parsed else None
        if gate_ok is not None and gate_ok != ok:
            raise AssertionError('%s %s: recomputed %s, gate %s' % (target, key, ok, gate_ok))
        out[key] = dict(ok=ok, margins=margins, undecided=undecided, **vals)

    # wind check and Manual: unchanged by any rule (before the loop runs); from the parser
    for key in ('wind check', 'Manual'):
        if key in parsed:
            out[key] = dict(ok=parsed[key]['ok'], margins={}, undecided=False)

    brake = phase_rows(s, lambda p: p == 'Acro 제동')
    if brake:
        end = brake[-1]['rate']
        ok = abs(end) < 6.0
        rec('Acro brake', ok, {'end': 6.0 - abs(end)}, abs(abs(end) - 6.0) < band, end=end)

    for mode, label in (('Acro', 'Acro'), ('Stabilized', 'Stab')):
        push = phase_rows(s, lambda p, m=mode: p == '%s 추종' % m)
        rel = phase_rows(s, lambda p, m=mode: p == '%s 놓음' % m)
        if push:
            key = '%s track' % label
            line = parsed.get(key, {}).get('line', '')
            clean = '정지 전제가 깨졌다' not in line
            mean = tail_mean(push, 3.0, signed=True)
            peak = max(r['rate'] for r in push)
            m_mean = 0.25 * SP - abs(mean - SP)
            m_peak = 1.4 * SP - peak
            ok = clean and abs(mean - SP) < 0.25 * SP and peak < 1.4 * SP
            und = clean and (abs(m_mean) < band or abs(m_peak) < band)
            rec(key, ok, {'mean': m_mean, 'peak': m_peak}, und, mean=mean, peak=peak, clean=clean)
        if rel:
            key = '%s release' % label
            turning = abs(rel[0]['rate']) > 0.5 * SP
            st = settled(rel)
            ok = turning and st is not None and st < 10.0
            lo, hi = settled(rel, 1.0 - band), settled(rel, 1.0 + band)
            ok_lo = turning and lo is not None and lo < 10.0
            ok_hi = turning and hi is not None and hi < 10.0
            und = (ok_lo != ok) or (ok_hi != ok)
            # the margin in seconds when settled; otherwise how far the end rate is above 1.0
            m = (10.0 - st) if st is not None else -(abs(rel[-1]['rate']) - 1.0)
            rec(key, ok, {'settle_s' if st is not None else 'end_rate': m}, und,
                settle=st, start=rel[0]['rate'], end=rel[-1]['rate'],
                reverse=min(r['rate'] for r in rel) if rel[0]['rate'] > 0 else max(r['rate'] for r in rel))

    # sitl_check :448-471: strong = authority / 54, weak = 0.4 strong, needed = torque / authority
    for key, pat, hold, needed in (('wind I on', 'I 켬: 적분기가', True, 1.0 / 54.0),
                                   ('wind I off', 'I 끔: 계속', False, 1.0 / 54.0),
                                   ('wind weak', '틸트 밴드 아래', True, 0.4 / 54.0)):
        rows = phase_rows(s, lambda p, q=pat: q in p and not p.endswith('회복'))
        if not rows:
            continue
        integ = tail_mean(rows, 2.0, 'integ')
        rate = tail_mean(rows, 2.0, 'rate')
        if hold:
            ok = rate < 1.0 and abs(integ - needed) < 0.5 * needed
            m = {'rate': 1.0 - rate, 'integ': 0.5 * needed - abs(integ - needed)}
            und = abs(m['rate']) < band or abs(m['integ']) < INTEG_BAND
        else:
            ok = rate > 1.0
            m = {'rate': rate - 1.0}
            und = abs(m['rate']) < band
        rec(key, ok, m, und, rate=rate, integ=integ, needed=needed)

    return out


def run_one(job):
    """job: dict(target, rule, gains, member, timing, seed, slew, tail_tau, tag)"""
    import gate
    target = job['target']
    loop = od.build(target, job['rule'], params=od.GAINS[job['gains']], noise=True, seed=job['seed'],
                    slew_deg=job.get('slew'), tail_tau=job.get('tail_tau'))
    out_dir = os.path.join(HERE, 'tmp_out', str(os.getpid()))
    os.makedirs(out_dir, exist_ok=True)
    res = gate.run_gate(loop, od.model_of(target), gate.Timing(**job['timing']), out_dir=out_dir)
    m = metrics(res, target)
    r = dict(job)
    r['status'] = res.status
    r['checks'] = m
    r['n_events'] = len(loop.controller.ctrl.events)
    return r


def run_jobs(jobs, path, workers=8):
    import multiprocessing as mp
    done = set()
    if os.path.exists(path):
        for line in open(path):
            done.add(json.loads(line)['id'])
    todo = [j for j in jobs if j['id'] not in done]
    print('%d jobs, %d already done, %d to run' % (len(jobs), len(done), len(todo)), flush=True)
    with mp.Pool(workers) as pool, open(path, 'a') as fh:
        for i, r in enumerate(pool.imap_unordered(run_one, todo), 1):
            fh.write(json.dumps(r, ensure_ascii=False) + '\n')
            fh.flush()
            if i % 20 == 0:
                print('  %d / %d' % (i, len(todo)), flush=True)
