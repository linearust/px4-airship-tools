#!/usr/bin/env python3
"""Gate runs for the cross-check, with my own margin computation from the
gate's sample rows (sitl_check.py :139-175, :371, :398-426, :432-446) and the
gate's own verdicts from report.results().

    python3 xgate.py OUT.jsonl CONFIG [CONFIG ...]
CONFIG = rule:P[:full|lite][:variant=value]; variant alloc_slew|servo|tail_tau
"""
import json
import math
import multiprocessing as mp
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True
import xlib  # noqa: E402

sys.path.insert(0, os.path.join(xlib.MODEL, 'review'))
from gain_sweep import ENSEMBLE  # noqa: E402

TIMING = [(label, tkw) for label, tkw, bkw in ENSEMBLE if not bkw]      # the 8 timing members
assert len(TIMING) == 8

SP = (0.5 - 0.1) / (1 - 0.1) * 15.0
BAND = {'gz_airship_vectored_independent': 0.4}       # 0.3-0.4 stated for rigid vectored peak/brake
BAND_DEFAULT = 0.15
INTEG_BAND = 5e-4


def settled(rows, thr=1.0, hold=1.5):
    start = None
    for r in rows:
        if abs(r['rate']) < thr:
            if start is None:
                start = r['t']
            if r['t'] - start >= hold:
                return start - rows[0]['t']
        else:
            start = None
    return None


def tail(rows, sec, key, signed=False):
    last = rows[-1]['t']
    v = [(r[key] if signed else abs(r[key])) for r in rows if r['t'] >= last - sec]
    return sum(v) / len(v)


def my_metrics(samples, target, rc):
    band = BAND.get(target, BAND_DEFAULT)
    ph = {}
    for r in samples:
        ph.setdefault(r['phase'], []).append(r)
    out = {}
    b = ph.get('Acro 제동')
    if b:
        end = b[-1]['rate']
        out['brake'] = dict(ok=abs(end) < 6, m=6 - abs(end), und=abs(6 - abs(end)) < band, end=end)
    for mode, lab in (('Acro', 'A'), ('Stabilized', 'S')):
        push, rel = ph.get(mode + ' 추종'), ph.get(mode + ' 놓음')
        if push:
            mean = tail(push, 3.0, 'rate', True)
            peak = max(r['rate'] for r in push)
            mm, mp_ = 0.25 * SP - abs(mean - SP), 1.4 * SP - peak
            out[lab + 'track'] = dict(ok=mm > 0 and mp_ > 0, m_mean=mm, m_peak=mp_, peak=peak, mean=mean,
                                      und=abs(mm) < band or abs(mp_) < band,
                                      integ_end=push[-1]['integ'])
        if rel:
            turning = abs(rel[0]['rate']) > 0.5 * SP
            st = settled(rel)
            ok = turning and st is not None and st < 10
            alt = []
            for thr in (1.0 - band, 1.0 + band):
                s2 = settled(rel, thr)
                alt.append(turning and s2 is not None and s2 < 10)
            out[lab + 'rel'] = dict(ok=ok, settle=st, m=(10 - st) if st is not None else None,
                                    end=rel[-1]['rate'], start=rel[0]['rate'], und=any(a != ok for a in alt),
                                    minrate=min(r['rate'] for r in rel), integ0=rel[0]['integ'])
    for key, pat, hold, need in (('wOn', 'I 켬: 적분기가', True, 1 / 54), ('wOff', 'I 끔: 계속', False, 1 / 54),
                                 ('wWk', '틸트 밴드 아래', True, 0.4 / 54)):
        rows = [r for p, rs in ph.items() if pat in p and not p.endswith('회복') for r in rs]
        if not rows:
            continue
        rows.sort(key=lambda r: r['t'])
        integ, rate = tail(rows, 2.0, 'integ'), tail(rows, 2.0, 'rate')
        if hold:
            mr, mi = 1 - rate, 0.5 * need - abs(integ - need)
            out[key] = dict(ok=mr > 0 and mi > 0, m_rate=mr, m_integ=mi, rate=rate, integ=integ,
                            und=abs(mr) < band or abs(mi) < INTEG_BAND)
        else:
            out[key] = dict(ok=rate > 1, m_rate=rate - 1, rate=rate, und=abs(rate - 1) < band)
    out['_rc'] = dict(n_release=getattr(rc, 'n_release', 0), n_deflect=getattr(rc, 'n_deflect', 0),
                      releases=getattr(rc, 'release_log', []))
    return out


NAMES = {'바람(외부 토크)이 기체를 민다': 'wchk', 'Manual 에서 스틱이 곧 토크': 'man',
         'Acro 가 남은 회전을 잡는다': 'brake', 'Acro: 스틱이 지시한 각속도를 따라간다': 'Atrack',
         'Acro: 스틱을 놓으면 선다': 'Arel', 'Stabilized: 스틱이 지시한 각속도를 따라간다': 'Strack',
         'Stabilized: 스틱을 놓으면 선다': 'Srel'}


def gate_name(n):
    if n in NAMES:
        return NAMES[n]
    if 'I 켬: 적분기가' in n:
        return 'wOn'
    if 'I 끔' in n:
        return 'wOff'
    if '틸트 밴드 아래' in n:
        return 'wWk'
    return n


def run(job):
    import gate
    t = job['target']
    loop = xlib.build(t, job['rule'], job['P'], job.get('I', 0.09), True, job['seed'],
                      tuple(job['variant']) if job.get('variant') else None)
    od = os.path.join(HERE, 'tmp_out', str(os.getpid()))
    os.makedirs(od, exist_ok=True)
    res = gate.run_gate(loop, xlib.spec(t)[0], gate.Timing(**job['timing']), out_dir=od)
    verdicts = {gate_name(n): bool(ok) for n, ok in res.results}
    m = my_metrics(res.samples, t, loop.controller.ctrl.rc)
    # my recomputed verdicts must equal the gate's
    for k, v in m.items():
        if k.startswith('_'):
            continue
        g = verdicts.get({'brake': 'brake', 'Atrack': 'Atrack', 'Arel': 'Arel', 'Strack': 'Strack',
                          'Srel': 'Srel'}.get(k, k))
        if g is not None and g != v['ok']:
            m[k]['MISMATCH_gate'] = g
    r = dict(job)
    r.update(status=res.status, verdicts=verdicts, m=m)
    return r


def members(mode):
    if mode == 'full':
        return [(l, tk, s) for l, tk in TIMING for s in range(1, 6)]
    if mode == 'lite':
        return [(l, tk, i % 5 + 1) for i, (l, tk) in enumerate(TIMING)] + [(TIMING[0][0], TIMING[0][1], s) for s in (2, 3, 4, 5)]
    if mode.startswith('nom'):
        n = int(mode[3:] or 3)
        return [(TIMING[0][0], TIMING[0][1], s) for s in range(1, n + 1)]
    raise ValueError(mode)


def jobs_for(spec):
    parts = spec.split(':')
    rule, P = parts[0], float(parts[1])
    mode = parts[2] if len(parts) > 2 else 'full'
    variant = None
    if len(parts) > 3:
        k, v = parts[3].split('=')
        variant = (k, float(v))
    out = []
    for t in xlib.T5:
        if variant and variant[0] in ('alloc_slew', 'servo') and xlib.spec(t)[1] != 'vectored':
            continue
        if variant and variant[0] == 'tail_tau' and xlib.spec(t)[1] != 'cloudship':
            continue
        for label, tk, seed in members(mode):
            out.append(dict(id='%s|%s|%s|%s|%d' % (spec, t, label, mode, seed), spec=spec, rule=rule, P=P,
                            target=t, member=label, timing=tk, seed=seed, variant=variant))
    return out


if __name__ == '__main__':
    path = os.path.join(HERE, sys.argv[1])
    jobs = [j for s in sys.argv[2:] for j in jobs_for(s)]
    done = set()
    if os.path.exists(path):
        done = {json.loads(l)['id'] for l in open(path)}
    todo = [j for j in jobs if j['id'] not in done]
    print('%d jobs, %d to run' % (len(jobs), len(todo)), flush=True)
    with mp.Pool(int(os.environ.get('W', 12))) as pool, open(path, 'a') as fh:
        for i, r in enumerate(pool.imap_unordered(run, todo), 1):
            fh.write(json.dumps(r, ensure_ascii=False) + '\n')
            fh.flush()
            if i % 50 == 0:
                print('  %d/%d' % (i, len(todo)), flush=True)
    print('done', flush=True)
