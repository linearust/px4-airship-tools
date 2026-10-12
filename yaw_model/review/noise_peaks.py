#!/usr/bin/env python3
"""Peak bias from gyro noise: the gate's peak is the max of ~111 samples; the
default model is noise-free, SITL is not (gz IMU stddev 0.00087 rad/s)."""
import multiprocessing as mp, os, sys, json
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE)); sys.dont_write_bytecode = True
T = ['gz_airship_vectored_independent', 'gz_airship_vectored_independent_am_fin_drag', 'gz_cloudship',
     'gz_cloudship_am_fin_drag', 'gz_airship_vectored_independent_fin']
def one(job):
    t, seed = job
    import closed_loop as cl, gate, reproduce as rp
    kw = {} if seed is None else {'noise': True, 'seed': seed}
    kw.setdefault('params', rp.record_params(t)[0])  # the gains the chain7/chain9 records were flown with (record_gains)
    loop = cl.build(t, **kw)
    od = os.path.join(HERE, 'tmp_out', str(os.getpid())); os.makedirs(od, exist_ok=True)
    res = gate.run_gate(loop, (cl.TARGETS.get(t) or cl.EXTRA_TARGETS[t])[0], out_dir=od)
    p = rp.parse(res.gate_lines)
    return t, seed, p['Acro track']['peak'], p['Stab track']['peak'], p['Acro track']['ok'], p['Acro brake']['end']
jobs = [(t, s) for t in T for s in (None, 1, 2, 3, 4, 5, 6)]
with mp.Pool(14) as pool:
    res = pool.map(one, jobs)
import collections
by = collections.defaultdict(list)
for t, s, a, b, ok, br in res:
    by[t].append((s, a, b, ok, br))
for t in T:
    rows = by[t]
    base = [r for r in rows if r[0] is None][0]
    noisy = [r for r in rows if r[0] is not None]
    na = [r[1] for r in noisy]; nb = [r[2] for r in noisy]
    print('%-45s noise-free Acro %.2f Stab %.2f | 6 noisy seeds: Acro %.2f..%.2f (mean %.3f) Stab %.2f..%.2f (mean %.3f); Acro verdicts %s; brake end %s'
          % (t, base[1], base[2], min(na), max(na), sum(na)/6, min(nb), max(nb), sum(nb)/6,
             ''.join('P' if r[3] else 'F' for r in noisy), ' '.join('%.1f' % r[4] for r in noisy)))
