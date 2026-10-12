#!/usr/bin/env python3
"""Cloudship gate with a first-order tail lag (the shared chain has none;
option E leaves this chain untouched, so this is the sensitivity of the
gains E defers to, P 0.5 vs P 0.7). Nominal timing member, gyro noise
seeds 1-5, tau 0 / 0.05 / 0.1 s.

    python3 xc_cloud_lag.py > xc_cloud_lag_out.txt
"""
import collections
import math
import multiprocessing as mp
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True

CS = ['gz_cloudship', 'gz_cloudship_am_fin_drag']


def one(job):
    target, P, tau, seed = job
    import xc_model as xm
    import gate
    import reproduce as rp
    loop = xm.build(target, 'base', params={'AS_YAWRATE_P': P, 'AS_YAWRATE_I': 0.09}, noise=True, seed=seed,
                    cloud_tau=tau)
    od = os.path.join(HERE, 'tmp_out', str(os.getpid()))
    os.makedirs(od, exist_ok=True)
    res = gate.run_gate(loop, xm.model_name(target), out_dir=od)
    return job, rp.parse(res.gate_lines)


if __name__ == '__main__':
    jobs = [(t, P, tau, s) for t in CS for P in (0.5, 0.7) for tau in (None, 0.05, 0.1) for s in (1, 2, 3, 4, 5)]
    with mp.Pool(int(os.environ.get('WORKERS', '3'))) as pool:
        out = pool.map(one, jobs, chunksize=1)
    by = collections.defaultdict(list)
    for (t, P, tau, s), parsed in out:
        by[(t, P, tau)].append(parsed)
    SP = (0.5 - 0.1) / 0.9 * 15.0
    for key in sorted(by, key=lambda k: (k[0], k[1], k[2] or 0)):
        rs = by[key]
        fails = collections.Counter(k for p in rs for k, v in p.items() if not v['ok'])
        pk = [p['Acro track']['peak'] for p in rs]
        spk = [p['Stab track']['peak'] for p in rs]
        rel = [p['Acro release']['settle'] for p in rs]
        srel = [p['Stab release']['settle'] for p in rs]
        brk = [p['Acro brake']['end'] for p in rs]
        print('%-26s P %.1f tau %-5s  fails %s | A.peak %.2f..%.2f (thr %.2f) S.peak %.2f..%.2f | A.rel %s S.rel %s | brake end %s'
              % (key[0], key[1], key[2], dict(fails) or '-', min(pk), max(pk), 1.4 * SP, min(spk), max(spk),
                 rel, srel, brk))
