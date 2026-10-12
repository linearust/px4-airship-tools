#!/usr/bin/env python3
"""Two identities, gate lines compared exactly:
  1. xc_model base == the shared closed_loop.build, all five plants,
     noise-free and gyro-noise seed 3;
  2. xc_model <variant> == options/E/e_model <variant> for every non-base
     variant on the three vectored plants, noise-free at P 0.5 and noise
     seed 2 at P 0.7.
"""
import multiprocessing as mp
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'E'))
sys.dont_write_bytecode = True

ALL = ['gz_airship_vectored_independent', 'gz_airship_vectored_independent_am_fin_drag',
       'gz_airship_vectored_independent_fin', 'gz_cloudship', 'gz_cloudship_am_fin_drag']
VEC = ALL[:3]
VARS = ['report_hold', 'report_band', 'report_all', 'deliver', 'deliver_report']


def one(job):
    kind, target, variant, P, seed = job
    import xc_model as xm
    import gate
    params = {'AS_YAWRATE_P': P, 'AS_YAWRATE_I': 0.09}
    kw = {'noise': True, 'seed': seed} if seed else {}
    if kind == 'shared':
        import closed_loop as cl
        loop = cl.build(target, params=params, **kw)
    elif kind == 'E':
        import e_model as em
        loop = em.build(target, variant, params=params, **kw)
    else:
        loop = xm.build(target, variant, params=params, **kw)
    od = os.path.join(HERE, 'tmp_out', str(os.getpid()))
    os.makedirs(od, exist_ok=True)
    t0 = time.time()
    res = gate.run_gate(loop, xm.model_name(target), out_dir=od)
    return job, res.gate_lines, time.time() - t0


if __name__ == '__main__':
    jobs = []
    for t in ALL:
        for seed in (None, 3):
            jobs += [('shared', t, 'base', 0.5, seed), ('xc', t, 'base', 0.5, seed)]
    for t in VEC:
        for v in VARS:
            for P, seed in ((0.5, None), (0.7, 2)):
                jobs += [('E', t, v, P, seed), ('xc', t, v, P, seed)]
    with mp.Pool(int(os.environ.get('WORKERS', '14'))) as pool:
        out = pool.map(one, jobs, chunksize=1)
    by = {j: lines for j, lines, _ in out}
    bad = 0
    for t in ALL:
        for seed in (None, 3):
            same = by[('shared', t, 'base', 0.5, seed)] == by[('xc', t, 'base', 0.5, seed)]
            bad += not same
            print('base vs shared  %-45s seed %-4s identical %s' % (t, seed, same))
    for t in VEC:
        for v in VARS:
            for P, seed in ((0.5, None), (0.7, 2)):
                a = by[('E', t, v, P, seed)]
                b = by[('xc', t, v, P, seed)]
                same = a == b
                bad += not same
                print('xc vs E  %-45s %-14s P %.1f seed %-4s identical %s' % (t, v, P, seed, same))
                if not same:
                    for x, y in zip(a, b):
                        if x != y:
                            print('    E : ' + x)
                            print('    xc: ' + y)
    print('ALL IDENTICAL' if not bad else 'DIFFERENT: %d' % bad)
