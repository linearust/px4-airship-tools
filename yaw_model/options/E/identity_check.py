#!/usr/bin/env python3
"""e_model.build(band='base') must give the shared model's gate lines exactly
(noise-free and one noisy seed), on every plant."""
import multiprocessing as mp
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True

TARGETS = ['gz_airship_vectored_independent', 'gz_airship_vectored_independent_am_fin_drag',
           'gz_airship_vectored_independent_fin', 'gz_cloudship', 'gz_cloudship_am_fin_drag']


def one(job):
    target, which, noise = job
    import e_model as em
    import closed_loop as cl
    import gate
    t0 = time.time()
    import reproduce as rp
    kw = {'noise': True, 'seed': 3} if noise else {}
    kw['params'] = rp.record_params(target)[0]  # the gains the chain7/chain9 records were flown with (record_gains)
    loop = cl.build(target, **kw) if which == 'shared' else em.build(target, 'base', **kw)
    od = os.path.join(HERE, 'tmp_out', str(os.getpid()))
    os.makedirs(od, exist_ok=True)
    res = gate.run_gate(loop, em.model_name(target), out_dir=od)
    return target, which, noise, res.gate_lines, time.time() - t0


if __name__ == '__main__':
    jobs = [(t, w, n) for t in TARGETS for w in ('shared', 'e') for n in (False, True)]
    with mp.Pool(10) as pool:
        out = pool.map(one, jobs)
    by = {(t, w, n): (lines, s) for t, w, n, lines, s in out}
    bad = 0
    for t in TARGETS:
        for n in (False, True):
            a, sa = by[(t, 'shared', n)]
            b, sb = by[(t, 'e', n)]
            same = a == b
            bad += not same
            print('%-45s noise %-5s identical %s  (%.0f s / %.0f s)' % (t, n, same, sa, sb))
    print('IDENTICAL' if not bad else 'DIFFERENT: %d' % bad)
