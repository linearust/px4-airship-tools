#!/usr/bin/env python3
"""Supplement to xbeyond.py: full-stick steps (not in option A's report) and
the review's P 0.7 I 0.09 as a third comparison set, to separate what P and
what I do to overshoot and stopping."""
import json, os, sys, multiprocessing as mp
sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import xbeyond, xgate
def job(spec):
    kind, t, params, seed = spec
    stick = {'step10': 1.0, 'step05': 0.5, 'step02': 0.2}[kind]
    return {'kind': kind, 'target': t, 'params': params, 'act': None, 'seed': seed,
            'res': xbeyond.step(t, params, None, seed, stick)}
if __name__ == '__main__':
    specs = []
    for P, I in ((0.5, 0.09), (0.7, 0.2), (0.7, 0.09)):
        params = {'AS_YAWRATE_P': P, 'AS_YAWRATE_I': I, 'AS_YR_INT_LIM': 0.3}
        for t in xgate.TARGETS:
            kinds = ['step10'] + (['step05', 'step02'] if (P, I) == (0.7, 0.09) else [])
            for k in kinds:
                specs.append((k, t, params, 11))
    with mp.Pool(int(os.environ.get('W', 3))) as pool, open(os.path.join(HERE, 'beyond2.jsonl'), 'w') as fh:
        for r in pool.imap_unordered(job, specs):
            fh.write(json.dumps(r) + '\n'); fh.flush()
    print('done')
