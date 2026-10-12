#!/usr/bin/env python3
"""Run the gate model over configs x plants x ensemble members, in parallel.

    python3 sweep.py OUT.jsonl --configs "0.7,0.09,0.0;0.7,0.09,0.4" [--plants all|i,j]
        [--members screen|full|seeds] [--seeds 1,2,3,4,5] [--act '{"slew": 60}'] [--pool 10]

members:
  screen   nominal timing, gyro noise off (one run per config x plant)
  noisy    nominal timing, gyro noise on, each seed
  full     every timing member of the reviewers' ensemble (gain_sweep.py:
           nominal, stick 0/50 ms, RTF 0.98/1.02, P .3153/.3177, cmd 30 ms)
           x gyro noise on x each seed
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
import ffcommon as fc  # noqa: E402


def clean(x):
    if isinstance(x, float) and not math.isfinite(x):
        return None
    return x


def one(job):
    target, (P, I, FF), (label, tkw), seed, act = job
    t0 = time.time()
    try:
        res, loop = fc.run_gate(target, P, I, FF, timing=tkw, noise=seed is not None, seed=seed or 1, **act)
        m = fc.metrics(res, target)
        m = {k: {f: clean(v) for f, v in d.items()} for k, d in m.items()}
        err = None
        status = res.status
    except Exception as e:  # keep the sweep going; record the failure
        m, err, status = {}, repr(e), None
    return {'target': target, 'P': P, 'I': I, 'FF': FF, 'member': label, 'seed': seed, 'act': act,
            'status': status, 'checks': m, 'err': err, 'secs': round(time.time() - t0, 1)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--configs', required=True)
    ap.add_argument('--plants', default='all')
    ap.add_argument('--members', default='screen', choices=('screen', 'noisy', 'full'))
    ap.add_argument('--seeds', default='1,2,3,4,5')
    ap.add_argument('--act', default='{}')
    ap.add_argument('--pool', type=int, default=10)
    a = ap.parse_args()
    configs = [tuple(float(x) for x in c.split(',')) for c in a.configs.split(';') if c.strip()]
    plants = fc.TARGETS5 if a.plants == 'all' else [fc.TARGETS5[int(i)] for i in a.plants.split(',')]
    seeds = [int(s) for s in a.seeds.split(',')]
    act = json.loads(a.act)

    if a.members == 'screen':
        members = [(('nominal', {}), None)]
    elif a.members == 'noisy':
        members = [(('nominal', {}), s) for s in seeds]
    else:
        members = [(m, s) for m in fc.TIMING for s in seeds]

    jobs = [(t, c, m, s, act) for c in configs for t in plants for m, s in members]
    print('%d jobs' % len(jobs), flush=True)
    with mp.Pool(a.pool) as pool, open(a.out, 'a') as fh:
        for i, r in enumerate(pool.imap_unordered(one, jobs), 1):
            fh.write(json.dumps(r, ensure_ascii=False) + '\n')
            fh.flush()
            if i % 20 == 0:
                print('%d/%d' % (i, len(jobs)), flush=True)


if __name__ == '__main__':
    main()
