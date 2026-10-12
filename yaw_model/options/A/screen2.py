#!/usr/bin/env python3
"""Phase 1 screen, pruned exactly: INT_LIM 0.5 on the whole P x I grid, then a
lower INT_LIM only where that run's integrator reached 0.9 x the lower limit
(below it the clamp never acts and the run is identical: same seed, same
arithmetic; the sampled maximum is read every 0.3 s and the integrator moves
at most I x error x 0.3 s between reads, hence the 0.9)."""
import itertools
import json
import multiprocessing as mp
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import optA  # noqa: E402

PS = [0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2]
IS = [0.03, 0.05, 0.07, 0.09, 0.11, 0.13, 0.15, 0.175, 0.2]
LS = [0.3, 0.2, 0.15]
OUT = os.path.join(HERE, 'screen.jsonl')


def load():
    d = {}
    if os.path.exists(OUT):
        for line in open(OUT):
            r = json.loads(line)
            p = r['params']
            d[(r['target'], p['AS_YAWRATE_P'], p['AS_YAWRATE_I'], p['AS_YR_INT_LIM'])] = r
    return d


def run(jobs):
    print(len(jobs), 'jobs', flush=True)
    with mp.Pool(14) as pool, open(OUT, 'a') as fh:
        for n, r in enumerate(pool.imap_unordered(optA.gate_job, jobs), 1):
            fh.write(json.dumps(r, ensure_ascii=False) + '\n')
            fh.flush()
            if n % 50 == 0:
                print(n, flush=True)


def main():
    done = load()
    run([(t, {'AS_YAWRATE_P': P, 'AS_YAWRATE_I': I, 'AS_YR_INT_LIM': 0.5}, 'nominal', 1, None)
         for P, I in itertools.product(PS, IS) for t in optA.TARGETS if (t, P, I, 0.5) not in done])
    done = load()
    jobs = []
    for P, I in itertools.product(PS, IS):
        for t in optA.TARGETS:
            base = done[(t, P, I, 0.5)]
            for L in LS:
                if base['integ_max'] >= 0.9 * L and (t, P, I, L) not in done:
                    jobs.append((t, {'AS_YAWRATE_P': P, 'AS_YAWRATE_I': I, 'AS_YR_INT_LIM': L}, 'nominal', 1, None))
    run(jobs)
    print('done', flush=True)


if __name__ == '__main__':
    main()
