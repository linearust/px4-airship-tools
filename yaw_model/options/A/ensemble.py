#!/usr/bin/env python3
"""Phase 2: the gate on each plant of a vehicle for the ensemble of the model's
unknowns (gain_sweep.py's timing members) x gyro-noise seeds.

    python3 ensemble.py OUT.jsonl GROUP SEEDS P I LIM [P I LIM ...]
      GROUP  vectored | cloudship | all
      SEEDS  e.g. 1-5  (every member runs every seed)   or  'd8' (member i runs seed i+1)
"""
import json
import multiprocessing as mp
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import optA  # noqa: E402


def load(path):
    done = set()
    if os.path.exists(path):
        for line in open(path):
            r = json.loads(line)
            p = r['params']
            done.add((r['target'], p['AS_YAWRATE_P'], p['AS_YAWRATE_I'], p['AS_YR_INT_LIM'], r['member'], r['seed'],
                      json.dumps(r['act'])))
    return done


def main():
    out = os.path.join(HERE, sys.argv[1])
    group = sys.argv[2]
    seeds = sys.argv[3]
    act = None
    rest = sys.argv[4:]
    if rest and rest[0].startswith('act='):
        k, v = rest[0][4:].split(':')
        act = (k, float(v))
        rest = rest[1:]
    sets = [tuple(float(x) for x in rest[i:i + 3]) for i in range(0, len(rest), 3)]
    targets = {'vectored': optA.VECTORED, 'cloudship': optA.CLOUDSHIP, 'all': optA.TARGETS}[group]
    members = list(optA.MEMBERS)
    if seeds.startswith('n'):            # nominal member only, seed n<k>
        pairs = [('nominal', int(seeds[1:]))]
    elif seeds == 'd8':
        pairs = [(m, i + 1) for i, m in enumerate(members)]
    else:
        a, b = (int(x) for x in seeds.split('-'))
        pairs = [(m, s) for m in members for s in range(a, b + 1)]
    done = load(out)
    jobs = []
    for P, I, L in sets:
        params = {'AS_YAWRATE_P': P, 'AS_YAWRATE_I': I, 'AS_YR_INT_LIM': L}
        for t in targets:
            if act and ((act[0] in ('slew', 'servo', 'rotor')) != (t in optA.VECTORED)) and act[0] != 'noise':
                continue
            for m, s in pairs:
                if (t, P, I, L, m, s, json.dumps(act)) not in done:
                    jobs.append((t, params, m, s, act))
    print(len(jobs), 'jobs', flush=True)
    with mp.Pool(14) as pool, open(out, 'a') as fh:
        for n, r in enumerate(pool.imap_unordered(optA.gate_job, jobs), 1):
            fh.write(json.dumps(r, ensure_ascii=False) + '\n')
            fh.flush()
            if n % 50 == 0:
                print(n, flush=True)
    print('done', flush=True)


if __name__ == '__main__':
    main()
