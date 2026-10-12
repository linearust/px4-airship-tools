#!/usr/bin/env python3
"""Gate runs of option C, in parallel, one JSON line per run.

    python3 scan.py coarse     # b 0.5..1.0 x 8 gain sets x 5 plants, nominal timing, noise seed 1
    python3 scan.py full       # the finalists x 5 plants x 8 timing members x 5 noise seeds
    python3 scan.py act        # the finalists under the actuator variants, nominal timing, 5 seeds

Every run: gyro noise ON (gz stddev 8.7e-4 rad/s), the gate on the model
(gate.run_gate = sitl_check.py's own main()).
"""
import json
import multiprocessing as mp
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True

import optc  # noqa: E402

BS = [0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
GAINS = [(0.5, 0.09), (0.7, 0.09), (0.7, 0.05), (1.0, 0.09), (0.5, 0.2), (0.7, 0.2), (1.0, 0.2), (0.35, 0.09)]
# finalists, chosen from coarse_summary.txt: FULL gets the whole cross
# (8 timing members x 5 seeds = 40 runs per cell), LIGHT the 8 timing members
# with one seed each (1..8) plus the nominal member on seeds 2..5 (12 runs)
# (the machine ran at load 50-90 from other jobs, so every finalist got the
# 12-run LIGHT ensemble; runs of the first FULL attempt are kept as extras)
FULL = []
LIGHT = [(0.7, 0.09, 1.0), (0.7, 0.09, 0.9), (0.7, 0.2, 1.0), (0.7, 0.2, 0.8),
         (0.5, 0.09, 1.0), (0.5, 0.09, 0.694), (0.7, 0.2, 0.9)]
# gains-alone comparators for the one place b added gate margin (vec rigid peak at P 0.7 I 0.2)
GAINS_ONLY = [(0.8, 0.2, 1.0), (0.7, 0.15, 1.0)]
ACT_SETTINGS = [(0.7, 0.09, 1.0), (0.7, 0.09, 0.9), (0.7, 0.2, 1.0), (0.7, 0.2, 0.8)]
ACTS = [None, ('slew', 60.0), ('slew', 240.0), ('servo', 60.0), ('tail_tau', 0.025), ('tail_tau', 0.05)]
SEEDS = [1, 2, 3, 4, 5]


def clean(x):
    if isinstance(x, float):
        return round(x, 5)
    if isinstance(x, dict):
        return {k: clean(v) for k, v in x.items()}
    return x


def one(job):
    target, P, I, b, member, tkw, seed, act = job
    import gate
    res, m = optc.run_gate(target, timing=gate.Timing(**tkw), P=P, I=I, b=b, seed=seed, act=act)
    return {'target': target, 'P': P, 'I': I, 'b': b, 'member': member, 'seed': seed,
            'act': list(act) if act else None, 'status': res.status, 'checks': clean(m)}


def jobs(kind):
    out = []
    if kind == 'coarse':
        for t in optc.TARGETS:
            for P, I in GAINS:
                for b in BS:
                    out.append((t, P, I, b, 'nominal', {}, 1, None))
    elif kind == 'full':
        for t in optc.TARGETS:
            for P, I, b in FULL:
                for label, tkw in optc.ensemble():
                    for s in SEEDS:
                        out.append((t, P, I, b, label, tkw, s, None))
            for P, I, b in LIGHT:
                for n, (label, tkw) in enumerate(optc.ensemble(), 1):
                    out.append((t, P, I, b, label, tkw, n, None))
                for s in SEEDS[1:]:
                    out.append((t, P, I, b, 'nominal', {}, s, None))
    elif kind == 'gains':
        for t in optc.TARGETS:
            for P, I, b in GAINS_ONLY:
                for n, (label, tkw) in enumerate(optc.ensemble(), 1):
                    out.append((t, P, I, b, label, tkw, n, None))
                for s in SEEDS[1:]:
                    out.append((t, P, I, b, 'nominal', {}, s, None))
    elif kind == 'act':
        for P, I, b in ACT_SETTINGS:
            for act in ACTS[1:]:
                vectored = act[0] in ('slew', 'servo')
                for t in optc.TARGETS:
                    if vectored != ('vectored' in t):
                        continue
                    for s in SEEDS[:2]:
                        out.append((t, P, I, b, 'nominal', {}, s, act))
    else:
        raise SystemExit(kind)
    return out


def main():
    kind = sys.argv[1]
    procs = int(os.environ.get('PROCS', '6'))
    todo = jobs(kind)
    path = os.path.join(HERE, 'scan_%s.jsonl' % kind)
    done = set()

    if os.path.exists(path):                 # resume
        for line in open(path):
            r = json.loads(line)
            done.add((r['target'], r['P'], r['I'], r['b'], r['member'], r['seed'],
                      tuple(r['act']) if r['act'] else None))

    todo = [j for j in todo if (j[0], j[1], j[2], j[3], j[4], j[6], j[7]) not in done]
    print('%s: %d runs to do (%d already in %s)' % (kind, len(todo), len(done), path), flush=True)

    with mp.Pool(procs) as pool, open(path, 'a') as fh:
        for n, r in enumerate(pool.imap_unordered(one, todo), 1):
            fh.write(json.dumps(r, ensure_ascii=False) + '\n')
            fh.flush()
            if n % 25 == 0:
                print('  %d / %d' % (n, len(todo)), flush=True)

    print('done', path, flush=True)


if __name__ == '__main__':
    main()
