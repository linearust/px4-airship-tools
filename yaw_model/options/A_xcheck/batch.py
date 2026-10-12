#!/usr/bin/env python3
"""Run a batch of xgate jobs in parallel, appending to a jsonl (resumable).

    python3 batch.py OUT.jsonl PLANTS SEEDS [act=kind:value] P I LIM [P I LIM ...]
      PLANTS  all | vectored | cloudship | comma list of short indices 0-4
      SEEDS   comma list and ranges, e.g. 1-5,11-15 (every member runs every seed)
"""
import json
import multiprocessing as mp
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import xgate  # noqa: E402


def seeds_of(s):
    out = []
    for part in s.split(','):
        if '-' in part:
            a, b = part.split('-')
            out += list(range(int(a), int(b) + 1))
        else:
            out.append(int(part))
    return out


def key(t, params, member, seed, act):
    return json.dumps([t, params['AS_YAWRATE_P'], params['AS_YAWRATE_I'], params['AS_YR_INT_LIM'], member, seed, act])


def main():
    out = os.path.join(HERE, sys.argv[1])
    plants = sys.argv[2]
    seeds = seeds_of(sys.argv[3])
    rest = sys.argv[4:]
    act = None
    if rest and rest[0].startswith('act='):
        k, v = rest[0][4:].split(':')
        act = [k, float(v)]
        rest = rest[1:]
    sets = [tuple(float(x) for x in rest[i:i + 3]) for i in range(0, len(rest), 3)]
    if plants == 'all':
        targets = xgate.TARGETS
    elif plants == 'vectored':
        targets = xgate.VECTORED
    elif plants == 'cloudship':
        targets = xgate.TARGETS[3:]
    else:
        targets = [xgate.TARGETS[int(i)] for i in plants.split(',')]
    done = set()
    if os.path.exists(out):
        for line in open(out):
            r = json.loads(line)
            done.add(key(r['target'], r['params'], r['member'], r['seed'], r['act']))
    jobs = []
    for P, I, L in sets:
        params = {'AS_YAWRATE_P': P, 'AS_YAWRATE_I': I, 'AS_YR_INT_LIM': L}
        for t in targets:
            if act and ((act[0] in ('slew', 'servo')) != (t in xgate.VECTORED)):
                continue
            for m in xgate.MEMBERS:
                for s in seeds:
                    j = (t, params, m, s, tuple(act) if act else None)
                    if key(t, params, m, s, act) not in done:
                        jobs.append(j)
    print(len(jobs), 'jobs', flush=True)
    with mp.Pool(int(os.environ.get('W', 12))) as pool, open(out, 'a') as fh:
        for n, r in enumerate(pool.imap_unordered(xgate.run, jobs), 1):
            r['act'] = list(r['act']) if r['act'] else None
            fh.write(json.dumps(r, ensure_ascii=False) + '\n')
            fh.flush()
            if n % 40 == 0:
                print(n, flush=True)
    print('done', flush=True)


if __name__ == '__main__':
    main()
