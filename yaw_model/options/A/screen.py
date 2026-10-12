#!/usr/bin/env python3
"""Phase 1 screen: P x I x INT_LIM grid, all five plants, nominal timing, noise seed 1."""
import itertools, json, multiprocessing as mp, os, sys
sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import optA
PS = [0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2]
IS = [0.03, 0.05, 0.07, 0.09, 0.11, 0.13, 0.15, 0.175, 0.2]
LS = [0.15, 0.2, 0.3, 0.5]
def main():
    out = os.path.join(HERE, sys.argv[1] if len(sys.argv) > 1 else 'screen.jsonl')
    done = set()
    if os.path.exists(out):
        for line in open(out):
            r = json.loads(line); p = r['params']
            done.add((r['target'], p['AS_YAWRATE_P'], p['AS_YAWRATE_I'], p['AS_YR_INT_LIM']))
    jobs = [(t, {'AS_YAWRATE_P': P, 'AS_YAWRATE_I': I, 'AS_YR_INT_LIM': L}, 'nominal', 1, None)
            for P, I, L in itertools.product(PS, IS, LS) for t in optA.TARGETS
            if (t, P, I, L) not in done]
    print(len(jobs), 'jobs', flush=True)
    with mp.Pool(15) as pool, open(out, 'a') as fh:
        for n, r in enumerate(pool.imap_unordered(optA.gate_job, jobs, chunksize=2), 1):
            fh.write(json.dumps(r, ensure_ascii=False) + '\n'); fh.flush()
            if n % 100 == 0:
                print(n, flush=True)
    print('done', flush=True)
if __name__ == '__main__':
    main()
