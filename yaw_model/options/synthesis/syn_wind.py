#!/usr/bin/env python3
"""Option A's beyond-gate scenarios (options/A/beyond.py, imported, not edited)
at P 0.7 I 0.09 INT_LIM 0.3 and at today's P 0.5 I 0.09, nominal actuators,
seed 1. Stick-centred cases (wind step, gust, hold noise) are identical with
the restore rule, which never fires with the stick centred; stick-step
overshoot is too (the rule acts only at the release)."""
import json, os, sys
import multiprocessing as mp
HERE = os.path.dirname(os.path.abspath(__file__))
A = os.path.join(os.path.dirname(HERE), 'A')
sys.path.insert(0, A); sys.dont_write_bytecode = True
import beyond as bA
import optA

if __name__ == '__main__':
    specs = []
    for P in (0.5, 0.7):
        params = {'AS_YAWRATE_P': P, 'AS_YAWRATE_I': 0.09, 'AS_YR_INT_LIM': 0.3}
        for t in optA.TARGETS:
            for k in ('wind_gate', 'wind_10pct', 'gust', 'step_half', 'step_small', 'noise'):
                specs.append((k, t, params, None, 1))
    with mp.Pool(15) as pool, open(os.path.join(HERE, 'wind.jsonl'), 'w') as fh:
        for r in pool.imap_unordered(bA.job, specs):
            fh.write(json.dumps(r) + '\n'); fh.flush()
    print('done')
