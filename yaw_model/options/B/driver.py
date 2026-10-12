#!/usr/bin/env python3
"""One pool for the remaining actuator-sensitivity gate runs and off-gate runs
of the recommended set (P 0.65 I 0.07), so small batches do not run one after
another. Same job functions and output files as sweep.py / beyond.py.

    python3 driver.py [--pool 12]
"""
import json
import multiprocessing as mp
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True
import beyond  # noqa: E402
import sweep  # noqa: E402

PI = (0.65, 0.07)
NOM = ('nominal', {})
SEEDS = [1, 2, 3, 4, 5]
V_ACTS = [{'slew': 60}, {'slew': 240}, {'phys_slew': 60}]
C_ACTS = [{'tail_tau': 0.025}, {'tail_tau': 0.05}]
VEC_RIGID, VEC_COMPLETE, CS_RIGID, CS_COMPLETE, VEC_FIN = (
    'gz_airship_vectored_independent', 'gz_airship_vectored_independent_am_fin_drag', 'gz_cloudship',
    'gz_cloudship_am_fin_drag', 'gz_airship_vectored_independent_fin')


def done_keys(path, kind):
    keys = set()
    if not os.path.exists(path):
        return keys
    for line in open(path):
        r = json.loads(line)
        if kind == 'gate':
            keys.add((r['target'], r['P'], r['I'], r['FF'], json.dumps(r['act'], sort_keys=True), r['seed']))
        else:
            keys.add((r['target'], r['P'], r['I'], r['FF'], json.dumps(r['act'], sort_keys=True), str(r['kind'])))
    return keys


def jobs():
    out = []
    g_done = done_keys(os.path.join(HERE, 'act2.jsonl'), 'gate')
    for act in V_ACTS:
        for t, ff in ((VEC_RIGID, 0.0), (VEC_COMPLETE, 0.0), (VEC_FIN, 0.0), (VEC_COMPLETE, 1.0), (VEC_FIN, 0.3)):
            for s in SEEDS:
                if (t, PI[0], PI[1], ff, json.dumps(act, sort_keys=True), s) not in g_done:
                    out.append(('gate', 'act2.jsonl', (t, PI + (ff,), NOM, s, act)))
    for act in C_ACTS:
        for t, ff in ((CS_RIGID, 0.0), (CS_COMPLETE, 0.0), (CS_COMPLETE, 0.1)):
            for s in SEEDS:
                if (t, PI[0], PI[1], ff, json.dumps(act, sort_keys=True), s) not in g_done:
                    out.append(('gate', 'act2.jsonl', (t, PI + (ff,), NOM, s, act)))
    b_done = done_keys(os.path.join(HERE, 'beyond2.jsonl'), 'beyond')
    base = [(t, 0.0, {}) for t in (VEC_RIGID, VEC_COMPLETE, CS_RIGID, CS_COMPLETE, VEC_FIN)]
    base += [(VEC_COMPLETE, 1.0, {}), (CS_COMPLETE, 0.1, {}), (VEC_FIN, 0.3, {})]
    for t, ff, act in base:
        for k in (0.25, 0.5, 1.0, 'wind'):
            if (t, PI[0], PI[1], ff, json.dumps(act, sort_keys=True), str(k)) not in b_done:
                out.append(('beyond', 'beyond2.jsonl', (t, PI + (ff,), 1, act, k)))
    for t, q in ((VEC_COMPLETE, 8.594), (CS_COMPLETE, 0.8594), (VEC_FIN, 2.578)):
        for k in (0.25, 0.5, 1.0):
            act = {'ffq': q}
            if (t, PI[0], PI[1], 0.0, json.dumps(act, sort_keys=True), str(k)) not in b_done:
                out.append(('beyond', 'beyond2.jsonl', (t, PI + (0.0,), 1, act, k)))
    ba_done = done_keys(os.path.join(HERE, 'beyond2_act.jsonl'), 'beyond')
    for act in V_ACTS:
        for t, ff in ((VEC_RIGID, 0.0), (VEC_COMPLETE, 0.0), (VEC_FIN, 0.0), (VEC_COMPLETE, 1.0), (VEC_FIN, 0.3)):
            if (t, PI[0], PI[1], ff, json.dumps(act, sort_keys=True), '0.5') not in ba_done:
                out.append(('beyond', 'beyond2_act.jsonl', (t, PI + (ff,), 1, act, 0.5)))
    for act in C_ACTS:
        for t, ff in ((CS_RIGID, 0.0), (CS_COMPLETE, 0.0), (CS_COMPLETE, 0.1)):
            if (t, PI[0], PI[1], ff, json.dumps(act, sort_keys=True), '0.5') not in ba_done:
                out.append(('beyond', 'beyond2_act.jsonl', (t, PI + (ff,), 1, act, 0.5)))
    return out


def run(job):
    kind, path, args = job
    return path, (sweep.one(args) if kind == 'gate' else beyond.one(args))


def main():
    pool_n = int(sys.argv[sys.argv.index('--pool') + 1]) if '--pool' in sys.argv else 12
    js = jobs()
    print('%d jobs' % len(js), flush=True)
    with mp.Pool(pool_n) as pool:
        for i, (path, r) in enumerate(pool.imap_unordered(run, js), 1):
            with open(os.path.join(HERE, path), 'a') as fh:
                fh.write(json.dumps(r, ensure_ascii=False) + '\n')
            if i % 10 == 0:
                print('%d/%d' % (i, len(js)), flush=True)
    print('DRIVER DONE', flush=True)


if __name__ == '__main__':
    main()
