#!/usr/bin/env python3
"""Gate ensemble for the option-E cross-check, on xc_model (independent of
options/E/e_model.py). Every job: gyro noise on, one seed, one timing member
of review/gain_sweep.py's ENSEMBLE (its 'noise s1' member dropped because
noise is on everywhere).

    python3 xc_gate.py --out xc_gate.jsonl [--workers 14] [--resume]

Design (jobs()): a complementary lite design. E used nominal x seeds 1-5 +
the 7 other members x seed 1; here nominal x seeds 6-10 + the 7 other
members x seed 2, so no run repeats one of E's noise draws. 12 runs per
cell; the reduced variants get nominal x seeds 6-10. Ordered by priority.
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

VEC = ['gz_airship_vectored_independent', 'gz_airship_vectored_independent_am_fin_drag',
       'gz_airship_vectored_independent_fin']
CS = ['gz_cloudship', 'gz_cloudship_am_fin_drag']
GAINS = {'P0.5': (0.5, 0.09), 'P0.7': (0.7, 0.09)}
MEMBERS = [
    ('nominal', {}),
    ('stick 0', {'stick_delay_s': 0.0}),
    ('stick 50ms', {'stick_delay_s': 0.05}),
    ('rtf .98', {'rtf': 0.98}),
    ('rtf 1.02', {'rtf': 1.02}),
    ('P .3153', {'listener_s': (0.3153 - 0.3) / 3}),
    ('P .3177', {'listener_s': (0.3177 - 0.3) / 3}),
    ('cmd 0/30ms', {'command_delay_s': 0.03}),
]
NOM_SEEDS = [6, 7, 8, 9, 10]
OTHER_SEED = 2


def one(job):
    target, variant, gname, member, tkw, seed = job
    import xc_model as xm
    import gate
    import reproduce as rp
    P, I = GAINS[gname]
    loop = xm.build(target, variant, params={'AS_YAWRATE_P': P, 'AS_YAWRATE_I': I}, noise=True, seed=seed)
    od = os.path.join(HERE, 'tmp_out', str(os.getpid()))
    os.makedirs(od, exist_ok=True)
    t0 = time.time()
    res = gate.run_gate(loop, xm.model_name(target), gate.Timing(**tkw), out_dir=od)
    parsed = rp.parse(res.gate_lines)
    checks = {k: {f: (v if not (isinstance(v, float) and not math.isfinite(v)) else None)
                  for f, v in rec.items() if f != 'line'} for k, rec in parsed.items()}
    st = loop.actuation.status()
    return {'target': target, 'variant': variant, 'gains': gname, 'member': member, 'seed': seed,
            'status': res.status, 'checks': checks, 'lines': res.gate_lines[:12],
            'jumps': st.get('jumps'), 'slew_rev': st.get('slew_rev'), 'wall': round(time.time() - t0, 1)}


def jobs():
    def cell(t, v, g, full):
        out = [(t, v, g, 'nominal', {}, s) for s in NOM_SEEDS]
        if full:
            out += [(t, v, g, label, tkw, OTHER_SEED) for label, tkw in MEMBERS[1:]]
        return out

    tiers = [
        [j for t in VEC + CS for j in cell(t, 'base', 'P0.7', True)],
        [j for t in VEC + CS for j in cell(t, 'base', 'P0.5', True)],
        [j for g in GAINS for t in VEC for v in ('report_band', 'deliver') for j in cell(t, v, g, True)],
        [j for g in GAINS for t in VEC for v in ('report_hold', 'report_all', 'deliver_report')
         for j in cell(t, v, g, False)],
    ]
    return tiers


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(HERE, 'xc_gate.jsonl'))
    ap.add_argument('--workers', type=int, default=14)
    ap.add_argument('--resume', action='store_true')
    args = ap.parse_args()
    order = {'gz_airship_vectored_independent_am_fin_drag': 0, 'gz_airship_vectored_independent_fin': 1,
             'gz_airship_vectored_independent': 2, 'gz_cloudship_am_fin_drag': 3, 'gz_cloudship': 4}
    todo = []
    for tier in jobs():
        todo += sorted(tier, key=lambda j: order[j[0]])
    mode = 'w'
    if args.resume and os.path.exists(args.out):
        done = set()
        for line in open(args.out):
            r = json.loads(line)
            done.add((r['target'], r['variant'], r['gains'], r['member'], r['seed']))
        todo = [j for j in todo if (j[0], j[1], j[2], j[3], j[5]) not in done]
        mode = 'a'
    start = time.time()
    print('%d runs' % len(todo), flush=True)
    with mp.Pool(args.workers) as pool, open(args.out, mode) as fh:
        for i, r in enumerate(pool.imap_unordered(one, todo, chunksize=1)):
            fh.write(json.dumps(r, ensure_ascii=False) + '\n')
            fh.flush()
            if (i + 1) % 25 == 0:
                print('%d/%d  %.0f s' % (i + 1, len(todo), time.time() - start), flush=True)
    print('done %.0f s' % (time.time() - start), flush=True)


if __name__ == '__main__':
    main()
