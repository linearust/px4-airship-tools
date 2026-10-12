#!/usr/bin/env python3
"""Option E on the gate model: every plant x steer-band variant x gain set x
ensemble member (gain_sweep.py's unknowns) x gyro-noise seed.

    python3 eval_gate.py --screen      # noise-free nominal only
    python3 eval_gate.py               # the full ensemble -> eval_gate.jsonl

The Cloudship plants are run with band 'base' only: option E does not touch
their chain (closed_loop.CloudshipAct is used whatever the band; the
collective pods get zero differential demand), so every band's Cloudship
verdicts ARE these.
"""
import json
import math
import multiprocessing as mp
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True

VECTORED = ['gz_airship_vectored_independent', 'gz_airship_vectored_independent_am_fin_drag',
            'gz_airship_vectored_independent_fin']
CLOUDSHIP = ['gz_cloudship', 'gz_cloudship_am_fin_drag']
BANDS = ['base', 'report_hold', 'report_band', 'report_all', 'deliver', 'deliver_report']
GAINS = {'current': (0.5, 0.09), 'A': (0.7, 0.09)}
# review/gain_sweep.py ENSEMBLE, less its 'noise s1' member: here gyro noise is
# on in every member, over SEEDS
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
SEEDS = [1, 2, 3, 4, 5]


def one(job):
    target, band, gname, member, tkw, seed, extra = job
    import e_model as em
    import gate
    import reproduce as rp
    P, I = GAINS[gname]
    kw = dict(extra)
    if seed is not None:
        kw.update(noise=True, seed=seed)
    loop = em.build(target, band, params={'AS_YAWRATE_P': P, 'AS_YAWRATE_I': I}, **kw)
    od = os.path.join(HERE, 'tmp_out', str(os.getpid()))
    os.makedirs(od, exist_ok=True)
    t0 = time.time()
    res = gate.run_gate(loop, em.model_name(target), gate.Timing(**tkw), out_dir=od)
    parsed = rp.parse(res.gate_lines)
    checks = {k: {f: (v if not (isinstance(v, float) and not math.isfinite(v)) else None)
                  for f, v in rec.items() if f != 'line'} for k, rec in parsed.items()}
    st = loop.actuation.status()
    return {'target': target, 'band': band, 'gains': gname, 'P': P, 'I': I, 'member': member,
            'seed': seed, 'extra': {k: v for k, v in extra.items()}, 'status': res.status,
            'checks': checks, 'swings': st.get('swings'), 'deliver_engages': st.get('deliver_engages'),
            'wall': round(time.time() - t0, 1)}


def jobs(screen=False, extra=None, bands=None, targets=None, gains=None, reduced=(), lite=()):
    """bands in `reduced` get the nominal member only (x SEEDS), not the
    whole timing ensemble."""
    extra = extra or {}
    out = []
    for target in (targets or VECTORED + CLOUDSHIP):
        for band in list(bands or BANDS) + [b for b in reduced if b not in (bands or BANDS)]:
            if target in CLOUDSHIP and band != 'base':
                continue
            for gname in (gains or GAINS):
                out.append((target, band, gname, 'nominal (noise-free)', {}, None, extra))
                if screen:
                    continue
                for label, tkw in (MEMBERS[:1] if band in reduced else MEMBERS):
                    # lite design: the nominal member over every seed, each
                    # other timing member over seed 1 only
                    seeds = SEEDS if (target not in lite or label == 'nominal') else SEEDS[:1]
                    for seed in seeds:
                        out.append((target, band, gname, label, tkw, seed, extra))
    return out


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--screen', action='store_true')
    ap.add_argument('--out', default=None)
    ap.add_argument('--workers', type=int, default=12)
    ap.add_argument('--slew', type=float, default=None, help='CA_AIRSHIP_TLT_R and servo, deg/s')
    ap.add_argument('--phys-slew', type=float, default=None, help='physical servo limit, deg/s')
    ap.add_argument('--bands', default=None)
    ap.add_argument('--vectored-only', action='store_true')
    ap.add_argument('--reduced', default='', help='bands run on the nominal member only')
    ap.add_argument('--nominal-only', action='store_true', help='every band: nominal member x seeds')
    ap.add_argument('--resume', action='store_true', help='append, skipping runs already in --out')
    ap.add_argument('--lite', default='', help='targets on the lite design (nominal x seeds, others x seed 1)')
    args = ap.parse_args()
    extra = {}
    if args.slew is not None:
        extra['slew_deg'] = args.slew
    if args.phys_slew is not None:
        extra['phys_slew_deg'] = args.phys_slew
    bands = args.bands.split(',') if args.bands else BANDS
    reduced = tuple(b for b in args.reduced.split(',') if b)
    if args.nominal_only:
        reduced = tuple(bands)
    lite = tuple(t for t in args.lite.split(',') if t)
    todo = jobs(args.screen, extra, bands, VECTORED if args.vectored_only else None, reduced=reduced, lite=lite)
    # longest first
    todo.sort(key=lambda j: (j[0] in CLOUDSHIP, j[0] != 'gz_airship_vectored_independent_am_fin_drag'))
    out = args.out or os.path.join(HERE, 'eval_screen.jsonl' if args.screen else 'eval_gate.jsonl')
    mode = 'w'
    if args.resume and os.path.exists(out):
        done = set()
        with open(out) as fh:
            for line in fh:
                r = json.loads(line)
                done.add((r['target'], r['band'], r['gains'], r['member'], r['seed']))
        todo = [j for j in todo if (j[0], j[1], j[2], j[3], j[5]) not in done]
        mode = 'a'
        print('resuming:', len(done), 'done,', len(todo), 'to go', flush=True)
    t0 = time.time()
    with mp.Pool(args.workers) as pool, open(out, mode) as fh:
        for i, r in enumerate(pool.imap_unordered(one, todo, chunksize=1)):
            fh.write(json.dumps(r, ensure_ascii=False) + '\n')
            fh.flush()
            if (i + 1) % 50 == 0:
                print('%d/%d  %.0f s' % (i + 1, len(todo), time.time() - t0), flush=True)
    print('written', out, len(todo), 'runs', '%.0f s' % (time.time() - t0))


if __name__ == '__main__':
    main()
