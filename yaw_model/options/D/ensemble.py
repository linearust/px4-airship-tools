#!/usr/bin/env python3
"""The full judgement: gate model on all five plants, gain_sweep.py's timing
variants (8) crossed with gyro-noise seeds, per candidate and gain set.

    python3 ensemble.py CONFIG [CONFIG ...]    CONFIG = name:gains[:full|lite]
e.g. reset:P0.7:full -> 8 variants x seeds 1..5 x 5 plants = 200 gate runs.
Appends to ensemble.jsonl (resumable)."""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.dont_write_bytecode = True
import gate_eval as ge
import option_d as od

CANDIDATES = {
    'none': {'kind': 'none'},
    'reset': {'kind': 'reset'},
    'reset_hold': {'kind': 'reset_hold'},
    'restore': {'kind': 'restore'},
    'restore0': {'kind': 'restore', 'edge': 'zero'},
    'rscale2': {'kind': 'restore_scaled', 'edge': 'zero', 'n': 2.0},
    'rscale1': {'kind': 'restore_scaled', 'edge': 'zero', 'n': 1.0},
    'reset0': {'kind': 'reset', 'edge': 'zero'},
    'reset_freeze': {'kind': 'reset_freeze', 'r_th': 1.0, 't_max': 5.0},
    'restore_freeze': {'kind': 'restore_freeze', 'r_th': 1.0, 't_max': 5.0},
}
for t in (2, 4, 8, 16, 32):
    CANDIDATES['bleed%g' % t] = {'kind': 'bleed', 'tau': t}
for L in (0.015, 0.025, 0.04, 0.06):
    for n in (1, 2):
        CANDIDATES['cap%g_n%d' % (L, n)] = {'kind': 'cap', 'L0': L, 'n': n}


def members(mode):
    """'full' = every timing variant x seeds 1..5 (40); 'lite' = each variant
    with one seed (variant i -> seed i % 5 + 1) plus nominal x seeds 2..5 (12)."""
    if mode == 'full':
        return [(label, tkw, seed) for label, tkw in ge.TIMING for seed in range(1, 6)]
    out = [(label, tkw, i % 5 + 1) for i, (label, tkw) in enumerate(ge.TIMING)]
    return out + [(ge.TIMING[0][0], ge.TIMING[0][1], seed) for seed in (2, 3, 4, 5)]


def jobs(name, gains, mode='full', extra=None, targets=None):
    out = []
    for t in (targets or od.TARGETS5):
        for label, tkw, seed in members(mode):
            if True:
                j = dict(id='%s|%s|%s|%s|%d' % (name, gains, t, label, seed), name=name,
                         rule=CANDIDATES[name], gains=gains, target=t, member=label, timing=tkw, seed=seed)
                if extra:
                    j.update(extra)
                    j['id'] += '|' + ','.join('%s=%s' % kv for kv in sorted(extra.items()))
                out.append(j)
    return out


if __name__ == '__main__':
    all_jobs = []
    for spec in sys.argv[1:]:
        parts = spec.split(':')
        name, gains = parts[0], parts[1]
        all_jobs += jobs(name, gains, parts[2] if len(parts) > 2 else 'full')
    ge.run_jobs(all_jobs, os.path.join(HERE, 'ensemble.jsonl'), workers=int(os.environ.get('W', 8)))
