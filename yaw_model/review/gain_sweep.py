#!/usr/bin/env python3
"""Reviewer's discrimination test: at other gains, is the model's gate verdict
robust against the perturbations the model itself treats as unknowns
(harness timing, RTF, gyro noise) - i.e. can it tell pass from fail?

For each target x gain set, run an ensemble of gate runs that differ only in
the model's stated unknowns, and report per check: pass count / members, and
the range of the judged number. The ensemble at the CURRENT gains is compared
with the two SITL runs (chain7, chain9) to see if its spread is SITL-like.

    python3 gain_sweep.py [--quick]
"""
import json
import math
import multiprocessing as mp
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
sys.dont_write_bytecode = True

TARGETS = ['gz_airship_vectored_independent', 'gz_airship_vectored_independent_am_fin_drag',
           'gz_cloudship', 'gz_cloudship_am_fin_drag']
GAINS = [(0.5, 0.09), (0.35, 0.09), (0.7, 0.09), (1.0, 0.09), (0.5, 0.03), (0.5, 0.2),
         (0.7, 0.05), (0.7, 0.2), (1.0, 0.2), (0.35, 0.05)]
ENSEMBLE = [
    ('nominal', {}, {}),
    ('stick 0', {'stick_delay_s': 0.0}, {}),
    ('stick 50ms', {'stick_delay_s': 0.05}, {}),
    ('rtf .98', {'rtf': 0.98}, {}),
    ('rtf 1.02', {'rtf': 1.02}, {}),
    ('P .3153', {'listener_s': (0.3153 - 0.3) / 3}, {}),
    ('P .3177', {'listener_s': (0.3177 - 0.3) / 3}, {}),
    ('noise s1', {}, {'noise': True}),
    ('cmd 0/30ms', {'command_delay_s': 0.03}, {}),
]
KEYS = {'Acro brake': 'end', 'Acro track': 'peak', 'Stab track': 'peak', 'Acro release': 'settle',
        'Stab release': 'settle', 'wind I on': 'rate', 'wind weak': 'rate'}


def one(job):
    target, (P, I), (label, tkw, bkw) = job
    import closed_loop as cl
    import gate
    import reproduce as rp
    loop = cl.build(target, params={'AS_YAWRATE_P': P, 'AS_YAWRATE_I': I}, **bkw)
    od = os.path.join(HERE, 'tmp_out', str(os.getpid()))
    os.makedirs(od, exist_ok=True)
    res = gate.run_gate(loop, cl.TARGETS[target][0], gate.Timing(**tkw), out_dir=od)
    parsed = rp.parse(res.gate_lines)
    out = {}
    for k, rec in parsed.items():
        out[k] = {f: (v if not (isinstance(v, float) and not math.isfinite(v)) else None)
                  for f, v in rec.items() if f != 'line'}
    return {'target': target, 'P': P, 'I': I, 'member': label, 'status': res.status, 'checks': out}


def main():
    quick = '--quick' in sys.argv
    gains = GAINS[:2] if quick else GAINS
    jobs = [(t, g, e) for t in TARGETS for g in gains for e in ENSEMBLE]
    out_path = os.path.join(HERE, 'gain_sweep.jsonl')
    with mp.Pool(14) as pool, open(out_path, 'w') as fh:
        for r in pool.imap_unordered(one, jobs):
            fh.write(json.dumps(r, ensure_ascii=False) + '\n')
            fh.flush()
    print('written', out_path)


if __name__ == '__main__':
    main()
