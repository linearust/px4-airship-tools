#!/usr/bin/env python3
"""The gate under actuator variants: tilt slew 60 / 240 deg/s (vectored
plants), tail thrust lag 0.025 / 0.05 s (Cloudships; gz has none), nominal
timing, gyro noise seeds 1-3. python3 act_gate.py CAND:GAINS [...]"""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.dont_write_bytecode = True
import gate_eval as ge
import option_d as od
from ensemble import CANDIDATES
if __name__ == '__main__':
    js = []
    for spec in sys.argv[1:]:
        name, gains = spec.split(':')
        for t in od.TARGETS5:
            vs = [('slew', 60.0), ('slew', 240.0)] if od.airframe_of(t) == 'vectored' else [('tail_tau', 0.025), ('tail_tau', 0.05)]
            for k, v in vs:
                for seed in (1, 2, 3):
                    js.append({'id': '%s|%s|%s|%s=%g|%d' % (name, gains, t, k, v, seed), 'name': name,
                               'rule': CANDIDATES[name], 'gains': gains, 'target': t, 'member': 'nominal',
                               'timing': {}, 'seed': seed, k: v})
    ge.run_jobs(js, os.path.join(HERE, 'act_gate.jsonl'), workers=int(os.environ.get('W', 8)))
