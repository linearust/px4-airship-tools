#!/usr/bin/env python3
"""Weathercock scenarios: turns 45/90/180 deg x wind directions -120..180 x
five plants, seed 1. python3 wcock_run.py CAND:GAINS [...]"""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.dont_write_bytecode = True
import beyond as b
import option_d as od
from ensemble import CANDIDATES
TURNS = (45, 90, 180)
DIRS = (-120, -60, 0, 60, 120, 180)
# the finned / damped plants, where a heading-dependent moment is the realistic disturbance
PLANTS = ('gz_airship_vectored_independent_am_fin_drag', 'gz_cloudship_am_fin_drag', 'gz_airship_vectored_independent_fin')
if __name__ == '__main__':
    js = []
    for spec in sys.argv[1:]:
        name, gains = spec.split(':')
        for t in PLANTS:
            for turn in TURNS:
                for d in DIRS:
                    scen = 'wcock%d_%d' % (turn, d)
                    js.append(dict(id='%s|%s|%s|%s' % (name, gains, t, scen), name=name, rule=CANDIDATES[name],
                                   gains=gains, target=t, scen=scen, seed=1))
    b.run_jobs(js, os.path.join(HERE, 'wcock.jsonl'), workers=int(os.environ.get('W', 4)))
