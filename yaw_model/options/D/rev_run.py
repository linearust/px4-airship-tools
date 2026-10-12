#!/usr/bin/env python3
"""Reversal and no-deadzone-sample cases for the zero-only trigger variants."""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.dont_write_bytecode = True
import beyond as b
import option_d as od
from ensemble import CANDIDATES
NAMES = sys.argv[1:] or ('restore0', 'reset0')
SCENS = ('rev_step', 'rev_ramp', 'nudge', 'edge', 'step0.5', 'partial', 'step1.0', 'relwind+', 'relwind-')
js = []
for name in NAMES:
    for t in od.TARGETS5:
        for scen in SCENS:
            js.append(dict(id='%s|P0.7|%s|%s|1|None' % (name, t, scen), name=name, rule=CANDIDATES[name],
                           gains='P0.7', target=t, scen=scen, seed=1))
b.run_jobs(js, os.path.join(HERE, 'beyond_rev.jsonl'), workers=int(os.environ.get('W', 2)))
