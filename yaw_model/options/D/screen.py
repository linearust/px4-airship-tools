#!/usr/bin/env python3
"""Screening: every candidate rule and parameter value, both gain sets, five
plants, nominal timing, gyro noise seed 1. Picks the values the full
ensemble (ensemble.py) then runs."""
import json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True
import gate_eval as ge
import option_d as od

RULES = [('none', {'kind': 'none'}),
         ('reset', {'kind': 'reset'}),
         ('reset0', {'kind': 'reset', 'edge': 'zero'}),
         ('reset_hold', {'kind': 'reset_hold'})]
RULES += [('bleed%g' % t, {'kind': 'bleed', 'tau': t}) for t in (2, 4, 8, 16)]
RULES += [('cap%g_n%d' % (L, n), {'kind': 'cap', 'L0': L, 'n': n}) for L in (0.015, 0.025, 0.04) for n in (1, 2)]
RULES += [('restore', {'kind': 'restore'})]
RULES += [('freeze%g' % r, {'kind': 'reset_freeze', 'r_th': r}) for r in (1.0, 0.5)]


def jobs():
    out = []
    for name, rule in RULES:
        for g in od.GAINS:
            for t in od.TARGETS5:
                out.append(dict(id='%s|%s|%s' % (name, g, t), name=name, rule=rule, gains=g, target=t,
                                member='nominal', timing={}, seed=1))
    return out


if __name__ == '__main__':
    ge.run_jobs(jobs(), os.path.join(HERE, 'screen.jsonl'), workers=int(os.environ.get('W', 8)))
