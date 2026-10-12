#!/usr/bin/env python3
"""Run beyond.py's scenarios: python3 beyond_run.py CAND:GAINS [...]
(seeds 1-2, five plants, all scenarios), plus the actuator variants with
ACT=1 (tilt slew 60/240 deg/s on the vectored plants, tail lag 0.025/0.05 s
on the Cloudships: step0.5, wind0.05, rest)."""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.dont_write_bytecode = True
import beyond as b
import option_d as od
from ensemble import CANDIDATES

SCEN = ['step0.2', 'step0.5', 'step1.0', 'partial', 'rev_step', 'rev_ramp', 'wind0.0185', 'wind0.05', 'wind0.1',
        'wind0.2', 'relwind+', 'relwind-', 'nudge', 'edge', 'rest', 'restwind']
ACT_SCEN = ['step0.5', 'wind0.05', 'rest', 'relwind+']
# scenarios whose metrics are noise statistics keep two seeds; the rest are deterministic to ~1e-2
NOISY = ('rest', 'restwind', 'nudge', 'edge')
SUBSETS = {'wind': ('wind0.0185', 'wind0.05', 'wind0.1', 'wind0.2', 'relwind+', 'relwind-', 'nudge', 'step0.5'),
           'basic': ('step0.5', 'step1.0', 'relwind+', 'relwind-', 'nudge', 'wind0.05')}


def jobs(name, gains, seeds=(1, 2), act=False):
    out = []
    for t in od.TARGETS5:
        variants = [None]
        if act:
            variants = [('slew', 60.0), ('slew', 240.0)] if od.airframe_of(t) == 'vectored' else \
                       [('tail_tau', 0.025), ('tail_tau', 0.05)]
        for v in variants:
            for scen in (ACT_SCEN if act else SCEN):
                for seed in (seeds if scen in NOISY else seeds[:1]):
                    j = dict(id='%s|%s|%s|%s|%d|%s' % (name, gains, t, scen, seed, v), name=name,
                             rule=CANDIDATES[name], gains=gains, target=t, scen=scen, seed=seed)
                    if v:
                        j[v[0]] = v[1]
                    out.append(j)
    return out


if __name__ == '__main__':
    act = os.environ.get('ACT') == '1'
    js = []
    for spec in sys.argv[1:]:
        parts = spec.split(':')
        name, gains = parts[0], parts[1]
        sub = SUBSETS[parts[2]] if len(parts) > 2 else None
        js += [j for j in jobs(name, gains, act=act) if sub is None or j['scen'] in sub]
    b.run_jobs(js, os.path.join(HERE, 'beyond_act.jsonl' if act else 'beyond.jsonl'),
               workers=int(os.environ.get('W', 8)))
