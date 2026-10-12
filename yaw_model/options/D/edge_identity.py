#!/usr/bin/env python3
"""restore (zero_or_sign trigger) vs restore0 (zero only): identical gate
rows on the two plants where the rule acts most (complete vectored, complete
Cloudship), P 0.7, noise seed 1."""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.dont_write_bytecode = True
import option_d as od
import gate
for t in ('gz_airship_vectored_independent_am_fin_drag', 'gz_cloudship_am_fin_drag'):
    out = []
    for rule in ({'kind': 'restore'}, {'kind': 'restore', 'edge': 'zero'}):
        loop = od.build(t, rule, params=od.GAINS['P0.7'], noise=True, seed=1)
        res = gate.run_gate(loop, od.model_of(t), out_dir=os.path.join(HERE, 'tmp_out', str(os.getpid())))
        out.append(([repr((r['rate'], r['torque'], r['integ'])) for r in res.samples], len(loop.controller.ctrl.events)))
    print(t, 'rows identical' if out[0][0] == out[1][0] else 'ROWS DIFFER', 'events', out[0][1], out[1][1])
