#!/usr/bin/env python3
"""Sensitivity of the Cloudship-complete reproduction to the Munk moment's
size: scale the yaw-row C_A term -(m_y - m_x) u v by s (s = 1 is the model;
0.92 is the engine probe's K -4.53 / -4.92 reading). Series rms against
both records over the pushes."""
import math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE)); sys.dont_write_bytecode = True
import closed_loop as cl, gate, planar, reproduce as rp
orig = planar.RollPlanarPlant._tau
def make(s):
    def _tau(self, st, phi, ext):
        out = orig(self, st, phi, ext)
        u, v = st[0], st[1]
        out[3] += (1.0 - s) * (self.mxy[1] - self.mxy[0]) * u * v
        return out
    return _tau
t = 'gz_cloudship_am_fin_drag'
recs = {tag: rp.read_series(rp.SERIES[(t, tag)]) for tag in ('chain7', 'chain9')}
for s in (0.85, 0.92, 1.0, 1.08):
    planar.RollPlanarPlant._tau = make(s)
    loop = cl.build(t, params=rp.record_params(t)[0])  # the gains the chain7/chain9 records were flown with (record_gains)
    res = gate.run_gate(loop, 'cloudship_am_fin_drag', out_dir=os.path.join(HERE, 'tmp_out', 'munk'))
    p = rp.parse(res.gate_lines)
    rms = []
    for tag, rows in recs.items():
        for r in rp.series_residuals(res.samples, rows):
            if r['phase'] in ('Acro 추종', 'Stabilized 추종', 'Acro 놓음', 'Stabilized 놓음'):
                rms.append('%s %s %.2f' % (tag, r['phase'][:5], r['rate'][0]))
    print('Munk x %.2f: Acro mean/peak %.2f/%.2f %s; Stab %.2f/%.2f; wind I on %.2f/%.4f; rate rms %s' % (
        s, p['Acro track']['mean'], p['Acro track']['peak'], 'pass' if p['Acro track']['ok'] else 'FAIL',
        p['Stab track']['mean'], p['Stab track']['peak'], p['wind I on']['rate'], p['wind I on']['integ'], ', '.join(rms)), flush=True)
planar.RollPlanarPlant._tau = orig
