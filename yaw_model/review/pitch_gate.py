#!/usr/bin/env python3
"""Sensitivity: the complete Cloudship with the pitch oscillation gz shows
(spawned level, trim -2.5 deg, period 3.5 s, half p-p 2.3 deg decaying to 0.4
deg over the run: log 2026-09-27/08_53_51.ulg vehicle_attitude_groundtruth)
fed into the fins' velocity only (w = (p, q + q_osc, r) at each cp). Pitch
dynamics stay held; this only asks how much the fin gate it drives moves
the gate's numbers."""
import math, os, sys
import numpy as np
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import planar, closed_loop, gate, reproduce
P05 = reproduce.record_params('gz_cloudship_am_fin_drag')[0]  # the gains the chain7/chain9 records were flown with (record_gains)

# half p-p [deg] per 10 s window of LOG time (log_drift / pitch table above)
AMP = [2.34, 2.06, 1.80, 2.28, 2.11, 1.85, 1.63, 1.50, 1.36, 1.22, 0.93, 0.71, 0.54, 0.51, 0.49, 0.47,
       0.46, 0.45, 0.43, 0.42, 0.41, 0.40, 0.39, 0.38, 0.37, 0.36]
OMEGA = 2 * math.pi / 3.5
LOG_MINUS_SIM = -5.0      # the model's Manual step is at sim 37.1 s, the log's at 32.1 s


class PitchedFins(planar.RollPlanarPlant):
    def __init__(self, *a, phase=0.0, scale=1.0, **k):
        super().__init__(*a, **k)
        self.t = 0.0
        self.phase = phase
        self.scale = scale

    def q_osc(self):
        tl = self.t + LOG_MINUS_SIM
        amp = math.radians(np.interp(tl, [5 + 10 * i for i in range(len(AMP))], AMP)) * self.scale
        return amp * OMEGA * math.cos(OMEGA * tl + self.phase)

    def _tau(self, s, phi, ext):
        # the fins' velocity gets the oscillating pitch rate; nothing else does
        saved = self.fins
        qo = self.q_osc()
        u, v, p, r = s
        extra = [0.0] * 6
        if saved:
            q = r * math.tan(phi) + qo
            w = (p, q, r)
            for f in saved:
                vcp = planar._cross(w, f['cp'])
                F = planar.lift_drag((u + vcp[0], v + vcp[1], vcp[2]), f)
                M = planar._cross(f['cp'], F)
                for kk in range(3):
                    extra[kk] += F[kk]
                    extra[3 + kk] += M[kk]
        self.fins = []
        try:
            out = super()._tau(s, phi, [ext[i] + extra[i] for i in range(6)])
        finally:
            self.fins = saved
        return out

    def step_actuated(self, dt, *a):
        super().step_actuated(dt, *a)
        self.t += dt


def build(phase, scale):
    loop = closed_loop.build('gz_cloudship_am_fin_drag', params=P05)
    loop.plant = PitchedFins('gz_cloudship_am_fin_drag', model='cloudship_am_fin_drag', phase=phase, scale=scale)
    return loop


def nums(res):
    out = []
    for name, ok, detail in res.verdicts:
        out.append(('P' if ok else 'F') + ' ' + detail.split(';')[0][:70])
    return out


base = gate.run_gate(closed_loop.build('gz_cloudship_am_fin_drag', params=P05), 'cloudship_am_fin_drag')
rows = [('model (pitch held)', nums(base))]
for scale in (1.0,):
    for ph in (0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi):
        res = gate.run_gate(build(ph, scale), 'cloudship_am_fin_drag')
        rows.append(('pitch osc x%.1f phase %.2f' % (scale, ph), nums(res)))
for label, n in rows:
    print('==', label)
    for line in n:
        print('   ', line)
