#!/usr/bin/env python3
"""The 'deliver' variant's tilt hunting: a window of the per-tick trace."""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.dont_write_bytecode = True
import e_model as em, gate
target, band, t0, t1 = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
P = float(sys.argv[5]) if len(sys.argv) > 5 else 0.5
loop = em.build(target, band, params={'AS_YAWRATE_P': P, 'AS_YAWRATE_I': 0.09}, trace=True)
od = os.path.join(HERE, 'tmp_out', 'hunt'); os.makedirs(od, exist_ok=True)
res = gate.run_gate(loop, em.model_name(target), out_dir=od)
DEG = 57.29577951308232
tr = loop.trace
prev = None
sw0 = None
for r in tr:
    if t0 <= r[0] <= t1:
        sw0 = r[15] if sw0 is None else sw0
        key = (r[10], r[11], r[8], r[9])
        if key != prev or abs(r[0] * 4 - round(r[0] * 4)) < 1e-6:
            print('sim %8.3f rate %+6.3f torque %+.4f integ %+.4f satP %d satN %d %-9s %-9s tilt %6.1f %6.1f N %+6.3f swings %d'
                  % (r[0], r[14] * DEG, r[4], r[5], r[8], r[9], r[10], r[11], r[12], r[13], r[6], r[15] - sw0))
        prev = key
