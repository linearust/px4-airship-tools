#!/usr/bin/env python3
"""Why does the fin hold-out hold ~10 deg/s at sp 6.67 in the Acro step when the
tilt slew is 60 deg/s? Trace the Acro 추종 phase."""
import math, sys
sys.dont_write_bytecode = True
import ffcommon as fc
t = 'gz_airship_vectored_independent_fin'
FF = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
loop = fc.build(t, 0.7, 0.09, FF, noise=True, seed=1, slew=60, trace=True)
res = fc.gate.run_gate(loop, fc.model_name(t), fc.gate.Timing(), out_dir=fc.os.path.join(fc.HERE, 'tmp_out', 'diag'))
ev = [e for e in res.events if 'stick 0.5' in e[2]]
print(ev[:4])
t_acro = ev[1][1] if len(ev) > 1 else None
tr = loop.trace
for (sim, mode, stick, meas, tq, integ, moment, wind, sp_, sn_, ex) in tr:
    if t_acro is not None and t_acro - 2 <= sim <= t_acro + 36 and int(round(sim * 250)) % 250 == 0:
        print('%7.1f %-10s st %.2f r %6.2f tq %+.4f I %+.4f N %+6.2f sat %d%d u %+.3f v %+.3f fins %d' % (
            sim, mode, stick, math.degrees(meas), tq, integ, moment, sp_, sn_, ex['u'], ex['v'], ex['fins_open']))
