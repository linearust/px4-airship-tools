#!/usr/bin/env python3
"""Where the steer band acts in the gate: a per-tick trace of the Acro and
Stabilized release (and the weak-wind case) on one vectored plant.

    python3 trace_release.py TARGET BAND [P I]
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True

import e_model as em  # noqa: E402
import gate  # noqa: E402

target = sys.argv[1] if len(sys.argv) > 1 else 'gz_airship_vectored_independent_am_fin_drag'
band = sys.argv[2] if len(sys.argv) > 2 else 'base'
P = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
I = float(sys.argv[4]) if len(sys.argv) > 4 else 0.09
loop = em.build(target, band, params={'AS_YAWRATE_P': P, 'AS_YAWRATE_I': I}, trace=True)
od = os.path.join(HERE, 'tmp_out', 'trace')
os.makedirs(od, exist_ok=True)
res = gate.run_gate(loop, em.model_name(target), out_dir=od)
print('\n'.join(res.gate_lines))
tr = loop.trace
DEG = 57.29577951308232

# release starts: stick goes 0.5 -> 0 while in acro/stabilized
starts = []
for k in range(1, len(tr)):
    if tr[k - 1][2] == 0.5 and tr[k][2] == 0 and tr[k][1] in ('acro', 'stabilized'):
        starts.append(k)

for k0 in starts:
    print('\n=== release in %s at sim %.2f' % (tr[k0][1], tr[k0][0]))
    print('  t+    rate   torque   integ  satP satN  mode_s    mode_p     tilt_s  tilt_p   N[Nm]')
    for k in range(k0 - 25, min(len(tr), k0 + int(12.5 / 0.004))):
        dtk = tr[k][0] - tr[k0][0]
        # print every 0.25 s, and every tick where a pod mode changes
        change = k > 0 and (tr[k][10] != tr[k - 1][10] or tr[k][11] != tr[k - 1][11]
                            or tr[k][8] != tr[k - 1][8] or tr[k][9] != tr[k - 1][9])
        if change or abs(dtk / 0.25 - round(dtk / 0.25)) < 1e-6:
            r = tr[k]
            print('%6.3f %7.3f %8.4f %7.4f  %d    %d   %-9s %-9s %7.1f %7.1f %7.3f%s'
                  % (dtk, r[3] * DEG, r[4], r[5], r[8], r[9], r[10], r[11], r[12], r[13], r[6],
                     '  <-' if change else ''))
