#!/usr/bin/env python3
"""Why the fin plant fails Acro tracking with a 60 deg/s tilt servo: one gate run
(nominal member, seed 1, P 0.7 I 0.2) with the pods' state sampled in the push."""
import math, os, sys
sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import optA, gate
loop = optA.build('gz_airship_vectored_independent_fin', {'AS_YAWRATE_P': 0.7, 'AS_YAWRATE_I': 0.2, 'AS_YR_INT_LIM': 0.3},
                  1, ('slew', 60.0))
samples = []
orig = loop.advance
def adv(dt):
    orig(dt)
    a = loop.actuation
    if a.row is not None:
        samples.append((round(loop.sim, 3), math.degrees(loop.read_rate()), loop.read_torque(), loop.controller.ctrl.integ,
                        a.row['tilt_px4_s'], a.row['tilt_px4_p'], a.row['mode_s'], a.row['mode_p'], a.row['N'],
                        loop.plant.extra()['u'], loop.plant.extra()['v'], loop.controller.ctrl.rc.sat_pos, loop.controller.ctrl.rc.sat_neg))
loop.advance = adv
res = gate.run_gate(loop, 'airship_vectored_independent_fin', gate.Timing(), out_dir=os.path.join(HERE, 'tmp_out'))
for l in res.gate_lines[:10]: print(l)
ev = [e for e in res.events if 'stick 0.5' in e[2]]
t0 = [e[1] for e in res.events if e[2] == 'stick 0.5'][1]      # the Acro push (index 0 = Manual)
print('Acro push starts at sim', t0)
last = None
for s in samples:
    if t0 - 1 <= s[0] <= t0 + 36 and (last is None or s[0] - last >= 1.0):
        last = s[0]
        print('t+%5.1f r %6.2f tq %+.4f I %+.4f tilt s/p %7.1f %7.1f %-8s %-8s N %+6.2f u %+.3f v %+.3f sat %d%d' %
              ((s[0] - t0,) + s[1:9] + (s[9], s[10], s[11], s[12])))
