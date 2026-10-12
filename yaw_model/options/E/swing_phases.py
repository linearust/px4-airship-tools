#!/usr/bin/env python3
"""Where in the gate the pods swing: swings (target moves > 90 deg) per gate
phase, and the torque / integrator there, for one plant and band.

    python3 swing_phases.py TARGET BAND [P I]
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True

import e_model as em  # noqa: E402
import gate  # noqa: E402

target = sys.argv[1]
band = sys.argv[2]
P = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
I = float(sys.argv[4]) if len(sys.argv) > 4 else 0.09
loop = em.build(target, band, params={'AS_YAWRATE_P': P, 'AS_YAWRATE_I': I}, trace=True)
od = os.path.join(HERE, 'tmp_out', 'swing')
os.makedirs(od, exist_ok=True)
res = gate.run_gate(loop, em.model_name(target), out_dir=od)
tr = loop.trace
DEG = 57.29577951308232

# phases from the gate's samples: (sim of first row, phase name)
phases = []
for row in res.samples:
    if not phases or phases[-1][1] != row['phase']:
        phases.append((row['sim'], row['phase']))
phases.append((1e9, 'end'))


def phase_of(t):
    name = 'pre'
    for t0, n in phases:
        if t >= t0:
            name = n
        else:
            break
    return name


counts = {}
first = {}
last_sw = 0
for r in tr:
    sw = r[15] or 0
    if sw > last_sw:
        ph = phase_of(r[0])
        counts[ph] = counts.get(ph, 0) + (sw - last_sw)
        first.setdefault(ph, r)
    last_sw = sw
print('\n'.join(res.gate_lines[:10]))
print('\nswings per phase (%s, %s, P %.2f I %.2f):' % (target, band, P, I))
for t0, n in phases[:-1]:
    if n in counts:
        r = first[n]
        print('  %-40s %4d   first at sim %.2f: torque %+.4f integ %+.4f rate %+.2f deg/s'
              % (n, counts[n], r[0], r[4], r[5], r[3] * DEG))
print('  total', sum(counts.values()))
