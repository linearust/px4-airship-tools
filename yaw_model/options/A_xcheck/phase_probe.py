#!/usr/bin/env python3
"""Print the brake and release rows of one gate run (rigid vectored), to see
what the gate's one-instant brake reading and settle() leave out."""
import os, sys
sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import xgate, gate
t = sys.argv[1]; P, I = float(sys.argv[2]), float(sys.argv[3]); seed = int(sys.argv[4]) if len(sys.argv) > 4 else 1
loop = xgate.build(t, {'AS_YAWRATE_P': P, 'AS_YAWRATE_I': I, 'AS_YR_INT_LIM': 0.3}, seed)
od = os.path.join(HERE, 'tmp_out', 'probe'); os.makedirs(od, exist_ok=True)
res = gate.run_gate(loop, xgate.model_name(t), gate.Timing(), out_dir=od)
ph = {}
for r in res.samples:
    ph.setdefault(r['phase'], []).append(r)
for name in ('Acro 제동', 'Acro 놓음', 'Stabilized 놓음'):
    rows = ph[name]
    print(name, 'n', len(rows), 'min %.2f max %.2f last %.2f' % (min(r['rate'] for r in rows), max(r['rate'] for r in rows), rows[-1]['rate']))
    print('  ' + ' '.join('%.1f' % r['rate'] for r in rows))
    print('  integ ' + ' '.join('%.3f' % r['integ'] for r in rows[::3]))
