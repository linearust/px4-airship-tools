"""Sanity: b = 1 is today's loop bit for bit; ff = P (b - 1) survives the
gate's AS_YAWRATE_I 0 / restore; b leaves every setpoint-0 phase untouched."""
import sys, os
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import optc, closed_loop as cl, gate
t = 'gz_airship_vectored_independent'
base = gate.run_gate(cl.build(t, noise=True, seed=1), optc.model_name(t), out_dir=os.path.join(optc.HERE, 'tmp_out', 'chk'))
res1, m1 = optc.run_gate(t, b=1.0, seed=1)
same = all(a['rate'] == b['rate'] and a['torque'] == b['torque'] for a, b in zip(base.samples, res1.samples))
print('b=1.0 identical to closed_loop.build (noise seed 1):', same, len(base.samples), len(res1.samples))
res7, m7 = optc.run_gate(t, b=0.7, seed=1)
loop = optc.build(t, b=0.7)
print('ff after build P0.5 b0.7:', loop.controller.ctrl.rc.ff)
loop.set_param('AS_YAWRATE_I', 0.0); print('ff after I=0:', loop.controller.ctrl.rc.ff, 'i', loop.controller.ctrl.rc.i)
by1, by7 = {}, {}
for r in res1.samples: by1.setdefault(r['phase'], []).append(r)
for r in res7.samples: by7.setdefault(r['phase'], []).append(r)
for ph in by1:
    a, b = by1[ph], by7.get(ph, [])
    n = min(len(a), len(b))
    d = max((abs(x['rate'] - y['rate']) for x, y in zip(a[:n], b[:n])), default=float('nan'))
    print('  %-40s rows %3d/%3d  max |d rate| %.4f' % (ph, len(a), len(b), d))
for k in m1:
    print(k, {x: (round(v, 3) if isinstance(v, float) else v) for x, v in m1[k].items()})
    print('   b.7', {x: (round(v, 3) if isinstance(v, float) else v) for x, v in m7[k].items()})
