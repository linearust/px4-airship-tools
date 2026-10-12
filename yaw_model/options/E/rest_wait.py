#!/usr/bin/env python3
"""The gate's rest wait (clean_start before the Acro step) on one plant/band:
rate, torque, integrator per sampled row, and the saturation flags."""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.dont_write_bytecode = True
import e_model as em, gate
target, band = sys.argv[1], sys.argv[2]
P = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
loop = em.build(target, band, params={'AS_YAWRATE_P': P, 'AS_YAWRATE_I': 0.09}, trace=True)
od = os.path.join(HERE, 'tmp_out', 'rest'); os.makedirs(od, exist_ok=True)
res = gate.run_gate(loop, em.model_name(target), out_dir=od)
rows = [r for r in res.samples if r['phase'] == '정지 대기']
# first rest wait only: rows until a gap
first = [rows[0]]
for r in rows[1:]:
    if r['t'] - first[-1]['t'] > 1.0: break
    first.append(r)
tr = loop.trace
def sat_at(sim):
    k = min(range(len(tr)), key=lambda i: abs(tr[i][0] - sim))
    return tr[k][8], tr[k][9], tr[k][10], tr[k][11]
print('%s %s P %.2f: first rest wait, %d rows (%.1f s)' % (target, band, P, len(first), first[-1]['t'] - first[0]['t']))
for r in first[::8]:
    sp, sn, ms, mp_ = sat_at(r['sim'])
    print('  t %7.2f rate %+6.2f deg/s torque %+.4f integ %+.4f satP %d satN %d  %s/%s' % (r['t'], r['rate'], r['torque'], r['integ'], sp, sn, ms, mp_))
