#!/usr/bin/env python3
"""Roll angle (and yaw rate) of the gz ground truth vs the closed-loop model,
in 10 s windows aligned on the Manual step (first stick-up)."""
import math, os, sys, bisect
import numpy as np
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pyulog import ULog
import closed_loop, gate, reproduce

target, log = sys.argv[1], sys.argv[2]
u = ULog(log, message_name_filter_list=['vehicle_attitude_groundtruth', 'manual_control_setpoint', 'vehicle_angular_velocity'])
D = {d.name: d.data for d in u.data_list if d.multi_id == 0}
at = D['vehicle_attitude_groundtruth']; ta = at['timestamp'] * 1e-6
q = np.vstack([at['q[%d]' % i] for i in range(4)]).T
roll = np.degrees(np.arctan2(2 * (q[:, 0] * q[:, 1] + q[:, 2] * q[:, 3]), 1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2)))
mc = D['manual_control_setpoint']; tm = mc['timestamp'] * 1e-6; st = mc['yaw']
up_log = [tm[i] for i in range(1, len(st)) if st[i] > 0.25 and st[i - 1] <= 0.25]

loop = closed_loop.build(target, trace=True, params=reproduce.record_params(target)[0])  # the gains the chain7/chain9 records were flown with (record_gains)
gate.run_gate(loop, (closed_loop.TARGETS.get(target) or closed_loop.EXTRA_TARGETS[target])[0])
tr = loop.trace
tt = np.array([r[0] for r in tr]); stk = np.array([r[2] for r in tr])
phi = np.array([r[10].get('phi_deg', 0.0) for r in tr])
up_mod = [tt[i] for i in range(1, len(stk)) if stk[i] > 0.25 and stk[i - 1] <= 0.25]
print('stick-ups log', ['%.1f' % (x - up_log[0]) for x in up_log], ' model', ['%.1f' % (x - up_mod[0]) for x in up_mod])
for a in range(-20, 260, 10):
    ml = (ta - up_log[0] >= a) & (ta - up_log[0] < a + 10)
    mm = (tt - up_mod[0] >= a) & (tt - up_mod[0] < a + 10)
    if ml.sum() < 5 or mm.sum() < 5:
        continue
    print('t %+4d..%+4d  roll half p-p  gz %.2f  model %.2f   (rms gz %.2f model %.2f)' % (
        a, a + 10, (roll[ml].max() - roll[ml].min()) / 2, (phi[mm].max() - phi[mm].min()) / 2,
        np.sqrt(np.mean((roll[ml] - roll[ml].mean()) ** 2)), np.sqrt(np.mean((phi[mm] - phi[mm].mean()) ** 2))))
