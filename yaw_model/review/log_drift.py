#!/usr/bin/env python3
"""Reviewer's check: the hull's drift (u, v at the model origin), roll and pitch
in the gz ground truth of a chain gate run, per gate phase, from the flight log
(read only). The phases are found from manual_control_setpoint.yaw and
vehicle_status.nav_state (1 ACRO? see below)."""
import math, sys
import numpy as np
from pyulog import ULog

path = sys.argv[1]
u = ULog(path, message_name_filter_list=['vehicle_local_position_groundtruth', 'vehicle_attitude_groundtruth',
                                         'manual_control_setpoint', 'vehicle_status', 'vehicle_angular_velocity'])
D = {d.name: d.data for d in u.data_list if d.multi_id == 0}
lp = D['vehicle_local_position_groundtruth']
at = D['vehicle_attitude_groundtruth']
mc = D['manual_control_setpoint']
vs = D['vehicle_status']
t_lp = lp['timestamp'] * 1e-6
x, y = lp['x'], lp['y']
t_at = at['timestamp'] * 1e-6
q = np.vstack([at['q[%d]' % i] for i in range(4)]).T
roll = np.degrees(np.arctan2(2 * (q[:, 0] * q[:, 1] + q[:, 2] * q[:, 3]), 1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2)))
pitch = np.degrees(np.arcsin(np.clip(2 * (q[:, 0] * q[:, 2] - q[:, 3] * q[:, 1]), -1, 1)))
yaw = np.arctan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2))


def vel(t, win=0.5):
    i0 = np.searchsorted(t_lp, t - win / 2)
    i1 = np.searchsorted(t_lp, t + win / 2) - 1
    dt = t_lp[i1] - t_lp[i0]
    vx = (x[i1] - x[i0]) / dt
    vy = (y[i1] - y[i0]) / dt
    psi = np.interp(t, t_at, np.unwrap(yaw))
    ub = vx * math.cos(psi) + vy * math.sin(psi)          # body forward
    vb = -vx * math.sin(psi) + vy * math.cos(psi)         # body right (FRD)
    return ub, vb, math.hypot(vx, vy)


t_mc = mc['timestamp'] * 1e-6
stick = mc['yaw']
t_vs = vs['timestamp'] * 1e-6
nav = vs['nav_state']
# stick steps up to 0.5
ups = [t_mc[i] for i in range(1, len(stick)) if stick[i] > 0.25 and stick[i - 1] <= 0.25]
downs = [t_mc[i] for i in range(1, len(stick)) if stick[i] <= 0.25 and stick[i - 1] > 0.25]
print('stick up at', ['%.2f' % (s - t_lp[0]) for s in ups])
print('nav states:', sorted(set((int(n)) for n in nav)))
names = ['Manual', 'Acro push', 'Stab push']
for k, (t0, t1) in enumerate(zip(ups, downs)):
    navk = int(nav[np.searchsorted(t_vs, t0) - 1])
    print('\n== %s (nav %d), %.1f s long, log t %.2f' % (names[k] if k < 3 else 'step %d' % k, navk, t1 - t0, t0 - t_lp[0]))
    for dt in np.arange(0, t1 - t0 + 12.0, 2.0):
        t = t0 + dt
        ub, vb, V = vel(t)
        print('   t+%5.1f  u %+.4f  v(FLU) %+.4f  |V| %.4f  roll %+6.2f  pitch %+6.2f' % (
            dt, ub, -vb, V, np.interp(t, t_at, roll), np.interp(t, t_at, pitch)))
# the first (at-rest) wind check window: from arming+? print pitch/roll at rest
i = np.searchsorted(t_at, ups[0] - 12)
print('\nattitude ~12 s before the Manual step: roll %+.3f pitch %+.3f deg' % (roll[i], pitch[i]))
