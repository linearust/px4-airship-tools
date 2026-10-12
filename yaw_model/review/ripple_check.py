#!/usr/bin/env python3
"""Alignment-free comparison of the fast parts: per phase, the rms of the yaw
rate minus its 2 s centred moving average (the ripple the gate's max-of-
samples peak and one-row brake reading pick up), and the roll angle's rms
about its phase mean with its dominant period, model vs flight log."""
import math
import os
import sys

import numpy as np
from pyulog import ULog

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
import closed_loop as cl  # noqa: E402
import gate  # noqa: E402
import reproduce as rp  # noqa: E402
from drift_check import RUNS, LOG  # noqa: E402


def movavg(x, n):
    k = np.ones(n) / n
    return np.convolve(x, k, mode='same')


def period(t, x):
    x = x - x.mean()
    z = np.nonzero((x[:-1] < 0) & (x[1:] >= 0))[0]
    return float(np.median(np.diff(t[z]))) if len(z) > 2 else float('nan')


def main(which=None):
    for target, logname, tag in RUNS:
        if which and target != which:
            continue
        u = ULog(os.path.join(LOG, logname), ['vehicle_angular_velocity', 'vehicle_attitude', 'vehicle_torque_setpoint'])
        d = {x.name: x.data for x in u.data_list if x.multi_id == 0}
        tq = d['vehicle_torque_setpoint']
        t_log0 = tq['timestamp_sample'][int(np.argmax(tq['xyz[2]'] >= 0.49))] * 1e-6
        loop = cl.build(target, trace=True, params=rp.record_params(target)[0])  # the gains the chain7/chain9 records were flown with (record_gains)
        res = gate.run_gate(loop, (cl.TARGETS.get(target) or cl.EXTRA_TARGETS[target])[0],
                            out_dir=os.path.join(HERE, 'tmp_out', 'ripple'))
        tr = loop.trace
        ts = np.array([x[0] for x in tr])
        t_mod0 = ts[int(np.argmax(np.array([x[4] for x in tr]) >= 0.49))]
        off = t_log0 - t_mod0
        # uniform 0.02 s grids
        grid = np.arange(ts[0], ts[-1], 0.02)
        rm = np.interp(grid, ts, np.degrees([x[3] for x in tr]))
        pm = np.interp(grid, ts, [x[-1].get('phi_deg', 0.0) for x in tr])
        av = d['vehicle_angular_velocity']
        rl = np.interp(grid, av['timestamp_sample'] * 1e-6 - off, np.degrees(av['xyz[2]']))
        att = d['vehicle_attitude']
        q0, q1, q2, q3 = (att['q[%d]' % k] for k in range(4))
        roll = np.degrees(np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)))
        pl = np.interp(grid, att['timestamp_sample'] * 1e-6 - off, roll)
        hm, hl = rm - movavg(rm, 100), rl - movavg(rl, 100)
        print('\n=== %s %s %s' % (target, tag, logname))
        print('  %-36s | %-19s | %-19s | %-15s' % ('phase', 'rate ripple rms m/l', 'roll rms deg m/l', 'roll period m/l'))
        phases, last = [], None
        for row in res.samples:
            if row['phase'] != last:
                phases.append([row['phase'], row['sim'], row['sim']])
                last = row['phase']
            phases[-1][2] = row['sim']
        for name, a, b in phases:
            s = (grid >= a + 1.0) & (grid <= b - 1.0)
            if s.sum() < 100:
                continue
            print('  %-36s |   %5.3f / %5.3f     |   %5.3f / %5.3f     |  %4.2f / %4.2f' % (
                name[:36], math.sqrt(np.mean(hm[s] ** 2)), math.sqrt(np.mean(hl[s] ** 2)),
                np.std(pm[s]), np.std(pl[s]), period(grid[s], pm[s]), period(grid[s], pl[s])))
        sys.stdout.flush()


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else None)
