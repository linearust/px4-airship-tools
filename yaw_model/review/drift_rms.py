#!/usr/bin/env python3
"""rms of model - log body drift (u, v) over the closed-loop pushes, GPS
velocity smoothed over 1 s (the navsat has ~0.025 m/s per-sample noise at
rest: see the pre-arm rows), and the same for a 'null' model u=v=0."""
import math, os, sys
import numpy as np
from pyulog import ULog
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE)); sys.path.insert(0, HERE)
import closed_loop as cl, gate, reproduce as rp
from drift_check import RUNS, LOG, gnss_velocity
for target, logname, tag in RUNS:
    u = ULog(os.path.join(LOG, logname), ['vehicle_gps_position', 'vehicle_gnss', 'vehicle_local_position', 'vehicle_torque_setpoint'])
    d = {x.name: x.data for x in u.data_list if x.multi_id == 0}
    tq = d['vehicle_torque_setpoint']
    t_log0 = tq['timestamp_sample'][int(np.argmax(tq['xyz[2]'] >= 0.49))] * 1e-6
    loop = cl.build(target, trace=True, params=rp.record_params(target)[0])  # the gains the chain7/chain9 records were flown with (record_gains)
    res = gate.run_gate(loop, (cl.TARGETS.get(target) or cl.EXTRA_TARGETS[target])[0], out_dir=os.path.join(HERE, 'tmp_out', 'rms'))
    tr = loop.trace; ts = np.array([x[0] for x in tr])
    off = t_log0 - ts[int(np.argmax(np.array([x[4] for x in tr]) >= 0.49))]
    tg, g_vn, g_ve = gnss_velocity(d); tg = tg - off
    k = np.ones(30) / 30
    vn = np.convolve(g_vn, k, 'same'); ve = np.convolve(g_ve, k, 'same')
    lp = d['vehicle_local_position']; psi = np.interp(ts, lp['timestamp_sample'] * 1e-6 - off, np.unwrap(lp['heading']))
    vn, ve = np.interp(ts, tg, vn), np.interp(ts, tg, ve)
    ul = vn * np.cos(psi) + ve * np.sin(psi); vl = vn * np.sin(psi) - ve * np.cos(psi)
    um = np.array([x[-1]['u'] for x in tr]); vm = np.array([x[-1]['v'] for x in tr])
    out = []
    for ph in ('Acro 추종', 'Stabilized 추종', 'Acro 제동'):
        rows = [r['sim'] for r in res.samples if r['phase'] == ph]
        s = (ts >= rows[0]) & (ts <= rows[-1])
        e = math.sqrt(np.mean((um[s] - ul[s]) ** 2 + (vm[s] - vl[s]) ** 2))
        n = math.sqrt(np.mean(ul[s] ** 2 + vl[s] ** 2))
        out.append('%s: |dV| rms %.3f m/s (log |V| rms %.3f)' % (ph, e, n))
    print('%-45s %s  ' % (target, tag) + ';  '.join(out), flush=True)
