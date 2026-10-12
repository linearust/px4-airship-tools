#!/usr/bin/env python3
"""Reviewer's independent check: the model's HIDDEN states (hull drift u, v,
roll phi) against the SITL flight logs of the same gate runs. The gate CSVs
log only rate/torque/integ, so the model's explanation of the Cloudship
tracking failure (drift x Munk) and of the rigid vectored ripple (roll x ixz)
was fitted/checked only through the rate. Here:

  drift  vehicle_gps_position.vel_n_m_s / vel_e_m_s (gz navsat on base_link,
         no pose -> the link origin O, cloudship/model.sdf:288-292,
         30 Hz, no noise block) rotated into the body by the EKF heading
         (vehicle_local_position.heading) -> u (fwd), v (FLU, port)
  roll   vehicle_attitude.q -> roll angle
  rate   vehicle_angular_velocity.xyz[2] (FRD)

Alignment: the first logged vehicle_torque_setpoint.xyz[2] >= 0.49 (the Manual
stick 0.5) against the model trace's first torque >= 0.49.
"""
import math
import os
import sys

import numpy as np
from pyulog import ULog

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import closed_loop as cl  # noqa: E402
import gate  # noqa: E402
import reproduce as rp  # noqa: E402

LOG = '/home/h/dev/PX4-Autopilot/build/px4_sitl_default/rootfs/fs/log'
RUNS = [
    ('gz_cloudship_am_fin_drag', '2026-09-25/14_50_29.ulg', 'chain7'),
    ('gz_cloudship_am_fin_drag', '2026-09-27/08_53_51.ulg', 'chain9'),
    ('gz_cloudship', '2026-09-27/08_31_54.ulg', 'chain9'),
    ('gz_airship_vectored_independent', '2026-09-27/08_36_30.ulg', 'chain9'),
    ('gz_airship_vectored_independent_am_fin_drag', '2026-09-27/08_47_52.ulg', 'chain9'),
    ('gz_airship_vectored_independent_fin', '2026-09-27/08_42_18.ulg', 'chain9'),
]
TOPICS = ['vehicle_gps_position', 'vehicle_local_position', 'vehicle_attitude', 'vehicle_angular_velocity',
          'vehicle_torque_setpoint']


def load(path):
    u = ULog(path, TOPICS)
    d = {x.name: x.data for x in u.data_list if x.multi_id == 0}
    return d, int(u.initial_parameters['SYS_AUTOSTART'])


def interp(t, tx, x):
    return np.interp(t, tx, x)


def main(which=None):
    for target, logname, tag in RUNS:
        if which and target != which:
            continue
        d, autostart = load(os.path.join(LOG, logname))
        tq = d['vehicle_torque_setpoint']
        t_tq = tq['timestamp_sample'] * 1e-6
        i0 = int(np.argmax(tq['xyz[2]'] >= 0.49))
        t_log0 = t_tq[i0]

        loop = cl.build(target, trace=True, params=rp.record_params(target)[0])  # the gains the chain7/chain9 records were flown with (record_gains)
        res = gate.run_gate(loop, (cl.TARGETS.get(target) or cl.EXTRA_TARGETS[target])[0])
        tr = loop.trace
        ts = np.array([x[0] for x in tr])
        tqm = np.array([x[4] for x in tr])
        t_mod0 = ts[int(np.argmax(tqm >= 0.49))]
        off = t_log0 - t_mod0

        gps = d['vehicle_gps_position']
        t_g = gps['timestamp_sample'] * 1e-6 - off
        lp = d['vehicle_local_position']
        t_lp = lp['timestamp_sample'] * 1e-6 - off
        att = d['vehicle_attitude']
        t_a = att['timestamp_sample'] * 1e-6 - off
        q0, q1, q2, q3 = (att['q[%d]' % k] for k in range(4))
        roll = np.degrees(np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)))
        av = d['vehicle_angular_velocity']
        t_av = av['timestamp_sample'] * 1e-6 - off
        rate_log = np.degrees(av['xyz[2]'])

        # model series on the trace grid
        u_m = np.array([x[-1]['u'] for x in tr])
        v_m = np.array([x[-1]['v'] for x in tr])
        phi_m = np.array([x[-1].get('phi_deg', 0.0) for x in tr])
        rate_m = np.degrees(np.array([x[3] for x in tr]))
        spd_m = np.hypot(u_m, v_m)

        # log series on the model grid
        vn = interp(ts, t_g, gps['vel_n_m_s'])
        ve = interp(ts, t_g, gps['vel_e_m_s'])
        hd = np.unwrap(lp['heading'])
        psi = interp(ts, t_lp, hd)
        u_l = vn * np.cos(psi) + ve * np.sin(psi)
        v_l = -(-vn * np.sin(psi) + ve * np.cos(psi))      # FRD y -> FLU y
        spd_l = np.hypot(vn, ve)
        phi_l = interp(ts, t_a, roll)
        rate_l = interp(ts, t_av, rate_log)
        ok = (ts > max(t_g[0], t_lp[0], t_a[0], t_av[0])) & (ts < min(t_g[-1], t_lp[-1], t_a[-1], t_av[-1]))

        print('\n=== %s  %s %s (SYS_AUTOSTART %d)  offset %.3f s  model status %s' %
              (target, tag, logname, autostart, off, res.status))
        # phases from the model's gate samples (sim column)
        phases = []
        last = None
        for row in res.samples:
            if row['phase'] != last:
                phases.append([row['phase'], row['sim'], row['sim']])
                last = row['phase']
            phases[-1][2] = row['sim']
        print('  %-40s %6s %6s | %-17s %-17s %-17s %-15s %-13s' % (
            'phase', 't0', 't1', 'speed m/s mod/log', 'u m/s mod/log', 'v m/s mod/log', 'roll dg rms m/l',
            'rate rms d'))
        for name, a, b in phases:
            sel = ok & (ts >= a) & (ts <= b)
            if sel.sum() < 5:
                continue
            e = sel.nonzero()[0][-1]
            s = sel.nonzero()[0][0]

            def rms(x):
                return math.sqrt(float(np.mean(x[sel] ** 2)))
            print('  %-40s %6.1f %6.1f | %5.3f>%5.3f/%5.3f>%5.3f %+6.3f/%+6.3f   %+6.3f/%+6.3f   %5.3f/%5.3f     %5.2f' % (
                name[:40], a, b, spd_m[s], spd_m[e], spd_l[s], spd_l[e], u_m[e], u_l[e], v_m[e], v_l[e],
                rms(phi_m - np.mean(phi_m[sel])), rms(phi_l - np.mean(phi_l[sel])), rms(rate_m - rate_l)))
        # push detail for the Cloudship complete
        for name, a, b in phases:
            if name in ('Acro 추종', 'Stabilized 추종') and 'cloudship_am' in target:
                print('  %s detail (every 2 s): t, rate mod/log, u mod/log, v mod/log, |V| mod/log' % name)
                for k in range(0, 36, 3):
                    j = int(np.argmin(abs(ts - (a + k))))
                    print('    %5.1f  %6.2f/%6.2f  %+6.3f/%+6.3f  %+6.3f/%+6.3f  %5.3f/%5.3f' % (
                        k, rate_m[j], rate_l[j], u_m[j], u_l[j], v_m[j], v_l[j], spd_m[j], spd_l[j]))
        sys.stdout.flush()


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else None)
