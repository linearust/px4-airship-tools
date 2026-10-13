#!/usr/bin/env python3
"""Cycle-level check of controller.py against the SITL flight logs of the gate
runs: drive SitlYawChain (controller + allocator + status cadence) with the
LOGGED rate, sticks, modes and parameter changes, no plant, and compare every
logged output of the chain.

    python3 ulog_check.py [log.ulg ...]      # default: the gate runs listed below

What the logs show about the timing, before any model (printed per log):
every logged timestamp_sample of vehicle_angular_velocity,
vehicle_torque_setpoint and actuator_* is a multiple of 4000 us with
timestamp == timestamp_sample (lockstep: the whole chain runs at the IMU
sample time), and control_allocator_status is published only at times that
are 0 mod 8000 us, i.e. every second 4 ms cycle (ControlAllocator.cpp:464).

Inputs taken from the log (the logger decimates, so between samples the rate
is interpolated linearly and the others hold):
  rate        vehicle_angular_velocity.xyz[2] (already filtered, so the
              controller's filter is off here), logged every ~10 ms
  stick       vehicle_rates_setpoint.yaw inverted through the deadzone while
              the loop is closed (published on the cycle a new stick is read:
              publishRatesSetpoint under new_sticks in airship_att_control_main.cpp),
              vehicle_torque_setpoint.xyz[2] in Manual (torque ==
              stick there), manual_control_setpoint.yaw (5 Hz) otherwise
  mode        vehicle_control_mode (armed, manual, rates flags), taken from the
              cycle after its timestamp
  I gain      the logged AS_YAWRATE_I changes, from the cycle after
Outputs compared: vehicle_torque_setpoint.xyz[2] (every logged sample),
rate_ctrl_status.yawspeed_integ, control_allocator_status.unallocated_torque[2],
actuator_servos (tilts) and actuator_motors (pods, tail).
"""
import bisect
import math
import os
import sys

import numpy as np
from pyulog import ULog

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import controller as C  # noqa: E402

LOGDIR = os.path.join(C.PX4, 'build/px4_sitl_default/rootfs/fs/log')
# The clean-start gate runs: 2026-09-25 on 9535f8abcd (the recorded results
# in the task; firmware differs from HEAD only in propulsiveYaw(), true on
# both airframes) and the 2026-09-27 chain9 runs on 1965de11ab.
DEFAULT_LOGS = [
    ('2026-09-25/14_34_05.ulg', 'vectored'),     # 2520 gz_airship_vectored_independent
    ('2026-09-25/14_39_53.ulg', 'vectored'),     # 2527 ..._am_fin_drag
    ('2026-09-25/14_45_54.ulg', 'cloudship'),    # 2510 gz_cloudship
    ('2026-09-25/14_50_29.ulg', 'cloudship'),    # 2517 gz_cloudship_am_fin_drag
    ('2026-09-27/08_31_54.ulg', 'cloudship'),    # 2510, chain9
    ('2026-09-27/08_36_30.ulg', 'vectored'),     # 2520, chain9
]
# HEAD 4e1baea97c, chain14 of 2026-09-29: the three airframes that fly a
# feedforward (record_gains.RUNS), the check of the FF port against SITL.
# Every log listed here predates the speed schedule (increment 1,
# 2026-10-07) and the restore rule with the quadratic law (increment A,
# 2026-10-12): check_log models them with the linear law and the restore rule
# off, which is what their firmware ran.
HEAD_FF_LOGS = [
    ('2026-09-29/03_05_52.ulg', 'vectored'),     # 2527, AS_YAWRATE_FF 1.0
    ('2026-09-29/03_12_03.ulg', 'vectored'),     # 2522, 0.3
    ('2026-09-29/03_22_07.ulg', 'cloudship'),    # 2517, 0.05
]
FRAME_OF_AUTOSTART = {2510: 'cloudship', 2512: 'cloudship', 2517: 'cloudship', 2520: 'vectored', 2522: 'vectored',
                      2527: 'vectored'}
TOPICS = ['vehicle_angular_velocity', 'vehicle_torque_setpoint', 'rate_ctrl_status', 'control_allocator_status',
          'vehicle_rates_setpoint', 'vehicle_control_mode', 'manual_control_setpoint', 'actuator_servos',
          'actuator_motors']


def stats(x):
    x = np.sort(np.abs(np.asarray(x)))
    if len(x) == 0:
        return 'n=0'
    return 'n=%d median %.2e p95 %.2e p99 %.2e max %.2e' % (len(x), x[len(x) // 2], x[int(0.95 * (len(x) - 1))],
                                                             x[int(0.99 * (len(x) - 1))], x[-1])


def stick_from_sp(sp, rmax, dz):
    if sp == 0.0:
        return 0.0
    return math.copysign(abs(sp) / rmax * (1.0 - dz) + dz, sp)


def ff_law_of(ip):
    """The feedforward law the log's firmware ran, from its parameter set: no
    AS_YR_FF_SLP is the linear law before 2026-10-07; AS_YR_FF_SLP without
    AS_SPD_LO is the quadratic law since increment A (2026-10-12). A log of
    the firmware between (both names) must be given its law explicitly."""
    if 'AS_YR_FF_SLP' not in ip:
        return 'linear'

    if 'AS_SPD_LO' not in ip:
        return 'quadratic'

    raise ValueError('a log of the 2026-10-07..10-12 firmware: pass ff_law')


def check_log(path, frame=None, mode_lag_cycles=0, infer_stick=True, verbose=True, ff_law=None):
    """The model against one log; ff_law None reads the law from the log
    (ff_law_of). controller.FF_LAW is set for the run and put back."""
    u = ULog(path, TOPICS)
    ip = u.initial_parameters
    law, C.FF_LAW = C.FF_LAW, (ff_law or ff_law_of(ip))

    try:
        return _check_log(u, path, frame, mode_lag_cycles, infer_stick, verbose)
    finally:
        C.FF_LAW = law


def _check_log(u, path, frame, mode_lag_cycles, infer_stick, verbose):
    d = {x.name: x.data for x in u.data_list if x.multi_id == 0}
    ip = u.initial_parameters
    frame = frame or FRAME_OF_AUTOSTART[int(ip['SYS_AUTOSTART'])]
    name = '%s/%s (SYS_AUTOSTART %d, %s, %s)' % (os.path.basename(os.path.dirname(path)), os.path.basename(path),
                                                  ip['SYS_AUTOSTART'], frame, u.msg_info_dict.get('ver_sw', '')[:10])
    av, tq, rc = d['vehicle_angular_velocity'], d['vehicle_torque_setpoint'], d['rate_ctrl_status']
    st, rs, cm = d['control_allocator_status'], d['vehicle_rates_setpoint'], d['vehicle_control_mode']
    mc, sv, mo = d['manual_control_setpoint'], d['actuator_servos'], d['actuator_motors']

    # --- timing facts
    grid_bad = sum(int(np.sum((x['timestamp_sample'] % 4000 != 0) | (x['timestamp'] != x['timestamp_sample'])))
                   for x in (av, tq, sv, mo))
    grid_ok = grid_bad == 0
    st_phase = sorted(set((st['timestamp'] % 8000).tolist()))
    min_gyro_step = int(np.min(np.diff(av['timestamp_sample'])))

    params = dict(C.DEFAULT_PARAMS)
    for k in ('AS_YAWRATE_P', 'AS_YAWRATE_I', 'AS_YR_INT_LIM', 'AS_YAWRATE_MAX', 'MAN_DEADZONE'):
        params[k] = float(ip[k])
    # A log without AS_YAWRATE_FF is firmware before d2647379af, which had no
    # feedforward at all (record_gains.py says why that is 0, not a guess)
    params['AS_YAWRATE_FF'] = float(ip['AS_YAWRATE_FF']) if 'AS_YAWRATE_FF' in ip else 0.0
    rmax, dz = math.radians(params['AS_YAWRATE_MAX']), params['MAN_DEADZONE']
    i_changes = sorted((t, float(v)) for (t, n, v) in u.changed_parameters if n == 'AS_YAWRATE_I')

    # --- input schedules
    t_av = av['timestamp_sample'].astype(np.int64)
    r_av = av['xyz[2]'].astype(float)
    cm_t = cm['timestamp'].astype(np.int64)
    ev = []                                    # stick evidence (time, stick, priority)
    for t, y in zip(mc['timestamp'], mc['yaw']):
        ev.append((int(t), float(y), 0))
    for t, y in zip(rs['timestamp'], rs['yaw']):
        ev.append((int(t), stick_from_sp(float(y), rmax, dz), 1))
    ev.sort(key=lambda e: (e[0], e[2]))
    manual_tq = {}                             # Manual cycles: torque == stick
    tq_t = tq['timestamp_sample'].astype(np.int64)
    tq_y = tq['xyz[2]'].astype(float)

    def mode_at(t):
        i = bisect.bisect_right(cm_t, t - 4000 * mode_lag_cycles) - 1
        if i < 0:
            return False, 'manual'
        armed = bool(cm['flag_armed'][i])
        loop = bool(cm['flag_control_manual_enabled'][i]) and bool(cm['flag_control_rates_enabled'][i])
        return armed, ('acro' if loop else ('manual' if cm['flag_control_manual_enabled'][i] else 'auto'))

    for t, y in zip(tq_t, tq_y):
        armed, mode = mode_at(int(t))
        if armed and mode == 'manual':
            manual_tq[int(t)] = float(y)
    for t, y in manual_tq.items():
        ev.append((t, y, 2))
    ev.sort(key=lambda e: (e[0], e[2]))
    ev_t = [e[0] for e in ev]
    tq_at = {int(a): float(b) for a, b in zip(tq_t, tq_y)}
    override = None                            # (since, stick) from a closed-loop torque sample
    n_override = 0

    # --- run
    t_first = int(max(t_av[0], cm_t[0]))
    t_first -= t_first % 8000                  # status phase: published at 0 mod 8000 (as logged)
    t_last = int(t_av[-1])
    chain = C.SitlYawChain(frame, params=params, filter_gyro=False, t0_us=t_first - 4000)
    chain._last_status_pub = 0

    out_tq, out_integ, out_status, out_servo, out_motor = {}, {}, {}, {}, {}
    status_times = []
    ii = 0
    k_av = 0
    t = t_first
    while t <= t_last:
        while ii < len(i_changes) and i_changes[ii][0] < t:
            chain.ctrl.set_param('AS_YAWRATE_I', i_changes[ii][1])
            ii += 1
        while k_av + 1 < len(t_av) and t_av[k_av + 1] <= t:
            k_av += 1
        if k_av + 1 < len(t_av) and t_av[k_av] <= t:
            ta, tb = t_av[k_av], t_av[k_av + 1]
            rate = r_av[k_av] + (r_av[k_av + 1] - r_av[k_av]) * (t - ta) / (tb - ta)
        else:
            rate = r_av[min(k_av, len(r_av) - 1)]
        j = bisect.bisect_right(ev_t, t) - 1
        stick = ev[j][1] if j >= 0 else 0.0
        if override is not None and (j < 0 or override[0] >= ev[j][0]):
            stick = override[1]
        armed, mode = mode_at(t)
        # The logged evidence lags a stick change by up to ~50-200 ms. Where a
        # closed-loop torque was logged this cycle, pick among the stick values
        # seen within 0.5 s the one whose P*(sp - rate) + I reproduces it:
        # a stick step of 0.5 moves the torque by P*0.1164 = 0.058, far above
        # any integrator error, so this only fixes WHEN the step happened
        # (to <= 20 ms) and is checked independently by the 5 Hz integrator.
        if infer_stick and t in tq_at and armed and mode == 'acro':
            lo, hi = bisect.bisect_left(ev_t, t - 500000), bisect.bisect_right(ev_t, t + 500000)
            cands = sorted(set([stick] + [ev[q][1] for q in range(lo, hi)]))
            c = chain.ctrl

            def pred(sv):
                sp = C.yaw_rate_setpoint(sv, dz, rmax)
                return c.rc.p * (sp - rate) + c.rc.integ + c.rc.ff * sp if c._was_active else None
            if c._was_active:
                best = min(cands, key=lambda sv: abs(pred(sv) - tq_at[t]))
                if best != stick and abs(pred(stick) - tq_at[t]) - abs(pred(best) - tq_at[t]) > 0.01:
                    stick = best
                    override = (t, best)
                    n_override += 1
        gen0 = chain.status_topic.gen
        tqm, _ = chain.step(rate, stick, mode, armed=armed, sticks_valid=True)
        assert chain.t_us == t
        out_tq[t] = (tqm, chain.ctrl.loop_active)
        out_integ[t] = chain.ctrl.integ
        if chain.status_topic.gen != gen0:
            status_times.append(t)
            out_status[t] = chain.status_topic.msg['unallocated_torque'][2]
        out_servo[t] = tuple(chain.alloc.servos)
        out_motor[t] = tuple(chain.alloc.motors)
        t += 4000

    # --- compare
    e_tq = [out_tq[int(ts)][0] - y for ts, y in zip(tq_t, tq_y) if int(ts) in out_tq and out_tq[int(ts)][1]]
    e_tq_open = [out_tq[int(ts)][0] - y for ts, y in zip(tq_t, tq_y) if int(ts) in out_tq and not out_tq[int(ts)][1]]
    e_i = [out_integ[int(ts)] - y for ts, y in zip(rc['timestamp'], rc['yawspeed_integ']) if int(ts) in out_integ]
    agree = total = 0
    mism = []
    # against the model's latest status at or before the logged one: a rare
    # skipped cycle in SITL shifts the logged publish phase by 4 ms for a while
    for ts, y in zip(st['timestamp'], st['unallocated_torque[2]']):
        j = bisect.bisect_right(status_times, int(ts)) - 1
        if j >= 0 and int(ts) - status_times[j] <= 8000:
            total += 1
            m = out_status[status_times[j]]
            if float(y) == m:
                agree += 1
            else:
                mism.append((int(ts), float(y), m))
    n_sat_log = int(np.sum(st['unallocated_torque[2]'] != 0))
    nserv = len(C.AirshipAllocator(frame).servos) if frame == 'vectored' else 1
    e_sv = [out_servo[int(ts)][i] - float(sv['control[%d]' % i][k]) for k, ts in enumerate(sv['timestamp_sample'])
            if int(ts) in out_servo for i in range(nserv)]
    nmot = 3 if frame == 'cloudship' else 2
    e_mo = [out_motor[int(ts)][i] - float(mo['control[%d]' % i][k]) for k, ts in enumerate(mo['timestamp_sample'])
            if int(ts) in out_motor for i in range(nmot)]
    tq_scale = float(np.max(np.abs(tq_y)))
    i_scale = float(np.nanmax(np.abs(rc['yawspeed_integ'])))

    res = dict(name=name, grid_ok=grid_ok, grid_bad=grid_bad, st_phase=st_phase, min_gyro_step=min_gyro_step,
               e_tq=e_tq, e_tq_open=e_tq_open, e_i=e_i, agree=agree, total=total, n_sat_log=n_sat_log,
               e_sv=e_sv, e_mo=e_mo, mism=mism, i_scale=i_scale, tq_scale=tq_scale,
               out_tq=out_tq, out_integ=out_integ, n_override=n_override)
    if verbose:
        print(name)
        print('  timing: logged samples off the 4 ms grid or with timestamp != timestamp_sample: %d; smallest gyro '
              'step %d us; control_allocator_status publish times mod 8000 us: %s'
              % (grid_bad, min_gyro_step, st_phase))
        print('  gains: P %g I %g INT_LIM %g FF %g%s, law %s' % (params['AS_YAWRATE_P'], params['AS_YAWRATE_I'],
                                                               params['AS_YR_INT_LIM'], params['AS_YAWRATE_FF'],
                                                               '' if 'AS_YAWRATE_FF' in ip else ' (no FF parameter)',
                                                               C.FF_LAW))
        print('  torque, loop closed : %s  (stick step times taken from the torque %d times)'
              % (stats(e_tq), n_override))
        print('  torque, loop open   : %s' % stats(e_tq_open))
        print('  integrator (|I| max %.4f): %s' % (i_scale, stats(e_i)))
        print('  yaw unallocated sign: %d/%d logged status samples agree; %d logged samples carry a yaw flag'
              % (agree, total, n_sat_log))
        if mism[:5]:
            print('    first disagreements (t, logged, model): %s' % mism[:5])
        print('  tilt servos          : %s' % stats(e_sv))
        print('  motors               : %s' % stats(e_mo))
    return res


if __name__ == '__main__':
    args = sys.argv[1:]
    logs = [(a, None) for a in args] if args else [(os.path.join(LOGDIR, p), f) for p, f in DEFAULT_LOGS + HEAD_FF_LOGS]
    for p, f in logs:
        if os.path.exists(p):
            check_log(p, f)
        else:
            print('missing', p)
