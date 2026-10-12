#!/usr/bin/env python3
"""Tests of the throttle path and the forward driver. Plain python:

    python3 test_forward.py

The throttle reaches the pods the way the firmware passes it (sitl_link's
truncated z, the receiver's z / 500 - 1, airship_manual_input's
(throttle + 1) / 2, the allocator ports); thrust 0 leaves the gate's plant
untouched; trim holds straight flight, is gz_model.trim_thrust's, and refuses
an airframe on which the fraction is not linear in thrust; the drag-free fin
plants hold u at throttle 0; the reversal counters count crossings, not
dither, per direction, and the two kinds (pod swings, tail flips) stay apart.

Stage 1 of D1 step 0 (2026-10-01): FRD in and out; the 0b check's legs and
its midpoint throttle; marks past the end; the record's windows (0.5 s u
means, 1 s r means, the 0.5 s moving-mean peak); commands and reversals at a
log's own sample times; fin names and the plugin's angle from a recorded
state; the entry-state inputs (every one at its default bit-identical, each
one doing what it says, the impossible ones refused); a start at the release
from a leg's own state; replay_kwargs on a record written from a model leg,
from both instants; the per-cycle hook, the feedforward and cap primitives
(the cap at AS_YAWRATE_MAX bit-identical, the cap in both places), a wind
that moves only the ground speed, the restore rule on a leg, the tilt range
override (A = 0.5).
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import closed_loop as cl  # noqa: E402
import controller as C  # noqa: E402
import forward as F  # noqa: E402

FAILS = []


def check(name, ok, detail=''):
    print('%s  %s%s' % ('PASS' if ok else 'FAIL', name, ('  -- ' + detail) if detail else ''))
    if not ok:
        FAILS.append(name)


def test_throttle_mapping():
    flown = lambda f, q=True: (C.throttle_stick(f, q) + 1.0) * 0.5
    check('thrust fraction 0.1221 is sent as z 122: PX4 flies 0.122', abs(flown(0.1221) - 0.122) < 1e-12,
          '%.6f' % flown(0.1221))
    check('0.0305 is sent as z 30: int() truncates 30.5, as sitl_link sends it', abs(flown(0.0305) - 0.030) < 1e-12,
          '%.6f' % flown(0.0305))
    check('unquantized fraction passes exactly', abs(flown(0.1221, False) - 0.1221) < 1e-12)
    check('fraction clamps to 0..1', flown(-0.3) == 0.0 and flown(1.7) == 1.0)
    check('manual_thrust: throttle -1 is no thrust, NaN reads as -1, +1 is full',
          C.manual_thrust(-1.0, 0.0) == (0.0, 0.0, 0.0) and C.manual_thrust(float('nan'), 0.0)[0] == 0.0
          and C.manual_thrust(1.0, 0.0)[0] == 1.0)
    check('manual_thrust: the pitch stick is thrust_z, manual_torque: -pitch is pitch torque',
          C.manual_thrust(-1.0, 0.4)[2] == 0.4 and C.manual_torque(0.1, 0.4, 0.2) == (0.1, -0.4, 0.2))


def test_vectored_allocation_with_thrust():
    """The 2520 allocation (AirshipAllocator port) with a collective thrust c
    and a yaw u: the pods are asked (c - u, 0) and (c + u, 0)."""
    def settle(c, u, ticks=600):
        a = C.AirshipAllocator('vectored')
        for _ in range(ticks):          # 2.4 s at 4 ms: past a 180 deg swing at 120 deg/s
            a.update([0.0, 0.0, u, c, 0.0, 0.0], 0.004)
        return a

    a = settle(0.12, 0.3)
    st = a.status()['unallocated_torque'][2]
    check('yaw 0.3 over thrust 0.12: the starboard pod reverses (tilt 180) and pushes 0.18 back, port 0.42',
          abs(abs(math.degrees(a.pods[0].tilt)) - 180.0) < 1e-6 and abs(math.degrees(a.pods[1].tilt)) < 1e-6
          and abs(a.motors[0] - 0.18) < 1e-6 and abs(a.motors[1] - 0.42) < 1e-6,
          'tilts %.1f / %.1f, motors %.4f / %.4f' % (math.degrees(a.pods[0].tilt), math.degrees(a.pods[1].tilt),
                                                    a.motors[0], a.motors[1]))
    check('  ... and delivers the yaw and the thrust both, no shortfall',
          abs(a.achieved[2] - 0.3) < 1e-6 and abs(a.achieved[3] - 0.12) < 1e-6 and st == 0.0)
    a = settle(0.12, 0.05)
    check('yaw 0.05 under thrust 0.12: no reversal, motors 0.07 / 0.17',
          abs(math.degrees(a.pods[0].tilt)) < 1e-6 and abs(a.motors[0] - 0.07) < 1e-6 and abs(a.motors[1] - 0.17) < 1e-6)
    a = settle(0.12, 0.95)
    check('yaw 0.95 over thrust 0.12: the port pod saturates at 1, the shortfall is flagged',
          abs(a.motors[1] - 1.0) < 1e-9 and a.status()['unallocated_torque'][2] == 1.0)


def test_thrust_zero_is_the_gate():
    """With the gate's sticks (throttle -1) the pods' forces are exactly 0:
    the Cloudship's side motors give a zero wrench, the vectored pods' surge
    is the yaw couple's alone."""
    loop = cl.build('gz_cloudship_am_fin_drag', params=dict(C.DEFAULT_PARAMS))
    loop.arm()
    loop.set_stick(0.5)
    loop.set_mode('acro')
    loop.advance(2.0)
    w = loop.actuation.pods.wrench()
    check('Cloudship, gate sticks: the side motors\' wrench is exactly 0', all(x == 0.0 for x in w), str(w))
    loop = cl.build('gz_airship_vectored_independent', params=dict(C.DEFAULT_PARAMS))
    loop.arm()
    loop.set_stick(0.0)
    loop.set_mode('manual')
    loop.advance(1.0)
    check('vectored, gate sticks, stick centred: no pod thrust', loop.actuation.row['T_s'] == 0.0
          and loop.actuation.row['T_p'] == 0.0)


def test_cloudship_pods():
    """The side motors at thrust fraction c: each prop c x its full thrust
    (THR_MDL_FAC 1 linearises it), the pair pushes 2 c T_full forward and
    nothing in roll or yaw; the tail is untouched by the throttle."""
    import cloudship_actuation as ca
    pods = ca.CloudshipPods('cloudship_am_fin_drag')
    for _ in range(500):                   # 2 s, past the rotor's 0.05 s lag
        pods.step(0.004, [0.0, 0.0, 0.3, 0.25, 0.0, 0.0])
    w = pods.wrench()
    full = pods.full_thrust
    check('each prop delivers 0.25 x full thrust (%.3f N)' % full[0],
          all(abs(t - 0.25 * f) < 1e-6 * f for t, f in zip(pods.thrust, full)), str(pods.thrust))
    check('the pair: X = 2 x 0.25 x full, roll and yaw cancel', abs(w[0] - 0.25 * sum(full)) < 1e-6
          and abs(w[3]) < 1e-9 and abs(w[5]) < 1e-9, str(w))
    chain = ca.TailChain('cloudship_am')
    check('the tail command is the yaw alone (collective pods carry none)', chain.stages(0.3)['cmd'] == ca.F32(0.3))


def test_trim():
    t2, _, _ = F.trim_throttle('airship_vectored_independent_am_fin_drag', 2.0)
    t4, d4, full = F.trim_throttle('airship_vectored_independent_am_fin_drag', 4.0)
    check('2527 trim from the files: 0.0305 at 2 m/s, 0.1221 at 4 m/s',
          round(t2, 4) == 0.0305 and round(t4, 4) == 0.1221, '%.5f, %.5f (drag %.3f N on %.1f N)' % (t2, t4, d4, full))
    check('drag-free fin plants trim at 0', F.trim_throttle('airship_vectored_independent_fin', 4.0)[0] == 0.0
          and F.trim_throttle('cloudship_fin', 4.0)[0] == 0.0)
    gm = F.gz_model
    same = all(F.trim_throttle(m, u) == (gm.trim_thrust(m, u), gm.trim_thrust(m, u) * gm.pod_thrust(m),
                                          gm.pod_thrust(m))
               for m in ('airship_vectored_independent_am_fin_drag', 'cloudship_am_fin_drag', 'cloudship_fin')
               for u in (1.0, 2.0, 4.0, 6.0))
    check('trim_throttle is gz_model.trim_thrust (the formula forward_yaw_check flies), the drag its fraction of '
          'pod_thrust', same)
    numbers = [F.linear_thrust_check(m) for m in ('airship_vectored_independent_am_fin_drag',
                                                   'airship_vectored_independent_fin', 'cloudship_am_fin_drag',
                                                   'cloudship_fin')]
    check('linear_thrust_check passes the four forward plants\' airframes', numbers == [2527, 2522, 2517, 2512],
          str(numbers))


def test_linearity_refusals():
    """Each precondition of the trim's division, broken one at a time in the
    airframe chain controller.airframe_defaults returns, raises."""
    real = C.airframe_defaults

    def broken(change):
        def defaults(number):
            out = dict(real(number))
            for name, value in change.items():
                if value is None:
                    out.pop(name, None)
                else:
                    out[name] = (value, 'mutated', 0)
            return out
        return defaults

    for change, kind, what in (({'THR_MDL_FAC': '0.5'}, ValueError, 'THR_MDL_FAC 0.5'),
                               ({'THR_MDL_FAC': None}, LookupError, 'THR_MDL_FAC unset'),
                               ({'SIM_GZ_EC_MAX1': '250'}, ValueError, 'SIM_GZ_EC_MAX1 below maxRotVelocity'),
                               ({'SIM_GZ_EC_MAX2': '1600'}, ValueError, 'SIM_GZ_EC_MAX2 above maxRotVelocity'),
                               ({'SIM_GZ_EC_MIN2': '10'}, ValueError, 'SIM_GZ_EC_MIN2 not 0')):
        C.airframe_defaults = broken(change)
        try:
            raised = None
            for model in ('airship_vectored_independent_am_fin_drag', 'cloudship_am_fin_drag'):
                try:
                    F.trim_throttle(model, 4.0)
                except kind as exc:
                    raised = (raised or 0) + 1
                    detail = str(exc)
        finally:
            C.airframe_defaults = real
        check('trim refuses an airframe with %s (both vehicles)' % what, raised == 2, detail if raised else 'no raise')


def test_straight_flight():
    for target, u0, q in (('gz_airship_vectored_independent_fin', 4.0, True),
                          ('gz_cloudship_fin', 2.0, True),
                          ('gz_airship_vectored_independent_am_fin_drag', 4.0, False)):
        leg = F.run_leg(target, u0, 0.0, quantize=q, manual_s=5.0, centred_s=5.0, stick_s=0.1, release_s=0.1)
        u_end = leg['metrics']['u_release_end']
        tol = 1e-6 if leg['setup']['trim'] == 0.0 else 1e-4
        check('%s holds u %.1f m/s through 10 s of straight flight at throttle %.4f' % (target, u0, leg['setup']['flown']),
              abs(u_end - u0) < tol and abs(leg['metrics']['r_rel_120']) < 1e-9, 'u %.8f' % u_end)


def test_hold_and_determinism():
    a = F.run_leg('gz_airship_vectored_independent_am_fin_drag', 4.0, 1.0, throttle=0.0, hold_u=4.0,
                  centred_s=1.0, stick_s=5.0, release_s=5.0)
    check('hold_u keeps the surge at 4 m/s through a full-stick turn', abs(a['metrics']['u_stick_end'] - 4.0) < 1e-12
          and abs(a['metrics']['u_release_end'] - 4.0) < 1e-12)
    b = F.run_leg('gz_airship_vectored_independent_am_fin_drag', 4.0, 1.0, throttle=0.0, hold_u=4.0,
                  centred_s=1.0, stick_s=5.0, release_s=5.0)
    check('a leg is deterministic', a['metrics'] == b['metrics'])


def test_schmitt():
    s = F.Schmitt(0.5)
    for x in (1.0, 0.4, 0.6, 0.0, 0.3, 0.9):       # dither inside the band from the forward side
        s.update(x)
    check('a dither inside the band does not count', s.flips == 0)
    for x in (-0.7, -0.2, -0.6, 0.8):
        s.update(x)
    check('two crossings count two, one each way', s.flips == 2 and s.to_neg == 1 and s.to_pos == 1)
    z = F.Schmitt(0.0)
    for x in (0.01, 0.0, -0.001, 0.0, 0.002):
        z.update(x)
    check('band 0: every sign change counts, an exact zero is neither side', z.flips == 2)


def test_reversal_kinds():
    """Pod swings and tail flips are separate metrics; rev_* is the
    vehicle's own count (what forward_yaw_check's 'flips' is held against)."""
    import cloudship_actuation as ca
    v = F.run_leg('gz_airship_vectored_independent_am_fin_drag', 4.0, 0.5, centred_s=1.0, stick_s=10.0, release_s=5.0)
    m = v['metrics']
    swings = sum(d['into_rear'] + d['back_forward'] for d in m['pod_swings_stick'].values())
    check('vectored: per-pod swings, no tail metrics, rev_stick their sum (a half stick at 4 m/s puts one pod in '
          'the rear)', m['tail_flips_stick'] is None and m['tail_thrust_signs_stick'] is None and swings == m['rev_stick']
          and m['pod_swings_stick']['stbd']['into_rear'] == 1 and m['pod_swings_stick']['stbd']['rear_s'] > 0.0,
          str(m['pod_swings_stick']))
    c = F.run_leg('gz_cloudship_am_fin_drag', 4.0, 1.0, centred_s=1.0, stick_s=20.0, release_s=30.0)
    m = c['metrics']
    check('Cloudship: tail flips at the band and thrust sign changes, no pod metrics, rev_* the band count '
          '(2517 at 4 m/s departs and the tail reverses)',
          m['pod_swings_stick'] is None and m['rev_stick'] == m['tail_flips_stick'] > 0
          and m['rev_release'] == m['tail_flips_release'] and m['tail_thrust_signs_release'] >= m['tail_flips_release'],
          'flips %d / %d, signs %d / %d' % (m['tail_flips_stick'], m['tail_flips_release'],
                                             m['tail_thrust_signs_stick'], m['tail_thrust_signs_release']))
    chain = ca.TailChain('cloudship_am')
    check('the tail band\'s size is read from the tail chain: %.4f N, %.4f N m' % (c['setup']['tail_band_N'],
                                                                                 c['setup']['tail_band_Nm']),
          c['setup']['tail_band_N'] == F.TAIL_BAND * chain.full_thrust and c['setup']['tail_band_Nm'] == F.TAIL_BAND * chain.K
          and chain.min_command == 0.0 and not chain.lag_tags)


def test_off_grid_segments():
    """A replayed log's segment lengths are not multiples of 4 ms; the leg
    takes them to the step instead of never switching."""
    leg = F.run_leg('gz_airship_vectored_independent_fin', 2.0, 1.0, manual_s=0.5, centred_s=3.035, stick_s=2.357,
                    release_s=1.0)
    s, r = leg['entry']['stick']['t'], leg['entry']['release']['t']
    check('off-grid segments land on the nearest step', abs(s - 3.036) < 1e-9 and abs(r - 5.392) < 1e-9,
          'stick %.4f release %.4f' % (s, r))


def test_flown_throttle_reaches_the_pods():
    """Through the whole chain: a leg at the 2527 trim flies the sent
    fraction on both pods before any yaw."""
    leg = F.run_leg('gz_airship_vectored_independent_am_fin_drag', 4.0, 0.0, manual_s=1.0, centred_s=1.0,
                    stick_s=0.1, release_s=0.1)
    row = leg['entry']['acro']
    full = 0.5 * leg['setup']['full_thrust_N']
    check('each pod pushes the flown fraction x its full thrust in straight flight',
          abs(row['T_s'] - leg['setup']['flown'] * full) < 1e-6 and abs(row['T_p'] - leg['setup']['flown'] * full) < 1e-6,
          'T %.4f / %.4f N, flown %.3f x %.1f N' % (row['T_s'], row['T_p'], leg['setup']['flown'], full))


# ---------------------------------------------------------------------------
# Stage 1 of D1 step 0: the record's frames and windows, the entry state, the
# replay inputs, and the 0c policy primitives
# ---------------------------------------------------------------------------

V27, V22 = 'gz_airship_vectored_independent_am_fin_drag', 'gz_airship_vectored_independent_fin'
C12, C17 = 'gz_cloudship_fin', 'gz_cloudship_am_fin_drag'


def same(a, b):
    """== all the way down (dicts, lists, arrays), NaN equal to NaN."""
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(same(a[k], b[k]) for k in a)
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(same(x, y) for x, y in zip(a, b))
    if hasattr(a, 'tolist') and hasattr(b, 'tolist'):
        return a.tolist() == b.tolist()
    return a == b or (a != a and b != b)


def same_leg(a, b, skip_setup=True):
    """Two legs alike in everything they computed: metrics, entry states,
    series rows, trace (the setup records the inputs, which differ)."""
    return (same(a['metrics'], b['metrics']) and same(a['entry'], b['entry']) and same(a['series'], b['series'])
            and same(a['trace'], b['trace']) and (skip_setup or same(a['setup'], b['setup'])))


def test_frd_and_throttle():
    leg = F.run_leg(V22, 2.0, 0.0, v0=0.05, r0=1.5, p0=0.5, phi0=2.0, manual_s=0.0, centred_s=0.1, stick_s=0.1,
                    release_s=0.1)
    e = leg['entry']['acro']
    check('v0, r0, p0, phi0 are FRD in and out: the entry reads them back as given (v +0.05 m/s to starboard, '
          'r +1.5 deg/s nose right, p +0.5 deg/s, roll +2 deg right wing down)',
          abs(e['v'] - 0.05) < 1e-15 and abs(e['r'] - 1.5) < 1e-12 and abs(e['p'] - 0.5) < 1e-12
          and abs(e['phi'] - 2.0) < 1e-12 and leg['setup']['frame'] == 'FRD',
          'v %r r %r p %r phi %r' % (e['v'], e['r'], e['p'], e['phi']))
    check('the legs are the 0b check\'s own (forward_yaw_check.default_legs): 2527, 2522, 2512 at 2 and 4 m/s, '
          '2517 at 2.0 and 2.5 m/s', F.default_legs(V27) == F.default_legs(V22) == F.default_legs(C12)
          == [(2.0, 1.0), (4.0, 1.0), (4.0, 0.5)] and F.default_legs(C17) == [(2.0, 1.0), (2.5, 1.0), (2.5, 0.5)],
          str([F.default_legs(t) for t in (V27, C17)]))
    flown = {q: F.run_leg(V27, 2.0, 0.0, quantize=q, manual_s=0.0, centred_s=0.1, stick_s=0.1,
                          release_s=0.1)['setup'] for q in ('midpoint', True, False)}
    check('2527 at 2 m/s: by default the leg flies 31/1000 as the 0b check does, truncated 30/1000, exact the trim',
          abs(flown['midpoint']['flown'] - 0.031) < 1e-12 and abs(flown[True]['flown'] - 0.030) < 1e-12
          and abs(flown[False]['flown'] - flown[False]['trim']) < 1e-15 and flown['midpoint']['quantize'] == 'midpoint',
          'trim %.5f' % flown[False]['trim'])


def test_marks_and_windows():
    import numpy as np
    leg = F.run_leg(V27, 4.0, 1.0, noise=True, seed=2, centred_s=1.0, stick_s=8.0, release_s=30.0)
    m, (_, _, t_on, t_rel, t_end) = leg['metrics'], leg['setup']['times']
    check('a mark past the end is None (a 30 s release has no +60 s), a mark inside is the step at it',
          m['r_rel_60'] is None and m['integ_rel_60'] is None and m['r_mean1s_rel_60'] is None
          and m['r_rel_10'] == F.trace_at(leg, 'r', t_rel + 10.0) and m['r_rel_10'] is not None)
    near_end = F.run_leg(V27, 4.0, 1.0, centred_s=1.0, stick_s=8.0, release_s=59.8)['metrics']
    check('a mark past the end by no more than forward_yaw_check\'s DURATION_TOL_S (0.3 s) is the end, as its '
          'mark_time() takes it', near_end['r_rel_60'] == near_end['r_rel_120'] is not None
          and near_end['r_mean1s_rel_60'] == near_end['r_mean1s_rel_end'])
    check('u as 0.5 s means and r as 1 s means of the measured rate, on forward_yaw_check\'s windows',
          m['u_mean05_stick_start'] == F.trace_mean(leg, 'u', t_on, t_on + 0.5)
          and m['u_mean05_stick_end'] == F.trace_mean(leg, 'u', t_rel - 0.5, t_rel)
          and m['u_mean05_release_end'] == F.trace_mean(leg, 'u', t_end - 0.5, t_end, closed=True)
          and m['r_mean1s_rel_10'] == F.trace_mean(leg, 'r_meas', t_rel + 9.0, t_rel + 10.0)
          and abs(m['u_mean05_stick_start'] - m['u_stick_start']) < 0.05)
    T, RM = np.array(leg['trace']['t']), np.array(leg['trace']['r_meas'])
    rows = RM[(T >= t_on - 1e-9) & (T < t_rel - 1e-9)]
    conv = np.convolve(rows, np.ones(125) / 125.0, mode='valid')
    check('the smoothed peak is the extreme of the measured rate\'s 0.5 s (125-step) moving mean over the stick, '
          'below the noisy samples\' own peak', abs(m['r_peak_smoothed'] - conv.max()) < 1e-12
          and m['r_peak_smoothed'] < rows.max(), '%.5f vs %.5f, raw %.5f' % (m['r_peak_smoothed'], conv.max(), rows.max()))


def test_command_at_and_reversals():
    leg = F.run_leg(V27, 4.0, 0.5)        # G4's half-stick leg: stbd into the rear in the stick, back in the release
    T = leg['trace']['t']
    i = 1000
    check('command_at: a log stamped at a cycle time holds that cycle\'s command (the trace row one step later); a '
          'plant quantity the step that ended there', F.command_at(leg, 'tilt_s', [T[i] - F.DT])[0]
          == leg['trace']['tilt_s'][i] and F.command_at(leg, 'u', [T[i]])[0] == leg['trace']['u'][i]
          and F.command_at(leg, 'tilt_s', [T[0] - 2 * F.DT])[0] is None)
    _, _, t_on, t_off, t_end = leg['setup']['times']
    cycles = [t - F.DT for t in T]
    own = {'stick': leg['metrics']['pod_swings_stick'], 'release': leg['metrics']['pod_swings_release']}
    every = {'stick': F.reversals_at(leg, cycles, t_on, t_off), 'release': F.reversals_at(leg, cycles, t_off, t_end + 1e-6)}
    check('reversals_at on every cycle counts what the leg counts, with forward_yaw_check\'s own functions (%s)'
          % ', '.join('%s %d' % (seg, every[seg]['flips']) for seg in every),
          all(every[seg][n][k] == own[seg][n][k] for seg in own for n in F.POD_NAMES
              for k in ('into_rear', 'back_forward')) and every['stick']['flips'] == leg['metrics']['rev_stick']
          and every['release']['flips'] == leg['metrics']['rev_release'] == 1, str(every))
    servo_times = [k * 0.1 for k in range(int(t_end / 0.1) + 1)]
    at100 = F.reversals_at(leg, servo_times, t_off, t_end + 1e-6)
    check('at the log\'s 100 ms actuator_servos times the same swing counts (a swing takes the tilt through 60 deg '
          'at 120 deg/s, 0.5 s)', at100['flips'] == every['release']['flips'], str(at100))
    c = F.run_leg(C17, 4.0, 1.0, centred_s=1.0, stick_s=20.0, release_s=30.0)
    _, _, a, b, e = c['setup']['times']
    cyc = [t - F.DT for t in c['trace']['t']]
    tail = F.reversals_at(c, cyc, b, e + 1e-6)
    torque = F.reversals_at(c, cyc, b, e + 1e-6, source='torque')
    t20 = F.reversals_at(c, [k * 0.02 for k in range(int(e / 0.02) + 1)], b, e + 1e-6, source='torque')
    t100 = F.reversals_at(c, [k * 0.1 for k in range(int(e / 0.1) + 1)], b, e + 1e-6)
    check('Cloudship: tail flips counted on the tail command and on the torque setpoint agree (the tail is '
          'clamp(torque)), and match the leg\'s own; sampled at 20 ms and 100 ms they can only miss flips '
          '(every %d, torque at 20 ms %d, tail at 100 ms %d)' % (tail['flips'], t20['flips'], t100['flips']),
          tail['flips'] == torque['flips'] == c['metrics']['tail_flips_release'] > 0
          and tail['flips'] >= t20['flips'] >= 0 and tail['flips'] >= t100['flips'] >= 0)


def test_fin_names_and_alphas():
    import planar
    ok_names, worst, noflow = True, 0.0, True
    for target in (V27, V22, C12, C17):
        model = cl.ALL_TARGETS[target][0]
        plant = planar.RollPlanarPlant(target, model=model)
        ok_names = ok_names and F.fin_names(model) == [f['name'] for f in plant.fins if f['vertical']]
        plant.set_state(u=3.0, v=-0.2, p=0.05, r=-0.1, phi=0.05)            # FLU: sliding to port, turning right
        want = {n: (None if a is None else math.degrees(a)) for n, a, _ in plant.fin_state()}
        got = F.fin_alphas(target, 3.0, 0.2, math.degrees(0.05), math.degrees(0.1), math.degrees(0.05))
        worst = max([worst] + [abs(got[n] - want[n]) for n in want])
        noflow = noflow and all(x is None for x in F.fin_alphas(target, -1.0, 0.0, 0.0, 0.0, 0.0).values())
    check('fin_names: the record\'s alpha_fin<n> (gz_model.vertical_fins order) named dorsal/ventral as the plant '
          'names them, on the four forward plants', ok_names)
    check('fin_alphas from an FRD state is the plant\'s fin_state (the plugin\'s fold, signed by each fin\'s '
          'upward), and no flow (None) for flow from behind', worst < 1e-9 and noflow, 'worst %.1e deg' % worst)


def test_entry_state():
    kw = dict(centred_s=1.0, stick_s=6.0, release_s=6.0)
    a = F.run_leg(V27, 4.0, 1.0, **kw)
    b = F.run_leg(V27, 4.0, 1.0, start='acro', pods0=None, motors0=None, integ0=None, sat0=None, gyro_settled=False,
                  hook=None, restore=None, restore_trim0=None, wind=None, tilt_range=None, **kw)
    check('every new input left at its default is the leg as it was (bit-identical)', same_leg(a, b))
    glide = dict(manual_s=0.0, centred_s=1.0, stick_s=5.0, release_s=1.0)
    pods0 = [{'tilt': 180.0, 'mode': 'released'}, {'tilt': 0.0, 'mode': 'released'}]
    plain = F.run_leg(V22, 2.0, 1.0, **glide)
    set_ = F.run_leg(V22, 2.0, 1.0, pods0=pods0, **glide)
    p = set_['entry']['acro']['actuators']['pods']
    r1 = lambda leg: F.trace_at(leg, 'r', 2.0)
    check('pods0 on 2522 gliding (zero thrust, pods released): the allocator\'s pods and the gz joints start where '
          'given (stbd 180, port 0, released, the joint settled)', abs(p[0]['tilt'] - 180.0) < 1e-5
          and abs(p[0]['gz'] - 180.0) < 1e-5 and p[0]['mode'] == 'released' and abs(p[1]['tilt']) < 1e-9
          and p[0]['target'] == p[0]['tilt'], str(p))
    check('  ... and change the first swing: the starboard pod is already in the rear (no swing into it), so 1 s '
          'into the stick the yaw rate is ahead of the leg from the pods forward (%.3f against %.3f deg/s)'
          % (r1(set_), r1(plain)), set_['metrics']['pod_swings_stick']['stbd']['into_rear'] == 0
          and plain['metrics']['pod_swings_stick']['stbd']['into_rear'] == 1 and r1(set_) > r1(plain) + 0.1)
    motors = a['entry']['acro']['actuators']['motors']
    c = F.run_leg(V27, 4.0, 1.0, motors0=motors, **kw)
    d = max(abs(c['trace']['r'][i] - a['trace']['r'][i]) for i in range(len(a['trace']['r'])))
    check('motors0 at the warm-up\'s own commands: each rotor at its steady state is the warm-up\'s (r within '
          '%.1e deg/s over the leg)' % d, d < 1e-9)
    short = dict(manual_s=0.0, centred_s=0.5, stick_s=0.1, release_s=0.1)
    s1 = F.run_leg(V27, 4.0, 0.0, integ0=0.1, r0=5.0, gyro_settled=True, **short)['entry']['acro']
    s0 = F.run_leg(V27, 4.0, 0.0, r0=5.0, **short)['entry']['acro']
    check('integ0 is the integrator at the start; gyro_settled starts the IMU filter at r0 (5 deg/s) where it '
          'otherwise starts at 0', s1['integ'] == 0.1 and abs(s1['r_meas'] - 5.0) < 1e-9 and s0['r_meas'] == 0.0)
    on = dict(manual_s=0.0, centred_s=0.0, stick_s=0.2, release_s=0.1)
    held = F.run_leg(V27, 4.0, 1.0, sat0=(True, False), **on)
    free = F.run_leg(V27, 4.0, 1.0, **on)
    check('sat0 (positive) is held until a status of the leg\'s own: full stick from t 0 does not wind the '
          'integrator on the first cycle; without it, it does', held['entry']['acro']['sat_pos']
          and held['trace']['integ'][0] == 0.0 and free['trace']['integ'][0] > 0.0)
    refused = []
    for name, call in (('a release start without integ0', lambda: F.run_leg(V27, 4.0, 1.0, start='release')),
                       ('a pod without a mode', lambda: F.run_leg(V27, 4.0, 1.0, pods0=[{'tilt': 0.0}] * 2, **short)),
                       ('a tilt outside the range', lambda: F.run_leg(C17, 2.0, 1.0, **short,
                                                                    pods0=[{'tilt': 120.0, 'mode': 'released'}] * 2)),
                       ('unequal collective pods', lambda: F.run_leg(C17, 2.0, 1.0, **short, pods0=[
                           {'tilt': 0.0, 'mode': 'released'}, {'tilt': 10.0, 'mode': 'released'}])),
                       ('an integrator beyond the limit', lambda: F.run_leg(V27, 4.0, 1.0, integ0=0.5, **short)),
                       ('both flags', lambda: F.run_leg(V27, 4.0, 1.0, sat0=(True, True), **short)),
                       ('a motor command over 1', lambda: F.run_leg(V27, 4.0, 1.0, motors0=[1.2, 0.1], **short)),
                       ('tilt_range on the Cloudship', lambda: F.run_leg(C17, 2.0, 1.0, tilt_range=(-90, 90), **short)),
                       ('a negative wind', lambda: F.run_leg(V27, 4.0, 1.0, wind=(-1.0, 0.0), **short))):
        try:
            call()
            refused.append('%s: ran' % name)
        except ValueError:
            pass
    check('entry inputs that cannot be a recorded state are refused (9 cases)', not refused, '; '.join(refused))


def release_errors(a, b):
    """|b - a| of the release metrics; 0 where both are None (a mark past
    the end), inf where one is."""
    ma, mb = a['metrics'], b['metrics']
    err = lambda x, y: 0.0 if x is None and y is None else (float('inf') if x is None or y is None else abs(y - x))
    return {k: err(ma[k], mb[k]) for k in ('heading_release', 'r_rel_10', 'r_rel_60', 'r_mean1s_rel_10',
                                           'integ_rel_120', 'u_mean05_release_end')}


def test_release_start():
    """A leg started at another leg's release, from that leg's own state
    there (release_kwargs), flies the same release."""
    for target, u0 in ((V27, 4.0), (C17, 2.0)):
        a = F.run_leg(target, u0, 1.0, centred_s=1.0, stick_s=20.0, release_s=60.0)
        b = F.run_leg(**F.release_kwargs(a))
        err = release_errors(a, b)
        check('%s at %.1f m/s: started at the release from the leg\'s own state, the release is the leg\'s (heading '
              '%.1e of %.1f deg, r %.1e deg/s, integrator %.1e; reversals %d / %d)'
              % (target, u0, err['heading_release'], a['metrics']['heading_release'],
                 max(err['r_rel_10'], err['r_rel_60'], err['r_mean1s_rel_10']), err['integ_rel_120'],
                 a['metrics']['rev_release'], b['metrics']['rev_release']),
              err['heading_release'] < 1e-3 and max(err['r_rel_10'], err['r_rel_60'], err['r_mean1s_rel_10']) < 1e-5
              and err['integ_rel_120'] < 1e-6 and err['u_mean05_release_end'] < 1e-6
              and a['metrics']['rev_release'] == b['metrics']['rev_release']
              and b['metrics']['r_mean_last5'] is None and b['setup']['times'][3] == 0.0)


def record_of(leg):
    """A 0b record's leg, in forward_yaw_check's form (the fields
    replay_kwargs reads: FRD, deg and deg/s, the Acro switch at t 0), made
    from a model leg's own entry states."""
    import record_gains as rg
    s, e = leg['setup'], leg['entry']

    def state(x):
        return {'u': x['u'], 'v': x['v'], 'p_deg_s': x['p'], 'r_deg_s': x['r'], 'roll_deg': x['phi'],
                'torque_z': x['torque']}

    def acts(x):
        a = x['actuators']
        if s['vectored']:
            return {'motor1': a['motors'][0], 'motor2': a['motors'][1], 'tilt_stbd_deg': a['pods'][0]['tilt'],
                    'tilt_port_deg': a['pods'][1]['tilt']}
        return {'motor1': a['motors'][0], 'motor2': a['motors'][1], 'motor3': a['tail'],
                'tilt_deg': a['pods'][0]['tilt']}

    _, _, t_on, t_off, t_end = s['times']
    rp = {'state': state(e['acro']), 'state_at_release': state(e['release']), 'actuators': acts(e['acro']),
          'actuators_at_release': acts(e['release']), 'integrator_at_release': e['release']['integ'],
          'throttle': s['flown'], 'stick': [[0.0, 0.0], [t_on, s['stick']], [t_off, 0.0]], 'end_s': t_end,
          'params': {k: s['params'][k] for k in rg.GAINS}}
    return {'replay': rp, 'stick_phase': {'flag_pos_end': e['release']['sat_pos'],
                                          'flag_neg_end': e['release']['sat_neg']}}


def test_replay_kwargs():
    """replay_kwargs maps a 0b record's leg onto run_leg: a record written
    from a model leg replays that leg, from the Acro switch and from the
    release."""
    for target, u0 in ((V27, 4.0), (C17, 2.0)):
        a = F.run_leg(target, u0, 1.0, manual_s=2.0, centred_s=3.0, stick_s=20.0, release_s=40.0)
        rec = record_of(a)
        model = cl.ALL_TARGETS[target][0]
        acro = F.run_leg(**F.replay_kwargs(rec, model, 'acro'))
        ma, mr = a['metrics'], acro['metrics']
        check('%s: the record replayed from its Acro switch (manual_s 0, the recorded throttle exact, pods %s) is '
              'the leg (stick: r last 5 s %.4f / %.4f deg/s, heading %.3f / %.3f deg; release heading %.3f / %.3f)'
              % (target, [x['mode'] for x in F.replay_kwargs(rec, model, 'acro')['pods0']], ma['r_mean_last5'],
                 mr['r_mean_last5'], ma['heading_stick'], mr['heading_stick'], ma['heading_release'],
                 mr['heading_release']),
              abs(ma['r_mean_last5'] - mr['r_mean_last5']) < 1e-4 and abs(ma['heading_stick'] - mr['heading_stick']) < 1e-2
              and abs(ma['heading_release'] - mr['heading_release']) < 2e-2 and ma['rev_stick'] == mr['rev_stick'])
        rel = F.run_leg(**F.replay_kwargs(rec, model, 'release'))
        err = release_errors(a, rel)
        check('  ... and from its release (integrator, flags, pods and motors as recorded): heading %.1e deg, r %.1e '
              'deg/s, integrator %.1e off the leg' % (err['heading_release'], err['r_rel_10'], err['integ_rel_120']),
              err['heading_release'] < 1e-3 and err['r_rel_10'] < 1e-5 and err['integ_rel_120'] < 1e-6
              and a['metrics']['rev_release'] == rel['metrics']['rev_release'])
    rec['replay']['state']['v'] = None
    try:
        F.replay_kwargs(rec, model, 'acro')
        refused = False
    except LookupError:
        refused = True
    check('a record that leaves a value unread is refused, not guessed', refused)


def test_hook_and_policies():
    short = dict(centred_s=1.0, stick_s=8.0, release_s=10.0)
    base = F.run_leg(V27, 4.0, 1.0, **short)
    calls = []
    idle = F.run_leg(V27, 4.0, 1.0, hook=lambda t, plant, ctrl, alloc: calls.append(t), **short)
    steps = [b - a for a, b in zip(calls, calls[1:])]
    check('the hook runs once per cycle of the leg, before the controller, from t = -manual_s (%d calls, %d steps)'
          % (len(calls), len(base['trace']['t'])), len(calls) == len(base['trace']['t']) and calls[0] == -2.0
          and max(abs(s - F.DT) for s in steps) < 1e-9)
    check('a hook that sets nothing leaves the leg bit-identical', same_leg(base, idle))
    ff0 = F.run_leg(V27, 4.0, 1.0, params={'AS_YAWRATE_FF': 0.0}, **short)
    hooked = F.run_leg(V27, 4.0, 1.0, hook=lambda t, plant, ctrl, alloc: ctrl.set_feedforward(0.0), **short)
    check('set_feedforward(0) from the hook every cycle is no speed term, bit-identical to no hook; AS_YAWRATE_FF 0 '
          '(2527 flies 0.81) is another leg', same_leg(base, hooked) and base['metrics'] != ff0['metrics'])
    capmax = F.run_leg(V27, 4.0, 1.0, hook=lambda t, plant, ctrl, alloc: ctrl.set_rate_cap(15.0), **short)
    check('a cap at AS_YAWRATE_MAX in both places is bit-identical to no cap', same_leg(base, capmax))

    def sp_in_stick(yaw_stick, **cap):
        leg = F.run_leg(V27, 4.0, yaw_stick, hook=lambda t, plant, ctrl, alloc: ctrl.set_rate_cap(2.0, **cap), **short)
        return F.trace_at(leg, 'sp', 5.0), leg

    full, capped = sp_in_stick(1.0)
    half_stick, _ = sp_in_stick(0.5, clamp=False)
    half_clamp, _ = sp_in_stick(0.5, stick=False)
    check('the cap (2 deg/s) reaches the loop as the plan has it: full stick asks 2.0, half stick 0.889 through '
          'the stick\'s scale, 2.0 through the clamp alone (%.3f / %.3f / %.3f deg/s)' % (full, half_stick, half_clamp),
          abs(full - 2.0) < 1e-12 and abs(half_stick - 0.4 / 0.9 * 2.0) < 1e-9 and abs(half_clamp - 2.0) < 1e-12
          and capped['metrics']['integ_stick_end'] < base['metrics']['integ_stick_end'])
    seen = []
    windy = F.run_leg(V27, 4.0, 1.0, wind=(2.0, 180.0), **short,
                      hook=lambda t, plant, ctrl, alloc: seen.append((plant.u_air, plant.u_ground, plant.heading)))
    plain = []
    F.run_leg(V27, 4.0, 1.0, **short, hook=lambda t, plant, ctrl, alloc: plain.append((plant.u_air, plant.u_ground)))
    strip = lambda leg: {k: [{c: v for c, v in row.items() if c not in ('u_g', 'v_g')} for row in leg[k]]
                         if k == 'series' else leg[k] for k in ('metrics', 'series')}
    worst = max(abs(g - (u + 2.0 * math.cos(math.radians(180.0 - h)))) for u, g, h in seen)
    check('a steady wind (2 m/s, a headwind at the start) moves only the ground speed: the leg is the windless '
          'one, u_ground = u_air + 2 cos(180 deg - heading) on every cycle (%.1e), -2 m/s at the start; without '
          'wind u_ground is u_air' % worst,
          same(strip(windy), strip(base)) and same(windy['trace']['r'], base['trace']['r']) and worst < 1e-12
          and abs(seen[0][1] - (seen[0][0] - 2.0)) < 1e-12 and all(a == b for a, b in plain)
          and 'u_g' in windy['series'][0] and 'u_g' not in base['series'][0])
    restored = F.run_leg(V27, 4.0, 1.0, restore=True, **short)
    ev = restored['setup']['restore_events']
    t_rel = restored['setup']['times'][3]
    check('restore on a leg: one put-back at the release cycle (t %.3f s), of the integrator saved at the stick '
          'onset; the release turns %.1f deg against HEAD\'s %.1f' % (ev[0][0] if ev else -1,
                                                                    restored['metrics']['heading_release'],
                                                                    base['metrics']['heading_release']),
          len(ev) == 1 and abs(ev[0][0] - t_rel) < 1e-9 and abs(ev[0][1]) < 1e-3
          and restored['metrics']['heading_release'] < 0.5 * base['metrics']['heading_release'])
    same_range = F.run_leg(V27, 4.0, 1.0, tilt_range=(-180.0, 180.0), **short)
    check('tilt_range (-180, 180), the airframe\'s own, is bit-identical to the default', same_leg(base, same_range))
    def manual_yaw(**kw):
        """The yaw the allocator delivers (its own belief, normalized) with
        Manual passing a full stick through as torque 1.0 for 3 s."""
        yaw = []
        leg = F.run_leg(V27, 4.0, 1.0, stick_in_manual=True, manual_s=3.0, stick_s=4.0, release_s=1.0, **kw,
                        hook=lambda t, plant, ctrl, alloc: yaw.append((t, alloc.achieved_yaw)))
        return max(y for t, y in yaw if t < 0.0), leg

    a_half, half = manual_yaw(tilt_range=(-90.0, 90.0))
    a_full, _ = manual_yaw()
    tilts = list(half['trace']['tilt_s']) + list(half['trace']['tilt_p'])
    check('tilt_range (-90, 90): pods that cannot point backward deliver at most half the yaw (A = 0.5): torque '
          '1.0 through Manual gets %.4f, against %.4f with the airframe\'s +-180; the tilts stay inside +-90 deg; in '
          'Acro the shortfall sets the positive flag' % (a_half, a_full),
          abs(a_half - 0.5) < 1e-9 and a_full > 0.9 and max(abs(x) for x in tilts) <= 90.0 + 1e-6
          and half['entry']['release']['sat_pos'])


if __name__ == '__main__':
    test_throttle_mapping()
    test_vectored_allocation_with_thrust()
    test_thrust_zero_is_the_gate()
    test_cloudship_pods()
    test_trim()
    test_straight_flight()
    test_hold_and_determinism()
    test_linearity_refusals()
    test_schmitt()
    test_reversal_kinds()
    test_off_grid_segments()
    test_flown_throttle_reaches_the_pods()
    test_frd_and_throttle()
    test_marks_and_windows()
    test_command_at_and_reversals()
    test_fin_names_and_alphas()
    test_entry_state()
    test_release_start()
    test_replay_kwargs()
    test_hook_and_policies()
    print('\n%d failed' % len(FAILS) + (': ' + ', '.join(FAILS) if FAILS else ''))
    sys.exit(1 if FAILS else 0)
