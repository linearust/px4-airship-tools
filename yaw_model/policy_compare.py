#!/usr/bin/env python3
"""Step 0c of the D1 plan (d1_universal_architecture_20261001.md "4) 단계"
0c): the forward-flight policies compared on the yaw model.

    python3 policy_compare.py                 # run what is missing, then the report
    python3 policy_compare.py --report        # the report from the records on disk only
    python3 policy_compare.py --jobs 10       # worker processes (default 8)
    python3 policy_compare.py --only main     # one group of runs

Writes step0c_runs_20261001.jsonl (one record per run, appended as each run
ends, so an interrupted run resumes: a record whose id is on disk is not run
again; the hold slopes and the G1 runs are records there too),
step0c_runs_20261001.csv (the leg records flattened) and
step0c_out_20261001.txt (the report). The gate harness's own samples.csv go
to a temporary directory (STEP0C_SCRATCH to keep them).

The runs (policies.py has the policies, the selector and the schedule):
  main   per plant config (2527, 2522 at the airframe's +-180 deg tilt, A 1;
         the same limited to +-90, A 0.5; 2512; 2517 reported, not gated),
         per speed 2 / 2.5 / 4 / 6 m/s at trim as the 0b check flies it
         (glide on the drag-free 2522 and 2512), per stick (full 20 s, half
         20 s, full 300 s), then 120 s released: HEAD and RESTORE once, and
         each scheduled policy per speed source (12 on the primary plants,
         6 on the others) and per FF error (k x 0.7, 1.3, 2.0, the perfect
         source)
  khold  2512 with k the plant's measured steady-turn slope instead of the
         fins' damping (policies.fin_slope: k on the Cloudship's files is
         16x short of what its tail needs, hold_slopes below)
  noise  gz's gyro noise, three seeds, on 2527 and 2512 at 4 m/s
  long   the release run to 900 s on a few cases: the check of the "to rest"
         extrapolation (heading at 120 s + I(120 s) / K_I)
  dist   the restore rule's cost, which no 0c condition exercises: a yaw
         couple of DIST_SHARE of the authority (the gate's world-z couple)
         constant, gone at the stick's onset or arriving at it, on 2527 and
         2522 at 4 m/s (HEAD, RESTORE, SCHED, SCHED+RESTORE)
  rest   gate 2: every policy and source at u = 0 in still air against HEAD,
         trace for trace
  g1     gate 2: the 48 at-rest gate verdicts (reproduce_head) with each
         policy on, and each target's gate lines and samples against HEAD's
  hold   the plants' steady-turn torque per rad/s per m/s (hold_slope), the
         quantity the plan's in-flight k measurement reads, against k

Added after the 0c review (2026-10-02):
  held   the surge held at u0 (run_leg's hold_u) where the labelled speed is
         not what the hull flies: the drag-free gliders' (2522, 2512) 300 s
         holds, which glide down to about 2.4 / 1.5 m/s or tumble, and every
         leg of the A 0.5 twins, whose yaw adds surge (2522/A0.5 runs away).
         These are the gate's legs there; the free-surge legs stay as the
         record of what the hull does to its speed. Ids end in '|held'
  SCHED_CB  SCHED with the interval term on the FF the ramp supplies
         (policies.schedule budget 'supplied'), on every gated cell SCHED
         flies (its own speed sources and k errors)
  edge   the 2 m/s stall edge: 2527 and 2522, HEAD and SCHED, full stick at
         1.8..2.4 m/s, free and held
  tau    900 s holds against the asymptote the time constant predicts
  dqcheck  the SDP3x with its band at its documented zero error
         (policies.CHECK_SOURCES) against the 1.05 Pa case, leg for leg
  long   rerun with the flags after the 120 s mark recorded
  kick   the surge while a pod swings into the rear at the stick's onset
         (the first 3 s of the series kept)
  cbfree SCHED_CB* on the review's own example of its finding 1: 2522
         gliding (free surge) through a 300 s hold with the 1.05 Pa band
A run whose record lacks what the revised driver writes (a long release
without its late-flag fields, a G1 run of the older hook) is flown again
(stale()); the report is written by this file's current code from every
record on disk.

Units and signs (FRD): deg, deg/s positive nose right (the stick is always
positive here, so a positive heading change continues the turn and a
negative one turns back), m/s, normalized torque.
"""
import argparse
import bisect
import contextlib
import csv
import io
import json
import math
import multiprocessing as mp
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import closed_loop as cl  # noqa: E402
import controller as ctl  # noqa: E402
import forward as F  # noqa: E402
import policies as P  # noqa: E402

RUNS = os.path.join(HERE, 'step0c_runs_20261001.jsonl')
CSV_OUT = os.path.join(HERE, 'step0c_runs_20261001.csv')
OUT = os.path.join(HERE, 'step0c_out_20261001.txt')
SCRATCH = os.environ.get('STEP0C_SCRATCH')
DT = F.DT
SPEEDS = (2.0, 2.5, 4.0, 6.0)
CASES = ((1.0, 20.0), (0.5, 20.0), (1.0, 300.0))        # (stick, hold s)
RELEASE_S = 120.0
CENTRED_S = 3.0
K_ERRORS = (0.7, 1.3, 2.0)
SETTLE_RATE = 0.2               # deg/s, "time to |r| < 0.2 deg/s"
DRIFT_FLOOR = 1.0               # deg: a release this small is no drift (gate 1)
LAST_S = 30.0                   # the release's last window for the "to rest" checks
PRIMARY = ('2527', '2522')
SECONDARY_SOURCES = ('perfect', 'ground_W0', 'ground_W2_head', 'ground_W2_tail', 'ground_W2_x90', 'pitot_bias_sdp')
LONG_SOURCES = ('perfect', 'ground_W2_head', 'pitot_bias_sdp')      # the secondary plants' 300 s holds
# The restore rule's cost (not a 0c condition: the plan's wind moves only the
# ground speed, so the air-relative trim never changes and a restored trim is
# never stale): a yaw disturbance couple of DIST_SHARE of the authority, the
# gate's world-z couple (closed_loop W4), constant through the leg, gone at
# the stick's onset, or arriving at it; where it acts before the stick the
# integrator starts trimmed against it (integ0, the steady state of a long
# straight flight: the forward-flight integrator learns in (D + P) / K_I,
# minutes)
DIST_SHARE = 0.05
DIST_VARIANTS = ('dist_const', 'dist_gone', 'dist_arrives')
# the matrix's policies (the six of the first 0c run); SCHED_CB, the review's
# formula variant, is flown on the gated cells beside them (CB)
MATRIX_POLICIES = ('HEAD', 'RESTORE', 'SCHED', 'SCHED_STALL', 'SCHED+RESTORE', 'SCHED+RESTORE_FWD')
SCHED_POLICIES = tuple(p for p in MATRIX_POLICIES if p.startswith('SCHED'))
SPEC_SCHED = tuple(p for p in P.SPEC_POLICIES if p.startswith('SCHED'))
CB = 'SCHED_CB'
# Where the gate's legs fly the surge held at u0 (0c review findings 7, 8):
# the drag-free gliders glide down in a long turn (2522 to about 2.4 m/s from
# any entry, 2512 to about 1.5 m/s or into a tumble), and the A 0.5 twins'
# yaw adds forward thrust (a pod that cannot point backward is clipped at 0)
GLIDERS = ('2522', '2512', '2512/khold')
A05 = ('2527/A0.5', '2522/A0.5')
GATED = ('2527', '2522', '2527/A0.5', '2522/A0.5', '2512')
EDGE_SPEEDS = (1.8, 1.9, 2.0, 2.1, 2.2, 2.3, 2.4)
TAU_HOLD = 900.0
DEAD_SHARE = 0.05               # a stick that turned the hull under this share of HEAD's heading change is dead
SLOW_SHARE = 0.25               # a pass whose stick turned the hull under this share is counted as slow


def held_cell(cfg, hold):
    """True where the gate's legs fly the surge held at the label."""
    return cfg in A05 or (cfg in GLIDERS and hold >= 100.0)

# The plant configs by name (policies.plant_configs), plus 2512 with k from
# its steady-turn slope (the run spec carries the value)
CONFIG_SPECS = {
    '2527': dict(target='gz_airship_vectored_independent_am_fin_drag'),
    '2522': dict(target='gz_airship_vectored_independent_fin'),
    '2527/A0.5': dict(target='gz_airship_vectored_independent_am_fin_drag', tilt_range=(-90.0, 90.0)),
    '2522/A0.5': dict(target='gz_airship_vectored_independent_fin', tilt_range=(-90.0, 90.0)),
    '2512': dict(target='gz_cloudship_fin'),
    '2517': dict(target='gz_cloudship_am_fin_drag', gated=False, settles=False),
    '2512/khold': dict(target='gz_cloudship_fin'),
}
_CFG_CACHE = {}


def config(name, k_override=None):
    key = (name, k_override)

    if key not in _CFG_CACHE:
        spec = dict(CONFIG_SPECS[name])
        _CFG_CACHE[key] = P.PlantConfig(name, spec.pop('target'), k_override=k_override, **spec)

    return _CFG_CACHE[key]


# ---------------------------------------------------------------------------
# The plants' steady-turn slope: what a held turn needs per rad/s per m/s
# ---------------------------------------------------------------------------

HOLD_TORQUES = (0.1, 0.3)


def hold_slope(target, u0, torque, seconds=150.0):
    """The plant alone (Manual: the torque passes through, the loop open),
    the surge held at u0, a constant normalized torque held to the steady
    turn: (torque / r [per rad/s], r deg/s, the change of r over the last
    10 s deg/s, v m/s FRD). The quantity the plan's in-flight procedure
    measures (index.md:118-120, yawspeed_integ / rate, extended to two
    speeds), at the operating point. step0a.steady_k's procedure."""
    model, _ = cl.ALL_TARGETS[target]
    loop = cl.build(target, params=ctl.head_params(model), hold_u=u0)
    loop.plant.set_state(u=u0)
    loop.arm()
    loop.set_stick(torque)
    loop.set_mode('manual')
    loop.advance(seconds - 10.0)
    r1 = -loop.plant.nu[3]
    loop.advance(10.0)
    r2 = -loop.plant.nu[3]
    return torque / r2, math.degrees(r2), math.degrees(r2 - r1), -loop.plant.nu[1]


def _hold_job(args):
    target, u0, torque = args
    rec = {'id': 'hold|%s|%g|%g' % (target, u0, torque), 'group': 'hold', 'target': target, 'u0': u0,
           'torque': torque}

    try:
        k, r, dr, v = hold_slope(target, u0, torque)
        rec.update(per_rad_s=k, per_m_s=k / u0, r=r, dr10=dr, v=v)
    except Exception as exc:        # recorded (an unstable plant may not hold a turn)
        rec['error'] = '%s: %s' % (type(exc).__name__, exc)

    return rec


# ---------------------------------------------------------------------------
# The matrix
# ---------------------------------------------------------------------------


def run_id(spec):
    base = '|'.join(str(spec[k]) for k in ('group', 'config', 'policy', 'source', 'k_factor', 'u0', 'stick', 'hold',
                                         'release', 'seed'))
    base += ('|' + spec['variant'] if spec.get('variant') else '')
    return base + ('|held' if spec.get('held') else '')


def matrix_sources(cfg, hold):
    """The speed sources a plant's scheduled policies fly at a hold."""
    if cfg in PRIMARY:
        return list(P.SOURCES)
    return list(SECONDARY_SOURCES if hold < 100.0 else LONG_SOURCES)


def matrix(k_hold_2512):
    """Every run spec, in the order they are reported."""
    specs = []

    def add(group, cfg, policy, source, k_factor, u0, stick, hold, release=RELEASE_S, seed=0, k_override=None,
            variant=None, held=False):
        spec = dict(group=group, config=cfg, policy=policy, source=source, k_factor=k_factor, u0=u0, stick=stick,
                    hold=hold, release=release, seed=seed, k_override=k_override)

        if variant:
            spec['variant'] = variant

        if held:
            spec['held'] = True

        spec['id'] = run_id(spec)
        specs.append(spec)

    def cell(group, cfg, u0, stick, hold, held, cb_only=False):
        """One cell's runs: HEAD and RESTORE, the scheduled policies per
        source and k error (SCHED+RESTORE_FWD on the primary plants only),
        and SCHED_CB where the cell is gated."""
        primary = cfg in PRIMARY
        srcs = matrix_sources(cfg, hold)

        if not cb_only:
            for policy in ('HEAD', 'RESTORE'):
                add(group, cfg, policy, 'none', 1.0, u0, stick, hold, held=held)

            # SCHED+RESTORE_FWD is SCHED+RESTORE wherever w(u_lo) > 0 on the
            # release cycle: flown in full on the primary plants only (gate
            # 2's at-rest legs take it on every plant)
            for policy in (SCHED_POLICIES if primary else SPEC_SCHED):
                for src in srcs:
                    add(group, cfg, policy, src, 1.0, u0, stick, hold, held=held)

                for kf in K_ERRORS:
                    add(group, cfg, policy, 'perfect', kf, u0, stick, hold, held=held)

        if cfg in GATED:
            for src in srcs:
                add(group, cfg, CB, src, 1.0, u0, stick, hold, held=held)

            for kf in K_ERRORS:
                add(group, cfg, CB, 'perfect', kf, u0, stick, hold, held=held)

    for cfg in ('2527', '2522', '2527/A0.5', '2522/A0.5', '2512', '2517'):
        for u0 in SPEEDS:
            for stick, hold in CASES:
                held = held_cell(cfg, hold)
                # the free-surge legs of every cell (the first run's), with
                # SCHED_CB only where they are the gate's legs
                cell('main', cfg, u0, stick, hold, False, cb_only=False)

                if held:
                    cell('main', cfg, u0, stick, hold, True)

    # the free-surge twins above add SCHED_CB to non-held gated cells only:
    # drop the free SCHED_CB legs of the held cells (the gate flies them held)
    specs[:] = [s for s in specs if not (s['policy'] == CB and not s.get('held')
                                         and held_cell(s['config'], s['hold']))]

    for u0 in SPEEDS:
        for stick, hold in CASES:
            for held in ((False, True) if held_cell('2512/khold', hold) else (False,)):
                for policy in SPEC_SCHED:
                    for src in ('perfect', 'ground_W2_head'):
                        add('khold', '2512/khold', policy, src, 1.0, u0, stick, hold, k_override=k_hold_2512, held=held)

                    for kf in K_ERRORS:
                        add('khold', '2512/khold', policy, 'perfect', kf, u0, stick, hold, k_override=k_hold_2512,
                            held=held)

    for cfg in ('2527', '2512'):
        for policy in MATRIX_POLICIES:
            for seed in (1, 2, 3):
                add('noise', cfg, policy, 'perfect' if policy.startswith('SCHED') else 'none', 1.0, 4.0, 1.0, 20.0,
                    seed=seed)

    for cfg, policy, src, kf, u0, hold in LONG_CASES:
        add('long', cfg, policy, src, kf, u0, 1.0, hold, release=900.0)

    for cfg in PRIMARY:
        for variant in DIST_VARIANTS:
            for hold in (20.0, 300.0):
                for held in ((False, True) if held_cell(cfg, hold) else (False,)):
                    for policy in ('HEAD', 'RESTORE', 'SCHED', 'SCHED+RESTORE'):
                        add('dist', cfg, policy, 'perfect' if policy.startswith('SCHED') else 'none', 1.0, 4.0, 1.0,
                            hold, variant=variant, held=held)

    for cfg in ('2527', '2522', '2527/A0.5', '2522/A0.5', '2512', '2517'):
        for stick in (1.0, 0.5):
            add('rest', cfg, 'RESTORE', 'none', 1.0, 0.0, stick, 20.0, release=30.0)

            for policy in SCHED_POLICIES:
                for src in P.SOURCES:
                    add('rest', cfg, policy, src, 1.0, 0.0, stick, 20.0, release=30.0)

    # the stall edge (finding 12): a full stick at 1.8..2.4 m/s, free and held
    for cfg in PRIMARY:
        for policy, src in (('HEAD', 'none'), ('SCHED', 'perfect')):
            for u0 in EDGE_SPEEDS:
                for held in (False, True):
                    add('edge', cfg, policy, src, 1.0, u0, 1.0, 20.0, held=held)

            add('edge', cfg, policy, src, 1.0, 2.0, 1.0, 300.0, held=True)

    # the long holds against the time constant's asymptote (finding 9): 2527
    # holds its speed on its trim thrust; 2522 glides, so it is held
    for cfg, u0, held in (('2527', 4.0, False), ('2527', 6.0, False), ('2522', 6.0, True)):
        for kf in K_ERRORS:
            add('tau', cfg, 'SCHED', 'perfect', kf, u0, 1.0, TAU_HOLD, held=held)

    # the SDP3x with its band at its documented zero error (findings 3, 10)
    for cfg in PRIMARY:
        for u0 in SPEEDS:
            for stick, hold in ((1.0, 20.0), (1.0, 300.0)):
                add('dqcheck', cfg, 'SCHED', 'pitot_bias_sdp_dq', 1.0, u0, stick, hold)

    # the surge at the stick's onset, while a pod swings into the rear
    # (finding 11): the leg's series kept for its first 3 s
    for cfg, policy, src, u0 in KICK_CASES:
        add('kick', cfg, policy, src, 1.0, u0, 1.0, 20.0, release=10.0)

    # the review's own example of finding 1, free surge (the record, not the
    # gate's legs): 2522's 300 s pitot_bias_ms holds glide into the ramp, so
    # u_lo falls inside it; SCHED_CB* beside the free SCHED legs
    for u0 in SPEEDS:
        add('cbfree', '2522', CB, 'pitot_bias_ms', 1.0, u0, 1.0, 300.0)

    return specs


KICK_CASES = (('2522', 'SCHED', 'perfect', 2.0), ('2527', 'SCHED', 'perfect', 2.0), ('2522', 'HEAD', 'none', 2.0),
              ('2527', 'HEAD', 'none', 2.0), ('2522', 'SCHED', 'perfect', 4.0), ('2527', 'SCHED', 'perfect', 4.0))
KICK_S = 3.0


def kick_metrics(leg):
    """The surge while a pod swings into the rear at the stick's onset (vectored legs; 10 Hz series rows): the
    swing's start and end (the first row past 90 deg of tilt, the first at 180 within 0.5 deg), u at the onset and
    at the swing's end, the largest forward force of the two pods (N, sum of T cos(tilt)) during the swing, the
    force's mean over the second after it, and u at the stick's end."""
    t_stick, t_rel = leg['setup']['times'][2], leg['setup']['times'][3]
    rows = [r for r in leg['series'] if t_stick - 1e-9 <= r['t'] <= t_stick + KICK_S + 1e-9]
    fx = lambda r: r['T_s'] * math.cos(math.radians(r['tilt_s'])) + r['T_p'] * math.cos(math.radians(r['tilt_p']))
    tilt = lambda r: max(abs(r['tilt_s']), abs(r['tilt_p']))
    end = next((r for r in rows if tilt(r) >= 179.5), None)
    during = [r for r in rows if end is None or r['t'] <= end['t'] + 1e-9]
    after = [r for r in leg['series'] if end is not None and end['t'] < r['t'] <= end['t'] + 1.0 + 1e-9]
    return {'u_onset': leg['entry']['stick']['u'], 'swing_end_s': round(end['t'] - t_stick, 2) if end else None,
            'u_swing_end': end['u'] if end else None, 'fx_max_N': max(fx(r) for r in during) if during else None,
            'fx_after_N': sum(fx(r) for r in after) / len(after) if after else None,
            'u_stick_end': leg['entry']['release']['u'], 'trim': leg['setup']['trim'],
            'rows': [(round(r['t'] - t_stick, 2), round(r['u'], 4), round(fx(r), 3), round(r['tilt_s'], 1),
                      round(r['tilt_p'], 1)) for r in rows]}


LONG_CASES = (('2527', 'HEAD', 'none', 1.0, 4.0, 20.0),
              ('2522', 'HEAD', 'none', 1.0, 2.0, 20.0),
              ('2527', 'SCHED', 'perfect', 0.7, 4.0, 300.0),
              ('2527', 'SCHED', 'perfect', 1.3, 4.0, 300.0),
              ('2527', 'SCHED', 'ground_W2_head', 1.0, 4.0, 300.0),
              ('2527', 'RESTORE', 'none', 1.0, 4.0, 20.0),
              ('2512', 'HEAD', 'none', 1.0, 4.0, 20.0),
              ('2512', 'SCHED', 'perfect', 1.0, 4.0, 300.0))


# ---------------------------------------------------------------------------
# One run and its summary
# ---------------------------------------------------------------------------


def _fingerprint(state):
    """The plant and loop state a leg enters a segment with, for comparing
    runs of one base case (they must enter the stick identically)."""
    return [state[k] for k in ('u', 'v', 'r', 'phi', 'heading', 'integ', 'torque', 'r_meas')]


def summarize(leg, hook, spec, cfg):
    s, m, tr, e = leg['setup'], leg['metrics'], leg['trace'], leg['entry']
    _, _, t_stick, t_rel, t_end = s['times']
    T, R, I, TQ = tr['t'], tr['r'], tr['integ'], tr['torque']
    KI, lim = s['params']['AS_YAWRATE_I'], s['params']['AS_YR_INT_LIM']
    i_rel = bisect.bisect_right(T, t_rel + 1e-9)
    rel_R = R[i_rel:]
    rel_T = T[i_rel:]
    rel_H = tr['heading'][i_rel:]
    h0 = e['release']['heading']
    sp_end = F.trace_at(leg, 'sp', t_rel)
    at = hook.summary()['at'] if hook is not None else {}
    end_row = at.get('stick_end')
    out = {'trim': s['trim'], 'flown': s['flown'], 'times': s['times'], 'A': cfg.A, 'k': cfg.k * spec['k_factor'],
           'FF0': s['params']['AS_YAWRATE_FF'], 'MAX': s['params']['AS_YAWRATE_MAX'],
           'entry_stick': _fingerprint(e['stick']) if 'stick' in e else None,
           'r_last5': m['r_mean_last5'], 'r_peak': m['r_peak'], 'r_peak_smoothed': m['r_peak_smoothed'],
           'sp_end': sp_end, 'heading_stick': m['heading_stick'],
           'cap_end': end_row[5] if end_row else s['params']['AS_YAWRATE_MAX'],
           'ff_end': end_row[4] if end_row else s['params']['AS_YAWRATE_FF'],
           'u_lo_end': end_row[1] if end_row else None, 'u_hi_end': end_row[2] if end_row else None,
           'src_end': end_row[3] if end_row else None, 'binding_end': end_row[6] if end_row else None,
           'I_rel': m['integ_stick_end'], 'I_max': max(abs(x) for x in I), 'I_10': m['integ_rel_10'],
           'I_60': m['integ_rel_60'], 'I_end': m['integ_rel_120'],
           'sat_end': [m['sat_pos_stick_end'], m['sat_neg_stick_end']], 'unalloc_end': m['unalloc_stick_end'],
           'torque_end': m['torque_stick_end'],
           'drift_120': m['heading_release'], 'r_10': m['r_rel_10'], 'r_60': m['r_rel_60'], 'r_120': m['r_rel_120'],
           'rel_r_max': max(rel_R) if len(rel_R) else None, 'rel_r_min': min(rel_R) if len(rel_R) else None,
           'rel_h_max': (max(rel_H) - h0) if len(rel_H) else None, 'rel_h_min': (min(rel_H) - h0) if len(rel_H) else None,
           'u_stick_start': m['u_stick_start'], 'u_stick_end': m['u_stick_end'], 'u_release_end': m['u_release_end'],
           'fin_stick': max(m['fin_max_alpha_ratio_stick'].values()) if m['fin_max_alpha_ratio_stick'] else None,
           'fin_release': max(m['fin_max_alpha_ratio_release'].values()) if m['fin_max_alpha_ratio_release'] else None,
           'stalled_stick': m['stalled_fraction_stick'], 'stalled_release': m['stalled_fraction_release'],
           'rev_stick': m['rev_stick'], 'rev_release': m['rev_release'],
           'restore_events': len(s['restore_events']),
           'restore_value': s['restore_events'][0][1] if s['restore_events'] else None}

    if s['vectored']:
        out['swings'] = {'stick': m['pod_swings_stick'], 'release': m['pod_swings_release']}
    else:
        out['flips'] = {'stick': m['tail_flips_stick'], 'release': m['tail_flips_release'],
                        'thrust_signs_stick': m['tail_thrust_signs_stick'],
                        'thrust_signs_release': m['tail_thrust_signs_release']}

    # the torque the release starts with against the stick's last: the restore
    # rule's put-back shows here (the row at t holds the cycle at t - DT)
    a, b = F.trace_at(leg, 'torque', t_rel), F.trace_at(leg, 'torque', round(t_rel + DT, 6))
    out['torque_step'] = (b - a) if (a is not None and b is not None) else None
    # flags over the stick and the release's last LAST_S (10 Hz rows)
    rows = leg['series']
    stick_rows = [r for r in rows if t_stick < r['t'] <= t_rel + 1e-9]
    out['sat_frac_stick'] = (sum(1 for r in stick_rows if r['sat_pos'] or r['sat_neg']) / len(stick_rows)
                             if stick_rows else None)
    last = [r for r in rows if r['t'] > t_end - LAST_S + 1e-9]
    out['sat_last'] = any(r['sat_pos'] or r['sat_neg'] for r in last)
    r_before = F.trace_at(leg, 'r', round(t_end - LAST_S, 6))
    out['r_last_max'] = max(abs(r['r']) for r in last) if last else None
    # time to |r| < SETTLE_RATE for good (inside the window), or None
    late = [t for t, r in zip(rel_T, rel_R) if abs(r) >= SETTLE_RATE]
    if not late:
        out['t_settle'] = 0.0
    elif late[-1] < t_end - 1e-9:
        out['t_settle'] = round(late[-1] + DT - t_rel, 3)
    else:
        out['t_settle'] = None

    # the rest of the turn: every remaining degree is I / K_I as long as the
    # integrator runs unclipped (no flag, inside its limit) and the loop
    # settles: sum(e dt) = -sum(r dt) = (I_inf - I) / K_I, I_inf = 0 in
    # still air (the "long" group checks it against 900 s releases)
    why = []
    if not cfg.settles:
        why.append('the plant does not settle in forward flight')
    if out['sat_last']:
        why.append('a saturation flag in the last %g s' % LAST_S)
    if abs(out['I_end']) >= lim - 1e-6:
        why.append('the integrator at its limit')
    if r_before is not None and abs(m['r_rel_120']) > abs(r_before) + 0.05:
        why.append('|r| grows over the last %g s' % LAST_S)
    out['rest_valid'] = not why
    out['rest_why'] = '; '.join(why)
    out['drift_rest'] = out['drift_120'] + math.degrees(out['I_end'] / KI) if not why else None
    # an estimate of the settling time past the window, quasi-static: r = I /
    # (D + P), I decaying with (D + P) / K_I
    out['t_settle_est'] = None
    r_end, i_end = m['r_rel_120'], out['I_end']
    if out['t_settle'] is None and not why and r_end * i_end > 0 and abs(r_end) > SETTLE_RATE:
        tau = (i_end / math.radians(r_end)) / KI
        out['t_settle_est'] = round((t_end - t_rel) + tau * math.log(abs(r_end) / SETTLE_RATE), 1)

    if hook is not None:
        summ = hook.summary()
        out['hook'] = {k: summ[k] for k in ('cap_min_stick', 'cap_max_stick', 'binding_stick', 'sources_seen',
                                            'rejects', 'switches', 'gated_edges', 'ramp_stick')}
        out['hook']['at'] = {name: list(row) for name, row in summ['at'].items()}

    # added after the 0c review: the surge the leg flew (held or free; the
    # mean over the stick), and the hull's largest rate in the release
    out['held'] = s['hold_u'] is not None
    stick_u = [tr['u'][i] for i in range(len(T)) if t_stick < T[i] <= t_rel + 1e-9]
    out['u_mean_stick'] = sum(stick_u) / len(stick_u) if stick_u else None
    out['rel_r_absmax'] = max(abs(x) for x in rel_R) if len(rel_R) else None
    return out


def _leg(spec):
    cfg = config(spec['config'], spec.get('k_override'))
    source = P.source(spec['source'])

    if spec['group'] == 'rest' and source is not None:
        source = source.calm()

    extra = {'hold_u': spec['u0']} if spec.get('held') else {}
    kw, hook = P.leg_kwargs(spec['policy'], cfg, source, spec['u0'], spec['stick'], k_factor=spec['k_factor'],
                            centred_s=CENTRED_S, stick_s=spec['hold'], release_s=spec['release'],
                            noise=spec['seed'] > 0, seed=max(1, spec['seed']), **extra)
    variant = spec.get('variant')

    if not variant:
        return F.run_leg(**kw), hook, cfg

    # the disturbance: set on the loop run_leg builds (captured from
    # closed_loop.build), every cycle, by the leg's time
    d_nm = DIST_SHARE * P.gz_model.yaw_authority(cfg.model)        # FRD, nose right: aids the turn
    # from the Acro switch (in Manual the loop is open and nothing would
    # cancel it), where the trimmed integrator meets it exactly
    on = {'dist_const': lambda t: t >= -1e-9, 'dist_gone': lambda t: -1e-9 <= t < CENTRED_S - 1e-9,
          'dist_arrives': lambda t: t >= CENTRED_S - 1e-9}[variant]
    loops = []
    policy_hook = kw['hook']

    def hook2(t, plant, ctrl, alloc):
        loops[0].set_wind(d_nm if on(t) else 0.0)

        if policy_hook is not None:
            policy_hook(t, plant, ctrl, alloc)

    kw['hook'] = hook2

    if variant in ('dist_const', 'dist_gone'):
        kw['integ0'] = -DIST_SHARE                  # trimmed: the integrator cancels the couple

    orig = cl.build

    def build(*a, **k):
        loop = orig(*a, **k)
        loops.append(loop)
        loop.set_wind(d_nm if on(-kw.get('manual_s', 2.0)) else 0.0)
        return loop

    cl.build = build

    try:
        leg = F.run_leg(**kw)
    finally:
        cl.build = orig

    leg['setup']['dist'] = {'variant': variant, 'N_m': d_nm, 'I_inf': -DIST_SHARE if variant != 'dist_gone' else 0.0}
    return leg, hook, cfg


_HEAD_REST = {}


def _rest_compare(spec):
    """Gate 2's leg at u = 0 in still air against HEAD's, everything the leg
    computed (forward.run_leg's metrics, entry states, series and trace)."""
    key = (spec['config'], spec['stick'])

    if key not in _HEAD_REST:
        head = dict(spec, policy='HEAD', source='none')
        _HEAD_REST[key] = _leg(head)[0]

    base = _HEAD_REST[key]
    leg, hook, cfg = _leg(spec)
    same = _same(base['metrics'], leg['metrics']) and _same(base['entry'], leg['entry']) and \
        _same(base['series'], leg['series']) and _same(base['trace'], leg['trace'])
    out = {'identical': same, 'A': cfg.A}

    if not same:
        diffs = {}

        for key2 in ('r', 'torque', 'integ', 'heading', 'sp'):
            a, b = base['trace'][key2], leg['trace'][key2]
            d = [abs(x - y) for x, y in zip(a, b)]
            first = next((t for t, x in zip(base['trace']['t'], d) if x != 0.0), None)
            diffs[key2] = {'max': max(d), 'first_t': first}

        out['diffs'] = diffs
        out['heading_stick'] = (base['metrics']['heading_stick'], leg['metrics']['heading_stick'])
        out['r_last5'] = (base['metrics']['r_mean_last5'], leg['metrics']['r_mean_last5'])

    if hook is not None:
        summ = hook.summary()
        out['hook'] = {'cap_min_stick': summ['cap_min_stick'], 'cap_max_stick': summ['cap_max_stick'],
                       'at': {k: list(v) for k, v in summ['at'].items()}}

    out['restore_events'] = len(leg['setup']['restore_events'])
    return out


def _same(a, b):
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(_same(a[k], b[k]) for k in a)
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(_same(x, y) for x, y in zip(a, b))
    if hasattr(a, 'tolist') and hasattr(b, 'tolist'):
        return a.tolist() == b.tolist()
    return a == b or (a != a and b != b)


def run_one(spec):
    """One run spec -> its record (spec + 'result' or 'error')."""
    t0 = time.time()
    rec = dict(spec)

    try:
        if spec['group'] == 'rest':
            rec['result'] = _rest_compare(spec)
        else:
            leg, hook, cfg = _leg(spec)
            rec['result'] = summarize(leg, hook, spec, cfg)

            if 'dist' in leg['setup']:
                # the rest of the turn with the couple still on (or gone):
                # the integrator ends at I_inf, not 0
                r, d = rec['result'], leg['setup']['dist']
                ki = leg['setup']['params']['AS_YAWRATE_I']
                r['dist'] = d
                r['drift_rest'] = (r['drift_120'] + math.degrees((r['I_end'] - d['I_inf']) / ki)
                                   if r['drift_rest'] is not None else None)

            if spec['group'] == 'kick':
                rec['result']['kick'] = kick_metrics(leg)

            if spec['group'] == 'long':
                s, tr = leg['setup'], leg['trace']
                t_rel = s['times'][3]
                KI = s['params']['AS_YAWRATE_I']
                mark = round(t_rel + RELEASE_S, 6)
                h120 = F.trace_at(leg, 'heading', mark) - leg['entry']['release']['heading']
                i120 = F.trace_at(leg, 'integ', mark)
                # the flags after the mark (10 Hz rows): one clips the
                # integrator, and the extrapolation from the mark misses
                # whatever the hull turns while it is held
                late = [r for r in leg['series'] if r['t'] > mark + 1e-9 and (r['sat_pos'] or r['sat_neg'])]
                rec['result']['long'] = {'heading_120': h120, 'I_120': i120,
                                         'predicted_rest': h120 + math.degrees(i120 / KI),
                                         'heading_end': rec['result']['drift_120'],
                                         'I_end': rec['result']['I_end'],
                                         'flown_rest': rec['result']['drift_120'] + math.degrees(
                                             rec['result']['I_end'] / KI),
                                         'flag_rows_after_120': len(late),
                                         'flags_after_120': [round(late[0]['t'] - t_rel, 1),
                                                             round(late[-1]['t'] - t_rel, 1)] if late else None,
                                         'I_flagged': late[0]['integ'] if late else None,
                                         'swings_release': rec['result'].get('swings', {}).get('release')}
    except Exception as exc:        # recorded, reported; the batch goes on
        import traceback
        rec['error'] = '%s: %s' % (type(exc).__name__, exc)
        rec['traceback'] = traceback.format_exc()[-2000:]

    rec['wall_s'] = round(time.time() - t0, 2)
    return rec


# ---------------------------------------------------------------------------
# G1 with each policy on: reproduce_head through the gate, a hook installed
# ---------------------------------------------------------------------------

G1_CONFIGS = (('HEAD', None), ('SCHED', 'ground_W0'), ('SCHED_STALL', 'ground_W0'), ('SCHED+RESTORE', 'ground_W0'),
              ('SCHED+RESTORE_FWD', 'ground_W0'), ('RESTORE', None), ('SCHED', 'ground_W2_head'),
              ('SCHED_STALL', 'ground_W2_head'))


def g1_one(args):
    try:
        return _g1_one(args)
    except Exception as exc:
        import traceback
        return {'id': 'g1|%s|%s' % args, 'group': 'g1', 'policy': args[0], 'source': args[1],
                'error': '%s: %s' % (type(exc).__name__, exc), 'traceback': traceback.format_exc()[-2000:]}


def _g1_one(args):
    """reproduce_head.reproduce() with `policy` on every gate loop it builds:
    the hook called before each controller cycle with the gate's plant (no
    wind in the speed sense: the gate's 'wind' is a world-z couple), the
    controller's own gains for FF0, MAX and INT_LIM, the source calm.
    Returns (agree, total, per target: gate lines, samples' r/torque/integ)."""
    policy, src = args
    import gate
    import reproduce_head as rh
    orig_build, orig_run = cl.build, gate.run_gate
    results = {}
    if SCRATCH:
        out_dir = os.path.join(SCRATCH, 'g1_%s_%s' % (policy.replace('+', 'p'), src or 'none'))
        os.makedirs(out_dir, exist_ok=True)
    else:
        import tempfile
        out_dir = tempfile.mkdtemp(prefix='step0c_g1_')

    def build(target, *a, **kw):
        loop = orig_build(target, *a, **kw)

        if policy == 'HEAD':
            return loop

        yc = loop.controller.ctrl

        if 'RESTORE' in policy:
            yc.set_restore(P.RESTORE_EDGE)

        if policy.startswith('SCHED'):
            cfg = P.PlantConfig(target, target)
            g = yc.params
            source = P.SOURCES[src].calm()
            c = cfg.stall['c'] if (policy == 'SCHED_STALL' and cfg.stall) else None
            hook = P.Scheduled(source, cfg.k, g['AS_YAWRATE_FF'], g['AS_YAWRATE_MAX'], g['AS_YR_INT_LIM'], cfg.A_dir,
                               c_stall=c, t_stick=1e9, forward_gate=policy.endswith('_FWD'),
                               budget=P.BUDGETS.get(policy, 'plan'))
            view = F.PlantView(loop.plant, None)
            step = yc.step

            def stepped(t_us, *args2, **kw2):
                hook(t_us * 1e-6, view, yc, None)
                return step(t_us, *args2, **kw2)

            yc.step = stepped
            loop._policy_hook = hook

        return loop

    def run_gate(loop, model_name, *a, **kw):
        kw['out_dir'] = out_dir
        res = orig_run(loop, model_name, *a, **kw)
        hook = getattr(loop, '_policy_hook', None)
        results[model_name] = {'status': res.status, 'gate_lines': res.gate_lines,
                               'samples': [(r.get('sim'), r.get('rate'), r.get('torque'), r.get('integ'))
                                           for r in res.samples],
                               'caps': (min(x[5] for x in hook.samples), max(x[5] for x in hook.samples))
                               if hook is not None and hook.samples else None}
        return res

    cl.build, gate.run_gate = build, run_gate
    buf = io.StringIO()

    try:
        with contextlib.redirect_stdout(buf):
            agree, total, table = rh.reproduce()
    finally:
        cl.build, gate.run_gate = orig_build, orig_run

    return {'id': 'g1|%s|%s' % (policy, src), 'group': 'g1', 'policy': policy, 'source': src, 'agree': agree,
            'total': total, 'table': [list(row) for row in table], 'targets': results, 'hook_version': HOOK_VERSION}


# the hook as revised after the 0c review (per-direction A, the budget
# option): G1 records of an older hook are flown again and compared
HOOK_VERSION = 2


def stale(rec):
    """A record on disk that the revised driver flies again: an error, a
    long release without its late-flag fields, a G1 run of the older hook."""
    if 'error' in rec:
        return True

    if rec.get('group') == 'long':
        return 'flag_rows_after_120' not in rec.get('result', {}).get('long', {})

    if rec.get('group') == 'g1':
        return rec.get('hook_version') != HOOK_VERSION

    return False


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------


def load_records(path=RUNS):
    out = {}

    if os.path.exists(path):
        with open(path, encoding='utf-8') as handle:
            for line in handle:
                line = line.strip()

                if line:
                    rec = json.loads(line)
                    out[rec['id']] = rec

    return out


def _append(records, handle):
    for rec in records:
        handle.write(json.dumps(rec) + '\n')
    handle.flush()


def k_hold_of(records, target='gz_cloudship_fin'):
    """The mean steady-turn slope of `target` over the hold records
    (normalized torque per rad/s per m/s), or None."""
    rows = [r for r in records.values() if r.get('group') == 'hold' and r['target'] == target and 'error' not in r]
    return sum(r['per_m_s'] for r in rows) / len(rows) if rows else None


def main():
    ap = argparse.ArgumentParser(description='step 0c: the policy comparison')
    ap.add_argument('--jobs', type=int, default=8)
    ap.add_argument('--report', action='store_true', help='the report from the records on disk only')
    ap.add_argument('--only', default=None, help='one group: hold, g1, main, khold, noise, long, rest, dist, edge, '
                                                 'tau, dqcheck, kick, cbfree')
    ap.add_argument('--limit', type=int, default=None, help='at most this many new leg runs (a smoke test)')
    args = ap.parse_args()

    if not args.report:
        done = load_records()

        with open(RUNS, 'a', encoding='utf-8') as handle:
            if args.only in (None, 'hold'):
                jobs = [(t, u, q) for t in ('gz_airship_vectored_independent_fin',
                                            'gz_airship_vectored_independent_am_fin_drag', 'gz_cloudship_fin',
                                            'gz_cloudship_am_fin_drag')
                        for u in SPEEDS for q in HOLD_TORQUES]
                jobs = [j for j in jobs if 'hold|%s|%g|%g' % j not in done]

                if jobs:
                    with mp.Pool(args.jobs) as pool:
                        _append(pool.map(_hold_job, jobs), handle)

            if args.only in (None, 'g1'):
                jobs = [c for c in G1_CONFIGS if 'g1|%s|%s' % c not in done or stale(done['g1|%s|%s' % c])]

                if jobs:
                    with mp.Pool(min(args.jobs, len(jobs))) as pool:
                        _append(pool.map(g1_one, jobs), handle)

        done = load_records()

        if args.only not in ('hold', 'g1'):
            k_hold = k_hold_of(done)

            if k_hold is None:
                raise SystemExit('the 2512 steady-turn slope is not measured yet: run --only hold first')

            specs = [s for s in matrix(k_hold) if args.only in (None, s['group'])]
            todo = [s for s in specs if s['id'] not in done or stale(done[s['id']])]
            # the longest first, so that the pool's tail is short
            todo.sort(key=lambda s: -(s['hold'] + s['release']))

            if args.limit is not None:
                todo = todo[:args.limit]

            print('%d runs in the matrix, %d records on disk, %d to run with %d workers'
                  % (len(specs), len(done), len(todo), args.jobs), flush=True)
            t0 = time.time()

            with mp.Pool(args.jobs) as pool, open(RUNS, 'a', encoding='utf-8') as handle:
                for n, rec in enumerate(pool.imap_unordered(run_one, todo), 1):
                    _append([rec], handle)

                    if n % 25 == 0 or n == len(todo) or 'error' in rec:
                        print('%d / %d  %.0f s  %s%s' % (n, len(todo), time.time() - t0, rec['id'],
                                                         ('  ERROR ' + rec['error']) if 'error' in rec else ''),
                              flush=True)

    write_report(load_records())


# ---------------------------------------------------------------------------
# The report
# ---------------------------------------------------------------------------

CASE_NAMES = {(1.0, 20.0): 'full 20 s', (0.5, 20.0): 'half 20 s', (1.0, 300.0): 'full 300 s'}
SHORT = {'HEAD': 'HEAD', 'RESTORE': 'RESTORE', 'SCHED': 'SCHED', 'SCHED_STALL': 'SCHED_STALL',
         'SCHED+RESTORE': 'SCHED+RESTORE', 'SCHED+RESTORE_FWD': '+RESTORE_FWD*', 'SCHED_CB': 'SCHED_CB*'}
CONFIG_TEXT = {
    '2527': '2527 airship_vectored_independent_am_fin_drag, tilt +-180 deg (A 1)',
    '2522': '2522 airship_vectored_independent_fin, tilt +-180 deg (A 1), gliding (no drag block: trim 0)',
    '2527/A0.5': '2527 with CA_AIRSHIP_TLMIN/TLMAX -90/90 deg (pods cannot point backward: A 0.5)',
    '2522/A0.5': '2522 with CA_AIRSHIP_TLMIN/TLMAX -90/90 deg (A 0.5), gliding',
    '2512': '2512 cloudship_fin, the reversible tail (A 1), gliding',
    '2517': '2517 cloudship_am_fin_drag (A 1): reported, not gated (directionally unstable in forward flight)',
    '2512/khold': '2512 with k its measured steady-turn slope (not the fins\' damping)',
}
FWD = (2.5, 4.0, 6.0)          # the schedule's band: w = 1 at the entry speed
A1 = ('2527', '2522')
RESTORERS = ('RESTORE', 'SCHED+RESTORE', 'SCHED+RESTORE_FWD')
VERDICTS = ('smaller', 'floor', 'dead', 'not')


def fnum(x, spec='%+.1f'):
    return '-' if x is None else spec % x


LEG_GROUPS = ('main', 'khold', 'noise', 'long', 'rest', 'dist', 'edge', 'tau', 'dqcheck', 'kick', 'cbfree')


class Book:
    """The records indexed for the report. get() reads one leg; gate()
    reads the leg the gate uses for a cell: held where held_cell() says the
    labelled speed needs the surge held, else free."""

    def __init__(self, records):
        self.all = records
        self.legs = {}
        self.errors = []

        for rec in records.values():
            if 'error' in rec:
                self.errors.append(rec)
                continue

            if rec.get('group') in LEG_GROUPS:
                key = (rec['group'], rec['config'], rec['policy'], rec['source'], rec['k_factor'], rec['u0'],
                       rec['stick'], rec['hold'], rec['release'], rec['seed'], rec.get('variant') or '',
                       bool(rec.get('held')))
                self.legs[key] = rec

        self.hold = [r for r in records.values() if r.get('group') == 'hold']
        self.g1 = {(r['policy'], r['source']): r for r in records.values() if r.get('group') == 'g1'}

    def get(self, cfg, policy, source='none', kf=1.0, u0=4.0, stick=1.0, hold=20.0, group='main', release=RELEASE_S,
            seed=0, variant='', held=False):
        if policy in ('HEAD', 'RESTORE'):
            source = 'none'
        rec = self.legs.get((group, cfg, policy, source, kf, u0, stick, hold, release, seed, variant, bool(held)))
        return rec['result'] if rec else None

    def gate(self, cfg, policy, source='none', kf=1.0, u0=4.0, stick=1.0, hold=20.0, group=None, **kw):
        """The gate's leg of a cell (the khold config reads its own group)."""
        group = group or ('khold' if cfg == '2512/khold' else 'main')
        return self.get(cfg, policy, source, kf, u0, stick, hold, group=group, held=held_cell(cfg, hold), **kw)


def case_name(stick, hold):
    return CASE_NAMES.get((stick, hold)) or '%s %g s' % ('full' if stick == 1.0 else 'half', hold)


def w_of(u):
    """The ramp's weight w(u) at the values 0c flies."""
    return P.interpolate(u, P.AS_SPD_LO, P.AS_SPD_HI, 0.0, 1.0)


END_CLASSES = ('w_lo=1', 'lo_in_ramp', 'lo_below', 'w_hi<1')


def end_class(r):
    """Where the selector's interval sat at the stick's end, or None (no schedule): 'w_lo=1' u_lo past the ramp
    (the plan's term holds the budget); with u_hi past it, 'lo_in_ramp' u_lo inside the ramp (0 < w(u_lo) < 1:
    the plan's term under-budgets, the supplied-FF term holds) or 'lo_below' u_lo at or below AS_SPD_LO
    (w(u_lo) = 0: the two terms are the same); 'w_hi<1' u_hi inside the ramp (the cap blended toward MAX)."""
    lo, hi = r.get('u_lo_end'), r.get('u_hi_end')

    if lo is None or hi is None:
        return None

    if w_of(lo) >= 1.0:
        return 'w_lo=1'

    if w_of(hi) < 1.0:
        return 'w_hi<1'

    return 'lo_in_ramp' if w_of(lo) > 0.0 else 'lo_below'


def drift_cell(r, width=0):
    if r is None:
        return 'n/r'
    rest = fnum(r['drift_rest'], '%+.0f') if r['rest_valid'] else '?'
    return '%s/%s' % (fnum(r['drift_120'], '%+.0f'), rest)


def stick_cell(r):
    if r is None:
        return 'n/r'
    return '%s/%s %s' % (fnum(r['r_last5'], '%.2f'), fnum(r['sp_end'], '%.2f'), fnum(r['I_rel'], '%+.3f'))


def rev_cell(r):
    if r is None:
        return 'n/r'
    fin = fnum(r['fin_stick'], '%.2f')
    stalled = r['stalled_stick'] or 0.0
    return '%s/%s %s%s' % (r['rev_stick'], r['rev_release'], fin, ' S%.0f%%' % (100 * stalled) if stalled else '')


def excursion(r):
    """The release's largest heading excursion from the release heading, deg (signed)."""
    lo, hi = r.get('rel_h_min'), r.get('rel_h_max')

    if lo is None or hi is None:
        return None

    return hi if abs(hi) >= abs(lo) else lo


def table(lines, head, rows):
    lines.append('| ' + ' | '.join(head) + ' |')
    lines.append('|' + '---|' * len(head))

    for row in rows:
        lines.append('| ' + ' | '.join(str(x) for x in row) + ' |')


def say_lines(say):
    """table() appends to a list; the report's say() is that list's append."""
    class _L:
        def append(self, text):
            say(text)
    return _L()


def _where(cfg, u0, stick, hold, src=None, kf=None, held=False):
    parts = [cfg, '%g m/s' % u0, case_name(stick, hold)]
    if src and src not in ('none', 'perfect'):
        parts.append(src)
    if kf not in (None, 1.0):
        parts.append('k x%g' % kf)
    if held:
        parts.append('held')
    return ' '.join(parts)


def _rows(book, cfgs, policies, sources=('perfect',), kfs=(1.0,), speeds=SPEEDS, cases=CASES, group='main',
          which='gate'):
    """[(cfg, policy, source, kf, u0, stick, hold, result)] that exist. which: 'gate' the gate's legs (held where
    held_cell says so), 'free' the free-surge legs, 'held' the held ones."""
    out = []

    for cfg in cfgs:
        grp = 'khold' if cfg == '2512/khold' else group

        for policy in policies:
            for src in (('none',) if policy in ('HEAD', 'RESTORE') else sources):
                for kf in ((1.0,) if policy in ('HEAD', 'RESTORE') else kfs):
                    for u0 in speeds:
                        for stick, hold in cases:
                            if which == 'gate':
                                r = book.gate(cfg, policy, src, kf, u0, stick, hold, group=grp)
                            else:
                                r = book.get(cfg, policy, src, kf, u0, stick, hold, group=grp, held=which == 'held')
                            if r is not None:
                                out.append((cfg, policy, src, kf, u0, stick, hold, r))

    return out


def _extreme(rows, key, valid_only=False):
    """(value, where) of the largest |result[key]| over rows, or (None, '')."""
    rows = [x for x in rows if x[7].get(key) is not None and (not valid_only or x[7]['rest_valid'])]
    if not rows:
        return None, ''
    x = max(rows, key=lambda x: abs(x[7][key]))
    return x[7][key], _where(x[0], x[4], x[5], x[6], x[2], x[3], x[7].get('held'))


def _extreme_of(rows, fn):
    """(value, where) of the largest |fn(result)| over rows, or (None, '')."""
    rows = [x for x in rows if fn(x[7]) is not None]
    if not rows:
        return None, ''
    x = max(rows, key=lambda x: abs(fn(x[7])))
    return fn(x[7]), _where(x[0], x[4], x[5], x[6], x[2], x[3], x[7].get('held'))


def _span(rows, key):
    vals = [x[7][key] for x in rows if x[7].get(key) is not None]
    return (min(vals), max(vals)) if vals else (None, None)


def _putback(rows):
    """(largest |put-back - I at release|, where) over rows with a restore."""
    rs = [x for x in rows if x[7]['restore_value'] is not None]
    if not rs:
        return None, ''
    x = max(rs, key=lambda x: abs(x[7]['restore_value'] - x[7]['I_rel']))
    return x[7]['restore_value'] - x[7]['I_rel'], _where(x[0], x[4], x[5], x[6], x[2], x[3], x[7].get('held'))


def plant_pols(cfg):
    pols = list(MATRIX_POLICIES) if cfg in PRIMARY else list(P.SPEC_POLICIES)
    return pols + ([CB] if cfg in GATED else [])


def sched_pols(cfg):
    return (SCHED_POLICIES if cfg in PRIMARY else SPEC_SCHED) + ((CB,) if cfg in GATED else ())


def sources_for(cfg, hold):
    if cfg == '2512/khold':
        return ['perfect', 'ground_W2_head']
    return matrix_sources(cfg, hold)


# --- the plants' hold slope and time constant --------------------------------

def hold_points(book, model):
    """[(u0, steady-turn torque per rad/s)] of a model at the larger hold torque, by speed."""
    return sorted((h['u0'], h['per_rad_s']) for h in book.hold
                  if 'error' not in h and cl.ALL_TARGETS[h['target']][0] == model and h['torque'] == HOLD_TORQUES[-1])


def d_at(points, u):
    """The steady-turn torque per rad/s at u: linear between the measured speeds, per m/s of the nearest end
    outside them; None without points."""
    if not points:
        return None
    if u <= points[0][0]:
        return points[0][1] * u / points[0][0]
    if u >= points[-1][0]:
        return points[-1][1] * u / points[-1][0]
    for (u0, d0), (u1, d1) in zip(points, points[1:]):
        if u0 <= u <= u1:
            return d0 + (d1 - d0) * (u - u0) / (u1 - u0)
    return None


def tau_of(book, cfg, u):
    """The integrator's time constant (D + P) / K_I at u (s): quasi-static, r = (FF sp + P (sp - r) + I) / D
    in a held turn, so I relaxes toward (D - FF) sp with (D + P) / K_I; or None."""
    c = config(cfg)
    d = d_at(hold_points(book, c.model), u)
    return None if d is None else (d + c.gains['AS_YAWRATE_P']) / c.gains['AS_YAWRATE_I']


def knee(book, cfg, u):
    """The yaw command up to which the allocator delivers 1:1 at the leg's thrust T (the flown throttle of HEAD's
    leg at u): pods over +-180 deg 1 - T (the forward pod saturates at full thrust), pods over +-90 deg T (the
    backward pod is clipped at 0), the Cloudship's tail its A (the collective is the side pods'). The allocator
    section shows it on the port of the firmware's allocator."""
    c = config(cfg)
    if not c.vectored:
        return c.A
    r = book.get(cfg, 'HEAD', u0=u)
    t = r['flown'] if r else F.trim_throttle(c.model, u)[0]
    if c.A == 1.0:
        return 1.0 - t
    if c.A == 0.5:
        return t
    raise ValueError('no knee for A %g' % c.A)


def asymptote(book, cfg, u, kf):
    """(I_target, I_stop, drift deg) of SCHED with k x kf and the perfect source at u, a long held turn at the
    full-stick cap: I_target = (D - FF) sp, what the integrator relaxes toward; I_stop where it stops, AS_YR_INT_LIM
    or, with FF short and D sp past the allocator's knee (the setpoint out of 1:1 reach), the shortfall flag's
    value knee - FF sp - P (sp - knee / D); the release I_stop / K_I. (None, None, None) without hold records."""
    c = config(cfg)
    g = c.gains
    d = d_at(hold_points(book, c.model), u)
    if d is None:
        return None, None, None
    ff, cap, _ = P.schedule(u, u, c.k * kf, g['AS_YAWRATE_FF'], g['AS_YAWRATE_MAX'], g['AS_YR_INT_LIM'], c.A)
    sp = math.radians(cap)                           # full stick: the setpoint is the cap
    lim = g['AS_YR_INT_LIM']
    target = (d - ff) * sp
    stop = lim
    kn = knee(book, cfg, u)
    if target > 0.0 and d * sp > kn:
        stop = min(lim, max(0.0, kn - ff * sp - g['AS_YAWRATE_P'] * (sp - kn / d)))
    i_inf = max(-lim, min(stop, target))
    return target, i_inf, math.degrees(i_inf / g['AS_YAWRATE_I'])


def predicted_release(book, cfg, u, kf, hold):
    """The release "in all" (deg) after a full-stick hold of `hold` s: the integrator relaxing toward I_target
    with tau, stopped at I_stop (asymptote)."""
    target, i_inf, _ = asymptote(book, cfg, u, kf)
    t = tau_of(book, cfg, u)
    if target is None or not t:
        return None
    i = target * (1.0 - math.exp(-hold / t))
    i = min(i, i_inf) if i >= 0.0 else max(i, i_inf)
    return math.degrees(i / config(cfg).gains['AS_YAWRATE_I'])


DEPART = 1.2                    # fins past this many alpha_stall in a stick: the turn departed (the stall edge)


def lim_drift(cfg='2527'):
    """The release the integrator at its limit holds, AS_YR_INT_LIM / K_I in deg (the defect's bound)."""
    g = config(cfg).gains
    return math.degrees(g['AS_YR_INT_LIM'] / g['AS_YAWRATE_I'])


def edge_departs(book, cfg, policy, held):
    """The entry speeds of the stall-edge legs whose fins passed DEPART x alpha_stall in the stick."""
    src = 'perfect' if policy.startswith('SCHED') else 'none'
    out = []
    for u in EDGE_SPEEDS:
        r = book.get(cfg, policy, src, 1.0, u, 1.0, 20.0, group='edge', held=held)
        if r is not None and (r['fin_stick'] or 0.0) > DEPART:
            out.append(u)
    return out


def edge_text(book, cfg='2527'):
    """'SCHED x / y, HEAD x / y' of the highest departing entry speed, free / held."""
    top = lambda p, h: fnum(max(edge_departs(book, cfg, p, h)) if edge_departs(book, cfg, p, h) else None, '%g')
    return 'SCHED up to %s m/s free, %s held; HEAD up to %s free, %s held' % (
        top('SCHED', False), top('SCHED', True), top('HEAD', False), top('HEAD', True))


def a_less_rho(cfg):
    """A - rho, rho = min(AS_YR_INT_LIM, A / 2): the authority the schedule's cap leaves the turn."""
    c = config(cfg)
    return c.A - min(c.gains['AS_YR_INT_LIM'], c.A / 2)


def edge_shift(book, held, cfg='2527'):
    """How far SCHED's highest departing entry speed sits above HEAD's on the stall-edge legs (m/s, '%.1f'), or
    '-' where either never departs."""
    a, b = edge_departs(book, cfg, 'SCHED', held), edge_departs(book, cfg, 'HEAD', held)
    return '%.1f' % (max(a) - max(b)) if a and b else '-'


def tau_span(book):
    ts = [tau_of(book, c, u) for c in A1 for u in FWD]
    ts = [t for t in ts if t]
    return (min(ts) / 60.0, max(ts) / 60.0) if ts else (0.0, 0.0)


# --- gate 1 -------------------------------------------------------------------

def gate1(book, configs, policies, sources_of, kf=1.0):
    """[dict] of every gated comparison on the gate's legs against HEAD's leg of the same cell:
    verdict 'smaller' (smaller than HEAD's on both measures, the stick live), 'floor' (passes only by the
    DRIFT_FLOOR, the stick live), 'dead' (would pass, but the stick turned the hull under DEAD_SHARE of HEAD's
    heading change: not a pass), 'not'; decided False where "to rest" is undecided (judged on 120 s alone)."""
    out = []

    for cfg in configs:
        base_cfg = '2512' if cfg == '2512/khold' else cfg

        for u0 in SPEEDS:
            for stick, hold in CASES:
                head = book.gate(base_cfg, 'HEAD', u0=u0, stick=stick, hold=hold)

                if head is None:
                    continue

                for policy in policies:
                    if policy == 'RESTORE' and cfg == '2512/khold':
                        continue

                    for src in (['none'] if policy == 'RESTORE' else sources_of(cfg, hold)):
                        r = book.gate(cfg, policy, src, kf, u0, stick, hold)

                        if r is None:
                            continue

                        decided = r['rest_valid'] and head['rest_valid']
                        pairs = [(r['drift_120'], head['drift_120'])]
                        if decided:
                            pairs.append((r['drift_rest'], head['drift_rest']))
                        smaller = all(abs(a) < abs(b) for a, b in pairs)
                        floor = all(abs(a) < abs(b) or abs(a) <= DRIFT_FLOOR for a, b in pairs)
                        turned = abs(r['heading_stick'] or 0.0)
                        dead = turned < DEAD_SHARE * abs(head['heading_stick'] or 0.0)
                        slow = turned < SLOW_SHARE * abs(head['heading_stick'] or 0.0)
                        verdict = 'not' if not floor else ('dead' if dead else ('smaller' if smaller else 'floor'))
                        out.append(dict(cfg=cfg, u0=u0, stick=stick, hold=hold, policy=policy, src=src, r=r, head=head,
                                        verdict=verdict, decided=decided, held=held_cell(cfg, hold), slow=slow))

    return out


def gate1_counts(rs):
    c = {v: sum(1 for x in rs if x['verdict'] == v) for v in VERDICTS}
    c['undecided'] = sum(1 for x in rs if x['verdict'] in ('smaller', 'floor') and not x['decided'])
    c['slow'] = sum(1 for x in rs if x['verdict'] in ('smaller', 'floor') and x['slow'])
    c['pass'] = c['smaller'] + c['floor']
    return c


# --- the report ---------------------------------------------------------------

def write_report(records):
    book = Book(records)
    L = []
    say = L.append
    import subprocess
    rev = subprocess.run(['git', '-C', ctl.PX4, 'rev-parse', '--short=10', 'HEAD'], capture_output=True, text=True)
    dirty = subprocess.run(['git', '-C', ctl.PX4, 'status', '--porcelain', '--untracked-files=no'],
                           capture_output=True, text=True)
    n_legs = sum(1 for r in records.values() if r.get('group') in LEG_GROUPS)
    say('Step 0c, 2026-10-01, revised 2026-10-02 after the 0c review: the forward-flight yaw policies compared on '
        'the yaw model (yaw_model/policy_compare.py, policies.py; records step0c_runs_20261001.jsonl/.csv)')
    say('PX4-Autopilot HEAD %s (%s); %d leg records, %d errors; the model as step 0a left it (forward.py, '
        'controller.py: G1 48/48, G2, G3, forward_out_20261001.txt; controller.set_rate_cap gained an optional '
        'per-direction cap, off by default)'
        % (rev.stdout.strip(), 'clean' if not dirty.stdout.strip() else 'NOT CLEAN', n_legs, len(book.errors)))
    report_summary(book, say)
    report_revision(book, say)
    report_setup(book, say)
    report_k(book, say)
    report_tau(book, say)
    report_method(book, say)

    for cfg in ('2527', '2522', '2527/A0.5', '2522/A0.5', '2512'):
        report_plant(book, say, cfg)

    report_sources(book, say)
    report_cb(book, say)
    report_ff_errors(book, say)
    report_khold(book, say)
    report_edge(book, say)
    report_a05(book, say)
    report_dist(book, say)
    report_2517(book, say)
    report_noise(book, say)
    g = report_gates(book, say)
    report_findings(book, say, g)
    report_owner(book, say)

    if book.errors:
        say('\n=== Errors ===')
        for rec in book.errors:
            say('  %s: %s' % (rec['id'], rec['error']))

    with open(OUT, 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(L) + '\n')

    write_csv(records)
    print('written %s (%d lines), %s' % (OUT, len(L), CSV_OUT))


def write_csv(records):
    keys, rows = None, []

    for rec in records.values():
        if rec.get('group') not in LEG_GROUPS or rec.get('group') == 'rest' or 'result' not in rec:
            continue

        r = rec['result']
        row = {k: rec[k] for k in ('id', 'group', 'config', 'policy', 'source', 'k_factor', 'k_override', 'u0', 'stick',
                                   'hold', 'release', 'seed', 'wall_s')}
        row['variant'] = rec.get('variant') or ''
        row['held_surge'] = bool(rec.get('held'))

        for k, v in r.items():
            if isinstance(v, (int, float, str, bool)) or v is None:
                row[k] = v

        sw = r.get('swings')
        if sw:
            for seg in ('stick', 'release'):
                for pod in ('stbd', 'port'):
                    d = (sw[seg] or {}).get(pod) or {}
                    row['%s_%s_into_rear' % (seg, pod)] = d.get('into_rear')
                    row['%s_%s_back_forward' % (seg, pod)] = d.get('back_forward')
                    row['%s_%s_rear_s' % (seg, pod)] = d.get('rear_s')
        fl = r.get('flips')
        if fl:
            for k, v in fl.items():
                row['tail_' + k] = v
        hk = r.get('hook')
        if hk:
            row['cap_min_stick'], row['cap_max_stick'] = hk['cap_min_stick'], hk['cap_max_stick']
            row['rejects'], row['switches'], row['gated_edges'] = hk['rejects'], hk['switches'], hk['gated_edges']
            for k, v in (hk.get('ramp_stick') or {}).items():
                row['ramp_stick_' + k] = v
        if 'long' in r:
            for k, v in r['long'].items():
                if not isinstance(v, (dict, list)):
                    row['long_' + k] = v
        rows.append(row)

    cols = []
    for row in rows:
        for k in row:
            if k not in cols:
                cols.append(k)

    with open(CSV_OUT, 'w', newline='', encoding='utf-8') as handle:
        w = csv.DictWriter(handle, fieldnames=cols)
        w.writeheader()
        for row in sorted(rows, key=lambda x: x['id']):
            w.writerow(row)


def cb_compare(book, cfgs=None):
    """[(cfg, src, kf, u0, stick, hold, SCHED result, SCHED_CB result)] on the gate's legs of `cfgs`, by default
    2527 and 2522, the A 1 vectored plants, where the formula's budget is the question (2512's k from its fins is
    16 x short and parks the integrator whatever the term; the A 0.5 twins stop it at the allocator's knee)."""
    out = []

    for cfg in (cfgs or A1):
        for u0 in SPEEDS:
            for stick, hold in CASES:
                for src in matrix_sources(cfg, hold):
                    a, b = book.gate(cfg, 'SCHED', src, 1.0, u0, stick, hold), book.gate(cfg, CB, src, 1.0, u0, stick,
                                                                                           hold)
                    if a is not None and b is not None:
                        out.append((cfg, src, 1.0, u0, stick, hold, a, b))
                for kf in K_ERRORS:
                    a, b = book.gate(cfg, 'SCHED', 'perfect', kf, u0, stick, hold), book.gate(cfg, CB, 'perfect', kf, u0,
                                                                                            stick, hold)
                    if a is not None and b is not None:
                        out.append((cfg, 'perfect', kf, u0, stick, hold, a, b))

    return out


def report_summary(book, say):
    """A few lines up front, every number from the records (the findings at the end say where each comes from)."""
    allsrc = tuple(P.SOURCES)
    say('\n=== In short (details in the sections below and the findings at the end) ===')
    say('(the gate\'s legs: free surge, except where the labelled speed needs the surge held: the gliders\' 300 s '
        'holds and every A 0.5 leg, "held")')
    head = [x for x in _rows(book, A1, ('HEAD',)) if x[5] == 1.0]
    if head:
        d = _span(head, 'drift_120')
        dr = _span([x for x in head if x[7]['rest_valid']], 'drift_rest')
        say('- HEAD: on 2527 and 2522 a full stick parks the integrator at %s and the release turns on %s..%s deg in '
            '120 s, %s..%s deg in all.' % ('/'.join(sorted({'%.3f' % x[7]['I_rel'] for x in head})),
                                           fnum(d[0], '%.0f'), fnum(d[1], '%.0f'), fnum(dr[0], '%.0f'),
                                           fnum(dr[1], '%.0f')))
    f = _rows(book, A1, ('SCHED',), sources=('perfect', 'ground_W0'), speeds=FWD)
    if f:
        a20, w20 = _extreme([x for x in f if x[6] == 20.0], 'drift_rest', True)
        a300, w300 = _extreme([x for x in f if x[6] == 300.0], 'drift_rest', True)
        say('- SCHED with the speed known, 2.5-6 m/s: the turn holds the cap and the release is at most %s deg in all '
            'after a 20 s hold (%s), %s deg after 300 s (%s).' % (fnum(a20, '%+.1f'), w20, fnum(a300, '%+.0f'), w300))
    cls = cb_class_stats(book)
    if cls:
        ir, bl = cls.get('lo_in_ramp'), cls.get('lo_below')
        say('- The plan\'s guarantee (the integrator within rho at any speed in the interval, k right) holds only '
            'where u_lo is past the ramp, w(u_lo) = 1: its interval term budgets against FF0 + k u_lo, the ramp '
            'supplies FF0 + w(u_lo) k u_lo. The term written on the FF supplied (SCHED_CB*, an addition) keeps the '
            'budget wherever u_hi stays past the ramp, and lowers the integrator where the interval straddles it: on '
            'the %d legs of 2527 and 2522 at 2.5-6 m/s that ended the stick with u_lo inside the ramp, the largest '
            'integrator at release goes from %s to %s (none at the limit either way); on the review\'s own example, '
            '2522 gliding with the 1.05 Pa band (free surge), %s. Where the hull itself slows into the ramp, u_hi '
            'follows it in and the cap\'s blend toward MAX takes over: neither term helps (%s). With u_lo at or '
            'below AS_SPD_LO (%d legs, ground speed only with the wind from ahead) the two terms are the same and '
            'hold their budget, but that budget is rho = min(AS_YR_INT_LIM, A / 2) = AS_YR_INT_LIM on the A 1 '
            'plants: the integrator still reaches %s (%s), and the release is bounded only by rho / K_I = %.0f deg, '
            'the defect\'s own bound.'
            % (ir['n'] if ir else 0, fnum(ir['I'][0][0] if ir else None, '%+.3f'),
               fnum(ir['I'][1][0] if ir else None, '%+.3f'), review_example_text(book, (4.0,)),
               review_example_text(book, (2.0, 2.5)), bl['n'] if bl else 0,
               fnum(bl['I'][1][0] if bl else None, '%+.3f'), bl['I'][1][1] if bl else '-', lim_drift()))
    ek = _rows(book, A1, ('SCHED',), sources=('perfect',), kfs=K_ERRORS, speeds=FWD)
    es = [x for x in _rows(book, A1, ('SCHED',), sources=allsrc, speeds=FWD) if x[2] not in ('perfect', 'ground_W0')]
    if ek and es:
        a, w = _extreme(ek, 'drift_rest', True)
        b, wb = _extreme(es, 'drift_rest', True)
        hr = _span([x for x in _rows(book, A1, ('HEAD',), speeds=FWD) if x[7]['rest_valid'] and x[5] == 1.0],
                   'drift_rest')
        tau6 = tau_of(book, '2527', 6.0)
        say('- SCHED carries whatever FF it lacks into the release: at 2.5-6 m/s (the gate\'s legs, the speed the '
            'label names), k x0.7 / 1.3 / 2 leave up to %s deg in all (%s) and the outdoor speed sources up to %s deg '
            '(%s), against HEAD\'s %s..%s deg after a full stick. The integrator collects an FF error with the time '
            'constant (D + P) / K_I, %.0f s on 2527 at 6 m/s, so the 300 s holds show %.0f..%.0f %% of the '
            'asymptote at 2.5..6 m/s (the time-constant section; 900 s holds check it).'
            % (fnum(a, '%+.0f'), w, fnum(b, '%+.0f'), wb, fnum(hr[0], '%.0f'), fnum(hr[1], '%.0f'), tau6 or 0,
               100 * (1 - math.exp(-300.0 / (tau6 or 1e9))), 100 * (1 - math.exp(-300.0 / (tau_of(book, '2527', 2.5)
                                                                                         or 1e9)))))
    r = _rows(book, GATED, ('SCHED+RESTORE',), sources=allsrc, kfs=(1.0,) + K_ERRORS, speeds=FWD)
    if r:
        a, w = _extreme(r, 'drift_rest', True)
        v, wv = _extreme_of([x for x in r if x[0] != '2512'], excursion)
        c, wc = _extreme_of([x for x in r if x[0] == '2512'], excursion)
        say('- SCHED+RESTORE: "in all" is at most %s deg (%s), by construction: it is the put-back over K_I, not the '
            'plant. The release\'s largest excursion in 120 s is the informative number: %s deg on the vectored '
            'plants (%s), %s deg on the Cloudship with k from its fins (%s). RESTORE alone leaves the stick as HEAD '
            'has it. The rule\'s cost, a yaw trim that changes during the turn, is about that change\'s I / K_I with '
            'or without it (%.0f deg for %g %% of authority).'
            % (fnum(a, '%+.1f'), w, fnum(v, '%+.1f'), wv, fnum(c, '%+.1f'), wc, math.degrees(DIST_SHARE / 0.09),
               100 * DIST_SHARE))
    edge = [book.get(cfg, 'HEAD', 'none', 1.0, 2.0, 1.0, 20.0, group='edge', held=h) for cfg in ('2527',)
            for h in (False, True)]
    two = [x for x in _rows(book, ('2527',), ('SCHED', 'SCHED_STALL'), speeds=(2.0,)) if x[5] == 1.0]
    if two and all(edge):
        say('- 2 m/s sits on the fins\' stall edge, for HEAD as for the schedule: a full stick stalls the fins once '
            'r exceeds alpha_stall u / arm (%.2f deg/s at 2 m/s on 2527). HEAD\'s own margin there is %.2f x '
            'alpha_stall free and %.2f x with the surge held; the schedule\'s cap, blended back toward MAX inside the '
            'ramp, takes it to %.1f x. On 2527 a full stick departs (fins past %g x) at entry speeds %s (the '
            'stall-edge section); where that bites follows from the ramp\'s ends, %g / %g m/s, which are 0c\'s '
            'choice, not the plan\'s.'
            % (math.degrees(config('2527').stall['c'] / P.STALL_MARGIN * 2.0), edge[0]['fin_stick'], edge[1]['fin_stick'],
               max(x[7]['fin_stick'] or 0 for x in two), DEPART, edge_text(book), P.AS_SPD_LO, P.AS_SPD_HI))
    alloc = alloc_table()
    if alloc:
        t4 = trim_of(book, '2527', 4.0)
        cell = alloc[(-90.0, 90.0)].get((round(t4, 4), 0.25)) if t4 is not None else None
        say('- A 0.5 (pods held to +-90 deg): a pod that cannot point backward delivers the yaw command 1:1 only up '
            'to the collective thrust; beyond it at half gain, with as much added surge (at 2527\'s 4 m/s trim %.3f a '
            'command of 0.25 delivers %s with surge %s, the allocator section). On the gliding 2522/A0.5 (trim 0) '
            'every unit of yaw adds surge, and a free 300 s turn runs away to %.0f m/s. The plan\'s A = 0.5 is the '
            'peak, not a steady authority. Over +-180 deg the forward pod saturates past 1 - T (%.3f at 2527\'s '
            '6 m/s trim), which A = 1 leaves out too. With the surge held, HEAD has no defect on the A 0.5 twins: '
            'its flag stops the integrator.'
            % (t4 or 0.0, fnum(cell['yaw'] if cell else None, '%.3f'), fnum(cell['surge'] if cell else None, '%.3f'),
               max([x[7]['u_stick_end'] for x in _rows(book, ('2522/A0.5',), ('HEAD',), cases=((1.0, 300.0),),
                                                       which='free')] or [0.0]),
               1.0 - (trim_of(book, '2527', 6.0) or 0.0)))
    rest = {}
    for key, rec in book.legs.items():
        if key[0] == 'rest':
            rest.setdefault((key[2], key[3]), []).append(rec['result']['identical'])
    g1 = {(p, s2): rr for (p, s2), rr in book.g1.items() if 'error' not in rr}
    if rest or g1:
        full = sorted(src for (p2, src), v in rest.items() if p2 == 'SCHED' and all(v))
        part = ['%s on %d of %d' % (src, sum(v), len(v)) for (p2, src), v in sorted(rest.items())
                if p2 == 'SCHED' and 0 < sum(v) < len(v)]
        say('- At rest: the schedule is bit-identical to HEAD only while the selector\'s interval at rest stays under '
            'AS_SPD_LO (on every plant with %s%s); ground only outdoors (W 2) puts the cap on at rest%s. The '
            'restore rule always acts at rest%s.'
            % (', '.join(full), ('; ' + ', '.join(part) + ' legs') if part else '',
               (' (the 48 gate verdicts: %d/48)' % g1[('SCHED', 'ground_W2_head')]['agree'])
               if ('SCHED', 'ground_W2_head') in g1 else '',
               (' (48 gate verdicts kept: %d/48, traces differ)' % g1[('RESTORE', None)]['agree'])
               if ('RESTORE', None) in g1 else ''))
    if k_hold_of(book.all):
        kh = _rows(book, ('2512/khold',), ('SCHED',), sources=('perfect',), speeds=(4.0, 6.0),
                   cases=((1.0, 20.0), (0.5, 20.0)))
        kf = _rows(book, ('2512',), ('SCHED',), sources=('perfect',), speeds=(4.0, 6.0), cases=((1.0, 20.0), (0.5, 20.0)))
        a, w = _extreme(kh, 'drift_rest', True)
        b, wb = _extreme(kf, 'drift_rest', True)
        cf_ = cloudship_closed_form(book)
        say('- The Cloudship (2512): k from its fins is %.1f x short of its steady-turn slope. A force balance of the '
            'tail\'s side force against the fins\' (no hull side force) gives m |x_t| |x_f| / (x_f - x_t) / authority '
            '= %.1f per rad/s per m/s against the %.2f measured: the turn\'s torque is set by the centripetal force '
            'and the tail-to-fin gap, not by fin damping. At 4 / 6 m/s, 20 s holds, perfect source: SCHED with k from '
            'the fins releases up to %s deg in all (%s); with k the measured slope, up to %s deg (%s).'
            % (k_hold_of(book.all) / config('2512').k, cf_['k_closed'] if cf_ else float('nan'),
               k_hold_of(book.all), fnum(b, '%+.0f'), wb, fnum(a, '%+.0f'), w))


def report_revision(book, say):
    say('\n=== Revised 2026-10-02 after the 0c review (16 findings; where each is answered) ===')
    for line in (
            '1  the plan\'s interval term against the FF the ramp supplies: SCHED_CB* flown on every gated cell; '
            'section "The interval term and the ramp"; "For the owner" 1-2',
            '2  rho = AS_YR_INT_LIM on the A 1 plants: "In short", the findings, "For the owner" 3',
            '3, 10  the pitot bands said for what they are (dq below the zero error, a misconfiguration; dq covering '
            'the bias); an SDP3x with its band at its documented zero error flown (dqcheck): the 1.05 Pa case bit for '
            'bit',
            '4  the ramp\'s ends 1.5 / 2.5 m/s labelled as 0c\'s choice (policies.py, setup, the 2 m/s findings)',
            '5  A per direction: PlantConfig.A_dir, the hook sets a per-direction cap where they differ '
            '(controller.set_rate_cap cap_neg_deg_s, off by default); no 0c plant differs',
            '6  gate 1: dead sticks (under %g %% of HEAD\'s heading change in the stick) are not passes; passes on '
            'merit, by the floor, undecided and slow (under %g %%) ones are counted apart'
            % (100 * DEAD_SHARE, 100 * SLOW_SHARE),
            '7  the gliders\' (2522, 2512) 300 s holds re-flown with the surge held at the label; these are the gate\'s '
            'legs, the free-surge legs stay as the record with the speed they flew',
            '8  the A 0.5 twins re-flown with the surge held; the allocator\'s delivery table; the plan\'s A = 0.5 '
            'is a peak, not a steady authority',
            '9  the integrator\'s time constant per plant and speed, the asymptote beside the 300 s holds, 900 s '
            'holds to check it; the setup sentence corrected',
            '11  the 2522 speed gain in a full-stick turn: the pod-swing kick at the onset, not the steady couple',
            '12  the 2 m/s stall edge flown at 1.8-2.4 m/s, free and held, HEAD and SCHED',
            '13  the 900 s check: one leg of eight is off by about half a degree, a late flag, named',
            '14  2517: SCHED raises the peak rate and the stalled share at 4 m/s',
            '15  the restore policies: the 120 s excursion beside "in all", which is the put-back by construction',
            '16  the Cloudship: the steady-turn force balance beside the measured slope'):
        say('  ' + line)


def report_setup(book, say):
    say('\n=== What is compared ===')
    say('The plan\'s structure (d1_universal_architecture_20261001.md 2.가-다) made executable in policies.py and '
        'flown through forward.run_leg\'s per-cycle hook, before the controller of each cycle:')
    say('  speed selector: pitot interval from the sample\'s q = 1/2 rho TAS^2 +- dq (rho %.3f), ground interval '
        '[max(u_g - W, 0), max(u_g + W, 0)], intersection when both are usable (empty: the pitot dropped that cycle), '
        'a lost source left at once and taken back after %g s usable, both bounds through a %g s first-order filter '
        '(alpha = dt / (tau + dt)), all lost: u_lo decays to 0, u_hi held while the loop is closed. The filter starts '
        'converged (the selector has run since boot; a leg starts in steady flight).'
        % (P.RHO, P.SELECTOR_HOLD_S, P.SELECTOR_TAU_S))
    say('  schedule: FF = FF0 + w(u_lo) k u_lo; r_lim = min(MAX, (A - rho) / (FF0 + k u_hi), rho / (k (u_hi - u_lo))'
        '[, c u_lo]); cap = MAX + w(u_hi) (r_lim - MAX), w = interpolate(u, %g, %g, 0, 1), rho = min(AS_YR_INT_LIM, '
        'A / 2); the cap is the stick\'s full scale and the clamp at the loop input (both); FF0, MAX, INT_LIM the '
        'plant\'s HEAD values (AS_YAWRATE_FF, AS_YAWRATE_MAX, AS_YR_INT_LIM); P and I untouched. The ramp\'s ends '
        '%g / %g m/s are 0c\'s choice: the plan names the ramp and what it must cover (the sensor floor, about '
        '1.5-2 m/s in the speed document, and the gz fins\' stall zone, 2-2.5 m/s), not its values. A is per '
        'direction (2.나); every 0c plant has the same A both ways'
        % (P.AS_SPD_LO, P.AS_SPD_HI, P.AS_SPD_LO, P.AS_SPD_HI))
    say('  policies: HEAD (today); RESTORE (options/D\'s restore rule, edge %s, on HEAD gains: the integrator saved '
        'when the stick leaves the deadzone is put back when it returns); SCHED; SCHED_STALL (SCHED with the stall '
        'term c u_lo); SCHED+RESTORE; and two additions not in the plan: SCHED+RESTORE_FWD* (SCHED+RESTORE with the '
        'put-back done only on a cycle with w(u_lo) > 0, so that it is inert at rest; flown on the primary plants, '
        'and at rest on every plant), and SCHED_CB* (after the review: SCHED with the interval term on the FF the '
        'ramp supplies, rho / (k (u_hi - w(u_lo) u_lo)); flown on every gated cell SCHED flies)' % P.RESTORE_EDGE)
    say('  speed sources (FF errors fly the perfect source):')

    for s in P.SOURCES.values():
        say('    %-15s %s' % (s.name, s.describe()))

    say('    the pitot band dq is the plan\'s tolerance (2.가 허용폭); one count (SDP3x %g Pa, MS4525DO %g Pa, the speed '
        'document) is a resolution. A +1 Pa bias reads %.2f m/s at rest. In the model a pitot is its bias and its '
        'band, nothing else: an SDP3x with its band at its documented CMODEL 0 offset (%g Pa) is the pitot_bias_ms '
        'case exactly (the dqcheck legs below)'
        % (P.DQ['SDP3x'], P.DQ['MS4525DO'], math.sqrt(2.0 / P.RHO), P.SDP_OFFSET))
    t25, t4, t6 = (tau_of(book, '2527', u) for u in (2.5, 4.0, 6.0))
    say('  legs (forward.run_leg, HEAD gains per airframe): warm-up 1 s held, Manual 2 s, Acro at t0, %g s centred, '
        'the stick (full 20 s, half 20 s, full 300 s), released %g s; throttle at trim as the 0b check flies it '
        '(gliding at 0 on the drag-free 2522 and 2512); u0 %s m/s; noise-free unless the noise section says so. The '
        '300 s hold is %.1f / %.1f / %.1f integrator time constants on 2527 at 2.5 / 4 / 6 m/s, so at the higher '
        'speeds it shows only part of a slow FF error\'s asymptote: the time-constant section gives the asymptotes '
        'and checks them with 900 s holds'
        % (CENTRED_S, RELEASE_S, ' / '.join('%g' % u for u in SPEEDS), 300.0 / (t25 or 1e9), 300.0 / (t4 or 1e9),
           300.0 / (t6 or 1e9)))
    say('  the gate\'s legs: free surge where the hull keeps its label (2527 on its trim thrust; the gliders\' 20 s '
        'holds); the surge held at the label (run_leg hold_u) on the gliders\' 300 s holds, which glide down to the '
        'same speed from any entry or tumble, and on every A 0.5 leg, whose yaw adds forward thrust. The free-surge '
        'legs there stay as the record, with the speed they flew')
    say('  primary plants (all 12 sources, every policy): 2527, 2522. Secondary (A 0.5 twins, 2512, 2517): sources '
        '%s on the 20 s holds, %s on the 300 s hold, no RESTORE_FWD*.' % (', '.join(SECONDARY_SOURCES),
                                                                         ', '.join(LONG_SOURCES)))
    say('Metrics. FRD, every stick positive: a positive heading change continues the turn, a negative one turns '
        'back.')
    say('  drift: the heading change over the 120 s release, and "to rest": that plus I(end) / K_I, the rest of the '
        'turn the integrator still holds (exact while no flag clips it and it stays inside its limit: the release '
        'integrates e = -r, so the remaining heading is I / K_I; checked below). Written 120/rest in deg; "?" where '
        'the extrapolation does not hold (a flag in the last %g s, the integrator at its limit, |r| growing, or a '
        'plant that does not settle). With a put-back, "to rest" is the put-back over K_I by construction; the '
        'release\'s excursion is then the plant\'s number' % LAST_S)
    say('  stick: r mean over its last 5 s against the setpoint at its end (= the cap at full stick), deg/s; the '
        'integrator at release (normalized torque; the defect parks it at AS_YR_INT_LIM %g with no flag)'
        % config('2527').gains['AS_YR_INT_LIM'])
    say('  reversals: pod swings (vectored; cos(tilt) past +-%g) or tail flips (Cloudship; outside +-%g), stick / '
        'release; fins: the largest |alpha| / alpha_stall in the stick, S = the share of the stick on the stalled '
        'branch' % (F.FIN_BAND, F.TAIL_BAND))
    say('  settle: the time after release from which |r| stays < %g deg/s, in the window; past it, "~" an estimate '
        '(quasi-static: r = I / (D + P), I decaying with (D + P) / K_I)' % SETTLE_RATE)


def report_k(book, say):
    say('\n=== k, c and A from the files ===')
    seen = set()

    for name in ('2527', '2522', '2512', '2517'):
        cfg = config(name)
        f, st = cfg.fins, cfg.stall

        if cfg.model in seen:
            continue

        seen.add(cfg.model)
        say('  %s %s: k = %.4f per rad/s per m/s = the engine\'s fin damping %.4f N m s/rad per m/s (mean of %s, '
            'the %s member\'s added_mass blocks) over the yaw authority %.3f N m; the files\' sum gives %.4f (%.4f), '
            'the replica linearised %.4f (%.4f)'
            % (name, cfg.model, f['k'], f['K_d'], ', '.join('%s %.4f' % w for w in f['runs']), f['member'],
               f['authority'], f['k_files'], f['K_files'], f['k_linear'], f['K_linear']))
        say('    c = %g x alpha_stall %.4f rad (%.2f deg) / arm %.4f m = %.5f rad/s per m/s (%.3f deg/s per m/s): the '
            'stall cap is %s deg/s at 2 / 2.5 / 4 / 6 m/s'
            % (st['margin'], st['alpha_stall'], math.degrees(st['alpha_stall']), st['arm'], st['c'],
               math.degrees(st['c']), ' / '.join('%.2f' % math.degrees(st['c'] * u) for u in SPEEDS)))

    for name in ('2527', '2522', '2527/A0.5', '2522/A0.5', '2512', '2517'):
        cfg = config(name)
        g = cfg.gains
        caps = []

        for u in SPEEDS:
            ff, cap, info = P.schedule(u, u, cfg.k, g['AS_YAWRATE_FF'], g['AS_YAWRATE_MAX'], g['AS_YR_INT_LIM'], cfg.A)
            _, cap_s, info_s = P.schedule(u, u, cfg.k, g['AS_YAWRATE_FF'], g['AS_YAWRATE_MAX'], g['AS_YR_INT_LIM'],
                                          cfg.A, c_stall=cfg.stall['c'])
            _, cap_c, _ = P.schedule(u, u, cfg.k, g['AS_YAWRATE_FF'], g['AS_YAWRATE_MAX'], g['AS_YR_INT_LIM'], cfg.A,
                                     budget='supplied')
            caps.append('%g: FF %.2f, cap %.2f (%s; stall %.2f, %s; CB %.2f)'
                        % (u, ff, cap, info['binding'], cap_s, info_s['binding'], cap_c))

        say('  %-10s A+/A- %.2f/%.2f, rho %.2f, FF0 %g, MAX %g; at a known speed (u_lo = u_hi) %s'
            % (name, cfg.A_dir[0], cfg.A_dir[1], min(g['AS_YR_INT_LIM'], cfg.A / 2), g['AS_YAWRATE_FF'],
               g['AS_YAWRATE_MAX'], '; '.join(caps)))

    say('  (CB: SCHED_CB*\'s cap. With u_lo = u_hi the plan\'s interval term is absent, but the supplied-FF term is '
        'k u (1 - w(u)), present inside the ramp: at 2 m/s it lowers the cap a little)')
    say('  The plants\' steady-turn torque per rad/s (hold_slope: surge held, Manual torque held to the steady turn, '
        'the plant alone; what the plan\'s in-flight k procedure reads, index.md:118-120 at two speeds), per m/s, '
        'against FF0 / u + k:')
    rows = {}

    for h in book.hold:
        rows.setdefault(h['target'], []).append(h)

    for target, hs in rows.items():
        model = cl.ALL_TARGETS[target][0]
        cfg = next(c for c in (config(n) for n in ('2527', '2522', '2512', '2517')) if c.model == model)
        parts = []
        steady = True

        for h in sorted(hs, key=lambda x: (x['u0'], x['torque'])):
            if 'error' in h:
                parts.append('%g m/s %g: %s' % (h['u0'], h['torque'], h['error']))
                continue
            parts.append('%g m/s at %g: %.3f (r %.2f deg/s, v %+.2f m/s)' % (h['u0'], h['torque'], h['per_m_s'], h['r'],
                                                                           h['v']))
            steady = steady and abs(h['r']) < 10.0

        ok = [h for h in hs if 'error' not in h]
        mean = sum(h['per_m_s'] for h in ok) / len(ok) if ok else None
        say('    %-44s %s' % (model, '; '.join(parts)))

        if steady and mean:
            say('      mean %.3f per rad/s per m/s = %.2f x k (%.4f); the schedule\'s FF0 + k u at 4 m/s supplies %.1f '
                '%% of the hold torque there'
                % (mean, mean / cfg.k, cfg.k, 100 * (cfg.gains['AS_YAWRATE_FF'] + cfg.k * 4.0) / (
                    next(h['per_rad_s'] for h in ok if h['u0'] == 4.0 and h['torque'] == HOLD_TORQUES[-1]))))
        else:
            say('      no steady turn: the hull departs into a spin (|r| %.0f..%.0f deg/s, the sway %.1f..%.1f m/s); the '
                'directionally unstable plant has no hold slope' % (min(abs(h['r']) for h in ok), max(abs(h['r']) for h in ok),
                                                                    min(h['v'] for h in ok), max(h['v'] for h in ok)))

    cf_ = cloudship_closed_form(book)
    if cf_:
        say('  The Cloudship\'s steady turn as a force balance (cloudship_fin, rigid: no added mass): the tail\'s side '
            'force F_t at x_t and the fins\' F_f at x_f (both from the centre of mass) carry the centripetal m u r, '
            'and their moments cancel, so F_t = m u r x_f / (x_f - x_t) and the torque command is |x_t| F_t over the '
            'authority: N / (u r) = m |x_t| |x_f| / (x_f - x_t) / authority = %.3f x %.3f x %.3f / %.3f / %.3f = %.2f '
            'per rad/s per m/s, against the %.3f measured (%+.1f %%: the balance takes the fins\' force as a pure side '
            'force and leaves out the bank of the turn). It scales with 1 / (x_f - x_t), the %.3f m tail-to-fin gap: '
            'with the arms taken from the link origin instead, %.3f m behind the centre of mass, it gives %.2f, %+.1f '
            '%% for %.0f mm. A tail thruster\'s k has to be measured in a held turn, since a few cm of tail position or '
            'fin cp move it a lot'
            % (cf_['m'], abs(cf_['x_t']), abs(cf_['x_f']), cf_['x_f'] - cf_['x_t'], cf_['authority'], cf_['k_closed'],
               cf_['k_meas'], 100 * (cf_['k_closed'] / cf_['k_meas'] - 1), cf_['x_f'] - cf_['x_t'], cf_['com_x'],
               cf_['k_origin'], 100 * (cf_['k_origin'] / cf_['k_closed'] - 1), 1000 * cf_['com_x']))


def cloudship_closed_form(book):
    """The Cloudship's steady-turn slope as a force balance (0c review finding 16), from the files: the rigid
    member's mass and centre of mass, the tail's position (cloudship_actuation.TailChain), the vertical fins' cp
    (their mean x), the yaw authority. None without the hold records."""
    khd = k_hold_of(book.all)
    if khd is None:
        return None
    import cloudship_actuation as ca
    model = config('2512').model
    m, com = P.gz_model.centre_of_mass(P.gz_model.family(model))
    tail = ca.TailChain('cloudship').tail_pos
    fins = P.gz_model.vertical_fins(model)
    x_f = sum(f['cp'][0] for f in fins) / len(fins) - com[0]
    x_t = tail[0] - com[0]
    authority = P.gz_model.yaw_authority(model)
    k_closed = m * abs(x_t) * abs(x_f) / (x_f - x_t) / authority
    # the same with the arms from the link origin (the review's numbers): the geometry's sensitivity
    xo_t, xo_f = x_t + com[0], x_f + com[0]
    k_origin = m * abs(xo_t) * abs(xo_f) / (xo_f - xo_t) / authority
    return {'m': m, 'x_t': x_t, 'x_f': x_f, 'com_x': com[0], 'authority': authority, 'k_closed': k_closed,
            'k_origin': k_origin, 'k_meas': khd}


def report_tau(book, say):
    say('\n=== The integrator\'s time constant and the long holds (review finding 9) ===')
    say('In a held turn the integrator relaxes toward (D - FF) sp with tau = (D + P) / K_I, D the plant\'s steady-turn '
        'torque per rad/s at the speed (the hold slopes above), P %g, K_I %g; a hold of T s reaches 1 - e^(-T / tau) '
        'of it. Beside it, tau from HEAD\'s own release decay (full stick 20 s, free surge: -120 / ln(I(120 s) / I at '
        'release); the gliders slow down in that release, so theirs is shorter):'
        % (config('2527').gains['AS_YAWRATE_P'], config('2527').gains['AS_YAWRATE_I']))
    rows = []
    for cfg in ('2527', '2522', '2512'):
        for u in SPEEDS:
            t = tau_of(book, cfg, u)
            h = book.get(cfg, 'HEAD', u0=u, stick=1.0, hold=20.0)
            obs = None
            if h and h['I_end'] and h['I_rel'] and h['I_end'] > 0 and h['I_rel'] > 0 and h['I_end'] < h['I_rel']:
                obs = -RELEASE_S / math.log(h['I_end'] / h['I_rel'])
            d = d_at(hold_points(book, config(cfg).model), u)
            rows.append([cfg, '%g' % u, fnum(d, '%.2f'), fnum(t, '%.0f'), fnum(obs, '%.0f'),
                         fnum(300.0 / t if t else None, '%.2f'),
                         fnum(100 * (1 - math.exp(-300.0 / t)) if t else None, '%.0f %%'),
                         fnum(100 * (1 - math.exp(-20.0 / t)) if t else None, '%.0f %%')])
    table(say_lines(say), ['plant', 'u m/s', 'D per rad/s', 'tau s', 'tau, HEAD release', '300 s in tau',
                           'reached in 300 s', 'in 20 s'], rows)
    say('SCHED with a wrong k (the perfect source, full stick, FF and the cap built on the wrong k): the integrator '
        'relaxes toward I_target = (D - FF) sp and stops at AS_YR_INT_LIM, or earlier where FF is short and D sp is '
        'past the allocator\'s 1:1 knee (1 - T on these +-180 deg pods, T the thrust; the allocator section): there '
        'the shortfall flag stops it at knee - FF sp - P (sp - knee / D). Predicted release "in all" after a hold T: '
        'I_target (1 - e^(-T / tau)) stopped there, over K_I; flown: the gate\'s legs (2522 held) and the 900 s holds '
        '(the tau group: 2527 free on its trim thrust, 2522 held). Deg:')
    rows = []
    for cfg in ('2527', '2522'):
        for u in FWD:
            for kf in K_ERRORS:
                target, i_inf, d_inf = asymptote(book, cfg, u, kf)
                r300 = book.gate(cfg, 'SCHED', 'perfect', kf, u, 1.0, 300.0)
                r900 = book.get(cfg, 'SCHED', 'perfect', kf, u, 1.0, TAU_HOLD, group='tau', held=held_cell(cfg, 300.0))
                lim = config(cfg).gains['AS_YR_INT_LIM']
                stop = '' if i_inf is None or i_inf == target else (
                    'the limit' if abs(abs(i_inf) - lim) < 1e-12 else 'the flag')
                rows.append([cfg, '%g' % u, 'x%g' % kf, '%.3f' % knee(book, cfg, u), fnum(target, '%+.3f'),
                             '%s%s' % (fnum(i_inf, '%+.3f'), (' (%s)' % stop) if stop else ''), fnum(d_inf, '%+.0f'),
                             fnum(predicted_release(book, cfg, u, kf, 300.0), '%+.0f'),
                             fnum(r300['drift_rest'] if r300 and r300['rest_valid'] else None, '%+.0f'),
                             fnum(predicted_release(book, cfg, u, kf, TAU_HOLD), '%+.0f') if r900 else '',
                             (fnum(r900['drift_rest'] if r900['rest_valid'] else None, '%+.0f') +
                              ' (u %.2f)' % r900['u_stick_end']) if r900 else ''])
    table(say_lines(say), ['plant', 'u m/s', 'k', 'knee', 'I_target', 'I stops at', 'asymptote', 'predicted 300 s',
                           'flown 300 s', 'predicted 900 s', 'flown 900 s (u at its end)'], rows)


def report_method(book, say):
    say('\n=== Method checks ===')
    # the identity behind "to rest", on every main leg without a put-back: the
    # release turns (I at release - I at its end) / K_I wherever no flag
    # clipped the integrator during it; a leg off by more than 0.5 deg had one
    res = []

    for key, rec in book.legs.items():
        if key[0] != 'main' or rec['result']['restore_events']:
            continue
        r = rec['result']
        res.append((abs(math.degrees((r['I_rel'] - r['I_end']) / 0.09) - r['drift_120']), key))

    if res:
        res.sort()
        close = sum(1 for d, _ in res if d < 0.5)
        say('  the identity behind "to rest" (each release cycle adds K_I e dt with e = -r, so a window without a '
            'flag turns (I at its start - I at its end) / K_I; K_I 0.09 on every plant), over the whole 120 s '
            'release: within 0.5 deg on %d of %d legs without a put-back, median %.3f deg; on the others a flag '
            'clipped the integrator somewhere in the release (a pod swinging back at the release, a stalled turn), '
            'worst %.1f deg (%s). "To rest" needs the identity only after the window, where it is required to hold '
            '(no flag in the last %g s, the integrator inside its limit, |r| not growing); the 900 s releases below '
            'test exactly that' % (close, len(res), res[len(res) // 2][0], res[-1][0],
                                   '|'.join(str(x) for x in res[-1][1][1:8]), LAST_S))

    longs = sorted((rec for key, rec in book.legs.items() if key[0] == 'long'), key=lambda x: x['id'])
    gaps = [abs(rec['result']['long']['predicted_rest'] - rec['result']['long']['flown_rest']) for rec in longs]
    say('  the "to rest" extrapolation against releases flown to 900 s (heading at 120 s + I(120 s) / K_I, against '
        'the heading at 900 s + I(900 s) / K_I): %d of %d within 0.01 deg%s:'
        % (sum(1 for g in gaps if g < 0.01), len(gaps), (', the other%s off by %s deg' % (
            's' if sum(1 for g in gaps if g >= 0.01) > 1 else '', ', '.join('%.2f' % g for g in gaps if g >= 0.01)))
           if any(g >= 0.01 for g in gaps) else ''))

    for rec in longs:
        lg = rec['result']['long']
        late = ''
        if lg.get('flag_rows_after_120'):
            sw = lg.get('swings_release') or {}
            back = sum((d or {}).get('back_forward', 0) for d in sw.values())
            late = ('; %d rows (10 Hz) flagged from +%.1f to +%.1f s after the release, the integrator held at %+.4f '
                    'meanwhile (%d pod swing%s back forward in the release): the flag the extrapolation from 120 s '
                    'cannot see' % (lg['flag_rows_after_120'], lg['flags_after_120'][0], lg['flags_after_120'][1],
                                    lg['I_flagged'], back, '' if back == 1 else 's'))
        say('    %-11s %-8s %-15s k x%g, %g m/s, %s: predicted %+.2f deg, flown %+.2f deg (I %+.4f at 120 s, %+.4f at '
            '900 s; %+.2f deg flown by 900 s)%s'
            % (rec['config'], rec['policy'], rec['source'], rec['k_factor'], rec['u0'],
               case_name(rec['stick'], rec['hold']), lg['predicted_rest'], lg['flown_rest'], lg['I_120'],
               lg['I_end'], lg['heading_end'], late))

    # entry states: every run of a base case entered the stick in one state
    groups = {}

    for key, rec in book.legs.items():
        if key[0] not in ('main', 'khold'):
            continue
        cfg = '2512' if key[1] == '2512/khold' else key[1]
        groups.setdefault((cfg, key[5], key[6], key[7], key[11]), []).append(rec['result']['entry_stick'])

    differ = [k for k, v in groups.items() if any(not _same(v[0], x) for x in v[1:])]
    say('  entry states: the runs of each base case (plant, speed, stick, hold, surge free or held) entered the stick '
        'in one state, every field of the plant and the loop equal: %d of %d base cases%s (the policies set FF and '
        'the cap while the stick is centred, where both multiply a zero setpoint)'
        % (len(groups) - len(differ), len(groups), '' if not differ else '; they differ on %s' % differ))
    dq = [(rec, book.get(rec['config'], 'SCHED', 'pitot_bias_ms', 1.0, rec['u0'], rec['stick'], rec['hold']))
          for key, rec in book.legs.items() if key[0] == 'dqcheck']
    same = sum(1 for a, b in dq if b is not None and _same(_strip_new(a['result']), _strip_new(b)))
    say('  the SDP3x with its band at its documented CMODEL 0 offset (%g Pa) against the +1 Pa bias (dqcheck): %d of %d '
        'legs (2527, 2522, SCHED, every speed, full 20 s and 300 s) equal to the pitot_bias_ms legs field for field'
        % (P.SDP_OFFSET, same, len(dq)))


def _strip_new(result):
    """A leg summary without the fields added after the review (for comparing a new leg to an older one)."""
    out = {k: v for k, v in result.items() if k not in ('held', 'u_mean_stick', 'rel_r_absmax')}
    if 'hook' in out:
        out['hook'] = {k: v for k, v in out['hook'].items() if k != 'ramp_stick'}
    return out


def report_plant(book, say, cfg):
    pols = plant_pols(cfg)
    say('\n=== %s ===' % CONFIG_TEXT[cfg])
    entry = []

    for u0 in SPEEDS:
        r = book.get(cfg, 'HEAD', u0=u0)
        if r:
            e = r['entry_stick']
            entry.append('%g m/s: throttle %.4f flown %.3f, at the stick u %.3f m/s, v %+.4f, r %+.3f deg/s'
                         % (u0, r['trim'], r['flown'], e[0], e[1], e[2]))
    say('Entry (HEAD, free surge; every policy enters the stick in this state, method checks): ' + '; '.join(entry))
    held_rows = [case_name(*c) for c in CASES if held_cell(cfg, c[1])]
    if held_rows:
        say('The gate\'s legs fly the surge held at the label on: %s (marked "H"); the free-surge record of those '
            'rows follows the tables' % ', '.join(held_rows))
    say('Release drift, deg (120 s / to rest), the perfect source, nominal k; u at the stick\'s end (HEAD / SCHED):')
    rows = []

    def tag(stick, hold):
        return ' H' if held_cell(cfg, hold) else ''

    for u0 in SPEEDS:
        for stick, hold in CASES:
            h, s = book.gate(cfg, 'HEAD', u0=u0, stick=stick, hold=hold), book.gate(cfg, 'SCHED', 'perfect', 1.0, u0,
                                                                                  stick, hold)
            rows.append(['%g' % u0, case_name(stick, hold) + tag(stick, hold)] +
                        [drift_cell(book.gate(cfg, p, 'perfect', 1.0, u0, stick, hold)) for p in pols] +
                        ['%s / %s' % (fnum((h or {}).get('u_stick_end'), '%.2f'), fnum((s or {}).get('u_stick_end'),
                                                                                      '%.2f'))])
    table(say_lines(say), ['u m/s', 'stick'] + [SHORT[p] for p in pols] + ['u flown'], rows)
    say('The stick: r mean last 5 s / setpoint at its end (deg/s; at full stick the setpoint is the applied '
        'cap, at half stick (0.5 - dz) / (1 - dz) = 0.444 of it), integrator at release:')
    rows = []

    for u0 in SPEEDS:
        for stick, hold in CASES:
            rows.append(['%g' % u0, case_name(stick, hold) + tag(stick, hold)] +
                        [stick_cell(book.gate(cfg, p, 'perfect', 1.0, u0, stick, hold)) for p in pols])
    table(say_lines(say), ['u m/s', 'stick'] + [SHORT[p] for p in pols], rows)
    say('Reversals stick/release (%s) and the fins (max |alpha|/alpha_stall in the stick; S = %% of it stalled):'
        % ('pod swings' if config(cfg).vectored else 'tail flips'))
    rows = []

    for u0 in SPEEDS:
        for stick, hold in CASES:
            rows.append(['%g' % u0, case_name(stick, hold) + tag(stick, hold)] +
                        [rev_cell(book.gate(cfg, p, 'perfect', 1.0, u0, stick, hold)) for p in pols])
    table(say_lines(say), ['u m/s', 'stick'] + [SHORT[p] for p in pols], rows)
    say('Release detail, full stick 20 s%s: r at +10/+60/+120 s (deg/s), settle time to |r| < %g deg/s (s), the '
        'torque step at release, the release\'s extremes of heading (deg):'
        % (' (held)' if held_cell(cfg, 20.0) else '', SETTLE_RATE))
    rows = []

    for u0 in SPEEDS:
        for p in pols:
            r = book.gate(cfg, p, 'perfect', 1.0, u0, 1.0, 20.0)
            if r is None:
                continue
            settle = fnum(r['t_settle'], '%.1f') if r['t_settle'] is not None else (
                '~%.0f' % r['t_settle_est'] if r['t_settle_est'] is not None else '>%g' % RELEASE_S)
            ovs = None
            if r['sp_end'] and r['r_last5'] is not None and r['r_last5'] >= 0.9 * r['sp_end']:
                ovs = 100.0 * (r['r_peak'] - r['sp_end']) / r['sp_end']
            rows.append(['%g' % u0, SHORT[p], '%s / %s / %s' % (fnum(r['r_10'], '%+.2f'), fnum(r['r_60'], '%+.2f'),
                                                                 fnum(r['r_120'], '%+.2f')), settle,
                         fnum(r['torque_step'], '%+.3f'), '%s..%s' % (fnum(r['rel_h_min'], '%+.1f'),
                                                                     fnum(r['rel_h_max'], '%+.1f')),
                         fnum(ovs, '%+.1f %%'), '%s/%s' % (fnum(r['I_max'], '%.3f'), fnum(r['I_end'], '%+.4f')),
                         '%s -> %s' % (fnum(r['u_stick_end'], '%.2f'), fnum(r['u_release_end'], '%.2f'))])
    table(say_lines(say), ['u m/s', 'policy', 'r +10/+60/+120', 'settle s', 'torque step', 'heading range',
                           'overshoot (reachable sp)', 'I max/end', 'u stick end -> release end'], rows)

    if held_rows:
        say('Free surge, as flown (the record; not the gate\'s legs): release 120/rest, u at the stick\'s start -> '
            'end, the integrator at release:')
        rows = []
        for u0 in SPEEDS:
            for stick, hold in CASES:
                if not held_cell(cfg, hold):
                    continue
                row = ['%g' % u0, case_name(stick, hold)]
                for p in ('HEAD', 'SCHED', 'SCHED+RESTORE'):
                    r = book.get(cfg, p, 'perfect', 1.0, u0, stick, hold)
                    row.append('n/r' if r is None else '%s, u %.2f -> %.2f, I %+.3f' % (
                        drift_cell(r), r['u_stick_start'], r['u_stick_end'], r['I_rel']))
                rows.append(row)
        table(say_lines(say), ['u m/s', 'stick', 'HEAD free', 'SCHED free', 'SCHED+RESTORE free'], rows)


def report_sources(book, say):
    say('\n=== Speed sources on the primary plants (nominal k; the gate\'s legs) ===')
    say('Release drift 120/rest (deg) per source; the stick\'s r last 5 s / setpoint (deg/s) and the source the '
        'selector ended the stick on with its interval [u_lo, u_hi] (m/s) and where that interval sits against the '
        'ramp (1: u_lo past it; s: u_hi past it, u_lo inside it; b: u_hi past it, u_lo at or below AS_SPD_LO; r: '
        'u_hi inside it); "H" a held leg:')

    for cfg in PRIMARY:
        for policy in ('SCHED', 'SCHED_STALL', 'SCHED+RESTORE', 'SCHED+RESTORE_FWD', CB):
            say('%s, %s:' % (cfg, SHORT[policy]))
            rows = []

            for src in P.SOURCES:
                row = [src]

                for stick, hold in ((1.0, 20.0), (1.0, 300.0)):
                    for u0 in SPEEDS:
                        r = book.gate(cfg, policy, src, 1.0, u0, stick, hold)
                        row.append(drift_cell(r))
                rows.append(row)

            table(say_lines(say), ['source'] + ['%g %s%s' % (u, h, ' H' if held_cell(cfg, 300.0 if h == '300 s' else 20.0)
                                                             else '') for h in ('20 s', '300 s') for u in SPEEDS], rows)

        say('%s, SCHED, the stick (full, 20 s and 300 s): r last 5 s / sp, and the selector at the stick\'s end:' % cfg)
        rows = []
        mark = {'w_lo=1': '1', 'lo_in_ramp': 's', 'lo_below': 'b', 'w_hi<1': 'r', None: '-'}

        for src in P.SOURCES:
            row = [src]
            for hold in (20.0, 300.0):
                for u0 in SPEEDS:
                    r = book.gate(cfg, 'SCHED', src, 1.0, u0, 1.0, hold)
                    if r is None:
                        row.append('n/r')
                        continue
                    row.append('%s/%s %s[%s,%s]%s' % (fnum(r['r_last5'], '%.2f'), fnum(r['sp_end'], '%.2f'),
                                                      (r['src_end'] or '-')[0], fnum(r['u_lo_end'], '%.1f'),
                                                      fnum(r['u_hi_end'], '%.1f'), mark[end_class(r)]))
            rows.append(row)

        table(say_lines(say), ['source'] + ['%g %s' % (u, h) for h in ('20 s', '300 s') for u in SPEEDS], rows)


W2_SOURCES = ('ground_W2_head', 'ground_W2_tail', 'ground_W2_x90')
W3_SOURCES = ('ground_W3_head', 'ground_W3_tail', 'ground_W3_x90')
CB_FAMILIES = (('one point (perfect, ground W0; k errors)', ('perfect', 'ground_W0')),
               ('pitot, dq below the bias (pitot_bias_sdp)', ('pitot_bias_sdp',)),
               ('pitot, dq covering the bias (pitot_bias_ms)', ('pitot_bias_ms',)),
               ('pitot stuck', ('pitot_stuck',)),
               ('pitot dropout, then ground W 2', ('pitot_dropout',)),
               ('ground only, W 2 (head, tail, 90)', W2_SOURCES),
               ('ground only, W 3 (head, tail, 90)', W3_SOURCES))
BANDS = (('2.5-6 m/s', FWD), ('2 m/s', (2.0,)))
LIMIT_I = 0.299                 # "at the limit": within 0.001 of AS_YR_INT_LIM 0.3


def _cb_row(sel, verdict):
    """The SCHED -> SCHED_CB* statistics of a selection of cb_compare tuples."""
    a = [(x[0], 'SCHED', x[1], x[2], x[3], x[4], x[5], x[6]) for x in sel]
    b = [(x[0], CB, x[1], x[2], x[3], x[4], x[5], x[7]) for x in sel]
    ok = lambda pol, x: verdict.get((x[0], x[3], x[4], x[5], pol, x[1], x[2])) in ('smaller', 'floor')
    return {'n': len(sel), 'identical': sum(1 for x in sel if _same(_strip_new(x[6]), _strip_new(x[7]))),
            'limit': (sum(1 for x in sel if abs(x[6]['I_rel']) >= LIMIT_I),
                      sum(1 for x in sel if abs(x[7]['I_rel']) >= LIMIT_I)),
            'I': (_extreme(a, 'I_rel'), _extreme(b, 'I_rel')),
            'rest': (_extreme(a, 'drift_rest', True), _extreme(b, 'drift_rest', True)),
            'pass': (sum(1 for x in sel if ok('SCHED', x)), sum(1 for x in sel if ok(CB, x))),
            'ends': {c: sum(1 for x in sel if end_class(x[6]) == c) for c in END_CLASSES}}


def _cb_verdicts(book):
    verdict = {}
    for kf in (1.0,) + K_ERRORS:
        for x in gate1(book, A1, ('SCHED', CB), lambda cfg, hold: matrix_sources(cfg, hold), kf=kf):
            verdict[(x['cfg'], x['u0'], x['stick'], x['hold'], x['policy'], x['src'], kf)] = x['verdict']
    return verdict


def cb_family_stats(book):
    """Per source family and speed band, SCHED against SCHED_CB* on the same gate legs of 2527 and 2522 (every
    stick and hold; the one-point family with its k errors): counts, the integrator at release, the release in
    all, gate 1's passes against HEAD, and where SCHED's interval sat at the stick's end."""
    cmp_ = cb_compare(book)
    verdict = _cb_verdicts(book)
    out = []
    for fam, srcs in CB_FAMILIES:
        for band, speeds in BANDS:
            sel = [x for x in cmp_ if x[1] in srcs and x[3] in speeds]
            if sel:
                out.append(dict(_cb_row(sel, verdict), family=fam, band=band))
    return out


def cb_class_stats(book, speeds=FWD):
    """{end class: SCHED -> SCHED_CB* statistics} over every source and k error of 2527 and 2522 at `speeds`,
    by where SCHED's interval sat at the stick's end."""
    cmp_ = [x for x in cb_compare(book) if x[3] in speeds]
    verdict = _cb_verdicts(book)
    return {c: _cb_row([x for x in cmp_ if end_class(x[6]) == c], verdict) for c in END_CLASSES
            if any(end_class(x[6]) == c for x in cmp_)}


def review_example(book, speeds=SPEEDS):
    """[(u0, SCHED, SCHED_CB*)] of the review's example: 2522 free surge, full stick 300 s, the 1.05 Pa band."""
    out = [(u, book.get('2522', 'SCHED', 'pitot_bias_ms', 1.0, u, 1.0, 300.0),
            book.get('2522', CB, 'pitot_bias_ms', 1.0, u, 1.0, 300.0, group='cbfree')) for u in speeds]
    return [x for x in out if x[1] is not None and x[2] is not None]


def review_example_text(book, speeds):
    """'from 4 m/s (ending at 1.72 m/s): I +0.210 -> +0.126, release +108/+134 -> +65/+80' per speed."""
    return '; '.join('from %g m/s (ending the stick at %.2f m/s): I %+.3f -> %+.3f, release %s -> %s'
                     % (u, a['u_stick_end'], a['I_rel'], b['I_rel'], drift_cell(a), drift_cell(b))
                     for u, a, b in review_example(book, speeds)) or 'n/r'


CLASS_TEXT = {'w_lo=1': 'u_lo past the ramp', 'lo_in_ramp': 'u_lo inside the ramp, u_hi past it',
              'lo_below': 'u_lo at or below AS_SPD_LO, u_hi past the ramp', 'w_hi<1': 'u_hi inside the ramp'}


def report_cb(book, say):
    say('\n=== The interval term and the ramp (review finding 1): SCHED against SCHED_CB* ===')
    say('The plan\'s interval term rho / (k (u_hi - u_lo)) assumes FF = FF0 + k u_lo; the ramp supplies FF0 + w(u_lo) '
        'k u_lo. With k right and the true speed u in the interval, a turn at sp leaves the integrator k (u - w(u_lo) '
        'u_lo) sp, at most k (u_hi - w(u_lo) u_lo) sp at u_hi, more than the rho the plan\'s term allows whenever '
        '0 < w(u_lo) < 1: the plan\'s promise needs u_lo past the ramp. SCHED_CB* writes the term on the FF supplied, '
        'rho / (k (u_hi - w(u_lo) u_lo)): its promise needs only u_hi past the ramp. The two terms are the same '
        'wherever w(u_lo) = 1 and wherever w(u_lo) = 0 (u_lo at or below AS_SPD_LO, k (u_hi - 0) either way): there '
        'both keep the integrator within rho, which on the A 1 plants is AS_YR_INT_LIM itself (finding 2). Where '
        'u_hi is inside the ramp the cap is blended back toward MAX by w(u_hi) and no interval term holds. Every '
        'SCHED leg of 2527 and 2522 against the same leg under SCHED_CB* (the gate\'s legs, every stick, hold, '
        'source and k error; not 2512, whose k from the fins parks the integrator whatever the term, nor the A 0.5 '
        'twins, where the allocator\'s knee stops it), by where SCHED\'s interval sat at the stick\'s end; "limit" '
        'the legs ending the stick with the integrator at AS_YR_INT_LIM, "passes" gate 1 against HEAD (merit or '
        'floor, a live stick):')
    verdict = None
    for band, speeds in BANDS:
        stats = cb_class_stats(book, speeds)
        rows = []
        for c in END_CLASSES:
            if c not in stats:
                continue
            st = stats[c]
            (ia, wa), (ib, wb) = st['I']
            (da, wda), (db, wdb) = st['rest']
            rows.append([CLASS_TEXT[c], st['n'], st['identical'], '%d / %d' % st['limit'],
                         '%s (%s) -> %s (%s)' % (fnum(ia, '%+.3f'), wa, fnum(ib, '%+.3f'), wb),
                         '%s -> %s' % (fnum(da, '%+.0f'), fnum(db, '%+.0f')), '%d / %d' % st['pass']])
        say('  %s:' % band)
        table(say_lines(say), ['interval at the stick\'s end (SCHED)', 'legs', 'identical', 'limit SCHED / CB',
                               'largest I at release SCHED -> CB', 'largest in all SCHED -> CB',
                               'gate-1 passes SCHED / CB'], rows)
    say('By source family (the same legs; "ends" counts SCHED\'s end states past / u_lo in the ramp / u_lo below / '
        'u_hi in the ramp):')
    rows = []
    for st in cb_family_stats(book):
        (ia, wa), (ib, wb) = st['I']
        (da, wda), (db, wdb) = st['rest']
        e = st['ends']
        rows.append([st['family'], st['band'], st['n'], st['identical'], '%d / %d' % st['limit'],
                     '%s -> %s' % (fnum(ia, '%+.3f'), fnum(ib, '%+.3f')),
                     '%s (%s) -> %s (%s)' % (fnum(da, '%+.0f'), wda, fnum(db, '%+.0f'), wdb),
                     '%d / %d' % st['pass'],
                     '%d / %d / %d / %d' % (e['w_lo=1'], e['lo_in_ramp'], e['lo_below'], e['w_hi<1'])])
    table(say_lines(say), ['source family', 'band', 'legs', 'identical', 'limit SCHED / CB',
                           'largest I at release SCHED -> CB', 'largest in all SCHED -> CB',
                           'gate-1 passes SCHED / CB', 'ends 1 / s / b / r'], rows)
    cmp_ = cb_compare(book)
    st = sorted([x for x in cmp_ if end_class(x[6]) == 'lo_in_ramp'], key=lambda x: -abs(x[6]['I_rel']))[:12]
    if st:
        say('The legs whose interval ended the stick with u_lo inside the ramp (where the two terms differ), the '
            'largest integrators first (I at release, release 120/rest, SCHED\'s interval at the stick\'s end with '
            'w(u_lo) / w(u_hi), u flown at its end):')
        rows = []
        for x in st:
            a, b = x[6], x[7]
            rows.append([_where(x[0], x[3], x[4], x[5], x[1], x[2], a.get('held')),
                         '%+.3f -> %+.3f' % (a['I_rel'], b['I_rel']), '%s -> %s' % (drift_cell(a), drift_cell(b)),
                         '[%.2f, %.2f] w %.2f/%.2f' % (a['u_lo_end'], a['u_hi_end'], w_of(a['u_lo_end']),
                                                       w_of(a['u_hi_end'])),
                         fnum(a['u_stick_end'], '%.2f')])
        table(say_lines(say), ['leg', 'I at release SCHED -> CB', 'release SCHED -> CB', 'interval (SCHED)', 'u'],
              rows)
    rev = [(u, book.get('2522', 'SCHED', 'pitot_bias_ms', 1.0, u, 1.0, 300.0),
            book.get('2522', CB, 'pitot_bias_ms', 1.0, u, 1.0, 300.0, group='cbfree')) for u in SPEEDS]
    rev = [x for x in rev if x[1] is not None and x[2] is not None]
    if rev:
        say('The review\'s own example, free surge (the record, not the gate\'s legs): 2522 gliding through a full stick '
            'held 300 s with the 1.05 Pa band, which slows the hull into the ramp so that u_lo falls inside it. SCHED '
            '-> SCHED_CB*: %s.'
            % '; '.join('%g m/s: interval at the stick\'s end [%.2f, %.2f] (w(u_lo) %.2f, u %.2f), I %+.3f -> %+.3f, '
                        'release %s -> %s' % (u, a['u_lo_end'], a['u_hi_end'], w_of(a['u_lo_end']), a['u_stick_end'],
                                              a['I_rel'], b['I_rel'], drift_cell(a), drift_cell(b))
                        for u, a, b in rev))
    # the blend weight, computed on the end states with u_lo inside the ramp: the steady integrator a turn at the
    # cap needs at the speed the hull flew, (D(u) - FF) cap, for the plan's term, the supplied-FF term, and both
    # weights w(u_lo)
    sel = [x for x in cmp_ if end_class(x[6]) == 'lo_in_ramp' and x[2] == 1.0]
    counts = {'plan': 0, 'supplied': 0, 'lo': 0}
    worst = {}
    n = 0
    for x in sel:
        cfg, a = x[0], x[6]
        c = config(cfg)
        g = c.gains
        d = d_at(hold_points(book, c.model), a['u_stick_end'])
        if d is None:
            continue
        n += 1
        rho = min(g['AS_YR_INT_LIM'], c.A / 2)
        for name, kw in (('plan', {}), ('supplied', {'budget': 'supplied'}), ('lo', {'budget': 'supplied', 'blend': 'lo'})):
            ff, cap, _ = P.schedule(a['u_lo_end'], a['u_hi_end'], c.k, g['AS_YAWRATE_FF'], g['AS_YAWRATE_MAX'],
                                    g['AS_YR_INT_LIM'], c.A, **kw)
            need = (d - ff) * math.radians(cap)
            counts[name] += need <= rho + 1e-9
            if name not in worst or need > worst[name][0]:
                worst[name] = (need, cap, _where(cfg, x[3], x[4], x[5], x[1], x[2], a.get('held')))
    if n:
        say('The blend weight, computed on those end states at nominal k (%d): the steady integrator a full-stick '
            'turn at the cap needs at the speed the hull flew, (D(u) - FF) cap, against rho. Within rho with the '
            'plan\'s term on %d, with the supplied-FF term on %d, and with the supplied term and the cap blended by '
            'w(u_lo) too (one weight, the review\'s other example) on %d. The largest need: %.3f (the plan\'s, cap '
            '%.2f deg/s, %s), %.3f (supplied, %.2f deg/s, %s), %.3f (w(u_lo) blend, %.2f deg/s, %s). Blending by '
            'w(u_lo) moves the cap toward MAX whenever u_lo is in the ramp; any blend toward MAX lifts the cap over '
            'the budget\'s r_lim, so no blend weight keeps the budget where u_lo or u_hi is in the ramp. (D is the '
            'plant\'s steady-turn torque per rad/s, a little over k u on these hulls: the k section.)'
            % (n, counts['plan'], counts['supplied'], counts['lo'], worst['plan'][0], worst['plan'][1],
               worst['plan'][2], worst['supplied'][0], worst['supplied'][1], worst['supplied'][2], worst['lo'][0],
               worst['lo'][1], worst['lo'][2]))


def report_ff_errors(book, say):
    say('\n=== FF errors (gate 3): the drift a wrong k leaves, the perfect source (the gate\'s legs) ===')
    say('k x 0.7 / 1.0 / 1.3 / 2.0 (FF and the cap both built on the wrong k); cells 120/rest deg; HEAD beside it; '
        '"H" a held leg:')

    for cfg in GATED:
        pols = sched_pols(cfg)
        rows = []

        for u0 in SPEEDS:
            for stick, hold in CASES:
                head = book.gate(cfg, 'HEAD', u0=u0, stick=stick, hold=hold)
                row = ['%g' % u0, case_name(stick, hold) + (' H' if held_cell(cfg, hold) else ''), drift_cell(head)]

                for policy in pols:
                    row.append(' '.join(drift_cell(book.gate(cfg, policy, 'perfect', kf, u0, stick, hold))
                                        for kf in (0.7, 1.0, 1.3, 2.0)))
                rows.append(row)

        say('%s:' % cfg)
        table(say_lines(say), ['u', 'stick', 'HEAD'] + ['%s x0.7 x1 x1.3 x2' % SHORT[p] for p in pols], rows)


def report_khold(book, say):
    say('\n=== 2512 with k its measured steady-turn slope (k = %s; the gate\'s legs, the 300 s rows held) ==='
        % fnum(k_hold_of(book.all), '%.3f'))
    say('Release drift 120/rest and the stick (r last 5 s / sp, I at release), sources perfect and ground W2 head, '
        'k x 0.7 / 1 / 1.3 / 2 on the perfect source:')
    rows = []

    for u0 in SPEEDS:
        for stick, hold in CASES:
            head = book.gate('2512', 'HEAD', u0=u0, stick=stick, hold=hold)
            row = ['%g' % u0, case_name(stick, hold) + (' H' if held_cell('2512/khold', hold) else ''), drift_cell(head)]

            for policy in SPEC_SCHED:
                r = book.gate('2512/khold', policy, 'perfect', 1.0, u0, stick, hold)
                w = book.gate('2512/khold', policy, 'ground_W2_head', 1.0, u0, stick, hold)
                errs = ' '.join(drift_cell(book.gate('2512/khold', policy, 'perfect', kf, u0, stick, hold))
                                for kf in (0.7, 1.3, 2.0))
                row.append('%s (%s) W2h %s; x0.7/1.3/2 %s' % (drift_cell(r), stick_cell(r), drift_cell(w), errs))
            rows.append(row)

    table(say_lines(say), ['u', 'stick', 'HEAD'] + [SHORT[p] for p in SPEC_SCHED], rows)
    free = [book.get('2512/khold', 'SCHED', 'perfect', 1.0, u, 1.0, 300.0, group='khold') for u in SPEEDS]
    if all(free):
        say('Free surge, 300 s, SCHED (the record): u at the stick\'s end %s m/s from 2 / 2.5 / 4 / 6 m/s, releases %s'
            % (' / '.join('%.2f' % r['u_stick_end'] for r in free), ' / '.join(drift_cell(r) for r in free)))


def report_edge(book, say):
    say('\n=== The stall edge at 2 m/s (review finding 12) ===')
    c = config('2527').stall
    say('A full stick for 20 s entered at 1.8..2.4 m/s, the surge free and held at the entry speed. The fins stall '
        'once r arm / u passes alpha_stall: the stall rate alpha_stall u / arm is %.3f deg/s per m/s on these hulls '
        '(the dorsal fin). Cells: fins max |alpha| / alpha_stall in the stick (S %% of it stalled), r peak deg/s, u at '
        'the stick\'s end, release 120/rest:' % math.degrees(c['c'] / c['margin']))
    rows = []
    for cfg in PRIMARY:
        for policy, src in (('HEAD', 'none'), ('SCHED', 'perfect')):
            for u0 in EDGE_SPEEDS:
                row = [cfg, policy, '%g' % u0, '%.2f' % math.degrees(c['c'] / c['margin'] * u0)]
                cap = None
                for held in (False, True):
                    r = book.get(cfg, policy, src, 1.0, u0, 1.0, 20.0, group='edge', held=held)
                    if r is None:
                        row.append('n/r')
                        continue
                    if cap is None:
                        cap = (r.get('hook') or {}).get('at', {}).get('stick', [None] * 6)[5] if r.get('hook') else \
                            r['sp_end']
                    st = r['stalled_stick'] or 0.0
                    row.append('%.2f%s, r %.1f, u %.2f, %s' % (r['fin_stick'] or 0.0, ' S%.0f%%' % (100 * st) if st else '',
                                                               r['r_peak'], r['u_stick_end'], drift_cell(r)))
                row.insert(4, fnum(cap, '%.2f'))
                rows.append(row)
    table(say_lines(say), ['plant', 'policy', 'u0', 'stall rate deg/s', 'cap at onset', 'free surge', 'surge held'],
          rows)
    for cfg in PRIMARY:
        parts = []
        for policy, src in (('HEAD', 'none'), ('SCHED', 'perfect')):
            r = book.get(cfg, policy, src, 1.0, 2.0, 1.0, 300.0, group='edge', held=True)
            if r:
                parts.append('%s fins %.2f x (S %.0f %%), r %.1f deg/s in the last 5 s, release %s'
                             % (policy, r['fin_stick'] or 0.0, 100 * (r['stalled_stick'] or 0.0), r['r_last5'],
                                drift_cell(r)))
        if parts:
            say('  %s, full stick 300 s with the surge held at 2.0 m/s: %s' % (cfg, '; '.join(parts)))


def trim_of(book, cfg, u):
    r = book.get(cfg, 'HEAD', u0=u)
    return r['trim'] if r else None


_ALLOC = {}


def alloc_delivery(tilt_range, thrust_x, tau, seconds=3.0):
    """The vectored allocator alone (vectored_actuation.Allocator, the firmware's port) for a constant yaw command
    tau and forward thrust demand thrust_x, run until the pods settle: the delivered yaw and surge (normalized,
    pod_wrench), the pod tilts (deg) and thrusts, the yaw shortfall's sign."""
    import vectored_actuation as va
    a = ctl.AIRFRAMES['vectored']
    p = {'tilt_min': va.radians_f(tilt_range[0]), 'tilt_max': va.radians_f(tilt_range[1]),
         'slew': va.radians_f(a['CA_AIRSHIP_TLT_R'])}
    al = va.Allocator(p)
    thr = [0.0, 0.0]
    for _ in range(int(round(seconds / DT))):
        _, thr = al.update((0.0, 0.0, tau), (thrust_x, 0.0, 0.0), DT)
    ach = [(math.cos(pod.tilt) * t, math.sin(pod.tilt) * t) for pod, t in zip(al.pods, thr)]
    w = va.pod_wrench(ach[0], ach[1])
    return {'yaw': w[2], 'surge': w[3], 'tilts': [math.degrees(pod.tilt) for pod in al.pods], 'thrust': list(thr),
            'unalloc': al.unallocated[2]}


ALLOC_TAUS = (0.1, 0.25, 0.5, 0.75, 1.0)


def alloc_table():
    """{tilt range: {(thrust, tau): alloc_delivery}} for the trims of 2527 at the 0c speeds and 0 (the gliders)."""
    if _ALLOC:
        return _ALLOC
    import vectored_actuation as va  # noqa: F401
    trims = sorted({round(F.trim_throttle(config('2527').model, u)[0], 4) for u in SPEEDS} | {0.0})
    for rng in ((-180.0, 180.0), (-90.0, 90.0)):
        _ALLOC[rng] = {(t, tau): alloc_delivery(rng, t, tau) for t in trims for tau in ALLOC_TAUS}
    return _ALLOC


def report_a05(book, say):
    say('\n=== What the pods deliver: the 1:1 knee (review finding 8, and the same on +-180 deg) ===')
    tab = alloc_table()
    say('The allocator alone (vectored_actuation.Allocator, the firmware\'s ActuatorEffectivenessAirship port), a '
        'constant yaw command tau against a forward thrust demand T (the trims of 2527 at 2 / 2.5 / 4 / 6 m/s, and 0, '
        'the gliders), pods settled: delivered yaw / surge, normalized. Each pod is asked for T -+ tau. A pod at '
        '+-90 deg cannot take a negative share, so past tau = T it is clipped at 0 and the other carries T + tau: the '
        'yaw is (T + tau) / 2, half gain, with the surge (T + tau) / 2 instead of T. Over +-180 deg the backward pod '
        'swings to 180 deg and takes its share, but the forward pod saturates at full thrust past tau = 1 - T: the '
        'yaw then grows at half gain while the surge drops. Past either knee the allocator flags the shortfall. (A '
        'pod asked for less than its %g engage threshold does not steer: the small-tau cells at small T.)'
        % __import__('vectored_actuation').Pod.ENGAGE)
    trims = sorted({t for (t, _) in tab[(-90.0, 90.0)]})
    rows = []
    for rng in ((-90.0, 90.0), (-180.0, 180.0)):
        for t in trims:
            rows.append(['+-%g' % rng[1], '%.4f' % t, '%.4f' % (t if rng[1] == 90.0 else 1.0 - t)] +
                        ['%.3f / %.3f%s' % (tab[rng][(t, tau)]['yaw'], tab[rng][(t, tau)]['surge'],
                                            ' F' if tab[rng][(t, tau)]['unalloc'] else '') for tau in ALLOC_TAUS])
    table(say_lines(say), ['tilt range', 'T', 'knee'] + ['tau %g' % tau for tau in ALLOC_TAUS], rows)
    t6 = trim_of(book, '2527', 6.0)
    say('(F: the yaw shortfall flagged.) So the yaw delivered 1:1 is T with pods that cannot reverse and 1 - T with '
        'pods that can, not the A = 0.5 (1 + rho_rev) of the plan\'s 2.나, which leaves out the collective thrust\'s '
        'share of the pods. With pods that cannot reverse, A = 0.5 is the peak (one pod at full thrust) with surge '
        '0.5, and beyond T a unit of yaw costs half a unit of extra forward thrust; on a glider (T = 0) every unit. '
        'On +-180 deg the knee matters at speed: at 2527\'s 6 m/s trim %.3f it is %.3f, and a turn at the schedule\'s '
        'cap (A - rho) / FF already asks about A - rho = %g of it with k right, so the integrator has about %.3f '
        'before the flag, not rho (the time-constant section: k x0.7 there is flagged, not wound). Flown, free '
        'surge (the record), the speed over the stick:'
        % (t6 or 0.0, 1.0 - (t6 or 0.0), a_less_rho('2527'), 1.0 - (t6 or 0.0) - a_less_rho('2527')))
    rows = []
    for cfg in A05:
        for stick, hold in ((1.0, 20.0), (1.0, 300.0)):
            row = [cfg, case_name(stick, hold)]
            for u0 in SPEEDS:
                r = book.get(cfg, 'HEAD', u0=u0, stick=stick, hold=hold)
                row.append('n/r' if r is None else '%.2f -> %.2f' % (r['u_stick_start'], r['u_stick_end']))
            rows.append(row)
    table(say_lines(say), ['plant', 'HEAD'] + ['%g m/s' % u for u in SPEEDS], rows)
    hd = _rows(book, A05, ('HEAD',))
    if hd:
        i = _span(hd, 'I_rel')
        d, w = _extreme(hd, 'drift_120')
        wound = [x for x in hd if abs(x[7]['I_rel']) > 0.01]
        say('With the surge held (the gate\'s legs), HEAD\'s integrator at release is %s..%s and its release at most %s '
            'deg in 120 s (%s)%s.'
            % (fnum(i[0], '%+.3f'), fnum(i[1], '%+.3f'), fnum(d, '%+.1f'), w,
               ('; parked where the torque stays under the thrust (nothing flagged): %s'
                % '; '.join('%s I %+.3f, %s' % (_where(x[0], x[4], x[5], x[6], held=True), x[7]['I_rel'],
                                                drift_cell(x[7])) for x in wound)) if wound else
               '; nowhere does it park: every full or half stick asks one pod to push backward and the flag stops the '
               'integrator'))


def report_dist(book, say):
    say('\n=== The restore rule\'s cost: a yaw couple that changes during the turn (not a 0c condition) ===')
    say('A couple of %g of the yaw authority (the gate\'s world-z couple, nose right: aiding the turn), from the Acro '
        'switch, at 4 m/s, full stick; constant (the integrator starts trimmed against it), gone at the stick\'s onset '
        '(trimmed, then nothing to trim), or arriving at it (untrimmed). Release 120 s/rest deg, "to rest" with the '
        'integrator ending at the trim the couple then needs; the integrator at release; the trim the rule put back. '
        'The 2522 300 s rows fly the surge held (H):'
        % DIST_SHARE)
    rows = []

    for cfg in PRIMARY:
        for variant in DIST_VARIANTS:
            for hold in (20.0, 300.0):
                held = held_cell(cfg, hold)
                row = [cfg, variant, '%g s%s' % (hold, ' H' if held else '')]
                for policy in ('HEAD', 'RESTORE', 'SCHED', 'SCHED+RESTORE'):
                    src = 'perfect' if policy.startswith('SCHED') else 'none'
                    r = book.get(cfg, policy, src, 1.0, 4.0, 1.0, hold, group='dist', variant=variant, held=held)
                    if r is None:
                        row.append('n/r')
                        continue
                    row.append('%s I %+.3f%s' % (drift_cell(r), r['I_rel'], (' put back %+.3f' % r['restore_value'])
                                                  if r['restore_value'] is not None else ''))
                rows.append(row)

    table(say_lines(say), ['plant', 'couple', 'hold', 'HEAD', 'RESTORE', 'SCHED', 'SCHED+RESTORE'], rows)


def report_2517(book, say):
    say('\n=== 2517 (not gated): what each policy does to the unstable plant ===')
    rows = []

    for u0 in SPEEDS:
        for stick, hold in CASES:
            for policy in P.SPEC_POLICIES:
                r = book.get('2517', policy, 'perfect', 1.0, u0, stick, hold)
                if r is None:
                    continue
                rows.append(['%g' % u0, case_name(stick, hold), SHORT[policy],
                             '%s/%s' % (fnum(r['r_last5'], '%.1f'), fnum(r['sp_end'], '%.2f')),
                             fnum(r['r_peak'], '%.1f'), fnum(r['drift_120'], '%+.0f'),
                             '%s / %s' % (fnum(r['r_60'], '%+.2f'), fnum(r['r_120'], '%+.2f')),
                             '%s/%s' % (fnum(r['fin_stick'], '%.2f'), fnum(r['fin_release'], '%.2f')),
                             '%.0f/%.0f' % (100 * (r['stalled_stick'] or 0), 100 * (r['stalled_release'] or 0)),
                             '%s/%s' % (r['rev_stick'], r['rev_release']),
                             '%s -> %s' % (fnum(r['u_stick_end'], '%.2f'), fnum(r['u_release_end'], '%.2f'))])
    table(say_lines(say), ['u', 'stick', 'policy', 'r last 5 s / sp', 'r peak', 'heading 120 s', 'r +60/+120',
                           'fins stick/release', '% stalled stick/release', 'tail flips', 'u stick end -> rel end'],
          rows)


def report_noise(book, say):
    say('\n=== gz gyro noise (three seeds) at 4 m/s, full stick 20 s, the perfect source ===')
    rows = []

    for cfg in ('2527', '2512'):
        for policy in MATRIX_POLICIES:
            src = 'perfect' if policy.startswith('SCHED') else 'none'
            clean = book.get(cfg, policy, src, 1.0, 4.0, 1.0, 20.0)
            noisy = [book.get(cfg, policy, src, 1.0, 4.0, 1.0, 20.0, group='noise', seed=s) for s in (1, 2, 3)]
            noisy = [n for n in noisy if n is not None]
            if not noisy or clean is None:
                continue
            spread = lambda key: '%s..%s' % (fnum(min(n[key] for n in noisy if n[key] is not None), '%+.2f'),
                                             fnum(max(n[key] for n in noisy if n[key] is not None), '%+.2f'))
            rows.append([cfg, SHORT[policy], '%s (%s)' % (fnum(clean['drift_120'], '%+.2f'), spread('drift_120')),
                         '%s (%s)' % (fnum(clean['r_last5'], '%.3f'), spread('r_last5')),
                         '%s (%s)' % (fnum(clean['I_rel'], '%+.4f'), spread('I_rel')),
                         '%s/%s (%s)' % (clean['rev_stick'], clean['rev_release'],
                                         ', '.join('%s/%s' % (n['rev_stick'], n['rev_release']) for n in noisy)),
                         '%s (%s)' % (fnum(clean['t_settle'], '%.1f'), ', '.join(fnum(n['t_settle'], '%.1f')
                                                                                  for n in noisy))])
    table(say_lines(say), ['plant', 'policy', 'drift 120 s clean (seeds)', 'r last 5 s', 'I at release',
                           'reversals clean (seeds)', 'settle s clean (seeds)'], rows)


def report_gates(book, say):
    say('\n=== Gates ===')
    out = {}
    pols = ('RESTORE',) + SCHED_POLICIES + (CB,)
    results = gate1(book, GATED, pols, sources_for)
    out['g1'] = results
    say('(1) the post-release drift smaller than HEAD\'s on the same plant, speed, stick and hold, for every source '
        '(nominal k), 120 s and to rest, on the gate\'s legs (held where the label needs it). A release within %g deg '
        'counts as no drift (HEAD itself does not drift where the allocator flags the shortfall, the A 0.5 plants: %d '
        'of the HEAD legs there release under %g deg), but only with a live stick: a stick that turned the hull under '
        '%g %% of HEAD\'s heading change in the stick did essentially nothing (the zero cap of SCHED_STALL, a few '
        'ground-only cases whose stick barely turned the hull) and is a dead stick, no pass whatever its release. Counted '
        'apart: smaller than HEAD on merit, passing only by the floor, dead, not smaller; of the passes, those judged '
        'on 120 s alone ("to rest" undecided) and those whose stick turned the hull under %g %% of HEAD\'s (slow, '
        'mostly ground speed in a wind):'
        % (DRIFT_FLOOR, sum(1 for x in results if x['policy'] == 'RESTORE' and x['cfg'] in A05
                            and abs(x['head']['drift_120']) <= DRIFT_FLOOR), DRIFT_FLOOR, 100 * DEAD_SHARE,
           100 * SLOW_SHARE))

    for policy in pols:
        rs = [x for x in results if x['policy'] == policy]
        if not rs:
            continue
        c = gate1_counts(rs)
        und = [x for x in rs if x['verdict'] in ('smaller', 'floor') and not x['decided']]
        why = sorted({w.strip() for x in und for w in (x['r']['rest_why'] + ';' + x['head']['rest_why']).split(';')
                      if w.strip()})
        extra = ''
        if policy in RESTORERS:
            tup = lambda x: (x['cfg'], policy, x['src'], 1.0, x['u0'], x['stick'], x['hold'], x['r'])
            put = [x for x in rs if x['r']['restore_events']]
            v, w = _extreme_of([tup(x) for x in put], excursion)
            a, wa = _extreme([tup(x) for x in put if x['u0'] in FWD], 'drift_rest', True)
            a2, wa2 = _extreme([tup(x) for x in put if x['u0'] not in FWD], 'drift_rest', True)
            extra = ('; on the %d legs with a put-back "in all" is the put-back over K_I wherever no flag clips the '
                     'release: at most %s deg at 2.5-6 m/s (%s), %s deg at 2 m/s (%s); their largest excursion in '
                     '120 s %s deg (%s)%s'
                     % (len(put), fnum(a, '%+.1f'), wa, fnum(a2, '%+.1f'), wa2, fnum(v, '%+.1f'), w,
                        ('; %d legs without one (the forward gate dropped it), which release as SCHED'
                         % (len(rs) - len(put))) if len(put) < len(rs) else ''))
        say('  %-18s %d comparisons: %d smaller on merit, %d by the %g deg floor, %d dead sticks, %d not smaller; of '
            'the %d passes %d judged on 120 s alone%s and %d with a slow stick%s'
            % (SHORT[policy], len(rs), c['smaller'], c['floor'], DRIFT_FLOOR, c['dead'], c['not'], c['pass'],
               c['undecided'], (' (%s)' % '; '.join(why)) if why else '', c['slow'], extra))

        for kind, label in (('not', 'not smaller'), ('dead', 'dead stick')):
            bad = [x for x in rs if x['verdict'] == kind]
            for x in bad[:30]:
                r, h = x['r'], x['head']
                say('      %-11s %-10s %g m/s %-10s %-15s %s against HEAD %s; u at the stick\'s end %s%s; heading in '
                    'the stick %+.1f against HEAD %+.1f'
                    % (label, x['cfg'], x['u0'], case_name(x['stick'], x['hold']), x['src'], drift_cell(r),
                       drift_cell(h), fnum(r['u_stick_end'], '%.2f'), ' (held)' if x['held'] else '',
                       r['heading_stick'] or 0.0, h['heading_stick'] or 0.0))
            if len(bad) > 30:
                say('      ... and %d more %s' % (len(bad) - 30, label))

    say('  with an FF error (the perfect source), against HEAD, as a record (gate 3 is the drift itself): passes '
        '(merit + floor) of comparisons')

    for kf in K_ERRORS:
        parts = []
        for policy in SCHED_POLICIES + (CB,):
            rs = gate1(book, GATED, (policy,), lambda cfg, hold: ['perfect'], kf=kf)
            if rs:
                c = gate1_counts(rs)
                parts.append('%s %d/%d%s' % (SHORT[policy], c['pass'], len(rs), (' (%d dead)' % c['dead'])
                                             if c['dead'] else ''))
        say('    k x%g: %s' % (kf, ', '.join(parts)))

    say('(2) at rest (u 0, still air), every policy and source against HEAD, trace for trace (metrics, entry states, '
        'series and every 4 ms step; full and half stick). SCHED_CB* is SCHED at rest wherever u_hi stays under '
        'AS_SPD_LO (w(u_hi) = 0: the cap is MAX and FF is FF0 whatever the term), and with u_lo = 0 its term is the '
        'plan\'s; it is not flown at rest:')
    rest = {}

    for key, rec in book.legs.items():
        if key[0] != 'rest':
            continue
        rest.setdefault((key[2], key[3]), []).append((key[1], key[6], rec['result']))

    for (policy, src), items in sorted(rest.items(), key=lambda kv: (P.POLICIES.index(kv[0][0]),
                                                                     list(P.SOURCES).index(kv[0][1])
                                                                     if kv[0][1] in P.SOURCES else -1)):
        ident = [i for i in items if i[2]['identical']]
        note = ''
        if len(ident) < len(items):
            worst = max((i for i in items if not i[2]['identical']),
                        key=lambda i: i[2]['diffs']['heading']['max'])
            r = worst[2]
            note = ('; e.g. %s stick %g: r last 5 s %.2f against HEAD %.2f deg/s, heading differs by up to %.1f deg '
                    'from t %s s' % (worst[0], worst[1], r['r_last5'][1], r['r_last5'][0], r['diffs']['heading']['max'],
                                     fnum(r['diffs']['heading']['first_t'] or r['diffs']['torque']['first_t'], '%.3f')))
        say('  %-18s %-15s identical on %d of %d%s' % (SHORT[policy], src, len(ident), len(items), note))

    say('  G1, the 48 at-rest gate verdicts (reproduce_head, today\'s sitl_check through gate.py) with each policy on '
        'every gate loop (the hook before each controller cycle; the source calm), and each target\'s gate lines '
        'and samples against HEAD\'s (NaN equal to NaN); flown again with the revised hook (per-direction A, the '
        'budget option):')
    head = book.g1.get(('HEAD', None))

    for (policy, src), rec in book.g1.items():
        if 'error' in rec:
            say('    %s %s: %s' % (policy, src, rec['error']))
            continue
        same = []
        for model, t in rec['targets'].items():
            h = head['targets'].get(model) if head else None
            if h is not None:
                same.append('%s %s' % (model, 'identical' if (_same(t['gate_lines'], h['gate_lines'])
                                                               and _same(t['samples'], h['samples'])) else
                                       ('lines equal, samples differ' if _same(t['gate_lines'], h['gate_lines'])
                                        else 'differs')))
        fails = ['%s %s' % (row[0].replace('gz_', ''), row[1]) for row in rec['table'] if not row[3]]
        say('    %-18s %-15s %d / %d%s; %s' % (SHORT[policy], src or '-', rec['agree'], rec['total'],
                                               (' (disagree: %s)' % ', '.join(fails)) if fails else '',
                                               ', '.join(same)))
        out[('g1', policy, src)] = rec

    say('(3) the drift each FF error leaves (the perfect source, the gate\'s legs): the largest |release| in 120 s / '
        'in all per policy, plant and k error over the sticks and holds, with where; at 2.5-6 m/s (the schedule\'s '
        'band) and at 2 m/s (inside the ramp, the fins\' stall boundary) apart:')

    for label, speeds in (('2.5-6 m/s', (2.5, 4.0, 6.0)), ('2 m/s', (2.0,))):
        rows = []

        for cfg in GATED:
            for policy in ('HEAD',) + sched_pols(cfg):
                row = [cfg, SHORT[policy]]
                for kf in ((1.0,) if policy == 'HEAD' else (0.7, 1.0, 1.3, 2.0)):
                    rs = [(u0, stick, hold, book.gate(cfg, policy, 'perfect', kf, u0, stick, hold))
                          for u0 in speeds for stick, hold in CASES]
                    rs = [x for x in rs if x[3] is not None]
                    if not rs:
                        row.append('n/r')
                        continue
                    w120 = max(rs, key=lambda x: abs(x[3]['drift_120']))
                    valid = [x for x in rs if x[3]['rest_valid']]
                    wrest = max(valid, key=lambda x: abs(x[3]['drift_rest'])) if valid else None
                    hm = lambda x: ' H' if held_cell(cfg, x[2]) else ''
                    row.append('%+.0f (%g %s%s) / %s' % (w120[3]['drift_120'], w120[0], case_name(w120[1], w120[2]),
                                                         hm(w120), ('%+.0f (%g %s%s)' % (
                                                             wrest[3]['drift_rest'], wrest[0],
                                                             case_name(wrest[1], wrest[2]), hm(wrest))) if wrest else '?'))
                if policy == 'HEAD':
                    row += ['', '', '']
                rows.append(row)

        say('  %s:' % label)
        table(say_lines(say), ['plant', 'policy', 'k x0.7 (HEAD: as flown)', 'k x1', 'k x1.3', 'k x2'], rows)

    return out


def report_findings(book, say, g):
    say('\n=== Findings ===')
    say('(every number is read from the records; "where" names the plant, speed and stick, and the source or k error '
        'when not the perfect source at k x1, and "held" for a leg with the surge held. "2.5-6 m/s" is the '
        'schedule\'s band: w(u) = 1 at the entry speed; 2 m/s is inside the ramp. The gate\'s legs throughout: free '
        'surge, held on the gliders\' 300 s holds and on the A 0.5 twins)')
    n = [0]

    def item(text):
        n[0] += 1
        say('%d. %s' % (n[0], text))

    allsrc = tuple(P.SOURCES)
    head = _rows(book, A1, ('HEAD',))
    full = [x for x in head if x[5] == 1.0]
    lo_i, hi_i = _span(full, 'I_rel')
    d120 = _span(full, 'drift_120')
    drest = _span([x for x in full if x[7]['rest_valid']], 'drift_rest')
    t_est = _span([x for x in full if x[7]['t_settle_est']], 't_settle_est')
    item('The defect at HEAD, the reference every gate is measured against. On 2527 and 2522 (A 1) a full stick parks '
         'the integrator at %s..%s with no flag at every speed (the %g deg/s setpoint is out of reach), and after the '
         'release the hull turns on by %s..%s deg in 120 s and %s..%s deg in all (I / K_I = %g / %g rad), |r| '
         'falling under %g deg/s only after ~%s..%s s.'
         % (fnum(lo_i, '%.3f'), fnum(hi_i, '%.3f'), config('2527').gains['AS_YAWRATE_MAX'], fnum(d120[0], '%.0f'),
            fnum(d120[1], '%.0f'), fnum(drest[0], '%.0f'), fnum(drest[1], '%.0f'),
            config('2527').gains['AS_YR_INT_LIM'], config('2527').gains['AS_YAWRATE_I'], SETTLE_RATE,
            fnum(t_est[0], '%.0f'), fnum(t_est[1], '%.0f')))

    fwd = _rows(book, A1, ('SCHED',), sources=('perfect', 'ground_W0'), speeds=FWD)
    if fwd:
        f20 = [x for x in fwd if x[6] == 20.0]
        v, w = _extreme(f20, 'drift_120')
        vr, wr = _extreme(f20, 'drift_rest', True)
        vi, wi = _extreme(f20, 'I_rel')
        ratio = [x[7]['r_last5'] / x[7]['sp_end'] for x in fwd if x[7]['sp_end']]
        st = [x[7]['t_settle'] for x in f20 if x[7]['t_settle'] is not None]
        l300 = [x for x in fwd if x[6] == 300.0]
        vl, wl = _extreme(l300, 'drift_rest', True)
        item('SCHED removes the defect at 2.5-6 m/s when the speed is known (the perfect source and the indoor ground '
             'source W 0, which agree): the turn holds its cap (r over the stick\'s last 5 s at %.2f..%.2f of the '
             'setpoint), the integrator at release is at most %+.4f (%s), the release turns at most %+.1f deg in 120 s '
             '(%s) and %s deg in all (%s), and |r| is under %g deg/s within %.1f s. After a 300 s hold the release is '
             'at most %s deg in all (%s): the FF the schedule gives is %s..%s %% of the plants\' steady-turn torque '
             '(the k section), and the integrator takes the rest over minutes. The price is the turn rate: the cap is '
             'what the authority reaches with rho in reserve (full stick, 20 s, 2.5 / 4 / 6 m/s: %s), where HEAD '
             'turns at %s with its integrator parked.'
             % (min(ratio), max(ratio), vi, wi, v, w, fnum(vr, '%+.1f'), wr, SETTLE_RATE, max(st) if st else -1,
                fnum(vl, '%+.0f'), wl, *ff_share(book),
                '; '.join('%s cap %s deg/s' % (c, ' / '.join(fnum((book.gate(c, 'SCHED', 'perfect', 1.0, u, 1.0, 20.0)
                                                                   or {}).get('sp_end'), '%.2f') for u in FWD))
                          for c in A1),
                '; '.join('%s %s deg/s' % (c, ' / '.join(fnum((book.gate(c, 'HEAD', u0=u) or {}).get('r_last5'), '%.2f')
                                                         for u in FWD)) for c in A1)))

    lines = []
    for band, speeds in BANDS:
        for c, st in cb_class_stats(book, speeds).items():
            (ia, wa), (ib, wb) = st['I']
            (da, wda), (db, wdb) = st['rest']
            lines.append('%s, %s: %d legs (%d identical), at the limit %d -> %d, the largest integrator %s (%s) -> %s '
                         '(%s), the largest release in all %s -> %s deg, gate-1 passes %d -> %d'
                         % (band, CLASS_TEXT[c], st['n'], st['identical'], st['limit'][0], st['limit'][1],
                            fnum(ia, '%+.3f'), wa, fnum(ib, '%+.3f'), wb, fnum(da, '%+.0f'), fnum(db, '%+.0f'),
                            st['pass'][0], st['pass'][1]))
    g1cb = gate1(book, GATED, ('SCHED', CB), sources_for)
    dead_s = {(x['cfg'], x['u0'], x['stick'], x['hold'], x['src']) for x in g1cb
              if x['policy'] == 'SCHED' and x['verdict'] == 'dead'}
    dead_cb = {(x['cfg'], x['u0'], x['stick'], x['hold'], x['src']) for x in g1cb
               if x['policy'] == CB and x['verdict'] == 'dead'}
    if lines:
        item('The plan\'s guarantee and the ramp (review finding 1). The schedule promises that with k right the '
             'integrator stays within rho at any true speed in the interval. Its interval term rho / (k (u_hi - '
             'u_lo)) assumes FF = FF0 + k u_lo, but the ramp supplies FF0 + w(u_lo) k u_lo, and the cap is re-opened '
             'toward MAX by w(u_hi), not w(u_lo): the promise holds only with u_lo past the ramp (u_lo >= AS_SPD_HI). '
             'SCHED_CB* (an addition) writes the term on the FF supplied, rho / (k (u_hi - w(u_lo) u_lo)), whose '
             'promise needs only u_hi past the ramp. Every SCHED leg of 2527 and 2522 against the same leg under '
             'SCHED_CB* (the gate\'s legs, every source and k error), by where SCHED\'s interval sat at the stick\'s '
             'end: %s. The review\'s own example, 2522 gliding with the 1.05 Pa band (free surge, the record): %s. So '
             'the failures split three ways. Where the interval straddles the ramp (u_lo inside it, u_hi past it) '
             'they are the formula\'s inconsistent weights, and the supplied-FF term lowers them; it cannot hold a '
             'budget the blend has already given up, so where the hull itself slows into the ramp (the gliders from 2 '
             'and 2.5 m/s, and every 2 m/s leg) u_hi follows it in and neither term helps (item 5). Where u_lo is at '
             'or below AS_SPD_LO (ground speed only, the wind from ahead) the two terms are the same and keep their '
             'budget, rho, which on A 1 is AS_YR_INT_LIM: those failures are finding 2\'s, not the weights\'. The '
             'supplied-FF term\'s price is a tighter cap where u_lo is in the ramp: gate 1 counts %d dead sticks '
             'under SCHED_CB* against %d under SCHED (%s). The source families are in the interval-term section.'
             % ('; '.join(lines), review_example_text(book, SPEEDS), len(dead_cb), len(dead_s),
                '; '.join('%s %g m/s %s %s' % (c, u, case_name(st, h), src) for c, u, st, h, src in sorted(dead_cb - dead_s))
                or 'the same legs'))

    sc = _rows(book, A1, ('SCHED', CB), sources=allsrc, kfs=(1.0,) + K_ERRORS, speeds=SPEEDS)
    if sc:
        parked = [x for x in sc if abs(x[7]['I_rel']) >= 0.299]
        cnt = {p: sum(1 for x in parked if x[1] == p) for p in ('SCHED', CB)}
        tot = {p: sum(1 for x in sc if x[1] == p) for p in ('SCHED', CB)}
        nk = [x for x in parked if x[3] == 1.0]
        item('rho on the A 1 plants (review finding 2). rho = min(AS_YR_INT_LIM %g, A / 2 %g) = %g = AS_YR_INT_LIM on '
             '2527, 2522 and the Cloudships, so "the integrator at or below rho" includes the parked state with no '
             'flag, the defect itself, and the schedule alone bounds the release only by rho / K_I = %.0f deg, '
             'HEAD\'s bound. The interval term is built to let a wide but correct interval use all of rho. Flown '
             '(2527 and 2522, every source and k error): SCHED ends the stick at the limit on %d of %d legs, '
             'SCHED_CB* on %d of %d; at nominal k %s. Only rho a strict fraction of AS_YR_INT_LIM (rho = m min(INT_LIM, '
             'A / 2), m < 1) bounds the release below %.0f deg, by m x %.0f deg; a formula question for the owner, '
             'beside the restore rule (For the owner, 3).'
             % (config('2527').gains['AS_YR_INT_LIM'], config('2527').A / 2, min(config('2527').gains['AS_YR_INT_LIM'],
                                                                              config('2527').A / 2),
                lim_drift(), cnt['SCHED'], tot['SCHED'], cnt[CB], tot[CB],
                '; '.join('%s %s' % (SHORT[x[1]], _where(x[0], x[4], x[5], x[6], x[2], x[3], x[7].get('held')))
                          for x in nk[:8]) or 'none', lim_drift(), lim_drift()))

    e = {(cfg, p, u, h): book.get(cfg, p, s, 1.0, u, 1.0, 20.0, group='edge', held=h)
         for cfg in PRIMARY for p, s in (('HEAD', 'none'), ('SCHED', 'perfect')) for u in EDGE_SPEEDS for h in (False, True)}
    if all(v is not None for v in e.values()):
        stall_rate = lambda u: math.degrees(config('2527').stall['c'] / P.STALL_MARGIN * u)
        long_h = {p: book.get('2527', p, s, 1.0, 2.0, 1.0, 300.0, group='edge', held=True)
                  for p, s in (('HEAD', 'none'), ('SCHED', 'perfect'))}
        dep = lambda cfg, p, h: ', '.join('%g' % u for u in edge_departs(book, cfg, p, h))
        two = [x for x in _rows(book, ('2527',), ('SCHED',), speeds=(2.0,)) if x[5] == 1.0 and x[6] == 20.0]
        onset = two[0][7]['hook']['at']['stick'][5] if two and two[0][7].get('hook') else None
        item('2 m/s is the fins\' stall edge, for HEAD as for the schedule (review finding 12). A full stick stalls '
             'the fins once r passes alpha_stall u / arm, %.2f deg/s at 2 m/s on these hulls. HEAD (cap %g deg/s, '
             'FF0, its integrator parked) turns 2527 at about %.1f deg/s there: the fins at %.2f x alpha_stall from a '
             'free 2.0 m/s (the pod-swing kick takes u to %.2f m/s, item 13) and %.2f x with the surge held at 2.0; '
             'from a free 1.8 m/s %.2f x; held at 2.0 for 300 s it departs (%.2f x, r %.1f deg/s). SCHED\'s cap at a '
             'free 2 m/s onset is %s deg/s, w(u_hi) = %g blending it back toward MAX, over the stall rate: on '
             '2527 it departs (fins past %g x) from %s m/s free and %s m/s held, HEAD from %s m/s free and %s m/s '
             'held (the stall-edge table; 2522: SCHED %s / %s, HEAD %s / %s). The boundary is the stall-rate '
             'condition r_cap > alpha_stall u / arm, and both policies sit on it: HEAD at 2 m/s within a few per cent '
             'of the stall angle, the schedule\'s blended cap moving the edge up on 2527 by %s m/s free and %s m/s '
             'held. The 2 m/s gate-1 failures are that edge, not a HEAD advantage. The speed where the schedule bites '
             'follows from the ramp\'s ends, %g / %g m/s, which are 0c\'s choice: the plan names the ramp, not its '
             'values. The stall term c u_lo sits inside the same blend and lowers the cap by under 1 deg/s (item 6), '
             'and FF is short there by (1 - w) k u.'
             % (stall_rate(2.0), config('2527').gains['AS_YAWRATE_MAX'], e[('2527', 'HEAD', 2.0, False)]['r_last5'],
                e[('2527', 'HEAD', 2.0, False)]['fin_stick'],
                e[('2527', 'HEAD', 2.0, False)]['u_stick_end'], e[('2527', 'HEAD', 2.0, True)]['fin_stick'],
                e[('2527', 'HEAD', 1.8, False)]['fin_stick'], long_h['HEAD']['fin_stick'] if long_h['HEAD'] else 0.0,
                long_h['HEAD']['r_last5'] if long_h['HEAD'] else 0.0, fnum(onset, '%.2f'), w_of(2.0), DEPART,
                dep('2527', 'SCHED', False) or 'none', dep('2527', 'SCHED', True) or 'none',
                dep('2527', 'HEAD', False) or 'none', dep('2527', 'HEAD', True) or 'none',
                dep('2522', 'SCHED', False) or 'none', dep('2522', 'SCHED', True) or 'none',
                dep('2522', 'HEAD', False) or 'none', dep('2522', 'HEAD', True) or 'none',
                edge_shift(book, False), edge_shift(book, True), P.AS_SPD_LO, P.AS_SPD_HI))

    stl = _rows(book, A1, ('SCHED_STALL', 'SCHED'), sources=allsrc, speeds=SPEEDS, cases=((1.0, 20.0), (1.0, 300.0)))
    if stl:
        zero = [x for x in stl if x[1] == 'SCHED_STALL' and x[7].get('hook') and x[7]['hook']['cap_min_stick'] == 0.0]
        dead = [x for x in zero if abs(x[7]['heading_stick']) <= DRIFT_FLOOR]
        dead_s = [x for x in stl if x[1] == 'SCHED' and abs(x[7]['heading_stick']) <= DRIFT_FLOOR]
        bind = sum(1 for x in stl if x[1] == 'SCHED_STALL' and x[4] in FWD and x[2] in ('perfect', 'ground_W0')
                   and x[7].get('hook') and 'stall' in x[7]['hook']['binding_stick'])
        known = sum(1 for x in stl if x[1] == 'SCHED_STALL' and x[4] in FWD and x[2] in ('perfect', 'ground_W0'))
        worst = max(dead, key=lambda x: x[6] * 1000 + x[4]) if dead else None
        same = book.gate(worst[0], 'SCHED', worst[2], 1.0, worst[4], worst[5], worst[6]) if worst else None
        groups = {}
        for x in dead:
            groups.setdefault(x[2], set()).add('%g' % x[4])
        gd = {p: gate1_counts([x for x in g['g1'] if x['policy'] == p])['dead'] for p in ('SCHED', 'SCHED_STALL')}
        item('The stall term as the plan has it (SCHED_STALL) does not earn its place. With the speed known at 2.5-6 '
             'm/s it binds on %d of %d full-stick legs (the authority term is lower there); at 2 m/s it sits inside the '
             'ramp\'s blend (item above). And where the selector\'s lower bound reaches 0 while its upper bound is past '
             'the ramp, c u_lo = 0 makes the cap 0: on %d of the full-stick legs on 2527 and 2522 the cap touched 0, '
             'and on %d the stick was dead, the hull turning under %g deg in the whole stick (sources and speeds: %s), '
             'against %d such leg(s) for SCHED. The longest, %s, turns %+.1f deg in the whole stick against SCHED\'s '
             '%+.1f. Gate 1 now counts a dead stick (under %g %% of HEAD\'s heading change in the stick) as no pass: '
             'SCHED_STALL has %d, SCHED %d (the gate section).'
             % (bind, known, len(zero), len(dead), DRIFT_FLOOR,
                '; '.join('%s at %s m/s' % (k, '/'.join(sorted(v, key=float))) for k, v in sorted(groups.items())),
                len(dead_s), _where(worst[0], worst[4], worst[5], worst[6], worst[2], held=worst[7].get('held'))
                if worst else '-', worst[7]['heading_stick'] if worst else 0.0, same['heading_stick'] if same else 0.0,
                100 * DEAD_SHARE, gd['SCHED_STALL'], gd['SCHED']))

    ramp = _rows(book, ('2527',), ('SCHED',), speeds=(2.5,), cases=((1.0, 300.0),))
    if ramp:
        item('Inside the ramp FF is short by (1 - w(u_lo)) k u_lo, and the integrator takes it: a 300 s full stick at '
             '2.5 m/s slows 2527 (free surge, trim thrust) to %s m/s (w %s) and leaves %s at release, a release of %s '
             'deg (120 s / in all, perfect source). The gliders\' 300 s holds are flown held, at w = 1.'
             % (' / '.join('%.2f' % x[7]['u_stick_end'] for x in ramp),
                ' / '.join('%.2f' % w_of(x[7]['u_stick_end']) for x in ramp),
                ' / '.join('%+.3f' % x[7]['I_rel'] for x in ramp), ' / '.join(drift_cell(x[7]) for x in ramp)))

    for kf in K_ERRORS:
        sc = _rows(book, A1, ('SCHED',), kfs=(kf,), speeds=FWD)
        if not sc:
            continue
        a, aw = _extreme([x for x in sc if x[6] == 20.0], 'drift_rest', True)
        b, bw = _extreme([x for x in sc if x[6] == 300.0], 'drift_rest', True)
        b1, bw1 = _extreme([x for x in sc if x[6] == 300.0], 'drift_120')
        asy = [(cfg, u) + asymptote(book, cfg, u, kf) for cfg in A1 for u in FWD]
        asy = [x for x in asy if x[4] is not None]
        ax = max(asy, key=lambda x: abs(x[4])) if asy else None
        flagged = [x for x in asy if x[2] is not None and x[3] != x[2] and abs(x[3]) < config(x[0]).gains['AS_YR_INT_LIM']]
        tau9 = [book.get(cfg, 'SCHED', 'perfect', kf, u, 1.0, TAU_HOLD, group='tau', held=held_cell(cfg, 300.0))
                for cfg, u in (('2527', 4.0), ('2527', 6.0), ('2522', 6.0))]
        c, cw = _extreme(_rows(book, A1, ('SCHED+RESTORE',), kfs=(kf,), speeds=FWD), 'drift_rest', True)
        item('k x%g (FF %s by %.0f %%), 2527 and 2522 at 2.5-6 m/s: SCHED releases at most %s deg in all after a '
             '20 s hold (%s) and %s deg in 120 s, %s deg in all after 300 s (%s). The integrator collects the FF error '
             'times the setpoint with the time constant (D + P) / K_I (%.1f-%.1f min here), so the 300 s holds show only '
             'part of the asymptote, (D - FF) sp / K_I stopped at the limit or at the allocator\'s flag: up to %s deg '
             '(%s %g m/s)%s; the 900 s holds (2527 4 / 6 m/s free, 2522 6 m/s held) release %s deg against %s '
             'predicted (the time-constant section). SCHED+RESTORE: at most %s deg in all (%s).'
             % (kf, 'short' if kf < 1 else 'over', abs(kf - 1) * 100, fnum(a, '%+.0f'), aw, fnum(b1, '%+.0f'),
                fnum(b, '%+.0f'), bw, *tau_span(book), fnum(ax[4] if ax else None, '%+.0f'), ax[0] if ax else '-',
                ax[1] if ax else 0,
                ('; where FF is short and the turn needs more than the pods deliver 1:1, the shortfall flag stops the '
                 'integrator first (%s)' % ', '.join('%s %g m/s at %+.3f' % (x[0], x[1], x[3]) for x in flagged))
                if flagged else '',
                ' / '.join(fnum(r['drift_rest'] if r and r['rest_valid'] else None, '%+.0f') for r in tau9),
                ' / '.join(fnum(predicted_release(book, cfg, u, kf, TAU_HOLD), '%+.0f')
                           for cfg, u in (('2527', 4.0), ('2527', 6.0), ('2522', 6.0))),
                fnum(c, '%+.0f'), cw))

    winds = tuple(s for s in allsrc if s.startswith('ground_W2') or s.startswith('ground_W3'))
    gw = _rows(book, A1, ('SCHED',), sources=winds, speeds=(4.0,), cases=((1.0, 20.0),))
    head4 = {x[0]: x[7] for x in _rows(book, A1, ('HEAD',), speeds=(4.0,), cases=((1.0, 20.0),))}
    gl = _rows(book, A1, ('SCHED',), sources=winds, speeds=FWD, cases=((1.0, 300.0),))
    if gw and gl:
        slow = ['%s %s: r %.2f of %.2f deg/s, %+.1f deg in the 20 s stick' % (
            x[0], x[2], x[7]['r_last5'], x[7]['sp_end'], x[7]['heading_stick']) for x in gw if x[2] == 'ground_W2_head']
        v, w = _extreme(gl, 'drift_120')
        vr, wr = _extreme(gl, 'drift_rest', True)
        vi, wi = _extreme(gl, 'I_rel')
        gsr = _rows(book, A1, ('SCHED+RESTORE',), sources=winds, speeds=FWD)
        c, cw = _extreme(gsr, 'drift_rest', True)
        gcb = _rows(book, A1, (CB,), sources=winds, speeds=FWD, cases=((1.0, 300.0),))
        vb, wb = _extreme(gcb, 'drift_rest', True)
        item('Ground speed only, outdoors (W 2 or 3, a wind of that speed): with the wind from ahead u_g - W reaches 0, '
             'FF falls to FF0 and the interval term rho / (k (u_hi - u_lo)) sets the cap near 1 deg/s; FF0 alone '
             'carries a few %% of the turn\'s torque, so the stick answers through the integrator, slowly (4 m/s, '
             'full stick: %s; HEAD %s). Over a 300 s hold the turn carries the wind round the hull, u_lo swings to 0 '
             'and back, and the integrator climbs toward rho: up to %+.3f at release (%s), a release of %+.0f deg in '
             '120 s (%s) and %s deg in all (%s); SCHED_CB* on the same legs up to %s deg in all (%s). SCHED+RESTORE '
             'over every wind case at 2.5-6 m/s: at most %s deg in all (%s).'
             % ('; '.join(slow), '; '.join('%s r %.2f, %+.1f deg' % (k, h['r_last5'], h['heading_stick'])
                                          for k, h in head4.items()), vi, wi, v, w, fnum(vr, '%+.0f'), wr,
                fnum(vb, '%+.0f'), wb, fnum(c, '%+.0f'), cw))

    pb = _rows(book, A1, ('SCHED',), sources=('pitot_bias_sdp', 'pitot_bias_ms', 'pitot_dropout', 'pitot_stuck'),
               speeds=FWD)
    if pb:
        txt = []
        for src in ('pitot_bias_sdp', 'pitot_bias_ms', 'pitot_dropout', 'pitot_stuck'):
            rows = [x for x in pb if x[2] == src]
            a, aw = _extreme(rows, 'drift_rest', True)
            txt.append('%s %s deg (%s)' % (src, fnum(a, '%+.0f'), aw))

        def iv(cfg, src, u0):
            r = book.gate(cfg, 'SCHED', src, 1.0, u0, 1.0, 300.0)
            return '[%s, %s]' % (fnum(r['u_lo_end'], '%.2f'), fnum(r['u_hi_end'], '%.2f')) if r else 'n/r'

        item('The pitot cases, SCHED, 2.5-6 m/s, the largest release in all: %s. A +1 Pa bias reads high (2.5 m/s as '
             '%.2f, 4 m/s as %.2f). pitot_bias_sdp sets dq to one SDP3x count (%g Pa), a resolution, 20 x under the '
             'bias: the band never contains the truth, still intersects ground W 2, and the selector keeps the biased '
             'value (2527 at the end of a 300 s hold from 2.5 / 4 m/s: %s / %s), so FF over-supplies and the release '
             'turns back. That is a band set below the zero error, a misconfiguration, not a property of the sensor '
             'class. pitot_bias_ms sets dq %g Pa, which covers the bias, and is also the band an uncalibrated SDP3x '
             'needs for its documented CMODEL 0 offset (%g Pa; the dqcheck legs are this case field for field): the '
             'design input is that row\'s trade-off, a band that covers the bias against the FF lost to a lower u_lo '
             'inside the ramp (2522 at the end of a 300 s hold, held at 2.5 / 4 m/s: %s / %s). A pitot stuck at its '
             'entry speed over-supplies as the turn bleeds the speed (a turn back). A dropout leaves the pitot at once '
             'and flies as ground only (W 2), the wind case above without the wind.'
             % ('; '.join(txt), math.sqrt(2 * (0.5 * P.RHO * 6.25 + 1.0) / P.RHO),
                math.sqrt(2 * (0.5 * P.RHO * 16.0 + 1.0) / P.RHO), P.DQ['SDP3x'], iv('2527', 'pitot_bias_sdp', 2.5),
                iv('2527', 'pitot_bias_sdp', 4.0), P.DQ['MS4525DO'], P.SDP_OFFSET, iv('2522', 'pitot_bias_ms', 2.5),
                iv('2522', 'pitot_bias_ms', 4.0)))

    sw = _rows(book, A1, ('HEAD', 'RESTORE', 'SCHED', 'SCHED+RESTORE'), speeds=FWD, cases=((1.0, 20.0), (1.0, 300.0)))
    kicks = {(r['config'], r['policy'], r['u0']): r['result']['kick'] for key, r in book.legs.items()
             if key[0] == 'kick'}
    if sw:
        text = []
        for cfg in A1:
            cnt = {}
            for x in sw:
                if x[0] == cfg:
                    key = '%s/%s' % (x[7]['rev_stick'], x[7]['rev_release'])
                    cnt.setdefault(x[1], {})
                    cnt[x[1]][key] = cnt[x[1]].get(key, 0) + 1
            text.append('%s: %s' % (cfg, '; '.join('%s %s' % (SHORT[p], ', '.join('%s x%d' % kv for kv in sorted(c.items())))
                                                    for p, c in cnt.items())))
        k1 = kicks.get(('2522', 'SCHED', 2.0))
        kick = ''
        if k1:
            kick = (' The onset has a surge kick (review finding 11): the swinging pod gets no thrust until it passes '
                    '90 deg while the other pushes forward with the whole couple, so the hull is pushed forward for '
                    'the swing\'s %.1f s. %s. Once both pods are on, the steady couple adds no surge (Fx %s N over '
                    'the second after the swing). This kick, not the steady couple, is what lifts 2522 at 2 m/s up '
                    'the ramp (u %.2f at the stick\'s end); it is the allocator\'s, worth a check in SITL (0b records '
                    'the pod reversals).'
                    % (k1['swing_end_s'], '; '.join('%s %s %g m/s: u %.2f -> %.2f, Fx up to %.1f N' % (
                        c, p, u, k['u_onset'], k['u_swing_end'], k['fx_max_N']) for (c, p, u), k in sorted(kicks.items())
                        if k['u_swing_end'] is not None), fnum(k1['fx_after_N'], '%+.2f'), k1['u_stick_end']))
        item('Pod swings, full stick at 2.5-6 m/s, stick/release (number of legs): %s. A full stick swings one pod '
             'into the rear at its onset under every policy (the yaw couple exceeds the collective thrust).%s After '
             'the release the pod swings back forward once the torque falls under the thrust: HEAD\'s parked '
             'integrator keeps the torque up through most of the 120 s (still turning), the others drop it at once; '
             'on 2522, which flies at zero thrust, a pod released at zero demand stays where it is. The policies add '
             'no swing in forward flight.' % (' | '.join(text), kick))

    rs = _rows(book, GATED, ('RESTORE',))
    if rs:
        v, w = _extreme(rs, 'drift_120')
        vr, wr = _extreme(rs, 'drift_rest', True)
        ex, wx = _extreme_of(rs, excursion)
        rsf = [x for x in rs if x[5] == 1.0 and x[6] == 20.0 and x[0] == '2527' and x[4] >= 2.5]
        pj, pw = _putback(rs)
        item('RESTORE alone puts the integrator back at the release, on every gated plant and speed, with no speed and '
             'no k: "in all" at most %s deg (%s), which is the put-back over K_I by construction wherever no flag '
             'clips the release (it integrates e = -r, so the rest of the turn is the integrator it starts with); the '
             'plant shows in the '
             'release\'s excursion, at most %s deg in 120 s (%s), and its 120 s heading change at most %+.1f deg '
             '(%s). It leaves the stick as HEAD has it: 2527\'s full stick still turns at %s deg/s against %g (2.5 / '
             '4 / 6 m/s), the integrator still parks at %s with no flag during it, and at the release the rule throws '
             'the parked integrator away in one cycle, a jump of up to %+.3f (%s) on top of the setpoint\'s own step.'
             % (fnum(vr, '%+.1f'), wr, fnum(ex, '%+.1f'), wx, v, w,
                ' / '.join('%.2f' % x[7]['r_last5'] for x in rsf), config('2527').gains['AS_YAWRATE_MAX'],
                ', '.join(sorted({'%.3f' % x[7]['I_rel'] for x in rsf})), pj or 0.0, pw))

    sr = _rows(book, GATED, ('SCHED+RESTORE',), sources=allsrc, kfs=(1.0,) + K_ERRORS, speeds=FWD)
    sa = _rows(book, GATED, ('SCHED',), sources=allsrc, kfs=(1.0,) + K_ERRORS, speeds=FWD)
    if sr and sa:
        vec = [x for x in sr if x[0] != '2512']
        cld = [x for x in sr if x[0] == '2512']
        v, w = _extreme_of(vec, excursion)
        vr, wr = _extreme(sr, 'drift_rest', True)
        cv, cw = _extreme_of(cld, excursion)
        sv, sw_ = _extreme(sa, 'drift_120')
        svr, swr = _extreme(sa, 'drift_rest', True)
        pj, pw = _putback(sr)
        sr2 = _rows(book, A1, ('SCHED+RESTORE',), sources=allsrc, kfs=(1.0,) + K_ERRORS, speeds=(2.0,))
        v2, w2 = _extreme_of(sr2, excursion)
        item('SCHED+RESTORE keeps SCHED\'s stick and makes the release independent of the speed source and of k. Over '
             'every gated plant at 2.5-6 m/s, every stick, source and k error, "in all" is at most %s deg (%s), by '
             'construction (the put-back over K_I, no flag clipping the release), against SCHED alone\'s %s deg (%s, '
             'and %+.1f deg in 120 s, %s). '
             'The informative number is the release\'s excursion in 120 s: at most %s deg on the vectored plants (%s); '
             'the Cloudship (k from its fins, item below) swings back by up to %s deg first (%s): the put-back of its '
             'parked integrator lets the sideslip it held unwind, and the integrator returns the heading over minutes. '
             'The put-back is the integrator SCHED would have released with: up to %+.3f (%s), largest where k or the '
             'source was furthest off. At 2 m/s it does not save the stalled turn (an excursion of up to %s deg, %s).'
             % (fnum(vr, '%+.1f'), wr, fnum(svr, '%+.0f'), swr, sv, sw_, fnum(v, '%+.1f'), w, fnum(cv, '%+.1f'), cw,
                pj or 0.0, pw, fnum(v2, '%+.0f'), w2))

    dist = [x for x in book.legs if x[0] == 'dist']
    if dist:
        rows = []
        for cfg in A1:
            for v in DIST_VARIANTS:
                parts = []
                for hold in (20.0, 300.0):
                    for policy in ('HEAD', 'SCHED', 'SCHED+RESTORE', 'RESTORE'):
                        src = 'perfect' if policy.startswith('SCHED') else 'none'
                        r = book.get(cfg, policy, src, 1.0, 4.0, 1.0, hold, group='dist', variant=v,
                                     held=held_cell(cfg, hold))
                        if r is not None:
                            parts.append('%s %g s %s' % (SHORT[policy], hold, drift_cell(r)))
                rows.append('%s %s: %s' % (cfg, v, ', '.join(parts)))
        item('The restore rule\'s cost, which no 0c condition exercises (the plan\'s wind moves only the ground speed, '
             'so the air-relative trim never changes and a restored trim is never stale): a yaw couple of %g of the '
             'authority at 4 m/s, full stick (2522\'s 300 s held), releases 120 s / in all: %s. A couple that changes '
             'during the turn costs about its own I / K_I (%.0f deg for %g) with or without the rule, because at speed '
             'the integrator learns it over minutes, not seconds; the rule adds back what the integrator learned '
             'during the hold (the 300 s rows). A couple that stays is free.'
             % (DIST_SHARE, '; '.join(rows), math.degrees(DIST_SHARE / 0.09), DIST_SHARE))

    rest = {}
    for key, rec in book.legs.items():
        if key[0] == 'rest':
            rest.setdefault((key[2], key[3]), []).append((key[1], rec['result']['identical']))
    if rest:
        parts = []
        for policy in ('SCHED', 'SCHED_STALL', 'SCHED+RESTORE_FWD', 'SCHED+RESTORE', 'RESTORE'):
            alls = [src for (p2, src), v in rest.items() if p2 == policy and all(i for _, i in v)]
            some = ['%s (%d of %d; not on %s)' % (src, sum(i for _, i in v), len(v),
                                                 ', '.join(sorted({c for c, i in v if not i})))
                    for (p2, src), v in rest.items() if p2 == policy and 0 < sum(i for _, i in v) < len(v)]
            none = [src for (p2, src), v in rest.items() if p2 == policy and not any(i for _, i in v)]
            if policy == 'RESTORE':
                n_legs = sum(len(v) for (p2, _), v in rest.items() if p2 == policy)
                n_same = sum(i for (p2, _), v in rest.items() if p2 == policy for _, i in v)
                parts.append('RESTORE: identical on %d of %d (the rule acts at every release)' % (n_same, n_legs))
                continue
            parts.append('%s: inert with %s%s; not with %s'
                         % (SHORT[policy], ', '.join(sorted(alls)) or 'no source',
                            ('; partly with ' + ', '.join(some)) if some else '', ', '.join(sorted(none)) or '-'))
        g1 = ['%s %s %d/%d' % (SHORT[p], s2 or '-', r['agree'], r['total']) for (p, s2), r in book.g1.items()
              if 'error' not in r]
        item('At rest (gate 2), bit-identical to HEAD on every plant and both sticks, by policy: %s. The schedule is '
             'inert at rest only while the selector\'s interval at rest stays under AS_SPD_LO: ground only with W >= '
             'AS_SPD_LO reads [0, W] at rest whatever the air does and switches the cap on (outdoors without a pitot '
             'the selector cannot tell calm air from a wind); the %g Pa band reads up to %.2f m/s at rest; a '
             'dropout falls back to ground W 2; and on the A 0.5 twins a full stick pushes the hull forward from rest, '
             'so the +1 Pa reading crosses AS_SPD_LO some seconds in. SCHED_CB* is SCHED at rest wherever w(u_hi) = 0. '
             'The restore rule acts at every release, at rest too. The 48 at-rest gate verdicts with each policy on '
             '(flown again with the revised hook): %s (the gate-lines and samples comparison is in the gate section).'
             % ('; '.join(parts), P.DQ['MS4525DO'], P.pitot_interval(0.0, P.DQ['MS4525DO'])[1], '; '.join(g1)))

    fw = _rows(book, A1, ('SCHED+RESTORE_FWD',), sources=allsrc, kfs=(1.0,) + K_ERRORS)
    if fw:
        groups = {}
        total = same_n = 0
        worst = None
        for x in fw:
            y = book.gate(x[0], 'SCHED+RESTORE', x[2], x[3], x[4], x[5], x[6])
            if y is None:
                continue
            total += 1
            if x[7]['restore_events'] == y['restore_events'] and abs(y['drift_120'] - x[7]['drift_120']) < 1e-9:
                same_n += 1
                continue
            label = '%g m/s %s' % (x[4], x[2] if x[3] == 1.0 else 'k x%g' % x[3])
            groups[label] = groups.get(label, 0) + 1
            if worst is None or abs(x[7]['drift_120']) > abs(worst[0][7]['drift_120']):
                worst = (x, y)
        by_src = {}
        for label, cnt in groups.items():
            src = label.split(' ', 2)[2]
            by_src[src] = by_src.get(src, 0) + cnt
        item('SCHED+RESTORE_FWD* (the addition: the put-back only on a cycle with w(u_lo) > 0) is bit-identical to '
             'HEAD at rest with the inert sources (above) and releases as SCHED+RESTORE on %d of %d forward legs. It '
             'drops the put-back wherever the selector\'s lower bound is under AS_SPD_LO at the release, which is not '
             'only at rest: the stalled 2 m/s turns, and every ground-only or dropout case whose u_g - W reaches 0 at '
             'the release. The legs where it differs, by source (all speeds): %s; the largest, %s, releases %s against '
             'SCHED+RESTORE\'s %s. So it gives up the safety net exactly when the speed source cannot vouch for '
             'forward flight.'
             % (same_n, total, ', '.join('%s %d' % kv for kv in sorted(by_src.items())),
                _where(worst[0][0], worst[0][4], worst[0][5], worst[0][6], worst[0][2], worst[0][3],
                       worst[0][7].get('held')) if worst else '-',
                drift_cell(worst[0][7]) if worst else '-', drift_cell(worst[1]) if worst else '-'))

    a05 = _rows(book, A05, ('HEAD', 'SCHED', CB), sources=tuple(SECONDARY_SOURCES), speeds=SPEEDS, cases=CASES)
    free = _rows(book, A05, ('HEAD',), speeds=SPEEDS, cases=CASES, which='free')
    if a05:
        hd = [x for x in a05 if x[1] == 'HEAD']
        i_h = _span(hd, 'I_rel')
        d_h = _extreme(hd, 'drift_120')
        wound = [x for x in hd if abs(x[7]['I_rel']) > 0.01]
        sc = [x for x in a05 if x[1] == 'SCHED']
        d_s = _extreme([x for x in sc if x[2] == 'perfect'], 'drift_rest', True)
        d_w = _extreme(sc, 'drift_rest', True)
        d_cb = _extreme([x for x in a05 if x[1] == CB], 'drift_rest', True)
        caps = ['%s %s' % (c, ' / '.join(fnum((book.gate(c, 'SCHED', 'perfect', 1.0, u, 1.0, 20.0) or {}).get('sp_end'),
                                              '%.2f') for u in FWD)) for c in A05]
        rates = ['%s %s' % (c, ' / '.join(fnum((book.gate(c, 'HEAD', u0=u) or {}).get('r_last5'), '%.2f') for u in FWD))
                 for c in A05]
        alloc = alloc_table()
        t4 = trim_of(book, '2527', 4.0)
        cell = alloc[(-90.0, 90.0)].get((round(t4, 4), 0.25)) if (alloc and t4 is not None) else None
        run = [x for x in free if x[6] == 300.0]
        item('A 0.5 (the pods held to +-90 deg; review finding 8). The allocator delivers the yaw command 1:1 only up '
             'to the collective thrust T: past it the pod that would push backward is clipped at 0 and the other '
             'carries T + tau, so the yaw is (T + tau) / 2, half gain, with the surge (T + tau) / 2 instead of T (at '
             '2527\'s 4 m/s trim %.3f a command of 0.25 delivers %s with surge %s; the allocator section). So the '
             'steady yaw available without adding surge is T (0 on the gliding 2522), not the A = 0.5 the plan\'s 2.나 '
             'assumes delivered 1:1: an input to A. Flown free, the yaw drives the speed (HEAD full stick 300 s: %s), '
             'so the gate flies the twins with the surge held. Held, HEAD\'s integrator stays at %s..%s and its '
             'release is at most %s deg in 120 s (%s): every full stick asks one pod to push backward, the shortfall '
             'is flagged and the integrator stopped. Where the torque stays under the thrust nothing is flagged and '
             'it parks: %s. SCHED lowers the setpoint (rho %g, A - rho %g: the full-stick cap %s deg/s at 2.5 / '
             '4 / 6 m/s, where HEAD turns at %s); its largest release in all %s deg (%s) with the speed known, %s deg '
             '(%s) over every source, where a wide interval keeps the torque under the thrust, nothing is flagged, '
             'FF is short and the integrator winds; SCHED_CB* %s deg (%s).'
             % (t4 or 0.0, fnum(cell['yaw'] if cell else None, '%.3f'), fnum(cell['surge'] if cell else None, '%.3f'),
                '; '.join('%s %g m/s %.2f -> %.2f' % (x[0], x[4], x[7]['u_stick_start'], x[7]['u_stick_end'])
                          for x in run),
                fnum(i_h[0], '%+.3f'), fnum(i_h[1], '%+.3f'), fnum(d_h[0], '%+.1f'), d_h[1],
                '; '.join('%s I %+.3f (flag up on %.0f %% of the stick), releases %s; SCHED there %s'
                          % (_where(x[0], x[4], x[5], x[6], held=True), x[7]['I_rel'],
                             100 * (x[7]['sat_frac_stick'] or 0), drift_cell(x[7]),
                             drift_cell(book.gate(x[0], 'SCHED', 'perfect', 1.0, x[4], x[5], x[6]))) for x in wound)
                or 'nowhere with the surge held', config('2527/A0.5').A - a_less_rho('2527/A0.5'),
                a_less_rho('2527/A0.5'), '; '.join(caps), '; '.join(rates), fnum(d_s[0], '%+.1f'), d_s[1], fnum(d_w[0], '%+.0f'), d_w[1],
                fnum(d_cb[0], '%+.0f'), d_cb[1]))

    cs = _rows(book, ('2512',), ('HEAD', 'SCHED', 'SCHED+RESTORE'), speeds=(2.0, 4.0, 6.0), cases=((1.0, 20.0),))
    kh = _rows(book, ('2512/khold',), ('SCHED', 'SCHED+RESTORE'), speeds=(2.0, 4.0, 6.0), cases=((1.0, 20.0),))
    hold = [h for h in book.hold if h['target'] == 'gz_cloudship_fin' and 'error' not in h]
    if cs and hold:
        cf_ = cloudship_closed_form(book)
        kfin = config('2512').k
        khd = k_hold_of(book.all) or 0.0
        l3 = {u: (book.gate('2512', 'HEAD', u0=u, stick=1.0, hold=300.0), book.get('2512', 'HEAD', u0=u, stick=1.0,
                                                                                    hold=300.0),
                  book.gate('2512', 'SCHED', 'perfect', 1.0, u, 1.0, 300.0),
                  book.get('2512', 'SCHED', 'perfect', 1.0, u, 1.0, 300.0)) for u in SPEEDS}
        k3 = [book.gate('2512/khold', 'SCHED', 'perfect', 1.0, u, 1.0, 300.0) for u in SPEEDS]
        item('The Cloudship (2512). k from its fins (%.4f) is %.1f x short of what its turns need (the steady-turn '
             'slope, %.3f per rad/s per m/s). A force balance shows why (review finding 16): the tail\'s side force at '
             'x_t %.3f m and the fins\' at x_f %.3f m (from the centre of mass) carry the centripetal m u r and cancel '
             'each other\'s moment, so the torque command per rad/s per m/s is m |x_t| |x_f| / (x_f - x_t) / authority '
             '= %.2f (m %.2f kg, authority %.3f N m), against the %.2f measured: the turn\'s torque is set by the '
             'centripetal force and the %.3f m tail-to-fin gap, not by fin damping. It is that sensitive to geometry: '
             'with the arms from the link origin (%.0f mm behind the centre of mass) it gives %.2f, so a tail '
             'thruster\'s k has to be measured in a held turn (the plan\'s procedure), not computed from the fins. '
             'With k from the fins the schedule supplies %.0f %% of the turn\'s torque and the integrator parks as at '
             'HEAD: %s. With k the measured slope: %s. HEAD: %s. The stall term from the fins\' geometry never binds '
             'on it (c u = %.1f deg/s at 2 m/s, over MAX). The 300 s holds (review finding 7): flown free, the gliding '
             'hull loses its speed or tumbles (HEAD from 4 / 6 m/s ends the stick at u %s m/s; SCHED at %s), so the '
             'gate flies them held at the label: HEAD %s, SCHED (k from the fins) %s, SCHED with k measured %s, at 2 / '
             '2.5 / 4 / 6 m/s (free: HEAD %s, SCHED %s).'
             % (kfin, khd / kfin, khd, cf_['x_t'] if cf_ else 0.0, cf_['x_f'] if cf_ else 0.0,
                cf_['k_closed'] if cf_ else 0.0, cf_['m'] if cf_ else 0.0, cf_['authority'] if cf_ else 0.0, khd,
                (cf_['x_f'] - cf_['x_t']) if cf_ else 0.0, 1000 * cf_['com_x'] if cf_ else 0.0,
                cf_['k_origin'] if cf_ else 0.0, 100 * kfin / khd,
                '; '.join('%g m/s r %.2f/%.2f I %+.3f %s' % (x[4], x[7]['r_last5'], x[7]['sp_end'], x[7]['I_rel'],
                                                           drift_cell(x[7])) for x in cs if x[1] == 'SCHED'),
                '; '.join('%s %g m/s r %.2f/%.2f I %+.3f %s' % (SHORT[x[1]], x[4], x[7]['r_last5'], x[7]['sp_end'],
                                                              x[7]['I_rel'], drift_cell(x[7])) for x in kh) or 'n/r',
                '; '.join('%g m/s r %.2f I %+.3f %s' % (x[4], x[7]['r_last5'], x[7]['I_rel'], drift_cell(x[7]))
                          for x in cs if x[1] == 'HEAD'),
                math.degrees(config('2512').stall['c'] * 2.0),
                ' / '.join(fnum((l3[u][1] or {}).get('u_stick_end'), '%.2f') for u in (4.0, 6.0)),
                ' / '.join(fnum((l3[u][3] or {}).get('u_stick_end'), '%.2f') for u in (4.0, 6.0)),
                ' / '.join(drift_cell(l3[u][0]) for u in SPEEDS), ' / '.join(drift_cell(l3[u][2]) for u in SPEEDS),
                ' / '.join(drift_cell(r) for r in k3), ' / '.join(drift_cell(l3[u][1]) for u in SPEEDS),
                ' / '.join(drift_cell(l3[u][3]) for u in SPEEDS)))

    s17 = _rows(book, ('2517',), P.SPEC_POLICIES, speeds=(2.0, 4.0), cases=((1.0, 20.0),))
    if s17:
        g4 = {x[1]: x[7] for x in s17 if x[4] == 4.0}
        item('2517 (not gated). None of the policies stabilises it: no policy changes P, so none changes the loop\'s '
             'damping, and the plant stays directionally unstable at speed. SCHED makes the spin worse at 4 m/s: its '
             'FF (from the fins, %.2f against HEAD\'s FF0 %g) pushes harder into the departure, r peak %.1f deg/s '
             'against HEAD\'s %.1f and %.0f %% of the stick stalled against %.0f %%. Full stick, 20 s: %s. The cap '
             'lowers the setpoint, not the departure; the restore rule puts the integrator back on a hull that is '
             'still spinning.'
             % (config('2517').gains['AS_YAWRATE_FF'] + config('2517').k * 4.0, config('2517').gains['AS_YAWRATE_FF'],
                g4['SCHED']['r_peak'], g4['HEAD']['r_peak'], 100 * (g4['SCHED']['stalled_stick'] or 0),
                100 * (g4['HEAD']['stalled_stick'] or 0),
                '; '.join('%s %g m/s r peak %.1f deg/s, fins %.1f x stall (%.0f %% stalled), released %+.0f deg in '
                          '120 s, r %+.2f deg/s at +120 s' % (SHORT[x[1]], x[4], x[7]['r_peak'], x[7]['fin_stick'] or 0,
                                                             100 * (x[7]['stalled_stick'] or 0), x[7]['drift_120'],
                                                             x[7]['r_120']) for x in s17)))

    nz = [rec for key, rec in book.legs.items() if key[0] == 'noise']
    if nz:
        spread = {}
        for rec in nz:
            clean = book.get(rec['config'], rec['policy'], rec['source'], 1.0, 4.0, 1.0, 20.0)
            if clean:
                spread.setdefault(rec['policy'], []).append(abs(rec['result']['drift_120'] - clean['drift_120']))
        item('gz\'s gyro noise (three seeds, 2527 and 2512 at 4 m/s) moves the 120 s release by at most %s deg.'
             % ', '.join('%s %.2f' % (SHORT[p], max(v)) for p, v in spread.items()))

    item('What wins where (the evidence for the owner\'s choices; no decision here). (a) Where the schedule\'s '
         'assumptions hold, 2.5-6 m/s with the speed known within its interval and k right, SCHED alone removes the '
         'defect and keeps a proportional stick (item 2). (b) Its formula guarantees the integrator only with u_lo '
         'past the ramp, and on the A 1 plants only up to rho = AS_YR_INT_LIM, the defect\'s own %.0f deg (items 3, '
         '4); writing the interval term on the FF supplied (SCHED_CB*) extends the guarantee to u_hi past the ramp. '
         '(c) SCHED alone carries any FF error (a wrong k, the ramp, a wide or biased interval) into the release, '
         'toward its asymptote over several minutes, up to the same %.0f deg as HEAD (items 8-12); the restore rule '
         'removes that whatever the error ("in all" by construction; the 120 s excursion is the plant\'s number, items '
         '14-15), at the price of a stale trim when the yaw trim changes during the turn (item 16). That is the input '
         'to remaining decision 1. (d) Neither fixes 2 m/s, where HEAD and the schedule both sit on the fins\' stall '
         'edge and the ramp\'s blend lifts the cap over it (item 5), nor the Cloudship with k from its fins (k has to '
         'come from a held turn). (e) The stall term as the plan has it finds no use and kills the stick where u_lo '
         'reaches 0 (item 6). (f) For pods that cannot reverse, A = 0.5 is a peak with surge, not a steady authority '
         '(item 19). (g) At rest the schedule is inert only with sources whose interval at rest stays under AS_SPD_LO; '
         'the restore rule never is, and gating it on forward flight (the addition) also drops it when the source '
         'cannot vouch for the speed (items 17-18).' % (lim_drift(), lim_drift()))

    say('\nGate verdicts, in short (gate 1 on the gate\'s legs: passes = smaller on merit + within the floor, both with '
        'a live stick; dead sticks and undecided passes counted apart):')

    for policy in ('RESTORE',) + SCHED_POLICIES + (CB,):
        rs = [x for x in g.get('g1', []) if x['policy'] == policy]
        if rs:
            c = gate1_counts(rs)
            bad = [x for x in rs if x['verdict'] in ('not', 'dead')]
            say('  (1) %-18s %d of %d pass (%d on merit, %d by the floor; %d of the passes judged on 120 s alone); %d '
                'dead sticks, %d not smaller%s'
                % (SHORT[policy], c['pass'], len(rs), c['smaller'], c['floor'], c['undecided'], c['dead'], c['not'],
                   '' if not bad else ' (at: %s)' % ', '.join(sorted({'%s %g m/s' % (x['cfg'], x['u0']) for x in bad}))))
    say('  (2) at rest: the gate section; (3) the FF-error table and items above.')


def report_owner(book, say):
    """The formula questions the review raised, for the plan's remaining decisions; evidence only."""
    say('\n=== For the owner: formula questions for the plan\'s remaining decisions (evidence, no decision) ===')
    cls = cb_class_stats(book, FWD)
    ir = cls.get('lo_in_ramp')
    say('  1. The interval term. The plan\'s rho / (k (u_hi - u_lo)) is consistent with its FF only where w(u_lo) = '
        '1 (u_lo >= AS_SPD_HI). Writing it on the FF supplied, rho / (k (u_hi - w(u_lo) u_lo)), keeps the promise for '
        'any u_lo while u_hi is past the ramp (tested on 3000 random intervals), and changes nothing where w(u_lo) is '
        '1 or 0. Flown (SCHED_CB*, the gate\'s legs of 2527 and 2522 at 2.5-6 m/s): on the %d legs whose interval '
        'ended the stick with u_lo inside the ramp, the largest integrator at release %s -> %s (none at the limit '
        'either way); the review\'s gliding 2522 with the 1.05 Pa band, free surge, %s. It does not help where the '
        'hull itself slows into the ramp (item 2). Proposed for the plan\'s 남은 결정: write the interval term on the '
        'FF supplied.'
        % (ir['n'] if ir else 0, fnum(ir['I'][0][0] if ir else None, '%+.3f'),
           fnum(ir['I'][1][0] if ir else None, '%+.3f'), review_example_text(book, (4.0,))))
    say('  2. Inside the ramp (u_hi < AS_SPD_HI). The cap is blended back toward MAX by w(u_hi), and any blend toward '
        'MAX lifts it over the budget\'s r_lim, so no interval term and no blend weight keeps the integrator within '
        'rho there; blending by w(u_lo) for both (one weight) moves the cap further toward MAX whenever u_lo is in the '
        'ramp (the interval-term section computes it). This is where the fins also stall, for HEAD too (2527, a full '
        'stick departs at entry speeds %s). The question is what the schedule should promise inside the ramp: e.g. '
        'a cap that blends toward the stall rate alpha_stall u / arm rather than MAX, or a ramp placed below the '
        'stall zone; the ramp\'s ends %g / %g m/s were 0c\'s choice.' % (edge_text(book), P.AS_SPD_LO, P.AS_SPD_HI))
    say('  3. rho on A 1. rho = min(AS_YR_INT_LIM, A / 2) = AS_YR_INT_LIM wherever A >= 2 AS_YR_INT_LIM (every A 1 '
        'plant at the default %g): the bound "the integrator within rho" is then the defect state, and the release '
        'bound rho / K_I = %.0f deg is HEAD\'s. rho = m min(AS_YR_INT_LIM, A / 2) with m < 1 bounds it at m x %.0f '
        'deg (m 0.5: %.0f deg) at the price of a lower cap wherever the interval term binds (it scales with rho). '
        'Beside the restore rule, the other way to bound the drift independently of the values.'
        % (config('2527').gains['AS_YR_INT_LIM'], lim_drift(), lim_drift(), 0.5 * lim_drift()))
    alloc = alloc_table()
    t4 = trim_of(book, '2527', 4.0)
    cell = alloc[(-90.0, 90.0)].get((round(t4, 4), 0.25)) if (alloc and t4 is not None) else None
    t6 = trim_of(book, '2527', 6.0)
    say('  4. A and the collective thrust (2.나). The allocator delivers a yaw command 1:1 only up to a knee set by '
        'the thrust T the pods also carry: T for pods that cannot reverse (beyond it half gain with as much added '
        'surge: 2527 at 4 m/s, T %.3f, a command of 0.25 delivers %s with surge %s), 1 - T for pods that can (beyond '
        'it half gain with the surge dropping: %.3f at 2527\'s 6 m/s trim, where the schedule\'s A - rho = %g leaves '
        'the integrator about %.3f before the flag instead of rho). The plan\'s A = 0.5 (1 + rho_rev) leaves T out: '
        'a steady A would be min(0.5, T) or 1 - T at the operating point, 0 on a gliding hull with pods that cannot '
        'reverse. The A formula is the input.'
        % (t4 or 0.0, fnum(cell['yaw'] if cell else None, '%.3f'), fnum(cell['surge'] if cell else None, '%.3f'),
           1.0 - (t6 or 0.0), a_less_rho('2527'), 1.0 - (t6 or 0.0) - a_less_rho('2527')))
    say('  5. A per direction (2.나) is now carried (PlantConfig.A_dir, a per-direction cap through '
        'controller.set_rate_cap); every 0c plant has the same A both ways, so no result moves. A tail without its '
        'CA_R_REV bit gets A- = 0: its negative stick has no propulsive yaw at speed, and the positive one keeps its '
        'cap (tested, test_policies.py).')
    r = _rows(book, GATED, ('SCHED+RESTORE',), sources=tuple(P.SOURCES), kfs=(1.0,) + K_ERRORS, speeds=FWD)
    s = _rows(book, A1, ('SCHED',), sources=('perfect',), kfs=K_ERRORS, speeds=FWD)
    v, w = _extreme_of([x for x in r if x[0] != '2512'], excursion)
    a, wa = _extreme(s, 'drift_rest', True)
    say('  6. Remaining decision 1 (a value-independent bound on the release). Without one, an FF error leaves its '
        'asymptote I_inf / K_I after a long turn, up to %.0f deg (flown at 2.5-6 m/s: %s deg, %s; the time-constant '
        'section has the asymptotes). The restore rule removes it whatever the error, its release excursion at most '
        '%s deg in 120 s on the vectored plants (%s); its cost is a stale trim when the yaw trim changes during the '
        'turn, about the change\'s own I / K_I (the disturbance section). Option 3 (a strict-fraction rho) bounds '
        'the drift without touching the integrator at the release.'
        % (lim_drift(), fnum(a, '%+.0f'), wa, fnum(v, '%+.1f'), w))


def ff_share(book):
    """(min, max) % of the steady-turn hold torque that FF0 + k u supplies on 2527 and 2522 at 2.5-6 m/s."""
    out = []
    for name in A1:
        cfg = config(name)
        for h in book.hold:
            if h.get('target') == cfg.target and 'error' not in h and h['u0'] in FWD:
                out.append(100.0 * (cfg.gains['AS_YAWRATE_FF'] + cfg.k * h['u0']) / h['per_rad_s'])
    return (fnum(min(out), '%.0f'), fnum(max(out), '%.0f')) if out else ('-', '-')


if __name__ == '__main__':
    main()
