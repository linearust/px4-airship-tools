#!/usr/bin/env python3
"""Tests of forward_replay.py, the 0b replay. Plain python:

    python3 test_forward_replay.py          # about two minutes on an idle machine
    python3 test_forward_replay.py --quick  # without the real records and the probe

1. Synthetic records: a model leg written as a log on SITL-like sample grids
   (forward_replay.model_log), with an acceleration in its throttle history,
   measured by forward_yaw_check.analyze itself into a kept summary beside
   an events file; the replay measures it again from that log, held in
   memory where a real record's ULog is on disk. Each passes every one of
   the record's verdicts; its run_leg block is the source leg's keyword
   arguments; the kept summary is the replay's measurement value for value.
   On the five exact cases (no gyro noise: the vectored airship, the glide,
   the Cloudship with its pods holding, a stick to the left, and the first
   with its rate setpoint logged as SITL logs it, once per 52 ms stick
   message, the stick changing on one) the replay is the source leg bit for
   bit and every compared number is the record's exactly. A sixth carries
   gz's gyro noise in the record.
2. Mutations, each refused by the precondition that names it: of the log
   (the airframe, a gain changed or absent, an allocator parameter, the
   gyro cutoff, a pitch, the integrator at the switch, the throttle, an
   unread entry state; from the release: the integrator's sample age, the
   pods' demand, the allocator status at the stick end), of today's
   measurement in memory (a verdict, the replay block's clock and states,
   a pod's start tilt, the table, the run_leg block, the model interface),
   of the run's files
   (the ULog, the events file), and of the kept summary's leg times. A kept
   summary edited, or written by another forward_yaw_check, is replayed from
   today's measurement, its differences reported.
3. What the comparison alone catches, the record's checks bypassed: the same
   wrong inputs flown against the record, the noisy one too.
4. The model's own metrics (forward.run_leg's, on every 4 ms step) against
   the replay's measurement of the same run through model_log and
   forward_yaw_check: the log's mapping (the velocity's rotation, the gyro's
   axes, the quaternion, the servos, the allocator status) seen
   independently, and a sway sign and a heading sign broken in it are seen.
5. The verdict logic of a compared number and the seeds' band, the fins'
   tolerance and the stall the noise decides, the centred rms against
   noise, the flip times against forward_yaw_check's counters, the log's
   timing convention, the motor commands and the allocator status derived
   from the run (on every series row against the sign the loop hands its
   controller, the Cloudship's with its tail saturating too), the CLI's
   exit codes and its CSV.
6. The real runs in ../sitl_out, where present, measured by today's
   forward_yaw_check from their ULog and events file: the two aborted legs
   refused by their failed verdicts, a full leg replayed; the 2527 SITL
   probe (an informal run, no events): refused, and its hold torque per
   rad/s against step0a.probes()' own number in forward_out_20261001.txt.
"""
import contextlib
import copy
import io
import json
import math
import os
import re
import shutil
import sys
import tempfile
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import closed_loop as cl  # noqa: E402
import forward as F  # noqa: E402
import forward_replay as R  # noqa: E402

fyc = R.fyc
BENCH = os.path.dirname(HERE)
SITL_OUT = os.path.join(BENCH, 'sitl_out')
V27, V22 = 'gz_airship_vectored_independent_am_fin_drag', 'gz_airship_vectored_independent_fin'
C12, C17 = 'gz_cloudship_fin', 'gz_cloudship_am_fin_drag'
S = 1_000_000
ACRO = 40 * S
# SITL's logged intervals (SDLOG_PROFILE 131; the summaries' 'rates'), each
# topic's phase set apart from the others' so that 'the sample at or before'
# matters; the rate setpoint every cycle on the first cases
GRIDS = {'vehicle_status': (500_000, 0), 'vehicle_control_mode': (500_000, 0),
         'manual_control_setpoint': (200_000, 12_000), 'vehicle_rates_setpoint': (4_000, 0),
         'vehicle_angular_velocity': (8_000, 4_000), fyc.GT_ATT: (20_000, 8_000), fyc.GT_POS: (20_000, 8_000),
         'vehicle_local_position': (20_000, 8_000), 'vehicle_torque_setpoint': (20_000, 4_000),
         'vehicle_thrust_setpoint': (20_000, 4_000), 'rate_ctrl_status': (200_000, 4_000),
         'control_allocator_status': (200_000, 16_000), 'actuator_motors': (100_000, 36_000),
         'actuator_servos': (100_000, 36_000)}
# The rate setpoint as SITL logs it: the controller publishes it only on a new
# stick (airship_att_control_main.cpp, publishRatesSetpoint under new_sticks),
# one per MANUAL_CONTROL message, 52 ms apart (the summaries' median), and the
# stick changes on a message. A grid through the onset, 36 ms off the
# others' phase, and a stick of 385 messages, so the release is on it too
SITL_RATES_SP = (52_000, 36_000)
STICK_52_S = 385 * 0.052
ROW_S = GRIDS[fyc.GT_POS][0] / 1e6          # the table's rows: the groundtruth samples
ACCEL_THROTTLE, ACCEL_END_S = 0.5, 10.0     # the synthetic log's throttle until this long before the switch
FAILS = []


def check(name, ok, detail=''):
    print('%s  %s%s' % ('PASS' if ok else 'FAIL', name, ('  -- ' + detail) if detail else ''), flush=True)

    if not ok:
        FAILS.append(name)


def grid(period, phase, a, b):
    k0 = -((ACRO + phase - a) // period)
    k1 = (b - ACRO - phase) // period
    return ACRO + phase + period * np.arange(k0, k1 + 1, dtype=np.int64)


def source_kwargs(target, u0, stick, throttle, v0, on_us, off_us, end_us):
    """run_leg's keyword arguments of a synthetic leg, as the record's run_leg
    block will give them back: the gains as the shortest decimals of their
    float32, the pods at tilt 0 in the mode AirshipPod's rule reaches after
    the log's acceleration (forward.infer_pod_mode, entering from above),
    integ0 0. throttle a binary fraction, so the float32 setpoints' mean is
    exact."""
    model, airframe = cl.ALL_TARGETS[target]
    vectored = airframe == 'vectored'
    files = fyc.plan_params(model)
    mode = F.infer_pod_mode(abs(throttle))
    return {'target': target, 'u0': u0, 'v0': v0, 'r0': 0.0, 'p0': 0.0, 'phi0': 0.0, 'stick': stick,
            'quantize': False, 'throttle': throttle,
            'params': {k: fyc.f32_decimal(float(np.float32(files[k]))) for k in fyc.GAINS},
            'start': 'acro', 'manual_s': 0.0, 'centred_s': on_us / 1e6, 'stick_s': off_us / 1e6 - on_us / 1e6,
            'release_s': end_us / 1e6 - off_us / 1e6,
            'pods0': [{'tilt': 0.0, 'mode': mode}, {'tilt': 0.0, 'mode': mode}],
            'motors0': [throttle, throttle] + ([] if vectored else [0.0]), 'integ0': 0.0, 'gyro_settled': True}


def synthetic(target, out_dir, stamp, u0, stick, throttle, v0=0.0, on_s=3.0, release_s=40.0, grids=None,
              noise_seed=None, stick_s=fyc.STICK_S):
    """A 0b record of a model leg, written by forward_yaw_check's own analyze.

    The leg (run_leg from source_kwargs, with gz's gyro noise when
    noise_seed is given) is written as a log on GRIDS (with `grids` over
    them) by forward_replay.model_log, held at its entry state for 30 s
    before the switch in Manual with an acceleration throttle until
    ACCEL_END_S before it, then analyzed as a live run (a plan and the
    command brackets) with the leg's own release length and analyze's
    stick length (its verdict allows the leg's within DURATION_TOL_S). The
    kept summary names the log by a path that is never written (the log
    stays in memory: record_of hands it over) and the events file written
    beside it. Returns {'path', 'log', 'ulg' (the log's path),
    'events_path', 'source', 'kwargs', 'checks' (the analysis's
    verdicts)}."""
    model = cl.ALL_TARGETS[target][0]
    on_us, off_us = int(round(on_s * S)), int(round((on_s + stick_s) * S))
    end_us = off_us + int(round(release_s * S))
    kwargs = source_kwargs(target, u0, stick, throttle, v0, on_us, off_us, end_us)
    source = R.run_model(copy.deepcopy(kwargs), noise=noise_seed is not None, seed=noise_seed or 1)
    params = {k: float(np.float32(v)) for k, v in fyc.plan_params(model).items()}
    params['IMU_GYRO_CUTOFF'] = float(np.float32(R.ctl.DEFAULT_PARAMS['IMU_GYRO_CUTOFF']))
    log0, log1 = ACRO - 30 * S, ACRO + end_us + 2 * S
    times = {topic: grid(p, phase, log0, log1) for topic, (p, phase) in dict(GRIDS, **(grids or {})).items()}
    acro_end = ACRO + end_us + 52_000
    modes = [(log0, fyc.NAV_MANUAL, True), (ACRO, fyc.NAV_ACRO, True), (acro_end, fyc.NAV_MANUAL, True)]
    plant = fyc.plant_info(model, params)
    log = R.model_log(source, 'acro', times, ACRO, 0.0, 20.0, modes, plant, params, (log0, log1))
    thrust = log.data['vehicle_thrust_setpoint']
    thrust['xyz[0]'] = np.where(thrust['timestamp'] < ACRO - int(ACCEL_END_S * S), ACCEL_THROTTLE, thrust['xyz[0]'])
    log.path = os.path.join(out_dir, 'forward_%s_%s.ulg' % (model, stamp))
    counts = int(round(throttle * 1000.0))
    plan = dict(fyc.vehicle_plan(model, [(u0, stick)])['legs'][0], speed=u0, stick=stick, number=1, rerun='1',
                throttle_counts=counts, throttle=counts / 1000.0)
    on, off = ACRO + on_us, ACRO + off_us
    events = {'acro': (ACRO - 30_000, ACRO - 10_000), 'stick_on': (on - 60_000, on - 40_000),
              'stick_off': (off - 60_000, off - 40_000), 'release_end': (acro_end - 20_000, acro_end + 5_000)}
    live = {'model': model, 'legs': [{'leg': 1, 'flown': True, 'plan': plan, 'events': events}]}
    # the events file as a run writes it, and analyze on what it reads back
    events_path = os.path.join(out_dir, 'forward_%s_%s_events.json' % (model, stamp))

    with open(events_path, 'w') as handle:
        json.dump(live, handle, indent=1, ensure_ascii=False, default=float)

    with open(events_path) as handle:
        live = json.load(handle)

    with contextlib.redirect_stdout(io.StringIO()):
        doc, checks = fyc.analyze(log, model, live=live, out_dir=out_dir, stamp=stamp, release_s=release_s)

    doc['live'] = {'events': events_path, 'aborted': None, 'plan': None, 'events_write_failures': []}
    path = os.path.join(out_dir, 'forward_%s_%s_summary.json' % (model, stamp))

    with open(path, 'w') as handle:
        json.dump(doc, handle, indent=1, ensure_ascii=False, default=float)

    return {'path': path, 'log': log, 'ulg': log.path, 'events_path': events_path, 'source': source,
            'kwargs': kwargs, 'checks': checks}


def record_of(made, leg=1, log=None):
    """forward_replay's Record of a synthetic summary, its log handed over
    (where a real record's is read from disk)."""
    return R.load(made['path'], [leg], logs={made['ulg']: made['log'] if log is None else log})[0]


def failed(result):
    return [name for name, ok, _ in result['checks'] if not ok]


CASES = (
    # (tag, name, target, u0, stick, throttle (a binary fraction), v0 FRD, synthetic()'s options)
    ('v27', 'vectored 2527, 2 m/s, full stick, 0.04 m/s sway to starboard', V27, 2.0, 1.0, 0.03125, 0.04, {}),
    ('v22', 'vectored 2522 gliding at 4 m/s, half stick to the left', V22, 4.0, -0.5, 0.0, 0.0, {}),
    ('c17', 'Cloudship 2517, 2 m/s, full stick, pods holding', C17, 2.0, 1.0, 3.0 / 256.0, 0.0, {}),
    # (0.02 m/s of sway here turns the hull 0.31 deg/s in the entry: an
    # unsteady entry, which the record's verdict refuses)
    ('c12', 'Cloudship 2512 gliding at 4 m/s, full stick to the left, 0.005 m/s sway to port', C12, 4.0, -1.0, 0.0,
     -0.005, {}),
    ('v27_52ms', 'vectored 2527 as the first, the rate setpoint logged as SITL logs it (once per 52 ms stick '
     'message, a 20.02 s stick)', V27, 2.0, 1.0, 0.03125, 0.04,
     {'grids': {'vehicle_rates_setpoint': SITL_RATES_SP}, 'stick_s': STICK_52_S}),
    ('v27_noisy', 'vectored 2527 as the first, gz\'s gyro noise in the record (seed 5)', V27, 2.0, 1.0, 0.03125, 0.04,
     {'noise_seed': 5}),
)
EXACT = ('v27', 'v22', 'c17', 'c12', 'v27_52ms')


# ---------------------------------------------------------------------------
# 1. synthetic records
# ---------------------------------------------------------------------------

def run_leg_differences(run_leg, kwargs):
    return {k: (run_leg.get(k), v) for k, v in kwargs.items() if run_leg.get(k) != v}


def test_synthetic(tmp):
    made = {}

    for tag, name, target, u0, stick, throttle, v0, options in CASES:
        out = os.path.join(tmp, tag)
        os.makedirs(out)
        m = made[tag] = synthetic(target, out, 'synth', u0, stick, throttle, v0, **options)
        bad = [n for n, ok, _ in m['checks'] if not ok]
        check('%s: forward_yaw_check.analyze passes every verdict of the record written from the model leg (%d)'
              % (name, len(m['checks'])), not bad, '; '.join(bad))

        with open(m['path']) as handle:
            doc = json.load(handle)

        leg = doc['legs'][0]
        run_leg, kwargs = leg['replay']['run_leg'], m['kwargs']
        differ = run_leg_differences(run_leg, kwargs) if run_leg else None

        if tag in EXACT:
            check('  ... its run_leg block is the source leg\'s keyword arguments, bit for bit (pod modes %s from the '
                  'logged demand)' % [p['mode'] for p in kwargs['pods0']], differ == {},
                  str(differ) if run_leg else str(leg['replay']['run_leg_unread']))
        else:
            # the record's gyro sample at the switch: the model's entry state
            # held (gyro_settled), its noise starting with the first step
            check('  ... its run_leg block is the source leg\'s keyword arguments but r0, the gyro sample at the switch '
                  '(%s deg/s)' % (run_leg or {}).get('r0'),
                  run_leg is not None and set(differ) <= {'r0'} and abs(run_leg['r0']) < 0.2, str(differ))

        if tag == 'v27_52ms':
            # its edges are the samples the stick changed on: the record reads
            # the stick at the length flown
            median = ((doc.get('rates') or {}).get('vehicle_rates_setpoint') or {}).get('median_ms')
            check('  ... the rate setpoint logged at a median %s ms; the record\'s stick %.3f s, the length flown'
                  % (median, leg['durations_s']['stick']), median == 52.0
                  and abs(leg['durations_s']['stick'] - STICK_52_S) < 1e-9)

        rec = record_of(m)
        seeds = 2 if tag == 'v27_noisy' else 0
        result = m['result'] = R.replay(rec, 'acro', seeds)
        check('  ... every precondition holds and it replays within tolerance%s' % (
            ' (2 seeds)' if seeds else ''), result['status'] == 'within tolerance',
            '%s; %s; %s' % (result['status'], failed(result),
                            [r['metric'] for r in result['rows'] if r['verdict'] == 'FAIL']))
        check('  ... the kept summary is the replay\'s own measurement of the log, value for value',
              result['kept'] == [], str((result['kept'] or [])[:3]))

        if result['model'] is None:
            continue

        if tag in EXACT:
            same = all(list(result['model']['trace'][k]) == list(m['source']['trace'][k]) for k in m['source']['trace'])
            check('  ... the replayed leg is the source leg, bit for bit (every trace column)', same)
            inexact = [r for r in result['rows'] if r['kind'] in ('value', 'count', 'rel') and r['diff'] not in (None, 0.0, 0)]
            rms = [r['model'] for r in result['rows'] if r['kind'] == 'upper']
            values = [r for r in result['rows'] if r['verdict'] != 'n/a']
            check('  ... every compared number is the record\'s exactly (%d, the hold torque and the centred rms too)'
                  % len(values), not inexact and rms == [0.0]
                  and all(r['verdict'] in ('ok', 'indeterminate', 'n/a') for r in result['rows']),
                  '; '.join('%s %r / %r' % (r['metric'], r['record'], r['model']) for r in inexact) + ' rms %s' % rms)
            inputs = [r for r in result['entry'] if r['metric'].startswith('at the Acro switch, inputs: ')
                      and r['diff'] is not None and r['diff'] != 0.0 and 'integrator' not in r['metric']
                      and 'setpoint' not in r['metric']]
            check('  ... the entry states at the switch are the record\'s, value for value', not inputs,
                  '; '.join('%s %r' % (r['metric'], r['diff']) for r in inputs))

            if tag == 'v27_52ms':
                names = [n for n, ok, _ in result['checks'] if ok]
                check('  ... the replay block\'s timeline check holds on the 52 ms stick samples',
                      'the replay block\'s timeline is the record\'s PX4 times' in names)
        else:
            c = next(r for r in result['rows'] if r['section'] == 'centred')
            found = re.search(r'raw rms of the gyro rows ([\d.]+) deg/s.*own noise ([\d.]+) deg/s', c['note'])
            raw, noise = (float(found.group(1)), float(found.group(2))) if found else (None, None)
            check('  ... the centred rms of the moving means, %.4f deg/s, is the model\'s, the record\'s gyro noise '
                  'left out: the raw rms %s deg/s is the record\'s own noise %s deg/s'
                  % (c['model'], raw, noise),
                  raw is not None and c['model'] < 0.5 * raw and abs(raw - noise) < 0.25 * raw and c['verdict'] == 'ok')

    return made


def test_verdict_names(made):
    """Each verdict the replay requires is one forward_yaw_check.analyze
    writes, found by its text exactly once per leg: a reworded verdict
    breaks this, not a replay."""
    with open(made['v27']['path']) as handle:
        names = [v['name'] for v in json.load(handle)['validity']]

    counts = {label: sum(1 for n in names if n.startswith('다리 1: ') and text in n) for label, text in R.LEG_VERDICTS}
    counts.update({label: sum(1 for n in names if n == text) for label, text in R.RUN_VERDICTS})
    check('every verdict the replay requires is in forward_yaw_check\'s record once (%d)' % len(counts),
          all(c == 1 for c in counts.values()), str(counts))


def worst_fraction(rows, sections=('release', 'speed')):
    """The largest |model - record| / tolerance of the compared values of
    the given sections (by default the release's heading, rates and
    integrator, and the speed)."""
    return max((abs(r['diff']) / r['tol'] for r in rows if r['kind'] == 'value' and r['diff'] is not None
                and r['tol'] and r['section'] in sections), default=None)


def test_release_start(made):
    """From the release: the record's state there, the integrator up to one
    logged sample old, the flags and the pod modes, against the rest of the
    leg. What it cannot have is the integrator at the instant: flown again
    with the source leg's own, the residual is the sample's age. The model
    starts from the record's pod tilts, tilted as they are there, to
    round-off."""
    for tag in ('v27', 'c17'):
        m = made[tag]
        rec = record_of(m)
        result = R.replay(rec, 'release', 0)
        worst = worst_fraction(result['rows'])
        fins = worst_fraction(result['rows'], ('fins',))
        rp = rec.leg['replay']
        source = m['source']
        exact = dict(result['kwargs'] or {}, integ0=source['entry']['release']['integ'])
        runs = [R.measure(rec, R.run_model(exact), 'release')] if result['kwargs'] else []
        true = worst_fraction(R.compare(rec, 'release', runs)) if runs else None
        check('%s from the release: within tolerance; the release\'s heading, rates, integrator and speed at worst %s '
              'of their tolerance, %s with the integrator at the release instant (the record\'s sample is %.3f s old, '
              '%+.1e off); the fins at %s'
              % (tag, '-' if worst is None else '%.3f' % worst, '-' if true is None else '%.3f' % true,
                 rec.leg['release']['entry']['integrator_age_s'],
                 rp['integrator_at_release'] - source['entry']['release']['integ'],
                 '-' if fins is None else '%.3f' % fins),
              result['status'] == 'within tolerance' and worst is not None and true is not None and true < 0.1
              and true <= worst + 1e-12, '%s; %s' % (result['status'], failed(result)))
        # the spec's "start tilt within 2 deg of the record's", which holds by
        # construction: run_leg sets the record's tilts (to round-off, a
        # trip through radians)
        tilts = [r for r in result['entry'] if r['metric'].startswith('at the release, inputs: tilt')]
        check('  ... the model starts from the record\'s pod tilts at the release (%s), to %.1e deg'
              % (', '.join('%s %.2f deg' % (r['metric'].split(': ')[1], r['record']) for r in tilts),
                 max([abs(r['diff']) for r in tilts if r['diff'] is not None], default=float('nan'))),
              tilts and all(r['diff'] is not None and abs(r['diff']) <= 1e-9 for r in tilts))


# ---------------------------------------------------------------------------
# 2. mutations
# ---------------------------------------------------------------------------

def mutated_record(m, tmp, tag, log_change=None, kept_change=None, rec_change=None, events=True, ulog=True):
    """A record of the synthetic `m` with its log changed (a copy), its kept
    summary changed (a copy, in its own directory, beside a copy of the
    events file, or none), or today's measurement changed in memory."""
    out = os.path.join(tmp, 'mutated_' + tag)
    os.makedirs(out)

    with open(m['path']) as handle:
        doc = json.load(handle)

    events_path = os.path.join(out, os.path.basename(m['events_path']))

    if events:
        shutil.copy(m['events_path'], events_path)

    doc['live']['events'] = events_path

    if kept_change is not None:
        kept_change(doc, doc['legs'][0])

    path = os.path.join(out, os.path.basename(m['path']))

    with open(path, 'w') as handle:
        json.dump(doc, handle, indent=1, ensure_ascii=False)

    log = copy.deepcopy(m['log'])

    if log_change is not None:
        log_change(log)

    rec = R.load(path, [1], logs={m['ulg']: log} if ulog else {})[0]

    if rec_change is not None:
        rec_change(rec)

    return rec


def drop(log, topic, keep):
    data = log.data[topic]

    for key in list(data):
        data[key] = np.asarray(data[key])[keep]


def last_index(log, topic, t_us):
    return int(np.searchsorted(log.t(topic), t_us, side='right')) - 1


def quaternion_rpy(roll, pitch, yaw):
    """PX4's FRD-to-NED quaternion of Euler angles, q_z(yaw) q_y(pitch) q_x(roll)."""
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def test_mutations(made, tmp):
    m = made['v27']

    with open(m['path']) as handle:
        tp = json.load(handle)['legs'][0]['times_px4_us']

    acro, on, off = tp['acro_us'], tp['on_us'], tp['off_us']

    def param(name, value):
        def change(log):
            if value is None:
                del log.params[name]
            else:
                log.params[name] = value
        return change

    def gain_changed(log):
        log.changed.append((acro + 10 * S, 'AS_YAWRATE_I', 0.1))

    def tlmax(log):
        log.params['CA_AIRSHIP_TLMAX'] -= 1.0

    def pitch(log):
        att = log.data[fyc.GT_ATT]
        roll, _, yaw = fyc.euler(att['q[0]'], att['q[1]'], att['q[2]'], att['q[3]'])
        q = quaternion_rpy(roll, np.radians(3.5), yaw)

        for i in range(4):
            att['q[%d]' % i] = q[i]

    def integ_first(log):
        i = int(np.searchsorted(log.t('rate_ctrl_status'), acro, side='left'))
        log.data['rate_ctrl_status']['yawspeed_integ'][i] = 0.01

    def throttle(log):
        thrust = log.data['vehicle_thrust_setpoint']
        inside = (thrust['timestamp'] >= on) & (thrust['timestamp'] < off)
        thrust['xyz[0]'] = np.where(inside, thrust['xyz[0]'] - 0.001, thrust['xyz[0]'])

    def entry_unread(log):
        i = last_index(log, fyc.GT_POS, acro)
        log.data[fyc.GT_POS]['vx'][i] = log.data[fyc.GT_POS]['vy'][i] = np.nan

    def integ_old(log):
        t = log.t('rate_ctrl_status')
        drop(log, 'rate_ctrl_status', ~((t > off - S // 2) & (t <= off)))

    def demand_unread(log):
        log.data['vehicle_thrust_setpoint']['xyz[2]'][last_index(log, 'vehicle_thrust_setpoint', off)] = np.nan

    def status_unread(log):
        i = last_index(log, 'control_allocator_status', off)
        log.data['control_allocator_status']['unallocated_torque[2]'][i] = np.nan
        log.data['control_allocator_status']['torque_setpoint_achieved'][i] = np.nan

    def no_verdict(rec):
        rec.doc['validity'] = [v for v in rec.doc['validity'] if '스로틀이' not in v['name']]

    def origin(rec):
        rp = rec.leg['replay']
        rp['stick'][1][0] += 0.2
        rp['stick'][2][0] += 0.2
        rp['end_s'] += 0.2

    def t0(rec):
        rec.leg['replay']['t0_px4_us'] -= 200_000

    def table_clock(rec):
        rec.rows['t_s'] = rec.rows['t_s'] + 0.2

    def block_state(rec):
        rp = rec.leg['replay']
        rp['state'] = dict(rp['state'], v=-rp['state']['v'])

    def start_row(rec):
        state = rec.leg['replay']['state']          # entry.at_acro's own dict
        state['v'] = -state['v']

    def start_tilt(rec):
        # the review's "pod tilt +5 deg": the replay block's and the run_leg
        # block's, which would fly it, against the table's
        rp = rec.leg['replay']
        rp['actuators']['tilt_stbd_deg'] += 5.0
        rp['run_leg']['pods0'][0]['tilt'] += 5.0

    def run_leg_none(rec):
        rec.leg['replay']['run_leg'], rec.leg['replay']['run_leg_unread'] = None, ['pods0[0].mode']

    def run_leg_differs(rec):
        rec.leg['replay']['run_leg']['stick_s'] += 0.004

    def interface(rec):
        for key in ('run_leg', 'run_leg_unread', 'pod_modes', 'pod_modes_at_release'):
            del rec.leg['replay'][key]
        del rec.leg['stick_phase']['r_peak_smoothed_05s_deg_s']

    def other_leg(doc, leg):
        leg['times_px4_us']['on_us'] += 1

    timeline = 'the replay block\'s timeline is the record\'s PX4 times'
    finite_ = 'every number the replay reads is finite'
    run_leg = 'the record\'s run_leg is whole and is forward.replay_kwargs\' reading'
    cases = (
        # (name, start, {log_change / rec_change / kept_change / events / ulog}, the precondition)
        ('the log\'s SYS_AUTOSTART another airframe\'s (2522)', 'acro', {'log_change': param('SYS_AUTOSTART', 2522.0)},
         'SYS_AUTOSTART is the model\'s airframe'),
        ('a gain changed during the log (AS_YAWRATE_I)', 'acro', {'log_change': gain_changed},
         'no gain, allocator parameter, THR_MDL_FAC or gyro cutoff changed during the log'),
        ('CA_AIRSHIP_TLMAX one degree off in the log', 'acro', {'log_change': tlmax},
         'the allocator parameters and THR_MDL_FAC are the model\'s airframe files\''),
        ('IMU_GYRO_CUTOFF 30 Hz in the log', 'acro', {'log_change': param('IMU_GYRO_CUTOFF', 30.0)},
         'IMU_GYRO_CUTOFF is the model gyro\'s'),
        ('the hull pitched 3.5 deg', 'acro', {'log_change': pitch}, '|pitch| under 3 deg at the start'),
        ('the integrator\'s first sample after the switch 0.01', 'acro', {'log_change': integ_first},
         'the integrator starts cleared (first sample after the Acro switch)'),
        ('a gain absent from the log (AS_YAWRATE_P)', 'acro', {'log_change': param('AS_YAWRATE_P', None)}, finite_),
        ('the throttle one count low through the stick', 'acro', {'log_change': throttle},
         'verdict passed: throttle constant'),
        ('the groundtruth velocity unread at the switch', 'acro', {'log_change': entry_unread}, finite_),
        ('the integrator\'s sample at the release 0.6 s old', 'release', {'log_change': integ_old},
         'the integrator sample at the release is at most one logged interval old'),
        ('the pods\' demand unread at the release (thrust z NaN)', 'release', {'log_change': demand_unread},
         'the record\'s release state and the pod modes there are read'),
        ('the allocator status unread at the stick end (NaN: no flag in forward_yaw_check)', 'release',
         {'log_change': status_unread}, finite_),
        ('a verdict missing from today\'s measurement (the throttle\'s)', 'acro', {'rec_change': no_verdict},
         'verdict passed: throttle constant'),
        ('the replay block\'s time origin 0.2 s early', 'acro', {'rec_change': origin}, timeline),
        ('t0 0.2 s before the Acro switch', 'acro', {'rec_change': t0}, timeline),
        ('the table\'s t_s off by 0.2 s', 'acro', {'rec_change': table_clock},
         'the table\'s rows: increasing times, t_s from the stick onset (t_acro_s from the switch), the phases'),
        ('the replay block\'s state not the entry state (the sway\'s sign)', 'acro', {'rec_change': block_state},
         'the replay block\'s states are the record\'s entry and release states'),
        ('the start state not the table\'s row (the sway\'s sign)', 'acro', {'rec_change': start_row},
         'the table\'s row at the start is the summary\'s start state'),
        ('the starboard pod\'s start tilt 5 deg off the table\'s (the replay and run_leg blocks)', 'acro',
         {'rec_change': start_tilt}, 'the table\'s row at the start is the summary\'s start state'),
        ('a run_leg block analyze could not fill', 'acro', {'rec_change': run_leg_none}, run_leg),
        ('a run_leg block that disagrees with the rest of the record (stick 4 ms longer)', 'acro',
         {'rec_change': run_leg_differs}, run_leg),
        ('today\'s measurement without the model interface', 'acro', {'rec_change': interface},
         'the record has forward_yaw_check\'s model interface of 2026-10-01'),
        ('a kept summary of another leg (its onset 1 us off)', 'acro', {'kept_change': other_leg},
         'the kept summary is this log\'s leg: its plant, and the leg\'s PX4 times today\'s'),
        ('the ULog not there', 'acro', {'ulog': False}, 'the ULog is read'),
        ('the events file not there', 'acro', {'events': False},
         'the run\'s events file is read (its timeline and plan)'),
    )

    for i, (name, start, how, expected) in enumerate(cases):
        rec = mutated_record(m, tmp, str(i), **how)
        result = R.replay(rec, start, 0)
        names = failed(result)
        check('mutated: %s: refused%s, by "%s"' % (name, ' from the release' if start == 'release' else '', expected),
              result['status'] == 'refused' and expected in names and result['model'] is None,
              '%s; failed: %s' % (result['status'], names))

    # A kept summary is not the replay's input: edited, or written by
    # another forward_yaw_check, the leg is replayed from today's
    # measurement of the log, the summary's differences reported
    def sway(doc, leg):
        for state in (leg['replay']['state'], leg['entry']['at_acro']):
            state['v'] = -state['v']
        leg['replay']['run_leg']['v0'] = -leg['replay']['run_leg']['v0']

    def other_code(doc, leg):
        doc['definitions']['fins by name'] += ' (measured by another forward_yaw_check)'

    for i, (name, change, paths) in enumerate((
            ('the sway\'s sign flipped through it', sway,
             {'legs[leg 1].replay.state.v', 'legs[leg 1].entry.at_acro.v', 'legs[leg 1].replay.run_leg.v0'}),
            ('one definition worded otherwise (another forward_yaw_check)', other_code, {'definitions.fins by name'}))):
        rec = mutated_record(m, tmp, 'kept%d' % i, kept_change=change)
        checks = R.preconditions(rec, 'acro')
        found = {p for p, _, _ in R.kept_differences(rec)}
        kwargs, _ = R.replay_inputs(rec, 'acro')
        check('a kept summary with %s: every precondition holds, the replay flies the log\'s v0 %+.3f m/s, the '
              'differences are named (%s)' % (name, kwargs['v0'], ', '.join(sorted(found))),
              all(ok for _, ok, _ in checks) and found == paths and kwargs['v0'] == m['kwargs']['v0'],
              str([n for n, ok, _ in checks if not ok]))


# ---------------------------------------------------------------------------
# 3. what the comparison alone catches
# ---------------------------------------------------------------------------

def fly_against(rec, kwargs):
    """The record against a model leg flown from `kwargs` (the record's
    checks bypassed): the failed numbers' names."""
    model = R.run_model(kwargs)
    runs = [R.measure(rec, model, 'acro')]
    return [r['metric'] for r in R.compare(rec, 'acro', runs) if r['verdict'] == 'FAIL']


def test_comparison_alone(made):
    """The record's checks catch the wrong inputs above before anything
    flies. Flown anyway, which of them would the comparison itself see? On
    the noisy record too, where the centred rms compares moving means."""
    m = made['v27']
    rec, kwargs = record_of(m), m['kwargs']
    sway_name = 'the sway\'s sign flipped (0.04 m/s to port instead of to starboard)'
    wrong = {
        sway_name: dict(kwargs, v0=-kwargs['v0']),
        'one throttle count low (0.03025 for 0.03125)': dict(kwargs, throttle=kwargs['throttle'] - 0.001),
        'the stick 0.2 s late': dict(kwargs, centred_s=kwargs['centred_s'] + 0.2),
    }
    seen = {name: fly_against(rec, copy.deepcopy(kw)) for name, kw in wrong.items()}
    print('      what the comparison sees: ' + '; '.join('%s: %s' % (n, ', '.join(f) or 'nothing')
                                                        for n, f in seen.items()))
    check('flown with the sway\'s sign flipped, the comparison sees it (the centred r rms or the stick\'s numbers)',
          bool(seen[sway_name]))
    check('flown one throttle count low, the comparison sees it in the speed',
          any(f.startswith('u, ') for f in seen['one throttle count low (0.03025 for 0.03125)']))
    check('flown with the stick 0.2 s late, the comparison sees it (the stick\'s rows and windows move)',
          bool(seen['the stick 0.2 s late']))
    noisy = made['v27_noisy']
    rec = record_of(noisy)
    kw = copy.deepcopy(rec.leg['replay']['run_leg'])
    seen = fly_against(rec, dict(kw, v0=-kw['v0']))
    print('      on the noisy record, %s: %s' % (sway_name, ', '.join(seen) or 'nothing'))
    check('on the record with gyro noise, flown with the sway\'s sign flipped, the centred rms of the moving means '
          'still sees it', any(f.startswith('r rms of the') for f in seen), ', '.join(seen))


# ---------------------------------------------------------------------------
# 4. the model's own metrics against the replay's measurement of it
# ---------------------------------------------------------------------------

def cross_pairs(model, numbers):
    """[(name, model's own, the replay's measurement, tolerance)]: forward.
    run_leg's metrics on every 4 ms step against forward_yaw_check's
    numbers of the same run through model_log. The tolerances allow for the
    sampling (20 ms rows, 8 ms gyro samples, a 200 ms integrator and
    allocator status) and the gyro's filter, which the model's metrics read
    on the true rate; a sign or axis broken in the mapping is far past them.
    A heading change is read on the rows at or before its two instants, up
    to a row (ROW_S) before each: the rate there times a row, at each end."""
    mm, sp, rl = model['metrics'], numbers['stick_phase'], numbers['release']
    e = model['entry']
    marks = rl['r_mean1s_deg_s']
    end = [k for k in marks if k not in ('+10', '+60')][0]
    rows = lambda a, b: 0.01 + ROW_S * (abs(e[a]['r']) + abs(e[b]['r']))
    pairs = [('r mean over the stick\'s last 5 s', mm['r_mean_last5'], sp['r_mean_last5_deg_s'], 0.01),
             ('r peak, 0.5 s moving mean', mm['r_peak_smoothed'], sp['r_peak_smoothed_05s_deg_s'], 0.02),
             ('heading change in the stick', mm['heading_stick'], sp['heading_change_deg'], rows('stick', 'release')),
             ('heading change in the release', mm['heading_release'], rl['heading_change_deg'], rows('release', 'end')),
             ('integrator at the stick end', mm['integ_stick_end'], sp['integrator_end'], 0.003),
             ('u at the stick start', mm['u_mean05_stick_start'], sp['u_start'], 0.002),
             ('u at the stick end', mm['u_mean05_stick_end'], sp['u_end'], 0.002),
             ('u at the release end', mm['u_mean05_release_end'], rl['u_end'], 0.002),
             ('r, 1 s mean at +10 s', mm['r_mean1s_rel_10'], marks['+10'], 0.005),
             ('r, 1 s mean at +60 s', mm['r_mean1s_rel_60'], marks.get('+60'), 0.005),
             ('r, 1 s mean at the end', mm['r_mean1s_rel_end'], marks[end], 0.005),
             ('saturation flag + at the stick end', mm['sat_pos_stick_end'], sp['flag_pos_end'], None),
             ('saturation flag - at the stick end', mm['sat_neg_stick_end'], sp['flag_neg_end'], None),
             ('unallocated yaw sign at the stick end', int(np.sign(mm['unalloc_stick_end'])),
              int(np.sign(sp['unallocated_end'])), None)]

    if model['setup']['vectored']:
        pairs += [('pod flips in the stick', mm['rev_stick'], sp['reversals']['flips'], None),
                  ('pod flips in the release', mm['rev_release'], rl['reversals']['flips'], None)]
    else:
        pairs += [('tail flips in the stick', mm['tail_flips_stick'], sp['reversals']['tail_flips'], None),
                  ('tail flips in the release', mm['tail_flips_release'], rl['reversals']['tail_flips'], None)]

    for seg, block in (('stick', sp), ('release', rl)):
        measured = {f['name']: f['max_ratio'] for f in block['fins']}

        for name, ratio in sorted(mm['fin_max_alpha_ratio_%s' % seg].items()):
            pairs.append(('%s fin max ratio, %s' % (name, seg), ratio, measured.get(name), 0.01 + 0.01 * ratio))

    return pairs


def cross_misses(pairs):
    out = []

    for name, mine, theirs, tol in pairs:
        if mine is None and theirs is None:
            continue

        if tol is None:
            ok = mine == theirs
        else:
            ok = R.finite(mine) and R.finite(theirs) and abs(mine - theirs) <= tol

        if not ok:
            out.append('%s %s / %s' % (name, mine, theirs))

    return out


def test_cross_check(made):
    """The synthetic records are written by model_log, and the replay measures
    the model through it too: a frame or sign error there would be on both
    sides and the replay still exact. The model's own metrics on its 4 ms
    steps see the log's mapping from outside."""
    worst = {}

    for tag, *_ in CASES:
        result = made[tag].get('result')

        if not result or result['model'] is None:
            check('%s: the replay ran' % tag, False)
            continue

        pairs = cross_pairs(result['model'], result['runs'][0][1])
        misses = cross_misses(pairs)
        fins = [abs(a - b) / b for name, a, b, tol in pairs if 'fin max ratio' in name and R.finite(a) and R.finite(b)]
        worst[tag] = max(fins) if fins else None
        check('%s: the model\'s own metrics on its 4 ms steps are the replay\'s measurement of the run through the log '
              '(%d numbers; the fins within %s)' % (tag, len(pairs), '-' if worst[tag] is None else
                                                    '%.2f %%' % (100 * worst[tag])), not misses, '; '.join(misses))

    # the same check, with a sign broken in the log's mapping
    m = made['v27']
    rec, model = record_of(m), m['result']['model']
    rows, quaternion = R.model_rows, R.quaternion

    def broken(name, replacement):
        setattr(R, name, replacement)

        try:
            return cross_misses(cross_pairs(model, R.measure(rec, model, 'acro')[1]))
        finally:
            R.model_rows, R.quaternion = rows, quaternion

    def sway(leg, start):
        out = rows(leg, start)
        out['v'] = -out['v']
        return out

    misses = broken('model_rows', sway)
    check('with the sway\'s sign broken in the log, the cross-check sees it (%s)' % '; '.join(misses), bool(misses))
    misses = broken('quaternion', lambda roll, yaw: quaternion(roll, -yaw))
    check('with the heading\'s sign broken in the log, the cross-check sees it (%s)' % '; '.join(misses[:3]),
          bool(misses))


# ---------------------------------------------------------------------------
# 5. the parts
# ---------------------------------------------------------------------------

def test_row_logic():
    row = R._row

    def v(record, models, tol, noisy=False):
        return row('s', 'm', 'u', record, models, 'value', tol, noisy=noisy)

    check('a value: within its tolerance of the noise-free run, ok; past it, FAIL; with seeds and not read through '
          'the gyro, still the noise-free run\'s verdict',
          v(1.0, [1.04], 0.05)['verdict'] == 'ok' and v(1.0, [1.06], 0.05)['verdict'] == 'FAIL'
          and v(1.0, [1.06, 0.98, 1.00], 0.05)['verdict'] == 'FAIL')
    r = v(1.0, [1.06, 1.00, 1.02], 0.05, noisy=True)
    check('a [noise] value: within its tolerance of the noise-free value +- 2 s of the seeds, ok, the noise-free '
          'verdict kept beside (band +-%.4f)' % r['band'],
          r['verdict'] == 'ok' and r['verdict_noise_free'] == 'FAIL' and abs(r['band'] - 2 * math.sqrt(0.0026)) < 1e-12)
    two, six = R.noise_band(1.0, [1.1, 0.9]), R.noise_band(1.0, [1.1, 0.9] * 3)
    check('the band does not widen with more seeds from the same spread (+-%.4f from two, +-%.4f from six), and is 0 '
          'without any' % (two[2], six[2]),
          all(abs(a - b) < 1e-12 for a, b in zip(two, six)) and R.noise_band(1.0, [])[2] == 0.0
          and R.noise_band(None, [1.0]) == (None, None, None))
    check('a value missing on one side fails; on both it is n/a',
          v(None, [1.0], 0.1)['verdict'] == 'FAIL' and v(1.0, [None], 0.1)['verdict'] == 'FAIL'
          and v(None, [None], 0.1)['verdict'] == 'n/a' and v(float('nan'), [1.0], 0.1)['verdict'] == 'FAIL')
    c = lambda record, models, allow: row('s', 'm', '', record, models, 'count', allow=allow)
    check('a count: equal, or within its allowance, on the noise-free run; a missing one fails and keeps its allowance',
          c(2, [2], 0)['verdict'] == 'ok' and c(2, [3], 0)['verdict'] == 'FAIL' and c(2, [3], 1)['verdict'] == 'ok'
          and c(2, [3, 2], 0)['verdict'] == 'FAIL' and c(2, [None], 1)['verdict'] == 'FAIL'
          and c(2, [None], 1)['tol'] == 1)
    b = lambda record, models, note='', noisy=False: row('s', 'm', '', record, models, 'stalled', note=note,
                                                       noisy=noisy)['verdict']
    check('a verdict: equal to the noise-free run\'s, a [noise] one too (a seed that agrees does not pass it: more '
          'seeds would pass more); indeterminate where the note says so',
          b(True, [True]) == 'ok' and b(True, [False]) == 'FAIL' and b(True, [False, True]) == 'FAIL'
          and b(True, [False, True], noisy=True) == 'FAIL' and b(True, [True, False], noisy=True) == 'ok'
          and b(True, [False], 'indeterminate: ratio 0.97') == 'indeterminate')
    u = lambda models: row('s', 'm', 'deg/s', None, models, 'upper', 0.05)['verdict']
    check('an upper bound: the noise-free run\'s value', u([0.04]) == 'ok' and u([0.06]) == 'FAIL'
          and u([0.06, 0.045]) == 'FAIL' and u([0.04, 0.06]) == 'ok' and u([None]) == 'FAIL')
    rel = lambda record, models: row('s', 'm', '', record, models, 'rel', 0.01)['verdict']
    check('a relative tolerance: 1 % of the record', rel(10.0, [10.09]) == 'ok' and rel(10.0, [10.11]) == 'FAIL'
          and rel(-10.0, [-10.09]) == 'ok')
    check('a tolerance of a percentage takes the record\'s magnitude and keeps its floor',
          R.pct(0.1, 0.03)(-10.0) == 0.3 and R.pct(0.1, 0.03)(1.0) == 0.1 and R.pct(0.1, 0.03)(None) == 0.1)


def test_fin_tolerance():
    """A fin's ratio within max(0.05, 5 % of the record's); the stalled
    verdict indeterminate where either ratio is 0.95..1.05, or the model's
    ratio band from the seeds holds 1; a ratio at the broadside bound (90 deg
    of alpha_stall) marked so."""
    fins = lambda *ratios: [{'name': n, 'max_ratio': r, 'no_flow_rows': 0} for n, r in zip(('dorsal', 'ventral'), ratios)]
    t = np.arange(0, 100, dtype=np.int64) * 20_000
    rec = types.SimpleNamespace(
        leg={'times_px4_us': {'on_us': 0, 'off_us': 1_000_000, 'release_end_us': 1_980_000},
             'stick_phase': {'fins': fins(0.13, 1.20)}, 'release': {'fins': fins(0.97, 5.464)}},
        doc={'plant': {'vertical_fins': [{'name': 'dorsal', 'alpha_stall': math.radians(16.169)},
                                         {'name': 'ventral', 'alpha_stall': math.radians(16.169)}]}},
        rows={'t_px4_us': t})
    model = {'stick_phase': {'fins': fins(0.115, 1.27)}, 'release': {'fins': fins(1.01, 5.566)}}
    rows = {r['metric']: r for r in R.fin_rows(rec, 'acro', [model])}
    verdicts = {k: r['verdict'] for k, r in rows.items()}
    check('fins: 0.130 / 0.115 of stall within the 0.05 floor (-11 % of the ratio), 1.20 / 1.27 past 5 %; 0.97 / '
          '1.01 stalls indeterminately; 5.464 / 5.566 (2517\'s release, at the broadside bound) within 5 % and marked',
          verdicts == {'dorsal fin max |alpha| / alpha_stall, stick': 'ok', 'dorsal fin stalled, stick': 'ok',
                       'ventral fin max |alpha| / alpha_stall, stick': 'FAIL', 'ventral fin stalled, stick': 'ok',
                       'dorsal fin max |alpha| / alpha_stall, release': 'ok',
                       'dorsal fin stalled, release': 'indeterminate',
                       'ventral fin max |alpha| / alpha_stall, release': 'ok', 'ventral fin stalled, release': 'ok'}
          and 'broadside' in rows['ventral fin max |alpha| / alpha_stall, release']['note']
          and 'broadside' not in rows['dorsal fin max |alpha| / alpha_stall, release']['note'], str(verdicts))
    # a stall the noise decides: the model's ratio band across 1
    rec.leg['stick_phase']['fins'] = fins(1.10, 0.50)
    model = {'stick_phase': {'fins': fins(0.90, 0.50)}, 'release': {'fins': fins(1.01, 5.566)}}
    seeds = [{'stick_phase': {'fins': fins(x, 0.50)}, 'release': {'fins': fins(1.01, 5.566)}} for x in (1.02, 0.78)]
    key = 'dorsal fin stalled, stick'
    alone = {r['metric']: r for r in R.fin_rows(rec, 'acro', [model])}[key]
    banded = {r['metric']: r for r in R.fin_rows(rec, 'acro', [model] + seeds)}[key]
    check('fins: stalled at 1.10 in the record and not at 0.90 in the model, FAIL on the noise-free run alone; with '
          'seeds at 1.02 and 0.78, indeterminate [%s]' % banded['note'],
          alone['verdict'] == 'FAIL' and banded['verdict'] == 'indeterminate' and 'holds 1' in banded['note'])


def test_centred():
    """The centred rms compares 0.5 s moving means: white gyro noise on the
    record's rows is left out of it (and reported as the record's own), an
    offset between the two is not."""
    rng = np.random.default_rng(3)
    a, b = 40 * S, 43 * S
    t = np.arange(a - 2 * S, b + 2 * S, 20_000, dtype=np.int64)
    slow = 0.05 * np.sin(2 * np.pi * (t - a) / 6e6)
    noise = rng.normal(0.0, 0.04, len(t))
    rec = types.SimpleNamespace(leg={'times_px4_us': {'acro_us': a, 'on_us': b}},
                                rows={'t_px4_us': t, 'r_deg_s': slow + noise})
    clean = R.centred(rec, {'t_px4_us': t.astype(float), 'r_deg_s': slow})
    shifted = R.centred(rec, {'t_px4_us': t.astype(float), 'r_deg_s': slow + 0.06})
    check('centred: 0.04 deg/s of white noise on the record\'s rows gives %.4f deg/s on the moving means (raw %.4f, '
          'the record\'s own noise read %.4f); a 0.06 deg/s offset gives %.4f'
          % (clean['rms'], clean['raw'], clean['noise'], shifted['rms']),
          clean['rms'] < 0.015 and abs(clean['raw'] - 0.04) < 0.008 and abs(clean['noise'] - 0.04) < 0.008
          and abs(shifted['rms'] - 0.06) < 0.015 and clean['smoothed_rows'] == 125 and clean['rows'] == 150)
    rec.rows['r_deg_s'] = np.where(t == t[110], np.nan, rec.rows['r_deg_s'])
    check('centred: an unread row leaves it unread', R.centred(rec, {'t_px4_us': t.astype(float), 'r_deg_s': slow})
          is None)


def test_flip_times():
    """_flip_times changes direction where forward_yaw_check's own counters
    count a flip."""
    rng = np.random.default_rng(7)
    t = np.arange(0, 3000, dtype=np.int64) * 20_000
    tilt = np.cumsum(rng.normal(0.0, 8.0, len(t)))
    tilt = (tilt + 180.0) % 360.0 - 180.0
    tail = np.cumsum(rng.normal(0.0, 0.01, len(t)))
    a, b = int(t[400]), int(t[2600])
    pods = R._flip_times(t, tilt, 'pod')
    into, back, _ = fyc.pod_reversals(t, tilt, a, b)
    tails = R._flip_times(t, tail, 'tail')
    check('pod flip times: as many inside a window as forward_yaw_check.pod_reversals counts (%d)' % (into + back),
          sum(1 for x in pods if a <= x < b) == into + back and into + back > 5)
    check('tail flip times: as many inside a window as forward_yaw_check.sign_reversals counts (%d)'
          % fyc.sign_reversals(t, tail, a, b),
          sum(1 for x in tails if a <= x < b) == fyc.sign_reversals(t, tail, a, b) and len(tails) > 5)


def test_log_convention(made):
    """The model's log: a plant topic's sample at T is the step that ended at
    or before T, a controller output's the cycle at or before T; the motor
    commands derived from the trace and the allocator status the hook kept
    are the model's own; the quaternion is what forward_yaw_check.euler reads
    back."""
    source = made['v27']['source']
    rows = R.model_rows(source, 'acro')
    tr = source['trace']
    k = 2500                                        # trace row k is t = (k + 1) DT
    t_cycle = tr['t'][k] - R.DT
    tau = np.array([t_cycle, tr['t'][k]])
    i_cmd = np.searchsorted(rows['t'], tau + R.DT + 1e-9, side='right') - 1
    i_plant = np.searchsorted(rows['t'], tau + 1e-9, side='right') - 1
    check('a controller output sampled at the cycle\'s own time is that cycle\'s (the trace row one step later); a '
          'plant quantity sampled at a step\'s end is that step\'s',
          rows['integ'][i_cmd[0]] == tr['integ'][k] and rows['u'][i_plant[1]] == tr['u'][k])
    worst = 0.0

    for name in ('stick', 'release', 'end'):
        e = source['entry'][name]
        j = int(np.argmin(np.abs(rows['t'] - e['t'])))
        worst = max(worst, abs(rows['motor1'][j] - e['actuators']['motors'][0]),
                    abs(rows['motor2'][j] - e['actuators']['motors'][1]))

    check('the pod motor commands derived from the trace (AirshipPod::drive) are the model\'s own at the stick, the '
          'release and the end (worst %.1e)' % worst, worst < 1e-12)

    def against_loop(leg):
        """The allocator status the hook kept, against the yaw sign the loop
        hands its controller (closed_loop.Loop._unallocated, each series
        row's 'unalloc': the same cycle's, a row at t holding the cycle at t
        - DT): (series rows, rows that differ, yaw-flagged rows, every
        yaw-flagged row not achieved, rows not achieved with the yaw 0)."""
        rows = R.model_rows(leg, 'acro')
        index = [int(round(s['t'] / R.DT)) for s in leg['series']]
        differ = sum(1 for i, s in zip(index, leg['series']) if rows['unalloc'][i] != s['unalloc'])
        flagged = rows['unalloc'] != 0.0
        return (len(index), differ, int(flagged.sum()), bool(np.all(rows['achieved'][flagged] == 0.0)),
                int(np.sum((rows['achieved'] == 0.0) & ~flagged)))

    # torque_setpoint_achieved is over the three torque axes (CA:686-688): a
    # roll the pods cannot give clears it with the yaw sign 0
    for tag in ('v27', 'c17'):
        n, differ, flagged, gated, other = against_loop(made[tag]['source'])
        check('%s: the allocator\'s yaw sign in the log is the loop\'s own on all %d series rows (%d differ; %d of the '
              'rows yaw-flagged, each not achieved; %d not achieved with the yaw 0, another axis short)'
              % (tag, n, differ, flagged, other), n > 0 and differ == 0 and gated)

    # the Cloudship's tail saturating: the hook reads the pods' own
    # AirshipAllocator, the loop the tail chain's map
    kwargs = source_kwargs(C17, 2.0, 1.0, 3.0 / 256.0, 0.0, 3 * S, 13 * S, 33 * S)
    kwargs['params'] = dict(kwargs['params'], AS_YAWRATE_P=8.0)
    n, differ, flagged, gated, other = against_loop(R.run_model(kwargs))
    check('Cloudship 2517 with AS_YAWRATE_P 8, its tail saturating: the yaw sign the hook reads off the pods\' '
          'AirshipAllocator is the TailChain\'s the loop hands its controller on all %d series rows (%d differ, %d '
          'rows flagged)' % (n, differ, flagged), n > 0 and differ == 0 and flagged > 0 and gated and other == 0)

    roll, yaw = np.radians([-12.0, 3.0, 40.0]), np.radians([-170.0, 10.0, 179.0])
    back = fyc.euler(*R.quaternion(roll, yaw))
    check('quaternion() through forward_yaw_check.euler gives the roll, no pitch and the heading back',
          np.allclose(back[0], roll, atol=1e-12) and np.allclose(back[1], 0.0, atol=1e-12)
          and np.allclose(back[2], yaw, atol=1e-12))
    pitched = fyc.euler(*quaternion_rpy(roll, np.radians(3.5), yaw))
    check('quaternion_rpy() (the pitch mutation\'s) gives its three angles back',
          np.allclose(pitched[1], np.radians(3.5), atol=1e-12) and np.allclose(pitched[0], roll, atol=1e-12)
          and np.allclose(pitched[2], yaw, atol=1e-12))


def test_cli(made, tmp):
    """The command line on a synthetic record (its ULog read from disk by
    read_ulog, stubbed to give the log in memory for its path), on one
    whose ULog is not there, on a file that is not there and a leg the
    summary does not have."""
    m = made['v27']
    out = os.path.join(tmp, 'cli')

    def run(*argv):
        buf = io.StringIO()

        with contextlib.redirect_stdout(buf):
            code = R.main(list(argv))

        return code, buf.getvalue()

    original = R.read_ulog
    open(m['ulg'], 'w').close()             # the path the summary names, as a run leaves its ULog
    R.read_ulog = lambda path: m['log'] if os.path.abspath(path) == os.path.abspath(m['ulg']) else original(path)

    try:
        code, text = run(m['path'], '--seeds', '0', '--start', 'both', '--out', out)
    finally:
        R.read_ulog = original
        os.remove(m['ulg'])

    written = [f for f in os.listdir(out)] if os.path.isdir(out) else []
    legs = text.count(': within tolerance on all ')
    ok = code == 0 and legs == 2 and 'identical to today' in text and len(written) == 1
    check('CLI on the synthetic record from both starts: exit 0, both within tolerance (%d), the kept summary '
          'identical, a CSV written' % legs, ok, '' if ok else 'exit %s; %s' % (code, text[-300:]))

    if written:
        with open(os.path.join(out, written[0])) as handle:
            header = handle.readline().strip().split(',')
            body = handle.read().splitlines()

        check('  ... the CSV has every column and the preconditions, the entry states and the numbers',
              tuple(header) == R.CSV_COLUMNS and any(',precondition,' in line for line in body)
              and any(',entry,' in line for line in body) and any(',stick,' in line for line in body))

    code, text = run(m['path'], '--seeds', '0', '--out', out)
    check('CLI without the ULog: refused, exit 2', code == 2 and 'REFUSED' in text and 'no ULog' in text,
          'exit %s' % code)
    code, _ = run(os.path.join(tmp, 'absent_summary.json'), '--out', out)
    check('CLI on a summary that is not there: exit 2', code == 2)
    code, _ = run(m['path'], '--leg', '7', '--out', out)
    check('CLI on a leg the summary does not have: exit 2', code == 2)


# ---------------------------------------------------------------------------
# 6. the real records and the probe
# ---------------------------------------------------------------------------

REAL = {
    'aborted 2517': ('forward_cloudship_am_fin_drag_20261001_133538_summary.json', 2,
                     ('verdict passed: timeline', 'verdict passed: throttle constant', 'verdict passed: durations',
                      'verdict passed: airborne and armed')),
    'aborted 2512': ('forward_cloudship_fin_20261001_133119_summary.json', 2,
                     ('verdict passed: timeline', 'verdict passed: durations', 'verdict passed: airborne and armed')),
}
REAL_FULL = 'forward_airship_vectored_independent_am_fin_drag_20261001_131350_summary.json'


def test_real_records():
    for name, (summary, leg, expected) in REAL.items():
        path = os.path.join(SITL_OUT, summary)

        if not os.path.exists(path):
            print('SKIP  %s: %s is not in %s' % (name, summary, SITL_OUT))
            continue

        result = R.replay(R.load(path, [leg])[0], 'acro', 0)
        names = failed(result)
        check('real record, %s leg %d (the run stopped at 5 m): refused, by %s' % (name, leg, ', '.join(expected)),
              result['status'] == 'refused' and all(e in names for e in expected), str(names))
        first = R.preconditions(R.load(path, [1])[0], 'acro')
        check('  ... while its leg 1 holds every precondition', all(ok for _, ok, _ in first),
              str([n for n, ok, _ in first if not ok]))

    path = os.path.join(SITL_OUT, REAL_FULL)

    if not os.path.exists(path):
        print('SKIP  the 2527 run is not in %s' % SITL_OUT)
        return

    result = R.replay(R.load(path, [1])[0], 'acro', 0)
    compared = [r for r in result['rows'] if r['verdict'] in ('ok', 'FAIL', 'indeterminate')]
    check('real record, 2527 leg 1 (2 m/s, full stick): replayed within tolerance on all %d compared numbers'
          % len(compared), result['status'] == 'within tolerance', '%s; %s' % (result['status'], failed(result)))
    print('      (%s)' % R.kept_text(result))


def probe_k_from_step0a():
    """(SITL, model) hold torque per rad/s of the 2527 probe as step0a.probes()
    printed them into forward_out_20261001.txt, or None."""
    path = os.path.join(HERE, 'forward_out_20261001.txt')

    if not os.path.exists(path):
        return None

    with open(path, encoding='utf-8') as handle:
        text = handle.read()

    block = text.split('10_39_42.ulg', 1)[-1]
    found = re.search(r'hold torque per rad/s over the stick\'s last 2 s .*?: SITL ([\d.]+), model ([\d.]+)', block)
    return (float(found.group(1)), float(found.group(2))) if found else None


def test_probe(tmp):
    """The 2527 SITL probe (scratchpad probe_fwd.py: no events, a 40 s
    release): its kept summary made by forward_yaw_check without a timeline,
    the replay measures its ULog the same way and refuses it. Its hold
    torque per rad/s on today's rows, against step0a.probes()' own on the
    log's samples."""
    ulg = os.path.join(SITL_OUT, 'probe_fwd_2527_20261001.ulg')
    printed = probe_k_from_step0a()

    if not os.path.exists(ulg) or printed is None:
        print('SKIP  the probe: %s or its step0a line is not there' % ulg)
        return

    out = os.path.join(tmp, 'probe')

    with contextlib.redirect_stdout(io.StringIO()):
        log = fyc.Log.read(ulg)
        doc, _ = fyc.analyze(log, fyc.model_of(log), out_dir=out, stamp='probe', release_s=40.0)

    path = os.path.join(out, 'probe_summary.json')

    with open(path, 'w') as handle:
        json.dump(doc, handle, indent=1, ensure_ascii=False, default=float)

    rec = R.load(path)[0]                   # its log: the summary's own path, the probe's ULog
    result = R.replay(rec, 'acro', 0)
    names = failed(result)
    check('the 2527 probe (no events): refused, by the events file and the timeline and stick verdicts it does not '
          'have', result['status'] == 'refused' and 'verdict passed: timeline' in names
          and 'verdict passed: stick as planned' in names
          and 'the run\'s events file is read (its timeline and plan)' in names, str(names))
    tp = rec.leg['times_px4_us']
    k, r = R._hold_k(rec.rows['t_px4_us'], rec.rows['torque_z'], rec.rows['r_deg_s'],
                     tp['off_us'] - int(R.HOLD_K_WINDOW_S * S), tp['off_us'])
    check('the probe\'s hold torque per rad/s on today\'s rows, %.3f at r %.3f deg/s, is step0a.probes()\' %.3f '
          'on the log\'s samples within 1 %% (%+.2f %%)' % (k, r, printed[0], 100 * (k / printed[0] - 1)),
          abs(k / printed[0] - 1) < 0.01)


def main():
    quick = '--quick' in sys.argv[1:]
    tmp = tempfile.mkdtemp(prefix='forward_replay_test_')

    try:
        made = test_synthetic(tmp)
        test_verdict_names(made)
        test_release_start(made)
        test_mutations(made, tmp)
        test_comparison_alone(made)
        test_cross_check(made)
        test_row_logic()
        test_fin_tolerance()
        test_centred()
        test_flip_times()
        test_log_convention(made)
        test_cli(made, tmp)

        if not quick:
            test_real_records()
            test_probe(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('\n%d failed' % len(FAILS) + (': ' + ', '.join(FAILS) if FAILS else ''))
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
