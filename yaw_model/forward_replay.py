#!/usr/bin/env python3
"""The 0b replay (D1 plan, d1_universal_architecture_20261001.md "4) 단계"
item 0b): one leg of a forward_yaw_check record flown again on the yaw model
(forward.py) from the record's own entry state, and every number the record
holds measured on the model the way the record measured it on SITL, against
the replay spec's tolerances (the 2026-10-01 interface review,
result.iface.replay_spec).

    python3 forward_replay.py ../sitl_out/forward_<model>_<stamp>_summary.json [--leg n] [--seeds 3]
    python3 forward_replay.py <summary> --start release      # from the stick's release
    python3 forward_replay.py <summary> --start both --out <dir>

What it reads. The summary names the run: its plant, its stamp, its ULog
and its events file (the live timeline and the plan). The replay measures
the record itself, with today's forward_yaw_check.analyze on that ULog and
events file (into a scratch directory, removed after: analyze writes each
leg's CSV there), and takes the leg's rows from forward_yaw_check.leg_table
on the same log. The code that measures the record is therefore the code
that measures the model, whatever the summary's age: no summary has to be
redone after forward_yaw_check changes. The kept summary is held against
that measurement: it must be of the same leg (its plant, and the leg's PX4
times, which the log's mode changes and stick edges fix), and every value
it holds otherwise is reported where it differs, never used (a summary
written by an older forward_yaw_check, or edited, shows there). Without
the ULog, or without the events file a live run's summary names, a leg is
refused.

Preconditions. A leg is refused, never replayed from a guess, unless all of
these hold (the spec's, and today's measurement's own consistency):
  - the ULog and the events file are read, today's analysis has the leg,
    and the kept summary is of that leg
  - its validity verdicts passed: the timeline, the stick, the throttle, the
    integrator cleared at the Acro entry, the steady entry, the durations,
    airborne and armed; and the run's topic-rate and gyro-cutoff verdicts
  - SYS_AUTOSTART is the model's airframe; no gain, allocator parameter,
    THR_MDL_FAC or IMU_GYRO_CUTOFF changed during the log
  - CA_AIRSHIP_TLMIN/TLMAX/TLT_R/GRP/TAIL/CS_K, CA_SV_CS_COUNT, CA_R_REV and
    THR_MDL_FAC as the model's airframe files give them (and, vectored,
    vectored_actuation.load_params); IMU_GYRO_CUTOFF as the model's gyro
  - |pitch| under 3 deg at the start; every number read finite, the
    allocator status the stick-end flags come from included (a NaN status
    reads as no flag in forward_yaw_check)
  - the stick's setpoint from the logged gains within 1e-3 deg/s of the
    logged one; every thrust setpoint within half a count of the replayed
    throttle, on the record's count and the plan's
  - the record has forward_yaw_check's model interface of 2026-10-01 (the
    run_leg block, the pod modes, the time-window peak, the fins by name
    with their flow, the tail flips on the torque, t_acro_s)
  - the measurement agrees with itself: the replay block against the leg's
    PX4 times, its entry and release states; the table's t_s and t_acro_s,
    phases and start row
  - from the Acro switch: the record's run_leg block whole, and equal to
    forward.replay_kwargs' reading of the same record except for the pod
    modes (the record's, from the log) and integ0 (0, the reset it shows)
  - from the release: the integrator sample at most one logged interval
    old, the pods' modes there read
The spec's "each pod's start tilt within 2 deg of the record's" holds by
construction, so it is no check here: the tilts are run_leg's inputs, the
replay block's (which the table's start row holds above) and equal to
forward.replay_kwargs' reading of the record, and run_leg pins and then
sets them (forward.py, _pinned and _impose_actuators); the tests hold the
model's start tilts to the record's within 1e-9 deg (a trip through
radians). What run_leg cannot start from is a gz joint caught mid-swing: it
starts the joint settled on the commanded tilt.

The replay. From the Acro switch, the record's replay.run_leg: forward.run_leg's
keyword arguments (FRD entry state, manual_s 0, the recorded throttle exact,
the logged gains as float32 decimals, the pods' tilts and motors with the
modes AirshipPod's rule gives on the logged demand, integ0 0, the IMU filter
settled). From the release, forward.replay_kwargs(leg, model, 'release') (the
state, the pods and motors, the integrator and the flags there) with the
record's pod modes at the release. At sample_hz 50: noise-free, plus --seeds
runs with gz's gyro noise (seeds 1..n). The stick's edges are the record's
own: forward_yaw_check reads them off vehicle_rates_setpoint, which the
controller publishes only on a new stick (airship_att_control_main.cpp,
publishRatesSetpoint under new_sticks), so its first sample past each edge
is the cycle that saw the stick change, its torque setpoint stepping in the
same cycle, although the logged interval is the stick messages' 52 ms.

Measured as the record measures. Each model run is written as a log
(forward_yaw_check.Log) on the record's own sample times: a plant quantity at
a sample time is the model's state after the last step that ended at or
before it; a controller or allocator output is the one of the last cycle at
or before it (forward.py's row convention: a trace row at t holds the cycle
at t - DT). Before its start the model holds its entry state, as its warm-up
did. forward_yaw_check's own leg_table() and leg_metrics() then measure that
log, so the model's numbers have the record's rows, windows, zero-order
holds, mark times, fin formula and Schmitt counts by construction. The
groundtruth heading is the record's at the start plus the model's change,
its velocity the body (u, v, 0) turned by the roll and the heading; the
gyro reads the model's measured yaw rate, its true roll rate (the model
filters no roll gyro) and r tan(phi) on its pitch axis; the pod motor
commands follow from the trace by AirshipPod::drive; control_allocator_status
carries the model allocator's last allocation, its yaw sign and its
torque_setpoint_achieved over the three torque axes (the vectored
allocator's signs, or the Cloudship's pods-and-tail AirshipAllocator, whose
yaw sign is the one the loop hands its controller from the tail chain: the
tests hold the two equal on every row), so the flags at the stick end are
derived from it by the controller's rule, as the record's are. The hold
torque per rad/s is the mean torque over the mean rate of the record's
table rows and of the model's rows at the same times.

Compared, model against record (tolerances of the spec; a percentage is of
the record's value; [noise]: read through the gyro, see below):
  centred  r rms of the 0.5 s moving means, on the rows whose window lies
           inside the segment (the raw rms and the record's own noise
           beside it)                                           <= 0.05 deg/s
  stick    r mean over the last 5 s [noise]                max(0.1 deg/s, 3 %; 5 % Cloudship)
           r peak, 0.5 s moving mean (time window) [noise]       max(0.2 deg/s, 5 %)
           integrator at the stick end                           0.005
           saturation flags, unallocated yaw sign at its end     equal
           heading change                                        max(2 deg, 5 %)
           hold torque per rad/s over its last 2 s [noise]       1 %
  release  heading change                                        max(2 deg, 5 %)
           r, 1 s mean ending at each mark (+10, +60, the end) [noise]   max(0.05 deg/s, 5 %)
           integrator at each mark                               0.003
  speed    u, 0.5 s means at the stick start, its end, the release end   max(0.02 m/s, 1 %)
  reversals per phase: flips, per pod into the rear and back forward     equal (+-1 when
           a flip lies within 0.1 s of the release); per pod time in the rear  0.2 s per transition;
           Cloudship tail flips, on the command and on the torque setpoint     equal (same +-1)
  fins     per fin by name (dorsal/ventral) max |alpha| / alpha_stall where the
           plugin has flow [noise]                               max(0.05, 5 %)
           stalled (that ratio past 1) [noise]   equal, indeterminate when either ratio is 0.95..1.05
                                                 or the model's ratio band holds 1
Every number is judged on the noise-free run. A [noise] number is read
through the gyro (the rate itself, the rate in the hold torque's
denominator, the fins' cp velocity), where the record carries SITL's gyro
noise and the noise-free model none: with seeds it passes when the record
lies within its tolerance of the noise-free value +- 2 s, s the seeds' rms
about it, and the noise-free verdict is printed beside where the two
differ. The band does not widen with more seeds: s converges. A verdict
(the stall, the flags) is the noise-free run's alone; where the seeds'
band of the ratio holds 1, the stall is indeterminate. Every other number
is the controller's or the groundtruth's, its seeds shown only.

Output: per leg the entry-state differences first (the replay's inputs, then
the stick onset and the release, each the record's row against the model's),
then each number, magnitude beside direction (FRD: r and heading positive
nose right, v to starboard, p and roll right wing down, torque and
integrator nose right), then the leg's verdict line; and a CSV of every
precondition, the kept summary's differences, the entry differences and the
numbers, model against record (replay_out/replay_<model>_<stamp>[_leg<n>]_
<start>.csv by default). Exit 0 when every requested leg was replayed and
every compared number is within tolerance, 1 when any is out of tolerance,
2 when none is but a leg was refused (or the summary cannot be read).
"""
import argparse
import contextlib
import csv
import io
import json
import math
import os
import shutil
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import closed_loop as cl  # noqa: E402
import controller as ctl  # noqa: E402
import forward as F  # noqa: E402
# the records replayed here were flown before 2026-10-12, on the linear feedforward law
ctl.FF_LAW = 'linear'
import record_gains as rg  # noqa: E402

fyc = F.fyc                     # forward_yaw_check: the record's own code, imported read only
DT = F.DT
OUT = os.path.join(HERE, 'replay_out')
SAMPLE_HZ = 50.0                # the spec's series rate
HOLD_S = fyc.STEADY_WINDOW_S + 1.0      # the model's held entry state before its start, in its log
AFTER_S = 1.0                   # and its end state held after its end
PITCH_MAX_DEG = 3.0
SETPOINT_TOL_DEG = 1e-3
TABLE_REL = 1e-9                # the table's start row against the summary's start state: one table, both

# The spec's tolerances
CENTRED_RMS = 0.05                          # deg/s, at most
CENTRED_WINDOW_S = 0.5                      # the moving means it compares (forward_yaw_check's PEAK_WINDOW_S)
R_LAST5 = (0.1, 0.03, 0.05)                 # deg/s floor; fraction vectored, Cloudship
R_PEAK = (0.2, 0.05)
INTEG_END = 0.005
HEADING = (2.0, 0.05)
R_MARK = (0.05, 0.05)
INTEG_MARK = 0.003                          # allows for the 200 ms sample age
U_MEAN = (0.02, 0.01)
REAR_S = 0.2                                # s per transition
FLIP_NEAR_S = 0.1                           # a flip this close to the release may count in either phase
# The spec's 5 %, of the record's ratio, as every other tolerance of the
# spec is a percentage of the record's value; the floor is this script's
# (the spec gives none): at 2522's half stick a ratio of 0.13 of stall is
# 1.2 deg of alpha, where 5 % would be 0.06 deg
FIN_RATIO = (0.05, 0.05)
FIN_BROADSIDE = 0.95                        # a ratio past this fraction of 90 deg / alpha_stall: u_cp near 0
STALL_BAND = (0.95, 1.05)
HOLD_K = 0.01                               # step0a.probes()' acceptance test, relative
HOLD_K_WINDOW_S = 2.0
NOISE_K = 2.0                               # a [noise] number's band: the noise-free value +- NOISE_K s

# forward_yaw_check's validity verdicts a leg must have passed, by the text
# their names carry (forward_yaw_check.analyze; the test runs analyze and
# finds each of them, so a reworded verdict breaks the test, not the replay)
LEG_VERDICTS = (
    ('timeline', '로그의 모드와 스틱 시각이 스크립트가 잰 PX4 시각과 맞는다'),
    ('stick as planned', '로그의 스틱이 계획의 스틱이다'),
    ('throttle constant', '스로틀이 진입 전 창부터 놓음 끝까지'),
    ('integrator cleared', 'Manual 에서 Acro 로 들어가 적분기가 비어서 시작했다'),
    ('steady entry', '정상 직진이다'),
    ('durations', '시뮬레이터 시계로 끝까지 갔다'),
    ('airborne and armed', '다리 내내 무장 상태로 떠 있었다'),
)
RUN_VERDICTS = (
    ('topic rates, no logger dropout', '필요한 토픽이 필요한 속도로 있고 로그가 끊기지 않았다'),
    ('the gyro cutoff is the model\'s', '로그의 IMU_GYRO_CUTOFF 가 모델이 쓰는 yaml 기본값이다'),
)
LAYOUT = ('CA_AIRSHIP_GRP', 'CA_AIRSHIP_TAIL', 'CA_AIRSHIP_TLMIN', 'CA_AIRSHIP_TLMAX', 'CA_AIRSHIP_TLT_R',
          'CA_AIRSHIP_CS_K', 'CA_SV_CS_COUNT', 'CA_R_REV')
WATCHED = rg.GAINS + LAYOUT + ('THR_MDL_FAC', 'IMU_GYRO_CUTOFF', 'SYS_AUTOSTART')
STATE_KEYS = ('u', 'v', 'r_deg_s', 'p_deg_s', 'roll_deg')
# the kept summary's fields a re-measurement does not write (paths and the
# live block), left out of its comparison
NOT_MEASURED = ('csv', 'log', 'live', 'stamp')

# Directions (FRD) of each kind of signed number
YAW = ('nose right', 'nose left')
SWAY = ('to starboard', 'to port')
SURGE = ('forward', 'backward')
ROLL = ('right wing down', 'left wing down')


# ---------------------------------------------------------------------------
# Reading the record: today's forward_yaw_check on the run's own files
# ---------------------------------------------------------------------------

def finite(x):
    return isinstance(x, (int, float, np.floating, np.integer)) and not isinstance(x, (bool, np.bool_)) \
        and math.isfinite(x)


def read_ulog(path):
    """A ULog as forward_yaw_check reads it (its TOPICS, instance 0)."""
    return fyc.Log.read(path)


def candidates(name, summary_path):
    """Where a file a summary names may be: as given when absolute, else
    under forward_yaw_check's directory (the base of its relative paths);
    and beside the summary."""
    base = os.path.dirname(os.path.abspath(summary_path))

    if not name:
        return []

    if os.path.isabs(name):
        return [name, os.path.join(base, os.path.basename(name))]

    return [os.path.join(fyc.HERE, name), os.path.join(base, name), os.path.join(base, os.path.basename(name))]


def find_log(kept, summary_path, logs=None):
    """The ULog the summary was made from: its 'log' path, else the copy
    forward_yaw_check keeps beside the summary. A path in `logs` (logs
    already read) counts as present."""
    base = os.path.dirname(os.path.abspath(summary_path))
    paths = candidates(kept.get('log'), summary_path)
    paths.append(os.path.join(base, 'forward_%s_%s.ulg' % (kept.get('model'), kept.get('stamp'))))
    return next((p for p in paths if (logs is not None and p in logs) or os.path.exists(p)), None)


def find_events(kept, summary_path):
    """(path or None, named): the run's events file, the one the summary's
    live block names, else forward_yaw_check's name for it beside the
    summary; named True when the summary names one (a live run's)."""
    named = (kept.get('live') or {}).get('events')
    base = os.path.dirname(os.path.abspath(summary_path))
    paths = candidates(named, summary_path)
    paths.append(os.path.join(base, 'forward_%s_%s_events.json' % (kept.get('model'), kept.get('stamp'))))
    return next((p for p in paths if os.path.exists(p)), None), bool(named)


def remeasure(log, model, events, stamp, release_s):
    """(summary dict, verdicts): today's forward_yaw_check.analyze of a log
    with its run's events, as --analyze <ulg> --events <file> does, into a
    scratch directory that takes the legs' CSVs and is removed after. Its
    stick length is analyze's default, the one every run of it flies."""
    scratch = tempfile.mkdtemp(prefix='forward_replay_')

    try:
        with contextlib.redirect_stdout(io.StringIO()):
            return fyc.analyze(log, model, live=events, out_dir=scratch, stamp=stamp, release_s=release_s)
    finally:
        shutil.rmtree(scratch, ignore_errors=True)


def leg_times(leg):
    """forward_yaw_check's leg dict (find_legs' form) of a record's leg."""
    out = dict(leg['times_px4_us'])
    out.update(off_seen=leg['stick_seen_off'], extra_sticks=leg['extra_sticks'])
    return out


class Record:
    """One leg: the summary as kept, and today's measurement of its log."""

    def __init__(self, summary_path, kept, number, log=None, log_path=None, events=None, events_path=None,
                 events_named=False, doc=None, rows=None, errors=None):
        self.summary_path = summary_path
        self.kept = kept
        self.kept_leg = next((l for l in kept.get('legs') or [] if l.get('leg') == number), None)
        self.number = number
        self.model = kept.get('model')
        self.target = 'gz_%s' % self.model
        self.log = log
        self.log_path = log_path
        self.events = events
        self.events_path = events_path
        self.events_named = events_named
        self.doc = doc                      # today's analysis, None when it could not run
        self.leg = next((l for l in (doc or {}).get('legs') or [] if l.get('leg') == number), None)
        self.rows = rows                    # today's leg_table, t_px4_us int64
        self.errors = dict(errors or {})
        self._times = None

    @property
    def vectored(self):
        return cl.ALL_TARGETS[self.target][1] == 'vectored'

    @property
    def label(self):
        return 'leg %s of %s' % (self.number, os.path.basename(self.summary_path))

    @property
    def times(self):
        """The log's sample times, {topic: int64 us}."""
        if self._times is None and self.log is not None:
            self._times = {topic: self.log.t(topic) for topic in fyc.TOPICS if self.log.has(topic)}

        return self._times


def load(summary_path, leg_numbers=None, logs=None):
    """[Record] for the requested legs of one summary (all when None), each
    measured from the run's ULog and events file by today's
    forward_yaw_check. logs: {path: forward_yaw_check.Log} already read (the
    tests' synthetic logs). Raises LookupError for a leg number the summary
    does not have; a log or events file that cannot be read, or an analysis
    that fails, is kept as the leg's refusal reason."""
    with open(summary_path, encoding='utf-8') as handle:
        kept = json.load(handle)

    numbers = [leg.get('leg') for leg in kept.get('legs') or []]

    if leg_numbers:
        missing = [n for n in leg_numbers if n not in numbers]

        if missing:
            raise LookupError('%s has no leg %s (legs %s)' % (summary_path, missing, numbers))

        numbers = [n for n in numbers if n in leg_numbers]

    errors = {}
    log = events = doc = None
    log_path = find_log(kept, summary_path, logs)
    events_path, named = find_events(kept, summary_path)

    if log_path is None:
        errors['log'] = 'no ULog: the summary names %s, and none is beside it' % kept.get('log')
    else:
        try:
            log = logs[log_path] if logs is not None and log_path in logs else read_ulog(log_path)
        except Exception as why:                # an unreadable log is a refusal reason, not a crash
            errors['log'] = '%s: %s' % (type(why).__name__, why)

    if events_path is not None:
        try:
            with open(events_path, encoding='utf-8') as handle:
                events = json.load(handle)
        except (OSError, ValueError) as why:
            errors['events'] = '%s: %s' % (type(why).__name__, why)
    elif named:
        errors['events'] = 'the summary names %s; it is not there' % kept['live']['events']
    else:
        errors['events'] = 'the summary names none and none is beside it (a log this script did not fly)'

    release_s = kept.get('release_s')
    # a log this script did not fly (no events file named or beside it) is
    # measured as its summary was made, without events, so that its verdicts
    # show; the events check refuses it all the same
    unflown = not named and events_path is None

    if log is None:
        errors['analysis'] = 'not run: no log'
    elif 'events' in errors and not unflown:
        errors['analysis'] = 'not run: the run\'s events file is not read'
    elif not finite(release_s):
        errors['analysis'] = 'not run: the summary has no release_s'
    else:
        try:
            doc, _ = remeasure(log, kept.get('model'), events, kept.get('stamp'), float(release_s))
        except Exception as why:
            errors['analysis'] = '%s: %s' % (type(why).__name__, why)

    out = []

    for number in numbers:
        rec = Record(summary_path, kept, number, log, log_path, events, events_path, named, doc, None, errors)

        if rec.leg is not None:
            try:
                plant = fyc.plant_info(rec.model, doc['params'])
                table = fyc.leg_table(log, leg_times(rec.leg), plant)
                rows = dict(table)
                rows['t_px4_us'] = np.asarray(table['t_px4_us']).astype(np.int64)
                rows['phase'] = [str(x) for x in table['phase']]
                rec.rows = rows
            except Exception as why:
                rec.errors['table'] = '%s: %s' % (type(why).__name__, why)

        out.append(rec)

    return out


# ---------------------------------------------------------------------------
# Preconditions
# ---------------------------------------------------------------------------

def _row_at(rows, t_us):
    """Index of the table row at or before a PX4 time, or None."""
    i = int(np.searchsorted(rows['t_px4_us'], t_us, side='right')) - 1
    return i if i >= 0 else None


def start_state(rec, start):
    rp = rec.leg.get('replay') or {}
    return (rp.get('state') if start == 'acro' else rp.get('state_at_release')) or {}


def start_actuators(rec, start):
    rp = rec.leg.get('replay') or {}
    return (rp.get('actuators') if start == 'acro' else rp.get('actuators_at_release')) or {}


def tilt_keys(rec):
    return ('tilt_stbd_deg', 'tilt_port_deg') if rec.vectored else ('tilt_deg',)


def motor_keys(rec):
    return ('motor1', 'motor2') if rec.vectored else ('motor1', 'motor2', 'motor3')


def preconditions(rec, start):
    """[(name, ok, detail)]: every one must hold for the leg to be replayed."""
    out = []

    def check(name, ok, detail=''):
        out.append((name, bool(ok), detail))
        return bool(ok)

    known = rec.model in fyc.ALLOWED and rec.target in cl.ALL_TARGETS
    check('the plant is one of the four fin plants and a yaw-model target', known,
          '%s (%s)' % (rec.model, rec.target))

    if not known:
        return out

    # the record: today's forward_yaw_check on the log and events file the
    # summary names
    check('the ULog is read', rec.log is not None, rec.errors.get('log') or rec.log_path)
    check('the run\'s events file is read (its timeline and plan)', 'events' not in rec.errors,
          rec.errors.get('events') or rec.events_path)
    measured = check('today\'s forward_yaw_check measured the log, and has the leg in it',
                     rec.doc is not None and rec.leg is not None and rec.rows is not None,
                     rec.errors.get('analysis') or rec.errors.get('table')
                     or ('legs %s' % [l.get('leg') for l in rec.doc['legs']] if rec.doc is not None and rec.leg is None
                         else 'forward_yaw_check.analyze and leg_table'))

    if not measured:
        return out

    doc, leg = rec.doc, rec.leg
    identity(rec, check)

    # the record's own verdicts
    number = rec.number
    verdicts = [(v.get('name', ''), v.get('ok')) for v in doc.get('validity') or []]
    said = lambda found: 'absent from the record' if not found else ('passed' if found == [True] else 'failed')

    for label, text in LEG_VERDICTS:
        found = [ok for name, ok in verdicts if name.startswith('다리 %s: ' % number) and text in name]
        check('verdict passed: %s' % label, found == [True], said(found))

    for label, text in RUN_VERDICTS:
        found = [ok for name, ok in verdicts if name == text]
        check('verdict passed: %s' % label, found == [True], said(found))

    entry, d = leg.get('entry') or {}, leg.get('durations_s') or {}
    first = entry.get('integrator_first')
    check('the integrator starts cleared (first sample after the Acro switch)',
          finite(first) and abs(first) <= fyc.INTEG_CLEAR,
          '%s at %s s (allowed %g)' % (first, entry.get('integrator_first_after_s'), fyc.INTEG_CLEAR))
    release_s = doc.get('release_s')
    check('the stick and the release ran their full lengths',
          finite(d.get('stick')) and d['stick'] >= fyc.STICK_S - fyc.DURATION_TOL_S and leg.get('stick_seen_off') is True
          and finite(release_s) and finite(d.get('acro_after_stick_off'))
          and d['acro_after_stick_off'] >= release_s - fyc.DURATION_TOL_S,
          'stick %s s, release %s s of %s' % (d.get('stick'), d.get('acro_after_stick_off'), release_s))

    # the airframe and its parameters (the log's, as today's analysis read
    # them) against the model's files
    params = doc.get('params') or {}
    want = ctl.airframe_for_model(rec.model)
    autostart = params.get('SYS_AUTOSTART')
    check('SYS_AUTOSTART is the model\'s airframe', finite(autostart) and int(autostart) == want,
          'log %s, model %s %d' % (autostart, rec.model, want))
    changes = [(int(t), name, float(value)) for t, name, value in rec.log.changed]
    moved = sorted({name for _, name, _ in changes if name in WATCHED})
    check('no gain, allocator parameter, THR_MDL_FAC or gyro cutoff changed during the log', not moved,
          'changed: %s' % ', '.join(moved) if moved else 'none of %d changes' % len(changes))
    key = cl.ALL_TARGETS[rec.target][1]
    files = ctl.check_allocator_config(key, want)
    table = ctl.AIRFRAMES[key]
    model_values = {name: float(table[name]) for name in LAYOUT if name in table}
    model_values['CA_SV_CS_COUNT'] = 1.0 if table['yaw_surfaces'] else 0.0
    model_values['THR_MDL_FAC'] = float(ctl.head_params(rec.model)['THR_MDL_FAC'])
    bad = ['%s log %s, model %s' % (name, params.get(name), value) for name, value in model_values.items()
           if not finite(params.get(name)) or float(params[name]) != value]

    if rec.vectored and not bad:
        import vectored_actuation as va
        p = va.load_params(rec.model)

        for name, mine in (('CA_AIRSHIP_TLMIN', p['tilt_min']), ('CA_AIRSHIP_TLMAX', p['tilt_max']),
                           ('CA_AIRSHIP_TLT_R', p['slew'])):
            if va.radians_f(params[name]) != mine:
                bad.append('%s log %s, load_params %s rad' % (name, params[name], mine))

        if params['THR_MDL_FAC'] != p['thr_mdl_fac']:
            bad.append('THR_MDL_FAC log %s, load_params %s' % (params['THR_MDL_FAC'], p['thr_mdl_fac']))

    check('the allocator parameters and THR_MDL_FAC are the model\'s airframe files\'', not bad and not files,
          '; '.join(bad + files) or ', '.join('%s %g' % kv for kv in model_values.items()))
    mine = ctl.DEFAULT_PARAMS['IMU_GYRO_CUTOFF']
    cutoff = params.get('IMU_GYRO_CUTOFF')
    check('IMU_GYRO_CUTOFF is the model gyro\'s', finite(cutoff) and float(cutoff) == mine,
          'log %s; model %g Hz' % (cutoff, mine))

    # the start state, the replay block and the stick
    rp = leg.get('replay') or {}
    state, acts = start_state(rec, start), start_actuators(rec, start)
    pitch = state.get('pitch_deg')
    check('|pitch| under %g deg at the start' % PITCH_MAX_DEG, finite(pitch) and abs(pitch) < PITCH_MAX_DEG,
          '%s deg' % pitch)
    stick = rp.get('stick') or []
    phase = leg.get('stick_phase') or {}
    values = [('state.%s' % k, state.get(k)) for k in STATE_KEYS + ('pitch_deg', 'heading_deg', 'torque_z', 'alt_m')]
    values += [('actuators.%s' % k, acts.get(k)) for k in tilt_keys(rec) + motor_keys(rec)]
    values += [('throttle', rp.get('throttle')), ('end_s', rp.get('end_s')),
               ('throttle.min', (leg.get('throttle') or {}).get('min')),
               ('throttle.max', (leg.get('throttle') or {}).get('max')),
               ('stick.setpoint_deg_s', (leg.get('stick') or {}).get('setpoint_deg_s'))]
    values += [('params.%s' % k, (rp.get('params') or {}).get(k)) for k in rg.GAINS]
    # the allocator status the stick-end flags are derived from: a NaN there
    # reads as no flag (a NaN fails every comparison), not as unread
    values += [('stick_phase.unallocated_end', phase.get('unallocated_end')),
               ('stick_phase.achieved_end', phase.get('achieved_end'))]
    shape = len(stick) == 3 and all(isinstance(x, (list, tuple)) and len(x) == 2 for x in stick)
    values += [('stick[%d][%d]' % (i, j), stick[i][j]) for i in range(3) for j in range(2)] if shape else \
        [('stick timeline', None)]

    if start == 'release':
        values += [('integrator_at_release', rp.get('integrator_at_release')),
                   ('integrator_age_s', ((leg.get('release') or {}).get('entry') or {}).get('integrator_age_s'))]

    flags = [phase.get('flag_pos_end'), phase.get('flag_neg_end')]
    check('the saturation flags at the stick end are read', all(isinstance(x, bool) for x in flags),
          'flag + %s, flag - %s' % tuple(flags))
    unread = [name for name, x in values if not finite(x)]
    check('every number the replay reads is finite', not unread, 'unread: %s' % ', '.join(unread) if unread else
          '%d numbers' % len(values))

    if unread:
        return out

    gains = {k: rg.f32_decimal(rp['params'][k]) for k in rg.GAINS}
    sp = math.degrees(ctl.yaw_rate_setpoint(stick[1][1], gains['MAN_DEADZONE'], math.radians(gains['AS_YAWRATE_MAX'])))
    logged_sp = leg['stick']['setpoint_deg_s']
    check('the stick\'s setpoint from the logged gains is the logged one', abs(sp - logged_sp) <= SETPOINT_TOL_DEG,
          '%.6f against %.6f deg/s' % (sp, logged_sp))
    th, thr = leg['throttle'], rp['throttle']
    counts = int(round(thr * 1000.0))
    plan = (leg.get('plan') or {}).get('throttle_counts')
    theirs = rp.get('throttle_counts')
    ok = abs(th['min'] - thr) <= fyc.THROTTLE_TOL and abs(th['max'] - thr) <= fyc.THROTTLE_TOL
    ok = ok and isinstance(theirs, int) and theirs == counts and (plan is None or int(plan) == counts)
    check('the thrust setpoint stayed on the replayed throttle, the record\'s count and the plan\'s', ok,
          'min %.6f max %.6f against %.6f (%d/1000); the record\'s count %s, the plan\'s %s'
          % (th['min'], th['max'], thr, counts, rp.get('throttle_counts', '(absent)'), plan))
    record_format(rec, check)
    consistency(rec, start, check)

    if start == 'release':
        age = leg['release']['entry']['integrator_age_s']
        step = (doc.get('resolution_ms') or {}).get('integrator')
        check('the integrator sample at the release is at most one logged interval old',
              finite(step) and 0.0 <= age <= step / 1000.0 + DT + 1e-9, '%.3f s, interval %s ms' % (age, step))

    coverage(rec, check)
    return out


def identity(rec, check):
    """The kept summary is today's measurement's leg: the same plant (and
    the events file's), and the leg's PX4 times, which the log's mode
    changes and stick edges fix. Every other value it holds is reported
    where it differs (kept_differences), not checked: the replay uses
    today's."""
    kept, today = rec.kept_leg, rec.leg
    plants = [rec.kept.get('model'), rec.doc.get('model')] + ([rec.events.get('model')] if rec.events else [])
    same_plant = len(set(plants)) == 1
    same_times = kept is not None and kept.get('times_px4_us') == today.get('times_px4_us')
    check('the kept summary is this log\'s leg: its plant, and the leg\'s PX4 times today\'s',
          same_plant and same_times,
          'plant %s; times kept %s, today %s' % (' / '.join(str(p) for p in plants),
                                                (kept or {}).get('times_px4_us'), today.get('times_px4_us'))
          if not (same_plant and same_times) else '%s, %d times' % (rec.model, len(today.get('times_px4_us') or {})))


def record_format(rec, check):
    """The fields of forward_yaw_check's model interface (2026-10-01) the
    replay reads: today's forward_yaw_check must still write them."""
    leg, rows = rec.leg, rec.rows
    rp = leg.get('replay') or {}
    absent = ['replay.%s' % k for k in ('run_leg', 'run_leg_unread', 'pod_modes', 'pod_modes_at_release',
                                         'throttle_counts', 'integrator_first_after_s') if k not in rp]
    absent += [] if 'r_peak_smoothed_05s_deg_s' in (leg.get('stick_phase') or {}) else \
        ['stick_phase.r_peak_smoothed_05s_deg_s']
    blocks = [leg.get(b) or {} for b in ('stick_phase', 'release')]
    fins = [f for block in blocks for f in block.get('fins') or []]
    absent += [] if fins and all('max_ratio' in f and 'name' in f and 'no_flow_rows' in f for f in fins) else \
        ['fins[].name, max_ratio, no_flow_rows']

    if not rec.vectored:
        absent += [] if all('tail_flips_torque' in (b.get('reversals') or {}) for b in blocks) else \
            ['reversals.tail_flips_torque']

    absent += [] if 't_acro_s' in rows else ['the table\'s t_acro_s']
    check('the record has forward_yaw_check\'s model interface of 2026-10-01', not absent,
          'absent: %s' % ', '.join(absent) if absent
          else 'run_leg, pod modes, the time-window peak, fins by name with flow, t_acro_s')


def consistency(rec, start, check):
    """Today's measurement against itself: the summary's replay block
    against its PX4 times and states, and its table."""
    leg = rec.leg
    rp, tp = leg['replay'], leg.get('times_px4_us') or {}
    need = ('manual_us', 'acro_us', 'on_us', 'off_us', 'acro_end_us', 'release_end_us')

    if not all(isinstance(tp.get(k), int) for k in need):
        check('the record\'s PX4 times are read', False, str(tp))
        return

    acro, on, off, end = tp['acro_us'], tp['on_us'], tp['off_us'], tp['release_end_us']
    stick = rp['stick']
    sec = lambda us: us / 1e6
    times_ok = (rp.get('t0_px4_us') == acro and stick[0] == [0.0, 0.0] and stick[2][1] == 0.0
                and abs(stick[1][0] - sec(on - acro)) <= 1e-6 and abs(stick[2][0] - sec(off - acro)) <= 1e-6
                and abs(rp['end_s'] - sec(end - acro)) <= 1e-6 and stick[1][1] == leg['stick']['value']
                and acro < on < off <= end)
    check('the replay block\'s timeline is the record\'s PX4 times',
          times_ok, 't0 %s / acro %s; stick on %s / %.6f, off %s / %.6f, end %s / %.6f s; stick %s / %s'
          % (rp.get('t0_px4_us'), acro, stick[1][0], sec(on - acro), stick[2][0], sec(off - acro), rp['end_s'],
             sec(end - acro), stick[1][1], leg['stick']['value']))
    release = leg.get('release') or {}
    same = (rp.get('state') == (leg.get('entry') or {}).get('at_acro')
            and rp.get('state_at_release') == (release.get('entry') or {}).get('state')
            and rp.get('actuators_at_release') == (release.get('entry') or {}).get('actuators')
            and rp.get('integrator_at_release') == (release.get('entry') or {}).get('integrator')
            and rp.get('integrator') == (leg.get('entry') or {}).get('integrator_first'))
    check('the replay block\'s states are the record\'s entry and release states', same, '')
    rows = rec.rows
    needed = ('t_s', 't_px4_us', 'phase', 'u', 'v', 'r_deg_s', 'p_deg_s', 'heading_deg', 'roll_deg', 'pitch_deg',
              'throttle', 'torque_z') + tilt_keys(rec) + motor_keys(rec)
    absent = [c for c in needed if c not in rows]

    if absent or not len(rows['t_px4_us']):
        check('the leg\'s table is read', False, 'absent columns %s' % absent if absent else 'no rows')
        return

    t = rows['t_px4_us']
    close = lambda column, origin: np.abs(rows[column] - (t - origin) / 1e6) \
        <= 5e-6 * np.maximum(1.0, np.abs(rows[column])) + 1e-9
    clocks = bool(np.all(close('t_s', on))) and 't_acro_s' in rows and bool(np.all(close('t_acro_s', acro)))
    phase = np.where(t < acro, 'steady', np.where(t < on, 'entry', np.where(t < off, 'stick', 'release')))
    phases = list(phase) == list(rows['phase'])
    ordered = bool(np.all(np.diff(t) > 0))
    worst = float(np.max(np.abs(rows['t_s'] - (t - on) / 1e6)))
    check('the table\'s rows: increasing times, t_s from the stick onset (t_acro_s from the switch), the phases',
          ordered and clocks and phases, 'worst t_s offset %.6f s, phases %s, increasing %s' % (worst, phases, ordered))
    t0 = acro if start == 'acro' else off
    i = _row_at(rows, t0)
    state, acts = start_state(rec, start), start_actuators(rec, start)

    if i is None:
        check('the table\'s row at the start is the summary\'s start state', False, 'no row at or before %d' % t0)
        return

    pairs = [(k, rows[k][i], state.get(k)) for k in STATE_KEYS + ('pitch_deg', 'heading_deg', 'torque_z')]
    pairs += [(k, rows[k][i], acts.get(k)) for k in tilt_keys(rec) + motor_keys(rec)]
    pairs.append(('throttle', rows['throttle'][i], state.get('throttle')))
    near = lambda a, b: finite(a) and finite(b) and abs(a - b) <= TABLE_REL * max(abs(a), abs(b))
    off_by = ['%s table %r summary %r' % (k, a, b) for k, a, b in pairs if not near(float(a), b)]
    check('the table\'s row at the start is the summary\'s start state', not off_by,
          '; '.join(off_by) if off_by else 'row at %+.3f s, %d values' % ((t[i] - t0) / 1e6, len(pairs)))


def coverage(rec, check):
    """Every topic forward_yaw_check reads has samples inside the leg."""
    tp = rec.leg['times_px4_us']
    a, b = tp['acro_us'], tp['release_end_us']
    empty = np.zeros(0, dtype=np.int64)
    thin = [topic for topic in fyc.TOPICS
            if not np.any((rec.times.get(topic, empty) >= a) & (rec.times.get(topic, empty) <= b))]
    check('every topic has samples inside the leg', not thin, 'none: %s' % ', '.join(thin) if thin else
          '%d topics' % len(fyc.TOPICS))


def _plain(x):
    """A value as the summary's JSON holds it (numpy numbers, tuples)."""
    return json.loads(json.dumps(x, default=float))


def _differ(a, b, path, out):
    if isinstance(a, dict) and isinstance(b, dict):
        for k in sorted(set(a) | set(b), key=str):
            if k not in NOT_MEASURED:
                _differ(a.get(k, '(absent)'), b.get(k, '(absent)'), '%s.%s' % (path, k) if path else str(k), out)
    elif isinstance(a, list) and isinstance(b, list) and len(a) == len(b):
        for i, (x, y) in enumerate(zip(a, b)):
            _differ(x, y, '%s[%d]' % (path, i), out)
    elif not (a == b or (isinstance(a, float) and isinstance(b, float) and math.isnan(a) and math.isnan(b))):
        out.append((path, a, b))


def kept_differences(rec):
    """[(path, kept, today's)]: the kept summary's run fields and this leg
    against today's measurement of the log (its paths and live block aside,
    NOT_MEASURED). Not used by the replay, which takes today's."""
    out = []
    _differ(_plain({k: v for k, v in rec.kept.items() if k != 'legs'}),
            _plain({k: v for k, v in rec.doc.items() if k != 'legs'}), '', out)
    _differ(_plain(rec.kept_leg or {}), _plain(rec.leg), 'legs[leg %s]' % rec.number, out)
    return out


# ---------------------------------------------------------------------------
# The replay's inputs
# ---------------------------------------------------------------------------

def replay_inputs(rec, start):
    """(run_leg kwargs, notes) of one start; raises LookupError (a value the
    record leaves unread) or ValueError (two readings of the record that
    disagree). From the Acro switch: the record's replay.run_leg, held
    against forward.replay_kwargs, which reads the same fields itself: every
    key equal but the pods' modes (the record's, by AirshipPod's rule on the
    logged demand; forward.infer_pod_mode assumes one history) and integ0 (the
    record's 0, the reset it showed). From the release: forward.replay_kwargs
    with the record's pod modes at the release."""
    leg, rp = rec.leg, rec.leg['replay']
    mine = F.replay_kwargs(leg, rec.model, start)
    inferred = [p['mode'] for p in mine['pods0']]
    notes = []

    if start == 'acro':
        theirs = rp['run_leg']

        if theirs is None:
            raise LookupError('the record\'s run_leg is None: %s unread' % ', '.join(rp.get('run_leg_unread') or ['?']))

        differ = []

        for key, value in mine.items():
            if key == 'pods0':
                if [p['tilt'] for p in theirs.get('pods0') or []] != [p['tilt'] for p in value]:
                    differ.append('pods0 tilts %s / %s' % (theirs.get('pods0'), value))
            elif theirs.get(key) != value:
                differ.append('%s %r / %r' % (key, theirs.get(key), value))

        extra = sorted(set(theirs) - set(mine) - {'integ0'})

        if extra or theirs.get('integ0') != 0.0:
            differ.append('keys only the record has: %s, integ0 %r' % (extra, theirs.get('integ0')))

        if differ:
            raise ValueError('the record\'s run_leg and forward.replay_kwargs disagree: %s' % '; '.join(differ))

        kwargs = json.loads(json.dumps(theirs))         # a copy: run_leg must not share the record's dicts
        modes = [p['mode'] for p in kwargs['pods0']]
        notes.append('the record\'s run_leg; pod modes %s from the logged demand%s' % (
            ', '.join(modes), '' if modes == inferred else ' (forward.infer_pod_mode would say %s)' % ', '.join(inferred)))
        return kwargs, notes

    modes = rp['pod_modes_at_release']

    if modes is None or len(modes) != len(mine['pods0']) or any(m.get('mode') is None for m in modes):
        raise LookupError('the record\'s pod modes at the release are unread: %s' % (modes,))

    mine['pods0'] = [dict(pod, mode=m['mode']) for pod, m in zip(mine['pods0'], modes)]
    used = [m['mode'] for m in modes]
    notes.append('pod modes %s at the release from the logged demand (%s)%s' % (
        ', '.join(used), ', '.join('%.4f' % m['demand'] for m in modes),
        '' if used == inferred else '; forward.infer_pod_mode would say %s' % ', '.join(inferred)))
    return mine, notes


# ---------------------------------------------------------------------------
# The model, written as a log on the record's sample times
# ---------------------------------------------------------------------------

def allocator_status(allocator):
    """(unallocated yaw, torque_setpoint_achieved) of the allocation the
    allocator made last, as control_allocator_status publishes it (CA:661-
    691 with the airship's getUnallocatedControl: each torque axis a
    saturation sign): vectored_actuation.Allocator's signs, or
    controller.AirshipAllocator.status() (the Cloudship's pods and tail);
    achieved when all three axes' signs are 0."""
    if hasattr(allocator, 'status'):
        status = allocator.status()
        return float(status['unallocated_torque'][2]), bool(status['torque_setpoint_achieved'])

    signs = [float(x) for x in allocator.unallocated]
    return signs[2], sum(x * x for x in signs) < 1e-6


def run_model(kwargs, noise=False, seed=1):
    """forward.run_leg with a read-only hook that keeps, once per cycle and
    before it runs, what the trace does not: the roll rate p and the roll
    phi (FRD, deg/s, deg) of the plant at t, and the allocator's status
    after the cycle at t - DT (the trace's row convention); and the
    allocator's status after the last cycle, which no hook call sees."""
    cycles = {'t': [], 'p': [], 'phi': [], 'unalloc': [], 'achieved': []}
    seen = []

    def hook(t, plant, controller, allocator):
        cycles['t'].append(t)
        cycles['p'].append(math.degrees(plant.raw.nu[2]))      # roll is the same axis in FLU and FRD
        cycles['phi'].append(math.degrees(plant.raw.phi))
        unalloc, achieved = allocator_status(allocator)
        cycles['unalloc'].append(unalloc)
        cycles['achieved'].append(achieved)

        if not seen:
            seen.append(allocator)

    leg = F.run_leg(noise=noise, seed=seed, sample_hz=SAMPLE_HZ, hook=hook, **kwargs)
    cycles['last'] = allocator_status(seen[0]) if seen else (float('nan'), False)
    leg['cycles'] = cycles
    return leg


def model_rows(leg, start):
    """The model's leg as rows on its own clock (s since its start): row 0
    its start (entry['acro'], the plant at t = 0 and the warm-up's last
    cycle), row k the trace row at k DT, with p, phi and the allocator's
    status from the hook (the last row from entry['end'] and the allocator
    after the run). The pod motor commands the trace does not carry follow
    from it by the allocator's drive law (AirshipPod::drive, EA:206-209: a
    pod's command is its demand along its tilt, clamped 0..1; the demand is
    (c -+ torque, 0) on the vectored pods, (c, 0) on the Cloudship's
    collective pair, c the flown thrust). From the release the start's
    torque is NaN: the warm-up's Manual torque is no torque the record had
    there."""
    tr, e, cy, s = leg['trace'], leg['entry'], leg['cycles'], leg['setup']
    n = len(tr['t'])

    if len(cy['t']) != n or any(abs(a - k * DT) > 1e-6 for k, a in enumerate(cy['t'][:3])):
        raise AssertionError('the hook ran %d times for %d steps' % (len(cy['t']), n))

    first, last = e['acro'], e['end']
    acts = first['actuators']
    rows = {'t': np.concatenate([[0.0], np.asarray(tr['t'], dtype=float)])}

    for key in ('u', 'v', 'r', 'r_meas', 'heading', 'integ', 'torque'):
        rows[key] = np.concatenate([[first[key]], np.asarray(tr[key], dtype=float)])

    rows['sp'] = np.concatenate([[np.nan], np.asarray(tr['sp'], dtype=float)])

    if start == 'release':
        rows['torque'][0] = np.nan

    rows['p'] = np.array(list(cy['p']) + [last['p']], dtype=float)
    rows['phi'] = np.array(list(cy['phi']) + [last['phi']], dtype=float)
    rows['unalloc'] = np.array(list(cy['unalloc']) + [cy['last'][0]], dtype=float)
    rows['achieved'] = np.array(list(cy['achieved']) + [cy['last'][1]], dtype=float)
    c = s['flown']

    if s['vectored']:
        for i, key in enumerate(('tilt_s', 'tilt_p')):
            rows[key] = np.concatenate([[acts['pods'][i]['tilt']], np.asarray(tr[key], dtype=float)])

        motors = [np.clip((c + sign * rows['torque']) * np.cos(np.radians(rows[key])), 0.0, 1.0)
                  for sign, key in ((-1.0, 'tilt_s'), (1.0, 'tilt_p'))]
    else:
        rows['tilt'] = np.concatenate([[acts['pods'][0]['tilt']], np.asarray(tr['tilt'], dtype=float)])
        rows['tail'] = np.concatenate([[acts['tail']], np.asarray(tr['tail'], dtype=float)])
        motors = [np.clip(c * np.cos(np.radians(rows['tilt'])), 0.0, 1.0) for _ in range(2)]

    for i in (0, 1):
        motors[i][0] = acts['motors'][i]

    rows['motor1'], rows['motor2'] = motors
    # the body pitch rate a heading change has while rolled (planar.py's
    # w = (p, r tan(phi), r) in FLU; FRD q = r tan(phi) with FRD r): what the
    # gyro's y axis reads, and the record's u at a fin's cp takes
    rows['q'] = rows['r'] * np.tan(np.radians(rows['phi']))
    return rows


def quaternion(roll, yaw):
    """PX4's FRD-to-NED quaternion (w, x, y, z) of a roll and a heading (rad),
    no pitch: q_z(yaw) q_x(roll), which forward_yaw_check.euler inverts."""
    cr, sr, cy, sy = np.cos(roll / 2), np.sin(roll / 2), np.cos(yaw / 2), np.sin(yaw / 2)
    return cy * cr, cy * sr, sy * sr, sy * cr


def model_log(leg, start, times, t0_us, heading0_deg, alt_m, modes, plant, params, window):
    """forward_yaw_check.Log of one model leg on given sample times.

    times: {topic: int64 us}, the record's (or a synthetic record's) own;
    t0_us: the PX4 time of the model's t = 0 (the Acro switch, or the
    release); heading0_deg, alt_m: the record's heading and altitude there;
    modes: [(t_us, nav_state, armed)], the mode timeline; plant:
    forward_yaw_check.plant_info; window: (a_us, b_us), the samples kept.
    The integrator and the rate setpoint are published from t0 on (the loop
    closed), and from the release the torque setpoint and the allocator's
    status too (the model has no torque before its first closed cycle there:
    its warm-up ran Manual); every other topic over the whole window. The
    thrust setpoint's z and the roll torque are the model's: 0 (the pitch
    and roll sticks centred, airship_manual_input's passthrough)."""
    rows = model_rows(leg, start)
    s = leg['setup']
    tm = rows['t']
    last = len(tm) - 1
    lo_us, hi_us = window
    t_stick, t_release = s['times'][2], s['times'][3]
    flown = s['flown']
    data = {}

    def samples(topic, closed=False):
        t = np.asarray(times.get(topic, np.zeros(0)), dtype=np.int64)
        t = np.unique(t[(t >= (t0_us if closed else lo_us)) & (t <= hi_us)])
        return t, (t - t0_us) / 1e6

    def at(tau, key, command):
        i = np.searchsorted(tm, tau + (DT if command else 0.0) + 1e-9, side='right') - 1
        return rows[key][np.clip(i, 0, last)]

    mode_t = np.array([m[0] for m in modes], dtype=np.int64)

    def mode_at(t):
        i = np.clip(np.searchsorted(mode_t, t, side='right') - 1, 0, len(modes) - 1)
        return np.array([modes[k][1] for k in i], dtype=float), np.array([modes[k][2] for k in i], dtype=float)

    t, _ = samples('vehicle_status')
    t = np.union1d(t, mode_t[(mode_t >= lo_us) & (mode_t <= hi_us)])
    nav, armed = mode_at(t)
    data['vehicle_status'] = {'timestamp': t, 'nav_state': nav, 'arming_state': np.where(armed > 0, fyc.ARMED, 1.0)}
    t, _ = samples('vehicle_control_mode')
    t = np.union1d(t, mode_t[(mode_t >= lo_us) & (mode_t <= hi_us)])
    nav, armed = mode_at(t)
    data['vehicle_control_mode'] = {'timestamp': t, 'flag_armed': armed, 'flag_control_manual_enabled': np.ones(len(t)),
                                    'flag_control_rates_enabled': (nav == fyc.NAV_ACRO).astype(float)}
    t, tau = samples('manual_control_setpoint')

    if start == 'acro':
        yaw = np.where((tau >= t_stick - 1e-9) & (tau < t_release - 1e-9), s['stick'], 0.0)
    else:
        yaw = np.where(tau < -1e-9, s['stick'], 0.0)

    data['manual_control_setpoint'] = {'timestamp': t, 'yaw': yaw}
    t, tau = samples('vehicle_rates_setpoint', closed=True)
    data['vehicle_rates_setpoint'] = {'timestamp': t, 'yaw': np.radians(at(tau, 'sp', True))}
    t, tau = samples('vehicle_angular_velocity')
    data['vehicle_angular_velocity'] = {'timestamp': t, 'xyz[0]': np.radians(at(tau, 'p', False)),
                                        'xyz[1]': np.radians(at(tau, 'q', False)),
                                        'xyz[2]': np.radians(at(tau, 'r_meas', False))}
    t, tau = samples(fyc.GT_ATT)
    q = quaternion(np.radians(at(tau, 'phi', False)), np.radians(heading0_deg + at(tau, 'heading', False)))
    data[fyc.GT_ATT] = {'timestamp': t, 'q[0]': q[0], 'q[1]': q[1], 'q[2]': q[2], 'q[3]': q[3]}

    def velocity(topic):
        """The link origin's NED velocity: the body (u, v, 0) turned by the
        roll, then the heading (R = Rz(psi) Rx(phi), no pitch), as the
        groundtruth gives a rolled hull's; the body's starboard axis dips
        with the roll, so v has a downward part v sin(phi)."""
        t, tau = samples(topic)
        u, v = at(tau, 'u', False), at(tau, 'v', False)
        phi = np.radians(at(tau, 'phi', False))
        psi = np.radians(heading0_deg + at(tau, 'heading', False))
        side = v * np.cos(phi)
        return t, u * np.cos(psi) - side * np.sin(psi), u * np.sin(psi) + side * np.cos(psi), v * np.sin(phi), psi

    t, vx, vy, vz, _ = velocity(fyc.GT_POS)
    data[fyc.GT_POS] = {'timestamp': t, 'vx': vx, 'vy': vy, 'vz': vz, 'z': np.full(len(t), -alt_m)}
    t, vx, vy, _, psi = velocity('vehicle_local_position')
    data['vehicle_local_position'] = {'timestamp': t, 'vx': vx, 'vy': vy,
                                      'heading': np.arctan2(np.sin(psi), np.cos(psi))}
    t, tau = samples('vehicle_torque_setpoint', closed=start == 'release')
    data['vehicle_torque_setpoint'] = {'timestamp': t, 'xyz[0]': np.zeros(len(t)), 'xyz[2]': at(tau, 'torque', True)}
    t, _ = samples('vehicle_thrust_setpoint')
    data['vehicle_thrust_setpoint'] = {'timestamp': t, 'xyz[0]': np.full(len(t), flown), 'xyz[2]': np.zeros(len(t))}
    t, tau = samples('rate_ctrl_status', closed=True)
    data['rate_ctrl_status'] = {'timestamp': t, 'yawspeed_integ': at(tau, 'integ', True)}
    t, tau = samples('control_allocator_status', closed=start == 'release')
    data['control_allocator_status'] = {'timestamp': t, 'unallocated_torque[2]': at(tau, 'unalloc', True),
                                        'torque_setpoint_achieved': at(tau, 'achieved', True)}
    t, tau = samples('actuator_servos')
    lo, hi = plant['tilt_range']
    servo = lambda key: 2.0 * (at(tau, key, True) - lo) / (hi - lo) - 1.0
    keys = ('tilt_s', 'tilt_p') if s['vectored'] else ('tilt',)
    data['actuator_servos'] = {'timestamp': t}

    for i in range(plant['servo_offset'] + len(keys)):
        data['actuator_servos']['control[%d]' % i] = np.zeros(len(t))

    for i, key in enumerate(keys):
        data['actuator_servos']['control[%d]' % (plant['servo_offset'] + i)] = servo(key)

    t, tau = samples('actuator_motors')
    data['actuator_motors'] = {'timestamp': t, 'control[0]': at(tau, 'motor1', True),
                               'control[1]': at(tau, 'motor2', True)}

    if not s['vectored']:
        data['actuator_motors']['control[2]'] = at(tau, 'tail', True)

    return fyc.Log(data, dict(params), start_us=lo_us, end_us=hi_us, info={'ver_sw': 'yaw model'})


def measure(rec, model, start):
    """(table, numbers): the model's leg measured by forward_yaw_check's own
    leg_table and leg_metrics, on the record's sample times."""
    doc, leg = rec.doc, rec.leg
    tp = leg['times_px4_us']
    t0 = tp['acro_us'] if start == 'acro' else tp['off_us']
    state = start_state(rec, start)
    plant = fyc.plant_info(rec.model, doc['params'])
    modes = [(tp['manual_us'], fyc.NAV_MANUAL, True), (tp['acro_us'], fyc.NAV_ACRO, True),
             (tp['acro_end_us'], fyc.NAV_MANUAL, True)]
    window = (t0 - int(HOLD_S * 1e6), tp['release_end_us'] + int(AFTER_S * 1e6))
    log = model_log(model, start, rec.times, t0, state['heading_deg'], state['alt_m'], modes, plant, doc['params'],
                    window)
    lt = leg_times(leg)
    table = fyc.leg_table(log, lt, plant)
    return table, fyc.leg_metrics(log, lt, table, plant, doc['params'], release_s=doc['release_s'])


# ---------------------------------------------------------------------------
# The comparison
# ---------------------------------------------------------------------------

def _range(values):
    xs = [x for x in values if finite(x)]
    return (min(xs), max(xs)) if xs else (None, None)


def noise_band(model, seeds):
    """(lo, hi, half) of a [noise] number: the noise-free value +- NOISE_K
    s, s the seeds' rms about it (0 without a read seed); Nones when the
    noise-free value is unread."""
    if not finite(model):
        return None, None, None

    xs = [float(x) for x in seeds if finite(x)]
    half = NOISE_K * math.sqrt(sum((x - model) ** 2 for x in xs) / len(xs)) if xs else 0.0
    return model - half, model + half, half


def _distance(x, lo, hi):
    return 0.0 if lo <= x <= hi else min(abs(x - lo), abs(x - hi))


def _row(section, metric, unit, record, models, kind, tol=None, dirs=None, note='', allow=0, noisy=False):
    """One compared number. models: the noise-free run's value first, then
    the seeds'. kind: 'value' (|record - model| <= tol), 'upper' (the
    model's value <= tol, no record), 'rel' (|record - model| <= tol
    |record|), 'count' (|record - model| <= allow), 'bool' (equal), 'stalled'
    (as bool, indeterminate when the note says so). Judged on the noise-free
    run ('verdict_noise_free'); a [noise] number with seeds against the band
    noise_band() gives ('verdict'). A verdict (bool, stalled) is the
    noise-free run's alone, the seeds shown only: an 'equal to any run's'
    would pass more often the more seeds ran (fin_rows makes a stall the
    seeds' band cannot settle indeterminate instead)."""
    model = models[0] if models else None
    seeds = list(models[1:])
    out = {'section': section, 'metric': metric, 'unit': unit, 'record': record, 'model': model, 'seeds': seeds,
           'kind': kind, 'tol': allow if kind == 'count' else tol, 'dirs': dirs, 'note': note, 'diff': None,
           'noisy': noisy, 'band': None, 'verdict': None, 'verdict_noise_free': None}

    if kind in ('bool', 'stalled'):
        if record is None and model is None:
            verdict = 'n/a'
        elif record is None or model is None:
            verdict = 'FAIL'
        elif kind == 'stalled' and note.startswith('indeterminate'):
            verdict = 'indeterminate'
        else:
            verdict = 'ok' if record == model else 'FAIL'

        out['verdict'] = out['verdict_noise_free'] = verdict
        return out

    if kind == 'upper':
        out['diff'] = model
        out['verdict'] = out['verdict_noise_free'] = 'ok' if finite(model) and model <= tol else 'FAIL'
        return out

    if record is None and model is None:
        out['verdict'] = out['verdict_noise_free'] = 'n/a'
        return out

    if not finite(record) or not finite(model) or (kind != 'count' and not finite(tol)):
        out['verdict'] = out['verdict_noise_free'] = 'FAIL'
        return out

    out['diff'] = model - record
    limit = allow if kind == 'count' else (tol * abs(record) if kind == 'rel' else tol + 1e-12)
    out['verdict_noise_free'] = 'ok' if abs(record - model) <= limit else 'FAIL'
    out['verdict'] = out['verdict_noise_free']

    if noisy and seeds:
        lo, hi, half = noise_band(model, seeds)
        out['band'] = half
        out['verdict'] = 'ok' if _distance(record, lo, hi) <= limit else 'FAIL'

    return out


def pct(floor, fraction):
    return lambda x: max(floor, fraction * abs(x)) if finite(x) else floor


def _rows_between(t, a_us, b_us):
    return (t >= a_us) & (t < b_us)


def _flip_times(t_us, values, kind):
    """PX4 times at which forward_yaw_check's counter changes direction,
    phases aside: pod_reversals' Schmitt on cos(tilt) at +-REVERSAL_COS
    (kind 'pod', values deg) or sign_reversals' band +-TAIL_DEADBAND ('tail')."""
    state, out = None, []

    for ti, x in zip(t_us, values):
        if kind == 'pod':
            c = math.cos(math.radians(x)) if math.isfinite(x) else float('nan')
            side = 'fwd' if c > fyc.REVERSAL_COS else ('rear' if c < -fyc.REVERSAL_COS else None)
        else:
            side = 1 if x > fyc.TAIL_DEADBAND else (-1 if x < -fyc.TAIL_DEADBAND else None)

        if state is not None and side is not None and side != state:
            out.append(int(ti))

        state = side or state

    return out


def _moving_mean(t_us, x, half_us):
    """The mean of x over the rows within +-half_us of each row."""
    lo = np.searchsorted(t_us, t_us - half_us, side='left')
    hi = np.searchsorted(t_us, t_us + half_us, side='right')
    sums = np.concatenate([[0.0], np.cumsum(x)])
    return (sums[hi] - sums[lo]) / (hi - lo)


def centred(rec, table):
    """The centred segment (the Acro switch to the stick onset), the record's
    gyro rows against the model's, matched by PX4 time: {'rms': of the two
    CENTRED_WINDOW_S moving means' difference, on the rows whose window lies
    inside the segment; 'raw': of the rows' own difference; 'noise': the
    record's rows about their moving mean (its gyro noise, the part of
    'raw' the moving mean removes); 'rows', 'smoothed_rows'}, or None when
    the rows do not match or one is unread."""
    tp = rec.leg['times_px4_us']
    a, b = tp['acro_us'], tp['on_us']
    rows = rec.rows
    mine = _rows_between(rows['t_px4_us'], a, b)
    t_model = np.asarray(table['t_px4_us']).astype(np.int64)
    theirs = _rows_between(t_model, a, b)
    t = rows['t_px4_us'][mine]

    if not len(t) or not np.array_equal(t, t_model[theirs]):
        return None

    x, y = rows['r_deg_s'][mine], np.asarray(table['r_deg_s'])[theirs]

    if not (np.all(np.isfinite(x)) and np.all(np.isfinite(y))):
        return None

    half = int(round(CENTRED_WINDOW_S * 1e6 / 2.0))
    inside = (t - half >= a) & (t + half < b)
    mx, my = _moving_mean(t, x, half), _moving_mean(t, y, half)
    rms = lambda d: float(np.sqrt(np.mean(d * d))) if len(d) else None
    return {'rms': rms((mx - my)[inside]), 'raw': rms(x - y), 'noise': rms((x - mx)[inside]), 'rows': int(len(t)),
            'smoothed_rows': int(inside.sum())}


def _hold_k(t_us, torque, r_deg, a_us, b_us):
    """(normalized torque per rad/s, mean r deg/s) over rows a <= t < b:
    step0a.probes()' acceptance test, mean torque over mean r."""
    rows = _rows_between(t_us, a_us, b_us)

    if not rows.any() or not np.all(np.isfinite(torque[rows])) or not np.all(np.isfinite(r_deg[rows])):
        return None, None

    r = float(np.mean(r_deg[rows]))
    return (float(np.mean(torque[rows])) / math.radians(r) if r != 0.0 else None), r


def compare(rec, start, runs):
    """Every compared number of one start, the record against the model
    runs [(table, numbers)], the noise-free run first."""
    leg = rec.leg
    tp = leg['times_px4_us']
    rows = rec.rows
    nums = [m for _, m in runs]
    tables = [t for t, _ in runs]
    out = []

    def add(section, metric, unit, get, kind, tol=None, dirs=None, note='', allow=0, tol_of=None, noisy=False):
        values = []

        for m in [leg] + nums:
            try:
                values.append(get(m))
            except (KeyError, TypeError, IndexError):
                values.append(None)

        if tol_of is not None:
            tol = tol_of(values[0])

        out.append(_row(section, metric, unit, values[0], values[1:], kind, tol, dirs, note, allow, noisy))

    r_last5 = pct(R_LAST5[0], R_LAST5[1] if rec.vectored else R_LAST5[2])

    if start == 'acro':
        c = [centred(rec, t) for t in tables]
        get = lambda key: [None if x is None else x[key] for x in c]
        first = c[0] or {}
        raw = get('raw')
        note = ('the raw rms of the gyro rows %s deg/s%s; the record\'s own noise %s deg/s (its rows about their '
                'moving mean); %s of %s rows' % (
                    '-' if raw[0] is None else '%.4f' % raw[0],
                    '' if len(raw) < 2 else ' (seeds %s)' % ', '.join('-' if x is None else '%.4f' % x for x in raw[1:]),
                    '-' if first.get('noise') is None else '%.4f' % first['noise'], first.get('smoothed_rows', '-'),
                    first.get('rows', '-')))
        out.append(_row('centred', 'r rms of the %g s moving means, model against the record' % CENTRED_WINDOW_S,
                        'deg/s', None, get('rms'), 'upper', CENTRED_RMS, note=note))
        add('stick', 'r mean over the last 5 s', 'deg/s', lambda m: m['stick_phase']['r_mean_last5_deg_s'], 'value',
            dirs=YAW, tol_of=r_last5, noisy=True)
        add('stick', 'r peak, 0.5 s moving mean (time window)', 'deg/s',
            lambda m: m['stick_phase']['r_peak_smoothed_05s_deg_s'], 'value', dirs=YAW, tol_of=pct(*R_PEAK), noisy=True)
        add('stick', 'integrator at the stick end', '', lambda m: m['stick_phase']['integrator_end'], 'value',
            INTEG_END, YAW)
        add('stick', 'saturation flag + at the stick end', '', lambda m: m['stick_phase']['flag_pos_end'], 'bool')
        add('stick', 'saturation flag - at the stick end', '', lambda m: m['stick_phase']['flag_neg_end'], 'bool')
        add('stick', 'unallocated yaw sign at the stick end', '',
            lambda m: None if not finite(m['stick_phase']['unallocated_end'])
            else int(np.sign(m['stick_phase']['unallocated_end'])), 'bool')
        add('stick', 'heading change in the stick', 'deg', lambda m: m['stick_phase']['heading_change_deg'], 'value',
            dirs=YAW, tol_of=pct(*HEADING))
        a, b = tp['off_us'] - int(HOLD_K_WINDOW_S * 1e6), tp['off_us']
        k_rec, r_rec = _hold_k(rows['t_px4_us'], rows['torque_z'], rows['r_deg_s'], a, b)
        hold = [_hold_k(np.asarray(t['t_px4_us']).astype(np.int64), t['torque_z'], t['r_deg_s'], a, b)
                for t in tables]
        out.append(_row('stick', 'hold torque per rad/s over the stick\'s last 2 s', 'per rad/s', k_rec,
                        [k for k, _ in hold], 'rel', HOLD_K, noisy=True,
                        note='at r %s / %s deg/s (record / model): its denominator is the gyro mean' % tuple(
                            '-' if x is None else '%.3f' % x for x in (r_rec, hold[0][1]))))
        add('speed', 'u, 0.5 s mean at the stick start', 'm/s', lambda m: m['stick_phase']['u_start'], 'value',
            dirs=SURGE, tol_of=pct(*U_MEAN))
        add('speed', 'u, 0.5 s mean at the stick end', 'm/s', lambda m: m['stick_phase']['u_end'], 'value',
            dirs=SURGE, tol_of=pct(*U_MEAN))

    add('release', 'heading change in the release', 'deg', lambda m: m['release']['heading_change_deg'], 'value',
        dirs=YAW, tol_of=pct(*HEADING))

    for mark in (leg.get('release') or {}).get('marks_after_s') or {}:
        add('release', 'r, 1 s mean ending at %s s' % mark, 'deg/s', lambda m, k=mark: m['release']['r_mean1s_deg_s'][k],
            'value', dirs=YAW, tol_of=pct(*R_MARK), noisy=True)
        add('release', 'integrator at %s s' % mark, '', lambda m, k=mark: m['release']['integrator'][k], 'value',
            INTEG_MARK, YAW)

    add('speed', 'u, 0.5 s mean at the release end', 'm/s', lambda m: m['release']['u_end'], 'value', dirs=SURGE,
        tol_of=pct(*U_MEAN))
    out += reversal_rows(rec, start, tables, nums)
    out += fin_rows(rec, start, nums)
    return out


def reversal_rows(rec, start, tables, nums):
    leg = rec.leg
    tp = leg['times_px4_us']
    rows = rec.rows
    off = tp['off_us']
    phases = (('stick', 'stick_phase'), ('release', 'release')) if start == 'acro' else (('release', 'release'),)
    window = {'stick': (tp['on_us'], off), 'release': (off, tp['release_end_us'] + 1)}
    near = lambda times: any(abs(t - off) <= FLIP_NEAR_S * 1e6 for t in times)
    allowed = 'a flip within %.1f s of the release: +-1' % FLIP_NEAR_S
    t_of = lambda table: np.asarray(table['t_px4_us']).astype(np.int64)
    out = []

    def get(m, block, *path):
        x = m[block]['reversals']

        for p in path:
            x = x[p]

        return x

    def count(m, block, *path):
        try:
            return get(m, block, *path)
        except (KeyError, TypeError):
            return None

    if rec.vectored:
        for pod, column in (('stbd', 'tilt_stbd_deg'), ('port', 'tilt_port_deg')):
            flips = [_flip_times(rows['t_px4_us'], rows[column], 'pod')]
            flips += [_flip_times(t_of(t), t[column], 'pod') for t in tables]
            allow = 1 if any(near(f) for f in flips) else 0

            for phase, block in phases:
                for what in ('into_rear', 'back_forward'):
                    out.append(_row('reversals', '%s pod %s, %s' % (pod, what.replace('_', ' '), phase), '',
                                    count(leg, block, pod, what), [count(m, block, pod, what) for m in nums], 'count',
                                    note=allowed if allow else '', allow=allow))

                moves = max([get(leg, block, pod, 'into_rear') + get(leg, block, pod, 'back_forward')]
                            + [get(m, block, pod, 'into_rear') + get(m, block, pod, 'back_forward') for m in nums])
                a, b = window[phase]
                when = lambda times: ', '.join('%+.2f s' % ((x - off) / 1e6) for x in times if a <= x < b) or 'none'
                out.append(_row('reversals', '%s pod time in the rear, %s' % (pod, phase), 's',
                                get(leg, block, pod, 'rear_s'), [get(m, block, pod, 'rear_s') for m in nums], 'value',
                                REAR_S * max(1, moves),
                                note='%d transitions; its flips from the release: record %s, model %s'
                                % (moves, when(flips[0]), when(flips[1]))))

        flips = [_flip_times(rows['t_px4_us'], rows[c], 'pod') for c in ('tilt_stbd_deg', 'tilt_port_deg')]
        flips += [_flip_times(t_of(t), t[c], 'pod') for t in tables for c in ('tilt_stbd_deg', 'tilt_port_deg')]
        allow = 1 if any(near(f) for f in flips) else 0

        for phase, block in phases:
            out.append(_row('reversals', 'pod flips, %s' % phase, '', count(leg, block, 'flips'),
                            [count(m, block, 'flips') for m in nums], 'count', allow=allow,
                            note=allowed if allow else ''))
        return out

    # the Cloudship's tail, on its command (actuator_motors) and on the yaw
    # torque setpoint the command is clamped from
    flips = [_flip_times(rows['t_px4_us'], rows['motor3'], 'tail'), _flip_times(rows['t_px4_us'], rows['torque_z'], 'tail')]
    flips += [_flip_times(t_of(t), t[c], 'tail') for t in tables for c in ('motor3', 'torque_z')]
    allow = 1 if any(near(f) for f in flips) else 0

    for phase, block in phases:
        for key, name in (('tail_flips', 'command'), ('tail_flips_torque', 'torque setpoint')):
            out.append(_row('reversals', 'tail flips (%s), %s' % (name, phase), '', count(leg, block, key),
                            [count(m, block, key) for m in nums], 'count', allow=allow, note=allowed if allow else ''))

    return out


def fin_entries(m, block):
    """[(name, ratio, stalled, rows without flow)] of one phase: each vertical
    fin by its name (dorsal/ventral: its cp against the centre of mass), its
    largest |alpha| / alpha_stall where the plugin has flow (the record's
    max_ratio; None where no row has flow), stalled when that is past 1."""
    out = []

    for fin in m[block]['fins']:
        ratio = fin['max_ratio']
        out.append((fin['name'], ratio, None if ratio is None else ratio > 1.0, fin['no_flow_rows']))

    return out


def fin_rows(rec, start, nums):
    """The fins' rows. The ratio's tolerance is max(0.05, 5 % of the
    record's); the note gives each side's rows without flow out of the
    phase's rows, and marks a ratio at the broadside bound (|alpha| near 90
    deg, u_cp near 0: the flow's direction, set by the few rows where u_cp
    passes 0, not a lift-curve angle). The stalled verdict is the noise-free
    run's, indeterminate where either ratio is in STALL_BAND or the model's
    ratio band (noise_band: the noise-free ratio +- NOISE_K s) holds 1."""
    leg = rec.leg
    tp = leg['times_px4_us']
    phases = (('stick', 'stick_phase', tp['on_us'], tp['off_us']),
              ('release', 'release', tp['off_us'], tp['release_end_us'] + 1))
    phases = phases if start == 'acro' else phases[1:]
    stall = {f.get('name'): math.degrees(f['alpha_stall']) for f in ((rec.doc or {}).get('plant') or {}).get(
        'vertical_fins') or [] if finite(f.get('alpha_stall'))}
    out = []

    for phase, block, a, b in phases:
        theirs = fin_entries(leg, block)
        runs = [{name: (ratio, stalled, rows) for name, ratio, stalled, rows in fin_entries(m, block)} for m in nums]
        n_rows = int(_rows_between(rec.rows['t_px4_us'], a, b).sum())

        for name, ratio, stalled, no_flow in theirs:
            models = [run.get(name, (None, None, None)) for run in runs]
            notes = []

            if no_flow or models[0][2]:
                notes.append('no flow on %s (record) and %s (model) of %d rows' % (no_flow, models[0][2], n_rows))

            bound = 90.0 / stall[name] if stall.get(name) else None
            past = [x for x in (ratio, models[0][0]) if bound and finite(x) and x >= FIN_BROADSIDE * bound]

            if past:
                notes.append('at the broadside bound %.3f (90 deg of %.2f): u_cp near 0, the flow\'s direction, not '
                             'a lift-curve angle' % (bound, stall[name]))

            out.append(_row('fins', '%s fin max |alpha| / alpha_stall, %s' % (name, phase), '', ratio,
                            [x[0] for x in models], 'value', pct(*FIN_RATIO)(ratio), note='; '.join(notes), noisy=True))
            inside = [x for x in (ratio, models[0][0]) if finite(x) and STALL_BAND[0] <= x <= STALL_BAND[1]]
            lo, hi, half = noise_band(models[0][0], [x[0] for x in models[1:]])
            why = ['ratio %s in %g..%g' % (', '.join('%.3f' % x for x in inside), *STALL_BAND)] if inside else []
            why += ['the model\'s band %.3f..%.3f holds 1' % (lo, hi)] if half and lo <= 1.0 <= hi else []
            out.append(_row('fins', '%s fin stalled, %s' % (name, phase), '', stalled, [x[1] for x in models], 'stalled',
                            note='indeterminate: %s' % '; '.join(why) if why else '', noisy=True))

    return out


def entry_rows(rec, start, runs, model, notes):
    """The entry-state differences, record against the model, as rows of
    verdict 'info'. model: the noise-free run's leg; notes: the inputs'
    provenance."""
    leg = rec.leg
    e, s = model['entry'], model['setup']
    measured = runs[0][1]
    out = []

    def info(where, metric, unit, record, value, dirs=None, note=''):
        row = {'section': 'entry', 'metric': '%s: %s' % (where, metric), 'unit': unit, 'record': record,
               'model': value, 'seeds': [], 'kind': 'info', 'tol': None, 'dirs': dirs, 'note': note,
               'diff': value - record if finite(record) and finite(value) else None, 'noisy': False, 'band': None,
               'verdict': 'info', 'verdict_noise_free': 'info'}
        out.append(row)
        return row

    # (the record's key, the model entry's key, the printed name, unit, directions)
    names = (('u', 'u', 'u', 'm/s', SURGE), ('v', 'v', 'v', 'm/s', SWAY), ('r_deg_s', 'r', 'r', 'deg/s', YAW),
             ('p_deg_s', 'p', 'p', 'deg/s', ROLL), ('roll_deg', 'phi', 'roll', 'deg', ROLL))
    rp = leg['replay']
    where = 'at the Acro switch, inputs' if start == 'acro' else 'at the release, inputs'
    state, acts = start_state(rec, start), start_actuators(rec, start)

    for key, mkey, name, unit, dirs in names:
        info(where, name, unit, state[key], e['acro'][mkey], dirs)

    for i, key in enumerate(tilt_keys(rec)):
        pod = e['acro']['actuators']['pods'][i]
        info(where, key, 'deg', acts[key], pod['tilt'], note='pod mode %s' % pod['mode'])

    motors = list(e['acro']['actuators']['motors']) + ([] if rec.vectored else [e['acro']['actuators']['tail']])

    for key, value in zip(motor_keys(rec), motors):
        info(where, key, '', acts[key], value)

    info(where, 'throttle (thrust setpoint)', '', rp['throttle'], s['flown'],
         note='logged %.6f..%.6f' % (leg['throttle']['min'], leg['throttle']['max']))
    info(where, 'stick', '', rp['stick'][1][1], s['stick'], YAW)
    info(where, 'stick setpoint', 'deg/s', leg['stick']['setpoint_deg_s'], s['sp'], YAW)
    gains = rp['params']

    for k in rg.GAINS:
        info(where, k, '', rg.f32_decimal(gains[k]), s['params'][k], note='the log\'s float32 %r' % gains[k])

    (_, _), (on, _), (off, _) = rp['stick']
    times = s['times']

    if start == 'acro':
        info(where, 'centred length', 's', on, times[2] - times[1])
        info(where, 'stick length', 's', off - on, times[3] - times[2])
        info(where, 'release length', 's', rp['end_s'] - off, times[4] - times[3])
        info(where, 'integrator, first sample after the switch', '', leg['entry']['integrator_first'],
             measured['entry']['integrator_first'], YAW,
             note='%s s after the switch' % leg['entry'].get('integrator_first_after_s'))
    else:
        info(where, 'release length', 's', rp['end_s'] - off, times[4] - times[3])
        info(where, 'integrator', '', rp['integrator_at_release'], e['acro']['integ'], YAW,
             note='the record\'s sample %.3f s before the release' % leg['release']['entry']['integrator_age_s'])
        info(where, 'saturation flag +', '', leg['stick_phase']['flag_pos_end'], e['acro']['sat_pos'])
        info(where, 'saturation flag -', '', leg['stick_phase']['flag_neg_end'], e['acro']['sat_neg'])

    for text in notes:
        out.append({'section': 'entry', 'metric': '%s: provenance' % where, 'unit': '', 'record': None, 'model': None,
                    'seeds': [], 'kind': 'info', 'tol': None, 'dirs': None, 'note': text, 'diff': None,
                    'noisy': False, 'band': None, 'verdict': 'info', 'verdict_noise_free': 'info'})

    def instant(where, rec_state, mod_state, rec_acts, mod_acts, ref_rec, ref_mod, ref_name):
        for key, _, name, unit, dirs in names:
            info(where, name, unit, rec_state.get(key), mod_state.get(key), dirs)

        rh, mh = rec_state.get('heading_deg'), mod_state.get('heading_deg')
        info(where, 'heading change since %s' % ref_name, 'deg',
             rh - ref_rec if finite(rh) and finite(ref_rec) else None,
             mh - ref_mod if finite(mh) and finite(ref_mod) else None, YAW)

        for key, name in (('yawspeed_integ', 'integrator (held sample)'), ('torque_z', 'torque setpoint')):
            info(where, name, '', rec_state.get(key), mod_state.get(key), YAW)

        for key in tilt_keys(rec) + motor_keys(rec):
            info(where, key, 'deg' if key.startswith('tilt') else '', rec_acts.get(key), mod_acts.get(key))

    if start == 'acro':
        ra, ma = leg['entry']['at_acro'], measured['entry']['at_acro']
        rs, ms = leg['entry']['at_stick'], measured['entry']['at_stick']
        instant('at the stick onset (the row at or before it)', rs, ms, leg['entry']['actuators_at_stick'],
                measured['entry']['actuators_at_stick'], ra['heading_deg'], ma['heading_deg'], 'the Acro switch')
        rr, mr = leg['release']['entry'], measured['release']['entry']
        instant('at the release (the row at or before it)', rr['state'], mr['state'], rr['actuators'], mr['actuators'],
                rs['heading_deg'], ms['heading_deg'], 'the stick onset')
        info('at the release (the row at or before it)', 'integrator, last sample', '', rr['integrator'],
             mr['integrator'], YAW, note='record %.3f s old, model %s' % (
                 rr['integrator_age_s'], '-' if mr.get('integrator_age_s') is None else '%.3f s old' % mr['integrator_age_s']))

    return out


# ---------------------------------------------------------------------------
# One leg, one start
# ---------------------------------------------------------------------------

def replay(rec, start='acro', seeds=0):
    """{'status': 'refused' | 'out of tolerance' | 'within tolerance',
    'checks', 'kept' (the kept summary's differences from today's
    measurement), 'entry', 'rows', 'kwargs', 'model' (the noise-free leg)}."""
    result = {'record': rec, 'start': start, 'status': None, 'checks': [], 'kept': None, 'entry': [], 'rows': [],
              'kwargs': None, 'model': None, 'runs': [], 'notes': []}
    checks = preconditions(rec, start)
    result['checks'] = checks

    if rec.doc is not None and rec.leg is not None and rec.kept_leg is not None:
        result['kept'] = kept_differences(rec)

    if not all(ok for _, ok, _ in checks):
        result['status'] = 'refused'
        return result

    label = ('the record\'s run_leg is whole and is forward.replay_kwargs\' reading' if start == 'acro'
             else 'the record\'s release state and the pod modes there are read')

    try:
        kwargs, notes = replay_inputs(rec, start)
    except (LookupError, ValueError, KeyError, TypeError) as why:
        checks.append((label, False, '%s: %s' % (type(why).__name__, why)))
        result['status'] = 'refused'
        return result

    checks.append((label, True, '; '.join(notes)))
    result['kwargs'], result['notes'] = kwargs, notes
    runs = []

    for noise, seed in [(False, 1)] + [(True, s) for s in range(1, seeds + 1)]:
        try:
            model = run_model(kwargs, noise, seed)
        except ValueError as why:            # run_leg refuses an entry state that cannot be a recorded one
            checks.append(('forward.run_leg takes the record\'s entry state', False, str(why)))
            result['status'] = 'refused'
            return result

        table, numbers = measure(rec, model, start)
        runs.append((table, numbers))

        if not noise:
            result['model'] = model

    result['runs'] = runs
    result['entry'] = entry_rows(rec, start, runs, result['model'], notes)
    result['rows'] = compare(rec, start, runs)
    result['status'] = 'out of tolerance' if any(r['verdict'] == 'FAIL' for r in result['rows']) else 'within tolerance'
    return result


# ---------------------------------------------------------------------------
# Text and CSV
# ---------------------------------------------------------------------------

def magnitude(x, unit='', dirs=None, spec='%.4g'):
    """A number with its direction: '4.195 deg/s nose right'; a bool as
    yes/no; '-' for None."""
    if x is None:
        return '-'

    if isinstance(x, (bool, np.bool_)):
        return 'yes' if x else 'no'

    if isinstance(x, (int, np.integer)) and dirs is None:
        return '%d%s' % (x, ' ' + unit if unit else '')

    if not finite(float(x)):
        return 'nan'

    text = spec % (abs(x) if dirs else x) + (' ' + unit if unit else '')

    if dirs and x != 0:
        text += ' ' + (dirs[0] if x > 0 else dirs[1])

    return text


def seed_text(r):
    """The seeds beside the noise-free value: their range as signed numbers
    (+ is the row's first direction), or each verdict; with the band a
    [noise] number is judged on."""
    seeds = r['seeds']

    if not seeds:
        return ''

    if r['kind'] in ('bool', 'stalled'):
        return ' (seeds %s)' % ', '.join(magnitude(x) for x in seeds)

    if r['kind'] == 'upper':
        return ' (seeds %s)' % ', '.join('-' if not finite(x) else '%.4g' % x for x in seeds)

    lo, hi = _range(seeds)
    text = '-' if lo is None else (('%+.4g..%+.4g' if r['dirs'] else '%.4g..%.4g') % (lo, hi))

    if r['noisy'] and r['band'] is not None:
        text += ', band +-%.3g' % r['band']

    return ' (seeds %s)' % text


def row_text(r):
    unit, dirs = r['unit'], r['dirs']
    note = '  [%s]' % r['note'] if r['note'] else ''
    tag = ' [noise]' if r.get('noisy') else ''
    verdict = r['verdict']

    if r.get('verdict_noise_free') not in (None, verdict):
        verdict += ' (noise-free %s)' % r['verdict_noise_free']

    if r['kind'] == 'upper':
        return '%-58s %s%s, at most %g: %s%s' % (r['metric'], magnitude(r['model'], unit), seed_text(r), r['tol'],
                                                 verdict, note)

    if r['kind'] == 'info' and r['record'] is None and r['model'] is None:
        return '%-58s %s' % (r['metric'], r['note'])

    text = '%-58s record %s | model %s%s' % (r['metric'] + tag, magnitude(r['record'], unit, dirs),
                                             magnitude(r['model'], unit, dirs), seed_text(r))

    if r['diff'] is not None:
        text += ' | %+.4g%s' % (r['diff'], ' ' + unit if unit else '')

        if r['kind'] in ('value', 'rel') and finite(r['record']) and r['record'] != 0:
            text += ' (%+.2f %%)' % (100.0 * r['diff'] / abs(r['record']))

    if r['kind'] == 'rel':
        text += ', tol %g %%' % (100 * r['tol'])
    elif r['kind'] == 'count':
        text += ', allowed +-%d' % r['tol']
    elif r['tol'] is not None:
        text += ', tol %.4g' % r['tol']

    return text + ': %s%s' % (verdict, note)


def short(x, n=60):
    text = json.dumps(x, ensure_ascii=False) if not isinstance(x, str) else x
    return text if len(text) <= n else text[:n - 3] + '...'


def kept_text(result):
    """One line on the kept summary against today's measurement."""
    kept = result['kept']

    if kept is None:
        return 'the kept summary: not compared (today\'s measurement of the leg is not there)'

    if not kept:
        return 'the kept summary: identical to today\'s measurement of its log'

    return ('the kept summary: %d values differ from today\'s measurement of its log, which the replay uses: %s%s'
            % (len(kept), '; '.join('%s %s -> %s' % (p, short(a), short(b)) for p, a, b in kept[:4]),
               '' if len(kept) <= 4 else '; ... (every one in the CSV)'))


def report(result, verbose=False):
    rec, start = result['record'], result['start']
    leg = rec.leg or rec.kept_leg or {}
    plan = leg.get('plan') or {}
    sp = (leg.get('stick') or {}).get('setpoint_deg_s')
    print('\n=== %s (%s, %s): %s m/s, stick %s (setpoint %s), throttle %s; from the %s ===' % (
        rec.label, rec.target, ((rec.doc or rec.kept).get('params') or {}).get('SYS_AUTOSTART'),
        '?' if plan.get('speed') is None else '%g' % plan['speed'],
        magnitude((leg.get('stick') or {}).get('value'), '', YAW, '%.2f'), magnitude(sp, 'deg/s', YAW, '%.2f'),
        '?' if (leg.get('replay') or {}).get('throttle') is None else '%.4f' % leg['replay']['throttle'],
        'Acro switch' if start == 'acro' else 'stick\'s release'))
    print('the record: today\'s forward_yaw_check on %s with %s' % (
        rec.log_path or 'no ULog', rec.events_path or 'no events file'))
    print(kept_text(result))
    failed = [(n, d) for n, ok, d in result['checks'] if not ok]
    print('preconditions: %d of %d hold' % (len(result['checks']) - len(failed), len(result['checks'])))

    for name, ok, detail in result['checks']:
        if verbose or not ok:
            print('  %s %s%s' % ('ok  ' if ok else 'FAIL', name, ('  [%s]' % detail) if detail else ''))

    if result['status'] == 'refused':
        print('leg %s, from the %s: REFUSED, %d precondition%s fail: %s' % (
            rec.number, 'Acro switch' if start == 'acro' else 'release', len(failed), '' if len(failed) == 1 else 's',
            '; '.join(n for n, _ in failed)))
        return

    kw = result['kwargs']
    print('replayed with: u0 %s, v0 %s, r0 %s, p0 %s, roll %s; pods %s; motors %s; throttle %.6f exact%s' % (
        magnitude(kw['u0'], 'm/s', SURGE), magnitude(kw['v0'], 'm/s', SWAY), magnitude(kw['r0'], 'deg/s', YAW),
        magnitude(kw['p0'], 'deg/s', ROLL), magnitude(kw['phi0'], 'deg', ROLL),
        ', '.join('%.2f deg %s' % (p['tilt'], p['mode']) for p in kw['pods0']),
        ', '.join('%.4f' % m for m in kw['motors0']), kw['throttle'],
        '' if start == 'acro' else '; integrator %+.5f, flags %s' % (kw['integ0'], kw['sat0'])))
    print('entry states, record | model (model - record):')
    where = None

    for r in result['entry']:
        place, metric = r['metric'].split(': ', 1)

        if place != where:
            print('  %s' % place)
            where = place

        print('    %s' % row_text(dict(r, metric=metric)))

    print('numbers, record | model (model - record), tolerance; judged on the noise-free run, a [noise] number '
          'with seeds on its band:')
    section = None

    for r in result['rows']:
        if r['section'] != section:
            print('  [%s]' % r['section'])
            section = r['section']

        print('    %s' % row_text(r))

    counts = {v: sum(1 for r in result['rows'] if r['verdict'] == v) for v in ('ok', 'FAIL', 'indeterminate', 'n/a')}
    compared = counts['ok'] + counts['FAIL'] + counts['indeterminate']
    extras = ', '.join('%d %s' % (counts[k], k) for k in ('indeterminate', 'n/a') if counts[k])
    banded = [r['metric'] for r in result['rows'] if r['verdict'] != r['verdict_noise_free']]
    head = 'leg %s, from the %s' % (rec.number, 'Acro switch' if start == 'acro' else 'release')
    tail = ('; the seeds\' band decided %d: %s' % (len(banded), '; '.join(banded))) if banded else ''

    if counts['FAIL']:
        print('%s: OUT OF TOLERANCE on %d of %d compared numbers%s: %s%s' % (
            head, counts['FAIL'], compared, ' (%s)' % extras if extras else '',
            '; '.join(r['metric'] for r in result['rows'] if r['verdict'] == 'FAIL'), tail))
    else:
        print('%s: within tolerance on all %d compared numbers%s%s' % (
            head, compared, ' (%s)' % extras if extras else '', tail))


CSV_COLUMNS = ('summary', 'leg', 'start', 'status', 'section', 'metric', 'unit', 'record', 'model', 'seed_min',
               'seed_max', 'model_minus_record', 'tol', 'kind', 'noisy', 'band', 'verdict', 'verdict_noise_free',
               'note')


def csv_rows(result):
    rec = result['record']
    base = {'summary': os.path.basename(rec.summary_path), 'leg': rec.number, 'start': result['start'],
            'status': result['status']}
    out = []

    def cell(x):
        if x is None:
            return ''
        if isinstance(x, (bool, np.bool_)):
            return str(bool(x))
        return repr(float(x)) if isinstance(x, (float, np.floating)) else str(x)

    for name, ok, detail in result['checks']:
        out.append(dict(base, section='precondition', metric=name, verdict='ok' if ok else 'FAIL', note=detail))

    for path, kept, today in result['kept'] or []:
        out.append(dict(base, section='kept summary', metric=path, kind='info', verdict='info',
                        note='kept %s, today %s' % (short(kept, 200), short(today, 200))))

    for r in result['entry'] + result['rows']:
        lo, hi = _range(r['seeds']) if r['kind'] not in ('bool', 'stalled') else (None, None)
        out.append(dict(base, section=r['section'], metric=r['metric'], unit=r['unit'], record=cell(r['record']),
                        model=cell(r['model']), seed_min=cell(lo), seed_max=cell(hi),
                        model_minus_record=cell(r['diff']), tol=cell(r['tol']), kind=r['kind'],
                        noisy=cell(r.get('noisy')), band=cell(r.get('band')), verdict=r['verdict'],
                        verdict_noise_free=r.get('verdict_noise_free'), note=r['note']))

    return out


def write_csv(path, results):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)

    with open(path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_COLUMNS)
        writer.writeheader()

        for result in results:
            writer.writerows(csv_rows(result))


def exit_code(results):
    if any(r['status'] == 'out of tolerance' for r in results):
        return 1

    return 2 if any(r['status'] == 'refused' for r in results) or not results else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description='replay 0b legs of a forward_yaw_check record on the yaw model')
    ap.add_argument('summary', help='sitl_out/forward_<model>_<stamp>_summary.json')
    ap.add_argument('--leg', type=int, action='append', help='the leg number (repeatable); default every leg')
    ap.add_argument('--seeds', type=int, default=3, help='runs with gz\'s gyro noise besides the noise-free one')
    ap.add_argument('--start', choices=('acro', 'release', 'both'), default='acro',
                    help='replay from the Acro switch, from the stick\'s release, or both')
    ap.add_argument('--out', default=OUT, help='directory of the CSV (default yaw_model/replay_out)')
    ap.add_argument('--verbose', action='store_true', help='print every precondition, not only the failed ones')
    args = ap.parse_args(argv)

    if args.seeds < 0:
        ap.error('--seeds: 0 or more')

    try:
        records = load(args.summary, args.leg)
    except (OSError, ValueError, LookupError) as why:
        print('cannot read the record: %s: %s' % (type(why).__name__, why))
        return 2

    starts = ('acro', 'release') if args.start == 'both' else (args.start,)
    results = []

    for rec in records:
        for start in starts:
            result = replay(rec, start, args.seeds)
            report(result, args.verbose)
            results.append(result)

    kept = records[0].kept if records else {}
    name = 'replay_%s_%s%s_%s.csv' % (kept.get('model'), kept.get('stamp'),
                                     ''.join('_leg%d' % n for n in args.leg) if args.leg else '', args.start)
    path = os.path.join(args.out, name)
    write_csv(path, results)
    print('\nwritten %s' % path)
    code = exit_code(results)
    print('exit %d: %s' % (code, {0: 'every leg replayed, every compared number within tolerance',
                                  1: 'a compared number out of tolerance',
                                  2: 'a leg refused, or nothing replayed'}[code]))
    return code


if __name__ == '__main__':
    sys.exit(main())
