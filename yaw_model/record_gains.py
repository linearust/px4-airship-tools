#!/usr/bin/env python3
"""The gains each recorded SITL gate run was flown with, read from that run.

    python3 record_gains.py          # every registered run: gains, source, cross-check
    python3 record_gains.py --write  # keep each present log's values in records/gains.json

The model's defaults are HEAD's (controller.DEFAULT_PARAMS, read from the
yaml), and a replay of an older run must not inherit them: chain7 and chain9
were flown at AS_YAWRATE_P 0.5 on firmware without AS_YAWRATE_FF, chain10 at
P 0.7, the stage-2 candidate B at P 0.65 I 0.07 with a feedforward per plant,
and chain12 onward at HEAD's gains. Every replay entry point asks here and
passes what it gets explicitly.

Sources, both read, never typed in:
  1. the run's flight log: the SITL ulog's initial parameters (pyulog). Each
     registered run names its log below; the log is checked to be the right
     airframe (its SYS_AUTOSTART against the target's posix airframe,
     controller.airframe_for_model). The map was built on 2026-10-01 by
     matching each samples_<target>_<run>.csv's write time (the end of its
     gate run, cp -p) against every log's [start, end] window and airframe:
     exactly one log per run, no ambiguity.
  2. from chain12 (2026-09-28) on, the record itself: sitl_check.py prints
     the gains it flew in its '지시 각속도' note and the chain scripts keep
     that line. Where both exist they must agree, or this module raises.
  3. records/gains.json: the values this module read from each log, written
     by --write while the logs were there, so a record replays on a machine
     without the PX4 build directory (CI). A log that is present is still
     the reading, and it must agree with the copy, or this module raises.

A firmware without AS_YAWRATE_FF (before d2647379af: chain7, chain9, chain10)
had no feedforward at all: RateControl's _gain_ff is a zero-initialised
matrix::Vector3f (rate_control.hpp:131) that nothing set, so FF is 0.0 there.
That is read too: the log must lack the parameter for it to count as 0.

ulog values are float32; a value is taken as the shortest decimal that names
the same float32 ('%.7g'), so 0.699999988 reads as 0.7.
"""
import functools
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import controller as ctl  # noqa: E402

LOGDIR = os.path.join(ctl.PX4, 'build/px4_sitl_default/rootfs/fs/log')
BENCH = os.path.dirname(HERE)
GAINS = ('AS_YAWRATE_P', 'AS_YAWRATE_I', 'AS_YR_INT_LIM', 'AS_YAWRATE_FF', 'AS_YAWRATE_MAX', 'MAN_DEADZONE')
KEPT = os.path.join(HERE, 'records', 'gains.json')     # {run: {target: {name: logged value}}}
KEPT_NAMES = GAINS + ('SYS_AUTOSTART',)

V, V27, VFIN = ('gz_airship_vectored_independent', 'gz_airship_vectored_independent_am_fin_drag',
                'gz_airship_vectored_independent_fin')
CS, CS17 = 'gz_cloudship', 'gz_cloudship_am_fin_drag'

# run -> (its chain log under the bench tools, the header of its block for a
# target ('%s' = target), {target: ulog under LOGDIR}). The chain logs of the
# replayed runs are the read-only copies in yaw_model/records (P 0.5) and
# yaw_model/records_head (HEAD gains); chain10 and stage 2 are read where the
# chain left them.
RUNS = {
    'chain7_20260925': ('yaw_model/records/chain7_20260925.txt', '=== gate %s ===', {
        V: '2026-09-25/14_34_05.ulg', V27: '2026-09-25/14_39_53.ulg',
        CS: '2026-09-25/14_45_54.ulg', CS17: '2026-09-25/14_50_29.ulg'}),
    'chain9_20260927': ('yaw_model/records/chain9_20260927.txt', '=== gate %s ===', {
        CS: '2026-09-27/08_31_54.ulg', V: '2026-09-27/08_36_30.ulg', VFIN: '2026-09-27/08_42_18.ulg',
        V27: '2026-09-27/08_47_52.ulg', CS17: '2026-09-27/08_53_51.ulg'}),
    'chain10_20260927': ('sitl_out/chain10_20260927.txt', '=== gate %s ===', {
        V: '2026-09-27/13_33_31.ulg', V27: '2026-09-27/13_39_23.ulg', VFIN: '2026-09-27/13_45_18.ulg',
        CS: '2026-09-27/13_50_53.ulg', CS17: '2026-09-27/13_55_25.ulg'}),
    'stage2B_20260927': ('sitl_out/stage2_20260927.txt', None, {
        V: '2026-09-27/14_34_10.ulg', V27: '2026-09-27/14_40_07.ulg', VFIN: '2026-09-27/14_45_53.ulg',
        CS: '2026-09-27/14_51_26.ulg', CS17: '2026-09-27/14_56_02.ulg'}),
    'chain12_20260928': ('yaw_model/records_head/chain12_20260928.txt', '=== gate %s ===', {
        V: '2026-09-28/11_02_30.ulg', V27: '2026-09-28/11_08_34.ulg', VFIN: '2026-09-28/11_15_02.ulg',
        CS: '2026-09-28/11_20_44.ulg', CS17: '2026-09-28/11_25_26.ulg'}),
    'chain12b_20260928': ('yaw_model/records_head/chain12b_20260928.txt', '=== gate %s ===', {
        CS17: '2026-09-28/11_33_36.ulg', V: '2026-09-28/11_38_12.ulg', V27: '2026-09-28/11_44_04.ulg',
        VFIN: '2026-09-28/11_50_15.ulg', CS: '2026-09-28/11_55_48.ulg'}),
    'chain12c_1_20260928': ('yaw_model/records_head/chain12c_20260928.txt', '=== gate %s run 1 ===', {
        V27: '2026-09-28/12_02_57.ulg'}),
    'chain12c_2_20260928': ('yaw_model/records_head/chain12c_20260928.txt', '=== gate %s run 2 ===', {
        V27: '2026-09-28/12_09_10.ulg'}),
    'chain13_20260928': ('yaw_model/records_head/chain13_20260928.txt', '=== gate %s ===', {
        V: '2026-09-28/13_47_30.ulg', V27: '2026-09-28/13_53_22.ulg', VFIN: '2026-09-28/13_59_33.ulg',
        CS: '2026-09-28/14_05_05.ulg', CS17: '2026-09-28/14_09_38.ulg'}),
    'chain14_20260929': ('yaw_model/records_head/chain14_20260929.txt', '=== gate %s ===', {
        V: '2026-09-29/02_58_59.ulg', V27: '2026-09-29/03_05_52.ulg', VFIN: '2026-09-29/03_12_03.ulg',
        CS: '2026-09-29/03_17_35.ulg', CS17: '2026-09-29/03_22_07.ulg'}),
    'chain14b_1_20260929': ('yaw_model/records_head/chain14b_20260929.txt', '=== gate %s run 1 ===', {
        V: '2026-09-29/03_31_02.ulg'}),
    'chain14b_2_20260929': ('yaw_model/records_head/chain14b_20260929.txt', '=== gate %s run 2 ===', {
        V: '2026-09-29/03_36_55.ulg'}),
}
# The samples series of a run, sitl_out/samples_<target>_<this>.csv
SERIES_TAG = {
    'chain7_20260925': None, 'chain9_20260927': 'chain9', 'chain10_20260927': 'chain10',
    'stage2B_20260927': 'B', 'chain12_20260928': 'chain12', 'chain12b_20260928': 'chain12b',
    'chain12c_1_20260928': 'chain12c_1', 'chain12c_2_20260928': 'chain12c_2', 'chain13_20260928': 'chain13',
    'chain14_20260929': 'chain14', 'chain14b_1_20260929': 'chain14b_1', 'chain14b_2_20260929': 'chain14b_2',
}
NOTE = re.compile(r'지시 각속도 [\d.]+ deg/s \(AS_YAWRATE_MAX (\S+), MAN_DEADZONE (\S+), AS_YAWRATE_P (\S+), '
                  r'AS_YAWRATE_I (\S+), AS_YAWRATE_FF (\S+)\)')


def f32(value):
    """The shortest decimal that names the same float32."""
    return float('%.7g' % float(value))


@functools.lru_cache(maxsize=None)
def ulog_params(relpath):
    """Initial parameters of one SITL log, {name: value}. Raises when the
    log is not there: a run whose gains cannot be read is not replayed at
    the defaults."""
    from pyulog import ULog
    path = os.path.join(LOGDIR, relpath)

    if not os.path.exists(path):
        raise FileNotFoundError('%s: the flight log of this run is gone; its gains cannot be read' % path)

    return dict(ULog(path, []).initial_parameters)


def kept_values(ip):
    """The logged values the gains need, as records/gains.json keeps them:
    the shortest decimals, SYS_AUTOSTART whole, a parameter the firmware
    did not have left out (its absence is what reads as FF 0)."""
    return {name: (int(ip[name]) if name == 'SYS_AUTOSTART' else f32(ip[name]))
            for name in KEPT_NAMES if name in ip}


def kept():
    try:
        with open(KEPT, encoding='utf-8') as handle:
            return json.load(handle)
    except FileNotFoundError:
        return {}


def log_params(run, target):
    """(logged values, where from) for one run's gate: the flight log when
    it is there, else the copy --write kept of it. With both, they agree or
    this raises; with neither, the run is not replayed."""
    relpath = RUNS[run][2][target]
    copy = kept().get(run, {}).get(target)

    if os.path.exists(os.path.join(LOGDIR, relpath)):
        values = kept_values(ulog_params(relpath))

        if copy is not None and copy != values:
            raise ValueError('%s %s: records/gains.json keeps %s, the log %s reads %s'
                             % (run, target, copy, relpath, values))

        return values, 'ulog %s' % relpath

    if copy is None:
        raise FileNotFoundError('%s: the flight log of this run is gone and records/gains.json has no copy; '
                                'its gains cannot be read' % os.path.join(LOGDIR, relpath))

    return copy, 'records/gains.json (from ulog %s)' % relpath


def block_lines(path, header):
    """The lines of one gate block of a chain log (between its header and
    the next '=== ' line)."""
    out, inside = [], False

    with open(path, encoding='utf-8', errors='replace') as handle:
        for line in handle:
            line = line.rstrip('\n')

            if line.startswith('=== '):
                inside = line == header
                continue

            if inside:
                out.append(line)

    return out


def from_note(run, target):
    """The gains the record's own '지시 각속도' note names, or None when the
    run predates that note."""
    text, header, _ = RUNS[run]

    if header is None:
        return None

    for line in block_lines(os.path.join(BENCH, text), header % target):
        m = NOTE.search(line)

        if m:
            mx, dz, p, i, ff = (float(x) for x in m.groups())
            return {'AS_YAWRATE_MAX': mx, 'MAN_DEADZONE': dz, 'AS_YAWRATE_P': p, 'AS_YAWRATE_I': i,
                    'AS_YAWRATE_FF': ff}

    return None


def gains(run, target):
    """(params, source): the gains `target` flew in `run`, from its log,
    cross-checked against the record's own note where it has one."""
    if run not in RUNS:
        raise KeyError('%s is not a registered run (%s)' % (run, ', '.join(RUNS)))

    logs = RUNS[run][2]

    if target not in logs:
        raise KeyError('%s has no %s gate' % (run, target))

    ip, where = log_params(run, target)
    model = target[3:] if target.startswith('gz_') else target
    want = ctl.airframe_for_model(model)

    if int(ip.get('SYS_AUTOSTART', -1)) != want:
        raise ValueError('%s: %s logged SYS_AUTOSTART %s, the target is %d'
                         % (run, logs[target], ip.get('SYS_AUTOSTART'), want))

    params = {}

    for name in GAINS:
        if name in ip:
            params[name] = f32(ip[name])
        elif name == 'AS_YAWRATE_FF':
            params[name] = 0.0          # the firmware had no feedforward (module docstring)
        else:
            raise LookupError('%s: %s does not log %s' % (run, logs[target], name))

    source = '%s (SYS_AUTOSTART %d%s)' % (where, want, '' if 'AS_YAWRATE_FF' in ip else ', no AS_YAWRATE_FF: FF 0')
    note = from_note(run, target)

    if note is not None:
        bad = {k: (v, params[k]) for k, v in note.items() if abs(v - params[k]) > 1e-9}

        if bad:
            raise ValueError('%s %s: the record says %s, its log %s' % (run, target, note, params))

        source += ' = the record\'s note'

    return params, source


def write_kept():
    """records/gains.json from every log that is present; runs whose log is
    gone keep whatever the file already held for them."""
    out = kept()
    present = 0

    for run, (_, _, logs) in RUNS.items():
        for target, relpath in logs.items():
            if os.path.exists(os.path.join(LOGDIR, relpath)):
                out.setdefault(run, {})[target] = kept_values(ulog_params(relpath))
                present += 1

    with open(KEPT, 'w', encoding='utf-8') as handle:
        json.dump(out, handle, indent=1, sort_keys=True)
        handle.write('\n')

    print('%s: %d gates from their logs, %d kept in all' % (os.path.relpath(KEPT, BENCH), present,
                                                              sum(len(v) for v in out.values())))


def main(argv):
    if argv == ['--write']:
        write_kept()

    for run, (_, _, logs) in RUNS.items():
        for target in logs:
            params, source = gains(run, target)
            print('%-22s %-46s P %-5g I %-5g LIM %-4g FF %-5g MAX %-4g DZ %-4g  %s'
                  % (run, target, params['AS_YAWRATE_P'], params['AS_YAWRATE_I'], params['AS_YR_INT_LIM'],
                     params['AS_YAWRATE_FF'], params['AS_YAWRATE_MAX'], params['MAN_DEADZONE'], source))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
