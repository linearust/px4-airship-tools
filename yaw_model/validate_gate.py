#!/usr/bin/env python3
"""Does gate.py run the gate the way the simulator run did?

1. Replay. A ClosedLoop that serves the recorded rows of a gate run in read
   order (rate, torque, integ per row) and nothing else. If gate.py's clock
   and I/O mapping are the gate's, sitl_check.main() consumes exactly the
   recorded rows, every row lands in the recorded phase (clean_start's
   variable-length rest wait included: its 1.0 s hold is counted on the
   virtual clock, so a wrong period would shift every later phase), the 't'
   column matches, and the verdict lines equal the chain log's, character
   for character.
   Every replay hands the gate the gains the record was flown with
   (record_gains.py), so the setpoint note and the I-off restore are the
   record's, not the model's HEAD defaults.
2. The same for the runs at HEAD gains (chain12 .. chain14b, copies in
   ./records_head): each run's verdict lines, and the notes its chain script
   kept (the grep and the cut are read from that script), must equal what
   today's sitl_check.py prints on the record's own rows. A run flown before
   a sitl_check change that alters what it prints or how many rows it takes
   cannot match, and is reported with the change, not judged.
3. The reference loop (reference_loop.py) on the four plants, to show the
   events reach a model. Its verdicts are NOT predictions of the SITL.
4. The same at RTF 0.9, to show where wall-clock windows move.
"""
import csv
import glob
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from gate import BENCH, ClosedLoop, Timing, run_gate  # noqa: E402
from reference_loop import reference_loop  # noqa: E402
import controller  # noqa: E402
import record_gains  # noqa: E402

OUT = os.path.join(HERE, 'gate_out')
RECORDS = os.path.join(HERE, 'gate_records')
RECORDS_HEAD = os.path.join(HERE, 'records_head')
SITL_OUT = os.path.join(BENCH, 'sitl_out')


def record_params(run, target):
    """What param_show answered in that run: its gains (record_gains) and
    the airframe's THR_MDL_FAC."""
    params, _ = record_gains.gains(run, target)
    model = target[3:] if target.startswith('gz_') else target
    params['THR_MDL_FAC'] = controller.head_params(model)['THR_MDL_FAC']
    return params


def chain_script(path):
    """(grep pattern, cut width) of the gate line of a chain script: what
    of sitl_check's output its log kept."""
    with open(path, encoding='utf-8') as handle:
        m = re.search(r'sitl_check\.py 2>&1 \| grep -E "([^"]+)" \| cut -c1-(\d+)', handle.read())

    if not m:
        raise LookupError('%s: no sitl_check gate line' % path)

    return m.group(1), int(m.group(2))


class ReplayLoop(ClosedLoop):
    def __init__(self, rows, params):
        self.rows, self.params, self.k = rows, dict(params), 0
        self._armed = False
        self.set_calls = []

    def param(self, name):
        return self.params.get(name)

    def set_param(self, name, value):
        self.set_calls.append(('param', name, value))

    def arm(self):
        self._armed = True

    def disarm(self):
        self._armed = False

    def armed(self):
        return self._armed

    def set_mode(self, mode):
        self.set_calls.append(('mode', mode))

    def set_stick(self, yaw):
        self.set_calls.append(('stick', yaw))

    def set_wind(self, moment):
        self.set_calls.append(('wind', moment))

    def advance(self, dt):
        pass

    def _row(self):
        if self.k >= len(self.rows):
            raise IndexError('the gate asked for row %d; the record has %d' % (self.k + 1, len(self.rows)))
        return self.rows[self.k]

    def read_rate(self):
        return math.radians(float(self._row()['rate']))

    def read_torque(self):
        return float(self._row()['torque'])

    def read_integ(self):
        value = float(self._row()['integ'])
        self.k += 1
        return None if math.isnan(value) else value


def chain_lines(path, target, header='=== gate %s ==='):
    """The indented lines (verdicts and the notes kept) of one gate block
    of a chain log."""
    lines, inside = [], False

    with open(path, encoding='utf-8', errors='replace') as handle:
        for line in handle:
            line = line.rstrip('\n')
            if line.startswith('=== '):
                inside = line == header % target
                continue
            if inside and line.startswith('  '):
                lines.append(line)

    return lines


def cut(line, width=330):
    """chain*.sh pipes through `cut -c1-N`, which GNU cut applies to bytes."""
    return line.encode('utf-8')[:width].decode('utf-8', errors='replace')


def cut330(line):
    return cut(line, 330)


class _FirstRow(Exception):
    pass


def first_row_t(model_name, params):
    """Where gate.py's default Timing stamps the first row of today's
    sitl_check: run it until its first rate read, then add the row's three
    listener calls (its 't' is taken after them, rounded to 0.01 s)."""
    nominal = Timing().listener_s

    def stop(topic, wall):
        if topic == 'vehicle_angular_velocity':
            raise _FirstRow(wall)
        return nominal

    try:
        run_gate(ReplayLoop([], params), model_name, Timing(listener_s=stop))
    except _FirstRow as first:
        return round(first.args[0] + 3 * nominal, 2)

    raise LookupError('the gate read no rate')


def clocked_by(loop, nominal=Timing().listener_s):
    """Listener latency that lands each row's stamp on the record's 't':
    the third read of a row (rate_ctrl_status) takes whatever is left."""
    def latency(topic, wall):
        if topic != 'rate_ctrl_status':
            return nominal
        return max(0.0, float(loop.rows[loop.k]['t']) - wall)
    return latency


def replay(samples_path, model_name, chain_path, target, params, clocked=False, judged=True, script=None,
           header='=== gate %s ==='):
    """clocked=False: gate.py's own Timing (tests the timing model too).
    clocked=True: the record's 't' clocks the rows (tests the sequence and
    the verdicts against a run whose period jittered). params: what
    param_show answered in that run (record_params). script: the chain
    script whose grep and cut made the log (chain_script); None is the
    chain7/chain9 form, verdict lines only, cut at 330 bytes."""
    with open(samples_path, encoding='utf-8') as handle:
        rows = list(csv.DictReader(handle))

    loop = ReplayLoop(rows, params)
    # The pre-arm (param_show calls, the EKF wait) is cosmetic: it only sets
    # where the 't' column starts. It changed under the records: since
    # 2026-09-28 sitl_check shows AS_YAWRATE_P and AS_YAWRATE_FF too, two
    # param_show more than the 24.98 s the chain7 fit assumed. So the first
    # row is placed on the record's first row, by the difference in the EKF
    # wait, before any window opens; everything from that row on is the
    # harness model under test.
    shift = float(rows[0]['t']) - first_row_t(model_name, params)
    timing = (Timing(listener_s=clocked_by(loop), ekf_wait_s=Timing().ekf_wait_s + shift)
              if clocked else Timing(ekf_wait_s=Timing().ekf_wait_s + shift))
    print('replay %s (%d rows) as %s, %s' % (os.path.basename(samples_path), len(rows), model_name,
                                             'clocked by the record' if clocked else 'gate.py timing'))

    try:
        result = run_gate(loop, model_name, timing)
    except IndexError as error:     # the gate wanted more rows than the record has
        print('  %s\n  %s' % (error, ('FAIL' if judged else
                                      'not judged: this run\'s period jittered by phase')))
        return False
    print('  status %s, rows consumed %d of %d' % (result.status, loop.k, len(rows)))

    phase_bad = [i for i, (a, b) in enumerate(zip(result.samples, rows)) if a['phase'] != b['phase']]
    print('  rows in a different phase: %d%s' % (len(phase_bad), '' if not phase_bad else ' (first at row %d)' % phase_bad[0]))

    dt = [a['t'] - float(b['t']) for a, b in zip(result.samples, rows)]
    print('  t (gate.py) - t (recorded): first %+.3f, max |.| %.3f, last %+.3f s over %.1f s'
          % (dt[0], max(abs(d) for d in dt), dt[-1], float(rows[-1]['t']) - float(rows[0]['t'])))

    # 't' is rounded to 0.01 s; a harness constant off by a few ms per call
    # drifts by tenths of a second over the run (validate by mutation: publish
    # 0.2 s -> 4.6 s, client 0.1 s -> 1.3 s).
    ok = loop.k == len(rows) and not phase_bad and (clocked or max(abs(d) for d in dt) <= 0.05)

    if chain_path:
        want = chain_lines(chain_path, target, header)

        if script is None:
            got = [cut330(line) for line in result.gate_lines if line.startswith('  ')]
        else:
            pattern, width = script
            got = [cut(line, width) for line in result.lines if line.startswith('  ') and re.search(pattern, line)]

        same = got == want
        ok = ok and same
        print('  verdict lines equal to %s [%s]: %s (%d lines)'
              % (os.path.basename(chain_path), header % target, same, len(want)))

        if not same:
            for g, w in zip(got, want):
                if g != w:
                    print('    gate.py : %s\n    recorded: %s' % (g, w))
            if len(got) != len(want):
                print('    %d lines against %d' % (len(got), len(want)))

    rtf = rtf_from_wind_check(rows, model_name)

    if rtf:
        print('  RTF from the wind check slope: %.4f +- %.4f (%d rows)' % rtf)

    print('  %s' % (('PASS' if ok else 'FAIL') if judged else
                    'not judged: this run\'s period jittered by phase, which no constant Timing follows'))
    return ok


def rtf_from_wind_check(rows, model_name):
    """RTF from the wind check's rate slope, on an undamped, finless plant
    (1 N m / Izz per sim second, measured per wall second). None elsewhere."""
    import gz_model
    model = gz_model.merged(model_name)

    if gz_model.hydrodynamics(model) or gz_model.lift_drag_fins(model):
        return None

    izz = gz_model.yaw_inertia(gz_model.family(model_name)) + gz_model.added_mass(model).get('rr', 0.0)
    push = [r for r in rows if r['phase'] == '바람 확인'][1:]     # row 0: the wrench is just landing
    x = [float(r['t']) for r in push]
    y = [float(r['rate']) for r in push]
    n = len(x)
    mx, my = sum(x) / n, sum(y) / n
    sxx = sum((a - mx) ** 2 for a in x)
    slope = sum((a - mx) * (b - my) for a, b in zip(x, y)) / sxx
    resid = math.sqrt(sum((b - my - slope * (a - mx)) ** 2 for a, b in zip(x, y)) / (n - 2))
    alpha = math.degrees(1.0 / izz)
    return -slope / alpha, resid / math.sqrt(sxx) / alpha, n


def reference_runs(rtf=1.0):
    os.makedirs(OUT, exist_ok=True)

    for model in ('airship_vectored_independent', 'airship_vectored_independent_am_fin_drag',
                  'cloudship', 'cloudship_am_fin_drag'):
        result = run_gate(reference_loop(model), model, Timing(rtf=rtf))
        tag = '' if rtf == 1.0 else '_rtf%g' % rtf
        path = os.path.join(OUT, 'reference_%s%s.txt' % (model, tag))

        with open(path, 'w', encoding='utf-8') as handle:
            handle.write('\n'.join(result.lines) + '\n\n# events (wall, sim, what)\n')
            handle.writelines('%9.4f %9.4f  %s\n' % e for e in result.events)

        with open(path.replace('.txt', '.csv'), 'w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(result.samples[0].keys()))
            writer.writeheader()
            writer.writerows(result.samples)

        print('=== reference loop, %s, RTF %g: status %s (%s)' % (model, rtf, result.status, os.path.basename(path)))
        for line in result.gate_lines:
            if line.startswith('  '):
                print(line)


# The runs at HEAD gains: (run, gz target, chain log, block header, samples
# file, chain script), all in ./records_head
HEAD_RUNS = []

for _run, (_text, _header, _logs) in record_gains.RUNS.items():
    if not _text.startswith('yaw_model/records_head/'):
        continue

    _tag = record_gains.SERIES_TAG[_run]
    _script = os.path.join(RECORDS_HEAD, re.sub(r'_\d+$', '', _tag) + '.sh')

    for _target in _logs:
        HEAD_RUNS.append((_run, _target, os.path.join(BENCH, _text), _header,
                          os.path.join(RECORDS_HEAD, 'samples_%s_%s.csv' % (_target, _tag)), _script))


def head_replays():
    """Section 2: every HEAD-gain run, clocked by its own rows. Returns
    {(run, target): True / False}."""
    out = {}

    for run, target, text, header, samples, script in HEAD_RUNS:
        out[(run, target)] = replay(samples, target[3:], text, target, record_params(run, target), clocked=True,
                                    script=chain_script(script), header=header)

    return out


def main():
    # The chain7 series (the last gate of chain7_20260925, gz_cloudship_am_fin_drag,
    # sitl_out/samples.csv as of 2026-09-25 23:54) is a copy: chain9 overwrote
    # the original on 2026-09-27 17:36.
    chain7 = (os.path.join(RECORDS, 'samples_gz_cloudship_am_fin_drag_chain7.csv'), 'cloudship_am_fin_drag',
              os.path.join(SITL_OUT, 'chain7_20260925.txt'), 'gz_cloudship_am_fin_drag',
              record_params('chain7_20260925', 'gz_cloudship_am_fin_drag'))
    ok = replay(*chain7)
    ok = replay(*chain7, clocked=True) and ok

    # Per-plant series of the running chain: sitl_out/samples_<target>_chain9.csv,
    # else a copy taken here. The verdict lines are compared only once the
    # chain log has them.
    found = {}

    for folder in (RECORDS, SITL_OUT):
        for path in glob.glob(os.path.join(folder, 'samples_*_chain9.csv')):
            found[os.path.basename(path)] = path

    chain = os.path.join(SITL_OUT, 'chain9_20260927.txt')

    for name in sorted(found):
        target = re.match(r'samples_(.*)_chain9\.csv', name).group(1)
        has = chain_lines(chain, target) if os.path.exists(chain) else []
        model = target[3:] if target.startswith('gz_') else target
        # gate.py's constant timing: reported, not judged (P jitters by phase
        # in these runs and the 12 s window's 38/39 rows follows it)
        params = record_params('chain9_20260927', target)
        replay(found[name], model, None, target, params, judged=False)
        ok = replay(found[name], model, chain if has else None, target, params, clocked=True) and ok

    print('\n=== the runs at HEAD gains (records_head) ===')
    head = head_replays()
    print('HEAD-gain runs whose lines equal today\'s gate on their own rows: %d of %d'
          % (sum(head.values()), len(head)))
    for key, good in head.items():
        if not good:
            print('  differs: %s %s' % key)

    reference_runs()
    reference_runs(rtf=0.9)
    print('\nvalidate_gate: %s' % ('PASS' if ok else 'FAIL'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
