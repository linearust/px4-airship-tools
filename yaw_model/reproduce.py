#!/usr/bin/env python3
"""The reproduction table: every recorded gate number next to the model's.

    python3 reproduce.py                 # writes reproduction_out.txt, reproduction.csv
    python3 reproduce.py --plant 1dof    # the same with the 1-DOF plant (the model before the fix)

Records (copied read-only into ./records/, md5 in reproduction_out.txt):
  chain7_20260925.txt   the clean-start gate of 2026-09-25, all four plants
                        (the task's "recorded gate results")
  chain9_20260927.txt   the gate of 2026-09-27, as far as the running chain has got
  samples_<target>_chain9.csv, samples_gz_cloudship_am_fin_drag_chain7.csv
                        the per-row series of those runs (t, phase, rate deg/s,
                        torque, integ)

The model is closed_loop.build(target) (planar plant by default) run through
gate.run_gate(), i.e. sitl_check.py's own main() on a virtual clock with the
harness timing gate.Timing() (RTF 1.0), at the gains the records were flown
with: record_gains.gains() reads them from each run's flight log (AS_YAWRATE_P
0.5, I 0.09, INT_LIM 0.3, no AS_YAWRATE_FF on 9535f8abcd / 1965de11ab), and
every record of a target must name the same set. The model's own defaults are
HEAD's since 2026-10-01; this replay does not use them.

What "misses" means here (stated, because the gate has no single tolerance):
  TOL  the band the gate itself applies to that number (sitl_check.py line in
       the table): e.g. the tracking mean passes within 0.25 sp = 1.667 deg/s
       of sp, so a model mean off by more than 1.667 misses by more than the
       gate's tolerance. Where the gate applies a one-sided limit, TOL is the
       distance from the limit to the reference it is measured against.
  VERDICT  the model's pass/fail against the record's.
  SPREAD   the run-to-run difference between chain7 and chain9 of the same
       plant, where both exist: the SITL's own repeatability. A model that is
       further from the records than they are from each other is flagged
       ('>spread') even when it is inside TOL.
"""
import argparse
import csv
import hashlib
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import closed_loop as cl  # noqa: E402
import gate  # noqa: E402
import controller as _ctl  # noqa: E402
# the records replayed here were flown before 2026-10-12, on the linear feedforward law
_ctl.FF_LAW = 'linear'
import record_gains  # noqa: E402

RECORDS = os.path.join(HERE, 'records')
CHAINS = (('chain7', os.path.join(RECORDS, 'chain7_20260925.txt')),
          ('chain9', os.path.join(RECORDS, 'chain9_20260927.txt')))
RUN_OF_CHAIN = {'chain7': 'chain7_20260925', 'chain9': 'chain9_20260927'}     # record_gains.RUNS
SERIES = {  # (target, record) -> csv
    ('gz_cloudship_am_fin_drag', 'chain7'): 'samples_gz_cloudship_am_fin_drag_chain7.csv',
    ('gz_cloudship', 'chain9'): 'samples_gz_cloudship_chain9.csv',
    ('gz_airship_vectored_independent', 'chain9'): 'samples_gz_airship_vectored_independent_chain9.csv',
    ('gz_airship_vectored_independent_am_fin_drag', 'chain9'):
        'samples_gz_airship_vectored_independent_am_fin_drag_chain9.csv',
    ('gz_cloudship_am_fin_drag', 'chain9'): 'samples_gz_cloudship_am_fin_drag_chain9.csv',
    ('gz_airship_vectored_independent_fin', 'chain9'): 'samples_gz_airship_vectored_independent_fin_chain9.csv',
}
SP = 6.6667     # (0.5 - 0.1) / 0.9 * 15 deg/s, sitl_check.py:240

NUM = r'(-?[\d.]+)'

# check key, regex on the verdict detail, fields, the gate's tolerance per field
# (sitl_check.py line), and how the field is judged
CHECKS = [
    ('wind check', r'^바람\(외부 토크\)', r'5 초 동안 %s -> %s deg/s .*주는 %s 의' % (NUM, NUM, NUM),
     ('start', 'end', 'expected'), {'end': ('0.5 x expected, :314', None)}),
    ('Manual', r'^Manual 에서', r'토크 %s, 각속도가 %s -> %s deg/s' % (NUM, NUM, NUM),
     ('torque', 'start', 'end'), {'torque': ('0.02, :324', 0.02)}),
    ('Acro brake', r'^Acro 가 남은', r'%s -> %s deg/s' % (NUM, NUM),
     ('start', 'end'), {'end': ('|r| < 6.0, :371', 6.0)}),
    ('Acro track', r'^Acro: 스틱이 지시', r'평균 %s, 최대 %s deg/s; 출발 %s deg/s, 적분기 %s' % (NUM, NUM, NUM, NUM),
     ('mean', 'peak', 'start', 'integ0'), {'mean': ('0.25 sp = 1.667, :402', 0.25 * SP),
                                           'peak': ('< 1.4 sp: 0.4 sp = 2.667, :403', 0.4 * SP)}),
    ('Acro release', r'^Acro: 스틱을 놓으면', r'%s -> %s deg/s, (.*)$' % (NUM, NUM),
     ('start', 'end', 'how'), {'settle': ('< 10.0 s, :425', 10.0)}),
    ('Stab track', r'^Stabilized: 스틱이 지시', r'평균 %s, 최대 %s deg/s; 출발 %s deg/s, 적분기 %s' % (NUM, NUM, NUM, NUM),
     ('mean', 'peak', 'start', 'integ0'), {'mean': ('0.25 sp = 1.667, :402', 0.25 * SP),
                                           'peak': ('< 1.4 sp: 0.4 sp = 2.667, :403', 0.4 * SP)}),
    ('Stab release', r'^Stabilized: 스틱을 놓으면', r'%s -> %s deg/s, (.*)$' % (NUM, NUM),
     ('start', 'end', 'how'), {'settle': ('< 10.0 s, :425', 10.0)}),
    ('wind I on', r'I 켬: 적분기가', r'\|각속도\| %s deg/s, 적분기 %s \(필요한 토크 %s' % (NUM, NUM, NUM),
     ('rate', 'integ', 'needed'), {'rate': ('< 1.0, :443', 1.0), 'integ': ('0.5 needed, :443', None)}),
    ('wind I off', r'I 끔: 계속', r'\|각속도\| %s deg/s, 적분기 %s \(필요한 토크 %s' % (NUM, NUM, NUM),
     ('rate', 'integ', 'needed'), {'rate': ('> 1.0, :443', 1.0)}),
    ('wind weak', r'틸트 밴드 아래', r'\|각속도\| %s deg/s, 적분기 %s \(필요한 토크 %s' % (NUM, NUM, NUM),
     ('rate', 'integ', 'needed'), {'rate': ('< 1.0, :443', 1.0), 'integ': ('0.5 needed, :443', None)}),
]


def parse(lines):
    """{check: {'ok': bool, field: value}} from verdict lines."""
    out = {}

    for line in lines:
        m = re.match(r'^\s*(통과|실패)\s+(.*?) — (.*)$', line)

        if not m:
            continue

        name, detail = m.group(2), m.group(3)

        for key, name_re, detail_re, fields, _ in CHECKS:
            if not re.search(name_re, name):
                continue

            d = re.search(detail_re, detail)
            rec = {'ok': m.group(1) == '통과', 'line': line.strip()}

            if d:
                for f, v in zip(fields, d.groups()):
                    rec[f] = v if f == 'how' else float(v)

                how = rec.get('how')

                if how is not None:
                    s = re.search(r'([\d.]+) 초 만에 정지', how)
                    k = re.search(r'마지막 3 초 (-?[\d.]+) deg/s\^2', how)
                    rec['settle'] = float(s.group(1)) if s else float('nan')
                    rec['slope'] = float(k.group(1)) if k else float('nan')
                    del rec['how']

            out[key] = rec
            break

    return out


def chain_block(path, target):
    lines, inside = [], False

    with open(path, encoding='utf-8', errors='replace') as handle:
        for line in handle:
            line = line.rstrip('\n')

            if line.startswith('=== '):
                inside = line == '=== gate %s ===' % target
                continue

            if inside and line.startswith('  '):
                lines.append(line)

    return lines


def read_series(name):
    path = os.path.join(RECORDS, name)

    if not os.path.exists(path):
        return None

    with open(path, encoding='utf-8') as handle:
        return list(csv.DictReader(handle))


def by_phase(rows):
    """{(phase, occurrence): rows}: a phase name that comes back later
    (clean_start's '정지 대기' and '출발' run once per mode) is a new block."""
    phases, seen, last = {}, {}, None

    for r in rows:
        if r['phase'] != last:
            seen[r['phase']] = seen.get(r['phase'], 0) + 1
            last = r['phase']

        phases.setdefault((r['phase'], seen[r['phase']]), []).append(r)

    return phases


def series_residuals(model_rows, rec_rows):
    """Per phase: rows compared (index-aligned within the phase), rms and
    max |model - record| of rate [deg/s], torque and integ."""
    mp, rp = by_phase(model_rows), by_phase(rec_rows)
    out = []

    for phase, rrows in rp.items():
        mrows = mp.get(phase)

        if not mrows:
            continue

        n = min(len(mrows), len(rrows))
        res = {'phase': phase[0] + ('' if phase[1] == 1 else ' #%d' % phase[1]), 'n': n,
               'n_model': len(mrows), 'n_rec': len(rrows)}

        for key in ('rate', 'torque', 'integ'):
            d = []

            for a, b in zip(mrows[:n], rrows[:n]):
                x, y = float(a[key]), float(b[key])

                if math.isfinite(x) and math.isfinite(y):
                    d.append(x - y)

            res[key] = (math.sqrt(sum(e * e for e in d) / len(d)), max(abs(e) for e in d), sum(d) / len(d)) if d else None

        out.append(res)

    return out


def fmt(v, key):
    if v is None or (isinstance(v, float) and not math.isfinite(v)):
        return '-'

    if key in ('integ', 'integ0', 'needed', 'torque'):
        return '%.4f' % v

    return '%.2f' % v


def record_params(target, chains=CHAINS):
    """(params, sources): the gains every record of `target` was flown
    with; raises unless they are one set."""
    found = []

    for tag, _ in chains:
        run = RUN_OF_CHAIN[tag]

        if record_gains.has_gate(run, target):
            found.append((tag,) + record_gains.gains(run, target))

    if not found:
        raise LookupError('no record of %s carries its gains' % target)

    first = found[0][1]

    for tag, params, _ in found[1:]:
        if params != first:
            raise ValueError('%s: %s flew %s, %s flew %s' % (target, found[0][0], first, tag, params))

    return dict(first), ['%s: %s' % (tag, src) for tag, _, src in found]


def run_model(target, plant, **kw):
    """One gate run of the model at the records' gains (kw may override)."""
    params, _ = record_params(target)
    params.update(kw.pop('params', {}))
    loop = cl.build(target, plant, params=params, **kw)
    return gate.run_gate(loop, cl.ALL_TARGETS[target][0]), loop


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--plant', default='roll', choices=('roll', 'planar', '1dof'))
    ap.add_argument('--out', default=None)
    args = ap.parse_args()
    out_txt = args.out or os.path.join(HERE, 'reproduction_out%s.txt' % ('' if args.plant == 'roll' else '_' + args.plant))
    out_csv = out_txt.replace('.txt', '.csv').replace('reproduction_out', 'reproduction')
    lines = []
    table = []

    def say(s=''):
        lines.append(s)
        print(s)

    say('Reproduction of the SITL yaw gate by the closed-loop model (plant: %s, gains: the records\' own, '
        'record_gains.py)' % args.plant)

    for name in sorted(os.listdir(RECORDS)):
        with open(os.path.join(RECORDS, name), 'rb') as handle:
            say('  record %-66s md5 %s' % (name, hashlib.md5(handle.read()).hexdigest()))

    targets = list(cl.TARGETS) + (list(cl.EXTRA_TARGETS) if args.plant == 'roll' else [])

    for target in targets:
        result, loop = run_model(target, args.plant)
        model = parse(result.gate_lines)
        recs = {}

        for tag, path in CHAINS:
            block = chain_block(path, target)

            if block:
                recs[tag] = parse(block)

        say('\n=== %s%s   model status %s (%d rows; %s)' % (target, '', result.status, len(result.samples),
                                                        ', '.join('%s %d/%d' % (t, sum(r['ok'] for r in v.values()),
                                                                                len(v)) for t, v in recs.items())))
        params, sources = record_params(target)
        say('  gains P %g I %g INT_LIM %g FF %g MAX %g DZ %g from %s'
            % (params['AS_YAWRATE_P'], params['AS_YAWRATE_I'], params['AS_YR_INT_LIM'], params['AS_YAWRATE_FF'],
               params['AS_YAWRATE_MAX'], params['MAN_DEADZONE'], '; '.join(sources)))
        if target in cl.EXTRA_TARGETS:
            say('  (hold-out: not one of the four plants of interest; nothing was built or checked against it)')
        say('  %-13s %-9s %9s %9s %9s  %8s  %-34s %s' % ('check', 'field', 'chain7', 'chain9', 'model',
                                                          'd(model)', 'gate tolerance (sitl_check.py)', 'flags'))

        for key, _, _, fields, tols in CHECKS:
            if key not in model and not any(key in r for r in recs.values()):
                continue

            m = model.get(key, {})
            shown = [f for f in fields if f not in ('how', 'expected', 'needed')]

            if 'release' in key:
                shown += ['settle', 'slope']

            for f in shown:
                rv = {t: recs[t].get(key, {}).get(f) for t in recs}
                mv = m.get(f)
                refs = [v for v in rv.values() if v is not None and math.isfinite(v)]
                d = None

                if mv is not None and math.isfinite(mv) and refs:
                    d = max((mv - v for v in refs), key=abs)

                tol_text, tol = tols.get(f, ('', None))

                if f == 'end' and key == 'wind check':
                    tol = 0.5 * m.get('expected', float('nan'))
                if f == 'integ' and tol_text.startswith('0.5 needed'):
                    tol = 0.5 * m.get('needed', float('nan'))

                flags = []

                if tol is not None and d is not None and abs(d) > tol:
                    flags.append('>TOL')

                if len(refs) == 2 and d is not None and abs(d) > abs(refs[0] - refs[1]) + 0.01:
                    flags.append('>spread(%.2f)' % abs(refs[0] - refs[1]))

                say('  %-13s %-9s %9s %9s %9s  %8s  %-34s %s'
                    % (key, f, fmt(rv.get('chain7'), f), fmt(rv.get('chain9'), f), fmt(mv, f),
                       fmt(d, f), tol_text, ' '.join(flags)))
                table.append({'target': target, 'check': key, 'field': f,
                              'chain7': rv.get('chain7'), 'chain9': rv.get('chain9'), 'model': mv,
                              'max_dev': d, 'tolerance': tol, 'tolerance_source': tol_text,
                              'flags': ' '.join(flags)})

            vr = {t: recs[t][key]['ok'] for t in recs if key in recs[t]}
            vm = m.get('ok')
            agree = all(v == vm for v in vr.values())
            say('  %-13s %-9s %9s %9s %9s  %8s' % (key, 'VERDICT', *(('pass' if vr[t] else 'FAIL') if t in vr else '-'
                                                                  for t in ('chain7', 'chain9')),
                                                    ('pass' if vm else 'FAIL') if vm is not None else '-',
                                                    'agree' if agree else 'DISAGREE'))

        for tag in ('chain7', 'chain9'):
            if (target, tag) not in SERIES:
                continue

            rows = read_series(SERIES[(target, tag)])

            if rows is None:
                continue

            say('  time series vs %s (%s): per phase, index-aligned rows, model - record' % (tag, SERIES[(target, tag)]))
            say('    %-40s %4s %5s   %-24s %-26s %-26s' % ('phase', 'n', 'm/r', 'rate rms/max/mean deg/s',
                                                           'torque rms/max/mean', 'integ rms/max/mean'))

            for res in series_residuals(result.samples, rows):
                def trip(x, f):
                    return '-' if x is None else (f % x)
                say('    %-40s %4d %2d/%-2d  %-24s %-26s %-26s'
                    % (res['phase'][:40], res['n'], res['n_model'], res['n_rec'],
                       trip(res['rate'], '%.2f/%.2f/%+.2f'), trip(res['torque'], '%.4f/%.4f/%+.4f'),
                       trip(res['integ'], '%.4f/%.4f/%+.4f')))

    with open(out_txt, 'w') as handle:
        handle.write('\n'.join(lines) + '\n')

    with open(out_csv, 'w', newline='') as handle:
        w = csv.DictWriter(handle, fieldnames=list(table[0].keys()))
        w.writeheader()
        w.writerows(table)

    print('\nwritten %s and %s' % (out_txt, out_csv))


if __name__ == '__main__':
    main()
