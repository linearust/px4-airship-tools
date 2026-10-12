#!/usr/bin/env python3
"""The 48 at-rest gate verdicts at HEAD gains, reproduced by the model.

    python3 reproduce_head.py              # writes reproduction_head_out.txt
    from reproduce_head import reproduce   # (lines, summary) for a report

The runs: every gate the chain logs of 2026-09-28/29 recorded on firmware at
AS_YAWRATE_P 0.7 with the per-airframe AS_YAWRATE_FF (chain12, 12b, 12c x2,
13, 14, 14b x2; read-only copies with their samples and chain scripts in
./records_head, md5 printed). Each target's gains are read from its runs
(record_gains.gains: the flight log, cross-checked against the gains line the
record prints) and must be one set; the model flies exactly that set
(closed_loop.build(params=...)), through gate.run_gate, i.e. today's
sitl_check.py on the virtual clock.

The 48: 2520 10, 2527 10, 2522 10, 2510 9, 2517 9. A check's recorded verdict
is what every run of that target says, leaving out the runs in which the
harness, not the firmware, decided that check. Those are named in HARNESS
below with the record that says so, and the model, whose harness drops
nothing, is still compared with them and the difference printed.

Tolerances, as reproduce.py states them (its CHECKS table): per number, TOL
is the band sitl_check itself applies to it (the tracking mean within
0.25 sp, the peak under 1.4 sp, the brake under 6 deg/s, a release settling
in under 10 s, the wind cases' rate and integrator bands); SPREAD is the
range of the clean runs, and a model value further from some run than the
runs are from each other ('>spread') is flagged even inside TOL. The verdict
is the gate's own pass/fail, compared exactly.
"""
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
import reproduce as rp  # noqa: E402

RECORDS_HEAD = os.path.join(HERE, 'records_head')
TARGETS = ['gz_airship_vectored_independent', 'gz_airship_vectored_independent_am_fin_drag',
           'gz_airship_vectored_independent_fin', 'gz_cloudship', 'gz_cloudship_am_fin_drag']
RUNS = [run for run, (text, _, _) in record_gains.RUNS.items() if text.startswith('yaw_model/records_head/')]

# (run, target, check): why the record's verdict of that check is the
# harness's, with the record that says so (bench-relative path:line)
HARNESS = {
    ('chain12_20260928', 'gz_cloudship_am_fin_drag', 'Acro track'):
        'the triple wrench clear delayed the Manual check 2.4 s and every check after it started from '
        'another state (yaw_retune_20260927.md:166-172; fixed for chain12b, sitl_out/chain12b.sh:2-4)',
    ('chain12b_20260928', 'gz_airship_vectored_independent_am_fin_drag', 'wind check'):
        'gz transport dropped the push (0.02 -> 0.01 deg/s in 5 s; yaw_retune_20260927.md:174-179, '
        'sitl_out/chain12c.sh:2-3)',
    ('chain12b_20260928', 'gz_airship_vectored_independent_am_fin_drag', 'wind I on'):
        'the push was gone ~27 s in (integrator 0.0092 against 0.0185; sitl_out/chain12c.sh:2-3)',
    ('chain14_20260929', 'gz_airship_vectored_independent', 'wind weak'):
        'both weak pushes lost, the guard marked the case unmeasured ("미측정" in the line; '
        'sitl_out/chain14b.sh:2-3)',
}


def blocks(target):
    """{run: parsed verdicts} of every HEAD run of `target`."""
    out = {}

    for run in RUNS:
        text, header, logs = record_gains.RUNS[run]

        if target not in logs:
            continue

        lines = [line for line in record_gains.block_lines(os.path.join(record_gains.BENCH, text), header % target)
                 if line.startswith('  ')]
        out[run] = rp.parse(lines)

    return out


def gains_of(target):
    """(params, sources) of `target` over its HEAD runs; one set or raise."""
    found = [(run,) + record_gains.gains(run, target) for run in RUNS if target in record_gains.RUNS[run][2]]
    first = found[0][1]

    for run, params, _ in found[1:]:
        if params != first:
            raise ValueError('%s: %s flew %s, %s flew %s' % (target, found[0][0], first, run, params))

    return dict(first), [run for run, _, _ in found]


def series(run, target):
    tag = record_gains.SERIES_TAG[run]
    path = os.path.join(RECORDS_HEAD, 'samples_%s_%s.csv' % (target, tag))

    if not os.path.exists(path):
        return None

    import csv
    with open(path, encoding='utf-8') as handle:
        return list(csv.DictReader(handle))


def reproduce(say=print):
    """Run the model on every target; returns (agree, total, rows) where
    rows are (target, check, model ok, recorded ok, n runs, note)."""
    say('Records (read-only copies in records_head/):')

    for name in sorted(os.listdir(RECORDS_HEAD)):
        if name.endswith('.txt'):
            with open(os.path.join(RECORDS_HEAD, name), 'rb') as handle:
                say('  %-60s md5 %s' % (name, hashlib.md5(handle.read()).hexdigest()))

    agree = total = 0
    table = []

    for target in TARGETS:
        params, runs = gains_of(target)
        loop = cl.build(target, params=params)
        result = gate.run_gate(loop, cl.ALL_TARGETS[target][0])
        model = rp.parse(result.gate_lines)
        recs = blocks(target)
        say('\n=== %s   model status %s, %d rows; gains P %g I %g INT_LIM %g FF %g (the runs\' own: %s)'
            % (target, result.status, len(result.samples), params['AS_YAWRATE_P'], params['AS_YAWRATE_I'],
               params['AS_YR_INT_LIM'], params['AS_YAWRATE_FF'], ', '.join(runs)))
        say('  %-13s %-8s %7s  %-24s %8s %7s  %-34s %s' % ('check', 'field', 'model', 'runs (clean) min..max', 'd(model)',
                                                         'spread', 'gate tolerance (sitl_check.py)', 'flags'))

        for key, _, _, fields, tols in rp.CHECKS:
            if key not in model and not any(key in r for r in recs.values()):
                continue

            m = model.get(key, {})
            clean = {run: r[key] for run, r in recs.items() if key in r and (run, target, key) not in HARNESS}
            shown = [f for f in fields if f not in ('how', 'expected', 'needed')]

            if 'release' in key:
                shown += ['settle', 'slope']

            for f in shown:
                refs = [v.get(f) for v in clean.values() if v.get(f) is not None and math.isfinite(v.get(f))]
                mv = m.get(f)
                d = max((mv - v for v in refs), key=abs) if mv is not None and math.isfinite(mv) and refs else None
                tol_text, tol = tols.get(f, ('', None))

                if f == 'end' and key == 'wind check':
                    tol = 0.5 * m.get('expected', float('nan'))
                if f == 'integ' and tol_text.startswith('0.5 needed'):
                    tol = 0.5 * m.get('needed', float('nan'))

                spread = (max(refs) - min(refs)) if refs else None
                flags = []

                if tol is not None and d is not None and abs(d) > tol:
                    flags.append('>TOL')
                if spread is not None and d is not None and len(refs) > 1 and abs(d) > spread + 0.01:
                    flags.append('>spread')

                say('  %-13s %-8s %7s  %-24s %8s %7s  %-34s %s'
                    % (key, f, rp.fmt(mv, f), ('%s..%s' % (rp.fmt(min(refs), f), rp.fmt(max(refs), f))) if refs else '-',
                       rp.fmt(d, f), rp.fmt(spread, f), tol_text, ' '.join(flags)))

            rec_ok = {v['ok'] for v in clean.values()}
            vm = m.get('ok')
            note = ''

            if len(rec_ok) != 1:
                verdict_ok = False
                note = 'the clean runs disagree'
            else:
                verdict_ok = vm == rec_ok.pop()

            excused = [(run, recs[run][key]['ok']) for run in recs if (run, target, key) in HARNESS and key in recs[run]]
            agree += verdict_ok
            total += 1
            say('  %-13s %-8s %7s  %-24s %s%s' % (key, 'VERDICT', ('pass' if vm else 'FAIL') if vm is not None else '-',
                                                 '%d clean runs: %s' % (len(clean), '/'.join(sorted({('pass' if v['ok'] else 'FAIL') for v in clean.values()}))),
                                                 'agree' if verdict_ok else 'DISAGREE', (' (' + note + ')') if note else ''))

            for run, ok in excused:
                say('  %-13s %-8s %7s  %-24s left out: %s says %s; %s'
                    % ('', '', '', '', run, 'pass' if ok else 'FAIL', HARNESS[(run, target, key)]))

            table.append((target, key, vm, verdict_ok, len(clean)))

        for run in recs:
            rows = series(run, target)

            if rows is None:
                continue

            res = rp.series_residuals(result.samples, rows)
            rate = [r['rate'] for r in res if r['rate'] is not None]
            n = sum(r['n'] for r in res if r['rate'] is not None)
            rms = math.sqrt(sum(x[0] ** 2 * r['n'] for x, r in zip(rate, [r for r in res if r['rate'] is not None])) / n)
            worst = max(res, key=lambda r: r['rate'][1] if r['rate'] else -1)
            say('  series vs %-20s rate rms %.2f deg/s over %d rows; largest phase max |d| %.2f deg/s (%s)'
                % (run, rms, n, worst['rate'][1], worst['phase']))

    say('\nverdicts: the model agrees with the recorded verdict on %d of %d checks' % (agree, total))
    return agree, total, table


def main():
    out = os.path.join(HERE, 'reproduction_head_out.txt')
    lines = []

    def say(text=''):
        lines.append(text)
        print(text, flush=True)

    say('Reproduction of the SITL yaw gate at HEAD gains by the closed-loop model (roll plant)')
    agree, total, _ = reproduce(say)

    with open(out, 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(lines) + '\n')

    print('written', out)
    return 0 if agree == total == 48 else 1


if __name__ == '__main__':
    sys.exit(main())
