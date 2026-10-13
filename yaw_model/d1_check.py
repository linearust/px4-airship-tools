#!/usr/bin/env python3
"""Increment-1 step 2 (d1_increment1_plan_20261007.md): the corrected schedule
(policies.schedule_d1, budget 'd1') on the model before any firmware.
d1_runs_20261007.jsonl is the increment-1 firmware's (ramp from AS_SPD_LO =
1.5); a rerun today models HEAD's ramp (policies.RAMP_START x AS_SPD_HI).

    python3 d1_check.py            # run what is missing (8 workers), then the report
    python3 d1_check.py --report   # the report from d1_runs_20261007.jsonl only

Two parts:
  regression  HEAD, D1, D1+RESTORE, and the 0c variants SCHED+RESTORE and
              SCHED_CB for reference, on 2527 and 2522 at 2.5 / 4 / 6 m/s,
              full and half stick for 20 s, full for 300 s, with the sources
              perfect / ground_W0 / ground_W2_head / pitot_bias_ms and the FF
              errors k x0.7 and x2 (perfect source). The question: does the
              corrected schedule keep 0c's result (D1+RESTORE releases at most
              2.5 deg in all and its 120 s excursion stays near 3 deg on the
              vectored plants), and what caps does it give.
  acceptance  the SITL predictions for step 1c: 2527, 2522 (k from the fins)
              and 2512 (k the measured steady-turn slope, 0c's khold) at 2 and
              4 m/s full stick and 4 m/s half stick, D1+RESTORE with the posix
              values (ground speed, W 0), from straight flight at the speed as
              forward_yaw_check flies it.

Reuses policy_compare's leg runner and summary (the same metrics as 0c), so
the records are comparable line for line with step0c_runs_20261001.jsonl.
"""
import argparse
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import policy_compare as PC      # noqa: E402
import policies as P             # noqa: E402
import controller as _ctl  # noqa: E402
# the SITL records this check compares with were flown before 2026-10-12, on the linear feedforward law
_ctl.FF_LAW = 'linear'

RUNS = os.path.join(HERE, 'd1_runs_20261007.jsonl')
OUT = os.path.join(HERE, 'd1_check_out_20261007.txt')

REG_PLANTS = ('2527', '2522')
REG_SPEEDS = (2.5, 4.0, 6.0)
REG_CASES = ((1.0, 20.0), (0.5, 20.0), (1.0, 300.0))
REG_POLICIES = ('HEAD', 'D1', 'D1+RESTORE', 'SCHED+RESTORE', 'SCHED_CB')
REG_SOURCES = ('perfect', 'ground_W0', 'ground_W2_head', 'pitot_bias_ms')
REG_KF = (0.7, 2.0)
ACC_LEGS = ((2.0, 1.0), (4.0, 1.0), (4.0, 0.5))
KHOLD_2512 = 12.738          # step0c_out_20261001.txt, "The plants' steady-turn torque per rad/s": cloudship_fin


def spec(part, config, policy, source, u0, stick, hold, k_factor=1.0, k_override=None, held=False):
    s = {'part': part, 'group': 'd1', 'config': config, 'policy': policy, 'source': source, 'u0': u0,
         'stick': stick, 'hold': hold, 'release': 120.0, 'k_factor': k_factor, 'seed': 0, 'held': held}
    if k_override is not None:
        s['k_override'] = k_override
    s['id'] = '|'.join(str(s[k]) for k in ('part', 'config', 'policy', 'source', 'u0', 'stick', 'hold',
                                         'k_factor', 'held')) + ('|k%s' % k_override if k_override else '')
    return s


def specs():
    out = []
    for c in REG_PLANTS:
        glide = c == '2522'
        for u in REG_SPEEDS:
            for stick, hold in REG_CASES:
                held = glide and hold > 100.0          # 0c: the gliders' 300 s holds are flown at the label
                for pol in REG_POLICIES:
                    if pol in ('HEAD',):
                        out.append(spec('reg', c, pol, None, u, stick, hold, held=held))
                        continue
                    for src in REG_SOURCES:
                        out.append(spec('reg', c, pol, src, u, stick, hold, held=held))
                    for kf in REG_KF:
                        out.append(spec('reg', c, pol, 'perfect', u, stick, hold, k_factor=kf, held=held))
    for c, kov in (('2527', None), ('2522', None), ('2512/khold', KHOLD_2512)):
        for u, stick in ACC_LEGS:
            for pol in ('HEAD', 'D1+RESTORE'):
                out.append(spec('acc', c, pol, None if pol == 'HEAD' else 'ground_W0', u, stick, 20.0, k_override=kov))
    return out


def run(s):
    try:
        leg, hook, cfg = PC._leg(s)
        rec = PC.summarize(leg, hook, s, cfg)
        rec.update({'id': s['id'], 'spec': s})
        return rec
    except Exception as exc:          # recorded, never silently dropped
        return {'id': s['id'], 'spec': s, 'error': '%s: %s' % (type(exc).__name__, exc)}


def load():
    recs = {}
    if os.path.exists(RUNS):
        for line in open(RUNS):
            r = json.loads(line)
            recs[r['id']] = r
    return recs


def f(x, fmt='%+.1f'):
    return '-' if x is None else fmt % x


def report(recs):
    lines = []
    say = lines.append
    errs = [r for r in recs.values() if 'error' in r]
    say('Increment-1 step 2, 2026-10-07: the corrected schedule (policies.schedule_d1) on the model (d1_check.py)')
    say('%d records, %d errors' % (len(recs), len(errs)))
    for r in errs:
        say('  ERROR %s: %s' % (r['id'], r['error']))
    say('')
    say('=== Regression: 2527 and 2522, release drift in all (deg) and the 120 s excursion (deg), D1 vs 0c ===')
    say('drift "in all" = heading change over 120 s + I(120 s)/K_I (0c\'s "to rest"); "-" where 0c\'s validity test fails')
    worst = {}
    for c in REG_PLANTS:
        for u in REG_SPEEDS:
            for stick, hold in REG_CASES:
                say('--- %s %.1f m/s stick %+.1f %g s' % (c, u, stick, hold))
                head = recs.get(spec('reg', c, 'HEAD', None, u, stick, hold, held=(c == '2522' and hold > 100))['id'])
                say('  HEAD: r last 5 s %s, I at release %s, release 120 s %s, in all %s'
                    % (f(head and head.get('r_last5'), '%.2f'), f(head and head.get('I_rel'), '%+.3f'),
                       f(head and head.get('drift_120')), f(head and head.get('drift_rest'))))
                for pol in REG_POLICIES[1:]:
                    cells = []
                    for src in REG_SOURCES:
                        cells.append((src, 1.0))
                    for kf in REG_KF:
                        cells.append(('perfect', kf))
                    row = []
                    for src, kf in cells:
                        r = recs.get(spec('reg', c, pol, src, u, stick, hold, k_factor=kf,
                                          held=(c == '2522' and hold > 100))['id'])
                        if not r or 'error' in r:
                            row.append('%s/x%g ?' % (src, kf))
                            continue
                        exc = max(abs(r.get('rel_h_max') or 0.0), abs(r.get('rel_h_min') or 0.0))
                        row.append('%s%s: cap %.2f I %+.3f all %s exc %.1f' % (src, '' if kf == 1.0 else '/k x%g' % kf,
                                   r['cap_end'], r['I_rel'], f(r.get('drift_rest')), exc))
                        key = (pol, src, kf)
                        prev = worst.get(key)
                        val = abs(r['drift_rest']) if r.get('drift_rest') is not None else None
                        if val is not None and (prev is None or val > prev[0]):
                            worst[key] = (val, '%s %.1f m/s %+.1f %g s' % (c, u, stick, hold))
                    say('  %-14s %s' % (pol, ' | '.join(row)))
    say('')
    say('=== Worst |release in all| per policy and source (deg) ===')
    for key in sorted(worst):
        say('  %-14s %-15s k x%-4g %6.1f  at %s' % (key[0], key[1], key[2], worst[key][0], worst[key][1]))
    say('')
    say('=== Acceptance table for SITL step 1c (D1+RESTORE, ground speed W 0, from straight flight) ===')
    say('plant u stick | HEAD: r5 I_rel rel120 | D1+RESTORE: cap ff r5 I_rel rel120 r+10/+60/+120 exc stall/fin')
    for c, kov in (('2527', None), ('2522', None), ('2512/khold', KHOLD_2512)):
        for u, stick in ACC_LEGS:
            h = recs.get(spec('acc', c, 'HEAD', None, u, stick, 20.0, k_override=kov)['id'])
            d = recs.get(spec('acc', c, 'D1+RESTORE', 'ground_W0', u, stick, 20.0, k_override=kov)['id'])
            if not h or not d or 'error' in h or 'error' in d:
                say('  %s %.1f %+.1f: missing or error' % (c, u, stick))
                continue
            exc = max(abs(d.get('rel_h_max') or 0.0), abs(d.get('rel_h_min') or 0.0))
            say('  %-10s %.1f %+.1f | %.2f %+.3f %+.1f | cap %.2f ff %.2f r5 %.2f I %+.3f rel %+.1f r %s/%s/%s exc %.1f fin %s'
                % (c, u, stick, h['r_last5'], h['I_rel'], h['drift_120'], d['cap_end'], d['ff_end'], d['r_last5'],
                   d['I_rel'], d['drift_120'], f(d['r_10'], '%+.2f'), f(d['r_60'], '%+.2f'), f(d['r_120'], '%+.2f'),
                   exc, f(d.get('fin_stick'), '%.2f')))
    text = '\n'.join(lines) + '\n'
    open(OUT, 'w').write(text)
    return text


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--report', action='store_true')
    ap.add_argument('--jobs', type=int, default=8)
    a = ap.parse_args()
    recs = load()
    if not a.report:
        todo = [s for s in specs() if s['id'] not in recs]
        print('%d runs to do (%d on disk)' % (len(todo), len(recs)), flush=True)
        with open(RUNS, 'a') as out, ProcessPoolExecutor(a.jobs) as ex:
            for i, rec in enumerate(ex.map(run, todo, chunksize=1)):
                out.write(json.dumps(rec) + '\n')
                out.flush()
                recs[rec['id']] = rec
                if (i + 1) % 25 == 0:
                    print('  %d/%d' % (i + 1, len(todo)), flush=True)
    print(report(recs))


if __name__ == '__main__':
    main()
