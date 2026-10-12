#!/usr/bin/env python3
"""Pass counts and margins per plant x band x gains from eval_gate.jsonl.

    python3 summarize.py [eval_gate.jsonl]

Margin = how far inside (+) or outside (-) the gate's threshold the judged
number is, in the number's own unit (sitl_check.py lines in reproduce.CHECKS).
A check is UNDECIDED when the ensemble splits, or when the worst member's
margin is within the model's stated uncertainty of the threshold:
  rate thresholds [deg/s]: rigid vectored peak and brake 0.4 (review: the
    rigid vectored peak and brake can be off by 0.3-0.4); Cloudship complete
    every rate 0.15 (pitch and heave held); elsewhere 0.12 (the noise-free
    peak bias, kept as a floor although noise is on)
  release settle time [s]: one gate row, 0.32
  wind integrator: 10 % of the needed torque
  Manual torque: 0.002
"""
import collections
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SP = (0.5 - 0.1) / 0.9 * 15.0
ORDER = ['wind check', 'Manual', 'Acro brake', 'Acro track', 'Acro release', 'Stab track', 'Stab release',
         'wind I on', 'wind I off', 'wind weak']
TARGET_ORDER = ['gz_airship_vectored_independent', 'gz_airship_vectored_independent_am_fin_drag',
                'gz_airship_vectored_independent_fin', 'gz_cloudship', 'gz_cloudship_am_fin_drag']
SHORT = {'gz_airship_vectored_independent': 'vectored rigid',
         'gz_airship_vectored_independent_am_fin_drag': 'vectored complete',
         'gz_airship_vectored_independent_fin': 'vectored _fin (2nd case)',
         'gz_cloudship': 'Cloudship rigid', 'gz_cloudship_am_fin_drag': 'Cloudship complete'}


def rate_u(target, what):
    if target == 'gz_airship_vectored_independent' and what in ('peak', 'brake'):
        return 0.4
    if target == 'gz_cloudship_am_fin_drag':
        return 0.15
    return 0.12


def margins(key, rec, target):
    """[(name, margin, uncertainty)] for one check record."""
    g = rec.get
    if key == 'wind check':
        if g('end') is None or g('start') is None:
            return [('d', None, 0.12)]
        return [('|d|-exp/2', abs(g('end') - g('start')) - 0.5 * g('expected'), rate_u(target, 'rate'))]
    if key == 'Manual':
        return [('torque', 0.02 - abs(g('torque') - 0.5), 0.002)]
    if key == 'Acro brake':
        return [('end', 6.0 - abs(g('end')), rate_u(target, 'brake'))]
    if key in ('Acro track', 'Stab track'):
        return [('mean', 0.25 * SP - abs(g('mean') - SP), rate_u(target, 'mean')),
                ('peak', 1.4 * SP - g('peak'), rate_u(target, 'peak'))]
    if key in ('Acro release', 'Stab release'):
        s = g('settle')
        return [('settle', None if s is None else 10.0 - s, 0.32)]
    if key in ('wind I on', 'wind weak'):
        n = g('needed')
        return [('rate', 1.0 - g('rate'), rate_u(target, 'rate')),
                ('integ', 0.5 * n - abs(g('integ') - n), 0.1 * n)]
    if key == 'wind I off':
        return [('rate', g('rate') - 1.0, rate_u(target, 'rate'))]
    return []


def load(path):
    rows = []
    with open(path) as fh:
        for line in fh:
            rows.append(json.loads(line))
    return rows


def summarize(rows, out=sys.stdout, bands=None, gains=None):
    groups = collections.defaultdict(list)
    for r in rows:
        groups[(r['target'], r['band'], r['gains'])].append(r)
    table = {}
    for (target, band, gname), rs in sorted(groups.items(), key=lambda kv: (TARGET_ORDER.index(kv[0][0]),
                                                                          kv[0][1], kv[0][2])):
        if bands and band not in bands:
            continue
        if gains and gname not in gains:
            continue
        ens = [r for r in rs if r['seed'] is not None]
        nf = [r for r in rs if r['seed'] is None]
        P, I = rs[0]['P'], rs[0]['I']
        print('\n%s [%s]  band %s  gains %s (P %.2f I %.2f)  %d noisy members + %d noise-free'
              % (SHORT[target], target, band, gname, P, I, len(ens), len(nf)), file=out)
        sw = [r['swings'] for r in ens if r['swings'] is not None]
        if sw:
            print('  pod target swings (>90 deg) per gate run: %d..%d' % (min(sw), max(sw)), file=out)
        summary = {}
        for key in ORDER:
            recs = [r['checks'].get(key) for r in ens]
            if all(x is None for x in recs):
                continue
            present = [x for x in recs if x is not None]
            npass = sum(1 for x in present if x['ok'])
            ms = collections.defaultdict(list)
            us = {}
            for x in present:
                for name, m, u in margins(key, x, target):
                    ms[name].append(m)
                    us[name] = u
            nfm = {}
            if nf and nf[0]['checks'].get(key):
                for name, m, u in margins(key, nf[0]['checks'][key], target):
                    nfm[name] = m
            parts = []
            undecided = 0 < npass < len(present)
            for name, vals in ms.items():
                good = [v for v in vals if v is not None]
                if good:
                    lo, hi = min(good), max(good)
                    worst = lo if len(good) == len(vals) else None
                    s = '%s %+.3f..%+.3f' % (name, lo, hi)
                    if len(good) < len(vals):
                        s += ' (%d/%d never)' % (len(vals) - len(good), len(vals))
                    if worst is not None and abs(worst) < us[name]:
                        undecided = True
                        s += ' [<%.2g]' % us[name]
                    if len(good) < len(vals) and good and max(good) > 0 and npass == len(present):
                        pass
                else:
                    s = '%s never' % name
                if name in nfm:
                    s += ' (nf %s)' % ('never' if nfm[name] is None else '%+.3f' % nfm[name])
                parts.append(s)
            extra = ''
            if key in ('Acro release', 'Stab release'):
                ends = [x.get('end') for x in present if x.get('end') is not None]
                if ends:
                    extra = '  end %.1f..%.1f deg/s' % (min(ends), max(ends))
            flag = 'UNDECIDED' if undecided else ('pass' if npass == len(present) else 'FAIL')
            summary[key] = (npass, len(present), flag)
            print('  %-13s %2d/%-2d %-9s %s%s' % (key, npass, len(present), flag, '; '.join(parts), extra),
                  file=out)
        table[(target, band, gname)] = summary
    return table


if __name__ == '__main__':
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, 'eval_gate.jsonl')
    rows = load(path)
    if all(r['seed'] is None for r in rows):
        for r in rows:          # screen: treat the noise-free run as the ensemble
            r['seed'] = 0
    summarize(rows)
