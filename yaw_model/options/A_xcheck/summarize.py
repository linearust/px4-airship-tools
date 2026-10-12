#!/usr/bin/env python3
"""Summaries of the cross-check runs.

    python3 summarize.py main.jsonl [--seeds 11-15] [--act kind:value] [--set P,I,L]

Per set x plant x check: pass count, worst / median / best margin per field
(full precision, gate units), and the worst margin over the band, with two
band tables: option A's, and an alternative where the rigid vectored RELEASE
also takes the 0.4 of that plant's ripple (the same instant-reading exposure
as its peak and brake).
"""
import collections
import json
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True
import xgate  # noqa: E402

CHECKS = ['Acro brake', 'Acro track', 'Acro release', 'Stab track', 'Stab release', 'wind I on', 'wind I off',
          'wind weak']
FIELDS = {'Acro brake': ['end'], 'Acro track': ['mean', 'peak'], 'Stab track': ['mean', 'peak'],
          'Acro release': ['release'], 'Stab release': ['release'], 'wind I on': ['rate', 'integ'],
          'wind I off': ['rate'], 'wind weak': ['rate', 'integ']}


def band(target, check, field, alt=False):
    if field == 'integ':
        return 0.001
    if target == 'gz_airship_vectored_independent':
        if check == 'Acro brake' or field == 'peak':
            return 0.4
        if alt and 'release' in check:
            return 0.4
    return 0.15


def seeds_of(s):
    out = []
    for part in s.split(','):
        if '-' in part:
            a, b = part.split('-')
            out += list(range(int(a), int(b) + 1))
        else:
            out.append(int(part))
    return set(out)


def load(path, seeds=None, act=None, pset=None):
    by = collections.defaultdict(list)
    for line in open(os.path.join(HERE, path)):
        r = json.loads(line)
        if seeds and r['seed'] not in seeds:
            continue
        if (r['act'] or None) != act:
            continue
        p = r['params']
        s = (p['AS_YAWRATE_P'], p['AS_YAWRATE_I'], p['AS_YR_INT_LIM'])
        if pset and s != pset:
            continue
        by[(s, r['target'])].append(r)
    return by


def summarize(recs, alt=False):
    out = {}
    t = recs[0]['target']
    for k in CHECKS:
        rs = [r for r in recs if k in r['ok']]
        if not rs:
            continue
        n = len(rs)
        npass = sum(bool(r['ok'][k]) for r in rs)
        fields = {}
        for f in FIELDS[k]:
            vals = [r['m'].get(k, {}).get(f) for r in rs]
            vals = [v for v in vals if v is not None]
            if vals:
                fields[f] = (min(vals), statistics.median(vals), max(vals))
        per = []
        for r in rs:
            mm = [r['m'][k][f] / band(t, k, f, alt) for f in FIELDS[k] if r['m'].get(k, {}).get(f) is not None]
            if mm:
                per.append(min(mm))
        worst, best = min(per), max(per)
        if npass == n and worst >= 1.0:
            v = 'PASS'
        elif npass == 0 and best <= -1.0:
            v = 'FAIL'
        else:
            v = 'UNDECIDED'
        out[k] = dict(n=n, npass=npass, fields=fields, worst=worst, best=best, verdict=v)
    return out


def main():
    args = sys.argv[1:]
    path = args[0]
    seeds = act = pset = None
    alt = '--alt' in args
    if '--seeds' in args:
        seeds = seeds_of(args[args.index('--seeds') + 1])
    if '--act' in args:
        k, v = args[args.index('--act') + 1].split(':')
        act = [k, float(v)]
    if '--set' in args:
        pset = tuple(float(x) for x in args[args.index('--set') + 1].split(','))
    by = load(path, seeds, act, pset)
    for s in sorted({k[0] for k in by}):
        print('\n##### P %.3f I %.3f INT_LIM %.2f  act %s  seeds %s  band table %s'
              % (s + (act, sorted(seeds) if seeds else 'all', 'alt (rigid release 0.4)' if alt else 'option A')))
        for t in xgate.TARGETS:
            recs = by.get((s, t))
            if not recs:
                continue
            im = max(r['integ_true_max'] for r in recs)
            print('  --- %s: %d runs; integrator true max %.4f' % (xgate.SHORT[t], len(recs), im))
            for k, v in summarize(recs, alt).items():
                fs = '; '.join('%s %+.3f/%+.3f/%+.3f' % (f, a, b, c) for f, (a, b, c) in v['fields'].items())
                print('    %-13s %3d/%-3d %-9s worst %+6.2f band | min/med/max %s'
                      % (k, v['npass'], v['n'], v['verdict'], v['worst'], fs))


if __name__ == '__main__':
    main()
