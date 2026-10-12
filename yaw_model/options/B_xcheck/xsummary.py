#!/usr/bin/env python3
"""Tables from the cross-check jsonl files.

    python3 xsummary.py FILE.jsonl [FILE2 ...]
Groups by (target, P, I, FF, act); per check: gate pass count, the judged numbers'
ranges, the margin range (> 0 inside), and UNDECIDED under the stated uncertainty
(UNC) and under the looser ALT_UNC (0.3 on the vectored complete / fin plants).
"""
import collections
import json
import sys

sys.dont_write_bytecode = True
import xcommon as xc  # noqa: E402


def rng(vals, f='%.2f'):
    vals = [v for v in vals if v is not None]
    if not vals:
        return '-'
    lo, hi = min(vals), max(vals)
    return (f % lo) if abs(hi - lo) < 1e-9 else (f + '..' + f) % (lo, hi)


def line(key, recs, target):
    n = len(recs)
    k = sum(1 for r in recs if r['ok'])
    und = any(xc.undecided(key, r, target, 'unc') for r in recs)
    und_alt = any(xc.undecided(key, r, target, 'alt') for r in recs)
    tag = '%d/%d' % (k, n) + (' UNDECIDED' if und else (' undecided@alt' if und_alt else ''))
    if key == 'brake':
        det = 'end %s m %s' % (rng([r['end'] for r in recs]), rng([r['m'] for r in recs]))
    elif key.endswith('track'):
        det = 'peak %s (m %s) mean %s (m %s)' % (rng([r['peak'] for r in recs]), rng([r['m_peak'] for r in recs]),
                                                 rng([r['mean'] for r in recs]), rng([r['m_mean'] for r in recs]))
    elif key.endswith('release'):
        st = [r['settle'] for r in recs]
        never = sum(1 for s in st if s is None)
        det = 'settle %s s%s r0 %s end %s | thr-u %s thr+u %s' % (
            rng(st, '%.2f'), (' never x%d' % never) if never else '', rng([r['r0'] for r in recs]),
            rng([r['end'] for r in recs]),
            rng([r['st_lo']['unc'] for r in recs], '%.2f') + ('(never x%d)' % sum(1 for r in recs if r['st_lo']['unc'] is None) if any(r['st_lo']['unc'] is None for r in recs) else ''),
            rng([r['st_hi']['unc'] for r in recs], '%.2f'))
    elif key == 'wind I off':
        det = 'rate %s (m %s)' % (rng([r['rate'] for r in recs]), rng([r['m'] for r in recs]))
    elif key in ('wind I on', 'wind weak'):
        det = 'rate %s (m %s) integ %s need %.4f (m_integ %s)' % (
            rng([r['rate'] for r in recs]), rng([r['m'] for r in recs]), rng([r['integ'] for r in recs], '%.4f'),
            recs[0]['needed'], rng([r['m_integ'] for r in recs], '%.4f'))
    else:
        det = ''
    return '    %-13s %-18s %s' % (key, tag, det)


def main():
    rows = []
    for f in sys.argv[1:]:
        rows += [json.loads(l) for l in open(f)]
    errs = [r for r in rows if r['err']]
    for e in errs[:3]:
        print('ERR', e['target'], e['err'][-400:])
    by = collections.defaultdict(list)
    for r in rows:
        if not r['err']:
            by[(r['target'], r['P'], r['I'], r['FF'], json.dumps(r['act'], sort_keys=True))].append(r)
    for key in sorted(by, key=lambda k: (k[4], xc.PLANTS.index(k[0]), k[1], k[2], k[3])):
        t, P, I, FF, act = key
        rs = by[key]
        members = sorted(set(r['member'] for r in rs))
        print('\n%-13s P %.2f I %.2f FF %.2f %s  (%d runs, %d timing members, seeds %s)'
              % (xc.NAME[t], P, I, FF, act if act != '{}' else '', len(rs), len(members),
                 ','.join(str(s) for s in sorted(set(r['seed'] for r in rs)))))
        for c in xc.ORDER:
            recs = [r['checks'][c] for r in rs if c in r['checks']]
            if recs:
                print(line(c, recs, t))


if __name__ == '__main__':
    main()
