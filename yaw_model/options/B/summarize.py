#!/usr/bin/env python3
"""Summarise sweep jsonl files: per config x plant x check, pass count,
margin range (min..max, > 0 inside the threshold) and an UNDECIDED flag
when any member sits within the model's stated uncertainty.

    python3 summarize.py FILE.jsonl [FILE2.jsonl ...] [--compact]
"""
import collections
import json
import sys

sys.dont_write_bytecode = True
import ffcommon as fc  # noqa: E402

ORDER = ['Acro brake', 'Acro track', 'Acro release', 'Stab track', 'Stab release',
         'wind I on', 'wind I off', 'wind weak', 'wind check', 'Manual']


def fmt_range(vals, f='%.2f'):
    vals = [v for v in vals if v is not None]
    if not vals:
        return '-'
    lo, hi = min(vals), max(vals)
    return (f % lo) if abs(hi - lo) < 1e-9 else ((f + '..' + f) % (lo, hi))


def describe(key, recs):
    oks = [r['ok'] for r in recs]
    n, k = len(oks), sum(oks)
    und = any(r.get('undecided') for r in recs)
    tag = '%d/%d' % (k, n) + (' UNDECIDED' if und else '')
    if key.endswith('track'):
        det = 'peak %s (m %s) mean %s (m %s)' % (
            fmt_range([r['peak'] for r in recs]), fmt_range([r['m_peak'] for r in recs]),
            fmt_range([r['mean'] for r in recs]), fmt_range([r['m_mean'] for r in recs]))
    elif key.endswith('release'):
        st = [r['settle'] for r in recs]
        never = sum(1 for s in st if s is None)
        det = 'settle %s s%s (m %s) r0 %s end %s' % (
            fmt_range(st, '%.1f'), (' never x%d' % never) if never else '',
            fmt_range([10.0 - s for s in st if s is not None], '%.1f'),
            fmt_range([r['r0'] for r in recs]), fmt_range([r['end'] for r in recs]))
    elif key == 'Acro brake':
        det = 'end %s (m %s)' % (fmt_range([r['end'] for r in recs]), fmt_range([r['margin'] for r in recs]))
    elif key in ('wind I on', 'wind weak'):
        det = 'rate %s (m %s) integ %s need %.4f (m %s)' % (
            fmt_range([r['rate'] for r in recs]), fmt_range([r['m_rate'] for r in recs]),
            fmt_range([r['integ'] for r in recs], '%.4f'), recs[0]['needed'],
            fmt_range([r['m_integ'] for r in recs], '%.4f'))
    elif key == 'wind I off':
        det = 'rate %s (m %s)' % (fmt_range([r['rate'] for r in recs]), fmt_range([r['margin'] for r in recs]))
    else:
        det = ''
    return tag, det


def main():
    files = [a for a in sys.argv[1:] if not a.startswith('--')]
    compact = '--compact' in sys.argv
    rows = []
    for f in files:
        rows += [json.loads(l) for l in open(f)]
    by = collections.defaultdict(list)
    for r in rows:
        by[(r['P'], r['I'], r['FF'], json.dumps(r.get('act', {}), sort_keys=True), r['target'])].append(r)
    last = None
    for key in sorted(by, key=lambda k: (k[0], k[1], k[2], k[3], fc.TARGETS5.index(k[4]))):
        P, I, FF, act, target = key
        rs = by[key]
        errs = [r for r in rs if r['err']]
        head = 'P %.2f I %.3f FF %.2f %s' % (P, I, FF, act if act != '{}' else '')
        if head != last:
            print('\n=== ' + head)
            last = head
        fails = []
        lines = []
        for c in ORDER:
            recs = [r['checks'][c] for r in rs if c in r.get('checks', {})]
            if not recs:
                continue
            tag, det = describe(c, recs)
            if not all(r['ok'] for r in recs) or 'UNDECIDED' in tag:
                fails.append('%s %s' % (c, tag))
            lines.append('    %-13s %-16s %s' % (c, tag, det))
        print('  %-13s (%d runs%s)  %s' % (fc.SHORT[target], len(rs), (', %d ERR' % len(errs)) if errs else '',
                                          ('NOT CLEAN: ' + '; '.join(fails)) if fails else 'all pass, decided'))
        if not compact:
            print('\n'.join(lines))
        for e in errs[:1]:
            print('    ERR', e['err'])


if __name__ == '__main__':
    main()
