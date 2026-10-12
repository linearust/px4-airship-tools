#!/usr/bin/env python3
"""Summarize xgate jsonl: per spec x plant, all-pass count, per-check pass
count, min / median margin, undecided count, and any recomputed-vs-gate
mismatch.   python3 xsum.py FILE.jsonl [spec-filter]"""
import collections
import json
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True
import xlib  # noqa: E402

rows = [json.loads(l) for l in open(os.path.join(HERE, sys.argv[1]))]
flt = sys.argv[2] if len(sys.argv) > 2 else None
by = collections.defaultdict(list)
for r in rows:
    if flt and flt not in r['spec']:
        continue
    by[(r['spec'], r['target'])].append(r)

CHECKS = [('brake', ['m']), ('Atrack', ['m_peak', 'm_mean']), ('Arel', ['m']), ('Strack', ['m_peak', 'm_mean']),
          ('Srel', ['m']), ('wOn', ['m_rate', 'm_integ']), ('wOff', ['m_rate']), ('wWk', ['m_rate', 'm_integ'])]


def fmt(v, k):
    if v is None:
        return 'None'
    return ('%+.4f' if 'integ' in k else '%+.2f') % v


for spec in sorted({k[0] for k in by}):
    print('\n== %s' % spec)
    for t in xlib.T5:
        rs = by.get((spec, t))
        if not rs:
            continue
        n = len(rs)
        ap = sum(1 for r in rs if r['status'] == 0)
        mism = sum(1 for r in rs for k, v in r['m'].items() if isinstance(v, dict) and 'MISMATCH_gate' in v)
        other = collections.Counter()
        for r in rs:
            for k, ok in r['verdicts'].items():
                if k in ('wchk', 'man') and not ok:
                    other[k] += 1
        print('  %-12s all-pass %d/%d%s%s' % (xlib.SHORT[t], ap, n, ('  MISMATCH %d' % mism) if mism else '',
                                              ('  other fails %s' % dict(other)) if other else ''))
        for c, keys in CHECKS:
            cs = [r['m'][c] for r in rs if c in r['m']]
            if not cs:
                continue
            ok = sum(1 for x in cs if x['ok'])
            und = sum(1 for x in cs if x['und'])
            parts = []
            for k in keys:
                v = [x[k] for x in cs if x.get(k) is not None]
                nn = sum(1 for x in cs if x.get(k) is None)
                if v:
                    parts.append('%s min %s med %s%s' % (k, fmt(min(v), k), fmt(statistics.median(v), k),
                                                         (' (%d unsettled)' % nn) if nn else ''))
                else:
                    parts.append('%s none settled' % k)
            extra = ''
            if c in ('Arel', 'Srel'):
                st = [x['settle'] for x in cs if x['settle'] is not None]
                ends = [x['end'] for x in cs]
                i0 = [x['integ0'] for x in cs]
                extra = '  settle %s  end %.2f..%.2f  integ@rel %.4f..%.4f' % (
                    ('%.2f..%.2f' % (min(st), max(st))) if st else '-', min(ends), max(ends), min(i0), max(i0))
            if c == 'wOff':
                rt = [x['rate'] for x in cs]
                extra = '  drift %.3f..%.3f' % (min(rt), max(rt))
            if c == 'wOn':
                ig = [x['integ'] for x in cs]
                rt = [x['rate'] for x in cs]
                extra = '  rate %.3f..%.3f integ %.4f..%.4f' % (min(rt), max(rt), min(ig), max(ig))
            if c == 'brake':
                e = [x['end'] for x in cs]
                extra = '  end %.2f..%.2f' % (min(e), max(e))
            if c == 'Atrack':
                pk = [x['peak'] for x in cs]
                extra = '  peak %.2f..%.2f' % (min(pk), max(pk))
            print('     %-7s pass %2d/%-2d und %2d  %s%s' % (c, ok, len(cs), und, '; '.join(parts), extra))
