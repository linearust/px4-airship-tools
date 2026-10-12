#!/usr/bin/env python3
"""Compact pass-count / margin matrix from scan_full.jsonl + scan_gains.jsonl
-> matrix.txt. One block per setting, one line per plant. Each check:
pass/runs and the margin range (min..max, + inside). Units: brake, peak,
mean, wind rate, I-off in deg/s; release in s (10 - settle), or 'end x..y'
= |rate| at the window end when it never settled (1.0 needed); wind integ
as % of the needed torque. '?' marks UNDECIDED (split, or a margin inside
the model's uncertainty: rate band 0.4 vec rigid / 0.15 cld complete /
0.12 else, settle 0.6 s)."""
import collections
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True
import optc  # noqa: E402

rows = []
for kind in ('full', 'gains'):
    rows += [json.loads(line) for line in open(os.path.join(HERE, 'scan_%s.jsonl' % kind))]
by = collections.defaultdict(list)
for r in rows:
    by[(r['P'], r['I'], r['b'], r['target'])].append(r['checks'])

ORDER = [(0.5, 0.09, 1.0), (0.5, 0.09, 0.694), (0.7, 0.09, 1.0), (0.7, 0.09, 0.9), (0.7, 0.15, 1.0),
         (0.7, 0.2, 1.0), (0.7, 0.2, 0.9), (0.7, 0.2, 0.8), (0.8, 0.2, 1.0)]


def cell(cs, key, target):
    ok = [c[key]['ok'] for c in cs]
    k, n = sum(ok), len(ok)
    band = optc.rate_band(target)
    und = 0 < k < n
    if key == 'Acro brake' or key == 'wind I off':
        ms = [c[key]['m'] for c in cs]
        und |= any(abs(m) <= band for m in ms)
        txt = '%+.2f..%+.2f' % (min(ms), max(ms))
    elif key.endswith('track'):
        mp = [c[key]['m_peak'] for c in cs]
        mm = [c[key]['m_mean'] for c in cs]
        und |= any(abs(m) <= band for m in mp + mm)
        txt = 'pk%+.2f..%+.2f mn%+.2f' % (min(mp), max(mp), min(mm))
    elif key.endswith('release'):
        st = [c[key]['settle'] for c in cs]
        if all(s is not None for s in st):
            ms = [10.0 - s for s in st]
            und |= any(abs(m) <= optc.SETTLE_BAND for m in ms)
            txt = '%+.1f..%+.1fs' % (min(ms), max(ms))
        else:
            ends = [abs(c[key]['end']) for c in cs if c[key]['settle'] is None]
            txt = 'end%.2f..%.2f' % (min(ends), max(ends))
            if any(s is not None for s in st):
                txt += '(%d settled)' % sum(s is not None for s in st)
    else:                      # wind I on / weak
        mr = [c[key]['m_rate'] for c in cs]
        mi = [100 * c[key]['m_integ'] / abs(c[key]['needed']) for c in cs]
        und |= any(abs(m) <= band for m in mr)
        txt = 'r%+.2f i%+.0f%%' % (min(mr), min(mi))
    return '%d/%d %s%s' % (k, n, txt, '?' if und else '')


KEYS = ['Acro brake', 'Acro track', 'Acro release', 'Stab track', 'Stab release', 'wind I on', 'wind I off',
        'wind weak']
out = [__doc__.strip(), '']
for s in ORDER:
    out.append('P %.2f I %.2f b %.3f' % s)
    for t in optc.TARGETS:
        cs = by.get(s + (t,))
        if not cs:
            continue
        parts = ['%s %s' % (k.replace('Acro ', 'A.').replace('Stab ', 'S.').replace('wind ', 'w.'), cell(cs, k, t))
                 for k in KEYS if k in cs[0]]
        out.append('  %-18s ' % optc.SHORT[t] + ' | '.join(parts))
    out.append('')
text = '\n'.join(out)
print(text)
open(os.path.join(HERE, 'matrix.txt'), 'w').write(text + '\n')
