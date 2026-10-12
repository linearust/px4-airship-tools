#!/usr/bin/env python3
"""Summarise gain_sweep.jsonl: per target x gains, per check, how many
ensemble members pass, and the min..max of the judged number."""
import collections
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
FIELDS = {'Acro brake': 'end', 'Acro track': 'peak', 'Stab track': 'peak', 'Acro release': 'settle',
          'Stab release': 'settle', 'wind I on': 'rate', 'wind I off': 'rate', 'wind weak': 'rate',
          'wind check': 'end', 'Manual': 'end'}
MEAN = {'Acro track': 'mean', 'Stab track': 'mean', 'Acro release': 'end', 'Stab release': 'end'}

rows = [json.loads(line) for line in open(os.path.join(HERE, 'gain_sweep.jsonl'))]
by = collections.defaultdict(list)
for r in rows:
    by[(r['target'], r['P'], r['I'])].append(r)

out = []
for (target, P, I) in sorted(by):
    rs = by[(target, P, I)]
    out.append('\n%s  P %.2f I %.2f  (%d members)' % (target, P, I, len(rs)))
    checks = []
    for r in rs:
        for k in r['checks']:
            if k not in checks:
                checks.append(k)
    marg = []
    for k in checks:
        oks = [r['checks'][k]['ok'] for r in rs if k in r['checks']]
        f = FIELDS.get(k)
        vals = [r['checks'][k].get(f) for r in rs if k in r['checks'] and r['checks'][k].get(f) is not None]
        g = MEAN.get(k)
        vals2 = [r['checks'][k].get(g) for r in rs if k in r['checks'] and r['checks'][k].get(g) is not None]
        n = len(oks)
        tag = 'all pass' if all(oks) else ('ALL FAIL' if not any(oks) else 'SPLIT %d/%d' % (sum(oks), n))
        if tag.startswith('SPLIT'):
            marg.append(k)
        rng = ('%s %.2f..%.2f' % (f, min(vals), max(vals))) if vals else ('%s -' % f)
        rng2 = ('  %s %.2f..%.2f' % (g, min(vals2), max(vals2))) if vals2 else ''
        out.append('  %-13s %-12s %s%s' % (k, tag, rng, rng2))
    out.append('  => split verdicts: %s' % (', '.join(marg) or 'none'))
text = '\n'.join(out)
print(text)
open(os.path.join(HERE, 'sweep_summary.txt'), 'w').write(text + '\n')
