#!/usr/bin/env python3
"""FF window table from screen jsonl: per (P, I) and plant, which FF values pass
every check (noise-free nominal screen), with the failing check named."""
import collections, json, sys
sys.dont_write_bytecode = True
import ffcommon as fc
rows = []
for p in sys.argv[1:]:
    rows += [json.loads(l) for l in open(p)]
by = collections.defaultdict(dict)
for r in rows:
    if r.get('act'):
        continue
    by[(r['P'], r['I'])].setdefault(r['target'], {})[r['FF']] = r
ABBR = {'Acro brake': 'brk', 'Acro track': 'Atr', 'Acro release': 'Arel', 'Stab track': 'Str',
        'Stab release': 'Srel', 'wind I on': 'wOn', 'wind I off': 'wOff', 'wind weak': 'wWk',
        'wind check': 'wChk', 'Manual': 'Man'}
for (P, I) in sorted(by):
    print('\nP %.2f I %.3f' % (P, I))
    for t in fc.TARGETS5:
        d = by[(P, I)].get(t, {})
        cells = []
        for ff in sorted(d):
            c = d[ff]['checks']
            bad = [ABBR[k] for k, v in c.items() if not v['ok']]
            und = [ABBR[k] for k, v in c.items() if v['ok'] and v.get('undecided')]
            s = 'ok' if not bad else 'X:' + '+'.join(bad)
            if und:
                s += '(?' + '+'.join(und) + ')'
            cells.append('FF%.2f %s' % (ff, s))
        print('  %-12s %s' % (fc.SHORT[t], ' | '.join(cells)))
