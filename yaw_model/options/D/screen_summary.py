#!/usr/bin/env python3
import collections, json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.dont_write_bytecode = True
import option_d as od
from screen import RULES
rows = [json.loads(l) for l in open(os.path.join(HERE, sys.argv[1] if len(sys.argv) > 1 else 'screen.jsonl'))]
by = {(r['name'], r['gains'], r['target']): r for r in rows}
ORDER = ['Acro brake', 'Acro track', 'Acro release', 'Stab track', 'Stab release', 'wind I on', 'wind I off', 'wind weak']
AB = {'Acro brake': 'brk', 'Acro track': 'aT', 'Acro release': 'aR', 'Stab track': 'sT', 'Stab release': 'sR',
      'wind I on': 'wOn', 'wind I off': 'wOff', 'wind weak': 'wWk'}
lines = []
for g in od.GAINS:
    lines.append('\n=== gains %s  (per plant: failing checks; * = undecided; release settle s; wind-on rate/integ)' % g)
    for name, _ in RULES:
        cells = []
        for t in od.TARGETS5:
            r = by.get((name, g, t))
            if not r:
                cells.append('%-34s' % '-'); continue
            c = r['checks']
            fails = [AB[k] + ('*' if c[k]['undecided'] else '') for k in ORDER if k in c and not c[k]['ok']]
            und = [AB[k] + '*' for k in ORDER if k in c and c[k]['ok'] and c[k]['undecided']]
            aR = c.get('Acro release', {}); sR = c.get('Stab release', {})
            fmt = lambda x: ('%.1f' % x['settle']) if x.get('settle') is not None else 'end%.1f' % x.get('end', float('nan'))
            wo = c.get('wind I on', {})
            cells.append('%-34s' % ('%s|%s R%s/%s w%.2f/%.4f' % (','.join(fails) or 'ok', ','.join(und), fmt(aR), fmt(sR),
                                                                  wo.get('rate', float('nan')), wo.get('integ', float('nan')))))
        lines.append('%-12s %s' % (name, ' '.join(cells)))
lines.insert(0, '%-12s %s' % ('', ' '.join('%-34s' % od.SHORT[t] for t in od.TARGETS5)))
text = '\n'.join(lines)
print(text)
