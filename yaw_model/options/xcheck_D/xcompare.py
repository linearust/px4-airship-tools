#!/usr/bin/env python3
"""1) My gate runs vs option D's ensemble.jsonl, member by member (same
target, timing label, seed): max |difference| of each judged number.
2) restore vs none (both mine, full), paired per member: the change in every
margin, to test 'every other check stays within 0.05 of baseline'."""
import collections
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True
import xlib  # noqa: E402

mine = [json.loads(l) for l in open(os.path.join(HERE, 'gate_full.jsonl'))]
theirs = [json.loads(l) for l in open(os.path.join(os.path.dirname(HERE), 'D', 'ensemble.jsonl'))]
NAME = {'restore:0.7:full': ('restore', 'P0.7'), 'none:0.7:full': ('none', 'P0.7'), 'none:0.7:lite': ('none', 'P0.7'), 'reset:0.7:full': ('reset', 'P0.7')}
MAP = [('brake', 'end', 'Acro brake', 'end'), ('Atrack', 'peak', 'Acro track', 'peak'),
       ('Atrack', 'mean', 'Acro track', 'mean'), ('Strack', 'peak', 'Stab track', 'peak'),
       ('Arel', 'settle', 'Acro release', 'settle'), ('Srel', 'settle', 'Stab release', 'settle'),
       ('Arel', 'end', 'Acro release', 'end'), ('wOn', 'rate', 'wind I on', 'rate'),
       ('wOn', 'integ', 'wind I on', 'integ'), ('wOff', 'rate', 'wind I off', 'rate'),
       ('wWk', 'integ', 'wind weak', 'integ')]
tidx = {(r['name'], r['gains'], r['target'], r['member'], r['seed']): r for r in theirs}
print('== 1) mine vs option D ensemble.jsonl, matched members')
stats = collections.defaultdict(lambda: [0, 0.0, 0])
for r in mine:
    if r['spec'] not in NAME:
        continue
    key = NAME[r['spec']] + (r['target'], r['member'], r['seed'])
    o = tidx.get(key)
    if not o:
        continue
    s = stats[r['spec']]
    s[0] += 1
    if (r['status'] == 0) != (o['status'] == 0):
        s[2] += 1
    for mk, mf, ok_, of in MAP:
        a = r['m'].get(mk, {}).get(mf)
        b = o['checks'].get(ok_, {}).get(of)
        if a is None and b is None:
            continue
        if (a is None) != (b is None):
            s[1] = float('inf')
            continue
        s[1] = max(s[1], abs(a - b))
for k, (n, d, st) in stats.items():
    print('  %-18s %3d matched members, max |diff| over all judged numbers %.3g, status disagreements %d' % (k, n, d, st))

print('\n== 2) restore - none, paired (mine, full ensemble): change of each margin, min / max over members')
idx = {(r['spec'], r['target'], r['member'], r['seed']): r for r in mine}
KEYS = [('brake', 'm'), ('Atrack', 'm_peak'), ('Atrack', 'm_mean'), ('Strack', 'm_peak'), ('Strack', 'm_mean'),
        ('Arel', 'm'), ('Srel', 'm'), ('wOn', 'm_rate'), ('wOn', 'm_integ'), ('wOff', 'm_rate'),
        ('wWk', 'm_rate'), ('wWk', 'm_integ')]
for t in xlib.T5:
    d = collections.defaultdict(list)
    for (spec, tt, mem, seed), r in idx.items():
        if spec != 'restore:0.7:full' or tt != t:
            continue
        b = idx.get(('none:0.7:full', t, mem, seed)) or idx.get(('none:0.7:lite', t, mem, seed))
        if not b:
            continue
        for c, f in KEYS:
            x, y = r['m'].get(c, {}).get(f), b['m'].get(c, {}).get(f)
            if x is None or y is None:
                d[(c, f)].append(None)
            else:
                d[(c, f)].append(x - y)
    if not d:
        continue
    parts = []
    for (c, f), v in d.items():
        vv = [x for x in v if x is not None]
        nn = len(v) - len(vv)
        if vv:
            fm = '%+.4f..%+.4f' if 'integ' in f else '%+.2f..%+.2f'
            parts.append('%s.%s %s%s' % (c, f, fm % (min(vv), max(vv)), (' [%d n/a]' % nn) if nn else ''))
        else:
            parts.append('%s.%s n/a' % (c, f))
    print('  %-12s (%d pairs)\n     %s' % (xlib.SHORT[t], len(next(iter(d.values()))), '\n     '.join(parts)))
