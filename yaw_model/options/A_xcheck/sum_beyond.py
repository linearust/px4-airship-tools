#!/usr/bin/env python3
"""Tables from beyond.jsonl: current (0.5/0.09) vs recommended (0.7/0.2)."""
import collections, json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.dont_write_bytecode = True
import xgate
by = collections.defaultdict(list)
for l in open(os.path.join(HERE, 'beyond.jsonl')):
    r = json.loads(l)
    s = (r['params']['AS_YAWRATE_P'], r['params']['AS_YAWRATE_I'])
    by[(r['kind'], r['target'], json.dumps(r['act']), s)].append(r['res'])
def fmt(v):
    if v is None: return '-'
    if isinstance(v, float): return '%.3g' % v if abs(v) < 0.01 else '%.2f' % v
    return str(v)
kinds = sorted({k[0] for k in by}); acts = sorted({k[2] for k in by})
for kind in kinds:
    for act in acts:
        rows = [(t, by.get((kind, t, act, (0.5, 0.09))), by.get((kind, t, act, (0.7, 0.2)))) for t in xgate.TARGETS]
        rows = [r for r in rows if r[1] or r[2]]
        if not rows: continue
        print('\n== %s  act %s   (current 0.5/0.09 -> recommended 0.7/0.2; seeds joined by |)' % (kind, act))
        for t, a, b in rows:
            fields = list((a or b)[0].keys())
            parts = []
            for f in fields:
                va = '|'.join(fmt(x.get(f)) for x in (a or []))
                vb = '|'.join(fmt(x.get(f)) for x in (b or []))
                parts.append('%s %s -> %s' % (f, va, vb))
            print('  %-12s %s' % (xgate.SHORT[t], '; '.join(parts)))
