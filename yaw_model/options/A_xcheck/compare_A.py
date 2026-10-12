#!/usr/bin/env python3
"""Record-by-record: my runs vs option A's ens.jsonl for the same (target,
set, member, seed): verdict agreement and the largest |margin difference|."""
import json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
A = os.path.join(HERE, '..', 'A', 'ens.jsonl')
def key(r):
    p = r['params']
    return (r['target'], p['AS_YAWRATE_P'], p['AS_YAWRATE_I'], p['AS_YR_INT_LIM'], r['member'], r['seed'], json.dumps(r['act'] and list(r['act'])))
theirs = {}
for l in open(A):
    r = json.loads(l); theirs[key(r)] = r
n = agree = 0; worst = (0, None)
for fn in sys.argv[1:]:
    for l in open(os.path.join(HERE, fn)):
        r = json.loads(l); k = key(r)
        if k not in theirs: continue
        t = theirs[k]; n += 1
        ok = all(bool(r['ok'][c]) == bool(t['checks'][c]['ok']) for c in t['checks'] if c in r['ok'])
        agree += ok
        for c, fs in t['margins'].items():
            for f, v in fs.items():
                mv = r['m'].get(c, {}).get(f)
                if v is not None and mv is not None and abs(v - mv) > worst[0]:
                    worst = (abs(v - mv), (k, c, f, v, mv))
print('matched runs', n, 'verdicts identical in', agree)
print('largest margin difference (their printed-2-decimal parse vs full precision):', worst)
