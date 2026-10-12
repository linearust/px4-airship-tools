import json, os, sys, collections
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.dont_write_bytecode = True
import xlib
f = sys.argv[1] if len(sys.argv) > 1 else 'bey.jsonl'
rows = [json.loads(l) for l in open(os.path.join(HERE, f))]
by = collections.defaultdict(list)
for r in rows:
    by[(r['scen'], str(r.get('variant')))].append(r)
def fv(v):
    if v is None: return 'never'
    if isinstance(v, float): return '%.4f' % v if abs(v) < 0.1 else '%.2f' % v
    return str(v)
for (s, var), rs in sorted(by.items()):
    print('\n## %s  variant %s' % (s, var))
    keys = [k for k in rs[0]['m'].keys()]
    for t in xlib.T5:
        for rule in ('none', 'restore', 'reset'):
            sel = sorted([r for r in rs if r['target'] == t and r['rule'] == rule], key=lambda r: r['seed'])
            for r in sel:
                print('  %-12s %-8s s%d  %s' % (xlib.SHORT[t], rule, r['seed'], '  '.join('%s %s' % (k, fv(r['m'][k])) for k in keys)))
