#!/usr/bin/env python3
"""Rank gain sets by the ensemble worst case: per vehicle, the minimum over
plants, checks (minus the structurally failing vec-complete releases) and
ensemble members of margin / band; the three lowest (plant, check) with
their raw worst margin; and a count of UNDECIDED checks.

    python3 rank_ensemble.py ens.jsonl [act-filter]
"""
import collections
import json
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import optA  # noqa: E402
from summarize_ensemble import summarize  # noqa: E402
from summarize_screen import EXCLUDE  # noqa: E402


def main():
    actf = sys.argv[2] if len(sys.argv) > 2 else 'null'
    by = collections.defaultdict(list)
    for line in open(os.path.join(HERE, sys.argv[1])):
        r = json.loads(line)
        if json.dumps(r['act']) != actf:
            continue
        p = r['params']
        by[((p['AS_YAWRATE_P'], p['AS_YAWRATE_I'], p['AS_YR_INT_LIM']), r['target'])].append(r)
    sets = sorted({k[0] for k in by})
    for group, targets in (('vectored', optA.VECTORED), ('cloudship', optA.CLOUDSHIP)):
        print('\n=== %s (act %s)' % (group, actf))
        out = []
        for s in sets:
            if not all((s, t) in by for t in targets):
                continue
            items, und, fails, n = [], [], [], set()
            for t in targets:
                n.add(len(by[(s, t)]))
                for k, v in summarize(by[(s, t)]).items():
                    if (t, k) in EXCLUDE:
                        continue
                    items.append((v['worst_norm'], optA.SHORT[t], k, v))
                    if v['verdict'] == 'UNDECIDED':
                        und.append('%s/%s' % (optA.SHORT[t], k))
                    if v['verdict'] == 'FAIL' or v['pass'] < v['n']:
                        fails.append('%s/%s %d/%d' % (optA.SHORT[t], k, v['pass'], v['n']))
            items.sort(key=lambda x: x[0])
            out.append((items[0][0], s, items, und, fails, n))
        out.sort(key=lambda x: -x[0])
        for w, s, items, und, fails, n in out:
            low = '; '.join('%s/%s %+.2f' % (it[1], it[2], it[0]) for it in items[:3])
            print('  P %.3f I %.3f L %.2f  runs/plant %s  worst %+.2f  | lowest: %s' % (s + (sorted(n), w, low)))
            print('        not-all-pass: %s | undecided: %s' % (', '.join(fails) or 'none', ', '.join(und) or 'none'))


if __name__ == '__main__':
    main()
