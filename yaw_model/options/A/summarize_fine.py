#!/usr/bin/env python3
"""Fine grid (one record per set x plant): per vehicle, failing checks and the
worst normalized margin with the binding check, plus the raw binding margins.

    python3 summarize_fine.py fine.jsonl
"""
import collections
import json
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import optA  # noqa: E402
from summarize_screen import evaluate  # noqa: E402


def main():
    d = collections.defaultdict(dict)
    for line in open(os.path.join(HERE, sys.argv[1])):
        r = json.loads(line)
        p = r['params']
        d[(p['AS_YAWRATE_P'], p['AS_YAWRATE_I'], p['AS_YR_INT_LIM'])][r['target']] = r
    for group, targets in (('vectored', optA.VECTORED), ('cloudship', optA.CLOUDSHIP)):
        print('\n=== %s' % group)
        res = []
        for s, recs in sorted(d.items()):
            if not all(t in recs for t in targets):
                continue
            fails, worst, ex = evaluate(recs, targets)
            # per-plant worst three normalized margins
            detail = []
            for t in targets:
                nm = [(m, k) for k, (ok, m) in optA.norm_margins(recs[t]).items()
                      if m is not None and (t, k) not in {('gz_airship_vectored_independent_am_fin_drag', 'Acro release'),
                                                          ('gz_airship_vectored_independent_am_fin_drag', 'Stab release')}]
                nm.sort()
                detail.append('%s: %s' % (optA.SHORT[t], ', '.join('%s %+.2f' % (k, m) for m, k in nm[:2])))
            res.append((worst[0], s, fails, worst, detail))
        res.sort(key=lambda x: (len(x[2]), -x[0]))
        for w, s, fails, worst, detail in res:
            print('  P %.3f I %.3f: %dF worst %+.2f (%s/%s) %s' % (s[0], s[1], len(fails), w, worst[1], worst[2],
                                                                ('FAILS ' + ', '.join('%s:%s' % f for f in fails)) if fails else ''))
            print('        ' + ' | '.join(detail))


if __name__ == '__main__':
    main()
