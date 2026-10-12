#!/usr/bin/env python3
"""Screen table: per gain set and vehicle, the failing checks and the worst
normalized margin (margin / the model's uncertainty band) over the checks
that can pass. INT_LIM rows that were pruned (identical to 0.5) inherit it.

    python3 summarize_screen.py [screen.jsonl] [--excl 'vec complete:Acro release,...']
"""
import collections
import itertools
import json
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import optA  # noqa: E402
from screen2 import PS, IS  # noqa: E402

LS = [0.5, 0.3, 0.2, 0.15]
# Structurally unpassable within Option A (verified on the grid below, not assumed):
EXCLUDE = {('gz_airship_vectored_independent_am_fin_drag', 'Acro release'),
           ('gz_airship_vectored_independent_am_fin_drag', 'Stab release')}


def load(path):
    d = {}
    for line in open(path):
        r = json.loads(line)
        p = r['params']
        d[(r['target'], p['AS_YAWRATE_P'], p['AS_YAWRATE_I'], p['AS_YR_INT_LIM'])] = r
    return d


def resolve(d, t, P, I, L):
    if (t, P, I, L) in d:
        return d[(t, P, I, L)]
    base = d.get((t, P, I, 0.5))
    if base is not None and base['integ_max'] < 0.9 * L:
        return base
    return None


def evaluate(recs, targets, exclude=EXCLUDE):
    """recs: {target: record}. -> (fails [(short, check)], worst (norm, short, check), excluded fails)"""
    fails, worst, exfails = [], (float('inf'), None, None), []
    for t in targets:
        r = recs[t]
        for k, (ok, m) in optA.norm_margins(r).items():
            if (t, k) in exclude:
                if not ok:
                    exfails.append((optA.SHORT[t], k))
                continue
            if not ok:
                fails.append((optA.SHORT[t], k))
            if m is not None and not ok and m > 0:
                m = -0.01            # failed on something the margins do not see (clean start)
            if m is not None and m < worst[0]:
                worst = (m, optA.SHORT[t], k)
    return fails, worst, exfails


def main():
    path = os.path.join(HERE, sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith('-') else 'screen.jsonl')
    d = load(path)
    rows = collections.defaultdict(dict)
    for P, I, L in itertools.product(PS, IS, LS):
        for group, targets in (('vectored', optA.VECTORED), ('cloudship', optA.CLOUDSHIP)):
            recs = {t: resolve(d, t, P, I, L) for t in targets}
            if any(v is None for v in recs.values()):
                continue
            rows[group][(P, I, L)] = evaluate(recs, targets)
    for group in ('vectored', 'cloudship'):
        print('\n=== %s: gain sets with no failure outside the excluded checks, by worst normalized margin' % group)
        ok = [(v[1][0], k, v) for k, v in rows[group].items() if not v[0]]
        ok.sort(reverse=True)
        for m, k, v in ok[:40]:
            print('  P %.2f I %.3f LIM %.2f  worst %6.2f (%s / %s)  excluded fails: %d'
                  % (k[0], k[1], k[2], m, v[1][1], v[1][2], len(v[2])))
        print('  (%d of %d sets pass)' % (len(ok), len(rows[group])))
        print('\n--- %s grid (L=0.5 unless it differs): fails / worst' % group)
        for L in LS:
            print('  INT_LIM %.2f' % L)
            print('  %6s ' % 'P\\I' + ' '.join('%12.3f' % I for I in IS))
            for P in PS:
                cells = []
                for I in IS:
                    v = rows[group].get((P, I, L))
                    if v is None:
                        cells.append('%12s' % '-')
                    else:
                        cells.append('%12s' % ('%dF %+.2f' % (len(v[0]), v[1][0]) if v[1][1] else '%dF' % len(v[0])))
                print('  %6.2f ' % P + ' '.join(cells))
        print('\n--- %s failing checks per set (L=0.5)' % group)
        for P in PS:
            for I in IS:
                v = rows[group].get((P, I, 0.5))
                if v and v[0]:
                    print('  P %.2f I %.3f: %s' % (P, I, ', '.join('%s:%s' % f for f in v[0])))


if __name__ == '__main__':
    main()
