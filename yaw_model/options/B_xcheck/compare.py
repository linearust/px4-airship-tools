#!/usr/bin/env python3
"""Pair my runs with option B's (same target, P, I, FF, act, timing member, seed)
and report the largest difference per check field, and any verdict that differs.

    python3 compare.py MINE.jsonl ../B/THEIRS.jsonl [...more of theirs]
"""
import collections
import json
import sys

sys.dont_write_bytecode = True
import xcommon as xc  # noqa: E402

MAP = {  # my check/field -> B's check/field
    ('brake', 'end'): ('Acro brake', 'end'),
    ('Acro track', 'peak'): ('Acro track', 'peak'), ('Acro track', 'mean'): ('Acro track', 'mean'),
    ('Stab track', 'peak'): ('Stab track', 'peak'), ('Stab track', 'mean'): ('Stab track', 'mean'),
    ('Acro release', 'settle'): ('Acro release', 'settle'), ('Acro release', 'r0'): ('Acro release', 'r0'),
    ('Stab release', 'settle'): ('Stab release', 'settle'), ('Stab release', 'r0'): ('Stab release', 'r0'),
    ('wind I on', 'rate'): ('wind I on', 'rate'), ('wind I on', 'integ'): ('wind I on', 'integ'),
    ('wind I off', 'rate'): ('wind I off', 'rate'), ('wind weak', 'rate'): ('wind weak', 'rate'),
}
OKMAP = {'brake': 'Acro brake'}


def key(r):
    return (r['target'], r['P'], r['I'], r['FF'], json.dumps({k: float(v) for k, v in r['act'].items()}, sort_keys=True), r['member'], r['seed'])


def main():
    mine = [json.loads(l) for l in open(sys.argv[1])]
    theirs = {}
    for f in sys.argv[2:]:
        for l in open(f):
            r = json.loads(l)
            theirs[key(r)] = r
    worst = collections.defaultdict(float)
    paired = collections.Counter()
    unpaired = collections.Counter()
    vdiff = []
    for m in mine:
        if m['err']:
            continue
        b = theirs.get(key(m))
        g = (xc.NAME[m['target']], m['FF'], json.dumps(m['act'], sort_keys=True))
        if not b:
            unpaired[g] += 1
            continue
        paired[g] += 1
        for c, rec in m['checks'].items():
            bc = OKMAP.get(c, c)
            if bc in b['checks'] and b['checks'][bc]['ok'] != rec['ok']:
                vdiff.append((g, m['member'], m['seed'], c, rec['ok'], b['checks'][bc]['ok']))
        for (c, f), (bc, bf) in MAP.items():
            if c in m['checks'] and bc in b['checks']:
                x, y = m['checks'][c].get(f), b['checks'][bc].get(bf)
                if x is None or y is None:
                    if (x is None) != (y is None):
                        worst[(g, c, f)] = float('inf')
                    continue
                worst[(g, c, f)] = max(worst[(g, c, f)], abs(x - y))
    for g in sorted(paired):
        print('%s FF %.2f %s: %d paired runs' % (g[0], g[1], g[2] if g[2] != '{}' else '', paired[g]))
        ws = {(c, f): v for (gg, c, f), v in worst.items() if gg == g}
        print('   max |mine - B| per field: ' + ', '.join('%s.%s %.2g' % (c, f, v) for (c, f), v in ws.items()))
    for g, n in unpaired.items():
        print('%s FF %.2f %s: %d runs with no B counterpart' % (g[0], g[1], g[2], n))
    print('verdict differences: %d' % len(vdiff))
    for v in vdiff[:20]:
        print('  ', v)


if __name__ == '__main__':
    main()
