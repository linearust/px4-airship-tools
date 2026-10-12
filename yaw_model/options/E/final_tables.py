#!/usr/bin/env python3
"""Compact matrix from eval_gate.jsonl: per plant, per band x gains, each
check's pass count over the noisy ensemble and its worst margin (the margin
of the member closest to failing, or furthest outside), with the flag.

    python3 final_tables.py > eval_gate_matrix.txt
"""
import collections
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import summarize as sm  # noqa: E402

ABBR = {'wind check': 'windchk', 'Manual': 'Manual', 'Acro brake': 'brake', 'Acro track': 'A.track',
        'Acro release': 'A.rel', 'Stab track': 'S.track', 'Stab release': 'S.rel', 'wind I on': 'I-on',
        'wind I off': 'I-off', 'wind weak': 'weak'}


def main(path):
    rows = sm.load(path)
    groups = collections.defaultdict(list)
    for r in rows:
        groups[(r['target'], r['band'], r['gains'])].append(r)
    for target in sm.TARGET_ORDER:
        keys = sorted([k for k in groups if k[0] == target], key=lambda k: (k[2] != 'current', k[1] != 'base', k[1]))
        if not keys:
            continue
        print('\n%s (%s)' % (sm.SHORT[target], target))
        for k in keys:
            rs = groups[k]
            ens = [r for r in rs if r['seed'] is not None]
            sw = [r['swings'] for r in ens if r.get('swings') is not None]
            parts = []
            for key in sm.ORDER:
                recs = [r['checks'].get(key) for r in ens if r['checks'].get(key) is not None]
                if not recs:
                    continue
                npass = sum(1 for x in recs if x['ok'])
                # per margin: the ensemble range; decided pass iff every member is
                # further inside than the uncertainty, decided fail iff every member
                # is further outside; else undecided
                lo, hi, us, never = {}, {}, {}, {}
                for x in recs:
                    for name, m, u in sm.margins(key, x, target):
                        us[name] = u
                        if m is None:
                            never[name] = never.get(name, 0) + 1
                            continue
                        lo[name] = min(lo.get(name, m), m)
                        hi[name] = max(hi.get(name, m), m)
                undecided = 0 < npass < len(recs)
                ms = []
                all_inside = True
                for name in us:
                    if name in lo:
                        ms.append('%+.2f..%+.2f' % (lo[name], hi[name]) if abs(lo[name]) >= 0.01 or abs(hi[name]) >= 0.01
                                  else '%+.4f..%+.4f' % (lo[name], hi[name]))
                        if never.get(name):
                            ms[-1] += ' (%d never)' % never[name]
                        u = us[name]
                        if npass == len(recs) and lo[name] < u:
                            undecided = True
                        if npass == 0 and hi[name] > -u and not never.get(name):
                            undecided = True
                        if lo[name] <= 0:
                            all_inside = False
                    else:
                        ms.append('never')
                        all_inside = False
                pre = npass < len(recs) and all_inside
                flag = 'U' if undecided and not pre else ('' if npass == len(recs) else ('F(pre)' if pre else 'F'))
                parts.append('%s %d/%d%s [%s]' % (ABBR[key], npass, len(recs), (' ' + flag) if flag else '',
                                                 '/'.join(ms)))
            print('  %-14s %-7s N=%-2d sw %s: %s' % (k[1], k[2], len(ens),
                                                   '%d-%d' % (min(sw), max(sw)) if sw else '-', '; '.join(parts)))


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, 'eval_gate.jsonl'))
