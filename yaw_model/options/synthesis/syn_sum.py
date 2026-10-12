#!/usr/bin/env python3
"""Summarize syn.jsonl (and optionally D's ensemble.jsonl restore P0.7) per
config x plant x check: passes/runs, min and median margin, undecided count.
Margin units: deg/s for rates, s for settled releases (10 - settle), negative
end-rate excess for releases that never settle, normalized torque for integ."""
import json, os, sys, statistics as st, collections
HERE = os.path.dirname(os.path.abspath(__file__))
SHORT = {'gz_airship_vectored_independent': 'vec rigid 2520',
         'gz_airship_vectored_independent_am_fin_drag': 'vec complete 2527',
         'gz_airship_vectored_independent_fin': 'vec fin 2522',
         'gz_cloudship': 'cs rigid 2510',
         'gz_cloudship_am_fin_drag': 'cs complete 2517'}
ORDER = list(SHORT)
CHECKS = ['wind check', 'Manual', 'Acro brake', 'Acro track', 'Acro release', 'Stab track', 'Stab release',
          'wind I on', 'wind I off', 'wind weak']


def load():
    rows = [json.loads(l) for l in open(os.path.join(HERE, 'syn.jsonl'))]
    for r in rows:
        r['cfg'] = '%s P%g I%g %s' % (r['rule'], r['P'], r['I'], 'seeds>10' if r['seed'] > 10 else 'seeds1-5')
    D = os.path.join(os.path.dirname(HERE), 'D', 'ensemble.jsonl')
    for l in open(D):
        r = json.loads(l)
        if r['name'] == 'restore' and r['gains'] == 'P0.7':
            r['cfg'] = 'restore P0.7 I0.09 D-full(40)'
            rows.append(r)
    return rows


def main():
    rows = load()
    by = collections.defaultdict(list)
    for r in rows:
        by[(r['cfg'], r['target'])].append(r)
    cfgs = sorted({r['cfg'] for r in rows})
    for cfg in cfgs:
        print('=' * 100)
        print(cfg)
        worst_cfg = []
        for t in ORDER:
            rs = by.get((cfg, t), [])
            if not rs:
                continue
            allpass = sum(all(c['ok'] for c in r['checks'].values()) for r in rs)
            print('  %-18s all-pass %d/%d' % (SHORT[t], allpass, len(rs)))
            for ck in CHECKS:
                cs = [r['checks'][ck] for r in rs if ck in r['checks']]
                if not cs or not cs[0]['margins']:
                    if cs:
                        n = sum(c['ok'] for c in cs)
                        if n != len(cs):
                            print('      %-13s %d/%d' % (ck, n, len(cs)))
                    continue
                n = sum(c['ok'] for c in cs)
                und = sum(c['undecided'] for c in cs)
                parts = []
                keys = sorted({k for c in cs for k in c['margins']})
                for k in keys:
                    v = [c['margins'][k] for c in cs if k in c['margins']]
                    parts.append('%s min %+.3f med %+.3f (n%d)' % (k, min(v), st.median(v), len(v)))
                extra = ''
                if 'release' in ck:
                    ends = [c.get('end') for c in cs]
                    rev = [c.get('reverse') for c in cs]
                    extra = ' | end %.2f..%.2f rev %.2f..%.2f' % (min(ends), max(ends), min(rev), max(rev))
                if 'track' in ck:
                    pk = [c['peak'] for c in cs]
                    extra = ' | peak %.2f..%.2f' % (min(pk), max(pk))
                if ck == 'Acro brake':
                    e = [c['end'] for c in cs]
                    extra = ' | end %.2f..%.2f' % (min(e), max(e))
                if ck.startswith('wind'):
                    rt = [c['rate'] for c in cs]
                    extra = ' | rate %.3f..%.3f' % (min(rt), max(rt))
                print('      %-13s %2d/%-2d und %2d  %s%s' % (ck, n, len(cs), und, '; '.join(parts), extra))


if __name__ == '__main__':
    main()
