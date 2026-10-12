#!/usr/bin/env python3
"""Per gain set x plant x check: pass count over the ensemble, the judged
margin (raw, gate units) min / median / max, the margin over the model's
uncertainty band, and a verdict:
  PASS       every member passes and the worst margin clears the band
  FAIL       every member fails and the best margin is below -band
  UNDECIDED  a split, or the worst (best) margin inside the band

    python3 summarize_ensemble.py ensemble.jsonl [act-filter]
"""
import collections
import json
import os
import statistics
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import optA  # noqa: E402

FIELDS = {'Acro brake': ['end'], 'Acro track': ['mean', 'peak'], 'Stab track': ['mean', 'peak'],
          'Acro release': ['release'], 'Stab release': ['release'], 'wind I on': ['rate', 'integ'],
          'wind I off': ['rate'], 'wind weak': ['rate', 'integ']}
UNIT = {'end': 'deg/s', 'mean': 'deg/s', 'peak': 'deg/s', 'release': 'deg/s (rest threshold shift)',
        'rate': 'deg/s', 'integ': 'norm. torque'}


def summarize(recs):
    """recs: records of one (set, target). -> {check: dict}"""
    out = {}
    for k in optA.CHECKS:
        rs = [r for r in recs if k in r['checks']]
        if not rs:
            continue
        n = len(rs)
        npass = sum(r['checks'][k]['ok'] for r in rs)
        fields = {}
        worst_norm, best_norm = float('inf'), -float('inf')
        for f in FIELDS[k]:
            vals = [r['margins'].get(k, {}).get(f) for r in rs]
            vals = [v for v in vals if v is not None]
            if not vals:
                continue
            b = optA.band(rs[0]['target'], k, f)
            fields[f] = (min(vals), statistics.median(vals), max(vals), b)
        # per member: the binding field is the min normalized margin
        per = []
        for r in rs:
            mm = [r['margins'].get(k, {}).get(f) / optA.band(r['target'], k, f)
                  for f in FIELDS[k] if r['margins'].get(k, {}).get(f) is not None]
            m = min(mm) if mm else None
            if m is not None and not r['checks'][k]['ok'] and m > 0:
                m = -0.01
            per.append(m)
        per = [p for p in per if p is not None]
        worst_norm, best_norm = (min(per), max(per)) if per else (None, None)
        if npass == n and worst_norm is not None and worst_norm >= 1.0:
            v = 'PASS'
        elif npass == 0 and best_norm is not None and best_norm <= -1.0:
            v = 'FAIL'
        else:
            v = 'UNDECIDED'
        out[k] = {'n': n, 'pass': npass, 'fields': fields, 'worst_norm': worst_norm, 'best_norm': best_norm,
                  'verdict': v}
    return out


def main():
    path = os.path.join(HERE, sys.argv[1])
    actf = sys.argv[2] if len(sys.argv) > 2 else 'null'
    by = collections.defaultdict(list)
    for line in open(path):
        r = json.loads(line)
        if json.dumps(r['act']) != actf:
            continue
        p = r['params']
        by[((p['AS_YAWRATE_P'], p['AS_YAWRATE_I'], p['AS_YR_INT_LIM']), r['target'])].append(r)
    sets = sorted({k[0] for k in by})
    for s in sets:
        print('\n##### P %.2f  I %.3f  INT_LIM %.2f   (act %s)' % (s + (actf,)))
        for t in optA.TARGETS:
            recs = by.get((s, t))
            if not recs:
                continue
            seeds = sorted({r['seed'] for r in recs})
            print('  --- %s (%s): %d runs, seeds %s' % (optA.SHORT[t], t, len(recs), seeds))
            for k, v in summarize(recs).items():
                fs = '; '.join('%s %+.3f/%+.3f/%+.3f (band %.3g)' % (f, a, b, c, bd)
                               for f, (a, b, c, bd) in v['fields'].items())
                print('    %-13s %2d/%-2d %-9s worst %+6.2f band  | margin min/med/max: %s'
                      % (k, v['pass'], v['n'], v['verdict'], v['worst_norm'] if v['worst_norm'] is not None else float('nan'), fs))


if __name__ == '__main__':
    main()
