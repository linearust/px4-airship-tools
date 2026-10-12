#!/usr/bin/env python3
"""Compact matrix from a gate jsonl: candidate:gains x plant ->
all-pass runs / members; failing (F) or split (S) checks with pass counts;
undecided counts (U); smallest margins of the release, brake, peak and
wind-hold checks. python3 matrix.py [file.jsonl]"""
import collections, json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.dont_write_bytecode = True
import option_d as od
AB = {'wind check': 'wchk', 'Manual': 'man', 'Acro brake': 'brk', 'Acro track': 'aT', 'Acro release': 'aR',
      'Stab track': 'sT', 'Stab release': 'sR', 'wind I on': 'wOn', 'wind I off': 'wOff', 'wind weak': 'wWk'}
rows = [json.loads(l) for l in open(os.path.join(HERE, sys.argv[1] if len(sys.argv) > 1 else 'ensemble.jsonl'))]
by = collections.defaultdict(list)
for r in rows:
    var = ('slew%g' % r['slew']) if r.get('slew') else ('tau%g' % r['tail_tau']) if r.get('tail_tau') else ''
    by[('%s:%s%s' % (r['name'], r['gains'], (':' + var) if var else ''), r['target'])].append(r)
cands = sorted({k[0] for k in by})
for c in cands:
    print('\n%s' % c)
    for t in od.TARGETS5:
        rs = by.get((c, t))
        if not rs:
            continue
        n = len(rs)
        ap = sum(1 for r in rs if r['status'] == 0)
        bad, und = [], []
        for k, ab in AB.items():
            cs = [r['checks'][k] for r in rs if k in r['checks']]
            ok = sum(1 for x in cs if x['ok'])
            if ok < len(cs):
                bad.append('%s %s%d/%d' % (ab, 'F' if ok == 0 else 'S', ok, len(cs)))
            u = sum(1 for x in cs if x['undecided'])
            if u:
                und.append('%s %d' % (ab, u))

        def mn(k, m):
            v = [r['checks'][k]['margins'][m] for r in rs if k in r['checks'] and m in r['checks'][k]['margins']]
            return min(v) if v else None
        rel = [mn('Acro release', 'settle_s'), mn('Stab release', 'settle_s')]
        rel = [x for x in rel if x is not None]
        marg = 'rel %s  brk %s  peakA %s  wOn.rate %s  wOn.integ %s  wOff %s' % (
            ('%+.2fs' % min(rel)) if rel else 'n/a', '%+.2f' % mn('Acro brake', 'end'),
            '%+.2f' % mn('Acro track', 'peak'), '%+.2f' % mn('wind I on', 'rate'),
            '%+.4f' % mn('wind I on', 'integ'), '%+.2f' % mn('wind I off', 'rate'))
        print('  %-13s all-pass %3d/%-3d  %-40s U[%s]  min margins: %s' % (od.SHORT[t], ap, n, ', '.join(bad) or 'every check passes',
                                                               ', '.join(und), marg))
