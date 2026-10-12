#!/usr/bin/env python3
"""Per candidate x gains x plant x check: pass count / members, worst and
median margin (units: deg/s for rates, s for settle, normalized torque for
integ), undecided count. `python3 ensemble_summary.py [file.jsonl] [filter]`"""
import collections, json, os, statistics, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.dont_write_bytecode = True
import option_d as od
path = os.path.join(HERE, sys.argv[1] if len(sys.argv) > 1 else 'ensemble.jsonl')
flt = sys.argv[2] if len(sys.argv) > 2 else None
rows = [json.loads(l) for l in open(path)]
ORDER = ['wind check', 'Manual', 'Acro brake', 'Acro track', 'Acro release', 'Stab track', 'Stab release',
         'wind I on', 'wind I off', 'wind weak']
by = collections.defaultdict(list)
for r in rows:
    key = (r['name'], r['gains'], r.get('slew'), r.get('tail_tau'))
    by[key].append(r)
out = []
for key in sorted(by, key=lambda k: (k[0], k[1], str(k[2]), str(k[3]))):
    name, gains, slew, tau = key
    if flt and flt not in '%s:%s' % (name, gains):
        continue
    rs = by[key]
    hdr = '\n##### %s  gains %s%s%s' % (name, gains, '  slew %s' % slew if slew else '', '  tail tau %s' % tau if tau else '')
    out.append(hdr)
    for t in od.TARGETS5:
        tr = [r for r in rs if r['target'] == t]
        if not tr:
            continue
        n = len(tr)
        allpass = sum(1 for r in tr if r['status'] == 0)
        out.append('  %-13s members %3d, all-pass runs %3d/%d, seeds %s' % (od.SHORT[t], n, allpass, n,
                   sorted({r['seed'] for r in tr})))
        for k in ORDER:
            cs = [r['checks'][k] for r in tr if k in r['checks']]
            if not cs:
                continue
            ok = sum(1 for c in cs if c['ok'])
            und = sum(1 for c in cs if c['undecided'])
            mk = sorted({m for c in cs for m in c['margins']})
            parts = []
            for m in mk:
                vals = [c['margins'][m] for c in cs if m in c['margins']]
                parts.append('%s min %+.3f med %+.3f (n%d)' % (m, min(vals), statistics.median(vals), len(vals)))
            extra = ''
            if 'release' in k:
                st = [c['settle'] for c in cs if c.get('settle') is not None]
                if st:
                    extra = '  settle %.1f..%.1f s' % (min(st), max(st))
                ends = [c['end'] for c in cs]
                extra += '  end %.2f..%.2f' % (min(ends), max(ends))
            if k in ('wind I on', 'wind weak', 'wind I off'):
                extra = '  rate %.2f..%.2f integ %.4f..%.4f' % (min(c['rate'] for c in cs), max(c['rate'] for c in cs),
                                                                min(c['integ'] for c in cs), max(c['integ'] for c in cs))
            if k == 'Acro brake':
                extra = '  end %.2f..%.2f' % (min(c['end'] for c in cs), max(c['end'] for c in cs))
            if 'track' in k:
                extra = '  mean %.2f..%.2f peak %.2f..%.2f' % (min(c['mean'] for c in cs), max(c['mean'] for c in cs),
                                                                min(c['peak'] for c in cs), max(c['peak'] for c in cs))
            verdict = 'PASS' if ok == len(cs) else ('FAIL' if ok == 0 else 'SPLIT')
            out.append('    %-12s %-5s %3d/%-3d und %2d  %s%s' % (k, verdict, ok, len(cs), und, '; '.join(parts), extra))
print('\n'.join(out))
