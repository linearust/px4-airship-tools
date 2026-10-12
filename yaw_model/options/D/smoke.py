import os, sys, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.dont_write_bytecode = True
import gate_eval as ge
for rule in ({'kind': 'none'}, {'kind': 'reset'}):
    r = ge.run_one(dict(id='x', target='gz_airship_vectored_independent_am_fin_drag', rule=rule, gains='P0.5',
                        member='nominal', timing={}, seed=1))
    print(rule, r['status'], r['n_events'])
    for k, v in r['checks'].items():
        print('  %-12s %s %s und=%s %s' % (k, 'PASS' if v['ok'] else 'FAIL', {a: round(b, 3) for a, b in v['margins'].items()}, v['undecided'],
              {a: (round(b, 3) if isinstance(b, float) else b) for a, b in v.items() if a not in ('ok', 'margins', 'undecided')}))
