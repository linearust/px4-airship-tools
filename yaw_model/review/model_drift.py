#!/usr/bin/env python3
"""The same table as log_drift.py from the closed-loop model's trace."""
import math, os, sys, bisect
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import closed_loop, gate, reproduce

target = sys.argv[1]
loop = closed_loop.build(target, trace=True, params=reproduce.record_params(target)[0])  # the gains the chain7/chain9 records were flown with (record_gains)
res = gate.run_gate(loop, (closed_loop.TARGETS.get(target) or closed_loop.EXTRA_TARGETS[target])[0])
tr = loop.trace
ts = [row[0] for row in tr]
ups = [tr[i][0] for i in range(1, len(tr)) if tr[i][2] > 0.25 and tr[i - 1][2] <= 0.25]
downs = [tr[i][0] for i in range(1, len(tr)) if tr[i][2] <= 0.25 and tr[i - 1][2] > 0.25]
names = ['Manual', 'Acro push', 'Stab push']
for k, (t0, t1) in enumerate(zip(ups, downs)):
    print('\n== %s, %.1f s long, sim t %.2f' % (names[k], t1 - t0, t0))
    dt = 0.0
    while dt <= t1 - t0 + 12.0:
        i = bisect.bisect_left(ts, t0 + dt)
        e = tr[min(i, len(tr) - 1)][10]
        print('   t+%5.1f  u %+.4f  v(FLU) %+.4f  |V| %.4f  roll %+6.2f  rate %6.2f  fins %d' % (
            dt, e['u'], e['v'], math.hypot(e['Vx'], e['Vy']), e.get('phi_deg', 0.0), -math.degrees(tr[min(i, len(tr)-1)][3]) * -1, e['fins_open']))
        dt += 2.0
