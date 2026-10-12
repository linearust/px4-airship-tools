#!/usr/bin/env python3
"""Weathercock table: per plant x turn, over the six wind directions, the
mean and worst |final heading change after release| and |max excursion|,
and the stop time, per candidate."""
import collections, json, os, sys, statistics
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.dont_write_bytecode = True
import option_d as od
rows = [json.loads(l) for l in open(os.path.join(HERE, 'wcock.jsonl'))]
by = collections.defaultdict(dict)
for r in rows:
    turn, d = r['scen'][5:].split('_')
    by[(r['target'], int(turn))].setdefault(r['name'] + ':' + r['gains'], {})[int(d)] = r['m']
for (t, turn) in sorted(by, key=lambda k: (od.TARGETS5.index(k[0]), k[1])):
    print('%-13s turn %3d deg' % (od.SHORT[t], turn))
    for c, dd in sorted(by[(t, turn)].items()):
        fin = [abs(m['final_dpsi']) for m in dd.values()]
        mx = [m['max_dpsi'] for m in dd.values()]
        st = [m['stop_1'] for m in dd.values() if m['stop_1'] is not None]
        nev = sum(1 for m in dd.values() if m['stop_1'] is None)
        wins = ' '.join('%+4d:%5.1f' % (d, dd[d]['final_dpsi']) for d in sorted(dd))
        print('   %-14s n%d |final| mean %5.1f max %5.1f  max excursion mean %5.1f max %5.1f  stop<1 mean %4.1f s%s   [%s]'
              % (c, len(dd), statistics.mean(fin), max(fin), statistics.mean(mx), max(mx),
                 statistics.mean(st) if st else float('nan'), (' (%d never)' % nev) if nev else '', wins))
