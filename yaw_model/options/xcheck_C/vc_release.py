"""Fin 2nd case at P 0.7 I 0.09 (b 1 / 0.9) and P 0.7 I 0.2 b 0.9: the gate's
release windows row by row near the end - how far from settling is the
'failure' (first |r| < 1.0 time, the slope)."""
import sys, os
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import xrun, optc, gate
t = 'gz_airship_vectored_independent_am_fin_drag'
od = os.path.join(xrun.HERE, 'tmp_out', 'vcrel'); os.makedirs(od, exist_ok=True)
for (P, I, b) in ((0.7, 0.2, 1.0), (0.7, 0.2, 0.9), (0.8, 0.2, 1.0)):
    for seed in (31, 32):
        res = gate.run_gate(xrun.build_mine(t, P, I, b, seed), optc.model_name(t), out_dir=od)
        for ph in ('Acro 놓음', 'Stabilized 놓음'):
            rows = [r for r in res.samples if r['phase'] == ph]
            t0 = rows[0]['t']
            first = next((r['t'] - t0 for r in rows if abs(r['rate']) < 1.0), None)
            at10 = min(rows, key=lambda r: abs(r['t'] - t0 - 10.0))
            v = [x for x in res.verdicts if ('Acro' if 'Acro' in ph else 'Stabilized') in x[0] and '선다' in x[0]]
            print('P %.1f I %.2f b %.1f seed %d %-16s start %+.2f first|r|<1 at %s s, r@10s %+.2f, end %+.2f (%.1f s) | %s' % (
                P, I, b, seed, ph, rows[0]['rate'], ('%.1f' % first) if first is not None else '-', at10['rate'], rows[-1]['rate'],
                rows[-1]['t'] - t0, v[0][1:] if v else ''))
