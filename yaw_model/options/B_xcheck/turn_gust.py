#!/usr/bin/env python3
"""Gust during a steady turn, measured against the SETPOINT (not a pre-gust base
that may still be converging), vec complete FF 0 vs 1.0 and vec fin FF 0 vs 0.3,
seeds 1-3. 200 s of stick 0.5, then the gate's strong gust for 90 s.
Also the release after the 200 s turn, recorded 90 s: the reverse rate that
lingers inside the pods' steer band.

    python3 turn_gust.py OUT.jsonl
"""
import json
import math
import multiprocessing as mp
import sys

sys.dont_write_bytecode = True
import xcommon as xc  # noqa: E402
import xbeyond as xb  # noqa: E402


def one(job):
    t, FF, seed, what = job
    loop = xb.start(t, 0.65, 0.07, FF, seed, {})
    loop.set_stick(0.5)
    turn = xb.rec(loop, 200.0)
    sp = (0.5 - 0.1) / 0.9 * 15.0
    pre = sum(x[0] for x in turn[-2500:]) / 2500
    out = {'target': t, 'FF': FF, 'seed': seed, 'what': what, 'pre_mean_minus_sp': pre - sp,
           'integ_pre': turn[-1][2]}
    if what == 'gust':
        loop.set_wind(xc.gate.GZ_WORLD_Z_TO_FRD * 1.0)
        g = xb.rec(loop, 90.0)
        e = [x[0] - sp for x in g]
        out.update({'dev_peak': min(e) if abs(min(e)) > abs(max(e)) else max(e),
                    'last_out_0.25': xb.last_exit(e, 0.25), 'last_out_0.5': xb.last_exit(e, 0.5),
                    'err_30_35': sum(e[7500:8750]) / 1250, 'err_60_65': sum(e[15000:16250]) / 1250,
                    'err_end5': sum(e[-1250:]) / 1250, 'integ_end': g[-1][2]})
    else:
        loop.set_stick(0.0)
        rel = xb.rec(loop, 90.0)
        r = [x[0] for x in rel]
        out.update({'r0': r[0], 'under': min(r), 'r_at_20': sum(r[4750:5250]) / 500,
                    'r_at_40': sum(r[9750:10250]) / 500, 'r_at_60': sum(r[14750:15250]) / 500,
                    'r_end': sum(r[-500:]) / 500, 'integ_end': rel[-1][2],
                    'torque_end': sum(x[1] for x in rel[-500:]) / 500, 'settle_gate_rule': xb.settle(r)})
    return out


def main():
    jobs = [(t, ff, s, w) for t, ffs in ((xc.VC, (0.0, 1.0)), (xc.VF, (0.0, 0.3)))
            for ff in ffs for s in (1, 2, 3) for w in ('gust', 'release')]
    with mp.Pool(8) as pool, open(sys.argv[1], 'w') as fh:
        for r in pool.imap_unordered(one, jobs):
            fh.write(json.dumps(r) + '\n')
            fh.flush()
    print('DONE')


if __name__ == '__main__':
    main()
