#!/usr/bin/env python3
"""What happens after the gate's 12 s release window closes. The gate's Acro step
as far as the rate loop sees it: from rest, Acro, stick 0.5 for 35 s, then stick
0 for 60 s (the gate looks at the first 12 s only). Rate sampled every 4 ms.
Reports the gate settle rule on the first 12 s and on the whole 60 s, and the
rate at 12/15/20/30/45/60 s. Also the allocator's saturation flags and the pods'
modes at those times, to show why a reversed rate is not braked.

    python3 release_long.py OUT.jsonl
"""
import json
import math
import multiprocessing as mp
import sys

sys.dont_write_bytecode = True
import xcommon as xc  # noqa: E402
import xbeyond as xb  # noqa: E402

DT = 0.004


def one(job):
    t, P, I, FF, seed, push_s = job
    loop = xb.start(t, P, I, FF, seed, {})
    loop.set_stick(0.5)
    xb.rec(loop, push_s)
    loop.set_stick(0.0)
    rows = []
    for k in range(int(60.0 / DT)):
        loop.advance(DT)
        rc = loop.controller.ctrl.rc
        act = getattr(loop.actuation, 'act', None)
        modes = tuple(p.mode for p in act.alloc.pods) if act else ()
        rows.append((math.degrees(loop.read_rate()), loop.read_torque(), rc.integ, rc.sat_pos, rc.sat_neg, modes))
    r = [x[0] for x in rows]
    first12 = r[:int(12.0 / DT)]
    at = {}
    for s in (5, 8, 12, 15, 20, 30, 45, 59.5):
        i = int(s / DT)
        at[str(s)] = {'rate': sum(r[i - 125:i + 125]) / 250 if i + 125 <= len(r) else r[-1],
                      'torque': rows[i][1], 'integ': rows[i][2], 'sat': [rows[i][3], rows[i][4]],
                      'modes': list(rows[i][5])}
    after12 = r[int(12.0 / DT):]
    return {'target': t, 'P': P, 'I': I, 'FF': FF, 'seed': seed, 'push_s': push_s, 'r0': r[0],
            'settle_12': xb.settle(first12), 'settle_60': xb.settle(r), 'min': min(r), 'max_after_1s': max(r[250:]),
            'max_abs_after_12': max(abs(x) for x in after12), 'mean_abs_12_60': sum(abs(x) for x in after12) / len(after12),
            'at': at}


def main():
    jobs = []
    for t, ffs in ((xc.VC, (0.0, 0.8, 1.0, 1.2)), (xc.VF, (0.0, 0.3)), (xc.CC, (0.0, 0.05)), (xc.VR, (0.0,)),
                   (xc.CR, (0.0,))):
        for ff in ffs:
            for s in (1, 2):
                jobs.append((t, 0.65, 0.07, ff, s, 35.0))
    with mp.Pool(8) as pool, open(sys.argv[1], 'w') as fh:
        for r in pool.imap_unordered(one, jobs):
            fh.write(json.dumps(r) + '\n')
            fh.flush()
    print('DONE')


if __name__ == '__main__':
    main()
