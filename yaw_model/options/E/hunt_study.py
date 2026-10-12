#!/usr/bin/env python3
"""Tilt hunting after a turn: stick 0.5 (6.67 deg/s) for 40 s, then centred
for 80 s, with gyro noise x k. Counts, over the 80 s after release, the tilt
servo's physical activity (e_model.PodE counters, both pods summed):
  reversals  target moved > 90 deg (commanded swings, incl. aborted ones)
  starts     the tilt started moving after standing still (stop-start chatter)
  dirrev     the slewing direction reversed
  travel     total tilt travel [deg]
and the rate at the end and the time to |r| < 0.5 deg/s held 2 s.

    python3 hunt_study.py            # -> hunt_study.txt
"""
import math
import multiprocessing as mp
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True

GYRO_NOISE = 0.0008726646
DEG = 180.0 / math.pi


def one(job):
    target, band, P, k = job
    import e_model as em
    import planar
    planar.GYRO_NOISE = GYRO_NOISE * k
    loop = em.build(target, band, params={'AS_YAWRATE_P': P, 'AS_YAWRATE_I': 0.09}, noise=True, seed=7,
                    trace=True)
    loop.set_stick(0.0)
    loop.set_mode('manual')
    loop.arm()
    loop.advance(1.0)
    loop.set_mode('acro')
    loop.advance(5.0)
    loop.set_stick(0.5)
    loop.advance(40.0)
    pods = loop.actuation.act.alloc.pods
    snap = [(p.swings, p.starts, p.dir_reversals, p.travel) for p in pods]
    k0 = len(loop.trace)
    loop.set_stick(0.0)
    loop.advance(80.0)
    after = [(p.swings, p.starts, p.dir_reversals, p.travel) for p in pods]
    d = [sum(a[i] - b[i] for a, b in zip(after, snap)) for i in range(4)]
    tr = loop.trace[k0:]
    t0 = tr[0][0]
    rest = None
    start = None
    for r in tr:
        if abs(r[14] * DEG) < 0.5:
            start = r[0] if start is None else start
            if r[0] - start >= 2.0:
                rest = start - t0
                break
        else:
            start = None
    tq = [r[4] for r in tr[-int(40 / 0.004):]]
    mu = sum(tq) / len(tq)
    std = math.sqrt(sum((x - mu) ** 2 for x in tq) / len(tq))
    return (target, band, P, k, d[0], d[1], d[2], math.degrees(d[3]), rest, tr[-1][14] * DEG, std)


if __name__ == '__main__':
    targets = ['gz_airship_vectored_independent', 'gz_airship_vectored_independent_am_fin_drag',
               'gz_airship_vectored_independent_fin']
    jobs = [(t, b, P, k) for t in targets for b in ('base', 'report_band', 'report_all', 'deliver')
            for P in (0.5, 0.7) for k in (1.0, 30.0, 60.0)]
    short = {targets[0]: 'rigid', targets[1]: 'complete', targets[2]: '_fin'}
    out = []
    with mp.Pool(int(sys.argv[1]) if len(sys.argv) > 1 else 2) as pool:
        for r in pool.imap(one, jobs):
            out.append(r)
    lines = ['80 s after releasing a 6.67 deg/s turn (both pods summed; noise seed 7; I 0.09)',
             '%-9s %-12s %4s %4s  %9s %6s %6s %8s  %-6s %8s %8s' % ('plant', 'band', 'P', 'k', 'reversals',
                                                                'starts', 'dirrev', 'travel', 'rest', 'r_end',
                                                                'tq std')]
    for (t, b, P, k, sw, st, dr, trv, rest, rend, std) in out:
        lines.append('%-9s %-12s %4.1f %4g  %9d %6d %6d %8.0f  %-6s %+8.2f %8.4f'
                     % (short[t], b, P, k, sw, st, dr, trv, 'never' if rest is None else '%.1f' % rest, rend, std))
    text = '\n'.join(lines)
    print(text)
    with open(os.path.join(HERE, 'hunt_study.txt'), 'w') as fh:
        fh.write(text + '\n')
