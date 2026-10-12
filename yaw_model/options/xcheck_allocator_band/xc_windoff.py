#!/usr/bin/env python3
"""After a steady wind stops: does the hull come back to rest, and how fast?
(E reported the wind-on response only.) Wind 80 s, then 200 s wind-free;
per 20 s window of the wind-free part: max |r| and rms r [deg/s], pod target
jumps, and the time the saturation flag was set.

    python3 xc_windoff.py > xc_windoff_out.txt
"""
import math
import multiprocessing as mp
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True
DEG = 180.0 / math.pi


def one(job):
    target, variant, P, wind = job
    import xc_model as xm
    loop = xm.build(target, variant, params={'AS_YAWRATE_P': P, 'AS_YAWRATE_I': 0.09}, noise=True, seed=11)
    loop.set_stick(0.0)
    loop.set_mode('manual')
    loop.arm()
    loop.advance(1.0)
    loop.set_mode('acro')
    loop.advance(5.0)
    loop.set_wind(wind)
    loop.advance(80.0)
    integ0 = loop.controller.ctrl.integ
    loop.set_wind(0.0)
    pods = getattr(loop.actuation, 'act', None)
    pods = pods.alloc.pods if pods else []
    wins = []
    for w in range(10):
        mx = ss = 0.0
        n = 0
        flag = 0.0
        j0 = sum(p.target_jumps for p in pods)
        for _ in range(int(20.0 / 0.004)):
            loop.advance(0.004)
            r = loop.plant.rate * DEG
            mx = max(mx, abs(r))
            ss += r * r
            n += 1
            rc = loop.controller.ctrl.rc
            if rc.sat_pos or rc.sat_neg:
                flag += 0.004
        wins.append((mx, math.sqrt(ss / n), sum(p.target_jumps for p in pods) - j0, flag, loop.controller.ctrl.integ))
    return job, integ0, wins


if __name__ == '__main__':
    V = ['gz_airship_vectored_independent', 'gz_airship_vectored_independent_am_fin_drag',
         'gz_airship_vectored_independent_fin']
    jobs = [(t, v, P, w) for t in V for v in ('base', 'deliver') for P in (0.5, 0.7) for w in (-1.0, -0.4)]
    jobs += [(t, 'base', P, w) for t in ('gz_cloudship', 'gz_cloudship_am_fin_drag') for P in (0.5, 0.7)
             for w in (-0.082963,)]
    with mp.Pool(int(os.environ.get('WORKERS', '4'))) as pool:
        out = pool.map(one, jobs, chunksize=1)
    for (t, v, P, w), integ0, wins in out:
        print('%-44s %-8s P %.1f wind %+.3f N m  I at wind-off %+.4f' % (t, v, P, w, integ0))
        print('   20-s windows after wind-off: max|r| ' + ' '.join('%.2f' % x[0] for x in wins))
        print('                                rms r  ' + ' '.join('%.2f' % x[1] for x in wins))
        print('                                jumps  ' + ' '.join('%4d' % x[2] for x in wins)
              + '   sat-flag s ' + ' '.join('%.1f' % x[3] for x in wins))
        print('                                integ  ' + ' '.join('%+.4f' % x[4] for x in wins))
