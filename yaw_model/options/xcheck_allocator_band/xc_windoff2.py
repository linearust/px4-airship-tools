#!/usr/bin/env python3
"""Is the post-wind limit cycle the allocator's? The xc_windoff.py case
(1 N m for 80 s, then 200 s wind-free) with:
  base, noise off           self-sustained, or noise-driven?
  ideal                     N = 54 clamp(u, -1, 1), reversible, no band, no
                            swing, no surge force (option E's e_ideal bound,
                            re-implemented)
  report_band, report_all   E's reporting variants

    python3 xc_windoff2.py > xc_windoff2_out.txt
"""
import math
import multiprocessing as mp
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True
DEG = 180.0 / math.pi


class IdealAct:
    def __init__(self):
        self.N = 0.0

    def step(self, dt, u):
        c = max(-1.0, min(1.0, u))
        self.N = 54.0 * c
        s = u - c
        return self.N, (1.0 if s > 1e-4 else -1.0 if s < -1e-4 else 0.0)

    def body_forces(self):
        return 0.0, 0.0

    def pod_wrench(self, dt):
        return (0.0, 0.0, 0.0, 0.0, 0.0, -self.N)

    def status(self):
        return {}


def one(job):
    target, variant, P, noise = job
    import xc_model as xm
    import closed_loop as cl
    import controller as ctl
    import planar
    params = {'AS_YAWRATE_P': P, 'AS_YAWRATE_I': 0.09}
    if variant == 'ideal':
        model, airframe = cl.TARGETS.get(target) or cl.EXTRA_TARGETS[target]
        loop = cl.Loop(ctl.GateController(airframe, params, filter_gyro=False), IdealAct(),
                       planar.RollPlanarPlant(target, noise=noise, seed=11, model=model), False)
    else:
        loop = xm.build(target, variant, params=params, noise=noise, seed=11)
    loop.set_stick(0.0)
    loop.set_mode('manual')
    loop.arm()
    loop.advance(1.0)
    loop.set_mode('acro')
    loop.advance(5.0)
    loop.set_wind(-1.0)
    loop.advance(80.0)
    loop.set_wind(0.0)
    act = getattr(loop.actuation, 'act', None)
    pods = act.alloc.pods if act else []
    wins = []
    for w in range(10):
        mx = ss = 0.0
        n = 0
        j0 = sum(p.target_jumps for p in pods)
        for _ in range(int(20.0 / 0.004)):
            loop.advance(0.004)
            r = loop.plant.rate * DEG
            mx = max(mx, abs(r))
            ss += r * r
            n += 1
        wins.append((mx, math.sqrt(ss / n), sum(p.target_jumps for p in pods) - j0, loop.controller.ctrl.integ))
    return job, wins


if __name__ == '__main__':
    V = ['gz_airship_vectored_independent', 'gz_airship_vectored_independent_am_fin_drag',
         'gz_airship_vectored_independent_fin']
    jobs = [(t, 'base', P, False) for t in V for P in (0.5, 0.7)]
    jobs += [(t, 'ideal', P, True) for t in V for P in (0.5, 0.7)]
    jobs += [(t, v, 0.7, True) for t in V for v in ('report_band', 'report_all', 'report_hold')]
    with mp.Pool(int(os.environ.get('WORKERS', '4'))) as pool:
        out = pool.map(one, jobs, chunksize=1)
    for (t, v, P, noise), wins in out:
        print('%-44s %-12s P %.1f noise %-5s  20-s windows after a 1 N m wind stops' % (t, v, P, noise))
        print('   max|r| ' + ' '.join('%.2f' % x[0] for x in wins))
        print('   rms r  ' + ' '.join('%.2f' % x[1] for x in wins))
        print('   jumps  ' + ' '.join('%4d' % x[2] for x in wins))
        print('   integ  ' + ' '.join('%+.4f' % x[3] for x in wins))
