#!/usr/bin/env python3
"""What FF should be, per plant: the normalized torque that holds a yaw rate
r in a steady turn, divided by r. FF_ideal(r) = tau_hold(r) / r, in the
firmware's units (normalized torque per rad/s, the same units as P).

Two readings:
  analytic   the plant part's own damping (plants.py): hull nRabsR r|r| plus
             the vertical fins broadside (one fin nominal, two as the upper
             bound), over the actuation authority (54 N m vectored, 4.478 N m
             Cloudship). Pure yaw, no drift, no Munk moment.
  closed     the closed-loop model (roll plant, drift, Munk, fins on their
             real velocity) held in Acro at a stick with P 0.7 I 0.09 FF 0
             until steady, torque and rate averaged over one full heading
             turn (360 deg, so the Munk sin(2 psi) term averages out), from
             rest. INT_LIM 0.3 is raised to 1.0 here so the hold torque is
             not capped (the cap is reported separately).

    python3 derive_ff.py > derive_ff_out.txt
"""
import math
import multiprocessing as mp
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True
import ffcommon as fc  # noqa: E402
import plants  # noqa: E402

AUTH = {'gz_airship_vectored_independent': 54.0, 'gz_airship_vectored_independent_am_fin_drag': 54.0,
        'gz_airship_vectored_independent_fin': 54.0, 'gz_cloudship': 4.478, 'gz_cloudship_am_fin_drag': 4.478}
SPS = [2.5, 5.0, fc.SP, 10.0, 15.0]


def stick_for(sp):
    return 0.1 + 0.9 * sp / 15.0


def analytic(target, r_deg):
    r = math.radians(r_deg)
    if target in plants.PLANTS:
        p = plants.PLANTS[target]
        hull = p.c_hull * r * r
        fins = [c * r * r for c in p.c_fin_each()] if p.fins else []
    else:   # the fin hold-out: rigid body + the complete model's two vertical fins, no hull drag
        p = plants.PLANTS['gz_airship_vectored_independent_am_fin_drag']
        hull = 0.0
        fins = [c * r * r for c in p.c_fin_each()]
    one = hull + (sum(fins) / len(fins) if fins else 0.0)
    two = hull + sum(fins)
    return one / AUTH[target], two / AUTH[target]


def closed(job):
    target, sp = job
    loop = fc.build(target, 0.7, 0.09, 0.0, int_lim=1.0)
    loop.set_stick(0.0)
    loop.arm()
    loop.set_mode('manual')
    loop.advance(1.0)
    loop.set_mode('acro')
    loop.advance(2.0)
    loop.set_stick(stick_for(sp))
    settle = 60.0 if target.endswith('drag') or target.endswith('fin') else 40.0
    loop.advance(settle)
    avg = 360.0 / sp
    n = 0
    tq = rt = ig = 0.0
    step = 0.02
    for _ in range(int(avg / step)):
        loop.advance(step)
        tq += loop.read_torque()
        rt += loop.read_rate()
        ig += loop.controller.ctrl.rc.integ
        n += 1
    tq, rt, ig = tq / n, rt / n, ig / n
    return target, sp, rt, tq, ig


def main():
    print('FF_ideal = tau_hold / r  [normalized torque per rad/s]')
    print('\nANALYTIC (hull r|r| + vertical fins broadside, no drift, no Munk)')
    print('  %-14s' % 'r deg/s' + ''.join('%16.2f' % s for s in SPS))
    for t in fc.TARGETS5:
        cells = []
        for s in SPS:
            one, two = analytic(t, s)
            r = math.radians(s)
            cells.append('%6.3f..%-6.3f' % (one / r, two / r))
        print('  %-14s' % fc.SHORT[t] + ''.join('%16s' % c for c in cells))
    print('  (torque to hold 6.67 deg/s: ' + ', '.join('%s %.4f..%.4f' % ((fc.SHORT[t],) + analytic(t, fc.SP))
                                                   for t in fc.TARGETS5) + ')')

    jobs = [(t, s) for t in fc.TARGETS5 for s in SPS]
    with mp.Pool(10) as pool:
        res = pool.map(closed, jobs)
    print('\nCLOSED LOOP (roll plant, Acro, P 0.7 I 0.09 FF 0, INT_LIM 1.0; mean over one 360 deg turn)')
    print('  %-14s %8s %8s %9s %9s %9s' % ('plant', 'sp', 'rate', 'torque', 'integ', 'FF_ideal'))
    for t, sp, rt, tq, ig in res:
        print('  %-14s %8.2f %8.3f %9.4f %9.4f %9.3f' % (fc.SHORT[t], sp, rt, tq, ig, tq / math.radians(sp)))


if __name__ == '__main__':
    main()
