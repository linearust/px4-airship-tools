#!/usr/bin/env python3
"""Is the complete vectored airship's full-stick shortfall the integrator
limit? Full stick 60 s at P 0.7 I 0.2 with AS_YR_INT_LIM 0.3 and 0.5: time
the integrator first reaches the limit, and the rate it settles to."""
import os, sys, multiprocessing as mp
sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import xbeyond
def one(args):
    t, L = args
    rows = xbeyond.simulate(t, {'AS_YAWRATE_P': 0.7, 'AS_YAWRATE_I': 0.2, 'AS_YR_INT_LIM': L}, None, 11, 62.0,
                            [(2.0, 'stick', 1.0)])
    hit = next((r[0] - 2.0 for r in rows if abs(r[3]) >= L - 1e-6), None)
    tail = [r for r in rows if r[0] >= 57.0]
    return t, L, hit, sum(r[1] for r in tail) / len(tail), rows[-1][3], rows[-1][4]
if __name__ == '__main__':
    jobs = [(t, L) for t in ('gz_airship_vectored_independent_am_fin_drag', 'gz_airship_vectored_independent_fin') for L in (0.3, 0.5)]
    with mp.Pool(4) as p:
        for t, L, hit, mean, integ, tq in p.map(one, jobs):
            print('%-45s INT_LIM %.1f  integ reaches the limit at %s s; last-5 s rate %.2f deg/s (sp 15); integ %.3f torque %.3f'
                  % (t, L, None if hit is None else round(hit, 1), mean, integ, tq))
