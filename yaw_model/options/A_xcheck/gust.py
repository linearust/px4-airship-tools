#!/usr/bin/env python3
"""5 s gust at 10 % of authority: the heading's maximum over the whole run
(the hull coasts on after the gust ends), not only while it blows."""
import os, sys, multiprocessing as mp
sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import xbeyond, xgate
def one(a):
    t, P, I = a
    W = 0.1 * xbeyond.AUTH[t in xgate.VECTORED]
    rows = xbeyond.simulate(t, {'AS_YAWRATE_P': P, 'AS_YAWRATE_I': I, 'AS_YR_INT_LIM': 0.3}, None, 11, 60.0,
                            [(2.0, 'wind', W), (7.0, 'wind', 0.0)])
    h0 = [r for r in rows if r[0] >= 2.0][0][2]
    return t, P, I, max(r[2] - h0 for r in rows if r[0] >= 2.0)
if __name__ == '__main__':
    jobs = [(t, P, I) for t in xgate.TARGETS for P, I in ((0.5, 0.09), (0.7, 0.2))]
    with mp.Pool(5) as p:
        for t, P, I, h in p.map(one, jobs):
            print('%-12s P %.1f I %.2f  heading max %.1f deg' % (xgate.SHORT[t], P, I, h))
