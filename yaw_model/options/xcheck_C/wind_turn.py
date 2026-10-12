"""Vec rigid, wind step in a steady stick-0.5 turn: is the b effect on the
deviation real or the phase of the steer-band limit cycle? Wind onsets
40..50 s; reports the rate mean/std over the 20 s before the wind, and the
peak / end deviation from that mean."""
import json, math, os, sys
import multiprocessing as mp
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import xrun
from xbeyond import Rec

def one(job):
    t, P, I, b, onset, scale = job
    loop = xrun.build_mine(t, P, I, b, 1, None, noise=False)
    d = Rec(loop)
    d.go(onset, 0.5)
    pre = d.seg(d.t - 20, d.t)
    m = sum(x[1] for x in pre) / len(pre)
    sd = (sum((x[1] - m) ** 2 for x in pre) / len(pre)) ** 0.5
    t0 = d.t
    d.go(40.0, wind=-scale * 1.0)
    w = d.seg(t0, d.t)
    dev = [x[1] - m for x in w]
    worst = max(dev, key=abs)
    return (t, P, I, b, onset, scale, m, sd, worst, sum(dev[-1250:]) / 1250, pre[-1][3])

jobs = [('gz_airship_vectored_independent', P, I, b, on, sc) for (P, I, b) in ((0.7, 0.09, 1.0), (0.7, 0.09, 0.9), (0.7, 0.09, 0.8), (0.7, 0.2, 1.0), (0.7, 0.2, 0.9))
        for on in (40.0, 42.0, 44.0, 46.0, 48.0, 50.0) for sc in (1.0, 5.0)]
with mp.Pool(4) as pool:
    res = pool.map(one, jobs)
res.sort()
for r in res:
    print('P %.1f I %.2f b %.1f onset %4.0f scale %.0f | pre mean %5.2f std %4.2f | peak dev %+6.2f last-5s dev %+5.2f | integ %.4f' % r[1:])
