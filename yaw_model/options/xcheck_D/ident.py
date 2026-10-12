"""My ReleaseRC vs option D's OptionD, row for row, and 'none' vs the shared loop."""
import os, sys, multiprocessing as mp
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.dont_write_bytecode = True
import xlib
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'D'))

def rows(res):
    return [repr((r['rate'], r['torque'], r['integ'])) for r in res.samples]

def one(job):
    import gate, closed_loop as cl, option_d as od
    t, rule, P, seed = job
    out = os.path.join(HERE, 'tmp_out', str(os.getpid())); os.makedirs(out, exist_ok=True)
    mine = xlib.build(t, rule, P, noise=True, seed=seed)
    a = gate.run_gate(mine, xlib.spec(t)[0], out_dir=out)
    if rule == 'none':
        ref = cl.build(t, params={'AS_YAWRATE_P': P, 'AS_YAWRATE_I': 0.09}, noise=True, seed=seed)
    else:
        ref = od.build(t, {'kind': rule}, params={'AS_YAWRATE_P': P, 'AS_YAWRATE_I': 0.09}, noise=True, seed=seed)
    b = gate.run_gate(ref, xlib.spec(t)[0], out_dir=out)
    ra, rb = rows(a), rows(b)
    ndiff = sum(1 for x, y in zip(ra, rb) if x != y)
    return t, rule, P, seed, ra == rb, len(ra), len(rb), ndiff, mine.controller.ctrl.rc.n_release, a.results == b.results

if __name__ == '__main__':
    jobs = [(t, r, 0.7, 2) for t in xlib.T5 for r in ('none', 'restore', 'reset')]
    with mp.Pool(15) as pool:
        for x in pool.imap(one, jobs):
            print('%-45s %-8s P%.1f seed %d  rows %s (%d/%d, %d differ)  releases %d  verdicts %s' % (x[0], x[1], x[2], x[3], 'SAME' if x[4] else 'DIFF', x[5], x[6], x[7], x[8], 'SAME' if x[9] else 'DIFF'))
