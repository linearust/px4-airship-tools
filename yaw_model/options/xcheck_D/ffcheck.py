"""Counter-check of 'only removing D(sp) at release does': baseline controller
(no release rule) with the existing RateControl feed-forward set (the model's
rc.ff; AAC never calls setFeedForwardGain today), vec complete + vec fin, P 0.7."""
import os, sys, multiprocessing as mp
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.dont_write_bytecode = True
import xlib, xgate
def one(job):
    import gate
    t, ff, P = job
    loop = xlib.build(t, 'none', P, seed=1)
    loop.controller.ctrl.set_param('AS_YAWRATE_FF', ff)   # rc.ff alone is reset by the next parameter update since 2026-10-01
    res = gate.run_gate(loop, xlib.spec(t)[0], out_dir=os.path.join(HERE, 'tmp_out', str(os.getpid())))
    m = xgate.my_metrics(res.samples, t, loop.controller.ctrl.rc)
    return t, ff, P, res.status, {k: ok for k, ok in res.results}, m
if __name__ == '__main__':
    jobs = [(t, ff, 0.7) for t in ('gz_airship_vectored_independent_am_fin_drag', 'gz_airship_vectored_independent_fin') for ff in (0.0, 0.4, 0.76)]
    with mp.Pool(6) as p:
        for t, ff, P, st, v, m in p.imap(one, jobs):
            print('%-44s ff %.2f P %.1f status %d  Arel %s settle %s end %.2f integ_end_push %.4f | Srel %s | peak %.2f | wOn %s wOff %s' % (
                t, ff, P, st, m['Arel']['ok'], m['Arel']['settle'], m['Arel']['end'], m['Atrack']['integ_end'], m['Srel']['ok'], m['Atrack']['peak'], m['wOn']['ok'], m['wOff']['ok']))
