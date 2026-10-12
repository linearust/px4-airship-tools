import os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.dont_write_bytecode = True
import xlib, gate
t = 'gz_airship_vectored_independent_am_fin_drag'
loop = xlib.build(t, 'restore', 0.7, seed=1)
rc = loop.controller.ctrl.rc
evs = []
orig = rc.update
cls_update = xlib.ReleaseRC.update
def upd(rate, sp, dt, landed=False):
    n = rc.n_release; d = rc.n_deflect
    out = cls_update(rc, rate, sp, dt, landed)
    if rc.n_release != n: evs.append(('release', round(loop.sim, 3), rc.release_log[-1]))
    if rc.n_deflect != d: evs.append(('deflect', round(loop.sim, 3), rc.trim))
    return out
rc.update = upd
res = gate.run_gate(loop, xlib.spec(t)[0], out_dir=os.path.join(HERE, 'tmp_out'))
ph = []
for r in res.samples:
    if not ph or ph[-1][0] != r['phase']: ph.append((r['phase'], r['sim']))
for e in evs: print(e)
for p in ph: print(p)
