"""The fin plant (2nd case) with a 60 deg/s tilt: Acro track reads ~10 deg/s
against 6.67 at every P/I/b. Trace the Acro push: rate, torque, integrator,
saturation flags the loop sees, pod modes and tilts."""
import math, os, sys
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import optc, gate
t = 'gz_airship_vectored_independent_fin'
loop = optc.build(t, 0.7, 0.09, 1.0, noise=True, seed=1, act=('slew', 60.0))
rows = []
orig = loop.advance
def adv(dt):
    orig(dt)
    rc = loop.controller.ctrl.rc
    st = loop.actuation.status()
    rows.append((loop.sim, loop._mode, loop._stick, math.degrees(loop.read_rate()), loop.read_torque(), rc.integ,
                 rc.sat_pos, rc.sat_neg, st['mode_s'], st['mode_p'], st['tilt_s'], st['tilt_p']))
loop.advance = adv
res = gate.run_gate(loop, optc.model_name(t), out_dir=os.path.join(optc.HERE, 'tmp_out', 'probe'))
ev = [e for e in res.events if 'mode' in e[2] or 'stick' in e[2]]
# the Acro push: the first stick 0.5 after the second-to-last 'mode acro' before the Stabilized block
push = [e for e in res.events if e[2] == 'stick 0.5']
t0 = push[1][1]   # second 0.5 stick = Acro push (the first is Manual)
last = -1
for r in rows:
    if r[0] >= t0 - 0.5 and r[0] <= t0 + 36 and r[0] - last >= 1.0:
        last = r[0]
        print('t %6.1f %-5s stick %.1f r %6.2f tq %+.4f integ %+.4f sat %d%d | %-8s %-8s tilt %7.1f %7.1f' % (
            r[0] - t0, r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9], r[10], r[11]))
print([v for v in res.verdicts if 'Acro' in v[0]])
