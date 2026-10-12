"""Why the rigid vectored hull holds 1.6 deg/s against a 2.5 deg/s stick
(beyond.jsonl, stick 0.25): trace torque, integrator, the saturation flags
the loop sees, pod modes and the delivered moment."""
import math, sys, os
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import optc
for b in (1.0,):
    loop = optc.build('gz_airship_vectored_independent', 0.5, 0.09, b, noise=False)
    loop.set_stick(0.0); loop.arm(); loop.set_mode('acro')
    loop.advance(2.0)
    loop.set_stick(0.25)
    t = 0.0
    for k in range(int(60 / 0.004)):
        loop.advance(0.004); t += 0.004
        if k % 250 == 0 and t > 20:
            rc = loop.controller.ctrl.rc
            st = loop.actuation.status()
            row = loop.actuation.row
            print('t %5.1f r %5.2f tq %.4f integ %.4f sat+ %d sat- %d | %s %s tilt %6.1f %6.1f | N %.3f T %.2f %.2f unalloc %+.0f' % (
                t, math.degrees(loop.read_rate()), loop.read_torque(), rc.integ, rc.sat_pos, rc.sat_neg,
                st['mode_s'][:4], st['mode_p'][:4], st['tilt_s'], st['tilt_p'], row['N'], row['T_s'], row['T_p'], loop._unallocated))
