#!/usr/bin/env python3
"""Linear picture of setpoint weighting on the rigid hulls (G = K / (J s),
no actuator dynamics): the loop's poles J s^2 + K P s + K I = 0 do not
depend on b; the setpoint path's zero -I/(b P) does. Step (and, mirrored,
release) response of T_b = K (b P s + I) / (J s^2 + K P s + K I), and the
integrator's extra steady content P (1 - b) r_sp that a release must unwind.
Rigid inertia from gz_model (the gate's own izz); K = normalized-unit yaw
authority (54 N m vectored beyond the steer band, 4.478 N m Cloudship)."""
import math
PLANTS = [('vec rigid', 129.15, 54.0), ('cld rigid', 5.80, 4.478)]
SP = math.radians((0.5 - 0.1) / 0.9 * 15.0)
dt = 0.001
for name, J, K in PLANTS:
    for P, I in ((0.5, 0.09), (0.7, 0.09), (0.5, 0.2)):
        print('%s P %.2f I %.2f' % (name, P, I))
        for b in (1.0, 0.9, 0.8, 0.7, 0.6, 0.5):
            r = xi = 0.0
            ts, rs = [], []
            for k in range(int(80 / dt)):
                e = SP - r
                u = P * (b * SP - r) + xi
                xi += I * e * dt
                r += K * u / J * dt
                ts.append(k * dt); rs.append(r)
            pk = max(rs)
            t10 = next(t for t, x in zip(ts, rs) if x >= 0.1 * SP)
            t90 = next(t for t, x in zip(ts, rs) if x >= 0.9 * SP)
            out = [t for t, x in zip(ts, rs) if abs(x - SP) > 0.1 * SP]
            # release from steady state = mirror: time until |r| < 1 deg/s for good
            below = [t for t, x in zip(ts, rs) if abs(SP - x) > math.radians(1.0)]
            print('   b %.1f: overshoot %5.1f %%  rise10-90 %5.1f s  in +-10%% after %5.1f s  release: |r|<1 deg/s for good after %5.1f s;'
                  ' extra integrator at the release P(1-b)r_sp = %.4f' % (
                      b, 100 * (pk - SP) / SP, t90 - t10, out[-1], below[-1], P * (1 - b) * SP))
