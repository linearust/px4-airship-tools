#!/usr/bin/env python3
"""What the one-axis plant leaves out, and how much each piece is worth.

    python3 coupling.py

A. Roll/yaw through the rigid product of inertia ixz: a pole-zero pair at the
   roll frequency. Compared with the yaw-loop crossover at today's gains.
B. A planar 3-DOF model (surge u, sway v, yaw r of the link origin O) with the
   rigid body's CoM offset, the <fluid_added_mass> block's planar part and its
   Coriolis terms (the Munk moment), the hull drag at O, the vertical fins
   through the plugin replica, and either a pure couple (pods, the gate's wind)
   or the Cloudship's tail FORCE. Run against the 1-DOF plant in the scenarios
   the gate produces, to size the sway coupling and the drift-dependence of the
   fins.
C. Terms bounded by hand: the added-mass cross products with roll/pitch rates,
   the hull drag cross terms, rotor drag and reaction torques.

Frame: gz body frame, x forward, y port, z up; r > 0 is nose to port. The
3-DOF equations are Fossen's (2011) 3-DOF rigid-body and diagonal added-mass
Coriolis forms with y_g = 0, which hold in any right-handed body frame.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import controller as ctl  # noqa: E402
import plants  # noqa: E402

gz_model = plants.gz_model
DEG = math.pi / 180.0
G = gz_model.G                     # worlds/lta.sdf:12 <gravity>0 0 -9.8 (gz_model.py:35-42)

# The yaml default of AS_YAWRATE_P, for the crossover only. The rate loop's
# torque is normalized, torque = P * (rate_sp - rate) + integral with the rate
# in rad/s (src/lib/rate_control/rate_control.cpp; the airship controller
# sets only the z gains, AirshipYawRateLoop::setGains), and a normalized 1 is
# N_full at the hull, so the crossover of P against the pure inertia is
# N_full * P / I.
P_NOW = ctl.DEFAULT_PARAMS['AS_YAWRATE_P']

HULL_Z = {'airship_vectored_independent': 2.05,   # airship_base/model.sdf:69 hull collision pose z
          'cloudship': 0.8986}                     # cloudship/model.sdf:85 hull collision pose z


def part_a():
    print('== A. roll/yaw product of inertia: pole-zero pair against the loop crossover')
    print('   r/N = (Ixx s^2 + K)/(s [Izz (Ixx s^2 + K) - Ixz^2 s^2]): zero at sqrt(K/Ixx), pole at')
    print('   sqrt(K/(Ixx - Ixz^2/Izz)); zero below pole = collocated (torque and rate on one axis).')

    for target, p in plants.PLANTS.items():
        e = plants.EXPECTED[target]
        fam = gz_model.family(e['model'])
        mass, com = gz_model.centre_of_mass(fam)
        _, cov = gz_model.centre_of_volume(fam)
        k_roll = mass * G * (cov[2] - com[2])
        ixx = gz_model.inertia(fam, 0)
        am = gz_model.added_mass(gz_model.merged(e['model']))

        if am:
            h = HULL_Z[gz_model.vehicle(e['model'])] - com[2]
            my = am['yy']
            ixx = ixx + h * h * my - (h * my) ** 2 / (mass + my)   # free-sway roll inertia (added_mass_check.py docstring)

        ixz = gz_model.product_of_inertia(fam, 'xz')
        wz = math.sqrt(k_roll / ixx)
        wp = math.sqrt(k_roll / (ixx - ixz * ixz / p.I))
        wc = p.n_full * P_NOW / p.I
        dev = abs((1 - wc ** 2 / wz ** 2) / (1 - wc ** 2 / wp ** 2) - 1)
        print('   %-44s K %.1f N m/rad, Ixx %.3f, Ixz %.4f, Izz %.3f: zero %.3f, pole %.3f rad/s (period %.3f s), '
              'crossover(P %.1f) %.3f rad/s, |G| change there %.1e'
              % (target, k_roll, ixx, ixz, p.I, wz, wp, 2 * math.pi / wz, P_NOW, wc, dev))

    print('   Roll periods agree with added_mass_check (chain5:21 1.659/1.723 s, :22 2.297 s, :33 1.628 s, :34 1.886 s).')


class Planar:
    """3-DOF about the link origin O. State (u, v, r) of O."""

    def __init__(self, target):
        e = plants.EXPECTED[target]
        self.p = plants.PLANTS[target]
        fam = gz_model.family(e['model'])
        merged = gz_model.merged(e['model'])
        self.m, com = gz_model.centre_of_mass(fam)
        self.xg = com[0]
        am = gz_model.added_mass(merged)
        self.mx, self.my, self.rr = am.get('xx', 0.0), am.get('yy', 0.0), am.get('rr', 0.0)
        self.Ic = gz_model.yaw_inertia(fam)
        hyd = gz_model.hydrodynamics(merged)
        self.X, self.Y, self.Nr = hyd.get('xUabsU', 0.0), hyd.get('yVabsV', 0.0), hyd.get('nRabsR', 0.0)
        self.fins = self.p.fins
        m, xg = self.m, self.xg
        a = [[m + self.mx, 0.0, 0.0],
             [0.0, m + self.my, m * xg],
             [0.0, m * xg, self.Ic + m * xg * xg + self.rr]]
        det = a[1][1] * a[2][2] - a[1][2] * a[2][1]
        self.inv = (1.0 / a[0][0], a[2][2] / det, -a[1][2] / det, a[1][1] / det)

    def deriv(self, s, couple, force_y, x_force, fins_on=True):
        u, v, r = s
        m, xg, mx, my = self.m, self.xg, self.mx, self.my
        # C_RB nu = [-m(xg r + v) r, m u r, m xg u r];  C_A nu = [-my v r, mx u r, (my - mx) u v]
        tx = m * (xg * r + v) * r + my * v * r + self.X * u * abs(u)
        ty = -m * u * r - mx * u * r + self.Y * v * abs(v) + force_y
        tn = -m * xg * u * r - (my - mx) * u * v + self.Nr * r * abs(r) + couple + x_force * force_y

        if fins_on:
            for f in self.fins:
                fx, fy = plants.vertical_fin_force(u, v + r * f['x_cp'], f)
                tx += fx
                ty += fy
                tn += f['x_cp'] * fy

        i0, i11, i12, i22 = self.inv
        return (tx * i0, i11 * ty + i12 * tn, i12 * ty + i22 * tn)

    def run(self, s0, t_end, couple=0.0, force_y=0.0, x_force=0.0, dt=0.002):
        s = list(s0)
        t = 0.0
        peak_u = peak_v = 0.0
        munk = 0.0

        while t < t_end - 1e-12:
            k1 = self.deriv(s, couple, force_y, x_force)
            k2 = self.deriv([s[i] + 0.5 * dt * k1[i] for i in range(3)], couple, force_y, x_force)
            k3 = self.deriv([s[i] + 0.5 * dt * k2[i] for i in range(3)], couple, force_y, x_force)
            k4 = self.deriv([s[i] + dt * k3[i] for i in range(3)], couple, force_y, x_force)
            s = [s[i] + dt * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6.0 for i in range(3)]
            t += dt
            peak_u = max(peak_u, abs(s[0]))
            peak_v = max(peak_v, abs(s[1]))
            munk = max(munk, abs((self.my - self.mx) * s[0] * s[1]))

        return s, peak_u, peak_v, munk


def one_dof(p, r0, t_end, torque, fins):
    return plants.integrate(p, lambda t, r: torque, r0, t_end, 0.002, fins=fins)[-1][1]


def part_b():
    print('\n== B. planar 3-DOF (u, v, r of the link origin) against the 1-DOF plant')
    print('   rates in deg/s, z-up; "1-DOF f0/f1/f2" = broadside fin share 0/1/2. The 3-DOF has no roll, so its')
    print('   fins act only where the drift makes forward . v > 0 at the fin (LiftDrag.cc:312).')

    cases = []
    cs, vi = 'gz_cloudship_am_fin_drag', 'gz_airship_vectored_independent_am_fin_drag'

    # the gate's wind: 1 N m couple from rest for 4.75 s (validate.py)
    for t in (cs, vi):
        cases.append((t, 'wind 1 N m couple, 4.75 s, u0 0', (0.0, 0.0, 0.0), 4.75, 1.0, 0.0, 0.0))

    # the Cloudship's stick step: the tail FORCE (0.5 x 2.3544 N, +y) at x -1.88 from O, 3 s from rest.
    # The 1-DOF uses the same moment about the centre of mass, 0.5 x N_full.
    pc = plants.PLANTS[cs]
    x_tail = -1.88                                          # cloudship_am/model.sdf:454
    f_tail = 0.5 * 2.3544                                   # cloudship_am/model.sdf:583, linear-tail assumption
    cases.append((cs, 'tail force 1.177 N, 3 s from rest', (0.0, 0.0, 0.0), 3.0, 0.0, f_tail, x_tail))
    cases.append((cs, 'tail force 1.177 N, 20 s from rest', (0.0, 0.0, 0.0), 20.0, 0.0, f_tail, x_tail))

    # holds and releases at the gate's 6.67 deg/s with a surge drift already there
    for t, drifts, r0d in ((vi, (0.0, 0.02, 0.15), 6.67), (cs, (0.0, 0.02, 0.10), 6.67)):
        p = plants.PLANTS[t]
        r0 = r0d * DEG
        hold = p.c_hull * r0 * r0 + p.c_fin_mean() * r0 * r0

        for u0 in drifts:
            if t == cs:
                # the force whose moment about the centre of mass is the hold torque; run() flips its sign
                # so that it holds a positive r0
                f_hold = hold / abs(x_tail - p.x_com)
                cases.append((t, 'hold by tail %.4f N, 35 s, r0 %.2f, u0 %.2f' % (f_hold, r0d, u0),
                              (u0, 0.0, r0), 35.0, 0.0, f_hold, x_tail))
            else:
                cases.append((t, 'hold couple %.3f N m, 35 s, r0 %.2f, u0 %.2f' % (hold, r0d, u0),
                              (u0, 0.0, r0), 35.0, hold, 0.0, 0.0))

            cases.append((t, 'release N = 0, 12 s, r0 %.2f, u0 %.2f' % (r0d, u0), (u0, 0.0, r0), 12.0, 0.0, 0.0, 0.0))

    for t, label, s0, t_end, couple, fy, xf in cases:
        pl = Planar(t)
        p = pl.p

        if fy and xf:
            # z-up: a +y force behind O turns the nose to starboard (negative r); the 1-DOF moment about the
            # centre of mass is (x_f - x_com) F. The hold cases pass +F with r0 > 0, so flip the force.
            if s0[2] > 0:
                fy = -fy
            n1 = (xf - p.x_com) * fy
        else:
            n1 = couple

        (u, v, r), pu, pv, munk = pl.run(s0, t_end, couple, fy, xf)
        ref = [one_dof(p, s0[2], t_end, n1, k) / DEG for k in (0.0, 1.0, 2.0)]
        print('   %-10s %-52s 3-DOF r %8.3f | 1-DOF f0 %8.3f f1 %8.3f f2 %8.3f | end u %+.3f v %+.3f, '
              'peak |u| %.3f |v| %.3f m/s, peak Munk %.4f N m'
              % ('Cloudship' if 'cloud' in t else 'vectored', label, r / DEG, ref[0], ref[1], ref[2], u, v, pu, pv, munk))

    pc3 = Planar(cs)
    a_ratio = None
    m, xg, my = pc3.m, pc3.xg, pc3.my
    det = (m + my) * (pc3.Ic + m * xg * xg + pc3.rr) - (m * xg) ** 2
    rdot_3 = ((m + my) * x_tail - m * xg) / det
    rdot_1 = (x_tail - xg) / pc.I
    a_ratio = rdot_3 / rdot_1
    print('   Cloudship tail force from rest, r_dot per newton: 3-DOF %.5f vs 1-DOF %.5f (ratio %.4f): the added sway'
          % (rdot_3, rdot_1, a_ratio))
    print('   mass sits at the hull centre x = 0 (cloudship_am/model.sdf:118) and the centre of mass at %.5f.' % xg)


def part_c():
    print('\n== C. bounded by hand')
    vi = plants.PLANTS['gz_airship_vectored_independent_am_fin_drag']
    am_vi = gz_model.added_mass(gz_model.merged(vi.model))
    cs = plants.PLANTS['gz_cloudship_am_fin_drag']
    am_cs = gz_model.added_mass(gz_model.merged(cs.model))

    for name, am, p, u, v, pr, qr in (('vectored', am_vi, vi, 0.15, 0.05, 0.01, 0.01),
                                      ('Cloudship', am_cs, cs, 0.10, 0.05, 0.02, 0.02)):
        xq_yp = am['xq'] + am['yp']
        pp_qq = am['pp'] - am['qq']
        munk = (am['yy'] - am['xx']) * u * v
        cross = abs(xq_yp) * (pr * u + qr * v) + abs(pp_qq) * pr * qr
        r = 6.67 * DEG
        hold = p.c_hull * r * r + p.c_fin_mean() * r * r
        print('   %s: yaw part of the 6x6 added-mass Coriolis = -(m_y - m_x) u v - (xq + yp)(p u - q v) + (pp - qq) p q'
              % name)
        print('     at u %.2f v %.2f m/s, p q %.2f rad/s: Munk %.3f N m, cross products <= %.3f N m '
              '(xq + yp = %.3f, pp - qq = %.3f); the 6.67 deg/s hold torque is %.3f N m'
              % (u, v, pr, munk, cross, xq_yp, pp_qq, hold))

    print('   hull drag cross terms (xQabsU, yPabsV, mUabsU, mQabsU, kVabsV, kPabsV; drag SDF lines 35-40 / 37-42):')
    print('     all in the x, y, k, m rows; the n row is nRabsR alone (plants.build_from_sdf asserts it), so they')
    print('     reach yaw only through sway and roll, i.e. through the Munk and cross products above.')

    # rotor drag, MulticopterMotorModel.cc:632-637: F = -|w| lambda v_perp, w the real rotor speed
    lam = 8.06428e-05                                       # airship_vectored_independent_am/model.sdf:200
    w_full = 300.0                                          # :195 maxRotVelocity
    y_pod = 1.5                                             # :18, :91
    for rd in (6.67, 27.8):
        r = rd * DEG
        # tilt 0/180: the axis is x, v_perp is the sway r (x_pod - x_com) = -r x_com, arm -x_com: 2 w lam x_com^2 r
        tilt0 = 2 * w_full * lam * vi.x_com * vi.x_com * r
        # tilt +-90: the axis is z, v_perp includes the surge -+ r y_pod, arm y_pod: 2 w lam y_pod^2 r
        tilt90 = 2 * w_full * lam * y_pod * y_pod * r
        print('   vectored rotor drag at %.2f deg/s, full rotor speed: pods at 0/180 deg %.2e N m, at +-90 deg %.4f N m'
              % (rd, tilt0, tilt90))

    k_m = 0.016                                             # :197 momentConstant
    print('   vectored reaction couple, MulticopterMotorModel.cc:655-656: %.3f N m per pod at 18 N, about the prop'
          % (k_m * 18.0))
    print('     axis: roll at tilt 0/180 (the turn in place), yaw only while a pod passes +-90 in a slew (%.1f %% of 54 N m)'
          % (100 * k_m * 18.0 / 54.0))
    print('   Cloudship: rod props at zero collective in a turn in place give nothing; the tail is a GenericMotorModel')
    print('     force_polynomial (GenericMotorModel.cpp:741-788): no drag, torque polynomial zero (:458).')
    print('   thrust has no inflow term in either plugin (MulticopterMotorModel.cc:579-589; the airship_fixed flight')
    print('     check matches u(t) = u0 + a t to 0.3 %% at 3 m/s, sitl_out/chain9_20260927.txt:9).')


def part_d():
    print('\n== D. the vertical fins against surge drift at the fins (plugin replica, both fins see the drift)')
    vi = plants.PLANTS['gz_airship_vectored_independent_am_fin_drag']
    # self-test: small incidence, u 1 m/s, r 0.01 rad/s must give -K_d u r with K_d = 0.5 rho sum(A cla arm^2);
    # added_mass_check expects 229.8141 for these fins (sitl_out/chain5_20260925.txt:18)
    kd = sum(0.5 * f['rho'] * f['area'] * f['cla'] * f['arm'] ** 2 for f in vi.fins)
    got = -vi.fin_moment_plugin(0.01, 1.0) / 0.01
    print('   replica self-test (vectored, u 1 m/s, r 0.01 rad/s): %.2f N m/(rad/s) against K_d = %.2f (chain5:18 229.81)'
          % (got, kd))

    for t in ('gz_airship_vectored_independent_am_fin_drag', 'gz_cloudship_am_fin_drag'):
        p = plants.PLANTS[t]
        print('   %s: fin yaw moment [N m], hull for scale' % t)
        print('     %9s %9s %9s %9s %9s %9s %9s | %9s' % ('r deg/s', 'bs x1', 'bs x2', 'u 0.005', 'u 0.02', 'u 0.05',
                                                       'u 0.15', 'hull'))

        for rd in (0.5, 1.0, 2.8, 6.67, 15.0):
            r = rd * DEG
            row = [-p.fin_moment_broadside(r, 1.0), -p.fin_moment_broadside(r, 2.0)]
            row += [-p.fin_moment_plugin(r, u) for u in (0.005, 0.02, 0.05, 0.15)]
            print('     %9.2f %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f | %9.4f' % tuple([rd] + row + [p.c_hull * r * r]))

    print('   Reading: at the gate rates a forward drift of a few cm/s turns both fins on and the moment is about')
    print('   the two-fin broadside; below ~1 deg/s the flow at the fin attaches and the moment becomes K_d u r,')
    print('   linear in r and proportional to the drift, which the hull (quadratic) does not have. A backward drift')
    print('   switches both fins off (LiftDrag.cc:312).')


def main():
    part_a()
    part_b()
    part_c()
    part_d()
    return 0


if __name__ == '__main__':
    sys.exit(main())
