#!/usr/bin/env python3
"""An airship's hull and tail, derived from one set of inputs: a HullSpec.

    python3 generic_hull_design.py                     # the generic airship (DEFAULT) and its checks
    python3 generic_hull_design.py --json              # the numbers the model files carry
    python3 generic_hull_design.py --spec vehicles/airship_2m.json [--json]

One hull: the prolate spheroid of fineness spec.fineness whose volume is
spec.hull_volume, drawn and collided alike, so buoyancy, added mass, the Munk
moment, the drag reference and the tail are sized on the same body (the
Cloudship's rule). The vehicle is neutrally buoyant: its mass is the air the
collision volumes displace, the pods' share set aside, and the hull axis sits
spec.hull_z above the link origin, the offset h of the added-mass and drag
blocks.

The tail: a cruciform of four rectangular fins of one chord, trailing edge
spec.te_margin short of the stern, roots on the hull at the mid-chord radius.
The ventral fin's span is what the ground allows (its tip spec.ground_margin
above the gondola floor, the ground contact); the dorsal and the two side fins
share one span, the shortest that meets R1. A shorter ventral fin is the usual
way a cruciform airship clears the ground.

Requirements, checked below:
  R1  yaw: the dorsal and ventral fins restore at least spec.stability_target
      x the hull's potential-flow Munk moment on the attached-flow slope at
      zero sideslip (Helmbold's), the one small-perturbation stability depends
      on. The plate slope the plugins carry is a secant to ~10 deg (its excess
      is vortex lift) and its ratio is reported, not judged.
  R2  pitch: the side fins against the Munk pitch moment on the same slope,
      the pendulum on top.
  R3  a full-rudder turn on the added-mass plants keeps both vertical fins at
      least spec.stall_margin_deg below their stall, transient included, from
      a sway-yaw simulation with the lift-drag plugin's semantics (stall on
      the geometric incidence, the flap added after). This sets the flap
      travel.
  G1  the ventral fin clears the ground at level attitude. A drawing
      constraint: the fins have no collision, so the simulation never
      touches, and a small nose-up on the ground already closes the margin.
      A real vehicle needs a stated ground-handling pitch.
Every number is an engineering estimate on a stated basis. DEFAULT is the
generic airship the committed family carries, a 7 m outdoor hull of the
Aero Drum 7 m RC blimp class (7.0 x 2.0 m, 14.66 m^3, 18 kg);
vehicles/airship_generic.json restates it. A HullSpec from a JSON file
describes another vehicle on the same basis (airship_family_gen.py), the
literature vehicles among them (vehicles/airship_aurora.json, ...).
"""
import dataclasses
import json
import math
import sys

import fin_basis as fb
import hull_basis

RHO, G = 1.225, 9.8
# The drag set: the reference design's dimensionless coefficients (an 8.3534 x
# 2.3693 m hull, fineness 3.5257) carried to any hull. The frontal reference
# radius scales with the half-height; the lateral term is crossflow drag, which
# scales with the planform, so it also takes the fineness ratio; the axial term
# takes Hoerner's fineness dependence of a streamlined body's frontal drag.
C_D, C_LAT, C_LP_LIN, C_LP_QUAD, C_MNR = 0.08906, 3.08324, 0.04, 0.18, 1.2
REF_RADIUS_PER_B = 1.16068 / 1.25
REF_HULL = (8.3534, 2.3693)
REF_FINENESS = REF_HULL[0] / REF_HULL[1]
ROT_SCALE = 0.003438
FINS = ('port', 'starboard', 'top', 'bottom')
# The hull the generic family carried until 2026-10-07: 39.2 m^3 at fineness
# 4.2, 10.97 m long. The turn speeds below and the pods' radii of gyration
# (airship_family_gen.write_pods) were set on it, and another hull's are
# scaled from its length, so they do not move when the generic hull does
SCALE_REFERENCE_LENGTH = 2.0 * 4.2 * (39.2 / (4.0 / 3.0 * math.pi * 4.2)) ** (1.0 / 3.0)


@dataclasses.dataclass(frozen=True)
class HullSpec:
    """Everything the hull and tail are sized from. Lengths m, masses kg.

    The link origin is the ground contact under the gondola: the gondola box
    stands on z = 0 and the hull axis is hull_z above it. The pods sit at
    +-pod_y on the plane z = pod_z, where the wrapper puts them."""
    hull_volume: float = 4.0 / 3.0 * math.pi * 3.5 * 1.0 * 1.0   # the 7.0 x 2.0 m spheroid
    fineness: float = 3.5
    hull_z: float = 1.30
    gondola: tuple = (0.5, 0.3, 0.3)
    gondola_z: float = 0.15
    gondola_mass: float = 11.37         # what neutral buoyancy leaves after the envelope at 0.1312 kg/m^2 and the fins
    pods_mass: float = 0.6           # the two pod links and their props together
    pod_y: float = 0.5
    pod_z: float = 0.25
    fin_areal: float = 0.5 / 1.4     # kg/m^2
    fin_thick: float = 0.02
    # the flap box is thicker than the fin box it overlaps, so the two are
    # never coplanar and a deflection shows in the GUI
    flap_thick: float = 0.025
    chord: float = 0.75
    te_margin: float = 0.19
    ground_margin: float = 0.05
    flap_fraction: float = 0.30
    stability_target: float = 1.2
    stall_margin_deg: float = 2.0
    pod_yaw: float = 10.0            # N m: the pods' full yaw couple, for the turn envelope

    @property
    def stall_margin(self):
        return math.radians(self.stall_margin_deg)

    @property
    def semi_axes(self):
        b = (self.hull_volume / (4.0 / 3.0 * math.pi * self.fineness)) ** (1.0 / 3.0)
        return self.fineness * b, b

    @property
    def gondola_volume(self):
        return self.gondola[0] * self.gondola[1] * self.gondola[2]

    @property
    def total_mass(self):
        """Neutral buoyancy: the air the collision volumes displace."""
        return RHO * (self.hull_volume + self.gondola_volume)

    @property
    def base_mass(self):
        """base_link's mass: hull, fins and gondola; the pods carry their own."""
        return self.total_mass - self.pods_mass

    @staticmethod
    def from_dict(values):
        """A spec from JSON: hull_length and hull_diameter may stand in for
        hull_volume and fineness; keys the dataclass lacks are ignored, so a
        vehicle file can carry notes and its propulsion beside the hull."""
        v = dict(values)
        if 'hull_length' in v and 'hull_diameter' in v:
            a, b = v.pop('hull_length') / 2.0, v.pop('hull_diameter') / 2.0
            v['hull_volume'] = 4.0 / 3.0 * math.pi * a * b * b
            v['fineness'] = a / b
        names = {f.name for f in dataclasses.fields(HullSpec)}
        kept = {k: (tuple(x) if isinstance(x, list) else x) for k, x in v.items() if k in names}
        return HullSpec(**kept)


DEFAULT = HullSpec()


def hoerner(f):
    return 3.0 * f + 4.5 / math.sqrt(f) + 21.0 / (f * f)


def spheroid_area(a, b):
    e = math.sqrt(1.0 - (b / a) ** 2)
    return 2.0 * math.pi * b * b * (1.0 + a / (b * e) * math.asin(e))


def restoring(fins, names, arm, key='cla'):
    """The fins' weathercock moment per (m/s)^2 of u v: 0.5 rho S cla l, summed."""
    return sum(0.5 * RHO * fins[n]['area'] * fins[n][key] * arm for n in names)


def fin_position(spec, n, x, radial):
    """A fin's centre: the cruciform about the hull axis, radial from it."""
    z = spec.hull_z
    return {'port': (x, radial, z), 'starboard': (x, -radial, z),
            'top': (x, 0.0, z + radial), 'bottom': (x, 0.0, z - radial)}[n]


def fin_box(n, chord, span, thick):
    """A fin's (or its flap's) box size: chord along x, span along its radial."""
    return (chord, span, thick) if n in ('port', 'starboard') else (chord, thick, span)


def fin(spec, c, span, root, flap_limit):
    ar = fb.joined_ar(c, span)
    cla = fb.cla(c, span, root)
    kwb = fb.k_wb(root, root + span)
    a_s = fb.stall(ar, kwb)
    cda = fb.induced_cda(cla, ar, a_s)
    cls, cds = fb.post_stall(cla, cda, a_s)
    return {'span': span, 'root': root, 'radial': root + span / 2.0, 'ar': ar, 'area': c * span, 'cla': cla,
            'cla_helmbold': fb.cla(c, span, root, fb.helmbold), 'cda': cda, 'k_wb': kwb, 'carryover': fb.carryover(root, root + span),
            'alpha_stall': a_s, 'cla_stall': cls, 'cda_stall': cds,
            'flap_slope': fb.flap_slope(c, span, root, spec.flap_fraction, delta_max=flap_limit),
            'mass': spec.fin_areal * c * span}


def design(spec=DEFAULT, flap_limit_deg=None):
    d = {'spec': dataclasses.asdict(spec)}
    a, b = spec.semi_axes
    d.update(a=a, b=b)
    k1, k2, kp = hull_basis.lamb(a, b)
    h = spec.hull_z
    d['added'] = hull_basis.added_mass_block(a, b, h, RHO * spec.hull_volume)
    d.update(k1=k1, k2=k2, kp=kp, munk=d['added']['yy'] - d['added']['xx'])
    c = spec.chord
    x_te = -(a - spec.te_margin)
    x_mid = x_te + c / 2.0
    x_cp = x_te + 0.75 * c
    root = fb.hull_radius(a, b, -x_mid)
    s_bot = math.floor((spec.hull_z - root - spec.ground_margin) * 100.0) / 100.0
    d.update(chord=c, x_te=x_te, x_mid=x_mid, x_cp=x_cp, root=root)
    # the arm, from a first centre of mass; refined below (it moves by mm)
    arm = abs(x_cp)
    target = spec.stability_target * d['munk']
    bottom = fin(spec, c, s_bot, root, None)
    s_big = s_bot
    while restoring({'top': fin(spec, c, s_big, root, None), 'bottom': bottom}, ('top', 'bottom'), arm, 'cla_helmbold') < target:
        s_big = round(s_big + 0.01, 2)
    d['spans'] = {'port': s_big, 'starboard': s_big, 'top': s_big, 'bottom': s_bot}
    limit = math.radians(flap_limit_deg) if flap_limit_deg else None
    d['fin'] = {n: fin(spec, c, d['spans'][n], root, limit) for n in FINS}
    d['flap_chord'] = spec.flap_fraction * c
    d['hinge_x'] = x_te + spec.flap_fraction * c
    # masses, the gondola station that puts the centre of mass over the centre of volume, the inertia
    m_fins = sum(f['mass'] for f in d['fin'].values())
    base_mass = spec.base_mass
    m_hull = base_mass - spec.gondola_mass - m_fins
    total = spec.total_mass
    v_g = spec.gondola_volume
    v_tot = spec.hull_volume + v_g
    x_g = -(m_fins * x_mid / total) / (spec.gondola_mass / total - v_g / v_tot)
    d.update(m_hull=m_hull, x_gondola=x_g, total=total)
    d['fin_pos'] = {n: fin_position(spec, n, x_mid, f['radial']) for n, f in d['fin'].items()}
    bodies = [(m_hull, (0.0, 0.0, spec.hull_z), (m_hull / 5 * 2 * b * b, m_hull / 5 * (a * a + b * b), m_hull / 5 * (a * a + b * b)))]
    gx, gy, gz = spec.gondola
    gm = spec.gondola_mass
    bodies.append((gm, (x_g, 0.0, spec.gondola_z), (gm / 12 * (gy * gy + gz * gz), gm / 12 * (gx * gx + gz * gz), gm / 12 * (gx * gx + gy * gy))))
    for n, f in d['fin'].items():
        sx, sy, sz = fin_box(n, c, f['span'], spec.fin_thick)
        m = f['mass']
        bodies.append((m, d['fin_pos'][n], (m / 12 * (sy * sy + sz * sz), m / 12 * (sx * sx + sz * sz), m / 12 * (sx * sx + sy * sy))))
    M = sum(m for m, _, _ in bodies)
    com = [sum(m * p[i] for m, p, _ in bodies) / M for i in range(3)]
    I = [[0.0] * 3 for _ in range(3)]
    for m, p, own in bodies:
        dd = [p[i] - com[i] for i in range(3)]
        for i in range(3):
            I[i][i] += own[i] + m * sum(dd[j] ** 2 for j in range(3) if j != i)
            for j in range(3):
                if i != j:
                    I[i][j] -= m * dd[i] * dd[j]
    d.update(base_com=com, inertia={'ixx': I[0][0], 'iyy': I[1][1], 'izz': I[2][2], 'ixy': I[0][1], 'ixz': I[0][2], 'iyz': I[1][2]}, base_mass=M)
    z_cov = (spec.hull_volume * spec.hull_z + v_g * spec.gondola_z) / v_tot
    com_all = [(base_mass * com[0]) / total, 0.0, (base_mass * com[2] + spec.pods_mass * spec.pod_z) / total]
    d.update(cov=(v_g * x_g / v_tot, 0.0, z_cov), com_all=com_all, stiffness=total * G * (z_cov - com_all[2]))
    d['arm'] = abs(x_cp - com_all[0])
    d['k_s'] = restoring(d['fin'], ('top', 'bottom'), d['arm'])
    d['k_s_helmbold'] = restoring(d['fin'], ('top', 'bottom'), d['arm'], 'cla_helmbold')
    d['k_p_helmbold'] = restoring(d['fin'], ('port', 'starboard'), d['arm'], 'cla_helmbold')
    d['ground_clearance'] = d['fin_pos']['bottom'][2] - d['fin']['bottom']['span'] / 2.0
    d['yaw_inertia'] = I[2][2] + spec.pods_mass * spec.pod_y ** 2
    r_ref = REF_RADIUS_PER_B * b
    area = math.pi * r_ref * r_ref
    surface, length = spheroid_area(a, b), 2.0 * a
    rot = 0.5 * RHO * surface * length ** 3 * ROT_SCALE
    lat = spec.fineness / REF_FINENESS
    axi = hoerner(spec.fineness) / hoerner(REF_FINENESS)
    xu, yv = -0.5 * RHO * area * C_D * axi, -0.5 * RHO * area * C_LAT * lat
    d['drag'] = {'xUabsU': xu, 'yVabsV': yv, 'zWabsW': yv, 'kP': -rot * C_LP_LIN, 'kPabsP': -rot * C_LP_QUAD,
                 'mQabsQ': -rot * C_MNR, 'nRabsR': -rot * C_MNR,
                 'xQabsU': 2 * h * xu, 'yPabsV': -2 * h * yv, 'mUabsU': h * xu, 'mQabsU': 2 * h * h * xu,
                 'kVabsV': -h * yv, 'kPabsV': 2 * h * h * yv}
    d.update(ref_radius=r_ref, ref_area=area, surface=surface, length=length, rot_scale=rot, lateral_factor=lat, axial_factor=axi)
    return d


def turn(d, u, added, drag, delta, seconds=150.0, dt=0.005, moment=0.0, start=(0.0, 0.0)):
    """Body-axis sway-yaw with u held: (m + m_y) v' + (m + m_x) u r = Y and
    (I + I_r) r' + (m_y - m_x) u v = N, v to starboard, r nose right. The two
    vertical fins at x_cp see v + r x_cp; the plugin's alpha is the flow angle
    in the fin's plane, cl stalled on it with the flap added after, cd as its
    absolute value. Returns (r, v, alpha, worst |alpha| / alpha_stall); worst is
    inf if the run went non-finite, so every caller's threshold fails."""
    m = d['total']
    mx = d['added']['xx'] if added else 0.0
    my = d['added']['yy'] if added else 0.0
    ir = d['added']['rr'] if added else 0.0
    I = d['yaw_inertia']
    yv = d['drag']['yVabsV'] if drag else 0.0
    nr = d['drag']['nRabsR'] if drag else 0.0
    x = d['x_cp'] - d['com_all'][0]
    fins = [d['fin'][n] for n in ('top', 'bottom')]
    v, r = start
    worst = 0.0
    for _ in range(int(seconds / dt)):
        vf = v + r * x
        vel = math.hypot(u, vf)
        alpha = math.atan2(vf, u)
        if not math.isfinite(alpha):      # max() keeps worst on NaN: fail closed
            return r, v, alpha, math.inf
        q = 0.5 * RHO * vel * vel
        Y = 0.0
        for f in fins:
            cl = fb.plugin_cl(alpha, f) + f['flap_slope'] * delta
            cd = fb.plugin_cd(alpha, f)
            Y += -cl * q * f['area'] * (u / vel) - cd * q * f['area'] * (vf / vel)
            worst = max(worst, abs(alpha) / f['alpha_stall'])
        N = Y * x + nr * r * abs(r) + moment
        Y += yv * v * abs(v)
        v += (Y - (m + mx) * u * r) / (m + my) * dt
        r += (N - (my - mx) * u * v) / (I + ir) * dt
    return r, v, alpha, worst


def turn_speeds(spec):
    """The forward speeds R3 and the envelope are judged at: 1.5, 3 and 6 m/s
    on the 10.97 m reference hull, scaled with the hull's length, so a
    smaller vehicle is judged at the speeds it flies."""
    scale = (2.0 * spec.semi_axes[0]) / SCALE_REFERENCE_LENGTH
    return tuple(round(u * scale, 2) for u in (1.5, 3.0, 6.0))


def flap_travel(spec=DEFAULT):
    """The largest whole-degree travel whose full-rudder turn keeps both
    vertical fins spec.stall_margin below stall on _am and _am_drag, transient
    included: both see one alpha, so the worst share of the lower stall is
    the largest |alpha| over the run."""
    best = None
    for deg in range(5, 26):
        dd = design(spec, deg)
        stall_min = min(dd['fin'][n]['alpha_stall'] for n in ('top', 'bottom'))
        ok = True
        for added, drag in ((True, False), (True, True)):
            for u in turn_speeds(spec):
                worst = turn(dd, u, added, drag, -math.radians(deg))[3]
                ok &= worst * stall_min <= stall_min - spec.stall_margin + 1e-9
        if ok:
            best = deg
        else:
            break
    return best


def final_design(spec=DEFAULT):
    """design() at the flap travel it resolves to, the travel in 'flap_limit_deg'."""
    travel = flap_travel(spec)
    d = design(spec, travel)
    d['flap_limit_deg'] = travel
    return d


def load_spec(path):
    """The HullSpec of a vehicle file: its 'hull' object, or the file's top level."""
    with open(path) as handle:
        values = json.load(handle)
    return HullSpec.from_dict(values.get('hull', values))


def report(spec, d):
    """The design in words, and whether it meets its requirements."""
    travel = d['flap_limit_deg']
    deg = math.degrees
    print('hull %.4f x %.4f, Lamb k1 %.5f k2 %.5f k\' %.5f, Munk %.3f' % (d['a'], d['b'], d['k1'], d['k2'], d['kp'], d['munk']))
    print('added: %s' % ', '.join('%s %.5f' % kv for kv in d['added'].items()))
    print('tail chord %.2f TE %.4f cp x %.4f root %.4f; hinge %.4f, flap chord %.2f, travel %s'
          % (d['chord'], d['x_te'], d['x_cp'], d['root'], d['hinge_x'], d['flap_chord'], 'none' if travel is None else '+-%d deg' % travel))
    for n in FINS:
        f = d['fin'][n]
        print('  %-9s span %.2f AR_j %.3f area %.3f cla %.3f (Helmbold %.3f) K %.3f K_W(B) %.3f cda %.4f stall %.4f (%.1f deg) cla_stall %.4f cda_stall %.4f flap %.4f mass %.4f'
              % (n, f['span'], f['ar'], f['area'], f['cla'], f['cla_helmbold'], f['carryover'], f['k_wb'], f['cda'], f['alpha_stall'], deg(f['alpha_stall']), f['cla_stall'], f['cda_stall'], f['flap_slope'], f['mass']))
    print('masses: hull %.4f gondola x %.4f; base CoM (%.4f, %.4f, %.4f); CoM (%.4f, %.4f); CoV (%.4f, %.4f)' % (d['m_hull'], d['x_gondola'], *d['base_com'], d['com_all'][0], d['com_all'][2], d['cov'][0], d['cov'][2]))
    print('inertia: %s; vehicle yaw inertia %.2f' % (', '.join('%s %.4f' % kv for kv in d['inertia'].items()), d['yaw_inertia']))
    print('drag (lateral x%.4f, axial x%.4f): %s' % (d['lateral_factor'], d['axial_factor'], ', '.join('%s %.4f' % kv for kv in d['drag'].items())))
    ok = True
    r1 = d['k_s_helmbold'] / d['munk']
    print('R1 yaw on Helmbold: %.2f / %.2f = %.3f (target %.1f); plate-slope secant %.3f  %s' % (d['k_s_helmbold'], d['munk'], r1, spec.stability_target, d['k_s'] / d['munk'], 'OK' if r1 >= spec.stability_target else 'FAIL'))
    ok &= r1 >= spec.stability_target
    r2 = d['k_p_helmbold'] / d['munk']
    print('R2 pitch on Helmbold: %.3f of Munk, pendulum %.0f N m/rad on top  %s' % (r2, d['stiffness'], 'OK' if r2 > 1.0 else 'FAIL'))
    ok &= r2 > 1.0
    print('G1 ventral tip %.3f m above ground  %s' % (d['ground_clearance'], 'OK' if d['ground_clearance'] >= spec.ground_margin - 1e-9 else 'FAIL'))
    ok &= d['ground_clearance'] >= spec.ground_margin - 1e-9
    ok &= travel is not None
    speeds = turn_speeds(spec)
    if travel is None:
        print('R3 no flap travel from 5 deg keeps both vertical fins %.1f deg below stall  FAIL' % spec.stall_margin_deg)
    else:
        for added, drag, name in ((True, False, '_am'), (True, True, '_am_drag'), (False, True, '_drag'), (False, False, 'rigid')):
            for u in speeds:
                r, v, alpha, worst = turn(d, u, added, drag, -math.radians(travel))
                print('R3 %-9s u %.1f: full rudder -> r %6.2f deg/s, sideslip %5.1f deg, fin alpha %5.2f deg (worst %.2f of stall)'
                      % (name, u, abs(deg(r)), deg(math.atan2(v, u)), abs(deg(alpha)), worst))
        u_ref = speeds[1] * 4.0 / 3.0
        print('rudder pair at %.1f m/s: %.1f N m; Munk at 5 deg sideslip %.1f N m'
              % (u_ref, sum(0.5 * RHO * u_ref ** 2 * d['fin'][n]['area'] * d['fin'][n]['flap_slope'] * math.radians(travel) * d['arm'] for n in ('top', 'bottom')),
                 d['munk'] * u_ref ** 2 * math.sin(math.radians(10)) / 2))
    # The vectored airship's envelope, u held: from rest, and from the stalled
    # branch reached at the lowest speed, to show the stall is hysteretic
    u_lo, u_hi = speeds[0], speeds[2]
    stalled = turn(d, speeds[0] * 4.0 / 3.0, True, True, 0.0, moment=spec.pod_yaw)
    threshold = None
    steps = 50
    for k in range(steps + 1):
        u = round(u_lo * 2.0 / 3.0 + (u_hi - u_lo * 2.0 / 3.0) * k / steps, 3)
        r, v, alpha, worst = turn(d, u, True, True, 0.0, moment=spec.pod_yaw)
        # attached: neither vertical fin past its own stall anywhere in the run,
        # the same test the '(stalled)' label uses
        if threshold is None and worst <= 1.0:
            threshold = u
        if k % 10 == 0:
            r2, v2, a2, w2 = turn(d, u, True, True, 0.0, moment=spec.pod_yaw, start=(stalled[1], stalled[0]))
            print('envelope _am_fin_drag u %.2f: full pod yaw from rest -> r %6.2f deg/s, fin alpha %5.1f deg%s; from the stalled branch -> r %6.2f deg/s, alpha %5.1f deg'
                  % (u, deg(r), abs(deg(alpha)), '  (stalled)' if worst > 1.0 else '', deg(r2), abs(deg(a2))))
    print('envelope: from rest both vertical fins stay attached from %s m/s up; once stalled they stay stalled with u held'
          % ('?' if threshold is None else '%.2f' % threshold))
    print('verdict:', 'all requirements met' if ok else 'REQUIREMENT FAILED')
    return ok


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    spec = DEFAULT
    if '--spec' in argv:
        spec = load_spec(argv[argv.index('--spec') + 1])
    d = final_design(spec)
    if '--json' in argv:
        print(json.dumps(d, indent=1))
        return 0 if d['flap_limit_deg'] is not None else 1
    return 0 if report(spec, d) else 1


if __name__ == '__main__':
    sys.exit(main())
