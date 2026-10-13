#!/usr/bin/env python3
"""Yaw plants of the four gz airship targets, turned in place about the centre of mass.

    python3 plants.py            # table + every parameter re-read from the SDFs and asserted

Each plant is   I * r_dot = N - c_hull * r|r| - N_fin(r)      [N m, rad/s, kg m^2]

r and N are the yaw rate and the yaw moment about the vertical through the
centre of mass, in ONE frame (both FRD like PX4, or both z-up like gz; the
plant is odd in r and N, so it does not care which). N is everything the
actuators (or a test wrench) apply; the actuator/allocator path is NOT in here.

Where every number comes from (paths relative to ~/dev/PX4-Autopilot at
airship/dev 4e1baea97c with the models submodule at 78389ea, the hull every
record under records/ and records_head/ was flown on; the tree since
2026-10-07 carries the 7 m hull, so build_from_sdf() is checked against
EXPECTED on first use, see PLANTS):

  I_rigid  gz_model.yaw_inertia(gz_model.family(model)): each link's own izz
           plus m d^2 about the family centre of mass (this repository's 
           gz_model.py:267-293, centre of mass :194-226). Link lines in SOURCES.
  rr       <fluid_added_mass><rr>, read by gz_model.added_mass (gz_model.py:261-264).
           The block has no term coupling r to anything (no pr, qr, yr, xr
           elements), so rr is the whole added yaw inertia, the same about any
           point on the vertical (the offset h is along z: hull_basis.py:182-186).
  c_hull   -<nRabsR> of the Hydrodynamics block. The plugin applies
           sum_jk (lin_ij + quadAbs_ijk |s_k|) s_j as a wrench at the link origin
           (gz_src/HydrodynamicsUtils.hh:98-110, gz_src/Hydrodynamics.cc:586-627);
           its n row has nRabsR and nothing else, so in yaw it is a pure couple
           -c_hull r|r|, the same about every point. No linear nR term exists.
           Its added-mass terms are off (disable_added_mass, no xDot* set) and its
           own Coriolis uses the plugin's zero Ma (Hydrodynamics.cc:572-576).
  N_fin    the vertical fins through the gz LiftDrag plugin (gz_src/LiftDrag.cc,
           tag gz-sim8_8.15.0 = the installed libgz-sim8 8.15.0):
             :294-300 velocity at cp = v_origin + w x cp
             :302-305 nothing if |v| <= 0.01 m/s
             :312-316 nothing if forward . v <= 0
             :385-393 alpha from the in-plane velocity, folded into +-90 deg
             :401-448 cl, cd with the stall branches, x cos^2(sweep)
             :506-508 force applied at cp (torque + cp x F at the link origin)
           In a turn in place about the centre of mass the velocity at a
           centre-line fin's cp is r * (x_cp - x_com) SIDEWAYS, i.e. along the
           fin's <upward> normal, and its forward component is exactly zero. So:
             - with no drift and no roll the plugin returns at :312 and the fin
               does nothing at all;
             - any positive forward component (surge drift, or the pitch-rate
               term q*z_cp = r*sin(phi)*z_cp that a roll angle phi gives) makes
               alpha ~ 90 deg, where cl = cla*as + cla_stall*(90deg - as) = 0 and
               cd = cda*as + cda_stall*(90deg - as) = 1.2 by construction
               (fin_basis.post_stall, fin_basis.py:109-112): pure broadside drag,
               N = -0.5 rho A 1.2 |x_cp - x_com|^3 r|r| per fin.
           So the fin term is a gate on the SIGN of a velocity that is ideally
           zero. A roll angle alone opens exactly one of the two vertical fins
           (z_cp above vs below the centre of mass); a surge drift opens both
           (forward) or neither (backward). Which one wins is decided by
           sub-mm/s quantities: the planar 3-DOF in coupling.py grows u from 0
           to 1e-5 m/s in 1 s and 6e-3 m/s in 4.5 s under the gate's 1 N m
           couple, purely from the centre of mass sitting off the added mass's
           centre, and then opens both. It cannot be predicted from the files.
           MEASURED, it is one: every open-loop record fits one fin broadside
           (validate.py): Cloudship _fin 42.14 deg/s vs 42.31 one fin / 38.66
           two / 46.94 none; Cloudship complete 19.87-19.93 vs 19.84 / 19.38 /
           20.33, and its 16-row time series fits one fin at rms 0.05 deg/s
           against 0.22-0.24 for none or two. So fins=1.0 is the nominal and
           0..2 the range to design across. With a sustained surge drift u
           both fins are on, or both off. The vectored pods' tilt slew makes
           one: inverting the rigid chain9 Manual phase (validate.py section 4)
           shows one pod thrusting alone for ~1.3 s after a 0.5 step (0.20-0.25
           of N_full delivered), i.e. ~9 N of net surge for ~1.3 s, an
           ESTIMATED ~0.2 m/s on 49-53 kg that the surge drag (xUabsU 0.2748)
           barely decays. Below about 1 deg/s the flow at the fin
           attaches and the moment becomes K_d u r, linear in r:
           fin_moment_plugin(r, u_fin) replicates the plugin for that case
           (self-test: 231.3 against added_mass_check's K_d 229.81).
           The horizontal fins (y_cp = +-y) see forward flow -r*y_cp on one side
           but at zero incidence: a0 = 0 and cda*alpha = 0 give no force; they
           are not in N_fin.
  pods/props  MulticopterMotorModel (gz_src/MulticopterMotorModel.cc:575-637):
           thrust = motorConstant w^2 with no inflow dependence, rotor drag
           -|w| rotorDragCoefficient v_perp (:632-637). In a turn in place the
           prop's velocity is along its axis (pods at tilt 0/180), v_perp =
           r*x_com: 2.4e-7 N m at 6.67 deg/s and full rotor speed; while a pod
           passes +-90 in a slew 0.013 N m (6.67 deg/s) to 0.053 N m (27.8
           deg/s), coupling.py C. The reaction couple (:655-656, 0.29 N m at
           18 N) is roll at tilt 0/180 and yaw only mid-slew. The Cloudship tail is a
           GenericMotorModel force_polynomial (PX4 src/modules/simulation/
           gz_plugins/generic_motor/GenericMotorModel.cpp:741-788): pure
           polynomial thrust, no drag, no flow dependence, torque polynomial 0
           (:458). Neither contributes a yaw damping term.

What is NOT in r_dot (all quantified in coupling.py): the roll/yaw product of
inertia (a collocated pole-zero pair at the roll frequency, >10x above the
current loop crossover), the Cloudship tail's sway (it is a force, not a
couple; the added sway mass at the hull centre changes r_dot by 0.5 %), the
hull drag cross terms (no n-row cross term; they act through sway and roll
only), the added-mass Coriolis yaw moment -(m_y - m_x) u v and its roll/pitch
cross products (second order in drift), and rotor drag.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = os.environ.get('AIRSHIP_BENCH', os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, BENCH)

import gz_model  # noqa: E402  pure: parses model.sdf, no simulator

M = 'Tools/simulation/gz/models'
DEG = math.pi / 180.0

# ---------------------------------------------------------------------------
# The fin plugin, replicated. gz_src/LiftDrag.cc line numbers in the comments.
# ---------------------------------------------------------------------------


def plugin_cl_cd(alpha, fin):
    """cl, cd the plugin applies at alpha (rad, already folded to +-pi/2), sweep 0."""
    a_s = fin['alpha_stall']

    if alpha > a_s:                                     # :401-408, :432-437
        cl = max(0.0, fin['cla'] * a_s + fin['cla_stall'] * (alpha - a_s))
        cd = fin['cda'] * a_s + fin['cda_stall'] * (alpha - a_s)
    elif alpha < -a_s:                                  # :409-416, :438-443
        cl = min(0.0, -fin['cla'] * a_s + fin['cla_stall'] * (alpha + a_s))
        cd = -fin['cda'] * a_s + fin['cda_stall'] * (alpha + a_s)
    else:                                               # :417-418, :444-445
        cl = fin['cla'] * alpha
        cd = fin['cda'] * alpha

    return cl, abs(cd)                                  # :448


def vertical_fin_force(u_f, v_f, fin):
    """(F_x, F_y) the plugin applies on a vertical fin (forward +x, upward +y,
    span z) whose cp moves at (u_f, v_f) in the body frame, air at rest.
    Zero exactly where the plugin returns early."""
    speed = math.hypot(u_f, v_f)

    if speed <= 0.01:                                   # :302-305
        return 0.0, 0.0

    if u_f <= 0.0:                                      # :312-316  forward . vel <= 0
        return 0.0, 0.0

    # :332 spanwise = forward x upward = z; :365 the in-plane velocity is (u_f, v_f)
    # :371 liftI = z x (u_f, v_f) = (-v_f, u_f)/speed; :378-379 cosAlpha = liftI . y
    cos_alpha = max(-1.0, min(1.0, u_f / speed))
    alpha = math.acos(cos_alpha) if -v_f >= 0.0 else -math.acos(cos_alpha)   # :385-387

    while abs(alpha) > 0.5 * math.pi:                   # :390-393
        alpha = alpha - math.pi if alpha > 0 else alpha + math.pi

    q = 0.5 * fin['rho'] * speed * speed                # :396-397
    cl, cd = plugin_cl_cd(alpha, fin)
    lift = cl * q * fin['area']                         # :428
    drag = cd * q * fin['area']                         # :451
    lx, ly = -v_f / speed, u_f / speed
    dx, dy = -u_f / speed, -v_f / speed                 # :368
    return lift * lx + drag * dx, lift * ly + drag * dy


def broadside_cd(fin):
    """cd at |alpha| = 90 deg, the stall branch at pi/2 (LiftDrag.cc:432-437)."""
    return abs(fin['cda'] * fin['alpha_stall'] + fin['cda_stall'] * (0.5 * math.pi - fin['alpha_stall']))


# ---------------------------------------------------------------------------
# The plants
# ---------------------------------------------------------------------------


class Plant:
    """One gz target's yaw axis in a turn in place about its centre of mass."""

    def __init__(self, target, model, I_rigid, rr, c_hull, fins, x_com, n_full, sources, measured):
        self.target = target
        self.model = model
        self.I_rigid = I_rigid
        self.rr = rr
        self.I = I_rigid + rr
        self.c_hull = c_hull
        self.fins = fins                  # vertical fins only, with 'arm' and 'c_bs'
        self.x_com = x_com
        self.n_full = n_full              # actuator side, for convenience only
        self.sources = sources
        self.measured = measured

    # -- fin terms ---------------------------------------------------------

    def c_fin_each(self):
        return [f['c_bs'] for f in self.fins]

    def c_fin_mean(self):
        return sum(self.c_fin_each()) / len(self.fins) if self.fins else 0.0

    def fin_moment_broadside(self, r, fins=1.0):
        """-fins * mean(c_bs) * r|r|, each fin silent below its 0.01 m/s floor."""
        if not self.fins or fins == 0.0:
            return 0.0

        live = [f['c_bs'] for f in self.fins if abs(r * f['arm']) > 0.01]

        if not live:
            return 0.0

        return -fins * (sum(live) / len(live)) * r * abs(r)

    def fin_moment_plugin(self, r, u_fin, phi=0.0):
        """Exact plugin sum over the vertical fins with a surge drift u_fin
        (m/s, body x) at the fins and a roll angle phi (rad): the fin's forward
        velocity is u_fin + r*sin(phi)*(z_cp - z_com) (the body pitch rate a roll
        angle gives a vertical-axis rotation), its sideways velocity r*arm."""
        n = 0.0

        for f in self.fins:
            u_f = u_fin + r * math.sin(phi) * f['dz']
            v_f = r * f['arm']
            _, fy = vertical_fin_force(u_f, v_f, f)
            n += f['arm'] * fy        # M_z = d_x F_y - d_y F_x, d_y = 0

        return n

    def fin_moment(self, r, fins=1.0, u_fin=None, phi=0.0):
        if u_fin is None:
            return self.fin_moment_broadside(r, fins)

        return self.fin_moment_plugin(r, u_fin, phi)

    # -- the plant ---------------------------------------------------------

    def damping(self, r, fins=1.0, u_fin=None, phi=0.0):
        """Total aerodynamic yaw moment at rate r (negative of the drag), N m."""
        return -self.c_hull * r * abs(r) + self.fin_moment(r, fins, u_fin, phi)

    def r_dot(self, r, N, fins=1.0, u_fin=None, phi=0.0, inertia=None):
        """rad/s^2. fins: number of vertical fins in broadside (0, 1 nominal, 2).
        u_fin: if given, the exact plugin with that surge drift replaces the
        broadside share. inertia: override I (e.g. self.I_measured())."""
        return (N + self.damping(r, fins, u_fin, phi)) / (inertia or self.I)

    def I_measured(self):
        m = self.measured
        return m['I_rigid'] + m.get('rr', 0.0)

    def params(self):
        return {'I_rigid': self.I_rigid, 'rr': self.rr, 'I': self.I, 'c_hull': self.c_hull,
                'c_fin_each': self.c_fin_each(), 'c_fin_nominal(1 fin)': self.c_fin_mean(),
                'x_com': self.x_com, 'N_full': self.n_full}


def integrate(plant, torque, r0=0.0, t_end=5.0, dt=0.001, **kw):
    """RK4 of r_dot under torque(t, r) -> N; returns [(t, r)]."""
    out = [(0.0, r0)]
    r, t = r0, 0.0

    while t < t_end - 1e-12:
        h = min(dt, t_end - t)
        k1 = plant.r_dot(r, torque(t, r), **kw)
        k2 = plant.r_dot(r + 0.5 * h * k1, torque(t + 0.5 * h, r + 0.5 * h * k1), **kw)
        k3 = plant.r_dot(r + 0.5 * h * k2, torque(t + 0.5 * h, r + 0.5 * h * k2), **kw)
        k4 = plant.r_dot(r + h * k3, torque(t + h, r + h * k3), **kw)
        r += h * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
        t += h
        out.append((t, r))

    return out


# ---------------------------------------------------------------------------
# The numbers, written out with their sources, then re-derived from the files
# by build_from_sdf() and asserted equal in check().
# ---------------------------------------------------------------------------

VI_RIGID_LINKS = (
    M + '/airship_base/model.sdf:56 (inertial pose 0.0067 0 0.8170), :57 (mass 48.0), :61 (izz 126.8963)',
    M + '/airship_vectored_independent/model.sdf:19,21,25 (pod_starboard pose 0 -1.5 0.8125, mass 0.4, izz 0.001)',
    M + '/airship_vectored_independent/model.sdf:55,57,61 (prop_starboard, 0.1 kg, izz 0.001)',
    M + '/airship_vectored_independent/model.sdf:92,94,98 (pod_port 0 1.5 0.8125, 0.4 kg, izz 0.001)',
    M + '/airship_vectored_independent/model.sdf:128,130,134 (prop_port, 0.1 kg, izz 0.001)',
)
VI_AM_LINKS = (
    M + '/airship_base_am/model.sdf:24 (inertial pose 0.0067 0 0.8170), :25 (mass 48.0), :29 (izz 126.8963)',
    M + '/airship_vectored_independent_am/model.sdf:18,20,24 / 54,56,60 / 91,93,97 / 127,129,133 (pods and props, as the rigid copy)',
)
CS_RIGID_LINKS = (
    M + '/cloudship/model.sdf:67 (inertial pose 0.0155 0 0.4183), :68 (mass 7.0148), :72 (izz 5.6703)',
    M + '/cloudship/model.sdf:298,300,304 (thruster_rod pose 0.32 0 0.05, 0.2 kg, izz 0.0135)',
    M + '/cloudship/model.sdf:336,338,342 and 373,375,379 (props at 0.32 -+0.45 0.05, 0.04 kg, izz 0.0005)',
    M + '/cloudship/model.sdf:421,423,427 (prop_tail -1.88 0 0.3352, 0.02 kg, izz 0.00004)',
)
CS_AM_LINKS = (
    M + '/cloudship_am/model.sdf:70 (inertial pose), :71 (mass 7.0148), :75 (izz 5.6703)',
    M + '/cloudship_am/model.sdf:331,333,337 / 369,371,375 / 406,408,412 / 454,456,460 (rod, props, tail, as the rigid copy)',
)

# Measured by added_mass_check.py in a probe world (1 N m for 2 s from rest,
# yaw angle parabola), sitl_out/chain5_20260925.txt:
# :19 vectored rigid 129.2818 (SDF 129.1503), :20 added 194.3086 (rr 192.2441),
# :31 Cloudship rigid 5.6922 (SDF 5.7986), :32 added 4.0866 (rr 3.9453).
MEAS_VI = {'I_rigid': 129.2818, 'rr': 194.3086, 'src': 'sitl_out/chain5_20260925.txt:19-20'}
MEAS_CS = {'I_rigid': 5.6922, 'rr': 4.0866, 'src': 'sitl_out/chain5_20260925.txt:31-32'}

# The four plants as numbers. build_from_sdf() re-reads each one.
EXPECTED = {
    'gz_airship_vectored_independent': dict(
        model='airship_vectored_independent', I_rigid=129.1503, rr=0.0, c_hull=0.0, x_com=0.00656,
        c_fin=(), n_full=54.0),
    'gz_airship_vectored_independent_am_fin_drag': dict(
        model='airship_vectored_independent_am_fin_drag', I_rigid=129.1503, rr=192.24415, c_hull=241.5613,
        x_com=0.00656, c_fin=(106.02, 89.28), n_full=54.0),
    'gz_cloudship': dict(
        model='cloudship', I_rigid=5.7986, rr=0.0, c_hull=0.0, x_com=0.02197, c_fin=(), n_full=4.478),
    'gz_cloudship_am_fin_drag': dict(
        model='cloudship_am_fin_drag', I_rigid=5.7986, rr=3.94525, c_hull=5.0873, x_com=0.02197,
        c_fin=(0.4993, 0.4993), n_full=4.478),
}

SOURCES = {
    'gz_airship_vectored_independent': {
        'I_rigid': 'gz_model.yaw_inertia(family) = 129.1503 over ' + '; '.join(VI_RIGID_LINKS)
                   + '; chain5 measured 129.2818',
        'rr': 'none: no <fluid_added_mass> (airship_base has no block; gz_model.added_mass -> {})',
        'c_hull': 'none: no Hydrodynamics plugin in the family',
        'fins': 'none: airship_base fins are visual only (' + M + '/airship_base/model.sdf:122-125)',
        'x_com': 'family centre of mass x 0.00656 (gz_model.centre_of_mass)',
        'N_full': '2 x 18 N x 1.5 m (' + M + '/airship_vectored_independent/model.sdf:196-197 '
                  'motorConstant 2e-4 x maxRotVelocity 300^2; :19,:92 y = -+1.5); '
                  'gz_model.yaw_authority; sitl_check.py:50',
    },
    'gz_airship_vectored_independent_am_fin_drag': {
        'I_rigid': 'as the rigid plant, family ' + '; '.join(VI_AM_LINKS),
        'rr': M + '/airship_base_am/model.sdf:56 <rr>192.24415</rr>; chain5 measured added 194.3086',
        'c_hull': M + '/airship_vectored_independent_am_fin_drag/model.sdf:26 <nRabsR>-241.5613</nRabsR>',
        'fins': M + '/airship_vectored_independent_am_fin/model.sdf: fin_top :80 rho 1.225, :81 cp x -4.2857 '
                'z 3.5268, :84 area 1.824, :87 cda 0.8201, :89 alpha_stall 0.1567, :91 cda_stall 0.7577; '
                'fin_bottom :97 rho, :98 cp x -4.2857 z 0.6932, :101 area 1.536, :104 cda 0.9805, '
                ':106 alpha_stall 0.1649, :108 cda_stall 0.7385; cd(90) = 1.2000 each; arm = -4.2857 - 0.00656',
        'x_com': 'family centre of mass x 0.00656',
        'N_full': 'as the rigid plant',
    },
    'gz_cloudship': {
        'I_rigid': 'gz_model.yaw_inertia(family) = 5.7986 over ' + '; '.join(CS_RIGID_LINKS)
                   + '; chain5 measured 5.6922',
        'rr': 'none: no <fluid_added_mass> in ' + M + '/cloudship/model.sdf',
        'c_hull': 'none: no Hydrodynamics plugin',
        'fins': 'none: visual only (' + M + '/cloudship/model.sdf:150-152)',
        'x_com': 'family centre of mass x 0.02197 (base 7.0148 kg at 0.0155, rod+props 0.28 kg at 0.32, tail 0.02 kg at -1.88)',
        'N_full': '2.3544 N x 1.902 m (' + M + '/cloudship/model.sdf:550-551 polynomial 2.3544 ref^2; '
                  ':421 tail x -1.88 vs x_com 0.02197); gz_model.yaw_authority 4.4780; sitl_check.py:51 uses 4.48',
    },
    'gz_cloudship_am_fin_drag': {
        'I_rigid': 'as the rigid plant, family ' + '; '.join(CS_AM_LINKS),
        'rr': M + '/cloudship_am/model.sdf:108 <rr>3.94525</rr>; chain5 measured added 4.0866',
        'c_hull': M + '/cloudship_am_fin_drag/model.sdf:30 <nRabsR>-5.0873</nRabsR>',
        'fins': M + '/cloudship_am_fin/model.sdf: fin_top :55 rho 1.225, :56 cp x -1.53125 z 1.6570, '
                ':59 area 0.18135, :62 cda 1.94, :64 alpha_stall 0.2822, :66 cda_stall 0.5064; fin_bottom '
                ':72-83 identical, cp z 0.1402 (:73); cd(90) = 1.2000; arm = -1.53125 - 0.02197',
        'x_com': 'family centre of mass x 0.02197',
        'N_full': 'as the rigid plant',
    },
}

FIN_NOTE = ('Models 78389ea (against its parent) changed the vectored fins cda 0.9133 -> 0.8201 and '
            'cda_stall 0.7474 -> 0.7577 (bottom 1.0379 -> 0.9805, 0.7318 -> 0.7385); cd(90 deg) is 1.2000 '
            'before and after, so the broadside term the gate runs of 2026-09-25 (chain5, before that commit) '
            'saw is the one here. It only matters for fin_moment_plugin at moderate alpha.')


def _fins_from_sdf(model_elem, com):
    fins = []

    for f in gz_model.lift_drag_fins(model_elem):
        if not f['vertical']:
            continue

        rho = float(f['elem'].find('air_density').text)
        rec = {'rho': rho, 'area': f['area'], 'cla': f['cla'], 'cda': f['cda'], 'alpha_stall': f['alpha_stall'],
               'cla_stall': f['cla_stall'], 'cda_stall': f['cda_stall'], 'x_cp': f['cp'][0], 'z_cp': f['cp'][2],
               'arm': f['cp'][0] - com[0], 'dz': f['cp'][2] - com[2],
               'name': 'dorsal' if f['cp'][2] > com[2] else 'ventral'}
        rec['cd90'] = broadside_cd(rec)
        rec['c_bs'] = 0.5 * rho * rec['area'] * rec['cd90'] * abs(rec['arm']) ** 3
        fins.append(rec)

    return fins


def build_from_sdf(target):
    """The plant as the model files say it, through gz_model."""
    e = EXPECTED[target]
    fam = gz_model.family(e['model'])
    merged = gz_model.merged(e['model'])
    _, com = gz_model.centre_of_mass(fam)
    I_rigid = gz_model.yaw_inertia(fam)
    rr = gz_model.added_mass(merged).get('rr', 0.0)
    hydro = gz_model.hydrodynamics(merged)
    n_terms = {k: v for k, v in hydro.items() if k.startswith('n')}

    if set(n_terms) - {'nRabsR'}:
        raise SystemExit('%s: the n row has more than nRabsR: %s' % (target, n_terms))

    c_hull = abs(hydro.get('nRabsR', 0.0))
    fins = _fins_from_sdf(merged, com)
    n_full = gz_model.yaw_authority(e['model'])
    meas = MEAS_VI if 'vectored' in target else MEAS_CS
    meas = dict(meas) if rr else {'I_rigid': meas['I_rigid'], 'src': meas['src']}
    return Plant(target, e['model'], I_rigid, rr, c_hull, fins, com[0], n_full, SOURCES[target], meas)


def check(plant):
    """The written-out numbers against the files: a model edit must not pass silently."""
    e = EXPECTED[plant.target]
    bad = []

    def close(a, b, tol):
        return abs(a - b) <= tol * max(1.0, abs(b))

    for key, tol in (('I_rigid', 1e-4), ('rr', 1e-6), ('c_hull', 1e-6), ('x_com', 1e-3), ('n_full', 1e-3)):
        if not close(getattr(plant, key), e[key], tol):
            bad.append('%s %s != %s' % (key, getattr(plant, key), e[key]))

    got = sorted(plant.c_fin_each(), reverse=True)
    want = sorted(e['c_fin'], reverse=True)

    if len(got) != len(want) or any(not close(a, b, 2e-3) for a, b in zip(got, want)):
        bad.append('c_fin %s != %s' % (got, want))

    for f in plant.fins:
        if abs(f['cd90'] - 1.2) > 1e-3:
            bad.append('%s cd90 %.4f' % (f['name'], f['cd90']))

    return bad


GYRO_NOISE = 0.0008726646          # rad/s, the IMU's gyro noise: cloudship/model.sdf:261, airship_base/model.sdf:236


class _Plants:
    """The anchored plants, built from the tree on first use and refused when
    the tree is not the hull the numbers were written out from (models
    78389ea): every importer used to get the live tree's plant at import,
    and a 7 m tree answered for 10.97 m records in silence."""

    def __init__(self):
        self._cache = {}

    def __getitem__(self, target):
        if target not in self._cache:
            p = build_from_sdf(target)
            bad = check(p)

            if bad:
                raise SystemExit('%s: the files in the tree are not the plant this model is anchored to (models 78389ea): %s'
                                 % (target, '; '.join(bad)))

            self._cache[target] = p

        return self._cache[target]

    def __contains__(self, target):
        return target in EXPECTED

    def __iter__(self):
        return iter(EXPECTED)

    def keys(self):
        return list(EXPECTED)

    def items(self):
        return [(t, self[t]) for t in EXPECTED]


PLANTS = _Plants()

ASSUMPTIONS = (
    'A1 turn in place: the rotation axis is the vertical through the family centre of mass; the pods (vectored) '
    'form a pure couple, the Cloudship tail is a force whose moment about the centre of mass is N (coupling.py '
    'B: the added sway mass at the hull centre changes r_dot per newton by 0.53 %).',
    'A2 small roll and pitch: r is both the body z rate the gate reads (vehicle_angular_velocity z) and the '
    'world yaw rate; the ixz roll coupling is a pole-zero pair at 2.7-3.9 rad/s, 10-30x above the P 0.5 crossover.',
    'A3 still air, no current (the Hydrodynamics current vector is 0, the world has no wind entity used here).',
    'A4 fins: nominal one vertical fin broadside (measured), range 0..2; drift-dependent via fin_moment_plugin.',
    'A5 gz plugin semantics read from gz-sim tag gz-sim8_8.15.0 sources (gz_src/, the installed libgz-sim8 is '
    '8.15.0); LiftDrag.cc, Hydrodynamics.cc, HydrodynamicsUtils.hh fetched here, MulticopterMotorModel.cc '
    'diffed identical to that tag.',
    'A6 the model files are models commit 78389ea (FIN_NOTE for the one change in that commit that touches a '
    'used number; it leaves cd(90) at 1.2).',
    'A7 SDF inertia (gz_model.yaw_inertia) is nominal; it ignores link rotations, which moves the Cloudship prop '
    'and tail izz by 4.2e-4 kg m^2 (0.007 %). The engine-measured inertias (chain5) are in I_measured(): '
    '+0.10 % vectored rigid, +0.68 % complete, -1.83 % Cloudship rigid, +0.36 % complete.',
    'A8 N excludes the actuator path: pod tilt slew, motor lag, THR_MDL_FAC, saturation, tail linearity are '
    'not in r_dot. n_full is carried for scale only.',
)


def get(target, fins=1.0, u_fin=None, measured=False):
    """(r_dot(r, N), params) for one target, with the fin case and inertia fixed.

    r in rad/s, N in N m, r_dot in rad/s^2, one frame for both."""
    p = PLANTS[target]
    inertia = p.I_measured() if measured else None

    def r_dot(r, N):
        return p.r_dot(r, N, fins=fins, u_fin=u_fin, inertia=inertia)

    params = dict(p.params(), fins=fins, u_fin=u_fin, I_used=inertia or p.I, sources=p.sources,
                  measured=p.measured)
    return r_dot, params


def main():
    """The table, and MISMATCH where the tree is not the anchor (exit 1)."""
    ok = True

    for t in EXPECTED:
        p = build_from_sdf(t)
        bad = check(p)
        ok &= not bad
        print('== %s  (model %s)%s' % (t, p.model, '' if not bad else '  MISMATCH: ' + '; '.join(bad)))
        print('   I = I_rigid %.4f + rr %.4f = %.4f kg m^2   (measured %.4f)'
              % (p.I_rigid, p.rr, p.I, p.I_measured()))
        print('   c_hull %.4f N m/(rad/s)^2 ; vertical fins broadside c = %s (nominal one fin %.4f)'
              % (p.c_hull, ', '.join('%.4f' % c for c in p.c_fin_each()) or '-', p.c_fin_mean()))

        for f in p.fins:
            print('     %-7s fin cp x %.4f z %.4f  arm %.4f  dz %+.4f  A %.5f  cd90 %.4f  c_bs %.4f  |v| floor at r = %.3f deg/s'
                  % (f['name'], f['x_cp'], f['z_cp'], f['arm'], f['dz'], f['area'], f['cd90'], f['c_bs'],
                     0.01 / abs(f['arm']) / DEG))

        print('   N_full %.4f N m (actuator side, for scale); open-loop 1/I %.5f rad/s^2 per N m'
              % (p.n_full, 1.0 / p.I))

        for rd in (1.0, 2.8, 6.67, 15.0, 27.8):
            r = rd * DEG
            print('     r %5.2f deg/s: hull %7.4f N m, fins x1 %7.4f, x2 %7.4f  (%5.2f %% / %5.2f %% of N_full)'
                  % (rd, p.c_hull * r * r, -p.fin_moment_broadside(r, 1.0), -p.fin_moment_broadside(r, 2.0),
                     100 * (p.c_hull * r * r - p.fin_moment_broadside(r, 1.0)) / p.n_full,
                     100 * (p.c_hull * r * r - p.fin_moment_broadside(r, 2.0)) / p.n_full))

        for k, v in p.sources.items():
            print('   [%s] %s' % (k, v))

    print('\n' + FIN_NOTE)
    print('\nAssumptions:')

    for a in ASSUMPTIONS:
        print('  ' + a)

    # the accessor, exercised: one second of 1 N m from rest on each plant
    print('\nget(): r after 1 s of 1 N m from rest [deg/s]')

    for t in PLANTS:
        rd, prm = get(t)
        r = 0.0

        for _ in range(1000):
            r += rd(r, 1.0) * 0.001

        print('  %-44s %.4f  (I %.4f)' % (t, r / DEG, prm['I_used']))

    print('\nall parameters re-read from the SDFs: %s' % ('OK' if ok else 'MISMATCH'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
