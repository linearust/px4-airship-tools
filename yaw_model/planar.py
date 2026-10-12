#!/usr/bin/env python3
"""Planar plants of a gz airship target, for the closed loop:

  PlanarPlant      surge u, sway v, yaw r            (closed_loop plant='planar')
  RollPlanarPlant  surge u, sway v, roll p, yaw r    (plant='roll', the default)

Both are plants.py's 1-DOF yaw axis plus the translation that the gate never
brings to rest, because the translation is what the added mass turns into yaw
moment and what the vertical fins' plugin gates on. The second adds roll,
which the rigid vectored hull's gyro reads through the roll/yaw product of
inertia. Every coefficient is read from the model files through gz_model;
nothing is fitted.

Why it is needed (closed_loop_out.txt, section 2): the gate's
clean_start() waits only for |yaw rate| < 0.5 deg/s (sitl_check.py:108-136);
it never waits for the hull to stop drifting. The Cloudship tail is a FORCE
(cloudship/model.sdf:421, +y at x -1.88), so every tail command also pushes the
hull sideways, and the complete Cloudship's surge drag is almost nil
(xUabsU -0.0946: cloudship_am_fin_drag/model.sdf:26), so a drift made in the
Manual and brake phases is still there 60 s later. A world-fixed drift V seen
from a hull turning at r is u = V cos(psi), v = -V sin(psi): the Munk moment
-(m_y - m_x) u v then swings with sin(2 psi), a period of 180 deg of heading
= 27 s at the gate's 6.67 deg/s (22 s at 8 deg/s). The vectored pods make
drift too: from rest, one pod thrusts forward at once while the other swings
through 90 deg (vectored_actuation regime A), a net forward impulse; and an
open fin's side force makes sway, which the yaw rate turns into surge.

Frame: gz body frame at the hull link origin O (x forward, y port, z up;
base_link pose 0 0 0: cloudship_am/model.sdf:66, airship_base_am/model.sdf:17),
r > 0 nose to port. PX4's FRD yaw rate is -r. The equations are Fossen's (2011)
3-DOF rigid-body and diagonal added-mass forms with y_g = 0, as in the plant
part's coupling.Planar (coupling.py:88-147), from which the terms are taken:

  M nu_dot = tau - C_RB(nu) nu - C_A(nu) nu + D(nu) + fins + actuators + wind
  M   = [[m + m_x, 0, 0], [0, m + m_y, m x_g], [0, m x_g, I_g + m x_g^2 + rr]]
  C_RB nu = [-m (x_g r + v) r, m u r, m x_g u r]
  C_A  nu = [-m_y v r, m_x u r, (m_y - m_x) u v]        (the Munk moment)
  D    = [xUabsU u|u|, yVabsV v|v|, nRabsR r|r|]         (coefficients < 0)

  m, x_g, I_g   gz_model.centre_of_mass / yaw_inertia of the family
  m_x, m_y, rr  <fluid_added_mass> xx, yy, rr (gz_model.added_mass): the
                engine folds the block into the link's spatial inertia at the
                link origin, Coriolis terms included (gz_src/Hydrodynamics.cc
                590-592: "DART implicitly computes C_A(v)*v"; the Munk moment
                was measured in the engine: bench sitl_out/chain9_20260927.txt:72,
                added_mass_check on airship_vectored_independent_am, K -38.45
                N m/(m/s)^2 against -(m_y - m_x) = -38.03)
  xUabsU, yVabsV, nRabsR  the Hydrodynamics block (gz_model.hydrodynamics),
                a wrench at the link origin from the velocity there
                (gz_src/HydrodynamicsUtils.hh:98-110, Hydrodynamics.cc:586-627)
  fins          plants.vertical_fin_force, the LiftDrag.cc replica, on the
                velocity at each vertical fin's cp: (u, v + r x_cp). Horizontal
                fins see zero incidence in the plane and give nothing.

Actuators, from the actuation parts (closed_loop.VectoredAct/CloudshipAct):
  Cloudship: the tail force F (+y) at x_tail (TailChain.tail_pos), which
             carries its own moment about O; the loop does not add K u again.
  vectored:  the pods' couple -N (FRD N -> FLU) and their net surge
             sum T cos(tilt) at x = 0 (pod poses x 0:
             airship_vectored_independent/model.sdf:19, :92), which has no
             moment about O beyond the couple already in N.

Left out, with the reason:
  roll and pitch: the xq/yp added-mass products, the drag cross terms and the
      roll angle, which gates the fins through u_fin = u + r sin(phi) dz
      (plants.fin_moment_plugin). Buoyancy stiffness puts roll at 1.6-2.3 s
      (coupling.py A); the tail's roll lever is 0.0688 m
      (cloudship_actuation.py stage 5). With any drift of a few mm/s the
      drift decides the fin gate; at exact rest (only the first wind check,
      before any actuator has pushed the hull) roll decides it and this plant
      gives both fins where the records show one (validate_out.txt section 1).
  heave from the pods' vertical thrust mid-swing: out of plane.
  The integrator: RK4 over each 4 ms step with the actuators held; gz's
  semi-implicit Euler differs at O(dt), against time constants of seconds.
"""
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import plants  # noqa: E402
import controller as ctl  # noqa: E402

gz_model = plants.gz_model
GYRO_NOISE = 0.0008726646          # cloudship/model.sdf:261, airship_base/model.sdf:236


class PlanarPlant:
    def __init__(self, target, noise=False, seed=1, fins_enabled=True, munk=True, pod_surge=True,
                 translation=True):
        """The last four switch terms OFF for the cause analysis only
        (closed_loop_out.txt): fins_enabled the fin plugin replica, munk the
        C_A yaw term -(m_y - m_x) u v, pod_surge the vectored pods' net surge
        force, translation all of u and v (then the fins act as plants.py's
        nominal one broadside fin, i.e. the 1-DOF plant)."""
        e = plants.EXPECTED[target]
        self.target = target
        self.p1 = plants.PLANTS[target]
        fam = gz_model.family(e['model'])
        merged = gz_model.merged(e['model'])
        self.m, com = gz_model.centre_of_mass(fam)
        self.xg = com[0]
        am = gz_model.added_mass(merged)
        self.mx, self.my, self.rr = am.get('xx', 0.0), am.get('yy', 0.0), am.get('rr', 0.0)
        self.Ig = gz_model.yaw_inertia(fam)
        hyd = gz_model.hydrodynamics(merged)
        self.X, self.Y, self.N = hyd.get('xUabsU', 0.0), hyd.get('yVabsV', 0.0), hyd.get('nRabsR', 0.0)
        self.fins = self.p1.fins if fins_enabled else []
        self.munk, self.pod_surge, self.translation = munk, pod_surge, translation
        m, xg = self.m, self.xg
        a11, a12, a22 = m + self.my, m * xg, self.Ig + m * xg * xg + self.rr
        det = a11 * a22 - a12 * a12
        self.inv0 = 1.0 / (m + self.mx)
        self.inv = (a22 / det, -a12 / det, a11 / det)
        self.nu = [0.0, 0.0, 0.0]          # u, v, r of O, FLU
        self.psi = 0.0                     # heading, world z-up [rad]
        self.pos = [0.0, 0.0]              # world x, y of O [m]
        self.lpf = ctl.LowPassFilter2p(ctl.LOOP_HZ, ctl.DEFAULT_PARAMS['IMU_GYRO_CUTOFF'])
        self.rng = random.Random(seed) if noise else None
        self.meas = 0.0
        self._act = None
        self.fin_open = [False] * len(self.fins)
        self.last_munk = 0.0

    # -- the gate's view: FRD --------------------------------------------------
    @property
    def rate(self):
        return -self.nu[2]

    def measured_rate(self):
        return self.meas

    # -- dynamics -----------------------------------------------------------------
    def _deriv(self, s, couple, fx, fy, x_f):
        u, v, r = s
        m, xg, mx, my = self.m, self.xg, self.mx, self.my
        tx = m * (xg * r + v) * r + my * v * r + self.X * u * abs(u) + fx
        ty = -m * u * r - mx * u * r + self.Y * v * abs(v) + fy
        tn = -m * xg * u * r + self.N * r * abs(r) + couple + x_f * fy

        if self.munk:
            tn -= (my - mx) * u * v

        if not self.translation:
            # the 1-DOF plant about the centre of mass, for the ablation only
            n_cm = couple + (x_f - xg) * fy + self.N * r * abs(r) + self.p1.fin_moment_broadside(r, 1.0)
            return (0.0, 0.0, n_cm / self.p1.I)

        for f in self.fins:
            ffx, ffy = plants.vertical_fin_force(u, v + r * f['x_cp'], f)
            tx += ffx
            ty += ffy
            tn += f['x_cp'] * ffy

        i11, i12, i22 = self.inv
        return (tx * self.inv0, i11 * ty + i12 * tn, i12 * ty + i22 * tn)

    def step_actuated(self, dt, actuation, moment_frd, wind_frd):
        """One gz step. moment_frd is the actuation's yaw moment about the
        centre of mass (used only for the pure-couple actuators)."""
        couple = -wind_frd                               # the gate's world-z couple, FLU
        fx = fy = x_f = 0.0

        if hasattr(actuation, 'chain'):                  # Cloudship tail: a force with its own lever
            _, fy = actuation.body_forces()
            x_f = actuation.chain.tail_pos[0]
            pods = getattr(actuation, 'pods', None)      # side motors: surge only (symmetric pair)
            fx = pods.wrench()[0] if pods is not None and self.pod_surge else 0.0
        else:                                            # vectored pods: a couple and a surge force
            couple += -moment_frd
            fx, _ = actuation.body_forces()
            fx = fx if self.pod_surge else 0.0

        s = self.nu
        k1 = self._deriv(s, couple, fx, fy, x_f)
        k2 = self._deriv([s[i] + 0.5 * dt * k1[i] for i in range(3)], couple, fx, fy, x_f)
        k3 = self._deriv([s[i] + 0.5 * dt * k2[i] for i in range(3)], couple, fx, fy, x_f)
        k4 = self._deriv([s[i] + dt * k3[i] for i in range(3)], couple, fx, fy, x_f)
        self.nu = [s[i] + dt * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6.0 for i in range(3)]
        u, v, r = self.nu
        c, sn = math.cos(self.psi), math.sin(self.psi)
        self.pos[0] += dt * (u * c - v * sn)
        self.pos[1] += dt * (u * sn + v * c)
        self.psi += dt * r
        self.fin_open = [u > 0.0 and math.hypot(u, v + r * f['x_cp']) > 0.01 for f in self.fins]
        self.last_munk = -(self.my - self.mx) * u * v
        g = -r + (self.rng.gauss(0.0, GYRO_NOISE) if self.rng else 0.0)
        self.meas = self.lpf.apply(g)

    def extra(self):
        u, v, r = self.nu
        c, sn = math.cos(self.psi), math.sin(self.psi)
        return {'u': u, 'v': v, 'psi_deg': math.degrees(self.psi),
                'Vx': u * c - v * sn, 'Vy': u * sn + v * c,
                'munk_frd': -self.last_munk, 'fins_open': sum(self.fin_open)}


# ---------------------------------------------------------------------------
# Surge, sway, ROLL and yaw
# ---------------------------------------------------------------------------

def _skew(a):
    return [[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm(a):
    n = math.sqrt(_dot(a, a))
    return (a[0] / n, a[1] / n, a[2] / n)


def _solve(A, b):
    """Gaussian elimination with partial pivoting, small dense systems."""
    n = len(b)
    M = [list(A[i]) + [b[i]] for i in range(n)]

    for c in range(n):
        piv = max(range(c, n), key=lambda i: abs(M[i][c]))
        M[c], M[piv] = M[piv], M[c]

        for i in range(c + 1, n):
            f = M[i][c] / M[c][c]
            for j in range(c, n + 1):
                M[i][j] -= f * M[c][j]

    x = [0.0] * n

    for i in range(n - 1, -1, -1):
        x[i] = (M[i][n] - sum(M[i][j] * x[j] for j in range(i + 1, n))) / M[i][i]

    return x


def lift_drag(vel, fin):
    """gz-sim 8.15.0 LiftDrag::Update (gz_src/LiftDrag.cc:262-508) in the body
    frame, no wind, no control joint, no radial symmetry, cma 0 (every fin
    here: cloudship_am_fin/model.sdf:63,67; airship_vectored_independent_am_fin
    /model.sdf:54,58,...). vel is the velocity at cp. Returns the force."""
    speed = math.sqrt(_dot(vel, vel))

    if speed <= 0.01:                                            # :302-305
        return (0.0, 0.0, 0.0)

    fwd, up = fin['forward'], fin['upward']

    if _dot(fwd, vel) <= 0.0:                                    # :312-316
        return (0.0, 0.0, 0.0)

    vel_i = (vel[0] / speed, vel[1] / speed, vel[2] / speed)
    span = _norm(_cross(fwd, up))                                # :332
    sin_sweep = max(-1.0, min(1.0, _dot(span, vel_i)))           # :337-338
    cos2 = 1.0 - sin_sweep * sin_sweep                           # :343
    vs = _dot(vel, span)
    in_plane = (vel[0] - vs * span[0], vel[1] - vs * span[1], vel[2] - vs * span[2])   # :365
    n_in = math.sqrt(_dot(in_plane, in_plane))

    if n_in == 0.0:
        return (0.0, 0.0, 0.0)

    drag_dir = (-in_plane[0] / n_in, -in_plane[1] / n_in, -in_plane[2] / n_in)          # :368
    lift_i = _norm(_cross(span, in_plane))                                               # :371
    cos_a = max(-1.0, min(1.0, _dot(lift_i, up)))                                        # :378-379
    a0 = fin['a0']
    alpha = a0 + math.acos(cos_a) if _dot(lift_i, fwd) >= 0.0 else a0 - math.acos(cos_a)  # :384-387

    while abs(alpha) > 0.5 * math.pi:                                                    # :390-393
        alpha = alpha - math.pi if alpha > 0 else alpha + math.pi

    q = 0.5 * fin['rho'] * n_in * n_in                                                   # :396-397
    a_s = fin['alpha_stall']

    if alpha > a_s:                                                                      # :400-419
        cl = max(0.0, (fin['cla'] * a_s + fin['cla_stall'] * (alpha - a_s)) * cos2)
        cd = (fin['cda'] * a_s + fin['cda_stall'] * (alpha - a_s)) * cos2
    elif alpha < -a_s:
        cl = min(0.0, (-fin['cla'] * a_s + fin['cla_stall'] * (alpha + a_s)) * cos2)
        cd = (-fin['cda'] * a_s + fin['cda_stall'] * (alpha + a_s)) * cos2
    else:
        cl = fin['cla'] * alpha * cos2
        cd = fin['cda'] * alpha * cos2

    cd = abs(cd)                                                                         # :448
    L = cl * q * fin['area']
    D = cd * q * fin['area']
    return tuple(L * lift_i[k] + D * drag_dir[k] for k in range(3))


def lift_drag_alpha(vel, fin):
    """The angle of attack lift_drag() works with (LiftDrag.cc:302-393: the
    same early returns, the same fold into +-90 deg), or None where it
    returns early. For reporting which branch of the curve a fin is on."""
    speed = math.sqrt(_dot(vel, vel))

    if speed <= 0.01 or _dot(fin['forward'], vel) <= 0.0:
        return None

    span = _norm(_cross(fin['forward'], fin['upward']))
    vs = _dot(vel, span)
    in_plane = (vel[0] - vs * span[0], vel[1] - vs * span[1], vel[2] - vs * span[2])

    if _dot(in_plane, in_plane) == 0.0:
        return None

    lift_i = _norm(_cross(span, in_plane))
    cos_a = max(-1.0, min(1.0, _dot(lift_i, fin['upward'])))
    alpha = fin['a0'] + math.acos(cos_a) if _dot(lift_i, fin['forward']) >= 0.0 else fin['a0'] - math.acos(cos_a)

    while abs(alpha) > 0.5 * math.pi:
        alpha = alpha - math.pi if alpha > 0 else alpha + math.pi

    return alpha


def fins_of(model_elem):
    """Every lift-drag plugin of the merged model (gz_model.lift_drag_fins),
    with forward/upward/cp as body vectors about the link origin."""
    out = []

    for f in gz_model.lift_drag_fins(model_elem):
        e = f['elem']
        vec = lambda tag: tuple(float(x) for x in e.find(tag).text.split())
        num = lambda tag, d: float(e.find(tag).text) if e.find(tag) is not None else d
        out.append({'cp': vec('cp'), 'forward': vec('forward'), 'upward': vec('upward'),
                    'rho': num('air_density', 1.2041), 'area': num('area', 1.0), 'a0': num('a0', 0.0),
                    'cla': num('cla', 1.0), 'cda': num('cda', 0.01), 'alpha_stall': num('alpha_stall', 0.5 * math.pi),
                    'cla_stall': num('cla_stall', 0.0), 'cda_stall': num('cda_stall', 1.0),
                    'vertical': f['vertical']})
    return out


class RollPlanarPlant:
    """Surge u, sway v, roll p and yaw r of the hull link origin O, plus the
    roll angle phi and the heading psi, with heave and pitch held (w = 0,
    theta = 0: buoyancy holds them far stiffer than anything here moves them).

    Adds to PlanarPlant, each from the files:
      rigid inertia at O: M_RB = [[m I, -m S(r_g)], [m S(r_g), I_O]] with
        I_g = [[Ixx, 0, ixz], [0, Iyy, 0], [ixz, 0, Izz]] about the family
        centre of mass (gz_model.inertia; ixz the SDF matrix element summed
        with the parallel-axis term, coupling.products) and
        I_O = I_g - m S(r_g)^2 (Fossen 2011 eq. 3.44);
      added mass at O: the whole <fluid_added_mass> block, xq and yp included;
      Coriolis of both, from the momentum (Kirchhoff): M nu_dot = tau -
        [w x P_l ; w x P_a + v x P_l], P = M nu (Fossen 2011 eq. 3.57 / 6.43),
        which is what the engine does with the folded-in added mass
        (gz_src/Hydrodynamics.cc:590-592);
      buoyancy stiffness: the lta world's uniform_fluid_density 1.225
        (worlds/lta.sdf:176) on base_link's collision volume at its centre of
        volume (gz_model.centre_of_volume); neutral (rho V = m to 1e-5), so the
        pair is a couple W (r_cov - r_com) x e_z(world), W = m g, g 9.8;
      the Hydrodynamics block's full damping rows for x, y, k, n with the
        velocity (u, v, 0, p, q, r) (gz_src/HydrodynamicsUtils.hh:74-114,
        applied at the link origin Hydrodynamics.cc:586-627; the names are
        row xyzkmn, column UVWPQR, "abs" + the |.| column: :305-335);
      every lift-drag plugin (horizontal fins too) through lift_drag() on
        the full velocity at cp, v_O + w x cp, with the body pitch rate a
        heading change has while rolled, q = r tan(phi) (theta held at 0),
        which is how a roll angle opens one vertical fin at rest;
      the actuators' roll: the pods' vertical thrust at y -+1.5, their
        reaction couple along the thrust axis (MulticopterMotorModel.cc:
        655-662) and the props' momentum (closed_loop.VectoredAct.pod_wrench);
        the Cloudship tail force at its link origin, z 0.3352 above O, i.e.
        0.0688 m below the centre of mass (cloudship/model.sdf:421);
      the gate's wrench, a world-z couple: body (0, tau sin phi, tau cos phi).
    The IMU reads the body z rate r (FRD -r), through the 40 Hz filter.
    """

    IDX = (0, 1, 3, 5)

    def __init__(self, target, noise=False, seed=1, model=None, munk=True, ixz=True, pod_roll=True,
                 fins_enabled=True, pod_surge=True, hold_u=None):
        """target: a gz target name; model: its gz model name when the
        target is not one of plants.EXPECTED's four (closed_loop.TARGETS).
        munk, ixz, pod_roll, fins_enabled, pod_surge switch single terms OFF
        for the cause analysis only (closed_loop_out.txt). hold_u [m/s]
        constrains the surge to that speed (u_dot = 0, whatever the forces),
        the reference review's fixed-speed model, for the ablation only;
        None leaves the surge free."""
        self.munk, self.pod_roll, self.pod_surge = munk, pod_roll, pod_surge
        self.hold_u = hold_u
        model = model or plants.EXPECTED[target]['model']
        self.target = target
        fam = gz_model.family(model)
        merged = gz_model.merged(model)
        m, com = gz_model.centre_of_mass(fam)
        vol, cov = gz_model.centre_of_volume(fam)
        self.m = m
        self.rg = com
        rho = 1.225                                  # worlds/lta.sdf:176
        self.W = m * gz_model.G
        self.buoy_ratio = rho * vol / m
        self.d_b = (cov[0] - com[0], cov[1] - com[1], cov[2] - com[2])
        ixx, iyy, izz = (gz_model.inertia(fam, a) for a in range(3))
        import coupling
        # ixz: True, False (0) or a number to scale it by (-1: the opposite
        # sign convention, a check of the SDF matrix-element reading)
        ixz = coupling.products(fam) * (1.0 if ixz is True else float(ixz))
        Ig = [[ixx, 0.0, ixz], [0.0, iyy, 0.0], [ixz, 0.0, izz]]
        S = _skew(com)
        S2 = [[sum(S[i][k] * S[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        IO = [[Ig[i][j] - m * S2[i][j] for j in range(3)] for i in range(3)]
        M = [[0.0] * 6 for _ in range(6)]

        for i in range(3):
            M[i][i] = m
            for j in range(3):
                M[i][3 + j] = -m * S[i][j]
                M[3 + i][j] = m * S[i][j]
                M[3 + i][3 + j] = IO[i][j]

        am = gz_model.added_mass(merged)
        names = 'xyzpqr'

        for key, val in am.items():
            i, j = names.index(key[0]), names.index(key[1])
            M[i][j] += val
            if i != j:
                M[j][i] += val

        self.M6 = M
        self.M4 = [[M[i][j] for j in self.IDX] for i in self.IDX]
        hyd = gz_model.hydrodynamics(merged)
        vel, mom = 'UVWPQR', 'xyzkmn'
        self.lin = {}
        self.qabs = {}

        for key, val in hyd.items():
            if 'abs' in key:
                a, b = key.split('abs')
                self.qabs[(mom.index(a[0]), vel.index(a[1]), vel.index(b))] = val
            elif key in ('disable_added_mass',):
                continue
            else:
                self.lin[(mom.index(key[0]), vel.index(key[1]))] = val

        self.fins = fins_of(merged) if fins_enabled else []

        for f in self.fins:                     # named as plants.py names them
            f['name'] = ('dorsal' if f['cp'][2] > com[2] else 'ventral') if f['vertical'] else \
                ('port' if f['cp'][1] > 0 else 'starboard')

        self.mxy = (am.get('xx', 0.0), am.get('yy', 0.0))
        self.nu = [0.0, 0.0, 0.0, 0.0]          # u, v, p, r (FLU, at O)
        if hold_u is not None:
            self.nu[0] = hold_u
        self.phi = 0.0
        self.psi = 0.0
        self.lpf = ctl.LowPassFilter2p(ctl.LOOP_HZ, ctl.DEFAULT_PARAMS['IMU_GYRO_CUTOFF'])
        self.rng = random.Random(seed) if noise else None
        self.meas = 0.0
        self.fin_open = []
        self._hx_prev = None
        self.info = {}

    @property
    def rate(self):
        return -self.nu[3]

    def measured_rate(self):
        return self.meas

    def _tau(self, s, phi, ext):
        """Generalized force at O on (x, y, z, k, m, n), FLU."""
        u, v, p, r = s
        q = r * math.tan(phi)
        full = (u, v, 0.0, p, q, r)
        tau = [0.0] * 6

        for (i, j), c in self.lin.items():
            tau[i] += c * full[j]

        for (i, j, k), c in self.qabs.items():
            tau[i] += c * abs(full[k]) * full[j]

        w = (p, q, r)
        opened = 0

        for f in self.fins:
            vcp = _cross(w, f['cp'])
            vel = (u + vcp[0], v + vcp[1], vcp[2])
            F = lift_drag(vel, f)

            if F != (0.0, 0.0, 0.0):
                opened += 1 if f['vertical'] else 0

            M = _cross(f['cp'], F)
            for k in range(3):
                tau[k] += F[k]
                tau[3 + k] += M[k]

        self._opened = opened
        sp, cp_ = math.sin(phi), math.cos(phi)
        ez = (0.0, sp, cp_)                                # world up in the body frame (theta 0)
        gb = _cross(self.d_b, (self.W * ez[0], self.W * ez[1], self.W * ez[2]))
        for k in range(3):
            tau[3 + k] += gb[k]

        for k in range(6):
            tau[k] += ext[k]

        # Coriolis from the momentum (w and q as above; heave w = 0)
        P = [sum(self.M6[i][j] * full[j] for j in range(6)) for i in range(6)]
        vl = (u, v, 0.0)
        c_l = _cross(w, P[:3])
        c_a = _cross(w, P[3:])
        c_av = _cross(vl, P[:3])
        C = [c_l[0], c_l[1], c_l[2], c_a[0] + c_av[0], c_a[1] + c_av[1], c_a[2] + c_av[2]]
        # q is not free: holding theta = 0 makes q = r tan(phi), so
        # q_dot = r_dot tan(phi) + r p / cos^2(phi). Its inertia terms M_i4 q_dot
        # move to the left (the r_dot part) and right (the rest) of each free
        # row; the pitch row's balance is the constraint moment. Without this,
        # a pitch rate would carry the centre of mass along (u_O stays 0 while
        # the CoM moves at q z_g), which is wrong exactly where the fins gate.
        self._qterm = (math.tan(phi), r * p / (math.cos(phi) ** 2))
        out = [tau[i] - C[i] - self.M6[i][4] * self._qterm[1] for i in self.IDX]

        if not self.munk:                                  # ablation: take -(m_y - m_x) u v back out
            out[3] += (self.mxy[1] - self.mxy[0]) * u * v

        return out

    def _deriv(self, s, phi, ext):
        rhs = self._tau(s, phi, ext)
        t = self._qterm[0]
        M = [[self.M4[a][b] + (self.M6[i][4] * t if b == 3 else 0.0) for b in range(4)]
             for a, i in enumerate(self.IDX)]

        if self.hold_u is not None:
            # u is a constraint, not a state: drop its row and column, so the
            # surge force becomes whatever holds it (the review's model)
            M = [row[1:] for row in M[1:]]
            acc = [0.0] + _solve(M, rhs[1:])
        else:
            acc = _solve(M, rhs)

        return acc, s[2]                                   # phi_dot = p (theta 0)

    def step_actuated(self, dt, actuation, moment_frd, wind_frd):
        tau_w = -wind_frd                                  # world z couple (gate.GZ_WORLD_Z_TO_FRD)
        ext = [0.0] * 6
        ext[4] += tau_w * math.sin(self.phi)
        ext[5] += tau_w * math.cos(self.phi)

        if hasattr(actuation, 'chain'):                    # Cloudship tail force at its link origin
            _, fy = actuation.body_forces()
            x, y, z = actuation.chain.tail_pos
            ext[1] += fy
            ext[3] += -z * fy
            ext[5] += x * fy
            pods = getattr(actuation, 'pods', None)        # the side motors, wrench at O (0 in the gate)
            if pods is not None:
                w = pods.wrench()
                for k in range(6):
                    ext[k] += w[k] if (k != 0 or self.pod_surge) else 0.0
        else:                                              # vectored pods
            ext_p = list(actuation.pod_wrench(dt))
            ext_p[3] = ext_p[3] if self.pod_roll else 0.0
            ext_p[0] = ext_p[0] if self.pod_surge else 0.0
            for k in range(6):
                ext[k] += ext_p[k]

        s, phi = self.nu, self.phi
        k1, d1 = self._deriv(s, phi, ext)
        s2 = [s[i] + 0.5 * dt * k1[i] for i in range(4)]
        k2, d2 = self._deriv(s2, phi + 0.5 * dt * d1, ext)
        s3 = [s[i] + 0.5 * dt * k2[i] for i in range(4)]
        k3, d3 = self._deriv(s3, phi + 0.5 * dt * d2, ext)
        s4 = [s[i] + dt * k3[i] for i in range(4)]
        k4, d4 = self._deriv(s4, phi + dt * d3, ext)
        self.nu = [s[i] + dt * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6.0 for i in range(4)]
        self.phi += dt * (d1 + 2 * d2 + 2 * d3 + d4) / 6.0
        self.psi += dt * self.nu[3] / math.cos(self.phi)
        self._tau(self.nu, self.phi, ext)                  # refresh the fin count for extra()
        g = -self.nu[3] + (self.rng.gauss(0.0, GYRO_NOISE) if self.rng else 0.0)
        self.meas = self.lpf.apply(g)

    def extra(self):
        u, v, p, r = self.nu
        c, sn = math.cos(self.psi), math.sin(self.psi)
        return {'u': u, 'v': v, 'p_deg': math.degrees(p), 'phi_deg': math.degrees(self.phi),
                'psi_deg': math.degrees(self.psi), 'Vx': u * c - v * sn, 'Vy': u * sn + v * c,
                'fins_open': self._opened, 'munk_frd': (self.mxy[1] - self.mxy[0]) * u * v}

    def set_state(self, u=0.0, v=0.0, p=0.0, r=0.0, phi=0.0, psi=0.0):
        """Body velocities at O (FLU: r > 0 nose to port), roll and heading.
        The gyro filter is left as it is: a state set mid-run is a jump the
        IMU would not see, so set it before the first step."""
        self.nu = [self.hold_u if self.hold_u is not None else u, v, p, r]
        self.phi, self.psi = phi, psi

    def fin_state(self):
        """[(name, alpha, alpha_stall)] of every vertical fin at the current
        state, alpha the plugin's own (lift_drag_alpha), None where the
        plugin applies nothing (flow from behind or below 0.01 m/s)."""
        u, v, p, r = self.nu
        w = (p, r * math.tan(self.phi), r)
        out = []

        for f in self.fins:
            if not f['vertical']:
                continue

            vcp = _cross(w, f['cp'])
            out.append((f['name'], lift_drag_alpha((u + vcp[0], v + vcp[1], vcp[2]), f), f['alpha_stall']))

        return out
