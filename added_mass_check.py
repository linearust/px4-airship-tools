#!/usr/bin/env python3
"""Does the added mass do in the engine what the _am file says?

    python3 added_mass_check.py [--vehicle cloudship]   # gz only, no px4; never while a gz world is up

One vehicle of the scheme (cloudship, airship_vectored_independent, ...):
the bare model, its _fin, _am and _am_fin members, by name. The pushes and
torques scale with the vehicle's own mass and inertia so that every plant
sees the same accelerations.

Each model gets its own probe world (aero_moment_check.py's pattern): the
model alone at 30 m under buoyancy, a persistent wrench on base_link, and the
pose stream as the record. Two experiments, three models:

  yaw torque   1 N m for 2 s from rest. The yaw angle is a parabola and its
               curvature is the inertia. On cloudship that is the rigid yaw
               inertia the file implies; on cloudship_am it is that plus rr,
               the added yaw inertia, which is the engine folding the block
               into the spatial inertia at all.
  push         a force through the centre of mass (the wrench acts at the
               link origin, so a couple cancels the offset) for 3 s, forward
               with a little to port. Nothing here commands a yaw, so a yaw
               rate that grows is the plant's own. On cloudship it stays at
               zero. On cloudship_am the Munk moment turns the nose AWAY
               from the velocity, which is what destabilising means for a
               slender body (it wants to lie broadside to the flow), and
               r(t) against the integral of u v is a line whose slope is
               that coefficient over the yaw inertia. On cloudship_am_fin
               the two vertical fins take a fixed fraction of it back,
               since their moment is also proportional to u v while the
               sideslip stays small; the push is sized to keep it under the
               fins' 20 deg stall, and the sideslip reached is reported.
               On cloudship_fin (fins on the rigid hull) the same push
               isolates the fins: no Munk moment, so what turns the nose
               INTO the velocity is the weathercock, and what then damps
               the rate is the tail's own sideslip (see below: two
               regressors on every fin model).

  roll release  a small roll couple for a second, then cleared, and the free
               roll period from the pose stream. This is the experiment
               that touches the block's offset terms at all: rr and
               m_y - m_x are the same about any point on the yaw axis, so
               the yaw experiments cannot tell a block taken about the link
               origin (the spec) from one taken about the centre of mass.
               Roll can. The added sway mass sits at the hull's centre of
               volume; about the link origin that is h_c = z_cov - z_com
               above the centre of mass and the free roll inertia is
               I + h_c^2 m_y - (h_c m_y)^2 / (m + m_y); read about the
               centre of mass instead it would sit a full h above it, a
               period 25% longer. The rigid hull's period calibrates the
               stiffness, m g (z_cov - z_com).

The push's couple. The wrench acts at the link origin O and the centre of
mass is at C = O + d, so the force alone turns the hull by (O - C) x F =
-(d x F); cancelling that takes +d x F, in the world frame the plugin uses,
so d is rotated by the hull's trim attitude first. The second run of this
check applied -(d x F) and doubled the offset moment instead; the yaw
verdicts survived because a pitch couple against 35 N m/rad of buoyancy
stiffness barely reaches yaw, which is also why the rigid residual is not
zero and is reported rather than assumed.

The fins do two things and one regressor cannot tell them apart. A yaw
rate swings the tail sideways, so the fins see r x_cp of sideslip on top of
v, and that is yaw damping, proportional to u r rather than u v. A fit of r
against the integral of u v alone therefore reads the fins wrong in either
direction: on cloudship_am_fin the Munk moment builds a rate within the
push and the damping is credited to the static term (70% strong, the second
run); on the generic hull the fins' own moment builds the rate, the tail
weathercocks in I / (K_d u) = 0.3 s, and the same fit reads the static
term at a fifth of its value (the first generic run). So on the rigid
_fin model r is fitted against BOTH integrals, u v and u r, which separates
the static coefficient from the damping one; the rigid and _am models, where
no rate damping exists, keep the one regressor, since with nothing to damp
the two integrals are collinear and the pair is ill-conditioned. Refitting
the generic run's records this way gave 37.4 for the 38.9 expected and 155
for the 163 of damping, so it was the fit, not the plugin. On _am_fin the
pair is ill-conditioned again whenever the plant is statically unstable
(the Cloudship's fins restore half its Munk moment): r then grows with the
sideslip it makes, the two integrals rise together, and the free fit put
the Cloudship's damping at 6.2 for the 3.9 the rigid hull had just shown.
So there the damping is HELD at the rigid _fin measurement, the same fins
in the same air, and only the static term is fitted, on r corrected by it.

The sign. Fossen writes the added-mass Coriolis yaw term as (X_udot -
Y_vdot) u v on the left of the equation of motion, i.e. a moment
-(m_y - m_x) u v in a z-down frame: nose to port for a velocity to
starboard. gz's body frame here is x forward, y port, z up, where both v and
the yaw rate change sign, so the moment reads -(m_y - m_x) u v again and the
slope of r against the integral of u v is NEGATIVE for u, v > 0. The first
run of this check expected it positive and called a 3% match a failure.

Everything expected is read from the SDFs (the added-mass block, the fin
plugins, the link masses and poses), so the check is the file against the
engine, not a number typed twice.
"""
import math
import os
import subprocess
import sys
import tempfile
import time

import gz_link
import gz_model
from gz_model import GZ_PLUGINS, MODEL_HOMES, RHO_AIR
from report import note, summary, verdict

VEHICLE = sys.argv[sys.argv.index('--vehicle') + 1] if '--vehicle' in sys.argv else 'cloudship'
MODELS = tuple(VEHICLE + t for t in ('', '_fin', '_am', '_am_fin'))
TORQUE_MODELS = (VEHICLE, VEHICLE + '_am')
WORLD_NAME = 'am_probe_%d' % os.getpid()
WORLD = """<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="%(world)s">
    <physics name="p" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"/>
    <plugin filename="gz-sim-buoyancy-system" name="gz::sim::systems::Buoyancy">
      <uniform_fluid_density>%(rho)s</uniform_fluid_density>
      <enable>probe::base_link</enable>
    </plugin>
    <include><name>probe</name><uri>model://%(model)s</uri><pose>0 0 30 0 0 0</pose></include>
  </world>
</sdf>
"""
SETTLE_S = 8.0
BASELINE_S = 1.0
YAW_ACCEL = 1.0 / 5.7986        # rad/s^2: the Cloudship's 1 N m on its 5.80 kg m^2
YAW_S = 2.0
PUSH_ACCEL = (6.0 / 7.3148, 0.5 / 7.3148, 0.0)   # m/s^2: the Cloudship's 6 N / 0.5 N on 7.31 kg, forward with a little to port
ROLL_ACCEL = 2.0 / 2.3865       # rad/s^2: the Cloudship's 2 N m on its 2.39 kg m^2 of roll inertia
# No wait between the wrench and its clear: the two `gz topic` publishes are
# about 0.9 s apart on their own, a quarter to half a period on either hull,
# which leaves the largest swing. A 1 s wait made it 1.85 s, one added-mass
# roll period, and a step released one period in returns to rest: the hull
# read as immune to roll (0.2 deg) while responding exactly like the rigid one.
ROLL_KICK_S = 0.0
ROLL_MIN_SWING_DEG = 1.0        # below this the kick, not the plant, is what was measured
ROLL_S = 8.0                    # about four periods
PUSH_S = 2.5                    # short: the Munk moment itself turns the hull into more sideslip
RATE_HZ = 60                    # SceneBroadcaster's dynamic_pose rate
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sitl_out')


def probe(model, wrench, seconds, release_after=None):
    """(baseline rows, rows under the wrench), each [(t, (x,y,z), (w,x,y,z))]."""
    with tempfile.TemporaryDirectory() as tmp:
        world = os.path.join(tmp, 'probe.sdf')
        open(world, 'w').write(WORLD % {'world': WORLD_NAME, 'rho': RHO_AIR, 'model': model})
        env = dict(os.environ, GZ_SIM_RESOURCE_PATH=':'.join(MODEL_HOMES), GZ_IP='127.0.0.1',
                   GZ_SIM_SYSTEM_PLUGIN_PATH=GZ_PLUGINS)
        server = subprocess.Popen(['gz', 'sim', '-s', '-r', world], env=env,
                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            time.sleep(SETTLE_S)
            before = sample(env, BASELINE_S)

            # A publish that fails leaves the hull untouched and the record
            # reads as a plant that does not respond; one run fitted a yaw
            # inertia of 1e22 from it. Checked and retried, then given up
            # loudly.
            for attempt in range(3):
                sent = subprocess.run(['gz', 'topic', '-t', '/world/%s/wrench/persistent' % WORLD_NAME,
                                       '-m', 'gz.msgs.EntityWrench', '-p',
                                       'entity: {name: "probe::base_link", type: LINK}, wrench: {%s}' % wrench],
                                      env=env, capture_output=True, text=True, timeout=20)

                if sent.returncode == 0:
                    break

                note('  렌치 발행 실패 (%d): %s' % (attempt, sent.stderr.strip()[:80]))
                time.sleep(1)
            else:
                return None, None

            if release_after is not None:
                time.sleep(release_after)
                subprocess.run(['gz', 'topic', '-t', '/world/%s/wrench/clear' % WORLD_NAME,
                                '-m', 'gz.msgs.Entity', '-p', 'name: "probe::base_link", type: LINK'],
                               env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=20)

            rows = sample(env, seconds)
        except subprocess.TimeoutExpired:
            note('  gz 가 응답하지 않는다 (%s)' % model)
            return None, None
        finally:
            server.kill()
            server.wait()

    return before, rows


def sample(env, seconds):
    count = int(seconds * RATE_HZ)
    out = subprocess.run(['gz', 'topic', '-e', '-t', '/world/%s/dynamic_pose/info' % WORLD_NAME, '-n', str(count)],
                         env=env, capture_output=True, text=True, timeout=seconds * 3 + 30).stdout
    return gz_link.parse_poses(out, 'probe')


def yaw(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def unwrap(angles):
    out = [angles[0]]

    for a in angles[1:]:
        d = a - out[-1]
        out.append(out[-1] + (d - 2 * math.pi * round(d / (2 * math.pi))))

    return out


def roll(q):
    w, x, y, z = q
    return math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))


def to_world(q, v):
    """Body vector into the world frame: q v q*."""
    w, x, y, z = q
    t = [2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0])]
    return [v[i] + w * t[i] + [y * t[2] - z * t[1], z * t[0] - x * t[2], x * t[1] - y * t[0]][i]
            for i in range(3)]


def period(rows):
    """Free oscillation period of the roll angle, from its mean crossings."""
    ts = [r[0] for r in rows]
    phi = [roll(r[2]) for r in rows]
    mean = sum(phi) / len(phi)
    crossings = []

    for i in range(1, len(phi)):
        a, b = phi[i - 1] - mean, phi[i] - mean

        if a == 0 or (a < 0) == (b < 0):
            continue

        crossings.append(ts[i - 1] + (ts[i] - ts[i - 1]) * a / (a - b))

    if len(crossings) < 3:
        return float('nan'), len(crossings)

    return 2 * (crossings[-1] - crossings[0]) / (len(crossings) - 1), len(crossings)


def to_body(q, v):
    """World vector into the body frame: q* v q."""
    w, x, y, z = q
    cx, cy, cz = -x, -y, -z
    t = [2 * (cy * v[2] - cz * v[1]), 2 * (cz * v[0] - cx * v[2]), 2 * (cx * v[1] - cy * v[0])]
    return [v[i] + w * t[i] + [cy * t[2] - cz * t[1], cz * t[0] - cx * t[2], cx * t[1] - cy * t[0]][i]
            for i in range(3)]


def kinematics(rows):
    """[(t, u, v, r)] by central differences, body frame."""
    ts = [r[0] for r in rows]
    psi = unwrap([yaw(r[2]) for r in rows])
    out = []

    for i in range(1, len(rows) - 1):
        dt = ts[i + 1] - ts[i - 1]

        if dt <= 0:
            continue

        vw = [(rows[i + 1][1][k] - rows[i - 1][1][k]) / dt for k in range(3)]
        u, v, _ = to_body(rows[i][2], vw)
        out.append((ts[i], u, v, (psi[i + 1] - psi[i - 1]) / dt))

    return out


def fit_line(xs, ys):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx if sxx > 0 else float('nan')
    return slope, my - slope * mx


def yaw_inertia(rows, torque):
    """Curvature of the yaw angle under a constant torque: I = tau / psi''."""
    t0 = rows[0][0]
    ts = [r[0] - t0 for r in rows]
    psi = unwrap([yaw(r[2]) for r in rows])
    # least squares for psi = c0 + c1 t + c2 t^2
    n = len(ts)
    S = [sum(t ** k for t in ts) for k in range(5)]
    Sy = [sum(p * t ** k for p, t in zip(psi, ts)) for k in range(3)]
    A = [[S[0], S[1], S[2]], [S[1], S[2], S[3]], [S[2], S[3], S[4]]]
    b = Sy[:]
    for i in range(3):                              # gaussian elimination
        piv = max(range(i, 3), key=lambda r: abs(A[r][i]))
        A[i], A[piv], b[i], b[piv] = A[piv], A[i], b[piv], b[i]
        for r in range(i + 1, 3):
            f = A[r][i] / A[i][i]
            for c in range(i, 3):
                A[r][c] -= f * A[i][c]
            b[r] -= f * b[i]
    c = [0.0] * 3
    for i in (2, 1, 0):
        c[i] = (b[i] - sum(A[i][j] * c[j] for j in range(i + 1, 3))) / A[i][i]
    return torque / (2 * c[2]) if c[2] != 0 else float('nan'), c[1]


def fit_plane(x1, x2, ys):
    """Least squares y = a x1 + b x2 + c by the normal equations."""
    n = len(ys)
    cols = (x1, x2, [1.0] * n)
    M = [[sum(p * q for p, q in zip(cols[i], cols[j])) for j in range(3)] for i in range(3)]
    b = [sum(p * y for p, y in zip(cols[i], ys)) for i in range(3)]

    for i in range(3):
        pivot = M[i][i]

        if abs(pivot) < 1e-12:
            return float('nan'), float('nan'), float('nan')

        M[i] = [m / pivot for m in M[i]]
        b[i] /= pivot

        for k in range(3):
            if k != i:
                f = M[k][i]
                M[k] = [mk - f * mi for mk, mi in zip(M[k], M[i])]
                b[k] -= f * b[i]

    return tuple(b)


def yaw_fit(kin, damped, damping_known=None):
    """I r' = K_s u v - K_d u r integrated: r against the integral of u v and,
    when the model has fins, of u r as well. With `damping_known` (K_d / I)
    the second regressor is subtracted instead of fitted. Takes the
    kinematics. Returns (K_s / I, K_d / I or None, intercept, max |u v|)."""
    X1, X2, R, uv, ur = [], [], [], 0.0, 0.0

    for i, (t, u, v, r) in enumerate(kin):
        if i > 0:
            t0, u0, v0, r0 = kin[i - 1]
            uv += 0.5 * (u * v + u0 * v0) * (t - t0)
            ur += 0.5 * (u * r + u0 * r0) * (t - t0)

        X1.append(uv)
        X2.append(ur)
        R.append(r)

    uvmax = max(abs(u * v) for _, u, v, _ in kin)

    if damping_known is not None:
        slope, icpt = fit_line(X1, [r + damping_known * x2 for r, x2 in zip(R, X2)])
        return slope, damping_known, icpt, uvmax

    if damped:
        a, b, icpt = fit_plane(X1, X2, R)
        return a, -b, icpt, uvmax

    slope, icpt = fit_line(X1, R)
    return slope, None, icpt, uvmax


def still(before):
    kin = kinematics(before)
    speed = max(math.hypot(u, v) for _, u, v, _ in kin)
    rate = max(abs(r) for _, _, _, r in kin)
    return speed < 0.05 and rate < 0.02, speed, rate


def expected():
    """From the SDFs: rigid yaw inertia about the vehicle's centre of mass,
    the added yaw inertia and Munk coefficient of the _am block, and the two
    vertical fins' moment coefficient per unit u v."""
    am = gz_model.merged(VEHICLE + '_am')
    family = gz_model.family(VEHICLE)
    mass, com = gz_model.centre_of_mass(family)
    izz = gz_model.yaw_inertia(family)
    block = gz_model.added_mass(am)
    vertical = [f for f in gz_model.lift_drag_fins(gz_model.merged(VEHICLE + '_fin')) if f['vertical']]
    fin_k = sum(0.5 * RHO_AIR * f['area'] * f['cla'] * abs(f['cp'][0] - com[0]) for f in vertical)
    arm = abs(vertical[-1]['cp'][0] - com[0]) if vertical else 0.0

    # Roll: the pendulum stiffness is m g (z_cov - z_com) with the centre of
    # volume from the buoyancy collisions, and the rigid roll inertia every
    # link's ixx plus its mass at its lever, both about the centre of mass.
    base = gz_model.merged(VEHICLE)
    z_cov = gz_model.centre_of_volume(family)[1][2]
    ixx = gz_model.inertia(family, 0)
    stiffness = mass * gz_model.G * (z_cov - com[2])
    h_c = z_cov - com[2]                       # the block about the link origin: sway mass at the CoV
    h_o = gz_model.pose(gz_model.named(base.find('link'), 'collision', 'hull_collision'), 2)
    m_y = block['yy']
    roll_rigid = 2 * math.pi * math.sqrt(ixx / stiffness)
    roll_origin = 2 * math.pi * math.sqrt((ixx + h_c * h_c * m_y - (h_c * m_y) ** 2 / (mass + m_y)) / stiffness)
    roll_com = 2 * math.pi * math.sqrt((ixx + h_o * h_o * m_y - (h_o * m_y) ** 2 / (mass + m_y)) / stiffness)
    return {'izz': izz, 'rr': block['rr'], 'munk': block['yy'] - block['xx'], 'fin': fin_k,
            'fin_damping': fin_k * arm, 'com': com, 'mass': mass, 'ixx': ixx, 'stiffness': stiffness,
            'roll_rigid': roll_rigid, 'roll_origin': roll_origin, 'roll_com': roll_com,
            'yaw_torque': YAW_ACCEL * izz, 'roll_torque': ROLL_ACCEL * ixx,
            'push': tuple(a * mass for a in PUSH_ACCEL)}


def main():
    # Anchored: an unanchored pattern matches its own launcher's text.
    if subprocess.run(['pgrep', '-f', '^gz sim'], capture_output=True).returncode == 0:
        note('gz 월드가 이미 떠 있다. 이 검사는 자기 월드를 띄운다')
        return 2

    missing = [m for m in MODELS if not gz_model.has_model(m)]

    if missing:
        # airship_fixed carries its fins as actuators and has no _fin member,
        # so this check's four-plant design does not apply to it; its fins
        # are measured by the flight check instead
        note('%s 에는 이 검사의 네 플랜트가 다 있지 않다: %s 없음' % (VEHICLE, ', '.join(missing)))
        return 2

    want = expected()
    note('기대: 강체 요 관성 %.4f, 부가 요 관성 rr %.4f, Munk 계수 %.4f, 수직 핀 정적 계수 %.4f (비 %.1f), 핀 감쇠 계수 %.4f, 무게중심 (%.4f, 0, %.4f)'
         % (want['izz'], want['rr'], want['munk'], want['fin'], want['munk'] / want['fin'], want['fin_damping'],
            want['com'][0], want['com'][2]))
    os.makedirs(OUT, exist_ok=True)
    inertia, slope, damping, sideslip, rolls, kins = {}, {}, {}, {}, {}, {}
    note('      롤 주기 예측: 강체 %.3f s, 부가 질량이 링크 원점 기준이면 %.3f s, 무게중심 기준으로 읽혔다면 %.3f s (강성 %.2f N m/rad, 강체 롤 관성 %.4f)'
         % (want['roll_rigid'], want['roll_origin'], want['roll_com'], want['stiffness'], want['ixx']))
    F = want['push']
    note('      가진: 요 토크 %.2f N m, 롤 커플 %.2f N m, 밀기 (%.1f, %.1f, 0) N' % (want['yaw_torque'], want['roll_torque'], F[0], F[1]))
    trim = None                                # the baseline attitude, per model, sets the couple

    for model in MODELS:
        if model in TORQUE_MODELS:
            before, rows = probe(model, 'torque: {z: %g}' % want['yaw_torque'], YAW_S)

            if rows is None or len(rows) < 30:
                verdict('%s: 요 토크 실험' % model, False, '표본 %s' % (len(rows) if rows else rows))
                continue

            ok, speed, rate = still(before)
            I, r0 = yaw_inertia(rows, want['yaw_torque'])

            if not (0.2 * want['izz'] < I < 5.0 * want['izz']):
                # The hull did not turn: a wrench that was accepted and never
                # applied. Once more, and the second answer stands.
                note('  %s: 요 토크에 응답이 없다 (관성 %.3g), 다시 한 번' % (model, I))
                before, rows = probe(model, 'torque: {z: %g}' % want['yaw_torque'], YAW_S)
                I, r0 = yaw_inertia(rows, want['yaw_torque']) if rows and len(rows) >= 30 else (float('nan'), 0)

            inertia[model] = I
            note('  %s: 렌치 전 속도 %.3f m/s 요율 %.4f rad/s%s; 요 토크 %.2f N m -> 관성 %.4f kg m^2 (표본 %s)'
                 % (model, speed, rate, '' if ok else ' (정지 아님)', want['yaw_torque'], I, len(rows) if rows else '없음'))

            if trim is None and before:
                trim = before[-1][2]

            before, rows = probe(model, 'torque: {x: %g}' % want['roll_torque'], ROLL_S, release_after=ROLL_KICK_S)

            if rows is None or len(rows) < 60:
                verdict('%s: 롤 방출 실험' % model, False, '표본 %s' % (len(rows) if rows else rows))
            else:
                T, n = period(rows)
                swing = max(roll(r[2]) for r in rows) - min(roll(r[2]) for r in rows)
                rolls[model] = T if math.degrees(swing) / 2 >= ROLL_MIN_SWING_DEG else float('nan')
                note('  %s: 롤 커플 %.2f N m 를 주고 놓음 -> 주기 %.3f s (교차 %d 회, 진폭 %.1f deg%s)'
                     % (model, want['roll_torque'], T, n, math.degrees(swing) / 2,
                        '' if math.degrees(swing) / 2 >= ROLL_MIN_SWING_DEG else ', 너무 작아 주기를 채택하지 않음'))

        # The couple that puts the force through the centre of mass: +d x F
        # with d rotated by the hull's trim attitude into the world frame,
        # which the plugin's wrench is in. The trim comes from this model's
        # own baseline sample, since the added mass does not change it but
        # the check should not assume so.
        q = trim if trim is not None else (1.0, 0.0, 0.0, 0.0)
        d = to_world(q, want['com'])
        couple = (d[1] * F[2] - d[2] * F[1], d[2] * F[0] - d[0] * F[2], d[0] * F[1] - d[1] * F[0])
        push = 'force: {x: %g, y: %g, z: %g}, torque: {x: %.4f, y: %.4f, z: %.4f}' % (F + couple)
        before, rows = probe(model, push, PUSH_S)

        if rows is None or len(rows) < 30:
            verdict('%s: 밀기 실험' % model, False, '표본 %s' % (len(rows) if rows else rows))
            continue

        if trim is None and before:
            trim = before[-1][2]

        kin = kinematics(rows)
        s, d, icpt, uvmax = yaw_fit(kin, damped=model.endswith('_fin'))
        slope[model] = s
        damping[model] = d
        kins[model] = kin
        t, u, v, r = kin[-1]
        beta = max(abs(math.degrees(math.atan2(v, u))) for _, u, v, _ in kin if math.hypot(u, v) > 0.3)
        sideslip[model] = beta
        note('  %s: 밀기 %.1f s 끝에 u %.2f v %.2f m/s, r %.3f rad/s, 최대 옆미끄럼 %.1f deg; r 대 ∫uv 기울기 %.4f%s (절편 %.4f), max|uv| %.3f'
             % (model, PUSH_S, u, v, r, beta, s, '' if d is None else ', ∫ur 감쇠 기울기 %.4f' % d, icpt, uvmax))

        with open(os.path.join(OUT, 'added_mass_%s.csv' % model), 'w') as f:
            f.write('t,u,v,r\n')
            f.writelines('%.4f,%.5f,%.5f,%.5f\n' % k for k in kin)

    RIGID, FIN, AM, AM_FIN = MODELS
    I_rigid, I_am = inertia.get(RIGID), inertia.get(AM)
    verdict('강체 요 관성이 SDF 가 말하는 값이다',
            I_rigid is not None and abs(I_rigid - want['izz']) <= 0.05 * want['izz'],
            '측정 %s, SDF %.4f' % ('%.4f' % I_rigid if I_rigid else '없음', want['izz']))
    verdict('부가 질량이 요 관성에 rr 만큼 더해진다',
            I_rigid is not None and I_am is not None and abs((I_am - I_rigid) - want['rr']) <= 0.06 * want['rr'],
            '측정 차 %s, rr %.4f' % ('%.4f' % (I_am - I_rigid) if I_rigid and I_am else '없음', want['rr']))
    T0, T1 = rolls.get(RIGID, float('nan')), rolls.get(AM, float('nan'))
    verdict('강체 롤 주기가 부력 강성과 SDF 관성이 말하는 값이다',
            math.isfinite(T0) and abs(T0 - want['roll_rigid']) <= 0.06 * want['roll_rigid'],
            '측정 %s s, 예측 %.3f s' % ('%.3f' % T0 if math.isfinite(T0) else '없음', want['roll_rigid']))
    verdict('부가 질량은 링크 원점 기준으로 읽힌다 (롤 주기가 원점 예측에 맞고 무게중심 예측에서는 멀다)',
            math.isfinite(T1) and abs(T1 - want['roll_origin']) <= 0.06 * want['roll_origin']
            and abs(T1 - want['roll_com']) >= 0.15 * want['roll_com'],
            '측정 %s s, 원점 예측 %.3f s, 무게중심 예측 %.3f s' % ('%.3f' % T1 if math.isfinite(T1) else '없음', want['roll_origin'], want['roll_com']))
    K = {m: slope[m] * (I_am if '_am' in m else I_rigid) for m in slope if (I_am if '_am' in m else I_rigid)}
    verdict('강체에는 Munk 모멘트가 없다',
            RIGID in K and abs(K[RIGID]) <= 0.15 * want['munk'],
            'K %s N m/(m/s)^2' % ('%.4f' % K[RIGID] if RIGID in K else '없음'))
    # Destabilising is the nose turning away from the velocity: for u, v > 0
    # in this y-port frame that is r < 0, so the slope is -(m_y - m_x) / I.
    verdict('부가 질량이 Munk 모멘트 -(m_y - m_x) u v 를 가져온다 (기수가 속도에서 멀어지는 쪽)',
            AM in K and abs(K[AM] + want['munk']) <= 0.15 * want['munk'],
            'K %s, 기대 %.4f' % ('%.4f' % K[AM] if AM in K else '없음', -want['munk']))
    D = {m: damping[m] * (I_am if '_am' in m else I_rigid) for m in damping
         if damping[m] is not None and (I_am if '_am' in m else I_rigid)}
    verdict('핀의 정적 풍향계 항: 강체 위에서 기수를 속도 쪽으로 돌린다 (옆미끄럼이 실속 아래일 때)',
            FIN in K and abs(K[FIN] - want['fin']) <= 0.15 * want['fin'] and sideslip.get(FIN, 90.0) < 15.0,
            'K_s(fin) %s, 기대 +%.4f; 최대 옆미끄럼 %s deg'
            % ('%.4f' % K[FIN] if FIN in K else '없음', want['fin'],
               '%.1f' % sideslip[FIN] if FIN in sideslip else '?'))
    verdict('핀의 요 감쇠 항은 정적 항 곱하기 팔 길이다',
            FIN in D and abs(D[FIN] - want['fin_damping']) <= 0.15 * want['fin_damping'],
            'K_d(fin) %s, 기대 %.4f' % ('%.4f' % D[FIN] if FIN in D else '없음', want['fin_damping']))
    if AM_FIN in kins and FIN in D and I_am:
        # The rigid hull's damping, the same fins in the same air, held; only
        # the static term is fitted (the module docstring says why).
        s_fixed, _, icpt, _ = yaw_fit(kins[AM_FIN], damped=True, damping_known=D[FIN] / I_am)
        K[AM_FIN] = s_fixed * I_am
        note('  %s: 강체 핀의 감쇠 %.4f 를 고정하고 다시 맞춤 -> 정적 항 K_s %.4f (절편 %.4f)' % (AM_FIN, D[FIN], K[AM_FIN], icpt))

    verdict('부가 질량 위의 핀: 강체에서 잰 감쇠를 빼고 남는 정적 항은 핀에서 Munk 를 뺀 것이다',
            AM_FIN in K and FIN in D and abs(K[AM_FIN] - (want['fin'] - want['munk'])) <= 0.15 * want['munk']
            and sideslip.get(AM_FIN, 90.0) < 15.0,
            'K_s %s (기대 %.4f), 고정한 K_d %s; 최대 옆미끄럼 %s deg'
            % ('%.4f' % K[AM_FIN] if AM_FIN in K else '없음', want['fin'] - want['munk'],
               '%.4f' % D[FIN] if FIN in D else '없음',
               '%.1f' % sideslip[AM_FIN] if AM_FIN in sideslip else '?'))
    return summary()


if __name__ == '__main__':
    sys.exit(main())
