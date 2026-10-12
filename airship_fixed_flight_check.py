#!/usr/bin/env python3
"""First flight of the 2500 airframe's gz twin: fixed thrusters, fin flaps.

    GZ_TARGET=gz_airship_fixed ./gz_run.sh ./airship_fixed_flight_check.py
    GZ_TARGET=gz_airship_fixed_drag ...   (_am, _am_drag likewise)

The vehicle: two fixed forward thrusters for one collective thrust, four fin
flaps (two elevators, two rudders) for pitch and yaw, nothing else. Its whole
attitude authority is aerodynamic, so it has none at rest, and that nullity
is the first thing this check asserts rather than the first thing it trips
over. In the order a first flight would take:

  1. the model is in the world, afloat, and neutrally buoyant. Measured as
     the hull's vertical motion on the simulator's clock: an acceleration on
     the drag-free plants, a terminal rate on the _drag ones, both turned
     into the force that would cause them and held to what the rest tests
     below can live with (STILL_U over the sim minute they take), not to a
     fraction of the weight. The first run of this check skipped this and
     read a hull rising at 0.16 m/s^2 as "at rest": airship_fixed had
     dropped the vectored airship's 0.4 kg pod links while keeping the hull
     box sized for them, and every verdict after that was taken in a 3 m/s
     updraft over the fins
  2. the allocator configured two motors and four surfaces
  3. it arms in Manual, where every stick is a passthrough (and on this
     airframe the controller never closes its yaw rate loop: 4b)
  4. AT REST (the check waits for it, bounded, and says so) a full yaw
     stick swings both rudders to the joint limit and turns nothing; a full
     pitch stick swings both elevators and pitches nothing. The flap angle
     is read off the flap links' own poses, which /world/lta/dynamic_pose
     publishes relative to the model, so it checks the bridge's
     SIM_GZ_SV_MINA/MAXA against the joint limits; the servo topic is read
     too, to say which side is wrong when one is. The motor speeds are read
     as well: a stick that spins the thrusters is the defect this vehicle
     cannot afford
  4b. in Stabilized at rest, half yaw stick is half rudder and releasing it
     centres the rudder: with collective pods and no tail the controller
     keeps the stick as torque in every mode
  5. throttle: the hull accelerates forward along its heading at the rate
     the motors' thrust and the hull's drag say, u = u0 + a t on the
     drag-free plants, u_t tanh on _drag with u_t = sqrt(T / c), and the
     rotor speed is sqrt(command) x 300 rad/s, which is THR_MDL_FAC 1
  6. at speed, yaw stick right then left: the nose turns that way, and the
     peak yaw rate in the 1.5 s after the stick is what the file's numbers
     predict. The pose reader is opened before the stick moves so the onset,
     which the window and its speed are timed from, is in the record. On the
     rigid and _drag members the rise peaks at about 0.4 s, which a record
     started after the gz CLI's attach delay has already missed; on the
     added-mass members the Munk moment keeps the rate climbing past 1 s
     (the engine peaked at 1.2 s, the model at 1.4-1.7 s, both within 2 %
     of that at 1.5 s).
     The prediction is a three-state simulation (sideslip, yaw rate, sway)
     of the rudder moment against the fins' own weathercock and rate
     damping, their lift and drag as the lift-drag plugin applies them, the
     Munk moment and the added masses' sway coupling on the added-mass
     plants and the quadratic damping on the _drag ones, judged to 10 % (a
     plant that lost its Munk moment reads 21 % low); r = u (dcl/ddelta)
     delta / (cla l_f) is the simpler ceiling it never reaches, since the
     sideslip the turn builds eats into the rudder's authority within the
     window
  6b. full right rudder held for 10 s: the rate in the last second is the
     turn the plant keeps, predicted by the same model with the hull's
     crossflow drag at the speed of that second (the drag-free plants slow
     in the turn). On the rigid hull the fins are the only yaw moment, so in a
     steady turn they carry no net side force and the rate dies away; hull
     drag keeps a weak turn; the added-mass plants keep turning because the
     fins must also hold the Munk moment
  7. pitch stick back then forward: nose up then down, the peak pitch rate
     in the first second likewise predicted and judged, with the pendulum stiffness
     m g (z_cov - z_com) and the heave the elevators' own lift starts. The
     angle at the end is reported, not judged: on the rigid and _drag hulls
     the elevators' lift makes heave until the fins carry no net lift, so
     the pitch comes back to about level while the path tilts; on the
     added-mass members the Munk moment holds a pitch of 2-3 deg against
     the pendulum. It is neither an elevator trim nor a path angle

Everything expected is read from the model files: the thrust, the fins'
lift, drag, stall and flap slope, the arms, the mass, the centre of mass,
the centre of volume, the inertias, the drag and added-mass blocks where the
variant has them. Rates and angles come from the simulator's pose stream on the
simulator's clock, since with the fin plugins on the sim does not keep up
with the wall. Every leg's kinematics are written to sitl_out/ so a verdict
line never has to be taken on faith.
"""
import math
import os
import re
import sys
import time

import fin_basis
import gz_link
import gz_model
from report import note, summary, verdict
from sitl_link import MODEL, MODEL_NAME, Sticks, commander, field, listener, px4, wait_for_ekf

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sitl_out')
MIN_HEIGHT_M = 5.0
SETTLE_S = 10.0              # sim seconds for the spawn transient
WINDOW_S = 10.0              # and then the buoyancy window
REST_HORIZON_S = 60.0        # sim seconds from the buoyancy window to the last stick at rest
PRE_STICK_S = 1.0            # wall seconds the pose reader runs before a stick moves
ONSET_DEG_S = 0.8            # a rate this far from its pre-stick mean is the stick's onset
STILL_U = 0.10               # m/s, rest before a stick means anything
STILL_RATE = 0.5             # deg/s
STILL_MAX_S = 60.0           # sim seconds to wait for rest
REST_S = 5.0                 # sim seconds a stick is held at rest
TARGET_U = 3.0               # m/s to reach before the sticks mean anything
THROTTLE = 0.5               # thrust fraction while accelerating
ACCEL_MAX_S = 25.0           # sim seconds allowed to reach TARGET_U
YAW_S = 1.5                  # sim seconds of yaw stick, and of the peak window
HELD_S = 10.0                # sim seconds of the held-rudder leg
HELD_ABS_DEG_S = 0.1         # its floor where the plant turns next to nothing; 10 % governs from 1 deg/s
PITCH_S = 4.0                # sim seconds of pitch stick
PITCH_PEAK_S = 1.0           # the peak window inside it
RECOVER_S = 6.0              # sim seconds with the sticks centred between legs
FLAP_TOL_DEG = 0.5            # the servo angle is exact; a SV range off the joint limit shows
STAB_HELD_TOL_DEG = 0.5        # the Stabilized leg: passthrough is exact; a loop reads a fifth of it
STAB_RELEASE_TOL_DEG = 0.2     # and a held integrator leaves ~0.46 deg
# One tolerance for every plant and both axes. The 2026-09-25 records of all
# four members, replayed through simulate() on the fin plugins they were
# flown with, match it within 2.5 % in yaw (the onset peaks, and the held
# turns of the added-mass members) and 4.5 % in pitch; a plant that lost its
# Munk moment would read 21 % low in yaw and 11-12 % low in pitch, so 10 %
# tells the two apart and holds the flaps' authority to 10 % rather than
# 30-40 %.
RATE_TOL = 0.10
RATE_HZ = 60.0               # pose stream rate assumed until the buoyancy window measures it
# {flap link: (axis, servo topic)}, from the model's servo and fin plugins
FLAPS = gz_model.flap_servos(gz_model.merged(MODEL_NAME))
# what fin_basis.plugin_cl/plugin_cd read of a fin, and its area
FIN_RECORD = ('area', 'cla', 'cda', 'alpha_stall', 'cla_stall', 'cda_stall')


def plant():
    """What the files say this vehicle is."""
    model = gz_model.merged(MODEL_NAME)
    family = gz_model.family(MODEL_NAME)
    mass, com = gz_model.centre_of_mass(family)
    drag = gz_model.hydrodynamics(model)
    added = gz_model.added_mass(model)
    thrust = 0.0

    for link in ('prop_starboard', 'prop_port'):
        full = gz_model.full_thrust(model, link)
        thrust += full[0] if full else 0.0

    fins = {'yaw': [], 'pitch': []}
    limit = None

    for f in gz_model.lift_drag_fins(model):
        missing = [k for k in FIN_RECORD if f[k] is None]

        if missing:
            # the plugin has defaults for these; a prediction on a guess of
            # them would be a verdict about some other fin
            raise LookupError('%s 의 양항력 플러그인 (cp %s) 에 %s 가 없다' % (MODEL_NAME, f['cp'], ', '.join(missing)))

        slope = f['control_joint_rad_to_cl']
        fin = {k: f[k] for k in FIN_RECORD}
        fin.update(slope=slope if slope is not None else 0.0, arm=abs(f['cp'][0] - com[0]))
        fins['yaw' if f['vertical'] else 'pitch'].append(fin)

    for joint in model.findall('joint'):
        upper = joint.find('axis/limit/upper')

        if joint.get('name', '').startswith('flap_') and upper is not None:
            limit = float(upper.text)

    # the buoyancy's lever over the centre of mass: the pendulum
    h_c = gz_model.centre_of_volume(family)[1][2] - com[2]
    return {'mass': mass, 'com': com, 'thrust': thrust, 'fins': fins, 'limit': limit,
            'drag_x': abs(drag['xUabsU']) if 'xUabsU' in drag else None,
            'drag_y': abs(drag['yVabsV']) if 'yVabsV' in drag else 0.0,
            'drag_z': abs(drag['zWabsW']) if 'zWabsW' in drag else None,
            'drag_r': abs(drag['nRabsR']) if 'nRabsR' in drag else 0.0,
            'drag_q': abs(drag['mQabsQ']) if 'mQabsQ' in drag else 0.0,
            'added_x': added.get('xx', 0.0), 'added_y': added.get('yy', 0.0), 'added_z': added.get('zz', 0.0),
            # the block's yaw inertia holds about any point on the yaw axis;
            # its pitch inertia is the block's qq moved from the link origin
            # to the CoM height z, qq - 2 z xq + z^2 xx (the CoM's few mm of x
            # would add x^2 zz, 0.002 kg m^2, as x^2 yy would in yaw: left out)
            'added_r': added.get('rr', 0.0),
            'added_q': (added.get('qq', 0.0) - 2.0 * com[2] * added.get('xq', 0.0)
                        + com[2] * com[2] * added.get('xx', 0.0)) if added else 0.0,
            'munk': added.get('yy', 0.0) - added.get('xx', 0.0),
            'izz': gz_model.inertia(family, 2), 'iyy': gz_model.inertia(family, 1),
            'stiffness': mass * gz_model.G * h_c, 'am': bool(added)}


def euler(q):
    """Roll, pitch, yaw (rad) of a w-x-y-z quaternion in gz's z-up frame."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def unwrap(angles):
    out = [angles[0]]

    for a in angles[1:]:
        d = a - out[-1]
        out.append(out[-1] + (d + math.pi) % (2 * math.pi) - math.pi)

    return out


def stream(seconds, chunk=60):
    """Pose rows of the model until the simulator's clock has advanced
    `seconds`, in chunks; bounded on the wall at four times that."""
    rows = []
    deadline = time.time() + 4.0 * seconds + 10.0

    while time.time() < deadline:
        part = gz_link.poses(MODEL, limit=chunk, timeout=30)

        if part:
            rows.extend(part)

            if rows[-1][0] - rows[0][0] >= seconds:
                break

    rows.sort()
    return rows


def rates(rows):
    """[(t, u, w_up, z, pitch_up_deg, q_deg_s, r_deg_s)] by central
    differences, skipping pairs that straddle a gap between chunks. FRD
    signs for the rates and the angle: nose up and nose right positive,
    which in gz's z-up frame are -d(pitch)/dt and -d(yaw)/dt."""
    if not rows or len(rows) < 3:
        return []

    ts = [r[0] for r in rows]
    eul = [euler(r[2]) for r in rows]
    yaws = unwrap([e[2] for e in eul])
    out = []

    for i in range(1, len(rows) - 1):
        dt = ts[i + 1] - ts[i - 1]

        if dt <= 0 or dt > 0.2:
            continue

        vx = (rows[i + 1][1][0] - rows[i - 1][1][0]) / dt
        vy = (rows[i + 1][1][1] - rows[i - 1][1][1]) / dt
        vz = (rows[i + 1][1][2] - rows[i - 1][1][2]) / dt
        yaw = yaws[i]
        u = vx * math.cos(yaw) + vy * math.sin(yaw)
        q = -math.degrees((eul[i + 1][1] - eul[i - 1][1]) / dt)
        r = -math.degrees((yaws[i + 1] - yaws[i - 1]) / dt)
        out.append((ts[i], u, vz, rows[i][1][2], -math.degrees(eul[i][1]), q, r))

    return out


def dump(name, kin):
    os.makedirs(OUT, exist_ok=True)

    with open(os.path.join(OUT, 'fixed_flight_%s_%s.csv' % (MODEL_NAME, name)), 'w') as f:
        f.write('t,u,w_up,z,pitch_up_deg,q_deg_s,r_deg_s\n')
        f.writelines('%.4f,%.5f,%.5f,%.4f,%.4f,%.4f,%.4f\n' % k for k in kin)


def flap_angles():
    """Deflection of each flap in deg from its link's pose (relative to the
    model in dynamic_pose/info), or None for a flap the stream lacked."""
    out = {}

    for link, (axis, _) in FLAPS.items():
        rows = None

        # pose/info, not dynamic_pose/info: a flap parked at its limit is
        # not in every dynamic message (the third run read one rudder as
        # absent while its servo topic and its twin both stood at the
        # limit); the static stream carries every entity every publish
        for _ in range(3):
            rows = gz_link.poses(link, limit=5, timeout=20, dynamic=False)

            if rows:
                break

        if not rows:
            out[link] = None
            continue

        roll, pitch, yaw = euler(rows[-1][2])
        out[link] = math.degrees(yaw if axis == 'yaw' else pitch)

    return out


def servo_angles():
    """The bridge's commanded angle on each servo topic, deg, or None."""
    out = {}

    for link, (_, topic) in FLAPS.items():
        text = gz_link.topic('/model/%s/%s' % (MODEL, topic), 1, 8)
        found = re.search(r'data:\s*(-?[\d.eE+-]+)', text or '')
        out[link] = math.degrees(float(found.group(1))) if found else None

    return out


def motor_max():
    """The larger of the two rotor speeds, rad/s, or None."""
    speeds = gz_link.motor_speeds()
    return max(speeds[:2]) if speeds and len(speeds) >= 2 else None


def onset_speed(kin, before):
    """The forward speed as the stick moved: the leg's own onset sample,
    else the read taken before it, else NaN (which fails the verdict with the
    speed shown, rather than predicting for a hull at rest)."""
    if kin:
        return kin[0][1]

    return before[1] if before else float('nan')


def now():
    rows = gz_link.poses(MODEL, limit=12, timeout=20)
    kin = rates(rows) if rows else []
    return kin[-1] if kin else None


def wait_still(max_s=STILL_MAX_S):
    """Until |u|, |w| < STILL_U and the rates < STILL_RATE, on the sim clock,
    bounded. Returns (stilled, last sample, sim seconds waited)."""
    first, last = None, None
    deadline = time.time() + 3.0 * max_s + 10.0

    while time.time() < deadline:
        kin = rates(stream(1.0, chunk=30))

        if not kin:
            continue

        first = first or kin[0][0]
        last = kin[-1]
        still = all(abs(k[1]) < STILL_U and abs(k[2]) < STILL_U and abs(k[5]) < STILL_RATE and abs(k[6]) < STILL_RATE
                    for k in kin)

        if still:
            return True, last, last[0] - first

        if last[0] - first > max_s:
            break

    return False, last, (last[0] - first) if last and first else float('nan')


def hold(sticks, seconds, axis, **values):
    """Open the pose reader, wait for it to attach, move the sticks, hold
    them `seconds` of the simulator's clock, centre them again (the
    throttle stays). Returns (kinematics from the onset on, onset_clean):
    the onset is the first sample whose rate on `axis` (5 pitch, 6 yaw)
    leaves its pre-stick mean by ONSET_DEG_S, and clean means the record
    holds pre-stick samples, so the rise's first half second is in it. A
    record that starts after the stick moved cannot see the peak of a
    0.4 s rise, which is what the first version of this measured."""
    cap = gz_link.Capture(int(RATE_HZ * (seconds + 2.0)))
    time.sleep(PRE_STICK_S)

    for k, v in values.items():
        setattr(sticks, k, v)

    rows = cap.collect(MODEL, timeout=4.0 * (seconds + 2.0) + 15.0)

    for k in values:
        if k != 'throttle':
            setattr(sticks, k, 0)

    kin = rates(rows) if rows else []

    if len(kin) < 10:
        return [], False

    col = 5 if axis == 'pitch' else 6
    pre = kin[:5]
    base = sum(k[col] for k in pre) / len(pre)
    onset = next((i for i, k in enumerate(kin) if abs(k[col] - base) > ONSET_DEG_S), None)

    if onset is None:
        return kin, False

    return kin[max(0, onset - 1):], onset >= 3


def leg(sticks, seconds, axis, record, **values):
    """hold() the sticks, once more if the record missed the onset, write the
    record to sitl_out, and return (kin, clean, the speed at the onset)."""
    for _ in range(2):
        before = now()
        kin, clean = hold(sticks, seconds, axis, **values)

        if clean:
            break

        stream(RECOVER_S)

    dump(record, kin)
    return kin, clean, onset_speed(kin, before)


def peak_check(p, kin, clean, axis, sign, window_s, u_on):
    """(ok, peak, predicted peak) over the first window_s of a record, both
    in the stick's direction: the yaw rate for a yaw stick, the pitch rate,
    nose up positive, for a pitch stick (back, -1, is nose up)."""
    col, s = (6, sign) if axis == 'yaw' else (5, -sign)
    pred = simulate(p, axis, u_on, p['limit'], window_s)[0]
    window = [k for k in kin if k[0] <= kin[0][0] + window_s] if kin else []
    peak = max((k[col] * s for k in window), default=float('nan'))
    return clean and math.isfinite(peak) and pred > 0 and abs(peak - pred) <= RATE_TOL * pred, peak, pred


def simulate(p, axis, u, delta, seconds, dt=0.002):
    """Peak rate and the rate at the end (deg/s, in the stick's own
    direction) of a step of flap angle `delta` at speed u, from the file's
    numbers. Three states: the angle the nose makes with the world, its
    rate, and the transverse velocity the fins' lift starts. The fins see
    the flow at their station, the angle less the transverse velocity there
    over u, which is the weathercock and the rate damping at once; the
    flaps add slope x delta to the lift coefficient; the added-mass plants
    carry the Munk moment (m_t - m_x) u^2 alpha and their added masses; the
    _drag plants their quadratic rate damping. Pitch adds the pendulum.
    Signs are the stick's: positive is the direction the stick asks for.

    Each fin's force is the lift-drag plugin's, as generic_hull_design.turn()
    applies it: cl and cd (fin_basis.plugin_cl/plugin_cd, stall included) on
    the flow angle at the fin, the dynamic pressure of that flow, both
    resolved across the hull axis. The drag's share is small at these angles
    but it is the plugin's, and without it the held turn of the added-mass
    plants reads 3-4 % above the engine."""
    fins = p['fins'][axis]

    # The transverse momentum in the frame the flow angle is written in is
    # (m + m_t) vt - (m_t - m_x) u angle, so its rate carries (m_t - m_x) u
    # times the turn rate: the body-frame (m + m_t) v' + (m + m_x) u r = Y
    # rewritten. Without it the added-mass predictions were 5 % low in yaw.
    if axis == 'yaw':
        I = p['izz'] + p['added_r']
        m = p['mass'] + p['added_y']
        coupling = p['added_y'] - p['added_x']
        stiff, c_rot = 0.0, p['drag_r']
        c_side = p['drag_y']
    else:
        I = p['iyy'] + p['added_q']
        m = p['mass'] + p['added_z']
        coupling = p['added_z'] - p['added_x']
        stiff, c_rot = p['stiffness'], p['drag_q']
        c_side = p['drag_z'] or 0.0

    # Signs: positive is the side the stick asks the nose to go to. `angle`
    # is the nose's angle to that side, `rate` its rate, `vt` the hull's
    # velocity toward that side. A fin at `arm` behind the centre of mass
    # sees the flow cross it at u angle - vt + arm rate (the tail moves
    # against the nose), and the force toward that side that flow makes is
    # the weathercock and the rate damping at once. The flap lifts the tail
    # the OTHER way, which is what turns the nose this way. A tail force
    # toward the side turns the nose away from it, moment = -arm x force.
    angle, rate, vt = 0.0, 0.0, 0.0
    peak = 0.0

    for _ in range(int(seconds / dt)):
        force, moment = 0.0, 0.0

        for f in fins:
            cross = u * angle - vt + f['arm'] * rate if u > 0.05 else 0.0
            vel = math.hypot(u, cross)
            alpha = math.atan2(cross, u)
            cl = fin_basis.plugin_cl(alpha, f) - abs(f['slope']) * delta
            side = 0.5 * gz_model.RHO_AIR * vel * f['area'] * (cl * u + fin_basis.plugin_cd(alpha, f) * cross)
            force += side
            moment -= f['arm'] * side

        # the hull's own crossflow drag on the _drag plants, on its velocity
        # across its axis: the only side force besides the fins, and on the
        # rigid-plus-drag hull the whole of what a held rudder turns with
        across = vt - u * angle
        force -= c_side * across * abs(across)
        moment += p['munk'] * u * u * (angle - vt / u if u > 0.05 else 0.0) - stiff * angle - c_rot * rate * abs(rate)
        vt += ((force + coupling * u * rate) / m) * dt
        rate += (moment / I) * dt
        angle += rate * dt
        peak = max(peak, rate)

    return math.degrees(peak), math.degrees(rate)


def main():
    if not commander('status'):
        note('px4 에 붙지 못했다')
        return 3

    p = plant()
    yaw_fins, pitch_fins = p['fins']['yaw'], p['fins']['pitch']
    limit_deg = math.degrees(p['limit']) if p['limit'] else float('nan')
    note('플랜트: 질량 %.3f kg, 추력 %.1f N, 핀 수직 %d 수평 %d (cla %.3f, 플랩 %.3f/rad, 팔 %.2f m), 조인트 한계 %s deg, izz %.1f iyy %.1f, 진자 강성 %.0f N m/rad, 항력 %s, 부가 질량 %s'
         % (p['mass'], p['thrust'], len(yaw_fins), len(pitch_fins),
            yaw_fins[0]['cla'] if yaw_fins else float('nan'), yaw_fins[0]['slope'] if yaw_fins else float('nan'),
            yaw_fins[0]['arm'] if yaw_fins else float('nan'), '%.1f' % limit_deg if math.isfinite(limit_deg) else '없음',
            p['izz'], p['iyy'], p['stiffness'], '있음' if p['drag_x'] else '없음', '있음' if p['am'] else '없음'))

    if not yaw_fins or not pitch_fins or not p['limit'] or p['thrust'] <= 0:
        verdict('모델에 핀, 플랩 조인트, 추진기가 있다', False, '핀 %d/%d, 조인트 한계 %s, 추력 %.1f' % (len(yaw_fins), len(pitch_fins), p['limit'], p['thrust']))
        return summary()

    # 1. in the world, afloat, balanced
    rows = gz_link.poses(MODEL)
    verdict('모델이 월드에 있다', rows is not None and len(rows) >= 10,
            '%s 의 자세 표본 %s 개' % (MODEL, '읽기실패' if rows is None else len(rows)))

    if not rows or len(rows) < 10:
        return summary()

    stream(SETTLE_S)
    kin = rates(stream(WINDOW_S))
    dump('buoyancy', kin)
    weight = p['mass'] * gz_model.G

    if len(kin) >= 20:
        global RATE_HZ
        RATE_HZ = max(20.0, (len(kin) - 1) / (kin[-1][0] - kin[0][0]))

    # What the rest tests can live with, in newtons: on a drag-free plant an
    # imbalance F accelerates the hull at F / (m + added) with nothing to
    # stop it, and the last stick at rest comes REST_HORIZON_S later; on a
    # _drag plant it settles at sqrt(F / c_z). Either must stay under
    # STILL_U. One percent of the weight, the first version's tolerance,
    # would have let a hull pass here and then fail every rest test.
    if p['drag_z']:
        tol_n = p['drag_z'] * STILL_U * STILL_U
    else:
        tol_n = STILL_U * (p['mass'] + p['added_z']) / REST_HORIZON_S

    if len(kin) < 20:
        verdict('중성부력으로 뜬다', False, '창 안의 표본 %d' % len(kin))
    elif min(k[3] for k in kin) < MIN_HEIGHT_M:
        verdict('중성부력으로 뜬다', False, '선체가 %.1f m 까지 내려왔다 (바닥 %.0f m)' % (min(k[3] for k in kin), MIN_HEIGHT_M))
    elif p['drag_z']:
        w = sum(k[2] for k in kin[-10:]) / 10.0
        implied = math.copysign(p['drag_z'] * w * w, w)
        verdict('중성부력으로 뜬다', abs(implied) < tol_n,
                '항력 플랜트: %.0f 초 정착 뒤 수직 속도 %+.3f m/s = 불균형 %+.3f N (허용 %.3f N, 정지 시험이 견디는 만큼), 무게 %.1f N 의 %.3f %%'
                % (SETTLE_S, w, implied, tol_n, weight, 100.0 * abs(implied) / weight))
    else:
        span = kin[-1][0] - kin[0][0]
        w0, w1 = sum(k[2] for k in kin[:10]) / 10.0, sum(k[2] for k in kin[-10:]) / 10.0
        accel = (w1 - w0) / span if span > 0 else float('nan')
        implied = (p['mass'] + p['added_z']) * accel
        verdict('중성부력으로 뜬다', math.isfinite(implied) and abs(implied) < tol_n,
                '감쇠 없는 플랜트: %.0f 초 정착 뒤 %.1f 초 동안 수직 속도 %+.3f -> %+.3f m/s, 가속도 %+.4f m/s^2 = 불균형 %+.3f N (허용 %.3f N, 정지 시험이 견디는 만큼), 무게 %.1f N 의 %.3f %%'
                % (SETTLE_S, span, w0, w1, accel, implied, tol_n, weight, 100.0 * abs(implied) / weight if math.isfinite(implied) else float('nan')))

    # 2. the allocator (the status client can miss its window right after
    #    boot, so three tries)
    count = -1

    for _ in range(3):
        status = px4('control_allocator', 'status') or ''
        configured = re.findall(r'Configured actuators:\s*(\d+)', status)

        if configured:
            count = int(configured[0])
            break

        time.sleep(2)

    verdict('할당기가 모터 둘과 조종면 넷을 잡았다', count == 6, '할당기가 보고한 출력 %s' % (count if count >= 0 else '없음'))

    if not wait_for_ekf():
        verdict('EKF 가 유효해진다', False, '위치 추정이 유효해지지 않았다')
        return summary()

    # 3. arm
    sticks = Sticks()
    sticks.start()
    time.sleep(2)
    commander('mode', 'manual')
    time.sleep(1)
    commander('arm')
    time.sleep(3)
    armed = field(listener('vehicle_control_mode'), 'flag_armed') == 1.0
    verdict('Manual 에서 무장한다', armed, '무장됨' if armed else '무장 거부')

    if not armed:
        sticks.stop = True
        return summary()

    # 4. at rest: the surfaces move and nothing else does
    show = lambda d: ', '.join('%s %s' % (k, '?' if v is None else '%+.1f' % v) for k, v in sorted(d.items()))

    for axis, sign, links in (('yaw', +1.0, ('flap_top', 'flap_bottom')), ('pitch', -1.0, ('flap_starboard', 'flap_port'))):
        stilled, sample, waited = wait_still()
        note('  %s 스틱 전 정지 대기 %.1f s: %s (u %.3f w %.3f m/s, q %.2f r %.2f deg/s)'
             % (axis, waited, '정지' if stilled else '정지 못함', *(sample[1:3] + sample[5:7])) if sample else '  정지 대기: 표본 없음')
        setattr(sticks, axis, sign)
        time.sleep(0.15)
        kin = rates(stream(REST_S))
        dump('rest_' + axis, kin)
        flaps = flap_angles()
        servos = servo_angles()
        spun = motor_max()
        setattr(sticks, axis, 0)
        col = 6 if axis == 'yaw' else 5
        peak = max((abs(k[col]) for k in kin), default=float('nan'))
        moved = max((math.hypot(k[1], k[2]) for k in kin), default=float('nan'))
        # +yaw stick: trailing edges to starboard, a positive rudder angle about
        # +z; pitch stick back (-1): trailing edges up, a positive elevator angle
        # about +y. Both read +limit on the flap links.
        deflected = all(flaps.get(l) is not None and abs(flaps[l] - limit_deg) <= FLAP_TOL_DEG for l in links)
        commanded = all(servos.get(l) is not None and abs(servos[l] - limit_deg) <= FLAP_TOL_DEG for l in links)
        verdict('정지 상태의 %s 스틱: 조종면은 한계까지 꺾이고 선체는 돌지 않고 추진기는 돌지 않는다' % ('요' if axis == 'yaw' else '피치'),
                stilled and deflected and commanded and math.isfinite(peak) and peak < 1.0 and moved < 0.3
                and spun is not None and spun < 1.0,
                '플랩 [%s] (기대 +%.1f), 서보 토픽 [%s]; %.1f s 동안 최대 %s 각속도 %s deg/s, 최대 속도 %s m/s, 회전수 %s rad/s%s'
                % (show({l: flaps.get(l) for l in links}), limit_deg, show({l: servos.get(l) for l in links}), REST_S,
                   '요' if axis == 'yaw' else '피치', '?' if not math.isfinite(peak) else '%.2f' % peak,
                   '?' if not math.isfinite(moved) else '%.2f' % moved, '?' if spun is None else '%.0f' % spun,
                   '' if stilled else '; 정지 전제가 깨졌다'))

    # 4b. the rate modes at rest. With collective pods and no tail the airship
    #     controller keeps the yaw stick as rudder torque in every mode (its
    #     rate loop has no airspeed scaling): half stick is half rudder, and
    #     releasing it centres the rudder. Firmware that closed the loop here
    #     showed a small P step, a ramp to its integrator limit, and a rudder
    #     that stayed deflected after release (measured at the pins on
    #     2026-09-25: 1650 us held with the stick centred).
    commander('mode', 'stabilized')
    time.sleep(1.5)
    control = listener('vehicle_control_mode')
    in_stabilized = field(control, 'flag_control_attitude_enabled') == 1.0 and field(control, 'flag_control_rates_enabled') == 1.0
    sticks.yaw = 0.5
    time.sleep(4.0)
    held = flap_angles()
    sticks.yaw = 0.0
    time.sleep(3.0)
    released = flap_angles()
    commander('mode', 'manual')
    time.sleep(1.0)
    back = field(listener('vehicle_control_mode'), 'flag_control_attitude_enabled') == 0.0
    rudders = ('flap_top', 'flap_bottom')
    want = 0.5 * limit_deg
    # A closed loop would read P x setpoint plus 4 s of integral held, about a
    # fifth of the passthrough value, and leave AS_YAWRATE_I x setpoint x 4 s
    # of rudder (about 0.46 deg at +-11 deg) after release: the tolerances are
    # below both. The servo angle itself is exact.
    verdict('Stabilized 에서도 요 스틱이 곧 방향타다 (정지 상태, 루프 없음)',
            in_stabilized and back
            and all(held.get(l) is not None and abs(held[l] - want) <= STAB_HELD_TOL_DEG for l in rudders)
            and all(released.get(l) is not None and abs(released[l]) <= STAB_RELEASE_TOL_DEG for l in rudders),
            '%s%s반 스틱 4 s: [%s] (기대 %+.1f), 놓고 3 s: [%s] (기대 0)'
            % ('' if in_stabilized else 'Stabilized 에 들어가지 못했다; ', '' if back else 'Manual 로 돌아오지 못했다; ',
               show({l: held.get(l) for l in rudders}), want, show({l: released.get(l) for l in rudders})))

    # 5. throttle: forward, along the heading, at the rate the files say
    stilled = wait_still()[0]
    accel = THROTTLE * p['thrust'] / (p['mass'] + p['added_x'])
    sticks.throttle = THROTTLE
    time.sleep(0.15)
    start = now()
    kin_all, reached, spun = [], None, None
    deadline = time.time() + 4.0 * ACCEL_MAX_S

    while time.time() < deadline and start:
        kin = rates(stream(1.0, chunk=30))
        kin_all.extend(kin)
        spun = motor_max() if spun is None else max(spun, motor_max() or 0.0)

        if kin and kin[-1][1] >= TARGET_U:
            reached = kin[-1]
            break

        if kin and kin[-1][0] - start[0] > ACCEL_MAX_S:
            break

    dump('throttle', kin_all)
    end = kin_all[-1] if kin_all else None
    elapsed = (end[0] - start[0]) if start and end else None
    u0 = start[1] if start else 0.0

    if elapsed is not None:
        if p['drag_x']:
            u_t = math.sqrt(THROTTLE * p['thrust'] / p['drag_x'])
            # u_t tanh(a t / u_t + atanh(u0 / u_t)), the drag ODE from u0
            u_pred = u_t * math.tanh(accel * elapsed / u_t + math.atanh(max(-0.99, min(0.99, u0 / u_t))))
        else:
            u_pred = u0 + accel * elapsed
    else:
        u_pred = float('nan')

    omega = math.sqrt(THROTTLE) * 300.0
    verdict('추력이 선체를 진행 방향으로 파일이 말하는 만큼 가속하고 회전수는 THR_MDL_FAC 1 의 것이다',
            reached is not None and elapsed is not None and abs(end[1] - u_pred) <= 0.15 * u_pred
            and spun is not None and abs(spun - omega) <= 0.03 * omega and stilled,
            '추력 %.0f %% = %.1f N, 예측 가속도 %.3f m/s^2; u %s -> %s m/s 가 시뮬레이터 %s s (예측 %s); 회전수 %s rad/s (기대 %.0f)%s'
            % (100 * THROTTLE, THROTTLE * p['thrust'], accel, '%.2f' % u0 if start else '?', '?' if end is None else '%.2f' % end[1],
               '?' if elapsed is None else '%.1f' % elapsed, '%.2f' % u_pred if math.isfinite(u_pred) else '?',
               '?' if spun is None else '%.0f' % spun, omega, '' if stilled else '; 출발 전 정지 못함'))

    # Hold the speed: nothing on a drag-free plant, the drag's worth on the others
    u_now = end[1] if end else TARGET_U
    sticks.throttle = min(1.0, p['drag_x'] * u_now * u_now / p['thrust']) if p['drag_x'] else 0.0

    # 6. yaw stick at speed
    for sign, name in ((+1.0, '오른쪽'), (-1.0, '왼쪽')):
        kin, clean, u_on = leg(sticks, YAW_S, 'yaw', 'yaw_%+d' % sign, yaw=sign)
        ok, peak, pred = peak_check(p, kin, clean, 'yaw', sign, YAW_S, u_on)
        verdict('%s 요 스틱이 기수를 %s으로 파일이 말하는 만큼 돌린다' % (name, name), ok,
                'u %.2f m/s 에서 스틱 뒤 %.1f s 동안 요 각속도 최대 %s deg/s, 예측 %+.1f (허용 %.0f %%)%s; 기록 %s'
                % (u_on, YAW_S, '?' if not math.isfinite(peak) else '%+.1f' % (peak * sign), pred * sign, 100 * RATE_TOL,
                   '' if clean else '; 기록이 스틱의 시작을 놓쳤다', 'sitl_out/fixed_flight_%s_yaw_%+d.csv' % (MODEL_NAME, sign)))
        stream(RECOVER_S)

    # 6b. a held rudder: the turn the plant keeps, not the onset
    kin, clean, u_on = leg(sticks, HELD_S, 'yaw', 'yaw_held', yaw=1.0)
    # The drag-free plants slow in the turn (3.10 -> 2.73 m/s over the 10 s on
    # _am, 2026-09-25) and the model holds u, so the turn it keeps is judged
    # at the speed of the last second, where the rate follows the speed
    last = [k for k in kin if kin[0][0] + HELD_S - 1.0 <= k[0] <= kin[0][0] + HELD_S] if kin else []
    held = sum(k[6] for k in last) / len(last) if last else float('nan')
    u_end = sum(k[1] for k in last) / len(last) if last else float('nan')
    pred_end = simulate(p, 'yaw', u_end, p['limit'], HELD_S)[1]
    ok = clean and math.isfinite(held) and abs(held - pred_end) <= max(RATE_TOL * abs(pred_end), HELD_ABS_DEG_S)
    verdict('요 스틱을 붙들면 기체가 파일이 말하는 선회를 유지한다', ok,
            'u %.2f m/s 에서 %.0f s 붙든 뒤 (마지막 1 s 의 u %.2f m/s) 요 각속도 %s deg/s, 그 속도의 예측 %+.2f (허용 %.0f %% 또는 %.1f deg/s)%s; 기록 %s'
            % (u_on, HELD_S, u_end, '?' if not math.isfinite(held) else '%+.2f' % held, pred_end, 100 * RATE_TOL, HELD_ABS_DEG_S,
               '' if clean else '; 기록이 스틱의 시작을 놓쳤다', 'sitl_out/fixed_flight_%s_yaw_held.csv' % MODEL_NAME))
    stream(RECOVER_S)

    # 7. pitch stick at speed: back is nose up
    for sign, name, want in ((-1.0, '뒤로', '위'), (+1.0, '앞으로', '아래')):
        kin, clean, u_on = leg(sticks, PITCH_S, 'pitch', 'pitch_%+d' % sign, pitch=sign)
        ok, peak, pred = peak_check(p, kin, clean, 'pitch', sign, PITCH_PEAK_S, u_on)
        tail = [k for k in kin if k[0] >= kin[-1][0] - 1.0] if kin else []
        angle = sum(k[4] for k in tail) / len(tail) if tail else float('nan')
        verdict('피치 스틱을 %s 밀면 기수가 %s로 파일이 말하는 만큼 간다' % (name, want), ok,
                'u %.2f m/s 에서 스틱 뒤 첫 %.0f s 의 피치 각속도 최대 %s deg/s, 예측 %+.1f (허용 %.0f %%)%s; %.1f s 뒤 피치 %s deg (판정 아님)'
                % (u_on, PITCH_PEAK_S, '?' if not math.isfinite(peak) else '%+.1f' % (peak * -sign), pred * -sign, 100 * RATE_TOL,
                   '' if clean else '; 기록이 스틱의 시작을 놓쳤다', PITCH_S, '?' if not math.isfinite(angle) else '%+.1f' % angle))
        stream(RECOVER_S)

    sticks.throttle = 0
    time.sleep(0.5)
    commander('disarm', '-f')
    sticks.stop = True
    return summary()


if __name__ == '__main__':
    sys.exit(main())
