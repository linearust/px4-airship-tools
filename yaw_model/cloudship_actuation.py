#!/usr/bin/env python3
"""Cloudship yaw actuation in gz SITL: normalized yaw torque u -> yaw moment N.

    python3 cloudship_actuation.py            # the full printout
    from cloudship_actuation import yaw_moment, K_YAW_NM, TailChain
        yaw_moment(u)          N = K clamp(u, -1, 1), for a loop model
        TailChain().stages(u)  every intermediate (tail cmd, sat sign, s, duty, ref, F, L/M/N)
        TailChain().respond(u_of_t, t_end, t_start, extra_steps=0|1)  N(t) on the 4 ms gz grid

u is vehicle_torque_setpoint.xyz[2] (FRD, + = nose right) as airship_att_control
publishes it; N is the yaw moment the gz plant receives about the vehicle's
centre of mass, FRD, N m. Everything outside this directory is only READ.

THE CHAIN, stage by stage (P = /home/h/dev/PX4-Autopilot, M = P/Tools/simulation/gz/models)

 0. u is published once per gyro sample and is NOT bounded by the rate
    controller: P/src/lib/rate_control/rate_control.cpp:78 sums P, I, D, FF with
    no constraint; P/src/modules/airship_att_control/airship_att_control_main.cpp
    :238 (torque(2) = controlYawRate) and :255 (publishTorqueSetpoint).
 1. Allocator, collective grouping with a tail (2510_gz_cloudship:79-81,86):
    - demand(YAW) = u - CA_AIRSHIP_CS_K * surface torque; surface torque is 0 here
      because CA_SV_CS_COUNT defaults to 0 (control_allocator/module.yaml:254-268)
      and 2510 does not set it
      (ActuatorEffectivenessAirship.cpp:171-179).
    - Collective: differential = {} so both pods get the same demand
      (ActuatorEffectivenessAirship.cpp:181-184), identical steer/drive, so
      podWrench YAW = 0.5*(port - starboard) = 0 (:159, :211).
    - tail = constrain(demand_yaw - 0, actuator_min(TAIL), actuator_max(TAIL))
      (:217-218). TAIL is motor index 2 (ActuatorEffectivenessAirship.hpp:62).
    - actuator_min(TAIL) = -1 because CA_R_REV = 4 sets bit 2
      (2510_gz_cloudship:86; ControlAllocator.cpp:526-528); max = +1 (:551).
    - trim: zero (Configuration.trim is zero-initialised, the motors are added
      with zero effectiveness at ActuatorEffectivenessAirship.cpp:79-81), and
      irrelevant anyway because updateSetpoint() overwrites actuator_sp(TAIL)
      after allocate() (ControlAllocator.cpp:437-442). clipActuatorSetpoint()
      only uses the trim when max < min (ControlAllocation.cpp:78-90).
    - no slew: CA_R2_SLEW defaults to 0 (control_allocator/module.yaml:84-99),
      2510 does not set it.
    - saturation feedback to the rate loop: sign of (demand - achieved) with a
      1e-4 deadband (ActuatorEffectivenessAirship.cpp:267-279, :303;
      ActuatorEffectivenessAirship.hpp:131), published at most every 5 ms
      (ControlAllocator.cpp:464), read at airship_att_control_main.cpp:177-192.
 2. Thrust model, all motors, THR_MDL_FAC = 1 (2510_gz_cloudship:65):
    s = sign(c) * sqrt(|c|) for |c| > FLT_EPSILON, c passed through otherwise
    (P/src/lib/mixer_module/functions/FunctionMotors.hpp:83-107). The tail is
    reversible (reversible_flags = CA_R_REV, ControlAllocator.cpp:754), so it is
    NOT remapped [0,1] -> [-1,1] (FunctionMotors.hpp:109-119).
 3. Output mapping: motor functions take the linear branch,
    duty = interpolate(s, -1, 1, MIN3, MAX3) (mixer_module.cpp:540-568; the centre
    branch at :556-561 is servos/gimbal/wheel only), interpolate() clamps at the
    ends (lib/mathlib/math/Functions.hpp:152-165). MIN3 1100, MAX3 1900, DIS3 1500
    (2510_gz_cloudship:57,60,61). No SIM_GZ_EC_REV (mixer_module.cpp:198-203,
    :547-549). No arming ramp: the gz ESC MixingOutput is built with ramp_up =
    false (gz_bridge/GZMixingInterfaceESC.hpp:81). The float duty goes out as
    Actuators.velocity[2] (GZMixingInterfaceESC.cpp:85-87; FUNC3 = 103,
    2510_gz_cloudship:54; <actuator_number>2</actuator_number>, M/cloudship/model.sdf:542).
 4. GenericMotorModel, force_polynomial + duty_cycle + bidirectional
    (M/cloudship/model.sdf:536-552; cloudship_am carries the identical block, and
    cloudship_am_fin_drag = cloudship_am + cloudship_am_fin + its drag block):
    - ref = (duty - (min + (max-min)*bi/2)) / ((max-min) - (max-min)*bi/2)
          = (duty - 1500) / 400, only when duty >= minDutyCycle
      (P/src/modules/simulation/gz_plugins/generic_motor/GenericMotorModel.cpp:713-719;
      below it the raw duty would be taken as ref, :721-722, which the clamp in
      stage 3 makes unreachable).
    - thrust = +sum p_i ref^i for ref >= minCommand (0), -sum n_i |ref|^i below
      (:747-761), p = n = [0, 0, 2.3544, 0] (model.sdf:550-551), so
      F = 2.3544 * ref * |ref| N along the prop link's +Z (:763).
    - no reaction torque: no torque polynomial -> zeros (:456-459, :474-477)
      (the "torque" at :782 is added as a force along Z, a plugin quirk, zero here).
    - NO LAG: this branch uses ref directly; the first-order rotor filter and
      timeConstantUp/Down exist only in the kVelocity branch (:911-914), and the
      model does not set them (model.sdf:527-535 says so too).
    - the plugin holds ref between messages (:693 only updates on a message).
    - AddWorldForce(force) acts at the link's centre of mass
      (/usr/include/gz/sim8/gz/sim/Link.hh:332-336); prop_tail has no inertial
      pose, so that is the link origin (-1.88, 0, 0.3352), rolled -90 deg so its
      +Z is body +Y, port (model.sdf:421).
 5. Moment about the composite centre of mass (all links, via gz_model):
    com = (0.02197, 0, 0.40397) in the y-port z-up model frame; r = (-1.90197, 0,
    -0.06877); r x (0, F, 0) = (+0.06877 F, 0, -1.90197 F) FLU = FRD roll +0.06877 F,
    yaw +1.90197 F. So N = 1.90197 * 2.3544 * ref|ref|.

Stages 2 and 4 cancel exactly: ref = s = sign(u) sqrt|u|, ref|ref| = u. Hence

    N = K * clamp(u, -1, 1),  K = 2.3544 * 1.90197 = 4.47801 N m,

linear in u, symmetric, with a hard saturation at |u| = 1 and no dynamics
other than the sample/transport timing below. (The only departures: |u| <=
FLT_EPSILON = 1.19e-7 passes un-rooted, a 3e-14 N dead spot, and float32
rounding of the duty; both are quantified by linearity_table().)

TIMING (assumptions, stated): the gz step is 4 ms (worlds/lta.sdf:8), the IMU is
250 Hz (model.sdf:242), so one u per step; gz free-runs at RTF 1 and PX4 follows
its clock (gz_bridge/GZBridge.cpp:342-359), there is no lockstep handshake. A
command computed from the IMU sample at the end of step k is applied during step
k+1 if it arrives before that step's PreUpdate, else one step later. So the
model is a zero-order hold per 4 ms step plus 0 or 1 extra step of delay
(EXTRA_STEPS, default 0). Not measured here.

SIDE MOTORS AND TILT: no yaw moment in the gate. The gate never sets the
throttle (sitl_link.py:274-279 starts it at 0, sitl_check.py never changes it),
so thrust_x = (throttle + 1)/2 = 0 and thrust_z = pitch stick = 0
(airship_manual_input.hpp:70, working tree); both pods get 0, MulticopterMotorModel runs at
0 rad/s (SIM_GZ_EC_MIN1/2 0, 2510_gz_cloudship:55-56), and every term it applies
(thrust, reaction couple, rotor drag, rolling moment) is proportional to the
rotor speed or its square. With thrust, the collective still gives no yaw:
equal commands, props at y = +-0.45 (model.sdf:336,373), ccw/cw (:474,:492) so
the couples cancel, and the tilt axis is -Y (:325) so the thrust stays in the
x-z plane. The residual is rotor-drag yaw damping, side_motor_yaw() below.
"""
import csv
import glob
import math
import os
import re
import sys

import numpy as np

PX4 = '/home/h/dev/PX4-Autopilot'
TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, TOOLS)
import gz_model  # noqa: E402  (pure module: reads model.sdf files, no simulator)

AIRFRAME = os.path.join(PX4, 'ROMFS/px4fmu_common/init.d-posix/airframes/2510_gz_cloudship')
HERE = os.path.dirname(os.path.abspath(__file__))
SITL_OUT = os.path.join(TOOLS, 'sitl_out')

F32 = np.float32
FLT_EPSILON = float(np.finfo(np.float32).eps)      # 1.1920929e-07, as FLT_EPSILON in C
DT_GZ = 0.004                                       # worlds/lta.sdf:8 max_step_size
DT_CTRL = 0.004                                     # model.sdf:242 IMU 250 Hz -> one u per step
EXTRA_STEPS = 0                                     # 0 or 1, see TIMING above
SAT_DEADBAND = 1e-4                                 # ActuatorEffectivenessAirship.hpp:131
TAIL = 2                                            # ActuatorEffectivenessAirship.hpp:62


def airframe_params(path=AIRFRAME):
    """param set-default NAME VALUE lines of one airframe file, as strings."""
    params = {}

    with open(path) as handle:
        for line in handle:
            m = re.match(r'\s*param set-default (\w+) (\S+)', line)

            if m:
                params[m.group(1)] = m.group(2)

    return params


class TailChain:
    """The Cloudship tail from u to the wrench, every number read from the files."""

    def __init__(self, model='cloudship'):
        p = airframe_params()
        self.params = p
        self.ca_r_rev = int(p['CA_R_REV'])                          # 2510:86
        self.thr_mdl_fac = float(p['THR_MDL_FAC'])                  # 2510:65
        self.ec_min = int(p['SIM_GZ_EC_MIN3'])                      # 2510:57
        self.ec_max = int(p['SIM_GZ_EC_MAX3'])                      # 2510:60
        self.ec_dis = int(p['SIM_GZ_EC_DIS3'])                      # 2510:61
        assert p['SIM_GZ_EC_FUNC3'] == '103' and p['CA_AIRSHIP_TAIL'] == '1' and p['CA_AIRSHIP_GRP'] == '0'
        assert 'SIM_GZ_EC_REV' not in p and 'CA_R2_SLEW' not in p and 'CA_SV_CS_COUNT' not in p
        # allocator range (ControlAllocator.cpp:526-531, :551)
        self.u_min = -1.0 if self.ca_r_rev & (1 << TAIL) else 0.0
        self.u_max = 1.0

        merged = gz_model.merged(model)
        plugin = gz_model.motor_plugin(merged, 'prop_tail')
        get = lambda tag: plugin.find(tag).text.strip()
        assert get('motorType') == 'force_polynomial' and get('controlMethod') == 'duty_cycle'
        assert int(get('actuator_number')) == TAIL
        self.min_duty = float(get('minDutyCycle'))
        self.max_duty = float(get('maxDutyCycle'))
        self.bidirectional = int(get('bidirectionalMotor'))
        self.min_command = float(get('minCommand'))
        poly = lambda tag: [float(c) for c in get(tag).strip('[]').split(',')]
        self.pos_poly = poly('positiveThrustPolynomial')
        self.neg_poly = poly('negativeThrustPolynomial')
        # plugin elements that would add dynamics or a couple, if they were present
        self.lag_tags = [t for t in ('timeConstantUp', 'timeConstantDown') if plugin.find(t) is not None]
        self.torque_tags = [t for t in ('positiveTorquePolynomial', 'negativeTorquePolynomial')
                            if plugin.find(t) is not None]

        tail = gz_model.named(merged, 'link', 'prop_tail')
        self.tail_pos = tuple(gz_model.pose(tail, i) for i in range(3))   # model frame, y port z up
        rpy = tuple(gz_model.pose(tail, i) for i in range(3, 6))
        self.mass, self.com = gz_model.centre_of_mass(gz_model.family(model))
        # thrust axis: link +Z rotated by the link's roll (pitch and yaw are 0 here)
        assert abs(rpy[1]) < 1e-9 and abs(rpy[2]) < 1e-9
        roll = rpy[0]                                                     # radians (gz_model.pose honours degrees="true")
        self.axis = (0.0, -math.sin(roll), math.cos(roll))               # R_x(roll) * (0,0,1)
        self.full_thrust = sum(self.pos_poly)                             # at ref = 1
        self.lever = self.com[0] - self.tail_pos[0]
        self.K = self.full_thrust * self.lever

    # ---- stage 1: allocator -------------------------------------------------
    def allocator(self, u):
        """(tail command, saturation sign the rate loop sees)."""
        demand = F32(u)                                             # no surfaces, no pod yaw
        cmd = F32(min(max(demand, F32(self.u_min)), F32(self.u_max)))
        short = float(demand - cmd)
        sat = 1.0 if short > SAT_DEADBAND else -1.0 if short < -SAT_DEADBAND else 0.0
        return cmd, sat

    # ---- stage 2: THR_MDL_FAC ----------------------------------------------
    def thrust_model(self, c, fac=None):
        fac = self.thr_mdl_fac if fac is None else fac
        c = F32(c)

        if fac > FLT_EPSILON and fac <= 1.0:
            a, b = F32(fac), F32(1.0 - fac)
            tmp1 = b / (F32(2) * a)
            tmp2 = b * b / (F32(4) * a * a)

            if c > FLT_EPSILON:
                c = -tmp1 + np.sqrt(tmp2 + c / a, dtype=F32)
            elif c < -FLT_EPSILON:
                c = tmp1 - np.sqrt(tmp2 - c / a, dtype=F32)

        assert self.ca_r_rev & (1 << TAIL)                          # reversible: no [0,1]->[-1,1] remap
        return F32(c)

    # ---- stage 3: bridge output mapping ------------------------------------
    def duty(self, s):
        lo, hi, s = F32(self.ec_min), F32(self.ec_max), F32(s)

        if s <= F32(-1):
            return lo
        if s > F32(1):
            return hi

        a = (hi - lo) / (F32(1) - F32(-1))
        b = lo - a * F32(-1)
        return F32(a * s + b)

    # ---- stage 4: GenericMotorModel ----------------------------------------
    def ref(self, duty):
        duty = float(duty)                                          # static_cast<double>

        if duty >= self.min_duty:
            span = self.max_duty - self.min_duty
            return ((duty - (self.min_duty + span * self.bidirectional / 2.0))
                    / (span - span * self.bidirectional / 2.0))

        return duty                                                 # :721-722, unreachable here

    def thrust(self, ref):
        if ref >= self.min_command:
            return sum(c * ref ** i for i, c in enumerate(self.pos_poly))
        if ref <= -self.min_command:
            return -sum(c * abs(ref) ** i for i, c in enumerate(self.neg_poly))
        return 0.0

    # ---- stage 5: wrench about the composite centre of mass, FRD ------------
    def wrench(self, F):
        r = [self.tail_pos[i] - self.com[i] for i in range(3)]
        f = [F * a for a in self.axis]
        m = (r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0])
        return (m[0], -m[1], -m[2])                                 # FLU -> FRD

    def stages(self, u, fac=None):
        cmd, sat = self.allocator(u)
        s = self.thrust_model(cmd, fac)
        duty = self.duty(s)
        ref = self.ref(duty)
        F = self.thrust(ref)
        L, M, N = self.wrench(F)
        return dict(u=u, cmd=float(cmd), sat=sat, s=float(s), duty=float(duty), ref=ref, F=F, L=L, M=M, N=N)

    def moment(self, u, fac=None):
        """Static map: yaw moment [N m, FRD] for a held u."""
        return self.stages(u, fac)['N']

    # ---- dynamics: timing only ---------------------------------------------
    def respond(self, u_of_t, t_end, t_start=0.0, extra_steps=EXTRA_STEPS, tau=0.0, fac=None):
        """[(t_k, N applied during [t_k, t_k + DT_GZ))] on the gz step grid.

        u_of_t(t_k) is the controller's output from the IMU sample at t_k (one
        per gz step). With extra_steps = 0 it acts on [t_k, t_k + dt), which is
        what happens when PX4 answers before gz's next PreUpdate; with 1 it acts
        one step later. tau > 0 adds a first-order lag the gz plant does NOT
        have (GenericMotorModel.cpp:741-789 uses ref directly): sensitivity only."""
        n = int(round((t_end - t_start) / DT_GZ))
        times = [t_start + k * DT_GZ for k in range(n)]
        # commands already in flight when the window opens, oldest first
        pending = [self.moment(u_of_t(t_start - (j + 1) * DT_GZ), fac) for j in range(extra_steps)][::-1]
        state = self.moment(u_of_t(t_start - (extra_steps + 1) * DT_GZ), fac)
        out = []

        for t in times:
            pending.append(self.moment(u_of_t(t), fac))
            target = pending.pop(0)

            if tau > 0:
                state += (target - state) * (1 - math.exp(-DT_GZ / tau))
            else:
                state = target

            out.append((t, state))

        return out


def servo_output_range():
    """(min, max) of a gz bridge servo output with SIM_GZ_SV_MIN/MAX unset:
    the standard_params defaults of the SIM_GZ_SV group
    (src/modules/simulation/gz_bridge/module.yaml:23-31). 2510 sets neither."""
    import yaml
    with open(os.path.join(PX4, 'src/modules/simulation/gz_bridge/module.yaml'), encoding='utf-8') as handle:
        groups = yaml.safe_load(handle)['actuator_output']['output_groups']
    sv = [g for g in groups if g.get('param_prefix') == 'SIM_GZ_SV']
    assert len(sv) == 1, 'gz_bridge/module.yaml: %d SIM_GZ_SV groups' % len(sv)
    std = sv[0]['standard_params']
    return float(std['min']['default']), float(std['max']['default'])


class CloudshipPods:
    """The two collective side thrusters, for forward flight: the allocator's
    pod half for the Cloudship airframe, the bridge, and the gz rotors and
    tilt joint, one 4 ms step per step(). The gate never drives them (throttle
    0, so every term below is 0 there); a forward leg does.

    Allocator: controller.AirshipAllocator('cloudship'), the port of
    ActuatorEffectivenessAirship::updateSetpoint() with GRP 0 + tail: both
    pods get the common demand (thrust_x, -thrust_z) and no differential
    (EA:183-186), so they steer and drive identically and give no yaw; the
    tail's half of the same allocation is TailChain's, which this class does
    not touch (the tail serves demand_yaw minus the pods' yaw, EA:219, and the
    pods' yaw is 0 here by construction). Motors 1 and 2 are pod motors,
    non-reversible (CA_R_REV 4 sets only the tail's bit); one tilt servo.
    Bridge: THR_MDL_FAC and SIM_GZ_EC_MIN1/MAX1 (2510_gz_cloudship:55-65) ->
    rotor speed, servo -> SIM_GZ_SV_MINA1/MAXA1 (:70-72), the same functions
    as the vectored pods (vectored_actuation.esc_output, servo_angle).
    gz: MulticopterMotorModel on prop_starboard / prop_port and the
    JointPositionController on tilt_joint (cloudship/model.sdf:321-333,
    :457-502), the vectored pods' GzRotor / GzTiltJoint with this model's
    numbers. The tilt joint turns the rod and both props, which sit on its
    axis: J = rod iyy + the props' iyy (pitched 90 deg about y, so their own
    y is the joint's).

    wrench() is the pods' generalized force at the link origin O (FLU) during
    the step just taken: each thrust T along (cos th, 0, sin th) at its prop's
    position, the rotor reaction -td T kM along the same axis
    (MulticopterMotorModel.cc:655-662) and -dH/dt of the spinning props. The
    two pods cancel each other in roll and yaw by symmetry; that is computed
    here, not assumed. Rotor drag (rotorDragCoefficient, the x500's) is left
    out: in straight flight the props' axis is along the flow, so it acts
    only on sideslip and yaw rate, at 8e-5 N per (rad/s rotor speed x m/s).
    """

    def __init__(self, model='cloudship', armed=True):
        import controller as ctl
        import vectored_actuation as va
        self.va = va
        p = airframe_params()
        assert p['CA_AIRSHIP_GRP'] == '0' and p['SIM_GZ_EC_FUNC1'] == '101' and p['SIM_GZ_EC_FUNC2'] == '102'
        assert p['SIM_GZ_SV_FUNC1'] == '201'
        self.fac = float(p['THR_MDL_FAC'])
        self.ec_min = [float(p['SIM_GZ_EC_MIN1']), float(p['SIM_GZ_EC_MIN2'])]
        self.ec_max = [float(p['SIM_GZ_EC_MAX1']), float(p['SIM_GZ_EC_MAX2'])]
        self.sv_mina, self.sv_maxa = float(p['SIM_GZ_SV_MINA1']), float(p['SIM_GZ_SV_MAXA1'])
        self.sv_min, self.sv_max = servo_output_range()
        self.alloc = ctl.AirshipAllocator('cloudship')
        assert self.alloc.num_tilt_servos == 1 and not self.alloc.independent and self.alloc.has_tail

        merged = gz_model.merged(model)
        rod = gz_model.named(merged, 'link', 'thruster_rod')
        joint = gz_model.named(merged, 'joint', 'tilt_joint')
        servo = [q for q in merged.findall('plugin') if 'joint-position-controller' in q.get('filename', '')
                 and q.find('joint_name').text.strip() == 'tilt_joint']
        assert len(servo) == 1 and [float(v) for v in joint.find('axis/xyz').text.split()] == [0.0, -1.0, 0.0]
        servo = servo[0]
        text = lambda e, tag, d=None: e.find(tag).text.strip() if e.find(tag) is not None else d
        props = []

        for side in ('starboard', 'port'):                                 # PX4 motor index 0, 1
            link = gz_model.named(merged, 'link', 'prop_' + side)
            motor = gz_model.motor_plugin(merged, 'prop_' + side)
            assert gz_model.MULTICOPTER in motor.get('filename', '')
            props.append({
                'pos': tuple(gz_model.pose(link, i) for i in range(3)),
                'iyy': float(link.find('inertial/inertia/iyy').text),
                'I_spin': float(link.find('inertial/inertia/izz').text),
                'turning': text(motor, 'turningDirection'), 'motor_number': int(text(motor, 'motorNumber')),
                'tau_up': float(text(motor, 'timeConstantUp')), 'tau_down': float(text(motor, 'timeConstantDown')),
                'max_rot': float(text(motor, 'maxRotVelocity')), 'kF': float(text(motor, 'motorConstant')),
                'kM': float(text(motor, 'momentConstant')),
                'slowdown': float(text(motor, 'rotorVelocitySlowdownSim', '10')),
            })

        assert [q['motor_number'] for q in props] == [0, 1]
        tilt = {'p': float(text(servo, 'p_gain', '1')), 'i': float(text(servo, 'i_gain', '0.1')),
                'd': float(text(servo, 'd_gain', '0.01')), 'cmd_max': float(text(servo, 'cmd_max', '1000')),
                'J': float(rod.find('inertial/inertia/iyy').text) + sum(q['iyy'] for q in props),
                'damping': float(text(joint, 'axis/dynamics/damping', '0')),
                'lower': float(text(joint, 'axis/limit/lower')), 'upper': float(text(joint, 'axis/limit/upper'))}
        self.props = props
        self.full_thrust = [q['kF'] * q['max_rot'] ** 2 for q in props]
        self.joint = va.GzTiltJoint(tilt, DT_GZ)
        self.rotors = [va.GzRotor(q, DT_GZ) for q in props]
        self.armed = armed
        self.thrust = [0.0, 0.0]
        self.theta = 0.0
        self.motors = [0.0, 0.0]
        self.servo = 0.0
        self._w = (0.0,) * 6

    def _H(self):
        """FLU angular momentum of the spinning props, along their axis."""
        c, s = math.cos(self.joint.theta), math.sin(self.joint.theta)
        h = sum(q['I_spin'] * r.joint_vel for q, r in zip(self.props, self.rotors))
        return (h * c, 0.0, h * s)

    def step(self, dt, control_sp):
        """control_sp = [roll, pitch, yaw, thrust_x, thrust_y, thrust_z] as the
        controller publishes it; one gz step. Returns the wrench at O."""
        assert abs(dt - DT_GZ) < 1e-12, 'the pods run on the gz step'
        self.alloc.update(control_sp, dt, armed=self.armed)
        self.motors = list(self.alloc.motors[:2])
        self.servo = self.alloc.servos[0]
        angle = self.va.servo_angle(self.servo, self.sv_mina, self.sv_maxa, self.sv_min, self.sv_max)
        omega = [self.va.esc_output(c, self.fac, self.ec_min[i], self.ec_max[i]) for i, c in enumerate(self.motors)]
        th = self.joint.theta                                          # forces from the start of the step
        H0 = self._H()
        self.joint.pre_update(angle)

        for rotor, w in zip(self.rotors, omega):
            rotor.pre_update(w)

        self.thrust = [r.thrust for r in self.rotors]
        self.theta = th
        axis = (math.cos(th), 0.0, math.sin(th))
        w = [0.0] * 6

        for q, rotor, T in zip(self.props, self.rotors, self.thrust):
            F = (T * axis[0], 0.0, T * axis[2])
            x, y, z = q['pos']
            moment = (y * F[2] - z * F[1], z * F[0] - x * F[2], x * F[1] - y * F[0])
            react = -rotor.td * T * q['kM']
            for k in range(3):
                w[k] += F[k]
                w[3 + k] += moment[k] + react * axis[k]

        self.joint.physics()

        for rotor in self.rotors:
            rotor.physics()

        H1 = self._H()

        for k in range(3):
            w[3 + k] -= (H1[k] - H0[k]) / dt

        self._w = tuple(w)
        return self._w

    def wrench(self):
        return self._w


def side_motor_yaw(thrust_frac, tilt_deg, r, model='cloudship'):
    """Yaw moment [N m, FRD] of the two collective side motors at yaw rate r
    [rad/s, FRD], from the MulticopterMotorModel terms that do not cancel.

    Assumes gz-sim's MulticopterMotorModel has the same velocity-branch terms as
    GenericMotorModel.cpp:791-921 (thrust ~ w^2, drag force -|w| k_d V_perp,
    rolling moment -|w| k_r V_perp, couple -dir*thrust*momentConstant). Equal
    commands (collective) and opposite directions cancel thrust-line yaw and the
    couples; rotor drag on the rotation-induced velocity does not cancel: it is
    a damping -2 |w| k_d (px^2 + py^2 sin^2 tilt) r. Rolling moments from the
    rotation cancel between the two props (odd in py)."""
    merged = gz_model.merged(model)
    plug = gz_model.motor_plugin(merged, 'prop_port')
    k_d = float(plug.find('rotorDragCoefficient').text)
    w_max = float(plug.find('maxRotVelocity').text)
    _, com = gz_model.centre_of_mass(gz_model.family(model))
    link = gz_model.named(merged, 'link', 'prop_port')
    px = gz_model.pose(link, 0) - com[0]
    py = gz_model.pose(link, 1) - com[1]
    # THR_MDL_FAC 1: command c -> rotor speed w_max * sqrt(c), thrust linear in c
    w = w_max * math.sqrt(max(thrust_frac, 0.0))
    a = math.radians(tilt_deg)
    return -2.0 * w * k_d * (px * px + py * py * math.sin(a) ** 2) * r


def linearity_table(chain):
    rows = []

    for u in (1e-7, 1.2e-7, 1e-6, 1e-5, 1e-4, 1e-3, 0.005, 0.0185, 0.05, 0.065, 0.1, 0.3, 0.5, 0.9, 1.0):
        for sign in (1, -1):
            N = chain.moment(sign * u)
            rows.append((sign * u, N, N / (sign * u) / chain.K - 1.0))

    return rows


def sitl_checks(chain):
    """Cross-checks against recorded runs (read only). Returns printable lines."""
    lines = []
    izz_rigid = gz_model.yaw_inertia(gz_model.family('cloudship'))
    rr = gz_model.added_mass(gz_model.merged('cloudship_am')).get('rr', 0.0)
    P = 0.5                                                          # AS_YAWRATE_P default in the runs

    # (a) wind hold: steady u that holds a known external torque, rest plants only
    lines.append('(a) wind hold, rigid-hull-at-rest runs: u_ss = integrator (P term <= P*|r|)')
    target = None
    seen = []

    for path in sorted(glob.glob(os.path.join(SITL_OUT, 'chain*_2026092*.txt'))
                       + glob.glob(os.path.join(SITL_OUT, 'variants_gz_cloudship*_sitl_check.txt'))
                       + glob.glob(os.path.join(SITL_OUT, 'gate_cloudship_*.txt'))):
        with open(path, errors='replace') as handle:   # the chain logs are cut mid-UTF-8 by `cut -c`
            for line in handle:
                m = re.search(r'=== gate gz_(\S+) ===', line) or re.search(r'기체: (\S+),', line)

                if m:
                    target = m.group(1)
                    continue

                m = re.search(r'바람 ([\d.]+) N m, I 켬.*?\|각속도\| ([\d.]+) deg/s, 적분기 ([\d.]+)', line)

                if m and target and target.startswith('cloudship'):
                    wind, rate, integ = float(m.group(1)), float(m.group(2)), float(m.group(3))
                    seen.append((os.path.basename(path), target, wind, rate, integ))

    for name, target, wind, rate, integ in seen:
        if rate >= 0.1 or integ >= 0.1:
            lines.append('    %-40s %-22s |r| %.2f deg/s integ %.4f  excluded (not at rest / wound up)'
                         % (name, target, rate, integ))
            continue

        pterm = P * math.radians(rate)
        lo, hi = wind / (integ + 5e-5 + pterm), wind / (integ - 5e-5 - pterm)
        lines.append('    %-40s %-22s |r| %.2f deg/s integ %.4f -> K = %.3f (%.3f..%.3f) N m/unit'
                     % (name, target, rate, integ, wind / integ, lo, hi))

    u_quad = math.sqrt(0.082963 / chain.K)
    lines.append('    linear law predicts u_ss = %.4f; a quadratic law (THR_MDL_FAC 0) would need %.4f'
                 % (0.082963 / chain.K, u_quad))

    # (b) Manual passthrough u = 0.5 on the complete plant (samples.csv, per the task: cloudship_am_fin_drag)
    lines.append('(b) Manual passthrough u = 0.5, wall-clock samples (RTF 1.00 per rtf_probe_summary.txt)')
    csvs = [(os.path.join(SITL_OUT, 'samples.csv'), 'cloudship_am_fin_drag')]
    csvs += [(p, re.search(r'samples_gz_(\S+)_chain9\.csv', os.path.basename(p)).group(1))
             for p in glob.glob(os.path.join(SITL_OUT, 'samples_gz_*_chain9.csv'))]

    for path, model in csvs:
        if not model.startswith('cloudship') or not os.path.exists(path):
            continue

        rows = [r for r in csv.DictReader(open(path, errors='replace')) if r['phase'] == 'Manual 통과'
                and r['torque'] not in ('', 'nan') and abs(float(r['torque']) - 0.5) < 1e-6]

        if len(rows) < 4:
            continue

        t = [float(r['t']) for r in rows]
        y = [math.radians(float(r['rate'])) for r in rows]
        n = len(t)
        mt, my = sum(t) / n, sum(y) / n
        slope = sum((a - mt) * (b - my) for a, b in zip(t, y)) / sum((a - mt) ** 2 for a in t)
        merged = gz_model.merged(model)
        ieff = izz_rigid + gz_model.added_mass(merged).get('rr', 0.0)
        hyd = gz_model.hydrodynamics(merged)
        damp = sum(abs(hyd.get('nRabsR', 0.0)) * v * abs(v) + abs(hyd.get('nR', 0.0)) * v for v in y) / n
        k_raw = ieff * slope / 0.5
        k_cor = (ieff * slope + damp) / 0.5
        lines.append('    %-12s %-22s n %d, rate %+.1f..%+.1f deg/s, slope %.2f deg/s^2, I_eff %.4f'
                     % (os.path.basename(path), model, n, math.degrees(y[0]), math.degrees(y[-1]),
                        math.degrees(slope), ieff))
        lines.append('        K = I*slope/u = %.3f; with the hull nRabsR term added back %.3f N m/unit'
                     ' (fin moments not modelled; odd in r, the window straddles 0)' % (k_raw, k_cor))
        lines.append('        predicted slope with K %.3f: %.2f deg/s^2 (quadratic law: %.2f)'
                     % (chain.K, math.degrees(0.5 * chain.K / ieff), math.degrees(0.25 * chain.K / ieff)))

    # (c) full command through actuator_test, rigid + added mass, no damping
    lines.append('(c) actuator_test tail +-1 on gz_cloudship_am, sim-clock slope (rtf_probe_gz_cloudship_am.txt, 09-23)')
    ieff = izz_rigid + rr
    pred = math.degrees(chain.K / ieff)
    path = os.path.join(SITL_OUT, 'rtf_probe_gz_cloudship_am.txt')
    text = open(path, errors='replace').read() if os.path.exists(path) else ''

    legs = {}

    for label in ('tail +1', 'tail -1'):
        m = re.search(re.escape(label) + r'.*?시뮬 ([\d.]+) s.*?요율 ([+-][\d.]+) -> ([+-][\d.]+) deg/s, 기울기 ([+-][\d.]+)', text)

        if m:
            span, r0, r1, got = float(m.group(1)), float(m.group(2)), float(m.group(3)), abs(float(m.group(4)))
            legs[label] = (span, r0, r1, got)
            lines.append('    %s: LS slope %.1f deg/s^2 vs %.2f predicted (I_eff %.4f) -> K = %.3f (%+.1f %%), %.1f -> %.1f deg/s'
                         % (label, got, pred, ieff, math.radians(got) * ieff, 100 * (got / pred - 1), r0, r1))

    if len(legs) == 2:
        # The +1 leg spins up 0 -> 145, the -1 leg spins down 150 -> 0 over the same |r|.
        # A term that opposes the rotation lowers the first and raises the second, so
        # their mean is the actuation and half their difference that term. A late
        # command start barely moves a 6 s LS slope (26.33 with 0.29 s / 0.48 s of dead
        # start fits 26.15 / 25.84) but does move the endpoints: solve for it.
        up, down = legs['tail +1'][3], legs['tail -1'][3]
        mean, opp = (up + down) / 2, (down - up) / 2
        late_up = legs['tail +1'][0] - abs(legs['tail +1'][2] - legs['tail +1'][1]) / up
        late_down = legs['tail -1'][0] - abs(legs['tail -1'][2] - legs['tail -1'][1]) / down
        lines.append('    mean of the two legs %.2f deg/s^2 -> K = %.3f (%+.1f %%); opposing term %.2f deg/s^2;'
                     ' endpoints then imply a %.2f / %.2f s late start (the released window also rose'
                     ' 145.4 -> 150.0, ~0.18 s of +1 after the window)'
                     % (mean, math.radians(mean) * ieff, 100 * (mean / pred - 1), opp, late_up, late_down))

    lines.append('    weak check: 0..150 deg/s is 20x the gate rate. Nothing in the actuation chain is inexact at'
                 ' |u| = 1 (s, duty, ref, F all exact), so the ~4 % is plant-side or the probe; unresolved here')

    lines.append('    rigid I %.4f (gz_model.yaw_inertia) + added rr %.5f (cloudship_am model.sdf)' % (izz_rigid, rr))
    return lines


def yaw_moment(u):
    """Convenience for loop models: N [N m, FRD] = K * clamp(u, -1, 1), the exact
    closed form of TailChain.moment() (float32 rounding aside)."""
    return K_YAW_NM * max(-1.0, min(1.0, u))


K_YAW_NM = 2.3544 * 1.9019738   # checked against TailChain().K in main()


def main():
    chain = TailChain('cloudship')
    assert abs(chain.K - K_YAW_NM) < 1e-5, (chain.K, K_YAW_NM)
    am = TailChain('cloudship_am_fin_drag')
    print('Cloudship tail actuation, gz SITL (2510_gz_cloudship + models/cloudship*)')
    print('  CA_R_REV %d -> tail range [%g, %g]; THR_MDL_FAC %g; SIM_GZ_EC_MIN3/MAX3/DIS3 %d/%d/%d'
          % (chain.ca_r_rev, chain.u_min, chain.u_max, chain.thr_mdl_fac, chain.ec_min, chain.ec_max, chain.ec_dis))
    print('  plugin duty %g..%g bidirectional %d minCommand %g, thrust poly +%s -%s'
          % (chain.min_duty, chain.max_duty, chain.bidirectional, chain.min_command, chain.pos_poly, chain.neg_poly))
    print('  plugin lag elements present: %s; torque polynomials present: %s'
          % (chain.lag_tags or 'none (no lag)', chain.torque_tags or 'none (no couple)'))
    print('  mass %.4f kg, com (%.5f, %.5f, %.5f), tail at %s, thrust axis %s (+Y = port)'
          % ((chain.mass,) + chain.com + (chain.tail_pos, tuple(round(a, 6) for a in chain.axis))))
    print('  lever %.5f m x full thrust %.4f N = K %.5f N m per unit u' % (chain.lever, chain.full_thrust, chain.K))
    print('  complete plant (cloudship_am_fin_drag) gives the same: K %.5f, com %s' % (am.K, tuple(round(c, 5) for c in am.com)))
    L, M, N = chain.wrench(chain.full_thrust)
    print('  wrench at u = +1, FRD: roll %+.4f, pitch %+.4f, yaw %+.4f N m' % (L, M, N))
    print()

    print('Stages for a held u (float32 where PX4 is float):')
    print('  %8s %8s %4s %10s %10s %10s %9s %9s %9s' % ('u', 'tail', 'sat', 's', 'duty', 'ref', 'F [N]', 'N [N m]', 'N/(K u)'))

    for u in (0.1, 0.3, 1.0, -0.1, -0.3, -1.0, 1.5, -1.5, 0.0):
        st = chain.stages(u)
        ratio = st['N'] / (chain.K * u) if u else float('nan')
        print('  %+8.3f %+8.3f %+4.0f %+10.6f %10.4f %+10.6f %+9.5f %+9.5f %9.6f'
              % (u, st['cmd'], st['sat'], st['s'], st['duty'], st['ref'], st['F'], st['N'], ratio))
    print()

    print('Step responses: u steps at t = 0 from 0 (controller samples every %.0f ms, gz step %.0f ms)'
          % (DT_CTRL * 1e3, DT_GZ * 1e3))
    header = '  %8s' % 't [ms]'
    cases = []

    for u in (0.1, 0.3, 1.0):
        step = lambda t, u=u: u if t >= -1e-12 else 0.0
        cases.append(('u=%.1f' % u, chain.respond(step, 0.024, -0.008)))
        cases.append(('u=%.1f +1st' % u, chain.respond(step, 0.024, -0.008, extra_steps=1)))

    for name, _ in cases:
        header += ' %12s' % name
    print(header)

    for k in range(len(cases[0][1])):
        line = '  %8.0f' % (cases[0][1][k][0] * 1e3)

        for _, series in cases:
            line += ' %12.5f' % series[k][1]
        print(line)

    print('  (+1st = the command misses its gz step and lands one 4 ms step later)')
    print('  final N/u: ' + ', '.join('%s %.5f' % (n, s[-1][1] / float(n.split('=')[1].split()[0]))
                                     for n, s in cases[::2]))
    print('  time to 63 % of final: 0 ms after the step it lands on (no lag in the plugin branch used)')
    print()

    print('Counterfactuals (NOT the gz plant; sensitivity only):')

    for u in (0.1, 0.3, 1.0):
        quad = chain.moment(u, fac=0.0)
        lag = chain.respond(lambda t, u=u: u if t >= 0 else 0.0, 0.2, 0.0, tau=0.025)
        t63 = next(t for t, n in lag if n >= 0.632 * chain.K * u)
        print('  u %.1f: THR_MDL_FAC 0 -> N %.4f (%.0f %% of linear); a tau 0.025 s prop lag -> 63 %% on the step'
              ' starting at %.0f ms' % (u, quad, 100 * quad / (chain.K * u), t63 * 1e3))
    print()

    print('Linearity (float32 chain): N/(K u) - 1')
    for u, N, dev in linearity_table(chain):
        print('  u %+11.4g  N %+12.6g N m  dev %+.2e' % (u, N, dev))
    print('  |u| <= FLT_EPSILON (1.19e-7) is not rooted and its duty rounds to 1500 in float32: N = 0, the only'
          ' non-linear spot; float32 duty rounding is < 2e-4 relative for |u| >= 1e-6')
    print()

    print('Side motors / collective tilt yaw (FRD), rotor-drag damping, per rad/s of yaw rate:')
    for thr in (0.0, 0.25, 1.0):
        print('  thrust %.2f: ' % thr + ', '.join('tilt %+3d deg %.4f' % (a, side_motor_yaw(thr, a, 1.0))
                                               for a in (0, 45, 90)) + ' N m/(rad/s)')
    print('  gate: throttle 0 -> 0 exactly. Compare the loop P action on the tail, 0.5 x K = %.3f N m/(rad/s)'
          % (0.5 * chain.K))
    print()

    print('SITL cross-checks (recorded runs, read only):')
    for line in sitl_checks(chain):
        print(line)


if __name__ == '__main__':
    main()
