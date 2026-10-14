#!/usr/bin/env python3
"""The SITL yaw loop as one closed-loop model per gz target: controller +
actuation + plant, stepped on the 4 ms gz/IMU grid, drivable by gate.py.

    from closed_loop import build
    loop = build('gz_cloudship_am_fin_drag')              # a gate.ClosedLoop (roll plant)
    loop = build(target, params={'AS_YAWRATE_P': 0.8})    # other gains
    loop = build(target, plant='1dof')                    # the plant before the fix
    python3 closed_loop.py gz_cloudship_am_fin_drag       # one gate run, echoed

Nothing here is new physics: it wires the part models together. Every part
keeps its own citations; this file cites only the wiring.

  controller  controller.GateController: airship_att_control's yaw path cycle
              by cycle (deadzone, rate setpoint, loop gate, RateControl PI with
              i_factor, integrator limit, anti-windup from the allocator's
              published saturation sign 1-2 cycles old, reset when the loop
              opens). Its gyro filter is switched off here because the plant's
              sensor stage applies the same filter once per IMU sample, so the
              listener reads the filtered rate as PX4's px4-listener does
              (vehicle_angular_velocity IS the filtered rate:
              VehicleAngularVelocity.cpp:770-771, 939).
  actuation   vectored: vectored_actuation.VectoredActuation (AirshipPod port,
              the 120 deg/s tilt slew, the gz JointPositionController and
              MulticopterMotorModel rotor lag, N = lever x thrust + reaction
              + rotor momentum). pod_wrench() adds the pods' surge force and
              roll moment for the planar plants.
              Cloudship: cloudship_actuation.TailChain (allocator clamp and
              sign, THR_MDL_FAC sqrt, bridge duty, GenericMotorModel
              polynomial, lever about the centre of mass): N = K clamp(u);
              body_forces() gives the tail FORCE for the planar plants.
  plant       'roll' (default): planar.RollPlanarPlant, surge-sway-roll-yaw
              of the hull link origin with the rigid and added-mass inertia
              and Coriolis (the Munk moment), buoyancy roll stiffness, the
              Hydrodynamics damping rows, every lift-drag fin on its true
              velocity, the actuators' forces where they act.
              'planar': planar.PlanarPlant, the same without roll (3-DOF).
              '1dof': plants.get(target, fins=1) I r_dot = N - c_hull r|r|
              - N_fin(r), the plant part's model, integrated as gz does
              (explicit in the aerodynamic terms: Hydrodynamics.cc:477-627
              and LiftDrag.cc:545 compute the wrench in PreUpdate from the
              velocity at the start of the step; step lta.sdf:8, 0.004 s).
              Why the default is not '1dof': closed_loop_out.txt, "cause".

One tick (gate.compose, airship_att_control_main.cpp:235 reads the allocator
status before computing): controller on the latest gyro sample and the status
published on an earlier tick -> allocator/actuators -> one gz step of the
plant with the wind added -> the IMU sample at the end of the step -> the
40 Hz Butterworth -> vehicle_angular_velocity.

ASSUMPTIONS (the wiring's own; the parts list theirs)
 W1 One PX4 cycle per gz step, 0 steps of transport: the command computed
    from the IMU sample that ends step k acts during step k+1 (IMU 250 Hz:
    airship_base/model.sdf:217, cloudship/model.sdf:242; step 4 ms: lta.sdf:8).
    Both actuation parts make the same assumption.
 W2 The gyro is noise-free unless noise=True (gyro noise stddev
    0.0008726646 rad/s: cloudship/model.sdf:261, airship_base/model.sdf:236;
    seeded, so a run is reproducible). The EKF gyro bias is 0.
 W3 The gate's throttle, pitch and roll sticks are 0 (sitl_link.py:276-279),
    so the pods' common demand is 0 and the Cloudship's side motors are off.
    A forward leg (forward.py) sets them through Loop.set_sticks(): the
    thrust goes through airship_manual_input's mapping (controller.
    manual_thrust, AMI:70-73) into the same allocator ports, so a yaw demand
    shares the vectored pods with the collective thrust (a pod whose share
    would go negative swings through its tilt, AirshipPod.cpp:189 clamps its
    thrust at 0) and the Cloudship's side motors push the hull forward.
 W4 The wind is the gate's world-z couple on the model (gz_link.py:203-205),
    FRD -tau on the level hull (gate.GZ_WORLD_Z_TO_FRD).
"""
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gate  # noqa: E402  (puts the bench tools on sys.path, no bytecode)
import controller as ctl  # noqa: E402
import plant_registry  # noqa: E402
import plants  # noqa: E402

DT = 0.004                         # lta.sdf:8 max_step_size; IMU 250 Hz (model.sdf:217 / :242)
GYRO_NOISE = plants.GYRO_NOISE

def _targets(*keys):
    """{that day's gz target: (model name a home holds today, airframe key
    in controller.AIRFRAMES)} of the registry plants `keys` name: the target
    is what the records and tests call the plant, the model what the live
    lookups (airframe_for_model, head_params, sitl_check) take."""
    return {e.target: (e.model, e.family) for e in map(plant_registry.resolve, keys)}


# The four plants of interest, the ones plants.py holds numbers for.
TARGETS = _targets('airship_vectored_independent_rigid', 'airship_vectored_independent_am_fin_drag',
                   'cloudship_rigid', 'cloudship_am_fin_drag')
# Not one of the four plants of interest: a hold-out check. 2522 sources 2520
# (2522_gz_airship_vectored_independent_fin:11-13), so its controller and
# actuation are the vectored airship's; only the roll plant takes it.
EXTRA_TARGETS = _targets('airship_vectored_independent_fin')
# Built for the forward legs only (no gate record exists): the Cloudship with
# fins on the rigid hull. 2512 sources 2510 (2512_gz_cloudship_fin:11-13).
FORWARD_TARGETS = _targets('cloudship_fin')
ALL_TARGETS = dict(TARGETS, **EXTRA_TARGETS, **FORWARD_TARGETS)


# ---------------------------------------------------------------------------
# Actuation adapters: step(dt, u) -> (N FRD [N m], raw unallocated yaw sign)
# ---------------------------------------------------------------------------

class VectoredAct:
    """vectored_actuation.VectoredActuation behind compose()'s interface.

    The raw allocator sign is handed back; GateController publishes it on the
    allocator's >= 5 ms cadence (ControlAllocator.cpp:464) exactly as
    controller.SitlYawChain does, so VectoredActuation's own copy of that
    cadence (row['sat_seen']) is not used."""

    def __init__(self, model, transport_ticks=0):
        import vectored_actuation as va
        self.va = va
        self.p = va.load_params(model)
        self.act = va.VectoredActuation(self.p, transport_ticks)
        self.row = None
        self._hx_prev = None
        self.thrust = (0.0, 0.0, 0.0)       # the controller's thrust setpoint (FRD), the gate's 0
        self.roll = 0.0                     # its roll torque (the roll stick)

    def set_manual(self, thrust, roll):
        """The rest of the wrench the controller publishes with the yaw
        torque: thrust (x, y, z) FRD and the roll torque (AMI:70-83)."""
        self.thrust = tuple(thrust)
        self.roll = roll

    def step(self, dt, u):
        assert abs(dt - self.act.dt) < 1e-12, 'the pods run on the gz step'
        self.row = self.act.step(u, self.thrust, self.roll)
        return self.row['N'], self.act.alloc.unallocated[2]

    def body_forces(self):
        """FLU body force of the two pods during the step just taken [N]:
        each pod pushes along its tilt axis (cos th, 0, sin th) in the x-z
        plane (tilt axis (0 -1 0): airship_vectored_independent/model.sdf:44,
        :117), so the pair gives surge X and heave Z; the yaw moment is N."""
        r = self.row
        th_s, th_p = math.radians(r['tilt_gz_s']), math.radians(r['tilt_gz_p'])
        fx = r['T_s'] * math.cos(th_s) + r['T_p'] * math.cos(th_p)
        return fx, 0.0

    def _hx(self):
        # FLU x angular momentum of the spinning props, I_spin * signed joint
        # rate * cos(tilt): the x twin of VectoredActuation._Hz
        # (vectored_actuation.py:579-581)
        return sum(I * r.joint_vel * math.cos(j.theta)
                   for I, r, j in zip(self.act.I_spin, self.act.rotors, self.act.joints))

    def pod_wrench(self, dt):
        """(X, Y, Z, K, M, N) FLU about the hull link origin during the step
        just taken, for the planar plants. Pods at x 0, y -+1.5 (model.sdf:19,
        :92), each thrusting along (cos th, 0, sin th):
          X = sum T cos th; Z = sum T sin th;
          N = -row['N'] (FRD -> FLU; lever + reaction + rotor momentum, as
              VectoredActuation computes it);
          K = sum y T sin th (row['roll'])
              - sum td T kM cos th   (the reaction couple (0,0,-td T kM) along
                                      the thrust axis, MulticopterMotorModel.cc:655-662)
              - dHx/dt               (the props' momentum, as N_gyro for yaw).
        The pitch moment is left at 0 (pitch is held)."""
        r = self.row
        th = (math.radians(r['tilt_gz_s']), math.radians(r['tilt_gz_p']))
        T = (r['T_s'], r['T_p'])
        X = sum(t * math.cos(a) for t, a in zip(T, th))
        Z = sum(t * math.sin(a) for t, a in zip(T, th))
        react = -sum(rot.td * t * rot.kM * math.cos(a) for rot, t, a in zip(self.act.rotors, T, th))
        hx = self._hx()
        gyro = -(hx - (self._hx_prev if self._hx_prev is not None else hx)) / dt
        self._hx_prev = hx
        return (X, 0.0, Z, r['roll'] + react + gyro, 0.0, -r['N'])

    def status(self):
        pods = self.act.alloc.pods
        return {'tilt_s': math.degrees(pods[0].tilt), 'tilt_p': math.degrees(pods[1].tilt),
                'mode_s': pods[0].mode, 'mode_p': pods[1].mode}


class CloudshipAct:
    """cloudship_actuation.TailChain behind compose()'s interface: the tail's
    moment about the centre of mass and the allocator's shortfall sign.
    Collective pods get 0 demand in the gate (W3) and deliver nothing; with a
    throttle they are cloudship_actuation.CloudshipPods, whose wrench the
    planar plants add (pods.wrench()). The collective pods carry no yaw
    (EA:184-186), so the tail's command and the shortfall the loop reads are
    the tail chain's either way."""

    def __init__(self, model):
        import cloudship_actuation as ca
        self.chain = ca.TailChain(model if model.startswith('cloudship_am') else 'cloudship')
        self.pods = ca.CloudshipPods(model)
        self.stage = None
        self.thrust = (0.0, 0.0, 0.0)
        self.roll = 0.0

    def set_manual(self, thrust, roll):
        self.thrust = tuple(thrust)
        self.roll = roll

    def step(self, dt, u):
        self.stage = self.chain.stages(u)
        # the controller's whole setpoint, pitch torque included (AMI:80-83)
        self.pods.step(dt, [self.roll, -self.thrust[2], u, self.thrust[0], self.thrust[1], self.thrust[2]])
        return self.stage['N'], self.stage['sat']

    def body_forces(self):
        """The tail thrust as a body force [N, FLU]: F along the prop's axis
        (0, -sin(roll), cos(roll)) = +y port (cloudship/model.sdf:421, roll
        -90 deg), applied at the tail's x (TailChain.tail_pos)."""
        s = self.stage
        return 0.0, s['F'] * self.chain.axis[1]

    def status(self):
        return {'tail': self.stage['cmd'] if self.stage else 0.0}


# ---------------------------------------------------------------------------
# Plants: step(dt, N_frd, act) ; rate (FRD, true); measured_rate()
# ---------------------------------------------------------------------------

class Gyro:
    """The IMU sample at the end of each gz step through
    VehicleAngularVelocity's 2nd-order Butterworth at IMU_GYRO_CUTOFF 40 Hz
    on 250 Hz samples (controller.LowPassFilter2p, the port of
    LowPassFilter2p.hpp:61-109), with gz's gyro noise if asked (W2)."""

    def __init__(self, noise=False, seed=1):
        self.lpf = ctl.LowPassFilter2p(ctl.LOOP_HZ, ctl.DEFAULT_PARAMS['IMU_GYRO_CUTOFF'])
        self.rng = random.Random(seed) if noise else None
        self.value = 0.0

    def sample(self, rate):
        if self.rng:
            rate += self.rng.gauss(0.0, GYRO_NOISE)
        self.value = self.lpf.apply(rate)


class OneDofPlant:
    """plants.get(target, fins): I r_dot = N - c_hull r|r| - N_fin(r), FRD.
    Explicit Euler over the 4 ms step, as gz evaluates the plugins' wrench at
    the start of the step (Hydrodynamics.cc:477-627, LiftDrag.cc:545)."""

    def __init__(self, target, fins=1.0, noise=False):
        self.r_dot, self.params = plants.get(target, fins=fins)
        self.rate = 0.0
        self.gyro = Gyro(noise)

    def step(self, dt, moment_frd):
        self.rate += self.r_dot(self.rate, moment_frd) * dt
        self.gyro.sample(self.rate)

    def measured_rate(self):
        return self.gyro.value


class Loop(gate.compose):
    """gate.compose with the plant also told about the actuators' forces
    (the planar plant needs them; the 1-DOF plant ignores them) and a trace
    for diagnostics."""

    def __init__(self, controller, actuation, plant, trace=False):
        super().__init__(controller, actuation, plant, control_hz=1.0 / DT)
        self.trace = [] if trace else None
        # manual_control_setpoint's other sticks: throttle -1 is z = 0, the
        # gate's (W3), so the thrust is 0 unless a driver sets them
        self._throttle, self._pitch, self._roll = -1.0, 0.0, 0.0

    def set_sticks(self, throttle=None, pitch=None, roll=None):
        """manual_control_setpoint.throttle (-1..1, controller.throttle_stick
        turns a sitl_link thrust fraction into it), pitch and roll."""
        if throttle is not None:
            self._throttle = throttle
        if pitch is not None:
            self._pitch = pitch
        if roll is not None:
            self._roll = roll

    def advance(self, dt):
        self._pending += dt

        while self._pending >= self.h - 1e-12:
            self._pending -= self.h
            meas = self._measured()
            self._torque = self.controller.update(self.h, meas, self._stick, self._mode,
                                                  self._armed, self._sticks_valid, self._unallocated)

            if hasattr(self.actuation, 'set_manual'):
                # AAC:224-228: the thrust and the roll torque are the sticks
                # while the input is usable, zero otherwise
                usable = self._armed and self._sticks_valid
                thrust = ctl.manual_thrust(self._throttle, self._pitch) if usable else (0.0, 0.0, 0.0)
                self.actuation.set_manual(thrust, ctl.finite_or(self._roll, 0.0) if usable else 0.0)

            moment, self._unallocated = self.actuation.step(self.h, self._torque)

            if hasattr(self.plant, 'step_actuated'):      # planar: forces where they act
                self.plant.step_actuated(self.h, self.actuation, moment, self._wind)
            else:
                self.plant.step(self.h, moment + self._wind)

            self.sim += self.h

            if self.trace is not None:
                self.trace.append((self.sim, self._mode, self._stick, meas, self._torque,
                                   self.controller.ctrl.integ, moment, self._wind,
                                   self.controller.ctrl.rc.sat_pos, self.controller.ctrl.rc.sat_neg,
                                   getattr(self.plant, 'extra', lambda: {})()))


def build(target, plant='roll', fins=1.0, params=None, noise=False, trace=False, transport_ticks=0,
          restore=None, **planar_options):
    """A gate.ClosedLoop for one gz target.

    params None: the gains the target flies at HEAD, the yaml defaults with
    its posix airframe chain on top (controller.head_params: 2527 gets
    AS_YAWRATE_FF 1.0). params given: exactly those, over the yaml defaults
    (controller.DEFAULT_PARAMS) and nothing from the airframe but its gate
    parameters (controller.GATE_PARAMS: AS_YAWRATE_MAX, which the airframe
    sets since its default became 0), which is what a replay of a record
    wants (record_gains.gains gives a record's full set). THR_MDL_FAC, which
    only the gate reads, is the airframe's either way.
    restore: None models the firmware before ec89e6bd84 (every record under
    records/), 'zero_or_sign' HEAD's rule (controller.set_restore)."""
    model, airframe = ALL_TARGETS[target]
    number = ctl.airframe_for_model(model)
    bad = ctl.check_allocator_config(airframe, number)

    if bad:
        raise SystemExit('controller.AIRFRAMES[%r] is not airframe %d: %s' % (airframe, number, '; '.join(bad)))

    head = ctl.head_params(model)
    gains = head if params is None else dict(ctl.DEFAULT_PARAMS, **{k: head[k] for k in ctl.GATE_PARAMS}, **params)
    controller = ctl.GateController(airframe, gains, filter_gyro=False,
                                    other_params={'THR_MDL_FAC': head['THR_MDL_FAC']})

    if restore is not None:
        controller.ctrl.set_restore(restore)

    if airframe == 'vectored':
        actuation = VectoredAct(model, transport_ticks)
    else:
        if transport_ticks:
            raise ValueError('the Cloudship adapter models 0 transport steps only')
        actuation = CloudshipAct(model)

    if plant == '1dof':
        body = OneDofPlant(target, fins, noise)
    elif plant == 'planar':
        import planar
        body = planar.PlanarPlant(target, noise=noise, **planar_options)
    elif plant == 'roll':
        import planar
        body = planar.RollPlanarPlant(target, noise=noise, model=model, **planar_options)
    else:
        raise ValueError(plant)

    return Loop(controller, actuation, body, trace)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='one gate run of the closed-loop model')
    parser.add_argument('target', choices=sorted(ALL_TARGETS))
    parser.add_argument('--plant', default='roll', choices=('roll', 'planar', '1dof'))
    parser.add_argument('--fins', type=float, default=1.0)
    args = parser.parse_args()
    loop = build(args.target, args.plant, args.fins)
    result = gate.run_gate(loop, ALL_TARGETS[args.target][0], echo=True)
    sys.exit(result.status)
