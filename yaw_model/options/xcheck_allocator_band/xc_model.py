#!/usr/bin/env python3
"""Cross-check of option E ("allocator-band"): an independent implementation of
its steer-band variants, written from option E's firmware snippets against
AirshipPod.cpp, NOT from options/E/e_model.py.

The shared model is imported, never edited. Only closed_loop.Loop is used
(not E's TracingLoop), so a difference between this file and e_model.py in
the gate lines means one of the two does not implement the stated change.

Variants (E's names):
  base          the firmware (vectored_actuation.Pod as is)
  report_hold   heldByChoice(): accept only TiltMode::Released
  report_band   drive(): by_choice = heldByChoice() && |demand| <= kSteerRelease
  report_all    drive(): withheld share always empty
  deliver       steer(): new branch before Holding:
                  magnitude > kSteerRelease && realized(demand, tilt()) <= 0
                  -> Steering, target = steerTarget(demand, magnitude)
  deliver_report  deliver + report_band

Extra knobs for the sensitivity runs:
  slew_deg        CA_AIRSHIP_TLT_R (the gz joint tracks its command closely,
                  so the slewed command IS the servo's rate)
  cloud_tau       a first-order lag [s] on the Cloudship tail's force and
                  moment (the shared chain has none: cloudship_actuation.py
                  "NO LAG")
  torque_noise    white noise std added to the rate controller's torque
                  output before the allocator (vibration / stick-noise proxy)
"""
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.abspath(os.path.join(HERE, '..', '..'))
if MODEL not in sys.path:
    sys.path.insert(0, MODEL)
sys.dont_write_bytecode = True

import closed_loop as cl  # noqa: E402
import controller as ctl  # noqa: E402
import vectored_actuation as va  # noqa: E402

VARIANTS = ('base', 'report_hold', 'report_band', 'report_all', 'deliver', 'deliver_report')


class XPod(va.Pod):
    def __init__(self, tilt_min, tilt_max, slew_rate, variant):
        super().__init__(tilt_min, tilt_max, slew_rate)
        self.variant = variant
        self.target_jumps = 0    # target moved > 90 deg in one tick
        self.slew_reversals = 0  # the slewing direction flipped
        self._last_dir = 0

    def steer(self, d, dt):
        # AirshipPod.cpp:49-70, with option E's 'deliver' branch inserted
        magnitude = math.hypot(d[0], d[1])
        engaged = self.mode in (va.STEERING, va.HOLDING)
        old = self.target
        if magnitude > self.ENGAGE:
            self.mode = va.STEERING
            self.target = self.steer_target(d, magnitude)
        elif (self.variant in ('deliver', 'deliver_report') and magnitude > self.RELEASE
              and self.realized(d, self.tilt) <= 0.0):
            self.mode = va.STEERING
            self.target = self.steer_target(d, magnitude)
        elif engaged and magnitude > self.RELEASE:
            self.mode = va.HOLDING
        else:
            self.mode = va.RELEASED
            self.target = self.tilt
        if abs(self.target - old) > 0.5 * math.pi:
            self.target_jumps += 1
        before = self.tilt
        self.slew_to_target(dt)
        mv = self.tilt - before
        if abs(mv) > 1e-9:
            s = 1 if mv > 0 else -1
            if self._last_dir and s != self._last_dir:
                self.slew_reversals += 1
            self._last_dir = s

    def held_by_choice(self):
        # AirshipPod.cpp:195-200
        if self.variant == 'report_all':
            return False
        modes = (va.RELEASED,) if self.variant == 'report_hold' else (va.HOLDING, va.RELEASED)
        return self.can_tilt() and self.mode in modes and abs(self.tilt - self.target) < self.SETTLED

    def drive(self, d, out_min, out_max):
        # AirshipPod.cpp:185-193 with E's report_band line
        axis = (math.cos(self.tilt), math.sin(self.tilt))
        thrust = va.clamp(d[0] * axis[0] + d[1] * axis[1], max(out_min, 0.0), out_max)
        achieved = (axis[0] * thrust, axis[1] * thrust)
        by_choice = self.held_by_choice()
        if self.variant in ('report_band', 'deliver_report'):
            by_choice = by_choice and math.hypot(d[0], d[1]) <= self.RELEASE
        withheld = (d[0] - achieved[0], d[1] - achieved[1]) if by_choice else (0.0, 0.0)
        return thrust, achieved, withheld


class XVectoredAct(cl.VectoredAct):
    def __init__(self, model, variant='base', slew_deg=None):
        super().__init__(model, 0)
        if slew_deg is not None:
            self.p = dict(self.p)
            self.p['slew'] = va.radians_f(slew_deg)
        self.act = va.VectoredActuation(self.p, 0)
        self.act.alloc.pods = [XPod(self.p['tilt_min'], self.p['tilt_max'], self.p['slew'], variant)
                               for _ in range(2)]

    def status(self):
        s = super().status()
        pods = self.act.alloc.pods
        s['jumps'] = sum(p.target_jumps for p in pods)
        s['slew_rev'] = sum(p.slew_reversals for p in pods)
        return s


class LaggedCloudshipAct(cl.CloudshipAct):
    """The shared Cloudship tail chain with a first-order lag on the tail's
    delivered force and moment (the gz plugin branch in use has none)."""

    def __init__(self, model, tau):
        super().__init__(model)
        self.tau = tau
        self._N = 0.0
        self._F = 0.0

    def step(self, dt, u):
        N, sat = super().step(dt, u)
        a = 1.0 - math.exp(-dt / self.tau)
        self._N += (N - self._N) * a
        self._F += (self.stage['F'] - self._F) * a
        self.stage = dict(self.stage)
        self.stage['N'] = self._N
        self.stage['F'] = self._F
        return self._N, sat


class NoisyTorque:
    """Wraps a GateController: adds white noise to the torque it returns
    (what the allocator sees), leaving the controller's own state alone."""

    def __init__(self, inner, std, seed):
        self.inner = inner
        self.std = std
        self.rng = random.Random(1000 + seed)

    def __getattr__(self, name):
        return getattr(self.inner, name)

    def update(self, *a, **k):
        tq = self.inner.update(*a, **k)
        if self.std > 0.0 and tq != 0.0:
            tq = tq + self.rng.gauss(0.0, self.std)
        return tq


def build(target, variant='base', params=None, noise=False, seed=1, slew_deg=None, cloud_tau=None,
          torque_noise=0.0, trace=False):
    model, airframe = cl.TARGETS.get(target) or cl.EXTRA_TARGETS[target]
    controller = ctl.GateController(airframe, params, filter_gyro=False)
    if torque_noise:
        controller = NoisyTorque(controller, torque_noise, seed)
    if airframe == 'vectored':
        actuation = XVectoredAct(model, variant, slew_deg)
    else:
        actuation = LaggedCloudshipAct(model, cloud_tau) if cloud_tau else cl.CloudshipAct(model)
    import planar
    body = planar.RollPlanarPlant(target, noise=noise, seed=seed, model=model)
    return cl.Loop(controller, actuation, body, trace)


def model_name(target):
    return (cl.TARGETS.get(target) or cl.EXTRA_TARGETS[target])[0]
