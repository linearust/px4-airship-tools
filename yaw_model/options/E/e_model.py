#!/usr/bin/env python3
"""Option E: the vectored allocator's steer band, reported honestly or delivered.

Copies (subclasses) of the shared model's pod / allocator / actuation classes;
the shared files are imported, never edited.

Firmware being modelled (read, not changed):
  AirshipPod.cpp:49-70   steer(): engage 0.02, release 0.01, Holding / Released
  AirshipPod.cpp:185-200 drive() / heldByChoice(): a Holding or Released pod
                         whose tilt sits at its target calls its whole residual
                         "withheld by choice"
  ActuatorEffectivenessAirship.cpp:254-265 shortfall(): the same-signed held
                         share is subtracted before the sign is published, so a
                         demand the band withholds is NOT reported as saturation
                         (commit db417c83b7, 2026-09-07: without it a yaw
                         integrator froze below the engage floor and the 0.4 N m
                         weak-wind case could not be held).
  The band itself is commit 4d76cbc608 (2026-08-10): atan2 of a near-zero pod
  demand is noise; on the bench it slammed the servos across their range at
  neutral sticks (108-159 jumps in 341 s).

VARIANTS (the `band` argument):
  'base'         the firmware as it is.
  'report_hold'  the literal band: only a HOLDING pod (engaged, release <
                 |d| <= engage) stops calling its residual held by choice.
  'report_band'  honest report above the noise floor: any withheld demand of
                 magnitude > release (0.01), engaged or not, is published as
                 unallocated with its sign, so the rate controller stops
                 integrating toward it. Below release the discount stays
                 (db417c83b7's weak-wind case).
  'report_all'   honest report everywhere: no residual is ever held by choice
                 (Holding and Released both publish) = db417c83b7 reverted.
  'deliver'      deliver it: a demand above release whose held tilt realizes
                 nothing of it (realized(d, tilt) <= 0, i.e. a reversal) engages
                 steering at once, whether or not the pod was engaged, instead
                 of waiting for |d| > engage 0.02. Reporting as in 'base'.
  'deliver_report' 'deliver' plus 'report_band' (a reversal above release
                 swings; anything else withheld above release is reported).

Sensitivity knobs:
  slew_deg       CA_AIRSHIP_TLT_R and the physical servo together (matched).
  phys_slew_deg  a physical servo rate limit on the gz joint command while the
                 firmware keeps CA_AIRSHIP_TLT_R 120 (mismatched servo).
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, MODEL)
sys.dont_write_bytecode = True

import closed_loop as cl  # noqa: E402
import controller as ctl  # noqa: E402
import vectored_actuation as va  # noqa: E402

BANDS = ('base', 'report_hold', 'report_band', 'report_all', 'deliver', 'deliver_report')


class PodE(va.Pod):
    def __init__(self, tilt_min, tilt_max, slew_rate, band='base'):
        super().__init__(tilt_min, tilt_max, slew_rate)
        assert band in BANDS, band
        self.band = band
        self.swings = 0            # number of times the target moved by more than 90 deg
        self.deliver_engages = 0   # engages that only the 'deliver' rule made
        self.travel = 0.0          # total commanded tilt travel [rad]
        self.dir_reversals = 0     # the slewing direction reversed (servo chatter/hunting)
        self._last_dir = 0
        self.starts = 0            # the tilt started moving after standing still (stop-start chatter)
        self._moving = False

    def steer(self, d, dt):
        magnitude = math.hypot(d[0], d[1])
        engaged = self.mode in (va.STEERING, va.HOLDING)
        old_target = self.target
        deliver = self.band in ('deliver', 'deliver_report')

        if magnitude > self.ENGAGE:
            self.mode = va.STEERING
            self.target = self.steer_target(d, magnitude)
        elif deliver and magnitude > self.RELEASE and self.realized(d, self.tilt) <= 0.0:
            # the held tilt realizes nothing of an above-noise demand: swing now
            self.mode = va.STEERING
            self.target = self.steer_target(d, magnitude)
            if abs(self.target - old_target) > 1e-6:
                self.deliver_engages += 1
        elif engaged and magnitude > self.RELEASE:
            self.mode = va.HOLDING
        else:
            self.mode = va.RELEASED
            self.target = self.tilt

        if abs(self.target - old_target) > 0.5 * math.pi:
            self.swings += 1

        before = self.tilt
        self.slew_to_target(dt)
        moved = self.tilt - before
        self.travel += abs(moved)
        if abs(moved) > 1e-9:
            d = 1 if moved > 0 else -1
            if self._last_dir and d != self._last_dir:
                self.dir_reversals += 1
            self._last_dir = d
            if not self._moving:
                self.starts += 1
        self._moving = abs(moved) > 1e-9

    def held_by_choice(self):
        if self.band == 'report_all':
            return False
        if self.band == 'report_hold':
            # literal "inside the band while engaged": only a HOLDING pod reports
            return (self.can_tilt() and self.mode == va.RELEASED
                    and abs(self.tilt - self.target) < self.SETTLED)
        return super().held_by_choice()

    def drive(self, d, out_min, out_max):
        thrust, achieved, withheld = super().drive(d, out_min, out_max)
        if self.band in ('report_band', 'deliver_report') and math.hypot(d[0], d[1]) > self.RELEASE:
            # a demand above the noise floor that the pod withholds is reported:
            # only what lies below kSteerRelease stays "by choice"
            withheld = (0.0, 0.0)
        return thrust, achieved, withheld


class AllocatorE(va.Allocator):
    def __init__(self, p, band='base'):
        super().__init__(p)
        self.pods = [PodE(p['tilt_min'], p['tilt_max'], p['slew'], band) for _ in range(2)]


class VectoredActuationE(va.VectoredActuation):
    def __init__(self, p, transport_ticks=0, armed=True, band='base', phys_slew=None):
        super().__init__(p, transport_ticks, armed)
        self.alloc = AllocatorE(p, band)
        self.phys_slew = phys_slew          # rad/s on the gz joint command, None = none
        self._phys_cmd = [0.0, 0.0]

    def step(self, u, thrust=(0.0, 0.0, 0.0), roll=0.0):
        if self.phys_slew is None:
            return super().step(u, thrust, roll)
        # a slower physical servo: rate-limit the joint command gz receives
        joints = self.joints
        originals = [j.pre_update for j in joints]
        limit = self.phys_slew * self.dt

        def limited(i, orig):
            def f(cmd):
                if cmd is not None:
                    self._phys_cmd[i] += ctl.constrain(cmd - self._phys_cmd[i], -limit, limit)
                    cmd = self._phys_cmd[i]
                return orig(cmd)
            return f

        for i, j in enumerate(joints):
            j.pre_update = limited(i, originals[i])
        try:
            return super().step(u, thrust, roll)
        finally:
            for j, o in zip(joints, originals):
                j.pre_update = o


class VectoredActE(cl.VectoredAct):
    def __init__(self, model, transport_ticks=0, band='base', slew_deg=None, phys_slew_deg=None):
        super().__init__(model, transport_ticks)
        p = dict(self.p)
        if slew_deg is not None:
            p['slew'] = va.radians_f(slew_deg)
        self.p = p
        self.act = VectoredActuationE(p, transport_ticks, band=band,
                                      phys_slew=None if phys_slew_deg is None else math.radians(phys_slew_deg))

    def status(self):
        s = super().status()
        pods = self.act.alloc.pods
        s['swings'] = sum(pp.swings for pp in pods)
        s['deliver_engages'] = sum(pp.deliver_engages for pp in pods)
        s['travel_deg'] = math.degrees(sum(pp.travel for pp in pods))
        s['dir_reversals'] = sum(pp.dir_reversals for pp in pods)
        s['starts'] = sum(pp.starts for pp in pods)
        return s


class TracingLoop(cl.Loop):
    """closed_loop.Loop that also records the pods' state per tick."""

    def advance(self, dt):
        self._pending += dt
        while self._pending >= self.h - 1e-12:
            self._pending -= self.h
            meas = self._measured()
            self._torque = self.controller.update(self.h, meas, self._stick, self._mode,
                                                  self._armed, self._sticks_valid, self._unallocated)
            moment, self._unallocated = self.actuation.step(self.h, self._torque)
            if hasattr(self.plant, 'step_actuated'):
                self.plant.step_actuated(self.h, self.actuation, moment, self._wind)
            else:
                self.plant.step(self.h, moment + self._wind)
            self.sim += self.h
            if self.trace is not None:
                st = self.actuation.status()
                self.trace.append((self.sim, self._mode, self._stick, meas, self._torque,
                                   self.controller.ctrl.integ, moment, self._wind,
                                   self.controller.ctrl.rc.sat_pos, self.controller.ctrl.rc.sat_neg,
                                   st.get('mode_s'), st.get('mode_p'), st.get('tilt_s'), st.get('tilt_p'),
                                   self.plant.rate, st.get('swings')))


def build(target, band='base', params=None, noise=False, seed=1, trace=False, slew_deg=None,
          phys_slew_deg=None, **planar_options):
    """closed_loop.build() with the option-E vectored actuation (Cloudship: the
    shared CloudshipAct, untouched by this option)."""
    model, airframe = cl.TARGETS.get(target) or cl.EXTRA_TARGETS[target]
    controller = ctl.GateController(airframe, params, filter_gyro=False)

    if airframe == 'vectored':
        actuation = VectoredActE(model, 0, band, slew_deg, phys_slew_deg)
    else:
        actuation = cl.CloudshipAct(model)

    import planar
    body = planar.RollPlanarPlant(target, noise=noise, seed=seed, model=model, **planar_options)
    return TracingLoop(controller, actuation, body, trace)


def model_name(target):
    return (cl.TARGETS.get(target) or cl.EXTRA_TARGETS[target])[0]
