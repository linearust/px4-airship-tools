#!/usr/bin/env python3
"""Option D: what the yaw rate integrator does when the pilot releases the
stick (the rate setpoint returns to zero). Built on the shared model; the
shared files are imported, never edited.

Rules (OptionD(rule=...)), each a change to AirshipAttitudeControl::Run()
between yawRateSetpoint() and controlYawRate():

  none        the firmware as it is (AAC:233-244): must reproduce the shared
              controller bit for bit (identity.py checks it).
  reset       resetIntegral(2) on the cycle the setpoint enters zero from a
              non-zero value (the deadzone entry) or changes sign (a reversal
              whose stick sample skipped the deadzone). Existing API only.
  reset_hold  the rover precedent (RoverControl.cpp:205-212): the integral is
              reset on every cycle the setpoint is zero; P still acts.
  bleed       integ *= exp(-dt / tau) on every cycle the setpoint is zero.
              Needs a RateControl method that scales or sets the integral
              (none exists) and a new time-constant parameter.
  cap         setIntegratorLimit(lim(|sp|)) every cycle, lim = L0 + (INT_LIM
              - L0) * (|sp| / sp_max)^n. The clamp acts inside updateIntegral
              (RC:115), i.e. on the cycle after the limit drops, exactly as
              calling the existing API each cycle would. New parameter L0.
  restore     on the cycle the setpoint leaves zero, remember the integral
              (the trim that held the vehicle with the stick centred); on the
              cycle it enters zero (or reverses), put that value back. Needs a
              RateControl setter; no new parameter.
  reset_freeze  reset as in `reset`, then stop integrating (update(...,
              landed = true), the existing API) until |rate| < r_th deg/s, the
              rate changes sign, the stick leaves the deadzone, or t_max s.
  restore_freeze  restore as in `restore`, then the same freeze.

The trigger for reset/restore/reset_freeze: sp == 0 and sp_prev != 0, or
sp * sp_prev < 0 (edge=`zero_or_sign`, default), or only the first
(edge=`zero`).
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, MODEL)
sys.dont_write_bytecode = True
os.environ['PYTHONDONTWRITEBYTECODE'] = '1'

import controller as ctl  # noqa: E402
import closed_loop as cl  # noqa: E402

TARGETS5 = ['gz_airship_vectored_independent', 'gz_airship_vectored_independent_am_fin_drag',
            'gz_cloudship', 'gz_cloudship_am_fin_drag', 'gz_airship_vectored_independent_fin']
SHORT = {'gz_airship_vectored_independent': 'vec rigid',
         'gz_airship_vectored_independent_am_fin_drag': 'vec complete',
         'gz_cloudship': 'cs rigid',
         'gz_cloudship_am_fin_drag': 'cs complete',
         'gz_airship_vectored_independent_fin': 'vec fin'}
AUTHORITY = {'vectored': 54.0, 'cloudship': 4.478}


def model_of(target):
    return (cl.TARGETS.get(target) or cl.EXTRA_TARGETS[target])[0]


def airframe_of(target):
    return (cl.TARGETS.get(target) or cl.EXTRA_TARGETS[target])[1]


class OptionD(ctl.AirshipYawController):
    """AirshipYawController.step() with one hook before RateControl::update."""

    def __init__(self, airframe='vectored', params=None, filter_gyro=True, gyro_bias=0.0, rule=None):
        self.rule = dict(rule or {'kind': 'none'})
        self._sp_prev = 0.0
        self._trim = 0.0
        self._frozen = False
        self._fz_rate0 = 0.0
        self._fz_t0 = 0
        self.t_us = 0
        self.events = []          # (t_us, what) for diagnostics
        super().__init__(airframe, params, filter_gyro, gyro_bias)

    # the integrator limit the firmware would set this cycle
    def _cap(self, sp):
        r = self.rule
        full = self.params['AS_YR_INT_LIM']
        L0 = r['L0']
        x = min(1.0, abs(sp) / self.yaw_rate_max) if self.yaw_rate_max > 0 else 0.0
        return L0 + (full - L0) * x ** r.get('n', 1.0)

    def _edge(self, sp):
        prev = self._sp_prev
        if self.rule.get('edge', 'zero_or_sign') == 'zero':
            return sp == 0.0 and prev != 0.0
        # the sign part only with a deadzone (proposal/: yawStickReleased); a no-op at MAN_DEADZONE 0.1
        return (sp == 0.0 and prev != 0.0) or (self.params['MAN_DEADZONE'] > 0.0 and sp * prev < 0.0)

    def _before_update(self, sp, rate, dt):
        kind = self.rule['kind']
        rc = self.rc

        if kind == 'none':
            return

        if kind == 'reset':
            if self._edge(sp):
                rc.integ = 0.0
                self.events.append((self.t_us, 'reset'))

        elif kind == 'reset_hold':
            if sp == 0.0:
                rc.integ = 0.0

        elif kind == 'bleed':
            if sp == 0.0:
                rc.integ *= math.exp(-dt / self.rule['tau'])

        elif kind == 'cap':
            rc.int_lim = self._cap(sp)

        elif kind == 'restore':
            if self._sp_prev == 0.0 and sp != 0.0:
                self._trim = rc.integ
            elif self._edge(sp):
                rc.integ = self._trim
                self.events.append((self.t_us, 'restore %.4f' % self._trim))

        elif kind == 'restore_scaled':
            # restore, and on every cycle |sp| falls without changing sign, shrink
            # the part above the trim by (|sp| / |sp_prev|)^n (n 2: quadratic hull damping)
            prev = self._sp_prev
            if prev == 0.0 and sp != 0.0:
                self._trim = rc.integ
            elif self._edge(sp):
                rc.integ = self._trim
                self.events.append((self.t_us, 'restore'))
            elif sp * prev > 0.0 and abs(sp) < abs(prev):
                rc.integ = self._trim + (rc.integ - self._trim) * (abs(sp) / abs(prev)) ** self.rule.get('n', 2.0)

        elif kind in ('reset_freeze', 'restore_freeze'):
            if kind == 'restore_freeze' and self._sp_prev == 0.0 and sp != 0.0:
                self._trim = rc.integ
            if self._edge(sp):
                rc.integ = 0.0 if kind == 'reset_freeze' else self._trim
                self._frozen = sp == 0.0
                self._fz_rate0 = rate
                self._fz_t0 = self.t_us
                self.events.append((self.t_us, kind))
            if self._frozen and (sp != 0.0 or abs(rate) < math.radians(self.rule.get('r_th', 1.0))
                                 or rate * self._fz_rate0 < 0.0
                                 or (self.t_us - self._fz_t0) * 1e-6 > self.rule.get('t_max', 5.0)):
                self._frozen = False
        else:
            raise ValueError(kind)

    def step(self, t_us, gyro_z, stick_yaw, mode='acro', armed=True, sticks_valid=True, status_topic=None):
        # AirshipYawController.step with the hook; every other line as there
        self.t_us = t_us
        rate = (self._lpf.apply(gyro_z) if self.filter_gyro else gyro_z) - self.gyro_bias
        self.rate = rate
        dt = ctl.constrain((t_us - self._last_run) * 1e-6, 0.000125, 0.02)
        self._last_run = t_us
        self.dt = dt
        usable = bool(armed) and bool(sticks_valid)
        torque = ctl.finite_or(stick_yaw, 0.0) if usable else 0.0
        active = ctl.yaw_rate_loop_active(mode, usable, self.propulsive_yaw)

        if active:
            self._update_saturation(status_topic)
            sp = ctl.yaw_rate_setpoint(stick_yaw, self.params['MAN_DEADZONE'], self.yaw_rate_max)
            self.rate_sp = sp
            if not self._was_active:
                self._sp_prev = 0.0
                self._trim = 0.0
                self._frozen = False
            self._before_update(sp, rate, dt)
            # frozen: RateControl::update(..., landed = true) skips updateIntegral (RC:80-83)
            t = self.rc.update(rate, sp, dt, landed=self._frozen)
            torque = t if ctl.isfinite(t) else 0.0
            self._sp_prev = sp
        elif self._was_active:
            self.rc.reset()
            self.rate_sp = 0.0
            self._sp_prev = 0.0
        else:
            self.rate_sp = 0.0

        self._was_active = active
        self.loop_active = active
        return torque


# ---------------------------------------------------------------------------
# actuator variants
# ---------------------------------------------------------------------------

class SlewVectoredAct(cl.VectoredAct):
    """The vectored pods with CA_AIRSHIP_TLT_R (the allocator's tilt slew,
    which the stiff gz joint PID follows) set to `slew_deg`."""

    def __init__(self, model, slew_deg, transport_ticks=0):
        super().__init__(model, transport_ticks)
        self.p = dict(self.p)
        self.p['slew'] = math.radians(slew_deg)
        self.act = self.va.VectoredActuation(self.p, transport_ticks)


class LaggedCloudshipAct(cl.CloudshipAct):
    """The Cloudship tail with a first-order thrust lag tau [s] after the
    allocator (gz's GenericMotorModel duty branch has none:
    cloudship_actuation.py:71-72; this is a sensitivity only). The allocator's
    saturation sign is unchanged (it is computed before the motor)."""

    def __init__(self, model, tau):
        super().__init__(model)
        self.tau = tau
        self.F = 0.0
        self.N = 0.0

    def step(self, dt, u):
        self.stage = dict(self.chain.stages(u))
        a = 1.0 - math.exp(-dt / self.tau)
        self.F += (self.stage['F'] - self.F) * a
        self.N += (self.stage['N'] - self.N) * a
        self.stage['F'] = self.F
        self.stage['N'] = self.N
        return self.N, self.stage['sat']


def build(target, rule=None, params=None, noise=False, seed=1, slew_deg=None, tail_tau=None, trace=False):
    """closed_loop.build() with the OptionD controller in place of the shared
    one (same airframe, params, filter setting) and an optional actuator
    variant."""
    kw = {'seed': seed} if noise else {}
    loop = cl.build(target, params=params, noise=noise, trace=trace, **kw)
    airframe = airframe_of(target)
    loop.controller.ctrl = OptionD(airframe, params, filter_gyro=False, rule=rule)

    if slew_deg is not None:
        assert airframe == 'vectored'
        loop.actuation = SlewVectoredAct(model_of(target), slew_deg)

    if tail_tau is not None:
        assert airframe == 'cloudship'
        loop.actuation = LaggedCloudshipAct(model_of(target), tail_tau)

    return loop


GAINS = {'P0.5': {'AS_YAWRATE_P': 0.5, 'AS_YAWRATE_I': 0.09},
         'P0.7': {'AS_YAWRATE_P': 0.7, 'AS_YAWRATE_I': 0.09}}
