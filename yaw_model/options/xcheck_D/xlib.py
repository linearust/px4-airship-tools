#!/usr/bin/env python3
"""Cross-check of option D ("integrator-release", rule `restore`), written
independently of options/D/option_d.py: the shared closed_loop.build() loop is
used unchanged, and only the yaw RateControl instance gets its class swapped
for one whose update() applies the release rule first. The shared
AirshipYawController.step() therefore runs as is (no copied step()).

Rule semantics follow the proposal diff (airship_manual_input.hpp
yawStickDeflected / yawStickReleased, AirshipAttitudeControl::restoreYawTrim,
called just before controlYawRate()):
  deflected: sp != 0 and sp_prev == 0           -> trim = integ (before update)
  released : (sp == 0 and sp_prev != 0) or (dz > 0 and sp * sp_prev < 0)
             restore -> integ = clamp(trim, +-lim); reset -> integ = 0
  sp_prev and trim cleared when the loop closes (the firmware clears them on
  the first closed cycle; the shared model calls rc.reset() on the first open
  cycle, and nothing touches rc in between, so clearing there is equivalent).
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, MODEL)
sys.dont_write_bytecode = True

import controller as ctl  # noqa: E402
import closed_loop as cl  # noqa: E402

T5 = ['gz_airship_vectored_independent', 'gz_airship_vectored_independent_am_fin_drag',
      'gz_cloudship', 'gz_cloudship_am_fin_drag', 'gz_airship_vectored_independent_fin']
SHORT = dict(zip(T5, ['vec rigid', 'vec complete', 'cs rigid', 'cs complete', 'vec fin']))


def spec(target):
    return cl.TARGETS.get(target) or cl.EXTRA_TARGETS[target]


class ReleaseRC(ctl.RateControlYaw):
    """RateControlYaw with the release rule in front of update()."""
    rule = 'none'
    dz = 0.1

    def _init_rule(self, rule, dz):
        self.rule = rule
        self.dz = dz
        self.sp_prev = 0.0
        self.trim = 0.0
        self.n_release = 0
        self.n_deflect = 0
        self.release_log = []        # (integ before, integ after)

    def update(self, rate, rate_sp, dt, landed=False):
        if self.rule != 'none':
            prev = self.sp_prev
            if rate_sp != 0.0 and prev == 0.0:
                self.trim = self.integ
                self.n_deflect += 1
            elif (rate_sp == 0.0 and prev != 0.0) or (self.dz > 0.0 and rate_sp * prev < 0.0):
                before = self.integ
                if self.rule == 'restore':
                    if ctl.isfinite(self.trim):
                        self.integ = ctl.constrain(self.trim, -self.int_lim, self.int_lim)
                elif self.rule == 'reset':
                    self.integ = 0.0
                else:
                    raise ValueError(self.rule)
                self.n_release += 1
                self.release_log.append((before, self.integ))
            self.sp_prev = rate_sp
        return super().update(rate, rate_sp, dt, landed)

    def reset(self):
        super().reset()
        self.sp_prev = 0.0
        self.trim = 0.0


def install_rule(loop, rule):
    ctrl = loop.controller.ctrl
    rc = ctrl.rc
    rc.__class__ = ReleaseRC
    rc._init_rule(rule, ctrl.params['MAN_DEADZONE'])
    return rc


# ---------------------------------------------------------------------------
# actuator variants
# ---------------------------------------------------------------------------

def set_alloc_slew(loop, deg_s):
    """CA_AIRSHIP_TLT_R = deg_s: the allocator's slew (the gz joint PD then
    follows that ramp, as with the 120 default)."""
    for pod in loop.actuation.act.alloc.pods:
        pod.slew = math.radians(deg_s)
        pod.slew_limited = deg_s > 0


def slow_servo(loop, deg_s):
    """The allocator keeps CA_AIRSHIP_TLT_R 120 but the physical tilt servo
    can only move deg_s: the joint's position command is rate limited after
    the allocator (a mismatch the allocator does not know about)."""
    step = math.radians(deg_s) * cl.DT
    for j in loop.actuation.act.joints:
        orig = j.pre_update
        state = {'cmd': None}

        def pre_update(cmd, orig=orig, state=state, j=j):
            if cmd is not None:
                cur = state['cmd'] if state['cmd'] is not None else j.cmd
                cmd = cur + ctl.constrain(cmd - cur, -step, step)
                state['cmd'] = cmd
            return orig(cmd)

        j.pre_update = pre_update


def tail_lag(loop, tau):
    """First-order lag on the Cloudship tail's rotor reference (the prop
    speed), thrust = the gz polynomial of the lagged reference; the
    allocator's saturation sign is unchanged. gz has no lag here
    (cloudship/model.sdf:527-535); the classic template's prop tau is 0.025."""
    act = loop.actuation
    chain = act.chain
    st = {'ref': 0.0}
    a = 1.0 - math.exp(-cl.DT / tau)

    def step(dt, u):
        s = dict(chain.stages(u))
        st['ref'] += (s['ref'] - st['ref']) * a
        F = chain.thrust(st['ref'])
        s['F'] = F
        s['L'], s['M'], s['N'] = chain.wrench(F)
        act.stage = s
        return s['N'], s['sat']

    act.step = step


def build(target, rule='none', P=0.7, I=0.09, noise=True, seed=1, variant=None, extra_params=None):
    params = {'AS_YAWRATE_P': P, 'AS_YAWRATE_I': I}
    params.update(extra_params or {})
    kw = {'seed': seed} if noise else {}
    loop = cl.build(target, params=params, noise=noise, **kw)
    install_rule(loop, rule)
    if variant:
        kind, val = variant
        if kind == 'alloc_slew':
            set_alloc_slew(loop, val)
        elif kind == 'servo':
            slow_servo(loop, val)
        elif kind == 'tail_tau':
            tail_lag(loop, val)
        else:
            raise ValueError(kind)
    return loop
