#!/usr/bin/env python3
"""The simplest closed loop gate.py can drive: a harness exercise, NOT a
model of the SITL yaw axis.

Controller: airship_att_control's yaw path with rate_control.cpp's PI, port
below with line numbers. Actuation: IDEAL - the normalized torque clipped to
+-1 times the vehicle's full-command yaw authority, instantly. No pod tilt
slew, no motor lag, no THR_MDL_FAC curve, no allocator pseudo-inverse. Plant:
one rigid yaw degree of freedom, rigid + added yaw inertia and the Hydrodynamics
yaw damping from the model files. No fins, no Munk moment, no sway coupling.
Those omissions are exactly what the real model parts have to add; this file
only proves that mode, stick, wind and parameter events reach a model the way
the gate issues them.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gate import BENCH, compose  # noqa: E402

if BENCH not in sys.path:
    sys.path.insert(0, BENCH)

import gz_model  # noqa: E402
import controller  # noqa: E402

# The yaml defaults (controller.DEFAULT_PARAMS, read from the files). Until
# 2026-10-01 this restated them, P 0.5 included; reference_loop() now takes
# a model's own set (controller.head_params: its airframe's FF and
# THR_MDL_FAC) unless it is handed one.
PARAMS = dict(controller.DEFAULT_PARAMS)


def deadzone(value, dz):
    """math::deadzone, src/lib/mathlib/math/Functions.hpp:125-133."""
    x = max(-1.0, min(1.0, value))
    dzc = max(0.0, min(0.99, dz))
    sign = (x > 0) - (x < 0)
    return (x - sign * dzc) / (1 - dzc) * (abs(x) > dzc)


class YawRateController:
    """airship_att_control_main.cpp:207-255 on the yaw axis."""

    def __init__(self, params=None):
        self.p = dict(PARAMS)
        self.p.update(params or {})
        self._int = 0.0
        self._published = False
        self._was_active = False

    def param(self, name):
        return self.p.get(name)

    def set_param(self, name, value):
        self.p[name] = value

    @property
    def integ(self):
        return self._int if self._published else None

    def update(self, dt, rate, stick, mode, armed, sticks_valid, unallocated):
        dt = max(0.000125, min(0.02, dt))                         # :215
        usable = armed and sticks_valid                            # :223, airship_manual_input.hpp:100-103
        torque = stick if usable else 0.0                          # :226, hpp:78-81
        # yawRateLoopActive (hpp:180-185): Acro and Stabilized have
        # flag_control_rates_enabled; propulsiveYaw is true on both gz
        # vehicles (vectored GRP > 0, Cloudship TAIL 1, no yaw surfaces).
        active = usable and mode in ('acro', 'stabilized')

        if active:
            sat_pos, sat_neg = unallocated > 1e-7, unallocated < -1e-7   # :181-190
            sp = deadzone(stick, self.p['MAN_DEADZONE']) * math.radians(self.p['AS_YAWRATE_MAX'])
            error = sp - rate                                       # rate_control.cpp:75
            torque = (self.p['AS_YAWRATE_P'] * error + self._int    # :78
                      + self.p.get('AS_YAWRATE_FF', 0.0) * sp)
            e = error                                               # :88-117
            if sat_pos:
                e = min(e, 0.0)
            if sat_neg:
                e = max(e, 0.0)
            factor = max(0.0, 1.0 - (e / math.radians(400.0)) ** 2)
            candidate = self._int + factor * self.p['AS_YAWRATE_I'] * e * dt
            limit = self.p['AS_YR_INT_LIM']
            self._int = max(-limit, min(limit, candidate))
            self._published = True                                  # main.cpp:244
        elif self._was_active:
            self._int = 0.0                                         # main.cpp:246-251
            self._published = True

        self._was_active = active
        return torque


class IdealAllocator:
    def __init__(self, authority_nm):
        self.authority = authority_nm

    def step(self, dt, torque_sp):
        served = max(-1.0, min(1.0, torque_sp))
        return served * self.authority, torque_sp - served


class RigidYaw:
    def __init__(self, izz, linear=0.0, quadratic=0.0):
        self.izz, self.linear, self.quadratic = izz, linear, quadratic
        self.rate = 0.0

    def step(self, dt, moment):
        damping = self.linear * self.rate + self.quadratic * self.rate * abs(self.rate)
        self.rate += (moment - damping) / self.izz * dt


def plant_numbers(model_name):
    """(Izz rigid + added, |nR|, |nRabsR|, authority), as sitl_check reads them (:303-306, :73)."""
    model = gz_model.merged(model_name)
    izz = gz_model.yaw_inertia(gz_model.family(model_name)) + gz_model.added_mass(model).get('rr', 0.0)
    terms = gz_model.hydrodynamics(model)
    return (izz, abs(terms.get('nR', 0.0)), abs(terms.get('nRabsR', 0.0)),
            gz_model.yaw_authority(model_name))


def reference_loop(model_name, params=None):
    izz, linear, quadratic, authority = plant_numbers(model_name)
    params = controller.head_params(model_name) if params is None else params
    return compose(YawRateController(params), IdealAllocator(authority), RigidYaw(izz, linear, quadratic))
