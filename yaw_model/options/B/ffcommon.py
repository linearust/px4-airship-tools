#!/usr/bin/env python3
"""Option B, yaw rate feedforward: shared helpers.

The shared model (../../closed_loop.py, controller.py, gate.py, ...) is only
imported, never edited. Feedforward is added the way the firmware proposal
would add it: RateControl already computes torque = P e + I_state + FF sp
(src/lib/rate_control/rate_control.cpp:78, _gain_ff from setFeedForwardGain,
rate_control.hpp:72); controller.RateControlYaw carries the same `ff` term.
Since 2026-10-01 the shared model is HEAD's, where this proposal landed
(d2647379af): AirshipYawController.parameters_updated() sets rc.ff from
AS_YAWRATE_FF through setGains (airship_yaw_rate_loop.hpp:58-69), so build()
passes FF as that parameter. Until then this file set rc.ff after build(),
which the old parameters_updated() left alone; the two are the same run.

Actuator sensitivity hooks (the gz model is untouched; they wrap it):
  slew        CA_AIRSHIP_TLT_R the allocator uses AND the pod's real slew
              (the gz JointPositionController tracks the slewed command
              within a few ms, so the allocator's slew IS the tilt slew)
  phys_slew   the tilt joint's command additionally rate-limited in the
              plant, the allocator still believing `slew` (a real servo
              slower than the parameter)
  tail_tau    a first-order lag on the Cloudship tail FORCE. The gz tail has
              none (GenericMotorModel duty_cycle branch, cloudship_actuation.py
              stage 4), so this is a real-motor assumption, not a gz number.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.abspath(os.path.join(HERE, '..', '..'))
os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
sys.dont_write_bytecode = True
for p in (MODEL, os.path.join(MODEL, 'review')):
    if p not in sys.path:
        sys.path.insert(0, p)

import closed_loop as cl  # noqa: E402
import gate  # noqa: E402
import reproduce as rp  # noqa: E402
from gain_sweep import ENSEMBLE  # noqa: E402  (the reviewers' 9 timing/noise members)

TARGETS5 = ['gz_airship_vectored_independent', 'gz_airship_vectored_independent_am_fin_drag',
            'gz_cloudship', 'gz_cloudship_am_fin_drag', 'gz_airship_vectored_independent_fin']
SHORT = {'gz_airship_vectored_independent': 'vec rigid',
         'gz_airship_vectored_independent_am_fin_drag': 'vec complete',
         'gz_cloudship': 'cs rigid',
         'gz_cloudship_am_fin_drag': 'cs complete',
         'gz_airship_vectored_independent_fin': 'vec fin'}
SP = (0.5 - 0.1) / (1 - 0.1) * 15.0          # sitl_check.py:240, deg/s
# The model's stated uncertainty on a rate-type number [deg/s], per plant
# (task statement / review): rigid vectored peak and brake 0.3-0.4; Cloudship
# complete 0.1-0.15 (pitch and heave held). The complete vectored and the fin
# hold-out are the same planar class with held pitch/heave: 0.15 assumed.
# Cloudship rigid: the noise-free peak bias bound, 0.12 (noise is on here).
UNC = {'gz_airship_vectored_independent': 0.4,
       'gz_airship_vectored_independent_am_fin_drag': 0.15,
       'gz_cloudship': 0.12,
       'gz_cloudship_am_fin_drag': 0.15,
       'gz_airship_vectored_independent_fin': 0.15}
# Timing members of the reviewers' ensemble; noise is switched on separately
TIMING = [(label, tkw) for label, tkw, bkw in ENSEMBLE if not bkw]


def model_name(target):
    return (cl.TARGETS.get(target) or cl.EXTRA_TARGETS[target])[0]


class LaggedTail(cl.CloudshipAct):
    """CloudshipAct with a first-order lag on the tail force (tau [s]); the
    allocator's saturation sign stays instantaneous, as in the firmware."""

    def __init__(self, model, tau):
        super().__init__(model)
        self.tau = tau
        self.F = 0.0

    def step(self, dt, u):
        _, sat = super().step(dt, u)
        self.F += (self.stage['F'] - self.F) * (1.0 - math.exp(-dt / self.tau))
        L, M, N = self.chain.wrench(self.F)
        self.stage = dict(self.stage, F=self.F, N=N)
        return N, sat


def _limit_joint(joint, rate_rad_s, dt):
    """Rate-limit a GzTiltJoint's position command (a servo slower than the
    allocator believes)."""
    orig = joint.pre_update
    state = {'cmd': None}

    def pre_update(cmd):
        if cmd is not None:
            if state['cmd'] is None:
                state['cmd'] = joint.cmd
            step = rate_rad_s * dt
            state['cmd'] += max(-step, min(step, cmd - state['cmd']))
            cmd = state['cmd']
        return orig(cmd)

    joint.pre_update = pre_update


def build(target, P=0.5, I=0.09, FF=0.0, noise=False, seed=1, slew=None, phys_slew=None, tail_tau=None,
          trace=False, int_lim=None, ffq=None):
    """ffq: a QUADRATIC feedforward K sp|sp| added to the torque instead of /
    on top of the linear one (not in RateControl: airship_att_control would
    add it to torque(2) after controlYawRate). Sensitivity variant only."""
    params = {'AS_YAWRATE_P': P, 'AS_YAWRATE_I': I, 'AS_YAWRATE_FF': FF}
    if int_lim is not None:
        params['AS_YR_INT_LIM'] = int_lim
    kw = {'noise': True, 'seed': seed} if noise else {}
    loop = cl.build(target, params=params, trace=trace, **kw)
    act = loop.actuation

    if ffq:
        rc = loop.controller.ctrl.rc
        linear_update = rc.update

        def update(rate, rate_sp, dt, landed=False):
            return linear_update(rate, rate_sp, dt, landed) + ffq * rate_sp * abs(rate_sp)

        rc.update = update

    if slew is not None:
        assert isinstance(act, cl.VectoredAct)
        for pod in act.act.alloc.pods:
            pod.slew = math.radians(slew)
            pod.slew_limited = pod.slew > 0.0

    if phys_slew is not None:
        assert isinstance(act, cl.VectoredAct)
        for j in act.act.joints:
            _limit_joint(j, math.radians(phys_slew), act.act.dt)

    if tail_tau:
        assert isinstance(act, cl.CloudshipAct)
        loop.actuation = LaggedTail(model_name(target), tail_tau)

    return loop


# ---------------------------------------------------------------------------
# sitl_check's own judgements, recomputed unrounded from the samples
# (sitl_check.py:150-175 tail_mean/settled; :371, :398-426, :432-446)
# ---------------------------------------------------------------------------

def tail_mean(rows, seconds, key='rate', signed=False):
    last = rows[-1]['t']
    vals = [r[key] if signed else abs(r[key]) for r in rows if r['t'] >= last - seconds]
    return sum(vals) / len(vals)


def settled(rows, threshold=1.0, hold=1.5):
    start = None
    for row in rows:
        if abs(row['rate']) < threshold:
            start = row['t'] if start is None else start
            if row['t'] - start >= hold:
                return start - rows[0]['t']
        else:
            start = None
    return None


def phases(samples):
    out = {}
    for r in samples:
        out.setdefault(r['phase'], []).append(r)
    return out


def metrics(res, target):
    """Per check: ok (the gate's own verdict line), the judged numbers and the
    margin to the threshold (> 0 inside, < 0 outside), unrounded."""
    parsed = rp.parse(res.gate_lines)
    ph = phases(res.samples)
    # clean_start's '정지 대기' rows are split between the two steps by order
    out = {}
    br = ph['Acro 제동']
    out['Acro brake'] = {'ok': parsed['Acro brake']['ok'], 'end': br[-1]['rate'],
                         'margin': 6.0 - abs(br[-1]['rate'])}

    for label in ('Acro', 'Stabilized'):
        key = 'Acro' if label == 'Acro' else 'Stab'
        push, rel = ph['%s 추종' % label], ph['%s 놓음' % label]
        mean = tail_mean(push, 3.0, signed=True)
        peak = max(r['rate'] for r in push)
        out[key + ' track'] = {'ok': parsed[key + ' track']['ok'], 'mean': mean, 'peak': peak,
                               'm_mean': 0.25 * SP - abs(mean - SP), 'm_peak': 1.4 * SP - peak,
                               'margin': min(0.25 * SP - abs(mean - SP), 1.4 * SP - peak)}
        r0 = rel[0]['rate']
        st = settled(rel)
        # the same rows with the rate threshold moved by the plant's uncertainty
        u = UNC[target]
        st_lo = settled(rel, threshold=1.0 - u)
        st_hi = settled(rel, threshold=1.0 + u)
        ok_lo = abs(r0) - u > 0.5 * SP and st_lo is not None and st_lo < 10.0
        ok_hi = abs(r0) + u > 0.5 * SP and st_hi is not None and st_hi < 10.0
        out[key + ' release'] = {'ok': parsed[key + ' release']['ok'], 'r0': r0, 'end': rel[-1]['rate'],
                                 'settle': st, 'm_turning': abs(r0) - 0.5 * SP,
                                 'margin': (10.0 - st) if st is not None else float('-inf'),
                                 'robust': ok_lo == ok_hi, 'settle_lo': st_lo, 'settle_hi': st_hi}

    for key, name_part in (('wind I on', 'I 켬: 적분기가'), ('wind I off', 'I 끔'), ('wind weak', '틸트 밴드 아래')):
        rows = [v for k, v in ph.items() if name_part in k and not k.endswith('회복')]
        if not rows:
            continue
        rows = rows[0]
        rate = tail_mean(rows, 2.0)
        integ = tail_mean(rows, 2.0, 'integ')
        needed = parsed[key]['needed']
        if key == 'wind I off':
            out[key] = {'ok': parsed[key]['ok'], 'rate': rate, 'integ': integ, 'margin': rate - 1.0}
        else:
            out[key] = {'ok': parsed[key]['ok'], 'rate': rate, 'integ': integ, 'needed': needed,
                        'm_rate': 1.0 - rate, 'm_integ': 0.5 * needed - abs(integ - needed),
                        'margin': 1.0 - rate}

    for key in ('wind check', 'Manual'):
        out[key] = {'ok': parsed[key]['ok']}

    for k, v in out.items():
        if k in ('wind check', 'Manual'):
            v['undecided'] = False
        elif k.endswith('release'):
            v['undecided'] = not v['robust']
        elif k == 'wind I on' or k == 'wind weak':
            v['undecided'] = abs(v['m_rate']) <= UNC[target]
        else:
            v['undecided'] = abs(v['margin']) <= UNC[target]
    return out


def run_gate(target, P, I, FF, timing=None, noise=True, seed=1, out_dir=None, **act):
    loop = build(target, P, I, FF, noise=noise, seed=seed, **act)
    od = out_dir or os.path.join(HERE, 'tmp_out', str(os.getpid()))
    os.makedirs(od, exist_ok=True)
    res = gate.run_gate(loop, model_name(target), gate.Timing(**(timing or {})), out_dir=od)
    return res, loop
