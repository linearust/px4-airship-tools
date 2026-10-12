#!/usr/bin/env python3
"""Option C: 2-DOF setpoint weighting on the yaw-rate P term.

    u = P (b r_sp - r) + I integral(r_sp - r)
      = P (r_sp - r) + I integral(r_sp - r) + FF r_sp,   FF = P (b - 1)

which is lib/rate_control's existing feedforward term (rate_control.cpp:78,
`_gain_ff.emult(rate_sp)`), set once per parameter update. The model's
RateControlYaw already carries that term (controller.py:249-263, `ff`), so
this file only sets it; no shared model file is edited.

Firmware shape of the proposal (NOT implemented, for the owner to decide):
in AirshipAttitudeControl::parameters_updated() (airship_att_control_main.cpp
:93-99) add
    _rate_control.setFeedForwardGain(Vector3f(0.f, 0.f,
            _param_as_yawrate_p.get() * (_param_as_yawrate_b.get() - 1.f)));
with a new AS_YAWRATE_B (dimensionless, 0..1, default 1.0 = today's loop).
It is modelled the same way: the hook below recomputes ff whenever the
controller's parameters_updated() runs (the gate's AS_YAWRATE_I 0 / restore
goes through it and leaves ff consistent).

Also here: the actuator variants for the sensitivity runs and the gate
margin extraction shared by the scripts in this directory.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, MODEL)
sys.path.insert(0, os.path.join(MODEL, 'review'))
os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
sys.dont_write_bytecode = True

import closed_loop as cl  # noqa: E402
import gate  # noqa: E402
import reproduce as rp  # noqa: E402

TARGETS = ['gz_airship_vectored_independent', 'gz_airship_vectored_independent_am_fin_drag',
           'gz_cloudship', 'gz_cloudship_am_fin_drag', 'gz_airship_vectored_independent_fin']
SHORT = {'gz_airship_vectored_independent': 'vec rigid',
         'gz_airship_vectored_independent_am_fin_drag': 'vec complete',
         'gz_cloudship': 'cld rigid',
         'gz_cloudship_am_fin_drag': 'cld complete',
         'gz_airship_vectored_independent_fin': 'vec fin (2nd case)'}

SP = (0.5 - 0.1) / (1 - 0.1) * 15.0      # sitl_check.py:240 at AS_YAWRATE_MAX 15, MAN_DEADZONE 0.1

# The model's stated uncertainty per plant, in the unit of each judged number
# (task brief + review/): noise-free peaks 0-0.12 deg/s low (noise is ON in
# every run here, the residual kept as the general band); Cloudship-complete
# 0.1-0.15 deg/s (pitch/heave held); rigid vectored peak and brake 0.3-0.4.
RATE_BAND = {'gz_airship_vectored_independent': 0.4, 'gz_cloudship_am_fin_drag': 0.15}
RATE_BAND_DEFAULT = 0.12
SETTLE_BAND = 0.6          # s: two 0.316 s rows (records vs model differ by up to 0.3 s: closed_loop_out.txt)
INTEG_BAND = 0.0010        # normalized: model vs record 0.0211-0.0214 vs 0.0214 (closed_loop_out.txt)


def rate_band(target):
    return RATE_BAND.get(target, RATE_BAND_DEFAULT)


# ---------------------------------------------------------------------------
# Building a loop: gains, b, actuator variant
# ---------------------------------------------------------------------------

def set_b(loop, b):
    """Install AS_YAWRATE_B on the modelled controller: ff = P (b - 1),
    recomputed on every parameters_updated() as the firmware would."""
    ctrl = loop.controller.ctrl
    ctrl.params['AS_YAWRATE_B'] = float(b)
    orig = ctrl.parameters_updated

    def parameters_updated():
        orig()
        ctrl.rc.ff = ctrl.params['AS_YAWRATE_P'] * (ctrl.params['AS_YAWRATE_B'] - 1.0)

    ctrl.parameters_updated = parameters_updated
    parameters_updated()
    return loop


def set_tilt_slew(loop, deg_s):
    """CA_AIRSHIP_TLT_R matched to a servo of this speed: the allocator's
    AirshipPod slew (and so the servo command ramp) at deg_s."""
    import vectored_actuation as va
    act = loop.actuation
    act.p = dict(act.p, slew=va.radians_f(deg_s))
    act.act = va.VectoredActuation(act.p, 0)
    return loop


def limit_servo(loop, deg_s):
    """A physical tilt servo that moves at most deg_s while CA_AIRSHIP_TLT_R
    stays at 120: the joint's position command is rate limited between the
    allocator and gz's JointPositionController (the allocator still believes
    its own 120 deg/s tilt)."""
    act = loop.actuation.act
    step = math.radians(deg_s) * act.dt

    for joint in act.joints:
        orig = joint.pre_update
        state = {'cmd': 0.0}         # jointPosCmd starts at 0 (vectored_actuation.GzTiltJoint)

        def pre_update(cmd, orig=orig, state=state):
            if cmd is not None:
                state['cmd'] += max(-step, min(step, cmd - state['cmd']))
            return orig(state['cmd'])

        joint.pre_update = pre_update

    return loop


def lag_tail(loop, tau):
    """A first-order lag of time constant tau on the Cloudship tail rotor's
    reference (the speed-like quantity; F = 2.3544 ref|ref|). gz's
    GenericMotorModel duty-cycle branch has NO lag (cloudship_actuation.py
    stage 4); this is a sensitivity case for a real motor, 0.025 s being the
    model's own counterfactual prop lag (cloudship_actuation.py:530)."""
    act = loop.actuation
    chain = act.chain
    orig = act.step
    state = {'ref': 0.0}
    alpha = 1.0 - math.exp(-cl.DT / tau)

    def step(dt, u):
        _, sat = orig(dt, u)
        s = act.stage
        state['ref'] += (s['ref'] - state['ref']) * alpha
        F = chain.thrust(state['ref'])
        L, M, N = chain.wrench(F)
        act.stage = dict(s, ref=state['ref'], F=F, L=L, M=M, N=N)
        return N, sat

    act.step = step
    return loop


def build(target, P=0.5, I=0.09, b=1.0, noise=True, seed=1, act=None, trace=False, params=None):
    """closed_loop.build + b + an actuator variant:
    act None | ('slew', deg/s) | ('servo', deg/s) | ('tail_tau', s)."""
    prm = {'AS_YAWRATE_P': P, 'AS_YAWRATE_I': I}
    prm.update(params or {})
    kw = {'noise': True, 'seed': seed} if noise else {}
    loop = cl.build(target, params=prm, trace=trace, **kw)
    set_b(loop, b)

    if act:
        kind, value = act
        if kind == 'slew':
            set_tilt_slew(loop, value)
        elif kind == 'servo':
            limit_servo(loop, value)
        elif kind == 'tail_tau':
            lag_tail(loop, value)
        else:
            raise ValueError(act)

    return loop


def model_name(target):
    return (cl.TARGETS.get(target) or cl.EXTRA_TARGETS[target])[0]


# ---------------------------------------------------------------------------
# Gate margins, from the gate's own rows at full precision
# ---------------------------------------------------------------------------

def _settled(rows, threshold=1.0, hold=1.5):
    """sitl_check.settled() (sitl_check.py:164-175)."""
    start = None
    for row in rows:
        if abs(row['rate']) < threshold:
            start = row['t'] if start is None else start
            if row['t'] - start >= hold:
                return start - rows[0]['t']
        else:
            start = None
    return None


def _tail(rows, seconds, key, signed=False):
    last = rows[-1]['t']
    vals = [r[key] if signed else abs(r[key]) for r in rows if r['t'] >= last - seconds]
    return sum(vals) / len(vals)


def margins(res, target):
    """{check: {'ok': gate verdict, 'm': margin (>0 inside), fields...}}.
    Margins are in the judged number's unit; the release margin is in
    seconds (10.0 - settle) when it settled, else None with 'end' the last
    |rate| (deg/s) for how far outside."""
    parsed = rp.parse(res.gate_lines)
    by = {}
    for row in res.samples:
        by.setdefault(row['phase'], []).append(row)
    out = {}

    for key, rec in parsed.items():
        d = {'ok': rec['ok']}

        if key == 'wind check':
            d['m'] = abs(rec['end'] - rec['start']) - 0.5 * rec['expected']
        elif key == 'Manual':
            d['m'] = None                                  # loop open: b cannot touch it
        elif key == 'Acro brake':
            end = by['Acro 제동'][-1]['rate']
            d['end'] = end
            d['m'] = 6.0 - abs(end)
        elif key in ('Acro track', 'Stab track'):
            ph = 'Acro 추종' if key == 'Acro track' else 'Stabilized 추종'
            rows = by[ph]
            mean = _tail(rows, 3.0, 'rate', signed=True)
            peak = max(r['rate'] for r in rows)
            d.update(mean=mean, peak=peak, m_peak=1.4 * SP - peak, m_mean=0.25 * SP - abs(mean - SP))
            d['m'] = min(d['m_peak'], d['m_mean'])
            d['t_peak'] = [r['t'] for r in rows if r['rate'] == peak][0] - rows[0]['t']
        elif key in ('Acro release', 'Stab release'):
            ph = 'Acro 놓음' if key == 'Acro release' else 'Stabilized 놓음'
            rows = by[ph]
            st = _settled(rows)
            d.update(start=rows[0]['rate'], end=rows[-1]['rate'], settle=st,
                     integ0=rows[0]['integ'])
            d['m'] = (10.0 - st) if st is not None else None
            d['m_turning'] = abs(rows[0]['rate']) - 0.5 * SP
        elif key.startswith('wind'):
            name = [p for p in by if (('I 켬' in p and '틸트' not in p and key == 'wind I on')
                                      or ('I 끔' in p and key == 'wind I off')
                                      or ('틸트 밴드' in p and key == 'wind weak')) and '회복' not in p][0]
            rows = by[name]
            rate = _tail(rows, 2.0, 'rate')
            integ = _tail(rows, 2.0, 'integ')
            needed = rec['needed']
            d.update(rate=rate, integ=integ, needed=needed)
            if key == 'wind I off':
                d['m'] = rate - 1.0
            else:
                d['m_rate'] = 1.0 - rate
                d['m_integ'] = 0.5 * abs(needed) - abs(integ - needed)
                d['m'] = min(d['m_rate'], d['m_integ'] / abs(needed))   # integ margin as a fraction of needed
        out[key] = d

    return out


def run_gate(target, timing=None, **kw):
    loop = build(target, **kw)
    od = os.path.join(HERE, 'tmp_out', str(os.getpid()))
    os.makedirs(od, exist_ok=True)
    res = gate.run_gate(loop, model_name(target), timing or gate.Timing(), out_dir=od)
    return res, margins(res, target)


# gain_sweep.py's ensemble of the model's unknowns (review/gain_sweep.py:26-36),
# its timing members; gyro noise is ON in every run here, so its 'noise s1'
# member is the nominal member with seed 1.
def ensemble():
    import gain_sweep
    return [(label, tkw) for label, tkw, bkw in gain_sweep.ENSEMBLE if not bkw]
