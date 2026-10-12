#!/usr/bin/env python3
"""Option A (gains only) harness: the shared yaw model, imported read-only.

    import optA
    rec = optA.gate_job(('gz_cloudship', {'AS_YAWRATE_P': 0.7}, 'nominal', 1, None))

A job is (target, params, member, seed, act) where
  member  one of gain_sweep.py's ENSEMBLE labels (the timing unknowns); the
          'noise s1' member is not used on its own because every run here has
          gyro noise on (seed = the noise seed)
  act     None, or an actuator variant for the sensitivity runs:
          ('slew', deg_s)        vectored: CA_AIRSHIP_TLT_R and the servo both at deg_s
          ('servo', deg_s)       vectored: allocator keeps 120, the physical servo
                                 is rate-limited to deg_s (mismatch)
          ('rotor', k)           vectored: pod rotor time constants x k
          ('tail_tau', tau_s)    Cloudship: first-order lag on the tail rotor ref
          ('noise', k)           gyro noise x k (either vehicle)

Margins: every judged number as a signed distance to its threshold (> 0 inside),
in the gate's own unit; the release's is expressed as a rate (deg/s): the
largest shift of the 1.0 deg/s rest threshold that keeps settle < 10 s (> 0)
or the shift needed to make it pass (< 0), found on the gate's own rows with
sitl_check's settled().
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, MODEL)
sys.dont_write_bytecode = True
os.environ['PYTHONDONTWRITEBYTECODE'] = '1'

import closed_loop as cl  # noqa: E402
import gate  # noqa: E402
import reproduce as rp  # noqa: E402

TARGETS = ['gz_airship_vectored_independent', 'gz_airship_vectored_independent_am_fin_drag',
           'gz_airship_vectored_independent_fin', 'gz_cloudship', 'gz_cloudship_am_fin_drag']
VECTORED = TARGETS[:3]
CLOUDSHIP = TARGETS[3:]
SHORT = {'gz_airship_vectored_independent': 'vec rigid', 'gz_airship_vectored_independent_am_fin_drag': 'vec complete',
         'gz_airship_vectored_independent_fin': 'vec fin (2nd case)', 'gz_cloudship': 'cs rigid',
         'gz_cloudship_am_fin_drag': 'cs complete'}

# gain_sweep.py's ENSEMBLE (review/gain_sweep.py:29-39) minus its 'noise s1'
# member: noise is on in every run here.
MEMBERS = {
    'nominal': {},
    'stick 0': {'stick_delay_s': 0.0},
    'stick 50ms': {'stick_delay_s': 0.05},
    'rtf .98': {'rtf': 0.98},
    'rtf 1.02': {'rtf': 1.02},
    'P .3153': {'listener_s': (0.3153 - 0.3) / 3},
    'P .3177': {'listener_s': (0.3177 - 0.3) / 3},
    'cmd 30ms': {'command_delay_s': 0.03},
}

SP = (0.5 - 0.1) / (1 - 0.1) * 15.0          # sitl_check.py:240 at AS_YAWRATE_MAX 15, MAN_DEADZONE 0.1
CHECKS = ['Acro brake', 'Acro track', 'Acro release', 'Stab track', 'Stab release',
          'wind I on', 'wind I off', 'wind weak']
FIXED = ['wind check', 'Manual']            # before the loop closes: gain independent

# The model's stated uncertainty (task statement / review/): the band around a
# threshold inside which a verdict is undecided, per plant and number.
#  - rigid vectored: peak and brake off by 0.3-0.4 deg/s (roll ripple read at
#    one instant) -> 0.4. Its release is not in that statement (it settled
#    4.7/4.8 s in SITL, 4.7 in the model) and takes the general band.
#  - Cloudship complete: 0.1-0.15 deg/s (pitch and heave held) -> 0.15.
#  - everything else: 0.15 deg/s, the largest |model - record| of a judged rate
#    in reproduction_out.txt (0.13, Cloudship complete peak; 0.12 Cloudship peak).
#  - integrator in the wind cases: 0.001 (reproduction <= 0.0005).
def band(target, key, field):
    if field == 'integ':
        return 0.001
    if target == 'gz_airship_vectored_independent' and (key == 'Acro brake' or field == 'peak'):
        return 0.4
    return 0.15


def settled(rows, threshold=1.0, hold=1.5):
    """sitl_check.settled (sitl_check.py:164-175), on the gate's rows."""
    start = None
    for row in rows:
        if abs(row['rate']) < threshold:
            start = row['t'] if start is None else start
            if row['t'] - start >= hold:
                return start - rows[0]['t']
        else:
            start = None
    return None


def release_margin(rows):
    """Rate-equivalent margin of the release verdict [deg/s]: > 0, the largest
    d with settled(rows, 1.0 - d) < 10 (rates could all read d higher and it
    still passes); < 0, minus the smallest d with settled(rows, 1.0 + d) < 10.
    Also requires the gate's was_turning (|rows[0]| > 0.5 sp)."""
    if not rows or not all(math.isfinite(r['rate']) for r in rows):
        return None
    turning = abs(rows[0]['rate']) - 0.5 * SP

    def ok(th):
        s = settled(rows, th)
        return s is not None and s < 10.0

    if ok(1.0):
        lo, hi = 0.0, 1.0
        if ok(1.0 - hi + 1e-9):
            m = hi
        else:
            for _ in range(30):
                mid = 0.5 * (lo + hi)
                lo, hi = (mid, hi) if ok(1.0 - mid) else (lo, mid)
            m = lo
    else:
        lo, hi = 0.0, 5.0
        if not ok(1.0 + hi):
            m = -hi
        else:
            for _ in range(30):
                mid = 0.5 * (lo + hi)
                lo, hi = (lo, mid) if ok(1.0 + mid) else (mid, hi)
            m = -hi
    return min(m, turning) if turning < 0 else m


class LaggedCloudshipAct(cl.CloudshipAct):
    """The Cloudship tail with a first-order lag on the rotor reference (the
    gz plant has none: cloudship_actuation.py:66-71). Thrust stays
    2.3544 ref|ref| on the lagged ref; the allocator's saturation sign is the
    allocator's own and does not see the lag."""

    def __init__(self, model, tau):
        super().__init__(model)
        self.tau = tau
        self.ref_f = 0.0

    def step(self, dt, u):
        st = self.chain.stages(u)
        a = 1.0 - math.exp(-dt / self.tau)
        self.ref_f += (st['ref'] - self.ref_f) * a
        F = self.chain.thrust(self.ref_f)
        L, M, N = self.chain.wrench(F)
        st = dict(st, ref=self.ref_f, F=F, L=L, M=M, N=N)
        self.stage = st
        return N, st['sat']


def _rate_limited_joint(va, w_max):
    class Joint(va.GzTiltJoint):
        def physics(self):
            super().physics()
            if abs(self.omega) > w_max:
                # the servo cannot turn faster: undo the step and redo it at w_max
                self.theta -= self.dt * self.omega
                self.omega = math.copysign(w_max, self.omega)
                self.theta += self.dt * self.omega
                self.theta = min(max(self.theta, self.lower), self.upper)
    return Joint


def build(target, params, seed=1, act=None, trace=False):
    import planar
    noise_k = 1.0
    if act and act[0] == 'noise':
        noise_k = act[1]
    planar.GYRO_NOISE = cl.GYRO_NOISE * noise_k          # runtime only, this process
    loop = cl.build(target, params=params, noise=True, seed=seed, trace=trace)

    if act is None or act[0] == 'noise':
        return loop

    kind, value = act
    if kind in ('slew', 'servo', 'rotor'):
        assert target in VECTORED
        va_mod = loop.actuation.va
        p = dict(loop.actuation.p)
        if kind == 'slew':
            p['slew'] = math.radians(value)
            loop.actuation.act = va_mod.VectoredActuation(p, 0)
        elif kind == 'servo':
            act_obj = va_mod.VectoredActuation(p, 0)
            J = _rate_limited_joint(va_mod, math.radians(value))
            pods = [p['plant']['pods'][s] for s in p['sides']]
            act_obj.joints = [J(pod, act_obj.dt) for pod in pods]
            loop.actuation.act = act_obj
        else:
            plant = dict(p['plant'])
            plant['pods'] = {s: dict(v, tau_up=v['tau_up'] * value, tau_down=v['tau_down'] * value)
                             for s, v in p['plant']['pods'].items()}
            p['plant'] = plant
            loop.actuation.act = va_mod.VectoredActuation(p, 0)
        loop.actuation.p = p
    elif kind == 'tail_tau':
        assert target in CLOUDSHIP
        model = cl.TARGETS[target][0]
        loop.actuation = LaggedCloudshipAct(model, value)
    else:
        raise ValueError(act)
    return loop


def model_name(target):
    return (cl.TARGETS.get(target) or cl.EXTRA_TARGETS[target])[0]


def gate_job(job):
    target, params, member, seed, act = job
    loop = build(target, params, seed, act)
    od = os.path.join(HERE, 'tmp_out', str(os.getpid()))
    os.makedirs(od, exist_ok=True)
    res = gate.run_gate(loop, model_name(target), gate.Timing(**MEMBERS[member]), out_dir=od)
    parsed = rp.parse(res.gate_lines)
    checks = {}
    for k, rec in parsed.items():
        checks[k] = {f: (v if not (isinstance(v, float) and not math.isfinite(v)) else None)
                     for f, v in rec.items() if f != 'line'}

    by_phase = {}
    for row in res.samples:
        by_phase.setdefault(row['phase'], []).append(row)

    m = {}
    c = checks
    if 'Acro brake' in c and c['Acro brake'].get('end') is not None:
        brake = by_phase.get('Acro 제동', [])
        m['Acro brake'] = {'end': 6.0 - abs(brake[-1]['rate'])} if brake else {}
    for mode, label in (('Acro', 'Acro'), ('Stab', 'Stabilized')):
        k = mode + ' track'
        if k in c and c[k].get('mean') is not None:
            m[k] = {'mean': 0.25 * SP - abs(c[k]['mean'] - SP), 'peak': 1.4 * SP - c[k]['peak']}
        rel = by_phase.get('%s 놓음' % label)
        if rel:
            m[mode + ' release'] = {'release': release_margin(rel)}
    for k, hold in (('wind I on', True), ('wind I off', False), ('wind weak', True)):
        if k in c and c[k].get('rate') is not None:
            if hold:
                m[k] = {'rate': 1.0 - c[k]['rate'], 'integ': 0.5 * c[k]['needed'] - abs(c[k]['integ'] - c[k]['needed'])}
            else:
                m[k] = {'rate': c[k]['rate'] - 1.0}

    # the largest |integrator| anywhere in the run (does AS_YR_INT_LIM bind?)
    integ_max = max((abs(r['integ']) for r in res.samples if isinstance(r['integ'], float) and math.isfinite(r['integ'])),
                    default=0.0)
    return {'target': target, 'params': params, 'member': member, 'seed': seed, 'act': act,
            'status': res.status, 'checks': checks, 'margins': m, 'integ_max': integ_max}


def norm_margins(rec):
    """{check: (ok, worst margin / band)} for the gain-dependent checks."""
    out = {}
    for k in CHECKS:
        if k not in rec['checks']:
            continue
        ok = rec['checks'][k]['ok']
        mm = rec['margins'].get(k, {})
        vals = [(v / band(rec['target'], k, f)) for f, v in mm.items() if v is not None]
        out[k] = (ok, min(vals) if vals else None)
    return out
