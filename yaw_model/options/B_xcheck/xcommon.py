#!/usr/bin/env python3
"""Cross-check of option B (yaw rate feedforward): my own loop builder,
actuator variants and gate metrics. Written independently of options/B/
ffcommon.py; only the shared model (closed_loop, gate, controller, planar,
the actuation files, review/gain_sweep.ENSEMBLE) is imported, never edited.

FF injection: controller.RateControlYaw.update computes
    torque = p e + integ + ff sp        (the port of rate_control.cpp:78)
and AirshipYawController.parameters_updated() never writes rc.ff, so setting
rc.ff once after build is what setFeedForwardGain(Vector3f(0, 0, FF)) in
airship_att_control's parameters_updated() would do (it would be re-set to
the same value on every parameter update, e.g. the gate's AS_YAWRATE_I 0/back).
verify_ff() checks this on a trace instead of trusting it.
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
from gain_sweep import ENSEMBLE  # noqa: E402

VR, VC, CR, CC, VF = ('gz_airship_vectored_independent', 'gz_airship_vectored_independent_am_fin_drag',
                      'gz_cloudship', 'gz_cloudship_am_fin_drag', 'gz_airship_vectored_independent_fin')
PLANTS = [VR, VC, CR, CC, VF]
NAME = {VR: 'vec rigid', VC: 'vec complete', CR: 'cs rigid', CC: 'cs complete', VF: 'vec fin'}
# recommended set of option B
REC = {VR: (0.65, 0.07, 0.0), VC: (0.65, 0.07, 1.0), CR: (0.65, 0.07, 0.0), CC: (0.65, 0.07, 0.05),
       VF: (0.65, 0.07, 0.3)}
SP = (0.5 - 0.1) / (1.0 - 0.1) * 15.0
# The model's stated uncertainty on a rate [deg/s]. Stated: rigid vectored peak and
# brake 0.3-0.4; Cloudship complete 0.1-0.15 (pitch/heave held). Not stated for the
# vectored complete / fin plants (same planar class, pitch/heave held): 0.15 taken,
# and ALT_UNC re-judges them at 0.3 to show what a looser bound would do.
UNC = {VR: 0.4, VC: 0.15, CR: 0.12, CC: 0.15, VF: 0.15}
ALT_UNC = {VR: 0.4, VC: 0.3, CR: 0.12, CC: 0.15, VF: 0.3}
TIMING = [(label, tkw) for label, tkw, bkw in ENSEMBLE if not bkw]      # 8 members, noise on separately

# sitl_check verdict names -> short keys
VERDICT_KEYS = [
    ('바람(외부 토크)', 'wind check'), ('Manual 에서', 'Manual'), ('Acro 가 남은', 'brake'),
    ('Acro: 스틱이 지시한', 'Acro track'), ('Acro: 스틱을 놓으면', 'Acro release'),
    ('Stabilized: 스틱이 지시한', 'Stab track'), ('Stabilized: 스틱을 놓으면', 'Stab release'),
    ('I 켬: 적분기가', 'wind I on'), ('I 끔', 'wind I off'), ('틸트 밴드 아래', 'wind weak'),
]
ORDER = ['brake', 'Acro track', 'Acro release', 'Stab track', 'Stab release', 'wind I on',
         'wind I off', 'wind weak', 'wind check', 'Manual']


def key_of(name):
    for frag, key in VERDICT_KEYS:
        if frag in name:
            return key
    raise KeyError(name)


def gz_name(target):
    return (cl.TARGETS.get(target) or cl.EXTRA_TARGETS[target])[0]


class TailLag:
    """Cloudship actuation with a first-order lag on the tail force; the
    allocator's saturation sign stays the instantaneous one."""

    def __init__(self, inner, tau):
        self.inner, self.tau, self.F = inner, tau, 0.0
        self.chain = inner.chain

    @property
    def stage(self):
        return self.inner.stage

    def step(self, dt, u):
        _, sat = self.inner.step(dt, u)
        self.F += (self.inner.stage['F'] - self.F) * (1.0 - math.exp(-dt / self.tau))
        _, _, N = self.chain.wrench(self.F)
        self.inner.stage = dict(self.inner.stage, F=self.F, N=N)
        return N, sat

    def body_forces(self):
        return self.inner.body_forces()

    def status(self):
        return self.inner.status()


def rate_limit_joint(joint, rate, dt):
    orig = joint.pre_update
    st = {'c': None}

    def pre_update(cmd):
        if cmd is not None:
            if st['c'] is None:
                st['c'] = joint.cmd
            st['c'] += max(-rate * dt, min(rate * dt, cmd - st['c']))
            cmd = st['c']
        return orig(cmd)

    joint.pre_update = pre_update


def make(target, P, I, FF, noise=True, seed=1, slew=None, phys_slew=None, tail_tau=None, trace=False):
    kw = {'noise': True, 'seed': seed} if noise else {}
    loop = cl.build(target, params={'AS_YAWRATE_P': P, 'AS_YAWRATE_I': I}, trace=trace, **kw)
    loop.controller.ctrl.rc.ff = FF
    act = loop.actuation
    if slew is not None:                       # CA_AIRSHIP_TLT_R and the pod slew both
        for pod in act.act.alloc.pods:
            pod.slew = math.radians(slew)
            pod.slew_limited = True
    if phys_slew is not None:                  # the joint slower than the allocator believes
        for j in act.act.joints:
            rate_limit_joint(j, math.radians(phys_slew), act.act.dt)
    if tail_tau:
        loop.actuation = TailLag(act, tail_tau)
    return loop


def settled(rows, threshold=1.0, hold=1.5):
    start = None
    for r in rows:
        if abs(r['rate']) < threshold:
            start = r['t'] if start is None else start
            if r['t'] - start >= hold:
                return start - rows[0]['t']
        else:
            start = None
    return None


def tail_mean(rows, seconds, key='rate', signed=False):
    last = rows[-1]['t']
    v = [r[key] if signed else abs(r[key]) for r in rows if r['t'] >= last - seconds]
    return sum(v) / len(v)


def judge(res, target):
    """Per check: the gate's own ok, and my margins (> 0 inside)."""
    ok = {key_of(n): o for n, o in res.results}
    ph = {}
    for r in res.samples:
        ph.setdefault(r['phase'], []).append(r)
    out = {}
    br = ph['Acro 제동']
    out['brake'] = dict(ok=ok['brake'], end=br[-1]['rate'], m=6.0 - abs(br[-1]['rate']))
    for lab, key in (('Acro', 'Acro'), ('Stabilized', 'Stab')):
        push, rel = ph['%s 추종' % lab], ph['%s 놓음' % lab]
        mean = tail_mean(push, 3.0, signed=True)
        peak = max(r['rate'] for r in push)
        out[key + ' track'] = dict(ok=ok[key + ' track'], mean=mean, peak=peak,
                                   m_mean=0.25 * SP - abs(mean - SP), m_peak=1.4 * SP - peak)
        out[key + ' track']['m'] = min(out[key + ' track']['m_mean'], out[key + ' track']['m_peak'])
        st = settled(rel)
        out[key + ' release'] = dict(ok=ok[key + ' release'], r0=rel[0]['rate'], end=rel[-1]['rate'],
                                     settle=st, m=(10.0 - st) if st is not None else None,
                                     m_turn=abs(rel[0]['rate']) - 0.5 * SP,
                                     st_lo={}, st_hi={})
        for name, table in (('unc', UNC), ('alt', ALT_UNC)):
            u = table[target]
            out[key + ' release']['st_lo'][name] = settled(rel, 1.0 - u)
            out[key + ' release']['st_hi'][name] = settled(rel, 1.0 + u)
    for key, frag in (('wind I on', 'I 켬: 적분기가'), ('wind I off', 'I 끔'), ('wind weak', '틸트 밴드 아래')):
        names = [k for k in ph if frag in k and not k.endswith('회복')]
        if not names:
            continue
        rows = ph[names[0]]
        rate, integ = tail_mean(rows, 2.0), tail_mean(rows, 2.0, 'integ')
        if key == 'wind I off':
            out[key] = dict(ok=ok[key], rate=rate, integ=integ, m=rate - 1.0)
        else:
            torque_nm = float(names[0].split()[1])
            needed = torque_nm / (54.0 if 'vectored' in target else 4.48)
            out[key] = dict(ok=ok[key], rate=rate, integ=integ, needed=needed, m=1.0 - rate,
                            m_integ=0.5 * needed - abs(integ - needed))
    out['wind check'] = dict(ok=ok['wind check'])
    out['Manual'] = dict(ok=ok['Manual'])
    return out


def undecided(key, rec, target, table='unc'):
    u = (UNC if table == 'unc' else ALT_UNC)[target]
    if key in ('wind check', 'Manual'):
        return False
    if key.endswith('release'):
        def verdict(st, du):
            return abs(rec['r0']) + du > 0.5 * SP and st is not None and st < 10.0
        lo = verdict(rec['st_lo'][table], -u)
        hi = verdict(rec['st_hi'][table], +u)
        return lo != hi
    return abs(rec['m']) <= u


def run(target, P, I, FF, tkw=None, seed=1, noise=True, out_dir=None, **act):
    loop = make(target, P, I, FF, noise=noise, seed=seed, **act)
    od = out_dir or os.path.join(HERE, 'tmp_out', str(os.getpid()))
    os.makedirs(od, exist_ok=True)
    res = gate.run_gate(loop, gz_name(target), gate.Timing(**(tkw or {})), out_dir=od)
    return res, loop
