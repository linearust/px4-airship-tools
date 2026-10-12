#!/usr/bin/env python3
"""Cross-check of option A: an independent gate runner and margin reader.

Builds the shared closed loop (closed_loop.build, gyro noise on, seeded),
runs gate.run_gate with gain_sweep.py's timing members (imported, not
retyped), and reads every margin from the raw sample rows (full precision),
not from the printed verdict lines. Verdicts come from the gate's own lines.

Actuator variants are implemented here, separately from option A's optA.py:
  ('slew', deg_s)     every allocator Pod's slew set in place (CA_AIRSHIP_TLT_R)
  ('servo', deg_s)    allocator stays 120; each gz tilt joint's velocity is
                      clamped to deg_s inside its physics step
  ('tail_tau', s)     Cloudship: first-order lag on the tail rotor ref
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, MODEL)
sys.path.insert(0, os.path.join(MODEL, 'review'))
sys.dont_write_bytecode = True
os.environ['PYTHONDONTWRITEBYTECODE'] = '1'

import closed_loop as cl  # noqa: E402
import gate  # noqa: E402
import gain_sweep  # noqa: E402  (ENSEMBLE only)

TARGETS = ['gz_airship_vectored_independent', 'gz_airship_vectored_independent_am_fin_drag',
           'gz_airship_vectored_independent_fin', 'gz_cloudship', 'gz_cloudship_am_fin_drag']
SHORT = dict(zip(TARGETS, ['vec rigid', 'vec complete', 'vec fin', 'cs rigid', 'cs complete']))
VECTORED = TARGETS[:3]
MEMBERS = {label: tkw for label, tkw, bkw in gain_sweep.ENSEMBLE if not bkw}   # 8 timing members
SP = (0.5 - 0.1) / (1 - 0.1) * 15.0


def model_name(t):
    return (cl.TARGETS.get(t) or cl.EXTRA_TARGETS[t])[0]


def build(target, params, seed, act=None):
    loop = cl.build(target, params=params, noise=True, seed=seed)
    if act is None:
        return loop
    kind, v = act
    if kind == 'slew':
        assert target in VECTORED
        for pod in loop.actuation.act.alloc.pods:
            pod.slew = math.radians(v)
            pod.slew_limited = True
    elif kind == 'servo':
        assert target in VECTORED
        wmax = math.radians(v)
        for j in loop.actuation.act.joints:
            def physics(j=j):
                j.omega = (j.omega + j.dt * j.tau / j.J) / (1.0 + j.dt * j.c / j.J)
                j.omega = max(-wmax, min(wmax, j.omega))
                j.theta += j.dt * j.omega
                if j.theta > j.upper:
                    j.theta, j.omega = j.upper, min(j.omega, 0.0)
                elif j.theta < j.lower:
                    j.theta, j.omega = j.lower, max(j.omega, 0.0)
            j.physics = physics
    elif kind == 'tail_tau':
        assert target not in VECTORED
        act_obj = loop.actuation
        chain = act_obj.chain
        state = {'ref': 0.0}
        a = 1.0 - math.exp(-cl.DT / v)

        def step(dt, u):
            st = chain.stages(u)
            state['ref'] += (st['ref'] - state['ref']) * a
            F = chain.thrust(state['ref'])
            L, M, N = chain.wrench(F)
            st = dict(st, ref=state['ref'], F=F, L=L, M=M, N=N)
            act_obj.stage = st
            return N, st['sat']
        act_obj.step = step
    else:
        raise ValueError(act)
    return loop


def settled(rows, th, hold=1.5):
    start = None
    for r in rows:
        if abs(r['rate']) < th:
            start = r['t'] if start is None else start
            if r['t'] - start >= hold:
                return start - rows[0]['t']
        else:
            start = None
    return None


def release_margin(rows):
    """1.0 - the smallest rest threshold at which the release passes (settle
    time is non-increasing in the threshold, and changes only where the
    threshold crosses a row's |rate|). Negative: it fails at 1.0."""
    if abs(rows[0]['rate']) <= 0.5 * SP:
        return abs(rows[0]['rate']) - 0.5 * SP - 10.0        # not turning: a failure by construction
    cands = sorted({abs(r['rate']) for r in rows})
    for c in cands:
        s = settled(rows, c + 1e-9)
        if s is not None and s < 10.0:
            return 1.0 - c
    return 1.0 - (cands[-1] + 1e-9)


KEYMAP = [('Acro brake', 'Acro 가 남은'), ('Acro track', 'Acro: 스틱이 지시'), ('Acro release', 'Acro: 스틱을 놓으면'),
          ('Stab track', 'Stabilized: 스틱이 지시'), ('Stab release', 'Stabilized: 스틱을 놓으면'),
          ('wind I on', 'I 켬: 적분기'), ('wind I off', 'I 끔'), ('wind weak', '틸트 밴드 아래'),
          ('wind check', '바람(외부'), ('Manual', 'Manual 에서')]


def key_of(name):
    for k, pat in KEYMAP:
        if pat in name:
            return k
    return None


def tail_mean(rows, sec, key):
    last = rows[-1]['t']
    v = [abs(r[key]) for r in rows if r['t'] >= last - sec]
    return sum(v) / len(v)


def run(job):
    target, params, member, seed, act = job
    loop = build(target, params, seed, act)
    # true integrator max, every controller cycle
    ctrl = loop.controller
    peak = {'integ': 0.0}
    orig = ctrl.update

    def upd(*a, **k):
        out = orig(*a, **k)
        v = abs(ctrl.ctrl.integ)
        if v > peak['integ']:
            peak['integ'] = v
        return out
    ctrl.update = upd

    od = os.path.join(HERE, 'tmp_out', str(os.getpid()))
    os.makedirs(od, exist_ok=True)
    res = gate.run_gate(loop, model_name(target), gate.Timing(**MEMBERS[member]), out_dir=od)
    ok = {}
    for name, passed, detail in res.verdicts:
        k = key_of(name)
        if k and k not in ok:
            ok[k] = passed
    ph = {}
    for r in res.samples:
        ph.setdefault(r['phase'], []).append(r)
    m = {}
    if 'Acro 제동' in ph:
        m['Acro brake'] = {'end': 6.0 - abs(ph['Acro 제동'][-1]['rate'])}
    for mode, label in (('Acro', 'Acro'), ('Stab', 'Stabilized')):
        push = ph.get('%s 추종' % label)
        if push:
            last = push[-1]['t']
            tail = [r['rate'] for r in push if r['t'] >= last - 3.0]
            mean = sum(tail) / len(tail)
            m[mode + ' track'] = {'mean': 0.25 * SP - abs(mean - SP), 'peak': 1.4 * SP - max(r['rate'] for r in push),
                                  'mean_val': mean, 'peak_val': max(r['rate'] for r in push)}
        rel = ph.get('%s 놓음' % label)
        if rel:
            m[mode + ' release'] = {'release': release_margin(rel), 'r0': rel[0]['rate'],
                                    'settle': settled(rel, 1.0)}
    strong = 1.0 / 54.0
    for phase, rows in ph.items():
        if phase.endswith('회복') or not phase.startswith('바람 ') or '확인' in phase:
            continue
        k = key_of(phase)
        rate = tail_mean(rows, 2.0, 'rate')
        integ = tail_mean(rows, 2.0, 'integ')
        if k == 'wind I off':
            m[k] = {'rate': rate - 1.0, 'rate_val': rate}
        else:
            needed = strong * (0.4 if k == 'wind weak' else 1.0)
            m[k] = {'rate': 1.0 - rate, 'integ': 0.5 * needed - abs(integ - needed), 'rate_val': rate,
                    'integ_val': integ}
    # the integrator at the start of each release (the carry the review describes)
    integ_rel = {mode: ph['%s 놓음' % label][0]['integ'] for mode, label in (('Acro', 'Acro'), ('Stab', 'Stabilized'))
                 if '%s 놓음' % label in ph}
    integ_ph = {}
    for phase, rows in ph.items():
        vals = [abs(r['integ']) for r in rows if isinstance(r['integ'], float) and math.isfinite(r['integ'])]
        if vals:
            integ_ph[phase] = max(vals)
    return {'target': target, 'params': params, 'member': member, 'seed': seed, 'act': act, 'status': res.status,
            'ok': ok, 'm': m, 'integ_true_max': peak['integ'], 'integ_rel': integ_rel, 'integ_phase_max': integ_ph,
            'lines': res.gate_lines}


if __name__ == '__main__':
    import json
    import time
    t0 = time.time()
    r = run((sys.argv[1], {'AS_YAWRATE_P': 0.7, 'AS_YAWRATE_I': 0.2, 'AS_YR_INT_LIM': 0.3}, 'nominal', 1, None))
    print(json.dumps({k: v for k, v in r.items() if k != 'integ_phase_max'}, ensure_ascii=False, indent=1))
    print('elapsed', time.time() - t0)
