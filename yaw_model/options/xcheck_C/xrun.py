#!/usr/bin/env python3
"""Cross-check of option C (setpoint weighting b on the yaw-rate P term).

Independent implementation of b: instead of option C's hook (ff = P (b - 1)
installed on parameters_updated), the modelled RateControlYaw.update is
replaced by the textbook 2-DOF PI form
    torque = P (b r_sp - r) + integ,   integ += I-factor * I * (r_sp - r) dt
with the same anti-windup (clamped error, I-factor, INT_LIM). b is read on
every call from ctrl.params, so it cannot be lost on a parameter update.
Option C's own margins() (options/C/optc.py) are reused for the numbers; the
pass/fail verdicts are the gate's own text (reproduce.parse).

    python3 xrun.py repro     # option C's 12-run ensemble, same seeds, my b
    python3 xrun.py fresh     # 8 timing members x 5 NEW seeds (101..105)
    python3 xrun.py act       # actuator variants, nominal timing, 5 seeds
    python3 xrun.py hookrepro # option C's own hook, same seeds (determinism)
"""
import json
import math
import multiprocessing as mp
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
CDIR = os.path.join(os.path.dirname(HERE), 'C')
sys.path.insert(0, CDIR)
sys.dont_write_bytecode = True
os.environ['PYTHONDONTWRITEBYTECODE'] = '1'

import optc  # noqa: E402  (option C's helpers: margins, actuator variants, ensemble)
import closed_loop as cl  # noqa: E402
import gate  # noqa: E402


def install_b_mine(loop, b):
    ctrl = loop.controller.ctrl
    ctrl.params['AS_YAWRATE_B_X'] = float(b)
    rc = ctrl.rc
    assert rc.ff == 0.0

    def update(self, rate, rate_sp, dt, landed=False):
        bb = ctrl.params['AS_YAWRATE_B_X']
        e = rate_sp - rate
        torque = self.p * (bb * rate_sp - rate) + self.integ
        if not landed:
            self._update_integral(e, dt)
        return torque

    rc.update = types.MethodType(update, rc)
    return loop


def build_mine(target, P, I, b, seed, act=None, noise=True):
    kw = {'noise': True, 'seed': seed} if noise else {}
    loop = cl.build(target, params={'AS_YAWRATE_P': P, 'AS_YAWRATE_I': I}, **kw)
    install_b_mine(loop, b)
    if act:
        kind, value = act
        {'slew': optc.set_tilt_slew, 'servo': optc.limit_servo, 'tail_tau': optc.lag_tail}[kind](loop, value)
    return loop


def clean(x):
    if isinstance(x, float):
        return round(x, 6) if math.isfinite(x) else None
    if isinstance(x, dict):
        return {k: clean(v) for k, v in x.items()}
    return x


def one(job):
    impl, target, P, I, b, member, tkw, seed, act = job
    od = os.path.join(HERE, 'tmp_out', str(os.getpid()))
    os.makedirs(od, exist_ok=True)
    if impl == 'mine':
        loop = build_mine(target, P, I, b, seed, act)
    else:
        loop = optc.build(target, P=P, I=I, b=b, noise=True, seed=seed, act=act)
    res = gate.run_gate(loop, optc.model_name(target), gate.Timing(**tkw), out_dir=od)
    m = optc.margins(res, target)
    # sanity: b survived the gate (my impl reads it live; C's hook: ff consistent)
    rc = loop.controller.ctrl.rc
    ff_end = rc.ff
    return {'impl': impl, 'target': target, 'P': P, 'I': I, 'b': b, 'member': member, 'seed': seed,
            'act': list(act) if act else None, 'status': res.status, 'ff_end': ff_end,
            'verdicts': [[n, ok] for n, ok, _ in res.verdicts],
            'checks': clean(m)}


LIGHT = [(0.5, 0.09, 1.0), (0.5, 0.09, 0.694), (0.7, 0.09, 1.0), (0.7, 0.09, 0.9), (0.7, 0.15, 1.0),
         (0.7, 0.2, 1.0), (0.7, 0.2, 0.9), (0.7, 0.2, 0.8), (0.8, 0.2, 1.0)]
FRESH = [(0.7, 0.2, 0.9), (0.7, 0.2, 1.0), (0.7, 0.09, 1.0), (0.7, 0.09, 0.9)]
ACT_SET = [(0.7, 0.2, 0.9), (0.7, 0.2, 1.0), (0.7, 0.2, 0.8)]
ACTS = [('slew', 60.0), ('slew', 240.0), ('servo', 60.0), ('tail_tau', 0.025), ('tail_tau', 0.05)]


def jobs(kind):
    ens = optc.ensemble()
    out = []
    if kind in ('repro', 'hookrepro'):
        impl = 'mine' if kind == 'repro' else 'hook'
        for t in optc.TARGETS:
            for P, I, b in LIGHT:
                for n, (label, tkw) in enumerate(ens, 1):
                    out.append((impl, t, P, I, b, label, tkw, n, None))
                for s in (2, 3, 4, 5):
                    out.append((impl, t, P, I, b, 'nominal', {}, s, None))
    elif kind in ('fresh', 'fresh2'):
        # 8 timing members x 3 seeds, every run its own seed (101..124)
        sets = FRESH[:2] if kind == 'fresh' else FRESH[2:]
        for P, I, b in sets:
            for t in optc.TARGETS:
                if kind == 'fresh2' and t not in ('gz_airship_vectored_independent_fin', 'gz_cloudship_am_fin_drag'):
                    continue
                for m, (label, tkw) in enumerate(ens):
                    for j in range(3):
                        out.append(('mine', t, P, I, b, label, tkw, 101 + 3 * m + j, None))
    elif kind == 'act':
        for P, I, b in ACT_SET:
            for act in ACTS:
                vec = act[0] in ('slew', 'servo')
                for t in optc.TARGETS:
                    if vec != ('vectored' in t):
                        continue
                    for s in (201, 202, 203, 204, 205):
                        out.append(('mine', t, P, I, b, 'nominal', {}, s, act))
    else:
        raise SystemExit(kind)
    return out


def main():
    kind = sys.argv[1]
    todo = jobs(kind)
    if len(sys.argv) > 2 and sys.argv[2] == 'hooksub':      # C's hook on the repro subset only
        pass
    path = os.path.join(HERE, 'x_%s.jsonl' % kind.replace('fresh2', 'fresh'))
    done = set()
    if os.path.exists(path):
        for line in open(path):
            r = json.loads(line)
            done.add((r['impl'], r['target'], r['P'], r['I'], r['b'], r['member'], r['seed'],
                      tuple(r['act']) if r['act'] else None))
    todo = [j for j in todo if (j[0], j[1], j[2], j[3], j[4], j[5], j[7], j[8]) not in done]
    procs = int(os.environ.get('PROCS', '14'))
    print('%s: %d to do, %d done' % (kind, len(todo), len(done)), flush=True)
    with mp.Pool(procs) as pool, open(path, 'a') as fh:
        for n, r in enumerate(pool.imap_unordered(one, todo), 1):
            fh.write(json.dumps(r, ensure_ascii=False) + '\n')
            fh.flush()
            if n % 50 == 0:
                print('  %d / %d' % (n, len(todo)), flush=True)
    print('done', path, flush=True)


if __name__ == '__main__':
    main()
