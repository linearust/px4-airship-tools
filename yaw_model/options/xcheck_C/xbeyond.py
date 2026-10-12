#!/usr/bin/env python3
"""Outside the gate, with my own b (xrun.install_b_mine) and my own driver:
stick steps (noise off and gz gyro noise on, 2 seeds), wind steps at rest
and in a turn, the actuator variants, torque-command noise.

    python3 xbeyond.py      # -> xb.jsonl
"""
import json
import math
import multiprocessing as mp
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True
import xrun  # noqa: E402
import optc  # noqa: E402

DT = 0.004
SETTINGS = [(0.5, 0.09, 1.0), (0.5, 0.09, 0.694), (0.7, 0.09, 1.0), (0.7, 0.09, 0.9), (0.7, 0.09, 0.8),
            (0.7, 0.2, 1.0), (0.7, 0.2, 0.9), (0.7, 0.2, 0.8)]
AUTH = {True: 54.0, False: 4.478}


def sp_of(stick):
    return max(0.0, (abs(stick) - 0.1) / 0.9) * 15.0 * (1 if stick >= 0 else -1)


class Rec:
    def __init__(self, loop):
        self.loop = loop
        loop.set_stick(0.0)
        loop.arm()
        loop.set_mode('acro')
        self.t = 0.0
        self.r = []

    def go(self, sec, stick=None, wind=None):
        if stick is not None:
            self.loop.set_stick(stick)
        if wind is not None:
            self.loop.set_wind(wind)
        n = int(round(sec / DT))
        for _ in range(n):
            self.loop.advance(DT)
            self.t += DT
            self.r.append((self.t, math.degrees(self.loop.read_rate()), self.loop.read_torque(),
                           self.loop.controller.ctrl.integ))

    def seg(self, a, b):
        return [x for x in self.r if a < x[0] <= b + 1e-9]


def first_hold(rows, t0, pred, hold):
    start = None
    for t, r, *_ in rows:
        if pred(r):
            start = t if start is None else start
            if t - start >= hold:
                return start - t0
        else:
            start = None
    return None


def step(target, P, I, b, act, stick, seed):
    loop = xrun.build_mine(target, P, I, b, seed or 1, act, noise=bool(seed))
    d = Rec(loop)
    d.go(2.0, 0.0)
    t0 = d.t
    d.go(60.0, stick)
    push = d.seg(t0, d.t)
    t1 = d.t
    d.go(40.0, 0.0)
    rel = d.seg(t1, d.t)
    sp = sp_of(stick)
    rate = [x[1] for x in push]
    pk = max(rate)
    # entry into +-10 % that is never left again before the push ends
    last_out = max((x[0] for x in push if abs(x[1] - sp) > 0.1 * sp), default=t0)
    settle10 = (last_out - t0) if last_out < push[-1][0] - 1e-9 else None
    t10 = next((x[0] - t0 for x in push if x[1] >= 0.1 * sp), None)
    t90 = next((x[0] - t0 for x in push if x[1] >= 0.9 * sp), None)
    tail = [x[1] for x in push if x[0] >= push[-1][0] - 10]
    return {'peak': pk, 'os_pct': 100 * (pk - sp) / sp, 't_peak': push[rate.index(pk)][0] - t0,
            'rise': (t90 - t10) if t10 is not None and t90 is not None else None, 'settle10': settle10,
            'mean10': sum(tail) / len(tail), 'integ_push_end': push[-1][3],
            'rel_hold': first_hold(rel, t1, lambda r: abs(r) < 1.0, 1.5), 'rel_under': min(x[1] for x in rel),
            'rel_integ0': rel[0][3], 'rel_end': rel[-1][1], 'torque_first': push[0][2],
            'torque_max': max(x[2] for x in push)}


def wind(target, P, I, b, stick, scale, seed):
    vec = 'vectored' in target
    tau = scale * AUTH[vec] / 54.0
    loop = xrun.build_mine(target, P, I, b, seed or 1, None, noise=bool(seed))
    d = Rec(loop)
    d.go(40.0, stick)
    pre = d.seg(d.t - 5, d.t)
    base = sum(x[1] for x in pre) / len(pre)
    ib = pre[-1][3]
    t0 = d.t
    d.go(60.0, wind=-tau)
    w = d.seg(t0, d.t)
    dev = [x[1] - base for x in w]
    worst = max(dev, key=abs)
    return {'tau': tau, 'base': base, 'integ_before': ib, 'peak_dev': worst, 'end_dev': sum(dev[-250:]) / 250,
            'integ_max': max(abs(x[3]) for x in w), 'integ_end': w[-1][3]}


def tnoise(target, P, I, b, stick, seed):
    loop = xrun.build_mine(target, P, I, b, seed, None, noise=True)
    d = Rec(loop)
    d.go(40.0, stick)
    t0 = d.t
    d.go(20.0)
    tq = [x[2] for x in d.seg(t0, d.t)]
    m = sum(tq) / len(tq)
    # high-pass: difference from a 1 s centred moving mean
    n = 250
    cs = [0.0]
    for x in tq:
        cs.append(cs[-1] + x)
    hf = [tq[i] - (cs[i + n // 2] - cs[i - n // 2]) / n for i in range(n // 2, len(tq) - n // 2)]
    return {'std': (sum((x - m) ** 2 for x in tq) / len(tq)) ** 0.5,
            'std_hf': (sum(x * x for x in hf) / len(hf)) ** 0.5}


def one(job):
    kind, t, P, I, b, extra = job
    f = {'step': step, 'wind': wind, 'noise': tnoise}[kind]
    kw = dict(extra)
    if kind == 'step':
        kw['act'] = tuple(kw['act']) if kw.get('act') else None
    return {'kind': kind, 'target': t, 'P': P, 'I': I, 'b': b, 'extra': extra, 'r': f(t, P, I, b, **kw)}


def jobs():
    out = []
    for t in optc.TARGETS:
        vec = 'vectored' in t
        for P, I, b in SETTINGS:
            for seed in (0, 11, 12):
                out.append(('step', t, P, I, b, {'act': None, 'stick': 0.5, 'seed': seed}))
            out.append(('step', t, P, I, b, {'act': None, 'stick': 1.0, 'seed': 0}))
            acts = [('slew', 60.0), ('slew', 240.0), ('servo', 60.0)] if vec else [('tail_tau', 0.025), ('tail_tau', 0.05)]
            for a in acts:
                for seed in (0, 11):
                    out.append(('step', t, P, I, b, {'act': list(a), 'stick': 0.5, 'seed': seed}))
            for stick, scale in ((0.0, 5.0), (0.5, 5.0), (1.0, 5.0), (0.5, 1.0)):
                out.append(('wind', t, P, I, b, {'stick': stick, 'scale': scale, 'seed': 11}))
            for stick in (0.0, 0.5):
                for seed in (11, 12):
                    out.append(('noise', t, P, I, b, {'stick': stick, 'seed': seed}))
    return out


def main():
    todo = jobs()
    path = os.path.join(HERE, 'xb.jsonl')
    print(len(todo), 'jobs', flush=True)
    with mp.Pool(int(os.environ.get('PROCS', '14'))) as pool, open(path, 'w') as fh:
        for n, r in enumerate(pool.imap_unordered(one, todo), 1):
            fh.write(json.dumps(r) + '\n')
            fh.flush()
    print('done', flush=True)


if __name__ == '__main__':
    main()
