#!/usr/bin/env python3
"""What option C does outside the gate: stick steps from rest, wind steps
at rest and in a turn, torque-command noise, and the same under slower /
faster actuators. The loop is driven directly (no gate harness): armed,
Acro, sticks valid, one 4 ms tick per step, the measured (filtered) rate.

    python3 beyond.py            # writes beyond.jsonl
"""
import json
import math
import multiprocessing as mp
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True

import optc  # noqa: E402

DT = 0.004
SETTINGS = [(0.5, 0.09, 1.0), (0.5, 0.09, 0.694),
            (0.7, 0.09, 1.0), (0.7, 0.09, 0.9), (0.7, 0.09, 0.8),
            (0.7, 0.2, 1.0), (0.7, 0.2, 0.9), (0.7, 0.2, 0.8)]
ACTS = [None, ('slew', 60.0), ('slew', 240.0), ('servo', 60.0), ('tail_tau', 0.025), ('tail_tau', 0.05)]
AUTH = {'vectored': 54.0, 'cloudship': 4.478}     # N m per unit (gz_model.yaw_authority, controller.py:738)


class Drive:
    def __init__(self, loop):
        self.loop = loop
        loop.set_stick(0.0)
        loop.arm()
        loop.set_mode('acro')
        self.t = 0.0
        self.rows = []

    def run(self, seconds, stick=None, wind_frd=None):
        if stick is not None:
            self.loop.set_stick(stick)
        if wind_frd is not None:
            self.loop.set_wind(wind_frd)
        for _ in range(int(round(seconds / DT))):
            self.loop.advance(DT)
            self.t += DT
            self.rows.append((self.t, math.degrees(self.loop.read_rate()), self.loop.read_torque(),
                              self.loop.controller.ctrl.integ))
        return self

    def window(self, t0, t1):
        return [r for r in self.rows if t0 < r[0] <= t1 + 1e-9]


def step_metrics(rows, sp, t0):
    """rows of a push starting at t0 toward sp (deg/s)."""
    rate = [r[1] for r in rows]
    t = [r[0] - t0 for r in rows]
    peak = max(rate)
    i_pk = rate.index(peak)
    t10 = next((tt for tt, x in zip(t, rate) if x >= 0.1 * sp), None)
    t90 = next((tt for tt, x in zip(t, rate) if x >= 0.9 * sp), None)
    outside = [tt for tt, x in zip(t, rate) if abs(x - sp) > 0.1 * sp]
    settle10 = outside[-1] if outside else 0.0
    if settle10 >= t[-1] - 1e-9:
        settle10 = None
    tail = [x for tt, x in zip(t, rate) if tt >= t[-1] - 10.0]
    inside = None                            # first entry into +-10 % that then holds 5 s
    for tt, x in zip(t, rate):
        if abs(x - sp) <= 0.1 * sp:
            inside = tt if inside is None else inside
            if tt - inside >= 5.0:
                break
        else:
            inside = None
    hold10 = inside if (inside is not None and t[-1] - inside >= 5.0) else None
    return {'peak': peak, 'hold_10pct': hold10, 'overshoot_pct': 100.0 * (peak - sp) / sp, 't_peak': t[i_pk],
            'rise_10_90': (t90 - t10) if (t10 is not None and t90 is not None) else None,
            'settle_10pct': settle10, 'mean_last10': sum(tail) / len(tail),
            'std_last10': (sum((x - sum(tail) / len(tail)) ** 2 for x in tail) / len(tail)) ** 0.5,
            'integ_end': rows[-1][3], 'torque_max': max(r[2] for r in rows)}


def release_metrics(rows, t0):
    rate = [r[1] for r in rows]
    t = [r[0] - t0 for r in rows]
    above = [tt for tt, x in zip(t, rate) if abs(x) >= 1.0]
    settle = above[-1] if above else 0.0
    if settle >= t[-1] - 1e-9:
        settle = None
    start = None                             # sitl_check.settled(): |r| < 1 held 1.5 s
    for tt, x in zip(t, rate):
        if abs(x) < 1.0:
            start = tt if start is None else start
            if tt - start >= 1.5:
                break
        else:
            start = None
    first = start if (start is not None and t[-1] - start >= 1.5) else None
    return {'first_hold_1p5': first, 'settle_1dps': settle, 'undershoot': min(rate), 'end': rate[-1],
            'integ0': rows[0][3], 'rate_at_5s': next(x for tt, x in zip(t, rate) if tt >= 5.0),
            'rate_at_10s': next(x for tt, x in zip(t, rate) if tt >= 10.0)}


def stick_step(target, P, I, b, act, stick=0.5, push=60.0, release=40.0):
    sp = max(0.0, (stick - 0.1) / 0.9) * 15.0
    d = Drive(optc.build(target, P, I, b, noise=False, act=act))
    d.run(2.0, stick=0.0)
    t0 = d.t
    d.run(push, stick=stick)
    pm = step_metrics(d.window(t0, d.t), sp, t0)
    t1 = d.t
    d.run(release, stick=0.0)
    rm = release_metrics(d.window(t1, d.t), t1)
    return {'push': pm, 'release': rm}


def wind_step(target, P, I, b, act, scale, stick):
    """A world-z wind couple of scale x (authority / 54) N m (the gate's
    strong case at scale 1), stepped on after 40 s of steady flight at the
    given stick, held 60 s. FRD moment = -tau (gate.GZ_WORLD_Z_TO_FRD)."""
    airframe = 'vectored' if 'vectored' in target else 'cloudship'
    tau = scale * AUTH[airframe] / 54.0
    sp = max(0.0, (stick - 0.1) / 0.9) * 15.0 if stick else 0.0
    d = Drive(optc.build(target, P, I, b, noise=False, act=act))
    d.run(40.0, stick=stick)
    before = d.window(d.t - 5.0, d.t)
    base = sum(r[1] for r in before) / len(before)
    t0 = d.t
    d.run(60.0, wind_frd=-tau)
    rows = d.window(t0, d.t)
    dev = [r[1] - base for r in rows]
    worst = max(dev, key=abs)
    t = [r[0] - t0 for r in rows]
    out = [tt for tt, x in zip(t, dev) if abs(x) > 0.5]
    return {'tau_nm': tau, 'base': base, 'sp': sp, 'peak_dev': worst, 't_peak': t[dev.index(worst)],
            'recover_0p5': (out[-1] if out else 0.0) if (not out or out[-1] < t[-1] - 1e-9) else None,
            'dev_end': dev[-1], 'integ_before': before[-1][3], 'integ_end': rows[-1][3]}


def torque_noise(target, P, I, b, stick):
    """Std of the published torque over 20 s of steady flight with gz's
    gyro noise on, after 40 s to settle (Cloudship-complete drift aside)."""
    d = Drive(optc.build(target, P, I, b, noise=True, seed=3))
    d.run(40.0, stick=stick)
    t0 = d.t
    d.run(20.0)
    rows = d.window(t0, d.t)
    tq = [r[2] for r in rows]
    # detrend with a 1 s moving mean so the slow Munk swing is not counted as noise
    n = int(1.0 / DT)
    hf = [tq[i] - sum(tq[i - n // 2:i + n // 2]) / n for i in range(n // 2, len(tq) - n // 2)]
    m = sum(tq) / len(tq)
    return {'std': (sum((x - m) ** 2 for x in tq) / len(tq)) ** 0.5,
            'std_hf': (sum(x * x for x in hf) / len(hf)) ** 0.5}


def one(job):
    kind, target, P, I, b, act, extra = job
    if kind == 'step':
        r = stick_step(target, P, I, b, act, **extra)
    elif kind == 'wind':
        r = wind_step(target, P, I, b, act, **extra)
    elif kind == 'noise':
        r = torque_noise(target, P, I, b, **extra)
    return {'kind': kind, 'target': target, 'P': P, 'I': I, 'b': b, 'act': list(act) if act else None,
            'extra': extra, 'r': r}


def jobs():
    out = []
    for t in optc.TARGETS:
        vec = 'vectored' in t
        for P, I, b in SETTINGS:
            for stick in (0.25, 0.5, 1.0):
                out.append(('step', t, P, I, b, None, {'stick': stick}))
            for act in ACTS[1:]:
                if (act[0] in ('slew', 'servo')) == vec:
                    out.append(('step', t, P, I, b, act, {'stick': 0.5}))
            for scale, stick in ((1.0, 0.0), (5.0, 0.0), (5.0, 0.5), (5.0, 1.0)):
                out.append(('wind', t, P, I, b, None, {'scale': scale, 'stick': stick}))
            for stick in (0.0, 0.5):
                out.append(('noise', t, P, I, b, None, {'stick': stick}))
    return out


def main():
    path = os.path.join(HERE, 'beyond.jsonl')
    todo = jobs()
    procs = int(os.environ.get('PROCS', '6'))
    print('%d jobs' % len(todo), flush=True)
    with mp.Pool(procs) as pool, open(path, 'w') as fh:
        for n, r in enumerate(pool.imap_unordered(one, todo), 1):
            fh.write(json.dumps(r) + '\n')
            fh.flush()
            if n % 50 == 0:
                print('  %d / %d' % (n, len(todo)), flush=True)
    print('done', path)


if __name__ == '__main__':
    main()
