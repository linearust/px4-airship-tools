#!/usr/bin/env python3
"""What the gate does not check but the vehicle needs, on the closed-loop
model (roll plant, gyro noise on, seed 1 unless given):

  step      from rest in Acro, stick s for 60 s, then 0 for 40 s; the rate
            recorded every 4 ms tick. Stick 0.25 / 0.5 / 1.0 = 2.5 / 6.67 /
            15 deg/s: the quadratic hull damping makes the hold torque
            ~r^2 while FF sp is linear, so FF matched at one rate is wrong at
            the others.
              rise    10 -> 90 % of sp
              over    (peak - sp) / sp, %
              ts10    last time |r - sp| > 10 % sp during the push (s after the step)
              ess     mean(r) - sp over the last 10 s of the push
              rel     stick 0: seconds until |r| < 1 deg/s held 1.5 s (the gate's
                      settled() rule, on the 4 ms record)
              rel10   seconds until |r| < 0.1 sp for good
              under   most negative rate after release (reversal overshoot)
  wind      at rest in Acro, stick 0: the gate's strong gust (authority/54
            N m: 1 N m vectored, 0.0829 N m Cloudship) switched on for 60 s:
            peak |r|, seconds until |r| < 0.25 deg/s for good, integrator at the
            end / needed; and the same gust arriving during a 6.67 deg/s turn:
            peak deviation from the turn rate.
  noise     high-pass (minus a 1 s moving mean) rms of the torque command:
            at rest; in the 6.67 deg/s turn with gyro noise only; and with a
            stick jitter N(0, 0.002) re-drawn every 20 ms on top (FF and P both
            see the setpoint, so the stick path's gain is P + FF; the gyro
            path's is P alone).

    python3 beyond.py OUT.jsonl --configs "P,I,FF;..." [--plants all|i,j] [--act JSON] [--seed 1]
"""
import argparse
import json
import math
import multiprocessing as mp
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True
import ffcommon as fc  # noqa: E402

DT = 0.004
GUST = {'gz_airship_vectored_independent': 1.0, 'gz_airship_vectored_independent_am_fin_drag': 1.0,
        'gz_airship_vectored_independent_fin': 1.0, 'gz_cloudship': 4.48 / 54.0,
        'gz_cloudship_am_fin_drag': 4.48 / 54.0}
AUTH = {'gz_airship_vectored_independent': 54.0, 'gz_airship_vectored_independent_am_fin_drag': 54.0,
        'gz_airship_vectored_independent_fin': 54.0, 'gz_cloudship': 4.48, 'gz_cloudship_am_fin_drag': 4.48}


def start(target, P, I, FF, seed, act):
    loop = fc.build(target, P, I, FF, noise=True, seed=seed, **act)
    loop.set_stick(0.0)
    loop.arm()
    loop.set_mode('manual')
    loop.advance(1.0)
    loop.set_mode('acro')
    loop.advance(2.0)
    return loop


def record(loop, seconds, jitter=None, base=0.0, rng=None):
    rows = []
    n = int(round(seconds / DT))
    for k in range(n):
        if jitter and k % 5 == 0:
            loop.set_stick(base + rng.gauss(0.0, jitter))
        loop.advance(DT)
        rows.append((math.degrees(loop.read_rate()), loop.read_torque(), loop.controller.ctrl.rc.integ))
    return rows


def hp_rms(xs, win=250):
    if len(xs) <= win:
        return float('nan')
    s = sum(xs[:win])
    acc = 0.0
    n = 0
    for i in range(win, len(xs)):
        mean = s / win
        d = xs[i - win // 2] - mean
        acc += d * d
        n += 1
        s += xs[i] - xs[i - win]
    return math.sqrt(acc / n)


def settled_time(rates, thr=1.0, hold=1.5):
    start_i = None
    for i, r in enumerate(rates):
        if abs(r) < thr:
            start_i = i if start_i is None else start_i
            if (i - start_i) * DT >= hold:
                return start_i * DT
        else:
            start_i = None
    return None


def last_exit(rates, band):
    last = None
    for i, r in enumerate(rates):
        if abs(r) >= band:
            last = i
    return 0.0 if last is None else (last + 1) * DT


def step_case(target, P, I, FF, stick, seed, act):
    sp = (stick - 0.1) / 0.9 * 15.0
    loop = start(target, P, I, FF, seed, act)
    loop.set_stick(stick)
    push = record(loop, 60.0)
    loop.set_stick(0.0)
    rel = record(loop, 40.0)
    r = [x[0] for x in push]
    rr = [x[0] for x in rel]
    t10 = next((i * DT for i, x in enumerate(r) if x >= 0.1 * sp), None)
    t90 = next((i * DT for i, x in enumerate(r) if x >= 0.9 * sp), None)
    peak = max(r)
    ess = sum(r[-2500:]) / 2500 - sp
    ts10 = last_exit([x - sp for x in r], 0.1 * sp)
    out = {'sp': sp, 'rise': (t90 - t10) if (t10 is not None and t90 is not None) else None,
           't90': t90, 'peak': peak, 'over': 100.0 * (peak - sp) / sp, 'ts10': ts10, 'ess': ess,
           'integ_end': push[-1][2], 'torque_end': sum(x[1] for x in push[-2500:]) / 2500,
           'r0': rr[0], 'rel': settled_time(rr), 'rel10': last_exit(rr, 0.1 * sp),
           'under': min(rr), 'rel_end': rr[-1]}
    if stick == 0.5:
        out['tq_hp_turn'] = hp_rms([x[1] for x in push[-5000:]])
    return out


def wind_case(target, P, I, FF, seed, act):
    g = GUST[target]
    loop = start(target, P, I, FF, seed, act)
    rest = record(loop, 10.0)
    loop.set_wind(fc.gate.GZ_WORLD_Z_TO_FRD * g)
    w = record(loop, 60.0)
    loop.set_wind(0.0)
    r = [x[0] for x in w]
    peak = max(abs(x) for x in r)
    rec = last_exit(r, 0.25)
    needed = g / AUTH[target]
    out = {'peak': peak, 'recover_025': rec, 'integ_end': w[-1][2], 'needed': needed,
           'rate_end': sum(abs(x) for x in r[-500:]) / 500, 'tq_hp_rest': hp_rms([x[1] for x in rest])}
    # the same gust during a steady 6.67 deg/s turn
    loop = start(target, P, I, FF, seed, act)
    loop.set_stick(0.5)
    turn = record(loop, 50.0)
    base = sum(x[0] for x in turn[-2500:]) / 2500
    loop.set_wind(fc.gate.GZ_WORLD_Z_TO_FRD * g)
    tw = record(loop, 40.0)
    dev = [x[0] - base for x in tw]
    out['turn_dev_peak'] = max(dev, key=abs)
    out['turn_recover_025'] = last_exit(dev, 0.25)
    # stick jitter in the turn
    loop = start(target, P, I, FF, seed, act)
    rng = random.Random(seed + 100)
    loop.set_stick(0.5)
    record(loop, 40.0)
    j = record(loop, 20.0, jitter=0.002, base=0.5, rng=rng)
    out['tq_hp_turn_jitter'] = hp_rms([x[1] for x in j])
    return out


def one(job):
    target, (P, I, FF), seed, act, kind = job
    try:
        if kind == 'wind':
            res = wind_case(target, P, I, FF, seed, act)
        else:
            res = step_case(target, P, I, FF, kind, seed, act)
        err = None
    except Exception as e:
        res, err = {}, repr(e)
    return {'target': target, 'P': P, 'I': I, 'FF': FF, 'seed': seed, 'act': act, 'kind': kind,
            'res': res, 'err': err}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--configs', required=True)
    ap.add_argument('--plants', default='all')
    ap.add_argument('--act', default='{}')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--kinds', default='0.25,0.5,1.0,wind')
    ap.add_argument('--pool', type=int, default=10)
    a = ap.parse_args()
    configs = [tuple(float(x) for x in c.split(',')) for c in a.configs.split(';') if c.strip()]
    plants = fc.TARGETS5 if a.plants == 'all' else [fc.TARGETS5[int(i)] for i in a.plants.split(',')]
    kinds = [k if k == 'wind' else float(k) for k in a.kinds.split(',')]
    act = json.loads(a.act)
    jobs = [(t, c, a.seed, act, k) for c in configs for t in plants for k in kinds]
    print('%d jobs' % len(jobs), flush=True)
    with mp.Pool(a.pool) as pool, open(a.out, 'a') as fh:
        for r in pool.imap_unordered(one, jobs):
            fh.write(json.dumps(r) + '\n')
            fh.flush()


if __name__ == '__main__':
    main()
