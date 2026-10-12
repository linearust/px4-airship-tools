#!/usr/bin/env python3
"""Off-gate checks of option B's claims, my own implementation (gyro noise on).

  step   from rest in Acro: stick s for 90 s, then 0 for 40 s.
         t90 = first time r >= 0.9 sp; peak, overshoot; ss = mean r over the last 10 s
         of the push; rel = the gate's settle rule (|r| < 1 held 1.5 s) from release;
         under = most negative r after release.
  gust   at rest in Acro: the gate's strong gust (1 N m vectored, 4.48/54 N m Cloudship)
         for 60 s: peak |r|, recovery = last time |r| > 0.25; integ/needed at the end.
  tgust  the same gust after 120 s of a stick-0.5 turn (long enough for FF 0 to settle),
         for 60 s: signed peak deviation from the pre-gust mean, recovery = last time
         |r - base| > 0.25, and the base itself.
  noise  high-pass (minus a 1 s centred moving mean) rms of the torque command at rest
         (gyro noise only) and in the turn with a stick jitter N(0, 0.002) every 20 ms.

    python3 xbeyond.py OUT.jsonl SET [--pool 8]
"""
import json
import math
import multiprocessing as mp
import random
import sys

sys.dont_write_bytecode = True
import xcommon as xc  # noqa: E402

DT = 0.004
GUST = {xc.VR: 1.0, xc.VC: 1.0, xc.VF: 1.0, xc.CR: 4.48 / 54.0, xc.CC: 4.48 / 54.0}
AUTH = {xc.VR: 54.0, xc.VC: 54.0, xc.VF: 54.0, xc.CR: 4.48, xc.CC: 4.48}


def start(t, P, I, FF, seed, act):
    loop = xc.make(t, P, I, FF, noise=True, seed=seed, **act)
    loop.set_stick(0.0)
    loop.arm()
    loop.set_mode('manual')
    loop.advance(1.0)
    loop.set_mode('acro')
    loop.advance(2.0)
    return loop


def rec(loop, seconds, jitter=None, base=0.0, rng=None):
    out = []
    for k in range(int(round(seconds / DT))):
        if jitter and k % 5 == 0:
            loop.set_stick(base + rng.gauss(0.0, jitter))
        loop.advance(DT)
        out.append((math.degrees(loop.read_rate()), loop.read_torque(), loop.controller.ctrl.rc.integ))
    return out


def settle(r, thr=1.0, hold=1.5):
    s = None
    for i, x in enumerate(r):
        if abs(x) < thr:
            s = i if s is None else s
            if (i - s) * DT >= hold:
                return s * DT
        else:
            s = None
    return None


def last_exit(r, band):
    last = None
    for i, x in enumerate(r):
        if abs(x) > band:
            last = i
    return 0.0 if last is None else (last + 1) * DT


def hp_rms(x, win=250):
    h = win // 2
    acc, n = 0.0, 0
    s = sum(x[:win])
    for i in range(win, len(x)):
        d = x[i - h] - s / win
        acc += d * d
        n += 1
        s += x[i] - x[i - win]
    return math.sqrt(acc / n) if n else float('nan')


def step(t, P, I, FF, stick, seed, act):
    sp = (stick - 0.1) / 0.9 * 15.0
    loop = start(t, P, I, FF, seed, act)
    loop.set_stick(stick)
    push = rec(loop, 90.0)
    loop.set_stick(0.0)
    rel = rec(loop, 40.0)
    r = [x[0] for x in push]
    rr = [x[0] for x in rel]
    t90 = next((i * DT for i, x in enumerate(r) if x >= 0.9 * sp), None)
    peak = max(r)
    return {'sp': sp, 't90': t90, 'peak': peak, 'over_pct': 100.0 * (peak - sp) / sp,
            'ss': sum(r[-2500:]) / 2500, 'ss_torque': sum(x[1] for x in push[-2500:]) / 2500,
            'integ_end': push[-1][2], 'r0': rr[0], 'rel': settle(rr), 'under': min(rr), 'rel_end': rr[-1]}


def gust(t, P, I, FF, seed, act):
    g = GUST[t]
    loop = start(t, P, I, FF, seed, act)
    rest = rec(loop, 10.0)
    loop.set_wind(xc.gate.GZ_WORLD_Z_TO_FRD * g)
    w = rec(loop, 60.0)
    loop.set_wind(0.0)
    r = [x[0] for x in w]
    out = {'peak': max(abs(x) for x in r), 'recover': last_exit(r, 0.25),
           'integ_ratio': w[-1][2] / (g / AUTH[t]), 'tq_hp_rest': hp_rms([x[1] for x in rest])}
    loop = start(t, P, I, FF, seed, act)
    loop.set_stick(0.5)
    turn = rec(loop, 120.0)
    base = sum(x[0] for x in turn[-2500:]) / 2500
    drift = sum(x[0] for x in turn[-2500:-1250]) / 1250 - sum(x[0] for x in turn[-1250:]) / 1250
    loop.set_wind(xc.gate.GZ_WORLD_Z_TO_FRD * g)
    tw = rec(loop, 60.0)
    dev = [x[0] - base for x in tw]
    out.update({'turn_base': base, 'turn_base_drift_5s': drift, 'turn_dev_peak': max(dev, key=abs),
                'turn_recover': last_exit(dev, 0.25), 'turn_dev_end': sum(dev[-1250:]) / 1250,
                'turn_integ_before': turn[-1][2], 'turn_integ_end': tw[-1][2]})
    loop = start(t, P, I, FF, seed, act)
    rng = random.Random(seed + 100)
    loop.set_stick(0.5)
    rec(loop, 60.0)
    j = rec(loop, 20.0, jitter=0.002, base=0.5, rng=rng)
    out['tq_hp_turn_jitter'] = hp_rms([x[1] for x in j])
    return out


def one(job):
    t, P, I, FF, kind, seed, act = job
    try:
        res = gust(t, P, I, FF, seed, act) if kind == 'gust' else step(t, P, I, FF, kind, seed, act)
        err = None
    except Exception:
        import traceback
        res, err = {}, traceback.format_exc()
    return {'target': t, 'P': P, 'I': I, 'FF': FF, 'kind': kind, 'seed': seed, 'act': act, 'res': res, 'err': err}


def jobs(name):
    out = []
    if name == 'main':
        cfg = [(xc.VR, 0.0), (xc.VC, 0.0), (xc.VC, 1.0), (xc.CR, 0.0), (xc.CC, 0.0), (xc.CC, 0.05),
               (xc.VF, 0.0), (xc.VF, 0.3)]
        for t, ff in cfg:
            for k in (0.25, 0.5, 1.0, 'gust'):
                out.append((t, 0.65, 0.07, ff, k, 1, {}))
        for P, I in ((0.5, 0.09), (0.7, 0.09)):         # B's wind-recovery comparison
            for t in (xc.VR, xc.VC, xc.CC):
                out.append((t, P, I, 0.0, 'gust', 1, {}))
    elif name == 'act':
        for a in ({'slew': 60.0}, {'phys_slew': 60.0}, {'slew': 240.0}):
            for t, ff in ((xc.VC, 1.0), (xc.VC, 0.0), (xc.VF, 0.3), (xc.VF, 0.0)):
                out.append((t, 0.65, 0.07, ff, 0.5, 1, a))
    else:
        raise SystemExit(name)
    return out


def main():
    out, name = sys.argv[1], sys.argv[2]
    n = int(sys.argv[sys.argv.index('--pool') + 1]) if '--pool' in sys.argv else 8
    js = jobs(name)
    print('%s: %d jobs' % (name, len(js)), flush=True)
    with mp.Pool(n) as pool, open(out, 'a') as fh:
        for r in pool.imap_unordered(one, js):
            fh.write(json.dumps(r) + '\n')
            fh.flush()
    print('DONE ' + name, flush=True)


if __name__ == '__main__':
    main()
