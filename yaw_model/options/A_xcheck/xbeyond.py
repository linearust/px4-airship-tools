#!/usr/bin/env python3
"""Cross-check of option A's beyond-gate claims, written separately from its
beyond.py: the shared closed loop (xgate.build, gyro noise on) driven in Acro
from rest, heading from the plant's own psi (not a re-integrated rate).

Scenarios: wind step (10 % of authority, 60 s, then off), the gate-size wind,
a 5 s gust at 10 %, stick steps 0.5 and 0.2 (60 s, then released), hold with
noise (torque command statistics, pod mode changes, tail sign reversals).

    python3 xbeyond.py OUT.jsonl
"""
import json
import math
import multiprocessing as mp
import os
import statistics
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import xgate  # noqa: E402

AUTH = {True: 54.0, False: 4.478}        # N m of yaw per unit command: vectored, Cloudship (gate: authority / 54)
DT = 0.004


def sp_of(stick):
    return max(0.0, (abs(stick) - 0.1) / 0.9) * 15.0 * (1 if stick >= 0 else -1)


def simulate(target, params, act, seed, T, events, probe=None):
    loop = xgate.build(target, params, seed, tuple(act) if act else None)
    loop.set_stick(0.0)
    loop.arm()
    loop.set_mode('manual')
    loop.advance(0.2)
    loop.set_mode('acro')
    psi0 = loop.plant.psi
    ev = sorted(events)
    rows = []
    ei = 0
    for k in range(int(round(T / DT))):
        t = k * DT
        while ei < len(ev) and ev[ei][0] <= t + 1e-9:
            _, kind, v = ev[ei]
            (loop.set_stick if kind == 'stick' else loop.set_wind)(v)
            ei += 1
        loop.advance(DT)
        if probe:
            probe(loop, t)
        if k % 5 == 0:
            # FRD heading = -psi (FLU)
            rows.append((t, math.degrees(loop.plant.rate), -math.degrees(loop.plant.psi - psi0),
                         loop.controller.ctrl.integ, loop.read_torque()))
    return rows


def hold_time(rows, t0, th, hold):
    start = None
    for r in rows:
        if r[0] < t0:
            continue
        if abs(r[1]) < th:
            start = r[0] if start is None else start
            if r[0] - start >= hold:
                return start - t0
        else:
            start = None
    return None


def wind(target, params, act, seed, frac, t_on=2.0, dur=60.0, T=110.0):
    vec = target in xgate.VECTORED
    W = frac * AUTH[vec]
    rows = simulate(target, params, act, seed, T, [(t_on, 'wind', W), (t_on + dur, 'wind', 0.0)])
    on = [r for r in rows if t_on <= r[0] < t_on + dur]
    off = [r for r in rows if r[0] >= t_on + dur]
    h0 = on[0][2]
    return {'peak_rate': max(r[1] for r in on), 'heading_peak': max(r[2] - h0 for r in on),
            'heading_at_off': on[-1][2] - h0, 'heading_final': rows[-1][2] - h0,
            'integ_at_off': on[-1][3], 'recover_0.2': hold_time(on, t_on, 0.2, 2.0),
            'kick_after_off': max(-r[1] for r in off), 'law_deg': math.degrees(frac / params['AS_YAWRATE_I'])}


def step(target, params, act, seed, stick, t_on=2.0, dur=60.0, T=110.0):
    sp = sp_of(stick)
    rows = simulate(target, params, act, seed, T, [(t_on, 'stick', stick), (t_on + dur, 'stick', 0.0)])
    seg = [r for r in rows if t_on <= r[0] < t_on + dur]
    peak = max(r[1] for r in seg)
    last_out = None
    for r in seg:
        if abs(r[1] - sp) > 0.1 * sp:
            last_out = r[0]
    settle = None if last_out is None else last_out - t_on
    if settle is not None and settle > dur - 0.1:
        settle = float('inf')
    rel = [r for r in rows if r[0] >= t_on + dur]
    return {'overshoot_pct': 100 * (peak - sp) / sp, 'settle10': settle,
            'mean_last5': statistics.fmean(r[1] for r in seg if r[0] >= t_on + dur - 5),
            'stop_1': hold_time(rel, t_on + dur, 1.0, 1.5), 'undershoot_after_release': -min(r[1] for r in rel),
            'integ_at_release': seg[-1][3]}


def noise(target, params, act, seed, T=60.0):
    vec = target in xgate.VECTORED
    st = {'tq': [], 'mode_changes': 0, 'modes': None, 'sign': 0, 'rev': 0, 'esc': []}

    def probe(loop, t):
        if t < 5.0:
            return
        st['tq'].append(loop.read_torque())
        if vec:
            modes = tuple(p.mode for p in loop.actuation.act.alloc.pods)
            if st['modes'] is not None and modes != st['modes']:
                st['mode_changes'] += 1
            st['modes'] = modes
        else:
            s = loop.actuation.stage['s']
            st['esc'].append(s)
            sg = (s > 0) - (s < 0)
            if sg and st['sign'] and sg != st['sign']:
                st['rev'] += 1
            if sg:
                st['sign'] = sg

    simulate(target, params, act, seed, T, [], probe)
    out = {'tq_std': statistics.pstdev(st['tq']), 'tq_max': max(abs(x) for x in st['tq'])}
    if vec:
        out['pod_mode_changes'] = st['mode_changes']
    else:
        out['tail_rev_per_s'] = st['rev'] / (T - 5.0)
        out['esc_std'] = statistics.pstdev(st['esc'])
    return out


def job(spec):
    kind, target, params, act, seed = spec
    try:
        if kind == 'wind10':
            r = wind(target, params, act, seed, 0.10)
        elif kind == 'windgate':
            r = wind(target, params, act, seed, 1 / 54.0)
        elif kind == 'gust':
            r = wind(target, params, act, seed, 0.10, dur=5.0, T=60.0)
        elif kind == 'step05':
            r = step(target, params, act, seed, 0.5)
        elif kind == 'step02':
            r = step(target, params, act, seed, 0.2)
        elif kind == 'noise':
            r = noise(target, params, act, seed)
        elif kind == 'noise10':
            import planar
            planar.GYRO_NOISE = xgate.cl.GYRO_NOISE * 10.0
            r = noise(target, params, act, seed)
            planar.GYRO_NOISE = xgate.cl.GYRO_NOISE
        else:
            raise ValueError(kind)
    except Exception as e:
        r = {'error': repr(e)}
    return {'kind': kind, 'target': target, 'params': params, 'act': act, 'seed': seed, 'res': r}


def main():
    out = os.path.join(HERE, sys.argv[1])
    sets = [(0.5, 0.09, 0.3), (0.7, 0.2, 0.3)]
    specs = []
    for P, I, L in sets:
        params = {'AS_YAWRATE_P': P, 'AS_YAWRATE_I': I, 'AS_YR_INT_LIM': L}
        for t in xgate.TARGETS:
            vec = t in xgate.VECTORED
            for k in ('wind10', 'windgate', 'gust', 'step05', 'step02', 'noise', 'noise10'):
                for seed in (11, 12):
                    specs.append((k, t, params, None, seed))
            acts = [['slew', 60.0], ['slew', 240.0], ['servo', 60.0]] if vec else [['tail_tau', 0.05], ['tail_tau', 0.1]]
            for a in acts:
                for k in ('wind10', 'step05', 'step02'):
                    specs.append((k, t, params, a, 11))
    print(len(specs), 'jobs', flush=True)
    with mp.Pool(int(os.environ.get('W', 4))) as pool, open(out, 'w') as fh:
        for r in pool.imap_unordered(job, specs):
            fh.write(json.dumps(r) + '\n')
            fh.flush()
    print('done', flush=True)


if __name__ == '__main__':
    main()
