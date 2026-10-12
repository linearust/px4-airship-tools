#!/usr/bin/env python3
"""What the gate does not check: stick steps, wind steps and gusts, noise on
the torque command, and the same under slower/faster actuators. Every run is
the shared closed loop (optA.build: controller + actuation + RollPlanarPlant,
gyro noise on) driven directly, Acro, from rest with the integrator cleared
(a pass through Manual), no drift.

    python3 beyond.py <out.jsonl> P I LIM [P I LIM ...]
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
import optA  # noqa: E402

AUTH = {'vectored': 54.0, 'cloudship': 4.478}
DT = 0.004


def airframe(target):
    return 'vectored' if target in optA.VECTORED else 'cloudship'


def sp_of(stick):
    dz = 0.1
    return max(0.0, (abs(stick) - dz) / (1 - dz)) * 15.0 * (1 if stick >= 0 else -1)


def simulate(target, params, act, seed, T, events, probe=None):
    """events: [(t, 'stick'|'wind', value)], wind in N m FRD. Returns rows every
    0.02 s: t, true rate deg/s FRD, measured, torque, integ, heading deg (FRD, unwrapped)."""
    loop = optA.build(target, params, seed, act)
    loop.set_stick(0.0)
    loop.arm()
    loop.set_mode('manual')
    loop.advance(0.2)
    loop.set_mode('acro')
    ev = sorted(events)
    rows = []
    n = int(round(T / DT))
    heading = 0.0
    ei = 0
    for k in range(n):
        t = k * DT
        while ei < len(ev) and ev[ei][0] <= t + 1e-9:
            _, kind, v = ev[ei]
            if kind == 'stick':
                loop.set_stick(v)
            else:
                loop.set_wind(v)
            ei += 1
        loop.advance(DT)
        r = loop.plant.rate
        heading += math.degrees(r) * DT
        if probe:
            probe(loop, t)
        if k % 5 == 0:
            rows.append((round(t, 3), math.degrees(r), math.degrees(loop.read_rate()), loop.read_torque(),
                         loop.controller.ctrl.integ, heading))
    return rows


def step_metrics(rows, t0, t1, sp):
    """Stick step at t0, released at t1."""
    seg = [r for r in rows if t0 <= r[0] < t1]
    rate = [r[1] for r in seg]
    s = math.copysign(1, sp)
    peak = max(x * s for x in rate)

    def first(pred, rs):
        for r in rs:
            if pred(r):
                return r[0] - rs[0][0]
        return None

    t10 = first(lambda r: r[1] * s >= 0.1 * abs(sp), seg)
    t90 = first(lambda r: r[1] * s >= 0.9 * abs(sp), seg)
    # settling: last time outside +-10 % of sp before t1 (window = the hold)
    band = 0.1 * abs(sp)
    last_out = None
    for r in seg:
        if abs(r[1] - sp) > band:
            last_out = r[0]
    settle10 = None if last_out is None else last_out - t0
    if settle10 is not None and settle10 >= (t1 - t0) - 0.05:
        settle10 = float('inf')
    band5 = 0.05 * abs(sp)
    last_out5 = None
    for r in seg:
        if abs(r[1] - sp) > band5:
            last_out5 = r[0]
    settle5 = None if last_out5 is None else last_out5 - t0
    if settle5 is not None and settle5 >= (t1 - t0) - 0.05:
        settle5 = float('inf')
    tail = [r[1] for r in seg if r[0] >= t1 - 5.0]
    # release
    rel = [r for r in rows if r[0] >= t1]
    stop1 = None
    start = None
    for r in rel:
        if abs(r[1]) < 1.0:
            start = r[0] if start is None else start
            if r[0] - start >= 1.5:
                stop1 = start - t1
                break
        else:
            start = None
    stop02 = None
    start = None
    for r in rel:
        if abs(r[1]) < 0.2:
            start = r[0] if start is None else start
            if r[0] - start >= 2.0:
                stop02 = start - t1
                break
        else:
            start = None
    under = min(x * s for x in (r[1] for r in rel)) if rel else None
    coast = (rel[-1][5] - rel[0][5]) * s if rel else None
    return {'peak': peak, 'overshoot_pct': 100.0 * (peak - abs(sp)) / abs(sp),
            'rise_10_90': (t90 - t10) if (t10 is not None and t90 is not None) else None,
            'settle10': settle10, 'settle5': settle5, 'mean_last5': statistics.fmean(tail) * s,
            'stop_1deg': stop1, 'stop_0.2deg': stop02, 'undershoot': under, 'coast_deg': coast}


def wind_metrics(rows, t0, t1, W_frd):
    """Wind step at t0 (FRD moment W), off at t1."""
    s = math.copysign(1, W_frd)
    on = [r for r in rows if t0 <= r[0] < t1]
    off = [r for r in rows if r[0] >= t1]
    peak = max(r[1] * s for r in on)
    h0 = on[0][5]
    hpeak = max((r[5] - h0) * s for r in on)
    hend = (on[-1][5] - h0) * s
    rec = None
    start = None
    for r in on:
        if abs(r[1]) < 0.2:
            start = r[0] if start is None else start
            if r[0] - start >= 2.0:
                rec = start - t0
                break
        else:
            start = None
    integ_end = on[-1][4]
    # after release: the integrator now pushes the other way
    peak_back = max(-r[1] * s for r in off) if off else None
    h1 = off[0][5] if off else None
    back_deg = max((h1 - r[5]) * s for r in off) if off else None
    return {'peak_rate': peak, 'heading_peak_deg': hpeak, 'heading_at_off_deg': hend,
            'recover_0.2deg': rec, 'integ_at_off': integ_end, 'peak_rate_after_off': peak_back,
            'heading_back_after_off_deg': back_deg}


def noise_metrics(target, params, act, seed, T=60.0):
    """Hold, stick 0, no wind: torque command noise and what it does downstream."""
    af = airframe(target)
    st = {'torque': [], 'esc': [], 'modes': 0, 'zero_x': 0, 'prev_sign': 0, 'prev_modes': None, 'tilt_moves': 0.0,
          'prev_tilt': None}

    def probe(loop, t):
        if t < 5.0:
            return
        st['torque'].append(loop.read_torque())
        if af == 'vectored':
            pods = loop.actuation.act.alloc.pods
            modes = tuple(p.mode for p in pods)
            if st['prev_modes'] is not None and modes != st['prev_modes']:
                st['modes'] += 1
            st['prev_modes'] = modes
            tilt = tuple(p.tilt for p in pods)
            if st['prev_tilt'] is not None:
                st['tilt_moves'] += sum(abs(a - b) for a, b in zip(tilt, st['prev_tilt']))
            st['prev_tilt'] = tilt
            st['esc'].append(max(loop.actuation.row['c_s'], loop.actuation.row['c_p']))
        else:
            s = loop.actuation.stage['s']
            st['esc'].append(s)
            sg = (s > 0) - (s < 0)
            if sg and st['prev_sign'] and sg != st['prev_sign']:
                st['zero_x'] += 1
            if sg:
                st['prev_sign'] = sg

    rows = simulate(target, params, act, seed, T, [], probe)
    dur = T - 5.0
    out = {'torque_std': statistics.pstdev(st['torque']), 'torque_max': max(abs(x) for x in st['torque']),
           'esc_std': statistics.pstdev(st['esc']), 'esc_max': max(abs(x) for x in st['esc']),
           'rate_rms': math.sqrt(statistics.fmean([r[1] ** 2 for r in rows if r[0] >= 5.0]))}
    if af == 'vectored':
        out['pod_mode_changes_per_min'] = st['modes'] / dur * 60
        out['tilt_travel_deg_per_min'] = math.degrees(st['tilt_moves']) / dur * 60
    else:
        out['tail_reversals_per_s'] = st['zero_x'] / dur
    return out


def job(spec):
    kind, target, params, act, seed = spec
    af = airframe(target)
    A = AUTH[af]
    try:
        if kind.startswith('step'):
            stick = {'step_small': 0.2, 'step_half': 0.5, 'step_full': 1.0}[kind]
            sp = sp_of(stick)
            rows = simulate(target, params, act, seed, 110.0, [(2.0, 'stick', stick), (62.0, 'stick', 0.0)])
            res = step_metrics(rows, 2.0, 62.0, sp)
        elif kind.startswith('wind'):
            frac = {'wind_gate': 1 / 54.0, 'wind_10pct': 0.10}[kind]
            W = frac * A
            rows = simulate(target, params, act, seed, 100.0, [(2.0, 'wind', W), (62.0, 'wind', 0.0)])
            res = wind_metrics(rows, 2.0, 62.0, W)
        elif kind == 'gust':
            W = 0.10 * A
            rows = simulate(target, params, act, seed, 60.0, [(2.0, 'wind', W), (7.0, 'wind', 0.0)])
            res = wind_metrics(rows, 2.0, 7.0, W)
            on = [r for r in rows if r[0] >= 2.0]
            res['heading_max_deg'] = max(r[5] - on[0][5] for r in on)
            res['heading_final_deg'] = on[-1][5] - on[0][5]
            res['rate_max_any'] = max(abs(r[1]) for r in on)
        elif kind.startswith('noise'):
            res = noise_metrics(target, params, act, seed)
        else:
            raise ValueError(kind)
    except Exception as e:  # report, do not hide
        res = {'error': repr(e)}
    return {'kind': kind, 'target': target, 'params': params, 'act': act, 'seed': seed, 'res': res}


KINDS = ['step_small', 'step_half', 'step_full', 'wind_gate', 'wind_10pct', 'gust', 'noise']


def acts_for(target):
    if target in optA.VECTORED:
        return [None, ('slew', 60), ('slew', 240), ('servo', 60), ('rotor', 2.0), ('noise', 10.0)]
    return [None, ('tail_tau', 0.025), ('tail_tau', 0.05), ('tail_tau', 0.1), ('noise', 10.0)]


def main():
    out = os.path.join(HERE, sys.argv[1])
    sets = [tuple(float(x) for x in sys.argv[i:i + 3]) for i in range(2, len(sys.argv), 3)]
    specs = []
    for P, I, L in sets:
        params = {'AS_YAWRATE_P': P, 'AS_YAWRATE_I': I, 'AS_YR_INT_LIM': L}
        for t in optA.TARGETS:
            for act in acts_for(t):
                kinds = KINDS if act is None else (['noise'] if act[0] == 'noise' else
                                                   ['step_half', 'step_small', 'wind_10pct', 'gust', 'noise'])
                for k in kinds:
                    specs.append((k, t, params, act, 1))
    print(len(specs), 'jobs', flush=True)
    with mp.Pool(int(os.environ.get("W", 15))) as pool, open(out, "w") as fh:
        for r in pool.imap_unordered(job, specs):
            fh.write(json.dumps(r) + '\n')
            fh.flush()
    print('done', flush=True)


if __name__ == '__main__':
    main()
