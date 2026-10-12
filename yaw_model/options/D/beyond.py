#!/usr/bin/env python3
"""What the gate does not check: scenarios driven straight on the closed-loop
model (option_d.build), Acro, loop closed, gyro noise on. Metrics use the
plant's TRUE yaw rate (FRD) and heading, sampled every 20 ms; the torque
command and the delivered moment are tapped on every 4 ms tick.

Scenarios (sp = stick setpoint, 0.5 stick -> 6.667 deg/s):
  step{S}      rest, stick S for 40 s, stick 0 for 40 s (S = 0.2, 0.5, 1.0)
  partial      stick 0.5 for 40 s, then 0.2 for 30 s (sp 6.67 -> 1.67)
  rev_step     stick 0.5 for 40 s, then -0.5 in one sample, 40 s
  rev_ramp     same through a 0.2 s ramp at 20 Hz (one sample lands at 0)
  wind{F}      rest, stick 0, wind step F x authority, 60 s
  relwind{+-}  wind 0.05 x authority (+ aids the turn, - opposes), 60 s trim,
               stick 0.5 for 40 s, stick 0 for 40 s
  nudge        wind 0.05 x authority, 60 s trim, then 60 s of stick
               0.13 sin(2 pi 0.25 t): small corrections that enter and leave
               the 0.1 deadzone four times a cycle (sp peaks 0.5 deg/s)
  edge         wind 0.05 x authority, 60 s trim, then 60 s of the stick
               resting at the deadzone edge, 0.1 + N(0, 0.005) per 20 ms
               (a 50 Hz manual_control stream)
  rest         60 s at rest, stick 0, no wind: torque-command noise
  restwind     60 s after a 60 s trim in wind 0.05 x authority: the same
"""
import json
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True

import option_d as od  # noqa: E402

DZ = 0.1
MAX = 15.0
CH = 0.02            # sample period of the metrics, s


def sp_of(stick):
    return 0.0 if abs(stick) <= DZ else math.copysign((abs(stick) - DZ) / (1 - DZ) * MAX, stick)


class Tap:
    """Per-tick torque command, delivered moment and allocator tilt."""

    def __init__(self, loop):
        self.loop = loop
        self.inner = loop.actuation.step
        loop.actuation.step = self.step
        self.vectored = hasattr(loop.actuation, 'act')
        self.auth = 54.0 if self.vectored else 4.478
        self.reset_counters()
        self._tilts = None

    def reset_counters(self):
        self.n = 0
        self.u_sum = self.u_sq = 0.0
        self.du_sq = 0.0
        self.u_prev = None
        self.u_sign_changes = 0
        self.N_sign_changes = 0
        self._N_sign = 0
        self._u_sign = 0
        self.tilt_travel = 0.0

    def step(self, dt, u):
        N, un = self.inner(dt, u)
        self.n += 1
        self.u_sum += u
        self.u_sq += u * u
        if self.u_prev is not None:
            self.du_sq += (u - self.u_prev) ** 2
        self.u_prev = u
        s = (u > 1e-3) - (u < -1e-3)
        if s and self._u_sign and s != self._u_sign:
            self.u_sign_changes += 1
        if s:
            self._u_sign = s
        dbN = 0.005 * self.auth          # 0.5 % of the yaw authority
        s = (N > dbN) - (N < -dbN)
        if s and self._N_sign and s != self._N_sign:
            self.N_sign_changes += 1
        if s:
            self._N_sign = s
        if self.vectored:
            st = self.loop.actuation.status()
            t = (st['tilt_s'], st['tilt_p'])
            if self._tilts is not None:
                self.tilt_travel += abs(t[0] - self._tilts[0]) + abs(t[1] - self._tilts[1])
            self._tilts = t
        return N, un

    def summary(self, seconds):
        n = max(self.n, 1)
        mean = self.u_sum / n
        return {'u_mean': mean, 'u_std': math.sqrt(max(self.u_sq / n - mean * mean, 0.0)),
                'du_rms': math.sqrt(self.du_sq / max(n - 1, 1)),
                'u_sign_per_min': self.u_sign_changes * 60.0 / seconds,
                'N_sign_per_min': self.N_sign_changes * 60.0 / seconds,
                'tilt_deg_per_min': self.tilt_travel * 60.0 / seconds}


class Run:
    def __init__(self, target, rule, gains, seed, slew=None, tail_tau=None):
        self.loop = od.build(target, rule, params=od.GAINS[gains], noise=True, seed=seed,
                             slew_deg=slew, tail_tau=tail_tau)
        self.target = target
        self.auth = od.AUTHORITY[od.airframe_of(target)]
        self.tap = Tap(self.loop)
        self.t = 0.0
        self.rows = []       # (t, true rate deg/s, heading deg, torque, integ, stick)
        L = self.loop
        L.arm()
        L.set_stick(0.0)
        L.set_mode('acro')
        self.stick = 0.0
        self.go(5.0)

    wind_fn = None       # heading (deg, FRD) -> fraction of authority, updated every CH

    def go(self, seconds, stick_fn=None, until=None):
        L = self.loop
        n = int(round(seconds / CH))
        for _ in range(n):
            if stick_fn is not None:
                self.stick = stick_fn(self.t)
                L.set_stick(self.stick)
            if self.wind_fn is not None:
                L.set_wind(self.wind_fn(-math.degrees(L.plant.psi)) * self.auth)
            if until is not None and until(self):
                return
            L.advance(CH)
            self.t += CH
            self.rows.append((self.t, math.degrees(L.plant.rate), -math.degrees(L.plant.psi),
                              L._torque, L.controller.ctrl.integ, self.stick))

    def set_stick(self, v):
        self.stick = v
        self.loop.set_stick(v)

    def wind(self, frac):
        self.loop.set_wind(frac * self.auth)

    def mark(self):
        return len(self.rows)


def held_below(rows, thr, hold, t0):
    """First time after t0 from which |r| < thr holds for `hold` s; None if never."""
    start = None
    for t, r, *_ in rows:
        if abs(r) < thr:
            start = t if start is None else start
            if t - start >= hold:
                return start - t0
        else:
            start = None
    return None


def held_near(rows, target, tol, hold, t0):
    start = None
    for t, r, *_ in rows:
        if abs(r - target) < tol:
            start = t if start is None else start
            if t - start >= hold:
                return start - t0
        else:
            start = None
    return None


def step_metrics(push, rel, sp, t_rel):
    rates = [r for _, r, *_ in push]
    t0 = push[0][0]
    peak = max(rates)
    t90 = next((t - t0 for t, r, *_ in push if r >= 0.9 * sp), None)
    # settle: last time outside +-10 % of sp
    band = 0.1 * sp
    last_out = t0
    for t, r, *_ in push:
        if abs(r - sp) > band:
            last_out = t
    tail = [r for t, r, *_ in push if t >= push[-1][0] - 5.0]
    out = {'overshoot_pct': 100.0 * (peak - sp) / sp, 't90': t90, 'settle10': last_out - t0,
           'mean_last5': sum(tail) / len(tail), 'integ_at_release': push[-1][4]}
    psi0 = rel[0][2]
    out['stop_1'] = held_below(rel, 1.0, 1.5, t_rel)
    out['stop_03'] = held_below(rel, 0.3, 3.0, t_rel)
    zero = next((i for i, row in enumerate(rel) if row[1] <= 0.0), None)
    out['coast_deg'] = (rel[zero][2] - psi0) if zero is not None else (rel[-1][2] - psi0)
    out['reverse_peak'] = -min(0.0, min(r for _, r, *_ in rel))
    out['drift_end_deg'] = rel[-1][2] - psi0
    out['rate_10s'] = next((r for t, r, *_ in rel if t - t_rel >= 10.0), None)
    out['rate_20s'] = next((r for t, r, *_ in rel if t - t_rel >= 20.0), None)
    out['integ_end'] = rel[-1][4]
    return out


def scenario(name, target, rule, gains, seed, slew=None, tail_tau=None):
    R = Run(target, rule, gains, seed, slew, tail_tau)
    res = {}

    if name.startswith('step'):
        S = float(name[4:])
        sp = sp_of(S)
        R.set_stick(S)
        a = R.mark()
        R.go(40.0)
        b = R.mark()
        t_rel = R.t
        R.set_stick(0.0)
        R.tap.reset_counters()
        R.go(40.0)
        res = step_metrics(R.rows[a:b], R.rows[b:], sp, t_rel)
        res.update({'rel_' + k: v for k, v in R.tap.summary(40.0).items()})

    elif name == 'partial':
        R.set_stick(0.5)
        R.go(40.0)
        b = R.mark()
        t1 = R.t
        R.set_stick(0.2)
        R.go(30.0)
        sp2 = sp_of(0.2)
        rel = R.rows[b:]
        res = {'settle_new_sp': held_near(rel, sp2, 0.5, 1.5, t1),
               'undershoot': max(0.0, sp2 - min(r for _, r, *_ in rel)),
               'mean_last5': sum(r for t, r, *_ in rel if t >= R.t - 5.0) / sum(1 for t, *_ in rel if t >= R.t - 5.0)}

    elif name in ('rev_step', 'rev_ramp'):
        R.set_stick(0.5)
        R.go(40.0)
        b = R.mark()
        t1 = R.t
        if name == 'rev_step':
            R.set_stick(-0.5)
            R.go(40.0)
        else:
            vals = [0.25, 0.0, -0.25, -0.5]
            for v in vals:
                R.set_stick(v)
                R.go(0.05)
            R.go(40.0 - 0.2)
        sp = sp_of(-0.5)
        rel = R.rows[b:]
        res = {'t_reach90': next((t - t1 for t, r, *_ in rel if r <= 0.9 * sp), None),
               'settle10': held_near(rel, sp, 0.1 * abs(sp), 3.0, t1),
               'overshoot_pct': 100.0 * (-min(r for _, r, *_ in rel) - abs(sp)) / abs(sp)}

    elif name.startswith('wind'):
        F = float(name[4:])
        a = R.mark()
        t0 = R.t
        R.wind(F)
        R.go(60.0)
        rows = R.rows[a:]
        psi0 = rows[0][2]
        tail = [row for row in rows if row[0] >= R.t - 10.0]
        res = {'peak': max(abs(r) for _, r, *_ in rows),
               'max_dpsi': max(abs(row[2] - psi0) for row in rows),
               'ss_rate': sum(abs(row[1]) for row in tail) / len(tail),
               'ss_integ': sum(row[4] for row in tail) / len(tail),
               'recover_03': held_below(rows, 0.3, 3.0, t0)}

    elif name.startswith('relwind'):
        F = 0.05 if name.endswith('+') else -0.05
        R.wind(F)
        R.go(60.0)
        trim = R.loop.controller.ctrl.integ
        R.set_stick(0.5)
        a = R.mark()
        R.go(40.0)
        b = R.mark()
        t_rel = R.t
        R.set_stick(0.0)
        R.go(40.0)
        res = step_metrics(R.rows[a:b], R.rows[b:], sp_of(0.5), t_rel)
        res['trim_before'] = trim
        res['peak_after'] = max(abs(r) for _, r, *_ in R.rows[b:])

    elif name in ('nudge', 'edge'):
        R.wind(0.05)
        R.go(60.0)
        a = R.mark()
        t0 = R.t
        rng = random.Random(seed + 100)
        R.loop.controller.ctrl.events.clear()
        if name == 'nudge':
            R.go(60.0, lambda t: 0.13 * math.sin(2 * math.pi * 0.25 * (t - t0)))
        else:
            R.go(60.0, lambda t: DZ + rng.gauss(0.0, 0.005))
        rows = R.rows[a:]
        psi0 = rows[0][2]
        spm = sum(sp_of(row[5]) for row in rows) / len(rows)
        res = {'mean_rate': sum(row[1] for row in rows) / len(rows),
               'mean_abs_rate': sum(abs(row[1]) for row in rows) / len(rows),
               'mean_sp': spm,
               'dpsi_60s': rows[-1][2] - psi0,
               'dpsi_vs_sp': (rows[-1][2] - psi0) - spm * 60.0,
               'events': len(R.loop.controller.ctrl.events),
               'integ_end': rows[-1][4]}

    elif name.startswith('wcock'):
        # wcock<turn>_<wind dir>: toy weathercock moment -W0 sin(psi - psi_w),
        # W0 = 0.05 x authority, psi_w = start heading + dir; 90 s trim; stick
        # 0.5 until the heading has turned <turn> deg; release; 60 s.
        turn, wdir = (float(x) for x in name[5:].split('_'))
        W0 = 0.05
        psi_w = -math.degrees(R.loop.plant.psi) + wdir
        R.wind_fn = lambda psi: -W0 * math.sin(math.radians(psi - psi_w))
        R.go(90.0)
        psi1 = R.rows[-1][2]
        trim = R.loop.controller.ctrl.integ
        R.set_stick(0.5)
        a = R.mark()
        R.go(200.0, until=lambda run: run.rows and run.rows[-1][2] - psi1 >= turn)
        b = R.mark()
        t_rel = R.t
        R.set_stick(0.0)
        R.go(60.0)
        rel = R.rows[b:]
        psi_r = rel[0][2]
        res = {'trim_before': trim, 'integ_at_release': R.rows[b - 1][4],
               'wind_at_release': -W0 * math.sin(math.radians(psi_r - psi_w)),
               'stop_1': held_below(rel, 1.0, 1.5, t_rel), 'stop_03': held_below(rel, 0.3, 3.0, t_rel),
               'peak_after': max(abs(r) for _, r, *_ in rel),
               'max_dpsi': max(abs(row[2] - psi_r) for row in rel),
               'final_dpsi': rel[-1][2] - psi_r,
               'reverse_peak': -min(0.0, min(r for _, r, *_ in rel))}

    elif name in ('rest', 'restwind'):
        if name == 'restwind':
            R.wind(0.05)
            R.go(60.0)
        R.tap.reset_counters()
        a = R.mark()
        R.go(60.0)
        rows = R.rows[a:]
        res = R.tap.summary(60.0)
        res['rate_std'] = math.sqrt(sum(r * r for _, r, *_ in rows) / len(rows))
    else:
        raise ValueError(name)

    return res


def run_one(job):
    r = dict(job)
    r['m'] = scenario(job['scen'], job['target'], job['rule'], job['gains'], job['seed'],
                      job.get('slew'), job.get('tail_tau'))
    return r


def run_jobs(jobs, path, workers=8):
    import multiprocessing as mp
    done = set()
    if os.path.exists(path):
        for line in open(path):
            done.add(json.loads(line)['id'])
    todo = [j for j in jobs if j['id'] not in done]
    print('%d jobs, %d done, %d to run' % (len(jobs), len(done), len(todo)), flush=True)
    with mp.Pool(workers) as pool, open(path, 'a') as fh:
        for i, r in enumerate(pool.imap_unordered(run_one, todo), 1):
            fh.write(json.dumps(r) + '\n')
            fh.flush()
            if i % 50 == 0:
                print('  %d / %d' % (i, len(todo)), flush=True)
