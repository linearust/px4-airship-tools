#!/usr/bin/env python3
"""Beyond-gate scenarios for the cross-check, on xlib.build() loops, Acro,
gyro noise on; true FRD rate and heading every 20 ms; torque command every
4 ms.

    python3 xbeyond.py OUT.jsonl SCEN[,SCEN] RULES P SEEDS [variant=value]
"""
import json
import math
import multiprocessing as mp
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True
import xlib  # noqa: E402

AUTH = {'vectored': 54.0, 'cloudship': 4.478}
CH = 0.02


def sp_of(stick, dz=0.1, mx=15.0):
    return 0.0 if abs(stick) <= dz else math.copysign((abs(stick) - dz) / (1 - dz) * mx, stick)


class R:
    def __init__(self, target, rule, P, seed, variant):
        self.L = xlib.build(target, rule, P, 0.09, True, seed, variant)
        self.auth = AUTH[xlib.spec(target)[1]]
        self.t = 0.0
        self.rows = []
        self.u = []
        inner = self.L.actuation.step

        def tap(dt, u):
            self.u.append(u)
            return inner(dt, u)

        self.L.actuation.step = tap
        self.L.arm()
        self.L.set_stick(0.0)
        self.L.set_mode('acro')
        self.stick = 0.0
        self.go(5.0)

    def go(self, sec, stick_fn=None, until=None):
        for _ in range(int(round(sec / CH))):
            if stick_fn:
                self.stick = stick_fn(self.t)
                self.L.set_stick(self.stick)
            if until and until(self):
                return
            self.L.advance(CH)
            self.t += CH
            p = self.L.plant
            self.rows.append((self.t, math.degrees(p.rate), -math.degrees(p.psi), self.L._torque,
                              self.L.controller.ctrl.integ, self.stick))

    def stick_to(self, v):
        self.stick = v
        self.L.set_stick(v)

    def wind(self, f):
        self.L.set_wind(f * self.auth)


def hold_below(rows, thr, hold, t0):
    s = None
    for t, r, *_ in rows:
        if abs(r) < thr:
            s = t if s is None else s
            if t - s >= hold:
                return round(s - t0, 3)
        else:
            s = None
    return None


def release_metrics(rel, t_rel):
    psi0 = rel[0][2]
    sgn = 1.0 if rel[0][1] >= 0 else -1.0
    zero = next((i for i, x in enumerate(rel) if x[1] * sgn <= 0.0), None)
    return {'hold1': hold_below(rel, 1.0, 1.5, t_rel), 'hold03': hold_below(rel, 0.3, 3.0, t_rel),
            'coast': (rel[zero][2] - psi0) if zero is not None else (rel[-1][2] - psi0),
            'back_peak': max(0.0, max(-sgn * x[1] for x in rel)),
            'dpsi_end': rel[-1][2] - psi0, 'max_abs_dpsi': max(abs(x[2] - psi0) for x in rel),
            'rate_end': rel[-1][1], 'integ0': rel[0][4], 'integ_end': rel[-1][4]}


def scen(name, target, rule, P, seed, variant):
    X = R(target, rule, P, seed, variant)
    rc = X.L.controller.ctrl.rc
    out = {}
    if name.startswith('rel') and not name.startswith('relwind'):   # rel<S>: stick S 40 s, release 60 s, no wind
        S = float(name[3:])
        X.stick_to(S)
        a = len(X.rows)
        X.go(40.0)
        push = X.rows[a:]
        sp = sp_of(S)
        out['overshoot_pct'] = 100 * (max(x[1] for x in push) - sp) / sp
        out['t90'] = next((x[0] - push[0][0] + CH for x in push if x[1] >= 0.9 * sp), None)
        out['integ_at_release'] = push[-1][4]
        b = len(X.rows)
        tr = X.t
        X.stick_to(0.0)
        X.go(60.0)
        out.update(release_metrics(X.rows[b:], tr))
        u = X.u[-int(60 / 0.004):]
        out['u_step_at_release'] = max(abs(u[i + 1] - u[i]) for i in range(0, 30))
    elif name.startswith('wstep'):                 # wstep<F>: stick 0, wind F x authority, 60 s
        F = float(name[5:])
        a = len(X.rows)
        X.wind(F)
        X.go(60.0)
        rows = X.rows[a:]
        psi0 = rows[0][2]
        tl = [x for x in rows if x[0] >= X.t - 10]
        out = {'max_dpsi': max(abs(x[2] - psi0) for x in rows), 'ss_rate': sum(abs(x[1]) for x in tl) / len(tl),
               'ss_integ': sum(x[4] for x in tl) / len(tl), 'peak': max(abs(x[1]) for x in rows)}
    elif name.startswith('relwind'):               # relwind+/-: 5 % wind, 90 s trim, stick .5 40 s, release 60 s
        F = 0.05 if name.endswith('+') else -0.05
        X.wind(F)
        X.go(90.0)
        out['trim'] = X.L.controller.ctrl.integ
        X.stick_to(0.5)
        X.go(40.0)
        b = len(X.rows)
        tr = X.t
        out['integ_at_release'] = X.rows[-1][4]
        X.stick_to(0.0)
        X.go(60.0)
        out.update(release_metrics(X.rows[b:], tr))
    elif name.startswith('stale'):                 # stale+/-: no wind at deflection, 5 % wind steps in
        F = 0.05 if name.endswith('+') else -0.05  # 20 s into a 40 s half-stick turn, release, 60 s
        X.stick_to(0.5)
        X.go(20.0)
        X.wind(F)
        X.go(20.0)
        b = len(X.rows)
        tr = X.t
        out['integ_at_release'] = X.rows[-1][4]
        X.stick_to(0.0)
        X.go(60.0)
        out.update(release_metrics(X.rows[b:], tr))
    elif name.startswith('gone'):                  # gone+/-: 5 % wind trimmed 90 s, wind stops 20 s into
        F = 0.05 if name.endswith('+') else -0.05  # the 40 s turn, release, 60 s
        X.wind(F)
        X.go(90.0)
        out['trim'] = X.L.controller.ctrl.integ
        X.stick_to(0.5)
        X.go(20.0)
        X.wind(0.0)
        X.go(20.0)
        b = len(X.rows)
        tr = X.t
        out['integ_at_release'] = X.rows[-1][4]
        X.stick_to(0.0)
        X.go(60.0)
        out.update(release_metrics(X.rows[b:], tr))
    elif name == 'edge':                           # 5 % wind trimmed, stick on the deadzone edge + noise
        X.wind(0.05)
        X.go(60.0)
        rng = random.Random(seed + 100)
        a = len(X.rows)
        nu = len(X.u)
        n0 = rc.n_release if hasattr(rc, 'n_release') else 0
        X.go(60.0, lambda t: 0.1 + rng.gauss(0.0, 0.005))
        rows = X.rows[a:]
        u = X.u[nu:]
        out = {'dpsi_60': rows[-1][2] - rows[0][2], 'releases': rc.n_release - n0,
               'du_rms': math.sqrt(sum((u[i + 1] - u[i]) ** 2 for i in range(len(u) - 1)) / (len(u) - 1)),
               'u_std': (lambda m: math.sqrt(sum((x - m) ** 2 for x in u) / len(u)))(sum(u) / len(u))}
    else:
        raise ValueError(name)
    out['n_release'] = getattr(rc, 'n_release', 0)
    return out


def run(job):
    r = dict(job)
    r['m'] = scen(job['scen'], job['target'], job['rule'], job['P'], job['seed'],
                  tuple(job['variant']) if job.get('variant') else None)
    return r


if __name__ == '__main__':
    path = os.path.join(HERE, sys.argv[1])
    scens = sys.argv[2].split(',')
    rules = sys.argv[3].split(',')
    P = float(sys.argv[4])
    seeds = [int(s) for s in sys.argv[5].split(',')]
    variant = None
    targets = xlib.T5
    if len(sys.argv) > 6:
        k, v = sys.argv[6].split('=')
        variant = (k, float(v))
        fam = 'cloudship' if k == 'tail_tau' else 'vectored'
        targets = [t for t in xlib.T5 if xlib.spec(t)[1] == fam]
    jobs = [dict(id='%s|%s|%s|%g|%d|%s' % (s, t, ru, P, sd, variant), scen=s, target=t, rule=ru, P=P, seed=sd,
                 variant=variant) for s in scens for t in targets for ru in rules for sd in seeds]
    done = {json.loads(l)['id'] for l in open(path)} if os.path.exists(path) else set()
    todo = [j for j in jobs if j['id'] not in done]
    print('%d jobs, %d to run' % (len(jobs), len(todo)), flush=True)
    with mp.Pool(int(os.environ.get('W', 12))) as pool, open(path, 'a') as fh:
        for r in pool.imap_unordered(run, todo):
            fh.write(json.dumps(r) + '\n')
            fh.flush()
    print('done', flush=True)
