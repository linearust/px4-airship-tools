#!/usr/bin/env python3
"""Upper bound for anything an allocator change can do: an IDEAL reversible
yaw actuator on the vectored plants, N = 54 u clamped to +-54 N m, applied in
the step it is commanded, saturation reported only beyond |u| = 1, no pods,
no band, no swing, no surge force. If a gate check fails with this, no steer
band change (report or deliver) can pass it: the failure is the controller's.

    python3 e_ideal.py
"""
import multiprocessing as mp
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True

import e_model as em  # noqa: E402  (puts the shared model on sys.path)
import controller as ctl  # noqa: E402

AUTHORITY = 54.0


class IdealAct:
    def __init__(self):
        self.N = 0.0
        self.u = 0.0

    def step(self, dt, u):
        c = ctl.constrain(u, -1.0, 1.0)
        self.u = u
        self.N = AUTHORITY * c
        short = u - c
        return self.N, (1.0 if short > 1e-4 else -1.0 if short < -1e-4 else 0.0)

    def body_forces(self):
        return 0.0, 0.0

    def pod_wrench(self, dt):
        return (0.0, 0.0, 0.0, 0.0, 0.0, -self.N)

    def status(self):
        return {}


def build(target, params=None, noise=False, seed=1):
    import planar
    model, airframe = em.cl.TARGETS.get(target) or em.cl.EXTRA_TARGETS[target]
    assert airframe == 'vectored'
    controller = ctl.GateController(airframe, params, filter_gyro=False)
    body = planar.RollPlanarPlant(target, noise=noise, seed=seed, model=model)
    return em.TracingLoop(controller, IdealAct(), body, False)


def one(job):
    target, P, I, seed = job
    import gate
    loop = build(target, {'AS_YAWRATE_P': P, 'AS_YAWRATE_I': I}, noise=seed is not None, seed=seed or 1)
    od = os.path.join(HERE, 'tmp_out', str(os.getpid()))
    os.makedirs(od, exist_ok=True)
    res = gate.run_gate(loop, em.model_name(target), out_dir=od)
    return target, P, I, seed, res.gate_lines[:10]


if __name__ == '__main__':
    jobs = [(t, P, I, s) for t in ('gz_airship_vectored_independent', 'gz_airship_vectored_independent_am_fin_drag',
                                   'gz_airship_vectored_independent_fin')
            for P, I in ((0.5, 0.09), (0.7, 0.09)) for s in (None, 1)]
    with mp.Pool(4) as pool:
        for target, P, I, seed, lines in pool.imap(one, jobs):
            print('\n== IDEAL actuator  %s  P %.2f I %.2f  %s' % (target, P, I,
                                                                 'noise-free' if seed is None else 'noise seed %d' % seed))
            print('\n'.join(lines))
