#!/usr/bin/env python3
"""Diagnostic, NOT option E: is the _fin release gap between the real pods and
the ideal actuator (e_ideal.py) the anti-windup freezing an integrator that is
UNWINDING during a pod swing? RateControl::updateIntegral clamps the error
sign whenever the allocator flags that direction (rate_control.cpp:91-100),
even when integrating that way shrinks |I|. The probe lets such integration
through (only growth of |I| into the flagged direction is blocked) and reruns
the noise-free gate on the vectored plants with band 'base'.

    python3 unwind_probe.py
"""
import multiprocessing as mp
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True


def patched_update_integral(rc):
    import controller as ctl

    def _update_integral(e, dt):
        if rc.sat_pos and not rc.integ < 0.0:
            e = min(e, 0.0)
        if rc.sat_neg and not rc.integ > 0.0:
            e = max(e, 0.0)
        f = e / rc.I_FACTOR_RATE
        f = max(0.0, 1.0 - f * f)
        rate_i = rc.integ + f * rc.i * e * dt
        if ctl.isfinite(rate_i):
            rc.integ = ctl.constrain(rate_i, -rc.int_lim, rc.int_lim)
    return _update_integral


def one(job):
    target, P, probe = job
    import e_model as em
    import gate
    loop = em.build(target, 'base', params={'AS_YAWRATE_P': P, 'AS_YAWRATE_I': 0.09})
    if probe:
        rc = loop.controller.ctrl.rc
        rc._update_integral = patched_update_integral(rc)
    od = os.path.join(HERE, 'tmp_out', str(os.getpid()))
    os.makedirs(od, exist_ok=True)
    res = gate.run_gate(loop, em.model_name(target), out_dir=od)
    return target, P, probe, res.gate_lines[:10]


if __name__ == '__main__':
    jobs = [(t, P, pr) for t in ('gz_airship_vectored_independent', 'gz_airship_vectored_independent_am_fin_drag',
                                 'gz_airship_vectored_independent_fin') for P in (0.5, 0.7) for pr in (True,)]
    with mp.Pool(3) as pool:
        for target, P, probe, lines in pool.imap(one, jobs):
            print('\n== %s P %.2f I 0.09, anti-windup lets |I| shrink: %s' % (target, P, probe))
            print('\n'.join(l for l in lines if '실패' in l or '스틱을 놓으면' in l or '제동' in l))
