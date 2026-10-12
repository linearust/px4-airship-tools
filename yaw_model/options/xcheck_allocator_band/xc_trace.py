#!/usr/bin/env python3
"""Where does the release time go? Noise-free gate run on xc_model, per-tick
record of the Acro and Stabilized releases: rate, torque command u, the
integrator, the saturation flags, the pods' modes and tilts, the yaw the
pods deliver (normalized) and the share withheld BY CHOICE (u - achieved
while both pods count as held). Summaries per release:
  t_u_neg      first time u < 0 (the loop starts asking to brake)
  swing        time with a pod in Steering after u < 0 (a reversal swing)
  flag_s       time the saturation flag is set in the release
  frozen_unwind  integrator-seconds lost: sum of |I e dt| blocked by the
                 flag while the integrator was unwinding (sign(e) != sign(I))
  held_Nms     integral of the by-choice withheld yaw x 54 N m [N m s]
  held_max     largest by-choice withheld yaw [normalized]

    python3 xc_trace.py TARGET VARIANT P
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True

import xc_model as xm  # noqa: E402
import gate  # noqa: E402
import vectored_actuation as va  # noqa: E402

DEG = 180.0 / math.pi


def main(target, variant, P):
    loop = xm.build(target, variant, params={'AS_YAWRATE_P': P, 'AS_YAWRATE_I': 0.09}, trace=True)
    act = loop.actuation
    rec = []
    orig = act.step

    def step(dt, u):
        N, sat = orig(dt, u)
        pods = act.act.alloc.pods
        held = all(p.held_by_choice() for p in pods)
        rec.append((u, act.act.alloc.achieved_yaw, pods[0].mode, pods[1].mode,
                    math.degrees(pods[0].tilt), math.degrees(pods[1].tilt), held, sat, N))
        return N, sat

    act.step = step
    od = os.path.join(HERE, 'tmp_out', 'trace')
    os.makedirs(od, exist_ok=True)
    res = gate.run_gate(loop, xm.model_name(target), out_dir=od)
    print('\n'.join(res.gate_lines[:10]))
    tr = loop.trace
    assert len(tr) == len(rec)
    starts = [k for k in range(1, len(tr)) if tr[k - 1][2] == 0.5 and tr[k][2] == 0 and tr[k][1] in ('acro', 'stabilized')]
    Ii = 0.09
    for k0 in starts:
        k1 = min(len(tr), k0 + int(12.0 / 0.004))
        t0 = tr[k0][0]
        t_u_neg = None
        swing = flag = frozen = held_nms = held_max = 0.0
        for k in range(k0, k1):
            sim, mode, stick, meas, u, integ, moment, wind, sp, sn, _ = tr[k]
            uu, ach, ms, mp_, ts_, tp_, held, sat, N = rec[k]
            if t_u_neg is None and u < 0:
                t_u_neg = sim - t0
            if t_u_neg is not None and (ms == va.STEERING or mp_ == va.STEERING) and abs(u) <= 0.5:
                swing += 0.004
            if sp or sn:
                flag += 0.004
                e = -meas            # rate setpoint 0 after release
                blocked = (sp and e > 0) or (sn and e < 0)
                if blocked and integ * e < 0:
                    frozen += abs(Ii * e * 0.004)
            if held:
                w = u - ach
                held_nms += abs(w) * 54.0 * 0.004
                held_max = max(held_max, abs(w))
        print('\n=== release in %s at %.1f: rate %.2f -> %.2f deg/s, integ %.4f -> %.4f'
              % (tr[k0][1], t0, tr[k0][3] * DEG, tr[k1 - 1][3] * DEG, tr[k0][5], tr[k1 - 1][5]))
        print('  u<0 from %s s; swing after that %.2f s; sat flag %.2f s; unwinding blocked by the flag %.4f;'
              ' by-choice withheld %.2f N m s (max %.4f)'
              % ('never' if t_u_neg is None else '%.2f' % t_u_neg, swing, flag, frozen, held_nms, held_max))
        for k in range(k0, k1, int(0.5 / 0.004)):
            sim, mode, stick, meas, u, integ, moment, wind, sp, sn, _ = tr[k]
            uu, ach, ms, mp_, ts_, tp_, held, sat, N = rec[k]
            print('  %5.1f r %6.2f u %+.4f I %+.4f ach %+.4f %-9s %-9s %6.1f %6.1f held %d sat %d/%d N %+6.2f'
                  % (sim - t0, meas * DEG, u, integ, ach, ms, mp_, ts_, tp_, held, sp, sn, N))


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], float(sys.argv[3]))
