#!/usr/bin/env python3
"""Reviewer's check: run added_mass_check.py's yaw-torque and push probes on
planar.RollPlanarPlant (the default plant of closed_loop.build), and fit the
result exactly as the check fits the gz record (r against the integral of u v,
K = slope * I_am). The gz engine read K -4.5278 / -4.5344 on cloudship_am
(chain_20260924.txt:21, chain5_20260925.txt:36) against -(m_y - m_x) -4.9228,
and -38.45 on airship_vectored_independent_am (chain5:24, chain9:72) against
-38.03. If the model plant is the engine's physics, the same probe on it must
read the engine's numbers, not the block's.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.dont_write_bytecode = True

import planar  # noqa: E402
import gz_model  # noqa: E402

DT = 0.004


def run(model, force_world=None, torque_z=0.0, seconds=2.0, couple_to_com=True, pitch_note=False):
    p = planar.RollPlanarPlant('probe', model=model)
    s, phi, psi = [0.0, 0.0, 0.0, 0.0], 0.0, 0.0
    rows = []
    t = 0.0
    xg, _, zg = p.rg

    for _ in range(int(round(seconds / DT))):
        ext = [0.0] * 6
        if force_world:
            c, sn = math.cos(psi), math.sin(psi)
            fx = force_world[0] * c + force_world[1] * sn
            fy = -force_world[0] * sn + force_world[1] * c
            # a force through the centre of mass: F at O plus r_g x F
            ext[0] += fx
            ext[1] += fy
            if couple_to_com:
                ext[3] += -zg * fy          # r_g x F, x
                ext[5] += xg * fy           # r_g x F, z  (y_g = 0)
        ext[5] += torque_z
        k1, d1 = p._deriv(s, phi, ext)
        s2 = [s[i] + 0.5 * DT * k1[i] for i in range(4)]
        k2, d2 = p._deriv(s2, phi + 0.5 * DT * d1, ext)
        s3 = [s[i] + 0.5 * DT * k2[i] for i in range(4)]
        k3, d3 = p._deriv(s3, phi + 0.5 * DT * d2, ext)
        s4 = [s[i] + DT * k3[i] for i in range(4)]
        k4, d4 = p._deriv(s4, phi + DT * d3, ext)
        s = [s[i] + DT * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6.0 for i in range(4)]
        phi += DT * (d1 + 2 * d2 + 2 * d3 + d4) / 6.0
        psi += DT * s[3] / math.cos(phi)
        t += DT
        rows.append((t, s[0], s[1], s[3], psi, phi))

    return rows, p


def inertia(rows, torque):
    # least-squares parabola psi = c0 + c1 t + c2 t^2
    ts = [r[0] for r in rows]
    ps = [r[4] for r in rows]
    S = [sum(t ** k for t in ts) for k in range(5)]
    Sy = [sum(p * t ** k for p, t in zip(ps, ts)) for k in range(3)]
    A = [[S[0], S[1], S[2]], [S[1], S[2], S[3]], [S[2], S[3], S[4]]]
    c = planar._solve(A, Sy)
    return torque / (2 * c[2])


def munk_fit(rows):
    X, R, acc = [], [], 0.0
    for i, (t, u, v, r, _, _) in enumerate(rows):
        if i:
            t0, u0, v0 = rows[i - 1][0], rows[i - 1][1], rows[i - 1][2]
            acc += 0.5 * (u * v + u0 * v0) * (t - t0)
        X.append(acc)
        R.append(r)
    n = len(X)
    mx, my = sum(X) / n, sum(R) / n
    return sum((x - mx) * (y - my) for x, y in zip(X, R)) / sum((x - mx) ** 2 for x in X)


if __name__ == '__main__':
    for vehicle in ('cloudship', 'airship_vectored_independent'):
        fam = gz_model.family(vehicle)
        mass, com = gz_model.centre_of_mass(fam)
        izz = gz_model.yaw_inertia(fam)
        yaw_torque = 1.0 / 5.7986 * izz
        push = (6.0 / 7.3148 * mass, 0.5 / 7.3148 * mass, 0.0)
        I = {}
        for m in (vehicle, vehicle + '_am'):
            rows, _ = run(m, torque_z=yaw_torque, seconds=2.0)
            I[m] = inertia(rows, yaw_torque)
        am = gz_model.added_mass(gz_model.merged(vehicle + '_am'))
        for m in (vehicle, vehicle + '_am'):
            rows, p = run(m, force_world=push, seconds=2.5)
            k = munk_fit(rows) * I[m]
            t, u, v, r, psi, phi = rows[-1]
            print('%-32s I %.4f  push end u %.3f v %.3f r %.4f phi %.2f deg  K %.4f  (block -(m_y-m_x) %.4f)'
                  % (m, I[m], u, v, r, math.degrees(phi), k, -(am['yy'] - am['xx']) if m.endswith('_am') else 0.0))
