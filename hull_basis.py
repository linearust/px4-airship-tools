#!/usr/bin/env python3
"""The hull half of the basis the airship family is sized on: the prolate
spheroid's potential-flow added mass, written once for the design script and
variant_check.py.

- lamb(a, b): Lamb's k1 (axial), k2 (transverse) and k' (rotational)
  coefficients for a prolate spheroid of semi-axes a > b.
- added_mass_block(a, b, h, displaced): the eight <fluid_added_mass> elements
  of that spheroid displacing `displaced` kg of fluid, written about a link
  origin h below the hull axis: T = 1/2 v_P^T A v_P with v_P = v_O + w x d
  gives xq = h m_x, yp = -h m_y, pp = h^2 m_y, qq = I_q + h^2 m_x.
"""
import math


def lamb(a, b):
    e = math.sqrt(1.0 - (b / a) ** 2)
    L = math.log((1.0 + e) / (1.0 - e))
    alpha0 = 2.0 * (1.0 - e * e) / e ** 3 * (0.5 * L - e)
    beta0 = 1.0 / (e * e) - (1.0 - e * e) / (2.0 * e ** 3) * L
    k1 = alpha0 / (2.0 - alpha0)
    k2 = beta0 / (2.0 - beta0)
    kp = e ** 4 * (beta0 - alpha0) / ((2.0 - e * e) * (2.0 * e * e - (2.0 - e * e) * (beta0 - alpha0)))
    return k1, k2, kp


def added_mass_block(a, b, h, displaced):
    k1, k2, kp = lamb(a, b)
    mx, my = k1 * displaced, k2 * displaced
    iq = kp * displaced * (a * a + b * b) / 5.0
    return {'xx': mx, 'yy': my, 'zz': my, 'xq': h * mx, 'yp': -h * my, 'pp': h * h * my, 'qq': iq + h * h * mx, 'rr': iq}
