#!/usr/bin/env python3
"""The one lift basis every fin in the airship family is sized on.

Written once so that the design script, the model writer and variant_check.py
(which holds the files to it) cannot drift apart. Every function is a stated
estimate, not a measurement.

- wing_slope(ar): the wing-alone lift-curve slope the plugins carry, on the two
  panels joined through the hull (AR_j = 2 b / c for rectangular panels): the
  2 pi AR / (AR + 2) that Ananda, Sukumar and Selig (2015) measured for flat
  plates of AR 2-3 at Re 6e4-1.6e5, a linear fit to about 10 deg. Its excess
  over lifting-surface theory is side-edge vortex lift, which is quadratic in
  incidence and adds nothing at zero incidence, so it is a secant, right for
  the forces of a manoeuvre and too stiff for small-perturbation stability.
- helmbold(ar): 2 pi AR / (2 + sqrt(4 + AR^2)), the attached-flow slope at
  zero incidence. Static stability is judged on this one.
- carryover(r, s): K_W(B) + K_B(W) = (1 + r/s)^2, the slender-body lift of the
  wing-body combination over the wing alone (NACA Report 1307, eqs. 14 and 21,
  the values it recommends for all combinations), r the hull radius at the fin
  station, s the semispan from the hull axis.
- k_wb(r, s): K_W(B) alone, 1307's slender-body closed form: the panel's own
  lift in the body's upwash. For a flap deflection on a body at zero incidence
  1307's factors are k_W(B) + k_B(W), whose slender-body sum is K_W(B).
- stall(ar, k_wb, clmax): the incidence where the linear range ends. The
  upwash that makes K_W(B) raises the panel's own incidence, so the panel
  reaches the plates' CLmax at CLmax / (wing slope x K_W(B)) of body angle
  (the equivalent-angle-of-attack method, Hemsch and Nielsen 1983). CLmax is
  CLMAX_PLATE, the upper end of the CLMAX_PLATE_LOW-CLMAX_PLATE the same data
  measured for AR 2-5, unless given: the Cloudship's AR_j 1.68 plates take
  CLMAX_LOW_AR_PLATE (Torres and Mueller's low-AR plates top out near cl
  1.2-1.3 on their own area).
- post_stall(cla, cda, alpha_s): the plugin's post-stall slopes, chosen so cl
  falls to 0 and cd rises to a broadside plate's CD_PLATE_90 at 90 deg.
- flap_slope(..., delta_max): d(cl)/d(delta) on the exposed area = tau x
  VISCOUS x wing slope x K_W(B), tau the thin-airfoil flap effectiveness
  1 - (theta - sin theta) / pi with cos theta = 2 cf/c - 1, VISCOUS the ~0.8
  that Caughey's "nearly 50 percent ... for a 25 percent chord flap" implies
  against the theory's 0.61. Linear inside 10-15 deg; with delta_max it is
  capped so full deflection alone stays inside CLMAX_PLATE x K_W(B).
- induced_cda(cla, ar, alpha_s): the plugin's linear-in-alpha drag slope,
  sized to CL^2 / (pi AR e) with e = 0.9 at the lower of 10 deg and the stall
  alpha_s: past the stall the plugin uses cda_stall, so a reference angle
  beyond it would size cda on a lift the fin never reaches.
- plugin_cl(alpha, f), plugin_cd(alpha, f): what the gz lift-drag plugin
  applies for one fin record (cla, cda, alpha_stall, cla_stall, cda_stall) at
  the geometric incidence alpha, stall included; a flap adds to cl after.
- panel(size, y, z, z_axis): a fin box's chord, span and root radius, as the
  model files draw it.
"""
import math

VISCOUS = 0.8
DRAG_ALPHA = math.radians(10.0)
OSWALD = 0.9
CLMAX_PLATE = 0.7
CLMAX_PLATE_LOW = 0.55       # the bottom of the range the AR 2-5 plates measured
CLMAX_LOW_AR_PLATE = 1.25    # Torres and Mueller's low-AR plates, near 1.2-1.3; the Cloudship's fins
CD_PLATE_90 = 1.2


def wing_slope(ar):
    """The flat-plate data's slope at Re 1e5, a secant to ~10 deg."""
    return 2.0 * math.pi * ar / (ar + 2.0)


def helmbold(ar):
    """The attached-flow slope at zero incidence, which stability is judged on."""
    return 2.0 * math.pi * ar / (2.0 + math.sqrt(4.0 + ar * ar))


def carryover(r, s):
    return (1.0 + r / s) ** 2


def k_wb(r, s):
    x = r / s
    a = (1.0 + x ** 4) * (0.5 * math.atan(0.5 * (1.0 / x - x)) + math.pi / 4.0)
    b = x * x * ((1.0 / x - x) + 2.0 * math.atan(x))
    return (2.0 / math.pi) * (a - b) / (1.0 - x) ** 2


def flap_tau(cf_over_c):
    theta = math.acos(2.0 * cf_over_c - 1.0)
    return 1.0 - (theta - math.sin(theta)) / math.pi


def joined_ar(chord, span):
    return 2.0 * span / chord


def cla(chord, span, r, slope=wing_slope):
    """Lift slope per rad on the exposed area of one rectangular panel."""
    return slope(joined_ar(chord, span)) * carryover(r, r + span)


def flap_slope(chord, span, r, cf_over_c, delta_max=None):
    linear = flap_tau(cf_over_c) * VISCOUS * wing_slope(joined_ar(chord, span)) * k_wb(r, r + span)

    if delta_max is None:
        return linear

    return min(linear, CLMAX_PLATE * k_wb(r, r + span) / delta_max)


def stall(ar, k_wb_value, clmax=None):
    return (CLMAX_PLATE if clmax is None else clmax) / (wing_slope(ar) * k_wb_value)


def post_stall(cla_value, cda_value, alpha_s):
    """(cla_stall, cda_stall): cl to 0 and cd to CD_PLATE_90 at 90 deg."""
    rest = math.pi / 2.0 - alpha_s
    return -cla_value * alpha_s / rest, (CD_PLATE_90 - cda_value * alpha_s) / rest


def induced_cda(cla_value, ar, alpha_s):
    ref = min(DRAG_ALPHA, alpha_s)
    cl = cla_value * ref
    return cl * cl / (math.pi * ar * OSWALD) / ref


def hull_radius(a, b, x):
    """Radius of the a x b x b ellipsoid at station x from its centre."""
    return b * math.sqrt(max(0.0, 1.0 - (x / a) ** 2))


def panel(size, y, z, z_axis):
    """(chord, span, root) of a fin drawn as a box of size (x, y, z) centred at
    (y, z) beside a hull whose axis is at height z_axis: the chord along x, the
    span the box's other long side, the root the radius of the box's centre
    from the axis less half the span. The thin side is the lift normal."""
    chord = size[0]
    thin = size.index(min(size))
    span = [s for i, s in enumerate(size) if i not in (0, thin)][0]
    return chord, span, math.hypot(y, z - z_axis) - span / 2


def plugin_cl(alpha, f):
    x = abs(alpha)
    if x <= f['alpha_stall']:
        return f['cla'] * alpha
    return math.copysign(max(0.0, f['cla'] * f['alpha_stall'] + f['cla_stall'] * (x - f['alpha_stall'])), alpha)


def plugin_cd(alpha, f):
    x = abs(alpha)
    if x <= f['alpha_stall']:
        return abs(f['cda'] * x)
    return abs(f['cda'] * f['alpha_stall'] + f['cda_stall'] * (x - f['alpha_stall']))


if __name__ == '__main__':
    # The design that uses this basis lives in generic_hull_design.py; this
    # prints the basis itself for one fin, for reading.
    for ar in (1.5, 2.0, 2.5, 3.0):
        print('AR_j %.1f: plate slope %.3f, Helmbold %.3f, tau(0.30) %.3f' % (ar, wing_slope(ar), helmbold(ar), flap_tau(0.30)))
