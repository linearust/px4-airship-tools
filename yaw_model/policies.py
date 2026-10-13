#!/usr/bin/env python3
"""Step 0c of the D1 plan (d1_universal_architecture_20261001.md, "2) 구조"
가/나/다, "4) 단계" 0c): the forward-flight yaw policies as forward.run_leg
hooks, and the speed selector they read. Nothing here is firmware; every
piece is the plan's text made executable, so that policy_compare.py can fly
it on the yaw model.

  SpeedSelector  the proposed airship_speed_selector (2.가): a pitot (TAS)
                 interval from the sample's dynamic pressure +-dq, a ground
                 interval [max(u_g - W, 0), max(u_g + W, 0)], their
                 intersection when both are usable (an empty one drops the
                 pitot for that cycle), a lost source switched away from at
                 once and taken back only after HOLD_S of continuous use, both
                 bounds through a 1 s first-order filter (PX4's AlphaFilter,
                 alpha = dt / (tau + dt)), and with every source lost the lower
                 bound decaying to 0 while the upper bound is held as long as
                 the loop is closed.
  schedule()     the proposed airship_yaw_schedule (2.다): FF = FF0 +
                 w(u_lo) k u_lo; r_lim = min(MAX, (A - rho) / (FF0 + k u_hi),
                 rho / (k (u_hi - u_lo))), rho = min(AS_YR_INT_LIM, A / 2);
                 the applied cap MAX + w(u_hi) (r_lim - MAX), w the ramp
                 math::interpolate(u, RAMP_LO_0C, RAMP_HI_0C, 0, 1) (0c's ramp; the
                 D1 schedule ramps as the firmware does, firmware_ramp). k <= 0 is
                 "off" and returns exactly (AS_YAWRATE_FF, AS_YAWRATE_MAX).
                 SCHED_STALL adds c u_lo to the min. budget 'supplied' (the
                 SCHED_CB variant, not the plan's text) writes the interval
                 term on the FF the ramp supplies, rho / (k (u_hi - w(u_lo)
                 u_lo)); blend 'lo' blends the cap by w(u_lo) (analysis only)
  tilt_authority()  A of the shared header (2.나): independent pods 0.5 (1 +
                 rho_rev), rho_rev the share of thrust the tilt range can point
                 backward; the Cloudship's tail 1 (reversible: CA_R_REV bit 2).
                 A is per direction (2.나): a plant carries (A+, A-), and a
                 hook whose two differ sets one cap per stick direction.
  fin_slope()    k, the fin yaw damping per m/s per rad/s over the yaw
                 authority (normalized torque per rad/s per m/s), from the
                 engine's measured fin damping (fin_damping.engine_runs, the
                 chain logs' added_mass blocks), the files' sum and the
                 replica's linearisation printed beside it.
  stall_slope()  c of SCHED_STALL: margin x min over the vertical fins of
                 alpha_stall / arm (rad/s per m/s), so that the yaw rate's
                 incidence at a fin's cp, r arm / u, stays under margin x its
                 stall angle at the interval's lowest speed.
  Source         the speed-source cases of 0c: perfect, ground only (W 0
                 indoor; W 2 and 3 with a steady wind at head, tail and 90
                 deg), pitot + ground with a +1 Pa pitot bias under two bands
                 dq (one SDP3x count, 0.05 Pa: a band below the zero error, a
                 misconfiguration; 1.05 Pa, one MS4525DO count and the SDP3x's
                 documented CMODEL 0 offset: a band that covers the bias), a
                 pitot dropout at the stick, a pitot stuck at its entry value.
                 In the model a pitot is only its bias and its band, so an
                 SDP3x set to dq 1.05 Pa is the 1.05 Pa case bit for bit
                 (CHECK_SOURCES flies it to show so).
  Scheduled      the SCHED hook (and SCHED_STALL): source -> selector ->
                 schedule -> controller.set_feedforward / set_rate_cap (the
                 cap both as the stick's full scale and at the loop input).
  leg_kwargs()   run_leg's keyword arguments for a policy on a plant config.

Signs: FRD everywhere a caller reads (forward.run_leg's convention): speeds
m/s, rates deg/s positive nose right, the wind (speed, toward deg) the
direction the air moves toward from the hull's heading at the leg's start,
positive to the right (180: a headwind at the start).
"""
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import closed_loop as cl  # noqa: E402
import controller as ctl  # noqa: E402
import forward as F  # noqa: E402

gz_model = cl.gate.gz_model
AS_SPD_HI_FW = float(ctl.DEFAULT_PARAMS['AS_SPD_HI'])   # m/s, the yaml default of the firmware's ramp end
DT = cl.DT
BENCH = cl.gate.BENCH
SPEED_DOC = os.path.join(BENCH, 'docs', 'd1_speed_source_and_fins_20261001.md')

# The plan's new parameters at the values 0c flies (2.마: names are drafts,
# values are set by measurement). The ramp's ends are 0c's own choice: the
# plan names the ramp and what it must cover (2.다: the sensor floor, about
# 1.5-2 m/s in the speed document, and the gz fins' attachment and stall
# zone, 2-2.5 m/s), not its values. The speed at which the 2 m/s results
# (a full stick stalling the fins inside the ramp) bite follows from them.
RAMP_LO_0C = 1.5                # m/s, w(u) = 0 at and below: 0c's own choice for the plan's schedule()
RAMP_HI_0C = 2.5                # m/s, w(u) = 1 above (0c)
AS_SPD_LO, AS_SPD_HI = RAMP_LO_0C, RAMP_HI_0C   # the names the 0c analysis (policy_compare) wrote the ramp under
# The firmware's ramp since a9e7c3c16b (increment A): from kRampStart x
# AS_SPD_HI to AS_SPD_HI, AS_SPD_HI the plant's own (its airframe chain).
# The increment-1 firmware of 2026-10-07 ramped from AS_SPD_LO = 1.5
# (d1_runs_20261007.jsonl was made on it: pass lo=1.5 to reproduce it).
RAMP_START = 0.5


def _firmware_ramp_start():
    """kRampStart out of airship_yaw_schedule.hpp, so a changed firmware
    ramp is noticed here and not in a comparison that quietly drifted."""
    path = os.path.join(gz_model.ROOT, 'src/modules/airship_att_control/airship_yaw_schedule.hpp')

    with open(path, encoding='utf-8') as handle:
        m = re.search(r'kRampStart = ([0-9.]+)f', handle.read())

    if m is None:
        raise LookupError('%s: kRampStart not found' % path)

    return float(m.group(1))


if _firmware_ramp_start() != RAMP_START:
    raise LookupError('the firmware ramps in from %g x AS_SPD_HI, this model from %g' % (_firmware_ramp_start(), RAMP_START))
SELECTOR_TAU_S = 1.0            # 2.가: both bounds through a 1 s first-order filter
SELECTOR_HOLD_S = 1.0           # 2.가: a lost source is taken back after this long usable (a code constant)
RHO = gz_model.RHO_AIR          # kg/m^3 (worlds/lta.sdf:176), the pitot's q = 1/2 rho TAS^2
STALL_MARGIN = 0.8              # SCHED_STALL keeps r arm / u under this share of alpha_stall
NONE, AIRSPEED, GROUND, TRUE = 'NONE', 'AIRSPEED', 'GROUND', 'TRUE'
# The task's five, then one addition (not in the plan): SCHED+RESTORE with
# the put-back gated on forward flight, w(u_lo) > 0 on the release cycle, so
# that the rule is inert at rest ("남은 결정" 1 asks for a safety net for
# forward flight; RESTORE as options/D has it acts at every release, at rest
# too, which is the 09-28 hover cost)
# A second addition, after the 0c review (finding 1): SCHED_CB, SCHED with
# the interval term written on the FF the ramp supplies (schedule's budget
# 'supplied'). It informs a formula question for the owner; it is not the
# plan's text.
# Since increment A (ec89e6bd84) the restore rule is the firmware's at every
# stick input: a policy without RESTORE models the firmware before it.
POLICIES = ('HEAD', 'RESTORE', 'SCHED', 'SCHED_STALL', 'SCHED+RESTORE', 'SCHED+RESTORE_FWD', 'SCHED_CB', 'D1',
            'D1+RESTORE')
SPEC_POLICIES = POLICIES[:5]
BUDGETS = {'SCHED_CB': 'supplied', 'D1': 'd1', 'D1+RESTORE': 'd1'}   # the policies whose schedule is not 0c's plan
D1_RESERVE = 0.5                # d1: rho = D1_RESERVE min(int_lim, A / 2) (the increment-1 plan's kIntegratorReserve)
RESTORE_EDGE = 'zero_or_sign'   # options/D's default edge (controller.set_restore)


def _speed_doc():
    with open(SPEED_DOC, encoding='utf-8') as handle:
        return handle.read()


def sensor_dq():
    """{'SDP3x': Pa, 'MS4525DO': Pa}: one count of each sensor class, read
    from d1_speed_source_and_fins_20261001.md (its "센서 한 눈금의 크기"
    lines, which cite MS4525DO.cpp:185-197 and SDP3X.hpp:62-64). One count
    is the sensor's resolution, not the tolerance band the plan's dq is
    (2.가 허용폭). Raises when the document no longer says it."""
    text = _speed_doc()
    out = {}

    for name, pattern in (('MS4525DO', r'MS4525DO: ([\d.]+) Pa'), ('SDP3x', r'SDP3x: ([\d.]+) Pa')):
        m = re.search(pattern, text)

        if not m:
            raise LookupError('%s does not give one count of %s' % (SPEED_DOC, name))

        out[name] = float(m.group(1))

    return out


def sdp_cmodel0_offset():
    """The SDP3x's documented zero-flow offset at the default CAL_AIR_CMODEL 0
    [Pa], read from the speed document ("흐름이 0이어도 약 1.05 Pa를
    더합니다", citing airspeed.cpp:92-93, 108, 160-166): the residual zero
    error a band on an uncalibrated SDP3x has to cover. Raises when the
    document no longer says it."""
    m = re.search(r'CAL_AIR_CMODEL 0에서는 흐름이 0이어도 약 ([\d.]+) Pa', _speed_doc())

    if not m:
        raise LookupError('%s does not give the SDP3x CMODEL 0 offset' % SPEED_DOC)

    return float(m.group(1))


DQ = sensor_dq()
SDP_OFFSET = sdp_cmodel0_offset()

# ---------------------------------------------------------------------------
# PX4 helpers, ported
# ---------------------------------------------------------------------------


def interpolate(value, x_low, x_high, y_low, y_high):
    """math::interpolate (src/lib/mathlib/math/Functions.hpp:152-166)."""
    if value <= x_low:
        return y_low

    if value > x_high:
        return y_high

    a = (y_high - y_low) / (x_high - x_low)
    b = y_low - (a * x_low)
    return (a * value) + b


def alpha_of(dt, tau):
    """AlphaFilter::setParametersSeconds (AlphaFilter.hpp:144-153)."""
    den = tau + dt
    return dt / den if den > ctl.FLT_EPSILON else 0.0


# ---------------------------------------------------------------------------
# 2.나: the yaw authority A
# ---------------------------------------------------------------------------


def tilt_authority(tmin_deg, tmax_deg):
    """A of independent pods with the tilt range [tmin, tmax] deg: 0.5 (1 +
    rho_rev), rho_rev = max(0, max over the range of -cos(tilt)) (the pods'
    yaw is half the difference of their forward forces,
    ActuatorEffectivenessAirship.cpp:161, and a pod's thrust is >= 0,
    AirshipPod.cpp:189). -cos is unimodal with its minimum at 0, so its
    maximum on an interval is at an end. +-180 deg: 1; +-90: 0.5."""
    if not (-180.0 <= tmin_deg <= tmax_deg <= 180.0):
        raise ValueError('tilt range %r..%r deg' % (tmin_deg, tmax_deg))

    rev = max(0.0, -math.cos(math.radians(tmin_deg)), -math.cos(math.radians(tmax_deg)))
    return 0.5 * (1.0 + rev)


def tail_authority(airframe):
    """A of a tail thruster: 1 forward; 1 backward only with the tail's
    CA_R_REV bit (motor 3, bit 2: controller.AirshipAllocator.tail_min).
    Returns (A_positive, A_negative)."""
    a = ctl.AIRFRAMES[airframe]
    return 1.0, 1.0 if (int(a['CA_R_REV']) & (1 << 2)) else 0.0


# ---------------------------------------------------------------------------
# k and c from the files
# ---------------------------------------------------------------------------


# How far an engine record's measured rigid yaw inertia may sit from the
# files' before it is taken for another hull's: the Cloudship's record reads
# 1.8 % from its nominal SDF inertia (link rotations, plants.py A7)
ENGINE_INERTIA_TOL = 0.05


def fin_slope(model):
    """k for gz `model`: the vertical fins' yaw damping per unit forward
    speed (N m s/rad per m/s) over the yaw authority (N m per unit of
    normalized torque): normalized torque per rad/s per m/s. None for a
    model with no vertical fin (the schedule is then off, k = 0).

    The damping is the engine's measurement of the rigid _fin member
    (added_mass_check's K_d(fin), the printed blocks fin_damping.engine_runs
    reads from the chain logs; the mean of the runs), checked to be the same
    fins as `model`'s. Beside it, for the record: the files' sum
    1/2 rho sum(area cla arm^2) (gz_model.fin_yaw_damping) and the model's
    replica linearised (fin_damping.linear_O)."""
    import fin_damping as fd
    fins = gz_model.vertical_fins(model)

    if not fins:
        return None

    vehicle = gz_model.vehicle(model)
    member = vehicle + '_fin'
    key = lambda fs: [(f['cp'], f['area'], f['cla'], f['alpha_stall'], f['upward']) for f in fs]

    if key(gz_model.vertical_fins(member)) != key(fins):
        raise ValueError('%s: its vertical fins are not %s\'s, whose engine damping k would be' % (model, member))

    runs = fd.engine_runs(vehicle)
    # a record is of the hull it was measured on: one whose measured rigid yaw
    # inertia is not this tree's is another hull (the generic family moved to
    # the AS800-class hull on 2026-10-07) and is refused, not mixed with these
    # files; added_mass_check.py measures the hull in the tree
    inertia = gz_model.yaw_inertia(gz_model.family(member))
    stale = [r for r in runs if abs(r['I'] - inertia) > ENGINE_INERTIA_TOL * inertia]

    if stale:
        raise LookupError('%s: the engine record (%s) measured a rigid yaw inertia of %s kg m^2 against the files\' %.4g: '
                          'it is of another hull; measure this one with added_mass_check.py'
                          % (vehicle, ', '.join(r['where']['I'] for r in stale), ', '.join('%.4g' % r['I'] for r in stale), inertia))

    authority = gz_model.yaw_authority(model)
    k_d = sum(r['K_d_fin'] for r in runs) / len(runs)
    return {'model': model, 'member': member, 'authority': authority, 'K_d': k_d, 'k': k_d / authority,
            'runs': [(r['where']['K_d_fin'], r['K_d_fin']) for r in runs],
            'K_files': gz_model.fin_yaw_damping(model), 'k_files': gz_model.fin_yaw_damping(model) / authority,
            'K_linear': fd.linear_O(model), 'k_linear': fd.linear_O(model) / authority}


def stall_slope(model, margin=STALL_MARGIN):
    """c for gz `model` [rad/s per m/s]: margin x min over its vertical fins
    of alpha_stall / arm, arm the fin's cp (the quarter chord, where the
    lift-drag plugin reads the incidence) aft of the family's centre of mass
    (gz_model.vertical_fins). A yaw rate r gives the fin a lateral velocity
    r arm, an incidence atan(r arm / u) ~ r arm / u; r <= c u keeps it under
    margin x alpha_stall. None without vertical fins."""
    fins = gz_model.vertical_fins(model)

    if not fins:
        return None

    worst = min(fins, key=lambda f: f['alpha_stall'] / abs(f['arm']))
    return {'c': margin * worst['alpha_stall'] / abs(worst['arm']), 'margin': margin,
            'alpha_stall': worst['alpha_stall'], 'arm': abs(worst['arm']), 'cp': worst['cp'],
            'fins': [(f['cp'], f['alpha_stall'], f['arm']) for f in fins]}


# ---------------------------------------------------------------------------
# 2.다: the schedule
# ---------------------------------------------------------------------------


def schedule(u_lo, u_hi, k, ff0, max_deg, int_lim, A, lo=AS_SPD_LO, hi=AS_SPD_HI, c_stall=None, budget='plan',
             blend='hi'):
    """(FF, cap deg/s, details) for the speed interval [u_lo, u_hi] m/s.

    k <= 0 (the schedule off): exactly (ff0, max_deg). Otherwise
      FF    = ff0 + w(u_lo) k u_lo
      rho   = min(int_lim, A / 2)
      r_lim = min(MAX, (A - rho) / (ff0 + k u_hi), rho / (k (u_hi - u_lo))
                  [, c_stall u_lo]),
              a term whose denominator is 0 left out (a one-point interval)
      cap   = MAX + w(u_hi) (r_lim - MAX)
    w(u) = interpolate(u, lo, hi, 0, 1). r_lim <= MAX always, so cap is
    finite, and with w = 0 it is MAX exactly (bit-identical to no cap).

    The plan's interval term assumes FF = ff0 + k u_lo; the ramp supplies
    ff0 + w(u_lo) k u_lo, so with u_lo inside or below the ramp the shortfall
    at u_hi, k (u_hi - w(u_lo) u_lo), exceeds the one the term budgets for
    (0c review, finding 1). budget 'supplied' (SCHED_CB) writes the term on
    the FF supplied: rho / (k (u_hi - w(u_lo) u_lo)); it equals the plan's
    wherever w(u_lo) = 1, and wherever u_lo = 0. blend 'lo' blends the cap by
    w(u_lo) instead of w(u_hi) (the review's other example; computed for the
    report, not flown). Defaults: the plan's text."""
    if budget not in ('plan', 'supplied', 'd1') or blend not in ('hi', 'lo'):
        raise ValueError('budget %r (plan, supplied, d1), blend %r (hi, lo)' % (budget, blend))

    if not k > 0.0:
        return ff0, max_deg, {'off': True}

    if budget == 'd1':
        return schedule_d1(u_lo, u_hi, k, ff0, max_deg, int_lim, A, lo, hi)     # the caller's ramp: the firmware's for D1

    w_lo = interpolate(u_lo, lo, hi, 0.0, 1.0)
    w_hi = interpolate(u_hi, lo, hi, 0.0, 1.0)
    ff = ff0 + w_lo * k * u_lo
    rho = min(int_lim, 0.5 * A)
    terms = {'max': max_deg}
    den = ff0 + k * u_hi

    if den > 0.0:
        terms['authority'] = math.degrees((A - rho) / den)

    span = k * (u_hi - u_lo) if budget == 'plan' else k * (u_hi - w_lo * u_lo)

    if span > 0.0:
        terms['interval'] = math.degrees(rho / span)

    if c_stall is not None:
        terms['stall'] = math.degrees(c_stall * u_lo)

    binding = min(terms, key=lambda n: terms[n])
    r_lim = terms[binding]
    w_cap = w_hi if blend == 'hi' else w_lo
    cap = max_deg + w_cap * (r_lim - max_deg)
    return ff, cap, {'off': False, 'w_lo': w_lo, 'w_hi': w_hi, 'rho': rho, 'r_lim': r_lim, 'binding': binding,
                     'terms': terms}


def budget_rate(ff0, ff_speed, budget, max_deg):
    """airship_yaw_schedule::budgetRate: the yaw rate (rad/s) whose turn takes
    exactly `budget`, the positive root of ff0 r^2 / r_max + ff_speed r =
    budget; budget / ff_speed without a rest term; infinite without either."""
    if ff0 > 0.0 and max_deg > 0.0:
        a = ff0 / math.radians(max_deg)
        return 2.0 * budget / (ff_speed + math.sqrt(ff_speed * ff_speed + 4.0 * a * budget))

    return budget / ff_speed if ff_speed > 0.0 else float('inf')


def schedule_d1(u_lo, u_hi, k, ff0, max_deg, int_lim, A, lo=None, hi=None):
    """The increment-1 schedule (d1_universal_architecture_20261001.md 2.다 as
    amended 2026-10-07, d1_increment1_plan_20261007.md 1.3), what the firmware
    implements, in the feedforward law controller.FF_LAW names. The ramp is the
    firmware's: hi = AS_SPD_HI (the yaml default when not given), lo =
    RAMP_START x hi (the increment-1 firmware's 1.5 is passed explicitly to
    reproduce its records). (FF, cap deg/s,
    details); FF is what the hook hands controller.set_feedforward: the speed
    term under 'quadratic', the whole gain under 'linear'. k > 0 (the caller
    handles k <= 0). The formulas below are the 'linear' law's; under
    'quadratic' the authority term is the positive root of
    FF0 r^2 / r_max + w(u_lo) k u_lo r = A - rho and the interval term drops FF0.

      FF_s  = ff0 + w(u_lo) k u_lo                 (the FF supplied)
      rho   = D1_RESERVE min(int_lim, A / 2)
      cap   = min(MAX, (A - rho) / FF_s            [FF_s > 0],
                       rho / (D(u_hi) - FF_s)      [D(u_hi) > FF_s])
      D(u)  = ff0 + k u                            (the torque per rad/s a turn needs)

    No blend toward MAX and no stall term (0c findings 3, 5, 6). The interval
    term is the shortfall at the fastest speed in the interval against the FF
    actually supplied, k (u_hi - w(u_lo) u_lo); the plan's text wrote
    k u_hi - FF_s, which drops ff0 and lets the integrator exceed rho by ff0 r
    (fixed here, recorded in the increment-1 report). At u_lo = u_hi = 0:
    FF_s = ff0 and the interval term is absent, so the cap is MAX unless
    (A - rho) / ff0 is below it."""
    hi = AS_SPD_HI_FW if hi is None else hi
    lo = RAMP_START * hi if lo is None else lo
    w_lo = interpolate(u_lo, lo, hi, 0.0, 1.0)
    w_hi = interpolate(u_hi, lo, hi, 0.0, 1.0)
    ff_speed = w_lo * k * u_lo
    rho = D1_RESERVE * min(int_lim, 0.5 * A)
    budget = A - rho
    terms = {'max': max_deg}

    if ctl.FF_LAW == 'quadratic':
        # airship_yaw_schedule since 2026-10-12: the hull's drag term FF0 r^2 / r_max is
        # the setpoint's share of AS_YAWRATE_FF (controller.step applies it), the hook
        # hands the controller the speed term alone, and the budget rule is the positive
        # root of FF0 r^2 / r_max + FF_u r = A - rho; FF0 cancels in the interval term
        ff = ff_speed
        rate = budget_rate(ff0, ff_speed, budget, max_deg)

        if math.isfinite(rate):
            terms['authority'] = math.degrees(rate)

        short = k * u_hi - ff_speed

    else:
        ff = ff0 + ff_speed

        if ff > 0.0:
            terms['authority'] = math.degrees(budget / ff)

        short = (ff0 + k * u_hi) - ff

    if short > 0.0:
        terms['interval'] = math.degrees(rho / short)

    binding = min(terms, key=lambda n: terms[n])
    cap = terms[binding]
    return ff, cap, {'off': False, 'w_lo': w_lo, 'w_hi': w_hi, 'rho': rho, 'r_lim': cap, 'binding': binding,
                     'terms': terms}


def ramp_class(info):
    """Where a schedule() result sits against the ramp, for the budget the
    interval term is meant to keep (0c review, finding 1):
      'off'       the schedule off (k <= 0)
      'w_lo=1'    u_lo past the ramp: the plan's term budgets the shortfall
      'straddle'  u_hi past the ramp, u_lo inside or below it: the plan's
                  term under-budgets (FF is short by (1 - w(u_lo)) k u_lo
                  more than it assumes); budget 'supplied' still holds
      'w_hi<1'    u_hi inside or below the ramp: the cap blends toward MAX
                  and no interval term holds the budget"""
    if info.get('off'):
        return 'off'

    if info['w_lo'] >= 1.0:
        return 'w_lo=1'

    return 'straddle' if info['w_hi'] >= 1.0 else 'w_hi<1'


# ---------------------------------------------------------------------------
# 2.가: the speed selector
# ---------------------------------------------------------------------------


def pitot_interval(tas, dq, rho=RHO):
    """[lo, hi] TAS m/s of a pitot sample: its dynamic pressure 1/2 rho TAS^2
    +- dq, floored at 0 (wider at low speed by itself)."""
    q = 0.5 * rho * tas * tas
    return math.sqrt(2.0 * max(q - dq, 0.0) / rho), math.sqrt(2.0 * (q + dq) / rho)


def ground_interval(ug, w_max):
    """[max(u_g - W, 0), max(u_g + W, 0)] m/s."""
    return max(ug - w_max, 0.0), max(ug + w_max, 0.0)


class SpeedSelector:
    """The proposed airship_speed_selector (2.가), per cycle.

    update(dt, tas, ug, loop_closed) -> (u_lo, u_hi, source): tas the pitot's
    TAS sample [m/s] or None (lost, not fitted, not finite), ug the body-x
    ground speed [m/s] or None. A source is used while its sample is usable
    and it has been usable for SELECTOR_HOLD_S without a break (a source in
    use at the start counts as held: the selector has run since boot, the
    leg starts in steady flight). Both used: the intersection of the two
    intervals; empty, the pitot is dropped for that cycle (rejects counts
    them). The raw bounds go through the filter; with no source, the lower
    bound decays to 0 and the upper bound is held while the loop is closed
    (decays once it opens).

    The filter starts at the first cycle's raw bounds, the converged state
    of a selector that has been running in steady flight (2.가's start value
    0 is the boot state; a leg starts long after it)."""

    def __init__(self, w_max=0.0, dq=0.0, tau=SELECTOR_TAU_S, hold=SELECTOR_HOLD_S, rho=RHO):
        self.w_max, self.dq, self.tau, self.hold, self.rho = float(w_max), float(dq), tau, hold, rho
        self.lo = self.hi = None
        self.run = {'pitot': hold, 'ground': hold}
        self.source = NONE
        self.rejects = 0
        self.switches = 0

    def update(self, dt, tas, ug, loop_closed):
        ok = {'pitot': tas is not None and ctl.isfinite(tas) and tas >= 0.0,
              'ground': ug is not None and ctl.isfinite(ug)}

        for name in ok:
            self.run[name] = (self.run[name] + dt) if ok[name] else 0.0

        use = {name: ok[name] and self.run[name] >= self.hold - 1e-9 for name in ok}
        raw, source = None, NONE

        if use['pitot'] and use['ground']:
            p, g = pitot_interval(tas, self.dq, self.rho), ground_interval(ug, self.w_max)
            lo, hi = max(p[0], g[0]), min(p[1], g[1])

            if lo <= hi:
                raw, source = (lo, hi), AIRSPEED
            else:
                raw, source = g, GROUND
                self.rejects += 1
        elif use['pitot']:
            raw, source = pitot_interval(tas, self.dq, self.rho), AIRSPEED
        elif use['ground']:
            raw, source = ground_interval(ug, self.w_max), GROUND

        if source != self.source and self.lo is not None:
            self.switches += 1

        self.source = source

        if self.lo is None:                     # the first cycle: converged
            self.lo, self.hi = raw if raw is not None else (0.0, 0.0)
            return self.lo, self.hi, source

        a = alpha_of(dt, self.tau)

        if raw is not None:
            self.lo += a * (raw[0] - self.lo)
            self.hi += a * (raw[1] - self.hi)
        else:
            self.lo += a * (0.0 - self.lo)

            if not loop_closed:
                self.hi += a * (0.0 - self.hi)

        return self.lo, self.hi, source


# ---------------------------------------------------------------------------
# The speed-source cases
# ---------------------------------------------------------------------------


class Source:
    """One speed-source case: what the selector is fed per cycle.

      kind 'perfect'  the true air speed itself, no selector (u_lo = u_hi =
                      max(u_air, 0)): the reference
      kind 'ground'   ground speed only (the EKF's body-x, here the
                      PlantView's u_ground: air speed plus the wind), W =
                      AS_SPD_WMAX
      kind 'both'     pitot (TAS) and ground: the pitot reads the true
                      dynamic pressure plus bias_pa; dq = AS_SPD_DP_TOL;
                      'drop' the pitot lost from the stick onset on; 'stuck'
                      the pitot's sample frozen at its first reading (the
                      leg's entry), still reported usable
    wind (speed, toward deg) for run_leg (it moves the ground speed only).
    The pitot reads max(u_air, 0): flow from behind reads as none. note: what
    the case stands for (the report prints it)."""

    def __init__(self, name, kind, w_max=0.0, wind=None, dq=None, bias_pa=0.0, drop=False, stuck=False, note=''):
        self.name, self.kind, self.w_max, self.wind = name, kind, float(w_max), wind
        self.dq, self.bias_pa, self.drop, self.stuck = dq, float(bias_pa), drop, stuck
        self.note = note

    def selector(self):
        return None if self.kind == 'perfect' else SpeedSelector(self.w_max, self.dq or 0.0)

    def calm(self):
        """The same configuration in still air (gate 2's at-rest legs)."""
        return Source(self.name, self.kind, self.w_max, None, self.dq, self.bias_pa, self.drop, self.stuck,
                      self.note)

    def describe(self):
        if self.kind == 'perfect':
            return 'the true air speed, no selector'

        parts = ['ground W %g' % self.w_max] if self.kind in ('ground', 'both') else []

        if self.kind == 'both':
            parts.insert(0, 'pitot dq %g Pa%s%s%s' % (self.dq, ', bias %+g Pa' % self.bias_pa if self.bias_pa else '',
                                                   ', lost from the stick onset' if self.drop else '',
                                                   ', stuck at its entry value' if self.stuck else ''))

        if self.wind:
            parts.append('wind %g m/s toward %g deg' % self.wind)

        return ' + '.join(parts) + (' (%s)' % self.note if self.note else '')

    def reader(self, t_stick):
        """A per-leg reader: read(t, plant) -> (tas or None, ug or None)."""
        frozen = []

        def read(t, plant):
            ug = plant.u_ground if self.kind in ('ground', 'both') else None
            tas = None

            if self.kind == 'both':
                u = max(plant.u_air, 0.0)
                q = 0.5 * RHO * u * u + self.bias_pa
                tas = math.sqrt(max(2.0 * q / RHO, 0.0))

                if self.stuck:
                    if not frozen:
                        frozen.append(tas)
                    tas = frozen[0]

                if self.drop and t >= t_stick - 1e-9:
                    tas = None

            return tas, ug

        return read


def sources():
    """The 0c speed-source cases, by name, in report order. The names are
    the record ids'; what each pitot case stands for is its note (the bias
    against the band dq is what decides an interval, not the sensor class:
    one count is a resolution, the plan's dq is a tolerance)."""
    out = [Source('perfect', 'perfect'), Source('ground_W0', 'ground', 0.0)]

    for w in (2.0, 3.0):
        for label, toward in (('head', 180.0), ('tail', 0.0), ('x90', 90.0)):
            out.append(Source('ground_W%g_%s' % (w, label), 'ground', w, (w, toward)))

    out += [Source('pitot_bias_sdp', 'both', 2.0, None, DQ['SDP3x'], bias_pa=1.0,
                   note='dq below the zero error, a misconfiguration: dq one SDP3x count, the bias 20x outside it'),
            Source('pitot_bias_ms', 'both', 2.0, None, DQ['MS4525DO'], bias_pa=1.0,
                   note='dq covering the bias: one MS4525DO count, and the band an uncalibrated SDP3x needs for its '
                        'documented CMODEL 0 offset of %g Pa' % SDP_OFFSET),
            Source('pitot_dropout', 'both', 2.0, None, DQ['SDP3x'], drop=True),
            Source('pitot_stuck', 'both', 2.0, None, DQ['SDP3x'], stuck=True)]
    return {s.name: s for s in out}


SOURCES = sources()
# After the 0c review (findings 3, 10): an SDP3x whose band is set at its
# documented zero error, against the same +1 Pa bias. The model's pitot is its
# bias and its band only, so this case is pitot_bias_ms exactly; it is flown
# on a subset to show so (policy_compare's dqcheck group), not in the matrix
CHECK_SOURCES = {'pitot_bias_sdp_dq': Source('pitot_bias_sdp_dq', 'both', 2.0, None, SDP_OFFSET, bias_pa=1.0,
                                             note='an SDP3x with dq at its documented CMODEL 0 offset')}


def source(name):
    """A source case by name, the matrix's or the check's."""
    return SOURCES.get(name) or CHECK_SOURCES.get(name)

# ---------------------------------------------------------------------------
# The policies as run_leg hooks
# ---------------------------------------------------------------------------


def firmware_ramp(gains):
    """(lo, hi) m/s of the firmware's speed-share ramp for a plant's gains:
    from RAMP_START x AS_SPD_HI to AS_SPD_HI (airship_yaw_schedule.hpp)."""
    return RAMP_START * gains['AS_SPD_HI'], gains['AS_SPD_HI']


class Scheduled:
    """The SCHED hook (SCHED_STALL with c_stall, SCHED_CB with budget
    'supplied'): every cycle, the speed interval (the source through the
    selector, or the true air speed), the schedule, and the controller's FF
    and cap (controller.set_feedforward, set_rate_cap(cap, stick=True,
    clamp=True): the stick's full scale and the clamp at the loop input).
    Keeps what it decided: at the marks (name -> t, the cycle's own time) and
    every `every` cycles.

    A: the authority, one number for both directions or (A+, A-) per stick
    direction (2.나). Where the two differ, the hook schedules each and sets
    one cap per direction (set_rate_cap's cap_neg_deg_s); the cap it keeps
    is the positive direction's. Where they are equal it is the one-number
    call, bit for bit."""

    def __init__(self, source, k, ff0, max_deg, int_lim, A, c_stall=None, t_stick=0.0, marks=None, every=25,
                 forward_gate=False, budget='plan', ramp=None):
        """forward_gate: the restore rule's put-back happens only on a
        cycle with w(u_lo) > 0 (SCHED+RESTORE_FWD); the save at the stick's
        onset is untouched. Installed on the controller at the first call
        (its _restore_edge, which only the put-back reads).
        ramp: (lo, hi) m/s of w(u); None is 0c's (RAMP_LO_0C, RAMP_HI_0C). A
        D1 hook (budget 'd1') takes the firmware's, firmware_ramp(gains)."""
        self.ramp_lo, self.ramp_hi = (RAMP_LO_0C, RAMP_HI_0C) if ramp is None else ramp
        a_pos, a_neg = (A, A) if isinstance(A, (int, float)) else (float(A[0]), float(A[1]))
        self.source, self.k, self.ff0, self.max_deg, self.int_lim = source, k, ff0, max_deg, int_lim
        self.A, self.A_neg = a_pos, (None if a_neg == a_pos else a_neg)
        self.c_stall = c_stall
        self.budget = budget
        self.sel = source.selector()
        self.read = source.reader(t_stick)
        self.t_stick = t_stick
        self.marks = dict(marks or {})
        self.at = {}
        self.every = every
        self.samples = []
        self.n = 0
        self.stick_caps = []
        self.binding = {}
        self.ramp = {}
        self.sources_seen = {}
        self.forward_gate = forward_gate
        self.forward = False
        self.gated_edges = 0
        self._installed = False

    def _install(self, ctrl):
        edge = ctrl._restore_edge

        def gated(sp):
            fired = edge(sp)

            if fired and not self.forward:
                self.gated_edges += 1

            return fired and self.forward

        ctrl._restore_edge = gated
        self._installed = True

    def __call__(self, t, plant, ctrl, alloc):
        tas, ug = self.read(t, plant)

        if self.sel is None:
            lo = hi = max(plant.u_air, 0.0)
            src = TRUE
        else:
            lo, hi, src = self.sel.update(DT, tas, ug, ctrl.loop_active)

        ff, cap, info = schedule(lo, hi, self.k, self.ff0, self.max_deg, self.int_lim, self.A, lo=self.ramp_lo,
                                 hi=self.ramp_hi, c_stall=self.c_stall, budget=self.budget)
        ctrl.set_feedforward(ff)

        if self.A_neg is None:
            ctrl.set_rate_cap(cap, stick=True, clamp=True)
        else:
            _, cap_neg, _ = schedule(lo, hi, self.k, self.ff0, self.max_deg, self.int_lim, self.A_neg,
                                     lo=self.ramp_lo, hi=self.ramp_hi, c_stall=self.c_stall, budget=self.budget)
            ctrl.set_rate_cap(cap, stick=True, clamp=True, cap_neg_deg_s=cap_neg)

        if self.forward_gate:
            if not self._installed:
                self._install(ctrl)

            self.forward = self.k > 0.0 and interpolate(lo, self.ramp_lo, self.ramp_hi, 0.0, 1.0) > 0.0

        row = (round(t, 6), lo, hi, src, ff, cap, info.get('binding'), tas, ug)

        for name, when in self.marks.items():
            if abs(t - when) < 1e-9:
                self.at[name] = row

        if t >= self.t_stick - 1e-9 and 'release' in self.marks and t < self.marks['release'] - 1e-9:
            self.stick_caps.append(cap)
            self.binding[info.get('binding')] = self.binding.get(info.get('binding'), 0) + 1
            ramp = ramp_class(info)
            self.ramp[ramp] = self.ramp.get(ramp, 0) + 1

        self.sources_seen[src] = self.sources_seen.get(src, 0) + 1

        if self.n % self.every == 0:
            self.samples.append(row)

        self.n += 1

    def summary(self):
        sel = self.sel
        return {'at': self.at, 'cap_min_stick': min(self.stick_caps) if self.stick_caps else None,
                'cap_max_stick': max(self.stick_caps) if self.stick_caps else None,
                'binding_stick': dict(self.binding), 'ramp_stick': dict(self.ramp),
                'sources_seen': dict(self.sources_seen),
                'rejects': sel.rejects if sel else 0, 'switches': sel.switches if sel else 0,
                'gated_edges': self.gated_edges}


class PlantConfig:
    """One plant of the comparison: a forward target, its tilt range
    override (vectored only; None = the airframe's), A per direction, gated
    or not.

    A_dir = (A+, A-), the authority toward a positive (nose right) and a
    negative stick (2.나: per direction). A is A+, the direction every 0c
    stick flies (each stick here is positive); a hook gets A_dir, so a plant
    whose two differ (a tail without its CA_R_REV bit: A- 0) keeps the
    positive stick's cap instead of zeroing both."""

    def __init__(self, name, target, tilt_range=None, gated=True, settles=True, k_override=None, label=None):
        self.name, self.target, self.tilt_range, self.gated, self.settles = name, target, tilt_range, gated, settles
        self.model, self.airframe = cl.ALL_TARGETS[target]
        self.vectored = self.airframe == 'vectored'
        self.k_override = k_override
        self.label = label or name

        if self.vectored:
            if tilt_range is None:
                a = ctl.AIRFRAMES['vectored']
                a_pos = tilt_authority(a['CA_AIRSHIP_TLMIN'], a['CA_AIRSHIP_TLMAX'])
            else:
                a_pos = tilt_authority(*tilt_range)

            self.A_dir = (a_pos, a_pos)
        else:
            if tilt_range is not None:
                raise ValueError('the Cloudship\'s collective tilt carries no yaw: no tilt range override')

            self.A_dir = tail_authority(self.airframe)

        self.A = self.A_dir[0]

        self.gains = ctl.head_params(self.model)
        self.fins = fin_slope(self.model)
        self.stall = stall_slope(self.model)

    @property
    def k(self):
        if self.k_override is not None:
            return self.k_override

        return self.fins['k'] if self.fins else 0.0


def make_hook(policy, cfg, source, k_factor=1.0, t_stick=3.0, t_release=23.0):
    """(hook or None, restore) of `policy` on `cfg` with `source`, the
    schedule's k times k_factor (both in FF and in the cap: an FF error is a
    wrong k)."""
    if policy not in POLICIES:
        raise ValueError('policy %r: one of %s' % (policy, POLICIES))

    restore = RESTORE_EDGE if 'RESTORE' in policy else None

    if not policy.startswith(('SCHED', 'D1')):
        return None, restore

    g = cfg.gains
    c = cfg.stall['c'] if (policy == 'SCHED_STALL' and cfg.stall) else None
    budget = BUDGETS.get(policy, 'plan')
    hook = Scheduled(source, cfg.k * k_factor, g['AS_YAWRATE_FF'], g['AS_YAWRATE_MAX'], g['AS_YR_INT_LIM'],
                     cfg.A_dir, c_stall=c, t_stick=t_stick,
                     marks={'stick': t_stick, 'stick_end': round(t_release - DT, 6), 'release': t_release},
                     forward_gate=policy.endswith('_FWD'), budget=budget,
                     ramp=firmware_ramp(g) if budget == 'd1' else None)
    return hook, restore


def leg_kwargs(policy, cfg, source, u0, stick, k_factor=1.0, centred_s=3.0, stick_s=20.0, release_s=120.0, **extra):
    """(run_leg kwargs, hook) for `policy` on `cfg` at u0 with the yaw stick
    held stick_s s: the plant's HEAD gains, the throttle at trim as the 0b
    check flies it (forward.run_leg's defaults), the source's wind (it moves
    the ground speed only, so HEAD and RESTORE take none), the tilt range."""
    t_stick = round(round(centred_s / DT) * DT, 6)
    t_release = round(round((centred_s + stick_s) / DT) * DT, 6)
    hook, restore = make_hook(policy, cfg, source, k_factor, t_stick, t_release)
    kw = dict(target=cfg.target, u0=u0, stick=stick, centred_s=centred_s, stick_s=stick_s, release_s=release_s,
              hook=hook, restore=restore, tilt_range=cfg.tilt_range,
              wind=source.wind if (hook is not None and source is not None) else None)
    kw.update(extra)
    return kw, hook


def plant_configs():
    """The 0c plants, in report order: 2527 and 2522 at the airframe's tilt
    range (+-180, A 1) and limited to +-90 (A 0.5), the rigid Cloudship with
    fins (2512) and the complete one (2517, reported, not gated: unstable in
    forward flight, step0a's stability rows)."""
    v27, v22 = 'gz_airship_vectored_independent_am_fin_drag', 'gz_airship_vectored_independent_fin'
    return [PlantConfig('2527', v27), PlantConfig('2522', v22),
            PlantConfig('2527/A0.5', v27, tilt_range=(-90.0, 90.0)),
            PlantConfig('2522/A0.5', v22, tilt_range=(-90.0, 90.0)),
            PlantConfig('2512', 'gz_cloudship_fin'),
            PlantConfig('2517', 'gz_cloudship_am_fin_drag', gated=False, settles=False)]
