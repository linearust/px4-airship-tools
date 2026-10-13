#!/usr/bin/env python3
"""The yaw loop's values for an airship family, from its model files.

    python3 airship_gains.py airship_indoor            # print the table
    python3 airship_gains.py airship_indoor --write    # and write them into the family's posix airframes
    python3 airship_gains.py airship_indoor --params vehicles/airship_indoor.params   # the FMU's set
    python3 airship_gains.py airship_indoor --measure sitl_out/forward_airship_indoor_*_summary.json --write
                                                       # the held-turn slope from those runs, kept in
                                                       # vehicles/airship_indoor.measured.json for later runs

The anchor is the 10.97 m generic airship the tree carried until
2026-10-07, whose values were tuned and measured in SITL and at the pins
(yaw_retune_20260927.md, D1 increment 1): P 0.7 and I 0.09 on a 54 N m,
129 + 192 kg m^2 complete plant. That hull is no longer in the tree, so its
numbers are REF_PLANT below, read from its files before it was replaced.
A vehicle gets the gains that give its complete plant the same loop
bandwidth N P / I and the same integral time P / I, so the loop behaves
alike on both and the gate verdicts transfer. Every other input is read
from the vehicle's model files through gz_model.

  P            = w_ref I_complete / N,  w_ref = N_ref P_ref / I_ref  [rad/s]
  I            = P / T_ref,  T_ref = P_ref / I_ref                   [s]
  AS_YR_INT_LIM, AS_YAWRATE_MAX, MAN_DEADZONE: the reference's
  AS_YAWRATE_FF  the torque that holds AS_YAWRATE_MAX in a turn in place over
               N, per rad/s: the hull's rotational drag |nRabsR| r on a _drag member,
               and on a member with fins but no drag the reference's fin value scaled
               by the fins' damping over the authority (the plugin gives the fins
               nothing in a pure rotation; what holds that turn is the drift the hull
               takes). The firmware applies it scaled by the setpoint's share of
               AS_YAWRATE_MAX (airship_yaw_schedule::feedforwardGain, 2026-10-12),
               since the drag grows with the square of the rate: the value holds
               every rate, not only the maximum
  AS_YR_FF_SLP   k / N: the torque that holds a turn, per m/s of forward speed and rad/s of yaw rate, over
               the authority. Seeded from the files and replaced by a held-turn measurement (--measure,
               forward_yaw_check summaries; the hardware procedure in the docs is the same measurement).
               The seed is not the fins' rotational damping (gz_model.fin_yaw_damping, 1/2 rho sum(area
               cla arm^2)): a steady turn sideslips until the fins carry the centripetal force (m + m_x) u r,
               so their moment is (m + m_x) u r x_bar whatever their lift slope, and the hull's Munk moment
               (m_y - m_x) u v on that sideslip takes some of it back:
                 k = (m + m_x) x_bar - (m_y - m_x) s,   s = 2 (m + m_x) / (rho S_a) - x_bar
               with S_a = sum(area cla) of the vertical fins, x_bar their lift-weighted arm and s the
               sideslip per unit r / u. Measured in SITL against this seed: the generic fin plant 212
               N m s/rad per m/s vs 210, its complete plant 235 vs 229, the indoor fin plant 0.652 vs
               0.645, its complete plant 0.583 vs 0.566 (the rotational damping would say 230 and 0.511).
               A seed at or below zero means the Munk moment outruns the fins in a turn (the Cloudship,
               -4.7): no schedule can be seeded, the fins are too small for a turn
  AS_SPD_HI      the ramp's end is where the cap the schedule allows past it, the yaw rate r whose turn
               takes the authority less the integrator's reserve, FF0 r^2 / r_max + (k u / N) r = A - rho
               (airship_yaw_schedule::rateCap), meets STALL_MARGIN of the yaw rate at which a fin stalls in
               a turn, alpha_stall u / x_eff with x_eff = s + arm the fin's incidence per r / u (its sideslip
               plus its own rotation; the measured incidences sit 0 to 16 % above this, D4); the firmware
               starts the ramp at half that. Below the ramp the reserve term keeps the cap well under the stall
  AS_SPD_SRC 2, AS_SPD_WMAX 0 on the fin members, as the generic family's

The written block sits between two markers in each airframe file, so a rerun
replaces it; the bare airframe gets P and I, the _fin and the complete member
their feedforward and schedule, as the generic family's 2520, 2522 and 2527.
"""
import argparse
import math
import os
import re
import sys
import textwrap

import gz_model

# The anchor plant: the 39.2 m^3, 10.97 m generic complete plant (posix 2527
# until 2026-10-07), read from its files with plant() before the generic
# family moved to the AS800-class hull. N the pods' yaw authority, I_rigid
# and rr its rigid and added yaw inertia, k the held-turn slope seed
REF_PLANT = {'vehicle': 'airship_vectored_independent (10.97 m, until 2026-10-07)', 'N': 54.0, 'I_rigid': 129.150344,
             'rr': 192.24415, 'k': 228.708906, 'length': 10.9714}
REF_GAINS = {'AS_YAWRATE_P': 0.7, 'AS_YAWRATE_I': 0.09, 'AS_YR_INT_LIM': 0.3, 'AS_YAWRATE_MAX': 15.0, 'MAN_DEADZONE': 0.1}
REF_FF_FIN = 0.3            # the generic _fin member's AS_YAWRATE_FF (2522)
STALL_MARGIN = 0.8          # the cap at the ramp's end, as a share of the stalling yaw rate there
RESERVE = 0.5               # airship_yaw_schedule::kIntegratorReserve, the share of the integrator left to the loop
HERE = os.path.dirname(os.path.abspath(__file__))
MEASURED_TOL = 0.01         # a stored measurement is of this plant if its mass, fin arm and authority are within this
# The one-line source note the yaw loop block starts with; the block runs to
# the end of the file, so the generator rewrites from here (no closing marker:
# upstream airframes carry no generated-block markers)
MARK_OPEN = '# Yaw loop values from airship_gains.py (px4-airship-tools) for this hull'


def plant(vehicle):
    """The yaw numbers of a vehicle's complete member and its fin member."""
    complete, fin = vehicle + '_am_fin_drag', vehicle + '_fin'
    fam = gz_model.family(complete)
    hydro = gz_model.hydrodynamics(gz_model.merged(complete))
    mass, _ = gz_model.centre_of_mass(fam)
    am = gz_model.added_mass_terms(complete, ('xx', 'yy'))
    fins = gz_model.vertical_fins(fin)
    s_a = sum(f['area'] * f['cla'] for f in fins)
    x_bar = sum(f['area'] * f['cla'] * f['arm'] for f in fins) / s_a
    slip = 2.0 * (mass + am['xx']) / (gz_model.RHO_AIR * s_a) - x_bar
    return {'vehicle': vehicle, 'N': gz_model.yaw_authority(complete), 'I_rigid': gz_model.yaw_inertia(fam),
            'rr': gz_model.added_mass(gz_model.merged(complete)).get('rr', 0.0), 'nRabsR': abs(hydro.get('nRabsR', 0.0)),
            'mass': mass, 'm_x': am['xx'], 'm_y': am['yy'], 'x_bar': x_bar, 'slip': slip,
            'k': (mass + am['xx']) * x_bar - (am['yy'] - am['xx']) * slip, 'k_rot': gz_model.fin_yaw_damping(fin),
            'length': gz_model.hull_length(complete),
            # the yaw rate per m/s at which a fin stalls in a turn: alpha_stall / (its sideslip + its own rotation)
            'stall_per_speed': min(f['alpha_stall'] / (slip + f['arm']) for f in fins)}


def reserve():
    """rho: the share of the full authority the cap leaves to the integrator."""
    return RESERVE * min(REF_GAINS['AS_YR_INT_LIM'], 0.5)


def budget_rate(ff0, ff_speed):
    """airship_yaw_schedule::budgetRate at the full authority less the
    integrator's reserve: the yaw rate whose turn, ff0 r^2 / r_max for the
    hull's drag plus ff_speed r for the fins, takes exactly that budget, rad/s
    (the cap past the ramp)."""
    budget = 1.0 - reserve()
    a = ff0 / math.radians(REF_GAINS['AS_YAWRATE_MAX'])
    return 2.0 * budget / (ff_speed + math.sqrt(ff_speed * ff_speed + 4.0 * a * budget)) if a > 0.0 else budget / ff_speed


def stall_ramp(p, ff0, slope):
    """AS_SPD_HI: where the cap past the ramp, budget_rate(ff0, slope u), equals
    STALL_MARGIN times the stalling yaw rate alpha_stall u / x_fin = g u (the
    firmware starts the ramp at half of it). With r = g u the budget rule reads
    (ff0 g^2 / r_max + slope g) u^2 = 1 - rho."""
    g = STALL_MARGIN * p['stall_per_speed']
    return math.sqrt((1.0 - reserve()) / (g * (ff0 * g / math.radians(REF_GAINS['AS_YAWRATE_MAX']) + slope)))


def gains(p, ref, measured_slope=None):
    w_ref = ref['N'] * REF_GAINS['AS_YAWRATE_P'] / (ref['I_rigid'] + ref['rr'])
    t_ref = REF_GAINS['AS_YAWRATE_P'] / REF_GAINS['AS_YAWRATE_I']
    P = w_ref * (p['I_rigid'] + p['rr']) / p['N']
    I = P / t_ref
    r_max = math.radians(REF_GAINS['AS_YAWRATE_MAX'])
    scale = p['length'] / ref['length']
    if measured_slope is None and p['k'] <= 0.0:
        raise SystemExit('%s: the seed k = (m + m_x) x_bar - (m_y - m_x) s is %.3g N m s/rad per m/s: the hull\'s Munk moment '
                         'outruns its fins in a turn, no schedule can be seeded (the fins are too small for a turn)' % (p['vehicle'], p['k']))

    slope = (measured_slope if measured_slope is not None else p['k']) / p['N']
    ff_drag = p['nRabsR'] * r_max / p['N']
    hi = stall_ramp(p, ff_drag, slope)
    out = dict(REF_GAINS)
    out.update({'AS_YAWRATE_P': P, 'AS_YAWRATE_I': I,
                'FF_drag': ff_drag,
                'FF_fin': REF_FF_FIN * slope / (ref['k'] / ref['N']),
                'AS_YR_FF_SLP': slope, 'slope_source': 'measured' if measured_slope is not None else 'files',
                'AS_SPD_HI': hi,
                'bandwidth': w_ref, 'integral_time': t_ref, 'length_scale': scale,
                # the yaw rate the fins allow at the ramp's end before stalling in a turn: alpha = r arm / u
                'r_stall_at_hi': math.degrees(p['stall_per_speed'] * hi),
                'cap_at_hi': math.degrees(budget_rate(ff_drag, slope * hi))})
    return out


def measure_slope(p, summaries):
    """k from forward_yaw_check summaries: for each leg the torque that held
    the turn over its last 5 s, less the hull's rotational drag, per u r;
    legs with a stalled fin or no turn are left out. Returns (k, [legs])."""
    import json
    legs = []

    for path in summaries:
        doc = json.load(open(path))
        complete = p['vehicle'] + '_am_fin_drag'

        if doc.get('model') != complete:
            # the fin member has neither drag nor added mass: its held turn
            # is a diagnostic of the fins, not the vehicle's slope
            raise SystemExit('%s: its model %s is not the complete member %s (or the hardware analysed as it)'
                             % (os.path.basename(path), doc.get('model'), complete))

        for leg in doc['legs']:
            sp = leg['stick_phase']
            r = math.radians(sp['r_mean_last5_deg_s'] or 0.0)
            u, torque = sp['u_end'], sp['torque_mean_last5']
            stalled = any((f.get('max_ratio') or 0.0) > 1.0 for f in sp.get('fins', []))

            if not (u and torque) or abs(r) < 1e-3 or stalled:
                continue

            k = (p['N'] * torque - p['nRabsR'] * r * abs(r)) / (u * r)
            legs.append({'summary': os.path.basename(path), 'leg': leg['leg'], 'u': u, 'r_deg_s': math.degrees(r), 'k': k})

    if not legs:
        raise SystemExit('no held turn to measure the slope from: every leg stalled a fin or did not turn')

    return sum(l['k'] for l in legs) / len(legs), legs


def measured_path(vehicle):
    return os.path.join(HERE, 'vehicles', vehicle + '.measured.json')


def plant_signature(p):
    return {'mass': p['mass'], 'm_x': p['m_x'], 'x_bar': p['x_bar'], 'N': p['N']}


def store_measurement(p, k, legs, summaries):
    """The measurement outlives the run: a later --write without --measure
    reads it back, as long as the plant it was taken on is still this one."""
    import json
    doc = {'k': k, 'k_over_N': k / p['N'], 'legs': legs, 'summaries': [os.path.basename(s) for s in summaries],
           'plant': plant_signature(p)}
    path = measured_path(p['vehicle'])
    open(path, 'w').write(json.dumps(doc, indent=1) + '\n')
    return path


def stored_measurement(p):
    """(k, note) from the vehicle's stored measurement, or (None, why not)."""
    import json
    path = measured_path(p['vehicle'])

    if not os.path.exists(path):
        return None, 'no stored measurement (%s): the slope is the files\' seed' % os.path.relpath(path, HERE)

    doc = json.load(open(path))
    now = plant_signature(p)
    off = [k for k in now if abs(doc['plant'][k] - now[k]) > MEASURED_TOL * abs(now[k])]

    if off:
        return None, ('the stored measurement (%s) is of another plant (%s changed): the slope is the files\' seed; measure again'
                      % (os.path.relpath(path, HERE), ', '.join(off)))

    return doc['k'], 'stored measurement %s (%d legs of %s)' % (os.path.relpath(path, HERE), len(doc['legs']), ', '.join(doc['summaries']))


def rnd(x, digits=3):
    return float(('%%.%dg' % digits) % x)


def comment(text):
    """A paragraph as the airframe files carry their comments: 80 columns, '# ' on every line."""
    return textwrap.fill(text, width=80, initial_indent='# ', subsequent_indent='# ')


def blocks(vehicle, p, v, ref):
    """{member suffix: text} of the parameter lines each airframe carries."""
    ff_fin, ff_drag, slp = rnd(v['FF_fin']), rnd(v['FF_drag']), rnd(v['AS_YR_FF_SLP'])
    hi = rnd(v['AS_SPD_HI'], 2)
    source = ('measured in held turns in the simulator' if v['slope_source'] == 'measured'
              else 'seeded from the files, to be replaced by a held-turn measurement')
    schedule = comment('Fin lift damps yaw with the forward speed: a held turn takes %s more normalized yaw torque per rad/s '
                       'for each m/s over the pods\' %.4g N m (%s). Scheduled on the ground speed with no wind bound, since '
                       'the world has none, and capped where the pods can hold the turn; the ramp ends where the cap meets %g '
                       'of the yaw rate that stalls the fins in a turn (%.3g deg/s at %s m/s) and starts at half that'
                       % (slp, p['N'], source, STALL_MARGIN, v['r_stall_at_hi'], hi)) + '''
param set-default AS_SPD_SRC 2
param set-default AS_SPD_WMAX 0
param set-default AS_SPD_HI %s
param set-default AS_YR_FF_SLP %s''' % (hi, slp)
    return {
        '': '''# The generic family's loop bandwidth N P / I and integral time P / I on this
# hull: N %.4g N m, yaw inertia %.4g + %.4g (added) kg m^2
param set-default AS_YAWRATE_P %s
param set-default AS_YAWRATE_I %s''' % (p['N'], p['I_rigid'], p['rr'], rnd(v['AS_YAWRATE_P']), rnd(v['AS_YAWRATE_I'])),
        '_fin': '''# Fin lift damps yaw: the generic fin member's feedforward scaled by the fins'
# held-turn slope over the yaw authority (%.3g against the generic %.3g per m/s)
param set-default AS_YAWRATE_FF %s

%s''' % (v['AS_YR_FF_SLP'], ref['k'] / ref['N'], ff_fin, schedule),
        '_am_fin_drag': '''# Hull drag damps yaw as the square of the rate: holding AS_YAWRATE_MAX in a
# turn in place takes |nRabsR| r^2 = %.3g N m, %s of the pods' yaw authority
# per rad/s, which the loop scales by the setpoint's share of that rate
param set-default AS_YAWRATE_FF %s

%s''' % (p['nRabsR'] * math.radians(REF_GAINS['AS_YAWRATE_MAX']) ** 2, ff_drag, ff_drag, schedule),
    }


def write_airframes(vehicle, p, v, ref):
    written = []

    for suffix, block in blocks(vehicle, p, v, ref).items():
        # the mask-0 member is the plain name in the tree, <vehicle>_rigid in the overlay
        model = gz_model.bare(vehicle) if not suffix else vehicle + suffix
        _, path = gz_model.airframe(model)
        t = open(path).read()
        new = '\n%s\n%s\n' % (MARK_OPEN, block)

        if MARK_OPEN in t:
            t = t[:t.index(MARK_OPEN)].rstrip('\n') + '\n' + new
        else:
            t = t.rstrip('\n') + '\n' + new

        open(path, 'w').write(t)
        written.append(os.path.basename(path))

    return written


def params_file(vehicle, p, v, path):
    """The set the FMU flies the vehicle with on its hardware preset: the
    loop's values of the complete member, which the real vehicle is."""
    lines = ['# %s: the yaw loop values of its complete member (airship_gains.py); the hardware preset supplies the rest' % vehicle,
             'AS_YAWRATE_P=%s' % rnd(v['AS_YAWRATE_P']), 'AS_YAWRATE_I=%s' % rnd(v['AS_YAWRATE_I']),
             'AS_YR_INT_LIM=%s' % v['AS_YR_INT_LIM'], 'AS_YAWRATE_MAX=%s' % v['AS_YAWRATE_MAX'],
             'AS_YAWRATE_FF=%s' % rnd(v['FF_drag']), 'AS_SPD_SRC=2', 'AS_SPD_WMAX=0',
             'AS_SPD_HI=%s' % rnd(v['AS_SPD_HI'], 2), 'AS_YR_FF_SLP=%s' % rnd(v['AS_YR_FF_SLP'])]
    open(path, 'w').write('\n'.join(lines) + '\n')
    return path


def report(p, v):
    print('%s: N %.4g N m, I %.4g + %.4g kg m^2, |nRabsR| %.4g, hull %.3g m, mass %.4g + %.3g (m_x) kg, m_y %.3g kg'
          % (p['vehicle'], p['N'], p['I_rigid'], p['rr'], p['nRabsR'], p['length'], p['mass'], p['m_x'], p['m_y']))
    print('  fins: lift-weighted arm %.3f m, sideslip %.3f m per r/u, held-turn k seed %.4g N m s/rad per m/s (rotational damping alone would say %.4g)'
          % (p['x_bar'], p['slip'], p['k'], p['k_rot']))
    print('  loop bandwidth %.4f rad/s, integral time %.2f s (the anchor plant\'s), length scale %.3f'
          % (v['bandwidth'], v['integral_time'], v['length_scale']))
    for k in ('AS_YAWRATE_P', 'AS_YAWRATE_I', 'AS_YR_INT_LIM', 'AS_YAWRATE_MAX', 'FF_drag', 'FF_fin', 'AS_YR_FF_SLP', 'AS_SPD_HI'):
        print('  %-14s %s' % (k, rnd(v[k], 4)))
    print('  slope %s; at AS_SPD_HI %.2f m/s the cap is %.2f deg/s against the %.2f deg/s that stalls the fins in a turn; AS_YAWRATE_MAX %g'
          % (v['slope_source'], v['AS_SPD_HI'], v['cap_at_hi'], v['r_stall_at_hi'], v['AS_YAWRATE_MAX']))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('vehicle', help='the bare model name, e.g. airship_indoor')
    ap.add_argument('--write', action='store_true', help='write the values into the posix airframes')
    ap.add_argument('--params', help='also write the FMU parameter set to this file')
    ap.add_argument('--measure', nargs='+', metavar='SUMMARY', help='forward_yaw_check summaries of this vehicle: the slope from their held turns')
    args = ap.parse_args(argv)
    ref = REF_PLANT
    p = plant(args.vehicle)
    if args.measure:
        measured, legs = measure_slope(p, args.measure)

        for l in legs:
            print('  held turn %s leg %d: u %.2f m/s, r %.2f deg/s -> k %.4g (k/N %.3f)' % (l['summary'], l['leg'], l['u'], l['r_deg_s'], l['k'], l['k'] / p['N']))

        print('  stored in %s' % os.path.relpath(store_measurement(p, measured, legs, args.measure), HERE))
    else:
        measured, note = stored_measurement(p)
        print('  ' + note)

    v = gains(p, ref, measured)
    report(p, v)

    if args.write:
        print('  wrote', ', '.join(write_airframes(args.vehicle, p, v, ref)))

    if args.params:
        print('  wrote', params_file(args.vehicle, p, v, args.params))

    return 0


if __name__ == '__main__':
    sys.exit(main())
