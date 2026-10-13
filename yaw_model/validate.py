#!/usr/bin/env python3
"""The plants against what the gate measured open-loop.

    python3 validate.py

The gate's first check (sitl_check.py's '바람 확인' phase) is the only one with a
known torque and no controller: Manual, stick centred (torque 0), a persistent
1 N m world-z couple on the model (gz_link.wrench) for sample('바람 확인', 5).
It reports after[-1] - before. That is open loop, so it tests the plant alone.

1. Every recorded wind check of the current hull revision (rev 3; chain4 and
   earlier had I = 122.8 and are excluded), on the four plants and on the
   single-term variants that isolate each term (_drag: hull damping alone;
   _fin: fins alone; _am_fin: added mass and fins).
2. The one time series on disk, gz_cloudship_am_fin_drag from chain7
   (sitl_out/samples.csv, 2026-09-25 23:54, frozen copy in data/): the wind
   phase fitted for the torque onset and the fin share, then the Manual phase
   (stick 0.5, open loop) predicted with one stated actuator assumption.
3. Any sitl_out/samples_<target>_chain9.csv present when this runs.

The effective torque duration. sample() is wall-clock (sitl_check.py:89-105):
the wrench is published between the last '바람 확인 기준' row and the first
'바람 확인' row, and in the chain7 series that first row still reads 0.05
deg/s while the next reads -1.82, so the couple lands at the first row. The
rows are 0.316 s apart and the phase has 16 of them: the torque has acted for
the 15 intervals, 4.74 s, when the last row is read, not 5 s. The fit below
recovers the onset from the data; the records are then evaluated at the
fitted duration and at 5.0 s for reference.
"""
import csv
import glob
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import plants  # noqa: E402

DEG = math.pi / 180.0
SITL_OUT = os.path.join(plants.BENCH, 'sitl_out')
SERIES = os.path.join(HERE, 'data', 'samples_gz_cloudship_am_fin_drag_chain7.csv')

# (model, record file:line, before, after) — the gate's own printed numbers.
RECORDS = [
    ('cloudship', 'chain5_20260925.txt:107', 0.08, -46.75),
    ('cloudship', 'chain6_20260925.txt:55', 0.10, -46.58),
    ('cloudship', 'chain7_20260925.txt:112', 0.10, -46.58),
    ('cloudship_drag', 'gate_cloudship_drag_20260925.txt:7', 0.05, -24.18),
    ('cloudship_fin', 'chain5_20260925.txt:227', 0.14, -42.00),
    ('cloudship_am_fin_drag', 'chain5_20260925.txt:248', 0.06, -19.87),
    ('cloudship_am_fin_drag', 'chain7_20260925.txt:133', 0.00, -19.87),
    ('airship_vectored_independent', 'chain5_20260925.txt:128', -0.03, -2.09),
    ('airship_vectored_independent', 'chain7_20260925.txt:66', -0.00, -2.09),
    ('airship_vectored_independent_drag', 'chain5_20260925.txt:151', -0.01, -1.91),
    ('airship_vectored_independent_fin', 'chain5_20260925.txt:174', -0.03, -2.00),
    ('airship_vectored_independent_am_fin', 'chain5_20260925.txt:197', 0.03, -0.82),
    ('airship_vectored_independent_am_fin_drag', 'chain5_20260925.txt:220', -0.04, -0.87),
    ('airship_vectored_independent_am_fin_drag', 'chain6_20260925.txt:32', -0.03, -0.84),
    ('airship_vectored_independent_am_fin_drag', 'chain7_20260925.txt:89', -0.01, -0.86),
    ('cloudship', 'chain9_20260927.txt:79', 0.10, -46.81),
    ('airship_vectored_independent', 'chain9_20260927.txt:100', 0.01, -2.11),
    ('airship_vectored_independent_fin', 'chain9_20260927.txt:123', -0.00, -1.97),
]


def invert_manual(p, rows, label):
    """On a rigid plant (pure inertia) the open-loop Manual phase inverts to the
    torque the actuators delivered: N = I dr/dt. Actuator-side, for the next part."""
    man = [r for r in rows if r['phase'] == 'Manual 통과']
    print('  %s Manual phase (stick 0.5, open loop), N = I dr/dt with I %.4f:' % (label, p.I))

    for a, b in zip(man, man[1:]):
        n = p.I * (b['rate'] - a['rate']) * DEG / (b['t'] - a['t'])
        print('     %.2f-%.2f s  %7.2f -> %7.2f deg/s  torque_sp %.3f  delivered %7.3f N m = %.3f of N_full'
              % (a['t'], b['t'], a['rate'], b['rate'], b['torque'], n, n / p.n_full))

    span = man[-1]['t'] - man[0]['t']
    imp = p.I * (man[-1]['rate'] - man[0]['rate']) * DEG
    print('     impulse %.2f N m s over %.2f s against the ideal %.2f (0.5 x %.3f N m): %.0f %%'
          % (imp, span, 0.5 * p.n_full * span, p.n_full, 100 * imp / (0.5 * p.n_full * span)))


def plant_for(model):
    """Any family member as a Plant from its files, refused when the family
    is not the hull the RECORDS were flown on (models 78389ea: the family's
    yaw inertia is held to plants.EXPECTED's rigid target of that vehicle)."""
    fam = plants.gz_model.family(model)
    merged = plants.gz_model.merged(model)
    rigid = 'gz_' + plants.gz_model.vehicle(model)
    expected = plants.EXPECTED[rigid]['I_rigid']
    inertia = plants.gz_model.yaw_inertia(fam)

    if abs(inertia - expected) > 1e-4 * expected:
        raise SystemExit('%s: the tree\'s family yaw inertia %.4f is not the anchor\'s %.4f (models 78389ea): '
                         'these records are of another hull' % (model, inertia, expected))

    _, com = plants.gz_model.centre_of_mass(fam)
    return plants.Plant(model, model, plants.gz_model.yaw_inertia(fam),
                        plants.gz_model.added_mass(merged).get('rr', 0.0),
                        abs(plants.gz_model.hydrodynamics(merged).get('nRabsR', 0.0)),
                        plants._fins_from_sdf(merged, com), com[0], plants.gz_model.yaw_authority(model), {}, {})


def rate_after(p, seconds, fins, torque=1.0):
    return plants.integrate(p, lambda t, r: torque, 0.0, seconds, 0.001, fins=fins)[-1][1] / DEG


def load(path):
    with open(path) as handle:
        return [{'t': float(r['t']), 'phase': r['phase'], 'rate': float(r['rate']),
                 'torque': float(r['torque']) if r['torque'] not in ('', 'nan') else float('nan')}
                for r in csv.DictReader(handle)]


def fit_wind(p, rows):
    """Least squares over (onset t0, fin share) with I and c_hull from the files.
    The gate's rate is FRD and the couple is +z up, so the rate goes negative."""
    wind = [r for r in rows if r['phase'] == '바람 확인']
    base = [r for r in rows if r['phase'] == '바람 확인 기준']
    r0 = -base[-1]['rate'] * DEG
    best = None
    shares = [0.0, 0.5, 1.0, 1.5, 2.0] if p.fins else [0.0]

    for share in shares:
        for k in range(-40, 41):
            t0 = wind[0]['t'] + 0.01 * k
            traj = plants.integrate(p, lambda t, r: 1.0, r0, wind[-1]['t'] - t0 + 0.01, 0.002, fins=share)
            sse = 0.0

            for row in wind:
                tau = row['t'] - t0
                model = 0.0 if tau <= 0 else traj[min(len(traj) - 1, int(round(tau / 0.002)))][1] / DEG
                sse += (model - (-row['rate'])) ** 2

            if best is None or sse < best[0]:
                best = (sse, share, t0)

    return best, wind


def report_series(p, rows, label):
    best, wind = fit_wind(p, rows)
    sse, share, t0 = best
    n = len(wind)
    print('  %s wind phase, %d rows %.2f..%.2f s: best fin share %.1f, onset %.2f s '
          '(%.2f s after the first row), rms %.3f deg/s'
          % (label, n, wind[0]['t'], wind[-1]['t'], share, t0, t0 - wind[0]['t'], math.sqrt(sse / n)))

    for s in ([0.0, 1.0, 2.0] if p.fins else [0.0]):
        traj = plants.integrate(p, lambda t, r: 1.0, 0.0, wind[-1]['t'] - t0, 0.002, fins=s)
        errs = []

        for row in wind:
            tau = row['t'] - t0
            model = 0.0 if tau <= 0 else traj[min(len(traj) - 1, int(round(tau / 0.002)))][1] / DEG
            errs.append(model - (-row['rate']))

        print('     fins %.0f: last row model %.2f vs measured %.2f deg/s, rms %.3f, max |err| %.3f'
              % (s, traj[-1][1] / DEG, -wind[-1]['rate'], math.sqrt(sum(e * e for e in errs) / n),
                 max(abs(e) for e in errs)))

    return t0, wind[-1]['t'] - t0


def manual_phase(p, rows, share):
    """Manual, stick 0.5, open loop. Assumption (actuator side, not the plant):
    the tail delivers exactly torque_sp x N_full with no lag (GenericMotorModel
    force_polynomial has none, cloudship/model.sdf:533-535; THR_MDL_FAC 1,
    2510_gz_cloudship:65, taken to make thrust linear in the command) and the
    wind couple is already cleared at the first Manual row. Onset fitted."""
    man = [r for r in rows if r['phase'] == 'Manual 통과']
    r_start = -man[0]['rate'] * DEG      # z-up, same sign as the wind response
    best = None

    for k in range(0, 33):
        t0 = man[0]['t'] + 0.01 * k
        # z-up: the stick's +0.5 FRD torque is -0.5 z-up
        traj = plants.integrate(p, lambda t, r: 0.0 if t < t0 - man[0]['t'] else -0.5 * p.n_full,
                                r_start, man[-1]['t'] - man[0]['t'], 0.002, fins=share)
        errs = []

        for row in man:
            tau = row['t'] - man[0]['t']
            errs.append(traj[min(len(traj) - 1, int(round(tau / 0.002)))][1] / DEG - (-row['rate']))

        rms = math.sqrt(sum(e * e for e in errs) / len(errs))

        if best is None or rms < best[0]:
            best = (rms, t0, errs, traj)

    rms, t0, errs, traj = best
    print('  Manual phase (stick 0.5 = %.3f N m assumed), %d rows %.2f..%.2f s, fins %.0f: onset %.2f s, '
          'rms %.3f deg/s, max |err| %.3f; last row model %.2f vs measured %.2f (FRD)'
          % (0.5 * p.n_full, len(man), man[0]['t'], man[-1]['t'], share, t0, rms, max(abs(e) for e in errs),
             -traj[-1][1] / DEG, man[-1]['rate']))


def main():
    print('== 1. the complete Cloudship time series (%s)' % os.path.relpath(SERIES, HERE))
    rows = load(SERIES)
    p = plants.PLANTS['gz_cloudship_am_fin_drag']
    t0, span = report_series(p, rows, 'gz_cloudship_am_fin_drag chain7')
    manual_phase(p, rows, 1.0)
    manual_phase(p, rows, 0.0)
    manual_phase(p, rows, 2.0)

    print('\n== 2. every recorded wind check, 1 N m open loop; model at the fitted %.2f s and at 5.00 s' % span)
    print('  %-42s %-38s %8s | %8s %8s %8s | %8s' % ('model', 'record', 'measured', 'fins 0', 'fins 1', 'fins 2',
                                                     '5 s, f1'))

    for model, where, before, after in RECORDS:
        q = plant_for(model)
        measured = before - after
        cols = [rate_after(q, span, s) for s in (0.0, 1.0, 2.0)] if q.fins else [rate_after(q, span, 0.0)] * 3
        five = rate_after(q, 5.0, 1.0 if q.fins else 0.0)
        print('  %-42s %-38s %8.2f | %8.3f %8.3f %8.3f | %8.3f   (I %.2f, c_hull %.2f, fins %s)'
              % (model, where, measured, cols[0], cols[1], cols[2], five, q.I, q.c_hull,
                 ','.join('%.1f' % c for c in q.c_fin_each()) or '-'))

    print('\n  Same with the measured inertias (added_mass_check, chain5:19-20, :31-32):')

    for model, where, before, after in RECORDS:
        q = plant_for(model)
        meas = plants.MEAS_VI if 'vectored' in model else plants.MEAS_CS
        q.I = meas['I_rigid'] + (meas['rr'] if q.rr else 0.0)
        cols = [rate_after(q, span, s) for s in (0.0, 1.0, 2.0)] if q.fins else [rate_after(q, span, 0.0)] * 3
        print('  %-42s %-38s %8.2f | %8.3f %8.3f %8.3f   (I %.2f)'
              % (model, where, before - after, cols[0], cols[1], cols[2], q.I))

    # the frozen copies in data/ first, then anything newer the chain has written since
    found = {}

    frozen = sorted(glob.glob(os.path.join(HERE, 'data', 'samples_*_chain9.csv')))
    live = sorted(glob.glob(os.path.join(SITL_OUT, 'samples_*_chain9.csv')))

    for path in frozen + live:
        found.setdefault(os.path.basename(path), path)

    print('\n== 3. chain9 series: %s' % (', '.join(os.path.relpath(x, HERE) if x.startswith(HERE) else x
                                                   for x in found.values()) or 'none yet'))
    rigid = []

    for name, path in sorted(found.items()):
        target = name[len('samples_'):-len('_chain9.csv')]
        model = target[3:] if target.startswith('gz_') else target
        try:
            q = plant_for(model)
            data = load(path)
            report_series(q, data, target + ' chain9')

            if not q.fins and not q.c_hull and not q.rr:
                rigid.append((q, data, target))
        except Exception as exc:  # a series still being written, or no wind phase
            print('  %s: skipped (%s)' % (target, exc))

    print('\n== 4. actuator-side, from the rigid plants: the Manual phase inverted')

    for q, data, target in rigid:
        invert_manual(q, data, target)

    return 0


if __name__ == '__main__':
    sys.exit(main())
