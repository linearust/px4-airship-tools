#!/usr/bin/env python3
"""The model's fin yaw damping, measured the way the engine's was.

    python3 fin_damping.py [model]       # default airship_vectored_independent_fin

added_mass_check.py measured the fins of the rigid _fin plants in gz: from
rest, a push through the centre of mass, the pose stream, (u, v, r) by central
differences, r fitted against the integrals of u v and u r (K_s / I, K_d / I),
each times the yaw inertia the same check measured with a YAW_ACCEL torque.
Its printouts (engine_runs reads them from the chain logs, CHAIN_LOGS: chain5
and chain9 for the vectored airship, chain5 for the Cloudship) gave K_d(fin)
216.90 and 216.44 on the vectored _fin plant against 229.81 from the files
(sum of 0.5 rho A cla |x_cp - x_com| times the last vertical fin's arm,
added_mass_check.expected()).

Here the same experiment runs on the model's plant (planar.RollPlanarPlant,
whose fins are the LiftDrag replica the forward legs fly on): the same push
and couple (+d x F, world frame, held while the hull turns, as gz's
persistent wrench is), the same yaw-torque experiment for the inertia, and
the engine check's own kinematics() and yaw_fit() run on poses the model
writes. Constants and functions are lifted from added_mass_check.py with ast,
so the procedure cannot drift from the one that made the engine numbers.

The stream, from the engine's own record. added_mass_check also writes the
kinematics of every push it fits to sitl_out/added_mass_<model>.csv (t, u, v,
r per pose but the first and last; each run overwrites them). The files on
disk refit to one printed run each (engine_record says which: chain9 for the
vectored airship, chain5 for the Cloudship), and they fix two things the
check's constants do not:
  - the interval. The poses come every 0.020 s (two gaps of 0.016 s on one
    record), every fifth 4 ms step, not 1 / RATE_HZ (the check's 60 Hz). The
    check asks for int(PUSH_S x RATE_HZ) = 150 poses, so its push window is
    3.0 s, not PUSH_S, and its yaw-torque window int(YAW_S x RATE_HZ) = 120
    poses, 2.4 s (if that experiment's poses came as the push's did; it
    keeps no record).
  - the start. Under the known push the first row's u says how long the push
    had acted when the stream began: the `gz topic -e` that samples starts
    after the publish returns, a delay the check does not print but its record
    carries.
So the model's push is sampled at the record's own pose times, from the whole
4 ms step at which its first row reads the record's first u (matched_start),
and its inertia as the check sampled it (int(YAW_S x RATE_HZ) poses at the
record's interval). What is left between the two is the plant: K_d / I and
K_s / I against the record's own fit (inertia-free), r(t) row by row on the
same times since the push began, and the two inertias apart.

The procedure's sensitivity. K_d / I moves by several per cent when the start
alone moves from 0 to 0.6 s (start_band): the fit is linear in u r on a plant
that is not, over a window whose content shifts with the start. A model-engine
difference inside that spread says the two agree as far as this procedure can
tell, not that they agree to that difference. The acceptance test of the fin
damping as flown is the forward probe's steady turn (step0a.probes: the hold
torque per rad/s, SITL against the model, at the operating point) and the 0b
check.
"""
import ast
import csv
import functools
import math
import os
import re
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gate  # noqa: E402  (bench tools on sys.path, no bytecode)
import planar  # noqa: E402

AMC = os.path.join(gate.BENCH, 'added_mass_check.py')
SITL_OUT = os.path.join(gate.BENCH, 'sitl_out')
# The engine runs whose added_mass printouts the model is held against
CHAIN_LOGS = ('chain5_20260925.txt', 'chain9_20260927.txt')
# A record belongs to a printed run when its refit gives that run's numbers
# to this relative tolerance: the record keeps u, v, r to 1e-5, which moves
# the refit by at most 1.1e-4 (the vectored _am_fin, a small K_s after the
# held damping is subtracted); the two vectored runs differ by 1.7e-3 there
# and 2.1e-3 in K_d
RECORD_TOL = 5e-4
DT = 0.004                                              # the gz step, worlds/lta.sdf
gz_model = planar.gz_model


def _lift():
    """added_mass_check's constants and fitting functions, compiled alone."""
    with open(AMC, encoding='utf-8') as handle:
        tree = ast.parse(handle.read(), AMC)

    consts = ('YAW_ACCEL', 'YAW_S', 'PUSH_ACCEL', 'PUSH_S', 'RATE_HZ')
    funcs = ('yaw', 'unwrap', 'to_body', 'to_world', 'kinematics', 'fit_line', 'fit_plane', 'yaw_fit',
             'yaw_inertia')
    nodes = [n for n in tree.body
             if (isinstance(n, ast.Assign) and len(n.targets) == 1 and getattr(n.targets[0], 'id', None) in consts)
             or (isinstance(n, ast.FunctionDef) and n.name in funcs)]
    found = {getattr(n, 'name', None) or n.targets[0].id for n in nodes}

    if set(consts + funcs) - found:
        raise LookupError('added_mass_check.py no longer defines %s' % ', '.join(sorted(set(consts + funcs) - found)))

    space = {'math': math}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), AMC, 'exec'), space)
    return space


AM = _lift()

# What one added_mass block prints, per quantity (added_mass_check.main's
# verdict lines); each must be found once in every block
_PRINTED = {
    'I': re.compile(r'요 관성이 SDF 가 말하는 값이다 — 측정 ([-+\d.]+), SDF'),
    'dI': re.compile(r'요 관성에 rr 만큼 더해진다 — 측정 차 ([-+\d.]+),'),
    'K_s_fin': re.compile(r'K_s\(fin\) ([-+\d.]+),'),
    'K_d_fin': re.compile(r'K_d\(fin\) ([-+\d.]+),'),
    'am_fin': re.compile(r'— K_s ([-+\d.]+) \(기대 [-+\d.]+\), 고정한 K_d ([-+\d.]+);'),
}


@functools.lru_cache(maxsize=None)
def engine_runs(vehicle):
    """The engine's printed numbers for `vehicle`, one dict per
    '=== added_mass <vehicle> ===' block of CHAIN_LOGS: I (the rigid yaw
    inertia it measured), I_am (I plus the measured difference), K_s_fin,
    K_d_fin, K_s_am_fin and K_d_held (the rigid fin damping it held), and
    'where' (file:line of each). Raises when a block lacks one of them, or
    when no block names the vehicle."""
    runs = []

    for name in CHAIN_LOGS:
        with open(os.path.join(SITL_OUT, name), encoding='utf-8') as handle:
            lines = handle.read().splitlines()

        run = None

        for number, line in enumerate(lines + ['=== end'], 1):
            if line.startswith('=== '):
                if run is not None:
                    runs.append(_closed(run))

                run = {'file': name, 'where': {}} if line == '=== added_mass %s ===' % vehicle else None
                continue

            if run is None:
                continue

            for key, pattern in _PRINTED.items():
                m = pattern.search(line)

                if not m:
                    continue

                if key == 'am_fin':
                    run['K_s_am_fin'], run['K_d_held'] = float(m.group(1)), float(m.group(2))
                else:
                    run[key] = float(m.group(1))

                run['where'][key] = '%s:%d' % (name, number)

    if not runs:
        raise LookupError('no "=== added_mass %s ===" block in %s' % (vehicle, ', '.join(CHAIN_LOGS)))

    return tuple(runs)


def _closed(run):
    missing = [k for k in _PRINTED if k not in run['where']]

    if missing:
        raise LookupError('%s: an added_mass block without %s' % (run['file'], ', '.join(missing)))

    run['I_am'] = run['I'] + run['dI']
    return run


def engine_record(model):
    """(rows, run, fit): the engine's own kinematics of its last push on
    `model` [(t, u, v, r)], the printed run (engine_runs) whose numbers they
    refit to, and that refit {'S', 'D'} per unit inertia (D None on an _am_fin
    record, whose damping the run held). run is None when the record refits to
    no printed run. Raises when the file is not there: a comparison that
    needs the engine's stream does not fall back to the check's constants."""
    path = os.path.join(SITL_OUT, 'added_mass_%s.csv' % model)

    with open(path, encoding='utf-8') as handle:
        rows = [(float(r['t']), float(r['u']), float(r['v']), float(r['r'])) for r in csv.DictReader(handle)]

    close = lambda a, b: abs(a - b) <= RECORD_TOL * abs(b)

    for run in engine_runs(gz_model.vehicle(model)):
        if model.endswith('_am_fin'):
            s, _, _, _ = AM['yaw_fit'](rows, damped=True, damping_known=run['K_d_held'] / run['I_am'])

            if close(s * run['I_am'], run['K_s_am_fin']):
                return rows, run, {'S': s, 'D': None}
        elif model.endswith('_fin'):
            s, d, _, _ = AM['yaw_fit'](rows, damped=True)

            if close(s * run['I'], run['K_s_fin']) and close(d * run['I'], run['K_d_fin']):
                return rows, run, {'S': s, 'D': d}

    return rows, None, None


def record_stream(rows):
    """(offsets, interval, gaps): the record's pose times after its first
    pose [s], the median interval, and the intervals that are not it. The
    kinematics rows are poses 1 .. n-2; the first and last poses, which the
    central differences use and the file does not keep, are one median
    interval outside."""
    times = [r[0] for r in rows]
    gaps = [round(b - a, 6) for a, b in zip(times, times[1:])]
    interval = statistics.median(gaps)
    poses = [times[0] - interval] + times + [times[-1] + interval]
    return [round(t - poses[0], 6) for t in poses], interval, sorted({g for g in gaps if abs(g - interval) > 1e-9})


def even(count, interval):
    """count poses every interval, as the check's sample() asks for them."""
    return [i * interval for i in range(count)]


class Probe:
    """RollPlanarPlant with a world-fixed wrench at the link origin and the
    world pose integrated, sampled as gz's pose stream."""

    def __init__(self, model, cla_scale=1.0):
        """cla_scale: the vertical fins' lift slope times this, for the
        comparison's own sensitivity (measure's cla_scale); 1 is the files'."""
        self.plant = planar.RollPlanarPlant(model, model=model)
        self.pos = [0.0, 0.0]
        self.rows = []

        for f in self.plant.fins:               # fins_of builds these per plant
            if f['vertical']:
                f['cla'] *= cla_scale

    def quaternion(self):
        p, s = self.plant.phi, self.plant.psi              # theta held at 0: q = q_z(psi) q_x(phi)
        cp, sp, cs, ss = math.cos(p / 2), math.sin(p / 2), math.cos(s / 2), math.sin(s / 2)
        return (cs * cp, cs * sp, ss * sp, ss * cp)

    def run(self, force_w, torque_w, offsets, dt=DT, late=0.0):
        """force_w, torque_w: world frame (x, y, z up), on base_link at O.
        offsets: the pose times after the stream's first pose [s]. late: how
        long the wrench acts before the stream's first pose (negative: the
        stream starts first, the hull at rest until the wrench comes on).
        Times are whole steps; a row's t is its time since the wrench came on."""
        start = int(round(late / dt))
        on = max(0, -start)                                 # the step the wrench comes on
        marks = {max(0, start) + int(round(o / dt)) for o in offsets}
        n = max(marks)

        for k in range(n + 1):
            if k in marks:
                self.rows.append(((k - on) * dt, (self.pos[0], self.pos[1], 0.0), self.quaternion()))

            if k == n:
                break

            ph, ps = self.plant.phi, self.plant.psi
            cps, sps, cph, sph = math.cos(ps), math.sin(ps), math.cos(ph), math.sin(ph)

            def body(v):
                # R_x(phi)^T R_z(psi)^T v (theta 0)
                x = cps * v[0] + sps * v[1]
                y = -sps * v[0] + cps * v[1]
                return (x, cph * y + sph * v[2], -sph * y + cph * v[2])

            on_now = k >= on
            f = body(force_w) if on_now else (0.0, 0.0, 0.0)
            t = body(torque_w) if on_now else (0.0, 0.0, 0.0)
            self.plant.step_actuated(dt, _Wrench(f + t), 0.0, 0.0)
            u, v = self.plant.nu[0], self.plant.nu[1]
            self.pos[0] += dt * (u * cps - v * cph * sps)
            self.pos[1] += dt * (u * sps + v * cph * cps)

        return self.rows


class _Wrench:
    """An 'actuation' whose pod_wrench is a fixed body wrench at O, FLU."""

    def __init__(self, w):
        self.w = w

    def pod_wrench(self, dt):
        return self.w


def _push(model):
    """(F, couple): added_mass_check's push on `model`, PUSH_ACCEL times the
    family's mass through its centre of mass (+d x F; the trim is level, so d
    is already in the world frame)."""
    mass, com = gz_model.centre_of_mass(gz_model.family(model))
    F = tuple(a * mass for a in AM['PUSH_ACCEL'])
    d = com
    return F, (d[1] * F[2] - d[2] * F[1], d[2] * F[0] - d[0] * F[2], d[0] * F[1] - d[1] * F[0])


@functools.lru_cache(maxsize=None)
def inertia(model, interval):
    """The yaw-torque experiment on `model`: YAW_ACCEL times the vehicle's
    SDF yaw inertia, sampled as the check samples it (int(YAW_S x RATE_HZ)
    poses at `interval`), from the torque's start (a pure quadratic's
    curvature does not depend on the start). (fitted inertia, torque)."""
    izz = gz_model.yaw_inertia(gz_model.family(gz_model.vehicle(model)))
    torque = AM['YAW_ACCEL'] * izz
    rows = Probe(model).run((0.0, 0.0, 0.0), (0.0, 0.0, torque), even(int(AM['YAW_S'] * AM['RATE_HZ']), interval))
    return AM['yaw_inertia'](rows, torque)[0], torque


def matched_start(model, offsets, u_first, lo=-25, hi=150):
    """(steps, |error|, u): the whole steps the push leads the model's stream
    by, so that its first kinematics row reads the record's first u. u at
    that row grows with the lead, so the search stops past the match."""
    F, couple = _push(model)
    best = None

    for k in range(lo, hi + 1):
        u = AM['kinematics'](Probe(model).run(F, couple, offsets[:3], late=k * DT))[0][1]
        err = abs(u - u_first)

        if best is not None and err > best[1]:
            break

        best = (k, err, u)

    return best


def measure(model='airship_vectored_independent_fin', late=None, aligned=True, cla_scale=1.0):
    """The engine's two experiments on the model: the yaw inertia on the
    RIGID member (added_mass_check runs its torque on the vehicle and its _am
    member only, and multiplies the _fin fit by the rigid one), the push on
    `model`. aligned: sampled at the engine record's pose times and started
    where it started (the module docstring); late [s] overrides the start.
    aligned False: the check's constants (int(PUSH_S x RATE_HZ) poses at
    1 / RATE_HZ) from the push's start, what the check assumes it does.
    cla_scale: the push on fins whose lift slope is scaled (the start is
    then taken from the files' fins, so only the fins differ)."""
    rigid = gz_model.vehicle(model)
    F, couple = _push(model)
    record = run = fit = start = None

    if aligned:
        record, run, fit = engine_record(model)
        offsets, interval, gaps = record_stream(record)
    else:
        interval = 1.0 / AM['RATE_HZ']
        offsets, gaps = even(int(AM['PUSH_S'] * AM['RATE_HZ']), interval), []

    if late is None:
        if record is not None:
            start = matched_start(model, offsets, record[0][1])
            late = start[0] * DT
        else:
            late = 0.0

    I, torque = inertia(rigid, interval)
    kin = AM['kinematics'](Probe(model, cla_scale).run(F, couple, offsets, late=late))
    s, dmp, icpt, uvmax = AM['yaw_fit'](kin, damped=True)
    beta = max(abs(math.degrees(math.atan2(v, u))) for _, u, v, _ in kin if math.hypot(u, v) > 0.3)
    _, com = gz_model.centre_of_mass(gz_model.family(model))

    # the files' value, as added_mass_check.expected() forms it
    vertical = [f for f in gz_model.lift_drag_fins(gz_model.merged(model)) if f['vertical']]
    fin_k = sum(0.5 * gz_model.RHO_AIR * f['area'] * f['cla'] * abs(f['cp'][0] - com[0]) for f in vertical)
    files = fin_k * abs(vertical[-1]['cp'][0] - com[0])
    out = {'model': model, 'rigid': rigid, 'late': late, 'start': start, 'interval': interval, 'gaps': gaps,
           'poses': len(offsets), 'window': offsets[-1], 'D': dmp, 'S': s, 'I_fit': I, 'torque': torque,
           'I_sdf': gz_model.yaw_inertia(gz_model.family(rigid)), 'K_s': s * I, 'K_d': dmp * I, 'icpt': icpt,
           'uvmax': uvmax, 'beta': beta, 'files': files, 'files_static': fin_k, 'linear_O': linear_O(model),
           'u_end': kin[-1][1], 'v_end': kin[-1][2], 'r_end': kin[-1][3], 'push': F, 'couple': couple,
           'record': record, 'run': run, 'record_fit': fit, 'kin': kin}

    if record is not None and len(record) == len(kin):
        out['rows_diff'] = {name: [m[i] - e[i] for m, e in zip(kin, record)] for i, name in ((1, 'u'), (2, 'v'), (3, 'r'))}
        out['t_skew'] = max(abs((m[0] - kin[0][0]) - (e[0] - record[0][0])) for m, e in zip(kin, record))

    return out


def linear_O(model, u0=2.0, r0=0.002):
    """The replica linearised: the vertical fins alone at u0, a small yaw
    rate r0, no sway, no roll; the moment about the link origin per unit u r,
    N m s/rad per m/s."""
    n_fin = 0.0

    for f in planar.RollPlanarPlant(model, model=model).fins:
        vcp = planar._cross((0.0, 0.0, r0), f['cp'])
        n_fin += planar._cross(f['cp'], planar.lift_drag((u0 + vcp[0], vcp[1], vcp[2]), f))[2]

    return -n_fin / (u0 * r0)


def fin_leverage(model, scales):
    """What a change of the vertical fins' lift slope does to this
    comparison: per scale, K_d / I and the r rms against the engine's record
    on its stream, the same start. The files' fins are scale 1."""
    out = []

    for scale in scales:
        m = measure(model, cla_scale=scale)
        r = m.get('rows_diff', {}).get('r')
        out.append((scale, m['D'], math.sqrt(sum(x * x for x in r) / len(r)) if r else None))

    return out


def rigid_residual(vehicle):
    """The engine's record of the push on the RIGID `vehicle` against the
    model's on the same stream and start: the motion the record carries that
    neither has fins for. {'start', 'u_rms', 'u_max', 'v_rms', 'r_end'
    (record, model) [rad/s]}, or None when there is no rigid record."""
    path = os.path.join(SITL_OUT, 'added_mass_%s.csv' % vehicle)

    if not os.path.exists(path):
        return None

    with open(path, encoding='utf-8') as handle:
        record = [(float(r['t']), float(r['u']), float(r['v']), float(r['r'])) for r in csv.DictReader(handle)]

    offsets, interval, _ = record_stream(record)
    start = matched_start(vehicle, offsets, record[0][1])
    F, couple = _push(vehicle)
    kin = AM['kinematics'](Probe(vehicle).run(F, couple, offsets, late=start[0] * DT))
    du = [m[1] - e[1] for m, e in zip(kin, record)]
    dv = [m[2] - e[2] for m, e in zip(kin, record)]
    rms = lambda xs: math.sqrt(sum(x * x for x in xs) / len(xs))
    return {'start': start, 'u_rms': rms(du), 'u_max': max(du, key=abs), 'v_rms': rms(dv),
            'r_end': (record[-1][3], kin[-1][3]), 'r_max': max(abs(x[3]) for x in record), 'rows': len(record)}


def start_band(model, lates):
    """K_d / I and K_s / I of the model's procedure, on the engine record's
    pose times, with the stream starting at each of `lates` [s] into the
    push: the procedure's own sensitivity to a quantity it does not print."""
    return [(late, m['D'], m['S']) for late, m in ((x, measure(model, late=x)) for x in lates)]


def measure_am_fin(vehicle, fin=None):
    """added_mass_check's third fit on the model: the inertia of the _am
    member by the torque experiment, the push on the _am_fin member (its
    record's stream, as measure()), r fitted against the integral of u v with
    the rigid _fin damping held (the model's own, measure(), the way the check
    holds its own), K_s = slope x I_am: Munk and fins together, the plant's
    static yaw stability per (m/s)^2."""
    fin = fin or measure(vehicle + '_fin')
    am, am_fin = vehicle + '_am', vehicle + '_am_fin'
    record, run, _ = engine_record(am_fin)
    offsets, interval, _ = record_stream(record)
    start = matched_start(am_fin, offsets, record[0][1])
    I_am, _ = inertia(am, interval)
    F, couple = _push(am_fin)
    kin = AM['kinematics'](Probe(am_fin).run(F, couple, offsets, late=start[0] * DT))
    slope, _, icpt, _ = AM['yaw_fit'](kin, damped=True, damping_known=fin['K_d'] / I_am)
    block = gz_model.added_mass(gz_model.merged(am))
    beta = max(abs(math.degrees(math.atan2(v, u))) for _, u, v, _ in kin if math.hypot(u, v) > 0.3)
    return {'vehicle': vehicle, 'I_am': I_am, 'K_s': slope * I_am, 'K_d_held': fin['K_d'], 'icpt': icpt,
            'files': fin['files_static'] - (block['yy'] - block['xx']), 'beta': beta, 'run': run, 'start': start,
            'interval': interval}


def report_am_fin(m, say=print):
    say('net static yaw term on %s_am_fin (Munk and fins), the engine\'s procedure on the model, on its record\'s '
        'stream (start %+d steps): K_s %.4f (inertia %.4f, the model\'s rigid fin damping %.4f held; max sideslip '
        '%.1f deg); files %.4f' % (m['vehicle'], m['start'][0], m['K_s'], m['I_am'], m['K_d_held'], m['beta'],
                                   m['files']))

    for e in engine_runs(m['vehicle']):
        say('  engine (%s, %s): K_s %.4f with its K_d %.4f held, inertia %.4f; model - engine %+.4f%s'
            % (e['where']['am_fin'], e['where']['dI'], e['K_s_am_fin'], e['K_d_held'], e['I_am'],
               m['K_s'] - e['K_s_am_fin'], '  (the record on disk is this run\'s)' if e is m['run'] else ''))


def report(m, say=print, lates=(), scales=()):
    """The comparison in text, per quantity the engine printed, with the
    procedure's sensitivity beside it: to the stream's start (lates, s) and
    to the fins themselves (scales of their lift slope)."""
    say('fin yaw damping, %s: the engine\'s procedure (added_mass_check.py) on the model\'s plant' % m['model'])
    run, record = m['run'], m['record']

    if record is not None:
        say('  the engine\'s record sitl_out/added_mass_%s.csv: %d rows, poses every %.3f s%s, %d poses over %.2f s '
            '(the check asks for int(PUSH_S x RATE_HZ) = %d at 1 / %g s); its first row u %.5f m/s'
            % (m['model'], len(record), m['interval'],
               ' (gaps of %s s too)' % ', '.join('%.3f' % g for g in m['gaps']) if m['gaps'] else '', m['poses'],
               m['window'], int(AM['PUSH_S'] * AM['RATE_HZ']), AM['RATE_HZ'], record[0][1]))

        if run is None:
            say('  the record refits to none of the printed runs (%s): it is some later run\'s, and the comparison '
                'below is against the printed numbers only' % ', '.join(CHAIN_LOGS))
        else:
            f = m['record_fit']
            say('  its refit times that run\'s inertia: K_d %.4f, K_s %.4f = %s (K_d %.4f), %s (K_s %.4f): the record '
                'of that run' % (f['D'] * run['I'], f['S'] * run['I'], run['where']['K_d_fin'], run['K_d_fin'],
                                 run['where']['K_s_fin'], run['K_s_fin']))

    if m['start'] is not None:
        k, err, u = m['start']
        say('  the model\'s stream started %+d steps (%.3f s) into its push, where its first row reads u %.5f '
            '(record %.5f, |error| %.5f m/s): the engine\'s own start, measured from its record'
            % (k, k * DT, u, record[0][1], err))
    else:
        say('  the model\'s stream started %.3f s into its push' % m['late'])

    say('  yaw torque %.3f N m on %s, %d poses at %.3f s -> inertia %.4f kg m^2 (SDF %.4f; engine %s)'
        % (m['torque'], m['rigid'], int(AM['YAW_S'] * AM['RATE_HZ']), m['interval'], m['I_fit'], m['I_sdf'],
           ', '.join('%.4f (%s)' % (e['I'], e['where']['I']) for e in engine_runs(m['rigid']))))
    say('  push (%.2f, %.2f, 0) N through the centre of mass: end u %.2f v %.2f m/s r %.3f rad/s, max sideslip %.1f deg'
        % (m['push'][0], m['push'][1], m['u_end'], m['v_end'], m['r_end'], m['beta']))
    say('  r against the integrals of u v and u r: K_s / I %.5f, K_d / I %.5f (intercept %.4f); times the inertia '
        'above: K_s %.4f, K_d %.4f N m s/rad per m/s' % (m['S'], m['D'], m['icpt'], m['K_s'], m['K_d']))

    if run is not None:
        f = m['record_fit']
        say('  inertia-free, against the engine\'s record on the same stream: K_d / I %.5f vs %.5f (%+.2f %%), '
            'K_s / I %.5f vs %.5f (%+.2f %%)' % (m['D'], f['D'], 100 * (m['D'] / f['D'] - 1), m['S'], f['S'],
                                                 100 * (m['S'] / f['S'] - 1)))

    if 'rows_diff' in m:
        rms = lambda xs: math.sqrt(sum(x * x for x in xs) / len(xs))
        d = m['rows_diff']
        peak = max(abs(x[3]) for x in record)
        say('  row by row on the same times since the push began (skew %.3f s): r rms %.4f, max %.4f deg/s (r reaches '
            '%.3f deg/s); u rms %.4f, v rms %.4f m/s'
            % (m['t_skew'], math.degrees(rms(d['r'])), math.degrees(max(abs(x) for x in d['r'])), math.degrees(peak),
               rms(d['u']), rms(d['v'])))

    for e in engine_runs(m['rigid']):
        say('  engine (%s, %s): K_d %.4f, K_s %.4f; model - engine: K_d %+.4f (%+.2f %%), K_s %+.4f (%+.2f %%)%s'
            % (e['where']['K_s_fin'], e['where']['K_d_fin'], e['K_d_fin'], e['K_s_fin'], m['K_d'] - e['K_d_fin'],
               100 * (m['K_d'] / e['K_d_fin'] - 1), m['K_s'] - e['K_s_fin'], 100 * (m['K_s'] / e['K_s_fin'] - 1),
               '  (the record\'s run)' if e is run else ''))

    if lates:
        band = start_band(m['model'], lates)
        ref = m['record_fit']['D'] if run is not None else m['D']
        ds = [b[1] for b in band]
        say('  the procedure\'s sensitivity: K_d / I with the stream starting %s s into the push: %s (%+.1f .. %+.1f %% '
            'of %s %.5f; a spread of %.1f %%)'
            % (' / '.join('%g' % b[0] for b in band), ' / '.join('%.4f' % x for x in ds),
               100 * (min(ds) / ref - 1), 100 * (max(ds) / ref - 1),
               'the record\'s' if run is not None else 'the model\'s', ref, 100 * (max(ds) - min(ds)) / ref))

    rigid = rigid_residual(m['rigid'])

    if rigid is not None:
        say('  the rigid hull\'s push (sitl_out/added_mass_%s.csv, no fins), against the model\'s on its stream and '
            'start (%+d steps): u rms %.4f (largest model - record %+.4f) m/s, v rms %.4f m/s; r at the end %+.3f deg/s '
            'in the record, %+.3f in the model (record\'s largest |r| %.3f): what the record carries outside the fins'
            % (m['rigid'], rigid['start'][0], rigid['u_rms'], rigid['u_max'], rigid['v_rms'],
               math.degrees(rigid['r_end'][0]), math.degrees(rigid['r_end'][1]), math.degrees(rigid['r_max'])))

    if scales and run is not None:
        lever = fin_leverage(m['model'], scales)
        say('  the comparison\'s leverage on the fins: their lift slope times %s gives K_d / I %s (%s %% of the record\'s) '
            'and r rms against the record %s deg/s'
            % (' / '.join('%g' % x[0] for x in lever), ' / '.join('%.5f' % x[1] for x in lever),
               ' / '.join('%+.2f' % (100 * (x[1] / m['record_fit']['D'] - 1)) for x in lever),
               ' / '.join('%.4f' % math.degrees(x[2]) for x in lever)))

    say('  files (fin_k x arm, added_mass_check.expected): %.4f; static K_s %.4f' % (m['files'], m['files_static']))
    say('  the replica linearised about the link origin (u 2 m/s, r 0.002 rad/s, fins only): %.4f' % m['linear_O'])


if __name__ == '__main__':
    name = sys.argv[1] if len(sys.argv) > 1 else 'airship_vectored_independent_fin'
    fin = measure(name)
    report(fin, lates=(0.0, 0.1, 0.2, 0.3, 0.45, 0.6), scales=(0.98, 1.0, 1.02))
    report_am_fin(measure_am_fin(gz_model.vehicle(name), fin))
