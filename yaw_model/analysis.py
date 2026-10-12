#!/usr/bin/env python3
"""Why the first closed-loop model missed, and what else was ruled out.

    python3 analysis.py        # writes closed_loop_out.txt

Every run here is gate.run_gate() on closed_loop.build(...) with one thing
changed. The records are ./records (see reproduce.py).
"""
import bisect
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import closed_loop as cl  # noqa: E402
import gate  # noqa: E402
import reproduce as rp  # noqa: E402

OUT = os.path.join(HERE, 'closed_loop_out.txt')
lines = []


def say(s=''):
    lines.append(s)
    print(s, flush=True)


def model_name(target):
    return (cl.TARGETS.get(target) or cl.EXTRA_TARGETS[target])[0]


def run(target, plant='roll', timing=None, trace=False, **kw):
    kw.setdefault('params', rp.record_params(target)[0])     # the records' own gains (record_gains)
    loop = cl.build(target, plant, trace=trace, **kw)
    res = gate.run_gate(loop, model_name(target), timing or gate.Timing())
    return rp.parse(res.gate_lines), res, loop


def records(target):
    out = {}
    for tag, path in rp.CHAINS:
        block = rp.chain_block(path, target)
        if block:
            out[tag] = rp.parse(block)
    return out


KEYS = [('wind check', 'end'), ('Manual', 'end'), ('Acro brake', 'start'), ('Acro brake', 'end'),
        ('Acro track', 'mean'), ('Acro track', 'peak'), ('Acro release', 'start'), ('Acro release', 'end'),
        ('Acro release', 'settle'), ('Acro release', 'slope'),
        ('Stab track', 'mean'), ('Stab track', 'peak'), ('Stab release', 'start'), ('Stab release', 'end'),
        ('Stab release', 'settle'), ('Stab release', 'slope'),
        ('wind I on', 'rate'), ('wind I on', 'integ'), ('wind I off', 'rate'), ('wind weak', 'rate'),
        ('wind weak', 'integ')]


def cell(parsed, key, field):
    rec = parsed.get(key)
    if rec is None:
        return '-'
    v = rec.get(field)
    if v is None or (isinstance(v, float) and not math.isfinite(v)):
        return '-' + ('' if rec.get('ok', True) else '*')
    s = ('%.4f' if field == 'integ' else '%.2f') % v
    return s + ('' if rec['ok'] else '*')


def table(title, target, columns):
    """columns: [(label, parsed)]; '*' marks a failed verdict."""
    say(title)
    say('  %-14s %-7s ' % ('check', 'field') + ''.join('%11s' % c[0] for c in columns))
    for key, field in KEYS:
        if all(key not in c[1] for c in columns):
            continue
        say('  %-14s %-7s ' % (key, field) + ''.join('%11s' % cell(c[1], key, field) for c in columns))


def trace_at(loop, t_sim):
    times = [tr[0] for tr in loop.trace]
    return loop.trace[min(bisect.bisect_left(times, t_sim), len(times) - 1)]


def phase_span(res, phase):
    rows = [r for r in res.samples if r['phase'] == phase]
    return rows[0]['sim'], rows[-1]['sim']


def main():
    say('Closed-loop model of the SITL yaw gate: reproduction summary and cause analysis')
    say('(numbers are the gate\'s own verdict-line numbers; * = that verdict failed; records: chain7_20260925.txt,')
    say(' chain9_20260927.txt; gains AS_YAWRATE_P 0.5, I 0.09, INT_LIM 0.3, MAX 15 deg/s, MAN_DEADZONE 0.1)')

    # ---------------------------------------------------------------- 1
    say('\n1. The three plant stages against the records (the controller and actuation are the same in all three)')
    say('   1dof   = plants.get(target, fins=1): I r_dot = N - c_hull r|r| - one fin broadside (the plant part)')
    say('   planar = + surge and sway, the added-mass Coriolis (Munk) moment, the fin plugin on the true fin velocity,')
    say('            the actuators as forces where they act (Cloudship tail force; the pods\' net surge)')
    say('   roll   = + roll: rigid ixz, buoyancy roll stiffness, the whole added-mass block (yp), roll damping rows,')
    say('            the pods\' vertical thrust and reaction couple in roll, horizontal fins (the default model)')
    for target in list(cl.TARGETS) + list(cl.EXTRA_TARGETS):
        recs = records(target)
        cols = [(t, recs[t]) for t in ('chain7', 'chain9') if t in recs]
        stages = ('roll',) if target in cl.EXTRA_TARGETS else ('1dof', 'planar', 'roll')
        for plant in stages:
            cols.append((plant, run(target, plant)[0]))
        table('\n  %s%s' % (target, '   (hold-out plant, not used to build anything)'
                            if target in cl.EXTRA_TARGETS else ''), target, cols)

    # ---------------------------------------------------------------- 2
    say('\n2. Cause of the 1dof misses, by switching single terms off in the full model')

    t = 'gz_cloudship_am_fin_drag'
    cols = [('chain7', records(t)['chain7']), ('chain9', records(t)['chain9']), ('roll', run(t)[0]),
            ('no Munk', run(t, munk=False)[0]), ('no fins', run(t, fins_enabled=False)[0]),
            ('planar', run(t, 'planar')[0]), ('pl.noMunk', run(t, 'planar', munk=False)[0]),
            ('pl.noTrans', run(t, 'planar', translation=False)[0])]
    table('\n  2a %s: the Acro tracking failure' % t, t, cols)
    say('     pl.noTrans = the planar plant with u, v held at 0 and one broadside fin: the 1dof plant again.')
    parsed, res, loop = run(t, trace=True)
    a0, a1 = phase_span(res, 'Acro 추종')
    say('     State of the hull during the Acro push (roll model; FRD rate; V = world drift of the hull origin;')
    say('     Munk = +(m_y - m_x) u v, the FRD yaw moment it adds, against the 0.5 x 4.478 = 2.239 N m of P per rad/s):')
    say('       %6s %8s %8s %8s %8s %8s %8s %7s %5s' % ('t', 'rate', 'torque', 'integ', 'u', 'v', '|V|', 'Munk', 'fins'))
    for k in range(0, 36, 2):
        tr = trace_at(loop, a0 + k)
        x = tr[-1]
        say('       %6.1f %8.2f %8.4f %8.4f %+8.4f %+8.4f %8.4f %+7.4f %5d'
            % (k, math.degrees(tr[3]), tr[4], tr[5], x['u'], x['v'], math.hypot(x['Vx'], x['Vy']),
               x['munk_frd'], x['fins_open']))
    say('     The hull enters the push already drifting at 0.15 m/s (the Manual phase\'s and the brake\'s tail FORCE;')
    say('     the rest wait stops only the yaw) and the push\'s own tail force adds to it. In a turn the drift')
    say('     rotates through the body at the yaw rate, so u v, and with it the Munk moment, goes through one')
    say('     cycle per 180 deg of heading (22-27 s at 8-6.7 deg/s). The rate follows: the Munk moment reaches')
    say('     twice the 0.076 N m it takes to hold 6.67 deg/s. Only one term does this: without it (no Munk) the')
    say('     model tracks like the 1dof plant; without the fins it still fails. The chain7 Stabilized push shows the')
    say('     same cycle: 7.59 at 132.2 s, 6.39 at 147.0 s, 7.30 again by 155.9 s. The drift never shows in the')
    say('     gate\'s columns; the gate logs no velocity.')

    t = 'gz_airship_vectored_independent_am_fin_drag'
    cols = [('chain7', records(t)['chain7']), ('chain9', records(t)['chain9']), ('roll', run(t)[0]),
            ('no surge', run(t, pod_surge=False)[0]), ('no fins', run(t, fins_enabled=False)[0]),
            ('no Munk', run(t, munk=False)[0]), ('1dof', run(t, '1dof')[0]), ('1dof f2', run(t, '1dof', fins=2.0)[0])]
    table('\n  2b %s: tracking 6.52 in the 1dof model against 6.23 recorded' % t, t, cols)
    parsed, res, loop = run(t, trace=True)
    for ph in ('Manual 통과', 'Acro 추종'):
        b0, b1 = phase_span(res, ph)
        say('     %s:' % ph)
        for k in (0.0, 0.5, 1.0, 1.5, 2.0, 3.0) if ph.startswith('Manual') else (0, 5, 10, 20, 34):
            tr = trace_at(loop, b0 + k)
            x = tr[-1]
            say('       t+%-5.1f rate %6.2f  u %+.4f v %+.4f m/s  vertical fins open %d' %
                (k, math.degrees(tr[3]), x['u'], x['v'], x['fins_open']))
    say('     The 1dof plant fixes the fin share at one. In the runs both vertical fins act from the Manual phase')
    say('     on, because the hull drifts forward (u > 0) and LiftDrag.cc:312 gives a fin nothing only when its')
    say('     flow comes from behind. Two sources of that drift, both in the model: the pods\' net surge while one')
    say('     of them swings (from rest the port pod thrusts at once while the starboard pod turns through 90 deg,')
    say('     vectored_actuation.py regime A: u 0.19 m/s by the end of the Manual step), and, with that switched')
    say('     off ("no surge"), the fins themselves: an open fin\'s side force makes sway, sway times yaw rate is')
    say('     surge through the Coriolis terms, and the forward drift then opens the other fin (later, in the Acro')
    say('     push). The surge drag (xUabsU 0.2748) hardly decays it. 1dof with two fins ("1dof f2") gets the')
    say('     pushes and releases but not the I-on wind case (0.14 against 0.20-0.22 deg/s): there the flow at the')
    say('     fins is the drift\'s, and the fin damping is the attached-flow K_d u r, not a broadside r|r|.')

    t = 'gz_airship_vectored_independent'
    rec9 = rp.read_series(rp.SERIES[(t, 'chain9')])
    variants = [('roll', {}), ('ixz 0', dict(ixz=False)), ('ixz -1', dict(ixz=-1.0)),
                ('pod roll 0', dict(pod_roll=False)), ('planar', dict(plant='planar'))]
    cols = [('chain7', records(t)['chain7']), ('chain9', records(t)['chain9'])]
    rms = []

    for label, kw in variants:
        kw = dict(kw)
        parsed, res, _ = run(t, kw.pop('plant', 'roll'), **kw)
        cols.append((label, parsed))
        d = []
        for r in rp.series_residuals(res.samples, rec9):
            if r['rate'] and r['phase'].split(' #')[0] in ('Manual 통과', 'Acro 제동', 'Acro 추종', 'Acro 놓음',
                                                           'Stabilized 추종', 'Stabilized 놓음'):
                d.append((r['rate'][0], r['n']))
        rms.append('%s %.2f' % (label, math.sqrt(sum(x * x * n for x, n in d) / sum(n for _, n in d))))

    table('\n  2c %s: peaks 0.3-0.4 low and a +-0.4 deg/s ripple in the planar model' % t, t, cols)
    say('     rate rms against the chain9 series over Manual, brake, both pushes and releases [deg/s]: ' + ', '.join(rms))
    say('     The rigid hull has no roll damping; buoyancy makes roll a 1.66 s pendulum (K 576 N m/rad, Ixx 43.3,')
    say('     ixz 20.1 kg m^2: coupling.py A; chain9_20260927.txt:69 measured 1.659 s). A swinging pod\'s thrust')
    say('     has a vertical part at 1.5 m from the centre line, a roll kick at every swing (pod roll 0 removes the')
    say('     ripple), and ixz feeds the roll swing into the body yaw rate the gyro reads (ixz 0 removes it too).')
    say('     The max-of-samples peak and the one-row brake reading pick the ripple up; the means do not.')
    say('     ixz -1 (the other sign convention for the SDF <ixz>) fits worse, which supports reading <ixz> as the')
    say('     inertia-matrix element (SDF spec; coupling.products adds -m dx dz accordingly).')

    # ---------------------------------------------------------------- 3
    say('\n3. The other candidate causes, each tried on the full model (verdicts and the numbers that moved most)')
    base = {t: run(t)[0] for t in cl.TARGETS}

    def moved(target, parsed):
        diffs = []
        for key, field in KEYS:
            a, b = base[target].get(key, {}).get(field), parsed.get(key, {}).get(field)
            if a is not None and b is not None and math.isfinite(a) and math.isfinite(b) and abs(a - b) > 0:
                f = '%.4f' if field == 'integ' else '%.2f'
                diffs.append((abs(a - b) * (100 if field == 'integ' else 1),
                              ('%s %s ' + f + '->' + f) % (key, field, a, b)))
        flips = [k for k in base[target] if k in parsed and base[target][k]['ok'] != parsed[k]['ok']]
        diffs.sort(reverse=True)
        return flips, [d[1] for d in diffs[:3]]

    cases = [
        ('RTF 0.98 (chain9 vectored wind-check slope 0.980 +- 0.010, validate_gate.py)', dict(timing=gate.Timing(rtf=0.98))),
        ('RTF 1.02', dict(timing=gate.Timing(rtf=1.02))),
        ('stick delay 0 s (20 Hz stream: 0-50 ms, sitl_link.py:283-299)', dict(timing=gate.Timing(stick_delay_s=0.0))),
        ('stick delay 0.05 s', dict(timing=gate.Timing(stick_delay_s=0.05))),
        ('gyro noise on (0.00087 rad/s, model.sdf)', dict(noise=True)),
        ('one extra 4 ms transport step (vectored only)', dict(transport_ticks=1)),
    ]
    for label, kw in cases:
        say('  %s' % label)
        for target in cl.TARGETS:
            if kw.get('transport_ticks') and 'cloudship' in target:
                continue
            parsed = run(target, **kw)[0]
            flips, top = moved(target, parsed)
            say('    %-44s verdict flips: %-4s largest moves: %s' % (target, ', '.join(flips) or 'none', '; '.join(top)))

    say('\n  Saturation / anti-windup and the integrator resets are in the model, not candidates for a fix:')
    say('  the recorded integrator column is reproduced in every closed-loop phase (reproduction_out.txt, integ')
    say('  rms <= 0.0012 against a range of 0.1): the pod-swing saturation flags that freeze it, the reset when')
    say('  Manual opens the loop (every "출발" row starts at |integ| <= 0.001), and the carry-over from Stabilized to')
    say('  Acro before the wind cases (Cloudship complete: the I-on case starts from the release\'s -0.021).')
    say('  Sampling is the gate\'s own: gate.py runs sitl_check.main() with its wall-clock windows.')

    # ---------------------------------------------------------------- 4
    say('\n4. What the model still misses, and why (no verdict disagrees; no number is off by more than the gate\'s')
    say('   own tolerance for it; reproduction_out.txt flags the numbers further off than the two SITL runs are')
    say('   from each other)')
    for target in ('gz_cloudship_am_fin_drag', 'gz_airship_vectored_independent_fin'):
        parsed, res, loop = run(target, trace=True)
        w0 = [e for e in res.events if 'wrench 1' in e[2]][0][1]
        opened = [trace_at(loop, w0 + k)[-1]['fins_open'] for k in (1.0, 2.0, 3.0, 4.0, 4.7)]
        tr = trace_at(loop, w0 + 4.7)[-1]
        say('  %-44s first wind check %.2f deg/s (records: %s); vertical fins open at +1..4.7 s: %s; u %+.4f m/s at +4.7 s'
            % (target, parsed['wind check']['end'],
               ', '.join('%s %.2f' % (t, r['wind check']['end']) for t, r in records(target).items()),
               opened, tr['u']))
    say('  a) The first wind check on a finned plant is the one phase that starts from exact rest. There the')
    say('     fin plugin\'s gate (flow from behind -> nothing, LiftDrag.cc:312) is decided by sub-mm/s surge')
    say('     drift and the roll angle. On the complete Cloudship the model opens the second fin by +2 s, which')
    say('     gz does not; the two gz runs disagree with each other too (-19.87, -20.01). Held here: heave and pitch')
    say('     (w = 0, theta = 0); the Cloudship\'s buoyancy sits 0.022 m aft of its centre of mass')
    say('     (gz_model.centre_of_volume vs centre_of_mass), a ~2.5 deg nose-down trim the model does not')
    say('     have; whether that is what decides the at-rest drift is untested. From the Manual phase on, the')
    say('     actuators\' own drift (0.15-0.33 m/s) swamps it.')
    say('  b) gz_cloudship Manual end / brake start (0.5-0.7 deg/s): those rows sample a 22 deg/s^2 ramp, so')
    say('     20-30 ms of harness timing moves them that far (section 3: stick delay 0 vs 50 ms moves them 1.1')
    say('     deg/s); chain7 and chain9 differ by 0.5-0.6 on the same two numbers.')
    say('  c) The rigid vectored ripple (section 2c) is modelled, but its phase over 100+ s is not a stable')
    say('     quantity: the Acro push row rms stays at 0.21 deg/s while the Stabilized push is 0.04.')

    with open(OUT, 'w') as handle:
        handle.write('\n'.join(lines) + '\n')
    print('\nwritten', OUT)


if __name__ == '__main__':
    main()
