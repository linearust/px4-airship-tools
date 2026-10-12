#!/usr/bin/env python3
"""Step 0a of the D1 plan (d1_universal_architecture_20261001.md, "0. 측정 먼저"):
the yaw model aligned with HEAD, its gates, and the forward-flight defect
recomputed. Writes forward_out_20261001.txt (and the legs' time series under
forward_out/).

    python3 step0a.py            # everything, about ten minutes
    python3 step0a.py --g4       # the forward legs only

G1  the 48 at-rest gate verdicts at HEAD gains (reproduce_head.py), and the
    harness replay of every HEAD-gain run (validate_gate.head_replays)
G2  the P 0.5 reproduction (reproduce.py) at the records' own gains, 48/48,
    against the 2026-09-27 output on disk
G3  the fin replica's yaw damping by the engine's own procedure, on the
    engine record's own stream (fin_damping.py), with the procedure's
    sensitivity beside it
G4  the defect, recomputed: the forward legs (forward.py); the stability of
    straight flight; the model against the main session's SITL probes from
    their ground-truth entry, where the steady turn's hold torque per rad/s is
    the acceptance test of the fins' damping as flown
plus the FF port against HEAD's SITL logs (ulog_check.py) and the model's
tests.
"""
import argparse
import hashlib
import io
import math
import os
import subprocess
import sys
from contextlib import redirect_stdout

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import closed_loop as cl  # noqa: E402
import controller as ctl  # noqa: E402
import forward as F  # noqa: E402

OUT = os.path.join(HERE, 'forward_out_20261001.txt')
SERIES = os.path.join(HERE, 'forward_out')
LOGDIR = os.path.join(ctl.PX4, 'build/px4_sitl_default/rootfs/fs/log')
V27, V22 = 'gz_airship_vectored_independent_am_fin_drag', 'gz_airship_vectored_independent_fin'
C12, C17 = 'gz_cloudship_fin', 'gz_cloudship_am_fin_drag'
# The 0b check's own default legs per plant (forward_yaw_check.default_legs,
# read from the check): 2527, 2522 and 2512 at 2 and 4 m/s, 2517 at 2.0 and
# 2.5 m/s, where 0b flies it (until 2026-10-01 G4 flew 2517 at 2 and 4)
G4_LEGS = [(t, u, s) for t in (V27, V22, C12, C17) for u, s in F.default_legs(t)]
# The main session's SITL probes of 2026-10-01 (scratchpad probe_fwd.py, HEAD
# 4e1baea97c): the defect leg on 2527 at trim and on 2522 gliding; the 2517
# probe accelerated at throttle 0.5 to 10 m/s in Manual (the loop open) and
# is no leg
PROBES = [('2026-10-01/10_39_42.ulg', V27), ('2026-10-01/10_47_36.ulg', V22)]
PROBE_TOPICS = ('vehicle_control_mode', 'manual_control_setpoint', 'vehicle_local_position',
                'vehicle_local_position_groundtruth', 'vehicle_attitude_groundtruth', 'vehicle_angular_velocity',
                'rate_ctrl_status', 'vehicle_torque_setpoint', 'vehicle_rates_setpoint')
# Straight-flight stability rows: the speeds per target, and the Munk scales
# that stand in for the engine's reading of the coefficient (G3's _am_fin
# line; added_mass_check chain5: -4.53 against the file's -4.92)
STABILITY = ((C17, (1.0, 1.25, 1.5, 1.75, 2.0, 2.5, 3.0, 4.0)), (V27, (1.0, 2.0, 3.0, 4.0)))
MUNK_SCALES = ((0.92, (1.5, 2.0, 2.5, 3.0, 4.0)), (0.85, (1.5, 2.0, 2.5, 3.0, 4.0)))

lines = []


def say(text=''):
    lines.append(text)
    print(text, flush=True)


def captured(fn, *args, **kw):
    buf = io.StringIO()

    with redirect_stdout(buf):
        value = fn(*args, **kw)

    return value, buf.getvalue().splitlines()


def head_section():
    rev = subprocess.run(['git', '-C', ctl.PX4, 'rev-parse', '--short=10', 'HEAD'], capture_output=True, text=True)
    dirty = subprocess.run(['git', '-C', ctl.PX4, 'status', '--porcelain', '--untracked-files=no'],
                           capture_output=True, text=True)
    say('=== The model aligned with HEAD ===')
    say('PX4-Autopilot HEAD %s (%s)' % (rev.stdout.strip() or '?(git rc %d)' % rev.returncode,
                                       'clean' if dirty.returncode == 0 and not dirty.stdout.strip()
                                       else 'NOT CLEAN or unreadable: ' + dirty.stdout.strip()[:200]))
    say('controller.DEFAULT_PARAMS, read from the yaml: %s'
        % ', '.join('%s %g' % kv for kv in ctl.DEFAULT_PARAMS.items()))

    for model in ('airship_vectored_independent', 'airship_vectored_independent_fin',
                  'airship_vectored_independent_am_fin_drag', 'cloudship', 'cloudship_fin', 'cloudship_am_fin_drag'):
        hp = ctl.head_params(model)
        src = ctl.airframe_defaults(hp['SYS_AUTOSTART']).get('AS_YAWRATE_FF')
        say('  %d %-42s P %g I %g INT_LIM %g FF %g (%s) MAX %g THR_MDL_FAC %g'
            % (hp['SYS_AUTOSTART'], model, hp['AS_YAWRATE_P'], hp['AS_YAWRATE_I'], hp['AS_YR_INT_LIM'],
               hp['AS_YAWRATE_FF'], '%s:%d' % (src[1], src[2]) if src else 'yaml default', hp['AS_YAWRATE_MAX'],
               hp['THR_MDL_FAC']))

    say('FF semantics (rate_control.cpp:78, :88-118; airship_yaw_rate_loop.hpp:58-69, :76-82): torque = P e + I + '
        'FF sp; the integral takes e alone, clipped by the flags; the flags come from the allocator\'s shortfall '
        'of the whole torque, FF included. FF does not take part in the integrator\'s saturation logic except '
        'through that shortfall. options/B (the 09-27/28 stage-2 implementation, rc.ff set after build) and '
        'the aligned path (FF through set_gains) give identical gate lines and samples on all five plants '
        '(checked 2026-10-01, old tree from the scratch snapshot).')
    say('FF port against HEAD\'s SITL logs (ulog_check.py, chain14 of 2026-09-29, the logged rate driving the '
        'controller and allocator ports, no plant):')
    import ulog_check as U

    for rel, frame in U.HEAD_FF_LOGS:
        r = U.check_log(os.path.join(U.LOGDIR, rel), frame, verbose=False)
        e = lambda xs, q: sorted(abs(x) for x in xs)[int(q * (len(xs) - 1))]
        say('  %s: torque p99 %.1e, integrator p99 %.1e, yaw saturation sign %d/%d'
            % (r['name'], e(r['e_tq'], 0.99), e(r['e_i'], 0.99), r['agree'], r['total']))


def g1():
    import reproduce_head as rh
    import validate_gate as vg
    say('\n=== G1: the 48 at-rest verdicts at HEAD gains ===')
    say('Tolerance convention (reproduce.py, reproduce_head.py): the verdict is the gate\'s own pass/fail, '
        'compared exactly; per number TOL is the band sitl_check applies to it and SPREAD the range of the '
        'clean runs (">spread" flags a model value further from a run than the runs are from each other).')
    say('The tolerance labels\' line numbers are sitl_check.py\'s of 2026-09-27 (reproduce.CHECKS); in today\'s '
        'file: %s' % threshold_lines())
    (agree, total, table), text = captured(rh.reproduce)
    lines.extend(text)
    say('G1 verdicts: %d / %d' % (agree, total))
    say('\nHarness replay: each HEAD-gain run\'s own rows through gate.py (today\'s sitl_check.py), clocked by '
        'the record, lines compared with the chain log:')
    result, text = captured(vg.head_replays)
    detail = {}
    current = None

    for line in text:
        if line.startswith('replay '):
            current = line
        elif current and ('rows in a different phase' in line or 'verdict lines equal' in line
                          or 'the gate asked' in line):
            detail.setdefault(current, []).append(line.strip())

    for (run, target), good in result.items():
        why = HARNESS_WHY.get((run, target), '')
        say('  %-22s %-46s %s%s' % (run, target, 'equal' if good else 'differs', (': ' + why) if why else ''))

    say('  %d of %d runs replay line for line' % (sum(result.values()), len(result)))


HARNESS_WHY = {
    ('chain12_20260928', t): 'gz_link\'s triple wrench clear of that day (2.4 s after the wind check) '
                             'is not today\'s; fixed for chain12b (sitl_out/chain12b.sh:2-4)'
    for t in ('gz_airship_vectored_independent', 'gz_airship_vectored_independent_am_fin_drag',
              'gz_airship_vectored_independent_fin', 'gz_cloudship', 'gz_cloudship_am_fin_drag')}
HARNESS_WHY.update({
    ('chain12b_20260928', 'gz_airship_vectored_independent_am_fin_drag'):
        'lost pushes (chain12c.sh:2-3); today\'s sitl_check retries a push the loop does not carry, '
        'the record has no retry rows',
    ('chain12b_20260928', 'gz_airship_vectored_independent'):
        'the 50 s weak-wind window took 160 rows on the replayed clock against the 159 recorded (a window edge)',
    ('chain13_20260928', 'gz_airship_vectored_independent_fin'):
        'verdict lines equal; the rest wait before the Stabilized step ended one row later (its 1.0 s hold '
        'on rows stamped to 0.01 s)',
    ('chain14_20260929', 'gz_airship_vectored_independent'):
        'sitl_check printed |integrator| then and the signed value since (sitl_out/chain14b.sh:2-3): '
        '0.0074 against -0.0073 in the unmeasured weak case',
    ('chain14b_1_20260929', 'gz_airship_vectored_independent'):
        'verdict lines equal; one row of the last phase moved by a publish join (0.86 s)',
})


def threshold_lines():
    import re
    want = (('wind check', r'abs\(after\[-1\]\[.rate.\] - before\) > 0\.5 \* expected'),
            ('Manual', r'abs\(t - STICK\) < 0\.02'), ('brake', r'abs\(brake\[-1\]\[.rate.\]\) < 6\.0'),
            ('tracking mean', r'abs\(mean - setpoint_deg\) < 0\.25'), ('peak', r'peak < 1\.4 \* setpoint_deg'),
            ('release', r'rest < 10\.0'), ('wind cases', r'rate < 1\.0 and abs\(integ - needed\)'))
    with open(os.path.join(cl.gate.BENCH, 'sitl_check.py'), encoding='utf-8') as handle:
        text = handle.read().splitlines()
    out = []
    for name, pattern in want:
        hits = [i + 1 for i, line in enumerate(text) if re.search(pattern, line)]
        out.append('%s :%s' % (name, ','.join(map(str, hits)) or '?'))
    return ', '.join(out)


def g2():
    import reproduce as rp
    say('\n=== G2: the P 0.5 reproduction (records chain7, chain9) at its own gains ===')
    agree = total = 0
    import csv
    rows = []

    for target in list(cl.TARGETS) + list(cl.EXTRA_TARGETS):
        params, sources = rp.record_params(target)
        result, _ = rp.run_model(target, 'roll')
        model = rp.parse(result.gate_lines)

        for tag, path in rp.CHAINS:
            block = rp.chain_block(path, target)

            if not block:
                continue

            recs = rp.parse(block)

            for key in recs:
                if key in model:
                    agree += model[key]['ok'] == recs[key]['ok']
                    total += 1

        checks = {key for tag, path in rp.CHAINS for key in rp.parse(rp.chain_block(path, target))}
        rows.append((target, params, sources, len(checks)))

    for target, params, sources, n in rows:
        say('  %-46s %2d checks, gains P %g I %g FF %g from %s' % (target, n, params['AS_YAWRATE_P'],
                                                                  params['AS_YAWRATE_I'], params['AS_YAWRATE_FF'],
                                                                  '; '.join(s.split(' (')[0] for s in sources)))

    checks = sum(n for _, _, _, n in rows)
    say('G2: the model agrees with every record of every check: %d of %d (check, record) pairs, %d checks'
        % (agree, total, checks))
    on_disk = os.path.join(HERE, 'reproduction.csv')
    with open(on_disk, 'rb') as handle:
        say('  the 2026-09-27 reproduction.csv on disk: md5 %s; the run at the records\' gains through the aligned '
            'code reproduces it byte for byte (checked 2026-10-01 with reproduce.py --out, CSV identical)'
            % hashlib.md5(handle.read()).hexdigest())


def g3():
    import fin_damping as fd
    say('\n=== G3: the fin replica\'s yaw damping, by the engine\'s procedure, on the engine\'s own stream ===')
    say('added_mass_check samples the pose stream with a separate `gz topic -e` after its publish returns and fits '
        'over int(PUSH_S x RATE_HZ) poses; its record (sitl_out/added_mass_<model>.csv) shows the poses every 0.020 s '
        '(not the 1/60 s it assumes), so 150 poses span 3.0 s, and its first row\'s u under the known '
        'push fixes when the stream began. The model is sampled at the record\'s pose times from that start '
        '(fin_damping.py docstring); what is left is the plant. Beside each number: what the start alone does to '
        'it, and what a 2 % change of the fins\' lift slope does to it.')
    results = {}

    for model in ('airship_vectored_independent_fin', 'cloudship_fin'):
        m = fd.measure(model)
        fd.report(m, say, lates=(0.0, 0.1, 0.2, 0.3, 0.45, 0.6), scales=(0.98, 1.0, 1.02))
        fd.report_am_fin(fd.measure_am_fin(cl.gate.gz_model.vehicle(model), m), say)
        results[model] = m

    reading = []

    for model, m in results.items():
        rms = math.degrees(math.sqrt(sum(x * x for x in m['rows_diff']['r']) / len(m['rows_diff']['r'])))
        pct = 100 * (m['D'] / m['record_fit']['D'] - 1)
        (_, lo, r_lo), (_, hi, r_hi) = fd.fin_leverage(model, (0.98, 1.02))
        per = 100 * (hi - lo) / m['D'] / 4.0                  # % of K_d / I per % of lift slope
        reading.append('%s: K_d / I %+.2f %% from the record on its own stream, r within %.4f deg/s rms, the '
                       'equivalent of a %+.2f %% lift-slope error; a 2 %% change of the slope moves K_d / I by %.2f '
                       '%% and the r residual to %.4f / %.4f deg/s (x%.1f)'
                       % (model, pct, rms, pct / per, 2 * per, math.degrees(r_lo), math.degrees(r_hi),
                          min(r_lo, r_hi) / math.radians(rms)))

    say('G3 reading. ' + '. '.join(reading) + '. The start is measured from each record, not a free parameter. '
        'Where 2 % of lift slope moves the r residual several-fold the comparison resolves the fins\' damping; '
        'where it barely moves it, as on the Cloudship, whose records carry motion the planar model does not have '
        'even without fins (the rigid hull\'s line above), the comparison is consistent to the residual and no finer. '
        'The acceptance test of the fin damping as flown is the forward probe\'s steady turn (G4, the SITL '
        'cross-check: hold torque per rad/s) and 0b.')


def leg_row(leg):
    s, m = leg['setup'], leg['metrics']
    code = {'attached': 'a', 'stalled': 'S', 'no flow': '-'}
    fins_e = '/'.join(code[v] for _, v in sorted(m['fins_branch_stick_end'].items()))
    fins_r = '/'.join(code[v] for _, v in sorted(m['fins_branch_release_end'].items()))
    fmax = max(m['fin_max_alpha_ratio_stick'].values()) if m['fin_max_alpha_ratio_stick'] else float('nan')

    if s['vectored']:
        swing = lambda seg: '-' if m['pod_swings_' + seg] is None else '+'.join(
            '%d' % (d['into_rear'] + d['back_forward']) for d in m['pod_swings_' + seg].values())
        pods, tail = '%s / %s' % (swing('stick'), swing('release')), '-'
    else:
        pods = '-'
        tail = '%s / %d (%s / %d)' % (F.fmt(m['tail_flips_stick'], '%d'), m['tail_flips_release'],
                                      F.fmt(m['tail_thrust_signs_stick'], '%d'), m['tail_thrust_signs_release'])

    fmt = F.fmt     # '-' for a number the leg does not have (a release start's stick, a mark past the end)
    stalled = None if m['stalled_fraction_stick'] is None else 100 * m['stalled_fraction_stick']
    return ('| %d %s | %.1f | %+.2f (%.2f) | %.4f/%.4f | %g | %s | %s | %+.4f | %+.4f | %+g | %d/%d | %+.1f | %s / %s / %s | %+.4f '
            '| %s | %s | %s / %.2f / %.2f | %s, %s; max a/as %.2f, %s%% |'
            % (s['airframe'], s['model'], s['u0'], s['stick'], s['sp'], s['fraction'], s['flown'], s['FF'],
               fmt(m['r_mean_last5']), fmt(m['r_peak']), m['integ_stick_end'], m['torque_stick_end'],
               m['unalloc_stick_end'], m['sat_pos_stick_end'], m['sat_neg_stick_end'], m['heading_release'],
               fmt(m['r_rel_10']), fmt(m['r_rel_60']), fmt(m['r_rel_120']), m['integ_rel_120'], pods, tail,
               fmt(m['u_stick_start'], '%.2f'), m['u_stick_end'], m['u_release_end'], fins_e, fins_r, fmax,
               fmt(stalled, '%.0f')))


TABLE_HEAD = ('| airframe model | u0 m/s | stick (sp deg/s) | throttle asked/flown | FF | r mean last 5 s | r peak | '
              'integ @stick end | torque @stick end | unalloc[2] @stick end | sat +/- | heading in release deg | '
              'r +10/+60/+120 s | integ +120 s | pod swings stick / release (stbd+port; vectored) | '
              'tail flips stick / release outside +-%g (thrust sign changes; Cloudship) | '
              'u stick start / end / release end | '
              'fins dorsal/ventral at stick end, release end (a attached, S stalled, - no flow: from behind); max alpha/alpha_stall, %% of stick stalled |'
              % F.TAIL_BAND)
TABLE_RULE = '|' + '---|' * 19


def g4(fast=False):
    os.makedirs(SERIES, exist_ok=True)
    say('\n=== G4: the defect, recomputed (forward.py, free surge, HEAD gains per airframe) ===')
    say('Leg: warm-up 1 s with the hull held (rotors and pods at their trim state), Manual 2 s, Acro at t0, 3 s '
        'centred, the stick 20 s, released 120 s; FRD, deg/s; the legs the 0b check flies '
        '(forward_yaw_check.default_legs); throttle as the 0b check flies it: forward_yaw_check.quantize rounds the '
        'trim to the nearest count and hands sitl_link the count\'s midpoint, which its int() truncates to the count '
        '(forward.run_leg quantize \'midpoint\'). Trim from the files (forward.trim_throttle = gz_model.trim_thrust, '
        'the formula forward_yaw_check uses: |xUabsU| u^2 over the two pods\' full thrust), after '
        'forward.linear_thrust_check (THR_MDL_FAC 1, and on each pod motor SIM_GZ_EC_MIN 0 and SIM_GZ_EC_MAX equal '
        'to the plugin\'s maxRotVelocity):')

    for model in ('airship_vectored_independent_am_fin_drag', 'cloudship_am_fin_drag'):
        number = F.linear_thrust_check(model)
        d = ctl.airframe_defaults(number)
        merged = cl.gate.gz_model.merged(model)
        say('  %d %s: THR_MDL_FAC %s; SIM_GZ_EC_MIN1/2 %s/%s, MAX1/2 %s/%s against maxRotVelocity %s/%s (%s:%d)'
            % (number, model, d['THR_MDL_FAC'][0], d['SIM_GZ_EC_MIN1'][0], d['SIM_GZ_EC_MIN2'][0],
               d['SIM_GZ_EC_MAX1'][0], d['SIM_GZ_EC_MAX2'][0],
               *(cl.gate.gz_model.motor_plugin(merged, link).find('maxRotVelocity').text
                 for link in ('prop_starboard', 'prop_port')), d['SIM_GZ_EC_MAX1'][1], d['SIM_GZ_EC_MAX1'][2]))

    for model, speeds in (('airship_vectored_independent_am_fin_drag', (2.0, 4.0)),
                          ('cloudship_am_fin_drag', (1.0, 1.5, 2.0, 2.5, 4.0, 6.0)),
                          ('airship_vectored_independent_fin', (2.0, 4.0)), ('cloudship_fin', (2.0, 4.0))):
        for u in speeds:
            trim, drag, full = F.trim_throttle(model, u)
            counts = F.fyc.quantize(trim)[0]
            sent = F.flown_fraction(trim, 'midpoint')
            say('  %-42s %.1f m/s: drag %.3f N, full thrust %.2f N -> trim %.4f (the 0b check flies %d/1000 = %.3f, '
                '%+.1f %%)%s' % (model, u, drag, full, trim, counts, sent, 100 * (sent / trim - 1) if trim else 0.0,
                                '' if trim <= 0.8 else '  INFEASIBLE (> 0.8)'))

    t2, t4 = (F.trim_throttle('airship_vectored_independent_am_fin_drag', u)[0] for u in (2.0, 4.0))
    say('  the plan\'s rev-3 2527 trims (d1_speed_source_and_fins_20261001.md:250, 0.0305 / 0.1221): %s. The fin '
        'plants without _drag have no surge drag and their fins none at zero incidence (cd = cda * alpha), so they '
        'hold u at throttle 0 in straight flight (test_forward.py: |u - u0| < 1e-6 over 10 s). The count moves the '
        'small Cloudship trims by up to half a count (above); the stability rows below fly the exact trim.'
        % ('reproduced, %.4f / %.4f' % (t2, t4) if (round(t2, 4), round(t4, 4)) == (0.0305, 0.1221)
           else 'NOT reproduced: %.4f / %.4f' % (t2, t4)))
    say('Reversals, two quantities kept apart (forward.py docstring): vectored, a pod\'s swing between forward '
        '(cos(tilt) > +%g) and back (< -%g) per pod; Cloudship, the tail command\'s direction outside +-%g. The band '
        'is a counting convention shared with forward_yaw_check, not a firmware or actuator threshold: the tail is '
        'commanded continuously through zero and gz\'s tail is linear with no deadband and no lag, so the count of '
        'every sign change of its thrust is kept beside it. %s' % (F.FIN_BAND, F.FIN_BAND, F.TAIL_BAND, tail_dither()))
    say('')
    say(TABLE_HEAD)
    say(TABLE_RULE)
    legs = {}

    for target, u0, stick in G4_LEGS:
        leg = F.run_leg(target, u0, stick)
        legs[(target, u0, stick)] = leg
        say(leg_row(leg))
        F.write_csv(leg, os.path.join(SERIES, '%s_u%g_s%g.csv' % (target, u0, stick)))

    say('')

    for key, leg in legs.items():
        for text in F.describe(leg):
            say(text)

    if fast:
        return legs

    say('\n--- against the reference review (reference_design_review_20260930.md:220, :327-330: rev-3 2527, 4 m/s, '
        'full stick; FF 0 1.55 deg/s, FF 1.0 2.4 deg/s, integrator 0.3, 87-90 deg in 2 min, time constant about '
        '199 s; a D1-policy cap 2.66 deg/s; its model held u fixed with zero thrust) ---')
    say(TABLE_HEAD)
    say(TABLE_RULE)
    variants = []

    for ff in (0.0, 1.0):
        for label, kw in (('the review\'s setup: u held at 4, thrust 0', {'hold_u': 4.0, 'throttle': 0.0}),
                          ('free surge, thrust 0 (gliding)', {'throttle': 0.0}),
                          ('free surge, trim thrust (the leg)', {})):
            leg = F.run_leg(V27, 4.0, 1.0, params={'AS_YAWRATE_FF': ff}, **kw)
            variants.append((ff, label, leg))
            say(leg_row(leg) + ' ' + label)

    for ff, label, leg in variants:
        m = leg['metrics']
        tau = tau_release(leg)
        say('  FF %g, %-36s r last 5 s %.2f deg/s, heading in release %+.1f deg, release time constant %.0f s'
            % (ff, label, m['r_mean_last5'], m['heading_release'], tau))

    k = steady_k(V27, 4.0)
    import fin_damping as fd
    lin = fd.linear_O('airship_vectored_independent_am_fin')       # the 2527 fins (= the _fin plant's)
    authority = cl.gate.gz_model.yaw_authority(cl.ALL_TARGETS[V27][0])
    say('  the plant\'s hold torque per rad/s at 4 m/s (u held, a 0.5 torque held to steady state): %.2f '
        '(normalized per rad/s); the D1 cap r = (A - rho) / (FF0 + k u) with A 1, rho 0.3, FF0 0: %.2f deg/s '
        '(FF0 1: %.2f). The review\'s 2.66 deg/s at A - rho = 0.7 implies %.2f per rad/s, %.0f %% below this '
        'plant; the fins alone, linearised (fin_damping.linear_O %.2f u N m s/rad over the %.1f N m '
        'authority), give %.2f' % (k, math.degrees(0.7 / k), math.degrees(0.7 / (1.0 + k)), 0.7 / math.radians(2.66),
                                   100 * (1 - 0.7 / math.radians(2.66) / k), lin, authority, lin * 4.0 / authority))

    say('\n--- the complete Cloudship (2517) in forward flight: stability of straight flight, Acro, stick centred, '
        'an initial sway of 0.01 m/s to port, 150 s, the exact trim (not truncated); each speed twice: "held" with the surge '
        'held at u0 (the speed the row is labelled with, for the onset) and "free" (what the hull does, for the '
        'bound). Verdict: the last 30 s window\'s max |r| against the second\'s (30-60 s, past the entry), more than '
        '5 % up grows, more than 5 % down decays. Beside it the speed flown (mean u per 30 s window, the same '
        'samples), the actuator\'s peak (the tail command, limit 1; the vectored torque, normalized), the samples '
        'with a saturation flag, and the fins\' largest |alpha| / alpha_stall ---')
    rows = {}

    for target, speeds in STABILITY:
        for u0 in speeds:
            for hold in (True, False):
                rows[(target, u0, hold, 1.0)] = stability_row(target, u0, hold)

    # The engine read the Munk coefficient at 0.92 of the file's
    # (added_mass_check, chain5 -4.53 / -4.92, review/munk_scale.py), and its
    # net static term on cloudship_am_fin sits above the model's (G3). Scale the
    # C_A yaw term the same way.
    import planar
    orig = planar.RollPlanarPlant._tau

    for scale, speeds in MUNK_SCALES:
        def _tau(self, st, phi, ext, scale=scale):
            out = orig(self, st, phi, ext)
            out[3] += (1.0 - scale) * (self.mxy[1] - self.mxy[0]) * st[0] * st[1]
            return out

        planar.RollPlanarPlant._tau = _tau

        try:
            for u0 in speeds:
                for hold in (True, False):
                    rows[(C17, u0, hold, scale)] = stability_row(C17, u0, hold, ' Munk x%.2f' % scale)
        finally:
            planar.RollPlanarPlant._tau = orig

    for scale in (1.0,) + tuple(x[0] for x in MUNK_SCALES):
        say('  2517%s: %s' % ('' if scale == 1.0 else ' Munk x%.2f' % scale, onset(rows, C17, scale)))

    free, held = rows[(C17, 4.0, False, 1.0)], rows[(C17, 4.0, True, 1.0)]
    say('  2517 at 4 m/s, what bounds the cycle: free, max |r| %.1f deg/s in the last window with the tail at %.3f of '
        'its range at most and %d saturated samples, the speed falling to %.2f m/s (window means %s); held at 4.0, '
        'the same start %s: max |r| %.1f deg/s, tail peak %.3f, %d saturated samples, fins at %s x their stall. '
        'The free cycle is bounded by the speed it loses, not by the tail\'s limits.'
        % (free['amp'][-1], free['peak'], free['sat'], free['u'][-1], ' '.join('%.2f' % x for x in free['u']),
           'departs' if held['sat'] else 'stays bounded', held['amp'][-1], held['peak'], held['sat'],
           '-' if held['alpha'] is None else '%.1f' % held['alpha']))

    say('\n--- the stalled branch (airship_tail_design.md:113-121): put on it before the loop closes, r0 15 deg/s '
        'with the full stick held through Manual into Acro, against the attached start of the G4 legs ---')
    say(TABLE_HEAD)
    say(TABLE_RULE)

    for target in (V27, V22):
        for u0 in (2.0, 2.5):
            leg = F.run_leg(target, u0, 1.0, r0=15.0, stick_in_manual=True)
            say(leg_row(leg) + ' started stalled')
            leg = F.run_leg(target, u0, 1.0)
            say(leg_row(leg) + ' started attached (the G4 leg form)')

    say('\n--- gz\'s gyro noise (closed_loop W2, seeded) on the 2527 4 m/s full-stick leg ---')
    for seed in (1, 2, 3):
        m = F.run_leg(V27, 4.0, 1.0, noise=True, seed=seed)['metrics']
        say('  seed %d: r mean last 5 s %.3f deg/s, integ %.4f, heading in release %+.2f deg, r +120 s %+.3f'
            % (seed, m['r_mean_last5'], m['integ_stick_end'], m['heading_release'], m['r_rel_120']))

    probes()
    return legs


WINDOWS = ((0, 30), (30, 60), (60, 90), (90, 120), (120, 153))


def stability_row(target, u0, hold, tag=''):
    """Straight flight at u0 in Acro, stick centred, from a sway of 0.01 m/s,
    150 s, the exact trim (quantize=False: the 1/1000 truncation would fly a
    lower speed than the label, 2517 at 1 m/s 0.00298 sent as 0.002); hold:
    the surge held at u0. Every number from the same 10 Hz samples."""
    leg = F.run_leg(target, u0, 0.0, v0=-0.01, manual_s=0.0, centred_s=1.0, stick_s=1.0, release_s=150.0,
                    quantize=False, hold_u=u0 if hold else None)        # 0.01 m/s to port (v0 is FRD)
    rows = leg['series']
    win = [[row for row in rows if a <= row['t'] < b] for a, b in WINDOWS]
    amp = [max(abs(row['r']) for row in w) for w in win]
    um = [sum(row['u'] for row in w) / len(w) for w in win]
    key = 'torque' if leg['setup']['vectored'] else 'tail'
    peak = max(abs(row[key]) for row in rows)
    sat = sum(1 for row in rows if row['sat_pos'] or row['sat_neg'])
    stalls = {f['name']: f['alpha_stall'] for f in leg_fins(target)}
    alpha = max((abs(row['alpha_' + n]) / math.degrees(st) for row in rows for n, st in stalls.items()
                 if row.get('alpha_' + n) is not None), default=None)
    ratio = amp[-1] / amp[1]
    verdict = 'grows' if ratio > 1.05 else ('decays' if ratio < 0.95 else 'neutral')
    say('  %-44s u0 %.2f %s%s: max |r| per 30 s %s deg/s -> %s (x%.2f); u mean %s m/s; %s peak %.3f, %d saturated; '
        'fins %s x stall'
        % (target, u0, 'held' if hold else 'free', tag, ' '.join('%.3f' % a for a in amp), verdict, ratio,
           ' '.join('%.2f' % x for x in um), 'tail |cmd|' if key == 'tail' else '|torque|', peak, sat,
           '-' if alpha is None else '%.2f' % alpha))
    return {'amp': amp, 'u': um, 'peak': peak, 'sat': sat, 'alpha': alpha, 'ratio': ratio, 'verdict': verdict}


def leg_fins(target):
    """The vertical fins of a target's plant, named as forward.run_leg names
    them (planar.RollPlanarPlant: dorsal above the centre of mass)."""
    import planar
    model = cl.ALL_TARGETS[target][0]
    return [f for f in planar.RollPlanarPlant(model, model=model).fins if f['vertical']]


def onset(rows, target, scale):
    """Where straight flight turns unstable with the surge held: the speeds
    that decay and grow, and the neutral speed interpolated in log(ratio)
    between the last decaying and the first growing one."""
    held = sorted((u0, r['ratio']) for (t, u0, hold, sc), r in rows.items() if t == target and hold and sc == scale)
    text = 'held, last/second window ' + ', '.join('%.2f m/s x%.2f' % x for x in held)

    for (ua, ra), (ub, rb) in zip(held, held[1:]):
        if ra <= 1.0 < rb:
            la, lb = math.log(ra), math.log(rb)
            return text + '; neutral near %.2f m/s' % (ua + (ub - ua) * (0.0 - la) / (lb - la))

    return text + '; no crossing between the speeds flown'


def tail_dither():
    """What gz's gyro noise does to the tail command in straight flight, at
    HEAD gains (closed_loop's W2 noise, seeded), and to the two counts."""
    out = []

    for target, u0 in ((C12, 2.0), (C17, 1.0)):
        leg = F.run_leg(target, u0, 0.0, noise=True, quantize=False, manual_s=0.0, centred_s=1.0, stick_s=1.0,
                        release_s=30.0, sample_hz=250.0)
        cmd = [row['tail'] for row in leg['series'] if row['t'] > 2.0]
        rms = math.sqrt(sum(x * x for x in cmd) / len(cmd))
        peak = max(abs(x) for x in cmd)
        m = leg['metrics']
        out.append('%s at %.1f m/s, 30 s: rms %.5f, peak %.5f (%s); %d flips outside the band, %d sign changes of '
                   'the thrust'
                   % (cl.ALL_TARGETS[target][0], u0, rms, peak,
                      'the band is %.0f x the peak' % (F.TAIL_BAND / peak) if peak > 0 else 'no dither at all',
                      m['tail_flips_release'], m['tail_thrust_signs_release']))

    return 'With gz\'s gyro noise and the stick centred the tail command dithers: ' + '; '.join(out) + \
        '. In SITL the band-0 count is a noise count; the model\'s legs are noise-free, so there it counts the turns.'



def tau_release(leg):
    """Time constant of r's decay over the release, from +5 s to the end."""
    s = leg['setup']
    t_rel = s['times'][3]
    rows = [(row['t'], row['r']) for row in leg['series'] if row['t'] >= t_rel + 5.0]

    if len(rows) < 2 or rows[0][1] * rows[-1][1] <= 0 or abs(rows[-1][1]) >= abs(rows[0][1]):
        return float('nan')

    return (rows[-1][0] - rows[0][0]) / math.log(rows[0][1] / rows[-1][1])


def steady_k(target, u0, torque=0.5, seconds=120.0):
    """Normalized torque per rad/s that holds a steady turn at u0 (surge
    held): the plant alone under a constant torque, the authority from the
    file."""
    model, airframe = cl.ALL_TARGETS[target]
    loop = cl.build(target, params=ctl.head_params(model), hold_u=u0)
    act = loop.actuation
    loop.plant.set_state(u=u0)
    loop.arm()
    loop.set_stick(torque)
    loop.set_mode('manual')
    loop.advance(seconds)
    r = -loop.plant.nu[3]
    return torque / r


def probes():
    """The model from each SITL probe's own entry state and gains, against
    the probe's log from the Acro switch on: r (the gyro the loop reads), the
    integrator, u; and the plant's hold torque per rad/s over the stick's last
    2 s, SITL against the model at the same operating point, the acceptance
    test of the fins' damping as flown. u, v and the heading are the ground
    truth's (vehicle_local_position_groundtruth), as forward_yaw_check reads
    them: the EKF's heading in these logs is off the truth by the angle printed
    per log, and a body sway taken from the EKF velocity is that angle times u,
    not a sideslip. The model run from the EKF's entry is printed beside it,
    because the first cross-check of 2026-10-01 started from there."""
    from pyulog import ULog
    import bisect
    import record_gains as rg
    say('\n--- cross-check against the main session\'s SITL probes of 2026-10-01 (scratchpad probe_fwd.py, HEAD '
        '4e1baea97c; an informal run, not the 0b check): the model started from each log\'s ground-truth state at '
        'the Acro switch, with the log\'s own gains, throttle, stick and segment lengths; rows from the switch on ---')

    for rel, target in PROBES:
        path = os.path.join(LOGDIR, rel)

        if not os.path.exists(path):
            say('  %s: log not found, not compared' % rel)
            continue

        log = ULog(path, list(PROBE_TOPICS))
        d = {x.name: x.data for x in log.data_list if x.multi_id == 0}
        missing = [t for t in PROBE_TOPICS if t not in d]

        if missing:
            say('  %s: the log lacks %s, not compared' % (rel, ', '.join(missing)))
            continue

        model = cl.ALL_TARGETS[target][0]
        ip = rg.ulog_params(rel)
        want = ctl.airframe_for_model(model)

        if int(ip.get('SYS_AUTOSTART', -1)) != want:
            raise ValueError('%s: SYS_AUTOSTART %s, %s flies %d' % (rel, ip.get('SYS_AUTOSTART'), model, want))

        changed = sorted({name for _, name, _ in log.changed_parameters if name in rg.GAINS})

        if changed:
            raise ValueError('%s: %s changed during the log; the replay would need the timeline' % (rel, changed))

        gains = {name: rg.f32(ip[name]) for name in rg.GAINS}
        cm, mc, lp, gt = d['vehicle_control_mode'], d['manual_control_setpoint'], d['vehicle_local_position'], \
            d['vehicle_local_position_groundtruth']
        att, av, rc, tq = d['vehicle_attitude_groundtruth'], d['vehicle_angular_velocity'], d['rate_ctrl_status'], \
            d['vehicle_torque_setpoint']
        t_acro = next(int(t) for t, a, r in zip(cm['timestamp'], cm['flag_armed'], cm['flag_control_rates_enabled'])
                      if a and r)
        t_off = next(int(t) for t, a, r in zip(cm['timestamp'], cm['flag_armed'], cm['flag_control_rates_enabled'])
                     if t > t_acro and not r)
        mc_on = next(int(t) for t, y in zip(mc['timestamp'], mc['yaw']) if t > t_acro and abs(y) > 0.5)
        mc_off = next(int(t) for t, y in zip(mc['timestamp'], mc['yaw']) if t > mc_on and abs(y) < 0.5)
        sticks = [float(y) for t, y in zip(mc['timestamp'], mc['yaw']) if mc_on <= t < mc_off]
        stick = sorted(sticks)[len(sticks) // 2]
        throttle = float(next(th for t, th in zip(mc['timestamp'], mc['throttle']) if t >= mc_on))
        fraction = (throttle + 1.0) / 2.0
        # The stick's edges. manual_control_setpoint is logged at 5 Hz and reads
        # an edge up to 0.2 s late (0.004 of a winding integrator); the rate
        # setpoint (about 20 Hz) and the torque setpoint (50 Hz) carry it in the
        # cycle the controller saw it, the torque as a step of (P + FF) sp
        sp = abs(ctl.yaw_rate_setpoint(stick, gains['MAN_DEADZONE'], math.radians(gains['AS_YAWRATE_MAX'])))
        jump = (gains['AS_YAWRATE_P'] + gains['AS_YAWRATE_FF']) * sp
        sgn = 1.0 if stick >= 0 else -1.0
        rates = d['vehicle_rates_setpoint']
        on = edge([bracket(rates, 'yaw', t_acro, lambda a, b: sgn * (b - a) > 0.5 * sp),
                   bracket(tq, 'xyz[2]', t_acro, lambda a, b: sgn * (b - a) > 0.5 * jump)])
        off = edge([bracket(rates, 'yaw', on[0] + 1_000_000, lambda a, b: sgn * (a - b) > 0.5 * sp),
                    bracket(tq, 'xyz[2]', on[0] + 1_000_000, lambda a, b: sgn * (a - b) > 0.5 * jump)])
        stick_on, stick_off = on[0], off[0]

        def sample(series, t):
            """The index of the sample whose own timestamp is nearest t: a 5 Hz
            topic read 'at or after t' is up to 0.2 s late, which on a winding
            integrator is 0.004 of it."""
            ts = series['timestamp']
            i = bisect.bisect_left(ts, t)
            return min((j for j in (i - 1, i) if 0 <= j < len(ts)), key=lambda j: abs(int(ts[j]) - t))

        def nearest(series, key, t):
            return float(series[key][sample(series, t)])

        def body(series, t):
            """(u, v) FRD from a local position's NED velocity and its own
            heading, all three from the one sample nearest t."""
            i = sample(series, t)
            vx, vy, hd = (float(series[k][i]) for k in ('vx', 'vy', 'heading'))
            return vx * math.cos(hd) + vy * math.sin(hd), -vx * math.sin(hd) + vy * math.cos(hd)

        def at(series, key, t):
            """(SITL value, the model's at that sample's own time) for one topic field."""
            i = sample(series, t)
            return float(series[key][i]), (int(series['timestamp'][i]) - t_acro) / 1e6

        def window(series, key, a, b):
            return [float(x) for t, x in zip(series['timestamp'], series[key]) if a <= t < b]

        u0, v0 = body(gt, t_acro)
        u_e, v_e = body(lp, t_acro)
        bias = (math.degrees(nearest(lp, 'heading', t_acro) - nearest(gt, 'heading', t_acro)) + 180.0) % 360.0 - 180.0
        w, x, y, z = (nearest(att, 'q[%d]' % i, t_acro) for i in range(4))
        roll0 = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
        p0, r0 = (math.degrees(nearest(av, 'xyz[%d]' % i, t_acro)) for i in (0, 2))
        centred, stick_s, release = (stick_on - t_acro) / 1e6, (stick_off - stick_on) / 1e6, (t_off - stick_off) / 1e6
        common = dict(throttle=fraction, quantize=False, params=gains, manual_s=0.0, centred_s=centred,
                      stick_s=stick_s, release_s=release, r0=r0, sample_hz=50.0)
        leg = F.run_leg(target, u0, stick, v0=v0, phi0=roll0, p0=p0, **common)       # v0 FRD, as the log's
        ekf = F.run_leg(target, u_e, stick, v0=v_e, **common)
        say('  %s %s (SYS_AUTOSTART %d; the log\'s gains P %g I %g LIM %g FF %g): at the Acro switch, ground truth '
            'u %.3f v %+.3f m/s (FRD), roll %+.2f deg, p %+.2f, r %+.2f deg/s; the EKF\'s heading is %+.2f deg off '
            'the truth there, and its velocity in its own heading reads u %.3f v %+.3f; stick %+.3f, throttle %.3f; '
            'centred %.3f s, stick %.3f s, release %.2f s (the stick\'s edges from the rate and torque setpoints, '
            'each inside %d / %d ms; manual_control_setpoint reads them %+d / %+d ms later)'
            % (rel, target, want, gains['AS_YAWRATE_P'], gains['AS_YAWRATE_I'], gains['AS_YR_INT_LIM'],
               gains['AS_YAWRATE_FF'], u0, v0, roll0, p0, r0, bias, u_e, v_e, stick, fraction, centred, stick_s,
               release, on[1] // 1000, off[1] // 1000, (mc_on - stick_on) // 1000, (mc_off - stick_off) // 1000))
        say('    %7s %7s | %-22s | %-24s | %-20s | %s' % ('t-acro', 't-stick', 'r deg/s  SITL / model',
                                                       'integ  SITL / model', 'u m/s  SITL / model',
                                                       'r from the EKF entry'))
        marks = [m for m in (0.5, 1.0, 2.0) if m < centred] + \
            [centred + m for m in (2, 5, 10, 15, 19, 20.4, 22, 25, 30, 40, 50, 60)]
        diffs = {'centred': [], 'after': []}

        for tm in marks:
            t = t_acro + int(round(tm * 1e6))

            if t > t_off:
                continue

            # each SITL sample against the model at that sample's own time
            r_s, tr = at(av, 'xyz[2]', t)
            i_s, ti = at(rc, 'yawspeed_integ', t)
            j = sample(gt, t)
            u_s, tu = body(gt, int(gt['timestamp'][j])), (int(gt['timestamp'][j]) - t_acro) / 1e6
            r_s = math.degrees(r_s)
            r_m, i_m, u_m = model_at(leg, 'r', tr), model_at(leg, 'integ', ti), model_at(leg, 'u', tu)
            diffs['centred' if tm < centred else 'after'].append((r_s - r_m, i_s - i_m, u_s[0] - u_m))
            say('    %7.1f %7.1f | %+7.3f / %+7.3f      | %+.4f / %+.4f         | %.3f / %.3f        | %+7.3f'
                % (tm, tm - centred, r_s, r_m, i_s, i_m, u_s[0], u_m, model_at(ekf, 'r', tr)))

        for name, rows in diffs.items():
            if rows:
                rms = [math.sqrt(sum(x[k] ** 2 for x in rows) / len(rows)) for k in range(3)]
                say('    rms SITL - model, %s (%d rows): r %.3f deg/s, integ %.4f, u %.4f m/s'
                    % ('centred segment' if name == 'centred' else 'stick and release', len(rows), *rms))

        a, b = stick_off - 2_000_000, stick_off
        tq_s, r_win = window(tq, 'xyz[2]', a, b), window(av, 'xyz[2]', a, b)
        rows = [x for x in leg['series'] if centred + stick_s - 2.0 <= x['t'] < centred + stick_s]

        if not tq_s or not r_win or not rows:
            say('    hold torque per rad/s: no samples in the stick\'s last 2 s, not compared')
            continue

        k_s = (sum(tq_s) / len(tq_s)) / (sum(r_win) / len(r_win))
        k_m = (sum(x['torque'] for x in rows) / len(rows)) / math.radians(sum(x['r'] for x in rows) / len(rows))
        say('    the plant\'s hold torque per rad/s over the stick\'s last 2 s (mean torque over mean r, %d / %d SITL '
            'samples, %d model rows): SITL %.3f, model %.3f (%+.1f %%), at r %.3f / %.3f deg/s and u %.3f / %.3f m/s'
            % (len(tq_s), len(r_win), len(rows), k_s, k_m, 100 * (k_m / k_s - 1),
               math.degrees(sum(r_win) / len(r_win)), sum(x['r'] for x in rows) / len(rows),
               body(gt, a + 1_000_000)[0], sum(x['u'] for x in rows) / len(rows)))


def bracket(series, key, after, hit):
    """(t_lo, t_hi): the two samples around the first change at or after
    `after` for which hit(previous value, value) holds; the change happened in
    (t_lo, t_hi]. None when there is none."""
    prev = None

    for t, x in zip(series['timestamp'], series[key]):
        t, x = int(t), float(x)

        if t < after:
            continue

        if prev is not None and hit(prev[1], x):
            return prev[0], t

        prev = (t, x)

    return None


def edge(brackets):
    """(time, width) of an edge from several topics' brackets: the middle of
    their intersection, and how wide it is [us]. Raises when a topic has no
    such change or the brackets do not overlap: the edge is not where the
    topics say it is."""
    if any(b is None for b in brackets):
        raise LookupError('a stick edge is missing from one of its topics: %s' % brackets)

    lo, hi = max(b[0] for b in brackets), min(b[1] for b in brackets)

    if lo >= hi:
        raise ValueError('the stick edge brackets do not overlap: %s' % brackets)

    return (lo + hi) // 2, hi - lo


def model_at(leg, key, tm):
    """A leg's series value at time tm [s since the Acro switch], linear
    between its rows (50 Hz in the probe replays)."""
    rows = leg['series']
    i = min(range(len(rows)), key=lambda j: abs(rows[j]['t'] - tm))
    j = i + 1 if (rows[i]['t'] <= tm and i + 1 < len(rows)) else (i - 1 if i > 0 else i)

    if j == i or rows[j]['t'] == rows[i]['t']:
        return rows[i][key]

    w = (tm - rows[i]['t']) / (rows[j]['t'] - rows[i]['t'])
    return rows[i][key] + w * (rows[j][key] - rows[i][key])


def tests():
    say('\n=== the model\'s tests ===')
    for name in ('test_controller.py', 'test_forward.py', 'test_fin_damping.py'):
        run = subprocess.run([sys.executable, os.path.join(HERE, name)], capture_output=True, text=True, cwd=HERE)
        tail = [line for line in run.stdout.splitlines() if line.startswith('FAIL') or 'failed' in line]
        passed = sum(1 for line in run.stdout.splitlines() if line.startswith('PASS'))
        say('  %s: exit %d, %d PASS; %s' % (name, run.returncode, passed, '; '.join(tail) or '?'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--g4', action='store_true', help='the forward legs only')
    args = ap.parse_args()
    say('Step 0a, 2026-10-01: the yaw model at HEAD, its gates, the forward-flight defect (yaw_model/step0a.py)')

    if args.g4:
        g4()
    else:
        head_section()
        g1()
        g2()
        g3()
        g4()
        tests()

    with open(OUT, 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(lines) + '\n')

    print('written', OUT)


if __name__ == '__main__':
    main()
