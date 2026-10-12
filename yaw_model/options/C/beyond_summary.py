#!/usr/bin/env python3
"""Tables from beyond.jsonl -> beyond_summary.txt."""
import collections
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True
import optc  # noqa: E402

rows = [json.loads(line) for line in open(os.path.join(HERE, 'beyond.jsonl'))]
by = collections.defaultdict(dict)
for r in rows:
    act = tuple(r['act']) if r['act'] else None
    key = (r['kind'], r['target'], act, tuple(sorted(r['extra'].items())))
    by[key][(r['P'], r['I'], r['b'])] = r['r']

settings = sorted({(r['P'], r['I'], r['b']) for r in rows}, key=lambda k: (k[0], k[1], -k[2]))


def f(x, fmt='%.1f', none='  -  '):
    return none if x is None else fmt % x


out = ['Option C outside the gate (loop driven directly, noise-free unless stated; from rest, no Manual phase).', '']

out.append('1. Stick steps from rest, Acro. push 60 s then release 40 s.')
out.append('   push: peak deg/s, overshoot %, 10-90 rise s, hold = first entry into +-10 % that holds 5 s, '
           'end = mean / std of the last 10 s, integ = integrator at the end of the push')
out.append('   release: gate-style = first |r| < 1 held 1.5 s (s), for good = last time |r| >= 1 (s), under = most '
           'negative rate, r@5s / r@10s')
for t in optc.TARGETS:
    for stick in (0.25, 0.5, 1.0):
        key = ('step', t, None, (('stick', stick),))
        if key not in by:
            continue
        sp = max(0.0, (stick - 0.1) / 0.9) * 15.0
        out.append('  %s (%s), stick %.2f -> %.2f deg/s' % (t, optc.SHORT[t], stick, sp))
        for s in settings:
            r = by[key].get(s)
            if not r:
                continue
            p, q = r['push'], r['release']
            out.append('    P %.2f I %.2f b %.3f | peak %5.2f (%+5.1f %%) rise %s hold %s end %5.2f/%.2f integ %.3f'
                       ' | release gate-style %s for good %s under %5.2f r@5 %5.2f r@10 %5.2f' % (
                           s[0], s[1], s[2], p['peak'], p['overshoot_pct'], f(p['rise_10_90']), f(p['hold_10pct']),
                           p['mean_last10'], p['std_last10'], p['integ_end'], f(q['first_hold_1p5']),
                           f(q['settle_1dps']), q['undershoot'], q['rate_at_5s'], q['rate_at_10s']))
    out.append('')

out.append('2. Actuator sensitivity: the stick 0.5 step under a slower / faster actuator.')
out.append('   slew X = CA_AIRSHIP_TLT_R matched to an X deg/s servo; servo 60 = a 60 deg/s servo with TLT_R left at'
           ' 120; tail_tau X = a first-order lag X s on the Cloudship tail rotor (gz has none; 0.025 = the model\'s'
           ' own counterfactual prop lag)')
for t in optc.TARGETS:
    acts = sorted({k[2] for k in by if k[0] == 'step' and k[1] == t and k[2]}, key=str)
    for act in [None] + acts:
        key = ('step', t, act, (('stick', 0.5),))
        if key not in by:
            continue
        out.append('  %s (%s) %s' % (t, optc.SHORT[t], ('act %s %g' % act) if act else 'nominal actuator'))
        for s in settings:
            r = by[key].get(s)
            if not r:
                continue
            p, q = r['push'], r['release']
            out.append('    P %.2f I %.2f b %.3f | peak %5.2f (%+5.1f %%) hold %s end %5.2f/%.2f | release gate-style %s'
                       ' for good %s under %5.2f' % (s[0], s[1], s[2], p['peak'], p['overshoot_pct'], f(p['hold_10pct']),
                                                     p['mean_last10'], p['std_last10'], f(q['first_hold_1p5']),
                                                     f(q['settle_1dps']), q['undershoot']))
    out.append('')

out.append('3. Wind steps: a world-z couple of scale x (authority / 54) N m stepped on after 40 s at the stick, held 60 s.')
out.append('   peak = worst deviation from the pre-wind rate (deg/s), recover = last time |dev| > 0.5 deg/s (s),'
           ' integ before -> end')
for t in optc.TARGETS:
    for scale, stick in ((1.0, 0.0), (5.0, 0.0), (5.0, 0.5), (5.0, 1.0)):
        key = ('wind', t, None, (('scale', scale), ('stick', stick)))
        if key not in by:
            continue
        rs = by[key]
        out.append('  %s (%s) wind x%g at stick %.1f' % (t, optc.SHORT[t], scale, stick))
        for s in settings:
            r = rs.get(s)
            if not r:
                continue
            out.append('    P %.2f I %.2f b %.3f | tau %.3f N m, base %5.2f, peak dev %+6.2f at %5.1f s, recover %s,'
                       ' end dev %+5.2f, integ %.4f -> %.4f' % (
                           s[0], s[1], s[2], r['tau_nm'], r['base'], r['peak_dev'], r['t_peak'], f(r['recover_0p5']),
                           r['dev_end'], r['integ_before'], r['integ_end']))
    out.append('')

out.append('4. Torque-command noise with gz gyro noise (8.7e-4 rad/s), 20 s after 40 s of steady flight:'
           ' std of vehicle_torque_setpoint z (normalized), total and above ~1 Hz')
for t in optc.TARGETS:
    for stick in (0.0, 0.5):
        key = ('noise', t, None, (('stick', stick),))
        if key not in by:
            continue
        out.append('  %s (%s) stick %.1f: %s' % (t, optc.SHORT[t], stick, '; '.join(
            'P%.1f I%.2f b%.2f %.5f/%.5f' % (s[0], s[1], s[2], by[key][s]['std'], by[key][s]['std_hf'])
            for s in settings if s in by[key])))
text = '\n'.join(out)
print(text)
open(os.path.join(HERE, 'beyond_summary.txt'), 'w').write(text + '\n')
