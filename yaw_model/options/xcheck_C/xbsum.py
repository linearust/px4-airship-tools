#!/usr/bin/env python3
"""Summary of xb.jsonl (xbeyond.py) -> xb_summary.txt."""
import collections
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'C'))
sys.dont_write_bytecode = True
import optc  # noqa: E402

rows = [json.loads(l) for l in open(os.path.join(HERE, 'xb.jsonl'))]
out = []


def f(x, fmt='%.2f'):
    return '-' if x is None else fmt % x


def rng(vals, fmt='%.2f'):
    vals = [v for v in vals if v is not None]
    if not vals:
        return '-'
    lo, hi = min(vals), max(vals)
    return (fmt % lo) if abs(hi - lo) < 1e-9 else (fmt + '..' + fmt) % (lo, hi)


by = collections.defaultdict(list)
for r in rows:
    e = r['extra']
    key = (r['kind'], r['target'], tuple(e['act']) if e.get('act') else None, e.get('stick'), e.get('scale'))
    by[key].append(r)

S = sorted({(r['P'], r['I'], r['b']) for r in rows})
out.append('STEP 0 -> stick (Acro from rest, push 60 s, release 40 s). noise-off value [noise seeds 11,12 range]')
out.append('  os%, t_pk s, rise10-90 s, settle10 = last exit from +-10 % (s), rel = first |r|<1 held 1.5 s, under = min rate after release, relI0 = integ at release')
for t in optc.TARGETS:
    for act in [None, ('slew', 60.0), ('slew', 240.0), ('servo', 60.0), ('tail_tau', 0.025), ('tail_tau', 0.05)]:
        for stick in (0.5, 1.0):
            rs = by.get(('step', t, act, stick, None))
            if not rs:
                continue
            out.append('  %s act %s stick %.1f' % (optc.SHORT[t], act, stick))
            for s in S:
                g = [r for r in rs if (r['P'], r['I'], r['b']) == s]
                if not g:
                    continue
                n0 = [r['r'] for r in g if r['extra']['seed'] == 0]
                nn = [r['r'] for r in g if r['extra']['seed'] != 0]
                if not n0:
                    continue
                a = n0[0]
                line = '    P %.2f I %.2f b %.3f | os %+5.1f%% [%s] t_pk %4.1f rise %s settle10 %s [%s] | rel %s [%s] under %+.2f relI0 %.4f end %+.2f' % (
                    s + (a['os_pct'], rng([x['os_pct'] for x in nn], '%+.1f'), a['t_peak'], f(a['rise'], '%.1f'),
                         f(a['settle10'], '%.1f'), rng([x['settle10'] for x in nn], '%.1f'), f(a['rel_hold'], '%.1f'),
                         rng([x['rel_hold'] for x in nn], '%.1f'), a['rel_under'], a['rel_integ0'], a['rel_end']))
                out.append(line)
out.append('')
out.append('WIND step (world-z couple, scale x authority/54 N m; FRD = -tau) after 40 s at the stick, noise seed 11')
out.append('  base = rate before, Ib = integ before, peak/end dev (deg/s), Imax = max |integ| in the 60 s')
for t in optc.TARGETS:
    for stick, scale in ((0.0, 5.0), (0.5, 5.0), (1.0, 5.0), (0.5, 1.0)):
        rs = by.get(('wind', t, None, stick, scale))
        if not rs:
            continue
        out.append('  %s stick %.1f scale %.0f' % (optc.SHORT[t], stick, scale))
        for s in S:
            g = [r['r'] for r in rs if (r['P'], r['I'], r['b']) == s]
            if g:
                a = g[0]
                out.append('    P %.2f I %.2f b %.3f | base %6.2f Ib %+.4f peak dev %+6.2f end dev %+6.2f Imax %.3f' % (
                    s + (a['base'], a['integ_before'], a['peak_dev'], a['end_dev'], a['integ_max'])))
out.append('')
out.append('TORQUE-COMMAND NOISE (gz gyro noise, seeds 11/12): std over 20 s, std_hf = minus a 1 s moving mean')
for t in optc.TARGETS:
    for stick in (0.0, 0.5):
        rs = by.get(('noise', t, None, stick, None))
        if not rs:
            continue
        out.append('  %s stick %.1f' % (optc.SHORT[t], stick))
        for s in S:
            g = [r['r'] for r in rs if (r['P'], r['I'], r['b']) == s]
            if g:
                out.append('    P %.2f I %.2f b %.3f | std %s  std_hf %s' % (
                    s + (rng([x['std'] for x in g], '%.5f'), rng([x['std_hf'] for x in g], '%.5f'))))
txt = '\n'.join(out)
open(os.path.join(HERE, 'xb_summary.txt'), 'w').write(txt + '\n')
print(txt)
