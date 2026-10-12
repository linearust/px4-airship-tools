#!/usr/bin/env python3
"""Tables from scan_*.jsonl.

    python3 summarize.py coarse   -> coarse_summary.txt
    python3 summarize.py full     -> full_summary.txt
    python3 summarize.py act      -> act_summary.txt

Margins: > 0 inside the gate's threshold, < 0 outside, in the unit the gate
judges (deg/s for rates and peaks, s for the release settle, fraction of
the needed torque for the wind integrator). UNDECIDED: the verdicts split
across the ensemble, or some member's margin is inside the model's stated
uncertainty (optc.RATE_BAND / SETTLE_BAND / INTEG_BAND).
"""
import collections
import json
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True
import optc  # noqa: E402

CHECKS = ['wind check', 'Manual', 'Acro brake', 'Acro track', 'Acro release', 'Stab track', 'Stab release',
          'wind I on', 'wind I off', 'wind weak']


def load(kind):
    return [json.loads(line) for line in open(os.path.join(HERE, 'scan_%s.jsonl' % kind))]


def band(check, target, field='m'):
    if check in ('Acro release', 'Stab release'):
        return optc.SETTLE_BAND
    if check in ('wind I on', 'wind weak'):
        return None            # two-part margin, handled below
    return optc.rate_band(target)


def fmt_track(c):
    return '%s%.2f/%.2f' % ('' if c['ok'] else '*', c['peak'], c['mean'])


def fmt_rel(c):
    if c['settle'] is not None:
        return '%s%.1fs' % ('' if c['ok'] else '*', c['settle'])
    return '*end%.1f' % abs(c['end'])


def coarse():
    rows = load('coarse')
    by = collections.defaultdict(dict)
    for r in rows:
        by[r['target']][(r['P'], r['I'], r['b'])] = r['checks']
    out = ['Option C coarse scan: nominal timing, gyro noise seed 1, one run per cell.',
           'track = peak/mean deg/s (pass: peak < %.3f, |mean - %.3f| < %.3f); release = settle s (pass < 10) or'
           ' *end = |rate| at the window end when it never settled; brake = end deg/s (pass |.| < 6);'
           ' wind on = rate/integ (pass rate < 1, |integ - need| < need/2); * = the gate fails it.'
           % (1.4 * optc.SP, optc.SP, 0.25 * optc.SP), '']
    for t in optc.TARGETS:
        out.append('== %s (%s)' % (t, optc.SHORT[t]))
        out.append('  P    I     b    | brake  | Acro track   Acro rel | Stab track   Stab rel | wind on      '
                   '| I off | weak         | fails')
        for key in sorted(by[t]):
            c = by[t][key]
            fails = [k for k in CHECKS if k in c and not c[k]['ok']]
            won = c['wind I on']
            weak = c.get('wind weak')
            out.append('  %.2f %.2f  %.3f | %s%5.1f | %-12s %-8s | %-12s %-8s | %s%.2f/%.4f | %s%.2f | %s | %d %s' % (
                key[0], key[1], key[2], '' if c['Acro brake']['ok'] else '*', c['Acro brake']['end'],
                fmt_track(c['Acro track']), fmt_rel(c['Acro release']),
                fmt_track(c['Stab track']), fmt_rel(c['Stab release']),
                '' if won['ok'] else '*', won['rate'], won['integ'],
                '' if c['wind I off']['ok'] else '*', c['wind I off']['rate'],
                ('%s%.2f/%.4f' % ('' if weak['ok'] else '*', weak['rate'], weak['integ'])) if weak else '     -      ',
                len(fails), ','.join(f.replace('Acro ', 'A.').replace('Stab ', 'S.') for f in fails)))
        out.append('')
    return '\n'.join(out)


def describe(check, cs, target):
    """pass count, margin range, undecided flag for one check over members."""
    oks = [c['ok'] for c in cs]
    n, k = len(oks), sum(oks)
    flag = ''
    if check == 'Manual':
        return '%2d/%d  (loop open: b-independent)' % (k, n), (k not in (0, n))
    if check in ('Acro track', 'Stab track'):
        mp = [c['m_peak'] for c in cs]
        mm = [c['m_mean'] for c in cs]
        bd = optc.rate_band(target)
        und = (0 < k < n) or any(abs(x) <= bd for x in mp) or any(abs(x) <= bd for x in mm)
        txt = '%2d/%d  peak %5.2f..%5.2f (m %+.2f..%+.2f)  mean %4.2f..%4.2f (m %+.2f..%+.2f)' % (
            k, n, min(c['peak'] for c in cs), max(c['peak'] for c in cs), min(mp), max(mp),
            min(c['mean'] for c in cs), max(c['mean'] for c in cs), min(mm), max(mm))
        return txt, und
    if check in ('Acro release', 'Stab release'):
        st = [c['settle'] for c in cs]
        settled = [s for s in st if s is not None]
        ends = [abs(c['end']) for c, s in zip(cs, st) if s is None]
        und = (0 < k < n) or any(abs(10.0 - s) <= optc.SETTLE_BAND for s in settled)
        txt = '%2d/%d  ' % (k, n)
        if settled:
            txt += 'settle %4.1f..%4.1f s (m %+.1f..%+.1f s)' % (min(settled), max(settled), 10 - max(settled),
                                                                 10 - min(settled))
        if ends:
            txt += '  never settled in %d: end |r| %.2f..%.2f (1.0 needed: %+.2f..%+.2f deg/s outside)' % (
                len(ends), min(ends), max(ends), min(ends) - 1.0, max(ends) - 1.0)
        mt = [c['m_turning'] for c in cs]
        if min(mt) < 0.5:
            txt += '  [start-turning margin %.2f]' % min(mt)
        return txt, und
    if check in ('wind I on', 'wind weak'):
        mr = [c['m_rate'] for c in cs]
        mi = [c['m_integ'] / abs(c['needed']) for c in cs]
        bd = optc.rate_band(target)
        und = (0 < k < n) or any(abs(x) <= bd for x in mr) or any(abs(x) * abs(cs[0]['needed']) < optc.INTEG_BAND
                                                                   for x in mi)
        txt = '%2d/%d  rate %4.2f..%4.2f (m %+.2f..%+.2f)  integ %.4f..%.4f (m %+.0f..%+.0f %% of need)' % (
            k, n, min(c['rate'] for c in cs), max(c['rate'] for c in cs), min(mr), max(mr),
            min(c['integ'] for c in cs), max(c['integ'] for c in cs), 100 * min(mi), 100 * max(mi))
        return txt, und
    ms = [c['m'] for c in cs]
    bd = optc.rate_band(target)
    und = (0 < k < n) or any(abs(x) <= bd for x in ms)
    unit = 'deg/s'
    extra = ''
    if check == 'Acro brake':
        extra = '  end %5.2f..%5.2f' % (min(c['end'] for c in cs), max(c['end'] for c in cs))
    return '%2d/%d  m %+.2f..%+.2f %s%s' % (k, n, min(ms), max(ms), unit, extra), und


def ensemble_table(kind):
    rows = load(kind)
    by = collections.defaultdict(list)
    for r in rows:
        by[(r['target'], r['P'], r['I'], r['b'], tuple(r['act']) if r['act'] else None)].append(r)
    out = []
    head = {'gains': 'Gains-alone comparators, same 12-run ensemble as full_summary.txt.',
            'full': ('Option C, ensemble: the 8 timing members of review/gain_sweep.py, each with gyro noise on its own seed (1..8),'
                     ' plus the nominal member on seeds 2..5 = 12 runs per cell (cells with more runs also hold members of an'
                     ' aborted 8 x 5 cross, same members, seeds 1..5)'),
            'act': 'Option C under actuator variants: nominal timing x 5 gyro-noise seeds per cell.'}[kind]
    out += [head, 'm = margin to the gate threshold (+ inside); UNDECIDED = split verdicts or a margin inside the'
            ' model\'s stated uncertainty (rate %s deg/s by plant, default %.2f; settle %.1f s; integ %.4f).'
            % (optc.RATE_BAND, optc.RATE_BAND_DEFAULT, optc.SETTLE_BAND, optc.INTEG_BAND), '']
    keys = sorted(by, key=lambda k: (optc.TARGETS.index(k[0]), str(k[4]), k[1], k[2], -k[3]))
    for key in keys:
        t, P, I, b, act = key
        rs = by[key]
        fails, unds = [], []
        lines = []
        for ch in CHECKS:
            cs = [r['checks'][ch] for r in rs if ch in r['checks']]
            if not cs:
                continue
            txt, und = describe(ch, cs, t)
            if not all(c['ok'] for c in cs):
                fails.append(ch)
            if und:
                unds.append(ch)
            lines.append('    %-12s %s%s' % (ch, txt, '   UNDECIDED' if und else ''))
        verdict = 'PASS all' if not fails else 'FAIL %s' % ', '.join(fails)
        out.append('== %-45s P %.2f I %.2f b %.3f%s  (%d runs)  %s%s' % (
            t, P, I, b, ('  act %s %g' % act) if act else '', len(rs), verdict,
            ('; undecided: ' + ', '.join(unds)) if unds else ''))
        out += lines
        out.append('')
    return '\n'.join(out)


if __name__ == '__main__':
    kind = sys.argv[1]
    text = coarse() if kind == 'coarse' else ensemble_table(kind)
    print(text)
    open(os.path.join(HERE, '%s_summary.txt' % kind), 'w').write(text + '\n')
