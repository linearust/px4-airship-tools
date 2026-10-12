#!/usr/bin/env python3
"""Summaries of the cross-check runs.

    python3 xsum.py repro   # my b vs option C's jsonl, same members/seeds: verdict + margin agreement; my matrix
    python3 xsum.py fresh   # new seeds (101..105) x 8 timing members: matrix
    python3 xsum.py act     # actuator variants: matrix per variant
Undecided ('?'): split verdicts, or a margin within the model's stated
uncertainty (rate 0.4 vec rigid / 0.15 cld complete / 0.12 else; settle 0.6 s;
wind integrator 0.001), or an unsettled release whose end |r| < 1.0 + band.
"""
import collections
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CDIR = os.path.join(os.path.dirname(HERE), 'C')
sys.path.insert(0, CDIR)
sys.dont_write_bytecode = True
import optc  # noqa: E402

KEYS = ['Acro brake', 'Acro track', 'Acro release', 'Stab track', 'Stab release', 'wind I on', 'wind I off',
        'wind weak', 'wind check']


def load(path):
    return [json.loads(line) for line in open(path)] if os.path.exists(path) else []


def cell(cs, key, target):
    cs = [c for c in cs if key in c]
    if not cs:
        return None
    ok = [c[key]['ok'] for c in cs]
    k, n = sum(ok), len(ok)
    band = optc.rate_band(target)
    und = 0 < k < n
    if key in ('Acro brake', 'wind I off', 'wind check'):
        ms = [c[key]['m'] for c in cs]
        und |= any(abs(m) <= band for m in ms)
        txt = '%+.2f..%+.2f' % (min(ms), max(ms))
    elif key.endswith('track'):
        mp = [c[key]['m_peak'] for c in cs]
        mm = [c[key]['m_mean'] for c in cs]
        und |= any(abs(m) <= band for m in mp + mm)
        txt = 'pk%+.2f..%+.2f mn%+.2f..%+.2f' % (min(mp), max(mp), min(mm), max(mm))
    elif key.endswith('release'):
        st = [c[key]['settle'] for c in cs]
        ms = [10.0 - s for s in st if s is not None]
        ends = [abs(c[key]['end']) for c in cs if c[key]['settle'] is None]
        parts = []
        if ms:
            und |= any(abs(m) <= optc.SETTLE_BAND for m in ms)
            parts.append('%+.1f..%+.1fs' % (min(ms), max(ms)))
        if ends:
            und |= any(e < 1.0 + band for e in ends)
            parts.append('end%.2f..%.2f(%d)' % (min(ends), max(ends), len(ends)))
        txt = ' '.join(parts)
    else:
        mr = [c[key]['m_rate'] for c in cs]
        mi = [c[key]['m_integ'] for c in cs]
        pct = [100 * c[key]['m_integ'] / abs(c[key]['needed']) for c in cs]
        und |= any(abs(m) <= band for m in mr) or any(abs(m) <= optc.INTEG_BAND for m in mi)
        txt = 'r%+.2f i%+.0f%%' % (min(mr), min(pct))
    return '%d/%d %s%s' % (k, n, txt, '?' if und else '')


def matrix(rows, title, group=lambda r: (r['P'], r['I'], r['b'])):
    by = collections.defaultdict(list)
    for r in rows:
        by[group(r) + (r['target'],)].append(r['checks'])
    groups = sorted({k[:-1] for k in by}, key=lambda g: tuple(str(x) for x in g))
    out = [title, '']
    for g in groups:
        out.append('  '.join(str(x) for x in g))
        for t in optc.TARGETS:
            cs = by.get(g + (t,))
            if not cs:
                continue
            parts = []
            for k in KEYS:
                c = cell(cs, k, t)
                if c is not None:
                    parts.append('%s %s' % (k.replace('Acro ', 'A.').replace('Stab ', 'S.').replace('wind ', 'w.'), c))
            out.append('  %-18s ' % optc.SHORT[t] + ' | '.join(parts))
        out.append('')
    return '\n'.join(out)


def repro():
    mine = load(os.path.join(HERE, 'x_repro.jsonl'))
    theirs = load(os.path.join(CDIR, 'scan_full.jsonl')) + load(os.path.join(CDIR, 'scan_gains.jsonl'))
    ti = {(r['target'], r['P'], r['I'], r['b'], r['member'], r['seed']): r for r in theirs}
    n = agree = 0
    worst = collections.defaultdict(float)
    mism = []
    for r in mine:
        k = (r['target'], r['P'], r['I'], r['b'], r['member'], r['seed'])
        t = ti.get(k)
        if not t:
            continue
        n += 1
        same = all(r['checks'][c]['ok'] == t['checks'][c]['ok'] for c in r['checks'] if c in t['checks'])
        agree += same
        if not same:
            mism.append(k)
        for c in r['checks']:
            for f in ('m', 'm_peak', 'm_mean', 'settle', 'end', 'rate', 'integ'):
                a, b = r['checks'][c].get(f), t['checks'].get(c, {}).get(f)
                if isinstance(a, (int, float)) and isinstance(b, (int, float)):
                    worst[(c, f)] = max(worst[(c, f)], abs(a - b))
    print('repro: %d of my runs matched to option C runs (same target/gains/b/member/seed); verdict sets identical in %d'
          % (n, agree))
    print('mismatches:', mism[:10])
    print('largest |mine - theirs| per field:', {'%s.%s' % k: round(v, 5) for k, v in worst.items() if v > 0} or 'all 0')
    txt = matrix(mine, 'MY MATRIX, option C ensemble (12 runs/cell: 8 timing members seeds 1-8 + nominal seeds 2-5), my b')
    open(os.path.join(HERE, 'x_repro_matrix.txt'), 'w').write(txt + '\n')
    print(txt)


def fresh():
    rows = load(os.path.join(HERE, 'x_fresh.jsonl'))
    txt = matrix(rows, 'FRESH SEEDS 101-105 x 8 timing members (40 runs/cell), my b')
    open(os.path.join(HERE, 'x_fresh_matrix.txt'), 'w').write(txt + '\n')
    print(txt)


def act():
    rows = load(os.path.join(HERE, 'x_act.jsonl'))
    txt = matrix(rows, 'ACTUATOR VARIANTS, nominal timing, seeds 201-205, my b',
                 group=lambda r: (tuple(r['act']), r['P'], r['I'], r['b']))
    open(os.path.join(HERE, 'x_act_matrix.txt'), 'w').write(txt + '\n')
    print(txt)


if __name__ == '__main__':
    {'repro': repro, 'fresh': fresh, 'act': act}[sys.argv[1]]()
