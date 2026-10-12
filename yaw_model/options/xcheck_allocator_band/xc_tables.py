#!/usr/bin/env python3
"""Pass counts and margins from xc_gate.jsonl (own margin code; thresholds
read from sitl_check.py: brake |r| < 6.0 (:371), tracking |mean - sp| <
0.25 sp and peak < 1.4 sp (:402-403), release settle < 10.0 (:425), wind
hold rate < 1.0 and |integ - needed| < 0.5 needed, no-hold rate > 1.0
(:443), wind check |d| > 0.5 expected (:314), Manual |torque - 0.5| < 0.02).

    python3 xc_tables.py [xc_gate.jsonl]

Margin + = inside the threshold. Uncertainty bands (the model's stated
ones, as option E used them): rigid vectored peak/brake 0.4 deg/s,
Cloudship-complete rates 0.15, other rates 0.12; release settle one gate
row 0.32 s; wind integrator 10 % of the needed torque. A cell is
  PASS  every run passes and every margin > its band
  FAIL  every run fails and every failing margin < -band (or never settles)
  F(pre) tracking failed on the clean-start precondition (line text
        '정지 전제가 깨졌다') while its thresholds were met
  U     anything else (split, or a margin within the band)
"""
import collections
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SP = (0.5 - 0.1) / 0.9 * 15.0
ORDER = ['wind check', 'Manual', 'Acro brake', 'Acro track', 'Acro release', 'Stab track', 'Stab release',
         'wind I on', 'wind I off', 'wind weak']
AB = {'wind check': 'windchk', 'Manual': 'Manual', 'Acro brake': 'brake', 'Acro track': 'A.trk',
      'Acro release': 'A.rel', 'Stab track': 'S.trk', 'Stab release': 'S.rel', 'wind I on': 'I-on',
      'wind I off': 'I-off', 'wind weak': 'weak'}
TORDER = ['gz_airship_vectored_independent', 'gz_airship_vectored_independent_am_fin_drag',
          'gz_airship_vectored_independent_fin', 'gz_cloudship', 'gz_cloudship_am_fin_drag']
SHORT = dict(zip(TORDER, ['vectored rigid', 'vectored complete', 'vectored _fin', 'Cloudship rigid',
                          'Cloudship complete']))
LABEL = {'Acro track': 'Acro: 스틱이 지시한', 'Stab track': 'Stabilized: 스틱이 지시한'}


def band(target, what):
    if target == 'gz_airship_vectored_independent' and what in ('peak', 'end'):
        return 0.4
    if target == 'gz_cloudship_am_fin_drag':
        return 0.15
    return 0.12


def margins(key, c, target):
    if c is None:
        return []
    g = c.get
    if key == 'wind check':
        return [('d', abs(g('end') - g('start')) - 0.5 * g('expected'), band(target, 'rate'))]
    if key == 'Manual':
        return [('tq', 0.02 - abs(g('torque') - 0.5), 0.002)]
    if key == 'Acro brake':
        return [('end', 6.0 - abs(g('end')), band(target, 'end'))]
    if key in ('Acro track', 'Stab track'):
        return [('mean', 0.25 * SP - abs(g('mean') - SP), band(target, 'mean')),
                ('peak', 1.4 * SP - g('peak'), band(target, 'peak'))]
    if key in ('Acro release', 'Stab release'):
        return [('settle', None if g('settle') is None else 10.0 - g('settle'), 0.32)]
    if key in ('wind I on', 'wind weak'):
        n = g('needed')
        return [('rate', 1.0 - g('rate'), band(target, 'rate')), ('integ', 0.5 * n - abs(g('integ') - n), 0.1 * n)]
    if key == 'wind I off':
        return [('rate', g('rate') - 1.0, band(target, 'rate'))]
    return []


def pre_failed(r, key):
    lab = LABEL.get(key)
    if not lab:
        return False
    for line in r['lines']:
        if lab in line:
            return '정지 전제가 깨졌다' in line
    return False


def fmt(x):
    return ('%+.4f' % x) if abs(x) < 0.01 else ('%+.2f' % x)


def cell(rs, key, target):
    recs = [(r, r['checks'].get(key)) for r in rs if r['checks'].get(key) is not None]
    if not recs:
        return None
    n = len(recs)
    npass = sum(1 for _, c in recs if c['ok'])
    npre = sum(1 for r, _ in recs if pre_failed(r, key))
    lo, hi, never, us = {}, {}, collections.Counter(), {}
    for _, c in recs:
        for name, m, u in margins(key, c, target):
            us[name] = u
            if m is None:
                never[name] += 1
                continue
            lo[name] = min(lo.get(name, m), m)
            hi[name] = max(hi.get(name, m), m)
    parts = []
    inside_all = True
    near = False
    for name, u in us.items():
        if name in lo:
            s = '%s..%s' % (fmt(lo[name]), fmt(hi[name]))
            if never[name]:
                s += ' (%d never)' % never[name]
            if lo[name] <= 0 or never[name]:
                inside_all = False
            if -u < lo[name] < u or -u < hi[name] < u:
                near = True
        else:
            s = 'never'
            inside_all = False
        parts.append(s)
    if npass == n and not near:
        flag = 'PASS'
    elif npass == 0 and npre == n and inside_all:
        flag = 'F(pre)'
    elif npass == 0 and not near:
        flag = 'FAIL'
    else:
        flag = 'U'
    extra = (' pre%d' % npre) if npre else ''
    return '%s %d/%d %s%s [%s]' % (AB[key], npass, n, flag, extra, ' / '.join(parts))


def main(path):
    rows = [json.loads(l) for l in open(path)]
    groups = collections.defaultdict(list)
    for r in rows:
        groups[(r['target'], r['variant'], r['gains'])].append(r)
    for t in TORDER:
        keys = sorted([k for k in groups if k[0] == t], key=lambda k: (k[2], k[1] != 'base', k[1]))
        if not keys:
            continue
        print('\n%s (%s)' % (SHORT[t], t))
        for k in keys:
            rs = groups[k]
            j = [r['jumps'] for r in rs if r.get('jumps') is not None]
            print('  %-14s %-4s N=%-2d jumps %s' % (k[1], k[2], len(rs), '%d-%d' % (min(j), max(j)) if j else '-'))
            for key in ORDER:
                c = cell(rs, key, t)
                if c:
                    print('      ' + c)


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, 'xc_gate.jsonl'))
