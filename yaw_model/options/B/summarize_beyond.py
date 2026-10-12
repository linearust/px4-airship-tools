#!/usr/bin/env python3
"""Tables from beyond.py jsonl files.  python3 summarize_beyond.py FILE.jsonl [...]"""
import collections
import json
import sys

sys.dont_write_bytecode = True
import ffcommon as fc  # noqa: E402


def f(x, spec='%.2f'):
    return '-' if x is None else spec % x


def main():
    rows = []
    for p in sys.argv[1:]:
        rows += [json.loads(l) for l in open(p)]
    by = collections.defaultdict(dict)
    for r in rows:
        key = (json.dumps(r['act'], sort_keys=True), r['target'], r['P'], r['I'], r['FF'])
        by[key][str(r['kind'])] = r
    print('step: rise 10-90 % [s], t90 [s], peak [deg/s], overshoot [%], ts10 = last exit from +-10 % sp [s],')
    print('      ess = mean(r) - sp over the last 10 s [deg/s], rel = gate settle rule from release [s],')
    print('      rel10 = last exit from 0.1 sp after release [s], under = most negative rate after release')
    print('wind: gust at rest: peak |r|, rec = last exit from 0.25 deg/s [s], integ/needed at 60 s;')
    print('      gust in a 6.67 deg/s turn: peak deviation, recovery to 0.25 deg/s [s]')
    print('noise: high-pass torque rms at rest / in the turn (gyro noise) / in the turn with stick jitter 0.002')
    for key in sorted(by, key=lambda k: (k[0], fc.TARGETS5.index(k[1]), k[2], k[3], k[4])):
        act, t, P, I, FF = key
        d = by[key]
        print('\n%-12s P %.2f I %.3f FF %.2f %s' % (fc.SHORT[t], P, I, FF, act if act != '{}' else ''))
        for k in ('0.25', '0.5', '1.0'):
            if k not in d:
                continue
            s = d[k]['res']
            if not s:
                print('   stick %s ERR %s' % (k, d[k]['err']))
                continue
            print('   stick %-4s sp %5.2f  rise %s t90 %s peak %s over %s%%  ts10 %s ess %+.2f | rel %s rel10 %s under %s'
                  % (k, s['sp'], f(s['rise'], '%.1f'), f(s['t90'], '%.1f'), f(s['peak']), f(s['over'], '%.0f'),
                     f(s['ts10'], '%.1f'), s['ess'], f(s['rel'], '%.1f'), f(s['rel10'], '%.1f'), f(s['under'])))
        if 'wind' in d and d['wind']['res']:
            w = d['wind']['res']
            print('   wind  rest: peak %.2f rec %s integ/need %.2f | turn: dev %+.2f rec %s'
                  % (w['peak'], f(w['recover_025'], '%.1f'), w['integ_end'] / w['needed'], w['turn_dev_peak'],
                     f(w['turn_recover_025'], '%.1f')))
            tq = d.get('0.5', {}).get('res', {}).get('tq_hp_turn')
            print('   noise torque hp rms: rest %.2e turn %s turn+stick jitter %.2e'
                  % (w['tq_hp_rest'], f(tq, '%.2e'), w['tq_hp_turn_jitter']))


if __name__ == '__main__':
    main()
