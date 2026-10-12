#!/usr/bin/env python3
"""Tables of beyond.py results: per plant, per actuator variant, per gain set."""
import collections
import json
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import optA  # noqa: E402


def f(x, fmt='%.2f'):
    if x is None:
        return '-'
    if x == float('inf'):
        return '>60'
    return fmt % x


def main():
    rows = [json.loads(line) for line in open(os.path.join(HERE, sys.argv[1] if len(sys.argv) > 1 else 'beyond.jsonl'))]
    by = collections.defaultdict(dict)
    sets = []
    for r in rows:
        p = r['params']
        s = (p['AS_YAWRATE_P'], p['AS_YAWRATE_I'], p['AS_YR_INT_LIM'])
        if s not in sets:
            sets.append(s)
        by[(r['target'], json.dumps(r['act']), s)][r['kind']] = r['res']
    sets.sort()
    lab = lambda s: 'P%.2f I%.2f L%.2f' % s
    for t in optA.TARGETS:
        acts = sorted({k[1] for k in by if k[0] == t}, key=lambda a: (a != 'null', a))
        print('\n==================== %s (%s)' % (optA.SHORT[t], t))
        for a in acts:
            print('  --- actuator %s' % ('nominal' if a == 'null' else a))
            for s in sets:
                d = by.get((t, a, s))
                if not d:
                    continue
                parts = []
                for k in ('step_small', 'step_half', 'step_full'):
                    x = d.get(k)
                    if x and 'error' not in x:
                        parts.append('%s os %s%% rise %s set10 %s set5 %s stop1 %s stop.2 %s under %s'
                                     % (k[5:], f(x['overshoot_pct'], '%.0f'), f(x['rise_10_90'], '%.1f'),
                                        f(x['settle10'], '%.1f'), f(x['settle5'], '%.1f'), f(x['stop_1deg'], '%.1f'),
                                        f(x['stop_0.2deg'], '%.1f'), f(x['undershoot'])))
                    elif x:
                        parts.append('%s ERROR %s' % (k, x['error']))
                for k in ('wind_gate', 'wind_10pct'):
                    x = d.get(k)
                    if x and 'error' not in x:
                        parts.append('%s peak %s deg/s hdg %s deg (at off %s) rec<0.2 %s s integ %s back %s deg/s %s deg'
                                     % (k, f(x['peak_rate']), f(x['heading_peak_deg'], '%.1f'), f(x['heading_at_off_deg'], '%.1f'),
                                        f(x['recover_0.2deg'], '%.1f'), f(x['integ_at_off'], '%.3f'),
                                        f(x['peak_rate_after_off']), f(x['heading_back_after_off_deg'], '%.1f')))
                x = d.get('gust')
                if x and 'error' not in x:
                    parts.append('gust5s peak %s deg/s hdg max %s deg final %s deg, after-off peak %s deg/s'
                                 % (f(x['peak_rate']), f(x['heading_max_deg'], '%.1f'), f(x['heading_final_deg'], '%.1f'),
                                    f(x['peak_rate_after_off'])))
                x = d.get('noise')
                if x and 'error' not in x:
                    extra = ('pod mode chg %s/min tilt %s deg/min' % (f(x['pod_mode_changes_per_min'], '%.1f'),
                                                                     f(x['tilt_travel_deg_per_min'], '%.1f'))
                             if 'pod_mode_changes_per_min' in x else 'tail reversals %s/s' % f(x['tail_reversals_per_s'], '%.1f'))
                    parts.append('hold noise: torque std %s max %s, motor cmd std %s max %s, rate rms %s deg/s, %s'
                                 % (f(x['torque_std'], '%.5f'), f(x['torque_max'], '%.4f'), f(x['esc_std'], '%.4f'),
                                    f(x['esc_max'], '%.3f'), f(x['rate_rms'], '%.3f'), extra))
                print('    %s' % lab(s))
                for p_ in parts:
                    print('        ' + p_)


if __name__ == '__main__':
    main()
