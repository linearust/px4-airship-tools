#!/usr/bin/env python3
"""Increment-1 step 1d (d1_increment1_plan_20261007.md): the SITL legs of
forward_yaw_check against the step-2 model (d1_check.py's acceptance part).

    python3 d1_sitl_compare.py ../sitl_out/forward_<model>_<stamp>_summary.json [...]

Each leg is held against two model legs, D1+RESTORE with the ground speed
and no wind bound (the posix values): the step-2 table's, flown from
straight flight at the nominal speed, and the same leg flown from the speed
the SITL leg actually had at the stick, which takes the trim's error out of
the comparison. The verdicts are on the second, with forward_replay's
tolerances: rates 3 % or 0.1 deg/s, headings 5 % or 2 deg, the integrator
0.005. The cap and the feedforward are compared at the stick's end, where
both depend on the speed the turn has bled off.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

import d1_check as D               # noqa: E402
from report import note, summary, verdict   # noqa: E402

CONFIG = {'airship_vectored_independent_am_fin_drag': ('2527', None),
          'airship_vectored_independent_fin': ('2522', None),
          'cloudship_fin': ('2512/khold', D.KHOLD_2512)}
TOL = {'rate': (0.1, 0.03), 'heading': (2.0, 0.05), 'integrator': (0.005, 0.0), 'cap': (0.05, 0.03),
       'ff': (0.05, 0.03)}


def within(kind, sitl, model, extra=0.0):
    """`extra` is an absolute allowance on top of the kind's: the schedule's
    feedforward and cap inside the ramp move with the speed the hull holds,
    which the model and SITL settle a few cm/s apart."""
    if sitl is None or model is None:
        return False, None
    floor, frac = TOL[kind]
    tol = max(floor, frac * abs(model)) + extra
    return abs(sitl - model) <= tol, tol


SPEED_SETTLE = 0.05         # m/s the two plants' held speeds may differ by


def speed_sensitivity(params, u, ff, cap):
    """(d ff / d u, d cap / d u) of the firmware's schedule at u, from the
    leg's own parameters: ff = FF0 + w k u with w the ramp on [LO, HI], so
    inside the ramp d ff / d u = k (w + u / (HI - LO)); the cap goes as 1 / ff
    past the ramp, so d cap / d u = -cap / ff d ff / d u."""
    k, hi = params.get('AS_YR_FF_SLP') or 0.0, params.get('AS_SPD_HI') or 0.0
    lo = params.get('AS_SPD_LO') or 0.5 * hi     # logs before 2026-10-12 carry AS_SPD_LO; since then the ramp starts at half AS_SPD_HI
    if not k or u is None or hi <= lo:
        return 0.0, 0.0
    w = min(max((u - lo) / (hi - lo), 0.0), 1.0)
    dff = k * (w + (u / (hi - lo) if lo < u < hi else 0.0))
    dcap = (cap / ff * dff) if ff else 0.0
    return dff, dcap


def model_leg(config, kov, u0, stick):
    spec = D.spec('acc', config, 'D1+RESTORE', 'ground_W0', round(u0, 3), stick, 20.0, k_override=kov)
    return D.run(spec)


def main(paths):
    recs = D.load()
    rows = []

    for path in paths:
        doc = json.load(open(path))
        model = doc['model']

        if model not in CONFIG:
            note('%s: %s 는 비교 대상이 아니다' % (path, model))
            continue

        config, kov = CONFIG[model]

        for leg in doc['legs']:
            plan = leg.get('plan') or {}
            speed, stick = plan.get('speed'), plan.get('stick')
            u_stick = leg['entry']['at_stick']['u']
            sc = leg.get('schedule') or {}
            nominal = recs.get(D.spec('acc', config, 'D1+RESTORE', 'ground_W0', speed, stick, 20.0,
                                      k_override=kov)['id'])
            entry = model_leg(config, kov, u_stick, stick)

            if 'error' in entry:
                verdict('%s 다리 %s: 모델이 돌았다' % (model, leg['leg']), False, entry['error'])
                continue

            rel = leg['release']
            r1 = rel['r_mean1s_deg_s']
            sitl = {'cap': (sc.get('at_release') or {}).get('cap_deg_s'), 'ff': (sc.get('at_release') or {}).get('ff'),
                    'r5': leg['stick_phase']['r_mean_last5_deg_s'], 'I_rel': rel['entry']['integrator'],
                    'rel120': rel['heading_change_deg'], 'r10': r1.get('+10'), 'r60': r1.get('+60'),
                    'r120': r1.get('+120')}
            mod = lambda r: None if r is None else {'cap': r['cap_end'], 'ff': r['ff_end'], 'r5': r['r_last5'],
                                                    'I_rel': r['I_rel'], 'rel120': r['drift_120'], 'r10': r['r_10'],
                                                    'r60': r['r_60'], 'r120': r['r_120']}
            m_nom, m_ent = mod(nominal), mod(entry)
            kinds = {'cap': 'cap', 'ff': 'ff', 'r5': 'rate', 'I_rel': 'integrator', 'rel120': 'heading',
                     'r10': 'rate', 'r60': 'rate', 'r120': 'rate'}
            name = '%s %.1f m/s 스틱 %+.1f (다리 %s, 스틱 때 u %.3f)' % (config, speed, stick, leg['leg'], u_stick)
            note('\n=== %s' % name)
            fails = []
            u_rel = (sc.get('at_release') or {}).get('u_lo')
            dff, dcap = speed_sensitivity(leg['replay']['params'], u_rel, sitl['ff'], sitl['cap'])
            extra = {'ff': abs(dff) * SPEED_SETTLE, 'cap': abs(dcap) * SPEED_SETTLE}

            for key, kind in kinds.items():
                ok, tol = within(kind, sitl[key], m_ent[key], extra.get(key, 0.0))
                fmt = lambda x: '?' if x is None else '%+.3f' % x
                note('  %-6s SITL %s | 모델 진입 속도 %s (허용 %s) %s | 표 (%.1f m/s) %s'
                     % (key, fmt(sitl[key]), fmt(m_ent[key]), '?' if tol is None else '%.3f' % tol,
                        '맞음' if ok else '다름', speed, fmt(m_nom and m_nom[key])))

                if not ok:
                    fails.append(key)

            verdict('%s: 모델 (SITL 의 진입 속도) 과 허용폭 안에서 맞는다' % name, not fails,
                    '다른 항목: %s' % (', '.join(fails) or '없음'))
            rows.append({'config': config, 'speed': speed, 'stick': stick, 'leg': leg['leg'], 'u_stick': u_stick,
                         'sitl': sitl, 'model_entry': m_ent, 'model_table': m_nom, 'fails': fails,
                         'summary': os.path.relpath(path, os.path.dirname(HERE))})

    out = os.path.join(HERE, 'd1_sitl_compare_out.json')
    json.dump(rows, open(out, 'w'), indent=1)
    note('\n기록 %s' % out)
    return summary()


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
