#!/usr/bin/env python3
"""The yaw rate loop at the pins on a surface-only (2500) allocation.

    프로펠러를 뺀다. ESC 전원을 끈다. QGC 를 닫는다. Logic 2 를 띄운다.
    python3 bench_rate_hold.py          # the 2500 allocation must already be live
    python3 bench_rate_hold.py --loop   # predict the closed loop (firmware without e550ec6597, e.g. 9535f8abcd)

The FMU sits on the desk, so the yaw rate reads zero and the rate error is
whatever the stick asks for. On 2500 the rudders are the only yaw effector
and take the loop's torque 1:1, so with the loop closed the pins show the
loop itself: a P step when the stick moves, the integral ramping at
AS_YAWRATE_I x setpoint, a cap at P x setpoint + AS_YR_INT_LIM, and, after
the stick is released, whatever the integrator holds.

Predictions are computed from the parameters the FMU reports, not restated,
and the run refuses a store that is not the 2500 allocation the predictions
model (the airship allocator, CA_AIRFRAME 16, with two equal yaw-only
rudders on CS2/CS3 and no other yaw effector). Firmware from e550ec6597
("keep yaw stick as torque on surface-only yaw") on keeps the yaw stick as
torque in every mode on such an airframe, so the default prediction is
passthrough: Stabilized reads like Manual. --loop predicts the closed loop
instead, for firmware without that commit (9535f8abcd, the first run,
head_20260925/t2_rate_hold.log). The design review (2026-09-25) predicted
from the code that with CA_AIRSHIP_CS_K 1 no yaw saturation is ever flagged
below full rudder, so the integral winds to its limit at rest and stays
there after release; --loop measures that on such an image. The chosen
prediction is printed and stored in rate_hold.json.
"""
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bench_yaw  # noqa: E402
import la_mcp  # noqa: E402
from bench_yaw import MODE_MANUAL, MODE_STABILIZED, mark, events  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rate_out')
RUDDERS = (2, 3)          # MAIN3/MAIN4 = CS2/CS3, TRQ_Y 0.5 each
START = (0.4, 0.9)        # s after the stick moves: the window the start reading is the median of
SEQUENCE = [              # (label, mode, yaw stick, seconds)
    ('manual yaw +0.5', MODE_MANUAL, 0.5, 4.0),
    ('manual centre', MODE_MANUAL, 0.0, 3.0),
    ('stabilized yaw +0.5', MODE_STABILIZED, 0.5, 35.0),
    ('stabilized released', MODE_STABILIZED, 0.0, 15.0),
    ('stabilized yaw +1.0', MODE_STABILIZED, 1.0, 10.0),
    ('stabilized released again', MODE_STABILIZED, 0.0, 8.0),
    ('manual centre end', MODE_MANUAL, 0.0, 5.0),
]


def main():
    model = 'loop' if '--loop' in sys.argv else 'passthrough'
    os.makedirs(OUT, exist_ok=True)
    fc = bench_yaw.Fc()
    saved = fc.get_param('COM_DISARM_PRFLT')
    names = ('SYS_AUTOSTART', 'CA_AIRFRAME', 'CA_AIRSHIP_GRP', 'CA_AIRSHIP_TLMIN', 'CA_AIRSHIP_TLMAX', 'CA_AIRSHIP_CS_K',
             'CA_AIRSHIP_TAIL', 'CA_SV_CS_COUNT', 'CA_SV_CS0_TRQ_Y', 'CA_SV_CS1_TRQ_Y',
             'CA_SV_CS2_TRQ_R', 'CA_SV_CS2_TRQ_P', 'CA_SV_CS2_TRQ_Y', 'CA_SV_CS3_TRQ_R', 'CA_SV_CS3_TRQ_P', 'CA_SV_CS3_TRQ_Y',
             'MAN_DEADZONE', 'AS_YAWRATE_P', 'AS_YAWRATE_I', 'AS_YAWRATE_MAX', 'AS_YR_INT_LIM', 'COM_RC_IN_MODE')
    p = {n: fc.get_param(n) for n in names}
    print('params:', json.dumps(p))
    print('prediction:', model)

    if None in p.values() or saved is None:
        print('파라미터를 다 읽지 못했다'); return 2

    # The firmware closes the loop on any allocator but the airship one, and
    # the rudder gain below is the pseudo-inverse of two equal yaw-only
    # columns: anything else is mispredicted, not measured
    if (p['CA_AIRFRAME'] != 16 or p['CA_AIRSHIP_GRP'] != 0 or p['CA_AIRSHIP_TLMIN'] != 0
            or p['CA_AIRSHIP_TLMAX'] != 0 or p['CA_AIRSHIP_TAIL'] != 0 or p['CA_SV_CS_COUNT'] != 4
            or p['CA_SV_CS0_TRQ_Y'] != 0 or p['CA_SV_CS1_TRQ_Y'] != 0
            or any(p['CA_SV_CS%d_TRQ_%s' % (n, axis)] != 0 for n in (2, 3) for axis in 'RP')
            or p['CA_SV_CS2_TRQ_Y'] == 0 or abs(p['CA_SV_CS2_TRQ_Y'] - p['CA_SV_CS3_TRQ_Y']) > 1e-6):
        print('2500 할당 (CA_AIRFRAME 16, GRP 0, TLMIN/TLMAX 0, TAIL 0, 면 4개, 방향타 CS2/CS3 만 같은 TRQ_Y) 이 아니다'); return 2

    bench_yaw.warn_rc_in_mode(p['COM_RC_IN_MODE'])

    if not la_mcp.available():
        print('Logic 2 MCP 없음'); return 2

    kp, ki, lim = p['AS_YAWRATE_P'], p['AS_YAWRATE_I'], p['AS_YR_INT_LIM']
    rmax = math.radians(p['AS_YAWRATE_MAX'])
    dz = p['MAN_DEADZONE']
    gain = 1.0 / (2.0 * p['CA_SV_CS2_TRQ_Y'])          # pinv of two equal TRQ_Y columns: rudder = torque x this
    us = lambda u: 1500.0 + 500.0 * max(-1.0, min(1.0, gain * u))

    capture_s = int(sum(s[3] for s in SEQUENCE) + 20)
    capture_id, t_capture = la_mcp.start(capture_s)
    mark('캡처 %s 시작 (%d s)' % (capture_id, capture_s))
    marks, ok, mode_ok = [], False, []

    bench_yaw.interrupt_on_signals()

    try:
        fc.set_param('COM_DISARM_PRFLT', -1.0)
        fc.run(1)
        fc.set_mode(MODE_MANUAL)
        fc.run(2)

        if fc.arm():
            fc.run(1)
            ok = True
            mode_now = MODE_MANUAL

            for label, mode, yaw, seconds in SEQUENCE:
                if mode != mode_now:
                    fc.set_mode(mode)
                    mode_now = mode
                    fc.run(1.0)

                fc.sticks = (0.0, 0.0, yaw, 0)
                t0 = time.time()
                mark('S %s (FC main mode %s)' % (label, fc.main_mode))
                fc.run(seconds)
                # Passthrough makes Stabilized read like Manual at the pins, so
                # a segment only counts if the FC said it was in that mode
                mode_ok.append(fc.main_mode == mode)
                marks.append((label, t0 - t_capture, seconds, mode, yaw))

                if not fc.armed:
                    mark('무장 풀림'); ok = False; break
    finally:
        restored = bench_yaw.restore(fc, saved)

    export_dir = os.path.join(OUT, 'export')
    la_mcp.finish(capture_id, os.path.join(OUT, 'rate_hold.sal'), export_dir)
    rows = la_mcp.widths(export_dir, window=0.1)

    def median(ch, start, end):
        return la_mcp.median(rows.get(ch, []), start, end)

    # The loop from the code: e = setpoint (the gyro reads zero), u = P e + I,
    # I integrates Ki e clamped at +-lim, reset when the loop opens (Manual).
    integ, report = 0.0, []
    print('\n%-28s %9s %9s %9s %9s %9s %11s %9s %9s' % ('segment', 'start', 'pred', 'end', 'pred', 'MAIN4', 'slope us/s', 'pred', 'motors'))

    for label, t0, seconds, mode, yaw in marks:
        sp = bench_yaw.deadzone(yaw, dz) * rmax if mode != MODE_MANUAL else 0.0

        if mode == MODE_MANUAL or model != 'loop':
            integ = 0.0
            start_pred = end_pred = us(yaw)
            slope_pred = 0.0
        else:
            # The integral ramps from the moment the stick moves, and the
            # median of a monotone ramp over START is its value at the middle
            start_pred = us(kp * sp + max(-lim, min(lim, integ + ki * sp * sum(START) / 2)))
            integ_end = max(-lim, min(lim, integ + ki * sp * seconds))
            end_pred = us(kp * sp + integ_end)
            slope_pred = 0.0 if abs(integ) >= lim and (integ > 0) == (sp > 0) else 500.0 * gain * ki * sp
            integ = integ_end

        start = median(RUDDERS[0], t0 + START[0], t0 + START[1])
        end = median(RUDDERS[0], t0 + seconds - 1.0, t0 + seconds)
        other = median(RUDDERS[1], t0 + seconds - 1.0, t0 + seconds)
        s = la_mcp.slope(rows.get(RUDDERS[0], []), t0 + 1.0, t0 + min(seconds, 12.0))
        motors = max(median(6, t0, t0 + seconds), median(7, t0, t0 + seconds))
        elev = (median(0, t0 + seconds - 1.0, t0 + seconds), median(1, t0 + seconds - 1.0, t0 + seconds))
        report.append({'label': label, 'start': start, 'start_pred': start_pred, 'end': end, 'end_pred': end_pred,
                       'main4_end': other, 'slope': s, 'slope_pred': slope_pred, 'motors': motors, 'elevators': elev})
        print('%-28s %9.1f %9.1f %9.1f %9.1f %9.1f %11.2f %9.2f %9.1f' % (label, start, start_pred, end, end_pred, other, s, slope_pred, motors))

    json.dump({'params': p, 'prediction': model, 'segments': report, 'events': events},
              open(os.path.join(OUT, 'rate_hold.json'), 'w'), indent=1)
    finite = all(all(v == v for v in (r['start'], r['end'], r['main4_end'])) for r in report)
    agree = finite and all(mode_ok) and len(mode_ok) == len(SEQUENCE) and all(
        abs(r['start'] - r['start_pred']) <= 3.0 and abs(r['end'] - r['end_pred']) <= 3.0
        and abs(r['main4_end'] - r['end']) <= 1.0 for r in report)
    if not all(mode_ok):
        print('모드 확인 실패: 구간별 FC 모드 일치 %s' % mode_ok)
    print('\n%s: 핀이 코드에서 계산한 %s 예측과 %s (시작/끝 3 us, 두 방향타 1 us 이내, 모드는 하트비트로 확인)%s'
          % ('통과' if agree else '실패', model, '맞는다' if agree else '다르다', '' if restored else '; 복구 확인 필요'))
    return 0 if ok and agree and restored else 1


if __name__ == '__main__':
    sys.exit(main())
