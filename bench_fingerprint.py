#!/usr/bin/env python3
"""Allocator fingerprint at the pins: a fixed set of stick vectors in Manual,
each held 4 s, all 8 channels read as the median pulse width over the last
second of the hold. Meant to be run before and after a refactor and diffed.
The run fails unless the FC's heartbeat reported Manual at the end of every
hold: in any other mode the yaw vectors carry loop output, and the diff would
blame the firmware.
"""
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import la_mcp  # noqa: E402
import bench_yaw  # noqa: E402
from bench_yaw import mark, events, MODE_MANUAL  # noqa: E402

HOLD = 4.0
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fp_out')

# (label, roll, pitch, yaw, z)   z 0..1000 -> throttle -1..1 -> thrust (t+1)/2
VECTORS = [
    ('idle', 0, 0, 0, 0),
    ('thrust 0.5', 0, 0, 0, 500),
    ('thrust 1.0', 0, 0, 0, 1000),
    ('yaw +0.5', 0, 0, 0.5, 0),
    ('yaw -0.5', 0, 0, -0.5, 0),
    ('yaw +0.015 (band)', 0, 0, 0.015, 0),
    ('roll +0.5', 0.5, 0, 0, 0),
    ('roll -0.5 + thrust 0.5', -0.5, 0, 0, 500),
    ('pitch +0.5', 0, 0.5, 0, 0),
    ('pitch -0.5 + thrust 0.5', 0, -0.5, 0, 500),
    ('yaw +0.5 + thrust 0.5', 0, 0, 0.5, 500),
    ('combo r.3 p-.3 y.2 t.5', 0.3, -0.3, 0.2, 500),
    ('idle end', 0, 0, 0, 0),
]


def main():
    os.makedirs(OUT, exist_ok=True)
    fc = bench_yaw.Fc()
    saved = fc.get_param('COM_DISARM_PRFLT')

    if saved is None:
        print('COM_DISARM_PRFLT 를 못 읽었다'); return 2

    params = {n: fc.get_param(n) for n in ('CA_AIRFRAME', 'CA_AIRSHIP_GRP', 'CA_AIRSHIP_TLMIN', 'CA_AIRSHIP_TLMAX',
                                           'CA_AIRSHIP_TLT_R', 'CA_AIRSHIP_CS_K', 'CA_AIRSHIP_TAIL', 'CA_SV_CS_COUNT',
                                           'COM_RC_IN_MODE', 'MAN_DEADZONE', 'AS_YAWRATE_P', 'AS_YAWRATE_I',
                                           'AS_YAWRATE_MAX', 'AS_YR_INT_LIM', 'AS_YAWRATE_FF')}
    print('params:', json.dumps(params))
    bench_yaw.warn_rc_in_mode(params.get('COM_RC_IN_MODE'))

    if not la_mcp.available():
        print('Logic 2 MCP 없음'); return 2

    capture_s = int(len(VECTORS) * HOLD + 25)
    capture_id, t_capture = la_mcp.start(capture_s)
    mark('캡처 %s 시작 (%d s)' % (capture_id, capture_s))
    marks, modes = [], {}
    ok = False

    bench_yaw.interrupt_on_signals()

    try:
        fc.set_param('COM_DISARM_PRFLT', -1.0)
        fc.run(1)
        fc.set_mode(MODE_MANUAL)
        fc.run(2)

        if fc.arm():
            fc.run(1)
            ok = True

            for label, roll, pitch, yaw, z in VECTORS:
                fc.sticks = (roll, pitch, yaw, z)
                t0 = time.time()
                mark('V %s' % label)
                fc.run(HOLD)
                modes[label] = fc.main_mode
                marks.append((label, t0 - t_capture))

                if not fc.armed:
                    mark('무장 풀림'); ok = False; break

    finally:
        ok_restore = bench_yaw.restore(fc, saved)

    export_dir = os.path.join(OUT, 'export')
    la_mcp.finish(capture_id, os.path.join(OUT, 'fingerprint.sal'), export_dir)
    rows = la_mcp.widths(export_dir, window=0.1)
    table = {}

    print('\n%-26s' % 'vector' + ''.join('%9s' % c.split()[0] + c.split()[-1][:1] for c in la_mcp.CHANNELS))

    for label, t0 in marks:
        vals = [la_mcp.median(rows.get(ch, []), t0 + HOLD - 1.0, t0 + HOLD) for ch in range(8)]
        table[label] = vals
        print('%-26s' % label + ''.join('%10.0f' % v for v in vals))

    json.dump({'params': params, 'table': table, 'modes': modes, 'hold': HOLD, 'events': events},
              open(os.path.join(OUT, 'fingerprint.json'), 'w'), indent=1)

    # Liveness: the sticks must have reached the pins. Idle and full thrust
    # have to separate on some channel, or every vector was measured with the
    # vehicle ignoring this script and the whole table is silence.
    idle, full = table.get('idle'), table.get('thrust 1.0')
    live = bool(idle and full) and max((abs(a - b) for a, b in zip(idle, full)
                                        if a == a and b == b), default=0.0) > 50.0

    if not live:
        print('\nidle 와 thrust 1.0 이 어느 핀에서도 50 us 넘게 갈리지 않는다.')
        print('스틱이 핀까지 닿지 않았다는 뜻이다. 이 표는 비교에 쓰지 않는다.')

    mode_ok = all(mode == MODE_MANUAL for mode in modes.values())

    if not mode_ok:
        print('모드 확인 실패: 벡터별 FC 모드 %s' % modes)

    return 0 if (ok and ok_restore and live and mode_ok) else 1


if __name__ == '__main__':
    sys.exit(main())
