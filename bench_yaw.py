#!/usr/bin/env python3
"""Bench check for the airship yaw axis, measured at the output pins.

The FMU sits on the desk, so the vehicle never rotates: the yaw rate error
stays at whatever the stick asks for. That makes the two halves of the
controller separately visible at the pins — the proportional term as a step
the moment the stick moves, the integral term as a straight ramp after it.

    프로펠러를 뺀다. ESC 와 서보 전원은 필요할 때만 넣는다.

Usage:
    python3 bench_yaw.py            로직 분석기와 함께
    python3 bench_yaw.py --no-la    스틱만 넣고 관측은 직접

The script owns the FMU port (fmu_link.PORT) alone: close QGroundControl first.
"""
import json
import math
import os
import signal
import sys
import time

from pymavlink import mavutil

import fmu_link

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import la_mcp  # noqa: E402

PORT = fmu_link.PORT
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'bench_out')
CAPTURE_S = 105

MODE_MANUAL, MODE_ACRO, MODE_STABILIZED = 1, 5, 7
MODE_NAMES = {MODE_MANUAL: 'Manual', MODE_ACRO: 'Acro', MODE_STABILIZED: 'Stabilized'}

# Stick deflections. 0.15 lands the torque demand inside the allocator's tilt
# steer band (0.02), 0.5 well outside it.
SMALL, BIG = 0.15, 0.5

events = []


def mark(text):
    events.append({'t': time.time(), 'text': text})
    print('  %s  %s' % (time.strftime('%H:%M:%S'), text), flush=True)


def warn_rc_in_mode(value):
    """Mark a COM_RC_IN_MODE that lets a powered receiver outrank the sent sticks.

    The symptom is pins that ignore every stick this script sends, a table
    flat everywhere, which reads as "unchanged" against any other flat one.
    """
    if value not in (1, 1.0):
        mark('COM_RC_IN_MODE=%s: RC may outrank the sent sticks, set it to 1 for the run' % value)


def deadzone(x, dz):
    """A stick through MAN_DEADZONE: clamped to [-1, 1], zero within dz, the rest rescaled to full range."""
    x = max(-1.0, min(1.0, x))
    return 0.0 if abs(x) <= dz else (x - (dz if x > 0 else -dz)) / (1.0 - dz)


class Fc:
    """One process, one serial port: sticks, commands and telemetry share it.

    run() sends `sticks` and tracks the FC's armed state and PX4 main mode
    from its heartbeat. `yaw_stick` sets the yaw axis alone.
    """

    def __init__(self):
        # fmu_link.connect retries the open: pymavlink can throw from inside
        # wait_heartbeat when the port is opened mid-stream (a TypeError in
        # its per-type instance table, seen 2026-09-23 on a clean FMU), and a
        # fresh connection a moment later parses the same stream cleanly.
        self.link = fmu_link.connect(port=PORT)

        if self.link is None:
            raise RuntimeError('FMU 가 응답하지 않는다')

        self.sys = self.link.target_system
        self.comp = self.link.target_component
        # (roll, pitch, yaw, z): roll, pitch and yaw in -1..1, z sent as is,
        # 0..1000. z = 0 is throttle -1: no forward thrust.
        self.sticks = (0.0, 0.0, 0.0, 0)
        self.main_mode = None          # PX4 main mode from the FC's heartbeat custom_mode
        self.armed = False
        self._last_heartbeat = 0.0

    @property
    def yaw_stick(self):
        return self.sticks[2]

    @yaw_stick.setter
    def yaw_stick(self, value):
        roll, pitch, _, z = self.sticks
        self.sticks = (roll, pitch, value, z)

    def run(self, seconds):
        """Hold the sticks for `seconds` while draining incoming messages."""
        end = time.time() + seconds

        while time.time() < end:
            now = time.time()

            if now - self._last_heartbeat >= 1.0:
                self.link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                             mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                self._last_heartbeat = now

            roll, pitch, yaw, z = self.sticks
            self.link.mav.manual_control_send(self.sys, int(pitch * 1000), int(roll * 1000), int(z),
                                              int(yaw * 1000), 0)
            until = now + 0.05

            while time.time() < until:
                message = self.link.recv_match(blocking=True, timeout=0.01)

                if message is None:
                    continue

                kind = message.get_type()

                if kind == 'HEARTBEAT' and message.get_srcSystem() == self.sys \
                        and message.type != mavutil.mavlink.MAV_TYPE_GCS:
                    self.armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.main_mode = (message.custom_mode >> 16) & 0xFF
                elif kind == 'STATUSTEXT':
                    mark('FC: ' + message.text.strip())

    def command(self, command, *params):
        self.link.mav.command_long_send(self.sys, self.comp, command, 0,
                                        *(list(params) + [0] * (7 - len(params))))

    def set_param(self, name, value):
        self.link.mav.param_set_send(self.sys, self.comp, name.encode(), float(value),
                                     mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        mark('파라미터 %s = %s' % (name, value))

    def get_param(self, name):
        m = fmu_link.read_param(self.link, name)
        return None if m is None else fmu_link.decode(m)[0]

    def set_mode(self, main_mode):
        # COMMAND_LONG DO_SET_MODE: param2 is the PX4 main mode itself
        self.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main_mode, 0)
        mark('모드 -> %s' % MODE_NAMES.get(main_mode, main_mode))

    def arm(self):
        self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        mark('무장 요청')

        for _ in range(20):
            self.run(0.5)
            if self.armed:
                break

        mark('무장 %s' % ('성공' if self.armed else '실패'))
        return self.armed

    def disarm(self):
        self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196)
        mark('강제 해제')

        for _ in range(10):
            self.run(0.5)
            if not self.armed:
                break


def hold(fc, seconds, label):
    """Hold the sticks, and stop the run if the vehicle disarmed under us.

    A silent disarm mid-sequence produces a capture full of idle pulses that
    reads like a frozen integrator. Losing the run is fine; misreading one
    is not.
    """
    fc.run(seconds)

    if not fc.armed:
        mark('무장이 풀렸다 (%s). 여기서 멈춘다.' % label)
        return False

    return True


def sequence(fc):
    """Phases ordered so the small-stick test starts with the pods level.

    Only the start of phase A is level: once the integral lifts the demand
    past the allocator's steer band the tilts sweep, which is the thing
    레슨 19 exists to restore.
    """
    fc.set_mode(MODE_MANUAL)
    fc.run(2)

    if not fc.arm():
        mark('무장하지 못했다. Hold 상태에서는 위치가 필요하니 Manual 로 바꾼 뒤 시도한다.')
        return False

    fc.run(2)

    fc.set_mode(MODE_ACRO)
    fc.run(3)

    fc.yaw_stick = SMALL
    mark('A. Acro 작은 스틱 0.15 — 틸트 밴드 안')

    if not hold(fc, 25, 'A'):
        return False

    fc.yaw_stick = 0
    mark('A. 놓음')
    fc.run(5)

    fc.yaw_stick = BIG
    mark('B. Acro 스틱 0.5 — P 계단과 I 경사')

    if not hold(fc, 12, 'B'):
        return False

    fc.yaw_stick = 0
    mark('B. 놓음 — 적분기는 그대로 유지')
    fc.run(6)

    fc.yaw_stick = -BIG
    mark('C. Acro 스틱 -0.5 — 적분기 되감기')

    if not hold(fc, 8, 'C'):
        return False

    fc.yaw_stick = 0
    mark('C. 놓음')
    fc.run(3)

    fc.set_mode(MODE_STABILIZED)
    fc.run(3)
    fc.yaw_stick = BIG
    mark('D. Stabilized 스틱 0.5 — Acro 와 같은 법칙')

    if not hold(fc, 7, 'D'):
        return False

    fc.yaw_stick = 0
    mark('D. 놓음')
    fc.run(3)

    fc.set_mode(MODE_MANUAL)
    fc.run(2)
    fc.yaw_stick = BIG
    mark('E. Manual 스틱 0.5 — 통과 경로(스틱 = 토크)')

    if not hold(fc, 5, 'E'):
        return False

    fc.yaw_stick = 0
    mark('E. 놓음')
    fc.run(3)

    fc.disarm()
    return True


def report(export_dir, t_capture, has_loop, gains):
    rows = la_mcp.widths(export_dir)
    motor = rows[la_mcp.MOTOR_P]
    tilt = rows[la_mcp.TILT_S]

    def when(prefix):
        for event in events:
            if event['text'].startswith(prefix):
                return event['t'] - t_capture
        return None

    print('\n=== 핀에서 본 결과 (포트 쪽 모터, µs) ===')
    print('%-42s %8s %8s %10s' % ('구간', '시작', '끝', '기울기'))

    for prefix, label, length in (('A. Acro 작은', '작은 스틱 0.15 (25 초)', 25),
                                  ('B. Acro 스틱 0.5', '스틱 0.5 (12 초)', 12),
                                  ('B. 놓음', '놓은 뒤 유지 (6 초)', 6),
                                  ('C. Acro 스틱 -0.5', '스틱 -0.5 되감기 (8 초)', 8),
                                  ('D. Stabilized', 'Stabilized 0.5 (7 초)', 7),
                                  ('E. Manual', 'Manual 통과 0.5 (5 초)', 5)):
        start = when(prefix)

        if start is None:
            continue

        print('%-42s %8.0f %8.0f %8.1f/s' % (label, la_mcp.at(motor, start + 1.0),
                                             la_mcp.at(motor, start + length - 1.0),
                                             la_mcp.slope(motor, start + 2.0, start + length - 1.0)))

    # The passthrough phase is the ruler: Manual sends the stick straight
    # through as torque, so this one number converts every other number on
    # the pin into normalized torque. It is also the liveness proof — if the
    # sticks never reached the FMU, this stays at idle and nothing below
    # means anything.
    manual = when('E. Manual')
    scale = float('nan')

    if manual is not None:
        idle = la_mcp.at(motor, manual - 1.0)
        held = la_mcp.at(motor, manual + 2.0)
        scale = (held - idle) / BIG
        print('\n핀 눈금: Manual 통과 0.5 에서 %.0f -> %.0f µs, 토크 1 당 %.0f µs' % (idle, held, scale))

        if not (held - idle) > 100:
            print('  스틱이 FMU 에 닿지 않았다. 아래 숫자는 모두 무의미하다.')
            print('  COM_RC_IN_MODE 가 1(MAVLink 전용)인지 보고, 아니면 그렇게 바꾸고 재부팅한다.')
            return False

    if not has_loop or gains is None:
        return True

    rate_max, dz, gain_p, gain_i, ff = gains
    ff = 0.0 if ff is None else ff

    def setpoint(stick):
        """스틱 -> 지시 각속도 [rad/s]: 데드존을 뺀 뒤 다시 폈다."""
        return abs(deadzone(stick, dz)) * math.radians(rate_max)

    small = when('A. Acro 작은')

    if small is not None:
        measured = la_mcp.slope(motor, small + 3.0, small + 22.0)
        expected = gain_i * setpoint(SMALL) * scale
        tilt_moved = abs(la_mcp.at(tilt, small + 24.0) - la_mcp.at(tilt, small + 1.0)) > 50
        print('\n작은 스틱 경사 %.2f µs/s (예상 %.2f), 틸트가 %s'
              % (measured, expected, '움직였다' if tilt_moved else '가만히 있었다'))
        print('  레슨 16 에서는 0 에 가깝고 틸트가 가만히 있는다 (할당기가 포화라고 거짓 보고).')
        print('  레슨 18 로는 이 기체가 안 바뀐다: 조종면이 달려 있으면 다른 길로 보고한다.')
        print('  레슨 19 뒤에 예상값 근처로 오르고 ~10 초 뒤 틸트가 스윕한다.')

    big = when('B. Acro 스틱 0.5')

    if big is not None:
        # The stick step and the integral ramp start at the same instant, so
        # a difference read a second later is P plus a second of I. Fit the
        # ramp and run it back to the instant the stick moved: what is left
        # is the P term alone.
        ramp = la_mcp.slope(motor, big + 2.0, big + 11.0)
        step = la_mcp.at(motor, big + 2.0) - ramp * 2.0 - la_mcp.at(motor, big - 1.0)
        print('\n스틱 0.5 계단 %+.0f µs (예상 %+.0f), 경사 %.1f µs/s (예상 %.1f)'
              % (step, (gain_p + ff) * setpoint(BIG) * scale, ramp, gain_i * setpoint(BIG) * scale))
        print('  계단 / 경사 = (P + FF) / I = %.1f 초. 눈금을 몰라도 이 비는 맞아야 한다.'
              % ((gain_p + ff) / gain_i))

    return True


def interrupt_on_signals():
    """Turn SIGTERM and SIGHUP into KeyboardInterrupt, the path restore()
    already survives, so a killed run or a closed terminal still disarms and
    puts COM_DISARM_PRFLT back. Call it before the try that clears it."""
    def _raise(signum, frame):
        raise KeyboardInterrupt('signal %d' % signum)

    signal.signal(signal.SIGTERM, _raise)
    signal.signal(signal.SIGHUP, _raise)


def restore(fc, saved):
    """Put the vehicle and the parameter back, whatever went wrong.

    Every step is guarded on its own and catches BaseException, because the
    thing most likely to interrupt a bench run is a second Ctrl-C landing
    inside the cleanup. A skipped step here leaves an armed vehicle or a
    permanently disabled preflight disarm timer: PX4 writes a MAVLink
    PARAM_SET straight to flash.
    """
    for label, action in (
            ('스틱 놓기', lambda: setattr(fc, 'sticks', (0.0, 0.0, 0.0, 0))),
            ('강제 해제', fc.disarm),
            ('파라미터 되돌리기', lambda: fc.set_param('COM_DISARM_PRFLT', saved)),
    ):
        try:
            action()
            fc.run(1)
        except BaseException as error:          # noqa: B036 - cleanup must not be skippable
            print('!! %s 실패: %s' % (label, error), file=sys.stderr)

    try:
        back = fc.get_param('COM_DISARM_PRFLT')
    except BaseException as error:              # noqa: B036
        back = None
        print('!! 되돌린 값을 못 읽었다: %s' % error, file=sys.stderr)

    if back is None or abs(back - saved) > 1e-3:
        print('\n!! COM_DISARM_PRFLT 가 %s 다. 원래 값은 %s.' % (back, saved))
        print('!! 직접 되돌린다: px4-param set COM_DISARM_PRFLT %s' % saved)
        return False

    if fc.armed:
        print('\n!! 아직 무장 상태다. 전원을 내리거나 px4-commander disarm -f.')
        return False

    return True


def main():
    use_la = '--no-la' not in sys.argv
    os.makedirs(OUT, exist_ok=True)

    fc = Fc()
    gains = [fc.get_param(name) for name in
             ('AS_YAWRATE_MAX', 'MAN_DEADZONE', 'AS_YAWRATE_P', 'AS_YAWRATE_I', 'AS_YAWRATE_FF')]
    has_loop = gains[2] is not None
    print('요 각속도 루프: %s' % ('있음 (레슨 11 이후)' if has_loop else '없음 (기준선)'))

    if has_loop:
        # AS_YAWRATE_FF exists from d2647379af on; a firmware without it has none
        print('  AS_YAWRATE_FF %s' % ('읽지 못함, 0 으로 예측한다' if gains[4] is None else gains[4]))

    if has_loop and use_la and None in gains[:4]:
        print('게인을 다 읽지 못했다 (MAX, DZ, P, I = %s). 예측을 할 수 없으니 돌리지 않는다.' % gains[:4])
        return 2

    # COM_RC_IN_MODE 3 locks the manual-control source to whichever link goes
    # valid first and keeps it until reboot. With an RC receiver wired, that
    # is the receiver, and every stick this script sends is discarded while
    # arming and telemetry still work — a run of flat traces that looks
    # exactly like the frozen integrator of 레슨 16.
    rc_mode = fc.get_param('COM_RC_IN_MODE')

    if rc_mode is not None and int(rc_mode) != 1:
        print('경고: COM_RC_IN_MODE 가 %d 다. 1(MAVLink 전용)로 두고 재부팅하는 것이 안전하다.' % int(rc_mode))
        print('      그대로 진행하되, 마지막 Manual 통과 구간이 스틱이 닿았는지 판정한다.')

    saved = fc.get_param('COM_DISARM_PRFLT')

    if saved is None:
        print('COM_DISARM_PRFLT 를 못 읽었다. 되돌릴 값을 모르면 바꾸지 않는다.')
        return 2

    if use_la and not la_mcp.available():
        print('Logic 2 의 MCP 서버에 닿지 않는다. --no-la 로 돌리거나 Logic 2 를 켠다.')
        return 2

    capture_id, t_capture = (None, time.time())

    if use_la:
        capture_id, t_capture = la_mcp.start(CAPTURE_S)
        mark('캡처 %s 시작' % capture_id)

    interrupt_on_signals()

    try:
        fc.set_param('COM_DISARM_PRFLT', -1.0)   # bench: no preflight disarm timer
        fc.run(1)
        ok = sequence(fc)
    finally:
        ok_restore = restore(fc, saved)

    if use_la:
        export_dir = os.path.join(OUT, 'export')
        la_mcp.finish(capture_id, os.path.join(OUT, 'bench_yaw.sal'), export_dir)
        mark('캡처 저장 완료')

        if ok and not report(export_dir, t_capture, has_loop, gains if has_loop else None):
            ok = False

    json.dump({'t_capture': t_capture, 'events': events},
              open(os.path.join(OUT, 'events.json'), 'w'), indent=1)
    return 0 if (ok and ok_restore) else 1


if __name__ == '__main__':
    sys.exit(main())
