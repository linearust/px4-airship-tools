#!/usr/bin/env python3
"""Does the mid-air disarm mitigation hold on the Cloudship, and can this check
tell if it did not?

    GZ_TARGET=gz_cloudship ./gz_run.sh ./cloudship_land_disarm_check.py

The bug: AirshipLandDetector::_get_landed_state() returns true the moment the
nav state is AUTO_LAND (AirshipLandDetector.cpp:51-60) - no velocity, thrust,
altitude or ground condition enters it - and the commander's handleAutoDisarm()
(Commander.cpp:2465-2507) then disarms a "landed" vehicle after COM_DISARM_LAND
seconds. On an airship that is 20 m up. The shipped mitigation is
COM_DISARM_LAND -1 in rc.airship_defaults, which 2510 sources; it was verified
on the vectored airship in August and never on this airframe.

Two-sided, because a mitigation check that only confirms "still armed" cannot
tell a mitigation that works from a trigger that never fired:

  (a) with the shipped -1: arm, enter Land at 20 m, prove the detector DID
      report landed, then hold for longer than every commander timer and prove
      the vehicle is STILL ARMED.
  (b) with the mitigation removed at runtime (COM_DISARM_LAND 3 - not 2, so a
      read-back cannot be confused with the firmware default): the same
      sequence must disarm, with latest_disarming_reason 6 (landing), inside
      the timer. That is the bug reproducing, and it is what proves (a) meant
      something.

Then the parameter goes back and is read back. Every time is PX4's clock, from
the topics' own stamps; the wall clock only bounds the polling.
"""
import sys
import time

import gz_link
import gz_model
from report import note, summary, verdict
from sitl_link import (MODEL, MODEL_NAME, Sticks, commander, field, finite, listener,
                       param_set, param_show, wait_for_ekf)

AUTO_LAND = 18                # vehicle_status.nav_state
DESCEND = 12                  # what the framework substitutes when Land cannot run
ARMED, DISARMED = 2, 1        # vehicle_status.arming_state
REASON_LANDING = 6            # vehicle_status.latest_disarming_reason
HOLD_S = 20.0                 # side (a): PX4 seconds in Land, armed
T_B = 3.0                     # side (b): COM_DISARM_LAND we set, seconds
WALL_FACTOR = 3               # wall-clock bound on a sim that may run slow
MIN_HEIGHT_M = 5.0            # "in the air" for a hull spawned at 20 m
REPUBLISH_S = 1.0             # LandDetector.cpp:143 republishes at 1 Hz


def status():
    """One sample of what the commander and the land detector say."""
    vs = listener('vehicle_status')
    ld = listener('vehicle_land_detected')
    return {
        'ts': field(vs, 'timestamp'),
        'arming': field(vs, 'arming_state'),
        'nav': field(vs, 'nav_state'),
        'nav_ts': field(vs, 'nav_state_timestamp'),
        'reason': field(vs, 'latest_disarming_reason'),
        'takeoff': field(vs, 'takeoff_time'),
        'landed': field(ld, 'landed'),
        'landed_ts': field(ld, 'timestamp'),
        'maybe': field(ld, 'maybe_landed'),
        'contact': field(ld, 'ground_contact'),
        'freefall': field(ld, 'freefall'),
    }


def height():
    """World z from gz. Not vehicle_local_position.z: the EKF origin is the
    20 m spawn point, so that reads ~0 in the air."""
    rows = gz_link.poses(MODEL, limit=5, timeout=15)
    return rows[-1][1][2] if rows else None


def arm_manual():
    commander('mode', 'manual')
    time.sleep(1)
    commander('arm')
    time.sleep(3)
    return status()


def enter_land():
    """Command Land and wait for the nav state to say so. A rejection is
    silent at the client (a forced mode change is always accepted; whether it
    RUNS is a different question), so the state is what is asserted."""
    commander('mode', 'auto:land')
    s = status()

    for _ in range(20):
        if s['nav'] == AUTO_LAND:
            break

        time.sleep(0.5)
        s = status()

    return s


def first_landed(s, t_land):
    """The first vehicle_land_detected message that says landed after Land.

    A message's timestamp is its publication time, not the transition time.
    The detector publishes when the state changes and then again every
    REPUBLISH_S (LandDetector.cpp:143-149), so a sample taken after a sleep
    reads the age of whichever republish is latest, and one run of this check
    called a 1.27 s old periodic message a 1.27 s detection latency. Sampling
    at once and stopping at the first landed message returns the transition
    message itself whenever that transition is younger than REPUBLISH_S, which
    is what the verdict's bound asserts: younger than a republish, so it is the
    transition, so the detector flipped on its first cycle after Land.
    """
    end = time.time() + WALL_FACTOR * 2.0

    while s['landed'] != 1.0 and time.time() < end:
        time.sleep(0.2)
        s = status()

    return s


def show(v, fmt='%s'):
    return fmt % v if finite(v) else '읽기실패'


def main():
    if not commander('status'):
        note('px4 에 붙지 못했다')
        return 3

    # A. The boot state has to be the shipped one, asserted rather than hoped.
    disarm_land = param_show('COM_DISARM_LAND')
    prflt = param_show('COM_DISARM_PRFLT')
    spool = param_show('COM_SPOOLUP_TIME')
    autostart = param_show('SYS_AUTOSTART')

    # Any physics variant of the gz Cloudship (the 2510 decade) is the vehicle.
    if disarm_land != -1.0 or gz_model.vehicle(MODEL_NAME) != 'cloudship' or autostart // 10 != 251:
        note('오염된 상태: COM_DISARM_LAND %s, SYS_AUTOSTART %s, 모델 %s'
             % (disarm_land, autostart, MODEL_NAME))
        return 2

    # The hold must outlast every timer the commander owns: COM_DISARM_LAND
    # (disabled here), COM_DISARM_PRFLT + COM_SPOOLUP_TIME (Commander.cpp:2480-
    # 2483, only before a takeoff is latched), and the 5 s kill-switch timer.
    if not finite(prflt, spool) or HOLD_S < prflt + spool + 5.0:
        note('HOLD_S %.0f 가 타이머 %s + %s + 5 보다 짧다' % (HOLD_S, prflt, spool))
        return 2

    z0 = height()
    note('시작: COM_DISARM_LAND %.0f, COM_DISARM_PRFLT %.0f, COM_SPOOLUP_TIME %.0f, 스폰 높이 %s m'
         % (disarm_land, prflt, spool, show(z0, '%.1f')))

    # B. Arm in Manual. On this detector, arming IS the takeoff: landed reads
    # !armed outside Land, so it flips True->False on arming and the commander
    # reads that edge as "Takeoff detected" (Commander.cpp:2322-2326). Without
    # that latch the preflight-inaction timer would disarm us and the wrong
    # bug would be under test.
    if not wait_for_ekf():
        verdict('EKF 가 유효해진다', False, '위치 추정이 유효해지지 않았다')
        return summary()

    time.sleep(10)
    sticks = Sticks()
    sticks.start()
    time.sleep(2)

    s = arm_manual()
    armed = s['arming'] == ARMED and field(listener('vehicle_control_mode'), 'flag_armed') == 1.0
    verdict('무장한다 (Manual)', armed,
            'arming_state %s, nav_state %s' % (show(s['arming']), show(s['nav'])))

    if not armed:
        sticks.stop = True
        return summary()

    verdict('무장만으로 이륙이 래치된다',
            s['landed'] == 0.0 and finite(s['takeoff']) and s['takeoff'] > 0,
            'landed %s, takeoff_time %s us' % (show(s['landed']), show(s['takeoff'], '%.0f')))

    # C. Side (a) trigger: Land, 20 m up, and the detector must say landed.
    s = enter_land()

    if s['nav'] != AUTO_LAND:
        verdict('Land 진입 (nav_state 18)', False,
                'nav_state %s%s' % (show(s['nav']),
                                    ' = Descend: Land 을 실행할 수 없어 대체됐다' if s['nav'] == DESCEND else ''))
        sticks.stop = True
        commander('disarm', '-f')
        return summary()

    t_land = s['nav_ts']
    s = first_landed(s, t_land)
    z = height()
    verdict('Land 진입 (nav_state 18)', True, 'nav_state_timestamp %s us' % show(t_land, '%.0f'))
    verdict('착륙 감지기가 공중에서 착륙을 보고한다',
            s['landed'] == 1.0 and finite(s['landed_ts'], t_land)
            and s['landed_ts'] - t_land < REPUBLISH_S * 1e6 and finite(z) and z > MIN_HEIGHT_M,
            'landed %s (Land 뒤 %s 초), maybe_landed %s, ground_contact %s, freefall %s, gz z %s m'
            % (show(s['landed']),
               show((s['landed_ts'] - t_land) / 1e6, '%+.2f') if finite(s['landed_ts'], t_land) else '?',
               show(s['maybe']), show(s['contact']), show(s['freefall']), show(z, '%.1f')))

    # D. Side (a) verdict: armed for HOLD_S of PX4 time, landed the whole way.
    rows = []
    last = s
    end = time.time() + WALL_FACTOR * HOLD_S

    while time.time() < end:
        last = status()
        rows.append(last)

        if not finite(last['ts'], last['arming'], last['landed']) or last['arming'] != ARMED:
            break

        if last['ts'] - t_land >= HOLD_S * 1e6:
            break

        time.sleep(1)

    z = height()
    held = (last['ts'] - t_land) / 1e6 if finite(last['ts'], t_land) else float('nan')
    verdict('(a) 완화 적용: 착륙 보고 중 %.0f 초 무장 유지 (COM_DISARM_LAND -1)' % HOLD_S,
            finite(held) and held >= HOLD_S
            and all(r['arming'] == ARMED and r['landed'] == 1.0 for r in rows),
            'PX4 시계 %s 초, 표본 %d, 마지막 arming_state %s, latest_disarming_reason %s, gz z %s m'
            % (show(held, '%.1f'), len(rows), show(last['arming']), show(last['reason']),
               show(z, '%.1f')))

    # E. Leave side (a) with a forced disarm. A plain one would also be
    # accepted here - disarm() lets a non-forced disarm through whenever the
    # detector says landed (Commander.cpp:756-775) - which is the bug's other
    # face and worth knowing, but not what this asserts.
    commander('disarm', '-f')
    time.sleep(3)
    s = status()
    note('(a) 종료: arming_state %s, reason %s, takeoff_time %s'
         % (show(s['arming']), show(s['reason']), show(s['takeoff'], '%.0f')))

    # F. Side (b): remove the mitigation at runtime and prove the write took.
    out = param_set('COM_DISARM_LAND', T_B)
    got = param_show('COM_DISARM_LAND')
    verdict('(b) 런타임 param set 이 반영된다', got == T_B,
            'param set -> %r, param show %s' % ((out or '').strip(), got))

    restored = None

    try:
        if got != T_B:
            return summary()

        # G. Re-arm 20 m up, disarmed, with landed True. Untested ground: first
        # light arms exactly once. arm() has no altitude check and home is
        # re-captured because landed is True (Commander.cpp:731-734).
        s = arm_manual()
        verdict('(b) 재무장 + 이륙 래치',
                s['arming'] == ARMED and s['landed'] == 0.0 and finite(s['takeoff']) and s['takeoff'] > 0,
                'arming_state %s, landed %s, takeoff_time %s us, nav_state %s'
                % (show(s['arming']), show(s['landed']), show(s['takeoff'], '%.0f'), show(s['nav'])))

        if s['arming'] != ARMED:
            return summary()

        # H. Land again; this time it has to disarm, for the bug's reason.
        s = enter_land()
        t_land = s['nav_ts']

        if s['nav'] != AUTO_LAND:
            verdict('(b) Land 진입', False, 'nav_state %s' % show(s['nav']))
            commander('disarm', '-f')
            return summary()

        landed_ts = None
        hit = None
        end = time.time() + WALL_FACTOR * (T_B + 6.0)

        while time.time() < end:
            s = status()

            if landed_ts is None and s['landed'] == 1.0:
                landed_ts = s['landed_ts']

            if s['arming'] == DISARMED:
                hit = s
                break

            time.sleep(0.5)

        z = height()
        dt = (hit['ts'] - t_land) / 1e6 if hit and finite(hit['ts'], t_land) else None
        verdict('(b) 완화 제거: 착륙 보고 뒤 %.0f 초 안에 무장 해제된다' % T_B,
                dt is not None and T_B - 0.5 <= dt <= T_B + 2.5 and hit['reason'] == REASON_LANDING,
                'Land 진입 뒤 %s 초에 arming_state 1, latest_disarming_reason %s (6 = landing), landed 는 %s 초에 True'
                % (show(dt, '%.1f') if dt is not None else '안 됨',
                   show(hit['reason']) if hit else '-',
                   show((landed_ts - t_land) / 1e6, '%+.2f') if finite(landed_ts, t_land) else '?'))
        verdict('(b) 공중에서 무장 해제됐다 (버그 재현)',
                hit is not None and finite(z) and z > MIN_HEIGHT_M,
                'gz z %s m' % show(z, '%.1f'))

    finally:
        # J. Put it back and read it back. A verdict failure here is a real
        # failure: the run is not clean with the parameter left at 3.
        out = param_set('COM_DISARM_LAND', -1)
        restored = param_show('COM_DISARM_LAND')
        verdict('COM_DISARM_LAND 복원 확인', restored == -1.0,
                'param set -> %r, param show %s' % ((out or '').strip(), restored))
        sticks.stop = True
        commander('disarm', '-f')

    return summary()


if __name__ == '__main__':
    sys.exit(main())
