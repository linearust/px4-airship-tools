#!/usr/bin/env python3
"""The Cloudship's flight airframe (2507) on a real FMU, read at the pins.

    python3 cloudship_ground_check.py            # read-only: parameters, pin map, allocator
    python3 cloudship_ground_check.py --drive    # also drives Servo1 and Motor3 (PROPS OFF)

Nothing is armed at any point. actuator_test reaches the outputs only while
disarmed (mixer_module.cpp:481-483), which is also why a check like this can
run on the vehicle with the ESCs unpowered: the pulse is the claim, and the
pulse is what the probe reads.

What it asserts, and what the 2026-09-23 hand run on the bench FMU
(81f72e4920, airframe switched 2510 -> 2507) showed:

  1. The airframe's parameters read back as 2507_cloudship sets them. On the
     bench two saved values survived the switch, CA_SV_CS_COUNT 4 and the
     2510 pin map; an airframe file's set-default never beats a saved value,
     so a vehicle configured after another one carries that one's leftovers
     until SYS_AUTOCONFIG resets everything. This check refuses on them.
  2. Each of the four functions (Motor1-3, Servo1) is on exactly one probed
     pin, read from PWM_MAIN_FUNC*/PWM_AUX_FUNC*, and the tail's disarmed
     pulse is the centre of its range: for a reversible ESC "disarmed" at MIN
     would be full reverse, which is why the SITL twin sets DIS3 to neutral.
  3. The allocator configured exactly four actuators with Motor3 reversible.
  4. (--drive) Servo1 at -1/+1/0 puts MIN/MAX/centre on its pin for the whole
     hold (1000/2000/1500 us for 8.0 s each on the bench), Motor3 the same
     (1000/2000/1500: a tail the mixer did not see as reversible would put
     its disarmed value there for -1), and every other probed pin holds its
     disarmed value for the whole capture.

The probe map is the bench's (la_mcp.py); on the vehicle pass its own as
GROUND_PROBES='{"AUX1": 0, ...}' (pin name -> Logic channel).
"""
import json
import os
import re
import sys
import time

import fmu_link
import la_mcp
from ground_drive import GAP_S, HOLD_S, LEAD_S, echoed, plateaus, shell
from report import note, summary, verdict

AIRFRAME = {                                  # 2507_cloudship + rc.airship_defaults
    'SYS_AUTOSTART': 2507, 'CA_AIRFRAME': 16, 'CA_AIRSHIP_GRP': 0, 'CA_AIRSHIP_TAIL': 1,
    'CA_AIRSHIP_TLMIN': -90.0, 'CA_AIRSHIP_TLMAX': 90.0, 'CA_AIRSHIP_TLT_R': 120.0,
    'CA_R_REV': 4, 'CA_SV_CS_COUNT': 0, 'COM_PREARM_MODE': 2, 'COM_DISARM_LAND': -1.0,
}
FUNCTIONS = {101: 'Motor1', 102: 'Motor2', 103: 'Motor3 (tail)', 201: 'Servo1 (tilt)'}
DRIVEN = ((201, 'tilt'), (103, 'tail'))
VALUES = (-1.0, 1.0, 0.0)
PROBES = json.loads(os.environ.get(
    'GROUND_PROBES',
    '{"MAIN1": 0, "MAIN2": 1, "MAIN3": 2, "MAIN4": 3, "MAIN5": 4, "MAIN6": 5, "AUX1": 6, "AUX2": 7}'))
PINS = ['MAIN%d' % i for i in range(1, 9)] + ['AUX%d' % i for i in range(1, 9)]
TOL_US = 5.0
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ground_out')


def read(link, name, tries=3):
    """One parameter by name, or None. INT32 comes back as int."""
    m = fmu_link.read_param(link, name, tries)
    return None if m is None else fmu_link.decode(m)[0]


def pwm(pin, field):
    group, index = pin[:-1], pin[-1]
    return 'PWM_%s_%s%s' % (group, field, index)


def channel_range(link, pin):
    rev_mask = read(link, 'PWM_%s_REV' % pin[:-1])
    rng = {f: read(link, pwm(pin, f)) for f in ('MIN', 'MAX', 'DIS', 'CENT')}
    rng['rev'] = rev_mask is not None and bool(int(rev_mask) >> (int(pin[-1]) - 1) & 1)
    return rng


def expected(value, rng):
    """mixer_module.cpp:547-566: REV negates, CENT unset -> linear MIN..MAX."""
    if rng['rev']:
        value = -value

    return rng['MIN'] + (value + 1.0) / 2.0 * (rng['MAX'] - rng['MIN'])


def held(segs, want):
    """The plateaus that satisfy `want` in order, or fewer if one is missing.

    Each value's pulse within TOL_US and, except for the last one, held for
    the full HOLD_S: order matters because the centre is the idle value at the
    pin, so it is credited only once it follows MAX. A tail the mixer did not
    see as reversible puts its disarmed value there for -1, and that trace has
    no MIN plateau at all; a hold cut by a second command is shorter than
    HOLD_S. Both read as fewer plateaus than wanted.
    """
    found, cursor = [], 0

    for k, w in enumerate(want):
        last = k == len(want) - 1
        hit = next(((i, s) for i, s in enumerate(segs[cursor:], cursor)
                    if abs(s[2] - w) <= TOL_US and (last or s[1] >= HOLD_S - 0.5)), None)

        if hit is None:
            break

        found.append(hit[1])
        cursor = hit[0] + 1

    return found


def flat(segs, dis, seconds):
    """One plateau at the disarmed value for (nearly) the whole capture."""
    return len(segs) == 1 and abs(segs[0][2] - dis) <= TOL_US and segs[0][1] >= seconds - 2.0


def drive(function, name, expect, others):
    seconds = LEAD_S + len(VALUES) * (HOLD_S + GAP_S) + 10.0
    cid, t0 = la_mcp.start(seconds)
    note('  캡처 %s (%.0f s): 기능 %d 을 %s 로 %g s 씩' % (cid, seconds, function, VALUES, HOLD_S))
    time.sleep(LEAD_S)

    for value in VALUES:
        cmd = 'actuator_test set -f %d -v %g -t %d' % (function, value, int(HOLD_S))
        said = shell(cmd)

        if not echoed(said, cmd):
            note('  셸이 명령을 되돌려주지 않았다 (값 %g%s)' % (value, ', 시간 초과' if said is None else ''))

        time.sleep(HOLD_S + GAP_S)

    la_mcp.finish(cid, os.path.join(OUT, name + '.sal'), os.path.join(OUT, name + '_export'))
    window = 0.1
    rows = la_mcp.widths(os.path.join(OUT, name + '_export'), window=window)
    segs = {ch: plateaus(rows.get(ch, []), window) for ch in range(8)}

    for ch in range(8):
        note('    D%d %s' % (ch, '  '.join('%5.1f+%4.1f:%6.1f' % s for s in segs[ch]) or 'no pulses'))

    ch = expect['channel']
    want = [expected(v, expect['range']) for v in VALUES]
    found = held(segs[ch], want)
    verdict('%s: -1 / +1 / 0 이 핀에서 %.0f / %.0f / %.0f us' % (name, want[0], want[1], want[2]),
            len(found) == len(want),
            'D%d 에서 %s' % (ch, ', '.join('%.0f us %.1f s' % (s[2], s[1]) for s in found) or '없음'))

    # Every other probed pin: one plateau at its disarmed value, capture-long.
    # During a test session all non-tested outputs are NaN -> disarmed value
    # (actuator_test.cpp:125-141, mixer_module.cpp:543-544).
    bad = ['D%d %s' % (other_ch, ' '.join('%.0f/%.1fs' % (p[2], p[1]) for p in segs[other_ch]) or 'no pulses')
           for other_ch, dis in others.items()
           if other_ch != ch and not flat(segs[other_ch], dis, seconds)]

    verdict('%s 구동 중 다른 핀은 캡처 내내 자기 해제값' % name, not bad,
            ', '.join(bad) if bad else '%d 핀 평탄' % (len(others) - 1))


def main():
    driving = '--drive' in sys.argv[1:]
    link = fmu_link.connect()

    if link is None:
        note('FMU 가 응답하지 않는다')
        return 3

    # 1. The airframe, as read back.
    got = {k: read(link, k) for k in AIRFRAME}
    wrong = ['%s %s (2507: %s)' % (k, got[k], v) for k, v in AIRFRAME.items()
             if got[k] is None or abs(float(got[k]) - v) > 1e-6]
    verdict('2507 에어프레임 파라미터가 읽힌 대로다', not wrong,
            ', '.join(wrong) if wrong else '%d 개 일치' % len(AIRFRAME))
    thr = read(link, 'THR_MDL_FAC')
    note('  THR_MDL_FAC %s (비행 파일은 0: 측정된 추력 곡선으로 채워야 요 게인이 유효하다)' % thr)

    if wrong:
        link.close()
        return summary()

    # 2. Pin map, then the ranges of every probed pin.
    on = {f: [] for f in FUNCTIONS}
    func_of = {}

    for pin in PINS:
        f = read(link, pwm(pin, 'FUNC'))
        func_of[pin] = None if f is None else int(f)

        if func_of[pin] in on:
            on[func_of[pin]].append(pin)

    problems = []

    for f, pins in on.items():
        if len(pins) != 1:
            problems.append('%s(%d) 은 %s' % (FUNCTIONS[f], f, '핀 %d 개' % len(pins) if pins else '어느 핀에도 없다'))
        elif pins[0] not in PROBES:
            problems.append('%s(%d) 은 탐침 없는 %s' % (FUNCTIONS[f], f, pins[0]))

    verdict('네 기능이 각각 탐침된 핀 하나에 있다', not problems,
            ', '.join(problems) if problems else
            ', '.join('%s=%s(D%d)' % (FUNCTIONS[f], on[f][0], PROBES[on[f][0]]) for f in sorted(on)))

    if problems:
        link.close()
        return summary()

    ranges = {pin: channel_range(link, pin) for pin in PROBES}
    link.close()

    for pin in PROBES:
        r = ranges[pin]
        note('  %-5s func %-3s MIN %s MAX %s DIS %s CENT %s REV %s'
             % (pin, func_of[pin], r['MIN'], r['MAX'], r['DIS'], r['CENT'], int(r['rev'])))

    tail_pin = on[103][0]
    r = ranges[tail_pin]
    mid = (r['MIN'] + r['MAX']) / 2.0
    verdict('꼬리의 해제 펄스가 범위의 중앙이다 (가역 ESC 의 정지)',
            None not in (r['MIN'], r['MAX'], r['DIS']) and abs(r['DIS'] - mid) <= 1.0,
            '%s DIS %s, 중앙 %.0f' % (tail_pin, r['DIS'], mid))

    # 3. The allocator's view.
    out = shell('control_allocator status', 'listener actuator_motors') or ''
    m = re.search(r'Configured actuators:\s*(\d+)', out)
    flags = re.search(r'reversible_flags:\s*(\d+)', out)
    age = re.search(r'timestamp: \d+ \(([\d.]+) seconds ago\)', out)
    verdict('할당기가 액추에이터 4 개, Motor3 가역으로 구성됐다',
            m is not None and int(m.group(1)) == 4 and flags is not None and int(flags.group(1)) == 4
            and age is not None and float(age.group(1)) < 0.5,
            'Configured actuators %s, reversible_flags %s, actuator_motors %s s 전'
            % (m.group(1) if m else '?', flags.group(1) if flags else '?', age.group(1) if age else '?'))

    if not driving:
        note('--drive 없이 끝: 출력은 움직이지 않았다')
        return summary()

    # 4. Drive, one function per capture. Props off is the caller's word.
    os.makedirs(OUT, exist_ok=True)
    others = {PROBES[pin]: ranges[pin]['DIS'] for pin in PROBES if func_of[pin]}

    for function, name in DRIVEN:
        pin = on[function][0]
        drive(function, name, {'channel': PROBES[pin], 'range': ranges[pin]}, others)

    return summary()


if __name__ == '__main__':
    sys.exit(main())
