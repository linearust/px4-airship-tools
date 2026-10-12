#!/usr/bin/env python3
"""Why does the tail push forward but not back?

    GZ_TARGET=gz_cloudship ./gz_run.sh ./tail_probe.py

Full forward reaches +50.0 deg/s, which is this hull's terminal rate for the
tail's yaw moment against nRabsR to three figures, so the model, the polynomial
and the duty mapping are all right in that direction. Full reverse moved it half
a degree a second. And the wire said the bridge puts minDutyCycle on the topic
for -1, which is the duty the plugin turns into a reference of exactly -1.

Two readings could not both be right, so this takes them at the same time: the
duty on the bridge topic, the hull's yaw rate, and reversible_flags as
ActuatorTest sees them, once a second through a command in each direction.

It found that the plant was never at fault. The tail reverses to -53.9 deg/s
within two seconds, three times over. What was wrong was the check that had
said otherwise: it commanded once, slept ten seconds and read once, and
actuator_test's override lapses on the simulator's clock while a sleep runs on
this machine's. A tail that pushed and was then let go reads exactly like one
that never pushed. This file is kept because that is a question worth being
able to ask again, and because sampling continuously is the shape of the
answer.
"""
import sys
import time

import gz_link
from report import note
from sitl_link import (actuator_hold, actuator_release, angular_rate, commander,
                       field, listener, param_show, settle_rate)

TAIL_FUNCTION = 103                       # SIM_GZ_EC_FUNC3
TAIL_INDEX = TAIL_FUNCTION - 101          # into the bridge's actuator array
LEG_S = 14.0


def yaw():
    rate = angular_rate()
    return None if rate is None else rate[2]


def flags():
    """reversible_flags as actuator_test sees them.

    ActuatorTest re-derives a motor's reversibility from the last
    actuator_motors message rather than from CA_R_REV, and treats a missing
    flag as "not reversible", which turns a negative test value into NAN and a
    NAN into the disarmed value. That is the one way a command the bridge maps
    correctly can still arrive as neutral, so it is sampled next to the duty.
    """
    return field(listener('actuator_motors'), 'reversible_flags')


def leg(value, seconds=LEG_S):
    note('--- 명령 %+.1f' % value)
    start = time.time()

    while time.time() - start < seconds:
        # Re-issued every pass, not commanded once: see the docstring.
        actuator_hold([(TAIL_FUNCTION, value)], timeout=int(seconds) + 6)
        d, y, f = gz_link.motor_speed(TAIL_INDEX), yaw(), flags()
        note('   t %4.1f s  듀티 %s  요 %s deg/s  reversible_flags %s'
             % (time.time() - start,
                '%8.1f' % d if d is not None else '   읽기실패',
                '%+7.2f' % y if y is not None else '  읽기실패',
                '%d' % f if f is not None else '?'))

    actuator_release([TAIL_FUNCTION])


def main():
    if not commander('status'):
        note('px4 에 붙지 못했다')
        return 3

    note('무장 상태: 해제 (actuator_test 는 해제 상태에서만 출력에 닿는다)')

    for name in ('CA_R_REV', 'SIM_GZ_EC_MIN3', 'SIM_GZ_EC_MAX3',
                 'SIM_GZ_EC_DIS3', 'THR_MDL_FAC'):
        note('  %-16s %s' % (name, param_show(name)))

    for value in (-1.0, 1.0, -1.0):
        note('정착 %s' % settle_rate(limit=4.0))
        leg(value)

    return 0


if __name__ == '__main__':
    sys.exit(main())
