#!/usr/bin/env python3
"""A negative SIM_GZ_EC_MIN does not drive a gz thruster backwards, and this

    GZ_TARGET=gz_airship_vectored_independent ./gz_run.sh ./reverse_thrust_check.py
pins down exactly what it does instead.

PX4 leaves a reversible motor's value in [-1, 1] and interpolates it onto
[SIM_GZ_EC_MIN, MAX]. The metadata bound of 0..1000 is not what stops a
negative minimum: param_info_s carries a name, a type and a value and no
bounds, and the store really does keep the -300. MixingOutput is what stops
it -- uint16_t _min_value[] (mixer_module.hpp:255), assigned from an int32_t
(mixer_module.cpp:175), and then swapped with the maximum because min now
exceeds max (:186).

So the result is arithmetic and predictable. With MIN = -MAX the range becomes
[MAX, 65536 - MAX], and the three commanded points land on MAX, 32768 and
65536 - MAX. Every one of them is a large positive rotor speed, and
MulticopterMotorModel clamps only from above, so a "reversible" motor set up
this way runs full forward at every command including neutral.

Asserting those three numbers is the point of this check. If PX4 ever widens
that storage, it fails, and that is the signal that the Cloudship's
GenericMotorModel duty-cycle workaround can be dropped for a plain signed
range. The route that does work today is in cloudship_sitl.md.

It reads the motor it commands. The reader used to slice the last two entries
off a multi-message dump, which is the first motor only on a two-motor
vehicle; on the Cloudship it returned the port thruster while the check drove
the starboard one, asserted the prediction against a motor it never touched,
and printed the note advising that the workaround could be dropped.
"""
import sys
import time

import gz_link
from report import note, summary, verdict
from sitl_link import actuator_hold, actuator_release, commander, param_set, param_show

MOTOR = 1                                 # which SIM_GZ_EC_* and which Motor
FUNCTION = 100 + MOTOR                    # OutputFunction::Motor1 is 101
INDEX = MOTOR - 1                         # into the bridge's actuator array
SETTLE_S = 4.0


def main():
    if not commander('status'):
        note('px4 에 붙지 못했다')
        return 3

    was_min = param_show('SIM_GZ_EC_MIN%d' % MOTOR)
    was_rev = param_show('CA_R_REV')
    top = param_show('SIM_GZ_EC_MAX%d' % MOTOR)

    if was_min is None or top is None:
        note('  실패  SIM_GZ_EC_MIN/MAX 를 읽지 못했다. px4 가 떠 있는지 본다')
        return 3

    top = int(top)
    param_set('SIM_GZ_EC_MIN%d' % MOTOR, -top)
    param_set('CA_R_REV', 1 << INDEX)
    time.sleep(3)

    try:
        # 1. the store takes it. This is the half that is true.
        after = param_show('SIM_GZ_EC_MIN%d' % MOTOR)
        verdict('파라미터 저장소가 음수를 받는다',
                after is not None and abs(after + top) < 1.0,
                'SIM_GZ_EC_MIN%d %s -> %s' % (MOTOR, was_min, after))

        # 2. and the consumer truncates it. uint16 wrap, then the min > max
        #    swap, so the range the bridge interpolates on is [top, 65536-top].
        want = {-1.0: float(top), 0.0: 32768.0, 1.0: 65536.0 - top}
        reach = {}

        for value in (-1.0, 0.0, 1.0):
            actuator_hold([(FUNCTION, value)], timeout=int(SETTLE_S) + 6)
            time.sleep(SETTLE_S)
            got = gz_link.motor_speed(INDEX)
            reach[value] = got
            actuator_hold([(FUNCTION, value)], timeout=int(SETTLE_S) + 6)

        actuator_release([FUNCTION])
        read_all = all(v is not None for v in reach.values())
        truncated = read_all and all(abs(reach[v] - want[v]) <= 1.0 for v in want)
        show = lambda v: '읽기실패' if reach[v] is None else '%.0f' % reach[v]
        verdict('소비자는 uint16 으로 자른다', truncated,
                '명령 -1/0/+1 -> %s / %s / %s, 잘린 범위 [%d, %d] 예측 %.0f / %.0f / %.0f'
                % (show(-1.0), show(0.0), show(1.0), top, 65536 - top,
                   want[-1.0], want[0.0], want[1.0]))

        if read_all and not truncated:
            note('  참고: 예측이 빗나갔다. 펌웨어가 폭을 넓혔다면 '
                 'Cloudship 의 duty_cycle 우회를 걷어낼 수 있다')

    finally:
        # Put the airframe back: this ran on a live vehicle's parameters, and
        # it runs in a `finally` because an exception between the two is
        # exactly when a vehicle is most likely to be left rewired.
        param_set('SIM_GZ_EC_MIN%d' % MOTOR, int(was_min))

        if was_rev is not None:
            param_set('CA_R_REV', int(was_rev))

        time.sleep(2)
        back = param_show('SIM_GZ_EC_MIN%d' % MOTOR)
        rev = param_show('CA_R_REV')
        verdict('파라미터 복구',
                back is not None and abs(back - was_min) < 1.0
                and (was_rev is None or (rev is not None and abs(rev - was_rev) < 0.5)),
                'SIM_GZ_EC_MIN%d %s, CA_R_REV %s' % (MOTOR, back, rev))

    return summary()


if __name__ == '__main__':
    sys.exit(main())
