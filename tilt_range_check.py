#!/usr/bin/env python3
"""Check that the gz tilt pods reach the angles the airframe says they can.

    GZ_TARGET=gz_airship_vectored_independent ./gz_run.sh ./tilt_range_check.py

The model stops the tilt joints at +-pi. Nothing should ever meet that stop:
the bridge maps the servo output onto SIM_GZ_SV_MINA/MAXA and the parameter
metadata bounds both to +-180 deg, so a full-scale command lands 2e-5 deg
inside it. That is an argument, not a measurement, and the two numbers live in
different repositories, so this measures it.

It drives each tilt servo to both ends with `actuator_test` (disarmed) and
reads the pod link's own orientation back out of gz. A pod that stops short of
its commanded angle is the failure this exists to find; so is one that keeps
going, which would mean the stop is not there at all.

The centre case used to prove nothing. A tilt joint's uncommanded rest state
IS the centre - SIM_GZ_SV_DIS defaults to 500 of 0..1000, which the servo
bridge maps to exactly (MINA+MAXA)/2, and JointPositionController holds 0 rad
with no command at all - so a pod already sitting at 0 satisfied the 0-degree
case before it was asked to move, and did so even with the servo function
removed from the airframe entirely. The run no longer parks the servo at
centre between cases and runs the centre case last, so it starts from the +1
end stop: it can only pass without moving in a run whose +1 case has already
failed. An arrival at a pose the pod already held is flagged in the detail, not
refused - on a +-180 range the +1 case starts at -180, the same orientation.
"""
import math
import sys
import time

import gz_link
from report import note, summary, verdict
from sitl_link import actuator_hold, actuator_release, commander, param_show

TOLERANCE_DEG = 1.0
DEADLINE_S = 25.0

PODS = (('pod_starboard', 201, '우현'), ('pod_port', 202, '좌현'))


def pod_angle_deg(link):
    """Tilt of one pod, from its orientation in the model frame.

    The joint axis is (0, -1, 0) and the pod's own pose carries no rotation,
    so the link's relative orientation is the joint angle and nothing else.

    Averaged as a direction rather than as a number, because the angle this
    test cares about most is +-180, where samples land on both sides of the
    cut and any average of the numbers reads back as zero.
    """
    rows = gz_link.poses(link)

    if not rows:
        return None

    sin_sum = cos_sum = 0.0

    for _, _, (w, x, y, z) in rows:
        angle = -2.0 * math.atan2(y, w)
        sin_sum += math.sin(angle)
        cos_sum += math.cos(angle)

    if math.hypot(sin_sum, cos_sum) < 0.5 * len(rows):
        return None    # still swinging through half a turn

    return math.degrees(math.atan2(sin_sum, cos_sum))


def near(angle, want):
    """Distance on the circle. +-180 is one orientation reached from either side."""
    return abs((angle - want + 180.0) % 360.0 - 180.0)


def drive_to(link, function, value, want_deg):
    """Hold one servo at `value` until the pod is at `want_deg`, or give up.

    Polled rather than slept on, and polled against the target rather than
    against the previous reading. "Two readings the same" is also what a pod
    that has not started moving looks like, and the first command of a run
    does take a moment to land: an earlier version of this returned the
    untouched zero within two seconds and reported it as settled, which reads
    exactly like a servo that cannot reach its commanded angle.

    Returns (angle, owed). `owed` is False when the pod was already at the
    target before the command, which is what an uncommanded joint sitting at
    its rest position looks like - and why the centre case passes its own line
    with the servo unmapped. It does NOT park the servo afterwards: the caller
    orders the cases so each one is reached from somewhere else, and parking
    at centre is what made the centre case unreachable-from-elsewhere.
    """
    # Where the pod was BEFORE the command. Arriving at a target it was not
    # already at is the proof that it moved - watching it move on the way is
    # not, because a fast servo gets there inside one `gz topic` read and the
    # in-loop sampling then reports a pod that never moved.
    #
    # The airship's two end stops are +-180, which is ONE orientation reached
    # from either side, so a pod at -180 is already at +180 and no motion is
    # owed there; the orientation this reads cannot tell the two joint angles
    # apart. That is reported rather than credited silently. The case it was
    # written for - a centre target the joint already rests at, with the servo
    # function removed from the airframe - still passes its own line, but the
    # run fails: a pod that cannot leave the centre fails both end cases, and
    # one that reaches them starts the centre case, which the caller orders
    # last, from +180.
    start = pod_angle_deg(link)
    owed = start is None or near(start, want_deg) > TOLERANCE_DEG

    deadline = time.time() + DEADLINE_S
    angle = None
    arrived = 0

    while time.time() < deadline:
        # Re-issued each pass: actuator_test only reaches the outputs while
        # disarmed and drops the override when its own timeout expires.
        actuator_hold([(function, value)], timeout=int(DEADLINE_S))
        angle = pod_angle_deg(link)

        if angle is None:
            continue

        if near(angle, want_deg) <= TOLERANCE_DEG:
            arrived += 1

            if arrived >= 2:
                break
        else:
            arrived = 0

    return angle, owed


def servo_limits(index):
    low = param_show('SIM_GZ_SV_MINA%d' % index)
    high = param_show('SIM_GZ_SV_MAXA%d' % index)
    return None if low is None or high is None else (min(low, high), max(low, high))


def main():
    if not commander('status'):
        note('px4 에 붙지 못했다')
        return 3

    for link, function, label in PODS:
        index = function - 200
        limits = servo_limits(index)

        if limits is None:
            verdict('%s 서보 범위를 읽는다' % label, False,
                    'SIM_GZ_SV_MINA%d/MAXA%d 를 읽지 못했다' % (index, index))
            continue

        low, high = limits
        verdict('%s 서보 범위가 조인트 스톱 안에 있다' % label,
                low >= -180.0 and high <= 180.0,
                'SIM_GZ_SV %g .. %g deg, 스톱 +-180' % (low, high))

        # Centre last, so it is reached from an end stop rather than from the
        # rest position it would sit at anyway.
        for value, want in ((-1.0, low), (1.0, high), (0.0, (low + high) / 2)):
            got, owed = drive_to(link, function, value, want)

            if got is None:
                verdict('%s 명령 %+.0f' % (label, value), False,
                        '%.0f 초 동안 gz 에서 포드 자세를 읽지 못했다' % DEADLINE_S)
                continue

            err = near(got, want)
            verdict('%s 명령 %+.0f -> %+.1f deg' % (label, value, want),
                    err <= TOLERANCE_DEG,
                    '실제 %+.1f deg, 오차 %.2f deg%s'
                    % (got, err, '' if owed else ' (명령 전부터 그 자세였다)'))

        actuator_release([function])

    return summary()


if __name__ == '__main__':
    sys.exit(main())
