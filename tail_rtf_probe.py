#!/usr/bin/env python3
"""Does the simulator crawl, or the tail weaken, under a held tail command
through px4? Real-time factor at rest and under actuator_test, with the
yaw-rate slope on the simulator's clock, printed and nothing judged.

    GZ_TARGET=gz_cloudship_am_fin ./gz_run.sh ./tail_rtf_probe.py
"""
import math
import sys
import time

from report import note
from sitl_link import actuator_hold, actuator_release, commander, field, listener, vector

TAIL = 103


def stamped():
    text = listener('vehicle_angular_velocity')
    t, xyz = field(text, 'timestamp'), vector(text, 'xyz')
    return None if t is None or xyz is None else (time.time(), t / 1e6, math.degrees(xyz[2]))


def window(label, seconds, hold=None):
    rows = []
    end = time.time() + seconds

    while time.time() < end:
        if hold is not None:
            actuator_hold([(TAIL, hold)], timeout=int(seconds) + 3)

        s = stamped()

        if s:
            rows.append(s)

    if len(rows) < 3:
        note('%s: 표본 부족' % label)
        return

    wall = rows[-1][0] - rows[0][0]
    sim = rows[-1][1] - rows[0][1]
    n = len(rows)
    mt = sum(r[1] for r in rows) / n
    mr = sum(r[2] for r in rows) / n
    stt = sum((r[1] - mt) ** 2 for r in rows)
    slope = sum((r[1] - mt) * (r[2] - mr) for r in rows) / stt if stt > 0 else float('nan')
    note('%-14s 표본 %3d, 벽 %.1f s, 시뮬 %.2f s, RTF %.2f, 요율 %+.1f -> %+.1f deg/s, 기울기 %+.1f deg/s^2'
         % (label, n, wall, sim, sim / wall if wall else 0, rows[0][2], rows[-1][2], slope))


def main():
    if not commander('status'):
        note('px4 에 붙지 못했다')
        return 3

    time.sleep(15)
    window('idle', 6.0)
    window('tail +1', 6.0, hold=1.0)
    actuator_release([TAIL])
    window('released', 6.0)
    window('tail -1', 6.0, hold=-1.0)
    actuator_release([TAIL])
    return 0


if __name__ == '__main__':
    sys.exit(main())
