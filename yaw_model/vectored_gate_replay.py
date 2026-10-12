#!/usr/bin/env python3
"""Closed-loop check of vectored_actuation.py on the RIGID vectored plant.

Replays the gate's sequence (bench sitl_check.py:318-426) with the PX4 yaw
rate loop around vectored_actuation.VectoredActuation and the undamped rigid
hull, Izz dr/dt = N. Only there to test the actuation model against what the
rigid gate recorded; the loop and plant models proper are other parts' work.

Loop (src/lib/rate_control/rate_control.cpp:71-118, airship_att_control_main.cpp:152-161, 171-194, 236-238):
  u = P e + I,  e = sp - r;  I += i_factor Ki e dt (e clipped by the saturation
  flags), |I| <= AS_YR_INT_LIM; i_factor = max(0, 1 - (e / 400 deg/s)^2)
  sp = deadzone(stick, MAN_DEADZONE) * AS_YAWRATE_MAX   (airship_manual_input.hpp:89-92)
  I is reset when the loop opens (Manual): airship_att_control_main.cpp:246-250
Gains from the task context: P 0.5, I 0.09, INT_LIM 0.3, MAX 15 deg/s, DZ 0.1.

Assumptions: gyro = true rate (no IMU noise/filter), 4 ms loop, rigid yaw
inertia 129.2 kg m^2 (the gate's own print, chain7_20260925.txt:66), no yaw
damping (same line: "요 감쇠 0"), no roll/pitch coupling into yaw.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vectored_actuation import VectoredActuation, load_params  # noqa: E402

P, KI, LIM, RMAX, DZ = 0.5, 0.09, 0.3, math.radians(15.0), 0.1
IZZ = 129.2


def deadzone(x, dz):
    # src/lib/mathlib/math/Functions.hpp deadzone: (x - dz sign x) / (1 - dz) outside the zone
    if abs(x) <= dz:
        return 0.0
    return (x - math.copysign(dz, x)) / (1.0 - dz)


class Sim:
    def __init__(self, p):
        self.act = VectoredActuation(p)
        self.dt = p['dt']
        self.r = 0.0
        self.I = 0.0
        self.sat = 0.0          # yaw saturation sign the controller reads
        self.t = 0.0
        self.log = []

    def tick(self, mode, stick, wind=0.0):
        if mode == 'manual':
            u = stick
            self.I = 0.0         # loop open: integrator cleared
        else:
            sp = deadzone(stick, DZ) * RMAX
            e = sp - self.r
            u = P * e + self.I
            ei = min(e, 0.0) if self.sat > 0 else max(e, 0.0) if self.sat < 0 else e
            f = max(0.0, 1.0 - (ei / math.radians(400.0)) ** 2)
            self.I = max(-LIM, min(LIM, self.I + f * KI * ei * self.dt))
        row = self.act.step(u)
        self.sat = row['sat_seen'] if mode != 'manual' else 0.0
        self.r += (row['N'] + wind) / IZZ * self.dt
        self.t += self.dt
        self.log.append((self.t, mode, stick, u, self.r, self.I, row['N'], row['tilt_gz_s'], row['tilt_gz_p']))

    def hold(self, mode, stick, seconds, wind=0.0):
        start = len(self.log)
        for _ in range(int(round(seconds / self.dt))):
            self.tick(mode, stick, wind)
        return self.log[start:]


def deg(x):
    return math.degrees(x)


def settled(rows, threshold=1.0, hold=1.5):
    start = None
    for row in rows:
        if abs(deg(row[4])) < threshold:
            start = row[0] if start is None else start
            if row[0] - start >= hold:
                return start - rows[0][0]
        else:
            start = None
    return None


def main():
    p = load_params()
    sim = Sim(p)
    sim.hold('manual', 0.0, 1.0)
    # the wind check left the hull at the recorded -2.7 deg/s (chain7_20260925.txt:67 first sample)
    sim.r = math.radians(-2.7)
    manual = sim.hold('manual', 0.5, 3.0)
    print('Manual u=0.5 for 3.0 s: %.1f -> %.1f deg/s   (recorded -2.7 -> 23.5 over a nominal 3 s window, :67)'
          % (-2.7, deg(manual[-1][4])))
    # stick back to 0 in Manual: the rate holds (no damping); Acro starts at the recorded 27.8
    sim.hold('manual', 0.0, 0.5)
    sim.r = math.radians(27.8)
    brake = sim.hold('acro', 0.0, 15.0)
    marks = [0.5, 1.0, 1.5, 2.0, 3, 4, 5, 6, 8, 10, 12, 14, 15]
    print('Acro brake from 27.8 deg/s (recorded 27.8 -> -7.7 after the 15 s window, :68):')
    print('   ' + '  '.join('t%-4g %+6.1f' % (m, deg(brake[int(m / sim.dt) - 1][4])) for m in marks))
    print('   u at 0.004 s %+.3f, pods swing until ~1.5 s; I at 15 s %+.3f' % (brake[0][3], brake[-1][5]))

    for label, mode in (('Acro', 'acro'), ('Stabilized', 'stabilized')):
        # clean start: Acro to rest, Manual 1 s (clears I), mode 1 s
        rest = None
        for _ in range(int(60 / 0.3)):
            rows = sim.hold('acro', 0.0, 0.3)
            if abs(deg(rows[-1][4])) < 0.5:
                rest = (rest or 0.0) + 0.3
                if rest >= 1.0:
                    break
            else:
                rest = None
        sim.hold('manual', 0.0, 1.0)
        start = sim.hold(mode, 0.0, 1.0)
        push = sim.hold(mode, 0.5, 35.0)
        release = sim.hold(mode, 0.0, 12.0)
        last3 = [deg(r[4]) for r in push if r[0] >= push[-1][0] - 3.0]
        peak = max(deg(r[4]) for r in push)
        t_peak = max(push, key=lambda r: r[4])[0] - push[0][0]
        st = settled(release)
        print('%s: start %.2f deg/s, pods %.0f/%.0f deg; push mean last 3 s %.2f, peak %.2f at %.1f s; '
              'release %.1f -> %.1f, settled %s'
              % (label, deg(start[-1][4]), start[-1][7], start[-1][8], sum(last3) / len(last3), peak, t_peak,
                 deg(release[0][4]), deg(release[-1][4]), 'in %.1f s' % st if st is not None else 'no'))
    print('recorded (chain7_20260925.txt:69-72): Acro mean 6.00 peak 8.84, release 5.8 -> -2.0 in 4.7 s; '
          'Stabilized mean 6.10 peak 8.78, release 6.3 -> -1.7 in 5.1 s')


if __name__ == '__main__':
    main()
