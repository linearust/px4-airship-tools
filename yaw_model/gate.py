#!/usr/bin/env python3
"""The SITL yaw gate, sitl_check.py, run on a model instead of the simulator.

    from gate import run_gate, compose
    result = run_gate(loop, 'cloudship_am_fin_drag')
    print('\\n'.join(result.gate_lines))

run_gate() executes sitl_check.py's own main() - the file on disk, loaded
fresh on every call, not a transcription - with its I/O bound to a virtual
clock that drives `loop`. The sequence, every threshold and every verdict
string are therefore sitl_check's by construction. What this file adds is
(a) the timing of the harness around main(), measured from the recorded run,
and (b) the mapping from each I/O call to an event on the model. The I/O that
main() does, all of it replaced here:

    time.time, time.sleep          -> the virtual wall clock (sim = RTF x wall)
    sitl_link.listener(topic)      -> a px4-listener text rendered from `loop`
    sitl_link.commander(...)       -> loop.set_mode / arm / disarm, delayed
    sitl_link.param_set/param_show -> loop.set_param / loop.param
    sitl_link.Sticks().yaw = v     -> loop.set_stick(v), delayed (20 Hz MAVLink)
    sitl_link.wait_for_ekf()       -> a fixed wait, True
    gz_link.wrench / wrench_clear  -> loop.set_wind(-torque) after the publish
    report.verdict / note          -> the real report.py, loaded fresh

field(), vector() and finite() are sitl_link.py's own, lifted out of its
source with ast, so the listener text is parsed by the gate's parser.
gz_model is the real module (pure: model.sdf files only).


THE GATE AS A SCRIPT (sitl_check.py line numbers; W = wall seconds)
-------------------------------------------------------------------
Pre-arm, not judged: param_show AS_YAWRATE_MAX, MAN_DEADZONE, AS_YAWRATE_I
(:231-233); setpoint = (0.5 - dz)/(1 - dz) x max (:240) = 6.667 deg/s at
15 / 0.1; THR_MDL_FAC read, a warning below 0.5 (:245-248); wait_for_ekf (:252); sleep 10 (:255);
param_set MAN_ARM_GESTURE 0 (:256); sticks start, sleep 2 (:258-260); mode
manual, sleep 1, arm, sleep 3 (:261-264); abort 2 unless armed (:266).

 1 settle   wrench_clear (:275); sample 3 W (:277); abort 2 unless every rate
            is finite and mean|r| of the last 3 W <= 2.0 deg/s (:283).
 2 wind     before = last row of sample 2 W (:292); wrench +1.0 N m on world
   check    z (:293); sample 5 W (:294); wrench_clear (:295). Expected =
            Euler integral, 5.0 s at dt 0.001, of (1 - |nR| r - |nRabsR| r|r|)
            / (Izz_rigid + added rr) from gz_model (:303-312). PASS iff
            both rates are finite and |r_last - before| > 0.5 x expected
            (:313-316). The wrench has acted ~15 P = 4.74 W by the last row,
            not the 5.0 s the expectation integrates.
 3 Manual   stick 0.5 (:319); sample 3 W (:320); stick 0 (:321). PASS iff
            every torque in the second half of the rows is within 0.02 of
            0.5 (:323-326). Abort 2 unless |last torque| >= 0.02 (:333).
 4 brake    mode acro (:350); sample 15 W (:351). Loop present iff any
            |torque| > 0.02 in rows[1:] (:205-219); an unread torque there
            exits 2 (:358-362), no loop exits 3 (:364-369). PASS iff |r| of
            the LAST row (one instant, 47 P = 14.85 W in) < 6.0 deg/s (:371).
 5 step x2  for acro, then stabilized (:375):
            clean_start (:108-136): mode acro; 1-row samples until |r| < 0.5
            has held >= 1.0 W (clock started when the first quiet row's
            sample() returned, so 5 consecutive quiet rows at P = 0.316),
            bounded 60 W; mode manual (the loop opens and the integrator is
            cleared); sleep 1; mode <mode>; sleep 1; one row '출발'.
            stick 0.5 (:390); push = sample 35 W (:395); stick 0 (:396);
            release = sample 12 W (:397).
            Tracking PASS iff the start settled and its rate and integrator
            were read (:382), every push rate is finite, |signed mean of the
            rows with t >= t_last - 3.0 - sp| < 0.25 sp and max(rate) < 1.4 sp
            (:398-404).
            Release PASS iff |release[0].rate| > 0.5 sp (:413) and settled()
            finds |r| < 1.0 held 1.5 W (:164-175; 6 consecutive rows at
            P = 0.316) starting < 10.0 W after the first release row
            (:424-426); the 38-row window ends 11.69 W in, so a start later
            than ~10.2 W cannot be seen at all. The text reports the least-squares
            slope of |r| over the last 3 W when < -0.02 deg/s^2 (:178-202,
            :419-420); the slope does not change the verdict.
 6 wind     mode acro; sleep 2 (:429-430). wind_case (:432-446): wrench(tau);
   cases    sample N W; wrench_clear; needed = tau / authority (= 1/54 on
            both vehicles); integ = mean|integ|, rate = mean|r| of the rows
            with t >= t_last - 2.0; hold PASS iff all rates and integs finite,
            rate < 1.0 and |integ - needed| < 0.5 needed; no-hold PASS iff
            rate > 1.0; then sample 10 W '회복', unjudged.
            a) strong = authority/54 N m (1 on the vectored, 0.082963 on
               the Cloudship), I on, hold, 40 W (:448-450).
            b) param_set AS_YAWRATE_I 0; mode manual; sleep 2; mode acro;
               sleep 1; strong, no hold, 25 W; param_set AS_YAWRATE_I back;
               sleep 1 (:456-468).
            c) vectored only (HAS_TILT_BAND): weak = 0.4 x strong, hold,
               50 W (:470-471).
 7 end      disarm; '=== 정리 ===' list and '통과 N / 실패 M' (:473-481).

Sampling (:89-105): a row = three px4-listener calls in the order
vehicle_angular_velocity z (rad/s -> deg/s), vehicle_torque_setpoint z,
rate_ctrl_status.yawspeed_integ; then 't' = round(wall - T0, 2); then
sleep 0.3. Rows are taken while wall < start + seconds, so a window of S
wall seconds holds ceil(S / P) rows and returns at start + n P.
Measured: P = 0.31605 s on chain7 (689 intervals); P varies run to run
and phase to phase (chain9 gz_cloudship: 0.3153-0.3177 by phase). Row
counts at P = 0.316: 3 W 10, 2 W 7, 5 W 16, 15 W 48, 35 W 111, 12 W 38,
40 W 127, 25 W 80, 50 W 159, 10 W 32; a 3 W tail mean averages 10 rows, a
2 W one 7. Only the 12 W release window is on a knife edge: it holds 39
rows when P < 12/38 = 0.31579 s (chain9 gz_cloudship: 39; chain7: 38).

WHERE WALL AND SIM TIME PART
----------------------------
Every window, sleep, hold and slope in the gate is on the wall clock
(time.time/time.sleep). PX4 runs on gz's clock (GZBridge.cpp:342-359 sets
CLOCK_MONOTONIC from /world/<w>/clock on every message) and gz steps 4 ms
toward RTF 1.0 (worlds/lta.sdf:8-10). At RTF < 1 a window of S wall
seconds is RTF x S of plant and firmware, so the 35 W push, the 1.5 W settle
hold, the 10 W settle limit, the 3 W / 2 W averaging windows and the 1.0 W
rest hold all shrink in plant time, and still_falling's slope is RTF x the
plant's slope. The listener's latency, the 0.3 s sleep, the stick stream
(sitl_link.py:299, 20 Hz on the wall clock) and the ~0.86 s gz topic publish
are wall time too. Evidence that RTF was 1 on the recorded runs:
tail_rtf_probe 2026-09-23, RTF 1.00 in all eight windows on cloudship_am and
cloudship_am_fin (sitl_out/rtf_probe_summary.txt:2-10). On an undamped plant
the gate's own wind check is an RTF meter: the rate grows at RTF x 1 N m /
Izz per wall second. chain9 gz_cloudship (2026-09-27): slope -9.905 deg/s^2
over 15 rows against 9.881 (Izz 5.7986, gz_model) -> RTF 1.002 +- 0.003;
chain9 gz_airship_vectored_independent: 0.980 +- 0.010 (a 2 deg/s rise,
so the 0.01 s rounding of 't' weighs more) (validate_gate.py
rtf_from_wind_check). chain7 endpoints (P of those runs
unknown, 0.3155-0.3165): Cloudship 0.10 -> -46.58 deg/s -> 0.995-0.998;
vectored -0.00 -> -2.09 at 0.4436 deg/s^2 -> 0.990-0.997. A transient crawl
did happen once on a first-light run ("the simulator advancing 10 ms per
round trip", README.md:207-213); the gate records no sim timestamp, so a
crawl inside a gate run would not show except through such consistency
checks. Timing.rtf takes a constant or a function of wall time.
"""
import ast
import contextlib
import heapq
import importlib.util
import io
import math
import os
import re
import sys
import types
from dataclasses import dataclass, field as dc_field
from typing import Callable, Optional, Union

BENCH = os.environ.get('AIRSHIP_BENCH', os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SITL_CHECK = os.path.join(BENCH, 'sitl_check.py')
SITL_LINK = os.path.join(BENCH, 'sitl_link.py')
REPORT = os.path.join(BENCH, 'report.py')
HERE = os.path.dirname(os.path.abspath(__file__))

# The bench tools directory is read-only to this harness: loading its
# modules must not write a __pycache__ entry there.
sys.dont_write_bytecode = True

if BENCH not in sys.path:
    sys.path.insert(0, BENCH)

import gz_model  # noqa: E402  (pure: reads model.sdf, no simulator)

# gz_link.wrench() puts `torque: {z: tau}` on the MODEL entity
# (gz_link.py:203-205), a world-frame couple about +z up. PX4's body z is
# down (FRD). The recorded wind checks show a +1 N m wrench driving the yaw
# rate negative: chain7_20260925.txt, "-0.00 -> -2.09 deg/s" (vectored) and
# "0.10 -> -46.58" (Cloudship); samples.csv rows t 31.21-35.95 (0.05 ->
# -19.87). So on the level hull the FRD yaw moment is -tau.
GZ_WORLD_Z_TO_FRD = -1.0


@dataclass
class Timing:
    """The harness's own time costs, wall seconds, and the RTF.

    Measured on the chain7 record ("samples.csv" below: the last gate of
    chain7_20260925, gz_cloudship_am_fin_drag, 706 rows; sitl_out/samples.csv
    as of 2026-09-25 23:54, overwritten by chain9 on 2026-09-27 17:36 and
    kept as gate_records/samples_gz_cloudship_am_fin_drag_chain7.csv) unless
    marked cosmetic.
    """
    # listener_s, client_s and publish_s are a grid fit of the whole 't'
    # column of samples.csv (706 rows, 237 s): with these three the replay
    # in validate_gate.py lands every row within 0.02 s of the record (rms
    # 0.0065 s; 't' is rounded to 0.01 s). Each also has a direct reading:
    #
    # One px4-listener round trip. A row is three of them, so the sample
    # period is 0.3 + 3 x 0.00535 = 0.3161 s, the mean of 689 in-phase
    # intervals of samples.csv. (tail_rtf_probe read 1502 listener calls in
    # 6.0 s at idle, 4.0 ms each: sitl_out/rtf_probe_summary.txt:7.)
    # Or a callable (topic, wall) -> seconds, e.g. to clock a replay by a
    # record's own 't' column (validate_gate.py).
    listener_s: Union[float, Callable[[str, float], float]] = 0.00535
    # One px4-commander / px4-param set call. The 정지 대기 -> 출발 gap,
    # 2.33 s = P + 2 c + 2.0 (two commands, two 1 s sleeps), gives c = 0.007;
    # the 회복 -> I 끔 gap, 4.19 s = P + 3 c + 3.0 + publish, gives 0.006.
    client_s: float = 0.006
    # One `gz topic -p` (gz_link.publish, gz_link.py:46-67). The gaps around
    # every wrench and wrench_clear are 1.17-1.18 s = P + 0.857.
    publish_s: float = 0.857
    # Where in the publish the wrench starts acting, as a fraction of
    # publish_s. 1.0 = when the call returns: in samples.csv the row taken
    # 0.016 s after the wrench() call returned still reads 0.050 deg/s and the
    # next, 0.316 s later, -1.816, which at the plant's 5.9 deg/s^2 puts the
    # onset within ~0.01 s of the return; the clear likewise acts ~0.1 s
    # before the Manual phase's first row (1.74 deg/s gained in 1.17 s).
    publish_effect: float = 1.0
    # A stick change reaches the FMU on the next MANUAL_CONTROL of a 20 Hz
    # wall-clock stream (sitl_link.py:283-299): 0 to 0.05 s. Mid value
    # assumed. Consistent with the records: the first row after a stick
    # change (0.016 s later) always shows the old stick (Manual 통과 row 0
    # torque 0.0000; Acro 제동 row 0 torque 0.5000; Acro 추종 row 0 0.0065).
    stick_delay_s: float = 0.025
    # vehicle_command -> the mode active in airship_att_control, after the
    # client returns. The commander client only publishes the command
    # (Commander.cpp:465-507, send_vehicle_command :166-186); the commander
    # loop runs every 10 ms (Commander.hpp:222). Fitted to Acro 제동 row 0 of
    # samples.csv: its torque (read ~14.0 ms after the command call started)
    # is still Manual's 0.5000, its integrator (read ~19.4 ms after) is
    # -0.00012, exactly one 4 ms loop cycle of 0.09 x -0.342 rad/s. So the
    # mode landed 8.0-13.4 ms after the client returned.
    command_delay_s: float = 0.011
    # Cosmetic: they set only where the 't' column starts. 4 param_show calls
    # plus the EKF wait = 8.1 s puts the first settle row at t = 24.98, as in
    # samples.csv. Nothing on the model moves before arming.
    param_show_s: float = 0.5
    ekf_wait_s: float = 6.08
    # sim seconds per wall second; a constant or a function of wall time.
    rtf: Union[float, Callable[[float], float]] = 1.0


class ClosedLoop:
    """What run_gate() drives. Implement these, or build one with compose().

    Time is SIM time everywhere here; the gate owns the wall clock and calls
    advance() with RTF x wall. advance() must accept any dt >= 0 (the gate
    steps in listener-sized pieces of ~2.7 ms as well as 0.3 s sleeps).
    Reads return what px4-listener would show at that instant: the latest
    published value.
    """

    def param(self, name):
        """Value of a firmware parameter, or None when the firmware lacks it.
        The gate reads AS_YAWRATE_MAX, MAN_DEADZONE, AS_YAWRATE_I and
        THR_MDL_FAC."""
        raise NotImplementedError

    def set_param(self, name, value):
        """The gate writes MAN_ARM_GESTURE (ignore it) and AS_YAWRATE_I."""
        raise NotImplementedError

    def arm(self):
        raise NotImplementedError

    def disarm(self):
        raise NotImplementedError

    def armed(self):
        raise NotImplementedError

    def set_mode(self, mode):
        """'manual', 'acro' or 'stabilized' (commander mode names)."""
        raise NotImplementedError

    def set_stick(self, yaw):
        """manual_control_setpoint.yaw in -1..1; the first call also marks
        the manual input valid (the sticks stream starts)."""
        raise NotImplementedError

    def set_wind(self, yaw_moment_frd_nm):
        """External yaw moment on the hull, body FRD z, N m; 0 clears."""
        raise NotImplementedError

    def advance(self, dt):
        raise NotImplementedError

    def read_rate(self):
        """vehicle_angular_velocity.xyz[2], rad/s, FRD."""
        raise NotImplementedError

    def read_torque(self):
        """vehicle_torque_setpoint.xyz[2], normalized."""
        raise NotImplementedError

    def read_integ(self):
        """rate_ctrl_status.yawspeed_integ, or None while never published
        (airship_att_control publishes it only while the loop is closed and
        once when it opens: airship_att_control_main.cpp:244, :250)."""
        raise NotImplementedError


class compose(ClosedLoop):
    """A ClosedLoop from three parts, stepped at the IMU rate.

    controller: update(dt, rate_meas, stick, mode, armed, sticks_valid,
                       unallocated) -> normalized yaw torque setpoint
                param(name), set_param(name, value), integ (None until
                published)
    actuation:  step(dt, torque_sp) -> (yaw moment N m FRD, unallocated
                normalized yaw torque; > 0 means a positive shortfall)
    plant:      step(dt, yaw_moment_frd_nm); rate (rad/s FRD, true); an
                optional measured_rate() for a gyro/filter model

    One tick = controller on the latest measurement and on the allocator
    status of the previous tick (airship_att_control_main.cpp:235 reads
    control_allocator_status before computing), then the actuation, then
    the plant over the tick with wind added. control_hz 250 = the IMU
    update_rate of both vehicles (airship_base/model.sdf:217,
    cloudship/model.sdf:242), which clocks the controller
    (airship_att_control_main.cpp:207-210).
    """

    def __init__(self, controller, actuation, plant, control_hz=250.0):
        self.controller, self.actuation, self.plant = controller, actuation, plant
        self.h = 1.0 / control_hz
        self._pending = 0.0
        self._armed = False
        self._sticks_valid = False
        self._stick = 0.0
        self._mode = 'manual'
        self._wind = 0.0
        self._unallocated = 0.0
        self._torque = 0.0
        self.sim = 0.0

    def param(self, name):
        return self.controller.param(name)

    def set_param(self, name, value):
        self.controller.set_param(name, value)

    def arm(self):
        self._armed = True

    def disarm(self):
        self._armed = False

    def armed(self):
        return self._armed

    def set_mode(self, mode):
        self._mode = mode

    def set_stick(self, yaw):
        self._stick = yaw
        self._sticks_valid = True

    def set_wind(self, yaw_moment_frd_nm):
        self._wind = yaw_moment_frd_nm

    def _measured(self):
        measure = getattr(self.plant, 'measured_rate', None)
        return measure() if measure else self.plant.rate

    def advance(self, dt):
        self._pending += dt

        while self._pending >= self.h - 1e-12:
            self._pending -= self.h
            self._torque = self.controller.update(self.h, self._measured(), self._stick, self._mode,
                                                  self._armed, self._sticks_valid, self._unallocated)
            moment, self._unallocated = self.actuation.step(self.h, self._torque)
            self.plant.step(self.h, moment + self._wind)
            self.sim += self.h

    def read_rate(self):
        return self._measured()

    def read_torque(self):
        return self._torque

    def read_integ(self):
        return self.controller.integ


class _Clock:
    """Virtual wall clock; sim advances by RTF x wall. Events are queued on
    the wall clock and fired in order inside sleeps."""

    def __init__(self, loop, timing):
        self.loop, self.timing = loop, timing
        self.wall = 0.0
        self.sim = 0.0
        self._queue = []
        self._seq = 0
        self.events = []

    def _rtf(self):
        rtf = self.timing.rtf
        return rtf(self.wall) if callable(rtf) else rtf

    def _run(self, wall_dt):
        if wall_dt <= 0.0:
            return

        sim_dt = self._rtf() * wall_dt
        self.loop.advance(sim_dt)
        self.wall += wall_dt
        self.sim += sim_dt

    def sleep(self, seconds):
        target = self.wall + max(0.0, float(seconds))

        while self._queue and self._queue[0][0] <= target:
            when, _, fn, label = heapq.heappop(self._queue)
            self._run(when - self.wall)
            fn()
            self.log(label)

        self._run(target - self.wall)

    def after(self, delay, fn, label):
        if delay <= 0.0:
            fn()
            self.log(label)
            return

        heapq.heappush(self._queue, (self.wall + delay, self._seq, fn, label))
        self._seq += 1

    def flush(self):
        if self._queue:
            self.sleep(max(when for when, _, _, _ in self._queue) - self.wall)

    def log(self, label):
        self.events.append((round(self.wall, 4), round(self.sim, 4), label))

    def time(self):
        return self.wall


def _lift_functions(path, names, env):
    """Named top-level functions of a source file, compiled on their own."""
    with open(path, encoding='utf-8') as handle:
        tree = ast.parse(handle.read(), path)

    nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in names]
    missing = set(names) - {n.name for n in nodes}

    if missing:
        raise LookupError('%s no longer defines %s' % (path, ', '.join(sorted(missing))))

    namespace = dict(env)
    exec(compile(ast.Module(body=nodes, type_ignores=[]), path, 'exec'), namespace)
    return {name: namespace[name] for name in names}


def _imported_names(path, module):
    """What `path` imports from `module`, so a fake that misses one fails
    loudly here instead of with an AttributeError mid-run."""
    with open(path, encoding='utf-8') as handle:
        tree = ast.parse(handle.read(), path)

    return {alias.name for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom) and node.module == module
            for alias in node.names}


def _load(name, path, modules):
    saved = {k: sys.modules.get(k) for k in modules}
    sys.modules.update(modules)

    try:
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v

    return module


def _fakes(clock, loop, timing, model_name):
    t = timing
    parsers = _lift_functions(SITL_LINK, ('field', 'vector', 'finite'), {'re': re, 'math': math})
    reads = []

    def fmt(value):
        return '%.5f' % value     # px4 listener prints float fields with %.5f (uORB.cpp:356)

    def render(topic):
        if topic == 'vehicle_angular_velocity':
            value = loop.read_rate()
            reads.append((clock.wall, clock.sim, topic, value))
            return ('TOPIC: %s\n %s\n    xyz: [0.00000, 0.00000, %s]\n' % (topic, topic, fmt(value)))

        if topic == 'vehicle_torque_setpoint':
            value = loop.read_torque()
            return 'TOPIC: %s\n %s\n    xyz: [0.00000, 0.00000, %s]\n' % (topic, topic, fmt(value))

        if topic == 'rate_ctrl_status':
            value = loop.read_integ()
            if value is None:
                return 'never published\n'     # listener_main.cpp:90,100
            return 'TOPIC: %s\n %s\n    yawspeed_integ: %s\n' % (topic, topic, fmt(value))

        if topic == 'vehicle_control_mode':
            return 'TOPIC: %s\n    flag_armed: %s\n' % (topic, 'True' if loop.armed() else 'False')

        if topic == 'vehicle_local_position':
            return 'TOPIC: %s\n    xy_valid: True\n    z_valid: True\n' % topic

        raise KeyError('gate.py does not render %s' % topic)

    def listener(topic):
        latency = t.listener_s(topic, clock.wall) if callable(t.listener_s) else t.listener_s
        clock.sleep(latency / 2)
        text = render(topic)
        clock.sleep(latency / 2)
        return text

    def commander(*args):
        clock.sleep(t.client_s)
        verb = args[0]

        if verb == 'mode':
            clock.after(t.command_delay_s, lambda: loop.set_mode(args[1]), 'mode ' + args[1])
        elif verb == 'arm':
            clock.after(t.command_delay_s, loop.arm, 'arm')
        elif verb == 'disarm':
            clock.after(t.command_delay_s, loop.disarm, 'disarm')
        elif verb != 'status':
            raise KeyError('gate.py does not handle commander %s' % (args,))

        return 'ok\n'

    def param_set(name, value):
        clock.sleep(t.client_s)
        clock.after(t.command_delay_s, lambda: loop.set_param(name, float(value)),
                    'param %s %g' % (name, float(value)))
        return ''

    def param_show(name, tries=15):
        clock.sleep(t.param_show_s)
        return loop.param(name)

    def wait_for_ekf(timeout=150):
        clock.sleep(t.ekf_wait_s)
        return True

    class Sticks:
        """sitl_link.Sticks: a 20 Hz MANUAL_CONTROL stream. Here each change
        of .yaw reaches the model stick_delay_s later."""

        def __init__(self):
            self.pitch = self.roll = self.throttle = 0
            self._yaw = 0
            self.stop = False

        @property
        def yaw(self):
            return self._yaw

        @yaw.setter
        def yaw(self, value):
            self._yaw = value
            clock.after(t.stick_delay_s, lambda: loop.set_stick(value), 'stick %g' % value)

        def start(self):
            value = self._yaw
            clock.after(t.stick_delay_s, lambda: loop.set_stick(value), 'sticks start %g' % value)

    sitl_link = types.ModuleType('sitl_link')
    sitl_link.MODEL_NAME = model_name
    sitl_link.MODEL = model_name + '_0'
    sitl_link.WORLD = 'lta'
    sitl_link.Sticks = Sticks
    sitl_link.commander = commander
    sitl_link.listener = listener
    sitl_link.param_set = param_set
    sitl_link.param_show = param_show
    sitl_link.wait_for_ekf = wait_for_ekf
    sitl_link.field = parsers['field']
    sitl_link.vector = parsers['vector']
    sitl_link.finite = parsers['finite']

    def publish_wind(torque_nm, label):
        clock.sleep(t.publish_s * t.publish_effect)
        loop.set_wind(GZ_WORLD_Z_TO_FRD * torque_nm)
        clock.log(label)
        clock.sleep(t.publish_s * (1.0 - t.publish_effect))
        return True

    gz_link = types.ModuleType('gz_link')
    gz_link.wrench = lambda torque_nm: publish_wind(torque_nm, 'wrench %g N m (gz world z)' % torque_nm)
    gz_link.wrench_clear = lambda: publish_wind(0.0, 'wrench clear')
    gz_link.TRACE = False       # sitl_check prints the publish trace only when this is set

    for module, fake in (('sitl_link', sitl_link), ('gz_link', gz_link)):
        missing = _imported_names(SITL_CHECK, module) - set(vars(fake))

        if missing:
            raise LookupError('sitl_check.py imports %s from %s; gate.py does not fake it'
                              % (', '.join(sorted(missing)), module))

    return sitl_link, gz_link, reads


class _Tee(io.StringIO):
    def __init__(self, echo):
        super().__init__()
        self.echo = echo

    def write(self, text):
        if self.echo:
            sys.__stdout__.write(text)
        return super().write(text)


@dataclass
class GateResult:
    status: Optional[int]          # main()'s return: 0 all pass, 1 a failure, 2 abort, 3 no loop
    lines: list                    # everything main() printed, in order
    samples: list                  # sitl_check's rows, plus 'sim': sim time of the rate read
    events: list                   # (wall, sim, what) as each command reached the model
    results: list = dc_field(default_factory=list)   # report.results(): [(name, ok)]

    @property
    def gate_lines(self):
        """The lines a chain log keeps (`grep -E "통과|실패"`)."""
        return [line for line in self.lines if '통과' in line or '실패' in line]

    @property
    def verdicts(self):
        """[(name, ok, detail)] from the verdict lines, in order."""
        found = []

        for line in self.lines:
            m = re.match(r'^  (통과|실패)  (.*?) — (.*)$', line)
            if m:
                found.append((m.group(2), m.group(1) == '통과', m.group(3)))

        return found


def run_gate(loop, model_name, timing=None, echo=False, out_dir=None):
    """Run sitl_check.main() on `loop` for gz model `model_name`
    (e.g. 'cloudship_am_fin_drag'; it picks the VEHICLES entry, the wind
    torques and the wind-check expectation exactly as the gate does)."""
    timing = timing or Timing()
    clock = _Clock(loop, timing)
    sitl_link, gz_link, reads = _fakes(clock, loop, timing, model_name)
    report = _load('report', REPORT, {})
    check = _load('sitl_check_on_model', SITL_CHECK,
                  {'sitl_link': sitl_link, 'gz_link': gz_link, 'report': report, 'gz_model': gz_model})
    check.time = types.SimpleNamespace(time=clock.time, sleep=clock.sleep)
    check.T0 = 0.0
    check.OUT = out_dir or os.path.join(HERE, 'gate_out')
    buffer = _Tee(echo)

    with contextlib.redirect_stdout(buffer):
        status = check.main()

    clock.flush()      # the final disarm is still queued when main() returns

    samples = [dict(row) for row in check.samples]

    for row, read in zip(samples, reads):
        row['sim'] = round(read[1], 4)

    return GateResult(status, buffer.getvalue().splitlines(), samples, clock.events, report.results())


if __name__ == '__main__':
    import argparse
    from reference_loop import reference_loop

    parser = argparse.ArgumentParser(description='the SITL gate on the reference loop')
    parser.add_argument('model', nargs='?', default='cloudship_am_fin_drag')
    parser.add_argument('--rtf', type=float, default=1.0)
    args = parser.parse_args()
    result = run_gate(reference_loop(args.model), args.model, Timing(rtf=args.rtf), echo=True)
    sys.exit(result.status)
