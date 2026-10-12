#!/usr/bin/env python3
"""The one way in to a running gz world.

Four files reached into gz with their own `gz topic` calls and their own
parsing, and the copies had already gone wrong in two ways worth naming.

A failed read looked like data. Every copy swallowed the timeout and returned
an empty string, and the caller turned that into a zero or a NaN and compared
it against a threshold. Every comparison against NaN is False, so "no reading"
arrived at the verdict as "within tolerance". Here a read that did not happen
returns None, and it is the caller's business to say so.

The motor-speed array was sliced by the vehicle's motor count. One file took
`values[-3:]`, another `values[-2:]`, a third indexed from the front, and each
was right only for the vehicle it was written against - point the -2 form at
the Cloudship and it reads the port thruster while the check commands the
starboard one, then reports the wrong conclusion about the firmware. There is
one reader now and it returns the whole array from one message, so the caller
indexes by actuator number and no motor count is assumed anywhere.
"""
import os
import re
import subprocess
import threading
import time

TRACE = os.environ.get('GZ_LINK_TRACE') == '1'   # print every publish with its wall-clock span


def _target():
    """(model, world) of the SITL run, read on use: the pose parsing here is
    also imported by the probe-world checks, which have no SITL target and
    must not resolve one (sitl_link exits on a model it cannot find)."""
    import sitl_link
    return sitl_link.MODEL, sitl_link.WORLD


def topic(name, count=1, timeout=25):
    """Echo `count` messages off a gz topic. None when nothing was read.

    None and '' are different answers: '' is a topic that published an empty
    message, None is a read that did not happen. Only the caller knows which
    of those is fatal, so this does not decide.
    """
    try:
        done = subprocess.run(['gz', 'topic', '-e', '-t', name, '-n', str(count)],
                              capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return None
    except FileNotFoundError:
        raise SystemExit('gz 가 PATH 에 없다')

    return done.stdout if done.returncode == 0 else None


def publish(name, kind, payload, timeout=10, tries=3):
    """True when `gz topic` accepted the message; False after `tries`.

    The return code used to be ignored, and a wrench clear that gz never took
    left the previous wind case's torque on the hull: the next case then held
    the rate at zero with an integrator at exactly twice the torque it should
    have needed (2026-09-24, the vectored airship's gate). A publish that
    fails is retried, and a caller that gets False has something to report.
    """
    for attempt in range(tries):
        start = time.time()

        try:
            done = subprocess.run(['gz', 'topic', '-t', name, '-m', kind, '-p', payload],
                                  capture_output=True, text=True, timeout=timeout)
        except (subprocess.TimeoutExpired, FileNotFoundError):
            done = None

        if TRACE:
            print('gz_publish wall %.2f-%.2f %s rc %s' % (start, time.time(), name.rsplit('/', 1)[-1],
                                                          None if done is None else done.returncode), flush=True)

        if done is not None and done.returncode == 0:
            return True

        time.sleep(1.0)

    return False


def messages(text):
    """One entry per message in an echo dump.

    `gz topic -e -n N` concatenates N messages, and every reader here wants
    whole messages rather than a flat list of fields: a slice taken across the
    concatenation can straddle two publishes.
    """
    return text.split('header {')[1:] if text else []


def stamp(block):
    """Simulator time on one message, in seconds. None when it has no stamp.

    The simulator's clock, which is the one the plant runs on. A displacement
    measured here and divided by a wall-clock sleep is a rate in neither.
    """
    found = re.search(r'stamp \{\s*sec:\s*(-?\d+)(?:\s*nsec:\s*(-?\d+))?', block)
    return None if not found else int(found.group(1)) + int(found.group(2) or 0) * 1e-9


def pose_topic(dynamic=True):
    """dynamic_pose/info carries what moved this step; pose/info carries every
    entity every publish, which is the one to read for a link parked at a
    joint limit (a flap held at +25 deg dropped out of the dynamic stream
    on one run in four while its servo topic stood at the limit)."""
    return '/world/%s/%s/info' % (_target()[1], 'dynamic_pose' if dynamic else 'pose')


def poses(entity, limit=40, timeout=40, dynamic=True):
    """[(sim_time, (x, y, z), (w, x, y, z))] for one entity, oldest first.

    `entity` is the name as it appears in the pose stream: a model's spawn
    name, or a bare link name (relative to its parent).
    """
    text = topic(pose_topic(dynamic), limit, timeout)
    return None if text is None else parse_poses(text, entity)


class Capture:
    """A pose stream opened BEFORE the stimulus it is to record.

    `gz topic -e` takes 0.3-1 s of wall to attach, and a record that starts
    after a stick has moved cannot say when the stick moved or what the
    first half second did: the peak of a 0.4 s rise is gone. So the reader
    is started first, the caller waits a moment, applies the stimulus, and
    collects the rows afterwards; the onset is then in the record.
    """

    def __init__(self, count, dynamic=True):
        self.count = count
        self.process = subprocess.Popen(['gz', 'topic', '-e', '-t', pose_topic(dynamic), '-n', str(count)],
                                        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)

    def collect(self, entity, timeout=60):
        """The rows for `entity`, or None when the stream did not finish."""
        try:
            text, _ = self.process.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.communicate()
            return None

        return parse_poses(text, entity) if self.process.returncode == 0 else None


def parse_poses(text, entity):
    """The rows of one entity out of an echo dump of a pose topic."""
    rows = []

    for block in messages(text):
        when = stamp(block)

        if when is None:
            continue

        for entry in block.split('pose {')[1:]:
            if not re.search(r'name:\s*"%s"' % re.escape(entity), entry):
                continue

            position = re.search(r'position \{([^}]*)\}', entry)
            orientation = re.search(r'orientation \{([^}]*)\}', entry)
            axes = lambda text: {k: float(v) for k, v in
                                 re.findall(r'\b(x|y|z|w):\s*(-?[\d.eE+-]+)', text)}
            p = axes(position.group(1)) if position else {}
            q = axes(orientation.group(1)) if orientation else {}
            rows.append((when,
                         (p.get('x', 0.0), p.get('y', 0.0), p.get('z', 0.0)),
                         (q.get('w', 1.0), q.get('x', 0.0), q.get('y', 0.0), q.get('z', 0.0))))
            break

    rows.sort()
    return rows


def motor_speeds():
    """Everything the bridge is handing the motor models, in actuator order.

    Index 0 is SIM_GZ_EC_FUNC1, index 1 FUNC2, and so on, for as many outputs
    as the bridge found set - GZMixingInterfaceESC::updateOutputs publishes the
    run of consecutive functions from index 0. ONE message: that is the literal
    1 below, and raising it would put 2N velocities in one list and shift every
    actuator index silently.
    """
    text = topic('/%s/command/motor_speed' % _target()[0], 1)

    if text is None:
        return None

    values = re.findall(r'velocity:\s*(-?[\d.eE+-]+)', text)
    return [float(v) for v in values] if values else None


def motor_speed(index):
    """One bridge output, or None when it was not read or was not published.

    The bounds check is the half that decides whether a short publish reads as
    a number instead of as an absence. Three files had their own copy of it,
    each with its own idea of how many motors the vehicle has.
    """
    speeds = motor_speeds()
    return None if speeds is None or len(speeds) <= index else speeds[index]


_repeats = None    # the thread still sending the repeats of the last clear


def wrench(torque_nm):
    """A persistent yaw couple on the hull. ApplyLinkWrench listens on topics.

    The model, always. There used to be an `entity` parameter, and the only
    thing it could name that MODEL does not is a link - which with the
    `type: MODEL` below matches nothing, so the wrench silently applies and the
    check measures the resulting stillness. A link wrench needs `type: LINK`
    as well (aero_moment_check.py's probe world is the example), so the two go
    together or not at all.

    Waits for the repeats of the last clear first: one landing after this
    push would clear it.

    Published once. publish() re-sends after a timeout or a non-zero exit,
    and gz may already have taken the message by then (the CLI publishes
    about 0.8 s in, then tears down); a second persistent wrench adds. A
    caller that must have the push judges by its effect and re-pushes after
    a clear (sitl_check.wind_case), so False here means not confirmed, not
    absent.
    """
    _finish_clear()
    model, world = _target()
    return publish('/world/%s/wrench/persistent' % world, 'gz.msgs.EntityWrench',
                   'entity: {name: "%s", type: MODEL}, wrench: {torque: {z: %f}}'
                   % (model, torque_nm), tries=1)


def _clear_once():
    model, world = _target()
    return publish('/world/%s/wrench/clear' % world, 'gz.msgs.Entity', 'name: "%s", type: MODEL' % model)


def _finish_clear():
    global _repeats

    if _repeats is not None:
        _repeats.join()
        _repeats = None


def push_up(force_n):
    """A persistent vertical force on the hull, N, positive up (gz's world z).
    The same message and the same rules as wrench(): published once, adds to
    any other persistent wrench, removed by wrench_clear()."""
    _finish_clear()
    model, world = _target()
    return publish('/world/%s/wrench/persistent' % world, 'gz.msgs.EntityWrench',
                   'entity: {name: "%s", type: MODEL}, wrench: {force: {z: %f}}'
                   % (model, force_n), tries=1)


def wrench_clear(times=3):
    """Clear every persistent wrench on the model.

    `gz topic -p` exits 0 even when the simulator never took the message (a
    short-lived publisher can leave before the subscriber has discovered it),
    so the return code cannot tell a dropped clear, and a dropped one leaves
    the push on for the rest of the run: persistent wrenches add, so every
    later wind case then holds the sum (2026-09-28: the Cloudship's integrator
    at 0.2418, which is (1 + 0.083) N m, against 0.0185 needed). A clear is
    idempotent, so it is sent `times` times; a push is sent once, because a
    repeated push would add.

    Only the first is sent before this returns, and its result is the return
    value. The others go out from a thread: each `gz topic` call costs wall
    clock, and three in a row moved everything after the clear. After the
    wind check the Cloudship's complete plant then entered the Manual check
    3.6 s after the push instead of 1.2 s, at -14.3 instead of -21.7 deg/s,
    and every Acro check after it started from a different state (2026-09-28,
    chain12: the Acro peak 9.46 against 8.96-9.02 before). wrench() waits for
    the repeats before the next push.
    """
    global _repeats
    _finish_clear()
    ok = _clear_once()

    def repeat():
        for _ in range(times - 1):
            time.sleep(0.2)
            _clear_once()

    _repeats = threading.Thread(target=repeat)
    _repeats.start()
    return ok
