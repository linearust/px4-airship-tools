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


_node = None
_publishers = {}


def _publisher(name, message_type, wait=10.0):
    """A publisher on `name` that lives as long as the process and is handed
    out only once the simulator's subscriber is connected to it, or None when
    none turned up within `wait` s (no world up, or one without the
    ApplyLinkWrench system).

    `gz topic -p` was a publisher that lived about 0.8 s: the subscriber had
    to discover it inside that, and the exit code could not tell whether it
    did. A clear lost that way left the previous wind case's torque on the
    hull, and the next case then held the rate at zero with an integrator at
    exactly twice the torque it should have needed (2026-09-24, and again
    2026-10-12 on the vectored airship's gate).
    """
    global _node

    try:
        import gz.transport13 as transport
    except ImportError:
        raise SystemExit('gz 파이썬 전송 바인딩이 없다: python3-gz-transport13 과 python3-gz-msgs10')

    if _node is None:
        _node = transport.Node()

    publisher = _publishers.get(name)

    if publisher is None:
        publisher = _node.advertise(name, message_type)
        _publishers[name] = publisher

    deadline = time.time() + wait

    while not publisher.has_connections():
        if time.time() > deadline:
            return None

        time.sleep(0.05)

    return publisher


def publish(name, message):
    """True when a connected subscriber took the message."""
    start = time.time()
    publisher = _publisher(name, type(message))
    ok = publisher is not None and publisher.publish(message)

    if TRACE:
        print('gz_publish wall %.2f-%.2f %s %s' % (start, time.time(), name.rsplit('/', 1)[-1], ok), flush=True)

    return ok


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


def _name_model(entity, model):
    """Fill an Entity message (a clear, or the entity field of a wrench) with
    the model's name and type. Filled in place: the Entity class the wrench
    field carries and the one entity_pb2 exports are two loads of the same
    descriptor, and CopyFrom() refuses one for the other."""
    entity.name = model
    entity.type = entity.MODEL
    return entity


def _wrench_message(model, torque_z=0.0, force_z=0.0):
    from gz.msgs10.entity_wrench_pb2 import EntityWrench
    message = EntityWrench()
    _name_model(message.entity, model)
    message.wrench.torque.z = torque_z
    message.wrench.force.z = force_z
    return message


def _clear_message(model):
    from gz.msgs10.entity_pb2 import Entity
    return _name_model(Entity(), model)


def wrench(torque_nm):
    """A persistent yaw couple on the hull. ApplyLinkWrench listens on topics.

    The model, always. There used to be an `entity` parameter, and the only
    thing it could name that MODEL does not is a link - which with the
    `type: MODEL` below matches nothing, so the wrench silently applies and the
    check measures the resulting stillness. A link wrench needs `type: LINK`
    as well (aero_moment_check.py's probe world is the example), so the two go
    together or not at all.

    Published once, to a connected subscriber: persistent wrenches add, so a
    repeat would double it. A caller that must have the push still judges by
    its effect and re-pushes after a clear (sitl_check.wind_case); False here
    means the simulator had no subscriber to take it.
    """
    model, world = _target()
    return publish('/world/%s/wrench/persistent' % world, _wrench_message(model, torque_z=torque_nm))


def push_up(force_n):
    """A persistent vertical force on the hull, N, positive up (gz's world z).
    The same message and the same rules as wrench(): published once, adds to
    any other persistent wrench, removed by wrench_clear()."""
    model, world = _target()
    return publish('/world/%s/wrench/persistent' % world, _wrench_message(model, force_z=force_n))


def wrench_clear(times=1):
    """Clear every persistent wrench on the model: idempotent, sent `times`
    times in a row. From a live publisher a clear costs no wall clock, so
    nothing after it moves in time (three CLI calls in a row used to shift
    every check after a wind case: 2026-09-28, chain12)."""
    model, world = _target()
    ok = True

    for _ in range(times):
        ok = publish('/world/%s/wrench/clear' % world, _clear_message(model)) and ok

    return ok
