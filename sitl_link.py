#!/usr/bin/env python3
"""The one way in to a running headless gz SITL airship, on the px4 side.

Every check here talks to px4 through its own client binaries, and each
carried its own copy of these helpers until the copies started to differ. Same
reason fmu_link.py exists for the bench FMU: one place for the addresses, the
retries and the parsing. gz_link.py is the other half, for the simulator, and
gz_model.py the third, for the model files.

Two things this module got wrong for a long time, both fixed here.

**The entity name.** MODEL used to prefer PX4_SIM_MODEL, which is the one
variable that cannot select anything: the make target is built as
`cmake -E env PX4_SIM_MODEL=gz_<model> ... px4`, and `cmake -E env` SETS rather
than defaults, so whatever the caller exported never reaches px4. GZ_TARGET
picks the target and therefore the entity. It also assumed the target is
exactly `gz_<model>`, when CMake also generates `gz_<model>_<world>` for every
non-default world. Both mistakes produce the same failure - a name that exists
nowhere - and that failure is silent: wrenches go to an entity that is not
there and the checks measure the resulting stillness. So the name is resolved
in the order that actually decides it and then CHECKED against the models
directory, because a stale name is worse than an error here.

**A failed read looked like a reading.** shell() swallowed the timeout and
returned '', the caller turned that into NaN, and every guard was a strict
comparison - `abs(rate) > 2.0`, `abs(torque) < 0.02` - which NaN fails, so
"could not read" arrived at the verdict as "within tolerance". The gate's own
comment called that "the most believable wrong answer this script can give"
and the guard was written in the one form it walks through. Now a read that
did not happen returns None all the way up, and `finite()` is here for the
guards that have to insist.
"""
import math
import os
import re
import subprocess
import threading
import time

from pymavlink import mavutil

import gz_model

BIN = os.path.join(gz_model.ROOT, 'build/px4_sitl_default/bin')
WORLD = os.environ.get('PX4_GZ_WORLD', 'lta')
INSTANCE = os.environ.get('PX4_INSTANCE', '0')


def _model_name():
    """The model px4 spawns, from whatever actually selected it.

    PX4_GZ_ENTITY first: gz_run.sh resolves the target once and exports it, so
    inside a managed run there is nothing left to infer. Otherwise GZ_TARGET,
    which is what `make px4_sitl <target>` consumed. PX4_SIM_MODEL last, and
    only for the case this module cannot see - a check attached by hand to a
    world someone else started.
    """
    resolved = os.environ.get('PX4_GZ_ENTITY')

    if resolved:
        # gz_run.sh split the target against the worlds that exist. Taking it
        # verbatim is the point: guessing again here could only disagree.
        return resolved

    name = (os.environ.get('GZ_TARGET')
            or os.environ.get('PX4_SIM_MODEL')
            or 'gz_airship_vectored_independent').removeprefix('gz_')

    # `gz_<model>_<world>` is as legitimate a target as `gz_<model>`; only the
    # first form names the model on its own. This is the hand-attached case,
    # where the world is whatever PX4_GZ_WORLD says and nothing else is known.
    if name.endswith('_' + WORLD):
        name = name[:-len(WORLD) - 1]

    return name


MODEL_NAME = _model_name()
MODEL = '%s_%s' % (MODEL_NAME, INSTANCE)

if not gz_model.has_model(MODEL_NAME):
    raise SystemExit(
        '%s 라는 모델이 %s 아래에 없다. GZ_TARGET/PX4_GZ_ENTITY 를 확인한다.\n'
        '이름을 못 맞히면 모든 렌치가 없는 엔티티로 가고 검사는 그 정적을 측정한다.'
        % (MODEL_NAME, ', '.join(gz_model.MODEL_HOMES)))

# One line of `param show -a`. It prints three flag columns with single spaces
# between them ("%c %c %c %s": used / unsaved-or-changed / readonly), so the
# prefix is not a run of flag characters, and a pattern that assumed it was
# read every parameter the caller had just written as absent. `python3
# sitl_link.py` checks this against all four shapes.
PARAM_LINE = r'^[x+*l ]*%s\s+\[-?\d+,\s*-?\d+\]\s*:\s*([-+0-9.eE]+)'


def shell(args, timeout=10):
    """stdout, or None when the command did not answer.

    None is not ''. An empty answer from a command that ran is a fact; a
    command that timed out is the absence of one, and every caller that folds
    the two together ends up asserting on a value it never read.

    A non-zero exit is an answer, not an absence. Several px4 clients report
    through their exit code - `px4-commander status` among them - so gating on
    returncode turned every `if not commander('status')` into an immediate
    'px4 에 붙지 못했다' against a perfectly healthy simulator.
    """
    try:
        done = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return None
    except FileNotFoundError:
        raise SystemExit('%s 가 없다. 먼저 make px4_sitl 로 빌드한다.' % args[0])

    return done.stdout


def px4(*args, timeout=15):
    """One px4-<client> call. Five files had their own copy of this line.

    To PX4_INSTANCE's px4, as MODEL names its hull: the client talks to
    instance 0 unless --instance comes right after the binary path.
    """
    instance = [] if INSTANCE == '0' else ['--instance', INSTANCE]
    return shell([os.path.join(BIN, 'px4-' + args[0])] + instance + [str(a) for a in args[1:]],
                 timeout=timeout)


def listener(topic):
    return px4('listener', topic, '1', timeout=5)


def commander(*args):
    return px4('commander', *args)


def param_set(name, value):
    return px4('param', 'set', name, value)


def param_show(name, tries=15):
    """The parameter's value, or None when it is not in this firmware.

    `param show <name>` lists a parameter only after some module has
    registered it: two seconds after boot 54 of 2095 are visible, ten
    seconds later 940 are. So an empty answer there means "not yet",
    not "not built in", and reading it as absence makes this script
    quietly measure the baseline and report a pass for the yaw-rate
    checks it never ran.

    `param show -a` dumps the whole table regardless, so a name missing
    from that dump really is missing.
    """
    for _ in range(tries):
        text = px4('param', 'show', '-a', timeout=30)

        if text is not None:
            found = re.search(PARAM_LINE % name, text, re.M)

            if found:
                return float(found.group(1))

            if 'parameters total' in text:
                return None

        time.sleep(2)

    raise RuntimeError('px4-param 이 응답하지 않는다: px4 가 떠 있는지 본다')


def field(text, name):
    if text is None:
        return None

    found = re.search(r'^\s*%s:\s*([-+0-9.eE]+|True|False)' % name, text, re.M)

    if not found:
        return None

    value = found.group(1)

    if value in ('True', 'False'):
        return 1.0 if value == 'True' else 0.0

    return float(value)


def vector(text, name):
    if text is None:
        return None

    found = re.search(r'^\s*%s:\s*\[([^\]]*)\]' % name, text, re.M)
    return [float(v) for v in found.group(1).split(',')] if found else None


def finite(*values):
    """True only when every value is a real number.

    The guards that must not fail open call this first. `abs(nan) > 2.0` is
    False and so is `abs(nan) < 0.02`, so a threshold alone cannot tell a
    quiet vehicle from an unread one.
    """
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


def angular_rate():
    """Body roll, pitch, yaw rates in deg/s, or None when unread."""
    xyz = vector(listener('vehicle_angular_velocity'), 'xyz')
    return None if xyz is None else [math.degrees(v) for v in xyz]


def settle_rate(axis=2, limit=5.0, deadline=80.0, period=2.0):
    """Wait, bounded, for one body rate to fall below `limit`. deg/s, or None.

    Bounded and reported: a caller that cannot tell "it settled" from "it ran
    out of time" will read the next measurement as if it started from rest.
    """
    end = time.time() + deadline

    while time.time() < end:
        rate = angular_rate()

        if rate is not None and abs(rate[axis]) < limit:
            return rate[axis]

        time.sleep(period)

    return None


def actuator_hold(pairs, timeout=20):
    """Drive several outputs at once, and keep driving them.

    ActuatorTest keeps one override per output but a SINGLE timeout, reset by
    whichever command arrives last, and it clears every override the moment it
    leaves test mode. So a set of outputs has to be re-issued together, and
    often enough that the session never lapses between them.

    The timeout is counted in the SIMULATOR's clock while a caller's sleep is
    counted in this machine's, and the two are equal only when the sim happens
    to be keeping up. That is why holding means re-issuing rather than
    commanding once and sleeping: a tail that pushed and was then let go reads
    exactly like one that never pushed.
    """
    for function, value in pairs:
        px4('actuator_test', 'set', '-f', int(function), '-v', '%.4f' % value,
            '-t', int(timeout))


def actuator_release(functions):
    for function in functions:
        px4('actuator_test', 'set', '-f', int(function), '-v', '0', '-t', 1)


def wait_for_ekf(timeout=150):
    end = time.time() + timeout

    while time.time() < end:
        text = listener('vehicle_local_position')

        if text and len(re.findall(r'^\s*(xy_valid|z_valid): True', text, re.M)) == 2:
            return True

        time.sleep(1)

    return False


class Sticks(threading.Thread):
    """Fake GCS: heartbeat at 1 Hz, MANUAL_CONTROL at 20 Hz.

    pitch, roll and yaw in -1..1 as PX4 reads them; throttle in 0..1 as the
    THRUST FRACTION the airship makes of it. MAVLink carries x, y, r as
    -1000..1000 and z as 0..1000; the receiver reads throttle = z / 500 - 1
    and the airship's forward thrust is (throttle + 1) / 2 = z / 1000. So z
    is throttle x 1000 here, and 0 keeps the thrusters off, which is what
    every check before this one relied on.
    """

    def __init__(self):
        super().__init__(daemon=True)
        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.throttle = 0
        self.stop = False
        # px4-rc.mavlink binds the GCS link of instance N to 18570 + N
        self.link = mavutil.mavlink_connection('udpout:127.0.0.1:%d' % (18570 + int(INSTANCE)),
                                               source_system=255)

    def run(self):
        last = 0.0

        while not self.stop:
            now = time.time()

            if now - last >= 1.0:
                self.link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                             mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                last = now

            # Clamped: the receiver drops a z outside 0..1000 and the airship then
            # reads its zero-initialised throttle as HALF thrust, silently.
            axis = lambda v: int(max(-1.0, min(1.0, v)) * 1000)
            # MAV_SYS_ID is px4_instance + 1 (rcS), and the receiver drops a
            # MANUAL_CONTROL addressed to another system
            self.link.mav.manual_control_send(int(INSTANCE) + 1, axis(self.pitch), axis(self.roll),
                                              int(max(0.0, min(1.0, self.throttle)) * 1000), axis(self.yaw), 0)
            time.sleep(0.05)


if __name__ == '__main__':
    # Not a test suite, just the parses that have bitten. `param show` prints
    # three flag columns with single spaces between them, and a regex that
    # allowed only a run of flag characters read every parameter the caller had
    # just written as absent. The model name has bitten twice: once by trusting
    # PX4_SIM_MODEL over GZ_TARGET, once by keeping a world suffix.
    bad = 0

    SAMPLES = (
        ('      SIM_GZ_EC_MIN1 [123,45] : 0.0000', 0.0, 'unused, default'),
        ('x     SIM_GZ_EC_MIN1 [123,45] : 0.0000', 0.0, 'used, default'),
        ('x +   SIM_GZ_EC_MIN1 [123,45] : -300.0000', -300.0, 'used, changed'),
        ('x * l SIM_GZ_EC_MIN1 [123,45] : -300.0000', -300.0, 'used, unsaved, readonly'),
    )

    for line, want, note in SAMPLES:
        found = re.search(PARAM_LINE % 'SIM_GZ_EC_MIN1', line, re.M)
        got = float(found.group(1)) if found else None
        ok = got == want
        bad += not ok
        print('  %s  %-26s %s' % ('통과' if ok else '실패', note,
                                  got if got is not None else '매치 실패'))

    NAMES = (
        ({'PX4_GZ_ENTITY': 'cloudship'}, 'cloudship', 'gz_run.sh 가 풀어 준 이름'),
        ({'GZ_TARGET': 'gz_cloudship'}, 'cloudship', 'GZ_TARGET, 기본 월드'),
        ({'GZ_TARGET': 'gz_cloudship_lta'}, 'cloudship', 'GZ_TARGET, 월드 접미사'),
        ({'GZ_TARGET': 'gz_cloudship', 'PX4_SIM_MODEL': 'airship_vectored_independent'},
         'cloudship', 'GZ_TARGET 가 PX4_SIM_MODEL 을 이긴다'),
        ({'PX4_SIM_MODEL': 'gz_cloudship'}, 'cloudship', 'PX4_SIM_MODEL 만, 접두사 붙은 채'),
    )
    kept = {k: os.environ.get(k) for k in
            ('PX4_GZ_ENTITY', 'GZ_TARGET', 'PX4_SIM_MODEL')}

    for env, want, note in NAMES:
        for k in kept:
            os.environ.pop(k, None)
        os.environ.update(env)
        got = _model_name()
        ok = got == want
        bad += not ok
        print('  %s  %-30s %s' % ('통과' if ok else '실패', note, got))

    for k, v in kept.items():
        os.environ.pop(k, None)
        if v is not None:
            os.environ[k] = v

    print('MODEL = %s, WORLD = %s' % (MODEL, WORLD))
    raise SystemExit(1 if bad else 0)
