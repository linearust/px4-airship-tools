#!/usr/bin/env python3
"""Check that the hull drag pitches and rolls the airship the way it should.

gz's Hydrodynamics plugin both applies its wrench at the link origin and
reads the velocity there, so the coefficient set in
airship_vectored_independent_drag is written about that origin and not about
the hull axis the drag acts on. Shifting a quadratic drag set off its own axis
costs six. The condition is that the plugin's wrench be the hull's, which is
two things: the force it applies must be the force on the hull, driven by the
velocity at the axis, and the couple must be d x F for the offset d between
the two points. With h that offset, each in-plane axis gains 2h*c on the
force, h*c as the arm, and 2h^2*c as the arm's rate term. Leave the arm out and the static moment comes out backwards;
leave the rate terms out and the roll damping comes out backwards, which is worse,
because then the hull rolls itself into a limit cycle in any real sideslip.

Two checks, cheap first:

  analytic  the six cross terms against the offset read out of the model
            files, so a change to the geometry that nobody carried into the
            coefficients is caught without starting gz
  sim       push the hull through the air with a force that passes through the
            center of mass, then check both that the settled attitude matches
            this geometry's prediction and that nothing is still swinging

The sim check is the slow one and needs gz on the path; pass --analytic to
skip it. Both report in the model's own units, not in a score.
"""
import argparse
import math
import os
import re
import shutil
import statistics
import subprocess
import sys
import tempfile
import time

import gz_link
import gz_model
from gz_model import GZ_MODELS, GZ_PLUGINS, named, sdf, zof
from report import note, summary, verdict
# Each vehicle is the model that carries base_link first, then whatever else
# adds mass to it. One entry is one airframe's worth of files, however many
# that turns out to be: the big airship is three merged models and the
# Cloudship is one.
VEHICLES = {
    'airship': gz_model.family('airship_vectored_independent_drag'),
    'cloudship': gz_model.family('cloudship_drag'),
}
G = 9.8  # gz's default gravity, not 9.80665: the check must predict what runs
TARGET_TILT = 0.0214  # the settled tilt the default force aims for, in world z of a body axis


def geometry(models):
    """Mass, center of mass, center of volume and hull axis, from the models.

    The mass and the com come from every link of the family; the volume from
    base_link's collisions alone. Pods, props and rods hang off the body
    without displacing anything, so they change the weight and not the lift.
    Where they hang matters: the big airship's all sit on its center of mass
    plane and move nothing, the Cloudship's tail prop sits 0.5 m below the
    rest and moves the whole vehicle's center of mass by 1.5 mm, which is 0.3%
    of the restoring arm.
    """
    mass, com = gz_model.centre_of_mass(models)
    link = named(sdf(models[0]), 'link', 'base_link')
    # The com is in the model frame; the collisions are in base_link's, so
    # base_link's own pose is what joins the two (centre_of_volume adds it).
    hull_axis = zof(link) + zof(named(link, 'collision', 'hull_collision'))
    vol, cov = gz_model.centre_of_volume(models)
    return mass, com[0], com[2], cov[2], hull_axis, vol


COEFFICIENT = re.compile(r'^[xyzkmn][UVWPQR](abs[UVWPQR]|Dot[UVWPQR]|[UVWPQR])?$')


def coefficients(models):
    """The Hydrodynamics plugin's SNAME coefficients, by SDF element name.

    Matched against the plugin's own naming rule rather than a first letter,
    so a future <namespace> or <nR> neither slips in nor throws on the way.
    """
    for model in reversed(models):
        for plugin in sdf(model).findall('plugin'):
            if 'hydrodynamics' in plugin.get('filename', ''):
                return {c.tag: float(c.text)
                        for c in plugin if COEFFICIENT.match(c.tag)}
    raise LookupError('%s 에 Hydrodynamics 플러그인이 없다' % ', '.join(models))


# Everything a hull here is allowed to carry. The six cross terms, the three
# translational ones they scale from, and the rate damping on each axis.
EXPECTED = {'xUabsU', 'yVabsV', 'zWabsW', 'kP', 'kPabsP', 'mQabsQ', 'nRabsR',
            'xQabsU', 'mUabsU', 'mQabsU', 'yPabsV', 'kVabsV', 'kPabsV'}


def analytic(models):
    mass, com_x, com, cov, hull, vol = geometry(models)
    c = coefficients(models)
    stiffness = mass * G * (cov - com)
    note('기하      질량 %.3f kg, 무게중심 (%.4f, %.4f) m, 부력중심 z %.4f m, 선체축 z %.4f m'
         % (mass, com_x, com, cov, hull))
    note('          배수량 %.3f m^3, 복원 강성 %.2f N m/rad' % (vol, stiffness))

    # An airship that is not neutrally buoyant is not the vehicle, whatever
    # its coefficients say: it climbs or sinks through every check below and
    # the settled attitude those checks read is a transient. The collision
    # volumes and the declared mass are set against each other by hand;
    # cloudship_geom_check asserts the same thing from the template's side.
    lift = vol * gz_model.RHO_AIR
    verdict('중성부력', abs(lift - mass) <= max(1e-3, mass * 1e-3),
            '부력 %.5f kg, 질량 %.5f kg, 차이 %+.5f' % (lift, mass, lift - mass))

    # Only the offset between the two points enters: matching a wrench does not
    # depend on where you refer it, so the center of mass is absent here on
    # purpose. If it ever reappears, the derivation has matched a moment about
    # the center of mass instead of a wrench, and has left the force wrong.
    h = hull

    # Every coefficient present has to be one this check knows. The sim half
    # reaches equilibrium at p = q = 0, where every rate cross term is
    # identically zero, so no settled state can see a stray one - this is the
    # only place a coefficient that nothing here reads can be caught at all.
    extra = sorted(set(c) - EXPECTED)
    verdict('아는 계수만 있다', not extra,
            '%d 개' % len(c) if not extra else '모르는 항: %s' % ', '.join(extra))

    # The kinematic graph too: a joint naming a link that is not there, two
    # joints claiming the same child, a link with no path to the root, or a
    # plugin driving something that does not exist all make everything below
    # meaningless, and this is the cheapest thing in the file to check. The
    # family is merged before it is checked, because a wrapper's joints name
    # links that live in the model it merge-includes.
    links, joints, plugins = set(), set(), []
    for model in models:
        root = sdf(model)
        links |= {lk.get('name') for lk in root.findall('link')}
        joints |= {j.get('name') for j in root.findall('joint')}
        plugins += [(model, pl) for pl in root.findall('plugin')]

    children = []

    for model in models:
        for joint in sdf(model).findall('joint'):
            parent, child = joint.find('parent').text, joint.find('child').text
            verdict('조인트 %s' % joint.get('name'),
                    (parent in links or parent == 'world')
                    and child in links and child not in children,
                    '%s -> %s' % (parent, child))
            children.append(child)

    roots = links - set(children)
    verdict('뿌리 링크가 base_link 하나다', roots == {'base_link'},
            ', '.join(sorted(roots)) or '없음')

    # Which ELEMENT dangles, not just which plugin: the sibling check in
    # cloudship_geom_check names the tag and this one did not, so the two
    # reports of the same fact read differently.
    dangling = [(pl.get('name'), tag, pl.find(tag).text)
                for _, pl in plugins
                for tag, pool in (('linkName', links), ('link_name', links),
                                  ('jointName', joints), ('joint_name', joints))
                if pl.find(tag) is not None and pl.find(tag).text not in pool]
    verdict('플러그인 참조', not dangling,
            '%d 개 모두 실재' % len(plugins) if not dangling
            else '; '.join('%s 의 %s "%s"' % d for d in dangling))

    for moment_term, force_term, scale, label, axis in (
            ('xQabsU', 'xUabsU', 2 * h, '힘의 각속도', '서지'),
            ('mUabsU', 'xUabsU', h, '팔', '피치'),
            ('mQabsU', 'xUabsU', 2 * h * h, '팔의 각속도', '피치'),
            ('yPabsV', 'yVabsV', -2 * h, '힘의 각속도', '스웨이'),
            ('kVabsV', 'yVabsV', -h, '팔', '롤'),
            ('kPabsV', 'yVabsV', 2 * h * h, '팔의 각속도', '롤')):
        want = scale * c.get(force_term, 0.0)
        got = c.get(moment_term)

        if got is None:
            verdict(moment_term, False,
                    '<%s> 이 없다. %s 의 %s 항이 빠졌다' % (moment_term, axis, label))
            continue

        verdict(moment_term, abs(got - want) <= max(1e-4, abs(want) * 1e-3),
                '%+10.4f  기대 %+10.4f = %+8.4f * %s  (%s %s)'
                % (got, want, scale, force_term, axis, label))

    return com_x, com, hull, stiffness


def variant_models(aero_model, tmp, keep_cross):
    """Copy the aero model out, optionally with the cross terms deleted."""
    name = 'probe'
    dst = os.path.join(tmp, name)
    shutil.copytree(os.path.join(GZ_MODELS, aero_model), dst)
    text = open(os.path.join(dst, 'model.sdf')).read()
    if not keep_cross:
        text = re.sub(r'\s*<(xQabsU|yPabsV|mUabsU|mQabsU|kVabsV|kPabsV)>[^<]*</\1>', '', text)
    text = text.replace('<model name="%s">' % aero_model, '<model name="%s">' % name)
    open(os.path.join(dst, 'model.sdf'), 'w').write(text)
    open(os.path.join(dst, 'model.config'), 'w').write(
        '<?xml version="1.0"?>\n<model><name>%s</name><version>1.0</version>'
        '<sdf version="1.10">model.sdf</sdf></model>\n' % name)



# The world carries this process's id so that a gz server left behind by an
# interrupted run, which would happily answer for a hull that never saw this
# wrench, is simply a different world and answers nothing.
WORLD_NAME = 'probe_%d' % os.getpid()

WORLD = """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="%(world)s">
    <physics name="p" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-apply-link-wrench-system"
            name="gz::sim::systems::ApplyLinkWrench"/>
    <plugin filename="gz-sim-buoyancy-system" name="gz::sim::systems::Buoyancy">
      <uniform_fluid_density>%(rho)s</uniform_fluid_density>
      <enable>probe::base_link</enable>
    </plugin>
    <include><uri>model://probe</uri><pose>0 0 30 0 0 0</pose></include>
  </world>
</sdf>
""" % {'world': WORLD_NAME, 'rho': gz_model.RHO_AIR}




def settle(aero_model, keep_cross, wrench, settle_s):
    """Build one variant, run one world against it, return the settled axes.

    The variant and the world are one step because they were three identical
    four-line blocks, and the failure of getting them apart is a world run
    against a resource directory nobody built - whose symptom is a simulator
    timeout rather than a check defect.
    """
    with tempfile.TemporaryDirectory() as tmp:
        variant_models(aero_model, tmp, keep_cross)
        return _settle(tmp, wrench, settle_s)


def _settle(tmp, wrench, settle_s):
    world = os.path.join(tmp, 'probe_world.sdf')
    open(world, 'w').write(WORLD)
    env = dict(os.environ, GZ_SIM_RESOURCE_PATH=tmp + ':' + GZ_MODELS, GZ_IP='127.0.0.1',
               GZ_SIM_SYSTEM_PLUGIN_PATH=GZ_PLUGINS)
    server = subprocess.Popen(['gz', 'sim', '-s', '-r', world], env=env,
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        time.sleep(6)
        if wrench is not None:
            subprocess.run(
                ['gz', 'topic', '-t', '/world/%s/wrench/persistent' % WORLD_NAME,
                 '-m', 'gz.msgs.EntityWrench', '-p',
                 'entity: {name: "probe::base_link", type: LINK}, wrench: {%s}' % wrench],
                env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=20)
        time.sleep(settle_s)
        # Long enough to see a swing grow, not just to see one. The pitch
        # axis goes unstable without mQabsU at an amplitude two orders below
        # the roll axis, so an absolute threshold alone would miss it; what
        # gives it away is that the swing is bigger at the end than at the
        # start.
        out = subprocess.run(
            ['gz', 'topic', '-e', '-t', '/world/%s/dynamic_pose/info' % WORLD_NAME, '-n', '2400'],
            env=env, capture_output=True, text=True, timeout=180).stdout
    except subprocess.TimeoutExpired:
        note('  gz 가 응답하지 않는다')
        return None
    finally:
        server.kill()
        server.wait()
    return poses(out)


def poses(text):
    """(t, world z of body x, world z of body y, (w, x, y, z)) for the probe, oldest first."""
    return [(t,
             2.0 * (x * z - w * y),    # world z of the body x axis
             2.0 * (y * z + w * x),    # world z of the body y axis
             (w, x, y, z))
            for t, _, (w, x, y, z) in gz_link.parse_poses(text, 'probe')]


def measure(rows, column):
    """Settled value, the swing still on top of it, and whether it is growing.

    Growth is the real test. A swing that is merely present may be a transient
    the window caught; one that is larger at the end of the window than at the
    start is a plant feeding itself, and that is what a missing rate cross
    term does.
    """
    third = max(1, len(rows) // 3)
    swing = lambda part: (max(part) - min(part)) / 2
    early = swing([r[column] for r in rows[:third]])
    late_rows = [r[column] for r in rows[-third:]]
    return statistics.median(late_rows), swing(late_rows), early


def rotate(quat, vector):
    """Turn a body-frame vector into the world frame. q v q*, written out."""
    w, x, y, z = quat
    u = (x, y, z)
    t = [2.0 * c for c in cross(u, vector)]
    return [vector[i] + w * t[i] + cross(u, t)[i] for i in range(3)]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def trim_attitude(aero_model, settle_s):
    """The attitude the hull settles into with nothing applied.

    Not always level. A vehicle whose center of mass is not directly under its
    center of volume hangs at an angle, and the Cloudship's is 22 mm forward of
    it, which is 2.6 degrees of nose down before anything pushes it. Measured
    once rather than per case: every coefficient here is quadratic in a
    velocity, so at rest they are all zero and the trim is the same whichever
    variant is loaded.
    """
    rows = settle(aero_model, True, None, settle_s)

    if not rows or len(rows) < 6:
        return None

    # measure() is the one definition of "the settled window" in this file;
    # the medians used to be taken here over a third computed separately, so
    # narrowing one narrowed only one.
    return [measure(rows, i)[0] for i in (1, 2)] + [rows[-1][3]]


def measured_stiffness(aero_model, trim, couple, settle_s, axis='y'):
    """Restoring stiffness from a pure couple, in N m per unit of tilt.

    Measured rather than taken from mass * g * BG, and not because that is
    wrong: it is the stiffness of a hull that has finished settling, and none
    of these have. They are pendulums damped by a quadratic term, which at
    these amplitudes is very weak, so the couple divided by the tilt reads
    large. The big airship comes out 0.2% high, the Cloudship 3.0%, which is
    the ratio of their damping and not of their geometry.

    Measured ONCE PER AXIS, because that under-settling bias only cancels
    against a case damped the same way. It was measured on pitch and reused
    for roll, and the two axes are not damped alike: the Cloudship carries kP
    and kPabsP on roll but only mQabsQ on pitch, so roll is six time constants
    down at 180 s where pitch is three. One extra gz run removes the
    assumption; the derived number is still reported, because the size of the
    gap says how far from settled that axis is.
    """
    column = 1 if axis == 'y' else 2
    rows = settle(aero_model, True, 'torque: {%s: %.4f}' % (axis, couple), settle_s)

    if not rows or len(rows) < 6:
        return None

    # A pitch couple tilts the nose (column 1); a roll couple tilts the port
    # axis (column 2), and with the opposite sign, because +x tips down for a
    # positive y-couple while +y tips UP for a positive x-couple.
    sign = -1.0 if axis == 'y' else 1.0
    tilt = measure(rows, column)[0] - trim[column - 1]
    return None if abs(tilt) < 1e-6 else couple / (sign * tilt)


def simulate(aero_model, geom, settle_s):
    com_x, com, hull, derived = geom
    trim = trim_attitude(aero_model, settle_s)

    if trim is None:
        # A recorded verdict, not a bare return: an unrecorded early exit
        # prints "통과 N / 실패 0" for a run that measured nothing.
        verdict('기준 자세를 읽는다', False, 'gz 에서 트림 자세를 읽지 못했다')
        return

    note('기준      기수 z %+.6f, 좌현 z %+.6f (밀기 전의 기체 자체 트림)'
         % (trim[0], trim[1]))

    # A couple big enough to move the hull well past its own noise and small
    # enough to stay in the same small-angle regime as the pushes below. Once
    # per axis: the under-settling bias only cancels against a case damped the
    # same way, and these two axes are not.
    stiffness = {}

    for axis, torque in (('피치', 'y'), ('롤', 'x')):
        stiffness[axis] = measured_stiffness(aero_model, trim, 0.02 * derived,
                                             settle_s, axis=torque)

        if stiffness[axis] is None:
            verdict('%s 축 강성을 잰다' % axis, False, '순수 커플로 재지 못했다')
            return

        note('강성 %s   측정 %.2f N m/rad, 기하에서 유도 %.2f (%+.1f%%)'
             % (axis, stiffness[axis], derived,
                100.0 * (stiffness[axis] / derived - 1.0)))

    # Where the center of mass is, in the frame the wrench is applied in.
    # ApplyLinkWrench takes world coordinates, and a hull with a trim is not
    # sitting square to them: the Cloudship hangs 2.6 degrees nose down, which
    # moves its center of mass 18 mm forward in world terms. Compensating with
    # the body-frame offset leaves 0.029 N m of yaw, and with nothing to
    # weathercock against it that is 4 deg/s, 780 degrees over the window, so
    # the world-frame push becomes a body-frame anything and the roll reading
    # is of nothing in particular.
    arm = rotate(trim[2], (com_x, 0.0, com))

    for axis, column, direction in (('피치', 1, (1.0, 0.0, 0.0)),
                                    ('롤', 2, (0.0, 1.0, 0.0))):
        # The push that lands THIS axis on the target tilt against the damping
        # it actually has. A fixed force tilts a 7 kg hull six times as far as
        # a 49 kg one, and far enough that the settled angle stops being the
        # quantity this predicts, so the force is derived rather than given -
        # every tolerance below is written for that regime.
        axis_force = TARGET_TILT * stiffness[axis] / (hull - com)
        note('밀기 %s   %.3f N (정착 기울기 %.4f 를 겨냥)'
             % (axis, axis_force, TARGET_TILT))
        # A force at the origin plus (arm x F) is the same force through the
        # center of mass, so the only attitude left to settle is the drag's.
        # All three components of that cross product, not just the one on the
        # axis under test.
        push = [d * axis_force for d in direction]
        couple = cross(arm, push)
        wrench = ('force: {x: %.5f, y: %.5f, z: %.5f}, '
                  'torque: {x: %.5f, y: %.5f, z: %.5f}' % (*push, *couple))
        kept = None
        for keep, label in ((True, '교차항 있음'), (False, '교차항 없음')):
            want = ((hull - com) if keep else -com) * axis_force / stiffness[axis]
            rows = settle(aero_model, keep, wrench, settle_s)

            if not rows or len(rows) < 6:
                verdict('%s %s' % (axis, label), False, '자세를 읽지 못했다')
                continue
            got, swing, early = measure(rows, column)
            got -= trim[column - 1]

            # Three things, because a swing that is still there is not the
            # same as one that is not going away. A hull damped only by a
            # quadratic term barely damps at these amplitudes, and the
            # Cloudship's pitch axis has no linear term at all, so it is still
            # visibly ringing at 180 s around a settled value good to 1%. What
            # must fail is a swing that is growing, which is a plant feeding
            # itself, and one that is holding steady, which is a limit cycle.
            # "Growing" has to mean growing to a size that matters against
            # the quantity being measured, not against a fixed 2e-5. The
            # Cloudship's roll mode is still ringing at 1.5e-4 after 180 s -
            # 0.7% of the tilt under test - and which end of the window
            # happens to be larger there is luck, so the flat floor turned a
            # settled case into "커지는 중" and took the stripped case's
            # comparison down with it.
            growing = swing > max(early * 1.3, 0.02 * abs(want), 2e-5)
            settled = swing <= max(1e-4, abs(want) * 0.05)
            decaying = swing <= early * 0.9
            good = (abs(got - want) <= max(2e-4, abs(want) * 0.05)
                    and not growing and (settled or decaying))

            # The stripped variant is a control, and what it has to show is
            # that the cross terms matter, not that a particular number comes
            # out. Two ways it can show that, and both are findings.
            #
            # It settles on the bare lever. That is the pitch axis, and the
            # number is the sharper statement, so it stays the requirement:
            # there the stripped swing is only twice the kept one.
            #
            # Or it does not settle where the kept variant settled. That is
            # the Cloudship's roll axis, where the cross terms carry the
            # rate damping: the same wrench with them settles to 1.5e-4 and
            # without them is still swinging 42 times wider at 180 s, and
            # runs out to 450 and 900 s have it *growing*, 0.0118 -> 0.0319.
            # There is no equilibrium there to compare a lever against, and
            # demanding one asks the plant for something it does not have.
            #
            # Only ever accepted against a kept variant that did settle, on
            # the same vehicle under the same wrench, so it is a controlled
            # comparison and not an excuse. Ten times is far outside the
            # factor of two the pitch axis shows.
            # An absolute floor as well as the ratio. A ratio alone is
            # meaningless at the noise floor, and on the big airship both
            # variants settle so hard that both swings print 0.000000 - there
            # the escape would arm on float noise and delete the lever
            # assertion, and `swing / kept[1]` would divide by zero on the way
            # to saying so. `growing` above carries a 2e-5 floor for exactly
            # this reason; this needs one too.
            diverged = (not keep and kept is not None and kept[0]
                        and not settled
                        and swing > 10.0 * kept[1]
                        and swing > max(1e-3, abs(want) * 0.5))
            if diverged:
                good = True

            verdict('%s %s' % (axis, label), good,
                    '기울기(트림 제외) %+.6f  %s  (%+.3f deg, %s) 진폭 %.6f -> %.6f (%s)'
                    % (got,
                       '교차항 있음 대비 %.0f 배로 안 멎는다'
                       % (swing / kept[1] if kept[1] > 0 else float('inf')) if diverged
                       else '기대 %+.6f' % want,
                       math.degrees(math.asin(max(-1.0, min(1.0, got)))),
                       '앞/좌현 올림' if got > 0 else '앞/좌현 내림', early, swing,
                       '커지는 중' if growing else '정착' if settled
                       else '줄어드는 중' if decaying else '유지'))

            if keep:
                kept = (good and settled, swing)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--vehicle', choices=sorted(VEHICLES), default='airship',
                    help='검사할 기체')
    ap.add_argument('--analytic', action='store_true', help='gz 를 띄우지 않는다')
    ap.add_argument('--settle', type=float, default=180.0, help='정착 대기 [s]')
    args = ap.parse_args()

    models = VEHICLES[args.vehicle]
    geom = analytic(models)

    if not args.analytic:
        # A missing gz, or a missing plugin directory, used to degrade the run
        # to the analytic half and still exit 0. Half a check reporting a pass
        # is how a queue records a green run it never did: --analytic is the
        # way to ask for half, and asking for the whole thing and not getting
        # it is a failure.
        if shutil.which('gz') is None:
            verdict('gz 를 띄울 수 있다', False,
                    'gz 가 PATH 에 없다. --analytic 이 아니면 절반만 돌 수 없다')
        elif not os.path.isdir(GZ_PLUGINS):
            verdict('gz 를 띄울 수 있다', False,
                    'gz 플러그인 디렉터리가 없다: %s' % GZ_PLUGINS)
        else:
            simulate(models[-1], geom, args.settle)

    return summary()


if __name__ == '__main__':
    sys.exit(main())
