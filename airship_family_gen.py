#!/usr/bin/env python3
"""Generate an airship's gz model family, world entries, posix airframes and
docs entries from one vehicle file.

    python3 airship_family_gen.py vehicles/airship_indoor.json          # write into the PX4 tree (PX4_ROOT)
    python3 airship_family_gen.py vehicles/airship_generic.json --check # regenerate the generic family in a
                                                                        # scratch copy and diff it against the tree

The vehicle file carries the hull (generic_hull_design.HullSpec), the pods
and the names: see vehicles/airship_generic.json, which describes the
committed generic family, and vehicles/airship_indoor.json. The family is
made from the generic family's committed files as templates, renamed, and
then written by apply_generic_hull's writers, so a new vehicle gets the same
eight models, derivations and comments as the generic airship with its own
numbers. A rerun writes the same files again. What the tree carries besides
the models is appended once: the lta world's buoyancy entries, the posix
airframes of the vehicle's decade (the bare one with the propulsion block,
the members as includes; an existing member keeps its gains), the posix
CMakeLists entries and decade line, and the docs entries (the vehicle list,
its table and the airship page's simulation list).
"""
import argparse
import dataclasses
import json
import math
import os
import re
import shutil
import sys
import tempfile

import apply_generic_hull as A
import generic_hull_design as g
import gz_model

ROLES = ('base', 'base_am', 'bare', 'drag', 'fin', 'am', 'am_fin', 'am_fin_drag')
# the scheme's suffix, airframe mask and docs name of each member
MEMBERS = (('bare', 0, ''), ('drag', 1, 'Hull Drag'), ('fin', 2, 'Fins'), ('am', 4, 'Added Mass'),
           ('am_fin', 6, 'Added Mass, Fins'), ('am_fin_drag', 7, 'Complete'))
MOMENT_CONSTANT = 0.016     # the x500 motor block's, which the wrappers keep


@dataclasses.dataclass(frozen=True)
class Pods:
    link_mass: float
    prop_mass: float
    thrust_n: float             # per pod at max_rot_velocity
    max_rot_velocity: float     # rad/s, SIM_GZ_EC_MAX
    tilt_deg: float             # +-tilt: the joint limits, SIM_GZ_SV_MINA/MAXA and CA_AIRSHIP_TLMIN/TLMAX
    tilt_rate_deg_s: float      # CA_AIRSHIP_TLT_R
    # radii of gyration of the pod link and the prop, m. Left out, they are
    # the generic pods' 0.05 and 0.1 scaled by the hull's length over the
    # 10.97 m reference: a smaller hull's pods are smaller
    gyration_m: float = None
    prop_gyration_m: float = None

    @property
    def motor_constant(self):
        return self.thrust_n / self.max_rot_velocity ** 2


@dataclasses.dataclass(frozen=True)
class Vehicle:
    name: str
    title: str          # the posix @name and docs heading of the bare member
    body_title: str     # how the base body's header names it ('Generic airship')
    decade: int
    hull: g.HullSpec
    pods: Pods
    path: str
    docs_note: str = None   # a sentence the bare member's docs section ends with: the vehicle's source
    surfaces: bool = False  # flaps on the fin members' fins: rudders and elevators beside the pods

    @property
    def family(self):
        if self.name == A.GENERIC.bare:
            return A.GENERIC
        return A.vectored_family(self.name, self.body_title, self.surfaces)

    def member_title(self, docs_name):
        if not docs_name:
            return self.title
        if self.title.endswith(')'):
            return self.title[:-1] + ', ' + docs_name + ')'
        return '%s (%s)' % (self.title, docs_name)


def load_vehicle(path):
    with open(path) as handle:
        v = json.load(handle)
    return Vehicle(v['name'], v['title'], v['body_title'], int(v['decade']), g.HullSpec.from_dict(v['hull']),
                   Pods(**v['pods']), os.path.abspath(path), v.get('docs_note'), bool(v.get('surfaces', False)))


def slug(title):
    return re.sub(r'[^a-z0-9 ]', '', title.lower()).strip().replace(' ', '-')


def read(path):
    with open(path, encoding='utf-8') as handle:
        return handle.read()


def write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write(text)


def sub1(t, pattern, repl, count=1):
    new, k = re.subn(pattern, repl, t, flags=re.S)
    assert k == count, (pattern[:70], k, count)
    return new


def fmt(x):
    return '%g' % x


# ---------------------------------------------------------------------------
# The models
# ---------------------------------------------------------------------------

def copy_family(src_fam, dst_fam, src_root, dst_root):
    """The destination family's eight directories from the source's files,
    every model name and the body's title renamed. A destination that is the
    source (the generic family in place) copies nothing."""
    renames = sorted(((getattr(src_fam, r), getattr(dst_fam, r)) for r in ROLES), key=lambda kv: -len(kv[0]))
    renames += [(src_fam.title, dst_fam.title), (src_fam.title.lower(), dst_fam.title.lower())]

    for role in ROLES:
        src, dst = os.path.join(src_root, getattr(src_fam, role)), os.path.join(dst_root, getattr(dst_fam, role))

        if os.path.abspath(src) == os.path.abspath(dst):
            continue

        for name in ('model.sdf', 'model.config'):
            t = read(os.path.join(src, name))

            for old, new in renames:
                t = t.replace(old, new)

            write(os.path.join(dst, name), t)


def write_pods(p, vehicle):
    """The propulsion block of the bare and _am wrappers: the pods' station,
    masses and tilt limits, the motors' constants, and the numbers the two
    comments restate. Identical in both, as variant_check requires."""
    s, pods, fam = p.s, vehicle.pods, p.fam
    scale = (2.0 * s.semi_axes[0]) / g.SCALE_REFERENCE_LENGTH
    r_pod = pods.gyration_m if pods.gyration_m is not None else 0.05 * scale
    r_prop = pods.prop_gyration_m if pods.prop_gyration_m is not None else 0.1 * scale
    pod_inertia = pods.link_mass * r_pod ** 2       # the generic pods' 0.001 at their 0.4 kg
    prop_inertia = pods.prop_mass * r_prop ** 2     # and their props' 0.001 at 0.1 kg
    # the tilt servo's position loop and the joint's damping keep the generic
    # pods' bandwidth and damping ratio: all three scale with the pod's
    # inertia (the generic 2.0, 0.1 and 0.01 on 0.001). Left unscaled, the
    # 0.01 N m s/rad damping held a 10 g pod to 30 deg/s against the 120 deg/s
    # the allocator commands (2026-10-07, the indoor hover gate)
    servo_scale = pod_inertia / 0.001
    lim = math.radians(pods.tilt_deg)
    couple = MOMENT_CONSTANT * pods.thrust_n
    stiffness = p.d['stiffness']

    for rel in (fam.bare + '/model.sdf', fam.am + '/model.sdf'):
        t = A.load(p, rel)

        for side, sign in (('starboard', -1.0), ('port', 1.0)):
            pod, prop = '<link name="pod_%s">' % side, '<link name="prop_%s">' % side
            t = A.in_elem(t, pod, '</link>', r'(<pose>)[^<]*(</pose>)', r'\g<1>0 %s %s 0 0 0\g<2>' % (fmt(sign * s.pod_y), fmt(s.pod_z)))
            t = A.in_elem(t, pod, '</link>', r'(<mass>)[^<]*(</mass>)', r'\g<1>%s\g<2>' % fmt(pods.link_mass))
            t = A.in_elem(t, prop, '</link>', r'(<pose degrees="true">)[^<]*(</pose>)',
                          r'\g<1>0 %s %s 0 90 0\g<2>' % (fmt(sign * s.pod_y), fmt(s.pod_z)))
            t = A.in_elem(t, prop, '</link>', r'(<mass>)[^<]*(</mass>)', r'\g<1>%s\g<2>' % fmt(pods.prop_mass))

            for tag in ('ixx', 'iyy', 'izz'):
                t = A.in_elem(t, pod, '</link>', r'(<%s>)[^<]*(</%s>)' % (tag, tag), r'\g<1>%s\g<2>' % fmt(pod_inertia))
                t = A.in_elem(t, prop, '</link>', r'(<%s>)[^<]*(</%s>)' % (tag, tag), r'\g<1>%s\g<2>' % fmt(prop_inertia))

            t = A.in_elem(t, '<joint name="tilt_%s_joint"' % side, '</joint>', r'(<lower>)[^<]*(</lower>\s*<upper>)[^<]*(</upper>)',
                          r'\g<1>%.6f\g<2>%.6f\g<3>' % (-lim, lim))
            t = A.in_elem(t, '<joint name="tilt_%s_joint"' % side, '</joint>', r'(<damping>)[^<]*(</damping>)',
                          r'\g<1>%s\g<2>' % fmt(0.01 * servo_scale))

        t = sub1(t, r'<p_gain>[^<]*</p_gain>', '<p_gain>%s</p_gain>' % fmt(2.0 * servo_scale), count=2)
        t = sub1(t, r'<d_gain>[^<]*</d_gain>', '<d_gain>%s</d_gain>' % fmt(0.1 * servo_scale), count=2)
        t = sub1(t, r'<maxRotVelocity>[^<]*</maxRotVelocity>', '<maxRotVelocity>%.1f</maxRotVelocity>' % pods.max_rot_velocity, count=2)
        t = sub1(t, r'<motorConstant>[^<]*</motorConstant>', '<motorConstant>%s</motorConstant>' % fmt(pods.motor_constant), count=2)
        t = sub1(t, A.ws(r"maxRotVelocity 1000 -> [\d.]+ \(the airframe's SIM_GZ_EC_MAX1/2\) and motorConstant 8\.54858e-06 -> [\deE.+-]+, so")
                 + r'\s+' + A.ws(r"[\deE.+-]+ \* [\d.]+\^2 = [\d.]+ N per pod, [\d.]+ N on a [\d.]+ kg vehicle\."),
                 "maxRotVelocity 1000 -> %s (the airframe's SIM_GZ_EC_MAX1/2) and motorConstant 8.54858e-06 -> %s, so\n"
                 "         %s * %s^2 = %s N per pod, %s N on a %s kg vehicle."
                 % (fmt(pods.max_rot_velocity), fmt(pods.motor_constant), fmt(pods.motor_constant), fmt(pods.max_rot_velocity),
                    fmt(pods.thrust_n), fmt(2 * pods.thrust_n), fmt(round(p.d['total'], 3))))
        t = sub1(t, A.ws(r"The reaction couple is [\d.]+ N m about each prop's own thrust axis"),
                 "The reaction couple is %.2f N m about each prop's own thrust axis" % couple)
        t = sub1(t, A.ws(r"full-differential residual is [\d.]+ N m against [\d.]+ N m/rad of roll stiffness, [\d.]+ deg\."),
                 "full-differential residual is %.2f N m against %.0f N m/rad of roll stiffness, %.2f deg."
                 % (couple, stiffness, math.degrees(couple / stiffness)))

        if abs(pods.tilt_deg - 180.0) > 1e-9:
            t = t.replace('The tilt joints stop at +-pi', 'The tilt joints stop at +-%s deg' % fmt(pods.tilt_deg))

        t = A.refill(t, "x500's motor block with two numbers changed", 78)
        A.save(p, rel, t)


FLAP_LINKS_MASS = 0.004     # the fixed twin's four massless-drawn flap links at 0.001 kg each


def write_fixed_thrusters(p, vehicle):
    """The fixed-thruster twin's propulsion (a family with one: the generic,
    airship_fixed and its _am wrapper): the vectored pods' stations, masses
    and motors with the tilt joints replaced by mounts. Its thruster links
    carry the pods' mass less the four flap links', so the twin weighs what
    the vectored airship does, and the comments restate the numbers."""
    s, pods, fam = p.s, vehicle.pods, p.fam

    if not fam.fixed:
        return

    scale = (2.0 * s.semi_axes[0]) / g.SCALE_REFERENCE_LENGTH
    r_pod = pods.gyration_m if pods.gyration_m is not None else 0.05 * scale
    r_prop = pods.prop_gyration_m if pods.prop_gyration_m is not None else 0.1 * scale
    link = pods.link_mass - FLAP_LINKS_MASS / 2.0
    total, v_g = p.d['total'], s.gondola_volume
    couple = MOMENT_CONSTANT * pods.thrust_n

    for rel in (m + '/model.sdf' for m in fam.fixed):
        t = A.load(p, rel)

        for side, sign in (('starboard', -1.0), ('port', 1.0)):
            pod, prop = '<link name="pod_%s">' % side, '<link name="prop_%s">' % side
            t = A.in_elem(t, pod, '</link>', r'(<pose>)[^<]*(</pose>)', r'\g<1>0 %s %s 0 0 0\g<2>' % (fmt(sign * s.pod_y), fmt(s.pod_z)))
            t = A.in_elem(t, pod, '</link>', r'(<mass>)[^<]*(</mass>)', r'\g<1>%s\g<2>' % fmt(round(link, 6)))
            t = A.in_elem(t, prop, '</link>', r'(<pose degrees="true">)[^<]*(</pose>)',
                          r'\g<1>0 %s %s 0 90 0\g<2>' % (fmt(sign * s.pod_y), fmt(s.pod_z)))
            t = A.in_elem(t, prop, '</link>', r'(<mass>)[^<]*(</mass>)', r'\g<1>%s\g<2>' % fmt(pods.prop_mass))

            for tag in ('ixx', 'iyy', 'izz'):
                t = A.in_elem(t, pod, '</link>', r'(<%s>)[^<]*(</%s>)' % (tag, tag), r'\g<1>%s\g<2>' % fmt(pods.link_mass * r_pod ** 2))
                t = A.in_elem(t, prop, '</link>', r'(<%s>)[^<]*(</%s>)' % (tag, tag), r'\g<1>%s\g<2>' % fmt(pods.prop_mass * r_prop ** 2))

        t = sub1(t, r'<maxRotVelocity>[^<]*</maxRotVelocity>', '<maxRotVelocity>%.1f</maxRotVelocity>' % pods.max_rot_velocity, count=2)
        t = sub1(t, r'<motorConstant>[^<]*</motorConstant>', '<motorConstant>%s</motorConstant>' % fmt(pods.motor_constant), count=2)
        t = sub1(t, A.ws(r"the collision volumes' [\d.]+ m\^3 \(hull [\d.]+, gondola [\d.]+\) lift the family's [\d.]+ kg, and without them "
                         r"the vehicle is [\d.]+ kg light and rises at [\d.]+ m/s\^2"),
                 "the collision volumes' %.3g m^3 (hull %.4g, gondola %.3g) lift the family's %.1f kg, and without them the vehicle is "
                 "%.3g kg light and rises at %.2g m/s^2" % (s.hull_volume + v_g, s.hull_volume, v_g, total, 2 * link, 2 * link * g.G / total))
        t = sub1(t, A.ws(r"[\d.]+ kg each, so that with the four flap links' 0\.001 kg the total is the vectored airship's [\d.]+ kg\."),
                 "%s kg each, so that with the four flap links' 0.001 kg the total is the vectored airship's %.1f kg." % (fmt(round(link, 6)), total))
        t = sub1(t, A.ws(r"thrust axis, points forward\. [\d.]+ N each at [\d.]+ rad/s\."),
                 "thrust axis, points forward. %s N each at %s rad/s." % (fmt(pods.thrust_n), fmt(pods.max_rot_velocity)))
        t = sub1(t, A.ws(r"maxRotVelocity 1000 -> [\d.]+ \(the airframe's SIM_GZ_EC_MAX1/2\) and motorConstant 8\.54858e-06 -> [\deE.+-]+, so")
                 + r'\s+' + A.ws(r"[\deE.+-]+ \* [\d.]+\^2 = [\d.]+ N per pod, [\d.]+ N on a [\d.]+ kg vehicle\."),
                 "maxRotVelocity 1000 -> %s (the airframe's SIM_GZ_EC_MAX1/2) and motorConstant 8.54858e-06 -> %s, so "
                 "%s * %s^2 = %s N per pod, %s N on a %s kg vehicle."
                 % (fmt(pods.max_rot_velocity), fmt(pods.motor_constant), fmt(pods.motor_constant), fmt(pods.max_rot_velocity),
                    fmt(pods.thrust_n), fmt(2 * pods.thrust_n), fmt(round(total, 3))))
        t = sub1(t, A.ws(r"The reaction couple is [\d.]+ N m about each"), "The reaction couple is %.2f N m about each" % couple)
        t = A.refill(t, 'Two fixed forward thrusters at the pod stations', 78)
        t = A.refill(t, "x500's motor block with two numbers changed", 78)
        A.save(p, rel, t)


FLAPS_OPEN = '    <!-- The four flaps:'
FINS_OPEN = '    <!-- The four fins as lifting surfaces'
SURFACE_TOPIC_OFFSET = 2    # the flaps' servo topics follow the pods' two tilt servos


def inject_flaps(fam, root):
    """A surfaces family's fin members get the fixed twin's four flaps: its
    flap comment, links, joints and position controllers (servo topics moved
    past the pods' two), and each lift-drag plugin its flap joint, all from
    the generic fixed twin as a template. write_fins then sizes them. A fin
    member that already has them is left as it is."""
    template = read(os.path.join(gz_model.GZ_MODELS, A.GENERIC.fixed[0], 'model.sdf'))
    block = template[template.index(FLAPS_OPEN):template.index('    <!-- The four fins as lifting surfaces WITH')]
    block = re.sub(r'<sub_topic>servo_(\d)</sub_topic>', lambda m: '<sub_topic>servo_%d</sub_topic>' % (int(m.group(1)) + SURFACE_TOPIC_OFFSET), block)
    hooks = {re.search(r'<!-- fin_(\w+) -->', b).group(1): re.search(r'\s*<control_joint_name>.*?</control_joint_rad_to_cl>', b, re.S).group(0)
             for b in re.findall(r'<plugin filename="gz-sim-lift-drag-system".*?</plugin>', template, flags=re.S)}

    for member in fam.fin_members:
        path = os.path.join(root, member, 'model.sdf')
        t = read(path)

        if FLAPS_OPEN in t:
            continue

        t = t.replace(FINS_OPEN, block + FINS_OPEN, 1)

        for b in re.findall(r'<plugin filename="gz-sim-lift-drag-system".*?</plugin>', t, flags=re.S):
            name = re.search(r'<!-- fin_(\w+) -->', b).group(1)
            t = t.replace(b, b.replace('\n    </plugin>', hooks[name] + '\n    </plugin>', 1), 1)

        write(path, t)


def write_models(vehicle, root):
    """The whole family under root: the generic files as templates, then the
    design, then the pods."""
    fam = vehicle.family
    copy_family(A.GENERIC, fam, gz_model.GZ_MODELS, root)

    if fam.surfaces:
        inject_flaps(fam, root)

    p = A.plan(vehicle.hull, fam, root)
    A.write_family(p)
    write_pods(p, vehicle)
    write_fixed_thrusters(p, vehicle)
    return p


# ---------------------------------------------------------------------------
# The world, the airframes, the docs
# ---------------------------------------------------------------------------

def write_world(path, fam):
    """The buoyancy entries of every member, bare and _0.._3, after the last
    one present; those already there are left."""
    t = read(path)
    lines = []

    for role, _, _ in MEMBERS:
        for instance in ('',) + tuple('_%d' % i for i in range(4)):
            line = '      <enable>%s%s::base_link</enable>' % (getattr(fam, role), instance)

            if line not in t:
                lines.append(line)

    if lines:
        end = t.index('\n', t.rindex('</enable>')) + 1
        t = t[:end] + '\n'.join(lines) + '\n' + t[end:]
        write(path, t)

    return len(lines)


def airframe_path(airframes, number, model):
    return os.path.join(airframes, '%d_gz_%s' % (number, model))


def write_airframes(vehicle, p, airframes):
    """The posix airframes of the decade. The bare one is the generic bare
    airframe with this vehicle's names and propulsion numbers; a member is a
    header, its PX4_SIM_MODEL and the include of the bare one. A member that
    exists keeps its text: the gains written below its include are the
    calculator's, not this generator's."""
    fam, pods = vehicle.family, vehicle.pods
    bare_path = airframe_path(airframes, vehicle.decade, fam.bare)
    # an existing bare airframe is updated in place, so the block the
    # calculator appends below its propulsion lines survives a regeneration
    t = read(bare_path) if os.path.exists(bare_path) else read(airframe_path(airframes, 2520, A.GENERIC.bare)).replace(A.GENERIC.bare, fam.bare)
    t = sub1(t, r'(# @name )[^\n]*', r'\g<1>%s' % vehicle.title.replace('\\', '\\\\'))
    t = sub1(t, r'rotor speed 0\.\.[\d.]+ rad/s', 'rotor speed 0..%s rad/s' % fmt(pods.max_rot_velocity))

    for n in (1, 2):
        t = sub1(t, r'(param set-default SIM_GZ_EC_MAX%d )[\d.-]+' % n, r'\g<1>%s' % fmt(pods.max_rot_velocity))
        t = sub1(t, r'(param set-default SIM_GZ_SV_MINA%d )[\d.-]+' % n, r'\g<1>%s' % fmt(-pods.tilt_deg))
        t = sub1(t, r'(param set-default SIM_GZ_SV_MAXA%d )[\d.-]+' % n, r'\g<1>%s' % fmt(pods.tilt_deg))

    t = sub1(t, r'(param set-default CA_AIRSHIP_TLMIN )[\d.-]+', r'\g<1>%s' % fmt(-pods.tilt_deg))
    t = sub1(t, r'(param set-default CA_AIRSHIP_TLMAX )[\d.-]+', r'\g<1>%s' % fmt(pods.tilt_deg))
    t = sub1(t, r'(param set-default CA_AIRSHIP_TLT_R )[\d.-]+', r'\g<1>%s' % fmt(pods.tilt_rate_deg_s))
    written = []

    if not os.path.exists(bare_path) or read(bare_path) != t:
        write(bare_path, t)
        written.append(os.path.basename(bare_path))

    for role, mask, docs_name in MEMBERS[1:]:
        path = airframe_path(airframes, vehicle.decade + mask, getattr(fam, role))

        if os.path.exists(path):
            continue

        write(path, '''#!/bin/sh
#
# @name %s
#
# @type Airship
# @class Airship
#
# @maintainer Hyunduk Shin <hyunduk@icarus-airship.com>
#

PX4_SIM_MODEL=${PX4_SIM_MODEL:=%s}

. ${R}etc/init.d-posix/airframes/%s
''' % (vehicle.member_title(docs_name), getattr(fam, role), os.path.basename(bare_path)))
        written.append(os.path.basename(path))

    return written


SURFACES_OPEN = '# --- control surfaces from airship_family_gen.py (regenerate, do not edit) ---'
SURFACES_CLOSE = '# --- end of control surfaces ---'
SURFACE_MEMBERS = (('fin', 2), ('am_fin', 6), ('am_fin_drag', 7))
SURFACE_OUTPUTS = ('# @output Motor1 starboard pod motor\n# @output Motor2 port pod motor\n# @output Servo1 right elevator\n'
                   '# @output Servo2 left elevator\n# @output Servo3 upper rudder\n# @output Servo4 lower rudder\n'
                   '# @output Servo5 starboard pod tilt\n# @output Servo6 port pod tilt\n')


def surfaces_block(p):
    lines = ['# The fins\' flaps as the 2500 airframe declares them, two elevators and two',
             '# rudders, which the allocator puts before the tilt servos (Servo1 to Servo4);',
             '# the pods\' tilt servos move to Servo5 and Servo6. The model\'s servo_0 and',
             '# servo_1 are the tilt joints, servo_2 to servo_5 the flaps, and the bridge maps',
             '# the +-1 surface command onto their +-%d deg. With yaw surfaces declared the' % p.travel,
             '# allocator reports them and the forward-speed schedule stays off',
             'param set-default CA_SV_CS_COUNT 4',
             'param set-default CA_SV_CS0_TYPE 3', 'param set-default CA_SV_CS0_TRQ_P 0.5',
             'param set-default CA_SV_CS1_TYPE 3', 'param set-default CA_SV_CS1_TRQ_P 0.5',
             'param set-default CA_SV_CS2_TYPE 4', 'param set-default CA_SV_CS2_TRQ_Y 0.5',
             'param set-default CA_SV_CS3_TYPE 4', 'param set-default CA_SV_CS3_TRQ_Y 0.5',
             'param set-default SIM_GZ_SV_FUNC1 205', 'param set-default SIM_GZ_SV_FUNC2 206']

    for n in range(3, 7):
        lines += ['param set-default SIM_GZ_SV_FUNC%d %d' % (n, 198 + n), 'param set-default SIM_GZ_SV_MINA%d %d' % (n, -p.travel),
                  'param set-default SIM_GZ_SV_MAXA%d %d' % (n, p.travel)]

    return '\n'.join([SURFACES_OPEN] + lines + [SURFACES_CLOSE])


def write_surface_airframes(vehicle, p, airframes):
    """A surfaces family's fin members: the surfaces block right after the
    include of the bare airframe (its tilt servo mapping is overridden), and
    the members' own @output lines. Updated in place."""
    fam = vehicle.family

    if not fam.surfaces:
        return []

    written = []
    block = surfaces_block(p)

    for role, mask in SURFACE_MEMBERS:
        path = airframe_path(airframes, vehicle.decade + mask, getattr(fam, role))
        t = read(path)
        new = t

        if '# @output' not in new:
            new = new.replace('# @maintainer', SURFACE_OUTPUTS + '#\n# @maintainer', 1)

        if SURFACES_OPEN in new:
            new = re.sub(re.escape(SURFACES_OPEN) + r'.*?' + re.escape(SURFACES_CLOSE), lambda m: block, new, count=1, flags=re.S)
        else:
            include = re.search(r'\. \$\{R\}etc/init\.d-posix/airframes/\S+\n', new)
            new = new[:include.end()] + '\n' + block + '\n' + new[include.end():]

        if new != t:
            write(path, new)
            written.append(os.path.basename(path))

    return written


def write_cmake(vehicle, airframes):
    """The decade line in the table comment and the six entries, in number order."""
    path = os.path.join(airframes, 'CMakeLists.txt')
    t = read(path)
    fam = vehicle.family
    changed = False
    decade_line = '\t#   %d %s' % (vehicle.decade, fam.bare)

    if decade_line not in t and not re.search(r'\t#   %d ' % vehicle.decade, t):
        # after the last decade line of the table
        last = list(re.finditer(r'\t#   \d{4} [^\n]*\n', t))[-1]
        t = t[:last.end()] + decade_line + '\n' + t[last.end():]
        changed = True

    for role, mask, _ in MEMBERS:
        entry = '\t%d_gz_%s' % (vehicle.decade + mask, getattr(fam, role))

        if entry + '\n' in t:
            continue

        # before the first 25xx or later entry with a larger number, else after the last 25xx one
        numbers = [(int(m.group(1)), m) for m in re.finditer(r'\t(\d{4})_[^\n]*\n', t)]
        after = [m for n, m in numbers if n > vehicle.decade + mask]
        at = after[0].start() if after else max(m.end() for n, m in numbers if n < vehicle.decade + mask)
        t = t[:at] + entry + '\n' + t[at:]
        changed = True

    if changed:
        write(path, t)

    return changed


def docs_sections(vehicle, p):
    fam = vehicle.family
    a, b = p.a, p.b
    intro = {
        'bare': '[%s](../frames_airship/index.md): two independently tilting thrust pods on a %.2f x %.2f m hull, the rigid hull: '
                'a torque keeps accelerating it and nothing damps a rotation.' % (vehicle.title, 2 * a, 2 * b),
        'drag': 'The same vehicle with hull drag and rotational damping.',
        'fin': 'The same vehicle with its four fins as lifting surfaces, isolated from the other physics so that their contribution can be measured on its own.\n'
               'The plant they exist for is the Added Mass, Fins variant, where they have the Munk moment to oppose.',
        'am': "The same vehicle with the hull's fluid added mass and nothing to oppose the Munk moment it brings.\n"
              'It exists to measure that term on its own.',
        'am_fin': 'Added mass and fin lift together, the Munk moment and the one thing on this hull that opposes it, with no drag.\n'
                  'With no hull drag, the fins are also all that damps a rate here.',
        'am_fin_drag': 'Everything: added mass, fin lift, hull drag and rotational damping.\nThis is the plant to tune the loops against.',
    }
    out = []

    if vehicle.docs_note:
        intro['bare'] += '\n' + vehicle.docs_note

    for role, _, docs_name in MEMBERS:
        out.append('### %s\n\n%s\n\n```sh\nmake px4_sitl gz_%s\n```\n' % (vehicle.member_title(docs_name), intro[role], getattr(fam, role)))

    return out


def write_docs(vehicle, p, root):
    """The vehicle list's sections (before the Cloudship's), the simulation
    index's table rows (after the generic vectored family's) and the airship
    page's simulation bullet; each once."""
    fam = vehicle.family
    changed = []
    vehicles_md = os.path.join(root, 'docs/en/sim_gazebo_gz/vehicles.md')
    t = read(vehicles_md)

    if '### %s\n' % vehicle.title not in t:
        at = t.index('\n### Cloudship\n') + 1
        t = t[:at] + '\n'.join(docs_sections(vehicle, p)) + '\n' + t[at:]
        write(vehicles_md, t)
        changed.append('vehicles.md')

    index_md = os.path.join(root, 'docs/en/sim_gazebo_gz/index.md')
    t = read(index_md)
    generic_complete = [l for l in t.splitlines() if l.startswith('| [Generic Airship (Independent Vectoring, Complete)]')]
    assert len(generic_complete) == 1, 'the simulation index table has lost the generic vectored family'
    widths = [len(c) for c in generic_complete[0].split('|')[1:-1]]
    rows = []

    for role, mask, docs_name in MEMBERS:
        title = vehicle.member_title(docs_name)
        cells = [' [%s](../sim_gazebo_gz/vehicles.md#%s)' % (title, slug(title)), ' `make px4_sitl gz_%s`' % getattr(fam, role),
                 ' %d' % (vehicle.decade + mask)]
        row = '|' + '|'.join(c.ljust(w) for c, w in zip(cells, widths)) + '|'

        if row.split('|')[1].strip() not in t:
            rows.append(row)

    if rows:
        at = t.index(generic_complete[0]) + len(generic_complete[0]) + 1
        t = t[:at] + '\n'.join(rows) + '\n' + t[at:]
        write(index_md, t)
        changed.append('index.md')

    airship_md = os.path.join(root, 'docs/en/frames_airship/index.md')
    t = read(airship_md)
    anchor = '../sim_gazebo_gz/vehicles.md#%s' % slug(vehicle.title)

    if anchor + ')' not in t:
        links = ', '.join('[%s](../sim_gazebo_gz/vehicles.md#%s)' % (docs_name, slug(vehicle.member_title(docs_name)))
                          for _, _, docs_name in MEMBERS[1:-1])
        last = '[%s](../sim_gazebo_gz/vehicles.md#%s)' % ('Complete', slug(vehicle.member_title('Complete')))
        bullet = '- [%s](%s), with its %s and %s variants\n' % (vehicle.title, anchor, links, last)
        generic = [l for l in t.splitlines(True) if l.startswith('- [Generic Airship (Independent Vectoring)]')]
        assert len(generic) == 1, 'the airship page has lost its simulation list'
        at = t.index(generic[0]) + len(generic[0])
        t = t[:at] + bullet + t[at:]
        write(airship_md, t)
        changed.append('frames_airship/index.md')

    return changed


def write_fixed_servos(vehicle, p, airframes):
    """A family with a fixed-thruster twin (the generic one: airship_fixed):
    its bare posix airframe maps the bridge's +-1 onto the flap joints'
    +-travel, SIM_GZ_SV_MINA/MAXA1..4 and the comment that says so."""
    fam = vehicle.family

    if not fam.fixed:
        return []

    import glob
    paths = glob.glob(os.path.join(airframes, '[0-9]*_gz_%s' % fam.fixed[0]))
    assert len(paths) == 1, paths
    t = read(paths[0])
    new = sub1(t, r'(the gz joints stop at \+-)\d+( deg)', r'\g<1>%d\g<2>' % p.travel)

    for n in (1, 2, 3, 4):
        new = sub1(new, r'(param set-default SIM_GZ_SV_MINA%d )[\d.-]+' % n, r'\g<1>%d' % -p.travel)
        new = sub1(new, r'(param set-default SIM_GZ_SV_MAXA%d )[\d.-]+' % n, r'\g<1>%d' % p.travel)

    if new == t:
        return []

    write(paths[0], new)
    return [os.path.basename(paths[0])]


def generate(vehicle, root):
    models = os.path.join(root, 'Tools/simulation/gz/models')
    airframes = os.path.join(root, 'ROMFS/px4fmu_common/init.d-posix/airframes')
    p = write_models(vehicle, models)
    n = write_world(os.path.join(root, 'Tools/simulation/gz/worlds/lta.sdf'), vehicle.family)
    print('world: %d entries added' % n)
    print('airframes written:', ', '.join(write_airframes(vehicle, p, airframes) + write_fixed_servos(vehicle, p, airframes)
                                          + write_surface_airframes(vehicle, p, airframes)) or 'none')
    print('cmake changed:', write_cmake(vehicle, airframes))
    print('docs changed:', ', '.join(write_docs(vehicle, p, root)) or 'none')
    return p


def check(vehicle):
    """Generate into a scratch copy of the tree's relevant files and diff."""
    import subprocess
    scratch = tempfile.mkdtemp(prefix='airship_family_')
    root = gz_model.ROOT

    for rel in ('Tools/simulation/gz/models', 'ROMFS/px4fmu_common/init.d-posix/airframes'):
        shutil.copytree(os.path.join(root, rel), os.path.join(scratch, rel))

    for rel in ('Tools/simulation/gz/worlds/lta.sdf', 'docs/en/sim_gazebo_gz/index.md', 'docs/en/sim_gazebo_gz/vehicles.md',
                'docs/en/frames_airship/index.md'):
        os.makedirs(os.path.dirname(os.path.join(scratch, rel)), exist_ok=True)
        shutil.copy(os.path.join(root, rel), os.path.join(scratch, rel))

    generate(vehicle, scratch)
    changed = 0

    for rel in ('Tools/simulation/gz/models', 'ROMFS/px4fmu_common/init.d-posix/airframes', 'Tools/simulation/gz/worlds/lta.sdf',
                'docs/en/sim_gazebo_gz/index.md', 'docs/en/sim_gazebo_gz/vehicles.md', 'docs/en/frames_airship/index.md'):
        out = subprocess.run(['diff', '-r', '-u', os.path.join(root, rel), os.path.join(scratch, rel)], capture_output=True, text=True)

        if out.returncode:
            changed += 1
            print(out.stdout)

    print('check: %s (%s)' % ('IDENTICAL to the tree' if not changed else '%d paths differ' % changed, scratch))
    return 0 if not changed else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('vehicle', help='the vehicle JSON file')
    ap.add_argument('--root', default=gz_model.ROOT, help='the PX4 tree to write into (default PX4_ROOT or ~/dev/PX4-Autopilot)')
    ap.add_argument('--check', action='store_true', help='generate into a scratch copy and diff against the tree instead of writing')
    args = ap.parse_args(argv)
    vehicle = load_vehicle(args.vehicle)

    if args.check:
        return check(vehicle)

    p = generate(vehicle, args.root)
    print('done: %s, decade %d, travel +-%d deg, spans %s' % (vehicle.name, vehicle.decade, p.travel, p.d['spans']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
