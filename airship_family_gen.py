#!/usr/bin/env python3
"""Generate an airship's gz model family, world entries, posix airframes and
docs entries from one vehicle file.

    python3 airship_family_gen.py vehicles/airship_indoor.json          # write into the PX4 tree (PX4_ROOT)
    python3 airship_family_gen.py vehicles/airship_generic.json --check # regenerate the generic family in a
                                                                        # scratch copy and diff it against the tree
    python3 airship_family_gen.py vehicles/airship_indoor.json --overlay            # the family into the overlay
    python3 airship_family_gen.py vehicles/airship_generic.json --upstream overlay  # what upstream carries, into
                                                                                    # --root, from the overlay's members

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
import glob
import json
import math
import os
import re
import shutil
import sys
import tempfile
import textwrap

import airship_gains
import apply_generic_hull as A
import generic_hull_design as g
import gz_model

ROLES = ('base', 'base_am', 'bare', 'drag', 'fin', 'am', 'am_fin', 'am_fin_drag')
# the scheme's suffix, airframe mask and docs name of each member
MEMBERS = (('bare', 0, ''), ('drag', 1, 'Hull Drag'), ('fin', 2, 'Fins'), ('am', 4, 'Added Mass'),
           ('am_fin', 6, 'Added Mass, Fins'), ('am_fin_drag', 7, 'Complete'))
MASK = {role: mask for role, mask, _ in MEMBERS}
MOMENT_CONSTANT = 0.016     # the x500 motor block's, which the wrappers keep
GENERIC_DECADE = 2520       # vehicles/airship_generic.json's decade: the tree's template family and bare airframe


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

    @property
    def overlay_family(self):
        """The same family under the overlay's rigid naming."""
        if self.name == A.GENERIC.bare:
            return A.GENERIC_RIGID
        return A.vectored_family(self.name, self.body_title, self.surfaces, rigid=True)

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
    rename = renamer(src_fam, dst_fam)

    for src_name, dst_name in family_pairs(src_fam, dst_fam):
        src, dst = os.path.join(src_root, src_name), os.path.join(dst_root, dst_name)

        if os.path.abspath(src) == os.path.abspath(dst):
            continue

        for name in ('model.sdf', 'model.config'):
            write(os.path.join(dst, name), rename(read(os.path.join(src, name))))


def family_pairs(src_fam, dst_fam):
    """(source name, destination name) of every directory the two families
    share by role: the eight vectored roles and the fixed twin's members a
    source family has (the generic's; a vectored-only destination has none)."""
    pairs = [(getattr(src_fam, r), getattr(dst_fam, r)) for r in ROLES]
    pairs += list(zip(src_fam.fixed, dst_fam.fixed)) + list(zip(src_fam.fixed_drag, dst_fam.fixed_drag))
    return pairs


def renamer(src_fam, dst_fam):
    """text -> text with every source family name (and the body's title)
    replaced by the destination's, in one pass, longest name first: a chain
    of replacements would let a new name be hit by a later rule (the rigid
    naming turns _base_am into _base and _base into _base_rigid)."""
    pairs = family_pairs(src_fam, dst_fam)
    pairs += [(src_fam.title, dst_fam.title), (src_fam.title.lower(), dst_fam.title.lower())]
    # every source name is in the pattern, the unchanged ones mapped to
    # themselves: a longer name must be consumed whole, or its prefix would
    # be renamed inside it (<name>_am_fin would become <name>_rigid_am_fin)
    mapping = dict(pairs)

    if all(old == new for old, new in mapping.items()):
        return lambda t: t

    pattern = re.compile('|'.join(re.escape(old) for old in sorted(mapping, key=len, reverse=True)))
    return lambda t: pattern.sub(lambda m: mapping[m.group(0)], t)


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


def write_models(vehicle, root, fam=None, template=None):
    """The whole family under root (a models directory): the template family
    (the tree's generic by default) copied under the family's names, then the
    design, then the pods."""
    fam = fam or vehicle.family
    tfam, tmodels = template or (A.GENERIC, gz_model.GZ_MODELS)
    copy_family(tfam, fam, tmodels, root)

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
    return write_world_entries(path, [getattr(fam, role) for role, _, _ in MEMBERS])


def write_world_entries(path, names):
    """The buoyancy entries of these models, bare and _0.._3, after the last
    one present; those already there are left."""
    t = read(path)
    lines = []

    for name in names:
        for instance in ('',) + tuple('_%d' % i for i in range(4)):
            line = '      <enable>%s%s::base_link</enable>' % (name, instance)

            if line not in t:
                lines.append(line)

    if lines:
        end = t.index('\n', t.rindex('</enable>')) + 1
        t = t[:end] + '\n'.join(lines) + '\n' + t[end:]
        write(path, t)

    return len(lines)


def airframe_path(airframes, number, model):
    return os.path.join(airframes, '%d_gz_%s' % (number, model))


def write_airframes(vehicle, p, airframes, fam=None, decade=None, template=None, members_from=None):
    """The posix airframes of the decade. The bare one is the template's bare
    airframe (template = (bare number, family, airframes dir); the generic's
    in the tree by default) with this vehicle's names and propulsion numbers;
    a member is a header, its PX4_SIM_MODEL and the include of the bare one,
    or, with members_from = (airframes dir, family, decade) naming this
    vehicle's own members elsewhere (the tree's, for the overlay), that
    member's text with its bare include renamed: gains and all.
    A member that exists keeps its text: the gains written below its include
    are the calculator's, not this generator's."""
    fam, pods = fam or vehicle.family, vehicle.pods
    decade = decade or vehicle.decade
    t_number, tfam, t_airframes = template or (GENERIC_DECADE, A.GENERIC, airframes)
    rename = renamer(tfam, fam)
    bare_path = airframe_path(airframes, decade, fam.bare)
    own_bare = airframe_path(members_from[0], members_from[2], members_from[1].bare) if members_from else None
    existing = read(bare_path) if os.path.exists(bare_path) else None
    # an existing bare airframe is updated in place, so the block the
    # calculator appends below its propulsion lines survives a regeneration;
    # a new one is this vehicle's own bare airframe elsewhere (gains and
    # all), else the template's renamed
    if existing is not None:
        t = existing
    elif own_bare and os.path.exists(own_bare):
        t = renamer(members_from[1], fam)(read(own_bare))
    else:
        t = rename(read(airframe_path(t_airframes, t_number, tfam.bare)))
    t = with_name(t, vehicle.member_title('Rigid Hull') if fam.bare.endswith('_' + gz_model.RIGID) else vehicle.title)
    t = sub1(t, r'rotor speed 0\.\.[\d.]+ rad/s', 'rotor speed 0..%s rad/s' % fmt(pods.max_rot_velocity))

    for n in (1, 2):
        t = sub1(t, r'(param set-default SIM_GZ_EC_MAX%d )[\d.-]+' % n, r'\g<1>%s' % fmt(pods.max_rot_velocity))
        t = sub1(t, r'(param set-default SIM_GZ_SV_MINA%d )[\d.-]+' % n, r'\g<1>%s' % fmt(-pods.tilt_deg))
        t = sub1(t, r'(param set-default SIM_GZ_SV_MAXA%d )[\d.-]+' % n, r'\g<1>%s' % fmt(pods.tilt_deg))

    t = sub1(t, r'(param set-default CA_AIRSHIP_TLMIN )[\d.-]+', r'\g<1>%s' % fmt(-pods.tilt_deg))
    t = sub1(t, r'(param set-default CA_AIRSHIP_TLMAX )[\d.-]+', r'\g<1>%s' % fmt(pods.tilt_deg))
    t = sub1(t, r'(param set-default CA_AIRSHIP_TLT_R )[\d.-]+', r'\g<1>%s' % fmt(pods.tilt_rate_deg_s))
    written = []

    if existing != t:
        write(bare_path, t)
        written.append(os.path.basename(bare_path))

    for role, mask, docs_name in MEMBERS[1:]:
        path = airframe_path(airframes, decade + mask, getattr(fam, role))

        if os.path.exists(path):
            continue

        if members_from:
            m_airframes, m_fam, m_decade = members_from
            source = airframe_path(m_airframes, m_decade + mask, getattr(m_fam, role))

            if os.path.exists(source):
                # this vehicle's own member elsewhere: its text, the bare include renamed
                text = read(source).replace(os.path.basename(airframe_path(m_airframes, m_decade, m_fam.bare)), os.path.basename(bare_path))
                write(path, text)
                written.append(os.path.basename(path))
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


def write_surface_airframes(vehicle, p, airframes, fam=None, decade=None):
    """A surfaces family's fin members: the surfaces block right after the
    include of the bare airframe (its tilt servo mapping is overridden), and
    the members' own @output lines. Updated in place."""
    fam = fam or vehicle.family
    decade = decade or vehicle.decade

    if not fam.surfaces:
        return []

    written = []
    block = surfaces_block(p)

    for role, mask in SURFACE_MEMBERS:
        path = airframe_path(airframes, decade + mask, getattr(fam, role))
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


def docs_sections(vehicle, p, fam=None, rigid_title='', command='make px4_sitl gz_%s', page_link='../frames_airship/index.md'):
    """One docs section per member: the heading, its sentence and how it is
    run (`command` with the model name; the overlay's has no make target)."""
    fam = fam or vehicle.family
    a, b = p.a, p.b
    intro = {
        'bare': '[%s](%s): two independently tilting thrust pods on a %.2f x %.2f m hull, the rigid hull: '
                'a torque keeps accelerating it and nothing damps a rotation.' % (vehicle.title, page_link, 2 * a, 2 * b),
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
        out.append('### %s\n\n%s\n\n```sh\n%s\n```\n'
                   % (vehicle.member_title(docs_name or rigid_title), intro[role], command % getattr(fam, role)))

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


def write_fixed_servos(vehicle, p, airframes, fam=None):
    """A family with a fixed-thruster twin (the generic one: airship_fixed):
    its bare posix airframe maps the bridge's +-1 onto the flap joints'
    +-travel, SIM_GZ_SV_MINA/MAXA1..4 and the comment that says so. An
    airframes directory without the twin's airframe (the overlay, whose fixed
    airframes are not generated yet) is left alone."""
    fam = fam or vehicle.family
    path = fixed_airframe(airframes, fam)

    if path is None:
        return []

    paths = [path]
    t = read(path)
    new = sub1(t, r'(the gz joints stop at \+-)\d+( deg)', r'\g<1>%d\g<2>' % p.travel)

    for n in (1, 2, 3, 4):
        new = sub1(new, r'(param set-default SIM_GZ_SV_MINA%d )[\d.-]+' % n, r'\g<1>%d' % -p.travel)
        new = sub1(new, r'(param set-default SIM_GZ_SV_MAXA%d )[\d.-]+' % n, r'\g<1>%d' % p.travel)

    if new == t:
        return []

    write(paths[0], new)
    return [os.path.basename(paths[0])]


# What the generator writes, relative to the tree root: the two directories
# it adds files to and the four files it edits in place (check() copies them
# to a scratch tree and diffs; the test lists the same).
TREE_DIRS = ('Tools/simulation/gz/models', 'ROMFS/px4fmu_common/init.d-posix/airframes')
TREE_FILES = ('Tools/simulation/gz/worlds/lta.sdf', 'docs/en/sim_gazebo_gz/index.md', 'docs/en/sim_gazebo_gz/vehicles.md',
              'docs/en/frames_airship/index.md')


def is_overlay(root):
    """Whether a root is an overlay directory (models/, airframes/) rather than a PX4 tree."""
    return os.path.isdir(os.path.join(root, 'models'))


def homes(root):
    """(models dir, airframes dir) of a root, overlay or tree."""
    if is_overlay(root):
        return os.path.join(root, 'models'), os.path.join(root, 'airframes')

    return tuple(os.path.join(root, d) for d in TREE_DIRS)


def fixed_airframe(airframes, fam):
    """The one posix airframe of a family's fixed-thruster twin under an
    airframes dir, or None where there is none (a vectored-only family; the
    overlay, whose fixed airframes are not generated yet)."""
    if not fam.fixed:
        return None

    paths = glob.glob(os.path.join(airframes, '[0-9]*_gz_%s' % fam.fixed[0]))
    assert len(paths) <= 1, paths
    return paths[0] if paths else None


def with_name(text, title):
    """An airframe's text with its `# @name` line set to title."""
    return sub1(text, r'(# @name )[^\n]*', r'\g<1>%s' % title.replace('\\', '\\\\'))


# ---------------------------------------------------------------------------
# What upstream carries: one complete plant per family, in merge-include
# shape (docs/upstream_split_design_20261012.md S1). The variant members the
# plant was measured against stay in this repository's overlay; the plain
# model merges the added-mass body and carries the pods, the fins and the
# drag block in one file, and a twin check holds it element for element to
# the overlay's complete member.
# ---------------------------------------------------------------------------

def own_blocks(text):
    """A merge-include member's own content: what follows its include up to
    the model's end, which is the block that member adds to the chain."""
    start = text.index('</include>') + len('</include>')
    end = text.rindex('  </model>')
    return text[start:end].strip('\n') + '\n'


def canonical(elem):
    """An element as nested tuples (tag, attributes, text, children): what two
    merged models are compared by, whitespace and comments aside."""
    return (elem.tag, tuple(sorted(elem.attrib.items())), (elem.text or '').strip(), tuple(canonical(c) for c in elem))


class at_root:
    """gz_model's model and airframe homes pointed at these roots while
    inside: an overlay directory (models/, airframes/) or a PX4 tree. It
    moves gz_model's homes only: sourced_path's other ROMFS files and the yaml
    cache read the tree at ROOT, and yaw_model.controller caches
    airframe_defaults and airframe_for_model per process, so do not call
    controller inside."""

    def __init__(self, *roots):
        self.roots = roots

    def __enter__(self):
        self.saved = (gz_model.MODEL_HOMES, gz_model.AIRFRAME_HOMES)
        gz_model.MODEL_HOMES = [homes(r)[0] for r in self.roots]
        gz_model.AIRFRAME_HOMES = [homes(r)[1] for r in self.roots]

    def __exit__(self, *_):
        gz_model.MODEL_HOMES, gz_model.AIRFRAME_HOMES = self.saved


BODY_HEADER = '''<?xml version="1.0" encoding="UTF-8"?>
<!--
%s
-->
'''
PLAIN_MODEL = BODY_HEADER + '''<sdf version="1.10">
  <model name="%s">
    <include merge="true">
      <uri>model://%s</uri>
    </include>
%s  </model>
</sdf>
'''


@dataclasses.dataclass(frozen=True)
class Plain:
    """One plant upstream carries of a family: its plain name and number, and
    the members it is built from under members_root (their names in that
    home's naming, their decade there)."""
    name: str               # the plain model upstream carries
    title: str              # its airframe's @name
    blocks: tuple           # member roles whose own blocks it carries over the body, in chain order
    bare: str               # the bare member (its airframe is the plain one's text)
    complete: str           # the complete member (its yaw loop block, the twin check's other side)
    complete_mask: int      # the complete member's airframe is members_decade + mask
    decade: int             # the plain airframe's number upstream
    members_decade: int     # the members' decade under members_root (the overlay's is OVERLAY_SHIFT above)
    what: str               # what the blocks add to the body, for the header


def plain_plants(vehicle, members_root):
    """The plants upstream carries of this vehicle's family: the vectored one,
    and the generic's fixed-thruster twin (its number read from its airframe),
    built from the members under members_root: the overlay (rigid naming, a
    decade OVERLAY_SHIFT above) or the tree."""
    overlay = is_overlay(members_root)
    mfam = vehicle.overlay_family if overlay else vehicle.family
    shift = OVERLAY_SHIFT if overlay else 0
    out = [Plain(vehicle.name, vehicle.title, (mfam.am, mfam.am_fin, mfam.am_fin_drag), mfam.bare, mfam.am_fin_drag,
                 MASK['am_fin_drag'], vehicle.decade, vehicle.decade + shift,
                 'the propulsion, the four fins as lifting surfaces and the hull\'s drag and rotational damping')]
    path = fixed_airframe(homes(members_root)[1], mfam)

    if path is not None:
        m = re.search(r'# @name ([^\n]*)', read(path))
        assert m, '%s: no @name line' % path
        number = int(os.path.basename(path).split('_')[0])
        # the fixed twin's complete member is fixed_am_drag
        out.append(Plain(vehicle.family.fixed[0], m.group(1), (mfam.fixed[1], mfam.fixed_drag[1]), mfam.fixed[0], mfam.fixed_drag[1],
                         MASK['am'] | MASK['drag'], number - shift, number,
                         'the fixed thrusters, the four fins with their flaps and the hull\'s drag and rotational damping'))

    return out


def upstream_models(vehicle, members_root):
    """{model name: model.sdf text} of the models upstream carries: the
    added-mass body under the family's base name and each plain plant, built
    from the members under members_root (the overlay, or today's tree)."""
    fam = vehicle.family
    mfam = vehicle.overlay_family if is_overlay(members_root) else vehicle.family
    models = homes(members_root)[0]
    member = lambda name: read(os.path.join(models, name, 'model.sdf'))
    source = os.path.relpath(vehicle.path, os.path.dirname(os.path.abspath(__file__)))
    body = member(mfam.base_am)
    body = body[body.index('<sdf '):]
    body = sub1(body, r'<model name="%s">' % re.escape(mfam.base_am), '<model name="%s">' % fam.base)
    body = BODY_HEADER % textwrap.fill("%s: the body the vehicle merges. The hull, the gondola, the tail and the sensors, with "
                                       "the hull's fluid added mass on base_link. Generated by px4-airship-tools "
                                       "(airship_family_gen.py, upstream mode) from %s." % (vehicle.body_title, source),
                                       78, initial_indent='  ', subsequent_indent='  ') + body
    out = {fam.base: body}

    for plain in plain_plants(vehicle, members_root):
        header = textwrap.fill("%s: the complete plant, %s merged with %s. Generated by px4-airship-tools "
                               "(airship_family_gen.py, upstream mode) from %s; the physics-variant members it was measured "
                               "against live there." % (plain.title, fam.base, plain.what, source), 78, initial_indent='  ',
                               subsequent_indent='  ')
        out[plain.name] = PLAIN_MODEL % (header, plain.name, fam.base, ''.join(own_blocks(member(role)) for role in plain.blocks))

    return out


UPSTREAM_MARK = '# Yaw loop values for this hull'      # the marker's wording in an upstream airframe: no tool names
# Parameters the upstream class and controller do not have (the fork's later
# rounds: pod grouping, the forward-speed schedule): a line setting one is
# dropped from an upstream airframe, with its paragraph when nothing else is set there
UPSTREAM_ABSENT = ('CA_AIRSHIP_GRP', 'AS_SPD_SRC', 'AS_SPD_WMAX', 'AS_SPD_DP_TOL', 'AS_SPD_HI', 'AS_YR_FF_SLP')
OVERLAY_INCLUDES = ('model://ground_markers', 'model://flight_grid')   # lab-only models the upstream world does not include


def without_absent(text):
    """`text` less the lines that set an UPSTREAM_ABSENT parameter; a paragraph
    (blank-line separated) left with comments only, having set only those, goes too."""
    out = []

    for para in text.split('\n\n'):
        lines = para.split('\n')
        kept = [l for l in lines if not any(l.startswith('param set-default %s ' % n) for n in UPSTREAM_ABSENT)]
        had = any(l.startswith('param set-default ') for l in lines)
        has = any(l.startswith('param set-default ') for l in kept)

        if had and not has:
            continue

        out.append('\n'.join(kept))

    return '\n\n'.join(out)


def upstream_airframe(plain, members_root):
    """The posix airframe of a plain plant: the bare member's text, named for
    the vehicle and spawning the plain model, with the complete member's yaw
    loop block after the bare one's (the resolved defaults of the chain).
    The upstream class drives independent pods only, so the grouping
    parameter is left out, and the gains marker names no tool."""
    airframes = homes(members_root)[1]
    bare = read(airframe_path(airframes, plain.members_decade, plain.bare))
    complete = read(airframe_path(airframes, plain.members_decade + plain.complete_mask, plain.complete))
    t = with_name(bare, plain.title)
    t = sub1(t, r'PX4_SIM_MODEL=\$\{PX4_SIM_MODEL:=%s\}' % re.escape(plain.bare), 'PX4_SIM_MODEL=${PX4_SIM_MODEL:=%s}' % plain.name)
    t = t.replace(airship_gains.MARK_OPEN, UPSTREAM_MARK)

    if airship_gains.MARK_OPEN in complete:
        block = complete[complete.index(airship_gains.MARK_OPEN) + len(airship_gains.MARK_OPEN):].strip('\n')
        t = t.rstrip('\n') + '\n\n' + block + '\n'

    return without_absent(t)


def model_config(name, author, description):
    """A model.config: the name, the sdf, the member's author block, the description."""
    return ('<?xml version="1.0"?>\n<model>\n  <name>%s</name>\n  <version>1.0</version>\n  <sdf version="1.10">model.sdf</sdf>\n'
            '%s  <description>%s</description>\n</model>\n' % (name, author, description))


def upstream_world(path, members_root, names):
    """The upstream world, made once from the members' world: its buoyancy
    entries only for `names` (bare and _0.._3) and without the lab-only
    includes (OVERLAY_INCLUDES)."""
    if os.path.exists(path):
        return False

    keep = {'%s%s' % (n, i) for n in names for i in ('',) + tuple('_%d' % k for k in range(4))}
    out = []

    for line in read(os.path.join(members_root, TREE_FILES[0])).split('\n'):
        m = re.match(r'\s*<enable>([^:]+)::', line)

        if m and m.group(1) not in keep:
            continue

        out.append(line)

    t = '\n'.join(out)

    for uri in OVERLAY_INCLUDES:
        t = re.sub(r'\n *<include>\s*<uri>%s</uri>\s*</include>' % re.escape(uri), '', t)

    write(path, t)
    return True


def write_upstream(vehicle, root, members_root, plants=None):
    """What upstream carries of the family into root (a PX4 tree trimmed of
    the family): the body, each plain plant (those named in `plants`, all by
    default), its model.config, its airframe and its world entries, from the
    members under members_root (the overlay, or a tree that still carries
    the members); the world itself is made from the members' when root has
    none. Refused when root is members_root (the plain plant would overwrite
    the members it is built from) or when root still carries the family's
    variant members (the trim comes first). Returns (written, world entries added)."""
    fam = vehicle.family
    mfam = vehicle.overlay_family if is_overlay(members_root) else vehicle.family
    member_models = homes(members_root)[0]

    if os.path.realpath(root) == os.path.realpath(members_root):
        raise SystemExit('--upstream: --root and MEMBERS_ROOT are the same tree; the plain plant would overwrite the '
                         'members it is built from')

    kept = {fam.base, fam.bare} | set(fam.fixed[:1])
    present = [m for m in fam.members if m not in kept and os.path.isfile(os.path.join(root, TREE_DIRS[0], m, 'model.sdf'))]

    if present:
        raise SystemExit('%s still carries the family\'s members: %s; --upstream writes into a trimmed tree' % (root, ', '.join(present)))

    written = []
    models = os.path.join(root, TREE_DIRS[0])

    plains = [p for p in plain_plants(vehicle, members_root) if plants is None or p.name in plants]
    configs = {fam.base: (mfam.base_am, '%s: the body the vehicle merges, with the hull\'s fluid added mass. Not meant to '
                                        'be spawned directly.' % vehicle.body_title)}
    configs.update({p.name: (p.complete, '%s: the complete plant for PX4 SITL, %s merged with %s.' % (p.title, fam.base, p.what))
                    for p in plains})

    for name, text in upstream_models(vehicle, members_root).items():
        if name not in configs:
            continue

        path = os.path.join(models, name, 'model.sdf')

        if not os.path.exists(path) or read(path) != text:
            write(path, text)
            written.append(name)

        member, description = configs[name]
        author = re.search(r'  <author>.*?</author>\n', read(os.path.join(member_models, member, 'model.config')), flags=re.S).group(0)
        config = model_config(name, author, description)
        path = os.path.join(models, name, 'model.config')

        if not os.path.exists(path) or read(path) != config:
            write(path, config)

    airframes = os.path.join(root, TREE_DIRS[1])

    for plain in plains:
        path = airframe_path(airframes, plain.decade, plain.name)
        text = upstream_airframe(plain, members_root)

        if not os.path.exists(path) or read(path) != text:
            write(path, text)
            written.append(os.path.basename(path))

    world = os.path.join(root, TREE_FILES[0])
    upstream_world(world, members_root, [plain.name for plain in plains])
    return written, write_world_entries(world, [plain.name for plain in plains])


def twin_check(vehicle, upstream_root, members_root, plants=None):
    """The problems that keep each plain plant under upstream_root from being
    its complete member under members_root: the merged models element for
    element, and the airframes' resolved defaults (less the parameters the
    upstream class and controller do not have). [] means twins."""
    problems = []

    for plain in [p for p in plain_plants(vehicle, members_root) if plants is None or p.name in plants]:
        with at_root(upstream_root):
            mine = canonical(gz_model.merged(plain.name))[3]
            mine_defaults = gz_model.airframe_defaults(plain.name)

        with at_root(members_root):
            theirs = canonical(gz_model.merged(plain.complete))[3]
            theirs_defaults = gz_model.airframe_defaults(plain.complete)

        if mine != theirs:
            first = next((i for i, (a, b) in enumerate(zip(mine, theirs)) if a != b), 'length')
            problems.append('%s: merged models differ, %d vs %d children, first difference at child %s' % (plain.name, len(mine), len(theirs), first))

        # the upstream class and controller lack UPSTREAM_ABSENT: the members' values of those are not compared
        theirs_defaults = {k: v for k, v in theirs_defaults.items() if k not in UPSTREAM_ABSENT}

        if mine_defaults != theirs_defaults:
            keys = sorted(k for k in set(mine_defaults) | set(theirs_defaults) if mine_defaults.get(k) != theirs_defaults.get(k))
            problems.append('%s: airframe defaults differ: ' % plain.name
                            + ', '.join('%s %s/%s' % (k, mine_defaults.get(k), theirs_defaults.get(k)) for k in keys))

    return problems


# ---------------------------------------------------------------------------
# The overlay: the whole family, under the rigid naming, in this repository
# (gz_model.OVERLAY) instead of the tree, numbered a hundred above the
# tree's decade (design S2, S5). The template is the overlay's own generic
# rigid family once it exists, the tree's generic family before. The overlay
# keeps its own added-mass body (<name>_base_am): upstream's plain plant
# merges a copy under the base name, and the twin check holds the two equal.
# ---------------------------------------------------------------------------

OVERLAY_SHIFT = gz_model.OVERLAY_SHIFT      # the overlay decade of a vehicle: its JSON decade plus this
OVERLAY_WORLD_NAME = 'lta_lab'


def overlay_template(overlay, template_root):
    """((family, models dir), (bare number, family, airframes dir)) the
    overlay generator copies from: the overlay's generic rigid family if it
    is there, else the tree's generic family under template_root."""
    models, airframes = homes(overlay)
    rigid = A.GENERIC_RIGID

    if os.path.isdir(os.path.join(models, rigid.bare)):
        return (rigid, models), (GENERIC_DECADE + OVERLAY_SHIFT, rigid, airframes)

    tree_models, tree_airframes = homes(template_root)
    return (A.GENERIC, tree_models), (GENERIC_DECADE, A.GENERIC, tree_airframes)


def overlay_world(path, template_root):
    """The overlay's lab world: the tree's lta world under its own name, made once."""
    if os.path.exists(path):
        return False

    t = read(os.path.join(template_root, TREE_FILES[0]))
    t = sub1(t, r'<world name="[^"]*">', '<world name="%s">' % OVERLAY_WORLD_NAME)
    write(path, t)
    return True


def write_overlay_decades(path, decade, bare):
    """One `<decade> <bare model>` line per family, in number order, once."""
    lines = read(path).splitlines() if os.path.exists(path) else []
    line = '%d %s' % (decade, bare)

    if line in lines:
        return False

    if any(l.split()[0] == str(decade) for l in lines if l.split()):
        raise SystemExit('%s: decade %d is taken (%s)' % (path, decade, [l for l in lines if l.startswith(str(decade))]))

    lines.append(line)
    write(path, '\n'.join(sorted(lines, key=lambda l: int(l.split()[0]))) + '\n')
    return True


OVERLAY_DOCS_HEAD = '''# Overlay vehicles

The families this repository carries beside the PX4 tree (`docs/upstream_split_design_20261012.md`):
every physics-variant member of each vehicle, under the overlay numbers. A member is flown with
`GZ_TARGET=gz_<model> ./gz_run.sh <check>`; there is no make target.

| Vehicle | Model | SYS_AUTOSTART |
|---|---|---|
'''


def write_overlay_docs(path, vehicle, p, fam, decade):
    """The overlay's vehicle list: a table row per member (the shape
    variant_check reads) and the members' sections, each once."""
    t = read(path) if os.path.exists(path) else OVERLAY_DOCS_HEAD
    rows = []

    for role, mask, docs_name in MEMBERS:
        title = vehicle.member_title(docs_name or 'Rigid Hull')
        row = '| [%s](#%s) | `gz_%s` | %d |' % (title, slug(title), getattr(fam, role), decade + mask)

        if '`gz_%s`' % getattr(fam, role) not in t:
            rows.append(row)

    if rows:
        end = t.index('\n', t.rindex('|---|---|---|')) + 1
        table_end = end

        while table_end < len(t) and t[table_end] == '|':
            table_end = t.index('\n', table_end) + 1

        t = t[:table_end] + '\n'.join(rows) + '\n' + t[table_end:]

    sections = docs_sections(vehicle, p, fam, rigid_title='Rigid Hull', command='GZ_TARGET=gz_%s ./gz_run.sh <check>',
                             page_link='https://docs.px4.io/main/en/frames_airship/')

    if '### %s\n' % vehicle.member_title('Rigid Hull') not in t:
        t = t.rstrip('\n') + '\n\n' + '\n'.join(sections)

    write(path, t)
    return bool(rows)


def generate_overlay(vehicle, overlay=None, template_root=None):
    """The vehicle's whole family into the overlay, self-contained: models,
    airframes a hundred above the tree's decade, the lab world's entries, the
    decade line and the docs."""
    overlay = overlay or gz_model.OVERLAY
    template_root = template_root or gz_model.ROOT
    fam = vehicle.overlay_family
    decade = vehicle.decade + OVERLAY_SHIFT
    (tfam, tmodels), bare_template = overlay_template(overlay, template_root)
    models, airframes = os.path.join(overlay, 'models'), os.path.join(overlay, 'airframes')
    os.makedirs(airframes, exist_ok=True)
    p = write_models(vehicle, models, fam=fam, template=(tfam, tmodels))
    # the vehicle's own members in the tree (before the trim) carry their gains
    # blocks: the overlay takes them as they are
    members_from = (os.path.join(template_root, TREE_DIRS[1]), vehicle.family, vehicle.decade)
    written = write_airframes(vehicle, p, airframes, fam=fam, decade=decade, template=bare_template, members_from=members_from)
    written += write_surface_airframes(vehicle, p, airframes, fam=fam, decade=decade)
    written += write_fixed_servos(vehicle, p, airframes, fam=fam)
    world = os.path.join(overlay, 'worlds', OVERLAY_WORLD_NAME + '.sdf')
    overlay_world(world, template_root)
    n = write_world_entries(world, [getattr(fam, role) for role, _, _ in MEMBERS])
    write_overlay_decades(os.path.join(airframes, 'decades'), decade, fam.bare)
    write_overlay_docs(os.path.join(overlay, 'vehicles.md'), vehicle, p, fam, decade)
    return p, written, n


def generate(vehicle, root):
    models = os.path.join(root, TREE_DIRS[0])
    airframes = os.path.join(root, TREE_DIRS[1])
    p = write_models(vehicle, models)
    n = write_world(os.path.join(root, TREE_FILES[0]), vehicle.family)
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

    for rel in TREE_DIRS:
        shutil.copytree(os.path.join(root, rel), os.path.join(scratch, rel))

    for rel in TREE_FILES:
        os.makedirs(os.path.dirname(os.path.join(scratch, rel)), exist_ok=True)
        shutil.copy(os.path.join(root, rel), os.path.join(scratch, rel))

    generate(vehicle, scratch)
    changed = 0

    for rel in TREE_DIRS + TREE_FILES:
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
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument('--check', action='store_true', help='generate into a scratch copy and diff against the tree instead of writing')
    mode.add_argument('--overlay', action='store_true',
                      help='write the whole family under the rigid naming into the overlay (gz_model.OVERLAY) instead of the tree')
    mode.add_argument('--upstream', metavar='MEMBERS_ROOT',
                      help='write what upstream carries (the plain complete plants in merge-include shape) into --root, a tree '
                           'trimmed of the family, from the members under MEMBERS_ROOT (the overlay, or a tree that still has '
                           'them), then run the twin check')
    ap.add_argument('--plants', help='with --upstream: the plain plants to write, comma-separated (default all)')
    args = ap.parse_args(argv)
    vehicle = load_vehicle(args.vehicle)

    if args.check:
        return check(vehicle)

    if args.overlay:
        p, written, n = generate_overlay(vehicle, template_root=args.root)
        print('overlay: %s, decade %d; airframes written: %s; world entries added %d'
              % (vehicle.name, vehicle.decade + OVERLAY_SHIFT, ', '.join(written) or 'none', n))
        return 0

    if args.upstream:
        plants = args.plants.split(',') if args.plants else None
        written, n = write_upstream(vehicle, args.root, args.upstream, plants)
        problems = twin_check(vehicle, args.root, args.upstream, plants)
        print('upstream: wrote %s; world entries added %d; twin check %s'
              % (', '.join(written) or 'nothing', n, 'OK' if not problems else '; '.join(problems)))
        return 0 if not problems else 1

    p = generate(vehicle, args.root)
    print('done: %s, decade %d, travel +-%d deg, spans %s' % (vehicle.name, vehicle.decade, p.travel, p.d['spans']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
