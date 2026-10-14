#!/usr/bin/env python3
"""Reading a gz model.sdf. No simulator required.

The split from gz_link.py is deliberate: these are the checks' questions about
what the vehicle IS, and they must stay answerable with nothing running.
aero_moment_check's analytic half and cloudship_geom_check are built entirely
on this module, and both have to work on a machine with no gz at all.

The rule throughout is that a number belongs to the model. Four files had
copies of the same quantities - the hull's drag coefficients, the pods' full
thrust, the duty range, the model root - and a copy is how a tolerance quietly
stops describing the vehicle it is applied to. Anything a check needs to know
about the plant it asks for here.
"""
import glob
import json
import math
import os
import re
import xml.etree.ElementTree as ET

ROOT = os.environ.get('PX4_ROOT', os.path.expanduser('~/dev/PX4-Autopilot'))
# No PX4_GZ_MODELS override. px4-rc.gzsim sources gz_env.sh, which exports it
# unconditionally, so a value set out here moves only what the CHECKS read and
# not the tree the simulator spawns from - which is a check measuring one
# vehicle and reporting on another. (The one path where it does survive is the
# "gazebo already running world" attach, which gz_run.sh refuses with exit
# 10.) PX4_ROOT moves the whole tree, which is the knob that means something.
GZ_MODELS = os.path.join(ROOT, 'Tools/simulation/gz/models')
AIRFRAMES = os.path.join(ROOT, 'ROMFS/px4fmu_common/init.d-posix/airframes')
FLIGHT_AIRFRAMES = os.path.join(ROOT, 'ROMFS/px4fmu_common/init.d/airframes')
CMAKE_POSIX = os.path.join(AIRFRAMES, 'CMakeLists.txt')
# A model or a posix gz airframe has two homes: this repository's overlay,
# where the families upstream does not carry live, and the PX4 tree. Names
# never collide between them (the overlay's mask-0 member carries the _rigid
# token, below), so the order only says where a name is looked for first.
# The writers (airship_family_gen, apply_generic_hull) still take a root.
HERE = os.path.dirname(os.path.abspath(__file__))
OVERLAY = os.environ.get('AIRSHIP_OVERLAY', os.path.join(HERE, 'overlay'))
MODEL_HOMES = [os.path.join(OVERLAY, 'models'), GZ_MODELS]
AIRFRAME_HOMES = [os.path.join(OVERLAY, 'airframes'), AIRFRAMES]
VEHICLES = os.path.join(HERE, 'vehicles')     # the vehicle files the generator and the checks read
# What the overlay keeps beside its models and airframes, the tree's CMake
# comment, world and docs table in overlay shape: a decade table (one
# `<decade> <bare model>[ reserved]` line each), the lab world the overlay
# members are spawned in, and the vehicle list
OVERLAY_DECADES = os.path.join(OVERLAY, 'airframes', 'decades')
OVERLAY_WORLD = os.path.join(OVERLAY, 'worlds', 'lta_lab.sdf')
OVERLAY_DOCS = os.path.join(OVERLAY, 'vehicles.md')
OVERLAY_SHIFT = 100         # an overlay family's decade is its vehicle's tree decade plus this: 25x0 -> 26x0
# The airship parameters an airframe may set that a tree may lack: the fork's
# later rounds (pod grouping, tail, surface credit, the forward-speed
# schedule) under the upstream-bound tree, none under the fork. A line setting
# one is dropped from an airframe written into that tree, and the twin check
# leaves it out, so the rule is asked of the tree, not fixed in a list
PARAM_CANDIDATES = ('CA_AIRSHIP_GRP', 'CA_AIRSHIP_TAIL', 'CA_AIRSHIP_CS_K',
                    'AS_SPD_SRC', 'AS_SPD_WMAX', 'AS_SPD_DP_TOL', 'AS_SPD_HI', 'AS_YR_FF_SLP')
PARAM_YAMLS = {'CA_': 'src/modules/control_allocator/module.yaml',
               'AS_': 'src/modules/airship_att_control/airship_att_control_params.yaml'}


def absent_params(root=ROOT):
    """The PARAM_CANDIDATES the tree at root (ROOT by default) does not define (yaml_default raises)."""
    out = []

    for name in PARAM_CANDIDATES:
        try:
            yaml_default(PARAM_YAMLS[name[:3]], name, root)
        except LookupError:
            out.append(name)

    return tuple(out)
# The gz world of a SITL run, as the launcher exported it: gz_run.sh resolves
# the tree's world (`lta` unless the target names another) or the overlay's
# `lta_lab`. `lta` is for a check attached by hand to a world someone else
# started; the one default lives here so no module keeps its own.
WORLD = os.environ.get('PX4_GZ_WORLD', 'lta')


def in_overlay(path):
    """Whether a model or airframe path is the overlay's (else the tree's)."""
    return os.path.abspath(path).startswith(os.path.abspath(OVERLAY) + os.sep)
GZ_PLUGINS = os.environ.get('PX4_GZ_PLUGINS',
                            os.path.join(ROOT, 'build/px4_sitl_default/src/'
                                               'modules/simulation/gz_plugins'))

# worlds/lta.sdf:12 is <gravity>0 0 -9.8</gravity> and :177 is
# <uniform_fluid_density>1.225</uniform_fluid_density>. Both were restated in
# three files with three different values between them - 9.8, 9.8066 and 9.81 -
# one of which called itself "the gz default" and was not. They live here
# because every consumer already imports this module and none of them needs a
# simulator to read a world constant.
G = 9.8
RHO_AIR = 1.225

# A plugin is matched by its filename or by its name, and the two spellings of
# one plugin are easy to hold against each other by mistake: keep them apart.
HYDRO_NAME = 'gz::sim::systems::Hydrodynamics'
HYDRO_FILE = 'gz-sim-hydrodynamics-system'
LIFT_DRAG_FILE = 'gz-sim-lift-drag-system'
MULTICOPTER = 'multicopter-motor-model'
GENERIC = 'GenericMotorModel'


def model_dir(name):
    """The directory of a model, from whichever home holds its model.sdf."""
    for home in MODEL_HOMES:
        path = os.path.join(home, name)

        if os.path.isfile(os.path.join(path, 'model.sdf')):
            return path

    raise LookupError('%s 라는 모델이 없다 (%s)' % (name, ', '.join(MODEL_HOMES)))


def has_model(name):
    try:
        model_dir(name)
    except LookupError:
        return False

    return True


def model_names():
    """Every model name over the homes, sorted. A name in both homes is an
    error: the homes are told apart by name, never by order."""
    seen = {}

    for home in MODEL_HOMES:
        if not os.path.isdir(home):
            continue

        for d in os.listdir(home):
            if not os.path.isfile(os.path.join(home, d, 'model.sdf')):
                continue

            if d in seen:
                raise LookupError('%s 모델이 두 집에 있다: %s, %s' % (d, seen[d], home))

            seen[d] = home

    return sorted(seen)


def airframe_files():
    """[(file name, path)] of every posix gz airframe over the homes, sorted
    by file name; a file name in both homes is an error, as for models."""
    seen = {}

    for home in AIRFRAME_HOMES:
        if not os.path.isdir(home):
            continue

        for f in os.listdir(home):
            path = os.path.join(home, f)

            if not os.path.isfile(path) or '_gz_' not in f and '_gazebo-classic_' not in f:
                continue

            if f in seen:
                raise LookupError('%s 에어프레임이 두 집에 있다: %s, %s' % (f, seen[f], path))

            seen[f] = path

    return sorted(seen.items())


def airframe_path(file_name):
    """The path of one posix airframe file, from whichever home holds it."""
    for home in AIRFRAME_HOMES:
        path = os.path.join(home, file_name)

        if os.path.isfile(path):
            return path

    raise LookupError('%s 에어프레임이 없다 (%s)' % (file_name, ', '.join(AIRFRAME_HOMES)))


def sourced_path(rel):
    """The file a `. ${R}etc/<rel>` line sources: a posix airframe is looked
    for over the homes (an overlay member sources its own bare one), anything
    else is the tree's ROMFS."""
    prefix = 'init.d-posix/airframes/'

    if rel.startswith(prefix) and '/' not in rel[len(prefix):]:
        return airframe_path(rel[len(prefix):])

    return os.path.join(ROOT, 'ROMFS/px4fmu_common', rel)


_SOURCE = re.compile(r'^\s*\.\s+\$\{R\}etc/(\S+)\s*$')
_SET_DEFAULT = re.compile(r'^\s*param set-default (\w+) (\S+)')
_SIM_MODEL = re.compile(r'^\s*PX4_SIM_MODEL=\$\{PX4_SIM_MODEL:=(\S+)\}')
_OUTPUT = re.compile(r'^\s*#\s*@output\s+(Motor|Servo)(\d+)\s+(.+?)\s*$')


def airframe_settings(path, follow=True, _depth=0):
    """(kind, name, value, file, line) of a posix airframe in shell order
    through every file it sources (follow=False: this file's own lines only):
    kind 'default' for a `param set-default` (value as written), 'model' for
    the PX4_SIM_MODEL default, 'output' for a `# @output` line (name 'motor1',
    'servo2', ...; value its label). The one walker of the chain: a later line
    of a name wins as it does in the shell, and a sourced file that cannot be
    read is an error, never a silent absence."""
    if _depth > 8:
        raise RecursionError('%s: 에어프레임 포함이 8 단보다 깊다' % path)

    with open(path, encoding='utf-8') as handle:
        for number, line in enumerate(handle, 1):
            m = _SOURCE.match(line)

            if m:
                if follow:
                    yield from airframe_settings(sourced_path(m.group(1)), True, _depth + 1)

                continue

            m = _SET_DEFAULT.match(line)

            if m:
                yield 'default', m.group(1), m.group(2), path, number
                continue

            m = _SIM_MODEL.match(line)

            if m:
                yield 'model', 'PX4_SIM_MODEL', m.group(1), path, number
                continue

            m = _OUTPUT.match(line)

            if m:
                yield 'output', '%s%s' % (m.group(1).lower(), m.group(2)), m.group(3), path, number


def vehicle_files():
    """{name: the file's dict} of every vectored vehicle file in vehicles/
    (pods and a hull), in file-name order."""
    out = {}

    for path in sorted(glob.glob(os.path.join(VEHICLES, '*.json'))):
        with open(path) as handle:
            v = json.load(handle)

        if 'pods' in v and 'hull' in v:
            out[v['name']] = v

    return out


_YAML_CACHE = {}


def yaml_default(rel, name, root=ROOT):
    """The default of one parameter in a PX4 module yaml (parameters: -
    group: definitions: NAME: default) of the tree at root (ROOT by default),
    as written. LookupError when the file is not there, does not define the
    name, or gives it no default: a default that cannot be read is not a
    default of 0. The file is parsed once."""
    path = os.path.join(root, rel)    # the cache is by path: a caller may ask another tree (absent_params(root))

    if path not in _YAML_CACHE:
        import yaml

        if not os.path.exists(path):
            raise LookupError('%s 가 없다' % path)

        with open(path, encoding='utf-8') as handle:
            _YAML_CACHE[path] = yaml.safe_load(handle)

    for group in _YAML_CACHE[path].get('parameters', []):
        spec = (group.get('definitions') or {}).get(name)

        if spec is not None:
            if 'default' not in spec:
                raise LookupError('%s: %s 에 기본값이 없다' % (rel, name))

            return spec['default']

    raise LookupError('%s 의 기본값을 %s 에서 찾지 못했다' % (name, rel))


def sdf(name):
    """The <model> element of <home>/<name>/model.sdf."""
    return ET.parse(os.path.join(model_dir(name), 'model.sdf')).getroot().find('model')


def named(parent, tag, name):
    """One child by its name attribute. Position is not identity."""
    for child in parent.findall(tag):
        if child.get('name') == name:
            return child
    raise LookupError('%s[@name="%s"] 이 없다' % (tag, name))


def pose(elem, index):
    """One component of an element's pose, 0 when it has none.

    Index 0..2 are metres. 3..5 are radians unless the element carries
    degrees="true", which the Cloudship's links do and the airship's do not -
    a difference that is invisible until something reads an angle.
    """
    found = elem.find('pose')

    if found is None:
        return 0.0

    value = float(found.text.split()[index])

    if index >= 3 and found.get('degrees') == 'true':
        return math.radians(value)

    return value


def zof(elem):
    return pose(elem, 2)


def displacement(collision):
    """Volume of one collision, or None when it displaces nothing.

    The shapes the Buoyancy plugin measures, in the same order its switch
    takes them. A shape it can lift but this cannot measure would read as
    zero displacement and quietly move the center of volume, so anything
    unrecognized is an error rather than a skip.
    """
    geom = collision.find('geometry')

    if geom is None:
        return None

    box = geom.find('box/size')
    if box is not None:
        return math.prod(float(x) for x in box.text.split())

    ellipsoid = geom.find('ellipsoid/radii')
    if ellipsoid is not None:
        return 4.0 / 3.0 * math.pi * math.prod(float(x) for x in ellipsoid.text.split())

    sphere = geom.find('sphere/radius')
    if sphere is not None:
        return 4.0 / 3.0 * math.pi * float(sphere.text) ** 3

    cylinder = geom.find('cylinder')
    if cylinder is not None:
        return (math.pi * float(cylinder.find('radius').text) ** 2
                * float(cylinder.find('length').text))

    if geom.find('plane') is not None:
        return None

    raise LookupError('%s 의 형상을 잴 수 없다' % collision.get('name'))


def parent(name):
    """The model a merge-include variant wraps, or None for one that defines
    its own links. The physics variants are chains of these: cloudship_am_fin
    wraps cloudship_am, which is a full copy and so has no parent."""
    include = sdf(name).find('include')

    if include is None or include.get('merge') != 'true':
        return None

    return include.find('uri').text.replace('model://', '')


def family(name):
    """[base, ..., name]: the include chain, the model that carries base_link
    first. This is what centre_of_mass(), centre_of_volume(), inertia() and
    merged() want (a table keyed by vehicle uses vehicle()), and it is read
    from the SDF rather than kept as a list, so a new variant is known the
    moment its model.sdf exists."""
    chain = [name]

    while parent(chain[0]):
        chain.insert(0, parent(chain[0]))

    return chain


# In the scheme's order, <vehicle>[_am][_fin][_drag]: variant_check reads the
# order from this tuple.
SCHEME_TOKENS = ('am', 'fin', 'drag')
# The overlay's mask-0 member, <vehicle>_rigid: none of the physics tokens,
# and exclusive with them. The tree's plain name is the vehicle itself, bare
# today and the complete plant once upstream carries only that.
RIGID = 'rigid'


def family_tokens(name):
    """The scheme tokens a model name carries, in order: ['am', 'fin'] for
    airship_vectored_independent_am_fin, ['rigid'] for the overlay's bare
    member, [] for a plain name. The rigid token beside a physics token is
    not the scheme's."""
    tokens = name.split('_')
    kept = []

    while len(tokens) > 1 and tokens[-1] in SCHEME_TOKENS:
        kept.insert(0, tokens.pop())

    if len(tokens) > 1 and tokens[-1] == RIGID:
        if kept:
            raise LookupError('%s: _rigid 는 물리 토큰과 함께 쓰지 않는다' % name)

        kept = [RIGID]

    return kept


def hull_length(name):
    """The hull's length, m: twice the x semi-axis of base_link's hull
    collision ellipsoid, which the airship bodies draw and collide alike.
    A hull collided as something else is not this function's to measure."""
    link = merged(name).find('link')
    collision = named(link, 'collision', 'hull_collision')
    radii = collision.find('geometry/ellipsoid/radii')

    if radii is None:
        raise LookupError('%s 의 hull_collision 이 타원체가 아니다' % name)

    return 2.0 * float(radii.text.split()[0])


def vehicle(name):
    """The vehicle a model is a physics variant of, by the naming scheme:
    the name with its trailing _am/_fin/_drag tokens removed. Not the
    include chain: airship_vectored_independent wraps airship_base, which
    is a body and not a vehicle, and a table keyed by vehicle wants the
    former."""
    parts = name.split('_')
    kept = family_tokens(name)
    return '_'.join(parts[:len(parts) - len(kept)])


def bare(vehicle):
    """The mask-0 member of a vehicle: its _rigid model where one exists (the
    overlay), else the plain name (the tree today)."""
    rigid = '%s_%s' % (vehicle, RIGID)
    return rigid if has_model(rigid) else vehicle


def includes(name):
    """Every <include> of a model, merge or not: a variant must have one."""
    return sdf(name).findall('include')


def merged(name):
    """One <model> element holding the whole family: what gz's merge include
    builds at load, so that a check can ask a variant the same questions it
    asks the base. Children come in chain order, base first, and a variant's
    own <include> is dropped since it is what this replaces. The links,
    joints and motor plugins of cloudship_am_fin_drag are cloudship_am's;
    its fins are cloudship_am_fin's; its drag block its own."""
    chain = family(name)
    root = ET.Element('model', {'name': name})

    for member in chain:
        for child in sdf(member):
            if child.tag != 'include':
                root.append(child)

    return root


def centre_of_mass(models):
    """(total mass, (x, y, z)) over every link of one airframe's model family.

    Takes the FAMILY and not one model, because a merge-include wrapper has no
    links of its own: the previous per-model form answered 0.0 kg for the
    49 kg airship_vectored_independent_drag and 1.0 kg for its sibling, because
    both are wrappers around airship_base. A vehicle that weighs nothing is a
    denominator, and gz_run.sh supports the _drag target today.

    A link's <inertial> pose is relative to the link frame, so both poses are
    summed. Two callers disagreed about that and agreed only while base_link
    sat at the origin, which it does in both families today.
    """
    total = 0.0
    moment = [0.0, 0.0, 0.0]

    for name in models:
        for link in sdf(name).findall('link'):
            inertial = link.find('inertial')

            if inertial is None or inertial.find('mass') is None:
                continue

            mass = float(inertial.find('mass').text)
            total += mass

            for i in range(3):
                moment[i] += mass * (pose(link, i) + pose(inertial, i))

    if total <= 0.0:
        raise LookupError('%s 에 질량이 있는 링크가 없다' % ', '.join(models))

    return total, tuple(c / total for c in moment)


def centre_of_volume(models):
    """(displaced volume, (x, y, z)) of base_link's collisions, in the model
    frame: what the Buoyancy plugin lifts and where. Same shape as
    centre_of_mass(), and like it takes the family; base_link is models[0]'s.

    Three checks each summed this, and the copies had drifted: one left out
    base_link's pose, one parsed only ellipsoids and boxes and skipped the
    rest, and one read a hull with no volume as a pendulum with no lever. A
    base_link that displaces nothing is an error here, as an unmeasurable
    shape is in displacement().
    """
    link = named(sdf(models[0]), 'link', 'base_link')
    volume = 0.0
    moment = [0.0, 0.0, 0.0]

    for collision in link.findall('collision'):
        v = displacement(collision)

        if v is None:
            continue

        volume += v

        for i in range(3):
            moment[i] += v * (pose(link, i) + pose(collision, i))

    if volume <= 0.0:
        raise LookupError('%s 의 base_link 에 부피를 잴 수 있는 충돌체가 없다' % models[0])

    return volume, tuple(c / volume for c in moment)


def hang_angle(name):
    """The pitch a buoyant hull hangs at with nothing applied, rad, positive
    nose down: the pendulum of its weight under its buoyancy trims where the
    centre of mass is below the centre of volume, so a centre of mass ahead
    of it hangs the nose down by atan((x_com - x_cov) / (z_cov - z_com)).
    The Cloudship's gz files give 2.52 deg here; the engine settled at 2.61
    (cloudship_sitl.md). With the throttle off such a hull glides down its own
    axis at u tan(hang)."""
    fam = family(name)
    _, com = centre_of_mass(fam)
    _, cov = centre_of_volume(fam)
    return math.atan2(com[0] - cov[0], cov[2] - com[2])


def added_mass(model):
    """The <fluid_added_mass> elements of the link that carries one, or {}."""
    block = model.find('link/inertial/fluid_added_mass')
    return {child.tag: float(child.text) for child in block} if block is not None else {}


def product_of_inertia(models, axes='xz'):
    """Family product of inertia about its centre of mass: each link's own
    <i{axes}> (the SDF tensor element) plus the parallel-axis term -m d_a d_b.
    ixz is what couples a roll swing into the body yaw rate the gyro reads."""
    a, b = ('xyz'.index(axes[0]), 'xyz'.index(axes[1]))
    _, com = centre_of_mass(models)
    result = 0.0

    for name in models:
        for link in sdf(name).findall('link'):
            inertial = link.find('inertial')

            if inertial is None or inertial.find('mass') is None:
                continue

            mass = float(inertial.find('mass').text)
            d = [pose(link, i) + pose(inertial, i) - com[i] for i in range(3)]
            own = inertial.find('inertia/i' + axes)
            result += (float(own.text) if own is not None else 0.0) - mass * d[a] * d[b]

    return result


def inertia(models, axis):
    """Rigid inertia about `axis` (0 x, 1 y, 2 z) through the family's centre
    of mass: every link's own diagonal term plus its mass at its lever. What
    a torque about that axis has to turn, before any added mass. Three
    checks each wrote this sum for their own axis; it lives here now."""
    own = ('ixx', 'iyy', 'izz')[axis]
    total, com = centre_of_mass(models)
    result = 0.0

    for name in models:
        for link in sdf(name).findall('link'):
            inertial = link.find('inertial')

            if inertial is None or inertial.find('mass') is None:
                continue

            mass = float(inertial.find('mass').text)
            d = [pose(link, i) + pose(inertial, i) - com[i] for i in range(3)]
            lever = sum(d[i] * d[i] for i in range(3) if i != axis)
            found = inertial.find('inertia/' + own)
            result += (float(found.text) if found is not None else 0.0) + mass * lever

    return result


def yaw_inertia(models):
    return inertia(models, 2)


def hydrodynamics(model):
    """Every coefficient in the Hydrodynamics block, by name, or {} if absent."""
    plugin = model.find('.//plugin[@name="%s"]' % HYDRO_NAME)

    if plugin is None:
        return {}

    return {child.tag: float(child.text) for child in plugin
            if child.tag not in ('link_name', 'disable_added_mass', 'namespace')}


LIFT_DRAG_NUMBERS = ('area', 'cla', 'cda', 'alpha_stall', 'cla_stall', 'cda_stall',
                     'air_density', 'a0', 'control_joint_rad_to_cl')


def lift_drag_fins(model):
    """Every lift-drag plugin of a model element, in document order, as a
    dict: 'elem' the plugin, 'cp' and 'upward' as tuples, 'vertical' when
    the lift is lateral (a vertical fin, a yaw surface), 'joint' the
    control_joint_name or None, and each of LIFT_DRAG_NUMBERS by its tag,
    None where the file leaves it out.

    Three checks parsed these plugins with three different vertical-fin
    tests, one a string compare that an upward of '0 -1 0' would have
    failed silently.
    """
    fins = []

    for p in model.findall('plugin'):
        if p.get('filename') != LIFT_DRAG_FILE:
            continue

        upward = tuple(float(v) for v in p.find('upward').text.split())
        joint = p.find('control_joint_name')
        fin = {'elem': p, 'cp': tuple(float(v) for v in p.find('cp').text.split()), 'upward': upward,
               'vertical': abs(upward[1]) > 0.5, 'joint': joint.text if joint is not None else None}

        for tag in LIFT_DRAG_NUMBERS:
            found = p.find(tag)
            fin[tag] = float(found.text) if found is not None else None

        fins.append(fin)

    return fins


def servo_joints(model):
    """[(sub_topic, joint_name)] of every joint position controller that
    listens on a topic: the joints the bridge's servo outputs drive."""
    return [(p.find('sub_topic').text, p.find('joint_name').text) for p in model.findall('plugin')
            if 'joint-position-controller' in p.get('filename', '') and p.find('sub_topic') is not None]


def flap_servos(model):
    """{flap link: (axis, servo topic)} for every servo whose joint a
    lift-drag plugin names as its control joint: the link that joint moves,
    'yaw' when that plugin is a vertical fin's and 'pitch' otherwise. The
    axis comes from the fin, not the hinge, so a hinge on the wrong axis
    still reads as a wrong deflection."""
    fins = {f['joint']: f for f in lift_drag_fins(model) if f['joint']}
    out = {}

    for topic, joint in servo_joints(model):
        if joint in fins:
            link = named(model, 'joint', joint).find('child').text
            out[link] = ('yaw' if fins[joint]['vertical'] else 'pitch', topic)

    return out


def translational_drag(model):
    """|xUabsU|, |yVabsV|, |zWabsW| — the three that turn a speed into a force.

    Magnitudes: the model writes them negative because the plugin applies
    -Dmat*v, and every caller here wants the drag that opposes motion.
    """
    terms = hydrodynamics(model)

    if not terms:
        # A plant without a drag block: the burst inversions that use these
        # run with c = 0 there, which is exactly right, F * span = m * dv.
        return [0.0, 0.0, 0.0]

    return [abs(terms[name]) for name in ('xUabsU', 'yVabsV', 'zWabsW')]


def motor_plugin(model, link):
    """The motor plugin driving one link, whichever of the two forms it is."""
    for plugin in model.findall('plugin'):
        target = plugin.find('linkName')

        if target is not None and target.text == link:
            filename = plugin.get('filename', '')

            if MULTICOPTER in filename or GENERIC in plugin.get('name', ''):
                return plugin

    return None


def full_thrust(model, link):
    """(forward, reverse) thrust in newtons at full command, or None.

    Two plugin forms answer the same question two ways, and a check that knows
    only one of them reads the other as absent - which is how the tail read as
    "None x 0^2 = 0.00 N" the moment it stopped being a MulticopterMotorModel.

    The reverse half is returned because on this vehicle it is the half that
    matters: the tail is the only yaw authority and the entire reason it runs
    a GenericMotorModel is that its reverse curve has to live in the model.
    Nothing used to read negativeThrustPolynomial at all.
    """
    plugin = motor_plugin(model, link)

    if plugin is None:
        return None

    if MULTICOPTER in plugin.get('filename', ''):
        # thrust = motorConstant * maxRotVelocity^2, and the bridge never
        # sends a negative rotor speed, so there is no reverse half.
        forward = (float(plugin.find('motorConstant').text)
                   * float(plugin.find('maxRotVelocity').text) ** 2)
        return forward, 0.0

    def curve(tag, fallback):
        found = plugin.find(tag)

        if found is None:
            return fallback

        # sum at ref = 1: the polynomial is evaluated at full command
        return sum(float(c) for c in found.text.strip('[] \n').split(','))

    forward = curve('positiveThrustPolynomial', None)

    if forward is None:
        return None

    # The plugin's own fallback: an absent negative curve reuses the positive
    # one (GenericMotorModel.cpp:440), so this must too or a model that omits
    # it would read as having no reverse thrust when it has full reverse.
    return forward, curve('negativeThrustPolynomial', forward)


def duty_range(model, link):
    """(minDutyCycle, maxDutyCycle) for a bidirectional motor, or None.

    The pair the airframe's SIM_GZ_EC_MIN/MAX has to equal. One number written
    twice in two repositories; this is the half that lives in the model.
    """
    plugin = motor_plugin(model, link)

    if plugin is None or plugin.find('minDutyCycle') is None:
        return None

    return float(plugin.find('minDutyCycle').text), float(plugin.find('maxDutyCycle').text)


def turning(model):
    """{link: 'cw'|'ccw'} for every motor plugin that declares a direction.

    A counter-rotating pair cancels its reaction couple, and nothing measured
    on the hull can see that pair co-rotate at the tilts where it matters: at
    +-90 the couple leaves the roll axis entirely. The property is declared
    here, so it is asserted here.
    """
    directions = {}

    for plugin in model.findall('plugin'):
        link, direction = plugin.find('linkName'), plugin.find('turningDirection')

        if link is not None and direction is not None:
            directions[link.text] = direction.text.strip()

    return directions


def yaw_authority(name):
    """Full-command yaw moment about the family's centre of mass, N m.

    A tail thruster (prop_tail), when the model has one, is the whole of it:
    the Cloudship's pods tilt together and give no yaw, and the tail pushes
    the weaker of its two directions. Otherwise the two pods, which tilt
    independently, each at full forward thrust, pointed opposite ways, at
    their lever across the centre of mass. Pods on fixed mounts give no yaw:
    that raises LookupError rather than returning their lever's number.
    """
    model = merged(name)
    _, com = centre_of_mass(family(name))

    try:
        tail = named(model, 'link', 'prop_tail')
    except LookupError:
        # pods give yaw only when they tilt apart: on fixed mounts (airship_fixed)
        # they thrust forward together and the yaw is the flaps'
        for pod in ('pod_starboard', 'pod_port'):
            mount = next((j for j in model.iter('joint') if j.findtext('child') == pod), None)

            if mount is None or mount.get('type') == 'fixed':
                raise LookupError('%s 의 %s 는 틸트 관절에 달려 있지 않다: 포드로는 요 권한이 없다' % (name, pod))

        return sum(full_thrust(model, pod)[0] * abs(pose(named(model, 'link', pod), 1) - com[1])
                   for pod in ('prop_starboard', 'prop_port'))

    return min(full_thrust(model, 'prop_tail')) * abs(pose(tail, 0) - com[0])


# Straight flight at a speed (2026-10-01, forward_yaw_check.py): what it costs
# in thrust, what the fins and the added mass give there to first order, and
# which posix airframe flies the model. Nothing above this line changed for
# these.


def pod_thrust(name):
    """Full forward thrust of the two pods together, N: what a collective
    thrust fraction of 1 pushes with. THR_MDL_FAC 1 in the gz airframes makes
    the fraction linear in thrust. Both pods or an error: a model with one
    is some other vehicle, and a missing pod read as 0 N would put every
    trim at infinity, or at zero."""
    model = merged(name)
    pods = [full_thrust(model, link) for link in ('prop_starboard', 'prop_port')]

    if any(p is None for p in pods):
        raise LookupError('%s 에 포드 추진기 둘 (prop_starboard, prop_port) 이 없다' % name)

    return sum(p[0] for p in pods)


def added_mass_terms(name, terms):
    """{term: value} of a model's <fluid_added_mass>, for each of `terms`
    ('xx', 'yy', 'rr', ...). A plant with no added mass has none: 0.0 each,
    for a member whose name carries no _am token and whose family carries no
    block. Anything else must carry every term asked for, or LookupError: an
    _am member that lost its block or one child of it would otherwise read as
    a hull without that added mass, and the Munk moment, yy - xx, is what
    makes the complete Cloudship directionally unstable."""
    block = added_mass(merged(name))

    if not block and 'am' not in family_tokens(name):
        return {term: 0.0 for term in terms}

    missing = [term for term in terms if term not in block]

    if missing:
        raise LookupError('%s 의 fluid_added_mass 에 %s 이 없다%s' % (
            name, ', '.join(missing), ' (블록이 없다)' if not block else ''))

    return {term: block[term] for term in terms}


def surge_mass(name):
    """What a surge force accelerates, kg: the family's rigid mass plus an
    _am member's added surge mass (xx)."""
    mass, _ = centre_of_mass(family(name))
    return mass + added_mass_terms(name, ('xx',))['xx']


def trim_thrust(name, speed):
    """The collective thrust fraction that holds `speed` in straight flight:
    the hull's surge drag |xUabsU| u^2 over pod_thrust(). 0.0 on a plant with
    no drag block, and that is not an omission: the lift-drag plugin's drag is
    cda x |alpha| (fin_basis.plugin_cd), nothing at zero incidence, so a
    drag-free plant coasts at its speed with the thrust off.

    Surge drag only. The cross terms that couple surge into pitch (mUabsU)
    trim the hull a fraction of a degree, and the fins it then meets at an
    incidence add their lift-induced drag; a check that flies this trim
    reports the speed it actually holds rather than assuming this one."""
    return translational_drag(merged(name))[0] * speed * speed / pod_thrust(name)


def vertical_fins(name):
    """The lift-drag fins whose lift is lateral, in file order, each the
    lift_drag_fins() record plus 'arm', m aft of the family's centre of mass
    to its cp (the quarter chord, where the plugin reads the incidence)."""
    _, com = centre_of_mass(family(name))
    fins = []

    for f in lift_drag_fins(merged(name)):
        if not f['vertical']:
            continue

        if f['area'] is None or f['cla'] is None or f['alpha_stall'] is None:
            # the plugin has defaults for these; a coefficient built on a
            # guess of them describes some other fin
            raise LookupError('%s 의 수직 핀 (cp %s) 에 area/cla/alpha_stall 이 없다' % (name, f['cp']))

        fin = dict(f)
        fin['arm'] = com[0] - f['cp'][0]
        fins.append(fin)

    return fins


def fin_yaw_damping(name):
    """k in N = -k u r: the vertical fins' yaw-rate damping per unit forward
    speed, N m s/rad per m/s, on the plugin's attached slope. A yaw rate r
    moves a fin at `arm` sideways at r arm, an incidence r arm / u read at its
    cp, so the moment is 1/2 rho u sum(area cla arm^2) r. The engine measured
    216.9 u on the rigid vectored _fin plant against 229.8 u from this sum
    (airship_tail_design.md); the files are 6 % stiff there."""
    return 0.5 * RHO_AIR * sum(f['area'] * f['cla'] * f['arm'] ** 2 for f in vertical_fins(name))


def lateral_derivatives(name):
    """The linear sway-yaw coefficients of straight flight at speed u, about
    the family's centre of mass, FRD signs (v to starboard, r nose right):

        m_y v' = y_v u v + (y_r - m_x) u r      i_z r' = n_v u v + n_r u r + N

    each coefficient per unit u, from the files: the vertical fins on their
    attached slope (y_v = -1/2 rho sum(area cla), y_r = -y_v-weighted arm,
    n_v their weathercock, n_r = -fin_yaw_damping()), and on an _am member
    the Munk moment -(yy - xx) u v inside n_v and the added masses inside m_x,
    m_y and i_z (rr). The hull's drag block is quadratic in v and r and adds
    nothing at straight flight, which is where these hold. 'munk' is kept
    apart so a reader can see which side of zero the fins leave n_v."""
    fam = family(name)
    mass, _ = centre_of_mass(fam)
    added = added_mass_terms(name, ('xx', 'yy', 'rr'))
    fins = vertical_fins(name)
    half_rho = 0.5 * RHO_AIR
    munk = added['yy'] - added['xx']
    weathercock = half_rho * sum(f['area'] * f['cla'] * f['arm'] for f in fins)
    return {'m_x': mass + added['xx'], 'm_y': mass + added['yy'],
            'i_z': inertia(fam, 2) + added['rr'],
            'y_v': -half_rho * sum(f['area'] * f['cla'] for f in fins), 'y_r': weathercock,
            'n_v': weathercock - munk, 'n_r': -fin_yaw_damping(name), 'munk': munk}


def airframe(name):
    """(number, path) of the posix gz airframe whose PX4_SIM_MODEL default is
    this model. Exactly one, by the scheme's file name, or an error; the
    model it spawns is the file's own first PX4_SIM_MODEL default (${:=}
    keeps the first one set, and a member sets its own before sourcing the
    bare one), read by the walker without following the sources."""
    found = [(f, p) for f, p in airframe_files() if re.match(r'^\d+_gz_%s$' % re.escape(name), f)]

    if len(found) != 1:
        raise LookupError('%s 의 posix 에어프레임이 %d 개다: %s' % (name, len(found), ', '.join(f for f, _ in found) or '없음'))

    path = found[0][1]
    models = [value for kind, _, value, _, _ in airframe_settings(path, follow=False) if kind == 'model']

    if not models or models[0] != name:
        raise LookupError('%s 의 PX4_SIM_MODEL 기본값이 %s 가 아니다 (%s)' % (found[0][0], name, models[0] if models else '없음'))

    return int(found[0][0].split('_')[0]), path


def airframe_defaults(name):
    """{PARAM: value} that the model's posix airframe sets with `param
    set-default`, its sourced files included at the point they are sourced,
    so a later line wins as it does in the shell (2522 sources 2520 and then
    sets its own AS_YAWRATE_FF). Numbers as floats, anything else as written."""
    values = {}

    for kind, param, value, _, _ in airframe_settings(airframe(name)[1]):
        if kind == 'default':
            try:
                values[param] = float(value)
            except ValueError:
                values[param] = value

    return values
