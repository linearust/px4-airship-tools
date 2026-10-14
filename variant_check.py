#!/usr/bin/env python3
"""Are the physics variants what the naming scheme says they are?

    python3 variant_check.py            # needs no gz

The scheme: <vehicle>[_am][_fin][_drag], tokens in that order. The bare name
is the rigid hull with buoyancy and nothing else fluid; _am adds the hull's
fluid added mass, _fin fin lift, _drag hull drag and rotational damping.
Every variant but _am is a thin merge include of the one before it in that
order, and _am is a full copy, because <fluid_added_mass> lives in a link's
<inertial> and a merge include can only add elements. The airframe number's
ones digit is the physics mask, 1 drag + 2 fin + 4 added mass, and 9 is the
gazebo-classic twin. The decades run in the order of the flight numbers:
2500 the generic airship (its gz twin airship_fixed, 2500-2505), 2510 the Cloudship
(2510-2517, classic at 2519), 2520 the independently vectored airship
(2520-2527, sharing its flight number).

A scheme that nothing enforces is a naming convention for about a week. So
this reads every airship model in the tree and holds each fact to the file:

  1. chain: each variant's tokens are in order, and it merges exactly the
     model with one token fewer (the last one dropped); _am and the bare name
     carry links themselves.
  2. copy: cloudship_am, with its <fluid_added_mass> removed and its name put
     back, is cloudship, element for element. Comments do not count.
  3. added mass: the eight numbers are Lamb's for the collision ellipsoid the
     file itself draws, about the link origin, and the other thirteen are
     absent. Recomputed here rather than copied, like the inertia is.
  4. fins: four lift-drag plugins, element-identical (comments do not count,
     as in fact 2) between _fin and _am_fin,
     one per fin visual of the base, cp at that fin's quarter chord, upward
     normal to its thin axis, area, lift slope, induced drag, stall (on the
     plates' CLmax for a joined-panel AR inside the plate data, 2-5, and on
     the low-AR plates' for the Cloudship's fins, which are sized on them),
     post-stall slopes and flap slope (capped
     at the joint limit) all from its planform on fin_basis.
  5. drag: the Hydrodynamics block is element-identical between cloudship_drag
     and cloudship_am_fin_drag, so the coefficients aero_moment_check.py
     defends on one are the ones the other runs.
  6. buoyancy: the lta world enables base_link for every gz airframe's model,
     bare and _0.._3, because a name missing there falls with no warning.
  7. numbers: every model has its airframe at vehicle decade + mask, listed in
     the ROMFS CMakeLists, defaulting PX4_SIM_MODEL to that model, and the
     docs table names the target with that number.
  8. inventory: each model's merged family carries exactly the fluid physics
     its tokens name and nothing else: four lift-drag plugins iff _fin, one
     Hydrodynamics block iff _drag, a fluid_added_mass iff _am, and no other
     plugin that touches the fluid. Presence was facts 2-5; this is absence,
     which is what "the bare name is rigid" actually claims.
  9. nothing off-scheme: a model directory or a gz airframe whose name starts
     with a vehicle's but whose tokens are not the scheme's is reported, not
     skipped, and a variant has exactly one include.
 10. _base: a plugin-free body wrapped by at least one model and never given
     an airframe, the shape x500_base and omniquad_base have upstream.
 11. display names: a member's posix @name is its docs heading, verbatim, and
     every airship link in index.md and frames_airship/index.md resolves to a
     heading; the docs build ignores dead links, so nothing else would notice.
 12. make targets: no two gz airframes produce one gz_<model>[_<world>] target,
     the way CMake would hit it.
 13. numbers: 25xx unique within each airframe directory, and every 25xx is
     upstream's (2500, 2507) or in the decade table, which is read from the
     comment above the 25xx entries of the posix CMakeLists, the one place it
     is written.
 14. the requirement, not the transcription: for every finned model of a
     body whose header declares it statically stable (STABLE, at the ratio
     the header promises), the two vertical fins' restoring moment over the Munk
     moment of its _am block, both read from the files, is at least the
     target at zero sideslip, on the attached-flow slope (Helmbold's); the
     plugins' plate slope is a secant and is reported. Facts 3 and 4 hold the
     files to their basis; this is the one that a design which misses its
     requirement fails.
 15. flap travel: for a vehicle whose fins are its actuators, every control
     joint a fin plugin names is driven by exactly one servo topic, and the
     posix airframe's SIM_GZ_SV_MINA/MAXA for each servo equal the limits of
     the joint on that servo's topic, so the bridge's +-1 is the full travel
     and no command is lost at a stop.
 16. the allocation is the flight airframe's: each gz vehicle's bare posix
     airframe carries its flight twin's CA_* set-defaults line for line
     (TWINS), since a twin that allocates differently regresses nothing. A
     vehicle file's family keeps the three values the flight airframe tells
     the user to set to the vehicle, the tilt stops and the slew rate
     (VEHICLE_ALLOCATION), which must then be the file's own.
 17. the plain plant: once a family lives in the overlay (its bare is the
     overlay's _rigid), the tree's model under the vehicle's plain name is
     the one plant upstream carries, numbered at the family's tree decade (a
     hundred below the overlay's, mask 0), and it is the family's complete
     member: the merged models element for element, the airframes' resolved
     defaults less the parameters this tree lacks (gz_model.absent_params()).
     Facts 1, 7, 8 and 13 read it so: its chain is the complete member's
     shape, its physics the complete member's. Before the trim there is no
     plain plant, and the facts read as they always did.
"""
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

import fin_basis
import gz_model
import hull_basis
from gz_model import CMAKE_POSIX, FLIGHT_AIRFRAMES, HYDRO_FILE, LIFT_DRAG_FILE, ROOT, RHO_AIR
from report import note, summary, verdict

TOKENS = gz_model.SCHEME_TOKENS
MASK = {'drag': 1, 'fin': 2, 'am': 4}


def decades():
    """{vehicle: decade} from the two places the table is written: the comment
    above the 25xx entries of the tree's posix CMakeLists ('#   2510 cloudship',
    the vehicle) and the overlay's decade file (gz_model.OVERLAY_DECADES,
    '2630 airship_indoor_rigid', the bare model, keyed here by its vehicle),
    when it exists. A vehicle marked reserved may have no model yet; a vehicle
    in both tables is an error, as a name in both homes is."""
    table, reserved = {}, set()
    sources = [(CMAKE_POSIX, r'\s*#\s+(%d\d0)\s+([a-z0-9_]+)(.*)' % TREE_HUNDREDS)]

    if os.path.isfile(gz_model.OVERLAY_DECADES):
        sources.append((gz_model.OVERLAY_DECADES, r'\s*(%d\d0)\s+([a-z0-9_]+)(.*)' % OVERLAY_HUNDREDS))

    for path, pattern in sources:
        for line in open(path):
            m = re.match(pattern, line)

            if not m:
                continue

            vehicle = gz_model.vehicle(m.group(2))

            if vehicle in table:
                raise SystemExit('%s 가 십의 자리 표 둘에 다 있다 (%d 와 %s)' % (vehicle, table[vehicle], m.group(1)))

            table[vehicle] = int(m.group(1))

            if 'reserved' in m.group(3):
                reserved.add(vehicle)

    return table, reserved


# the hundreds of the two homes' numbers: the tree's 25xx, the overlay's 26xx
TREE_HUNDREDS = 25
OVERLAY_HUNDREDS = TREE_HUNDREDS + gz_model.OVERLAY_SHIFT // 100
DECADE, RESERVED = decades()
# the flight airframe each gz vehicle without a vehicle file is the twin of
TWINS = {'airship_fixed': 2500, 'cloudship': 2507}


# The allocation values 2520 tells the user to set to the vehicle (its
# comment: the tilt angles at the servo end stops and the servo slew rate),
# as a vehicle file declares them. A family generated from a file carries
# the file's values here and the flight airframe's everywhere else.
VEHICLE_ALLOCATION = {'CA_AIRSHIP_TLMIN': lambda v: -v['pods']['tilt_deg'],
                      'CA_AIRSHIP_TLMAX': lambda v: v['pods']['tilt_deg'],
                      'CA_AIRSHIP_TLT_R': lambda v: v['pods']['tilt_rate_deg_s']}


VEHICLE_FILES = gz_model.vehicle_files()


def allocation_matches(mine, theirs, own):
    """Whether a twin's CA_* lines `mine` are the flight airframe's `theirs`
    line for line, with the vehicle's own values `own` ({name: value}) standing
    in on their lines. Returns (same, the lines that differ)."""
    want = [(k, '%g' % own[k]) if k in own else (k, val) for k, val in theirs]
    same = len(mine) == len(want) and all(
        a == b and (float(x) == float(y) if a in own else x == y) for (a, x), (b, y) in zip(mine, want))
    return same, sorted(set(mine) ^ set(want))


def vehicle_file_twins():
    """Every vectored vehicle file's family flies the independent-vectoring
    preset 2520 in its bare member (a surfaces family declares its flaps in
    the fin members only), each with its own tilt values (VEHICLE_ALLOCATION)."""
    return {name: 2520 for name in VEHICLE_FILES}


TWINS = dict(vehicle_file_twins(), **TWINS)
DOCS = os.path.join(ROOT, 'docs/en/sim_gazebo_gz/index.md')
WORLDS = os.path.join(ROOT, 'Tools/simulation/gz/worlds')
LTA = os.path.join(WORLDS, 'lta.sdf')


def split(name):
    """(vehicle, [physics tokens]) or None when the name is not the scheme's:
    the vehicle must be in the decade table, the tokens in the scheme's order
    (gz_model.family_tokens), and the overlay's _rigid bare carries none."""
    try:
        tokens = gz_model.family_tokens(name)
    except LookupError:
        return None

    vehicle = gz_model.vehicle(name)

    if vehicle not in DECADE:
        return None

    return vehicle, [t for t in tokens if t != gz_model.RIGID]


def airframe_number(name):
    """The airframe number the scheme gives a model: its vehicle's decade
    plus the physics mask of its tokens; the tree's plain plant of an overlay
    family keeps the tree decade, a hundred below the family's."""
    vehicle, tokens = split(name)

    if is_plain(name):
        return DECADE[vehicle] - gz_model.OVERLAY_SHIFT

    return DECADE[vehicle] + sum(MASK[t] for t in tokens)


def posix_airframes(vehicle):
    """The posix gz airframe files of a vehicle's bare model (its _rigid
    model in the overlay): one, or none."""
    try:
        return [os.path.basename(gz_model.airframe(gz_model.bare(vehicle))[1])]
    except LookupError:
        return []


def body_of(vehicle):
    """The <body>_base the vehicle's bare model merges (the overlay's bare
    merges <body>_base_rigid; the name without the token is returned, as the
    _am members merge <body>_base_am either way), or None when the bare model
    is its own body (cloudship)."""
    parent = gz_model.parent(gz_model.bare(vehicle))
    rigid = '_base_' + gz_model.RIGID

    if parent and parent.endswith(rigid):
        return parent[:-len(rigid)] + '_base'

    return parent if parent and parent.endswith('_base') else None


def plain(vehicle):
    """The tree's plain plant of a vehicle whose family lives in the overlay:
    the vehicle's own name, a tree model beside the overlay's _rigid bare.
    None while the plain name is the bare (the tree before the trim)."""
    return vehicle if gz_model.bare(vehicle) != vehicle and gz_model.has_model(vehicle) else None


def is_plain(name):
    found = split(name)
    return bool(found) and not found[1] and plain(found[0]) == name


def complete(vehicle, names):
    """The family member carrying every physics token the family has: the
    overlay's _am_fin_drag, the fixed twin's _am_drag."""
    members = [n for n in names if split(n)[0] == vehicle and not is_plain(n)]
    return max(members, key=lambda n: sum(MASK[t] for t in split(n)[1]))


def actuator_fins(model):
    """The lift-drag plugins that name a control joint: a vehicle whose
    fins are its actuators (airship_fixed) carries them in its bare model."""
    return [p for p in model.findall('plugin')
            if p.get('filename') == LIFT_DRAG_FILE and p.find('control_joint_name') is not None]


def has_actuator_fins(vehicle):
    return bool(actuator_fins(gz_model.merged(gz_model.bare(vehicle))))


def models():
    """Every airship model directory, by the scheme's vehicles."""
    return [d for d in gz_model.model_names() if split(d)]


def is_body(name):
    """A plugin-free body or its added-mass copy: <body>_base, <body>_base_am,
    and the overlay's <body>_base_rigid."""
    return name.endswith(('_base', '_base_am', '_base_' + gz_model.RIGID))


def off_scheme():
    """Names that begin with a vehicle's and are not the scheme's: a model
    directory or a gz airframe. Reported by fact 9, since a directory that
    split() rejects would otherwise vanish from every other fact."""
    found = []

    for d in gz_model.model_names():
        if is_body(d):
            continue                                   # a body, judged by fact 10

        if any(d.startswith(v + '_') for v in DECADE) and not split(d):
            found.append('models/' + d)

    for f, _ in gz_model.airframe_files():
        if '_gz_' in f:
            model = f.split('_gz_', 1)[1]

            if any(model.startswith(v + '_') for v in DECADE) and not split(model):
                found.append('airframes/' + f)

    return found


def slug(heading):
    """VitePress's anchor for a heading: lowercase, punctuation dropped,
    spaces to hyphens."""
    text = re.sub(r'[^a-z0-9 -]', '', heading.lower())
    return re.sub(r'-+', '-', text.strip().replace(' ', '-'))


def canonical(elem):
    """The element as text with whitespace-only text dropped, for equality."""
    for e in elem.iter():
        if e.text is not None and not e.text.strip():
            e.text = None

        if e.tail is not None and not e.tail.strip():
            e.tail = None

    return ET.tostring(elem, encoding='unicode')


def expected_added_mass(base):
    """The eight elements from the hull ellipsoid: the collision's, which is
    also what both bodies draw (a hull whose collision were not an ellipsoid
    would fall back to its visual). h is the collision's pose."""
    link = base.find('link')
    hull = gz_model.named(link, 'collision', 'hull_collision')
    radii = hull.find('geometry/ellipsoid/radii')

    if radii is None:
        radii = gz_model.named(link, 'visual', 'hull_visual').find('geometry/ellipsoid/radii')

    a, b, c = [float(v) for v in radii.text.split()]
    return hull_basis.added_mass_block(a, b, gz_model.pose(hull, 2), RHO_AIR * 4.0 / 3.0 * math.pi * a * b * c)


def hull_axis_z(base):
    """Height of the hull axis above the link origin: the hull collision's pose."""
    return gz_model.pose(gz_model.named(base.find('link'), 'collision', 'hull_collision'), 2)


def stable_bodies():
    """Bodies whose header declares static directional stability, and the
    ratio each promises: every *_base model (the overlay's *_base_rigid,
    keyed without the token as body_of() names it) whose header says the
    fins 'restore at least R x the hull's potential-flow Munk moment'
    (generic_hull_design's requirement R1, written by apply_generic_hull).
    The Cloudship's fins restore about half its Munk moment by the drawing
    and its header makes no such promise."""
    out = {}
    rigid = '_base_' + gz_model.RIGID

    for name in gz_model.model_names():
        path = os.path.join(gz_model.model_dir(name), 'model.sdf')

        if not name.endswith(('_base', rigid)):
            continue

        found = re.search(r"restore\s+at\s+least\s+([\d.]+)\s+x\s+the\s+hull's\s+potential-flow\s+Munk\s+moment", open(path).read())

        if found:
            out[name[:-len(rigid)] + '_base' if name.endswith(rigid) else name] = float(found.group(1))

    return out


STABLE = stable_bodies()


def fin_expectations(base):
    """{cp: (upward, area, cla, chord, root, span)} from the four fin
    visuals, on the family's one basis (fin_basis.py): the plate slope on
    the two panels joined through the hull, times the slender-body carryover
    (1 + r/s)^2 with r the root radius the visual implies (its radial centre
    minus half its span) and s the root plus the span."""
    out = {}
    z_axis = hull_axis_z(base)

    for visual in base.find('link').findall('visual'):
        if not visual.get('name', '').startswith('fin_'):
            continue

        size = [float(v) for v in visual.find('geometry/box/size').text.split()]
        chord, span, root = fin_basis.panel(size, gz_model.pose(visual, 1), gz_model.pose(visual, 2), z_axis)
        thin = size.index(min(size))
        cp = (gz_model.pose(visual, 0) + chord / 4, gz_model.pose(visual, 1), gz_model.pose(visual, 2))
        upward = tuple(1.0 if i == thin else 0.0 for i in range(3))
        out[tuple(round(v, 5) for v in cp)] = (upward, chord * span, fin_basis.cla(chord, span, root), chord, root, span)

    return out


def main():
    names = models()
    note('모델 %d 개: %s' % (len(names), ', '.join(names)))

    # 1. chain
    bad = []

    for name in names:
        vehicle, tokens = split(name)
        parent = gz_model.parent(name)

        body = body_of(vehicle)

        if not tokens:
            # the bare model: its own body, or the family's _base (_base_rigid under the overlay's rigid bare)
            want = body + '_' + gz_model.RIGID if body and name.endswith('_' + gz_model.RIGID) else body
        elif tokens == ['am']:
            want = body + '_am' if body else None   # a copy of a self-contained body, or a wrapper of <body>_base_am
        elif len(tokens) == 1:
            want = gz_model.bare(vehicle)           # _fin or _drag wraps the bare model (_rigid in the overlay)
        else:
            want = '_'.join([vehicle] + tokens[:-1])

        if parent != want:
            bad.append('%s 은 %s 를 감싸야 하는데 %s' % (name, want, parent))

    verdict('변형의 포함 사슬이 규칙대로다', not bad, ', '.join(bad) if bad else '%d 개 모델' % len(names))

    # 2. copies: every body that has an _am is that body plus the block. A
    #    self-contained vehicle's _am is a copy of the vehicle (cloudship); a
    #    _base family's is <body>_base_am, and the vehicle's _am member then
    #    restates its propulsion block on it, held equal to the bare model's.
    vehicles = sorted({split(n)[0] for n in names})
    copies = []

    for v in vehicles:
        body = body_of(v)
        # the body the bare merges (its _rigid form in the overlay) against its _am copy; a self-contained vehicle's bare itself
        copies.append((gz_model.parent(gz_model.bare(v)), body + '_am') if body else (gz_model.bare(v), v + '_am'))

    copies = sorted(set(c for c in copies if gz_model.has_model(c[1])))
    bad, carriers = [], {}

    for original, copy in copies:
        base = gz_model.sdf(original)
        am = gz_model.sdf(copy)
        owners = [link.get('name') for link in am.findall('link') if link.find('inertial/fluid_added_mass') is not None]

        if owners != ['base_link']:
            bad.append('%s: fluid_added_mass 를 가진 링크 %s' % (copy, ', '.join(owners) or '없음'))
            continue

        stripped = ET.fromstring(ET.tostring(am))
        inertial = stripped.find('link/inertial')
        inertial.remove(inertial.find('fluid_added_mass'))
        stripped.set('name', original)

        if canonical(stripped) != canonical(ET.fromstring(ET.tostring(base))):
            bad.append('%s 은 %s 에 블록만 더한 것이 아니다' % (copy, original))

        carriers[copy] = (base, am.find('link/inertial/fluid_added_mass'))

    def propulsion(name, vehicle):
        m = ET.fromstring(ET.tostring(gz_model.sdf(name)))

        for inc in m.findall('include'):
            m.remove(inc)

        m.set('name', vehicle)
        return canonical(m)

    for v in vehicles:
        if body_of(v) and gz_model.has_model(v + '_am'):
            if propulsion(gz_model.bare(v), v) != propulsion(v + '_am', v):
                bad.append('%s_am 의 추진 블록이 %s 의 것과 다르다' % (v, v))

    verdict('_am 은 몸통에 base_link 의 fluid_added_mass 만 더한 사본이고, 감싸는 _am 은 추진 블록을 그대로 되풀이한다',
            not bad, ', '.join(bad) if bad else ', '.join('%s -> %s' % c for c in copies))

    # 3. added mass values, per carrier, from the hull it draws
    bad = []

    for copy, (base, block) in carriers.items():
        want = expected_added_mass(base)
        got = {child.tag: float(child.text) for child in block}
        off = ['%s %s (기대 %.5f)' % (k, got.get(k), v) for k, v in want.items()
               if k not in got or abs(got[k] - v) > 1e-4 * max(1.0, abs(v))]
        extra = [k for k, v in got.items() if k not in want and abs(v) > 0]

        if off or extra:
            bad.append('%s: %s' % (copy, ', '.join(off + extra)))

    verdict('부가 질량은 그 선체 타원체에 대한 Lamb 값이고 링크 원점 기준이다', not bad,
            '; '.join(bad) if bad else ', '.join('%s rr %.3f' % (c, float(b.find('rr').text)) for c, (_, b) in carriers.items()))

    # 4. fins, per vehicle: the fin plugins of a vehicle's fin-bearing models
    #    (_fin and _am_fin, or the bare model and _am when the fins are its
    #    actuators) are identical, one per fin visual of the body, at that
    #    fin's quarter chord, with the family's one lift basis.
    bad, bits, skipped, low = [], [], set(), set()

    for v in vehicles:
        if has_actuator_fins(v):
            bearers = [gz_model.bare(v)] + ([v + '_am'] if gz_model.has_model(v + '_am') else [])
        else:
            bearers = [n for n in names if split(n)[0] == v and split(n)[1][-1:] == ['fin']]   # the block lives where the token is last

        if not bearers:
            continue

        fins = fin_expectations(gz_model.merged(gz_model.bare(v)))
        blocks = {}

        for name in bearers:
            blocks[name] = [canonical(ET.fromstring(ET.tostring(f['elem']))) for f in gz_model.lift_drag_fins(gz_model.sdf(name))]

        first = blocks[bearers[0]]

        if any(blocks[n] != first for n in bearers) or len(first) != 4:
            bad.append('%s: 핀 블록이 %s 에서 같지 않다' % (v, ', '.join(bearers)))

        seen = set()
        merged = gz_model.merged(bearers[0])

        for f in gz_model.lift_drag_fins(gz_model.sdf(bearers[0])):
            p = f['elem']
            cp = tuple(round(x, 5) for x in f['cp'])
            seen.add(cp)

            if cp not in fins:
                bad.append('%s: cp %s 는 어느 핀의 1/4 시위도 아니다' % (v, cp))
                continue

            up, area, cla = f['upward'], f['area'], f['cla']
            w_up, w_area, w_cla, chord, root, span = fins[cp]

            if up != w_up or abs(area - w_area) > 1e-4 or abs(cla - w_cla) > 2e-3:
                bad.append('%s: cp %s upward %s area %.5f cla %.3f (기대 %s %.5f %.3f)' % (v, cp, up, area, cla, w_up, w_area, w_cla))

            # drag, stall and post-stall on the same basis: induced cda at
            # the lower of 10 deg and the stall, the plates' CLmax over the
            # wing slope (the AR 2-5 plate data, and the low-AR plates' for
            # the Cloudship), and cl to 0 / cd to a plate's 1.2 at 90 deg
            ar = fin_basis.joined_ar(chord, span)
            cda, a_s = f['cda'], f['alpha_stall']
            w_cda = fin_basis.induced_cda(w_cla, ar, a_s)
            w_cls, w_cds = fin_basis.post_stall(cla, cda, a_s)
            got = (f['cla_stall'], f['cda_stall'])

            if abs(cda - w_cda) > 2e-3 * max(1.0, w_cda) or abs(got[0] - w_cls) > 2e-3 or abs(got[1] - w_cds) > 2e-3:
                bad.append('%s: cp %s cda %.4f 실속 뒤 %.4f/%.4f (기대 %.4f, %.4f/%.4f)' % (v, cp, cda, got[0], got[1], w_cda, w_cls, w_cds))

            # the low-AR CLmax is sourced for the Cloudship's plates alone (the fin
            # the writer sizes on it); any other fin outside AR 2-5 is skipped
            clmax = (fin_basis.CLMAX_PLATE if 2.0 <= ar <= 5.0
                     else fin_basis.CLMAX_LOW_AR_PLATE if v == 'cloudship' else None)

            if clmax is None:
                skipped.add('%s AR_j %.2f' % (v, ar))       # past the plate data: the file states its own stall basis
            else:
                w_as = fin_basis.stall(ar, fin_basis.k_wb(root, root + span), clmax=clmax)

                if ar < 2.0:
                    low.add('%s AR_j %.2f' % (v, ar))

                if abs(a_s - w_as) > 2e-3:
                    bad.append('%s: cp %s alpha_stall %.4f (기대 %.4f, 판 CLmax %.2f / (날개 기울기 x K_W(B)))' % (v, cp, a_s, w_as, clmax))

            if p.find('forward').text.split() != ['1', '0', '0'] or p.find('link_name').text != 'base_link':
                bad.append('%s: cp %s 의 forward/link_name 이 선체의 것이 아니다' % (v, cp))

            # the air the engine uses and the zero-lift angle: a fin in 1/10
            # of the air, or with an offset a0, meets every basis above
            dens, a0 = p.find('air_density'), p.find('a0')

            if dens is None or abs(float(dens.text) - gz_model.RHO_AIR) > 1e-6 or a0 is None or float(a0.text) != 0.0:
                bad.append('%s: cp %s air_density %s a0 %s (기대 %.3f, 0)' % (v, cp, None if dens is None else dens.text,
                                                                          None if a0 is None else a0.text, gz_model.RHO_AIR))

            joint = p.find('control_joint_name')

            if joint is not None:
                # gz_model.named() raises on a missing name, so look first:
                # a plugin naming a joint or flap link that is not there is a
                # failed fact, not a traceback
                flap = next((e for e in merged.findall('link') if e.get('name') == joint.text.replace('_joint', '')), None)
                hinge = next((e for e in merged.findall('joint') if e.get('name') == joint.text), None)

                if flap is None or hinge is None:
                    bad.append('%s: 조종 조인트 %s 또는 그 플랩 링크가 없다' % (v, joint.text))
                    continue

                # the flap's chord fraction from its link's visual, its slope
                # from the family basis: tau x viscous x wing slope x K_W(B)
                flap_chord = float(flap.find('visual/geometry/box/size').text.split()[0])
                upper = hinge.find('axis/limit/upper')
                w_slope = fin_basis.flap_slope(chord, span, root, flap_chord / chord,
                                               delta_max=float(upper.text) if upper is not None else None)
                slope = abs(float(p.find('control_joint_rad_to_cl').text))

                if abs(slope - w_slope) > 2e-3:
                    bad.append('%s: %s 플랩 기울기 %.3f (기대 %.3f, 시위비 %.2f)' % (v, joint.text, slope, w_slope, flap_chord / chord))

        if seen != set(fins):
            bad.append('%s: 핀 %d 장 중 플러그인이 있는 것 %d 장' % (v, len(fins), len(seen & set(fins))))

        bits.append('%s %s%s' % (v, '/'.join(bearers), ' (조종면)' if has_actuator_fins(v) else ''))

    verdict('핀 플러그인이 기체마다 같고 네 핀 각각의 1/4 시위에 하나씩, 한 가지 양력 기준으로 있다', not bad,
            ', '.join(bad) if bad else '; '.join(bits)
            + ('; 저종횡비 판 CLmax %.2f 로 실속각: %s' % (fin_basis.CLMAX_LOW_AR_PLATE, ', '.join(sorted(low))) if low else '')
            + ('; 판 자료 밖이라 실속각은 파일 자신의 근거: %s' % ', '.join(sorted(skipped)) if skipped else ''))

    # 5. drag: every _drag member of one hull carries the same block, since
    #    the hull is what the block describes.
    groups = {}

    for n in names:
        v, tokens = split(n)

        if 'drag' in tokens:
            groups.setdefault(body_of(v) or v, []).append(n)

    bad, bits = [], []

    for hull, members in sorted(groups.items()):
        blocks = {}

        for n in members:
            plugins = [p for p in gz_model.sdf(n).findall('plugin') if p.get('filename') == HYDRO_FILE]
            blocks[n] = [canonical(ET.fromstring(ET.tostring(p))) for p in plugins]

        first = blocks[members[0]]

        if len(first) != 1 or any(blocks[n] != first for n in members):
            bad.append('%s: %s' % (hull, ', '.join('%s %d' % (n, len(blocks[n])) for n in members)))

        bits.append('%s x%d' % (hull, len(members)))

    verdict('항력 블록이 선체마다 그 선체의 모든 _drag 구성원에서 같다', not bad, ', '.join(bad) if bad else ', '.join(bits))

    # 6. buoyancy, and 7. numbers, over every gz airframe. A tree member is
    #    listed in the tree's world, CMake and docs table; an overlay member
    #    in the overlay's world, decade file and vehicle list (same shapes)
    enabled = {e.text for e in ET.parse(LTA).getroot().iter('enable')}
    enabled_overlay = ({e.text for e in ET.parse(gz_model.OVERLAY_WORLD).getroot().iter('enable')}
                       if os.path.isfile(gz_model.OVERLAY_WORLD) else set())
    cmake = open(CMAKE_POSIX).read()
    docs = open(DOCS).read()
    docs_overlay = open(gz_model.OVERLAY_DOCS).read() if os.path.isfile(gz_model.OVERLAY_DOCS) else ''
    decade_lines = open(gz_model.OVERLAY_DECADES).read() if os.path.isfile(gz_model.OVERLAY_DECADES) else ''
    missing, wrong = [], []

    for name in names:
        number = airframe_number(name)
        airframe = '%d_gz_%s' % (number, name)

        try:
            path = gz_model.airframe_path(airframe)
        except LookupError:
            wrong.append('%s 없음' % airframe)
            continue

        overlay = gz_model.in_overlay(path)

        for spawn in [name] + ['%s_%d' % (name, i) for i in range(4)]:
            if '%s::base_link' % spawn not in (enabled_overlay if overlay else enabled):
                missing.append(spawn)

        text = open(path).read()

        if 'PX4_SIM_MODEL=${PX4_SIM_MODEL:=%s}' % name not in text:
            wrong.append('%s 의 PX4_SIM_MODEL 기본값이 %s 가 아니다' % (airframe, name))

        if overlay:
            if not re.search(r'^\s*%d\s+%s\b' % (number // 10 * 10, re.escape(gz_model.bare(split(name)[0]))), decade_lines, re.M):
                wrong.append('%s 의 십의 자리가 오버레이 decades 파일에 없다' % airframe)

        elif '\t%s\n' % airframe not in cmake:
            wrong.append('%s 가 CMakeLists 에 없다' % airframe)

        # the docs table names the target with that number; the overlay's vehicle list names the model (no make target)
        row = (r'`gz_%s`\s*\|\s*%d\s*\|' if overlay else r'`make px4_sitl gz_%s`\s*\|\s*%d\s*\|') % (re.escape(name), number)

        if not re.search(row, docs_overlay if overlay else docs):
            wrong.append('%s 표에 gz_%s = %d 가 없다' % ('오버레이 vehicles.md' if overlay else 'docs', name, number))

    others = sorted(f for f, _ in gz_model.airframe_files() if '_gz_' in f and split(f.split('_gz_', 1)[1]) and
                    f not in ['%d_gz_%s' % (airframe_number(n), n) for n in names])
    verdict('lta 월드가 모든 기체를 bare 와 _0.._3 로 띄운다', not missing, ', '.join(missing) if missing else '%d 항목' % (5 * len(names)))
    verdict('에어프레임 번호가 십의 자리 = 기체, 일의 자리 = 물리 마스크이고 CMake 와 docs 가 따른다',
            not wrong and not others,
            ', '.join(wrong + ['규칙 밖의 파일 %s' % o for o in others]) if wrong or others else
            ', '.join('%d=%s' % (airframe_number(n), n) for n in names))
    # 8. inventory: the physics each model carries is exactly what its tokens
    #    name. Facts 2-5 established presence; this is absence.
    FLUID = {LIFT_DRAG_FILE: 'fin', HYDRO_FILE: 'drag'}
    wrong = []

    for name in names:
        vehicle, tokens = split(name)
        merged = gz_model.merged(name)
        counts = {}

        if is_plain(name):
            tokens = split(complete(vehicle, names))[1]     # the plain plant carries what the complete member carries

        for p in merged.findall('plugin'):
            f = p.get('filename', '')

            if f in FLUID or 'buoyancy' in f or 'wind' in f or 'aero' in f.lower() or 'lift' in f or 'drag' in f or 'hydro' in f:
                counts[f] = counts.get(f, 0) + 1

        want = {}

        if 'fin' in tokens or has_actuator_fins(vehicle):
            want[LIFT_DRAG_FILE] = 4

        if 'drag' in tokens:
            want[HYDRO_FILE] = 1

        has_am = merged.find('link/inertial/fluid_added_mass') is not None

        if has_actuator_fins(vehicle) and 'fin' in tokens:
            wrong.append('%s: 핀이 조종면인 기체에 _fin 구성원이 있다' % name)

        if counts != want or has_am != ('am' in tokens):
            wrong.append('%s: 플러그인 %s (기대 %s), 부가 질량 %s (기대 %s)'
                         % (name, {k.replace('gz-sim-', '').replace('-system', ''): v for k, v in counts.items()} or '없음',
                            {k.replace('gz-sim-', '').replace('-system', ''): v for k, v in want.items()} or '없음',
                            '있음' if has_am else '없음', '있음' if 'am' in tokens else '없음'))

    verdict('각 모델은 토큰이 말하는 유체 물리만 갖는다 (없어야 할 것이 없다)', not wrong,
            ', '.join(wrong) if wrong else '%d 개 모델의 목록이 토큰과 일치' % len(names))

    # 9. nothing off-scheme, and one include per variant
    stray = off_scheme()
    multi = ['%s: include %d 개' % (n, len(gz_model.includes(n))) for n in names
             if len(gz_model.includes(n)) > (0 if gz_model.parent(n) is None else 1)]
    verdict('규칙 밖의 모델이나 에어프레임이 없고 변형마다 include 는 하나다', not stray and not multi,
            ', '.join(stray + multi) if stray or multi else '%d 개 모델, %d 개 에어프레임 검사' % (len(names), len(names)))

    # 10. _base: a plugin-free body wrapped by at least one model, never an
    #     airframe (x500_base, omniquad_base: upstream's own shape).
    all_dirs = gz_model.model_names()
    includers = {}

    for d in all_dirs:
        try:
            model = gz_model.sdf(d)
        except ET.ParseError:
            continue                       # an upstream file with a namespace prefix ET will not parse

        if model is None:
            continue                       # upstream's Fuel stubs have no <model> of their own

        for inc in model.findall('include'):
            uri = inc.find('uri')

            if uri is not None:
                includers.setdefault(uri.text.replace('model://', ''), []).append(d)

    # a body serves the vehicles its prefix names: airship_base the airship_*
    # vehicles, <vehicle>_base (a generated family) that vehicle itself
    bases = [d for d in all_dirs if is_body(d)
             and any(v == d.split('_base')[0] or v.startswith(d.split('_base')[0] + '_') for v in DECADE)]
    bad = []

    for b in bases:
        if not includers.get(b):
            bad.append('%s 를 감싸는 모델이 없다' % b)

        if any(f.endswith('_gz_' + b) for f, _ in gz_model.airframe_files()):
            bad.append('%s 에 에어프레임이 있다' % b)

        if gz_model.sdf(b).findall('plugin'):
            bad.append('%s 에 플러그인이 있다' % b)

    verdict('_base 는 플러그인 없는 몸통이고 감싸는 모델이 있으며 에어프레임이 없다', not bad,
            ', '.join(bad) if bad else ', '.join('%s <- %s' % (b, ','.join(includers[b])) for b in bases) or '_base 없음')

    # 11. display names: posix @name is the docs heading of the same target,
    #     and every airship link in the docs resolves to a heading. The docs
    #     build ignores dead links, so nothing else would notice.
    vehicles_md = open(os.path.join(ROOT, 'docs/en/sim_gazebo_gz/vehicles.md')).read()
    headings = {slug(h): h for h in re.findall(r'^#{2,3} (.+)$', vehicles_md, re.M)}
    bad = []

    for name in names:
        try:
            path = gz_model.airframe_path('%d_gz_%s' % (airframe_number(name), name))
        except LookupError:
            continue

        m = re.search(r'^# @name (.+)$', open(path).read(), re.M)
        at = m.group(1).strip() if m else None
        overlay = gz_model.in_overlay(path)

        # an overlay member's heading is in the overlay's vehicle list, which has no index.md row
        if at is None or ('### ' + at) not in (docs_overlay if overlay else vehicles_md):
            bad.append('%s 의 @name %r 이 %s 제목에 없다' % (os.path.basename(path), at, '오버레이 vehicles.md' if overlay else 'vehicles.md'))
        elif not overlay and not re.search(r'vehicles\.md#%s\)[^\n]*`make px4_sitl gz_%s`' % (re.escape(slug(at)), re.escape(name)), docs):
            bad.append('index.md 의 gz_%s 행이 #%s 를 가리키지 않는다' % (name, slug(at)))

    for doc in ('docs/en/sim_gazebo_gz/index.md', 'docs/en/frames_airship/index.md'):
        text = open(os.path.join(ROOT, doc)).read()

        for anchor in re.findall(r'vehicles\.md#([a-z0-9-]+)', text):
            if anchor not in headings:
                bad.append('%s 가 vehicles.md#%s 를 가리키는데 그런 제목이 없다' % (doc.split('/')[-2], anchor))

    verdict('posix @name 은 docs 제목이고 docs 의 링크는 모두 제목에 닿는다', not bad,
            '; '.join(bad) if bad else '%d 개 @name, %d 개 링크' % (len(names), len(re.findall(r'vehicles\.md#', docs))))

    # 12. make targets: two gz airframes may not produce one target, the way
    #     CMake would hit it (gz_<model>[_<world>] joined by _, never parsed).
    worlds = [w[:-4] for w in os.listdir(WORLDS) if w.endswith('.sdf')]
    # the tree's members only: an overlay member has no make target (gz_run.sh spawns it)
    models_all = [f.split('_gz_', 1)[1] for f, p in gz_model.airframe_files() if '_gz_' in f and not gz_model.in_overlay(p)]
    targets = {}

    for m in models_all:
        for w in worlds:
            t = 'gz_%s' % m if w == 'default' else 'gz_%s_%s' % (m, w)
            targets.setdefault(t, []).append(m)

    clashes = ['%s <- %s' % (t, ', '.join(ms)) for t, ms in targets.items() if len(ms) > 1]
    verdict('make 타깃이 겹치지 않는다', not clashes, '; '.join(clashes) if clashes else '%d 타깃' % len(targets))

    # 13. numbers: 25xx (tree) and 26xx (overlay) unique over the posix homes
    #     and within the flight directory; a flight 25xx is upstream's 2500 or
    #     2507 or a decade in the table, and a posix number is in a table
    #     decade, with the classic twin at x9.
    bad = []

    for files, label in (([f for f, _ in gz_model.airframe_files()], 'posix'), (os.listdir(FLIGHT_AIRFRAMES), 'flight')):
        nums = {}

        for f in files:
            m = re.match(r'(%d\d\d|%d\d\d)_' % (TREE_HUNDREDS, OVERLAY_HUNDREDS), f)

            if m:
                nums.setdefault(int(m.group(1)), []).append(f)

        for n, fs in sorted(nums.items()):
            if len(fs) > 1:
                bad.append('%s %d 중복: %s' % (label, n, ', '.join(fs)))

            if label == 'flight' and n in (2500, 2507):
                continue                        # upstream's numbers, not a decade of their own

            # a tree decade of an overlay family is the family's less the shift (the plain plant's, the classic twin's)
            in_table = lambda decade: decade in DECADE.values() or decade + gz_model.OVERLAY_SHIFT in DECADE.values()

            if label == 'flight' and not in_table(n):
                bad.append('flight %d 는 어느 십의 자리 표에도 없다' % n)

            if label == 'posix':
                classic = any('_gazebo-classic_' in f for f in fs)

                if not in_table(n // 10 * 10):
                    bad.append('posix %d 는 표의 십의 자리 밖이다' % n)
                elif classic and n % 10 != 9:
                    bad.append('posix %d 는 classic 쌍둥이인데 일의 자리가 9 가 아니다' % n)
                elif not classic and n % 10 not in (0, 1, 2, 3, 4, 5, 6, 7):
                    bad.append('posix %d 의 일의 자리가 마스크 범위 밖이다' % n)

    verdict('번호 (트리 25xx, 오버레이 26xx) 가 디렉터리 안에서 유일하고 전부 표 안에 있다', not bad,
            '; '.join(bad) if bad else '표: %s' % ', '.join('%d=%s%s' % (v, k, ' (예약)' if k in RESERVED else '') for k, v in sorted(DECADE.items(), key=lambda kv: kv[1])))
    # 14. the requirement: fins over Munk, from the files, for every finned
    #     vehicle of a body the design declares stable. Judged at zero
    #     sideslip on the attached-flow slope (Helmbold's); the plate slope the
    #     plugins carry is a secant with vortex lift in it and is reported.
    bad, bits = [], []

    for body, target in STABLE.items():
        amms = [v for v in vehicles if body_of(v) == body and gz_model.has_model(v + '_am')]
        finned = sorted({n for n in names if body_of(split(n)[0]) == body
                         and (split(n)[1][-1:] == ['fin'] or (has_actuator_fins(split(n)[0]) and not split(n)[1]))})

        if not amms or not finned:
            bad.append('%s: _am 이나 핀 모델이 없다' % body)
            continue

        block = gz_model.added_mass(gz_model.merged(amms[0] + '_am'))
        munk = block['yy'] - block['xx']

        for model in finned:
            mass, com = gz_model.centre_of_mass(gz_model.family(model))
            fins = fin_expectations(gz_model.merged(gz_model.bare(split(model)[0])))
            ks = kh = 0.0
            vertical = 0

            for f in gz_model.lift_drag_fins(gz_model.merged(model)):
                if not f['vertical']:
                    continue                                  # the horizontal pair: pitch

                cp = tuple(round(x, 5) for x in f['cp'])
                _, _, _, chord, root, span = fins[cp]
                area, arm = f['area'], abs(cp[0] - com[0])
                rho = f['air_density']                        # the engine's air, not an assumed one
                ks += 0.5 * rho * area * f['cla'] * arm
                kh += 0.5 * rho * area * fin_basis.cla(chord, span, root, fin_basis.helmbold) * arm
                vertical += 1

            ok = vertical == 2 and kh / munk >= target
            (bits if ok else bad).append('%s: 수직 핀 %d 장, Helmbold %.2f / Munk %.2f = %.3f (목표 %.1f), 판 기울기 %.3f'
                                         % (model, vertical, kh, munk, kh / munk, target, ks / munk))

    verdict('정적 안정을 약속한 몸통은 그 요구를 파일에서 충족한다 (핀 복원 / Munk, 영 옆미끄럼)', not bad, '; '.join(bad or bits))

    # 15. flap travel: SIM_GZ_SV_MINA/MAXA against the flap joints
    bad, bits = [], []

    for v in vehicles:
        if not has_actuator_fins(v):
            continue

        airframe = next(iter(posix_airframes(v)), None)

        if airframe is None:
            bad.append('%s: 에어프레임이 없다' % v)
            continue

        text = open(gz_model.airframe_path(airframe)).read()
        params = {k: float(x) for k, x in re.findall(r'^param set-default (SIM_GZ_SV_\w+) (-?[\d.]+)', text, re.M)}
        compared = 0

        for model in [gz_model.bare(v)] + ([v + '_am'] if gz_model.has_model(v + '_am') else []):
            merged = gz_model.merged(model)
            pairs = gz_model.servo_joints(merged)
            # every control joint a fin names, each driven by one servo topic
            # of its own: with no servo on it, or two, nothing below compares it
            want = sorted(p.find('control_joint_name').text for p in actuator_fins(merged))
            driven = sorted(j for _, j in pairs if j in want)

            if driven != want or len({topic for topic, _ in pairs}) != len(pairs):
                bad.append('%s: 서보가 모는 조종면 조인트 %s, 조종면 조인트 %s, 서보 토픽 %s'
                           % (model, driven, want, sorted(topic for topic, _ in pairs)))

            for topic, joint in sorted(pairs):
                n = int(topic.split('_')[-1]) + 1
                j = next((e for e in merged.findall('joint') if e.get('name') == joint), None)

                if j is None:
                    bad.append('%s: %s 가 가리키는 조인트 %s 가 없다' % (model, topic, joint))
                    continue

                lo, hi = (math.degrees(float(j.find('axis/limit/%s' % k).text)) for k in ('lower', 'upper'))
                mina, maxa = params.get('SIM_GZ_SV_MINA%d' % n), params.get('SIM_GZ_SV_MAXA%d' % n)

                if mina is None or maxa is None or abs(mina - lo) > 0.05 or abs(maxa - hi) > 0.05:
                    bad.append('%s %s: SIM_GZ_SV_MINA/MAXA%d %s/%s, 조인트 %.2f/%.2f deg' % (model, joint, n, mina, maxa, lo, hi))

                compared += 1

        bits.append('%s: 조인트 %d 개와 비교, %s' % (v, compared, ', '.join('%s +-%.0f' % (k[-5:], p) for k, p in sorted(params.items()) if 'MAXA' in k)))

    verdict('조종면 서보 범위가 플랩 조인트 한계와 같다', not bad, '; '.join(bad or bits))

    # 16. the twin's allocation block is the flight airframe's
    bad, bits = [], []
    allocation = lambda path: re.findall(r'^param set-default (CA_\w+) (\S+)', open(path).read(), re.M)

    for v in vehicles:
        if v not in TWINS:
            bad.append('%s: TWINS 에 비행 쪽 짝이 없다' % v)
            continue

        posix = posix_airframes(v)
        flight = [f for f in os.listdir(FLIGHT_AIRFRAMES) if f.startswith('%d_' % TWINS[v])]

        if len(posix) != 1 or len(flight) != 1:
            bad.append('%s: posix %s, 비행 %s' % (v, posix, flight))
            continue

        mine, theirs = allocation(gz_model.airframe_path(posix[0])), allocation(os.path.join(FLIGHT_AIRFRAMES, flight[0]))
        # the file's own values stand in for the flight airframe's on the
        # lines the flight airframe leaves to the vehicle; everything else,
        # order included, is the flight airframe's
        own = {k: f(VEHICLE_FILES[v]) for k, f in VEHICLE_ALLOCATION.items()} if v in VEHICLE_FILES else {}
        same, extra = allocation_matches(mine, theirs, own)

        if not theirs or not same:
            bad.append('%s -> %s: %s' % (posix[0], flight[0], ', '.join('%s %s' % kv for kv in extra) or '순서가 다르다'))
        else:
            bits.append('%s = %s (%d 줄%s)' % (posix[0], flight[0], len(mine),
                                              '; 기체 파일의 %s' % ' '.join('%s %g' % kv for kv in sorted(own.items())) if own else ''))

    verdict('gz 쌍둥이의 할당 블록이 비행 에어프레임의 것이다 (기체 파일의 틸트 값은 그 파일의 것)', not bad, '; '.join(bad or bits))

    # 17. the plain plant is the complete member: merged models element for
    #     element, airframe defaults less what upstream lacks
    bad, bits = [], []

    for v in vehicles:
        p = plain(v)

        if p is None:
            continue

        c = complete(v, names)
        mine = [canonical(ET.fromstring(ET.tostring(e))) for e in gz_model.merged(p)]
        theirs = [canonical(ET.fromstring(ET.tostring(e))) for e in gz_model.merged(c)]

        if mine != theirs:
            first = next((i for i, (a, b) in enumerate(zip(mine, theirs)) if a != b), 'length')
            bad.append('%s 의 merged 모델이 %s 와 다르다 (자식 %d/%d, 첫 차이 %s)' % (p, c, len(mine), len(theirs), first))

        mine_defaults = gz_model.airframe_defaults(p)
        theirs_defaults = {k: x for k, x in gz_model.airframe_defaults(c).items() if k not in gz_model.absent_params()}

        if mine_defaults != theirs_defaults:
            keys = sorted(k for k in set(mine_defaults) | set(theirs_defaults) if mine_defaults.get(k) != theirs_defaults.get(k))
            bad.append('%s 의 에어프레임 기본값이 %s 와 다르다: %s' % (p, c, ', '.join('%s %s/%s' % (k, mine_defaults.get(k), theirs_defaults.get(k)) for k in keys)))

        if not bad or bad[-1].split(' ')[0] != p:
            bits.append('%s = %s (%d 원소, %d 기본값)' % (p, c, len(mine), len(mine_defaults)))

    verdict('트리의 평범한 플랜트는 오버레이 가족의 완전 멤버다 (merged 모델 원소 단위, 에어프레임 기본값)', not bad,
            '; '.join(bad or bits) or '평범한 플랜트 없음 (트림 전)')

    return summary()


if __name__ == '__main__':
    sys.exit(main())
