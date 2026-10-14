#!/usr/bin/env python3
"""Write a hull design (generic_hull_design.py) into an airship family's model files.

    python3 apply_generic_hull.py           # the generic family in the tree, from DEFAULT

Element by element and between stable comment markers, so a rerun writes the
same files again: the body (<base>, <base>_am), the fin plugins (the vectored
_fin members and, for the generic family, airship_fixed/_am), the flaps and
their joints, the drag block of every _drag member and its header, the
Cloudship fins' stall, and the design numbers other files restate (three
model.config descriptions, the am_fin header). Every edit asserts it found
its target exactly once, inside the element it names, and every comment is
formatted from the design's own numbers, so a revision rewrites the words
with the numbers.

The writers take a plan, built by plan(spec, family, root): which vehicle
(a HullSpec), which model directories carry it (a Family) and where they are
(the models root). airship_family_gen.py uses the same writers on a family
it has just copied from the generic one, which is how a new vehicle gets the
same files, derivations and comments as the generic airship."""
import dataclasses
import io
import math
import os
import re
import textwrap
import types

import fin_basis as fb
import generic_hull_design as g
import gz_model

M = gz_model.GZ_MODELS
# An operator stays with its operands when a paragraph is filled, so no line
# starts with '= 10.97' or '* c': it is glued to them with a no-break space,
# which textwrap does not break at, and the space is put back after the fill.
GLUE = '\u00a0'


@dataclasses.dataclass(frozen=True)
class Family:
    """The model directories one vehicle's family is written into, by role.
    The generic family also has the flapped 2500 twin (fixed) and its drag
    members; a vectored-only family leaves those empty. A self-contained
    family (the Cloudship: its bare model is its own body and its _am a full
    copy of it) has no base and no base_am."""
    title: str              # 'Generic airship': how the header names the body
    base: str               # None for a self-contained family
    base_am: str
    bare: str
    drag: str
    fin: str
    am: str
    am_fin: str
    am_fin_drag: str
    fixed: tuple = ()       # flapped members: (fixed, fixed_am)
    fixed_drag: tuple = ()  # their drag members: (fixed_drag, fixed_am_drag)
    surfaces: bool = False  # the fin members' fins carry flaps on servo_2..5, after the pods' tilt servos

    @property
    def fin_members(self):
        return (self.fin, self.am_fin)

    @property
    def drag_members(self):
        return (self.drag, self.am_fin_drag) + tuple(self.fixed_drag)

    @property
    def fin_members_text(self):
        """How the header refers to the members that give the fins lift."""
        return 'airship_*_fin' if self.bare == 'airship_vectored_independent' else '%s and %s' % self.fin_members

    @property
    def members(self):
        """Every directory of the family, body first (none for a self-contained family)."""
        return tuple(m for m in (self.base, self.base_am, self.bare, self.drag, self.fin, self.am, self.am_fin, self.am_fin_drag)
                     + tuple(self.fixed) + tuple(self.fixed_drag) if m)


GENERIC = Family('Generic airship', 'airship_base', 'airship_base_am', 'airship_vectored_independent',
                 'airship_vectored_independent_drag', 'airship_vectored_independent_fin', 'airship_vectored_independent_am',
                 'airship_vectored_independent_am_fin', 'airship_vectored_independent_am_fin_drag',
                 fixed=('airship_fixed', 'airship_fixed_am'), fixed_drag=('airship_fixed_drag', 'airship_fixed_am_drag'))


# The same family under the overlay's rigid naming: the mask-0 members carry
# the _rigid token (gz_model.RIGID, design S2) and the added-mass body keeps
# its name, so the overlay is self-contained; upstream's plain plant merges
# its own copy of that body under the base name, held equal by the twin check
GENERIC_RIGID = Family('Generic airship', 'airship_base_rigid', 'airship_base_am', 'airship_vectored_independent_rigid',
                       'airship_vectored_independent_drag', 'airship_vectored_independent_fin', 'airship_vectored_independent_am',
                       'airship_vectored_independent_am_fin', 'airship_vectored_independent_am_fin_drag',
                       fixed=('airship_fixed_rigid', 'airship_fixed_am'), fixed_drag=('airship_fixed_drag', 'airship_fixed_am_drag'))


# The Cloudship: a ported model, not a designed one (no hull spec, no pods;
# its numbers are the gazebo-classic model's measurements), self-contained,
# written by hand and only copied by the generator. The same two namings.
CLOUDSHIP = Family('Cloudship', None, None, 'cloudship', 'cloudship_drag', 'cloudship_fin', 'cloudship_am', 'cloudship_am_fin',
                   'cloudship_am_fin_drag')
CLOUDSHIP_RIGID = Family('Cloudship', None, None, 'cloudship_' + gz_model.RIGID, 'cloudship_drag', 'cloudship_fin', 'cloudship_am',
                         'cloudship_am_fin', 'cloudship_am_fin_drag')


def vectored_family(name, title, surfaces=False, rigid=False):
    """A vectored-only family under one name: <name>_base, <name>, <name>_fin,
    ...; rigid: the overlay's naming, <name>_base_rigid and <name>_rigid for
    the mask-0 members, the rest as before."""
    if rigid:
        return Family(title, name + '_base_rigid', name + '_base_am', name + '_rigid', name + '_drag', name + '_fin', name + '_am',
                      name + '_am_fin', name + '_am_fin_drag', surfaces=surfaces)

    return Family(title, name + '_base', name + '_base_am', name, name + '_drag', name + '_fin', name + '_am',
                  name + '_am_fin', name + '_am_fin_drag', surfaces=surfaces)


def load(p, rel):
    return io.open(os.path.join(p.root, rel), encoding='utf-8').read()


def save(p, rel, t):
    io.open(os.path.join(p.root, rel), 'w', encoding='utf-8').write(t)
    print('  wrote', rel)


def sub1(t, pattern, repl, count=1):
    new, k = re.subn(pattern, repl, t, flags=re.S)
    assert k == count, (pattern[:70], k, count)
    return new


def element(t, open_tag, close_tag):
    """The one element whose opening text is open_tag, through close_tag."""
    i = t.index(open_tag)
    assert t.count(open_tag) == 1, open_tag
    j = t.index(close_tag, i) + len(close_tag)
    return t[i:j]


def in_elem(t, open_tag, close_tag, pattern, repl):
    """sub1 inside that one element: a pattern whose child the element lacks
    fails here instead of running on into the next element that has one."""
    old = element(t, open_tag, close_tag)
    return t.replace(old, sub1(old, pattern, repl), 1)


def ws(pattern):
    """A refilled comment can break a line at any space of the pattern."""
    return pattern.replace(' ', r'\s+')


def glue(words):
    return re.sub(r'(?<=\S) (<=|>=|[=*/+x.-]) (?=\S)', GLUE + r'\1' + GLUE, words)


def fill(para, width):
    """One comment paragraph, given as its lines, filled to width. Keeps the
    first line's prefix (indent and '<!-- '), the continuation indent and a
    closing ' -->'."""
    lead = para[0][:len(para[0]) - len(para[0].lstrip())]
    opener = '<!-- ' if para[0].lstrip().startswith('<!-- ') else ''
    cont = para[1][:len(para[1]) - len(para[1].lstrip())] if len(para) > 1 else lead + ' ' * len(opener)
    closes = para[-1].rstrip().endswith('-->')
    words = ' '.join(l.strip() for l in para)
    if opener:
        words = words[len('<!-- '):]
    if closes:
        words = words[:-len('-->')].rstrip()
    return textwrap.fill(glue(words) + (' -->' if closes else ''), width, initial_indent=lead + opener, subsequent_indent=cont,
                         break_on_hyphens=False, break_long_words=False).replace(GLUE, ' ')


def refill(t, needle, width=79):
    """Refill the one comment paragraph of t that contains needle. Paragraphs
    end at blank lines and at the comment's ends, so structured paragraphs
    elsewhere in the same comment are untouched."""
    assert t.count(needle) == 1, needle
    lines = t.split('\n')
    at = t[:t.index(needle)].count('\n')
    first = at
    while first > 0 and lines[first - 1].strip() and '<!--' not in lines[first] and lines[first - 1].strip() != '<!--':
        first -= 1
    last = at
    while '-->' not in lines[last] and last + 1 < len(lines) and lines[last + 1].strip() and lines[last + 1].strip() != '-->':
        last += 1
    return '\n'.join(lines[:first] + fill(lines[first:last + 1], width).split('\n') + lines[last + 1:])


def put(t, anchor, template):
    """Replace the one match of anchor with template, its comment filled."""
    return sub1(t, anchor, lambda m: refill(template, '<!--'))


def rewrap(comment, width=78):
    """Fill each blank-line-separated paragraph of a <!-- --> block, so a
    substituted number never leaves a line long or short."""
    body = comment[len('<!--\n'):-len('\n-->')]
    return '<!--\n' + '\n\n'.join(fill(['  ' + ' '.join(p.split())], width) for p in body.split('\n\n')) + '\n-->'


def plan(spec=g.DEFAULT, fam=GENERIC, root=M):
    """The design, resolved at its flap travel, and what the files carry of
    it: the geometry rounded as written, and each fin recomputed from that
    rounded geometry, as variant_check reads it back."""
    d = g.final_design(spec)
    assert d['flap_limit_deg'] is not None, 'R3: no flap travel keeps the fins below stall (python3 generic_hull_design.py)'
    p = types.SimpleNamespace(d=d, s=spec, fam=fam, root=root, travel=d['flap_limit_deg'])
    p.lim = round(math.radians(p.travel), 4)
    a, b = round(d['a'], 5), round(d['b'], 5)
    if 4.0 / 3.0 * math.pi * a * b * b < spec.hull_volume:
        b = round(b + 1e-5, 5)
    p.a, p.b, p.c = a, b, d['chord']
    p.x_mid = round(d['x_mid'], 4)
    p.x_cp = round(p.x_mid + 0.25 * p.c, 4)
    p.hinge_x = round(p.x_mid - p.c / 2.0 + d['flap_chord'], 4)
    p.F = {}
    for n in g.FINS:
        span = d['spans'][n]
        radial = round(d['fin'][n]['radial'], 4)
        f = g.fin(spec, p.c, span, radial - span / 2.0, math.radians(p.travel))
        pos = g.fin_position(spec, n, p.x_mid, radial)
        f.update(pos=pos, cp=(p.x_cp,) + pos[1:], size='%.2f %.2f %.2f' % g.fin_box(n, p.c, span, spec.fin_thick))
        p.F[n] = f
    p.top, p.bot = p.F['top'], p.F['bottom']
    p.ks_h = g.restoring(p.F, ('top', 'bottom'), d['arm'], 'cla_helmbold')
    p.ks = g.restoring(p.F, ('top', 'bottom'), d['arm'])
    p.kp_h = g.restoring(p.F, ('port', 'starboard'), d['arm'], 'cla_helmbold')
    # The gondola's top face against the hull's underside, along its length:
    # how deep the centre line is inside the envelope and how far the long
    # edges are below it (the underside at (x, y) is
    # z = hull_z - sqrt(b^2 (1 - x^2/a^2) - y^2))
    gx = [d['x_gondola'] - spec.gondola[0] / 2.0 + spec.gondola[0] * k / 50.0 for k in range(51)]
    top = spec.gondola_z + spec.gondola[2] / 2.0
    under = lambda x, y: spec.hull_z - math.sqrt(b * b * (1.0 - x * x / (a * a)) - y * y)
    p.inside = [100.0 * (top - under(x, 0.0)) for x in gx]
    p.below = [100.0 * (under(x, spec.gondola[1] / 2.0) - top) for x in gx]
    # G1 as an attitude: the nose-up about the gondola's aft floor edge that
    # brings the ventral fin's trailing edge down to the floor's plane
    p.g1_deg = math.degrees(math.atan(d['ground_clearance'] / ((d['x_gondola'] - spec.gondola[0] / 2.0) - (p.x_mid - p.c / 2.0))))
    # R3's worst fin on the added-mass plant at full rudder, as a share of the stall
    p.r3_worst = g.turn(d, g.turn_speeds(spec)[1], True, False, math.radians(p.travel))[3]
    p.v_gondola = spec.gondola_volume
    return p


def header(p):
    d, s, fam = p.d, p.s, p.fam
    vtot = s.hull_volume + p.v_gondola
    # what the words claim: the lift is exactly the weight, and the centre of
    # mass is under the centre of volume (the gondola station is solved for it)
    assert abs(g.RHO * vtot - d['total']) < 1e-9, (vtot, d['total'])
    assert abs(d['cov'][0] - d['com_all'][0]) < 1e-9, (d['cov'], d['com_all'])
    flaps = ('and the flaps of %s travel +-%%(tr)d deg, the' % fam.fixed[0]) if fam.fixed \
        else 'and flaps of %g %%%% chord would travel +-%%(tr)d deg, the' % (100 * s.flap_fraction)
    lift = ('%(fins)s gives them lift and %(fixed)s gives them flaps.' % dict(fins=fam.fin_members_text, fixed=fam.fixed[0])) if fam.fixed \
        else '%s give them lift.' % fam.fin_members_text
    template = """<!--
  %(title)s base body: hull, gondola, tail and sensors; propulsion
  is added by wrapper models. Neutrally buoyant in the lta world, whose
  Buoyancy plugin sets the fluid density and must name this model to reach
  it: the collision volumes total %(vtot)g m^3, so lift at %(rho)g kg/m^3 is exactly
  the declared %(total)g kg total (hull %(mh).2f + fins %(mf).2f + gondola %(gm)g +
  pods/props %(pm)g). The hull is one prolate spheroid of fineness %(fin)g, %(L).2f x
  %(D).2f m, drawn and collided alike: its %(vh)g m^3 is what the buoyancy, the
  added mass, the Munk moment, the drag reference and the tail are all sized
  on. The gondola's top meets the envelope: along its centre line it sits %(sink_lo).1f to %(sink_hi).1f cm inside, and its long edges %(gap_lo).1f to %(gap_hi).1f cm below it. The centre of
  mass (%(cx).3f, 0, %(cz).4f) m sits under the centre of volume (%(vx).3f, 0,
  %(vz).3f), %(dz).1f mm above the pods' plane.

  The tail is a sizing, not a drawing: a cruciform of %(c).2f m chord, trailing
  edge %(tem).2f m short of the stern at x = %(xte).3f, roots on the hull at the
  mid-chord radius %(r).3f m. The ventral fin is %(sb).2f m, what the ground
  allows at level attitude: its tip is %(gc).0f cm above the gondola floor,
  which %(g1).1f deg of nose-up on the ground uses up. The fins have no
  collision, so the simulation never touches; a vehicle built to this needs a
  stated ground pitch. The dorsal and side fins are
  %(sa).2f m, the shortest that meets the requirement (NACA Report 184): the
  dorsal and ventral fins restore at least %(target)g x the hull's potential-flow
  Munk moment at every speed, judged at zero sideslip on the attached-flow
  slope (Helmbold's) times the slender-body carryover of NACA Report 1307:
  %(ksh).2f against %(munk).2f N m per (m/s)^2 of u v, a ratio of %(r1).2f. The plugins
  carry the flat-plate slope 2 pi AR / (AR + 2) that Ananda, Sukumar and Selig
  (2015) measured, a secant to about 10 deg that includes vortex lift (%(r1s).2f on
  it). The side fins make pitch stable at every speed (%(r2).2f on the same
  basis) with the pendulum on top. A fin stalls where its own incidence, the
  body angle times the hull's upwash K_W(B), reaches the plates' CLmax of %(cmax)g
  (%(as).1f deg of body angle), """ + flaps + """
  most that keeps a full-rudder turn on the added-mass plants %(margin)g deg below
  that. The margin rests on the %(cmax)g: at the %(cmin)g bottom of the measured range
  the same turn is at %(low).0f %% of the stall. Here the fins are visual only, in the inertia at %(areal).2f kg/m^2;
  """ + lift + """ A box on a
  tapering stern sits inside the hull at the leading edge and off it at the
  trailing edge. The gondola sits %(xg).3f m forward so that the fins' mass
  leaves the centre of mass under the centre of volume.
  Added mass is not modeled here: %(base_am)s is this file plus the
  <fluid_added_mass> block, and %(drag)s adds hull
  drag and rotational damping.
-->"""
    text = template % dict(title=fam.title, vtot=vtot, rho=g.RHO, total=d['total'], gm=s.gondola_mass, pm=s.pods_mass, fin=s.fineness, vh=s.hull_volume,
              mh=d['m_hull'], mf=sum(f['mass'] for f in p.F.values()), L=2 * p.a, D=2 * p.b,
              sink_lo=min(p.inside), sink_hi=max(p.inside), gap_lo=min(p.below), gap_hi=max(p.below),
              cx=d['com_all'][0], cz=d['com_all'][2], vx=d['cov'][0], vz=d['cov'][2], dz=1000 * (d['com_all'][2] - s.pod_z),
              c=p.c, tem=s.te_margin, xte=p.x_mid - p.c / 2, r=p.top['root'], sb=p.bot['span'], sa=p.top['span'],
              target=s.stability_target, ksh=p.ks_h, munk=d['munk'], r1=p.ks_h / d['munk'], r1s=p.ks / d['munk'], r2=p.kp_h / d['munk'],
              cmax=fb.CLMAX_PLATE, cmin=fb.CLMAX_PLATE_LOW, margin=s.stall_margin_deg, areal=s.fin_areal,
              **{'as': math.degrees(min(p.top['alpha_stall'], p.bot['alpha_stall']))}, tr=p.travel, xg=d['x_gondola'],
              gc=100 * d['ground_clearance'], g1=p.g1_deg, low=100 * p.r3_worst * fb.CLMAX_PLATE / fb.CLMAX_PLATE_LOW,
              base_am=fam.base_am, drag=fam.drag)
    return rewrap(text)


def inertial(p):
    d, I, com, s = p.d, p.d['inertia'], p.d['base_com'], p.s
    return """        <!-- Hull as a solid spheroid (%.4f kg at the hull centre), gondola
             box (%g kg at x %.4f), four fin plates at their centres (dorsal
             and side %.4f kg, ventral %.4f kg), parallel-axis about the
             combined CoM, products included; pods/props carry their own mass
             in the wrapper, on the pods' plane. -->
        <pose>%.4f 0.0000 %.4f 0 0 0</pose>
        <mass>%r</mass>
        <inertia>
          <ixx>%.4f</ixx>
          <iyy>%.4f</iyy>
          <izz>%.4f</izz>
          <ixy>0.0000</ixy>
          <ixz>%.4f</ixz>
          <iyz>0.0000</iyz>
        </inertia>""" % (d['m_hull'], s.gondola_mass, d['x_gondola'], p.top['mass'], p.bot['mass'], com[0], com[2],
                         round(d['base_mass'], 4), I['ixx'], I['iyy'], I['izz'], I['ixz'])


def added_mass(p):
    d, A, s = p.d, p.d['added'], p.s
    rv = g.RHO * s.hull_volume
    return """        <!-- Fluid added mass of the hull: Lamb's prolate-spheroid values for
             its fineness a/b = %g: k1 %.5f axial, k2 %.5f transverse, k'
             %.5f rotational, on the displaced mass rho V = %.4f kg of the
             spheroid with semi-axes a = %.5f m and b = %.5f m (the collision,
             which is also what is drawn)
             and its transverse inertia rho V (a^2 + b^2) / 5 = %.4f kg m^2.
             Written about the link origin, h = %g m below the hull axis,
             with the offset terms that implies (xq = h m_x, yp = -h m_y,
             pp = h^2 m_y, qq = I_q + h^2 m_x; T = 1/2 v_P^T A v_P with
             v_P = v_O + w x d). The engine folds this into the spatial
             inertia, Coriolis terms and all, so it brings the Munk moment
             with it: -(m_y - m_x) u v = -%.3f u v N m of yaw in this y-port,
             z-up frame, the nose turning away from the velocity. That is what
             the fins of the _fin members are sized against. The fin plates'
             own added mass is not in it. -->
        <fluid_added_mass>
          <xx>%.5f</xx>
          <yy>%.5f</yy>
          <zz>%.5f</zz>
          <xq>%.5f</xq>
          <yp>%.5f</yp>
          <pp>%.5f</pp>
          <qq>%.5f</qq>
          <rr>%.5f</rr>
        </fluid_added_mass>""" % (s.fineness, d['k1'], d['k2'], d['kp'], rv, p.a, p.b, rv * (p.a * p.a + p.b * p.b) / 5.0, s.hull_z, d['munk'],
                                  A['xx'], A['yy'], A['zz'], A['xq'], A['yp'], A['pp'], A['qq'], A['rr'])


def basis(p):
    top, bot = p.top, p.bot
    # induced_cda's reference angle, which the sentence names
    ref = lambda f: 'the stall' if f['alpha_stall'] < fb.DRAG_ALPHA else '%g deg' % math.degrees(fb.DRAG_ALPHA)
    # one word when both fins take it at the same angle (every family before
    # the MPI 5 m one, whose small ventral fin stalls past 10 deg); else both
    reference = ref(top) if ref(top) == ref(bot) else '%s on the dorsal and side fins, %s on the ventral' % (ref(top), ref(bot))
    return """         Basis, per fin: a %.2f m chord; the dorsal and side fins %.2f m of
         exposed span, the ventral %.2f m. Each fin with its mirror image has
         AR_j = 2 span / chord (%.3f and %.3f), the flat-plate slope
         2 pi AR / (AR + 2) (%.3f and %.3f per rad; Ananda, Sukumar and Selig
         2015, a secant to about 10 deg), times the slender-body carryover
         (1 + r/s)^2 with r the hull radius at the mid-chord station (%.4f) and
         s the root plus the span (%.3f and %.3f; NACA Report 1307 eqs. 14 and
         21): cla = %.3f and %.3f per rad on the exposed area. cp at the
         quarter chord (x = %.4f) on each fin's span centre. <upward> is the
         lift direction, normal to the fin's plane. Drag is the plugin's
         linear-in-alpha form, cda sized to the induced drag CL^2 / (pi AR e)
         with e = %g at the lower of %g deg and the stall (%s here), since
         past the stall the plugin uses cda_stall. A fin stalls where its own incidence, the
         body angle times the hull's upwash K_W(B) (%.3f and %.3f), reaches
         the plates' CLmax of %g, the top of the %g-%g the same data
         measured: alpha_stall %.4f and %.4f rad of body angle. Past it cl falls
         to 0 and cd rises to a broadside plate's %g at 90 deg. The plugin
         stalls on the geometric incidence alone and applies nothing when the
         flow at cp comes from behind (forward . v <= 0), so these fins are
         absent in reverse flight and pure lateral drift. The fins' own added
         mass is not modelled. All of it is an estimate with a stated basis,
         not a measurement. -->""" % (p.c, top['span'], bot['span'], top['ar'], bot['ar'], fb.wing_slope(top['ar']), fb.wing_slope(bot['ar']),
                                       top['root'], top['root'] + top['span'], bot['root'] + bot['span'], top['cla'], bot['cla'], p.x_cp,
                                       fb.OSWALD, math.degrees(fb.DRAG_ALPHA), reference, top['k_wb'], bot['k_wb'],
                                       fb.CLMAX_PLATE, fb.CLMAX_PLATE_LOW, fb.CLMAX_PLATE, top['alpha_stall'], bot['alpha_stall'], fb.CD_PLATE_90)


def fin_comments(p):
    s = p.s
    vec = """    <!-- The four fins as lifting surfaces, one lift-drag plugin each, on the
         planform %s draws and its header sizes. The plugin reads
         the velocity at <cp> (worldLinVel + worldAngVel x cp) and applies the
         force there, so no arm terms are needed.
""" % p.fam.base + basis(p)
    fix = """    <!-- The four fins as lifting surfaces WITH their flaps: the same four
         lift-drag plugins as the _fin members, each naming its flap joint,
         with d(cl)/d(delta) = %.3f per rad on the dorsal and side fins and
         %.3f on the ventral: thin-airfoil flap effectiveness %.4f at %g %%
         chord, times the ~%g viscous factor Caughey's "nearly 50 percent
         for a 25 percent chord flap" implies, times the wing-alone slope,
         times K_W(B) rather than the full angle-of-attack carryover, since a
         deflected panel on a body at zero angle of attack gets NACA 1307's
         incidence factors k_W(B) + k_B(W), which slender-body theory puts at
         K_W(B). Linear over the +-%d deg the flaps travel. This vehicle's fins
         are its actuators, so they are in the bare model and it has no _fin
         member. The plugin reads the velocity at <cp> (worldLinVel +
         worldAngVel x cp) and applies the force there, so no arm terms are
         needed.
""" % (p.top['flap_slope'], p.bot['flap_slope'], fb.flap_tau(s.flap_fraction), 100 * s.flap_fraction, fb.VISCOUS, p.travel) + basis(p)
    return vec, fix


def flap_comment(p, vectored=False):
    s = p.s
    pct = 100 * s.flap_fraction
    topics = ("each driven by a JointPositionController on the servo topic the 2500 airframe's output order gives it: Servo1 right "
              "elevator (starboard horizontal fin), Servo2 left elevator, Servo3 upper rudder, Servo4 lower rudder.") if not vectored else (
              "each driven by a JointPositionController on servo_2 to servo_5, after the pods' tilt servos on servo_0 and servo_1; "
              "the fin members' airframes map them to the allocator's Servo1 right elevator (starboard horizontal fin), Servo2 left "
              "elevator, Servo3 upper rudder and Servo4 lower rudder, which come before the tilt servos in its output order.")
    return ("""    <!-- The four flaps: %g %% of the fin chord (%.2f m), hinged at x = %.4f, massless
         links on revolute joints the way rc_cessna's surfaces are, """ + topics + """ Elevator hinges are the y axis (a positive angle lifts
         the trailing edge, lift goes down, the nose goes up: PX4's positive
         pitch), rudder hinges the z axis (a positive angle swings the trailing
         edge to starboard, lift goes to port, the nose to starboard: PX4's
         positive yaw), and the fin plugins below carry the matching signs on
         control_joint_rad_to_cl. Limits +-%d deg, the most that keeps a
         full-rudder turn on the added-mass plants %g deg below the fins' stall
         (%s's header). The flap box is drawn over the aft %g %% of
         the fixed fin box, which a merge include cannot shorten, so it is
         %.2f m thick against the fin's %.2f: at rest its faces are not
         coplanar with the fin's, and at a deflection the GUI shows both.
         Visual only; the lift-drag plugins read the joint. -->""") % (pct, p.d['flap_chord'], p.hinge_x, p.travel, s.stall_margin_deg,
                                                                         p.fam.base, pct, s.flap_thick, s.fin_thick)


def write_body(p):
    print('body')
    s, fam = p.s, p.fam
    head, inert = header(p), inertial(p)
    for rel in (fam.base + '/model.sdf', fam.base_am + '/model.sdf'):
        t = load(p, rel)
        if rel.startswith(fam.base + '/'):
            first = t.index('<!--')
            t = t[:first] + head + t[t.index('-->', first) + len('-->'):]
        t = put(t, r'        <!-- Hull as a solid.*?</inertia>', inert)
        for kind in ('collision', 'visual'):
            hull, gondola = ('<%s name="%s_%s">' % (kind, part, kind) for part in ('hull', 'gondola'))
            t = in_elem(t, hull, '</%s>' % kind, r'(<pose>)[^<]*(</pose>)', r'\g<1>0 0 %g 0 0 0\g<2>' % s.hull_z)
            t = in_elem(t, hull, '</%s>' % kind, r'(<radii>)[^<]*(</radii>)', r'\g<1>%.5f %.5f %.5f\g<2>' % (p.a, p.b, p.b))
            t = in_elem(t, gondola, '</%s>' % kind, r'(<pose>)[^<]*(</pose>)', r'\g<1>%.4f 0 %g 0 0 0\g<2>' % (p.d['x_gondola'], s.gondola_z))
            t = in_elem(t, gondola, '</%s>' % kind, r'(<size>)[^<]*(</size>)', r'\g<1>%g %g %g\g<2>' % s.gondola)
        t = sub1(t, r'(<!-- The hull, drawn and collided alike; buoyancy volume )[\d.]+( m\^3 -->)', r'\g<1>%g\g<2>' % s.hull_volume)
        t = sub1(t, r'(<!-- Ground contact; buoyancy volume )[\d.]+( m\^3 -->)', r'\g<1>%g\g<2>' % p.v_gondola)
        for n, f in p.F.items():
            t = sub1(t, r'(<visual name="fin_%s_visual">\s*<pose>)[^<]*(</pose>)' % n, r'\g<1>%s 0 0 0\g<2>' % xyz(f['pos']))
            t = in_elem(t, '<visual name="fin_%s_visual">' % n, '</visual>', r'(<size>)[^<]*(</size>)', r'\g<1>%s\g<2>' % f['size'])
        save(p, rel, t)
    rel = fam.base_am + '/model.sdf'
    t = load(p, rel)
    t = put(t, r'        <!-- Fluid added mass of the hull:.*?</fluid_added_mass>', added_mass(p))
    save(p, rel, t)


def xyz(v):
    return '%.4f %.4f %.4f' % v


def write_fins(p):
    print('fins')
    s, fam = p.s, p.fam
    vec, fix = fin_comments(p)
    targets = [(m + '/model.sdf', vec, fam.surfaces) for m in fam.fin_members] + [(m + '/model.sdf', fix, True) for m in fam.fixed]
    for rel, comment, flapped in targets:
        t = load(p, rel)
        t = put(t, r'    <!-- The four fins as lifting surfaces.*?-->', comment)
        names = []
        for blk in re.findall(r'<plugin filename="gz-sim-lift-drag-system".*?</plugin>', t, flags=re.S):
            n = re.search(r'<!-- fin_(\w+) -->', blk).group(1)
            names.append(n)
            f = p.F[n]
            new = blk
            for tag, val in (('cp', xyz(f['cp'])), ('area', '%.3f' % f['area']), ('cla', '%.3f' % f['cla']), ('cda', '%.4f' % f['cda']),
                             ('alpha_stall', '%.4f' % f['alpha_stall']), ('cla_stall', '%.4f' % f['cla_stall']), ('cda_stall', '%.4f' % f['cda_stall'])):
                new = sub1(new, r'<%s>[^<]*</%s>' % (tag, tag), '<%s>%s</%s>' % (tag, val, tag))
            if flapped:
                new = sub1(new, r'<control_joint_rad_to_cl>(-?)[\d.]+</control_joint_rad_to_cl>',
                           lambda m: '<control_joint_rad_to_cl>%s%.3f</control_joint_rad_to_cl>' % (m.group(1), f['flap_slope']))
            t = t.replace(blk, new, 1)
        # one plugin per fin: a repeated label would write one fin's numbers
        # onto another's plugin
        assert sorted(names) == sorted(g.FINS), (rel, names)
        if flapped:
            t = put(t, r'    <!-- The four flaps:.*?-->', flap_comment(p, vectored=rel.split('/')[0] in fam.fin_members))
            for n, f in p.F.items():
                t = sub1(t, r'(<link name="flap_%s">\s*<pose>)[^<]*(</pose>)' % n, r'\g<1>%.4f %.4f %.4f 0 0 0\g<2>' % ((p.hinge_x,) + f['pos'][1:]))
                t = sub1(t, r'(<visual name="flap_%s_visual">\s*<pose>)[^<]*(</pose>)' % n, r'\g<1>%.3f 0 0 0 0 0\g<2>' % (-p.d['flap_chord'] / 2))
                # three places: a 7 m hull's 0.225 m flap chord read back as
                # 0.22 at two, a chord ratio of 0.29 against the 0.30 the
                # plugin's slope was computed for (variant_check)
                size = '%.3f %.3f %.3f' % g.fin_box(n, p.d['flap_chord'], f['span'], s.flap_thick)
                t = in_elem(t, '<visual name="flap_%s_visual">' % n, '</visual>', r'(<size>)[^<]*(</size>)', r'\g<1>%s\g<2>' % size)
                t = in_elem(t, '<joint name="flap_%s_joint"' % n, '</joint>', r'(<lower>)[^<]*(</lower>\s*<upper>)[^<]*(</upper>)',
                            r'\g<1>%.4f\g<2>%.4f\g<3>' % (-p.lim, p.lim))
        save(p, rel, t)


def write_drag(p):
    print('drag')
    s, fam = p.s, p.fam
    d, dr = p.d, p.d['drag']
    for rel in (m + '/model.sdf' for m in fam.drag_members):
        t = load(p, rel)
        for tag, val in dr.items():
            t = sub1(t, r'<%s>-?[\d.]+</%s>' % (tag, tag), '<%s>%.4f</%s>' % (tag, val, tag))
        t = sub1(t, r'(<!-- The same two drags written about the link origin, h = )[\d.]+( m below)', r'\g<1>%g\g<2>' % s.hull_z)
        save(p, rel, t)
    # The header's figures, from the set itself. h: link origin to hull axis;
    # c: link origin to centre of mass; l = h - c. Rolling at p about the centre
    # of mass in still air, the axis moves at -p l, so the hull's own damping is
    # |yv| l^3. The plugin reads v = p c at the origin, and its yVabsV, yPabsV,
    # kVabsV and kPabsV terms add up about the centre of mass to
    # |yv| (c^3 - 3 h c^2 + 2 h^2 c).
    yv, kpp = abs(dr['yVabsV']), abs(dr['kPabsP'])
    h, cz = s.hull_z, d['com_all'][2]
    lever = h - cz
    sway = yv * (h * 0.3) ** 2
    still, should = yv * (cz ** 3 - 3 * h * cz ** 2 + 2 * h * h * cz), yv * lever ** 3
    high = (still + kpp) / (should + kpp)
    rel = fam.drag + '/model.sdf'
    t = load(p, rel)
    t = sub1(t, ws(r"\(c_d [\d.]+ axial, c_lat [\d.]+ lateral, c_lp_lin [\d.]+ and c_lp_quad [\d.]+ roll, c_mq = c_nr [\d.]+ pitch and yaw\)"
                   r".*?rotational terms -[\d.]+ \* c\."),
             """(c_d %g axial, c_lat %g lateral, c_lp_lin %g and c_lp_quad %g
  roll, c_mq = c_nr %g pitch and yaw) on an %g x %g m hull. The
  fluid is air at rho = %g kg/m^3 despite the plugin's name. On this hull,
  %s's %.2f x %.2f m spheroid, the frontal reference radius scales
  with the half-height to %.5f m (A = %.4f m^2); the lateral coefficient is
  crossflow drag, which scales with the planform, so it also takes the
  fineness ratio (%g / %.4f, x %.4f); the axial one takes Hoerner's fineness
  dependence of a streamlined body's frontal drag (x %.4f); and the rotational
  scale is 0.5 * rho * S * L^3 * %g with S = %.4f m^2 and L = %.4f m,
  giving %.4f N m. Translational terms are then -0.5 * rho * A * c times those
  factors, rotational terms -%.4f * c.""" % (g.C_D, g.C_LAT, g.C_LP_LIN, g.C_LP_QUAD, g.C_MNR, g.REF_HULL[0], g.REF_HULL[1], g.RHO,
                                              fam.base, 2 * p.a, 2 * p.b, d['ref_radius'], d['ref_area'], s.fineness, g.REF_FINENESS, d['lateral_factor'],
                                              d['axial_factor'], g.ROT_SCALE, d['surface'], d['length'], d['rot_scale'], d['rot_scale']))
    t = sub1(t, ws(r'of mass it is worth [\d.]+ N of sway and [\d.]+ N m of roll at 0\.3 rad/s against [\d.]+ N m/rad of restoring'),
             'of mass it is worth %.1f N of sway and %.1f N m of roll at 0.3 rad/s against %.0f N m/rad of restoring'
             % (sway, sway * lever, d['stiffness']))
    # the header calls the still-air total "a quarter high and on the safe
    # side" where it is within 5 % of that; another hull gets its own fraction
    words = 'a quarter high' if abs(high - 1.25) < 0.05 else '%.0f %% high' % (100 * (high - 1))
    # the template may carry either wording, so the pattern takes both
    t = sub1(t, ws(r'damping reads [\d.]+ N m/\(rad/s\)\^2 where it should read [\d.]+, so the total with kPabsP comes out [\d.]+ against [\d.]+, (?:a quarter|\d+ %) high'),
             'damping reads %.1f N m/(rad/s)^2 where it should read %.1f, so the total with kPabsP comes out %.1f against %.1f, %s'
             % (still, should, still + kpp, should + kpp, words))
    # Rolling at p about the centre of mass at sideslip v, the plugin's roll
    # moment per unit rate is kP + 2 yv c (c - h)|v| without both rate terms and
    # kP + 2 yv c (c - 2h)|v| without kPabsV alone (yv < 0, so both grow
    # positive: damping lost at the |v| where they cross zero)
    k_p = dr['kP']
    no_rate = -2.0 * yv * cz * (cz - h)
    no_kpv = -2.0 * yv * cz * (cz - 2.0 * h)
    t = sub1(t, ws(r"The rate terms are what stop the hull rolling itself apart:") + r".*?(?=\n\n)",
             "The rate terms are what stop the hull rolling itself apart: without both of them the roll moment per unit "
             "rate about the centre of mass is %.2f + %.2f |v| N m/(rad/s), which stops damping above %.2f m/s of "
             "sideslip (on the earlier hull's coefficients that settled into a 6 deg limit cycle), and without kPabsV "
             "alone it is %.2f + %.2f |v|, which stops damping above %.2f m/s."
             % (k_p, no_rate, -k_p / no_rate, k_p, no_kpv, -k_p / no_kpv))
    t = sub1(t, ws(r'it sits [\d.]+ m below the axis the drag acts on'), 'it sits %g m below the axis the drag acts on' % s.hull_z)
    t = sub1(t, ws(r'd x F for d = \(0, 0, [\d.]+\), the offset'), 'd x F for d = (0, 0, %g), the offset' % s.hull_z)
    t = refill(t, 'The plugin applies its wrench at the link origin', 78)
    t = refill(t, 'Two conditions make the plugin', 78)
    t = refill(t, 'So each in-plane axis needs four coefficients')
    t = refill(t, 'The dimensionless coefficients are the airship-control-design')
    t = refill(t, 'One term of the shift has no home')
    save(p, rel, t)


def write_cloudship(fam=CLOUDSHIP, root=M):
    """The Cloudship's fin stall on the family basis, from its own fin
    visuals: the same plate slope and K_W(B), with the low-AR plates' CLmax.
    The family's bare model (cloudship_rigid in the overlay) draws the fins
    its two _fin members get the stall for."""
    print('cloudship')
    p = types.SimpleNamespace(root=root)
    link = gz_model.sdf(fam.bare).find('link')
    z_axis = gz_model.pose(gz_model.named(link, 'collision', 'hull_collision'), 2)
    panels = [fb.panel([float(v) for v in vis.find('geometry/box/size').text.split()], gz_model.pose(vis, 1), gz_model.pose(vis, 2), z_axis)
              for vis in link.findall('visual') if vis.get('name', '').startswith('fin_')]
    assert len(panels) == 4 and all(max(abs(u - v) for u, v in zip(q, panels[0])) < 1e-9 for q in panels), panels
    chord_c, span_c, root_c = panels[0]
    ar_c = fb.joined_ar(chord_c, span_c)
    kwb_c = fb.k_wb(root_c, root_c + span_c)
    as_c = fb.stall(ar_c, kwb_c, clmax=fb.CLMAX_LOW_AR_PLATE)
    for rel in (m + '/model.sdf' for m in fam.fin_members):
        t = load(p, rel)
        # each plugin's post-stall slopes from its own cla and cda, the way
        # write_fins does, so one fin's numbers never land in another's block
        names = []
        for blk in re.findall(r'<plugin filename="%s".*?</plugin>' % gz_model.LIFT_DRAG_FILE, t, flags=re.S):
            names.append(re.search(r'<!-- fin_(\w+) -->', blk).group(1))
            cla_c = float(re.search(r'<cla>([\d.]+)</cla>', blk).group(1))
            cda_c = float(re.search(r'<cda>([\d.]+)</cda>', blk).group(1))
            cls_c, cds_c = fb.post_stall(cla_c, cda_c, as_c)
            new = sub1(blk, r'<alpha_stall>[^<]*</alpha_stall>', '<alpha_stall>%.4f</alpha_stall>' % as_c)
            new = sub1(new, r'<cla_stall>[^<]*</cla_stall>', '<cla_stall>%.4f</cla_stall>' % cls_c)
            new = sub1(new, r'<cda_stall>[^<]*</cda_stall>', '<cda_stall>%.4f</cda_stall>' % cds_c)
            t = t.replace(blk, new, 1)
        assert len(names) == 4 and len(set(names)) == 4, (rel, names)
        t = sub1(t, ws(r"CL\^2 / \(pi AR_j e\) at [\d.]+ deg with e = [\d.]+\.") + '.*?' + ws(r"rises to a broadside plate's [\d.]+ at 90 deg\."),
                 """CL^2 / (pi AR_j e) at %g deg with e = %g. A plate of this aspect
         ratio tops out near cl 1.2-1.3 on its own area (Torres and Mueller's
         low-AR plates), and the panel's own incidence is the body angle
         times the hull's upwash K_W(B) %.4f, so the linear range ends at
         %g / (%.3f x %.4f) = %.4f rad of body angle, %.0f deg on the
         panel itself: the plate slope, a secant to about 10 deg, is carried
         that far. Past it cl falls to 0 and cd rises to a broadside plate's
         %g at 90 deg.""" % (math.degrees(fb.DRAG_ALPHA), fb.OSWALD, kwb_c, fb.CLMAX_LOW_AR_PLATE, fb.wing_slope(ar_c), kwb_c, as_c,
                             math.degrees(as_c * kwb_c), fb.CD_PLATE_90))
        save(p, rel, t)


def write_restated(p):
    """The design numbers other files restate, written here so that a
    revision cannot leave them behind."""
    print('restated')
    s, fam = p.s, p.fam
    rel = fam.am_fin + '/model.sdf'
    t = load(p, rel)
    t = sub1(t, ws(r'restore at least [\d.]+ x that'), 'restore at least %g x that' % s.stability_target)
    t = sub1(t, ws(r'the plate secant, [\d.]+ x\.'), 'the plate secant, %.2f x.' % (p.ks / p.d['munk']))
    t = refill(t, 'The vectored airship with fluid added mass')
    save(p, rel, t)
    for rel, edits in ((fam.base + '/model.config', ((r'[\d.]+ m x [\d.]+ m spheroid hull \(fineness [\d.]+\)',
                                                      '%.2f m x %.2f m spheroid hull (fineness %g)' % (2 * p.a, 2 * p.b, s.fineness)),)),
                       (fam.base_am + '/model.config', ((r"for the hull's fineness [\d.]+\)", "for the hull's fineness %g)" % s.fineness),)),
                       (fam.bare + '/model.config', ((r'%s \([\d.]+ m hull' % re.escape(fam.base), '%s (%.2f m hull' % (fam.base, 2 * p.a)),
                                                     (r'buoyancy-neutral at [\d.]+ kg', 'buoyancy-neutral at %g kg' % p.d['total'])))):
        t = load(p, rel)
        for pattern, text in edits:
            t = sub1(t, pattern, text)
        save(p, rel, t)


def write_family(p):
    """Every file of the family the plan names, from its design."""
    write_body(p)
    write_fins(p)
    write_drag(p)
    write_restated(p)


def main():
    p = plan()
    write_family(p)
    write_cloudship()
    print('done: travel +-%d deg, spans %s' % (p.travel, p.d['spans']))
    return 0


if __name__ == '__main__':
    main()
