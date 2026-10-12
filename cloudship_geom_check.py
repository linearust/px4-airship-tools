#!/usr/bin/env python3
"""Rebuild the gz Cloudship's numbers from the measurements and check the SDF.

The model's provenance is the gazebo-classic template
(Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/cloudship/
cloudship.sdf.jinja), which is parameterized by real measurements. Those
measurements are repeated here and everything else is derived, so a typo in
the SDF, or a measurement that changes, shows up as a mismatch rather than as
a vehicle that is quietly the wrong weight.

What is NOT taken from the template is its inertia. Its parallel-axis terms
use the wrong offsets -- the gondola's ixx is built from the x offset and its
izz from the z one -- so the tensor here is recomputed about the real center
of mass from the same bodies.

    python3 cloudship_geom_check.py

Needs no gz. aero_moment_check.py --vehicle cloudship covers the drag set and
the plant; this covers the body.
"""
import math
import sys

import gz_model
import report
from report import summary, verdict

MODEL = 'cloudship'
RHO_AIR, RHO_GAS = 1.225, 0.175
# Not world gravity. This converts the template's thrusts, which it states in
# kilograms-force, into newtons. Three files carried a bare 9.8-ish literal
# with three different values and three different meanings between them.
KGF_TO_N = 9.81
report.ALIGN = 28

# --- the template's measurements -------------------------------------------
HULL_LENGTH, HULL_DIA, HULL_VOLUME, HULL_COV = 4.7, 1.6, 5.97, 2.22
GONDOLA = (0.4, 0.17, 0.1)
FIN_CHORD, FIN_THICK, FIN_SPAN = 0.465, 0.01, 0.39
D_GON_X, R_GON_Z = 2.0, 0.8486
D_PL_X, R_PL_Z = 2.4, 0.784
D_P_X = 0.1
D_FIN_X_S, R_FIN_RAD = 3.635, 0.5634
D_MOT_Y = 0.45
HULL_M, GONDOLA_TOTAL_M, PL_M, FIN_M = 1.7, 3.3, 1.05, 0.055
MOT_MAX_VEL, MOT_MAX_THRUST_KG = 1500.0, 1.62
RUD_MAX_VEL, RUD_MAX_THRUST_KG = 1500.0, 0.24

# --- this model's own choices ----------------------------------------------
# The thruster assembly's mass comes out of the gondola's, which is where the
# template leaves it; the split itself is a placeholder.
ROD_M, PROP_M, TAIL_M = 0.20, 0.04, 0.02

def close(got, want, tol=5e-5):
    return got is not None and abs(got - want) <= tol


def box_inertia(m, sx, sy, sz):
    return (m * (sy * sy + sz * sz) / 12,
            m * (sx * sx + sz * sz) / 12,
            m * (sx * sx + sy * sy) / 12)


def derive():
    """Every number the SDF should contain, from the measurements."""
    a, b = HULL_LENGTH / 2, HULL_DIA / 2
    gl, gw, gh = GONDOLA
    gas_m = RHO_GAS * HULL_VOLUME
    gondola_m = GONDOLA_TOTAL_M - (ROD_M + 2 * PROP_M + TAIL_M)

    z_hull = R_GON_Z + gh / 2                 # hull axis above the gondola's underside
    x_gon, x_pl = HULL_COV - D_GON_X, HULL_COV - D_PL_X
    x_thr = HULL_COV - (D_P_X + D_GON_X - gl / 2)
    x_fin = HULL_COV - (D_FIN_X_S + FIN_CHORD / 2)
    r_fin = R_FIN_RAD + FIN_SPAN / 2

    base = [
        (HULL_M, (0.0, 0.0, z_hull),
         (HULL_M * 2 * b * b / 3, HULL_M * (b * b + a * a) / 3, HULL_M * (b * b + a * a) / 3)),
        (gas_m, (0.0, 0.0, z_hull),
         (gas_m * 2 * b * b / 5, gas_m * (b * b + a * a) / 5, gas_m * (b * b + a * a) / 5)),
        (gondola_m, (x_gon, 0.0, gh / 2), box_inertia(gondola_m, gl, gw, gh)),
        (PL_M, (x_pl, 0.0, z_hull - R_PL_Z), (0.0, 0.0, 0.0)),
        (FIN_M, (x_fin, r_fin, z_hull), box_inertia(FIN_M, FIN_CHORD, FIN_SPAN, FIN_THICK)),
        (FIN_M, (x_fin, -r_fin, z_hull), box_inertia(FIN_M, FIN_CHORD, FIN_SPAN, FIN_THICK)),
        (FIN_M, (x_fin, 0.0, z_hull + r_fin), box_inertia(FIN_M, FIN_CHORD, FIN_THICK, FIN_SPAN)),
        (FIN_M, (x_fin, 0.0, z_hull - r_fin), box_inertia(FIN_M, FIN_CHORD, FIN_THICK, FIN_SPAN)),
    ]
    mass = sum(m for m, _, _ in base)
    com = [sum(m * p[i] for m, p, _ in base) / mass for i in range(3)]
    inertia = [0.0] * 3
    ixz = 0.0
    for m, p, own in base:
        d = [p[i] - com[i] for i in range(3)]
        inertia[0] += own[0] + m * (d[1] ** 2 + d[2] ** 2)
        inertia[1] += own[1] + m * (d[0] ** 2 + d[2] ** 2)
        inertia[2] += own[2] + m * (d[0] ** 2 + d[1] ** 2)
        # Minus, because SDF says so: "the product of inertia convention
        # -m x z (not +m x z) is used" (sdformat 1.10 inertial.sdf). A sign
        # here tips the principal axes the wrong way and nothing else notices.
        ixz -= m * d[0] * d[2]

    # The collision ellipsoid is the drawn one scaled so that it plus the
    # gondola displaces exactly the vehicle's weight in air.
    total = mass + ROD_M + 2 * PROP_M + TAIL_M
    gondola_vol = gl * gw * gh
    hull_vol = total / RHO_AIR - gondola_vol
    scale = (hull_vol / (4 / 3 * math.pi * a * b * b)) ** (1 / 3)

    return {
        'mass': mass, 'com': com, 'inertia': inertia, 'ixz': ixz,
        'z_hull': z_hull, 'x_gon': x_gon, 'x_pl': x_pl, 'x_thr': x_thr,
        'x_fin': x_fin, 'r_fin': r_fin, 'total': total,
        # The yaw thruster goes at the TRAILING EDGE of the ventral fin root,
        # not at the fin station on the hull axis: the template's call is
        # thruster(3, ..., r_fin_x - fin_root_chord/2, 0, -r_fin_rad, ...).
        # This derivation used to say the axis, which is 0.5634 m too high and
        # inside the hull, and the model agreed with it - so the fact passed
        # while both halves were wrong.
        'x_tail': x_fin - FIN_CHORD / 2, 'z_tail': z_hull - R_FIN_RAD,
        'radii': (a * scale, b * scale, b * scale), 'visual_radii': (a, b, b),
        'mot_k': MOT_MAX_THRUST_KG * KGF_TO_N / MOT_MAX_VEL ** 2,
        'rud_k': RUD_MAX_THRUST_KG * KGF_TO_N / RUD_MAX_VEL ** 2,
    }


def pose(elem):
    return [gz_model.pose(elem, i) for i in range(3)]


def main():
    want = derive()
    model = gz_model.sdf(MODEL)
    base = gz_model.named(model, 'link', 'base_link')
    inertial = base.find('inertial')

    verdict('base_link 질량', close(float(inertial.find('mass').text), want['mass'], 5e-5),
            '%s kg (유도 %.4f)' % (inertial.find('mass').text, want['mass']))
    com = pose(inertial)
    verdict('base_link 무게중심', close(com[0], want['com'][0], 5e-5) and close(com[2], want['com'][2], 5e-5),
            'x %.4f z %.4f (유도 %.4f, %.4f)' % (com[0], com[2], want['com'][0], want['com'][2]))
    tensor = inertial.find('inertia')
    for tag, value in (('ixx', want['inertia'][0]), ('iyy', want['inertia'][1]),
                       ('izz', want['inertia'][2]), ('ixz', want['ixz'])):
        got = float(tensor.find(tag).text)
        verdict('관성 %s' % tag, close(got, value, 5e-5), '%.4f (유도 %.4f)' % (got, value))

    radii = [float(v) for v in
             gz_model.named(base, 'collision', 'hull_collision').find('geometry/ellipsoid/radii').text.split()]
    verdict('선체 충돌체 반지름', all(close(g, w, 5e-5) for g, w in zip(radii, want['radii'])),
            '%s (유도 %.5f %.5f %.5f)' % (' '.join('%.5f' % r for r in radii), *want['radii']))
    visual = [float(v) for v in
              gz_model.named(base, 'visual', 'hull_visual').find('geometry/ellipsoid/radii').text.split()]
    verdict('선체 외형 반지름', all(close(g, w) for g, w in zip(visual, want['visual_radii'])),
            '%s (그려진 선체 %.2f %.2f %.2f)' % (' '.join('%.2f' % r for r in visual), *want['visual_radii']))

    # The gondola's box used to be taken from GONDOLA rather than from the
    # model, so half the displaced volume was compared against itself: a box
    # typed 0.4 x 0.17 x 0.3 in the SDF changed nothing any of these facts
    # could see, while it moved the lift by 0.0167 kg. Read it, check it, and
    # displace with what the model actually says.
    box = [float(v) for v in
           gz_model.named(base, 'collision', 'gondola_collision')
           .find('geometry/box/size').text.split()]
    verdict('곤돌라 충돌체 크기', all(close(g, w, 5e-5) for g, w in zip(box, GONDOLA)),
            '%s (템플릿 %.3f %.3f %.3f)' % (' '.join('%.3f' % v for v in box), *GONDOLA))

    volume = 4 / 3 * math.pi * radii[0] * radii[1] * radii[2] + math.prod(box)
    verdict('중성부력', close(volume * RHO_AIR, want['total'], 1e-4),
            '배수량 %.5f m^3 -> %.5f kg, 질량 %.4f kg' % (volume, volume * RHO_AIR, want['total']))

    for name, x, z in (('gondola_collision', want['x_gon'], GONDOLA[2] / 2),
                       ('hull_collision', 0.0, want['z_hull'])):
        p = pose(gz_model.named(base, 'collision', name))
        verdict('%s 위치' % name, close(p[0], x, 5e-5) and close(p[2], z, 5e-5),
                'x %.4f z %.4f (유도 %.4f, %.4f)' % (p[0], p[2], x, z))

    for name, x, y, z in (('fin_port_visual', want['x_fin'], want['r_fin'], want['z_hull']),
                          ('fin_starboard_visual', want['x_fin'], -want['r_fin'], want['z_hull']),
                          ('fin_top_visual', want['x_fin'], 0.0, want['z_hull'] + want['r_fin']),
                          ('fin_bottom_visual', want['x_fin'], 0.0, want['z_hull'] - want['r_fin'])):
        p = pose(gz_model.named(base, 'visual', name))
        verdict(name, all(close(g, w, 5e-5) for g, w in zip(p, (x, y, z))),
                '%s (유도 %.4f %.4f %.4f)' % (' '.join('%.4f' % v for v in p), x, y, z))

    for link, x, y, z in (('thruster_rod', want['x_thr'], 0.0, GONDOLA[2] / 2),
                          ('prop_starboard', want['x_thr'], -D_MOT_Y, GONDOLA[2] / 2),
                          ('prop_port', want['x_thr'], D_MOT_Y, GONDOLA[2] / 2),
                          ('prop_tail', want['x_tail'], 0.0, want['z_tail'])):
        p = pose(gz_model.named(model, 'link', link))
        verdict('%s 위치' % link, all(close(g, w, 5e-5) for g, w in zip(p, (x, y, z))),
                '%s (유도 %.4f %.4f %.4f)' % (' '.join('%.4f' % v for v in p), x, y, z))

    for link, want_mass in (('thruster_rod', ROD_M), ('prop_starboard', PROP_M),
                            ('prop_port', PROP_M), ('prop_tail', TAIL_M)):
        got = float(gz_model.named(model, 'link', link).find('inertial/mass').text)
        verdict('%s 질량' % link, close(got, want_mass),
                '%s kg (이 모델의 선택 %.2f, 곤돌라 몫에서 덜어낸다)' % (got, want_mass))

    # The kinematic graph, which is cheap and catches a class of SDF errors
    # that reads as a vehicle simply not behaving: a joint naming a link that
    # is not there, two joints claiming the same child, a link with no path to
    # base_link, or a plugin driving something that does not exist.
    links = {lk.get('name') for lk in model.findall('link')}
    joints = {j.get('name') for j in model.findall('joint')}
    children = []
    for joint in model.findall('joint'):
        parent, child = joint.find('parent').text, joint.find('child').text
        verdict('조인트 %s' % joint.get('name'),
                parent in links and child in links and child not in children,
                '%s -> %s' % (parent, child))
        children.append(child)
    roots = links - set(children)
    verdict('뿌리 링크가 하나다', roots == {'base_link'}, ', '.join(sorted(roots)) or '없음')

    dangling = [(pl.get('name'), tag, pl.find(tag).text)
                for pl in model.findall('plugin')
                for tag, pool in (('linkName', links), ('link_name', links),
                                  ('jointName', joints), ('joint_name', joints))
                if pl.find(tag) is not None and pl.find(tag).text not in pool]
    verdict('플러그인 참조가 모두 실재한다', not dangling,
            '%d 개 플러그인' % len(model.findall('plugin')) if not dangling
            else '; '.join('%s 의 %s "%s"' % d for d in dangling))

    # Full-command thrust in newtons, whichever plugin a motor uses, and both
    # halves of the tail's curve. Nothing used to read
    # negativeThrustPolynomial at all - not here, not anywhere in the tree -
    # although the reverse half is the entire reason the tail is a
    # GenericMotorModel: it is the vehicle's only yaw authority and the reverse
    # coefficient could have been wrong by a factor of twenty unnoticed.
    for link, k, top, label, reversible in (
            ('prop_starboard', want['mot_k'], MOT_MAX_VEL, '주 추진기', False),
            ('prop_port', want['mot_k'], MOT_MAX_VEL, '주 추진기', False),
            ('prop_tail', want['rud_k'], RUD_MAX_VEL, '꼬리', True)):
        thrust = gz_model.full_thrust(model, link)
        wanted = k * top ** 2
        forward = None if thrust is None else thrust[0]
        verdict('%s 전개추력' % link, close(forward, wanted, 1e-4),
                '%s N, 유도 %.4f N (%s)'
                % ('%.4f' % forward if forward is not None else '없음', wanted, label))

        if reversible:
            reverse = None if thrust is None else thrust[1]
            verdict('%s 역추력' % link, close(reverse, wanted, 1e-4),
                    '%s N, 유도 %.4f N (대칭이어야 한다)'
                    % ('%.4f' % reverse if reverse is not None else '없음', wanted))

    return summary()


if __name__ == '__main__':
    sys.exit(main())
