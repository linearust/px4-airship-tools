#!/usr/bin/env python3
"""The bare-hull added mass this tool chain writes, against a published CFD
result for a real hull: the Lotte airship (University of Stuttgart).

    python3 lotte_added_mass_check.py

Li & Nahon, "Modeling and Simulation of Airship Dynamics", J. Guidance,
Control, and Dynamics 30(6), 2007, Table 1, quotes the Lotte's added masses
from a panel-method CFD (Lutz et al.) and from their own estimate, both on
the bare hull and with its fins. The generator writes the bare hull's added
mass as Lamb's ellipsoid factors k1, k2, k' on the displaced air
(hull_basis.added_mass_block), the same estimate Li & Nahon use. This prints
the ratio of each to the CFD; it judges nothing (the Lotte's hull is not a
spheroid, and no fins are modelled here). Volume 107 m^3, fineness 4.0 (the
k1 and k2 the CFD implies on that volume match a fineness near 4).
"""
import math
import sys

import generic_hull_design as g
import hull_basis

LOTTE_VOLUME, LOTTE_FINENESS = 107.0, 4.0
# Li & Nahon 2007, Table 1: (CFD, their prediction), kg and kg m^2
BARE = {'m11': (13.0, 11.0), 'm22': (112.0, 114.0), 'm55': (759.0, 793.0)}
WITH_FINS = {'m11': (12.7, 11.0), 'm22': (129.0, 125.0), 'm55': (1379.0, 1279.0)}


def main():
    spec = g.HullSpec(hull_volume=LOTTE_VOLUME, fineness=LOTTE_FINENESS)
    a, b = spec.semi_axes
    block = hull_basis.added_mass_block(a, b, 0.0, g.RHO * LOTTE_VOLUME)
    ours = {'m11': block['xx'], 'm22': block['yy'], 'm55': block['qq']}
    k1, k2, kp = hull_basis.lamb(a, b)
    print('Lotte as a spheroid of %g m^3 at fineness %g: %.2f x %.2f m, displaced air %.1f kg, Lamb k1 %.4f k2 %.4f k\' %.4f'
          % (LOTTE_VOLUME, LOTTE_FINENESS, 2 * a, 2 * b, g.RHO * LOTTE_VOLUME, k1, k2, kp))
    print('%-4s %10s %10s %10s %14s %14s' % ('term', 'ours', 'CFD bare', 'L&N bare', 'ours/CFD', 'L&N/CFD'))

    for k, (cfd, ln) in BARE.items():
        print('%-4s %10.1f %10.1f %10.1f %14.3f %14.3f' % (k, ours[k], cfd, ln, ours[k] / cfd, ln / cfd))

    print('with fins (CFD / L&N; this tool writes the fins\' share separately, not compared): %s'
          % ', '.join('%s %g / %g' % (k, *v) for k, v in WITH_FINS.items()))
    return 0


if __name__ == '__main__':
    sys.exit(main())
