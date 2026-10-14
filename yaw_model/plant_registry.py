#!/usr/bin/env python3
"""The plant registry: every plant the yaw model names, keyed by its overlay name.

    python3 plant_registry.py        # the table, and the aliases

A plain tree name changes meaning at the trim (docs/upstream_split_design_
20261012.md S3, 6): today 2520_gz_airship_vectored_independent spawns the
rigid hull and 2527 the complete plant; after the trim the tree's 2520 IS the
complete plant and the rigid one lives in the overlay as 2620
airship_vectored_independent_rigid. A key that means one plant on one day
and another the next cannot anchor a record, so the yaw model keys its
plants by the overlay name (`<vehicle>_rigid`, `<vehicle>_fin`,
`<vehicle>_am_fin_drag`: one physics, whatever the day), and each entry
carries what that plant was on the day its records were flown: the tree
name and model directory it was read from, the SYS_AUTOSTART it logged, and
the snapshot numbers plants.py is anchored to (plants.EXPECTED is a view of
them, keyed by that day's gz target; plants.SOURCES says where each came from).

Lookups (resolve) take the key, the tree name of that day, or either with
the gz_ prefix. A tree name resolves to what it meant on the records' day,
never to what a trimmed tree spawns under it: the records, their chain logs
and records/gains.json are data, keyed by that day's names and numbers, and
only the registry turns those into a plant. An entry's `model` is the
directory the plant's files are under TODAY (the key where a home has it,
the tree name otherwise: gz_model.bare's rule), which is what every live
lookup (controller.airframe_for_model, head_params, gz_model) takes.
"""
import collections
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = os.environ.get('AIRSHIP_BENCH', os.path.dirname(HERE))

if BENCH not in sys.path:
    sys.path.insert(0, BENCH)

import gz_model  # noqa: E402  pure: looks for a model.sdf, no simulator

OVERLAY_SHIFT = gz_model.OVERLAY_SHIFT         # 100: a tree decade 25x0 is the overlay's 26x0
MODELS = 'Tools/simulation/gz/models'           # the model directories, relative to PX4_ROOT (plants.M)

# vehicle -> (controller.AIRFRAMES key, has a tilt band, yaw authority the
# wind cases scale from, N m; None: read from its vehicle file, two pods of
# pods.thrust_n at +-hull.pod_y (sitl_check.vehicle_files))
Vehicle = collections.namedtuple('Vehicle', 'family tilt_band authority_nm')
VEHICLES = {
    'airship_vectored_independent': Vehicle('vectored', True, None),
    # 2.3544 N tail x 1.902 m; plants carries the model's 4.478, sitl_check
    # has scaled its cases from 4.48 since 2026-09 (within its 1 % check)
    'cloudship': Vehicle('cloudship', False, 4.48),
}


class Entry:
    """One plant: its overlay key, and what it was on its records' day."""

    def __init__(self, key, vehicle, tree_name, number_tree, expected=None):
        self.key = key                      # the overlay name: the registry key
        self.vehicle = vehicle              # VEHICLES key (gz_model.vehicle of either name)
        self.tree_name = tree_name          # the model directory of the records' day
        self.number_tree = number_tree      # SYS_AUTOSTART the records logged
        self.expected = expected            # plants.EXPECTED's snapshot numbers; None: not an anchored plant

    @property
    def family(self):
        return VEHICLES[self.vehicle].family

    @property
    def target(self):
        """The gz target of the records' day: the records, chain-log block
        headers, samples files and gains.json are keyed by it."""
        return 'gz_' + self.tree_name

    @property
    def number_overlay(self):
        return self.number_tree + OVERLAY_SHIFT

    @property
    def directory(self):
        """Where the plant's files were read from, relative to PX4_ROOT."""
        return MODELS + '/' + self.tree_name

    @property
    def model(self):
        """The model name a home holds the plant under today: the key once
        the overlay has it, the tree name until then (gz_model.bare)."""
        return self.key if gz_model.has_model(self.key) else self.tree_name

    def __repr__(self):
        return 'Entry(%r, %r, %d)' % (self.key, self.tree_name, self.number_tree)


ENTRIES = (
    Entry('airship_vectored_independent_rigid', 'airship_vectored_independent', 'airship_vectored_independent', 2520,
          dict(I_rigid=129.1503, rr=0.0, c_hull=0.0, x_com=0.00656, c_fin=(), n_full=54.0)),
    Entry('airship_vectored_independent_fin', 'airship_vectored_independent', 'airship_vectored_independent_fin', 2522),
    Entry('airship_vectored_independent_am_fin_drag', 'airship_vectored_independent',
          'airship_vectored_independent_am_fin_drag', 2527,
          dict(I_rigid=129.1503, rr=192.24415, c_hull=241.5613, x_com=0.00656, c_fin=(106.02, 89.28), n_full=54.0)),
    Entry('cloudship_rigid', 'cloudship', 'cloudship', 2510,
          dict(I_rigid=5.7986, rr=0.0, c_hull=0.0, x_com=0.02197, c_fin=(), n_full=4.478)),
    Entry('cloudship_fin', 'cloudship', 'cloudship_fin', 2512),
    Entry('cloudship_am_fin_drag', 'cloudship', 'cloudship_am_fin_drag', 2517,
          dict(I_rigid=5.7986, rr=3.94525, c_hull=5.0873, x_com=0.02197, c_fin=(0.4993, 0.4993), n_full=4.478)),
)
KEYS = tuple(e.key for e in ENTRIES)

# Number aliases. OVERLAY_NUMBER: a tree number of today -> the overlay
# number of the same physics after the trim (25x0..25x7 -> 26x0..26x7 over
# the family decades: rigid 2520 -> 2620 _rigid, 2521..2527 -> 2621..2627,
# 2511..2517 -> 2611..2617, the fixed, indoor, aurora, jaxa, mpi decades
# alike). PLAIN_AFTER_TRIM: the tree's plain number after the trim -> the
# number of today's complete member it then spawns (S3: gz 2520/2510/2500 =
# the old 2527/2517/2505). The classic Cloudship (2519, 2507) stays where
# it is and has no alias (S6).
TREE_DECADES = (2500, 2510, 2520, 2530, 2540, 2550, 2560)
OVERLAY_NUMBER = {n: n + OVERLAY_SHIFT for decade in TREE_DECADES for n in range(decade, decade + 8)}
PLAIN_AFTER_TRIM = {2520: 2527, 2510: 2517, 2500: 2505}


def resolve(name):
    """The entry `name` means: its key, its tree name of the records' day,
    or either with the gz_ prefix."""
    bare = name[3:] if name.startswith('gz_') else name

    for e in ENTRIES:
        if bare in (e.key, e.tree_name):
            return e

    raise LookupError('%s: not a registered plant; the registry keys are %s (yaw_model/plant_registry.py)'
                      % (name, ', '.join(KEYS)))


def by_number(number):
    """The entry whose records logged SYS_AUTOSTART `number`."""
    for e in ENTRIES:
        if e.number_tree == int(number):
            return e

    raise LookupError('SYS_AUTOSTART %s: no registered plant logged it (%s)'
                      % (number, ', '.join('%d %s' % (e.number_tree, e.key) for e in ENTRIES)))


def anchored():
    """The entries plants.py holds numbers for, in registry order."""
    return [e for e in ENTRIES if e.expected is not None]


def main():
    print('%-44s %-44s %-5s %-5s %-9s %s' % ('key', 'tree name (records)', 'tree', 'ovl', 'anchored', 'model today'))

    for e in ENTRIES:
        print('%-44s %-44s %-5d %-5d %-9s %s' % (e.key, e.tree_name, e.number_tree, e.number_overlay,
                                                 'yes' if e.expected else 'no', e.model))

    print('\nOVERLAY_NUMBER: %s' % ', '.join('%d-%d -> %d-%d' % (d, d + 7, d + OVERLAY_SHIFT, d + 7 + OVERLAY_SHIFT)
                                              for d in TREE_DECADES))
    print('PLAIN_AFTER_TRIM: %s' % ', '.join('%d = old %d' % kv for kv in sorted(PLAIN_AFTER_TRIM.items())))
    return 0


if __name__ == '__main__':
    sys.exit(main())
