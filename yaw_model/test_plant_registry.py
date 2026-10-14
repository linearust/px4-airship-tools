#!/usr/bin/env python3
"""The plant registry against its consumers and today's tree (split design S3).

    python3 test_plant_registry.py

Pins: every key resolves, by every name it has; the number aliases are
bijective where defined and the entries sit on them; every record_gains.RUNS
entry references a key; each key's overlay number is the airframe file the
overlay holds today (the tree, trimmed, holds the plain numbers alone); the records' own SYS_AUTOSTART (records/gains.json, data) is
the registry's number of that day; plants.EXPECTED and closed_loop's tables
are views of the registry; an unknown name is refused with a clear error.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import closed_loop as cl  # noqa: E402
import controller as ctl  # noqa: E402
import gz_model  # noqa: E402
import plant_registry as reg  # noqa: E402
import plants  # noqa: E402
import record_gains as rg  # noqa: E402

FAILS = []


def check(name, ok, detail=''):
    print('%s  %s%s' % ('PASS' if ok else 'FAIL', name, ('  -- ' + detail) if detail else ''))
    if not ok:
        FAILS.append(name)


def test_resolve():
    for e in reg.ENTRIES:
        names = (e.key, e.tree_name, 'gz_' + e.key, e.target)
        check('%s resolves by its key, tree name and both gz_ forms' % e.key,
              all(reg.resolve(n) is e for n in names), ', '.join(names))
        check('%s: by_number(%d) is the entry' % (e.key, e.number_tree), reg.by_number(e.number_tree) is e)
        check('%s: the key carries a scheme token or _rigid, the vehicle is both names\' (gz_model.vehicle)' % e.key,
              gz_model.family_tokens(e.key) and gz_model.vehicle(e.key) == e.vehicle == gz_model.vehicle(e.tree_name),
              '%s %s' % (gz_model.family_tokens(e.key), e.vehicle))
        check('%s: the vehicle is in VEHICLES and its family an AIRFRAMES key' % e.key,
              e.vehicle in reg.VEHICLES and e.family in ctl.AIRFRAMES, '%s %s' % (e.vehicle, e.family))

    check('the keys are distinct, the tree names are distinct, the numbers are distinct',
          len(set(reg.KEYS)) == len(reg.ENTRIES) == len({e.tree_name for e in reg.ENTRIES})
          == len({e.number_tree for e in reg.ENTRIES}))


def test_unknown():
    for name in ('gz_airship_fixed', 'airship_indoor_am_fin_drag', 'cloudship_am', ''):
        try:
            reg.resolve(name)
            check('resolve(%r) raises' % name, False, 'returned')
        except LookupError as exc:
            check('resolve(%r) raises a LookupError naming the name and the registry' % name,
                  name in str(exc) and 'plant_registry' in str(exc) and 'cloudship_rigid' in str(exc), str(exc)[:80])

    try:
        reg.by_number(2537)
        check('by_number(2537) raises', False, 'returned')
    except LookupError as exc:
        check('by_number(2537) raises a LookupError listing the numbers', '2537' in str(exc) and '2527' in str(exc))

    check('an unregistered name is not in plants.PLANTS, a registered key is',
          'gz_airship_fixed' not in plants.PLANTS and 'cloudship_rigid' in plants.PLANTS
          and 'gz_cloudship' in plants.PLANTS)


def test_aliases():
    check('OVERLAY_NUMBER: every tree number maps to itself + OVERLAY_SHIFT, 8 per decade',
          all(v == k + reg.OVERLAY_SHIFT for k, v in reg.OVERLAY_NUMBER.items())
          and len(reg.OVERLAY_NUMBER) == 8 * len(reg.TREE_DECADES), '%d numbers' % len(reg.OVERLAY_NUMBER))
    check('OVERLAY_NUMBER is bijective', len(set(reg.OVERLAY_NUMBER.values())) == len(reg.OVERLAY_NUMBER))
    check('PLAIN_AFTER_TRIM is bijective and maps plain numbers to that decade\'s complete member',
          len(set(reg.PLAIN_AFTER_TRIM.values())) == len(reg.PLAIN_AFTER_TRIM)
          and all(k % 10 == 0 and k < v < k + 10 for k, v in reg.PLAIN_AFTER_TRIM.items()),
          str(reg.PLAIN_AFTER_TRIM))
    check('every entry\'s overlay number is its alias', all(reg.OVERLAY_NUMBER[e.number_tree] == e.number_overlay
                                                           for e in reg.ENTRIES))
    numbers = {int(name.split('_')[0]) for name, _ in gz_model.airframe_files() if name.split('_')[1] == 'gz'}

    for plain, complete in reg.PLAIN_AFTER_TRIM.items():
        # the tree holds the plain number alone; the family's members are in the overlay under their aliases
        members = [n for n in range(plain, plain + 8) if reg.OVERLAY_NUMBER[n] in numbers]
        vehicle = gz_model.vehicle(ctl.airframe_model(plain))
        tokens = {n: [t for t in gz_model.family_tokens(ctl.airframe_model(reg.OVERLAY_NUMBER[n])) if t != gz_model.RIGID]
                  for n in members}
        check('PLAIN_AFTER_TRIM %d -> %d: the tree\'s %d is the plain %s and nothing else of the decade, %d is in the overlay as %d, '
              'the member with every token' % (plain, complete, plain, vehicle, complete, reg.OVERLAY_NUMBER[complete]),
              plain in numbers and gz_model.family_tokens(ctl.airframe_model(plain)) == []
              and not any(n in numbers for n in range(plain + 1, plain + 8)) and complete in members
              and all(gz_model.vehicle(ctl.airframe_model(reg.OVERLAY_NUMBER[n])) == vehicle for n in members)
              and all(set(t) <= set(tokens[complete]) for t in tokens.values()),
              '%s: %s' % (ctl.airframe_model(reg.OVERLAY_NUMBER[complete]),
                          ' '.join('%d %s' % (reg.OVERLAY_NUMBER[n], '_'.join(tokens[n]) or '-') for n in members)))


def test_overlay_today():
    """The tree is trimmed (split step 6): each entry's plant is in the
    overlay under its key, spawned by the airframe at its overlay number,
    and that is the file the live lookup finds; the tree holds a decade's
    number only for its plain plant."""
    files = dict(gz_model.airframe_files())

    for e in reg.ENTRIES:
        file_name = '%d_gz_%s' % (e.number_overlay, e.key)
        check('%s: airframe %s exists today and spawns %s' % (e.key, file_name, e.key),
              file_name in files and ctl.airframe_model(e.number_overlay) == e.key)
        check('%s: the live lookup of its model today gives its overlay number (trimmed tree)' % e.key,
              e.model == e.key and ctl.airframe_for_model(e.model) == e.number_overlay,
              '%s %d' % (e.model, ctl.airframe_for_model(e.model)))
        check('%s: its model directory in the overlay holds a model.sdf' % e.key,
              os.path.isfile(os.path.join(gz_model.model_dir(e.key), 'model.sdf')), gz_model.model_dir(e.key))
        check('%s: the tree holds its records\' number only as the plain plant' % e.key,
              ('%d_gz_%s' % (e.number_tree, e.tree_name) in files) == (e.number_tree in reg.PLAIN_AFTER_TRIM))


def test_runs():
    for run, (_, _, logs) in rg.RUNS.items():
        check('RUNS %s references registry keys only' % run, all(k in reg.KEYS for k in logs), ', '.join(logs))
        check('RUNS %s: targets() and has_gate() agree with the keys' % run,
              rg.targets(run) == [reg.resolve(k).target for k in logs]
              and all(rg.has_gate(run, t) for t in rg.targets(run))
              and not any(rg.has_gate(run, k) for k in reg.KEYS if k not in logs))

    with open(rg.KEPT, encoding='utf-8') as handle:
        kept = json.load(handle)

    for run, gates in kept.items():
        for target, values in gates.items():
            e = reg.resolve(target)
            check('gains.json %s %s: its SYS_AUTOSTART %s is the registry\'s number of that day'
                  % (run, target, values.get('SYS_AUTOSTART')),
                  run in rg.RUNS and e.key in rg.RUNS[run][2] and values.get('SYS_AUTOSTART') == e.number_tree)

    params, source = rg.gains('chain9_20260927', 'cloudship_am_fin_drag')
    by_target = rg.gains('chain9_20260927', 'gz_cloudship_am_fin_drag')
    check('gains() takes the key or that day\'s target alike and reports the registry\'s number',
          (params, source) == by_target and 'SYS_AUTOSTART 2517' in source, source)


def test_views():
    anchored = reg.anchored()
    check('plants.EXPECTED is keyed by the anchored entries\' targets, in registry order',
          list(plants.EXPECTED) == [e.target for e in anchored])
    check('plants.EXPECTED carries each entry\'s snapshot numbers and its model today',
          all(plants.EXPECTED[e.target] == dict(model=e.model, **e.expected) for e in anchored))
    check('plants.SOURCES names the same targets', set(plants.SOURCES) == set(plants.EXPECTED))
    check('closed_loop.TARGETS are the anchored entries, as (model, family)',
          cl.TARGETS == {e.target: (e.model, e.family) for e in anchored})
    check('closed_loop.ALL_TARGETS is every entry', cl.ALL_TARGETS == {e.target: (e.model, e.family) for e in reg.ENTRIES})
    check('every VEHICLES family is a controller.AIRFRAMES key and every entry\'s vehicle is listed',
          {v.family for v in reg.VEHICLES.values()} <= set(ctl.AIRFRAMES)
          and {e.vehicle for e in reg.ENTRIES} == set(reg.VEHICLES))
    check('the Cloudship\'s fixed authority is within 1 %% of the model\'s (sitl_check\'s check)',
          abs(reg.VEHICLES['cloudship'].authority_nm - gz_model.yaw_authority('cloudship')) <= 0.01 * 4.48,
          '%.3f' % gz_model.yaw_authority('cloudship'))


if __name__ == '__main__':
    test_resolve()
    test_unknown()
    test_aliases()
    test_overlay_today()
    test_runs()
    test_views()
    print('\n%d failed' % len(FAILS) + (': ' + ', '.join(FAILS) if FAILS else ''))
    sys.exit(1 if FAILS else 0)
