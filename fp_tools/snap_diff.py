#!/usr/bin/env python3
"""snap_diff.py REF.json NEW.json — parameter snapshots from snapshot_params.py.
Lists values that changed, then names only in one side. The three volatile ones
(CAL_BARO0_OFF, COM_FLIGHT_UUID, LND_FLIGHT_T_LO) always move; anything else is a
configuration difference that must be explained before a fingerprint is compared."""
import json, sys
a, b = (json.load(open(p)) for p in sys.argv[1:3])
volatile = {'CAL_BARO0_OFF', 'COM_FLIGHT_UUID', 'LND_FLIGHT_T_LO'}
changed = [(k, a[k]['v'], b[k]['v']) for k in sorted(set(a) & set(b)) if a[k]['v'] != b[k]['v']]
real = [c for c in changed if c[0] not in volatile]
print('ref %d params, new %d params' % (len(a), len(b)))
print('changed (%d, of which %d non-volatile):' % (len(changed), len(real)))
for k, x, y in changed: print('  %-20s %s -> %s%s' % (k, x, y, '' if k in volatile else '   <-- explain'))
oa, ob = sorted(set(a) - set(b)), sorted(set(b) - set(a))
print('only in ref (%d): %s' % (len(oa), ' '.join(oa)))
print('only in new (%d): %s' % (len(ob), ' '.join(ob)))
sys.exit(0 if not real else 1)
