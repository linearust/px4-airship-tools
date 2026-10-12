#!/usr/bin/env python3
"""fp_diff.py REF.json NEW.json  — the comparison the bench READMEs quote.

Prints per-channel NEW-REF for every vector, then "max |diff| X us, K of 13 vectors differ".
A vector differs when any channel moves by >= 1 us (the FMU's PWM resolution; the
run-to-run noise on identical firmware is < 0.2 us). A nan cell (no pulses in the
window) is reported, never silently passed. Exit 0 only when 0 vectors differ,
no nan, and both params blocks match on the allocator keys."""
import json, math, sys
ref, new = (json.load(open(p)) for p in sys.argv[1:3])
ta, tb = ref['table'], new['table']
chans = ['MAIN1f', 'MAIN2f', 'MAIN3f', 'MAIN4f', 'MAIN5S', 'MAIN6P', 'AUX1S', 'AUX2P']
print('%-26s' % 'vector (NEW - REF, us)' + ''.join('%9s' % c for c in chans))
mx, nvec, nan = 0.0, 0, 0
for k in ta:
    if k not in tb:
        print('%-26s MISSING in NEW' % k); nvec += 1; continue
    d = [b - a for a, b in zip(ta[k], tb[k])]
    cells = []
    for v in d:
        if math.isnan(v): nan += 1; cells.append('%9s' % 'nan')
        else: cells.append('%+9.1f' % v)
    finite = [abs(v) for v in d if not math.isnan(v)]
    m = max(finite) if finite else 0.0
    mx = max(mx, m)
    if m >= 1.0: nvec += 1
    print('%-26s' % k + ''.join(cells))
print('\nmax |diff| %.1f us, %d of %d vectors differ (>= 1 us), %d nan cells' % (mx, nvec, len(ta), nan))
keys = ['CA_AIRFRAME', 'CA_AIRSHIP_GRP', 'CA_AIRSHIP_TLMIN', 'CA_AIRSHIP_TLMAX', 'CA_AIRSHIP_TLT_R',
        'CA_AIRSHIP_CS_K', 'CA_AIRSHIP_TAIL', 'CA_SV_CS_COUNT', 'MAN_DEADZONE']
pa, pb = ref.get('params', {}), new.get('params', {})
bad = [k for k in keys if pa.get(k) != pb.get(k)]
if bad:
    print('CONFIG MISMATCH, the diff above is meaningless:')
    for k in bad: print('  %-18s ref %s  new %s' % (k, pa.get(k), pb.get(k)))
if pb.get('COM_RC_IN_MODE') != pa.get('COM_RC_IN_MODE'):
    print('note: COM_RC_IN_MODE ref %s new %s (source selection only, not an allocator input)' % (pa.get('COM_RC_IN_MODE'), pb.get('COM_RC_IN_MODE')))
sys.exit(0 if (nvec == 0 and nan == 0 and not bad) else 1)
