#!/usr/bin/env python3
"""ramp.py FP_DIR — tilt slew ramps from a fingerprint run's export (FP_DIR/export + FP_DIR/fingerprint.json).
The table's last-second median cannot see the slew rate; the 0.1 s trace can. At CA_AIRSHIP_TLT_R 120
and a 1000 us / 360 deg servo (2.778 us/deg), a 180 deg move must take ~1.5 s (1.6 in 0.1 s windows).
An unlimited slew shows ramps of <= 0.2 s; a lost rate shows no ramps at all (tilts stuck)."""
import json, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import la_mcp
d = sys.argv[1]
fp = json.load(open(os.path.join(d, 'fingerprint.json')))
ev = fp['events']
t_cap = next(e['t'] for e in ev if '캡처' in e['text'] and '시작' in e['text'])
marks = [(e['text'][2:], e['t'] - t_cap) for e in ev if e['text'].startswith('V ')]
rows = la_mcp.widths(os.path.join(d, 'export'), window=0.1)
US_PER_DEG, RATE = 1000.0 / 360.0, float(fp['params'].get('CA_AIRSHIP_TLT_R') or 120.0)
bad = 0
for ch, name in ((4, 'MAIN5 tilt S'), (5, 'MAIN6 tilt P')):
    r = rows.get(ch, [])
    print('%s: %d windows' % (name, len(r)))
    i = 0
    while i < len(r) - 1:
        if abs(r[i + 1][1] - r[i][1]) > 3:
            j = i
            while j < len(r) - 1 and abs(r[j + 1][1] - r[j][1]) > 3: j += 1
            t0, v0, t1, v1 = r[i][0], r[i][1], r[j][0], r[j][1]
            deg = abs(v1 - v0) / US_PER_DEG
            expect = deg / RATE
            ramp = t1 - t0
            vec = [l for l, t in marks if t <= t0 + 0.2]
            ok = expect - 0.25 <= ramp <= expect + 0.35
            bad += (not ok)
            print('   t=%6.2f  %4.0f -> %4.0f us  ramp %.1f s  expect %.2f s (%.0f deg at %.0f deg/s)  %s  after "%s"'
                  % (t0, v0, v1, ramp, expect, deg, RATE, 'ok' if ok else '!! OFF', vec[-1] if vec else '?'))
            i = j
        i += 1
print('\n%d ramps off their expected duration' % bad)
sys.exit(1 if bad else 0)
