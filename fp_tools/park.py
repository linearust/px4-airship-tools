#!/usr/bin/env python3
"""park.py FP_DIR — what the pins did at the disarm that ends a fingerprint run
(FP_DIR/export + FP_DIR/fingerprint.json). Per channel: the width held over the
last armed second, the width 4-6 s after the disarm mark, and when the two differ,
how long the move took. A tilt that was away from rest when the vehicle disarmed
is expected to slew back to 1500 us at CA_AIRSHIP_TLT_R (the rate limiter runs on
the rest command too); the motors are expected at the disarmed pulse (PWM_*_DIS,
1000 us on the bench) within a window, not at the armed idle 1100 us."""
import json, math, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import la_mcp
d = sys.argv[1]
fp = json.load(open(os.path.join(d, 'fingerprint.json')))
ev = fp['events']
t_cap = next(e['t'] for e in ev if '캡처' in e['text'] and '시작' in e['text'])
t_dis = next(e['t'] for e in ev if '강제 해제' in e['text']) - t_cap
rows = la_mcp.widths(os.path.join(d, 'export'), window=0.1)
print('disarm mark at %.2f s into the capture' % t_dis)
print('%-16s %9s %9s %s' % ('channel', 'armed us', 'parked us', 'move'))
bad = 0
for ch, name in enumerate(la_mcp.CHANNELS):
    r = rows.get(ch, [])
    before = la_mcp.median(r, t_dis - 1.0, t_dis)
    after = la_mcp.median(r, t_dis + 4.0, t_dis + 6.0)
    move = ''
    if math.isnan(before) or math.isnan(after):
        move = '!! no pulses'; bad += 1
    elif abs(after - before) > 3:
        # first and last window of the move, from the disarm mark on
        seg = [(t, w) for t, w in r if t >= t_dis - 0.2 and t <= t_dis + 6.0]
        t0 = next((t for t, w in seg if abs(w - before) > 3), None)
        t1 = next((t for t, w in reversed(seg) if abs(w - after) > 3), None)
        if t0 is None or t1 is None:
            move = '!! moved, but no window found'; bad += 1
        else:
            # windows are 0.1 s wide: a move inside one window is 0.1 s long
            move = 'done %.1f s after the mark, %.1f s long' % (max(t1 - t_dis, 0.0), max(t1 - t0, 0.0) + 0.1)
    print('%-16s %9.0f %9.0f %s' % (name, before, after, move))
sys.exit(1 if bad else 0)
