# Pass 9 bench verification at the pins (2026-09-16, fmu-v6x, Saleae Logic)

Two firmwares, one flash apart, built from the same tree with only
`AirshipPod.{cpp,hpp}` swapped, so the embedded git hash and build datetime are
identical in both: tell them apart by the uploader's verify step, not `ver all`.

- old = aebe5a8648 pod code (before pass 9)
- new = 3b0a61e319 pod code (d1bd6c6919 realized(), 117046d8a7 rear cone, 3b0a61e319 test)

Each run: `bench_fingerprint.py`, 13 stick vectors x 4 s, 8 channels, median pulse
width over the last second of each hold. Props off, QGC closed, no vehicle.

## Regression, bench config CA_AIRSHIP_TLMIN/TLMAX = -180/180
`fpA_old_bench` vs `fpB_new_bench`: **max |diff| 0.0 us**, 0 of 13 vectors differ.
The refactor is output-identical at the pins where the host harness said it would
be (the harness "bench" parameter set is bit-identical across these commits).

## The fix, corner config -88/92 (ends exactly 180 deg apart: no seam check fires)
`fpA2_old_corner` vs `fpB2_new_corner`: 3 of 13 vectors differ, all of them the
tilt servo of the pod that receives the straight-back demand.

| vector | channel | old | new |
|---|---|---|---|
| yaw +0.5 | MAIN5 starboard tilt | 2000 us (+92 deg) | 1489 us (held at 0 deg) |
| yaw +0.5 | AUX1 starboard motor | 1107 us | 1100 us |
| yaw -0.5 | MAIN6 port tilt | 2000 us | 1489 us |
| yaw -0.5 | AUX2 port motor | 1107 us | 1100 us |
| yaw +0.015 (band) | MAIN6 port tilt | 2000 us (residue of the sweep before) | 1489 us |

The old code swept the servo the full 92 deg of its range and the motor stayed at
1107 us, 7 us above idle: all the travel, none of the thrust. No control surface
channel and no other vector moved, so the change is confined to what it claims.

## Parameters
`snapshot_before.json` vs `snapshot_after.json`: 1079 each, 3 values differ, all
volatile (CAL_BARO0_OFF, COM_FLIGHT_UUID, LND_FLIGHT_T_LO), the same three as the
2026-09-09 run. The tilt range was set to -88/92 for the corner runs and restored.
