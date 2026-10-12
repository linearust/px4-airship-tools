#!/usr/bin/env python3
"""apply_params.py NAME=VALUE ...  — PARAM_SET each, read back, exit 1 on any mismatch.
INT32 params are detected from the read-back type; values are written with that type."""
import struct, sys, time

import fmu_link

link = fmu_link.connect()

if link is None:
    sys.exit('no heartbeat')

bad = 0
for arg in sys.argv[1:]:
    name, value = arg.split('=', 1)
    m = fmu_link.read_param(link, name, tries=5)
    if m is None:
        print('%-18s 읽기 실패' % name); bad += 1; continue
    before, _ = fmu_link.decode(m)
    ptype = m.param_type
    want = int(float(value)) if ptype == fmu_link.INT32 else float(value)
    link.mav.param_set_send(link.target_system, link.target_component, name.encode(),
                            fmu_link.encode(want, ptype), ptype)
    after = None
    t0 = time.time()
    while time.time() - t0 < 3:   # the set is acknowledged by a PARAM_VALUE broadcast of the same name
        m = link.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if m and m.param_id.strip('\x00') == name:
            after = fmu_link.decode(m)[0]
            break
    if after is None:
        m = fmu_link.read_param(link, name, tries=5)
        after = None if m is None else fmu_link.decode(m)[0]
    # REAL32: the FMU stores what fits in a float32, so compare against the
    # round trip of the requested value. An absolute 1e-6 passes a parameter
    # the board rounded away (0.08906 -> 0.089059998) and fails a large one
    # it stored exactly (120.0 vs 119.99999)
    if ptype == fmu_link.INT32:
        ok = after is not None and after == want
    else:
        want32 = struct.unpack('<f', struct.pack('<f', want))[0]
        ok = after is not None and after == want32
    print('%-18s %s -> %s %s' % (name, before, after, 'ok' if ok else '!! MISMATCH')); bad += (0 if ok else 1)
sys.exit(1 if bad else 0)
