#!/usr/bin/env python3
"""Dump every parameter of the bench FMU (fmu_link's by-id port) to a JSON file
{name: {"v": value, "t": "INT32"|"REAL32", "i": index}}; exits non-zero
unless the count matches what the vehicle announced."""
import json, sys, time

import fmu_link

OUT = sys.argv[1]


def real(params):
    """Announced parameters only: PX4 also sends the virtual _HASH_CHECK at
    index 65535, which is not in param_count and made a complete 1078-entry
    snapshot exit 1 as "1079 / announced 1078" on 2026-09-23."""
    return sum(1 for p in params.values() if p['i'] < 65535)


link = fmu_link.connect()

if link is None:
    sys.exit('no heartbeat')
sysid, compid = link.target_system, link.target_component
params, total = {}, None
link.mav.param_request_list_send(sysid, compid)
last = time.time()
while time.time() - last < 5:
    m = link.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
    if m is None:
        continue
    last = time.time()
    total = m.param_count
    v, t = fmu_link.decode(m)
    params[m.param_id.strip('\x00')] = {'v': v, 't': t, 'i': m.param_index}
    if real(params) >= total:
        break
# fill gaps by index
if total:
    have = {p['i'] for p in params.values()}
    for idx in range(total):
        if idx in have:
            continue
        for _ in range(3):
            link.mav.param_request_read_send(sysid, compid, b'', idx)
            m = link.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
            if m and m.param_index == idx:
                v, t = fmu_link.decode(m)
                params[m.param_id.strip('\x00')] = {'v': v, 't': t, 'i': idx}
                break
json.dump(params, open(OUT, 'w'), indent=0, sort_keys=True)
print('params: %d / announced %s (+%d virtual) -> %s' % (real(params), total, len(params) - real(params), OUT))
sys.exit(0 if total and real(params) == total else 1)
