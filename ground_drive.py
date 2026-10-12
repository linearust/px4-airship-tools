#!/usr/bin/env python3
"""Hand-run helper: drive one output function through actuator_test while
Logic 2 captures every probed pin, then print the plateaus each pin held.

    python3 ground_drive.py FUNCTION V1,V2,... OUTNAME [CAPTURE_ID]

Disarmed throughout: actuator_test reaches the outputs only while disarmed
(mixer_module.cpp:481-483). Each value is held HOLD_S and the next command
waits for the override to lapse, so a plateau that is shorter than HOLD_S
was cut by something other than this script. Channel labels are the 2507
bench map read from PWM_MAIN_FUNC*/PWM_AUX_FUNC* on 2026-09-23.
"""
import os
import statistics
import subprocess
import sys
import time

import la_mcp

HOLD_S = 8.0
GAP_S = 4.0
LEAD_S = 5.0
TOL_US = 4.0
MIN_SEG_S = 0.3
CHANNELS = ['MAIN1 Servo1 tilt', 'MAIN2 Motor3 tail', 'MAIN3 Servo3', 'MAIN4 Servo4',
            'MAIN5 Servo5', 'MAIN6 Servo6', 'AUX1 Motor1', 'AUX2 Motor2']
SHELL = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fmu_shell.py')


def shell(*commands):
    """fmu_shell.py's stdout, or None when it did not finish in time.

    The one port has one owner at a time, so a caller holding a MAVLink link
    closes it first. Bounded here because fmu_shell cannot bound itself:
    MavlinkSerialPort (Tools/mavlink_shell.py) waits for a heartbeat with no
    timeout, and fmu_shell reads each command for up to 20 s.
    """
    try:
        return subprocess.run([sys.executable, SHELL] + list(commands), capture_output=True,
                              text=True, timeout=30 + 25 * len(commands)).stdout
    except subprocess.TimeoutExpired:
        return None


def echoed(said, cmd):
    """Whether the board echoed cmd. fmu_shell prints '$ <cmd>' itself
    first, so the board's echo is the second copy."""
    return said is not None and said.count(cmd) >= 2


def plateaus(rows, window):
    """[(start, duration, median width)] for stretches within TOL_US."""
    out, seg = [], []

    def flush():
        if seg and (seg[-1][0] - seg[0][0] + window) >= MIN_SEG_S:
            out.append((seg[0][0], seg[-1][0] - seg[0][0] + window,
                        statistics.median(w for _, w in seg)))

    for t, w in rows:
        if seg and abs(w - statistics.median(x for _, x in seg)) > TOL_US:
            flush()
            seg = []
        seg.append((t, w))

    flush()
    return out


def main():
    function = int(sys.argv[1])
    values = [float(v) for v in sys.argv[2].split(',')]
    name = sys.argv[3]
    # Logic 2 is another process with another cwd (its AppImage mount, which
    # is read-only), so every path it is handed has to be absolute.
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ground_out')
    os.makedirs(out, exist_ok=True)
    existing = sys.argv[4] if len(sys.argv) > 4 else None   # decode a capture already taken

    if existing is None:
        seconds = LEAD_S + len(values) * (HOLD_S + GAP_S) + 10.0
        cid, t0 = la_mcp.start(seconds)
        print('capture %s, %.0f s' % (cid, seconds), flush=True)
        time.sleep(LEAD_S)
    else:
        cid, t0, values = existing, time.time(), []

    for value in values:
        cmd = 'actuator_test set -f %d -v %g -t %d' % (function, value, int(HOLD_S))
        print('  t=%5.1f  %s' % (time.time() - t0, cmd), flush=True)
        said = shell(cmd)
        print('           shell %s' % ('NO ANSWER (timeout)' if said is None
                                       else 'echoed' if echoed(said, cmd) else 'NO ECHO'), flush=True)
        time.sleep(HOLD_S + GAP_S)

    la_mcp.finish(cid, os.path.join(out, name + '.sal'), os.path.join(out, name + '_export'))
    window = 0.1
    rows = la_mcp.widths(os.path.join(out, name + '_export'), window=window)
    print('\nplateaus (start s, duration s, width us):')

    for ch, label in enumerate(CHANNELS):
        segs = plateaus(rows.get(ch, []), window)
        print('  D%d %-18s %s' % (ch, label, '  '.join('%5.1f+%4.1f:%6.1f' % s for s in segs) if segs else 'no pulses'))

    return 0


if __name__ == '__main__':
    sys.exit(main())
