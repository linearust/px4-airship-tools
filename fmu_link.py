#!/usr/bin/env python3
"""One MAVLink link to the bench FMU, so the port, the retry and the priming
heartbeat are decided once.

Three scripts each carried their own connect(), and they had already drifted
apart: six attempts against eight, a two second pause against three, and one
of them exiting where the others returned. Nothing recorded why, which is how
a retry policy ends up wrong in the one script that matters.
"""
import struct
import sys
import time

from pymavlink import mavutil

#: The kernel renames ttyACM* across reboots; the by-id path does not move.
PORT = '/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00'
BAUD = 57600
SOURCE_SYSTEM, SOURCE_COMPONENT = 250, 190

INT32 = mavutil.mavlink.MAV_PARAM_TYPE_INT32


def heartbeat(link):
    """The USB protocol autostart only answers a link it has already heard."""
    link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)


def connect(attempts=6, pause=2, port=PORT):
    """A link that has exchanged heartbeats, or None."""
    for attempt in range(attempts):
        try:
            link = mavutil.mavlink_connection(port, baud=BAUD, source_system=SOURCE_SYSTEM,
                                              source_component=SOURCE_COMPONENT)

            for _ in range(10):
                heartbeat(link)

                if link.wait_heartbeat(timeout=1) is not None:
                    return link

        except Exception as e:  # pymavlink connect flake
            print('connect retry %d: %s' % (attempt, e), file=sys.stderr)

        time.sleep(pause)

    return None


def decode(m):
    """PARAM_VALUE carries an INT32 in the bits of a float."""
    if m.param_type == INT32:
        return struct.unpack('<i', struct.pack('<f', m.param_value))[0], 'INT32'

    return m.param_value, 'REAL32'


def encode(value, ptype):
    """The wire value for a set: an INT32 goes over as its own bits."""
    if ptype == INT32:
        return struct.unpack('<f', struct.pack('<i', int(value)))[0]

    return float(value)


def read_param(link, name, tries=3, window=2.0):
    """The PARAM_VALUE of one parameter by name, or None.

    Each try drops what is already queued, so a late reply to an earlier
    try cannot answer it, re-requests, and reads for `window` s, skipping
    replies for other names.
    """
    for _ in range(tries):
        while link.recv_match(blocking=False) is not None:
            pass

        link.mav.param_request_read_send(link.target_system, link.target_component, name.encode(), -1)
        end = time.time() + window

        while time.time() < end:
            m = link.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)

            if m is not None and m.param_id.rstrip('\0') == name:
                return m

    return None
