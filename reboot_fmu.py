#!/usr/bin/env python3
"""Reboot the FMU with MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN and wait for it to come back (USB re-enumeration)."""
import sys, time
from pymavlink import mavutil

import fmu_link
link = fmu_link.connect(attempts=8, pause=3)
if link is None:
    sys.exit('no heartbeat before reboot')
link.mav.command_long_send(link.target_system, link.target_component,
                           mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
ack = link.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
print('reboot ack:', ack.result if ack else 'none (the board normally resets first)')

if ack is not None and ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
    link.close(); sys.exit('reboot refused: MAV_RESULT %d' % ack.result)

link.close()
time.sleep(12)
link = fmu_link.connect(attempts=8, pause=3)
print('back:', link is not None)
sys.exit(0 if link else 1)
