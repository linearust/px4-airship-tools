#!/usr/bin/env python3
"""Run NuttX shell commands on the FMU over MAVLink SERIAL_CONTROL and print the output.
usage: fmu_shell.py 'cmd1' 'cmd2' ..."""
import sys, time, os
sys.path.insert(0, os.path.expanduser('~/dev/PX4-Autopilot/Tools'))
from pymavlink import mavutil

import fmu_link
from mavlink_shell import MavlinkSerialPort

port = None
for attempt in range(6):
    try:
        port = MavlinkSerialPort(fmu_link.PORT, 57600, devnum=mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL)
        port.mav.wait_heartbeat(timeout=10)
        break
    except Exception as e:
        print('connect retry %d: %s' % (attempt, e), file=sys.stderr); port = None; time.sleep(2)
if port is None:
    sys.exit('no shell')
port.write('\n'); time.sleep(0.5); port.read(4096)
for cmd in sys.argv[1:]:
    port.write(cmd + '\n')
    if cmd.strip() == 'reboot':      # the link dies: do not wait for output or close()
        time.sleep(0.5); print('$ reboot (sent)'); os._exit(0)
    out, quiet, t0 = '', 0.0, time.time()
    while time.time() - t0 < 20:
        chunk = port.read(4096)
        if chunk:
            out += chunk; quiet = time.time()
        elif out and time.time() - quiet > 1.2:
            break
        else:
            time.sleep(0.05)
    print('$ ' + cmd); print(out.replace('\r', '').strip()); print('---')
port.close()
