#!/usr/bin/env python3
"""Forward flight of the airship yaw rate loop on HEAD firmware: the first SITL
measurement of the forward-flight defect (D1 plan, step 0b). One plant per run:

    PX4_PARAM_SDLOG_DIRS_MAX=0 GZ_TARGET=gz_airship_vectored_independent_fin ./gz_run.sh ./forward_yaw_check.py
    PX4_PARAM_SDLOG_DIRS_MAX=0 GZ_TARGET=gz_airship_vectored_independent_am_fin_drag ./gz_run.sh ./forward_yaw_check.py
    PX4_PARAM_SDLOG_DIRS_MAX=0 GZ_TARGET=gz_cloudship_fin ./gz_run.sh ./forward_yaw_check.py
    PX4_PARAM_SDLOG_DIRS_MAX=0 GZ_TARGET=gz_cloudship_am_fin_drag ./gz_run.sh ./forward_yaw_check.py

    ./forward_yaw_check.py --plan               # every plant's legs, trims and time budget; no px4
    GZ_TARGET=gz_cloudship_fin ./forward_yaw_check.py --plan
    FORWARD_LEGS=2 PX4_PARAM_SDLOG_DIRS_MAX=0 GZ_TARGET=... ./gz_run.sh ./forward_yaw_check.py   # rerun leg 2
    FORWARD_LEGS=4.0:1.0,4.0:0.5 PX4_PARAM_SDLOG_DIRS_MAX=0 GZ_TARGET=... ./gz_run.sh ...       # or speed:stick
    ./forward_yaw_check.py --analyze sitl_out/forward_<model>_<stamp>.ulg [--events <its _events.json>]
    FORWARD_SOURCE_LOSS_S=8 FORWARD_LEGS=4.0:1.0 GZ_TARGET=... ./gz_run.sh ./forward_yaw_check.py      # lose the speed source 8 s into the stick

On firmware with the forward-speed schedule (D1 increment 1: airship_speed,
AS_SPD_SRC, AS_YR_FF_SLP) every leg also records the schedule: the speed
interval, feedforward, cap and source at the stick, the release and the
release's end, and the rate setpoint against the stick's share of the cap.
With the schedule on (a source and a slope) each leg adds the schedule's
verdicts, which count in the exit code but refuse no replay: the stick
commanded its share of the cap, the integrator stayed off its limit, and the
yaw rate reached the setpoint, or after FORWARD_SOURCE_LOSS_S the selector
reported no source and held the upper bound.

gz_run.sh passes no arguments, so a rerun selects legs with FORWARD_LEGS; --legs
does the same for a direct run. Any other model is refused before px4 is asked
anything.

PX4_PARAM_SDLOG_DIRS_MAX=0 belongs to the command, for two reasons. The plan
(D1 0b) asks that the PX4_PARAM_ route be seen working before anything relies
on it: rcS sets every PX4_PARAM_<name> after the airframe, and this check
refuses to fly (exit 2) unless each one it finds reads back as asked and at
least one of them reads back as other than its default, which is what shows
the route and not the airframe set it. And SITL's rcS sets SDLOG_DIRS_MAX 7: a
logger starting with more date folders than that deletes the oldest folder's
logs, file by file, until there are seven (logger/util.cpp), and SDLOG_MODE 1
starts it at boot, before this check can see anything. 0 keeps them all.

Legs are numbered by the default leg they are, so FORWARD_LEGS=<n> reruns the
leg the records call n: `FORWARD_LEGS=2` flies leg 2 and its records say leg
2. A speed:stick leg that is no default is numbered on from the defaults (4,
5, ...) and its rerun is its speed:stick; one leg twice in a run is refused.

The integrator (rate_ctrl_status) and the allocator's status are logged at
5 Hz by SITL's profile (SDLOG_PROFILE 131), so the times read from them carry
0.2 s steps; the summary gives every quantity's time step ('resolution_ms').
PX4_PARAM_SDLOG_PROFILE=147 adds the high-rate bit 16, which logs
rate_ctrl_status at 50 Hz (logged_topics.cpp add_high_rate_topics) but leaves
control_allocator_status at 5 Hz, and logs the gyro, the actuators and both
setpoints at every publication. Not flown on this machine yet: a logger
dropout inside a leg fails that leg.

The defect, as the model has it (never measured before this check): the fins'
yaw damping grows with forward speed u, so the AS_YAWRATE_MAX setpoint of a
full stick becomes unreachable; the integrator then parks at AS_YR_INT_LIM with
no saturation flag, because the loop's output stays below the pods' or the
tail's limits, and after the stick is released the parked integrator keeps the
hull turning. This check MEASURES that and judges nothing about it. Every
verdict is about whether the measurement is valid; the defect's numbers are
printed and written for the model replay (yaw_model/forward.py).

Each leg, every duration on the simulator's clock (PX4's, which lockstep ties
to the simulator; a wall-clock sleep is a duration of neither):
  a. sticks centred: accelerate with a bounded throttle, step down to the leg's
     constant throttle as the speed arrives (the trim from the files on a drag
     plant, gz_model.trim_thrust; zero on a drag-free one, which then coasts:
     the lift-drag fins have no drag at zero incidence), and wait, bounded, for
     steady straight flight: the speed within tolerance and not drifting,
     |r| < RATE_TOL_DEG, small sway. The speed the trim actually holds is
     reported next to the one the files say it should.
     Manual on a plant that is directionally stable with the loop open. On one
     that is not (cloudship_am_fin_drag: its fins restore about half its Munk
     moment, so with no yaw torque it turns away from its own velocity) the
     acceleration and the wait run in Acro, and a pass through Manual of
     MANUAL_DWELL_S opens the loop before the entry. gz_model's linear
     sway-yaw coefficients decide which, per plant; --plan prints them.
  b. Acro, entered from Manual: the loop opened in Manual, which cleared the
     integrator, and closes now. ENTRY_S centred: the entry state.
  c. yaw stick for STICK_S.   d. centred for RELEASE_S: the release.
  e. Manual, then Acro with a fresh integrator and the stick centred, until the
     turn has stopped (bounded): the start of the next leg.
A leg whose entry never became steady is not flown: a stick taken from an
unsteady entry cannot be compared with a replay of it. The run stops early
(Abort) when the hull comes within MIN_HEIGHT_M of the ground, disarms, or
px4 or the simulator's clock stops answering; what was flown until then is
still measured, and the leg it stopped in is judged incomplete. gz_run.sh
spawns at 20 m; PX4_GZ_MODEL_POSE=0,0,<h> (px4-rc.gzsim) gives more room.

The record is the ULog. SITL logs from boot to disarm (rcS: SDLOG_MODE 1); this
reads the file it is writing from `logger status` after arming, closes it by
disarming at the end (with `logger off` and then `logger stop` behind that),
copies it to sitl_out/forward_<model>_<stamp>.ulg and segments the legs FROM
THE LOG: the modes from vehicle_status, the stick from vehicle_rates_setpoint
(published on every new stick while the loop is closed; the logged
manual_control_setpoint is decimated to 5 Hz and lags a stick change by up to
0.2 s) with the value from manual_control_setpoint. Each leg is then held
against the PX4 times this script read live around its own commands, and fails
if the two disagree; and its logged thrust setpoint against the plan's throttle
count, from the steady window before the entry to the release's end, since a
throttle that moved would be a different leg from the one a replay flies.

Where each quantity is read: u and v are the model origin's (base_link's)
velocity from vehicle_local_position_groundtruth, rotated into the heading
frame by the groundtruth heading, FRD signs (v to starboard); r and p are the
gyro the loop reads (vehicle_angular_velocity); the heading and the attitude
are the groundtruth's. vehicle_angular_velocity_groundtruth is not used: the
bridge differentiates Euler angles for it (GZBridge.cpp poseInfoCallback), so
its yaw component jumps by 2 pi / dt wherever the heading crosses 180 deg. The
EKF's own body-x speed (u_ekf, what a firmware speed source would read) is kept
beside the truth. A fin's incidence is read where the lift-drag plugin reads
it, at its cp, from the origin's velocity plus the rotation; the fins by name
(fin_<name>_* columns) first turn that velocity into the body frame by the
full groundtruth attitude, as the plugin reads it in the link's own frame.

Outputs, per run: the ULog copy; forward_<model>_<stamp>_leg<n>.csv per leg
(entry, stick and release, plus the STEADY_WINDOW_S before the entry, one row
per groundtruth sample, every other topic held at its latest sample; a column
added later goes after every earlier one, so each keeps its place);
forward_<model>_<stamp>_summary.json (every leg's numbers, the parameters the
log carries, the topic rates, the verdicts, and a 'replay' block per leg with
what a model replay needs: the state at the entry, at the stick and at its
release, the constant throttle and its count, the stick timeline, the gains,
the pods' tilts and modes, and 'run_leg', yaw_model/forward.run_leg's keyword
arguments that replay the leg from its Acro switch, whole or None, None for a
leg a verdict of its own or of the run failed; and 'definitions', what each
counted or sampled number is). A field added later leaves every earlier one as
it was, with its meaning;
forward_<model>_<stamp>_events.json (the live timeline and the plan, rewritten
whole after every command the run gives, so a run killed from outside, which
runs no `finally`, still leaves its timeline for --analyze --events). Exit as
report.summary(): 0 when every validity verdict passes, 1 when one fails; 2
when a precondition stops the run first (the firmware's airframe, thrust model
and logging, the PX4_PARAM_ route, the EKF, arming, the log), 3 when px4 does
not answer. One validity verdict is a replay's precondition rather than the
flight's, and gates the exit all the same: the run's IMU_GYRO_CUTOFF against
its yaml default, which the model's gyro filter takes and cannot be handed
another of (yaw_model/planar.py builds it from controller.DEFAULT_PARAMS). A
run flown with another cutoff, a PX4_PARAM_ override say, exits 1 although
its flight was valid, and has no run_leg.
"""
import argparse
import csv
import json
import math
import os
import re
import shutil
import sys
import time

import numpy as np

import gz_model
from report import note, summary, verdict

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'sitl_out')
# px4's working directory: `logger status` prints the log path relative to it
# (platforms/posix main.cpp: PX4_BINARY_DIR/rootfs; LOG_ROOT is ./fs/log on SITL)
ROOTFS = os.path.join(gz_model.ROOT, 'build/px4_sitl_default/rootfs')

# The fin plants: every model whose name carries the scheme's _fin token. The
# defect is the fins' damping growing with speed, so the finless members
# (2520, 2510) have nothing to show here; the plan keeps them as the unchanged
# reference of the next increment. Read from the models directory, so a
# generated family (airship_family_gen.py) is a plant the moment it exists.
ALLOWED = tuple(sorted(n for n in os.listdir(gz_model.GZ_MODELS)
                       if 'fin' in gz_model.family_tokens(n) if os.path.isfile(os.path.join(gz_model.GZ_MODELS, n, 'model.sdf'))))

# (low, high) speed per plant, m/s. Default legs: full stick at both, half
# stick at the high speed, right (positive) yaw. The vectored airship flies the
# plan's 2 and 4. The Cloudship's choice is computed, and --plan prints the
# table it comes from: its pods hold any of these speeds (the drag plant's
# trims are a few percent of full thrust; the drag-free one coasts), so thrust
# is not what limits it. The rigid-fin Cloudship flies the same 2 and 4, where
# the files put full stick out of the loop's reach at both and its fins below
# their stall. The complete one is directionally unstable (Munk over fins):
# open loop at every speed and, with HEAD's P and I, above about 1.2 m/s; at
# 4 m/s the predicted divergence grows more than GROWTH_LIMIT over the entry
# measurement, so no steady entry exists there. 2.5 m/s is the fastest
# half-metre step under that limit and 2.0 the vectored airship's low speed;
# full stick still parks the integrator at both, and half stick does not.
LEG_SPEEDS = {
    'cloudship_fin': (2.0, 4.0),
    'cloudship_am_fin_drag': (2.0, 2.5),
}
# Any other plant flies the middle of its schedule's ramp and LEG_PAST_RAMP
# times the ramp's end, rounded to 0.05 m/s: where the vectored airship's 2.0
# and 4.0 m/s sit on its 1.5 to 2.5 m/s ramp. The ramp is set where the fins
# would stall in a turn (airship_gains.py), so a leg means the same thing on
# every hull; a length scale put the indoor hull's legs below its ramp
LEG_RAMP_MIDDLE, LEG_PAST_RAMP = 0.5, 1.6


def leg_speeds(model):
    """(low, high) m/s for a plant: the table's, or from its airframe's ramp."""
    if model in LEG_SPEEDS:
        return LEG_SPEEDS[model]

    chain = gz_model.airframe_defaults(model)     # the airframe's ramp end, or the firmware's default one
    hi = float(chain['AS_SPD_HI']) if 'AS_SPD_HI' in chain else firmware_default('AS_SPD_HI')
    lo = 0.5 * hi                                 # the firmware starts the ramp at half AS_SPD_HI (2026-10-12)
    return tuple(round(round(u / 0.05) * 0.05, 2) for u in (lo + LEG_RAMP_MIDDLE * (hi - lo), LEG_PAST_RAMP * hi))
STICK_FULL = 1.0
STICK_HALF = 0.5

# Durations, simulator seconds
STICK_S = 20.0
RELEASE_S = 120.0
ENTRY_S = 3.0               # Acro, stick centred, before the stick: the entry state
STEADY_WINDOW_S = 5.0       # the straight flight the entry is judged on
STEADY_MAX_S = 90.0         # bound on waiting for it once the leg's throttle is set
ACCEL_MAX_S = 120.0         # bound on reaching the leg's speed
MANUAL_DWELL_S = 1.0        # Manual between two Acro phases: opens the loop, clears the integrator
BRAKE_HOLD_S = 2.0          # the turn has stopped: |r| < RATE_TOL_DEG this long
BRAKE_MAX_S = 60.0
# The acceleration throttle is sized for this rate on top of the trim, and
# stepped down LEAD_S of it early: a live read of the speed lags by a poll
ACCEL_RATE = 0.15           # m/s^2
LEAD_S = 0.3
ACCEL_THROTTLE_MAX = 0.5    # thrust fraction: the bound on any throttle this check sets
SETTLE_S = 10.0             # wall seconds after the EKF is valid, before arming (sitl_check's)
POLL_S = 0.2                # wall seconds between live reads
WALL_FACTOR = 4.0           # a sim-clock wait gives up after this many wall seconds per sim second
WALL_SLACK_S = 30.0

# Validity tolerances. The steady-flight ones are the same live and in the log.
SPEED_TOL = 0.05            # |mean u - target| over the window, fraction of the target
SPEED_TOL_ABS = 0.05        # m/s, floor of the above
DRIFT_TOL = 0.02            # |u over the last second - u over the first| per window, fraction
DRIFT_TOL_ABS = 0.02        # m/s
RATE_TOL_DEG = 0.3          # deg/s, |r| everywhere in the window
SWAY_TOL = 0.05             # m/s, |v| everywhere in the window
INTEG_CLEAR = 0.005         # |yawspeed_integ| at the first closed-loop sample after Manual
DURATION_TOL_S = 0.3        # stick and release lengths in the log against STICK_S and RELEASE_S
TIMELINE_TOL_S = 0.5        # a logged event against the live PX4-time bracket around its command
MIN_HEIGHT_M = 5.0          # above the model origin's ground: below it the run stops
GROWTH_LIMIT = 2.0          # a leg whose predicted divergence grows more than this over the entry is flagged
THROTTLE_TOL = 0.5e-3       # half a MANUAL_CONTROL throttle count (z in 0..1000)
# Reversals are counted the way yaw_model/forward.py counts them, so the record
# and the model's replay of it count one thing (forward_yaw_check_test.py holds
# both numbers against forward.py's FIN_BAND and TAIL_BAND): per pod a Schmitt
# trigger on cos(tilt), forward above +REVERSAL_COS (|tilt| < 60 deg) and back
# below -REVERSAL_COS (|tilt| > 120 deg), flips summed over the pods; the
# Cloudship's tail command by its sign, neither inside +-TAIL_DEADBAND.
REVERSAL_COS = 0.5
TAIL_DEADBAND = 0.02
# FLT_EPSILON (2^-23): AirshipYawRateLoop::setSaturation flags a yaw shortfall
# strictly past it, each sign (airship_yaw_rate_loop.hpp; the test reads it)
FLAG_EPS = float(np.finfo(np.float32).eps)

# The time estimate in --plan: the 2026-09-27..29 gate logs on these plants ran
# 0.99-1.00 of real time (span over file age), and a gz_run.sh boot (its 6 s
# pause, the make, gz and px4 up) takes about this long before the check starts.
# RTF_SLOW sizes the outer timeout --plan suggests, with room for a slower run.
RTF = 1.0
RTF_SLOW = 0.8
BOOT_WALL_S = 60.0
FINISH_WALL_S = 20.0

# vehicle_status values, read from the message definition rather than restated
VEHICLE_STATUS_MSG = ('msg/versioned/VehicleStatus.msg', 'msg/VehicleStatus.msg')


def msg_constant(name, files=VEHICLE_STATUS_MSG):
    """A uint8 constant of a uORB message definition in the PX4 tree, from the
    first of `files` that exists. LookupError when that file does not define
    it or none exists: a mode compared against a guessed number segments the
    log into legs that were never flown."""
    for rel in files:
        path = os.path.join(gz_model.ROOT, rel)

        if not os.path.exists(path):
            continue

        with open(path) as handle:
            found = re.search(r'^uint8\s+%s\s*=\s*(\d+)' % re.escape(name), handle.read(), re.M)

        if not found:
            raise LookupError('%s 에 %s 가 없다' % (path, name))

        return int(found.group(1))

    raise LookupError('메시지 정의가 없다: %s' % ', '.join(os.path.join(gz_model.ROOT, f) for f in files))


NAV_MANUAL = msg_constant('NAVIGATION_STATE_MANUAL')     # vehicle_status.nav_state
NAV_ACRO = msg_constant('NAVIGATION_STATE_ACRO')
ARMED = msg_constant('ARMING_STATE_ARMED')               # vehicle_status.arming_state

# The frame the legs are measured in: the simulator's groundtruth, or on a
# hardware log the estimator's own (use_estimator_frame()), whose fields of
# the same names stand in: vehicle_local_position vx/vy/vz/z and heading,
# vehicle_attitude q. The u, v and headings are then the EKF's, with its
# errors, which is also what the loop flew on
GT_POS = 'vehicle_local_position_groundtruth'
GT_ATT = 'vehicle_attitude_groundtruth'
ESTIMATOR_FRAME = ('vehicle_local_position', 'vehicle_attitude')
TOPICS = ('vehicle_status', 'vehicle_control_mode', 'manual_control_setpoint', 'vehicle_rates_setpoint',
          'vehicle_angular_velocity', GT_ATT, GT_POS, 'vehicle_local_position', 'vehicle_torque_setpoint',
          'vehicle_thrust_setpoint', 'rate_ctrl_status', 'control_allocator_status', 'actuator_motors',
          'actuator_servos', 'airship_speed')
# The interval each topic must hold, ms (median, inside the legs), for what
# the records do with it. These are the analysis's needs, not the logger's
# settings: SITL logs every one of them at least this often (the yaw rate at
# 125 Hz, the groundtruth at 50 Hz, the integrator and the allocator at 5 Hz),
# and a logger change that dropped below would show here first. The integrator
# and the rate setpoint are published only while the loop is closed, so they
# are measured on the Acro part of each leg.
NEEDED_MS = {
    'vehicle_status': 1000, 'vehicle_control_mode': 1000, 'manual_control_setpoint': 250,
    'vehicle_rates_setpoint': 100, 'vehicle_angular_velocity': 25, GT_ATT: 25, GT_POS: 25,
    'vehicle_local_position': 110, 'vehicle_torque_setpoint': 25, 'vehicle_thrust_setpoint': 25,
    'rate_ctrl_status': 250, 'control_allocator_status': 250, 'actuator_motors': 110, 'actuator_servos': 110,
}
CLOSED_LOOP_ONLY = ('vehicle_rates_setpoint', 'rate_ctrl_status')
GAP_FACTOR = 5.0            # the longest gap inside a leg, in needed intervals
# The topic each recorded quantity is read from. The summary gives each one the
# median logged interval of its topic inside the legs as its time step, so a
# reader knows what an "at the limit after 9.8 s" or a 7 % flagged share can
# resolve (the integrator and the allocator: 200 ms on SITL's profile 131)
RESOLUTION = {
    'yaw and roll rate (gyro)': 'vehicle_angular_velocity',
    'u, v, altitude, fin incidence (groundtruth)': GT_POS,
    'heading, roll, pitch (groundtruth)': GT_ATT,
    'integrator': 'rate_ctrl_status',
    'allocator flags and unallocated torque': 'control_allocator_status',
    'torque setpoint': 'vehicle_torque_setpoint',
    'throttle (thrust setpoint)': 'vehicle_thrust_setpoint',
    'stick onset and release (rate setpoint)': 'vehicle_rates_setpoint',
    'stick value': 'manual_control_setpoint',
    'tilt and pod reversals': 'actuator_servos',
    'motor and tail commands': 'actuator_motors',
}

# The forward-speed schedule (D1 increment 1). airship_att_control publishes
# airship_speed every cycle in every mode and SITL logs it at 10 Hz (logger
# add_optional_topic 100 ms). Firmware from before it has no such topic and no
# such parameters: its legs carry schedule.logged False and no schedule
# verdict, and NEEDED_MS leaves the topic out so its logs still pass.
SPEED_TOPIC = 'airship_speed'
SCHEDULE_PARAMS = ('AS_SPD_SRC', 'AS_SPD_WMAX', 'AS_SPD_DP_TOL', 'AS_SPD_HI', 'AS_YR_FF_SLP')
SPEED_SOURCES = {0: 'none', 1: 'airspeed', 2: 'ground', 3: 'both'}      # AirshipSpeed.msg SOURCE_*
# The rate setpoint against the stick's share of the cap the firmware
# published beside it, each setpoint sample against the cap sample at or
# before it (deg/s, fraction): the cap moves with the filtered speed between
# its 100 ms samples
SCHED_SP_TOL = (0.05, 0.02)
# The stick's last 5 s yaw rate against its setpoint (deg/s, fraction)
SCHED_R_TOL = (0.1, 0.10)
# FORWARD_SOURCE_LOSS_S=<s>: that long into every stick AS_SPD_SRC goes to
# SOURCE_LOSS_SRC, the airspeed sensor alone, which the gz airships do not
# carry, so the selector loses every source in mid-turn; the leg's release end
# puts the value back. The last 5 s of the stick, where its verdicts read,
# must lie wholly after the loss and its 1 s filter
SOURCE_LOSS_ENV = 'FORWARD_SOURCE_LOSS_S'
SOURCE_LOSS_SRC = 1
SOURCE_LOSS_MARGIN_S = 6.0

def use_estimator_frame():
    """Rebind the measurement frame to the estimator's topics (a hardware
    log): the topic list, the needed rates and the resolution names follow."""
    global GT_POS, GT_ATT, TOPICS, NEEDED_MS, RESOLUTION
    old = {GT_POS: ESTIMATOR_FRAME[0], GT_ATT: ESTIMATOR_FRAME[1]}
    TOPICS = tuple(dict.fromkeys(old.get(t, t) for t in TOPICS))
    # the estimator publishes at its own rates: local position about 10 Hz on
    # hardware, attitude at the IMU's
    NEEDED_MS = {t: ms for t, ms in NEEDED_MS.items() if t not in old}
    NEEDED_MS[ESTIMATOR_FRAME[0]] = 110
    NEEDED_MS[ESTIMATOR_FRAME[1]] = 25
    RESOLUTION = {k.replace('(groundtruth)', '(estimator)'): old.get(t, t) for k, t in RESOLUTION.items()}
    GT_POS, GT_ATT = ESTIMATOR_FRAME


# The parameters a replay needs and the plan reads, in the order they print
PARAMS = ('AS_YAWRATE_P', 'AS_YAWRATE_I', 'AS_YR_INT_LIM', 'AS_YAWRATE_FF', 'AS_YAWRATE_MAX', 'MAN_DEADZONE',
          'CA_AIRSHIP_GRP', 'CA_AIRSHIP_TAIL', 'CA_AIRSHIP_TLMIN', 'CA_AIRSHIP_TLMAX', 'CA_AIRSHIP_TLT_R',
          'CA_AIRSHIP_CS_K', 'CA_SV_CS_COUNT', 'CA_R_REV', 'THR_MDL_FAC', 'SYS_AUTOSTART')
# The logger's, read live beside them: whether there is a log, what it holds,
# and whether this boot's logger deleted old log folders
LOG_PARAMS = ('SDLOG_MODE', 'SDLOG_PROFILE', 'SDLOG_DIRS_MAX')
# Where the firmware defaults of PARAMS are defined; an airframe's
# `param set-default` comes on top (gz_model.airframe_defaults)
PARAM_YAMLS = ('src/modules/airship_att_control/airship_att_control_params.yaml',
               'src/modules/manual_control/manual_control_params.yaml',
               'src/modules/control_allocator/module.yaml',
               'src/lib/mixer_module/motor_params.yaml')
# Read from the log for the model replay, beside PARAMS, each with the yaml
# whose default the model takes: the gyro filter's cutoff (yaw_model
# controller.DEFAULT_PARAMS feeds the IMU's 2nd-order Butterworth with it).
# Kept out of PARAMS, which the flight reads live and plans on
ANALYSIS_PARAMS = {'IMU_GYRO_CUTOFF': 'src/modules/sensors/vehicle_angular_velocity/imu_gyro_parameters.yaml'}
# The gains a replay hands run_leg (yaw_model/record_gains.GAINS; the test
# holds the two equal)
GAINS = ('AS_YAWRATE_P', 'AS_YAWRATE_I', 'AS_YR_INT_LIM', 'AS_YAWRATE_FF', 'AS_YAWRATE_MAX', 'MAN_DEADZONE')
# AirshipPod::steer's band, float32 as the firmware compares them
# (AirshipPod.hpp kSteerEngage, kSteerRelease; the test reads the header)
POD_ENGAGE = np.float32(0.02)
POD_RELEASE = np.float32(0.01)
# The time window of the stick's smoothed peak, s (yaw_model/forward.py's
# PEAK_WINDOW_S: the model's r_peak_smoothed is this on its measured rate)
PEAK_WINDOW_S = 0.5
# m/s: at or below this speed of the flow at its cp the lift-drag plugin
# returns before any force (LiftDrag.cc vel.Length() <= 0.01; the model's
# planar.lift_drag_alpha, which the test reads, returns there too)
FIN_MIN_SPEED = 0.01


# ---------------------------------------------------------------------------
# The plan: which legs, at what throttle, for how long. Files only, no px4.
# ---------------------------------------------------------------------------

def entity():
    """The model gz_run.sh resolved (PX4_GZ_ENTITY), else GZ_TARGET's, or None.

    sitl_link's own resolution, without importing it: --plan must not need
    a simulator, and sitl_link exits on a model it cannot find."""
    name = os.environ.get('PX4_GZ_ENTITY')

    if name:
        return name

    target = os.environ.get('GZ_TARGET')

    if not target:
        return None

    name = target.removeprefix('gz_')
    world = os.environ.get('PX4_GZ_WORLD', 'lta')
    return name[:-len(world) - 1] if name.endswith('_' + world) else name


def refuse(model):
    """SystemExit unless `model` is one of the fin plants."""
    if model not in ALLOWED:
        raise SystemExit('%s 은 이 검사의 기체가 아니다. 핀이 있는 기체만 난다: %s'
                         % (model, ', '.join(ALLOWED)))


def firmware_default(name):
    """A parameter's firmware default from its module yaml. LookupError when
    none of PARAM_YAMLS defines it: a plan built on a guessed gain is a plan
    for some other firmware."""
    import yaml

    for rel in PARAM_YAMLS:
        path = os.path.join(gz_model.ROOT, rel)

        if not os.path.exists(path):
            continue

        with open(path) as handle:
            doc = yaml.safe_load(handle)

        for group in doc.get('parameters', []):
            found = (group.get('definitions') or {}).get(name)

            if found is not None and 'default' in found:
                return float(found['default'])

    raise LookupError('%s 의 기본값을 %s 에서 찾지 못했다' % (name, ', '.join(PARAM_YAMLS)))


def yaml_default(name, rel):
    """A parameter's default from one module yaml of the PX4 tree (the
    analysis's, ANALYSIS_PARAMS; the plan's go through firmware_default).
    LookupError when the file is not there or does not define it."""
    import yaml

    path = os.path.join(gz_model.ROOT, rel)

    if not os.path.exists(path):
        raise LookupError('%s 가 없다' % path)

    with open(path) as handle:
        doc = yaml.safe_load(handle)

    for group in doc.get('parameters', []):
        found = (group.get('definitions') or {}).get(name)

        if found is not None and 'default' in found:
            return float(found['default'])

    raise LookupError('%s 의 기본값을 %s 에서 찾지 못했다' % (name, rel))


def f32_decimal(value):
    """The shortest decimal that names the same float32 as `value`: a ulog
    parameter is a float32, so AS_YAWRATE_P 0.7 logs as 0.699999988, and a
    replay handed this gets the yaml's 0.7 back."""
    for digits in range(1, 10):
        text = '%.*g' % (digits, value)

        if np.float32(float(text)) == np.float32(value):
            return float(text)

    return float(value)


def plan_params(model):
    """PARAMS as the files say this model's airframe boots them: the module
    default, then the posix airframe chain's set-default."""
    chain = gz_model.airframe_defaults(model)
    out = {}

    for name in PARAMS:
        if name == 'SYS_AUTOSTART':
            out[name] = float(gz_model.airframe(model)[0])
        elif name in chain:
            out[name] = float(chain[name])
        else:
            out[name] = firmware_default(name)

    return out


def setpoint_deg(stick, params):
    """The yaw rate a stick commands, deg/s: MAN_DEADZONE first, then linear
    to AS_YAWRATE_MAX (airship_manual_input.hpp yawRateSetpoint)."""
    dz = params['MAN_DEADZONE']
    s = abs(stick)

    if s <= dz:
        return 0.0

    return math.copysign((s - dz) / (1.0 - dz), stick) * params['AS_YAWRATE_MAX']


def quantize(fraction):
    """(counts, the value to give Sticks.throttle, the fraction that arrives).

    MANUAL_CONTROL carries the throttle as an integer z in 0..1000 and
    Sticks truncates (int(throttle * 1000)): a trim of 0.0119 would arrive as
    11 counts, 8 % short of its thrust. Handing it the count's midpoint makes
    the truncation land on the rounded count."""
    counts = int(round(max(0.0, min(1.0, fraction)) * 1000.0))
    return counts, (counts + 0.5) / 1000.0, counts / 1000.0


def default_legs(model):
    low, high = leg_speeds(model)
    return [(low, STICK_FULL), (high, STICK_FULL), (high, STICK_HALF)]


def parse_legs(spec, model):
    """[(speed, stick)] from FORWARD_LEGS/--legs, or the defaults.

    Items are 1-based indices into the default legs ('2', '1,3') or
    speed:stick pairs ('4.0:1.0'), comma separated. Anything else is an
    error: a rerun that silently flew the defaults would be recorded as the
    rerun."""
    defaults = default_legs(model)

    if not spec:
        return defaults

    legs = []

    for item in spec.split(','):
        item = item.strip()

        if re.fullmatch(r'\d+', item):
            index = int(item)

            if not 1 <= index <= len(defaults):
                raise ValueError('다리 %s 가 없다: 기본 다리는 1..%d' % (item, len(defaults)))

            legs.append(defaults[index - 1])
            continue

        found = re.fullmatch(r'([0-9.]+):([-+]?[0-9.]+)', item)

        if not found:
            raise ValueError('다리 "%s" 를 읽지 못했다: 번호 또는 속도:스틱' % item)

        speed, stick = float(found.group(1)), float(found.group(2))

        if not (0.3 <= speed <= 8.0) or not (0.0 < abs(stick) <= 1.0):
            raise ValueError('다리 "%s": 속도는 0.3..8 m/s, 스틱은 0 이 아닌 -1..1' % item)

        legs.append((speed, stick))

    if not legs:
        raise ValueError('다리가 하나도 없다: "%s"' % spec)

    if len(set(legs)) != len(legs):
        # a leg's number names its CSV and its verdicts: twice in one run, the
        # second would overwrite the first's record under the same name
        raise ValueError('같은 다리가 두 번 있다: "%s"' % spec)

    return legs


def leg_numbers(model, legs):
    """[(number, rerun)] for `legs` in flight order. A default leg keeps its
    default index wherever it comes in the run, so FORWARD_LEGS=<number>
    reruns exactly the leg a record calls <number> and two runs' records of
    one leg carry one name; a leg that is no default is numbered on from the
    defaults in its order, and its rerun is its speed:stick."""
    defaults = default_legs(model)
    out, extra = [], len(defaults)

    for speed, stick in legs:
        if (speed, stick) in defaults:
            number = defaults.index((speed, stick)) + 1
            out.append((number, str(number)))
        else:
            extra += 1
            out.append((extra, '%g:%g' % (speed, stick)))

    if len({n for n, _ in out}) != len(out):
        raise ValueError('같은 다리가 두 번 있다: %s' % legs)

    return out


def loop_reach(model, params, speed, stick):
    """One-degree-of-freedom estimate of what the loop holds at `speed`, from
    the files (the plan's speed choice; a prediction, never a verdict):

        P (sp - r) + I_int + FF sp = (k u r + c_r r |r|) / A

    with k the fins' yaw damping per unit speed (gz_model.fin_yaw_damping),
    c_r the hull's quadratic yaw damping of a _drag member and A the yaw
    authority. With the integrator at its limit the left side is at most
    (P + FF) sp + AS_YR_INT_LIM, which sets the fastest turn the loop can
    hold; below the setpoint the integrator parks. No sideslip and no Munk
    moment, so it is an upper estimate on a stable hull. The fins' incidence
    at that rate is r arm / u, against the plugin's stall angle.

    Returns {'setpoint', 'reach' (deg/s, at most the setpoint), 'parks',
    'integrator' (what holding the setpoint would take, normalized), 'alpha'
    and 'stall' (deg, the worst vertical fin)}."""
    A = gz_model.yaw_authority(model)
    k = gz_model.fin_yaw_damping(model)
    c_r = abs(gz_model.hydrodynamics(gz_model.merged(model)).get('nRabsR', 0.0))
    sp = math.radians(abs(setpoint_deg(stick, params)))
    P, FF, L = params['AS_YAWRATE_P'], params['AS_YAWRATE_FF'], params['AS_YR_INT_LIM']
    a, b, c = c_r / A, P + k * speed / A, (P + FF) * sp + L
    r = c / b if a == 0.0 else (-b + math.sqrt(b * b + 4.0 * a * c)) / (2.0 * a)
    held = min(r, sp)
    fins = gz_model.vertical_fins(model)
    alpha = max(math.atan2(held * f['arm'], speed) for f in fins)
    return {'setpoint': math.degrees(sp), 'reach': math.degrees(held), 'parks': r < sp,
            'integrator': (k * speed * sp + c_r * sp * sp) / A - FF * sp,
            'alpha': math.degrees(alpha), 'stall': math.degrees(min(f['alpha_stall'] for f in fins))}


def lateral_growth(model, params, speed, closed=True):
    """Largest real part, 1/s, of the linear sway-yaw dynamics at `speed`
    (gz_model.lateral_derivatives), with the loop's P and I closed on the yaw
    rate when `closed` (the stick centred, so FF does nothing) and with the yaw
    torque at zero otherwise. Positive is a divergence. The pods' tilt and the
    tail are taken as instant; on 2522, whose pods sit at zero thrust, a small
    yaw demand swings a pod first, which this does not see."""
    d = gz_model.lateral_derivatives(model)
    A = gz_model.yaw_authority(model)
    u = speed
    a11 = d['y_v'] * u / d['m_y']
    a12 = (d['y_r'] - d['m_x']) * u / d['m_y']
    a21 = d['n_v'] * u / d['i_z']

    if not closed:
        return float(max(np.linalg.eigvals(np.array([[a11, a12], [a21, d['n_r'] * u / d['i_z']]])).real))

    a22 = (d['n_r'] * u - A * params['AS_YAWRATE_P']) / d['i_z']
    a23 = -A * params['AS_YAWRATE_I'] / d['i_z']
    matrix = np.array([[a11, a12, 0.0], [a21, a22, a23], [0.0, 1.0, 0.0]])
    return float(max(np.linalg.eigvals(matrix).real))


def accel_mode(model, params, speeds):
    """'acro' when the plant diverges with the loop open at any of the legs'
    speeds, else 'manual'."""
    return 'acro' if any(lateral_growth(model, params, u, closed=False) > 0.0 for u in speeds) else 'manual'


def accel_throttle(model, speed):
    """The trim plus ACCEL_RATE's worth of thrust on the surge mass, bounded."""
    extra = ACCEL_RATE * gz_model.surge_mass(model) / gz_model.pod_thrust(model)
    return min(ACCEL_THROTTLE_MAX, gz_model.trim_thrust(model, speed) + extra)


def surge_time(model, u0, target, throttle, bound=ACCEL_MAX_S):
    """Simulator seconds for the surge equation m u' = T throttle - c u|u| to
    take u0 to within the lead of target, or None past `bound`. Above the
    target the throttle is taken as zero; a drag-free plant never slows."""
    m = gz_model.surge_mass(model)
    T = gz_model.pod_thrust(model)
    c = gz_model.translational_drag(gz_model.merged(model))[0]
    lead = ACCEL_RATE * LEAD_S
    u, t, dt = u0, 0.0, 0.01
    slowing = u0 > target + lead

    while t < bound:
        if (slowing and u <= target + lead) or (not slowing and u >= target - lead):
            return t

        thrust = 0.0 if slowing else T * throttle
        u += (thrust - c * u * abs(u)) / m * dt
        t += dt

        if slowing and c == 0.0:
            return None

    return None


def leg_plan(model, params, speed, stick, u_prev, mode):
    """Everything one leg is flown with, and what it should cost."""
    trim = gz_model.trim_thrust(model, speed)
    counts, command, cruise = quantize(trim)
    c = gz_model.translational_drag(gz_model.merged(model))[0]
    push = accel_throttle(model, speed)
    accel_s = surge_time(model, u_prev, speed, push)
    reach = loop_reach(model, params, speed, stick)
    growth = lateral_growth(model, params, speed)
    # the brake: the loop's P and the fins' damping on the yaw inertia, from
    # at most the stick's reach down to RATE_TOL_DEG, then held
    d = gz_model.lateral_derivatives(model)
    tau = d['i_z'] / (params['AS_YAWRATE_P'] * gz_model.yaw_authority(model) - d['n_r'] * speed)
    brake_s = MANUAL_DWELL_S + tau * math.log(max(1.0, reach['reach'] / RATE_TOL_DEG)) + BRAKE_HOLD_S
    dwell = MANUAL_DWELL_S if mode == 'acro' else 0.0
    fixed = dwell + ENTRY_S + STICK_S + RELEASE_S
    typical = (accel_s if accel_s is not None else ACCEL_MAX_S) + STEADY_WINDOW_S + 1.0 + fixed + brake_s
    worst = ACCEL_MAX_S + STEADY_MAX_S + fixed + BRAKE_MAX_S + MANUAL_DWELL_S
    return {'speed': speed, 'stick': stick, 'setpoint_deg_s': setpoint_deg(stick, params),
            'trim': trim, 'throttle_counts': counts, 'throttle_command': command, 'throttle': cruise,
            'holds_m_s': math.sqrt(cruise * gz_model.pod_thrust(model) / c) if c > 0.0 else None,
            'accel_throttle': push, 'accel_from': u_prev, 'accel_s': accel_s, 'accel_mode': mode,
            'reach': reach, 'growth': growth, 'growth_entry': math.exp(max(0.0, growth) * (STEADY_WINDOW_S + ENTRY_S)),
            'brake_s': brake_s, 'sim_typical_s': typical, 'sim_worst_s': worst}


def vehicle_plan(model, legs=None, params=None):
    """The whole run on one plant: the legs in order, each from the speed the
    one before left (its target; a drag-free hull that slowed in its turn
    takes a little longer), and the time it should take."""
    refuse(model)
    params = params or plan_params(model)
    legs = legs or default_legs(model)
    mode = accel_mode(model, params, [s for s, _ in legs])
    out, u_prev = [], 0.0

    for (speed, stick), (number, rerun) in zip(legs, leg_numbers(model, legs)):
        leg = leg_plan(model, params, speed, stick, u_prev, mode)
        leg.update(number=number, rerun=rerun)
        out.append(leg)
        u_prev = speed

    prologue = SETTLE_S + 10.0      # the EKF wait and the arming, about
    sim_typ = prologue + sum(p['sim_typical_s'] for p in out)
    sim_max = prologue + sum(p['sim_worst_s'] for p in out)
    return {'model': model, 'airframe': gz_model.airframe(model)[0], 'params': params, 'accel_mode': mode,
            'legs': out, 'sim_typical_s': sim_typ, 'sim_worst_s': sim_max,
            'wall_typical_s': BOOT_WALL_S + sim_typ / RTF + FINISH_WALL_S,
            'wall_worst_s': BOOT_WALL_S + sim_max / RTF + FINISH_WALL_S,
            'wall_worst_slow_s': BOOT_WALL_S + sim_max / RTF_SLOW + FINISH_WALL_S}


def feasibility(model, params, speeds=(1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0)):
    """The table the Cloudship's speeds were chosen from, recomputed."""
    rows = []
    pods = gz_model.pod_thrust(model)
    c = gz_model.translational_drag(gz_model.merged(model))[0]

    for u in speeds:
        trim = gz_model.trim_thrust(model, u)
        counts, _, cruise = quantize(trim)
        full, half = loop_reach(model, params, u, STICK_FULL), loop_reach(model, params, u, STICK_HALF)
        growth = lateral_growth(model, params, u)
        rows.append({'speed': u, 'trim': trim, 'counts': counts,
                     'holds': math.sqrt(cruise * pods / c) if c > 0.0 else None,
                     'open': lateral_growth(model, params, u, closed=False), 'closed': growth,
                     'growth_entry': math.exp(max(0.0, growth) * (STEADY_WINDOW_S + ENTRY_S)),
                     'full': full, 'half': half})

    return rows


def print_plan(plan, table=False):
    model = plan['model']
    p = plan['params']
    pods = gz_model.pod_thrust(model)
    c = gz_model.translational_drag(gz_model.merged(model))[0]
    d = gz_model.lateral_derivatives(model)
    print('기체 %s (posix %d), 가속 %s' % (model, plan['airframe'], plan['accel_mode']))
    print('  파일: 포드 추력 합 %.2f N, 서지 질량 %.3f kg, 선체 서지 항력 %s, 수직 핀 요 감쇠 %.1f u N m s/rad, '
          '요 권한 %.3f N m, n_v %+.2f u (핀 %.2f, Munk %.2f)'
          % (pods, gz_model.surge_mass(model), '%.4f u^2' % c if c > 0.0 else '없음 (무추력 활주)',
             gz_model.fin_yaw_damping(model), gz_model.yaw_authority(model), d['n_v'], d['y_r'], d['munk']))
    print('  펌웨어 (yaml + 에어프레임): P %g, I %g, INT_LIM %g, FF %g, MAX %g deg/s, DEADZONE %g, '
          'TLMIN/TLMAX %g/%g, GRP %g, TAIL %g, THR_MDL_FAC %g'
          % (p['AS_YAWRATE_P'], p['AS_YAWRATE_I'], p['AS_YR_INT_LIM'], p['AS_YAWRATE_FF'], p['AS_YAWRATE_MAX'],
             p['MAN_DEADZONE'], p['CA_AIRSHIP_TLMIN'], p['CA_AIRSHIP_TLMAX'], p['CA_AIRSHIP_GRP'],
             p['CA_AIRSHIP_TAIL'], p['THR_MDL_FAC']))

    for leg in plan['legs']:
        r = leg['reach']
        holds = ('항력 없음: 그 속도로 활주' if leg['holds_m_s'] is None
                 else '이 스로틀이 잡는 속도 %.3f m/s' % leg['holds_m_s'])
        print('  다리 %d%s: %.2f m/s, 요 스틱 %+.2f (지시 %.2f deg/s); 스로틀 %.4f -> %d/1000 = %.3f (%s); '
              '가속 스로틀 %.3f, %.1f -> %.1f m/s 약 %s s'
              % (leg['number'], '' if leg['rerun'] == str(leg['number']) else ' (다시 날기 FORWARD_LEGS=%s)' % leg['rerun'],
                 leg['speed'], leg['stick'], leg['setpoint_deg_s'], leg['trim'], leg['throttle_counts'],
                 leg['throttle'], holds, leg['accel_throttle'], leg['accel_from'], leg['speed'],
                 '?' if leg['accel_s'] is None else '%.0f' % leg['accel_s']))
        print('          파일의 1자유도 추정 (판정 아님): 적분기 한계에서 잡는 선회 %.2f deg/s (%s), '
              '지시를 잡을 적분기 %.3f (한계 %g); 그 선회의 수직 핀 받음각 %.1f deg (실속 %.1f)'
              % (r['reach'], '지시에 못 미쳐 적분기가 한계에 붙는다' if r['parks'] else '지시에 닿는다',
                 r['integrator'], p['AS_YR_INT_LIM'], r['alpha'], r['stall']))
        print('          선형 횡-요 (P, I 닫힘): 최대 실수부 %+.4f /s, 진입 측정 %.0f s 동안 x%.2f%s; '
              '시뮬레이터 시간 보통 %.0f s, 최대 %.0f s'
              % (leg['growth'], STEADY_WINDOW_S + ENTRY_S, leg['growth_entry'],
                 '' if leg['growth_entry'] <= GROWTH_LIMIT else ' -- 진입이 정상 직진이 되지 못할 것으로 본다',
                 leg['sim_typical_s'], leg['sim_worst_s']))

    print('  합계: 시뮬레이터 보통 %.0f s (%.1f 분), 최대 %.0f s; 벽시계 보통 %.0f s (%.1f 분), 최대 %.0f s '
          '(부팅 %.0f s, RTF %.1f, 마무리 %.0f s); 바깥 timeout 은 최대보다 길게, RTF %.1f 이면 %.0f s'
          % (plan['sim_typical_s'], plan['sim_typical_s'] / 60.0, plan['sim_worst_s'], plan['wall_typical_s'],
             plan['wall_typical_s'] / 60.0, plan['wall_worst_s'], BOOT_WALL_S, RTF, FINISH_WALL_S, RTF_SLOW,
             plan['wall_worst_slow_s']))

    if not table:
        return

    print('  속도 선택 표 (파일에서; 선택: %s m/s)' % ', '.join('%g' % s for s in leg_speeds(model)))

    for row in feasibility(model, p):
        print('    %.1f m/s: 트림 %.4f (%d/1000%s), 열린 루프 %+.3f /s, 닫힌 루프 %+.4f /s (진입 x%.2f), '
              '전타 %.2f/%.2f deg/s 핀 %.1f/%.1f deg, 반타 %.2f/%.2f deg/s'
              % (row['speed'], row['trim'], row['counts'],
                 '' if row['holds'] is None else ', %.3f m/s' % row['holds'], row['open'], row['closed'],
                 row['growth_entry'], row['full']['reach'], row['full']['setpoint'], row['full']['alpha'],
                 row['full']['stall'], row['half']['reach'], row['half']['setpoint']))


# ---------------------------------------------------------------------------
# The record: one ULog, segmented and measured. No px4, no gz: testable.
# ---------------------------------------------------------------------------

class Log:
    """The topics of TOPICS out of one ULog, by name, as numpy arrays.
    Timestamps are PX4's clock in microseconds; instance 0 of each topic."""

    def __init__(self, data, params, changed=(), info=None, dropouts=(), start_us=0, end_us=0, path=None):
        self.data = data
        self.params = params
        self.changed = list(changed)
        self.info = info or {}
        self.dropouts = list(dropouts)
        self.start_us = int(start_us)
        self.end_us = int(end_us)
        self.path = path

    @classmethod
    def read(cls, path):
        from pyulog import ULog

        ulog = ULog(path, list(TOPICS))
        data = {d.name: d.data for d in ulog.data_list if d.multi_id == 0}
        return cls(data, dict(ulog.initial_parameters), ulog.changed_parameters, dict(ulog.msg_info_dict),
                   [(d.timestamp, d.duration) for d in ulog.dropouts], ulog.start_timestamp,
                   ulog.last_timestamp, path)

    def has(self, topic):
        return topic in self.data and len(self.data[topic]['timestamp']) > 0

    def t(self, topic):
        return np.asarray(self.data[topic]['timestamp'], dtype=np.int64)

    def col(self, topic, name):
        return np.asarray(self.data[topic][name], dtype=float)

    def param(self, name):
        value = self.params.get(name)
        return None if value is None else float(value)


def mode_changes(log):
    """[(t_us, nav_state, armed)] wherever either changes, from vehicle_status."""
    out, previous = [], None

    for t, nav, arming in zip(log.t('vehicle_status'), log.col('vehicle_status', 'nav_state'),
                              log.col('vehicle_status', 'arming_state')):
        key = (int(nav), int(arming) == ARMED)

        if key != previous:
            out.append((int(t), key[0], key[1]))
            previous = key

    return out


def stick_segments(log, start_us, end_us, eps=1e-6):
    """[(on_us, off_us or None)] of the yaw stick between two times, from
    vehicle_rates_setpoint: the controller publishes it on every new stick
    while the loop is closed, with the setpoint exactly zero inside
    MAN_DEADZONE, so the first nonzero sample is the onset to within one
    stick message. None for an off that never came."""
    if not log.has('vehicle_rates_setpoint'):
        return []

    t = log.t('vehicle_rates_setpoint')
    yaw = log.col('vehicle_rates_setpoint', 'yaw')
    inside = (t >= start_us) & (t < end_us)
    segments, on = [], None

    for ti, y in zip(t[inside], yaw[inside]):
        moving = math.isfinite(y) and abs(y) > eps

        if moving and on is None:
            on = int(ti)
        elif not moving and on is not None:
            segments.append((on, int(ti)))
            on = None

    if on is not None:
        segments.append((on, None))

    return segments


def find_legs(log, release_s=RELEASE_S):
    """Every Acro interval entered from Manual while armed that carries a yaw
    stick, as a dict of PX4 times: manual_us (the Manual before it), acro_us,
    on_us, off_us, acro_end_us (the next mode or arming change), and
    release_end_us = the earlier of acro_end_us and off_us + release_s. An Acro
    interval without a stick (a brake) is not a leg; a second stick in one
    interval is counted in 'extra_sticks' and is no leg of its own (one that
    begins before the release's end is in that release's numbers, and the
    leg's run_leg is refused for it)."""
    changes = mode_changes(log)
    legs = []

    for i, (t, nav, armed) in enumerate(changes):
        if nav != NAV_ACRO or not armed or i == 0 or changes[i - 1][1] != NAV_MANUAL or not changes[i - 1][2]:
            continue

        end = changes[i + 1][0] if i + 1 < len(changes) else log.end_us
        segments = stick_segments(log, t, end)

        if not segments:
            continue

        on, off = segments[0]
        complete = off is not None
        off = off if complete else end
        legs.append({'manual_us': changes[i - 1][0], 'acro_us': t, 'on_us': on, 'off_us': off,
                     'off_seen': complete, 'acro_end_us': end,
                     'release_end_us': min(end, off + int(release_s * 1e6)),
                     'extra_sticks': len(segments) - 1})

    return legs


def zoh(ts, values, at):
    """Each value of `values` held from its timestamp to the next: the latest
    sample at or before each time of `at`, NaN before the first."""
    ts = np.asarray(ts)
    values = np.asarray(values, dtype=float)
    at = np.asarray(at)

    if len(ts) == 0:
        return np.full(len(at), np.nan)

    index = np.searchsorted(ts, at, side='right') - 1
    out = values[np.clip(index, 0, len(ts) - 1)].astype(float)
    out[index < 0] = np.nan
    return out


def euler(q0, q1, q2, q3):
    """Roll, pitch, yaw (rad) of PX4's FRD-to-NED quaternion arrays (w, x, y, z)."""
    roll = np.arctan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2))
    pitch = np.arcsin(np.clip(2.0 * (q0 * q2 - q3 * q1), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3))
    return roll, pitch, yaw


def airframe_outputs(model):
    """{'motor1': 'starboard pod motor', 'servo1': ...}: the `# @output` lines
    of the model's posix airframe and the files it sources (the physics
    variants source their vehicle's, which carries them), keyed by the CSV's
    column names. What each column is, in the airframe's own words."""
    romfs = os.path.join(gz_model.ROOT, 'ROMFS/px4fmu_common')
    out, todo = {}, [gz_model.airframe(model)[1]]

    while todo:
        with open(todo.pop(0)) as handle:
            for line in handle.read().splitlines():
                found = re.match(r'^#\s*@output\s+(Motor|Servo)(\d+)\s+(.+)$', line.strip())

                if found:
                    out['%s%s' % (found.group(1).lower(), found.group(2))] = found.group(3).strip()

                source = re.match(r'^\.\s+\$\{R\}etc/(init\.d-posix/airframes/\S+)$', line.strip())

                if source:
                    todo.append(os.path.join(romfs, source.group(1)))

    return out


def plant_info(model, params):
    """What the analysis needs to know about the plant, from the files and
    the logged allocation parameters: the actuator layout (the motors first,
    the tail after the pods; the tilt servos after CA_SV_CS_COUNT surfaces),
    the tilt range, the vertical fins, the surge numbers. A layout parameter
    the log does not carry is an error: guessing it would put the tail's
    column on a pod or a tilt in the wrong range."""
    layout = ('CA_AIRSHIP_TAIL', 'CA_AIRSHIP_GRP', 'CA_SV_CS_COUNT', 'CA_AIRSHIP_TLMIN', 'CA_AIRSHIP_TLMAX')
    missing = [k for k in layout if params.get(k) is None]

    if missing:
        raise LookupError('로그에 %s 이 없다: 액추에이터 배치를 알 수 없다' % ', '.join(missing))

    tail = int(params['CA_AIRSHIP_TAIL']) == 1
    independent = int(params['CA_AIRSHIP_GRP']) > 0
    info = {'model': model, 'tail': tail, 'independent': independent,
            'n_motors': 3 if tail else 2, 'n_tilts': 2 if independent else 1,
            'servo_offset': int(params['CA_SV_CS_COUNT']),
            'tilt_range': (params['CA_AIRSHIP_TLMIN'], params['CA_AIRSHIP_TLMAX']),
            'fins': [], 'pod_thrust': None, 'surge_drag': 0.0, 'surge_mass': None, 'outputs': {}}

    if model:
        info['fins'] = [{'cp': f['cp'], 'alpha_stall': f['alpha_stall']} for f in gz_model.vertical_fins(model)]
        info['pod_thrust'] = gz_model.pod_thrust(model)
        info['surge_drag'] = gz_model.translational_drag(gz_model.merged(model))[0]
        info['surge_mass'] = gz_model.surge_mass(model)
        info['com'] = gz_model.centre_of_mass(gz_model.family(model))[1]
        info['outputs'] = airframe_outputs(model)
        # Each fin by yaw_model/planar.py's name, which forward.fin_names
        # repeats: dorsal when its cp is above the family's centre of mass,
        # else ventral. Two fins on one side would be one name, and one key,
        # there; here each then takes its file-order number (dorsal1,
        # dorsal2) to keep two CSV columns, and matches no model fin by
        # name. None of the four plants has two (the test holds that)
        names = ['dorsal' if f['cp'][2] > info['com'][2] else 'ventral' for f in info['fins']]

        for i, (fin, plugin) in enumerate(zip(info['fins'], gz_model.vertical_fins(model))):
            fin['name'] = names[i] if names.count(names[i]) == 1 else '%s%d' % (names[i], i + 1)
            fin.update(fin_axes(plugin))

    return info


def fin_axes(plugin):
    """{'forward', 'upward', 'a0'} of one lift-drag plugin (gz_model.
    lift_drag_fins' record), which decide where it has flow and whether its
    angle is atan2(v_cp, u_cp) (leg_table): forward as the file gives it, or
    None; upward None where radial_symmetry makes the plugin take it from the
    inflow; a0 the plugin's default 0 where the file has none (LiftDrag.cc
    alpha0, radialSymmetry)."""
    forward = plugin['elem'].find('forward')
    radial = plugin['elem'].find('radial_symmetry')
    symmetric = radial is not None and (radial.text or '').strip().lower() in ('true', '1')
    return {'forward': None if forward is None else tuple(float(x) for x in forward.text.split()),
            'upward': None if symmetric else tuple(plugin['upward']),
            'a0': 0.0 if plugin['a0'] is None else float(plugin['a0'])}


def body_frd(q, north, east, down):
    """(u, v, w) in body FRD of NED vector arrays: R(q)^T v, with q PX4's
    FRD-to-NED quaternion arrays (w, x, y, z), normalized here. NaN where any
    input is."""
    w, x, y, z = (np.asarray(c, dtype=float) for c in q)
    n = np.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    u = (1.0 - 2.0 * (y * y + z * z)) * north + 2.0 * (x * y + w * z) * east + 2.0 * (x * z - w * y) * down
    v = 2.0 * (x * y - w * z) * north + (1.0 - 2.0 * (x * x + z * z)) * east + 2.0 * (y * z + w * x) * down
    d = 2.0 * (x * z + w * y) * north + 2.0 * (y * z - w * x) * east + (1.0 - 2.0 * (x * x + y * y)) * down
    return u, v, d


def fin_names(plant):
    return ['alpha_fin%d_deg' % (i + 1) for i in range(len(plant['fins']))]


def tilt_names(plant):
    if plant['independent']:
        return ['tilt_stbd_deg', 'tilt_port_deg']
    return ['tilt_deg']


def leg_table(log, leg, plant):
    """{column: array} on the groundtruth samples from STEADY_WINDOW_S before
    the Acro entry to the release end, every other topic held at its latest
    sample (zoh). 't_s' is relative to the stick onset."""
    a = max(log.start_us, leg['acro_us'] - int(STEADY_WINDOW_S * 1e6))
    b = leg['release_end_us']
    gt = log.t(GT_POS)
    keep = (gt >= a) & (gt <= b)
    T = gt[keep]
    cols = {}
    cols['t_s'] = (T - leg['on_us']) / 1e6
    cols['t_px4_us'] = T.astype(float)
    cols['phase'] = np.where(T < leg['acro_us'], 'steady',
                             np.where(T < leg['on_us'], 'entry', np.where(T < leg['off_us'], 'stick', 'release')))
    cols['nav_state'] = zoh(log.t('vehicle_status'), log.col('vehicle_status', 'nav_state'), T)
    mode_t = log.t('vehicle_control_mode')
    closed = (log.col('vehicle_control_mode', 'flag_armed') * log.col('vehicle_control_mode', 'flag_control_manual_enabled')
              * log.col('vehicle_control_mode', 'flag_control_rates_enabled'))
    cols['loop_closed'] = zoh(mode_t, closed, T)
    q = [zoh(log.t(GT_ATT), log.col(GT_ATT, 'q[%d]' % i), T) for i in range(4)]
    roll, pitch, yaw = euler(*q)
    psi = np.unwrap(yaw) if len(yaw) else yaw
    vx, vy = log.col(GT_POS, 'vx')[keep], log.col(GT_POS, 'vy')[keep]
    cols['u'] = vx * np.cos(psi) + vy * np.sin(psi)
    cols['v'] = -vx * np.sin(psi) + vy * np.cos(psi)
    cols['w_down'] = log.col(GT_POS, 'vz')[keep]
    cols['alt_m'] = -log.col(GT_POS, 'z')[keep]

    if log.has('vehicle_local_position'):
        lp = log.t('vehicle_local_position')
        hv = zoh(lp, log.col('vehicle_local_position', 'heading'), T)
        ex, ey = zoh(lp, log.col('vehicle_local_position', 'vx'), T), zoh(lp, log.col('vehicle_local_position', 'vy'), T)
        cols['u_ekf'] = ex * np.cos(hv) + ey * np.sin(hv)
        cols['v_ekf'] = -ex * np.sin(hv) + ey * np.cos(hv)
    else:
        cols['u_ekf'] = cols['v_ekf'] = np.full(len(T), np.nan)

    av = log.t('vehicle_angular_velocity')
    p_rad = zoh(av, log.col('vehicle_angular_velocity', 'xyz[0]'), T)
    r_rad = zoh(av, log.col('vehicle_angular_velocity', 'xyz[2]'), T)
    cols['p_deg_s'] = np.degrees(p_rad)
    cols['r_deg_s'] = np.degrees(r_rad)
    # the plant's own heading rate, beside the gyro's: equal to r while the
    # hull is level; NaN where two groundtruth samples share a time
    if len(T) > 2 and np.all(np.diff(T) > 0):
        cols['r_hdg_deg_s'] = np.degrees(np.gradient(psi, T / 1e6))
    else:
        cols['r_hdg_deg_s'] = np.full(len(T), np.nan)
    cols['heading_deg'] = np.degrees(psi)
    cols['roll_deg'] = np.degrees(roll)
    cols['pitch_deg'] = np.degrees(pitch)
    cols['yaw_stick'] = zoh(log.t('manual_control_setpoint'), log.col('manual_control_setpoint', 'yaw'), T)
    sp = np.degrees(zoh(log.t('vehicle_rates_setpoint'), log.col('vehicle_rates_setpoint', 'yaw'), T)) \
        if log.has('vehicle_rates_setpoint') else np.full(len(T), np.nan)
    # published only while the loop is closed: a held value from the last
    # closed phase is not a setpoint
    cols['yaw_rate_sp_deg_s'] = np.where(cols['loop_closed'] > 0.5, sp, np.nan)
    cols['throttle'] = zoh(log.t('vehicle_thrust_setpoint'), log.col('vehicle_thrust_setpoint', 'xyz[0]'), T)
    cols['torque_z'] = zoh(log.t('vehicle_torque_setpoint'), log.col('vehicle_torque_setpoint', 'xyz[2]'), T)
    cols['yawspeed_integ'] = zoh(log.t('rate_ctrl_status'), log.col('rate_ctrl_status', 'yawspeed_integ'), T)
    cas = log.t('control_allocator_status')
    unalloc = zoh(cas, log.col('control_allocator_status', 'unallocated_torque[2]'), T)
    achieved = zoh(cas, log.col('control_allocator_status', 'torque_setpoint_achieved'), T)
    # what AirshipYawRateLoop::setSaturation hands RateControl
    shortfall = np.where(achieved > 0.5, 0.0, unalloc)
    cols['unalloc_yaw'] = unalloc
    cols['torque_achieved'] = achieved
    cols['sat_pos'] = (shortfall > FLAG_EPS).astype(float)
    cols['sat_neg'] = (shortfall < -FLAG_EPS).astype(float)
    motors = log.t('actuator_motors')

    for i in range(plant['n_motors']):
        cols['motor%d' % (i + 1)] = zoh(motors, log.col('actuator_motors', 'control[%d]' % i), T)

    servos = log.t('actuator_servos')
    lo, hi = plant['tilt_range']

    for i, name in enumerate(tilt_names(plant)):
        raw = zoh(servos, log.col('actuator_servos', 'control[%d]' % (plant['servo_offset'] + i)), T)
        cols['servo%d' % (i + 1)] = raw
        # AirshipPod::setServoSetpoint: tilt = min + (servo + 1) / 2 (max - min)
        cols[name] = lo + (raw + 1.0) * 0.5 * (hi - lo) if lo is not None and hi is not None else np.full(len(T), np.nan)

    for name, fin in zip(fin_names(plant), plant['fins']):
        # the velocity at the cp, from the origin's: v + (w x cp)_y, FRD, the
        # cp read off the model in FLU (x forward, z up)
        x, z = fin['cp'][0], fin['cp'][2]
        cols[name] = np.degrees(np.arctan2(cols['v'] + r_rad * x + p_rad * z, cols['u']))

    # Appended, so every column above keeps its place. t_acro_s: the replay
    # block's clock and the model's, s from the Acro switch
    cols['t_acro_s'] = (T - leg['acro_us']) / 1e6
    q_rad = zoh(av, log.col('vehicle_angular_velocity', 'xyz[1]'), T)
    # The fins by name read the flow as the lift-drag plugin does, in the
    # link's own frame: the groundtruth's NED velocity turned into body FRD
    # by the full groundtruth attitude (u and v above are the heading
    # frame's, which leave the pitch and the roll out), relative to still
    # air (the lta world sets no wind)
    u_b, v_b, w_b = body_frd(q, vx, vy, cols['w_down'])

    for fin in plant['fins']:
        # Each fin by its name (plant_info) where the plugin has flow. The
        # velocity at the cp is the body velocity plus w x cp: u_cp = u_b +
        # r y - q z, v_cp = v_b + r x + p z, w_cp = w_b - p y - q x (the
        # gyro's FRD rates, the cp in FLU). The plugin returns before any
        # force where that is no faster than FIN_MIN_SPEED or has no
        # positive part along the fin's forward, body x (LiftDrag.cc): flow
        # 0 and no angle. The angle is atan2(v_cp, u_cp), the plugin's own
        # for forward +x, upward +y and a0 0, FRD-signed as the alpha_fin<n>
        # columns, which take the heading frame's u and v and leave r y - q z
        # out. A fin with other axes or another a0 is not read: both of its
        # columns stay unread
        x, y, z = fin['cp']
        flow, alpha = np.full(len(T), np.nan), np.full(len(T), np.nan)

        if (fin.get('forward'), fin.get('upward'), fin.get('a0')) == ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), 0.0):
            u_cp = u_b + r_rad * y - q_rad * z
            v_cp = v_b + r_rad * x + p_rad * z
            w_cp = w_b - p_rad * y - q_rad * x
            speed = np.sqrt(u_cp * u_cp + v_cp * v_cp + w_cp * w_cp)
            flow = np.where(np.isfinite(u_cp) & np.isfinite(speed),
                            ((u_cp > 0.0) & (speed > FIN_MIN_SPEED)).astype(float), np.nan)
            alpha = np.where(flow > 0.5, np.degrees(np.arctan2(v_cp, u_cp)), np.nan)

        cols['fin_%s_alpha_deg' % fin['name']] = alpha
        cols['fin_%s_flow' % fin['name']] = flow

    return cols


def rows_between(table, a_us, b_us):
    """Boolean mask of the table rows with a <= t < b (PX4 microseconds)."""
    t = table['t_px4_us']
    return (t >= a_us) & (t < b_us)


def steady_judgement(t_s, u, v, r_deg, target):
    """(ok, numbers) for a window of straight flight at `target` m/s: the mean
    speed within tolerance, its last second against its first within the
    drift tolerance, |r| and |v| small everywhere. Live and in the log alike,
    so the run waits for the same thing the verdict later reads. A window
    with an unread sample fails, and so does a target that is not a number:
    every comparison against NaN is False, and None has no speed."""
    t_s, u, v, r_deg = (np.asarray(x, dtype=float) for x in (t_s, u, v, r_deg))
    numbers = {'n': int(len(t_s)), 'span_s': float(t_s[-1] - t_s[0]) if len(t_s) else 0.0,
               'u_mean': None, 'u_drift': None, 'r_max_abs': None, 'v_max_abs': None, 'target': target}

    if len(t_s) < 3 or not (isinstance(target, (int, float)) and math.isfinite(target)):
        return False, numbers

    first, last = u[t_s <= t_s[0] + 1.0], u[t_s >= t_s[-1] - 1.0]
    numbers.update(u_mean=float(np.mean(u)), u_drift=float(np.mean(last) - np.mean(first)),
                   r_max_abs=float(np.max(np.abs(r_deg))), v_max_abs=float(np.max(np.abs(v))))
    every = np.concatenate([u, v, r_deg])

    if not np.all(np.isfinite(every)):
        return False, numbers

    # On the estimator's frame a heading off by delta leaks u sin(delta) into
    # v as a constant: 0.35 m/s at 4 m/s and 5 deg in SITL. That is not
    # motion, so there the sway is judged on its variation about the window's
    # mean; the groundtruth's reads the sway itself
    numbers['v_var_abs'] = float(np.max(np.abs(v - np.mean(v))))
    sway = numbers['v_var_abs'] if GT_POS == ESTIMATOR_FRAME[0] else numbers['v_max_abs']
    ok = (abs(numbers['u_mean'] - target) <= max(SPEED_TOL * target, SPEED_TOL_ABS)
          and abs(numbers['u_drift']) <= max(DRIFT_TOL * target, DRIFT_TOL_ABS)
          and numbers['r_max_abs'] < RATE_TOL_DEG and sway < SWAY_TOL)
    return bool(ok), numbers


def pod_reversals(t, tilt_deg, a, b, band=REVERSAL_COS):
    """(into_rear, back_forward, rear_s) of one pod between PX4 times a and b.

    Tilt 0 thrusts forward and +-180 straight back (AirshipPod.hpp). The count
    is yaw_model/forward.py's: a Schmitt trigger on cos(tilt), forward above
    +band and back below -band (60 and 120 deg at 0.5), so a pod passing
    through the vertical counts once and one dithering anywhere between the two
    does not count at all. The state at `a` is the last definite one before
    it, as forward.py's trigger carries its state into each phase. rear_s
    runs to b: the state of the last sample before b holds until b, as
    forward.py counts every tick to the phase's end."""
    t = np.asarray(t)
    c = np.cos(np.radians(np.asarray(tilt_deg, dtype=float)))
    state, into, back, rear = None, 0, 0, 0.0
    previous = None

    for ti, ci in zip(t, c):
        side = 'fwd' if ci > band else ('rear' if ci < -band else None)

        if ti >= b:
            break

        if ti >= a and state is not None and side is not None and side != state:
            into += side == 'rear'
            back += side == 'fwd'

        if ti >= a and previous is not None and state == 'rear':
            rear += (ti - max(previous, a)) / 1e6

        state = side or state
        previous = ti

    # the interval after the last sample before b, held in its state
    if state == 'rear' and previous is not None and b > max(previous, a):
        rear += (b - max(previous, a)) / 1e6

    return into, back, rear


def sign_reversals(t, value, a, b, deadband=TAIL_DEADBAND):
    """Direction changes of a reversible command (the Cloudship's tail)
    between PX4 times a and b; inside the deadband it is neither direction,
    and the direction at `a` is the last definite one before it (forward.py's
    trigger at TAIL_BAND, the same count)."""
    state, flips = None, 0

    for ti, x in zip(np.asarray(t), np.asarray(value, dtype=float)):
        if ti >= b:
            break

        side = 1 if x > deadband else (-1 if x < -deadband else None)

        if ti >= a and state is not None and side is not None and side != state:
            flips += 1

        state = side or state

    return flips


def pod_modes(log, plant, params, at):
    """[(mode, demand)] per allocator pod (starboard, port; on the Cloudship
    the collective pair, twice) at PX4 time `at`: the mode AirshipPod::steer
    is in, by its own rule run on the logged demand from the log's start to
    `at` (AirshipPod.cpp: steering above POD_ENGAGE; holding at or below it
    and above POD_RELEASE once engaged; released below that, on a NaN, or
    never engaged; parked while disarmed), and that demand's magnitude at
    `at`. The demand is the setpoints as ActuatorEffectivenessAirship::
    updateSetpoint splits them: common (thrust x, -thrust z), less (yaw,
    roll) torque on the starboard pod and plus it on the port pod of an
    independent pair, the common one alone on a collective pair; float32,
    as the firmware computes and compares it.

    The setpoints are logged every 20 ms (SITL's profile) against the
    allocator's 4 ms: a crossing shorter than that is not seen. None as a
    whole when a setpoint is not logged or control surfaces take a share
    (CA_SV_CS_COUNT with CA_AIRSHIP_CS_K credits their torque, which the
    log does not give); a pod's mode None when its demand at `at` is
    unread."""
    surfaces, credit = params.get('CA_SV_CS_COUNT'), params.get('CA_AIRSHIP_CS_K')

    if surfaces is None or (surfaces > 0 and (credit is None or credit != 0.0)):
        return None

    need = (('vehicle_thrust_setpoint', ('xyz[0]', 'xyz[2]')), ('vehicle_torque_setpoint', ('xyz[0]', 'xyz[2]')),
            ('vehicle_status', ('arming_state',)))

    if any(not log.has(topic) or any(f not in log.data[topic] for f in fields) for topic, fields in need):
        return None

    times = np.union1d(*(log.t(topic)[log.t(topic) <= at] for topic in ('vehicle_thrust_setpoint',
                                                                       'vehicle_torque_setpoint')))

    if not len(times):
        return None

    held = lambda topic, field: zoh(log.t(topic), log.col(topic, field), times).astype(np.float32)
    forward, up = held('vehicle_thrust_setpoint', 'xyz[0]'), -held('vehicle_thrust_setpoint', 'xyz[2]')
    roll, yaw = held('vehicle_torque_setpoint', 'xyz[0]'), held('vehicle_torque_setpoint', 'xyz[2]')
    armed = zoh(log.t('vehicle_status'), log.col('vehicle_status', 'arming_state'), times) == ARMED
    pods = ([(forward - yaw, up - roll), (forward + yaw, up + roll)] if plant['independent']
            else [(forward, up), (forward, up)])
    out = []

    for x, y in pods:
        magnitude = np.sqrt(x * x + y * y)          # Vector2f::norm, float32
        mode = 'parked'

        for m, on in zip(magnitude, armed):
            if not on:
                mode = 'parked'
            elif m > POD_ENGAGE:
                mode = 'steering'
            elif mode in ('steering', 'holding') and m > POD_RELEASE:
                mode = 'holding'
            else:
                mode = 'released'

        last = float(magnitude[-1])
        out.append((mode if math.isfinite(last) else None, last if math.isfinite(last) else None))

    return out


def samples(log, topic, field, a, b):
    t = log.t(topic)
    keep = (t >= a) & (t < b)
    return t[keep], log.col(topic, field)[keep]


def last_at(log, topic, field, at):
    """(value, t_us) of the latest sample at or before `at`, or (None, None)."""
    if not log.has(topic):
        return None, None

    t = log.t(topic)
    i = int(np.searchsorted(t, at, side='right')) - 1
    return (float(log.col(topic, field)[i]), int(t[i])) if i >= 0 else (None, None)


def first_at(log, topic, field, at):
    """(value, t_us) of the earliest sample at or after `at`, or (None, None)."""
    if not log.has(topic):
        return None, None

    t = log.t(topic)
    i = int(np.searchsorted(t, at, side='left'))
    return (float(log.col(topic, field)[i]), int(t[i])) if i < len(t) else (None, None)


def mean_or_none(x):
    x = np.asarray(x, dtype=float)
    return float(np.mean(x)) if len(x) and np.all(np.isfinite(x)) else None


def at_time(table, column, t_us):
    """The table's value at or just before a PX4 time, or None."""
    t = table['t_px4_us']
    i = int(np.searchsorted(t, t_us, side='right')) - 1
    value = float(table[column][i]) if i >= 0 else float('nan')
    return value if math.isfinite(value) else None


def gyro_mean(log, a, b):
    """Mean yaw rate, deg/s, of the gyro samples between two PX4 times."""
    _, r = samples(log, 'vehicle_angular_velocity', 'xyz[2]', a, b)
    return None if not len(r) else float(np.degrees(np.mean(r)))


def gyro_at(log, when):
    """The yaw rate, deg/s, of the gyro sample at or before a PX4 time, or
    None: one sample of the filtered rate the loop reads, with SITL's gyro
    noise (about 0.04 deg/s, against 0.004 for a 1 s mean). Not what
    yaw_model/forward.py's at() reads at a mark (the plant's true rate at the
    nearest step: no filter, no noise); a model is held against the 1 s
    mean."""
    value, _ = last_at(log, 'vehicle_angular_velocity', 'xyz[2]', when)
    return None if value is None or not math.isfinite(value) else math.degrees(value)


def schedule_on(sched):
    """The firmware's switch (airship_att_control_main.cpp): a speed source
    and a positive slope. Its third condition, no yaw control surfaces, shows
    as a published cap equal to AS_YAWRATE_MAX."""
    src, slope = sched.get('AS_SPD_SRC'), sched.get('AS_YR_FF_SLP')
    return bool(src) and slope is not None and slope > 0.0


def source_loss_s(env=None):
    """FORWARD_SOURCE_LOSS_S as seconds into the stick, None when unset;
    ValueError for a value that is not a number."""
    value = (os.environ if env is None else env).get(SOURCE_LOSS_ENV)
    return None if value in (None, '') else float(value)


def schedule_metrics(log, leg, params, sched, stick_value, lost_us=None):
    """The schedule's numbers in one leg, from airship_speed: the interval,
    the feedforward, the cap and the source at the stick, at the release and
    at the release's end; over the stick's last 5 s, the rate setpoint against
    the stick's share of the cap the firmware published (each setpoint sample
    against the cap sample at or before it); after a source loss at
    `lost_us`, when the selector reported no source."""
    on, off, end = leg['on_us'], leg['off_us'], leg['release_end_us']
    out = {'on': schedule_on(sched), 'logged': log.has(SPEED_TOPIC), 'params': dict(sched)}

    if not out['logged']:
        return out

    def at(when):
        read = lambda f: last_at(log, SPEED_TOPIC, f, when)[0]
        cap, src, held = read('yaw_rate_cap'), read('source'), read('upper_held')
        return {'u_lo': read('u_lo'), 'u_hi': read('u_hi'), 'ff': read('yaw_rate_ff'),
                'cap_deg_s': None if cap is None else math.degrees(cap), 'authority': read('yaw_authority'),
                'source': None if src is None else SPEED_SOURCES.get(int(src), int(src)),
                'upper_held': None if held is None else bool(held)}

    out.update(at_stick=at(on), at_release=at(off), at_end=at(end))
    _, cap = samples(log, SPEED_TOPIC, 'yaw_rate_cap', on, off)
    out['cap_stick_min_deg_s'] = float(np.degrees(np.min(cap))) if len(cap) else None
    out['cap_stick_max_deg_s'] = float(np.degrees(np.max(cap))) if len(cap) else None
    _, src = samples(log, SPEED_TOPIC, 'source', on, end + 1)
    out['sources_seen'] = sorted({SPEED_SOURCES.get(int(x), int(x)) for x in src})
    t_sp, sp = samples(log, 'vehicle_rates_setpoint', 'yaw', off - int(5.0 * 1e6), off)
    want = err = None

    if len(t_sp) and stick_value is not None and params.get('MAN_DEADZONE') is not None:
        t_cap, caps = log.t(SPEED_TOPIC), log.col(SPEED_TOPIC, 'yaw_rate_cap')
        i = np.searchsorted(t_cap, t_sp, side='right') - 1

        if np.all(i >= 0):
            want = np.array([setpoint_deg(stick_value, {'MAN_DEADZONE': params['MAN_DEADZONE'],
                                                        'AS_YAWRATE_MAX': math.degrees(c)}) for c in caps[i]])
            err = np.degrees(sp) - want

    out.update(setpoint_last5_deg_s=float(np.degrees(np.median(sp))) if len(sp) else None,
               setpoint_from_cap_last5_deg_s=None if want is None else float(np.median(want)),
               setpoint_minus_cap_share_max_abs_deg_s=None if err is None else float(np.max(np.abs(err))),
               setpoint_samples_last5=int(len(sp)))

    if lost_us is not None:
        t_s, after = samples(log, SPEED_TOPIC, 'source', lost_us, off)
        none_at = next((int(t) for t, x in zip(t_s, after) if int(x) == 0), None)
        out['loss'] = {'at_px4_us': int(lost_us), 'after_stick_on_s': (lost_us - on) / 1e6,
                       'source_none_after_s': None if none_at is None else (none_at - lost_us) / 1e6,
                       'before': at(lost_us)}

    return out


def schedule_checks(name, m, lost):
    """The schedule's verdicts on one leg flown with it on: the topic is
    there, the stick commanded no more than its share of the published cap,
    the integrator stayed off its limit, and either the yaw rate reached the
    setpoint (a leg whose interval was past AS_SPD_HI at the stick) or, after
    a source loss, the selector reported none and held the upper bound."""
    fmt = lambda x, f='%.3f': '?' if x is None else f % x
    s, sp = m['schedule'], m['stick_phase']
    rel = s.get('at_release') or {}
    ok = bool(s['logged']) and rel.get('cap_deg_s') is not None
    out = [('%s: airship_speed 가 로그에 있다' % name, ok, 'logged %s, 놓을 때 상한 %s deg/s'
            % (s['logged'], fmt(rel.get('cap_deg_s'))))]

    if not ok:
        return out

    # The schedule needs a speed: without one it runs at its at-rest values
    # (AS_YAWRATE_FF, AS_YAWRATE_MAX) and the leg measures HEAD with the
    # restore rule. The ground source needs the EKF's horizontal velocity
    # valid and not dead-reckoned; until 2026-10-07 it also needed
    # heading_good_for_control, which a hull spawned in the air waited some
    # 200 s for, and the first two legs of that day flew without a source
    at_stick = s.get('at_stick') or {}
    out.append(('%s: 스틱을 걸 때 속도원이 있었다' % name, at_stick.get('source') not in (None, 'none'),
                '원천 %s, u %s..%s m/s' % (at_stick.get('source'), fmt(at_stick.get('u_lo')), fmt(at_stick.get('u_hi')))))
    err, want = s['setpoint_minus_cap_share_max_abs_deg_s'], s['setpoint_from_cap_last5_deg_s']
    tol = None if want is None else max(SCHED_SP_TOL[0], SCHED_SP_TOL[1] * abs(want))
    out.append(('%s: 스틱이 상한의 제 몫만 지시했다 (마지막 5 s)' % name, err is not None and err <= tol,
                '설정 중앙값 %s, 상한의 몫 %s deg/s, 표본마다 최대 차이 %s (허용 %s), 표본 %d'
                % (fmt(s['setpoint_last5_deg_s']), fmt(want), fmt(err), fmt(tol), s['setpoint_samples_last5'])))
    out.append(('%s: 스틱 동안 적분기가 한계에 닿지 않았다' % name, sp['integrator_reached_limit'] is False,
                '최대 |적분기| %s, 한계 %s' % (fmt(sp['integrator_max_abs'], '%.4f'), fmt(sp['integrator_limit'], '%.2f'))))

    # Inside the ramp the feedforward holds only part of the turn and the
    # integrator builds the rest at its own pace (tens of seconds): the rate
    # is judged against the setpoint only where the whole interval was past
    # the ramp at the stick
    u_lo, ramp_end = (s.get('at_stick') or {}).get('u_lo'), s['params'].get('AS_SPD_HI')
    past_ramp = u_lo is not None and ramp_end is not None and u_lo >= ramp_end

    if lost is None:
        if past_ramp:
            r5, spv = sp['r_mean_last5_deg_s'], s['setpoint_last5_deg_s']
            tol = None if spv is None else max(SCHED_R_TOL[0], SCHED_R_TOL[1] * abs(spv))
            out.append(('%s: 요 각속도가 설정에 닿았다 (마지막 5 s)' % name,
                        r5 is not None and spv is not None and abs(r5 - spv) <= tol,
                        'r %s, 설정 %s deg/s (허용 %s)' % (fmt(r5), fmt(spv), fmt(tol))))
    else:
        loss = s.get('loss') or {}
        ok = (loss.get('source_none_after_s') is not None and rel.get('source') == 'none'
              and rel.get('upper_held') is True)
        out.append(('%s: 속도원을 잃자 원천이 none 이 되고 상한 구간이 유지됐다' % name, ok,
                    '잃은 뒤 none 까지 %s s, 놓을 때 원천 %s, u_hi 유지 %s, u_lo %s -> %s, u_hi %s -> %s m/s, '
                    '상한 %s -> %s deg/s' % (fmt(loss.get('source_none_after_s'), '%.2f'), rel.get('source'),
                                            rel.get('upper_held'), fmt((loss.get('before') or {}).get('u_lo')),
                                            fmt(rel.get('u_lo')), fmt((loss.get('before') or {}).get('u_hi')),
                                            fmt(rel.get('u_hi')), fmt((loss.get('before') or {}).get('cap_deg_s')),
                                            fmt(rel.get('cap_deg_s')))))

    return out


def definitions():
    """What the summary's counted and sampled numbers are, in words, from the
    constants that make them: the summary carries this beside the numbers."""
    return {
        'reversals': "yaw_model/forward.py's count. Per pod a Schmitt trigger on cos(tilt) at +-%g: forward above "
                     "+%g (|tilt| < %.0f deg), back below -%g (|tilt| > %.0f deg), its state carried into each phase; "
                     "into_rear and back_forward are its flips each way inside the phase (the stick: onset to "
                     "release; the release: release to its end, the release instant in it, where forward.py "
                     "counts it in the stick), rear_s the time it held 'back', and 'flips' sums the pods' flips. "
                     "The tilt is the commanded one: actuator_servos through CA_AIRSHIP_TLMIN/TLMAX. "
                     "Cloudship: tail_flips (= flips) counts the tail command's sign with a +-%g band."
                     % (REVERSAL_COS, REVERSAL_COS, math.degrees(math.acos(REVERSAL_COS)), REVERSAL_COS,
                        math.degrees(math.acos(-REVERSAL_COS)), TAIL_DEADBAND),
        'release marks': "release.r_deg_s: the one gyro sample at or before each mark (vehicle_angular_velocity, the "
                         "filtered rate the loop reads, with SITL's gyro noise: about 0.04 deg/s). That is not what "
                         "yaw_model/forward.py's at() reads (the plant's true rate at the nearest step, no filter, no "
                         "noise): hold a model's rate against release.r_mean1s_deg_s, the gyro mean over the second "
                         "ending at the mark (about 0.004 deg/s of noise), with the model's own 1 s mean of its "
                         "measured rate over the same window. release.integrator: the rate_ctrl_status sample at or "
                         "before it; release.marks_after_s: when each was taken. The last mark is the release's end: "
                         "a release up to %g s short of its length (the duration verdict's allowance) has it taken "
                         "there." % DURATION_TOL_S,
        'means': "stick_phase.r_mean_last5_deg_s: the gyro mean over the stick's last 5 s; u_start and u_end: the "
                 "groundtruth's mean over 0.5 s; replay.throttle: the thrust setpoint's mean from the Acro entry to "
                 "the release's end.",
        'states': "entry.at_acro, entry.at_stick, release.entry: the table row at or before the Acro switch, the "
                  "stick onset and the stick release (groundtruth rows, every other topic held at its latest "
                  "sample, so the integrator in a state is up to one rate_ctrl_status interval old).",
        'flags': "sat_pos/sat_neg and flagged_fraction: AirshipYawRateLoop::setSaturation's rule, a yaw shortfall "
                 "(unallocated_torque[2] while torque_setpoint_achieved is false) strictly past FLT_EPSILON %.3g."
                 % FLAG_EPS,
        # each key below names a field added on 2026-10-01 (step 0b's model
        # interface); every field above keeps its meaning
        'smoothed peak': "stick_phase.r_peak_smoothed_deg_s: the extreme of a 25-row moving mean of the table's gyro "
                         "column, 0.5 s only where the groundtruth rows are 20 ms apart (kept as it was); "
                         "r_peak_smoothed_05s_deg_s: the extreme, in the stick's direction, of the mean of the gyro "
                         "samples themselves over a trailing %g s TIME window, every window wholly inside the stick, "
                         "whatever the logging interval (yaw_model/forward.py's r_peak_smoothed is this on its "
                         "measured rate); r_peak_smoothed_05s_after_s: that window's end, s after the onset."
                         % PEAK_WINDOW_S,
        'tail flips (torque)': "Cloudship: tail_flips_torque counts the sign of vehicle_torque_setpoint xyz[2] with the "
                               "same +-%g band, on its own samples (every 20 ms on SITL's profile) from the leg table's "
                               "first row, beside tail_flips on actuator_motors control[2] (every 100 ms). The "
                               "collective pods' yaw is zero, so the tail command is clamp(yaw torque) "
                               "(ActuatorEffectivenessAirship::updateSetpoint): one sign and one band, and a flip "
                               "shorter than 100 ms is in this count and not in tail_flips." % TAIL_DEADBAND,
        'rear_s': "reversals.<pod>.rear_s runs to the phase's end: the state of the last sample before it holds until "
                  "it (until 2026-10-01 the last interval, up to one 20 ms row, was left out).",
        'fins by name': "plant.vertical_fins[i].name, and the leg CSV's fin_<name>_alpha_deg and fin_<name>_flow, name "
                        "each vertical fin as yaw_model/planar.py and forward.fin_names do: dorsal when its cp is "
                        "above the family's centre of mass, else ventral. Two fins on one side would share that "
                        "name, one key in planar.py; here each takes its file-order number (dorsal1, dorsal2) and "
                        "matches no model fin by name (none of the four plants has two). The flow at the cp is "
                        "read as the lift-drag plugin reads it, in the link's frame: the groundtruth's NED velocity "
                        "turned into body FRD by the full groundtruth attitude (pitch and roll in; the lta world "
                        "has no wind), plus the gyro's rotation, u_cp = u_b + r y - q z, v_cp = v_b + r x + p z, "
                        "w_cp = w_b - p y - q x (FRD rates, the cp in FLU). flow 1: faster than %g m/s and u_cp, "
                        "along the fin's forward (body x), positive; 0: either not, where the plugin returns "
                        "before any force (LiftDrag.cc); empty: unread, or a fin whose forward, upward or a0 is "
                        "not +x, +y, 0 (all four plants' are; plant.vertical_fins carries them). The angle, only "
                        "where there is flow: atan2(v_cp, u_cp), the plugin's own for those axes, FRD-signed as "
                        "alpha_fin<n>, which take the heading frame's u and v and leave r y - q z out. Until "
                        "2026-10-02 these columns took the heading frame's u and v too: on the Cloudship, 2-3 deg "
                        "nose down, a few rows near u_cp = 0 read otherwise. Hold only |alpha| / alpha_stall "
                        "against a model. Each phase's fins entry adds name, max_ratio "
                        "(the largest |alpha| / alpha_stall where there is flow: forward.py's "
                        "fin_max_alpha_ratio), no_flow_rows, and branch_end ('attached', 'stalled' or 'no flow' on "
                        "the phase's last row: forward.py's fins_branch); its max_abs_deg and stalled are as they "
                        "were, over every row, flow or not." % FIN_MIN_SPEED,
        'integrator first': "replay.integrator: the first rate_ctrl_status sample at or after the Acro switch, up to "
                            "one 200 ms interval after it (replay.integrator_first_after_s: when). At the switch "
                            "the firmware's integrator is 0, reset when the loop opened in Manual; replay.run_leg "
                            "passes that 0 as integ0 only when the first sample is within %g." % INTEG_CLEAR,
        'throttle counts': "replay.throttle_counts: the MANUAL_CONTROL count (z, 0..1000; thrust setpoint = z / "
                           "1000) every thrust setpoint from the Acro switch to the release's end rounds to; None "
                           "when they do not all round to one. replay.throttle stays the mean of the float32 "
                           "setpoints (0.031 logs as 0.0309999995: a replay hands it over exact, quantize False).",
        'pod modes': "replay.pod_modes and replay.pod_modes_at_release: per allocator pod (starboard, port; the "
                     "Cloudship's collective pair twice) {'mode', 'demand'}: the mode AirshipPod::steer is in at the "
                     "Acro switch and at the release, by its rule (steering above %g, holding between %g and %g once "
                     "engaged, released below or never engaged, parked while disarmed) run on the logged thrust and "
                     "torque setpoints up to that instant as ActuatorEffectivenessAirship::updateSetpoint splits "
                     "them, and the demand's magnitude there. Logged every 20 ms against the allocator's 4 ms: a "
                     "crossing shorter than that is not seen. None where a setpoint is unread or control surfaces "
                     "take a share." % (POD_ENGAGE, POD_RELEASE, POD_ENGAGE),
        'run_leg': "replay.run_leg: yaw_model/forward.run_leg's keyword arguments that replay the leg from its Acro "
                   "switch (frame and provenance in replay.run_leg_frame), whole or None: run_leg takes a None as "
                   "its default (an integrator of 0, pods from its warm-up), so a block missing any value is None "
                   "and replay.run_leg_unread names what was missing: unread, or past its guard (integ0: the "
                   "integrator not shown clear; throttle: the setpoints not on one count; extra_sticks: a second "
                   "stick began before the release's end, which a block of one stick would not fly). It is None "
                   "too for a leg a validity verdict failed, the leg's own or the run's ('verdict: ' and the "
                   "verdict's name, each), so a whole block is a leg a replay may fly as it is; and where the "
                   "replay's inputs met an error ('error: ' and the error), their fields None. The verdicts "
                   "stay in 'validity' for the replay to check as well.",
        't_acro_s': "the leg CSV's t_acro_s: s from the Acro switch, the replay block's clock and forward.py's "
                    "(t_s counts from the stick onset: t_acro_s = t_s + replay.stick[1][0]).",
        'IMU_GYRO_CUTOFF': "params.IMU_GYRO_CUTOFF, also in each replay.params: the gyro filter's cutoff the log "
                           "carries; a verdict holds it against the default in %s, which the model's IMU filter "
                           "takes (yaw_model/controller.py DEFAULT_PARAMS; planar.py's plants build their gyro "
                           "filter from it, so a replay cannot be handed another cutoff). A replay's precondition "
                           "among the validity verdicts: it gates the run's exit, so a run flown with another "
                           "cutoff exits 1 although its flight was valid, and none of its legs has a run_leg. A "
                           "yaml that does not read fails it." % ANALYSIS_PARAMS['IMU_GYRO_CUTOFF'],
    }


# run_leg's frame and where each of its values comes from, the replay block's
RUN_LEG_FRAME = ("FRD, as yaw_model/forward.run_leg takes its entry since 2026-10-01: u0 m/s forward and v0 m/s to "
                 "starboard (an older forward.py took v0 FLU, port positive: hand it -v0), r0 and p0 deg/s nose right "
                 "and right wing down, phi0 deg right wing down, all at the hull link origin (base_link), the state "
                 "at the Acro switch (replay.state). Times s from the Acro switch, manual_s 0: the record starts at "
                 "the switch. throttle the thrust setpoints' mean, exact (quantize False), given only when every one "
                 "rounds to one count (replay.throttle_counts). params the gains "
                 "(GAINS) as the shortest decimal of each logged float32. pods0 the allocator's tilts at the switch "
                 "(actuator_servos through CA_AIRSHIP_TLMIN/TLMAX) with the mode replay.pod_modes gives; motors0 "
                 "actuator_motors control[0..1], and the Cloudship's tail control[2]. integ0 0: the firmware's "
                 "reset when the loop opened in Manual, given only when the first sample after the switch read "
                 "within INTEG_CLEAR. gyro_settled True: the IMU filter at the entry's rate.")


def run_leg_kwargs(plant, replay, integ_first, modes, second_stick=False):
    """(kwargs, unread): yaw_model/forward.run_leg's keyword arguments that
    replay one leg from its Acro switch, from the replay block (RUN_LEG_FRAME
    says how), and the names of those it could not give: unread, or past a
    guard (integ0 when the integrator was not shown clear, throttle when the
    setpoints did not stay on one count, 'extra_sticks' when a second stick
    began before the release's end, which a block of one stick would not
    fly). run_leg takes a None as its default, which here would be a guess
    (an integrator of 0 never shown clear, pods from the warm-up), so kwargs
    is None when any is missing. analyze() refuses it further for a leg a
    verdict failed (refuse_run_leg)."""
    state, acts = replay['state'], replay['actuators']
    (_, _), (on_s, stick), (off_s, _) = replay['stick']
    tilts = ('tilt_stbd_deg', 'tilt_port_deg') if plant['independent'] else ('tilt_deg', 'tilt_deg')
    motors = ('motor1', 'motor2') + (('motor3',) if plant['tail'] else ())
    clear = integ_first is not None and abs(integ_first) <= INTEG_CLEAR
    logged = replay['params']
    kw = {'target': 'gz_' + plant['model'] if plant.get('model') else None,
          'u0': state.get('u'), 'v0': state.get('v'), 'r0': state.get('r_deg_s'), 'p0': state.get('p_deg_s'),
          'phi0': state.get('roll_deg'), 'stick': stick, 'quantize': False,
          # one throttle the leg flew, or none: the mean of one that moved was never flown
          'throttle': replay.get('throttle') if replay.get('throttle_counts') is not None else None,
          'params': {k: None if logged.get(k) is None else f32_decimal(logged[k]) for k in GAINS},
          'start': 'acro', 'manual_s': 0.0, 'centred_s': on_s, 'stick_s': off_s - on_s,
          'release_s': replay['end_s'] - off_s,
          'pods0': [{'tilt': acts.get(key), 'mode': None if modes is None else modes[i][0]}
                    for i, key in enumerate(tilts)],
          'motors0': [acts.get(key) for key in motors],
          'integ0': 0.0 if clear else None, 'gyro_settled': True}
    unread = []

    for key, value in kw.items():
        if isinstance(value, dict):
            unread += ['%s.%s' % (key, k) for k, v in value.items() if v is None]
        elif isinstance(value, list):
            for i, item in enumerate(value):
                parts = item.items() if isinstance(item, dict) else [(None, item)]
                unread += ['%s[%d]%s' % (key, i, '' if k is None else '.' + k) for k, v in parts if v is None]
        elif value is None:
            unread.append(key)

    if second_stick:
        unread.append('extra_sticks')

    return (None if unread else kw), unread


def refuse_run_leg(replay, reasons):
    """No run_leg for a leg a replay must not fly: None, with each reason
    added to run_leg_unread once."""
    replay['run_leg'] = None
    replay['run_leg_unread'] = list(replay.get('run_leg_unread') or []) + [
        r for r in reasons if r not in (replay.get('run_leg_unread') or [])]


def leg_metrics(log, leg, table, plant, params, target=None, release_s=RELEASE_S):
    """Every number of one leg (see the module docstring), and the replay
    block. `target` is the planned speed; without a plan the entry's own mean
    speed stands in, which leaves the steady judgement about drift alone.
    `release_s` is the release the legs were cut with (find_legs)."""
    us = 1e6
    on, off, acro = leg['on_us'], leg['off_us'], leg['acro_us']
    end = leg['release_end_us']
    L = params.get('AS_YR_INT_LIM')
    entry = rows_between(table, acro, on)
    pre = rows_between(table, acro - int(STEADY_WINDOW_S * us), acro)
    t_rel = table['t_s']
    entry_u = mean_or_none(table['u'][entry])
    target = target if target is not None else entry_u
    steady_pre = steady_judgement(t_rel[pre], table['u'][pre], table['v'][pre], table['r_deg_s'][pre], target)
    steady_entry = steady_judgement(t_rel[entry], table['u'][entry], table['v'][entry], table['r_deg_s'][entry], target)

    # the speed the constant throttle holds, against the file's drag: m u' =
    # T - c u^2 read backwards over the pre-entry window, so a trim that is
    # right but still settling is told from one that is wrong
    holds = None

    if plant['pod_thrust'] and pre.any():
        tt, uu = t_rel[pre], table['u'][pre]
        slope = float(np.polyfit(tt, uu, 1)[0]) if len(tt) > 2 and np.all(np.isfinite(uu)) else None
        thrust = mean_or_none(table['throttle'][pre])

        if slope is not None and thrust is not None:
            force = thrust * plant['pod_thrust']
            u2 = float(np.mean(uu * uu))
            # below half a metre per second the drag is a hundredth of a newton
            # and the quotient is the noise of the slope
            implied = (force - plant['surge_mass'] * slope) / u2 if u2 >= 0.25 and force > 0.0 else None
            holds = {'throttle': thrust, 'thrust_n': force, 'u_mean': float(np.mean(uu)), 'du_dt': slope,
                     'drag_file': plant['surge_drag'], 'drag_implied': implied,
                     'u_file': math.sqrt(force / plant['surge_drag']) if plant['surge_drag'] > 0.0 else None}

    stick_value = None
    _, yaws = samples(log, 'manual_control_setpoint', 'yaw', on + int(0.3 * us), off)

    if len(yaws):
        stick_value = float(np.median(yaws))

    _, sps = samples(log, 'vehicle_rates_setpoint', 'yaw', on, off)
    setpoint = float(np.degrees(np.median(sps))) if len(sps) else None
    sign = 1.0 if (setpoint or stick_value or 1.0) >= 0.0 else -1.0
    stick_rows = rows_between(table, on, off)
    release_rows = rows_between(table, off, end + 1)
    t_av, r_av = samples(log, 'vehicle_angular_velocity', 'xyz[2]', on, off)
    r_deg = np.degrees(r_av)
    peak = peak_t = None

    if len(r_deg):
        i = int(np.argmax(sign * r_deg))
        peak, peak_t = float(r_deg[i]), (int(t_av[i]) - on) / us

    smoothed = None
    rs = table['r_deg_s'][stick_rows]

    if len(rs) >= 25:
        window = np.convolve(rs, np.ones(25) / 25.0, mode='valid')     # 0.5 s of 50 Hz rows
        smoothed = float(window[int(np.argmax(sign * window))])

    # The same peak on a PEAK_WINDOW_S time window of the gyro samples
    # themselves, whatever the rows' spacing: each window (t - 0.5 s, t]
    # ends on a sample and lies wholly inside the stick, the mean of every
    # sample in it. An unread sample leaves it unread
    smoothed_05 = smoothed_05_t = None
    window_us = int(round(PEAK_WINDOW_S * us))
    ends = np.nonzero(t_av - window_us >= on)[0]

    if len(ends) and np.all(np.isfinite(r_deg)):
        starts = np.searchsorted(t_av, t_av[ends] - window_us, side='right')
        running = np.concatenate([[0.0], np.cumsum(r_deg)])
        means = (running[ends + 1] - running[starts]) / (ends + 1 - starts)
        j = int(np.argmax(sign * means))
        smoothed_05, smoothed_05_t = float(means[j]), (int(t_av[ends[j]]) - on) / us

    t_i, integ = samples(log, 'rate_ctrl_status', 'yawspeed_integ', on, off)
    at_limit = reached = None       # None: unread; reached False: read, and it never got there

    if L and len(integ):
        hit = np.nonzero(np.abs(integ) >= 0.99 * L)[0]
        reached = bool(len(hit))
        at_limit = (int(t_i[hit[0]]) - on) / us if len(hit) else None

    t_c, unalloc = samples(log, 'control_allocator_status', 'unallocated_torque[2]', on, off)
    _, achieved = samples(log, 'control_allocator_status', 'torque_setpoint_achieved', on, off)
    flagged = (achieved < 0.5) & (np.abs(unalloc) > FLAG_EPS)
    _, torque = samples(log, 'vehicle_torque_setpoint', 'xyz[2]', on, off)
    _, torque5 = samples(log, 'vehicle_torque_setpoint', 'xyz[2]', off - int(5.0 * us), off)
    unalloc_end, _ = last_at(log, 'control_allocator_status', 'unallocated_torque[2]', off)
    achieved_end, _ = last_at(log, 'control_allocator_status', 'torque_setpoint_achieved', off)
    shortfall_end = None if unalloc_end is None or achieved_end is None else (0.0 if achieved_end > 0.5 else unalloc_end)
    window_u = lambda a, b: mean_or_none(table['u'][rows_between(table, a, b)])

    def by_name(rows, fin):
        """The fin by its name on the rows where the plugin has flow
        (leg_table): the largest |alpha| / alpha_stall there (forward.py's
        fin_max_alpha_ratio; None where no row has flow), the rows without
        flow, and the branch on the phase's last row (forward.py's
        fins_branch). All None when a row is unread."""
        flow = table['fin_%s_flow' % fin['name']][rows]
        alpha = np.abs(table['fin_%s_alpha_deg' % fin['name']][rows])
        stall = math.degrees(fin['alpha_stall'])
        out = {'name': fin['name'], 'max_ratio': None, 'no_flow_rows': None, 'branch_end': None}
        with_flow = flow > 0.5

        if not len(flow) or not np.all(np.isfinite(flow)) or not np.all(np.isfinite(alpha[with_flow])):
            return out

        out['no_flow_rows'] = int(np.sum(~with_flow))
        out['max_ratio'] = float(np.max(alpha[with_flow])) / stall if with_flow.any() else None
        out['branch_end'] = 'no flow' if not with_flow[-1] else ('stalled' if alpha[-1] > stall else 'attached')
        return out

    def fins(rows):
        out = []

        for name, fin in zip(fin_names(plant), plant['fins']):
            values = np.abs(table[name][rows])
            worst = float(np.nanmax(values)) if len(values) and np.any(np.isfinite(values)) else None
            out.append({'max_abs_deg': worst, 'stall_deg': math.degrees(fin['alpha_stall']),
                        'stalled': None if worst is None else worst > math.degrees(fin['alpha_stall'])})
            out[-1].update(by_name(rows, fin))
        return out

    def reversals(a, b):
        t = table['t_px4_us']

        if plant['independent']:
            out, flips = {}, 0

            for name in tilt_names(plant):
                into, back, rear = pod_reversals(t, table[name], a, b)
                out[name.replace('tilt_', '').replace('_deg', '')] = {'into_rear': int(into), 'back_forward': int(back),
                                                                      'rear_s': round(float(rear), 3)}
                flips += into + back

            out['flips'] = int(flips)       # forward.py's rev_stick / rev_release
            return out

        tilt = table['tilt_deg'][rows_between(table, a, b)]
        out = {'tilt_min_deg': float(np.nanmin(tilt)) if len(tilt) else None,
               'tilt_max_deg': float(np.nanmax(tilt)) if len(tilt) else None}

        if plant['tail']:
            out['tail_flips'] = out['flips'] = sign_reversals(t, table['motor3'], a, b)
            # Beside it, the sign of the yaw torque setpoint (definitions():
            # the tail command is clamp of it), on its own 20 ms samples from
            # the table's first row, so its direction carries into the phase
            # from the same span
            tt, tz = samples(log, 'vehicle_torque_setpoint', 'xyz[2]', int(t[0]) if len(t) else a, b)
            out['tail_flips_torque'] = (int(sign_reversals(tt, tz, a, b)) if len(tz) and np.all(np.isfinite(tz))
                                        else None)

        return out

    def mark_time(seconds):
        """The PX4 time `seconds` after the stick came off, or None past the
        release. A mark past the release's end by no more than the duration
        verdict's own allowance is taken at that end, so a release that passes
        as long enough has its last mark too; marks_after_s records where."""
        when = off + int(round(seconds * us))

        if when <= end:
            return when

        return end if when - end <= DURATION_TOL_S * us else None

    marks = {'+%g' % s: mark_time(s) for s in (10.0, 60.0, release_s)}
    at_marks = lambda read: {k: None if w is None else read(w) for k, w in marks.items()}
    span = min(end, off + int(round(release_s * us)))
    heading_change = None
    h0, h1 = at_time(table, 'heading_deg', off), at_time(table, 'heading_deg', span)

    if h0 is not None and h1 is not None:
        heading_change = h1 - h0

    integ_first, integ_first_t = first_at(log, 'rate_ctrl_status', 'yawspeed_integ', acro)
    integ_off, integ_off_t = last_at(log, 'rate_ctrl_status', 'yawspeed_integ', off)
    state_at = lambda when: {k: at_time(table, c, when) for k, c in (
        ('u', 'u'), ('v', 'v'), ('w_down', 'w_down'), ('p_deg_s', 'p_deg_s'), ('r_deg_s', 'r_deg_s'),
        ('roll_deg', 'roll_deg'), ('pitch_deg', 'pitch_deg'), ('heading_deg', 'heading_deg'), ('alt_m', 'alt_m'),
        ('u_ekf', 'u_ekf'), ('v_ekf', 'v_ekf'), ('yawspeed_integ', 'yawspeed_integ'), ('throttle', 'throttle'),
        ('torque_z', 'torque_z'))}
    actuators_at = lambda when: {c: at_time(table, c, when) for c in table if c.startswith(('motor', 'servo', 'tilt'))}
    # the throttle the leg flew, from the steady window the entry is judged
    # on to the release's end: the logged thrust setpoint, every sample
    throttle_from = acro - int(STEADY_WINDOW_S * us)
    _, thrust = samples(log, 'vehicle_thrust_setpoint', 'xyz[0]', throttle_from, end + 1)
    thrust_read = len(thrust) > 0 and bool(np.all(np.isfinite(thrust)))

    out = {
        'times_px4_us': {k: leg[k] for k in ('manual_us', 'acro_us', 'on_us', 'off_us', 'acro_end_us', 'release_end_us')},
        'durations_s': {'entry': (on - acro) / us, 'stick': (off - on) / us, 'release': (end - off) / us,
                        'acro_after_stick_off': (leg['acro_end_us'] - off) / us},
        'stick_seen_off': leg['off_seen'], 'extra_sticks': leg['extra_sticks'],
        'stick': {'value': stick_value, 'setpoint_deg_s': setpoint,
                  'setpoint_from_params_deg_s': setpoint_deg(stick_value, params)
                  if stick_value is not None and params.get('MAN_DEADZONE') is not None
                  and params.get('AS_YAWRATE_MAX') is not None else None},
        'steady': {'before_entry': steady_pre[1], 'entry': steady_entry[1], 'ok': steady_pre[0] and steady_entry[0],
                   'throttle_holds': holds},
        'entry': {'at_acro': state_at(acro), 'at_stick': state_at(on), 'actuators_at_stick': actuators_at(on),
                  'u_mean': entry_u, 'v_mean': mean_or_none(table['v'][entry]), 'r_mean_deg_s': gyro_mean(log, acro, on),
                  'integrator_first': integ_first,
                  'integrator_first_after_s': None if integ_first_t is None else (integ_first_t - acro) / us,
                  'integrator_at_stick': last_at(log, 'rate_ctrl_status', 'yawspeed_integ', on)[0]},
        'stick_phase': {'r_mean_last5_deg_s': gyro_mean(log, off - int(5.0 * us), off),
                        'r_peak_deg_s': peak, 'r_peak_after_s': peak_t, 'r_peak_smoothed_deg_s': smoothed,
                        'integrator_end': last_at(log, 'rate_ctrl_status', 'yawspeed_integ', off)[0],
                        'integrator_max_abs': float(np.max(np.abs(integ))) if len(integ) else None,
                        'integrator_limit': L, 'integrator_reached_limit': reached,
                        'integrator_at_limit_after_s': at_limit,
                        'unallocated_end': unalloc_end, 'achieved_end': achieved_end,
                        'flag_pos_end': None if shortfall_end is None else shortfall_end > FLAG_EPS,
                        'flag_neg_end': None if shortfall_end is None else shortfall_end < -FLAG_EPS,
                        'flagged_fraction': float(np.mean(flagged)) if len(flagged) else None,
                        'allocator_samples': int(len(t_c)),
                        'torque_mean': float(np.mean(torque)) if len(torque) else None,
                        'torque_mean_last5': float(np.mean(torque5)) if len(torque5) else None,
                        'u_start': window_u(on, on + int(0.5 * us)), 'u_end': window_u(off - int(0.5 * us), off),
                        'heading_change_deg': None if at_time(table, 'heading_deg', on) is None
                        or at_time(table, 'heading_deg', off) is None
                        else at_time(table, 'heading_deg', off) - at_time(table, 'heading_deg', on),
                        'r_minus_heading_rate_deg_s': mean_or_none(table['r_deg_s'][stick_rows]
                                                                   - table['r_hdg_deg_s'][stick_rows]),
                        'fins': fins(stick_rows), 'reversals': reversals(on, off)},
        'release': {'window_s': (span - off) / us, 'heading_change_deg': heading_change,
                    # the segment's entry state, as the leg's own: the release
                    # is where the defect is measured, and two runs' releases
                    # are compared from here
                    'entry': {'state': state_at(off), 'actuators': actuators_at(off), 'integrator': integ_off,
                              'integrator_age_s': None if integ_off_t is None else (off - integ_off_t) / us},
                    'marks_after_s': at_marks(lambda w: (w - off) / us),
                    'r_deg_s': at_marks(lambda w: gyro_at(log, w)),
                    'r_mean1s_deg_s': at_marks(lambda w: gyro_mean(log, w - int(1.0 * us), w)),
                    'integrator': at_marks(lambda w: last_at(log, 'rate_ctrl_status', 'yawspeed_integ', w)[0]),
                    'u_end': window_u(end - int(0.5 * us), end + 1), 'fins': fins(release_rows),
                    'reversals': reversals(off, end + 1)},
        'throttle': {'from_px4_us': throttle_from, 'to_px4_us': end, 'n': int(len(thrust)),
                     'min': float(np.min(thrust)) if thrust_read else None,
                     'max': float(np.max(thrust)) if thrust_read else None},
    }
    out['replay'] = {
        'frame': 'FRD body, NED heading. u, v: model origin (base_link) velocity from '
                 'vehicle_local_position_groundtruth in the heading frame; p, r: vehicle_angular_velocity; '
                 'attitude: vehicle_attitude_groundtruth; times relative to t0_px4_us (the Acro entry)',
        't0_px4_us': acro, 'state': out['entry']['at_acro'], 'state_at_stick': out['entry']['at_stick'],
        'state_at_release': out['release']['entry']['state'],
        'actuators': {c: at_time(table, c, acro) for c in table if c.startswith(('motor', 'servo', 'tilt'))},
        'actuators_at_release': out['release']['entry']['actuators'],
        'integrator': integ_first,          # None when unread: a replay must not start from a guessed 0
        'integrator_at_release': integ_off,
        'throttle': mean_or_none(table['throttle'][rows_between(table, acro, end + 1)]),
        'stick': [[0.0, 0.0], [(on - acro) / us, stick_value], [(off - acro) / us, 0.0]],
        'end_s': (end - acro) / us,
        'params': {k: params.get(k) for k in PARAMS},
    }
    # Added on 2026-10-01 (definitions() says what each is), after every
    # earlier field, which keeps its value and meaning
    out['stick_phase'].update(r_peak_smoothed_05s_deg_s=smoothed_05, r_peak_smoothed_05s_after_s=smoothed_05_t)
    replay = out['replay']
    replay['params'].update({k: params.get(k) for k in ANALYSIS_PARAMS})
    replay.update(integrator_first_after_s=out['entry']['integrator_first_after_s'], throttle_counts=None,
                  pod_modes=None, pod_modes_at_release=None, run_leg=None, run_leg_unread=[],
                  run_leg_frame=RUN_LEG_FRAME)

    # The replay's inputs. An error in them must not cost the leg's record,
    # which after a flight is written by this same call: their fields stay
    # None and run_leg_unread names the error
    try:
        # the throttle's MANUAL_CONTROL count, from every sample the
        # replay's mean is of: one count, or None
        _, thrust_replay = samples(log, 'vehicle_thrust_setpoint', 'xyz[0]', acro, end + 1)
        counts = None

        if len(thrust_replay) and np.all(np.isfinite(thrust_replay)):
            each = np.unique(np.round(thrust_replay * 1000.0).astype(int))
            counts = int(each[0]) if len(each) == 1 else None

        modes_acro, modes_off = pod_modes(log, plant, params, acro), pod_modes(log, plant, params, off)
        as_dicts = lambda modes: None if modes is None else [{'mode': m, 'demand': d} for m, d in modes]
        replay.update(throttle_counts=counts, pod_modes=as_dicts(modes_acro), pod_modes_at_release=as_dicts(modes_off))
        # a second stick inside the span the block replays (the leg's own
        # first): extra_sticks counts the whole Acro interval
        second = len(stick_segments(log, acro, end + 1)) > 1
        kwargs, unread = run_leg_kwargs(plant, replay, integ_first, modes_acro, second_stick=second)
        replay.update(run_leg=kwargs, run_leg_unread=unread)
    except Exception as error:
        refuse_run_leg(replay, ['error: %s: %s' % (type(error).__name__, error)])

    return out


def timeline_check(leg, events, tol_s=TIMELINE_TOL_S):
    """(ok, detail): each logged time inside the live bracket around its
    command, widened by `tol_s`. `events` maps 'acro', 'stick_on', 'stick_off'
    and 'release_end' to (px4_before_us, px4_after_us); a bracket with an
    unread end is a failure, not a pass."""
    tol = int(tol_s * 1e6)
    pairs = (('acro', leg['acro_us']), ('stick_on', leg['on_us']), ('stick_off', leg['off_us']),
             ('release_end', leg['acro_end_us']))
    ok, parts = True, []

    for name, logged in pairs:
        bracket = events.get(name)

        if not bracket:
            ok = False
            parts.append('%s 명령이 없었다 (중단)' % name)
            continue

        if bracket[0] is None or bracket[1] is None:
            ok = False
            parts.append('%s 의 PX4 시각을 읽지 못했다' % name)
            continue

        before, after = bracket
        inside = before - tol <= logged <= after + tol
        ok = ok and inside
        parts.append('%s %+.3f s%s' % (name, (logged - after) / 1e6, '' if inside else ' (밖)'))

    return ok, ', '.join(parts)


def topic_rates(log, windows, closed_windows):
    """{topic: {'median_ms', 'max_gap_ms', 'n', 'needed_ms'}} inside the given
    (a, b) windows: the closed-loop part for the topics published only there."""
    out = {}

    for topic, needed in NEEDED_MS.items():
        if not log.has(topic):
            out[topic] = {'median_ms': None, 'max_gap_ms': None, 'n': 0, 'needed_ms': needed}
            continue

        t = log.t(topic)
        gaps = []
        n = 0

        for a, b in (closed_windows if topic in CLOSED_LOOP_ONLY else windows):
            inside = t[(t >= a) & (t < b)]
            n += len(inside)

            if len(inside) > 1:
                gaps.append(np.diff(inside))

        gaps = np.concatenate(gaps) / 1000.0 if gaps else np.array([])
        out[topic] = {'median_ms': float(np.median(gaps)) if len(gaps) else None,
                      'max_gap_ms': float(np.max(gaps)) if len(gaps) else None, 'n': int(n), 'needed_ms': needed}

    return out


def rates_ok(rates):
    bad = [t for t, r in rates.items()
           if r['median_ms'] is None or not (r['median_ms'] <= 1.1 * r['needed_ms'])
           or not (r['max_gap_ms'] <= GAP_FACTOR * r['needed_ms'])]
    return not bad, bad


def write_csv(path, table):
    columns = list(table)

    with open(path, 'w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(columns)

        for i in range(len(table['t_s'])):
            row = []

            for c in columns:
                x = table[c][i]

                if isinstance(x, str):
                    row.append(x)
                elif c == 't_px4_us':
                    row.append('%d' % x)
                elif not math.isfinite(float(x)):
                    row.append('')
                else:
                    row.append('%.6g' % float(x))

            writer.writerow(row)


def model_of(log):
    """The plant a log was flown on, by its SYS_AUTOSTART, or None."""
    autostart = log.param('SYS_AUTOSTART')

    for name in ALLOWED:
        if autostart is not None and int(autostart) == gz_model.airframe(name)[0]:
            return name

    return None


def analyze(log, model, live=None, out_dir=OUT, stamp='analysis', release_s=RELEASE_S, stick_s=STICK_S):
    """Segment and measure one log. Returns (summary dict, [(name, ok, detail)]).

    live: what this script recorded while flying it, {'legs': [{'leg', 'plan',
    'flown', 'events': {name: (px4_before_us, px4_after_us)}}...]}, or None for
    a log it did not fly (a gate's, say). Without it the steady judgement is
    about drift alone and the timeline is not judged."""
    params = {k: log.param(k) for k in PARAMS + tuple(ANALYSIS_PARAMS)}
    sched = {k: log.param(k) for k in SCHEDULE_PARAMS}
    plant = plant_info(model, params)
    legs = find_legs(log, release_s)
    checks = []
    sched_checks = []       # the schedule's, outside the validity verdicts: they refuse no replay
    results = []
    own_checks = []         # (metrics, the verdicts of that leg): the legs a replay must not fly
    live_legs = [l for l in (live or {}).get('legs', []) if l.get('flown')]
    os.makedirs(out_dir, exist_ok=True)
    windows, closed = [], []

    for leg in legs:
        windows.append((leg['acro_us'] - int(STEADY_WINDOW_S * 1e6), leg['release_end_us']))
        closed.append((leg['acro_us'], leg['acro_end_us']))

    # Match each flown leg to the logged one whose onset falls in its bracket.
    # A bracket with an unread end falls back to the order of flight, and its
    # timeline check then fails on the unread time, which is the finding.
    matched = []

    if live is not None:
        for order, flown in enumerate(live_legs):
            bracket = flown['events'].get('stick_on') or (None, None)
            best = None

            if bracket[0] is None or bracket[1] is None:
                best = legs[order] if order < len(legs) else None
            else:
                for leg in legs:
                    if bracket[0] - TIMELINE_TOL_S * 1e6 <= leg['on_us'] <= bracket[1] + TIMELINE_TOL_S * 1e6:
                        best = leg
                        break

            matched.append((flown, best))
    else:
        matched = [(None, leg) for leg in legs]

    for position, (flown, leg) in enumerate(matched, 1):
        # the leg's own number, which its plan, its rerun and the run's list
        # of unflown legs all use; the order in the log only for a log this
        # script did not fly
        number = flown['leg'] if flown else position
        name = '다리 %d' % number
        plan = flown['plan'] if flown else None

        if leg is None:
            checks.append(('%s: 로그에 이 다리가 있다' % name, False,
                           '스크립트가 스틱을 건 PX4 시각 %s 부근에 로그의 스틱이 없다'
                           % (flown['events'].get('stick_on'),)))
            continue

        table = leg_table(log, leg, plant)
        metrics = leg_metrics(log, leg, table, plant, params, target=plan['speed'] if plan else None,
                              release_s=release_s)
        path = os.path.join(out_dir, 'forward_%s_%s_leg%d.csv' % (model or 'unknown', stamp, number))
        write_csv(path, table)
        metrics.update(leg=number, plan=plan, csv=os.path.relpath(path, HERE) if path.startswith(HERE) else path)
        lost = ((flown or {}).get('events') or {}).get('source_lost')
        lost_us = lost[1] if lost and lost[1] is not None else None
        metrics['schedule'] = schedule_metrics(log, leg, params, sched, metrics['stick']['value'], lost_us)
        metrics['replay']['params'].update(sched)

        if metrics['schedule']['on']:
            sched_checks.extend(schedule_checks(name, metrics, lost))

        d = metrics['durations_s']
        first_check = len(checks)

        if flown:
            ok, detail = timeline_check(leg, flown['events'])
            checks.append(('%s: 로그의 모드와 스틱 시각이 스크립트가 잰 PX4 시각과 맞는다' % name, ok, detail))
            value = metrics['stick']['value']
            ok = value is not None and abs(value - plan['stick']) < 0.005
            checks.append(('%s: 로그의 스틱이 계획의 스틱이다' % name, ok,
                           '로그 %s, 계획 %+.3f' % ('?' if value is None else '%+.3f' % value, plan['stick'])))

        # every thrust setpoint from the steady window to the release's end on
        # the plan's count; without a plan, on one value
        th = metrics['throttle']
        want = plan['throttle'] if plan else None

        if th['min'] is None:
            ok = False
        elif want is not None:
            ok = abs(th['min'] - want) <= THROTTLE_TOL and abs(th['max'] - want) <= THROTTLE_TOL
        else:
            ok = th['max'] - th['min'] <= THROTTLE_TOL

        checks.append(('%s: 스로틀이 진입 전 창부터 놓음 끝까지 %s' % (
                           name, '계획의 %d/1000 에 머물렀다' % plan['throttle_counts'] if plan
                           else '한 값에 머물렀다 (계획 없음)'), ok,
                       '추력 설정 %s..%s, %d 표본 (%s, 허용 ±%g)' % (
                           '?' if th['min'] is None else '%.5f' % th['min'],
                           '?' if th['max'] is None else '%.5f' % th['max'], th['n'],
                           '계획 %.3f' % want if want is not None else '계획 없음', THROTTLE_TOL)))

        first = metrics['entry']['integrator_first']
        ok = first is not None and abs(first) <= INTEG_CLEAR
        checks.append(('%s: Manual 에서 Acro 로 들어가 적분기가 비어서 시작했다' % name, ok,
                       'Manual %.2f s 뒤 Acro, 첫 적분기 %s (%s s 뒤; 허용 %g)'
                       % ((leg['acro_us'] - leg['manual_us']) / 1e6, '?' if first is None else '%+.5f' % first,
                          '?' if metrics['entry']['integrator_first_after_s'] is None
                          else '%.3f' % metrics['entry']['integrator_first_after_s'], INTEG_CLEAR)))
        s = metrics['steady']
        target = plan['speed'] if plan else None
        checks.append(('%s: 진입이 %s 정상 직진이다' % (name, '%.1f m/s 의' % target if target else '(계획 없음, 속도는 판정 안 함)'),
                       s['ok'], steady_text(s, target)))
        ok = (d['stick'] >= stick_s - DURATION_TOL_S and leg['off_seen']
              and d['acro_after_stick_off'] >= release_s - DURATION_TOL_S)
        checks.append(('%s: 스틱 %g s 와 놓음 %g s 가 시뮬레이터 시계로 끝까지 갔다' % (name, stick_s, release_s), ok,
                       '진입 %.2f s, 스틱 %.2f s%s, 놓은 뒤 Acro %.2f s' % (d['entry'], d['stick'],
                                                                      '' if leg['off_seen'] else ' (놓지 않았다)',
                                                                      d['acro_after_stick_off'])))
        span = rows_between(table, leg['acro_us'] - int(STEADY_WINDOW_S * 1e6), leg['release_end_us'] + 1)
        alt = table['alt_m'][span]
        low = float(np.min(alt)) if len(alt) and np.all(np.isfinite(alt)) else None
        armed_all = all(not (leg['acro_us'] - STEADY_WINDOW_S * 1e6 <= t <= leg['release_end_us']) or a
                        for t, _, a in mode_changes(log))

        if GT_POS == ESTIMATOR_FRAME[0]:
            # the estimator's z is relative to where it started, not to the
            # ground: the floor is the range finder's height when it has one,
            # and otherwise the verdict is about the arming alone
            a_us, b_us = leg['acro_us'] - int(STEADY_WINDOW_S * 1e6), leg['release_end_us'] + 1
            _, dist = samples(log, 'vehicle_local_position', 'dist_bottom', a_us, b_us)
            _, valid = samples(log, 'vehicle_local_position', 'dist_bottom_valid', a_us, b_us)
            dist = dist[valid > 0.5] if len(valid) == len(dist) else dist[:0]
            low = float(np.min(dist)) if len(dist) and np.all(np.isfinite(dist)) else None
            ok = armed_all and (low is None or low >= MIN_HEIGHT_M)
            checks.append(('%s: 다리 내내 무장 상태였다 (거리계가 있으면 바닥 위에)' % name, ok,
                           '거리계 최저 %s m (바닥 %g m), %s' % ('없음' if low is None else '%.1f' % low, MIN_HEIGHT_M,
                                                        '무장 유지' if armed_all else '중간에 무장이 풀렸다')))
        else:
            ok = low is not None and low >= MIN_HEIGHT_M and armed_all
            checks.append(('%s: 다리 내내 무장 상태로 떠 있었다' % name, ok,
                           '최저 고도 %s m (바닥 %g m), %s' % ('?' if low is None else '%.1f' % low, MIN_HEIGHT_M,
                                                        '무장 유지' if armed_all else '중간에 무장이 풀렸다')))
        results.append(metrics)
        own_checks.append((metrics, checks[first_check:]))

    first_run_check = len(checks)
    rates = topic_rates(log, windows, closed)
    ok, bad = rates_ok(rates)
    drops = [(t, d) for t, d in log.dropouts if any(a <= t <= b for a, b in windows)]
    checks.append(('필요한 토픽이 필요한 속도로 있고 로그가 끊기지 않았다', ok and not drops and bool(legs),
                   '%s; 다리 안의 로거 끊김 %d 번%s' % (
                       ', '.join('%s %s/%g ms (최대 간격 %s)' % (t.replace('vehicle_', 'v_'),
                                                             '?' if r['median_ms'] is None else '%.0f' % r['median_ms'],
                                                             r['needed_ms'],
                                                             '?' if r['max_gap_ms'] is None else '%.0f' % r['max_gap_ms'])
                                 for t, r in rates.items()), len(drops),
                       '' if not bad else '; 모자란 토픽: ' + ', '.join(bad))))

    # The model takes these from their yaml (the gyro filter's cutoff): the
    # log must carry that value, or a replay filters a different gyro. A
    # replay's precondition among the validity verdicts, so it gates the
    # exit (the module docstring), after every earlier verdict, which keep
    # their places. A yaml that does not read, for whatever reason, fails it
    for name, rel in ANALYSIS_PARAMS.items():
        logged = params.get(name)

        try:
            want, why = yaml_default(name, rel), None
        except Exception as error:
            want, why = None, '%s: %s' % (type(error).__name__, error)

        ok = logged is not None and want is not None and abs(logged - want) <= 1e-6 * max(1.0, abs(want))
        checks.append(('로그의 %s 가 모델이 쓰는 yaml 기본값이다' % name, ok,
                       '로그 %s, %s %s' % ('없음' if logged is None else '%g' % logged, rel,
                                          why if want is None else '%g' % want)))

    # No run_leg for a leg a verdict failed, its own or the run's: a whole
    # block is a leg a replay may fly as it is
    for metrics, own in own_checks:
        failed = [n for n, ok, _ in own + checks[first_run_check:] if not ok]

        if failed:
            refuse_run_leg(metrics['replay'], ['verdict: %s' % n for n in failed])

    summary_doc = {'check': 'forward_yaw_check', 'model': model, 'stamp': stamp, 'log': log.path,
                   'firmware': log.info.get('ver_sw'), 'params': params,
                   'param_changes': [(int(t), n, float(v)) for t, n, v in log.changed],
                   'plant': {k: plant[k] for k in plant if k != 'fins'} | {'vertical_fins': plant['fins']},
                   'rates': rates, 'release_s': release_s,
                   'resolution_ms': {k: rates[topic]['median_ms'] for k, topic in RESOLUTION.items()},
                   'definitions': definitions(), 'legs': results, 'legs_in_log': len(legs),
                   'validity': [{'name': n, 'ok': bool(o), 'detail': d} for n, o, d in checks],
                   'schedule_params': sched,
                   'schedule_verdicts': [{'name': n, 'ok': bool(o), 'detail': d} for n, o, d in sched_checks]}
    return summary_doc, checks + sched_checks


def steady_text(s, target):
    pre, entry, holds = s['before_entry'], s['entry'], s['throttle_holds']
    fmt = lambda x, f: '?' if x is None else f % x
    text = ('진입 전 %.1f s: u %s m/s (변화 %s), |r| 최대 %s deg/s, |v| 최대 %s m/s; Acro 진입 %.1f s: u %s, |r| %s, |v| %s'
            % (pre['span_s'], fmt(pre['u_mean'], '%.3f'), fmt(pre['u_drift'], '%+.3f'), fmt(pre['r_max_abs'], '%.2f'),
               fmt(pre['v_max_abs'], '%.3f'), entry['span_s'], fmt(entry['u_mean'], '%.3f'),
               fmt(entry['r_max_abs'], '%.2f'), fmt(entry['v_max_abs'], '%.3f')))

    if target:
        text += ' (허용 u ±%.3f, 변화 ±%.3f, |r| < %g, |v| < %g)' % (
            max(SPEED_TOL * target, SPEED_TOL_ABS), max(DRIFT_TOL * target, DRIFT_TOL_ABS), RATE_TOL_DEG, SWAY_TOL)

    if holds:
        text += '; 스로틀 %.4f = %.3f N, du/dt %+.4f m/s^2' % (holds['throttle'], holds['thrust_n'], holds['du_dt'])

        if holds['u_file'] is not None:
            text += ', 파일의 항력 %.4f 이면 %.3f m/s, 측정이 말하는 항력 %s' % (
                holds['drag_file'], holds['u_file'], fmt(holds['drag_implied'], '%.4f'))

    return text


def print_leg(m):
    """The numbers of one leg, as notes: none of these is a verdict."""
    fmt = lambda x, f='%.2f': '?' if x is None else f % x
    plan = m['plan'] or {}
    e, s, r = m['entry'], m['stick_phase'], m['release']
    a = e['at_stick']
    note('\n=== 다리 %d: %s m/s, 스틱 %s (지시 %s deg/s), 스로틀 %s ===' % (
        m['leg'], fmt(plan.get('speed'), '%.2f'), fmt(m['stick']['value'], '%+.2f'),
        fmt(m['stick']['setpoint_deg_s']), fmt(m['replay']['throttle'], '%.4f')))
    acts = ', '.join('%s %s' % (k, fmt(v, '%+.1f' if 'tilt' in k else '%.3f')) for k, v in sorted(e['actuators_at_stick'].items())
                     if not k.startswith('servo'))
    note('  진입 (Acro %.1f s): u %s v %s m/s (EKF u %s), r %s deg/s, 롤 %s 피치 %s deg, 헤딩 %s deg, 고도 %s m, '
         '적분기 Acro 첫 표본 %s / 스틱 직전 %s; %s'
         % (m['durations_s']['entry'], fmt(a['u'], '%.3f'), fmt(a['v'], '%+.3f'), fmt(a['u_ekf'], '%.3f'),
            fmt(e['r_mean_deg_s'], '%+.3f'), fmt(a['roll_deg'], '%+.2f'), fmt(a['pitch_deg'], '%+.2f'),
            fmt(a['heading_deg'], '%.1f'), fmt(a['alt_m'], '%.1f'), fmt(e['integrator_first'], '%+.5f'),
            fmt(e['integrator_at_stick'], '%+.5f'), acts))
    rp = m['replay']
    note('  재현 입력: 포드 모드 Acro %s / 놓을 때 %s, 스로틀 계수 %s, run_leg %s'
         % (pod_text(rp.get('pod_modes')), pod_text(rp.get('pod_modes_at_release')),
            fmt(rp.get('throttle_counts'), '%d'),
            '있음' if rp.get('run_leg') else '없음 (까닭: %s)' % ', '.join(rp.get('run_leg_unread') or ['?'])))
    flags = '+' if s['flag_pos_end'] else ('-' if s['flag_neg_end'] else '없음')
    fins = ', '.join('%s %s/%s deg%s (흐름 있는 곳 최대 비 %s)' % (
        f.get('name', '?'), fmt(f['max_abs_deg'], '%.1f'), fmt(f['stall_deg'], '%.1f'), ' 실속' if f['stalled'] else '',
        fmt(f.get('max_ratio'), '%.2f')) for f in s['fins'])
    limit = {None: '?', False: '없음'}.get(s['integrator_reached_limit'], fmt(s['integrator_at_limit_after_s'], '%.1f s'))
    note('  스틱 %.1f s: 마지막 5 s 평균 r %s deg/s (지시 %s), 최대 %s (스틱 뒤 %s s; 0.5 s 평균 최대 %s, '
         '0.5 s 시간 창 %s 스틱 뒤 %s s), '
         '헤딩 %s deg; 적분기 끝 %s (한계 %s, 한계 도달 %s), 미할당 요 %s, 토크 달성 %s, 깃발 %s '
         '(스틱 중 깃발 %s %%), 토크 평균 %s (마지막 5 s %s), u %s -> %s m/s; 수직 핀 받음각 최대 %s'
         % (m['durations_s']['stick'], fmt(s['r_mean_last5_deg_s'], '%+.3f'), fmt(m['stick']['setpoint_deg_s']),
            fmt(s['r_peak_deg_s'], '%+.2f'), fmt(s['r_peak_after_s'], '%.1f'), fmt(s['r_peak_smoothed_deg_s'], '%+.2f'),
            fmt(s.get('r_peak_smoothed_05s_deg_s'), '%+.3f'), fmt(s.get('r_peak_smoothed_05s_after_s'), '%.1f'),
            fmt(s['heading_change_deg'], '%+.1f'), fmt(s['integrator_end'], '%+.4f'), fmt(s['integrator_limit'], '%.2f'),
            limit, fmt(s['unallocated_end'], '%+.0f'), fmt(s['achieved_end'], '%.0f'), flags,
            fmt(None if s['flagged_fraction'] is None else 100.0 * s['flagged_fraction'], '%.0f'),
            fmt(s['torque_mean'], '%+.3f'), fmt(s['torque_mean_last5'], '%+.3f'), fmt(s['u_start']), fmt(s['u_end']),
            fins))
    b = r['entry']['state']
    note('  놓을 때: u %s v %s m/s, r %s deg/s, 롤 %s deg, 헤딩 %s deg, 적분기 %s (%s s 전 표본), 토크 %s; %s'
         % (fmt(b['u'], '%.3f'), fmt(b['v'], '%+.3f'), fmt(b['r_deg_s'], '%+.3f'), fmt(b['roll_deg'], '%+.2f'),
            fmt(b['heading_deg'], '%.1f'), fmt(r['entry']['integrator'], '%+.4f'),
            fmt(r['entry']['integrator_age_s'], '%.2f'), fmt(b['torque_z'], '%+.4f'),
            ', '.join('%s %s' % (k, fmt(v, '%+.1f' if 'tilt' in k else '%.3f'))
                      for k, v in sorted(r['entry']['actuators'].items()) if not k.startswith('servo'))))
    marks = ', '.join('%s s (%s) %s / %s' % (k, fmt(r['marks_after_s'][k], '%.2f'), fmt(r['r_deg_s'][k], '%+.3f'),
                                             fmt(r['integrator'][k], '%+.4f')) for k in r['r_deg_s'])
    note('  놓음 %.1f s: 헤딩 변화 %s deg; 표시 (실제 시각) r 한 표본 deg/s / 적분기: %s; r 1 s 평균 %s; u 끝 %s m/s'
         % (r['window_s'], fmt(r['heading_change_deg'], '%+.1f'), marks,
            ', '.join('%s %s' % (k, fmt(v, '%+.3f')) for k, v in r['r_mean1s_deg_s'].items()), fmt(r['u_end'])))
    note('  반전: 스틱 중 %s; 놓음 중 %s' % (reversal_text(s['reversals']), reversal_text(r['reversals'])))
    sc = m.get('schedule') or {}

    if sc.get('logged'):
        at = lambda k: sc.get(k) or {}
        part = lambda k: 'u %s..%s m/s, ff %s, 상한 %s deg/s, 원천 %s%s' % (
            fmt(at(k).get('u_lo')), fmt(at(k).get('u_hi')), fmt(at(k).get('ff'), '%.3f'), fmt(at(k).get('cap_deg_s')),
            at(k).get('source'), ' (u_hi 유지)' if at(k).get('upper_held') else '')
        note('  속도 스케줄 (%s): 스틱 %s; 놓을 때 %s; 끝 %s; 스틱 중 상한 %s..%s, 원천 %s; 마지막 5 s 설정 %s, 상한의 몫 %s deg/s%s'
             % ('켬' if sc['on'] else '끔', part('at_stick'), part('at_release'), part('at_end'),
                fmt(sc.get('cap_stick_min_deg_s')), fmt(sc.get('cap_stick_max_deg_s')), ', '.join(sc.get('sources_seen', [])),
                fmt(sc.get('setpoint_last5_deg_s'), '%.3f'), fmt(sc.get('setpoint_from_cap_last5_deg_s'), '%.3f'),
                '' if not sc.get('loss') else '; 스틱 %.1f s 에 속도원 잃음' % sc['loss']['after_stick_on_s']))

    note('  기록 %s' % m['csv'])


def reversal_text(rev):
    """One phase's reversals in words: the flips forward.py counts, then per
    pod the swings into the rear hemisphere and back and the time spent
    there; or the tail's direction changes and the collective tilt's range."""
    if 'tail_flips' in rev or 'tilt_min_deg' in rev:
        parts = []

        if 'tail_flips' in rev:
            parts.append('꼬리 방향 바뀜 %d 번 (토크 설정으로는 %s 번)' % (
                rev['tail_flips'], '?' if rev.get('tail_flips_torque') is None else rev['tail_flips_torque']))

        if rev.get('tilt_min_deg') is not None:
            parts.append('집합 틸트 %+.1f..%+.1f deg' % (rev['tilt_min_deg'], rev['tilt_max_deg']))

        return ', '.join(parts) or '?'

    names = {'stbd': '우현', 'port': '좌현'}
    return '%d 번 (%s)' % (rev['flips'], ', '.join(
        '%s 뒤로 %d 앞으로 %d, 뒤쪽 %.1f s' % (names.get(k, k), v['into_rear'], v['back_forward'], v['rear_s'])
        for k, v in rev.items() if k != 'flips'))


def pod_text(modes):
    """The pods' modes, each with its demand's magnitude, in words."""
    if modes is None:
        return '?'

    return ', '.join('%s (요구 %s)' % (p['mode'] or '?', '?' if p['demand'] is None else '%.4f' % p['demand'])
                     for p in modes)


def stamp_of(path):
    """'YYYYMMDD_HHMMSS' from a PX4 log path (<date>/<HH_MM_SS>.ulg) or a kept
    copy (forward_<model>_<stamp>.ulg), else from its mtime."""
    found = re.search(r'(\d{4})-(\d{2})-(\d{2})/(\d{2})_(\d{2})_(\d{2})\.ulg$', path)

    if found:
        return '%s%s%s_%s%s%s' % found.groups()

    found = re.search(r'_(\d{8}_\d{6})\.ulg$', path)

    if found:
        return found.group(1)

    return time.strftime('%Y%m%d_%H%M%S', time.gmtime(os.path.getmtime(path)))


def report_analysis(summary_doc, checks, out_dir, stamp, model):
    for m in summary_doc['legs']:
        print_leg(m)

    note('\n시간 분해능 (다리 안의 로그 간격 중앙값, ms): %s' % ', '.join(
        '%s %s' % (k, '?' if v is None else '%.0f' % v) for k, v in summary_doc['resolution_ms'].items()))
    note('')

    for name, ok, detail in checks:
        verdict(name, ok, detail)

    path = os.path.join(out_dir, 'forward_%s_%s_summary.json' % (model or 'unknown', stamp))

    with open(path, 'w') as handle:
        json.dump(summary_doc, handle, indent=1, ensure_ascii=False, default=float)

    note('요약 %s' % path)


def main_analyze(path, events_path=None, out_dir=None, release_s=RELEASE_S, model=None):
    """Redo the records of a kept log into `out_dir` (sitl_out by default).
    Never into the PX4 tree, where a gate's log may sit: this tool writes
    nothing there."""
    if not os.path.exists(path):
        print('로그가 없다: %s' % path)
        return 2

    out_dir = os.path.abspath(out_dir or OUT)

    if out_dir.startswith(os.path.abspath(gz_model.ROOT) + os.sep):
        print('%s 는 PX4 트리 안이다: 기록은 그 밖에 쓴다 (--out)' % out_dir)
        return 2

    live = None

    if events_path:
        with open(events_path) as handle:
            live = json.load(handle)

    log = Log.read(path)
    model = model or (live or {}).get('model') or model_of(log)
    stamp = stamp_of(path)
    note('기록 다시 읽기: %s (기체 %s, SYS_AUTOSTART %s, 펌웨어 %s)%s'
         % (path, model, log.param('SYS_AUTOSTART'), log.info.get('ver_sw'),
            '' if live else '; 사건 기록이 없어 시각 대조와 속도는 판정하지 않는다'))
    summary_doc, checks = analyze(log, model, live=live, out_dir=out_dir, stamp=stamp, release_s=release_s)

    if live is not None:
        summary_doc['live'] = live_block(live, events_path, out_dir, model, stamp, path)

    report_analysis(summary_doc, checks, out_dir, stamp, model)
    return summary()


def live_block(live, events_path, out_dir, model, stamp, log_path):
    """The 'live' block main_live writes beside the analysis, for a summary
    redone from that run's events file: the file, its abort and its plan from
    the file itself. Whether the run failed to write the file, only the run
    knew: that comes from the summary this one replaces when that is the
    same log's, else None (unknown)."""
    failures = None
    previous = os.path.join(out_dir, 'forward_%s_%s_summary.json' % (model or 'unknown', stamp))

    try:
        with open(previous) as handle:
            old = json.load(handle)

        if old.get('stamp') == stamp and os.path.realpath(str(old.get('log'))) == os.path.realpath(log_path):
            failures = (old.get('live') or {}).get('events_write_failures')
    except (OSError, ValueError, AttributeError):
        pass

    events = os.path.abspath(events_path)
    return {'events': os.path.relpath(events, HERE) if events.startswith(HERE + os.sep) else events,
            'aborted': live.get('aborted'), 'plan': live.get('plan'), 'events_write_failures': failures}


# ---------------------------------------------------------------------------
# The flight. Everything below talks to a running px4.
# ---------------------------------------------------------------------------

class Abort(Exception):
    """A state in which no later leg can mean anything: the hull near the
    ground, disarmed, px4 not answering, a mode that would not change, or the
    simulator's clock stalled against the wall."""


class Live:
    """The run's live side: reads through sitl_link, the sticks, and the
    timeline of PX4 times around every command it gives. `save` writes that
    timeline to disk; it runs after every command."""

    def __init__(self, sl, sticks, save=None):
        self.sl = sl
        self.sticks = sticks
        self.events = []
        self.misses = 0
        self.armed_read = 0.0
        self.save = save or (lambda: None)

    def now_us(self, tries=3):
        """PX4's clock, us since boot (the newest gyro sample's timestamp), or None."""
        for _ in range(tries):
            value = self.sl.field(self.sl.listener('vehicle_angular_velocity'), 'timestamp')

            if value is not None:
                return int(value)

        return None

    def state(self, guard=True):
        """{'t_us', 'u', 'v', 'r', 'alt'} from the groundtruth and the gyro, or
        None when either did not read. With `guard`, Abort below MIN_HEIGHT_M
        or when the vehicle is no longer armed."""
        gt = self.sl.listener(GT_POS)
        xyz = self.sl.vector(self.sl.listener('vehicle_angular_velocity'), 'xyz')
        t, vx, vy, z, psi = (self.sl.field(gt, k) for k in ('timestamp', 'vx', 'vy', 'z', 'heading'))

        if xyz is None or len(xyz) < 3 or not self.sl.finite(t, vx, vy, z, psi, xyz[2]):
            self.misses += 1

            if self.misses > 25:
                raise Abort('px4 가 상태를 25 번 연속 내주지 않았다')

            return None

        self.misses = 0
        s = {'t_us': int(t), 'u': vx * math.cos(psi) + vy * math.sin(psi), 'v': -vx * math.sin(psi) + vy * math.cos(psi),
             'r': math.degrees(xyz[2]), 'alt': -z}

        if not guard:
            return s

        if s['alt'] < MIN_HEIGHT_M:
            raise Abort('고도 %.1f m: 바닥 %g m 아래다' % (s['alt'], MIN_HEIGHT_M))

        if time.time() - self.armed_read > 2.0:
            self.armed_read = time.time()
            arming = self.sl.field(self.sl.listener('vehicle_status'), 'arming_state')

            if arming is not None and int(arming) != ARMED:
                raise Abort('무장이 풀렸다 (arming_state %d)' % int(arming))

        return s

    def event(self, leg, name, action, keep=None, **extra):
        """Run `action` between two reads of PX4's clock and keep the bracket:
        in the timeline, in `keep` under `name` when given (a leg's events),
        and on disk before this returns. A run killed from outside (an outer
        `timeout` signals the whole process group, and SIGTERM runs no
        `finally`) still leaves every command it gave."""
        before = self.now_us()
        action()
        after = self.now_us()
        self.events.append(dict(leg=leg, name=name, px4_before_us=before, px4_after_us=after, wall=time.time(), **extra))

        if keep is not None:
            keep[name] = (before, after)

        self.save()
        return before, after

    def mode(self, leg, name, mode, keep=None):
        """Switch the main mode and confirm it from vehicle_status; Abort if it
        does not change: every leg depends on knowing which loop is closed."""
        want = {'manual': NAV_MANUAL, 'acro': NAV_ACRO}[mode]
        bracket = self.event(leg, name, lambda: self.sl.commander('mode', mode), keep=keep, mode=mode)

        for _ in range(15):
            nav = self.sl.field(self.sl.listener('vehicle_status'), 'nav_state')

            if nav is not None and int(nav) == want:
                return bracket

            time.sleep(0.2)

        raise Abort('%s 로 바뀌지 않았다' % mode)

    def wait(self, seconds, since_us=None, done=None):
        """Until PX4's clock is `seconds` past `since_us` (now when None), or
        `done(state)` is true. Bounded on the wall. Returns (got_it, sim
        seconds elapsed, last state); got_it is True only when `done` said so,
        or, without `done`, when the time ran out on the simulator's clock."""
        start = since_us if since_us is not None else self.now_us()

        if start is None:
            raise Abort('PX4 시각을 읽지 못했다')

        deadline = time.time() + WALL_FACTOR * seconds + WALL_SLACK_S
        last = None

        while time.time() < deadline:
            s = self.state()

            if s is not None:
                last = s
                elapsed = (s['t_us'] - start) / 1e6

                if done is not None and done(s):
                    return True, elapsed, s

                if elapsed >= seconds:
                    return done is None, elapsed, s

            time.sleep(POLL_S)

        reached = '읽은 상태 없음' if last is None else '%.1f s 까지 갔다' % ((last['t_us'] - start) / 1e6)
        raise Abort('시뮬레이터 시계가 %.0f s 를 벽시계 %.0f s 안에 가지 못했다 (%s)'
                    % (seconds, WALL_FACTOR * seconds + WALL_SLACK_S, reached))


def accelerate(live, leg, plan, drag_free):
    """Bring the hull to the leg's speed with the sticks centred, then set the
    leg's constant throttle. Returns (reached, sim seconds, last state)."""
    target = plan['speed']
    lead = ACCEL_RATE * LEAD_S
    s = None

    while s is None:
        s = live.state()

    if s['u'] > target + lead and drag_free:
        # Nothing slows a drag-free hull (the throttle stick has no reverse):
        # inside the steady tolerance it flies as it is, past it the leg cannot
        got = s['u'] <= target + 0.8 * max(SPEED_TOL * target, SPEED_TOL_ABS)
        elapsed = 0.0
    elif s['u'] > target + lead:
        live.event(leg, 'slow', lambda: setattr(live.sticks, 'throttle', 0.0), throttle=0.0)
        got, elapsed, s = live.wait(ACCEL_MAX_S, done=lambda st: st['u'] <= target + lead)
    elif s['u'] < target - lead:
        live.event(leg, 'accelerate', lambda: setattr(live.sticks, 'throttle', plan['accel_throttle']),
                   throttle=plan['accel_throttle'])
        got, elapsed, s = live.wait(ACCEL_MAX_S, done=lambda st: st['u'] >= target - lead)
    else:
        got, elapsed = True, 0.0

    live.event(leg, 'cruise', lambda: setattr(live.sticks, 'throttle', plan['throttle_command']),
               throttle=plan['throttle'])
    return got, elapsed, s


def wait_steady(live, target):
    """Bounded wait for straight flight at `target`, judged on the last
    STEADY_WINDOW_S + POLL_S of samples, and only once every one of them was
    taken after the leg's throttle was set (this starts right after it). The
    log's verdict reads the STEADY_WINDOW_S before the Acro switch, which then
    lies wholly in the constant-throttle phase too. A live window allowed to
    be shorter let the log's reach back into the acceleration: on the test's
    fake plant it read the throttle at 0.043 for a trim of 0.031.
    Returns (steady, sim seconds waited, numbers of the last window)."""
    samples = []
    numbers = {}
    span = STEADY_WINDOW_S + POLL_S

    def done(s):
        samples.append(s)

        if (s['t_us'] - samples[0]['t_us']) / 1e6 < span:
            return False

        start = s['t_us'] - int(span * 1e6)
        window = [w for w in samples if w['t_us'] >= start]
        del samples[1:len(samples) - len(window)]          # keep the first (the phase start) and the window
        t0 = window[0]['t_us']
        ok, found = steady_judgement([(w['t_us'] - t0) / 1e6 for w in window], [w['u'] for w in window],
                                     [w['v'] for w in window], [w['r'] for w in window], target)
        numbers.update(found)
        return ok

    got, elapsed, _ = live.wait(STEADY_MAX_S, done=done)
    return got, elapsed, numbers


def brake(live, leg):
    """From Manual, which the release ended in and which cleared the
    integrator: Acro with the stick centred until the turn has stopped.
    Bounded; returns (stopped, sim seconds)."""
    live.wait(MANUAL_DWELL_S)
    live.mode(leg, 'brake', 'acro')
    quiet = {}

    def done(s):
        if abs(s['r']) < RATE_TOL_DEG:
            quiet.setdefault('since', s['t_us'])
            return (s['t_us'] - quiet['since']) / 1e6 >= BRAKE_HOLD_S

        quiet.clear()
        return False

    got, elapsed, _ = live.wait(BRAKE_MAX_S, done=done)
    return got, elapsed


def fly_leg(live, record, mode, drag_free, last):
    """One leg, a. to e., filling `record` as it goes: the caller keeps it
    before the leg starts, so a run stopped mid-leg (Abort) still hands the
    analysis the events it reached, and the log's part of that leg is
    measured and judged incomplete rather than dropped."""
    number, plan = record['leg'], record['plan']
    leg = 'leg%d' % number
    note('\n다리 %d: %.2f m/s, 요 스틱 %+.2f (지시 %.2f deg/s), 스로틀 %d/1000, 가속 %s'
         % (number, plan['speed'], plan['stick'], plan['setpoint_deg_s'], plan['throttle_counts'], mode))

    if mode == 'manual':
        live.mode(leg, 'accel_mode', 'manual')
    else:
        live.mode(leg, 'accel_mode', 'acro')

    got, elapsed, s = accelerate(live, leg, plan, drag_free)
    note('  가속: %s, %.1f s, u %.3f m/s' % ('도달' if got else '도달 못함', elapsed, s['u'] if s else float('nan')))
    record['accel'] = {'reached': got, 'sim_s': elapsed}

    if not got:
        note('  이 다리는 날지 않는다: %.1f m/s 에 닿지 못했다%s'
             % (plan['speed'], ' (항력 없는 기체는 속도를 줄일 수 없다)' if drag_free and s and s['u'] > plan['speed'] else ''))
        return

    steady, waited, numbers = wait_steady(live, plan['speed'])
    record['steady_live'] = {'steady': steady, 'sim_s': waited, 'numbers': numbers}
    note('  정상 직진 %s: %.1f s 대기, u %s m/s (변화 %s), |r| 최대 %s deg/s, |v| 최대 %s m/s'
         % ('도달' if steady else '도달 못함', waited, *('?' if numbers.get(k) is None else '%.3f' % numbers[k]
                                                      for k in ('u_mean', 'u_drift', 'r_max_abs', 'v_max_abs'))))

    if not steady:
        note('  이 다리는 날지 않는다: 정상 직진이 아닌 진입에서 건 스틱은 재현과 비교할 수 없다')
        return

    if mode == 'acro':
        live.mode(leg, 'entry_manual', 'manual')
        live.wait(MANUAL_DWELL_S)

    # Each bracket goes into the record before the event writes the timeline
    # to disk, and the leg counts as flown from the save that carries its stick
    events = record['events']
    live.mode(leg, 'acro', 'acro', keep=events)
    live.wait(ENTRY_S, since_us=events['acro'][1])
    record['flown'] = True
    loss_s = source_loss_s()
    source = live.sl.param_show('AS_SPD_SRC') if loss_s is not None else None
    live.event(leg, 'stick_on', lambda: setattr(live.sticks, 'yaw', plan['stick']), keep=events, stick=plan['stick'])

    if loss_s is not None:
        live.wait(loss_s, since_us=events['stick_on'][1])
        live.event(leg, 'source_lost', lambda: live.sl.param_set('AS_SPD_SRC', SOURCE_LOSS_SRC), keep=events,
                   value=SOURCE_LOSS_SRC, was=source)

    live.wait(STICK_S, since_us=events['stick_on'][1])
    live.event(leg, 'stick_off', lambda: setattr(live.sticks, 'yaw', 0.0), keep=events)
    live.wait(RELEASE_S, since_us=events['stick_off'][1])
    live.mode(leg, 'release_end', 'manual', keep=events)

    if loss_s is not None:
        live.event(leg, 'source_back', lambda: live.sl.param_set('AS_SPD_SRC', int(source)), keep=events,
                   value=int(source))

    note('  스틱 %g s, 놓음 %g s 끝%s' % (STICK_S, RELEASE_S, '' if loss_s is None else
                                       ' (스틱 %g s 에 속도원 잃음, 놓음 끝에 AS_SPD_SRC %d 로 되돌림)'
                                       % (loss_s, int(source))))

    if not last:
        stopped, took = brake(live, leg)
        record['brake'] = {'stopped': stopped, 'sim_s': took}
        note('  제동: %s, %.1f s' % ('멈췄다' if stopped else '멈추지 못했다', took))


def read_params(sl, names):
    """({name: value or None}, {name: changed or None}) from one `param show
    -a` dump, None as a whole when px4-param never answered. A None value is a
    name this firmware does not have: the full dump lists every one,
    registered or not, which is why it is read rather than `param show <name>`
    (sitl_link.param_show). `changed` is the dump's second flag column, '+'
    (saved) or '*' (unsaved) where a value is not the default the firmware,
    the airframe and rcS gave it (param.cpp do_show_print): what tells a
    PX4_PARAM_ that came through from one that only equals its default."""
    line = sl.PARAM_LINE.replace('^[x+*l ]*', '^([x+*l ]*)', 1)

    if line == sl.PARAM_LINE:
        raise LookupError('sitl_link.PARAM_LINE 의 모양이 바뀌어 플래그 열을 읽을 수 없다: %s' % sl.PARAM_LINE)

    for _ in range(15):
        text = sl.px4('param', 'show', '-a', timeout=30)

        if text is not None and 'parameters total' in text:
            values, changed = {}, {}

            for name in names:
                found = re.search(line % re.escape(name), text, re.M)
                values[name] = float(found.group(2)) if found else None
                changed[name] = bool(set(found.group(1)) & set('+*')) if found else None

            return values, changed

        time.sleep(2)

    return None


def logger_path(sl):
    """The file the logger is writing, absolute, or None."""
    text = sl.px4('logger', 'status')
    found = re.search(r'Log file:\s*(\S+\.ulg)', text or '')

    if not found or 'Full File Logging Running' not in (text or ''):
        return None

    rel = found.group(1)
    candidates = [os.path.normpath(os.path.join(ROOTFS, rel))]

    if sl.INSTANCE != '0':
        candidates.append(os.path.normpath(os.path.join(ROOTFS, sl.INSTANCE, rel)))

    return next((c for c in candidates if os.path.exists(c)), None)


def close_log(sl, path):
    """Stop the file and wait until it stops growing. SDLOG_MODE 1 stops it at
    disarm; `logger off` drops a manual override, `logger stop` the module.
    Returns (closed, how)."""
    def running():
        text = sl.px4('logger', 'status')
        return text is None or 'Full File Logging Running' in text

    how = None

    for command, wait_s in ((None, 15.0), ('off', 10.0), ('stop', 10.0)):
        if command:
            sl.px4('logger', command)

        end = time.time() + wait_s

        while time.time() < end:
            if not running():
                how = command or 'disarm'
                break

            time.sleep(0.5)

        if how:
            break

    size = -1

    for _ in range(20):
        now = os.path.getsize(path) if os.path.exists(path) else -1

        if now == size and now > 0:
            return how is not None, how or 'still running'

        size = now
        time.sleep(1.0)

    return False, 'size still changing'


def tree_head():
    import subprocess

    try:
        return subprocess.run(['git', '-C', gz_model.ROOT, 'rev-parse', '--short=10', 'HEAD'],
                              capture_output=True, text=True, timeout=10).stdout.strip() or None
    except (OSError, subprocess.TimeoutExpired):
        return None


def stopped():
    """A precondition stopped the run before anything was measured: the tally,
    then exit 2, the module docstring's code for it, never the 1 of a
    measurement that failed."""
    summary()
    return 2


def main_live(spec):
    model = entity()

    if model is None:
        print('GZ_TARGET 도 PX4_GZ_ENTITY 도 없다: gz_run.sh 로 돌린다 (또는 --plan)')
        return 2

    refuse(model)
    import sitl_link as sl

    if not sl.commander('status'):
        note('px4 에 붙지 못했다')
        return 3

    started = time.time()
    plan_files = plan_params(model)
    overrides = {k[len('PX4_PARAM_'):]: v for k, v in os.environ.items() if k.startswith('PX4_PARAM_')}
    names = PARAMS + LOG_PARAMS + SCHEDULE_PARAMS
    read = read_params(sl, names + tuple(k for k in overrides if k not in names))

    if read is None:
        print('px4-param 이 응답하지 않는다')
        return 3

    live_params, changed = read
    # The legs are planned on what the firmware runs; a value it lacks keeps
    # the files' and is named, so the plan never carries a hole
    missing = [k for k in ('AS_YAWRATE_P', 'AS_YAWRATE_I', 'AS_YR_INT_LIM', 'AS_YAWRATE_FF', 'AS_YAWRATE_MAX',
                           'MAN_DEADZONE', 'SYS_AUTOSTART', 'THR_MDL_FAC') if live_params.get(k) is None]

    if missing:
        print('펌웨어에서 %s 를 읽지 못했다: 계획을 세울 수 없다' % ', '.join(missing))
        return 2

    params = {k: live_params[k] if live_params.get(k) is not None else plan_files[k] for k in PARAMS}
    absent = [k for k in PARAMS if live_params.get(k) is None]

    if absent:
        note('펌웨어에 없어서 파일 값을 쓴다: %s' % ', '.join(absent))

    sched_live = {k: live_params.get(k) for k in SCHEDULE_PARAMS}
    note('속도 스케줄 %s: %s' % ('켬' if schedule_on(sched_live) else '끔', ', '.join(
        '%s %s' % (k, '없음' if v is None else '%g' % v) for k, v in sched_live.items())))

    try:
        loss_s = source_loss_s()
    except ValueError:
        loss_s = float('nan')

    if loss_s is not None:
        ok = schedule_on(sched_live) and 0.0 < loss_s <= STICK_S - SOURCE_LOSS_MARGIN_S
        verdict('%s 는 스케줄이 켜진 펌웨어에서 스틱의 (0, %g] s 이다' % (SOURCE_LOSS_ENV, STICK_S - SOURCE_LOSS_MARGIN_S),
                ok, '%s=%s, 스케줄 %s' % (SOURCE_LOSS_ENV, os.environ.get(SOURCE_LOSS_ENV),
                                         '켬' if schedule_on(sched_live) else '끔'))

        if not ok:
            return stopped()

    legs = parse_legs(os.environ.get('FORWARD_LEGS') if spec is None else spec, model)
    plan = vehicle_plan(model, legs, params)
    print_plan(plan)
    mode = plan['accel_mode']
    drag_free = gz_model.translational_drag(gz_model.merged(model))[0] == 0.0
    head = tree_head()
    different = {k: (plan_files[k], params[k]) for k in PARAMS if params.get(k) is not None
                 and abs(plan_files[k] - params[k]) > 1e-6}
    note('펌웨어 값이 파일과 %s' % ('같다' if not different else '다르다: ' + ', '.join(
        '%s 파일 %g 펌웨어 %g' % (k, a, b) for k, (a, b) in different.items())))
    airframe_number = gz_model.airframe(model)[0]
    dirs_max = live_params.get('SDLOG_DIRS_MAX')
    ok = int(params['SYS_AUTOSTART']) == airframe_number and abs(params['THR_MDL_FAC'] - 1.0) < 1e-6 \
        and live_params.get('SDLOG_MODE') is not None and int(live_params['SDLOG_MODE']) in (0, 1, 2, 4)
    verdict('펌웨어가 이 계획의 에어프레임과 추력 모델로 떠 있고 로그를 남긴다', ok,
            'SYS_AUTOSTART %d (기대 %d), THR_MDL_FAC %g (트림은 추력 비율이 선형일 때의 것), SDLOG_MODE %s, '
            'SDLOG_PROFILE %s, SDLOG_DIRS_MAX %s' % (int(params['SYS_AUTOSTART']), airframe_number, params['THR_MDL_FAC'],
                                                    live_params.get('SDLOG_MODE'), live_params.get('SDLOG_PROFILE'),
                                                    dirs_max))

    if not ok:
        return stopped()

    if dirs_max is None or dirs_max > 0:
        note('주의: SDLOG_DIRS_MAX %s: 로거는 시작할 때 날짜 폴더가 이보다 많으면 가장 오래된 폴더의 로그를 지운다 '
             '(logger/util.cpp); PX4_PARAM_SDLOG_DIRS_MAX=0 이 지킨다' % dirs_max)

    # rcS sets each PX4_PARAM_<name> after the airframe (`param set
    # ${name#PX4_PARAM_}`), and the plan (D1 0b) asks that the route be seen
    # working before anything relies on it. Every one must read back as asked,
    # and one at least as changed from its default: a value equal to its
    # default reads back the same whether the route works or not.
    def took(name, wanted):
        try:
            value = float(wanted)
        except ValueError:
            return False
        return live_params.get(name) is not None and abs(live_params[name] - value) <= 1e-6 * max(1.0, abs(value))

    arrived = {k: took(k, v) for k, v in overrides.items()}
    shown = [k for k in overrides if arrived[k] and changed.get(k)]
    ok = bool(shown) and all(arrived.values())
    verdict('환경의 PX4_PARAM_ 이 rcS 를 거쳐 펌웨어에 들어갔다 (기본값이 아닌 값이 하나 이상)', ok,
            ', '.join('%s=%s -> %s%s' % (k, overrides[k], live_params.get(k),
                                         ' (들어가지 않았다)' if not arrived[k] else ' (기본값과 다르다)' if changed.get(k)
                                         else ' (기본값과 같아 경로를 보이지 못한다)') for k in overrides)
            or 'PX4_PARAM_ 이 하나도 없다: 계획 (D1 0b) 은 이 경로를 먼저 확인한다. '
               'PX4_PARAM_SDLOG_DIRS_MAX=0 이 경로를 보이고 오래된 로그 폴더도 지킨다')

    if not ok:
        return stopped()

    print('EKF 를 기다린다...', flush=True)

    if not sl.wait_for_ekf():
        print('EKF 가 유효해지지 않았다')
        return 2

    time.sleep(SETTLE_S)
    sl.param_set('MAN_ARM_GESTURE', 0)
    sticks = sl.Sticks()
    sticks.start()
    records = []
    log_source = events_path = stamp = None
    aborted = None
    doc = {'model': model, 'airframe': airframe_number, 'source': None, 'copy': None, 'closed': None,
           'started_wall': started, 'tree_head': head, 'params_live': live_params, 'params_changed': changed,
           'params_files': plan_files, 'overrides': overrides, 'aborted': None, 'finished': False, 'plan': plan,
           'legs': records, 'events': None}
    failed_saves = []

    def save():
        """The live document, whole, replaced atomically, so neither a reader
        nor a kill ever meets half a file. A write that fails is said once
        and the flight goes on: the ULog is the record, this its index."""
        if events_path is None:
            return

        partial = events_path + '.partial'

        try:
            with open(partial, 'w') as handle:
                json.dump(doc, handle, indent=1, ensure_ascii=False, default=float)

            os.replace(partial, events_path)
        except OSError as why:
            if not failed_saves:
                note('사건 기록을 쓰지 못했다: %s' % why)

            failed_saves.append(str(why))

    live = Live(sl, sticks, save)
    doc['events'] = live.events

    try:
        time.sleep(2)
        sl.commander('mode', 'manual')
        time.sleep(1)
        sl.commander('arm')
        time.sleep(3)
        armed = sl.field(sl.listener('vehicle_control_mode'), 'flag_armed') == 1.0
        verdict('Manual 에서 무장했다', armed, '무장됨' if armed else '무장 거부')

        if not armed:
            return stopped()

        s = None

        for _ in range(5):
            s = s or live.state(guard=False)

        verdict('모델이 월드에 있고 떠 있다', s is not None and s['alt'] >= MIN_HEIGHT_M,
                '고도 %s m (바닥 %g m), u %s m/s' % ('?' if s is None else '%.1f' % s['alt'], MIN_HEIGHT_M,
                                               '?' if s is None else '%.3f' % s['u']))

        if s is None or s['alt'] < MIN_HEIGHT_M:
            return stopped()

        log_source = logger_path(sl)
        note('로그: %s' % (log_source or '찾지 못했다'))

        if log_source is None:
            verdict('이 실행이 쓰는 로그를 찾았다', False, '`logger status` 가 쓰고 있는 파일을 말하지 않았다')
            return stopped()

        stamp = stamp_of(log_source)
        os.makedirs(OUT, exist_ok=True)
        events_path = os.path.join(OUT, 'forward_%s_%s_events.json' % (model, stamp))
        doc['source'] = log_source
        save()
        note('사건 기록 (명령마다 다시 쓴다): %s' % events_path)

        for position, leg_plan_ in enumerate(plan['legs'], 1):
            record = {'leg': leg_plan_['number'], 'rerun': leg_plan_['rerun'], 'plan': leg_plan_, 'flown': False,
                      'events': {}}
            records.append(record)
            save()
            fly_leg(live, record, mode, drag_free, position == len(plan['legs']))
            save()

    except Abort as why:
        aborted = str(why)
        note('\n중단: %s' % aborted)
    except BaseException as why:
        # an interrupt (gz_run.sh passes INT on) or a defect: on the record,
        # then on its way
        aborted = '%s: %s' % (type(why).__name__, why)
        raise
    finally:
        doc['aborted'] = aborted
        save()
        sticks.yaw = 0
        sticks.throttle = 0
        time.sleep(0.5)
        sl.commander('mode', 'manual')
        time.sleep(0.5)
        sl.commander('disarm', '-f')
        time.sleep(1.0)
        sticks.stop = True

    if log_source is None:
        # An Abort before the log was found. None can come today (the floating
        # check reads five states, an Abort takes 26 misses), but one that did
        # would have measured nothing, and the verdicts so far must not add up
        # to a pass; px4 not answering is what an Abort that early would be
        verdict('이 실행이 쓰는 로그를 찾았다', False, '로그를 찾기 전에 중단: %s' % aborted)
        summary()
        return 3

    closed, how = close_log(sl, log_source)
    copy = os.path.join(OUT, 'forward_%s_%s.ulg' % (model, stamp))
    shutil.copy2(log_source, copy)
    copied = os.path.getsize(copy) == os.path.getsize(log_source)
    doc.update(copy=copy, closed=how, finished=True)
    save()

    try:
        log = Log.read(copy)
    except Exception as why:        # a truncated or unreadable file is the finding, not a crash
        verdict('이 실행이 쓴 로그를 찾아 닫고 읽었다', False, '%s: %s' % (copy, why))
        return summary()

    ours = (log.param('SYS_AUTOSTART') is not None and int(log.param('SYS_AUTOSTART')) == airframe_number
            and os.path.getmtime(log_source) >= started)
    verdict('이 실행이 쓴 로그를 찾아 닫고 읽었다', closed and copied and ours,
            '%s -> %s (%.1f MB, %s 로 닫힘), SYS_AUTOSTART %s, 펌웨어 %s (트리 HEAD %s), 길이 %.1f s'
            % (log_source, os.path.relpath(copy, HERE), os.path.getsize(copy) / 1e6, how, log.param('SYS_AUTOSTART'),
               log.info.get('ver_sw', '?')[:10], head, (log.end_us - log.start_us) / 1e6))

    if head and not str(log.info.get('ver_sw', '')).startswith(head):
        note('주의: 로그의 펌웨어 %s 가 트리 HEAD %s 가 아니다' % (log.info.get('ver_sw', '?')[:10], head))

    summary_doc, checks = analyze(log, model, live=doc, out_dir=OUT, stamp=stamp)
    summary_doc['live'] = {'events': os.path.relpath(events_path, HERE), 'aborted': aborted, 'plan': plan,
                           'events_write_failures': failed_saves}
    report_analysis(summary_doc, checks, OUT, stamp, model)

    if aborted:
        verdict('모든 다리를 끝까지 날았다', False, '중단: %s' % aborted)

    unflown = [r['leg'] for r in records if not r['flown']]
    unreached = [p['number'] for p in plan['legs'][len(records):]]

    if unflown or unreached:
        verdict('계획한 다리를 모두 날았다', False, '날지 않은 다리: %s%s' % (
            unflown or '없음', '; 중단으로 시작하지 못한 다리: %s' % unreached if unreached else ''))

    return summary()


def main(argv=None):
    parser = argparse.ArgumentParser(description='forward flight of the airship yaw rate loop (D1 step 0b)')
    parser.add_argument('--plan', action='store_true', help='print the legs, trims and time budget; no px4')
    parser.add_argument('--legs', help='as FORWARD_LEGS: 1-based default leg numbers or speed:stick, comma separated')
    parser.add_argument('--analyze', metavar='ULG', help='redo the records of a kept log')
    parser.add_argument('--events', metavar='JSON', help='with --analyze: the live timeline that log was flown with')
    parser.add_argument('--out', metavar='DIR', help='with --analyze: where the records go (default sitl_out)')
    parser.add_argument('--release', type=float, default=RELEASE_S,
                        help='with --analyze: the release window, s (a gate log releases for 12)')
    parser.add_argument('--hardware', metavar='MODEL',
                        help='with --analyze: a hardware log, measured on the estimator\'s frame, flown as this plant (its gz twin)')
    args = parser.parse_args(argv)

    if args.analyze:
        if args.hardware:
            refuse(args.hardware)
            use_estimator_frame()

        return main_analyze(args.analyze, args.events, args.out, args.release, model=args.hardware)

    if args.plan:
        model = entity()
        models = [model] if model else list(ALLOWED)

        for name in models:
            refuse(name)
            legs = parse_legs(args.legs if args.legs is not None else os.environ.get('FORWARD_LEGS'), name)
            print_plan(vehicle_plan(name, legs), table=name.startswith('cloudship'))
            print()

        return 0

    return main_live(args.legs)


if __name__ == '__main__':
    sys.exit(main())
