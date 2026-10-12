#!/bin/bash
# Start the airship in headless gz SITL, run a script against it, clean up.
#
#     ./gz_run.sh ./sitl_check.py
#
# GZ_TARGET picks the make target, so the same check can run against the
# plain model and the hull-aerodynamics one:
#     GZ_TARGET=gz_airship_vectored_independent_drag gz_run.sh sitl_check.py
#
# It is the only variable that needs setting. This script resolves it into
# PX4_GZ_ENTITY - the name px4 actually spawns - and exports that, so no check
# has to guess. Do not also set PX4_SIM_MODEL: the make target overrides it
# with `cmake -E env`, so a value set here changes nothing except what the
# checks believe.
#
# The stdin pipe into make is load-bearing: with stdin closed the pxh console
# spins printing its prompt and fills the disk.
set -u
ROOT="${PX4_ROOT:-$HOME/dev/PX4-Autopilot}"
TOOLS="$(cd "$(dirname "$0")" && pwd)"
SCRIPT="${1:-$TOOLS/sitl_check.py}"
# Before the boot, not after it. A wrong path used to cost the full make, the
# parameter wipe and the 150 s uORB wait, and then say "can't open file".
[ -f "$SCRIPT" ] || { echo "검사 파일이 없다: $SCRIPT"; exit 7; }
LOG="$TOOLS/sitl_out/server.log"
export GZ_IP=127.0.0.1
# SITL's rcS sets SDLOG_DIRS_MAX 7, and a logger that starts with more date
# folders than that deletes the oldest folder's logs (logger/util.cpp). The
# yaw model validates on the 2026-09-25 and 2026-09-27 logs, so every run here
# keeps them all unless the caller says otherwise. rcS applies PX4_PARAM_*
# after the airframe and before rc.logging; the server log shows
# "SDLOG_DIRS_MAX: curr: 7 -> new: 0" when it took (2026-10-01).
export PX4_PARAM_SDLOG_DIRS_MAX="${PX4_PARAM_SDLOG_DIRS_MAX:-0}"

cleanup() {
	pkill -9 -x px4 2>/dev/null
	pkill -9 -f "[g]z sim" 2>/dev/null
	pkill -f "[t]ail -f /dev/null" 2>/dev/null
	rm -f /tmp/px4_lock-0 /tmp/px4-sock-0
}

# A killed run used to leave px4 and gz sim holding the ports, so the next
# run raced them; the explicit cleanup at the end never reached an
# interrupted shell
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

mkdir -p "$TOOLS/sitl_out"
cleanup
# gz-transport's discovery socket outlives the process that held it, so a
# server started too soon after one was SIGKILLed can fail to bind and die
# with "Subprocess killed" before px4 ever sees a world. One second was not
# enough: the two runs that hit this both followed a short-lived gz.
sleep 6

# Start from the airframe defaults, not from a previous run's saved values.
# px4 keeps them in rootfs/fs/, not rootfs/: this used to delete the latter,
# which is exactly the failure the line below is here to catch, except that
# the check had the same wrong path and so passed on an empty glob every time.
# Every run since was inheriting the one before it, which is how a probe's
# SIM_GZ_EC_MIN1 of -300 turned up in the next airframe.
PARAMS="$ROOT/build/px4_sitl_default/rootfs/fs"
rm -f "$PARAMS"/parameters*.bson "$ROOT"/build/px4_sitl_default/rootfs/parameters*.bson

if compgen -G "$PARAMS/parameters*.bson" >/dev/null; then
	echo "저장된 파라미터를 지우지 못했다: $PARAMS"
	exit 1
fi

export GZ_TARGET="${GZ_TARGET:-gz_airship_vectored_independent}"

# Resolve the model and the world here, once, and export both. Every check used
# to infer the model, and inferring it is how two bugs got in: PX4_SIM_MODEL was
# preferred over GZ_TARGET although the make target sets PX4_SIM_MODEL itself
# with `cmake -E env` and the caller's value never reaches px4, and the
# `gz_<model>_<world>` target shape was never accounted for. Both produce a
# name that exists nowhere, and that failure is silent: the wrenches go to no
# entity and the checks measure the stillness.
#
# CMake generates one target per model per world (gz_bridge/CMakeLists.txt:113):
# `gz_<model>` for the default world and `gz_<model>_<world>` for every other.
# So the split is against the worlds that exist, which is the same data CMake
# used - subtracting a guessed suffix turned gz_cloudship_windy into a model
# called "cloudship_windy" and then into an error about a missing directory,
# which is the right refusal for the wrong reason.
MODELS="$ROOT/Tools/simulation/gz/models"
WORLDS="$ROOT/Tools/simulation/gz/worlds"
ENTITY="${GZ_TARGET#gz_}"
WORLD="${PX4_GZ_WORLD:-}"

if [ ! -d "$MODELS/$ENTITY" ]; then
	for candidate in "$WORLDS"/*.sdf; do
		name=$(basename "$candidate" .sdf)

		case "$ENTITY" in
		*_"$name")
			if [ -d "$MODELS/${ENTITY%_$name}" ]; then
				WORLD="$name"
				ENTITY="${ENTITY%_$name}"
				break
			fi
			;;
		esac
	done
fi

# The airframes default to lta, which is the only world here that carries a
# Buoyancy plugin, so an unqualified target lands there.
export PX4_GZ_ENTITY="$ENTITY"
export PX4_GZ_WORLD="${WORLD:-lta}"

if [ ! -d "$MODELS/$ENTITY" ]; then
	echo "실패: $ENTITY 라는 모델이 $MODELS 아래에 없다 (GZ_TARGET=$GZ_TARGET)"
	exit 8
fi

# A world with no buoyancy for this entity is fatal for a lighter-than-air
# vehicle and silent everywhere else: the hull simply falls, and every check
# below then measures a plummeting airship without saying so. The plugin lifts
# only the links it is given, so the spawn name has to be in the list.
if ! grep -q 'buoyancy-system' "$WORLDS/$PX4_GZ_WORLD.sdf" 2>/dev/null; then
	echo "실패: $PX4_GZ_WORLD 월드에는 Buoyancy 플러그인이 없다. 비행선은 뜨지 않는다."
	echo "  부력이 있는 월드: $(grep -l 'buoyancy-system' "$WORLDS"/*.sdf | xargs -n1 basename | tr '\n' ' ')"
	exit 9
elif ! grep -q "<enable>${ENTITY}_0::" "$WORLDS/$PX4_GZ_WORLD.sdf"; then
	echo "실패: $PX4_GZ_WORLD 월드의 Buoyancy <enable> 목록에 ${ENTITY}_0 이 없다."
	echo "  목록에 없는 링크에는 힘이 가지 않고, 경고도 없다."
	exit 9
fi

# Spawn clear of the ground. These models put their origin on the ground under
# the gondola, so the default spawn rests the hull on the ground plane, and a
# hull that is too HEAVY then does not move at all: the buoyancy check measured
# zero drift, inferred zero imbalance and passed. Excess lift was visible and a
# shortfall was not, which is the direction that actually grounds the vehicle.
# Airborne the measurement is two-sided, and every other check is rid of the
# contact friction it was running against for its first minute.
export PX4_GZ_MODEL_POSE="${PX4_GZ_MODEL_POSE:-0,0,20}"

( tail -f /dev/null | HEADLESS=1 make -C "$ROOT" px4_sitl "$GZ_TARGET" > "$LOG" 2>&1 ) &

up=""
for _ in $(seq 1 150); do
	"$ROOT/build/px4_sitl_default/bin/px4-listener" vehicle_status 1 >/dev/null 2>&1 && { up=1; break; }
	sleep 1
done

# The wait used to have no failure branch: it ran out after 150 s and the check
# ran anyway, against nothing, and reported whatever a total absence of data
# looks like to it.
if [ -z "$up" ]; then
	echo "실패: px4 가 150 초 안에 uORB 에 응답하지 않았다. $LOG 를 본다"
	grep -m3 -E "Subprocess|error|Error|ERROR" "$LOG" | sed 's/^/  /'
	exit 5
fi

# px4's output reaches the log through a pipe and is block buffered, so it
# answering on uORB says nothing about what has been written. Wait for the
# outcome to appear rather than for the request, and long enough that a full
# block has had to flush.
for _ in $(seq 1 40); do
	grep -qE "Gazebo world is ready|Timed out waiting for Gazebo world|gazebo already running world" "$LOG" && break
	sleep 1
done

# Two different things: that px4 asked for a world, and that the world came up.
# Only the first used to be checked, so a gz server that started and died read
# as a healthy run whose checks then found an empty world and blamed the model.
if grep -q "Timed out waiting for Gazebo world" "$LOG"; then
	echo "실패: gz 월드가 올라오지 않았다. $LOG 를 본다"
	grep -m3 -E "Subprocess|error|Error|ERROR" "$LOG" | sed 's/^/  /'
	exit 4
elif grep -q "gazebo already running world" "$LOG"; then
	# cleanup killed every gz sim before the boot, so another process started
	# this world meanwhile. px4-rc.gzsim attaches to the first /world/*/clock
	# it finds and then prints the readiness line for that world too, so this
	# is tested first. The checks were given PX4_GZ_WORLD=$PX4_GZ_WORLD, whose
	# buoyancy was validated above; the attached world is neither.
	echo "실패: px4 가 이 스크립트가 띄우지 않은 gz 월드 $(grep -m1 'gazebo already running world' "$LOG" | sed 's/.*world: //') 에 붙었다"
	exit 10
elif grep -q "Gazebo world is ready" "$LOG"; then
	grep -m1 "Starting gazebo with world" "$LOG" | sed 's#.*/##; s/\.sdf//; s/^/월드: /'
else
	# Not a warning. px4 answers on uORB long before its world exists, so
	# reaching here means the readiness line never arrived in 40 s and
	# nothing has confirmed there is a world to measure.
	echo "실패: 월드가 준비되었다는 줄이 로그에 없다. $LOG 를 본다"
	tail -5 "$LOG" | sed 's/^/  /'
	exit 6
fi

echo "기체: $PX4_GZ_ENTITY, 월드: $PX4_GZ_WORLD (GZ_TARGET=$GZ_TARGET)"
python3 "$SCRIPT"
status=$?

cleanup
exit "$status"
