#!/bin/bash
# Start the airship in headless gz SITL, run a script against it, clean up.
#
#     ./gz_run.sh ./sitl_check.py
#
# GZ_TARGET picks the model, so the same check can run against the plain
# plant and the hull-aerodynamics one:
#     GZ_TARGET=gz_airship_vectored_independent_drag gz_run.sh sitl_check.py
#
# It is the only variable that needs setting. This script resolves it into
# PX4_GZ_ENTITY - the name actually spawned - and PX4_GZ_WORLD, and exports
# both, so no check has to guess. Do not also set PX4_SIM_MODEL: on the tree
# path the make target overrides it with `cmake -E env`, on the overlay path
# this script does, so a value set here changes nothing except what the
# checks believe.
#
# A model has two homes (gz_model.MODEL_HOMES), and the home decides the
# launch path:
#   tree    - the model is under the PX4 tree: `make px4_sitl gz_<model>`,
#             which builds, starts the world and spawns, as before.
#   overlay - the model is under the overlay (AIRSHIP_OVERLAY, default
#             <this repo>/overlay): there is no make target for it and PX4's
#             rc.gzsim resolves a model under its own models directory only,
#             so this script builds with `make px4_sitl`, installs the overlay
#             airframes into the build, starts the overlay's lab world itself,
#             spawns the model through the world's create service and runs the
#             px4 binary in its standalone mode, attached to that entity
#             (overlay/README.md; docs/upstream_split_design_20261012.md S4).
#             The overlay can only run against a trimmed tree: a tree that
#             still carries the family would give rcS two airframes.
# The overlay's world is `lta_lab`, so PX4_GZ_WORLD also tells the two kinds
# of run apart in a log.
#
# The stdin pipe into px4 is load-bearing: with stdin closed the pxh console
# spins printing its prompt and fills the disk.
set -u
ROOT="${PX4_ROOT:-$HOME/dev/PX4-Autopilot}"
TOOLS="$(cd "$(dirname "$0")" && pwd)"
SCRIPT="${1:-$TOOLS/sitl_check.py}"
# Before the boot, not after it. A wrong path used to cost the full make, the
# parameter wipe and the 150 s uORB wait, and then say "can't open file".
[ -f "$SCRIPT" ] || { echo "검사 파일이 없다: $SCRIPT"; exit 7; }
# gz_model.OVERLAY's rule, resolved once and exported so the check reads the
# same home this script spawned from.
export AIRSHIP_OVERLAY="${AIRSHIP_OVERLAY:-$TOOLS/overlay}"
OVERLAY="$AIRSHIP_OVERLAY"
# The one world the generator writes the overlay members into
# (airship_family_gen.OVERLAY_WORLD_NAME, gz_model.OVERLAY_WORLD)
OVERLAY_WORLD=lta_lab
LOG="$TOOLS/sitl_out/server.log"
GZLOG="$TOOLS/sitl_out/gz.log"
BUILD="$ROOT/build/px4_sitl_default"
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
PARAMS="$BUILD/rootfs/fs"
rm -f "$PARAMS"/parameters*.bson "$BUILD"/rootfs/parameters*.bson

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
# The overlay is asked first, by the same test gz_model.model_dir makes (a
# model.sdf under the home). A name in both homes is refused, as
# gz_model.model_names refuses it: rcS would find two airframes for the one
# PX4_SIM_MODEL and silently load the tree's.
MODELS="$ROOT/Tools/simulation/gz/models"
WORLDS="$ROOT/Tools/simulation/gz/worlds"
ENTITY="${GZ_TARGET#gz_}"
WORLD="${PX4_GZ_WORLD:-}"
PATH_KIND=tree

if [ -f "$OVERLAY/models/$ENTITY/model.sdf" ]; then
	if [ -d "$MODELS/$ENTITY" ]; then
		echo "실패: $ENTITY 모델이 두 집에 있다: $OVERLAY/models 와 $MODELS. 오버레이는 트림된 트리에서만 돈다."
		exit 8
	fi

	# rcS finds the airframe by listing the build's airframes directory for
	# `<number>_gz_<model>`; the overlay's copy of that listing is its own
	# airframes directory, so a member without one would boot to "Unknown
	# model" after the build.
	AIRFRAME=$(compgen -G "$OVERLAY/airframes/[0-9]*_gz_$ENTITY" | head -1)

	if [ -z "$AIRFRAME" ]; then
		echo "실패: 오버레이 모델 $ENTITY 의 에어프레임 (<번호>_gz_$ENTITY) 이 $OVERLAY/airframes 에 없다"
		exit 8
	fi

	# The overlay has one world. A caller's PX4_GZ_WORLD naming another is a
	# tree habit that would send the checks to a world this run never
	# started, so it is refused rather than overridden.
	if [ -n "$WORLD" ] && [ "$WORLD" != "$OVERLAY_WORLD" ]; then
		echo "실패: 오버레이 멤버 $ENTITY 는 $OVERLAY_WORLD 월드에서만 뜬다 (PX4_GZ_WORLD=$WORLD)"
		exit 9
	fi

	PATH_KIND=overlay
	WORLD="$OVERLAY_WORLD"
	WORLD_SDF="$OVERLAY/worlds/$OVERLAY_WORLD.sdf"
elif [ ! -d "$MODELS/$ENTITY" ]; then
	# CMake generates one target per model per world (gz_bridge/CMakeLists.txt:113):
	# `gz_<model>` for the default world and `gz_<model>_<world>` for every other.
	# So the split is against the worlds that exist, which is the same data CMake
	# used - subtracting a guessed suffix turned gz_cloudship_windy into a model
	# called "cloudship_windy" and then into an error about a missing directory,
	# which is the right refusal for the wrong reason.
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

# The tree airframes default to lta, which is the only tree world that carries
# a Buoyancy plugin, so an unqualified target lands there.
export PX4_GZ_ENTITY="$ENTITY"
export PX4_GZ_WORLD="${WORLD:-lta}"

if [ "$PATH_KIND" = tree ]; then
	WORLD_SDF="$WORLDS/$PX4_GZ_WORLD.sdf"

	if [ ! -d "$MODELS/$ENTITY" ]; then
		echo "실패: $ENTITY 라는 모델이 $MODELS 아래에도 $OVERLAY/models 아래에도 없다 (GZ_TARGET=$GZ_TARGET)"
		exit 8
	fi
fi

# A world with no buoyancy for this entity is fatal for a lighter-than-air
# vehicle and silent everywhere else: the hull simply falls, and every check
# below then measures a plummeting airship without saying so. The plugin lifts
# only the links it is given, so the spawn name has to be in the list.
if ! grep -q 'buoyancy-system' "$WORLD_SDF" 2>/dev/null; then
	echo "실패: $PX4_GZ_WORLD 월드 ($WORLD_SDF) 에는 Buoyancy 플러그인이 없다. 비행선은 뜨지 않는다."
	[ "$PATH_KIND" = tree ] && echo "  부력이 있는 월드: $(grep -l 'buoyancy-system' "$WORLDS"/*.sdf | xargs -n1 basename | tr '\n' ' ')"
	exit 9
elif ! grep -q "<enable>${ENTITY}_0::" "$WORLD_SDF"; then
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

if [ "$PATH_KIND" = tree ]; then
	( tail -f /dev/null | HEADLESS=1 make -C "$ROOT" px4_sitl "$GZ_TARGET" > "$LOG" 2>&1 ) &
	job=$!
else
	# Build only: the ROMFS regeneration of a build deletes the installed
	# overlay airframes, so the install has to follow every build, and the
	# run cannot start before either. A build has no natural bound, so it
	# gets one here rather than holding the gate open for ever.
	if ! timeout 1800 env HEADLESS=1 make -C "$ROOT" px4_sitl > "$LOG" 2>&1; then
		echo "실패: make px4_sitl 이 실패했다. $LOG 를 본다"
		grep -m3 -E "error|Error|FAILED" "$LOG" | sed 's/^/  /'
		exit 3
	fi

	# install.sh reads AIRSHIP_OVERLAY (exported above), so a scratch overlay
	# installs from itself through the repository's script.
	if ! "$TOOLS/overlay/install.sh" "$ROOT" >> "$LOG" 2>&1; then
		echo "실패: 오버레이 에어프레임을 빌드에 넣지 못했다. $LOG 를 본다"
		exit 3
	fi

	# The build's gz_env.sh is what rc.gzsim would source: the tree's models
	# and worlds, PX4's system plugins and its server.config. gz_env.sh
	# appends to variables it does not guard, so `set -u` goes off around it.
	# Names never collide between the two homes, so the overlay going first
	# is a convention, not a precedence.
	set +u
	. "$BUILD/gz_env.sh"
	set -u
	export GZ_SIM_RESOURCE_PATH="$OVERLAY/models:$GZ_SIM_RESOURCE_PATH"

	gz sim --verbose=1 -r -s "$WORLD_SDF" > "$GZLOG" 2>&1 &
	gzjob=$!
	up=""

	# The same readiness test rc.gzsim makes (a provider on the world's
	# scene/info service), with the same 30 s; every gz CLI call below is
	# wrapped, since one against a dead server waits on discovery.
	for _ in $(seq 1 30); do
		kill -0 "$gzjob" 2>/dev/null || break
		timeout 5 gz service -i --service "/world/$PX4_GZ_WORLD/scene/info" 2>&1 | grep -q "Service providers" && { up=1; break; }
		sleep 1
	done

	if [ -z "$up" ]; then
		echo "실패: gz 월드 $PX4_GZ_WORLD 가 30 초 안에 올라오지 않았다. $GZLOG 를 본다"
		grep -m3 -E "\[Err\]|Error|error" "$GZLOG" | sed 's/^/  /'
		exit 4
	fi

	# A world whose plugins did not load comes up and answers like a healthy
	# one; the Buoyancy plugin checked above is one of them, and a model
	# whose own plugins failed is a hull that does nothing.
	if grep -q "Failed to load system plugin" "$GZLOG"; then
		echo "실패: gz 가 플러그인을 싣지 못했다. $GZLOG 를 본다"
		grep -m3 "Failed to load system plugin" "$GZLOG" | sed 's/^/  /'
		exit 4
	fi

	# What rc.gzsim does with PX4_SIM_MODEL, from the overlay's model file:
	# the instance name <model>_0 is what gz_bridge attaches to, the pose is
	# PX4_GZ_MODEL_POSE, which the standalone px4 never reads.
	IFS=, read -r px py pz pr pp pyaw <<< "$PX4_GZ_MODEL_POSE"
	pose="${px:-0} ${py:-0} ${pz:-0} ${pr:-0} ${pp:-0} ${pyaw:-0}"
	sdf="<sdf version=\"1.6\"><include><uri>file://$OVERLAY/models/$ENTITY/model.sdf</uri><pose>$pose</pose></include></sdf>"
	reply=$(timeout 20 gz service -s "/world/$PX4_GZ_WORLD/create" --reqtype gz.msgs.EntityFactory \
		--reptype gz.msgs.Boolean --timeout 5000 \
		--req "name: \"${ENTITY}_0\", allow_renaming: false, sdf: '$sdf'" 2>&1)

	if ! grep -q "data: true" <<< "$reply"; then
		echo "실패: $PX4_GZ_WORLD 월드가 ${ENTITY}_0 을 만들지 않았다: ${reply:-응답 없음}"
		grep -m3 -E "\[Err\]|Error" "$GZLOG" | sed 's/^/  /'
		exit 4
	fi

	# The create service answers true for a request it accepted, not for a
	# model that exists: an include whose uri does not resolve is reported in
	# the server log and nothing is spawned. pose/info carries every entity,
	# so the name has to be in it before px4 is told to attach.
	if ! timeout 20 gz topic -e -t "/world/$PX4_GZ_WORLD/pose/info" -n 1 2>/dev/null | grep -q "name: \"${ENTITY}_0\""; then
		echo "실패: ${ENTITY}_0 이 $PX4_GZ_WORLD 월드에 없다. $GZLOG 를 본다"
		grep -m3 -E "\[Err\]|Error" "$GZLOG" | sed 's/^/  /'
		exit 4
	fi

	# PX4's documented standalone path (px4-rc.gzsim): no world is started
	# or looked for, the readiness wait goes against PX4_GZ_WORLD, and
	# PX4_GZ_MODEL_NAME makes gz_bridge attach instead of spawn. The working
	# directory is the one the make targets use (SITL_WORKING_DIR), which is
	# also px4's own default.
	( cd "$BUILD/rootfs" && tail -f /dev/null | env PX4_SIM_MODEL="gz_$ENTITY" PX4_GZ_STANDALONE=1 \
		PX4_GZ_MODEL_NAME="${ENTITY}_0" PX4_GZ_WORLD="$PX4_GZ_WORLD" "$BUILD/bin/px4" >> "$LOG" 2>&1 ) &
	job=$!
fi

up=""
for _ in $(seq 1 150); do
	kill -0 "$job" 2>/dev/null || break
	"$BUILD/bin/px4-listener" vehicle_status 1 >/dev/null 2>&1 && { up=1; break; }
	sleep 1
done

# The wait used to have no failure branch: it ran out after 150 s and the check
# ran anyway, against nothing, and reported whatever a total absence of data
# looks like to it.
if [ -z "$up" ]; then
	echo "실패: px4 가 끝났거나 150 초 안에 uORB 에 응답하지 않았다. $LOG 를 본다"
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
	# buoyancy was validated above; the attached world is neither. The overlay
	# path never reaches here: its px4 runs standalone and is told the world
	# this script started, so a foreign world is not attached to, only
	# ignored.
	echo "실패: px4 가 이 스크립트가 띄우지 않은 gz 월드 $(grep -m1 'gazebo already running world' "$LOG" | sed 's/.*world: //') 에 붙었다"
	exit 10
elif grep -q "gz_bridge failed to start" "$LOG"; then
	# The world was ready and the bridge still did not come up: on the
	# overlay path that is an attach to a name the world does not hold.
	echo "실패: gz_bridge 가 ${ENTITY}_0 에 붙지 못했다. $LOG 를 본다"
	grep -m3 -E "gz_bridge|ERROR" "$LOG" | sed 's/^/  /'
	exit 4
elif grep -q "Gazebo world is ready" "$LOG"; then
	if [ "$PATH_KIND" = tree ]; then
		grep -m1 "Starting gazebo with world" "$LOG" | sed 's#.*/##; s/\.sdf//; s/^/월드: /'
	else
		echo "월드: $PX4_GZ_WORLD (오버레이, $WORLD_SDF)"
	fi
else
	# Not a warning. px4 answers on uORB long before its world exists, so
	# reaching here means the readiness line never arrived in 40 s and
	# nothing has confirmed there is a world to measure.
	echo "실패: 월드가 준비되었다는 줄이 로그에 없다. $LOG 를 본다"
	tail -5 "$LOG" | sed 's/^/  /'
	exit 6
fi

# On the overlay path the airframe is the fork's, and a trimmed firmware may
# not carry every parameter it sets; rcS runs with `set +e`, so each such line
# is a warning in the log and nothing else. Said here, once, so a gate verdict
# is read against what the airframe actually applied.
if [ "$PATH_KIND" = overlay ]; then
	unknown=$(grep -oE "Parameter [A-Z0-9_]+ not found" "$LOG" | sort -u | sed 's/Parameter //; s/ not found//' | tr '\n' ' ')
	[ -n "$unknown" ] && echo "참고: 이 펌웨어에 없는 파라미터를 에어프레임이 건드렸다: $unknown"
fi

echo "기체: $PX4_GZ_ENTITY, 월드: $PX4_GZ_WORLD, 경로: $PATH_KIND (GZ_TARGET=$GZ_TARGET)"
python3 "$SCRIPT"
status=$?

cleanup
exit "$status"
