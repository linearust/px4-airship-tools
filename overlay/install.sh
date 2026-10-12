#!/bin/sh
# Put the overlay airframes where a built PX4 SITL finds them, and print the
# environment that makes gz find the overlay models.
#
#     overlay/install.sh [PX4_ROOT]      # default $PX4_ROOT or ~/dev/PX4-Autopilot
#
# rcS resolves PX4_SIM_MODEL and SYS_AUTOSTART by listing
# build/px4_sitl_default/etc/init.d-posix/airframes, so a file copied there is
# a working airframe without a CMake entry; gz resolves a model by name on
# GZ_SIM_RESOURCE_PATH, and PX4's gz_env.sh appends its own directories after
# whatever the path already holds, so a directory exported before the launch
# wins. Re-run after every `make px4_sitl` that rebuilds the etc tree.
set -eu
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="${1:-${PX4_ROOT:-$HOME/dev/PX4-Autopilot}}"
ETC="$ROOT/build/px4_sitl_default/etc/init.d-posix/airframes"
[ -d "$ETC" ] || { echo "no built SITL under $ROOT (expected $ETC)" >&2; exit 1; }

. "$HERE/../pins.env"
have=$(git -C "$ROOT" rev-parse HEAD 2>/dev/null || echo unknown)
[ "$have" = "$PX4_COMMIT" ] || echo "note: $ROOT is at $(printf %.10s "$have"), the overlay was generated against $(printf %.10s "$PX4_COMMIT")" >&2

n=0
for f in "$HERE"/airframes/*; do
	[ -f "$f" ] || continue
	cp "$f" "$ETC/"
	n=$((n + 1))
done
echo "installed $n airframe files into $ETC" >&2
echo "export GZ_SIM_RESOURCE_PATH=$HERE/models\${GZ_SIM_RESOURCE_PATH:+:\$GZ_SIM_RESOURCE_PATH}"
