#!/bin/sh
# Put the overlay airframes where a built PX4 SITL finds them, and print the
# environment that makes gz find the overlay models.
#
#     overlay/install.sh [PX4_ROOT]      # default $PX4_ROOT or ~/dev/PX4-Autopilot
#
# The overlay is AIRSHIP_OVERLAY (gz_model.OVERLAY's rule), default this
# directory, so a scratch overlay installs through this same script.
#
# rcS resolves PX4_SIM_MODEL and SYS_AUTOSTART by listing
# build/px4_sitl_default/etc/init.d-posix/airframes, so a file copied there is
# a working airframe without a CMake entry. The ROMFS regeneration of every
# `make px4_sitl` deletes the copies, so this runs after the build and before
# the launch (gz_run.sh does both). The printed GZ_SIM_RESOURCE_PATH line is
# for the gz server gz_run.sh starts; names never collide between the overlay
# and the tree (overlay/README.md), so the order of the two is immaterial.
set -eu
HERE="$(cd "$(dirname "$0")" && pwd)"
OVERLAY="${AIRSHIP_OVERLAY:-$HERE}"
ROOT="${1:-${PX4_ROOT:-$HOME/dev/PX4-Autopilot}}"
ETC="$ROOT/build/px4_sitl_default/etc/init.d-posix/airframes"
[ -d "$ETC" ] || { echo "no built SITL under $ROOT (expected $ETC)" >&2; exit 1; }

. "$HERE/../pins.env"
have=$(git -C "$ROOT" rev-parse HEAD 2>/dev/null || echo unknown)
[ "$have" = "$PX4_COMMIT" ] || echo "note: $ROOT is at $(printf %.10s "$have"), the overlay was generated against $(printf %.10s "$PX4_COMMIT")" >&2

# Only the airframe files: the directory also holds the generator's `decades`
# table, which rcS's `<number>_` listing would ignore but has no place there.
n=0
for f in "$OVERLAY"/airframes/[0-9]*_gz_*; do
	[ -f "$f" ] || continue
	cp "$f" "$ETC/"
	n=$((n + 1))
done
echo "installed $n airframe files from $OVERLAY/airframes into $ETC" >&2
echo "export GZ_SIM_RESOURCE_PATH=$OVERLAY/models\${GZ_SIM_RESOURCE_PATH:+:\$GZ_SIM_RESOURCE_PATH}"
