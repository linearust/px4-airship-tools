#!/bin/bash
# Pod-only identity and branch coverage, no PX4 build needed (AirshipPod is
# pure math). The pod's outputs never feed the next step's demand, so a
# bit-identical tilt trajectory under the harness script is equivalent to
# full-harness identity for a change confined to AirshipPod.{cpp,hpp}.
#
#   pod_tools.sh replay  <dir-with-AirshipPod.cpp+hpp> <out.txt> [extra g++ flags]
#   pod_tools.sh coverage <dir-with-AirshipPod.cpp+hpp>
#
# Add "-mfma -ffp-contract=fast" as extra flags to reproduce the Cortex-M7
# contraction question on x86 (2026-09-16: the dot-loop and a*b+c*d spellings
# contract differently on ARM; the decisions are tie-banded so the trajectory
# stayed identical either way).
set -e
P=${PX4_DIR:-${PX4_ROOT:-$HOME/dev/PX4-Autopilot}}
H=$(dirname "$(readlink -f "$0")")
INC="-I $P/src -I $P/src/lib -I $P/src/lib/matrix -I $P/platforms/common/include -I $P/platforms/posix/include -I $P/src/include -I $P/build/px4_sitl_test"
FLAGS="-std=c++17 -O2 -w -fno-signed-zeros -fno-trapping-math -freciprocal-math -fno-math-errno"
case "$1" in
  replay)
    g++ $FLAGS ${@:4} $INC -I "$2" "$H/pod_replay.cpp" "$2/AirshipPod.cpp" -o /tmp/pod_replay && /tmp/pod_replay "$3" && wc -l "$3" ;;
  coverage)
    W=$(mktemp -d); cp "$2"/AirshipPod.hpp "$W"/; cp "$H"/pod_coverage.hpp "$W"/
    python3 - "$2/AirshipPod.cpp" "$W/AirshipPod_cov.cpp" <<'PY'
import sys
src=open(sys.argv[1]).read()
reps=[('#include "AirshipPod.hpp"','#include "AirshipPod.hpp"\n#include "pod_coverage.hpp"'),
 ('\tfloat tilt = atan2f(demand(1), demand(0));','\t++cov.total;\n\tfloat tilt = atan2f(demand(1), demand(0));'),
 ('\t\tif (fabsf(reverse_hi - reverse_lo) > FLT_EPSILON) {\n\t\t\ttilt = reverse_hi > reverse_lo ? rear_hi : rear_lo;','\t\t++cov.rear;\n\t\tif (fabsf(reverse_hi - reverse_lo) > FLT_EPSILON) {\n\t\t\t++cov.rear_pick;\n\t\t\ttilt = reverse_hi > reverse_lo ? rear_hi : rear_lo;\n\t\t\tcov.rear_pick_moves += (fabsf(tilt - _tilt_target) > 1e-3f);'),
 ('\t\t\ttilt = _tilt_target >= 0.f ? rear_hi : rear_lo;','\t\t\t++cov.rear_tie;\n\t\t\ttilt = _tilt_target >= 0.f ? rear_hi : rear_lo;'),
 ('\t} else if (tilt < _tilt_min || tilt > _tilt_max) {','\t} else if (tilt < _tilt_min || tilt > _tilt_max) {\n\t\t++cov.outrange;'),
 ('\t\tif (fmaxf(p_hi, p_lo) <= FLT_EPSILON) {','\t\tif (fmaxf(p_hi, p_lo) <= FLT_EPSILON) {\n\t\t\t++cov.out_hold;'),
 ('\t\t\t\ttilt = p_hi > p_lo + switch_margin ? _tilt_max : _tilt_min;\n\t\t\t}\n\t\t}','\t\t\t\ttilt = p_hi > p_lo + switch_margin ? _tilt_max : _tilt_min;\n\t\t\t}\n\n\t\t\tcov.out_switch += committed_hi ? (tilt == _tilt_min) : (tilt == _tilt_max);\n\t\t}'),
 ('\tif (fabsf(tilt - _tilt_target) > M_PI_F) {','\tif (fabsf(tilt - _tilt_target) > M_PI_F) {\n\t\t++cov.seam_checked;'),
 ('\t\tif (realized(demand, tilt) <= switch_margin) {','\t\t++cov.cone_priced;\n\t\tif (realized(demand, tilt) <= switch_margin) {\n\t\t\t++cov.cone_hold;'),
]
n=0
for a,b in reps:
    if a in src: src=src.replace(a,b,1); n+=1
# the seam hold: whichever spelling the seam compare uses, count the hold assignment inside it
import re
src,k=re.subn(r'(realized\(demand, _tilt_target\)\) \+ switch_margin\) \{\n)(\t\t\ttilt = _tilt_target;)', r'\1\t\t\t++cov.seam_held;\n\2', src, count=1)
open(sys.argv[2],'w').write(src)
TOTAL = 10
print(f"instrumented {n+k}/{TOTAL} anchors (tracking AirshipPod.cpp at e582116632)")
if n + k < TOTAL:
    sys.exit("pod_tools.sh: the source drifted from these anchors. A zero column below would be the TOOL, not a coverage hole.")
PY
    g++ $FLAGS $INC -I "$W" "$H/pod_coverage.cpp" "$W/AirshipPod_cov.cpp" -o /tmp/pod_coverage && /tmp/pod_coverage; rm -rf "$W" ;;
  *) sed -n '2,12p' "$0" ;;
esac
