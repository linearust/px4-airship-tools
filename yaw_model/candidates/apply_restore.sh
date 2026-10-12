#!/bin/bash
# Apply option D (the yaw integrator hold, option_d_restore.diff) and AS_YAWRATE_P 0.7
# to the tree, uncommitted. Run from PX4-Autopilot at 860ebca856. The diff is the
# whole code change; the P default belongs to the gain decision and stays here.
set -e
S=$(dirname "$0")
git apply "$S/option_d_restore.diff"
python3 - <<'PY'
import io
p = 'src/modules/airship_att_control/airship_att_control_params.yaml'
s = io.open(p, encoding='utf-8').read()
i, j = s.index('    AS_YAWRATE_P:'), s.index('    AS_YAWRATE_I:')
blk = s[i:j]; assert blk.count('default: 0.5') == 1
io.open(p, 'w', encoding='utf-8').write(s[:i] + blk.replace('default: 0.5', 'default: 0.7') + s[j:])
PY
echo "integrator hold + P 0.7 applied"
