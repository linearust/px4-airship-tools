#!/bin/sh
# The checks that need only the PX4 source tree at PX4_ROOT: no gz, no build.
# CI runs this against the commits in pins.env.
set -e
cd "$(dirname "$0")"
python3 airship_family_gen_test.py
python3 variant_check.py
python3 forward_yaw_check_test.py
python3 yaw_model/test_controller.py
# Not run here: yaw_model/test_forward.py, test_forward_replay.py,
# test_policies.py and test_fin_damping.py replay records flown on the
# 10.97 m hull the tree carried until 2026-10-07 and read the plant from the
# tree, which is the 7 m hull now; they wait for the model's re-anchoring
# (docs/professor_decisions_20261012.md).
