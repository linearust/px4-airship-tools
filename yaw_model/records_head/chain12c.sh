#!/bin/bash
# 2026-09-28: chain12b's 2527 lost two wind pushes (first check 0.02 -> 0.01 deg/s; I-on integrator 0.0092 against
# 0.0185, the push gone ~27 s in). Same firmware, 2527 twice with every gz publish traced against samples.csv time.
cd "$(dirname "$0")/.."
for n in 1 2; do
  t=gz_airship_vectored_independent_am_fin_drag
  echo "=== gate $t run $n ==="; GZ_LINK_TRACE=1 GZ_TARGET=$t timeout 1800 ./gz_run.sh ./sitl_check.py 2>&1 | grep -E "통과|실패|걸리지|지시 각속도|렌치|gz_publish" | cut -c1-360; echo "exit ${PIPESTATUS[0]}"
  cp -p sitl_out/samples.csv "sitl_out/samples_${t}_chain12c_${n}.csv" 2>/dev/null
done
echo "=== CHAIN DONE ==="
