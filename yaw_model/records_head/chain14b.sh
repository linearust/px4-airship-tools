#!/bin/bash
# 2026-09-29: chain14's 2520 lost both weak-wind pushes (the new guard marked the case unmeasured; the signed integrator
# showed -0.0073 against 0.0074). sitl_check now reads the integrator signed. 2520 twice, publishes traced.
cd "$(dirname "$0")/.."
for n in 1 2; do
  t=gz_airship_vectored_independent
  echo "=== gate $t run $n ==="; GZ_LINK_TRACE=1 GZ_TARGET=$t timeout 1800 ./gz_run.sh ./sitl_check.py 2>&1 | grep -E "통과|실패|걸리지|지시 각속도|렌치|gz_publish" | cut -c1-360; echo "exit ${PIPESTATUS[0]}"
  cp -p sitl_out/samples.csv "sitl_out/samples_${t}_chain14b_${n}.csv" 2>/dev/null
done
echo "=== CHAIN DONE ==="
