#!/bin/bash
# 2026-09-28 night: the refactor pass (16 commits on d2647379af, tools entries 17-24). Expected as chain12b/12c:
# (2527 1.0, 2522 0.3, 2517 0.05, the rest 0). Expected: fixed flights 13/13, gates 10/10, 10/10, 10/10, 9/9, 9/9,
# tilt range 8/8. Each gate's time series kept as samples_<target>_chain13.csv; the gains each run used are in its
# '지시 각속도' line.
cd "$(dirname "$0")/.."
for t in gz_airship_fixed gz_airship_fixed_am_drag; do
  echo "=== flight $t ==="; GZ_TARGET=$t timeout 1500 ./gz_run.sh ./airship_fixed_flight_check.py 2>&1 | grep -E "통과|실패" | cut -c1-360; echo "exit ${PIPESTATUS[0]}"
done
for t in gz_airship_vectored_independent gz_airship_vectored_independent_am_fin_drag gz_airship_vectored_independent_fin gz_cloudship gz_cloudship_am_fin_drag; do
  echo "=== gate $t ==="; GZ_TARGET=$t timeout 1800 ./gz_run.sh ./sitl_check.py 2>&1 | grep -E "통과|실패|걸리지|지시 각속도" | cut -c1-360; echo "exit ${PIPESTATUS[0]}"
  cp -p sitl_out/samples.csv "sitl_out/samples_${t}_chain13.csv" 2>/dev/null
done
echo "=== tilt_range gz_airship_vectored_independent ==="; GZ_TARGET=gz_airship_vectored_independent timeout 1500 ./gz_run.sh ./tilt_range_check.py 2>&1 | grep -E "통과|실패" | cut -c1-300; echo "exit ${PIPESTATUS[0]}"
echo "=== CHAIN DONE ==="
