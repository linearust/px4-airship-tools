#!/bin/bash
# 2026-09-28: chain12's 2517 Acro failure was the harness, not the firmware: the triple wrench clear delayed the
# Manual check 2.4 s and changed the plant state every later check started from. gz_link now sends the first clear
# only and the repeats from a thread (the next push waits for them). The five gates again on the same firmware.
cd "$(dirname "$0")/.."
for t in gz_cloudship_am_fin_drag gz_airship_vectored_independent gz_airship_vectored_independent_am_fin_drag gz_airship_vectored_independent_fin gz_cloudship; do
  echo "=== gate $t ==="; GZ_TARGET=$t timeout 1800 ./gz_run.sh ./sitl_check.py 2>&1 | grep -E "통과|실패|걸리지|지시 각속도|렌치" | cut -c1-360; echo "exit ${PIPESTATUS[0]}"
  cp -p sitl_out/samples.csv "sitl_out/samples_${t}_chain12b.csv" 2>/dev/null
done
echo "=== CHAIN DONE ==="
