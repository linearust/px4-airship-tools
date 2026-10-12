#!/bin/bash
# Per-plant FF at P 0.65 I 0.07: the judged ensemble (8 timing members x gyro noise x seeds 1-5)
cd "$(dirname "$0")"
S=1,2,3,4,5
timeout 7200 python3 sweep.py ens2.jsonl --configs "0.65,0.07,0" --plants 0,2,3 --members full --seeds $S --pool 12
timeout 7200 python3 sweep.py ens2.jsonl --configs "0.65,0.07,1.0" --plants 1 --members full --seeds $S --pool 12
timeout 7200 python3 sweep.py ens2.jsonl --configs "0.65,0.07,0.1" --plants 3 --members full --seeds $S --pool 12
timeout 7200 python3 sweep.py ens2.jsonl --configs "0.65,0.07,0.3" --plants 4 --members full --seeds $S --pool 12
echo ENS2 DONE
