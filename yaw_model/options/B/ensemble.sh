#!/bin/bash
# The judged runs: 8 timing members of the reviewers' ensemble x gyro noise x seeds 1-5 (40 per plant-config)
cd "$(dirname "$0")"
S=1,2,3,4,5
timeout 7200 python3 sweep.py ens.jsonl --configs "0.7,0.09,0;0.7,0.09,0.1" --members full --seeds $S --pool 12
timeout 7200 python3 sweep.py ens.jsonl --configs "0.7,0.09,1.0" --plants 1 --members full --seeds $S --pool 12
timeout 7200 python3 sweep.py ens.jsonl --configs "0.7,0.09,0.3" --plants 4 --members full --seeds $S --pool 12
echo ENS DONE
