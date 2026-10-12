#!/bin/bash
cd "$(dirname "$0")"
# 1. the recommended set: seeds 1-5 (option A's own) and 11-15 (fresh), 8 members each
python3 batch.py main.jsonl all 1-5,11-15 0.7 0.2 0.3
# 2. the window edges and beyond the searched I range (vectored), fresh seeds
python3 batch.py edges.jsonl vectored 11,12 0.725 0.2 0.3 0.675 0.2 0.3 0.7 0.18 0.3 0.7 0.22 0.3 0.7 0.25 0.3
# 3. current gains, fresh seed, all plants
python3 batch.py edges.jsonl all 11 0.5 0.09 0.3
# 4. actuator variants, fresh seed
for a in slew:60 slew:240 servo:60 tail_tau:0.05 tail_tau:0.1; do
  python3 batch.py act.jsonl all 11 act=$a 0.7 0.2 0.3 0.5 0.09 0.3
done
echo ALL FINISHED
