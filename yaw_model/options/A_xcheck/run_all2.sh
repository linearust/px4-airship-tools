#!/bin/bash
cd "$(dirname "$0")"
# 1. the recommended set: fresh seeds 11-15 on every plant, plus option A's seed 1 as a reproduction spot check
#    (vec rigid already has seeds 1-5 and 11-15)
python3 batch.py main.jsonl 1,2,3,4 1,11-15 0.7 0.2 0.3
# 2. current gains, fresh seed, all plants
python3 batch.py edges.jsonl all 11 0.5 0.09 0.3
# 3. the window edges and beyond the searched I range (vectored), fresh seed
python3 batch.py edges.jsonl vectored 11 0.725 0.2 0.3 0.675 0.2 0.3 0.7 0.18 0.3 0.7 0.22 0.3 0.7 0.25 0.3
# 4. actuator variants, fresh seed
python3 batch.py act.jsonl vectored 11 act=slew:60 0.7 0.2 0.3 0.5 0.09 0.3
python3 batch.py act.jsonl vectored 11 act=servo:60 0.7 0.2 0.3 0.5 0.09 0.3
python3 batch.py act.jsonl vectored 11 act=slew:240 0.7 0.2 0.3
python3 batch.py act.jsonl cloudship 11 act=tail_tau:0.05 0.7 0.2 0.3
python3 batch.py act.jsonl cloudship 11 act=tail_tau:0.1 0.7 0.2 0.3
echo ALL FINISHED
