#!/bin/bash
cd "$(dirname "$0")"
while pgrep -f "timeout 5400 ./phase2[.]sh" > /dev/null; do sleep 10; done
python3 ensemble.py ens.jsonl all 1-5 0.7 0.2 0.3 0.725 0.2 0.3
python3 ensemble.py ens.jsonl all d8 act=slew:60 0.7 0.2 0.3 0.5 0.09 0.3
python3 ensemble.py ens.jsonl all d8 act=slew:240 0.7 0.2 0.3
python3 ensemble.py ens.jsonl all d8 act=servo:60 0.7 0.2 0.3 0.5 0.09 0.3
python3 ensemble.py ens.jsonl all d8 act=tail_tau:0.05 0.7 0.2 0.3 0.5 0.09 0.3
python3 ensemble.py ens.jsonl all d8 act=tail_tau:0.1 0.7 0.2 0.3
python3 ensemble.py probe.jsonl all n1 0.7 0.25 0.3 0.7 0.3 0.3 0.7 0.4 0.3 0.8 0.3 0.3 0.7 0.2 0.1 0.7 0.2 0.08
echo PHASE3 finished
