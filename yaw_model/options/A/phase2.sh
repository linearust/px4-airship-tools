#!/bin/bash
cd "$(dirname "$0")"
python3 ensemble.py ens.jsonl all d8 0.7 0.2 0.3 0.725 0.2 0.3 0.675 0.2 0.3 0.7 0.18 0.3 0.5 0.09 0.3 0.7 0.09 0.3
python3 ensemble.py ens.jsonl vectored d8 0.725 0.18 0.3 0.75 0.16 0.3 0.7 0.16 0.3 0.7 0.2 0.15
python3 ensemble.py ens.jsonl cloudship d8 0.75 0.2 0.3 0.8 0.14 0.3 0.775 0.12 0.3
echo PHASE2 finished
