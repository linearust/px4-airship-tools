#!/bin/bash
# FF windows per plant over a P x I grid (noise-free nominal screen)
cd "$(dirname "$0")"
PI="0.5,0.05 0.5,0.09 0.5,0.2 0.7,0.05 0.7,0.2 1.0,0.05 1.0,0.09 1.0,0.2"
mk() { local out=""; for pi in $PI; do for ff in $1; do out="$out$pi,$ff;"; done; done; echo "$out"; }
timeout 3000 python3 sweep.py grid.jsonl --configs "$(mk '0 0.1 0.2')" --plants 0,2,3 --pool 11
timeout 3000 python3 sweep.py grid.jsonl --configs "$(mk '0.6 0.8 1.0 1.2')" --plants 1 --pool 11
timeout 3000 python3 sweep.py grid.jsonl --configs "$(mk '0 0.2 0.4')" --plants 4 --pool 11
echo GRID DONE
