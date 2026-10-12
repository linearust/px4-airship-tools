#!/bin/bash
cd "$(dirname "$0")"
C=""; for i in 0.12 0.15; do for ff in 0 0.1 0.2 0.3 0.4 0.6; do C="${C}0.7,$i,$ff;"; done; done
timeout 3000 python3 sweep.py grid2.jsonl --configs "$C" --pool 11
timeout 3000 python3 sweep.py grid2.jsonl --configs "0.7,0.2,0.05;0.7,0.2,0.1;0.7,0.2,0.2;0.7,0.2,0.3;0.7,0.2,0.4" --plants 1 --pool 11
timeout 3000 python3 sweep.py grid2.jsonl --configs "0.7,0.2,0.05" --plants 0,2,3,4 --pool 11
timeout 3000 python3 sweep.py grid2.jsonl --configs "0.65,0.2,0;0.65,0.2,0.1;0.65,0.2,0.2;0.65,0.2,0.3" --pool 11
echo GRID2 DONE
