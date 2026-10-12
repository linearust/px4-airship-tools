#!/bin/bash
# per-plant FF at lower P / I: can every check be decided?
cd "$(dirname "$0")"
for pi in 0.6,0.05 0.65,0.05 0.6,0.07 0.65,0.07; do
  timeout 3000 python3 sweep.py grid4.jsonl --configs "$pi,0" --plants 0,2 --pool 6
  timeout 3000 python3 sweep.py grid4.jsonl --configs "$pi,1.0;$pi,1.2" --plants 1 --pool 6
  timeout 3000 python3 sweep.py grid4.jsonl --configs "$pi,0.1" --plants 3 --pool 6
  timeout 3000 python3 sweep.py grid4.jsonl --configs "$pi,0.3" --plants 4 --pool 6
done
echo GRID4 DONE
