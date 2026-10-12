#!/bin/bash
# Off-gate behaviour and actuator sensitivity for the recommended per-plant set at P 0.65 I 0.07
cd "$(dirname "$0")"
T=3600; PI=0.65,0.07
until grep -q "ENS2 DONE" ens2.log; do sleep 10; done
for A in '{"slew": 60}' '{"slew": 240}' '{"phys_slew": 60}'; do
  timeout $T python3 sweep.py act2.jsonl --configs "$PI,0" --plants 0,1,4 --members noisy --act "$A" --pool 12
  timeout $T python3 sweep.py act2.jsonl --configs "$PI,1.0" --plants 1 --members noisy --act "$A" --pool 12
  timeout $T python3 sweep.py act2.jsonl --configs "$PI,0.3" --plants 4 --members noisy --act "$A" --pool 12
done
for A in '{"tail_tau": 0.025}' '{"tail_tau": 0.05}'; do
  timeout $T python3 sweep.py act2.jsonl --configs "$PI,0" --plants 2,3 --members noisy --act "$A" --pool 12
  timeout $T python3 sweep.py act2.jsonl --configs "$PI,0.1" --plants 3 --members noisy --act "$A" --pool 12
done
timeout $T python3 beyond.py beyond2.jsonl --configs "$PI,0" --pool 12
timeout $T python3 beyond.py beyond2.jsonl --configs "$PI,1.0" --plants 1 --pool 12
timeout $T python3 beyond.py beyond2.jsonl --configs "$PI,0.1" --plants 3 --pool 12
timeout $T python3 beyond.py beyond2.jsonl --configs "$PI,0.3" --plants 4 --pool 12
timeout $T python3 beyond.py beyond2.jsonl --configs "$PI,0" --plants 1 --act '{"ffq": 8.594}' --kinds 0.25,0.5,1.0 --pool 12
timeout $T python3 beyond.py beyond2.jsonl --configs "$PI,0" --plants 3 --act '{"ffq": 0.8594}' --kinds 0.25,0.5,1.0 --pool 12
timeout $T python3 beyond.py beyond2.jsonl --configs "$PI,0" --plants 4 --act '{"ffq": 2.578}' --kinds 0.25,0.5,1.0 --pool 12
for A in '{"slew": 60}' '{"slew": 240}' '{"phys_slew": 60}'; do
  timeout $T python3 beyond.py beyond2_act.jsonl --configs "$PI,0" --plants 0,1,4 --kinds 0.5 --act "$A" --pool 12
  timeout $T python3 beyond.py beyond2_act.jsonl --configs "$PI,1.0" --plants 1 --kinds 0.5 --act "$A" --pool 12
  timeout $T python3 beyond.py beyond2_act.jsonl --configs "$PI,0.3" --plants 4 --kinds 0.5 --act "$A" --pool 12
done
for A in '{"tail_tau": 0.025}' '{"tail_tau": 0.05}'; do
  timeout $T python3 beyond.py beyond2_act.jsonl --configs "$PI,0" --plants 2,3 --kinds 0.5 --act "$A" --pool 12
  timeout $T python3 beyond.py beyond2_act.jsonl --configs "$PI,0.1" --plants 3 --kinds 0.5 --act "$A" --pool 12
done
echo AFTER2 DONE
