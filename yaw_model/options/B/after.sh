#!/bin/bash
# Off-gate behaviour and actuator sensitivity for the candidate settings
cd "$(dirname "$0")"
T=3600
# 1. step / wind / noise
timeout $T python3 beyond.py beyond.jsonl --configs "0.5,0.09,0;0.7,0.09,0;0.7,0.09,0.1" --pool 6
timeout $T python3 beyond.py beyond.jsonl --configs "0.7,0.09,1.0" --plants 1 --pool 6
timeout $T python3 beyond.py beyond.jsonl --configs "0.7,0.09,0.3" --plants 4 --pool 6
# quadratic FF, same torque as the linear one at 6.67 deg/s (K = FF / 0.1164 rad/s)
timeout $T python3 beyond.py beyond.jsonl --configs "0.7,0.09,0" --plants 1 --act '{"ffq": 8.594}' --kinds 0.25,0.5,1.0 --pool 6
timeout $T python3 beyond.py beyond.jsonl --configs "0.7,0.09,0" --plants 3 --act '{"ffq": 0.8594}' --kinds 0.25,0.5,1.0 --pool 6
timeout $T python3 beyond.py beyond.jsonl --configs "0.7,0.09,0" --plants 4 --act '{"ffq": 2.578}' --kinds 0.25,0.5,1.0 --pool 6
# 2. gate under actuator variants (nominal timing, gyro noise, seeds 1-5)
for A in '{"slew": 60}' '{"slew": 240}' '{"phys_slew": 60}'; do
  timeout $T python3 sweep.py act.jsonl --configs "0.7,0.09,0" --plants 0,1,4 --members noisy --act "$A" --pool 6
  timeout $T python3 sweep.py act.jsonl --configs "0.7,0.09,1.0" --plants 1 --members noisy --act "$A" --pool 6
  timeout $T python3 sweep.py act.jsonl --configs "0.7,0.09,0.3" --plants 4 --members noisy --act "$A" --pool 6
  timeout $T python3 beyond.py beyond_act.jsonl --configs "0.7,0.09,0" --plants 0,1,4 --kinds 0.5 --act "$A" --pool 6
  timeout $T python3 beyond.py beyond_act.jsonl --configs "0.7,0.09,1.0" --plants 1 --kinds 0.5 --act "$A" --pool 6
  timeout $T python3 beyond.py beyond_act.jsonl --configs "0.7,0.09,0.3" --plants 4 --kinds 0.5 --act "$A" --pool 6
done
for A in '{"tail_tau": 0.025}' '{"tail_tau": 0.05}'; do
  timeout $T python3 sweep.py act.jsonl --configs "0.7,0.09,0;0.7,0.09,0.1" --plants 2,3 --members noisy --act "$A" --pool 6
  timeout $T python3 beyond.py beyond_act.jsonl --configs "0.7,0.09,0;0.7,0.09,0.1" --plants 2,3 --kinds 0.5 --act "$A" --pool 6
done
echo AFTER DONE
