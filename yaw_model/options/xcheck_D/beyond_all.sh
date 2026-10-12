#!/bin/bash
cd "$(dirname "$0")"
export W=3
python3 xbeyond.py bey.jsonl rel0.5,rel1.0 none,restore 0.7 1,2
python3 xbeyond.py bey.jsonl relwind+,relwind-,stale+,stale-,gone+,gone- none,restore,reset 0.7 1
python3 xbeyond.py bey.jsonl wstep0.05,edge none,restore,reset 0.7 1
python3 xbeyond.py bey_act.jsonl rel0.5 none,restore 0.7 1 alloc_slew=60
python3 xbeyond.py bey_act.jsonl rel0.5 none,restore 0.7 1 servo=60
python3 xbeyond.py bey_act.jsonl rel0.5 none,restore 0.7 1 alloc_slew=240
python3 xbeyond.py bey_act.jsonl rel0.5 none,restore 0.7 1 tail_tau=0.05
python3 xbeyond.py bey_act.jsonl rel0.5 none,restore 0.7 1 tail_tau=0.1
echo ALLDONE
