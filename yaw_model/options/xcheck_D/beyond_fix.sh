#!/bin/bash
cd "$(dirname "$0")"
export W=3
python3 xbeyond.py bey.jsonl relwind+,relwind-,stale+,stale-,gone+,gone- none,restore,reset 0.7 1
echo FIXDONE
