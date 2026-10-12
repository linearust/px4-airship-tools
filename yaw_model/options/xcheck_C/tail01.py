"""Cloudship tail lag 0.1 s (2 x the side motors' timeConstant 0.05 in
cloudship/model.sdf:476-494; the tail's own gz branch has none) at the best
settings: gate verdicts and margins, 3 seeds."""
import json, os, sys
import multiprocessing as mp
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import xrun
jobs = [('mine', t, 0.7, 0.2, b, 'nominal', {}, s, ('tail_tau', 0.1))
        for t in ('gz_cloudship', 'gz_cloudship_am_fin_drag') for b in (1.0, 0.9) for s in (301, 302, 303)]
with mp.Pool(6) as pool, open(os.path.join(xrun.HERE, 'x_tail01.jsonl'), 'w') as fh:
    for r in pool.imap_unordered(xrun.one, jobs):
        fh.write(json.dumps(r, ensure_ascii=False) + '\n')
print('done')
