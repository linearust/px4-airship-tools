#!/usr/bin/env python3
"""What the rigid vectored brake and release windows hold beyond the gate's
reading: the reverse swing (min rate), the mean |rate| of the window's last
3 s, and the rate at the window's end, per gain set and seed."""
import json, os, sys, multiprocessing as mp
sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import xgate, gate
def one(job):
    t, P, I, seed = job
    loop = xgate.build(t, {'AS_YAWRATE_P': P, 'AS_YAWRATE_I': I, 'AS_YR_INT_LIM': 0.3}, seed)
    od = os.path.join(HERE, 'tmp_out', str(os.getpid())); os.makedirs(od, exist_ok=True)
    res = gate.run_gate(loop, xgate.model_name(t), gate.Timing(), out_dir=od)
    ph = {}
    for r in res.samples: ph.setdefault(r['phase'], []).append(r)
    out = {'target': t, 'P': P, 'I': I, 'seed': seed}
    for name, key in (('Acro 제동', 'brake'), ('Acro 놓음', 'acro_rel'), ('Stabilized 놓음', 'stab_rel')):
        rows = ph[name]; last = rows[-1]['t']
        s0 = 1 if rows[0]['rate'] > 0 else -1
        out[key] = {'start': rows[0]['rate'], 'reverse_peak': -s0 * min(s0 * r['rate'] for r in rows),
                    'end': rows[-1]['rate'], 'tail3_mean_abs': sum(abs(r['rate']) for r in rows if r['t'] >= last - 3) / len([r for r in rows if r['t'] >= last - 3]),
                    'settle': xgate.settled(rows, 1.0)}
    return out
if __name__ == '__main__':
    jobs = [(t, P, I, s) for t in ('gz_airship_vectored_independent', 'gz_cloudship')
            for P, I in ((0.5, 0.09), (0.7, 0.2)) for s in (11, 12, 13)]
    with mp.Pool(3) as pool, open(os.path.join(HERE, 'swing.jsonl'), 'w') as fh:
        for r in pool.imap_unordered(one, jobs):
            fh.write(json.dumps(r) + '\n'); fh.flush()
    print('done')
