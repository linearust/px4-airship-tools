#!/usr/bin/env python3
"""Synthesis runs: Option D's restore rule (options/D/option_d.py, imported, not
edited) at gain sets the option reports did not run, judged with D's own
gate_eval.metrics (sitl_check formulas, verdict asserted equal to the gate).

    python3 syn_run.py SPEC [SPEC ...]
SPEC = rule:P:I:mode[:seedbase]   mode lite (12 members) | full (40)
seedbase shifts every seed (fresh noise draws): seed -> seed + seedbase.
Appends to syn.jsonl (resumable)."""
import json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
D = os.path.join(os.path.dirname(HERE), 'D')
sys.path.insert(0, D); sys.dont_write_bytecode = True
import gate_eval as ge
import option_d as od
import ensemble as ens


def run_one(job):
    import gate
    t = job['target']
    params = {'AS_YAWRATE_P': job['P'], 'AS_YAWRATE_I': job['I']}
    loop = od.build(t, ens.CANDIDATES[job['rule']], params=params, noise=True, seed=job['seed'],
                    slew_deg=job.get('slew'), tail_tau=job.get('tail_tau'))
    out_dir = os.path.join(HERE, 'tmp_out', str(os.getpid()))
    os.makedirs(out_dir, exist_ok=True)
    res = gate.run_gate(loop, od.model_of(t), gate.Timing(**job['timing']), out_dir=out_dir)
    r = dict(job)
    r['status'] = res.status
    r['checks'] = ge.metrics(res, t)
    return r


def jobs(rule, P, I, mode, seedbase=0, targets=None, extra=None):
    out = []
    for t in (targets or od.TARGETS5):
        for label, tkw, seed in ens.members(mode):
            s = seed + seedbase
            j = dict(id='%s|P%g|I%g|%s|%s|%d' % (rule, P, I, t, label, s), rule=rule, P=P, I=I,
                     target=t, member=label, timing=tkw, seed=s)
            if extra:
                j.update(extra)
                j['id'] += '|' + ','.join('%s=%s' % kv for kv in sorted(extra.items()))
            out.append(j)
    return out


if __name__ == '__main__':
    import multiprocessing as mp
    all_jobs = []
    for spec in sys.argv[1:]:
        p = spec.split(':')
        all_jobs += jobs(p[0], float(p[1]), float(p[2]), p[3], int(p[4]) if len(p) > 4 else 0)
    path = os.path.join(HERE, 'syn.jsonl')
    done = set()
    if os.path.exists(path):
        done = {json.loads(l)['id'] for l in open(path)}
    todo = [j for j in all_jobs if j['id'] not in done]
    print('%d jobs, %d to run' % (len(all_jobs), len(todo)), flush=True)
    with mp.Pool(int(os.environ.get('W', 12))) as pool, open(path, 'a') as fh:
        for i, r in enumerate(pool.imap_unordered(run_one, todo), 1):
            fh.write(json.dumps(r, ensure_ascii=False) + '\n'); fh.flush()
            if i % 20 == 0:
                print('  %d / %d' % (i, len(todo)), flush=True)
    print('done', flush=True)
