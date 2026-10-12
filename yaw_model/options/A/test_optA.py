import sys, os, json, multiprocessing as mp
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import optA
jobs = [(t, {}, 'nominal', 1, None) for t in optA.TARGETS]
jobs += [('gz_airship_vectored_independent', {}, 'nominal', 1, ('slew', 60)),
         ('gz_airship_vectored_independent', {}, 'nominal', 1, ('servo', 60)),
         ('gz_cloudship_am_fin_drag', {}, 'nominal', 1, ('tail_tau', 0.05)),
         ('gz_cloudship', {}, 'nominal', 1, ('noise', 10))]
with mp.Pool(9) as pool:
    for r in pool.imap(optA.gate_job, jobs):
        print(optA.SHORT[r['target']], r['act'], 'status', r['status'], 'integ_max %.3f' % r['integ_max'])
        for k, (ok, m) in optA.norm_margins(r).items():
            print('   %-13s %s  norm %s  raw %s' % (k, 'P' if ok else 'F', None if m is None else round(m, 2),
                  {f: (None if v is None else round(v, 3)) for f, v in r['margins'].get(k, {}).items()}))
