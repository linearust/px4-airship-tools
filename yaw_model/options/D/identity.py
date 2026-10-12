#!/usr/bin/env python3
"""OptionD(rule none) must reproduce the shared controller exactly: same gate
lines and the same sample rows (rate, torque, integ) on all five plants,
noise-free and with noise seed 3."""
import multiprocessing as mp
import os
import sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True
import option_d as od  # noqa: E402


def one(job):
    target, noise = job
    import closed_loop as cl, gate
    out = os.path.join(HERE, 'tmp_out', str(os.getpid()))
    os.makedirs(out, exist_ok=True)
    kw = {'noise': True, 'seed': 3} if noise else {}
    a = gate.run_gate(cl.build(target, **kw), od.model_of(target), out_dir=out)
    b = gate.run_gate(od.build(target, {'kind': 'none'}, noise=noise, seed=3), od.model_of(target), out_dir=out)
    same_lines = a.gate_lines == b.gate_lines
    rows = [repr((r['rate'], r['torque'], r['integ'])) for r in a.samples]
    rows_b = [repr((r['rate'], r['torque'], r['integ'])) for r in b.samples]
    return target, noise, same_lines, rows == rows_b, len(rows), a.gate_lines[-1].strip()


if __name__ == '__main__':
    jobs = [(t, n) for t in od.TARGETS5 for n in (False, True)]
    with mp.Pool(10) as pool:
        for t, n, sl, sr, nrows, last in pool.imap(one, jobs):
            print('%-45s noise %-5s lines %s rows %s (%d rows)  %s' % (t, n, 'SAME' if sl else 'DIFF',
                                                                   'SAME' if sr else 'DIFF', nrows, last))
