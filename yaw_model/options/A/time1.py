import os, sys, time
sys.dont_write_bytecode = True
M = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, M)
import closed_loop as cl, gate, reproduce as rp
for t in ['gz_airship_vectored_independent', 'gz_cloudship_am_fin_drag']:
    t0 = time.time()
    loop = cl.build(t, noise=True, seed=1, params=rp.record_params(t)[0])  # the gains the chain7/chain9 records were flown with (record_gains)
    od = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tmp_out'); os.makedirs(od, exist_ok=True)
    res = gate.run_gate(loop, cl.TARGETS[t][0], out_dir=od)
    print(t, time.time()-t0, 's', res.status, len(res.samples))
    for l in res.gate_lines: print(l)
