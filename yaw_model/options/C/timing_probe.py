import os, sys, time
sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))
import closed_loop as cl, gate, reproduce as rp
for t in ['gz_cloudship_am_fin_drag', 'gz_airship_vectored_independent']:
    t0 = time.time()
    loop = cl.build(t, noise=True, seed=1)
    od = os.path.join(HERE, 'tmp_out', 'probe'); os.makedirs(od, exist_ok=True)
    res = gate.run_gate(loop, cl.TARGETS[t][0], out_dir=od)
    print(t, 'status', res.status, '%.1f s wall, sim %.1f s' % (time.time() - t0, loop.sim))
    for k, v in rp.parse(res.gate_lines).items():
        print('  ', k, {f: x for f, x in v.items() if f != 'line'})
