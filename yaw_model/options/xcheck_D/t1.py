import os, sys, time
sys.dont_write_bytecode = True
M = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, M)
import closed_loop as cl, gate
t0 = time.time()
loop = cl.build('gz_airship_vectored_independent_am_fin_drag', params={'AS_YAWRATE_P': 0.7}, noise=True, seed=1)
res = gate.run_gate(loop, 'airship_vectored_independent_am_fin_drag', out_dir=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tmp_out'))
print('%.1f s wall, sim %.1f s' % (time.time() - t0, loop.sim))
for l in res.gate_lines: print(l)
print(res.samples[0].keys(), len(res.samples))
print(sorted(set(r['phase'] for r in res.samples)))
