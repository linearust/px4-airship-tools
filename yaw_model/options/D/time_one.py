import os, sys, time
sys.dont_write_bytecode = True
M = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, M)
import closed_loop as cl, gate, reproduce as rp
t0 = time.time()
loop = cl.build('gz_airship_vectored_independent_am_fin_drag', noise=True, seed=1)
res = gate.run_gate(loop, 'airship_vectored_independent_am_fin_drag', out_dir='/tmp/claude-1000/-home-h-dev-PX4-Autopilot/1e48cf6c-455a-42ab-ba26-91399c708d3c/scratchpad/yaw_model/options/D/tmp_out')
print('%.1f s wall, sim %.1f s' % (time.time() - t0, loop.sim))
for l in res.gate_lines: print(l)
