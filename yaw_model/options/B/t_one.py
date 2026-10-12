import sys, time, json
sys.dont_write_bytecode = True
import ffcommon as fc
t = sys.argv[1]; P, I, FF = map(float, sys.argv[2:5])
t0 = time.time()
res, loop = fc.run_gate(t, P, I, FF, noise=False)
print('%.1f s' % (time.time() - t0), res.status)
for l in res.gate_lines: print(l)
m = fc.metrics(res, t)
for k, v in m.items(): print(k, {a: (round(b, 3) if isinstance(b, float) else b) for a, b in v.items()})
