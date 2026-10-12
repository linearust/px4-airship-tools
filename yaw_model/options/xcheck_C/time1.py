import sys, os, time
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'C'))
import optc
t0 = time.time()
res, m = optc.run_gate('gz_airship_vectored_independent', P=0.7, I=0.2, b=0.9, seed=1)
print('elapsed', time.time() - t0, res.status, len(res.samples))
for k, v in m.items():
    print(k, {x: (round(y, 3) if isinstance(y, float) else y) for x, y in v.items()})
