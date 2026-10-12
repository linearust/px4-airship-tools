import time, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import syn_run as s
t0 = time.time()
j = s.jobs('restore', 0.7, 0.09, 'lite', targets=['gz_airship_vectored_independent_am_fin_drag'])[0]
r = s.run_one(j)
print('%.1f s' % (time.time() - t0))
print({k: (v['ok'], v['margins']) for k, v in r['checks'].items()})
