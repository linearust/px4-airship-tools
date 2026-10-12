"""My b vs C's hook, same seed: sample-level agreement; b=1 of mine vs base."""
import sys, os
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import xrun, optc, gate, closed_loop as cl
od = os.path.join(xrun.HERE, 'tmp_out', 'sanity'); os.makedirs(od, exist_ok=True)
for t in ['gz_airship_vectored_independent', 'gz_cloudship_am_fin_drag']:
    for b in (1.0, 0.9, 0.694):
        la = optc.build(t, P=0.7, I=0.2, b=b, noise=True, seed=3)
        lb = xrun.build_mine(t, 0.7, 0.2, b, 3)
        ra = gate.run_gate(la, optc.model_name(t), out_dir=od)
        rb = gate.run_gate(lb, optc.model_name(t), out_dir=od)
        n = min(len(ra.samples), len(rb.samples))
        d = max(abs(x['rate'] - y['rate']) for x, y in zip(ra.samples[:n], rb.samples[:n]))
        dt = max(abs(x['torque'] - y['torque']) for x, y in zip(ra.samples[:n], rb.samples[:n]))
        va = [v[1] for v in ra.verdicts]; vb = [v[1] for v in rb.verdicts]
        print(t, 'b', b, 'rows', len(ra.samples), len(rb.samples), 'max|drate| %.2e max|dtorque| %.2e' % (d, dt), 'verdicts same', va == vb, va)
        print('   hook ff at end', la.controller.ctrl.rc.ff, 'expected', 0.7 * (b - 1))
