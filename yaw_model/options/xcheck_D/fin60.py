import os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.dont_write_bytecode = True
import xlib, gate
t = 'gz_airship_vectored_independent_fin'
for P in (0.7, 0.5):
    loop = xlib.build(t, 'restore', P, seed=1, variant=('alloc_slew', 60.0))
    res = gate.run_gate(loop, xlib.spec(t)[0], out_dir=os.path.join(HERE, 'tmp_out', 'fin60'))
    push = [r for r in res.samples if r['phase'] in ('출발', 'Acro 추종')]
    print('P', P, 'status', res.status, [l.strip()[:90] for l in res.gate_lines if 'Acro:' in l])
    print('  Acro push rate every 1 s:', ' '.join('%.1f' % r['rate'] for r in push[::3]))
    print('  torque:', ' '.join('%.2f' % r['torque'] for r in push[::3]))
    st = [r for r in res.samples if r['phase'] in ('Stabilized 추종',)]
    print('  Stab push rate:', ' '.join('%.1f' % r['rate'] for r in st[::3]))
