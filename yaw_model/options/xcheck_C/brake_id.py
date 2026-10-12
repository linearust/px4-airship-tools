"""Is the pre-step part of the gate (settle, wind check, Manual, Acro brake)
identical across b on every plant? Compares the gate rows phase by phase, and
the controller's r_sp over the brake window."""
import sys, os, math
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import xrun, optc, gate
od = os.path.join(xrun.HERE, 'tmp_out', 'bid'); os.makedirs(od, exist_ok=True)
for t in optc.TARGETS:
    res = {}
    sps = {}
    for b in (1.0, 0.9):
        loop = xrun.build_mine(t, 0.7, 0.2, b, 1)
        seen = []
        ctrl = loop.controller.ctrl
        orig = loop.advance
        def adv(dt, orig=orig, seen=seen, loop=loop):
            orig(dt)
            seen.append((loop.sim, loop._mode, ctrl.rate_sp, loop._stick))
        loop.advance = adv
        res[b] = gate.run_gate(loop, optc.model_name(t), out_dir=od)
        sps[b] = seen
    by = {}
    for b in res:
        for r in res[b].samples:
            by.setdefault(b, {}).setdefault(r['phase'], []).append(r)
    out = []
    for ph in list(by[1.0])[:6]:
        a, c = by[1.0][ph], by[0.9].get(ph, [])
        n = min(len(a), len(c))
        d = max((abs(x['rate'] - y['rate']) for x, y in zip(a[:n], c[:n])), default=float('nan'))
        out.append('%s %d/%d %.5f' % (ph, len(a), len(c), d))
    # first time r_sp != 0 in the b=1 run and mode then
    nz = next(((s, m, sp, st) for s, m, sp, st in sps[1.0] if sp != 0.0), None)
    print(optc.SHORT[t], '|', ' ; '.join(out))
    print('   first nonzero r_sp (sim, mode, r_sp, stick):', nz)
