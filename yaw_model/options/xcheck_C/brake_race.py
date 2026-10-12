import sys, os
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import xrun, optc, gate
t = 'gz_cloudship'
loop = xrun.build_mine(t, 0.7, 0.2, 0.9, 1)
ctrl = loop.controller.ctrl
seen = []
orig = loop.advance
def adv(dt):
    orig(dt)
    seen.append((round(loop.sim, 3), loop._mode, ctrl.rate_sp, loop._stick))
loop.advance = adv
res = gate.run_gate(loop, optc.model_name(t), out_dir=os.path.join(xrun.HERE, 'tmp_out', 'race'))
ev = [e for e in res.events if 36 < e[1] < 60]
for e in ev: print('event', e)
nz = [s for s in seen if s[2] != 0.0 and s[0] < 60]
print('ticks with r_sp != 0 before sim 60:', len(nz), nz[:3], nz[-1:] if nz else None)
brake = [r for r in res.samples if r['phase'] == 'Acro 제동']
print('brake rows sim', brake[0].get('sim'), '..', brake[-1].get('sim'))
