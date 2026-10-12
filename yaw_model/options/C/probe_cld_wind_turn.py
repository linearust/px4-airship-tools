"""Cloudship complete: why a 0.415 N m wind (5 x the gate's) while turning at
6.67 deg/s drives the integrator to AS_YR_INT_LIM and the turn collapses,
when the same wind at rest is held with integ 0.078. Trace the hull state."""
import math, os, sys
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import optc
from beyond import Drive
for b in (1.0,):
    loop = optc.build('gz_cloudship_am_fin_drag', 0.7, 0.09, b, noise=False)
    d = Drive(loop)
    d.run(40.0, stick=0.5)
    print('t_after_wind   rate  torque  integ      u      v    |V|  munk_frd  fins  psi')
    for k in range(31):
        if k:
            d.run(2.0, wind_frd=-0.4146)
        e = loop.plant.extra()
        print('%5.0f %8.2f %7.3f %6.3f %6.3f %6.3f %6.3f %8.4f %5d %6.0f' % (
            2 * k, math.degrees(loop.read_rate()), loop.read_torque(), loop.controller.ctrl.integ,
            e['u'], e['v'], math.hypot(e['Vx'], e['Vy']), e['munk_frd'], e['fins_open'], e['psi_deg'] % 360))
