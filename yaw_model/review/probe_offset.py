import math, sys, os
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from probe_munk import run, inertia, munk_fit
import gz_model

for vehicle in ('cloudship', 'airship_vectored_independent'):
    fam = gz_model.family(vehicle)
    mass, com = gz_model.centre_of_mass(fam)
    izz = gz_model.yaw_inertia(fam)
    yaw_torque = 1.0 / 5.7986 * izz
    push = (6.0 / 7.3148 * mass, 0.5 / 7.3148 * mass, 0.0)
    for off in (0.0, 0.2, 0.4, 0.6, 0.8, 1.0):
        I = {}
        for m in (vehicle, vehicle + '_am'):
            rows, _ = run(m, torque_z=yaw_torque, seconds=off + 2.0)
            I[m] = inertia([r for r in rows if r[0] > off], yaw_torque)
        rows, _ = run(vehicle + '_am', force_world=push, seconds=off + 2.5)
        k = munk_fit([r for r in rows if r[0] > off]) * I[vehicle + '_am']
        print('%-30s offset %.1f  I_rigid %.3f  I_am %.3f  K %.4f' % (vehicle, off, I[vehicle], I[vehicle + '_am'], k))
