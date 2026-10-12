#!/usr/bin/env python3
"""2026-10-12 D-f check (professor_decisions_20261012.md): the hover gate's Acro step and release
replayed for any vehicle-file family, with the yaw model's actuation path (allocator, AirshipPod
steer/slew, gz tilt joints and rotors: vectored_actuation.VectoredActuation) read
from the family's own bare airframe and model, and the restore rule where the
firmware restores the integrator after every stick input. Plant: (I + rr) r_dot = N_pods - c_hull r|r| - c_fin r|r|."""
import math, os, sys
TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, TOOLS); sys.path.insert(0, os.path.join(TOOLS, 'yaw_model'))
import gz_model, airship_gains  # noqa
import controller as ctl  # noqa
import vectored_actuation as va  # noqa
import gate  # noqa

RHO = 1.225
STICK, DZ = 0.5, 0.1

def bare_airframe(vehicle):
    for f, path in gz_model.airframe_files():
        if f.endswith('_gz_' + gz_model.bare(vehicle)):
            return path
    raise LookupError(vehicle)

class Plant:
    def __init__(self, model, member):
        fam = gz_model.family(model)
        self.I = gz_model.yaw_inertia(fam) + gz_model.added_mass(gz_model.merged(model)).get('rr', 0.0)
        hydro = gz_model.hydrodynamics(gz_model.merged(model)) if 'drag' in member else {}
        self.c_hull = abs(hydro.get('nRabsR', 0.0))
        fins = gz_model.vertical_fins(model) if 'fin' in member else []
        self.c_fin = max([0.5 * RHO * f['area'] * 1.2 * abs(f['arm']) ** 3 for f in fins] or [0.0])
        self.rate = 0.0
    def step(self, dt, moment):
        self.rate += (moment - (self.c_hull + self.c_fin) * self.rate * abs(self.rate)) / self.I * dt

class Act:
    def __init__(self, vehicle, model):
        self.act = va.VectoredActuation(va.load_params(model, check=False, airframe=bare_airframe(vehicle)))
    def step(self, dt, u):
        row = self.act.step(u, (0.0, 0.0, 0.0), 0.0)
        return row['N'], self.act.alloc.unallocated[2]
    def status(self):
        pods = self.act.alloc.pods
        return '%.0f/%.0f' % (math.degrees(pods[0].tilt), math.degrees(pods[1].tilt))

def run(vehicle, member, quadratic, push_s=35.0, release_s=12.0, wind=0.0, i_off=False):
    model = vehicle + member
    params = ctl.head_params(model)
    number = ctl.airframe_for_model(model)
    defaults = gz_model.airframe_defaults(model)
    # the feedforward law is the controller module's switch: 'quadratic' is the
    # firmware since 2026-10-12, 'linear' the one before
    ctl.FF_LAW = 'quadratic' if quadratic else 'linear'
    c = ctl.GateController('vectored', params, filter_gyro=False, other_params={'THR_MDL_FAC': params['THR_MDL_FAC']})
    c.ctrl.set_restore('zero_or_sign')      # the firmware restores after every stick input (2026-10-12)
    if i_off:
        c.set_param('AS_YAWRATE_I', 0.0)
    loop = gate.compose(c, Act(vehicle, model), Plant(model, member))
    loop.arm(); loop.set_mode('acro'); loop.set_stick(0.0); loop.set_wind(wind)
    loop.advance(2.0)
    r_max = math.radians(params['AS_YAWRATE_MAX'])
    sp_deg = math.degrees((STICK - DZ) / (1.0 - DZ) * r_max) if wind == 0.0 else 0.0
    loop.set_stick(STICK if wind == 0.0 else 0.0)
    rows = []
    t = 0.0
    while t < push_s - 1e-9:
        loop.advance(0.1); t += 0.1
        rows.append((t, math.degrees(loop.read_rate()), loop.read_integ(), loop.read_torque()))
    push = rows
    loop.set_stick(0.0)
    rel, t = [], 0.0
    while t < release_s - 1e-9:
        loop.advance(0.1); t += 0.1
        rel.append((t, math.degrees(loop.read_rate()), loop.read_integ(), loop.read_torque()))
    o = {'ff': params['AS_YAWRATE_FF'], 'sp': sp_deg}
    if wind == 0.0:
        last3 = [x[1] for x in push if x[0] >= push[-1][0] - 3.0]
        o['mean3'] = sum(last3) / len(last3); o['peak'] = max(x[1] for x in push); o['integ_end'] = push[-1][2] or 0.0
        o['track_ok'] = abs(o['mean3'] - sp_deg) < 0.25 * sp_deg and o['peak'] < 1.4 * sp_deg
        start = settle = None
        for tt, rd, _, _ in rel:
            if abs(rd) < 1.0:
                start = tt if start is None else start
                if tt - start >= 1.5 - 1e-9:
                    settle = start - rel[0][0] + 0.1; break
            else:
                start = None
        o['settle'] = settle; o['rel_min'] = min(x[1] for x in rel)
        o['release_ok'] = abs(rel[0][1]) > 0.5 * sp_deg and settle is not None and settle < 10.0
        o['rate_end'] = None
    else:
        last2 = [x[1] for x in push if x[0] >= push[-1][0] - 2.0]
        o['rate_end'] = abs(sum(last2) / len(last2))
    return o

def main():
    vehicles = sys.argv[1:] or ['airship_jaxa', 'airship_vectored_independent', 'airship_indoor', 'airship_aurora', 'airship_mpi']
    print('%-30s %-10s %6s %6s %7s %7s %7s %6s %5s %5s' % ('model', 'FF form', 'FF', 'mean3', 'peak', 'integ', 'settle', 'relmin', 'trk', 'rel'))
    for v in vehicles:
        for member in ('_am_fin_drag', '_fin'):
            for quad in (False, True):
                o = run(v, member, quad)
                print('%-30s %-10s %6.3f %6.2f %7.2f %7.4f %7s %6.2f %5s %5s' % (
                    v + member, 'quadratic' if quad else 'linear', o['ff'], o['mean3'], o['peak'], o['integ_end'],
                    'none' if o['settle'] is None else '%.1f' % o['settle'], o['rel_min'],
                    'ok' if o['track_ok'] else 'FAIL', 'ok' if o['release_ok'] else 'FAIL'))
        N = gz_model.yaw_authority(v + '_am_fin_drag')
        o = run(v, '_am_fin_drag', False, push_s=25.0, release_s=0.1, wind=N / 54.0, i_off=True)
        print('  %s wind %.3f N m, I off: |rate| %.2f deg/s (old threshold > 1.0)' % (v, N / 54.0, o['rate_end']))

if __name__ == '__main__':
    main()
