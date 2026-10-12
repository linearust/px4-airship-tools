#!/usr/bin/env python3
"""Compare a recorded rigid vectored gate time series with the actuation model.

    python3 vectored_compare_samples.py rec_samples_gz_airship_vectored_independent_chain9.csv

The CSV is a copy of (bench) sitl_out/samples_gz_airship_vectored_independent_chain9.csv
(chain9, 2026-09-27 17:42). Columns (bench sitl_check.py:97-100): t [wall s],
phase, rate [deg/s], torque [= u, vehicle_torque_setpoint z], integ.

Rigid plant: Izz 129.2 kg m^2 and no yaw damping (the gate's print, chain9_20260927.txt,
"요 관성 129.2 kg m^2 와 요 감쇠 0"). Wall time is mapped to sim time by
RTF = 2.11 / 2.22 = 0.950 from the same run's wind check (1 N m for a nominal
5 s moved the rate 0.01 -> -2.11 deg/s, 2.22 predicted); RTF 1 is shown too.

1. Manual (u = 0.5 stick passthrough, pods at 0): fit the stick's arrival
   delay d for three actuation hypotheses - this model (pod sweep), an ideal
   54u from the stick, and one pod only (27u) - and report the RMS error.
2. Acro brake: closed loop (vectored_gate_replay.Sim) from the first sample.
3. Acro / Stabilized push and release: closed loop from each phase's
   recorded start rate and integrator, pods where the model's own run of the
   gate sequence leaves them (starboard 180, port 0; vectored_gate_replay.py).
"""
import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vectored_actuation import load_params, prepared  # noqa: E402
from vectored_gate_replay import Sim  # noqa: E402

IZZ = 129.2
RTF = 2.11 / 2.22


def load(path):
    with open(path) as handle:
        rows = list(csv.DictReader(handle))
    for row in rows:
        for key in ('t', 'rate', 'torque', 'integ'):
            row[key] = float(row[key])
    return rows


def phase(rows, name):
    return [r for r in rows if r['phase'] == name]


def manual_curve(p, kind, seconds, u=0.5):
    """Rate change [deg/s] vs sim time from the stick's arrival."""
    dt = p['dt']
    out, r = [0.0], 0.0
    if kind == 'model':
        act = prepared(p)
        for _ in range(int(seconds / dt)):
            r += act.step(u)['N'] / IZZ * dt
            out.append(math.degrees(r))
    else:
        N = 54.0 * u if kind == 'ideal 54u' else 27.0 * u
        for _ in range(int(seconds / dt)):
            r += N / IZZ * dt
            out.append(math.degrees(r))
    return out


def main():
    rows = load(sys.argv[1])
    p = load_params()
    dt = p['dt']

    man = phase(rows, 'Manual 통과')
    t0, r0 = man[0]['t'], man[0]['rate']
    print('1. Manual u=0.5 from pods at 0, %d samples; model time = RTF x wall - d' % len(man))
    fits = {}
    for rtf in (RTF, 1.0):
        for kind in ('model', 'ideal 54u', 'one pod 27u'):
            curve = manual_curve(p, kind, 4.0)
            best = None
            for k in range(0, 101):
                d = k * 0.005
                err = []
                for s in man:
                    tau = rtf * (s['t'] - t0) - d
                    pred = r0 + (curve[int(round(tau / dt))] if tau > 0 else 0.0)
                    err.append(s['rate'] - pred)
                rms = math.sqrt(sum(e * e for e in err) / len(err))
                if best is None or rms < best[0]:
                    best = (rms, d, err)
            fits[(rtf, kind)] = best
            print('   RTF %.3f  %-12s best d %.3f s  RMS %.2f deg/s  max |err| %.2f'
                  % (rtf, kind, best[1], best[0], max(abs(e) for e in best[2])))
    rms, d, err = fits[(RTF, 'model')]
    curve = manual_curve(p, 'model', 4.0)
    print('   at RTF %.3f, d %.3f s:  wall t   recorded   model   err' % (RTF, d))
    for s, e in zip(man, err):
        tau = RTF * (s['t'] - t0) - d
        print('      %5.2f  %+7.2f  %+7.2f  %+5.2f' % (s['t'] - t0, s['rate'], s['rate'] - e, e))

    print('\n2. Acro brake, closed loop from the first sample (loop active at once):')
    brake = phase(rows, 'Acro 제동')
    for rtf in (RTF, 1.0):
        sim = Sim(p)
        sim.act = prepared(p, 0.5, 3.0, 0.3)       # pods left pointed for + by Manual
        sim.r = math.radians(brake[0]['rate'])
        closed = sim.hold('acro', 0.0, (brake[-1]['t'] - brake[0]['t']) * rtf + 0.1)
        err = []
        lines = []
        for s in brake:
            c = closed[min(int(round(rtf * (s['t'] - brake[0]['t']) / dt)), len(closed) - 1)]
            err.append(s['rate'] - math.degrees(c[4]))
            lines.append('      %5.2f  %+7.2f  %+7.2f  %+.3f  %+.3f'
                         % (s['t'] - brake[0]['t'], s['rate'], math.degrees(c[4]), s['torque'], c[3]))
        print('   RTF %.3f: RMS %.2f deg/s, end recorded %+.2f model %+.2f'
              % (rtf, math.sqrt(sum(e * e for e in err) / len(err)), brake[-1]['rate'], math.degrees(closed[-1][4])))
        if rtf == RTF:
            print('      wall t  recorded  model   u_rec   u_model')
            print('\n'.join(lines[::3]))

    print('\n3. Push (stick 0.5, sp 6.67 deg/s) and release, closed loop from the recorded start:')
    for label, mode in (('Acro', 'acro'), ('Stabilized', 'stabilized')):
        start = phase(rows, '출발')
        push = phase(rows, '%s 추종' % label)
        rel = phase(rows, '%s 놓음' % label)
        begin = [s for s in start if s['t'] < push[0]['t']][-1]
        sim = Sim(p)
        sim.act = prepared(p, 0.05, 3.0, 1.0)      # starboard 180, port 0, rotors stopped
        sim.r = math.radians(begin['rate'])
        sim.I = begin['integ']
        span = (rel[-1]['t'] - push[0]['t']) * RTF + 0.2
        log_push = sim.hold(mode, 0.5, (rel[0]['t'] - push[0]['t']) * RTF)
        log_rel = sim.hold(mode, 0.0, span - (rel[0]['t'] - push[0]['t']) * RTF)
        log = log_push + log_rel
        err = []
        for s in push + rel:
            c = log[min(int(round(RTF * (s['t'] - push[0]['t']) / dt)), len(log) - 1)]
            err.append(s['rate'] - math.degrees(c[4]))
        peak_rec = max(s['rate'] for s in push)
        peak_mod = max(math.degrees(c[4]) for c in log_push)
        last3 = [math.degrees(c[4]) for c in log_push if c[0] >= log_push[-1][0] - 3.0]
        last3_rec = [s['rate'] for s in push if s['t'] >= push[-1]['t'] - 3.0]
        print('   %-10s start %+.2f deg/s I %+.4f | peak rec %.2f model %.2f | last-3 s mean rec %.2f model %.2f '
              '| RMS over push+release %.2f deg/s, max |err| %.2f'
              % (label, begin['rate'], begin['integ'], peak_rec, peak_mod, sum(last3_rec) / len(last3_rec),
                 sum(last3) / len(last3), math.sqrt(sum(e * e for e in err) / len(err)), max(abs(e) for e in err)))


if __name__ == '__main__':
    main()
