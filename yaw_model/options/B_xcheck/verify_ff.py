#!/usr/bin/env python3
"""Does rc.ff act as RateControl's _gain_ff would? Check on a trace that every
active cycle's torque = P (sp - meas) + integ_before + FF sp, and that the
gate's AS_YAWRATE_I 0 / back does not clear FF. Also check the actuator hooks
take effect (slew: tilt rate seen in the allocator; tail lag: force lags)."""
import math
import sys

sys.dont_write_bytecode = True
import xcommon as xc  # noqa: E402
import controller as ctl  # noqa: E402

for target, FF in ((xc.VC, 1.0), (xc.CC, 0.05)):
    loop = xc.make(target, 0.65, 0.07, FF, noise=True, seed=3)
    c = loop.controller.ctrl
    loop.set_stick(0.0); loop.arm(); loop.set_mode('manual'); loop.advance(1.0)
    loop.set_mode('acro'); loop.advance(1.0)
    worst = 0.0
    n = 0
    for k, stick in enumerate([0.5] * 2500 + [0.0] * 500 + [0.3] * 500):
        if k == 1000:
            loop.set_param('AS_YAWRATE_I', 0.0)
        if k == 1500:
            loop.set_param('AS_YAWRATE_I', 0.07)
        loop.set_stick(stick)
        integ_before = c.rc.integ
        meas = loop._measured()
        loop.advance(0.004)
        sp = ctl.yaw_rate_setpoint(stick, 0.1, math.radians(15.0))
        expect = c.rc.p * (sp - meas) + integ_before + FF * sp
        worst = max(worst, abs(loop.read_torque() - expect))
        n += 1
    print('%-45s FF %.2f  rc.ff after run %.3f  max |torque - (P e + I + FF sp)| over %d cycles = %.2e'
          % (target, FF, c.rc.ff, n, worst))

# slew hook: the allocator's tilt moves at most slew * dt per tick
for slew in (60.0, 240.0):
    loop = xc.make(xc.VC, 0.65, 0.07, 1.0, noise=False, slew=slew)
    loop.set_stick(0.0); loop.arm(); loop.set_mode('acro')
    loop.set_stick(-0.5)
    tilts = []
    for _ in range(1500):
        loop.advance(0.004)
        tilts.append(loop.actuation.act.alloc.pods[0].tilt)
    d = max(abs(b - a) for a, b in zip(tilts, tilts[1:])) / 0.004
    print('slew %5.0f: max allocator tilt rate %.1f deg/s' % (slew, math.degrees(d)))

loop = xc.make(xc.VC, 0.65, 0.07, 1.0, noise=False, phys_slew=60.0)
loop.set_stick(0.0); loop.arm(); loop.set_mode('acro'); loop.set_stick(-0.5)
th = []
for _ in range(1500):
    loop.advance(0.004)
    th.append(loop.actuation.act.joints[0].theta)
d = max(abs(b - a) for a, b in zip(th, th[1:])) / 0.004
print('phys_slew 60: max gz joint rate %.1f deg/s (allocator believes 120)' % math.degrees(d))

for tau in (0.025, 0.05):
    loop = xc.make(xc.CC, 0.65, 0.07, 0.05, noise=False, tail_tau=tau)
    loop.set_stick(0.0); loop.arm(); loop.set_mode('manual'); loop.set_stick(0.5)
    Fs = []
    for _ in range(50):
        loop.advance(0.004)
        Fs.append(loop.actuation.F)
    Ffinal = loop.actuation.inner.chain.stages(0.5)['F']
    i63 = next(i for i, f in enumerate(Fs) if f >= 0.632 * Ffinal)
    print('tail tau %.3f: 63%% of the step force after %.3f s' % (tau, (i63 + 1) * 0.004))
