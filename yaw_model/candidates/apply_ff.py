"""Apply option B (yaw feedforward) to the tree, uncommitted: P 0.65, I 0.07, AS_YAWRATE_FF (default 0),
FF 1.0 / 0.3 / 0.05 on the posix 2527 / 2522 / 2517 airframes. Run from PX4-Autopilot."""
import io, re
def patch(p, pairs):
    s = io.open(p, encoding='utf-8').read()
    for o, n in pairs:
        assert s.count(o) == 1, (p, o[:60]); s = s.replace(o, n)
    io.open(p, 'w', encoding='utf-8').write(s)
Y = 'src/modules/airship_att_control/airship_att_control_params.yaml'
s = io.open(Y, encoding='utf-8').read()
i, j = s.index('    AS_YAWRATE_P:'), s.index('    AS_YAWRATE_I:')
blk = s[i:j]; assert blk.count('default: 0.7') == 1 or blk.count('default: 0.5') == 1, blk
blk = re.sub(r'default: 0\.[57]\n', 'default: 0.65\n', blk, count=1)
s = s[:i] + blk + s[j:]
i, j = s.index('    AS_YAWRATE_I:'), s.index('    AS_YR_INT_LIM:')
blk = s[i:j]; assert blk.count('default: 0.09') == 1; s = s[:i] + blk.replace('default: 0.09', 'default: 0.07') + s[j:]
FF = """    AS_YAWRATE_FF:
      description:
        short: Yaw rate feedforward
        long: Normalized yaw torque per rad/s of commanded yaw rate, added to the
          rate controller output. It supplies the torque that holds a commanded
          rate against the hull's yaw damping, so the integrator holds less and
          the vehicle stops sooner when the stick is released. Set it to the
          torque that holds a steady turn divided by that turn rate (both
          measured, torque in the unit of the yaw torque setpoint). Hull
          damping grows with the square of the rate, so the value is right at
          one rate only. Too large overshoots and reverses on release at
          smaller rates, too small leaves the rest to the integrator. 0 on a
          vehicle without aerodynamic yaw damping. Unused where the yaw rate
          loop stays open (see AS_YAWRATE_MAX).
      type: float
      default: 0.0
      min: 0.0
      max: 3.0
      decimal: 3
      increment: 0.01
"""
k = s.index('    AS_YAWRATE_MAX:'); s = s[:k] + FF + s[k:]
io.open(Y, 'w', encoding='utf-8').write(s)
patch('src/modules/airship_att_control/airship_att_control.hpp', [(
"\t\t(ParamFloat<px4::params::AS_YAWRATE_I>) _param_as_yawrate_i,\n",
"\t\t(ParamFloat<px4::params::AS_YAWRATE_I>) _param_as_yawrate_i,\n\t\t(ParamFloat<px4::params::AS_YAWRATE_FF>) _param_as_yawrate_ff,\n")])
patch('src/modules/airship_att_control/airship_att_control_main.cpp', [(
"\t_rate_control.setIntegratorLimit(Vector3f(0.f, 0.f, _param_as_yr_int_lim.get()));\n",
"\t_rate_control.setIntegratorLimit(Vector3f(0.f, 0.f, _param_as_yr_int_lim.get()));\n\t// torque += FF x rate setpoint: the torque a steady turn needs against the hull's damping\n\t_rate_control.setFeedForwardGain(Vector3f(0.f, 0.f, _param_as_yawrate_ff.get()));\n")])
for n, v in (('2527_gz_airship_vectored_independent_am_fin_drag', '1.0'), ('2522_gz_airship_vectored_independent_fin', '0.3'), ('2517_gz_cloudship_am_fin_drag', '0.05')):
    p = 'ROMFS/px4fmu_common/init.d-posix/airframes/' + n
    s = io.open(p, encoding='utf-8').read()
    src = [l for l in s.splitlines() if l.startswith('. ${R}etc/init.d-posix/airframes/')]
    assert len(src) == 1, p
    s = s.replace(src[0] + '\n', src[0] + '\n\nparam set-default AS_YAWRATE_FF ' + v + '\n')
    io.open(p, 'w', encoding='utf-8').write(s)
print('FF applied')
