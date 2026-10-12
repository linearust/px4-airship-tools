#!/usr/bin/env python3
"""Offline tests of forward_yaw_check.py. No simulator, no px4:

    python3 forward_yaw_check_test.py

1. the trim against the design script's surge drag for 2527 at its two leg
   speeds, the throttle's count arithmetic, the airframe chain;
2. --plan: every plant's legs, trims and budget, without importing the px4
   link, and the refusal of any other model; leg selection;
3. a synthetic log whose every answer is known: segmentation, the table, the
   metrics, the pod and tail reversals, the steady judgement, the timeline;
   and what the model replay's interface review of 2026-10-01 added: the
   replay block's throttle count, run_leg keywords and pod modes, the
   integrator's first-sample time, the fins by name with their flow, the
   tail flips on the torque setpoint, rear_s to the phase's end, the 0.5 s
   time-window peak, t_acro_s, IMU_GYRO_CUTOFF against its yaml, the
   corrected definitions; and what the review of 2026-10-02 asked: the fins'
   flow in the body frame by the full attitude with the plugin's 0.01 m/s
   return, read only for the axes its angle formula holds for, the names on
   all four plants; run_leg refused for a failed verdict, a second stick or
   an error, the IMU_GYRO_CUTOFF verdict's gate on the exit;
4. the SITL gate logs of 2026-09-29 (Manual -> Acro, a 0.5 stick for 35 s):
   the analysis against the gate's own printed numbers and its live samples,
   two records of one flight taken by different routes;
5. the live sequence against a fake px4 on a virtual clock: the PX4_PARAM_
   and firmware preconditions, the leg numbers, the timeline on disk while
   the run is still flying.

Exit as report.summary(). A missing gate log is reported as skipped, and
counts as neither a pass nor a failure.
"""
import ast
import contextlib
import csv
import io
import json
import math
import os
import re
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

for name in [k for k in os.environ if k.startswith('PX4_PARAM_')] + ['GZ_TARGET', 'PX4_GZ_ENTITY', 'FORWARD_LEGS']:
    os.environ.pop(name, None)

import gz_model                         # noqa: E402
import forward_yaw_check as F           # noqa: E402
from report import note, summary, verdict   # noqa: E402

VEC_FIN = 'airship_vectored_independent_fin'
VEC_ALL = 'airship_vectored_independent_am_fin_drag'
CS_FIN = 'cloudship_fin'
CS_ALL = 'cloudship_am_fin_drag'
# The generic plant's numbers the fixtures below stand on, from its files, so
# a regenerated hull does not strand them: its two leg speeds (the middle of
# its ramp and 1.6 times its end), its second leg's throttle (the synthetic
# legs fly it), its airframe's yaw values
LOW, HIGH = F.leg_speeds(VEC_ALL)
PLAN_THRUST = F.vehicle_plan(VEC_ALL)['legs'][1]['throttle']
P27 = F.plan_params(VEC_ALL)
LOG_ROOT = os.path.join(gz_model.ROOT, 'build/px4_sitl_default/rootfs/fs/log')


def near(a, b, tol):
    return a is not None and b is not None and math.isfinite(a) and abs(a - b) <= tol


# ---------------------------------------------------------------------------
# 1. trim, throttle counts, airframe chain, firmware defaults
# ---------------------------------------------------------------------------

def test_trim():
    import generic_hull_design
    design = generic_hull_design.final_design(generic_hull_design.DEFAULT)

    with open(os.path.join(HERE, 'vehicles', 'airship_generic.json')) as handle:
        both_pods = 2.0 * json.load(handle)['pods']['thrust_n']

    for speed in (LOW, HIGH):
        # the design script's surge drag over the vehicle file's two pods: the
        # numbers the drag and motor blocks were written from
        want = abs(design['drag']['xUabsU']) * speed * speed / both_pods
        got = gz_model.trim_thrust(VEC_ALL, speed)
        verdict('2527 의 %.2f m/s 트림이 설계 스크립트의 |xUabsU| u^2 / %g N, %.4f 다' % (speed, both_pods, want), near(got, want, 5e-5), '%.6f' % got)

    for model in (VEC_FIN, CS_FIN):
        verdict('%s 은 항력이 없어 트림이 0 이다' % model, gz_model.trim_thrust(model, 4.0) == 0.0,
                '%g' % gz_model.trim_thrust(model, 4.0))

    # the trim is |xUabsU| u^2 over both pods' full thrust, each from the file
    c = gz_model.translational_drag(gz_model.merged(VEC_ALL))[0]
    # the motor constant is written to six figures: the file's thrust is the
    # vehicle file's to 1e-6, and the trim divides by the file's
    verdict('트림은 파일의 항력과 포드 추력에서 나온다', near(gz_model.trim_thrust(VEC_ALL, 3.0), c * 9.0 / gz_model.pod_thrust(VEC_ALL), 1e-12)
            and near(gz_model.pod_thrust(VEC_ALL), both_pods, 1e-3), 'xUabsU %.4f, 포드 %.1f N' % (c, gz_model.pod_thrust(VEC_ALL)))
    # Sticks sends int(throttle * 1000): the midpoint must land on the count
    bad = [n for n in range(1001) if int(max(0.0, min(1.0, F.quantize(n / 1000.0)[1])) * 1000) != n]
    verdict('스로틀의 중간값이 Sticks 의 절삭에서 반올림한 계수가 된다 (0..1000)', not bad, '어긋난 계수 %s' % bad[:5])
    trim = gz_model.trim_thrust(VEC_ALL, LOW)
    counts, command, cruise = F.quantize(trim)
    verdict('2527 의 %.2f m/s 트림 %.4f 는 가장 가까운 계수 %d/1000 으로 간다' % (LOW, trim, round(trim * 1000)),
            counts == round(trim * 1000) and cruise == counts / 1000.0, '%d, %.4f' % (counts, command))


def test_files():
    d22, d27 = gz_model.airframe_defaults(VEC_FIN), gz_model.airframe_defaults(VEC_ALL)
    d12, d17 = gz_model.airframe_defaults(CS_FIN), gz_model.airframe_defaults(CS_ALL)
    import airship_gains
    v = airship_gains.gains(airship_gains.plant('airship_vectored_independent'), airship_gains.REF_PLANT,
                            airship_gains.stored_measurement(airship_gains.plant('airship_vectored_independent'))[0])
    ff22, ff27 = airship_gains.rnd(v['FF_fin']), airship_gains.rnd(v['FF_drag'])
    verdict('에어프레임 사슬: 2522 는 2520 을 읽고 계산기의 핀 FF %g 를 얹는다' % ff22,
            d22.get('AS_YAWRATE_FF') == ff22 and d22.get('CA_AIRSHIP_TLMIN') == -180.0 and d22.get('CA_AIRSHIP_GRP') == 1.0,
            'FF %s, TLMIN %s, GRP %s' % (d22.get('AS_YAWRATE_FF'), d22.get('CA_AIRSHIP_TLMIN'), d22.get('CA_AIRSHIP_GRP')))
    verdict('2527 은 계산기의 FF %g, 2517 FF 0.05 와 꼬리, 2512 는 FF 를 두지 않는다' % ff27,
            d27.get('AS_YAWRATE_FF') == ff27 and d17.get('AS_YAWRATE_FF') == 0.05 and d17.get('CA_AIRSHIP_TAIL') == 1.0
            and 'AS_YAWRATE_FF' not in d12, '%s %s %s' % (d27.get('AS_YAWRATE_FF'), d17.get('AS_YAWRATE_FF'),
                                                         d12.get('AS_YAWRATE_FF')))
    p = F.plan_params(CS_FIN)
    verdict('펌웨어 기본값은 yaml 에서: P 0.7, I 0.09, INT_LIM 0.3, MAX 15, DEADZONE 0.1, 2512 의 FF 0',
            (p['AS_YAWRATE_P'], p['AS_YAWRATE_I'], p['AS_YR_INT_LIM'], p['AS_YAWRATE_MAX'], p['MAN_DEADZONE'],
             p['AS_YAWRATE_FF']) == (0.7, 0.09, 0.3, 15.0, 0.1, 0.0), str({k: p[k] for k in sorted(p)}))
    numbers = {m: gz_model.airframe(m)[0] for m in F.ALLOWED}
    known = {VEC_FIN: 2522, VEC_ALL: 2527, CS_FIN: 2512, CS_ALL: 2517}
    verdict('핀 기체마다 posix 번호가 하나이고 네 기체는 제 번호다',
            all(numbers.get(m) == n for m, n in known.items()) and len(numbers) == len(F.ALLOWED), str(numbers))
    n17, n27 = gz_model.lateral_derivatives(CS_ALL)['n_v'], gz_model.lateral_derivatives(VEC_ALL)['n_v']
    verdict('Munk 가 핀을 이기는 것은 완전한 Cloudship 뿐이다 (n_v)', n17 < 0.0 < n27, '2517 %+.3f, 2527 %+.3f' % (n17, n27))
    import generic_hull_design as ghd
    design = ghd.final_design(ghd.DEFAULT)
    # the design script's own fins and arm, a path that never reads the SDF
    want = sum(0.5 * ghd.RHO * design['fin'][n]['area'] * design['fin'][n]['cla'] * design['arm'] ** 2 for n in ('top', 'bottom'))
    verdict('수직 핀 요 감쇠가 설계 스크립트의 핀과 팔로 계산한 %.2f u 다' % want, near(gz_model.fin_yaw_damping(VEC_FIN), want, 1e-3 * want),
            '%.3f' % gz_model.fin_yaw_damping(VEC_FIN))
    # the CSV's actuator columns against the airframes' own @output words
    agree = []

    for model in F.ALLOWED:
        out, plant = F.airframe_outputs(model), F.plant_info(model, F.plan_params(model))

        if plant['independent']:
            agree.append(F.tilt_names(plant) == ['tilt_stbd_deg', 'tilt_port_deg'] and 'starboard' in out['servo1']
                         and 'port' in out['servo2'] and 'starboard' in out['motor1'] and 'port' in out['motor2']
                         and plant['n_motors'] == 2 and 'motor3' not in out)
        else:
            agree.append(F.tilt_names(plant) == ['tilt_deg'] and plant['tail'] and plant['n_motors'] == 3
                         and 'tail' in out['motor3'] and 'tilt' in out['servo1'] and 'servo2' not in out)

    verdict('CSV 의 모터, 서보 열이 에어프레임의 @output 이 말하는 것이다 (꼬리 motor3, 우현 틸트 servo1)', all(agree), str(agree))

    try:
        F.plant_info(VEC_ALL, dict(F.plan_params(VEC_ALL), CA_AIRSHIP_TAIL=None))
        refused = False
    except LookupError:
        refused = True

    verdict('로그에 배치 파라미터가 없으면 짐작하지 않고 멈춘다', refused, '')
    # the 1-DOF reach's closed form against a bisection of the same balance,
    # P (sp - r) + L + FF sp = (k u r + c_r r^2) / A, on 2527 at its high leg
    p27 = F.plan_params(VEC_ALL)
    A, k = gz_model.yaw_authority(VEC_ALL), gz_model.fin_yaw_damping(VEC_ALL)
    c_r = abs(gz_model.hydrodynamics(gz_model.merged(VEC_ALL)).get('nRabsR', 0.0))

    def bisect(ff):
        sp = math.radians(F.setpoint_deg(1.0, p27))
        f = lambda r: p27['AS_YAWRATE_P'] * (sp - r) + p27['AS_YR_INT_LIM'] + ff * sp - (k * HIGH * r + c_r * r * r) / A
        lo, hi = 0.0, 10.0

        for _ in range(200):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if f(mid) > 0.0 else (lo, mid)

        return math.degrees(min(lo, sp))

    with_ff = F.loop_reach(VEC_ALL, p27, HIGH, 1.0)['reach']
    without = F.loop_reach(VEC_ALL, dict(p27, AS_YAWRATE_FF=0.0), HIGH, 1.0)['reach']
    verdict('1자유도 추정의 닫힌 식이 같은 균형의 이분법과 맞는다 (2527, %.2f m/s 전타, FF 0 과 에어프레임의 FF)' % HIGH,
            near(without, bisect(0.0), 1e-6) and near(with_ff, bisect(p27['AS_YAWRATE_FF']), 1e-6),
            'FF 0 %.3f/%.3f, FF %g %.3f/%.3f' % (without, bisect(0.0), p27['AS_YAWRATE_FF'], with_ff, bisect(p27['AS_YAWRATE_FF'])))


def test_firmware_constants():
    """The numbers the check compares the log against, held against the files
    that define them: the modes and the arming state (VehicleStatus.msg), the
    saturation rule (airship_yaw_rate_loop.hpp), the reversal count
    (yaw_model/forward.py)."""
    verdict('모드와 무장 값은 VehicleStatus.msg 에서 읽는다: MANUAL 0, ACRO 10, ARMED 2',
            (F.NAV_MANUAL, F.NAV_ACRO, F.ARMED) == (0, 10, 2), str((F.NAV_MANUAL, F.NAV_ACRO, F.ARMED)))

    try:
        F.msg_constant('NAVIGATION_STATE_NO_SUCH_MODE')
        refused = False
    except LookupError:
        refused = True

    verdict('메시지에 없는 상수는 짐작하지 않고 멈춘다', refused, '')
    path = os.path.join(gz_model.ROOT, 'src/modules/airship_att_control/airship_yaw_rate_loop.hpp')

    with open(path) as handle:
        loop = handle.read()

    strict = (re.search(r'setPositiveSaturationFlag\(2,\s*yaw_shortfall\s*>\s*FLT_EPSILON\)', loop)
              and re.search(r'setNegativeSaturationFlag\(2,\s*yaw_shortfall\s*<\s*-FLT_EPSILON\)', loop))
    verdict('깃발 문턱은 펌웨어의 것이다: setSaturation 이 FLT_EPSILON 을 넘을 때 (엄격히), FLAG_EPS = float32 eps',
            bool(strict) and F.FLAG_EPS == float(np.finfo(np.float32).eps), 'FLAG_EPS %.6g' % F.FLAG_EPS)
    model_path = os.path.join(HERE, 'yaw_model/forward.py')

    with open(model_path) as handle:
        model = handle.read()

    found = {name: re.search(r'^%s\s*=\s*([-+0-9.eE]+)' % name, model, re.M) for name in ('FIN_BAND', 'TAIL_BAND')}
    bands = {k: float(v.group(1)) if v else None for k, v in found.items()}
    verdict('반전을 yaw_model/forward.py 와 같이 센다: cos(틸트) 띠 %g, 꼬리 띠 %g' % (F.REVERSAL_COS, F.TAIL_DEADBAND),
            bands == {'FIN_BAND': F.REVERSAL_COS, 'TAIL_BAND': F.TAIL_DEADBAND}, str(bands))


def test_added_mass_reads():
    """An _am member's added mass decides how the check flies it (accel_mode):
    a lost term must stop it, never read as 0 (no Munk moment, a stable
    Cloudship, an acceleration in Manual that the probe saw depart)."""
    d17, d12 = gz_model.lateral_derivatives(CS_ALL), gz_model.lateral_derivatives(CS_FIN)
    verdict('부가 질량이 없는 기체는 0, 있는 기체는 파일의 값 (2517 Munk 4.92)',
            d12['munk'] == 0.0 and d12['m_x'] == d12['m_y'] and near(d17['munk'], 4.923, 0.001),
            '2512 %.3f, 2517 %.3f' % (d12['munk'], d17['munk']))
    family = gz_model.family(CS_ALL)
    sources = {name: os.path.join(gz_model.model_dir(name), 'model.sdf') for name in family}
    kept = gz_model.MODEL_HOMES
    outcomes = {}

    with tempfile.TemporaryDirectory() as tmp:
        def tree(drop):
            """The family's model.sdf files with `drop` removed from the block:
            a child's tag, or 'block' for the whole of it. 1 when it was
            there to remove (the file's comments name the element too, so
            this edits the element, not the text)."""
            for name in family:
                os.makedirs(os.path.join(tmp, name), exist_ok=True)
                shutil.copy(sources[name], os.path.join(tmp, name, 'model.sdf'))

            path = os.path.join(tmp, family[0], 'model.sdf')
            doc = ET.parse(path)
            holders = [i for i in doc.getroot().iter('inertial') if i.find('fluid_added_mass') is not None]

            if len(holders) != 1:
                return 0

            block = holders[0].find('fluid_added_mass')
            target = block if drop == 'block' else block.find(drop)

            if target is None:
                return 0

            (holders[0] if drop == 'block' else block).remove(target)
            doc.write(path)
            return 1

        try:
            gz_model.MODEL_HOMES = [tmp]

            for drop, call in (('yy', lambda: gz_model.lateral_derivatives(CS_ALL)),
                               ('xx', lambda: gz_model.surge_mass(CS_ALL)),
                               ('rr', lambda: gz_model.lateral_derivatives(CS_ALL)),
                               ('block', lambda: F.accel_mode(CS_ALL, F.plan_params(CS_ALL), [2.0]))):
                if tree(drop) != 1:
                    outcomes[drop] = 'fixture'
                    continue

                try:
                    outcomes[drop] = repr(call())
                except LookupError:
                    outcomes[drop] = 'LookupError'
        finally:
            gz_model.MODEL_HOMES = kept

    verdict('_am 기체의 fluid_added_mass 에서 yy, xx, rr 이나 블록 전체가 빠지면 0 으로 읽지 않고 멈춘다 (가속 방식도)',
            all(v == 'LookupError' for v in outcomes.values()) and len(outcomes) == 4, str(outcomes))


# ---------------------------------------------------------------------------
# 2. --plan and the legs
# ---------------------------------------------------------------------------

def run_main(argv, env=None):
    kept = {k: os.environ.get(k) for k in ('GZ_TARGET', 'PX4_GZ_ENTITY', 'FORWARD_LEGS')}

    for k in kept:
        os.environ.pop(k, None)

    os.environ.update(env or {})
    out = io.StringIO()
    code = None

    try:
        with contextlib.redirect_stdout(out):
            code = F.main(argv)
    except SystemExit as stop:
        code = stop
    finally:
        for k, v in kept.items():
            os.environ.pop(k, None)
            if v is not None:
                os.environ[k] = v

    return code, out.getvalue()


def test_plan():
    before = 'sitl_link' in sys.modules
    code, text = run_main(['--plan'])
    verdict('--plan 이 px4 연결 모듈 없이 끝난다', code == 0 and not before and 'sitl_link' not in sys.modules,
            '종료 %s, sitl_link 가져옴 %s' % (code, 'sitl_link' in sys.modules))
    verdict('--plan 이 핀 기체를 모두 보인다 (%d)' % len(F.ALLOWED), all('기체 %s' % m in text for m in F.ALLOWED),
            '%d 줄' % len(text.splitlines()))
    want = ['%.4f -> %d/1000' % (gz_model.trim_thrust(VEC_ALL, u), F.quantize(gz_model.trim_thrust(VEC_ALL, u))[0]) for u in (LOW, HIGH)]
    verdict('--plan 에 2527 의 두 트림과 그 계수가 있다 (%s)' % ', '.join(want), all(w in text for w in want), '')
    verdict('완전한 Cloudship 만 Acro 에서 가속한다',
            '기체 %s (posix 2517), 가속 acro' % CS_ALL in text
            and all('기체 %s (posix %d), 가속 manual' % (m, gz_model.airframe(m)[0]) in text for m in (VEC_FIN, VEC_ALL, CS_FIN)),
            '')
    n_plants = len(F.ALLOWED)
    n_cloud = sum(m.startswith('cloudship') for m in F.ALLOWED)
    verdict('--plan 이 기체마다 시뮬레이터와 벽시계 시간, 바깥 timeout 을 낸다',
            text.count('합계: 시뮬레이터 보통') == n_plants and text.count('바깥 timeout 은 최대보다 길게') == n_plants, '')
    verdict('Cloudship 기체마다 속도 선택 표가 있다 (%d)' % n_cloud, text.count('속도 선택 표') == n_cloud, '')
    code, text = run_main(['--plan'], {'GZ_TARGET': 'gz_cloudship_fin'})
    verdict('GZ_TARGET 이 있으면 그 기체만', code == 0 and text.count('기체 ') == 1 and CS_FIN in text, '')
    code, text = run_main(['--plan'], {'GZ_TARGET': 'gz_airship_vectored_independent'})
    verdict('다른 기체는 px4 를 묻기 전에 거절한다', isinstance(code, SystemExit) and 'airship_vectored_independent' in str(code),
            str(code)[:80])
    code, text = run_main(['--plan'], {'GZ_TARGET': 'gz_cloudship_am_fin_drag', 'FORWARD_LEGS': '4.0:1.0'})
    verdict('4 m/s 의 완전한 Cloudship 다리는 진입이 발산한다고 표시된다', '진입이 정상 직진이 되지 못할 것으로 본다' in text, '')
    # a world-suffixed target names the same model
    code, text = run_main(['--plan'], {'GZ_TARGET': 'gz_cloudship_fin_lta'})
    verdict('월드 접미사가 붙은 대상도 같은 기체다', code == 0 and CS_FIN in text, '')
    # a rerun is named as the leg it reruns, so the label is what FORWARD_LEGS takes
    code, text = run_main(['--plan'], {'GZ_TARGET': 'gz_airship_vectored_independent_fin', 'FORWARD_LEGS': '3'})
    verdict('FORWARD_LEGS=3 의 다리는 다리 3 이다 (기록의 이름이 다시 날 번호다)',
            code == 0 and '다리 3: %.2f m/s, 요 스틱 +0.50' % HIGH in text and '다리 1' not in text, '')
    code, text = run_main(['--plan'], {'GZ_TARGET': 'gz_airship_vectored_independent_fin',
                                       'FORWARD_LEGS': '3.0:1.0,%g:0.5' % HIGH})
    verdict('기본이 아닌 다리는 4 부터 다시 날 속도:스틱과, 기본과 같은 속도:스틱은 그 번호로',
            code == 0 and '다리 4 (다시 날기 FORWARD_LEGS=3:1): 3.00 m/s' in text
            and '다리 3: %.2f m/s, 요 스틱 +0.50' % HIGH in text, '')


def test_legs():
    verdict('기본 다리: 낮은 속도 전타, 높은 속도 전타, 높은 속도 반타',
            F.parse_legs(None, VEC_ALL) == [(LOW, 1.0), (HIGH, 1.0), (HIGH, 0.5)]
            and F.parse_legs('', CS_ALL) == [(2.0, 1.0), (2.5, 1.0), (2.5, 0.5)], '')
    verdict('번호와 속도:스틱으로 다시 고른다',
            F.parse_legs('2', VEC_FIN) == [(HIGH, 1.0)] and F.parse_legs('1,3', VEC_FIN) == [(LOW, 1.0), (HIGH, 0.5)]
            and F.parse_legs('4.0:1.0, 2.5:-0.5', CS_FIN) == [(4.0, 1.0), (2.5, -0.5)], '')
    bad = []

    for spec in ('0', '4', 'x', '4:0', '9.0:1.0', '2.0:1.5', ','):
        try:
            F.parse_legs(spec, VEC_FIN)
            bad.append(spec)
        except ValueError:
            pass

    verdict('읽을 수 없는 다리는 기본값으로 바뀌지 않고 오류다', not bad, '통과해 버린 것 %s' % bad)
    twice = []

    for spec in ('2,2', '2,%g:1.0' % HIGH, '1,%g:1' % LOW, '3.0:1.0,3:1'):
        try:
            F.parse_legs(spec, VEC_FIN)
            twice.append(spec)
        except ValueError:
            pass

    verdict('한 실행에 같은 다리 두 번은 오류다 (번호와 기록 이름이 겹친다)', not twice, '통과해 버린 것 %s' % twice)
    legs = [(HIGH, 0.5), (3.0, 1.0), (LOW, 1.0), (5.0, -1.0)]
    numbers = F.leg_numbers(VEC_FIN, legs)
    verdict('다리 번호: 기본 다리는 그 번호, 아닌 다리는 4 부터; 다시 날 명세는 그 다리로 읽힌다',
            numbers == [(3, '3'), (4, '3:1'), (1, '1'), (5, '5:-1')]
            and all(F.parse_legs(spec, VEC_FIN) == [leg] for leg, (_, spec) in zip(legs, numbers)), str(numbers))
    params = F.plan_params(VEC_FIN)
    verdict('지시 각속도: 반 스틱 6.667, 전 스틱 15, 데드존 안 0',
            near(F.setpoint_deg(0.5, params), 6.6667, 1e-3) and F.setpoint_deg(-1.0, params) == -15.0
            and F.setpoint_deg(0.05, params) == 0.0, '')


# ---------------------------------------------------------------------------
# 3. a synthetic log
# ---------------------------------------------------------------------------

S = 1_000_000
ACRO, ON, OFF, MANUAL2, ACRO2, END = 40 * S, 43 * S, 63 * S, 183_200_000, 184_200_000, 200 * S
U0, HEADING0 = 2.0, math.radians(30.0)
R_STICK, TAU_STICK = 4.0, 2.0              # deg/s the stick turn settles at, s
R_REL, TAU_REL = None, 50.0                # the release decays from the stick's last rate
I_LIM, TAU_I = 0.3, 200.0


def r_true(t):
    """deg/s at PX4 seconds t."""
    if t < ON / S:
        return 0.0
    if t < OFF / S:
        return R_STICK * (1.0 - math.exp(-(t - ON / S) / TAU_STICK))
    r_end = R_STICK * (1.0 - math.exp(-(OFF - ON) / S / TAU_STICK))
    return r_end * math.exp(-(t - OFF / S) / TAU_REL)


def heading_true(t):
    """rad, the integral of r_true."""
    if t <= ON / S:
        return HEADING0
    a = min(t, OFF / S) - ON / S
    turned = R_STICK * (a - TAU_STICK * (1.0 - math.exp(-a / TAU_STICK)))
    if t > OFF / S:
        r_end = R_STICK * (1.0 - math.exp(-(OFF - ON) / S / TAU_STICK))
        turned += r_end * TAU_REL * (1.0 - math.exp(-(t - OFF / S) / TAU_REL))
    return HEADING0 + math.radians(turned)


def integ_true(t):
    if t < ACRO / S or t >= MANUAL2 / S:
        return 0.0
    if t < ON / S:
        return 0.0
    if t < OFF / S:
        return min(I_LIM, 0.03 * (t - ON / S))
    return I_LIM * math.exp(-(t - OFF / S) / TAU_I)


def tilt_stbd(t):
    """deg: forward until the stick, a 1.5 s swing to 180, back at 100 s."""
    if t < ON / S + 0.5:
        return 0.0
    if t < ON / S + 2.0:
        return 120.0 * (t - ON / S - 0.5)
    if t < 100.0:
        return 180.0
    if t < 101.5:
        return 180.0 - 120.0 * (t - 100.0)
    return 0.0


STAB, ACRO3, ON3, OFF3 = 190 * S, 192 * S, 194 * S, 196 * S


def synthetic_log(stick=1.0, tail=False, crab_deg=0.0, integ_start=0.0, stab_acro=False, ignored_flag=False,
                  flag_value=1.0, thrust=PLAN_THRUST, torque_blip=False, torque_at_acro=0.0, q_rad=0.0, gt_ms=20,
                  imu_cutoff='yaml', integ_late_ms=0, rate=None, gyro_ms=8, speed=U0, sink=0.0, pitch_deg=0.0,
                  roll_deg=0.0, second_stick=None):
    """A Log with the topics forward_yaw_check reads, at their SITL rates.

    crab_deg: the velocity that far to starboard of the nose throughout.
    integ_start: the integrator the leg's Acro starts with (0 is a clear one).
    stab_acro: Stabilized at 190 s, then Acro at 192 s with a stick at 194 s:
    an Acro entered from another mode, which no leg may be built on.
    ignored_flag: the allocator reports -1 on yaw with the torque achieved at
    55-57 s, which the loop ignores (AirshipYawRateLoop::setSaturation).
    flag_value: the yaw shortfall the allocator reports, not achieved, at
    50-52 s.
    thrust: the thrust setpoint, a number or a function of PX4 seconds.
    torque_blip: the yaw torque setpoint at -0.1 for 40 ms at 60 s, shorter
    than actuator_motors' 100 ms.
    torque_at_acro: the yaw torque setpoint from 30 s to the stick onset.
    q_rad: the gyro's pitch rate throughout.
    gt_ms: the groundtruth's logging interval (the setpoints stay at 20 ms).
    imu_cutoff: IMU_GYRO_CUTOFF in the log's parameters: 'yaml' its default,
    None absent.
    integ_late_ms: the first rate_ctrl_status sample after each Acro switch
    comes this late.
    rate: the gyro's yaw rate, deg/s, a function of PX4 seconds in place of
    r_true (the groundtruth heading stays r_true's).
    gyro_ms: the gyro's logging interval (SITL's is 8 ms).
    speed: the horizontal speed, m/s (U0); sink: the groundtruth's vz, m/s
    down; pitch_deg, roll_deg: a constant pitch and roll in the groundtruth
    attitude, heading psi, ZYX (each 0: the attitude is the heading alone).
    second_stick: (on s, off s) of a second stick in the leg's Acro."""
    def grid(period_ms, start=0, end=END):
        return np.arange(start, end, period_ms * 1000, dtype=np.int64)

    def nav(t):
        if t < 10 * S:
            return 0, 1
        if t < ACRO:
            return 0, 2
        if t < MANUAL2:
            return 10, 2
        if t < ACRO2:
            return 0, 2
        if stab_acro and STAB <= t < ACRO3:
            return 15, 2
        return 10, 2

    def acro_rates(t):
        return nav(t)[0] == 10 and nav(t)[1] == 2

    data = {}
    # vehicle_status at 2 Hz plus every change
    changes = [10 * S, ACRO, MANUAL2, ACRO2] + ([STAB, ACRO3] if stab_acro else [])
    ts = np.unique(np.concatenate([grid(500), np.array(changes, dtype=np.int64)]))
    data['vehicle_status'] = {'timestamp': ts, 'nav_state': np.array([nav(t)[0] for t in ts]),
                              'arming_state': np.array([nav(t)[1] for t in ts])}
    closed = np.array([nav(t)[0] in (10, 15) for t in ts], dtype=float)
    data['vehicle_control_mode'] = {'timestamp': ts, 'flag_armed': np.array([nav(t)[1] == 2 for t in ts], dtype=float),
                                    'flag_control_manual_enabled': np.ones(len(ts)),
                                    'flag_control_rates_enabled': closed}
    tm = grid(200, 5 * S)
    moving = lambda t, lag: ((ON + lag <= t < OFF + lag) or (stab_acro and ON3 + lag <= t < OFF3 + lag)
                             or (second_stick is not None and second_stick[0] * S + lag <= t < second_stick[1] * S + lag))
    yaw = np.array([stick if moving(t, 100_000) else 0.0 for t in tm])   # the 5 Hz log lags the stick
    data['manual_control_setpoint'] = {'timestamp': tm, 'yaw': yaw}
    tr = np.array([t for t in grid(50, ACRO) if nav(t)[0] in (10, 15)], dtype=np.int64)
    sp = math.radians(F.setpoint_deg(stick, F.plan_params(VEC_FIN)))
    data['vehicle_rates_setpoint'] = {'timestamp': tr, 'yaw': np.array([sp if moving(t, 0) else 0.0 for t in tr])}
    tg = grid(gyro_ms)
    data['vehicle_angular_velocity'] = {'timestamp': tg, 'xyz[0]': np.zeros(len(tg)), 'xyz[1]': np.full(len(tg), q_rad),
                                        'xyz[2]': np.radians([(rate or r_true)(t / S) for t in tg])}
    t50 = grid(gt_ms)
    psi = np.array([heading_true(t / S) for t in t50])
    course = psi + math.radians(crab_deg)
    if pitch_deg or roll_deg:
        # ZYX: heading psi, then pitch, then roll
        cy, sy = np.cos(psi / 2), np.sin(psi / 2)
        cp, sp_ = math.cos(math.radians(pitch_deg) / 2), math.sin(math.radians(pitch_deg) / 2)
        cr, sr = math.cos(math.radians(roll_deg) / 2), math.sin(math.radians(roll_deg) / 2)
        data[F.GT_ATT] = {'timestamp': t50, 'q[0]': cr * cp * cy + sr * sp_ * sy, 'q[1]': sr * cp * cy - cr * sp_ * sy,
                          'q[2]': cr * sp_ * cy + sr * cp * sy, 'q[3]': cr * cp * sy - sr * sp_ * cy}
    else:
        data[F.GT_ATT] = {'timestamp': t50, 'q[0]': np.cos(psi / 2), 'q[1]': np.zeros(len(t50)),
                          'q[2]': np.zeros(len(t50)), 'q[3]': np.sin(psi / 2)}
    data[F.GT_POS] = {'timestamp': t50, 'vx': speed * np.cos(course), 'vy': speed * np.sin(course),
                      'vz': np.full(len(t50), float(sink)), 'z': np.full(len(t50), -20.0)}
    data['vehicle_local_position'] = {'timestamp': t50, 'vx': speed * np.cos(course), 'vy': speed * np.sin(course),
                                      'heading': psi}
    ts = grid(20)

    def torque(t):
        if torque_blip and 60 * S <= t < 60 * S + 40_000:
            return -0.1
        if 30 * S <= t < ON and torque_at_acro:
            return torque_at_acro
        return integ_true(t / S)

    thrust_at = thrust if callable(thrust) else (lambda t, c=thrust: c)
    data['vehicle_torque_setpoint'] = {'timestamp': ts, 'xyz[0]': np.zeros(len(ts)), 'xyz[1]': np.zeros(len(ts)),
                                       'xyz[2]': np.array([torque(t) for t in ts])}
    data['vehicle_thrust_setpoint'] = {'timestamp': ts, 'xyz[0]': np.array([thrust_at(t / S) for t in ts]),
                                       'xyz[1]': np.zeros(len(ts)), 'xyz[2]': np.zeros(len(ts))}
    t5 = np.concatenate([grid(200, ACRO + integ_late_ms * 1000, MANUAL2), np.array([MANUAL2], dtype=np.int64),
                         grid(200, ACRO2 + integ_late_ms * 1000, END)])
    integ = np.array([integ_true(t / S) + (integ_start if ACRO <= t < ON else 0.0) for t in t5])
    data['rate_ctrl_status'] = {'timestamp': t5, 'yawspeed_integ': integ}
    tc = grid(200)
    flag = (tc >= 50 * S) & (tc < 52 * S)
    ignored = ((tc >= 55 * S) & (tc < 57 * S)) if ignored_flag else np.zeros(len(tc), dtype=bool)
    data['control_allocator_status'] = {'timestamp': tc,
                                        'unallocated_torque[2]': np.where(flag, flag_value,
                                                                          np.where(ignored, -1.0, 0.0)),
                                        'torque_setpoint_achieved': np.where(flag, 0.0, 1.0)}
    ta = grid(100)
    servo = np.array([tilt_stbd(t / S) / 180.0 for t in ta])
    data['actuator_servos'] = {'timestamp': ta, 'control[0]': servo, 'control[1]': np.zeros(len(ta))}
    tail_cmd = np.where((ta >= ON) & (ta < OFF), 0.5, np.where((ta >= OFF) & (ta < 80 * S), -0.3, 0.0))
    data['actuator_motors'] = {'timestamp': ta, 'control[0]': np.full(len(ta), 0.1), 'control[1]': np.full(len(ta), 0.2),
                               'control[2]': tail_cmd}
    params = dict(F.plan_params(CS_ALL if tail else VEC_ALL))
    params['SYS_AUTOSTART'] = 2517.0 if tail else 2527.0

    if imu_cutoff is not None:
        params['IMU_GYRO_CUTOFF'] = gyro_cutoff_yaml() if imu_cutoff == 'yaml' else float(imu_cutoff)

    return F.Log(data, params, start_us=5 * S, end_us=END, info={'ver_sw': 'synthetic'})


def gyro_cutoff_yaml():
    return F.yaml_default('IMU_GYRO_CUTOFF', F.ANALYSIS_PARAMS['IMU_GYRO_CUTOFF'])


def test_synthetic():
    log = synthetic_log()
    legs = F.find_legs(log)
    verdict('합성 로그: 스틱이 있는 Acro 구간 하나만 다리다 (제동 Acro 는 아니다)', len(legs) == 1, '%d 개' % len(legs))

    if len(legs) != 1:
        return

    leg = legs[0]
    verdict('다리의 시각: Acro 진입, 스틱 켬/끔, Acro 끝, 놓음 끝',
            (leg['manual_us'], leg['acro_us'], leg['on_us'], leg['off_us'], leg['acro_end_us'], leg['release_end_us'])
            == (10 * S, ACRO, ON, OFF, MANUAL2, OFF + 120 * S), str(leg))
    plant = F.plant_info(VEC_ALL, {k: log.param(k) for k in F.PARAMS})
    table = F.leg_table(log, leg, plant)
    t = table['t_px4_us']
    verdict('표는 진입 5 s 전부터 놓음 끝까지 50 Hz 다',
            t[0] == ACRO - 5 * S and t[-1] == OFF + 120 * S and np.all(np.diff(t) == 20_000), '%d 행' % len(t))
    stick_rows = F.rows_between(table, ON, OFF)
    verdict('u, v 는 진로 좌표의 대지 속도다 (u 2, v 0)',
            np.allclose(table['u'], U0, atol=1e-9) and np.allclose(table['v'], 0.0, atol=1e-9), '')
    verdict('헤딩 변화가 각속도의 적분이다',
            np.allclose(np.radians(table['heading_deg']), [heading_true(x / S) for x in t], atol=1e-9), '')
    verdict('헤딩 미분이 자이로와 같다 (스틱 중)',
            np.nanmax(np.abs(table['r_hdg_deg_s'][stick_rows] - table['r_deg_s'][stick_rows])) < 0.05,
            '%.4f deg/s' % np.nanmax(np.abs(table['r_hdg_deg_s'][stick_rows] - table['r_deg_s'][stick_rows])))
    verdict('틸트가 서보에서 도로 계산된다 (-180..180)',
            near(F.at_time(table, 'tilt_stbd_deg', 50 * S), 180.0, 1e-9)
            and near(F.at_time(table, 'tilt_stbd_deg', 41 * S), 0.0, 1e-9), '')
    verdict('루프가 닫힌 곳에서만 지시 각속도가 있다',
            np.all(np.isnan(table['yaw_rate_sp_deg_s'][F.rows_between(table, MANUAL2, ACRO2)]))
            or not F.rows_between(table, MANUAL2, ACRO2).any(), '')
    m = F.leg_metrics(log, leg, table, plant, {k: log.param(k) for k in F.PARAMS}, target=2.0)
    s, r, e = m['stick_phase'], m['release'], m['entry']
    want_last5 = np.mean([r_true(x) for x in np.arange(OFF / S - 5.0, OFF / S, 0.008)])
    verdict('스틱 마지막 5 s 평균 각속도', near(s['r_mean_last5_deg_s'], want_last5, 1e-3),
            '%.5f, 기대 %.5f' % (s['r_mean_last5_deg_s'], want_last5))
    verdict('스틱 중 최대 각속도와 그 시각', near(s['r_peak_deg_s'], r_true(OFF / S - 0.008), 1e-3)
            and near(s['r_peak_after_s'], 20.0, 0.02), '%s at %s s' % (s['r_peak_deg_s'], s['r_peak_after_s']))
    verdict('적분기: 끝 값, 한계 도달 시각 10 s', near(s['integrator_end'], I_LIM, 1e-12)
            and near(s['integrator_at_limit_after_s'], 9.8, 0.21), '%s, %s s' % (s['integrator_end'], s['integrator_at_limit_after_s']))
    verdict('깃발: 끝에서 없음, 스틱 중 2 s 동안 (10 %)', s['flag_pos_end'] is False and s['flag_neg_end'] is False
            and near(s['flagged_fraction'], 0.1, 1e-9), '%s' % s['flagged_fraction'])
    verdict('토크 평균 (마지막 5 s)', near(s['torque_mean_last5'], I_LIM, 1e-9), '%s' % s['torque_mean_last5'])
    want_turn = math.degrees(heading_true(OFF / S + 120.0) - heading_true(OFF / S))
    verdict('놓은 뒤 120 s 의 헤딩 변화', near(r['heading_change_deg'], want_turn, 0.01),
            '%.3f, 기대 %.3f deg' % (r['heading_change_deg'], want_turn))

    def window_mean(a, b):
        return np.mean([r_true(x / S) for x in np.arange(int(a * S), int(b * S), 8000)])

    offsets = (('+10', 10.0), ('+60', 60.0), ('+120', 120.0))
    one = {k: r_true(OFF / S + v) for k, v in offsets}          # the 125 Hz gyro has a sample on each mark
    means = {k: window_mean(OFF / S + v - 1.0, OFF / S + v) for k, v in offsets}
    verdict('놓은 뒤 +10/+60/+120 s 의 각속도: 그 시각의 자이로 한 표본과 앞 1 s 평균을 따로',
            all(near(r['r_deg_s'][k], one[k], 1e-6) and near(r['r_mean1s_deg_s'][k], means[k], 1e-6) for k in one)
            and r['marks_after_s'] == {'+10': 10.0, '+60': 60.0, '+120': 120.0},
            ', '.join('%s %.4f/%.4f (1 s %.4f/%.4f)' % (k, r['r_deg_s'][k], one[k], r['r_mean1s_deg_s'][k], means[k])
                      for k in one))
    verdict('놓은 뒤 +120 s 의 적분기', near(r['integrator']['+120'], integ_true(OFF / S + 120.0), 1e-3),
            '%s, 기대 %.5f' % (r['integrator']['+120'], integ_true(OFF / S + 120.0)))
    rv = s['reversals'], r['reversals']
    # the swing back passes cos(tilt) = +0.5 (60 deg) at 101.0 s
    verdict('포드 반전: 스틱 중 우현이 뒤로 1 번, 놓음 중 앞으로 1 번, 좌현 0, 단계마다 합 1',
            rv[0]['stbd']['into_rear'] == 1 and rv[0]['stbd']['back_forward'] == 0
            and rv[1]['stbd']['back_forward'] == 1 and rv[1]['stbd']['into_rear'] == 0
            and rv[0]['port'] == {'into_rear': 0, 'back_forward': 0, 'rear_s': 0.0}
            and rv[0]['flips'] == 1 and rv[1]['flips'] == 1
            and near(rv[1]['stbd']['rear_s'], 101.0 - OFF / S, 0.15), str(rv))
    verdict('진입 상태: 적분기 0 에서 시작, u 2', e['integrator_first'] == 0.0 and near(e['at_acro']['u'], 2.0, 1e-9), '')
    b = r['entry']
    verdict('놓을 때의 상태가 진입 상태처럼 기록된다 (u, r, 적분기, 틸트; 재현 블록에도)',
            near(b['state']['u'], U0, 1e-9) and near(b['state']['r_deg_s'], r_true(OFF / S), 1e-6)
            and near(b['integrator'], I_LIM, 1e-12) and b['integrator_age_s'] == 0.0
            and near(b['actuators']['tilt_stbd_deg'], 180.0, 1e-9) and m['replay']['state_at_release'] == b['state']
            and m['replay']['integrator_at_release'] == b['integrator']
            and m['replay']['actuators_at_release'] == b['actuators'], str(b['state']))
    th = m['throttle']
    verdict('스로틀: 진입 전 창부터 놓음 끝까지의 추력 설정 표본',
            th['min'] == th['max'] == PLAN_THRUST and th['n'] > 0 and th['from_px4_us'] == ACRO - 5 * S
            and th['to_px4_us'] == OFF + 120 * S, str(th))
    # a release the duration verdict passes (0.2 s short, inside 0.3) keeps its
    # last mark, taken at its end; one 0.5 s short has none
    cut = {}

    for short_us in (200_000, 500_000):
        end_us = OFF + 120 * S - short_us
        short = dict(leg, acro_end_us=end_us, release_end_us=end_us)
        cut[short_us] = F.leg_metrics(log, short, F.leg_table(log, short, plant), plant,
                                      {k: log.param(k) for k in F.PARAMS}, target=2.0)['release']

    ok_cut, far_cut = cut[200_000], cut[500_000]
    verdict('놓음이 허용 안에서 짧으면 +120 은 그 끝에서 실제 시각과 함께, 허용 밖이면 없다',
            ok_cut['marks_after_s']['+120'] == 119.8 and near(ok_cut['r_deg_s']['+120'], r_true(OFF / S + 119.8), 1e-6)
            and near(ok_cut['integrator']['+120'], integ_true(OFF / S + 119.8), 1e-9)
            and far_cut['marks_after_s']['+120'] is None and far_cut['r_deg_s']['+120'] is None
            and far_cut['integrator']['+120'] is None and far_cut['r_deg_s']['+60'] is not None,
            '%s / %s' % (ok_cut['marks_after_s'], far_cut['marks_after_s']))
    verdict('정상 직진 판정 (목표 2.0)', m['steady']['ok'] is True, str(m['steady']['before_entry']))
    m_far = F.leg_metrics(log, leg, table, plant, {k: log.param(k) for k in F.PARAMS}, target=2.5)
    verdict('속도가 다르면 정상 직진이 아니다 (목표 2.5)', m_far['steady']['ok'] is False, '')
    replay = m['replay']
    verdict('재현 블록: 진입 상태, 스로틀, 스틱 시각, 이득',
            replay['stick'] == [[0.0, 0.0], [3.0, 1.0], [23.0, 0.0]] and near(replay['throttle'], PLAN_THRUST, 1e-9)
            and replay['params']['AS_YAWRATE_FF'] == P27['AS_YAWRATE_FF'] and replay['state']['u'] is not None, str(replay['stick']))


def test_synthetic_variants():
    """The cases the first fixture could not tell apart: a mutation run on
    2026-10-01 showed a flipped sway sign, an integrator assumed clear, an
    Acro entered from Stabilized and a flag the loop ignores all passing."""
    params = lambda log: {k: log.param(k) for k in F.PARAMS}
    log = synthetic_log(crab_deg=5.0)
    leg = F.find_legs(log)[0]
    plant = F.plant_info(VEC_ALL, params(log))
    table = F.leg_table(log, leg, plant)
    before = F.rows_between(table, ACRO, ON)
    beta = math.radians(5.0)
    verdict('옆미끄럼: 코보다 5 deg 우현으로 가면 v = +U sin 5, u = U cos 5 (FRD)',
            np.allclose(table['v'][before], U0 * math.sin(beta), atol=1e-9)
            and np.allclose(table['u'][before], U0 * math.cos(beta), atol=1e-9),
            'v %.5f, 기대 %.5f' % (float(np.mean(table['v'][before])), U0 * math.sin(beta)))
    alphas = [float(np.mean(table[name][before])) for name in F.fin_names(plant)]
    verdict('회전이 없을 때 수직 핀 받음각이 옆미끄럼각이다 (+5 deg)', all(near(a, 5.0, 1e-6) for a in alphas), str(alphas))
    verdict('EKF 의 u, v 도 같은 좌표다', np.allclose(table['v_ekf'][before], U0 * math.sin(beta), atol=1e-9), '')
    # with rotation: the cp moves sideways at r x_cp, x_cp aft of the origin
    end = int(np.nonzero(F.rows_between(table, ON, OFF))[0][-1])
    r_rad = math.radians(table['r_deg_s'][end])
    want = [math.degrees(math.atan2(U0 * math.sin(beta) + r_rad * f['cp'][0], U0 * math.cos(beta)))
            for f in plant['fins']]
    got = [float(table[name][end]) for name in F.fin_names(plant)]
    verdict('선회 중 수직 핀 받음각은 cp 에서의 흐름이다 (옆미끄럼 + r x_cp)', all(near(a, b, 1e-6) for a, b in zip(got, want))
            and got[0] < 0.0, '%s, 기대 %s' % (got, want))

    with tempfile.TemporaryDirectory() as out:
        log = synthetic_log(integ_start=0.05)
        _, checks = F.analyze(log, VEC_ALL, out_dir=out, stamp='carry')
        clear = [ok for name, ok, _ in checks if '적분기가 비어서 시작했다' in name]
        verdict('남은 적분기 (0.05) 로 시작한 다리는 그 판정에서 실패한다', clear == [False], str(clear))
        # a gyro gap of 2 s inside the leg, a logger dropout, a hull that sank
        log = synthetic_log()
        av = log.data['vehicle_angular_velocity']
        keep = (av['timestamp'] < 70 * S) | (av['timestamp'] >= 72 * S)
        log.data['vehicle_angular_velocity'] = {k: v[keep] for k, v in av.items()}
        _, checks = F.analyze(log, VEC_ALL, out_dir=out, stamp='gap')
        rates = [(ok, d) for name, ok, d in checks if '토픽' in name]
        verdict('다리 안에서 2 s 빠진 자이로는 토픽 판정을 실패시킨다', len(rates) == 1 and not rates[0][0]
                and 'vehicle_angular_velocity' in rates[0][1].split('모자란 토픽:')[-1], rates[0][1][-80:] if rates else '')
        log = synthetic_log()
        log.dropouts = [(100 * S, 300)]
        _, checks = F.analyze(log, VEC_ALL, out_dir=out, stamp='drop')
        rates = [ok for name, ok, _ in checks if '토픽' in name]
        verdict('다리 안의 로거 끊김은 토픽 판정을 실패시킨다', rates == [False], str(rates))
        log = synthetic_log()
        gt = log.data[F.GT_POS]
        gt['z'] = np.where(gt['timestamp'] >= 150 * S, -4.0, gt['z'])
        _, checks = F.analyze(log, VEC_ALL, out_dir=out, stamp='low')
        airborne = [ok for name, ok, _ in checks if '떠 있었다' in name]
        verdict('놓음 중 고도 4 m 로 내려온 다리는 떠 있지 않았다', airborne == [False], str(airborne))
    log = synthetic_log(stab_acro=True)
    legs = F.find_legs(log)
    verdict('Stabilized 에서 들어간 Acro 의 스틱은 다리가 아니다 (적분기가 비지 않았다)',
            len(legs) == 1 and legs[0]['acro_us'] == ACRO and F.stick_segments(log, ACRO3, END) == [(ON3, OFF3)],
            '%d 개, 그 Acro 의 스틱 %s' % (len(legs), F.stick_segments(log, ACRO3, END)))
    log = synthetic_log(ignored_flag=True)
    leg = F.find_legs(log)[0]
    table = F.leg_table(log, leg, plant)
    ignored, real = F.rows_between(table, 55 * S, 57 * S), F.rows_between(table, 50 * S + 200_000, 52 * S)
    m = F.leg_metrics(log, leg, table, plant, params(log))
    verdict('토크가 달성된 동안의 미할당 부호는 루프가 무시한다: 깃발은 달성되지 않은 동안만',
            not table['sat_neg'][ignored].any() and table['sat_pos'][real].all() and not table['sat_neg'].any()
            and near(m['stick_phase']['flagged_fraction'], 0.1, 1e-9),
            'sat_neg %d 행, sat_pos %d/%d 행, 비율 %s' % (int(table['sat_neg'].sum()), int(table['sat_pos'][real].sum()),
                                                      int(real.sum()), m['stick_phase']['flagged_fraction']))
    # setSaturation's bound is FLT_EPSILON (1.19e-7): 5e-7 is a flag the loop
    # sees and a 1e-6 bound would not; 1e-7 is under it
    seen = {}

    for value in (5e-7, 1e-7):
        log = synthetic_log(flag_value=value)
        leg = F.find_legs(log)[0]
        table = F.leg_table(log, leg, plant)
        m = F.leg_metrics(log, leg, table, plant, params(log))
        rows = F.rows_between(table, 50 * S + 200_000, 52 * S)
        seen[value] = (bool(table['sat_pos'][rows].all()), bool(table['sat_pos'].any()), m['stick_phase']['flagged_fraction'])

    verdict('깃발 문턱은 FLT_EPSILON: 부족분 5e-7 은 깃발 (1e-6 문턱이면 놓친다), 1e-7 은 아니다',
            seen[5e-7][0] and near(seen[5e-7][2], 0.1, 1e-9) and not seen[1e-7][1] and seen[1e-7][2] == 0.0, str(seen))


def test_live_state_frame():
    """Live.state's u, v from a listener text, with the same frame as the log's."""
    import types

    beta, psi, speed = math.radians(5.0), math.radians(150.0), 3.0

    def listener(topic):
        if topic == F.GT_POS:
            return ('    timestamp: 7000000 (0.0 seconds ago)\n    vx: %.5f\n    vy: %.5f\n    z: -20.00000\n    heading: %.5f\n'
                    % (speed * math.cos(psi + beta), speed * math.sin(psi + beta), psi))
        if topic == 'vehicle_angular_velocity':
            return '    timestamp: 7000000\n    xyz: [0.00000, 0.00000, 0.01000]\n'
        return '    nav_state: 10\n    arming_state: 2\n'

    import importlib
    os.environ['PX4_GZ_ENTITY'] = VEC_ALL
    real = importlib.import_module('sitl_link')
    os.environ.pop('PX4_GZ_ENTITY', None)
    sl = types.SimpleNamespace(listener=listener, field=real.field, vector=real.vector, finite=real.finite)
    s = F.Live(sl, None).state()
    verdict('실시간 상태도 v 는 우현이 양이다 (5 deg 옆미끄럼, 헤딩 150 deg)',
            s is not None and near(s['v'], speed * math.sin(beta), 1e-4) and near(s['u'], speed * math.cos(beta), 1e-4)
            and near(s['r'], math.degrees(0.01), 1e-6) and s['alt'] == 20.0, str(s))


def test_judgement_and_timeline():
    t = np.arange(0.0, 5.0, 0.2)
    ok, n = F.steady_judgement(t, np.full(len(t), 4.0), np.zeros(len(t)), np.zeros(len(t)), 4.0)
    verdict('정상 직진: 고른 4 m/s', ok, str(n))
    tests = (('속도 5 % 밖', np.full(len(t), 4.25), 0.0, 0.0),
             ('끌려가는 속도', 3.9 + 0.04 * t, 0.0, 0.0),
             ('|r| 0.3 deg/s', np.full(len(t), 4.0), 0.0, 0.3),
             ('옆미끄럼 0.06 m/s', np.full(len(t), 4.0), 0.06, 0.0))
    for name, u, v, r in tests:
        ok, n = F.steady_judgement(t, u, np.full(len(t), v), np.full(len(t), r), 4.0)
        verdict('정상 직진이 아니다: %s' % name, not ok, str(n))
    u = np.full(len(t), 4.0)
    u[3] = float('nan')
    ok, _ = F.steady_judgement(t, u, np.zeros(len(t)), np.zeros(len(t)), 4.0)
    verdict('읽지 못한 표본이 있는 창은 정상이 아니다', not ok, '')
    leg = {'acro_us': ACRO, 'on_us': ON, 'off_us': OFF, 'acro_end_us': MANUAL2}
    good = {'acro': (ACRO - 30_000, ACRO - 10_000), 'stick_on': (ON - 60_000, ON - 40_000),
            'stick_off': (OFF - 60_000, OFF - 40_000), 'release_end': (MANUAL2 - 20_000, MANUAL2 + 5_000)}
    ok, detail = F.timeline_check(leg, good)
    verdict('시각 대조: 괄호 안이면 맞는다', ok, detail)
    late = dict(good, stick_on=(ON - 2_060_000, ON - 2_040_000))
    ok, detail = F.timeline_check(leg, late)
    verdict('시각 대조: 2 s 어긋난 스틱은 틀린다', not ok, detail)
    unread = dict(good, stick_off=(None, OFF))
    ok, detail = F.timeline_check(leg, unread)
    verdict('시각 대조: 읽지 못한 시각은 통과가 아니다', not ok, detail)


def test_reversal_counters():
    t = np.arange(0, 100, dtype=np.int64) * 100_000
    dither = 90.0 + 5.0 * np.sin(np.arange(100))
    verdict('90 deg 근처의 떨림은 반전이 아니다', F.pod_reversals(t, dither, 0, 10 * S)[:2] == (0, 0), '')
    wrap = np.where(np.arange(100) % 2 == 0, 179.0, -179.0)
    verdict('+-180 를 오가는 것은 둘 다 뒤쪽이다', F.pod_reversals(t, wrap, 0, 10 * S)[:2] == (0, 0), '')
    swing = np.concatenate([np.zeros(30), np.linspace(0, 180, 20), np.full(30, 180.0), np.linspace(180, 0, 20)])
    verdict('앞 -> 뒤 -> 앞 은 뒤로 1, 앞으로 1', F.pod_reversals(t, swing, 0, 10 * S)[:2] == (1, 1), '')
    verdict('구간 밖의 반전은 세지 않는다', F.pod_reversals(t, swing, 6 * S, 10 * S)[:2] == (0, 1), '')
    # forward.py's band: 60 and 120 deg. A pod swinging 50 <-> 110 deg reaches
    # neither back threshold and counts nothing; 50 <-> 130 counts each swing
    # (a 90 +- 10 deg band would have counted the first)
    near_side = np.where(np.arange(100) % 20 < 10, 50.0, 110.0)
    far_side = np.where(np.arange(100) % 20 < 10, 50.0, 130.0)
    verdict('반전은 forward.py 의 띠다: 50 <-> 110 deg 는 0 번, 50 <-> 130 deg 는 오갈 때마다',
            F.pod_reversals(t, near_side, 0, 10 * S)[:2] == (0, 0) and F.pod_reversals(t, far_side, 0, 10 * S)[:2] == (5, 4),
            '%s / %s' % (F.pod_reversals(t, near_side, 0, 10 * S), F.pod_reversals(t, far_side, 0, 10 * S)))
    tail = np.array([0.5, 0.01, -0.5, -0.01, 0.5, 0.0, 0.015, -0.6])
    tt = np.arange(len(tail), dtype=np.int64) * S
    verdict('꼬리 방향 바뀜: 데드밴드 안은 어느 쪽도 아니다', F.sign_reversals(tt, tail, 0, 10 * S) == 3,
            str(F.sign_reversals(tt, tail, 0, 10 * S)))
    log = synthetic_log(tail=True)
    leg = F.find_legs(log)[0]
    plant = F.plant_info(CS_ALL, {k: log.param(k) for k in F.PARAMS})
    table = F.leg_table(log, leg, plant)
    m = F.leg_metrics(log, leg, table, plant, {k: log.param(k) for k in F.PARAMS})
    verdict('Cloudship: 꼬리가 스틱 중 한 방향, 놓은 뒤 한 번 바뀐다 (flips 도 같다), 집합 틸트 하나',
            m['stick_phase']['reversals'].get('tail_flips') == 0 and m['release']['reversals'].get('tail_flips') == 1
            and m['stick_phase']['reversals'].get('flips') == 0 and m['release']['reversals'].get('flips') == 1
            and 'tilt_deg' in table and 'motor3' in table, str(m['release']['reversals']))


def test_analyze_end_to_end():
    log = synthetic_log()
    plan = F.vehicle_plan(VEC_ALL)['legs'][1]
    plan = dict(plan, speed=2.0, stick=1.0)
    events = {'acro': (ACRO - 30_000, ACRO - 10_000), 'stick_on': (ON - 60_000, ON - 40_000),
              'stick_off': (OFF - 60_000, OFF - 40_000), 'release_end': (MANUAL2 - 20_000, MANUAL2 + 5_000)}
    live = {'model': VEC_ALL, 'legs': [{'leg': 1, 'flown': True, 'plan': plan, 'events': events}]}

    with tempfile.TemporaryDirectory() as out:
        doc, checks = F.analyze(log, VEC_ALL, live=live, out_dir=out, stamp='test')
        files = sorted(os.listdir(out))
        failed = [(n, d) for n, ok, d in checks if not ok]
        verdict('합성 로그 전체 분석: 스로틀과 자이로 필터를 포함한 모든 타당성 판정이 통과한다',
                not failed and len(checks) == 9 and any('스로틀이' in n for n, _, _ in checks)
                and checks[-1][0] == '로그의 IMU_GYRO_CUTOFF 가 모델이 쓰는 yaml 기본값이다', str(failed)[:300])
        verdict('다리 CSV 를 쓴다', files == ['forward_%s_test_leg1.csv' % VEC_ALL], str(files))
        verdict('요약에 시간 분해능 (적분기 200 ms, 자이로 8 ms) 과 정의 (반전은 forward.py 의 것) 가 있다',
                doc['resolution_ms']['integrator'] == 200.0 and doc['resolution_ms']['yaw and roll rate (gyro)'] == 8.0
                and doc['resolution_ms']['allocator flags and unallocated torque'] == 200.0
                and 'forward.py' in doc['definitions']['reversals'] and doc['release_s'] == 120.0,
                str(doc['resolution_ms']))
        # the throttle verdict: a throttle that moved in the release, or a plan
        # one count away, fails it; without a plan, a moved one still does
        moved = synthetic_log()
        thrust = moved.data['vehicle_thrust_setpoint']
        thrust['xyz[0]'] = np.where(thrust['timestamp'] >= 100 * S, 0.0, thrust['xyz[0]'])
        one_off = {'model': VEC_ALL, 'legs': [{'leg': 1, 'flown': True, 'events': events,
                                                'plan': dict(plan, throttle=0.123, throttle_counts=123)}]}
        outcome = []

        for name, run in (('moved', lambda: F.analyze(moved, VEC_ALL, live=live, out_dir=out, stamp='thr1')),
                          ('one count', lambda: F.analyze(log, VEC_ALL, live=one_off, out_dir=out, stamp='thr2')),
                          ('moved, no plan', lambda: F.analyze(moved, VEC_ALL, out_dir=out, stamp='thr3'))):
            outcome.append((name, [ok for n, ok, d in run()[1] if '스로틀이' in n]))

        verdict('놓음 중 바뀐 스로틀, 계획과 한 계수 다른 스로틀은 그 다리의 스로틀 판정을 실패시킨다 (계획 없이도)',
                all(found == [False] for _, found in outcome), str(outcome))
        # a leg is named by its plan number, not by its place in the log
        third = {'model': VEC_ALL, 'legs': [{'leg': 3, 'flown': True, 'plan': plan, 'events': events}]}
        doc3, checks = F.analyze(log, VEC_ALL, live=third, out_dir=out, stamp='third')
        names = [n for n, _, _ in checks if n.startswith('다리')]
        verdict('분석의 다리 이름, CSV, 요약은 계획의 번호를 쓴다 (로그의 순서가 아니다)',
                [m['leg'] for m in doc3['legs']] == [3] and os.path.exists(os.path.join(out, 'forward_%s_third_leg3.csv' % VEC_ALL))
                and names and all(n.startswith('다리 3:') for n in names), str(names[:2]))

        with open(os.path.join(out, files[0])) as handle:
            rows = list(csv.reader(handle))

        need = ('t_s', 'u', 'v', 'r_deg_s', 'heading_deg', 'roll_deg', 'pitch_deg', 'yaw_stick', 'throttle', 'torque_z',
                'yawspeed_integ', 'unalloc_yaw', 'sat_pos', 'sat_neg', 'motor1', 'motor2', 'tilt_stbd_deg', 'tilt_port_deg')
        verdict('CSV 에 재현이 쓸 열이 모두 있다', all(c in rows[0] for c in need) and len(rows) == len(
            F.leg_table(log, F.find_legs(log)[0], F.plant_info(VEC_ALL, doc['params']))['t_s']) + 1,
            '없는 열 %s' % [c for c in need if c not in rows[0]])
        shifted = {'model': VEC_ALL, 'legs': [{'leg': 1, 'flown': True, 'plan': plan,
                                                'events': dict(events, acro=(ACRO - 3 * S, ACRO - 2 * S))}]}
        _, checks = F.analyze(log, VEC_ALL, live=shifted, out_dir=out, stamp='test2')
        timeline = [ok for n, ok, d in checks if '시각' in n]
        verdict('스크립트가 잰 시각과 로그가 어긋나면 그 다리가 실패한다', timeline == [False], str(timeline))
        lost = {'model': VEC_ALL, 'legs': [{'leg': 1, 'flown': True, 'plan': plan,
                                             'events': dict(events, stick_on=(ON + 30 * S, ON + 31 * S))}]}
        _, checks = F.analyze(log, VEC_ALL, live=lost, out_dir=out, stamp='test3')
        verdict('스크립트의 다리가 로그에 없으면 실패한다', any(not ok and '로그에 이 다리가 있다' in n for n, ok, d in checks), '')

    code, _ = run_main(['--analyze', __file__, '--out', os.path.join(gz_model.ROOT, 'build')])
    verdict('--analyze 는 PX4 트리 안에 쓰지 않는다', code == 2, str(code))
    verdict('로그 이름에서 표지를 만든다', F.stamp_of('/x/2026-10-01/10_47_36.ulg') == '20261001_104736'
            and F.stamp_of('sitl_out/forward_cloudship_fin_20261001_104736.ulg') == '20261001_104736', '')


# ---------------------------------------------------------------------------
# 3b. the model replay's interface: what the 2026-10-01 review of the record
#     against yaw_model/forward.py asked the record to add
# ---------------------------------------------------------------------------

# The leg CSV's columns before 2026-10-01, in order (the 2527 run's header):
# every one keeps its place, the new ones come after
OLD_COLUMNS_VECTORED = ['t_s', 't_px4_us', 'phase', 'nav_state', 'loop_closed', 'u', 'v', 'w_down', 'alt_m', 'u_ekf',
                        'v_ekf', 'p_deg_s', 'r_deg_s', 'r_hdg_deg_s', 'heading_deg', 'roll_deg', 'pitch_deg',
                        'yaw_stick', 'yaw_rate_sp_deg_s', 'throttle', 'torque_z', 'yawspeed_integ', 'unalloc_yaw',
                        'torque_achieved', 'sat_pos', 'sat_neg', 'motor1', 'motor2', 'servo1', 'tilt_stbd_deg',
                        'servo2', 'tilt_port_deg', 'alpha_fin1_deg', 'alpha_fin2_deg']
NEW_COLUMNS = ['t_acro_s', 'fin_dorsal_alpha_deg', 'fin_dorsal_flow', 'fin_ventral_alpha_deg', 'fin_ventral_flow']


def leg_of(log, model=VEC_ALL, target=2.0):
    """(leg, plant, table, metrics) of a synthetic log's one leg, with the
    parameters analyze() reads."""
    params = {k: log.param(k) for k in F.PARAMS + tuple(F.ANALYSIS_PARAMS)}
    leg = F.find_legs(log)[0]
    plant = F.plant_info(model, params)
    table = F.leg_table(log, leg, plant)
    return leg, plant, table, F.leg_metrics(log, leg, table, plant, params, target=target)


def model_source(name):
    with open(os.path.join(HERE, 'yaw_model', name)) as handle:
        return handle.read()


def test_interface_constants():
    """The constants the new fields stand on, against the files that define
    them (read as text: yaw_model is another tool's, imported by nothing
    here)."""
    found = re.search(r'^GAINS = (\(.*?\))$', model_source('record_gains.py'), re.M)
    gains = ast.literal_eval(found.group(1)) if found else None
    verdict('run_leg 에 넘길 이득은 yaw_model/record_gains.GAINS 다', gains == F.GAINS, str(gains))
    found = re.search(r'^PEAK_WINDOW_S\s*=\s*([0-9.]+)', model_source('forward.py'), re.M)
    verdict('평활 최대의 시간 창은 yaw_model/forward.py 의 PEAK_WINDOW_S 다 (%g s)' % F.PEAK_WINDOW_S,
            found is not None and float(found.group(1)) == F.PEAK_WINDOW_S, found.group(0) if found else '없음')
    path = os.path.join(gz_model.ROOT, 'src/modules/control_allocator/VehicleActuatorEffectiveness/AirshipPod.hpp')

    with open(path) as handle:
        pod = handle.read()

    engage = re.search(r'kSteerEngage\s*=\s*([0-9.]+)f;', pod)
    release = re.search(r'kSteerRelease\s*=\s*([0-9.]+)f;', pod)
    verdict('포드 모드의 띠는 AirshipPod.hpp 의 kSteerEngage, kSteerRelease 다 (float32 로 비교)',
            engage is not None and release is not None and F.POD_ENGAGE == np.float32(float(engage.group(1)))
            and F.POD_RELEASE == np.float32(float(release.group(1))) and F.POD_ENGAGE.dtype == np.float32
            and F.POD_RELEASE.dtype == np.float32, '%s %s' % (engage and engage.group(1), release and release.group(1)))
    rule = "('dorsal' if f['cp'][2] > com[2] else 'ventral')"
    verdict('핀 이름은 yaw_model/planar.py 의 규칙이다: cp 가 무게 중심보다 위면 dorsal', rule in model_source('planar.py'), rule)
    found = re.search(r"if speed <= ([0-9.]+) or _dot\(fin\['forward'\], vel\) <= 0\.0:", model_source('planar.py'))
    verdict('핀에 흐름이 없는 속도는 yaw_model/planar.py lift_drag_alpha 의 것이다 (%g m/s 이하, LiftDrag.cc 의 조기 반환)'
            % F.FIN_MIN_SPEED, found is not None and float(found.group(1)) == F.FIN_MIN_SPEED,
            found.group(0) if found else '없음')
    tree = ast.parse(model_source('forward.py'))
    run_leg = next((n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'run_leg'), None)
    names = {a.arg for a in run_leg.args.args + run_leg.args.kwonlyargs} if run_leg else set()
    block = leg_of(synthetic_log())[3]['replay']['run_leg'] or {}
    verdict('replay.run_leg 의 이름이 모두 yaw_model/forward.run_leg 의 인자다 (그대로 넘길 수 있다)',
            bool(block) and set(block) <= names and {'target', 'u0', 'stick'} <= set(block),
            '인자에 없는 것 %s' % sorted(set(block) - names))
    verdict('IMU_GYRO_CUTOFF 의 기본값은 yaml 의 40 Hz 다', gyro_cutoff_yaml() == 40.0, '%s' % gyro_cutoff_yaml())

    try:
        F.yaml_default('IMU_NO_SUCH_PARAM', F.ANALYSIS_PARAMS['IMU_GYRO_CUTOFF'])
        refused = False
    except LookupError:
        refused = True

    verdict('yaml 에 없는 파라미터는 짐작하지 않고 멈춘다', refused, '')
    values = [F.f32_decimal(float(np.float32(x))) for x in (0.7, 0.09, 15.0, 0.05, 1.0 / 3.0)]
    verdict('로그의 float32 는 같은 float32 를 가리키는 가장 짧은 십진수로: 0.699999988 -> 0.7',
            values[:4] == [0.7, 0.09, 15.0, 0.05] and np.float32(values[4]) == np.float32(1.0 / 3.0), str(values))


def test_replay_block():
    """throttle_counts, integrator_first_after_s, the pods' modes and the
    run_leg block."""
    log = synthetic_log(thrust=float(np.float32(0.031)))
    log.params['AS_YAWRATE_P'] = float(np.float32(0.7))         # as a ulog carries it
    _, plant, _, m = leg_of(log)
    rp, rl = m['replay'], m['replay']['run_leg']
    verdict('스로틀 계수: float32 0.0309999995 들의 계수는 31 이다 (int 로 자르면 30)',
            rp['throttle_counts'] == 31 and near(rp['throttle'], 0.031, 1e-8), '%s, %r' % (rp['throttle_counts'], rp['throttle']))
    moved = leg_of(synthetic_log(thrust=lambda t: PLAN_THRUST if t < 100.0 else 0.0))[3]['replay']
    verdict('한 계수에 머물지 않은 스로틀은 계수가 없고 (None), 그 평균은 난 적 없는 값이라 run_leg 도 없다',
            moved['throttle_counts'] is None and moved['run_leg'] is None and moved['run_leg_unread'] == ['throttle']
            and moved['throttle'] is not None, '%s, %s' % (moved['throttle_counts'], moved['run_leg_unread']))
    want_params = {k: P27[k] for k in ('AS_YAWRATE_I', 'AS_YR_INT_LIM', 'AS_YAWRATE_FF', 'AS_YAWRATE_MAX', 'MAN_DEADZONE')}
    want_params['AS_YAWRATE_P'] = 0.7       # the log's float32 0.7 above, back as its shortest decimal
    verdict('run_leg: Acro 시점의 FRD 상태, manual_s 0, 스틱 3 s 뒤 20 s, 놓음 120 s, 기록한 스로틀 그대로 (quantize False), '
            '적분기 0, 포드 틸트와 모드, 모터, 이득은 float32 의 십진수',
            rl is not None and rp['run_leg_unread'] == [] and rl['target'] == 'gz_' + VEC_ALL and rl['start'] == 'acro'
            and rl['manual_s'] == 0.0 and near(rl['centred_s'], 3.0, 1e-9) and near(rl['stick_s'], 20.0, 1e-9)
            and near(rl['release_s'], 120.0, 1e-9) and rl['quantize'] is False and rl['throttle'] == rp['throttle']
            and rl['stick'] == 1.0 and rl['integ0'] == 0.0 and rl['gyro_settled'] is True
            and (rl['u0'], rl['v0'], rl['r0'], rl['p0'], rl['phi0'])
            == tuple(rp['state'][k] for k in ('u', 'v', 'r_deg_s', 'p_deg_s', 'roll_deg'))
            and rl['pods0'] == [{'tilt': 0.0, 'mode': 'steering'}, {'tilt': 0.0, 'mode': 'steering'}]
            and rl['motors0'] == [0.1, 0.2] and rl['params'] == want_params
            and rp['params']['AS_YAWRATE_P'] == float(np.float32(0.7)) and 'FRD' in rp['run_leg_frame'],
            str(rl)[:300])
    held = leg_of(synthetic_log(thrust=lambda t: 0.03 if t < 30.0 else 0.015, torque_at_acro=0.01))[3]['replay']
    verdict('run_leg 의 포드 모드는 기록의 것이다: 0.015 에 요 0.01 이면 우현 released, 좌현 steering (짐작한 steering 이 아니다)',
            held['run_leg'] is not None and [p['mode'] for p in held['run_leg']['pods0']] == ['released', 'steering']
            == [p['mode'] for p in held['pod_modes']], str(held['run_leg'] and held['run_leg']['pods0']))
    crab = leg_of(synthetic_log(crab_deg=5.0))[3]['replay']['run_leg']
    verdict('run_leg 의 v0 는 FRD 다: 코보다 5 deg 우현으로 가면 +U sin 5',
            crab is not None and near(crab['v0'], U0 * math.sin(math.radians(5.0)), 1e-9), str(crab and crab['v0']))
    late = leg_of(synthetic_log(integ_late_ms=200))[3]
    verdict('재현 블록의 적분기 첫 표본에 그 시각이 붙는다 (Acro 뒤 0.2 s), 진입의 것과 같다',
            late['replay']['integrator_first_after_s'] == 0.2 == late['entry']['integrator_first_after_s']
            and late['replay']['integrator'] == late['entry']['integrator_first'], str(late['replay']['integrator_first_after_s']))
    carried = leg_of(synthetic_log(integ_start=0.05))[3]['replay']
    verdict('비지 않은 적분기 (0.05) 로 시작한 다리의 run_leg 은 없다 (None 이 run_leg 의 기본값 0 이 되지 않게)',
            carried['run_leg'] is None and carried['run_leg_unread'] == ['integ0'], str(carried['run_leg_unread']))
    cloud = leg_of(synthetic_log(tail=True), CS_ALL)[3]['replay']['run_leg']
    verdict('Cloudship 의 run_leg: 집합 포드 둘이 같은 틸트와 모드, 모터 셋째가 꼬리',
            cloud is not None and cloud['pods0'] == [{'tilt': 0.0, 'mode': 'steering'}] * 2
            and cloud['pods0'][0] is not cloud['pods0'][1] and cloud['motors0'] == [0.1, 0.2, 0.0]
            and cloud['target'] == 'gz_' + CS_ALL and cloud['params']['AS_YAWRATE_FF'] == 0.05, str(cloud and cloud['pods0']))
    unplanted = leg_of(synthetic_log(), None)[3]['replay']
    verdict('기체를 모르는 로그의 run_leg 은 없다 (target 을 읽지 못함)',
            unplanted['run_leg'] is None and 'target' in unplanted['run_leg_unread'], str(unplanted['run_leg_unread']))


def test_pod_modes():
    """AirshipPod::steer's mode, replayed on the logged demand."""
    params = lambda log: {k: log.param(k) for k in F.PARAMS}
    vec = F.plant_info(VEC_ALL, F.plan_params(VEC_ALL))
    cases = (('%g: 위' % PLAN_THRUST, synthetic_log(), [('steering', PLAN_THRUST)] * 2),
             ('0.03 에서 0.015: 띠에 위에서 들어와 유지', synthetic_log(thrust=lambda t: 0.03 if t < 30.0 else 0.015),
              [('holding', 0.015)] * 2),
             ('0.005 에서 0.015: 걸린 적 없이 띠', synthetic_log(thrust=lambda t: 0.005 if t < 30.0 else 0.015),
              [('released', 0.015)] * 2),
             ('0.03 에서 float32 0.02: 문턱과 같으면 위가 아니다',
              synthetic_log(thrust=lambda t: 0.03 if t < 30.0 else float(np.float32(0.02))), [('holding', 0.02)] * 2),
             ('0.015 에 요 토크 0.01: 우현 0.005 풀림, 좌현 0.025 조향',
              synthetic_log(thrust=lambda t: 0.03 if t < 30.0 else 0.015, torque_at_acro=0.01),
              [('released', 0.005), ('steering', 0.025)]))
    got = {}

    for name, log, want in cases:
        modes = F.pod_modes(log, vec, params(log), ACRO)
        got[name] = modes
        ok = modes is not None and [m for m, _ in modes] == [m for m, _ in want] \
            and all(near(d, w, 1e-6) for (_, d), (_, w) in zip(modes, want))
        verdict('포드 모드 (Acro 시점): %s' % name, ok, str(modes))

    cloud_log = synthetic_log(tail=True, thrust=lambda t: 0.03 if t < 30.0 else 0.015, torque_at_acro=0.01)
    cloud = F.plant_info(CS_ALL, F.plan_params(CS_ALL))
    modes = F.pod_modes(cloud_log, cloud, params(cloud_log), ACRO) or []
    verdict('Cloudship 의 집합 포드는 요를 나누지 않는다: 둘 다 0.015 로 유지',
            [m for m, _ in modes] == ['holding', 'holding'] and all(near(d, 0.015, 1e-6) for _, d in modes), str(modes))
    log = synthetic_log()
    before = F.pod_modes(log, vec, params(log), 9 * S) or []
    verdict('무장 전에는 parked', [m for m, _ in before] == ['parked', 'parked'], str(before))
    surfaces = F.pod_modes(log, vec, dict(params(log), CA_SV_CS_COUNT=2.0, CA_AIRSHIP_CS_K=1.0), ACRO)
    log.data['vehicle_torque_setpoint'].pop('xyz[0]')
    unread = F.pod_modes(log, vec, params(log), ACRO)
    verdict('조종면이 몫을 가져가거나 설정점이 기록에 없으면 모드를 짐작하지 않는다 (None)', surfaces is None and unread is None,
            '%s, %s' % (surfaces, unread))
    release = leg_of(synthetic_log())[3]['replay']
    verdict('놓을 때의 포드 모드: 토크 0.3 에서 우현 |%g - 0.3|, 좌현 %g, 둘 다 조향 (크기와 함께)' % (PLAN_THRUST, PLAN_THRUST + 0.3),
            [p['mode'] for p in release['pod_modes_at_release']] == ['steering', 'steering']
            and near(release['pod_modes_at_release'][0]['demand'], abs(PLAN_THRUST - 0.3), 1e-6)
            and near(release['pod_modes_at_release'][1]['demand'], PLAN_THRUST + 0.3, 1e-6)
            and [p['mode'] for p in release['pod_modes']] == ['steering', 'steering'], str(release['pod_modes_at_release']))


def test_fins_by_name():
    """dorsal/ventral by planar.py's rule, flow from behind marked no flow."""
    log = synthetic_log()
    _, plant, table, m = leg_of(log)
    _, com = gz_model.centre_of_mass(gz_model.family(VEC_ALL))
    names = [f['name'] for f in plant['fins']]
    verdict('2527 의 수직 핀: 1 번 dorsal (cp z %.2f > 무게 중심 %.2f), 2 번 ventral (%.2f), forward 는 몸 x'
            % (plant['fins'][0]['cp'][2], com[2], plant['fins'][1]['cp'][2]),
            names == ['dorsal', 'ventral'] and all(f['forward'] == (1.0, 0.0, 0.0) for f in plant['fins']), str(names))
    verdict('앞으로 나는 동안 흐름이 있고, q 0 이면 이름 붙은 받음각이 alpha_fin<n> 과 같다',
            np.all(table['fin_dorsal_flow'] == 1.0) and np.all(table['fin_ventral_flow'] == 1.0)
            and np.allclose(table['fin_dorsal_alpha_deg'], table['alpha_fin1_deg'], atol=1e-12)
            and np.allclose(table['fin_ventral_alpha_deg'], table['alpha_fin2_deg'], atol=1e-12), '')
    stick = m['stick_phase']['fins']
    rows = F.rows_between(table, ON, OFF)
    want = [float(np.max(np.abs(table['fin_%s_alpha_deg' % n][rows]))) / math.degrees(f['alpha_stall'])
            for n, f in zip(names, plant['fins'])]
    last = [abs(table['fin_%s_alpha_deg' % n][rows][-1]) > math.degrees(f['alpha_stall']) for n, f in zip(names, plant['fins'])]
    verdict('단계의 핀 항목: 이름, 흐름 있는 곳의 최대 |alpha|/실속, 흐름 없는 행 0, 끝의 갈래; 전의 항목은 그대로',
            [f['name'] for f in stick] == names and all(near(f['max_ratio'], w, 1e-12) for f, w in zip(stick, want))
            and all(f['no_flow_rows'] == 0 for f in stick)
            and [f['branch_end'] for f in stick] == ['stalled' if s else 'attached' for s in last]
            and all(set(f) >= {'max_abs_deg', 'stall_deg', 'stalled'} for f in stick), str(stick))
    back = synthetic_log(crab_deg=180.0)
    _, _, tb, mb = leg_of(back)
    sb = mb['stick_phase']['fins']
    n_stick = int(F.rows_between(tb, ON, OFF).sum())
    verdict('뒤로 가면 (흐름이 뒤에서) 흐름 0, 받음각 없음, 갈래 no flow; 전의 alpha_fin<n> 은 90 deg 를 넘어 실속으로 읽힌다',
            np.all(tb['fin_dorsal_flow'] == 0.0) and np.all(np.isnan(tb['fin_dorsal_alpha_deg']))
            and all(f['no_flow_rows'] == n_stick and f['max_ratio'] is None and f['branch_end'] == 'no flow' for f in sb)
            and all(f['stalled'] is True and f['max_abs_deg'] > 90.0 for f in sb), str(sb))
    # the gyro fast enough that the fins' incidence at the stick's end passes
    # both stalls by a quarter (on the 10.97 m hull, 1.5 x: 12.6 deg); the
    # onset's does not
    end_alpha = max(abs(table['fin_%s_alpha_deg' % n][rows][-1]) for n in names)
    factor = max(1.5, 1.25 * max(math.degrees(f['alpha_stall']) for f in plant['fins']) / end_alpha)
    faster = leg_of(synthetic_log(rate=lambda t: factor * r_true(t)))
    sf, tf = faster[3]['stick_phase']['fins'], faster[2]
    first = F.rows_between(tf, ON, OFF)
    verdict('끝에서 실속한 핀의 갈래는 stalled (스틱 첫 행에서는 붙어 있다), 최대 비는 1 을 넘는다',
            [f['branch_end'] for f in sf] == ['stalled', 'stalled'] and all(f['max_ratio'] > 1.0 for f in sf)
            and abs(tf['fin_dorsal_alpha_deg'][first][0]) < math.degrees(faster[1]['fins'][0]['alpha_stall']),
            str([(f['branch_end'], f['max_ratio']) for f in sf]))
    # nose up fast enough that the dorsal fin's cp moves backwards through
    # the air: a quarter past U0 / z (0.6 rad/s on the 10.97 m hull)
    q = max(0.6, 1.25 * U0 / plant['fins'][0]['cp'][2])
    pitched = synthetic_log(q_rad=q)
    _, pp, tq, _ = leg_of(pitched)
    z = [f['cp'][2] for f in pp['fins']]
    u_cp = [U0 - q * zi for zi in z]
    i = int(np.nonzero(F.rows_between(tq, ON, OFF))[0][-1])
    r_rad = math.radians(tq['r_deg_s'][i])
    want_v = math.degrees(math.atan2(r_rad * pp['fins'][1]['cp'][0], u_cp[1]))
    verdict('기수 들기 q %.2f rad/s: cp 의 앞 속도 u - q z 가 dorsal 에서 %.3f (흐름 없음), ventral 에서 %.3f m/s (흐름)'
            % (q, u_cp[0], u_cp[1]), u_cp[0] < 0.0 < u_cp[1] and np.all(tq['fin_dorsal_flow'] == 0.0)
            and np.all(tq['fin_ventral_flow'] == 1.0) and near(float(tq['fin_ventral_alpha_deg'][i]), want_v, 1e-9),
            '%s / %.6f' % (tq['fin_ventral_alpha_deg'][i], want_v))

    with tempfile.TemporaryDirectory() as out:
        doc, _ = F.analyze(log, VEC_ALL, out_dir=out, stamp='fins')

    verdict('요약의 수직 핀에 이름과 forward, upward, a0 가 있다', [f.get('name') for f in doc['plant']['vertical_fins']] == names
            and all(list(f['forward']) == [1.0, 0.0, 0.0] and list(f['upward']) == [0.0, 1.0, 0.0] and f['a0'] == 0.0
                    for f in doc['plant']['vertical_fins']), str(doc['plant']['vertical_fins'])[:200])


def fin_expectation(table, plant, pitch_deg, roll_deg, q_rad):
    """{fin name: (flow, alpha deg)} by a second route to leg_table's: the
    table's heading-frame u, v and w_down turned into the body by explicit
    Euler matrices for a constant pitch and roll, Rx(roll)^T Ry(pitch)^T,
    no quaternion algebra; the gyro's p, r from the table, q given; the
    lift-drag plugin's two early returns."""
    th, ph = math.radians(pitch_deg), math.radians(roll_deg)
    u, v, w = table['u'], table['v'], table['w_down']
    u1, w1 = math.cos(th) * u - math.sin(th) * w, math.sin(th) * u + math.cos(th) * w
    ub, vb, wb = u1, math.cos(ph) * v + math.sin(ph) * w1, -math.sin(ph) * v + math.cos(ph) * w1
    p, r = np.radians(table['p_deg_s']), np.radians(table['r_deg_s'])
    out = {}

    for f in plant['fins']:
        x, y, z = f['cp']
        ucp, vcp, wcp = ub + r * y - q_rad * z, vb + r * x + p * z, wb - p * y - q_rad * x
        flow = (ucp > 0.0) & (np.sqrt(ucp * ucp + vcp * vcp + wcp * wcp) > 0.01)
        out[f['name']] = (flow.astype(float), np.where(flow, np.degrees(np.arctan2(vcp, ucp)), np.nan))

    return out


def test_fins_body_frame():
    """The fins by name read the flow in the link's frame, as the lift-drag
    plugin does (review of 2026-10-02): the full attitude, the plugin's
    0.01 m/s return, and only for the axes its angle formula holds for."""
    log = synthetic_log(pitch_deg=-3.0, roll_deg=4.0, sink=0.3, crab_deg=10.0, q_rad=0.02)
    _, plant, table, _ = leg_of(log)
    want, level = fin_expectation(table, plant, -3.0, 4.0, 0.02), fin_expectation(table, plant, 0.0, 0.0, 0.02)
    same = all(np.array_equal(table['fin_%s_flow' % n], want[n][0])
               and np.allclose(table['fin_%s_alpha_deg' % n], want[n][1], atol=1e-9, equal_nan=True) for n in want)
    moved = max(float(np.nanmax(np.abs(want[n][1] - level[n][1]))) for n in want)
    verdict('이름 붙은 핀의 흐름은 몸 좌표다: 피치 -3, 롤 4 deg, 가라앉음 0.3 m/s 에서 행마다 오일러 행렬로 따로 낸 값과 같다 '
            '(헤딩 좌표로 읽으면 받음각이 %.2f deg 다르다)' % moved, same and moved > 0.1,
            '%s' % {n: float(np.nanmax(np.abs(table['fin_%s_alpha_deg' % n] - want[n][1]))) for n in want})
    # the groundtruth's quaternion is not of unit length (GZBridge's
    # q_ENU_to_NED takes 0.70711: |q| 1.0000045); the rotation is its
    # direction's. Here 1.001, so that a rotation built on the raw q shows
    scaled = synthetic_log(pitch_deg=-3.0, roll_deg=4.0, sink=0.3, crab_deg=10.0, q_rad=0.02)
    att = scaled.data[F.GT_ATT]

    for i in range(4):
        att['q[%d]' % i] = att['q[%d]' % i] * 1.001

    ts = leg_of(scaled)[2]
    off = max(float(np.nanmax(np.abs(ts['fin_%s_alpha_deg' % n] - table['fin_%s_alpha_deg' % n]))) for n in want)
    verdict('자세 쿼터니언의 길이가 1 이 아니어도 (1.001) 몸 좌표의 흐름은 같다: 회전은 정규화한 방향의 것',
            off < 1e-9 and all(np.array_equal(ts['fin_%s_flow' % n], table['fin_%s_flow' % n]) for n in want), '%.3g deg' % off)
    # nose 10 deg up, sinking 0.5 m/s at 0.05 m/s forward: the heading frame's
    # u is positive, the body's u 0.05 cos 10 - 0.5 sin 10 = -0.038 is not
    _, pc, tc, mc = leg_of(synthetic_log(pitch_deg=10.0, sink=0.5, speed=0.05))
    n_stick = int(F.rows_between(tc, ON, OFF).sum())
    verdict('기수를 10 deg 든 채 0.5 m/s 로 가라앉으면 헤딩 좌표의 u 는 양이어도 몸 x 의 흐름은 뒤에서 온다: 흐름 없음',
            np.all(tc['u'] > 0.0) and all(np.all(tc['fin_%s_flow' % f['name']] == 0.0) for f in pc['fins'])
            and all(f['no_flow_rows'] == n_stick and f['branch_end'] == 'no flow' for f in mc['stick_phase']['fins']),
            str([(f['name'], f['no_flow_rows']) for f in mc['stick_phase']['fins']]))
    # 0.008 m/s forward: no flow on the entry (the plugin's 0.01 m/s), flow
    # in the stick once r x_cp adds to it
    _, ps, ts, _ = leg_of(synthetic_log(speed=0.008))
    entry, stick = F.rows_between(ts, ACRO, ON), F.rows_between(ts, ON, OFF)
    slow = fin_expectation(ts, ps, 0.0, 0.0, 0.0)
    verdict('0.01 m/s 이하의 흐름에는 플러그인이 힘을 주지 않는다: 0.008 m/s 로 앞으로 가는 진입은 흐름 없음, 스틱 끝에서는 흐름 '
            '(행마다 따로 낸 값과 같다)',
            np.all(ts['u'][entry] > 0.0) and all(np.all(ts['fin_%s_flow' % n][entry] == 0.0)
                                                 and ts['fin_%s_flow' % n][stick][-1] == 1.0
                                                 and np.array_equal(ts['fin_%s_flow' % n], slow[n][0]) for n in slow), '')
    # the same with 0.006 m/s of sink and q 0.001 rad/s: w_cp = w - q x_cp
    # (x_cp -4.29) is 0.0103 m/s, which takes both fins past 0.01 m/s; the
    # opposite sign (0.0017) or no w would leave both under it
    _, pw, tw, _ = leg_of(synthetic_log(speed=0.008, sink=0.006, q_rad=0.001))
    entry = F.rows_between(tw, ACRO, ON)
    sunk = fin_expectation(tw, pw, 0.0, 0.0, 0.001)
    verdict('흐름의 크기에는 아래쪽 성분 w_cp = w - p y - q x 가 든다: 0.006 m/s 가라앉고 q 0.001 이면 진입에서 두 핀 다 흐름',
            all(np.all(tw['fin_%s_flow' % n][entry] == 1.0) and np.array_equal(tw['fin_%s_flow' % n], sunk[n][0])
                for n in sunk), '')
    # an angle formula the plugin does not have: not read
    log = synthetic_log()
    leg = F.find_legs(log)[0]
    params = {k: log.param(k) for k in F.PARAMS + tuple(F.ANALYSIS_PARAMS)}
    odd = F.plant_info(VEC_ALL, params)
    odd['fins'][0]['a0'], odd['fins'][1]['upward'] = 0.1, None
    to = F.leg_table(log, leg, odd)
    mo = F.leg_metrics(log, leg, to, odd, params, target=2.0)
    verdict('a0 가 0 이 아니거나 upward 가 +y 가 아닌 (radial_symmetry 로 None 인) 핀은 읽지 않는다: 빈 열, 단계 항목 None',
            all(np.all(np.isnan(to['fin_%s_%s' % (n, c)])) for n in ('dorsal', 'ventral') for c in ('alpha_deg', 'flow'))
            and all(f['max_ratio'] is None and f['no_flow_rows'] is None and f['branch_end'] is None
                    for f in mo['stick_phase']['fins']), str(mo['stick_phase']['fins'])[:200])
    element = lambda text, a0=None: {'elem': ET.fromstring('<plugin>%s</plugin>' % text), 'upward': (0.0, 1.0, 0.0),
                                     'a0': a0}
    axes = [F.fin_axes(element('<forward>1 0 0</forward>')),
            F.fin_axes(element('<forward>1 0 0</forward><radial_symmetry>true</radial_symmetry>')),
            F.fin_axes(element('<forward>1 0 0</forward><radial_symmetry>false</radial_symmetry>', a0=0.05)),
            F.fin_axes(element(''))]
    verdict('플러그인의 축: a0 가 없으면 플러그인 기본값 0, radial_symmetry 면 upward 는 흐름이 정해서 None, forward 가 없으면 None',
            axes == [{'forward': (1.0, 0.0, 0.0), 'upward': (0.0, 1.0, 0.0), 'a0': 0.0},
                     {'forward': (1.0, 0.0, 0.0), 'upward': None, 'a0': 0.0},
                     {'forward': (1.0, 0.0, 0.0), 'upward': (0.0, 1.0, 0.0), 'a0': 0.05},
                     {'forward': None, 'upward': (0.0, 1.0, 0.0), 'a0': 0.0}], str(axes))
    # the names: planar.py's rule, which forward.fin_names repeats, on all
    # four plants, never shared there; a shared one would be numbered
    rule = "'dorsal' if f['cp'][2] > com[2] else 'ventral'"
    named = {}

    for model in F.ALLOWED:
        _, com = gz_model.centre_of_mass(gz_model.family(model))
        plant = F.plant_info(model, F.plan_params(model))
        plain = ['dorsal' if f['cp'][2] > com[2] else 'ventral' for f in gz_model.vertical_fins(model)]
        # a name two fins share is numbered (the JAXA hull: its heavy gondola
        # puts the centre of mass below the lower fin's cp, so both are
        # 'dorsal' by the rule)
        want = [n + ('%d' % (plain[:i].count(n) + 1) if plain.count(n) > 1 else '') for i, n in enumerate(plain)]
        named[model] = ([f['name'] for f in plant['fins']], want, [(f['forward'], f['upward'], f['a0']) for f in plant['fins']])

    verdict('핀 기체의 핀 이름은 planar.py 와 forward.fin_names 의 규칙 그대로이고 겹치는 이름은 번호가 붙는다, '
            '축은 모두 forward +x, upward +y, a0 0',
            rule in model_source('planar.py') and rule in model_source('forward.py')
            and all(got == want and len(set(got)) == len(got)
                    and all(a == ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), 0.0) for a in ax) for got, want, ax in named.values()),
            str({m: v[0] for m, v in named.items()}))
    real = gz_model.vertical_fins

    try:
        gz_model.vertical_fins = lambda model: real(model) + real(model)[:1]
        shared = [f['name'] for f in F.plant_info(VEC_ALL, F.plan_params(VEC_ALL))['fins']]
    finally:
        gz_model.vertical_fins = real

    verdict('한쪽에 핀이 둘이면 (네 기체에는 없다) 각자 파일 순서 번호를 받아 CSV 열이 둘이 된다', shared == ['dorsal1', 'ventral', 'dorsal3'],
            str(shared))


def test_tail_torque_flips():
    log = synthetic_log(tail=True, torque_blip=True)
    s = leg_of(log, CS_ALL)[3]['stick_phase']['reversals']
    plain = leg_of(synthetic_log(tail=True), CS_ALL)[3]
    verdict('Cloudship: 60 s 의 40 ms 토크 반전은 토크 설정 (20 ms) 의 세기에 두 번, actuator_motors (100 ms) 꼬리 세기에는 없다',
            s['tail_flips_torque'] == 2 and s['tail_flips'] == 0 and s['flips'] == 0, str(s))
    verdict('각 세기는 제 신호로: 반전 없는 토크는 0 번, 놓음 중 꼬리 명령은 1 번 (합성 로그의 꼬리는 토크와 따로 움직인다)',
            plain['stick_phase']['reversals']['tail_flips_torque'] == 0
            and plain['release']['reversals']['tail_flips_torque'] == 0 and plain['release']['reversals']['tail_flips'] == 1,
            str(plain['release']['reversals']))
    # -0.05 from 30 s to the onset, then the stick's rising torque: the
    # direction before the phase carries in, so leaving the band upward at
    # 43.7 s is a flip, as forward.py's trigger counts it
    carried = leg_of(synthetic_log(tail=True, torque_at_acro=-0.05), CS_ALL)[3]['stick_phase']['reversals']
    verdict('토크 세기는 단계 전의 방향을 이어받는다: 스틱 전 -0.05 에서 스틱의 + 로 가면 한 번',
            carried['tail_flips_torque'] == 1 and carried['tail_flips'] == 0, str(carried))
    vec = leg_of(synthetic_log())[3]['stick_phase']['reversals']
    verdict('벡터 추력 기체에는 꼬리 세기가 없다', 'tail_flips_torque' not in vec and 'tail_flips' not in vec, str(sorted(vec)))


def test_rear_s_last_interval():
    t = np.arange(0, 5, dtype=np.int64) * S
    rear_only = F.pod_reversals(t, np.full(5, 180.0), int(0.5 * S), int(2.5 * S))
    swing = F.pod_reversals(t, np.array([0.0, 0.0, 180.0, 180.0, 180.0]), 0, int(4.5 * S))
    verdict('뒤쪽 시간은 구간 끝까지: 0.5..2.5 s 내내 뒤쪽이면 2.0 s, 2 s 에 뒤로 가 4.5 s 까지면 2.5 s (마지막 표본 뒤 포함)',
            rear_only == (0, 0, 2.0) and swing[:2] == (1, 0) and near(swing[2], 2.5, 1e-12), '%s / %s' % (rear_only, swing))
    rv = leg_of(synthetic_log())[3]['stick_phase']['reversals']['stbd']
    verdict('합성 스틱: 우현은 44.6 s (100 ms 서보가 120 deg 를 넘은 표본) 부터 놓는 63 s 까지 뒤쪽, 18.4 s (전에는 18.38)',
            rv['rear_s'] == 18.4 and rv['into_rear'] == 1, str(rv))


def bump(t):
    """r_true plus a 1 s triangle of 3 deg/s at 50 s: a peak a window's
    length can be seen in."""
    return r_true(t) + 3.0 * max(0.0, 1.0 - abs(t - 50.0))


def window_extreme(log, pick=max):
    """(mean, end s after the onset) of the trailing 0.5 s gyro window, (t -
    0.5 s, t] as yaw_model/forward.py's 125 steps of 4 ms are, wholly inside
    the stick, that `pick` chooses; by brute force."""
    tg = log.data['vehicle_angular_velocity']['timestamp']
    rg = np.degrees(log.data['vehicle_angular_velocity']['xyz[2]'])
    found = []

    for j in np.nonzero((tg >= ON + 500_000) & (tg < OFF))[0]:
        found.append((float(np.mean(rg[(tg > tg[j] - 500_000) & (tg <= tg[j])])), (int(tg[j]) - ON) / S))

    return pick(found, key=lambda x: x[0])


def test_smoothed_peak():
    log = synthetic_log(rate=bump)
    s = leg_of(log)[3]['stick_phase']
    best = window_extreme(log)
    verdict('0.5 s 시간 창 평활 최대: 스틱 안 모든 0.5 s 창의 자이로 평균 가운데 최대, 그 창의 끝 (%.3f deg/s, %.3f s)' % best,
            near(s['r_peak_smoothed_05s_deg_s'], best[0], 1e-9) and near(s['r_peak_smoothed_05s_after_s'], best[1], 1e-9)
            and s['r_peak_smoothed_05s_deg_s'] < s['r_peak_deg_s'],
            '%s at %s' % (s['r_peak_smoothed_05s_deg_s'], s['r_peak_smoothed_05s_after_s']))
    s25 = leg_of(synthetic_log(rate=bump, gt_ms=25))[3]['stick_phase']
    verdict('그라운드트루스가 25 ms 이면 25 행 평활 최대는 0.6 s 창이 되어 바뀌고, 시간 창의 값은 그대로다',
            s25['r_peak_smoothed_05s_deg_s'] == s['r_peak_smoothed_05s_deg_s']
            and abs(s25['r_peak_smoothed_deg_s'] - s['r_peak_smoothed_deg_s']) > 0.02,
            '행 %.5f / %.5f, 시간 창 %.5f / %.5f' % (s25['r_peak_smoothed_deg_s'], s['r_peak_smoothed_deg_s'],
                                                s25['r_peak_smoothed_05s_deg_s'], s['r_peak_smoothed_05s_deg_s']))
    # a 0.1 s spike of 10 deg/s at the onset: a window reaching back before
    # the onset would hold little else; one wholly inside the stick dilutes it
    spike = synthetic_log(rate=lambda t: 10.0 if ON / S <= t < ON / S + 0.1 else r_true(t))
    ss = leg_of(spike)[3]['stick_phase']
    verdict('창은 모두 스틱 안: 스틱 시작의 0.1 s 10 deg/s 는 0.5 s 창에서 묽어진다 (r_true 의 끝 근처가 최대)',
            ss['r_peak_smoothed_05s_deg_s'] < 4.0 and ss['r_peak_smoothed_05s_after_s'] > 19.0, str(ss['r_peak_smoothed_05s_deg_s']))
    # a left stick: the peak is the most negative window, here the first full
    # one after the onset (the synthetic rate does not follow the stick's sign)
    left_log = synthetic_log(stick=-1.0, rate=bump)
    left = leg_of(left_log)[3]['stick_phase']
    worst = window_extreme(left_log, min)
    verdict('왼쪽 스틱에서는 스틱 쪽 (음) 으로 가장 먼 창, 곧 평균이 가장 작은 창 (%.4f deg/s, %.3f s)' % worst,
            near(left['r_peak_smoothed_05s_deg_s'], worst[0], 1e-9) and near(left['r_peak_smoothed_05s_after_s'], worst[1], 1e-9),
            '%s at %s' % (left['r_peak_smoothed_05s_deg_s'], left['r_peak_smoothed_05s_after_s']))
    # on a 10 ms gyro a sample lies exactly 0.5 s before each window's end:
    # the window leaves it out, as the model's 125 steps leave out the 126th
    # (on SITL's 8 ms none does, so the two ends cannot differ there)
    ten = synthetic_log(rate=bump, gyro_ms=10)
    s10, want10 = leg_of(ten)[3]['stick_phase'], window_extreme(ten)
    verdict('10 ms 자이로: 창은 (t - 0.5 s, t], 0.5 s 앞의 표본은 빼고 50 개 (%.4f deg/s, %.3f s)' % want10,
            near(s10['r_peak_smoothed_05s_deg_s'], want10[0], 1e-9) and near(s10['r_peak_smoothed_05s_after_s'], want10[1], 1e-9),
            '%s at %s' % (s10['r_peak_smoothed_05s_deg_s'], s10['r_peak_smoothed_05s_after_s']))
    log = synthetic_log()
    av = log.data['vehicle_angular_velocity']
    av['xyz[2]'] = np.where(av['timestamp'] == 50 * S, np.nan, av['xyz[2]'])
    verdict('읽지 못한 자이로 표본이 있으면 시간 창 평활 최대도 없다 (None)',
            leg_of(log)[3]['stick_phase']['r_peak_smoothed_05s_deg_s'] is None, '')


def test_csv_and_definitions():
    with tempfile.TemporaryDirectory() as out:
        F.analyze(synthetic_log(), VEC_ALL, out_dir=out, stamp='cols')

        with open(os.path.join(out, 'forward_%s_cols_leg1.csv' % VEC_ALL)) as handle:
            rows = list(csv.reader(handle))

    header = rows[0]
    verdict('CSV 의 열: 전의 열은 같은 자리, 새 열 (t_acro_s, 핀 이름의 받음각과 흐름) 은 그 뒤',
            header == OLD_COLUMNS_VECTORED + NEW_COLUMNS, str(header[len(OLD_COLUMNS_VECTORED):]))
    i_s, i_a = header.index('t_s'), header.index('t_acro_s')
    gaps = [float(r[i_a]) - float(r[i_s]) for r in rows[1:]]
    verdict('t_acro_s 는 Acro 전환부터의 시각: t_s + 3 s (스틱이 Acro 3 s 뒤)', all(abs(g - 3.0) < 1e-6 for g in gaps),
            '%.6f..%.6f' % (min(gaps), max(gaps)))
    d = F.definitions()
    marks = d['release marks']
    verdict("놓음 표시의 정의가 forward.py 의 at() 처럼 읽는다고 하지 않고, 모델은 1 s 평균과 비교한다고 한다",
            "as forward.py's at() reads" not in marks and 'not what' in marks and 'r_mean1s_deg_s' in marks, marks[:100])
    added = ('smoothed peak', 'tail flips (torque)', 'rear_s', 'fins by name', 'integrator first', 'throttle counts',
             'pod modes', 'run_leg', 't_acro_s', 'IMU_GYRO_CUTOFF')
    verdict('전의 정의는 그대로, 새 필드마다 정의가 있다',
            all(k in d for k in ('reversals', 'release marks', 'means', 'states', 'flags') + added),
            str([k for k in added if k not in d]))
    fins, run_leg, imu = d['fins by name'], d['run_leg'], d['IMU_GYRO_CUTOFF']
    verdict('정의가 2026-10-02 의 것을 말한다: 핀은 전체 자세의 몸 좌표와 0.01 m/s, run_leg 은 판정과 둘째 스틱과 오류로 None, '
            'IMU_GYRO_CUTOFF 판정은 종료를 가른다',
            'full groundtruth attitude' in fins and 'faster than 0.01 m/s' in fins and 'Until 2026-10-02' in fins
            and "'verdict: '" in run_leg and 'extra_sticks' in run_leg and "'error: '" in run_leg
            and 'exits 1' in imu and 'gates the run' in imu, '')


def test_analyze_keeps_live():
    """--analyze --events rewrites a run's summary with the 'live' block the
    run wrote beside it (main_live): the events file, the abort and the plan
    from that file, and the failed writes from the summary it replaces when
    that is the same log's, else None."""
    import report

    plan = dict(F.vehicle_plan(VEC_ALL)['legs'][1], speed=2.0, stick=1.0)
    events = {'acro': (ACRO - 30_000, ACRO - 10_000), 'stick_on': (ON - 60_000, ON - 40_000),
              'stick_off': (OFF - 60_000, OFF - 40_000), 'release_end': (MANUAL2 - 20_000, MANUAL2 + 5_000)}
    run_plan = {'model': VEC_ALL, 'legs': [plan]}
    live = {'model': VEC_ALL, 'aborted': None, 'plan': run_plan, 'legs': [{'leg': 1, 'flown': True, 'plan': plan, 'events': events}]}
    kept, real = list(report._results), F.Log.read
    found = {}

    with tempfile.TemporaryDirectory() as tmp:
        stem = os.path.join(tmp, 'forward_%s_20261001_120000' % VEC_ALL)
        ulg, ev, out = stem + '.ulg', stem + '_events.json', os.path.join(tmp, 'out')
        summary_path = os.path.join(out, os.path.basename(stem) + '_summary.json')
        os.makedirs(out)

        with open(ulg, 'wb') as handle:
            handle.write(b'ULog stand-in')

        with open(ev, 'w') as handle:
            json.dump(live, handle)

        def read(cls, path):
            log = synthetic_log()
            log.path = path
            return log

        F.Log.read = classmethod(read)

        try:
            for case, previous in (('none', None), ('same log', {'log': ulg, 'stamp': '20261001_120000'}),
                                   ('other log', {'log': ulg + '.other', 'stamp': '20261001_120000'})):
                if previous is not None:
                    with open(summary_path, 'w') as handle:
                        json.dump(dict(previous, live={'events_write_failures': ['disk full']}), handle)
                elif os.path.exists(summary_path):
                    os.remove(summary_path)

                report._results.clear()         # main_analyze's own verdicts, its exit code from them alone
                code, _ = run_main(['--analyze', ulg, '--events', ev, '--out', out])

                with open(summary_path) as handle:
                    found[case] = (code, json.load(handle).get('live'))
        finally:
            F.Log.read = real
            report._results[:] = kept

    want = {'events': ev, 'aborted': None, 'plan': json.loads(json.dumps(run_plan))}
    verdict('--analyze --events 의 요약에도 실행이 쓴 live 블록이 있다: 사건 파일, 중단, 계획; 쓰기 실패는 같은 로그의 전 요약에서, '
            '없으면 None',
            all(code == 0 for code, _ in found.values()) and found['none'][1] == dict(want, events_write_failures=None)
            and found['same log'][1] == dict(want, events_write_failures=['disk full'])
            and found['other log'][1] == dict(want, events_write_failures=None),
            str({k: (c, v and v.get('events_write_failures')) for k, (c, v) in found.items()}))


IMU_VERDICT = '로그의 IMU_GYRO_CUTOFF 가 모델이 쓰는 yaml 기본값이다'


def test_imu_cutoff():
    outcome = {}

    with tempfile.TemporaryDirectory() as out:
        for name, value in (('yaml', 'yaml'), ('100 Hz', 100.0), ('없음', None)):
            doc, checks = F.analyze(synthetic_log(imu_cutoff=value), VEC_ALL, out_dir=out, stamp='imu')
            rp = doc['legs'][0]['replay']
            outcome[name] = ([ok for n, ok, _ in checks if 'IMU_GYRO_CUTOFF' in n], doc['params'].get('IMU_GYRO_CUTOFF'),
                             rp['params'].get('IMU_GYRO_CUTOFF'), rp['run_leg'] is not None, rp['run_leg_unread'])

    refused = ['verdict: ' + IMU_VERDICT]
    verdict('IMU_GYRO_CUTOFF: yaml 의 40 이면 통과, 100 (4006 처럼 에어프레임이 바꾼 값) 이나 없으면 실패하고 그 실행의 다리에는 '
            'run_leg 이 없다; 요약과 재현의 params 에 있다',
            outcome == {'yaml': ([True], 40.0, 40.0, True, []), '100 Hz': ([False], 100.0, 100.0, False, refused),
                        '없음': ([False], None, None, False, refused)}, str(outcome))
    # a yaml that does not read for another reason than a missing name fails
    # the verdict with that reason; the analysis goes on
    log, real = synthetic_log(), F.yaml_default

    def unreadable(name, rel):
        raise ValueError('yaml 이 깨졌다 (시험)')

    F.yaml_default = unreadable

    try:
        with tempfile.TemporaryDirectory() as out:
            doc, checks = F.analyze(log, VEC_ALL, out_dir=out, stamp='yaml')
    finally:
        F.yaml_default = real

    imu = [(ok, d) for n, ok, d in checks if n == IMU_VERDICT]
    verdict('yaml 이 어떤 까닭으로든 읽히지 않으면 IMU_GYRO_CUTOFF 판정이 그 까닭과 함께 실패하고 분석은 끝난다',
            len(imu) == 1 and imu[0][0] is False and 'ValueError: yaml 이 깨졌다' in imu[0][1]
            and doc['legs'][0]['replay']['run_leg'] is None, str(imu))
    # the gate the module docstring states: a run flown with another cutoff
    # exits 1 on that verdict alone, its flight otherwise valid
    code, _, fake, recorded, _ = fake_live_run(VEC_ALL, '1', log_params={'IMU_GYRO_CUTOFF': 100.0})
    doc = json.loads(fake.summary_text) if fake.summary_text else {'legs': []}
    verdict('다른 IMU_GYRO_CUTOFF 로 난 실행은 비행이 타당해도 종료 1 이다 (실패한 판정은 그것 하나), 그 다리의 run_leg 은 없다',
            code == 1 and [n for n, ok in recorded if not ok] == [IMU_VERDICT] and doc['legs']
            and all(m['replay']['run_leg'] is None and 'verdict: ' + IMU_VERDICT in m['replay']['run_leg_unread']
                    for m in doc['legs']), '종료 %s, 실패 %s' % (code, [n for n, ok in recorded if not ok]))


def test_run_leg_refusals():
    """run_leg whole for a leg a replay may fly as it is, else None with the
    reason (review of 2026-10-02): a failed verdict of the leg or of the
    run, a second stick inside the replayed span, an error in the replay's
    inputs."""
    low_log, gap_log = synthetic_log(), synthetic_log()
    gt = low_log.data[F.GT_POS]
    gt['z'] = np.where(gt['timestamp'] >= 150 * S, -4.0, gt['z'])
    av = gap_log.data['vehicle_angular_velocity']
    keep = (av['timestamp'] < 70 * S) | (av['timestamp'] >= 72 * S)
    gap_log.data['vehicle_angular_velocity'] = {k: v[keep] for k, v in av.items()}

    with tempfile.TemporaryDirectory() as out:
        doc, checks = F.analyze(synthetic_log(), VEC_ALL, out_dir=out, stamp='whole')
        whole = doc['legs'][0]['replay']
        low = F.analyze(low_log, VEC_ALL, out_dir=out, stamp='low')[0]['legs'][0]['replay']
        gap = F.analyze(gap_log, VEC_ALL, out_dir=out, stamp='gap')[0]['legs'][0]['replay']

    # one leg's failure is its own: two flown legs matched to the log's one
    # (the same stick bracket), the first with its Acro bracket 3 s off, so
    # its timeline verdict fails; the second passes every verdict
    plan = dict(F.vehicle_plan(VEC_ALL)['legs'][1], speed=2.0, stick=1.0)
    events = {'acro': (ACRO - 30_000, ACRO - 10_000), 'stick_on': (ON - 60_000, ON - 40_000),
              'stick_off': (OFF - 60_000, OFF - 40_000), 'release_end': (MANUAL2 - 20_000, MANUAL2 + 5_000)}
    two = {'model': VEC_ALL, 'legs': [
        {'leg': 1, 'flown': True, 'plan': plan, 'events': dict(events, acro=(ACRO - 3 * S, ACRO - 2 * S))},
        {'leg': 2, 'flown': True, 'plan': plan, 'events': events}]}

    with tempfile.TemporaryDirectory() as out:
        doc2, checks2 = F.analyze(synthetic_log(), VEC_ALL, live=two, out_dir=out, stamp='two')

    timeline = 'verdict: 다리 1: 로그의 모드와 스틱 시각이 스크립트가 잰 PX4 시각과 맞는다'
    first, second = (m['replay'] for m in doc2['legs'])
    verdict('한 다리의 실패한 판정은 다른 다리의 run_leg 을 막지 않는다: 시각이 어긋난 다리 1 은 None, 모두 통과한 다리 2 는 있다',
            [n for n, ok, _ in checks2 if not ok] == [timeline[len('verdict: '):]]
            and first['run_leg'] is None and first['run_leg_unread'] == [timeline]
            and second['run_leg'] is not None and second['run_leg_unread'] == [],
            '%s / %s' % (first['run_leg_unread'], second['run_leg_unread']))
    airborne = 'verdict: 다리 1: 다리 내내 무장 상태로 떠 있었다'
    topics = 'verdict: 필요한 토픽이 필요한 속도로 있고 로그가 끊기지 않았다'
    verdict('모든 판정이 통과한 다리만 run_leg 이 있다: 놓음 중 4 m 로 내려온 다리 (제 판정) 와 다리 안에서 자이로가 2 s 빠진 실행 '
            '(실행의 판정) 은 None, 까닭은 실패한 판정의 이름',
            all(ok for _, ok, _ in checks) and whole['run_leg'] is not None and whole['run_leg_unread'] == []
            and low['run_leg'] is None and low['run_leg_unread'] == [airborne]
            and gap['run_leg'] is None and topics in gap['run_leg_unread']
            and all(x.startswith('verdict: ') for x in gap['run_leg_unread']),
            '%s / %s' % (low['run_leg_unread'], gap['run_leg_unread']))
    # a second stick at 100 s, inside the release (63..183 s); one at 150 s
    # with the release cut at 60 s (63..123 s), past it
    inside = leg_of(synthetic_log(second_stick=(100.0, 102.0)))[3]
    late_log = synthetic_log(second_stick=(150.0, 152.0))
    leg = F.find_legs(late_log, release_s=60.0)[0]
    params = {k: late_log.param(k) for k in F.PARAMS + tuple(F.ANALYSIS_PARAMS)}
    plant = F.plant_info(VEC_ALL, params)
    late = F.leg_metrics(late_log, leg, F.leg_table(late_log, leg, plant), plant, params, target=2.0, release_s=60.0)
    verdict('놓음이 끝나기 전에 시작한 둘째 스틱은 run_leg 을 없앤다 (까닭 extra_sticks), 놓음 끝 뒤의 것은 아니다; 둘 다 extra_sticks 1',
            inside['extra_sticks'] == 1 and inside['replay']['run_leg'] is None
            and inside['replay']['run_leg_unread'] == ['extra_sticks']
            and late['extra_sticks'] == 1 and late['replay']['run_leg'] is not None and late['replay']['run_leg_unread'] == [],
            '%s / %s' % (inside['replay']['run_leg_unread'], late['replay']['run_leg_unread']))
    # an error in the replay's inputs leaves the leg's record, those fields
    # None and the error named, and the analysis writes its summary
    log, real = synthetic_log(), F.pod_modes

    def broken(*args, **kwargs):
        raise RuntimeError('시험')

    F.pod_modes = broken

    try:
        errored = leg_of(log)[3]

        with tempfile.TemporaryDirectory() as out:
            doc, _ = F.analyze(log, VEC_ALL, out_dir=out, stamp='err')
            written = os.path.exists(os.path.join(out, 'forward_%s_err_leg1.csv' % VEC_ALL))
    finally:
        F.pod_modes = real

    rp = errored['replay']
    verdict('재현 입력의 오류는 다리 기록을 잃게 하지 않는다: 그 필드는 None, run_leg 은 None 에 오류 이름, 측정 값과 CSV 는 그대로',
            rp['run_leg'] is None and rp['run_leg_unread'] == ['error: RuntimeError: 시험'] and rp['pod_modes'] is None
            and rp['pod_modes_at_release'] is None and rp['throttle_counts'] is None
            and rp['integrator_first_after_s'] == errored['entry']['integrator_first_after_s']
            and rp['run_leg_frame'] == F.RUN_LEG_FRAME and errored['stick_phase']['r_mean_last5_deg_s'] is not None
            and doc['legs'][0]['replay']['run_leg_unread'] == ['error: RuntimeError: 시험'] and written,
            str(rp['run_leg_unread']))


# ---------------------------------------------------------------------------
# 4. the gate logs of 2026-09-29
# ---------------------------------------------------------------------------

# chain14_20260929.txt, each gate's Acro step: '지시 6.67, 마지막 3 초 평균 m, 최대 p deg/s;
# 출발 r0 deg/s, 적분기 i0' and '... s 만에 정지'; its live samples in
# samples_<target>_chain14.csv
GATE_LOGS = {
    '2026-09-29/03_05_52.ulg': {'model': VEC_ALL, 'mean3': 7.10, 'peak': 7.24, 'start': -0.32, 'integ': 0.001,
                                'samples': 'samples_gz_airship_vectored_independent_am_fin_drag_chain14.csv'},
    '2026-09-29/03_12_03.ulg': {'model': VEC_FIN, 'mean3': 7.05, 'peak': 7.10, 'start': -0.07, 'integ': 0.000,
                                'samples': 'samples_gz_airship_vectored_independent_fin_chain14.csv'},
}


def gate_samples(path):
    """The 'Acro 추종' rows of a gate's live samples: (t wall s, rate, torque, integ)."""
    rows = []

    with open(path) as handle:
        for row in csv.DictReader(handle):
            if row['phase'] == 'Acro 추종':
                rows.append(tuple(float(row[k]) for k in ('t', 'rate', 'torque', 'integ')))

    return rows


def test_gate_logs():
    found = 0

    for rel, ref in GATE_LOGS.items():
        path = os.path.join(LOG_ROOT, rel)

        if not os.path.exists(path):
            note('  건너뜀: %s 가 없다 (SDLOG_DIRS_MAX 가 오래된 로그를 지운다)' % rel)
            continue

        found += 1
        log = F.Log.read(path)
        label = '%s (%s)' % (rel, gz_model.airframe(ref['model'])[0])
        verdict('%s: 로그의 SYS_AUTOSTART 와 펌웨어가 HEAD 의 것이다' % label,
                F.model_of(log) == ref['model'] and str(log.info.get('ver_sw', '')).startswith('4e1baea97c'),
                '%s %s' % (log.param('SYS_AUTOSTART'), log.info.get('ver_sw', '')[:10]))
        legs = F.find_legs(log, release_s=12.0)
        verdict('%s: Manual -> Acro 에 스틱이 걸린 다리 하나 (제동과 바람 Acro 는 스틱이 없다)' % label, len(legs) == 1,
                '%d 개' % len(legs))

        if len(legs) != 1:
            continue

        leg = legs[0]
        params = {k: log.param(k) for k in F.PARAMS}
        plant = F.plant_info(ref['model'], params)
        table = F.leg_table(log, leg, plant)
        m = F.leg_metrics(log, leg, table, plant, params)
        d = m['durations_s']
        verdict('%s: 진입 1-2 s, 스틱 35 s (게이트의 clean_start 와 추종 창)' % label,
                1.0 <= d['entry'] <= 2.0 and abs(d['stick'] - 35.0) < 0.5, '진입 %.2f, 스틱 %.2f s' % (d['entry'], d['stick']))
        verdict('%s: 스틱 0.5, 지시 6.667 deg/s' % label,
                near(m['stick']['value'], 0.5, 1e-6) and near(m['stick']['setpoint_deg_s'], 6.6667, 1e-3),
                '%s, %s' % (m['stick']['value'], m['stick']['setpoint_deg_s']))
        off = leg['off_us']
        mean3 = F.gyro_mean(log, off - 3 * S, off)
        verdict('%s: 스틱 마지막 3 s 평균 각속도가 게이트가 출력한 %.2f 다' % (label, ref['mean3']), near(mean3, ref['mean3'], 0.15),
                '%.3f deg/s' % mean3)
        peak = m['stick_phase']['r_peak_deg_s']
        verdict('%s: 최대 각속도가 게이트의 3 Hz 표본 최대 %.2f 이상이다' % (label, ref['peak']),
                peak is not None and ref['peak'] - 0.05 <= peak <= ref['peak'] + 0.6, '%.3f deg/s' % peak)
        first = m['entry']['integrator_first']
        verdict('%s: 적분기가 비어서 시작한다 (게이트 %.3f)' % (label, ref['integ']),
                first is not None and abs(first) <= F.INTEG_CLEAR, '%s' % first)
        onset = F.gyro_mean(log, leg['on_us'] - int(0.3 * S), leg['on_us'])
        verdict('%s: 스틱 직전 각속도가 게이트의 출발 %.2f 와 같다' % (label, ref['start']), near(onset, ref['start'], 0.3),
                '%.3f deg/s' % onset)
        release = m['release']['r_deg_s']['+10']
        verdict('%s: 놓은 뒤 10 s 에는 서 있다 (게이트: 10 s 안에 정지)' % label, release is not None and abs(release) < 1.0,
                '%s deg/s' % release)
        samples_path = os.path.join(F.OUT, ref['samples'])

        if os.path.exists(samples_path):
            rows = gate_samples(samples_path)
            last = [x for x in rows if x[0] >= rows[-1][0] - 3.0]
            live = [float(np.mean([x[k] for x in last])) for k in (1, 2, 3)]
            _, torque = F.samples(log, 'vehicle_torque_setpoint', 'xyz[2]', off - 3 * S, off)
            _, integ = F.samples(log, 'rate_ctrl_status', 'yawspeed_integ', off - 3 * S, off)
            ours = [mean3, float(np.mean(torque)), float(np.mean(integ))]
            verdict('%s: 마지막 3 s 의 각속도, 토크, 적분기가 게이트의 실시간 표본과 같다' % label,
                    near(ours[0], live[0], 0.15) and near(ours[1], live[1], 0.01) and near(ours[2], live[2], 0.005),
                    '로그 %.3f / %.4f / %.4f, 표본 %.3f / %.4f / %.4f' % tuple(ours + live))
        else:
            note('  건너뜀: %s 가 없다' % ref['samples'])

        rates = F.topic_rates(log, [(leg['acro_us'] - 5 * S, leg['release_end_us'])], [(leg['acro_us'], leg['acro_end_us'])])
        ok, bad = F.rates_ok(rates)
        verdict('%s: 필요한 토픽이 필요한 속도로 있다' % label, ok, '모자란 토픽 %s' % bad)
        flagged = m['stick_phase']['flagged_fraction']
        verdict('%s: 0.5 스틱은 거의 깃발 없이 따라간다 (정지 상태: 결함이 없는 경우)' % label,
                flagged is not None and flagged < 0.2, '스틱 중 깃발 %.0f %%' % (100 * flagged))

    if not found:
        note('  건너뜀: 2026-09-29 게이트 로그가 하나도 없다')


# ---------------------------------------------------------------------------
# 5. the live sequence against a fake px4 on a virtual clock
# ---------------------------------------------------------------------------

class FakePX4:
    """Just enough of px4 for main_live: a toy surge and yaw plant from the
    2527 files with HEAD's PI loop in Acro and the passthrough in Manual, the
    client calls the check makes, and a log of what happened, built as a Log
    at the end. Every client call costs 20 ms of the virtual clock, every
    sleep its argument; the simulator runs at real time on it. A toy: no sway,
    no pods' swing, no saturation flags. It exercises the sequence, not the
    physics.

    Parameters as px4 boots them: the files' values (plus `firmware`, values
    an airframe would have set instead), rcS's SDLOG_* defaults, then every
    PX4_PARAM_ of the environment set on top unless `take_overrides` is False
    (a broken route); `param show -a` flags a value that differs from its
    default '+'. With `snapshot`, the events JSON is read off the disk one
    simulator second after the yaw stick first moves, while the run is still
    flying: what a kill there would leave."""

    def __init__(self, model, sticks_holder, stall_after_s=None, firmware=None, take_overrides=True, snapshot=None,
                 log_params=None):
        self.model = model
        self.log_params = dict(log_params or {})     # the log's parameters beyond PARAMS, on top of the yaml's
        self.wall = 1000.0
        self.sim_us = 5 * S
        self.nav, self.armed = 4, False
        self.armed_ever = False
        self.u = self.r = 0.0
        self.psi = 0.0
        self.integ = 0.0
        self.closed_prev = False
        self.sticks = sticks_holder
        self.defaults = dict(F.plan_params(model), **(firmware or {}))
        self.defaults.update(SDLOG_MODE=1.0, SDLOG_PROFILE=131.0, SDLOG_DIRS_MAX=7.0)
        self.values = dict(self.defaults)

        if take_overrides:
            self.values.update({k[len('PX4_PARAM_'):]: float(v) for k, v in os.environ.items()
                                if k.startswith('PX4_PARAM_')})

        self.p = {k: self.values[k] for k in F.PARAMS}
        self.snapshot_path, self.snapshot, self.snapshot_due = snapshot, None, None
        self.m = gz_model.surge_mass(model)
        self.T = gz_model.pod_thrust(model)
        self.c = gz_model.translational_drag(gz_model.merged(model))[0]
        self.k = gz_model.fin_yaw_damping(model)
        self.cr = abs(gz_model.hydrodynamics(gz_model.merged(model)).get('nRabsR', 0.0))
        self.A = gz_model.yaw_authority(model)
        self.Iz = gz_model.lateral_derivatives(model)['i_z']
        self.tail = self.p['CA_AIRSHIP_TAIL'] == 1.0
        self.rows = {name: [] for name in F.TOPICS}
        self.logging = True
        self.stall_after_s = stall_after_s
        self.next = {}

    # the clock
    def tick(self, seconds):
        self.wall += seconds
        self.advance(int(round((self.wall - 1000.0) * 1e6)) + 5 * S)

        if self.snapshot_path and self.snapshot is None:
            if self.snapshot_due is None and self.sticks.yaw:
                self.snapshot_due = self.sim_us + S
            elif self.snapshot_due is not None and self.sim_us >= self.snapshot_due:
                exists = os.path.exists(self.snapshot_path)
                self.snapshot = {'exists': exists, 'text': open(self.snapshot_path).read() if exists else None,
                                 'partial': os.path.exists(self.snapshot_path + '.partial')}

    def due(self, topic, period_us):
        if self.sim_us >= self.next.get(topic, 0):
            self.next[topic] = self.sim_us + period_us
            return True
        return False

    def advance(self, until_us):
        if self.stall_after_s is not None and until_us > self.stall_after_s * S:
            until_us = self.stall_after_s * S          # the simulator stops; the wall goes on

        dt = 0.004

        while self.sim_us + 4000 <= until_us:
            self.sim_us += 4000
            stick = self.sticks.yaw if self.armed else 0.0
            throttle = max(0.0, min(1.0, int(self.sticks.throttle * 1000) / 1000.0)) if self.armed else 0.0
            closed = self.armed and self.nav == F.NAV_ACRO
            sp = math.radians(F.setpoint_deg(stick, self.p)) if closed else 0.0

            if closed and not self.closed_prev:
                self.integ = 0.0

            if not closed and self.closed_prev:
                self.integ = 0.0
                self.rows['rate_ctrl_status'].append((self.sim_us, 0.0))

            r_rad = math.radians(self.r)

            if closed:
                torque = self.p['AS_YAWRATE_P'] * (sp - r_rad) + self.integ + self.p['AS_YAWRATE_FF'] * sp
                self.integ = max(-self.p['AS_YR_INT_LIM'], min(self.p['AS_YR_INT_LIM'],
                                                               self.integ + self.p['AS_YAWRATE_I'] * (sp - r_rad) * dt))
            else:
                torque = stick if self.armed else 0.0

            torque = max(-1.0, min(1.0, torque))
            self.closed_prev = closed
            self.u += (self.T * throttle - self.c * self.u * abs(self.u)) / self.m * dt
            r_rad += (self.A * torque - self.k * self.u * r_rad - self.cr * r_rad * abs(r_rad)) / self.Iz * dt
            self.r = math.degrees(r_rad)
            self.psi += r_rad * dt
            t = self.sim_us
            R = self.rows

            if self.due('av', 8000):
                R['vehicle_angular_velocity'].append((t, 0.0, 0.0, r_rad))
            if self.due('gt', 20000):
                R[F.GT_POS].append((t, self.u * math.cos(self.psi), self.u * math.sin(self.psi), 0.0, -20.0))
                R[F.GT_ATT].append((t, math.cos(self.psi / 2), 0.0, 0.0, math.sin(self.psi / 2)))
                R['vehicle_local_position'].append((t, self.u * math.cos(self.psi), self.u * math.sin(self.psi), self.psi))
                R['vehicle_torque_setpoint'].append((t, torque))
                R['vehicle_thrust_setpoint'].append((t, throttle))
            if self.due('status', 500000):
                R['vehicle_status'].append((t, self.nav, 2 if self.armed else 1))
                R['vehicle_control_mode'].append((t, float(self.armed), 1.0, float(self.nav == F.NAV_ACRO)))
            if self.due('mcs', 200000):
                R['manual_control_setpoint'].append((t, stick))
            if closed and self.due('rsp', 50000):
                R['vehicle_rates_setpoint'].append((t, sp))
            if closed and self.due('rcs', 200000):
                R['rate_ctrl_status'].append((t, self.integ))
            if self.due('cas', 200000):
                R['control_allocator_status'].append((t, 0.0, 1.0))
            if self.due('act', 100000):
                if self.tail:
                    # collective pods forward, the tail takes the yaw, one tilt servo at level
                    R['actuator_motors'].append((t, throttle, throttle, torque))
                    R['actuator_servos'].append((t, 0.0, 0.0))
                else:
                    stbd, port = throttle - torque, throttle + torque
                    R['actuator_motors'].append((t, abs(stbd), abs(port), 0.0))
                    R['actuator_servos'].append((t, 1.0 if stbd < 0 else 0.0, 1.0 if port < 0 else 0.0))

    def mode_change(self):
        self.rows['vehicle_status'].append((self.sim_us, self.nav, 2 if self.armed else 1))
        self.rows['vehicle_control_mode'].append((self.sim_us, float(self.armed), 1.0, float(self.nav == F.NAV_ACRO)))

    # the clients
    def listener(self, topic):
        self.tick(0.02)
        t = self.sim_us

        if topic == 'vehicle_angular_velocity':
            return 'TOPIC\n    timestamp: %d (0.0 seconds ago)\n    xyz: [0.00000, 0.00000, %.5f]\n' % (t, math.radians(self.r))
        if topic == F.GT_POS:
            return ('    timestamp: %d (0.0 seconds ago)\n    vx: %.5f\n    vy: %.5f\n    z: -20.00000\n    heading: %.5f\n'
                    % (t, self.u * math.cos(self.psi), self.u * math.sin(self.psi), math.atan2(math.sin(self.psi), math.cos(self.psi))))
        if topic == 'vehicle_status':
            return '    timestamp: %d\n    nav_state: %d\n    arming_state: %d\n' % (t, self.nav, 2 if self.armed else 1)
        if topic == 'vehicle_control_mode':
            return '    timestamp: %d\n    flag_armed: %s\n' % (t, 'True' if self.armed else 'False')
        return None

    def commander(self, *args):
        self.tick(0.02)

        if args[0] == 'status':
            return 'ok'
        if args[0] == 'mode':
            self.nav = {'manual': F.NAV_MANUAL, 'acro': F.NAV_ACRO}[args[1]]
            self.mode_change()
        elif args[0] == 'arm':
            self.armed = self.armed_ever = True
            self.mode_change()
        elif args[0] == 'disarm':
            self.armed = False
            self.logging = False
            self.mode_change()
        return ''

    def px4(self, *args, timeout=15):
        self.tick(0.02)

        if args[:3] == ('param', 'show', '-a'):
            # param.cpp do_show_print: "%c %c %c %s [%d,%d] : ", the second
            # column '+' for a value that is not its default
            lines = ['x %s   %s [1,2] : %.4f' % (' ' if self.defaults.get(k) == v else '+', k, v)
                     for k, v in self.values.items()]
            return ('Symbols: x = used, + = saved, * = unsaved, l = locked\n' + '\n'.join(lines)
                    + '\n\n 2000 parameters total, 900 used.\n')
        if args[:2] == ('logger', 'status'):
            return ('INFO  [logger] Full File Logging Running:\nINFO  [logger] Log file: ./fs/log/2026-10-01/10_00_00.ulg\n'
                    if self.logging else 'INFO  [logger] Not logging\n')
        return ''

    def log(self):
        def pack(topic, names):
            rows = self.rows[topic]
            cols = list(zip(*rows)) if rows else [[] for _ in names]
            return {n: np.array(c) for n, c in zip(names, cols)}

        data = {
            'vehicle_status': pack('vehicle_status', ('timestamp', 'nav_state', 'arming_state')),
            'vehicle_control_mode': pack('vehicle_control_mode', ('timestamp', 'flag_armed', 'flag_control_manual_enabled',
                                                                  'flag_control_rates_enabled')),
            'manual_control_setpoint': pack('manual_control_setpoint', ('timestamp', 'yaw')),
            'vehicle_rates_setpoint': pack('vehicle_rates_setpoint', ('timestamp', 'yaw')),
            'vehicle_angular_velocity': pack('vehicle_angular_velocity', ('timestamp', 'xyz[0]', 'xyz[1]', 'xyz[2]')),
            F.GT_POS: pack(F.GT_POS, ('timestamp', 'vx', 'vy', 'vz', 'z')),
            F.GT_ATT: pack(F.GT_ATT, ('timestamp', 'q[0]', 'q[1]', 'q[2]', 'q[3]')),
            'vehicle_local_position': pack('vehicle_local_position', ('timestamp', 'vx', 'vy', 'heading')),
            'vehicle_torque_setpoint': pack('vehicle_torque_setpoint', ('timestamp', 'xyz[2]')),
            'vehicle_thrust_setpoint': pack('vehicle_thrust_setpoint', ('timestamp', 'xyz[0]')),
            'rate_ctrl_status': pack('rate_ctrl_status', ('timestamp', 'yawspeed_integ')),
            'control_allocator_status': pack('control_allocator_status', ('timestamp', 'unallocated_torque[2]',
                                                                          'torque_setpoint_achieved')),
            'actuator_motors': pack('actuator_motors', ('timestamp', 'control[0]', 'control[1]', 'control[2]')),
            'actuator_servos': pack('actuator_servos', ('timestamp', 'control[0]', 'control[1]')),
        }
        # rows appended out of order at a mode change: sort each topic by time
        for topic, cols in data.items():
            order = np.argsort(cols['timestamp'], kind='stable')
            data[topic] = {k: v[order] for k, v in cols.items()}

        params = dict(self.p, IMU_GYRO_CUTOFF=gyro_cutoff_yaml())      # a SITL log carries every parameter
        params.update(self.log_params)
        return F.Log(data, params, start_us=5 * S, end_us=self.sim_us, info={'ver_sw': 'fake'}, path='fake.ulg')


def fake_live_run(model, legs, stall_after_s=None, env=None, firmware=None, take_overrides=True, snapshot=False,
                  log_params=None):
    """main_live against FakePX4. Returns (exit code, output, the fake, the
    verdicts main_live recorded, the files it wrote). `env` is the run's
    PX4_PARAM_ environment, the documented PX4_PARAM_SDLOG_DIRS_MAX=0 unless
    given; any the caller's shell had are out of the way meanwhile.
    `log_params`: parameters the log carries on top of the fake's (an
    IMU_GYRO_CUTOFF other than the yaml's, say). The fake keeps the summary
    and the events JSON's text (summary_text, events_text)."""
    import importlib
    import types
    import report

    env = {'PX4_PARAM_SDLOG_DIRS_MAX': '0'} if env is None else env
    outside = {k: v for k, v in os.environ.items() if k.startswith('PX4_PARAM_')}

    for k in outside:
        os.environ.pop(k)

    os.environ.update(env)
    os.environ['PX4_GZ_ENTITY'] = model
    real = importlib.import_module('sitl_link')

    class Sticks:
        def __init__(self):
            self.pitch = self.roll = self.yaw = self.throttle = 0
            self.stop = False

        def start(self):
            pass

    holder = Sticks()
    saved = {'sitl_link': sys.modules.get('sitl_link'), 'time': F.time, 'ROOTFS': F.ROOTFS, 'OUT': F.OUT,
             'read': F.Log.read, 'results': list(report._results)}
    out = io.StringIO()

    with tempfile.TemporaryDirectory() as tmp:
        rootfs = os.path.join(tmp, 'rootfs')
        os.makedirs(os.path.join(rootfs, 'fs/log/2026-10-01'))

        with open(os.path.join(rootfs, 'fs/log/2026-10-01/10_00_00.ulg'), 'wb') as handle:
            handle.write(b'ULog stand-in')

        events_path = os.path.join(tmp, 'out', 'forward_%s_20261001_100000_events.json' % model)
        fake = FakePX4(model, holder, stall_after_s, firmware, take_overrides, events_path if snapshot else None,
                       log_params)
        module = types.ModuleType('sitl_link')
        module.field, module.vector, module.finite = real.field, real.vector, real.finite
        module.PARAM_LINE = real.PARAM_LINE
        module.INSTANCE = '0'
        module.listener, module.commander, module.px4 = fake.listener, fake.commander, fake.px4
        module.param_set = lambda name, value: fake.tick(0.02)
        module.wait_for_ekf = lambda timeout=150: True
        module.Sticks = lambda: holder
        clock = types.SimpleNamespace(time=lambda: fake.wall, sleep=fake.tick, strftime=time_module.strftime,
                                      gmtime=time_module.gmtime)
        sys.modules['sitl_link'] = module
        F.time, F.ROOTFS, F.OUT = clock, rootfs, os.path.join(tmp, 'out')
        F.Log.read = classmethod(lambda cls, path: fake.log())
        report._results.clear()

        try:
            with contextlib.redirect_stdout(out):
                code = F.main_live(legs)
            recorded = report.results()
            files = sorted(os.listdir(F.OUT)) if os.path.isdir(F.OUT) else []
            fake.summary_text = fake.events_text = None

            for name in files:
                if name.endswith('_summary.json'):
                    with open(os.path.join(F.OUT, name)) as handle:
                        fake.summary_text = handle.read()

                if name.endswith('_events.json'):
                    with open(os.path.join(F.OUT, name)) as handle:
                        fake.events_text = handle.read()
        finally:
            sys.modules['sitl_link'] = saved['sitl_link']
            F.time, F.ROOTFS, F.OUT, F.Log.read = saved['time'], saved['ROOTFS'], saved['OUT'], saved['read']
            report._results[:] = saved['results']
            os.environ.pop('PX4_GZ_ENTITY', None)

            for k in env:
                os.environ.pop(k, None)

            os.environ.update(outside)

    return code, out.getvalue(), fake, recorded, files


import time as time_module   # noqa: E402  (the fake clock lends the real formatting functions)


def test_live_sequence():
    code, text, fake, recorded, files = fake_live_run(VEC_ALL, '1,2')
    failed = [name for name, ok in recorded if not ok]
    verdict('가짜 px4 에서 두 다리를 날고 모든 타당성 판정이 통과한다', code == 0 and not failed and len(recorded) >= 12,
            '종료 %s, 판정 %d, 실패 %s' % (code, len(recorded), failed))
    verdict('기록 네 가지를 남긴다 (로그 사본, 사건, 다리 CSV 둘, 요약), 반쯤 쓴 파일은 없다',
            len([f for f in files if f.endswith('.csv')]) == 2 and any(f.endswith('_events.json') for f in files)
            and any(f.endswith('_summary.json') for f in files) and any(f.endswith('.ulg') for f in files)
            and not any(f.endswith('.partial') for f in files), str(files))
    verdict('진입마다 Manual 에서 Acro 로, 놓음 끝은 Manual 로 (가짜의 기록)',
            '다리 1: Manual 에서 Acro 로 들어가 적분기가 비어서 시작했다' in text and '다리 2: 로그의 모드와 스틱 시각이' in text, '')
    # the toy has no schedule and no cap: given an integral gain that fills the
    # limit inside the 20 s stick (the 10.97 m anchor's 0.09, through
    # PX4_PARAM_), the defect's shape must be what the record shows
    _, toy_text, _, _, _ = fake_live_run(VEC_ALL, '1,2', env={'PX4_PARAM_SDLOG_DIRS_MAX': '0', 'PX4_PARAM_AS_YAWRATE_I': '0.09'})
    verdict('토이 플랜트에서도 결함 모양이 기록된다: 전타에서 적분기가 한계, 깃발 없음 (I 0.09)',
            '적분기 끝 +0.3000' in toy_text and '깃발 없음' in toy_text, '')
    events = json.loads(fake.events_text) if fake.events_text else {}
    verdict('PX4_PARAM_SDLOG_DIRS_MAX=0 이 경로를 보이고 (기본값 7 과 다르다), 사건 기록이 끝까지 쓰였다',
            '환경의 PX4_PARAM_ 이 rcS 를 거쳐 펌웨어에 들어갔다 (기본값이 아닌 값이 하나 이상)' in [n for n, ok in recorded if ok]
            and events.get('overrides') == {'SDLOG_DIRS_MAX': '0'} and events.get('params_changed', {}).get('SDLOG_DIRS_MAX')
            and events.get('finished') is True and events.get('aborted') is None
            and [r['leg'] for r in events.get('legs', [])] == [1, 2], str({k: events.get(k) for k in ('overrides', 'finished')}))
    # The entry's trim check must read the constant throttle only: a live
    # steady window shorter than the log's let it reach back into the
    # acceleration (0.0432 for 0.031), and no verdict caught that
    with_summary = [f for f in files if f.endswith('_summary.json')]
    holds = []

    if with_summary:
        doc = json.loads(fake.summary_text)
        holds = [(leg['steady']['throttle_holds']['throttle'], leg['plan']['throttle'],
                  leg['steady']['throttle_holds']['drag_implied']) for leg in doc['legs']]

    verdict('진입 전 창은 일정한 스로틀 구간 안이다 (트림 검사가 가속을 읽지 않는다)',
            len(holds) == 2 and all(near(a, b, 1e-9) and near(c, gz_model.translational_drag(gz_model.merged(VEC_ALL))[0], 0.005)
                                    for a, b, c in holds), str(holds))
    # the simulator stalls mid-release: the wait gives up on the wall, the run
    # says so, and the leg it was in is still measured and judged incomplete
    code, text, fake, recorded, files = fake_live_run(VEC_ALL, '1', stall_after_s=120.0)
    failed = [name for name, ok in recorded if not ok]
    verdict('시뮬레이터 시계가 멈추면 벽시계 한도에서 중단하고 그렇게 말한다',
            code == 1 and '중단: 시뮬레이터 시계가' in text and '모든 다리를 끝까지 날았다' in failed, '실패 판정 %s' % failed)
    verdict('중단된 다리도 기록되고 미완으로 판정된다',
            '다리 1: 스틱 20 s 와 놓음 120 s 가 시뮬레이터 시계로 끝까지 갔다' in failed
            and '다리 1: 진입이 2.0 m/s 의 정상 직진이다' not in failed
            and any(f.endswith('_leg1.csv') for f in files), str(files))
    code, text, fake, recorded, files = fake_live_run(CS_ALL, '1')
    verdict('완전한 Cloudship 은 Acro 에서 가속하고 Manual 을 거쳐 Acro 로 진입한다',
            '가속 acro' in text and code == 0, '종료 %s' % code)


def test_live_records():
    """What the review of 2026-10-01 found in the live side: legs named by
    their order of flight, a timeline written only after the flight, a
    PX4_PARAM_ route never checked, a firmware precondition exiting 1."""
    # leg 1's entry never becomes steady: leg 2 is the only one flown, and
    # every record of it says leg 2 (a rerun from its label flies it)
    real_wait = F.wait_steady
    calls = []

    def unsteady_first(live, target):
        calls.append(target)
        return (False, 0.0, {}) if len(calls) == 1 else real_wait(live, target)

    F.wait_steady = unsteady_first

    try:
        code, text, fake, recorded, files = fake_live_run(VEC_ALL, '1,2')
    finally:
        F.wait_steady = real_wait

    doc = json.loads(fake.summary_text) if fake.summary_text else {'legs': []}
    named = [n for n, _ in recorded if n.startswith('다리 ')]
    verdict('다리 1 을 날지 못하면 날은 다리는 기록, CSV, 판정에서 모두 다리 2 다',
            code == 1 and [m['leg'] for m in doc['legs']] == [2] and [f for f in files if f.endswith('.csv')]
            == ['forward_%s_20261001_100000_leg2.csv' % VEC_ALL] and named and all(n.startswith('다리 2:') for n in named)
            and '날지 않은 다리: [1]' in text, '종료 %s, %s, %s' % (code, [m['leg'] for m in doc['legs']], named[:1]))
    # a rerun of default leg 3 alone flies first and is still leg 3 everywhere
    # (legs 1 and 2 above number as they fly, which would hide a position)
    code, text, fake, recorded, files = fake_live_run(VEC_ALL, '3')
    doc = json.loads(fake.summary_text) if fake.summary_text else {'legs': []}
    events = json.loads(fake.events_text) if fake.events_text else {'legs': []}
    named = [n for n, _ in recorded if n.startswith('다리 ')]
    verdict('FORWARD_LEGS=3 으로 다시 난 다리는 처음 날아도 기록, CSV, 판정, 사건 기록에서 다리 3 이다',
            code == 0 and [m['leg'] for m in doc['legs']] == [3] and [r['leg'] for r in events['legs']] == [3]
            and [f for f in files if f.endswith('.csv')] == ['forward_%s_20261001_100000_leg3.csv' % VEC_ALL]
            and named and all(n.startswith('다리 3:') for n in named), '종료 %s, %s' % (code, named[:1]))
    # an Abort before the log is found measured nothing: never a pass
    real_path = F.logger_path

    def no_answer(sl):
        raise F.Abort('px4 가 상태를 25 번 연속 내주지 않았다 (시험)')

    F.logger_path = no_answer

    try:
        code, text, fake, recorded, files = fake_live_run(VEC_ALL, '1')
    finally:
        F.logger_path = real_path

    verdict('로그를 찾기 전에 중단된 실행은 통과가 아니다 (종료 3, 실패한 판정 하나)',
            code == 3 and [n for n, ok in recorded if not ok] == ['이 실행이 쓰는 로그를 찾았다'], '종료 %s' % code)
    # the timeline is on disk while the stick is held, before any finally
    code, text, fake, recorded, files = fake_live_run(VEC_ALL, '1', snapshot=True)
    shot = fake.snapshot or {}
    taken = json.loads(shot['text']) if shot.get('text') else {}
    legs = taken.get('legs') or [{}]
    verdict('스틱을 건 1 s 뒤 디스크의 사건 기록에 그 다리의 스틱이 있다 (바깥에서 죽여도 남는다)',
            shot.get('exists') and not shot.get('partial') and legs[0].get('flown') is True
            and 'stick_on' in legs[0].get('events', {}) and 'stick_off' not in legs[0].get('events', {})
            and taken.get('finished') is False and taken.get('aborted') is None
            and any(e['name'] == 'stick_on' for e in taken.get('events') or []), str({k: shot.get(k) for k in ('exists', 'partial')}))
    # the PX4_PARAM_ route, before anything is flown: none at all, one equal
    # to its default (it reads back the same with the route broken), one that
    # never arrived; and a firmware precondition. Each stops the run with 2.
    cases = (('PX4_PARAM_ 없음', dict(env={})),
             ('기본값과 같은 값뿐', dict(env={'PX4_PARAM_AS_YAWRATE_P': '%g' % P27['AS_YAWRATE_P']})),
             ('경로가 끊김', dict(take_overrides=False)),
             ('THR_MDL_FAC 0', dict(firmware={'THR_MDL_FAC': 0.0})))
    outcome = {}

    for name, knobs in cases:
        code, text, fake, recorded, files = fake_live_run(VEC_ALL, '1', **knobs)
        outcome[name] = (code, fake.armed_ever, [n for n, ok in recorded if not ok])

    want = {'PX4_PARAM_ 없음': '환경의 PX4_PARAM_', '기본값과 같은 값뿐': '환경의 PX4_PARAM_',
            '경로가 끊김': '환경의 PX4_PARAM_', 'THR_MDL_FAC 0': '펌웨어가 이 계획의'}
    verdict('PX4_PARAM_ 이 없거나, 기본값과 같기만 하거나, 들어가지 않았거나, 펌웨어 전제가 틀리면 무장 전에 2 로 멈춘다',
            all(c == 2 and not armed and len(bad) == 1 and bad[0].startswith(want[n])
                for n, (c, armed, bad) in outcome.items()), str(outcome))


def with_speed(log, cap_deg, source=2, upper_held=False, after_us=None, after=None):
    """The synthetic log with airship_speed at 10 Hz and the schedule's
    parameters on: a constant cap and source, or `after` = (source, held)
    from `after_us` on."""
    t = np.arange(0, END, 100_000, dtype=np.int64)
    late = (t >= after_us) if after_us is not None else np.zeros(len(t), dtype=bool)
    src = np.where(late, after[0], source) if after else np.full(len(t), source)
    held = np.where(late, after[1], upper_held) if after else np.full(len(t), upper_held)
    log.data['airship_speed'] = {'timestamp': t, 'u_lo': np.full(len(t), 3.9), 'u_hi': np.full(len(t), 4.1),
                                 'yaw_rate_ff': np.full(len(t), 17.0), 'yaw_authority': np.ones(len(t)),
                                 'yaw_rate_cap': np.full(len(t), math.radians(cap_deg)),
                                 'source': src.astype(float), 'upper_held': held.astype(float)}
    log.params.update({'AS_SPD_SRC': 2.0, 'AS_YR_FF_SLP': 4.01, 'AS_SPD_HI': 2.5, 'AS_SPD_WMAX': 0.0,
                       'AS_SPD_DP_TOL': 1.05})
    return log


def test_schedule():
    """The schedule's record and verdicts (D1 increment 1)."""
    full = F.setpoint_deg(1.0, F.plan_params(VEC_FIN))      # the synthetic setpoint: the stick's share of MAX

    with tempfile.TemporaryDirectory() as out:
        doc, checks = F.analyze(synthetic_log(), VEC_ALL, out_dir=out, stamp='off')
        sc = doc['legs'][0]['schedule']
        verdict('스케줄 없는 펌웨어의 로그: 기록은 logged False, 스케줄 판정은 없다',
                sc['on'] is False and sc['logged'] is False and doc['schedule_verdicts'] == []
                and len(checks) == len(doc['validity']), str(sc))

        doc, checks = F.analyze(with_speed(synthetic_log(), full), VEC_ALL, out_dir=out, stamp='on')
        v = {d['name']: d['ok'] for d in doc['schedule_verdicts']}
        sc = doc['legs'][0]['schedule']
        verdict('상한이 설정과 같으면 \'제 몫만 지시\' 가 통과하고, 기록에 구간, ff, 원천이 있다',
                v.get('다리 1: airship_speed 가 로그에 있다') is True
                and v.get('다리 1: 스틱이 상한의 제 몫만 지시했다 (마지막 5 s)') is True
                and sc['at_release']['source'] == 'ground' and abs(sc['at_release']['u_lo'] - 3.9) < 1e-9
                and abs(sc['setpoint_from_cap_last5_deg_s'] - full) < 1e-6
                and doc['legs'][0]['replay']['params']['AS_YR_FF_SLP'] == 4.01
                and len(checks) == len(doc['validity']) + len(doc['schedule_verdicts']), str(v))
        verdict('스케줄 판정은 재현을 막지 않는다 (적분기 판정이 실패해도 run_leg 이 있다)',
                v.get('다리 1: 스틱 동안 적분기가 한계에 닿지 않았다') is False
                and doc['legs'][0]['replay']['run_leg'] is not None, str(v))

        doc, _ = F.analyze(with_speed(synthetic_log(), full / 3.0), VEC_ALL, out_dir=out, stamp='over')
        v = {d['name']: (d['ok'], d['detail']) for d in doc['schedule_verdicts']}
        nosrc = {d['name']: d['ok'] for d in F.analyze(with_speed(synthetic_log(), full, source=0), VEC_ALL, out_dir=out,
                                                           stamp='nosrc')[0]['schedule_verdicts']}
        verdict('스틱 때 속도원이 none 이면 \'속도원이 있었다\' 가 실패하고, ground 면 통과한다',
                nosrc.get('다리 1: 스틱을 걸 때 속도원이 있었다') is False
                and {d['name']: d['ok'] for d in doc['schedule_verdicts']}.get('다리 1: 스틱을 걸 때 속도원이 있었다') is True,
                str(nosrc))
        verdict('설정이 상한의 몫을 넘으면 \'제 몫만 지시\' 가 실패한다',
                v['다리 1: 스틱이 상한의 제 몫만 지시했다 (마지막 5 s)'][0] is False, str(v)[:200])

        log = with_speed(synthetic_log(), full)
        log.data['airship_speed']['u_lo'][:] = 2.0          # inside the 1.5..2.5 ramp at the stick
        doc, _ = F.analyze(log, VEC_ALL, out_dir=out, stamp='ramp')
        names = [d['name'] for d in doc['schedule_verdicts']]
        verdict('램프 안에서 건 스틱은 요 각속도를 설정과 견주지 않는다 (적분기가 제 속도로 쌓는다)',
                '다리 1: 요 각속도가 설정에 닿았다 (마지막 5 s)' not in names and len(names) == 4, str(names))

        plan = dict(F.vehicle_plan(VEC_ALL)['legs'][1], speed=2.0, stick=1.0)
        lost = ON + 8 * S
        events = {'acro': (ACRO - 30_000, ACRO - 10_000), 'stick_on': (ON - 60_000, ON - 40_000),
                  'source_lost': (lost - 20_000, lost), 'stick_off': (OFF - 60_000, OFF - 40_000),
                  'release_end': (MANUAL2 - 20_000, MANUAL2 + 5_000)}
        live = {'model': VEC_ALL, 'legs': [{'leg': 1, 'flown': True, 'plan': plan, 'events': events}]}

        for name, after, want in (('none, held', (0, True), True), ('ground stays', (2, False), False),
                                  ('none, not held', (0, False), False)):
            log = with_speed(synthetic_log(), full, after_us=lost + 300_000, after=after)
            doc, _ = F.analyze(log, VEC_ALL, live=live, out_dir=out, stamp='loss')
            v = {d['name']: (d['ok'], d['detail']) for d in doc['schedule_verdicts']}
            key = '다리 1: 속도원을 잃자 원천이 none 이 되고 상한 구간이 유지됐다'
            loss = doc['legs'][0]['schedule'].get('loss') or {}
            verdict('속도원 잃음 (%s): 판정 %s' % (name, want),
                    key in v and v[key][0] is want and '다리 1: 요 각속도가 설정에 닿았다 (마지막 5 s)' not in v
                    and abs(loss.get('after_stick_on_s', -1) - 8.0) < 1e-9, str(v.get(key)))

    env = {F.SOURCE_LOSS_ENV: '8'}
    verdict('FORWARD_SOURCE_LOSS_S 를 초로 읽고, 없으면 None', F.source_loss_s(env) == 8.0 and F.source_loss_s({}) is None,
            '')
    verdict('스케줄 스위치는 원천과 기울기 둘 다', F.schedule_on({'AS_SPD_SRC': 2.0, 'AS_YR_FF_SLP': 4.0})
            and not F.schedule_on({'AS_SPD_SRC': 0.0, 'AS_YR_FF_SLP': 4.0})
            and not F.schedule_on({'AS_SPD_SRC': 2.0, 'AS_YR_FF_SLP': 0.0})
            and not F.schedule_on({'AS_SPD_SRC': None, 'AS_YR_FF_SLP': None}), '')


def main():
    for test in (test_trim, test_files, test_firmware_constants, test_added_mass_reads, test_plan, test_legs,
                 test_synthetic, test_synthetic_variants, test_live_state_frame, test_judgement_and_timeline,
                 test_reversal_counters, test_analyze_end_to_end, test_interface_constants, test_replay_block,
                 test_pod_modes, test_fins_by_name, test_fins_body_frame, test_tail_torque_flips,
                 test_rear_s_last_interval, test_smoothed_peak, test_csv_and_definitions, test_imu_cutoff,
                 test_run_leg_refusals, test_analyze_keeps_live, test_gate_logs, test_live_sequence, test_live_records,
                 test_schedule):
        note('\n[%s]' % test.__name__)
        test()

    return summary()


if __name__ == '__main__':
    sys.exit(main())
