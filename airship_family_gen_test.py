#!/usr/bin/env python3
"""airship_family_gen.py and airship_gains.py against the tree, no gz.

    python3 airship_family_gen_test.py

1. The generic vehicle regenerated into a scratch copy of the tree is the tree
   (every model, airframe, world and docs file).
2. The indoor vehicle generated into another scratch copy: eight model
   directories, 30 world entries, six airframes, the CMake entries and decade
   line, the docs entries; a second run changes nothing; variant_check.py on
   that copy passes every fact; the design meets its requirements.
3. airship_gains.py: its anchor (REF_PLANT, the 10.97 m hull the gains were
   tuned on, P 0.7 and I 0.09) holds the numbers read from that hull; on the generic
   vehicle it gives the anchor's loop bandwidth N P / I and integral time
   P / I, and the slope seed over N; on the indoor vehicle it writes its three
   blocks, a second run identically.
"""
import json
import re
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import airship_family_gen as gen             # noqa: E402
import gz_model                              # noqa: E402
from report import note, summary, verdict    # noqa: E402

ROOT = gz_model.ROOT
COPIED = gen.TREE_DIRS + ('ROMFS/px4fmu_common/init.d/airframes',)
FILES = gen.TREE_FILES


def scratch_tree():
    root = tempfile.mkdtemp(prefix='airship_family_test_')

    for rel in COPIED:
        shutil.copytree(os.path.join(ROOT, rel), os.path.join(root, rel))

    for rel in FILES:
        os.makedirs(os.path.dirname(os.path.join(root, rel)), exist_ok=True)
        shutil.copy(os.path.join(ROOT, rel), os.path.join(root, rel))

    return root


def run(args, root):
    env = dict(os.environ, PX4_ROOT=root)
    return subprocess.run([sys.executable] + args, cwd=HERE, env=env, capture_output=True, text=True)


def differs(root):
    out = []

    for rel in COPIED + FILES:
        r = subprocess.run(['diff', '-rq', os.path.join(ROOT, rel), os.path.join(root, rel)], capture_output=True, text=True)

        if r.returncode:
            out.extend(l for l in r.stdout.splitlines() if l)

    return out


def snapshot(root):
    out = {}

    for base, _, files in os.walk(root):
        for f in files:
            p = os.path.join(base, f)
            out[os.path.relpath(p, root)] = open(p, 'rb').read()

    return out


def main():
    note('[generic round trip]')
    root = scratch_tree()
    r = run(['airship_family_gen.py', 'vehicles/airship_generic.json'], root)
    verdict('생성기가 generic 기체 파일로 돈다', r.returncode == 0, (r.stderr or r.stdout)[-300:])
    d = differs(root)
    verdict('generic 가족을 다시 만들면 트리와 같다 (모델, 에어프레임, 월드, 문서)', not d, '; '.join(d)[:400] or '같음')

    # the fixed twin's flap servo range is the generator's: a hand-set range
    # that disagrees with the flap joints is written back
    fixed = [f for f in os.listdir(os.path.join(root, 'ROMFS/px4fmu_common/init.d-posix/airframes')) if f.endswith('_gz_airship_fixed')]
    path = os.path.join(root, 'ROMFS/px4fmu_common/init.d-posix/airframes', fixed[0]) if len(fixed) == 1 else None
    before = open(path).read() if path else ''
    open(path, 'w').write(re.sub(r'(param set-default SIM_GZ_SV_MINA1 )[\d.-]+', r'\g<1>-99', before, count=1))
    r = run(['airship_family_gen.py', 'vehicles/airship_generic.json'], root)
    verdict('고정 쌍둥이의 플랩 서보 범위는 생성기가 플랩 행정에서 다시 쓴다', r.returncode == 0 and open(path).read() == before,
            (r.stderr or '')[-200:])

    note('[indoor family]')
    root = scratch_tree()
    r = run(['airship_family_gen.py', 'vehicles/airship_indoor.json'], root)
    verdict('생성기가 indoor 기체 파일로 돈다', r.returncode == 0, (r.stderr or r.stdout)[-300:])
    models = os.path.join(root, 'Tools/simulation/gz/models')
    dirs = sorted(d for d in os.listdir(models) if d.startswith('airship_indoor'))
    verdict('모델 디렉터리 8개', len(dirs) == 8 and all(os.path.isfile(os.path.join(models, d, 'model.sdf')) for d in dirs), ', '.join(dirs))
    world = open(os.path.join(root, 'Tools/simulation/gz/worlds/lta.sdf')).read()
    verdict('월드 부력 항목 30개', world.count('<enable>airship_indoor') == 30, '%d' % world.count('<enable>airship_indoor'))
    airframes = os.path.join(root, 'ROMFS/px4fmu_common/init.d-posix/airframes')
    files = sorted(f for f in os.listdir(airframes) if '_gz_airship_indoor' in f)
    verdict('에어프레임 2530/2531/2532/2534/2536/2537', [f.split('_')[0] for f in files] == ['2530', '2531', '2532', '2534', '2536', '2537'], ', '.join(files))
    cmake = open(os.path.join(airframes, 'CMakeLists.txt')).read()
    verdict('CMake 항목 6개와 십 자리 줄', cmake.count('_gz_airship_indoor') == 6 and '#   2530 airship_indoor' in cmake, '')
    vehicles_md = open(os.path.join(root, 'docs/en/sim_gazebo_gz/vehicles.md')).read()
    index_md = open(os.path.join(root, 'docs/en/sim_gazebo_gz/index.md')).read()
    airship_md = open(os.path.join(root, 'docs/en/frames_airship/index.md')).read()
    verdict('문서: 제목 6개, 표 행 6개, 목록 항목 1개', vehicles_md.count('### Indoor Airship') == 6 and index_md.count('| [Indoor Airship') == 6
            and airship_md.count('- [Indoor Airship]') == 1, '%d %d %d' % (vehicles_md.count('### Indoor Airship'), index_md.count('| [Indoor Airship'), airship_md.count('- [Indoor Airship]')))
    before = snapshot(root)
    r = run(['airship_family_gen.py', 'vehicles/airship_indoor.json'], root)
    after = snapshot(root)
    verdict('두 번째 실행은 아무것도 바꾸지 않는다', r.returncode == 0 and before == after, ', '.join(k for k in after if before.get(k) != after[k])[:300])
    r = run(['variant_check.py'], root)
    last = (r.stdout.strip().splitlines() or [''])[-1]
    verdict('variant_check 가 그 트리에서 모두 통과한다', r.returncode == 0 and last.startswith('통과') and '실패 0' in last, last[:200])
    r = run(['generic_hull_design.py', '--spec', 'vehicles/airship_indoor.json'], root)
    verdict('indoor 설계가 요구 조건을 만족한다', r.returncode == 0 and 'all requirements met' in r.stdout, r.stdout.strip().splitlines()[-1][:200])

    note('[surfaces option]')
    # a literature family with flaps on its fin members: generated into a
    # scratch tree, it passes the scheme check and a second run changes nothing
    root = scratch_tree()
    spec = json.load(open(os.path.join(HERE, 'vehicles', 'airship_aurora.json')))
    spec['surfaces'] = True
    surf = os.path.join(root, 'aurora_surfaces.json')
    open(surf, 'w').write(json.dumps(spec))
    r = run(['airship_family_gen.py', surf], root)
    before = snapshot(root)
    r2 = run(['airship_family_gen.py', surf], root)
    after = snapshot(root)
    fin = open(os.path.join(root, 'Tools/simulation/gz/models/airship_aurora_fin/model.sdf')).read()
    member = open(os.path.join(root, 'ROMFS/px4fmu_common/init.d-posix/airframes/2547_gz_airship_aurora_am_fin_drag')).read()
    verdict('조종면 가족: 핀 멤버에 플랩 넷 (servo_2..5, 핀마다 제 플랩 조인트), 에어프레임에 조종면 넷과 틸트 서보 Servo5/6, 두 번째 실행은 같다',
            r.returncode == 0 and r2.returncode == 0 and before == after and fin.count('<link name="flap_') == 4
            and sorted(re.findall(r'<sub_topic>(servo_\d)</sub_topic>', fin)) == ['servo_2', 'servo_3', 'servo_4', 'servo_5']
            and fin.count('<control_joint_name>') == 4 and 'param set-default CA_SV_CS_COUNT 4' in member
            and 'param set-default SIM_GZ_SV_FUNC1 205' in member and '# @output Servo5 starboard pod tilt' in member,
            (r.stderr or r2.stderr or '')[-200:])
    r = run(['variant_check.py'], root)
    last = (r.stdout.strip().splitlines() or [''])[-1]
    verdict('조종면 가족의 트리에서 variant_check 가 모두 통과한다', r.returncode == 0 and '실패 0' in last, last[:200])

    note('[upstream]')
    # the plain plant upstream carries, written into a scratch tree trimmed of
    # the generic family, is the tree's complete member element for element
    scratch = tempfile.mkdtemp(prefix='airship_upstream_')

    for rel in gen.TREE_DIRS:
        shutil.copytree(os.path.join(ROOT, rel), os.path.join(scratch, rel))

    for rel in gen.TREE_FILES:
        os.makedirs(os.path.dirname(os.path.join(scratch, rel)), exist_ok=True)
        shutil.copy(os.path.join(ROOT, rel), os.path.join(scratch, rel))

    vehicle = gen.load_vehicle('vehicles/airship_generic.json')

    for m in vehicle.family.members:
        if os.path.isdir(os.path.join(scratch, gen.TREE_DIRS[0], m)):
            shutil.rmtree(os.path.join(scratch, gen.TREE_DIRS[0], m))

    for f in os.listdir(os.path.join(scratch, gen.TREE_DIRS[1])):
        if f[:4].isdigit() and 2500 <= int(f[:4]) <= vehicle.decade + 7:      # the fixed twin's 2500-2505 too
            os.remove(os.path.join(scratch, gen.TREE_DIRS[1], f))

    written, _ = gen.write_upstream(vehicle, scratch, ROOT)
    problems = gen.twin_check(vehicle, scratch, ROOT)
    # the second write through the CLI: nothing to write, the twin check OK
    import io, contextlib
    out = io.StringIO()

    with contextlib.redirect_stdout(out):
        rc = gen.main(['vehicles/airship_generic.json', '--root', scratch, '--upstream', ROOT])

    again = [] if rc == 0 and 'wrote nothing' in out.getvalue() and 'twin check OK' in out.getvalue() else ['cli: %s' % out.getvalue()[-200:]]
    plain = open(os.path.join(scratch, gen.TREE_DIRS[0], vehicle.name, 'model.sdf')).read()
    fixed = open(os.path.join(scratch, gen.TREE_DIRS[0], vehicle.family.fixed[0], 'model.sdf')).read()
    verdict('upstream 모양의 완전 플랜트 둘 (몸체 + 평범한 이름, merge include 두 단: 벡터링과 고정 쌍둥이) 은 트리의 완전 멤버와 원소 단위로 '
            '같고 에어프레임의 기본값이 같으며, 두 번째 쓰기 (CLI) 는 아무것도 바꾸지 않는다',
            sorted(written) == sorted([vehicle.family.base, vehicle.name, '%d_gz_%s' % (vehicle.decade, vehicle.name),
                                       vehicle.family.fixed[0], '2500_gz_%s' % vehicle.family.fixed[0]])
            and vehicle.decade == gen.GENERIC_DECADE and gen.MASK['am_fin_drag'] == 7
            and not problems and not again and plain.count('<include') == 1 and 'model://%s</uri>' % vehicle.family.base in plain
            and fixed.count('<include') == 1 and 'model://%s</uri>' % vehicle.family.base in fixed
            and '--' not in plain.split('<!--', 1)[1].split('-->', 1)[0],
            '; '.join(problems + again) if problems or again else 'wrote %s' % ', '.join(written))
    # the guard: an untrimmed copy of the tree is refused and left as it was; the modes exclude each other
    untrimmed = tempfile.mkdtemp(prefix='airship_untrimmed_')

    for rel in gen.TREE_DIRS:
        shutil.copytree(os.path.join(ROOT, rel), os.path.join(untrimmed, rel))

    for rel in gen.TREE_FILES:
        os.makedirs(os.path.dirname(os.path.join(untrimmed, rel)), exist_ok=True)
        shutil.copy(os.path.join(ROOT, rel), os.path.join(untrimmed, rel))

    before = snapshot(os.path.join(untrimmed, gen.TREE_DIRS[0]))
    refused = ''

    try:
        gen.write_upstream(vehicle, untrimmed, ROOT)
    except SystemExit as why:
        refused = str(why)

    try:
        gen.write_upstream(vehicle, ROOT, ROOT)
        same_root = ''
    except SystemExit as why:
        same_root = str(why)

    try:
        with contextlib.redirect_stderr(io.StringIO()):
            gen.main(['vehicles/airship_generic.json', '--check', '--overlay'])
        exclusive = False
    except SystemExit as why:
        exclusive = why.code == 2

    verdict('--upstream 은 트림되지 않은 트리 (가족 멤버가 남은) 와 MEMBERS_ROOT 와 같은 트리를 거부하고 아무것도 쓰지 않으며, '
            '--check/--overlay/--upstream 은 서로 배타적이다',
            'airship_base_am' in refused and 'same tree' in same_root and before == snapshot(os.path.join(untrimmed, gen.TREE_DIRS[0]))
            and exclusive, (refused + ' | ' + same_root)[:200])
    shutil.rmtree(untrimmed)
    # the overlay: each family under the rigid naming, a hundred above its
    # decade, self-contained, an exact twin of the tree's family (and so of
    # the plain plant upstream carries), generated twice without a change
    overlay = tempfile.mkdtemp(prefix='airship_overlay_')
    bad = []

    for name in ('airship_generic', 'airship_indoor'):
        v = gen.load_vehicle('vehicles/%s.json' % name)
        ofam = v.overlay_family
        _, written, entries = gen.generate_overlay(v, overlay=overlay, template_root=ROOT)
        _, again, entries_again = gen.generate_overlay(v, overlay=overlay, template_root=ROOT)

        with gen.at_root(overlay):
            o_merged = gen.canonical(gz_model.merged(ofam.am_fin_drag))[3]
            o_defaults = gz_model.airframe_defaults(ofam.am_fin_drag)
            o_bare = gz_model.airframe_defaults(ofam.bare)
            o_chain = gz_model.family(ofam.am_fin_drag)
            o_tokens = gz_model.family_tokens(ofam.bare)

        with gen.at_root(ROOT):
            t_merged = gen.canonical(gz_model.merged(v.family.am_fin_drag))[3]
            t_defaults = gz_model.airframe_defaults(v.family.am_fin_drag)
            t_bare = gz_model.airframe_defaults(v.family.bare)

        if not (len(written) == 6 and entries == 5 and again == [] and entries_again == 0):
            bad.append('%s: wrote %s, entries %d; again %s %d' % (name, written, entries, again, entries_again))

        if not (o_merged == t_merged and o_defaults == t_defaults and o_bare == t_bare):
            bad.append('%s: not the tree\'s twin (models %s, complete defaults %s, bare defaults %s)'
                       % (name, o_merged == t_merged, o_defaults == t_defaults, o_bare == t_bare))

        if not (o_chain[0] == ofam.base_am and o_tokens == [gz_model.RIGID] and o_defaults['SYS_AUTOSTART'] == v.decade + gen.OVERLAY_SHIFT + 7
                if 'SYS_AUTOSTART' in o_defaults else o_chain[0] == ofam.base_am and o_tokens == [gz_model.RIGID]):
            bad.append('%s: chain %s, tokens %s' % (name, o_chain, o_tokens))

    # the calculator finds the overlay's members (the bare one under its rigid
    # name) and rewrites their blocks without a change
    before = {f: open(os.path.join(overlay, 'airframes', f)).read() for f in os.listdir(os.path.join(overlay, 'airframes'))}

    import airship_gains as gains

    with gen.at_root(overlay):
        rc = gains.main(['airship_indoor', '--write'])

    after = {f: open(os.path.join(overlay, 'airframes', f)).read() for f in os.listdir(os.path.join(overlay, 'airframes'))}
    verdict('airship_gains 가 오버레이의 멤버 (bare 는 _rigid 이름) 를 찾아 블록을 다시 쓰고, 아무것도 바뀌지 않는다',
            rc == 0 and before == after, ', '.join(f for f in before if before[f] != after.get(f)) or 'same')
    # the overlay as MEMBERS_ROOT: the plain plant written from it is the one written from the tree, byte for byte
    scratch2 = tempfile.mkdtemp(prefix='airship_upstream2_')

    for rel in gen.TREE_DIRS:
        shutil.copytree(os.path.join(scratch, rel), os.path.join(scratch2, rel))

    for rel in gen.TREE_FILES:
        os.makedirs(os.path.dirname(os.path.join(scratch2, rel)), exist_ok=True)
        shutil.copy(os.path.join(scratch, rel), os.path.join(scratch2, rel))

    for name in (vehicle.family.base, vehicle.name, vehicle.family.fixed[0]):
        shutil.rmtree(os.path.join(scratch2, gen.TREE_DIRS[0], name))

    os.remove(os.path.join(scratch2, gen.TREE_DIRS[1], '%d_gz_%s' % (vehicle.decade, vehicle.name)))
    os.remove(os.path.join(scratch2, gen.TREE_DIRS[1], '2500_gz_%s' % vehicle.family.fixed[0]))
    from_overlay, _ = gen.write_upstream(vehicle, scratch2, overlay)
    problems2 = gen.twin_check(vehicle, scratch2, overlay)
    # the same elements (the overlay's copies carry renamed comments, so not the same bytes)
    import xml.etree.ElementTree as ET
    same = all(gen.canonical(ET.parse(os.path.join(scratch, gen.TREE_DIRS[0], n, 'model.sdf')).getroot())
               == gen.canonical(ET.parse(os.path.join(scratch2, gen.TREE_DIRS[0], n, 'model.sdf')).getroot())
               for n in (vehicle.family.base, vehicle.name))
    verdict('오버레이를 MEMBERS_ROOT 로 한 --upstream 은 트리에서 만든 것과 원소 단위로 같은 몸체와 완전 플랜트를 쓰고 (고정 쌍둥이는 '
            '오버레이에 에어프레임이 없어 건너뜀) 쌍둥이 검사를 통과한다',
            sorted(from_overlay) == sorted([vehicle.family.base, vehicle.name, '%d_gz_%s' % (vehicle.decade, vehicle.name)])
            and not problems2 and same, '; '.join(problems2) if problems2 else 'wrote %s' % ', '.join(from_overlay))
    shutil.rmtree(scratch2)

    decades = open(os.path.join(overlay, 'airframes', 'decades')).read().split()
    world = open(os.path.join(overlay, 'worlds', gen.OVERLAY_WORLD_NAME + '.sdf')).read()
    docs = open(os.path.join(overlay, 'vehicles.md')).read()
    verdict('오버레이: 두 가족이 rigid 이름과 +100 번호로 자체 완결되게 생성되고 (세계 lta_lab, decades, vehicles.md), 트리 가족과 '
            'merged 모델·에어프레임 기본값이 같으며, 두 번째 생성은 아무것도 바꾸지 않는다',
            not bad and decades == ['2620', 'airship_vectored_independent_rigid', '2630', 'airship_indoor_rigid']
            and '<world name="%s">' % gen.OVERLAY_WORLD_NAME in world and 'airship_indoor_rigid_3::base_link' in world
            and '`gz_airship_indoor_am_fin_drag` | 2637 |' in docs and docs.count('### ') == 12
            and 'make px4_sitl' not in docs and '../frames_airship' not in docs,
            '; '.join(bad) if bad else 'decades %s' % ' '.join(decades))
    shutil.rmtree(overlay)
    shutil.rmtree(scratch)

    # variant_check on a populated overlay: a vehicle the tree does not carry
    # (so the two homes do not collide), its file placed where the checks read
    # vehicle files, generated into its own overlay; every fact passes
    with open('vehicles/airship_indoor.json') as handle:
        spec = json.load(handle)

    spec.update(name='airship_scratch', title='Scratch Airship', decade=2570)
    overlay2 = tempfile.mkdtemp(prefix='airship_overlay2_')
    vehicle_file = os.path.join(gz_model.VEHICLES, 'airship_scratch.json')

    try:
        with open(vehicle_file, 'w') as handle:
            json.dump(spec, handle)

        gen.generate_overlay(gen.load_vehicle(vehicle_file), overlay=overlay2, template_root=ROOT)
        env = dict(os.environ, PX4_ROOT=ROOT, AIRSHIP_OVERLAY=overlay2)
        r = subprocess.run([sys.executable, os.path.join(HERE, 'variant_check.py')], capture_output=True, text=True, env=env)
    finally:
        os.remove(vehicle_file)
        shutil.rmtree(overlay2)

    lines = r.stdout.strip().splitlines()
    verdict('오버레이에 가족이 하나 있을 때 variant_check 가 모든 사실을 통과한다 (멤버 46 개: 트리 40 + 오버레이 6, 몸체 둘은 사실 10 의 것)',
            r.returncode == 0 and lines and lines[-1].endswith('실패 0') and '모델 46 개' in r.stdout,
            (lines[-1] if lines else r.stderr[-300:]) + ' | ' + '; '.join(l.strip()[:90] for l in lines if l.strip().startswith('실패'))[:400])

    note('[gains]')
    import airship_gains as G
    ref = G.REF_PLANT
    verdict('닻 플랜트는 2026-10-07 에 10.97 m 파일에서 읽은 값이다 (N 54, I 129.15 + 192.24, k 228.7) 와 그 이득 0.7/0.09',
            (ref['N'], round(ref['I_rigid'], 2), round(ref['rr'], 2), round(ref['k'], 1)) == (54.0, 129.15, 192.24, 228.7)
            and (G.REF_GAINS['AS_YAWRATE_P'], G.REF_GAINS['AS_YAWRATE_I']) == (0.7, 0.09), str(ref))
    r = run(['airship_gains.py', 'airship_vectored_independent'], root)
    vals = {k: float(v) for k, v in (l.split()[:2] for l in r.stdout.splitlines() if l.strip().startswith(('AS_', 'FF_')))}
    p = G.plant('airship_vectored_independent')
    w_ref = G.REF_PLANT['N'] * 0.7 / (G.REF_PLANT['I_rigid'] + G.REF_PLANT['rr'])
    w = p['N'] * vals.get('AS_YAWRATE_P', 0.0) / (p['I_rigid'] + p['rr'])
    t = vals.get('AS_YAWRATE_P', 0.0) / max(vals.get('AS_YAWRATE_I', 0.0), 1e-9)
    measured = G.stored_measurement(p)[0]
    k = measured if measured is not None else p['k']
    verdict('generic 기체: 닻의 대역폭과 적분 시간, 기울기는 보관된 측정 (없으면 씨앗) k/N, 씨앗은 측정의 3 % 안',
            r.returncode == 0 and abs(w - w_ref) < 1e-3 * w_ref and abs(t - 0.7 / 0.09) < 1e-3 * 0.7 / 0.09
            and abs(vals.get('AS_YR_FF_SLP', 0.0) - k / p['N']) < 1e-3 * k / p['N'] and abs(p['k'] - k) <= 0.03 * k,
            'w %.4f/%.4f rad/s, P/I %.3f s, slope %.4g vs %.4g (%s), seed %.4g' % (w, w_ref, t, vals.get('AS_YR_FF_SLP', 0.0), k / p['N'],
                                                                              'measured' if measured is not None else 'seed', p['k'] / p['N']))
    r = run(['airship_gains.py', 'airship_indoor', '--write'], root)
    first = snapshot(airframes)
    r2 = run(['airship_gains.py', 'airship_indoor', '--write'], root)
    second = snapshot(airframes)
    bare = open(os.path.join(airframes, '2530_gz_airship_indoor')).read()
    verdict('indoor 의 세 에어프레임에 값 블록을 쓰고 (출처 한 줄, 닫는 표식 없음), 두 번째 쓰기는 같다',
            r.returncode == 0 and r2.returncode == 0 and first == second and bare.count('airship_gains.py') == 1
            and bare.count(G.MARK_OPEN) == 1 and '---' not in bare.split(G.MARK_OPEN)[1]
            and 'AS_YAWRATE_P' in bare and all('AS_YR_FF_SLP' in open(os.path.join(airframes, f)).read() for f in ('2532_gz_airship_indoor_fin', '2537_gz_airship_indoor_am_fin_drag')),
            (r.stderr or r.stdout)[-200:])
    return summary()


if __name__ == '__main__':
    sys.exit(main())
