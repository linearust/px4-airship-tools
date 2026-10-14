#!/usr/bin/env python3
"""airship_family_gen.py and airship_gains.py against the tree, no gz.

    python3 airship_family_gen_test.py

1. The committed overlay regenerated into a copy of itself (the five
   vehicles and the Cloudship, --overlay) writes and changes nothing; its
   content is the split's shape (seven decades under the rigid naming, the
   generic's fixed twin, the copied Cloudship), with the indoor family's
   facts; a surfaces family generated beside them, from the overlay's own
   generic rigid template, passes every fact of variant_check.py.
2. --upstream from the overlay into a tree copy trimmed of a plain plant
   writes it back as the tree holds it (the twin check, then the copy is the
   tree byte for byte); a tree still holding a family member is refused, and
   the two modes exclude each other.
3. The split's end state replayed: a tree copy trimmed as the procedure
   trims it (docs/upstream_split_design_20261012.md, section 7, step (6)) and
   written back from the overlay is the tree again and passes every fact of
   variant_check.py (43 models: the overlay's 40 and the plain plants).
4. airship_gains.py: its anchor (REF_PLANT, the 10.97 m hull the gains were
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
# the file every posix airship airframe sources, which resolving an airframe's defaults (the twin checks) reads
SOURCED = ('ROMFS/px4fmu_common/init.d/rc.airship_defaults',)


def scratch_tree():
    root = tempfile.mkdtemp(prefix='airship_family_test_')

    for rel in COPIED:
        shutil.copytree(os.path.join(ROOT, rel), os.path.join(root, rel))

    for rel in FILES + SOURCED:
        os.makedirs(os.path.dirname(os.path.join(root, rel)), exist_ok=True)
        shutil.copy(os.path.join(ROOT, rel), os.path.join(root, rel))

    # the module yamls: gz_model.absent_params() asks a tree which airship
    # parameters its firmware defines, and a scratch tree without them would
    # read as a tree without any
    for rel in gz_model.PARAM_YAMLS.values():
        os.makedirs(os.path.dirname(os.path.join(root, rel)), exist_ok=True)
        shutil.copy(os.path.join(ROOT, rel), os.path.join(root, rel))

    return root


def trim(root):
    """The split's trim (docs/upstream_split_design_20261012.md, section 7,
    step (6)) on a scratch tree: every airship family's model directories,
    the posix 25xx gz airframes (the classic Cloudship stays), their CMake
    entries and the decade table, the lta world's buoyancy entries, and the
    docs' Airship sections, index rows and simulation bullets. What
    --upstream then writes back is what the tree carries after the split."""
    models = os.path.join(root, gen.TREE_DIRS[0])

    for d in os.listdir(models):
        if d.startswith(('airship_', 'cloudship')):
            shutil.rmtree(os.path.join(models, d))

    airframes = os.path.join(root, gen.TREE_DIRS[1])

    for f in os.listdir(airframes):
        if re.match(r'25\d\d_gz_', f):
            os.remove(os.path.join(airframes, f))

    edits = ((os.path.join(airframes, 'CMakeLists.txt'), ((r'\n\t# Airship gz decades.*?(?=\n\t\d{4}_)', ''), (r'\t25\d\d_gz_[^\n]*\n', ''))),
             (os.path.join(root, gen.TREE_FILES[0]), ((r' *<enable>(airship_|cloudship)[^\n]*\n', ''),)),
             (os.path.join(root, gen.TREE_FILES[1]), ((r'^\| \[[^\n]*`make px4_sitl gz_(airship_|cloudship)[^\n]*\n', ''),)),
             (os.path.join(root, gen.TREE_FILES[3]), ((r'(?:^- \[[^\n]*vehicles\.md#[^\n]*\n)+\n?', ''),)))

    for path, subs in edits:
        t = open(path).read()

        for pattern, repl in subs:
            t = re.sub(pattern, repl, t, flags=re.M | re.S)

        open(path, 'w').write(t)

    path = os.path.join(root, gen.TREE_FILES[2])
    t = open(path).read()
    start = t.index('\n## Airship\n') + 1
    end = re.search(r'^## ', t[start + 1:], re.M)
    end = start + 1 + end.start() if end else len(t)
    open(path, 'w').write(t[:start] + re.sub(r'^### .*?(?=^### |\Z)', '', t[start:end], flags=re.M | re.S) + t[end:])


def run(args, root, overlay=None):
    env = dict(os.environ, PX4_ROOT=root)

    if overlay:
        env['AIRSHIP_OVERLAY'] = overlay

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


OVERLAY = gz_model.OVERLAY
VEHICLES = ('airship_generic', 'airship_indoor', 'airship_aurora', 'airship_jaxa', 'airship_mpi')
DECADES = ['2600', 'airship_fixed_rigid', '2610', 'cloudship_rigid', '2620', 'airship_vectored_independent_rigid',
           '2630', 'airship_indoor_rigid', '2640', 'airship_aurora_rigid', '2650', 'airship_jaxa_rigid', '2660', 'airship_mpi_rigid']


def scratch_overlay():
    """A copy of the committed overlay."""
    root = tempfile.mkdtemp(prefix='airship_overlay_')
    shutil.rmtree(root)
    shutil.copytree(OVERLAY, root)
    return root


def variant_check(root, overlay):
    env = dict(os.environ, PX4_ROOT=root, AIRSHIP_OVERLAY=overlay)
    r = subprocess.run([sys.executable, os.path.join(HERE, 'variant_check.py')], capture_output=True, text=True, env=env)
    lines = r.stdout.strip().splitlines()
    failed = '; '.join(l.strip()[:120] for l in lines if l.strip().startswith('실패'))[:600]
    return r, (lines[-1] if lines else r.stderr[-300:]) + (' | ' + failed if failed else '')


def main():
    import io, contextlib
    vehicle = gen.load_vehicle('vehicles/airship_generic.json')
    loaded = {v: gen.load_vehicle('vehicles/%s.json' % v) for v in VEHICLES}

    note('[overlay round trip]')
    # the committed overlay regenerated into a copy of itself: the template is
    # its own generic rigid family, the lab world the tree's; nothing is written
    overlay = scratch_overlay()
    before = snapshot(overlay)
    runs = [run(['airship_family_gen.py', 'vehicles/%s.json' % v, '--overlay'], ROOT, overlay) for v in VEHICLES]
    runs.append(run(['airship_family_gen.py', 'cloudship', '--overlay'], ROOT, overlay))
    after = snapshot(overlay)
    verdict('다섯 기체와 cloudship 의 --overlay 를 오버레이 사본에 다시 돌리면 아무것도 쓰지 않고 바꾸지 않는다 (템플릿은 오버레이의 generic rigid '
            '가족, 월드는 트리의)',
            all(r.returncode == 0 and 'airframes written: none' in r.stdout and 'world entries added 0' in r.stdout for r in runs)
            and after == before,
            '; '.join((r.stderr or r.stdout)[-120:] for r in runs if r.returncode or 'written: none' not in r.stdout)
            or ', '.join(k for k in after if before.get(k) != after[k])[:300] or '같음')
    decades_text = open(os.path.join(OVERLAY, 'airframes', 'decades')).read()
    decades = decades_text.split()
    world = open(os.path.join(OVERLAY, 'worlds', gen.OVERLAY_WORLD_NAME + '.sdf')).read()
    docs = open(os.path.join(OVERLAY, 'vehicles.md')).read()
    models = sorted(os.listdir(os.path.join(OVERLAY, 'models')))
    airframes = sorted(f for f in os.listdir(os.path.join(OVERLAY, 'airframes')) if '_gz_' in f)
    fixed_bare = open(os.path.join(OVERLAY, 'airframes', '2600_gz_airship_fixed_rigid')).read()
    fixed_drag = open(os.path.join(OVERLAY, 'airframes', '2601_gz_airship_fixed_drag')).read()
    verdict('오버레이: 일곱 가족이 rigid 이름과 +100 번호로 (decades 7 줄, 모델 50, 에어프레임 40, lta_lab 에 모델마다 bare 와 _0.._3 의 enable 200, '
            'vehicles.md 절 40 행 40, make 명령과 트리 링크 없음), generic 의 고정 쌍둥이 (2600/2601/2604/2605) 가 트리의 2500 가족에서 복사되어 있다',
            decades == DECADES and len(models) == 50 and all(os.path.isfile(os.path.join(OVERLAY, 'models', m, 'model.sdf')) for m in models)
            and len(airframes) == 40 and '<world name="%s">' % gen.OVERLAY_WORLD_NAME in world and world.count('<enable>') == 200
            and all('<enable>%s%s::base_link' % (m, i) in world for m in models if not m.endswith(('_base_am', '_base_rigid'))
                    for i in ('', '_0', '_3'))
            and docs.count('### ') == 40 and len(re.findall(r'^\| \[', docs, re.M)) == 40
            and '`gz_airship_indoor_am_fin_drag` | 2637 |' in docs and '`gz_airship_fixed_am_drag` | 2605 |' in docs
            and '### Generic Airship (Rigid Hull)\n' in docs and '### Generic Airship (Hull Drag)\n' in docs
            and 'make px4_sitl' not in docs and '../frames_airship' not in docs
            and '# @name Generic Airship (Rigid Hull)\n' in fixed_bare and 'PX4_SIM_MODEL=${PX4_SIM_MODEL:=airship_fixed_rigid}' in fixed_bare
            and 'param set-default SIM_GZ_SV_MINA4 -' in fixed_bare and 'airframes/2600_gz_airship_fixed_rigid\n' in fixed_drag
            and 'PX4_SIM_MODEL=${PX4_SIM_MODEL:=airship_fixed_drag}' in fixed_drag,
            'decades %s; models %d, airframes %d, enables %d, sections %d' % (' '.join(decades), len(models), len(airframes),
                                                                              world.count('<enable>'), docs.count('### ')))
    chains = {v: gz_model.family(loaded[v].overlay_family.am_fin_drag) for v in VEHICLES}
    verdict('오버레이 가족의 사슬은 _base_am 에서 완전 멤버까지고 완전 멤버의 번호는 십 자리 + 7, bare 의 토큰은 rigid 하나다; cloudship 의 사슬은 '
            '_am 부터 완전 멤버까지다',
            all(chains[v][0] == loaded[v].overlay_family.base_am and chains[v][-1] == loaded[v].overlay_family.am_fin_drag
                and gz_model.airframe(loaded[v].overlay_family.am_fin_drag)[0] == loaded[v].decade + gen.OVERLAY_SHIFT + 7
                and gz_model.family_tokens(loaded[v].overlay_family.bare) == [gz_model.RIGID] for v in VEHICLES)
            and gz_model.family('cloudship_am_fin_drag') == ['cloudship_am', 'cloudship_am_fin', 'cloudship_am_fin_drag']
            and gz_model.family_tokens('cloudship_rigid') == [gz_model.RIGID], str(chains))
    rigid = open(os.path.join(OVERLAY, 'models', 'cloudship_rigid', 'model.sdf')).read()
    drag = open(os.path.join(OVERLAY, 'models', 'cloudship_drag', 'model.sdf')).read()
    bare = open(os.path.join(OVERLAY, 'airframes', '2610_gz_cloudship_rigid')).read()
    fin = open(os.path.join(OVERLAY, 'airframes', '2612_gz_cloudship_fin')).read()
    verdict('cloudship 가족: 트리에 있던 여섯 멤버가 bare 만 cloudship_rigid 로 (모델, 에어프레임 2610..2617, 월드, decades, 문서 절 6)',
            '<model name="cloudship_rigid">' in rigid and 'models/cloudship/cloudship.sdf.jinja' in rigid and '2507_cloudship' in rigid
            and 'wrap cloudship_rigid' in rigid and '<uri>model://cloudship_rigid</uri>' in drag
            and '# @name Cloudship (Rigid Hull)\n' in bare and 'PX4_SIM_MODEL=${PX4_SIM_MODEL:=cloudship_rigid}' in bare
            and 'airframes/2610_gz_cloudship_rigid\n' in fin and 'PX4_SIM_MODEL=${PX4_SIM_MODEL:=cloudship_fin}' in fin
            and 'param set-default AS_YR_FF_SLP 12.74' in fin and 'cloudship_rigid_3::base_link' in world
            and '`gz_cloudship_rigid` | 2610 |' in docs and '`gz_cloudship_am_fin_drag` | 2617 |' in docs
            and '### Cloudship (Rigid Hull)\n' in docs and docs.count('### Cloudship') == 6 and 'GZ_TARGET=gz_cloudship_rigid ./gz_run.sh' in docs, '')
    # the calculator finds the overlay's members (the bare one under its rigid
    # name) and rewrites their blocks without a change
    import airship_gains as gains
    before = snapshot(os.path.join(overlay, 'airframes'))

    with gen.at_root(overlay):
        rc = gains.main(['airship_indoor', '--write'])

    after = snapshot(os.path.join(overlay, 'airframes'))
    verdict('airship_gains 가 오버레이의 멤버 (bare 는 _rigid 이름) 를 찾아 블록을 다시 쓰고, 아무것도 바뀌지 않는다',
            rc == 0 and before == after, ', '.join(f for f in before if before[f] != after.get(f)) or 'same')

    note('[indoor family]')
    dirs = [m for m in models if m.startswith('airship_indoor')]
    files = [f for f in airframes if '_gz_airship_indoor' in f]
    verdict('indoor 가족은 오버레이에: 모델 디렉터리 8, lta_lab 의 enable 30, 에어프레임 2630/2631/2632/2634/2636/2637, decades 줄, 문서 절 6 행 6',
            len(dirs) == 8 and world.count('<enable>airship_indoor') == 30
            and [f.split('_')[0] for f in files] == ['2630', '2631', '2632', '2634', '2636', '2637']
            and '2630 airship_indoor_rigid\n' in decades_text and docs.count('### Indoor Airship') == 6 and docs.count('| [Indoor Airship') == 6,
            '%s; %s' % (', '.join(dirs), ', '.join(files)))
    r = run(['generic_hull_design.py', '--spec', 'vehicles/airship_indoor.json'], ROOT)
    verdict('indoor 설계가 요구 조건을 만족한다', r.returncode == 0 and 'all requirements met' in r.stdout,
            (r.stdout.strip().splitlines() or [r.stderr[-200:]])[-1][:200])

    note('[surfaces option]')
    # a vectored family with flaps on its fin members, under a name neither home
    # carries, generated into the overlay copy from its generic rigid template:
    # the scheme check passes with it, a second generation changes nothing
    spec = json.load(open(os.path.join(HERE, 'vehicles', 'airship_aurora.json')))
    spec.update(name='airship_scratch', title='Scratch Airship', decade=2570, surfaces=True)
    vehicle_file = os.path.join(gz_model.VEHICLES, 'airship_scratch.json')

    try:
        with open(vehicle_file, 'w') as handle:
            json.dump(spec, handle)

        v = gen.load_vehicle(vehicle_file)
        _, written, entries = gen.generate_overlay(v, overlay=overlay, template_root=ROOT)
        middle = snapshot(overlay)
        _, again, entries_again = gen.generate_overlay(v, overlay=overlay, template_root=ROOT)
        after = snapshot(overlay)
        r, last = variant_check(ROOT, overlay)
    finally:
        os.remove(vehicle_file)

    fin = open(os.path.join(overlay, 'models', 'airship_scratch_fin', 'model.sdf')).read()
    member = open(os.path.join(overlay, 'airframes', '2677_gz_airship_scratch_am_fin_drag')).read()
    verdict('조종면 가족 (2670, 오버레이 사본에): 핀 멤버에 플랩 넷 (servo_2..5, 핀마다 제 플랩 조인트), 에어프레임에 조종면 넷과 틸트 서보 Servo5/6, '
            '월드 항목 30, 두 번째 생성은 아무것도 바꾸지 않는다',
            written and entries == 30 and again == [] and entries_again == 0 and middle == after and fin.count('<link name="flap_') == 4
            and sorted(re.findall(r'<sub_topic>(servo_\d)</sub_topic>', fin)) == ['servo_2', 'servo_3', 'servo_4', 'servo_5']
            and fin.count('<control_joint_name>') == 4 and 'param set-default CA_SV_CS_COUNT 4' in member
            and 'param set-default SIM_GZ_SV_FUNC1 205' in member and '# @output Servo5 starboard pod tilt' in member,
            'wrote %s, entries %d; again %s %d' % (written, entries, again, entries_again))
    verdict('그 오버레이 (가족 하나 더) 와 트림된 트리에서 variant_check 가 모든 사실을 통과한다 (모델 49 개: 오버레이 46 + 평범한 플랜트 3)',
            r.returncode == 0 and last.startswith('통과') and '실패 0' in last and '모델 49 개' in r.stdout, last)
    shutil.rmtree(overlay)

    note('[upstream]')
    # the generic vehicle's plain plants, written from the overlay's members
    # into a tree copy trimmed of them: twins of the complete members, the
    # copy the tree again; a second write through the CLI changes nothing
    scratch = scratch_tree()

    for m in (vehicle.family.base, vehicle.name, vehicle.family.fixed[0]):
        shutil.rmtree(os.path.join(scratch, gen.TREE_DIRS[0], m))

    for f in ('%d_gz_%s' % (vehicle.decade, vehicle.name), '2500_gz_%s' % vehicle.family.fixed[0]):
        os.remove(os.path.join(scratch, gen.TREE_DIRS[1], f))

    written, _ = gen.write_upstream(vehicle, scratch, OVERLAY)
    upstream_written = sorted([vehicle.family.base, vehicle.name, '%d_gz_%s' % (vehicle.decade, vehicle.name),
                               vehicle.family.fixed[0], '2500_gz_%s' % vehicle.family.fixed[0]])
    problems = gen.twin_check(vehicle, scratch, OVERLAY)
    out = io.StringIO()

    with contextlib.redirect_stdout(out):
        rc = gen.main(['vehicles/airship_generic.json', '--root', scratch, '--upstream', OVERLAY])

    again = [] if rc == 0 and 'wrote nothing' in out.getvalue() and 'twin check OK' in out.getvalue() else ['cli: %s' % out.getvalue()[-200:]]
    plain = open(os.path.join(scratch, gen.TREE_DIRS[0], vehicle.name, 'model.sdf')).read()
    fixed = open(os.path.join(scratch, gen.TREE_DIRS[0], vehicle.family.fixed[0], 'model.sdf')).read()
    d = differs(scratch)
    verdict('upstream 모양의 완전 플랜트 둘 (몸체 + 평범한 이름, merge include 두 단: 벡터링과 고정 쌍둥이) 은 오버레이의 완전 멤버와 원소 단위로 '
            '같고 에어프레임의 기본값이 같으며 (쌍둥이 검사), 두 번째 쓰기 (CLI) 는 아무것도 바꾸지 않고, 사본은 트리와 바이트 단위로 같다',
            sorted(written) == upstream_written and vehicle.decade == gen.GENERIC_DECADE and gen.MASK['am_fin_drag'] == 7
            and not problems and not again and not d and plain.count('<include') == 1 and 'model://%s</uri>' % vehicle.family.base in plain
            and fixed.count('<include') == 1 and 'model://%s</uri>' % vehicle.family.base in fixed
            and '--' not in plain.split('<!--', 1)[1].split('-->', 1)[0],
            '; '.join(problems + again + d)[:400] if problems or again or d else 'wrote %s' % ', '.join(written))
    shutil.rmtree(scratch)
    # the guard: a tree copy still holding a family member is refused and left as it was; the modes exclude each other
    untrimmed = scratch_tree()
    shutil.copytree(os.path.join(OVERLAY, 'models', vehicle.overlay_family.base_am),
                    os.path.join(untrimmed, gen.TREE_DIRS[0], vehicle.family.base_am))
    before = snapshot(os.path.join(untrimmed, gen.TREE_DIRS[0]))
    refused = ''

    try:
        gen.write_upstream(vehicle, untrimmed, OVERLAY)
    except SystemExit as why:
        refused = str(why)

    try:
        gen.write_upstream(vehicle, ROOT, ROOT)
        same_root = ''
    except SystemExit as why:
        same_root = str(why)

    try:
        with contextlib.redirect_stderr(io.StringIO()):
            gen.main(['vehicles/airship_generic.json', '--overlay', '--upstream', OVERLAY])
        exclusive = False
    except SystemExit as why:
        exclusive = why.code == 2

    verdict('--upstream 은 가족 멤버가 남은 트리 (airship_base_am) 와 MEMBERS_ROOT 와 같은 트리를 거부하고 아무것도 쓰지 않으며, '
            '--overlay 와 --upstream 은 서로 배타적이다',
            'airship_base_am' in refused and 'same tree' in same_root and before == snapshot(os.path.join(untrimmed, gen.TREE_DIRS[0]))
            and exclusive, (refused + ' | ' + same_root)[:200])
    shutil.rmtree(untrimmed)

    note('[cloudship]')
    # the Cloudship's plain plant from the overlay into a tree copy trimmed of
    # it: self-contained (no include), the _am copy with the fin and drag
    # blocks, the airframe the rigid bare's text with the complete member's
    # block and no variant notes; the twin check holds, the copy is the tree
    # again, a second write (CLI, --plants honoured) changes nothing
    scratch3 = scratch_tree()
    shutil.rmtree(os.path.join(scratch3, gen.TREE_DIRS[0], 'cloudship'))
    os.remove(os.path.join(scratch3, gen.TREE_DIRS[1], '2510_gz_cloudship'))
    written3, _ = gen.write_upstream(gen.CLOUDSHIP, scratch3, OVERLAY)
    problems3 = gen.twin_check(gen.CLOUDSHIP, scratch3, OVERLAY)
    out = io.StringIO()

    with contextlib.redirect_stdout(out):
        rc = gen.main(['cloudship', '--root', scratch3, '--upstream', OVERLAY, '--plants', 'cloudship'])

    again3 = [] if rc == 0 and 'wrote nothing' in out.getvalue() and 'twin check OK' in out.getvalue() else ['cli: %s' % out.getvalue()[-200:]]
    plain = open(os.path.join(scratch3, gen.TREE_DIRS[0], 'cloudship', 'model.sdf')).read()
    config = open(os.path.join(scratch3, gen.TREE_DIRS[0], 'cloudship', 'model.config')).read()
    airframe = open(os.path.join(scratch3, gen.TREE_DIRS[1], '2510_gz_cloudship')).read()
    d = differs(scratch3)
    verdict('cloudship --upstream (오버레이에서): 자체 완결 완전 플랜트 (include 없음, 부가 질량 + 핀 넷 + 항력) 와 2510 (rigid bare 의 본문 + '
            '2617 의 블록, 변형 설명 없음, 제 그룹 CA_AIRSHIP_GRP 0) 을 쓰고 쌍둥이 검사를 통과하며, 두 번째 쓰기는 아무것도 바꾸지 않고, 사본은 트리와 같다',
            sorted(written3) == ['2510_gz_cloudship', 'cloudship'] and not problems3 and not again3 and not d
            and '<include' not in plain and '<model name="cloudship">' in plain and plain.count('gz-sim-lift-drag-system') == 4
            and plain.count('gz-sim-hydrodynamics-system') == 1 and '<fluid_added_mass>' in plain and 'Generated by px4-airship-tools' in plain
            and '<name>cloudship</name>' in config and 'complete plant' in config
            and '# @name Cloudship\n' in airframe and 'PX4_SIM_MODEL=${PX4_SIM_MODEL:=cloudship}' in airframe
            and 'param set-default AS_YAWRATE_FF 0.05' in airframe and 'param set-default CA_AIRSHIP_GRP 0' in airframe
            and 'physics mask' not in airframe and 'rigid hull' not in airframe and 'complete plant' in airframe,
            '; '.join(problems3 + again3 + d)[:400] if problems3 or again3 or d else 'wrote %s' % ', '.join(written3))
    shutil.rmtree(scratch3)

    note('[trimmed tree]')
    # the split's end state replayed: a tree copy trimmed as the procedure
    # trims it (the plain plants and their entries gone too), the plain plants
    # written back from the committed overlay (--plants honoured on the way);
    # every fact of variant_check holds, with the plain plants held to the
    # overlay's complete members (fact 17), and the copy is the tree again
    trimmed = scratch_tree()
    trim(trimmed)
    only_fixed, _ = gen.write_upstream(vehicle, trimmed, OVERLAY, plants=[vehicle.family.fixed[0]])
    rest, _ = gen.write_upstream(vehicle, trimmed, OVERLAY)
    cloud, _ = gen.write_upstream(gen.CLOUDSHIP, trimmed, OVERLAY)
    problems = gen.twin_check(vehicle, trimmed, OVERLAY) + gen.twin_check(gen.CLOUDSHIP, trimmed, OVERLAY)
    r, last = variant_check(trimmed, OVERLAY)
    models = sorted(d for d in os.listdir(os.path.join(trimmed, gen.TREE_DIRS[0])) if d.startswith(('airship_', 'cloudship')))
    posix = sorted(f for f in os.listdir(os.path.join(trimmed, gen.TREE_DIRS[1])) if f.startswith('25'))
    cmake = open(os.path.join(trimmed, gen.TREE_DIRS[1], 'CMakeLists.txt')).read()
    world = open(os.path.join(trimmed, gen.TREE_FILES[0])).read()
    index_md = open(os.path.join(trimmed, gen.TREE_FILES[1])).read()
    vehicles_md = open(os.path.join(trimmed, gen.TREE_FILES[2])).read()
    airship_md = open(os.path.join(trimmed, gen.TREE_FILES[3])).read()
    sections = re.findall(r'^### (.*)$', vehicles_md[vehicles_md.index('\n## Airship\n'):], re.M)
    d = differs(trimmed)
    verdict('트림된 트리 + 오버레이 전체: 평범한 플랜트 셋 (airship_fixed 2500, cloudship 2510, airship_vectored_independent 2520, 몸체 '
            'airship_base) 과 classic 2519 만 남고 CMake 항목·월드 15 줄·문서 항목 셋이 번호 순이며, --plants 는 그것만 쓰고, '
            'variant_check 가 모든 사실 (17) 을 통과한다 (모델 43 개: 오버레이 40 + 평범한 플랜트 3)',
            sorted(only_fixed) == ['2500_gz_airship_fixed', 'airship_base', 'airship_fixed']
            and sorted(rest) == ['2520_gz_airship_vectored_independent', 'airship_vectored_independent']
            and sorted(cloud) == ['2510_gz_cloudship', 'cloudship'] and not problems
            and models == ['airship_base', 'airship_fixed', 'airship_vectored_independent', 'cloudship']
            and posix == ['2500_gz_airship_fixed', '2510_gz_cloudship', '2519_gazebo-classic_cloudship', '2520_gz_airship_vectored_independent']
            and re.findall(r'\t(25\d\d_[^\n]*)', cmake) == posix and '# Airship gz decades' not in cmake
            and world.count('<enable>') == 15 and all('<enable>%s_3::base_link' % m in world for m in models if m != 'airship_base')
            and sections == ['Generic Airship', 'Cloudship', 'Generic Airship (Independent Vectoring)']
            and [int(n) for n in re.findall(r'`make px4_sitl gz_(?:airship_|cloudship)[^`]*`\s*\|\s*(\d{4})', index_md)] == [2500, 2510, 2520]
            and re.findall(r'^- \[([^\]]*)\]\(\.\./sim_gazebo_gz/vehicles\.md', airship_md, re.M) == sections
            and r.returncode == 0 and last == '통과 17 / 실패 0' and '모델 43 개' in r.stdout,
            last + (' | ' + '; '.join(problems) if problems else ''))
    verdict('트림하고 다시 쓴 사본은 트리와 바이트 단위로 같다: 트리의 평범한 플랜트·에어프레임·CMake 항목·월드·문서 항목은 생성기가 쓴 그대로다',
            not d, '; '.join(d)[:400] or '같음')
    shutil.rmtree(trimmed)

    note('[gains]')
    import airship_gains as G
    ref = G.REF_PLANT
    verdict('닻 플랜트는 2026-10-07 에 10.97 m 파일에서 읽은 값이다 (N 54, I 129.15 + 192.24, k 228.7) 와 그 이득 0.7/0.09',
            (ref['N'], round(ref['I_rigid'], 2), round(ref['rr'], 2), round(ref['k'], 1)) == (54.0, 129.15, 192.24, 228.7)
            and (G.REF_GAINS['AS_YAWRATE_P'], G.REF_GAINS['AS_YAWRATE_I']) == (0.7, 0.09), str(ref))
    r = run(['airship_gains.py', 'airship_vectored_independent'], ROOT)
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
    # the indoor family's three blocks, written into an overlay copy (the bare member under its rigid name), twice
    overlay = scratch_overlay()
    airframes = os.path.join(overlay, 'airframes')
    r = run(['airship_gains.py', 'airship_indoor', '--write'], ROOT, overlay)
    first = snapshot(airframes)
    r2 = run(['airship_gains.py', 'airship_indoor', '--write'], ROOT, overlay)
    second = snapshot(airframes)
    bare = open(os.path.join(airframes, '2630_gz_airship_indoor_rigid')).read()
    verdict('indoor 의 세 에어프레임 (오버레이 사본, bare 는 2630 _rigid) 에 값 블록을 쓰고 (출처 한 줄, 닫는 표식 없음), 두 번째 쓰기는 같다',
            r.returncode == 0 and r2.returncode == 0 and first == second and bare.count('airship_gains.py') == 1
            and bare.count(G.MARK_OPEN) == 1 and '---' not in bare.split(G.MARK_OPEN)[1]
            and 'AS_YAWRATE_P' in bare and all('AS_YR_FF_SLP' in open(os.path.join(airframes, f)).read()
                                               for f in ('2632_gz_airship_indoor_fin', '2637_gz_airship_indoor_am_fin_drag')),
            (r.stderr or r.stdout)[-200:])
    shutil.rmtree(overlay)
    return summary()


if __name__ == '__main__':
    sys.exit(main())
