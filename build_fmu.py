#!/usr/bin/env python3
"""Build the airship FMU firmware and check that its Airship airframe metadata matches the ROMFS.

    python3 build_fmu.py            # build px4_fmu-v6x_airship, check, stage
    python3 build_fmu.py --check    # check the existing .px4 only

The .px4 carries two copies of the airframe list. The ROMFS one is what the
FMU boots from and is rebuilt whenever an airframe file changes. The other is
airframes.xml, which QGroundControl reads when it flashes the image and uses
for its airframe picker; its build rule depends only on the generator script,
so an incremental build keeps whatever list the build directory first had.
This branch's did: the image flashed on 2026-09-25 advertised the independent
airframe as 2504, a number with no file, because airframes.xml was from
2026-08-06. Picked through QGC, that sets SYS_AUTOSTART 2504 and the next boot
finds no airframe.

This is not fixed in the tree (that is an upstream build rule, out of this
branch's scope). The build here deletes airframes.xml and the .px4 first, so
the one is regenerated and the other repacked, then decodes the metadata out
of the .px4 and fails unless its Airship entries equal the airship airframes
in the ROMFS source and every one of them is in the build's own rc.autostart
(the filtered copy the image is built from). The rest of airframes.xml is
upstream's list of every airframe in the ROMFS source, filtered only by
per-airframe "@board <board> exclude" tags and not by the CONFIG_MODULES_*
selection that decides which files the ROMFS gets, so the image also
advertises airframes it does not carry. This tool counts those but neither
checks nor fixes them.

A passing image is copied to fw_<commit>[-dirty]/, the commit and dirty mark
from git describe on the tree. Staging is refused when that describe, less
the dirty mark, is not the image's own git_identity: HEAD moved during the
build, and the name would file the image under a commit it was not built
from. The staged copy is the file to flash. `make <target> upload` rebuilds
first, and after a tree change that rebuild repacks the .px4 around the
existing airframes.xml, which its rule does not regenerate: an image nobody
checked.
"""
import base64
import json
import os
import re
import shutil
import subprocess
import sys
import zlib

import gz_model

TARGET = 'px4_fmu-v6x_airship'
BUILD = os.path.join(gz_model.ROOT, 'build', TARGET)
PX4 = os.path.join(BUILD, TARGET + '.px4')
AUTOSTART = os.path.join(BUILD, 'etc/init.d/rc.autostart')
ROMFS = gz_model.FLIGHT_AIRFRAMES
OUT = os.path.dirname(os.path.abspath(__file__))


def autostart_numbers():
    """The SYS_AUTOSTART numbers the built ROMFS maps, from the build's rc.autostart, or None without one."""
    if not os.path.exists(AUTOSTART):
        return None

    return {int(n) for n in re.findall(r'param compare SYS_AUTOSTART (\d+)', open(AUTOSTART).read())}


def romfs_airships():
    """{number: name} of the flight airframes whose @type is Airship."""
    out = {}

    for f in sorted(os.listdir(ROMFS)):
        m = re.match(r'(\d+)_', f)
        path = os.path.join(ROMFS, f)

        if m and os.path.isfile(path):
            text = open(path, encoding='utf-8').read()

            if re.search(r'^#\s*@type\s+Airship\s*$', text, re.M):
                out[int(m.group(1))] = re.search(r'^#\s*@name\s+(.+?)\s*$', text, re.M).group(1)

    return out


def metadata_airships(px4):
    """({number: name} of the Airship entries, every airframe number, git_identity) of the image's airframe_xml."""
    image = json.load(open(px4))
    xml = zlib.decompress(base64.b64decode(image['airframe_xml'])).decode()
    out = {}

    for group in re.findall(r'<airframe_group[^>]*name="Airship"[^>]*>(.*?)</airframe_group>', xml, re.S):
        for name, number in re.findall(r'<airframe name="([^"]*)" id="(\d+)"', group):
            out[int(number)] = name

    every = {int(n) for n in re.findall(r'<airframe name="[^"]*" id="(\d+)"', xml)}
    return out, every, image.get('git_identity', '?')


def check():
    if not os.path.exists(PX4):
        print('실패: .px4 없음 (%s)' % PX4)
        return False, '?'

    want = romfs_airships()
    got, every, identity = metadata_airships(PX4)
    built = autostart_numbers()
    ok = want == got and built is not None and set(want) <= built
    print('built rc.autostart has  : %s' % ('없음' if built is None else
                                            ', '.join(str(n) for n in sorted(n for n in built if n in want or n in got))))
    print('ROMFS airship airframes : %s' % ', '.join('%d %s' % kv for kv in sorted(want.items())))
    print('.px4 Airship group      : %s' % ', '.join('%d %s' % kv for kv in sorted(got.items())))
    if built is not None:
        print('.px4 가 광고하지만 이미지에 파일이 없는 기체: %d 개 (upstream 생성기, 이 도구 범위 밖)' % len(every - built))
    why = []
    if want != got:
        why.append('.px4 메타데이터가 ROMFS 와 다르다')
    if built is None:
        why.append('빌드의 rc.autostart 없음 (%s)' % AUTOSTART)
    elif not set(want) <= built:
        why.append('빌드의 rc.autostart 에 없는 ROMFS 기체: %s' % ', '.join(str(n) for n in sorted(set(want) - built)))
    print('%s: %s (%s)' % ('통과' if ok else '실패', 'Airship 그룹이 ROMFS 의 airship 기체와 같고, 모두 rc.autostart 에 있다' if ok else '; '.join(why), identity))
    return ok, identity


def stage_dir(described):
    """fw_<commit>[-dirty] from a git describe output."""
    return os.path.join(OUT, 'fw_%s' % described.rsplit('-g', 1)[-1])


def names_image(described, identity):
    """Whether a git describe --dirty output names the image whose git_identity (describe without --dirty) is given."""
    return bool(described) and described.removesuffix('-dirty') == identity


def main():
    if '--check' in sys.argv:
        return 0 if check()[0] else 1

    # airframes.xml is regenerated only if missing, and the .px4 is repacked
    # only if missing or older than its inputs, so both go
    for stale in (os.path.join(BUILD, 'airframes.xml'), PX4):
        if os.path.exists(stale):
            os.remove(stale)
            print('지웠다:', stale)

    done = subprocess.run(['make', '-C', gz_model.ROOT, TARGET], capture_output=True, text=True)

    if done.returncode != 0:
        print(done.stdout[-2000:], done.stderr[-2000:])
        print('빌드 실패')
        return 2

    ok, identity = check()

    if not ok:
        return 1

    # The image's git_identity has no dirty mark; the name must, or an image
    # built from uncommitted changes is filed under the commit it is not
    described = subprocess.run(['git', '-C', gz_model.ROOT, 'describe', '--always', '--tags', '--dirty', '--exclude', 'ext/*'],
                               capture_output=True, text=True).stdout.strip()

    if not described:
        print('git describe 가 아무것도 내지 않았다: 더러움을 모르는 이미지는 이름 붙여 두지 않는다')
        return 1

    if not names_image(described, identity):
        print('git describe %s 와 이미지 git_identity %s 가 다르다: 이 이름으로 붙이지 않는다 (빌드 중에 HEAD 가 움직였을 수 있다)' % (described, identity))
        return 1

    stage = stage_dir(described)
    staged = os.path.join(stage, os.path.basename(PX4))
    os.makedirs(stage, exist_ok=True)
    shutil.copy2(PX4, staged)
    print('준비됨:', staged)
    # The checked copy, not make's upload target: that one rebuilds and may
    # repack the .px4 around an airframes.xml it does not regenerate
    print('올리기 (사용자): ! python3 %s %s' % (os.path.join(gz_model.ROOT, 'Tools', 'px4_uploader.py'), staged))
    return 0


if __name__ == '__main__':
    sys.exit(main())
