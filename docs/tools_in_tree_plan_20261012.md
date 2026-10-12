# 도구를 트리에 넣는 계획 (2026-10-12, 교수 초안; 사용자 확인 뒤 실행)

근거: upstream_pr_plan_20261007.md "PX4 관례" 1 — 리뷰어가 다시 만들 수 없는 생성물 (모델 SDF, 에어프레임, 문서 표, 이득 블록) 은 받지 않는다. 지금 생성기·계산기는
`~/dev/px4-airship-bench-tools` (git 아님) 에 있고 에어프레임의 표식 주석이 `airship_gains.py` 를 이름으로 가리킨다.

## 의존 그래프 (2026-10-12, import 기준)

- 공용: `gz_model.py` (SDF·에어프레임·CMake 읽기, ROOT/GZ_MODELS/AIRFRAMES 경로; 모델과 PX4 양쪽을 읽음), `report.py` (verdict/note/summary),
  `hull_basis.py` (Lamb 부가질량·Munk), `fin_basis.py` (핀 양력·실속·유도항력), `sitl_link.py`/`gz_link.py` (SITL 전용).
- 생성기: `airship_family_gen.py` → `apply_generic_hull.py` → `generic_hull_design.py` → hull_basis/fin_basis; 모두 gz_model.
  출력: models 저장소 (model.sdf ×8, 월드 부력 항목), PX4 (posix 에어프레임, CMake 항목, docs/en 두 문서).
- 계산기: `airship_gains.py` → gz_model. 입력 `vehicles/<v>.json` + `vehicles/<v>.measured.json`; 출력 에어프레임 표식 블록, `<v>.params`.
- 검사: `variant_check.py` (gz_model, hull_basis, fin_basis, report; 트리 정적 검사), `airship_family_gen_test.py`, `forward_yaw_check_test.py`.
- SITL 게이트 (트리 밖에 남긴다, upstream_pr_plan 5): `sitl_check.py`, `forward_yaw_check.py`, `added_mass_check.py`, `aero_moment_check.py`, `gz_run.sh`, 요 모델.

## 선택지

**A. 한 집 (권고): PX4 `Tools/airship/`** 에 생성기·계산기·공용 모듈·`vehicles/` 를 모두 둔다. PX4 트리는 모델을 서브모듈 (`Tools/simulation/gz/models`) 로 품으므로
한 도구가 SDF 와 에어프레임을 함께 쓸 수 있고 gz_model/hull_basis/fin_basis 가 한 번만 있다. models 저장소의 생성 SDF 는 머리 주석에 "PX4-Autopilot
Tools/airship/airship_family_gen.py 가 vehicles/<v>.json 에서 생성" 과 생성한 PX4 커밋을 적는다. 약점: models 저장소 리뷰어는 생성기를 자기 저장소에서 못 본다
(선례 `tools/avl_automation` 은 자기 저장소에 있음).

**B. 두 집 (10-07 결정 그대로)**: 선체 설계·SDF 작성기 (generic_hull_design, apply_generic_hull, hull_basis, fin_basis, gz_model 의 SDF 읽기) 는 models `tools/airship/`,
에어프레임·CMake·문서 작성기와 이득 계산기는 PX4 `Tools/airship/`, 기체 JSON 은 PX4 에 두고 models 도구가 경로로 받는다. 약점: gz_model 을 둘로 쪼개거나 (SDF 읽기 vs
에어프레임 읽기) 복제해야 하고, PX4 쪽 계산기도 SDF 를 읽으므로 (nRabsR, 관성, 핀) 결국 양쪽에 SDF 리더가 생긴다. 두 저장소 시험 두 벌.

교수 권고: **A**. 두 저장소 사이의 유일한 결합은 "서브모듈 포인터" 이고 생성물의 재현은 PX4 트리에서 한 명령으로 된다. 사용자가 models 저장소 쪽 관례를 더 중시하면 B.

## A 의 실행 순서 (하루, 서브에이전트 없음; SITL 체인이 돌지 않을 때만 — 체인이 같은 디렉터리의 도구를 import 한다)

1. `Tools/airship/` 만들고 `gz_model.py report.py hull_basis.py fin_basis.py generic_hull_design.py apply_generic_hull.py airship_family_gen.py airship_gains.py variant_check.py`
   와 `vehicles/` 를 `git mv` 가 아닌 복사 (벤치 도구 디렉터리는 git 이 아님) 로 넣고, 벤치 디렉터리의 원본은 트리 쪽을 import 하는 얇은 껍데기 (sys.path 에 Tools/airship 추가) 로 바꾼다
   — sitl_check/forward_yaw_check/added_mass_check 가 gz_model/report 를 계속 쓰므로.
2. 경로 상수: gz_model.ROOT 를 `__file__` 기준 (Tools/airship → 두 단계 위) 으로; `vehicles/` 경로를 Tools/airship/vehicles 로; airship_gains 의 측정 보관도 같은 곳.
3. 에어프레임 표식 주석을 `Tools/airship/airship_gains.py` 로 (생성기 재실행이 바꿈), 모델 SDF 머리 주석에 생성기 경로 (models 커밋 하나).
4. 시험을 트리 관례로: `Tools/airship/test/` 에 pytest 꼴로 (`airship_family_gen_test.py`, `forward_yaw_check_test.py` 는 SITL 산출물을 읽으므로 트리 밖 유지), CI 에 넣을지는 P4 때.
5. `make check_format` 은 Python 을 보지 않는다; 저장소의 Python 관례 (`.github/instructions/*.instructions.md` 의 Python 항목, flake8 설정) 를 읽고 맞춘다 (80/120 열, 따옴표).
6. README·upstream_pr_plan 갱신, 커밋 PX4 `build(tools): add the airship family generator and gain calculator` (+ models 머리 주석 커밋), 벤치 도구 껍데기 확인 (variant 16/16, gen 17/17, forward 219/219).

## 하지 않는 것

- 요 모델 (`yaw_model/`) 과 SITL 게이트는 트리 밖 (upstream 의 gz CI 는 lta 월드를 돌리지 않는다; P3 때 재검토).
- 문서 (한국어 계획·기록) 는 벤치 디렉터리에 남는다.

## 재검토 (18:30, 사용자: "결국 upstream 에 올릴 건데 PX4 관례에 타당한 방향으로") — 선택지 C 를 권고로 바꾼다

트리에서 확인한 관례:
- PX4 `Tools/` 는 빌드·CI·메타데이터·업로더·로그 분석 도구뿐이다 (px4airframes, px_process_*, px4_uploader, ecl_ekf, msg). 기체 설계 생성기는 하나도 없고, ROMFS 에어프레임
  가운데 "generated / do not edit" 표식을 가진 파일은 우리 것뿐이다. upstream 에서 에어프레임과 docs/en 은 손으로 쓰는 소스다.
- PX4-gazebo-models (서브모듈 루트) 에는 `tools/avl_automation` 이 있다: 기하 입력 (yml) → AVL → Advanced Lift Drag 플러그인 블록을 생성해 model.sdf 에 붙여 넣는 생성기, README·
  templates 포함. gazebo-classic 의 sitl_gazebo 도 jinja 로 모델을 생성했다. 즉 **모델 생성기는 모델 저장소에** 두는 것이 선례다. 그 저장소의 CI 는 `tests/test_model.py` (pytest) 와
  `sdf_parsing` (SDF 검증) 을 돌린다 — 우리 "다시 만들면 같다" 검사가 들어갈 자리가 있다.

**C. 모델 저장소 `tools/airship/` 한 집 + PX4 는 소스만.**
- models 저장소: 선체·핀 설계 (`generic_hull_design`, `hull_basis`, `fin_basis`), SDF 작성기 (`apply_generic_hull`, gz_model 의 SDF 읽기), **기체 JSON** (측정 기울기는 JSON 의
  `measured` 필드로 합침), 요 이득 계산기 (`airship_gains`: SDF 와 공식만 읽으므로 PX4 의존 없음; 파라미터 블록을 **인쇄**하고 `--params` 파일을 쓴다), README (avl_automation 꼴),
  `tests/` 에 "JSON 에서 다시 만들면 커밋된 model.sdf 와 같다" pytest. 생성된 model.sdf 머리 주석 한 줄: "tools/airship 이 vehicles/<v>.json 에서 생성; JSON 을 고치고 다시 만든다".
- PX4: 에어프레임·CMake 항목·docs/en 은 보통의 소스로 커밋한다. 표식 블록 "(regenerate, do not edit)" 은 지우고, 값 위의 물리 주석과 출처 한 줄 ("values from the models
  repo's tools/airship gains calculator, vehicles/airship_indoor.json") 만 둔다. PX4 에는 생성기가 들어가지 않는다 (`Tools/airship/` 없음).
- 벤치 (트리 밖, 지금처럼): PX4 파일을 쓰는 작성기 (`airship_family_gen` 의 PX4 쪽), variant_check, SITL 게이트, 요 모델. 이들은 서브모듈 경로의 트리 도구를 import 해 쓴다 —
  저자의 작업 흐름이지 upstream 산출물이 아니다 (upstream 의 gz CI 는 lta 월드를 돌리지 않는다).

A 보다 나은 이유: PX4 `Tools/` 에 ROMFS 에어프레임을 다시 쓰는 생성기를 넣는 것은 선례가 없고 리뷰에서 "왜 생성기가 init.d 를 쓰나" 를 부른다; 모델 저장소에는 정확한 선례가 있다.
B 와 다른 점: PX4 쪽 작성기를 upstream 에 내지 않으므로 공용 모듈을 쪼갤 일이 없다 (SDF 읽기는 모델 저장소에, 에어프레임 읽기는 벤치에 자연히 나뉜다).

비용: 하루. 순서: (1) 서브모듈에 `tools/airship/` (설계·SDF·계산기·JSON·README·pytest), (2) 커밋된 model.sdf 머리 주석, (3) PX4 에어프레임의 표식 블록을 출처 한 줄로 (값 불변;
variant 16/16·--check IDENTICAL 로 확인), (4) 벤치 도구를 트리 쪽 import 로, (5) upstream_pr_plan 의 M1/M2 에 tools 와 pytest 를 넣는다.
