# upstream 준비 재검토 (2026-10-12 19:20~20:00, 교수 역할) — 도구의 집과 upstream 반영 전 코드베이스 전체

사용자 지시: "아예 별도 관리 레포를 만들어도 좋아. 현재 관례나 타당성 벗어나는 것은 내 linearust 계정에 레포를 파서 관리해도 좋아. 한 번 더 깊게 검토해.
그리고 기존 내가 작성해서 아직 upstream 반영 전인 모든 코드베이스에 대해서 다시 검토해봐."

검토 범위 (트리에서 잰 것): PX4 airship/dev 가 upstream main 보다 **165 커밋, 89 파일, +8,659/−169 줄** 앞섬 (배분기 3,261 + 라이브러리 508, 컨트롤러 2,545, posix 에어프레임
1,227, 문서 650, rate_control 179, 파라미터 173, 비행 에어프레임 70, 메시지 40). models airship/models 는 **34 커밋, 105 파일, +10,675 줄, 모델 디렉터리 52개**. 벤치 도구 디렉터리는
git 아님, 5.0 GB (산출물 포함), 파이썬 16k 줄 + 요 모델 20k 줄 + 문서 26 편.

## 1. 결론 — 집은 셋이 아니라 둘: upstream 에는 소스만, 나머지는 한 레포

| 어디 | 무엇 |
|---|---|
| **PX4-Autopilot (upstream)** | 펌웨어 (배분기·포드·요 배분·컨트롤러·setIntegral), 메시지 2, 파라미터 17→15, 비행 에어프레임 2500/2507/2520, docs/en 두 페이지, **gz 에어프레임은 완전 플랜트 셋** (2500 고정, 2510 Cloudship, 2520 벡터링), 착륙 감지 기본값, classic 2519 수정 |
| **PX4-gazebo-models (upstream)** | lta 월드 + **완전 모델 셋** (base/am/fin/drag 를 하나로 펼친 model.sdf, 머리에 출처 한 줄), 생성기는 넣지 않음 |
| **linearust/px4-airship-tools (새 공개 레포)** | 생성기·계산기·기체 JSON, SITL 게이트·전진 검사·요 모델, 물리 변형 가족 (ablation 멤버 전부) 과 문헌 가족 (실내·AURORA·JAXA·MPI) 의 모델·에어프레임 **오버레이**, 벤치 절차와 펌웨어 스테이징 스크립트, 설계 기록 문서 |

왜 C (모델 저장소 `tools/airship/`) 보다 이것인가:
- 관례: PX4 `Tools/` 에 기체 설계 생성기는 없고 ROMFS 에어프레임은 손으로 쓴 소스다. 모델 저장소에는 `tools/avl_automation` 선례가 있지만 그 저장소는 모델 30여 개의 작은 저장소라
  52개 모델 디렉터리와 2,000 줄짜리 생성기 + 시험을 한 번에 받을 가능성이 낮다. 생성기 없이 **펼친 모델 셋**은 리뷰어가 파일만 보고 판단할 수 있고, 생성기는 "원하면 뒤에 PR" 로 제안한다.
- 재현성: 모델과 에어프레임의 머리 주석이 `https://github.com/linearust/px4-airship-tools` 와 생성 명령·JSON 을 가리킨다. 외부 도구 출처는 upstream 모델에 흔한 형태다 (CAD·AVL).
- 한 코드베이스: 생성기와 그 시험, 게이트, 요 모델은 서로 import 한다 (gz_model/report/hull_basis/fin_basis 공용). 쪼개면 복제가 생긴다. 지금의 벤치 디렉터리가 이미 그 한 집이다.
- 사용자 지시와 일치: "관례나 타당성을 벗어나는 것은 linearust 레포". 물리 변형 가족 (8 멤버 × 6 가족) 은 **검증 장치**이지 upstream 제품이 아니다 — 이것이 이번 검토의 가장 큰 발견이다 (2절 A1).

### 새 레포 제안: `linearust/px4-airship-tools`

- 이름에 사업 맥락 없음. 라이선스 BSD-3 (PX4 와 같게). README 영어 (설계 기록 한국어 문서는 `docs/ko/` 에 그대로).
- 구조: `tools/` (airship_family_gen, airship_gains, generic_hull_design, apply_generic_hull, gz_model, hull_basis, fin_basis, report), `vehicles/` (JSON + measured 합침),
  `checks/` (variant_check, sitl_check, forward_yaw_check, added_mass_check, aero_moment_check, airship_fixed_flight_check, cloudship_*), `yaw_model/`, `bench/` (gz_run.sh, sitl_link, gz_link,
  bench_*.py, build_fmu.py, fingerprint, tilt_range_check, snapshot/apply_params), `overlay/` (아래), `docs/ko/`.
- **오버레이 (upstream 에 없는 가족을 로컬 PX4 에서 날리는 법):** gz 모델은 `GZ_SIM_RESOURCE_PATH` 에 `overlay/models` 를 더하면 된다 (서브모듈 수정 없음). posix 에어프레임은 빌드 산출
  디렉터리 `build/px4_sitl_default/etc/init.d-posix/airframes/` 에 복사하면 rcS 가 번호로 찾는다 (CMake 등록 불필요) — `overlay/install.sh <px4 tree>` 가 한다. 월드의 Buoyancy
  `<enable>` 항목은 월드 오버레이 (`overlay/worlds/lta_lab.sdf`) 로. 이렇게 하면 upstream 트리는 소스 셋만 가지고 벤치는 지금처럼 48 멤버를 다 날린다.
- 산출물은 git 에 넣지 않는다: `.gitignore` 에 `sitl_out/ fw_*/ head_*/ identity/ baseline/ *.ulg __pycache__/` (지금 5.0 GB 의 거의 전부). 기록으로 남길 요약 (summary.json, 판정 텍스트)
  만 `records/` 에.
- CI (GitHub Actions): pytest 로 생성기·계산기·variant 시험 (gz 불필요: 파일만 읽음), pinned PX4 커밋에 대해 `--check` 재현. SITL 게이트는 로컬 전용 (문서에 명시).
- 비용: 이틀. 첫날 레포 뼈대·이동·오버레이 스크립트·.gitignore·README, 둘째 날 CI 와 upstream 쪽 트림 (2절 A1~A3).

## 2. upstream 반영 전 코드베이스 재검토 — 발견 (심각도 순)

### A. 구조·범위 (PR 전에 반드시)

- **A1. 물리 변형 가족이 upstream 범위를 넘는다.** gz 모델 52개, posix 에어프레임 40개, 십 자리 표 7개, 문서 vehicles.md 371 줄, CMake 주석 14 줄, 월드의 enable 항목 가족당 30개.
  code-review 지침 "Necessity: challenge every addition" 에 바로 걸린다. 과학적 가치 (한 루프를 여러 플랜트에 돌려 모델의 몫과 루프의 몫을 가름) 는 우리 검증의 것이지 사용자의 것이
  아니다. **upstream 은 완전 플랜트 셋 (generic 고정 2500, Cloudship 2510, 벡터링 2520) 과 lta 월드만**; 문헌 가족·실내 가족·ablation 멤버는 새 레포의 오버레이. 실내 가족은
  하드웨어 검증 기체지만 하드웨어는 2520 프리셋으로 날므로 upstream 에 실내 gz 가족이 없어도 "검증된 것" 의 주장은 2520 + 벤치 기록으로 선다. 에어프레임 번호의 ones-digit 마스크
  규칙과 십 자리 표는 upstream 에서 사라진다 (레포 문서로).
- **A2. 생성물 표식.** 에어프레임 10개의 "--- yaw loop values from airship_gains.py (regenerate, do not edit) ---" 블록. upstream 에어프레임에 생성 표식은 없다. 값 위의 물리 주석 1~3 줄과
  출처 한 줄 ("values from px4-airship-tools airship_gains.py, vehicles/airship_generic.json") 로 바꾼다 (값 불변). 생성기는 그 출처 줄을 표식으로 써서 계속 다시 쓸 수 있다.
- **A3. 주석 밀도.** posix 에어프레임 (2510 은 96 줄 중 60 줄이 주석: uint16 MIN 처리, 적분기 수치 등), `ActuatorEffectivenessAirship.hpp` 의 30 줄 블록 (kShortfallDeadband,
  shortfall), gz 모델 머리 (airship_base 40+ 줄 설계 논증). upstream 밀도는 "무엇·왜 3 줄". 첫 커밋 (4943cb5720 "trim ... to upstream density") 이 한 번 줄였는데 다시 자랐다. 긴 논증은
  PR 본문과 레포 문서로, 코드에는 결론만. 이것은 가장 손이 많이 가는 항목이다 (하루).
- **A4. 커밋 이력.** 165 커밋 중 fix(control_allocator) 22, refactor 28, test 17 — 같은 파일을 수십 번 고친 흔적이다. upstream 은 PR 단위로 squash 하므로 이력 자체는 문제가 아니지만,
  PR 브랜치는 upstream main 에서 새로 떠서 **논리 단위 커밋 2~6개** 로 다시 쌓는다 (cherry-pick + squash). 165 커밋 브랜치를 그대로 PR 하지 않는다. P1~P5 분할은 upstream_pr_plan 대로.
- **A5. DCO.** 모든 커밋 `Signed-off-by` (사용자) — 검토 뒤 `git rebase --exec`; AI 는 `Assisted-by:` 로만 ✓.

### B. 리뷰어가 물을 것 (답을 준비하거나 고친다)

- **B1. 파라미터 17개.** CA_AIRSHIP_* 6 + AS_* 11. messages 지침 "every new param is a configuration burden". 줄일 수 있는 둘:
  - `AS_SPD_LO`: 계산기가 항상 HI/2 로 쓴다 (stall_ramp). 파라미터 하나로 (`AS_SPD_HI` 만 두고 LO = HI/2 를 문서에). −1.
  - `AS_YR_RESTORE`: D1 결정 (10-07) 은 "복원 규칙 항상" 인데 파라미터 기본값 0 이고 gz 멤버 11개만 1 로 켠다; 비행 에어프레임 2520 은 꺼져 있다 (실내 params 파일만 1). 결정대로라면
    파라미터를 없애고 항상 켠다 (데드존 가장자리 규칙 포함). −1. 남기려면 기본값 1 과 2520 의 명시가 필요하다 — 지금은 결정과 코드가 어긋나 있다.
  - `AS_SPD_DP_TOL` 에 unit 이 없다 (Pa 는 yaml 단위 목록에 없음) — short 에 [Pa] 로 둔 지금 그대로 가고 PR 에서 말한다.
- **B2. 메시지 2개.** `airship_allocator_status` 는 parameters 지침 (다른 모듈의 파라미터를 읽지 말고 uORB 로) 의 직접 결과라 근거가 선다. `airship_speed` 는 로깅·텔레메트리용 내부
  구간 — "rate_ctrl_status 나 기존 토픽에 못 넣나" 를 받을 것이다. 답: 구간·원천·상한은 다른 토픽에 자리가 없고 로그 분석 (전진 검사) 이 그것으로 판정한다; 선택 토픽 (add_optional_topic)
  이라 비행선 아닌 기체의 로그에는 없다.
- **B3. 착륙 감지 우회.** `rc.airship_defaults` 의 `COM_DISARM_LAND -1` 은 `AirshipLandDetector` 가 Land 모드를 고르면 곧 "착륙" 이라 하는 것 (AirshipLandDetector.cpp:54) 의 결과를
  막는 기본값이다. 리뷰어는 감지기를 고치라 할 수 있다. 답: 비행선에는 착륙을 가를 센서가 없고 감지기의 그 줄은 upstream 의 의도적 동작이므로, 공중 무장 해제를 막는 기본값이 옳다;
  재현 로그 (airship-midair-disarm-bug) 를 PR 에.
- **B4. module.yaml 의 틸트 서보 표시.** "Pod Tilt Servo count 1 + Port Pod Tilt Servo count CA_AIRSHIP_GRP" — 범위가 0 이면 QGC 가 안 쓰는 서보 하나를 보여 준다. 메타데이터 표현력의
  한계이고 주석에 있다; QGC 쪽 리뷰어가 볼 수 있으니 PR 본문에 적는다.
- **B5. Cloudship 2507 의 배분기 이전 (CA_AIRFRAME 9 → 16) 과 `@maintainer` 추가.** 하드웨어 검증 없음 (gz 쌍둥이 9/9 뿐). upstream_pr_plan 3 번 답 그대로; 거부되면 P5 로.
  maintainer 줄은 upstream 2507 에 없던 것 — 사용자가 Cloudship 도 맡는다는 뜻이면 그대로, 아니면 뺀다.
- **B6. 모드 범위.** AS_YAWRATE_MAX 설명이 "Acro, Stabilized, Altitude, Position" 에서 루프가 닫힌다고 하나 비행선에는 자세·위치 컨트롤러가 없다. 지금 코드는 그 모드에서도 스틱 통과 +
  요 루프라 틀린 말은 아니지만, "Altitude/Position 은 수동 통과와 같다" 를 문서에 한 줄 적는 편이 안전하다.
- **B7. 보드 발자국.** 모듈이 10개 보드 설정 (micoair ×4, zenfc, sitl ×2, fmu-v6x airship) 에 들어간다. 속도 선택기·스케줄로 커진 플래시·RAM 을 PR 에 적는다 (bloaty 비교, micoair_h743-aio
  기준).

### C. 잘 된 것 (바꾸지 않는다)

- 라이브러리 경계: `ControlSurfaceType.hpp` 는 복제가 아니라 모듈의 enum 을 lib 로 옮긴 것 (ActuatorEffectivenessControlSurfaces.hpp 가 include); `kCaAirframeAirship` 은
  ControlAllocator.hpp 의 enum 이 쓰는 단일 출처. 포드·배분기는 uORB 없이 시험 가능.
- 시험: gtest 64 (배분기) + 25 (포드) + 8 (요 배분) + 7 (rate_control) + 61 (컨트롤러 넷 + 기능 시험) — upstream 기준으로 두터운 편. 기능 시험은 ManualControlTest 선례.
- 문서: docs/en 두 페이지가 지침 (의미 단위 줄바꿈, UK 철자 normalised, 상대 링크) 을 따른다. 단 vehicles.md 의 비행선 절은 A1 과 함께 1/10 로 준다.
- 스타일: check_format 통과, 라이선스 머리, PX4_ISFINITE 가드, 매직 넘버는 명명 상수.

## 3. 실행 순서 (제안)

1. 새 레포 (이틀): 뼈대·이동·오버레이·CI. 벤치 디렉터리는 그 레포의 작업 사본이 된다.
2. upstream 트림 브랜치 (이틀): A1 (가족 셋만, 펼친 모델 셋, 월드 항목 셋), A2 (표식 → 출처 줄), A3 (주석 밀도), B1 (파라미터 둘), 문서 축소. 모든 게이트는 오버레이로 그대로 돈다.
3. PR 브랜치 재구성 (하루): upstream main 에서 P1~P5 를 논리 커밋으로; 사용자 검토·서명.
4. 그 사이 하드웨어: fw 플래시·지문, 기체 제작 뒤 H1/H2.

이 순서는 10월 비행 검증과 독립이다 (펌웨어 동작은 바뀌지 않는다). 2절 B1 의 파라미터 둘은 동작이 바뀌므로 (복원 항상 켬) 비행 전에 정하는 편이 좋다.
