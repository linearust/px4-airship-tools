# upstream 분리 설계 (2026-10-12 22:20, 교수 역할) — 트리에는 완전 플랜트 셋, 레포에는 가족 전부

사용자 지시: "PX4 upstream 에 올리는 작업이나, 내가 직접 검토하기 전에, 그리고 실내 비행선 테스트 기체를 만들기 전에 할만한 작업을 미리 진행해. 깊게 검토해서 진행해."
앞선 검토 (`upstream_readiness_review_20261012.md` A1) 의 결론을 실행 가능한 설계로 적는다. 결정은 번호로, 근거는 그 아래.

## 0. 먼저 하는 것: 파라미터 둘 (증분 A, 하드웨어 비행 전)

**A1. `AS_YR_RESTORE` 를 없애고 복원 규칙을 항상 켠다.** D1 결정 (2026-10-07 "복원 항상") 이 이미 섰는데 yaml 기본값은 0 이고, 비행 에어프레임 2520 (하드웨어)
에는 줄이 없어 `vehicles/airship_indoor.params` 가 1 로 덮어쓰지 않으면 꺼진 채 난다. 끄는 경우가 없다: 핀 없는 플랜트에서는 놓을 때 적분기가 0 근처라
규칙이 아무것도 바꾸지 않고, 핀 있는 플랜트에서는 규칙 없이 놓은 뒤 회전이 37~145° (step0) 다. 코드: `setRestore(enabled, on_reversal)` → `setRestoreOnReversal(bool)`
(MAN_DEADZONE > 0 일 때 부호 반전도 놓음으로 본다), yaml 항목 삭제, 문서 Forward Flight 절의 불릿은 고정 동작 설명으로, posix 에어프레임 10개의
`param set-default AS_YR_RESTORE 1` 줄 삭제 (생성기가 쓰는 블록도), `.params` 에서 삭제, 도구 (`hover_ff_family`, `forward_yaw_check` SCHEDULE_PARAMS, 시험 픽스처).
**A2. `AS_SPD_LO` 를 없애고 램프 시작을 `AS_SPD_HI / 2` 로 고정한다.** 계산기는 항상 LO = HI/2 로 쓴다 (`stall_ramp`). 10-07 에 "핀 실속과 피토 하한이 다른 속도"
라 둘을 뒀는데, 피토의 하한은 `AS_SPD_DP_TOL` 이 구간 [u_lo, u_hi] 를 넓혀 처리하지 램프가 처리하지 않는다. 램프는 핀이 선회를 떠받기 시작하는 속도 하나와
그 폭인데, 폭을 바꾸는 사람이 없으면 파라미터는 부담이다 (messages 지침). 이름은 동결이므로 `AS_SPD_HI` 를 두고 설명을 "이 속도의 절반부터 몫이 들어온다" 로.
코드: `schedule(..., ramp_lo, ramp_hi, ...)` → `ramp_hi` 하나 (`ramp(u, 0.5f * hi, hi)`), yaml 삭제, 문서, posix 에어프레임 10개의 LO 줄 삭제, `.params`, 계산기
(`REF_RAMP`, 블록), `forward_yaw_check` (다리 배치가 LO 를 읽음 → HI/2), 시험 픽스처. 요 모델의 `policies.AS_SPD_LO` 는 그 기록의 상수라 둔다.
검증: gtest, check_format, SITL 게이트 + 전진 검사 한 플랜트 (실내 완전: 램프 0.4/0.8 그대로), fmu 빌드 (Manual 지문은 이 파라미터와 무관).

## 1. 범위 결정

| 집 | 가진다 |
|---|---|
| PX4-Autopilot | 가족마다 **완전 플랜트 하나**: `2500_gz_airship_fixed` (= 오늘의 2505), `2510_gz_cloudship` (= 2517), `2520_gz_airship_vectored_independent` (= 2527), 그리고 `2519_gazebo-classic_cloudship`. 그 셋의 요 루프 값. CMake 목록·십 자리 표 없음, `vehicles.md` 항목 3개, 월드 `<enable>` 15줄 (모델 3 × 5 인스턴스). |
| PX4-gazebo-models | `airship_fixed`, `cloudship`, `airship_vectored_independent` — 체인을 **펼친** model.sdf 하나씩 (merge include 없음, `_base` 없음), 머리에 출처 한 줄. lta 월드. |
| px4-airship-tools `overlay/` | 가족 7 (generic fixed/vectored, Cloudship, indoor, AURORA, JAXA, MPI) 의 **모든 멤버** (bare→rigid, drag, fin, am, am_fin, am_fin_drag, _base, _base_am), 에어프레임, 월드. |

근거는 A1 그대로: 변형 멤버는 모델의 몫과 루프의 몫을 가르는 검증 장치다. 실내 가족은 하드웨어 기체지만 하드웨어는 2520 프리셋으로 날므로, 검증 주장은 2520 + 벤치 기록으로 선다.

## 2. 이름과 번호: 충돌을 어떻게 피하나

upstream 의 완전 플랜트가 **평범한 이름** (`airship_vectored_independent`) 과 **기본 번호** (2520) 를 가져간다. 하드웨어 에어프레임 2520 의 이름이 그것이고,
upstream 사용자가 날리는 것은 완전 플랜트여야 한다. 그러면 오버레이의 bare 멤버가 지금 쓰는 같은 이름·번호와 충돌한다 (gz 는 `GZ_SIM_RESOURCE_PATH` 에서
이름으로 찾으므로 오버레이가 upstream 모델을 **조용히 가린다** — 오버레이 설치 여부에 따라 `gz_airship_vectored_independent` 의 뜻이 바뀌는 함정; rcS 는 번호로
찾으므로 2520 파일 둘은 충돌).

**D1. bare 멤버는 `_rigid` 토큰을 얻는다.** 문법 `<body>[_<propulsion>][_am][_fin][_drag]` 에서 토큰 없는 뿌리만 `<...>_rigid` 로: `airship_vectored_independent_rigid`,
`cloudship_rigid`, `airship_fixed_rigid`, `airship_indoor_rigid`, .... `_fin`·`_drag` 는 `_rigid` 를 merge include 하고, `_am` 은 지금처럼 `_base_am` 규칙의 전체 복사.
토큰 있는 멤버의 이름은 그대로 (`_am`, `_fin`, `_am_fin_drag` 는 upstream 에 없으니 충돌 없음). 버린 대안: 오버레이에 다른 몸체 이름 (`airship_lab`) — 문법 "몸체 = 기체" 를
깬다; upstream 에 다른 이름 — 하드웨어 2520 의 이름을 바꿔야 한다.
**D2. 오버레이 번호는 26x0 + 마스크.** 2600 fixed, 2610 Cloudship, 2620 vectored, 2630 indoor, 2640 AURORA, 2650 JAXA, 2660 MPI; 일의 자리는 지금 규칙 그대로
(rigid 0, drag 1, fin 2, am 4). 2600~2999 는 upstream main 에도 우리 브랜치에도 비어 있다 (확인). 오버레이의 완전 멤버 (예 2627) 는 upstream 2520 과 **물리적으로
같은 플랜트**이고 둘 다 둔다: 체인의 끝이자 쌍둥이 검사의 기준 (D4).
**D3. 기록의 별칭.** 분리 전 기록·레지스트리에서 평범한 이름 (`gz_airship_vectored_independent`, `gz_cloudship`, `gz_airship_fixed`) 은 **bare** 플랜트였다. 분리 뒤 같은
이름은 완전 플랜트다. `record_gains.RUNS` 와 `gate` 의 레지스트리에 `plant` 를 명시하고, `airship_family_naming.md` 의 별칭 표에 날짜와 함께 적는다 (표는 이미 있다).
기록 파일 이름은 바꾸지 않는다.

## 3. 생성기와 도구

**D4. 생성기는 두 출력 모드.** `--overlay <dir>` 는 가족 전체를 `_rigid` 뿌리와 26xx 번호로 오버레이 세 디렉터리에 쓴다 (월드는 upstream 월드 + 오버레이 entry 의
사본). `--upstream` (기본, PX4_ROOT) 는 완전 멤버만: 체인을 펼친 model.sdf (include 를 따라가며 합친 것을 쓴다; gz_model 이 이미 SDF 를 읽는다), 기본 번호의
에어프레임 (완전 멤버의 요 값, 표식 대신 출처 한 줄), 월드 entry 5줄, vehicles.md 항목 하나. `variant_check` 에 쌍둥이 검사 추가: 펼친 upstream 모델 == 오버레이
완전 멤버 (링크·관성·플러그인 원소 단위), upstream 에어프레임 == 오버레이 완전 에어프레임 (번호·이름 줄 빼고).
**D5. 모델·에어프레임 찾기는 한 곳에서.** `gz_model.GZ_MODELS/AIRFRAMES` 를 경로 문자열로 쓰는 79곳 (파일 20) 을 `gz_model.model_dir(name)`, `airframe_path(number|name)`
로 모으고, 탐색 순서는 [오버레이, 트리]. 이름이 다르므로 (D1) 가리기는 없다. `gz_run.sh`: 대상이 오버레이 멤버면 오버레이 월드로 gz 를 먼저 띄우고 px4 를 붙인다
(rc.gzsim 의 "이미 떠 있는 서버" 경로); 에어프레임은 `overlay/install.sh` 가 빌드 etc 에 복사.
**D6. 트림 커밋 (PX4·모델).** 모델 서브모듈: 47 디렉터리 삭제, 펼친 3 추가 (airship/models 브랜치, 먼저 푸시). PX4: posix 에어프레임 37 삭제 + 3 을 기본 번호로 (내용은
완전 멤버의 것, 표식 제거 (A2), 주석 밀도 (A3)), CMake 목록·표, 월드 15줄, vehicles.md, frames_airship 문서의 가족 설명 (마스크 규칙 삭제, 레포 링크 한 줄).
요 모델 `controller.airframe_for_model` 등은 D5 의 찾기를 쓴다.

## 4. 순서와 검증

1. 증분 A (오늘). 검증은 0절.
2. 레포 쪽 (트리 불변): D4 `--overlay` + D5 찾기 + D1/D2/D3 → `variant_check` 두 집 + 쌍둥이, 생성기 왕복 (두 모드), `run_tests.sh`. 오버레이로 SITL: 실내 완전·JAXA 완전·
   2527 상당 게이트 10/10, 전진 42/42 (오늘 기준선과 같아야 한다 — 플랜트 파일은 이름만 다르다).
3. 트림 (D6). 검증: upstream 2520 게이트 10/10 (= 오늘 2527), 2510 9/9, 2500 비행 검사, `make px4_sitl_default` 와 fmu-v6x 빌드, 지문 0.000 µs (에어프레임 메타데이터가
   바뀌므로 `build_fmu.py --check`).
4. P1~P5 PR 브랜치 재구성 (upstream main 에서 논리 커밋). 사용자 검토·서명.

비용: 2 는 하루, 3 은 하루 + gz 시간, 4 는 반나절. 증분 A 는 두세 시간.

## 5. 열어 두는 것

- 요 모델 재앵커링 ("기록이 제 플랜트를 지닌다"): D3 의 `plant` 필드가 그 첫걸음이지만, 플랜트 수치의 스냅샷은 다음 일.
- `vehicles.md` 의 upstream 항목에 "변형 가족은 px4-airship-tools 에" 한 줄을 둘지는 PR 때 리뷰어 반응을 본다.

## 6. 반박 검토 뒤 수정 (22:37, 워크플로 에이전트 5개, 837k 토큰, 발견 45: 반박 26 · 불확실 7 · 유지 12)

증분 A 는 결론이 선다. 근거와 검증을 고친다:
- **A1 근거 수정.** "핀 없는 플랜트에서 규칙이 아무것도 바꾸지 않는다" 는 틀렸다: 하드웨어 프리셋 (FF 0, MAN_DEADZONE 0.1 이라 반전 에지도 켜짐) 에서 규칙은 Stabilized/Acro 의
  모든 놓음을 적분기가 모은 만큼의 토크 단차 (최대 AS_YR_INT_LIM) 와 포드 스윙으로 끝낸다 — 느린 풀림 대신. 받아들이는 근거는 step0 표 (규칙 없이 37~145°, 있으면 ≤ 3°)
  와 10-07 의 결정이다. 문서의 고정 동작 문장에 "선회 중 바뀐 트림은 묵은 값으로 돌아오고, 그 오차는 (트림 변화)/AS_YAWRATE_I rad 로 풀린다" 를 적는다.
  하드웨어: 벤치 절차에 **Stabilized 풀스틱 놓음 검사** (핀에서 토크 단차와 포드 스윙이 CA_AIRSHIP_TLT_R 안) 를 더하고, 첫 부팅의 "unrecognised parameter" 경고와
  `.params`·벤치 스크립트에서 두 이름 제거를 적는다. Manual 지문은 이 변화를 보지 못한다.
- **A2 보충.** 써 있는 LO 는 반올림 때문에 HI/2 와 다르다 (2522/2527 1.1 → 1.05, 2542/2547 1.7 → 1.75), Cloudship 2512 는 기본값 1.5 → 1.25 로 움직인다. 문서 170~171 행의
  수치 (1.5°/s at 1.1 → 1.6°/s at 1.05). 전진 기준선 42/42 는 A 뒤 2527 과 2512 에서 다시 잰 것이 불변량이다.
- **A 검증.** 호버 게이트를 2517, 2520 (rigid), 2521 (drag 만: 하드웨어 프리셋의 가장 가까운 쌍둥이 — FF 0, 복원 켬, 선체 감쇠) 에서 돌려 A 뒤 기준선으로 기록한다.

분리 설계는 네 군데가 바뀐다:
- **S1 (D6 대체) upstream 모델은 merge include 모양을 지킨다.** upstream 이 이미 쓰는 모양 (x500_depth 등) 이므로 펼치지 않는다: `airship_base` (= 오늘의 `airship_base_am`),
  `airship_fixed` 와 `airship_vectored_independent` 는 그것을 merge 하고 제 블록 (추진·핀·Hydrodynamics) 을 더한다, `cloudship` 은 자체 완결 (= cloudship_am + 핀·항력 블록).
  디렉터리 넷, 펼치기 작성기 없음. 쌍둥이 검사 = 오버레이 완전 멤버의 merged 모델과 upstream 모델을 원소 단위 비교 (텍스트 아님), 에어프레임은 resolved 기본값 비교.
  오버레이의 rigid 몸체는 `airship_base_rigid`, `_am` 멤버는 upstream `airship_base` 를 merge 한다.
- **S2 (D1 보강) `_rigid` 는 문법 토큰이다, 이름 바꾸기가 아니다.** gz_model.SCHEME_TOKENS 와 MASK 에 'rigid' (마스크 0, 다른 셋과 배타) 를 더하고, `vehicle()`/`family_tokens()` 가
  벗기며, `gz_model.bare(vehicle)` → `<vehicle>_rigid` 를 variant_check 사실 1/2/4/14/15/16, posix_airframes(), sitl_check.VEHICLE, fin_damping, policies, coupling,
  cloudship_land_disarm (십 자리 25 하드코딩), apply_generic_hull.Family 가 쓴다. 뜻은 "마스크 0: _am/_fin/_drag 없음" 이지 "유체 없음" 이 아니다 (fixed 의 rigid 는 플랩 핀을 가진다).
  표시 이름: upstream 평범 멤버는 비행 @name 그대로 ("(Complete)" 없음), 오버레이 rigid 는 "<flight name> (Rigid Hull)". D1 의 기각 사유는 "upstream 에는 물리 접미사가 없다
  (스폰 가능한 모델은 평범한 이름 하나)" 로 바꾼다. 가리기 사유는 "rcS 는 번호 glob 의 마지막 파일을 조용히 고른다 (rcS:217)" 로.
- **S3 (D3 대체) 평범한 이름은 도구의 플랜트 키가 아니다.** 날짜에 따라 뜻이 바뀌는 키를 두지 않는다: yaw_model 의 **플랜트 레지스트리** 하나 (키는 오버레이 이름 `_rigid`/`_fin`/
  `_am_fin_drag`; 항목은 만든 디렉터리, 그날의 SYS_AUTOSTART, plants.EXPECTED 의 스냅샷 수치). RUNS 는 그 키를 참조하고, gains() 는 로그의 SYS_AUTOSTART 를 레지스트리의
  그때 번호와 비교한다 (살아 있는 트리가 아니라). closed_loop.TARGETS/PLANT_AIRFRAME/sitl_check.VEHICLES 는 레지스트리에서 이름을 받는다. 번호 별칭 표 (2521~2527, 2511~2517 →
  2621~2627, 2611~2617; gz 2520/2510/2500 = 옛 2527/2517/2505) 를 더한다. 이것이 5절의 재앵커링이며 **분리와 같은 증분**에서 한다 (차단 발견).
- **S4 (D5 대체) 오버레이 실행 경로.** (1) 트리의 25xx 멤버가 남아 있으면 rcS 의 이름 탐색이 번호 둘을 내고 트리 것이 실린다 — 오버레이 검증은 **트림된 트리에서만**
  (2·3 단계를 한 증분으로: 로컬 브랜치 `airship/trim` 에서). (2) `make px4_sitl gz_<x>` 대상은 오버레이 멤버에 없다: gz_run.sh 가 `make px4_sitl` (빌드만) → `overlay/install.sh`
  (ROMFS 재생성이 복사본을 지우므로 매번 빌드 뒤) → 빌드의 `gz_env.sh` 를 source 하고 오버레이 모델 경로를 앞에 둔 채 `gz sim -r -s overlay/worlds/lta_lab.sdf` 를 직접 띄움
  (플러그인 로드 실패 줄이 없는지 확인) → `gz service /world/lta_lab/create` 로 오버레이 모델 파일을 직접 스폰 (rc.gzsim 은 PX4_GZ_MODELS 경로에서만 찾는다) → px4 바이너리를
  `PX4_SIM_MODEL=gz_<x> PX4_GZ_STANDALONE=1 PX4_GZ_MODEL_NAME=<x>_0 PX4_GZ_WORLD=lta_lab` 로 직접 실행 (rc.gzsim 의 문서화된 standalone + 기존 모델 붙기 경로). 월드 이름은
  `lta_lab` 로 구별하고 exit-10 규칙은 "이 스크립트가 띄운 월드가 아니면" 으로 좁힌다. 순서가 무관해지므로 GZ_SIM_RESOURCE_PATH 의 "앞선 것이 이긴다" 문장은 지운다.
- **S5 (D4·D2 보강) 생성기.** `--overlay` 는 오버레이에 커밋된 generic `_rigid` 가족을 템플릿으로 (copy_family 는 src == dst 면 no-op), 십 자리는 JSON decade + 100,
  write_cmake/write_docs 는 오버레이 파일 (`overlay/airframes/decades` 표, overlay README 표) 로, 월드는 트리 lta.sdf 사본 위에. `--upstream` 은 완전 멤버만: 펼치기 대신 S1 의
  모양 (base + vehicle merge), 십 자리 줄 없음. 왕복 검사 둘: 오버레이 = 오버레이 디렉터리 사본에서의 멱등성, upstream = 쌍둥이 검사 (원소 단위) 를 부른다.
  variant_check: decade 표의 원천을 오버레이 (`vehicles/*.json` + cloudship/fixed 두 줄) 와 트리 (세 줄) 로 나누고 정규식을 2[56]\d\d 로, 사실 7/11 의 오버레이 판본,
  forward_yaw_check 의 ALLOWED 는 토큰이 아니라 merged 모델의 lift-drag 플러그인 유무로.
- **S6 (D6 보강) 트림 목록 추가.** 비행 2520 의 32~38 행 (변형 언급), AirshipYawScheduleTest.cpp:39 주석, 2500/2510 CMake 상호 참조 주석, worlds.md:50~53, classic Cloudship 은
  **2507 로 둔다** (2519 개명은 기존 사용자의 파라미터 초기화를 부르므로 upstream PR 에서 뺀다; 별칭 표에서 2519 삭제). 트림 게이트:
  `grep -rnE "\b25(0[1-9]|1[1-8]|2[1-9]|[3-6][0-9])\b|_am_fin|_am_drag" src ROMFS docs/en` 이 비어야 한다. vehicles.md 항목은 upstream 모양 (≤ 3 문장, make 명령, 스크린샷).
- **비용 재추정.** 2·3 단계 (한 증분): 2~3 일 + gz 시간, 순서는 (1) gz_model 두 집 + rigid 토큰 → (2) variant_check 두 집 (뒤의 모든 것의 게이트) → (3) 생성기 오버레이 모드
  + 시험 → (4) 플랜트 레지스트리 (S3) → (5) gz_run.sh 실행 경로 (S4) → (6) 트림 커밋 → (7) SITL 재기준선. (2) 와 (5) 가 통과하기 전에 SITL 을 돌리지 않는다.

## 7. 실행 기록과 다음 단계의 구체 목록 (2026-10-12 23:05)

**(1) 끝남 — main 에 fast-forward 병합 2026-10-12 23:22 (..275505f); split/step1 과 worktree 는 지움.**
`gz_model`: `OVERLAY`(env `AIRSHIP_OVERLAY`, 기본 `<repo>/overlay`), `MODEL_HOMES`/`AIRFRAME_HOMES` = [오버레이, 트리], `model_dir`/`has_model`/`model_names`/
`airframe_files`/`airframe_path`/`sourced_path`(`${R}etc/init.d-posix/airframes/<f>` 는 두 집에서, 나머지는 트리 ROMFS), `RIGID` 토큰 (`family_tokens` 가 `['rigid']`,
물리 토큰 옆에서는 거부; `vehicle()` 이 벗김; `bare(vehicle)` = `_rigid` 가 있으면 그것, 없으면 평범한 이름). 변환한 자리: sitl_link, added_mass_check, aero_moment_check
(`GZ_SIM_RESOURCE_PATH` 는 두 집을 모두), forward_yaw_check (ALLOWED), 그 시험 (MODEL_HOMES 패치), hover_ff (bare_airframe), controller (airframe_file/airframe_walk/
airframe_for_model), variant_check (목록·경로 16곳; decades() 와 사실 7/11/13 은 아직 트리 기준). 검증: 오버레이가 비어 있으므로 동작 동일 — run_tests.sh exit 0
(115 PASS + 253 통과, SKIP 1), 스크래치 트리의 variant_check 16/16. 덤: test_controller 가 `yaw_model/records` 의 기록도 재생한다 (CI 에서도 115).
`gz_link`: 렌치·해제를 `gz.transport13` 의 살아 있는 노드에서 발행 (구독자 연결 뒤; 월드 없으면 10 s 뒤 False). `gz topic -p` 의 유실이 오늘 밤 2520 rigid 게이트의 "I 켬" 사례를
적분기 2× (0.0370 vs 0.0185, 각속도 0.05 deg/s) 로 만들었다 — 루프는 렌치 둘을 잡고 있었다. 게이트를 다시 돌려 확인한다.

**(2) 끝남 (2026-10-13 11:50, 도구 main).** `variant_check.decades()` 가 트리 CMake 주석 (25x0) 과 오버레이 `overlay/airframes/decades`
(26x0, 있을 때) 를 합친다 (같은 기체가 둘에 있으면 SystemExit); 사실 7 은 멤버의 집에 따라 트리 (lta.sdf · CMake · docs 표) 나 오버레이
(`overlay/worlds/lta_lab.sdf` · decades 파일 · `overlay/vehicles.md`) 를 본다; 사실 12 (make 타깃) 는 트리 멤버만; 사실 13 의 정규식은
`2[56]\d\d`. gz_model 에 `OVERLAY_DECADES/OVERLAY_WORLD/OVERLAY_DOCS` 와 `in_overlay(path)`. 오버레이가 비어 있어 동작 동일 (16/16, 생성기
시험 17/17); 오버레이 멤버가 있는 검증은 (3) 의 트림된 스크래치 트리 시험에서.

**(3) 생성기 두 모드 끝남 (2026-10-13 오후, 도구 main eaa7400..55a0703).** `--upstream MEMBERS_ROOT`: 가족의 멤버들 (오버레이, 또는 오늘의 트리) 에서
upstream 이 가지는 둘 — 부가질량 몸체 (`<body>_base`) 와 평범한 이름의 완전 플랜트 (몸체 merge + 추진·핀·항력 블록 한 파일, 두 단) — 와 그
에어프레임 (bare 의 본문 + 완전 멤버의 요 값 블록) 을 쓰고, 쌍둥이 검사 (merged 모델 원소 단위, 에어프레임 resolved 기본값) 를 돌린다.
`--overlay`: 가족 전체를 rigid 이름 (`<name>_base_rigid`, `<name>_rigid`; 나머지 그대로) 과 JSON decade + 100 으로 오버레이에 (모델, 에어프레임,
`worlds/lta_lab.sdf`, `airframes/decades`, `vehicles.md`); 템플릿은 오버레이의 generic rigid 가족, 그것이 없으면 트리의 generic; 멤버 에어프레임은
그 기체의 트리 멤버 (요 값 블록째) 를 bare 포함만 바꿔 가져온다. **S1/S2 보정:** 오버레이는 제 부가질량 몸체 (`<name>_base_am`, 이름 그대로) 를
가진다 (자체 완결; `_am` 멤버가 트리의 `<name>_base` 를 merge 하지 않는다) — 두 몸체는 쌍둥이 검사가 같게 붙든다. 이유: 생성기 멱등성 (두 번째 실행이
템플릿을 오버레이에서 읽는다) 과 두 집의 이름 충돌 규칙. 시험: 트림된 스크래치 트리 (generic 가족 삭제) 에 `--upstream`, 스크래치 오버레이에 generic·
indoor `--overlay` → 세 쌍둥이 검사 모두 같음, 두 번째 생성 변화 0, 계산기 재작성 변화 0 (generator test 20/20). airship_gains 의 rigid bare 는 4efd7ca, fixed 가족의 upstream 작성기
(`airship_fixed`, 2500) 는 55a0703. 남은 것: cloudship 의 upstream 모양 (자체 완결), 트리 쪽 variant_check 의 upstream 모양 사실, (4) 플랜트
레지스트리, (5) gz_run.sh, (6) 트림, (7) 재기준선.

**(4) 끝남 (2026-10-14 11:30, 도구 main 8971ee2..fec3bf8).** `yaw_model/plant_registry.py`: 키는 오버레이 이름 (`<vehicle>_rigid`/`_fin`/`_am_fin_drag`, generic 과
cloudship 여섯), 항목은 기록이 날던 날의 트리 이름·SYS_AUTOSTART·모델 디렉터리·plants.EXPECTED 스냅샷, `model` 은 오늘 그 플랜트를 가진 집의 이름
(오버레이에 있으면 키, 아니면 트리 이름: gz_model.bare 의 규칙). 별칭 `OVERLAY_NUMBER` (25x0..7 → 26x0..7, 일곱 decade) 와 `PLAIN_AFTER_TRIM`
(2520 = 옛 2527, 2510 = 2517, 2500 = 2505). 소비자 전환: record_gains.RUNS/gains(), closed_loop.TARGETS/EXTRA/FORWARD (`{e.target: (e.model, e.family)}`),
plants.EXPECTED (뷰), forward_replay (로그 검사는 number_tree, 배분기 검사는 오늘의 파일), sitl_check.VEHICLES, reproduce/reproduce_head/validate_gate.
기록 파일은 손대지 않음. 시험 `yaw_model/test_plant_registry.py` (131 PASS; run_tests 254/0, SKIP 1 그대로). 트림 (6) 에서 그 시험의 두 단언
(`model == tree_name`, 트리 에어프레임 존재) 을 오버레이 쪽 (`model == key`, `<number_overlay>_gz_<key>`) 으로 뒤집는다 — S3 의 재앵커링.
**(5) 끝남 (2026-10-14, 도구 split/step5: 2286820 생성기 월드, 5b7b12d 월드 변수, a72d976 실행 경로; (4) 는 아직).** gz_run.sh 가 모델의 집으로 경로를 고른다: `GZ_TARGET=gz_<model>` 의 `<model>` 에 `$AIRSHIP_OVERLAY/models/<model>/model.sdf` 가 있으면 오버레이 경로 (두 집에 다 있으면 exit 8, `<n>_gz_<model>` 에어프레임이 없으면 exit 8, PX4_GZ_WORLD 가 lta_lab 이 아니면 exit 9), 아니면 트리 경로 그대로. 오버레이 경로 = S4: `make px4_sitl` (빌드만, 1800 s 상한) → `overlay/install.sh <tree>` (AIRSHIP_OVERLAY 를 읽고 `[0-9]*_gz_*` 만 복사) → 빌드 `gz_env.sh` source + `GZ_SIM_RESOURCE_PATH` 앞에 오버레이 models → `gz sim --verbose=1 -r -s overlay/worlds/lta_lab.sdf` (준비 = `/world/lta_lab/scene/info` 의 제공자, 30 s; gz.log 의 "Failed to load system plugin" 은 exit 4) → `/world/lta_lab/create` 로 `<model>_0` 을 `file://` 모델 파일에서 PX4_GZ_MODEL_POSE 로 스폰 (create 는 include 가 풀리지 않아도 true 라 `pose/info` 에 이름이 있어야 함) → `bin/px4` 를 rootfs 에서 `PX4_SIM_MODEL=gz_<model> PX4_GZ_STANDALONE=1 PX4_GZ_MODEL_NAME=<model>_0 PX4_GZ_WORLD=lta_lab` 로 (stdin 파이프, SDLOG_DIRS_MAX 0) → 기존 uORB·"world is ready" 대기 (+ "gz_bridge failed" 분기). 검사에는 PX4_GZ_ENTITY, PX4_GZ_WORLD=lta_lab, AIRSHIP_OVERLAY 가 간다; exit-10 규칙은 트리 경로의 것 (standalone px4 는 다른 월드를 찾지 않는다). 하드코딩 `lta` 는 sitl_link.WORLD, forward_yaw_check 의 target 해석, gate.py 의 sitl_link stub 세 곳 → `gz_model.WORLD` 하나 (기본 lta; aero_moment/added_mass 의 probe 월드와 variant_check 의 트리 lta.sdf 는 실행 월드가 아니라 그대로). 생성기: lab 월드는 템플릿 트리가 아니라 PX4_ROOT (실행 트리) 의 lta.sdf 사본 — 포크 lta.sdf 의 `model://ground_markers`·`flight_grid` include 가 트림된 트리의 서브모듈에 없어 서버가 월드를 거부했다 (generator test 23/23 그대로). 검증 (트림된 트리 px4-pr-airship e93b52d0e1, 서브모듈 e9580fce): 스크래치 오버레이 = `PX4_ROOT=<trimmed> AIRSHIP_OVERLAY=<scratch> airship_family_gen.py vehicles/airship_generic.json --overlay --root <fork 27fd5f1ffd>` (가족은 포크에서, 월드는 트림된 트리에서; 트리 둘 다 읽기만); 오버레이 게이트 `GZ_TARGET=gz_airship_vectored_independent_rigid` (2620, lta_lab) **10/10**, 트리 대조 `gz_airship_vectored_independent` (2520, lta) **10/10**. 트림된 펌웨어가 거부한 줄: 2620 의 `CA_AIRSHIP_GRP 1` 하나 (rcS 는 set +e 라 경고뿐, 게이트에 영향 없음); 2627 의 AS_SPD_SRC/WMAX/HI·AS_YR_FF_SLP 도 그 펌웨어에 없다 (돌리지 않음). 남은 것: (4) 플랜트 레지스트리, (6) 트림, (7) 재기준선; `inject_flaps` 는 조종면 가족의 플랩 템플릿을 아직 gz_model.GZ_MODELS (PX4_ROOT) 에서 읽는다 (generic 은 조종면이 없어 안 닿음); 오버레이 실행 뒤 빌드 etc 에 남는 26xx 사본은 이름이 달라 트리 경로에 무해; cleanup 의 bash "Killed/Terminated" 잡 알림은 전부터 있던 소음.

**(6) 생성기 쪽 끝남 (2026-10-14, 도구 split/step6a: fbb643e variant_check, 096c9cb 생성기, + 이 문서 커밋).** `--overlay` 가 generic 의 고정 쌍둥이 (트리 2500-2505 → `airship_fixed_rigid` 2600/2601/2604/2605: 멤버 에어프레임은
트리 것을 bare 포함만 바꿔 복사, 플랩 서보 범위는 생성기가, 월드·decades·vehicles.md 의 절은 트리 vehicles.md 의 절을 이름만 바꿔 복사) 를 함께 쓰고,
`airship_family_gen.py cloudship --overlay` (기체 파일 대신 이름: Cloudship 은 설계된 기체가 아니라 이식된 모델이라 hull·pods 가 없고, JSON 을 두면 `vehicle_files()` 를 읽는
TWINS 가 2520 으로 잘못 짝짓고 VEHICLE_ALLOCATION 이 pods 를 찾는다) 가 트리의 여섯 멤버를 bare 만 `cloudship_rigid` 로 (2610..2617; `2507_cloudship` 과 classic 의
`cloudship/cloudship.sdf.jinja` 참조는 그대로) 복사한다 (두 번째 실행 변화 0; 오버레이에 이미 있으면 트리를 읽지 않으므로 트림 뒤에도 돈다). `--upstream` 은 셋 다 쓴다:
cloudship 평범 플랜트는 오버레이 `cloudship_am` 사본에 `_am_fin`·`_am_fin_drag` 의 제 블록을 붙인 자체 완결 한 파일, posix 2510 은 rigid bare 의 본문 (변형 설명 문장 —
physics mask·CMakeLists·rigid hull·member 를 말하는 문장 — 을 빼고 "This is the complete plant: ..." 한 문장을 붙임) + 완전 멤버의 include 뒤 블록 전부 (MARK 가 없어도).
`--upstream` 이 이제 posix CMakeLists 항목과 문서 항목 셋 (vehicles.md 절, index.md 행, frames_airship 불릿; index.md 의 번호로 정렬) 도 쓴다 — 트림이 지우고 생성기가
되돌리는 모양. variant_check 는 두 모양에서 돈다: `plain(vehicle)` (bare 가 오버레이 `_rigid` 인 기체의 트리 평범 이름) 은 사실 1 의 사슬을 완전 멤버 모양으로, 사실 7 의 번호를
십 자리 − 100 (마스크 0) 으로, 사실 8 의 목록을 완전 멤버의 토큰으로 읽고, 사실 13 은 오버레이 십 자리 − 100 의 트리 번호를 받으며, 사실 14 의 STABLE 은 `_base_rigid` 의 머리도
읽고 (안정 약속은 rigid 몸체의 머리에만 있다), 사실 2 는 자체 완결 기체의 사본을 bare (`cloudship_rigid`) 와 비교한다; **사실 17** (새로): 평범 플랜트 == 오버레이 완전 멤버
(merged 원소 단위, 에어프레임 기본값 − `gz_model.UPSTREAM_ABSENT`). 시험: 생성기 26/26 (cloudship 복사 멱등, cloudship upstream 쌍둥이, `--plants`, 트림된 스크래치 트리 +
오버레이 전체 (기체 5 + cloudship, 모델 43 = 오버레이 40 + 평범 3) 에서 variant_check 17/17), 오늘 트리 17/17, run_tests 254 PASS / 0 FAIL / SKIP 1.

**(6) 절차 — 트림 (다음 에이전트가 그대로 실행).** 포크 트리 `~/dev/PX4-Autopilot` (airship/dev 27fd5f1ffd, 서브모듈 0c4307b) 은 끝까지 읽기 전용 템플릿으로 두고, 트림은
worktree 에서 한다. 도구는 main 에 이 단계가 병합된 뒤.

1. worktree 와 가지: `cd ~/dev/PX4-Autopilot && git worktree add ../px4-trim -b airship/trim airship/dev && cd ../px4-trim && git submodule update --init Tools/simulation/gz
   && git -C Tools/simulation/gz checkout -b airship/models-trim`. 아래 `T=~/dev/px4-trim`, `F=~/dev/PX4-Autopilot`, `G=~/dev/px4-airship-tools`.
2. 서브모듈 삭제 (`$T/Tools/simulation/gz`): `git rm -r models/airship_* models/cloudship*` (50 디렉터리: 벡터링 가족 5 × 8, airship_fixed 4, cloudship 6). `worlds/lta.sdf`:
   `<enable>` 줄 전부 (airship_*·cloudship*, 200 줄: Buoyancy 플러그인 안에는 다른 enable 이 없다). 커밋 (`refactor(airship): trim the families to the plain plants`).
3. PX4 삭제 (`$T`): `git rm ROMFS/px4fmu_common/init.d-posix/airframes/25[0-6][0-9]_gz_*` (40 개; `2519_gazebo-classic_cloudship` 은 남긴다); 그 `CMakeLists.txt` 의 `\t25xx_gz_*`
   40 줄과 "Airship gz decades" 주석 블록 (표 7 줄과 그 위 설명 7 줄; `2519_gazebo-classic_cloudship` 줄은 남긴다); `docs/en/sim_gazebo_gz/vehicles.md` 의 `## Airship` 아래
   `###` 절 40 개 전부 (서론 두 문단은 남기되 명명 규칙·마스크·"Regress the allocator on the rigid hull" 문단은 S6 대로 손으로: 가족은 px4-airship-tools 에 있다는 한 줄 +
   링크); `docs/en/sim_gazebo_gz/index.md` 의 `make px4_sitl gz_airship_*`/`gz_cloudship*` 행 40 개; `docs/en/frames_airship/index.md` 의 Simulation 절 불릿 7 개와
   "Each is a family of models named by..." 문장 (레포 링크 한 줄로); `docs/en/sim_gazebo_gz/worlds.md` 50-53 행 ("lists each airship model's bare name and instances
   0 to 3" → 평범 플랜트 셋); 비행 `init.d/airframes/2520_airship_vectored_independent` 32-38 행 (변형 언급); `src/.../AirshipYawScheduleTest.cpp:39` 주석. 아직 커밋하지 않는다.
4. 오버레이 생성 (트리는 포크에서, 월드는 트림된 트리에서 — (5) 의 검증과 같은 조합): `cd $G && for v in airship_generic airship_indoor airship_aurora airship_jaxa airship_mpi; do
   PX4_ROOT=$T python3 airship_family_gen.py vehicles/$v.json --overlay --root $F; done && PX4_ROOT=$T python3 airship_family_gen.py cloudship --overlay --root $F`.
   결과: `overlay/models` 50 디렉터리, `overlay/airframes` 40 파일 + `decades` 7 줄 (2600 airship_fixed_rigid, 2610 cloudship_rigid, 2620, 2630, 2640, 2650, 2660),
   `overlay/worlds/lta_lab.sdf` (enable 200 줄, `<world name="lta_lab">`), `overlay/vehicles.md` (행 40, 절 40). 같은 명령을 한 번 더: "airframes written: none; world entries added 0".
5. 평범 플랜트 셋을 오버레이에서 트림된 트리로: `PX4_ROOT=$T python3 airship_family_gen.py vehicles/airship_generic.json --upstream overlay --root $T` (`airship_base`,
   `airship_fixed` + 2500, `airship_vectored_independent` + 2520) 와 `PX4_ROOT=$T python3 airship_family_gen.py cloudship --upstream overlay --root $T` (`cloudship` + 2510).
   각각 "twin check OK". 쓰는 것: 모델 4 (model.sdf + model.config), posix 3, CMake 항목 3, `lta.sdf` enable 15 줄, vehicles.md 절 3 / index.md 행 3 / frames_airship 불릿 3
   (2500, 2510, 2520 순). 2500·2510 의 머리 주석은 생성기가 "This is the complete plant: ..." 로 고쳐 쓴다; 문서 절 셋은 upstream 모양 (≤ 3 문장) 으로 손질하고 스크린샷은 있으면.
6. 검사: (a) `PX4_ROOT=$T AIRSHIP_OVERLAY=$G/overlay python3 variant_check.py` → `통과 17 / 실패 0`, `모델 43 개`; (b) S6 게이트 — `cd $T && grep -rnE
   "\b25(0[1-9]|1[1-8]|2[1-9]|[3-6][0-9])\b|_am_fin|_am_drag" ROMFS docs/en src/modules/control_allocator src/modules/airship_att_control src/lib/control_allocation` 이 비어야
   한다 (S6 의 `src` 전체 grep 은 tflite 시험 자료·msp_osd 의 무관한 숫자에 걸린다); (c) `grep -rn "airship_\|cloudship" $T/Tools/simulation/gz/worlds/lta.sdf | wc -l` = 15;
   (d) `cd $T && make px4_sitl_default` (오버레이 없이 트리만: gz 없이 빌드); (e) 도구 쪽 — `pins.env` 를 트림 커밋으로 바꾸기 **전에** 시험을 트림 모양으로 옮긴다:
   `airship_family_gen_test` 의 트리 모드 세 경우 ([generic round trip]·[indoor family]·[surfaces option] 은 트리에 가족이 있을 때의 것) 를 오버레이 왕복 (`overlay/` 사본에
   `--overlay` 다섯 + cloudship → 변화 0) 과 `--upstream` 왕복 (트리 사본에 `--upstream overlay` 둘 → "wrote nothing; twin check OK") 으로, `yaw_model/test_plant_registry.py` 의
   두 단언을 (4) 에 적은 대로 (`model == key`, `<number_overlay>_gz_<key>` 존재) 뒤집고, `apply_generic_hull.main()` 의 기본 (`plan(..., GENERIC, M)`·`write_cloudship()`) 을
   오버레이 (`GENERIC_RIGID`·`CLOUDSHIP_RIGID`, `gz_model.OVERLAY/models`) 로; 그 뒤 `PX4_ROOT=$T ./run_tests.sh` 녹색.
7. 커밋과 푸시 순서 (gz-submodule-publish-order): 서브모듈 가지 `airship/models-trim` 을 포크에 먼저 푸시 → PX4 `airship/trim` 커밋 (서브모듈 포인터 포함; 3·5 의 변경 한
   커밋 또는 삭제/생성 두 커밋) → 도구 레포: `overlay/models overlay/airframes overlay/worlds overlay/vehicles.md` 와 6(e) 의 시험 변경, `pins.env` (PX4_COMMIT·GZ_MODELS_COMMIT
   = 트림 커밋, 가지 이름) 을 커밋. 사용자가 diff 를 검토하기 전에는 푸시하지 않는다 (commit-after-user-review).
8. 남는 것 (건드리지 않음): posix `2519_gazebo-classic_cloudship` 과 classic 모델; 비행 에어프레임 `2500_generic_airship`·`2507_cloudship`·`2520_airship_vectored_independent`
   (2520 은 32-38 행만); 넓은 펌웨어 — `AS_SPD_*`·`AS_YR_FF_SLP`·`CA_AIRSHIP_GRP` 는 포크 펌웨어에 남고 오버레이 에어프레임만 설정한다 (평범 플랜트 셋은 설정하지 않는다:
   `UPSTREAM_ABSENT`); 요 모델 기록 파일. 그 다음이 (7) SITL 재기준선: 트리 2520 게이트 (= 오늘 2527 10/10), 2510 (= 2517 9/9), 2500 비행 검사; 오버레이 2627·2617·2637
   (전진 42/42) 는 `GZ_TARGET=gz_<model> ./gz_run.sh` 로.

**(6)/(7) 끝남 (2026-10-14, 트림 worktree `~/dev/px4-trim` airship/trim 02541aa001 삭제 + b86228fb4e 생성, 서브모듈 airship/models-trim 002b01e 삭제 + 2628193
생성, 도구 split/step6b 8900a8b..40f3aa7 + 이 문서; 전부 로컬, 푸시 안 함; 포크 체크아웃 27fd5f1ffd 과 px4-pr-airship 은 손대지 않음).**
- 삭제: 서브모듈 모델 50 디렉터리 (101 파일, 10,099 줄) 와 enable 200; PX4 posix 25xx 40, CMake 55 줄 (항목 40 + decades 주석 14 + 빈 줄 1), vehicles.md 절 40 (366 줄),
  index.md 행 40, frames_airship 불릿 7 + 가족 문장 ("Running an airframe against several of its plants" 문장도 레포 링크 한 줄에 접음), worlds.md 한 줄, 비행 2520 의 32-38,
  AirshipYawScheduleTest.cpp:39 — 48 파일 −1614/+10. PX4 는 삭제 / 생성 두 커밋으로 각각 서브모듈의 제 커밋을 가리킨다; 삭제 상태는 시험의 `trim()` 으로 만들고 복원을
  해시로 검증했다 (trim + `--upstream` 둘 = 트리, 바이트 동일).
- 생성: 오버레이 모델 50 / 에어프레임 40 / decades 7 / lta_lab enable 200 / vehicles.md 절 40 행 40, 두 번째 실행 md5 동일; `--upstream` 둘이 모델 4 + posix 3 + CMake 3 + enable 15
  + 문서 3/3/3 을 쓰고 twin check OK. 2500·2510 머리는 "This is the complete plant"; 문서 절 셋은 생성된 두 문장 + make 명령 그대로 (스크린샷 없음).
- 검사: (a) 17/17, 모델 43; (b) 적힌 grep 은 비지 않는다 — 2507 (남기는 비행 Cloudship: airframe_reference.md, frames_airship:78, 할당기 시험 둘, 2510 머리의 "same vehicle as 2507")
  과 무관한 upstream 숫자 (Arduino 2560, arXiv 2509.11481, VEHICLE_CMD 2501/2511, 2560×1440) 뿐, 그것을 빼면 빈다; (c) 15; (d) `make px4_sitl_default` rc 0, `make check_format`
  통과 (생성기가 vehicles.md 끝에 빈 줄을 남겨 한 번 실패 → f9a5b4a); (e) `PX4_ROOT=$T ./run_tests.sh` 녹색: 생성기 17/17, variant_check 17/17, forward 196/0, registry 0 failed,
  controller 0 failed + SKIP 10 (기록 ulog 는 포크의 build/log 에만 있어 worktree 와 CI 에는 없다), sitl_link.
- 절차 밖의 변경: ① `CA_AIRSHIP_GRP` 를 `UPSTREAM_ABSENT` 에서 뺌 (83dedc7): 포크 펌웨어의 기본값이 0 (collective) 이라 GRP 없는 평범 2520 은 집단 틸트로 할당됐다 (전진 시험의
  13 번째 기체가 그렇게 잡혔다); 비행 에어프레임처럼 평범 셋이 제 그룹을 적는다 (2500·2510 0, 2520 1), PX4 생성 커밋에 반영. ② forward_yaw_check 의 ALLOWED 를 토큰에서 merged
  모델의 플랩 없는 lift-drag 핀으로 (S5 의 뜻; 토큰 규칙은 평범 완전 플랜트를 거부했고, 고정 가족의 플랩 핀은 방향타라 제외), 평범 cloudship 의 다리 표 항목 (0cad4fa).
  ③ 시험 재지정은 세 경우를 넘었다: [upstream]/[cloudship]/[trimmed tree] 도 트리 가족을 템플릿·MEMBERS_ROOT 로 읽었고, test_plant_registry 의 PLAIN_AFTER_TRIM 검사,
  cloudship_actuation.AIRFRAME 과 closed_loop 의 rigid 사슬 → `gz_model.bare('cloudship')` (2610), test_controller 는 그 경유 (7ae9ee8); 시험의 `trim()` 은 불릿 뒤 빈 줄도 먹는다.
  ④ 평범 2520 의 다리: AS_SPD_HI 없음 → 펌웨어 기본 램프 2.5 → 1.90/4.00 m/s (2627 은 2.1 → 1.60/3.35).
- (7) 재기준선 (`PX4_ROOT=$T GZ_TARGET=gz_<model> ./gz_run.sh ./<check>`, 한 번에 하나, timeout 1800):

  | 대상 | 검사 | 경로 / 월드 | 결과 | 전 기준 |
  |---|---|---|---|---|
  | 2520 `airship_vectored_independent` | sitl_check | tree / lta | 통과 10 / 실패 0 | 2527 10/10 |
  | 2520 `airship_vectored_independent` | forward_yaw_check | tree / lta | 통과 28 / 실패 0 | 2527 42/42 — 스케줄 꺼짐 (`AS_SPD_SRC 0`) 이라 다리마다 스케줄 판정이 없다 (14), 다리 1.90/4.00 |
  | 2620 `_rigid` | sitl_check | overlay / lta_lab | 통과 10 / 실패 0 | 10/10 |
  | 2621 `_drag` | sitl_check | overlay / lta_lab | 통과 10 / 실패 0 | 10/10 |
  | 2510 `cloudship` | sitl_check | tree / lta | 통과 9 / 실패 0 | 2517 9/9 |
  | 2612 `cloudship_fin` (POSE 0,0,80) | forward_yaw_check | overlay / lta_lab | 통과 43 / 실패 0 | 2512 43/43 |
  | 2510 `cloudship` | cloudship_land_disarm_check | tree / lta | 통과 10 / 실패 0 | 10/10 |
  | 2500 `airship_fixed` | airship_fixed_flight_check | tree / lta | 통과 12 / 실패 1 | 13/13 (10-07) |

- 열린 것: 2500 비행 검사의 "할당기가 모터 둘과 조종면 넷을 잡았다" 는 `Configured actuators` 6 을 기대하는데 2f013b1a4a (10-13, 틸트 슬롯 둘을 모든 그룹에 선언, 고정
  마운트는 구동 안 함) 뒤 8 이다 — 트림과 무관, 마지막 기록 13/13 은 그 커밋 전; 검사의 기대값은 사용자가 정한다. 평범 2520 은 속도 스케줄 없이 난다 (설계대로). 문서 절의
  "airship_base merged with ..." 문구는 생성기의 것. 푸시는 gz-submodule-publish-order 대로 서브모듈 먼저, 사용자 검토 뒤.

**(3) 의 원래 변경 목록 (참고).**
- `apply_generic_hull.Family`/`vectored_family(name, title, surfaces, rigid=False)`: rigid 가족은 `bare = name + '_rigid'`, `base = name + '_base_rigid'`, `base_am` 그대로,
  나머지 그대로. `GENERIC` 도 두 판 (트리용 지금 것, 오버레이용 rigid). `copy_family` 의 renames 는 긴 이름부터 (이미 그렇다) — `_base_am` 이 `_base_rigid` 로 잘못 바뀌지 않게
  base 앞에 base_am 을 둔다 (지금 정렬 규칙 `-len` 이 보장).
- `write_models(vehicle, root, template_root)`: 템플릿을 `gz_model.GZ_MODELS` 가 아니라 인자로 (오버레이 모드는 오버레이에 커밋된 generic rigid 가족을 템플릿으로).
  `A.plan(..., root)` 과 `write_family` 는 root 아래 역할 이름으로 쓰므로 Family 이름만 바꾸면 따라온다 (확인 필요: write_body 등이 `fam.base` 를 쓰는지 grep).
- `write_airframes(vehicle, p, airframes, decade)`: 번호는 인자 (`vehicle.decade + 100` 오버레이), bare 템플릿은 오버레이 모드에서 `<2620>_gz_airship_vectored_independent_rigid`
  (오버레이에 커밋), 멤버는 `. ${R}etc/init.d-posix/airframes/<bare 파일>` 로 bare 를 source — rcS 는 빌드 etc 에 복사된 파일을 같은 디렉터리에서 찾으므로 그대로 통한다.
- `write_world(path, fam)`: 오버레이는 `overlay/worlds/lta_lab.sdf` (없으면 트리 `lta.sdf` 사본, `<world name="lta_lab">`); 트리 모드는 upstream 셋의 entry 만.
- `write_cmake` → 오버레이 모드에서는 `overlay/airframes/decades` (`<decade> <bare 모델>` 한 줄씩) 에 쓰고 트리 CMake 는 건드리지 않음; `write_docs` → `overlay/vehicles.md`
  (같은 표 모양). variant_check 의 `decades()` 는 두 원천 (오버레이 파일 / 트리 CMake 주석) 을 모드로 읽는다.
- `--upstream` (트리 기본): 완전 멤버만 — `<body>_base` (= base_am 내용, 이름만), `<vehicle>` (= am_fin_drag 의 merged 결과를 base include 하나 + 제 블록으로: 즉 오늘의
  `_am_fin_drag` 가 `_am_fin` 을, 그것이 `_am` 을, 그것이 `_base_am` 을 merge 하는 체인을 **두 단으로 접는다**: `<vehicle>` 은 `<body>_base` 를 merge 하고 추진·핀·항력
  블록을 직접 가진다), 에어프레임 `<decade>0_gz_<vehicle>` (완전 값, 출처 한 줄), 월드 entry 5, docs 항목 1. cloudship 은 자체 완결 (cloudship_am + 핀·항력).
- 검사: `check --overlay` = 오버레이 디렉터리 사본에서 멱등성; `check` (트리) = 쌍둥이 검사 호출 (variant_check 에 추가: 오버레이 `_am_fin_drag` merged == 트리 `<vehicle>` merged,
  원소 단위; 에어프레임은 resolved 기본값). 시험: `airship_family_gen_test` 에 **트림된 스크래치 트리** (25xx 가족 멤버와 가족 모델을 지운 사본) 에서의 오버레이 생성 + 쌍둥이.
- 오버레이 내용은 트림 커밋과 같은 증분에서 채운다 (그 전에는 두 집에 같은 이름이 있어 model_names() 가 거부한다 — 의도된 안전장치).
