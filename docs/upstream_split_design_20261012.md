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

**(1) 끝남 — 브랜치 `split/step1` (worktree `~/dev/px4-airship-tools-split`, 푸시됨; 체인이 끝나면 main 에 병합).**
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

**(3) 생성기 오버레이·upstream 모드 — 읽은 구조에서 나온 변경 목록 (다음 세션).**
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
