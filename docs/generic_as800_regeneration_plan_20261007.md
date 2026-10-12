# generic 가족을 AS800 급으로 재생성: 실행 계획 (2026-10-07 19:30, 교수 → 실행 역할 인계)

결정 근거: reference_vehicle_selection_20261007.md (대표 기체 = AURORA/AS800 급, 34 m³, L/D 3.5). 이 문서는 Opus 가 실행할 수 있게
쓴 것이지만 **그대로 믿지 말 것**: 교수 역할이 코드를 다시 읽지 않고 기억으로 쓴 전제가 있다 (8절에 표시). 전제는 코드·측정으로
먼저 확인하고, 틀리면 코드·측정을 따르고 표에 적는다. 설계 결정이 필요한 것 (9절) 은 추측하지 말고 멈춰 적는다. 규칙: 서브에이전트·워크플로 없음, 게이트는 배경 스크립트 하나로 묶어 요약만 읽기, 커밋은 로컬·서명 없음·
`Assisted-by: Claude:claude-opus-5-5` 트레일러, Co-Authored-By 없음, 푸시 없음, 끝나면 멈추고 사용자 검토 (교수 역할이 결과를 검토).

## 0. 바뀌는 것과 안 바뀌는 것

- 바뀜: `vehicles/airship_generic.json` 의 선체 (`hull_volume` 39.2 → **34.0**, `fineness` 4.2 → **3.5**), 그에 따라 생성되는 모델
  SDF (선체·핀·부가질량·드래그), posix 에어프레임 2500~2505/2520~2527 의 값, 문서 표의 치수, 비행 에어프레임 2520/2500 의 요 이득 값.
- 안 바뀜: 이름·번호 (airship_vectored_independent, 2520 대, airship_fixed 2500 대), 포드 (18 N, pod_y 1.5, ±180°), 곤돌라 상자,
  설계 규칙 (R1 1.2×Munk, 플랩 ±11°, 실속 여유), 파라미터 이름, 코드 경로.
- 곤돌라 질량: 중립 부력이 되도록 생성기/설계 스크립트의 보고를 보고 맞춘다 (`generic_hull_design.py --spec` 이 "all requirements
  met" 과 질량 수지를 낸다; 전비 = 공기 밀도 × 부피 규칙). AS800 의 탑재 10 kg 은 곤돌라 질량 안이다.
- 등가 타원체 9.27 × 2.65 m 는 입력이 아니라 결과다 (부피·fineness 에서 생성기가 낸다). 확인만 한다.

## 1. 계산기의 기준 닻을 상수로 (먼저, 생성 전에)

`airship_gains.py` 는 지금 `plant(REFERENCE)` 로 트리의 generic 플랜트를 읽어 대역폭 N·P/I 와 적분 시간 P/I 를 만든다. generic 이 바뀌면
순환이므로, 11 m 플랜트의 측정값을 상수로 박는다:
```
REF_PLANT = {'N': 54.0, 'I_rigid': 129.2, 'rr': 192.2, 'k': 228.7, 'length': 10.97}   # the 10.97 x 2.61 m generic, measured 2026-09/10
```
`gains(p, ref)` 와 `blocks()` 의 `ref[...]` 는 이 상수를 쓴다 (docstring 의 "The reference is the generic vectored airship, whose values were
tuned and measured in SITL and at the pins" 를 "an 11 m plant that no longer exists in the tree; its loop bandwidth 0.1176 rad/s and
integral time 7.78 s are the anchor" 로 고친다). `airship_family_gen_test.py` 의 "generic 기체는 제 이득을 돌려준다 (P 0.7, I 0.09, 기울기
4.01 의 7 % 안)" 판정은 새 generic 에 맞게 바꾼다: 새 generic 의 P/I 는 규칙이 내는 값 (0.7/0.09 가 아님), 기울기는 씨앗 vs 측정 3 % 안.

## 2. 생성

```
cd ~/dev/px4-airship-bench-tools
$EDITOR vehicles/airship_generic.json        # hull_volume 34.0, fineness 3.5, gondola_mass (수지)
python3 generic_hull_design.py --spec vehicles/airship_generic.json      # all requirements met 확인, 질량 수지
python3 airship_family_gen.py vehicles/airship_generic.json             # 모델·월드·에어프레임·CMake·문서 (제자리)
python3 airship_gains.py airship_vectored_independent --write           # 2520/2522/2527 표식 블록 (씨앗 기울기)
python3 variant_check.py && python3 airship_family_gen_test.py
```
주의: 생성기의 "기존 멤버는 이득을 유지" 규칙 때문에 generic 에어프레임의 손으로 적은 값 (P 0.7, I 0.09, FF 1.0/0.3, SLP 4.01, 램프
1.5/2.5) 이 남을 수 있다. 이 값들은 11 m 플랜트의 것이므로 **표식 블록으로 대체**한다 (실내 가족과 같은 꼴). 비행 에어프레임
2520_airship_vectored_independent 와 2500_generic_airship 의 AS_* 값도 새 값으로 (variant_check 의 "gz 쌍둥이의 할당 블록이 비행
에어프레임의 것" 판정이 지켜 준다).

## 3. 11 m 숫자를 박아 둔 곳 (grep 2026-10-07 19:29; 전부 새 파일값으로 바꾸거나 파일에서 읽게 한다)

airship_family_gen_test.py (3), apply_generic_hull.py (1), gz_model.py (1: fin_yaw_damping docstring 의 216.9/229.8),
yaw_model/plants.py (6), yaw_model/coupling.py (2), yaw_model/vectored_actuation.py (2), yaw_model/vectored_compare_samples.py (3),
yaw_model/vectored_gate_replay.py (2), yaw_model/fin_damping.py (1), forward_yaw_check_test.py (3: "229.8 u", 2527 FF 1.0 등).
sitl_check.py 의 VEHICLES 표 (generic 의 권한 54 N m) 도 새 N 으로. 패턴: `229\.8|10\.97|129\.2|192\.2|4\.01\b|39\.1|38\.9|1\.5, 2\.5`.
요 모델 (yaw_model/plants.py 등) 이 CHAIN_LOGS 의 엔진 측정 블록 ("=== added_mass airship_vectored_independent ===") 을 읽는 부분은
**이번에 건드리지 않는다** — 새 선체의 블록은 5절의 미룬 항목. 요 모델 시험 (test_policies 등) 이 그 블록 때문에 깨지면 깨진 목록만
적고 넘어간다.

## 4. 게이트 (배경 스크립트 하나, 요약만 읽기)

순서대로, 각 결과를 이 파일의 6절 표에 적는다:
1. `make px4_sitl_default` 빌드.
2. 정지 게이트 `sitl_check.py` on gz_airship_vectored_independent (2520) 와 _am_fin_drag (2527): 기대 10/10 (실내와 같은 판정 수).
3. 전진 `forward_yaw_check.py` on 2522 (_fin) 과 2527 (완전): 다리 속도는 램프에서 자동. 완전 플랜트 42/42, 핀 플랜트는 다리 3 이
   감속 불가로 못 날 수 있음 (알려진 것).
4. `airship_gains.py airship_vectored_independent --measure <2527 요약> --write` → 기울기를 측정값으로, 에어프레임 재작성 → 전진 2527
   한 번 더 (측정값으로; 42/42, k 재현 3 % 안).
5. 첫 비행 7/7 (`first_light` 류 검사가 있으면), `variant_check`, 생성기 시험, forward_yaw_check_test, 2500 쌍둥이 (airship_fixed)
   비행 검사 13/13.
6. `python3 build_fmu.py` (v6x) 빌드만; 플래시·지문은 사용자.

## 5. 이번에 하지 않는 것 (교수 검토 뒤 별도)

- 요 모델의 엔진 측정 블록 재측정 (added_mass_check 체인 로그) 과 d1_sitl_compare 의 모델 대조.
- 벤치 지문 (사용자 플래시 뒤).
- 도구의 트리 이전 (upstream_pr_plan 의 "PX4 관례" 1번).
- Jones & DeLaurier 의 선체–핀 간섭 계수 도입.

## 6. 결과 표 (실행 역할이 채움)

| 단계 | 결과 | 비고 |
|---|---|---|
| 설계 보고 (등가 타원체, 핀, 질량 수지) | **all requirements met.** 9.266 × 2.647 m, 34.0 m³, 전비 42.63 kg (기낭 8.23, 곤돌라 30.9, 핀 2.5, 포드 1.0); 핀 1.54 (등·옆) / 1.21 m (배), 시위 1.2; R1 1.203 × Munk, R2 1.393, G1 0.056 m; 플랩 ±10° (R3 최악 0.72 × 실속); 요 관성 87.7 + 105.4 (부가) kg m² (옛 129.2 + 192.2) | DEFAULT == JSON 확인 |
| 생성·멱등·variant_check·gen test | 생성기 결함 1 수정 (드래그 주석 정규식); generic `--check` IDENTICAL, 실내 `--check` IDENTICAL (generic 재생성 전후 모두, 실내 이득 불변); variant_check 16/16; gen test 14/14 (11 m 고정 판정 → 닻 상수 가드 + 대역폭·적분시간 판정) | 모델 15 파일, 에어프레임 2500 (플랩 ±10)·2520·2522·2527 |
| 정지 2520 / 2527 | **2520 (강체) 8/10**, **2527 (완전) 10/10** | 2520 실패 둘: "Acro 제동" 40.9 → −8.4 deg/s (옛 선체도 실패한 기준선, 9/24 기록 43.8 → −7.1), "Acro 추종" 평균 6.12/6.67 (앞 판정의 과회전이 60 s 안에 서지 않아 전제 깨짐). 원인: 규칙이 완전 플랜트의 대역폭을 맞추므로 강체 플랜트의 대역폭이 0.293 → 0.259 rad/s (부가질량 비 rr/I 1.49 → 1.20). 이득 규칙은 9절이라 두었다 (D-d) |
| 전진 2522 / 2527 (씨앗) | **2522 42/42** (세 다리 다), **2527 42/42**; 다리 2.20 / 4.65 m/s (램프 1.45/2.91 의 중앙과 1.6 HI); 핀 받음각 ≤ 0.38 × 실속, 적분기 한계 없음, 놓은 뒤 120 s 헤딩 ≤ 1.1° | |
| 측정 k vs 씨앗, 전진 2527 (측정값) | 측정 k 156.0 (153.8 / 157.6 / 156.7) vs 씨앗 152.4: **+2.4 %**; SLP 2.82 → 2.89, 램프 1.4/2.9, FF 0.609 (완전) / 0.205 (핀). 측정값으로 **2527 42/42**, 다리 2.15 / 4.65 m/s, k 재현 156.1 (+0.1 %), 핀 ≤ 0.39 × 실속, 놓은 뒤 120 s 헤딩 ≤ 1.1° | vehicles/airship_vectored_independent.measured.json; 2522 는 측정값으로 다시 날지 않았다 (FF 0.2 → 0.205, 램프 1.45/2.91 → 1.4/2.9 의 작은 변화) |
| 2500 쌍둥이 13/13, 기타 시험 | **airship_fixed 13/13**; 측정값 반영 뒤 variant_check 16/16, gen test 14/14 (기울기 판정: 보관된 측정, 씨앗은 그 3 % 안), forward_yaw_check_test 219/219, generic·실내 `--check` IDENTICAL, `make check_format` 통과; 요 모델 시험은 아래 "깨진 목록" | |
| v6x 빌드 | **통과**, `fw_462c60d3828/px4_fmu-v6x_airship.px4` (메타데이터 검사 통과). 하드웨어 내용은 fw_aabd28bc0c1 과 같다 (바뀐 것은 posix 에어프레임과 모델뿐) | 플래시·지문은 사용자 |
| 커밋 (models → PX4) | models `4b495ae` feat(airship): regenerate the generic family at an AS800-class hull; PX4 `462c60d382` feat(airframes): yaw values for the regenerated generic airship (서브모듈 포인터 + 2500/2520/2522/2527) | 로컬, 서명 없음, 푸시 없음, `Assisted-by: Claude:claude-opus-5-5` |

깨진 목록 (요 모델, 계획 §3 대로 고치지 않음; 재측정 증분에서):
- test_policies: 6 통과 뒤 새 가드에서 멈춤 (엔진 기록 129.3 kg m² 대 트리 87.7) — 의도한 거절.
- test_fin_damping 2 실패: 옛 선체의 엔진 기록을 새 파일과 대조.
- test_controller 1 실패: options/D 의 2527 기록 재현 (옛 선체 기록).
- test_forward 3 실패: 2527 트림 0.0305/0.1221 과 다리 2/4 m/s 고정값 (step 0b 기록의 다리).
- test_forward_replay 22 실패 + TypeError: **이번 재생성과 무관, 이미 깨져 있었다** — 합성 로그의 GRIDS 에 `airship_speed` 가 없는데
  forward_yaw_check.TOPICS 가 D1 증분 1 (오늘 오전) 부터 그 토픽을 읽는다; Cloudship 경우 (c17, c12) 도 같은 이유로 실패.
- D-d (9절, 새로): 이득 규칙이 완전 플랜트만 맞추므로 강체 변형 (2520) 의 Acro 추종이 깨진다. 강체는 실제 기체가 아니지만 upstream 의
  gz 대상이다. 선택: 강체 변형도 자기 플랜트 (I_rigid) 로 이득을 받게 하거나, 맨 기체의 이득을 완전 플랜트 것으로 두고 판정을 받아들이기.

## 7. 커밋

1. 모델 저장소 (airship/models): `feat(airship): regenerate the generic family at the AS800-class hull (34 m³, L/D 3.5)`.
2. PX4 (airship/dev): 서브모듈 포인터 + 에어프레임 + 문서 + (도구는 git 아님) 한 커밋; 메시지에 선정 근거 한 문단 (de Paiva 2006,
   34 m³, two vectored propellers + X tail) — 사업 맥락 없음.

## 8. 확인하고 쓸 전제 (교수 역할의 기억; 틀릴 수 있음)

| # | 전제 | 어떻게 확인 | 틀리면 |
|---|---|---|---|
| P1 | `airship_family_gen.py` 는 기존 에어프레임 멤버의 이득 블록을 유지한다 (README "an existing member keeps its gains") | 생성기의 write_airframes 를 읽고, 생성 뒤 2520/2522/2527 의 diff 를 본다 | 유지하지 않으면 손 값이 지워진 것이니 표식 블록으로 채우면 된다 (같은 결과) |
| P2 | `generic_hull_design.py --spec` 이 질량 수지 (중립 부력에 필요한 곤돌라 질량) 를 보고한다 | 스크립트의 report() 를 읽는다 | 보고하지 않으면: 부력 = 공기 밀도 × 부피 (gz Buoyancy 플러그인의 밀도를 gz_model 에서 읽음), 질량 합 = 선체 + 핀 + 포드 + 곤돌라 로 손 계산해 gondola_mass 를 맞추고 식을 표에 적는다 |
| P3 | JSON 의 `hull.pod_yaw` 54.0 은 포드 요 권한의 설계 목표 (18 N × 1.5 m × 2) 이고 선체와 무관하다 | generic_hull_design 에서 pod_yaw 의 쓰임을 읽는다 | pod_y 가 pod_yaw 에서 도출되면 그대로 두고 (포드 위치 불변), 아니면 그 뜻을 표에 적는다 |
| P4 | 11 m 숫자가 박힌 파일 목록 (3절) 이 전부다 | 같은 grep 을 다시 돌리고, 시험 전체를 돌려 깨진 것으로 확인 | 빠진 곳은 표에 추가 |
| P5 | 요 모델 (yaw_model/) 의 플랜트는 CHAIN_LOGS 의 엔진 측정 블록과 트리의 파일을 섞어 읽으므로, 선체가 바뀌면 재측정 전까지 뜻이 없다 | closed_loop/plants/policies 에서 CHAIN_LOGS 와 gz_model 읽기를 확인 | 섞어 읽는 게 맞으면 d1_check·d1_sitl_compare 를 돌리지 말고, 싸게 되면 "블록의 선체 ≠ 트리의 선체" 를 알리는 가드를 넣는다 (설계 변경 아님) |
| P6 | `sitl_check.py` 의 VEHICLES 표가 generic 의 권한을 숫자로 박고 있다 | 표를 읽는다 | 파일에서 읽는 구조면 할 일 없음 |
| P7 | 포드 18 N × 2 가 AS800 의 최대 13 m/s 에 맞는다 (Hoerner 항력 추정 27 N) | 생성 뒤 `trim_thrust(model, 13)` 이 1 미만인지 | 1 을 넘으면 thrust_n 을 올리지 말고 표에 적는다 (설계 결정, 9절) |
| P8 | 2500 쌍둥이 (airship_fixed) 비행 검사와 "첫 비행 7/7" 검사의 스크립트 이름 | README 와 디렉터리에서 찾는다 | 없으면 있는 검사만 |

## 9. 멈추고 교수 역할에 넘길 것

설계 규칙 변경 (R1 1.2×Munk, 플랩 ±11°, 실속 여유), 파라미터 이름·기본값의 뜻 변경, 포드 추력·위치 변경, 계산기의 물리 (씨앗
식, 램프 식) 변경, 요 모델 구조 변경, 게이트 판정 기준 완화. 이런 것이 필요해 보이면 그 이유와 측정값을 6절 표에 적고 멈춘다.
반대로 숫자 갱신, 깨진 시험의 기대값을 새 파일값으로 바꾸기, 스크립트의 경로·이름 수정, 가드 추가는 실행 역할이 한다.

## 10. 전제 확인 결과 (실행 역할, Opus 5.5, 2026-10-07 19:35~)

| # | 결과 | 한 일 |
|---|---|---|
| P1 | **맞음.** `write_airframes` 는 기존 멤버 파일을 건너뛰고, 맨 기체는 추진 숫자만 제자리 갱신 | 2522/2527 의 손으로 쓴 11 m 튜닝 (FF 0.3/1.0, SLP 4.01, 주석) 을 지우고 계산기 표식 블록으로 대체. 2520 맨 기체에 P/I 블록 (실내와 같은 꼴) |
| P2 | **반만 맞음.** 질량 수지는 자동이다: 전비 = ρ (부피 + 곤돌라 부피), 선체 = 기체 − 곤돌라 − 핀. `gondola_mass` 는 수지가 아니라 나눔 비율 | 곤돌라 36 kg 을 두면 기낭이 3.1 kg (0.050 kg/m²) 로 비현실적. 기존 기낭 면밀도 0.1312 kg/m² 를 유지해 곤돌라 30.9 kg (기낭 8.2, 핀 2.5, 전비 42.63 kg). 설계 규칙 변경 아님 (입력 하나) |
| P3 | **맞음.** `pod_yaw` 54 는 R3 선회 포락선의 입력 모멘트일 뿐, `pod_y` 와 독립 | 그대로 |
| P4 | **틀림 (빠진 것 있음).** 숨은 연결 둘: (a) `generic_hull_design.DEFAULT` 가 generic 의 정의라 JSON 만 바꾸면 `apply_generic_hull.py main()` 이 11 m 를 다시 쓴다; (b) `turn_speeds` 와 `write_pods` 가 "DEFAULT 의 길이" 로 비례 — generic 을 바꾸면 **실내 가족의 선회 속도·포드 관성까지 바뀐다**. 또 생성기 정규식 결함 (드래그 주석 "a quarter high" 만 다시 읽음) 이 새 선체의 "38 % high" 에서 두 번째 실행·실내 생성을 깨뜨림 | DEFAULT 를 JSON 과 같게 (같음을 확인). 비례 기준을 상수 `SCALE_REFERENCE_LENGTH` (10.97 m) 로 고정. generic 포드는 JSON 에 회전반경 0.05/0.1 m 를 명시 (`Pods.gyration_m`, `prop_gyration_m`, 없으면 예전 규칙). 정규식이 두 문구를 다 받게. 결과: 실내 가족 `--check` IDENTICAL (재생성 전후 모두), 실내 이득 불변 (0.778/0.100/15.4/0.39-0.78) |
| P5 | **맞음 (섞어 읽음).** `policies.fin_slope` 가 k 를 엔진 기록 (옛 선체 K_d_fin) 에서, 권한·핀은 트리에서 | 가드 추가: 엔진 기록이 잰 강체 요 관성이 트리와 5 % 넘게 다르면 LookupError (generic: 129.3 대 87.7 로 거절; Cloudship: 5.69 대 5.80, 1.8 % 로 통과). d1_check·d1_sitl_compare 는 돌리지 않음 |
| P6 | **맞음, 할 일 없음.** `sitl_check.VEHICLES` 의 54 N m 는 포드 숫자 (불변) | 그대로 |
| P7 | **틀림.** 18 N × 2 로 최대 속도 **11.8 m/s** (trim(13) = 1.21, trim(10) = 0.72). AS800 의 13~14 m/s 에 못 미침. 옛 11 m 선체도 11.4 m/s 였음 | 포드 추력은 그대로 (9절). 교수 검토 항목 |
| P8 | **반만 맞음.** `airship_fixed_flight_check.py` 있음. "첫 비행 7/7" 은 `cloudship_first_light.py` 로 Cloudship 전용 | generic 은 정지 게이트 `sitl_check.py` 가 대신 |

### 9절에 해당해 멈추고 넘기는 것 (실행은 계속, 결정은 교수)

- **D-a. 모듈 기본값 (AS_YAWRATE_P 0.7, I 0.09) 과 문서 129~131 행.** 기본값은 11 m 선체에서 튜닝한 값인데 그 선체는 트리에 없다. 지금
  gz generic 은 자기 에어프레임에 P 0.421, I 0.0541, FF 0.609 (완전) / 0.2 (핀) 을 갖는다. 문서의 "The defaults were tuned on the
  Gazebo models of the Generic Airship ... 1.0 on the complete vectored airship, 0.3 on its fin variant" 는 이제 사실이 아니다.
  선택: (1) 기본값 유지 + 문서를 "기본값은 약 11 m 선체에 맞고, gz 모델은 같은 대역폭 규칙으로 자기 값을 갖는다" 로, (2) 기본값을 새
  generic 값으로. 비행 에어프레임 2520 (벤치 FMU) 은 기본값을 쓰므로 (2) 는 하드웨어의 요 이득을 바꾼다. 손대지 않았다.
- **D-b. 포드 추력 대 AS800 속도 (P7).** 최대 11.8 m/s. 대표 기체의 "최대 13 m/s" 와 다르다.
- **D-c. 플랩 ±10°.** 규칙 R3 는 그대로이고 그 출력이 11 → 10° 로 바뀌었다. 2500 쌍둥이의 SIM_GZ_SV_MINA/MAXA 를 ±10 으로 맞춤
  (variant_check 15 통과). 비행 에어프레임 2500 (하드웨어) 에는 플랩 각이 없다.
- **관찰 (결정 아님).** `forward_yaw_check.loop_reach` 의 1자유도 계획 추정이 아직 핀의 회전 감쇠 (½ρΣ area·cla·arm²) 를 쓴다; 계산기는
  오늘 (m+m_x)x̄ − Munk 로 바꿨다. 판정이 아니라 계획 예측이라 두었다.

