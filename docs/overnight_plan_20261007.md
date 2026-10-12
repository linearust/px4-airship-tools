# 야간 무인 실행 계획 (2026-10-07 21:20, 교수 → Opus 실행). 사용자 부재 중 끝까지 진행하고 멈춘다.

근거: reference_vehicle_selection_20261007.md §6 (최종 방향), generic_as800_regeneration_plan_20261007.md (오늘 밤의 실행 기록·전제 표,
그 방식 그대로). 규칙: 서브에이전트·워크플로 없음; 게이트는 배경 스크립트 하나로 묶고 요약만 읽기 (오늘 밤의 e8/e9 스크립트 꼴);
커밋은 로컬·서명 없음·`Assisted-by: Claude:claude-opus-5-5`, Co-Authored-By/Claude-Session 없음, 푸시 없음; 트리 안 문서는 docs/en
만; upstream 행이므로 PX4 관례 (사업 맥락 없음, 생성물 헤더에 트리 밖 도구 이름 추가 금지는 아직 — 이미 있는 표식 그대로).
**전제는 코드로 먼저 확인하고 틀리면 코드·측정을 따르고 표에 적는다. 9절 유형은 추측하지 말고 적고 다음 단계로 넘어간다** (멈추지
않는다 — 사용자가 없으므로, 그 단계만 건너뛴다). 진행 기록은 `overnight_20261007_log.md` 에 시각과 함께 (요약만).

## 0. 교수가 확정한 결정 (실행 역할은 바꾸지 않는다)

- **D-a 모듈 기본값**: AS_YAWRATE_P 0.7 / I 0.09 유지 (벤치 지문의 값, 실기체가 튜닝할 자리). 문서 `docs/en/frames_airship/index.md`
  129~131 행을 사실로 고친다: "The defaults were tuned in Gazebo on an 11 m hull (34 m³ class) the tree no longer carries; each
  Gazebo airframe sets the values the same loop-bandwidth rule gives its own hull (P, I), its hull drag (AS_YAWRATE_FF) and its
  held-turn slope (AS_YR_FF_SLP)." 피드포워드 값 나열 (1.0/0.3/0.05) 은 지운다. 165~171 행의 스케줄 예시 숫자 (4.01, 1.1°/s, 1.3 m/s
  1.6°/s, 1.5 m/s 1.4°/s, 4 m/s 2.9°/s) 는 7 m generic 의 새 값으로 다시 계산해 바꾼다 (식: ρ = 0.5·min(AS_YR_INT_LIM, 0.5) = 0.15;
  제자리 상한 = ρ/(k·u_hi) rad/s 로 u_hi = AS_SPD_WMAX 2 와 피토 하한 1.3; 램프 아래 u 에서 ρ/(k·u); 램프 지난 u 에서 (1−ρ)/(FF + k·u);
  예시 속도는 새 LO 와 1.6·HI). 모든 숫자는 새 에어프레임의 값에서 계산하고 로그에 식과 함께 적는다.
- **D-b 포드 추력**: generic 7 m 10 N (ASSUMED), AURORA 25 N (13 m/s 의 항력 43.6 N 에 여유), JAXA 80 N (14 m/s 의 115 N + 상승 여유).
- **D-c 플랩**: 규칙 R3 의 출력 그대로. 2500 쌍둥이의 SIM_GZ_SV_MINA/MAXA 를 **생성기가** 플랩 행정에서 쓰도록 고친다 (오늘은 손으로 ±10).
- **D-d 강체 멤버 이득**: 규칙 유지 (맨 에어프레임 = 완전 플랜트의 P/I). 강체 정지 판정 8/10 ("Acro 제동", "Acro 추종") 은 감쇠가 전혀 없는
  비물리 플랜트의 알려진 기준선으로 기록한다 (Cloudship 9/10 과 같은 취급). 코드 변경 없음.
- **이름·십 자리**: generic 그대로 (`airship_vectored_independent` 2520 / `airship_fixed` 2500); **2540 `airship_aurora`** "AURORA Airship
  (AS800 class)"; **2550 `airship_jaxa`** "JAXA 14 m LTA Test Vehicle"; **2560 `airship_mpi`** "MPI 5 m Outdoor Blimp"; 2570 은 Lotte 예약
  (이번엔 정적 검사만). 기관명이 upstream 에서 문제되면 이름은 JSON 한 줄이다 — 지금은 이대로.
- **upstream 구성** (PR 때): PX4 에 generic + Cloudship + 실내; 문헌 가족은 모델 저장소 우선. 지금은 모두 트리에 생성한다.
- **스케줄과 조종면**: 조종면이 있는 가족은 배분기가 yaw_surfaces 를 올려 스케줄이 꺼진다 (E1 설계). 그런 가족의 전진 검사는 "스케줄 꺼짐" 을
  전제로 읽는다 (forward_yaw_check 가 그 경우를 어떻게 다루는지 전제 P14 에서 확인).

## 1. 기체 사양 (JSON 값; ASSUMED 는 notes 에 그렇게 적는다)

### 1.1 generic 7 m — `vehicles/airship_generic.json` (이름·제목·십 자리 불변)
hull: hull_length 7.0, hull_diameter 2.0 (→ 14.66 m³, f 3.5; from_dict 가 받음), hull_z 1.30, gondola [0.5, 0.3, 0.3], gondola_z 0.15,
gondola_mass **규칙으로** (기낭 0.1312 kg/m² 유지: 설계 스크립트로 기낭 4.68 + 핀 1.36 을 빼면 ≈ 10.97; 핀 질량은 최종 설계에서 다시 읽어
맞춘다), pods_mass 0.6, pod_y 0.5, pod_z 0.25, fin_areal 0.35714285714285715, fin_thick 0.02, flap_thick 0.025, chord 0.75, te_margin 0.19,
ground_margin 0.05, flap_fraction 0.30, stability_target 1.2, stall_margin_deg 2.0, pod_yaw 10.0.
pods: link_mass 0.25, prop_mass 0.05, thrust_n 10.0, max_rot_velocity 300.0, tilt_deg 180.0, tilt_rate_deg_s 120.0 (회전반경 필드 없음 →
길이 비례 기본값 0.032/0.064).
notes: "Aero Drum Ltd 7 m outdoor RC blimp class (7 x 2 m, 14 m^3, polyurethane 100 um, lift capacity 2-2.5 kg; the vendor's vehicle
has one collective tilt axis, four fixed foam fins and a lower-fin yaw motor). Actuation is this project's: two independently tilting
pods, four fins. ASSUMED: pod thrust 10 N, pod masses and station, gondola box; the envelope keeps the earlier hull's 0.1312 kg/m^2."
설계 스크립트 `generic_hull_design.DEFAULT` 를 JSON 과 같게 (hull_volume 은 계산값 그대로, 소수 넷째 자리), `SCALE_REFERENCE_LENGTH`
10.97 은 그대로. 설계 보고 기대값: 핀 1.46/0.70 m, 플랩 ±8°, R1 ok (교수가 21:15 에 같은 스크립트로 확인; 전비 18.0 kg).

### 1.2 AURORA — `vehicles/airship_aurora.json` (오늘 밤 generic 에 넣었던 AS800 값을 옮긴다)
name airship_aurora, title "AURORA Airship (AS800 class)", body_title "AURORA airship", decade 2540.
hull: 오늘 밤 JSON 그대로 (hull_volume 34.0, fineness 3.5, hull_z 2.05, gondola [1.25,0.8,0.8], gondola_z 0.4, gondola_mass 30.9, pods_mass
1.0, pod_y 1.5, pod_z 0.8125, chord 1.2, te_margin 0.30, fin_thick 0.03, flap_thick 0.04, …), **pod_yaw 75.0**.
pods: link_mass 0.4, prop_mass 0.1, **thrust_n 25.0**, max_rot_velocity 300, tilt 180, rate 120, gyration_m 0.05, prop_gyration_m 0.1.
notes: de Paiva et al. 2006 (J. Field Robotics 23(3/4), DOI 10.1002/rob.20111): 10.5 x 3.0 m, 34 m^3, payload ~10 kg, max ~14 m/s,
two vectored propellers (-30..+120 deg, one group), X tail (+-30 deg), stern thruster; here the spheroid of the same volume and
fineness (9.27 x 2.65 m), independent +-180 deg pods, '+' fins by the stability rule (no fin data published), no surfaces until the
surfaces option (6단계). ASSUMED: thrust 25 N per pod, masses.
오늘 밤의 `vehicles/airship_vectored_independent.measured.json` 은 **지운다** (AS800 선체·N 54 의 것; AURORA 는 N 75 로 다시 잰다).

### 1.3 JAXA — `vehicles/airship_jaxa.json`
name airship_jaxa, title "JAXA 14 m LTA Test Vehicle", body_title "JAXA 14 m airship", decade 2550.
hull: hull_length 14.0, hull_diameter 4.0 (→ 117.3 m³ = 논문 117 ✓), hull_z 3.0, gondola [2.0, 1.0, 1.0], gondola_z 0.5, gondola_mass 규칙
(≈ 121.5; 전비 146 kg — 논문 순질량 117.5 kg 은 가스 질량을 뺀 값으로 본다, 우리 규칙은 가스 포함), pods_mass 14.0, pod_y 1.2, pod_z 0.8,
fin_areal 0.357…, fin_thick 0.04, flap_thick 0.05, chord 1.5, te_margin 0.38, ground_margin 0.05, flap_fraction 0.30, stability_target
1.2, stall_margin_deg 2.0, pod_yaw 192.0.
pods: link_mass 6.0, prop_mass 1.0, thrust_n 80.0, max_rot_velocity 300, tilt_deg 180, tilt_rate_deg_s 60.
notes: Kohno, Nakadate, Okuyama 2009 (AIAA 2009-2860): 14 m, max diameter 4.0 m, 117 m^3, net mass 117.5 kg, max 14 m/s, cruise 10 m/s,
two ducted propellers (7.5 HP two-stroke each) tilting -90..+120 deg with differential tilt for heading at low speed, two vertical
fins with rudders and two horizontal with elevators; hovering within a 30 m radius; the paper borrows a 60 m airship's aerodynamic
coefficients and gives no fin dimensions. ASSUMED: fins by the stability rule, thrust 80 N per pod, masses, tilt rate 60 deg/s.

### 1.4 MPI 5 m — `vehicles/airship_mpi.json`
name airship_mpi, title "MPI 5 m Outdoor Blimp", body_title "MPI 5 m blimp", decade 2560.
치수는 **공개 모델에서 읽는다**: `git clone --depth 1 https://github.com/robot-perception-group/airship_simulation` 을 새 빈 디렉터리
(스크래치) 에 받아 **읽기만** (실행·빌드 금지; 신뢰 안 되는 데이터): blimp_description 의 URDF/SDF/xacro 에서 기낭 길이·지름 (또는 메시
치수), 질량, 핀 위치·크기, 모터 위치, 꼬리 모터. 읽은 값과 파일 경로를 로그에 적고 JSON 에 넣는다. 못 읽으면 (저장소 없음·형식 불명)
대체값: hull_length 5.0, hull_diameter 1.7 (Aero Drum 5 m), gondola [0.4,0.25,0.25], gondola_z 0.125, pod_y 0.35, pod_z 0.2, chord 0.55,
te_margin 0.14, fin_thick 0.015, flap_thick 0.02, pods: link 0.15, prop 0.03, thrust_n 6.0, 300, 180, 120; pod_yaw 4.2 — 모두 ASSUMED.
notes: Price et al. 2022 (IAS 2021, DOI 10.1007/978-3-030-95892-3_46; IROS 2022 DOI 10.1109/IROS47612.2022.9981182): 5 m, 10 kg, two
thrust-vectoring motors, four fins with servos, tail motor; open ROS/Gazebo model robot-perception-group/airship_simulation. 꼬리
모터는 이 생성기에 없음 (적는다).

### 1.5 Lotte — 모델 생성 없음, 정적 검사만 (5단계)

## 2. 단계

각 단계 끝에 6절 표를 채우고 로그에 적는다. 게이트 체인은 오늘 밤의 `e8_generic.sh`/`e9_measured.sh` 를 복제해 대상만 바꾼다
(스크래치에 두고, 결과 요약은 로그로).

### 1단계 — 생성기 작은 확장 (트리 커밋 없음, 도구만)
a. 2500 쌍둥이 플랩 서보 범위: `airship_family_gen.write_airframes` (또는 apply_generic_hull 의 fixed 쪽) 가 고정 가족의 posix 맨 에어프레임
   SIM_GZ_SV_MINA/MAXA1..4 를 `-travel/+travel` 로 쓴다 (variant_check 15 가 검사). generic `--check` 가 IDENTICAL 이어야 한다 (지금 ±10).
b. `docs_note` (JSON 선택 필드): 가족의 vehicles.md 첫 절 (맨 멤버) 끝에 한 문장을 쓴다 — 문헌 가족의 출처 인용. 없으면 아무것도 안 씀.
   generic·실내 `--check` IDENTICAL 유지.
c. `sitl_check.VEHICLES` 를 표 대신 JSON 에서 읽게 하거나 (vehicles/*.json 의 pods.thrust_n × hull.pod_y × 2), 못 하면 표에 세 가족을 더한다.
d. 시험: variant_check, airship_family_gen_test, forward_yaw_check_test 모두 초록.

### 2단계 — generic 7 m
1. JSON (1.1) + DEFAULT 동기화 + `generic_hull_design.py --spec vehicles/airship_generic.json` (all requirements met; 질량 수지 → gondola_mass).
2. `rm vehicles/airship_vectored_independent.measured.json`; `airship_family_gen.py vehicles/airship_generic.json`; `airship_gains.py
   airship_vectored_independent --write`; variant_check, gen test, forward test; generic·실내 `--check` IDENTICAL.
3. 게이트 체인 A: sitl build → hover 2520 (기준선 8/10 허용: 실패가 "Acro 제동"/"Acro 추종" 뿐일 것) → hover 2527 (10/10) → forward 2522,
   2527 (42/42; 핀 플랜트는 다리 3 감속 불가 허용) → fixed 2500 (13/13).
4. `--measure <2527 요약> --write` → 체인 B: build → forward 2527 (42/42, k 재현 3 % 안).
5. 문서: frames_airship/index.md D-a 편집 (0절 식으로 계산); README "Parametric families" 의 숫자 (34 m³ → 7 m) 갱신.
6. `make check_format`; 커밋: models `feat(airship): move the generic family to a 7 m outdoor hull` (본문: Aero Drum 7 m 급 근거, 치수,
   ASSUMED), PX4 `feat(airframes): yaw values for the 7 m generic airship` (서브모듈 + 2500/2520/2522/2527 + docs).

### 3단계 — AURORA 2540, JAXA 2550
각 가족: JSON → generate → gains --write (씨앗) → 시험 셋 → 체인 (hover 맨/완전, forward _fin/완전, measure, write, forward 완전 재실행)
→ 문서 (생성기가 씀 + docs_note) → 커밋 둘 (models `feat(airship): add the AURORA (AS800-class) family`, PX4 `feat(airframes): gz
airframes for the AURORA family`; JAXA 같은 꼴). 판정 기대: 완전 hover 10/10, 강체 8/10 이상, forward 완전 42/42. JAXA 는 느리고 크다
— 다리 속도는 램프에서 자동 (아마 4~8 m/s), 가속 시간이 길면 forward_yaw_check 의 ACCEL_MAX_S 한도에 걸릴 수 있다 (P15).

### 4단계 — MPI 5 m 2560
1.4 대로. 공개 모델에서 읽은 값과 대체값 중 무엇을 썼는지 notes 와 로그에. 같은 체인·커밋.

### 5단계 — Lotte 정적 검사 (도구만, 30 분)
`lotte_added_mass_check.py`: HullSpec(hull_volume 107, fineness 4.0) 의 Lamb 부가질량 (k1 m', k2 m', k' I'; hull_basis.lamb) 을 Li & Nahon
2007 표 1 의 맨 선체 CFD (m11 13, m22 112, m55 759 kg m²) 와 비교해 비율을 인쇄 (판정 없음, 보고만; 핀 포함 값 12.7/129/1379 는 참고).
README 한 줄. 결과를 표에.

### 6단계 — 벡터링 가족의 조종면 옵션 (시간 상자 4 시간; 안 되면 상태만 적고 끝)
JSON `"surfaces": true` → 그 가족의 모든 _fin 멤버의 핀에 플랩 (고정 가족의 플랩 쓰기 코드 재사용: 힌지 조인트, flap_fraction, R3 행정),
gz 서보 채널은 틸트 2 뒤 (servo_2..5 → SIM_GZ_SV_FUNC3..6 = 201..204, MINA/MAXA ±행정), posix 맨 에어프레임에 2500 의 CA_SV_CS_COUNT 4 +
CA_SV_CS*_TYPE/TRQ 블록 (2500_gz_airship_fixed 에서 복사해 유형만 확인). 스킴 토큰 추가 없음 (핀 = 플랩 핀). 적용: AURORA, JAXA, MPI 에
surfaces true 로 재생성 → 시험 셋 + hover 완전 10/10 + forward 완전 (스케줄 꺼짐 전제, P14) + 러더·엘리베이터 응답 검사 (`airship_fixed_
flight_check.py` 가 틸트 포드 0° 의 벡터링 가족에도 도는지 P16; 안 돌면 그 검사는 건너뛰고 적는다) → 커밋 (가족마다 둘). generic·실내는
surfaces 없음 (불변, --check IDENTICAL).

### 7단계 — 마무리
`build_fmu.py` (v6x) + `make micoair_h743-aio_default`; README·메모리 (`~/.claude/.../memory/vehicle-roadmap-20261007.md` 에 결과 요약 한
항목, MEMORY.md 한 줄); 이 파일 6절 표 완성; `git log --oneline` 두 저장소; **멈춤**. 푸시·서명 없음.

## 3. 확인하고 쓸 전제 (교수의 기억; 틀리면 코드를 따르고 표에)
| # | 전제 | 확인 | 틀리면 |
|---|---|---|---|
| P9 | `HullSpec.from_dict` 가 hull_length/hull_diameter 를 받아 부피·fineness 로 바꾼다 | 코드 | hull_volume/fineness 로 직접 |
| P10 | 생성기가 임의 이름·십 자리의 가족을 만든다 (실내가 그 증거) 고 docs 절·CMake 십 자리 줄을 더한다 | 실내 생성 경로 | 이름 규칙이 걸리면 적고 가족 이름을 규칙에 맞춤 |
| P11 | `airship_gains.plant(vehicle)` 가 `<vehicle>_am_fin_drag`/`_fin` 만 쓰므로 새 가족에 그대로 됨 | 코드 | — |
| P12 | forward_yaw_check 의 ALLOWED 가 `_fin` 토큰에서 자동, 다리 속도는 램프에서 자동 | 코드 | — |
| P13 | sitl_check 의 VEHICLES 표만 가족별 손 입력 | 코드 | 1단계 c |
| P14 | 조종면 가족에서 배분기가 yaw_surfaces 를 올리고 스케줄이 꺼진다; forward_yaw_check 가 "스케줄 꺼짐" 다리를 판정 없이 기록한다 | 코드 (airship_att_control applyAllocatorStatus, forward_yaw_check 의 schedule 판정) | 판정이 스케줄 켬을 전제하면 6단계 전진 검사는 건너뛰고 적는다 |
| P15 | forward_yaw_check 의 가속 한도 (ACCEL_MAX_S, ACCEL_THROTTLE_MAX 0.5) 가 JAXA (146 kg, 80 N ×2) 의 다리 속도에 닿는다 | `--plan` 으로 먼저 본다 | 닿지 않으면 다리 속도를 FORWARD_LEGS 로 낮춰 날고 적는다 (램프 중앙·1.6 HI 가 아닌 것을 표시) |
| P16 | airship_fixed_flight_check 가 틸트 포드 가족에도 (포드 0°) 돈다 | 스크립트의 전제 (추력 방향, 서보 수) | 안 돌면 건너뜀 |
| P17 | 2540~2560 십 자리와 에어프레임 번호가 비어 있다 | CMake 표·디렉터리 | 비어 있는 다음 십 자리 |
| P18 | 공개 모델 저장소 airship_simulation 이 공개이고 blimp_description 에 치수가 있다 | clone (읽기만) | 1.4 대체값 |

## 4. 9절 유형 (건너뛰고 적는다): 설계 규칙 (R1/R3/실속 여유), 파라미터 이름·뜻, 스킴 토큰, 배분기·컨트롤러 C++ 코드, 요 모델 구조,
게이트 판정 기준 완화, 포드 추력·위치의 변경 (1절 값은 확정값이므로 바꾸지 않음).

## 5. 멈추는 조건: 7단계 끝; 또는 한 단계가 두 번 재실행해도 게이트를 못 넘으면 그 단계를 적고 다음 단계로; 6단계는 4 시간 상자;
전체는 사용자가 돌아올 때까지의 시간 (약 12 시간) — 시간이 모자라면 7단계 (마무리) 는 반드시 한다.

## 6. 결과 표 (실행 역할이 채움; Opus 5.5, 2026-10-07 21:21 ~ 10-08 00:20; 진행 기록 overnight_20261007_log.md)
| 단계 | 결과 | 비고 |
|---|---|---|
| 1 생성기 확장 (a, b, c, d) | **완료.** (a) 고정 쌍둥이 플랩 서보 범위를 생성기가 씀 + 변이 판정; (b) `docs_note`; (c) sitl_check 기체 표를 vehicles/*.json 에서, 밀기 기준 54 N m 상수 (generic 이 바뀌어도 판정 불변); (d) 시험 초록 | 추가로 고친 생성기 결함 셋: 고정 쌍둥이 추진기를 포드에서 씀 (옛 18 N/±1.5 m/0.398 kg 이 남아 중립 부력이 0.4 kg 어긋날 뻔), 플랩 시각 상자 소수 셋째 자리 (7 m 의 0.225 m 시위가 0.22 로 읽혀 variant 시위비 0.29), 핀 주석의 유도항력 기준각 둘 허용 (MPI 의 배 핀) |
| 2 generic 7 m: 설계·생성·시험 | 7.0 × 2.0 m, 14.66 m³, 18.0 kg, 곤돌라 11.37 (규칙), 핀 1.46/0.70, 플랩 ±8°, R1 1.206; DEFAULT == JSON; --check IDENTICAL | 계획의 곤돌라 ≈ 10.97 은 포드 1.0 kg 가정; 0.6 kg 포드로 11.37 |
| 2 generic 7 m: 게이트 A/B, 측정 k | hover 2520 **10/10**, 2527 **10/10**; forward 2522 30/31 (다리 3 감속 불가, 허용), 2527 **42/42**; 고정 2500 **13/13**; 측정 k 51.8 vs 씨앗 50.6 (+2.5 %), B 42/42, 재현 −0.1 % | P 0.628 I 0.0808 FF 0.81/0.367 SLP 5.18 램프 1.0/2.1 |
| 2 generic 7 m: 문서·커밋 | D-a: 문서 129~131 행 재작성 + 예시 숫자 (0.8, 1.3, 1.7@1.0, 2.7@3.35; 계산기가 옛 숫자 1.1/1.6/1.4/2.9 를 먼저 재현); models **71842c5**, PX4 **8e368e999f** | 계획 문구의 "34 m³ class" 는 사실과 달라 "39 m³" 로 (기본값을 튜닝한 선체는 10.97 m, 39.2 m³) |
| 3 AURORA 2540 | hover 2540 8/10 (D-d 기준선), 2547 10/10; forward 2542/2547 **42/42**; k 155.9 vs 152.4 (+2.3 %; N 54 일 때도 156.0), B 42/42 재현 +0.3 %; models **89a8577**, PX4 **988e6c5102** | P 0.303 I 0.0389 FF 0.439/0.147 SLP 2.08 램프 1.7/3.4 |
| 3 JAXA 2550 | hover 2550 10/10, **2557 7/10** (9절 D-f); forward 2552 30/31 (다리 3, 허용), 2557 **42/42**; k 832.7 vs 814.8 (+2.2 %), B 42/42 재현 −0.2 %; models **22dc63b**, PX4 **dffe1e9bf9**; variant_check 15/16 (9절 D-e) | P 0.789 I 0.1015 FF 1.35/0.307 SLP 4.34 램프 1.5/3.1; 틸트 60°/s |
| 4 MPI 2560 | 공개 모델에서 치수 (5.0 × 2.0 m, 10.47 m³, 모터 ±0.375 m, 핀 시위 0.85 m); hover 10/10 ×2, forward **42/42** ×2; k 21.32 vs 20.69 (+3.0 %), B 42/42 재현 −0.1 %; models **81e8937**, PX4 **183bd03268** | P 0.511 I 0.0656 FF 0.481/0.336 SLP 4.74 램프 0.93/1.87; 원 모델의 모터 상수 (2.9 kN) 는 쓰지 않음 |
| 5 Lotte 정적 | `lotte_added_mass_check.py`: 107 m³ f 4 회전타원체의 Lamb 값 / Li & Nahon 2007 표 1 CFD = m11 0.822, m22 **1.006**, m55 **1.229** (L&N 자신의 추정 0.846/1.018/1.045) | 옆 부가질량은 맞고 요·피치 부가 관성은 회전타원체 근사에서 23 % 크다 — 교수 검토 |
| 6 조종면 옵션 | **코드·시험 완료, 적용 건너뜀 (9절 D-g).** Family.surfaces, inject_flaps (servo_2..5), 핀 멤버 조종면 블록 (CS 4, 틸트 Servo5/6), variant TWINS; 생성기 시험의 조종면 판정 둘 | CS_K 기본 1 이면 요 루프가 닫히지 않음 (문서 92행) — 계획의 기대와 모순; CS_K 값과 적용 여부는 교수 결정 |
| 7 펌웨어·문서·메모리 | `fw_183bd032683/px4_fmu-v6x_airship.px4` (메타데이터 통과), `micoair_h743-aio_default.px4` 1.8 MB 빌드 OK; 최종 시험 variant 15/16, gen 15/17 (셋 모두 D-e 하나에서), forward 219/219, 다섯 가족 --check IDENTICAL, check_format 통과; README·메모리 갱신 | 플래시·지문은 사용자 |
| 틀렸던 전제 | P14 부분 (도구의 schedule_on 은 파라미터만 봄; servo_offset 은 이미 지원), P15 틀림 (가속 한도 문제 없음), P16 틀림 (airship_fixed_flight_check 는 2500 전용), P18 부분 (모델은 있었으나 추력 상수는 못 씀; 치수는 대체값 5 × 1.7 이 아니라 5 × 2.0) | P9~P13, P17 맞음 |
| 건너뛴 것 (9절) | **D-e** JAXA 틸트 속도 60 ≠ 2520 의 120 → variant 쌍둥이 판정 실패 (60 을 120 으로? 아니면 기체별 값 TLMIN/TLMAX/TLT_R 을 비교에서 빼기?). **D-f** JAXA 완전 hover 7/10: 큰 선체 항력 (|nRabsR|/N 5.2) 에서 FF 규칙 (AS_YAWRATE_MAX 에서 2차 항력에 맞춤) 이 6.67°/s 에서 약 2배 과함 → Acro 8.2°/s, 놓음 정지 10.1 s, Stabilized 3.6/6.7, I 끔 표류 0.75°/s. **D-g** 조종면 옵션의 CS_K 와 적용. 관찰: JAXA 의 아래 핀이 'dorsal' 로 이름 붙음 (무게중심 기준 규칙, 판정 무관); Lotte 의 요 부가 관성 1.23 배 | 각 항목의 선택지는 진행 기록에 |
