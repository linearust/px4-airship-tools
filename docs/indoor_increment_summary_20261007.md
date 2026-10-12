# 실내 검증 기체 증분: 한 번에 읽는 정리 (2026-10-07, 작성 중)

사용자가 정한 전제: 꼼수 없이 가장 좋은 설계, 수정이 커도 됨, 결정은 교수 역할이 스스로, 중간 검토 없이 끝에 한 번,
검증 기체 (2 m급 실내, 독립 틸트 포드 2개, 고정 핀 가능, 러더·꼬리 없음, 광류) 로 검증할 수 있는 범위에 집중,
배분기는 하나. 결정과 계획은 `indoor_vehicle_plan_20261007.md`, 그 앞의 D1 1단계는 `d1_increment1_report_20261007.md`.

## 읽는 순서

1. 이 문서의 2~4절.
2. `indoor_vehicle_plan_20261007.md` 1절: 결정 D1~D8 과 근거.
3. 커밋 (아래 5절의 표 순서로). 각 커밋의 시험이 의도를 말한다.
4. `docs/en/frames_airship/index.md` 의 Yaw Rate Controller 와 Forward Flight 절: 사용자 관점의 설명.
5. `vehicles/airship_indoor.json` 과 `README.md` 의 "Parametric families" 절: 기체 치수를 바꾸는 법.
6. `indoor_hardware_package_20261007.md`: 기체가 나왔을 때 따라 할 절차.
7. `upstream_pr_plan_20261007.md`: upstream 에 내는 순서와 예상 질문.

## 1. 한 줄 결과

펌웨어 두 커밋 (배분기 → 컨트롤러 메시지, 속도원 조건) 은 출력 비트 동일 + 정지 48/48 + 전진 42/42 ×3 + 모델 대조
9/9. 실내 가족은 파일 하나에서 생성되고 (멱등, 검사 16/16, 생성기 시험 13/13), 정지 판정 10/10 ×2, 전진 완전 플랜트
42/42 (기울기 측정 두 번 0.578/0.577, 씨앗 0.566), 핀 플랜트 30/31 (감속 못 하는 다리 하나). 펌웨어 두 보드 빌드 OK
(fw_aabd28bc0c1 v6x, micoair_h743-aio). 벤치 지문은 사용자 플래시 뒤.

## 2. 무엇이 바뀌었나

- **펌웨어, 배분기 → 컨트롤러 (E1, 55445b6711).** 배분기가 `airship_allocator_status` 로 요 축의 뜻 (추진 요 유무, 조종면
  유무, 방향별 요 권한) 을 게시하고, 컨트롤러는 배분기 파라미터를 읽지 않는다. 출력은 비트 단위로 같다.
- **펌웨어, 속도원 (E2).** 지면 속도원이 heading_good_for_control 을 요구하지 않는다: EKF 의 속도와 헤딩은 한 좌표계라
  기체 x 속도는 절대 헤딩 오차와 무관 (직진에서 2차항). 실내 자력계에서도 스케줄이 켜진다.
- **도구, 파라미터 사슬 (E3, E4).** 기체 파일 하나 → gz 가족 8개, 월드 항목, posix 에어프레임, CMake, 문서
  (`airship_family_gen.py`); 같은 파일 → 요 이득, 피드포워드, 스케줄 기울기, 램프, FMU 파라미터 세트 (`airship_gains.py`).
  generic 가족은 제 파일에서 다시 만들면 트리와 같다. 검사 도구들은 표 대신 파일을 읽는다.
- **기울기의 물리 (E4, e0a41ec8fe).** 전진 선회를 유지하는 토크는 핀의 회전 감쇠가 아니라, 선체가 옆미끄러져 핀이
  구심력을 질 때 생기는 (m+m_x)·u·r × 핀 팔 길이에서 그 옆미끄럼의 Munk 모멘트를 뺀 것이다 (핀 양력기울기와 무관).
  시뮬레이터 유지 선회가 두 기체 네 플랜트에서 이 씨앗의 3 % 안; 회전 감쇠는 +9 %/−21 % 였다. 계산기가 전진 요약에서
  기울기를 재어 보관하고 (`--measure`), 램프는 옆미끄럼을 포함한 핀 받음각이 실속에 닿기 전 (0.8 배) 에 끝난다.
  하드웨어도 같은 측정이다 (H4). Cloudship 의 씨앗이 음수인 것은 알려진 2517 선회 불안정의 설명이다.
- **실내 기체 가족 `airship_indoor` (2530~2537).** 가정한 치수로 만든 첫 판. 치수가 정해지면 JSON 을 고치고 두 명령을
  다시 돌린다.
- **하드웨어 꾸러미 (E5).** 벤치 절차, 비행 카드 6장, EKF 만으로 도는 분석 모드 (`forward_yaw_check.py --analyze
  --hardware`).

## 3. 결정 중 사용자가 다시 볼 만한 것

- D2 의 전제: 광류·UWB 로 EKF 가 수평 속도를 내는지는 하드웨어에서 확인해야 한다 (비행 카드 H1).
- 핀 크기 제안: 시위 0.22 m 에 등·옆 핀 0.62 m, 배 핀 0.22 m. 안정 규칙 (Munk 의 1.2배) 이 요구하는 크기라 크다. 시위를
  늘려도 폭은 거의 줄지 않는다 (0.35 m 시위에 0.58 m). 핀을 안 달면 전진비행이 방향 불안정이고, 스케줄은 검증할 것이
  없다.
- 포드가 무게중심 0.3 m 아래: 최대 추력에서 기수가 약 10 deg 든다 (진자 0.9 N m/rad). 곤돌라 질량이 커지면 줄어든다.
- 기울기 15.4 per m/s (generic 4.2): 상한이 램프 끝 0.78 m/s 에서 3.9 deg/s, 1 m/s 에서 3.0. 선회 반경 10~30 m 로 홀보다
  클 수 있다 (하드웨어 꾸러미 2절). 상한에서 포드는 180°/0° 로 서로 밀며 추력의 85 % 를 요 커플에 쓴다. 실내 선회는
  느릴 것이고 그것이 이 기체다.

## 4. 남은 위험

- **치수가 가정값.** 기낭 2.0 × 0.8 m, 곤돌라 0.2 kg, 포드 0.25 N 은 ASSUMED. 제작 치수로 JSON 을 고치면 가족·이득·램프가
  모두 바뀐다 (6절). 기준 (대표) 기체는 7 m (18:56 결정): 치수가 오면 generic 가족을 7 m 로 다시 만든다 (큰 작업). 그때까지
  계산기의 기준은 49 kg 11 m 플랜트의 측정값.
- **핀 없는 10월 기체.** 전진비행은 Munk 모멘트로 방향 불안정 (H3). 요 루프와 속도원 (H1, H2) 만 검증 가능; 스케줄은 고정
  핀 뒤.
- **added_mass_check 의 핀 두 항이 실내 선체에서 27~31 % 낮게 읽힌다** (generic 9/9). 유지 선회의 핀 받음각은 파일의 1.00~1.07
  배라 파일이 아니라 그 검사의 회귀 영역 (0.1 s 에 풍향계가 서는 가벼운 선체) 문제로 본다. 미해결, 스케줄과 무관.
- **드래그 없는 플랜트의 다리 순서.** _fin 플랜트는 감속할 수 없어 세 번째 다리 (같은 속도, 반 스틱) 가 두 번째보다 느릴 수
  없다; 완전 플랜트에서는 난다.
- **작은 프롭의 회전자 상수** (rotorDrag/moment) 는 x500 값 복사. 틸트 포드의 요 커플에는 영향 없음, 피치 트림에는 작게.
- **SITL EKF 헤딩이 groundtruth 와 5~15° 다르다.** D2 가 맞다면 (속도와 헤딩이 한 좌표계) 스케줄에는 무관; 원인 미조사.
- **실내 선회 반경.** 상한에서 10~30 m. 홀이 작으면 다리를 짧게 날고 허용값을 비례해 줄인다 (하드웨어 꾸러미).

## 5. 커밋 (로컬, 서명 없음, 푸시 없음)

airship/dev (d51ed755cb 위):

| 커밋 | 내용 |
|---|---|
| 55445b6711 | feat(control_allocator): report the airship yaw allocation to the controller (E1, D3) |
| 0c38526b99 | fix(airship_att_control): take the ground speed without a heading good for control (E2, D2) |
| b599bcf796 | feat(airframes): gz airframes for the indoor airship family (E3) |
| e49b0fbbd9 | build(simulation): bump gz models for the indoor pods' tilt damping (E3) |
| e0a41ec8fe | feat(airframes): measured yaw slope and stall ramp for the indoor family (E4) |
| aabd28bc0c | docs(airship): estimate the yaw slope from mass and fin arm (docs/en 한 문장) |

airship/models (208cb2b 위): b2ddfa4 feat(airship): generate the indoor airship family from a vehicle file; e780819
fix(airship): scale the indoor pods' tilt damping with their inertia.

벤치 도구 (git 아님): airship_family_gen.py, airship_gains.py, generic_hull_design.py, apply_generic_hull.py,
vehicles/*.json + airship_indoor.params + airship_indoor.measured.json, variant_check.py, sitl_check.py,
forward_yaw_check.py (+ --hardware, 램프 기반 다리 속도), 시험 두 벌, 문서 다섯.

## 6. 치수를 바꿔 다시 만드는 명령

```sh
cd ~/dev/px4-airship-bench-tools
$EDITOR vehicles/airship_indoor.json
python3 airship_family_gen.py vehicles/airship_indoor.json          # 모델, 월드, 에어프레임, CMake, 문서
python3 airship_gains.py airship_indoor --write --params vehicles/airship_indoor.params
python3 variant_check.py && python3 airship_family_gen_test.py
```
