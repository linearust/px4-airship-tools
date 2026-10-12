# 실내 검증 기체: 벤치와 비행 절차 (초안 2026-10-07)

기체: 2 m급 실내 비행선, 독립 틸트 포드 2개 (RS1102 10000KV, Gemfan 1610 40 mm, 2S), KST X06 서보, MicoAir 743 AIO
FC, MTF-02P 광류, LinkTrack P-AS UWB, LR24-F-Mini 수신기. 고정 핀 가능. 하드웨어 에어프레임은 2520 (Generic Airship,
Independent Vectoring) 에 이 기체의 값을 얹은 것이다. 시뮬레이터 쌍둥이는 `airship_indoor` 가족 (2530 대).

기체의 치수가 바뀌면 `vehicles/airship_indoor.json` 을 고치고 `airship_family_gen.py` 와 `airship_gains.py --write
--params` 를 다시 돌린다. 아래 값은 그 출력이다.

## 1. 벤치, 프롭 제거 ESC 전원 차단 상태에서

1. **펌웨어와 에어프레임.** 검증 기체의 FC 는 MicoAir 743v2 AIO: 보드 설정 `boards/micoair/h743-aio/default.px4board`
   (와 `h743-v2`) 가 airship_att_control 을 이미 켠다. `make micoair_h743-aio_default` 로 빌드해 올리고 (받은 보드의
   정확한 이름을 먼저 확인), SYS_AUTOSTART 2520. fmu-v6x 는 벤치용이다. `python3 snapshot_params.py` 로 기준 스냅샷.
2. **파라미터 세트.** `python3 apply_params.py $(grep -v '^#' vehicles/airship_indoor.params | tr '\n' ' ')`. CA_* 는 2520 의
   것 (GRP 1, TLMIN/TLMAX ±180, TLT_R 120) 이 기체와 같은지 CAD 의 멈춤쇠로 확인; 다르면 CA_AIRSHIP_TLMIN/TLMAX 를
   맞추고 `vehicles/airship_indoor.json` 의 tilt_deg 도 맞춘다.
3. **틸트 범위 보정.** `tilt_range_check.py` 의 방식: 서보 출력 -1/0/+1 에서 포드 각도를 실측해 PWM_MAIN_MINn/MAXn 과
   CA_AIRSHIP_TLMIN/TLMAX 가 같은 각을 말하게 한다. 기어비가 들어가므로 서보 각이 아니라 포드 각을 잰다.
4. **추력 곡선과 THR_MDL_FAC.** 포드 하나를 저울에 걸고 모터 명령 0.2/0.4/0.6/0.8/1.0 에서 추력을 잰다. 추력이 명령의
   제곱에 가까우면 THR_MDL_FAC 를 1 쪽으로, 선형이면 0 쪽으로: 정규화 토크 요구가 모든 수준에서 같은 토크를 내야
   요 루프 이득이 뜻을 가진다 (frames_airship/index.md 의 Tuning 1). 두 포드의 최대 추력이 `vehicles/airship_indoor.json`
   의 thrust_n 과 다르면 그 값을 고치고 생성기를 다시 돌린다.
5. **지문.** `bench_fingerprint.py` 로 Manual 출력을 기록해 두고, 펌웨어를 바꿀 때마다 0.000 µs 를 확인한다.
6. **CPU.** `fmu_shell.py 'perf'` 에서 airship_att_control 과 control_allocator 의 평균을 적는다 (2026-10-07: 8.8 µs, 29.6 µs).

## 2. 비행 카드, 실내, 바람 없음

진행 순서대로. 각 카드는 로그를 남기고 (SDLOG_MODE 1), 분석은 `forward_yaw_check.py --analyze --hardware` (EKF 만 쓰는
모드, 작성 예정) 로 한다.

| # | 카드 | 하는 일 | 보는 것 |
|---|---|---|---|
| H1 | 속도원 | 제자리에서 광류와 UWB 가 EKF 에 들어가는지 | vehicle_local_position v_xy_valid 가 참, airship_speed source 가 ground (2) |
| H2 | 정지 요 | Acro, **정지에서** (EKF 지면 속도 0.1 m/s 아래 — 표류 0.2 m/s 면 스케줄이 상한을 2.8 deg/s 로 깎는다, SITL 2026-10-12), 요 스틱 ±0.5 를 4 초씩, 그리고 ±0.25 를 4 초씩 | 각속도가 6.7 deg/s (0.25 에서 2.5 deg/s) 에 닿고 놓으면 선다 (sitl_check 의 두 판정). 두 스틱 수준에서 넘침이 비슷해야 한다: 피드포워드가 설정값의 제곱 (AS_YAWRATE_FF × |r_sp|/r_max, 2026-10-12) 으로 들어가므로 작은 스틱에서 과하게 돌지 않는다 |
| H3 | 전진 안정 | Manual 로 0.5 m/s 직진 | 핀 없는 선체는 Munk 모멘트로 돌아선다. Acro 로 바꿔 루프가 잡는지 |
| H4 | 선회 유지 두 속도 | Acro, 램프 중앙 0.6 과 램프 지난 1.25 m/s 에서 끝까지 스틱 (시뮬레이터 다리와 같은 자리); 홀이 작으면 0.3 과 0.6 | 다리마다 (N·토크 − |nRabsR|·r|r|)/(u·r) 가 k (`airship_gains.py airship_indoor --measure <분석 요약> --write --params ...` 가 재서 보관); 예상 (m+m_x)·x̄ − (m_y−m_x)·s = 0.57 N m s/rad per m/s, 시뮬레이터 0.58 |
| H5 | 놓은 뒤 회전 | H4 의 놓음 120 s | 헤딩 변화가 수 도 이내 (복원 규칙); 적분기가 한계 0.3 에 닿지 않음 |
| H6 | 속도원 상실 | 선회 중 AS_SPD_SRC 1 로 | 원천 none, 상한이 내려가 선회가 멈춤, 놓은 뒤 회전 작음 |

예측 (시뮬레이터 쌍둥이, 가정한 치수, 측정 기울기 15.4 per m/s, 램프 0.39/0.78): 상한은 0.3 m/s 에서 1.9 °/s (반경
9 m), 0.6 에서 2.0 (17 m), 0.78 에서 3.9 (11 m), 1.0 에서 3.0 (19 m), 1.25 에서 2.5 °/s (29 m). 선회 반경이 홀보다 클
수 있다: 다리의 길이 (진입 ≥ 5 s + 스틱 20 s + 놓음 120 s) 는 홀이 정하고, 짧게 날면 놓은 뒤 헤딩 변화의 허용값
(5 °/120 s) 을 시간에 비례해 줄인다. 상한에서 포드는 180°/0° 로 서로 밀며 추력의 85 % 를 요 커플에 쓴다 (시뮬레이터
다리 2: 모터 0.84/0.89): 틸트가 ±180° 까지 가고 서보가 그 자세를 버텨야 한다. 전진 중 선회가 느린 것은 결함이 아니라
이 기체의 핀이 포드에 비해 커서다 (k/N 15.4 대 generic 4.2). 포드가 무게중심 0.3 m 아래라 최대 추력에서 기수가 약
10 deg 든다.

## 3. 분석 도구에서 남은 일

- `forward_yaw_check.py --analyze --hardware`: groundtruth 토픽 없이 vehicle_local_position 과 vehicle_attitude 로 다리를
  자르고 재는 모드 (작성됨; 옆흐름은 추정기 헤딩 오차가 새는 상수를 빼고 변동으로, 고도는 거리계로 판정). 그 요약을
  `airship_gains.py --measure` 에 주면 k 가 보관되고 에어프레임·파라미터 세트가 그 값으로 다시 써진다.
- 시뮬레이터 쌍둥이의 같은 카드 (`sitl_check.py`, `forward_yaw_check.py`) 결과를 옆에 두고 읽는다.

## 4. 다음 단계의 카드 (사용자 로드맵)

- **고정 핀 뒤:** H3~H6 이 본 검증이 된다. 핀 크기는 `generic_hull_design.py --spec` 이 낸 값에서 출발.
- **서보 Y선 일체 틸트:** 2520 대신 일체 모드 (CA_AIRSHIP_GRP 0, 틸트 출력 하나) 로 같은 기체를 띄워 추력 투영과 틸트
  슬루를 본다. 좌우 대칭 장착이면 한쪽을 뒤집는 Y선. 꼬리도 러더도 없으면 요 루프는 열린다 (요 스틱이 토크, 권한 0).
- **핀 제어 뒤:** 러더·엘리베이터를 CA_SV_CS* 로 선언하면 2500 구성. 조종면 크레딧 CA_AIRSHIP_CS_K 와 속도별 러더
  크레딧 (다음 증분) 의 하드웨어 검증.
