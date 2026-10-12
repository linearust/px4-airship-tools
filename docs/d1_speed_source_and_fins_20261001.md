# D1: 속도원(대기속도 센서 + 대지속도)과 핀 — 조사 결과와 질문 (2026-10-01)

Workflow wf_464a13e6-b3b, 72 agents, verified findings 52, refuted 14.

## 2번 질문에 대한 답: 둘 다 지원

경로: PX4 = /home/h/dev/PX4-Autopilot (airship/dev 4e1baea97c), 벤치 = /home/h/dev/px4-airship-bench-tools. 세 트리 모두 읽기만 했습니다. 빌드와 시뮬레이터는 돌리지 않았습니다.

**결론: 둘 다 지원하는 것은 어렵지 않고, 둘 다 지원하는 편이 맞습니다.**
변경은 airship_att_control 한 모듈 안에서 끝납니다. 들어가는 것은 구독 2개(airspeed, vehicle_local_position), 속도원을 고르는 순수 함수 1개와 그 단위 시험, 파라미터 몇 개입니다. 보드 파일, 시작 스크립트, EKF2, airspeed_selector는 바꾸지 않습니다.

**둘 다가 맞는 이유: 두 속도원이 맡는 조건이 다릅니다**
- 핀이 내는 힘은 모두 대기속도로 정해집니다. 감쇠는 u에 비례하고, 핀 받음각은 대략 r·l/u입니다. 그래서 바람이 부는 실외에서는 피토가 1순위입니다.
- 실내에는 바람이 없어서 대지속도가 곧 대기속도입니다. 반면 피토는 1.5 m/s 이하에서 동압이 1.4 Pa 이하라 거의 읽지 못합니다.
- 실외 3~6 m/s에서는 동압이 5.5~22 Pa라 피토를 쓸 만합니다. 바람 때문에 대지속도가 틀어지는 곳도 바로 이 속도대입니다.

**지금 PX4에 있는 것 (코드로 확인)**
- 드라이버는 이미 들어 있습니다. 비행선 v6x 이미지에 SDP3X, MS4525DO, MS5525DSO I2C 드라이버와 DroneCAN 대기속도가 포함되어 있습니다(boards/px4/fmu-v6x/default.px4board:19).
- 켜는 데는 기체 파라미터 하나면 됩니다: SENS_EN_SDP3X, SENS_EN_MS4525DO, SENS_EN_MS5525DS 중 하나를 1로 둡니다(ROMFS/px4fmu_common/init.d/rc.sensors:147-179).
- sensors 모듈이 차압을 `airspeed` 토픽(IAS, TAS, confidence, timestamp_sample)으로 바꿔 발행합니다. 기체 종류를 따지지 않습니다(src/modules/sensors/sensors.cpp:265-351, 610-614).
- airspeed_selector는 비행선 보드에서 빠져 있고(boards/px4/fmu-v6x/airship.px4board:1) 시작되지도 않습니다(rc.airship_apps:11-21). 켜더라도 비행선에서는 거의 아무것도 하지 않습니다:
  - MAV_TYPE 7이면 vehicle_type이 UNSPECIFIED로 남습니다(Commander.cpp:816, 1935-1950). 그래서 고정익 전용 검사는 전부 꺼집니다(airspeed_selector_main.cpp:374-376, 427).
  - 남는 것은 1 s 무자료 검사 하나입니다(:434-437). 이 검사는 기본값 ASPD_FS_T_START −1에서 신호가 한 번 끊기면 재부팅할 때까지 센서를 꺼 둡니다(AirspeedValidator.cpp:368-371).
  - '대지속도 − 바람' 대체값은 비행선에서 NaN입니다(airspeed_selector_main.cpp:594-596, 618-633).
- EKF2도 비행선에서는 대기속도를 쓰지 않습니다(airspeed_fusion.cpp:98-100, EKF2.cpp:2830). 하지만 D1에 필요한 것은 대기속도이지 바람 추정이 아닙니다. 그래서 EKF2를 바꿀 필요가 없습니다.
- 검토 문서 reference_design_review_20260930.md:163, :207, :263은 'EKF2 변경도 필요하다'고 적었습니다. 이 말은 바람 추정(후보 A)에만 맞으므로 정정합니다.

**정확한 선택 논리 (airship_att_control 안의 순수 함수)**
매 제어 주기마다 판정합니다. 새 메시지가 왔을 때만 보는 것이 아니고, 각 표본 자신의 타임스탬프로 나이를 잽니다.
- 대기속도 사용 조건(모두 만족): 속도원 마스크 비트0이 켜져 있음, airspeed 표본 나이 1 s 미만, TAS가 유한한 값, confidence > 0.
- 대지속도 사용 조건(모두 만족): 마스크 비트1이 켜져 있음, vehicle_local_position 나이 1 s 미만, v_xy_valid, dead_reckoning 아님, vx·vy·heading이 유한한 값.
  - GNSS는 요구하지 않습니다. 그래서 실내 광류나 비전도 속도원이 됩니다. airspeed_selector는 GNSS를 요구합니다(airspeed_selector_main.cpp:582-586).
- 동체 x 대지속도: u_g = vx·cos(heading) + vy·sin(heading) (VehicleLocalPosition.msg:11, 26-27, 42, 77).

어느 값을 쓰는지:
1) 대기속도를 쓸 수 있으면: u = max(TAS, 0).
   - 대지속도도 쓸 수 있으면 u = min(u, max(u_g, 0) + W_MAX)로 위를 묶습니다. 후류나 영점 오차 때문에 높게 읽는 피토가 FF를 과하게 넣지 못하게 하려는 것입니다.
2) 대기속도는 없고 대지속도만 있으면:
   - FF와 실속 상한에는 max(u_g − W_MAX, 0)을 씁니다.
   - 1/u 상한에는 u_g + W_MAX를 씁니다.
   - 두 쪽 모두 모르는 바람에 대해 안전한 방향입니다. 실내는 W_MAX가 0이라 값이 정확합니다.
3) 둘 다 없으면: 목표 u = 0, 즉 오늘과 같은 동작입니다.

전환과 필터:
- 속도원을 잃으면 즉시 다른 것으로 바꾸고, 되찾으면 1~2 s 뒤에 돌아옵니다(히스테리시스).
- u는 매 주기 1 s 저역 통과 필터를 거칩니다. 쓸 수 있는 속도원이 없으면 0까지 줄어들 뿐, 마지막 값에 멈춰 있지 않습니다.
- 업스트림 세 곳이 이 부분을 틀리게 구현했으므로 따라 하지 않습니다: FixedwingRateControl.cpp:168-169(실행되지 않는 리셋), FixedWingModeManager.cpp:174, vtol_att_control_main.cpp:393-395.
- 피토가 유효한데 하한보다 낮게 읽으면 대지속도로 넘어가지 않습니다. 스케줄이 평평해질 뿐입니다. 실외 저속에서 대지속도로 넘어가면 뒷바람일 때 FF가 과하게 들어가기 때문입니다.
- W_MAX는 기체마다 예상하는 선체 방향 최대 풍속입니다. 실내에서는 0입니다.

**PX4에서 바뀌는 것과 바뀌지 않는 것 (권장안: 원시 `airspeed` 토픽을 직접 읽기)**

바뀌는 것:
- airship_att_control: 지금은 속도를 하나도 구독하지 않습니다(airship_att_control.hpp:125-130).
- 새 AS_* 파라미터.
- FF를 매 주기 넣는 setter: 지금은 파라미터가 바뀔 때만 들어갑니다(airship_yaw_rate_loop.hpp:58-69).
- u와 선택된 속도원의 로그.
- 파라미터 설명과 문서: 지금은 '대기속도 스케일링 없음'이라고 적혀 있습니다(airship_att_control_params.yaml:69, docs/en/frames_airship/index.md:93).

바뀌지 않는 것: 보드 파일, rc.airship_apps, EKF2, airspeed_selector, commander, 배분기.

기체마다 설정할 것(하드웨어에 따라 다르므로 기체 파일 기본값에는 넣지 않습니다): SENS_EN_*=1, SENS_DPRES_OFF 보정, SDP3x를 쓰면 CAL_AIR_CMODEL.

원시 토픽을 읽을 때 잃는 것:
- ASPD_SCALE.
- 다중 센서 전환.
- QGC HUD의 대기속도 표시. VFR_HUD는 airspeed_validated의 센서 원천만 보여 줍니다(VFR_HUD.hpp:85-91).
- 다만 D1의 상수를 같은 피토 값으로 측정하면 위치 오차 중 비례하는 부분은 상수에 흡수됩니다. 그래서 D1에 스케일 보정이 꼭 필요하지는 않습니다.
- EKF2도 검증된 토픽이 없으면 원시 토픽을 쓰는데, 이것이 같은 방식의 선례입니다(EKF2.cpp:2246-2303).

선택 사항(D1과 별개):
- HUD 표시나 다중 센서가 필요하면 세 가지만 바꿉니다:
  - airship.px4board:1을 y로 바꿉니다.
  - rc.airship_apps에 `airspeed_selector start` 한 줄을 넣습니다. NuttX와 posix가 함께 씁니다(rc.vehicle_setup:83-87).
  - ASPD_FS_T_START를 0보다 큰 기본값으로 둡니다.
- 비용은 약 +12 KB로, 빌드하지 않은 추정입니다. 여유 공간은 231 KB입니다.

하지 말 것: 선택기나 EKF2의 고정익 게이트를 비행선까지 넓히는 것.
- 하중계수 검사는 부력이 무게를 받치는 기체에서 반드시 실패합니다(AirspeedValidator.cpp:272-276).
- 바람 추정과 스케일 추정은 옆미끄럼이 0이라고 가정합니다(:81-89).

**함정**

1. 낮은 동압
- 동압은 2/3/4/6 m/s에서 2.45/5.51/9.80/22.05 Pa입니다.
- 센서 한 눈금의 크기:
  - MS4525DO: 1.05 Pa. 2 m/s 동압의 43%입니다(MS4525DO.cpp:185-197).
  - MS5525DSO: 0.69 Pa.
  - SDP3x: 0.05 Pa 이하(SDP3X.hpp:62-64).
- 영점이 1 Pa 틀리면 속도 오차는 정지에서 1.28 m/s, 2 m/s에서 0.41 m/s, 4 m/s에서 0.20 m/s입니다.
- 거친 센서는 정지 근처에서 같은 값을 계속 내서 confidence가 0이 될 수 있습니다(sensors.cpp:72-73).
- 그래서 SDP3x 계열을 권합니다. 단 SDP3x에는 주의할 점이 있습니다:
  - 기본값 CAL_AIR_CMODEL 0에서는 흐름이 0이어도 약 1.05 Pa를 더합니다.
  - 그래서 정지해 있어도 ±1.31 m/s, 실제 2 m/s에서는 2.60 m/s로 읽습니다(airspeed.cpp:92-93, 108, 160-166; 계산으로 재현).
  - 설치 상태에 맞게 CMODEL을 바꾸고, 정지 상태와 알려진 속도에서 확인합니다.
- 어떤 센서를 쓰든 스케줄은 약 1.5~2 m/s 아래에서 평평해야 합니다.

2. 포드 후류
- 선체에 단 피토의 위치 오차나 후류에 관한 자료는 트리에 없습니다(검증 안 됨).
- 위험한 쪽은 '높게 읽기'입니다. FF가 과하게 들어갑니다. '낮게 읽기'(관 막힘 등)는 오늘 동작 쪽으로 물러날 뿐입니다.
- 대책 세 가지:
  - (a) 포드 후류와 선체 앞 정체 영역을 피해 답니다. 기수 앞 붐이나 선체 위 마스트가 후보입니다. 틸트가 ±180°라 후류 방향도 바뀝니다. (일반 원칙이고 트리에 근거는 없습니다.)
  - (b) 위 선택 논리의 u ≤ u_g + W_MAX 묶음.
  - (c) 바람 없는 공기에서 스로틀과 틸트를 바꿔 가며 피토 값과 대지속도를 비교합니다. 스로틀에 따라 변하는 차이가 후류의 몫입니다.

3. 바람(대지속도의 한계)
- 대지속도 = 대기속도 + 선체 방향 바람입니다.
- 역풍일 때: FF가 모자라고 1/u 상한이 느슨해서, 오늘처럼 적분기가 감깁니다. 예: 역풍 3 m/s, 대지 4 m/s에서 상한은 2.9~3.9 °/s를 허용하지만 실제 도달은 1.9~2.2 °/s입니다(검토 :237, 모델).
- 뒷바람일 때: FF가 넘치고 실속 상한이 느슨해서 꼬리를 실속시킬 수 있습니다. 예: 가정한 꼬리 A, 대지 4 m/s, 뒷바람 2 m/s에서 실속 경계는 6.2~7.9 °/s인데 대지속도 기준으로는 12.4~15 °/s를 허용하고, FF는 약 2배가 됩니다(모델).
- PX4는 비행선에서 비행 중 바람을 추정하지 않습니다(sideslip_fusion.cpp:52-55, airspeed_fusion.cpp:98-100).

4. 아무도 대기속도를 검사하지 않음
- 비행선에서는 선택기 검사, EKF2 혁신 검사, 무장 전 대기속도 검사(airspeedCheck.cpp:46-49)가 모두 꺼져 있습니다.
- 그래서 위의 표본 나이, 유한값, confidence, 음수, 교차검증은 D1이 직접 해야 합니다.

5. 보정
- QGC의 대기속도 보정 메뉴는 SYS_HAS_NUM_ASPD가 0이 아닐 때만 보인다는 조사 결과가 있습니다. QGC 소스는 이 트리 밖이라 제가 직접 확인하지 못했습니다.
- 이 파라미터는 비행선 펌웨어 동작에는 영향이 없습니다(airspeedCheck.cpp:46-49).

**권장**
- D1은 원시 `airspeed`(TAS)를 먼저 쓰고, 없으면 EKF 동체 x 대지속도를, 둘 다 없으면 오늘 동작으로 갑니다. 보드, 시작 스크립트, EKF2, airspeed_selector는 그대로 둡니다.
- 속도원 마스크 하나로 기체마다 고릅니다:
  - 실내 고정 핀: 대지속도, W_MAX 0.
  - 실외: 대기속도 우선, 대지속도 대체. 또는 대기속도만.
- 센서는 SDP31/32/33 계열을 권합니다(SDP32의 측정 상한은 약 137 Pa, 약 15 m/s). CAL_AIR_CMODEL은 설치 상태에 맞추고, 포드 후류 밖에 답니다.


## 핀에 대한 질문

1. 1번 질문이 반만 답이 되어 다시 여쭙니다. MATLAB 기체(8.35 m)의 핀은 움직입니까? 움직인다면 다섯 가지를 알려 주십시오: 어느 핀이 움직이는지, 핀 전체가 도는지 아니면 뒷전 방향타만 도는지, 방향타의 시위 비율과 스팬, 최대 각도(±°), PX4 서보 출력이 직접 구동하는지. 이유: 러더를 요 루프에 넣을지, CA_SV_CS*와 CA_AIRSHIP_CS_K를 어떻게 둘지가 이 답으로 정해집니다. 지금 기본값 CS_K 1에서 요 러더를 선언하면 요 각속도 루프가 아예 닫히지 않습니다(PX4 src/modules/airship_att_control/airship_manual_input.hpp:183).
2. 핀은 몇 개이고 어떻게 배치되어 있습니까(+형, X형 등)? 위·아래·옆 핀의 크기가 같습니까? 이유: 요에 작용하는 핀 면적이 이것으로 정해집니다. 벤치 설계 도구는 직사각형 판을 + 배치로 붙인 경우만 다룹니다(벤치 fin_basis.py, generic_hull_design.py).
3. 핀마다 다음 치수를 알려 주십시오: 뿌리 시위, 끝 시위, 선체 표면에서 잰 노출 스팬, 후퇴각. 치수를 적은 스케치나 자를 대고 찍은 사진이면 충분합니다. 이유: FF 기울기 k, k/u 상한, 꼬리 실속 속도가 모두 이 치수에서 나옵니다.
4. 핀 뿌리의 앞전과 뒷전은 기수에서 몇 m 위치에 있습니까? 무게중심의 x 위치는 어디입니까? 이유: 핀 팔 길이는 감쇠에 제곱으로 들어갑니다. 팔 길이가 10% 틀리면 k는 21% 틀립니다.
5. 핀 단면과 구조는 어떻습니까(평판, 폼, 에어포일, 공기주입식 중 무엇인지)? 두께는 얼마입니까? 이유: 양력 기울기와 실속각이 이것으로 정해집니다. 최대 양력계수가 0.55냐 0.7이냐에 따라 실속 속도가 13% 달라집니다.
6. 핀이 붙은 꼬리 부분의 선체가 MATLAB의 8.3534 × 2.3693 m 회전타원체와 같은 모양입니까? 다르다면 핀 뿌리 위치에서의 선체 반지름을 알려 주십시오. 이유: 선체와 핀이 서로 영향을 주는 몫이 gz 핀에서는 감쇠의 약 43%입니다.
7. (덜 급함) 핀 한 장의 질량은 부착물을 포함해 얼마입니까? MomentOfInertia.m:4의 관성 대각값에 핀과 가스가 들어가 있습니까? 이유: 요 관성이 핀 때문에 8~15%, 가스 때문에 22~30% 달라질 수 있습니다. 응답 시간상수에는 영향이 있고, 정상 선회의 FF에는 영향이 없습니다.

## 그 밖의 질문

1. 벤치 FMU에 저장된 설정이 실기체에 쓸 설정입니까, 아니면 벤치 시험용입니까? 설정 내용: SYS_AUTOSTART 2520, CA_SV_CS_COUNT 4, CS0·CS1은 엘리베이터 TRQ_P 0.5, CS2·CS3은 러더 TRQ_Y 0.5, CA_AIRSHIP_CS_K 0.5 (벤치 head_20260928/before.json). 이유: 실기체 설정이라면 지금 펌웨어에서도 요 루프가 포드와 러더가 섞인 플랜트 위에서 닫혀 있습니다. 이 플랜트는 정지 상태 이득이 절반이고 속도가 오르면 이득이 커집니다. D1 스케줄은 이런 구성을 다루지 않으며, 제안의 가드가 이 구성에서는 스케줄을 끕니다.
2. 러더가 움직이는 기체라면, D1 1차 동안 러더를 요에서 빼는 데 동의하십니까? 방법은 CA_SV_CSn_TRQ_Y를 0으로 두는 것이고, 엘리베이터의 피치는 그대로 둡니다. 러더 요를 루프에 넣는 것은 대기속도가 필요한 다음 증분으로 미룹니다. 이유: 동의하지 않으시면 D1 범위가 배분기의 속도 인지 표면 크레딧 작업까지 커집니다.
3. 대기속도 센서는 어떤 종류를 어디에 달 계획입니까? 종류 후보: SDP31/32/33, MS4525DO, MS5525DSO, DroneCAN. 위치 후보: 기수 앞 붐, 선체 위 마스트 등. 이유: 2 m/s 근처의 분해능과 포드 후류에 의한 오차가 이것으로 정해집니다.
4. 실외에서 운용할 전진 속도 범위와 예상 최대 풍속은 얼마입니까? 이유: 저속 대역(U_LO, U_HI), W_MAX, 피토 교차검증 허용폭의 기본값이 이것으로 정해집니다.
5. 실내 기체에 광류나 비전 같은 수평 속도원이 실제로 달려 있습니까? 벤치 FMU 설정은 EKF2_OF_CTRL 1, EKF2_GPS_CTRL 7입니다. 이유: 속도원이 없으면 실내에서 v_xy_valid가 거짓이 되어, 스케줄은 꺼진 채(오늘 동작)로 남습니다.
6. QGC HUD에 대기속도 표시가 필요합니까? 이유: 필요하면 airspeed_selector를 보드에 넣고 시작하는 작은 변경을 따로 합니다(약 12 KB, 추정). D1은 이 변경 없이도 동작합니다.
7. 검토 6.4의 3~5번을 이 제안대로 진행해도 됩니까? 내용은 세 가지입니다: FF 스케줄과 상한을 함께 넣고 기울기 파라미터를 따로 두며 저속 처리를 넣는 것, 복원 규칙은 모델 비교 후보로만 두는 것, 비행 기체 기본값은 꺼 두는 것. 이유: 작업 순서가 이 세 가지를 전제로 짜여 있습니다.

## 최종 제안 초안 (답을 받은 뒤 확정)

경로: PX4 = /home/h/dev/PX4-Autopilot (4e1baea97c), 벤치 = /home/h/dev/px4-airship-bench-tools.

**전제**
- 1번 질문은 '핀이 있다'까지만 답을 받았습니다. 그래서 검토 6.4-1의 '핀 없음' 분기는 닫습니다.
- 핀 치수와 가동 여부는 아직 모르므로 따로 다시 여쭙니다.
- 2번은 '대기속도 우선, EKF 대지속도로 대체'로 정합니다.
- 아래 1~4단계는 gz 기체로 지금 바로 할 수 있습니다. 답이 필요한 것은 실기체 값을 넣는 5단계뿐입니다.

**1. 범위: 넣는 것**

1) 속도원 헬퍼
- 2번 답의 선택 논리를 순수 함수로 만들고, AirshipManualInputTest처럼 단위 시험을 붙입니다.

2) 스케줄
- 구조는 여기서 고정하고, 숫자는 측정에서 정합니다.
- FF(u) = AS_YAWRATE_FF + w(u)·k·u
- 상한 r_max(u) = min(AS_YAWRATE_MAX, m/(k·u), c_stall·u). 이 값을 w(u)로 AS_YAWRATE_MAX와 섞습니다.
- w(u)는 속도에 따라 스케줄을 서서히 켜는 가중치입니다.
  - U_LO 아래에서는 0입니다(오늘과 같은 동작). U_HI 위에서는 1입니다.
  - 이 구간 하나가 센서 하한(약 1.5~2 m/s)과 gz 꼬리가 붙었다 실속하는 구간(2~2.5 m/s)을 함께 덮습니다(벤치 airship_tail_design.md:113-121).
- 상한은 FF와 같은 기울기 k에서 나옵니다.
  - k = N_r'/N_포드이고, 상한 = N_포드/(N_r'·u)입니다.
  - m은 여유 계수입니다. gz 2527 기준은 0.8×0.25/u입니다(airship_family_review_20260925.md:212).
  - 상한만 따로 넣지는 않습니다(검토 :231).
- 실속 분기 c_stall·u는 후보로만 둡니다.
  - gz에서는 약 2.3~2.6 m/s 아래에서만 걸립니다.
  - 측정 기체 크기의 꼬리를 가정하면 포드가 꼬리보다 5~10배 강해서, 2~4 m/s 전 구간에서 상한이 됩니다(모델).
  - 넣을지는 2단계에서 정합니다.

3) FF setter
- FF를 매 주기 넣는 전용 setter를 만듭니다. 지금은 파라미터가 바뀔 때만 넣습니다(airship_yaw_rate_loop.hpp:58-69).

4) 새 파라미터 4~6개 (이름은 가안)
- AS_SPD_SRC: 속도원 비트마스크.
- AS_YR_FF_SLP: 기울기 k.
- AS_SPD_LO, AS_SPD_HI: 스케줄을 켜는 속도 구간.
- AS_SPD_WMAX: 예상 최대 풍속 W_MAX.
- AS_YR_STALL_K: 실속 상수(넣을 경우).
- k = 0이고 실속 상수 = 0이면, 어떤 속도원을 쓰든 출력은 오늘과 같습니다.
- 기울기를 따로 두는 이유: AS_YAWRATE_FF의 최대값 10(airship_att_control_params.yaml:55)이 2527에서 4 m/s에 필요한 값 약 17(검토 :233)보다 작습니다.

5) 가드
- 요 표면이 선언된 구성(hasYawSurfaces, airship_manual_input.hpp:132-145)에서는 스케줄을 쓰지 않습니다.

6) 로그
- u, 속도원, FF, 상한을 기록합니다. RateCtrlStatus에는 적분기 값만 있습니다. 새 메시지를 만들지, 기존 메시지를 넓힐지는 정해야 합니다.

7) 문서
- 피토 설정을 적습니다: SENS_EN_*, 보정, CAL_AIR_CMODEL, 장착 위치.
- 대지속도는 바람 없는 공기에서만 정확하다는 한계를 적습니다.
- FF 측정 절차(index.md:115-122)를 두 가지 전진 속도로 넓힙니다.

**범위: 빼는 것**
- airspeed_selector 빌드와 시작: 선택 사항이고, 따로 합니다.
- EKF2 변경: 바람 추정은 후보 A입니다.
- 고정익 게이트 넓히기: 하지 않습니다.
- 러더를 요 루프에 넣기: 다음 증분으로 미룹니다.
- 헤딩 유지 H.
- 비행 기체의 기본 동작 변경.
- 기준 저장소 수정.

**2. 고정 핀과 움직이는 핀**

고정 핀(실내 등)
- 핀은 속도에 비례하는 감쇠만 만듭니다. 그래서 FF0 + k·u와 k/u 상한이 맞는 모양입니다(모델).
- 실내는 바람이 없어서 대지속도(광류, 비전)를 그대로 써도 맞습니다.

움직이는 핀: 지금 게이트의 동작
- 기본값 CA_AIRSHIP_CS_K 1에서 요 러더를 선언하면 루프가 열립니다(airship_manual_input.hpp:183).
- CS_K가 1보다 작으면 루프가 닫히지만, 포드와 러더가 섞인 플랜트 위에서 닫힙니다.
- 이 혼합 플랜트는 정지 상태 이득이 (1−K)배이고, 속도가 오르면 u²에 비례해 이득이 커집니다(ActuatorEffectivenessAirship.cpp:173-181, 모델).
- 이 모양은 D1 스케줄이 다루지 못합니다.

움직이는 핀: D1 1차의 처리
- 러더를 요에서 뺍니다. 방법은 CA_SV_CSn_TRQ_Y를 0으로 두는 것입니다. 엘리베이터의 피치는 그대로입니다.
- 그러면 요에 대해서는 고정 핀과 같아져서 D1을 그대로 쓸 수 있습니다.
- 대가: 러더가 어느 모드에서도 요를 만들지 않습니다.
- 포드가 꼬리보다 강한 기체라면 이 손실은 작을 것으로 추정합니다. 하지만 러더 크기를 알아야 확인할 수 있습니다.

주의할 함정
- CS_K 1에서 요 러더들의 TRQ_Y 합이 1 이상이면 포드에 요 몫이 전혀 가지 않습니다.
- 그러면 정지 비행에서는 스틱을 움직여도 기체가 돌지 않습니다(ActuatorEffectivenessAirship.cpp:176-181).

다음 증분: 러더 요를 루프에 넣기
- 배분기에서 표면의 크레딧이나 효과를 동압에 맞게 바꾸는 방식이 모든 구성에 통하는 구조로 봅니다.
- 선례는 틸트로터의 실행 중 행렬 갱신입니다(ActuatorEffectivenessTiltrotorVTOL.cpp:68-69, 177-179). 이것은 설계 추론이고, 시제품은 없습니다.
- 러더 권한은 동압에 비례하므로, 이 단계에서는 대기속도가 사실상 필요합니다. D1의 속도원을 그대로 다시 씁니다.

**3. 작업 순서**
단계마다 검증하고, 멈춰서 검토를 받습니다. 커밋은 검토를 받은 뒤에 합니다.

1) 측정 먼저 (펌웨어 변경 없음)
- yaw_model을 HEAD에 맞춥니다:
  - P를 0.7로 하고 AS_YAWRATE_FF를 읽게 합니다. 지금은 P 0.5이고 FF를 읽지 않습니다(yaw_model/controller.py:88-95, 327-340).
  - 스로틀이 포드까지 전달되게 합니다. 지금은 추력이 0으로 들어갑니다(closed_loop.py:116, gate.py:542).
  - 자유 서지 전진 스크립트를 보관 파일로 만듭니다.
- 검토의 87~90°와 2.66 °/s는 보관되지 않은 임시 모델에서 나왔습니다. 그 모델은 u를 고정하고 추력을 0으로 두었으므로 다시 계산합니다.
- 검증: 제자리 판정 48/48이 그대로 재현되는지, 핀 복제가 엔진 측정값 216.4~216.9u와 맞는지 대조합니다.
- SITL: 새 스크립트 forward_yaw_check.py를 만듭니다.
  - sitl_check.py는 그대로 두어 48개 판정을 회귀 기준으로 씁니다.
  - 2527은 파일에서 계산한 트림 스로틀로 날립니다. 검토에 적힌 스틱 −0.949/−0.795는 rev-3 이전 항력 기준이라 1.83/3.66 m/s밖에 나오지 않습니다. rev-3의 트림 추력은 0.0305/0.1221입니다.
  - 2522는 항력이 없어서 트림이 없습니다. 추력 0으로 미끄러지게 하거나 속도 루프를 씁니다.
  - 각 다리마다 가속과 트림을 다시 하고, 정지 상태를 확인합니다.
  - 각 다리의 순서: Manual에서 Acro로 바꿈, 스틱 20 s, 놓고 120 s.
  - 측정 기록은 ULog로 남깁니다.
- 검증: 모델이 같은 입력으로 SITL 결과를 재현하는지 봅니다. 이를 위해 gate.py를 확장합니다.

2) 모델에서 정책 비교
- 비교할 정책: 스케줄 + 상한(필요하면 실속 분기 포함), 복원 규칙, 둘의 조합.
- 비교할 조건:
  - 속도 2/2.5/4/6 m/s.
  - 속도원 오류: 대지속도에 ±2~3 m/s 바람, 피토 +1 Pa, 신호 끊김과 고착.
  - FF 오차 ±30%와 2배.
- 핀 치수를 받으면, 기준 선체에 실제 핀을 붙인 평면 모델을 두 번째 사례로 만들어 실속 분기를 확인합니다.
- 검증: 고른 정책이 HEAD 기준선보다 놓은 뒤 드리프트가 작고, 제자리 판정은 그대로인지 봅니다.

3) 펌웨어 (단위 시험을 먼저 씀)
- 시험할 경우: NaN, 오래된 표본, confidence 0, 음수, 마스크, 히스테리시스, 교차검증 묶음, 속도원이 없을 때 0으로 줄어드는지, 요 표면 가드, k = 0일 때 오늘과 비트 단위로 같은지.
- 검증: 단위 시험, identity 하네스(기본값에서 출력이 같은지), SITL 48/48, 새 v6x 이미지로 벤치 Manual 지문 0.000 µs.

4) SITL 게이트
- 2522와 2527의 전진 케이스를 posix 값으로 돌립니다.
- 속도원 시험 세 가지:
  - (i) 대지속도만 쓰는 경우.
  - (ii) PX4_PARAM_SENS_EN_ARSPDSIM=1로 시뮬레이션 피토를 켜는 경우(px4-rc.gzsim:226-229). 이 피토는 바람을 보지 못하고, 8 Hz에 1 Pa 잡음이며, off/stuck/wrong 고장을 넣을 수 있습니다. 대체 동작을 확인합니다.
  - (iii) 바람: 2522는 세계 바람만 겁니다. 2527은 /world/lta/wind와 /ocean_current에 같은 벡터를 겁니다(enable_wind: true 필요).
- 바람을 반영하는 대기속도가 필요하면 모델에 px4::AirSpeed 플러그인을 넣어야 합니다.
  - 근거: src/modules/simulation/gz_plugins/airspeed/AirSpeed.cpp:126-146. advanced_plane/model.sdf:617-619가 같은 방식으로 씁니다.
  - 모델 서브모듈 변경이므로 따로 커밋합니다.

5) 실기체 측정 카드 (펌웨어와 별개)
- 대기속도를 기록하면서 다음을 잽니다:
  - 호버와 두 전진 속도에서 Manual 정상 선회.
  - 스틱을 놓은 뒤의 감쇠.
  - 실속 시작점(모멘트를 천천히 올려 가며).
  - 바람 없는 공기에서 피토 값 대 대지속도(후류 확인).
  - 추력 곡선과 틸트 슬루.
- 바람이 고르면 피토 없이도 k를 잴 수 있습니다:
  - 일정한 스로틀과 각속도로 정확히 한 바퀴 돌면, 그동안의 동체 x 대지속도 평균이 대기속도와 같습니다(유도만 했고 시험은 안 했습니다).
  - gz 기체 기준으로 한 바퀴에 70~165 s 걸립니다.
- 이 값이 들어오기 전까지 2520의 스케줄은 꺼 둡니다.

그다음이 H(헤딩 유지)입니다.

**4. 기본값**

비행 기체 2500, 2507, 2520
- k = 0으로 스케줄이 꺼져 있습니다. 동작과 벤치 지문이 오늘과 같습니다.
- 마스크는 3(대기속도 우선, 대지속도 대체)으로 두어도, k = 0이면 아무 영향이 없습니다.
- 2500은 추진 요가 없어서 원래 루프가 열려 있습니다.

gz 기체
- 핀 기체 2522와 2527에는 gz에서 잰 k와 속도 구간을 넣고, W_MAX는 0으로 둡니다(바람 없는 공기).
- 2526과 Cloudship 핀 기체(2512, 2516, 2517)는 측정한 뒤에 넣습니다.
- 핀 없는 기체는 꺼 둡니다.
- 시뮬레이션 피토는 기본으로 켜지 않습니다.

**5. 아직 검증되지 않은 것**
- 실기체 핀에 관한 모든 것: 치수, 가동 여부, PX4가 구동하는지. 벤치 FMU의 4면 CS_K 0.5 설정이 실기체 설정인지도 모릅니다.
- 전진비행 결함과 D1의 효과는 모델 결과뿐입니다. 벡터링 비행선을 SITL에서 전진 비행시킨 적은 한 번도 없습니다.
- 피토 관련:
  - 2~6 m/s에서의 잡음과 영점 드리프트(데이터시트 확인 필요).
  - 선체 위치 오차와 포드 후류.
  - CMODEL이 실제 배관에 맞는지.
- 기체 관련:
  - 실기체 포드 권한 116.9 N·m. 여기에 쓰인 35 N은 Simulink 포화값입니다.
  - 실제 핀의 실속과 이력.
  - 관성값에 핀과 가스가 들어 있는지.
- gz 관련:
  - 2527의 Munk가 바람 속에서 지면 기준으로 남는지. 코드만 읽었습니다.
  - PX4_PARAM_ 환경 변수가 gz_run.sh를 거쳐 실제로 적용되는지.
  - px4::AirSpeed 플러그인의 온도 문제 가능성: 이 플러그인은 온도를 싣지 않습니다. 그러면 GZBridge.cpp:430이 −273 °C로 받고, VehicleAirData.cpp:94-99가 이를 −60 °C로 잘라서 SITL의 TAS가 약 14% 낮게 나올 수 있습니다. 제가 코드만 읽은 것이고 실행해 보지는 않았습니다.
- airspeed_selector를 넣을 경우의 크기: 약 12 KB라는 추정이고, 링크해 보지 않았습니다.


## 반박되어 쓰지 않은 주장

- [airspeed_path] SITL cannot tell airspeed from ground speed. Both simulated airspeed paths read ground velocity with zero wind. — The two paths the finding lists are wind-blind, as it says. The finding misses a third gz airspeed path that is already in the tree and does use wind, so "SITL cannot tell airspeed from ground speed" 
- [fw_precedent] Fixed-wing reads calibrated airspeed (CAS) from airspeed_validated. When it is missing or disabled, it uses a configured constant, never ground speed. fw_rate_control does not subscribe to vehicle_loc — Every cited line is accurate, and the narrow facts about fw_rate_control hold. The finding's load-bearing parts fail: 'never ground speed' for fixed-wing, and the D1 consequence that the EKF ground-sp
- [fw_precedent] Use calibrated airspeed (CAS) as the airspeed, as every fixed-wing consumer does. At airship altitudes the CAS/TAS difference is at most about 5%, which is small next to the wind error of ground speed — The citations are accurate, and so is the arithmetic under the finding's own model. Checked: FixedwingRateControl.cpp:158,166; FixedwingAttitudeControl.cpp:154,162; FwLateralLongitudinalControl.cpp:70
- [fw_precedent] Upstream declares a source lost after 1 s everywhere. Consumers have no switch hysteresis. The selector itself latches a failed sensor off, and by default never turns it back on in flight. — I checked every citation in the airship/dev tree at 4e1baea97c. These files are unchanged from the merge base with upstream fe87271f9c. On upstream/main b3e343c153 only line numbers move: FixedwingRat
- [fw_precedent] The only in-tree ladder that mixes airspeed and EKF ground speed is VTOL back-transition, and it is ground-first because the decision is kinematic. Other modules use ground speed only where ground spe — The central claim is false: VTOL back-transition is not the only in-tree ladder that mixes airspeed and EKF ground speed.

(1) airspeed_selector is an explicit ladder:
- Rungs: airspeed sensors 1-3, t
- [fw_precedent] SITL can exercise the ground-speed rung and the source switching, but not the airspeed rung as it stands, and not wind behaviour. — The finding is half wrong. SITL can already exercise the airspeed rung, with no model change. The wind half stands, and it is stronger than the finding says.

1) The airspeed rung works in SITL throug
- [fins_surfaces] 'The vehicle has fins' resolves only half of the review's question 1. The fins' size decides whether the vehicle is directionally stable. The MATLAB hull's Munk coefficient is m22 - m11 = 22.24 kg, wh — The arithmetic holds. The stability conclusion and the D1 consequence do not.

What holds (recomputed):
- Munk coefficient. From getParameters.m:35-52 I get k1 0.09748, k2 0.83685 and rho*V 30.077 kg,
- [fins_surfaces] In hover the fins also add quadratic yaw damping from their broadside drag, and that sets FF0. Neither gz nor the yaw_model replica carries it, because LiftDrag applies nothing when forward·v ≤ 0. A l — The physics is right and the arithmetic reproduces. The central sub-claim is false: gz and the yaw_model both carry this term. It is gated, not absent, and a second number is stale.

1. **gz carries i
- [fins_surfaces] What I could not verify:
- Anything about the real vehicle's fins. No tree has a record, and the reference repo contains no fin, tail or rudder geometry.
- Whether the MATLAB vehicle's fins move.
- Th — Partly refuted. The cited lines are accurate, but the finding inherits an overstated claim from the review, and two of its "could not verify" items can be settled from local evidence. Everything here 
- [reference_fins] The reference vehicle model has no fin of any kind. This holds for the MATLAB, its Python port and the SLX: no fin geometry, fin mass, fin share of inertia, fin coefficient, or fin force or moment ter — Refuted as worded. The core about fin aerodynamics holds, and so does the D1 consequence. The absolute "no fin of any kind" does not hold, because two of its sub-claims are unsupported.

VERIFIED (eve
- [reference_fins] For a plausible cruciform tail (ASSUMED, not the real one), speed-dependent fin yaw damping is one to two orders of magnitude larger than the model's r|r| moment at 2-6 m/s. — The core inputs check out, but the headline and several of its supporting numbers do not.

**Verified** by recomputing in /tmp/claude-1000/-home-h-dev-PX4-Autopilot/1e48cf6c-455a-42ab-ba26-91399c708d3
- [reference_fins] Whether the fins are movable decides whether D1 runs on this vehicle at all. — The citations are accurate, but the headline does not follow from them.

What I confirmed:
- hasPropulsiveYaw returns `(pod_grouping > 0 || tail) && !(yaw_surfaces && surface_credit >= 1.f)` (airship_
- [measurement_first] Run time, estimated and not measured. SITL per plant at real-time factor 1: about 1-1.5 min for boot, EKF and arming; 2 × (6-13 s acceleration + 10 s settle); 4 × (2 + 20 + 120 + 15-20 s). That is abo — The SITL half holds up. The model-cost half does not: its own evidence does not support it, and the RTF statement leaves out measurements that exist.

What holds:
- The SITL figure is labelled as an e
- [measurement_first] At 2 m/s, full stick is on the tail's hysteretic stall boundary, so that leg's outcome depends on the state it starts from. — The cited evidence puts the hysteresis at a different speed than the claim does.

(1) /home/h/dev/px4-airship-bench-tools/airship_tail_design.md:113-118 records the following, and /home/h/dev/px4-airs
