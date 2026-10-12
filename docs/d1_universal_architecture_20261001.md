# D1 범용 구조 최종 제안 (2026-10-01)

Workflow wf_041c48d1-b87 (근거 3, 설계안 3, 심사 3, 종합, 핵심 주장 반박 검증).

선택: staged design (best-scored) with the judges' grafts and fixes. Increment 1 (first firmware increment, removes the forward-flight defect): controller-only speed schedule in airship_att_control — one dual-source speed selector (raw airspeed TAS with a dynamic-pressure tolerance band, EKF body-x ground speed ± W, intersection with pitot rejection when empty, per-cycle ageing, 1 s decay, upper bound held while the loop is closed) published as one airship_speed message; per-cycle FF(u) = FF0 + w·k·u_lo; rate cap from a tilt-range-aware authority A, a reserve ρ = min(AS_YR_INT_LIM, A/2) and the speed interval, applied twice: as the stick's full scale (the max_rate argument, so the whole stick stays useful) and as a clamp on every setpoint at the loop input (the rover's placement, so a later heading-hold source is limited too); guard that keeps today's behaviour where yaw surfaces are declared; allocator byte-identical. Increment 2 (movable '+' fins, the general case): per-cycle, per-axis, speed-scaled surface credit CS_K·min(1,(u_lo/CA_AIRSHIP_CS_V)^2) at the existing ActuatorEffectivenessAirship seam (:174/:305) on axes propulsion also serves, CS_K unchanged on surface-only axes, static matrix, credit decays to 0 on speed loss, static gate fed the at-rest credit; verified on a new gz member (independently tilting pods + four equal '+' flaps) built in parallel with increment 1, cross-checked on airship_fixed* with GRP 1. Measurement first (increment 0), defaults bit-identical to HEAD 4e1baea97c.

## 0단계 결과 반영 (2026-10-07)

0단계(측정)는 2026-10-02에 끝났습니다(step0_report_20261002.md). 1차 증분 설계는 그 결과로 다음을 고쳤고, 고친 자리에 날짜를 적었습니다. 실행 계획은 d1_increment1_plan_20261007.md입니다.
- 결정 1: 복원 규칙을 항상 씁니다(2.다, 남은 결정).
- ρ = 0.5·min(AS_YR_INT_LIM, A/2). 한계와 같은 ρ는 결함 상태를 허용했습니다(2.다).
- 구간 항은 실제로 내는 FF_s로 씁니다(2.다).
- 램프 안에서 상한을 MAX 쪽으로 섞지 않습니다. 두 항이 속도 0에서 저절로 MAX를 넘습니다(2.다).
- 실속 항은 뺍니다(2.다).
- A는 정지 권한으로 두고, 집합 추력은 배분기 깃발에 맡깁니다(2.나).
- 실외 대지속도만 쓰는 구성은 정지에서도 상한이 낮습니다. 의도된 동작으로 문서화합니다(2.다, 3-7).
- 꼬리 추진기의 k는 핀으로 계산하지 않고 선회 유지로 잽니다(Cloudship 핀 0.79 대 실제 12.74).
- 기반 커밋은 2026-10-07 동기화 뒤의 airship/dev bccba2ef8c입니다(옛 4e1baea97c와 내용이 같음).

## 1) 결론

단계형 설계(staged)를 바탕으로 하고, 심사에서 나온 보완과 결함 수정을 넣었습니다.
- 속도에 따라 변하는 두 항을, 그 모델을 이미 가진 곳에 둡니다.
  - 핀 감쇠(속도 u에 비례)는 핀이 고정이든 움직이든 모든 핀 기체에 있는 플랜트 항입니다. 제어기(airship_att_control)가 매 주기 FF(u)와 속도 상한으로 다룹니다.
  - 움직이는 핀의 조종 권한(동압, u²에 비례)은 액추에이터 항입니다. 배분기(ActuatorEffectivenessAirship)가 이미 매 주기 쓰는 표면 크레딧(cpp:174, :305)을 속도에 맞춥니다. 행렬은 그대로 둡니다.
- 속도는 제어기 안의 선택기 하나가 고릅니다. 순서는 피토(TAS) → EKF 동체 x 대지속도 → 없음(오늘 동작)입니다. 결과는 하한·상한 구간으로 airship_speed 토픽 하나에 실리고, 두 모듈이 같은 값을 씁니다.
- 1차 증분은 제어기만 바꾸고(배분기 바이트 동일) 전진비행 결함을 없앱니다. 2차 증분은 일반 경우인 움직이는 '+' 핀을 같은 구조에 꽂습니다. 2차를 검증할 gz 기체(독립 틸트 포드 + '+' 플랩 4장)는 1차와 병행해 만듭니다.
- 새 파라미터 기본값에서 출력은 HEAD 4e1baea97c와 비트 단위로 같습니다. 비행 기체는 오늘 동작 그대로입니다. 속도원이 없으면 오늘 동작으로 돌아가는 것은, 고정익이 대기속도가 없을 때 FW_AIRSPD_TRIM(스케일 1)으로 돌아가는 것과 같은 모양입니다(FixedwingRateControl.cpp:158-162).

값이 바뀔 때(소유자 말씀대로 핀 치수·위치, 센서, 기체가 모두 설계 중):
- 바뀌는 것은 파라미터뿐이고 구조는 그대로입니다. 핀이 바뀌면 FF 기울기 k, 표면 TRQ, CS_V, 램프 구간만 바뀝니다.
- 포드 틸트 범위가 바뀌면 권한 A가 공유 함수에서 저절로 따라갑니다. 센서가 바뀌면 허용폭 Δq와 마스크만 바뀝니다.
- k가 틀리면 유지 선회 중 적분기 값에 바로 드러납니다(yawspeed_integ ÷ 각속도 = 모자란 FF, 문서의 기존 절차 index.md:118-120). 이 절차를 두 전진 속도로 넓히면 k를 비행 로그에서 다시 잴 수 있습니다.

이 문서의 D1은 09-30 검토의 '속도 인지 수동 요 각속도 루프'입니다. 09-25 리뷰의 D1(e550ec6597, 표면 게이트)과 다릅니다.

## 2) 구조

**데이터 흐름.** 자이로 콜백 한 주기(400 Hz)에 rate_ctrl 작업 큐에서 이 순서로 돕니다.
1. 선택기가 돌고 airship_speed를 발행합니다. 모든 모드에서, 렌치보다 먼저입니다.
2. 스케줄이 FF와 상한을 냅니다.
3. 요 루프가 돕니다. 게이트는 그대로입니다.
4. 렌치를 발행합니다.
5. 배분기 콜백이 돕니다. 2차부터는 같은 주기의 airship_speed로 축별 크레딧을 계산하고, 포드·꼬리 → 부족 부호 → 적분기 정지로 이어집니다. 적분기 정지는 그대로입니다(airship_yaw_rate_loop.hpp:76-82).

**가. 속도 선택기** — 새 파일 airship_att_control/airship_speed_selector.hpp. uORB 없는 순수 헤더로, airship_manual_input.hpp와 같은 방식입니다.
- 입력은 둘입니다.
  - 원시 airspeed: IAS, TAS, confidence, timestamp_sample.
  - vehicle_local_position: vx, vy, heading, v_xy_valid, dead_reckoning, heading_good_for_control.
- 사용 조건. 매 주기 각 표본 자신의 timestamp_sample로 나이를 재고, 1 s 미만이어야 합니다.
  - 피토: 유한값이고 confidence > 0.
  - 대지속도: v_xy_valid, dead_reckoning 아님, heading_good_for_control, 유한값. GNSS는 요구하지 않으므로 실내 광류·비전도 속도원이 됩니다.
- 구간(TAS):
  - 피토: 표본의 동압에 센서 허용폭 ±Δq를 준 구간입니다. 저속에서 저절로 넓어집니다.
  - 대지: [max(u_g−W, 0), max(u_g+W, 0)], u_g = vx·cosψ + vy·sinψ.
  - 둘 다 쓸 수 있으면 교집합을 씁니다. 교집합이 비면 그 주기에는 피토를 버립니다(막힌 관, 후류, 영점 오류). 높게 읽는 쪽과 낮게 읽는 쪽을 모두 막습니다.
- 전환과 필터:
  - 속도원을 잃으면 즉시 다른 쪽으로 바꿉니다. 돌아올 때는 연속으로 유효한 뒤에 돌아옵니다(코드 상수).
  - 두 경계는 매 주기 1 s 1차 필터를 거칩니다. 시작값은 0이고 리셋 분기는 없습니다.
- 모두 잃으면: 하한은 0으로 줄어듭니다. 상한은 요 루프가 닫혀 있는 동안 유지되고, 루프가 열리면 줄어듭니다.
- 메시지는 airship_speed 하나입니다.
  - 담는 것: 구간, 속도원(NONE/AIRSPEED/GROUND), 원시 후보 값, 상한 유지 표시, 스케줄 결과(FF, 상한, A).
  - 모든 모드에서 매 주기 발행합니다. 로그는 rover_rate_status처럼 100 ms 간격의 선택 토픽입니다. k 측정에 고속 로그 프로필이 필요 없습니다.

**나. 공유 헤더** — lib/control_allocation/actuator_effectiveness/, AirshipAirframe.hpp 옆. 제어기와 배분기가 같은 함수를 부르므로 두 쪽이 어긋나지 않습니다.
- 요 권한 A(방향별, 정규화 토크):
  - 독립 포드: 0.5·(1+ρ_rev). ρ_rev는 틸트 범위로 뒤로 낼 수 있는 추력 비율입니다. ±180°면 A = 1, 고정·±90°면 A = 0.5입니다. 포드 요가 두 포드 전진력 차의 절반이고(ActuatorEffectivenessAirship.cpp:161) 추력은 음수가 될 수 없기 때문입니다(AirshipPod.cpp:189).
  - 꼬리: +1. 역방향은 CA_R_REV 비트가 있을 때만 1입니다.
  - A는 정지 권한입니다(2026-10-07). 집합 추력 T가 크면 포드는 1−T(뒤로 도는 포드)나 T(못 도는 포드)까지만 1:1로 요를 냅니다(0c). 그 너머는 배분기의 부족 깃발이 적분기를 세우므로 상한이 다룰 일이 아닙니다. 그래서 A에 T를 넣지 않습니다.
  - 2차부터: 크레딧 받은 표면 c·Σ|TRQ_Y|.
- surfaceCredit(CS_K, CS_V, u): CS_V ≤ 0이면 CS_K, 아니면 CS_K·min(1, (u/CS_V)²). 지수는 동압이라는 물리에서 나옵니다.
- airship_speed의 신선도 규칙.

**다. 제어기 스케줄(1차)** — 새 순수 함수 airship_yaw_schedule.hpp, 그리고 AirshipYawRateLoop에 FF만 바꾸는 setter.
- 켜지는 조건은 파라미터가 바뀔 때 정합니다. k(AS_YR_FF_SLP) > 0이고, 명령 요 토크가 물리 단위여야 합니다.
  - 1차: 요 표면이 없을 때.
  - 2차부터: CS_V > 0일 때도.
  - 꺼져 있으면 정확히 {AS_YAWRATE_FF, AS_YAWRATE_MAX}를 돌려줍니다.
- 램프 w(u) = interpolate(u, AS_SPD_LO, AS_SPD_HI, 0, 1). 센서 하한과 핀 부착·실속 구간을 함께 덮습니다(gz는 2~2.5 m/s, 이력 있음, airship_tail_design.md:113-121).
- FF = FF0 + w(u_lo)·k·u_lo. 매 주기 setFeedForwardGain으로 넣습니다. 고정익과 같은 방식이고(FixedwingRateControl.cpp:389-392) lib/rate_control은 바꾸지 않습니다.
- 상한(0단계 결과로 고친 식, 2026-10-07):
  - FF_s = FF0 + w(u_lo)·k·u_lo. 위의 FF, 실제로 내는 값입니다.
  - ρ = 0.5·min(AS_YR_INT_LIM, A/2). 0.5는 코드 상수입니다. ρ가 한계와 같으면 '적분기가 ρ 안'이 결함 상태를 포함합니다(0c).
  - r_cap = min(MAX, (A−ρ)/FF_s [FF_s > 0일 때], ρ/(k·u_hi − FF_s) [k·u_hi > FF_s일 때]).
  - 램프 가중치로 상한을 MAX 쪽으로 섞던 것은 없앴습니다. 두 항이 속도 0에서 저절로 MAX 이상이 되므로 정지에서는 MAX가 그대로 남습니다. 램프는 FF에만 있습니다.
  - 들어가는 곳은 두 군데입니다(핵심 주장 검증의 첫 반박을 반영).
    - 스틱 최대값: airship_att_control_main.cpp:206-207의 max_rate 인자를 상한으로 바꿉니다. 그래서 스틱 전체 행정이 순항에서도 쓸모 있습니다.
    - 루프 입력: :208에서 _yaw_rate_loop.update()로 들어가는 설정점을 같은 상한으로 자릅니다. Ackermann 로버가 속도 상한을 두는 자리이고(AckermannRateControl.cpp:71-74), 모든 설정점 출처에 걸립니다.
    - 로버처럼 입력에서 자르기만 하면 스틱 사상이 고정이라, 2527의 4 m/s 순항(±180° 틸트라 A = 1, 모델의 D1 정책 상한 약 2.66 °/s(검토 :327), AS_YAWRATE_MAX 기본 15 °/s)에서 스틱의 약 82 %가 아무 일도 하지 않습니다. 스틱만 바꾸면 나중의 헤딩 유지(H)가 상한을 받지 않습니다. 그래서 둘 다 씁니다.
    - 지금은 스틱이 유일한 출처라(airship_manual_input.hpp:191-192) 입력 제한은 1차에서 걸리지 않습니다. 단위 시험으로 지킵니다. 기본값(상한 = MAX)에서 constrain은 입력을 그대로 돌려주므로 비트 동일이 유지됩니다.
  - 첫 항: FF가 내는 토크 FF_s·r에 적분기 몫 ρ를 더해도 권한 A 안에 들게 합니다.
  - 둘째 항: 구간 안 가장 빠른 속도 u_hi에서 FF가 모자라는 몫 (k·u_hi − FF_s)·r을 적분기가 ρ 안에서 받게 합니다. FF가 충분하면 무한대입니다. 0c가 지적한 대로 실제로 내는 FF_s로 씁니다. 옛 식 ρ/(k·(u_hi−u_lo))는 램프 밖에서만 맞았습니다.
  - 그래서 k가 맞으면, 구간 안 어느 실제 속도에서도 설정점에 닿고 적분기는 ρ 이하에 머뭅니다. 깃발 없이 한계에 붙는 일이 없습니다.
  - 실속 항은 넣지 않습니다. 0c에서 하한이 0이면 상한이 0이 되어 스틱이 죽었습니다(28개 다리). 2 m/s 부근의 핀 실속은 HEAD도 같은 경계에 있고, 실기체 핀 측정 뒤에 다시 봅니다.
  - 실외에서 대지속도만 쓰면(W > 0) 정지에서도 구간이 [0, W]라 둘째 항이 상한을 낮춥니다. 바람을 모르면 그만큼 느리게 도는 것이 맞는 동작이므로 그대로 두고, AS_SPD_WMAX를 실제 바람 한계로 두라고 문서에 적습니다. 실내(W 0)와 피토가 있는 구성은 정지에서 HEAD와 같습니다.
  - 수치 예(2527, 4 m/s, 속도 정확, k 4.01, FF0 1, A 1): FF_s 17.05, ρ 0.15, 상한 2.86 °/s. HEAD는 같은 속도에서 2.44 °/s로 돌며 적분기가 한계에 붙어 있습니다.
- 바꾸지 않는 것: P, I, 적분기 한계와 그 단위, anti-windup, 게이트, Manual과 비수동 모드.
- 복원 규칙(0단계 결정 1, 2026-10-07): AS_YR_RESTORE = 1이면 스틱이 데드존을 벗어나는 주기에 적분기를 저장하고, 데드존으로 돌아오는 주기에 저장값을 되돌립니다. 부호가 바뀌는 반전은 놓음과 새 스틱으로 봅니다. 루프가 열리면 저장값도 지웁니다. 규칙은 yaw_model/options/D와 policies.py의 것과 같아야 합니다. lib/rate_control에 setIntegral 하나를 더합니다. 0c: 모든 속도원과 k 오차에서 놓은 뒤 2.5° 이내, 120 s 안 최대 흔들림 2.9°(벡터드). 비용은 선회 중 요 트림이 바뀔 때 옛 트림을 되돌리는 것, 권한의 5 % 변화당 약 32°. 기본값 0이라 비트 동일이고, posix 쌍둥이에서 켭니다.

**라. 배분기 크레딧(2차)** — ActuatorEffectivenessAirship만 바꿉니다.
- 새 CA_AIRSHIP_CS_V [m/s]: CA_SV_CSn_TRQ_*가 성립하는 기준 대기속도이고, 포드 단위입니다. 0이면 오늘 코드 그대로입니다.
- airship_speed를 vehicle_status 옆에서 구독하고, updateArmedState처럼 읽습니다.
- 축별 크레딧을 매 주기 계산해 cpp:174의 상수 자리에 넣고, 같은 값을 :305의 보고에 씁니다.
  - 추진도 맡는 축(요: 독립 포드나 꼬리, 롤: 독립 포드): surfaceCredit(CS_K, CS_V, u_lo).
  - 표면만 맡는 축(피치는 항상, 집합 포드의 요·롤): CS_K 그대로입니다. 그래서 2500은 출력과 보고가 모두 바뀌지 않고, CS_K 1이면 피치 거짓 포화도 없습니다.
- 토픽이 없거나 오래되면 u_lo = 0, 즉 크레딧 0입니다. 포드와 꼬리가 전부를 맡습니다. 헬리콥터 RPM 조속기가 시간초과에 보정을 0으로 두는 것이 선례입니다(RpmControl.cpp:76-82).
- 행렬, 처짐/토크 비율, Manual의 표면 처짐은 바뀌지 않습니다. NO_EXTERNAL_UPDATE에서는 계속 false를 돌려주므로(cpp:69-71) 100 ms 갱신 제한과 약한 행 0 처리(ControlAllocator.cpp:482-483, 591-602)를 겪지 않습니다.
- 제어기 쪽에서 바뀌는 입력은 셋입니다.
  - 게이트에 정지 크레딧을 줍니다. CS_V > 0이면 0이므로, 포드나 꼬리가 있으면 러더가 있어도 루프가 닫힙니다. 게이트는 여전히 설정만의 함수입니다.
  - 물리 단위 판정에 CS_V > 0을 더합니다.
  - A에 크레딧 받은 표면을 더합니다.

**마. 코드 위치와 새 파라미터** (이름은 가안, 값은 측정에서 정합니다)
- 1차:
  - 파일: airship_att_control 안의 새 헤더 2개, 루프 setter, 구독·발행, params yaml, 시험. 공유 헤더 1개. msg/AirshipSpeed.msg와 로거 한 줄.
  - 파라미터: AS_SPD_SRC(마스크: 1 대기, 2 대지; 기본 0 = 꺼짐), AS_SPD_WMAX [m/s], AS_SPD_DP_TOL [Pa], AS_SPD_LO와 AS_SPD_HI [m/s], AS_YR_FF_SLP(k, 0이면 꺼짐), AS_YR_RESTORE(0/1, 기본 0). 실속 항과 AS_YR_STALL_K는 없습니다. lib/rate_control에 setIntegral을 더합니다.
- 2차:
  - 파일: ActuatorEffectivenessAirship.{hpp,cpp}, control_allocator/module.yaml.
  - 파라미터: CA_AIRSHIP_CS_V(0이면 꺼짐).
- 기존 이름은 그대로 두고 역할만 문서에 적습니다.
  - AS_YAWRATE_FF: 정지 FF. 최대 10을 유지하고, 속도 몫은 기울기가 맡습니다.
  - AS_YAWRATE_MAX: 상한의 최대이자 정지 상한.
  - AS_YR_INT_LIM: 적분기 한계이자 ρ의 상한.
  - CA_AIRSHIP_CS_K: CS_V > 0일 때, CS_V 이상에서의 크레딧이자 표면만 맡는 축의 크레딧.

## 3) 기체 경우별 동작

1. **고정 핀 + 포드** (gz 2522/2526/2527, 실내 고정 핀 기체)
   - 1차가 그대로 맞습니다. A는 ±180°면 1, 고정·±90°면 0.5입니다.
   - FF가 핀 감쇠를 대고 상한이 설정점을 닿을 수 있게 만들어, 결함(깃발 없이 적분기가 AS_YR_INT_LIM에 붙음)이 사라집니다.
   - 2차는 표면이 없으므로 관계없습니다.
2. **움직이는 핀 + 포드** — 일반 경우입니다. MATLAB 기체는 독립 틸트 포드에 같은 크기 '+' 핀 4장입니다.
   - 1차: 요 표면이 선언되어 있으면 가드가 스케줄을 끄므로 오늘과 같습니다. 나빠지지는 않습니다.
     - 선택 사항: 러더의 토크 열을 0으로 두면 트림에 머물러(ControlAllocationPseudoInverse.cpp:192) 고정 핀처럼 되고, 1차 효과를 바로 받습니다. 대신 러더 요를 잃습니다.
   - 2차 설정: 엘리베이터 TRQ_P와 러더 TRQ_Y를 CS_V 기준 포드 단위로 주고, CS_K 1, CS_V는 운용 최고 속도로 둡니다.
     - 정지에서는 크레딧이 0이라 포드가 요를 전부 맡고 루프가 닫힙니다. 오늘 CS_K 1에서 스틱 요가 포드에 가지 않던 함정이 사라집니다.
     - 속도가 오르면 러더가 (u_lo/CS_V)²만큼, 포드가 나머지를 맡습니다. 그래서 루프가 보는 토크가 속도와 무관해집니다.
     - 피치는 크레딧 1이라 거짓 포화가 없습니다.
     - '+'와 X의 차이, 핀 크기·위치는 TRQ와 k의 값만 바꿉니다.
3. **꼬리 추진기** (Cloudship 2507, gz 2510~2517)
   - A는 꼬리이고, 역추력 비트에 따라 방향별입니다.
   - 핀이 고정이면 1차가 그대로 맞습니다.
   - 2516/2517은 방향 정적 불안정이지만, 스케줄이 P를 건드리지 않으므로 루프 감쇠는 그대로입니다.
   - 움직이는 핀이 생기면 2차에서 꼬리가 나머지를 맡습니다.
4. **표면만** (2500, gz 2500~2505)
   - 추진 요가 없어 루프는 계속 열려 있습니다. 1차와 2차 모두 출력이 바뀌지 않습니다.
   - 선택기는 로그만 남깁니다.
5. **실내**
   - 대지속도만, W 0으로 두면 구간이 한 점이고 바람이 없어 정확합니다. 광류·비전으로 충분합니다.
   - 수평 속도원이 없으면 v_xy_valid가 거짓이 되어 오늘 동작입니다.
6. **실외, 피토와 대지속도**
   - 피토 구간과 대지 ± W의 교집합을 씁니다. 교집합이 비면 피토를 버립니다.
   - 저속은 램프가 평평하게 만듭니다. 바람은 피토가 읽으므로 정확합니다.
7. **실외, 피토 없음**
   - 대지 ± W라 구간이 넓습니다. 상한이 보수적이어서 선회가 느립니다. 그래도 적분기는 ρ 안에 머뭅니다.
   - 정지에서도 구간이 [0, W]라 상한이 이미 낮습니다(2.다). W를 실제 바람 한계로 둡니다.
   - 2차의 크레딧은 하한을 쓰므로 과소 크레딧, 즉 과다 전달 쪽으로 틀립니다.
8. **피토만** (대지속도 없음)
   - 교차검증이 없습니다. 낮게 읽으면 오늘 쪽으로 물러나고, 높게 읽으면(후류, 영점) FF와 크레딧이 과해집니다.
   - CS_V > 0이면 두 속도원을 쓰도록 문서화합니다.
9. **속도원 없음**
   - 구간이 [0, 0]이라 FF0, MAX를 씁니다. 크레딧은 정지값입니다(CS_V 0이면 CS_K).
   - 기본값에서 HEAD와 비트 단위로 같습니다.
10. **비행 중 속도 상실**
    - 다른 속도원으로 즉시 바꿉니다.
    - 둘 다 없으면 하한이 약 1 s에 걸쳐 0으로 줄어듭니다. FF는 FF0으로, 크레딧은 0으로 서서히 갑니다.
    - 상한은 루프가 닫혀 있는 동안 유지되므로 상한 값은 오히려 작아지고 선회가 느려집니다. 게이트는 그대로라 루프가 갑자기 열리지 않습니다.
    - 2차에서는 포드가 전부를 맡고 러더가 더해집니다. 그래서 루프 이득이 최대 1+(u/CS_V)²까지 오릅니다. CS_V가 최고 속도면 2배 이하입니다(계산).
    - 루프가 열리면(Manual 등) 오늘 동작입니다.

## 4) 단계

단계마다 멈추고 검토를 받은 뒤에 커밋합니다.

**0. 측정 먼저** — 벤치 저장소만 바꾸고 펌웨어는 바꾸지 않습니다. 고정 핀만 다룹니다.
- 0a. yaw_model을 HEAD에 맞춥니다.
  - P 0.7로 하고 AS_YAWRATE_FF를 읽게 합니다. 추력이 포드까지 전달되게 합니다. 자유 서지 전진 드라이버를 보관합니다.
  - 게이트: 제자리 판정 48/48 재현, 핀 복제 216.4~216.9u, 2527 결함을 다시 계산해 기록(검토 :220의 1.55/2.4 °/s, 2분 87~90°).
- 0b. forward_yaw_check.py를 HEAD에서 돌립니다.
  - 방식: sim 시계 창, ULog, 다리마다 진입 상태 비교. 2~2.5 m/s 다리는 알려진 실속 분기에서 시작합니다. PX4_PARAM_ 전달을 먼저 확인합니다.
  - 기체: 2522(무추력 활주), 2527(파일에서 계산한 트림 추력), 2512, 2517.
  - 기록: r, yawspeed_integ, unallocated_torque[2], 포드 반전 횟수.
  - 게이트: 결함의 첫 SITL 측정, 모델이 각 다리를 허용폭 안에서 재현, sitl_check 48/48 그대로.
- 0c. 모델로 정책을 비교합니다.
  - 정책: 스케줄+상한(실속 항 유무), 복원 규칙(비교 기준이자 안전장치 후보).
  - 조건: 2/2.5/4/6 m/s, 바람 ±2~3 m/s, 피토 +1 Pa·끊김·고착, FF ±30%와 2배, 적분기 시간상수(약 3분)보다 긴 유지, A = 1과 0.5.
  - 게이트: 모든 경우에 놓은 뒤 드리프트가 HEAD보다 작고, 제자리 판정이 그대로일 것. FF 오차가 남기는 드리프트를 기록해 남은 결정 1의 근거로 씁니다.

**1. 1차 증분** — 제어기만 바꾸고, 배분기는 바이트 단위로 같습니다. 결함을 없애는 가장 작은 증분입니다.
- 1a. 선택기 헤더, 단위 시험, AirshipSpeed.msg. 아직 배선하지 않습니다.
  - 게이트: 단위 시험, SITL과 v6x 빌드.
- 1b. 배선합니다: 구독 둘, 모든 모드에서 매 주기 선택기, 렌치 전에 발행, 로그.
  - 게이트:
    - 새 제어기 identity 하네스가 속도 토픽이 있을 때와 없을 때 모두 IDENTICAL.
    - 기능 시험.
    - SITL 2527에서 GNSS와 모의 피토 사이 전환, off/stuck/wrong(관 막힘) 주입, GNSS 상실이 로그에 맞게 보임.
    - sitl_check 48/48.
    - 벤치 Manual 지문 0.000 µs, perf 카운터와 이미지 크기.
- 1c. 스케줄, 공유 권한 헤더, FF setter, 복원 규칙(setIntegral), 새 파라미터(기본 꺼짐), 스틱 최대값과 루프 입력 제한.
  - 게이트:
    - 단위 시험: k = 0일 때 비트 동일, A, ρ, 상한의 연속성과 단조성, 램프 끝, 가드.
    - identity: 기본값에서, 그리고 '파라미터는 켜고 속도원은 없음'에서 IDENTICAL.
    - 배분기 하네스 12/12(트립와이어).
    - SITL 48/48, airship_fixed_flight_check 13/13.
    - gz에서 잰 k로 2522/2527/2512/2517 전진:
      - 적분기가 한계에서 떨어져 있음.
      - 드리프트가 예측 안.
      - 상한에 도달.
      - 선회 중 속도를 잃어도 같음.
      - 0c의 SCHED+RESTORE 예측(고친 식으로 다시 낸 것)과 비교: 스틱 중 마지막 5 s의 r이 상한의 5 % 안, 놓을 때 적분기 ≤ ρ, 놓은 뒤 120 s 회전 ≤ 3°(벡터드).
    - 핀 없는 2520/2510은 그대로.
    - 벤치 지문 0.000 µs.
- 1d. posix 값과 문서.
  - 값: 2522, 2527, 그리고 측정되면 Cloudship 핀 기체.
  - 문서: '대기속도 스케일링 없음' 문장 6곳, 피토 설정, FF 측정을 두 속도로. 업스트림 문서는 짧고 사실만 씁니다.
  - 게이트: 예측 판정 통과, 비행 기체 값 변경 없음.
- 1e는 1c에 들어갔습니다(결정 1 = 복원 규칙 항상, 2026-10-07). 호버 비용은 sitl_check의 바람 사례가 그대로 잽니다(48/48 유지).

**P. 2차 준비** — 1차와 병행합니다. 모델과 도구만 바꾸고 펌웨어는 바꾸지 않습니다.
- P1. 기존 airship_fixed*에서 Manual 러더 계단을 2/3/4/6 m/s로 날려, 기준 속도의 표면 토크와 u² 법칙을 잽니다.
- P2. yaw_model에 표면 경로를 넣습니다: 크레딧 포트, 플랩 Δcl ∝ q·δ, 바람.
  - 게이트: P1과 airship_fixed_flight_check의 정점을 재현.
- P3. 목표 구조의 gz 기체를 만듭니다: 독립 틸트 포드에 같은 크기 '+' 플랩 4장. 값은 설계 도구에서 생성하고, 바뀌면 다시 생성합니다.
  - 서보: 플랩 Servo1~4, 틸트 Servo5~6. 배분기가 표면을 틸트보다 먼저 선언하기 때문입니다(cpp:82-99).
  - 질량을 다시 맞추고 등록합니다: 기체 번호, CMake 표, lta.sdf Buoyancy, 문서.
  - 도구 4종을 갱신합니다.
  - 모델 서브모듈을 먼저 커밋합니다.
  - 게이트:
    - variant_check, first light, 틸트 검사.
    - 엔진에서 잰 플랩 권한이 파일 예측과 맞음.
    - 러더 열을 0으로 두면 1차의 고정 핀 결과를 재현.
    - HEAD 펌웨어 동작이 그대로.

**2. 2차 증분** — 움직이는 핀.
- 2a. 공유 함수(크레딧, 축 분류, 표면 권한)와 시험.
  - 게이트: 단위 시험, 제어기 identity.
- 2b. 배분기: CS_V, 구독, 축별 크레딧.
  - 게이트:
    - 하네스 12/12가 CS_V 0에서 IDENTICAL.
    - 새 세트(CS_V > 0 속도 대본, 오래된 토픽, 속도 없음 → 0, 2500형 불변)를 덤프해 검토.
    - 새 단위 시험.
- 2c. 제어기: 게이트 인자, 물리 판정, A.
  - 게이트:
    - P3 기체에서 제자리 판정, 2/4/6 m/s 전진, 모의 피토와 GNSS 상실.
    - 최고 속도 선회 중 속도를 잃어도 진동이 없음.
    - 두 번째 경우로 airship_fixed*에 CA_AIRSHIP_GRP 1(고정 추진기, A = 0.5)을 주고 교차 확인. 이 도구의 '루프 열림' 단정은 조정이 필요합니다.
    - 순항 포드 반전 횟수 기록.
    - 벤치 지문 0.000 µs.
- 2d. 문서, posix 값, 측정 카드.

**X. 조건부: 러더 우선 분할.** 2c의 반전 횟수를 보고 정합니다(남은 결정 3).

**그다음 H(헤딩 유지).** 1차의 상한을 그대로 씁니다. 루프 입력 제한이 H의 설정점에도 걸립니다.

**실기체 측정 카드** (펌웨어와 별개)
- 잴 것: 호버와 두 전진 속도의 유지 선회(러더를 켜고/끄고), 놓은 뒤의 감쇠, 바람 없는 공기에서 피토 대 대지속도, 틸트 슬루, 추력 곡선.
- 이 값이 들어오기 전까지 비행 기체의 k와 CS_V는 0입니다.

## 5) 검증 계획

**단위 시험** — uORB 없는 헤더에 gtest를 붙입니다.
- 선택기: NaN, 새 메시지가 없어도 나이가 드는지, confidence 0, 음수, 마스크, heading_good과 dead_reckoning, 감김과 후진, 정지·2·4 m/s의 Δq 구간, 교집합과 버림, 히스테리시스, 멈추지 않는 감쇠, 루프가 닫혀 있을 때만 상한 유지.
- 스케줄: 꺼짐일 때 비트 동일, LO = HI, 세 항, ρ의 두 경우.
- 루프 입력 제한: 상한보다 큰 설정점이 잘리는지, 상한이 MAX일 때 입력이 그대로인지.
- 권한: 틸트 범위, 꼬리 방향.
- FF setter: P, I, 한계, 적분기를 건드리지 않는지.
- 크레딧: CS_V 0이면 CS_K 그대로, 속도 0이면 0, 단조, 축 분류.

**기능 시험** — AirshipAttitudeControlTest의 Run() 고정 장치에서 봅니다.
- 배선, 메시지가 오지 않을 때의 표본 나이, 상한이 rates setpoint에 반영되는지, Manual과 비수동 모드 불변, 모든 모드에서 발행.

**identity**
- 새 제어기 하네스를 스크래치에 만들고 커밋하지 않습니다.
  - 모드, 스틱, 자이로, 속도를 대본으로 줍니다.
  - 토크, 추력, rates setpoint, rate_ctrl_status를 16진 실수로 덤프하고 idcmp.sh로 HEAD와 비교합니다.
  - 필요한 이유: 지금의 하네스는 배분기만 보고, 벤치 지문은 Manual만 봅니다.
- 배분기 12세트는 매 단계 IDENTICAL이어야 합니다. 이 하네스는 updateSetpoint를 직접 부르므로 2차의 크레딧도 덮습니다. 2차는 실행 중 행렬 갱신을 쓰지 않습니다.

**yaw_model**: HEAD 동기, 전진 드라이버, 긴 유지, 정책 비교, 표면 경로, gate.py로 SITL 재현.

**SITL** — sim 시계, 진입 상태 비교.
- 회귀: sitl_check 48/48, airship_fixed_flight_check 13/13.
- 전진: 2522/2527/2512/2516/2517. 핀 없는 2520/2510은 그대로여야 합니다.
- 속도원:
  - GNSS.
  - SENS_EN_ARSPDSIM 모의 피토: 바람을 보지 못하고, 8 Hz, 1 Pa 잡음, 고장 주입 가능.
  - GNSS 상실.
- 바람:
  - 같은 벡터를 /world/lta/wind(enable_wind)와 /ocean_current에 함께 겁니다. 2522/2512/2501에서만 합니다.
  - 바람을 보는 피토 플러그인은 온도 문제를 먼저 확인합니다.
- 2차: P3 기체와 매개변수 혼합 기체.

**벤치** (v6x, 핀에서)
- 펌웨어 단계마다 사용자가 build_fmu.py가 준비한 이미지를 올리고, Manual 지문 0.000 µs를 확인합니다.
- airship_speed가 NONE으로 보이는지, perf 카운터와 크기.
- 선택: SDP3x를 달고 불어서 실제 드라이버 경로를 확인합니다. 임시 파라미터는 스냅샷 → 복원 → 검증합니다.

## 6) 하지 않는 것

- 배분기 행렬을 속도로 바꾸지 않습니다(틸트로터식 실행 중 갱신, 처짐 ∝ 1/q). 100 ms 제한, 저속 행 0 처리, Manual 처짐의 센서 의존이 생기기 때문입니다.
- 요 토크 전체에 고정익식 × s²를 곱하지 않습니다. 포드 몫까지 줄어들기 때문입니다. P·I의 속도 스케일도 하지 않습니다.
- 게이트를 속도에 따라 바꾸지 않습니다(비행 중 루프가 열리고 적분기가 지워짐). 표면만으로 요 루프를 닫지도 않습니다.
- airspeed_selector 빌드, EKF2 변경, 고정익 게이트 넓히기, 바람 추정(A)은 하지 않습니다.
- VTOL식 두 할당 인스턴스는 쓰지 않습니다. X의 대안으로 기록만 합니다.
- 비행 기체 기본값과 기존 파라미터 이름은 바꾸지 않습니다.
- lib/rate_control은 setIntegral 하나(복원 규칙)만 더합니다. 다른 것은 바꾸지 않습니다.
- H는 D1 다음입니다.
- 기준 설계 저장소는 고치지 않습니다.

## 7) 위험과 검증되지 않은 것

1. 결함은 0단계에서 SITL로 측정했습니다(step0_report_20261002.md: 2527/2522/2512, 놓은 뒤 최대 191°). 정책의 효과는 아직 모델 결과이고, 1차 증분의 SITL이 처음 잽니다.
2. 구조가 보장하는 것은 구간 안의 속도 오차까지입니다.
   - k 자체가 틀리면 적분기가 그 차이를 한계까지 들고, 놓은 뒤 I/Ki 드리프트로 남습니다. 최대 191°로, 오늘과 같은 상한입니다.
   - 이 오차는 로그에 바로 보입니다. 값과 무관한 상한은 복원 규칙이 둡니다(결정 1).
3. 러더는 같은 모멘트에서도 포드보다 덜 돌립니다. 4 m/s 전타에서 강체 0.00, 부가질량 2.7 °/s입니다(airship_tail_design.md:108-112).
   - 크레딧은 모멘트만 맞춥니다. 그래서 2차의 k는 러더를 켠 상태에서 잽니다.
4. 속도를 높게 읽으면 크레딧이 과해져 포드 몫이 줄고, 결손이 숨습니다. Manual에도 해당합니다.
   - 하한을 쓰는 것과 교집합 검사가 이를 막습니다. 피토만 쓰는 구성은 여전히 노출됩니다.
5. 속도를 낮게 읽거나 잃으면 과다 전달이 됩니다.
   - 이득은 최대 1+(u/CS_V)²입니다.
   - 구간이 넓으면 놓은 뒤 되돌이가 생길 수 있습니다.
   - 실외에서 대지속도만 쓰면 선회가 느립니다.
6. 순항 트림 추력이 작습니다(2527, 4 m/s에서 0.12, d1 :250). 요 요구가 이를 넘으면 포드가 약 1.5 s 동안 돌아갑니다.
   - 반전은 부족으로 보고되지만 과도는 남습니다.
   - CS_V 위에서는 러더가 (u/CS_V)²만큼 크레딧보다 더 냅니다.
7. 포드와 꼬리의 속도 무관성은 gz에서만 정확합니다. 그래서 A를 공유 함수 한 곳에 둡니다.
   - gz 핀은 러더 처짐으로 실속하지 않습니다.
   - _am 기체의 Munk는 바람을 무시합니다.
   - 피토 플러그인의 TAS 약 14% 저하는 코드만 읽고 판단했습니다.
   - PX4_PARAM_ 전달은 확인하지 않았습니다.
   - 크레딧은 TAS로 계산하므로 밀도 차(수 %)만큼 틀립니다.
8. 배분기 안의 속도 입력은 병합된 PX4에 없는 일입니다. 매 주기 발행의 CPU·플래시 비용은 아직 재지 않았습니다. 목표 구조의 gz 기체는 P3 전에는 없습니다.

## 8) 남은 결정

remaining_decisions_ko에 적었습니다.

## 남은 결정

- [결정 2026-10-07, 0단계 결과] 놓은 뒤 드리프트 안전장치는 복원 규칙을 항상 씁니다(SCHED+RESTORE). 근거: 0c에서 스케줄만은 k ±30 %·2배와 실외 속도원 오차에서 최대 191°를 남겼고, 복원 규칙을 더하면 모든 조건에서 2.5° 이내였습니다. 비용(선회 중 트림 변화 시 약 32°/5 %)은 받아들입니다. 09-28의 호버 결정(FF)은 그대로이고 그 위에 더합니다. 2517의 전진비행 방향 불안정은 D1 밖의 별도 항목으로 둡니다(후속).
- CS_V > 0일 때 Manual에도 같은 속도 크레딧을 쓸지(권고: 씁니다). 어느 쪽이든 표면 처짐은 센서와 무관합니다. 쓰면 순항 Manual에서 포드 몫이 줄어 포드 반전이 적습니다. 대신 속도를 높게 잘못 읽으면 저속 Manual 요가 줄어듭니다. 쓰지 않으면 배분기가 모드를 읽어야 하고, 순항 Manual에서 포드가 전부를 맡아 반전이 많아집니다.
- 러더 우선 분할(X)을 할지: P3 기체에서 잰 순항 포드 반전 횟수를 보고 정합니다. 하면 표면 처짐이 속도에 따라 커지므로, Manual을 어떻게 둘지도 함께 정해야 합니다. 업스트림 선례가 있는 대안은 VTOL식 두 할당 인스턴스입니다.
- 2차를 업스트림에 언제, 어떤 형태로 낼지: 배분기 안에 속도를 넣는 것은 병합된 PX4에 선례가 없습니다. 받아들여지지 않으면 쓸 제어기 쪽 대안(혼합 플랜트의 1/g(u) 이득 스케일)을 기록해 둡니다.

## 핵심 주장 검증

- [유지] The yaw integrator's only stop besides the ±AS_YR_INT_LIM clamp is the sign of control_allocator_status.unallocated_torque[2], ignored when torque_setpoint_achieved is set (src/modules/airship_att_control/airship_yaw_rate_loop.hpp:76-82); RateControl clips the rate error to one sign only on those flags and clamps the integral to ±lim_int (src/lib/rate_control/rate_control.cpp:91-99, 115).
- [유지] AS_YAWRATE_FF reaches RateControl only through AirshipYawRateLoop::setGains() (airship_yaw_rate_loop.hpp:58-69), called from parameters_updated() (src/modules/airship_att_control/airship_att_control_main.cpp:101-106); RateControl::setFeedForwardGain is an inline setter (src/lib/rate_control/rate_control.hpp:72) that FixedwingRateControl calls every cycle with an airspeed-scaled gain (src/modules/fw_rate_control/FixedwingRateControl.cpp:389-392).
- [반박] The stick's full-scale yaw rate is only the max_rate argument passed at airship_att_control_main.cpp:206-207 to yawRateSetpoint() = deadzone(stick)·max_rate (src/modules/airship_att_control/airship_manual_input.hpp:91-94); the Ackermann rover applies its speed-dependent yaw-rate limit at the equivalent point, before its PID (src/modules/rover_ackermann/AckermannRateControl/AckermannRateControl.cpp:71-74).
  - 정정: The stick's full-scale yaw rate is only the max_rate argument, radians(AS_YAWRATE_MAX) (airship_att_control_main.cpp:106). It is passed at airship_att_control_main.cpp:206-207 to yawRateSetpoint() = deadzone(finiteOr(stick, 0), MAN_DEADZONE)·max_rate (airship_manual_input.hpp:91-94), and full stick gives ±max_rate (Functions.hpp:127-132). Nothing downstream clamps it (airship_yaw_rate_loop.hpp:96-97, rate_control.cpp:71-86).

The Ackermann rover keeps its stick map independent of speed, with full scale RO_YAW_RATE_LIM (AckermannManualMode.cpp:52, 81-83, 111-113, 175-177). Its speed-dependent limit is a separate clamp at the input of its rate controller. It applies to every incoming rover_rate_setpoint, before the slew, feedforward and PID (AckermannRateControl.cpp:71-74, 142-146, 76-96).

The matching airship point is the yaw_rate_sp handed to _yaw_rate_loop.update() (airship_att_control_main.cpp:208). A constrain() there, with max_rate left at AS_YAWRATE_MAX, follows the rover precedent. Scaling max_rate by speed is a different design: it rescales the whole stick instead of saturating it, and it limits only the stick path.

The rover's speed is estimated from the throttle command (AckermannRateControl.cpp:133-139), and its bound goes to zero at zero speed (68, 104-110). The precedent therefore supports where the limit goes, not its speed source or its shape.
  - 반영(2026-10-01): 상한을 스틱 최대값과 루프 입력 제한 두 곳에 둡니다(2.다, 1c). 입력 제한만 쓰면 순항에서 스틱 대부분이 죽고, 스틱만 쓰면 H가 상한을 받지 않습니다.
- [유지] hasPropulsiveYaw() returns (pod_grouping > 0 || tail) && !(yaw_surfaces && surface_credit >= 1.f) (airship_manual_input.hpp:183) and is evaluated only on a parameter update from the allocator's parameters (airship_att_control_main.cpp:118-120), so the loop gate is a configuration-time copy of the allocator.
- [유지] The surface credit is the single scalar CA_AIRSHIP_CS_K read in updateSetpoint() (src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessAirship.cpp:174) and subtracted on all three torque axes (:179-181); the saturation report uses 1 minus the same parameter (:305); updateSetpoint() runs on every allocation right after allocate() (src/modules/control_allocator/ControlAllocator.cpp:439-442).
- [유지] ActuatorEffectivenessAirship declares pods, tail and tilt servos with zero effectiveness so only surfaces carry matrix entries (ActuatorEffectivenessAirship.cpp:73-99) and returns false on NO_EXTERNAL_UPDATE (:69-71); a runtime matrix change would be rate-limited to 100 ms (ControlAllocator.cpp:482-483) and have rows whose entries are all at most kMinEffectiveness 0.05 zeroed (ControlAllocator.cpp:591-602; src/lib/control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp:87).
- [유지] Pseudo-inverse allocation is actuator_sp = trim + mix·(control_sp − control_trim) (src/lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp:192), so a surface whose torque column is all zero stays at its trim.
- [유지] Pod yaw is half the difference of the two pods' forward forces (ActuatorEffectivenessAirship.cpp:161) and pod thrust is clamped at >= 0 (src/modules/control_allocator/VehicleActuatorEffectiveness/AirshipPod.cpp:189), so independent pods whose tilt range cannot point backward reach at most 0.5 normalized yaw, while a ±180° range reaches 1.
- [유지] AS_YR_INT_LIM may be set up to 1.0 (src/modules/airship_att_control/airship_att_control_params.yaml:36-38) and AS_YAWRATE_FF is capped at 10 (airship_att_control_params.yaml:53-55).
- [반박] Only airship_fixed and airship_fixed_am carry control joints (4 control_joint_name each in Tools/simulation/gz/models/airship_fixed*/model.sdf); airship_vectored_independent_fin, airship_vectored_independent_am_fin, cloudship_fin and cloudship_am_fin each carry 4 LiftDrag plugins and no control joint, so no gz plant combines propulsive yaw with movable fins.
  - 정정: Movable flaps (4 LiftDrag control_joint_name, 4 servo-driven flap joints on servo_0..3) are in airship_fixed and airship_fixed_am, and through merge includes also in airship_fixed_drag and airship_fixed_am_drag. airship_vectored_independent_fin, airship_vectored_independent_am_fin, cloudship_fin and cloudship_am_fin (and their _drag children) carry 4 fixed LiftDrag plugins on base_link with no control joint, and their joint controllers drive only the pod tilt. So no gz plant combines tilt-vectored pods or a tail thruster with movable fins.

A gz plant does combine propulsive yaw with movable fins: the airship_fixed family has two independently commanded forward thrusters at y = -1.5 m and +1.5 m. Setting CA_AIRSHIP_GRP 1 (tilt range kept at 0, CA_AIRSHIP_CS_K < 1) gives differential-thrust yaw plus flap rudders and closes the yaw-rate loop on a pod+rudder plant in SITL with parameter changes only. Only the shipped 2500/2501/2504/2505 posix airframes (CA_AIRSHIP_GRP 0) leave that plant without propulsive yaw. Its fixed pods do not reverse, so the plant's at-rest yaw differs from the vectored pods (half the couple plus forward thrust at zero throttle).
  - 반영(2026-10-01): P1과 2c의 교차 확인이 이 조합(airship_fixed*, CA_AIRSHIP_GRP 1)을 씁니다. 목표 구조(틸트 포드 + 움직이는 핀)는 여전히 P3의 새 기체가 필요합니다.

## 심사 점수

- universality: controller — Speed-blind allocator, spee 6, allocator — Physical-torque allocation:  5.5, staged — D1 increment 1 in the controlle 7
- px4fit: controller — Speed-blind allocator, spee 6, allocator — Physical-torque allocation:  4.5, staged — Staged D1: damping schedule in  7.5
- delivery: staged 8, allocator 6, controller 4.5
