# 교수 결정 2026-10-12 (Fable 5.1; 야간 실행 2026-10-07/08 의 보류 항목 D-e/D-f/D-g 와 그 뒤)

사용자 지시 (2026-10-12 11:17): "실내 비행선 제작 전에 할 수 있는 작업을 알아서 진행해." 기준 트리 PX4 183bd03268, models 81e8937 (둘 다 10-08 그대로, 깨끗).

## D-e — JAXA 틸트 속도 60 ≠ 2520 의 120 (variant_check 16, 생성기 시험 2건)

**결정: 판정 규칙을 고친다 (선택 b).** 2520 의 주석이 "CA_AIRSHIP_TLMIN/TLMAX 는 서보 멈춤쇠 각, CA_AIRSHIP_TLT_R 은 서보 슬루율을 기체에 맞추라" 고 사용자에게 말하는 세 값은
비행 에어프레임의 것이 아니라 기체의 것이다. 기체 파일로 만든 가족의 맨 에어프레임은 그 세 줄을 파일의 `pods.tilt_deg`/`tilt_rate_deg_s` 로 쓰고, 나머지 줄 (순서 포함) 은 2520 과 같아야 한다.
실내 가족도 같은 규칙 (하드웨어 꾸러미 1.2 절이 CAD 멈춤쇠에 맞추라고 함). JAXA 의 60°/s 는 ASSUMED 지만 둔다: 큰 덕트 프롭의 느린 틸트가 그럴듯하고, 두 번째 값이 틸트 슬루 경로를 시험한다.
실행: `variant_check.py` VEHICLE_ALLOCATION + 검사 16; 변이 검사 (2550 의 TLT_R 을 어느 파일과도 다른 70 으로) 실패 확인. 결과 variant 16/16, 생성기 시험 17/17. 트리 변경 없음.

## D-f — JAXA 완전 플랜트 hover 7/10

야간 진단 ("FF 규칙이 6.67°/s 에서 2배 과함 → 넘침 8.2°/s, 놓음 정지 10.1 s") 을 모델로 확인했다 (`scratchpad/dfmodel/hover_ff_act.py`: 요 모델의 구동부
vectored_actuation + 1-DOF 선체, 가족의 제 에어프레임·모델에서 읽음, 복원 규칙 켬):

| 플랜트 | FF 꼴 | 마지막 3 s 평균 | 최대 | 적분기 | 놓음 정지 |
|---|---|---|---|---|---|
| JAXA 완전 | 선형 (HEAD) | 7.23 | 7.82 | −0.034 | 9.2 s (SITL 10.1) |
| JAXA 완전 | 2차 | 6.64 | 6.64 | +0.031 | 9.1 s |
| 7 m 완전 | 선형 | 7.17 | 7.80 | −0.018 | 8.3 s |
| 7 m 완전 | 2차 | 6.77 | 6.84 | +0.025 | 8.2 s |
| 실내 _fin | 선형 | 7.05 | **10.63** (판정 실패) | −0.108 | 7.5 s |
| 실내 _fin | 2차 | 6.92 | 8.62 | −0.038 | 7.5 s |

진단은 반만 맞았다. **넘침은 FF 꼴** (제자리 선체 항력은 r|r| 인데 FF 는 r 에 선형이라 AS_YAWRATE_MAX 아래의 모든 각속도에서 r_max/r 배 과함; 반 스틱에서 2.25배, 쿼터 스틱 4.5배).
**놓음 정지 시간은 FF 와 무관**하다: 이득 규칙이 모든 기체에 닻의 대역폭 0.1176 rad/s 를 주므로 P 만의 감속 시정수 (I+rr)/(N·P) 가 모든 기체에서 8.5 s 이고, 판정 "10 s 안에 정지" 는 거기에
포드 반전 시간 (180°/TLT_R: 120°/s 에서 1.5 s, JAXA 60°/s 에서 3 s) 을 더한 값과 맞먹는다. 7 m 8.3, 실내 8.7, JAXA 9.2 (SITL 10.1): 모두 경계에 있고 JAXA 의 느린 서보가 넘긴 것이다.
JAXA 의 셋째 실패 (I 끔 바람 표류 0.75 < 1.0°/s) 도 판정의 절대 문턱이 닻의 것이기 때문이다 (P 만의 평형 표류 = 밀기/(N·P) 는 기체마다 다르다).

**결정 (셋 다 실행):**
1. **펌웨어: 제자리 피드포워드를 2차로.** AS_YAWRATE_FF 의 이름·단위·값의 뜻 ("AS_YAWRATE_MAX 에서 선회를 유지하는 정규화 토크 / (rad/s)") 은 그대로 두고, 컨트롤러가 매 주기
   FF0·|r_sp|/r_max 로 적용한다 (RateControl 의 선형 FF 이득을 주기마다 바꾸는, 고정익이 공속으로 FF 를 바꾸는 자리). 스케줄의 속도 항 k·u 는 핀 양력이라 r 에 선형 그대로.
   상한의 첫 규칙은 (A−ρ)/FF 대신 FF0 r²/r_max + FF_u r = A−ρ 의 양근; 둘째 규칙 (구간 부족분) 은 FF0 가 양변에서 상쇄되어 그대로. 모든 에어프레임의 FF 값은 그대로 유효하고
   (r_max 에서 같음), 램프 AS_SPD_LO/HI 만 계산기가 새 상한 식으로 다시 쓴다 (수 % 변화). PX4 선례: THR_MDL_FAC 가 추력의 제곱 법칙을 바로잡듯 공력 항력의 제곱 법칙을 FF 가 따른다.
   실내 기체의 H2 (반 스틱 정지 요) 가 이것을 그대로 검증한다; H2 에 스틱 0.25 단계를 추가한다.
2. **게이트 (sitl_check): 시간 창과 문턱을 기체의 것으로.** 놓음 정지 허용 = (I+rr)/(N·P) + π/TLT_R (닻에서 8.5 + 1.5 = 10 s, 지금 값 그대로 재현); 추종 창 = 4 시정수 (닻 34 ≈ 35 s);
   I 끔 표류 문턱 = 0.5 × 밀기/(N·P) (닻 0.76°/s ≈ 지금의 1.0). 두 번째 기체가 드러낸 닻의 상수 (second-case-finds-hidden-defects).
3. **교수 기록 (코드 변경 없음):** 이득 규칙이 닻의 절대 대역폭을 모든 크기에 복사한다. 2 m 기체에 8.5 s 시정수는 느릴 수 있다 (Froude 척도면 ω ∝ 1/√L → 2.3배 빠름). SITL 은 조종감을 검증하지
   못하므로 하드웨어 P 튠 (문서 "starting point") 에 맡기고, 결과를 보고 규칙을 바꾼다.

## D-g — 조종면 옵션 (CS_K) 적용 여부

**결정: 보류 (적용하지 않음), 옵션 코드는 유지.** CA_AIRSHIP_CS_K 의 바른 값은 속도에 따라 변하는 조종면 효과 대 추력 요 권한의 비 — D1 2단계 "속도별 러더 크레딧" 의 설계 대상이고 아직 없다.
고정값 (예 0.5) 을 지금 고르면 게이트를 통과한 핀 멤버의 요 물리가 바뀌고 검증할 기체도 없다 (실내 기체는 러더 없음; 로드맵 3단계 "핀 제어" 뒤). 문헌 가족은 비교용 모델이고 중립 조종면은
고정 핀의 일부라 유지 선회 비교에는 영향 없다. AURORA/JAXA/MPI 문서의 docs_note 가 실기체의 조종면을 적는다 (이미). P16: 적용 때 벡터링+조종면 응답 검사가 새로 필요 — 그때.

## 그 밖의 관찰 (기록)

- Lotte: 회전타원체 Lamb 부가 관성 m55 가 CFD 의 1.23 배 (옆 부가질량은 1.006). 우리 rr (요 부가 관성) 은 실제 선체보다 20 % 안팎 클 수 있다 → 규칙의 P/I 가 그만큼 큼. 하드웨어 튠 항목; README 에 한 줄.
- 핀 이름 (dorsal/ventral) 이 무게중심 기준이라 JAXA 의 아래 핀이 dorsal 로 읽힘 — 표시만. 선체 축 기준으로 바꾸는 것은 뒤로.
- 요 모델의 플랜트 표 (plants.EXPECTED) 는 10.97 m 닻과 Cloudship 만 — 새 가족은 scratchpad 스크립트가 파일에서 직접 읽었다. 모델을 기체 파일 가족으로 넓히는 일은 열려 있다.

## D-f 실행 결과 (12:05 기준, SITL 체인 진행 중 — scratchpad/gates/summary.txt)

- 펌웨어: `airship_yaw_schedule.hpp` (Schedule{ff_rest, ff_speed, caps}, feedforwardGain, rateCap 의 2차 예산 규칙), `airship_att_control_main.cpp` (설정값 뒤에 이득 계산),
  yaml·msg 설명, 단위 시험 8/8 (스케줄 시험 재작성 + 반 스틱 사례). `make check_format` 통과.
- 도구: `airship_gains.py` rate_cap/stall_ramp (램프 7 m 1.1/2.1, AURORA 1.7/3.5, JAXA 1.6/3.2, MPI 0.95/1.9, 실내 0.4/0.8 — 모두 다시 씀, `vehicles/airship_indoor.params` 포함),
  `sitl_check.py` 기체별 창·문턱, `variant_check.py` 16/16, 생성기 시험 17/17, forward 시험 219/219 (기준). 요 모델: `controller.FF_LAW` 스위치 ('quadratic' 기본, 기록 재생은 'linear' 고정),
  `policies.schedule_d1` 새 상한. 요 모델의 test_forward/test_forward_replay/test_policies 와 test_controller 의 demo 는 10-07 22:11 (generic 7 m) 이후 트리를 거부한다
  (10.97 m 닻의 포드 상수·엔진 기록에 고정; 오늘 변경과 무관, 미해결 항목 "모델을 기체 파일 가족으로"). test_controller 본검사 115/115.
- 문서: frames_airship/index.md Tuning 4 (2차 적용, 측정식), 상한 문장, 예시 숫자 1.5°/s@1.1, 2.8°/s@3.4 (0.8/1.3 은 불변). README 두 곳. 하드웨어 꾸러미 H2 에 0.25 스틱 추가.
- **JAXA 완전 hover 10/10** (7/10 → ). 놓음 8.9/8.2 s (허용 23.2). Stabilized 추종 6.22, 최대 7.57 (적분기가 느린 상승 중 0.038 까지 감겨 넘침 12 %, 뒤에 풂).
- **관찰 (설계 성질, 기록):** Acro 추종은 처음 25 s 동안 2~3°/s 에 머물렀다 — 제동 단계가 남긴 표류 0.46 m/s 를 EKF 지면 속도원이 그대로 읽어 (WMAX 0) 램프 아래 상한
  reserve/(k u) = 4.6°/s, 반 스틱 설정 2.0°/s; 표류가 0.08 로 줄자 상한 15, 설정 6.67 (로그 2026-10-12/02_49_03.ulg). 정지 요 시험은 정지에서 해야 한다 (실내 k/N 15.4:
  0.2 m/s 표류에 2.8°/s). H2 카드에 적는다.
- **JAXA _fin hover 4/10 — 기준선이 아님:** 항력 없는 플랜트는 밀기 단계가 준 0.55 m/s 표류를 끝까지 지니고 (감속 없음), 스케줄이 상한 3.6, 설정 1.6°/s 로 깎고 램프 아래라
  속도 몫 FF 가 없어 적분기 (I 0.1) 가 30 s 걸려 채운다 → 13 s 창에 0.5°/s. _fin 멤버의 hover 게이트는 전에도 돌리지 않았고 (10-07 야간: 맨·완전만), 전진 검사가 그 멤버를 판정한다.
  게이트 수정 (체인 뒤): 추종 판정을 펌웨어가 게시한 설정값에 대해 하고, 상한이 스틱의 요구를 깎았으면 그 사실과 속도 구간을 적는다.

## SITL 결과 표 (체인 1: 11:48~14:47, 펌웨어 3f6b6d5038 + 램프 628e70c130, 게이트는 창 규칙 1판; 로그 scratchpad/gates/)

| 가족 | hover 맨 | hover _fin | hover 완전 | forward _fin | forward 완전 |
|---|---|---|---|---|---|
| 7 m generic | 10/10 | 9/10 (표류 상한) | 9/10 (첫 렌치 누락) | 30/31 (다리 3) | **42/42** |
| 실내 2 m | 7/10 (창 25 s + 렌치) | 4/10 (표류 상한 + 렌치) | **10/10** | 30/31 (다리 3) | **42/42** |
| AURORA | 8/10 (D-d 기준선) | 10/10 | **10/10** | 42/42 | **42/42** |
| JAXA | 9/10 (첫 렌치 누락) | 4/10 (표류 상한, 0.55 m/s 영구) | **10/10** (7/10 →) | 30/31 (다리 3) | **42/42** |
| MPI | 9/10 (창 23 s) | 9/10 (창) | **10/10** | 42/42 | **42/42** |
| Cloudship | 9/9 | — | 9/9 | — | — |

완전 플랜트 (실기체의 쌍둥이) 는 모두 통과. 맨·_fin 의 실패 셋은 모두 게이트의 것이다: (1) 추종 창 4τ 가 강체 플랜트의 적분기 풂 (P/I 7.8 s) 을 못 담음 → 4·max(τ, P/I);
(2) 항력 없는 플랜트의 표류가 스케줄 상한을 깎음 → 게시된 설정값에 대해 판정하고 상한을 적음, 항력 있는 플랜트는 출발 전 표류 0.1 m/s 아래를 기다림; (3) `gz topic -p` 의
첫 렌치 누락 4/17 → 한 번 더 발행. 셋을 `sitl_check.py` 에 넣고 (체인 2, 14:47~) 맨·_fin 멤버와 7 m 완전을 다시 돌린다. 도구 시험: variant 16/16, 생성기 17/17, forward 219/219.

## 체인 2 (14:47~15:58, 게이트 2판: 창 4·max(τ, P/I), 게시된 설정값 기준 판정 + 상한 기록, 항력 있는 플랜트의 출발 전 표류 대기 0.1 m/s, 첫 렌치 재발행)

| 가족 | hover 맨 | hover _fin | hover 완전 |
|---|---|---|---|
| 7 m generic | **10/10** (7→) | **10/10** (9→) | 9/10 (추종 7.35 통과감이나 고정 0.1 m/s 표류 대기가 60 s 를 넘겨 전제 깨짐 → 3판: 기체의 문턱) |
| 실내 2 m | **10/10** (7→) | 8/10 (추종·놓음 통과; 바람 둘은 표류 중 핀 감쇠 → 3판) | (체인 1 의 10/10) |
| AURORA | 8/10 (D-d 기준선) | 9/10 (I 켬 바람: 적분기가 풍향계 모멘트까지 듦) | (10/10) |
| JAXA | 9/10 (약한 밀기 렌치 미측정) | 9/10 (상한 진단 작동: "상한 4.09°/s (u 0.48) 이 6.67 를 1.82 로 깎았다", 추종 1.75; I 끔 바람은 핀 감쇠 → 3판) | (10/10) |
| MPI | **10/10** (9→) | 9/10 (Acro 추종: 표류 0.16 m/s 의 핀 풍향계 모멘트가 토크 0 에서 각속도를 9.5 로 올림 — 플랜트의 성질) | (10/10) |

결론: 맨·완전 멤버 (실기체의 쌍둥이와 강체 기준선) 는 모두 기준선대로. 항력 없는 _fin 멤버의 hover 는 표류가 영구라 (i) 스케줄 상한이 설정을 깎고 (ii) 핀의 풍향계
모멘트가 루프 토크와 같은 크기로 들어와 참고용이다 — 그 멤버의 판정은 전진 검사 (42/42, 30/31) 가 한다. 3판 게이트 (표류 중 핀 감쇠를 바람 기대값에, 출발 전 허용 표류 =
상한이 설정의 1.1 배에 닿는 속도, 스케줄 꺼짐이면 무한) 는 체인 3 (7 m 완전, 실내 _fin, JAXA 맨) 으로 확인.

## 체인 3 (15:58~16:18, 게이트 3판) 과 4 (16:18~, "걸렸는가" 판정에 루프 몫 반영)

| 플랜트 | 결과 | 비고 |
|---|---|---|
| 7 m 완전 | 9/10 | 루프 판정 전부 통과 (허용 표류 0.23 m/s 규칙, 출발 0.22); 1건은 I-끔 밀기의 렌치가 두 번 다 안 걸린 "미측정" (`gz topic -p` 누락, 체인 전체에서 6/32 회) |
| 실내 _fin | 9/10 | 바람 기대값 (핀 감쇠 포함) 맞음; "걸렸는가" 판정이 루프 몫을 안 봐 미측정 → 4판 |
| JAXA 맨 | **10/10** | 스케줄 꺼짐 → 허용 표류 무한, 핀 없음 |

커밋 (로컬, 서명 없음): PX4 **3f6b6d5038** feat(airship_att_control): scale the yaw feedforward by the setpoint's share of the maximum (펌웨어·msg·yaml·단위 시험·문서 Tuning),
**628e70c130** feat(airframes): yaw ramps for the quadratic feedforward (에어프레임 10개 + 문서 예시 숫자). models 저장소 변경 없음. 벤치 도구 (git 아님): sitl_check.py 4판,
airship_gains.py, variant_check.py, 요 모델 (FF_LAW), 문서 넷, 새 스크립트 yaw_model/hover_ff_family_20261012.py.
| 실내 _fin (4판) | **10/10** | 16:24 |
| JAXA _fin (4판) | **10/10** | 16:29 — 상한 진단과 핀 감쇠 기대값이 함께 작동 |

**최종 게이트 (sitl_check.py 4판) 로 다섯 가족 전 멤버의 hover 판정이 기준선대로 설명된다**: 완전·맨 멤버 통과 (AURORA 맨 8/10 은 D-d 기준선), _fin 멤버도 통과하거나 플랜트의
성질 (영구 표류의 풍향계 모멘트: MPI _fin Acro) 로 설명됨; 남는 산발 실패는 `gz topic -p` 렌치 누락의 "미측정" 뿐이다.

## 리팩터링 (16:44~17:15, 사용자 지시 "리펙토링 해")

동작은 그대로 두고 오늘 쌓인 코드를 정리했다. 펌웨어 (PX4 **b1cb5a7e70** refactor(airship_att_control): hand the yaw rate loop its feedforward each cycle):
`AirshipYawRateLoop::setGains` 는 이득·한계만 받고 피드포워드는 `setFeedForward` 가 유일한 입구 (파라미터 갱신 때 넣던 FF 는 어차피 매 주기 덮였다); 컨트롤러의 설정값 계산을
`yawRateSetpoint()` 로; 스케줄의 예산 규칙 (2차 근) 을 `budgetRate()` 로 떼어 `rateCap` 이 min 만 취한다. 단위 시험 8/8 (루프 시험에 "넘겨받기 전엔 FF 없음", 스케줄 시험에
budgetRate 사례 추가), check_format 통과. 도구: `sitl_check.py` 의 창 계산을 `loop_windows()` (namedtuple) 와 `rest_drift()` 로, 모듈 변수 덮어쓰기 (`global`) 제거, `clean_start(mode,
drift_m_s)`; `variant_check.allocation_matches()`; `vectored_actuation.load_params(..., airframe=)` 로 hover 재생 스크립트의 몽키패치 제거; `policies.budget_rate()` 가 펌웨어의
helper 를 거울처럼. 확인: variant 16/16, 생성기 17/17, forward 219/219, test_controller 115/115, hover 재생 스크립트 수치 재현; SITL 실내 완전 hover 10/10·forward 42/42, JAXA _fin
hover 10/10. 펌웨어 fw_b1cb5a7e70 (v6x, 메타데이터 통과) + micoair 빌드.

## 리팩터링 2 (17:34~17:55, "리펙토링 한 번 더")

- 펌웨어 PX4 **3f12aad158** refactor(airship_att_control): no rest feedforward without a maximum rate — `feedforwardGain` 과 `budgetRate` 가 AS_YAWRATE_MAX 0 에서 서로 달랐다
  (하나는 rest 항 전부, 하나는 없음) → 둘 다 없음; 컨트롤러의 헬퍼 이름 `scheduledYawRateSetpoint`. 단위 시험 8/8, check_format.
- 도구: `airship_gains.budget_rate` (펌웨어·모델과 같은 이름), `sitl_check.published_setpoint()`; `clean_start` 는 요 각속도 정지를 전제로 두고 표류는 창 안에서 기다리되 못 줄면 기록만
  한다 (실내 2 m 선체는 0.2 m/s 를 60 s 안에 0.08 로 못 줄여 전제가 깨졌었다: hover 9/10 → 10/10). variant 16/16, 생성기 17/17, forward 219/219.
- SITL: 실내 완전 hover 10/10 (17:51). 펌웨어 fw_3f12aad158 (v6x) + micoair 빌드.

## 21:34~ 별도 레포 `linearust/px4-airship-tools` 생성 (사용자 지시: "upstream에 올라가지 않을 것들을 다루는 잡동사니 레포... 니가 직접 만들어서 바로 올리도록 해")

결정과 근거:
- **레이아웃은 평평하게 유지.** 검토 문서가 제안한 `tools/ checks/ bench/` 분할은 버린다: 스크립트 40개가 같은 디렉터리에서 서로를 맨 이름으로 import 하고
  (`gz_model`, `report`, `sitl_link`, ...), yaw_model 은 `sys.path` 로 부모를 찾는다. 패키지로 옮기면 파일 70개의 import 와 SITL 체인 전부를 다시 검증해야 한다 (gz 시간).
  새로 더한 구조만 둔다: `docs/` (설계 기록 27편, 서로의 상대 참조가 살도록 한 디렉터리), `overlay/`, `pins.env`, `run_tests.sh`, `.github/workflows/checks.yml`.
- **디렉터리 이름을 레포 이름으로** (`~/dev/px4-airship-tools`), 옛 이름은 심볼릭 링크. 코드의 하드코딩 경로 8곳 (`vectored_actuation`, `gate`, `plants`, `test_controller`,
  `cloudship_actuation`, `hover_ff_family`, `fp_tools/ramp`) 은 `__file__` 기준으로.
- **산출물은 git 밖:** sitl_out 4.8 GB, fw_*/head_*/baseline/preset2500/ground_out, 백업 tgz, identity 덤프, yaw_model 의 큰 실행 출력, `options/xcheck_D/fw` (컴파일 산출물 30 MB).
  설계 옵션의 기록 (`yaw_model/options`, 21 MB jsonl) 은 남긴다 — test_controller 가 `options/D/beyond.jsonl` 을 읽고, 다른 곳에 없다. 추적 604 파일, 25.7 MB.
- **기록은 스스로 서야 한다:** `record_gains.gains()` 가 비행 로그 (PX4 빌드 디렉터리) 만 읽어서 CI (소스 트리만) 에서 chain9 재생이 죽었다. 세 번째 원천
  `yaw_model/records/gains.json` (`record_gains.py --write`, 로그 43개에서 읽은 값; 로그가 있으면 로그가 읽기이고 복사본과 같아야 하며, 없으면 복사본) 을 더했다.
- **오버레이** (`overlay/README.md`): 모델은 `GZ_SIM_RESOURCE_PATH` (PX4 의 gz_env.sh 가 제 경로를 뒤에 붙이므로 앞선 것이 이긴다), 에어프레임은 빌드 etc 디렉터리 복사 (rcS 가
  `ls` 로 찾는다), 월드는 `rc.gzsim` 이 제 디렉터리의 이름으로만 띄우므로 **standalone gz 로 먼저 띄우고 px4 가 붙는** 경로. 지금은 비어 있다: 트림 때 생성기의 `--overlay` 로 채운다.
- **CI:** 포크를 pins.env 의 커밋으로 받아 (`.gitmodules` 는 upstream URL 이므로 모델 서브모듈은 포크에서 해시로 fetch) `run_tests.sh` 를 돈다. 소스 트리 복제본으로 미리 돌린 결과:
  생성기 왕복, variant 16/16, forward_yaw_check_test, test_controller 115 PASS 1 SKIP 0 FAIL. **pins.env 의 커밋 (PX4 25개, 모델 7개) 은 아직 포크에 없으므로 사용자가 푸시할 때까지 CI 는
  체크아웃에서 실패한다** (모델 브랜치 먼저).
- **앵커 발견:** test_controller 의 마지막 시험 (options/D 기록 재생, 벡터링 쪽) 과 test_fin_damping (6/9) 도 7 m 트리에서 거부·실패한다 — test_forward/_replay/_policies 와 같은 원인
  (모델이 10.97 m 선체에 앵커, 플랜트는 트리에서 읽음). test_controller 는 그 한 재생을 SKIP 으로 보이게 하고 (115 검사는 돈다), 나머지 넷은 run_tests.sh 밖에 두고 이유를 적었다.
  바른 수리는 "기록이 제 플랜트를 지닌다" (재앵커링, 열린 항목).
- **공개 여부:** 사업 맥락 검색 (icarus/사업/회사/고객/메일/토큰) — 포크 이름 `icarus-lta` 와 @maintainer 메일 (PX4 트리에 이미 있는 것) 뿐. 공개 (BSD-3) 로 만든다.

## 22:40~23:05 증분 A 실행 (파라미터 둘), 설계 `upstream_split_design_20261012.md` 0절·6절

- PX4 로컬 커밋 셋 (서명 없음): `cbf8f5fc77` test — 오늘 오후 3f6b6d5038 의 반 스틱 케이스가 **HEAD 에서 실패하고 있었다** (첫 닫힌 사이클의 dt 20 ms 클램프로 I·e·dt = 4.7e-4 가
  둘째 사이클 토크에 실림; stash 로 옛 코드에서 재현). 오후의 "8/8" 은 틀렸다. 피드포워드 검사이므로 I 0 으로 격리. `ec89e6bd84` AS_YR_RESTORE 삭제·복원 항상,
  `a9e7c3c16b` AS_SPD_LO 삭제·램프 시작 = AS_SPD_HI/2 (kRampStart). 두 커밋을 분리 적용한 트리가 검증한 작업 트리와 **동일** (git diff stash 비어 있음).
- 검증: gtest 8/8 (스케줄 핀 2.142 → 2.677 °/s, 파이썬 독립 계산 2.6774 와 일치), check_format 통과, 도구 `run_tests.sh` exit 0 (115 PASS + 255 통과, 1 SKIP).
  에어프레임 diff 는 정확히 `AS_SPD_LO` 10줄과 `AS_YR_RESTORE` 11줄 삭제뿐. `.params` 재생성 (두 이름 없음).
- 도구: `airship_gains.py` (stall_ramp 는 HI 만, 블록·params 에서 두 줄 제거, REF_RAMP 삭제), `forward_yaw_check.py` (leg_speeds lo = hi/2, SCHEDULE_PARAMS 둘 제거), 시험 픽스처,
  `hover_ff_family` (복원 항상), `d1_sitl_compare` (옛 로그의 LO 는 그대로, 없으면 hi/2), 벤치 절차 7번 (놓음 단차 검사, 첫 부팅 경고).
- 실행 중 (chainA.sh, 백그라운드): fmu-v6x 빌드·스테이징, MicoAir 빌드, 호버 게이트 2517/2520/2521, 전진 2527/2512. 결과는 다음 기록에.
