# 첫 upstream PR 제안 (2026-10-13 14:20, 교수 역할; 사용자 결정 대기)

## 1. 현 상황

**upstream (PX4 main a4697ed08e).** 비행선은 에어프레임 둘뿐이다: 2500 Generic Airship (고정 추진기 둘 + 엘리베이터 둘 + 러더 둘,
`@maintainer` 는 사용자) 과 2507 Cloudship (일체 틸트 + 꼬리). 둘 다 `CA_AIRFRAME 9` (Custom) 로 돌고, 비행선 전용 배분기는 없다.
`airship_att_control` 은 216 줄 스틱 패스스루이고 사용자의 #28739 (피치 토크 부호) 가 머지돼 있다. gz 비행선 시뮬은 없고,
`rc.airship_defaults` 는 두 줄 (VEHICLE_TYPE, MAV_TYPE) 이다.

**우리 브랜치 (airship/dev, upstream 위 178 커밋, 포크에 미푸시 38).** 틸트 포드·일체 틸트·꼬리·조종면 크레딧을 다 다루는 배분기
(`CA_AIRFRAME 16`), 요 각속도 PI+FF 루프, 전진 속도 스케줄, 메시지 둘, 파라미터 17 개, gz 가족 7 종 × 물리 변형 8 (모델 52, 에어프레임 40),
문서. 검증은 **시뮬과 벤치뿐**이다: fmu-v6x 핀 지문 (2500 은 upstream Custom 9 대비 0.0 us, 2520 프리셋), 호버·전진 게이트 (2520 10/10,
2517 9/9, 2527 42/42, 2512 43/43). **비행은 없다**: 2 m 실내 기체는 아직 만들지 않았다.

**도구 레포 (linearust/px4-airship-tools).** 생성기 (가족, `--upstream` 완전 플랜트, `--overlay`), 검사·게이트, yaw 모델, 설계·결정 기록.
분리 설계 1~3 단계가 끝났다 (오버레이 = upstream 이 안 가질 것의 집).

**판단의 축.** upstream 리뷰어가 보는 것은 셋이다: 실제 기체가 날았는가 (로그), 한 자리에서 이해되는가, 누가 유지하는가. 지금 단계에서
비행 로그는 없으므로, 비행 없이도 정확히 검증되고 설명되는 것만이 보수적이다. 배분기·요 루프·스케줄은 2 m 기체의 비행 뒤로 미룬다.

## 2. 선정: **Generic Airship (2500) 의 gz SITL 쌍둥이와 lta 월드**

한 문장: upstream 이 이미 가진 기체 (2500, 사용자가 유지자) 에 시뮬 쌍둥이를 준다. 펌웨어 코드는 한 줄도 바꾸지 않는다.

### 왜 이것인가

- **비행 없이 완결된다.** 검증은 시뮬 안에서 끝나고, 리뷰어도 `make px4_sitl gz_airship_fixed` 로 재현한다.
- **기존 사용자 영향 0.** 비행 에어프레임·배분기·컨트롤러를 건드리지 않는다. posix 에어프레임은 upstream 의 2500 (Custom 9) 을 source 하고
  gz 매핑만 더한다.
- **정확히 말할 수 있다.** 모델의 수치는 문헌과 엔진 측정에서 왔고 (부가질량 9/9, 공력 모멘트 14/14, 핀 감쇠 0.07 %), 2500 의 액추에이터
  매핑은 핀에서 upstream 과 0.0 us 로 같다는 지문이 있다.
- **뒤의 모든 PR 의 바닥이다.** 배분기·요 루프 PR 은 시뮬 결과로 설명해야 하는데, 그 시뮬이 upstream 에 먼저 있어야 한다.
- **유지자 이야기가 선다.** 2500 의 `@maintainer` 가 그 기체의 시뮬을 낸다.

틸트 포드 배분기를 먼저 내는 안은 비행 로그가 없어 보수적이지 않고, 공중 무장 해제 수정 (COM_DISARM_LAND) 은 10 줄이라 주제가 아니다
(언제든 곁들인다, 3 절 끝).

### 내용 (최소)

| 레포 | 파일 | 내용 |
|---|---|---|
| PX4-gazebo-models (M1) | `models/airship_base/` | 몸체: 선체·곤돌라·꼬리·센서, 부력 체적, `fluid_added_mass` (오늘의 `airship_base_am` 내용, 이름만) |
| | `models/airship_fixed/` | 2500 의 쌍둥이 완전 플랜트: 몸체 merge + 고정 추진기 둘 + 핀 넷 (플랩 조인트 = 엘리베이터·러더) + 양력·항력 (`--upstream` 이 만드는 두 단 모양) |
| | `worlds/lta.sdf` | 중력, 균일 유체 밀도, Buoyancy enable 목록 (airship_fixed 와 _0.._3), 시스템 목록 (Magnetometer 에 tesla·NED 플래그 포함) |
| PX4 (P4 의 일부) | `init.d-posix/airframes/2500_gz_airship_fixed` | `. 2500_generic_airship` (Custom 9) + SIM_GZ_EC/SV 매핑, SV_MINA/MAXA = 플랩 행정 ±8°. AS_* 요 파라미터 없음 (upstream 모듈에 없다) |
| | `init.d-posix/airframes/CMakeLists.txt` | 항목 한 줄 |
| | `docs/en/sim_gazebo_gz/vehicles.md`, `index.md`, `frames_airship/index.md` | 항목 하나, 행 하나, 불릿 하나 (≤ 3 문장, make 명령, 스크린샷) |
| | `Tools/simulation/gz` 포인터 | M1 로 |

**넣지 않는 것:** 가족·물리 변형·Cloudship gz·문헌 가족 (오버레이), 배분기 16, 요 루프, 스케줄, 메시지, AS_* 파라미터, 2500/2507 의 이전,
생성 표식, 십의 자리 표, 마스크 규칙 문서.

### 열린 질문 (PR 전에 정한다)

1. **모델 이름.** 오늘 `airship_fixed`. upstream 의 기체 이름은 Generic Airship 이므로 `generic_airship` 이 더 자연스럽다. 이름을 바꾸면
   생성기·검사의 `GENERIC`/`fixed` 역할 이름과 오버레이 쌍둥이 검사가 따라간다 (하루).
2. **플랩 행정 ±8°.** 생성기의 설계값 (R3 실속 여유). upstream 2500 은 행정을 말하지 않는다 (CA_SV_CSn_TRQ 0.5 만). 쌍둥이의 ±8° 는 모델의
   것이고, 리뷰어에게는 "조종면 효과 계수 0.5 의 근거는 없고 (2019 의 추정), 쌍둥이는 그 값으로 날며 다음과 같이 응답한다" 로 적는다.
3. **월드.** upstream 은 월드마다 시스템 목록을 두지 않는다 (server.config). Buoyancy 는 enable 목록이 월드에 있어야 하므로 lta 월드가
   필요하다. 이 설명과 Magnetometer 플래그의 이유 (legacy 모드 경고) 를 PR 본문에.
4. **플러그인 의존.** Buoyancy·Hydrodynamics·LiftDrag 는 gz-sim 의 표준 시스템이다 (우리 플러그인 없음). 명시한다.

## 3. 실행 계획 (사용자 승인 뒤; 1~2 일)

1. **PR 브랜치를 upstream main 에서 뜬다** (worktree). 모델 레포: `airship-sitl` 브랜치에 모델 둘 + 월드. PX4: `airship-sitl` 브랜치에 posix
   에어프레임·CMake·문서·포인터. 모델 둘은 생성기 `--upstream` 의 출력 (쌍둥이 검사 통과 23/23) 에서 가져오고, 머리 주석은 upstream 밀도로.
2. **posix 에어프레임을 Custom 9 위에 쓴다.** 포크의 2500 gz 에어프레임에서 CA_AIRFRAME 16 블록을 빼고 upstream 2500 을 source.
   액추에이터 순서 (Motor1/2, Servo1..4 = CS0..3) 가 모델의 토픽 순서와 같음을 확인 (핀 지문의 근거와 같은 매핑).
3. **upstream 기반 빌드에서 검증한다** (PX4_ROOT = PR worktree): (a) 첫 빛: 중성 부력, 표류 없음; (b) 패스스루 비행: 스로틀 → 전진, 엘리베이터 →
   피치, 러더 → 요 (`airship_fixed_flight_check.py`, 기록 fixed_flight_20260925/0927 의 기준선을 Custom 9 에서 다시); (c) px4 로그에
   "legacy mode" 경고 없음. 결과 수치를 PR 본문에 그대로.
4. **PR 본문** (영어, `docs/upstream_pr_bodies.md` 의 M1 절을 다듬어): 무엇을 더하나, 왜 월드가 따로인가, 수치의 출처, 재현 명령, 스크린샷.
   모델 레포 PR 먼저, 머지된 포인터로 PX4 PR.
5. **곁들일 10 줄 PR (별도, 언제든):** `rc.airship_defaults` 에 `COM_DISARM_LAND -1` — AirshipLandDetector 가 Land 모드를 고르는 순간 "착륙" 이라
   하므로 2 s 뒤 공중에서 무장 해제된다. 재현은 우리 gz 로 했고 (10/10), 본문에는 코드 경로로 설명한다.

## 4. 이 선택이 바꾸는 것

- 분리 설계의 upstream 쪽은 "가족 셋" 이 아니라 **2500 쌍둥이 하나** 로 시작한다. 벡터링 완전 플랜트 (`airship_vectored_independent`) 와
  Cloudship 은 배분기·기체가 upstream 에 간 뒤의 다음 PR 이다. 설계 문서 6 절의 표를 그렇게 고친다.
- 틸트 포드 배분기는 "독립 포드 둘" 로 좁히는 안 (14:00 의 제안) 과 함께 2 m 기체 비행 뒤로 간다. 그때까지 코드 삭제는 하지 않는다 (시뮬 검증
  장치로 계속 쓴다).
- 도구 레포는 그대로 (오버레이에 모든 가족).
