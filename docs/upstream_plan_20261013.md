# upstream 계획 (2026-10-13 14:50, 교수 역할; 전제: 실내 테스트 기체로 충분히 시험한 뒤 올린다)

`upstream_first_pr_20261013.md` (비행 전 전제의 "2500 시뮬 쌍둥이" 안) 을 대체한다.

## 1. 전제가 바꾸는 것

실내 기체 (2 m, 독립 틸트 포드 둘, 러더 없음, 광류, 바람 없음; 서보 Y 선으로 일체 틸트도 시험 가능) 가 비행 로그를 준다. 그러면 리뷰어의
첫 물음 "실제 기체가 날았는가" 에 답할 수 있고, 가장 타당한 주제는 **그 기체가 검증하는 것 그 자체** 가 된다. 기체가 검증하지 못하는 것은
그대로 upstream 에서 뺀다.

| 기체가 검증하는 것 | 검증하지 못하는 것 |
|---|---|
| 독립 틸트 포드 둘의 배분 (틸트 조향·슬루·멈춤쇠·비가역 클램프·조향 밴드) | 꼬리 추진기 (Cloudship 의 것) |
| 요 각속도 루프 (P·I·FF·복원 규칙·배분기 포화 피드백) 의 정지·실내 비행 | 조종면 크레딧 (핀 제어는 다음 증분) |
| 수동 패스스루, 공중 무장 해제 수정 | 전진 속도 스케줄 (핀이 있어야 뜻이 있다: 다음 증분) |
| 일체 틸트의 추력 투영 (Y 선; 요는 없음) | 문헌 가족·물리 변형 (검증 장치) |

## 2. 주제: **독립 틸트 포드 비행선 — 배분과 요 각속도 제어, 2 m 실내 기체로 비행 검증**

PR 셋이 한 주제다. 각각 작고, 같은 기체의 같은 로그로 설명한다.

| 순서 | PR | 내용 | 증거 |
|---|---|---|---|
| 1 | control_allocator: 틸트 포드 비행선 (`CA_AIRFRAME 16`) | `ActuatorEffectivenessAirship` 을 **독립 포드 둘** 로 좁힌 것 + `AirshipPod` (조향 밴드·슬루·멈춤쇠); 파라미터 `CA_AIRSHIP_TLMIN/TLMAX/TLT_R` 셋; 비행 에어프레임 2520; `rc.airship_defaults` 의 `COM_DISARM_LAND -1` (공중 무장 해제) | 벤치 핀 지문 (airship/dev 와 동일, SITL 쌍둥이와 동일), 틸트 멈춤쇠·슬루 로그, 수동 비행 로그 (Flight Review) |
| 2 | airship_att_control: 요 각속도 루프 | 스틱 → 각속도, PI + 설정값 제곱 FF, 복원 규칙, 배분기 포화 피드백, `rate_control::setIntegral/getIntegral`; 파라미터 `AS_YAWRATE_P/I/INT_LIM/FF/MAX` 다섯; **루프는 `CA_AIRFRAME 16` 에서만 닫힌다** (Custom 의 2500/2507 은 지금처럼 패스스루) | 스텝 응답·놓음 표류·정지 중 windup 한계의 비행 로그, 같은 카드의 gz 쌍둥이 결과 표 |
| 3 | gz SITL: 벡터링 비행선 + lta 월드 + 문서 | `airship_base` (부가질량 몸체) + `airship_vectored_independent` (7 m generic, 완전 플랜트 두 단) + `lta.sdf` + posix 2520 쌍둥이 + vehicles.md 항목 | 호버 게이트 10/10, 2 m 쌍둥이 (오버레이) 의 시뮬-비행 비교 |

upstream 에 가는 수: 파라미터 **8 개** (17 → 8), 메시지 **0 개** (`airship_allocator_status` 는 "추진이 요를 만드는가" 를 알리는 것인데 포드 둘뿐이면
항상 참이고, `airship_speed` 는 스케줄과 함께 간다), 새 모듈 코드는 배분기·루프·시험.

빼는 것 (포크 airship/dev 와 도구 레포 오버레이에 남는다): `CA_AIRSHIP_GRP/TAIL/CS_K`, 꼬리·조종면 경로, Cloudship gz, 속도 선택기·스케줄·
`AS_SPD_*`·`AS_YR_FF_SLP`, 물리 변형·문헌 가족, 2500/2507 의 배분기 이전 (손대지 않는다).

### 왜 이렇게 좁히나

- 리뷰어는 날지 않은 경로를 받지 않고, 안 날 경로가 섞이면 난 경로까지 의심한다. 꼬리·크레딧·일체 모드는 이 기체가 날리지 못한다
  (Y 선 일체 틸트는 요가 없어 추력 투영만 보여 준다: 그 경로는 Cloudship 급 기체나 그 소유자가 검증할 때 낸다).
- 좁히면 설명이 한 자리에 들어온다: "틸트 포드 둘, 요는 차동 추력, 루프는 그 배분기에서만".
- 루프를 `CA_AIRFRAME 16` 에서만 닫는 것은 기존 사용자 영향 0 을 위해서다. 지금 브랜치의 "다른 배분기는 루프를 둔다" 규칙은 Custom 의 2500
  (러더뿐) 에서 정지 중 적분기가 감기는 문제를 만든다.
- 증분이 기체 로드맵을 따른다: 고정 핀 → 스케줄 PR, 핀 제어 → 조종면 크레딧 PR. 각각 그때의 비행으로.

## 3. 기체가 오기 전에 할 일 (지금; 2~3 일)

1. **PR 브랜치를 upstream main 에서 뜬다** (`pr/airship-allocator`, `pr/airship-yaw-loop`, `pr/airship-sitl`; airship/dev 는 연구 브랜치로 그대로).
   코드는 airship/dev 에서 가져오되 좁힌다: 배분기에서 그룹·꼬리·조종면 분기와 상태 메시지 제거, 루프에서 선택기·스케줄·`airship_speed` 제거,
   루프 게이트는 `CA_AIRFRAME == 16`. 주석 밀도는 오늘 패스의 것.
2. **좁힌 펌웨어가 넓은 것과 같음을 확인한다.** 독립 포드 경로는 같은 코드이므로: gtest, 배분기 identity 하니스 (12 세트) 가 airship/dev 와 동일,
   fmu-v6x 2520 프리셋 핀 지문 0.0 us, SITL 호버 게이트 2520 10/10 (도구 레포가 PR worktree 를 PX4_ROOT 로).
3. **기체는 PR 브랜치의 펌웨어를 난다.** 비행이 검증하는 것이 제출할 코드여야 한다. `build_fmu.py` 를 PR worktree 에서 돌려 스테이징.
4. **하드웨어 패키지의 카드마다 "어느 PR 의 어떤 주장을 뒷받침하는가" 를 적는다** (벤치 1~7, 비행 카드): 지문·멈춤쇠·추력 곡선 → PR 1; 스텝·
   놓음·windup 한계·Land 모드 무장 유지 → PR 2; 같은 카드의 gz 쌍둥이 → PR 3 의 비교 표.
5. PR 본문 초안 (`upstream_pr_bodies.md`) 을 이 셋으로 다시 쓴다: 숫자 자리는 비행 뒤 채운다.

## 4. 기체가 온 뒤

벤치 카드 → 비행 카드 (수동 → Stabilized/Acro) → 로그를 Flight Review 에 → PR 본문 완성 → 1, 2, 3 순서로 제출. 사용자 검토·서명 뒤.

## 5. 열린 결정 (사용자)

- gz 모델 이름 `airship_vectored_independent` 를 upstream 에서 그대로 둘지 (`airship_tiltpod` 같은 짧은 이름이 upstream 모양에 가깝다).
- 2 m 실내 모델 자체를 upstream 에 낼지 (M2): 비행과 시뮬이 같은 기체라는 가장 센 증거가 되지만, 기체 치수가 공개된다. 기본은 7 m generic 만.
- 일체 틸트 Y 선 시험을 할지: PR 에는 쓰이지 않으므로 우리 지식용.

## 6. 최종 제안 (15:00; 사용자 조건: 레포마다 PR 하나, PR 하나에 큰 주제 하나, 실내 시험 뒤 제출)

**PX4-Autopilot PR 하나 — 틸트 포드 비행선 배분기 (`CA_AIRFRAME 16`, Airship: tilt pods).**
**PX4-gazebo-models PR 하나 — 그 기체의 gz 쌍둥이와 lta 월드.** (PX4 PR 의 posix 2520·문서·포인터가 이것을 가리킨다.)

요 각속도 루프는 **다음 라운드** 의 PX4 PR 이다. 이유:

- 배분기는 기계적이고 핀에서 검증된다 (지문 = 쌍둥이 = 벤치). 리뷰어는 effectiveness 클래스를 추가하는 PR 을 많이 봐 왔다 (헬리콥터·로버).
  루프는 제어 법칙 (FF 모양, 복원 규칙, 데드존 에지) 의 토론을 부르고 가장 긴 비행 증거가 필요하다. 둘을 한 PR 에 섞으면 두 영역의 리뷰어가
  한 PR 을 봐야 하고, 루프의 논쟁이 배분기까지 붙든다.
- 배분기만으로 기체는 완전히 난다: Manual/Stabilized 패스스루에서 틸트로 추력을 벡터링하고 요는 차동 추력 (요 스틱 = 토크). Cloudship 이
  지금 나는 방식과 같다. 루프는 그 위의 개선이다.
- 배분기가 먼저 있어야 루프 PR 이 설명된다 ("이 배분기에서만 루프를 닫는다").

### PX4 PR 의 내용

| 들어가는 것 | 비고 |
|---|---|
| `ActuatorEffectivenessAirship` (독립 포드 둘로 좁힘) + `AirshipPod` + 두 시험 | 그룹·꼬리·조종면 크레딧·상태 메시지·요 권한 계산 (`AirshipYawAllocation.hpp`) 은 뺀다 |
| `CA_AIRFRAME` 16 "Airship (tilt pods)", 파라미터 `CA_AIRSHIP_TLMIN/TLMAX/TLT_R` | 파라미터 셋 |
| 비행 에어프레임 2520 (`COM_DISARM_LAND -1` 은 **이 에어프레임 안에**: 2500/2507 에 영향 없음) | `rc.airship_defaults` 는 손대지 않는다 |
| posix `2520_gz_airship_vectored_independent`, CMake, 문서 (frames_airship 의 틸트 포드 절, 시뮬 항목), 서브모듈 포인터 | 모델 PR 머지 뒤 |
| `airship_att_control` 변경 없음, 메시지 없음, `rate_control` 변경 없음 | |

리뷰어 질문 둘에 답을 준비한다: "왜 Custom (9) 으로 안 되나" — Custom 은 틸트 서보를 토크·추력면으로밖에 못 쓰고 추력을 틸트각으로 투영하지
못한다 (Cloudship 의 틸트는 오늘 수동 패스스루); 비가역 프롭의 조향 밴드·슬루·멈춤쇠도 없다. "왜 틸트로터 VTOL (3) 이 아닌가" — 그것은
고정익·멀티콥터 혼합과 대기속도를 가정하고 ±180° 조향이 없다.

### 모델 PR 의 내용

`airship_base` (부가질량 몸체), `airship_vectored_independent` (7 m generic 완전 플랜트, 두 단 merge include), `lta.sdf` (부력 enable 목록, 시스템
목록, Magnetometer 플래그). 생성기 `--upstream` 의 출력, 쌍둥이 검사 통과.

### 실내 시험이 줘야 하는 증거

벤치: 핀 지문 (= SITL 쌍둥이 = airship/dev), 틸트 멈춤쇠와 TLT_R, 추력 곡선 (THR_MDL_FAC). 비행 (PR 브랜치 펌웨어로): Manual/Stabilized 호버,
틸트 벡터링 전후·상하 이동, 차동 추력 요 (조향 밴드에서 떨림 없음, actuator_servos/motors 대 토크·추력 설정값), Land 모드에서 무장 유지.
2 m 쌍둥이 (오버레이) 로 같은 카드의 시뮬-비행 비교. 로그는 Flight Review 에.

### 지금 할 일

1. upstream main 에서 PR 브랜치 (PX4 `pr/airship-tiltpod`, 모델 `pr/airship-sitl`) 를 뜨고 배분기를 좁혀 옮긴다 (2 일).
2. 좁힌 배분기 = 넓은 배분기의 독립 포드 경로: identity 하니스 12 세트 동일, fmu-v6x 2520 지문 0.0 us, SITL 호버 게이트 (루프 없는 패스스루
   항목만) 통과.
3. PR 브랜치 펌웨어를 스테이징한다. 실내 시험은 이것으로 난다 (요 루프 시험은 airship/dev 펌웨어로 따로; 그 증거는 다음 라운드).
4. 하드웨어 패키지 카드에 "PR 주장 ↔ 카드" 를 적고, PR 본문 초안을 이 둘로 다시 쓴다.
