# upstream PR 계획 (2026-10-07, 교수 역할 초안)

airship/dev 는 upstream main 보다 151 커밋 앞서 있다 (동기화 끝 bccba2ef8c 위에 오늘 12). upstream 은 PR 을 squash
병합하므로 PR 하나가 main 의 커밋 하나가 된다: 나누는 단위는 커밋이 아니라 PR 이다. 하나의 배분기, 두 등급의 검증
(결정 D1) 을 PR 마다 적는다. 모델 저장소 (PX4-gazebo-models) 의 PR 이 먼저 병합되어야 PX4 쪽이 그 커밋을 가리킬 수 있다
([[gz-submodule-publish-order]]).

## 순서

| # | 저장소 | 내용 | 검증 등급 | 의존 |
|---|---|---|---|---|
| M1 | PX4-gazebo-models | lta 월드, airship_base/_am, airship_vectored_independent 가족 (6), airship_fixed 가족 (4), cloudship 가족 (6), 부력 목록. 생성기의 출력이라는 머리 주석. | 시뮬레이터 (엔진 측정: 부가질량 9/9, 공력 모멘트 14/14, 핀 감쇠 0.07 %) | 없음 |
| M2 | PX4-gazebo-models | airship_indoor 가족 (8) + 월드 항목 | 시뮬레이터 + 하드웨어 (검증 기체) | M1 |
| P1 | PX4 | 비행선 배분기 CA_AIRFRAME 16: 독립/일체 틸트 포드, 꼬리 추진기, 조종면 크레딧 (`ActuatorEffectivenessAirship`, `AirshipPod`, `AirshipYawAllocation`), `airship_allocator_status`; 2520 비행 에어프레임; 2500/2507 을 새 배분기로 이전 | 독립 틸트: 하드웨어 (2 m 기체, fmu-v6x 벤치 지문). 꼬리·조종면·일체 틸트: 시뮬레이터만 | 없음 (시뮬레이터 없이도 빌드·단위 시험) |
| P2 | PX4 | 요 각속도 루프 (`airship_att_control`: 스틱→각속도, PI+FF — FF 는 설정값의 제곱, 선체 항력의 법칙 (2026-10-12), 포화 피드백, `rate_control::setIntegral`), 파라미터 AS_YAWRATE_*, 문서 Tuning 절 | 하드웨어 (2 m 기체) + 시뮬레이터 48/48 | P1 |
| P3 | PX4 | 전진비행 스케줄: `airship_speed`, 속도 구간 선택기, 스케줄과 상한, 적분기 복원, AS_SPD_*/AS_YR_* 7개, 문서 Forward Flight 절 | 하드웨어 (고정 핀 달린 2 m 기체) + 시뮬레이터 (모델 대조 7/9) | P2 |
| P4 | PX4 | gz SITL: posix 에어프레임 2500~2537, 서브모듈 포인터 (M1+M2), 문서 vehicles.md/index.md, `lta` 월드 설명 | 시뮬레이터 | M1, M2, P1~P3 |
| P5 | PX4 | 나머지: gazebo-classic Cloudship 2519 수정, land detector, 메타데이터 | 시뮬레이터 | P1 |

P1 과 P2 는 분리할 수 있지만 P2 없는 P1 은 요 스틱이 토크 패스스루라 2520 의 비행이 바뀐다. 리뷰어가 받아들이기 쉬운
크기를 보고 P1+P2 를 한 PR 로 낼지 정한다 (코드 약 5,800줄 vs 2,700줄).

## 리뷰에서 나올 질문과 준비할 답

1. **파라미터 7개 (AS_SPD_*, AS_YR_*).** 각 파라미터의 소유 모듈과 역할이 yaml 에 있다. AS_SPD_DP_TOL 은 unit 필드가
   없다 (검사기의 단위 목록에 Pa 가 없음; sensors 의 `#unit: Pa` 선례). 줄일 수 있는 것: AS_SPD_LO/HI 를 하나로 (HI = 2 LO
   고정) — 지금은 둘 다 둔다, 핀 실속과 피토 하한이 다른 속도라서.
2. **메시지 2개.** `airship_speed` (로깅·텔레메트리, 10 Hz), `airship_allocator_status` (모듈 간 결과 전달, 파라미터
   갱신 때만). 둘 다 버전 없음. 대안인 컨트롤러의 CA_* 직접 읽기는 저장소 규칙이 금한다
   (`.github/instructions/parameters.instructions.md`).
3. **2500/2507 의 배분기 이전.** upstream 의 두 에어프레임이 custom rotor set (9) 에서 16 으로 옮겨 간다. 하드웨어 검증
   없음, 시뮬레이터 쌍둥이 (2500 twin 13/13, Cloudship 9/9) 만. PR 본문에 그대로 적고, 거부되면 P1 에서 두 파일의 이전만
   떼어 P5 로 미룬다 (코드 경로는 그대로 둔다).
4. **DCO.** 모든 커밋에 `-s` 서명이 필요하다. 지금은 서명 없음: 사용자가 검토한 뒤 `git rebase --exec 'git commit --amend
   --no-edit -s'` 로 한 번에 붙인다 ([[commit-after-user-review]]). AI 보조는 `Assisted-by:` 트레일러로만 밝힌다.
5. **문서.** `docs/en` 만 고친다; `docs/ko` 등은 Crowdin 산출물. `msg_docs/*.md` 와 parameter_reference 는 메타데이터
   동기화가 만든다. 문서 CI 의 prettier 와 링크 검사: 표 행 폭을 맞췄고 (생성기), 링크는 variant_check 가 제목에 닿는지
   확인한다.
6. **스타일.** `make check_format` 통과 (astyle). clang-tidy 는 CI 에서 본다.
7. **CI 의 모든 보드 빌드.** fmu-v6x airship 보드만 빌드했다. 기본 보드들 (`px4_fmu-v6x_default` 등) 에 airship 모듈이
   포함되는지 Kconfig 를 보고, 포함되면 그 빌드를 한 번 돌린다 (P1 전에).

## 사용자 지침 (2026-10-07 17:24)

실내 기체로 검증된 것만 올린다. P1+P2 는 10월 기체 (독립 틸트, 요 루프), P3 는 고정 핀을 단 뒤, 조종면 크레딧과
일체 틸트 (2500/2507 이전) 는 핀 제어와 서보 Y선 일체 틸트를 단 3단계 뒤. 푸시 전 사용자가 직접 검토한다.

## 하지 않는 것

- 히스토리 재작성 (squash, reword) 은 사용자 검토 뒤에만. 지금은 커밋 순서와 메시지를 보존한다.
- 푸시는 사용자가 한다. 포크 푸시도 지시 뒤.
- 전체 차이에 대한 자동 검토 워크플로는 마지막에 한 번, 비용을 먼저 말하고.

## PX4 관례 (사용자 지침 2026-10-07 18:56: 모든 작업이 upstream 행)

지금 트리에서 upstream 관례에 어긋나거나 리뷰어가 물을 것, 그리고 결정:

1. **생성 도구가 트리 밖에 있다.** 커밋된 에어프레임 3개의 표식 주석이 `airship_gains.py` 를 이름으로 가리키고, 모델
   SDF·에어프레임·문서 표는 `airship_family_gen.py` 의 출력이다. 리뷰어가 다시 만들 수 없는 생성물은 받지 않는다.
   결정: 도구를 트리에 넣는다. 선체 설계와 SDF 작성기 (`generic_hull_design.py`, `apply_generic_hull.py`, gz_model 의
   SDF 읽기) 는 모델이 사는 PX4-gazebo-models 의 `tools/` (선례 `tools/avl_automation`); 에어프레임·CMake·문서 작성기와
   이득 계산기 (`airship_family_gen.py` 의 PX4 쪽, `airship_gains.py`) 는 PX4 의 `Tools/` (제안 `Tools/airship/`). 기체
   JSON 은 두 저장소에 하나씩이 아니라 PX4 쪽에 두고 모델 도구가 경로로 받는다. 측정 보관 (`*.measured.json`) 은 도구의
   입력이라 같이 간다. 비용: 하루 분량 (의존 모듈 gz_model/report 분리, 두 저장소 시험, 문서), 토큰은 보통.
   그때 표식 주석을 트리 안 경로로 바꾼다.
2. **기체 고유 이름.** "Indoor Airship" (2530대) 은 회사·사업 맥락이 없는 일반 이름이라 둔다. 실내 가족이 upstream 에
   가는 근거는 "생성기의 두 번째 예제 (작은 선체)" 이지 특정 기체가 아니다; 7 m 기준 기체가 generic 이 되면 실내 가족은
   그 예제로 남는다.
3. **문서.** docs/en 만 고쳤다. 벤치 도구의 한국어 문서는 upstream 에 가지 않는다 (설계 근거는 PR 본문에 영어로 요약).
4. **커밋.** 관례 형식, `Assisted-by:` 트레일러, DCO 서명은 사용자 검토 뒤. 메시지에 사업 맥락 없음 ✓.
5. **시험.** 단위 시험 (gtest) 은 트리에 ✓; SITL 게이트·전진 검사는 트리 밖 (PX4 의 `test/` 통합 시험 체계로 옮길지는
   P3 때 결정: upstream 의 gz SITL CI 가 lta 월드를 돌리지 않는다).
6. **파라미터·메시지 규칙** (`.github/instructions/*.instructions.md`) ✓ (E1 때 확인).

## 2026-10-12 추가

- 도구를 트리에 넣는 결정의 집 (한 집 PX4 `Tools/airship/` 권고 vs 두 집) 은 `tools_in_tree_plan_20261012.md`; 사용자 확인 뒤 실행.
- 리뷰 질문 8: 왜 FF 가 설정값의 제곱인가 — 제자리 요 항력이 r|r| 이라 선형 FF 는 최대 각속도 아래에서 r_max/r 배 과하다 (SITL 14 m 선체: 반 스틱에서 넘침 8.2 대 6.7 deg/s); THR_MDL_FAC 가 추력의 제곱 법칙을 바로잡는 것과 같은 자리.
