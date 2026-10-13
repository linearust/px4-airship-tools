# 교수 결정 2026-10-13 (Fable 5.1) — 리팩터링·정리·미완 작업 (사용자 지시 "리펙토링해. 테스트 기체 나오기 전에 할 정리작업, 미완작업완료 등 진행해.")

## 10:42 검토 (워크플로: 탐색 7 + 반박 7 = 에이전트 14, 2.51M 토큰, 25 분)

범위: PX4 airship/dev 의 미푸시 28 커밋 (59 파일) + 도구 레포 전체 + 열린 항목 장부 (문서 9 종). 확정 62, 기각 24:
컨트롤러 C++ 8, 컨트롤러 시험 7, 배분기·메시지 8, 에어프레임·문서 10, 도구 핵심 12, yaw 모델 17; 장부 52 항목 (끝남 15, 열림 34, 대체됨 3).
기각의 주된 사유: 행동을 바꾸는 "정리" (COM_PREARM_MODE 이동, 바람 사례 가드), 설계에서 이미 정한 것 (오버레이 실행 경로), 취향 (시험 이름 길이, 파일 이름 규칙).

## 결정

- **D1 적분기 "지상 windup" 메모 (airship_yaw_rate_loop.hpp).** 처리하지 않고 적는다: 착륙 감지기는 무장 해제나 AUTO_LAND 에서만 "착륙" 을
  말하고 그때 루프는 열려 있다; 지상에서 스틱을 잡고 있으면 적분기는 AS_YR_INT_LIM 까지만 감기고 복원 규칙이 되돌린다. 지면 접촉 신호는
  별개 기능이다.
- **D2 파라미터는 갱신 때 한 번 읽는다.** schedule() 9 인자·SpeedSelector::update() 8 인자를 `Params` 구조체 하나와 `setParameters()` 로.
  mc/fw_rate_control 의 모양. 비트 동일 (gtest 8/8).
- **D3 주석 밀도.** 모듈·배분기·에어프레임의 설계 서사 (상한 없던 시절의 실패, 측정 이력, FW_ACRO_YAW_EN 비교, uint16 절단 이야기) 는
  코드에서 빼고 결론만 남긴다; 서사는 `upstream_pr_bodies.md` 로.
- **D4 생성 표식.** "regenerate, do not edit" 두 줄 → 출처 한 줄 (`# Yaw loop values from airship_gains.py (px4-airship-tools) for this hull`),
  블록은 파일 끝까지, 생성기는 그 줄부터 다시 쓴다. 15 파일, 파라미터 줄 변화 0.
- **D5 2519 @output 헤더.** PWM_MAIN_FUNC 매핑대로 고친다 (MAIN1 starboard, MAIN2 port, MAIN3 tilt, MAIN4 tail). 클래식 모델의 모터 0 이 +y 라
  클래식 시뮬의 차동추력 요 부호는 미검증 — 모델 항목으로 남긴다. (분리 설계의 "2507 로 되돌림" 은 트림 커밋에서.)
- **D6 시험.** 같은 propulsive_yaw=false 경로를 세 번 도는 기능 시험 하나 삭제; 스케줄 배선·복원 규칙·데드존 에지를 Run() 을 통해 덮는 사례
  넷 (TEST_F 다섯: 스케줄 2, 복원 1, 데드존 에지 2; 기능 시험 10 → 14) 추가; 데드존 없는 복원 (옛 규칙으로의 회귀가 12 시험을 다 통과했다) 과 정확한 보유값 단언.
  한 시험에서 모듈을 두 번 만들면 rate_ctrl_status (PublicationMulti) 가 새 인스턴스로 가서 구독이 끊긴다 — 시험당 모듈 하나.
- **D7 yaw 모델의 닻.** plants.PLANTS 는 import 때가 아니라 첫 사용 때 만들고 EXPECTED 와 다르면 SystemExit (test_controller 의 SKIP 경로가
  받는다); validate.py 도 같은 거부. 재앵커링 (레지스트리) 은 분리 4 단계 그대로.
- **D8 모델의 램프.** policies 의 D1 스케줄은 펌웨어의 kRampStart × AS_SPD_HI 램프 (hpp 에서 읽어 단언), 0c 램프는 RAMP_LO_0C/HI_0C 로 이름을
  바꾼다; d1_runs_20261007.jsonl 은 증분 1 펌웨어 (lo 1.5) 의 것이라고 적는다.
- **D9 2512 전진 중단의 원인.** 플랜트가 아니라 검사의 스폰 높이: Cloudship 은 코가 2.52° 아래로 매달리고 (무게중심이 부력중심보다 0.022 m
  앞), 추력 없이 제 축을 따라 u·tan(hang) 으로 가라앉는다 (4 m/s 에서 0.17 m/s; 로그의 w 0.076 m/s @ 1.73 m/s 가 정확히 tan 2.5°).
  20 m 스폰으로는 다리 2 를 못 끝낸다. 검사가 계획에서 침하 합 (71 m) 을 더해 필요한 높이 (78 m) 를 내고, 낮으면 다리 전에 거부하며
  PX4_GZ_MODEL_POSE=0,0,80 을 말한다. 기본 스폰 20 m 는 그대로 (다른 기준선을 흔들지 않는다).
- **D10 SITL 자력계 (포크 동기화 후속 FS-MAG).** 실제 결함: lta.sdf 가 제 플러그인 목록을 쓰면서 Magnetometer 에 use_units_gauss=false ·
  use_earth_frame_ned=false 를 안 줘서 gz_bridge 가 "legacy mode ... heading will be wrong" 을 매 실행 경고 (sitl_out/server.log 2026-10-12).
  SITL EKF 헤딩이 groundtruth 와 5~15° 어긋나던 원인 후보. 월드 플래그 + 모델 12 개의 잡음을 tesla (3e-7) 로 한 커밋에. 지면은 upstream 의
  100 km 상자. 재기준선 체인 D 로 확인.
- **D11 보류·미실행.** OBS-FINNAME (핀 이름을 선체 축 기준으로: 생성기 주석만, 모델 diff 를 흐려 오늘은 안 함), R2 (무장 중 GRP 전환: 문서로
  '무장 해제 상태에서 바꾼다'), D-g, hover_ff_family 스크립트는 유지 (정리만).

## 커밋 (전부 로컬, 서명 없음)

PX4 airship/dev (a9e7c3c16b 위): afa1a26c6d rate_control getIntegral · c3fa91ba8f 컨트롤러 리팩터링 · 52e433fc87 시험 · 78459fa71d 배분기 주석 ·
1865a03eaf 에어프레임 · 48b45f3e5f 문서 · 1d2aef0e5c 서브모듈 범프 (models 0c4307b: 자력계 tesla + lta 플래그 + 지면 상자) · 7791bc86c4 CA_AIRSHIP_GRP 는 무장 해제 상태에서 바꾼다 (D11 R2, module.yaml).
도구 main: c85a9f9 gz_model 통합 · 7f8e74d 닻·램프 · 62a2e62 전진 검사 스폰 높이 · 17c1efc 생성기 표식 · bf0cd59 이 기록·PR 본문·pins · b7f5573 variant_check 두 집 · 00a5d73/c322ee6 체인 D·3 단계 기록 · eaa7400 f927e8b 4efd7ca 55a0703 생성기 두 모드 · edc01af pins at PX4 7791bc86c4.

## 검증

gtest 8/8 (Airship) + RateControl, check_format, 도구 run_tests (생성기 17/17 (bf0cd59 시점; 3 단계 뒤 20/20), 변형 16/16, 전진 분석 220/220, 컨트롤러 0 실패 1 건너뜀,
sitl_link 9/9), 보드 빌드 micoair h743 ×4 + zenfc + fmu-v6x airship (fw_48b45f3e5f8 스테이징, 올리기는 사용자), 전진 검사 `--plan` 의
Cloudship 침하 줄. 체인 D (SITL 재기준선: 2520 호버, 2517 호버, 2527 전진, 2512 전진 @80 m) 는 아래에.

## 체인 D 결과 (11:52~12:17, PX4 1d2aef0e5c + models 0c4307b, 자력계 수리 뒤의 재기준선; 7791bc86c4 는 module.yaml 설명만, 체인 D 와 같은 펌웨어)

| 검사 | 결과 | 비고 |
|---|---|---|
| 호버 2520 rigid | 10/10 | px4 로그에 "legacy mode" 경고 0 건 (어제는 매 실행) |
| 호버 2517 Cloudship 완전 | 9/9 | |
| 전진 2527 | 42/42 | EKF 헤딩 오차 중앙값 +0.63°, p95 0.69° (어제 로그 +5.17° / 6.11°); EKF u 1.585 vs groundtruth 1.583 m/s |
| 전진 2512 Cloudship fin, PX4_GZ_MODEL_POSE=0,0,80 | **43/43** (어제 18/20 중단) | 스폰 79.9 m, 필요 78 m 판정 통과; 세 다리 모두 완주. 헤딩 오차 −0.29°, p95 0.57° |

D9·D10 은 닫힌다. 2512 는 앞으로 80 m 스폰으로 돌린다 (검사가 20 m 에서는 다리 전에 거부하고 그 값을 말한다).

## 12:00~ 분리 3 단계 (split/step3 worktree 에서 작업, 12:19 main 에 cherry-pick: eaa7400 평범한 완전 플랜트 · f927e8b 오버레이 모드 · 4efd7ca airship_gains rigid bare · 55a0703 고정 쌍둥이; 브랜치·worktree 는 지움)

`--upstream` (몸체 + 평범한 완전 플랜트 둘: 벡터링과 고정 쌍둥이, merge include 두 단, 에어프레임, 쌍둥이 검사) 과 `--overlay` (rigid 이름 +100 번호의
자체 완결 가족, lta_lab, decades, vehicles.md; 템플릿은 오버레이의 generic rigid), airship_gains 의 rigid bare. 생성기 시험 20/20: 트림된 스크래치 트리와
스크래치 오버레이에서 세 쌍둥이 검사 모두 같음, 두 번째 생성 변화 0, 계산기 재작성 변화 0. S1/S2 보정 (오버레이는 제 부가질량 몸체를 가진다) 은
설계 문서 7 절에. 남은 것: cloudship 의 upstream 모양 (자체 완결), 트리 쪽 variant_check 의 upstream 모양 사실, (4) 플랜트 레지스트리, (5) gz_run.sh,
(6) 트림, (7) 재기준선.
