# MASTER_PLAN.md — 단계 순서·동결 게이트·실차 사다리 (실행 정본)

> 계획의 **근거·개정 이력** = `~/Desktop/개발현황/0719_실차전환_마스터플랜.md` (역사 정본).
> 이 파일은 그 §7 개정 2(3자 수렴)의 실행본. 완료 로그는 여기 적지 않는다 (→ 날짜별 현황).

## 1. 확정 순서 12단계 (07-19 개정 2 — "최종 시나리오는 실차 후")

1. ~~G 묶음 봉합~~ ✅ → 2. ~~전체 회귀 녹색 기준선~~ ✅ → 3. ~~중간보고서~~ ✅
4. **SpeedManager → GoalManager → E2E 공통 하네스 단계적 분리** ◀ 현재 (§2)
5. **platform-core-freeze** (release tag + hash manifest)
6. 역할 B V1 최소 계약 확정 — ★ 책임경계 (b) camera-frame 3D 는 07-20 확정, 세부(필드·주기·실패 표현)만 잔여 (병렬 진행 가능)
7. 가상 시나리오를 검증 대본으로 실차 R0~R8 진행 (§3)
8. 실차 경험 반영 → 최종 시나리오 확정 (정책 8문항 = §4)
9. FSM 순수화 + Mission-v1 구현
10. 인지 주입 E2E → **mission-logic-RC**
11. Orbbec·YOLO·Perception Adapter 실측 통합 (캘리브레이션·데이터 수집은 R 사다리와 병렬 개시 가능)
12. 통제환경 정상·오류 시험 통과 → **mission-v1-freeze**

## 2. platform-core 구조 분리 (동작 불변 리팩터 — 단계마다 회귀 전량, 한 단계 = 한 커밋 묶음)

1. ~~**SpeedManager 추출**~~ ✅ 완료 (07-20, `f94da44` — 미준비 정책 = timeout 30초 후 FAULT 확정. 기록 = `0719_현황.md §19`)
2. **GoalManager 추출** ◀ 현재 — goal 전송·cancel 확인 사슬·stale goal 방어 전부 이관 (최대 수익). 완료조건·경계 = `CURRENT_HANDOFF.md`
3. **E2E 공통 하네스** — readiness·cleanup·send_goal 재전송을 셸 함수 라이브러리로. 이때 readiness "최대 90초" 문구·deadline 함수 통일 (Codex §14.5 P2)
4. FSM 순수화(상태+이벤트→다음상태+명령 표) — **시나리오 확정 후로 보류** (순서 9에서)

각 단계 완료 게이트 = `TEST_GATES.md` §1 전체 게이트 전량 PASS → 한 커밋+push → Codex 독립 검토.
세 분리 + Codex 최종 게이트 통과 후에만 platform-core-freeze 태그.

## 3. 실차 검증 사다리 R0~R8 (통과 전 다음 단계 금지, 전 단계 rosbag 기록)

| 단계 | 무엇을 | 통과 판정 |
|---|---|---|
| R0 바퀴 공중 | 통신·방향·부호·정지 | 모터 방향·encoder 부호 일치, **cmd_vel 단절 0.5s 내 정지**(watchdog), E-stop 물리 차단 |
| R1 유선 저속 | 최초 지면 주행 (0.05m/s) | 명령대로 직진·회전, 이상 진동·편향 없음 |
| R2 odom ★최대 관문 | 3m 직진·제자리 1바퀴 | `/odom` 3m±3% · yaw 2π±10% (회전 슬립은 예상된 오차 — EKF 흡수 범위 판단) |
| R3 센서 rosbag | EKF 재료 수집 | odom/imu/scan 주기·timestamp 단조·covariance 의미값 |
| R4 EKF 단독 | bag replay 융합 검증 | yaw jump 없음·정지 드리프트 미미 |
| R5 SLAM 지도 | 실터널 지도 제작 | 벽 직선·루프 닫힘·시작 pose 재현 절차 확립 |
| R6 Nav2 단일 goal | 자율주행 최초 | 오차 ≤0.3m·collision ON 정지거리 확인·부정 회귀 실차판 |
| R7 무인 전체 미션 | 상태머신 실차 | PATROL→…→ESCAPED + abort 실정지 실차판 |
| R8 감독하 추종 | 사람 포함 (안전요원+E-stop) | 추종감시·SEARCH_BACK 동작, 오탐 기록 → 역할 B 융합 요구사항 |

사전 준비(S4~S5, 트리거 임박 시): **`tunnel_bringup` 별도 패키지** (시뮬 파일은 한 글자도 안 바꿈),
실측 URDF/footprint·collision ON·velocity_smoother 명시·절대경로 제거·use_sim_time false·Jetson aarch64 소스 빌드.
실측 수치·반영 지점 정본 = **`docs/REAL_ROBOT_VALUES.md`**.
⚠ 구판 이식 체크리스트(`0707_로드맵_통합계획.md §2-B`, `실차값_수령체크리스트.md §2`)는 **시뮬 파일을 직접
고치는 전제**로 쓰여 있다 — 항목(절대경로·use_sim_time·collision ON·penalty 등)은 유효하되, 반영 위치는
전부 `tunnel_bringup` 신규 파일로 읽는다.

## 4. 최종 시나리오 확정 회의 안건 — 사람 탐색 정책 8문항 (순서 8에서)

① 몇 프레임 관측이면 사람 확정? ② 다수 인원이면 누구 기준? ③ 접근 가능성 판단은 누가?
④ "집결 완료" 판정(시간/인원/관제 승인)? ⑤ 이동·가림 시 대응? ⑥ GUIDE 중 동일인 추적 유지?
⑦ 놓치면 재탐색/관제 문의/단독 탈출? ⑧ 화재·사람 동시 위험 시 우선순위?

## 5. 월별 큰 그림 (2026-07 → 10)

- 7월 하순: 구조 분리 3종 + platform-core-freeze / 병렬: 역할 B V1 세부 미팅·구동부 대기
- 8월: bringup 준비 + micro-ROS 계약 확정, 구동부 트리거 시 R0~R2 / 병렬: 감지 3차(지도 배경제거)
- 9월: R3~R8 완주, /detections 실물 통합, 관제 C5(영상)·C6(로그인+rosbridge 보안)
- 10월: 통제환경 반복 시험 → 시나리오 동결 → 리허설 → 최종 발표 (rosbag 재생 백업 필수 — 현장 데모는 한 번은 실패한다는 전제)

## 6. 리스크와 플랜 B

| 리스크 | 징후 | 플랜 B |
|---|---|---|
| 구동부 지연 | 8월 중순까지 3m 미통과 | 시뮬 고도화 전환(감지 3차·BT BackUp·쌍굴 확장), 시연 백업 = 시뮬 데모 |
| R2 odom 품질 한계 | 3m 에 2.5m 등 | realodom 파라미터로 SLAM 흡수 시도 → 한계면 저속 한정 운용 |
| 역할 B 지연 | 9월까지 실물 없음 | "근거리 물체 추종감시" 명칭 유지 시연 (라이다 클러스터가 커버, 보고서에 한계 명시) |
| 라이다 실물 문제 | R3 bag 에서 scan 끊김 | scan watchdog 방어 완료 + sllidar respawn 정책 추가 |
| 검토 루프 발산 | 재검토마다 새 P1 다수 | 탈출 조건 발동 (AGENTS.md §5) — P0 만 반영 후 동결 |

## 7. SpeedManager 이후 예약 항목 (비차단 보류 — 지금 손대지 않음)

1. `map_promote.sh` 는 best-effort transaction — 다음 정기 지도 제작 때 release evidence 3종(staging 로드 로그·FAIL 시 정본 hash 불변·PASS 후 manifest 일치) 실전 확인.
2. readiness 문구·deadline 통일 → E2E 하네스 추출 때 (§2-3).
3. 마스터플랜 §3.3 "동일 바이너리" 표현 → "동일 소스·아키텍처별 빌드"로 다음 문서 정리 때.

## 8. 유효 결정 색인 (한 줄씩 — 상세 근거는 링크 절. 100줄 초과 시 별도 파일로 분리)

| 결정 | 날짜 | 근거 |
|---|---|---|
| 운영 = localization 모드 (라이브 SLAM 은 지도 제작 시만) | 07-06 | 0705_현황 §18 |
| `rolling_window: false` 가 SLAM+Nav2 표준 (rolling+static 결함) | 07-05 | 0705_현황 §12.2 |
| goal header.stamp = 0 (최신 TF) | 07-05 | 0705_현황 §12.2 |
| 가짜 detection 금지 — 계약+깡통 퍼블리셔만 | 07-05 | 0705_실차전_전략 |
| 그래프 선언 시 직선 fallback 폐기 (동일 투영점 → yaml 고정 집결지) | 07-19 | 0719_현황 §13 |
| 지도 수락 기준 = 스모크 ≠ 수락, negative 통과까지 (G3 자동화) | 07-19 | 0719_현황 §16·§17 |
| "사람 인식" 호칭 금지 → "근거리 물체 추종감시" (융합·혼동행렬 전까지) | 07-19 | 마스터플랜 S3 |
| regression_3goals 허용치 0.3m / 벤치 수치는 시뮬 상한 (실차 인용 금지) | 07-19 | 마스터플랜 S3 |
| 동결 2단 분리 + 최종 시나리오는 실차 후 | 07-19 | 마스터플랜 §7 개정 2 |
| 역할 B 책임경계 = (b) camera-frame 3D (YOLO 측 depth 결합) | 07-20 | YOLO_탐지연동_합의사항 |
| 시뮬 자산 보존 — 실차는 tunnel_bringup 별도 패키지, T자·쌍굴 회귀 유지 | 07-19 | 마스터플랜 §3.1 |
