# CURRENT_HANDOFF.md — 지금 할 한 묶음 (매 세션 시작점)

> 작업 완료 시 구현자가 갱신하고 검토자가 사실관계·테스트 결과를 확인한다.
> 이전 묶음 내용은 날짜별 현황으로 보낸다. 항상 이 한 묶음만 유지.

- **현재 기준 커밋**: 이 파일과 같은 커밋 (main — E2E 공통 하네스 추출 = 구조 분리 3/3 구현 완료, 07-24)
- **현재 단계**: 마스터플랜 12단계 중 4단 — 구조 분리 **3/3 E2E 공통 하네스 구현 완료 · Codex 독립 검토 대기**
  (검토 통과 시 구조 분리 3종 완결 → 5단 platform-core-freeze. 검토 범위 = `docs/TEST_GATES.md §7` 셸 도구).
- **직전 완료**: E2E 공통 하네스 추출 — `tools/lib_e2e.sh` 신설(cleanup·fail·trap·state·deadline·
  wait_nav2_ready·send_goal 공통화), 4개 E2E 가 source 로 교체. 판정 기준(PASS·오차·타임아웃) 무변경
  순수 리팩터, 4 E2E 전부 리팩터 전과 동일 PASS(abort 0.0m·3goals 0.138m·negative 금지3 ABORTED·
  mission ESCAPED). 구현 기록 = `0723_현황.md §10`. (선행 GoalManager 2/3 는 §7 검토 통과 —
  `CODEX 현황/0723검토현황.md §7`.)

## 이번 한 묶음 목표 — E2E 공통 하네스 추출 (구조 분리 3/3)

> ✅ **구현 완료 (07-24)** — `tools/lib_e2e.sh` 추출, 4 E2E 동일 PASS (`0723_현황.md §10`).
> 아래는 이 묶음의 요구·검토 기준(검토자 참조용). Codex 독립 검토(범위 `docs/TEST_GATES.md §7`)
> 통과 시 → 구조 분리 3종 완결 → platform-core-freeze.

readiness 활성 대기·프로세스 cleanup·`send_goal` 재전송·deadline 계산을 4개 E2E 스크립트
(`mission_e2e`·`abort_e2e`·`regression_negative`·`regression_3goals`)가 각자 복붙하고 있다.
이를 **셸 함수 라이브러리 하나로 추출**해 중복을 없애고, readiness "최대 90초" 문구·deadline
함수를 한 곳으로 통일한다 (Codex §14.5 P2 — `MASTER_PLAN.md §7`).

## 완료조건

1. 공통 셸 함수 라이브러리(예: `tools/lib_e2e.sh`)를 신설하고 4개 E2E 스크립트가 그것을
   source 해서 쓴다: 프로세스 cleanup(부모 launch 먼저 → nav2 → gazebo, 브래킷 트릭),
   Nav2 readiness 대기(단일 함수·단일 문구), goal send + 응답 대기, deadline/경과시간 계산.
2. readiness "최대 90초" 문구와 deadline 함수를 라이브러리 한 곳에만 둔다(중복 제거).
3. E2E 판정 기준(PASS 조건·허용 오차·타임아웃)은 **한 글자도 바꾸지 않는다** — 순수 리팩터.
4. 4개 E2E가 리팩터 전과 동일하게 PASS하는지 각각 실행해 확인한다(수치 회귀 기록).
5. `bash -n`(문법)과 shellcheck(있으면)로 각 스크립트를 검사한다.

## 허용 파일/범위

- `tools/lib_e2e.sh` (신설) 또는 유사 공통 라이브러리
- `tools/mission_e2e.sh` · `tools/abort_e2e.sh` · `tools/regression_negative.sh` ·
  `tools/regression_3goals.sh` (공통부 추출·source 로 교체)

## 금지 범위

`mission_node.py`·`speed_manager.py`·`goal_manager.py` 동작 변경, Nav2 파라미터·costmap·
planner·지도 자산·`make_map.sh` 실런을 섞지 않는다. E2E 판정 기준(PASS 조건·허용치·타임아웃)
변경 금지 — 셸 중복 제거만.

## 보존해야 할 안전 불변조건

- cleanup 은 부모(`ros2 launch`)부터 kill → `pkill -9 -f "lib/nav2[_]"` (좀비 bt_navigator
  가로챔 방지, `AGENTS.md §4`). 브래킷 트릭으로 자기 매칭 자살 금지.
- 각 E2E 의 완료 판정(abort 실정지·3goals 오차≤0.3m·negative 3종 ABORTED·mission ESCAPED)
  기준은 리팩터 전후 동일해야 한다.
- Gazebo E2E 는 어떤 worktree 에서도 동시 실행 금지 (전역 프로세스 cleanup 충돌).

## 완료 판정 + 필수 테스트

변경 직후 `bash -n` 전 스크립트, 묶음 완료 시 `TEST_GATES.md §1` 전량(E2E 4종)을 순차 실행해
리팩터 전과 동일 PASS를 확인한다. 테스트 수치·남은 위험을 `0723_현황.md` 다음 절에 기록하고,
`bash tools/doc_check.sh --strict` → 한 커밋+push → `bash tools/doc_check.sh --after-push`
순서를 지킨다. 검토자 실행 범위는 `TEST_GATES.md §7`(셸 도구 변경 = `doc_check` + `bash -n`).

## 완료 후 다음 단계

E2E 하네스 독립 검토 통과 → 구조 분리 3종 완료 → **platform-core-freeze**
(release tag + hash manifest, `MASTER_PLAN.md §1` 5단계).

## 근거 문서

`docs/MASTER_PLAN.md §2` · `docs/MASTER_PLAN.md §7` · `docs/TEST_GATES.md §1` ·
`docs/PITFALLS.md §1` · `~/Desktop/개발현황/0723_현황.md §1`
