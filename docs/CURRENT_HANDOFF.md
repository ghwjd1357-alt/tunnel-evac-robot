# CURRENT_HANDOFF.md — 지금 할 한 묶음 (매 세션 시작점)

> 작업 완료 시 구현자가 갱신하고 검토자가 사실관계·테스트 결과를 확인한다.
> 이전 묶음 내용은 날짜별 현황으로 보낸다. 항상 이 한 묶음만 유지.

- **현재 기준 커밋**: 이 파일과 같은 커밋 (main — SpeedManager 보완 5차 진행, 07-23)
- **현재 단계**: 마스터플랜 12단계 중 4단 — 구조 분리 **1/3 보완 5차 (진행 중)**.
  5차 통과 = SpeedManager 동결 → 2/3(GoalManager) 착수 (`CODEX 현황/0720검토현황.md §13`)
- **직전 완료**: 4차 §12 P1 봉합 (`322d146`, `0720_현황.md §24`) — 게이트 술어를 latch→live
  로 교정. §12 원 반례는 닫혔으나, **§13 에서 보존한다던 §22.3 정책의 구멍**이 드러남.
- **이번 한 묶음 목표 = SpeedManager 보완 5차 (§13 P1 봉합)**: guide 저속 복구 예산이
  **SEARCH_BACK 전환 중** 소진되면 실패 통보(`_on_guide_speed_fail`)가 콜백 else 로 **무시**돼
  (state 가 GATHER/GUIDE 가 아님) cancel·FAULT 없이 GUIDE 복귀 → **고장 은폐 영구 정지**.
  §24 와 같은 오류 클래스(fail-open 기본값). 실패 결정을 **콜백 한 번**에 맡기지 말고
  **매 tick live** 로 확인한다(=§24 의 latch→live 교훈을 한 단계 위에 적용).
  ⚠ **B(취소 종결 직렬화)는 이 묶음에 넣지 않는다 — 여전히 GoalManager 소관** (아래 예고).
  여기 쓰는 `cancel_current_goal()`은 branch ② 가 이미 쓰는 **기존 호출**이라 B 를 선점하지 않는다.

## 완료조건 (SpeedManager 보완 5차 — §13 P1)

1. **`guide_speed_recovery_exhausted` (live) 신설** (`speed_manager.py`) — "한 번 성공한 뒤
   reconcile 예산 소진"만 뜻한다(최초 진입 실패 포함 금지 → 그래서 `_failed` 아님):
   `_settle_gave_up ∧ _desired origin=guide ∧ _applied≠요구값 ∧ ¬_inflight`. 재무장
   (`request_guide`→`_new_request`)이 `_settle_gave_up=False` 로 즉시 False 화.
2. **live fail-closed 가드** (`mission_node.py` `tick()` — `self.speed.tick()` 직후,
   **`ensure_sync` 앞**): `state ∈ {GUIDE, SEARCH_BACK} ∧ guide_speed_recovery_exhausted`
   → `cancel_current_goal()`+`enter_fault()`. **`return` 하지 않는다** — 같은 tick 에 FAULT
   가 발행되고 이후 GUIDE/SEARCH_BACK 동작은 state 가 FAULT 라 자동으로 실행 안 됨.
   ⚠ 가드를 `ensure_sync` 뒤에 두면 sync 요청이 `_inflight=True` 로 술어를 가려 뚫린다.
3. **FAULT→SEARCH_BACK 재시도 재무장** (`mission_node.py:833` 부근) — 기존엔 resume 가
   GUIDE 일 때만 `request_guide` 했다. `state ∈ {GUIDE, SEARCH_BACK}` 로 넓힌다.
   안 하면 SEARCH_BACK 복귀 시 소진 술어가 그대로 남아 **즉시 재-FAULT** 루프.
4. **콜백 backstop 정리** (`mission_node.py:1037`) — branch ② 를 `state ∈ {GUIDE,
   SEARCH_BACK}` 로 넓혀 즉시성 확보. live 가드(2)는 **콜백 유실에 대한 backstop**.
5. 공격 테스트 (구현 전 확정, `MissionNode`/`SpeedManager` 조합):
   - **N16** SEARCH_BACK 중 실제 reconcile 소진 → 같은 tick 에 cancel+FAULT (콜백 경로).
   - **N17** 콜백이 이미 유실됐다고 가정(술어만 True) → 다음 tick 에 live 가드가 FAULT.
   - **N18** FAULT→SEARCH_BACK 재시도 시 `request_guide()` 재무장 → 소진 술어 해제,
     즉시 재-FAULT 하지 않음.
   - 기존 N8/N9·N11·N13·N14/N15 녹색 유지.
   - T자·**쌍굴** 정상 설정 역회귀(E2E) — 4차 §24.5 가 쌍굴 실행 증거를 빠뜨린 P2 동봉.
6. ★ **적색 확인** — live 가드를 제거하면 **N17 이 실패**하는지 확인 후 되돌린다.

## 다음 묶음(2/3) 착수 예고 — GoalManager 헌장 (★ 이번 5차엔 손대지 않음)

> B(취소 종결 게이트)를 여기서 담당한다. 5차 통과 후 별도 묶음으로 착수.

1. send_goal·응답 seq 검증·뒤늦은 수락 즉시취소·cancel 확인 사슬·stale 결과 감시를
   Manager 내부로 이관 (기존 goal_seq 방식 유지 — 동작 불변)
2. MissionNode 에는 "어떤 상태에서 어디로 간다" 정책 + on_reached/enter_fault 콜백만
3. 기존 `test_goal_lifecycle.py` 15개 시나리오 전부 새 이음새에서 녹색 유지
4. SpeedManager 와 동일한 요령: 껍데기 노드 + 진짜 Manager 조합 공격 테스트
5. **공격 테스트 목록 (구현 전 확정 — 기존 15종 이관 + 신규 3종)**
   - 이관: 뒤늦게 수락된 stale goal 즉시 취소 / 거절된 stale goal 은 취소 불필요 /
     현재 goal 은 저장하고 취소 안 함 / 수락 후 취소가 handle 에 도달 / result 가 handle 정리 /
     stale result 무시 / cancel 응답 `goals_canceling` 확인·빈 응답 경고 /
     stale goal 최종 결과 감시(CANCELED 는 조용히, 그 외는 에러) /
     예외 4종: goal_response 예외→FAULT, stale goal_response 예외→조용히,
     result 예외→FAULT, cancel 호출 예외→방어
   - 신규: ① reset/abort 로 seq 가 오른 뒤 도착한 in-flight 응답·결과가 새 goal 을 죽이지 않음
     ② 연속 send_goal 2회에서 이전 handle 이 취소 확인까지 도달 ③ handle 없는 상태의 취소 요청이
     조용히 무사 통과 (goal_active 잔존 금지)
6. ★ **B — GUIDE 저속 상실 취소의 종결 직렬화** (07-20 §12 에서 이관, Codex §12 확정 문구):
   - "live 저속 상실 즉시 취소"가 **아니다**. **상위 정책이 실제로 cancel 을 결정한 시점**
     (SpeedManager 제한 복구 소진 = `_on_guide_failed`)부터 `guide_stop_pending=True` 유지.
   - pending 동안엔 저속이 0.12 로 다시 확인돼도 **CANCELED 종결 전 신규 goal 재전송 금지**
     (대체 goal 직렬화 — `on_result` stale 분기 `mission_node.py:622` 가 종결 관측점).
   - 취소 호출 예외·응답 예외·빈 `goals_canceling`·deadline 초과·non-CANCELED 종결
     → FAULT, 재전송 금지. 그 뒤 live 술어 True 면 다음 tick 에 escape 재전송.
7. ★ **SpeedManager 보완 4회에서 얻은 교훈을 선반영** (`0720_현황.md §23.2`·`docs/PITFALLS.md §8`)
   - **요청/과거 ≠ 현재 실효값** — 게이트가 참조하는 술어도 latch(과거)면 안 된다.
     07-20 P1 4건 중 3건이 이 함정(`AGENTS.md §3-3`) — 그 중 §12 는 소비 지점 게이트의
     **입력 자체가 stale latch** 였다(위치는 옳고 술어가 틀림).
   - **위험한 동작은 소비 지점에서 fail-closed** — 진입 경로 세기는 3회 연속 실패했다.
   - 비동기 요청엔 요청 단위 deadline / Manager 만 찌르지 말고 `tick` 진입점 통과 테스트.
   - ★ **테스트가 실제로 적색이 되는지 확인** — N13 초판은 통과만 하고 버그를 고정했다.

## 허용 파일/범위 (이번 5차)

- `src/mission_manager/mission_manager/speed_manager.py` (live 술어 `guide_speed_recovery_exhausted` 신설)
- `src/mission_manager/mission_manager/mission_node.py` (tick live 가드 + 재시도 재무장 + 콜백 backstop)
- `src/mission_manager/test/test_speed_manager.py` (N16~N18)

## 금지 범위

**B(취소 종결 게이트)·GoalManager 이관·FSM 순수화·인지 기능·make_map 실런을 이번 묶음에 섞지 않는다.**
`cancel_current_goal()`은 기존 호출 그대로 쓴다 — 그 취소의 CANCELED 종결 직렬화는 여전히 B/GoalManager.
지도 자산 불변. (SpeedManager 의 비차단 보류 항목은 `0720_현황.md §23.5`.)

## 보존해야 할 동작·안전 불변조건

- 뒤늦게 수락된 stale goal 은 '무시'가 아니라 즉시 취소 + 최종 결과 감시 (07-19 P0)
- 취소는 '요청'이 아니라 접수·종결 확인까지 (Codex §3.2) / 콜백 예외 = FAULT 정리 (S1-4)
- abort_e2e 의 실정지·cmd_vel 잠잠·취소 접수 로그 기준 유지
- **주행 중 stale 표류 = reconcile→소진 시 cancel+FAULT (§22.3) — 즉시 정지 아님**

## 완료 판정 + 필수 테스트

동작 불변: **기존 테스트 전량**(기준선 수치 정본 = `TEST_GATES.md` §1) + 새 공격 테스트
전부 녹색 → `TEST_GATES.md` §1 전체 게이트
(pytest → colcon → negative → 3goals → mission → abort, 각 프로세스 완전 종료 후 순차)
전량 PASS → 변경 파일·책임 이동표·테스트 수치·남은 위험을 `0720_현황.md` 다음 절(§24 신설)에
기록 → 한 커밋+push → `bash tools/doc_check.sh --after-push`.
검토자 실행 범위는 `TEST_GATES.md §7` (구현자는 §1 전량 그대로).

## 완료 후 다음 단계

5차 Codex 통과 = **새 P1 에 허용된 마지막 연장 검토 종료 → SpeedManager 동결** →
**GoalManager 추출 (구조 분리 2/3, 위 헌장)** → E2E 공통 하네스 3/3 → platform-core-freeze
게이트. 병렬: 역할 B V1 세부 미팅 / micro-ROS 구동부 대기.

## 근거 문서의 정확한 절

`docs/MASTER_PLAN.md §2-2` · `~/Desktop/개발현황/0720_현황.md §23`(SpeedManager 보완 기록·요령) ·
`~/Desktop/개발현황/0719_실차전환_마스터플랜.md §7.3`(구조 분리 근거 — 계획 이력 정본)
