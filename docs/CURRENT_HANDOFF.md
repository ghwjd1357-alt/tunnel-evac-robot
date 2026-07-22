# CURRENT_HANDOFF.md — 지금 할 한 묶음 (매 세션 시작점)

> 작업 완료 시 구현자가 갱신하고 검토자가 사실관계·테스트 결과를 확인한다.
> 이전 묶음 내용은 날짜별 현황으로 보낸다. 항상 이 한 묶음만 유지.

- **현재 기준 커밋**: 이 파일과 같은 커밋 (main — SpeedManager 보완 4차 진행, 07-20)
- **현재 단계**: 마스터플랜 12단계 중 4단 — 구조 분리 **1/3 보완 4차 (진행 중)**.
  4차 통과 후 2/3(GoalManager) 착수 (`CODEX 현황/0720검토현황.md §12`)
- **직전 완료**: 재검토 3차 P1 봉합 (`0720_현황.md §23`) — 그러나 §12 에서 그 게이트의
  **술어가 stale latch** 였음이 드러남(guide 과거 1회 성공 ≠ 지금 저속 적용).
- **이번 한 묶음 목표 = SpeedManager 보완 4차 (A 한정)**: 소비 지점 게이트의 술어를
  **latch → live** 로 교정한다. `guide_speed_applied`(지금 적용값이 저속인가) 신설 +
  내부 걸쇠 `_guide_confirmed → _guide_was_confirmed` 개명(겸직 오독 방지).
  이것이 `§12` P1(늦은 sync 0.26 이 덮은 뒤 새 escape goal 전송)의 정확한 봉합이다.
  ⚠ **B(취소 종결 게이트)는 이 묶음에 넣지 않는다 — GoalManager 로 이관** (아래 착수 예고).
  근거: 주행 중 일시적 표류는 기존 reconcile→소진 시 cancel+FAULT(`§22.3`)가 이미 담당,
  "즉시 정지"는 그보다 엄격한 새 정책이라 §12 P1 필수조건 아님(Codex §12 철회 확인).

## 완료조건 (SpeedManager 보완 4차 — A 한정)

1. **`guide_speed_applied` (live) 신설** — `_desired` origin 이 guide 이고 `_applied`
   가 그 요구값과 같을 때만 True. "guide 가 한 번이라도 성공했나"(과거)가 아니라
   "지금 controller 에 저속이 적용돼 있나"(현재)를 뜻한다. `_on_stale_result` 가
   `_applied` 를 0.26 으로 덮으면 즉시 False 가 되어야 한다.
2. **내부 걸쇠 개명** `_guide_confirmed → _guide_was_confirmed` — public live 와
   거의 같은 이름을 남기면 다음 검토가 겸직으로 오독한다(§22 전례). 걸쇠는
   `_settle` 재조정 자격·유지실패 분류 전용으로 남긴다.
3. **소비 지점 게이트가 live 를 참조** (`mission_node.py` GUIDE 분기) — 신규 goal
   전송만 가른다. 이미 주행 중인 goal 은 건드리지 않는다(그 정지는 §22.3·B 소관).
4. 공격 테스트 (구현 전 확정, `MissionNode.tick()` 진입):
   - **N14** goal 없음 + stale 0.26: live=False → 신규 goal 0건, reconcile 성공 후 1건.
   - **N15** 주행 중 + 일시적 stale 0.26: **즉시 cancel 안 함**, reconcile 성공 뒤에도
     기존 goal 유지 + 중복 goal 0건 (A 가 주행을 건드리지 않는다는 회귀 가드).
   - 기존 N8/N9(소진 시 cancel+FAULT)·N11(금지상태 부재)·N13(FAULT 복귀) 녹색 유지.
   - T자·쌍굴 정상 설정 역회귀(E2E).
5. ★ **테스트가 실제로 적색이 되는지 확인** — 게이트를 옛 latch 로 임시 되돌려 N14 가
   0.26 에서 escape 를 쏘는지(적색) 확인 후 되돌린다 (N13 초판 교훈).

## 다음 묶음(2/3) 착수 예고 — GoalManager 헌장 (★ 이번 4차엔 손대지 않음)

> B(취소 종결 게이트)를 여기서 담당한다. 4차 통과 후 별도 묶음으로 착수.

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

## 허용 파일/범위 (이번 4차)

- `src/mission_manager/mission_manager/speed_manager.py` (property/걸쇠 개명 + live 술어)
- `src/mission_manager/mission_manager/mission_node.py` (GUIDE 게이트 술어 1줄)
- `src/mission_manager/test/test_speed_manager.py`

## 금지 범위

**B(취소 종결 게이트)·GoalManager 이관·FSM·인지 기능·make_map 실런을 이번 묶음에 섞지 않는다.**
주행 중 표류의 '즉시 정지'는 넣지 않는다(§22.3 유지 — reconcile 먼저, 소진 시에만 cancel+FAULT).
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

4차 Codex 통과 → **GoalManager 추출 (구조 분리 2/3, 위 헌장)** → E2E 공통 하네스 3/3
→ platform-core-freeze 게이트. 병렬: 역할 B V1 세부 미팅 / micro-ROS 구동부 대기.

## 근거 문서의 정확한 절

`docs/MASTER_PLAN.md §2-2` · `~/Desktop/개발현황/0720_현황.md §23`(SpeedManager 보완 기록·요령) ·
`~/Desktop/개발현황/0719_실차전환_마스터플랜.md §7.3`(구조 분리 근거 — 계획 이력 정본)
