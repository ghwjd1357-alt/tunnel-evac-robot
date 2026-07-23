# CURRENT_HANDOFF.md — 지금 할 한 묶음 (매 세션 시작점)

> 작업 완료 시 구현자가 갱신하고 검토자가 사실관계·테스트 결과를 확인한다.
> 이전 묶음 내용은 날짜별 현황으로 보낸다. 항상 이 한 묶음만 유지.

- **현재 기준 커밋**: 이 파일과 같은 커밋 (main — SpeedManager 동결, 07-23)
- **현재 단계**: 마스터플랜 12단계 중 4단 — 구조 분리 **2/3 GoalManager 착수 대기**.
- **직전 완료**: SpeedManager 1/3 + 보완 6회. 마지막 §14 P1은 SEARCH_BACK 신규
  goal 소비 지점도 `guide_speed_applied`로 막아, stale 0.26 reconcile 중과
  FAULT 재무장 응답 대기 중 모두 적용 확인 전 신규 goal 0건으로 봉합했다.
  구현·동일 주체 검증 기록 = `0720_현황.md §26`; 마지막 독립 검토 =
  `CODEX 현황/0720검토현황.md §14`.
- **역할 예외**: 사용자 명시 승인으로 마지막 최소 봉합은 Codex가 `CLAUDE.md` 구현자
  절차를 따라 구현한 뒤 동일 세션에서 재검토했다. 독립 검토는 아니며 §26/§15에 명시했다.

## 이번 한 묶음 목표 — GoalManager 추출 (구조 분리 2/3)

goal 전송·응답·취소·최종 결과의 비동기 수명주기를 `GoalManager`로 옮긴다.
MissionNode에는 “어떤 상태에서 어디로 간다” 정책과 `on_reached`/`enter_fault`
콜백만 남긴다. 기존 `goal_seq` 세대 방식과 외부 동작은 보존한다.

## 완료조건

1. `send_goal`, goal response seq 검증, 뒤늦은 수락 즉시 취소, cancel 응답 확인,
   result/stale result 감시를 Manager 내부로 이관한다.
2. MissionNode는 상태 정책·목적지 선택·도착/FAULT 콜백만 소유한다.
3. 기존 `test_goal_lifecycle.py` 15개 시나리오를 새 Manager 이음새로 전부 이관하고
   껍데기 MissionNode + 진짜 Manager 조합 테스트를 둔다.
4. 구현 전 확정 공격 목록:
   - stale 수락 goal 즉시 취소 / stale 거절은 취소 불필요
   - 현재 goal 저장 / 수락 후 취소가 handle에 도달 / result가 handle 정리
   - stale result 무시 / cancel `goals_canceling` 확인 / 빈 응답 경고
   - stale goal 최종 결과: CANCELED는 조용히, 그 외는 error
   - 예외 4종: 현재 goal_response·result·cancel 예외→FAULT, stale response 예외→무시
   - reset/abort로 seq가 오른 뒤 도착한 응답·결과가 새 goal을 죽이지 않음
   - 연속 send_goal에서 이전 handle이 취소 확인까지 도달
   - handle 없는 취소 요청이 조용히 통과하고 `goal_active`가 남지 않음
5. ★ GUIDE 저속 상실 취소 종결 직렬화(B):
   - live 저속 상실 즉시 취소가 아니라, SpeedManager 복구 소진으로 상위 정책이
     cancel을 결정한 시점부터 `guide_stop_pending=True`.
   - CANCELED 최종 종결 전에는 저속 0.12가 다시 확인돼도 신규 goal 재전송 금지.
   - cancel 호출/응답 예외, 빈 `goals_canceling`, deadline 초과, non-CANCELED 종결
     → FAULT + 재전송 금지.
6. ★ SpeedManager 동결 회귀 N13~N20 보존:
   - GUIDE와 SEARCH_BACK 모두 신규 goal은 `guide_speed_applied=True`에서만 전송.
   - 일시적 stale 표류 중 이미 주행 중인 goal은 즉시 취소하지 않음.
   - 복구 소진 시 cancel+FAULT, 통보 유실은 tick live 가드가 backstop.
7. 새 방어 테스트는 실제 가드를 임시 무력화했을 때 적색이 되는지 확인한다.

## 허용 파일/범위

- `src/mission_manager/mission_manager/goal_manager.py` (신설)
- `src/mission_manager/mission_manager/mission_node.py` (goal 수명주기 이관·정책 콜백 연결)
- `src/mission_manager/test/test_goal_lifecycle.py`
- 필요 시 `src/mission_manager/setup.py` 또는 패키지 등록 파일

## 금지 범위

SpeedManager 동작 변경·FSM 순수화·인지 기능·Nav2 파라미터·지도 자산·`make_map.sh`
실런을 섞지 않는다. SEARCH_BACK 신규 goal live 게이트와 N19/N20은 동결 앵커다.

## 보존해야 할 안전 불변조건

- 호출≠접수≠종결≠실효: cancel은 CANCELED 최종 종결까지 확인한다.
- 뒤늦게 수락된 stale goal은 즉시 취소하고 최종 결과까지 감시한다.
- seq가 바뀐 뒤의 늦은 콜백이 현재 goal·FAULT·상태를 덮지 않는다.
- abort E2E의 FAULT·실정지·cmd_vel 잠잠·취소 접수 로그 기준을 유지한다.
- GUIDE/SEARCH_BACK 신규 goal은 저속 적용 확인 전 0건이다.

## 완료 판정 + 필수 테스트

변경 직후 pytest, 묶음 완료 시 `TEST_GATES.md §1` 전량을 순차 실행한다.
테스트 수치·책임 이동표·남은 위험을 `0720_현황.md` 다음 절에 기록하고,
`bash tools/doc_check.sh --strict` → 한 커밋+push →
`bash tools/doc_check.sh --after-push` 순서를 지킨다.
검토자 실행 범위는 `TEST_GATES.md §7`.

## 완료 후 다음 단계

GoalManager 독립 검토 통과 → E2E 공통 하네스 3/3 → platform-core-freeze 게이트.

## 근거 문서

`docs/MASTER_PLAN.md §2` · `docs/PITFALLS.md §8` ·
`~/Desktop/개발현황/0720_현황.md §19~§26` ·
`~/Desktop/개발현황/CODEX 현황/0720검토현황.md §3~§14`
