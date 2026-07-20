# CURRENT_HANDOFF.md — 지금 할 한 묶음 (매 세션 시작점)

> 작업 완료 시 구현자가 갱신하고 검토자가 사실관계·테스트 결과를 확인한다.
> 이전 묶음 내용은 날짜별 현황으로 보낸다. 항상 이 한 묶음만 유지.

- **현재 기준 커밋**: 이 파일과 같은 커밋 (main — SpeedManager 추출 완료, 07-20)
- **현재 단계**: 마스터플랜 12단계 중 4단 — platform-core 구조 분리 **2/3**
- **직전 완료**: SpeedManager 동작 불변 추출 (`0719_현황.md §19` — 책임 이동표·
  테스트 수치·남은 위험 3건 기록. 미준비 정책 = **timeout 30초 후 FAULT** 사용자 확정)
- **이번 한 묶음 목표**: **GoalManager 동작 불변 추출** (goal 전송·cancel 확인 사슬·
  stale goal 방어를 MissionNode 에서 분리 — `docs/MASTER_PLAN.md §2-2`, "최대 수익")

## 완료조건

1. send_goal·응답 seq 검증·뒤늦은 수락 즉시취소·cancel 확인 사슬·stale 결과 감시를
   Manager 내부로 이관 (기존 goal_seq 방식 유지 — 동작 불변)
2. MissionNode 에는 "어떤 상태에서 어디로 간다" 정책 + on_reached/enter_fault 콜백만
3. 기존 `test_goal_lifecycle.py` 15개 시나리오 전부 새 이음새에서 녹색 유지
4. SpeedManager 추출과 동일한 요령: 껍데기 노드 + 진짜 Manager 조합 공격 테스트
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

## 허용 파일/범위

- `src/mission_manager/mission_manager/` (goal_manager.py 신설 + mission_node.py 의 goal 부분 이관)
- `src/mission_manager/test/`

## 금지 범위

**SpeedManager 재수정·FSM·인지 기능·make_map 실런을 이번 묶음에 섞지 않는다.** 지도 자산 불변.
(SpeedManager 의 비차단 보류 3건은 `0719_현황.md §19.4` — 실측 사례 전 재논의 금지)

## 보존해야 할 동작·안전 불변조건

- 뒤늦게 수락된 stale goal 은 '무시'가 아니라 즉시 취소 + 최종 결과 감시 (07-19 P0)
- 취소는 '요청'이 아니라 접수·종결 확인까지 (Codex §3.2) / 콜백 예외 = FAULT 정리 (S1-4)
- abort_e2e 의 실정지·cmd_vel 잠잠·취소 접수 로그 기준 유지

## 완료 판정 + 필수 테스트

동작 불변: pytest 108개 + 새 공격 테스트 전부 녹색 → `TEST_GATES.md` §1 전체 게이트
(pytest → colcon → negative → 3goals → mission → abort, 각 프로세스 완전 종료 후 순차)
전량 PASS → 변경 파일·책임 이동표·테스트 수치·남은 위험을 `0719_현황.md` 다음 절에
기록 → 한 커밋+push.

## 완료 후 다음 단계

E2E 공통 하네스 추출 (구조 분리 3/3) → platform-core-freeze 게이트.
병렬: 역할 B V1 세부 미팅 / micro-ROS 구동부 대기.

## 근거 문서의 정확한 절

`docs/MASTER_PLAN.md §2` · `~/Desktop/개발현황/0719_현황.md §19`(SpeedManager 기록·요령) ·
`~/Desktop/개발현황/0719_실차전환_마스터플랜.md §7.3`(구조 분리 근거 — 계획 이력 정본)
