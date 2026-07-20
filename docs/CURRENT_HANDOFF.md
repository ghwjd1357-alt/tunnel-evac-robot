# CURRENT_HANDOFF.md — 지금 할 한 묶음 (매 세션 시작점)

> 작업 완료 시 구현자가 갱신하고 검토자가 사실관계·테스트 결과를 확인한다.
> 이전 묶음 내용은 날짜별 현황으로 보낸다. 항상 이 한 묶음만 유지.

- **현재 기준 커밋**: 이 파일과 같은 커밋 (main — SpeedManager 보완 3차, 07-20)
- **현재 단계**: 마스터플랜 12단계 중 4단 — 구조 분리 **1/3 보완 3차 완료**,
  **Codex 재검토 대기**. 통과해야 2/3(GoalManager) 착수 (`CODEX 현황/0720검토현황.md §11.7`)
- **직전 완료**: 재검토 3차 P1 봉합 (`0720_현황.md §23` — GUIDE 주행을 fail-closed
  게이트로. 저속 '적용 확인' 전에는 진입 경로와 무관하게 주행 goal 0건)
  + `TEST_GATES §7` 보완 3건(재실행/재독해 구분·fail-closed 기본 행)
- **이번 한 묶음 목표** (★ Codex 최종 재검토 통과 후 착수): **GoalManager 동작 불변 추출** (goal 전송·cancel 확인 사슬·
  stale goal 방어를 MissionNode 에서 분리 — `docs/MASTER_PLAN.md §2-2`, "최대 수익")

## 완료조건

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
6. ★ **SpeedManager 보완 3회에서 얻은 교훈을 선반영** (`0720_현황.md §23.2`·`docs/PITFALLS.md §8`)
   - **요청 전송 ≠ 완결** — `goal_active` 가 "보냈다"와 "살아있다"를 겸직하지 않는지 확인.
     07-20 P1 3건 중 2건이 이 함정이었다 (`AGENTS.md §3-3`).
   - **위험한 동작은 소비 지점에서 fail-closed 로 막는다** — 진입 경로를 세는 방식은
     07-20 에 세 번 연속 실패했다. goal 을 '보내는 지점'에서 전제를 검사할 것.
   - 비동기 요청엔 요청 단위 deadline (cancel 확인이 영영 안 오는 경우)
   - Manager 만 찌르지 말고 **노드의 진짜 진입점**(`on_cmd`·`tick`)을 통과하는 테스트 포함
   - ★ **테스트가 실제로 적색이 되는지 확인** — N13 초판은 통과만 하고 버그를 고정했다

## 허용 파일/범위

- `src/mission_manager/mission_manager/` (goal_manager.py 신설 + mission_node.py 의 goal 부분 이관)
- `src/mission_manager/test/`

## 금지 범위

**SpeedManager 재수정·FSM·인지 기능·make_map 실런을 이번 묶음에 섞지 않는다.** 지도 자산 불변.
(SpeedManager 의 비차단 보류 항목은 `0720_현황.md §23.5` — 실측 사례 전 재논의 금지)

## 보존해야 할 동작·안전 불변조건

- 뒤늦게 수락된 stale goal 은 '무시'가 아니라 즉시 취소 + 최종 결과 감시 (07-19 P0)
- 취소는 '요청'이 아니라 접수·종결 확인까지 (Codex §3.2) / 콜백 예외 = FAULT 정리 (S1-4)
- abort_e2e 의 실정지·cmd_vel 잠잠·취소 접수 로그 기준 유지

## 완료 판정 + 필수 테스트

동작 불변: **기존 테스트 전량**(기준선 수치 정본 = `TEST_GATES.md` §1) + 새 공격 테스트
전부 녹색 → `TEST_GATES.md` §1 전체 게이트
(pytest → colcon → negative → 3goals → mission → abort, 각 프로세스 완전 종료 후 순차)
전량 PASS → 변경 파일·책임 이동표·테스트 수치·남은 위험을 `0720_현황.md` 다음 절(§24 신설)에
기록 → 한 커밋+push → `bash tools/doc_check.sh --after-push`.
검토자 실행 범위는 `TEST_GATES.md §7` (구현자는 §1 전량 그대로).

## 완료 후 다음 단계

E2E 공통 하네스 추출 (구조 분리 3/3) → platform-core-freeze 게이트.
병렬: 역할 B V1 세부 미팅 / micro-ROS 구동부 대기.

## 근거 문서의 정확한 절

`docs/MASTER_PLAN.md §2-2` · `~/Desktop/개발현황/0720_현황.md §23`(SpeedManager 보완 기록·요령) ·
`~/Desktop/개발현황/0719_실차전환_마스터플랜.md §7.3`(구조 분리 근거 — 계획 이력 정본)
