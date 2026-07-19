# CURRENT_HANDOFF.md — 지금 할 한 묶음 (매 세션 시작점)

> 작업 완료 시 구현자가 갱신하고 검토자가 사실관계·테스트 결과를 확인한다.
> 이전 묶음 내용은 날짜별 현황으로 보낸다. 항상 이 한 묶음만 유지.

- **현재 기준 커밋**: `8042464` (main, 원격 push 완료)
- **현재 단계**: 마스터플랜 12단계 중 4단 — platform-core 구조 분리 **1/3**
- **이번 한 묶음 목표**: **SpeedManager 동작 불변 추출** (속도 변경 비동기 수명주기를 MissionNode 에서 분리)

## 완료조건 (0719_현황.md §18.3 전문 — 요약 아님)

1. 속도 SetParameters 요청·성공 확인·3회 재시도·purpose 별 최종처리를 Manager 내부로 이관
2. generation/token 으로 guide·sync·restore 의 stale 성공·실패 응답을 모두 구분
3. token 은 콜백을 무시하는 데서 끝내지 않고, 오래된 요청이 원격 controller 에 늦게 적용되면
   **현재 desired speed 로 재조정(reconcile)** ← Codex §14.3 필수 조건
4. controller parameter 서비스 **장기 미준비 정책 확정** — 현재는 GATHER 무기한 안전대기.
   timeout 후 FAULT 또는 관제대기 중 하나를 명시하고 테스트로 고정
   ★ **미결 사용자 결정** — Claude 추천 = timeout 후 FAULT (무기한 대기는 관제 화면에서 고장 은폐,
   FAULT 는 즉시 가시화 + reset/abort 복구 경로 존재). 구현 착수 전 사용자 확인.
5. 공격 테스트: guide↔sync↔restore 응답 역전, reset/abort 중 in-flight 요청,
   늦은 성공·실패, 3회 실패, call_async 예외, 장기 service-unready
   (+ reconcile 자체의 stale 방지 — 최신 generation 만 유효 규칙을 reconcile 에도 적용, 핑퐁 금지)
6. MissionNode 에는 "어떤 상태에서 어떤 속도를 원한다"는 정책만 남기고 비동기 수명주기 제거

## 허용 파일/범위

- `src/mission_manager/mission_manager/` (speed_manager.py 신설 + mission_node.py 의 속도 부분 이관)
- `src/mission_manager/test/` (공격 테스트 추가 — `MissionNode.__new__` 우회 기법: 0719_현황.md §12 참조)

## 금지 범위

**GoalManager·FSM·인지 기능·make_map 실런을 이번 묶음에 섞지 않는다.** 지도 자산 불변.

## 보존해야 할 동작·안전 불변조건

- GUIDE 진입은 저속 적용 '성공 응답' 후에만 (F2) / 3회 실패·예외 = goal 취소+FAULT
- reset/abort 후 늦은 응답이 상태·FAULT 를 덮지 않음 (G1 가드의 정석 해법으로 승격)
- `_speed_synced` 는 성공 콜백에서만 True / mission_e2e 의 GUIDE 0.12 실측 assert 유지

## 완료 판정 + 필수 테스트

동작 불변: 기존 pytest 96개 + 새 공격 테스트 전부 녹색 → `TEST_GATES.md` §1 전체 게이트
(pytest → colcon → negative → 3goals → mission → abort, 각 프로세스 완전 종료 후 순차) 전량 PASS
→ 변경 파일·책임 이동표·테스트 수치·남은 위험을 `0719_현황.md` 다음 절에 기록 → 한 커밋+push.

## 완료 후 다음 단계

GoalManager 추출 (마스터플랜 §2-2). 병렬: 역할 B V1 세부 미팅 / micro-ROS 구동부 대기.

## 근거 문서의 정확한 절

`0719_현황.md §18.3~18.4` · `CODEX 현황/0719검토현황.md §14.3` · Desktop 마스터플랜 §7.3-1
