# -*- coding: utf-8 -*-
"""
test_speed_manager.py — 속도 변경 비동기 수명주기 공격 테스트 (07-20 구조 분리 1/3)
============================================================
[무엇을 잡나 — 0719_현황.md §18.3 완료조건 5]
  guide↔sync↔restore 응답 순서 역전, reset/abort 중 in-flight 요청,
  늦은 성공·실패, 3회 실패, call_async 예외, 장기 service-unready,
  그리고 reconcile 자체의 stale 방지(최신 세대만 유효 — 핑퐁 금지).

[이관 이력]
  기존 test_goal_lifecycle.py 의 S1-5·F2·G1 속도 테스트 12개 시나리오를
  SpeedManager 이음새로 이관 — 동작 불변 앵커로 전부 유지.

[테스트 기법]
  MissionNode.__new__ 로 껍데기 노드를 만들고(무거운 __init__ 우회 — §12),
  진짜 SpeedManager 를 가짜 서비스 클라이언트·손으로 돌리는 시계와 함께
  붙인다. 응답 콜백을 임의 순서로 주입해 실서비스로는 재현 불가능한
  ms 단위 레이스를 결정적으로 재현한다.
"""

import types

from mission_manager.mission_node import MissionNode, State
from mission_manager.speed_manager import SpeedManager


# ============================================================
# 가짜 부품
# ============================================================
def ok_resp():
    res = types.SimpleNamespace(results=[
        types.SimpleNamespace(successful=True, reason='')])
    return types.SimpleNamespace(result=lambda: res)


def fail_resp(reason='rejected'):
    res = types.SimpleNamespace(results=[
        types.SimpleNamespace(successful=False, reason=reason)])
    return types.SimpleNamespace(result=lambda: res)


def make_env(ready=True, state=State.GATHER, timeout=30.0):
    """껍데기 MissionNode + 진짜 SpeedManager + 가짜 서비스/시계.

    반환 node 에 계측 필드:
      node._logs / node._faults / node._cancels
      node._calls  = call_async 로 나간 속도값 리스트 (요청 순서)
      node._cbs    = (속도값, 응답콜백) 리스트 — 임의 순서 주입용
      node._ready  = {'ready': bool} 서비스 준비 토글
      node._clock  = {'t': float} monotonic 시계 (손으로 전진)
    """
    node = MissionNode.__new__(MissionNode)
    logs = []
    logger = types.SimpleNamespace(
        warn=lambda msg, **kw: logs.append(msg),
        info=lambda msg, **kw: logs.append(msg),
        error=lambda msg, **kw: logs.append(msg))
    node.get_logger = lambda: logger
    node._logs = logs
    node.state = state
    node._guide_pending = False
    node._faults = []
    node._cancels = []
    node.cancel_current_goal = lambda: node._cancels.append(1)
    node.enter_fault = lambda: node._faults.append(1)

    calls, cbs = [], []
    ready_box = {'ready': ready}
    clock = {'t': 0.0}

    def call_async(req):
        v = req.parameters[0].value.double_value
        calls.append(v)
        fut = types.SimpleNamespace()
        fut.add_done_callback = lambda cb: cbs.append((v, cb))
        return fut

    cli = types.SimpleNamespace(
        service_is_ready=lambda: ready_box['ready'],
        call_async=call_async)
    node.speed = SpeedManager(
        cli, logger,
        on_guide_confirmed=node._on_guide_speed_ok,
        on_guide_failed=node._on_guide_speed_fail,
        unready_timeout_sec=timeout,
        now_fn=lambda: clock['t'])
    node._calls = calls
    node._cbs = cbs
    node._ready = ready_box
    node._clock = clock
    return node


def respond(node, idx, resp):
    """idx 번째로 나간 요청의 응답을 주입 (임의 순서 = 역전 재현)."""
    node._cbs[idx][1](resp)


# ============================================================
# S1-5 이관: 속도 변경은 '요청'이 아니라 '확인'까지
# ============================================================
def test_speed_change_confirmed_on_success():
    node = make_env()
    node.speed.request_restore(0.26)
    assert node._calls == [0.26]
    respond(node, 0, ok_resp())
    assert any('변경 확인' in m for m in node._logs)


def test_speed_change_retries_then_errors():
    """실패 응답 → 재시도 2회 → 3회째도 실패면 에러 보고 (조용한 실패 금지)."""
    node = make_env()
    node.speed.request_restore(0.26)
    respond(node, 0, fail_resp())            # 1차 실패 → 재시도
    assert len(node._calls) == 2
    respond(node, 1, fail_resp())            # 2차 실패 → 재시도
    assert len(node._calls) == 3
    respond(node, 2, fail_resp())            # 3차 실패 → 포기+에러
    assert len(node._calls) == 3             # 더 안 쏨
    assert any('3회 실패' in m for m in node._logs)
    assert node._faults == []                # restore 실패는 FAULT 아님


# ============================================================
# F2 이관: GUIDE 는 저속 '확인' 전 주행 금지
# ============================================================
def test_guide_gate_service_not_ready_no_guide():
    """서비스 미준비 — 요청이 안 나가고 GUIDE 진입 없음 (timeout 대기 모드).

    07-20 정책 변경: 기존 '즉시 생략 후 tick 재시도(무기한)' 대신
    Manager 가 timeout 까지 준비를 감시한다 — GATHER 정지 유지는 동일."""
    node = make_env(ready=False)
    node._guide_pending = True
    node.speed.request_guide(0.12)
    assert node.state == State.GATHER        # 주행 안 함
    assert node._calls == []                 # 요청 자체가 안 나감
    assert any('준비 대기' in m for m in node._logs)


def test_guide_gate_success_enters_guide():
    """저속 적용 성공 확인 → 그때서야 GATHER→GUIDE 전환."""
    node = make_env()
    node._guide_pending = True
    node.speed.request_guide(0.12)
    assert node.state == State.GATHER        # 응답 전엔 전환 금지
    respond(node, 0, ok_resp())
    assert node.state == State.GUIDE
    assert not node._guide_pending


def test_guide_gate_three_failures_fault_not_guide():
    """3회 실패 — GUIDE 진입 대신 goal 취소+FAULT (평시 속도 유도 금지)."""
    node = make_env()
    node._guide_pending = True
    node.speed.request_guide(0.12)
    for i in range(3):
        respond(node, i, fail_resp())
    assert node.state == State.GATHER        # GUIDE 로 안 감
    assert node._faults == [1]
    assert node._cancels == [1]
    assert not node._guide_pending


def test_guide_gate_call_exception_fault_not_guide():
    """call_async 자체 예외 3연속 — 전파 금지 + FAULT (Codex §12.3 재현 봉쇄)."""
    node = make_env()

    def explode(req):
        raise RuntimeError('service gone')
    node.speed._cli.call_async = explode
    node._guide_pending = True
    node.speed.request_guide(0.12)           # 예외가 새면 여기서 터짐
    assert node.state == State.GATHER
    assert node._faults == [1]


def test_guide_gate_late_confirm_after_abort_no_transition():
    """확인 대기 중 abort — 늦은 성공 응답이 GUIDE 로 덮으면 안 됨.
    + 늦게 적용된 값 == desired 라 reconcile 도 안 나감 (불필요 재요청 금지)."""
    node = make_env()
    node._guide_pending = True
    node.speed.request_guide(0.12)
    node.state = State.FAULT                 # abort 가 먼저 도착
    node._guide_pending = False
    node.speed.cancel_pending('abort')       # 진행 중 요청 stale 화
    respond(node, 0, ok_resp())              # 늦은 성공 응답
    assert node.state == State.FAULT         # 그대로
    assert len(node._calls) == 1             # v==desired → reconcile 불필요


def test_sync_true_only_after_confirmed():
    """synced 는 성공 '응답'에서만 True — 요청 직후 True 금지 (Codex §12.3)."""
    node = make_env()
    node.speed.ensure_sync(0.26)
    assert node._calls == [0.26]
    assert not node.speed.synced             # 응답 전
    node.speed.ensure_sync(0.26)             # inflight 중 중복 요청 금지 (멱등)
    assert len(node._calls) == 1
    respond(node, 0, ok_resp())
    assert node.speed.synced


def test_sync_final_failure_sets_cooldown_not_synced():
    """sync 3회 실패 — synced False 유지, cooldown 소진 후에만 재요청."""
    node = make_env()
    node.speed.ensure_sync(0.26)
    for i in range(3):
        respond(node, i, fail_resp())
    assert not node.speed.synced
    node.speed.ensure_sync(0.26)             # cooldown 중 — 재요청 금지
    assert len(node._calls) == 3
    for _ in range(SpeedManager.SYNC_COOLDOWN_TICKS):
        node.speed.tick()
    node.speed.ensure_sync(0.26)             # cooldown 소진 — 재요청 허용
    assert len(node._calls) == 4


# ============================================================
# G1 이관: 늦은 '실패' 응답도 stale 구분 (유령 FAULT 정석 봉쇄 — 세대 토큰)
# ============================================================
def test_guide_late_failure_after_reset_ignored():
    """guide 요청 중 reset(PATROL) → 늦은 실패 응답은 재시도·FAULT 금지."""
    node = make_env()
    node._guide_pending = True
    node.speed.request_guide(0.12)
    node.state = State.PATROL                # 관제 reset 이 먼저 성공
    node._guide_pending = False
    node.speed.cancel_pending('reset')
    respond(node, 0, fail_resp())            # 이전 세대 늦은 실패
    assert len(node._calls) == 1             # 재시도 안 함
    assert node._faults == []                # 유령 FAULT 없음
    assert node._cancels == []
    assert node.state == State.PATROL        # reset 결과 유지
    assert any('늦은 guide 속도 실패' in m for m in node._logs)


def test_guide_late_third_failure_after_reset_no_final_fault():
    """재시도 2회가 GATHER 중 이미 소모된 뒤 reset — 3회째 늦은 실패가
    최종처리(cancel+FAULT)로 떨어지면 안 됨."""
    node = make_env()
    node._guide_pending = True
    node.speed.request_guide(0.12)
    respond(node, 0, fail_resp())            # 1차 실패 (GATHER 중)
    respond(node, 1, fail_resp())            # 2차 실패 (GATHER 중)
    assert len(node._calls) == 3
    node.state = State.PATROL                # reset 성공
    node._guide_pending = False
    node.speed.cancel_pending('reset')
    respond(node, 2, fail_resp())            # 3회째는 reset 후 도착
    assert node._faults == []                # 최종 FAULT 강등 없음
    assert node._cancels == []
    assert node.state == State.PATROL


def test_guide_failure_while_still_gather_retries():
    """★역회귀 앵커: 상태가 그대로(GATHER+pending)면 기존 재시도 동작 유지."""
    node = make_env()
    node._guide_pending = True
    node.speed.request_guide(0.12)
    respond(node, 0, fail_resp())
    assert len(node._calls) == 2             # 정상 재시도는 그대로


# ============================================================
# ★ 신규: 응답 역전 + reconcile (Codex §14.3 — 무시로 끝내지 않는다)
# ============================================================
def test_reversed_sync_guide_success_reconciles():
    """sync(0.26)·guide(0.12) 동시 in-flight, 응답 역전 —
    guide 성공(GUIDE 진입) 뒤 stale sync 성공이 오면 controller 는 0.26 으로
    뒤늦게 덮인 것 → desired 0.12 로 reconcile 재요청이 나가야 한다."""
    node = make_env()
    node.speed.ensure_sync(0.26)             # 요청 0
    node._guide_pending = True
    node.speed.request_guide(0.12)           # 요청 1 (세대 교체 — sync 는 stale)
    respond(node, 1, ok_resp())              # guide 성공 먼저 → GUIDE
    assert node.state == State.GUIDE
    respond(node, 0, ok_resp())              # ★ 늦은 sync 성공 = 0.26 늦게 적용
    assert node._calls == [0.26, 0.12, 0.12]  # reconcile 로 0.12 재요청
    assert any('재조정' in m for m in node._logs)
    respond(node, 2, ok_resp())              # reconcile 확인 — 추가 요청 없음
    assert len(node._calls) == 3


def test_reconcile_skipped_when_current_request_inflight():
    """stale 성공 도착 시 현재 세대 요청이 응답 대기 중이면 reconcile 생략
    (그 요청이 곧 desired 를 덮는다 — 중복 발사 금지)."""
    node = make_env()
    node.speed.ensure_sync(0.26)             # 요청 0
    node._guide_pending = True
    node.speed.request_guide(0.12)           # 요청 1 — 아직 응답 대기
    respond(node, 0, ok_resp())              # stale sync 성공 먼저 도착
    assert len(node._calls) == 2             # reconcile 안 나감
    assert any('재조정 생략' in m for m in node._logs)
    respond(node, 1, ok_resp())              # 현재 요청이 정상 확인 → GUIDE
    assert node.state == State.GUIDE
    assert len(node._calls) == 2


def test_stale_reconcile_respects_latest_generation_no_pingpong():
    """reconcile 자체도 '최신 세대만 유효' — 낡은 reconcile 성공이
    새 desired 를 또 덮어쓰는 핑퐁 금지."""
    node = make_env()
    node.speed.ensure_sync(0.26)             # 요청 0
    node._guide_pending = True
    node.speed.request_guide(0.12)           # 요청 1
    respond(node, 1, ok_resp())              # GUIDE 진입
    respond(node, 0, ok_resp())              # stale sync → 요청 2 = reconcile(0.12)
    assert node._calls == [0.26, 0.12, 0.12]
    node.speed.request_restore(0.26)         # 요청 3 — 세대 교체 (reconcile stale 화)
    respond(node, 2, ok_resp())              # 낡은 reconcile 성공 (0.12 늦게 적용)
    # 현재 세대(restore 0.26)가 응답 대기 중 → 추가 reconcile 없이 그 요청에 위임
    assert len(node._calls) == 4
    respond(node, 3, ok_resp())              # restore 확인 — 종결, 추가 요청 없음
    assert len(node._calls) == 4


def test_late_applied_old_speed_after_restore_confirmed_reconciles():
    """reset 완료(restore 0.26 확인) 뒤에야 낡은 guide 성공이 도착 —
    controller 가 0.12 로 뒤늦게 덮였으므로 0.26 reconcile 이 나가야 한다
    (Codex §14.3 의 원 시나리오: 상태 전이는 막아도 값이 0.12 로 남는 구멍)."""
    node = make_env()
    node._guide_pending = True
    node.speed.request_guide(0.12)           # 요청 0
    node.state = State.PATROL                # reset
    node._guide_pending = False
    node.speed.cancel_pending('reset')
    node.speed.request_restore(0.26)         # 요청 1
    respond(node, 1, ok_resp())              # restore 확인 완료
    respond(node, 0, ok_resp())              # ★ 그 뒤에 낡은 guide 성공 도착
    assert node._calls == [0.12, 0.26, 0.26]  # 요청 2 = reconcile(0.26)
    assert node.state == State.PATROL        # 상태는 불변


def test_reconcile_final_failure_error_log_only():
    """reconcile 3회 실패 — 에러 보고만, FAULT 아님 (정지/종결 국면)."""
    node = make_env()
    node._guide_pending = True
    node.speed.request_guide(0.12)
    node.state = State.PATROL
    node._guide_pending = False
    node.speed.cancel_pending('reset')
    node.speed.request_restore(0.26)
    respond(node, 1, ok_resp())              # restore 확인
    respond(node, 0, ok_resp())              # 낡은 guide 성공 → reconcile(0.26)
    respond(node, 2, fail_resp())
    respond(node, 3, fail_resp())
    respond(node, 4, fail_resp())            # reconcile 3회 실패
    assert any('3회 실패' in m for m in node._logs)
    assert node._faults == []
    assert node.state == State.PATROL


# ============================================================
# ★ 신규: 장기 service-unready (07-20 사용자 결정 = timeout 후 FAULT)
# ============================================================
def test_unready_guide_sends_when_service_appears():
    """timeout 안에 서비스가 뜨면 — 그때 요청이 나가고 정상 GUIDE 진입."""
    node = make_env(ready=False, timeout=30.0)
    node._guide_pending = True
    node.speed.request_guide(0.12)
    node._clock['t'] = 10.0
    node.speed.tick()                        # 아직 미준비 — 대기 유지
    assert node._calls == []
    assert node._faults == []
    node._ready['ready'] = True
    node._clock['t'] = 20.0
    node.speed.tick()                        # 준비됨 — 이제 발사
    assert node._calls == [0.12]
    respond(node, 0, ok_resp())
    assert node.state == State.GUIDE


def test_unready_guide_timeout_faults():
    """★ 정책 고정: timeout 소진까지 미준비 — FAULT (무기한 GATHER 은폐 금지)."""
    node = make_env(ready=False, timeout=30.0)
    node._guide_pending = True
    node.speed.request_guide(0.12)
    node._clock['t'] = 29.9
    node.speed.tick()
    assert node._faults == []                # 아직 timeout 전
    node._clock['t'] = 30.0
    node.speed.tick()
    assert node._faults == [1]               # timeout → FAULT
    assert node._cancels == [1]
    assert not node._guide_pending
    assert any('미준비' in m and 'FAULT' in m for m in node._logs)
    node._clock['t'] = 60.0
    node.speed.tick()                        # 대기는 1회로 정리 — 중복 FAULT 금지
    assert node._faults == [1]


def test_unready_wait_cancelled_by_reset_no_fault():
    """미준비 대기 중 reset — 대기가 무효화되고 timeout 이 지나도 FAULT 없음."""
    node = make_env(ready=False, timeout=30.0)
    node._guide_pending = True
    node.speed.request_guide(0.12)
    node.state = State.PATROL
    node._guide_pending = False
    node.speed.cancel_pending('reset')
    node._clock['t'] = 100.0
    node.speed.tick()
    assert node._faults == []
    assert node.state == State.PATROL


def test_restore_unready_skipped_with_warning():
    """restore 는 미준비면 경고 후 생략 (기존 동작 유지 — 정지 국면)."""
    node = make_env(ready=False)
    node.speed.request_restore(0.26)
    assert node._calls == []
    assert any('생략' in m for m in node._logs)


def test_sync_unready_waits_silently_then_fires():
    """sync 는 미준비면 조용히 대기 — 준비된 tick 의 ensure_sync 가 발사."""
    node = make_env(ready=False)
    node.speed.ensure_sync(0.26)
    assert node._calls == []
    node._ready['ready'] = True
    node.speed.ensure_sync(0.26)
    assert node._calls == [0.26]


def test_sync_not_refired_after_guide_confirmed():
    """guide 성공 확인 뒤 ensure_sync 가 0.26 을 쏘면 GUIDE 저속을 덮는다 —
    성공 확인된 어떤 요청이든 시작 동기화 목적을 충족한 것으로 간주."""
    node = make_env()
    node._guide_pending = True
    node.speed.request_guide(0.12)
    respond(node, 0, ok_resp())              # GUIDE 진입
    node.speed.ensure_sync(0.26)
    assert len(node._calls) == 1             # sync 재발사 없음 (0.12 보존)


def test_guide_wait_not_overwritten_by_sync():
    """guide 미준비 대기 중 ensure_sync 가 세대를 덮으면 대기가 무효화된다 —
    대기 중엔 sync 발사 금지."""
    node = make_env(ready=False)
    node._guide_pending = True
    node.speed.request_guide(0.12)           # 대기 모드
    node._ready['ready'] = True              # 서비스가 방금 떴다
    node.speed.ensure_sync(0.26)             # tick 순서상 sync 검사가 먼저 올 수 있음
    assert node._calls == []                 # sync 가 가로채면 안 됨
    node.speed.tick()
    assert node._calls == [0.12]             # guide 대기가 정상 발사


# ============================================================
# ★ 0720 Codex 검토 보완 — 부정 회귀 N1~N6 (0720_현황.md §20.7)
#   "desired 가 controller 에 반영 안 된 채 아무도 재시도하지 않는" 종결을 금지한다.
# ============================================================
def drain_failures(node, seen=0):
    """아직 응답 안 준 요청 전부에 실패 응답 주입 (재시도로 늘어나는 것 포함)."""
    while seen < len(node._cbs):
        idx, seen = seen, seen + 1
        node._cbs[idx][1](fail_resp())
    return seen


def arm_stale_guide_over_restore(node):
    """N1·N2·N5 공통 전제: applied=0.12(늦은 guide), desired=0.26(restore in-flight).

    ① 정상 sync 로 synced=True 를 만든다 (ensure_sync 안전망이 죽은 상태 재현)
    ② guide 0.12 in-flight 중 reset → restore 0.26
    ③ 늦은 guide 성공 도착 = controller 에 0.12 가 뒤늦게 적용 (reconcile 은 생략됨)
    """
    node.speed.ensure_sync(0.26)             # 요청 0
    respond(node, 0, ok_resp())              # synced=True, applied=0.26
    node._guide_pending = True
    node.speed.request_guide(0.12)           # 요청 1
    node.state = State.PATROL                # reset
    node._guide_pending = False
    node.speed.cancel_pending('reset')
    node.speed.request_restore(0.26)         # 요청 2 — in-flight
    respond(node, 1, ok_resp())              # ★ 늦은 guide 성공 → applied=0.12
    assert len(node._calls) == 3             # reconcile 생략됨 (기존 최적화)
    assert any('재조정 생략' in m for m in node._logs)


def test_n1_reconcile_skipped_then_current_request_final_fails_recovers():
    """N1 — Codex 0720 P1 반례.

    reconcile 을 '현재 요청이 곧 덮는다'며 생략했는데 그 요청이 최종 실패하면,
    desired(0.26)는 영영 복구되지 않았다 (synced=True 라 ensure_sync 도 무력).
    → 종결 후 tick 이 재평가해 0.26 을 다시 쏴야 한다."""
    node = make_env()
    arm_stale_guide_over_restore(node)
    respond(node, 2, fail_resp())            # restore 1차 실패 → 재시도
    respond(node, 3, fail_resp())            # 2차
    respond(node, 4, fail_resp())            # 3차 = 최종 실패 (에러 로그만)
    assert len(node._calls) == 5
    assert node.speed.synced is True         # ensure_sync 안전망은 죽어 있다
    node.speed.ensure_sync(0.26)
    assert len(node._calls) == 5             # ★ 실제로 아무것도 안 쏜다 (P1 조건)

    node.speed.tick()                        # ★ 종결 재평가가 복구를 담당
    assert node._calls[-1] == 0.26
    assert len(node._calls) == 6
    respond(node, 5, ok_resp())              # 복구 확인 → 더 안 쏨
    node.speed.tick()
    assert len(node._calls) == 6


def test_n2_request_evaporated_by_service_death_recovers():
    """N2 — Claude 발견 2번째 누출 경로 (0720_현황.md §20.3).

    재시도 도중 서비스가 소멸하면 non-guide 요청은 _final_fail 조차 없이
    증발한다. 서비스가 돌아오면 desired 를 다시 쏴야 한다."""
    node = make_env()
    arm_stale_guide_over_restore(node)
    node._ready['ready'] = False             # 재시도 직전 서비스 소멸
    respond(node, 2, fail_resp())            # 실패 → 재시도하려다 증발
    assert len(node._calls) == 3             # 요청이 나가지 못했다

    node.speed.tick()                        # 아직 미준비 — 헛발사 금지
    assert len(node._calls) == 3
    node._ready['ready'] = True
    node.speed.tick()                        # 준비됨 → 복구 발사
    assert node._calls[-1] == 0.26
    assert len(node._calls) == 4


def test_n3_never_completing_future_faults_at_deadline():
    """N3 — 서비스는 ready 인데 응답이 영영 안 오는 경우.

    07-20 확정 정책 = '저속 적용 확인까지 총 30초' → 무기한 GATHER 은폐 금지."""
    node = make_env(timeout=30.0)            # ready=True
    node._guide_pending = True
    node.speed.request_guide(0.12)
    assert node._calls == [0.12]             # 요청은 나갔고 응답이 없다
    node._clock['t'] = 29.9
    node.speed.tick()
    assert node._faults == []                # 아직 예산 안 소진
    node._clock['t'] = 30.0
    node.speed.tick()
    assert node._faults == [1]               # ★ FAULT 로 가시화
    assert node._cancels == [1]
    assert not node._guide_pending
    node._clock['t'] = 300.0
    node.speed.tick()
    assert node._faults == [1]               # 중복 FAULT 금지


def test_n4_late_response_after_deadline_does_not_revive_guide():
    """N4 — deadline 으로 포기한 요청의 응답이 뒤늦게 성공으로 와도
    FAULT 를 덮고 GUIDE 로 되살아나면 안 된다 (stale 규칙이 그대로 적용)."""
    node = make_env(timeout=30.0)
    node._guide_pending = True
    node.speed.request_guide(0.12)
    node._clock['t'] = 30.0
    node.speed.tick()                        # deadline → FAULT
    assert node._faults == [1]
    respond(node, 0, ok_resp())              # ★ 그 뒤 늦은 성공 도착
    assert node.state != State.GUIDE         # 되살아나지 않음
    assert node._faults == [1]
    assert len(node._calls) == 1             # 불필요한 reconcile 도 없음


def test_n5_permanently_broken_controller_terminates_finitely():
    """N5 — controller 가 영구 고장이어도 재평가가 무한 재발사로 번지면 안 된다
    (Codex §5 '유한 종결' 확인 항목). 상한 소진 후엔 조용해진다."""
    node = make_env()
    arm_stale_guide_over_restore(node)
    seen = drain_failures(node, 2)           # restore 3회 실패
    for _ in range(500):                     # tick 을 아무리 돌려도
        node.speed.tick()
        seen = drain_failures(node, seen)    # 나가는 족족 실패
    total = len(node._calls)
    assert total < 30, f'무한 재발사 의심 — 총 {total}회'
    assert any('재조정 포기' in m for m in node._logs)
    before = len(node._calls)
    for _ in range(100):
        node.speed.tick()
    assert len(node._calls) == before        # 완전히 종결 — 추가 발사 0


# ============================================================
# ★ N6: MissionNode.on_cmd() wiring (Codex 0720 P2 — 테스트 공백)
#   기존 24개는 cancel_pending 을 손으로 불러서, mission_node 쪽 호출이
#   빠지거나 순서가 바뀌어도 잡지 못했다.
# ============================================================
def make_cmd_env():
    """on_cmd() 를 실제로 통과시킬 수 있는 껍데기 노드 (reset/abort 경로 전용)."""
    node = make_env(state=State.GUIDE)
    node.patrol_idx = 3
    node.fire = (1.0, 2.0)
    node.gather_wp = (3.0, 4.0)
    node.gather_since = 1
    node._escaped_logged = True
    node.search_attempts = 2
    node.give_up = True
    node.last_seen = (5.0, 6.0)
    node.search_goal = (7.0, 8.0)
    node.refind_since = 1
    node.fault_retries = 1
    node.fault_since = None
    node.MAX_RETRIES = 2                     # 실노드는 __init__ 에서 세팅 (537줄)
    node.resume_state = State.GUIDE
    node.wp = {'normal_speed': 0.26, 'guide_speed': 0.12}
    node._sirens = []
    node.set_siren = lambda on: node._sirens.append(on)
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: 0)
    node._order = []
    real_cancel = node.speed.cancel_pending
    real_restore = node.speed.request_restore

    def rec_cancel(reason=''):
        node._order.append('cancel')
        return real_cancel(reason)

    def rec_restore(v):
        node._order.append('restore')
        return real_restore(v)

    node.speed.cancel_pending = rec_cancel
    node.speed.request_restore = rec_restore
    return node


def test_n6_on_cmd_reset_cancels_before_restore():
    """reset — cancel_pending 이 실제로 불리고, restore '앞'에 와야 한다.

    순서가 뒤집히면 restore 요청 자체가 stale 이 되어 늦은 응답 취급을 받는다."""
    node = make_cmd_env()
    node._guide_pending = True
    node.speed.request_guide(0.12)           # 진행 중 요청 (요청 0)
    node.on_cmd(types.SimpleNamespace(data='reset'))
    assert node._order == ['cancel', 'restore']   # ★ wiring + 순서
    assert node.state == State.PATROL
    assert not node._guide_pending
    assert node._calls == [0.12, 0.26]
    respond(node, 0, ok_resp())              # 늦은 guide 성공 = stale
    assert node.state == State.PATROL        # 상태를 덮지 못함


def test_n6_on_cmd_abort_cancels_and_faults_without_restore():
    """abort — cancel_pending 은 불리되 restore 는 하지 않는다 (정지 유지)."""
    node = make_cmd_env()
    node._guide_pending = True
    node.speed.request_guide(0.12)
    node.on_cmd(types.SimpleNamespace(data='abort'))
    assert node._order == ['cancel']          # ★ restore 없음
    assert node.state == State.FAULT
    assert not node._guide_pending
    assert node._calls == [0.12]
    respond(node, 0, ok_resp())               # 늦은 guide 성공
    assert node.state == State.FAULT          # FAULT 유지


def test_n7_settle_request_is_also_deadline_guarded():
    """N7 — 자기 논증 반증(AGENTS.md §3-4)에서 발견한 구멍.

    _settle 이 보낸 복구 요청은 _new_request 를 거치지 않는다. deadline 무장을
    _new_request 에만 두면 이 요청이 무응답일 때 _inflight=True 로 굳어
    _settle 이 영원히 막힌다 — 즉 P1 을 고치면서 같은 모양의 정지를 새로
    만든다. 발사 경로와 무관하게 감시되는지 고정한다."""
    node = make_env(timeout=30.0)
    arm_stale_guide_over_restore(node)
    seen = drain_failures(node, 2)               # restore 3회 실패로 종결
    node.speed.tick()                            # 복구 발사 (응답을 주지 않는다)
    assert len(node._calls) == 6
    assert node.speed._inflight is True

    node._clock['t'] = 30.0
    node.speed.tick()                            # ★ deadline 이 이 요청도 잡아야
    assert any('무응답 포기' in m for m in node._logs)
    assert node.speed._inflight is False         # 영구 정지 아님

    for _ in range(500):                         # 이후에도 유한 종결
        node._clock['t'] += 1.0
        node.speed.tick()
        seen = drain_failures(node, seen)
    assert any('재조정 포기' in m for m in node._logs)
    before = len(node._calls)
    for _ in range(100):
        node._clock['t'] += 1.0
        node.speed.tick()
    assert len(node._calls) == before


# ============================================================
# ★ 0720 Codex 재검토 보완 — N8~N12 (0720_현황.md §22)
#   신규 P1: GUIDE 진입 '후' stale 이 덮어쓴 뒤의 내부 reconcile 종결 누락.
#   뿌리 = purpose 겸직 — 상위 정책 출처(origin)와 in-flight 요청 종류가 한 값.
# ============================================================
def arm_guide_then_stale_sync_overwrites(node):
    """N8~N11 공통 전제: GUIDE 진입 확인 후 stale sync 0.26 이 controller 를 덮음.

    ① sync 0.26 요청 → ② guide 0.12 요청·성공 = GUIDE 진입(applied 0.12)
    ③ 늦은 sync 0.26 성공 도착 = controller 가 0.26 으로 덮임 → 내부 reconcile(0.12) 발사
    이 시점에서 로봇은 '사람을 앞에서 유도하는 중'인데 속도가 평시값이다.
    """
    node.speed.ensure_sync(0.26)             # 요청 0
    node._guide_pending = True
    node.speed.request_guide(0.12)           # 요청 1
    respond(node, 1, ok_resp())              # GUIDE 진입, applied=0.12
    assert node.state == State.GUIDE
    respond(node, 0, ok_resp())              # ★ 늦은 sync 성공 → applied=0.26
    assert node._calls == [0.26, 0.12, 0.12]  # reconcile(0.12) 발사됨


def test_n8_guide_origin_reconcile_failure_stops_guiding():
    """N8 — GUIDE 중 저속 복구가 끝내 실패하면 유도를 계속하면 안 된다.

    확정 정책(07-20 사용자): 제한 재시도 후 goal 취소 + FAULT.
    이전엔 _settle 이 'origin 이 guide' 라는 이유로 빠져 0.26 인 채 유도가 계속됐다."""
    node = make_env()
    arm_guide_then_stale_sync_overwrites(node)
    seen = drain_failures(node, 2)           # reconcile 3회 실패
    for _ in range(500):                     # 예산 안에서 재시도 → 소진 → 종결
        node.speed.tick()
        seen = drain_failures(node, seen)
    assert node._faults == [1]               # ★ FAULT 로 종결
    assert node._cancels == [1]              # goal 취소까지
    assert len(node._calls) < 30             # 유한 종결


def test_n9_guide_origin_reconcile_no_response_stops_guiding():
    """N9 — 같은 전제에서 reconcile 이 무응답이어도 동일한 안전 종결.
    (deadline 장치가 in-flight 요청을 실제로 감시하는지)"""
    node = make_env(timeout=30.0)
    arm_guide_then_stale_sync_overwrites(node)   # reconcile 발사 — 응답 주지 않음
    for i in range(500):
        node._clock['t'] += 1.0
        node.speed.tick()
    assert node._faults == [1]
    assert node._cancels == [1]


def test_n10_deadline_classifies_by_inflight_purpose_not_origin():
    """N10 — deadline 만료 시 '실제 만료한 요청의 종류'로 분류해야 한다.

    이전엔 _desired 의 origin(guide)을 읽어, 만료한 게 reconcile 인데도
    guide 진입 실패 콜백을 불렀다. 그 콜백은 _guide_pending=False 라 통보를
    무시해 아무 일도 일어나지 않았다 (조용한 실종)."""
    node = make_env(timeout=30.0)
    arm_guide_then_stale_sync_overwrites(node)
    node._clock['t'] = 30.0
    node.speed.tick()                        # reconcile deadline 만료
    # 진입 게이트 실패로 오분류되면 '늦은 guide 속도 실패 통보 ... 무시' 가 찍힌다
    assert not any('통보' in m and '무시' in m for m in node._logs), \
        'in-flight purpose 가 아니라 origin 으로 분류됨 (오분류)'
    assert any('reconcile' in m for m in node._logs)


def test_n11_forbidden_state_never_survives_any_ordering():
    """N11 — 금지 상태: GUIDE 인데 평시속도가 적용된 채 재시도도 FAULT 도 없음.
    응답 순서를 바꿔가며 어떤 조합에서도 남지 않아야 한다 (부정 회귀의 핵심)."""
    for fail_at in range(3):                 # reconcile 이 몇 번째에 무너지든
        node = make_env()
        arm_guide_then_stale_sync_overwrites(node)
        seen = 2
        for i in range(fail_at):             # 일부는 성공 응답으로 흔들어 본다
            respond(node, seen, fail_resp())
            seen += 1
        seen = drain_failures(node, seen)
        for _ in range(500):
            node.speed.tick()
            seen = drain_failures(node, seen)
        forbidden = (node.state == State.GUIDE
                     and node.speed._applied != 0.12
                     and node._faults == [])
        assert not forbidden, f'금지 상태 잔존 (fail_at={fail_at})'


def test_n12_unready_restore_contract_no_budget_then_resumes():
    """N12 — 계약 확정(§10.4 P2): 서비스가 처음부터 없는 restore 는
    '재시도 자체가 불가능한 상태'라 예산을 소모하지 않는다. 대신 복귀하면
    즉시 재개한다 — 문서 문장과 동작을 같은 말로 고정한다."""
    node = make_env(ready=False)
    node.speed.request_restore(0.26)
    node._clock['t'] = 300.0
    for _ in range(100):
        node.speed.tick()
    assert node._calls == []                     # 못 쏨
    assert node.speed._settle_rounds == 0        # 예산 미소모 (재시도 불가 상태)
    assert not node.speed._settle_gave_up        # 포기로 굳지 않음
    node._ready['ready'] = True
    node.speed.tick()
    assert node._calls == [0.26]                 # ★ 복귀 즉시 재개


class _Stamp:
    """tick() 의 FAULT 재시도 경과시간 계산용 최소 가짜 시각."""

    def __init__(self, ns):
        self.ns = ns

    def __sub__(self, other):
        return types.SimpleNamespace(nanoseconds=self.ns - other.ns)


def test_n13_guide_speed_regated_on_fault_resume():
    """N13 — FAULT 자동 재시도로 GUIDE 에 복귀할 때 저속 보장은 함께 돌아오지 않는다.

    ★ 07-20 재검토 §11.3: 이 테스트의 초판은 `state==GUIDE and _calls==[0.12]`
    만 확인해 **버그를 정상으로 고정**했다. 요청을 '보낸 것'을 적용 '확인된 것'과
    동일시한 것이다 — 다음 tick 에 escape goal 이 나가 평시속도로 유도가 시작됐다.
    완료판정은 "확인 전에는 GUIDE 주행 goal 이 단 한 건도 전송되지 않는다".

    §22 의 유지-실패 종결은 FAULT 를 만들지만, MAX_RETRIES 안이면 tick 이
    GUIDE 로 자동 복귀시킨다. 이때 controller 는 아직 0.26 이고 Manager 의
    복구 예산(_settle_gave_up)은 소진돼 있어, 아무도 다시 고치지 않으면
    '금지 상태'(GUIDE + 평시속도 + 재시도 0)가 부활한다."""
    node = make_env(state=State.FAULT)
    node.wp = {'normal_speed': 0.26, 'guide_speed': 0.12,
               'escape': {'x': -12.0, 'y': 0.0, 'yaw': 0.0}}
    node.siren_on = False
    node.state_pub = types.SimpleNamespace(publish=lambda m: None)
    node.siren_pub = types.SimpleNamespace(publish=lambda m: None)
    node.MAX_RETRIES = 2
    node.RETRY_WAIT = 3.0
    node.fault_retries = 0
    node.resume_state = State.GUIDE
    node.fault_since = _Stamp(0)
    node.get_clock = lambda: types.SimpleNamespace(now=lambda: _Stamp(100 * 10**9))
    node.speed.synced = True                 # sync 가 대신 쏘는 경로 배제

    node._goals = []
    node.goal_active = False
    node.give_up = True
    node.monitor = types.SimpleNamespace(
        visible=lambda zone=None: True, lost=lambda zone=None: False,
        scan_stale=lambda: False, reset=lambda k: None)
    node.send_goal = lambda wp, tag='': (
        node._goals.append((tag, node.speed._applied)),
        setattr(node, 'goal_active', True))[0]
    node.speed._applied = 0.26                # 유지 실패 종결 직후 = 평시속도 적용됨

    node.tick()
    assert node.state == State.GUIDE          # 복귀했고
    assert node._calls == [0.12]              # 저속을 다시 '요청'했고
    assert not node.speed.guide_speed_applied     # ★ 아직 '확인'은 안 됐다
    assert node._goals == []                  # ★ 그러므로 주행 goal 금지

    node.tick()                               # 응답 없는 채 다음 tick
    assert node._goals == [], '저속 미확인 상태로 유도 주행이 시작됨 (F2 위반)'

    respond(node, 0, ok_resp())               # 이제 적용 확인
    assert node.speed.guide_speed_applied
    node.tick()
    assert node._goals == [('escape', 0.12)]  # ★ 확인 후에야 출발


# ============================================================
# §12 P1 봉합(A): 게이트 술어를 latch → live 로 — MissionNode.tick() 진입
# ============================================================
def _tick_harness(node):
    """N14~N15: MissionNode.tick() 로 GUIDE 주행 게이트를 실제로 통과시키는 하네스.
    make_env 는 speed 만 계측하므로 tick() 이 참조하는 노드 필드를 채운다 (N13 요령).
    give_up=True 로 SEARCH_BACK 추종 블록을 배제하고, send_goal 은 (tag, 적용속도)를
    _goals 에 남긴다 — '어떤 속도로 유도를 시작했나'를 검사에서 직접 본다."""
    node.wp = {'normal_speed': 0.26, 'guide_speed': 0.12,
               'escape': {'x': -12.0, 'y': 0.0, 'yaw': 0.0}}
    node.siren_on = False
    node.state_pub = types.SimpleNamespace(publish=lambda m: None)
    node.siren_pub = types.SimpleNamespace(publish=lambda m: None)
    node.speed.synced = True                 # sync 가 대신 쏘는 경로 배제
    node._goals = []
    node.goal_active = False
    node.give_up = True
    node.monitor = types.SimpleNamespace(
        visible=lambda zone=None: True, lost=lambda zone=None: False,
        scan_stale=lambda: False, reset=lambda k: None)
    node.send_goal = lambda wp, tag='': (
        node._goals.append((tag, node.speed._applied)),
        setattr(node, 'goal_active', True))[0]


def test_n14_stale_sync_overwrite_blocks_new_guide_goal():
    """N14 — 재검토 §12 P1: guide 확인 뒤 늦은 sync 0.26 이 controller 를 덮으면,
    적용값이 평시속도(0.26)인데도 새 escape goal 이 나가던 구멍.

    A(guide_speed_applied = 지금 적용값이 저속인가, live)로 봉합. 이전 게이트 술어인
    latch(_guide_was_confirmed)는 '과거 1회 성공'을 뜻해 이 창을 못 막았다 —
    소비 지점은 옳았으나 술어가 stale 였다."""
    node = make_env()
    arm_guide_then_stale_sync_overwrites(node)   # applied=0.26, reconcile(0.12) in-flight
    _tick_harness(node)
    assert node.state == State.GUIDE
    assert node.speed._applied == 0.26
    assert not node.speed.guide_speed_applied     # ★ 지금 평시속도 = 저속 미적용

    node.tick()
    assert node._goals == [], 'stale 평시속도 적용 상태에서 유도 주행이 시작됨 (F2 위반)'
    node.tick()                                   # 응답 없는 다음 tick 도 금지
    assert node._goals == []

    respond(node, 2, ok_resp())                   # reconcile(0.12) 성공 → applied=0.12
    assert node.speed._applied == 0.12
    assert node.speed.guide_speed_applied
    node.tick()
    assert node._goals == [('escape', 0.12)]      # ★ 적용 확인 후에야 출발


def test_n15_stale_sync_does_not_disturb_moving_guide_goal():
    """N15 — 회귀 가드(Codex §12.3 철회분): A 는 '신규 전송'만 막고, 이미 주행
    중인 escape goal 을 즉시 cancel 하지 않는다 (§22.3 유지 — 일시적 표류는
    reconcile 이 먼저, 예산 소진 시에만 cancel+FAULT: N8/N9). reconcile 성공
    뒤에도 기존 goal 을 유지하고 중복 goal 을 보내지 않는다.

    '즉시 정지'는 §22.3 보다 엄격한 새 정책이고, 그 취소의 CANCELED 종결 직렬화는
    GoalManager(2/3, B) 소관이다 — 이 묶음에 넣지 않는다."""
    node = make_env()
    arm_guide_then_stale_sync_overwrites(node)
    _tick_harness(node)
    node._goals = [('escape', 0.12)]              # 이미 escape 주행 중
    node.goal_active = True

    node.tick()                                   # stale 0.26 적용 상태
    assert node._cancels == [], '일시적 표류에 escape goal 을 즉시 취소함 (§22.3 위반)'
    assert node._goals == [('escape', 0.12)]      # 중복 전송도 없음

    respond(node, 2, ok_resp())                   # reconcile 성공 → applied=0.12
    node.tick()
    assert node._cancels == []                    # 여전히 취소 없음
    assert node._goals == [('escape', 0.12)]      # 기존 goal 유지, 중복 없음
