# -*- coding: utf-8 -*-
"""
test_goal_lifecycle.py — Nav2 goal 전송/취소 레이스 단위테스트 (07-19 P0 수정의 회귀 방어선)
============================================================
[무엇을 잡나]
  send_goal 응답이 오기 전에 cancel_current_goal()(알람·abort·역행)이 먼저 실행되는
  레이스: 취소 시점엔 goal 핸들이 아직 없어 취소할 게 없고, 그 goal 이 뒤늦게
  수락되면 Nav2 혼자 주행 계속 — "abort 했는데 안 멈춤". 창(window)이 수 ms 라
  E2E 로는 재현이 거의 불가능 → 콜백에 stale 응답을 직접 주입해 단위테스트로 검증.

[테스트 기법 — MissionNode.__new__]
  MissionNode.__init__ 은 rclpy·yaml·액션클라이언트가 다 필요해 무겁다.
  콜백 로직만 검증하면 되므로 __new__ 로 '빈 껍데기' 인스턴스를 만들고
  콜백이 만지는 속성만 손으로 채운다 (get_logger 는 가짜로 덮어씀 —
  파이썬은 인스턴스 속성이 클래스 메서드보다 먼저 조회됨).
"""

import types

from mission_manager.mission_node import MissionNode, State


# ============================================================
# 가짜 부품
# ============================================================
class FakeGoalHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.cancel_called = False
        self.result_cb = None
        self.cancel_response_cb = None   # ★ 07-19 Codex: 취소 응답 확인 콜백

    def cancel_goal_async(self):
        self.cancel_called = True
        fut = types.SimpleNamespace()
        fut.add_done_callback = lambda cb: setattr(self, 'cancel_response_cb', cb)
        return fut

    def get_result_async(self):
        fut = types.SimpleNamespace()
        fut.add_done_callback = lambda cb: setattr(self, 'result_cb', cb)
        return fut


def fake_future(handle):
    """send_goal_async 가 돌려주는 future 흉내 — result() 가 goal 핸들."""
    return types.SimpleNamespace(result=lambda: handle)


def fake_cancel_response(n_canceling):
    """CancelGoal.Response 흉내 — goals_canceling 에 n개 잡힌 응답."""
    resp = types.SimpleNamespace(goals_canceling=[object()] * n_canceling)
    return types.SimpleNamespace(result=lambda: resp)


def bare_node():
    """__init__ 우회 — 콜백이 만지는 최소 속성만 채운 껍데기 노드."""
    node = MissionNode.__new__(MissionNode)
    node.goal_seq = 0
    node.goal_active = False
    node._goal_handle = None
    logs = []
    node.get_logger = lambda: types.SimpleNamespace(
        warn=lambda msg, **kw: logs.append(msg),
        info=lambda msg, **kw: logs.append(msg),
        error=lambda msg, **kw: logs.append(msg))
    node._logs = logs
    return node


# ============================================================
# ★ P0 레이스: 응답 전 취소 → 뒤늦은 수락은 즉시 취소돼야 한다
# ============================================================
def test_stale_accepted_goal_cancelled_immediately():
    """알람/abort 가 send_goal 응답보다 먼저 → 늦게 수락된 goal 은 즉시 cancel."""
    node = bare_node()
    node.goal_seq = 1                    # goal(seq=1) 전송, 응답 대기 중
    node.goal_active = True

    node.cancel_current_goal()           # 알람·abort 가 먼저 도착 (핸들 아직 없음)
    assert node.goal_seq == 2            # seq 증가로 stale 마킹
    assert not node.goal_active

    late = FakeGoalHandle(accepted=True)
    node.on_goal_response(1, fake_future(late))   # 그 뒤에야 수락 응답 도착

    assert late.cancel_called            # ★ 핵심: 무시가 아니라 즉시 취소
    assert node._goal_handle is None     # stale 핸들을 현재 핸들로 오인하지 않음


def test_stale_rejected_goal_needs_no_cancel():
    """뒤늦게 '거부' 응답이 온 stale goal 은 취소할 것도 없다 (죽지만 않으면 됨)."""
    node = bare_node()
    node.goal_seq = 2
    late = FakeGoalHandle(accepted=False)
    node.on_goal_response(1, fake_future(late))
    assert not late.cancel_called
    assert node._goal_handle is None


def test_current_goal_response_stored_not_cancelled():
    """정상 경로: seq 일치하는 수락 응답은 취소 없이 핸들 보관 + 결과 콜백 등록."""
    node = bare_node()
    node.goal_seq = 3
    handle = FakeGoalHandle(accepted=True)
    node.on_goal_response(3, fake_future(handle))
    assert not handle.cancel_called
    assert node._goal_handle is handle
    assert handle.result_cb is not None  # get_result_async 콜백 등록됨


def test_cancel_after_acceptance_cancels_handle():
    """수락 완료 후의 취소는 기존 경로대로 핸들을 직접 cancel."""
    node = bare_node()
    node.goal_seq = 3
    handle = FakeGoalHandle(accepted=True)
    node.on_goal_response(3, fake_future(handle))

    node.cancel_current_goal()
    assert handle.cancel_called
    assert node._goal_handle is None
    assert node.goal_seq == 4


def test_result_clears_handle():
    """goal 완료(on_result) 후엔 핸들이 비워져야 — 낡은 핸들 취소 헛손질 방지."""
    from action_msgs.msg import GoalStatus
    node = bare_node()
    node.goal_seq = 5
    node.fault_retries = 1
    node.on_reached = lambda: None       # 상태 전이는 이 테스트 관심사 아님
    handle = FakeGoalHandle(accepted=True)
    node.on_goal_response(5, fake_future(handle))

    result = types.SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)
    node.on_result(5, types.SimpleNamespace(result=lambda: result))
    assert node._goal_handle is None
    assert not node.goal_active
    assert node.fault_retries == 0       # 성공 시 재시도 카운터 리셋 (기존 동작 유지)


def test_stale_result_ignored():
    """취소된(=seq 지난) goal 의 결과 통보는 상태를 건드리면 안 됨."""
    from action_msgs.msg import GoalStatus
    node = bare_node()
    node.goal_seq = 6
    node.goal_active = True              # 새 goal 이 진행 중이라 가정
    reached = []
    node.on_reached = lambda: reached.append(1)

    result = types.SimpleNamespace(status=GoalStatus.STATUS_CANCELED)
    node.on_result(5, types.SimpleNamespace(result=lambda: result))
    assert node.goal_active              # 현재 goal 의 진행 상태 불변
    assert not reached


# ============================================================
# ★ 07-19 Codex §3.2: 취소는 '요청'이 아니라 '접수·종결 확인'까지
# ============================================================
def test_cancel_response_confirmed_when_accepted():
    """취소 응답에 goals_canceling 이 잡히면 조용히 확인 로그만 (에러 없음)."""
    node = bare_node()
    node.goal_seq = 3
    handle = FakeGoalHandle(accepted=True)
    node.on_goal_response(3, fake_future(handle))
    node.cancel_current_goal()
    assert handle.cancel_response_cb is not None   # 응답 확인 콜백이 등록됐다
    handle.cancel_response_cb(fake_cancel_response(1))
    assert any('취소 접수 확인' in m for m in node._logs)


def test_cancel_response_empty_reports_warning():
    """취소 응답이 비었으면(거절/이미 종결) 침묵 금지 — 경고가 남아야 한다."""
    node = bare_node()
    node.goal_seq = 3
    handle = FakeGoalHandle(accepted=True)
    node.on_goal_response(3, fake_future(handle))
    node.cancel_current_goal()
    handle.cancel_response_cb(fake_cancel_response(0))
    assert any('취소 접수 안 됨' in m for m in node._logs)


def test_stale_goal_watches_final_result():
    """stale 취소 경로: 핸들을 버리기 전에 최종 결과 콜백이 등록돼야 한다
    (취소 거절 시 'CANCELED 아닌 종결'을 보고할 유일한 통로)."""
    node = bare_node()
    node.goal_seq = 2
    late = FakeGoalHandle(accepted=True)
    node.on_goal_response(1, fake_future(late))    # stale 수락 → 즉시 취소
    assert late.cancel_called
    assert late.result_cb is not None              # ★ 결과 감시 등록
    assert late.cancel_response_cb is not None     # ★ 취소 응답 감시 등록


def test_stale_goal_result_canceled_is_quiet():
    """stale goal 이 정상대로 CANCELED 종결 — 확인 로그만, 에러 없음."""
    from action_msgs.msg import GoalStatus
    node = bare_node()
    node.goal_seq = 2
    late = FakeGoalHandle(accepted=True)
    node.on_goal_response(1, fake_future(late))
    result = types.SimpleNamespace(status=GoalStatus.STATUS_CANCELED)
    late.result_cb(types.SimpleNamespace(result=lambda: result))
    assert any('CANCELED 종결 확인' in m for m in node._logs)
    assert not any('취소되지 않고' in m for m in node._logs)


def test_stale_goal_result_not_canceled_reports_error():
    """★ 핵심: stale goal 이 취소 안 되고 SUCCEEDED 로 끝남 = 로봇이 죽은 목표로
    주행했다는 뜻 — 에러로 보고돼야 관제가 알 수 있다."""
    from action_msgs.msg import GoalStatus
    node = bare_node()
    node.goal_seq = 2
    late = FakeGoalHandle(accepted=True)
    node.on_goal_response(1, fake_future(late))
    result = types.SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)
    late.result_cb(types.SimpleNamespace(result=lambda: result))
    assert any('취소되지 않고' in m for m in node._logs)


# ============================================================
# ★ S1-4 (07-19 Codex §10.6): Action 콜백 예외 방어
#   future.result()/cancel 호출이 던지는 예외가 콜백 밖으로 새면 executor
#   로그에만 남고 미션은 goal_active=True 로 영구 대기 — FAULT 로 정리돼야.
# ============================================================
def failing_future(exc=RuntimeError('rmw down')):
    def boom():
        raise exc
    return types.SimpleNamespace(result=boom)


def test_goal_response_exception_enters_fault():
    """응답 future 가 예외 — 전파 금지 + 상태 정리 + FAULT."""
    node = bare_node()
    node.goal_seq = 1
    node.goal_active = True
    faults = []
    node.enter_fault = lambda: faults.append(1)
    node.on_goal_response(1, failing_future())   # 예외가 새면 여기서 터짐
    assert faults == [1]
    assert not node.goal_active
    assert node._goal_handle is None


def test_stale_goal_response_exception_quiet():
    """지난 seq 의 응답 예외 — 현재 미션 상태를 건드리면 안 됨 (FAULT 금지)."""
    node = bare_node()
    node.goal_seq = 2
    node.goal_active = True                      # 새 goal 진행 중 가정
    faults = []
    node.enter_fault = lambda: faults.append(1)
    node.on_goal_response(1, failing_future())
    assert not faults
    assert node.goal_active                      # 현재 goal 불변


def test_result_exception_enters_fault():
    """결과 future 예외 — 핸들 정리 + FAULT (영구 대기 방지)."""
    node = bare_node()
    node.goal_seq = 3
    handle = FakeGoalHandle(accepted=True)
    node.on_goal_response(3, fake_future(handle))
    faults = []
    node.enter_fault = lambda: faults.append(1)
    node.on_result(3, failing_future())
    assert faults == [1]
    assert node._goal_handle is None
    assert not node.goal_active


def test_cancel_call_exception_guarded():
    """cancel_goal_async 호출 자체가 예외 — 전파 금지 + '정지 미보장' 에러 로그."""
    class ExplodingCancelHandle(FakeGoalHandle):
        def cancel_goal_async(self):
            raise RuntimeError('server gone')

    node = bare_node()
    node.goal_seq = 3
    handle = ExplodingCancelHandle(accepted=True)
    node.on_goal_response(3, fake_future(handle))
    node.cancel_current_goal()                   # 예외가 새면 여기서 터짐
    assert node._goal_handle is None
    assert any('취소 요청 실패' in m for m in node._logs)


# ============================================================
# ★ S1-5 (07-19 Codex §9.4): 속도 변경은 '요청'이 아니라 '확인'까지
# ============================================================
def speed_node():
    node = bare_node()
    calls = []
    node.param_cli = types.SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda req: calls.append(req) or types.SimpleNamespace(
            add_done_callback=lambda cb: setattr(node, '_speed_cb', cb)))
    node._speed_calls = calls
    return node


def fake_speed_response(successful, reason=''):
    res = types.SimpleNamespace(results=[
        types.SimpleNamespace(successful=successful, reason=reason)])
    return types.SimpleNamespace(result=lambda: res)


def test_speed_change_confirmed_on_success():
    node = speed_node()
    node.set_nav_speed(0.12)
    assert len(node._speed_calls) == 1
    node._speed_cb(fake_speed_response(True))
    assert any('변경 확인' in m for m in node._logs)


def test_speed_change_retries_then_errors():
    """실패 응답 → 재시도 2회 → 3회째도 실패면 에러 보고 (조용한 실패 금지)."""
    node = speed_node()
    node.set_nav_speed(0.12)
    node._speed_cb(fake_speed_response(False, 'rejected'))   # 1차 실패 → 재시도
    assert len(node._speed_calls) == 2
    node._speed_cb(fake_speed_response(False, 'rejected'))   # 2차 실패 → 재시도
    assert len(node._speed_calls) == 3
    node._speed_cb(fake_speed_response(False, 'rejected'))   # 3차 실패 → 포기+에러
    assert len(node._speed_calls) == 3                       # 더 안 쏨
    assert any('3회 실패' in m for m in node._logs)


# ============================================================
# ★ F2 (07-19 Codex §12.3): GUIDE 는 저속 '확인' 전 주행 금지
# ============================================================
def guide_gate_node(ready=True):
    """speed_node + GUIDE 게이트 검증용 속성 (GATHER 상태·FAULT 계측)."""
    node = speed_node()
    node.state = State.GATHER
    node._guide_pending = False
    node._speed_synced = False
    node._speed_sync_inflight = False
    node._speed_sync_cooldown = 0
    node.param_cli.service_is_ready = lambda: ready
    node._cancels = []
    node._faults = []
    node.cancel_current_goal = lambda: node._cancels.append(1)
    node.enter_fault = lambda: node._faults.append(1)
    return node


def test_guide_gate_service_not_ready_no_guide():
    """서비스 미준비 — 요청 자체가 생략되고 GUIDE 진입 없음 (GATHER 정지 유지)."""
    node = guide_gate_node(ready=False)
    node._guide_pending = True                    # tick 이 요청 직전 세운 플래그
    node.set_nav_speed(0.12, purpose='guide')
    assert node.state == State.GATHER             # 주행 안 함
    assert not node._guide_pending                # tick 재시도 가능하게 해제
    assert not node._speed_calls


def test_guide_gate_success_enters_guide():
    """저속 적용 성공 확인 → 그때서야 GATHER→GUIDE 전환."""
    node = guide_gate_node()
    node._guide_pending = True
    node.set_nav_speed(0.12, purpose='guide')
    assert node.state == State.GATHER             # 응답 전엔 전환 금지
    node._speed_cb(fake_speed_response(True))
    assert node.state == State.GUIDE
    assert not node._guide_pending


def test_guide_gate_three_failures_fault_not_guide():
    """3회 실패 — GUIDE 진입 대신 goal 취소+FAULT (평시 속도 유도 금지)."""
    node = guide_gate_node()
    node._guide_pending = True
    node.set_nav_speed(0.12, purpose='guide')
    for _ in range(3):
        node._speed_cb(fake_speed_response(False, 'rejected'))
    assert node.state == State.GATHER             # GUIDE 로 안 감
    assert node._faults == [1]
    assert node._cancels == [1]
    assert not node._guide_pending


def test_guide_gate_call_exception_fault_not_guide():
    """call_async 자체 예외 3연속 — 전파 금지 + FAULT (Codex §12.3 재현 봉쇄)."""
    node = guide_gate_node()

    def explode(req):
        raise RuntimeError('service gone')
    node.param_cli.call_async = explode
    node._guide_pending = True
    node.set_nav_speed(0.12, purpose='guide')     # 예외가 새면 여기서 터짐
    assert node.state == State.GATHER
    assert node._faults == [1]


def test_guide_gate_late_confirm_after_abort_no_transition():
    """확인 대기 중 abort 로 FAULT 전환 — 늦은 성공 응답이 GUIDE 로 덮으면 안 됨."""
    node = guide_gate_node()
    node._guide_pending = True
    node.set_nav_speed(0.12, purpose='guide')
    node.state = State.FAULT                      # abort 가 먼저 도착
    node._guide_pending = False
    node._speed_cb(fake_speed_response(True))     # 늦은 성공 응답
    assert node.state == State.FAULT              # 그대로


def test_sync_true_only_after_confirmed():
    """_speed_synced 는 성공 '응답'에서만 True — 요청 직후 True 금지 (Codex §12.3)."""
    node = guide_gate_node()
    node.set_nav_speed(0.26, purpose='sync')
    assert not node._speed_synced                 # 응답 전
    node._speed_cb(fake_speed_response(True))
    assert node._speed_synced
    assert not node._speed_sync_inflight


def test_sync_final_failure_sets_cooldown_not_synced():
    """sync 3회 실패 — synced 는 False 유지, cooldown 후 재요청 가능."""
    node = guide_gate_node()
    node._speed_sync_inflight = True              # tick 이 세웠다고 가정
    node.set_nav_speed(0.26, purpose='sync')
    for _ in range(3):
        node._speed_cb(fake_speed_response(False, 'err'))
    assert not node._speed_synced
    assert not node._speed_sync_inflight          # 재요청 허용
    assert node._speed_sync_cooldown > 0


# ============================================================
# ★ G1 (07-19 Codex §13.3): 실패 응답에도 stale 가드 — 성공만 가드하던 비대칭 종결
# ============================================================
def test_guide_gate_late_failure_after_reset_ignored():
    """Codex 축소 재현 봉쇄: guide 요청 중 reset(PATROL) → 늦은 '실패' 응답은
    재시도도 FAULT 도 일으키면 안 됨 (기존엔 fault_calls=1 실측)."""
    node = guide_gate_node()
    node._guide_pending = True
    node.set_nav_speed(0.12, purpose='guide')
    node.state = State.PATROL                     # 관제 reset 이 먼저 성공
    node._guide_pending = False
    node._speed_cb(fake_speed_response(False, 'rejected'))   # 이전 세대 늦은 실패
    assert len(node._speed_calls) == 1            # 재시도 안 함
    assert node._faults == []                     # 유령 FAULT 없음
    assert node._cancels == []
    assert node.state == State.PATROL             # reset 결과 유지
    assert any('늦은 guide' in m for m in node._logs)


def test_guide_gate_late_third_failure_after_reset_no_final_fault():
    """재시도 2회가 GATHER 중 이미 소모된 뒤 reset — 3회째 늦은 실패가
    _speed_final_fail(cancel+FAULT)로 떨어지면 안 됨."""
    node = guide_gate_node()
    node._guide_pending = True
    node.set_nav_speed(0.12, purpose='guide')
    node._speed_cb(fake_speed_response(False, 'rejected'))   # 1차 실패 (GATHER 중)
    node._speed_cb(fake_speed_response(False, 'rejected'))   # 2차 실패 (GATHER 중)
    assert len(node._speed_calls) == 3
    node.state = State.PATROL                     # reset 성공
    node._guide_pending = False
    node._speed_cb(fake_speed_response(False, 'rejected'))   # 3회째는 reset 후 도착
    assert node._faults == []                     # 최종 FAULT 강등 없음
    assert node._cancels == []
    assert node.state == State.PATROL


def test_guide_gate_failure_while_still_gather_retries():
    """★역회귀 앵커: 상태가 그대로(GATHER+pending)면 기존 재시도 동작 유지."""
    node = guide_gate_node()
    node._guide_pending = True
    node.set_nav_speed(0.12, purpose='guide')
    node._speed_cb(fake_speed_response(False, 'rejected'))
    assert len(node._speed_calls) == 2            # 정상 재시도는 그대로
