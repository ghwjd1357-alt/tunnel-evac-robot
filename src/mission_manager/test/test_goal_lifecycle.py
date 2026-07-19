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

from mission_manager.mission_node import MissionNode


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
