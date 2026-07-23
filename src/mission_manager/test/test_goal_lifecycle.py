# -*- coding: utf-8 -*-
"""
test_goal_lifecycle.py — Nav2 goal 비동기 수명주기 공격 테스트 (07-23 구조 분리 2/3)
============================================================
[무엇을 잡나 — 핸드오프 완료조건 3·4·5]
  send_goal 응답이 오기 전에 cancel 이 먼저 실행되는 레이스(늦게 수락된 stale
  goal 이 혼자 주행), 취소의 접수·종결 미확인, 콜백 예외로 인한 영구 대기,
  그리고 ★ B: 저속 상실로 유도를 정지시킬 때 그 취소가 CANCELED 로 종결되기
  전에는 신규 goal 을 절대 재전송하지 않는 '종결 직렬화'.

[이관 이력]
  구판(07-19)은 MissionNode 의 goal 메서드를 직접 찔렀다. 07-23 에 수명주기가
  GoalManager 로 이관되면서 15개 시나리오를 매니저 이음새로 전부 옮겼다 —
  동작 불변 앵커로 유지되고, 노드는 얇은 위임만 남았다.

[테스트 기법]
  진짜 GoalManager 를 가짜 ActionClient(응답 콜백을 손으로 주입)와 가짜 goal
  핸들에 붙여, 실서비스로는 재현 불가능한 ms 단위 레이스를 결정적으로 재현한다.
  '껍데기 MissionNode + 진짜 GoalManager' 조합 테스트로 노드 진입점(send_goal/
  cancel_current_goal/on_reached)이 실제로 매니저와 맞물리는지도 통과시킨다.
"""

import types

from action_msgs.msg import GoalStatus

from mission_manager.mission_node import MissionNode, State
from mission_manager.goal_manager import GoalManager

CANCELED = GoalStatus.STATUS_CANCELED
SUCCEEDED = GoalStatus.STATUS_SUCCEEDED
ABORTED = GoalStatus.STATUS_ABORTED


# ============================================================
# 가짜 부품
# ============================================================
class FakeGoalHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.cancel_called = False
        self.result_cb = None
        self.cancel_response_cb = None

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
    """send_goal_async future 흉내 — result() 가 goal 핸들."""
    return types.SimpleNamespace(result=lambda: handle)


def fake_result(status):
    """get_result_async future 흉내 — result().status 가 종결 상태."""
    return types.SimpleNamespace(
        result=lambda: types.SimpleNamespace(status=status))


def fake_cancel_response(n_canceling):
    """CancelGoal.Response 흉내 — goals_canceling 에 n개 잡힌 응답."""
    resp = types.SimpleNamespace(goals_canceling=[object()] * n_canceling)
    return types.SimpleNamespace(result=lambda: resp)


def failing_future(exc=RuntimeError('rmw down')):
    def boom():
        raise exc
    return types.SimpleNamespace(result=boom)


def _logger(logs):
    return types.SimpleNamespace(
        warn=lambda m, **k: logs.append(m),
        info=lambda m, **k: logs.append(m),
        error=lambda m, **k: logs.append(m))


def _mk_fut(sent, goal, cbs):
    """send_goal_async: 전송 goal 기록 + 응답 콜백을 cbs 에 순서대로 쌓는다."""
    sent.append(goal)
    fut = types.SimpleNamespace()
    fut.add_done_callback = lambda cb: cbs.append(cb)
    return fut


def make_gm(ready=True):
    """진짜 GoalManager + 가짜 ActionClient/콜백 + 계측 필드.

      env.gm       = GoalManager
      env.goals    = 전송된 goal 리스트 (send_goal 검증)
      env.resp_cbs = send_goal_async 응답 콜백 (순서대로 — 임의 순서 주입)
      env.reached / env.faults / env.actives = 노드 정책 콜백 기록
      env.logs     = 로그
      env.ready    = {'ready': bool} 서비스 준비 토글
    """
    logs, reached, faults, actives = [], [], [], []
    ready_box = {'ready': ready}
    goals, resp_cbs = [], []
    nav = types.SimpleNamespace(
        server_is_ready=lambda: ready_box['ready'],
        send_goal_async=lambda g: _mk_fut(goals, g, resp_cbs))
    gm = GoalManager(
        nav, _logger(logs),
        on_reached=lambda: reached.append(1),
        on_fault=lambda: faults.append(1),
        on_active=lambda v: actives.append(v))
    return types.SimpleNamespace(
        gm=gm, goals=goals, resp_cbs=resp_cbs, reached=reached,
        faults=faults, actives=actives, logs=logs, ready=ready_box)


def send_and_accept(env, wp=None, tag='patrol'):
    """send_goal → 수락 응답 주입 → 핸들 보관까지. 반환 = 수락된 핸들."""
    env.gm.send_goal(wp or {'x': 1.0, 'y': 2.0}, tag=tag, state_name='PATROL')
    handle = FakeGoalHandle(accepted=True)
    env.resp_cbs[-1](fake_future(handle))
    return handle


# ============================================================
# ★ P0 레이스: 응답 전 취소 → 뒤늦은 수락은 즉시 취소돼야 한다 (이관 #1·#2)
# ============================================================
def test_stale_accepted_goal_cancelled_immediately():
    """알람/abort 가 send_goal 응답보다 먼저 → 늦게 수락된 goal 은 즉시 cancel."""
    env = make_gm()
    env.gm.send_goal({'x': 1, 'y': 2}, tag='patrol')   # seq1, 응답 대기
    assert env.gm.active
    env.gm.cancel_current_goal()                        # 핸들 없이 취소 (seq2)
    assert env.gm._seq == 2
    assert not env.gm.active

    late = FakeGoalHandle(accepted=True)
    env.resp_cbs[0](fake_future(late))                  # 그 뒤 수락 응답 도착
    assert late.cancel_called                           # ★ 무시가 아니라 즉시 취소
    assert env.gm._handle is None


def test_stale_rejected_goal_needs_no_cancel():
    """뒤늦게 '거부' 응답이 온 stale goal 은 취소할 것도 없다."""
    env = make_gm()
    env.gm.send_goal({'x': 1, 'y': 2})
    env.gm.cancel_current_goal()
    late = FakeGoalHandle(accepted=False)
    env.resp_cbs[0](fake_future(late))
    assert not late.cancel_called
    assert env.gm._handle is None


def test_current_goal_response_stored_not_cancelled():
    """정상: seq 일치 수락 응답은 취소 없이 핸들 보관 + 결과 콜백 등록."""
    env = make_gm()
    env.gm.send_goal({'x': 1, 'y': 2})
    handle = FakeGoalHandle(accepted=True)
    env.resp_cbs[0](fake_future(handle))
    assert not handle.cancel_called
    assert env.gm._handle is handle
    assert handle.result_cb is not None


def test_cancel_after_acceptance_cancels_handle():
    """수락 완료 후 취소는 핸들을 직접 cancel + 세대 증가."""
    env = make_gm()
    handle = send_and_accept(env)
    env.gm.cancel_current_goal()
    assert handle.cancel_called
    assert env.gm._handle is None
    assert env.gm._seq == 2


def test_result_clears_handle_and_reaches():
    """goal 성공(on_result SUCCEEDED) → 핸들 비움 + active False + on_reached."""
    env = make_gm()
    handle = send_and_accept(env)
    handle.result_cb(fake_result(SUCCEEDED))
    assert env.gm._handle is None
    assert not env.gm.active
    assert env.reached == [1]


def test_stale_result_ignored():
    """취소된(=seq 지난) goal 의 결과 통보는 현재 goal 을 건드리면 안 됨."""
    env = make_gm()
    env.gm._seq = 6
    env.gm._active = True                # 새 goal 진행 중 가정
    env.gm._on_result(5, fake_result(CANCELED))
    assert env.gm.active                 # 현재 goal 불변
    assert env.reached == []


# ============================================================
# ★ 07-19 Codex §3.2: 취소는 '요청'이 아니라 '접수·종결 확인'까지 (이관 #7~#11)
# ============================================================
def test_cancel_response_confirmed_when_accepted():
    """취소 응답에 goals_canceling 이 잡히면 확인 로그만 (에러 없음)."""
    env = make_gm()
    handle = send_and_accept(env)
    env.gm.cancel_current_goal()
    assert handle.cancel_response_cb is not None
    handle.cancel_response_cb(fake_cancel_response(1))
    assert any('취소 접수 확인' in m for m in env.logs)


def test_cancel_response_empty_reports_warning():
    """취소 응답이 비었으면(거절/이미 종결) 침묵 금지 — 경고가 남아야."""
    env = make_gm()
    handle = send_and_accept(env)
    env.gm.cancel_current_goal()
    handle.cancel_response_cb(fake_cancel_response(0))
    assert any('취소 접수 안 됨' in m for m in env.logs)


def test_stale_goal_watches_final_result():
    """stale 취소 경로: 핸들 버리기 전에 최종결과·취소응답 콜백이 등록돼야."""
    env = make_gm()
    env.gm.send_goal({'x': 1, 'y': 2})
    env.gm.cancel_current_goal()
    late = FakeGoalHandle(accepted=True)
    env.resp_cbs[0](fake_future(late))
    assert late.cancel_called
    assert late.result_cb is not None
    assert late.cancel_response_cb is not None


def test_stale_goal_result_canceled_is_quiet():
    """stale goal 이 정상대로 CANCELED 종결 — 확인 로그만, 에러 없음."""
    env = make_gm()
    env.gm.send_goal({'x': 1, 'y': 2})
    env.gm.cancel_current_goal()
    late = FakeGoalHandle(accepted=True)
    env.resp_cbs[0](fake_future(late))
    late.result_cb(fake_result(CANCELED))
    assert any('CANCELED 종결 확인' in m for m in env.logs)
    assert not any('취소되지 않고' in m for m in env.logs)


def test_stale_goal_result_not_canceled_reports_error():
    """★ stale goal 이 취소 안 되고 SUCCEEDED 로 끝남 = 죽은 목표로 주행한 것 —
    에러로 보고돼야 관제가 안다."""
    env = make_gm()
    env.gm.send_goal({'x': 1, 'y': 2})
    env.gm.cancel_current_goal()
    late = FakeGoalHandle(accepted=True)
    env.resp_cbs[0](fake_future(late))
    late.result_cb(fake_result(SUCCEEDED))
    assert any('취소되지 않고' in m for m in env.logs)


# ============================================================
# ★ S1-4: Action 콜백 예외 방어 (이관 #12~#15)
# ============================================================
def test_goal_response_exception_enters_fault():
    """응답 future 예외 — 전파 금지 + 상태 정리 + FAULT."""
    env = make_gm()
    env.gm.send_goal({'x': 1, 'y': 2})           # seq1, active True
    env.resp_cbs[0](failing_future())
    assert env.faults == [1]
    assert not env.gm.active
    assert env.gm._handle is None


def test_stale_goal_response_exception_quiet():
    """지난 seq 의 응답 예외 — 현재 미션 상태를 건드리면 안 됨 (FAULT 금지)."""
    env = make_gm()
    env.gm._seq = 2
    env.gm._active = True                         # 새 goal 진행 중 가정
    env.gm._on_goal_response(1, failing_future())
    assert env.faults == []
    assert env.gm.active


def test_result_exception_enters_fault():
    """결과 future 예외 — 핸들 정리 + FAULT (영구 대기 방지)."""
    env = make_gm()
    handle = send_and_accept(env)
    handle.result_cb(failing_future())
    assert env.faults == [1]
    assert env.gm._handle is None
    assert not env.gm.active


def test_cancel_call_exception_guarded():
    """cancel_goal_async 호출 자체 예외 — 전파 금지 + '정지 미보장' 에러."""
    class ExplodingCancelHandle(FakeGoalHandle):
        def cancel_goal_async(self):
            raise RuntimeError('server gone')

    env = make_gm()
    env.gm.send_goal({'x': 1, 'y': 2})
    handle = ExplodingCancelHandle(accepted=True)
    env.resp_cbs[0](fake_future(handle))
    env.gm.cancel_current_goal()
    assert env.gm._handle is None
    assert any('취소 요청 실패' in m for m in env.logs)


# ============================================================
# ★ 완료조건 4: handle 없는 취소는 조용히 통과 + goal_active 미러
# ============================================================
def test_cancel_without_handle_passes_quietly():
    """전송했지만 아직 미수락(핸들 없음) 상태의 취소 — 조용히 통과, active False."""
    env = make_gm()
    env.gm.send_goal({'x': 1, 'y': 2})           # active True, 핸들 없음
    env.gm.cancel_current_goal()
    assert not env.gm.active
    assert env.actives[-1] is False


def test_active_mirror_tracks_lifecycle():
    """on_active 콜백이 전송=True / 종결=False 로 goal_active 를 미러한다."""
    env = make_gm()
    handle = send_and_accept(env)
    assert env.gm.active and env.actives[-1] is True
    handle.result_cb(fake_result(SUCCEEDED))
    assert not env.gm.active and env.actives[-1] is False


def test_send_goal_when_server_not_ready_noop():
    """액션서버 미준비면 세대·active 를 건드리지 않고 조용히 생략 (블로킹 금지)."""
    env = make_gm(ready=False)
    env.gm.send_goal({'x': 1, 'y': 2})
    assert env.goals == []
    assert not env.gm.active
    assert env.gm._seq == 0
    assert any('액션서버 아직 없음' in m for m in env.logs)


# ============================================================
# ★ B: 유도정지 취소 '종결 직렬화' (핸드오프 완료조건 5 — 이 묶음 신규)
# ============================================================
def test_b_guide_stop_blocks_new_goal_until_canceled():
    """저속 상실 정지(intent='guide_stop') 후 CANCELED 종결 전에는 신규 goal
    전면 봉쇄 — 저속이 다시 확인돼도. CANCELED 종결 관찰 시 해제되고 재전송 허용."""
    env = make_gm()
    handle = send_and_accept(env, tag='escape')      # 유도 주행 중 (seq1)
    env.gm.cancel_current_goal(intent='guide_stop')  # 저속 상실 → 유도정지
    assert env.gm._stop_pending

    env.gm.send_goal({'x': 9, 'y': 9}, tag='escape')  # 저속 확인됐다 쳐도
    assert len(env.goals) == 1                        # ★ 신규 goal 안 나감
    assert any('종결 대기' in m for m in env.logs)

    handle.result_cb(fake_result(CANCELED))           # 취소 goal 이 CANCELED 종결
    assert not env.gm._stop_pending
    assert any('신규 goal 허용' in m for m in env.logs)

    env.gm.send_goal({'x': 9, 'y': 9}, tag='escape')  # 이제 허용
    assert len(env.goals) == 2


def test_b_guide_stop_empty_canceling_faults_and_blocks():
    """유도정지 취소가 접수조차 안 됨(빈 goals_canceling) → FAULT + 재전송 금지."""
    env = make_gm()
    handle = send_and_accept(env, tag='escape')
    env.gm.cancel_current_goal(intent='guide_stop')
    handle.cancel_response_cb(fake_cancel_response(0))
    assert env.faults == [1]
    assert env.gm._stop_pending                       # 여전히 봉쇄
    env.gm.send_goal({'x': 9, 'y': 9})
    assert len(env.goals) == 1


def test_b_guide_stop_cancel_call_exception_faults_and_blocks():
    """유도정지 취소 '호출' 자체가 예외 → FAULT + 재전송 금지."""
    class ExplodingCancelHandle(FakeGoalHandle):
        def cancel_goal_async(self):
            raise RuntimeError('server gone')

    env = make_gm()
    env.gm.send_goal({'x': 1, 'y': 2}, tag='escape')
    handle = ExplodingCancelHandle(accepted=True)
    env.resp_cbs[0](fake_future(handle))
    env.gm.cancel_current_goal(intent='guide_stop')
    assert env.faults == [1]
    assert env.gm._stop_pending


def test_b_guide_stop_cancel_response_exception_faults_and_blocks():
    """유도정지 취소 '응답' 수신 예외 → FAULT + 재전송 금지."""
    env = make_gm()
    handle = send_and_accept(env, tag='escape')
    env.gm.cancel_current_goal(intent='guide_stop')
    handle.cancel_response_cb(failing_future())
    assert env.faults == [1]
    assert env.gm._stop_pending


def test_b_guide_stop_non_canceled_terminal_faults_and_blocks():
    """유도정지 취소한 goal 이 CANCELED 아닌 status 로 종결 → FAULT + 재전송 금지."""
    env = make_gm()
    handle = send_and_accept(env, tag='escape')
    env.gm.cancel_current_goal(intent='guide_stop')
    handle.result_cb(fake_result(SUCCEEDED))          # 취소 안 되고 SUCCEEDED
    assert env.faults == [1]
    assert env.gm._stop_pending
    assert any('CANCELED 아닌' in m for m in env.logs)


def test_b_guide_stop_budget_exhaustion_faults():
    """CANCELED 종결이 영영 안 오면 대기 예산 소진 → FAULT (무기한 봉쇄 금지)."""
    env = make_gm()
    send_and_accept(env, tag='escape')
    env.gm.cancel_current_goal(intent='guide_stop')
    for _ in range(GoalManager.CANCEL_STOP_MAX_BLOCKS - 1):
        env.gm.send_goal({'x': 9, 'y': 9})
    assert env.faults == []                            # 아직 예산 안 소진
    env.gm.send_goal({'x': 9, 'y': 9})                 # 예산 소진 tick
    assert env.faults == [1]
    assert len(env.goals) == 1                         # 끝까지 신규 goal 0건


def test_b_hard_cancel_clears_stop_pending():
    """운영자 reset/abort(intent='hard')는 진행 중 직렬화를 강제 해제한다."""
    env = make_gm()
    send_and_accept(env, tag='escape')
    env.gm.cancel_current_goal(intent='guide_stop')
    assert env.gm._stop_pending
    env.gm.cancel_current_goal(intent='hard')
    assert not env.gm._stop_pending
    env.gm.send_goal({'x': 9, 'y': 9})
    assert len(env.goals) == 2                         # hard 후 신규 goal 허용


def test_b_incidental_cancel_preserves_stop_pending():
    """일반 취소(intent=None, 재발견 복귀 등)는 유도정지 직렬화를 건드리지 않는다."""
    env = make_gm()
    send_and_accept(env, tag='escape')
    env.gm.cancel_current_goal(intent='guide_stop')
    assert env.gm._stop_pending
    env.gm.cancel_current_goal(intent=None)
    assert env.gm._stop_pending                        # ★ 직렬화 유지
    env.gm.send_goal({'x': 9, 'y': 9})
    assert len(env.goals) == 1                         # 여전히 봉쇄


def test_b_guide_stop_no_request_does_not_arm():
    """전송조차 없어 멈출 대상이 정말 없으면 직렬화 미무장 — 조용히 통과.

    ★ 0723검토 P1: 구판은 '전송했지만 미수락'까지 '핸들 없음 = 멈출 대상 없음'으로
    묶어 조용히 통과시켰다(버그를 앵커로 고정). 이제 '요청조차 없는' 경우만 미무장이다.
    '전송 후 미수락' 무장은 아래 §P1 부정 회귀가 담당한다."""
    env = make_gm()
    env.gm.cancel_current_goal(intent='guide_stop')    # send_goal 자체가 없었다
    assert not env.gm._stop_pending
    assert env.faults == []


def test_b_canceled_terminal_before_empty_response_no_double_fault():
    """레이스: CANCELED 종결이 빈 취소응답보다 먼저 오면, 이미 해제됐으므로
    뒤늦은 빈 응답이 FAULT 를 유발하면 안 된다 (해제 뒤 조용)."""
    env = make_gm()
    handle = send_and_accept(env, tag='escape')
    env.gm.cancel_current_goal(intent='guide_stop')
    handle.result_cb(fake_result(CANCELED))            # 먼저 정상 종결 → 해제
    assert not env.gm._stop_pending
    handle.cancel_response_cb(fake_cancel_response(0))  # 뒤늦은 빈 응답
    assert env.faults == []                            # ★ 이중 FAULT 없음


# ============================================================
# ★ 완료조건 7: 새 방어 테스트가 진짜 가드를 검증하는지 (가드 무력화 → 적색)
# ============================================================
def test_b_gate_is_load_bearing():
    """_stop_pending 가드를 임시로 끄면 봉쇄가 풀려 신규 goal 이 나간다 —
    위 B 테스트들이 딴 이유가 아니라 이 가드를 보고 있음을 고정한다."""
    env = make_gm()
    send_and_accept(env, tag='escape')
    env.gm.cancel_current_goal(intent='guide_stop')
    env.gm.send_goal({'x': 9, 'y': 9})                 # 가드 살아 있으면 봉쇄
    assert len(env.goals) == 1
    env.gm._stop_pending = False                       # ★ 가드 임시 무력화
    env.gm.send_goal({'x': 9, 'y': 9})
    assert len(env.goals) == 2, '가드를 껐는데도 봉쇄되면 가드가 아닌 딴 걸 본 것'


# ============================================================
# ★ 0723검토 P1 보완 — 수락응답 대기 중 guide_stop (접수 전환 레이스)
#   `_handle is None` 을 "멈출 goal 없음"으로 축약해, 서버가 이미 수락해 주행
#   중인데 응답만 늦은 goal 을 놓쳤다. 부정 회귀로 이 창을 직접 고정한다.
# ============================================================
def test_p1_pending_guide_stop_blocks_until_canceled():
    """전송 후 수락 전 guide_stop → 무장. 늦은 수락은 즉시 취소·감시하고, CANCELED
    종결 전 신규 goal 0건, 종결 후 1건 (검토자 핵심 부정 회귀)."""
    env = make_gm()
    env.gm.send_goal({'x': 1, 'y': 2}, tag='escape')   # seq1, 응답 보류
    env.gm.cancel_current_goal(intent='guide_stop')    # 수락 전 guide_stop
    assert env.gm._stop_pending                        # ★ 이제 무장됨(구판은 안 됐음)

    env.gm.send_goal({'x': 9, 'y': 9}, tag='escape')   # CANCELED 전 봉쇄
    assert len(env.goals) == 1

    late = FakeGoalHandle(accepted=True)
    env.resp_cbs[0](fake_future(late))                 # 늦은 수락 → 즉시 취소 + 감시
    assert late.cancel_called
    assert late.result_cb is not None
    env.gm.send_goal({'x': 9, 'y': 9})                 # 아직 CANCELED 전 → 봉쇄 유지
    assert len(env.goals) == 1

    late.result_cb(fake_result(CANCELED))              # 옛 goal CANCELED 종결
    assert not env.gm._stop_pending
    env.gm.send_goal({'x': 9, 'y': 9})                 # 이제 허용
    assert len(env.goals) == 2


def test_p1_pending_guide_stop_non_canceled_terminal_faults():
    """전송 후 미수락 guide_stop → 늦은 수락 → 취소 안 되고 SUCCEEDED 종결 →
    FAULT + 봉쇄 유지."""
    env = make_gm()
    env.gm.send_goal({'x': 1, 'y': 2}, tag='escape')
    env.gm.cancel_current_goal(intent='guide_stop')
    late = FakeGoalHandle(accepted=True)
    env.resp_cbs[0](fake_future(late))
    late.result_cb(fake_result(SUCCEEDED))
    assert env.faults == [1]
    assert env.gm._stop_pending
    env.gm.send_goal({'x': 9, 'y': 9})
    assert len(env.goals) == 1


def test_p1_pending_guide_stop_empty_cancel_faults():
    """늦은 수락의 취소가 빈 goals_canceling 으로 접수 실패 → FAULT + 봉쇄."""
    env = make_gm()
    env.gm.send_goal({'x': 1, 'y': 2}, tag='escape')
    env.gm.cancel_current_goal(intent='guide_stop')
    late = FakeGoalHandle(accepted=True)
    env.resp_cbs[0](fake_future(late))
    late.cancel_response_cb(fake_cancel_response(0))
    assert env.faults == [1]
    assert env.gm._stop_pending


def test_p1_pending_guide_stop_cancel_call_exception_faults():
    """늦은 수락의 취소 '호출' 자체가 예외 → FAULT + 봉쇄."""
    class ExplodingCancelHandle(FakeGoalHandle):
        def cancel_goal_async(self):
            raise RuntimeError('server gone')

    env = make_gm()
    env.gm.send_goal({'x': 1, 'y': 2}, tag='escape')
    env.gm.cancel_current_goal(intent='guide_stop')
    late = ExplodingCancelHandle(accepted=True)
    env.resp_cbs[0](fake_future(late))                 # stale 즉시 취소 시도 → 예외
    assert env.faults == [1]
    assert env.gm._stop_pending


def test_p1_pending_guide_stop_response_exception_faults():
    """B 대상 goal-response Future 자체가 예외 → 수락 여부 불명 → FAULT + 봉쇄."""
    env = make_gm()
    env.gm.send_goal({'x': 1, 'y': 2}, tag='escape')
    env.gm.cancel_current_goal(intent='guide_stop')
    env.resp_cbs[0](failing_future())                  # 응답 자체가 예외
    assert env.faults == [1]
    assert env.gm._stop_pending
    env.gm.send_goal({'x': 9, 'y': 9})
    assert len(env.goals) == 1


def test_p1_pending_guide_stop_result_exception_faults():
    """늦은 수락 뒤 terminal(result) Future 예외 → 정지 확인 불가 → FAULT + 봉쇄."""
    env = make_gm()
    env.gm.send_goal({'x': 1, 'y': 2}, tag='escape')
    env.gm.cancel_current_goal(intent='guide_stop')
    late = FakeGoalHandle(accepted=True)
    env.resp_cbs[0](fake_future(late))
    late.result_cb(failing_future())
    assert env.faults == [1]
    assert env.gm._stop_pending


def test_p1_accepted_handle_result_exception_faults():
    """0723검토 §6.3 신규 P1: 수락된 핸들을 guide_stop 으로 취소한 뒤 terminal
    Future 가 예외면, 늦은 수락 경로와 대칭으로 즉시 FAULT + 봉쇄여야 한다.
    (구판은 _on_result 예외 분기가 현재 세대만 FAULT 하고 stale B 대상은 로그만
    남겨 faults=0 — 예산 소진까지 정지 불능이 관제에 안 드러났다.)"""
    env = make_gm()
    handle = send_and_accept(env, tag='escape')        # 수락된 핸들
    env.gm.cancel_current_goal(intent='guide_stop')
    handle.result_cb(failing_future())                 # terminal 수신 예외
    assert env.faults == [1]                           # ★ 즉시 FAULT
    assert env.gm._stop_pending                         # 봉쇄 유지
    env.gm.send_goal({'x': 9, 'y': 9})
    assert len(env.goals) == 1                          # 신규 goal 0건
    env.gm.send_goal({'x': 9, 'y': 9})
    assert len(env.goals) == 1


def test_p1_result_exception_symmetric_accepted_vs_pending():
    """§6.3 대칭성 앵커: accepted-handle 과 pending-late-accept 의 result Future
    예외가 동일한 FAULT 관찰값(faults==[1], _stop_pending=True)을 낸다 —
    두 경로가 공통 helper(_stop_target_terminal_lost)를 공유함을 고정한다."""
    a = make_gm()
    ha = send_and_accept(a, tag='escape')              # accepted-handle 경로
    a.gm.cancel_current_goal(intent='guide_stop')
    ha.result_cb(failing_future())

    b = make_gm()                                       # pending → 늦은 수락 경로
    b.gm.send_goal({'x': 1, 'y': 2}, tag='escape')
    b.gm.cancel_current_goal(intent='guide_stop')
    hb = FakeGoalHandle(accepted=True)
    b.resp_cbs[0](fake_future(hb))
    hb.result_cb(failing_future())

    assert a.faults == b.faults == [1]
    assert a.gm._stop_pending and b.gm._stop_pending


# --- P1 역회귀: 안전한 반대 경로 보존 (봉쇄가 과하지 않은지) ---
def test_p1_pending_guide_stop_late_rejected_releases():
    """전송 후 미수락 guide_stop → 늦은 '거부' → 실제 주행 goal 없음 → cancel 0건,
    직렬화 해제, 다음 goal 허용 (거부를 실패로 오인해 영구 봉쇄하면 안 됨)."""
    env = make_gm()
    env.gm.send_goal({'x': 1, 'y': 2}, tag='escape')
    env.gm.cancel_current_goal(intent='guide_stop')
    assert env.gm._stop_pending
    late = FakeGoalHandle(accepted=False)
    env.resp_cbs[0](fake_future(late))
    assert not late.cancel_called                      # 취소할 것도 없음
    assert not env.gm._stop_pending                    # 직렬화 정상 해제
    assert env.faults == []
    env.gm.send_goal({'x': 9, 'y': 9})
    assert len(env.goals) == 2


def test_p1_none_cancel_on_pending_keeps_stale_policy_no_b():
    """일반 취소(intent=None)의 수락응답 대기 goal — B 미무장, 늦은 수락은 기존
    stale 즉시 cancel 정책 유지(§22.3 보존), 봉쇄 없음."""
    env = make_gm()
    env.gm.send_goal({'x': 1, 'y': 2}, tag='patrol')
    env.gm.cancel_current_goal(intent=None)
    assert not env.gm._stop_pending                    # B 미무장
    late = FakeGoalHandle(accepted=True)
    env.resp_cbs[0](fake_future(late))
    assert late.cancel_called                          # 기존 stale 즉시 cancel 유지
    env.gm.send_goal({'x': 9, 'y': 9})                 # 봉쇄 없음
    assert len(env.goals) == 2


def test_p1_hard_then_late_cancel_response_no_false_fault():
    """hard 로 새 B 세대가 된 뒤 옛 세대 취소 응답이 늦게 와도 false FAULT 없음
    (모든 B 콜백을 대상 seq 에 귀속 — 검토자 §5 권장 8)."""
    env = make_gm()
    handle = send_and_accept(env, tag='escape')        # seq1 accepted
    env.gm.cancel_current_goal(intent='guide_stop')    # B 무장(seq1), handle 취소
    env.gm.cancel_current_goal(intent='hard')          # 직렬화 해제 + 세대 증가
    assert not env.gm._stop_pending
    handle.cancel_response_cb(fake_cancel_response(0))  # 옛 seq1 취소 응답 늦게 도착
    assert env.faults == []                            # ★ 새 세대에 false FAULT 없음


def test_p1_response_pending_seq_not_cleared_by_stale_response():
    """오래된 응답이 새 요청의 '수락응답 대기' 표시를 지우지 않는다(§5 권장 3)."""
    env = make_gm()
    env.gm.send_goal({'x': 1, 'y': 2})                 # seq1 pending
    env.gm.cancel_current_goal(intent=None)            # seq2 (seq1 stale)
    env.gm.send_goal({'x': 3, 'y': 4})                 # seq3 pending (덮어씀)
    assert env.gm._response_pending_seq == 3
    late1 = FakeGoalHandle(accepted=True)
    env.resp_cbs[0](fake_future(late1))                # 옛 seq1 응답 도착
    assert env.gm._response_pending_seq == 3           # ★ seq3 pending 은 그대로


# ============================================================
# ★ 완료조건 3: 껍데기 MissionNode + 진짜 GoalManager 조합 — 노드 진입점 통과
# ============================================================
def test_combo_node_send_success_mirrors_and_resets_faults():
    """노드의 send_goal 로 전송→수락→성공까지 태워, goal_active 미러와
    on_reached(fault_retries 리셋·patrol 전진)가 실제로 맞물리는지 본다."""
    node = MissionNode.__new__(MissionNode)
    node.state = State.PATROL
    node.patrol_idx = 0
    node.wp = {'patrol': [{'x': 0, 'y': 0}, {'x': 1, 'y': 1}]}
    node.fault_retries = 3
    node._cancel_intent = None
    logs = []
    node.get_logger = lambda: _logger(logs)
    goals_sent, resp_cbs = [], []
    nav = types.SimpleNamespace(
        server_is_ready=lambda: True,
        send_goal_async=lambda g: _mk_fut(goals_sent, g, resp_cbs))
    node.goals = GoalManager(
        nav, node.get_logger(),
        on_reached=node.on_reached, on_fault=lambda: None,
        on_active=lambda v: setattr(node, 'goal_active', v))

    node.send_goal(node.wp['patrol'][0], tag='patrol')
    assert node.goal_active                            # ★ 미러 True
    assert len(goals_sent) == 1
    handle = FakeGoalHandle(accepted=True)
    resp_cbs[-1](fake_future(handle))
    handle.result_cb(fake_result(SUCCEEDED))
    assert not node.goal_active                        # 종결 미러 False
    assert node.fault_retries == 0                     # ★ on_reached 가 리셋
    assert node.patrol_idx == 1                        # 다음 patrol


def test_combo_node_cancel_passes_intent_and_clears_hint():
    """노드 cancel_current_goal 이 _cancel_intent 힌트를 매니저로 전달하고
    소비 후 즉시 비운다(다음 취소로 새지 않게) — 진짜 매니저로 통과 검증."""
    node = MissionNode.__new__(MissionNode)
    node.state = State.GUIDE
    node._cancel_intent = None
    logs = []
    node.get_logger = lambda: _logger(logs)
    goals_sent, resp_cbs = [], []
    nav = types.SimpleNamespace(
        server_is_ready=lambda: True,
        send_goal_async=lambda g: _mk_fut(goals_sent, g, resp_cbs))
    node.goals = GoalManager(
        nav, node.get_logger(),
        on_reached=lambda: None, on_fault=lambda: None,
        on_active=lambda v: setattr(node, 'goal_active', v))

    node.send_goal({'x': 1, 'y': 2}, tag='escape')
    handle = FakeGoalHandle(accepted=True)
    resp_cbs[-1](fake_future(handle))

    node._cancel_intent = 'guide_stop'
    node.cancel_current_goal()
    assert node._cancel_intent is None                 # ★ 소비 후 비움
    assert node.goals._stop_pending                    # guide_stop 이 매니저에 전달됨


def test_combo_guide_speed_fail_maintenance_arms_guide_stop():
    """_on_guide_speed_fail 유지실패(②)가 진짜 cancel_current_goal 을 통해
    'guide_stop' 의도를 매니저로 밀어넣는지 (배선 누락·의도 오전달 방지)."""
    node = MissionNode.__new__(MissionNode)
    node.state = State.GUIDE
    node._guide_pending = False
    node._cancel_intent = None
    logs = []
    node.get_logger = lambda: _logger(logs)
    node._faults = []
    node.enter_fault = lambda: node._faults.append(1)
    goals_sent, resp_cbs = [], []
    nav = types.SimpleNamespace(
        server_is_ready=lambda: True,
        send_goal_async=lambda g: _mk_fut(goals_sent, g, resp_cbs))
    node.goals = GoalManager(
        nav, node.get_logger(),
        on_reached=lambda: None, on_fault=node.enter_fault,
        on_active=lambda v: setattr(node, 'goal_active', v))
    node.send_goal({'x': 1, 'y': 2}, tag='escape')
    handle = FakeGoalHandle(accepted=True)
    resp_cbs[-1](fake_future(handle))

    node._on_guide_speed_fail('lost')                  # 유지 실패 콜백
    assert node.goals._stop_pending                    # ★ guide_stop 무장됨
    assert node._faults == [1]                          # FAULT 로 종결


def test_combo_reset_hard_intent_clears_serialization():
    """on_cmd('reset')가 진행 중 유도정지 직렬화를 'hard'로 해제하는지 (배선)."""
    node = MissionNode.__new__(MissionNode)
    node.state = State.GUIDE
    node.patrol_idx = 5
    node.fire = 1
    node.gather_wp = 1
    node.gather_since = 1
    node._escaped_logged = True
    node.search_attempts = 2
    node.give_up = True
    node.last_seen = 1
    node.search_goal = 1
    node.refind_since = 1
    node.fault_retries = 1
    node.fault_since = 1
    node.resume_state = State.GUIDE
    node._guide_pending = False
    node._cancel_intent = None
    node.wp = {'normal_speed': 0.26}
    logs = []
    node.get_logger = lambda: _logger(logs)
    node.set_siren = lambda on: None
    node.speed = types.SimpleNamespace(
        cancel_pending=lambda r='': None,
        request_restore=lambda v: None)
    goals_sent, resp_cbs = [], []
    nav = types.SimpleNamespace(
        server_is_ready=lambda: True,
        send_goal_async=lambda g: _mk_fut(goals_sent, g, resp_cbs))
    node.goals = GoalManager(
        nav, node.get_logger(),
        on_reached=lambda: None, on_fault=lambda: None,
        on_active=lambda v: setattr(node, 'goal_active', v))
    node.send_goal({'x': 1, 'y': 2}, tag='escape')
    handle = FakeGoalHandle(accepted=True)
    resp_cbs[-1](fake_future(handle))
    node._cancel_intent = 'guide_stop'
    node.cancel_current_goal()
    assert node.goals._stop_pending                    # 유도정지 진행 중

    node.on_cmd(types.SimpleNamespace(data='reset'))
    assert not node.goals._stop_pending                # ★ hard 로 해제됨
    assert node.state == State.PATROL
