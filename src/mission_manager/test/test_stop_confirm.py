#!/usr/bin/env python3
"""안전정지의 **비동기 전 단계** 보호 회귀 — 2026-08-22 (독립 검토 §88.2).

🔴 **1차 보완이 동기 경로만 고쳤다.** `_confirmed_stop()` 이 `safety_stop` 을 걸고
순서도 맞췄지만, `GoalManager` 의 **비동기 등록 네 자리**가 맨몸이었다.

    get_result_async()  ·  add_done_callback()  — 둘 다 **동기 예외를 낼 수 있다**

예외가 새면 그 뒤 줄이 아예 안 돌고 `_stop_pending` 만 True 로 남는다. 그러면
**상위는 정지 여부를 영원히 모르고**, 최악은 `:318` — 수락돼 주행 중인 옛 goal 에
**취소를 못 보낸다.** 그건 직접적인 이동 P0 다.
"""
import types

import test_goal_lifecycle as G


def stop_target(env):
    """safety_stop 세대를 하나 만든다 — 이후 실패가 `_stop_failed` 로 귀속되게."""
    env.gm.send_goal({'x': 1.0, 'y': 0.0, 'yaw': 0.0}, tag='t')
    env.gm.cancel_current_goal(intent='safety_stop')


def test_s1_a_result_registration_that_throws_does_not_skip_the_cancel():
    """🔴 **§88.2 ① 이동 P0** — 결과 감시 등록이 터져도 **취소는 나가야 한다.**

    구판은 `handle.get_result_async().add_done_callback(...)` 이 맨몸이라, 예외가
    나면 **그 다음 줄의 `_cancel_with_confirm` 이 아예 안 불렸다.** Nav2 가 수락해
    주행 중인 옛 goal 이 그대로 달린다.
    """
    env = G.make_gm()
    env.gm.send_goal({'x': 1.0, 'y': 0.0, 'yaw': 0.0}, tag='t')
    env.gm.cancel_current_goal(intent='safety_stop')      # 응답 전에 취소 → stale 세대
    cancels = []
    handle = types.SimpleNamespace(
        accepted=True,
        get_result_async=lambda: (_ for _ in ()).throw(RuntimeError('등록 실패')),
        cancel_goal_async=lambda: (cancels.append(1),
                                   G._mk_plain_fut())[1] if hasattr(
                                       G, '_mk_plain_fut') else cancels.append(1))
    fut = types.SimpleNamespace(result=lambda: handle, exception=lambda: None)
    env.resp_cbs[0](fut)                                   # 뒤늦은 수락
    assert cancels, '🔴 결과 감시 등록이 터지자 취소 시도 자체가 건너뛰어졌다'


def test_s2_a_cancel_callback_registration_that_throws_reports_failure():
    """🔴 §88.2 ② — 취소 응답 등록이 터지면 **정지 실패로 올라와야** 한다.

    구판은 `fut.add_done_callback` 이 try 밖이라 예외가 샜고, `_stop_failed` 가
    안 불려 상위가 취소 접수 여부를 몰랐다.
    """
    env = G.make_gm()
    env.gm.send_goal({'x': 1.0, 'y': 0.0, 'yaw': 0.0}, tag='t')
    bad_fut = types.SimpleNamespace(
        add_done_callback=lambda cb: (_ for _ in ()).throw(RuntimeError('등록 실패')))
    handle = types.SimpleNamespace(accepted=True,
                                   cancel_goal_async=lambda: bad_fut,
                                   get_result_async=lambda: bad_fut)
    env.gm._handle = handle
    env.gm._stop_pending = True
    env.gm._stop_seq = env.gm._seq
    env.gm._stop_intent = 'safety_stop'
    env.gm._cancel_with_confirm(handle, '시험', stop_seq=env.gm._seq)
    assert any('등록 실패' in m or '정지' in m for m in env.logs), \
        '🔴 등록 예외가 조용히 새고 정지 실패가 보고되지 않았다'


def test_s3_the_watch_boundary_never_lets_an_exception_escape():
    """🔵 `_watch` 는 어떤 경우에도 예외를 밖으로 내보내지 않는다."""
    env = G.make_gm()
    ok = env.gm._watch(lambda: (_ for _ in ()).throw(RuntimeError('x')),
                       lambda f: None, '시험')
    assert ok is False
    ok2 = env.gm._watch(
        lambda: types.SimpleNamespace(
            add_done_callback=lambda cb: (_ for _ in ()).throw(RuntimeError('y'))),
        lambda f: None, '시험2')
    assert ok2 is False
