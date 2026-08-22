#!/usr/bin/env python3
"""`HOLD`(놓침 뒤 제자리 재수집) 회귀 — 2026-08-22 신설.

🔴 **왜 이 상태를 만들었나** — 08-21 21:32 실차 리허설에서 `SEARCH_BACK` 이 두 번
떴고 **두 번 다 15초 만에** `GUIDE` 로 복귀했다(339.0→353.5 · 365.5→381.0).
사람이 잠깐 안 보였을 뿐인데 **180° 역행(약 22초)** 을 시작했다 취소한 것이다.
`HOLD` 는 그 앞에 **싼 한 걸음**을 넣는다: 서서 4초 더 본다.

`FollowerMonitor` 가 이미 쓰는 비대칭과 같은 철학이다 —
*"놓침 선언(비싼 역행 유발)은 신중히, 재발견은 빠르게."*

하네스는 `test_search_back_entry.py` 의 것을 그대로 쓴다 — **진짜 FollowerMonitor +
진짜 waypoints.yaml** 이라, 여기 숫자가 생산 설정과 어긋나면 시험이 깨진다.
"""
import test_search_back_entry as T
from mission_manager.mission_node import HOLD_SEC_DEFAULT, State

EMPTY = T.EMPTY
PERSON = T.PERSON


def hold_sec():
    """생산 설정에서 읽는다 — 시험이 값을 베껴 적으면 설정이 바뀌어도 초록이다."""
    return float(T.load_wp()['search_back'].get('hold_sec', HOLD_SEC_DEFAULT))


def test_h1_loss_enters_hold_not_search_back():
    """놓침 확정은 **먼저 HOLD** 로 간다. 곧바로 역행하지 않는다."""
    env = T.make_env()
    T.run(env, 0.1, PERSON)
    T.run(env, 3.6, EMPTY)                 # lost_sec(3.0) 초과
    assert env.node.state == State.HOLD, \
        '놓침 직후 곧바로 역행했다 — HOLD 가 건너뛰어졌다'
    assert env.node.search_attempts == 0, \
        '🔴 HOLD 는 역행이 아니다 — 역행 예산을 깎으면 안 된다'


def test_h2_refind_while_holding_skips_the_expensive_turn():
    """🎯 **이 도구의 존재 이유** — 서서 기다리다 다시 보이면 역행을 안 한다.

    08-21 리허설의 두 번이 이 가지로 왔어야 했다. 그랬으면 21초짜리 역행을
    두 번 아꼈다.
    """
    env = T.make_env()
    T.run(env, 0.1, PERSON)
    T.run(env, 3.6, EMPTY)
    assert env.node.state == State.HOLD
    T.run(env, 1.5, PERSON)                # seen_sec(1.0) 충족
    assert env.node.state == State.GUIDE, '제자리 재발견인데 복귀하지 않았다'
    assert env.node.search_attempts == 0, \
        '🔴 역행을 하지 않았는데 예산이 깎였다'


def test_h3_returning_from_hold_does_not_relose_immediately():
    """🔴 부정 회귀 — 복귀 직후 곧바로 다시 놓치면 HOLD 를 만든 의미가 없다.

    `[reset-role] hold-return` 이 빠지면 `lost` 타이머가 놓침 당시 그대로라
    복귀 첫 tick 에 다시 lost 가 참이 되고, 아낀 역행이 결국 나간다.
    """
    env = T.make_env()
    T.run(env, 0.1, PERSON)
    T.run(env, 3.6, EMPTY)
    T.run(env, 1.5, PERSON)
    assert env.node.state == State.GUIDE
    T.run(env, 2.5, EMPTY)                 # lost_sec 미만
    assert env.node.state == State.GUIDE, \
        '복귀 직후 세대가 재무장되지 않아 즉시 재-놓침했다'


def test_h4_hold_times_out_into_search_back():
    """정말 사라졌으면 hold_sec 뒤에는 역행으로 넘어간다 — 갇히지 않는다."""
    env = T.make_env()
    T.run(env, 0.1, PERSON)
    T.run(env, 3.6, EMPTY)
    assert env.node.state == State.HOLD
    T.run(env, hold_sec() + 0.6, EMPTY)
    assert env.node.state == State.SEARCH_BACK, \
        f'hold_sec({hold_sec()}) 를 넘겼는데 역행이 안 열렸다'
    assert env.node.search_attempts == 1


def test_h5_hold_issues_no_goal_that_is_the_stop():
    """🔴 HOLD 는 goal 을 하나도 내지 않는다 — 그것이 '정지'의 구현이다."""
    env = T.make_env()
    T.run(env, 0.1, PERSON)
    T.run(env, 3.6, EMPTY)
    assert env.node.state == State.HOLD
    before = len(env.goals)
    T.run(env, hold_sec() - 1.0, EMPTY)    # HOLD 를 유지하는 동안
    assert env.node.state == State.HOLD
    assert len(env.goals) == before, \
        f'HOLD 중 goal 이 {len(env.goals) - before}건 나갔다 — 로봇이 계속 간다'


def test_h6_a_dead_lidar_does_not_push_us_into_a_blind_reversal():
    """🔴 부정 회귀 — `/scan` 이 죽은 동안은 HOLD 시간을 세지 않는다.

    세면 **센서가 죽었다는 이유로 역행이 시작된다.** 그때의 역행은 눈을 감고
    도는 것이다. 같은 취지의 보류가 GUIDE 의 `record_last_seen` 에도 걸려 있다.
    """
    env = T.make_env()
    T.run(env, 0.1, PERSON)
    T.run(env, 3.6, EMPTY)
    assert env.node.state == State.HOLD
    T.tick_only(env, hold_sec() + 3.0)     # /scan 한 장도 없이 시간만 흐른다
    assert env.node.monitor.scan_stale(), '전제 불성립: stale 이 안 됐다'
    assert env.node.state == State.HOLD, \
        '🔴 라이다가 죽은 채로 역행에 들어갔다'
    assert env.node.search_attempts == 0


def test_h7_a_refused_reversal_does_not_trap_us_in_hold():
    """🔴 부정 회귀 — 역행이 거부되면 GUIDE 로 돌아가야 한다.

    `enter_search_back` 은 예산 소진·`last_seen is None` 이면 **상태를 안 바꾸고
    그냥 돌아온다.** 구판은 호출자가 GUIDE 라 그대로 단독 탈출로 이어졌다.
    HOLD 에서 부르면 그 경로가 **HOLD 에 갇힌다 — 로봇이 영원히 서 있는다.**
    """
    env = T.make_env(tf_ok=False)          # TF 사망 → last_seen 이 영원히 None
    T.run(env, 0.1, PERSON)
    T.run(env, 3.6, EMPTY)
    assert env.node.state == State.HOLD
    T.run(env, hold_sec() + 1.0, EMPTY)
    assert env.node.state == State.GUIDE, \
        '🔴 역행이 거부됐는데 HOLD 에 갇혔다 — 로봇이 영원히 정지한다'
    assert env.node.search_attempts == 1, '거부도 예산은 깎아야 보고 경로가 열린다'


def test_h8_the_report_path_still_opens_when_reversal_never_works():
    """역행이 매번 거부돼도 결국 관제 보고(give_up)까지 간다 — 갇힘 없음."""
    env = T.make_env(tf_ok=False)
    T.run(env, 0.1, PERSON)
    T.run(env, 25.0, EMPTY)
    assert env.node.give_up, \
        'HOLD 를 넣으면서 보고 경로가 막혔다'


def test_h9_hold_sec_is_optional_and_defaults_safely():
    """🔴 촬영 중 키 하나가 없어서 미션이 죽는 것보다 기본값으로 도는 편이 낫다.

    `validate_waypoints()` 머리말의 규약대로 `.get(기본값)` 선택 키다.
    """
    env = T.make_env()
    env.node.wp['search_back'].pop('hold_sec', None)
    T.run(env, 0.1, PERSON)
    T.run(env, 3.6, EMPTY)
    assert env.node.state == State.HOLD
    T.run(env, HOLD_SEC_DEFAULT + 0.6, EMPTY)
    assert env.node.state == State.SEARCH_BACK, \
        f'hold_sec 부재 시 기본값 {HOLD_SEC_DEFAULT} 로 동작하지 않았다'


def test_h10_production_config_carries_the_key():
    """🔵 기본값이 있어도 **실제 설정에는 적어 둔다** — 읽는 사람이 알아야 한다."""
    sb = T.load_wp()['search_back']
    assert 'hold_sec' in sb, 'waypoints.yaml 에 hold_sec 이 없다'
    assert 0.0 < float(sb['hold_sec']) <= 10.0, \
        '역행(약 22초) 앞의 보험치고 너무 길거나 0 이다'


# ── 🔴 08-22 P0 — 정지를 선언하기 전에 취소가 종결됐는지 확인한다 ──────
# 독립 검토 §87.2. 구판은 인자 없는 `cancel_current_goal()` 을 부르고 곧바로
# "섰다"고 선언했다. 일반 취소는 `GoalManager._stop_pending` 을 무장하지 않으므로
# **노드의 미러만 false 가 되고 Nav2 의 옛 goal 은 계속 달릴 수 있었다.**
# `BLOCKED` 이 §84.2 에서 바로 이 이유로 고쳐졌는데 신규 상태가 그 전 패턴을 다시 썼다.

def lose_person(env, seconds=3.6):
    T.run(env, 0.1, PERSON)
    T.run(env, seconds, EMPTY)


def test_h11_entering_hold_asks_for_a_confirmed_stop():
    """🔴 일반 취소가 아니라 **safety_stop** 이어야 신규 goal 이 봉쇄된다."""
    env = T.make_env()
    seen = []
    orig = env.node.cancel_current_goal

    def spy():
        seen.append(env.node._cancel_intent)
        orig()
    env.node.cancel_current_goal = spy
    lose_person(env)
    assert env.node.state == State.HOLD
    assert seen == ['safety_stop'], f'취소 의도가 {seen} 였다 — 정지가 무장되지 않는다'


def test_h12_a_cancel_that_never_confirms_does_not_become_a_reversal():
    """🔴 부정 회귀 — 취소가 종결되지 않으면 **역행으로 넘어가지 않는다.**

    로봇이 아직 달리고 있을 수 있는데 새 goal 을 얹으면 옛 goal 과 겹친다.
    사람이 E-stop 으로 물리 정지를 확인해야 하는 상황이므로 그 자리에 머문다.
    """
    env = T.make_env()
    env.stop_ok[0] = False                 # 취소가 CANCELED 로 안 끝난다
    lose_person(env)
    assert env.node.state == State.HOLD
    assert env.node.stop_state == 'unconfirmed'
    before = len(env.goals)
    T.run(env, hold_sec() + 5.0, EMPTY)    # 넉넉히 넘겨도
    assert env.node.state == State.HOLD, '정지 미확인인데 역행으로 넘어갔다'
    assert len(env.goals) == before, '정지 미확인인데 신규 goal 이 나갔다'


def test_h13_an_unconfirmed_stop_never_starts_the_clock():
    """🔴 확인 전의 4초는 "서서 기다린 4초" 가 아니다 — 세면 안 된다."""
    env = T.make_env()
    env.stop_ok[0] = False
    lose_person(env)
    T.run(env, hold_sec() + 2.0, EMPTY)
    assert env.node.hold_since is None, '정지가 확인되지 않았는데 기산이 열렸다'


def test_h14_a_healthy_cancel_opens_the_clock_at_once():
    """🔵 역회귀 — 정상 CANCELED 는 기다릴 이유가 없다. 거짓 FAULT 도 없다."""
    env = T.make_env()
    lose_person(env)
    assert env.node.state == State.HOLD
    assert env.node.stop_state == 'confirmed'
    assert env.node.hold_since is not None
    T.run(env, hold_sec() + 0.6, EMPTY)
    assert env.node.state == State.SEARCH_BACK, '정상 취소인데 역행이 안 열렸다'


# ── 🔴 08-22 §88.2 — 종결 무응답 상한 · steady clock ──────────────────
def test_h15_a_stop_that_never_terminates_becomes_unconfirmed():
    """🔴 부정 회귀 — 종결 콜백이 **영원히 안 오면** 상한이 실패로 승격해야 한다.

    ⚠ 판정은 `tick` 이 아니라 `safety_watchdog` 이 한다(§89.2) — `/clock` 이 멈추면
    tick 자체가 안 불리므로 그 안에 두면 평가될 기회가 없다.
    """
    import mission_manager.mission_node as MN

    env = T.make_env()
    env.node.arm_stop_deadline(True)
    assert env.node.stop_state == 'pending'
    real, base = MN.time.monotonic, env.node._stop_pending_since
    MN.time.monotonic = lambda: base + MN.STOP_CONFIRM_TIMEOUT_SEC + 0.1
    try:
        env.node.safety_watchdog()
    finally:
        MN.time.monotonic = real
    assert env.node.stop_state == 'unconfirmed', \
        '🔴 종결이 영원히 안 왔는데 pending 에 머물렀다'


def test_h15b_the_deadline_does_not_fire_early():
    """🔵 역회귀 — 상한 전에 실패로 올리면 정상 취소가 E-stop 요구가 된다."""
    import mission_manager.mission_node as MN

    env = T.make_env()
    env.node.arm_stop_deadline(True)
    real, base = MN.time.monotonic, env.node._stop_pending_since
    MN.time.monotonic = lambda: base + MN.STOP_CONFIRM_TIMEOUT_SEC - 0.5
    try:
        env.node.safety_watchdog()
    finally:
        MN.time.monotonic = real
    assert env.node.stop_state == 'pending'


def test_h16_the_safety_deadline_runs_off_a_steady_timer():
    """🔴 §89.2 — **계산이 monotonic 이어도 호출이 ROS clock 이면 소용없다.**

    2차 보완이 정확히 그 상태였다: `time.monotonic()` 비교를 `tick()` 안에 뒀는데,
    `tick` 타이머는 노드 기본 clock(ROS_TIME)이라 `use_sim_time` + `/clock` 정지에서
    **아예 안 불린다.** 구판 `test_h16` 은 함수 소스를 문자열로 봐서 이 경계를 놓쳤다.
    """
    import inspect

    import mission_manager.mission_node as MN

    tick_src = inspect.getsource(MN.MissionNode.tick)
    assert 'STOP_CONFIRM_TIMEOUT_SEC' not in tick_src, \
        '안전 상한이 아직 ROS clock tick 안에 있다'
    wd = inspect.getsource(MN.MissionNode.safety_watchdog)
    assert 'STOP_CONFIRM_TIMEOUT_SEC' in wd and 'time.monotonic' in wd
    assert 'SCAN_GOAL_TIMEOUT_DEFAULT' in wd, '훑기 상한도 여기 있어야 한다(§89.4)'
    init = inspect.getsource(MN.MissionNode.__init__)
    assert 'ClockType.STEADY_TIME' in init, '정상시계 타이머가 없다'
    assert 'clock=self._steady_clock' in init, '워치독이 그 시계를 안 쓴다'


def test_h17_every_safety_stop_entry_arms_the_deadline():
    """🔴 §89.2 — **safety_stop 을 여는 자리가 전부 공통 무장을 타는가.**

    1·2차 보완은 `_confirmed_stop()` 안에만 기산을 뒀고, `BLOCKED` 진입과 `SCAN`
    타임아웃이 그것을 안 탔다 — 재현값 `BLOCKED / pending / since=None`.
    새 진입을 만들 때 `arm_stop_deadline()` 을 부르는 것이 계약이다.
    """
    import inspect

    import mission_manager.mission_node as MN

    src = inspect.getsource(MN.MissionNode)
    setters = src.count("_cancel_intent = 'safety_stop'")
    # ⚠ `def arm_stop_deadline(` 도 같은 문자열을 포함한다 — 정의를 호출로 세면
    #   진입 하나가 무장을 안 타도 숫자가 맞아버린다(실제로 변이가 통과했다).
    arms = src.count('self.arm_stop_deadline(')
    assert arms >= setters, (
        f'safety_stop 세팅 {setters}곳 vs 공통 무장 {arms}곳 — '
        f'무장을 안 타는 진입이 있다')


def test_h18_the_scan_deadline_fires_from_the_steady_watchdog():
    """🔴 §89.4 — 훑기 상한도 `/clock` 과 무관하게 돌아야 한다.

    ⚠ 소스에 상수 이름이 있는지만 보면 **호출을 통째로 죽여도 통과한다**
    (변이로 실제 확인했다). 동작으로 본다.
    """
    import mission_manager.mission_node as MN

    env = T.make_env(state=State.SCAN_AREA)
    env.node.wp['scan_goal_timeout_sec'] = 3.0
    env.node.goal_active = True
    env.node.scan_goal_since = 100.0
    real = MN.time.monotonic
    MN.time.monotonic = lambda: 110.0
    try:
        env.node.safety_watchdog()
    finally:
        MN.time.monotonic = real
    assert env.node.state == MN.State.FAULT, \
        '🔴 훑기 무응답이 steady 워치독에서 유한 종결되지 않았다'


def test_h18b_a_scan_within_the_deadline_is_left_alone():
    """🔵 역회귀 — 상한 안의 정상 스텝을 자르면 훑기가 아예 안 된다."""
    import mission_manager.mission_node as MN

    env = T.make_env(state=State.SCAN_AREA)
    env.node.wp['scan_goal_timeout_sec'] = 90.0
    env.node.goal_active = True
    env.node.scan_goal_since = 100.0
    real = MN.time.monotonic
    MN.time.monotonic = lambda: 110.0
    try:
        env.node.safety_watchdog()
    finally:
        MN.time.monotonic = real
    assert env.node.state == MN.State.SCAN_AREA
