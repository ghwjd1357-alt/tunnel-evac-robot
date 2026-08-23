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


# ============================================================
# 🔴 08-23 §91 P0-3 — /scan 이 죽었을 때 GUIDE 가 **계속 간다**는 사실을 못박는다
# ============================================================
#
# 검토 §91 P0-3 은 *"GUIDE 중 scan publisher 중단 시 1 tick 안에 정지 명령과 신규 goal
# 차단"* 을 요구했다. **지금 코드는 그렇게 하지 않는다** — `mission_node.py` 의 GUIDE
# 분기는 `'⚠ /scan 끊김 — 추종감시 불가 (유도는 계속)'` 을 찍고 goal 을 유지한다.
#
# 🔴 이건 버그가 아니라 **의도된 설계**다. `FollowerMonitor.lost()` 주석이 근거를 적어
#   뒀다: *"놓침 선언은 비싼 역행을 일으키므로 데이터 없이 내리면 안 됨 — 대피 유도는
#   계속하는 게 안전한 쪽."* 불타는 터널에서 사람을 데리고 나가던 로봇이 라이다가
#   죽었다고 그 자리에 서면, 그게 더 위험하다는 판단이다.
#
# 🔴 그런데 **촬영에서는 전제가 뒤집힌다.** 불이 없고, 대본상 사람과 장애물이 경로
#   근처에 있다. 앞을 못 보면서 달리는 쪽이 위험하다.
#
# 그래서 08-23 에는 **코드를 안 바꾸고** 다음 셋으로 처리했다:
#   ① 런북 §10·§12 = 라이다 이상 인지 **즉시 E-stop · 그 테이크 폐기** (사람이 막는다)
#   ② 이 검사 = 현재 거동을 **명시적으로 고정**한다. 누가 조용히 뒤집으면 여기서 깨진다.
#   ③ `MASTER_PLAN §7` 예약 = 어느 쪽을 정본으로 삼을지 사용자 결정 항목으로 올린다.
#
# ⚠ 코드를 안 고친 이유를 남긴다: 정지를 새로 넣으려면 **이미 4회차 불승인 상태인
#   정지 직렬화 사슬**(§90.1·§90.2)에 경로를 하나 더 붙여야 하고, 실차 검증 없이
#   새벽에 그걸 하는 것이 `AGENTS §6` 이 말하는 발산이다.
#
# 🔵 이 검사를 **깨뜨리는 쪽이 정답이 되는 날**이 온다 — 그때 이 주석째로 갈아엎어라.


def test_h_p0_3_guide_keeps_driving_while_scan_is_dead():
    """🔴 현재 계약: /scan 이 죽어도 GUIDE 는 유지되고 goal 도 안 취소된다."""
    env = T.make_env()
    T.run(env, 1.0, PERSON)                 # 정상 유도 중
    # 🔴 08-23 §91(2회차) P1-1 — 구판은 `cancels_before = env.cancels` 로 **리스트
    #   자체를 별칭 저장**했다. 하네스가 같은 리스트에 append 하므로 아래 비교가
    #   **항상 참**이 되어, 'stale 중에 취소가 붙는' 변이를 통과시켰다(검토가 주입해 확인).
    #   → 정수 스냅샷으로 바꾼다.
    goals_before, cancels_before = len(env.goals), len(env.cancels)

    T.tick_only(env, 3.0)                   # 🔴 /scan 한 장도 안 온다 (timeout 1.0s)

    assert env.node.monitor.scan_stale(), '전제 실패 — stale 이 안 됐다'
    assert env.node.state == State.GUIDE, \
        ('GUIDE 가 유지되지 않았다. 정지를 넣기로 **결정**했다면 이 검사와 위 주석을 '
         '같이 갈아엎어라 — 조용히 통과시키지 마라.')
    assert len(env.cancels) == cancels_before, \
        '🔴 거동이 바뀌었다: scan 끊김에 취소가 붙었다 (§91 P0-3 결정 없이)'
    assert len(env.goals) == goals_before, \
        '🔴 거동이 바뀌었다: scan 끊김 중에 새 goal 이 나갔다'


def test_h_p0_3_the_only_thing_scan_death_does_is_warn():
    """그 구간에 로봇이 하는 일은 **경고 한 줄**뿐이라는 것까지 고정한다."""
    env = T.make_env()
    T.run(env, 1.0, PERSON)
    env.logs.clear()
    T.tick_only(env, 3.0)

    warned = [m for m, _ in env.logs if '/scan 끊김' in m]
    assert warned, '🔴 라이다가 죽었는데 경고조차 안 나왔다 — 관제가 모른다'
    assert all('추종감시 불가' in m for m in warned), \
        f'경고 문구가 바뀌었다 — 런북 §10 이 이 문구를 인용한다: {warned}'


# ============================================================
# 🔴 08-23 §91 P0-2 — 재발견 취소가 **직렬화 의도를 실어 보내는지** (호출 자리)
# ============================================================
#
# `test_goal_lifecycle.py` 의 같은 번호 시험은 `GoalManager` 기구만 본다 —
# `cancel_current_goal(intent='guide_stop')` 를 손으로 부르므로 **호출 자리를
# 안 지난다.** 실제로 `mission_node.py` 의 수정을 되돌려도 그 파일은 전부 통과했다.
#
# 여기서는 **진짜 상태머신을 SEARCH_BACK 까지 굴린 뒤** 재발견 가지가 실제로
# `_cancel_intent = 'guide_stop'` 을 세우고 취소하는지 본다. 수정을 되돌리면
# 여기가 깨진다 — 그게 이 시험의 존재 이유다.
#
# ⚠ 하네스의 `cancel_current_goal` 은 가짜라 intent 를 안 본다. 그래서 시험이
#   그 함수를 **감싸서** 호출 순간의 `node._cancel_intent` 를 기록한다.
#   (하네스 자체를 고치면 다른 시험의 `env.cancels` 계약이 흔들린다.)


def _spy_cancel(env):
    """취소가 불릴 때의 `_cancel_intent` 를 순서대로 기록한다."""
    seen = []
    inner = env.node.cancel_current_goal

    def wrapped():
        seen.append(env.node._cancel_intent)
        return inner()

    env.node.cancel_current_goal = wrapped
    return seen


def _drive_to_search_back(env):
    """GUIDE → 놓침 → HOLD → hold_sec 경과 → SEARCH_BACK 까지 진짜 경로로 굴린다."""
    T.run(env, 0.1, PERSON)
    T.run(env, 3.6, EMPTY)                      # lost_sec(3.0) 초과 → HOLD
    assert env.node.state == State.HOLD, '전제 실패 — HOLD 에 못 갔다'
    T.run(env, hold_sec() + 0.6, EMPTY)         # hold_sec 경과 → SEARCH_BACK
    assert env.node.state == State.SEARCH_BACK, \
        f'전제 실패 — SEARCH_BACK 에 못 갔다 (지금 {env.node.state})'


def test_h_p0_2_refind_cancel_carries_guide_stop_intent():
    """🎯 재발견 취소는 **`guide_stop`** 이어야 한다 — 일반 취소면 직렬화가 안 걸린다."""
    env = T.make_env()
    _drive_to_search_back(env)
    intents = _spy_cancel(env)

    # seen_sec 연속 검출 → 재발견 가지. 🔴 08-23 §91(3회차) 이후 복귀는 **두 tick**
    #   이라(취소 → 종결 확인 → GUIDE) 한 tick 여유를 더 준다.
    T.run(env, float(T.load_wp()['search_back']['seen_sec']) + 1.2, PERSON)

    assert env.node.state == State.GUIDE, '재발견했는데 GUIDE 로 복귀하지 않았다'
    assert intents, '재발견 복귀인데 취소를 아예 안 불렀다 — 역행 goal 이 살아 있다'
    assert intents[-1] == 'guide_stop', (
        '🔴 §91 P0-2 재발생 — 재발견 취소가 일반 취소(intent=%r)로 나갔다. '
        'B 직렬화가 안 걸려 역행 goal 의 취소가 종결되기 전에 다음 GUIDE goal 이 '
        '나간다.' % (intents[-1],))


def test_h_p0_2_hold_return_needs_no_cancel_which_is_why_only_search_back_broke():
    """역회귀 — HOLD 복귀는 **취소 자체가 없다**(로봇이 이미 서 있다).

    이 대비가 §91 P0-2 의 요지다: 두 재발견 가지 중 취소를 내는 쪽은 SEARCH_BACK
    뿐이고, 그래서 그 한 곳만 직렬화 의도를 빠뜨릴 수 있었다.
    """
    env = T.make_env()
    T.run(env, 0.1, PERSON)
    T.run(env, 3.6, EMPTY)
    assert env.node.state == State.HOLD
    intents = _spy_cancel(env)

    T.run(env, 1.5, PERSON)                     # HOLD 복귀 창(4.0 − seen_sec) 안

    if env.node.state == State.GUIDE:
        assert not intents, \
            f'HOLD 복귀가 취소를 냈다 — 로봇은 이미 서 있다: {intents}'
    else:
        # seen_sec 3.0 · hold_sec 4.0 이면 복귀 창이 1초뿐이라 여기로 온다.
        # 그 사실 자체를 고정한다 (§91 P1-1 · 샷리스트 ⑪ 이 이 값에 의존한다).
        assert env.node.state == State.SEARCH_BACK, \
            f'HOLD 도 GUIDE 도 아닌 곳으로 갔다: {env.node.state}'


# ============================================================
# 🔴 08-23 §91(2회차) P0-2 — 취소가 **동기 실패**하면 FAULT 를 덮지 않는다
# ============================================================
#
# 1회차 보완(`_cancel_intent='guide_stop'`)이 **새 P0 를 만들었다.**
# `guide_stop` 은 `stop_seq` 를 세우는 유일한 의도 중 하나라, 취소 요청이 동기
# 예외로 실패하면 `_stop_failed()` → `enter_fault()` 가 **그 자리에서** 돈다.
# 그런데 그 다음 줄이 무조건 `state = GUIDE` 를 썼다 → FAULT 가 지워진다.
# 검토 주입 관찰값: `state=GUIDE · resume_state=SEARCH_BACK · stop_pending=True ·
#                    goal_active=False · sent_goals=1 · FAULT 로그 있음`
#
# ⚠ 하네스의 가짜 취소는 항상 성공한다. 그래서 이 시험은 **실패하는 취소**를
#   직접 만들어 끼운다 — 생산 경로가 하는 것과 같은 순서로
#   (`enter_fault()` 를 취소 안에서 부른다).


def _make_cancel_fail_synchronously(env):
    """취소가 `enter_fault()` 를 동기로 부르고 돌아오게 바꾼다 (생산 실패 경로 모사)."""
    def failing_cancel():
        env.cancels.append(1)
        env.node._cancel_intent = None      # 생산과 같이 의도를 소비한다
        env.node.goal_active = False
        env.node.enter_fault()              # `_stop_failed` → `enter_fault` 와 같은 지점
    env.node.cancel_current_goal = failing_cancel


def test_h_p0_2_sync_cancel_failure_must_not_be_overwritten_by_guide():
    """🎯 취소가 동기 실패해 FAULT 면, 재발견 복귀가 그것을 **덮지 않는다**."""
    env = T.make_env()
    _drive_to_search_back(env)
    _make_cancel_fail_synchronously(env)

    T.run(env, float(T.load_wp()['search_back']['seen_sec']) + 0.6, PERSON)

    assert env.cancels, '전제 실패 — 재발견 가지가 취소를 안 불렀다'
    assert env.node.state == State.FAULT, (
        '🔴 §91(2회차) P0-2 재발생 — 취소가 동기 실패해 FAULT 였는데 '
        f'{env.node.state} 로 덮였다. 내부는 정지 실패를 아는데 관제는 그걸 못 본다.')
    assert env.node.resume_state == State.SEARCH_BACK, (
        '재개 지점이 SEARCH_BACK 이 아니다 — FAULT 재시도가 엉뚱한 상태로 돌아간다: '
        f'{env.node.resume_state}')


def test_h_p0_2_healthy_cancel_still_returns_to_guide():
    """역회귀 — 취소가 정상이면 GUIDE 로 복귀한다 (과잉 방어 금지).

    🔴 08-23 §91(3회차) P0-1 이후 이 복귀는 **두 tick**이다:
      tick A — 역행 goal 을 `guide_stop` 으로 취소. 상태는 **SEARCH_BACK 유지**.
      tick B — 정지가 CANCELED 로 종결된 것을 관찰하고 그때 GUIDE.
    중간 상태(`refind_stopping`)를 여기서 같이 고정한다 — 이 단계가 사라지면
    늦은 취소 실패가 다시 `resume_state=GUIDE` 를 만든다.
    """
    env = T.make_env()
    _drive_to_search_back(env)
    seen = float(T.load_wp()['search_back']['seen_sec'])

    # tick A 까지만 — 취소는 나갔지만 아직 SEARCH_BACK 이어야 한다
    T.run(env, seen + 0.6, PERSON)
    assert env.node.state == State.SEARCH_BACK, (
        '취소와 동시에 GUIDE 로 갔다 — 늦은 취소 실패가 GUIDE 를 resume_state 로 '
        f'저장하는 §91(3회차) P0-1 이 되살아났다: {env.node.state}')
    assert env.node.refind_stopping, '취소를 냈는데 대기 깃발이 안 섰다'

    # tick B — 종결이 확인됐으므로 이제 GUIDE
    T.run(env, 0.6, PERSON)
    assert env.node.state == State.GUIDE, \
        f'정지 종결이 확인됐는데 GUIDE 로 복귀하지 않았다: {env.node.state}'
    assert not env.node.refind_stopping, '복귀했는데 대기 깃발이 안 내려갔다'


# ============================================================
# 🔴 08-23 §91(3회차) P0-1 — **늦게 오는** 취소 실패가 GUIDE 를 재개점으로 만들면 안 된다
# ============================================================
#
# 2회차 보완은 **동기** 실패만 막았다. 검토 3회차가 늦은 실패 세 경로를 주입해
# 같은 결함을 다시 냈다:
#   ① 취소응답 Future 예외  ② 빈 `goals_canceling`  ③ non-CANCELED terminal
# 셋 다 `state=FAULT · resume_state=GUIDE · stop_pending=True` 였고,
# `RETRY_WAIT` 뒤 **정지 미확인 goal 을 안은 채 GUIDE 로 자동재개**했다.
#
# 원인은 타이밍이 아니라 **순서**였다 — 재발견 분기가 취소 직후 이미 GUIDE 라서,
# 나중에 도착한 실패가 `enter_fault()` 를 부를 때 저장되는 `resume_state` 가
# GUIDE 였던 것이다. 그래서 보완은 "실패를 잡는다" 가 아니라 **"종결 전에는 GUIDE 가
# 되지 않는다"** 여야 한다.
#
# 아래 시험은 늦은 실패를 **tick 경계 뒤에** 주입해 그 순서를 고정한다.


def _late_stop_failure(env, unconfirmed=True):
    """취소가 나간 **뒤에** 실패가 도착하는 상황을 만든다 (생산 `_stop_failed` 경로)."""
    if unconfirmed:
        env.node.on_safety_stop_unconfirmed('하네스: 늦은 취소 실패 모사')
    env.node.enter_fault()


def test_h_p0_1_late_cancel_failure_resumes_to_search_back_not_guide():
    """🎯 늦은 실패가 와도 재개 원점은 **SEARCH_BACK** 이어야 한다."""
    env = T.make_env()
    _drive_to_search_back(env)
    env.stop_ok[0] = False              # 취소가 종결을 못 낸다
    seen = float(T.load_wp()['search_back']['seen_sec'])

    T.run(env, seen + 0.6, PERSON)      # tick A — 취소만 나간다
    assert env.node.refind_stopping, '전제 실패 — 취소 대기 단계에 안 들어갔다'
    assert env.node.state == State.SEARCH_BACK, \
        f'취소와 동시에 상태가 바뀌었다: {env.node.state}'

    _late_stop_failure(env)             # 늦은 실패 도착

    assert env.node.state == State.FAULT, f'실패인데 FAULT 가 아니다: {env.node.state}'
    assert env.node.resume_state == State.SEARCH_BACK, (
        '🔴 §91(3회차) P0-1 재발생 — 재개 원점이 '
        f'{env.node.resume_state} 다. GUIDE 면 정지 미확인 goal 을 안은 채 '
        '자동재개한다.')


def test_h_p0_1_no_new_goal_while_the_refind_cancel_is_unresolved():
    """🎯 종결 전에는 **신규 goal 0**. 사람이 다시 안 보여도 역행을 새로 안 낸다.

    이 가지가 이 보완의 핵심이다 — 대기 판정을 '가시성' 이 아니라 '상태' 로
    갈랐기 때문에, 취소 뒤 사람이 사라져도 새 goal 이 안 나간다.
    """
    env = T.make_env()
    _drive_to_search_back(env)
    env.stop_ok[0] = False
    seen = float(T.load_wp()['search_back']['seen_sec'])

    T.run(env, seen + 0.6, PERSON)
    assert env.node.refind_stopping
    goals_before = len(env.goals)

    T.run(env, 6.0, EMPTY)              # 다시 안 보인다 — 구판이면 새 역행 goal

    assert len(env.goals) == goals_before, (
        f'🔴 정지 미확인 상태에서 신규 goal 이 나갔다 '
        f'({goals_before} → {len(env.goals)})')
    assert env.node.state == State.SEARCH_BACK, \
        f'대기 중에 상태가 바뀌었다: {env.node.state}'


def test_h_p0_1_unconfirmed_stop_never_becomes_guide_on_its_own():
    """🔴 'unconfirmed' 로 굳으면 **자동 복귀하지 않는다** — 사람이 E-stop 할 자리다."""
    env = T.make_env()
    _drive_to_search_back(env)
    env.stop_ok[0] = False
    seen = float(T.load_wp()['search_back']['seen_sec'])

    T.run(env, seen + 0.6, PERSON)
    env.node.on_safety_stop_unconfirmed('하네스: 종결 실패')
    env.logs.clear()

    T.run(env, 10.0, PERSON)            # 계속 보여도

    assert env.node.state != State.GUIDE, (
        '🔴 정지 미확인인데 GUIDE 로 갔다 — 로봇이 아직 달리고 있을 수 있다')
    assert any('E-stop' in m for m, _ in env.logs), \
        '정지 미확인인데 사람에게 E-stop 을 요구하지 않았다'
