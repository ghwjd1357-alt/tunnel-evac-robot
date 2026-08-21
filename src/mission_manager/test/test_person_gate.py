#!/usr/bin/env python3
"""사람 판정 게이트(`RESCUE`·`NO_VICTIM`) 회귀 — 2026-08-22 신설.

정본 = `PROJECT_CONTEXT §4.1-b`. 어댑터 쪽 회귀 = `perception_adapter/test/test_person_path.py`.

🔴 **구판에는 이 분기가 없었다.** `GATHER → GUIDE` 가 `gather_wait_sec` 타이머만
보고 무조건 넘어갔다 — 즉 **집결지에 아무도 없어도 12초 뒤 유도를 시작했다.**
아무도 없는 복도를 앞장서서 걸어 나가는 것이다.

🔴 **그리고 이 게이트는 기본이 꺼짐이어야 한다.** 본편 테이크는 `camera:=false` 라
어댑터가 없고 `/person_status` 가 한 건도 안 온다. 켜 두면 그 침묵이 'stale' 로
굳어 **사람이 멀쩡히 서 있는데도 RESCUE 로 빠진다.**
"""
import test_search_back_entry as T
from mission_manager.mission_node import (PERSON_GATE_DEFAULT, State)

EMPTY = T.EMPTY
PERSON = T.PERSON


def gathered(gate=True):
    env = T.make_env(state=State.GATHER)
    env.node.wp['person_gate'] = gate
    return env


def say(env, status):
    """어댑터가 상태를 한 건 보냈다고 흉내낸다 (수신 시각도 같이 찍힌다)."""
    env.node.on_person_status(type('M', (), {'data': status})())


def run_live(env, seconds, scan, status, dt=0.1):
    """어댑터가 **살아서 10 Hz 로 쏘는 동안** 시간을 진행한다.

    🔴 처음엔 `say()` 를 한 번만 부르고 `T.run` 을 돌렸는데, 미션의 신선도 가드가
    1초 뒤 그것을 'stale' 로 떨어뜨려 모든 시험이 엉뚱한 곳으로 갔다.
    **가드가 제 일을 한 것이고 시험이 실물과 달랐다** — 실제 어댑터는 10 Hz 로
    계속 쏜다. 어댑터가 죽는 경우는 `run_dead` 로 따로 만든다.
    """
    for _ in range(round(seconds / dt)):
        env.clock.advance(dt)
        say(env, status)
        env.node.monitor.update(scan)
        env.step += 1
        if env.step % 5 == 0:
            env.node.tick()


def run_dead(env, seconds, scan, dt=0.1):
    """어댑터가 **죽어서 한 건도 안 오는** 동안 시간을 진행한다."""
    T.run(env, seconds, scan, dt)


# ── 🔴 기본이 꺼짐이어야 한다 ──────────────────────────────────────────
def test_g1_the_gate_is_off_by_default():
    """🔴 부정 회귀 — 기본이 켜지면 본편 테이크가 통째로 깨진다."""
    assert PERSON_GATE_DEFAULT is False
    env = T.make_env(state=State.GATHER)          # wp 를 안 건드린다
    assert env.node.person_gate_on() is False, \
        '🔴 waypoints 기본값이 게이트를 켜고 있다 — 본편에 어댑터가 없다'


def test_g2_with_the_gate_off_it_behaves_exactly_like_before():
    """게이트가 꺼져 있으면 사람 상태와 무관하게 구판 경로로 간다."""
    env = gathered(gate=False)
    run_live(env, 14.0, PERSON, 'fallen')         # 켜져 있었다면 RESCUE 였을 것
    assert env.node.state == State.GUIDE, \
        '게이트가 꺼졌는데 판정이 개입했다'


# ── 분기 ───────────────────────────────────────────────────────────────
def test_g3_standing_person_leads_to_guide():
    env = gathered()
    run_live(env, 14.0, PERSON, 'ok')
    assert env.node.state == State.GUIDE


def test_g4_fallen_person_leads_to_rescue_not_guide():
    """🎯 이 경로의 존재 이유 — 쓰러진 사람을 두고 나가지 않는다."""
    env = gathered()
    run_live(env, 14.0, PERSON, 'fallen')
    assert env.node.state == State.RESCUE, \
        '🔴 쓰러진 사람을 두고 유도를 시작했다'


def test_g5_nobody_there_leads_to_no_victim():
    """🔴 구판은 여기서 빈 복도를 유도했다."""
    env = gathered()
    run_live(env, 14.0, PERSON, 'none')
    assert env.node.state == State.NO_VICTIM


def test_g6_unknown_waits_instead_of_guessing():
    """판정 보류는 **정지**다 — 모르는 채로 떠나지 않는다."""
    env = gathered()
    run_live(env, 14.0, PERSON, 'unknown')
    assert env.node.state == State.GATHER, \
        '판정이 안 섰는데 어디론가 갔다'


def test_g7_waiting_forever_is_not_an_answer_either():
    """🔴 보류가 무한이면 로봇이 영원히 선다 — timeout 뒤에는 **신고**로 간다."""
    env = gathered()
    env.node.wp['person_decide_timeout_sec'] = 3.0
    run_live(env, 20.0, PERSON, 'unknown')
    assert env.node.state == State.RESCUE, \
        '모르는 채로 timeout 이 지났는데 신고하지 않았다'


# ── 🔴 미션 자신의 신선도 가드 ─────────────────────────────────────────
def test_g8_a_dead_adapter_is_not_a_standing_person():
    """🔴 부정 회귀 — 어댑터가 죽으면 마지막 `ok` 를 믿으면 안 된다.

    어댑터는 **자기 입력**이 끊기면 'stale' 을 말해 준다. 그런데 어댑터 **자체가**
    죽으면 아무 말도 못 한다. 그때 마지막 'ok' 를 붙들면 미션은 사람이 있는 줄 알고
    유도를 시작한다 — 아무도 없는 복도를.
    """
    env = gathered()
    env.node.wp['person_status_timeout_sec'] = 0.5
    env.node.wp['person_decide_timeout_sec'] = 3.0
    say(env, 'ok')                                # 마지막으로 한 건 왔고
    run_dead(env, 20.0, EMPTY)                    # 그 뒤로 어댑터가 죽었다
    assert env.node.fresh_person_status() == 'stale'
    assert env.node.state == State.RESCUE, \
        '🔴 어댑터가 죽었는데 마지막 ok 로 유도를 시작했다'


def test_g9_status_starts_as_stale_not_none():
    """기동 직후는 '사람 없음'이 아니다 — 아직 아무 말도 못 들었다."""
    env = T.make_env(state=State.GATHER)
    assert env.node.person_status == 'stale'
    assert env.node.fresh_person_status() == 'stale'


# ── 정지 상태의 성질 ───────────────────────────────────────────────────
def test_g10_rescue_and_no_victim_issue_no_goal():
    """🔴 새 goal 을 안 내는 것이 '정지'의 구현이다 (BLOCKED 과 같은 구조)."""
    for status, want in (('fallen', State.RESCUE), ('none', State.NO_VICTIM)):
        env = gathered()
        run_live(env, 14.0, PERSON, status)
        assert env.node.state == want
        before = len(env.goals)
        run_live(env, 5.0, PERSON, status)
        assert len(env.goals) == before, \
            f'{want.name} 인데 goal 이 {len(env.goals) - before}건 나갔다'


def test_g11_reset_puts_the_verdict_back_to_stale():
    """🔴 부정 회귀 — `reset` 뒤에 판정이 남아 있으면 다음 임무가 오염된다.

    'none' 이 남으면 다음 임무 첫 판정이 곧바로 NO_VICTIM 이고,
    'ok' 가 남으면 사람이 없어도 유도가 시작된다.
    """
    env = gathered()
    run_live(env, 14.0, PERSON, 'none')
    assert env.node.state == State.NO_VICTIM
    env.node.on_cmd(type('M', (), {'data': 'reset'})())
    assert env.node.state == State.PATROL
    assert env.node.person_status == 'stale'
    assert env.node.person_status_t is None
    assert env.node.victim is None


# ── 🆕 제자리 360° 훑기 (SCAN_AREA) ────────────────────────────────────
def test_g12_normalize_angle_exists_and_folds():
    """🔴 08-22 — 이 함수를 **정의하지 않고 호출했었다.**

    문법도 통과했고 회귀 220 도 초록이었다 — 시험이 `SCAN_AREA` 경로를 한 번도
    안 탔기 때문이다. 실차에서는 훑기 첫 스텝에서 `NameError` 로 죽고 테이크가
    통째로 날아간다. **회귀가 없는 코드는 문법이 맞아도 안 돈다.**
    """
    import math

    from mission_manager.mission_node import normalize_angle
    # ⚠ ±π 는 같은 방향이다. `atan2` 는 `sin(3π)` 가 아주 작은 **음수**라
    #   −π 를 돌려준다 — 부호가 아니라 **크기**로 본다.
    assert abs(abs(normalize_angle(3.0 * math.pi)) - math.pi) < 1e-9
    assert abs(abs(normalize_angle(-3.0 * math.pi)) - math.pi) < 1e-9
    assert abs(normalize_angle(0.5) - 0.5) < 1e-12
    assert abs(normalize_angle(0.5 + 2.0 * math.pi) - 0.5) < 1e-9
    # 접힌 값은 항상 [-π, π] 안이다
    for a in (-20.0, -7.0, -0.1, 0.0, 3.2, 9.9, 100.0):
        assert -math.pi - 1e-9 <= normalize_angle(a) <= math.pi + 1e-9


def test_g13_scan_runs_only_when_the_gate_is_on():
    """🔴 게이트가 꺼진 본편은 훑지 않는다 — 구판처럼 곧바로 GATHER 다."""
    env = T.make_env(state=State.APPROACH)
    env.node.wp['person_gate'] = False
    env.node.on_reached()
    assert env.node.state == State.GATHER

    env = T.make_env(state=State.APPROACH)
    env.node.wp['person_gate'] = True
    env.node.on_reached()
    assert env.node.state == State.SCAN_AREA


def test_g14_scan_goals_keep_the_position_and_only_turn():
    """🔴 위치가 움직이면 '제자리 훑기'가 아니고, 화재 배제거리 전제도 흔들린다."""
    import math

    env = T.make_env(state=State.APPROACH)
    env.node.wp['person_gate'] = True
    env.node.on_reached()
    base = env.node.wp['gather']
    seen = []
    for i in range(env.node.scan_steps()):
        env.node.scan_idx = i
        g = env.node.scan_goal()
        assert abs(g['x'] - base['x']) < 1e-9
        assert abs(g['y'] - base['y']) < 1e-9
        seen.append(g['yaw'])
    # 네 스텝이 서로 다른 방향이어야 한 바퀴가 된다
    assert len({round(y, 3) for y in seen}) == env.node.scan_steps()
    assert all(-math.pi - 1e-9 <= y <= math.pi + 1e-9 for y in seen)


def test_g15_a_fallen_person_ends_the_scan_immediately():
    """🔵 쓰러진 사람이 확정되면 남은 스텝을 마저 돌 이유가 없다."""
    env = T.make_env(state=State.APPROACH)
    env.node.wp['person_gate'] = True
    env.node.on_reached()
    assert env.node.state == State.SCAN_AREA
    run_live(env, 2.0, PERSON, 'fallen')
    assert env.node.state == State.RESCUE, '훑는 중 확정인데 계속 돌았다'


def test_g16_a_full_sweep_hands_over_to_gather():
    """한 바퀴를 다 돌면 판정은 GATHER 가 한다 (분기를 두 곳에 두지 않는다)."""
    env = T.make_env(state=State.APPROACH)
    env.node.wp['person_gate'] = True
    env.node.wp['scan_dwell_sec'] = 0.5
    env.node.on_reached()
    for _ in range(env.node.scan_steps()):
        env.node.on_reached()                 # 그 스텝 goal 도착
        run_live(env, 1.0, PERSON, 'unknown')
    assert env.node.state == State.GATHER, '한 바퀴를 다 돌고도 넘어가지 않았다'


# ── 🆕 예약 61 — 역행 중 카메라 병행 ───────────────────────────────────
def sb_env(camera=False):
    env = T.make_env(state=State.SEARCH_BACK)
    env.node.wp['search_back']['camera_refind'] = camera
    return env


def test_g17_camera_refind_is_off_by_default():
    """🔴 예약 61 이 정한 대로 **기본 꺼짐**이다.

    카메라의 3~4 m·저조도 검출은 아직 미검증이다(역할 B 08-18 G5 표: 원거리 ❌).
    검증 안 된 신호로 라이다 판정을 흔들면, 잘 되던 것까지 같이 망가진다.
    """
    env = T.make_env(state=State.SEARCH_BACK)
    assert env.node.wp['search_back'].get('camera_refind', False) is False
    say(env, 'ok')
    assert env.node.camera_refind_status() is None


def test_g18_camera_alone_can_refind_when_enabled():
    """켜면 라이다가 못 봐도 카메라만으로 복귀한다 — OR 이지 AND 가 아니다."""
    env = sb_env(camera=True)
    run_live(env, 2.0, EMPTY, 'ok')               # 라이다에는 아무것도 없다
    assert env.node.state == State.GUIDE


def test_g19_lidar_alone_still_works_with_the_camera_on():
    """🔴 부정 회귀 — 카메라를 켜도 라이다 경로가 살아 있어야 한다.

    OR 을 AND 로 잘못 쓰면 카메라가 없을 때 재발견이 **아예 안 된다** —
    잘 되던 것이 조용히 죽는다.
    """
    env = sb_env(camera=True)
    run_live(env, 2.0, PERSON, 'stale')           # 카메라는 죽었고 라이다만 본다
    assert env.node.state == State.GUIDE


def test_g20_a_fallen_person_seen_while_reversing_is_not_passed_by():
    """🔴 역행 중에 쓰러진 사람이 보이면 유도로 복귀하면 안 된다 — 두고 나가는 것이다."""
    env = sb_env(camera=True)
    run_live(env, 2.0, EMPTY, 'fallen')
    assert env.node.state == State.RESCUE
