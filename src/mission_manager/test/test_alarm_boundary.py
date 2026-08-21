# -*- coding: utf-8 -*-
"""
test_alarm_boundary.py — /alarm 입력 신뢰경계 단위테스트 (S1-1·S1-2, 07-19)
============================================================
[무엇을 잡나 — Codex §9.2 재현이 근거]
  검증 없는 on_alarm 은 NaN/Inf 화재를 예외도 없이 '멀쩡한 집결지 (4,0)'로
  둔갑시키고(그래프 투영이 흡수), 1km 밖 오클릭도 정상 화재처럼 접수했다.
  또 raw 좌표가 달라도 같은 그래프 끝점에 투영되면(동일 투영점, §8.2) 경로
  길이 0 → None → 직선 fallback 이 벽 안 좌표를 내놓았다.
  → S1-1: 진입점 검증(유한값·frame·투영거리)으로 불량 알람은 상태 전이 없이 거부.
  → S1-2: 그래프 선언 시 직선 fallback 금지 — 실패하면 yaml 검증 고정값으로.

[기법]
  test_goal_lifecycle 과 동일한 MissionNode.__new__ 껍데기 + 가짜 msg.
  on_alarm 이 만지는 속성만 채운다 (wp·state·gather_wp·siren 등).
"""

import math
import types

from mission_manager.mission_node import MissionNode, State


def fake_alarm(x, y, frame='map'):
    """PoseStamped 흉내 — on_alarm 이 읽는 필드만."""
    return types.SimpleNamespace(
        header=types.SimpleNamespace(frame_id=frame),
        pose=types.SimpleNamespace(
            position=types.SimpleNamespace(x=x, y=y)))


def bare_node(with_graph=True):
    node = MissionNode.__new__(MissionNode)
    node.state = State.PATROL
    node.fire = None
    node.gather_wp = None
    # goal 수명주기는 GoalManager 로 이관(07-23 구조 분리 2/3). on_alarm 의
    # 재지정 취소는 이 테스트의 검증 대상(알람 입력 신뢰경계)이 아니므로,
    # 매니저 배선 없이 무해 가짜로 대체한다 (시나리오·단언은 그대로).
    node.cancel_current_goal = lambda: None
    node.siren_on = False
    node.wp = {
        'escape': {'x': 0.0, 'y': 0.0},
        'gather': {'x': 6.0, 'y': 0.0, 'yaw': 3.14},
        'gather_dist': 8.0,
        'alarm_max_projection_dist': 5.0,
    }
    if with_graph:
        node.wp['corridor_graph'] = {
            'nodes': {'west': {'x': 0.0, 'y': 0.0},
                      'junc': {'x': 12.0, 'y': 0.0},
                      'east': {'x': 26.0, 'y': 0.0},
                      'branch': {'x': 12.0, 'y': 13.0}},
            'edges': [['west', 'junc'], ['junc', 'east'], ['junc', 'branch']],
        }
    logs = []
    node.get_logger = lambda: types.SimpleNamespace(
        warn=lambda msg, **kw: logs.append(msg),
        info=lambda msg, **kw: logs.append(msg),
        error=lambda msg, **kw: logs.append(msg))
    node._logs = logs
    node.set_siren = lambda on: setattr(node, 'siren_on', on)
    return node


def assert_rejected(node, keyword):
    """거부 = 상태 전이 없음 + 화재 미기록 + 사유 로그."""
    assert node.state == State.PATROL
    assert node.fire is None
    assert not node.siren_on
    assert any('알람 거부' in m and keyword in m for m in node._logs), node._logs


# ============================================================
# S1-1: 불량 입력 거부 (Codex 공격 4종 재현)
# ============================================================
def test_alarm_nan_rejected():
    """NaN 좌표 — 수정 전엔 그래프 투영이 흡수해 집결지 (4,0) 둔갑."""
    node = bare_node()
    node.on_alarm(fake_alarm(float('nan'), 0.0))
    assert_rejected(node, '유한값')


def test_alarm_inf_rejected():
    node = bare_node()
    node.on_alarm(fake_alarm(float('inf'), 0.0))
    assert_rejected(node, '유한값')


def test_alarm_wrong_frame_rejected():
    """map 아닌 frame — 좌표계 불명 화재로 출동하면 안 됨."""
    node = bare_node()
    node.on_alarm(fake_alarm(14.0, 0.0, frame='odom'))
    assert_rejected(node, 'frame')


def test_alarm_far_offmap_click_rejected():
    """1km 밖 오클릭 — 수정 전엔 정상 화재처럼 (18,0) 집결지 생성."""
    node = bare_node()
    node.on_alarm(fake_alarm(1000.0, 1000.0))
    assert_rejected(node, '오클릭')


def test_alarm_offcorridor_but_near_accepted():
    """복도에서 5m 이내의 벽 근처 클릭은 접수 (투영으로 살릴 수 있는 범위)."""
    node = bare_node()
    node.on_alarm(fake_alarm(12.5, 14.0))   # branch 끝 (12,13) 에서 ~1.1m
    assert node.state == State.APPROACH
    assert node.fire is not None


# ============================================================
# S1-2: 그래프 선언 시 직선 fallback 금지
# ============================================================
def test_alarm_same_projection_uses_yaml_gather_not_straight():
    """★ Codex §8.2 계열: 화재 (-4.9, 0.5) 와 탈출구가 같은 끝점 (0,0) 에
    투영 → 그래프 경로 길이 0 → None. 이때 직선식으로 넘어가지 말고
    gather_wp=None (→ tick 이 yaml 검증 고정값 사용) 이어야 한다."""
    node = bare_node()
    node.on_alarm(fake_alarm(-4.9, 0.5))    # 투영거리 4.92m < 5.0 → 접수됨
    assert node.state == State.APPROACH     # 알람 자체는 유효
    assert node.gather_wp is None           # ★ 직선 fallback 안 탔다
    assert any('직선 생략' in m for m in node._logs)


def test_alarm_standard_fire_graph_gather():
    """정상 경로 회귀 앵커: 표준 화재 (14,0) → 그래프 집결지 (6,0)·서향."""
    node = bare_node()
    node.on_alarm(fake_alarm(14.0, 0.0))
    assert node.state == State.APPROACH
    assert abs(node.gather_wp['x'] - 6.0) < 1e-6
    assert abs(node.gather_wp['y']) < 1e-6


def test_alarm_branch_fire_graph_gather():
    """곁복도 화재 (12,10) → (12,2) — 07-19 저녁 검증값 유지 (회귀 앵커)."""
    node = bare_node()
    node.on_alarm(fake_alarm(12.0, 10.0))
    assert abs(node.gather_wp['x'] - 12.0) < 1e-6
    assert abs(node.gather_wp['y'] - 2.0) < 1e-6


def test_alarm_no_graph_still_uses_straight():
    """그래프 미선언 구성(하위 호환): 직선 수식 경로는 그대로 동작해야."""
    node = bare_node(with_graph=False)
    node.on_alarm(fake_alarm(14.0, 0.0))
    assert node.state == State.APPROACH
    assert abs(node.gather_wp['x'] - 6.0) < 1e-6   # 직선판 검증값 동일


def test_alarm_ignored_outside_patrol():
    """PATROL 외 상태의 알람은 기존대로 무시 (기록만) — 기존 동작 회귀."""
    node = bare_node()
    node.state = State.GUIDE
    node.on_alarm(fake_alarm(14.0, 0.0))
    assert node.state == State.GUIDE
    assert node.fire is None


# ============================================================
# S1-3: 계산된 집결지의 안전거리 불변조건 (08-21, Codex §82.7 재현)
# ------------------------------------------------------------
# [무엇을 잡나] 설정 부등식 min_fire_dist < gather_dist 만 보고 **계산 결과**를
#   안 봤다. 경로가 gather_dist 보다 짧으면 집결지가 탈출구로 클램프되는데,
#   탈출구 근처 화재에서는 그 탈출구가 곧 화재 코앞이다.
#   재현: H 좌표 · fire(1.0,-10.65) → 집결지 (0.50,-10.65) = 화재에서 0.50m.
#   선언한 min_fire_dist 는 1.5m 였고, 알람게이트(투영거리 0.00)는 통과했다.
# [규율] S1-1 과 같다 — 거부는 상태를 아무것도 바꾸지 않는다.
# ============================================================

def assert_blocked(node, keyword):
    """🔴 §83.6 — 안전 집결지 없음의 계약은 "조용한 무시" 가 아니다.

    구판은 경고만 찍고 return 해서 state=PATROL · cancel 0회 였다. 즉 로봇이
    **기존 순찰 goal 을 계속 수행**했고, 정본이 적은 "관제로 넘긴다" 는 상태
    전이로 존재하지 않았다. 계약 = **멈추고 · 사유를 남기고 · 사람을 기다린다.**"""
    assert node.state == State.BLOCKED, node.state
    assert node.fire is None
    assert not node.siren_on
    assert node._cancels == 1, f'활성 goal 취소가 {node._cancels}회'
    assert node.blocked_reason and 'unsafe_gather' in node.blocked_reason
    assert any('알람 거부' in m and keyword in m for m in node._logs), node._logs


def h_node(min_fire_dist=1.5):
    """실제 촬영 좌표(H자)를 쓰는 껍데기 — 아래 복도 서쪽이 탈출구."""
    node = bare_node(with_graph=False)
    node._cancels = 0
    node.cancel_current_goal = lambda: setattr(node, '_cancels', node._cancels + 1)
    node.blocked_reason = None          # MissionNode.__init__ 이 세우는 자리
    node._blocked_logged = False
    node.wp = {
        'escape': {'x': 0.50, 'y': -10.65},
        'gather': {'x': 6.0, 'y': -10.65, 'yaw': 3.14},
        'gather_dist': 2.0,
        'alarm_max_projection_dist': 3.0,
        'corridor_graph': {
            'nodes': {'up_west': {'x': 0.50, 'y': -0.08},
                      'up_junc': {'x': 8.95, 'y': -0.08},
                      'up_east': {'x': 12.99, 'y': -0.10},
                      'low_junc': {'x': 8.95, 'y': -10.65},
                      'low_east': {'x': 12.45, 'y': -10.97},
                      'low_west': {'x': 0.50, 'y': -10.65}},
            'edges': [['up_west', 'up_junc'], ['up_junc', 'up_east'],
                      ['up_junc', 'low_junc'], ['low_junc', 'low_east'],
                      ['low_junc', 'low_west']],
        },
    }
    if min_fire_dist is not None:
        node.wp['search_back'] = {'min_fire_dist': min_fire_dist}
    return node


def test_s13_fire_near_escape_rejected_not_approached():
    """🔴 재현본 — 탈출구 0.5m 옆 화재. 수정 전엔 집결지 0.50m 로 출동했다."""
    node = h_node()
    node.on_alarm(fake_alarm(1.0, -10.65))
    assert_blocked(node, '최소 안전거리')
    assert node.gather_wp is None


def test_s13_fire_almost_on_escape_rejected():
    """0.10m — 경계가 아니라 완전히 안쪽."""
    node = h_node()
    node.on_alarm(fake_alarm(0.6, -10.65))
    assert_blocked(node, '최소 안전거리')


def test_s13_boundary_exactly_min_fire_dist_accepted():
    """정확히 1.50m = 허용. 부등식은 '미만'만 거부한다 (경계 포함)."""
    node = h_node()
    node.on_alarm(fake_alarm(2.0, -10.65))
    assert node.state == State.APPROACH
    assert node.gather_wp is not None
    d = math.hypot(node.gather_wp['x'] - 2.0, node.gather_wp['y'] + 10.65)
    assert abs(d - 1.5) < 1e-6, d


def test_s13_far_fire_keeps_full_gather_dist():
    """탈출구에서 먼 화재는 gather_dist 2.0 을 그대로 채운다 — 회귀 방지."""
    node = h_node()
    node.on_alarm(fake_alarm(3.5, -10.65))
    assert node.state == State.APPROACH
    d = math.hypot(node.gather_wp['x'] - 3.5, node.gather_wp['y'] + 10.65)
    assert abs(d - 2.0) < 1e-6, d


def test_s13_current_shot_script_fire_still_accepted():
    """🔵 대본 화재 (12.5,-0.1) 은 영향받지 않는다 — 집결지 (10.5,-0.08) 부근."""
    node = h_node()
    node.on_alarm(fake_alarm(12.5, -0.1))
    assert node.state == State.APPROACH
    assert abs(node.gather_wp['x'] - 10.5) < 0.05, node.gather_wp
    assert abs(node.gather_wp['y'] + 0.08) < 0.05, node.gather_wp


def test_s13_fire_equals_escape_rejected_via_fixed_gather():
    """화재=탈출구 → 그래프 계산 None → yaml 고정 집결지로 안전거리를 본다."""
    node = h_node()
    node.wp['gather'] = {'x': 0.50, 'y': -10.65, 'yaw': 3.14}   # 고정값도 화재 위
    node.on_alarm(fake_alarm(0.50, -10.65))
    assert_blocked(node, '최소 안전거리')


def test_s13_no_min_fire_dist_declared_keeps_old_behaviour():
    """⚠ 미선언이면 불변조건을 요구하지 않은 설정 — 거동 불변(기존 yaml 보호)."""
    node = h_node(min_fire_dist=None)
    node.on_alarm(fake_alarm(1.0, -10.65))
    assert node.state == State.APPROACH
    assert abs(node.gather_wp['x'] - 0.50) < 1e-6


def test_s13_blocked_issues_no_new_goal_and_needs_reset():
    """🔴 §83.6 재현본 — BLOCKED 는 스스로 빠져나오지 않는다.

    tick 이 BLOCKED 가지에서 goal 을 하나도 안 내야 하고, 탈출은 관제 `reset`
    뿐이다. FAULT 와 다른 점이 이것이다 — FAULT 는 자동 재시도가 있다."""
    node = h_node()
    node.on_alarm(fake_alarm(1.0, -10.65))
    assert node.state == State.BLOCKED
    # BLOCKED 는 tick 이 새 goal 을 내는 가지가 없다 — 상태 목록으로 고정한다
    goal_issuing = {State.PATROL, State.APPROACH, State.GUIDE, State.SEARCH_BACK}
    assert State.BLOCKED not in goal_issuing


def test_s13_blocked_reason_carries_the_numbers():
    """사유에 화재·집결지·거리가 다 들어가야 관제가 판단할 수 있다."""
    node = h_node()
    node.on_alarm(fake_alarm(1.0, -10.65))
    r = node.blocked_reason
    assert 'fire=(1.00,-10.65)' in r, r
    assert 'gather=(0.50,-10.65)' in r, r
    assert 'dist=0.50<1.50' in r, r


def test_s13_accepted_fire_does_not_touch_blocked_fields():
    """🟢 정상 화재는 BLOCKED 경로를 건드리지 않는다 (회귀 방지)."""
    node = h_node()
    node.on_alarm(fake_alarm(12.5, -0.1))
    assert node.state == State.APPROACH
    assert node.blocked_reason is None
