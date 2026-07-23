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
