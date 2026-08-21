# -*- coding: utf-8 -*-
"""어댑터 **노드 계층** 검사 — 재무장 관문과 런타임 파라미터 (08-21 §84.3·§84.4).

순수 함수는 `test_adapter_pure.py` 가 본다. 여기서는 실제 노드를 띄워
`set_parameters` 와 `/adapter_cmd` 경로를 시험한다 — 그 둘은 순수 함수로
못 뽑는 자리이고, 정확히 거기서 §84 가 구멍을 찾았다.
"""

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('tunnel_interfaces')

import rclpy                                            # noqa: E402
from rclpy.parameter import Parameter                   # noqa: E402
from std_msgs.msg import String                         # noqa: E402

from perception_adapter.adapter_node import PerceptionAdapter   # noqa: E402


@pytest.fixture
def node():
    rclpy.init()
    n = PerceptionAdapter()
    n._said = []
    n.say = lambda t: n._said.append(t)
    yield n
    n.destroy_node()
    rclpy.shutdown()


def rearm(n):
    m = String()
    m.data = 'rearm'
    n.on_cmd(m)


def state(n, s):
    m = String()
    m.data = s
    n.on_mission_state(m)


# ── §84.3 재무장 관문 ──────────────────────────────────────────────────

@pytest.mark.parametrize('bad', ['NOT_PATROL', 'PATROLLING', 'BLOCKED PATROL',
                                 'APPROACH', 'GUIDE', 'BLOCKED', ''])
def test_rearm_rejects_anything_that_is_not_exactly_patrol(node, bad):
    """🔴 재현본 — 구판 `'PATROL' not in st` 는 앞 셋을 전부 통과시켰다."""
    state(node, bad)
    node.fired_at = 123.0
    rearm(node)
    assert node.fired_at == 123.0, f'"{bad}" 로 재무장됐다'
    assert any('REARM_REJECTED' in s for s in node._said), node._said


def test_rearm_accepts_exact_patrol(node):
    state(node, 'PATROL')
    node.fired_at = 123.0
    rearm(node)
    assert node.fired_at is None
    assert 'REARMED' in node._said, node._said


def test_rearm_rejects_when_state_never_seen(node):
    """상태 미수신도 거부다 — fail-closed."""
    node.fired_at = 123.0
    rearm(node)
    assert node.fired_at == 123.0
    assert any('REARM_REJECTED' in s for s in node._said)


def test_rearm_rejects_stale_state(node):
    """🔴 §84.3 — 마지막 관측이 오래됐으면 그건 현재가 아니다."""
    state(node, 'PATROL')
    node._mission_state_t = node.now_sec() - 5.0      # 5초 전 값
    node.fired_at = 123.0
    rearm(node)
    assert node.fired_at == 123.0, '과거 PATROL 로 재무장됐다'
    assert any('낡음' in s for s in node._said), node._said


def test_rearm_opt_out_still_works_for_bench_testing(node):
    """미션 없이 어댑터만 시험하는 경로는 남겨 둔다 (명시적으로 꺼야 한다)."""
    node.set_parameters([Parameter('rearm_requires_patrol', value=False)])
    node.fired_at = 123.0
    rearm(node)
    assert node.fired_at is None


# ── §84.4 런타임 파라미터 ──────────────────────────────────────────────

@pytest.mark.parametrize('name,val', [
    ('max_range', -1.0), ('max_range', 0.0), ('max_range', float('nan')),
    ('confirm_assoc_radius_m', 1000.0), ('confirm_assoc_radius_m', -1.0),
    ('tf_wait_sec', 5.0), ('tf_wait_sec', -0.1),
    ('confirm_frames', 1), ('confirm_frames', 0),
    ('min_confidence', 1.5), ('min_confidence', -0.1),
    ('confirm_window_sec', 0.0), ('fixed_range', -2.0),
    ('max_stamp_age_sec', 0.0), ('refire_cooldown_sec', float('inf')),
])
def test_runtime_set_rejects_bad_values(node, name, val):
    """🔴 재현본 — 구판은 이 전부를 successful=True 로 저장했다."""
    before = node.get_parameter(name).value
    r = node.set_parameters([Parameter(name, value=val)])[0]
    assert not r.successful, f'{name}={val!r} 이 통과했다'
    assert node.get_parameter(name).value == before


def test_runtime_set_rebuilds_the_tracker(node):
    """🔴 구판은 tracker 가 초기 객체라 confirm_frames 변경이 전혀 안 먹었다."""
    assert node.tracker.need == 5
    r = node.set_parameters([Parameter('confirm_frames', value=7)])[0]
    assert r.successful
    assert node.tracker.need == 7


def test_runtime_set_clears_accumulated_hits(node):
    """확정 정책이 바뀌면 옛 정책으로 모은 근거를 이어 쓰지 않는다."""
    node.tracker.add(1.0, 1.0, (2.0, 0.0, 0.0))
    assert node.tracker.count() == 1
    node.set_parameters([Parameter('confirm_window_sec', value=5.0)])
    assert node.tracker.count() == 0


def test_runtime_set_accepts_a_valid_combination(node):
    r = node.set_parameters([
        Parameter('max_range', value=4.0),
        Parameter('confirm_assoc_radius_m', value=1.5),
    ])
    assert all(x.successful for x in r)
    assert node.get_parameter('max_range').value == 4.0


def test_runtime_set_is_atomic_across_the_batch(node):
    """한 건이 불량이면 그 건이 거부된다 — 나머지가 조용히 섞이지 않는다."""
    before = node.get_parameter('max_range').value
    r = node.set_parameters([Parameter('max_range', value=-5.0)])[0]
    assert not r.successful
    assert node.get_parameter('max_range').value == before
