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
def ros():
    """노드를 만들지 않는 시험용 — 기동 실패 경로를 보는 자리."""
    rclpy.init()
    yield
    rclpy.shutdown()


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


def test_rearm_needs_a_new_observation_after_the_request(node):
    """🔴 §85.4 — 요청 시점 캐시만으로는 재무장하지 않는다 (handshake)."""
    state(node, 'PATROL')
    node.fired_at = 123.0
    rearm(node)
    assert node.fired_at == 123.0, '캐시만 보고 재무장했다'
    assert any('REARM_PENDING' in s for s in node._said), node._said
    state(node, 'PATROL')                      # 요청 **뒤** 새 관측
    assert node.fired_at is None
    assert 'REARMED' in node._said, node._said


def test_rearm_rejects_when_state_never_seen(node):
    """상태 미수신도 거부다 — fail-closed."""
    node.fired_at = 123.0
    rearm(node)
    assert node.fired_at == 123.0
    assert any('REARM_REJECTED' in s for s in node._said)


def test_rearm_rejects_when_state_changed_right_after_the_request(node):
    """🔴 §85.4 재현본 — 요청 0.1초 전 PATROL, 실제로는 APPROACH.

    §84.3 은 캐시에 수신시각만 붙여서 이 창을 못 막았다 — age 도 1.5초 이하이고
    문자열도 exact PATROL 이라 그대로 REARMED 가 났다."""
    state(node, 'PATROL')
    node.fired_at = 123.0
    rearm(node)                                 # 캐시는 PATROL
    state(node, 'APPROACH')                     # 그런데 지금은 APPROACH
    assert node.fired_at == 123.0, '요청 뒤 APPROACH 인데 재무장됐다'
    assert any('요청 뒤 APPROACH' in s for s in node._said), node._said


def test_rearm_request_expires_without_a_new_observation(node):
    """새 관측이 안 오면 요청을 버린다 — REARMED 를 영원히 기다리지 않게."""
    state(node, 'PATROL')
    node.fired_at = 123.0
    rearm(node)
    node._rearm_pending_since = node.now_sec() - 99.0
    node.tick()
    assert node.fired_at == 123.0
    assert any('만료' in s for s in node._said), node._said


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
    ('rearm_ack_timeout_sec', float('inf')), ('rearm_ack_timeout_sec', 0.0),
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


# ── §85.5 패턴 수정: 목록 대조를 기계가 한다 ────────────────────────────

def test_no_declared_numeric_param_escapes_validation(node):
    """🔴 §85.5 — 선언된 수치 파라미터와 검증 목록의 차집합이 0 이어야 한다.

    §84.4("모든 수치를 검증한다")를 넣은 **바로 그 커밋**에서 새 수치를 목록에
    안 넣었다. 손으로 관리하는 목록은 다음 파라미터에서 또 샌다 —
    그래서 기동이 이 차집합을 강제한다."""
    assert node.unvalidated_numeric_params() == set()


def test_startup_refuses_when_a_numeric_param_is_unlisted(node, monkeypatch):
    """목록에서 하나를 빼면 **기동이 막혀야** 한다 (검사의 검사)."""
    from perception_adapter.adapter_node import PerceptionAdapter as PA
    short = tuple(x for x in PA.RUNTIME_NUMERIC if x != 'max_range')
    monkeypatch.setattr(PA, 'RUNTIME_NUMERIC', short)
    with pytest.raises(ValueError, match='검증 목록 밖'):
        PA()


# ── §86.3 기동 실패는 자원을 만들기 전에 ────────────────────────────────

def test_bad_override_raises_before_creating_the_aux_node(ros):
    """🔴 재현본 — 검증이 보조 노드·스레드 **뒤**면 프로세스가 안 죽는다.

    구판은 `_tf_node` 와 non-daemon 리스너 스레드를 만든 뒤 검증해서,
    `max_range=-1` 기동이 `ValueError` 를 내고도 **자발 종료하지 않았다**
    (실측: timeout 이 SIGTERM 으로 죽였다). `/adapter_status` 없는 반쪽 노드가
    남아 node-list 준비 판정을 오도한다 — 08-20 무증상 실패와 같은 형태다."""
    import threading
    before = threading.active_count()
    with pytest.raises(ValueError):
        PerceptionAdapter(parameter_overrides=[Parameter('max_range', value=-1.0)])
    assert threading.active_count() == before, '거부됐는데 스레드가 남았다'


def test_unlisted_numeric_also_raises_before_resources(ros, monkeypatch):
    """목록 누락도 같은 자리에서 막혀야 한다 (자원 생성 전)."""
    import threading
    from perception_adapter.adapter_node import PerceptionAdapter as PA
    short = tuple(x for x in PA.RUNTIME_NUMERIC if x != 'max_range')
    monkeypatch.setattr(PA, 'RUNTIME_NUMERIC', short)
    before = threading.active_count()
    with pytest.raises(ValueError, match='검증 목록 밖'):
        PA()
    assert threading.active_count() == before


# ── §86.4 전달용 중복은 하나의 의도다 ──────────────────────────────────

def test_triple_send_produces_exactly_one_rearm(node):
    """🔴 재현본 — 런북이 `-w 1 --times 3` 을 시킨다. 셋은 **한 의도**다.

    구판은 셋을 서로 다른 요청으로 봐서 `REARMED` 가 3번, tracker reset 도 3회였다."""
    state(node, 'PATROL')
    for _ in range(3):
        rearm(node)
        state(node, 'PATROL')
    assert node._said.count('REARMED') == 1, node._said
    assert node._said.count('REARM_DUP_IGNORED') == 2, node._said


def test_late_duplicate_does_not_wipe_accumulated_hits(node):
    """🔴 가장 나쁜 경로 — 장면이 시작된 뒤 3발째가 늦게 도착한다.

    구판에서는 누적 관측이 2 → 0 으로 지워졌다. 진짜 화재를 보고 있던 증거가
    사라진다는 뜻이다."""
    state(node, 'PATROL')
    rearm(node)
    state(node, 'PATROL')                       # 정상 재무장
    node.tracker.add(1.0, 1.0, (2.0, 0.0, 0.0))
    node.tracker.add(1.1, 2.0, (2.0, 0.0, 0.0))
    assert node.tracker.count() == 2
    rearm(node)                                 # 늦게 도착한 중복
    state(node, 'PATROL')
    assert node.tracker.count() == 2, '늦은 중복이 누적을 지웠다'


def test_a_genuine_next_take_still_rearms_after_the_dedup_window(node):
    """🟢 진짜 다음 테이크는 재무장돼야 한다 — 병합이 영구가 아니다."""
    state(node, 'PATROL')
    rearm(node)
    state(node, 'PATROL')
    assert node._said.count('REARMED') == 1
    node._last_rearm_t = node.now_sec() - 99.0   # 창 밖
    node.fired_at = 123.0
    rearm(node)
    state(node, 'PATROL')
    assert node._said.count('REARMED') == 2, node._said
    assert node.fired_at is None


def test_duplicate_while_pending_does_not_open_a_second_handshake(node):
    """요청이 열려 있는 동안의 중복도 새 handshake 를 만들지 않는다."""
    state(node, 'PATROL')
    rearm(node)
    rearm(node)
    rearm(node)
    assert node._said.count('REARM_PENDING (다음 PATROL 관측 대기)') == 1, node._said
