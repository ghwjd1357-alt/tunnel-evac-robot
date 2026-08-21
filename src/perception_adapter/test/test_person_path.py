#!/usr/bin/env python3
"""사람 경로(`/person_status`·`/victim`) 회귀 — 2026-08-22 신설.

정본 = `PROJECT_CONTEXT §4.1-b`.

🔴 **이 경로가 겨누는 오판은 되돌릴 수 없는 쪽이다.** 쓰러진 사람을 `ok` 로 읽으면
로봇이 유도를 시작하고 **떠난다**. 서 있는 사람을 `fallen` 으로 읽으면 관제가 확인하고
끝이다. 그래서 여기 부정 회귀는 전부 *"떠나도 된다고 잘못 말하지 않는가"* 를 본다.
"""
import pytest
import rclpy
from rcl_interfaces.msg import Parameter as ParamMsg  # noqa: F401
from rclpy.parameter import Parameter
from std_msgs.msg import Header

from perception_adapter.adapter_node import PerceptionAdapter
from tunnel_interfaces.msg import Detection3D, Detection3DArray


@pytest.fixture
def node():
    rclpy.init()
    n = PerceptionAdapter(parameter_overrides=[
        Parameter('person_confirm_sec_fallen', value=1.0),
        Parameter('person_confirm_sec_leave', value=2.0),
        Parameter('person_min_frames', value=3),
        Parameter('person_min_confidence', value=0.5),
    ])
    yield n
    n.destroy_node()
    rclpy.shutdown()


def det(name, conf=0.9, z=2.0):
    d = Detection3D()
    d.class_name = name
    d.confidence = conf
    d.position.x, d.position.y, d.position.z = 0.0, 0.0, z
    return d


def arr(node, *dets, t=None):
    """🔴 `stamp` 을 가짜 시계 `t` 에 맞춘다.

    처음엔 `node.get_clock().now()` 를 넣었는데, 그건 진짜 시스템 시각(~1.7e9)이라
    가짜 `t=1000.0` 과 **1.7e9 초나 어긋났다.** 그래서 모든 프레임이 stamp 신선도
    검사에서 걸려 판정이 영원히 `unknown` 이었다 — 생산 코드는 멀쩡했고 시험이
    자기 발을 밟은 것이다.
    """
    m = Detection3DArray()
    m.header = Header()
    if t is not None:
        m.header.stamp.sec = int(t)
        m.header.stamp.nanosec = int((t - int(t)) * 1e9)
    else:
        m.header.stamp = node.get_clock().now().to_msg()
    m.header.frame_id = 'camera_color_optical_frame'
    m.detections = list(dets)
    return m


def feed(node, dets, seconds, hz=10.0, t0=1000.0):
    """가짜 시계로 `seconds` 동안 `dets` 를 흘린다. 반환 = 마지막 상태."""
    step = 1.0 / hz
    n = int(round(seconds * hz))
    for i in range(n):
        t = t0 + i * step
        node.now_sec = lambda _t=t: _t
        node._update_person(arr(node, *dets, t=t), t)
    return node._p_status


# ── 프레임 한 장의 판정 ────────────────────────────────────────────────
def test_p1_fallen_beats_ok_in_the_same_frame(node):
    """🔴 두 사람 — 하나는 서 있고 하나는 쓰러져 있다. `ok` 로 접으면 **유기**다."""
    v, best = node._person_frame_verdict(
        arr(node, det('person_ok'), det('person_fallen')))
    assert v == 'fallen'
    assert best is not None


def test_p2_low_confidence_detections_do_not_count(node):
    """문턱 밑은 없는 것으로 본다 — 안 그러면 노이즈가 신고를 만든다."""
    v, _ = node._person_frame_verdict(arr(node, det('person_fallen', conf=0.2)))
    assert v == 'none'


def test_p3_unknown_is_not_folded_into_ok(node):
    """🔴 부정 회귀 — 자세 판정 실패를 '괜찮다'로 접으면 안 된다."""
    v, _ = node._person_frame_verdict(arr(node, det('person_unknown')))
    assert v == 'unknown'


# ── 디바운스 ───────────────────────────────────────────────────────────
def test_p4_one_frame_does_not_confirm_anything(node):
    """한 장으로 확정되면 오탐 한 프레임이 곧 신고다."""
    assert feed(node, [det('person_fallen')], 0.1) == 'unknown'


def test_p5_fallen_confirms_faster_than_leaving(node):
    """🔴 **비대칭이 살아 있는가** — 이 시험이 이 경로의 심장이다.

    같은 1.5초를 줬을 때 `fallen` 은 확정되고 `ok` 는 아직 아니어야 한다.
    뒤집히면 로봇이 쓰러진 사람을 두고 떠나는 쪽이 빨라진다.
    """
    assert feed(node, [det('person_fallen')], 1.5) == 'fallen'
    node2_status = feed(node, [det('person_ok')], 1.5, t0=2000.0)
    assert node2_status == 'unknown', '떠나도 된다는 판정이 너무 빨리 섰다'


def test_p6_leaving_confirms_after_the_longer_wait(node):
    assert feed(node, [det('person_ok')], 2.5) == 'ok'


def test_p7_nobody_there_also_needs_the_long_wait(node):
    """🔴 `none` 도 '떠나도 된다'는 판정이다 — `ok` 와 같은 시간을 쓴다.

    짧게 잡으면 사람이 한 순간 가려진 것만으로 "아무도 없다" 가 되고,
    미션은 빈 복도를 유도하며 나간다.
    """
    assert feed(node, [], 1.5) == 'unknown'
    assert feed(node, [], 2.5, t0=2000.0) == 'none'


def test_p8_flicker_never_confirms(node):
    """🔴 한 프레임씩 뒤집히면 아무것도 확정되면 안 된다."""
    step, t = 0.1, 3000.0
    for i in range(40):
        node.now_sec = lambda _t=t + i * step: _t
        node._update_person(
            arr(node, det('person_fallen' if i % 2 else 'person_ok'),
                t=t + i * step),
            t + i * step)
    assert node._p_status == 'unknown'


# ── stale ≠ none ───────────────────────────────────────────────────────
def test_p9_silence_is_stale_not_none(node):
    """🔴 부정 회귀 — 발행이 끊긴 것을 '사람 없음'으로 읽으면 안 된다.

    `none` 은 "봤는데 없다", `stale` 은 "못 봤다"다. 섞으면 센서가 죽은 순간
    미션이 `NO_VICTIM` 으로 가서 사람을 두고 나간다.
    """
    assert feed(node, [det('person_ok')], 2.5) == 'ok'
    node.now_sec = lambda: 1000.0 + 2.5 + 5.0        # 5초 침묵
    node._person_tick()
    assert node._p_status == 'stale'


def test_p10_status_starts_as_stale_not_none(node):
    """기동 직후는 '사람 없음'이 아니다 — 아직 한 장도 못 봤다."""
    assert node._p_status == 'stale'


# ── /victim ────────────────────────────────────────────────────────────
def test_p11_victim_is_rearmed_after_leaving_fallen(node):
    """같은 임무에서 두 번째 쓰러짐이 생기면 신고가 다시 나가야 한다."""
    node._p_victim_pos = (1.0, 2.0)
    node._p_set_status('fallen')
    assert node._p_victim_sent
    node._p_set_status('ok')
    assert not node._p_victim_sent, '재무장되지 않아 두 번째 신고가 막힌다'


# ── 🔴 화재가 나간 뒤에도 살아 있는가 ──────────────────────────────────
def test_p12_person_path_survives_after_the_fire_alarm_fired(node):
    """🔴 **이 시험이 가장 중요하다.**

    `on_detections` 의 ①번 재발사 억제는 화재가 한 번 나가면 기본값이 **평생
    `return`** 이다. 사람 판정을 그 아래 두면 **화재 경보가 나간 순간 죽는데**,
    `SCAN_AREA`(사람 찾기)는 바로 그 다음 국면이다. 구현 중에 실제로 밟을 뻔했다.
    """
    node.fired_at = 1000.0                  # 화재가 이미 나갔다
    node.now_sec = lambda: 1000.5           # 쿨다운 안
    node.on_detections(arr(node, det('person_fallen'), t=1000.5))
    assert node._p_last_det_t is not None, \
        '🔴 화재 발사 뒤 사람 판정이 죽었다 — SCAN_AREA 가 통째로 안 돈다'
