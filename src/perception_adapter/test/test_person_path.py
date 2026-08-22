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


def test_p2_low_confidence_is_unknown_not_nobody(node):
    """🔴 08-22 §87.5 — **이 시험이 결함을 고정하고 있었다.**

    구판은 `person_fallen(conf=0.2)` → `none` 이었고 시험도 그렇게 못 박았다.
    그런데 `none` 은 "봤는데 아무도 없다" 라 4초 뒤 `NO_VICTIM` 으로 간다 —
    **저조도에서 사람 후보가 계속 낮은 confidence 로 오면, 사람이 눈앞에 있는데
    "아무도 없다"고 신고하고 떠난다.**
    후보가 **있는데 못 믿는 것**은 `unknown`(판정 보류)이다.
    """
    v, _ = node._person_frame_verdict(arr(node, det('person_fallen', conf=0.2)))
    assert v == 'unknown'


def test_p2b_no_person_candidate_at_all_is_none(node):
    """🔵 역회귀 — `none` 은 후보가 **전혀 없는** 프레임에만 나온다."""
    assert node._person_frame_verdict(arr(node))[0] == 'none'
    assert node._person_frame_verdict(arr(node, det('fire', 0.9)))[0] == 'none'


def test_p2c_non_finite_confidence_never_becomes_ok(node):
    """🔴 부정 회귀 — `NaN < 0.5` 는 파이썬에서 **False** 라 그대로 통과했다.

    검토 재현: `person_ok(conf=NaN)` → `ok`. 쓰레기 한 프레임이 "떠나도 된다"로
    접힌 것이다. `Inf` 와 1.0 초과도 같다.
    """
    for bad in (float('nan'), float('inf'), -0.1, 1.5):
        v, _ = node._person_frame_verdict(arr(node, det('person_ok', conf=bad)))
        assert v == 'unknown', f'conf={bad} 가 {v} 로 접혔다'


def test_p2d_one_unclear_person_blocks_leaving(node):
    """🔴 부정 회귀 — 여러 사람 중 하나라도 판정 불가면 떠나지 않는다.

    구판은 unknown 한 명이 있어도 다른 ok 한 명 때문에 `ok` 를 냈다(§87.5 재현).
    그 한 명이 실은 쓰러진 사람일 수 있다.
    """
    v, _ = node._person_frame_verdict(
        arr(node, det('person_ok', 0.9), det('person_unknown', 0.9)))
    assert v == 'unknown'
    v, _ = node._person_frame_verdict(
        arr(node, det('person_ok', 0.9), det('person_ok', 0.2)))
    assert v == 'unknown', '못 믿는 후보가 섞였는데 떠나도 된다고 했다'


def test_p2e_all_trusted_ok_still_means_ok(node):
    """🔵 역회귀 — 조이면서 정상 경로를 막으면 유도가 영원히 시작 안 된다."""
    v, _ = node._person_frame_verdict(
        arr(node, det('person_ok', 0.9), det('person_ok', 0.8)))
    assert v == 'ok'


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


# ── 🔴 08-22 §87.6 — 새 fallen 은 자기 세대 좌표만 발행한다 ────────────
def test_p13_a_second_victim_never_reuses_the_first_position(node):
    """🔴 부정 회귀 — 두 번째 쓰러진 사람의 TF 가 실패하면 **아무것도 안 낸다.**

    구판은 `_p_victim_pos` 를 생성 때만 None 으로 두고 그 뒤로 지우지 않아,
    두 번째 fallen 의 변환 실패 시 **첫 사람 좌표를 다시 발행**했다
    (검토 §87.6 재현: `/victim` = [(1,2), (1,2)]). 관제가 두 번째 사람을 첫 사람
    자리로 찾으러 가고, 그동안 진짜 사람은 그대로 있다.

    ⚠ `_p_set_status` 를 직접 부르지 않는다 — 그러면 세대 경계를 우회해
    **실제로 도는 경로를 안 본다.** 프레임을 흘려 `_update_person` 이 가르게 한다.
    """
    sent = []
    node.victim_pub = type('P', (), {'publish': lambda _s, m: sent.append(
        (m.pose.position.x, m.pose.position.y))})()

    # ① 첫 쓰러진 사람 — 이 세대에서 좌표를 얻었다고 두고 확정시킨다
    node._p_streak_class = 'fallen'
    node._p_streak_since = 7000.0
    node._p_streak_frames = 10
    node._p_victim_pos = (1.0, 2.0)
    node.now_sec = lambda: 7002.0
    node._update_person(arr(node, det('person_fallen'), t=7002.0), 7002.0)
    assert sent == [(1.0, 2.0)], f'첫 신고가 안 나갔다: {sent}'

    # ② 그 사람이 일어났다 — 세대가 바뀐다
    for i in range(45):
        t = 7003.0 + i * 0.1
        node.now_sec = lambda _t=t: _t
        node._update_person(arr(node, det('person_ok'), t=t), t)
    assert node._p_status == 'ok'

    # ③ 두 번째 쓰러진 사람 — 이 세대에서는 TF 가 실패해 좌표를 못 얻는다
    for i in range(30):
        t = 7010.0 + i * 0.1
        node.now_sec = lambda _t=t: _t
        node._update_person(arr(node, det('person_fallen'), t=t), t)
    assert node._p_status == 'fallen', '두 번째 확정이 안 됐다'
    assert sent == [(1.0, 2.0)], f'🔴 첫 사람 좌표가 다시 나갔다: {sent}'


def test_p14_a_new_fallen_streak_drops_the_previous_candidate(node):
    """🔴 부정 회귀 — 새 fallen **세대**는 이전 후보 좌표를 물려받지 않는다.

    ⚠ 이 시험은 `_p_set_status` 의 부수효과에 기대면 안 된다. 그쪽에서 지우면
    확정 전 streak 를 쌓는 동안 매 프레임 좌표가 날아가, **확정 프레임의 TF 가
    실패하면 신고 좌표가 없다.** 폐기는 **세대 경계**에서만 일어나야 하므로
    여기서는 `_update_person` 을 타지 않고 그 경계만 직접 본다.
    """
    node._p_victim_pos = (9.0, 9.0)
    node._p_streak_class = 'ok'
    node._p_streak_since = 5000.0
    node._p_streak_frames = 3
    node.now_sec = lambda: 5000.1
    # 'ok' → 'fallen' 로 프레임 판정이 바뀌는 순간이 세대 경계다
    node._update_person(arr(node, det('person_fallen'), t=5000.1), 5000.1)
    assert node._p_streak_class == 'fallen'
    assert node._p_victim_pos is None, '이전 세대 좌표가 새 세대로 넘어왔다'


def test_p15_a_coordinate_from_earlier_in_the_streak_survives_to_confirm(node):
    """🔵 역회귀 — 같은 세대 안에서 앞 프레임이 얻은 좌표는 **살아 있어야** 한다.

    확정되는 그 한 프레임에서만 TF 가 성공해야 신고가 나간다면, 회전 중
    한 번씩 실패하는 실물에서 신고가 거의 안 나간다.
    """
    sent = []
    node.victim_pub = type('P', (), {'publish': lambda _s, m: sent.append(
        (m.pose.position.x, m.pose.position.y))})()
    node._p_streak_class = 'fallen'
    node._p_streak_since = 6000.0
    node._p_streak_frames = 10
    node._p_victim_pos = (3.0, 4.0)          # 이 세대의 앞 프레임이 얻은 좌표
    node.now_sec = lambda: 6002.0
    # 확정 프레임 — TF 는 실패해도(_person_to_map None) 세대 좌표로 신고한다
    node._update_person(arr(node, det('person_fallen'), t=6002.0), 6002.0)
    assert node._p_status == 'fallen'
    assert sent == [(3.0, 4.0)], f'세대 좌표가 버려졌다: {sent}'


def test_p16_an_interrupted_fallen_streak_drops_its_coordinate(node):
    """🔴 세대가 끊기면 좌표도 끊긴다 — `unknown` 한 프레임이 streak 를 리셋한다.

    끊긴 뒤 다시 fallen 이 서면 그것은 **새 세대**다. 옛 좌표를 물려받으면
    그 사람이 그 자리에 있다는 보장이 없다.
    """
    node._p_streak_class = 'fallen'
    node._p_streak_since = 8000.0
    node._p_streak_frames = 5
    node._p_victim_pos = (5.0, 6.0)
    node.now_sec = lambda: 8001.0
    # 자세를 못 믿는 프레임 한 장 → `_p_reset_streak`
    node._update_person(arr(node, det('person_unknown'), t=8001.0), 8001.0)
    assert node._p_status == 'unknown'
    assert node._p_victim_pos is None, '세대가 끊겼는데 좌표가 남았다'


# ── 🔴 08-22 역할 B 실측과 상수의 정합 ─────────────────────────────────
# 인수인계서 §8-a·§9 의 실측값. 🔴 여기 숫자를 고칠 때는 **출처를 같이 고친다** —
# 상수만 맞춰 놓고 실측이 바뀌면 판정이 조용히 무효가 된다.
ROLE_B_DETECTIONS_HZ = 3.8      # §8-a 정지 상태 (🔴 회전 중 미측정)
ROLE_B_MAX_GAP_SEC = 0.729      # §8-a 프레임 간 최대 간격
ROLE_B_FIRE_FALSE_MAX = 0.58    # §9 불이 없는데 뜬 fire 의 최대 confidence


def defaults():
    import rclpy
    rclpy.init()
    n = PerceptionAdapter()
    try:
        return {k: n.get_parameter(k).value for k in (
            'person_min_frames', 'person_confirm_sec_fallen',
            'person_confirm_sec_leave', 'person_stale_sec', 'min_confidence')}
    finally:
        n.destroy_node()
        rclpy.shutdown()


def test_p17_fallen_can_actually_confirm_at_the_measured_frame_rate():
    """🔴 부정 회귀 — 창에 들어오는 프레임보다 `min_frames` 가 크면 **영원히 확정 안 된다.**

    계약은 10 Hz 인데 실측은 **3.8 Hz** 다. 그러면 `fallen 1.5s` 창에 5.7 프레임만
    들어오는데 구값 `min_frames=6` 은 그것을 넘는다 — **쓰러진 사람을 보고도 신고가
    안 나갔다.** 계약 숫자가 아니라 **실측**으로 맞춰야 한다.
    """
    d = defaults()
    got = d['person_confirm_sec_fallen'] * ROLE_B_DETECTIONS_HZ
    assert d['person_min_frames'] <= got - 1.0, (
        f"fallen 창에 {got:.1f} 프레임이 들어오는데 min_frames={d['person_min_frames']} "
        f"— 확정이 불가능하거나 여유가 없다")


def test_p18_leaving_still_needs_a_lot_more_evidence_than_fallen():
    """🔵 역회귀 — 상수를 실측에 맞추면서 비대칭이 뒤집히면 안 된다."""
    d = defaults()
    assert d['person_confirm_sec_leave'] > d['person_confirm_sec_fallen'] * 2, \
        '떠나도 된다는 판정이 신고보다 신중해야 한다는 원칙이 깨졌다'


def test_p19_stale_is_not_declared_during_normal_gaps():
    """🔴 부정 회귀 — 계약값 0.5 는 실측 최대 간격 0.729 보다 **작다.**

    그대로 두면 정상 동작 중에 `stale` 이 떠서 판정이 계속 튕기고 아무것도 확정되지
    않는다. ⚠ 계약(§4.1)은 여전히 0.5 이고 **구현이 계약을 못 지키는 것**이다 —
    이 시험은 그 사실을 우리 쪽에서 흡수한 자리를 지킨다.
    """
    d = defaults()
    assert d['person_stale_sec'] > ROLE_B_MAX_GAP_SEC * 1.5, (
        f"stale={d['person_stale_sec']} 가 실측 최대 간격 {ROLE_B_MAX_GAP_SEC} 에 너무 가깝다")


def test_p20_the_fire_threshold_clears_the_observed_false_positives():
    """🔴 불이 없는데 뜨는 fire(0.45~0.58)가 문턱을 통과하면 본편 테이크가 죽는다.

    ⚠ 이것은 근거 있는 문턱이 아니라 **응급 조치**다 — 진짜 화재의 confidence 분포를
    아직 아무도 안 쟀다. 거짓 알람은 테이크를 버리고, 놓친 자동 검출은 오퍼레이터가
    수동 `/alarm` 으로 메운다. 되돌릴 수 있는 쪽을 고른 것이다.
    """
    d = defaults()
    assert d['min_confidence'] > ROLE_B_FIRE_FALSE_MAX, (
        f"min_confidence={d['min_confidence']} 로는 관측된 오탐 "
        f"{ROLE_B_FIRE_FALSE_MAX} 이 그대로 통과한다")
