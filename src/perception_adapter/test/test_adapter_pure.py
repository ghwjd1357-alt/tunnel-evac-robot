"""어댑터 순수 함수부 단위테스트 — ROS 를 띄우지 않고 판정 수식만 본다.

🔴 이 파일은 `src/mission_manager/test/` 와 **별도 패키지**다.
   `tools/doc_check.sh` 가 `pytest src/mission_manager/test/` 개수(184)를 계약으로
   잠그고 있어서, mission_manager 안에 테스트를 넣으면 그 계약이 깨진다.
   어댑터를 별도 패키지로 만든 이유 중 하나가 이것이다.
"""

import math

import pytest

from perception_adapter.adapter_node import (
    VALID_CLASSES, ConfirmTracker, clamp_range, fix_range, is_finite_point,
    validate_params,
    pick_best, stamp_age_sec)


class _P:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = float(x), float(y), float(z)


class _D:
    """Detection3D 최소 대역 — ROS 메시지 없이 pick_best 를 시험한다."""
    def __init__(self, class_name, confidence, x=1.0, y=0.0, z=0.0):
        self.class_name = class_name
        self.confidence = confidence
        self.position = _P(x, y, z)


# ── is_finite_point ────────────────────────────────────────────────────

def test_finite_normal():
    assert is_finite_point(1.0, 2.0, 3.0)


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), float('-inf')])
def test_finite_rejects_nonfinite_in_every_axis(bad):
    # 🔴 축마다 따로 본다 — x 만 검사하고 y 를 빠뜨리는 것이 흔한 구멍이다.
    assert not is_finite_point(bad, 0.0, 0.0)
    assert not is_finite_point(0.0, bad, 0.0)
    assert not is_finite_point(0.0, 0.0, bad)


# ── clamp_range ────────────────────────────────────────────────────────

def test_clamp_leaves_near_point_untouched():
    x, y, z, c = clamp_range(2.0, 0.0, 0.0, 5.0)
    assert (x, y, z) == (2.0, 0.0, 0.0)
    assert c is False


def test_clamp_pulls_far_point_to_max_and_keeps_bearing():
    x, y, z, c = clamp_range(20.0, 0.0, 0.0, 5.0)
    assert c is True
    assert math.isclose(math.sqrt(x * x + y * y + z * z), 5.0, rel_tol=1e-9)
    # 방위가 보존돼야 한다 — 거리만 자르는 것이 이 함수의 계약이다
    assert x > 0 and math.isclose(y, 0.0) and math.isclose(z, 0.0)


def test_clamp_preserves_direction_in_3d():
    x, y, z, c = clamp_range(6.0, 8.0, 0.0, 5.0)     # 원래 거리 10
    assert c is True
    assert math.isclose(math.sqrt(x * x + y * y + z * z), 5.0, rel_tol=1e-9)
    assert math.isclose(x / y, 6.0 / 8.0, rel_tol=1e-9)


def test_clamp_at_exact_boundary_does_not_clamp():
    _, _, _, c = clamp_range(5.0, 0.0, 0.0, 5.0)
    assert c is False


def test_clamp_zero_distance_is_returned_unchanged():
    # ⚠ 거리 0 은 방향이 없어 자를 수 없다. ZeroDivision 이 나면 안 된다.
    x, y, z, c = clamp_range(0.0, 0.0, 0.0, 5.0)
    assert (x, y, z) == (0.0, 0.0, 0.0)
    assert c is False


# ── fix_range ──────────────────────────────────────────────────────────

def test_fix_range_sets_distance_and_keeps_bearing():
    x, y, z = fix_range(10.0, 0.0, 0.0, 2.0)
    assert math.isclose(math.sqrt(x * x + y * y + z * z), 2.0, rel_tol=1e-9)
    assert x > 0


def test_fix_range_works_when_original_is_closer_than_fixed():
    # 격하 모드는 "멀면 당긴다" 가 아니라 "무조건 그 거리" 다
    x, y, z = fix_range(0.5, 0.0, 0.0, 2.0)
    assert math.isclose(math.sqrt(x * x + y * y + z * z), 2.0, rel_tol=1e-9)


def test_fix_range_returns_none_at_zero_distance():
    assert fix_range(0.0, 0.0, 0.0, 2.0) is None


# ── stamp_age_sec ──────────────────────────────────────────────────────

def test_stamp_age_positive_for_past():
    assert math.isclose(stamp_age_sec(100.0, 99.0), 1.0)


def test_stamp_age_negative_for_future_is_not_squashed():
    # 🔴 미래 stamp 를 0 으로 뭉개면 시계 어긋남이 '신선함'으로 통과한다
    assert stamp_age_sec(100.0, 105.0) < 0


# ── ConfirmTracker ─────────────────────────────────────────────────────
# 🔴 08-21 §82.4 이후 서명 = add(수신시각, 촬영시각stamp, 좌표).
#    수신시각만 세던 구판은 같은 프레임 한 장의 재전송을 확정으로 올렸다.
P = (2.0, 0.0, 0.0)          # 기준 좌표 — 특별한 뜻 없음


def test_tracker_needs_n_hits():
    t = ConfirmTracker(need=3, window_sec=10.0)
    assert t.add(1.0, 1.0, P) is False
    assert t.add(2.0, 2.0, P) is False
    assert t.add(3.0, 3.0, P) is True


def test_tracker_drops_hits_outside_window():
    t = ConfirmTracker(need=3, window_sec=2.0)
    t.add(0.0, 0.0, P)
    t.add(0.5, 0.5, P)
    # 10초 뒤 관측 하나 — 앞의 둘은 창 밖이라 버려진다
    assert t.add(10.0, 10.0, P) is False
    assert t.count(10.0) == 1


def test_tracker_tolerates_a_dropped_frame():
    # ⚠ '연속'이 아니라 '창 안에서 N번'이라는 계약. 깜빡임이 확정을 막으면 안 된다
    t = ConfirmTracker(need=3, window_sec=3.0)
    assert t.add(0.0, 0.0, P) is False
    # 1.0 에 프레임이 빠짐
    assert t.add(2.0, 2.0, P) is False
    assert t.add(2.5, 2.5, P) is True


def test_tracker_reset_clears():
    t = ConfirmTracker(need=2, window_sec=10.0)
    t.add(1.0, 1.0, P)
    t.reset()
    assert t.count() == 0
    assert t.add(2.0, 2.0, P) is False


# ── §82.4 재현본: 반복 관측이 아닌 것을 반복으로 세지 않는다 ────────────

def test_tracker_same_stamp_replay_never_confirms():
    """🔴 재현 — 같은 프레임 한 장을 5번. 구판은 [F,F,F,F,True] 였다."""
    t = ConfirmTracker(need=5, window_sec=10.0)
    got = [t.add(x / 10.0, 7.0, P) for x in range(5)]
    assert got == [False] * 5, got
    assert t.count() == 1


def test_tracker_out_of_order_stamp_rejected():
    """역순 stamp — 지연 도착이거나 시계 어긋남. 새 근거로 세지 않는다."""
    t = ConfirmTracker(need=2, window_sec=10.0)
    assert t.add(1.0, 5.0, P) is False
    assert t.add(1.1, 4.0, P) is False
    assert t.count() == 1


def test_tracker_nonfinite_stamp_rejected():
    t = ConfirmTracker(need=1, window_sec=10.0)
    assert t.add(1.0, float('nan'), P) is False
    assert t.add(1.0, float('inf'), P) is False
    assert t.count() == 0


def test_tracker_walking_chain_does_not_confirm():
    """🔴 08-21 §83.2 재현본 — 0.9 m 씩 걸어가면 매 걸음은 반경 안이다.

    구판은 **직전 점과만** 비교하는 single-link 라 첫 점과 3.6 m 떨어졌는데도
    확정됐다. 이어 붙기만 하면 복도 끝까지 한 화재가 된다.
    → seed(첫 hit) 기준 반경으로 잠갔다."""
    t = ConfirmTracker(need=5, window_sec=10.0, assoc_radius=1.0)
    pts = [(0.0, 0, 0), (0.9, 0, 0), (1.8, 0, 0), (2.7, 0, 0), (3.6, 0, 0)]
    got = [t.add(i * 0.1, float(i), q) for i, q in enumerate(pts)]
    assert not any(got), got


def test_tracker_seed_radius_allows_wander_within_the_circle():
    """🟢 seed 반경 **안**에서 흔들리는 것은 같은 대상이다 — 끊으면 안 된다."""
    t = ConfirmTracker(need=4, window_sec=10.0, assoc_radius=1.0)
    pts = [(2.0, 0.0, 0), (2.4, 0.3, 0), (1.7, -0.4, 0), (2.2, 0.2, 0)]
    got = [t.add(i * 0.1, float(i), q) for i, q in enumerate(pts)]
    assert got[-1] is True, got


def test_tracker_spatially_separated_hits_do_not_accumulate():
    """🔴 서로 다른 자리의 한-프레임 오탐 5개 — 한 화재가 아니다."""
    t = ConfirmTracker(need=3, window_sec=10.0, assoc_radius=1.0)
    pts = [(0.0, 0.0, 0.0), (5.0, 0.0, 0.0), (10.0, 0.0, 0.0),
           (15.0, 0.0, 0.0), (20.0, 0.0, 0.0)]
    got = [t.add(i * 0.1, float(i), q) for i, q in enumerate(pts)]
    assert not any(got), got
    assert t.count() == 1          # 매번 새로 시작한다


def test_tracker_same_place_jitter_still_confirms():
    """🟢 진짜 화재는 몇 cm 흔들린다 — 그건 같은 대상이다."""
    t = ConfirmTracker(need=3, window_sec=10.0, assoc_radius=1.0)
    pts = [(2.0, 0.0, 0.0), (2.10, 0.05, 0.0), (1.95, -0.03, 0.0)]
    got = [t.add(i * 0.1, float(i), q) for i, q in enumerate(pts)]
    assert got[-1] is True, got


def test_tracker_reset_clears_stamp_monotonic_guard():
    """재무장(다음 테이크) 뒤에는 stamp 기준도 지워져야 한다 (§82.5)."""
    t = ConfirmTracker(need=1, window_sec=10.0)
    assert t.add(1.0, 100.0, P) is True
    t.reset()
    assert t.add(2.0, 50.0, P) is True      # 더 이른 stamp 도 새 시작으로 받는다


# ── validate_params (§82.4) ────────────────────────────────────────────

GOOD = {
    'min_confidence': 0.4, 'confirm_frames': 5, 'confirm_window_sec': 3.0,
    'max_stamp_age_sec': 1.0, 'max_range': 5.0, 'fixed_range': 2.0,
    'refire_cooldown_sec': -1.0, 'confirm_assoc_radius_m': 1.0,
    'tf_wait_sec': 0.10,
}


def test_params_good_set_passes():
    assert validate_params(dict(GOOD)) == []


def test_params_negative_cooldown_is_allowed():
    """0 이하 = '평생 1회' 라는 뜻이라 부호를 막지 않는다."""
    assert validate_params({**GOOD, 'refire_cooldown_sec': 0.0}) == []


def test_params_reject_bad_values():
    """🔴 각 파라미터의 NaN/Inf/음수/0/범위밖을 항목별로 막는다."""
    bad = [
        ('min_confidence', 1.5), ('min_confidence', -0.1),
        ('min_confidence', float('nan')),
        ('confirm_frames', 0), ('confirm_frames', -3), ('confirm_frames', 2.5),
        ('confirm_window_sec', 0.0), ('confirm_window_sec', -1.0),
        ('confirm_window_sec', float('inf')),
        ('max_stamp_age_sec', 0.0), ('max_stamp_age_sec', float('nan')),
        ('max_range', -1.0), ('max_range', 0.0), ('max_range', float('inf')),
        ('fixed_range', -2.0), ('fixed_range', float('nan')),
        ('refire_cooldown_sec', float('nan')),
        ('refire_cooldown_sec', float('inf')),
        ('confirm_assoc_radius_m', 0.0), ('confirm_assoc_radius_m', -1.0),
        # 🔴 §83.2 — 양수만 보면 1000 도 통과했다. 그러면 결합이 사실상 사라진다.
        ('confirm_assoc_radius_m', 1000.0), ('confirm_assoc_radius_m', 2.5),
        ('tf_wait_sec', -0.1), ('tf_wait_sec', 5.0),
        ('tf_wait_sec', float('nan')),
        ('min_confidence', 'x'), ('max_range', None),
    ]
    for k, v in bad:
        out = validate_params({**GOOD, k: v})
        assert out and any(k in m for m in out), (k, v, out)


# ── pick_best ──────────────────────────────────────────────────────────

def test_pick_best_takes_highest_confidence_of_target_class():
    best, bad = pick_best(
        [_D('fire', 0.55), _D('fire', 0.77), _D('person_ok', 0.99)],
        'fire', 0.40)
    assert best is not None and math.isclose(best.confidence, 0.77)
    assert bad == []


def test_pick_best_returns_none_on_empty_array():
    # 빈 배열 = 정상 미탐지. 실패가 아니다 (합의사항 §6)
    best, bad = pick_best([], 'fire', 0.40)
    assert best is None and bad == []


def test_pick_best_rejects_below_min_confidence():
    best, _ = pick_best([_D('fire', 0.10)], 'fire', 0.40)
    assert best is None


def test_pick_best_rejects_nonfinite_position():
    best, _ = pick_best([_D('fire', 0.99, x=float('nan'))], 'fire', 0.40)
    assert best is None


def test_pick_best_reports_contract_violation_and_does_not_use_it():
    # 🔴 'human'·'car' 는 계약 열거 밖 — 역할 B가 drop 하기로 한 값이다.
    #    우리에게 오면 계약이 깨진 것이므로 조용히 버리지 않고 보고한다.
    best, bad = pick_best([_D('human', 0.99)], 'fire', 0.40)
    assert best is None
    assert bad == ['human']


def test_pick_best_ignores_other_valid_class():
    best, bad = pick_best([_D('smoke', 0.99)], 'fire', 0.40)
    assert best is None and bad == []


def test_valid_classes_matches_contract():
    # 계약 정본 = 합의사항 §15-b · tunnel_interfaces/msg/Detection3D.msg
    assert set(VALID_CLASSES) == {
        'person_fallen', 'person_ok', 'person_unknown', 'fire', 'smoke'}


# ── §82.4 재현본: confidence 계약 (0~1 유한값) ──────────────────────────

def test_pick_best_rejects_nan_confidence():
    """🔴 재현 — NaN 은 어떤 비교에도 False 라 `< min_conf` 를 그냥 통과했다.

    구판에서 NaN 짜리가 best 로 채택됐고 violations 는 빈 목록이었다.
    즉 계약 위반인데 **아무도 몰랐다.**"""
    best, bad = pick_best([_D('fire', float('nan'))], 'fire', 0.40)
    assert best is None
    assert bad and 'confidence' in bad[0], bad


def test_pick_best_rejects_inf_confidence():
    best, bad = pick_best([_D('fire', float('inf'))], 'fire', 0.40)
    assert best is None and bad


def test_pick_best_rejects_out_of_range_confidence():
    """계약은 0~1 이다. 1.5 는 다른 스케일을 쓰고 있다는 신호라 거부 + 신고."""
    for v in (1.5, -0.2):
        best, bad = pick_best([_D('fire', v)], 'fire', 0.40)
        assert best is None and bad, v


def test_pick_best_bad_confidence_does_not_hide_a_good_one():
    """🟢 불량 한 건이 같은 프레임의 정상 탐지를 가리면 안 된다."""
    best, bad = pick_best(
        [_D('fire', float('nan')), _D('fire', 0.66)], 'fire', 0.40)
    assert best is not None and math.isclose(best.confidence, 0.66)
    assert bad


def test_params_zero_tf_wait_is_allowed():
    """0 = 기다리지 않는다. 유효한 선택이라 막지 않는다."""
    assert validate_params({**GOOD, 'tf_wait_sec': 0.0}) == []


def test_params_assoc_radius_upper_bound_is_two_metres():
    """상한 근거 = 연결통로 반폭 0.825 m · 아래복도 반폭 1.18 m.

    화재는 정지 물체이므로 map 좌표에서 그보다 크게 튀면 같은 대상이 아니다."""
    assert validate_params({**GOOD, 'confirm_assoc_radius_m': 2.0}) == []
    assert validate_params({**GOOD, 'confirm_assoc_radius_m': 2.01}) != []


# ── §84.5 재현본: 만료된 seed 가 창 안 누적을 지우면 안 된다 ────────────

def test_tracker_expired_seed_does_not_reset_valid_window():
    """🔴 재현 — 창(3.0s) 안 2.8~3.2 의 5건은 서로 최대 0.9m 다.

    §83.2 가 seed 결합을 넣으면서 **비교를 prune 보다 먼저** 했다. 그래서 이미
    만료된 x=0 seed 와 비교해 전부 reset 됐고 count 2 · 확정 0 이었다.
    walking-chain 을 막은 대가로 반대 방향 false negative 가 생긴 것이다."""
    t = ConfirmTracker(need=5, window_sec=3.0, assoc_radius=1.0)
    seq = [(0.0, 0.0), (2.8, 0.9), (2.9, 0.9), (3.0, 0.9), (3.1, 1.8), (3.2, 1.8)]
    got = [t.add(tt, tt, (x, 0.0, 0.0)) for tt, x in seq]
    assert got[-1] is True, (got, t.count())
    assert t.count() == 5, t.count()


def test_tracker_still_rejects_walking_chain_after_prune_fix():
    """🟢 회귀 방지 — §83.2 가 막은 것을 §84.5 수정이 되살리면 안 된다."""
    t = ConfirmTracker(need=5, window_sec=10.0, assoc_radius=1.0)
    pts = [(0.0, 0, 0), (0.9, 0, 0), (1.8, 0, 0), (2.7, 0, 0), (3.6, 0, 0)]
    assert not any(t.add(i * 0.1, float(i), q) for i, q in enumerate(pts))


def test_tracker_boundary_exactly_at_window_edge():
    """창 경계 정확히 위의 hit 는 살아 있다 (`>=` 계약)."""
    t = ConfirmTracker(need=2, window_sec=1.0, assoc_radius=1.0)
    t.add(0.0, 0.0, (0.0, 0, 0))
    assert t.add(1.0, 1.0, (0.0, 0, 0)) is True, t.count()


# ── §84.4: confirm_frames 하한 ─────────────────────────────────────────

def test_params_reject_confirm_frames_one():
    """🔴 need=1 이면 '반복 관측' 이 한 장이라 억제가 없다 — 존재 이유가 사라진다."""
    assert validate_params({**GOOD, 'confirm_frames': 1}) != []
    assert validate_params({**GOOD, 'confirm_frames': 2}) == []
