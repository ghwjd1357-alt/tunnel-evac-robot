# -*- coding: utf-8 -*-
"""yaw 관문 판정 검사 — ROS 없이 순수 함수만 (08-21, Codex §83.9)."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from yaw_gate import angle_gap, verdict, wrap, yaw_from_quaternion   # noqa: E402

PI = math.pi


def test_exact_pi_passes():
    ok, gap = verdict(PI, PI, 0.3)
    assert ok and gap < 1e-9


def test_negative_pi_is_the_same_heading():
    """🔴 wrap 없으면 -π 가 6.28 rad 차이로 보인다 — 같은 방향인데 탈락한다."""
    ok, gap = verdict(-PI, PI, 0.3)
    assert ok, gap
    assert gap < 1e-6


def test_wrap_boundary_minus_pi_plus_epsilon():
    """`-π+ε` 도 통과해야 한다 (검토가 지정한 경계)."""
    ok, gap = verdict(-PI + 0.05, PI, 0.3)
    assert ok and abs(gap - 0.05) < 1e-9


def test_just_inside_and_outside_tolerance():
    assert verdict(PI - 0.29, PI, 0.3)[0]
    assert not verdict(PI - 0.31, PI, 0.3)[0]


def test_half_turn_is_rejected():
    """진입 yaw 가 π/2 면 180° 가 아니라 90° 가 나온다 — 막아야 한다."""
    assert not verdict(PI / 2, PI, 0.3)[0]


def test_zero_yaw_is_rejected():
    """시작 yaw 0 → 진입 yaw 0 이면 회전이 거의 0 이다."""
    assert not verdict(0.0, PI, 0.3)[0]


def test_start_pi_but_entry_half_pi_is_what_this_gate_exists_for():
    """🔴 §83.9 의 핵심 — 시작 때 π 였어도 진입 때 π/2 면 탈락해야 한다.

    구판 런북은 **테이크 시작 전**에 쟀다. 그 뒤 Nav2 주행·코너 회전이 yaw 를
    바꾸므로 시작 값은 완료판정의 대리값이 아니다."""
    assert verdict(PI, PI, 0.3)[0]              # 시작 시점: 통과
    assert not verdict(PI / 2, PI, 0.3)[0]      # 진입 시점: 탈락


def test_wrap_normalizes_into_range():
    for a in (3 * PI, -3 * PI, 7.0, -7.0):
        assert -PI - 1e-9 <= wrap(a) <= PI + 1e-9


def test_angle_gap_is_symmetric():
    assert abs(angle_gap(1.0, 2.0) - angle_gap(2.0, 1.0)) < 1e-12


def test_quaternion_to_yaw_round_trips():
    for deg in (0, 45, 90, 179, -179, -90):
        th = math.radians(deg)
        q = (0.0, 0.0, math.sin(th / 2), math.cos(th / 2))
        assert abs(wrap(yaw_from_quaternion(*q) - th)) < 1e-9, deg
