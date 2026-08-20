# -*- coding: utf-8 -*-
"""optical frame 회전 검산 — ROS 없이 사원수 산술로만 (08-21, Codex §82.3).

[무엇을 잡나]
  `tools/e2e_adapter.py` 가 map→camera TF 를 **단위 사원수**로 두고 있었다.
  그래서 좌표 왕복 13/13 이 통과해도 REP-103 optical 축(x=오른쪽·y=아래·z=앞)을
  한 번도 시험하지 않았다. 그리고 `adapter.launch.py` 는 optical=true 에서
  `cam_yaw` 를 무시하고 있었다 — 운용자가 값을 줘도 좌표가 안 변했다.

  둘 다 "숫자가 맞아 보이는데 축이 틀린" 종류라 눈으로는 안 잡힌다.
  그래서 회전만 따로 떼어 합성 좌표로 검산한다.

[규약]
  optical (x_r, y_d, z_f)  →  base ( z_f, -x_r, -y_d )
  즉 앞(z)→앞(x), 오른쪽(x)→오른쪽(-y), 아래(y)→아래(-z).
"""

import math

# e2e 하네스가 쓰는 값과 같아야 한다 (여기가 정본).
OPTICAL_Q = (-0.5, 0.5, -0.5, 0.5)          # (x, y, z, w) = rpy(-π/2, 0, -π/2)


def quat_from_rpy(r, p, y):
    """tf2 와 같은 ZYX 순서. 반환 (x, y, z, w)."""
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def rotate(q, v):
    """사원수 q 로 벡터 v 를 돌린다 (v' = q v q*)."""
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 * (q_vec × v)
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def close(a, b, tol=1e-6):
    return all(abs(p - q) < tol for p, q in zip(a, b))


# ── ① 상수가 실제로 rpy(-π/2, 0, -π/2) 인가 ────────────────────────────

def test_optical_quaternion_matches_declared_rpy():
    q = quat_from_rpy(-math.pi / 2, 0.0, -math.pi / 2)
    assert close(q, OPTICAL_Q, 1e-9), (q, OPTICAL_Q)


# ── ② 세 방향이 규약대로 도는가 ────────────────────────────────────────

def test_optical_forward_becomes_base_forward():
    """앞 2m (optical z) → base +x 2m."""
    assert close(rotate(OPTICAL_Q, (0.0, 0.0, 2.0)), (2.0, 0.0, 0.0))


def test_optical_right_becomes_base_negative_y():
    """오른쪽 1m (optical +x) → base -y. 🔴 여기가 뒤집히면 좌우가 바뀐다."""
    assert close(rotate(OPTICAL_Q, (1.0, 0.0, 0.0)), (0.0, -1.0, 0.0))


def test_optical_left_becomes_base_positive_y():
    assert close(rotate(OPTICAL_Q, (-1.0, 0.0, 2.0)), (2.0, 1.0, 0.0))


def test_optical_down_becomes_base_negative_z():
    """아래 1m (optical +y) → base -z."""
    assert close(rotate(OPTICAL_Q, (0.0, 1.0, 0.0)), (0.0, 0.0, -1.0))


def test_identity_rotation_would_give_a_wrong_answer():
    """🔴 구판 대조 — 단위 사원수면 '앞 2m' 가 '위 2m' 가 된다.

    e2e 가 13/13 을 통과하면서도 축을 못 본 이유가 정확히 이것이다."""
    ident = (0.0, 0.0, 0.0, 1.0)
    assert close(rotate(ident, (0.0, 0.0, 2.0)), (0.0, 0.0, 2.0))


# ── ③ 장착 yaw 합성 (launch 의 --yaw 계산) ─────────────────────────────

def composed(theta):
    """adapter.launch.py 가 만드는 회전 = rpy(-π/2, 0, θ-π/2)."""
    return quat_from_rpy(-math.pi / 2, 0.0, theta - math.pi / 2)


def test_zero_mount_yaw_equals_plain_optical():
    assert close(composed(0.0), OPTICAL_Q, 1e-9)


def test_mount_yaw_rotates_the_forward_axis():
    """🔴 구판은 cam_yaw 를 무시했다 — 이 시험이 그때는 통과할 수 없다.

    카메라를 θ 만큼 돌려 달면 '앞 2m' 는 base 에서 (2cosθ, 2sinθ) 여야 한다."""
    for deg in (10, 30, -45, 90, 180):
        th = math.radians(deg)
        got = rotate(composed(th), (0.0, 0.0, 2.0))
        want = (2 * math.cos(th), 2 * math.sin(th), 0.0)
        assert close(got, want, 1e-6), (deg, got, want)


def test_mount_yaw_preserves_down_axis():
    """장착 yaw 는 수평 회전이다 — 아래 방향은 안 바뀌어야 한다."""
    for deg in (10, -60, 120):
        got = rotate(composed(math.radians(deg)), (0.0, 1.0, 0.0))
        assert close(got, (0.0, 0.0, -1.0), 1e-6), (deg, got)
