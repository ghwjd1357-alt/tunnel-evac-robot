# -*- coding: utf-8 -*-
"""
test_follower_monitor.py — FollowerMonitor + 화재 클램프 수식 단위테스트
============================================================
[왜 단위테스트인가]
  follower_monitor 는 "clock 주입 + 순수 로직" 구조라 시뮬 없이 몇 ms 만에
  검증 가능. 나중에 지도 배경제거·카메라 융합으로 알고리즘을 갈아탈 때
  이 테스트들이 회귀(기존에 되던 게 깨짐)를 자동으로 잡아준다.

[실행]
  cd ~/ros2_ws/src/mission_manager && python3 -m pytest test/ -q
  (colcon test 로도 돌아감)

[가짜 부품 2개]
  FakeClock : rclpy clock 흉내 — now() 가 FakeTime 을 주고, 뺄셈하면
              .nanoseconds 를 가진 객체가 나온다 (모니터가 쓰는 만큼만 흉내).
  make_scan : LaserScan 흉내 — 360빔, 배경 10m(벽 멀리), 지정한 곳에만
              점뭉치를 심는다.
"""

import math
import types

from mission_manager.follower_monitor import FollowerMonitor
from mission_manager.mission_node import clamp_to_fire_min_dist


# ============================================================
# 가짜 부품
# ============================================================
class _FakeDuration:
    def __init__(self, ns):
        self.nanoseconds = ns


class _FakeTime:
    def __init__(self, ns):
        self.ns = ns

    def __sub__(self, other):
        return _FakeDuration(self.ns - other.ns)


class FakeClock:
    # ⚠ float 초를 누적하면 0.1×11 = 1.0999…9 처럼 오차가 생겨
    #   디바운스 경계(정확히 1.0초)를 못 넘기는 가짜 실패가 난다 → 정수 ns 로 누적.
    def __init__(self):
        self.ns = 0

    def advance(self, sec):
        self.ns += int(round(sec * 1e9))

    def now(self):
        return _FakeTime(self.ns)


def make_scan(clusters, n=360, background=10.0):
    """clusters: [(중심각도(도, 0=전방/±180=후방), 빔 수, 거리m)] 로 가짜 스캔 생성."""
    inc = 2 * math.pi / n
    ranges = [background] * n
    for deg, width, r in clusters:
        center = round((math.radians(deg) + math.pi) / inc)
        for k in range(-(width // 2), width - (width // 2)):
            ranges[(center + k) % n] = r
    return types.SimpleNamespace(angle_min=-math.pi, angle_increment=inc,
                                 range_min=0.12, ranges=ranges)


def feed(mon, clock, scan, duration, dt=0.1):
    """같은 스캔을 duration 초 동안 반복 주입 (라이다 10Hz 흉내)."""
    steps = round(duration / dt)   # int() 는 11.999→11 로 한 스텝 깎아먹음
    for _ in range(steps):
        clock.advance(dt)
        mon.update(scan)


def new_monitor(clock, **kw):
    return FollowerMonitor(clock, cone_half_deg=60.0, max_range=2.5,
                           lost_sec=3.0, seen_sec=1.0, **kw)


# ============================================================
# FollowerMonitor 테스트
# ============================================================
def test_rear_person_visible_after_seen_sec():
    """정후방 1.5m 사람 크기 덩어리 → 1초(seen_sec) 지나면 rear/any 둘 다 검출."""
    clock = FakeClock()
    mon = new_monitor(clock)
    person = make_scan([(180.0, 10, 1.5)])   # 10빔 × 1.5m ≈ 폭 0.26m (사람 크기)

    feed(mon, clock, person, 0.5)
    assert not mon.visible('rear')           # 아직 1초 미만 — 디바운스가 막아야
    feed(mon, clock, person, 0.7)
    assert mon.visible('rear')
    assert mon.visible('any')
    assert not mon.lost('rear')


def test_rear_person_wraparound_merge():
    """정후방 덩어리는 배열 양끝(−π/+π)에 반씩 걸침 — 랩 병합 없으면 놓친다."""
    clock = FakeClock()
    mon = new_monitor(clock)
    # 180° 중심 10빔 = 인덱스 0~4 와 355~359 로 쪼개져 들어감 (make_scan 이 %n 처리)
    person = make_scan([(180.0, 10, 1.2)])
    feed(mon, clock, person, 1.2)
    assert mon.visible('rear')


def test_wall_arc_rejected():
    """문턱 안의 '긴 벽 호'(60빔 × 2.0m ≈ 폭 2.1m)는 사람이 아니다 — 배제."""
    clock = FakeClock()
    mon = new_monitor(clock)
    wall = make_scan([(90.0, 60, 2.0)])
    feed(mon, clock, wall, 2.0)
    assert not mon.visible('any')            # 1차 구현이었다면 여기서 오탐
    assert not mon.visible('rear')


def test_threshold_edge_sliver_rejected():
    """문턱(2.5m)에 걸친 벽 조각(2.4m — edge_margin 안쪽)은 슬라이버로 배제."""
    clock = FakeClock()
    mon = new_monitor(clock)
    sliver = make_scan([(45.0, 6, 2.4)])     # 폭은 사람 크기지만 문턱 코앞
    feed(mon, clock, sliver, 2.0)
    assert not mon.visible('any')


def test_front_person_any_only():
    """전방 사람: any 존만 검출, rear 존은 미검출 (역행 중 재발견 시나리오)."""
    clock = FakeClock()
    mon = new_monitor(clock)
    front = make_scan([(0.0, 10, 1.5)])
    feed(mon, clock, front, 1.2)
    assert mon.visible('any')
    assert not mon.visible('rear')
    assert not mon.lost('rear')              # rear 는 본 적 없음 → 판단 보류


def test_lost_debounce_asymmetric():
    """놓침은 3초(lost_sec) 연속 미검출 후에만 확정 — 그 전엔 False."""
    clock = FakeClock()
    mon = new_monitor(clock)
    person = make_scan([(180.0, 10, 1.5)])
    empty = make_scan([])

    feed(mon, clock, person, 1.5)            # 충분히 보여줌
    assert mon.visible('rear')
    feed(mon, clock, empty, 2.0)             # 2초 사라짐 — 아직 미확정
    assert not mon.lost('rear')
    feed(mon, clock, empty, 1.2)             # 누적 3.2초 — 확정
    assert mon.lost('rear')


def test_reset_prevents_immediate_relost():
    """reset('rear') = '방금 봤다' 처리 → 직후 재-놓침 판정이 나면 안 됨."""
    clock = FakeClock()
    mon = new_monitor(clock)
    person = make_scan([(180.0, 10, 1.5)])
    empty = make_scan([])

    feed(mon, clock, person, 1.5)
    feed(mon, clock, empty, 3.5)
    assert mon.lost('rear')                  # 놓침 상태에서
    mon.reset('rear')                        # GUIDE 복귀 시점의 리셋
    assert not mon.lost('rear')              # 즉시 재-놓침 금지
    feed(mon, clock, empty, 3.5)
    assert mon.lost('rear')                  # 그 후 다시 3초 지나면 정상 판정


def test_noise_points_rejected():
    """점 1~2개(min_points 미만) 노이즈는 무시."""
    clock = FakeClock()
    mon = new_monitor(clock)
    noise = make_scan([(180.0, 2, 1.5)])
    feed(mon, clock, noise, 2.0)
    assert not mon.visible('rear')


def test_inf_nan_ranges_safe():
    """실물 라이다의 inf/nan 빔이 섞여도 죽지 않고 정상 판정."""
    clock = FakeClock()
    mon = new_monitor(clock)
    scan = make_scan([(180.0, 10, 1.5)])
    scan.ranges[0] = float('inf')
    scan.ranges[10] = float('nan')
    feed(mon, clock, scan, 1.2)
    assert mon.visible('rear')


# ============================================================
# 화재 안전하한 클램프 수식 테스트 (§14.2 손검산의 자동화)
# ============================================================
def test_clamp_far_target_unchanged():
    """화재에서 충분히 먼 목표는 그대로."""
    assert clamp_to_fire_min_dist(0.0, 0.0, 14.0, 0.0, 5.0) == (0.0, 0.0)


def test_clamp_near_target_pushed_out():
    """§14.2 검산 사례: 목표(10,0)·화재(14,0)·하한5 → (9,0) 으로 밀림."""
    gx, gy = clamp_to_fire_min_dist(10.0, 0.0, 14.0, 0.0, 5.0)
    assert abs(gx - 9.0) < 1e-9 and abs(gy) < 1e-9


def test_clamp_diagonal_keeps_direction():
    """대각선 방향도 화재→목표 방향 유지한 채 정확히 dmin 거리로."""
    gx, gy = clamp_to_fire_min_dist(11.0, 1.0, 14.0, 4.0, 5.0)
    d = math.hypot(gx - 14.0, gy - 4.0)
    assert abs(d - 5.0) < 1e-9
    # 방향 확인: 화재→목표 단위벡터가 (−1/√2, −1/√2)
    assert gx < 14.0 and gy < 4.0 and abs((gx - 14.0) - (gy - 4.0)) < 1e-9


def test_clamp_target_equals_fire_gives_none():
    """목표=화재 지점 → 방향 정의 불가 → None (역행 포기 신호)."""
    assert clamp_to_fire_min_dist(14.0, 0.0, 14.0, 0.0, 5.0) is None
