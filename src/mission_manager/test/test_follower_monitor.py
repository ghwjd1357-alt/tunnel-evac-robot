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
from mission_manager.mission_node import (clamp_to_fire_min_dist,
                                          compute_gather_point,
                                          compute_gather_point_graph,
                                          route_through_graph,
                                          validate_waypoints)


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


def test_reset_any_rearms_lost_after_failed_search():
    """역행 재탐색 실패 → GUIDE 복귀 시나리오 (07-07 수정):
    reset('any') 없이는 lost('any') 가 즉시 참 → 같은 지점 2차 역행.
    재무장 후엔 lost_sec 을 새로 채워야만 다시 놓침 판정."""
    clock = FakeClock()
    mon = new_monitor(clock)
    person = make_scan([(180.0, 10, 1.5)])
    empty = make_scan([])

    feed(mon, clock, person, 1.5)            # 유도 중 추종자 목격
    feed(mon, clock, empty, 20.0)            # 놓침 + 역행 + 재탐색 대기 (오래 미검출)
    assert mon.lost('any')                   # 재무장 없으면 여전히 놓침 = 즉시 2차 역행
    mon.reset('any')                         # ★ 역행 실패 → GUIDE 복귀 시점의 재무장
    assert not mon.lost('any')               # 복귀 직후엔 놓침 아님
    feed(mon, clock, empty, 2.5)
    assert not mon.lost('any')               # lost_sec(3초) 미만 — 아직 아님
    feed(mon, clock, empty, 1.0)
    assert mon.lost('any')                   # 3초 넘게 새로 미검출 → 이제 정당한 2차


def test_noise_points_rejected():
    """점 1~2개(min_points 미만) 노이즈는 무시."""
    clock = FakeClock()
    mon = new_monitor(clock)
    noise = make_scan([(180.0, 2, 1.5)])
    feed(mon, clock, noise, 2.0)
    assert not mon.visible('rear')


def test_scan_outage_suspends_judgement():
    """★ watchdog (07-19): /scan 이 끊기면 visible 도 lost 도 아닌 '판단 보류'.
    수정 전엔 마지막 프레임의 '보임'이 동결돼 라이다가 죽어도 영원히
    visible=True — GUIDE 가 감시 없이 정상인 척 계속되는 구멍."""
    clock = FakeClock()
    mon = new_monitor(clock)
    person = make_scan([(180.0, 10, 1.5)])

    feed(mon, clock, person, 1.5)
    assert mon.visible('any')                # 정상 검출 중
    assert not mon.scan_stale()

    clock.advance(5.0)                       # /scan 사망 (update 호출 없음, 5초)
    assert mon.scan_stale()
    assert not mon.visible('any')            # ★ '보임' 동결 금지
    assert not mon.visible('rear')
    # 5초 > lost_sec(3초) 지만 데이터 없이 '놓침'(=비싼 역행 유발)도 선언 금지
    assert not mon.lost('any')
    assert not mon.lost('rear')


def test_scan_resume_recovers_detection():
    """끊겼던 /scan 이 복구되면 판정도 정상 복귀."""
    clock = FakeClock()
    mon = new_monitor(clock)
    person = make_scan([(180.0, 10, 1.5)])

    feed(mon, clock, person, 1.5)
    clock.advance(5.0)                       # 끊김
    assert not mon.visible('any')
    feed(mon, clock, person, 1.2)            # 복구 — 다시 검출
    assert not mon.scan_stale()
    assert mon.visible('any')


def test_scan_resume_empty_requires_fresh_lost_sec():
    """★ 07-19 Codex §3.1 재현: 사람을 보던 중 5초 단절 → 복구 첫 '빈' 프레임.
    수정 전엔 단절 5초가 미검출 시간으로 계산돼 빈 프레임 한 장에 즉시
    lost=True → 데이터 한 장으로 비싼 SEARCH_BACK 발동. 복구 후 lost_sec(3초)을
    새로 채워야만 놓침 확정이어야 한다."""
    clock = FakeClock()
    mon = new_monitor(clock)
    person = make_scan([(180.0, 10, 1.5)])
    empty = make_scan([])

    feed(mon, clock, person, 1.5)            # 사람을 보던 중
    clock.advance(5.0)                       # /scan 5초 단절 (> lost_sec)
    feed(mon, clock, empty, 0.1)             # 복구 첫 빈 프레임
    assert not mon.lost('rear')              # ★ 즉시 놓침 금지 (수정 전 True)
    assert not mon.lost('any')
    feed(mon, clock, empty, 2.5)             # 복구 후 연속 미검출 2.6초 — 아직
    assert not mon.lost('rear')
    feed(mon, clock, empty, 0.6)             # 3.2초 — 이제 정당한 놓침
    assert mon.lost('rear')


def test_scan_resume_detection_requires_fresh_seen_sec():
    """★ 07-19 Codex §3.1 재현: 0.5초만 보고(seen_sec 미달) 5초 단절 → 복구 첫
    '사람' 프레임. 수정 전엔 단절 전 타이머가 이어져 즉시 visible=True →
    SEARCH_BACK 이 프레임 한 장으로 재발견을 확정. 복구 후 seen_sec(1초)을
    새로 채워야만 검출 확정이어야 한다."""
    clock = FakeClock()
    mon = new_monitor(clock)
    person = make_scan([(180.0, 10, 1.5)])

    feed(mon, clock, person, 0.5)            # 부분 검출 (1초 미달)
    clock.advance(5.0)                       # 단절
    feed(mon, clock, person, 0.1)            # 복구 첫 사람 프레임
    assert not mon.visible('any')            # ★ 즉시 재발견 금지 (수정 전 True)
    feed(mon, clock, person, 0.5)            # 복구 후 연속 0.6초 — 아직
    assert not mon.visible('any')
    feed(mon, clock, person, 0.6)            # 1.2초 — 이제 정당한 검출
    assert mon.visible('any')


def test_never_scanned_is_stale():
    """시작 직후(한 장도 못 받음)도 stale — 판단 보류 상태로 출발."""
    clock = FakeClock()
    mon = new_monitor(clock)
    clock.advance(2.0)
    assert mon.scan_stale()
    assert not mon.visible('any')
    assert not mon.lost('any')


def test_repeated_stamp_scan_not_fresh():
    """★ S1-6 (07-19 Codex §10.6-5): 드라이버가 멈춘 채 같은 header.stamp 프레임을
    재전송 — 도착시각만 보면 '신선'으로 오인해 watchdog 이 영영 안 울린다.
    stamp 가 전진하지 않으면 프레임을 무시 → scan_timeout 뒤 stale 로 판정돼야."""
    clock = FakeClock()
    mon = new_monitor(clock)
    person = make_scan([(180.0, 10, 1.5)])

    def stamped(ns):
        s = types.SimpleNamespace(**vars(person))
        s.header = types.SimpleNamespace(stamp=types.SimpleNamespace(
            sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000))
        return s

    ns = 0
    for _ in range(12):                      # 정상: stamp 전진하며 1.2초 수신
        clock.advance(0.1)
        ns += 100_000_000
        mon.update(stamped(ns))
    assert mon.visible('rear')
    assert not mon.scan_stale()

    for _ in range(20):                      # 고장: 같은 stamp 재전송 2초
        clock.advance(0.1)
        mon.update(stamped(ns))              # stamp 그대로
    assert mon.scan_stale()                  # ★ 재전송은 신선도 갱신 못 함
    assert not mon.visible('rear')           # 판단 보류로 전환


def test_headerless_scan_still_works():
    """header 없는 스캔(테스트 픽스처·구형 소스)은 stamp 검사 생략 — 하위 호환."""
    clock = FakeClock()
    mon = new_monitor(clock)
    feed(mon, clock, make_scan([(180.0, 10, 1.5)]), 1.2)
    assert mon.visible('rear')


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


# ============================================================
# 집결지 계산 테스트 (ⓐ 모듈 — "화재에 가깝되 안전한 곳" 수식)
# ============================================================
def test_gather_standard_fire_reproduces_verified_point():
    """표준 테스트 화재(14,0)·탈출구(0,0)·거리8 → (6,0)·서향(π) = 기존 검증 집결지."""
    g = compute_gather_point(14.0, 0.0, 0.0, 0.0, 8.0)
    assert abs(g['x'] - 6.0) < 1e-9 and abs(g['y']) < 1e-9
    assert abs(g['yaw'] - math.pi) < 1e-9


def test_gather_lies_between_fire_and_escape_at_exact_dist():
    """집결지는 화재→탈출구 선분 위, 화재에서 정확히 gather_dist."""
    g = compute_gather_point(10.0, 6.0, 2.0, 0.0, 5.0)
    assert abs(math.hypot(g['x'] - 10.0, g['y'] - 6.0) - 5.0) < 1e-9
    # 선분 위(방향 일치): 화재→집결지 벡터와 화재→탈출구 벡터가 평행·같은 방향
    cross = (g['x'] - 10.0) * (0.0 - 6.0) - (g['y'] - 6.0) * (2.0 - 10.0)
    assert abs(cross) < 1e-9


def test_gather_yaw_faces_escape():
    """집결지 yaw = 탈출구를 바라보는 방향 (집결 후 바로 유도 출발)."""
    g = compute_gather_point(0.0, 10.0, 0.0, 0.0, 4.0)   # 북쪽 화재 → 남쪽 탈출구
    assert abs(g['yaw'] - (-math.pi / 2)) < 1e-9          # 남향(-y)


def test_gather_fire_near_escape_clamped_to_escape():
    """화재가 탈출구에 gather_dist 보다 가까우면 탈출구 자체로 클램프
    (지나쳐서 화재 반대편 미지 영역으로 나가면 안 됨)."""
    g = compute_gather_point(3.0, 0.0, 0.0, 0.0, 8.0)
    assert (g['x'], g['y']) == (0.0, 0.0)


def test_gather_fire_equals_escape_gives_none():
    """화재=탈출구 → 방향 정의 불가 → None (호출부가 yaml 고정값 fallback)."""
    assert compute_gather_point(0.0, 0.0, 0.0, 0.0, 8.0) is None


# ============================================================
# validate_waypoints (fail-fast, 07-07) 테스트
# ============================================================
def _valid_wp():
    """검사 통과하는 최소 구성 (waypoints.yaml 의 필수 키 축소판)."""
    return {
        'patrol': [{'x': 3.0, 'y': 0.0}],
        'gather': {'x': 6.0, 'y': 0.0},
        'escape': {'x': 0.0, 'y': 0.0},
        'gather_wait_sec': 8.0,
        'guide_speed': 0.12,
        'normal_speed': 0.26,
        'search_back': {'max_attempts': 2, 'min_fire_dist': 5.0,
                        'refind_wait_sec': 10.0},
    }


def test_validate_complete_config_passes():
    assert validate_waypoints(_valid_wp()) == []


def test_validate_reports_missing_escape_coord():
    """escape.y 누락 — 옛날엔 GUIDE 진입 순간(미션 한복판)에야 KeyError."""
    wp = _valid_wp()
    del wp['escape']['y']
    assert 'escape.y' in validate_waypoints(wp)


def test_validate_reports_missing_search_back_key():
    wp = _valid_wp()
    del wp['search_back']['min_fire_dist']
    assert 'search_back.min_fire_dist' in validate_waypoints(wp)


def test_validate_empty_patrol_rejected():
    wp = _valid_wp()
    wp['patrol'] = []
    assert any('patrol' in m for m in validate_waypoints(wp))


# ============================================================
# 복도 그래프 집결지 (07-19 — 곁복도 화재 대응) 테스트
# ============================================================
def _t_graph():
    """waypoints.yaml 의 T자 복도 그래프 축소판 (실제 좌표 그대로)."""
    return {
        'nodes': {
            'west':   {'x': 0.0,  'y': 0.0},
            'junc':   {'x': 12.0, 'y': 0.0},
            'east':   {'x': 26.0, 'y': 0.0},
            'branch': {'x': 12.0, 'y': 13.0},
        },
        'edges': [['west', 'junc'], ['junc', 'east'], ['junc', 'branch']],
    }


def test_graph_route_straight_corridor():
    """메인복도 위 두 점 — 경로는 직선과 동일해야."""
    path = route_through_graph(14.0, 0.0, 0.0, 0.0, _t_graph())
    total = sum(math.hypot(x1 - x0, y1 - y0)
                for (x0, y0), (x1, y1) in zip(path, path[1:]))
    assert abs(total - 14.0) < 1e-6


def test_graph_route_branch_goes_through_junction():
    """곁복도 → 탈출구 — 분기점(12,0)을 반드시 경유 (벽 뚫는 직선 금지)."""
    path = route_through_graph(12.0, 10.0, 0.0, 0.0, _t_graph())
    assert any(math.hypot(x - 12.0, y - 0.0) < 1e-6 for x, y in path)
    total = sum(math.hypot(x1 - x0, y1 - y0)
                for (x0, y0), (x1, y1) in zip(path, path[1:]))
    assert abs(total - 22.0) < 1e-6      # 10(남하) + 12(서진)


def test_graph_gather_standard_fire_matches_line_version():
    """표준 화재 (14,0): 그래프판도 기존 검증값 (6,0)·서향을 재현해야 (회귀 앵커)."""
    g = compute_gather_point_graph(14.0, 0.0, 0.0, 0.0, 8.0, _t_graph())
    assert abs(g['x'] - 6.0) < 1e-6 and abs(g['y']) < 1e-6
    assert abs(abs(g['yaw']) - math.pi) < 0.01


def test_graph_gather_branch_fire_stays_in_corridor():
    """★ 사용자 보고 재현: 곁복도 화재 (12,10) — 직선판은 (5.9, 4.9)=벽 안
    (복도 밖: 메인복도는 y≤3, 곁복도는 x 9~15), 그래프판은 곁복도 중심선 위
    (12,2) 가 나와야 한다."""
    bad = compute_gather_point(12.0, 10.0, 0.0, 0.0, 8.0)
    assert bad['y'] > 3.0 and bad['x'] < 9.0   # 직선판이 실제로 벽 안을 내놓는지 확인
    g = compute_gather_point_graph(12.0, 10.0, 0.0, 0.0, 8.0, _t_graph())
    assert abs(g['x'] - 12.0) < 1e-6 and abs(g['y'] - 2.0) < 1e-6
    assert abs(g['yaw'] - (-math.pi / 2)) < 0.01   # 진행 방향 = 남쪽(분기점으로)


def test_graph_gather_offcorridor_click_projected():
    """관제 클릭이 복도 밖(벽 안 12.5, 20)이어도 — 가까운 복도로 투영해 계산."""
    g = compute_gather_point_graph(12.5, 20.0, 0.0, 0.0, 8.0, _t_graph())
    assert abs(g['x'] - 12.0) < 1e-6 and abs(g['y'] - 5.0) < 1e-6


def test_graph_gather_fire_near_escape_clamped():
    """화재가 탈출구 코앞 — 탈출구 자체로 클램프 (직선판과 같은 규약)."""
    g = compute_gather_point_graph(3.0, 0.0, 0.0, 0.0, 8.0, _t_graph())
    assert abs(g['x']) < 1e-6 and abs(g['y']) < 1e-6


def test_graph_gather_fire_equals_escape_gives_none():
    """화재=탈출구 — None → 호출부가 직선판→yaml 로 fallback."""
    assert compute_gather_point_graph(0.0, 0.0, 0.0, 0.0, 8.0, _t_graph()) is None


def test_graph_disconnected_gives_none():
    """간선 선언이 빠져 안 이어진 그래프 — None (조용한 오답 금지)."""
    g = _t_graph()
    g['edges'] = [['west', 'junc']]      # east·branch 고립
    assert route_through_graph(12.0, 10.0, 0.0, 0.0, g) is not None  # branch 투영은 junc-east 아님
    g2 = {'nodes': {'a': {'x': 0.0, 'y': 0.0}, 'b': {'x': 5.0, 'y': 0.0},
                    'c': {'x': 20.0, 'y': 20.0}, 'd': {'x': 25.0, 'y': 20.0}},
          'edges': [['a', 'b'], ['c', 'd']]}
    assert route_through_graph(25.0, 20.0, 0.0, 0.0, g2) is None


def test_validate_graph_edge_with_unknown_node():
    """corridor_graph 간선이 미선언 노드를 참조 — 시작 즉시 검거 (fail-fast)."""
    wp = _valid_wp()
    wp['corridor_graph'] = {'nodes': {'a': {'x': 0.0, 'y': 0.0}},
                            'edges': [['a', 'ghost']]}
    assert any('corridor_graph.edges[0]' in m for m in validate_waypoints(wp))


# --- ★ S1-3 (07-19 Codex §10.6 공격 재현): 설정 전 항목 숫자·범위 검증 ---
def test_validate_nan_patrol_coord_rejected():
    """Codex 주입 재현: patrol.x=NaN — 수정 전엔 검증 통과."""
    wp = _valid_wp()
    wp['patrol'][0]['x'] = float('nan')
    assert any('patrol[0].x' in m for m in validate_waypoints(wp))


def test_validate_nan_speed_rejected():
    """Codex 주입 재현: guide_speed=NaN — 수정 전엔 검증 통과."""
    wp = _valid_wp()
    wp['guide_speed'] = float('nan')
    assert any('guide_speed' in m for m in validate_waypoints(wp))


def test_validate_negative_wait_rejected():
    """Codex 주입 재현: gather_wait_sec=-1 — 음수 대기시간."""
    wp = _valid_wp()
    wp['gather_wait_sec'] = -1
    assert any('gather_wait_sec' in m for m in validate_waypoints(wp))


def test_validate_negative_fire_dist_rejected():
    """Codex 주입 재현: min_fire_dist=-5 — 음수 안전거리는 안전장치 무력화."""
    wp = _valid_wp()
    wp['search_back']['min_fire_dist'] = -5
    assert any('min_fire_dist' in m for m in validate_waypoints(wp))


def test_validate_zero_speed_rejected():
    """속도 0 은 '영원히 도착 안 함' — 양수 강제."""
    wp = _valid_wp()
    wp['normal_speed'] = 0
    assert any('normal_speed' in m for m in validate_waypoints(wp))


def test_validate_bool_coord_rejected():
    """bool 은 int 하위 타입 — escape.x: true 가 숫자로 통과 금지."""
    wp = _valid_wp()
    wp['escape']['x'] = True
    assert any('escape.x' in m for m in validate_waypoints(wp))


def test_validate_string_number_rejected():
    """yaml 따옴표 실수: x: "3.0" (문자열) 도 검거."""
    wp = _valid_wp()
    wp['gather']['x'] = '3.0'
    assert any('gather.x' in m for m in validate_waypoints(wp))


# --- ★ 07-19 Codex §3.3: 그래프 fail-fast 강화 (숫자·기하·연결성) ---
def _graph_wp(graph):
    wp = _valid_wp()
    wp['corridor_graph'] = graph
    return wp


def test_validate_graph_non_numeric_coord():
    """★ Codex 재현 그대로: x: "not-a-number" — 수정 전엔 검증 통과 후
    화재 순간 라우팅 실패 → 조용한 직선 fallback (그래프 무력화)."""
    wp = _graph_wp({'nodes': {'a': {'x': 'not-a-number', 'y': 0.0},
                              'b': {'x': 5.0, 'y': 0.0}},
                    'edges': [['a', 'b']]})
    assert any('nodes.a.x(숫자 아님' in m for m in validate_waypoints(wp))


def test_validate_graph_nan_inf_coord():
    """NaN/inf 좌표도 유한값 검사에 걸려야 한다 (다익스트라 거리 계산 오염 방지)."""
    wp = _graph_wp({'nodes': {'a': {'x': float('nan'), 'y': 0.0},
                              'b': {'x': float('inf'), 'y': 0.0}},
                    'edges': [['a', 'b']]})
    bad = validate_waypoints(wp)
    assert any('nodes.a.x' in m for m in bad)
    assert any('nodes.b.x' in m for m in bad)


def test_validate_graph_bool_coord_rejected():
    """파이썬 함정: bool 은 int 하위 타입 — x: true 가 숫자로 통과하면 안 됨."""
    wp = _graph_wp({'nodes': {'a': {'x': True, 'y': 0.0},
                              'b': {'x': 5.0, 'y': 0.0}},
                    'edges': [['a', 'b']]})
    assert any('nodes.a.x' in m for m in validate_waypoints(wp))


def test_validate_graph_empty_nodes_edges():
    """빈 nodes/edges — 선언만 하고 내용 없음 = 그래프 무력 상태를 시작에 검거."""
    assert any('비어 있음' in m for m in validate_waypoints(
        _graph_wp({'nodes': {}, 'edges': []})))


def test_validate_graph_self_edge_rejected():
    wp = _graph_wp({'nodes': {'a': {'x': 0.0, 'y': 0.0},
                              'b': {'x': 5.0, 'y': 0.0}},
                    'edges': [['a', 'a'], ['a', 'b']]})
    assert any('자기 자신 간선' in m for m in validate_waypoints(wp))


def test_validate_graph_zero_length_edge_rejected():
    """같은 좌표의 두 노드를 잇는 길이 0 간선 — 진행 방향(yaw) 정의 불가 소지."""
    wp = _graph_wp({'nodes': {'a': {'x': 0.0, 'y': 0.0},
                              'b': {'x': 0.0, 'y': 0.0}},
                    'edges': [['a', 'b']]})
    assert any('길이 0 간선' in m for m in validate_waypoints(wp))


def test_validate_graph_disconnected_reported():
    """고립 구역 — 그 구역의 화재는 런타임에야 fallback 으로 새던 것을 시작에 검거."""
    wp = _graph_wp({'nodes': {'a': {'x': 0.0, 'y': 0.0}, 'b': {'x': 5.0, 'y': 0.0},
                              'c': {'x': 20.0, 'y': 20.0}, 'd': {'x': 25.0, 'y': 20.0}},
                    'edges': [['a', 'b'], ['c', 'd']]})
    assert any('비연결' in m for m in validate_waypoints(wp))


def test_validate_graph_actual_yaml_files_pass():
    """실제 배포 yaml 2종(T자·쌍굴)이 강화된 검사를 통과하는지 — 회귀 앵커."""
    import os
    import yaml
    base = os.path.join(os.path.dirname(__file__), '..', 'config')
    for fname in ('waypoints.yaml', 'waypoints_twin.yaml'):
        with open(os.path.join(base, fname)) as f:
            wp = yaml.safe_load(f)
        assert validate_waypoints(wp) == [], f'{fname} 검사 실패'
