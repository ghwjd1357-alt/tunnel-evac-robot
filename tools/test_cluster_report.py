# -*- coding: utf-8 -*-
"""cluster_width_report 의 판정 축 검사 — 합성 스캔만 쓴다 (08-21, Codex §82.6).

[무엇을 잡나]
  구판 지표는 `통과 클러스터 / 전체 클러스터` 또는 `/스캔 수` 였다. 둘 다
  **평균**이라, 같은 총량이 시간축에 어떻게 놓였는지를 구별하지 못한다.
  실제 판정은 평균이 아니다 — `visible()` 은 **1초 연속** 검출을 요구한다.

  그래서 여기서는 **총 검출 수가 같고 배치만 다른** 두 입력을 만들어,
  판정이 갈리는지 본다. 갈리지 않으면 지표가 틀린 것이다.

⚠ 이 시험은 bag 을 안 읽는다. 실제 벽 오탐 수치는 실측 몫이다.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..',
                                'src', 'mission_manager'))

import pytest                                                    # noqa: E402

pytest.importorskip('sensor_msgs', reason='ROS 미설치 환경')
pytest.importorskip('rosbag2_py', reason='rosbag2 미설치 환경')

from sensor_msgs.msg import LaserScan                            # noqa: E402

import cluster_width_report as R                                 # noqa: E402

N = 360
HZ = 10


def scan(person=True, dist=1.5, half_beams=4, ns=0):
    """뒤쪽(각도 π 부근)에 사람 크기 덩어리 하나를 놓은 스캔.

    ⚠ `header.stamp` 를 반드시 전진시킨다 — `FollowerMonitor.update()` 는 stamp 가
      안 늘면 **드라이버 재전송**으로 보고 통째로 무시한다(S1-6 watchdog).
      합성 스캔에서 이걸 빼먹으면 첫 장만 처리되고 나머지가 조용히 사라진다."""
    s = LaserScan()
    s.header.stamp.sec = ns // 1_000_000_000
    s.header.stamp.nanosec = ns % 1_000_000_000
    s.angle_min = -math.pi
    s.angle_max = math.pi
    s.angle_increment = 2 * math.pi / N
    s.range_min = 0.05
    s.range_max = 10.0
    s.ranges = [float('inf')] * N
    if person:
        centre = N - 1        # 각도 ≈ +π → 후방
        for k in range(-half_beams, half_beams + 1):
            s.ranges[(centre + k) % N] = dist
    return s


def series(pattern):
    """pattern = 스캔마다 사람 유무(bool) 목록 → [(시각ns, scan)]."""
    step = int(1e9 / HZ)
    return [(i * step, scan(person=p, ns=i * step)) for i, p in enumerate(pattern)]


# ── ① 같은 총량, 다른 배치 ─────────────────────────────────────────────

def test_continuous_detection_confirms_but_bursts_do_not():
    """🔴 핵심 — 검출 30장으로 같지만 배치가 다르면 판정이 달라야 한다.

    연속 : ●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●  (3초 내리)
    몰림 : ●●●●○○○○○●●●●○○○○○…            (0.4초씩 끊김)
    구판 지표(스캔당 평균)로는 둘이 **완전히 같다.**"""
    cont = series([False] * 5 + [True] * 30 + [False] * 5)
    burst = series(([True] * 4 + [False] * 5) * 6 + [False] * 5)
    assert sum(1 for _, s in cont if min(s.ranges) < 9) == 30
    assert sum(1 for _, s in burst if min(s.ranges) < 9) == 24

    runs_c = R.replay(cont, width=0.8, rng=2.35, min_pts=3)[0]
    runs_b = R.replay(burst, width=0.8, rng=2.35, min_pts=3)[0]
    assert runs_c >= 1, '연속 3초인데 재발견이 안 잡혔다'
    assert runs_b == 0, f'0.4초 조각이 재발견으로 잡혔다 (runs={runs_b})'


def test_exactly_one_second_is_the_boundary():
    """seen_sec = 1.0초. 0.9초는 안 되고 1.1초는 돼야 한다."""
    short = series([False] * 3 + [True] * 9 + [False] * 3)    # 0.9초
    long_ = series([False] * 3 + [True] * 12 + [False] * 3)   # 1.2초
    assert R.replay(short, 0.8, 2.35, 3)[0] == 0
    assert R.replay(long_, 0.8, 2.35, 3)[0] >= 1


def test_empty_series_has_no_runs():
    assert R.replay(series([False] * 40), 0.8, 2.35, 3)[0] == 0


# ── ② 문턱이 실제로 판정을 바꾸는가 ────────────────────────────────────

def test_width_threshold_below_object_width_rejects_it():
    """덩어리 폭보다 좁은 문턱이면 사람으로 안 본다."""
    ser = series([True] * 30)
    wide_ok = R.replay(ser, 0.8, 2.35, 3)[0]
    too_tight = R.replay(ser, 0.05, 2.35, 3)[0]
    assert wide_ok >= 1
    assert too_tight == 0


def test_detect_range_shorter_than_object_rejects_it():
    """1.5 m 에 있는 덩어리는 detect_range 1.0 에서 안 보여야 한다."""
    ser = series([True] * 30)
    assert R.replay(ser, 0.8, 2.35, 3)[0] >= 1
    assert R.replay(ser, 1.0, 1.0, 3)[0] == 0


def test_min_points_filters_thin_objects():
    """점 3개짜리 조각은 min_points 5 에서 걸러져야 한다."""
    step = int(1e9 / HZ)
    thin = [(i * step, scan(person=True, half_beams=1, ns=i * step))
            for i in range(30)]
    assert R.replay(thin, 0.8, 2.35, 3)[0] >= 1
    assert R.replay(thin, 0.8, 2.35, 5)[0] == 0


# ── ③ 구간 나누기 ──────────────────────────────────────────────────────

def test_parse_segments_splits_by_seconds():
    ser = series([True] * 90)          # 9초
    segs = R.parse_segments('a=0:3,b=3:6,c=6:9', ser)
    assert [lb for lb, _ in segs] == ['a', 'b', 'c']
    assert [len(s) for _, s in segs] == [30, 30, 30]


def test_parse_segments_default_is_whole_bag():
    ser = series([True] * 20)
    segs = R.parse_segments('', ser)
    assert len(segs) == 1 and len(segs[0][1]) == 20
