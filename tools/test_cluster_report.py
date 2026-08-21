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


PHASES = [(ph, tf) for ph in (0.0, 0.25, 0.5, 0.75) for tf in (False, True)]


def worst(ser, width=0.8, rng=2.35, mp=3):
    """도구와 같은 최악값 — 8조합(위상4×같은시각 순서2) 중 최대 run."""
    return max(R.replay(ser, width, rng, mp, phase=ph, tick_first=tf)[0]
               for ph, tf in PHASES)


def always(ser, width=0.8, rng=2.35, mp=3):
    """8조합 **전부**에서 잡히는가 — 사람 쪽 기준."""
    return min(R.replay(ser, width, rng, mp, phase=ph, tick_first=tf)[0]
               for ph, tf in PHASES)


def hold(ser, width=0.8, rng=2.35, mp=3):
    """8조합 중 **최소** 연속 지속시간 [초] — 안정성 지표."""
    return min(R.longest_run(R.replay(ser, width, rng, mp,
                                      phase=ph, tick_first=tf)[3])
               for ph, tf in PHASES)


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
    """seen_sec = 1.0초. 0.9초는 어떤 위상에서도 안 잡혀야 한다.

    ⚠ 1.2초는 **잡힐 수도, 안 잡힐 수도** 있다 — mission 은 2 Hz 로만 보므로
      창이 tick 사이에 들어가면 놓친다. 그래서 "최악값" 과 "최선값" 을 나눠 본다.
      이것이 §83.5 가 말한 tick 위상 문제의 정체다."""
    short = series([False] * 3 + [True] * 9 + [False] * 5)     # 0.9초
    long_ = series([False] * 3 + [True] * 12 + [False] * 5)    # 1.2초
    assert worst(short) == 0, '0.9초가 어딘가에서 확정됐다'
    assert worst(long_) >= 1, '1.2초가 어느 위상에서도 안 잡힌다'
    # 🔴 그리고 1.2초는 **모든** 위상에서 잡히지는 않는다 — 사람 기준으로 쓰면 안 된다
    assert always(long_) == 0, '1.2초가 전 위상에서 잡히면 이 경고가 무의미하다'
    stable = series([False] * 3 + [True] * 40 + [False] * 5)   # 4.0초
    assert always(stable) >= 1, '4초 연속인데 놓치는 위상이 있다'


# ── §83.5 재현본: tick 위상과 같은 시각 순서 ───────────────────────────

def test_tick_before_callback_sees_what_scan_first_hides():
    """🔴 재현 — 검출 10장(0.0~0.9) 뒤 빈 장(1.0).

    구판 replay 는 scan 직후만 표본화해 run **0** 을 냈다. 그런데 t=1.0 의 tick 이
    빈 scan 콜백보다 **먼저** 돌면 `visible()==True` 다. 대조군에서 이 차이는
    "mission 이 실제로 잡을 오탐을 0 으로 보고" 하는 것이라 치명적이다."""
    ser = series([True] * 10 + [False] * 3)
    assert R.replay(ser, 0.8, 2.35, 3, phase=0.0, tick_first=False)[0] == 0
    assert R.replay(ser, 0.8, 2.35, 3, phase=0.0, tick_first=True)[0] == 1
    assert worst(ser) == 1, '최악값이 knife-edge 를 못 잡는다'


def test_phase_offset_moves_the_tick_grid_not_the_scans():
    """위상은 **tick 격자**를 밀어야 한다. scan 을 밀면 t0 도 같이 밀려 무효다.

    첫 구현이 정확히 그 실수를 했고 재현으로 잡혔다."""
    ser = series([True] * 10 + [False] * 3)
    got = {R.replay(ser, 0.8, 2.35, 3, phase=p, tick_first=True)[0]
           for p in (0.0, 0.25, 0.5, 0.75)}
    assert len(got) > 1, f'위상을 바꿔도 결과가 그대로다 ({got})'


# ── §83.5: 정렬 기준은 run 개수가 아니라 안정성 ────────────────────────

def test_stable_beats_blinking_on_the_ranking_metric():
    """🔴 재현 — 안정 9초는 run 1, 1.2초 깜빡임은 run 여러 개.

    구판 정렬은 `sum(run)` 내림차순이라 **깜빡임을 "여유가 크다" 며 골랐다.**
    run 은 성공 **횟수**이지 안정성이 아니다. 새 기준은 최소 연속 지속시간이다."""
    stable = series([True] * 90)
    blink = series(([True] * 12 + [False]) * 7)
    assert R.replay(stable, 0.8, 2.35, 3)[0] <= R.replay(blink, 0.8, 2.35, 3)[0], \
        '이 시험의 전제(깜빡임 run 이 더 많다)가 깨졌다'
    assert hold(stable) > hold(blink), '안정 쪽 연속시간이 더 길어야 한다'


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
