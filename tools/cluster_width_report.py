#!/usr/bin/env python3
"""사람 점뭉치 폭 실측 — `cluster_max_width` · `detect_range` 를 정하기 위한 것.

사용:
    python3 tools/cluster_width_report.py <bag_경로> [--range 2.5]

왜 이 도구가 있나
-----------------
`search_back.cluster_max_width: 0.8` 은 **폭 6 m 시뮬 터널** 값이다. 실복도는
**2.35 m**(반폭 1.18 m)라, 검출 범위 안에 벽이 들어온다. 벽 조각이 0.8 m 밑으로
잘리면 **벽이 사람으로** 잡히고, 반대로 사람이 벽과 한 덩어리가 되면
**사람이 벽으로 배제**된다. 둘 다 `SEARCH_BACK` 장면을 깬다.

🔴 **판정기와 같은 알고리즘을 써야 의미가 있다.** 그래서 이 도구는 자기 나름의
클러스터링을 새로 짜지 않고 `FollowerMonitor._find_clusters` 를 **그대로 불러
쓴다.** 도구가 따로 구현하면 "도구에선 되는데 로봇에선 안 되는" 자리가 생긴다.
"""

import argparse
import math
import statistics
import sys

sys.path.insert(0, 'src/mission_manager')

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import LaserScan

from mission_manager.follower_monitor import FollowerMonitor


class _Clock:
    """FollowerMonitor 가 요구하는 최소 시계 — bag 재생이라 실시계를 안 쓴다."""
    class _T:
        def __init__(self, ns):
            self.nanoseconds = ns

    def __init__(self):
        self.ns = 0

    def now(self):
        return _Clock._T(self.ns)


def collect(bag, max_range):
    """bag 의 /scan 전량에서 클러스터를 뽑는다. 반환 (rows, scans)."""
    clock = _Clock()
    mon = FollowerMonitor(clock, max_range=max_range)
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('', ''))
    rows, scans = [], 0
    while r.has_next():
        topic, data, t = r.read_next()
        if topic != '/scan':
            continue
        scans += 1
        clock.ns = t
        scan = deserialize_message(data, LaserScan)
        for c in mon._find_clusters(scan):
            mean_r = sum(rr for _, rr in c) / len(c)
            span = (c[-1][0] - c[0][0]) * scan.angle_increment
            width = mean_r * span + 0.05          # 판정기와 동일 수식
            rows.append((mean_r, width, len(c)))
    return rows, scans


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--range', type=float, default=2.5,
                    help='검출 범위 [m] — 이 안의 점만 본다')
    ap.add_argument('--compare', metavar='EMPTY_BAG',
                    help='🔴 사람 **없는** 대조군 bag. 주면 두 분포를 대조해 '
                         'cluster_max_width 를 추천한다')
    a = ap.parse_args()

    rows, scans = collect(a.bag, a.range)

    if not rows:
        print(f'🔴 클러스터 0개 — /scan 이 없거나 {a.range} m 안에 아무것도 없다')
        return 1

    print(f'스캔 {scans}개 · 클러스터 {len(rows)}개 (검출 범위 {a.range} m)\n')
    print(f'{"거리대(m)":>10} {"개수":>6} {"폭 중앙값":>10} {"폭 p90":>8} {"폭 최대":>8}')
    bands = [(0.0, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 2.5), (2.5, 99.0)]
    for lo, hi in bands:
        sel = [w for d, w, _ in rows if lo <= d < hi]
        if not sel:
            continue
        sel.sort()
        p90 = sel[int(len(sel) * 0.9)] if len(sel) > 1 else sel[0]
        print(f'{lo:.1f}~{hi:<5.1f} {len(sel):>6} {statistics.median(sel):>10.3f} '
              f'{p90:>8.3f} {max(sel):>8.3f}')

    widths = sorted(w for _, w, _ in rows)
    print()
    for th in (0.5, 0.6, 0.8, 1.0, 1.2):
        keep = sum(1 for w in widths if w <= th)
        print(f'  cluster_max_width={th:.1f} → 통과 {keep}/{len(widths)} '
              f'({keep / len(widths) * 100:.0f}%)')
    print()
    if not a.compare:
        print('🔴 읽는 법 — 사람이 선 거리대의 폭 중앙값보다 **약간 크게** 잡는다.')
        print('   ⚠ 이 표만으로는 문턱을 못 정한다. 같은 자리에서 **사람 없는**')
        print('     대조군 bag 을 찍어 `--compare` 로 대조할 것.')
        return 0

    return compare(rows, scans, a.compare, a.range)


def compare(rows, scans, empty_bag, max_range):
    """사람 있는 bag ↔ 없는 bag 을 대조해 문턱을 추천한다.

    🔴 왜 대조가 필요한가 — 사람 분포만 알면 문턱을 못 정한다. 08-21 실측에서
    **사람이 없는 bag 도 클러스터의 27~30% 가 0.8 문턱을 통과**했다(전부 벽 조각).
    그리고 `detect_range` 를 2.5→1.2 로 줄여도 그 비율이 안 줄었다.

    지표: `여유(t) = 사람bag 스캔당 통과 − 대조군 스캔당 통과`
      · 1.0 근처면 그 문턱에서 **사람 한 명이 정확히 더 보인다**
      · 0 근처면 사람이 벽 잡음에 묻혀 **구분이 안 된다**
    그리고 `대조군 스캔당 통과` 가 곧 **오탐률**이다 — 1.0 을 넘으면 매 스캔마다
    벽이 사람 하나로 보인다는 뜻이라 `visible()` 이 잠길 수 있다.
    """
    erows, escans = collect(empty_bag, max_range)
    if not erows:
        print(f'🔴 대조군 bag 에 /scan 이 없다: {empty_bag}')
        return 1

    print(f'\n대조군: {empty_bag} — 스캔 {escans}개 · 클러스터 {len(erows)}개\n')
    print(f'{"문턱":>6} {"사람bag/스캔":>12} {"대조군/스캔":>11} {"여유":>7}  판정')
    best = None
    for t in (0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.2):
        pp = sum(1 for _, w, _ in rows if w <= t) / scans
        ep = sum(1 for _, w, _ in erows if w <= t) / escans
        gain = pp - ep
        mark = ''
        if gain >= 0.8 and ep < 1.0:
            mark = '🟢 사람이 구분된다'
            if best is None or ep < best[2]:
                best = (t, gain, ep)
        elif gain >= 0.8:
            mark = '🔶 구분은 되나 오탐이 잦다'
        elif ep >= 1.0:
            mark = '🔴 벽이 매 스캔 사람으로 보인다'
        else:
            mark = '🔴 사람이 안 구분된다'
        print(f'{t:>6.1f} {pp:>12.2f} {ep:>11.2f} {gain:>7.2f}  {mark}')

    print()
    if best:
        print(f'🟢 추천 cluster_max_width = {best[0]:.1f} '
              f'(여유 {best[1]:.2f} · 오탐 {best[2]:.2f}/스캔)')
        print('   → waypoints_real_H.yaml 의 search_back.cluster_max_width 에 넣는다.')
        return 0
    print('🔴 어느 문턱에서도 사람이 깨끗이 구분되지 않는다.')
    print('   ① min_points 를 올려 본다 (노이즈 조각 배제)')
    print('   ② 대본에서 사람이 멈추는 자리를 벽에서 더 먼 곳으로 옮긴다')
    print('   ③ 그래도 안 되면 SEARCH_BACK 장면은 도박이다 — 감수 여부를 사람이 정한다')
    return 1


if __name__ == '__main__':
    sys.exit(main())
