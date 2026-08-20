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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--range', type=float, default=2.5,
                    help='검출 범위 [m] — 이 안의 점만 본다')
    a = ap.parse_args()

    clock = _Clock()
    mon = FollowerMonitor(clock, max_range=a.range)

    r = SequentialReader()
    r.open(StorageOptions(uri=a.bag, storage_id='sqlite3'), ConverterOptions('', ''))

    rows = []       # (평균거리, 폭, 점수)
    scans = 0
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
    print('🔴 읽는 법 — 사람이 선 거리대의 폭 중앙값보다 **약간 크게** 잡는다.')
    print('   그보다 넓은 덩어리(벽)가 같은 거리대에 섞여 있으면 두 분포가 겹친다는')
    print('   뜻이고, 그때는 cluster_max_width 로 못 가른다 → detect_range 를 줄인다.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
