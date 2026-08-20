#!/usr/bin/env python3
"""예약 57 판정 — 제자리 선회가 **교착인지 정상인지**를 명령 부호전환으로 가른다.

사용:
    python3 tools/rotation_stall_report.py <bag_경로>

왜 이 도구가 있나
-----------------
08-20 `route_fwd_0820` 이 코너에서 **64초에 2.3 cm** 로 교착했다. 처음엔 불감대
교착(명령이 너무 작아 못 넘음)으로 의심했는데, 실측이 그걸 뒤집었다:

    정상 회전   명령 |ω| 중앙값 0.500   부호전환  0회/10초   →  64초에 96.6°
    🔴 교착     명령 |ω| 중앙값 0.500   부호전환 15회/10초   →  64초에  5.9°

🔴 **명령 크기가 같다.** 다른 것은 **부호가 뒤집히는 횟수**뿐이다. 0.34초마다
방향이 바뀌니 정지마찰을 이길 만큼 한 방향 토크가 안 쌓인다.

그래서 판정 지표는 **크기가 아니라 부호전환 횟수**다. 이 도구가 그걸 센다.
정본 = `docs/PITFALLS.md §6` · `docs/REAL_ROBOT_VALUES.md §1-l-6`.
"""

import math
import sys

from geometry_msgs.msg import Twist
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

WINDOW = 10.0          # 판정 창 [s]
DEADLOCK = 10          # 이 이상이면 교착 (실측 교착 15 vs 정상 3~5)
EPS = 0.02             # 이보다 작은 명령은 '방향 없음' 으로 보고 부호를 안 센다


def read_cmd(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('', ''))
    out = []
    while r.has_next():
        topic, data, t = r.read_next()
        if topic != '/cmd_vel':
            continue
        m = deserialize_message(data, Twist)
        out.append((t / 1e9, m.angular.z, m.linear.x))
    return out


def worst_window(pts):
    """모든 10초 창 중 부호전환이 가장 많은 창을 찾는다."""
    worst = (0, None, None)
    for i, (t0, _, _) in enumerate(pts):
        flips = 0
        prev = 0
        j = i
        while j < len(pts) and pts[j][0] - t0 <= WINDOW:
            w = pts[j][1]
            if abs(w) > EPS:
                s = 1 if w > 0 else -1
                if prev and s != prev:
                    flips += 1
                prev = s
            j += 1
        if flips > worst[0]:
            worst = (flips, t0, pts[j - 1][0] if j > i else t0)
    return worst


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    pts = read_cmd(sys.argv[1])
    if not pts:
        print('🔴 /cmd_vel 이 bag 에 없다 — 판정 불능')
        return 1

    t0 = pts[0][0]
    dur = pts[-1][0] - t0
    rot = [w for _, w, _ in pts if abs(w) > EPS]
    mag = sorted(abs(w) for w in rot)
    med = mag[len(mag) // 2] if mag else 0.0

    flips, ws, we = worst_window(pts)
    print(f'bag 길이 {dur:.1f}s · /cmd_vel {len(pts)}건 · 회전 명령 {len(rot)}건')
    print(f'명령 |ω| 중앙값 = {med:.3f} rad/s')
    print(f'🔴 최악 10초 창 부호전환 = {flips} 회', end='')
    if ws is not None:
        print(f'  (t+{ws - t0:.1f} ~ t+{we - t0:.1f}s)')
    else:
        print()
    print()
    if flips >= DEADLOCK:
        print(f'🔴 교착 — 예약 57 재현이다 ({flips} >= {DEADLOCK}).')
        print('   명령 크기는 정상인데 부호가 뒤집혀 로봇이 제자리에서 떤다.')
        print('   처방 후보: max_angular_accel 10.0 → 4.9 (물리 천장 9.68 이하)')
        print('   또는 목표를 코너가 아니라 직선 구간 안쪽에 둔다.')
        return 1
    print(f'🟢 정상 ({flips} < {DEADLOCK}). 실측 정상 범위는 0~5 회/10초다.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
