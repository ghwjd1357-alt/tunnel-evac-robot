#!/usr/bin/env python3
"""로봇을 **실제 벽 기준**으로 정렬한다 — 지도 좌표가 아니라 라이다가 보는 벽이 기준.

사용:
    python3 tools/wall_align.py            # 한 번 재고 끝
    python3 tools/wall_align.py --watch     # 계속 갱신 (회전시키며 보기)

왜 지도 좌표로 안 맞추나
------------------------
`yaw_goal_tolerance` 가 0.25 rad(14°)라 Nav2 목표로는 그 이상 못 맞춘다.
그리고 지도는 SLAM 결과라 실제 벽과 미세하게 틀어져 있을 수 있다.
클러스터 판별처럼 **벽까지 거리가 곧 결과**인 측정에서는 실물 벽이 기준이어야 한다.

무엇을 재는가
-------------
좌우 90° 부근 빔으로 벽에 직선을 맞춘다(최소제곱). 로봇 좌표계에서 벽이
로봇 x축과 평행하면 기울기 0 이다. 기울기의 arctan 이 곧 **틀어진 각도**다.
좌우 두 벽의 평균을 쓴다 — 한쪽이 문·개구부로 끊겨도 덜 흔들린다.
"""

import argparse
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# 벽으로 볼 빔의 각도 범위 [deg] — 정면·후면은 사람·개구부가 섞여 제외한다
LEFT = (55.0, 125.0)
RIGHT = (-125.0, -55.0)
MAX_R = 4.0        # 이보다 먼 점은 벽으로 안 본다 (반대편 복도)


def fit(pts):
    """y = m*x + c 최소제곱. 반환 (기울기, 거리중앙값, 표본수)."""
    n = len(pts)
    if n < 8:
        return None
    sx = sum(p[0] for p in pts); sy = sum(p[1] for p in pts)
    sxx = sum(p[0]*p[0] for p in pts); sxy = sum(p[0]*p[1] for p in pts)
    den = n*sxx - sx*sx
    if abs(den) < 1e-9:
        return None
    m = (n*sxy - sx*sy) / den
    ys = sorted(abs(p[1]) for p in pts)
    return m, ys[n // 2], n


def report(scan):
    left, right = [], []
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r < scan.range_min or r > MAX_R:
            continue
        a = math.degrees(scan.angle_min + i * scan.angle_increment)
        a = (a + 180.0) % 360.0 - 180.0          # -180~180 으로 정규화
        p = (r * math.cos(math.radians(a)), r * math.sin(math.radians(a)))
        if LEFT[0] <= a <= LEFT[1]:
            left.append(p)
        elif RIGHT[0] <= a <= RIGHT[1]:
            right.append(p)

    fl, fr = fit(left), fit(right)
    if fl is None and fr is None:
        print('🔴 좌우 어느 쪽에서도 벽을 못 찾았다 (표본 부족)')
        return
    angs = []
    for name, f in (('좌', fl), ('우', fr)):
        if f is None:
            print(f'  {name}벽  표본 부족')
            continue
        m, d, n = f
        deg = math.degrees(math.atan(m))
        angs.append(deg)
        print(f'  {name}벽  거리 {d:5.2f} m · 기울기 {deg:+6.2f}° · 점 {n:3d}개')

    err = sum(angs) / len(angs)
    print(f'\n🔴 로봇이 벽에서 {err:+.2f}° 틀어져 있다')
    if abs(err) < 1.0:
        print('🟢 정렬됨 (±1° 이내)')
        return
    # 실측 회전율 약 8°/s (명령 0.45 rad/s) 기준 필요 시간
    sec = abs(err) / 8.0
    turn = '반시계(z 양수)' if err < 0 else '시계(z 음수)'
    sign = '' if err < 0 else '-'
    print(f'→ {turn} 로 약 {sec:.1f}초 돌린다:')
    print(f'   M=\'{{angular: {{z: {sign}0.45}}}}\'')
    print(f'   timeout {max(sec,0.3):.1f} ros2 topic pub -r 10 /cmd_vel '
          f'geometry_msgs/msg/Twist "$M"')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--watch', action='store_true', help='계속 갱신')
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('wall_align')
    got = []
    node.create_subscription(LaserScan, '/scan', lambda m: got.append(m), 10)

    try:
        while True:
            got.clear()
            end = node.get_clock().now().nanoseconds + 5_000_000_000
            while not got and node.get_clock().now().nanoseconds < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            if not got:
                print('🔴 /scan 이 5초간 없다 — 라이다 확인')
                return 1
            print('=' * 46)
            report(got[-1])
            if not a.watch:
                return 0
            end = node.get_clock().now().nanoseconds + 1_500_000_000
            while node.get_clock().now().nanoseconds < end:
                rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
