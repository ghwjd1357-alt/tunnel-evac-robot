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
from geometry_msgs.msg import Twist, Vector3
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
    got = {}
    for name, f in (('좌', fl), ('우', fr)):
        if f is None:
            print(f'  {name}벽  표본 부족')
            continue
        m, d, n = f
        deg = math.degrees(math.atan(m))
        got[name] = deg
        print(f'  {name}벽  거리 {d:5.2f} m · 기울기 {deg:+6.2f}° · 점 {n:3d}개')

    # 🔴 08-21 실차 — 부호를 한 번 뒤집어 놨었다. 근거를 남긴다.
    #   복도축을 월드 X 로 두고 로봇 yaw 를 θ(반시계 +) 라 하면, 벽 위의 점은
    #   월드 (X, C) 이고 로봇 좌표로는
    #       x_r =  X·cosθ + C·sinθ ,  y_r = -X·sinθ + C·cosθ
    #   → dy_r/dx_r = -tanθ.  즉 **기울기 deg = -θ** 다.
    #   틀어진 각 θ 를 0 으로 만들려면 -θ = +deg 만큼 돌린다.
    #   ⇒ deg 가 양수면 **반시계(z 양수)**. (검증 = cluster_none_pass_0821)
    def advise(deg, tag=''):
        if abs(deg) < 1.0:
            print(f'🟢 {tag}정렬됨 (±1° 이내)')
            return
        sec = max(abs(deg) / 8.0, 0.3)
        ccw = deg > 0
        turn = '반시계(z 양수)' if ccw else '시계(z 음수)'
        sign = '' if ccw else '-'
        print(f'→ {tag}{turn} 로 약 {sec:.1f}초:')
        print(f"   M='{{angular: {{z: {sign}0.45}}}}'")
        print(f'   timeout {sec:.1f} ros2 topic pub -r 10 /cmd_vel '
              f'geometry_msgs/msg/Twist "$M"')
        print('   🔴 위 두 줄을 순서대로. M= 을 먼저 안 치면 빈 메시지가 나간다.')

    if len(got) == 2:
        gap = abs(got['좌'] - got['우'])
        if gap > 3.0:
            print(f'\n🔴 좌우 벽이 {gap:.1f}° 어긋난다 — 평균은 양쪽 다 아닌 값이다.')
            print('   한쪽이 곧은 벽이 아니다(문·기둥·벽감, 벽 근처 사람·장비).')
            print('   🔵 깨끗한 쪽 하나만 골라 그 값으로 맞춰라. 둘 다 인쇄한다:')
            for name in ('좌', '우'):
                print(f'\n[{name}벽 기준 {got[name]:+.2f}°]')
                advise(got[name], tag='')
            return
        if gap > 1.5:
            print(f'\n⚠ 좌우가 {gap:.1f}° 어긋난다 — 평균이 그만큼 흔들린다.')

    err = sum(got.values()) / len(got)
    print(f'\n🔴 로봇이 벽에서 {err:+.2f}° 틀어져 있다')
    advise(err)


TURN_RATE = 0.45        # rad/s 명령 (회전 불감대 0.2329 를 확실히 넘는 값)
DEG_PER_SEC = 8.0       # 그때 실측 회전율 (08-21 M2: 7.5~8.1 °/s)
MAX_TURN = 45.0         # 안전 상한 — 이보다 크게 돌리려면 여러 번 나눠 친다


def turn(node, deg):
    """제자리 회전. 🔴 무장(z=2.0) 이 아니면 거부한다 — 명령만 나가고 안 도는
    상태가 가장 헷갈린다(에러도 안 난다)."""
    if abs(deg) > MAX_TURN:
        print(f'🔴 {deg:+.1f}° 는 한 번에 너무 크다 (상한 {MAX_TURN}°). 나눠서 돌려라.')
        return 1

    diag = []
    node.create_subscription(Vector3, '/drive/diag', lambda m: diag.append(m.z), 10)
    end = node.get_clock().now().nanoseconds + 3_000_000_000
    while not diag and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not diag:
        print('🔴 /drive/diag 가 3초간 없다 — 구동부 확인')
        return 1
    if abs(diag[-1] - 2.0) > 0.1:
        print(f'🔴 무장 안 됨 (z={diag[-1]:.1f}, 2.0 이어야 함). 회전하지 않는다.')
        print('   ros2 service call /drive/enable std_srvs/srv/SetBool "{data: true}"')
        return 1

    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    msg = Twist()
    msg.angular.z = TURN_RATE if deg > 0 else -TURN_RATE
    sec = abs(deg) / DEG_PER_SEC
    print(f'🔴 {"반시계" if deg > 0 else "시계"} {abs(deg):.1f}° · '
          f'{sec:.1f}초 · z={msg.angular.z:+.2f}  — 시작')
    end = node.get_clock().now().nanoseconds + int(sec * 1e9)
    while node.get_clock().now().nanoseconds < end:
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
    stop = Twist()
    for _ in range(5):
        pub.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.05)
    print('🟢 정지. 다시 재라: python3 tools/wall_align.py')
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--watch', action='store_true', help='계속 갱신')
    ap.add_argument('--turn', type=float, metavar='DEG',
                    help='제자리 회전 [도]. 양수=반시계. 무장 상태만 실행된다')
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('wall_align')
    if a.turn is not None:
        try:
            return turn(node, a.turn)
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
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
