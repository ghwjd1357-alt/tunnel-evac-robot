#!/usr/bin/env python3
"""구동계 10초 점검 — 엔코더가 미끄러지나 · 좌우가 대칭인가.

사용 (🔴 무장돼 있어야 한다):
    python3 tools/drive_health.py            # 4초 회전하며 잰다
    python3 tools/drive_health.py --sec 6    # 더 길게
    python3 tools/drive_health.py --watch    # 안 돌리고 지금 흐르는 값만 본다

왜 이 도구가 있나
-----------------
08-21 실차에서 회전이 하루 종일 느려졌고, 구현자는 그것을 **배터리 탓으로 오진**했다.
`/odom` 으로 쟀기 때문이다. 뒤늦게 `/imu/data` 와 대조해보니 진실이 반대였다:

    M2 14:00   odom 0.1283 ≈ IMU 0.1262   오차  1.6%   🟢 로봇 정상
    20:00      odom 0.0965  vs IMU 0.1456  오차 34%    🔴 **오도메트리가 고장**

즉 **로봇은 느려지지 않았고 오도메트리가 덜 세고 있었다.** 자이로는 바퀴와 물리적으로
무관하므로 둘이 어긋나면 그 사이(엔코더·커플러·축) 어딘가가 미끄러진다는 뜻이다.
분해하지 않고도 알 수 있다 — 그게 이 도구의 존재 이유다.

두 가지를 따로 잰다
-------------------
① **엔코더 미끄러짐**  odom ω vs IMU ω. 자이로가 진짜다.
② **좌우 비대칭**      제자리 회전 명령인데 `odom linear.x` 가 0 이 아니면
                       좌우 바퀴 속도가 다르다. 부호로 어느 쪽이 약한지 갈린다.
   실측: 정상 -0.0002 m/s → 이상 -0.0160 m/s (예약 44 = 우전륜 유격과 같은 자리)

🔴 이 도구는 로봇을 **제자리에서 돌린다.** 반경 1 m 를 비우고 E-stop 을 쥔다.
"""

import argparse
import math
import os
import statistics as st

import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

# 🔴 08-21 — 구동부(micro-ROS)는 BEST_EFFORT 로 발행한다. RELIABLE 구독자는
#   **한 건도 못 받고**, 그게 에러가 아니라 경고 한 줄로 지나간다.
#   같은 함정 = `tools/wall_align.py` · 같은 패턴 = `tools/nav2_preflight.py:48`.
BEST_EFFORT = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)

SELF = os.path.abspath(__file__)
TURN_RATE = 0.45        # rad/s — 회전 불감대(0.2329) 를 확실히 넘는 값
TRACK = 0.49            # m — URDF 의 바퀴 간격 (robot_real.urdf: y=±0.245)
MAX_SEC = 10.0

# 판정선 — 08-21 실측 기준
SLIP_OK = 10.0          # odom/IMU 오차 [%] 이 아래면 정상 (실측 정상 1.6%)
ASYM_OK = 0.005         # |odom linear.x| [m/s] (실측 정상 0.0002 · 이상 0.0160)


def verdict(slip_pct, asym, omega):
    print()
    ok = True
    if abs(slip_pct) <= SLIP_OK:
        print(f'  🟢 엔코더  오차 {slip_pct:+.1f}%  (정상 ±{SLIP_OK:.0f}% 이내)')
    else:
        ok = False
        which = '덜 센다' if slip_pct > 0 else '더 센다'
        print(f'  🔴 엔코더  오차 {slip_pct:+.1f}% — 오도메트리가 회전을 {which}')
        print(f'     → 엔코더·커플러·축 중 어딘가가 미끄러진다. 조임 점검.')
    if abs(asym) <= ASYM_OK:
        print(f'  🟢 좌우    linear.x {asym:+.4f} m/s  (제자리 회전이면 0)')
    else:
        ok = False
        # vl = d - ωL/2 · vr = d + ωL/2 → d·ω 의 부호가 약한 쪽을 가른다
        weak = '오른쪽' if asym * omega < 0 else '왼쪽'
        print(f'  🔴 좌우    linear.x {asym:+.4f} m/s — **{weak}이 약하다**')
        print(f'     → 제자리로 안 돌고 호를 그린다. 예약 44(우전륜 유격) 참조.')
    print()
    print('  🟢 구동계 정상' if ok else '  🔴 조임·결합 점검이 필요하다')
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sec', type=float, default=4.0, help='회전 시간 [s]')
    ap.add_argument('--watch', action='store_true', help='안 돌리고 관측만')
    a = ap.parse_args()
    if a.sec > MAX_SEC:
        print(f'🔴 {a.sec}s 는 너무 길다 (상한 {MAX_SEC}s)')
        return 1

    rclpy.init()
    node = rclpy.create_node('drive_health')
    odo, imu, diag = [], [], []
    node.create_subscription(Odometry, '/odom',
                             lambda m: odo.append((m.twist.twist.linear.x,
                                                   m.twist.twist.angular.z)),
                             BEST_EFFORT)
    node.create_subscription(Imu, '/imu/data',
                             lambda m: imu.append(m.angular_velocity.z), BEST_EFFORT)
    node.create_subscription(Vector3, '/drive/diag',
                             lambda m: diag.append(m.z), BEST_EFFORT)

    def spin(sec):
        end = node.get_clock().now().nanoseconds + int(sec * 1e9)
        while node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    try:
        spin(2.0)
        if not odo or not imu:
            print(f'🔴 /odom {len(odo)}건 · /imu/data {len(imu)}건 — 스택 확인')
            return 1

        if a.watch:
            print('관측만 (로봇을 안 돌린다) — 지금 흐르는 값:')
            spin(3.0)
            ow = st.median([abs(w) for _, w in odo])
            iw = st.median([abs(w) for w in imu])
            print(f'  odom |ω| {ow:.4f}   IMU |ω| {iw:.4f}   linear.x '
                  f'{st.median([v for v, _ in odo]):+.4f}')
            print('  ⚠ 정지 중이면 둘 다 0 이라 판정이 안 된다. 회전이 필요하다.')
            return 0

        if not diag or abs(diag[-1] - 2.0) > 0.1:
            z = diag[-1] if diag else float('nan')
            print(f'🔴 무장 안 됨 (z={z:.1f}, 2.0 이어야 함). 회전하지 않는다.')
            print('   ros2 service call /drive/enable std_srvs/srv/SetBool "{data: true}"')
            return 1

        pub = node.create_publisher(Twist, '/cmd_vel', 10)
        msg = Twist()
        msg.angular.z = TURN_RATE
        print(f'🔴 제자리 회전 {a.sec:.1f}초 · z=+{TURN_RATE} — 반경 1 m 를 비웠는지 확인')
        odo.clear(); imu.clear()
        end = node.get_clock().now().nanoseconds + int(a.sec * 1e9)
        while node.get_clock().now().nanoseconds < end:
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)
        stop = Twist()
        for _ in range(6):
            pub.publish(stop)
            rclpy.spin_once(node, timeout_sec=0.05)

        # 🔴 가속 구간을 버린다 — 앞 30% 는 불감대를 넘느라 둘 다 0 에 가깝다
        cut = max(1, len(odo) // 3)
        ow = st.median([abs(w) for _, w in odo[cut:]]) if len(odo) > cut else 0.0
        iw = st.median([abs(w) for w in imu[len(imu)//3:]]) if imu else 0.0
        asym = st.median([v for v, _ in odo[cut:]]) if len(odo) > cut else 0.0
        print(f'\n  표본  odom {len(odo)}건 · IMU {len(imu)}건 (앞 1/3 버림)')
        print(f'  odom |ω| {ow:.4f} rad/s   IMU |ω| {iw:.4f} rad/s '
              f'= {math.degrees(iw):.2f} °/s')
        slip = 100.0 * (1 - ow / iw) if iw > 1e-6 else 0.0
        return verdict(slip, asym, TURN_RATE)
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
