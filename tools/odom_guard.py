#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""odom_guard.py — 찢어진 `/odom` 메시지가 EKF 를 죽이지 못하게 막는다.

[왜 있는가 — 2026-08-14 실측]
  지도 세션 두 번이 같은 이유로 무너졌다. `/odom` 이 가끔 **부분만 쓰인 메시지**를 낸다:

      정상        twist cov 대각 = [0.02,  0.02,  0, 0, 0,  0.1 ]
      1회차 12건                 = [0.02,  4.89e-295,  1.74e-60, 5.26e-315, 5.26e-315,  0.0]
      2회차  2건                 = [4.89e-295,  3.69e+53,  ... ,  0.0]

  🔴 **첫 칸만 맞고 그 뒤가 초기화 안 된 메모리다.** 12건 중 11건에서 쓰레기 값이
  `4.89397200e-295` 로 **완전히 동일**하다 — 같은 stale 메모리를 읽었다는 뜻이고,
  메시지 구조체를 채우는 도중에 직렬화가 일어난 **torn write** 로 보인다.

  robot_localization 은 이걸 그대로 먹는다. 공분산이 0 이거나 0 에 가까우면 "이 측정은
  무한히 확실하다" 가 되어 갱신 행렬이 무너지고 **필터 전체가 NaN 이 된다.**
  🔴 그리고 **회복 경로가 없다** — 한 번 NaN 이면 세션이 끝날 때까지 NaN 을 발행한다.

      1회차: +130.275s 에 첫 깨진 메시지 → +130.296s NaN → 21분의 89.5% 가 NaN
      2회차: +299.527s                  → +299.531s NaN → 세션의 46.9% 가 NaN

  before/after 로 증명했다 — 같은 bag 에서 **그 12건만 빼면 NaM 이 안 난다**
  (오프라인 재현. `MASTER_PLAN §7` 예약 41).

[무엇을 하는가]
  `/odom` 을 구독해 **정상 범위의 메시지만** `/odom_guarded` 로 다시 발행한다.
  EKF 는 `/odom_guarded` 를 읽는다(`ekf_real.yaml` `odom0`).

[🔴 무엇을 하지 않는가 — 숨기지 않는다]
  · **원인을 없애지 않는다.** 찢어진 메시지는 계속 나온다. 뿌리는 펌웨어/전송이고
    그건 재굽기(Tier A)라 별도 묶음이다. 이건 **방어**다.
  · **값을 고치지 않는다.** 의심스러우면 버릴 뿐 보간하지 않는다 — 지어낸 측정은
    없는 측정보다 나쁘다. 50Hz 에서 한둘 빠지는 것은 EKF 가 예측으로 덮는다.
  · **조용히 버리지 않는다.** 버린 건수를 1Hz 로 찍고 처음 5건은 값까지 남긴다.
    "가드가 있으니 괜찮다" 가 "얼마나 나빠지고 있는지 모른다" 가 되면 안 된다.

[판정 기준 — 관측이 정했다]
  EKF 가 실제로 읽는 자리(`odom0_config` 의 vx·vy·vyaw)만 본다. 6x6 행 우선이므로
  대각은 인덱스 0·7·14·21·28·35 이고 그중 **0(vx)·7(vy)·35(vyaw)** 이 융합된다.
  🔴 **어느 패턴이 치명적인지 맞히려 하지 않는다** — 2026-08-14 2회차에서 깨진 둘 중
  하나는 EKF 를 죽였고 하나는 안 죽였다. **정상 범위 밖은 전부 버린다.**
"""
import argparse
import math
import sys

# 6x6 행 우선 공분산에서 EKF 가 융합하는 자리 (vx, vy, vyaw)
FUSED_COV_IDX = (0, 7, 35)

# 정상값은 0.02 · 0.02 · 0.1 이다. 아래 창은 그보다 훨씬 넓게 잡았다 —
# 관측된 쓰레기(4.89e-295 · 3.69e+53 · 0.0)와 정상값 사이가 워낙 멀어서
# 창을 좁힐 이유가 없다. 좁히면 정상값의 미래 변경을 잡아 거짓 폐기를 만든다.
COV_MIN = 1e-9
COV_MAX = 1e6

# `.ino` MAX_LINEAR_CMD 의 2배. 🔴 08-22 상한이 0.12 -> 0.20 으로 올라가 같이 옮긴다
# (0.25 -> 0.40). 관측된 쓰레기 vx(2.42 · 3.33)와는 여전히 한참 떨어져 있다.
VX_ABS_MAX = 0.40


def check(cov, vx, vy, wz):
    """정상이면 None, 아니면 버리는 사유 문자열."""
    for i in FUSED_COV_IDX:
        c = cov[i]
        if not math.isfinite(c):
            return f'cov[{i}] 이 유한하지 않다 ({c})'
        if c <= 0.0:
            return f'cov[{i}] = {c!r} — 0 이하는 "무한히 확실" 이라 행렬을 무너뜨린다'
        if c < COV_MIN:
            return f'cov[{i}] = {c:.3e} < {COV_MIN:g} (사실상 0)'
        if c > COV_MAX:
            return f'cov[{i}] = {c:.3e} > {COV_MAX:g}'
    for name, v in (('vx', vx), ('vy', vy), ('wz', wz)):
        if not math.isfinite(v):
            return f'{name} 이 유한하지 않다 ({v})'
    if abs(vx) > VX_ABS_MAX:
        return f'vx = {vx:.4f} — 물리 상한(MAX_LINEAR_CMD 0.20)의 2배를 넘는다'
    return None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--in-topic', default='/odom')
    ap.add_argument('--out-topic', default='/odom_guarded')
    ap.add_argument('--report-sec', type=float, default=1.0)
    args = ap.parse_args(argv)

    import rclpy                                            # noqa: PLC0415
    from nav_msgs.msg import Odometry                       # noqa: PLC0415
    from rclpy.node import Node                             # noqa: PLC0415
    from rclpy.qos import QoSProfile, ReliabilityPolicy     # noqa: PLC0415

    class Guard(Node):
        def __init__(self):
            super().__init__('odom_guard')
            # 🔴 EKF 의 `/odom` 구독은 BEST_EFFORT 다(`ekf_real.yaml` 머리말 실측).
            #   BEST_EFFORT 퍼블리셔 + RELIABLE 구독자는 DDS 가 아예 매칭하지 않으므로
            #   입출력 양쪽을 BEST_EFFORT 로 맞춘다.
            qos = QoSProfile(depth=50)
            qos.reliability = ReliabilityPolicy.BEST_EFFORT
            self.pub = self.create_publisher(Odometry, args.out_topic, qos)
            self.create_subscription(Odometry, args.in_topic, self.cb, qos)
            self.passed = 0
            self.dropped = 0
            self.shown = 0
            self.create_timer(args.report_sec, self.report)
            self.get_logger().info(
                f'{args.in_topic} → {args.out_topic}  '
                f'(cov 창 {COV_MIN:g}~{COV_MAX:g} · |vx| ≤ {VX_ABS_MAX})')

        def cb(self, m):
            why = check(m.twist.covariance,
                        m.twist.twist.linear.x, m.twist.twist.linear.y,
                        m.twist.twist.angular.z)
            if why is None:
                self.passed += 1
                self.pub.publish(m)
                return
            self.dropped += 1
            # 처음 몇 건은 값까지 남긴다 — 나중에 "무엇이 깨졌나"를 되짚을 수 있어야 한다.
            if self.shown < 5:
                self.shown += 1
                self.get_logger().warn(
                    f'🔴 버림 #{self.dropped}: {why}  '
                    f'[vx={m.twist.twist.linear.x:+.4f} '
                    f'wz={m.twist.twist.angular.z:+.5f} '
                    f'stamp={m.header.stamp.sec}.{m.header.stamp.nanosec:09d}]')

        def report(self):
            # 🔴 0 건이어도 찍는다 — "가드가 살아 있다"는 것 자체가 관측값이다.
            self.get_logger().info(
                f'통과 {self.passed} · 버림 {self.dropped}', throttle_duration_sec=0)

    rclpy.init()
    node = Guard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'종료 — 통과 {node.passed} · 버림 {node.dropped}')
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
