#!/usr/bin/env python3
"""주행 중 구동계 감시 — **재발을 테이크 안에서 잡는다** (2026-08-22 신설).

사용 (로봇을 움직이지 않는다 · 보기만 한다):
    python3 tools/drive_watch.py

🔴 **왜 이 도구가 있나** — 08-21 21:32 에 우측 합성 계측이 절반으로 떨어졌고,
08-22 낮에 **아무것도 안 건드렸는데 증상이 사라졌다.** 죽은 채널은 스스로 살아나지
않으므로 **간헐 고장이고, 재발을 전제해야 한다.**

`drive_health.py --straight` 는 **테이크 전** 10초 점검이다. 그런데 고장이 테이크
**중간에** 재발하면 그 테이크는 이미 망가진 채로 끝나고, 편집 단계에서야 회전이
어설픈 것을 발견한다. 🔴 **재촬영이 없는 일정에서 그건 되돌릴 수 없다.**

★ **지문** — 직진 명령만 주는데 `/odom` 이 "휜다" 고 말하면 좌우가 다르게 읽히는 것이다.

    r = odomω · BASE / (2 · odom속도) = (kR − kL)/(kR + kL)      ← v 가 지워진다

정지·회전 구간은 건너뛴다(그때는 이 식이 성립하지 않는다). 정본 =
`REAL_ROBOT_VALUES §1-m-11` · 짝 도구 = `drive_health.py`(전 점검) ·
`bag_drive_report.py`(사후 판독).
"""
import argparse
import os
import statistics as st

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

# 🔴 구동부(micro-ROS)는 BEST_EFFORT 로 발행한다. RELIABLE 구독자는 한 건도 못 받고
#   그것이 에러가 아니라 경고 한 줄로 지나간다 (`PITFALLS §17`).
BEST_EFFORT = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
SELF = os.path.abspath(__file__)
ODOM_WHEEL_BASE = 0.829     # `.ino` ODOM_WHEEL_BASE — 회귀가 대조한다
BAL_OK = 0.05               # |r| 이 이 아래면 정상
MIN_SPEED = 0.02            # 이보다 느리면 판정하지 않는다


def verdict(lin, ow, base=ODOM_WHEEL_BASE):
    """직진 표본에서 (r, 약한 쪽). 판정 불가면 (None, None)."""
    if lin is None or lin < MIN_SPEED:
        return None, None
    r = ow * base / (2.0 * lin)
    if abs(r) <= BAL_OK:
        return r, None
    return r, ('오른쪽' if r < 0 else '왼쪽')


class Watch:
    """순수 상태기계 — ROS 없이 회귀할 수 있게 갈라 뒀다."""

    def __init__(self, window_sec=2.0):
        self.window = window_sec
        self.buf = []            # (t, lin, w) — 직진 명령 중인 표본만
        self.alerts = 0

    def feed(self, t, cmd_v, cmd_w, odom_lin, odom_w):
        """반환: 경고 문자열 또는 None."""
        straight = cmd_v > 0.05 and abs(cmd_w) < 0.02
        if not straight:
            self.buf.clear()     # 회전·정지가 섞이면 창을 버린다
            return None
        self.buf.append((t, odom_lin, odom_w))
        self.buf = [b for b in self.buf if t - b[0] <= self.window]
        if len(self.buf) < 10 or t - self.buf[0][0] < self.window * 0.8:
            return None
        lin = st.median([b[1] for b in self.buf])
        ow = st.median([b[2] for b in self.buf])
        r, weak = verdict(lin, ow)
        if weak is None:
            return None
        self.alerts += 1
        k = (1.0 - abs(r)) / (1.0 + abs(r))
        return (f'🔴 구동계 이상 재발 — {weak}이 {k:.2f} 배로 읽힌다 '
                f'(r={r:+.3f} · odom {lin:.3f} m/s · ω {ow:+.3f})')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--window', type=float, default=2.0, help='판정 창 [s]')
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('drive_watch')
    w = Watch(a.window)
    cmd = {'v': 0.0, 'w': 0.0}
    node.create_subscription(Twist, '/cmd_vel',
                             lambda m: cmd.update(v=m.linear.x, w=m.angular.z), 10)

    def on_odom(m):
        t = node.get_clock().now().nanoseconds / 1e9
        msg = w.feed(t, cmd['v'], cmd['w'],
                     m.twist.twist.linear.x, m.twist.twist.angular.z)
        if msg:
            print('\n' + '=' * 62)
            print('  ' + msg)
            print('  🔴 이 테이크는 다시 찍는다. 편집에서 알면 늦다.')
            print(f'  → 멈추고: python3 {SELF.replace("drive_watch", "drive_health")}'
                  f' --straight 8')
            print('=' * 62, flush=True)

    node.create_subscription(Odometry, '/odom', on_odom, BEST_EFFORT)
    print(f'주행 감시 시작 — 직진 {a.window:.1f}초 창으로 좌우 균형을 본다.')
    print('🔵 로봇을 움직이지 않는다. 보기만 한다. (Ctrl+C 로 종료)')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f'\n종료 — 경고 {w.alerts}회')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
