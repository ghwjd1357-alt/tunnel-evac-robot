#!/usr/bin/env python3
"""RPP 가 제자리 회전에 얼마나 시간을 쓰는지 잰다 — `rotate_to_heading_min_angle` A/B 용.

사용 (시뮬이 이미 떠 있어야 한다):
    python3 tools/nav2_rotate_probe.py --goal 8.0 --label A

무엇을 재는가
-------------
`rotate_to_heading_min_angle` 을 낮추면 RPP 는 방향 오차가 작을 때도 **선속도를 0 으로
두고 제자리 회전**한다. 실차에서는 그것이 불감대를 넘기는 유일한 길이지만, 대가는
**주행이 끊기는 것**이다. 그 대가의 크기를 숫자로 낸다.

    회전 국면 = |linear.x| < 0.01  AND  |angular.z| > 0.01
    전진 국면 = |linear.x| >= 0.01

🔴 이 도구가 답하지 **못하는** 것
--------------------------------
시뮬에는 **구동 불감대가 없다.** 그래서 이 시험은 *"낮추면 고쳐지는가"* 를 못 본다.
보는 것은 반대쪽, *"낮추면 망가지는가"* 다 — 진동·정체·goal 실패 같은 하방 위험.
실차 효과는 `nav2_params_real_db.yaml` 로 로봇에서 따로 본다.
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener

MAP_FRAME = "map"
BASE_FRAME = "base_footprint"
LIN_EPS = 0.01
ANG_EPS = 0.01


class Probe(Node):
    def __init__(self):
        super().__init__("nav2_rotate_probe")
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.samples = []          # (t, lin, ang)
        self.create_subscription(Twist, "cmd_vel", self._cb, 20)

    def _cb(self, m):
        self.samples.append((time.time(), m.linear.x, m.angular.z))

    def wait_tf(self, timeout=60.0):
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            try:
                tf = self.buf.lookup_transform(MAP_FRAME, BASE_FRAME, rclpy.time.Time())
            except Exception:
                continue
            t, q = tf.transform.translation, tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return t.x, t.y, yaw
        return None


def summarize(samples, label, wall):
    """회전 국면 비율과 에피소드 수. 표본이 없으면 그렇다고 말한다."""
    if not samples:
        print("  🔴 %s: /cmd_vel 표본 0건 — 컨트롤러가 명령을 안 냈다." % label)
        return None
    rot = sum(1 for _, l, a in samples if abs(l) < LIN_EPS and abs(a) > ANG_EPS)
    fwd = sum(1 for _, l, a in samples if abs(l) >= LIN_EPS)
    idle = len(samples) - rot - fwd
    # 회전 에피소드 = 전진→회전 전이 횟수
    eps, prev = 0, False
    for _, l, a in samples:
        cur = abs(l) < LIN_EPS and abs(a) > ANG_EPS
        if cur and not prev:
            eps += 1
        prev = cur
    n = len(samples)
    print("  %-4s 표본 %-5d  회전 %5.1f%%  전진 %5.1f%%  정지 %5.1f%%  "
          "회전 에피소드 %-3d  벽시계 %.1fs"
          % (label, n, 100.0 * rot / n, 100.0 * fwd / n, 100.0 * idle / n, eps, wall))
    return dict(n=n, rot=rot, fwd=fwd, idle=idle, eps=eps, wall=wall)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--goal", type=float, default=8.0, help="정면 거리 [m]")
    ap.add_argument("--left", type=float, default=0.0,
                    help="왼쪽 옆으로 [m] — 초기 방위 오차를 만든다")
    ap.add_argument("--label", default="?")
    ap.add_argument("--abs", nargs=2, type=float, metavar=("X", "Y"),
                    help="map 절대좌표 goal (--goal/--left 를 무시한다)")
    ap.add_argument("--spin", type=float, default=0.0,
                    help="goal 전에 제자리로 이만큼 돌려 방위오차를 만든다 [deg]")
    ap.add_argument("--timeout", type=float, default=180.0)
    args = ap.parse_args(argv)

    rclpy.init()
    node = Probe()
    try:
        pose = node.wait_tf()
        if pose is None:
            print("🔴 %s→%s TF 없음 — 스택이 안 떴다." % (MAP_FRAME, BASE_FRAME))
            return 2
        x, y, yaw = pose
        if args.spin:
            # 🔴 계측 전에 방위오차를 만든다. Nav2 밖에서 직접 돌리므로 이 구간은
            #    samples 에 안 들어간다(아래에서 clear 한다).
            pub = node.create_publisher(Twist, "cmd_vel", 10)
            w = 0.5 if args.spin > 0 else -0.5
            dur = abs(math.radians(args.spin)) / 0.5
            t_end = time.time() + dur
            tw = Twist(); tw.angular.z = w
            while time.time() < t_end:
                pub.publish(tw); rclpy.spin_once(node, timeout_sec=0.05)
            pub.publish(Twist())
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.05)
            x, y, yaw = node.wait_tf(timeout=10.0) or (x, y, yaw)
            print("  제자리 %+.0f° 회전 후 yaw %.1f°" % (args.spin, math.degrees(yaw)))

        if args.abs:
            gx, gy = args.abs
            err = abs(math.degrees(math.atan2(gy - y, gx - x) - yaw))
            err = min(err, 360.0 - err)
        else:
            gx = x + args.goal * math.cos(yaw) - args.left * math.sin(yaw)
            gy = y + args.goal * math.sin(yaw) + args.left * math.cos(yaw)
            err = abs(math.degrees(math.atan2(args.left, args.goal)))
        print("  출발 (%.2f, %.2f) yaw %.1f°  →  목표 (%.2f, %.2f)  초기 방위오차 %.1f°"
              % (x, y, math.degrees(yaw), gx, gy, err))

        if not node.client.wait_for_server(timeout_sec=60.0):
            print("🔴 navigate_to_pose 액션 서버 없음.")
            return 2

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = MAP_FRAME
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        node.samples.clear()
        t0 = time.time()
        fut = node.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            print("🔴 goal 거부됨.")
            return 1

        res_fut = handle.get_result_async()
        # 🔴 유한 상한. 무한 대기는 하네스를 매달아 놓는다 (AGENTS §3-9).
        end = time.time() + args.timeout
        while rclpy.ok() and not res_fut.done() and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        wall = time.time() - t0

        if not res_fut.done():
            print("  ⏱ %.0f초 상한 초과 — 미완주로 센다." % args.timeout)
            status = -1
        else:
            status = res_fut.result().status

        st = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED", -1: "TIMEOUT"}.get(status, str(status))
        print("  결과 %s" % st)
        summarize(node.samples, args.label, wall)
        return 0 if status == 4 else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
