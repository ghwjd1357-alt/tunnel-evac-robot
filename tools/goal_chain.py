#!/usr/bin/env python3
"""goal 을 **짧게 여러 번** 보내 긴 거리를 간다 — 큰 방위오차가 쌓이기 전에 목표를 갱신한다.

사용 (🔴 무장·스택은 사람이 먼저 세운다):
    python3 tools/goal_chain.py --step 1.0 --count 12          # 1m 씩 12번 = 약 12m
    python3 tools/goal_chain.py --step 1.5 --count 8 --label C2
    python3 tools/goal_chain.py --step 1.0 --count 12 --dry     # 계획만 인쇄

왜 이 도구가 있나
-----------------
`MASTER_PLAN §7` 예약 40 의 세 번째 보완안이다:

    "③ 목표를 1m 단위로 쪼갠다 (**코드 변경 0** · 08-18 실증)"

08-19 가 밝힌 교착의 모양은 *"방위오차가 14~34° 로 커지면 명령이 불감대에 갇힌다"* 였다.
목표를 짧게 끊으면 **오차가 그 구간까지 자라기 전에** 목표가 앞으로 옮겨간다. 파라미터도
펌웨어도 안 건드리는 우회이며, 분기 1️⃣(`max_angular_accel`)이 실패했을 때의 다음 수다.

🔴 이 도구가 하지 않는 것
------------------------
- **무장하지 않는다.** 사람이 `/drive/enable` 로 한다.
- **성공을 판정하지 않는다.** 각 구간의 status 와 누적 이동을 인쇄할 뿐이다.
- **경로를 계획하지 않는다.** 매 구간 "지금 보고 있는 방향 앞 N미터"다 — 벽이 있어도 쏜다.
  🔴 그래서 **직선 통로에서 쓴다.** 곡선 경로가 필요하면 `--abs-list` 로 좌표를 준다.

⚠ 실행 전 확인 (사용자 상시 규칙)
    ros2 topic echo /drive/enabled --qos-reliability best_effort --once   # data: true
    ros2 topic echo /drive/diag    --qos-reliability best_effort --once   # z: 2.0 ARMED
    ros2 topic echo /estop/state   --qos-reliability best_effort --once   # data: false
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener

MAP_FRAME = "map"
BASE_FRAME = "base_footprint"
STATUS = {2: "CANCELED", 4: "SUCCEEDED", 5: "ABORTED", 6: "CANCELED"}


class Chain(Node):
    def __init__(self):
        super().__init__("goal_chain")
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def pose(self, timeout=10.0):
        end = self.get_clock().now() + Duration(seconds=timeout)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self.buf.lookup_transform(MAP_FRAME, BASE_FRAME, rclpy.time.Time())
            except Exception:
                continue
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return t.x, t.y, yaw
        return None

    def send(self, gx, gy, gyaw, timeout):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = MAP_FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation.z = math.sin(gyaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(gyaw / 2.0)
        fut = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return None
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=timeout)
        out = res.result()
        return None if out is None else out.status


def main(argv=None):
    ap = argparse.ArgumentParser(description="짧은 goal 을 연쇄로 보낸다")
    ap.add_argument("--step", type=float, default=1.0, help="한 구간 전진 거리 [m]")
    ap.add_argument("--count", type=int, default=12, help="구간 수")
    ap.add_argument("--left", type=float, default=0.0, help="구간마다 왼쪽 offset [m]")
    ap.add_argument("--abs-list", default=None,
                    help='절대좌표 열 "x,y x,y ..." (--step/--count 를 무시한다)')
    ap.add_argument("--timeout", type=float, default=90.0, help="구간당 상한 [s]")
    ap.add_argument("--label", default="?")
    ap.add_argument("--dry", action="store_true")
    args = ap.parse_args(argv)

    legs = None
    if args.abs_list:
        legs = [tuple(float(v) for v in p.split(",")) for p in args.abs_list.split()]

    print("=" * 68)
    print("  연쇄 goal   label=%s" % args.label)
    if legs:
        print("  절대좌표 %d 구간" % len(legs))
    else:
        print("  %d 구간 × %.2f m = 약 %.1f m  (구간별 왼쪽 offset %.2f m)"
              % (args.count, args.step, args.count * args.step, args.left))
    print("  🔴 직선 통로 전제 — 매 구간 '지금 보는 방향 앞'이라 벽이 있어도 쏜다.")
    print("  🔴 물리 E-stop 담당자 상시.")
    print("=" * 68)
    if args.dry:
        print("  --dry 라 보내지 않는다.")
        return 0

    rclpy.init()
    node = Chain()
    try:
        start = node.pose()
        if start is None:
            print("🔴 %s→%s TF 없음 — 스택이 안 떴거나 위치추정이 안 섰다."
                  % (MAP_FRAME, BASE_FRAME))
            return 2
        if not node.client.wait_for_server(timeout_sec=30.0):
            print("🔴 navigate_to_pose 액션 서버 없음 — Nav2 가 ACTIVE 인지 본다.")
            return 2

        x0, y0, _ = start
        n = len(legs) if legs else args.count
        t_start = time.time()
        done = 0
        for i in range(n):
            cur = node.pose()
            if cur is None:
                print("  🔴 구간 %d: TF 를 잃었다 — 중단." % (i + 1))
                break
            x, y, yaw = cur
            if legs:
                gx, gy = legs[i]
                gyaw = math.atan2(gy - y, gx - x)
            else:
                gx = x + args.step * math.cos(yaw) - args.left * math.sin(yaw)
                gy = y + args.step * math.sin(yaw) + args.left * math.cos(yaw)
                gyaw = yaw
            t0 = time.time()
            status = node.send(gx, gy, gyaw, args.timeout)
            dt = time.time() - t0
            after = node.pose() or (x, y, yaw)
            moved = math.hypot(after[0] - x, after[1] - y)
            total = math.hypot(after[0] - x0, after[1] - y0)
            name = STATUS.get(status, "거부/무응답" if status is None else str(status))
            mark = "🟢" if status == 4 else "🔴"
            print("  %s 구간 %2d/%d  →(%.2f, %.2f)  %-12s %5.1fs  구간 %.2fm  누적 %.2fm"
                  % (mark, i + 1, n, gx, gy, name, dt, moved, total))
            if status == 4:
                done += 1
            else:
                print("     🔴 여기서 멈춘다 — 다음 구간을 보내지 않는다.")
                break

        end = node.pose() or start
        total = math.hypot(end[0] - x0, end[1] - y0)
        print("\n" + "=" * 68)
        print("  완료 구간 %d/%d · 누적 이동 %.2f m · 총 %.0f초"
              % (done, n, total, time.time() - t_start))
        print("  🔴 판정은 이 출력이 아니라 bag 이다 — tools/nav2_trial_report.py 로 읽는다.")
        print("=" * 68)
        return 0 if done == n else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
