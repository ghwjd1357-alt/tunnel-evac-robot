#!/usr/bin/env python3
"""현재 로봇 위치 기준 **정면 N미터** 앞으로 Nav2 goal 을 쏜다.

사용:
    python3 tools/goal_ahead.py 3.0            # 정면 3m
    python3 tools/goal_ahead.py 10.0           # 정면 10m
    python3 tools/goal_ahead.py 3.0 --dry      # 좌표만 계산하고 보내지 않는다
    python3 tools/goal_ahead.py 3.0 --left 0.5 # 정면 3m · 왼쪽 0.5m

왜 이 도구가 필요한가
---------------------
08-18 현장에서 goal 을 손으로 계산해 넣다가 `<현재x+2.0>` 같은 문자열이 그대로
들어가 `could not convert string to float` 로 죽었다. 사람이 TF 를 읽고 삼각함수를
암산해서 YAML 에 옮겨 적는 절차 자체가 실패 지점이었다.

이 도구는 그 세 단계(TF 읽기 → 좌표 계산 → goal 전송)를 한 번에 한다.

🔴 이 도구가 하지 않는 것
------------------------
- **무장하지 않는다.** 무장은 사람이 `/drive/enable` 로 한다. 도구가 로봇을 무장시키면
  "내가 언제 무장했는지"를 사람이 모르는 상태가 생긴다.
- **장애물을 보지 않는다.** 정면 N미터가 벽이어도 그대로 쏜다. 판단은 Nav2 와 사람이 한다.
- 결과를 판정하지 않는다. SUCCEEDED/ABORTED 를 그대로 인쇄할 뿐이다.

⚠ 실행 전 확인 (`docs/TEST_GATES.md` · 사용자 상시 규칙)
    ros2 topic echo /drive/enabled --once     # → data: true
    ros2 topic echo /drive/diag    --once     # → z: 2.0  (2 = ARMED)
    ros2 topic echo /estop/state   --once     # → data: false
"""

import argparse
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener

MAP_FRAME = "map"
BASE_FRAME = "base_footprint"
TF_WAIT_SEC = 10.0


def yaw_from_quat(q):
    """쿼터니언 → yaw(rad). 평면 주행이라 yaw 만 쓴다."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class GoalAhead(Node):
    def __init__(self):
        super().__init__("goal_ahead")
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def current_pose(self):
        """map → base_footprint 를 읽어 (x, y, yaw). 못 읽으면 None."""
        deadline = self.get_clock().now() + Duration(seconds=TF_WAIT_SEC)
        while rclpy.ok() and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self.buf.lookup_transform(
                    MAP_FRAME, BASE_FRAME, rclpy.time.Time())
            except Exception:
                continue
            t = tf.transform.translation
            return t.x, t.y, yaw_from_quat(tf.transform.rotation)
        return None


def main(argv=None):
    ap = argparse.ArgumentParser(description="현재 위치 기준 정면 N미터 goal")
    ap.add_argument("distance", type=float, help="정면 거리 [m]")
    ap.add_argument("--left", type=float, default=0.0,
                    help="왼쪽 옆으로 [m] (음수 = 오른쪽)")
    ap.add_argument("--turn", type=float, default=0.0,
                    help="도착 방위를 현재에서 이만큼 돌린다 [deg]")
    ap.add_argument("--dry", action="store_true", help="계산만 하고 보내지 않는다")
    args = ap.parse_args(argv)

    rclpy.init()
    node = GoalAhead()
    try:
        pose = node.current_pose()
        if pose is None:
            print("🔴 %s → %s TF 를 %.0f초 안에 못 읽었다." % (
                MAP_FRAME, BASE_FRAME, TF_WAIT_SEC))
            print("   스택이 떠 있는지 · 위치추정이 됐는지 먼저 본다:")
            print("     ros2 run tf2_ros tf2_echo %s %s" % (MAP_FRAME, BASE_FRAME))
            return 2

        x, y, yaw = pose
        # 로봇 정면(+x_body)과 왼쪽(+y_body)을 map 으로 옮긴다.
        gx = x + args.distance * math.cos(yaw) - args.left * math.sin(yaw)
        gy = y + args.distance * math.sin(yaw) + args.left * math.cos(yaw)
        gyaw = yaw + math.radians(args.turn)

        print("=" * 62)
        print("  현재  x=%+.3f  y=%+.3f  yaw=%+.1f°" % (x, y, math.degrees(yaw)))
        print("  목표  x=%+.3f  y=%+.3f  yaw=%+.1f°" % (gx, gy, math.degrees(gyaw)))
        print("  이동  정면 %.2fm · 옆 %.2fm · 회전 %.1f°"
              % (args.distance, args.left, args.turn))
        print("=" * 62)

        if args.dry:
            print("  --dry 라 보내지 않는다.")
            return 0

        if not node.client.wait_for_server(timeout_sec=10.0):
            print("🔴 navigate_to_pose 액션 서버가 없다 — Nav2 가 ACTIVE 인지 본다.")
            return 2

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = MAP_FRAME
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        qx, qy, qz, qw = quat_from_yaw(gyaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        send = node.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            print("🔴 goal 이 거부됐다.")
            return 1
        print("  goal 수납됨 — 결과를 기다린다 (Ctrl+C 로 빠져도 로봇은 계속 간다)")

        result = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result)
        res = result.result()
        # status 4 = SUCCEEDED (action_msgs/GoalStatus)
        print("  결과 status = %s%s" % (
            res.status, "  ✅ SUCCEEDED" if res.status == 4 else ""))
        return 0 if res.status == 4 else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
