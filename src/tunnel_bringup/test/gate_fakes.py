#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gate_fakes.py — `readiness_gate` 회귀용 '가짜 조건' 발생기 (로봇·Gazebo 불필요).

============================================================
[왜 필요한가]
  게이트가 보는 것은 프로세스가 아니라 **토픽·TF·lifecycle 서비스·액션**이다.
  그래서 실물이 없어도 그 네 가지만 흉내 내면 게이트를 그대로 시험할 수 있다.
  (이 성질은 원래 설계 의도이기도 하다 — 그래서 `ros2 bag play` 로도 상위 스택이 돈다.)

[이 파일이 만드는 것 — 모드별]
  sensors    : /scan 10Hz · /odom 50Hz · /imu/data 50Hz (+옵션 /odometry/filtered) 를
               micro-ROS 토픽 계약대로 발행. ⚠ IMU 는 07-30 확정 **50Hz** 다
               (`REAL_ROBOT_VALUES.md §1` 의 '100Hz 발행 예정'은 옛 값 — 정본 갱신 필요).
               --messages N 으로 **N건만 보내고 멈추는** 죽은 퍼블리셔도 흉내 낸다.
  lifecycle  : `<노드>/get_state` 서비스를 대본대로 응답. 대본 항목:
                 active   = ACTIVE(3) 응답
                 inactive = INACTIVE(2) 응답
                 hang     = **응답하지 않음** (서비스 이름은 그래프에 그대로 남는다)
               ★ 'active,hang' 이 2026-07-29 Codex P1 의 재현 입력이다 —
                 "한 번 ACTIVE 를 답한 직후 executor 가 멎은 노드".
  action     : 액션 서버를 그래프에 올려 둔다 (goal 은 전부 거절 — 존재만 흉내).

[QoS 를 일부러 BEST_EFFORT 로 발행하는 이유]
  실차 /odom·/imu/data 는 micro-ROS 쪽이 BEST_EFFORT 예정이다(TEENSY 합의사항 §4.5).
  게이트가 RELIABLE 로 구독하면 매칭이 안 돼 '센서가 죽은 것처럼' 보인다.
  여기서 BEST_EFFORT 로 쏘면, 게이트 구독 QoS 가 잘못 바뀌는 순간 회귀가 실패한다.

[실행 — 하네스(tools/test_gate_regression.sh)가 부른다]
  python3 gate_fakes.py sensors --messages 0
  python3 gate_fakes.py lifecycle --node /planner_server:active,hang
  python3 gate_fakes.py action --name /navigate_to_pose
"""

import argparse
import math
import sys
import threading
import time

from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState

from nav2_msgs.action import NavigateToPose

from nav_msgs.msg import Odometry

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import Imu, LaserScan


BEST_EFFORT = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)


class FakeSensors(Node):
    """계약대로 도는 가짜 센서. --messages N 이면 N건 뒤 발행을 멈춘다(죽은 퍼블리셔)."""

    def __init__(self, messages, with_filtered, delay=0.0):
        super().__init__('gate_fake_sensors')
        self.limit = messages          # 0 = 무제한
        self.sent = {}
        # ★ 퍼블리셔는 지금 만들고 **발행만 늦춘다.** 게이트는 그래프에서 타입을 찾아
        #   구독하므로 퍼블리셔가 먼저 있어야 하고, "1건만 보내고 죽는" 케이스는 그
        #   1건이 구독 뒤에 와야 한다. 안 그러면 구독 전에 흘려보내 '수신 0건'이 되고,
        #   시험하려던 것(1건 받고도 통과 못 함)이 아니라 엉뚱한 이유로 실패한다.
        self._go_at = time.monotonic() + delay
        self.scan_pub = self.create_publisher(LaserScan, '/scan', BEST_EFFORT)
        self.odom_pub = self.create_publisher(Odometry, '/odom', BEST_EFFORT)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', BEST_EFFORT)
        self.create_timer(0.1, self._scan)      # 10Hz
        self.create_timer(0.02, self._odom)     # 50Hz
        self.create_timer(0.02, self._imu)      # 50Hz (07-30 확정 — 옛 '100Hz 예정' 아님)
        if with_filtered:
            # EKF 를 띄우지 않는 게이트 단독 케이스용 — EKF 출력 자리를 대신 채운다.
            self.filtered_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
            self.create_timer(0.033, self._filtered)

    def _budget(self, key):
        """이 토픽이 아직 발행해도 되는지 (한도 소진 = 죽은 퍼블리셔 흉내)."""
        if time.monotonic() < self._go_at:
            return False
        sent = self.sent.get(key, 0)
        if self.limit and sent >= self.limit:
            return False
        self.sent[key] = sent + 1
        return True

    def _stamp(self, msg, frame):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame
        return msg

    def _scan(self):
        """RPLIDAR C1 흉내 — 360빔, 전방 5m 벽."""
        if not self._budget('scan'):
            return
        msg = self._stamp(LaserScan(), 'lidar_link')
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = 2.0 * math.pi / 360.0
        msg.range_min = 0.1
        msg.range_max = 12.0
        msg.ranges = [5.0] * 360
        self.scan_pub.publish(msg)

    def _odom(self):
        """구동부 /odom 흉내 — covariance 는 0 이 아니어야 EKF 가 받아들인다."""
        if not self._budget('odom'):
            return
        msg = self._stamp(Odometry(), 'odom')
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.orientation.w = 1.0
        for i in (0, 7, 14, 21, 28, 35):
            msg.pose.covariance[i] = 0.05
            msg.twist.covariance[i] = 0.05
        self.odom_pub.publish(msg)

    def _imu(self):
        """BNO055 흉내 (100Hz — 합의사항)."""
        if not self._budget('imu'):
            return
        msg = self._stamp(Imu(), 'imu_link')
        msg.orientation.w = 1.0
        for i in (0, 4, 8):
            msg.orientation_covariance[i] = 0.01
            msg.angular_velocity_covariance[i] = 0.01
            msg.linear_acceleration_covariance[i] = 0.05
        self.imu_pub.publish(msg)

    def _filtered(self):
        """EKF 출력 자리."""
        if not self._budget('filtered'):
            return
        msg = self._stamp(Odometry(), 'odom')
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.orientation.w = 1.0
        self.filtered_pub.publish(msg)


class FakeLifecycle(Node):
    """`<노드>/get_state` 를 대본대로 응답한다 (대본이 떨어지면 마지막 항목 반복)."""

    def __init__(self, specs):
        super().__init__('gate_fake_lifecycle')
        self._group = ReentrantCallbackGroup()
        self._counts = {}
        self._stop = threading.Event()
        for name, script in specs:
            srv = name if name.endswith('/get_state') else f'{name}/get_state'
            self.create_service(
                GetState, srv,
                self._make_cb(name, script), callback_group=self._group)
            self.get_logger().info(f'가짜 lifecycle {srv} — 대본 {script}')

    def _make_cb(self, name, script):
        """노드 하나의 응답 콜백을 만든다."""
        def _cb(_request, response):
            index = self._counts.get(name, 0)
            self._counts[name] = index + 1
            behavior = script[min(index, len(script) - 1)]
            if behavior == 'hang':
                # ★ 응답하지 않는다. 서비스 이름은 그래프에 그대로 남으므로,
                #   게이트 입장에서는 "엔드포인트는 있는데 답이 없다"가 된다.
                self.get_logger().warn(f'{name}: 요청 {index + 1} — 응답 정지(멎은 노드)')
                self._stop.wait()
                return response
            if behavior == 'active':
                response.current_state.id = State.PRIMARY_STATE_ACTIVE
                response.current_state.label = 'active'
            else:
                response.current_state.id = State.PRIMARY_STATE_INACTIVE
                response.current_state.label = 'inactive'
            self.get_logger().info(f'{name}: 요청 {index + 1} → {behavior}')
            return response
        return _cb


class FakeAction(Node):
    """액션 서버를 그래프에 올려 두기만 한다 (goal 은 전부 거절)."""

    def __init__(self, name):
        super().__init__('gate_fake_action')
        self._server = ActionServer(self, NavigateToPose, name, self._execute)
        self.get_logger().info(f'가짜 액션 서버 {name}')

    def _execute(self, goal_handle):
        """존재만 흉내 — 실제 주행은 하지 않는다."""
        goal_handle.abort()
        return NavigateToPose.Result()


def parse_spec(text):
    """`/planner_server:active,hang` → ('/planner_server', ['active', 'hang'])."""
    if ':' in text:
        name, script = text.split(':', 1)
    else:
        name, script = text, 'active'
    behaviors = [b.strip() for b in script.split(',') if b.strip()]
    for behavior in behaviors:
        if behavior not in ('active', 'inactive', 'hang'):
            raise ValueError(f'알 수 없는 대본 항목: {behavior}')
    return name, behaviors


def build_parser():
    """CLI 정의."""
    parser = argparse.ArgumentParser(description='readiness_gate 회귀용 가짜 조건 발생기')
    sub = parser.add_subparsers(dest='mode', required=True)

    sensors = sub.add_parser('sensors', help='/scan·/odom·/imu/data 발행')
    sensors.add_argument('--messages', type=int, default=0,
                         help='토픽마다 N건만 보내고 멈춘다 (0=무제한)')
    sensors.add_argument('--with-filtered', action='store_true',
                         help='/odometry/filtered 도 발행 (EKF 를 안 띄우는 케이스용)')
    sensors.add_argument('--delay', type=float, default=0.0,
                         help='퍼블리셔는 즉시 만들되 발행은 N초 뒤부터 (구독 뒤에 흘리기)')

    lifecycle = sub.add_parser('lifecycle', help='<노드>/get_state 서비스')
    lifecycle.add_argument('--node', action='append', required=True,
                           help='이름[:대본] (예: /planner_server:active,hang)')

    action = sub.add_parser('action', help='액션 서버 존재만 흉내')
    action.add_argument('--name', default='/navigate_to_pose')
    return parser


def main(argv=None):
    """모드에 맞는 가짜 노드를 띄우고 kill 될 때까지 돈다."""
    args = build_parser().parse_args(argv)
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=8)
    if args.mode == 'sensors':
        node = FakeSensors(args.messages, args.with_filtered, args.delay)
    elif args.mode == 'lifecycle':
        node = FakeLifecycle([parse_spec(spec) for spec in args.node])
    else:
        node = FakeAction(args.name)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 멎은 콜백이 스레드를 붙들고 있어도 프로세스는 확실히 끝나야 한다
        # (하네스가 다음 케이스로 못 넘어가는 것을 막는다).
        time.sleep(0.1)
        sys.stdout.flush()
    return 0


if __name__ == '__main__':
    sys.exit(main())
