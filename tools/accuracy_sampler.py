#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
accuracy_sampler.py — SLAM 추정위치 vs 시뮬 실위치(ground truth) 연속 기록기
============================================================

[왜 필요한가]
  기존 회귀(regression_3goals.sh)는 '끝점 1개' 오차만 본다.
  주행 "도중" 얼마나 어긋나는지(궤적 전체 오차)는 못 봄 →
  이 노드가 매 주기(기본 1초) 두 값을 동시에 찍어 CSV 로 남긴다:
    ① ground truth : /gazebo/get_entity_state 서비스 (시뮬의 진짜 위치, world 좌표)
    ② SLAM 추정    : TF map→base_footprint (로봇이 '믿는' 위치, map 좌표)
  좌표계 통일: map = world + offset (스폰 world(-12,0) 이 map(0,0) → offset=(12,0)).

[출력 CSV] t,gt_x,gt_y,est_x,est_y,err   (t=시뮬시각 초, err=순간 위치오차 m)

[실행] (accuracy_bench.sh 가 알아서 켬 — 단독 실행도 가능)
  python3 tools/accuracy_sampler.py --ros-args -p use_sim_time:=true \
      -p csv:=/tmp/trace.csv

[설계 원칙 — 기존 노드들과 동일]
  콜백 안 블로킹 금지(§12.2): 서비스는 call_async, TF 는 예외 무시 후 다음 틱.
"""

import math

import rclpy
from rclpy.node import Node
import tf2_ros

from gazebo_msgs.srv import GetEntityState


class AccuracySampler(Node):

    def __init__(self):
        super().__init__('accuracy_sampler')

        # --- 파라미터 ---
        self.declare_parameter('csv', '/tmp/accuracy_trace.csv')
        self.declare_parameter('robot_name', 'tunnel_robot')
        self.declare_parameter('offset_x', 12.0)   # map = world + offset
        self.declare_parameter('offset_y', 0.0)
        self.declare_parameter('period', 1.0)      # 샘플 주기(초)

        # --- TF 수신 준비 (map→base_footprint = SLAM 추정) ---
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)

        # --- ground truth 서비스 (world 플러그인 libgazebo_ros_state) ---
        self.cli = self.create_client(GetEntityState, '/gazebo/get_entity_state')

        path = self.get_parameter('csv').value
        self.f = open(path, 'w')
        self.f.write('t,gt_x,gt_y,est_x,est_y,err\n')
        self._pending = False   # 서비스 응답 대기 중 중복 호출 방지

        self.create_timer(float(self.get_parameter('period').value), self.tick)
        self.get_logger().info(f'샘플링 시작 → {path}')

    def tick(self):
        if self._pending or not self.cli.service_is_ready():
            return
        req = GetEntityState.Request()
        req.name = self.get_parameter('robot_name').value
        self._pending = True
        self.cli.call_async(req).add_done_callback(self.on_gt)

    def on_gt(self, future):
        self._pending = False
        res = future.result()
        if res is None or not res.success:
            return                      # 로봇 스폰 전 — 다음 틱에 재시도
        try:
            tfm = self.buf.lookup_transform('map', 'base_footprint',
                                            rclpy.time.Time())   # Time()=최신 TF
        except Exception:
            return                      # SLAM 이 아직 map→odom 못 채움 — 다음 틱

        gx = res.state.pose.position.x + self.get_parameter('offset_x').value
        gy = res.state.pose.position.y + self.get_parameter('offset_y').value
        ex = tfm.transform.translation.x
        ey = tfm.transform.translation.y
        t = self.get_clock().now().nanoseconds / 1e9
        err = math.hypot(gx - ex, gy - ey)
        self.f.write(f'{t:.2f},{gx:.3f},{gy:.3f},{ex:.3f},{ey:.3f},{err:.3f}\n')
        self.f.flush()                  # 프로세스 kill 돼도 기록 보존


def main(args=None):
    rclpy.init(args=args)
    node = AccuracySampler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.f.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
