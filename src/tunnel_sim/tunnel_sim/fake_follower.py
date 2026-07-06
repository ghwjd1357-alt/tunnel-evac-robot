#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fake_follower.py — 가짜 추종자 (로봇 뒤를 따라 걷는 원기둥) — ②단계 시험 무대
============================================================

[왜 필요한가]
  시나리오 ④"라이다로 사람들이 따라오는지 확인" / ⑥"놓치면 역행 재탐색"을
  개발·검증하려면 **로봇을 따라오는 움직이는 무언가**가 시뮬에 있어야 한다.
  - 터널의 victim(person_standing)은 정지 모델 → 못 따라옴.
  - Gazebo 걷는 actor 는 무거워서 크래시 전력(CLAUDE.md GUI 함정 ②).
  → 원기둥 하나를 이 노드가 매 틱 로봇 뒤로 이동시킨다 (사이비 걷기).
    라이다엔 사람 다리든 원기둥이든 "벽 아닌 점 뭉치"로 동일하게 찍힘 → 감지 알고리즘 검증엔 충분.

[동작]
  ① /spawn_entity 서비스로 원기둥(r=0.15, h=1.0) 을 로봇 뒤 1.5m 에 소환.
  ② 0.2초마다: 로봇 위치(/gazebo/get_entity_state) 를 읽고,
     원기둥을 로봇 쪽으로 '사람 걸음 속도(기본 0.8m/s)' 만큼 이동(/gazebo/set_entity_state).
     로봇과 1.2m 이내면 정지(부딪히지 않게).
  ③ /follower_cmd (std_msgs/String) 로 조종:
     "follow" = 따라가기(기본) / "stop" = 그 자리에 멈춤 (★ '놓침' 시나리오 재현 버튼!)

[전제] tunnel.world 에 libgazebo_ros_state.so 플러그인 (07-05 추가).

[실행]
  ros2 run tunnel_sim fake_follower --ros-args -p use_sim_time:=true
  (또는 ros2 launch tunnel_sim mission.launch.py 가 알아서 켬)
  놓침 재현: ros2 topic pub --times 3 -w 1 /follower_cmd std_msgs/msg/String "{data: stop}"
  ⚠ --once 금지 (§14.3 함정): 디스커버리 매칭 전에 쏘고 죽어 유실될 수 있음
    → -w 1 (구독자 매칭 대기) + --times 로.
"""

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from gazebo_msgs.srv import SpawnEntity, GetEntityState, SetEntityState

# 원기둥 SDF (사람 몸통 근사: 반지름 0.15m, 높이 1.0m).
# static 이 아니라 kinematic 처럼 취급 — 우리가 위치를 직접 갱신하므로
# 물리엔진이 밀지 않게 gravity 끄고 질량은 형식상만.
FOLLOWER_SDF = """
<sdf version="1.6">
  <model name="fake_follower">
    <link name="body">
      <gravity>false</gravity>
      <inertial><mass>1.0</mass>
        <inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz>
                 <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="col">
        <geometry><cylinder><radius>0.15</radius><length>1.0</length></cylinder></geometry>
      </collision>
      <visual name="vis">
        <geometry><cylinder><radius>0.15</radius><length>1.0</length></cylinder></geometry>
        <material><ambient>0.9 0.4 0.1 1</ambient></material>
      </visual>
    </link>
  </model>
</sdf>
"""


class FakeFollower(Node):

    def __init__(self):
        super().__init__('fake_follower')

        # --- 파라미터 (조정 가능) ---
        self.declare_parameter('walk_speed', 0.8)     # 사람 걸음 속도 m/s
        self.declare_parameter('keep_distance', 1.2)  # 로봇과 유지 거리 m
        self.declare_parameter('robot_name', 'tunnel_robot')

        # --- Gazebo 서비스 클라이언트 3종 ---
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.get_cli = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        self.set_cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')

        # --- 조종 토픽: "follow" / "stop" ---
        self.mode = 'follow'
        self.create_subscription(String, '/follower_cmd', self.on_cmd, 10)

        self.spawned = False
        self.pos = None                 # 원기둥 현재 위치 (스폰 후 채움)
        self._pending = False           # 서비스 응답 대기 중 플래그 (중복 호출 방지)

        # 0.2초마다 한 걸음 (5Hz — 라이다 10Hz 가 충분히 따라잡음)
        self.timer = self.create_timer(0.2, self.step)
        self.get_logger().info('가짜 추종자 시작 — 로봇 스폰 대기')

    def on_cmd(self, msg: String):
        if msg.data in ('follow', 'stop'):
            self.mode = msg.data
            self.get_logger().info(f'모드 변경 → {self.mode}'
                                   + (' (놓침 재현!)' if self.mode == 'stop' else ''))

    # ---------------------------------------------------------
    # 매 틱: 로봇 위치 조회 → 원기둥 한 걸음 이동
    #   모든 서비스는 비동기(call_async) — 콜백 안 블로킹 금지 원칙.
    # ---------------------------------------------------------
    def step(self):
        if self._pending:
            return                      # 이전 요청이 아직 진행 중이면 건너뜀
        if not self.get_cli.service_is_ready():
            self.get_logger().warn('gazebo_ros_state 서비스 대기중 (world 플러그인 확인)',
                                   throttle_duration_sec=5.0)
            return
        # ① 로봇 위치 조회 (비동기 → 응답 콜백에서 다음 단계)
        req = GetEntityState.Request()
        req.name = self.get_parameter('robot_name').value
        self._pending = True
        self.get_cli.call_async(req).add_done_callback(self.on_robot_state)

    def on_robot_state(self, future):
        self._pending = False
        res = future.result()
        if not res.success:
            self.get_logger().warn('로봇 상태 조회 실패 (아직 스폰 전?)',
                                   throttle_duration_sec=5.0)
            return
        rp = res.state.pose.position
        rq = res.state.pose.orientation
        ryaw = 2.0 * math.atan2(rq.z, rq.w)   # 로봇 진행방향 (평면이라 z,w 만)

        # ② 최초 1회: 로봇 '뒤' 1.5m 에 원기둥 스폰
        if not self.spawned:
            bx = rp.x - 1.5 * math.cos(ryaw)
            by = rp.y - 1.5 * math.sin(ryaw)
            self.spawn(bx, by)
            return

        if self.mode == 'stop' or self.pos is None:
            return                      # 놓침 재현 중 — 제자리

        # ③ 원기둥을 로봇 쪽으로 한 걸음 (걸음속도 × 0.2초)
        dx = rp.x - self.pos[0]
        dy = rp.y - self.pos[1]
        dist = math.hypot(dx, dy)
        keep = self.get_parameter('keep_distance').value
        if dist <= keep:
            return                      # 충분히 가까움 — 대기 (부딪힘 방지)
        step_len = min(self.get_parameter('walk_speed').value * 0.2, dist - keep)
        nx = self.pos[0] + dx / dist * step_len
        ny = self.pos[1] + dy / dist * step_len
        self.teleport(nx, ny, math.atan2(dy, dx))

    def spawn(self, x, y):
        if not self.spawn_cli.service_is_ready():
            return
        req = SpawnEntity.Request()
        req.name = 'fake_follower'
        req.xml = FOLLOWER_SDF
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = 0.5      # 원기둥 중심 (높이 1.0 의 절반)
        self._pending = True
        fut = self.spawn_cli.call_async(req)

        def done(f):
            self._pending = False
            res = f.result()
            if res.success:
                self.spawned = True
                self.pos = (x, y)
                self.get_logger().info(f'추종자 스폰 완료 ({x:.2f}, {y:.2f}) — follow 시작')
            elif 'already exists' in res.status_message:
                # ★ 멱등 처리: 이전 실행이 남긴 원기둥이 있으면 새로 만들지 말고 '접수'.
                #   (이거 없으면 0.2초마다 스폰 재시도 → 경고 폭주 실측 07-05)
                self.spawned = True
                self.get_logger().info('기존 추종자 모델 접수 — 시작 위치로 이동 후 follow')
                self.teleport(x, y, 0.0)
            else:
                self.get_logger().warn(f'스폰 실패: {res.status_message}',
                                       throttle_duration_sec=5.0)
        fut.add_done_callback(done)

    def teleport(self, x, y, yaw):
        req = SetEntityState.Request()
        req.state.name = 'fake_follower'
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = 0.5
        req.state.pose.orientation.z = math.sin(yaw / 2.0)
        req.state.pose.orientation.w = math.cos(yaw / 2.0)
        self._pending = True

        def done(f):
            self._pending = False
            if f.result().success:
                self.pos = (x, y)
        self.set_cli.call_async(req).add_done_callback(done)


def main(args=None):
    rclpy.init(args=args)
    node = FakeFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
