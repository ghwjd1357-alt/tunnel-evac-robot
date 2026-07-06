#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mission_node.py — 대피 유도 로봇 임무 상태머신 (③단계: 후방 추종감시 + SEARCH_BACK)
============================================================

[이 노드의 역할]
  Nav2 '위에서' 도는 지휘관. "어떻게 갈지(경로·회피·바퀴)"는 Nav2 가 다 하고,
  우리는 "지금 어디로 갈지"만 결정한다. 그 결정을 상태(state)로 관리하는 게 상태머신.

[③단계 상태도 — 시나리오 그림 전체 구현]
  PATROL ─화재─> APPROACH ─도착─> GATHER(T초·싸이렌) ─경과─> GUIDE(저속 유도+후방감시)
                                                              │            │
                                                          추종놓침      도착
                                                              ▼            ▼
                                              SEARCH_BACK(역행 재탐색)   ESCAPED
                                                │재발견→GUIDE 복귀
                                                │제한초과→보고 후 단독 탈출(GUIDE)
  + FAULT: Nav2 실패 자동 재시도 2회 → 소진 시 정지.

  ★ SEARCH_BACK 안전장치 2개 (설계 §12.0에서 못박음):
    ① 재시도 횟수 제한(max_attempts) — GUIDE⇄SEARCH_BACK 무한 왕복 방지.
    ② 화재 안전하한(min_fire_dist) — 역행 목표가 화재에 이보다 가까우면 뒤로 클램프.
       (놓친 사람 찾으러 불속으로 들어가는 로직 원천 차단)

  ⚠ 시나리오는 확정이 아님(잠정 합의) — 상태 추가·순서 변경 가능성 높음 (0705_현황.md §12.0).
    예: 카메라 관절추정으로 거동가능 판별 → 거동불능자 분기. 상태 로직은 얇게 유지.

  ★ funnel 원칙 (§12.5): 외부 토픽은 콜백 하나 → 내부 dict 번역, 없는 필드도 자리 예약.
  ★ 후방감지는 FollowerMonitor 모듈에 격리 — visible()/lost() 두 답만 사용.
    지도 배경제거·카메라 융합으로 업그레이드해도 이 파일은 안 바뀜.

[통신 요약]
  구독  /alarm (PoseStamped)   ← 화재 신호(+좌표). 관제 계약 미정 — 임시.
  구독  /scan  (LaserScan)     ← 후방 추종감시 재료 (FollowerMonitor 로 전달).
  발행  /mission_state, /siren
  액션  navigate_to_pose        → 유일한 주행 명령 경로.
  서비스 /controller_server/set_parameters → GUIDE 저속/복원.
  TF    map→base_footprint 조회 → 마지막 목격 지점 기록(SEARCH_BACK 목표).

[실행 — ★ 시뮬에선 use_sim_time 필수]
  ros2 run mission_manager mission_node --ros-args -p use_sim_time:=true
"""

import math
import os

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

from enum import Enum, auto
from functools import partial

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

import tf2_ros

from ament_index_python.packages import get_package_share_directory
from mission_manager.follower_monitor import FollowerMonitor


class State(Enum):
    PATROL = auto()       # 평시 순찰
    APPROACH = auto()     # 화재 → 집결지 이동 (싸이렌 ON)
    GATHER = auto()       # T초 집결 대기
    GUIDE = auto()        # 저속 선행 유도 + 후방 추종감시
    SEARCH_BACK = auto()  # 놓침 → 마지막 목격 지점으로 역행 재탐색
    ESCAPED = auto()      # 탈출 완료
    FAULT = auto()        # Nav2 실패 → 자동 재시도 → 소진 시 정지


def yaw_to_quat(yaw):
    """yaw(각도 1개) → quaternion. 평면 로봇이라 z,w 만 유효."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def clamp_to_fire_min_dist(gx, gy, fx, fy, dmin):
    """안전장치 ②의 수식부 (순수 함수로 분리 — 단위테스트 대상, 07-06).
    역행 목표 (gx,gy)가 화재 (fx,fy)에서 dmin 보다 가까우면
    화재→목표 방향을 유지한 채 dmin 지점으로 밀어낸 좌표를 돌려준다.
    목표가 화재와 사실상 같은 점이면 방향 정의 불가 → None (역행 포기)."""
    d = math.hypot(gx - fx, gy - fy)
    if d >= dmin:
        return (gx, gy)                 # 충분히 멀다 — 그대로
    if d < 1e-6:
        return None                     # 목표=화재 지점 — 밀어낼 방향이 없음
    return (fx + (gx - fx) / d * dmin,
            fy + (gy - fy) / d * dmin)


def compute_gather_point(fx, fy, ex, ey, gather_dist):
    """집결지 계산 (순수 함수 — 단위테스트 대상, 07-06 ⓐ 모듈).

    시나리오 요구 = "화재에 가깝되 안전한 곳". 대피자들은 화재에서 탈출구
    방향으로 도망치므로, 집결지 = **화재→탈출구 방향선 위, 화재에서
    gather_dist 만큼 떨어진 점.** yaw 는 탈출구를 바라보게(집결 후 바로
    그 방향으로 유도 출발).

        탈출구(ex,ey) ←—— ●집결지 ——— 🔥화재(fx,fy)
                          └ gather_dist ┘

    반환: {'x','y','yaw'} dict (send_goal 이 먹는 waypoint 형식 그대로)
          화재=탈출구 동일점(방향 정의 불가)이면 None → 호출부가 yaml
          고정 집결지로 fallback.
    경계: 화재가 탈출구에 gather_dist 보다 가까우면 탈출구 자체로 클램프
          (탈출구를 지나쳐 화재 반대편으로 나가는 것 방지).
    ⚠ 한계(의도적): 직선 수식이라 화재가 곁복도(분기)에 있으면 벽을 뚫는
      지점이 나올 수 있음 → Nav2 가 거부해 FAULT 재시도가 흡수. 복도
      그래프 경유지 방식은 시나리오 확정 후 과제 (0705_현황.md §16)."""
    d = math.hypot(ex - fx, ey - fy)
    if d < 1e-6:
        return None                     # 화재=탈출구 — 방향 정의 불가
    if d <= gather_dist:
        gx, gy = ex, ey                 # 화재가 탈출구 코앞 — 탈출구에서 집결
    else:
        gx = fx + (ex - fx) / d * gather_dist
        gy = fy + (ey - fy) / d * gather_dist
    yaw = math.atan2(ey - gy, ex - gx)  # 탈출구 바라보기
    if gx == ex and gy == ey:           # 클램프된 경우: 화재 반대 방향 바라보기
        yaw = math.atan2(ey - fy, ex - fx)
    return {'x': gx, 'y': gy, 'yaw': yaw}


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_manager')

        # --- 설정 yaml (좌표·타이밍·속도·탐색 파라미터 — 전부 코드 밖) ---
        default_wp = os.path.join(
            get_package_share_directory('mission_manager'),
            'config', 'waypoints.yaml')
        self.declare_parameter('waypoints_file', default_wp)
        wp_path = self.get_parameter('waypoints_file').value
        with open(wp_path, 'r') as f:
            self.wp = yaml.safe_load(f)
        self.get_logger().info(f'웨이포인트 로드: {wp_path}')

        # --- 통신 구성 ---
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.state_pub = self.create_publisher(String, '/mission_state', 10)
        self.siren_pub = self.create_publisher(Bool, '/siren', 10)
        self.create_subscription(PoseStamped, '/alarm', self.on_alarm, 10)
        self.param_cli = self.create_client(SetParameters,
                                            '/controller_server/set_parameters')

        # --- 후방 추종감시 (③단계) ---
        sb = self.wp.get('search_back', {})
        self.monitor = FollowerMonitor(
            self.get_clock(),
            cone_half_deg=float(sb.get('cone_half_deg', 60.0)),
            max_range=float(sb.get('detect_range', 2.5)),
            lost_sec=float(sb.get('lost_sec', 3.0)),
            seen_sec=float(sb.get('seen_sec', 1.0)),
            max_cluster_width=float(sb.get('cluster_max_width', 0.8)))
        # ⚠ 시뮬 라이다 QoS = sensor(BestEffort) — 기본 Reliable 구독이면 한 장도 안 옴
        self.create_subscription(LaserScan, '/scan', self.on_scan,
                                 qos_profile_sensor_data)
        # TF 조회 (마지막 목격 지점 기록용)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- 상태머신 내부 변수 ---
        self.state = State.PATROL
        self.patrol_idx = 0
        self.goal_active = False
        self.goal_seq = 0
        self._goal_handle = None
        self.gather_since = None
        self._escaped_logged = False
        self.siren_on = False
        self.fire = None                # funnel 번역된 화재 정보
        self.gather_wp = None           # 화재 좌표로 계산한 집결지 (없으면 yaml 고정값)

        # --- SEARCH_BACK 관리 ---
        self.search_attempts = 0        # 역행 시도 횟수 (안전장치 ①)
        self.give_up = False            # 제한 초과 → 단독 탈출 모드
        self.last_seen = None           # 마지막 목격 시점의 로봇 map 좌표 (x, y)
        self.search_goal = None         # 이번 역행의 목표
        self.refind_since = None        # 역행 목표 도착 후 재탐색 대기 시작 시각

        # --- FAULT 자동 재시도 ---
        self.fault_retries = 0
        self.MAX_RETRIES = 2
        self.RETRY_WAIT = 5.0
        self.fault_since = None
        self.resume_state = None

        self.timer = self.create_timer(0.5, self.tick)
        self.get_logger().info('임무 노드 시작 → PATROL')

    # ===========================================================
    # Nav2 목표 전송 (리모컨)
    # ===========================================================
    def send_goal(self, wp, tag=''):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        # stamp=0 유지 — "최신 TF 사용" (§12.2 ①)
        goal.pose.pose.position.x = float(wp['x'])
        goal.pose.pose.position.y = float(wp['y'])
        _, _, qz, qw = yaw_to_quat(float(wp.get('yaw', 0.0)))
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        if not self.nav.server_is_ready():        # 블로킹 금지 (§12.2 ②)
            self.get_logger().warn('Nav2 액션서버 아직 없음',
                                   throttle_duration_sec=5.0)
            return

        self.goal_seq += 1
        seq = self.goal_seq
        self.goal_active = True
        self.get_logger().info(
            f'[{self.state.name}] 목표전송 {tag} → ({wp["x"]:.1f}, {wp["y"]:.1f})')
        fut = self.nav.send_goal_async(goal)
        fut.add_done_callback(partial(self.on_goal_response, seq))

    def on_goal_response(self, seq, future):
        if seq != self.goal_seq:
            return
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('목표 거부됨 → FAULT')
            self.goal_active = False
            self.enter_fault()
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(partial(self.on_result, seq))

    def on_result(self, seq, future):
        if seq != self.goal_seq:
            return
        self.goal_active = False
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.fault_retries = 0
            self.on_reached()
        else:
            self.get_logger().warn(f'목표 실패(status={status}) → FAULT (재시도 판단)')
            self.enter_fault()

    def cancel_current_goal(self):
        self.goal_seq += 1
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self.goal_active = False

    # ===========================================================
    # 도달 시 상태 전이
    # ===========================================================
    def on_reached(self):
        if self.state == State.PATROL:
            self.patrol_idx = (self.patrol_idx + 1) % len(self.wp['patrol'])

        elif self.state == State.APPROACH:
            self.state = State.GATHER
            self.gather_since = self.get_clock().now()
            self.get_logger().info(
                f'집결지 도착 → GATHER: {self.wp["gather_wait_sec"]}초 집결대기')

        elif self.state == State.GUIDE:
            self.set_siren(False)
            self.set_nav_speed(float(self.wp['normal_speed']))
            self.state = State.ESCAPED

        elif self.state == State.SEARCH_BACK:
            # 역행 목표 도착 — 아직 재발견 못 함 → 그 자리에서 잠시 더 기다림
            self.refind_since = self.get_clock().now()
            self.get_logger().info('역행 지점 도착 — 재탐색 대기')

    # ===========================================================
    # 심장박동
    # ===========================================================
    def tick(self):
        # 상태·싸이렌 상시 발행
        m = String()
        m.data = self.state.name
        self.state_pub.publish(m)
        b = Bool()
        b.data = self.siren_on
        self.siren_pub.publish(b)

        if self.state == State.PATROL:
            if not self.goal_active:
                self.send_goal(self.wp['patrol'][self.patrol_idx], tag='patrol')

        elif self.state == State.APPROACH:
            if not self.goal_active:
                # 계산된 집결지 우선, 계산 불가였으면 yaml 고정값 (fallback)
                self.send_goal(self.gather_wp or self.wp['gather'], tag='gather')

        elif self.state == State.GATHER:
            elapsed = (self.get_clock().now() - self.gather_since).nanoseconds / 1e9
            if elapsed >= float(self.wp['gather_wait_sec']):
                self.get_logger().info('집결대기 종료 → GUIDE (저속 유도 시작)')
                self.set_nav_speed(float(self.wp['guide_speed']))
                self.state = State.GUIDE

        elif self.state == State.GUIDE:
            if not self.goal_active:
                self.send_goal(self.wp['escape'], tag='escape')
            # --- 추종감시 (give_up 이면 단독 탈출 — 더는 안 돌아봄) ---
            # ★ zone='any'(전방위) 로 판정 (07-06 E2E 가 잡은 설계 구멍 수정):
            #   집결지에서 로봇이 180° 회전하면 추종자가 로봇 '앞'에 있고,
            #   유도 초반 추월 구간에선 '옆'에 있다 — rear(후방 부채꼴)만 보면
            #   그 동안 가짜 '놓침'이 뜨며 역행 예산 2회를 전부 태워먹는다 (실측).
            #   유도의 본질은 "사람이 근처에 있나"지 "정확히 뒤에 있나"가 아님.
            #   (1차 점개수 구현에선 any 가 벽 오탐 탓에 못 쓸 물건이었지만,
            #    클러스터 크기 판별로 any 가 신뢰 가능해져 이 수정이 가능해짐)
            if not self.give_up:
                if self.monitor.visible(zone='any'):
                    self.record_last_seen()       # 보이는 동안 위치 갱신
                elif self.monitor.lost(zone='any'):
                    self.enter_search_back()      # 놓침 확정 → 역행

        elif self.state == State.SEARCH_BACK:
            # 재발견은 zone='any'(전방위) — 역행 중엔 사람이 로봇 '앞'에 있으므로!
            if self.monitor.visible(zone='any'):
                # ★ 재발견 → 유도 재개
                self.get_logger().info('★ 추종자 재발견 → GUIDE 복귀')
                self.cancel_current_goal()
                self.refind_since = None
                self.monitor.reset('any')    # 타이머 리셋 — 복귀 즉시 재-놓침 방지
                self.state = State.GUIDE
            elif not self.goal_active and self.refind_since is None:
                self.send_goal(self.search_goal, tag='search_back')
            elif self.refind_since is not None:
                # 역행 지점 도착 후 대기 — 시간 다 되면 이번 시도 실패
                waited = (self.get_clock().now() - self.refind_since).nanoseconds / 1e9
                if waited >= float(self.wp['search_back']['refind_wait_sec']):
                    self.refind_since = None
                    self.get_logger().warn(
                        f'역행 재탐색 실패 ({self.search_attempts}/'
                        f'{self.wp["search_back"]["max_attempts"]}) → 유도 재개')
                    self.state = State.GUIDE   # 놓친 채 계속 — 재놓침 판정은 GUIDE 가 함

        elif self.state == State.ESCAPED:
            if not self._escaped_logged:
                self.get_logger().info('★ 탈출 완료 — 임무 종료.')
                self._escaped_logged = True

        elif self.state == State.FAULT:
            if self.fault_retries < self.MAX_RETRIES and self.resume_state is not None:
                elapsed = (self.get_clock().now() - self.fault_since).nanoseconds / 1e9
                if elapsed >= self.RETRY_WAIT:
                    self.fault_retries += 1
                    self.state = self.resume_state
                    self.resume_state = None
                    self.get_logger().warn(
                        f'재시도 {self.fault_retries}/{self.MAX_RETRIES} → {self.state.name} 복귀')

    # ===========================================================
    # SEARCH_BACK 진입 — 안전장치 2개가 여기서 작동
    # ===========================================================
    def enter_search_back(self):
        sb = self.wp['search_back']
        # 안전장치 ①: 시도 횟수 제한 → 초과 시 보고 후 단독 탈출
        if self.search_attempts >= int(sb['max_attempts']):
            if not self.give_up:
                self.give_up = True
                self.get_logger().error(
                    '⚠ 역행 재시도 소진 — 관제 보고: 추종자 확인 불가. 단독 탈출 계속.')
            return
        if self.last_seen is None:
            self.get_logger().warn('마지막 목격 지점 없음 — 역행 불가, 유도 계속')
            return

        gx, gy = self.last_seen

        # 안전장치 ②: 화재 안전하한 — 역행 목표가 화재에 너무 가까우면 뒤로 클램프
        # (수식은 clamp_to_fire_min_dist 순수 함수 — 단위테스트로 검증됨)
        if self.fire is not None:
            fx, fy = self.fire['pos']
            dmin = float(sb['min_fire_dist'])
            clamped = clamp_to_fire_min_dist(gx, gy, fx, fy, dmin)
            if clamped is None:
                self.get_logger().error('역행 목표=화재 지점 — 역행 포기')
                return
            if clamped != (gx, gy):
                gx, gy = clamped
                self.get_logger().warn(
                    f'⚠ 화재 안전하한 작동: 역행 목표를 화재에서 {dmin}m 지점으로 클램프')

        # 여기까지 왔으면 실제로 역행한다 — 이때만 시도 횟수 소모
        # (클램프 포기 등으로 역행 없이 return 하는 경로는 예산을 안 깎음, 07-06 수정)
        self.search_attempts += 1
        self.get_logger().warn(
            f'★ 추종 놓침 확정 → SEARCH_BACK {self.search_attempts}/{sb["max_attempts"]}: '
            f'마지막 목격 ({gx:.1f}, {gy:.1f}) 로 역행')
        self.cancel_current_goal()
        self.search_goal = {'x': gx, 'y': gy, 'yaw': 0.0}
        self.refind_since = None
        self.state = State.SEARCH_BACK

    def record_last_seen(self):
        """추종자가 보이는 동안 로봇의 map 좌표를 기록 (추종자는 ~1.2m 뒤 = 근사 충분)."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint',
                                                rclpy.time.Time())
            self.last_seen = (t.transform.translation.x,
                              t.transform.translation.y)
        except Exception:
            pass                        # TF 아직 없음 — 다음 tick 에

    # ===========================================================
    # 이벤트 콜백 (funnel)
    # ===========================================================
    def on_scan(self, msg: LaserScan):
        # /scan 은 모니터에만 전달 — 판정 로직은 전부 모듈 안 (교체 가능)
        self.monitor.update(msg)

    def on_alarm(self, msg: PoseStamped):
        """funnel: raw msg 저장 금지, 내부 dict 번역. 계약 바뀌면 여기 한 곳만."""
        if self.state != State.PATROL:
            return
        self.fire = {
            'pos': (msg.pose.position.x, msg.pose.position.y),
            'kind': 'fire',             # 자리 예약
        }
        # ⓐ 집결지 계산 (07-06): 화재 좌표 기반 — 화재→탈출구 방향 gather_dist 지점
        fx, fy = self.fire['pos']
        esc = self.wp['escape']
        self.gather_wp = compute_gather_point(
            fx, fy, float(esc['x']), float(esc['y']),
            float(self.wp.get('gather_dist', 8.0)))
        if self.gather_wp is not None:
            self.get_logger().info(
                f'집결지 계산: 화재({fx:.1f},{fy:.1f}) → '
                f'({self.gather_wp["x"]:.1f}, {self.gather_wp["y"]:.1f})')
        else:
            self.get_logger().warn('집결지 계산 불가(화재=탈출구) — yaml 고정 집결지 사용')
        self.get_logger().info('🔥 화재 알람 수신 → PATROL 중단, APPROACH 시작 (싸이렌 ON)')
        self.cancel_current_goal()
        self.set_siren(True)
        self.state = State.APPROACH

    def enter_fault(self):
        if self.state != State.FAULT:
            self.resume_state = self.state
        self.state = State.FAULT
        self.fault_since = self.get_clock().now()
        if self.fault_retries >= self.MAX_RETRIES:
            self.get_logger().error(
                f'FAULT — 재시도 {self.MAX_RETRIES}회 소진, 정지. (사람 개입 필요)')
        else:
            self.get_logger().warn(f'FAULT — {self.RETRY_WAIT}초 후 재시도 예정')

    def set_siren(self, on: bool):
        self.siren_on = on              # 발행은 tick 이 매번 반복

    def set_nav_speed(self, v: float):
        """RPP 순항속도 동적 변경 (비동기 — 블로킹 금지)."""
        if not self.param_cli.service_is_ready():
            self.get_logger().warn('controller_server 파라미터 서비스 없음 — 속도 변경 생략')
            return
        p = Parameter()
        p.name = 'FollowPath.desired_linear_vel'
        p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=v)
        self.param_cli.call_async(SetParameters.Request(parameters=[p]))
        self.get_logger().info(f'주행속도 변경 요청 → {v} m/s')


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
