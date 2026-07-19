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

import heapq
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


def project_to_segment(px, py, ax, ay, bx, by):
    """점 P 를 선분 AB 위로 '수선 투영' (07-19 그래프 모듈의 기하 부품).

    수선의 발이 선분 밖이면 가까운 끝점으로 클램프 (t 를 0~1 로 제한).
    반환: (t, qx, qy, dist)
      t    = A→B 위 비율 (0=A, 1=B)
      q    = 투영점 좌표
      dist = P 에서 투영점까지 거리 (여러 선분 중 '가장 가까운 복도' 고르기 용)."""
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        t = 0.0                          # 길이 0 선분 — A 로 취급
    else:
        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    qx, qy = ax + t * dx, ay + t * dy
    return t, qx, qy, math.hypot(px - qx, py - qy)


def route_through_graph(sx, sy, tx, ty, graph):
    """복도 그래프 위 최단 경로 폴리라인 (07-19 — 순수 함수, 단위테스트 대상).

    왜: 직선 수식은 곁복도(분기) 화재 때 벽을 뚫는 집결지를 내놓는다(§16 한계).
    복도 '중심선'들을 그래프로 선언해 두면, 어떤 두 점 사이든 경로가
    항상 복도 위로만 지나간다 — Nav2 가 거부할 이유가 없는 지점만 나옴.

    동작: ① 시작·끝점을 가장 가까운 간선 위로 투영(관제 클릭이 벽 안이어도 복도로 끌어옴)
          ② 투영점을 임시 노드로 그래프에 붙이고 다익스트라 최단경로
          ③ 좌표 폴리라인 [(x,y), ...] 반환. 그래프가 비었거나 안 이어지면 None.

    graph = {'nodes': {이름: {'x','y'}}, 'edges': [[이름, 이름], ...]} (yaml 그대로)."""
    try:
        nodes = {n: (float(p['x']), float(p['y']))
                 for n, p in graph['nodes'].items()}
        edges = [(a, b) for a, b in graph['edges']
                 if a in nodes and b in nodes]
    except (KeyError, TypeError, ValueError):
        return None
    if not edges:
        return None

    # ① 시작(S)·끝(T)을 가장 가까운 간선 위로 투영
    def nearest_on_graph(px, py):
        best = None
        for a, b in edges:
            t, qx, qy, d = project_to_segment(px, py, *nodes[a], *nodes[b])
            if best is None or d < best[0]:
                best = (d, (a, b), (qx, qy))
        return best[1], best[2]          # (실린 간선, 투영 좌표)

    s_edge, s_pt = nearest_on_graph(sx, sy)
    t_edge, t_pt = nearest_on_graph(tx, ty)

    # ② 투영점을 임시 노드 '_S'/'_T' 로 붙인 인접 리스트 구성
    #    (같은 간선에 둘 다 실렸으면 그 간선 위 직결도 추가)
    adj = {n: [] for n in nodes}
    adj['_S'], adj['_T'] = [], []
    pos = dict(nodes, _S=s_pt, _T=t_pt)

    def connect(a, b):
        d = math.hypot(pos[a][0] - pos[b][0], pos[a][1] - pos[b][1])
        adj[a].append((b, d))
        adj[b].append((a, d))

    for a, b in edges:
        on = [v for v, e in (('_S', s_edge), ('_T', t_edge)) if e == (a, b)]
        if not on:
            connect(a, b)
        else:                            # 간선을 투영점에서 쪼개서 연결
            chain = sorted(on, key=lambda v: math.hypot(
                pos[v][0] - nodes[a][0], pos[v][1] - nodes[a][1]))
            for u, v in zip([a] + chain, chain + [b]):
                connect(u, v)

    # ③ 다익스트라 (그래프가 노드 몇 개라 성능 고려 불필요 — 정확성만)
    dist = {'_S': 0.0}
    prev = {}
    pq = [(0.0, '_S')]
    while pq:
        d, u = heapq.heappop(pq)
        if u == '_T':
            break
        if d > dist.get(u, math.inf):
            continue
        for v, w in adj[u]:
            nd = d + w
            if nd < dist.get(v, math.inf):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    if '_T' not in dist:
        return None                      # 그래프가 안 이어짐 (edges 선언 누락)

    path, cur = [], '_T'
    while True:
        path.append(pos[cur])
        if cur == '_S':
            break
        cur = prev[cur]
    path.reverse()
    return path


def compute_gather_point_graph(fx, fy, ex, ey, gather_dist, graph):
    """집결지 계산 — 복도 그래프판 (07-19, compute_gather_point 의 상위 호환).

    집결지 = 화재→탈출구 '그래프 최단 경로'를 따라 화재에서 gather_dist 만큼
    걸어간 지점. 직선판과 달리 곁복도 화재도 항상 복도 중심선 위의 도달
    가능한 지점이 나온다. yaw = 그 지점에서 탈출구로 가는 진행 방향.

    경로 전체가 gather_dist 보다 짧으면(화재가 탈출구 코앞) 탈출구로 클램프.
    그래프 계산 불가(빈 그래프·비연결·화재=탈출구)면 None → 호출부가
    직선판 → yaml 고정값 순으로 fallback."""
    path = route_through_graph(fx, fy, ex, ey, graph)
    if not path or len(path) < 2:
        return None
    total = sum(math.hypot(x1 - x0, y1 - y0)
                for (x0, y0), (x1, y1) in zip(path, path[1:]))
    if total < 1e-6:
        return None                      # 화재≈탈출구 — 진행 방향 정의 불가
    remaining = float(gather_dist)
    for (x0, y0), (x1, y1) in zip(path, path[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg < 1e-9:
            continue
        if remaining <= seg:
            r = remaining / seg
            return {'x': x0 + (x1 - x0) * r,
                    'y': y0 + (y1 - y0) * r,
                    'yaw': math.atan2(y1 - y0, x1 - x0)}
        remaining -= seg
    (x0, y0), (x1, y1) = path[-2], path[-1]   # 경로가 짧다 — 탈출구로 클램프
    return {'x': x1, 'y': y1, 'yaw': math.atan2(y1 - y0, x1 - x0)}


def validate_waypoints(wp):
    """waypoints.yaml 필수 키 검사 (fail-fast, 07-07 — 순수 함수, 단위테스트 대상).

    왜: 키가 빠져도 지금은 '그 키를 처음 쓰는 순간'(예: escape 는 GUIDE 진입)에야
    KeyError 로 죽음 = 미션 한복판 크래시. 설정 실수는 시작 순간에 잡아야 한다.
    검사 대상 = 코드가 wp['...'] 로 직접 인덱싱하는 키 전부
    (.get(기본값) 으로 읽는 선택 키는 제외 — gather_dist, cone_half_deg 등).
    반환: 빠진 키 경로 문자열 리스트 (빈 리스트 = 통과)."""
    missing = []

    def need(d, key, path):
        if not isinstance(d, dict) or key not in d:
            missing.append(path)
            return None
        return d[key]

    for k in ('gather_wait_sec', 'guide_speed', 'normal_speed'):
        need(wp, k, k)
    patrol = need(wp, 'patrol', 'patrol')
    if patrol is not None:
        if not isinstance(patrol, list) or not patrol:
            missing.append('patrol(리스트 아님/비어있음)')
        else:
            for n, p in enumerate(patrol):
                for k in ('x', 'y'):
                    need(p, k, f'patrol[{n}].{k}')
    for name in ('gather', 'escape'):
        pt = need(wp, name, name)
        if pt is not None:
            for k in ('x', 'y'):
                need(pt, k, f'{name}.{k}')
    sb = need(wp, 'search_back', 'search_back')
    if sb is not None:
        for k in ('max_attempts', 'min_fire_dist', 'refind_wait_sec'):
            need(sb, k, f'search_back.{k}')
    # corridor_graph 는 선택 키 — 있으면 구조까지 검사 (07-19).
    # 오타 난 간선 이름은 '그 화재가 처음 난 순간' 조용한 fallback 으로 숨어버림 → 시작에 검거.
    cg = wp.get('corridor_graph')
    if cg is not None:
        nodes = need(cg, 'nodes', 'corridor_graph.nodes')
        edges = need(cg, 'edges', 'corridor_graph.edges')
        if isinstance(nodes, dict):
            for n, p in nodes.items():
                for k in ('x', 'y'):
                    need(p, k, f'corridor_graph.nodes.{n}.{k}')
        elif nodes is not None:
            missing.append('corridor_graph.nodes(딕셔너리 아님)')
        if isinstance(edges, list):
            for i, e in enumerate(edges):
                if (not isinstance(e, list) or len(e) != 2
                        or not isinstance(nodes, dict)
                        or e[0] not in nodes or e[1] not in nodes):
                    missing.append(f'corridor_graph.edges[{i}](미선언 노드/형식 오류)')
        elif edges is not None:
            missing.append('corridor_graph.edges(리스트 아님)')
    return missing


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
        # ★ fail-fast (07-07): 필수 키 누락은 미션 한복판 KeyError 가 아니라
        #   시작 즉시, 어떤 키가 없는지 명확한 메시지로 죽는다.
        bad = validate_waypoints(self.wp)
        if bad:
            self.get_logger().fatal(
                f'waypoints.yaml 필수 키 누락: {", ".join(bad)} (파일: {wp_path})')
            raise SystemExit(1)
        self.get_logger().info(f'웨이포인트 로드: {wp_path}')

        # --- 통신 구성 ---
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.state_pub = self.create_publisher(String, '/mission_state', 10)
        self.siren_pub = self.create_publisher(Bool, '/siren', 10)
        self.create_subscription(PoseStamped, '/alarm', self.on_alarm, 10)
        # 관제 인터페이스 (07-07): reset = 처음부터 / abort = 즉시 정지 (FAULT 유지)
        # 현장 복구가 '프로세스 재시작'뿐이던 것을 토픽 한 줄로. (⚠ pub 은 -w 1, --once 금지)
        self.create_subscription(String, '/mission_cmd', self.on_cmd, 10)
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
            max_cluster_width=float(sb.get('cluster_max_width', 0.8)),
            # 아래 3개는 라이다 물리 특성(각해상도·노이즈)에 묶인 값 —
            # 실물 RPLIDAR C1 전환 시 1순위 재튜닝 대상이라 yaml 로 노출 (07-07)
            min_points=int(sb.get('min_points', 3)),
            range_jump=float(sb.get('range_jump', 0.3)),
            edge_margin=float(sb.get('edge_margin', 0.2)),
            # ★ watchdog (07-19): /scan 이 이 시간 넘게 끊기면 visible/lost 판정 보류
            #   (라이다 사망을 '추종 양호'로 오독 방지 — 실물 USB 라이다 대비)
            scan_timeout=float(sb.get('scan_timeout', 1.0)))
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
        self._speed_synced = False      # 시작 속도 동기화 완료 여부 (아래 tick 참조)

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
        handle = future.result()
        if seq != self.goal_seq:
            # ★ 취소 레이스 수정 (07-19 P0): send_goal 응답이 오기 전에
            #   cancel_current_goal()(알람·abort·역행)이 먼저 실행되면 그 시점엔
            #   핸들이 없어 취소할 게 없다. 그 goal 이 뒤늦게 수락되면 Nav2 는
            #   혼자 주행을 계속 — "abort 했는데 로봇이 안 멈춤"이 되는 구멍.
            #   → stale 응답은 '무시'가 아니라 '수락됐으면 즉시 취소'.
            if handle is not None and handle.accepted:
                self.get_logger().warn(
                    f'뒤늦게 수락된 이전 목표(seq={seq}) 즉시 취소 (현재 seq={self.goal_seq})')
                handle.cancel_goal_async()
            return
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
        # 끝난 goal 핸들은 즉시 비움 (07-19): 남겨두면 다음 cancel 이 '이미 끝난
        # 핸들'을 취소하는 헛손질을 하고, 위 stale-취소 경로와 조합 시 혼동 소지.
        self._goal_handle = None
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
        # ★ 시작 속도 동기화 (07-07): 주행속도는 controller_server(Nav2) 쪽에 저장되는
        #   원격 파라미터라, GUIDE(0.12) 도중 미션 노드만 재시작하면 새 PATROL 이
        #   저속을 물려받는다 (Nav2 가 계속 떠 있는 실차 운영 패턴에서 발생).
        #   → "PATROL 시작 = normal_speed" 전제를 남에게 맡기지 않고 직접 선언.
        #   서비스가 늦게 뜰 수 있어 준비될 때까지 tick 재시도 (블로킹 금지 §12.2).
        #   상태 전환의 속도 변경보다 같은 tick 안에서 항상 먼저 실행 → 덮어쓰기 없음.
        if not self._speed_synced and self.param_cli.service_is_ready():
            self.set_nav_speed(float(self.wp['normal_speed']))
            self._speed_synced = True

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
                # watchdog 발동 중이면 판정 자체가 보류 상태 — 관제가 알아야 할 이상
                if self.monitor.scan_stale():
                    self.get_logger().warn(
                        '⚠ /scan 끊김 — 추종감시 불가 (유도는 계속)',
                        throttle_duration_sec=5.0)
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
                    # ★ lost 타이머 재무장 (07-07): 리셋 없이 GUIDE 로 돌아가면
                    #   "마지막 목격 = 한참 전" 그대로라 다음 tick 에 lost 가 즉시 참
                    #   → 방금 10초 기다리다 떠나온 같은 last_seen 으로 즉시 2차 역행,
                    #   예산이 "같은 곳 두 번"으로 소진 (max_attempts 의 의도 붕괴).
                    #   재무장하면 2차는 새로 lost_sec 연속 미검출을 다시 채워야 나감.
                    self.monitor.reset('any')
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
            # 조용히 버리면 "알람 보냈는데 왜 무반응?"을 디버깅할 단서가 없다 (07-07).
            # 다중 화재·위치 갱신 대응은 시나리오 확정 후 과제 — 지금은 기록만.
            self.get_logger().warn(
                f'알람 수신했으나 상태 {self.state.name} — 무시 (PATROL 에서만 접수)',
                throttle_duration_sec=5.0)
            return
        self.fire = {
            'pos': (msg.pose.position.x, msg.pose.position.y),
            'kind': 'fire',             # 자리 예약
        }
        # ⓐ 집결지 계산 (07-06, 07-19 그래프판 승격): 화재→탈출구 경로 위 gather_dist 지점.
        #   1순위 = 복도 그래프(곁복도 화재도 벽 안 뚫는 지점 보장)
        #   2순위 = 직선 수식(그래프 미선언/계산 불가 시)
        #   3순위 = yaml 고정값 (APPROACH tick 의 기존 fallback)
        fx, fy = self.fire['pos']
        esc = self.wp['escape']
        gd = float(self.wp.get('gather_dist', 8.0))
        graph = self.wp.get('corridor_graph')
        self.gather_wp = None
        if graph is not None:
            self.gather_wp = compute_gather_point_graph(
                fx, fy, float(esc['x']), float(esc['y']), gd, graph)
            if self.gather_wp is None:
                self.get_logger().warn('복도 그래프 집결지 계산 불가 — 직선 수식으로 대체')
        if self.gather_wp is None:
            self.gather_wp = compute_gather_point(
                fx, fy, float(esc['x']), float(esc['y']), gd)
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

    def on_cmd(self, msg: String):
        """관제 명령 (07-07). 얇게 유지 — 명령 2개, 나머지는 무시+로그."""
        cmd = msg.data.strip().lower()
        if cmd == 'reset':
            # 임무 전체를 초기 상태로 — FAULT 소진·ESCAPED 후 재가동용
            self.get_logger().warn('★ 관제 reset — 임무 초기화 → PATROL')
            self.cancel_current_goal()
            self.state = State.PATROL
            self.patrol_idx = 0
            self.fire = None
            self.gather_wp = None
            self.gather_since = None
            self._escaped_logged = False
            self.search_attempts = 0
            self.give_up = False
            self.last_seen = None
            self.search_goal = None
            self.refind_since = None
            self.fault_retries = 0
            self.fault_since = None
            self.resume_state = None
            self.set_siren(False)
            self.set_nav_speed(float(self.wp['normal_speed']))
        elif cmd == 'abort':
            # 즉시 정지, 자동 재시도 없이 FAULT 유지 (복구는 reset 으로)
            self.get_logger().error('★ 관제 abort — 목표 취소, 정지 (재가동은 reset)')
            self.cancel_current_goal()
            self.set_siren(False)
            self.fault_retries = self.MAX_RETRIES   # 자동 재시도 차단
            self.resume_state = None
            self.fault_since = self.get_clock().now()
            self.state = State.FAULT
        else:
            self.get_logger().warn(f'알 수 없는 /mission_cmd: "{msg.data}" (reset|abort)')

    def enter_fault(self):
        if self.state != State.FAULT:
            self.resume_state = self.state
        self.state = State.FAULT
        self.fault_since = self.get_clock().now()
        if self.fault_retries >= self.MAX_RETRIES:
            self.get_logger().error(
                f'FAULT — 재시도 {self.MAX_RETRIES}회 소진, 정지. (사람 개입 필요)')
            # 정지로 끝나도 속도는 평시값 복원 — GUIDE 저속(0.12) 채로 남기지 않기 (07-07)
            self.set_nav_speed(float(self.wp['normal_speed']))
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
