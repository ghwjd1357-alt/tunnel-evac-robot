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

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.srv import SetParameters

import tf2_ros

from ament_index_python.packages import get_package_share_directory
from mission_manager.follower_monitor import FollowerMonitor
from mission_manager.speed_manager import SpeedManager
# ★ goal 전송·응답·취소·최종결과의 비동기 수명주기는 전부 GoalManager 소유
#   (07-23 구조 분리 2/3). 이 노드에는 정책(어디로 갈지)과 콜백만 남긴다.
from mission_manager.goal_manager import GoalManager


class State(Enum):
    PATROL = auto()       # 평시 순찰
    APPROACH = auto()     # 화재 → 집결지 이동 (싸이렌 ON)
    GATHER = auto()       # T초 집결 대기
    GUIDE = auto()        # 저속 선행 유도 + 후방 추종감시
    HOLD = auto()         # 🆕 08-22 — 놓침 확정 직후 **제자리에 서서** 라이다 재수집.
    #                       비싼 행동(역행) 전에 싼 행동(기다리기)을 먼저 한다.
    #                       근거: 08-21 21:32 리허설에서 SEARCH_BACK 이 두 번 떴고
    #                       **두 번 다 15초 만에** GUIDE 로 복귀했다(339.0→353.5 ·
    #                       365.5→381.0). 사람이 잠깐 안 보였을 뿐인데 180° 역행을
    #                       시작했다 취소한 것이다. 그 21초가 통째로 낭비였다.
    #                       FollowerMonitor 가 이미 쓰는 비대칭과 같은 철학이다 —
    #                       "놓침 선언(비싼 역행 유발)은 신중히, 재발견은 빠르게."
    SEARCH_BACK = auto()  # 놓침 → 마지막 목격 지점으로 역행 재탐색
    SCAN_AREA = auto()    # 🆕 08-22 — 집결지에서 **제자리로 한 바퀴 돌며** 사람을 찾는다.
    #                       🔴 cmd_vel 을 직접 내지 않는다 — 미션에는 그 발행자가 없고
    #                       tick 도 2 Hz 다. 대신 같은 (x,y) 에 yaw 만 돌린 Nav2 goal 을
    #                       `scan_steps` 번 낸다. 08-21 에 고친 회전 경로
    #                       (PoseProgressChecker · xy_goal_tolerance 0.25)를 그대로 쓴다.
    RESCUE = auto()       # 🆕 08-22 — 쓰러진 사람 확정. 관제에 신고하고 **그 자리에 선다.**
    #                       BLOCKED 과 같은 구조다(새 goal 을 안 낸다 = 정지).
    #                       탈출은 관제 `reset` 뿐이다 — 로봇이 스스로 "이제 됐다"
    #                       고 판단할 근거가 없기 때문이다.
    NO_VICTIM = auto()    # 🆕 08-22 — 집결지에 아무도 없다고 확정. 신고하고 선다.
    #                       🔴 구판은 이 경우가 **없었다** — gather_wait_sec 타이머만
    #                       보고 무조건 GUIDE 로 갔다. 즉 아무도 없는 복도를 앞장서서
    #                       걸어 나갔다(`mission_node.py` GATHER 분기, 08-22 이전).
    ESCAPED = auto()      # 탈출 완료
    FAULT = auto()        # Nav2 실패 → 자동 재시도 → 소진 시 정지
    BLOCKED = auto()      # 🔴 08-21 §83.6 — 안전한 집결지를 못 만들어 사람에게 넘긴 상태.
    #                       FAULT 와 다르다: FAULT 는 Nav2 가 실패한 것이고 **자동
    #                       재시도**가 있다. BLOCKED 는 계산이 안전 조건을 못 채운
    #                       것이라 재시도할 대상이 없다 — 사람이 판단할 때까지 선다.
    #                       tick 에 새 goal 을 내는 가지가 없으므로 로봇은 정지한다.


# 🆕 08-22 — HOLD 대기 시간의 기본값 [s]. `waypoints.yaml` 의
#   `search_back.hold_sec` 로 덮어쓴다. 🔵 `.get(기본값)` 으로 읽는 **선택 키**이므로
#   `validate_waypoints()` 의 필수 목록에 넣지 않는다 (그 함수 머리말의 규약).
#   ⚠ 촬영 중 키 하나가 없어서 미션 노드가 죽는 것보다, 의도한 값으로 도는 편이 낫다.
# 왜 4초인가: 역행 한 번이 180° 회전 약 22초 + 되돌아가는 거리를 쓴다. 그 앞에
#   4초를 서 보는 것은 1/5 값도 안 되는 보험이다. 그리고 `lost` 가 이미 3초
#   연속 미검출을 요구하므로, 사람이 정말 사라졌다면 여기서 7초를 쓴 셈이 된다.
HOLD_SEC_DEFAULT = 4.0

# 🆕 08-22 사람 판정 게이트 (`PROJECT_CONTEXT §4.1-b` 를 소비하는 쪽).
# 🔴 **기본이 꺼짐이어야 한다.** 본편 테이크는 `camera:=false` 라 어댑터가 아예 없고,
#   그러면 `/person_status` 가 한 건도 안 온다. 게이트를 무조건 걸면 그 상태가
#   'stale' 로 굳어 **본편이 RESCUE 로 빠진다** — 사람이 멀쩡히 서 있는데도.
#   예약 61 이 정한 방식과 같다: OR 로만 더하고 **기본 OFF**.
PERSON_GATE_DEFAULT = False
# 판정이 안 서는 채로 이만큼 더 기다리면 신고 쪽으로 간다. 🔴 모르면 신고다 —
# 판정 불가를 '괜찮다'로 접으면 쓰러진 사람을 두고 나간다.
PERSON_DECIDE_TIMEOUT_DEFAULT = 10.0
# 🔴 미션 자신의 신선도 가드. 어댑터는 자기 입력이 끊기면 'stale' 을 **말해 주지만**,
#   어댑터 자체가 죽으면 아무 말도 못 한다. 그때 마지막 'ok' 를 붙들고 있으면
#   미션은 사람이 있는 줄 안다. 10 Hz 발행이므로 1초면 충분히 관대하다.
PERSON_STATUS_TIMEOUT_DEFAULT = 1.0
# 🆕 제자리 훑기. 4 스텝 × 90° 가 기본이다.
# 🔴 스텝을 늘리면 그림은 부드러워지지만 goal 왕복이 그만큼 늘어 시간이 길어진다.
#   촬영 대본은 집결 장면에 40~60초를 쓸 수 있다 — 4 스텝이 그 안이다.
SCAN_STEPS_DEFAULT = 4
# 각 스텝 도착 뒤 서서 판정 프레임을 모으는 시간. 어댑터가 10 Hz 이므로 2초면
# 20 프레임이고, `person_min_frames`(6) 를 넉넉히 넘는다.
SCAN_DWELL_SEC_DEFAULT = 2.0
# 🔴 08-22 (§87.7) — 한 훑기 스텝의 **벽시계 상한**. 명시 실패(REJECTED/ABORTED)는
#   `GoalManager` 가 `enter_fault` 로 유한 종결하지만, **응답이나 결과가 영원히 안
#   오는 경우에는 상한이 없었다** — `goal_active=True` 가 무한 유지되고 미션이 그
#   자리에서 멈춘다. Nav2/action 통신이 반쯤 죽으면 네 스텝 중 하나에서 걸린다.
#   90초 = 90° 회전(약 11초) + 계획·여유. 넘으면 기존 FAULT 재시도 예산으로 보낸다.
SCAN_GOAL_TIMEOUT_DEFAULT = 90.0


def normalize_angle(a):
    """각을 (−π, π] 로 접는다.

    🔴 08-22 — 처음에 이 함수를 **정의하지 않고 호출했다.** 문법도 통과하고 회귀
    220 도 초록이었다 — 시험이 `SCAN_AREA` 경로를 한 번도 안 탔기 때문이다.
    실차에서는 훑기 첫 스텝에서 `NameError` 로 죽는다. 테이크 하나가 통째로 날아간다.
    🔵 **회귀가 없는 코드는 문법이 맞아도 안 돈다** 는 것을 이 자리가 증명한다.
    """
    return math.atan2(math.sin(a), math.cos(a))


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


def distance_to_graph(px, py, graph):
    """점에서 복도 그래프(간선들)까지의 최단거리 (S1-1 알람 검증용 순수 함수).

    관제 클릭이 복도에서 이만큼 떨어져 있으면 '지도 밖 오클릭'으로 의심할 근거.
    그래프 구조가 계산 불가면 None (호출부가 검사 생략 판단)."""
    try:
        nodes = {n: (float(p['x']), float(p['y']))
                 for n, p in graph['nodes'].items()}
        edges = [(a, b) for a, b in graph['edges']
                 if a in nodes and b in nodes]
    except (KeyError, TypeError, ValueError):
        return None
    if not edges:
        return None
    return min(project_to_segment(px, py, *nodes[a], *nodes[b])[3]
               for a, b in edges)


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

    def num_ok(v):
        # ★ bool 은 int 의 하위 타입 — true 가 숫자로 통과하는 것 차단 (그래프 검사와 동일 함정)
        return (not isinstance(v, bool) and isinstance(v, (int, float))
                and math.isfinite(v))

    def need_num(d, key, path, positive=False, nonneg=False, optional=False):
        """숫자 키 검사 (S1-3, 07-19 Codex §10.6 공격 재현 반영).

        기존 검사는 '키 존재'만 봐서 patrol.x=NaN·guide_speed=NaN·음수 대기시간이
        전부 통과 — 미션 한복판에서야 이상 거동. 타입·유한값·부호까지 시작에 검거."""
        if not isinstance(d, dict) or key not in d:
            if not optional:
                missing.append(path)
            return
        v = d[key]
        if not num_ok(v):
            missing.append(f'{path}(숫자 아님/NaN/inf: {v!r})')
        elif positive and v <= 0:
            missing.append(f'{path}(양수 아님: {v!r})')
        elif nonneg and v < 0:
            missing.append(f'{path}(음수: {v!r})')

    need_num(wp, 'gather_wait_sec', 'gather_wait_sec', nonneg=True)
    need_num(wp, 'guide_speed', 'guide_speed', positive=True)
    need_num(wp, 'normal_speed', 'normal_speed', positive=True)
    need_num(wp, 'gather_dist', 'gather_dist', positive=True, optional=True)
    need_num(wp, 'alarm_max_projection_dist', 'alarm_max_projection_dist',
             positive=True, optional=True)
    # 07-20: GUIDE 저속 서비스 미준비 timeout(초) — 없으면 코드 기본 30초
    need_num(wp, 'speed_unready_timeout_sec', 'speed_unready_timeout_sec',
             positive=True, optional=True)
    patrol = need(wp, 'patrol', 'patrol')
    if patrol is not None:
        if not isinstance(patrol, list) or not patrol:
            missing.append('patrol(리스트 아님/비어있음)')
        else:
            for n, p in enumerate(patrol):
                for k in ('x', 'y'):
                    need_num(p, k, f'patrol[{n}].{k}')
                need_num(p, 'yaw', f'patrol[{n}].yaw', optional=True)
    for name in ('gather', 'escape'):
        pt = need(wp, name, name)
        if pt is not None:
            for k in ('x', 'y'):
                need_num(pt, k, f'{name}.{k}')
            need_num(pt, 'yaw', f'{name}.yaw', optional=True)
    def need_int(d, key, path, minimum, optional=False):
        """정수 키 검사 (F6, 07-19 Codex §12.4): 횟수·개수는 0.5 같은
        소수가 오면 비교(>=)는 되지만 의미가 깨진다 — 정수+최솟값 강제."""
        if not isinstance(d, dict) or key not in d:
            if not optional:
                missing.append(path)
            return
        v = d[key]
        if isinstance(v, bool) or not isinstance(v, int):
            missing.append(f'{path}(정수 아님: {v!r})')
        elif v < minimum:
            missing.append(f'{path}(최소 {minimum} 미만: {v!r})')

    def need_bool(d, key, path, optional=True):
        """🔴 08-22 (§87.9) — **엄격한 bool**. 문자열을 받지 않는다.

        `person_gate: 'false'` 처럼 따옴표가 붙으면 파이썬에서 `bool('false')` 는
        **True** 다. 즉 오타 하나가 게이트를 **켠다** — 본편 테이크에서 어댑터가
        없는데 사람 판정이 돌아 `RESCUE` 로 빠진다. 검토가 이 자리를 실제로
        재현했다(7변이 전부 통과했다).
        """
        if not isinstance(d, dict) or key not in d:
            if not optional:
                missing.append(path)
            return
        if not isinstance(d[key], bool):
            missing.append(f'{path}(bool 아님 — 따옴표 없는 true/false: {d[key]!r})')

    # 🔴 08-22 (§87.9) — B 가 더한 설정 7종이 **fail-fast 검사를 전부 우회**했다.
    #   `hold_sec=NaN` 이면 `waited >= NaN` 이 영원히 false 라 **HOLD 에 갇힌다.**
    #   `scan_steps=true` 는 bool 이 int 하위형이라 통과해 0 나눗셈 근처로 간다.
    #   배포 5벌 값은 정상이지만, 촬영 직전 오타가 **기동 실패가 아니라 잘못된
    #   전이·무한 정지**로 나타나는 것이 이 검사가 막으려는 것이다.
    need_bool(wp, 'person_gate', 'person_gate')
    need_num(wp, 'person_decide_timeout_sec', 'person_decide_timeout_sec',
             positive=True, optional=True)
    need_num(wp, 'person_status_timeout_sec', 'person_status_timeout_sec',
             positive=True, optional=True)
    need_int(wp, 'scan_steps', 'scan_steps', minimum=1, optional=True)
    need_num(wp, 'scan_dwell_sec', 'scan_dwell_sec', nonneg=True, optional=True)
    need_num(wp, 'scan_goal_timeout_sec', 'scan_goal_timeout_sec',
             positive=True, optional=True)

    sb = need(wp, 'search_back', 'search_back')
    if sb is not None:
        # F6: 횟수·개수는 정수 강제 (0.5 회 시도·0.5 점 클러스터는 무의미)
        need_int(sb, 'max_attempts', 'search_back.max_attempts', minimum=0)
        need_num(sb, 'min_fire_dist', 'search_back.min_fire_dist', nonneg=True)
        need_num(sb, 'refind_wait_sec', 'search_back.refind_wait_sec',
                 positive=True)
        # 선택 튜닝 키 — 있으면 타입·부호까지 (라이다 물리 파라미터라 오타가 잦은 곳)
        for k in ('cone_half_deg', 'detect_range', 'lost_sec', 'seen_sec',
                  'cluster_max_width', 'range_jump', 'edge_margin',
                  'scan_timeout'):
            need_num(sb, k, f'search_back.{k}', positive=True, optional=True)
        need_int(sb, 'min_points', 'search_back.min_points',
                 minimum=1, optional=True)
        # 🆕 08-22 — HOLD 대기시간과 카메라 병행 스위치
        need_num(sb, 'hold_sec', 'search_back.hold_sec',
                 nonneg=True, optional=True)
        need_bool(sb, 'camera_refind', 'search_back.camera_refind')
        # F6: 상한·상호관계 (Codex §12.4 공격 재현 봉쇄) —
        #   cone_half_deg=999 는 부채꼴이 전방위를 넘고,
        #   edge_margin >= detect_range 는 판정 존이 사라져 '항상 놓침'이 된다.
        chd = sb.get('cone_half_deg') if isinstance(sb, dict) else None
        if num_ok(chd) and not (0 < chd <= 180):
            missing.append(f'search_back.cone_half_deg(0~180 범위 밖: {chd!r})')
        dr = sb.get('detect_range') if isinstance(sb, dict) else None
        em = sb.get('edge_margin') if isinstance(sb, dict) else None
        if num_ok(dr) and num_ok(em) and em >= dr:
            missing.append(
                f'search_back.edge_margin({em!r}) >= detect_range({dr!r}) — '
                f'판정 존이 0 이 됨')
    # corridor_graph 는 선택 키 — 있으면 구조까지 검사 (07-19).
    # 오타 난 간선 이름은 '그 화재가 처음 난 순간' 조용한 fallback 으로 숨어버림 → 시작에 검거.
    cg = wp.get('corridor_graph')
    if cg is not None:
        nodes = need(cg, 'nodes', 'corridor_graph.nodes')
        edges = need(cg, 'edges', 'corridor_graph.edges')
        if isinstance(nodes, dict):
            if not nodes:
                missing.append('corridor_graph.nodes(비어 있음)')
            for n, p in nodes.items():
                for k in ('x', 'y'):
                    v = need(p, k, f'corridor_graph.nodes.{n}.{k}')
                    # ★ 07-19 Codex §3.3: '키 존재'만 보면 x: "not-a-number" 가
                    #   통과 → 화재 순간 라우팅 실패 → 조용한 직선 fallback 으로
                    #   그래프 도입 목적(벽 안 집결지 방지)이 무력화. 숫자·유한값까지.
                    #   (bool 은 파이썬에서 int 의 하위 타입 — True 가 숫자로 통과하는 것 차단)
                    if v is not None and (isinstance(v, bool)
                                          or not isinstance(v, (int, float))
                                          or not math.isfinite(v)):
                        missing.append(
                            f'corridor_graph.nodes.{n}.{k}(숫자 아님/NaN/inf: {v!r})')
        elif nodes is not None:
            missing.append('corridor_graph.nodes(딕셔너리 아님)')
        if isinstance(edges, list):
            if not edges:
                missing.append('corridor_graph.edges(비어 있음)')
            for i, e in enumerate(edges):
                if (not isinstance(e, list) or len(e) != 2
                        or not isinstance(nodes, dict)
                        or e[0] not in nodes or e[1] not in nodes):
                    missing.append(f'corridor_graph.edges[{i}](미선언 노드/형식 오류)')
                elif e[0] == e[1]:
                    missing.append(f'corridor_graph.edges[{i}](자기 자신 간선: {e[0]})')
        elif edges is not None:
            missing.append('corridor_graph.edges(리스트 아님)')
        # 구조가 멀쩡할 때만 기하·연결성 검사 (위에서 걸렸으면 좌표를 못 믿음)
        if not any('corridor_graph' in m for m in missing):
            coord = {n: (p['x'], p['y']) for n, p in nodes.items()}
            adj = {n: set() for n in nodes}
            for i, (a, b) in enumerate(edges):
                if math.hypot(coord[a][0] - coord[b][0],
                              coord[a][1] - coord[b][1]) < 1e-9:
                    missing.append(f'corridor_graph.edges[{i}](길이 0 간선: {a}-{b})')
                adj[a].add(b)
                adj[b].add(a)
            # 연결성: 고립 구역이 있으면 그 구역의 화재는 라우팅 실패 →
            # 런타임에야 직선 fallback 으로 조용히 새는 걸 시작 시점에 검거
            seen, stack = set(), [next(iter(nodes))]
            while stack:
                u = stack.pop()
                if u not in seen:
                    seen.add(u)
                    stack.extend(adj[u] - seen)
            isolated = sorted(set(nodes) - seen)
            if isolated:
                missing.append(
                    f'corridor_graph(비연결 — 고립 노드: {", ".join(isolated)})')
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
        # 🆕 08-22 — 어댑터가 배달하는 사람 판정 (`PROJECT_CONTEXT §4.1-b`).
        #   🔵 미션은 어댑터의 존재를 모른다 — `/alarm` 과 같은 방식이다.
        self.create_subscription(String, '/person_status', self.on_person_status, 10)
        self.create_subscription(PoseStamped, '/victim', self.on_victim, 10)
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
        # 🆕 HOLD 진입 시각. None = HOLD 중이 아니다.
        self.hold_since = None
        # 🆕 사람 판정 (08-22)
        self.person_status = 'stale'   # 🔴 기동 직후는 '없음'이 아니라 '못 봤다'
        self.person_status_t = None    # 마지막 수신 시각 (미션 자체 신선도 가드)
        self.victim = None             # /victim 좌표 (신고에 싣는다)
        self._rescue_logged = False
        self._no_victim_logged = False
        self.scan_idx = 0              # 지금 몇 번째 훑기 스텝인가
        self.scan_dwell_since = None   # 스텝 도착 뒤 관측 시작 시각 (None = 이동 중)
        # 🔴 08-22 (§87.8) — sweep 한 바퀴 동안 **본 것을 모은다.** 없으면 마지막
        #   카메라 방향이 앞의 관측을 덮어, 중간 사분면에만 있는 사람을 보고도
        #   "사람 없음" 으로 보고한다(검토 재현: none→ok→none→none → NO_VICTIM).
        self.scan_seen = set()
        self.scan_verdict = None       # sweep 종료 시 확정 — GATHER 가 소비한다
        self.scan_goal_since = None    # 이번 스텝 goal 을 낸 시각 (§87.7 상한 기산)
        # ★ 직전 tick 이 '실제로 분기시킨' 상태 (08-01 검토 §26 P1 — GUIDE 진입 감지용).
        #   resume_state(FAULT 복귀 목적지)와 혼동 금지: 이건 순수한 전이 감지 재료다.
        self._prev_tick_state = None
        self.patrol_idx = 0
        # ★ goal 수명주기(전송·수락·취소확인·결과감시·stale 세대 구분·유도정지
        #   종결 직렬화)는 전부 GoalManager 소유 (07-23 구조 분리 2/3).
        #   이 노드에는 goal_active(정책 입력 미러)와 "어디로 갈지"만 남는다.
        #   goal_active 는 Manager 가 on_active 콜백으로 미러해 준다(진짜 원본은
        #   Manager._active) — tick 의 정책 분기가 읽는 평범한 속성으로 유지.
        self.goal_active = False
        # 취소 의도 힌트: cancel_current_goal 은 tick·콜백에서 인자 없이 불리므로
        #   (동결 테스트가 0-인자 가짜로 덮음) 의도는 이 전이 속성으로 전달한다.
        #   'guide_stop'(저속상실 정지) / 'hard'(reset·abort) / None(일반 취소).
        self._cancel_intent = None
        self.goals = GoalManager(
            self.nav, self.get_logger(),
            on_reached=self.on_reached,
            on_fault=self.enter_fault,
            on_stop_unconfirmed=self.on_safety_stop_unconfirmed,
            on_stop_confirmed=self.on_safety_stop_confirmed,
            on_active=lambda v: setattr(self, 'goal_active', v))
        self.gather_since = None
        self._escaped_logged = False
        self.siren_on = False
        self.fire = None                # funnel 번역된 화재 정보
        self.blocked_reason = None      # §83.6 BLOCKED 사유 (관제·로그용)
        self._blocked_logged = False
        # 🔴 §84.2 — BLOCKED 는 "새 goal 을 안 낸다" 만으로 정지가 아니다.
        #   기존 goal 이 실제로 CANCELED 로 끝났는지까지 봐야 "멈췄다" 이다.
        #   'none'        취소할 goal 이 애초에 없었다 = 정지 상태
        #   'pending'     취소는 보냈고 종결 대기 — **아직 멈췄다고 말하지 않는다**
        #   'confirmed'   CANCELED 종결 관찰 = 정지 완료
        #   'unconfirmed' 취소가 거절·예외·비-CANCELED 종결 — 🔴 사람이 세워야 한다
        self.stop_state = None
        self.gather_wp = None           # 화재 좌표로 계산한 집결지 (없으면 yaml 고정값)
        # ★ 속도 변경의 비동기 수명주기(요청·확인·3회 재시도·stale 세대 구분·
        #   reconcile)는 전부 SpeedManager 소유 (07-20 구조 분리 1/3).
        #   이 노드에는 "어떤 상태에서 어떤 속도를 원한다"는 정책 + 콜백만 남긴다.
        #   F2 불변조건 유지: GUIDE 진입은 저속 '성공 확인' 후에만 (콜백이 전환).
        #   07-20 사용자 결정: 서비스 장기 미준비는 timeout(기본 30초) 후 FAULT.
        self.speed = SpeedManager(
            self.param_cli, self.get_logger(),
            on_guide_confirmed=self._on_guide_speed_ok,
            on_guide_failed=self._on_guide_speed_fail,
            unready_timeout_sec=float(
                self.wp.get('speed_unready_timeout_sec', 30.0)))
        self._guide_pending = False         # GUIDE 저속 적용 확인 대기 중 (GATHER 유지)

        # --- SEARCH_BACK 관리 ---
        self.search_attempts = 0        # 역행 시도 횟수 (안전장치 ①)
        self.give_up = False            # 제한 초과 → 단독 탈출 모드
        # ★ last_seen 의 실제 의미 (08-01 검토 §26 P2 정정 — 구 주석은 '마지막 목격
        #   시점의 좌표'라고 했으나 코드와 다르다): **역행 목표의 근사값으로 쓰는
        #   로봇 자신의 map 좌표**이며, 갱신 조건은 "이번 GUIDE 세대에서 놓침이
        #   확정되기 전 · /scan 이 신선한 tick"이다. 검출 성공의 증거가 아니다 —
        #   ① 검출이 끊긴 뒤에도 lost 확정 직전까지 갱신되고(그래서 첫 놓침의 목표는
        #      마지막 검출 지점보다 최대 lost_sec×guide_speed 만큼 앞선다),
        #   ② 세대 재무장(GUIDE 진입 · SEARCH_BACK 복귀 2곳) 뒤에는 실제 재검출이
        #      0건이어도 갱신된다.
        #   근사가 허용되는 이유는 추종자가 ~1.2m 뒤라는 전제뿐이다. 이 값을
        #   '사람을 봤다'는 관측 증거로 읽고 상태 전이를 만들면 안 된다.
        self.last_seen = None           # (x, y) — 위 규약대로만 해석할 것
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
    # Nav2 목표 전송 (리모컨) — 수명주기는 GoalManager, 여기는 얇은 위임만
    # ===========================================================
    def send_goal(self, wp, tag=''):
        """목적지 정책만 전달 — 전송·응답·결과 감시는 Manager 가 소유.
        (상태 이름은 로그용으로만 넘긴다.)"""
        self.goals.send_goal(wp, tag=tag, state_name=self.state.name)

    def cancel_current_goal(self):
        """현재 goal 취소 — 상위 의도(_cancel_intent)를 Manager 로 전달.
        의도는 이 호출 직전에 세팅되고 여기서 소비 후 즉시 비운다(다음 취소로
        새지 않게). tick·콜백에서 인자 없이 불리는 호출 규약을 유지한다."""
        intent = self._cancel_intent
        self._cancel_intent = None
        self.goals.cancel_current_goal(intent=intent)

    # ===========================================================
    # 도달 시 상태 전이
    # ===========================================================
    def on_reached(self):
        # 도착 = 진전 → FAULT 재시도 예산 리셋 (구 on_result 성공 경로에서 이관 —
        #  goal 성공은 SUCCEEDED 일 때만 on_reached 를 부르므로 동작 동일).
        self.fault_retries = 0
        if self.state == State.PATROL:
            self.patrol_idx = (self.patrol_idx + 1) % len(self.wp['patrol'])

        elif self.state == State.APPROACH:
            # 🆕 08-22 — 사람 판정을 쓸 때만 훑는다. 🔴 게이트가 꺼져 있으면
            #   (본편 테이크) 구판과 한 글자도 다르지 않게 GATHER 로 간다.
            if self.person_gate_on():
                self.enter_scan_area()
            else:
                self.state = State.GATHER
                self.gather_since = self.get_clock().now()
                self.get_logger().info(
                    f'집결지 도착 → GATHER: {self.wp["gather_wait_sec"]}초 집결대기')

        elif self.state == State.SCAN_AREA:
            # 한 스텝 도착 — 그 자리에 서서 판정 프레임을 모은다.
            self.scan_dwell_since = self.get_clock().now()
            self.get_logger().info(
                f'훑기 {self.scan_idx + 1}/{self.scan_steps()} 도착 — '
                f'{self.scan_dwell_sec():.1f}초 관측')

        elif self.state == State.GUIDE:
            self.set_siren(False)
            self.speed.request_restore(float(self.wp['normal_speed']))
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
        #   inflight/cooldown/준비 감시는 SpeedManager 내부 (ensure_sync 는 멱등).
        #   상태 전환의 속도 변경보다 같은 tick 안에서 항상 먼저 실행 → 덮어쓰기 없음.
        self.speed.tick()

        # ★ 07-23 §13 P1 backstop — guide 저속 복구가 끝내 소진됐는데 유도 활성
        #   상태(GUIDE/SEARCH_BACK)면, 실패 통보 콜백이 상태 전환 중 유실됐어도
        #   여기서 매 tick 잡아 cancel+FAULT 한다. 그러지 않으면 GUIDE 복귀 후
        #   live 게이트가 신규 goal 은 막아도(§24) FAULT 없이 영구 정지 = 고장 은폐.
        #   ★ ensure_sync 앞에 둔다 — sync 요청이 _inflight=True 로 술어를 가리면
        #     이 tick 을 놓친다. return 하지 않는다 — enter_fault 가 state 를 FAULT 로
        #     바꿔 아래 GUIDE/SEARCH_BACK 분기는 자동으로 건너뛰고, 같은 tick 에
        #     FAULT 상태가 발행된다(SEARCH_BACK 은 §22.3 과 동일 취급: 소진=cancel+FAULT).
        if (self.state in (State.GUIDE, State.SEARCH_BACK)
                and self.speed.guide_speed_recovery_exhausted):
            self.get_logger().error(
                '★ GUIDE 저속 복구 소진 — 유도 불가, 정지(FAULT)')
            # ★ B(07-23): 이 취소는 CANCELED 종결까지 신규 goal 을 봉쇄한다 —
            #   저속이 다시 확인돼도 옛 목표가 확실히 멈춘 뒤에만 다음 명령.
            self._cancel_intent = 'guide_stop'
            self.cancel_current_goal()
            self.enter_fault()

        self.speed.ensure_sync(float(self.wp['normal_speed']))

        # 상태·싸이렌 상시 발행
        m = String()
        m.data = self.state.name
        self.state_pub.publish(m)
        b = Bool()
        b.data = self.siren_on
        self.siren_pub.publish(b)

        # ★ GUIDE 진입 = 추종 관측 '세대'의 시작 (08-01 검토 §26 P1 봉합)
        #   ─────────────────────────────────────────────────────────────
        #   결함: 놓침 타이머(FollowerMonitor._last_seen_t)는 상태 경계를
        #   모른다. /scan 은 상태와 무관하게 계속 들어오므로(on_scan) GUIDE 로
        #   들어오는 순간 타이머가 이미 '남의 상태에서' 만료돼 있을 수 있다.
        #   그러면 GUIDE 첫 tick 이 lost=True 로 시작한다 — 아래 GUIDE 분기는
        #   lost 를 '먼저' 읽고 그 뒤에만 기록하므로(elif) TF 가 멀쩡해도
        #   record_last_seen() 호출 기회가 **0회**다. 그 상태에서 ②′ 안전망은
        #   last_seen is None 을 "GUIDE 내내 TF 가 안 풀렸다"로 오분류해
        #   예산 2회를 3 tick 만에 태우고 give_up(단독 탈출)으로 넘어간다.
        #   실측(검토 §26.2 재현): 전환 후 1.5초 만에 SEARCH_BACK **0회**로
        #   attempts 2/2 소진 · tf_calls 0 — 역행 한 번 없이 사람을 버렸다.
        #   반대 경계도 같은 뿌리다: GATHER 부터 한 프레임도 못 본 경우
        #   _last_seen_t 가 None 이라 lost 가 **영원히 False** → 역행도 보고도
        #   열리지 않고 escape 만 계속된다.
        #   ─────────────────────────────────────────────────────────────
        #   불변조건: **놓침 판정은 그 GUIDE 구간에서 관측한 시간으로만 한다.**
        #   왜 진입 경로마다가 아니라 여기 한 곳인가 — 아래 저속 fail-closed
        #   게이트가 이미 같은 논증을 했다(경로를 세지 말고 '시작하는 지점'을
        #   막는다). grep 전수로 GUIDE 로 들어오는 자리는 4곳이고
        #   (SEARCH_BACK 재발견 · SEARCH_BACK 대기실패 · GATHER 저속확인 콜백 ·
        #   FAULT resume_state 복귀) 그중 재무장하던 곳은 2곳뿐이었다.
        #   전이 감지로 걸면 남은 2곳과 **앞으로 생길 경로**가 자동으로 덮인다.
        #   관제 reset 은 last_seen 만 비우고 모니터를 안 건드렸는데, 다음 임무의
        #   GUIDE 진입이 여기를 지나므로 그 잔재도 같이 닫힌다.
        #   ⚠ 'any' 만 재무장한다 — GUIDE 추종감시가 쓰는 zone 이 'any' 하나다.
        #   ⚠ 사용자 정책 결정 (08-01): 한 번도 못 본 경우도 **다른 놓침과 동일
        #     취급**한다. reset 은 _last_seen_t 를 None→now 로 바꾸므로 모니터의
        #     "본 적 없으면 판단 보류"가 GUIDE 구간에서만 해제된다 — 유도 중의
        #     '판단 보류'는 곧 무한 방치이기 때문이다. 그 결과 never-seen 은
        #     역행 2회 → '추종자 확인 불가' 보고 → 단독 탈출로 **유한하게** 끝난다.
        #   ⚠ 이 재무장은 **관측 없이** 기산점을 세운다. 그래서 라이다가 아직 한 번도
        #     안 살아난 상태에서 여기를 지나면 첫 유효 스캔까지의 시간이 미검출
        #     시간으로 샌다 — 08-02 검토 §27 P1. 그 누수는 FollowerMonitor.update()
        #     의 단절 복구가 **최초 스캔까지** 덮도록 고쳐서 막았다.
        #     **이 자리의 안전은 그 보호에 의존한다** — 둘을 따로 손대지 말 것.
        if self.state == State.GUIDE and self._prev_tick_state != State.GUIDE:
            self.monitor.reset('any')    # [reset-role] guide-entry
        # 이 tick 이 실제로 분기시킬 상태를 기록한다. 분기 도중 바뀐 상태(예:
        # SEARCH_BACK→GUIDE)는 다음 tick 에서 '진입'으로 잡힌다 — 그래서 스냅샷을
        # 분기 '앞'에서 뜬다. (분기 뒤에 뜨면 같은 tick 안의 전이가 삼켜진다.)
        self._prev_tick_state = self.state

        if self.state == State.PATROL:
            if not self.goal_active:
                self.send_goal(self.wp['patrol'][self.patrol_idx], tag='patrol')

        elif self.state == State.APPROACH:
            if not self.goal_active:
                # 계산된 집결지 우선, 계산 불가였으면 yaml 고정값 (fallback)
                self.send_goal(self.gather_wp or self.wp['gather'], tag='gather')

        elif self.state == State.SCAN_AREA:
            # 🔵 **쓰러진 사람이 확정되면 더 돌 이유가 없다.** 즉시 신고로 간다 —
            #   나머지 스텝을 마저 도는 동안 사람은 계속 쓰러져 있다.
            if self.fresh_person_status() == 'fallen':
                self.get_logger().error('🔴 훑는 중 쓰러진 사람 확정 → RESCUE (남은 스텝 생략)')
                self.enter_rescue()
            elif self.scan_dwell_since is None:
                # 이동 중 — goal 이 없으면 이번 스텝의 goal 을 낸다.
                if not self.goal_active:
                    self.send_goal(self.scan_goal(), tag='scan')
                    self.scan_goal_since = self.get_clock().now()
                elif self.scan_goal_since is not None:
                    # 🔴 §87.7 — 응답·결과가 영원히 안 오는 경우의 상한.
                    #   명시 실패는 GoalManager 가 FAULT 로 보내지만, 아무 콜백도
                    #   안 오면 여기 말고는 아무도 시간을 세지 않는다.
                    waited = (self.get_clock().now()
                              - self.scan_goal_since).nanoseconds / 1e9
                    if waited >= float(self.wp.get('scan_goal_timeout_sec',
                                                   SCAN_GOAL_TIMEOUT_DEFAULT)):
                        self.get_logger().error(
                            f'🔴 훑기 {self.scan_idx + 1} 스텝이 {waited:.0f}초째 '
                            f'응답이 없다 — FAULT 로 보낸다')
                        self.scan_goal_since = None
                        self._cancel_intent = 'safety_stop'
                        self.cancel_current_goal()
                        self.enter_fault()
            else:
                # 🔴 관측 창 동안 본 것을 **모은다** (§87.8). 마지막 방향이 앞의
                #   관측을 지우면 훑기를 하는 의미가 없다.
                live = self.fresh_person_status()
                if live in ('ok', 'unknown', 'none'):
                    self.scan_seen.add(live)
                waited = (self.get_clock().now()
                          - self.scan_dwell_since).nanoseconds / 1e9
                if waited >= self.scan_dwell_sec():
                    self.scan_idx += 1
                    self.scan_dwell_since = None
                    self.scan_goal_since = None
                    if self.scan_idx >= self.scan_steps():
                        # 🔴 sweep 전체를 하나의 판정으로 접는다 (§87.8).
                        #   보수 우선: 한 방향이라도 `unknown` 이면 떠나지 않는다.
                        #   (`unknown` 은 "사람 같은 걸 봤는데 자세를 못 믿는다" 다 —
                        #    벽은 후보가 없어 `none` 이므로 여기 안 온다.)
                        if 'unknown' in self.scan_seen:
                            self.scan_verdict = 'unknown'
                        elif 'ok' in self.scan_seen:
                            self.scan_verdict = 'ok'
                        elif 'none' in self.scan_seen:
                            self.scan_verdict = 'none'
                        self.get_logger().info(
                            f'훑기 관측 {sorted(self.scan_seen) or "없음"} '
                            f'→ 판정 {self.scan_verdict or "보류"}')
                        # 한 바퀴 다 돌았다 → 판정은 GATHER 가 한다(이미 있는 분기).
                        self.state = State.GATHER
                        self.gather_since = self.get_clock().now()
                        self.get_logger().info(
                            f'훑기 완료 → GATHER: '
                            f'{self.wp["gather_wait_sec"]}초 집결대기')

        elif self.state == State.GATHER:
            elapsed = (self.get_clock().now() - self.gather_since).nanoseconds / 1e9
            if elapsed >= float(self.wp['gather_wait_sec']) and not self._guide_pending:
                # ★ F2 (Codex §12.3): GUIDE 진입은 저속 '적용 확인' 후 —
                #   전환은 _on_guide_speed_ok 성공 콜백이 한다.
                #   실패(3회/예외/미준비 timeout)면 평시 0.26 으로 유도하는 대신 FAULT.
                #   확인까지 로봇은 GATHER 로 정지 상태 = 안전.
                #   서비스 미준비 대기·timeout 판정은 SpeedManager 가 담당.
                # 🆕 08-22 — 사람 판정 게이트. 🔴 **기본은 꺼짐**이라 본편
                #   (`camera:=false`, 어댑터 없음)은 구판과 한 글자도 다르지 않게 돈다.
                if not self.person_gate_on():
                    self._enter_guide_lowspeed()
                else:
                    waited = elapsed - float(self.wp['gather_wait_sec'])
                    verdict = self.person_verdict(waited)
                    if verdict == 'guide':
                        self._enter_guide_lowspeed()
                    elif verdict == 'rescue':
                        self.get_logger().error(
                            f'🔴 집결지 판정 = 쓰러진 사람 → RESCUE '
                            f'(상태 {self.fresh_person_status()}, 대기 {waited:.1f}s)')
                        self.enter_rescue()
                    elif verdict == 'no_victim':
                        self.get_logger().error(
                            f'🔴 집결지 판정 = 사람 없음 → NO_VICTIM (대기 {waited:.1f}s)')
                        self.enter_no_victim()
                    else:
                        # 판정 보류 — 🔵 로봇은 GATHER 에 **서 있다**(goal 을 안 낸다).
                        self.get_logger().warn(
                            f'집결지 사람 판정 보류 ({self.fresh_person_status()}) '
                            f'— 대기 {waited:.1f}s',
                            throttle_duration_sec=3.0)

        elif self.state == State.GUIDE:
            # ★ fail-closed 게이트 (07-20 재검토 §11.3 P1) — 저속이 '적용 확인'되기
            #   전에는 유도 주행 goal 을 단 한 건도 보내지 않는다.
            #   왜 진입 경로가 아니라 여기서 막나: FAULT 자동복귀·재발견 복귀 등
            #   GUIDE 로 들어오는 길이 여럿이고, 07-20 에 그 중 하나를 세 번 연속
            #   놓쳤다. 경로를 세는 대신 '주행을 시작하는 지점' 하나를 막으면
            #   앞으로 생길 경로까지 자동으로 덮인다.
            #   요청을 보낸 것 ≠ 적용된 것 (AGENTS.md §3-3 — 호출≠접수≠종결≠실효).
            #   ★ §12 P1: 술어는 latch(과거 1회 성공)가 아니라 live(지금 저속 적용)
            #   여야 한다 — guide_speed_applied. 늦은 sync 0.26 이 controller 를 덮으면
            #   즉시 False 가 되어 새 escape 가 안 나간다. 이미 주행 중인 goal 은 여기서
            #   멈추지 않는다 — 그 정지는 §22.3(reconcile→소진 시 cancel+FAULT)·B(GoalManager).
            if not self.speed.guide_speed_applied:
                self.get_logger().warn(
                    '⚠ GUIDE 저속 미적용 — 유도 주행 보류 (적용 확인 후 출발)',
                    throttle_duration_sec=5.0)
            elif not self.goal_active:
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
                stale = self.monitor.scan_stale()
                if stale:
                    self.get_logger().warn(
                        '⚠ /scan 끊김 — 추종감시 불가 (유도는 계속)',
                        throttle_duration_sec=5.0)
                # ★ ②′ 술어 불일치 봉합 (08-01 예약 16 — 0730_현황.md §2.5)
                #   구판은 visible(엄격·1초 연속 검출)이 last_seen 을 '쓰고'
                #   lost(관대·3초 미검출)가 그걸 '읽었다'. 검출이 깜빡이면
                #   visible 은 한 번도 참이 안 되는데 lost 는 참이 된다 →
                #   last_seen=None 인 채 놓침 확정 → enter_search_back 이 조용히
                #   return → 시도 횟수를 안 깎으니 give_up('관제 보고: 추종자
                #   확인 불가')에 영영 도달하지 못한다. 그동안 /mission_state 는
                #   GUIDE 를 계속 발행하고 escape goal 도 살아 있다 =
                #   "놓친 걸 알면서 돌아가지도 알리지도 않고 혼자 나간다."
                #   불변조건: 기록은 그 기록을 소비하는 술어와 같은 타이머를 쓴다.
                #   → last_seen 을 '읽는' 곳은 enter_search_back 하나뿐이고
                #     (전수 근거 = grep -n "self.last_seen" — 나머지는 초기화 2·
                #     기록 1) 그 진입 술어가 lost 이므로, 기록 조건도 lost 가
                #     보는 타이머에 맞춘다. ⚠ 줄번호는 적지 않는다 (곧 어긋난다).
                #   ⚠ 안전 술어 visible 자체는 건드리지 않는다 (③ 철회) — 그
                #     비대칭은 버그가 아니라 설계다("따라온다" 오판 = 사람 유기).
                #     last_seen 은 로봇 자기 좌표 저장일 뿐 안전 판단이 아니라서
                #     1초 연속 검출로 잠글 이유가 없었던 것뿐이다.
                #   ⚠ stale 중엔 기록도 보류한다: lost 는 stale 중 False 를 주므로
                #     'not lost' 만으로 쓰면 라이다가 죽은 동안 목격 지점이 로봇을
                #     따라 계속 전진해 역행 목표가 무의미해진다.
                if self.monitor.lost(zone='any'):
                    # 🆕 08-22 — 곧바로 역행하지 않는다. 먼저 **제자리에 서서** 본다.
                    #   `record_last_seen` 이 바로 위에서 멈췄으므로 목격 지점은
                    #   HOLD 동안 그대로 유지된다 — 로봇이 안 움직이니 오히려 정확하다.
                    self.enter_hold()
                elif not stale:
                    self.record_last_seen()       # 놓치기 전까지 위치 갱신

        elif self.state == State.HOLD:
            # 🔴 08-22 P0 — 정지가 **확인되기 전에는** 아무 판단도 하지 않는다.
            #   구판은 취소를 부르자마자 "서 있다"고 전제하고 4초를 셌다.
            if not self.stop_confirmed():
                if self.stop_state == 'unconfirmed':
                    # 취소가 끝내 종결되지 않았다 = 로봇이 아직 달릴 수 있다.
                    # 🔴 역행으로 넘어가지 않는다(신규 goal 금지 유지). 사람이 본다.
                    self.get_logger().error(
                        '🔴 HOLD · 정지 미확인 — 로봇이 아직 달리고 있을 수 있다. '
                        '**E-stop 으로 물리 정지를 확인할 것.**',
                        throttle_duration_sec=2.0)
                else:
                    self.get_logger().warn(
                        '🔶 HOLD · 정지 종결 대기 — 아직 "멈췄다" 로 읽지 말 것',
                        throttle_duration_sec=2.0)
            elif self.hold_since is None:
                # 이제서야 "서 있다" 가 참이다 — 여기서 기산을 연다.
                self.hold_since = self.get_clock().now()
            # 🆕 08-22 — 로봇은 이미 서 있다(`enter_hold` 가 goal 을 취소했다).
            #   🔴 여기서는 goal 을 하나도 내지 않는다 — 그것이 '정지'의 구현이다
            #   (`BLOCKED` 과 같은 구조. 검증된 패턴이다).
            elif self.monitor.visible(zone='any'):
                # 🎯 역행을 안 하고 끝났다. 08-21 리허설의 두 번이 이 가지로 왔을 것이다.
                self.get_logger().info('★ 제자리에서 재발견 → GUIDE 복귀 (역행 안 함)')
                self.monitor.reset('any')   # [reset-role] hold-return — 복귀 즉시 재-놓침 방지
                self.hold_since = None
                self.state = State.GUIDE
            elif self.monitor.scan_stale():
                # ⚠ 라이다가 죽은 동안은 **시간을 세지 않는다.** 안 그러면 센서가
                #   죽었다는 이유로 역행이 시작된다 — 그때 역행은 눈 감고 도는 것이다.
                #   같은 취지의 보류가 GUIDE 의 `record_last_seen` 에도 걸려 있다.
                self.hold_since = self.get_clock().now()
                self.get_logger().warn(
                    '⚠ /scan 끊김 — HOLD 대기시간 보류 (역행으로 넘어가지 않는다)',
                    throttle_duration_sec=5.0)
            else:
                waited = (self.get_clock().now()
                          - self.hold_since).nanoseconds / 1e9
                hold_sec = float(self.wp['search_back'].get(
                    'hold_sec', HOLD_SEC_DEFAULT))
                if waited >= hold_sec:
                    self.get_logger().info(
                        f'제자리 {waited:.1f}초 재수집 실패 → 역행 시작')
                    self.enter_search_back()
                    # 🔴 `enter_search_back` 은 시도 소진·give_up 이면 **상태를 안 바꾸고
                    #   그냥 돌아온다.** 구판에서는 호출자가 GUIDE 였으므로 그대로
                    #   단독 탈출로 이어졌다. HOLD 에서 부르면 그 경로가 **HOLD 에
                    #   갇힌다** — 로봇이 영원히 서 있는다. 그래서 되돌린다.
                    if self.state == State.HOLD:
                        self.hold_since = None
                        self.state = State.GUIDE

        elif self.state == State.SEARCH_BACK:
            # 🆕 08-22 예약 61 — 카메라 병행 재발견. 🔴 **OR 로만 더한다.**
            #   라이다 판정을 대체하지 않는다 — 라이다는 빛과 무관하고 3 m 까지
            #   실측으로 검증됐지만, 카메라의 그 거리·저조도는 아직 미검증이다
            #   (역할 B 08-18 G5 표: 원거리 ❌). 기본은 꺼짐이다.
            cam = self.camera_refind_status()
            if cam == 'fallen':
                # 🔴 역행 중에 **쓰러진 사람이 보인다.** 유도로 복귀하면 그를 두고
                #   나가는 것이다. 보이는데 지나치지 않는다.
                self.get_logger().error('🔴 역행 중 쓰러진 사람 확정 → RESCUE')
                self.enter_rescue()
            elif self.monitor.visible(zone='any') or cam == 'ok':
                # ★ 재발견 → 유도 재개
                self.get_logger().info('★ 추종자 재발견 → GUIDE 복귀')
                self.cancel_current_goal()
                self.refind_since = None
                self.monitor.reset('any')    # [reset-role] refind-return — 복귀 즉시 재-놓침 방지
                self.state = State.GUIDE
            elif not self.goal_active and self.refind_since is None:
                # ★ 07-23 §14 P1 — SEARCH_BACK 도 guide 유도 임무의 일부이므로
                # 신규 역행 goal 은 저속이 controller 에 실제 적용된 뒤에만 보낸다.
                # request_guide/reconcile 를 '호출'한 것만으로는 부족하다:
                # 응답 대기 중 _applied=0.26 이면 평시속도로 새 goal 이 출발한다.
                # 이미 주행 중인 goal(goal_active=True)은 건드리지 않아 §22.3의
                # "일시 표류는 reconcile 먼저, 소진 시 cancel+FAULT"를 보존한다.
                if not self.speed.guide_speed_applied:
                    self.get_logger().warn(
                        '⚠ SEARCH_BACK 저속 미적용 — 역행 주행 보류 '
                        '(적용 확인 후 출발)',
                        throttle_duration_sec=5.0)
                else:
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
                    self.monitor.reset('any')    # [reset-role] refind-timeout-return
                    self.state = State.GUIDE   # 놓친 채 계속 — 재놓침 판정은 GUIDE 가 함

        elif self.state == State.ESCAPED:
            if not self._escaped_logged:
                self.get_logger().info('★ 탈출 완료 — 임무 종료.')
                self._escaped_logged = True

        elif self.state == State.BLOCKED:
            # 🔴 §83.6 — 아무 goal 도 안 낸다. 자동 복귀도 없다. 관제 `reset` 만이
            #   여기서 나가는 길이다 — 그것이 "사람에게 넘긴다" 의 실제 구현이다.
            # 🔴 §84.2 — 그런데 "새 goal 을 안 낸다" 는 **필요조건일 뿐**이다.
            #   기존 goal 의 CANCELED 종결까지 봐야 "멈췄다" 라고 말할 수 있다.
            if self.stop_state == 'unconfirmed':
                self.get_logger().error(
                    f'🔴 BLOCKED · 정지 미확인 — 취소가 종결로 확인되지 않았다. '
                    f'로봇이 **아직 달리고 있을 수 있다.** 즉시 E-stop 으로 물리 정지를 '
                    f'확인할 것. 사유: {self.blocked_reason}',
                    throttle_duration_sec=2.0)
            elif self.stop_state == 'pending':
                self.get_logger().warn(
                    '🔶 BLOCKED · 정지 종결 대기 — 아직 "멈췄다" 로 읽지 말 것',
                    throttle_duration_sec=2.0)
            elif not self._blocked_logged:
                self.get_logger().error(
                    f'🔴 BLOCKED (정지 확인됨) — 자동 주행을 멈췄다. '
                    f'사유: {self.blocked_reason}. '
                    f'관제에서 대피 경로를 판단하고 `reset` 으로 재가동할 것')
                self._blocked_logged = True

        elif self.state == State.RESCUE:
            # 🔴 BLOCKED 과 같은 구조 — 새 goal 을 안 낸다. 그것이 정지의 구현이다.
            #   사이렌은 유지한다(화재는 여전히 있다).
            if not self._rescue_logged:
                where = (f'({self.victim[0]:.2f}, {self.victim[1]:.2f})'
                         if self.victim else '좌표 미수신')
                self.get_logger().error(
                    f'🔴 RESCUE — 움직일 수 없는 사람을 발견했다. 자동 주행을 멈췄다. '
                    f'위치 {where}. 관제가 판단하고 `reset` 으로 재가동할 것')
                self._rescue_logged = True

        elif self.state == State.NO_VICTIM:
            if not self._no_victim_logged:
                self.get_logger().error(
                    '🔴 NO_VICTIM — 집결지에 사람이 없다. 자동 주행을 멈췄다. '
                    '관제가 판단하고 `reset` 으로 재가동할 것')
                self._no_victim_logged = True

        elif self.state == State.FAULT:
            if self.fault_retries < self.MAX_RETRIES and self.resume_state is not None:
                elapsed = (self.get_clock().now() - self.fault_since).nanoseconds / 1e9
                if elapsed >= self.RETRY_WAIT:
                    self.fault_retries += 1
                    self.state = self.resume_state
                    self.resume_state = None
                    self.get_logger().warn(
                        f'재시도 {self.fault_retries}/{self.MAX_RETRIES} → {self.state.name} 복귀')
                    if self.state in (State.GUIDE, State.SEARCH_BACK):
                        # ★ 저속 보장은 상태와 함께 자동 복귀하지 않는다 (07-20 재검토
                        #   자기반증). FAULT 원인이 속도였다면 controller 는 아직
                        #   평시값이고 Manager 의 복구 예산도 소진된 상태다 —
                        #   GUIDE 로 되돌아가기 전에 저속을 다시 확인받는다.
                        #   ★ 07-23 §13: SEARCH_BACK 복귀도 재무장한다. 안 하면
                        #   _settle_gave_up 이 그대로 남아 위 live 가드가 다음 tick
                        #   즉시 재-FAULT → 재시도 예산이 헛돌며 영구 정지로 굳는다.
                        #   request_guide→_new_request 가 _settle_gave_up=False 로
                        #   복구 예산을 새로 줘 소진 술어를 해제한다.
                        self.speed.request_guide(float(self.wp['guide_speed']))

    # ===========================================================
    # SEARCH_BACK 진입 — 안전장치 2개가 여기서 작동
    # ===========================================================
    def _enter_guide_lowspeed(self):
        """구판 GATHER→GUIDE 경로를 **그대로** 뽑아낸 것. 동작을 바꾸지 않는다.

        ★ F2 (Codex §12.3): GUIDE 진입은 저속 '적용 확인' 후 — 전환은
        `_on_guide_speed_ok` 성공 콜백이 한다. 실패(3회/예외/미준비 timeout)면
        평시 0.26 으로 유도하는 대신 FAULT. 확인까지 로봇은 GATHER 로 정지 = 안전.
        """
        self._guide_pending = True
        self.get_logger().info(
            '집결대기 종료 — GUIDE 저속 적용 요청 (확인 후 유도 시작)')
        self.speed.request_guide(float(self.wp['guide_speed']))

    def on_person_status(self, msg: String):
        self.person_status = msg.data.strip().lower()
        self.person_status_t = self.get_clock().now()

    def on_victim(self, msg: PoseStamped):
        self.victim = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().error(
            f'🔴 쓰러진 사람 좌표 수신 ({self.victim[0]:.2f}, {self.victim[1]:.2f})')

    def person_gate_on(self):
        return bool(self.wp.get('person_gate', PERSON_GATE_DEFAULT))

    def fresh_person_status(self):
        """미션이 **자기 눈으로** 본 신선도. 어댑터가 죽으면 아무 말도 못 하므로,
        마지막 값을 그대로 믿으면 사람이 있는 줄 안다."""
        if self.person_status_t is None:
            return 'stale'
        age = (self.get_clock().now() - self.person_status_t).nanoseconds / 1e9
        timeout = float(self.wp.get('person_status_timeout_sec',
                                    PERSON_STATUS_TIMEOUT_DEFAULT))
        return 'stale' if age > timeout else self.person_status

    def person_verdict(self, waited):
        """집결 대기가 끝난 뒤 어디로 갈지. 'guide'|'rescue'|'no_victim'|'wait'.

        🔴 `unknown`·`stale` 은 **판정 보류**다 — 그동안 로봇은 GATHER 에 서 있다.
        영원히 서 있을 수는 없으므로 `person_decide_timeout_sec` 뒤에는 **신고**로
        간다. 모르는 채로 떠나는 것보다 모른다고 신고하는 쪽이 되돌릴 수 있다.
        """
        # 🔴 `fallen` 은 **항상 지금 값이 이긴다** — 훑는 동안 서 있던 사람이
        #   그 뒤에 쓰러졌을 수 있고, 그 경우 옛 판정을 쓰면 두고 나간다.
        live = self.fresh_person_status()
        if live == 'fallen':
            return 'rescue'
        # 그 외에는 sweep 이 모은 판정을 쓴다(§87.8). 훑기를 안 했으면 지금 값.
        s = self.scan_verdict or live
        if s == 'ok':
            return 'guide'
        if s == 'fallen':
            return 'rescue'
        if s == 'none':
            return 'no_victim'
        timeout = float(self.wp.get('person_decide_timeout_sec',
                                    PERSON_DECIDE_TIMEOUT_DEFAULT))
        return 'rescue' if waited >= timeout else 'wait'

    def camera_refind_status(self):
        """카메라 재발견이 켜져 있을 때의 사람 상태. 꺼져 있으면 None.

        예약 61 — 사용자 요청으로 등록했고 **기본 OFF** 로 좁혔다.
        ⚠ 어댑터가 없으면 `fresh_person_status()` 가 'stale' 이라 켜 두어도 아무
          영향이 없다. 그래도 키를 따로 두는 이유는 **의도를 남기기 위해서**다 —
          "카메라도 본다" 는 결정은 설정에 적혀 있어야 한다.
        """
        # ⚠ `.get('search_back', {})` — 최소 wp 로 만든 회귀 환경에는 이 절이
        #   없다. 선택 기능 하나가 그런 환경을 KeyError 로 죽이면 안 된다.
        if not bool(self.wp.get('search_back', {}).get('camera_refind', False)):
            return None
        s = self.fresh_person_status()
        return s if s in ('ok', 'fallen') else None

    def scan_steps(self):
        return max(1, int(self.wp.get('scan_steps', SCAN_STEPS_DEFAULT)))

    def scan_dwell_sec(self):
        return float(self.wp.get('scan_dwell_sec', SCAN_DWELL_SEC_DEFAULT))

    def enter_scan_area(self):
        self.scan_idx = 0
        self.scan_dwell_since = None
        self.scan_seen = set()         # 새 sweep 세대 — 앞 세대 관측을 물려받지 않는다
        self.scan_verdict = None
        self.scan_goal_since = None
        self.state = State.SCAN_AREA
        self.get_logger().info(
            f'집결지 도착 → SCAN_AREA: 제자리로 한 바퀴 훑는다 '
            f'({self.scan_steps()} 스텝 × {360.0 / self.scan_steps():.0f}°)')

    def scan_goal(self):
        """현재 스텝의 goal — 집결지와 **같은 (x,y)**, yaw 만 돌린다.

        🔴 위치를 그대로 두는 것이 핵심이다. 조금이라도 옮기면 화재 배제거리·
        keepout 전제가 흔들리고, 무엇보다 '제자리 훑기' 가 아니게 된다.
        """
        base = self.gather_wp or self.wp['gather']
        step = 2.0 * math.pi / self.scan_steps()
        yaw = float(base['yaw']) + step * (self.scan_idx + 1)
        # normalize 는 목표 yaw 표현만 정리한다 — 회전량 자체는 Nav2 가 정한다.
        return {'x': float(base['x']), 'y': float(base['y']),
                'yaw': normalize_angle(yaw)}

    def _confirmed_stop(self):
        """🔴 08-22 P0 — 정지를 전제하는 상태의 **공통 진입 절차**.

        구판은 인자 없는 `cancel_current_goal()` 을 부르고 곧바로 "섰다"고 선언했다.
        일반 취소는 `GoalManager` 의 `_stop_pending` 을 **무장하지 않는다**
        (`goal_manager.py:175` — `guide_stop`·`safety_stop` 일 때만 선다).
        그래서 노드의 미러만 `active=False` 가 되고 **Nav2 의 옛 goal 은 계속
        달릴 수 있었다.** `/mission_state` 는 HOLD 인데 로봇은 간다.

        🔴 `BLOCKED` 이 §84.2 에서 바로 이 이유로 `safety_stop` 으로 고쳐졌는데,
        신규 상태 셋이 **고쳐지기 전 패턴을 다시 썼다**(독립 검토 §87.2 P0).

        `safety_stop` 을 걸면 두 가지가 따라온다:
          · `send_goal` 이 `_stop_pending` 동안 **신규 goal 을 전면 봉쇄**한다
          · CANCELED 종결이 관찰돼야 `on_safety_stop_confirmed` 가 온다
        """
        had_goal = self.goal_active
        # 🔴 **순서가 중요하다.** `stop_state` 를 취소 **앞에서** 세운다.
        #   취소가 동기로 CANCELED 를 관찰하면 `on_safety_stop_confirmed` 가 그
        #   자리에서 'confirmed' 를 세우는데, 뒤에서 'pending' 을 쓰면 **그것을
        #   덮어써** 정지가 영원히 확인되지 않는다(08-22 보완 중 실제로 밟았다).
        #   `BLOCKED`(§84.2)도 같은 순서다.
        # 'none' = 애초에 멈출 goal 이 없었다(예: GATHER→NO_VICTIM). 거짓 FAULT 를
        #   만들지 않으려면 이 경우를 'pending' 과 갈라야 한다.
        self.stop_state = 'pending' if had_goal else 'none'
        self._cancel_intent = 'safety_stop'
        self.cancel_current_goal()

    def stop_confirmed(self):
        """'섰다'고 말해도 되는가. 🔴 'pending'·'unconfirmed' 는 아니다."""
        return self.stop_state in ('none', 'confirmed')

    def enter_rescue(self):
        self._confirmed_stop()
        self._rescue_logged = False
        self.state = State.RESCUE

    def enter_no_victim(self):
        self._confirmed_stop()
        self._no_victim_logged = False
        self.state = State.NO_VICTIM

    def enter_hold(self):
        """🆕 08-22 — 놓침 확정 직후 제자리 정지. 비싼 역행 앞의 싼 한 걸음.

        goal 을 취소하는 것이 곧 '정지'다 — tick 의 HOLD 가지가 새 goal 을 내지
        않으므로 로봇은 그 자리에 선다.
        ⚠ `search_attempts` 를 여기서 올리지 않는다. HOLD 는 역행이 아니고,
          역행 예산(`max_attempts`)은 실제로 되돌아간 횟수여야 한다.
        """
        self._confirmed_stop()
        # 🔴 08-22 P0 — 정지가 **확인된 뒤에만** 시간을 센다. 확인 전의 4초는
        #   "서서 기다린 4초" 가 아니다 — 그동안 로봇은 아직 달리고 있을 수 있다.
        #   🔵 취소가 그 자리에서 CANCELED 로 종결되면(흔한 경우) 기다릴 이유가
        #     없으므로 즉시 연다. 늦어지는 경우만 tick 이 대신 연다.
        self.hold_since = (self.get_clock().now()
                           if self.stop_confirmed() else None)
        self.get_logger().info(
            '추종자 놓침 → 제자리 정지하고 재수집 (역행 전 단계)')
        self.state = State.HOLD

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
            # ★ ②′ 안전망 (08-01 예약 16): 구판은 여기서 그냥 return 했다.
            #   그러면 시도 횟수가 0 에 고정돼 위 give_up 보고 분기가 영영 안
            #   열린다 = 놓침을 판정하고도 조용히 갇힌다.
            #   ⚠ 이 자리가 열리는 조건은 08-01 검토 §26 P1 보완으로 **좁아졌다**.
            #   구판 논증("None = GUIDE 내내 TF 실패")은 그때 반증됐다 — GUIDE 진입
            #   순간 이미 lost 였으면 TF 가 멀쩡해도 기록 기회가 0회였기 때문이다
            #   (재현: 전환 후 tf_calls 0 인 채 1.5초 만에 예산 소진).
            #   지금은 tick 의 GUIDE 진입 초크포인트가 세대를 재무장하므로
            #   진입 첫 tick 은 반드시 lost=False 로 시작한다 → 그 tick 에서
            #   record_last_seen 이 호출된다. /scan 두절 중에는 lost 자체가 False 라
            #   여기 못 오고, 두절이 풀리면 monitor.update 가 다시 재무장한다.
            #   ⇒ **여기 도달 = 진입 이후 신선한 모든 tick 에서 TF 조회가 실패했다**
            #   (last_seen 은 한 번 기록되면 관제 reset 전엔 None 으로 안 돌아간다).
            #   그건 순간 딸꾹질이 아니라 Nav2 주행 자체가 불가능한 상태이므로
            #   '실패한 시도'로 예산을 소모하고, 소진되면 위 보고 경로로 빠진다.
            #   ⚠ 아래 화재 클램프 포기는 여전히 예산을 안 깎는다 (07-06 결정 유지):
            #     그쪽은 로봇이 움직이면 last_seen 이 갱신돼 회복 가능한 실패지만,
            #     이쪽은 이번 유도 구간 안에서 회복될 수 없는 실패다.
            self.search_attempts += 1
            self.get_logger().warn(
                f'마지막 목격 지점 없음 — 역행 불가 '
                f'({self.search_attempts}/{sb["max_attempts"]}), 유도 계속',
                throttle_duration_sec=5.0)
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
        """역행 목표의 근사값으로 쓸 **로봇 자신의** map 좌표를 기록.

        ⚠ 이름이 'last_seen' 이지만 호출 조건은 '검출 중'이 아니다 (08-01 §26 P2):
        호출자(GUIDE 분기)의 조건은 `not lost and not scan_stale` 이므로
        검출이 끊긴 뒤에도 놓침 확정 직전까지, 세대 재무장 직후에는 실제 재검출
        0건에도 갱신된다. 규약 전문 = `self.last_seen` 선언부 주석.
        근사가 성립하는 전제는 '추종자는 ~1.2m 뒤'뿐이다."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint',
                                                rclpy.time.Time())
            self.last_seen = (t.transform.translation.x,
                              t.transform.translation.y)
        except Exception as e:
            # ★ ① 진단 (08-01 예약 16): 구판은 여기서 조용히 삼켰다(pass).
            #   그래서 last_seen 이 왜 None 인지 다음 재현 때 밝힐 재료가 없었다
            #   (07-30 '깜빡임 15초 지속' 원인 미규명 — 0730_현황.md §1.4).
            #   동작은 그대로다: 예외를 밖으로 내지 않고 다음 tick 에 재시도하며
            #   기존 기록도 지우지 않는다. 바뀌는 것은 로그 한 줄뿐.
            #   ⚠ TF 미준비면 매 tick(2Hz) 찍히므로 throttle 필수 (기존 코드 관례).
            self.get_logger().warn(
                f'마지막 목격 지점 기록 실패 (TF map→base_footprint): {e}',
                throttle_duration_sec=5.0)

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
        # ★ S1-1 알람 신뢰경계 (07-19 Codex §9.2 재현 반영): 검증 통과 전엔
        #   상태를 아무것도 바꾸지 않는다. NaN/Inf 는 그래프 투영에서 예외도 없이
        #   '멀쩡한 집결지'로 둔갑하고(재현: NaN→(4,0)), 1km 밖 오클릭도 정상
        #   화재처럼 처리되던 구멍 — 진입점에서 거부가 유일한 방어선.
        fx, fy = msg.pose.position.x, msg.pose.position.y
        if not (math.isfinite(fx) and math.isfinite(fy)):
            self.get_logger().warn(
                f'알람 거부: 좌표가 유한값 아님 ({fx!r}, {fy!r})')
            return
        if msg.header.frame_id != 'map':
            self.get_logger().warn(
                f'알람 거부: frame_id "{msg.header.frame_id}" (map 만 접수 — '
                f'좌표계 불명 화재로 출동 금지)')
            return
        graph = self.wp.get('corridor_graph')
        maxd = float(self.wp.get('alarm_max_projection_dist', 5.0))
        if graph is not None:
            d = distance_to_graph(fx, fy, graph)
            if d is not None and d > maxd:
                self.get_logger().warn(
                    f'알람 거부: 복도에서 {d:.1f}m 떨어진 좌표 (허용 {maxd}m) — '
                    f'지도 밖 오클릭 의심. 관제에서 좌표 확인 후 재발사 요망')
                return
        # ⓐ 집결지 계산 (07-06 → 07-19 그래프판 → S1-2 정책 확정):
        #   그래프 선언 시   = 그래프 계산, 실패하면 직선 '건너뛰고' yaml 고정값
        #   그래프 미선언 시 = 직선 수식, 실패하면 yaml 고정값
        #   (직선 fallback 을 그래프 실패에 쓰지 않는 이유 — Codex §8.2 재현:
        #    화재·탈출구가 raw 좌표는 달라도 같은 그래프 끝점에 투영되면 경로
        #    길이 0 → None 인데, 이때 직선식은 벽 안 좌표를 내놓는다. yaml
        #    고정값은 사람이 검증한 좌표라 항상 안전한 쪽.)
        esc = self.wp['escape']
        gd = float(self.wp.get('gather_dist', 8.0))
        if graph is not None:
            gather = compute_gather_point_graph(
                fx, fy, float(esc['x']), float(esc['y']), gd, graph)
            if gather is None:
                self.get_logger().warn(
                    '그래프 집결지 계산 불가(화재≈탈출구 또는 동일 투영점) — '
                    '직선 생략, yaml 검증 고정 집결지 사용')
        else:
            gather = compute_gather_point(
                fx, fy, float(esc['x']), float(esc['y']), gd)

        # ★ S1-3 집결지 안전거리 불변조건 (08-21 Codex §82.7 재현 반영 — 동결 예외 아홉 번째)
        #   [무엇이 틀렸었나] 설정 부등식 `min_fire_dist < gather_dist` 만 검사하고
        #   **계산 결과**는 검사하지 않았다. 경로가 gather_dist 보다 짧으면
        #   compute_gather_point_graph 가 탈출구로 클램프하는데, 그러면 집결지가
        #   화재 코앞이 된다. 재현(08-21): H yaml · fire(1.0,-10.65) → 알람게이트 통과
        #   (투영거리 0.00) → 집결지 (0.50,-10.65) = 화재에서 **0.50m**. 선언한
        #   min_fire_dist 는 1.5m 였다. fire(0.6,…) 이면 0.10m 까지 좁혀진다.
        #
        #   [왜 여기서 막나] SEARCH_BACK 목표만 나중에 클램프해도 **이미 수행한
        #   APPROACH 는 되돌릴 수 없다.** 그래서 상태를 바꾸기 전에 거부한다
        #   (S1-1 과 같은 규율 — 검증 통과 전엔 아무것도 안 바꾼다).
        #
        #   [정책] V1 에는 대체 탈출구가 없다. 안전한 집결지를 못 만들면
        #   **자동 출동하지 않고** 관제 판단으로 넘긴다. 조용히 가까이 가는 것보다
        #   안 가고 사람이 아는 편이 낫다.
        #
        #   ⚠ min_fire_dist 미선언이면 이 불변조건을 요구하지 않은 설정이므로 건너뛴다.
        #     🔵 08-21 §83.6 정정 — 이 경로는 **production 에 없다.**
        #     `validate_waypoints()` 가 `search_back.min_fire_dist` 를 필수로 요구해
        #     기동 자체를 막는다(실측: 키를 지우면 `['search_back.min_fire_dist']`).
        #     열리는 것은 `on_alarm` 만 직접 부르는 불완전 객체(단위 시험)뿐이다.
        #     구판 주석이 "기존 yaml 보호" 라고 적은 것은 근거가 틀렸다.
        sb = self.wp.get('search_back') or {}
        min_fd = sb.get('min_fire_dist')
        if min_fd is not None:
            eff = gather if gather is not None else self.wp.get('gather')
            if eff is not None:
                d_fire = math.hypot(float(eff['x']) - fx, float(eff['y']) - fy)
                if d_fire < float(min_fd):
                    # 🔴 08-21 §83.6 재현 반영 — 구판은 경고만 찍고 `return` 했다.
                    #   그래서 state=PATROL · cancel 0회 · siren False 로 **로봇이
                    #   기존 순찰 goal 을 그대로 계속 수행**했다. 정본은 "관제로
                    #   넘긴다" 라고 적었는데 그 이관이 **상태 전이로 존재하지
                    #   않았다.** 관제는 raw /alarm 마커만 보고 접수됐다고 오해한다.
                    #   → 활성 goal 을 취소하고 BLOCKED 를 발행해 **멈춘 뒤 사람에게
                    #     넘긴다.** 자동 순찰 재개는 하지 않는다(fail-closed).
                    msg_txt = (
                        f'알람 거부: 계산된 집결지 '
                        f'({float(eff["x"]):.2f},{float(eff["y"]):.2f}) 가 화재에서 '
                        f'{d_fire:.2f}m — 최소 안전거리 {float(min_fd):.2f}m 미만. '
                        f'탈출구 근처 화재라 경로가 짧아 집결지가 화재로 끌려왔다. '
                        f'자동 출동하지 않는다 — 관제에서 대피 경로를 판단할 것')
                    self.get_logger().warn(msg_txt)
                    # 🔴 §84.2 — 일반 취소가 아니라 **안전정지 의도**로 건다.
                    #   일반 취소는 세대만 stale 로 만들고 종결을 확인하지 않는다
                    #   (재현: cancel 응답을 빈 목록으로 주입해도 faults 0 ·
                    #    stop_pending False → 미션은 BLOCKED 인데 Nav2 는 계속 달림).
                    # 🔴 08-21 §85.3 — **취소 전에** 사유와 pending 을 세운다.
                    #   구판은 취소 뒤에 썼다. 그래서 `cancel_goal_async()` 가
                    #   **동기 예외**를 던지는 입력에서 `_stop_failed` 가 잠깐
                    #   `unconfirmed` 로 올려놨는데, 돌아온 뒤 이 줄들이 그걸
                    #   `pending` 과 원래 사유로 **덮었다**(재현 최종값
                    #   `BLOCKED/pending/unsafe_gather…`). E-stop 을 요구하는
                    #   신호가 사라진 것이다 — 순서가 곧 계약이다.
                    had_goal = self.goals.active
                    self._blocked_logged = False
                    self.blocked_reason = (
                        f'unsafe_gather fire=({fx:.2f},{fy:.2f}) '
                        f'gather=({float(eff["x"]):.2f},{float(eff["y"]):.2f}) '
                        f'dist={d_fire:.2f}<{float(min_fd):.2f}')
                    self.stop_state = 'pending' if had_goal else 'none'
                    self.state = State.BLOCKED
                    self._cancel_intent = 'safety_stop'
                    self.cancel_current_goal()
                    return

        self.fire = {
            'pos': (fx, fy),
            'kind': 'fire',             # 자리 예약
        }
        self.gather_wp = gather
        if self.gather_wp is not None:
            self.get_logger().info(
                f'집결지 계산: 화재({fx:.1f},{fy:.1f}) → '
                f'({self.gather_wp["x"]:.1f}, {self.gather_wp["y"]:.1f})')
        else:
            self.get_logger().warn('집결지 계산값 없음 — yaml 검증 고정 집결지 사용')
        self.get_logger().info('🔥 화재 알람 수신 → PATROL 중단, APPROACH 시작 (싸이렌 ON)')
        self.cancel_current_goal()
        self.set_siren(True)
        self.state = State.APPROACH

    def on_safety_stop_unconfirmed(self, reason):
        """§84.2 — 안전정지 취소의 종결 확인 실패. **FAULT 로 안 보낸다.**

        FAULT 는 자동 재시도가 있어 로봇이 스스로 재개한다. 안전한 집결지가
        없어서 멈춘 상황에서는 재시도할 goal 자체가 없다 — 사람이 판단해야 한다."""
        # 🔴 §85.3 — 한 번 unconfirmed 로 올라가면 같은 세대에서 내려오지 않는다.
        #   순서 수정으로 덮어쓰기는 사라졌지만, 늦은 콜백이 또 오는 경로가 있어
        #   **여기서도** 잠근다(방어 두 겹).
        if self.stop_state == 'unconfirmed':
            return
        self.stop_state = 'unconfirmed'
        # 🔴 08-22 — `blocked_reason` 은 **BLOCKED 전용**이다. 08-22 에 정지 확인을
        #   HOLD·RESCUE·NO_VICTIM 까지 넓히면서 이 콜백이 공용이 됐는데, 구판 그대로
        #   두면 HOLD 의 취소 실패가 BLOCKED 의 사유 문구를 덮어쓴다(관제가 "안전한
        #   집결지가 없다"로 읽는다). 그래서 그 상태에서만 쓴다.
        if self.state == State.BLOCKED:
            self.blocked_reason = (
                f'{self.blocked_reason or "안전정지"} · 정지 미확인({reason})')
        self.get_logger().error(
            f'🔴 {self.state.name} · 정지 종결 확인 실패 — {reason}. 로봇이 아직 '
            f'달리고 있을 수 있다. **E-stop 으로 물리 정지를 확인할 것.**')

    def on_safety_stop_confirmed(self):
        """§84.2 — CANCELED 종결 관찰. 이제서야 '멈췄다' 라고 말할 수 있다."""
        if self.stop_state in ('pending', None):
            self.stop_state = 'confirmed'
            self._blocked_logged = False
            self.get_logger().warn('🔵 안전정지 CANCELED 종결 확인 — 로봇이 섰다')

    def on_cmd(self, msg: String):
        """관제 명령 (07-07). 얇게 유지 — 명령 2개, 나머지는 무시+로그."""
        cmd = msg.data.strip().lower()
        if cmd == 'reset':
            # 임무 전체를 초기 상태로 — FAULT 소진·ESCAPED 후 재가동용
            self.get_logger().warn('★ 관제 reset — 임무 초기화 → PATROL')
            # ★ B: 운영자 개입은 최우선 — 진행 중이던 유도정지 취소 직렬화도 해제
            #   (재가동을 옛 취소의 CANCELED 종결에 볼모잡히지 않게).
            self._cancel_intent = 'hard'
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
            self._guide_pending = False   # F2: 응답 유실로 남은 게이트 잔재 청소
            self.blocked_reason = None    # §83.6 — BLOCKED 탈출은 reset 뿐이다
            self._blocked_logged = False
            self.stop_state = None      # §84.2
            # 🆕 08-22 — 사람 판정 잔재. 🔴 `person_status` 를 'stale' 로 되돌리는
            #   것이 핵심이다. 'none' 으로 두면 다음 임무 첫 판정이 곧바로
            #   NO_VICTIM 이 되고, 'ok' 로 두면 사람이 없어도 유도가 시작된다.
            self.person_status = 'stale'
            self.person_status_t = None
            self.victim = None
            self._rescue_logged = False
            self._no_victim_logged = False
            self.scan_idx = 0
            self.scan_dwell_since = None
            self.scan_seen = set()
            self.scan_verdict = None
            self.scan_goal_since = None
            # 진행 중 속도 요청 전부 stale 화 — 늦은 응답이 PATROL 을 못 덮게.
            # 늦게 '적용'된 낡은 속도는 SpeedManager 가 reconcile 로 재조정.
            self.speed.cancel_pending('reset')
            self.set_siren(False)
            self.speed.request_restore(float(self.wp['normal_speed']))
        elif cmd == 'abort':
            # 즉시 정지, 자동 재시도 없이 FAULT 유지 (복구는 reset 으로)
            self.get_logger().error('★ 관제 abort — 목표 취소, 정지 (재가동은 reset)')
            self._cancel_intent = 'hard'   # ★ B: 운영자 정지도 직렬화 강제 해제
            self.cancel_current_goal()
            self._guide_pending = False   # F2: 늦은 저속 확인이 FAULT 를 덮지 않게
            self.speed.cancel_pending('abort')   # 진행 중 속도 요청 stale 화
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
            self.speed.request_restore(float(self.wp['normal_speed']))
        else:
            self.get_logger().warn(f'FAULT — {self.RETRY_WAIT}초 후 재시도 예정')

    def set_siren(self, on: bool):
        self.siren_on = on              # 발행은 tick 이 매번 반복

    # ===========================================================
    # 속도 정책 콜백 (SpeedManager → 노드) — 비동기 수명주기는 Manager 소유
    # ===========================================================
    def _on_guide_speed_ok(self):
        """GUIDE 저속 '성공 확인' 콜백 — 이때만 GATHER→GUIDE 전환 (F2).

        stale(낡은 세대) 응답은 SpeedManager 가 이미 걸러 여기 안 오지만,
        상태 검사를 한 겹 더 둔다 — 콜백 대기 중 다른 경로(FAULT 등)로
        상태가 바뀌었으면 전환하지 않음 (늦은 응답이 상태를 덮으면 안 됨)."""
        if self._guide_pending and self.state == State.GATHER:
            self.get_logger().info('집결대기 종료 → GUIDE (저속 유도 시작)')
            self.state = State.GUIDE
        self._guide_pending = False

    def _on_guide_speed_fail(self, reason):
        """GUIDE 저속 최종 실패(3회/예외/미준비 timeout) 콜백 — goal 취소+FAULT.

        저속 보장 불가 → 평시 속도로 사람을 유도하는 대신 정지(FAULT).
        enter_fault 의 재시도 경로가 GATHER 를 resume → 처음부터 재요청."""
        if self._guide_pending and self.state == State.GATHER:
            # ① 진입 게이트 실패 — 아직 GATHER 정지 상태. 유도를 시작하지 않는다.
            self._guide_pending = False
            self.get_logger().error('GUIDE 저속 적용 실패 — 유도 진입 중단, FAULT')
        elif self.state in (State.GUIDE, State.SEARCH_BACK):
            # ② ★ 유지 실패 (07-20 Codex 재검토 P1) — 이미 사람을 앞에서 유도하며
            #   주행 중인데 controller 속도가 평시값(0.26)으로 덮인 상태다.
            #   과속 유도가 즉시 위험하므로 정지시킨다. 이전엔 아래 '무시' 로 빠져
            #   경고 한 줄만 남기고 0.26 인 채 유도가 계속됐다 (재검토 §10.3).
            #   ★ 07-23 §13: SEARCH_BACK 도 포함한다 — 유도 임무의 소풍일 뿐
            #   (두 복귀 경로 모두 GUIDE 로 돌아감) 저속 의무는 살아 있다. 이걸
            #   '늦은 통보'로 버리면 GUIDE 복귀 후 고장 없이 영구 정지했다(§13 P1).
            #   즉시성은 여기서, 통보 유실 대비는 tick 의 live 가드가 backstop.
            self.get_logger().error(
                f'★ GUIDE 저속 보장 상실({reason}) — 유도 중단, 정지 (FAULT)')
        else:
            # ③ 그 외 = 정말로 유도 안 하는 상태(PATROL/APPROACH/ESCAPED/FAULT 등).
            #   여기선 desired origin 이 이미 guide 가 아니라 애초에 이 콜백이 잘
            #   안 오지만, 오더라도 유도 위험이 없어 상태를 덮지 않는다.
            self.get_logger().warn(
                f'늦은 guide 속도 실패 통보({reason}) 무시 — 상태 변경됨'
                f'(현재 {self.state.name}), FAULT 안 함')
            return
        # ★ B: 저속 상실에 의한 정지 — 취소가 CANCELED 로 종결될 때까지 신규 goal
        #   봉쇄. 진입 실패(①)는 주행 goal(핸들)이 없어 Manager 가 자동으로
        #   직렬화를 무장하지 않는다(핸들 있을 때만) — 유지 실패(②)에서만 발동.
        self._cancel_intent = 'guide_stop'
        self.cancel_current_goal()
        self.enter_fault()


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
