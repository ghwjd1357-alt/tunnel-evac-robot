#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
follower_monitor.py — 후방 추종감지 (라이다 기반) — ★ 교체 가능한 모듈
============================================================

[역할]
  "로봇 뒤에 따라오는 무언가(사람)가 있는가?"를 /scan 만으로 판정.
  mission_node 는 이 클래스의 visible()/lost() 두 답만 사용 —
  내부 알고리즘을 업그레이드해도 mission_node 는 안 바뀜 (이 파일이 그 증명:
  1차 '점 개수' → 2차 '클러스터 크기 판별'로 갈아탔지만 인터페이스 동일).

[2차 구현 원리 — 클러스터(점뭉치) 크기 판별 (07-06 업그레이드)]
  1차(점 개수만 세기)의 구멍: 벽도 문턱(2.5m) 안에 들어오면 점 3개는 우습게
  넘긴다. 특히 SEARCH_BACK 재발견(zone='any', 전방위)은 로봇이 벽·코너에
  접근하면 벽을 사람으로 오인 → "재발견!" 하고 유도를 잘못 재개하는,
  시나리오상 가장 나쁜 방향의 오류였다 (rear 오탐은 '안전한 쪽' 오류지만
  any 오탐은 '위험한 쪽' 오류).
  → 2차: 점의 '개수'가 아니라 '뭉치의 실제 폭'을 본다.
    ① 문턱 안의 점들을 이웃 빔끼리 덩어리(클러스터)로 묶는다
       (빔 간격 ≤2, 거리 차 ≤ range_jump — 끊긴 점은 다른 물체).
    ② 덩어리 폭 = 평균거리 × 각도폭 (호 길이). 사람(원기둥 r0.15 / 다리)은
       ~0.3m, 벽은 수 m 의 긴 호 → max_cluster_width(0.8m) 로 벽 배제.
    ③ 경계 슬라이버 배제: 벽이 문턱에 '걸치는' 순간엔 잘린 조각이 사람
       크기로 보일 수 있다 → 덩어리 전체가 문턱 근처(max_range−edge_margin
       이상)에만 있으면 무시. 진짜 추종자는 1~2m 로 확실히 안쪽.
    ④ 라이다 배열 양끝(−π/+π)은 실제론 같은 '정후방' → 랩어라운드 병합
       (후방 감시가 주 임무인데 정후방 덩어리가 둘로 쪼개지면 안 됨).
  남은 한계: 사람 크기의 '물체'(배럴 등)는 여전히 구분 못 함 →
  3차 후보 = 지도 배경제거("/map 에 없는 점만"), 카메라(역할 B) 융합.

[디바운스 — 깜빡임으로 인한 무한 왕복 방지 (§12.0 설계 판단)]
  raw 판정을 그대로 쓰면 노이즈 한 프레임에 GUIDE⇄SEARCH_BACK 진동.
  → lost()  : lost_sec(기본 3초) 연속으로 안 보여야 True  (놓침 확정)
  → visible(): seen_sec(기본 1초) 연속으로 보여야 True    (재발견 확정)
  비대칭인 이유: 놓침 선언(비싼 역행 유발)은 신중히, 재발견은 빠르게.

[두 감시 구역]
  'rear' = 후방 부채꼴(±cone_half_deg) — GUIDE 중 "따라오나?" 판정용.
  'any'  = 전방위                      — SEARCH_BACK 중 "재발견?" 판정용.
    역행 중엔 놓친 사람이 로봇 '앞'에 있으므로 후방만 보면 영원히 못 찾는다!
"""

import math


class FollowerMonitor:

    def __init__(self, clock,
                 cone_half_deg=60.0,     # 후방 부채꼴 반각(도)
                 max_range=2.5,          # 이 거리 안의 점만 '추종자 후보' (벽 배제 문턱)
                 min_points=3,           # 덩어리가 이만큼 점을 가져야 인정 (노이즈 배제)
                 lost_sec=3.0,           # 놓침 확정에 필요한 연속 미검출 시간
                 seen_sec=1.0,           # 재발견 확정에 필요한 연속 검출 시간
                 max_cluster_width=0.8,  # 사람 크기 상한(m) — 이보다 넓은 덩어리 = 벽
                 range_jump=0.3,         # 이웃 빔 거리차가 이보다 크면 다른 물체로 분리
                 edge_margin=0.2):       # 문턱 경계 슬라이버 배제 폭(m)
        self.clock = clock              # 노드의 clock (sim time 따라감)
        self.cone_half = math.radians(cone_half_deg)
        self.max_range = max_range
        self.min_points = min_points
        self.lost_sec = lost_sec
        self.seen_sec = seen_sec
        self.max_cluster_width = max_cluster_width
        self.range_jump = range_jump
        self.edge_margin = edge_margin

        self._last_seen_t = {'rear': None, 'any': None}
        self._first_seen_t = {'rear': None, 'any': None}
        self._has_scan = False

    # -----------------------------------------------------------
    # /scan 콜백에서 호출 — 클러스터를 찾고 구역별 검출 여부를 갱신.
    # -----------------------------------------------------------
    def update(self, scan):
        self._has_scan = True
        person_like = [c for c in self._find_clusters(scan)
                       if self._is_person_like(c, scan)]
        detected_any = len(person_like) > 0
        detected_rear = any(self._in_rear_cone(c, scan) for c in person_like)

        now = self.clock.now()
        for zone, det in (('rear', detected_rear), ('any', detected_any)):
            if det:
                if self._first_seen_t[zone] is None:
                    self._first_seen_t[zone] = now    # 연속 검출 시작
                self._last_seen_t[zone] = now
            else:
                self._first_seen_t[zone] = None       # 연속 끊김

    # -----------------------------------------------------------
    # 내부: 문턱 안의 점들을 이웃끼리 덩어리로 묶기
    #   반환: [[(빔인덱스, 거리), ...], ...]  (랩어라운드 병합분은 음수 인덱스)
    # -----------------------------------------------------------
    def _find_clusters(self, scan):
        pts = []
        for i, r in enumerate(scan.ranges):
            # inf/nan(미반사 빔) 방어 — 실물 라이다에서 흔함
            if math.isfinite(r) and scan.range_min < r < self.max_range:
                pts.append((i, r))
        if not pts:
            return []

        clusters = [[pts[0]]]
        for p in pts[1:]:
            last_i, last_r = clusters[-1][-1]
            if p[0] - last_i <= 2 and abs(p[1] - last_r) <= self.range_jump:
                clusters[-1].append(p)      # 이웃 빔 + 거리 연속 → 같은 물체
            else:
                clusters.append([p])        # 끊김 → 새 물체

        # ★ 랩어라운드 병합: 배열 첫/끝은 실제론 이웃 방향(−π=+π=정후방).
        #   정후방 물체가 둘로 쪼개지면 각각 min_points 미달로 놓칠 수 있다.
        n = len(scan.ranges)
        if len(clusters) >= 2:
            first, last = clusters[0], clusters[-1]
            gap = first[0][0] + (n - 1 - last[-1][0])
            if gap <= 2 and abs(first[0][1] - last[-1][1]) <= self.range_jump:
                # 끝쪽 덩어리 인덱스를 음수로 당겨 붙임 (각도 계산 일관성 유지)
                merged = [(i - n, r) for (i, r) in last] + first
                clusters = [merged] + clusters[1:-1]
        return clusters

    # -----------------------------------------------------------
    # 내부: 이 덩어리가 '사람 크기'인가?
    # -----------------------------------------------------------
    def _is_person_like(self, cluster, scan):
        if len(cluster) < self.min_points:
            return False                    # 점 1~2개 = 노이즈
        mean_r = sum(r for _, r in cluster) / len(cluster)
        span = (cluster[-1][0] - cluster[0][0]) * scan.angle_increment
        width = mean_r * span + 0.05        # 호 길이 (+빔 자체 폭 보정)
        if width > self.max_cluster_width:
            return False                    # 넓은 호 = 벽
        # 경계 슬라이버 배제: 덩어리 전체가 문턱 코앞이면 문턱에 걸친 벽 조각
        if min(r for _, r in cluster) > self.max_range - self.edge_margin:
            return False
        return True

    # -----------------------------------------------------------
    # 내부: 덩어리 중심이 후방 부채꼴 안인가?
    # -----------------------------------------------------------
    def _in_rear_cone(self, cluster, scan):
        mid_idx = (cluster[0][0] + cluster[-1][0]) / 2.0
        ang = scan.angle_min + mid_idx * scan.angle_increment
        ang = math.atan2(math.sin(ang), math.cos(ang))   # [-π,π] 정규화
        return abs(ang) > (math.pi - self.cone_half)

    # -----------------------------------------------------------
    # mission_node 가 묻는 답 (디바운스 적용) — 인터페이스 불변
    # -----------------------------------------------------------
    def lost(self, zone='rear'):
        """놓침 확정? — lost_sec 연속 미검출. (GUIDE 는 rear 사용)"""
        if not self._has_scan or self._last_seen_t[zone] is None:
            return False    # 한 번도 본 적 없으면 판단 보류 (시작 직후 오판 방지)
        gap = (self.clock.now() - self._last_seen_t[zone]).nanoseconds / 1e9
        return gap >= self.lost_sec

    def visible(self, zone='rear'):
        """검출 확정? — seen_sec 연속 검출. (SEARCH_BACK 재발견은 zone='any')"""
        if self._first_seen_t[zone] is None:
            return False
        held = (self.clock.now() - self._first_seen_t[zone]).nanoseconds / 1e9
        return held >= self.seen_sec

    def reset(self, zone='rear'):
        """타이머 리셋 = "방금 봤다"로 간주. 재발견 → GUIDE 복귀 직후 호출:
        안 하면 후방 타이머가 '3초 전(놓침 당시)' 그대로라 복귀 즉시 재-놓침 판정."""
        self._last_seen_t[zone] = self.clock.now()
        self._first_seen_t[zone] = None
