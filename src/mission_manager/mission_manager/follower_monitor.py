#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
follower_monitor.py — 후방 추종감지 (라이다 기반, ③단계) — ★ 교체 가능한 모듈
============================================================

[역할]
  "로봇 뒤에 따라오는 무언가(사람)가 있는가?"를 /scan 만으로 판정.
  mission_node 는 이 클래스의 visible()/lost() 두 답만 사용 —
  나중에 지도 배경제거·카메라 융합으로 업그레이드해도 mission_node 는 안 바뀜.

[1차 구현 원리 — 후방 부채꼴 + 거리 문턱]
  라이다는 360°(−π~π) → 후방 = 각도가 ±π 근처인 빔들.
  후방 부채꼴(기본 ±60°) 안에서 거리 < 문턱(기본 2.5m) 인 점이
  min_points(기본 3)개 이상 → "뒤에 뭔가 있다"(raw).

  왜 2.5m 문턱이면 벽이 안 걸리나: 폭 6m 터널 중앙에서 옆벽까지 ~3m.
  후방 부채꼴 가장자리(뒤축에서 60°)로 옆벽을 봐도 거리 = 3/sin(60°) ≈ 3.5m > 2.5m.
  추종자는 1~2m 뒤 → 확실히 문턱 안. (한계: 로봇이 벽 근처·회전 중이면 오탐
  가능 → 아래 디바운스가 흡수. 2차 개선 = 지도 배경제거.)

[디바운스 — 깜빡임으로 인한 무한 왕복 방지 (§12.0 설계 판단)]
  raw 판정을 그대로 쓰면 노이즈 한 프레임에 GUIDE⇄SEARCH_BACK 진동.
  → lost()  : lost_sec(기본 3초) 연속으로 안 보여야 True  (놓침 확정)
  → visible(): seen_sec(기본 1초) 연속으로 보여야 True    (재발견 확정)
  비대칭인 이유: 놓침 선언(비싼 역행 유발)은 신중히, 재발견은 빠르게.
"""

import math


class FollowerMonitor:

    def __init__(self, clock,
                 cone_half_deg=60.0,   # 후방 부채꼴 반각(도)
                 max_range=2.5,        # 이 거리 안의 점만 '추종자 후보' (벽 배제 문턱)
                 min_points=3,         # 점이 이만큼 뭉쳐야 인정 (노이즈 1~2점 배제)
                 lost_sec=3.0,         # 놓침 확정에 필요한 연속 미검출 시간
                 seen_sec=1.0):        # 재발견 확정에 필요한 연속 검출 시간
        self.clock = clock            # 노드의 clock (sim time 따라감)
        self.cone_half = math.radians(cone_half_deg)
        self.max_range = max_range
        self.min_points = min_points
        self.lost_sec = lost_sec
        self.seen_sec = seen_sec

        # ★ 두 감시 구역을 병행 운영 (③단계 설계 구멍 수정):
        #   'rear' = 후방 부채꼴 — GUIDE 중 "따라오나?" 판정용.
        #   'any'  = 전방위      — SEARCH_BACK 중 "재발견?" 판정용.
        #     역행 중엔 놓친 사람이 로봇 '앞'에 있으므로 후방만 보면 영원히 못 찾는다!
        self._last_seen_t = {'rear': None, 'any': None}
        self._first_seen_t = {'rear': None, 'any': None}
        self._has_scan = False

    # -----------------------------------------------------------
    # /scan 콜백에서 호출 — 구역별로 가까운 점 개수를 센다.
    # -----------------------------------------------------------
    def update(self, scan):
        self._has_scan = True
        n_rear = 0
        n_any = 0
        angle = scan.angle_min
        for r in scan.ranges:
            if scan.range_min < r < self.max_range:
                n_any += 1
                # 후방 = 각도의 절댓값이 (π − 반각) 보다 큰 빔
                if abs(angle) > (math.pi - self.cone_half):
                    n_rear += 1
            angle += scan.angle_increment

        now = self.clock.now()
        for zone, count in (('rear', n_rear), ('any', n_any)):
            if count >= self.min_points:
                if self._first_seen_t[zone] is None:
                    self._first_seen_t[zone] = now    # 연속 검출 시작
                self._last_seen_t[zone] = now
            else:
                self._first_seen_t[zone] = None       # 연속 끊김

    # -----------------------------------------------------------
    # mission_node 가 묻는 답 (디바운스 적용)
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
