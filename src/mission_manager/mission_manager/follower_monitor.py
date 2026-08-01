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
                 edge_margin=0.2,        # 문턱 경계 슬라이버 배제 폭(m)
                 scan_timeout=1.0):      # ★ watchdog: 이 시간 넘게 /scan 없으면 판정 보류
        self.clock = clock              # 노드의 clock (sim time 따라감)
        self.cone_half = math.radians(cone_half_deg)
        self.max_range = max_range
        self.min_points = min_points
        self.lost_sec = lost_sec
        self.seen_sec = seen_sec
        self.max_cluster_width = max_cluster_width
        self.range_jump = range_jump
        self.edge_margin = edge_margin
        self.scan_timeout = scan_timeout

        self._last_seen_t = {'rear': None, 'any': None}
        self._first_seen_t = {'rear': None, 'any': None}
        self._has_scan = False
        self._last_scan_t = None        # 마지막 /scan 수신 시각 (watchdog 재료)
        self._last_stamp_ns = None      # 마지막 scan 의 header.stamp (S1-6 재료)

    # -----------------------------------------------------------
    # /scan 콜백에서 호출 — 클러스터를 찾고 구역별 검출 여부를 갱신.
    # -----------------------------------------------------------
    def update(self, scan):
        # ★ S1-6 (07-19 Codex §10.6-5): watchdog 이 '콜백 도착 시각'만 보면
        #   드라이버가 멈춘 채 같은 프레임을 재전송해도 신선하다고 오인.
        #   header.stamp 가 전진하지 않는 스캔은 통째로 무시 → 수신시각이 안
        #   갱신되므로 scan_timeout 뒤 watchdog 이 정상적으로 stale 판정.
        #   (stamp 의 '절대 나이' 검사는 실차 clock 동기 확인 후 과제 — 마스터플랜 R3)
        hdr = getattr(scan, 'header', None)
        if hdr is not None:
            ns = hdr.stamp.sec * 1_000_000_000 + hdr.stamp.nanosec
            if self._last_stamp_ns is not None and ns <= self._last_stamp_ns:
                return
            self._last_stamp_ns = ns
        now = self.clock.now()
        # ★ 복구 경계 (07-19 Codex 검토 §3.1): 단절돼 있던 시간을 debounce
        #   연속시간에 포함하면 안 된다. 리셋 없이 이어 쓰면 —
        #   · 단절 전 0.5초 본 것 + 복구 첫 사람 프레임 → 즉시 visible (가짜 재발견)
        #   · 단절 5초가 '미검출 시간'으로 계산 → 복구 첫 빈 프레임에 즉시 lost
        #     (빈 프레임 한 장으로 비싼 SEARCH_BACK 발동)
        #   → 복구 순간: seen 타이머는 백지에서, lost 기산점은 '지금'으로 재무장.
        #     (단, 한 번도 본 적 없는 zone 의 _last_seen_t=None 은 유지 —
        #      "본 적 없으면 판단 보류" 의미를 재무장이 덮어쓰면 안 됨)
        #   ★ 08-02 검토 §27 P1: 이 가드가 `_last_scan_t is not None` 이었다 —
        #     즉 **스캔과 스캔 사이의 단절**만 보호하고 "프로세스 시작 ~ 첫 스캔"은
        #     보호하지 못했다("다시 뜨는" 것만 재무장하고 "처음 뜨는" 것은 아니었다).
        #     예전엔 그 구간에 _last_seen_t 가 None 이라 lost 가 False 여서 드러나지
        #     않았는데, 08-01 예약 16 이 GUIDE 진입 시 기산점을 세우면서 사각이 열렸다:
        #     라이다가 늦게 살아나면 그동안 흐른 벽시계 시간이 통째로 '미검출 시간'이
        #     되어 **첫 유효 빈 프레임 한 장에 즉시 lost** → 그 구간엔 stale 이라
        #     좌표 기록도 못 했으므로 역행 0회로 예산만 태우고 단독 탈출(실측 재현).
        #     라이다 드라이버·DDS discovery 가 미션 노드보다 늦게 뜨는 것은 실차의
        #     **정상 기동 순서**다 — 시뮬에선 거의 안 나지만 실물에선 흔하다.
        #   → 최초 스캔도 '단절 복구'로 친다. 계약을 한 문장으로 굳히면:
        #     **센서 데이터가 존재하지 않았던 시간은 debounce 에 포함하지 않는다.**
        #     이 자리에서 막으면 기산점을 세우는 호출자가 앞으로 늘어도 자동으로 덮인다
        #     (호출자 쪽에서 막으면 새 호출자마다 같은 구멍이 다시 열린다).
        #   ⚠ 동작 확대는 아래 안쪽 가드가 막는다: `_last_seen_t is not None` 인 zone만
        #     건드리는데, 첫 스캔 이전에 그게 non-None 인 경우는 **reset() 이 불린
        #     경우뿐**이다(검출은 스캔 없이 못 일어난다). 그 외에는 전부 no-op 이다.
        if self._last_scan_t is None or \
                (now - self._last_scan_t).nanoseconds / 1e9 >= self.scan_timeout:
            for zone in self._first_seen_t:
                self._first_seen_t[zone] = None
                if self._last_seen_t[zone] is not None:
                    self._last_seen_t[zone] = now
        self._has_scan = True
        self._last_scan_t = now
        person_like = [c for c in self._find_clusters(scan)
                       if self._is_person_like(c, scan)]
        detected_any = len(person_like) > 0
        detected_rear = any(self._in_rear_cone(c, scan) for c in person_like)
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
    def scan_stale(self):
        """★ watchdog (07-19): /scan 이 scan_timeout 넘게 안 왔나?
        시뮬에선 안 끊기지만 실물 USB 라이다는 끊긴다(0623 ttyUSB 씨름 참조).
        데이터가 죽었는데 마지막 판정을 계속 믿으면 '라이다 사망 = 추종 양호'."""
        if self._last_scan_t is None:
            return True     # 아직 한 장도 못 받음 = 신선한 데이터 없음
        age = (self.clock.now() - self._last_scan_t).nanoseconds / 1e9
        return age >= self.scan_timeout

    def lost(self, zone='rear'):
        """놓침 확정? — lost_sec 연속 미검출. (GUIDE 는 rear 사용)
        ★ scan 끊김 중엔 False (판단 보류): 놓침 선언은 비싼 역행(SEARCH_BACK)을
        일으키므로 데이터 없이 내리면 안 됨 — 대피 유도는 계속하는 게 안전한 쪽."""
        if self.scan_stale():
            return False
        if not self._has_scan or self._last_seen_t[zone] is None:
            return False    # 한 번도 본 적 없으면 판단 보류 (시작 직후 오판 방지)
        gap = (self.clock.now() - self._last_seen_t[zone]).nanoseconds / 1e9
        return gap >= self.lost_sec

    def visible(self, zone='rear'):
        """검출 확정? — seen_sec 연속 검출. (SEARCH_BACK 재발견은 zone='any')
        ★ scan 끊김 중엔 False: 마지막 프레임의 '보임'을 동결 유지하면
        라이다가 죽어도 영원히 visible=True 로 남는 구멍 (07-19 watchdog)."""
        if self.scan_stale():
            return False
        if self._first_seen_t[zone] is None:
            return False
        held = (self.clock.now() - self._first_seen_t[zone]).nanoseconds / 1e9
        return held >= self.seen_sec

    def reset(self, zone='rear'):
        """**관측 세대를 새로 연다** = lost 기산점을 '지금'으로, seen 연속시간은 백지로.
        한 문장 의미는 어느 호출자든 같다 — "놓침 판정을 처음부터 다시 시작하라".

        ★ 호출 역할 (08-02 검토 §27 P2 — 구 설명은 "두 곳에서 호출"이라고 적혀 있었고
          세 번째 호출자가 생기면서 **거짓이 됐다**). 생산 호출 자리마다 같은 이름의
          `[reset-role] <이름>` 주석이 달려 있고, `test_search_back_entry.py` 의
          `test_sb23_…` 이 이 목록과 그 주석을 **양방향으로** 대조한다 —
          자리를 추가하고 여기를 안 고치면 그 검사가 FAIL 한다. 개수를 세는 대신
          역할을 적는 이유는, 숫자는 다음 회차에 또 갈라지기 때문이다:

        · `[reset-role] guide-entry` — GUIDE 진입 (08-01 예약 16).
          놓침 타이머는 상태 경계를 모르고 /scan 은 상태와 무관하게 흐르므로,
          남의 상태에서 만료된 타이머를 물고 유도를 시작하면 첫 tick 부터 lost 다.
        · `[reset-role] refind-return` — 재발견 복귀.
          안 하면 타이머가 '놓침 당시' 그대로라 복귀 즉시 재-놓침.
        · `[reset-role] refind-timeout-return` — 역행 재탐색 실패 복귀 (07-07).
          안 하면 다음 tick 에 lost 가 즉시 참 → 같은 지점으로 2차 역행
          (예산이 '같은 곳 두 번'으로 소진돼 max_attempts 의 의도가 무너진다).

        ⚠ 이 함수는 **관측이 없어도** 기산점을 세운다 — 호출 자체가 "지금부터 세라"는
          선언이기 때문이다. 그래서 센서가 아직 한 번도 안 살아난 상태에서 부르면
          첫 유효 스캔까지의 시간이 미검출 시간으로 샐 수 있었다(§27 P1).
          그 누수는 `update()` 의 단절 복구가 최초 스캔까지 덮도록 고쳐 막았다 —
          **이 함수의 안전은 그 보호에 의존한다.** 둘을 따로 손대지 말 것."""
        self._last_seen_t[zone] = self.clock.now()
        self._first_seen_t[zone] = None
