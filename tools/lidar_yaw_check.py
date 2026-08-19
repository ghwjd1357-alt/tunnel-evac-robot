#!/usr/bin/env python3
"""라이다가 로봇 정면과 얼마나 틀어져 붙었는지 잰다 (yaw 정렬 오차).

사용:
    python3 tools/lidar_yaw_check.py                 # 20 스캔 평균
    python3 tools/lidar_yaw_check.py --scans 50      # 더 오래 본다
    python3 tools/lidar_yaw_check.py --fov 30        # 정면 ±30° 만 본다
    python3 tools/lidar_yaw_check.py --topic /scan_raw

무엇을 재는가
-------------
`robot_real.urdf` 의 `lidar_joint` 는 `rpy="0 0 0"` 이다 — 즉 **라이다 0° 가 로봇
정면(+x)과 정확히 같은 방향이라고 선언**하고 있다. 실제로 틀어져 붙어 있으면
costmap 의 장애물·SLAM 의 지도가 통째로 그 각도만큼 돌아간다.

    3 m 앞 장애물 기준 위치 오차 = 3 m x sin(θ)
        θ=1° →  52 mm      θ=2° → 105 mm
        θ=5° → 262 mm      θ=5° @10m → 872 mm

🔴 이 도구의 전제 — 이게 안 지켜지면 숫자가 통째로 거짓말이다
------------------------------------------------------------
**로봇이 평평한 벽에 물리적으로 수직**이어야 한다. 그 수직을 **라이다로 맞추면
순환논법**이다(재려는 것을 기준으로 삼는 꼴). 반드시 라이다와 무관한 기계적
기준으로 맞춘다:

    로봇 앞면의 좌·우 대칭인 두 점에서 벽까지 줄자로 재고, 두 값이 같아질 때까지
    로봇을 돌린다. 두 점 간격이 넓을수록 정밀하다.
      예) 좌우 500 mm 떨어진 두 점의 차가 5 mm  →  잔여 오차 atan(5/500) = 0.57°

    ⚠ 그 잔여 오차가 이 도구 결과의 바닥값이다. 0.5° 를 못 맞췄으면 0.5° 는 못 잰다.

원리
----
평평한 벽을 정면으로 보면, 스캔점들이 **직선** 하나를 이룬다. 로봇이 벽에 정확히
수직이면 그 직선은 라이다 좌표계의 y축과 나란해야 한다(= 각도 90°). 실제로 잰
직선 각도가 90° 에서 벗어난 만큼이 **라이다 yaw 오차**다.

부호 유도 (🔴 헷갈리는 자리라 근거를 남긴다 — 합성 데이터로 검산했다)
    라이다가 로봇 기준 반시계로 θ_L 돌아 붙었다면, 로봇 방위 0° 에 있는 것은
    라이다 눈에는 −θ_L 로 보인다. 우리가 직선맞춤으로 얻는 것은 **라이다가 본
    벽 법선의 방위** = −θ_L 이므로, 그 부호를 뒤집어야 라이다 자신의 회전량이다.

        θ_L = −(직선각도 − 90°)

    θ_L > 0  =  라이다가 로봇 기준 **반시계(왼쪽)** 로 돌아 붙었다
    θ_L < 0  =  **시계(오른쪽)** 로 돌아 붙었다
    → URDF 보정값은 **같은 부호** 그대로: `rpy="0 0 <θ_L 라디안>"`
      (base_link→lidar_link 변환이 곧 라이다의 물리 회전이기 때문)

🔴 이 도구가 하지 않는 것
------------------------
- URDF 를 고치지 않는다. 숫자만 낸다. (08-19 금지범위 = URDF 수정 금지)
- 벽이 정말 평평한지 판단하지 않는다 — 잔차(RMS)를 같이 찍으니 사람이 본다.
- 높이(z)·전후좌우(x,y) 오프셋은 안 본다. yaw 하나만이다.
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan


def fit_line_angle(pts):
    """점들에 직선을 맞추고 (직선 각도[rad], 원점~직선 수직거리[m], 잔차RMS[m]).

    PCA 로 한다 — 최소제곱(y=ax+b)은 벽이 y축과 나란할 때(바로 우리 경우다)
    기울기가 무한대로 발산해서 못 쓴다. PCA 는 그 특이점이 없다.
    """
    n = len(pts)
    cx = sum(p[0] for p in pts) / n
    cy = sum(p[1] for p in pts) / n

    sxx = syy = sxy = 0.0
    for x, y in pts:
        dx, dy = x - cx, y - cy
        sxx += dx * dx
        syy += dy * dy
        sxy += dx * dy
    sxx /= n
    syy /= n
    sxy /= n

    # 2x2 공분산의 최대 고유벡터 = 직선 방향
    tr = sxx + syy
    det = sxx * syy - sxy * sxy
    disc = max(tr * tr / 4.0 - det, 0.0)
    lam1 = tr / 2.0 + math.sqrt(disc)      # 큰 고유값 = 직선 방향
    lam2 = tr / 2.0 - math.sqrt(disc)      # 작은 고유값 = 두께(잔차)

    if abs(sxy) > 1e-12:
        vx, vy = lam1 - syy, sxy
    else:
        vx, vy = (1.0, 0.0) if sxx >= syy else (0.0, 1.0)
    norm = math.hypot(vx, vy)
    vx, vy = vx / norm, vy / norm

    ang = math.atan2(vy, vx)               # 직선 방향 각도
    # 원점에서 직선까지 수직거리 = 중심점을 법선에 투영
    nx, ny = -vy, vx
    dist = abs(cx * nx + cy * ny)
    rms = math.sqrt(max(lam2, 0.0))
    return ang, dist, rms


def wrap_deg(d):
    """직선은 180° 주기다 (같은 벽을 반대로 읽어도 같은 벽) → (-90, 90] 로 접는다."""
    while d <= -90.0:
        d += 180.0
    while d > 90.0:
        d -= 180.0
    return d


class YawCheck(Node):
    def __init__(self, topic, fov_deg, rmax):
        super().__init__("lidar_yaw_check")
        self.fov = math.radians(fov_deg)
        self.rmax = rmax
        self.results = []
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(LaserScan, topic, self._cb, qos)
        self.frame = None
        self.rejected = 0

    def _cb(self, m):
        self.frame = m.header.frame_id
        pts = []
        for i, r in enumerate(m.ranges):
            if not math.isfinite(r) or r <= m.range_min or r >= m.range_max:
                continue
            if r > self.rmax:
                continue
            a = m.angle_min + i * m.angle_increment
            if abs(a) > self.fov:          # 정면 ±fov 만
                continue
            pts.append((r * math.cos(a), r * math.sin(a)))

        if len(pts) < 20:
            self.rejected += 1
            return
        self.results.append(fit_line_angle(pts) + (len(pts),))


def main(argv=None):
    ap = argparse.ArgumentParser(description="라이다 yaw 정렬 오차 측정")
    ap.add_argument("--topic", default="/scan")
    ap.add_argument("--scans", type=int, default=20, help="평균낼 스캔 수")
    ap.add_argument("--fov", type=float, default=40.0,
                    help="정면 ±이 각도 안의 점만 쓴다 [deg]")
    ap.add_argument("--rmax", type=float, default=4.0,
                    help="이 거리보다 먼 점은 버린다 [m] (옆방 벽 배제)")
    args = ap.parse_args(argv)

    rclpy.init()
    node = YawCheck(args.topic, args.fov, args.rmax)
    print("=" * 66)
    print("  라이다 yaw 정렬 측정 — %s 에서 %d 스캔" % (args.topic, args.scans))
    print("  🔴 전제: 로봇이 벽에 **줄자로** 수직 정렬돼 있어야 한다.")
    print("=" * 66)

    try:
        deadline = node.get_clock().now().nanoseconds + int(30e9)
        while rclpy.ok() and len(node.results) < args.scans:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.get_clock().now().nanoseconds > deadline:
                break

        if not node.results:
            print("🔴 쓸 만한 스캔이 0건이다.")
            print("   · 토픽이 도는가:  ros2 topic hz %s" % args.topic)
            print("   · 정면 ±%.0f° · %.1fm 안에 벽이 있는가 (버린 스캔 %d건)"
                  % (args.fov, args.rmax, node.rejected))
            return 2

        # 🔴 부호 뒤집기 — 직선맞춤이 주는 것은 "라이다가 본 벽 법선 방위"이고,
        #    라이다 자신의 회전량은 그 반대다. 유도 = 이 파일 머리말.
        angs = [-wrap_deg(math.degrees(r[0]) - 90.0) for r in node.results]
        dists = [r[1] for r in node.results]
        rmss = [r[2] for r in node.results]
        npts = [r[3] for r in node.results]

        n = len(angs)
        mean = sum(angs) / n
        var = sum((a - mean) ** 2 for a in angs) / n
        sd = math.sqrt(var)

        print()
        print("  frame_id      : %s" % node.frame)
        print("  사용 스캔      : %d 건 (스캔당 점 %d~%d)" % (n, min(npts), max(npts)))
        print("  벽까지 수직거리 : %.3f m" % (sum(dists) / n))
        print("  직선 잔차 RMS  : %.4f m   %s" % (
            sum(rmss) / n,
            "✅ 평평한 벽" if sum(rmss) / n < 0.01 else
            "🔴 벽이 안 평평하거나 다른 물체가 섞였다 — --fov 를 줄여라"))
        print()
        print("  " + "-" * 62)
        print("  🔵 라이다 yaw 오차 =  %+.2f°   (표준편차 %.2f°)" % (mean, sd))
        print("  " + "-" * 62)
        print()

        if mean > 0:
            print("  → 라이다가 로봇 정면 기준 **반시계(왼쪽)** 로 %.2f° 돌아 붙었다." % abs(mean))
        else:
            print("  → 라이다가 로봇 정면 기준 **시계(오른쪽)** 로 %.2f° 돌아 붙었다." % abs(mean))

        print()
        print("  3m 앞 장애물 위치 오차  = %.0f mm" % (3000.0 * math.sin(math.radians(abs(mean)))))
        print("  10m 앞 장애물 위치 오차 = %.0f mm" % (10000.0 * math.sin(math.radians(abs(mean)))))
        print()

        if abs(mean) < 0.5:
            print("  ✅ 0.5° 미만 — 줄자 정렬 정밀도와 같은 급이다. 정렬됐다고 본다.")
        elif abs(mean) < 2.0:
            print("  🔶 0.5~2° — 실재할 수 있다. 로봇을 180° 돌려 재측정해 확인한다(아래).")
        else:
            print("  🔴 2° 이상 — 기계적으로 다시 붙이거나 URDF lidar_joint rpy 로 보정한다.")
            print("     URDF 보정값(같은 부호) : rpy=\"0 0 %+.5f\"  (%+.2f°)"
                  % (math.radians(mean), mean))
            print("     🔴 단, URDF 수정은 08-19 금지범위다 — 오늘은 숫자만 남긴다.")

        print()
        print("  ── 🔴 확인 필수: 180° 뒤집어 한 번 더 재라 ──")
        print("  로봇을 제자리에서 180° 돌려 같은 벽을 보고 다시 잰다.")
        print("    · 부호가 뒤집히고 크기가 비슷 → 로봇/라이다 문제 (진짜 정렬 오차)")
        print("    · 같은 부호로 비슷하게 나옴  → 벽이나 줄자 정렬 문제 (측정 오류)")
        print("  예약 39 가 08-18 에 바닥과 로봇을 가른 방법이 정확히 이것이다.")
        print()
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
