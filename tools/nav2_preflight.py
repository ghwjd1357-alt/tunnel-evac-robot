#!/usr/bin/env python3
"""시행 직전 5초 점검 — **무엇이 실제로 돌고 있는지**를 한 화면에 세운다.

사용:
    python3 tools/nav2_preflight.py                 # 무장까지 요구 (시행 직전)
    python3 tools/nav2_preflight.py --no-arm-check  # 스택만 확인 (무장 전)
    python3 tools/nav2_preflight.py --expect-db     # 🔴 튜닝 사본이 실렸는지까지 요구

왜 이 도구가 있나
-----------------
08-19 에 `nav2_params:=nav2_params_real_db.yaml` 한 토막을 빠뜨린 채 시행을 돌렸다.
로그는 **정상으로 보였고**, 결과도 그럴듯했고, 파라미터를 직접 조회하고 나서야
현행판이 돌고 있었다는 걸 알았다 — **시행 하나가 통째로 폐기**됐다.

그 실패의 모양이 이렇다: *"지금 무엇이 돌고 있는지"* 가 기동 로그에 안 남는다.
이 도구는 시행 **전에** 그걸 강제로 인쇄한다. 판정이 아니라 **관측**이다.

무엇을 보는가
-------------
1. 🔴 **파라미터 판(版)** — 4개 값으로 원본/시험판을 가른다. 값이 섞여 있으면 FAIL.
2. **Nav2 lifecycle** — 5종이 active 인가.
3. **위치추정** — `map → base_footprint` 가 서 있고 원점 근처인가.
4. **무장 3종** — `/drive/enabled` · `/drive/diag` z · `/estop/state`.
5. **살아있는 토픽** — `/scan`·`/odom`·`/imu/data`·`/firmware/pulse` 수신 여부.
6. **펌웨어 지문** — 보드가 무엇인가.

종료코드: 0 = GO · 1 = NO-GO · 2 = 점검 자체가 실패(판정 불능)
"""

import argparse
import math
import re
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist  # noqa: F401  (토픽 존재 확인용 타입 로딩)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Vector3
from rcl_interfaces.srv import GetParameters
from tf2_ros import Buffer, TransformListener

BEST_EFFORT = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)

#: (파라미터, 원본값, 시험판값) — `nav2_params_real.yaml` vs `_db.yaml` 의 차이 전량.
#: 🔴 이 표가 곧 "판(版) 식별자"다. 하나라도 섞이면 어느 판도 아니다.
PARAM_SPEC = [
    ("FollowPath.rotate_to_heading_min_angle", 0.6, 0.25),
    ("FollowPath.min_approach_linear_velocity", 0.05, 0.10),
    ("FollowPath.max_angular_accel", 2.0, 10.0),
    ("progress_checker.movement_time_allowance", 20.0, 30.0),
]
LIFECYCLE = ["/controller_server", "/planner_server", "/bt_navigator",
             "/behavior_server", "/velocity_smoother"]
GREEN, RED, YEL, OFF = "\033[32m", "\033[31m", "\033[33m", "\033[0m"


class Preflight(Node):
    def __init__(self):
        super().__init__("nav2_preflight")
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        self.seen = {}
        self.armed = None
        self.estop = None
        self.diag = None
        self.fwinfo = None
        subs = [("/scan", LaserScan), ("/odom", Odometry), ("/imu/data", Imu),
                ("/firmware/pulse", String)]
        for name, typ in subs:
            self.create_subscription(
                typ, name, (lambda m, n=name: self.seen.__setitem__(n, self.seen.get(n, 0) + 1)),
                BEST_EFFORT)
        self.create_subscription(Bool, "/drive/enabled",
                                 lambda m: setattr(self, "armed", m.data), BEST_EFFORT)
        self.create_subscription(Bool, "/estop/state",
                                 lambda m: setattr(self, "estop", m.data), BEST_EFFORT)
        self.create_subscription(Vector3, "/drive/diag",
                                 lambda m: setattr(self, "diag", m), BEST_EFFORT)
        self.create_subscription(String, "/firmware/info",
                                 lambda m: setattr(self, "fwinfo", m.data), BEST_EFFORT)

    def spin_for(self, sec):
        end = time.time() + sec
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def get_params(self, node_name, names, timeout=4.0):
        cli = self.create_client(GetParameters, "%s/get_parameters" % node_name)
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        req = GetParameters.Request()
        req.names = list(names)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            return None
        out = {}
        for name, val in zip(names, res.values):
            out[name] = val.double_value if val.type == 3 else None
        return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="시행 직전 5초 점검")
    ap.add_argument("--no-arm-check", action="store_true", help="무장 요구 안 함(스택만)")
    ap.add_argument("--expect-db", action="store_true", help="🔴 튜닝 사본이 실려야 GO")
    ap.add_argument("--origin-tol", type=float, default=0.5,
                    help="위치추정이 원점에서 이만큼 안이어야 한다 [m]")
    args = ap.parse_args(argv)

    rclpy.init()
    node = Preflight()
    fails, warns = [], []
    try:
        node.spin_for(4.0)
        print("=" * 68)
        print("  시행 전 점검   %s" % time.strftime("%H:%M:%S"))
        print("=" * 68)

        # ── 1. 파라미터 판 ──────────────────────────────────────────
        print("\n[1] 🔴 Nav2 파라미터 판(版)")
        got = node.get_params("/controller_server", [n for n, _, _ in PARAM_SPEC])
        verdict = "판정 불능"
        if got is None:
            print("    🔴 /controller_server 파라미터를 못 읽었다 — Nav2 가 안 떴다")
            fails.append("파라미터 조회 실패")
        else:
            n_orig = n_db = 0
            for name, orig, db in PARAM_SPEC:
                v = got.get(name)
                tag = "?"
                if v is not None and math.isclose(v, orig, rel_tol=1e-6):
                    tag = "원본"
                    n_orig += 1
                elif v is not None and math.isclose(v, db, rel_tol=1e-6):
                    tag = "시험판"
                    n_db += 1
                print("    %-46s %-8s %s" % (name, "—" if v is None else "%.3f" % v, tag))
            if n_db == len(PARAM_SPEC):
                verdict = "시험판 (_db.yaml)"
                print("    → %s%s%s" % (GREEN, verdict, OFF))
            elif n_orig == len(PARAM_SPEC):
                verdict = "원본 (nav2_params_real.yaml)"
                print("    → %s%s%s" % (YEL, verdict, OFF))
            else:
                verdict = "🔴 섞였다 — 어느 판도 아니다"
                print("    → %s%s%s (원본 %d · 시험판 %d)" % (RED, verdict, OFF, n_orig, n_db))
                fails.append("파라미터 판이 섞였다")
            if args.expect_db and n_db != len(PARAM_SPEC):
                fails.append("--expect-db 인데 시험판이 아니다")

        # ── 2. lifecycle ───────────────────────────────────────────
        print("\n[2] Nav2 lifecycle")
        alive = {n for n, _ in node.get_node_names_and_namespaces()}
        for full in LIFECYCLE:
            short = full.lstrip("/")
            ok = short in alive
            print("    %-22s %s" % (full, "%s있음%s" % (GREEN, OFF) if ok
                                    else "%s없음%s" % (RED, OFF)))
            if not ok:
                fails.append("%s 없음" % full)

        # ── 3. 위치추정 ────────────────────────────────────────────
        print("\n[3] 위치추정 (map → base_footprint)")
        try:
            tf = node.buf.lookup_transform("map", "base_footprint", rclpy.time.Time())
            t = tf.transform.translation
            d = math.hypot(t.x, t.y)
            mark = GREEN if d <= args.origin_tol else YEL
            print("    [%+.3f, %+.3f]   원점거리 %s%.3f m%s" % (t.x, t.y, mark, d, OFF))
            if d > args.origin_tol:
                warns.append("원점에서 %.2fm — 지도 시작점이 맞는지 본다" % d)
        except Exception as err:
            print("    %s없다%s — %s" % (RED, OFF, str(err).split("\n")[0][:60]))
            fails.append("map→base_footprint TF 없음")

        # ── 4. 무장 ────────────────────────────────────────────────
        print("\n[4] 무장 상태")
        z = None if node.diag is None else int(node.diag.z)
        y = None if node.diag is None else int(node.diag.y)
        zname = {0: "DISARMED", 1: "READY", 2: "ARMED", 3: "PENDING", 4: "ARMING"}
        print("    /drive/enabled  %s" % node.armed)
        print("    /drive/diag  z=%s (%s)  y=%s" % (z, zname.get(z, "?"), y))
        print("    /estop/state    %s" % node.estop)
        if node.estop is True:
            fails.append("E-stop 눌림")
        if not args.no_arm_check:
            if z != 2 or node.armed is not True:
                fails.append("무장 안 됨 (z=%s enabled=%s)" % (z, node.armed))

        # ── 5. 토픽 ────────────────────────────────────────────────
        print("\n[5] 살아있는 토픽 (4초 창)")
        for name in ("/scan", "/odom", "/imu/data", "/firmware/pulse"):
            n = node.seen.get(name, 0)
            ok = n > 0
            print("    %-18s %s%d 건%s" % (name, GREEN if ok else RED, n, OFF))
            if not ok:
                fails.append("%s 무수신" % name)

        # ── 6. 펌웨어 ──────────────────────────────────────────────
        print("\n[6] 펌웨어 지문")
        if node.fwinfo:
            got2 = dict(re.findall(r"(\w+)=([^;]+)", node.fwinfo))
            print("    version %s" % got2.get("version", "?"))
            print("    build   %s" % got2.get("build", "?"))
        else:
            print("    %s/firmware/info 무수신%s" % (YEL, OFF))
            warns.append("/firmware/info 무수신")

        # ── 판정 ───────────────────────────────────────────────────
        print("\n" + "=" * 68)
        for w in warns:
            print("  %s⚠%s  %s" % (YEL, OFF, w))
        if fails:
            print("  %sNO-GO%s — %d 건" % (RED, OFF, len(fails)))
            for f in fails:
                print("      · %s" % f)
            print("=" * 68)
            return 1
        print("  %sGO%s   파라미터 = %s" % (GREEN, OFF, verdict))
        print("  🔴 물리 E-stop 담당자 확인 후 시행한다.")
        print("=" * 68)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
