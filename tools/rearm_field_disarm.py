#!/usr/bin/env python3
"""§7-c-E 부정 7·8 — 해제·NaN 이 그 자리에서 모터를 세우는가.

🔴 설계의 핵심: **해제 뒤에도 비영 발행을 계속한다.**
그러지 않으면 발행이 끊긴 셈이라 watchdog(500ms)이 대신 세워 버리고,
그러면 "해제가 세운 것"과 "watchdog 이 세운 것"을 구분할 수 없다.
비영을 계속 주면 watchdog 은 영원히 발동하지 않으므로, 바퀴가 서는 유일한 이유는
`driveDisarm()` 의 정지 + `driveOutputAllowed()` 의 0 덮어쓰기(검토 §54.1 두 겹)뿐이다.
DISARMED 에서 비영은 어차피 무시되므로 이 발행 자체는 안전하다.

정지 판정선은 `tools/watchdog_report.py` 정본과 동일 — 5 mm/s · 200ms 창.

판정 정본 = docs/JETSON_SETUP.md §7-c-E · 계약 = docs/REAL_ROBOT_VALUES.md §1-f.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

STATE = {0: "DISARMED", 1: "READY", 2: "ARMED", 3: "PENDING", 4: "ARMING"}
MOTION_RATE_MM_S = 5.0
MOTION_WINDOW_MS = 200


class Field(Node):
    def __init__(self):
        super().__init__("rearm_field_disarm")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cli = self.create_client(SetBool, "/drive/enable")
        self.diag, self.enab, self.odom = [], [], []
        self.t0 = time.monotonic()
        self.create_subscription(
            Vector3, "/drive/diag", self.on_diag, qos_profile_sensor_data)
        self.create_subscription(
            Bool, "/drive/enabled", self.on_en, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, "/odom", self.on_odom, qos_profile_sensor_data)

    def on_diag(self, m):
        self.diag.append((self.now(), m.x, m.y, m.z))

    def on_en(self, m):
        self.enab.append((self.now(), m.data))

    def on_odom(self, m):
        p = m.pose.pose.position
        self.odom.append((self.now(), p.x, p.y))

    def now(self):
        return time.monotonic() - self.t0

    def spam(self, vx, dur, hz=20.0):
        msg = Twist()
        msg.linear.x = float(vx)
        end = time.monotonic() + dur
        while time.monotonic() < end:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=1.0 / hz)
        return self.now()

    def call(self, data, timeout=10.0):
        fut = self.cli.call_async(SetBool.Request(data=data))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result(), self.now()

    def state(self):
        return int(self.diag[-1][3]) if self.diag else -1

    def arm(self):
        self.spam(0.0, 2.0)
        if self.state() != 1:
            print(f"    ⚠ READY 가 아니다 (z={self.state()}) — E-stop·발행자 확인")
            return False
        res, _ = self.call(True)
        if res is None or not res.success:
            print(f"    ⚠ enable 거절/무응답: {res}")
            return False
        self.spam(0.0, 1.2)
        if self.state() != 2:
            print(f"    ⚠ ARMED 가 아니다 (z={self.state()})")
            return False
        return True


def last_motion_time(samples, since=None):
    eps_m = MOTION_RATE_MM_S * MOTION_WINDOW_MS / 1000.0 / 1000.0
    win = MOTION_WINDOW_MS / 1000.0
    pts = [s for s in samples if since is None or s[0] >= since]
    last = None
    for i, (t, x, y) in enumerate(pts):
        far = 0.0
        for t2, x2, y2 in pts[i:]:
            if t2 - t > win:
                break
            far = max(far, math.hypot(x2 - x, y2 - y))
        if far > eps_m:
            last = t
    return last


def main():
    rclpy.init()
    n = Field()
    if not n.cli.wait_for_service(timeout_sec=10.0):
        print("FAIL: /drive/enable 서비스가 안 보인다")
        return
    n.spam(0.0, 1.0)
    if not n.odom:
        print("FAIL: /odom 이 안 온다 — 정지 판정 불가")
        return

    # ---------------- 부정 7 ----------------
    print("=" * 62)
    print("[부정 7] ARMED 주행 중 disable(false) — 응답이 오는 순간 이미 서 있어야 한다")
    print("         🔴 해제 뒤에도 0.05 를 계속 보낸다 (watchdog 을 공범에서 뺀다)")
    input("         준비되면 Enter (E-stop 에 손, 바퀴 공중) > ")
    if not n.arm():
        return
    print(f"    ARMED (z={n.state()}) — 2초 주행")
    n.spam(0.05, 2.0)
    res, t_resp = n.call(False)
    print(f"    disable 응답: success={res.success if res else None}  t_resp={t_resp:.3f}s")
    print("    해제 뒤에도 0.05 를 2초 더 발행한다 — 🔴 바퀴는 서 있어야 한다")
    n.spam(0.05, 2.0)
    last = last_motion_time(n.odom, since=t_resp)
    if last is None:
        print("    ✅ 응답 이후 이동 관측 **없음** — §54.1 두 겹이 배선돼 있다")
    else:
        print(f"    응답 → 마지막 이동 = {(last - t_resp) * 1000.0:.1f} ms")
        print("    ⚠ 수백 ms 이내면 기계 관성(타행)일 수 있다. 500ms 를 넘거나 계속 돌면")
        print("      🔴 §54.1 이 안 고쳐진 것이다 — 비영을 계속 주므로 watchdog 은 발동하지 않는다")
    print(f"    z={n.state()} ({STATE.get(n.state(), '?')})  기대: 0 DISARMED")
    n.spam(0.0, 1.5)

    # ---------------- 부정 8 ----------------
    print("\n" + "=" * 62)
    print("[부정 8] ARMED 주행 중 linear.x = NaN — 즉시 정지 · DISARMED · 재무장 필요")
    input("         준비되면 Enter > ")
    if not n.arm():
        return
    print(f"    ARMED (z={n.state()}) — 2초 주행")
    n.spam(0.05, 2.0)
    nan_msg = Twist()
    nan_msg.linear.x = float("nan")
    t_nan = n.now()
    for _ in range(3):
        n.pub.publish(nan_msg)
        rclpy.spin_once(n, timeout_sec=0.05)
    print(f"    NaN 3회 발행 (t={t_nan:.3f}s)")
    print("    이어서 0.05 를 2초 발행한다 — 🔴 재무장 없이는 돌면 안 된다")
    n.spam(0.05, 2.0)
    last8 = last_motion_time(n.odom, since=t_nan)
    if last8 is None:
        print("    ✅ NaN 이후 이동 관측 **없음**")
    else:
        print(f"    NaN → 마지막 이동 = {(last8 - t_nan) * 1000.0:.1f} ms")
    z8 = n.state()
    print(f"    z={z8} ({STATE.get(z8, '?')})  기대: 0 DISARMED (2 로 남으면 안 고쳐진 것)")

    print("\n[정리] zero + 명시적 해제")
    n.spam(0.0, 1.0)
    n.call(False)
    n.spam(0.0, 1.0)
    print(f"    z={n.state()} ({STATE.get(n.state(), '?')})")

    print("\n" + "=" * 62)
    print("=== /drive/diag (t, x=호출수, y=거절사유, z=상태) ===")
    for t, x, y, zz in n.diag:
        print(f"  t={t:6.2f}  x={x:5.0f}  y={y:3.0f}  z={zz:3.0f} {STATE.get(int(zz), '?')}")
    print("=== /drive/enabled 변화만 ===")
    prev = None
    for t, d in n.enab:
        if d != prev:
            print(f"  t={t:6.2f}  {d}")
            prev = d
    rclpy.shutdown()


main()
