#!/usr/bin/env python3
"""§7-c-E 부정 6 — re-arm 장벽(PENDING) 실기 배선 시험.

왜 스크립트인가: `ros2 topic pub` 은 노드 생성·DDS 탐색에 0.5~1.5초가 걸려
장벽(500ms) 안에 명령을 꽂을 수 없다. 늦게 도착한 명령으로 바퀴가 도는 것을
"장벽 미배선"으로 오판하는 거짓 FAIL 을 막으려면, 발행자를 미리 띄워 두고
응답 직후에 주입해야 한다. 주입 지연을 함께 인쇄해 그 회차가 유효했는지 남긴다.

판정 정본 = docs/JETSON_SETUP.md §7-c-E · 계약 = docs/REAL_ROBOT_VALUES.md §1-f.
이 스크립트는 전이를 판정하지 않는다(그건 tools/rearm_gate_host_test.sh 몫).
여기서 보는 것은 배선 하나다 — 장벽 중에 들어온 비영 명령이 무장을 깨뜨리는가.
"""
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

STATE = {0: "DISARMED", 1: "READY", 2: "ARMED", 3: "PENDING", 4: "ARMING"}


class Neg6(Node):
    def __init__(self):
        super().__init__("rearm_neg6_field")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cli = self.create_client(SetBool, "/drive/enable")
        self.diag, self.enab, self.t0 = [], [], time.monotonic()
        self.create_subscription(Vector3, "/drive/diag", self.on_diag, 10)
        self.create_subscription(Bool, "/drive/enabled", self.on_en, 10)

    def on_diag(self, m):
        self.diag.append((time.monotonic() - self.t0, m.x, m.y, m.z))

    def on_en(self, m):
        self.enab.append((time.monotonic() - self.t0, m.data))

    def spam(self, vx, dur, hz=20.0):
        """vx 를 dur 초 동안 hz 로 계속 발행한다(그 사이 구독도 돈다)."""
        msg = Twist()
        msg.linear.x = float(vx)
        end = time.monotonic() + dur
        while time.monotonic() < end:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=1.0 / hz)

    def call(self, data, timeout=10.0):
        fut = self.cli.call_async(SetBool.Request(data=data))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()


def main():
    rclpy.init()
    n = Neg6()
    if not n.cli.wait_for_service(timeout_sec=10.0):
        print("FAIL: /drive/enable 서비스가 안 보인다 — agent·펌웨어를 먼저 본다")
        return

    print("[1] zero 2초 → READY 를 만든다")
    n.spam(0.0, 2.0)
    z = n.diag[-1][3] if n.diag else -1
    print(f"    직전 z = {z:.0f} ({STATE.get(int(z), '?')})")
    if int(z) != 1:
        print("    ⚠ READY 가 아니다 — E-stop 해제·다른 발행자 없음을 확인하고 다시 돌린다")
        return

    print("[2] enable(true) 호출")
    res = n.call(True)
    t_resp = time.monotonic()
    if res is None:
        print("FAIL: 서비스 응답 없음(timeout)")
        return
    print(f"    success={res.success}   t_resp={t_resp - n.t0:.3f}s")

    print("[3] 응답 직후 0.05 주입 — 🔴 바퀴를 보세요 (3초)")
    first = Twist()
    first.linear.x = 0.05
    n.pub.publish(first)
    lat = (time.monotonic() - t_resp) * 1000.0
    n.spam(0.05, 3.0)

    print("[4] 정리 — zero 3초 + 명시적 해제 1회")
    n.spam(0.0, 3.0)
    n.call(False)
    n.spam(0.0, 1.0)

    ok_lat = lat < 500.0
    armed = [t for t, d in n.enab if d]
    print("\n=== 주입 지연 ===")
    print(
        f"  응답 → 첫 비영 발행 = {lat:.1f} ms  "
        + ("✅ 장벽 안 — 유효한 시험" if ok_lat else "🔴 500ms 초과 — 판정 불능, 다시 돌린다")
    )
    print("=== /drive/diag (t, x=호출수, y=거절사유, z=상태) ===")
    for t, x, y, zz in n.diag:
        mark = "  <-- 응답 근처" if abs(t - (t_resp - n.t0)) < 0.6 else ""
        print(
            f"  t={t:6.2f}  x={x:5.0f}  y={y:3.0f}  z={zz:3.0f} "
            f"{STATE.get(int(zz), '?'):9s}{mark}"
        )
    print("=== /drive/enabled ===")
    for t, d in n.enab:
        print(f"  t={t:6.2f}  {d}")
    print(
        f"\n=== 판정 보조 ===  enabled=true 관측 {len(armed)}건 "
        + ("(없음 = 기대대로)" if not armed else "🔴 (무장됐다 — 장벽 미배선 의심)")
    )
    rclpy.shutdown()


main()
