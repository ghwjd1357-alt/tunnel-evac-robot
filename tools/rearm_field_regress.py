#!/usr/bin/env python3
"""§7-c-E 역회귀 1·2 + 부정 5 — ARMED 주행과 watchdog 이 살아 있는지 실기 확인.

왜 스크립트인가: "발행을 끊고 0.5초 안에 정지" 를 눈으로 재면 앞뒤 100ms 를 못 가른다.
정지 판정선은 저장소 정본(`tools/watchdog_report.py`)과 **같은 기준**을 쓴다 —
5 mm/s · 200ms 창(창 안 변위 1.0mm). 다른 기준을 새로 만들면 R0 기록과 비교가 안 된다.

🔴 이 스크립트는 `TODO(D+0) #11`(R0 watchdog 계약 판정)을 닫지 않는다.
여기서 보는 것은 **re-arm 래치가 watchdog 을 가리지 않았는가** 하나다.
#11 의 1차 증거는 60fps 영상이고 그 기준 재정의는 사용자 결정 사항이다.

판정 정본 = docs/JETSON_SETUP.md §7-c-E · 계약 = docs/REAL_ROBOT_VALUES.md §1-f.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

STATE = {0: "DISARMED", 1: "READY", 2: "ARMED", 3: "PENDING", 4: "ARMING"}

# tools/watchdog_report.py 정본과 동일 — 바꾸면 R0 기록과 비교가 깨진다
MOTION_RATE_MM_S = 5.0
MOTION_WINDOW_MS = 200


class Field(Node):
    def __init__(self):
        super().__init__("rearm_field_regress")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cli = self.create_client(SetBool, "/drive/enable")
        self.diag, self.enab, self.odom = [], [], []
        self.t0 = time.monotonic()
        self.create_subscription(Vector3, "/drive/diag", self.on_diag, 10)
        self.create_subscription(Bool, "/drive/enabled", self.on_en, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)

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

    def idle(self, dur, hz=50.0):
        """발행하지 않고 구독만 돌린다 — watchdog 관측 구간."""
        end = time.monotonic() + dur
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=1.0 / hz)

    def call(self, data, timeout=10.0):
        fut = self.cli.call_async(SetBool.Request(data=data))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result(), self.now()

    def state(self):
        return int(self.diag[-1][3]) if self.diag else -1

    def arm(self):
        """정상 4단계 무장. 성공하면 True."""
        self.spam(0.0, 2.0)
        if self.state() != 1:
            print(f"    ⚠ READY 가 아니다 (z={self.state()}) — E-stop·발행자 확인")
            return False
        res, _ = self.call(True)
        if res is None or not res.success:
            print(f"    ⚠ enable 거절/무응답: {res}")
            return False
        self.spam(0.0, 1.2)          # 장벽 0.5초 + 진단 1주기 여유
        if self.state() != 2:
            print(f"    ⚠ ARMED 가 아니다 (z={self.state()})")
            return False
        return True


def last_motion_time(samples, since=None):
    """정본 기준(5mm/s · 200ms 창)으로 '마지막으로 움직인 시각'을 낸다.

    각 표본 i 에서 이후 200ms 안의 최대 변위를 보고 1.0mm 를 넘으면 그 시각을 이동으로 센다.
    증분 하나로 판정하면 노이즈에 흔들리므로 창으로 본다 — 검토 §52 가 그 결함을 잡았다.
    """
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
    n.idle(1.0)
    if not n.odom:
        print("FAIL: /odom 이 안 온다 — 정지 판정을 할 수 없다. agent·펌웨어를 먼저 본다")
        return

    print("=" * 62)
    print("[역회귀 1] 정상 4단계로 무장하고 0.05 로 3초 주행 — 🔴 바퀴가 돌아야 정상")
    input("           준비되면 Enter (E-stop 에 손, 바퀴 공중) > ")
    if not n.arm():
        return
    print(f"    ARMED 확인 (z={n.state()}) — 3초 주행")
    t_start = n.now()
    t_last_pub = n.spam(0.05, 3.0)
    moved = last_motion_time(n.odom, since=t_start)
    print(f"    주행 중 이동 관측: {'있음 ✅' if moved else '없음 🔴'}")

    print("\n[역회귀 2] 🔴 발행을 끊는다 — watchdog 이 살아 있으면 0.5초 안에 선다")
    n.idle(3.0)
    stop = last_motion_time(n.odom, since=t_last_pub)
    if stop is None:
        print("    마지막 발행 이후 이동 없음 — 이미 정지")
        dt_ms = 0.0
    else:
        dt_ms = (stop - t_last_pub) * 1000.0
    print(f"    마지막 발행 → 마지막 이동 = {dt_ms:.1f} ms")
    print(f"    상태 z={n.state()} ({STATE.get(n.state(), '?')})  "
          f"enabled={n.enab[-1][1] if n.enab else '?'}")
    print("    ⚠ 이 숫자는 R0 #11 의 계약 판정이 아니다 — re-arm 이 watchdog 을")
    print("      가리지 않았는지만 본다. #11 은 60fps 영상이 1차 증거이고 여전히 열려 있다.")
    print("      (08-07 R0 재산출 기록 = 519.9 / 532.0 / 516.2 ms)")

    print("\n[부정 5] ARMED 에서 서비스 재호출 — 멱등이 아니라 거절이어야 한다")
    if n.state() != 2:
        print(f"    ⚠ 지금 z={n.state()} 라 ARMED 가 아니다. 다시 무장한다")
        print("      (watchdog 정지가 무장까지 풀었는지 여기서 드러난다)")
        if not n.arm():
            return
    res, _ = n.call(True)
    n.spam(0.0, 1.2)
    y = n.diag[-1][2] if n.diag else -1
    print(f"    success={res.success if res else None}  y={y:.0f}  (기대: success=False · y=3)")

    print("\n[정리] 명시적 해제")
    n.call(False)
    n.spam(0.0, 1.0)
    print(f"    z={n.state()} ({STATE.get(n.state(), '?')})")

    print("\n" + "=" * 62)
    print("=== /drive/diag (t, x=호출수, y=거절사유, z=상태) ===")
    for t, x, yy, zz in n.diag:
        print(f"  t={t:6.2f}  x={x:5.0f}  y={yy:3.0f}  z={zz:3.0f} {STATE.get(int(zz), '?')}")
    print("=== /drive/enabled 변화만 ===")
    prev = None
    for t, d in n.enab:
        if d != prev:
            print(f"  t={t:6.2f}  {d}")
            prev = d
    rclpy.shutdown()


main()
