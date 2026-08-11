#!/usr/bin/env python3
"""§5-G6 — E-stop 개폐 10회로 `/estop/state` 신뢰를 다시 채운다.

왜 다시 하나: **펌웨어를 다시 구우면 `/estop/state` 신뢰를 회수한다**
(`docs/ELECTRICAL_BASELINE.md §7`). 08-07 의 10/10 은 그때 굽혀 있던 바이너리에 대한
근거였고, 보드가 바뀌면 근거도 다시 만든다.

판정: 10회 전부 `false→true→false` 를 따라오면 통과. **한 번이라도 안 바뀌면
건접점(§7-3) 문제**이며 그 자리에서 멈춘다 — 10회 중 9회는 통과가 아니다.

⚠ 여기 찍히는 ms 는 **스위치 반응 속도가 아니다.** `/estop/state` 는 펌웨어의
`publishDiagnostics()` 에서 **1초에 한 번** 나가므로 최대 1초의 샘플링 지연이 섞여 있다.
사람이 누르는 시점도 자유다. 그래서 이 값은 진단용이고, 판정하는 것은 **전이의 유무**다.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

CYCLES = 10
WAIT_TIMEOUT_S = 60.0


class EstopWatch(Node):
    def __init__(self):
        super().__init__("estop_toggle_check")
        self.samples = []
        self.state = None
        self.create_subscription(Bool, "/estop/state", self.on_state, 10)

    def on_state(self, m):
        self.samples.append((time.monotonic(), m.data))
        self.state = m.data

    def wait_for(self, want, timeout=WAIT_TIMEOUT_S):
        """상태가 want 가 될 때까지 기다린다. (성공여부, 걸린초)"""
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.state == want:
                return True, time.monotonic() - start
        return False, time.monotonic() - start


def main():
    rclpy.init()
    n = EstopWatch()

    print("§5-G6 — E-stop 개폐 10회.  /estop/state 를 기다리는 중...")
    ok, _ = n.wait_for(False, timeout=15.0)
    if n.state is None:
        print("FAIL: /estop/state 가 안 온다 — agent·펌웨어를 먼저 본다")
        return 2
    if not ok:
        print(f"⚠ 지금 /estop/state = {n.state} 다. E-stop 이 눌려 있으면 떼고 다시 돌린다")
        return 2
    print("평상시 false 확인 (G3). 이제 10회 반복한다.\n")

    rows = []
    for i in range(1, CYCLES + 1):
        print(f"▶ {i:2d}/{CYCLES}회차 — 🔴 E-stop 을 **누르세요**", flush=True)
        ok_press, dt_press = n.wait_for(True)
        if not ok_press:
            print(f"   🔴 FAIL — {WAIT_TIMEOUT_S:.0f}초 안에 true 로 안 바뀌었다")
            print("   §7-3 건접점 문제다. 여기서 멈춘다.")
            rows.append((i, None, None))
            break
        print(f"   true 관측 ({dt_press:.2f}s)   — 이제 **떼세요**", flush=True)
        ok_rel, dt_rel = n.wait_for(False)
        if not ok_rel:
            print(f"   🔴 FAIL — {WAIT_TIMEOUT_S:.0f}초 안에 false 로 안 돌아왔다")
            print("   복귀 실패다(§7-3). 여기서 멈춘다.")
            rows.append((i, dt_press, None))
            break
        print(f"   false 복귀 ({dt_rel:.2f}s)  ✅\n")
        rows.append((i, dt_press, dt_rel))

    good = [r for r in rows if r[1] is not None and r[2] is not None]
    print("=" * 58)
    print(f"§5-G6 결과 — {len(good)}/{CYCLES}")
    print("  회차   누름→true    뗌→false")
    for i, a, b in rows:
        sa = f"{a:8.2f}s" if a is not None else "   FAIL "
        sb = f"{b:8.2f}s" if b is not None else "   FAIL "
        print(f"  {i:3d}   {sa}    {sb}")
    print("  ⚠ 이 초는 스위치 반응이 아니다 — /estop/state 는 1Hz 발행이고 누르는")
    print("    시점도 사람 자유다. 판정하는 것은 전이의 유무다.")
    if len(good) == CYCLES:
        print("\n✅ 10/10 — /estop/state 신뢰를 다시 채웠다.")
        print("   다음: bash tools/d0_check.sh 로 검사 8 (G7) 을 확인한다.")
        rclpy.shutdown()
        return 0
    print(f"\n🔴 {len(good)}/{CYCLES} — 통과가 아니다. 9/10 도 통과가 아니다(§5-G6).")
    print("   건접점(§7-3) 을 보고, 신뢰 회수 상태를 유지한 채 R1 을 금지한다.")
    rclpy.shutdown()
    return 1


sys.exit(main())
