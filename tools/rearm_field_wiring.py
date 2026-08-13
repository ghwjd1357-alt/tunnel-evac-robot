#!/usr/bin/env python3
"""§7-c-E 축소판 — **배선을 보는 6 행**만 한 번에 밟는다 (§7-c-E2 · 08-13 신설).

왜 축소하나
-----------
`§7-c-E` 는 13 행이고 터미널 4 개 + 명령 10 여 개를 사람이 순서대로 쳐야 한다. 실제로
08-13 밤에 40 분이 들었고, 그중 절반은 무장 4 단계를 손으로 밟는 시간이었다. 그런데
13 행은 성질이 둘로 갈린다:

  ⓐ **상태기계·서비스만 보는 행** (부정 2·3, 전환 1·2·3, 부정 5, 해제)
     `success` / `y`(거절사유) / `z`(상태) 만 본다. 이 경로의 정본은 `rearm_gate.h` 이고
     `tools/rearm_gate_host_test.sh` 가 **동작 967 + 구조 7** 로 결정론적으로 닫았다.
     실기로 다시 밟아도 같은 것을 더 나쁜 해상도로 볼 뿐이다.
  ⓑ **배선을 보는 행** (부정 1·4·7·8, 역회귀 1·2)
     *모터가 실제로 서는가 / 도는가* 를 본다. 🔴 **이것만은 실기로만 볼 수 있다.**

→ 이 도구는 ⓑ 여섯 행만 밟는다.

🔴 축소 범위를 못 박는다 — 이 도구가 스스로 거부한다
----------------------------------------------------
이 축소가 **정당한 조건**은 하나다: *굽는 diff 가 상태기계·정지 배선·PWM 출력단·
그리고 **생략한 7 행이 지나는 `.ino` 통합 배선**을 안 건드릴 것.*

⚠ **08-13 밤 정정 (검토 §68.3)** — 앞 판은 `rearm_gate.h`·`drive_wiring.h` **두 헤더만**
잠갔다. 그런데 생략한 행들(서비스 거절·전환·해제)이 실제로 지나는 곳은 `.ino` 의
`driveEnableCallback` · `checkSafety` · `cmdVelCallback` · `loop` 장벽 순서다. 두 헤더를
그대로 둔 채 그 배선만 바꿔도 관문이 통과했다 — **fail-closed 라는 주장이 일반적으로
거짓이었다.** (그 시점 blob 에서 전이가 바뀌었다는 뜻은 아니다. 호스트 967/0 + 구조
7/0 이 통과했다. 도구가 스스로 강제한다는 **계약**이 거짓이었다는 뜻이다.)

→ 이제 두 헤더 + **생략 행을 소유한 `.ino` 함수 7 개**의 원문 sha256 을 함께 잠근다.
어느 하나라도 다르면 **거부하고 13 행 전량으로 돌려보낸다.**

⚠ **08-13 밤의 교훈** — 그날 §7-c-E 13 행을 "전항목 통과" 로 기록했는데, 그때
**왼쪽 모터 드라이버 배선이 빠져 있었다.** "선다" 를 보는 행들은 왼쪽에 대해 자동으로
참이었고, "돈다" 를 보는 역회귀 1 은 왼쪽에서 충족되지 않았다. 그래서 이 도구는
**네 바퀴를 각각 묻는다** — "바퀴가 돌았나" 가 아니라 "**네 바퀴 다** 돌았나" 다.

사용
----
    python3 -u tools/rearm_field_wiring.py        # 🔴 -u 를 붙인다(프롬프트 즉시 출력)

    종료 0 = 6 행 전량 통과 / 1 = 계약 위반 / 2 = 판정 불능(헤더 변경·환경)

🔴 **전량 바퀴 공중.** 통과한 뒤에 내린다. 안전요원이 E-stop 을 들고 있어야 한다.

정본 = docs/JETSON_SETUP.md §7-c-E2 · 계약 = docs/REAL_ROBOT_VALUES.md §1-f.
"""

import hashlib
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SKETCH = os.path.join(ROOT, "firmware", "teensy_integrated_base_v1_4")

#: 🔴 축소가 성립하는 전제. 이 둘이 바뀌면 이 도구는 판정을 거부한다.
#:   갱신하려면 `rearm_gate_host_test.sh` 를 다시 돌려 967/0 을 확인한 뒤 옮긴다.
GATE_SHA256 = {
    "rearm_gate.h":
        "ddf416b939c79cd094a6aeaac989da5050db25928410890fbc91a2ff8d10b340",
    "drive_wiring.h":
        "f34ba116fbd94a317362754dd1fc846a39ca76a387cd9d1e7a9d43783e08b860",
}

#: 🔴 생략한 7 행이 지나는 `.ino` 통합 배선 (검토 §68.3). 헤더만으로는 안 잠긴다.
#:   각 함수가 어느 행을 소유하는가:
#:     driveEnableCallback   부정 2·3·5 · 전환 2 · 해제  (서비스 수용/거절 사유)
#:     cmdVelCallback        전환 1 (zero-hold -> READY) · 비영 명령의 장벽 파기
#:     loop                  전환 2·3 (rearmGateArmBarrierStart · rearmGateTick 순서)
#:     checkSafety           E-stop 감시 -> 해제 경로
#:     updateMotorOutputs    출력 허용/정지 (배선 행과 상태 행이 만나는 자리)
#:     disarmDrive(WithReason) 해제의 정지 부작용
#:   갱신하려면 `rearm_gate_host_test.sh` 967/0 과 `§7-c-E` **13 행 전량**을 다시 밟은 뒤
#:   그 결과로 옮긴다. 🔴 값만 옮기면 이 관문이 자기가 승인한 것을 검사하게 된다.
INO_WIRING_SHA256 = {
    "driveEnableCallback":
        "1a523213090384c23a447048e8aec8056369899f16a34a3847b55a7a3b7f692a",
    "checkSafety":
        "2c5d9be3e60cc4242022482523b2619ea546e70258274b310ac1675715fd2301",
    "cmdVelCallback":
        "f5f349b806403533dc0bba01ede5533b7f044dd3791c3c44b9663adce691dac1",
    "loop":
        "2b6a42bfd6bb0631e202f1664ee7886cb395e7fd1a86af16a8428e62c231a761",
    "updateMotorOutputs":
        "2e6317a73516e768b6584a416b57b4dc2787b804337c1f0d458b3d0b18a7c067",
    "disarmDrive":
        "dd0b871a75708f727c1fc94159353292ae686d5b61981c5cd824dabfb857c74b",
    "disarmDriveWithReason":
        "07a51a11542c8f0cfc2ba65fd553c890f04dbf1fc2a5ee3564616f8a68c9b037",
}

STATE = {0: "DISARMED", 1: "READY", 2: "ARMED", 3: "PENDING", 4: "ARMING"}
RESULTS = []


def record(row, ok, note=""):
    RESULTS.append((row, ok, note))
    mark = "\033[32mPASS\033[0m" if ok else "\033[31mFAIL\033[0m"
    print("      %s  %s%s" % (mark, row, ("  — " + note) if note else ""))


def ask(prompt):
    """사람에게 묻는다. 🔴 기본값을 '안전한 쪽' 으로 두지 않는다 — 명시적으로 답하게 한다."""
    while True:
        got = input("      %s [y/n] > " % prompt).strip().lower()
        if got in ("y", "yes"):
            return True
        if got in ("n", "no"):
            return False
        print("      y 또는 n 으로 답한다 (모르면 그 자리에서 멈추고 다시 본다)")


def wait(prompt):
    input("      %s — 준비되면 Enter > " % prompt)


class Field(Node):
    def __init__(self):
        super().__init__("rearm_field_wiring")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cli = self.create_client(SetBool, "/drive/enable")
        self.diag, self.enab, self.odom = [], [], []
        self.t0 = time.monotonic()
        self.create_subscription(Vector3, "/drive/diag", self._diag, 10)
        self.create_subscription(Bool, "/drive/enabled", self._en, 10)
        self.create_subscription(Odometry, "/odom", self._odom, 10)

    def _diag(self, m):
        self.diag.append((self.now(), m.x, m.y, m.z))

    def _en(self, m):
        self.enab.append((self.now(), m.data))

    def _odom(self, m):
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

    def pump(self, dur):
        end = time.monotonic() + dur
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def state(self):
        return int(self.diag[-1][3]) if self.diag else -1

    def enabled(self):
        return self.enab[-1][1] if self.enab else None

    def call(self, value, timeout=5.0):
        req = SetBool.Request()
        req.data = bool(value)
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def arm(self):
        """무장 4 단계를 스크립트가 밟는다 — 사람이 안 친다."""
        self.spam(0.0, 1.5)                       # ① zero-hold -> READY
        self.call(True)                           # ② -> ARMING/PENDING
        self.pump(1.5)                            # ③ quiet 장벽 -> ARMED
        return self.state()

    def disarm(self):
        self.call(False)
        self.pump(0.5)


def gate_headers_unchanged():
    """축소 전제 전량을 확인한다 — 두 헤더 **와** `.ino` 통합 배선 7 함수."""
    bad = []
    for name, want in GATE_SHA256.items():
        path = os.path.join(SKETCH, name)
        try:
            with open(path, "rb") as handle:
                got = hashlib.sha256(handle.read()).hexdigest()
        except OSError as error:
            bad.append("%s 를 못 읽는다 (%s)" % (name, error))
            continue
        if got != want:
            bad.append("%s  실제 %s\n            기대 %s" % (name, got, want))

    # 🔴 검토 §68.3 — 생략한 7 행이 지나는 `.ino` 배선까지 잠근다.
    try:
        sys.path.insert(0, os.path.join(ROOT, "tools"))
        import ino_host_probe as probe                  # noqa: PLC0415
        source = probe.load()
    except Exception as error:                          # noqa: BLE001
        return bad + ["`.ino` 를 못 읽는다 (%s) — 축소 전제를 확인할 수 없다" % error]

    for name, want in INO_WIRING_SHA256.items():
        try:
            got = hashlib.sha256(probe.function(source, name).encode()).hexdigest()
        except Exception as error:                      # noqa: BLE001
            bad.append("`.ino` 의 %s 를 못 뗀다 (%s)" % (name, error))
            continue
        if got != want:
            bad.append(".ino:%s  실제 %s\n            기대 %s" % (name, got, want))
    return bad


def main():
    print("=" * 74)
    print("§7-c-E2 배선 6 행 — 🔴 전량 바퀴 공중 · 안전요원 E-stop")
    print("=" * 74)

    bad = gate_headers_unchanged()
    if bad:
        print("\n\033[31m판정 불능\033[0m — 게이트 헤더가 승인 시점과 다르다:")
        for line in bad:
            print("    " + line)
        print("\n  🔴 이 축소는 **상태기계·정지 배선·`.ino` 통합 배선이"
              " 그대로일 때만** 성립한다.")
        print("     헤더가 바뀌었으면 §7-c-E **13 행 전량**을 밟는다. 축소하지 않는다.")
        return 2
    print("  ✅ 게이트 헤더 2 + `.ino` 통합 배선 %d 함수 sha256 일치 —"
          " 축소 전제 성립 (§7-c-E2)" % len(INO_WIRING_SHA256))

    rclpy.init()
    node = Field()
    if not node.cli.wait_for_service(timeout_sec=10.0):
        print("\n\033[31m판정 불능\033[0m — /drive/enable 서비스가 안 보인다."
              " agent·펌웨어를 먼저 본다")
        return 2
    node.pump(1.5)

    try:
        # ── [1/6] 부정 1 ────────────────────────────────────────────────────
        print("\n[1/6] 부정 1 — E-stop 누른 채 0.05 발행 → 해제해도 안 돌아야 한다")
        print("      🔴 래치의 존재 이유다. 돌면 즉시 중단한다.")
        wait("E-stop 을 **누른** 상태로 둔다")
        node.spam(0.05, 2.0)
        print("      ... 0.05 발행 중. **지금 E-stop 을 떼세요.**")
        node.spam(0.05, 5.0)
        z, en = node.state(), node.enabled()
        turned = ask("바퀴가 하나라도 돌았습니까?")
        record("부정 1  잔류 명령으로 무장되지 않는다",
               (not turned) and z == 0 and en is not True,
               "z=%s(%s) enabled=%s" % (z, STATE.get(z, "?"), en))
        node.spam(0.0, 1.0)

        # ── [2/6][3/6] 역회귀 1·2 ───────────────────────────────────────────
        print("\n[2/6] 역회귀 1 — 정상 무장 뒤 0.05 → 🔴 **네 바퀴 다** 돌아야 한다")
        print("      ⚠ 08-13 밤에 왼쪽 드라이버가 빠진 채 '통과' 로 기록된 자리다.")
        wait("바퀴 공중 확인 · E-stop 에 손")
        z = node.arm()
        if z != 2:
            record("역회귀 1  무장", False, "z=%s — ARMED 가 아니다" % z)
        else:
            node.spam(0.05, 4.0)
            four = ask("🔴 **네 바퀴가 전부** 돌았습니까? (하나라도 아니면 n)")
            record("역회귀 1  ARMED 에서 네 바퀴가 돈다", four)

            print("\n[3/6] 역회귀 2 — 발행을 끊는다 → 0.5 초 안에 watchdog 이 세운다")
            node.pump(2.5)
            stopped = ask("바퀴가 멈췄습니까?")
            record("역회귀 2  watchdog 이 살아 있다", stopped)
        node.disarm()

        # ── [4/6] 부정 4 ────────────────────────────────────────────────────
        print("\n[4/6] 부정 4 — ARMED 주행 중 E-stop → 즉시 정지 · z=0 · enabled=false")
        wait("E-stop 을 **뗀** 상태에서 시작한다")
        z = node.arm()
        if z != 2:
            record("부정 4  무장", False, "z=%s" % z)
        else:
            print("      ... 0.05 주행 중. **지금 E-stop 을 누르세요.**")
            node.spam(0.05, 5.0)
            z, en = node.state(), node.enabled()
            stopped = ask("바퀴가 즉시 멈췄습니까?")
            record("부정 4  E-stop 이 주행을 끊는다",
                   stopped and z == 0 and en is not True,
                   "z=%s(%s) enabled=%s" % (z, STATE.get(z, "?"), en))
        node.spam(0.0, 1.0)
        node.disarm()

        # ── [5/6] 부정 7 ────────────────────────────────────────────────────
        print("\n[5/6] 부정 7 — ARMED 주행 중 disable → **응답이 오는 순간 이미 서 있어야**")
        print("      🔴 해제 뒤에도 0.05 를 계속 보낸다 — watchdog 을 공범에서 뺀다.")
        wait("E-stop 을 **뗀** 상태 · 바퀴 공중")
        z = node.arm()
        if z != 2:
            record("부정 7  무장", False, "z=%s" % z)
        else:
            node.spam(0.05, 3.0)
            t_call = time.monotonic()
            res = node.call(False)
            latency = (time.monotonic() - t_call) * 1000.0
            print("      응답 success=%s · 왕복 %.1f ms — 지금 바퀴를 보세요"
                  % (getattr(res, "success", None), latency))
            node.spam(0.05, 3.0)          # 🔴 해제 뒤에도 계속 준다
            z = node.state()
            already = ask("응답이 온 시점에 바퀴가 **이미** 서 있었습니까?")
            record("부정 7  해제가 그 자리에서 세운다 (§54.1)",
                   already and z == 0, "z=%s · 왕복 %.1f ms" % (z, latency))
        node.spam(0.0, 1.0)

        # ── [6/6] 부정 8 ────────────────────────────────────────────────────
        print("\n[6/6] 부정 8 — ARMED 주행 중 NaN → 즉시 정지 · z=0 · 재무장 필요")
        wait("바퀴 공중 · E-stop 에 손")
        z = node.arm()
        if z != 2:
            record("부정 8  무장", False, "z=%s" % z)
        else:
            node.spam(0.05, 3.0)
            nan_msg = Twist()
            nan_msg.linear.x = float("nan")
            for _ in range(5):
                node.pub.publish(nan_msg)
                rclpy.spin_once(node, timeout_sec=0.05)
            node.pump(2.0)
            z = node.state()
            stopped = ask("바퀴가 즉시 멈췄습니까?")
            record("부정 8  NaN 이 fail-closed 로 간다 (§54.3)",
                   stopped and z == 0, "z=%s(%s)" % (z, STATE.get(z, "?")))
        node.spam(0.0, 1.0)
        node.disarm()

    except KeyboardInterrupt:
        print("\n\033[31m중단됨\033[0m — 부분 결과는 판정으로 쓰지 않는다")
        return 2
    finally:
        try:
            node.spam(0.0, 0.5)
            node.call(False)
        except Exception:                                    # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.shutdown()

    # ── 결과 ────────────────────────────────────────────────────────────────
    passed = sum(1 for _r, ok, _n in RESULTS if ok)
    print("\n" + "=" * 74)
    print("배선 6 행: %d PASS / %d FAIL" % (passed, len(RESULTS) - passed))
    for row, ok, note in RESULTS:
        print("  %s  %-42s %s" % ("✅" if ok else "🔴", row, note))
    if passed != 6 or len(RESULTS) != 6:
        print("\n🔴 전량 통과가 아니다 — R1·R2 지면 주행을 금지한다 (§7-a 셋째 줄)")
        return 1
    print("\n✅ 배선 6 행 전량 통과.")
    print("   ⚠ 이것이 증명하지 않는 것: 상태 전이(호스트 967/0 이 봤다) ·")
    print("     PWM 파형 · 응답이 클라이언트에 닿은 시각.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
