#!/usr/bin/env python3
"""PCB 이식·재조립 직후 **1분 배선 점검** — 40분짜리 13행을 밟기 전에 GO/NO-GO 를 낸다.

사용 (🔴 무장은 사람이 먼저 한다 — 이 도구는 무장하지 않는다):
    python3 tools/pcb_wiring_check.py                 # 🔵 기본 = 바퀴 공중
    python3 tools/pcb_wiring_check.py --ground        # 지면 (IMU 교차검증이 열린다)
    python3 tools/pcb_wiring_check.py --hold 8        # 국면당 시간 늘리기
    python3 tools/pcb_wiring_check.py --dry           # 계획만 인쇄

왜 이 도구가 있나
-----------------
재조립은 **배선을 다시 꽂는 일**이고, 이 프로젝트는 그 자리에서 두 번 크게 당했다:

    2026-08-11  좌전륜 **엔코더 부호 반전** — 5초 직진에 `69.5°` 가 적분됐는데
                로봇은 눈으로 완벽히 직진했다. 🔴 **육안으로는 절대 안 잡힌다**
                (네 바퀴가 지면에 묶여 빠른 바퀴는 미끄러질 뿐이다).
    2026-08-13  **좌측 드라이버가 빠진 채** 시험이 "통과"로 기록됐다 (예약 42).

둘 다 *"돌긴 도는데 값이 거짓말한다"* 는 모양이라, 다음 40분·2시간이 통째로 무효가 됐다.
이 도구는 그 두 고장을 **1분 안에** 드러내는 것만 한다.

무엇을 가르는가 — 그리고 못 가르는 것
--------------------------------------
🔴 **바퀴별 엔코더가 지금 발행되지 않는다** (예약 48 의 선결 항목이 아직 없다). 그래서
**"어느 바퀴"까지는 못 간다 — "좌/우 어느 쪽이 어긋났는가"까지다.**

원리는 `tools/drive_encoder_check.py` 가 이미 적어 둔 그것이다. 펌웨어는

    deltaLeft  = 0.5*(dFL + dRL)      deltaRight = 0.5*(dFR + dRR)
    deltaYaw   = (deltaRight - deltaLeft) / WHEEL_BASE

로 좌우를 평균한다. 그래서 **한쪽 바퀴의 엔코더가 죽거나 뒤집히면 그 쪽 평균이 무너지고
직진 명령에 없던 회전이 생긴다.** 이 도구는 그 **유령 회전**을 본다.

| 검사 | 무엇이 걸리나 |
|---|---|
| 정지 드리프트 | 엔코더 부유·노이즈 |
| 전진/후진 응답 | 모터 무응답 — 드라이버·전원·모터선 |
| 🔴 유령 회전 | **좌우 엔코더 불균형** (08-11 고장) |
| 전후 극성 | 모터 방향 반전 |
| 회전 부호 | 좌우 채널 뒤바뀜 |
| IMU 교차 (`--ground`) | odom 이 거짓말하는가 (엔코더 vs 실제) |
| `applied_pwm` 대칭 | 제어 계산 문제인가 배선 문제인가 |

🔴 이 도구가 하지 않는 것
------------------------
- **무장하지 않는다.** 사람이 `/drive/enable` 로 한다.
- **E-stop 게이트를 대신하지 않는다.** 여기선 1회 왕복만 안내하고, 신뢰 회수는
  `tools/estop_toggle_check.py` 의 **10회**가 한다(`ELECTRICAL_BASELINE §7`).
- **보정하지 않는다.** 배율·게인은 안 본다. **연결됐는가**만 본다.
- **13행을 대신하지 않는다.** 이건 그 앞에 서는 싸구려 관문이다.

⚠ 실행 전 (사용자 상시 규칙)
    ros2 topic echo /drive/enabled --qos-reliability best_effort --once   # data: true
    ros2 topic echo /drive/diag    --qos-reliability best_effort --once   # z: 2.0 ARMED
    ros2 topic echo /estop/state   --qos-reliability best_effort --once   # data: false
🔴 **첫 점검은 바퀴를 공중에 띄우고 한다** — 배선이 틀렸을 때 로봇이 튀어나가지 않는다.
"""

import argparse
import math
import os
import re
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from firmware_constants import firmware_double  # noqa: E402

BEST_EFFORT = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
GREEN, RED, YEL, OFF = "\033[32m", "\033[31m", "\033[33m", "\033[0m"

#: 정지 국면에서 이보다 크면 엔코더가 부유한다 [m/s], [rad/s].
IDLE_V, IDLE_W = 0.005, 0.020
#: 명령이 나갔는데 이보다 작으면 "안 돈다"로 본다.
RESP_V, RESP_W = 0.020, 0.050
#: 🔴 유령 회전 임계는 **절대값이 아니라 고장 신호의 비율**이다.
#:   한쪽 엔코더가 뒤집히면 deltaLeft=0 이 되어 유령 ω = |v|/ODOM_WHEEL_BASE 가 되고,
#:   죽으면 그 절반이다. 그래서 그 크기를 1.0 으로 놓고 재는 것이 물리적으로 맞다.
#:   ⚠ 절대 임계(0.10/0.20)로 두면 시험속도 0.10 에서 **사망(0.060)을 통째로 놓친다** —
#:     이 값은 속도에 비례하므로 공중/지면에서도 자동으로 따라간다.
#:   실측 대조 (v=0.10 · ODOM_WHEEL_BASE=0.829):
#:     부호반전 1.00 · 사망 0.50 · 실제 좌향 편향(예약 39) 0.048
GHOST_RATIO_WARN, GHOST_RATIO_FAIL = 0.20, 0.40


class Wiring(Node):
    def __init__(self):
        super().__init__("pcb_wiring_check")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.odom = []        # (t, vx, wz)
        self.imu = []         # (t, wz)
        self.armed = None
        self.estop = None
        self.pwm = None       # (FL, RL, FR, RR)
        self.create_subscription(Odometry, "/odom", self._odom, BEST_EFFORT)
        self.create_subscription(Imu, "/imu/data", self._imu, BEST_EFFORT)
        self.create_subscription(Bool, "/drive/enabled",
                                 lambda m: setattr(self, "armed", m.data), BEST_EFFORT)
        self.create_subscription(Bool, "/estop/state",
                                 lambda m: setattr(self, "estop", m.data), BEST_EFFORT)
        self.create_subscription(String, "/firmware/info", self._info, BEST_EFFORT)

    def _odom(self, m):
        self.odom.append((time.time(), m.twist.twist.linear.x, m.twist.twist.angular.z))

    def _imu(self, m):
        self.imu.append((time.time(), m.angular_velocity.z))

    def _info(self, m):
        got = dict(re.findall(r"(\w+)=([^;]+)", m.data))
        raw = got.get("applied_pwm")
        if raw:
            try:
                vals = [int(v) for v in raw.split(",")]
                if len(vals) == 4:
                    self.pwm = tuple(vals)
            except ValueError:
                pass

    def spin_for(self, sec):
        end = time.time() + sec
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def phase(self, vx, wz, hold, settle):
        """`vx`·`wz` 를 20Hz 로 인가하고 안정 구간의 평균을 돌려준다."""
        self.pwm = None
        tw = Twist()
        tw.linear.x = vx
        tw.angular.z = wz
        t0 = time.time()
        end = t0 + hold
        while rclpy.ok() and time.time() < end:
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.05)
        t1 = time.time()
        a, b = t0 + settle, t1
        ov = [v for t, v, _ in self.odom if a <= t <= b]
        ow = [w for t, _, w in self.odom if a <= t <= b]
        iw = [w for t, w in self.imu if a <= t <= b]
        mean = lambda s: sum(s) / len(s) if s else float("nan")   # noqa: E731
        return {"v": mean(ov), "w": mean(ow), "imu": mean(iw),
                "n": len(ov), "pwm": self.pwm}

    def stop(self):
        for _ in range(12):
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)


def fmt(x, w=8, p=4):
    return ("%*.*f" % (w, p, x)) if not math.isnan(x) else "%*s" % (w, "—")


def main(argv=None):
    ap = argparse.ArgumentParser(description="PCB 이식 뒤 배선 점검")
    ap.add_argument("--ground", action="store_true",
                    help="지면에서 한다 (IMU 교차검증이 열린다). 기본은 바퀴 공중")
    ap.add_argument("--lin", type=float, default=0.10, help="전후 시험 속도 [m/s]")
    ap.add_argument("--ang", type=float, default=0.45,
                    help="회전 시험 속도 [rad/s] — 불감대 위여야 한다")
    ap.add_argument("--hold", type=float, default=6.0,
                    help="국면당 시간 [s]. /firmware/info 가 5초 주기라 그보다 커야 한다")
    ap.add_argument("--settle", type=float, default=1.5)
    ap.add_argument("--dry", action="store_true")
    args = ap.parse_args(argv)

    mode = "지면" if args.ground else "🔵 바퀴 공중"
    print("=" * 70)
    print("  PCB 배선 점검   모드 = %s" % mode)
    print("  국면 5개 × %.1fs ≈ %.0f초" % (args.hold, 5 * args.hold + 4))
    if not args.ground:
        print("  ⚠ 공중 모드는 IMU 교차검증을 못 한다 — 로봇이 실제로 안 움직인다.")
        print("    공중에서 통과하면 --ground 로 한 번 더 한다.")
    print("  🔴 물리 E-stop 담당자 상시 · 바퀴 주변을 비운다")
    print("=" * 70)
    if args.dry:
        print("  --dry 라 명령을 보내지 않는다.")
        return 0

    rclpy.init()
    node = Wiring()
    try:
        node.spin_for(3.0)
        if node.armed is not True:
            print("🔴 무장돼 있지 않다 (/drive/enabled = %s). 이 도구는 무장하지 않는다."
                  % node.armed)
            return 2
        if node.estop is not False:
            print("🔴 E-stop 상태가 %s 다." % node.estop)
            return 2
        if not node.odom:
            print("🔴 /odom 무수신 — micro-ROS agent 부터 본다.")
            return 2

        try:
            base = firmware_double("ODOM_WHEEL_BASE")
        except (KeyError, OSError) as err:
            print("🔴 판정 불능 — `.ino` 에서 ODOM_WHEEL_BASE 를 못 읽었다: %s" % err)
            return 2

        plan = [
            ("정지",   0.0,        0.0),
            ("전진",   args.lin,   0.0),
            ("후진",  -args.lin,   0.0),
            ("좌회전", 0.0,        args.ang),
            ("우회전", 0.0,       -args.ang),
        ]
        res = {}
        print("\n  %-8s %-9s %-9s %-9s %-6s %s"
              % ("국면", "odom v", "odom ω", "IMU ω", "표본", "applied_pwm FL,RL,FR,RR"))
        for name, vx, wz in plan:
            r = node.phase(vx, wz, args.hold, args.settle)
            res[name] = r
            print("  %-8s %s %s %s %-6d %s"
                  % (name, fmt(r["v"]), fmt(r["w"]), fmt(r["imu"]), r["n"],
                     "—" if r["pwm"] is None else ",".join(str(v) for v in r["pwm"])))
            node.stop()
            node.spin_for(1.0)
        node.stop()

        # ── 판정 ────────────────────────────────────────────────────
        fails, warns, notes = [], [], []
        idle, fwd, bwd, left, right = (res[k] for k in
                                       ("정지", "전진", "후진", "좌회전", "우회전"))

        print("\n" + "=" * 70)
        print("  검사")
        print("=" * 70)

        def say(ok, label, detail):
            mark = "%s통과%s" % (GREEN, OFF) if ok else "%s실패%s" % (RED, OFF)
            print("  [%s] %-26s %s" % (mark, label, detail))

        # 1. 정지 드리프트
        ok = abs(idle["v"]) < IDLE_V and abs(idle["w"]) < IDLE_W
        say(ok, "정지 드리프트",
            "v %.4f (<%.3f) · ω %.4f (<%.3f)" % (idle["v"], IDLE_V, idle["w"], IDLE_W))
        if not ok:
            fails.append("정지 중 엔코더가 값을 낸다 — 부유·노이즈·접지 의심")

        # 2. 전후 응답
        for label, r, sign in (("전진 응답", fwd, +1), ("후진 응답", bwd, -1)):
            ok = abs(r["v"]) > RESP_V
            say(ok, label, "v %.4f (|v|>%.3f)" % (r["v"], RESP_V))
            if not ok:
                fails.append("%s: 명령이 나갔는데 바퀴가 안 돈다 — 드라이버·전원·모터선"
                             % label)

        # 3. 전후 극성
        ok = fwd["v"] > 0 and bwd["v"] < 0
        say(ok, "전후 극성", "전진 %+.4f · 후진 %+.4f" % (fwd["v"], bwd["v"]))
        if not ok:
            fails.append("전후 부호가 뒤집혔다 — 모터선 극성 또는 엔코더 A/B 교환")

        # 4. 🔴 유령 회전 — 고장 신호 크기 대비 비율로 본다
        for label, r in (("전진", fwd), ("후진", bwd)):
            unit = abs(r["v"]) / base if base else float("nan")
            ratio = abs(r["w"]) / unit if unit and unit > 1e-6 else float("nan")
            if math.isnan(ratio):
                notes.append("%s: 속도가 0 이라 유령 회전을 판정할 수 없다" % label)
                continue
            ok = ratio < GHOST_RATIO_WARN
            say(ok, "유령 회전 (%s)" % label,
                "ω %.4f / 고장신호 %.4f = %.2f  (경고 %.2f · 실패 %.2f)"
                % (r["w"], unit, ratio, GHOST_RATIO_WARN, GHOST_RATIO_FAIL))
            if ratio >= GHOST_RATIO_FAIL:
                guess = "부호 반전" if ratio >= 0.75 else "엔코더 사망"
                fails.append("🔴 %s 중 유령 회전 비 %.2f — **좌우 엔코더 불균형**. "
                             "한쪽 바퀴 %s 으로 보인다 (08-11 고장). ω 부호가 %s 쪽을 가리킨다"
                             % (label, ratio, guess, "우" if r["w"] > 0 else "좌"))
            elif ratio >= GHOST_RATIO_WARN:
                warns.append("%s 중 유령 회전 비 %.2f — 경계값. --ground 로 IMU 대조를 한다"
                             % (label, ratio))

        # 5. 회전 부호
        ok = left["w"] > RESP_W and right["w"] < -RESP_W
        say(ok, "회전 부호", "좌 %+.4f · 우 %+.4f (|ω|>%.3f)"
            % (left["w"], right["w"], RESP_W))
        if not ok:
            if abs(left["w"]) < RESP_W and abs(right["w"]) < RESP_W:
                notes.append("회전이 둘 다 약하다 — 공중이면 정상일 수 있고, 지면이면 "
                             "불감대다. tools/drive_deadband_sweep.py 로 D 를 잰다")
            else:
                fails.append("회전 부호가 어긋났다 — 좌우 채널이 뒤바뀐 것을 의심한다")

        # 6. IMU 교차 (지면 전용)
        if args.ground:
            for label, r in (("좌회전", left), ("우회전", right)):
                if math.isnan(r["imu"]) or abs(r["imu"]) < 0.01:
                    warns.append("%s: IMU ω 가 거의 0 이다 — 로봇이 실제로 안 돌았다" % label)
                    continue
                same = (r["w"] > 0) == (r["imu"] > 0)
                say(same, "IMU 부호 대조 (%s)" % label,
                    "odom %+.4f vs IMU %+.4f" % (r["w"], r["imu"]))
                if not same:
                    fails.append("🔴 %s 에서 odom 과 IMU 의 회전 **부호가 반대**다 — "
                                 "엔코더가 거짓말한다" % label)
                else:
                    ratio = abs(r["w"]) / abs(r["imu"])
                    if not (0.5 <= ratio <= 2.0):
                        warns.append("%s: odom/IMU 비 %.2f — 배율이 어긋난다(배선은 맞다)"
                                     % (label, ratio))
            f = fwd
            f_unit = abs(f["v"]) / base if base else 0.0
            if (not math.isnan(f["imu"]) and f_unit > 1e-6
                    and abs(f["w"]) / f_unit >= GHOST_RATIO_WARN):
                if abs(f["imu"]) < 0.02:
                    fails.append("🔴 직진 중 odom 은 회전을 주장하는데 IMU 는 0 이다 — "
                                 "**로봇은 직진했고 엔코더가 거짓말한다**(08-11 그대로)")
        else:
            notes.append("공중 모드라 IMU 교차검증을 건너뛰었다 — --ground 로 한 번 더 한다")

        # 7. applied_pwm 대칭
        if fwd["pwm"]:
            fl, rl, fr, rr = fwd["pwm"]
            lsum, rsum = abs(fl) + abs(rl), abs(fr) + abs(rr)
            ok = max(lsum, rsum) == 0 or abs(lsum - rsum) <= 0.25 * max(lsum, rsum)
            say(ok, "applied_pwm 좌우 대칭",
                "좌 %d,%d · 우 %d,%d" % (fl, rl, fr, rr))
            if not ok:
                notes.append("직진인데 좌우 PWM 이 비대칭이다 — 제어가 이미 보정 중이라는 "
                             "뜻이고 원인은 엔코더 쪽일 수 있다")
        else:
            notes.append("/firmware/info 를 국면 안에 못 받았다(5초 주기) — --hold 를 늘린다")

        # ── 결론 ────────────────────────────────────────────────────
        print("\n" + "=" * 70)
        for n in notes:
            print("  ·  %s" % n)
        for w in warns:
            print("  %s⚠%s  %s" % (YEL, OFF, w))
        if fails:
            print("  %sNO-GO%s — %d 건" % (RED, OFF, len(fails)))
            for f in fails:
                print("      · %s" % f)
            print()
            print("  🔴 13행·주행 시험으로 넘어가지 않는다. 배선을 먼저 본다.")
            print("     바퀴별로 좁히려면 손으로 한 바퀴씩 굴린 bag 을 만들어")
            print("     tools/drive_encoder_check.py 로 분해한다.")
            print("=" * 70)
            return 1
        print("  %sGO%s   배선 연결 이상 없음 (%s 모드)" % (GREEN, OFF, mode))
        if not args.ground:
            print("  → 다음: --ground 로 한 번 더 → E-stop 10회"
                  "(tools/estop_toggle_check.py) → 무장 13행")
        else:
            print("  → 다음: E-stop 10회(tools/estop_toggle_check.py) → 무장 13행")
        print("  🔴 이건 '연결됐다'까지다. 배율·게인은 안 봤다.")
        print("=" * 70)
        return 0
    finally:
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
