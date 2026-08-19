#!/usr/bin/env python3
"""제자리 회전 불감대 `D` 를 계단 스윕으로 재고 **명령 등가 상한과 대조해 판정까지 낸다**.

사용 (🔴 이미 무장돼 있어야 한다 — 이 도구는 무장하지 않는다):
    python3 tools/drive_deadband_sweep.py
    python3 tools/drive_deadband_sweep.py --steps 0.20,0.26,0.32,0.38,0.44,0.48
    python3 tools/drive_deadband_sweep.py --hold 7 --settle 2.0
    python3 tools/drive_deadband_sweep.py --dry        # 명령 안 보내고 계획만 인쇄

왜 이 도구가 있나
-----------------
08-18 은 5점 스윕을 **손으로** 했다. 08-20 재조립 뒤에는 이 값이 그날의 분기를 정한다:

    D <  상한  →  Nav2 튜닝으로 간다 (`max_angular_accel = D / dt`)
    D >= 상한  →  🔴 Nav2 로는 못 고친다. 구동부(FF·게인·기구)로 넘어간다
                  ⚠ 엄격한 불가능 증명은 `D > 상한` 이고 등호는 여유가 정확히 0 이다.
                    `>=` 로 가르는 것이 검토 §81 이 승인한 **보수 경계**다.

손계산이 그 분기를 떠받치면 안 된다. 08-13 에 줄자 한 번을 잘못 읽어 상수·문서·검토
3회차가 통째로 기각된 자리가 정확히 이 구조였다.

무엇을 재는가
-------------
명령 ω 를 오름차순 계단으로 인가하고 각 계단의 **실측 ω 를 IMU 자이로**로 잰다.
그다음 `실제 = a · (명령 − D)` 를 최소제곱으로 맞춰 `D`(불감대)와 `a`(전달률)를 낸다.

🔴 **자이로가 정본인 이유** — `PITFALLS §12` 가 *"유효 윤거는 상수가 아니다"* 로 적어 둔
자리다. 엔코더 yaw 는 스크럽 때문에 회전에서 믿을 수 없다. 08-18 §1-h-1 에서 제자리
회전은 `odom`↔자이로가 100.2% 로 맞았지만, 그 일치 자체가 매번 성립한다는 보장이 없다.

🔵 **덤으로 회전 plant PWM 을 같이 낸다** (예약 51 선결 측정). `/firmware/info` 의
`applied_pwm_max` 는 **무장 epoch 안의 누적 최대**라 계단별 값이 아니다. 그런데 스윕이
**오름차순**이면 누적 최대가 계단마다 단조 증가하므로 **끝값의 차분이 계단별 PWM** 이 된다.
⚠ `applied_pwm_epoch` 가 스윕 도중 바뀌면(= 재무장) 그 대조가 깨지므로 **판정 불능**으로 낸다.
⚠ `/firmware/info` 는 **5초 주기 스냅샷**이라 `--hold` 가 그보다 짧으면 계단이 비어 나온다.

🔴 이 도구가 하지 않는 것
------------------------
- **무장하지 않는다.** 무장은 사람이 `/drive/enable` 로 한다. 도구가 무장시키면
  "내가 언제 무장했는지"를 사람이 모르는 상태가 생긴다.
- **주행하지 않는다.** `linear.x` 는 항상 0 이다. 제자리 회전만 본다.
  🔴 주행 중 조향은 **다른 물리**다(`MASTER_PLAN §7` 예약 40 — 주행 중엔 불감대가 없다).
- **상수를 고치지 않는다.** 숫자와 판정만 낸다.

⚠ 실행 전 확인 (`docs/TEST_GATES.md` · 사용자 상시 규칙)
    ros2 topic echo /drive/enabled --qos-reliability best_effort --once   # data: true
    ros2 topic echo /drive/diag    --qos-reliability best_effort --once   # z: 2.0 ARMED
    ros2 topic echo /estop/state   --qos-reliability best_effort --once   # data: false
🔴 물리 E-stop 담당자가 옆에 있어야 한다. 바퀴는 지면에 두되 주변을 비운다.
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
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from firmware_constants import firmware_double  # noqa: E402

DEFAULT_STEPS = (0.20, 0.24, 0.28, 0.32, 0.36, 0.40, 0.44, 0.48)
#: 이보다 작은 실측은 자이로 잡음으로 보고 적합에서 뺀다 [rad/s].
NOISE_FLOOR = 0.010
BEST_EFFORT = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)


def rotation_ceiling():
    """제자리 회전에서 **명령으로 표현 가능한** 최대 등가 각속도 [rad/s].

    `applySkidSteerCommand()` 는 `wheel = ±ω · CMD_WHEEL_BASE/2` 를 만든 뒤
    `max(|L|,|R|) > MAX_WHEEL_CMD` 면 좌우를 같은 비로 줄인다. 제자리(linear=0)에서
    그 한계는 `MAX_WHEEL_CMD / (CMD_WHEEL_BASE/2)` 다.

    🔴 `MAX_ANGULAR_CMD` 가 아니다 — 그 값(0.50)은 제자리에서 **더 낮은 휠 상한에 가려진다**
    (검토 §80·§81 · 예약 50-2). `ω=0.50` → 휠 ±0.155 → 배율 0.967742 → ±0.15 → 다시 0.4839.

    🔴 **이것은 물리 상한이 아니라 *명령 등가 상한* 이다** (검토 §81 한정) — 제자리 휠
    목표로 **표현 가능한** 각속도의 한계이지, 슬립·과도응답까지 봉인한 값이 아니다.
    """
    return firmware_double("MAX_WHEEL_CMD") / (firmware_double("CMD_WHEEL_BASE") / 2.0)


def least_squares(xs, ys):
    """`y = m·x + b` 최소제곱. (m, b, r2) — 점이 2개 미만이면 None."""
    n = len(xs)
    if n < 2:
        return None
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0.0:
        return None
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    m = sxy / sxx
    b = my - m * mx
    ss_res = sum((y - (m * x + b)) ** 2 for x, y in zip(xs, ys))
    ss_tot = sum((y - my) ** 2 for y in ys)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    return m, b, r2


class Sweep(Node):
    def __init__(self):
        super().__init__("drive_deadband_sweep")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.gyro = []               # (t, wz)
        self.armed = None
        self.estop = None
        self.pwm_max = None
        self.pwm_epoch = None
        self.create_subscription(Imu, "/imu/data", self._imu, BEST_EFFORT)
        self.create_subscription(Bool, "/drive/enabled", self._armed, BEST_EFFORT)
        self.create_subscription(Bool, "/estop/state", self._estop, BEST_EFFORT)
        self.create_subscription(String, "/firmware/info", self._info, BEST_EFFORT)

    def _imu(self, m):
        self.gyro.append((time.time(), m.angular_velocity.z))

    def _armed(self, m):
        self.armed = m.data

    def _estop(self, m):
        self.estop = m.data

    def _info(self, m):
        got = dict(re.findall(r"(\w+)=([^;]+)", m.data))
        try:
            self.pwm_max = int(got["applied_pwm_max"])
            self.pwm_epoch = int(got["applied_pwm_epoch"])
        except (KeyError, ValueError):
            pass

    def spin_for(self, seconds):
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def publish_for(self, wz, seconds):
        """`wz` 를 20Hz 로 `seconds` 동안 인가한다. linear 는 항상 0."""
        tw = Twist()
        tw.angular.z = wz
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.05)

    def stop(self):
        for _ in range(10):
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    def mean_abs_gyro(self, t_from, t_to):
        vals = [abs(w) for t, w in self.gyro if t_from <= t <= t_to]
        return (sum(vals) / len(vals), len(vals)) if vals else (float("nan"), 0)


def main(argv=None):
    ap = argparse.ArgumentParser(description="제자리 회전 불감대 스윕")
    ap.add_argument("--steps", default=",".join("%.2f" % s for s in DEFAULT_STEPS),
                    help="명령 ω 계단 [rad/s], 쉼표 구분. 🔴 오름차순이어야 PWM 차분이 성립한다")
    ap.add_argument("--hold", type=float, default=7.0,
                    help="계단당 인가 시간 [s]. /firmware/info 가 5초 주기라 그보다 커야 한다")
    ap.add_argument("--settle", type=float, default=2.0, help="계단 앞 버리는 시간 [s]")
    ap.add_argument("--label", default="?")
    ap.add_argument("--dry", action="store_true", help="명령 없이 계획만 인쇄")
    args = ap.parse_args(argv)

    steps = [float(s) for s in args.steps.split(",") if s.strip()]
    if steps != sorted(steps):
        print("🔴 --steps 가 오름차순이 아니다 — applied_pwm_max 차분이 성립하지 않는다.")
        return 2
    if args.hold <= args.settle:
        print("🔴 --hold 가 --settle 보다 커야 한다.")
        return 2

    try:
        ceiling = rotation_ceiling()
    except (KeyError, OSError) as err:
        print("🔴 판정 불능 — `.ino` 에서 상수를 못 읽었다: %s" % err)
        return 2

    print("=" * 72)
    print("  제자리 회전 불감대 스윕   label=%s" % args.label)
    print("  계단 %s" % " ".join("%.2f" % s for s in steps))
    print("  계단당 %.1fs (앞 %.1fs 버림)   예상 소요 %.0fs"
          % (args.hold, args.settle, args.hold * len(steps) + 5))
    print("  🔵 명령 등가 상한 = MAX_WHEEL_CMD / (CMD_WHEEL_BASE/2) = %.6f rad/s" % ceiling)
    print("  🔴 물리 E-stop 담당자 · 주변 비움 · 바퀴 지면")
    print("=" * 72)
    if args.dry:
        print("  --dry 라 명령을 보내지 않는다.")
        return 0

    rclpy.init()
    node = Sweep()
    try:
        node.spin_for(3.0)                      # 상태 토픽 수신 대기
        if node.armed is not True:
            print("🔴 무장돼 있지 않다 (/drive/enabled = %s). 이 도구는 무장하지 않는다."
                  % node.armed)
            print("   무장 절차 = zero 를 0.5초 이상 발행 → /drive/enable true → 0.5초 대기")
            return 2
        if node.estop is not False:
            print("🔴 E-stop 상태가 %s 다. 풀고 다시." % node.estop)
            return 2
        if node.pwm_epoch is None:
            print("⚠ /firmware/info 를 아직 못 읽었다 — PWM 차분은 판정 불능으로 낸다.")

        epoch0 = node.pwm_epoch
        rows = []
        for w in steps:
            t0 = time.time()
            node.publish_for(w, args.hold)
            t1 = time.time()
            meas, n = node.mean_abs_gyro(t0 + args.settle, t1)
            rows.append({"cmd": w, "meas": meas, "n": n,
                         "pwm": node.pwm_max, "epoch": node.pwm_epoch})
            print("   명령 %.3f → 실측 %.4f rad/s  (표본 %d · pwm_max %s)"
                  % (w, meas, n, node.pwm_max))
        node.stop()

        print()
        print("=" * 72)
        print("  결과")
        print("=" * 72)
        epoch_ok = (epoch0 is not None
                    and all(r["epoch"] == epoch0 for r in rows if r["epoch"] is not None))
        if not epoch_ok:
            print("  ⚠ applied_pwm_epoch 가 스윕 도중 바뀌었다(재무장) — PWM 차분 판정 불능.")

        print("  %-8s %-10s %-8s %-10s %s" % ("명령ω", "실측ω", "표본", "pwm_max", "계단PWM"))
        prev = None
        for r in rows:
            d = "—"
            if epoch_ok and r["pwm"] is not None and prev is not None:
                d = "%+d" % (r["pwm"] - prev)
            if r["pwm"] is not None:
                prev = r["pwm"]
            print("  %-8.3f %-10.4f %-8d %-10s %s"
                  % (r["cmd"], r["meas"], r["n"],
                     "—" if r["pwm"] is None else str(r["pwm"]), d))

        live = [r for r in rows if r["meas"] > NOISE_FLOOR and not math.isnan(r["meas"])]
        print()
        if len(live) < 2:
            print("  🔴 판정 불능 — 잡음(%.3f) 위로 올라온 계단이 %d 개뿐이다."
                  % (NOISE_FLOOR, len(live)))
            print("     불감대가 시험 범위(최대 %.2f)보다 높을 수 있다. 계단을 올려 다시 잰다."
                  % steps[-1])
            return 1

        fit = least_squares([r["cmd"] for r in live], [r["meas"] for r in live])
        slope, intercept, r2 = fit
        if slope <= 0:
            print("  🔴 판정 불능 — 기울기가 0 이하다(%.4f). 부호·배선을 먼저 본다." % slope)
            return 1
        D = -intercept / slope

        print("  적합 (잡음 위 %d 계단):  실제 = %.4f × (명령 − %.4f)     R² = %.4f"
              % (len(live), slope, D, r2))
        print("  " + "-" * 66)
        print("  🔵 불감대 D = %.4f rad/s      전달률 a = %.3f" % (D, slope))
        print("  🔵 명령 등가 상한 = %.4f rad/s  여유 = %.4f (%.0f%%)"
              % (ceiling, ceiling - D, 100.0 * (ceiling - D) / ceiling))
        print("  " + "-" * 66)
        print()
        if D < ceiling:
            need = D / 0.05                      # dt = 1/controller_frequency(20Hz)
            cap = ceiling / 0.05
            print("  🟢 D < 상한 — **Nav2 튜닝으로 갈 수 있다.**")
            print("     max_angular_accel 재유도:  D/dt = %.2f  (상한 천장/dt = %.2f)"
                  % (need, cap))
            print("     ⚠ 여유가 %.0f%% 다. 30%% 밑이면 램프가 경계에 걸려 얇다(검토 §80 단서)."
                  % (100.0 * (ceiling - D) / ceiling))
            rc = 0
        else:
            print("  🔴 D >= 상한 — **Nav2 파라미터로는 못 고친다.**")
            print("     ⚠ 등호는 여유가 정확히 0 이다 — 검토 §81 이 승인한 보수 경계다.")
            print("     어떤 값을 써도 명령이 불감대를 못 넘는다. 구동부로 넘어간다:")
            print("     `MASTER_PLAN §7` 예약 51 — FF 에 회전 스크럽 항이 없고")
            print("     제어 여유가 INTEGRAL_PWM_LIMIT 에서 잘린다.")
            rc = 1
        print()
        print("  🔴 이 값은 판정 근거일 뿐이다 — 상수를 여기서 바꾸지 않는다.")
        print("     기록 = `REAL_ROBOT_VALUES §1-h` · 분기 = `CURRENT_HANDOFF` 완료 후 다음 단계.")
        return rc
    finally:
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
