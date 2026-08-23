#!/usr/bin/env python3
"""선형 불감대 `D_lin` 을 계단 스윕으로 잰다 — `guide_speed` 를 정하기 위한 것.

사용 (🔴 이미 무장돼 있어야 한다 — 이 도구는 무장하지 않는다):
    python3 tools/drive_linear_deadband.py --dry            # 계획만 인쇄
    python3 tools/drive_linear_deadband.py
    python3 tools/drive_linear_deadband.py --steps 0.04,0.06,0.08,0.10,0.12

왜 이 도구가 있나
-----------------
회전 불감대는 08-20 에 쟀다(`D = 0.2329 rad/s`, `drive_deadband_sweep.py`).
🔴 **선형 불감대는 한 번도 잰 적이 없다.**

그런데 미션의 `GUIDE` 는 `guide_speed` 로 저속 유도한다. 만약

    guide_speed  <  D_lin

이면 **로봇이 아예 안 움직인다.** 그리고 그 상태는 `FAULT` 도 아니다 —
저속은 '적용'됐고 goal 도 살아 있는데 주행만 0 이다. 진단이 아무것도 안 뜬다.
08-20 에 세 번 당한 **무증상 실패**와 같은 형태다.

구동부 명령 상한이 `MAX_LINEAR_CMD`(08-22 이후 `0.20 m/s`) 이므로, `D_lin` 이 그 근처면 *"저속 유도"* 라는
개념 자체가 성립하지 않는다. **그 판정이 이 측정의 목적이다.**

무엇을 재는가
-------------
명령 `linear.x` 를 오름차순 계단으로 인가하고 각 계단의 실측 전진속도를
`/odom` 의 `twist.linear.x` **중앙값**으로 잡는다. 그다음

    실제 = a · (명령 − D_lin)

를 최소제곱으로 맞춰 `D_lin`(불감대)과 `a`(전달률)를 낸다.

🔴 왜 평균이 아니라 중앙값인가 — 08-20 에 `/odom` 이 EMI 로 300 ms 씩 끊긴 적이
있다. 그때 튀는 표본 하나가 평균을 통째로 옮긴다. 중앙값은 안 옮긴다.

🔴 이 도구가 하지 않는 것
------------------------
- **무장하지 않는다.** 무장은 사람이 `/drive/enable` 로 한다.
- **회전을 섞지 않는다.** `angular.z` 는 항상 0 이다. 섞으면 스크럽이 들어와
  전진속도가 오염된다(`PITFALLS §12` — 유효 윤거는 상수가 아니다).

🔴 안전 — 이건 로봇이 **실제로 앞으로 가는** 시험이다
----------------------------------------------------
기본 계단·유지시간에서 총 이동거리는 약 **2.4 m** 다. 시작 전에 앞쪽으로
최소 **4 m** 를 비운다. `--dry` 로 예상 거리를 먼저 인쇄해서 확인할 것.
사람이 E-stop 에 손을 두고 시작한다.
"""

import math
import argparse
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

# 🔴 구동부 명령 상한 — `.ino` `MAX_LINEAR_CMD` 의 복사본이다 (2026-08-23 §91 P1-3).
#   구판은 0.12 를 세 자리에 하드코딩했고, 08-22 에 펌웨어가 0.20 으로 올라갔는데
#   따라가지 않아 **여유·판정을 틀린 기준으로 계산**했다.
#   정본 = `docs/REAL_ROBOT_VALUES.md §1-n`.
MAX_LINEAR_CMD = 0.20


class UsageError(ValueError):
    """사용자 입력이 실차로 나가기 전에 막는다."""


def parse_steps(text):
    """`--steps` 를 검증하며 파싱한다 (2026-08-23 §91 2회차 P1-2).

    🔴 구판은 `tuple(float(x) for x in text.split(','))` 한 줄이라 **아무 검증이 없었다.**
    이 값은 곧바로 `/cmd_vel` 로 나가는 **실제 주행 명령**이다. 음수·NaN·Inf·상한 초과·
    역순이 그대로 통과했다 — 검토가 `0.200001` 과 역순을 넣어 확인했다.

    계약: 각 계단은 **유한 · 0 초과 · `MAX_LINEAR_CMD` 이하**이고 **오름차순**이다.
    (오름차순을 요구하는 이유 = 불감대 산출이 "낮은 쪽부터 올려 처음 움직인 지점" 을
     찾는 절차라, 순서가 섞이면 `D_lin` 자체가 의미를 잃는다.)
    """
    raw = [x.strip() for x in text.split(',') if x.strip()]
    if not raw:
        raise UsageError('--steps 가 비어 있다')
    out = []
    for x in raw:
        try:
            v = float(x)
        except ValueError:
            raise UsageError(f'숫자가 아니다: {x!r}') from None
        if not math.isfinite(v):
            raise UsageError(f'유한값이 아니다: {x!r}')
        if v <= 0.0:
            raise UsageError(f'0 이하다: {v}')
        if v > MAX_LINEAR_CMD:
            raise UsageError(
                f'{v} 가 구동부 명령 상한 {MAX_LINEAR_CMD} 를 넘는다 — 실차로 못 보낸다')
        out.append(v)
    if any(b <= a for a, b in zip(out, out[1:])):
        raise UsageError(f'오름차순이 아니다(중복 포함): {out}')
    return tuple(out)


DEFAULT_STEPS = (0.04, 0.06, 0.08, 0.10, 0.12)


def fit_deadband(pts):
    """(명령, 실측) 점들에 `실제 = a·(명령 − D)` 를 최소제곱으로 맞춘다.

    움직인 계단(실측 > 0)만 쓴다 — 안 움직인 계단은 y=0 으로 눌려 있어
    직선에 넣으면 기울기를 끌어내린다.
    반환: (D, a, 사용한 점 수) 또는 None (점이 2개 미만).
    """
    used = [(c, m) for c, m in pts if m > 0.005]
    if len(used) < 2:
        return None
    n = len(used)
    sx = sum(c for c, _ in used)
    sy = sum(m for _, m in used)
    sxx = sum(c * c for c, _ in used)
    sxy = sum(c * m for c, m in used)
    den = n * sxx - sx * sx
    if abs(den) < 1e-12:
        return None
    a = (n * sxy - sx * sy) / den
    b = (sy - a * sx) / n
    if abs(a) < 1e-9:
        return None
    return (-b / a), a, n


class LinearSweep(Node):

    def __init__(self, steps, hold, settle, dry):
        super().__init__('drive_linear_deadband')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.samples = []
        self.collecting = False
        self.create_subscription(Odometry, '/odom', self._odom, 20)
        self.steps, self.hold, self.settle, self.dry = steps, hold, settle, dry

    def _odom(self, m):
        if self.collecting:
            self.samples.append(m.twist.twist.linear.x)

    def _send(self, v):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = 0.0        # 🔴 회전을 절대 섞지 않는다
        self.pub.publish(t)

    def _spin(self, secs):
        end = time.time() + secs
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def run(self):
        dist = sum(s * self.hold for s in self.steps)
        print(f'계단 {self.steps} · 유지 {self.hold}s · 정착 {self.settle}s')
        print(f'🔴 예상 총 이동거리 ≈ {dist:.2f} m — 앞쪽 {dist * 1.6:.1f} m 이상 비울 것')
        if self.dry:
            print('--dry — 명령을 보내지 않고 종료한다')
            return 0

        pts = []
        for v in self.steps:
            # 🔴 10 Hz 로 계속 보낸다 — 펌웨어 watchdog 이 500 ms 다.
            #   한 번만 쏘면 0.5초 뒤 스스로 선다.
            self.samples = []
            self.collecting = False
            end = time.time() + self.settle
            while time.time() < end and rclpy.ok():
                self._send(v)
                rclpy.spin_once(self, timeout_sec=0.1)
            self.collecting = True
            end = time.time() + self.hold
            while time.time() < end and rclpy.ok():
                self._send(v)
                rclpy.spin_once(self, timeout_sec=0.1)
            self.collecting = False
            med = statistics.median(self.samples) if self.samples else 0.0
            pts.append((v, med))
            print(f'  명령 {v:.3f}  →  실측 중앙값 {med:.4f} m/s  '
                  f'(표본 {len(self.samples)})')

        # 정지 — 0 을 넉넉히 보낸다
        for _ in range(15):
            self._send(0.0)
            rclpy.spin_once(self, timeout_sec=0.05)

        print()
        r = fit_deadband(pts)
        if r is None:
            print('🔴 판정 불능 — 움직인 계단이 2개 미만이다.')
            print('   그 자체가 결과다: 시험한 전 구간이 불감대 안이라는 뜻이고,')
            print('   그러면 guide_speed 를 이 범위에서 고를 수 없다.')
            return 1
        D, a, n = r
        print(f'선형 불감대 D_lin = {D:.4f} m/s   전달률 a = {a:.3f}   (점 {n}개)')
        print(f'구동부 명령 상한 = {MAX_LINEAR_CMD} m/s  →  여유 = '
              f'{MAX_LINEAR_CMD - D:.4f} m/s '
              f'({(MAX_LINEAR_CMD - D) / MAX_LINEAR_CMD * 100:.0f}%)')
        print()
        if D >= MAX_LINEAR_CMD:
            print('🔴 D_lin 이 명령 상한 이상이다 — 저속 유도가 성립하지 않는다.')
            print('   guide_speed 를 낮추는 방향으로는 못 푼다. 구동부(FF·게인) 문제다.')
        elif D >= 0.08:
            print('🔶 여유가 좁다. guide_speed 를 D_lin 위로 확실히 띄워야 한다.')
            print(f'   권장 guide_speed ≥ {min(MAX_LINEAR_CMD, D * 1.5):.3f} (D_lin 의 1.5배)')
        else:
            print(f'🟢 여유가 있다. 권장 guide_speed = {max(D * 1.5, 0.06):.3f} '
                  f'~ 0.10 (normal_speed 보다 낮게)')
        return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--steps', default=','.join('%.2f' % s for s in DEFAULT_STEPS))
    ap.add_argument('--hold', type=float, default=6.0, help='계단 유지 [s]')
    ap.add_argument('--settle', type=float, default=1.5, help='계단 앞 버리는 시간 [s]')
    ap.add_argument('--dry', action='store_true')
    a = ap.parse_args()
    try:
        steps = parse_steps(a.steps)
    except UsageError as exc:
        print(f'입력 오류 — {exc}', file=sys.stderr)
        return 2

    rclpy.init()
    node = LinearSweep(steps, a.hold, a.settle, a.dry)
    try:
        rc = node.run()
    except KeyboardInterrupt:
        for _ in range(10):
            node._send(0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
        print('\n중단 — 정지 명령을 보냈다')
        rc = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
