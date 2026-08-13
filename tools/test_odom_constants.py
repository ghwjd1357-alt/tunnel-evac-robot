#!/usr/bin/env python3
"""odom 상수쌍을 **지면 실측**으로 검증한다 (예약 32-e · PITFALLS §12).

왜 이 시험이 있나
-----------------
2026-08-13 하루가 **줄자 한 번 잘못 읽은 것** 위에 서 있었다. 오후 직진 시행의 줄자가
`3105 mm` 로 적혔는데 그 명령·그 시간이면 로봇은 `3.84 m` 를 간다. 그 한 값이
`odom/줄자 = 1.238` 을 만들었고, 거기서 `ODOM_WHEEL_RADIUS = 0.04603` 이 나왔고, 그 위에
펌웨어·정본·검토 3 회차가 쌓였다. 🔴 **검토는 P0 0 · P1 0 을 냈고 그 판정도 옳았다** —
구현은 문서가 말한 것을 정확히 했다. 틀린 것은 문서가 말한 **내용**이었고, 그 근거는
저장소 밖의 실측이라 어떤 정적 검토도 못 본다.

→ 그래서 **실측을 저장소 안으로 가져온다.** 이 파일의 `GROUND` 표는 08-13 밤에 지면에서
잰 값이고, `.ino` 의 상수가 그 실측을 재현하는지 매번 확인한다. 상수를 바꾸면 이 시험이
**깨지는 것이 정상**이고, 깨진 것을 고치려면 **새 실측**을 가져와야 한다.

무엇을 검사하나
--------------
  ① C10 — 바퀴 회전수로 직접 구한 구름 반지름이 `.ino` 의 값을 감싸는가
  ② 직진 재현 — 세 시행 전부 `odom/줄자` 가 R2 합격선 `0.97~1.03` 안인가
  ③ 회전 비 — `r/base` 가 회전 두 시행이 확정한 비와 같은가
  ④ 🔴 역회귀 — 기각된 쌍 `(0.04603, 0.670)` 을 넣으면 ② 가 **반드시 실패**해야 한다
  ⑤ 🔴 역회귀 — **하중 반지름**(축 높이 `0.0451`)을 구름 반지름으로 쓰면 ② 가 반드시
     실패해야 한다. `PITFALLS §12` 의 "반지름 셋을 섞지 않는다" 를 코드로 고정한다

왜 상수를 `.ino` 에서 읽나
-------------------------
숫자를 이 파일에 적어 두면 펌웨어가 바뀐 뒤에도 옛 계약이 초록으로 남는다 (검토 §65.3 이
`test_drive_checks` 를 "자기확인" 이라 부른 자리). 실측(`GROUND`)만 여기 두고, 상수는
매번 `.ino` 에서 가져온다 — **비교 대상 둘 중 하나는 반드시 저장소 밖에서 온 사실**이다.

사용
----
    python3 tools/test_odom_constants.py
    echo $?      # 0 = 통과 / 1 = 계약 위반 / 2 = 판정 불능

정본 = docs/MASTER_PLAN.md §7 예약 32-e · docs/REAL_ROBOT_VALUES.md §1-b-0 · PITFALLS §12.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from firmware_constants import firmware_double  # noqa: E402

#: R2 완료조건 — `odom/줄자` 가 이 안에 들어야 한다.
R2_BAND = (0.97, 1.03)

#: 🔴 08-13 밤 지면 실측. **저장소 밖에서 온 사실**이다 — 상수에서 유도하지 않았다.
#:   `odom_mm` 은 그 시행 때 보드가 실제로 발행한 값이고, `rec_radius` 는 그때 보드에
#:   구워져 있던 반지름이다. 다른 반지름으로 환산하려면 비례식 하나면 된다.
GROUND = (
    # (이름,                 줄자 mm, odom mm, 기록 시점 반지름, 바퀴 회전수)
    ("r2_line_0813_2119", 3065.0, 2473.8, 0.04603, None),
    ("r2_roll_0813_2143", 3900.0, 3161.6, 0.04603, 11),
)

#: 회전 두 시행이 확정한 비. odom Δyaw ∝ r/base 라 회전은 **비만** 정한다.
#:   r2_spin_0813_2134 : odom 263.6° vs IMU 265.9°  -> 0.991
#:   r2_spin_0813_2130 : odom 1303.6° vs IMU 1305.2° -> 0.999 (3.6 바퀴)
SPIN_RATIO = 0.04603 / 0.670
SPIN_RATIO_TOL = 0.005          # 회전 실측이 0.9%·0.1% 로 갈렸으므로 그만큼 연다

#: 🔴 기각된 쌍과 하중 반지름. 이 값들이 통과하면 안 된다 (역회귀).
REJECTED = (
    ("32-d 기각쌍 (0.04603 / 0.670)", 0.04603, 0.670),
    ("하중 반지름을 구름 반지름으로 (축 높이 0.0451)", 0.0451, 0.0451 / SPIN_RATIO),
)

FAILURES = []


def check(label, ok, detail=""):
    if ok:
        print("  \033[32mOK\033[0m  " + label)
    else:
        print("  \033[31mNG\033[0m  " + label + ("\n      " + detail if detail else ""))
        FAILURES.append(label)


def odom_over_tape(radius, run):
    """그 시행의 odom 을 `radius` 로 환산해 줄자와 견준다."""
    _, tape, odom, rec_radius, _ = run
    return (odom * radius / rec_radius) / tape


def straight_violations(radius):
    """R2 합격선을 벗어난 시행 목록. 빈 목록이면 통과."""
    bad = []
    for run in GROUND:
        ratio = odom_over_tape(radius, run)
        if not R2_BAND[0] <= ratio <= R2_BAND[1]:
            bad.append("%s %.4f" % (run[0], ratio))
    return bad


def main():
    print("odom 상수쌍 대 지면 실측 (예약 32-e · PITFALLS §12)")
    try:
        radius = firmware_double("ODOM_WHEEL_RADIUS")
        base = firmware_double("ODOM_WHEEL_BASE")
    except (KeyError, OSError) as error:
        print("\n\033[31m판정 불능\033[0m — `.ino` 에서 상수를 못 읽었다: %s" % error)
        return 2
    print("  `.ino` ODOM_WHEEL_RADIUS=%.5f · ODOM_WHEEL_BASE=%.3f" % (radius, base))

    # ── ① C10 — 회전수로 직접 구한 구름 반지름 ─────────────────────────────
    print("\n[1] C10 — 바퀴 회전수로 구한 구름 반지름 (엔코더를 안 거친다)")
    rolled = [r for r in GROUND if r[4]]
    if not rolled:
        check("C10 실측이 표에 있다", False, "회전수를 센 시행이 하나도 없다")
    for name, tape, _odom, _rec, revs in rolled:
        # 표시 바퀴가 선회 안쪽/바깥쪽 중 어디였는지 모르므로 밴드로 받는다.
        centre = tape / (revs * 2.0 * math.pi) / 1000.0
        band = (centre * 0.985, centre * 1.015)
        print("      %s : %d 회전 · %.0f mm -> 중심선 %.2f mm (밴드 %.2f~%.2f)"
              % (name, revs, tape, centre * 1000, band[0] * 1000, band[1] * 1000))
        check("① C10 밴드가 `.ino` 의 %.5f 를 감싼다" % radius,
              band[0] <= radius <= band[1],
              "구름 반지름은 하중 반지름(축 높이)이 아니다 — PITFALLS §12")

    # ── ② 직진 재현 ────────────────────────────────────────────────────────
    print("\n[2] 직진 — odom/줄자 가 R2 합격선 %.2f~%.2f 안인가" % R2_BAND)
    for run in GROUND:
        ratio = odom_over_tape(radius, run)
        print("      %-20s 줄자 %.0f mm -> odom/줄자 %.4f" % (run[0], run[1], ratio))
    bad = straight_violations(radius)
    check("② 세 시행 전부 합격선 안", not bad, " · ".join(bad))

    # ── ③ 회전 비 ──────────────────────────────────────────────────────────
    print("\n[3] 회전 — r/base 가 회전 두 시행이 확정한 비와 같은가")
    ratio = radius / base
    print("      `.ino` r/base = %.6f   회전 실측이 확정한 비 = %.6f"
          % (ratio, SPIN_RATIO))
    check("③ r/base 가 회전 실측의 비와 일치 (±%.1f%%)" % (SPIN_RATIO_TOL * 100),
          abs(ratio - SPIN_RATIO) / SPIN_RATIO <= SPIN_RATIO_TOL,
          "회전은 **비만** 정한다 — r 을 바꾸면 base 도 같이 옮겨야 한다")

    # ── ④⑤ 역회귀 ─────────────────────────────────────────────────────────
    print("\n[4] 역회귀 — 틀린 값을 넣으면 반드시 실패해야 한다")
    for label, bad_r, bad_base in REJECTED:
        caught_straight = bool(straight_violations(bad_r))
        caught_spin = abs(bad_r / bad_base - SPIN_RATIO) / SPIN_RATIO > SPIN_RATIO_TOL
        check("④ %s -> 계약이 깨진다" % label,
              caught_straight or caught_spin,
              "이 값이 통과하면 이 시험은 상수를 검증하지 못하는 것이다")
        if caught_straight:
            print("      직진 위반: %s" % straight_violations(bad_r)[0])

    print()
    if FAILURES:
        print("\033[31m%d 건 위반\033[0m" % len(FAILURES))
        return 1
    print("\033[32m전량 통과\033[0m — 상수가 지면 실측을 재현하고, 틀린 값은 잡힌다")
    return 0


if __name__ == "__main__":
    sys.exit(main())
