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
  ② 직진 재현 — `GROUND` 두 시행이 `odom/줄자` R2 합격선 `0.97~1.03` 안인가
  ③ 회전 비 — 🔴 **각 시행의 원자료가 요구하는 `r/base`** 를 계산해 밴드를 만들고,
     `.ino` 의 비가 그 안인가. **상수를 베끼지 않는다**(검토 §68.1)
  ④ 🔴 역회귀 — 기각쌍 `(0.04603, 0.670)` 과 하중 반지름 `0.0451` 은 **직진에서**
     반드시 잡힌다. ⚠ 기각쌍은 **회전에서는 통과하는 것이 정상**이다 — 비를 보존하도록
     만든 값이기 때문이다(앞 판 32-e 완료판정의 *"둘 다 벗어남"* 은 불가능했다)
  ⑤ 🔴 감도 — `base` 를 ±3% 흔들면 ③ 이 깨져야 한다. 밴드가 넓어 아무거나 통과하는
     상태가 아님을 시험이 스스로 보인다

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

#: 🔴 회전 원자료. **상수를 베끼지 않는다** (검토 §68.1).
#:   앞 판은 `SPIN_RATIO = 0.04603 / 0.670` 으로 **기각한 쌍을 그대로 정답으로 베꼈다.**
#:   후보 비 `0.05698/0.829 = 0.068733` 과 기각쌍 `0.068701` 은 0.046% 차이라 — 보존이
#:   설계 목표이므로 — 그 검사는 "내가 베낀 값을 보존했나" 만 봤다. `SPIN_RATIO` 자체가
#:   잘못 베껴졌는지는 원리적으로 못 잡는다. §65.3(도구가 상수를 베낌) · §66.2(시험이
#:   로직을 베낌) 에 이은 **같은 병의 세 번째**다.
#:
#: 각 행 = (이름, 기록 시점 r, 기록 시점 base, odom Δyaw°, 독립 기준 Δyaw°)
#:   기준은 IMU 다 — 바퀴 상수와 무관한 관측이라 이 표에서 유일하게 외부 사실이다.
SPIN = (
    ("r2_spin2pi_0813_1640", 0.05698, 0.62,   475.39,  355.53),   # 오후 · 옛 펌웨어
    ("r2_spin_0813_2134",    0.04603, 0.670,  263.60,  265.91),   # 밤 · 굽기 뒤
    ("r2_spin_0813_2130",    0.04603, 0.670, 1303.60, 1305.20),   # 밤 · 3.6 바퀴
)
#: 시행 간 흩어짐(0.83%)보다 조금 넓게. 밴드는 원자료가 만들고 상수가 안 만든다.
SPIN_BAND_MARGIN = 0.004

#: 🔴 기각된 쌍과 하중 반지름. 이 값들이 **직진에서** 걸려야 한다 (역회귀).
#:   ⚠ 기각쌍은 **회전에서는 통과하는 것이 정상**이다 — 비를 보존하도록 만든 값이라
#:   같은 카운트를 옛 쌍으로 환산해도 yaw 는 거의 그대로다. 앞 판의 32-e 완료판정은
#:   *"둘 다 벗어나야"* 라고 적었는데 그건 물리적으로 불가능하다(검토 §68.1).
REJECTED = (
    ("32-d 기각쌍 (0.04603 / 0.670)", 0.04603, 0.670, True),
    ("하중 반지름을 구름 반지름으로 (축 높이 0.0451)", 0.0451, 0.6543, True),
)


def spin_required_ratio(record_r, record_base, odom_deg, reference_deg):
    """한 회전 시행이 **요구하는** `r/base` 를 원자료에서 계산한다.

    odom Δyaw 는 `(r/base) × 엔코더 카운트차` 에 비례한다. 그 시행의 카운트는 이미
    일어난 사실이므로, 기록 시점 비에 `기준/odom` 을 곱하면 "그 카운트로 기준 각을
    내려면 비가 얼마여야 했나" 가 나온다. **상수를 하나도 베끼지 않는다.**
    """
    return (record_r / record_base) * (reference_deg / odom_deg)


def spin_band():
    """세 시행이 요구하는 비의 밴드. 🔴 밴드를 원자료가 만든다."""
    need = [spin_required_ratio(r, b, od, im) for _n, r, b, od, im in SPIN]
    return min(need) * (1.0 - SPIN_BAND_MARGIN), max(need) * (1.0 + SPIN_BAND_MARGIN)


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
        # 🔴 검토 §68.4 — 이 한 시행은 46.03 을 **강하게 기각**하지만 56.98 을 단독으로
        #   확정하지는 않는다. marker wheel 이 좌/우 어느 쪽이었는지 기록이 없어 밴드
        #   ±1.5% 도 기하가 아니라 손으로 고른 값이다.
        print("      ⚠ 이 한 시행은 46.03 을 기각하지만 56.98 을 **단독 확정하지 않는다**"
              " — 굽기 후보다 (검토 §68.4)")
        print("      ⚠ 같은 시행의 엔코더 10.93 회전은 **교차 관측**이지 독립 표본이"
              " 아니다. 바퀴별 C10 에서 marker wheel·좌우·정렬·yaw 를 기록한다")

    # ── ② 직진 재현 ────────────────────────────────────────────────────────
    print("\n[2] 직진 — odom/줄자 가 R2 합격선 %.2f~%.2f 안인가" % R2_BAND)
    for run in GROUND:
        ratio = odom_over_tape(radius, run)
        print("      %-20s 줄자 %.0f mm -> odom/줄자 %.4f" % (run[0], run[1], ratio))
    bad = straight_violations(radius)
    check("② `GROUND` 두 시행 전부 합격선 안", not bad, " · ".join(bad))

    # ── ③ 회전 비 — 🔴 원자료가 밴드를 만든다 (검토 §68.1) ──────────────────
    print("\n[3] 회전 — 각 시행의 원자료가 요구하는 r/base 를 계산한다")
    for name, rec_r, rec_b, odom_deg, ref_deg in SPIN:
        print("      %-22s 기록 r/base=%.6f · odom %8.2f° / 기준 %7.2f° -> 요구 %.6f"
              % (name, rec_r / rec_b, odom_deg, ref_deg,
                 spin_required_ratio(rec_r, rec_b, odom_deg, ref_deg)))
    lo, hi = spin_band()
    ratio = radius / base
    print("      원자료 밴드 %.6f ~ %.6f   ·   `.ino` r/base = %.6f"
          % (lo, hi, ratio))
    check("③ r/base 가 회전 **원자료** 밴드 안에 있다",
          lo <= ratio <= hi,
          "회전은 **비만** 정한다 — r 을 바꾸면 base 도 같이 옮겨야 한다")
    # 🔴 오후 1640 은 완전히 다른 상수로 기록됐다. 그 시행이 요구하는 비가 후보와
    #    만나는 것이 이 상수쌍의 **독립 확인**이다 — 밤 시행들과 같은 근거가 아니다.
    afternoon = spin_required_ratio(*SPIN[0][1:])
    print("      🔴 오후 1640 은 옛 상수(r/base=%.6f)로 기록됐는데 요구비 %.6f 가"
          % (SPIN[0][1] / SPIN[0][2], afternoon))
    print("         후보 %.6f 와 %.3f%% 안에서 만난다 — 독립 확인이다"
          % (ratio, abs(afternoon - ratio) / ratio * 100))

    # ── ④ 역회귀 ───────────────────────────────────────────────────────────
    print("\n[4] 역회귀 — 틀린 값은 **직진에서** 잡혀야 한다")
    print("      ⚠ 기각쌍은 **회전에서는 통과하는 것이 정상**이다 — 비를 보존하도록")
    print("        만든 값이라 같은 카운트를 옛 쌍으로 환산해도 yaw 는 그대로다.")
    for label, bad_r, bad_base, must_pass_spin in REJECTED:
        bad_straight = straight_violations(bad_r)
        spin_ok = lo <= (bad_r / bad_base) <= hi
        check("④ %s -> **직진**이 깨진다" % label,
              bool(bad_straight),
              "이 값이 직진을 통과하면 이 시험은 상수를 검증하지 못하는 것이다")
        if bad_straight:
            print("      직진 위반: %s" % bad_straight[0])
        if must_pass_spin:
            check("④ %s -> 회전은 보존된다(정상)" % label.split(" (")[0],
                  spin_ok,
                  "비를 보존한 쌍인데 회전이 깨지면 밴드나 원자료가 틀린 것이다")

    # ── ⑤ 밴드가 실제로 좁은가 — 시험 자신의 감도 ───────────────────────────
    print("\n[5] 감도 — base 를 흔들면 ③ 이 깨져야 한다")
    for factor, label in ((1.03, "base +3%"), (0.97, "base -3%")):
        shaken = radius / (base * factor)
        check("⑤ %s -> ③ 이 깨진다" % label,
              not (lo <= shaken <= hi),
              "밴드가 너무 넓어 base 를 사실상 검증하지 않는다")

    print()
    if FAILURES:
        print("\033[31m%d 건 위반\033[0m" % len(FAILURES))
        return 1
    print("\033[32m전량 통과\033[0m — 상수가 지면 실측을 재현하고, 틀린 값은 잡힌다")
    return 0


if __name__ == "__main__":
    sys.exit(main())
