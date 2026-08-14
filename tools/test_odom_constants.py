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
  ③ 회전 **일관성 screen** — 🔴 각 시행의 원자료가 요구하는 `r/base` 를 계산해 서로
     일관된지 본다. **정량 판정이 아니다**(검토 §70.1 — 예산 ±1.5° 가 미실측이다).
     R2 회전의 최종 권위는 **굽기 뒤 지면 2π** 다. 상수는 베끼지 않는다(§68.1)
  ④ 🔴 역회귀 — 기각쌍 `(0.04603, 0.670)` 과 하중 반지름 `0.0451` 은 **직진에서**
     반드시 잡힌다. ⚠ 회전은 **이 둘을 못 가른다** — 두 비의 차이(`0.046%`)가 가장 좁은
     시행 예산(`0.115%`)보다 작아 원리적으로 판별력이 없다. 그래서 판별은 거리가 한다
     (앞 판 32-e 완료판정의 *"둘 다 벗어남"* 은 불가능했고, "회전은 보존된다" 는 단언도
     느슨한 밴드에 기댄 것이었다)
  ⑤ 🔴 감도 — `base` 를 ±1%·±3% 흔들면 ③ 이 깨져야 한다
  ⑥ 🔴 **fixture 무결성** — 주근거 행의 네 원자료 필드를 ±2% 흔들면 **그 행이** 실패해야
     한다. 앞 판은 `min×0.996~max×1.004` 라 원자료가 오염될수록 밴드가 **넓어져** 계속
     통과했다(검토 §69.1 이 2134 의 odom 을 200° 로 바꿔 재현). 이제 시행별 독립 예산이다
  ⑦ 보조 시행(264°)을 빼도 후보가 성립하는가 — 짧은 시행에 기대고 있지 않음을 보인다

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
#: 각 행 = (이름, 기록 시점 r, 기록 시점 base, odom Δyaw°, 독립 기준 Δyaw°, 주근거인가)
#:   기준은 IMU 다 — 바퀴 상수와 무관한 관측이라 이 표에서 유일하게 외부 사실이다.
#:
#: 🔴 `primary=False` 인 이유 (검토 §69.2 답변) — 끝점 각도 오차는 회전각과 **무관하게**
#:   거의 일정하다. 그래서 264° 시행의 상대오차가 1305° 시행보다 **약 5 배** 크다.
#:   짧은 시행을 긴 시행과 같은 정보량으로 취급하면 잡음을 근거로 삼는 것이다.
#:   → 보조 교차관측으로 인쇄하되 **판정에는 안 쓴다.** 다만 완전히 버리지도 않는다 —
#:     느슨한 상한(주근거 예산의 SPIN_AUX_SLACK 배)을 벗어나면 그건 오기다.
SPIN = (
    ("r2_spin2pi_0813_1640", 0.05698, 0.62,   475.39,  355.53, True),   # 오후 · 옛 펌웨어
    ("r2_spin_0813_2134",    0.04603, 0.670,  263.60,  265.91, False),  # 264° · 보조
    ("r2_spin_0813_2130",    0.04603, 0.670, 1303.60, 1305.20, True),   # 3.6 바퀴
)

#: 🔴 끝점 각도 오차 예산 (검토 §69.1). **밴드를 표본 극값으로 만들지 않는다.**
#:   앞 판은 `min×0.996 ~ max×1.004` 였다. 그러면 원자료 한 행이 오염될수록 밴드가
#:   **넓어져** 후보가 계속 통과한다 — 검토가 2134 의 odom 을 263.60 -> 200.00 으로
#:   바꾸자 상한이 0.0917 까지 벌어졌는데도 PASS 였다. 밴드가 자기 오염을 흡수한 것이다.
#:   → 이제 **시행마다 독립 예산**으로 본다. 한 행이 틀리면 **그 행이 실패**하고, 다른
#:     행의 허용폭에는 영향이 없다.
#:
#: ⚠ **08-14 정정 (검토 §70.1) — 이 검사는 정량 판정이 아니라 `일관성 screen` 이다.**
#:   앞 판은 IMU 몫 ±1.0° 를 "BNO055 **융합 모드** 전형값" 이라고 적었다. 🔴 틀렸다 —
#:   생산 펌웨어는 `OPERATION_MODE_AMG` 로 BNO055 를 켜고(융합 없음 · `REAL_ROBOT_VALUES`
#:   §3), 정지 500 표본 bias 를 뺀 gyroZ 를 dt 로 **직접 적분**해 `/imu/yaw_deg` 를 만든다.
#:   융합 heading 의 전형 오차는 이 reference 에 적용할 수 없다.
#:   그리고 `2130` 잔차가 0.994° 라 1.5° 는 통과시키지만 0.99° 아래 예산은 실패시킨다 —
#:   **실측 없이 고른 숫자가 결과를 가르는 자리에 있었다.**
#:
#: → 그래서 **주장을 낮춘다.** 이 절이 하는 일은 *"세 시행이 서로 일관된 r/base 를
#:   요구하는가"* 이지 *"후보가 옳음을 정량으로 증명"* 이 아니다.
#:   🔴 **R2 회전의 최종 권위는 굽기 뒤 지면 2π** 다(합격선 324°~396°).
#:   ⚠ 남는 노출의 크기: 세 시행이 각각 요구하는 base 는 0.82218~0.82902 로 최대 **0.83%**
#:     흩어진다. R2 합격선은 **±10%** 라 그 8% 만 먹는다 — 이 예산이 틀려도 R2 판정은
#:     안 뒤집힌다. 그게 이 screen 을 굽기 전에 닫지 않아도 되는 이유다.
#:   **재개방** — post-burn 2π 가 324~396 을 벗어나면 AMG 적분 경로의 오차를 실측해
#:     예산을 재산정한다. 새 실측 없이 이 숫자를 다시 고르지 않는다.
ENDPOINT_YAW_ERR_DEG = 1.5
#: 보조 시행이 이 배수를 넘으면 예산으로도 설명이 안 된다 — 오기로 본다.
SPIN_AUX_SLACK = 3.0

#: 🔴 기각된 쌍과 하중 반지름. 이 값들이 **직진에서** 걸려야 한다 (역회귀).
#:   ⚠ 기각쌍은 **회전에서는 통과하는 것이 정상**이다 — 비를 보존하도록 만든 값이라
#:   같은 카운트를 옛 쌍으로 환산해도 yaw 는 거의 그대로다. 앞 판의 32-e 완료판정은
#:   *"둘 다 벗어나야"* 라고 적었는데 그건 물리적으로 불가능하다(검토 §68.1).
REJECTED = (
    ("32-d 기각쌍 (0.04603 / 0.670)", 0.04603, 0.670),
    # 🔴 base 를 **비가 정확히 보존되게** 잡았다(0.0451 / 0.068733). 그래야 이 역회귀가
    #   말하는 바가 분명해진다 — *비를 보존해도 거리에서 잡힌다.*
    ("하중 반지름을 구름 반지름으로 (축 높이 0.0451)", 0.0451, 0.65616),
)


def spin_required_ratio(record_r, record_base, odom_deg, reference_deg):
    """한 회전 시행이 **요구하는** `r/base` 를 원자료에서 계산한다.

    odom Δyaw 는 `(r/base) × 엔코더 카운트차` 에 비례한다. 그 시행의 카운트는 이미
    일어난 사실이므로, 기록 시점 비에 `기준/odom` 을 곱하면 "그 카운트로 기준 각을
    내려면 비가 얼마여야 했나" 가 나온다. **상수를 하나도 베끼지 않는다.**
    """
    return (record_r / record_base) * (reference_deg / odom_deg)


def spin_tolerance(reference_deg):
    """그 시행의 허용 상대오차. 🔴 회전각이 길수록 좁아진다 (검토 §69.1)."""
    return ENDPOINT_YAW_ERR_DEG / abs(reference_deg)


def spin_verdicts(candidate_ratio):
    """시행별로 **독립 판정**한다. 한 행의 오염이 다른 행을 넓히지 못한다.

    반환 = `[(이름, 요구비, 허용, 실제 상대오차, 주근거인가, 통과인가), ...]`
    """
    out = []
    for name, rec_r, rec_b, odom_deg, ref_deg, primary in SPIN:
        need = spin_required_ratio(rec_r, rec_b, odom_deg, ref_deg)
        tol = spin_tolerance(ref_deg)
        rel = abs(candidate_ratio - need) / need
        limit = tol if primary else tol * SPIN_AUX_SLACK
        out.append((name, need, tol, rel, primary, rel <= limit))
    return out


def spin_violations(candidate_ratio):
    """판정에 쓰는 위반 목록. 보조 시행은 느슨한 상한만 본다."""
    bad = []
    for name, need, tol, rel, primary, ok in spin_verdicts(candidate_ratio):
        if ok:
            continue
        limit = tol if primary else tol * SPIN_AUX_SLACK
        bad.append("%s %s 요구 %.6f · 실제오차 %.2f%% > 허용 %.2f%%"
                   % (name, "주" if primary else "보조", need,
                      rel * 100, limit * 100))
    return bad


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

    # ── ③ 회전 비 — 🔴 시행별 독립 오차 예산 (검토 §69.1) ───────────────────
    print("\n[3] 회전 **일관성 screen** — 시행별 독립 예산 (끝점 ±%.1f° · 🔴 미실측)"
          % ENDPOINT_YAW_ERR_DEG)
    print("      ⚠ 정량 판정이 아니다 (검토 §70.1). R2 회전의 최종 권위는 "
          "굽기 뒤 지면 2π 다")
    ratio = radius / base
    for name, need, tol, rel, primary, ok in spin_verdicts(ratio):
        limit = tol if primary else tol * SPIN_AUX_SLACK
        print("      %-22s %s 요구 %.6f · 허용 ±%.2f%% · 실제 %.2f%%  %s"
              % (name, "주  " if primary else "보조", need, limit * 100, rel * 100,
                 "OK" if ok else "🔴"))
    print("      `.ino` r/base = %.6f" % ratio)
    bad_spin = spin_violations(ratio)
    check("③ 주근거 회전 시행이 서로 일관된 r/base 를 요구한다 (screen)",
          not bad_spin, " · ".join(bad_spin))
    # 🔴 오후 1640 은 완전히 다른 상수로 기록됐다. 그 시행이 요구하는 비가 후보와
    #    만나는 것이 이 상수쌍의 **독립 확인**이다 — 밤 시행들과 같은 근거가 아니다.
    afternoon = spin_required_ratio(*SPIN[0][1:5])
    print("      🔴 오후 1640 은 옛 상수(r/base=%.6f)로 기록됐는데 요구비 %.6f 가"
          % (SPIN[0][1] / SPIN[0][2], afternoon))
    print("         후보 %.6f 와 %.3f%% 안에서 만난다 — 독립 확인이다"
          % (ratio, abs(afternoon - ratio) / ratio * 100))
    aux = [v for v in spin_verdicts(ratio) if not v[4]]
    for name, need, tol, rel, _p, _ok in aux:
        print("      ⚠ %s 는 264° 짜리라 상대오차가 %.1f 배 크다 — 요구 %.6f 는 후보와"
              % (name, tol / spin_tolerance(SPIN[2][4]), need))
        print("         %.2f%% 갈리지만 **판정에 안 쓴다**(보조 교차관측 · 검토 §69.1)"
              % (rel * 100))

    # ── ④ 역회귀 ───────────────────────────────────────────────────────────
    print("\n[4] 역회귀 — 틀린 값은 **직진에서** 잡혀야 한다")
    print("      ⚠ 기각쌍은 **회전에서는 통과하는 것이 정상**이다 — 비를 보존하도록")
    print("        만든 값이라 같은 카운트를 옛 쌍으로 환산해도 yaw 는 그대로다.")
    for label, bad_r, bad_base in REJECTED:
        bad_straight = straight_violations(bad_r)
        check("④ %s -> **직진**이 깨진다" % label,
              bool(bad_straight),
              "이 값이 직진을 통과하면 이 시험은 상수를 검증하지 못하는 것이다")
        if bad_straight:
            print("      직진 위반: %s" % bad_straight[0])
        # 🔴 "회전이 통과한다/실패한다" 가 아니라 **"회전은 이 둘을 못 가른다"** 가 참인
        #   문장이다. 두 비의 차이가 어느 시행의 예산보다도 작으면, 회전 관측은 원리적으로
        #   판별력이 없다 — 옛 쌍이 회전에서 아슬아슬하게 통과하든 말든 의미가 없다.
        #   (앞 판은 느슨한 밴드를 근거로 "회전은 보존된다" 고 단언했는데, 예산을 조이자
        #    2130 에서 0.124% vs 0.115% 로 갈렸다. 그 0.009%p 를 판정으로 읽으면 안 된다.)
        gap = abs(ratio - bad_r / bad_base) / ratio
        tightest = min(spin_tolerance(r[4]) for r in SPIN if r[5])
        check("④ %s -> 회전은 이 둘을 **못 가른다**(그래서 거리가 판별한다)"
              % label.split(" (")[0],
              gap < tightest,
              "회전이 가를 수 있다면 이 역회귀의 서술을 다시 써야 한다 "
              "(차이 %.3f%% vs 최소 예산 %.3f%%)" % (gap * 100, tightest * 100))
        print("      두 비의 차이 %.3f%% · 가장 좁은 예산 %.3f%% -> 회전은 판별력 없음"
              % (gap * 100, tightest * 100))

    # ── ⑤ 감도 — 상수와 원자료 **양쪽**을 흔든다 (검토 §69.1) ────────────────
    print("\n[5] 감도 — 상수를 흔들면 ③ 이 깨져야 한다")
    for factor in (1.01, 0.99, 1.03, 0.97):
        shaken = radius / (base * factor)
        check("⑤ base ×%.2f -> ③ 이 깨진다" % factor,
              bool(spin_violations(shaken)),
              "예산이 너무 넓어 base 를 사실상 검증하지 않는다")

    print("\n[6] 🔴 fixture 무결성 — 원자료 한 행이 오염되면 **그 행이** 실패해야 한다")
    print("      (앞 판은 min/max 밴드라 오염될수록 넓어져 계속 통과했다 — 검토 §69.1)")
    saved = SPIN
    try:
        for index, row in enumerate(saved):
            if not row[5]:
                continue                      # 보조 행은 판정에 안 쓴다
            for field, name in ((1, "record_r"), (2, "record_base"),
                                (3, "odom_deg"), (4, "reference_deg")):
                for factor in (1.02, 0.98):
                    mutated = list(saved)
                    shaken = list(row)
                    shaken[field] = row[field] * factor
                    mutated[index] = tuple(shaken)
                    globals()["SPIN"] = tuple(mutated)
                    caught = bool(spin_violations(ratio))
                    check("⑥ %s.%s ×%.2f -> 실패" % (row[0][-4:], name, factor),
                          caught,
                          "원자료 오염을 시험이 흡수했다 — §69.1 이 지적한 바로 그 구조다")
    finally:
        globals()["SPIN"] = saved

    print("\n[7] 보조 시행을 빼도 후보가 성립하는가 (검토 §69.1 필수 회귀)")
    saved = SPIN
    try:
        globals()["SPIN"] = tuple(r for r in saved if r[5])
        check("⑦ 짧은 2134 를 빼도 후보가 통과한다",
              not spin_violations(ratio),
              "주근거 둘만으로 후보가 안 서면 보조에 기대고 있던 것이다")
    finally:
        globals()["SPIN"] = saved

    print()
    if FAILURES:
        print("\033[31m%d 건 위반\033[0m" % len(FAILURES))
        return 1
    print("\033[32m전량 통과\033[0m — 상수가 지면 실측을 재현하고, 틀린 값은 잡힌다")
    return 0


if __name__ == "__main__":
    sys.exit(main())
