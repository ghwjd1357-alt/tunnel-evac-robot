#!/usr/bin/env python3
"""반지름 재교정이 **제어를 안 바꿨는지** 를 산술로 증명한다 (검토 §65.1 역회귀).

무엇을 증명하나
---------------
08-13 의 odom 재교정은 `ODOM_WHEEL_RADIUS` 를 0.05698 -> 0.04603 으로 줄였다.
부모판에서 이 상수는 odom 만 만드는 것이 아니라

    DISTANCE_PER_COUNT -> measuredWheelVelocity -> filteredWheelVelocity
                       -> measuredAlongCommand  (= PI 오차의 측정항)

경로로 **제어기가 보는 속도**까지 만들었다. 그래서 반지름만 줄이면 제어기는 로봇이
0.808 배로 느려진 줄 알고 더 밟는다. 검토 §65.1 이 이 자리를 짚었고, 그건 예약 33
(FF·게인 조정)을 이 묶음에서 몰래 여는 일이다.

보완판은 PI 에 들어가기 직전 `CONTROL_FEEDBACK_SCALE` 을 곱해 옛 눈금을 복원한다.
이 시험은 그 복원이 **정확한지** 를 본다:

  ① 제어 경로 : 같은 엔코더 카운트열에 대해 보완판의 measuredAlongCommand 가
                부모판(구 반지름 하나로 돌던 코드)의 값과 같아야 한다.
                -> 같으면 desiredPwm/appliedPwm 궤적도 같다. 제어는 안 바뀌었다.
  ② odom 경로 : 같은 카운트열에 대해 보완판의 이동거리가 부모판의 정확히
                0.04603/0.05698 = 0.80783 배여야 한다. -> odom 만 정직해졌다.

왜 EMA 를 그대로 돌려보나
------------------------
"선형이니까 당연히 같다"는 말은 증명이 아니다. 지수이동평균은 상태를 들고 있어서,
상태를 안 옮기고 입력만 배율하면 과도구간에서 값이 다를 수 있다. 여기서는 상태가
0 에서 시작하고 곱셈이 선형이라 같지만, 그건 **돌려봐야** 아는 것이다.

왜 상수를 `.ino` 에서 읽나
-------------------------
숫자를 이 파일에 적어 두면 `.ino` 가 바뀐 뒤에도 옛 계약이 초록으로 남는다.
검토 §65.3 이 `test_drive_checks` 를 "구 상수를 정답으로 고정한 자기확인" 이라고
부른 것과 같은 함정이다. 그래서 매번 `.ino` 에서 파싱한다.

사용
----
    python3 tools/test_control_feedback_scale.py
    echo $?      # 0 = 통과

정본 = docs/MASTER_PLAN.md §7 예약 32-d · 검토현황 §65.1.
"""

import math
import os
import re
import sys

INO = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..",
    "firmware",
    "teensy_integrated_base_v1_4",
    "teensy_integrated_base_v1_4.ino",
)

FAILURES = []


def constant(source, name):
    """`static const double NAME = 값;` 하나를 .ino 에서 읽는다."""
    match = re.search(
        r"static\s+const\s+double\s+" + re.escape(name) + r"\s*=\s*([0-9.eE+-]+)\s*;",
        source)
    if not match:
        print("  \033[31mNG\033[0m  .ino 에서 %s 를 못 읽었다 — 이름이 바뀌었다" % name)
        sys.exit(1)
    return float(match.group(1))


def check(label, condition, detail=""):
    if condition:
        print("  \033[32mOK\033[0m  " + label)
    else:
        print("  \033[31mNG\033[0m  " + label + ("\n      " + detail if detail else ""))
        FAILURES.append(label)


def run_chain(counts, distance_per_count, alpha, dt, output_scale):
    """`.ino` 의 updateOdometry 속도 사슬을 그대로 흉내낸다.

    반환 = (필터 출력열, 누적 이동거리). output_scale 은 PI 에 넣기 직전 곱셈이다.
    """
    filtered = 0.0
    outputs = []
    travelled = 0.0
    for delta_count in counts:
        delta = delta_count * distance_per_count
        travelled += delta
        measured = delta / dt
        filtered += alpha * (measured - filtered)
        outputs.append(filtered * output_scale)
    return outputs, travelled


def main():
    with open(INO, encoding="utf-8") as handle:
        source = read = handle.read()

    odom_radius = constant(source, "ODOM_WHEEL_RADIUS")
    control_radius = constant(source, "CONTROL_WHEEL_RADIUS")
    total_ppr = constant(source, "TOTAL_PPR")
    alpha = constant(source, "VELOCITY_FILTER_ALPHA")

    print("── 제어 피드백 불변 검사 (검토 §65.1) ────────────────────────")
    print("  ODOM_WHEEL_RADIUS    = %.5f  (odom 발행용 — 재교정된 값)" % odom_radius)
    print("  CONTROL_WHEEL_RADIUS = %.5f  (PI 피드백용 — 판재 이전 눈금)" % control_radius)
    print("  TOTAL_PPR            = %.1f" % total_ppr)
    print("  VELOCITY_FILTER_ALPHA= %.2f" % alpha)

    # 코드가 CONTROL_FEEDBACK_SCALE 을 실제로 이 비로 정의했는지부터 본다.
    scale_decl = re.search(
        r"CONTROL_FEEDBACK_SCALE\s*=\s*CONTROL_WHEEL_RADIUS\s*/\s*ODOM_WHEEL_RADIUS",
        source)
    check("CONTROL_FEEDBACK_SCALE = CONTROL_WHEEL_RADIUS / ODOM_WHEEL_RADIUS",
          scale_decl is not None,
          "이 비가 아니면 아래 산술 증명이 코드와 다른 것을 증명한다")

    # PI 오차의 측정항이 그 배율을 실제로 쓰는지 본다. 상수만 있고 안 곱하면 무의미하다.
    uses_scale = re.search(
        r"measuredAlongCommand\s*=\s*\n?\s*direction\s*\*\s*filteredWheelVelocity\[motor\]"
        r"\s*\*\s*CONTROL_FEEDBACK_SCALE",
        source)
    check("measuredAlongCommand 가 CONTROL_FEEDBACK_SCALE 을 곱한다",
          uses_scale is not None,
          "상수만 선언하고 안 곱하면 제어는 여전히 새 눈금으로 돈다")

    # odom 은 그 배율을 쓰면 안 된다 — 쓰면 재교정이 취소된다.
    odom_lines = re.findall(r"^.*DISTANCE_PER_COUNT.*$", source, re.M)
    check("odom 거리 환산에는 CONTROL_FEEDBACK_SCALE 이 안 섞였다",
          all("CONTROL_FEEDBACK_SCALE" not in line for line in odom_lines))

    scale = control_radius / odom_radius
    parent_dpc = (2.0 * math.pi * control_radius) / total_ppr
    child_dpc = (2.0 * math.pi * odom_radius) / total_ppr

    # 실제 시행을 닮은 카운트열: 정지 -> 가속 -> 순항 -> 감속 -> 정지.
    # 한 자리 수부터 포화 근처까지 폭을 넓게 잡아 과도구간을 일부러 포함시킨다.
    dt = 0.02
    counts = ([0] * 10 + list(range(0, 40, 3)) + [39] * 200 +
              list(range(39, -1, -3)) + [0] * 10 + [-39] * 50)

    parent_control, parent_travel = run_chain(counts, parent_dpc, alpha, dt, 1.0)
    child_control, child_travel = run_chain(counts, child_dpc, alpha, dt, scale)

    worst = 0.0
    for a, b in zip(parent_control, child_control):
        denominator = max(abs(a), 1e-12)
        worst = max(worst, abs(a - b) / denominator)

    check("① 제어 입력이 부모판과 같다 (상대오차 최악 %.3e)" % worst,
          worst < 1e-12,
          "이 값이 크면 PI 오차가 달라진다 = desiredPwm 궤적이 달라진다")

    travel_ratio = child_travel / parent_travel if parent_travel else float("nan")
    expected_ratio = odom_radius / control_radius
    check("② odom 이동거리가 부모판의 %.5f 배다 (실측 %.5f)"
          % (expected_ratio, travel_ratio),
          abs(travel_ratio - expected_ratio) < 1e-12,
          "이 비가 1 이면 재교정이 안 먹은 것이다")

    # 실측 근거와의 대조: 08-13 직진 시행은 odom/줄자 = 1.238 이었다.
    # 새 상수로 그 시행을 다시 환산하면 1 에 붙어야 한다.
    measured_odom_over_tape = 3842.6 / 3105.0
    corrected = measured_odom_over_tape * (odom_radius / control_radius)
    check("③ r2_line_0813_1516 을 새 상수로 재환산하면 odom/줄자 = %.5f" % corrected,
          abs(corrected - 1.0) < 0.005,
          "0.5% 밖이면 상수가 그 시행을 설명하지 못한다")

    print("──────────────────────────────────────────────────────────────")
    if FAILURES:
        print("  실패 %d 건" % len(FAILURES))
        return 1
    print("  전량 통과 — 반지름 재교정은 odom 에만 들어갔다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
