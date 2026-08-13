#!/usr/bin/env python3
"""반지름 재교정이 **제어를 안 바꿨는지** 를 생산 코드로 증명한다 (검토 §65.1·§66.2).

무엇을 증명하나
---------------
08-13 의 odom 재교정은 굴림 반지름을 0.05698 -> 0.04603 으로 줄였다. 부모판에서
이 상수 하나가

    DISTANCE_PER_COUNT -> measuredWheelVelocity -> filteredWheelVelocity
                       -> measuredAlongCommand  (= PI 오차의 측정항)

경로로 **제어기가 보는 속도**까지 만들었다. 그래서 반지름만 줄이면 제어기는 로봇이
0.808 배로 느려진 줄 알고 더 밟는다 — odom 만 고치려던 묶음이 게인 조정(예약 33)을
몰래 여는 것이다. 보완판은 PI 직전에 `CONTROL_FEEDBACK_SCALE` 을 곱해 옛 눈금을
복원한다.

세 세대를 **같은 엔코더 카운트열**로 나란히 돌린다:

  부모판 `a1268dc` — 반지름 0.05698 하나. 제어 튜닝이 측정된 눈금이다.
  결함판 `6aca792` — 반지름만 0.04603 으로. 검토 §65.1 이 짚은 그 상태.
  현행판 HEAD      — 상수 분리 + CONTROL_FEEDBACK_SCALE 복원.

검사 (검토 §66.2 완료판정)
  ① 현행판의 `desiredPwm` 궤적이 부모판과 **정수까지 완전히 같다**.
  ② 결함판의 궤적은 부모판과 **달라야 한다** — 이 시험이 §65.1 결함을 실제로 본다는
     증거다. 이게 없으면 "아무 차이도 못 보는 시험"과 구별이 안 된다.
  ③ 직진 카운트열에서 현행판 odom 이동거리가 부모판의 0.04603/0.05698 배다.
  ④ 🔴 **변이 주입 자가검사** — 검토 §66.2 가 뚫은 변이(`+ 0.010`)와 배율 제거를
     현행 원문에 넣으면 ① 이 반드시 깨진다.

왜 파이썬으로 다시 짜지 않나
----------------------------
초판(§65 보완)은 산술을 파이썬 복제 모형으로 돌렸다. 검토 §66.2 가 제어식에
`+ 0.010` 을 더해도 전량 통과시켰다 — 구조 검사는 정규식으로 식의 **접두부만** 봤고,
숫자는 복제본에서 나왔기 때문이다. §65.3 에서 고친 "상수를 베껴 자기확인" 과 같은
병이 로직에서 재발한 것이다. 그래서 이제 `.ino` 원문을 떼어다 g++ 로 컴파일한다
(받침대 = `tools/ino_host_probe.py`).

사용
----
    python3 tools/test_control_feedback_scale.py
    echo $?      # 0 = 통과 / 1 = 계약 위반 / 2 = 판정 불능(컴파일러·추출)

정본 = docs/MASTER_PLAN.md §7 예약 32-d · 검토현황 §65.1 · §66.2.
"""

import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ino_host_probe as probe  # noqa: E402

#: 제어 튜닝이 측정된 눈금을 들고 있던 마지막 커밋.
PARENT_REF = "a1268dc"
#: 반지름만 바꿔 제어까지 같이 옮겼던 커밋 (검토 §65.1 이 짚은 상태).
DEFECT_REF = "6aca792"

FAILURES = []


def check(label, condition, detail=""):
    if condition:
        print("  \033[32mOK\033[0m  " + label)
    else:
        print("  \033[31mNG\033[0m  " + label + ("\n      " + detail if detail else ""))
        FAILURES.append(label)


# ── 시나리오 ────────────────────────────────────────────────────────────────
# 정지 -> 가속 -> 순항 -> 좌우 비대칭(회전) -> 감속 -> 후진. 과도구간을 일부러 넣는다.
# 두 세대에 **같은 배열**을 넣어야 비교가 성립하므로 여기서 한 번만 만든다.
def scenario():
    steps = []
    for index in range(300):
        if index < 10:
            target, delta = 0.0, 0
        elif index < 60:                    # 가속
            target = 0.10 * (index - 10) / 50.0
            delta = int(round(target * 0.8 * 2641.1 * 0.02 / (2 * math.pi * 0.05698)))
        elif index < 150:                   # 순항
            target, delta = 0.10, 24
        elif index < 210:                   # 회전 — 좌우가 다르다
            target, delta = 0.10, 24
        elif index < 260:                   # 감속
            target = 0.10 * (260 - index) / 50.0
            delta = int(round(target * 0.9 * 2641.1 * 0.02 / (2 * math.pi * 0.05698)))
        else:                               # 후진
            target, delta = -0.06, -15
        # 회전 구간에서만 좌우를 벌린다(FL,RL = 왼쪽 / FR,RR = 오른쪽).
        turning = 150 <= index < 210
        left_target = target * (0.5 if turning else 1.0)
        left_delta = int(delta * (0.5 if turning else 1.0))
        steps.append((left_target, target, left_delta, delta))
    return steps


STEPS = scenario()


def scenario_arrays():
    """C 배열 리터럴로 굳힌다. 두 세대가 글자 그대로 같은 입력을 받는다."""
    targets_left = ", ".join("%.9f" % s[0] for s in STEPS)
    targets_right = ", ".join("%.9f" % s[1] for s in STEPS)
    delta_left = ", ".join(str(s[2]) for s in STEPS)
    delta_right = ", ".join(str(s[3]) for s in STEPS)
    return """
static const int STEP_COUNT = %d;
static const double TARGET_L[STEP_COUNT] = {%s};
static const double TARGET_R[STEP_COUNT] = {%s};
static const int DELTA_L[STEP_COUNT] = {%s};
static const int DELTA_R[STEP_COUNT] = {%s};
""" % (len(STEPS), targets_left, targets_right, delta_left, delta_right)


# ── shim — Arduino·micro-ROS 쪽만 흉내낸다. 산술은 하나도 안 짠다 ─────────────
SHIM = """
double clampDouble(double, double, double);
double normalizeAngle(double);
int wheelVelocityToFeedforwardPwm(int, double);
void resetWheelControllerState(int);
void updateWheelControllers(double);
void updateOdometry();
void publishOdometry(uint64_t);
uint64_t getMonotonicTimestampNs();
bool isEstopActive();
void stopAllMotors();
long readEncoderCount(int);
uint32_t micros();

double targetWheelVelocity[4] = {0.0, 0.0, 0.0, 0.0};
double measuredWheelVelocity[4] = {0.0, 0.0, 0.0, 0.0};
double filteredWheelVelocity[4] = {0.0, 0.0, 0.0, 0.0};
double wheelIntegralError[4] = {0.0, 0.0, 0.0, 0.0};
double wheelPreviousError[4] = {0.0, 0.0, 0.0, 0.0};
double wheelFilteredDerivative[4] = {0.0, 0.0, 0.0, 0.0};
int desiredPwm[4] = {0, 0, 0, 0};
bool cmdVelReceived = false;
long previousCount[4] = {0, 0, 0, 0};
uint32_t previousOdomUs = 0;
double odomX = 0.0, odomY = 0.0, odomYaw = 0.0;
double odomLinearVelocity = 0.0, odomAngularVelocity = 0.0;

static long g_count[4] = {0, 0, 0, 0};
static uint32_t g_nowUs = 0;
static int g_stopCalls = 0;

long readEncoderCount(int motor) { return g_count[motor]; }
uint32_t micros() { return g_nowUs; }
bool isEstopActive() { return false; }
uint64_t getMonotonicTimestampNs() { return 0; }
void publishOdometry(uint64_t) {}
/* 예기치 않은 정지도 궤적에 남아야 한다 — 조용히 삼키면 차이를 못 본다. */
void stopAllMotors()
{
  ++g_stopCalls;
  for (int motor = 0; motor < 4; ++motor) desiredPwm[motor] = 0;
}
"""

MAIN = """
int main()
{
  cmdVelReceived = true;
  previousOdomUs = 0;

  for (int step = 0; step < STEP_COUNT; ++step) {
    targetWheelVelocity[FL] = TARGET_L[step];
    targetWheelVelocity[RL] = TARGET_L[step];
    targetWheelVelocity[FR] = TARGET_R[step];
    targetWheelVelocity[RR] = TARGET_R[step];

    g_count[FL] += DELTA_L[step];
    g_count[RL] += DELTA_L[step];
    g_count[FR] += DELTA_R[step];
    g_count[RR] += DELTA_R[step];

    g_nowUs += 20000;               /* ODOM_PERIOD_US 와 같게 — 매 회 통과 */
    updateOdometry();

    printf("%d %d %d %d %d %.15e %.15e\\n",
           desiredPwm[0], desiredPwm[1], desiredPwm[2], desiredPwm[3],
           g_stopCalls, odomX, odomYaw);
  }
  return 0;
}
"""

STRAIGHT_MAIN = MAIN.replace("TARGET_R[step]", "TARGET_L[step]") \
                    .replace("DELTA_R[step]", "DELTA_L[step]")

#: `.ino` 에서 원문 그대로 떼어올 조각들. 산술은 전부 여기 들어 있다.
FUNCTIONS = ["clampDouble", "normalizeAngle", "resetWheelControllerState",
             "wheelVelocityToFeedforwardPwm", "updateWheelControllers",
             "updateOdometry"]


def run(source, main_source, tag):
    """한 세대의 생산 코드를 컴파일해 궤적을 돌려준다."""
    pieces = [probe.enum(source, "MotorIndex"),
              probe.constants(source),
              SHIM,
              scenario_arrays()]
    pieces += [probe.function(source, name) for name in FUNCTIONS]
    pieces.append(main_source)

    rows = []
    for line in probe.compile_and_run(pieces, "%s.cpp" % tag).strip().split("\n"):
        parts = line.split()
        rows.append((tuple(int(v) for v in parts[:5]),
                     float(parts[5]), float(parts[6])))
    return rows


def pwm_track(rows):
    return [row[0] for row in rows]


def main():
    print("생산 코드 실행 회귀 — 반지름 분리가 제어를 안 바꿨는가 (§65.1·§66.2)")
    print("  받침대 = tools/ino_host_probe.py (`.ino` 원문을 g++ 로 컴파일)")

    try:
        head_source = probe.load()
        parent_source = probe.load(PARENT_REF)
        defect_source = probe.load(DEFECT_REF)

        print("\n[1] 세 세대를 같은 엔코더 카운트열(%d 스텝)로 돌린다" % len(STEPS))
        head = run(head_source, MAIN, "head")
        parent = run(parent_source, MAIN, "parent")
        defect = run(defect_source, MAIN, "defect")
    except probe.ProbeError as error:
        print("\n\033[31m판정 불능\033[0m — %s" % error)
        return 2

    # ① 현행판 == 부모판. desiredPwm 은 정수라 근사가 아니라 완전 일치여야 한다.
    same = pwm_track(head) == pwm_track(parent)
    first_diff = next(
        (i for i, (a, b) in enumerate(zip(pwm_track(head), pwm_track(parent)))
         if a != b), None)
    check("① desiredPwm 궤적이 부모판 %s 와 정수까지 완전히 같다" % PARENT_REF,
          same,
          "" if same else "첫 불일치 step=%s  현행=%s  부모=%s"
          % (first_diff, pwm_track(head)[first_diff], pwm_track(parent)[first_diff]))

    # ② 결함판은 달라야 한다 — 시험의 감도 증거.
    defect_diff = next(
        (i for i, (a, b) in enumerate(zip(pwm_track(defect), pwm_track(parent)))
         if a != b), None)
    check("② 결함판 %s 의 궤적은 부모판과 다르다 (시험이 §65.1 을 실제로 본다)"
          % DEFECT_REF,
          defect_diff is not None,
          "결함판과 부모판이 같게 나오면 이 시험은 아무것도 못 보는 것이다")
    if defect_diff is not None:
        print("      결함판 첫 갈림 step=%d  결함=%s  부모=%s"
              % (defect_diff, pwm_track(defect)[defect_diff],
                 pwm_track(parent)[defect_diff]))

    # ③ odom 은 반지름 비만큼 줄어야 한다. 직진에서 봐야 윤거가 안 섞인다.
    try:
        head_straight = run(head_source, STRAIGHT_MAIN, "head_s")
        parent_straight = run(parent_source, STRAIGHT_MAIN, "parent_s")
    except probe.ProbeError as error:
        print("\n\033[31m판정 불능\033[0m — %s" % error)
        return 2

    expected = (probe_double(head_source, "ODOM_WHEEL_RADIUS") /
                probe_double(parent_source, "WHEEL_RADIUS"))
    measured = head_straight[-1][1] / parent_straight[-1][1]
    check("③ 직진 odom 이동거리 비 = 반지름 비 %.5f" % expected,
          abs(measured - expected) < 1e-12,
          "실측 %.12f" % measured)
    print("      부모 %.6f m -> 현행 %.6f m  (비 %.9f)"
          % (parent_straight[-1][1], head_straight[-1][1], measured))

    # ④ 변이 주입 자가검사 — 검토 §66.2 가 뚫은 두 변이를 이 시험이 잡는가.
    print("\n[2] 변이 주입 자가검사 (검토 §66.2 가 통과시킨 변이)")
    anchor = ("        direction * filteredWheelVelocity[motor] * "
              "CONTROL_FEEDBACK_SCALE;")
    mutations = [
        ("제어식에 + 0.010 을 더한다", anchor,
         anchor[:-1] + " + 0.010;"),
        ("CONTROL_FEEDBACK_SCALE 곱셈을 지운다", anchor,
         "        direction * filteredWheelVelocity[motor];"),
    ]
    for label, old, new in mutations:
        try:
            mutated = run(probe.mutate(head_source, old, new), MAIN, "mutant")
            caught = pwm_track(mutated) != pwm_track(parent)
        except probe.ProbeError as error:
            print("\n\033[31m판정 불능\033[0m — 변이 컴파일 실패: %s" % error)
            return 2
        check("④ %s -> ① 이 깨진다" % label, caught,
              "변이를 넣었는데 궤적이 그대로면 이 시험은 생산 코드를 안 돌린 것이다")

    print()
    if FAILURES:
        print("\033[31m%d 건 위반\033[0m" % len(FAILURES))
        return 1
    print("\033[32m전량 통과\033[0m — 반지름 분리는 제어를 안 바꿨고, "
          "이 시험은 바꿨으면 잡는다")
    return 0


def probe_double(source, name):
    """세대마다 이름이 다르므로 원문에서 직접 읽는다."""
    match = re.search(
        r"static\s+const\s+double\s+%s\s*=\s*([0-9][0-9.eE+-]*)\s*;" % name, source)
    if match is None:
        raise probe.ProbeError("상수 %s 를 못 찾았다" % name)
    return float(match.group(1))


if __name__ == "__main__":
    sys.exit(main())
