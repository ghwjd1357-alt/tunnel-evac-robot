#!/usr/bin/env python3
"""`applied_pwm_max` 가 **시행마다** 다시 세는지 생산 코드로 확인한다 (§65.2·§66.2).

무엇을 증명하나
---------------
`applied_pwm_max` 는 부팅 이후 최대였다. 그래서 한 번 160 을 찍으면 그 뒤 모든 시행이
영원히 160 으로 보인다 — "이번 주행에서 포화했나"를 물을 수 없다. 보완판은 무장
전이(`!ARMED -> ARMED`)에서만 최댓값을 0 으로 되돌리고 `applied_pwm_epoch` 를 하나
올린다. epoch 는 **무장 횟수**지 폴링 횟수가 아니다.

어떻게 확인하나 (검토 §66.2 완료판정)
  실제 `rearm_gate.h` 상태기계로 무장 절차를 두 번 밟고, `.ino` 에서 떼어온
  `updateAppliedPwmEpoch()` · `updateMotorOutputs()` 원문을 그대로 돌린다.
  ① 1회차 최대 160 -> 해제 -> 2회차 최대 **80** (160 이 안 남는다)
  ② epoch 는 정확히 **2** — 무장 두 번 = 2 이지 폴링 수가 아니다
  ③ 시행 안에서는 단조증가 (한 번 찍은 포화가 뒤에 지워지지 않는다)
  ④ 음수 PWM 은 크기로 센다
  ⑤ 매 tick `appliedPwm` 배열 — 🔴 **정본이 '부모판과 같은 궤적' 이라고 말하므로
     봉인이 거기까지 닿아야 한다** (검토 §67.3)
  ⑥ 🔴 **변이 주입 자가검사** — 검토 §66.2 가 뚫은 `== -> !=` 와 §67.3 이 뚫은
     `PWM_RAMP_STEP 2 -> 3` 을 포함해 **네 변이**가 전부 이 시험을 깨야 한다.

왜 파이썬 복제 모형을 버렸나
----------------------------
초판은 `PwmObserver` 라는 파이썬 사본을 돌렸다. 검토 §66.2 가 생산 코드의 ARMED
조건을 `==` 에서 `!=` 로 뒤집었는데 전량 통과했다 — 돌아간 것은 사본이었기 때문이다.
`rearm_gate.h` 무수정 검사도 `git diff HEAD` 로 봐서, **커밋된 뒤에는 항상 비어**
영원히 초록인 검사였다. 지금은 내용 sha256 으로 못 박는다.

사용
----
    python3 tools/test_applied_pwm_epoch.py
    echo $?      # 0 = 통과 / 1 = 계약 위반 / 2 = 판정 불능(컴파일러·추출)

정본 = docs/REAL_ROBOT_VALUES.md §1-b-2 · 검토현황 §65.2 · §66.2.
"""

import hashlib
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ino_host_probe as probe  # noqa: E402

ROOT = probe.ROOT
SKETCH = probe.SKETCH_DIR

#: 🔴 게이트 헤더 내용 고정. `git diff HEAD` 는 커밋 뒤 항상 비어 무의미했다(§66.2).
#:   이 값이 바뀌면 host harness 재검증 없이는 이 시험이 통과하면 안 된다.
GATE_SHA256 = {
    "rearm_gate.h":
        "abff1f7b292f766540854b3c6a8493525f5494f3ff177dd290ed74a3aa77eea3",
    "drive_wiring.h":
        "f34ba116fbd94a317362754dd1fc846a39ca76a387cd9d1e7a9d43783e08b860",
}

FAILURES = []


def check(label, condition, detail=""):
    if condition:
        print("  \033[32mOK\033[0m  " + label)
    else:
        print("  \033[31mNG\033[0m  " + label + ("\n      " + detail if detail else ""))
        FAILURES.append(label)


# ── shim — 게이트·PWM 출력의 바깥쪽만 흉내낸다 ───────────────────────────────
SHIM = """
#include "rearm_gate.h"
#include "drive_wiring.h"

int movePwmTowardTarget(int, int);
void updateAppliedPwmEpoch();
void updateMotorOutputs();
bool isEstopActive();
uint32_t millis();
void writeMotorPwm(int, int);

struct FakeDriveSink {
  int stopCalls = 0;
  bool cmdVelReceived = false;
  uint32_t lastCmdVelMs = 0;
  void stopAllMotors();
  void setCmdVelReceived(bool value) { cmdVelReceived = value; }
  void noteCommandAccepted(uint32_t nowMs)
  {
    lastCmdVelMs = nowMs;
    cmdVelReceived = true;
  }
};

RearmGate driveGate;
FakeDriveSink driveSink;

int desiredPwm[4] = {0, 0, 0, 0};
int appliedPwm[4] = {0, 0, 0, 0};
int appliedPwmMaxMagnitude = 0;
uint32_t appliedPwmEpoch = 0;
uint8_t appliedPwmEpochPrevState = 0;
double targetWheelVelocity[4] = {0.0, 0.0, 0.0, 0.0};
uint32_t startBoostUntilMs[4] = {0, 0, 0, 0};
uint32_t lastPwmUpdateMs = 0;

static uint32_t g_nowMs = 0;
static bool g_estop = false;
static int g_lastWritten[4] = {0, 0, 0, 0};

uint32_t millis() { return g_nowMs; }
bool isEstopActive() { return g_estop; }
void writeMotorPwm(int motor, int pwm) { g_lastWritten[motor] = pwm; }
void FakeDriveSink::stopAllMotors()
{
  ++stopCalls;
  for (int motor = 0; motor < 4; ++motor) {
    appliedPwm[motor] = 0;
    g_lastWritten[motor] = 0;
  }
}

/* 매 루프 한 칸. .ino loop() 의 순서를 그대로 지킨다 —
   rearmGateTick -> updateAppliedPwmEpoch -> updateMotorOutputs. */
static void tickOnce(uint32_t stepMs)
{
  g_nowMs += stepMs;
  rearmGateTick(&driveGate, g_nowMs);
  updateAppliedPwmEpoch();
  updateMotorOutputs();
}

/* 무장 절차 = 실제 게이트 API 로만. 상태를 손으로 대입하지 않는다. */
static void armThroughGate()
{
  for (int i = 0; i < 4; ++i) {          /* zero-hold 충족 -> READY */
    rearmGateOnCommand(&driveGate, 0.0, 0.0, g_nowMs);
    tickOnce(200);
  }
  rearmGateOnService(&driveGate, true, g_estop);   /* -> ARMING */
  rearmGateArmBarrierStart(&driveGate, g_nowMs);   /* -> PENDING */
  for (int i = 0; i < 8; ++i) {                    /* quiet 장벽 완주 -> ARMED */
    tickOnce(100);
  }
}

/* 🔴 검토 §67.3 — 매 tick 의 appliedPwm 배열을 찍는다.
   앞 판은 시행별 **최대**와 epoch 만 봉인했다. 검토가 PWM_RAMP_STEP 2->3 을 주입하자
   과도 궤적이 달라졌는데도 endpoint 가 같아 통과했다. 정본과 .ino 주석은 부모판과
   '같은 desiredPwm/appliedPwm 궤적' 이라고 말하므로, 봉인이 그 문장까지 닿아야 한다. */
static void trace(int step)
{
  printf("T %d %d %d %d %d\\n", step,
         appliedPwm[0], appliedPwm[1], appliedPwm[2], appliedPwm[3]);
}

/* 목표 PWM 을 주고 램프가 다 오를 때까지 돌린다. */
static int g_traceStep = 0;

static void driveTo(int pwm, int loops)
{
  for (int motor = 0; motor < 4; ++motor) {
    targetWheelVelocity[motor] = 0.10;
    desiredPwm[motor] = pwm;
  }
  for (int i = 0; i < loops; ++i) {
    rearmGateOnCommand(&driveGate, 0.10, 0.0, g_nowMs);
    tickOnce(20);
    trace(g_traceStep++);          /* 🔴 §67.3 — 과도 궤적 전량 */
  }
}

static void report(const char* tag)
{
  printf("%s state=%d max=%d epoch=%lu applied=%d\\n",
         tag, (int)driveGate.state, appliedPwmMaxMagnitude,
         (unsigned long)appliedPwmEpoch, appliedPwm[0]);
}

"""

MAIN = """
int main()
{
  rearmGateInit(&driveGate);
  report("boot");

  /* ── 1회차: 160 까지 포화 ─────────────────────────────────────────── */
  armThroughGate();
  report("trial1-armed");
  driveTo(160, 120);
  report("trial1-peak");

  /* 시행 안에서 목표를 낮춰도 최댓값은 안 내려간다 */
  driveTo(40, 120);
  report("trial1-after-drop");

  /* ── 해제 ─────────────────────────────────────────────────────────── */
  driveDisarm(&driveGate, driveSink);
  for (int i = 0; i < 5; ++i) tickOnce(100);
  report("disarmed");

  /* ── 2회차: 80 까지만 ─────────────────────────────────────────────── */
  armThroughGate();
  report("trial2-armed");
  driveTo(80, 120);
  report("trial2-peak");

  /* ── ARMED 를 유지하면 epoch 는 안 올라간다 ───────────────────────── */
  for (int i = 0; i < 50; ++i) tickOnce(20);
  report("trial2-held");

  /* ── 후진(음수 PWM)도 크기로 센다 ─────────────────────────────────── */
  driveDisarm(&driveGate, driveSink);
  for (int i = 0; i < 5; ++i) tickOnce(100);
  armThroughGate();
  for (int motor = 0; motor < 4; ++motor) {
    targetWheelVelocity[motor] = -0.10;
    desiredPwm[motor] = -120;
  }
  for (int i = 0; i < 120; ++i) {
    rearmGateOnCommand(&driveGate, -0.10, 0.0, g_nowMs);
    tickOnce(20);
  }
  report("trial3-reverse");
  return 0;
}
"""

FUNCTIONS = ["movePwmTowardTarget", "updateAppliedPwmEpoch", "updateMotorOutputs"]


def run(source, tag="epoch"):
    """생산 코드를 컴파일해 각 지점의 관측값을 dict 로 돌려준다."""
    pieces = [probe.constants(source), SHIM]
    pieces += [probe.function(source, name) for name in FUNCTIONS]
    pieces.append(MAIN)

    out = probe.compile_and_run(pieces, "%s.cpp" % tag, include_sketch=True)
    marks, track = {}, []
    for line in out.strip().split("\n"):
        parts = line.split()
        if parts[0] == "T":                     # 🔴 §67.3 — 매 tick applied 배열
            track.append(tuple(int(v) for v in parts[2:]))
            continue
        marks[parts[0]] = {
            key: int(value)
            for key, value in (piece.split("=") for piece in parts[1:])
        }
    marks["_track"] = track
    return marks


# ── 구조 검사 — 배선은 텍스트로만 볼 수 있다 (약한 증거인 줄 알고 쓴다) ───────
def structural_checks(source):
    for name, expected in GATE_SHA256.items():
        path = os.path.join(SKETCH, name)
        with open(path, "rb") as handle:
            actual = hashlib.sha256(handle.read()).hexdigest()
        check("게이트 헤더 %s 내용이 host 계약 시점 그대로다" % name,
              actual == expected,
              "sha256 %s\n      기대   %s\n"
              "      🔴 헤더를 고쳤으면 rearm_gate_host_test 재실행 뒤 이 값을 갱신한다"
              % (actual, expected))

    loop_body = re.search(r"void\s+loop\(\)\s*\{(.*?)\n\}", source, re.S)
    if loop_body is None:
        check("loop() 를 찾았다", False)
        return
    body = loop_body.group(1)
    epoch_at = body.find("updateAppliedPwmEpoch();")
    outputs_at = body.find("updateMotorOutputs();")
    check("loop() 가 updateMotorOutputs() **앞에서** updateAppliedPwmEpoch() 를 부른다",
          0 <= epoch_at < outputs_at,
          "뒤에서 부르면 리셋이 그 루프의 관측을 지운다 (epoch=%d, outputs=%d)"
          % (epoch_at, outputs_at))

    check("/firmware/info 가 applied_pwm_epoch 를 발행한다",
          "applied_pwm_epoch=%lu" in source,
          "epoch 를 못 읽으면 현장에서 시행 경계를 확인할 방법이 없다")

    check("`시행 시작·끝의 차이` 안내가 정본에서 빠졌다",
          "시행 시작·끝의 차이" not in source,
          "epoch 가 생긴 뒤에는 그 안내가 틀린 사용법이다")


# ── 변이 주입 자가검사 ───────────────────────────────────────────────────────
ARMED_CONDITION = ("  if (nowState == static_cast<uint8_t>(DRIVE_ARMED) &&\n"
                   "      appliedPwmEpochPrevState != "
                   "static_cast<uint8_t>(DRIVE_ARMED)) {")

MUTATIONS = [
    ("ARMED 비교를 == 에서 != 로 뒤집는다 (검토 §66.2 가 통과시킨 변이)",
     ARMED_CONDITION,
     ARMED_CONDITION.replace("nowState == static_cast",
                             "nowState != static_cast")),
    ("무장 전이의 최댓값 리셋을 지운다",
     "    appliedPwmMaxMagnitude = 0;\n    ++appliedPwmEpoch;",
     "    ++appliedPwmEpoch;"),
    ("epoch 증가를 지운다",
     "    appliedPwmMaxMagnitude = 0;\n    ++appliedPwmEpoch;",
     "    appliedPwmMaxMagnitude = 0;"),
    # 🔴 검토 §67.3 이 뚫은 변이 — 과도 궤적만 바꾸고 endpoint 는 그대로 두는 변이다.
    #    최대·epoch 만 보면 안 잡히고, 매 tick 배열을 견줘야 잡힌다.
    ("PWM_RAMP_STEP 을 2 에서 3 으로 (과도 궤적만 바뀐다 · 검토 §67.3)",
     "static const int PWM_RAMP_STEP = 2;",
     "static const int PWM_RAMP_STEP = 3;"),
]


def contract(marks, baseline_track=None):
    """계약을 한 곳에서 판정한다 — 변이 검사도 같은 잣대를 쓴다.

    `baseline_track` 을 주면 **매 tick appliedPwm 배열**까지 견준다 (검토 §67.3).
    """
    violations = []
    if baseline_track is not None:
        got = marks.get("_track") or []
        if got != baseline_track:
            first = next((i for i, (a, b) in enumerate(zip(got, baseline_track))
                          if a != b), min(len(got), len(baseline_track)))
            violations.append(
                "applied 궤적이 기준과 다르다 (step %d: %s vs %s · 길이 %d vs %d)"
                % (first,
                   got[first] if first < len(got) else "-",
                   baseline_track[first] if first < len(baseline_track) else "-",
                   len(got), len(baseline_track)))
    if marks["trial1-peak"]["max"] != 160:
        violations.append("1회차 최대 %d (160 이어야)" % marks["trial1-peak"]["max"])
    if marks["trial1-after-drop"]["max"] != 160:
        violations.append("시행 중 단조증가 깨짐 %d"
                          % marks["trial1-after-drop"]["max"])
    if marks["trial2-peak"]["max"] != 80:
        violations.append("2회차 최대 %d (80 이어야 — 160 이면 시행 분리 실패)"
                          % marks["trial2-peak"]["max"])
    if marks["trial2-held"]["epoch"] != 2:
        violations.append("무장 두 번 뒤 epoch %d (2 여야 — 폴링 수가 아니다)"
                          % marks["trial2-held"]["epoch"])
    if marks["trial3-reverse"]["max"] != 120:
        violations.append("음수 PWM 크기 관측 %d (120 이어야)"
                          % marks["trial3-reverse"]["max"])
    if marks["trial3-reverse"]["epoch"] != 3:
        violations.append("세 번째 무장 뒤 epoch %d"
                          % marks["trial3-reverse"]["epoch"])
    return violations


def main():
    print("생산 코드 실행 회귀 — applied_pwm_max 가 시행마다 다시 세는가 (§65.2·§66.2)")
    print("  받침대 = tools/ino_host_probe.py (`.ino` + 실제 rearm_gate.h 를 g++ 로)")

    try:
        source = probe.load()
        print("\n[1] 실제 게이트로 무장 -> 주행 -> 해제 -> 재무장을 두 번 밟는다")
        marks = run(source)
    except probe.ProbeError as error:
        print("\n\033[31m판정 불능\033[0m — %s" % error)
        return 2

    for tag in ("trial1-peak", "trial1-after-drop", "disarmed",
                "trial2-peak", "trial2-held", "trial3-reverse"):
        print("      %-18s state=%d max=%3d epoch=%d"
              % (tag, marks[tag]["state"], marks[tag]["max"], marks[tag]["epoch"]))

    baseline = marks.get("_track") or []
    print("      매 tick applied 궤적 %d 스텝 확보 (검토 §67.3)" % len(baseline))
    violations = contract(marks)
    check("① ~ ④ 시행 경계 계약", not violations, "\n      ".join(violations))

    print("\n[2] 구조 검사 (약한 증거 — 배선은 텍스트로만 보인다)")
    structural_checks(source)

    print("\n[3] 변이 주입 자가검사")
    for label, old, new in MUTATIONS:
        try:
            mutant = run(probe.mutate(source, old, new), "mutant")
        except probe.ProbeError as error:
            print("\n\033[31m판정 불능\033[0m — 변이 컴파일 실패: %s" % error)
            return 2
        caught = bool(contract(mutant, baseline))
        check("⑤ %s -> 계약이 깨진다" % label, caught,
              "변이를 넣었는데 계약이 통과하면 이 시험은 생산 코드를 안 돌린 것이다")
        if caught:
            print("      잡은 위반: %s" % contract(mutant, baseline)[0])

    print()
    if FAILURES:
        print("\033[31m%d 건 위반\033[0m" % len(FAILURES))
        return 1
    print("\033[32m전량 통과\033[0m — 시행 경계가 서 있고, 무너뜨리면 이 시험이 잡는다")
    return 0


if __name__ == "__main__":
    sys.exit(main())
