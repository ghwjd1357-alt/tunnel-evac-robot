#!/usr/bin/env python3
"""`applied_pwm_max` 가 **시행별** 최대인지 본다 (검토 §65.2 역회귀).

무엇이 문제였나
---------------
08-13 첫 판의 `appliedPwmMaxMagnitude` 는 부팅 시 0 에서 시작해 더 큰 값을 만날 때만
갱신되고 어디서도 리셋되지 않았다. 그래서 부팅 뒤 한 번 160 에 닿으면, 다음 시행이
80 이었는지 160 이었는지 **영원히 구분되지 않는다** — 둘 다 "시작=끝=160, 차이 0" 이다.
포화 관측 수단이 첫 고출력 시행에서 죽는다. 예약 33(FF·게인 조정)이 그 값을 근거로
쓰려던 것이므로, 그대로 두면 잘못된 튜닝 근거가 된다.

보완 = 무장 전이(!ARMED -> ARMED)마다 0 으로 리셋하고 `applied_pwm_epoch` 를 1 올린다.
`/firmware/info` 를 읽는 쪽은 **epoch 가 같은 표본끼리만** max 를 비교한다.

무엇을 보나
-----------
  구조 ① `rearm_gate.h` 를 안 건드렸다 — §64 에서 막 승인된 상태기계 계약이다.
  구조 ② 리셋이 ARMED **진입 순간**에만 일어난다 (ARMED 안에서 매 루프 지우면
         시행 중 포화가 사라진다).
  구조 ③ epoch 갱신이 `updateMotorOutputs()` **앞**에서 불린다 — 무장 직후 첫 PWM 이
         새 epoch 에 들어가야 한다.
  구조 ④ `/firmware/info` 가 epoch 를 실제로 내보낸다 — 안 내보내면 밖에서 시행 경계를
         못 긋고, 절대값 비교가 다시 의미를 잃는다.
  동작 ⑤ 첫 시행 최대 160, 둘째 시행 최대 80 -> 둘째가 **80** 으로 나와야 한다.
  동작 ⑥ 한 시행 안에서는 단조증가다 (160 뒤 80 을 봐도 160 을 유지).
  동작 ⑦ ARMED 를 안 거치면 epoch 가 안 는다 (READY·PENDING 왕복은 시행이 아니다).

사용
----
    python3 tools/test_applied_pwm_epoch.py
    echo $?      # 0 = 통과

정본 = docs/MASTER_PLAN.md §7 예약 33 · 검토현황 §65.2.
"""

import os
import re
import subprocess
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
FW = os.path.join(ROOT, "firmware", "teensy_integrated_base_v1_4")
INO = os.path.join(FW, "teensy_integrated_base_v1_4.ino")
GATE = os.path.join(FW, "rearm_gate.h")

DISARMED, READY, ARMED, PENDING, ARMING = 0, 1, 2, 3, 4

FAILURES = []


def check(label, condition, detail=""):
    if condition:
        print("  \033[32mOK\033[0m  " + label)
    else:
        print("  \033[31mNG\033[0m  " + label + ("\n      " + detail if detail else ""))
        FAILURES.append(label)


# ── 동작 모형 — `.ino` 의 updateAppliedPwmEpoch() 를 그대로 옮긴 것 ──────────
class PwmObserver:
    """무장 epoch 별 최대 PWM 관측기."""

    def __init__(self):
        self.max_magnitude = 0
        self.epoch = 0
        self.previous_state = DISARMED

    def observe_state(self, state):
        if state == ARMED and self.previous_state != ARMED:
            self.max_magnitude = 0
            self.epoch += 1
        self.previous_state = state

    def observe_pwm(self, applied):
        magnitude = abs(applied)
        if magnitude > self.max_magnitude:
            self.max_magnitude = magnitude


def structural_checks():
    with open(INO, encoding="utf-8") as handle:
        ino = handle.read()

    # ① 게이트 헤더는 이 묶음에서 안 바뀐다.
    changed = subprocess.run(
        ["git", "-C", ROOT, "diff", "--name-only", "HEAD", "--",
         "firmware/teensy_integrated_base_v1_4/rearm_gate.h"],
        capture_output=True, text=True).stdout.strip()
    check("① rearm_gate.h 를 안 건드렸다 (§64 상태기계 계약 유지)",
          changed == "",
          "게이트를 고치면 967+7 회귀의 의미가 달라진다")

    # ② 리셋은 진입 전이에서만. 조건에 '이전 상태가 ARMED 가 아니다' 가 있어야 한다.
    body = re.search(
        r"void\s+updateAppliedPwmEpoch\(\)\s*\{(.*?)\n\}", ino, re.S)
    check("② updateAppliedPwmEpoch() 가 존재한다", body is not None)
    if body:
        text = body.group(1)
        has_entry_guard = (
            "appliedPwmEpochPrevState !=" in text and "DRIVE_ARMED" in text)
        check("② 리셋이 ARMED **진입**에서만 일어난다",
              has_entry_guard and "appliedPwmMaxMagnitude = 0" in text,
              "이전 상태 비교가 없으면 ARMED 동안 매 루프 지워 포화가 사라진다")
        check("② epoch 가 같은 자리에서 증가한다", "++appliedPwmEpoch" in text)

    # ③ 호출 순서 — loop() 안에서 updateMotorOutputs() 보다 앞이어야 한다.
    loop_body = re.search(r"void\s+loop\(\)\s*\{(.*?)\n\}", ino, re.S)
    check("③ loop() 를 읽었다", loop_body is not None)
    if loop_body:
        text = loop_body.group(1)
        epoch_at = text.find("updateAppliedPwmEpoch();")
        motor_at = text.find("updateMotorOutputs();")
        check("③ epoch 갱신이 updateMotorOutputs() 보다 먼저 불린다",
              0 <= epoch_at < motor_at,
              "뒤에 있으면 무장 직후 첫 PWM 이 앞 시행 epoch 로 샌다")

    # ④ 밖으로 나가야 시행 경계를 그을 수 있다.
    check("④ /firmware/info 가 applied_pwm_epoch 를 발행한다",
          "applied_pwm_epoch=%lu" in ino and "appliedPwmEpoch" in ino)

    # 낡은 계약 문구가 남아 있으면 읽는 사람이 옛 방식으로 읽는다.
    check("④ '시행 시작·끝의 차이로 읽는다' 는 옛 안내가 지워졌다",
          "시행 시작·끝의 차이" not in ino,
          "그 문장은 부팅 이후 단조증가일 때의 읽는 법이다")


def behavioural_checks():
    # ⑤ 첫 시행 160, 둘째 시행 80.
    observer = PwmObserver()
    for state in (DISARMED, READY, ARMING, PENDING, ARMED):
        observer.observe_state(state)
    for pwm in (60, 120, 160, 140):
        observer.observe_pwm(pwm)
    first_max, first_epoch = observer.max_magnitude, observer.epoch

    for state in (DISARMED, READY, ARMING, PENDING, ARMED):
        observer.observe_state(state)
    for pwm in (40, 80, 70):
        observer.observe_pwm(pwm)
    second_max, second_epoch = observer.max_magnitude, observer.epoch

    check("⑤ 첫 시행 최대 = 160 (실측 %d)" % first_max, first_max == 160)
    check("⑤ 둘째 시행 최대 = 80 (실측 %d) — 부모판이면 160 이 남는다" % second_max,
          second_max == 80,
          "이 자리가 검토 §65.2 가 짚은 정보 손실 지점이다")
    check("⑤ epoch 가 시행마다 는다 (%d -> %d)" % (first_epoch, second_epoch),
          second_epoch == first_epoch + 1)

    # ⑥ 시행 안에서는 단조증가 — 큰 값 뒤 작은 값이 와도 안 내려간다.
    observer = PwmObserver()
    observer.observe_state(ARMED)
    for pwm in (160, 30, -20, 90):
        observer.observe_pwm(pwm)
    check("⑥ 시행 중에는 단조증가 (160 뒤 30 을 봐도 160)",
          observer.max_magnitude == 160)

    # 부호는 크기로 본다 — 후진 -160 도 포화다.
    observer = PwmObserver()
    observer.observe_state(ARMED)
    observer.observe_pwm(-160)
    check("⑥ 후진 -160 도 크기 160 으로 잡힌다", observer.max_magnitude == 160)

    # ⑦ ARMED 를 안 거치면 시행이 아니다.
    observer = PwmObserver()
    for state in (DISARMED, READY, ARMING, PENDING, DISARMED, READY, DISARMED):
        observer.observe_state(state)
    check("⑦ ARMED 없이 왕복하면 epoch 가 0 그대로 (실측 %d)" % observer.epoch,
          observer.epoch == 0,
          "READY·PENDING 은 시행이 아니다 — 모터가 안 돈다")

    # ARMED 를 유지하는 동안 재진입으로 세면 안 된다.
    observer = PwmObserver()
    for state in (ARMED, ARMED, ARMED, ARMED):
        observer.observe_state(state)
    check("⑦ ARMED 유지 중에는 epoch 가 안 는다 (실측 %d)" % observer.epoch,
          observer.epoch == 1)


def main():
    print("── applied_pwm_max 시행 경계 검사 (검토 §65.2) ────────────────")
    print("  [구조] .ino / rearm_gate.h")
    structural_checks()
    print("  [동작] 무장 epoch 모형")
    behavioural_checks()
    print("──────────────────────────────────────────────────────────────")
    if FAILURES:
        print("  실패 %d 건" % len(FAILURES))
        return 1
    print("  전량 통과 — max 는 이번 무장 안의 최대다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
