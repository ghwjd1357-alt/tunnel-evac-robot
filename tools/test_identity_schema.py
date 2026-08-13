#!/usr/bin/env python3
"""`/firmware/info` 정체 **스키마**가 조용히 줄어들지 않는지 본다 (검토 §68.2).

왜 이 시험이 있나
-----------------
`d0_check` 검사 7 은 기대 키를 손으로 들고 있었다. 08-13 밤에 `CONTROL_WHEEL_RADIUS`
를 지우자(예약 32-e) 목록이 `.ino` 와 어긋나 검사가 통째로 멈췄다. 그래서 format 에서
직접 뽑게 고쳤는데 — 🔴 **그러자 반대 구멍이 열렸다.** 필수 필드가 펌웨어와 format 에서
**함께** 사라지면 검사기가 줄어든 계약을 그대로 새 정답으로 받는다.

**값 사본을 없앤 것**과 **스키마를 안 드는 것**은 다른 문제다. 값은 계속 `.ino` 에서
읽되, *어떤 키가 있어야 하는가* 는 `REQUIRED_IDENTITY_KEYS` 가 소유한다.

무엇을 검사하나
--------------
  ① 현행 `.ino` 가 필수 키를 전부 싣는다
  ② 🔴 역회귀 — 필수 키를 format+인자에서 **함께** 지우면 반드시 거부한다
     (검토 §68.2 가 재현한 바로 그 경로다. format 만 지우면 짝이 안 맞아 따로 막힌다)
  ③ 비필수(진단) 필드 제거는 **통과**한다 — 계약을 넓히는 방향은 위험하지 않다
  ④ 스키마 축소는 정본 개정이므로, 이 목록이 줄면 정본도 같이 줄어야 한다

사용
----
    python3 tools/test_identity_schema.py
    echo $?      # 0 = 통과 / 1 = 계약 위반 / 2 = 판정 불능

정본 = docs/REAL_ROBOT_VALUES.md §1-b · docs/MASTER_PLAN.md §7 예약 32-e.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import firmware_constants as fc                      # noqa: E402
import firmware_info_length_check as fil             # noqa: E402

#: 필수 키를 지우려면 format 조각과 인자 **둘 다** 지워야 짝이 맞는다.
#: 하나만 지우면 변환지시자/인자 수가 어긋나 별도 경로로 막힌다 — 그것도 fail-closed 다.
REMOVALS = (
    ("odom_wheel_radius", "odom_wheel_radius=%.5f; ", "      ODOM_WHEEL_RADIUS,\n"),
    ("cmd_wheel_base", "cmd_wheel_base=%.3f; ", "      CMD_WHEEL_BASE,\n"),
    ("odom_wheel_base", "odom_wheel_base=%.3f; ", "      ODOM_WHEEL_BASE,\n"),
    ("kp", "kp=%.3f; ", "      WHEEL_KP,\n"),
    ("ki", "ki=%.3f; ", "      WHEEL_KI,\n"),
    ("kd", "kd=%.3f; ", "      WHEEL_KD,\n"),
)
#: 계약이 아닌 진단 필드. 사라져도 검사가 막으면 안 된다(과잉 방어).
OPTIONAL = ("min_speed", "min_speed=%.3f; ", "      MIN_EFFECTIVE_WHEEL_CMD,\n")

FAILURES = []


def check(label, ok, detail=""):
    if ok:
        print("  \033[32mOK\033[0m  " + label)
    else:
        print("  \033[31mNG\033[0m  " + label + ("\n      " + detail if detail else ""))
        FAILURES.append(label)


def with_source(mutate):
    """`.ino` 를 문자열만 바꿔 읽히게 한다. 🔴 저장소 파일은 안 건드린다."""
    original = fil.read_ino
    fil.read_ino = lambda: mutate(original())
    try:
        return fc.firmware_identity_keys(), None
    except Exception as error:                       # noqa: BLE001
        return None, error
    finally:
        fil.read_ino = original


def main():
    print("정체 스키마 회귀 (검토 §68.2)")
    print("  필수 키: %s" % ", ".join(fc.REQUIRED_IDENTITY_KEYS))

    keys, error = with_source(lambda s: s)
    if error is not None:
        print("\n\033[31m판정 불능\033[0m — 현행 `.ino` 를 못 읽는다: %s" % error)
        return 2

    print("\n[1] 현행 — 필수 키가 전부 실려 있는가")
    for want in fc.REQUIRED_IDENTITY_KEYS:
        check("① %s 가 실린다" % want,
              any(k.startswith(want + "=") for k in keys))

    print("\n[2] 🔴 역회귀 — 필수 키를 format+인자에서 함께 지우면 거부해야 한다")
    for name, fmt_bit, arg_bit in REMOVALS:
        got, err = with_source(
            lambda s, f=fmt_bit, a=arg_bit: s.replace(f, "").replace(a, ""))
        check("② %s 소실 -> 거부" % name, got is None and err is not None,
              "검사기가 줄어든 계약을 새 정답으로 받았다 — §68.2 가 재현한 경로다")

    print("\n[3] 비필수 진단 필드는 통과해야 한다 (과잉 방어 금지)")
    name, fmt_bit, arg_bit = OPTIONAL
    got, err = with_source(lambda s: s.replace(fmt_bit, "").replace(arg_bit, ""))
    check("③ %s 소실 -> 통과" % name, got is not None,
          "계약을 넓히는 방향까지 막으면 진단 필드를 못 뺀다: %s" % err)

    print("\n[4] 스키마를 줄이는 것은 **정본 개정**이다")
    canon = os.path.join(fc.ROOT if hasattr(fc, "ROOT") else ".",
                         "docs", "REAL_ROBOT_VALUES.md")
    try:
        with open(canon, encoding="utf-8") as handle:
            text = handle.read()
    except OSError:
        text = ""
    check("④ 정본이 필수 키 집합의 소유자를 밝힌다",
          "REQUIRED_IDENTITY_KEYS" in text,
          "정본에 근거가 없으면 이 목록은 코드가 혼자 정한 값이 된다")

    print()
    if FAILURES:
        print("\033[31m%d 건 위반\033[0m" % len(FAILURES))
        return 1
    print("\033[32m전량 통과\033[0m — 스키마가 조용히 줄지 않는다")
    return 0


if __name__ == "__main__":
    sys.exit(main())
