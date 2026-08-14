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
  ⑤ 🔴 `build` 필드를 **세미콜론 경계까지** 본다 — missing·empty·stale·문법오류·
     **정상 prefix 뒤 garbage**·중복이 전부 `ng` 다 (검토 §71.2)

사용
----
    python3 tools/test_identity_schema.py
    echo $?      # 0 = 통과 / 1 = 계약 위반 / 2 = 판정 불능

정본 = docs/REAL_ROBOT_VALUES.md §1-b · docs/MASTER_PLAN.md §7 예약 32-e.
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import firmware_constants as fc                      # noqa: E402
import firmware_info_length_check as fil             # noqa: E402

#: 필수 키를 지우려면 format 조각과 인자 **둘 다** 지워야 짝이 맞는다.
#: 하나만 지우면 변환지시자/인자 수가 어긋나 별도 경로로 막힌다 — 그것도 fail-closed 다.
#: 필수 키를 지우려면 format 조각과 인자 **둘 다** 지워야 짝이 맞는다.
#: 하나만 지우면 변환지시자/인자 수가 어긋나 별도 경로로 막힌다 — 그것도 fail-closed 다.
#: 🔴 08-13 밤 2차 (검토 §69.2) — 정적 동작·안전 설정 **전량**으로 넓혔다.
REMOVALS = (
    ("odom_wheel_radius", "odom_wheel_radius=%.5f; ", "      ODOM_WHEEL_RADIUS,\n"),
    ("cmd_wheel_base", "cmd_wheel_base=%.3f; ", "      CMD_WHEEL_BASE,\n"),
    ("odom_wheel_base", "odom_wheel_base=%.3f; ", "      ODOM_WHEEL_BASE,\n"),
    ("control", "control=%s; ", '      USE_PID_D_TERM ? "PID" : "PI",\n'),
    ("kp", "kp=%.3f; ", "      WHEEL_KP,\n"),
    ("ki", "ki=%.3f; ", "      WHEEL_KI,\n"),
    ("kd", "kd=%.3f; ", "      WHEEL_KD,\n"),
    ("min_speed", "min_speed=%.3f; ", "      MIN_EFFECTIVE_WHEEL_CMD,\n"),
    ("start_boost_ms", "start_boost_ms=%lu; ",
     "      static_cast<unsigned long>(START_BOOST_DURATION_MS),\n"),
    ("estop_debounce_ms", "estop_debounce_ms=%lu; ",
     "      static_cast<unsigned long>(ESTOP_DEBOUNCE_MS),\n"),
)
#: 배열 필드는 지시자·인자가 넷씩이라 따로 다룬다.
ARRAY_REMOVALS = (
    ("hold_pwm", "hold_pwm=%d,%d,%d,%d; ",
     ("      LOW_SPEED_HOLD_PWM[FL],\n", "      LOW_SPEED_HOLD_PWM[RL],\n",
      "      LOW_SPEED_HOLD_PWM[FR],\n", "      LOW_SPEED_HOLD_PWM[RR],\n")),
    ("encoder_polarity", "encoder_polarity=%d,%d,%d,%d; ",
     ("      ENCODER_POLARITY[FL],\n", "      ENCODER_POLARITY[RL],\n",
      "      ENCODER_POLARITY[FR],\n", "      ENCODER_POLARITY[RR],\n")),
)
#: 🔴 검토 §70.2 — format 에 리터럴로 박힌 계약. 지시자가 없어 앞 판이 못 봤다.
LITERAL_REMOVALS = (
    ("transport", "transport=serial; "),
    ("baud", "baud=115200; "),
    ("low_speed_mode", "low_speed_mode=continuous_start_boost; "),
)

#: 계약이 아닌 런타임 카운터. 사라져도 검사가 막으면 안 된다(과잉 방어).
OPTIONAL_REMOVALS = (
    ("estop_raw_edges", "estop_raw_edges=%lu; ",
     "      static_cast<unsigned long>(estopFilter.rawEdges),\n"),
    ("disarm_estop", "disarm_estop=%lu; ",
     "      static_cast<unsigned long>(driveGate.disarmEstopCount),\n"),
)
#: 🔴 값이 바뀌면(소실이 아니라) 보드 대조에서 걸려야 한다 — 여기서는 **키 목록**이
#:   값을 싣고 나가는지만 본다. 실제 보드 대조는 `d0_check` 검사 7 이 한다.
VALUE_SHIFTS = (
    ("odom_wheel_radius", "static const double ODOM_WHEEL_RADIUS = 0.05698;",
     "static const double ODOM_WHEEL_RADIUS = 0.04603;"),
    ("estop_debounce_ms", "static const uint32_t ESTOP_DEBOUNCE_MS = 30;",
     "static const uint32_t ESTOP_DEBOUNCE_MS = 99;"),
)

FAILURES = []


def check(label, ok, detail=""):
    if ok:
        print("  \033[32mOK\033[0m  " + label)
    else:
        print("  \033[31mNG\033[0m  " + label + ("\n      " + detail if detail else ""))
        FAILURES.append(label)


def sketch_text():
    """스케치 전체(`.ino` + root 헤더)를 한 문자열로. 상수가 헤더에 있을 수 있다."""
    text = ""
    for path in fc._sketch_sources():                # noqa: SLF001
        with open(path, encoding="utf-8") as handle:
            text += handle.read() + "\n"
    return text


def with_source(mutate):
    """`.ino`·헤더를 문자열만 바꿔 읽히게 한다. 🔴 저장소 파일은 안 건드린다.

    format 은 `fil.read_ino` 로, 상수 값은 `fc._SOURCE_OVERRIDE` 로 간다 — 둘 다
    같은 변이를 받아야 "필드 소실" 과 "값 변경" 을 모두 재현할 수 있다.
    """
    original = fil.read_ino
    fil.read_ino = lambda: mutate(original())
    fc._SOURCE_OVERRIDE = mutate(sketch_text())      # noqa: SLF001
    try:
        return fc.firmware_identity_keys(), None
    except Exception as error:                       # noqa: BLE001
        return None, error
    finally:
        fil.read_ino = original
        fc._SOURCE_OVERRIDE = None                   # noqa: SLF001


#: 🔴 `d0_check` 의 build 필드 판정 반례 (검토 §71.2).
#:   앞 판은 정상 prefix 만 `grep -o` 해서 `...09:12:33garbage;` 가 정상 기대값과 같아졌다.
#:   필드를 **세미콜론 경계까지** 통째로 집어야 닫힌다. 여기서 그 규칙을 회귀로 못 박는다.
#:   (스크립트 자체는 실기 토픽이 있어야 돌므로, 판정 규칙만 같은 정규식으로 재현한다.)
BUILD_FIELD_RE = r"build=[^;]*;"
BUILD_CASES = (
    ("정상", "v; build=Aug 14 2026 09:12:33; x;", "Aug 14 2026 09:12:33", "ok"),
    ("정상 prefix 뒤 garbage", "v; build=Aug 14 2026 09:12:33garbage; x;",
     "Aug 14 2026 09:12:33", "ng"),
    ("stale (구판 build)", "v; build=Aug 12 2026 15:24:31; x;",
     "Aug 14 2026 09:12:33", "ng"),
    ("문법 오류", "v; build=Foo 99 99:99:99; x;", "Aug 14 2026 09:12:33", "ng"),
    ("build 중복", "v; build=A; z; build=B; x;", "A", "ng"),
    ("build 없음", "v; version=1; x;", "Aug 14 2026 09:12:33", "ng"),
)


def build_verdict(sample, expected):
    """`d0_check` 검사 7 의 build 분기와 **같은 규칙**으로 판정한다."""
    fields = re.findall(BUILD_FIELD_RE, sample)
    if not fields:
        return "ng"                       # build 없음
    if len(fields) != 1:
        return "ng"                       # 표본이 섞였거나 잘렸다
    return "ok" if fields[0].rstrip(";") == "build=" + expected else "ng"


def main():
    print("정체 스키마 회귀 (검토 §68.2 · §71.2)")
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

    print("\n[2b] 배열 필드(정수 4개)도 필수다 — 앞 판은 파싱조차 못 했다")
    for name, fmt_bit, arg_bits in ARRAY_REMOVALS:
        def drop(text, f=fmt_bit, bits=arg_bits):
            text = text.replace(f, "")
            for bit in bits:
                text = text.replace(bit, "")
            return text
        got, err = with_source(drop)
        check("② %s 소실 -> 거부" % name, got is None and err is not None,
              "정수·배열 상수를 필수로 못 올리면 동작 설정이 조용히 빠진다")

    print("\n[2c] 리터럴 필드도 필수다 — 지시자가 없어 앞 판이 못 봤다 (검토 §70.2)")
    for name, fmt_bit in LITERAL_REMOVALS:
        got, err = with_source(lambda s, f=fmt_bit: s.replace(f, "", 1))
        check("② %s 소실 -> 거부" % name, got is None and err is not None,
              "전송 계약이 조용히 빠지면 agent 가 안 붙는 이유를 못 찾는다")

    print("\n[3] 런타임 카운터는 통과해야 한다 (과잉 방어 금지)")
    for name, fmt_bit, arg_bit in OPTIONAL_REMOVALS:
        got, err = with_source(
            lambda s, f=fmt_bit, a=arg_bit: s.replace(f, "").replace(a, ""))
        check("③ %s 소실 -> 통과" % name, got is not None,
              "계약을 넓히는 방향까지 막으면 진단 필드를 못 뺀다: %s" % err)

    print("\n[3b] 값이 바뀌면 목록에도 새 값이 실려 나가야 한다")
    before = {k.split("=", 1)[0]: k.split("=", 1)[1] for k in keys}
    for name, old, new in VALUE_SHIFTS:
        got, err = with_source(lambda s, o=old, n=new: s.replace(o, n))
        after = ({k.split("=", 1)[0]: k.split("=", 1)[1] for k in got}
                 if got is not None else {})
        check("③b %s 값 변경이 목록에 반영된다" % name,
              name in after and after[name] != before.get(name),
              "상수를 바꿨는데 목록이 그대로면 `.ino` 를 안 읽고 있는 것이다 "
              "(%s -> %s · %s)" % (before.get(name), after.get(name), err))

    print("\n[4] 🔴 코드 목록 ↔ 정본 표를 **양방향** 대조한다 (검토 §69.2)")
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    canon_path = os.path.join(root, "docs", "REAL_ROBOT_VALUES.md")
    try:
        with open(canon_path, encoding="utf-8") as handle:
            canon = handle.read()
    except OSError as error:
        check("④ 정본을 읽는다", False, str(error))
        canon = ""
    check("④ 정본이 필수 키 집합의 소유자를 밝힌다",
          "REQUIRED_IDENTITY_KEYS" in canon,
          "정본에 근거가 없으면 이 목록은 코드가 혼자 정한 값이 된다")
    # 🔴 앞 판은 **단어가 있는지**만 봤다 — 코드 목록이 줄어도 계속 통과했다.
    #   ⚠ 절을 자를 때 다음 `####` 까지만 본다. 넓게 자르면 §1-b-3 의 다른 표까지
    #     읽어 엉뚱한 키를 "정본이 안다" 고 세게 된다.
    after = canon.split("§1-b-4", 1)[-1]
    table = after.split("\n#### ", 1)[0]
    for key in fc.REQUIRED_IDENTITY_KEYS:
        check("④ 정본 표에 `%s` 가 있다" % key, ("| `%s`" % key) in table,
              "코드가 필수로 삼는데 정본이 모르면 다음 사람이 지워도 된다고 읽는다")
    documented = set(re.findall(r"^\| `([a-z_0-9]+)`", table, re.M))
    extra = documented - set(fc.REQUIRED_IDENTITY_KEYS)
    check("④ 정본 표에 코드가 모르는 필수 키가 없다", not extra,
          "정본만 늘고 코드가 안 따라오면 검사는 여전히 통과한다: %s" % sorted(extra))

    print("\n[5] 🔴 build 필드는 세미콜론 경계까지 통째로 본다 (검토 §71.2)")
    for label, sample, expected, want in BUILD_CASES:
        got = build_verdict(sample, expected)
        check("⑤ %-22s -> %s" % (label, want), got == want,
              "정상 prefix 만 보면 뒤에 뭐가 붙어도 통과한다 (실제 판정: %s)" % got)

    print()
    if FAILURES:
        print("\033[31m%d 건 위반\033[0m" % len(FAILURES))
        return 1
    print("\033[32m전량 통과\033[0m — 스키마가 조용히 줄지 않는다")
    return 0


if __name__ == "__main__":
    sys.exit(main())
