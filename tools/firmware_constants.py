#!/usr/bin/env python3
"""펌웨어 `.ino` 의 상수를 **거기서 읽어 온다**. 베껴 적지 않는다 (검토 §65.3).

왜 이 모듈이 있나
-----------------
`drive_encoder_check` · `watchdog_video` 같은 판정 도구는 펌웨어와 같은 상수를 써야
같은 물건을 잰다. 그런데 08-13 까지는 각 도구가 숫자를 **손으로 베껴** 들고 있었고,
회귀 시험도 그 베낀 숫자를 정답으로 고정했다:

    self.assertEqual(0.05698, ec.WHEEL_RADIUS_M)   # ← 도구와 시험이 서로를 확인

펌웨어가 0.04603 으로 바뀌어도 이 시험은 초록이다. 도구와 시험이 같은 옛 숫자를 보며
서로에게 맞다고 해 주는 **자기확인**이고, 검토 §65.3 이 이 상태를 지적했다.

이 모듈은 `.ino` 를 정본으로 삼는다. 펌웨어가 바뀌면 시험이 **깨진다** — 그게 목적이다.

쓰는 법
-------
    from tools.firmware_constants import firmware_double

    firmware_double('ODOM_WHEEL_RADIUS')   # -> 0.04603
    firmware_double('CMD_WHEEL_BASE')      # -> 0.62

이름이 없으면 `KeyError` 다. 조용히 기본값으로 넘어가면 자기확인이 되살아난다.

이 로봇의 상수 세 갈래 (합치지 않는다)
--------------------------------------
    물리 0.49       줄자로 잰 실제 바퀴 간격. URDF 몫. `.ino` 에 없다.
    명령 0.62       cmd_vel -> 바퀴 목표          `CMD_WHEEL_BASE`
    odom 0.670      엔코더 -> yaw                  `ODOM_WHEEL_BASE`
    odom 반지름     엔코더 -> 거리                 `ODOM_WHEEL_RADIUS`
    제어 반지름     PI 피드백 (판재 이전 눈금)     `CONTROL_WHEEL_RADIUS`
"""

import os
import re

INO_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..",
    "firmware",
    "teensy_integrated_base_v1_4",
    "teensy_integrated_base_v1_4.ino",
)

_CACHE = {}
_LITERALS = {}


#: 🔴 08-13 밤 2차 (검토 §69.2) — `double` 만 읽던 것을 정수·bool·배열까지 넓혔다.
#:   앞 판은 `LOW_SPEED_HOLD_PWM`·`ENCODER_POLARITY`·`ESTOP_DEBOUNCE_MS` 같은
#:   **동작·안전 설정을 필수 스키마로 올릴 방법 자체가 없었다.**
_NUM_TYPES = r"(?:double|float|int|long|unsigned|u?int\d+_t)"
_SCALAR_RE = re.compile(
    r"static\s+const\s+" + _NUM_TYPES + r"\s+(\w+)\s*=\s*(-?[0-9][0-9.eE+-]*)\s*;")
_BOOL_RE = re.compile(r"static\s+const\s+bool\s+(\w+)\s*=\s*(true|false)\s*;")
_ARRAY_RE = re.compile(
    r"static\s+const\s+" + _NUM_TYPES +
    r"\s+(\w+)\s*\[\s*\d*\s*\]\s*=\s*\{([^}]*)\}\s*;")


#: 스케치 root 의 헤더도 컴파일에 들어간다 — 상수가 거기 있을 수 있다
#: (`ESTOP_DEBOUNCE_MS` 는 `estop_debounce.h` 다). `.ino` 만 읽으면 못 찾는다.
def _sketch_sources():
    directory = os.path.dirname(os.path.abspath(INO_PATH))
    paths = [INO_PATH]
    for entry in sorted(os.listdir(directory)):
        if entry.endswith((".h", ".hpp")):
            paths.append(os.path.join(directory, entry))
    return paths


#: 🔴 시험 전용 이음매. `test_identity_schema` 가 값 변이를 주입할 때만 쓴다.
#:   `None` 이면 언제나 실제 스케치를 읽는다 — 생산 경로에는 영향이 없다.
_SOURCE_OVERRIDE = None


def _load():
    if _SOURCE_OVERRIDE is None and _CACHE:
        return _CACHE
    if _SOURCE_OVERRIDE is not None:
        table = {}
        for name, value in _SCALAR_RE.findall(_SOURCE_OVERRIDE):
            table[name] = float(value)
        for name, value in _BOOL_RE.findall(_SOURCE_OVERRIDE):
            table[name] = (value == "true")
        for name, body in _ARRAY_RE.findall(_SOURCE_OVERRIDE):
            try:
                table[name] = [float(p.strip()) for p in body.split(",") if p.strip()]
            except ValueError:
                continue
        return table
    source = ""
    for path in _sketch_sources():
        with open(path, encoding="utf-8") as handle:
            source += handle.read() + "\n"
    for name, value in _SCALAR_RE.findall(source):
        _CACHE[name] = float(value)
    for name, value in _BOOL_RE.findall(source):
        _CACHE[name] = (value == "true")
    # 🔴 배열은 원소 목록으로 담는다 — `hold_pwm=%d,%d,%d,%d` 같은 필드가 쓴다.
    for name, body in _ARRAY_RE.findall(source):
        try:
            _CACHE[name] = [float(piece.strip()) for piece in body.split(",")
                            if piece.strip()]
        except ValueError:
            continue                      # HIGH/LOW 등 보드 전용 심볼은 건너뛴다
    if not _CACHE:
        raise RuntimeError("`.ino` 에서 상수를 하나도 못 읽었다: " + INO_PATH)
    return _CACHE


def firmware_double(name):
    """`.ino` 의 `static const double NAME = 값;` 을 읽는다.

    🔴 없으면 `KeyError`. 기본값으로 대신하지 않는다 — 이름이 바뀐 것을 시험이
    알아채야 하고, 조용히 넘어가면 이 모듈의 존재 이유가 사라진다.
    """
    table = _load()
    if isinstance(table.get(name), (list, bool)):
        raise KeyError("상수 %s 는 배열/bool 이다 — firmware_double 로 읽지 않는다" % name)
    if name not in table:
        raise KeyError(
            "`.ino` 에 상수 %r 이 없다. 이름이 바뀌었으면 부르는 쪽을 고쳐라 — "
            "여기서 기본값을 주면 검토 §65.3 의 자기확인으로 되돌아간다. "
            "지금 있는 이름: %s" % (name, ", ".join(sorted(table))))
    return table[name]


def firmware_constants():
    """읽은 상수 전부. 디버깅·목록 확인용."""
    return dict(_load())


if __name__ == "__main__":
    for key, val in sorted(firmware_constants().items()):
        print("%-34s = %s" % (key, val))


#: 🔴 `/firmware/info` 가 **반드시** 실어야 하는 **안전·동작 핵심 shortlist**.
#:
#:   ⚠ **08-14 정정 (검토 §70.2)** — 앞 판은 이 목록을 *"정적 동작·안전 설정 **전량**"*
#:   이라고 불렀다. **거짓이다.** `_load()` 가 읽는 부팅 고정값에는 `MAX_LINEAR_CMD`,
#:   `MAX_ANGULAR_CMD`, `WATCHDOG_TIMEOUT_MS`, `FEEDFORWARD_*`, `MAX_CONTROL_PWM`,
#:   `START_BOOST_PWM`, `MIN_RUNNING_PWM`, `PWM_RAMP_*`, `COMMAND_DEADBAND`,
#:   `TOTAL_PPR`, `ESTOP_ACTIVE_LOW`, `REARM_*` 등이 더 있고 **그 대부분은 애초에
#:   `/firmware/info` 로 나가지도 않는다.** 그래서 "전량" 이라는 경계는 이 검사가
#:   지킬 수 있는 것이 아니다 — 주장을 **shortlist** 로 낮춘다.
#:
#:   선정 기준(주장할 수 있는 만큼만): *`/firmware/info` 로 실제로 나가면서, 틀리면
#:   지금까지의 실차 실측을 통째로 무효로 만드는 값.* 🔴 "부팅 고정값 전량" 이 아니다 —
#:   그런 값의 대부분은 애초에 발행되지 않는다(§71.2).
#:   ⚠ 줄이는 것은 **스키마 개정**이다: 이 표와 `REAL_ROBOT_VALUES §1-b-4` 를 같이
#:     고친다. 늘어나는 것(진단 필드 추가)은 자동 통과 — 계약을 넓히는 방향이다.
#:   🔴 **재개방** — `/firmware/info` 에 새 정적 설정을 실으면 여기 넣을지 그때 판단한다.
#:     자동으로 필수가 되지 않는다(§69.2 정책 그대로).
REQUIRED_IDENTITY_KEYS = (
    # 구동 기하 — odom 과 명령 경로
    "odom_wheel_radius", "cmd_wheel_base", "odom_wheel_base",
    # 제어 — 모든 시험 데이터의 전제
    "control", "kp", "ki", "kd",
    # 저속 구동 — R2 회전의 물리 하한과 실제 출력
    "min_speed", "start_boost_ms", "hold_pwm",
    # 센서 방향 — 부호가 뒤집히면 odom 이 통째로 거짓이 된다
    "encoder_polarity",
    # 안전 입력 판정 문턱
    "estop_debounce_ms",
    # 🔴 검토 §70.2 — format 에 **리터럴로 박힌** 계약. 상수가 아니라 문자열이라
    #   앞 판 파서가 아예 못 봤고, 지워도 수용됐다. 전송 계약이 바뀌면 agent 가 안 붙는다.
    "transport", "baud", "low_speed_mode",
)

#: 주행이 쌓는 값. 사라져도 정체 검사를 막지 않는다(계약이 아니다).
RUNTIME_COUNTER_KEYS = (
    "estop_raw_edges", "estop_max_high_ms", "estop_rejected",
    "estop_rejected_max_ms", "disarm_estop", "disarm_nonfinite",
    "disarm_nonzero", "applied_pwm", "applied_pwm_max", "applied_pwm_epoch",
)

#: `%s` 로 나가지만 값이 `.ino` 상수가 아닌 것(빌드 시각·경로 등). 스키마에서 뺀다.
_BUILD_META_KEYS = ("version", "git_sha", "git_short", "build", "source",
                    "arduino_macro", "teensyduino_macro", "libraries")

#: 필드 하나가 여러 변환지시자를 쓰는 경우 (`hold_pwm=%d,%d,%d,%d`).
_ARRAY_ARITY = {"hold_pwm": 4, "encoder_polarity": 4, "applied_pwm": 4}


#: 배열 첨자 이름 -> 인덱스 (`.ino` 의 enum MotorIndex).
_MOTOR_INDEX = {"FL": 0, "RL": 1, "FR": 2, "RR": 3}


def _peel(expression):
    """인자 표현식에서 상수 이름(과 첨자)을 벗겨낸다 (검토 §69.2).

    실제 format 인자는 `static_cast<unsigned long>(ESTOP_DEBOUNCE_MS)` ·
    `LOW_SPEED_HOLD_PWM[FL]` 같은 모양이다. 앞 판은 문자열을 그대로 상수표에서 찾아
    **정수·배열 필드를 하나도 못 풀었고**, 그래서 그것들을 필수로 올릴 수도 없었다.
    """
    text = expression.strip()
    cast = re.match(r"static_cast<[^>]+>\s*\((.*)\)\s*$", text)
    if cast:
        text = cast.group(1).strip()
    return text


def _lookup(table, expression):
    """벗겨낸 표현식을 값으로. 첨자가 있으면 그 원소를 돌려준다."""
    text = _peel(expression)
    subscript = re.match(r"(\w+)\s*\[\s*(\w+)\s*\]\s*$", text)
    if subscript:
        array = table.get(subscript.group(1))
        if not isinstance(array, list):
            return None
        key = subscript.group(2)
        position = _MOTOR_INDEX.get(key)
        if position is None and key.isdigit():
            position = int(key)
        if position is None or position >= len(array):
            return None
        return array[position]
    return table.get(text)


def _format_one(spec, value):
    """하나의 변환지시자를 그 값으로 찍는다. `%.Nf` · `%d` · `%lu` 를 받는다."""
    if spec.endswith("f"):
        digits = int(re.search(r"\.(\d+)", spec).group(1))
        return "%.*f" % (digits, value)
    return "%d" % int(round(value))


def firmware_identity_fields():
    """`/firmware/info` format 이 싣는 **상수 유래 필드 전량**을 `{키: 값문자열}` 로.

    🔴 08-13 밤 2차 (검토 §69.2) — `%f` 만 보던 것을 `%d`·`%lu`·배열까지 넓혔다.
    앞 판은 정수·배열 상수를 **필수로 올릴 방법 자체가 없었다.**
    ⚠ 이것은 **발견 목록**이지 계약이 아니다 — 계약은 `REQUIRED_IDENTITY_KEYS` 다.
    """
    import firmware_info_length_check as fil          # noqa: PLC0415

    source = fil.read_ino()
    _buffer, fmt, args = fil.extract(source)
    table = _load()

    specs = fil.SPEC.findall(fmt)
    if len(specs) != len(args):
        raise ValueError("변환지시자 %d 개 vs 인자 %d 개 — 짝이 안 맞는다"
                         % (len(specs), len(args)))

    # 🔴 검토 §70.2 — `transport=serial;` 처럼 **변환지시자 없이 리터럴로 박힌** 필드.
    #   앞 판은 지시자만 훑어서 이런 계약을 처음부터 못 봤다.
    for name, literal in re.findall(r"(\w+)=([^%;\"]+);", fmt):
        if name not in _BUILD_META_KEYS:
            found_literal = literal.strip()
            if found_literal:
                _LITERALS[name] = found_literal

    found, cursor, index = {}, 0, 0
    while index < len(specs):
        spec = specs[index]
        at = fmt.index(spec, cursor)
        head = fmt[:at]
        if not head.endswith("="):
            cursor = at + len(spec)
            index += 1
            continue
        name = re.split(r"[;\s]", head[:-1])[-1]
        arity = _ARRAY_ARITY.get(name, 1)
        raw = _peel(args[index])
        # 🔴 `control=%s` 처럼 인자가 삼항이면 상수를 풀어 실제 문자열을 낸다.
        ternary = re.match(r'(\w+)\s*\?\s*"([^"]*)"\s*:\s*"([^"]*)"', raw)
        if ternary:
            flag = table.get(ternary.group(1))
            if flag is not None:
                if name not in _BUILD_META_KEYS:
                    found[name] = ternary.group(2) if flag else ternary.group(3)
                cursor = at + len(spec)
                index += 1
                continue
        pieces = []
        if arity > 1:
            # 🔴 `hold_pwm=%d,%d,%d,%d` 는 인자가 **첨자로 네 개** 펼쳐져 있다.
            for offset in range(arity):
                if index + offset >= len(args):
                    pieces = []
                    break
                element = _lookup(table, args[index + offset])
                if element is None:
                    pieces = []
                    break
                pieces.append(_format_one(specs[index + offset], element))
        else:
            value = _lookup(table, args[index])
            if value is not None:
                pieces = [_format_one(spec, value)]
        if pieces and name not in _BUILD_META_KEYS:
            found[name] = ",".join(pieces)
        # 🔴 배열 필드를 소비했으면 **그 지시자 개수만큼** 커서를 밀어야 한다.
        #   앞 판은 첫 지시자 뒤에만 두어, 다음 회차가 같은 필드의 둘째 `%d` 를 다시
        #   집고 `head` 가 `=` 로 안 끝나 조용히 건너뛰었다 — 그 뒤 전부 어긋났다.
        consumed = arity if len(pieces) > 1 else 1
        cursor = at
        for _ in range(consumed):
            cursor = fmt.index(specs[index], cursor) + len(specs[index])
        index += consumed
    found.update(_LITERALS)
    _LITERALS.clear()
    return found


def firmware_identity_keys():
    """`d0_check` 검사 7 이 보드 출력에서 찾을 `키=값` 목록 (검토 §68.2 · §69.2).

    🔴 **필수 키가 하나라도 format 에 없으면 예외**다. 검사기가 축소된 계약을 조용히
    받아들이는 길을 막는다.

    ⚠ `build` 는 여기서 안 낸다 — 문자열이라 `.ino` 상수에 없다. `d0_check` 가 별도로
      **기대 문자열과 대조**한다(정본은 `build` 가 굽힘 판별의 유일한 기준이라 한다).
      기대값이 없으면 `ok` 가 아니라 **미판정**으로 찍는다.
    """
    found = firmware_identity_fields()
    missing = [k for k in REQUIRED_IDENTITY_KEYS if k not in found]
    if missing:
        raise KeyError(
            "`/firmware/info` format 에 필수 정체 키가 없다: %s\n"
            "  🔴 필드를 지웠다면 그것은 **스키마 개정**이다 — "
            "REQUIRED_IDENTITY_KEYS 와 정본(REAL_ROBOT_VALUES §1-b-4)을 같이 고쳐라.\n"
            "  지금 있는 키: %s"
            % (", ".join(missing), ", ".join(sorted(found))))
    return ["%s=%s" % (k, found[k]) for k in sorted(found)]
