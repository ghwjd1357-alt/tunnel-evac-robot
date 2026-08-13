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


def _load():
    if _CACHE:
        return _CACHE
    with open(INO_PATH, encoding="utf-8") as handle:
        source = handle.read()
    for name, value in re.findall(
            r"static\s+const\s+double\s+(\w+)\s*=\s*([0-9][0-9.eE+-]*)\s*;", source):
        _CACHE[name] = float(value)
    if not _CACHE:
        raise RuntimeError("`.ino` 에서 상수를 하나도 못 읽었다: " + INO_PATH)
    return _CACHE


def firmware_double(name):
    """`.ino` 의 `static const double NAME = 값;` 을 읽는다.

    🔴 없으면 `KeyError`. 기본값으로 대신하지 않는다 — 이름이 바뀐 것을 시험이
    알아채야 하고, 조용히 넘어가면 이 모듈의 존재 이유가 사라진다.
    """
    table = _load()
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


#: 🔴 `/firmware/info` 가 **반드시** 실어야 하는 의미 키 (검토 §68.2).
#:   왜 목록을 여기서 소유하나: 앞 판은 format 에 **있는 것**을 긁어 기대 목록으로 삼았다.
#:   그러면 필수 필드가 펌웨어와 format 에서 **함께** 사라질 때 검사기가 줄어든 계약을
#:   그대로 새 정답으로 받아들인다 — 값 사본을 없앤 것과 **스키마를 안 드는 것**은 다른
#:   문제였다. 값은 계속 `.ino` 에서 읽고, **어떤 키가 있어야 하는가**만 여기서 든다.
#:   🔴 이 집합을 줄이는 것은 **스키마 개정**이다 — 정본(`REAL_ROBOT_VALUES §1-b`)을
#:   같이 고치고, 왜 필요 없어졌는지를 적는다. 자동으로 줄어들지 않는다.
#:   ⚠ 여기 없는 필드가 새로 생기는 것은 **진단 필드 추가**로 보고 통과시킨다 —
#:     계약을 넓히는 방향이라 위험하지 않다. 필수로 올리려면 이 집합에 명시로 넣는다.
REQUIRED_IDENTITY_KEYS = (
    "odom_wheel_radius",   # odom 거리·yaw 눈금 (예약 32-e)
    "cmd_wheel_base",      # 명령 경로 윤거 — odom 과 섞이면 안 된다
    "odom_wheel_base",     # odom yaw 전용 유효 윤거
    "kp", "ki", "kd",      # 제어 게인 — 시험 데이터의 전제
)


def firmware_identity_fields():
    """`/firmware/info` format 이 싣는 **실수 필드 전량**을 `{키: 값문자열}` 로 돌려준다.

    format 과 인자 목록에서 `이름=%.Nf` 짝을 직접 읽는다. 값은 `.ino` 상수에서 온다.
    🔴 이것은 **발견 목록**이지 계약이 아니다 — 계약은 `REQUIRED_IDENTITY_KEYS` 다.
    """
    import firmware_info_length_check as fil          # noqa: PLC0415

    source = fil.read_ino()
    _buffer, fmt, args = fil.extract(source)
    table = _load()

    found = {}
    specs = fil.SPEC.findall(fmt)
    if len(specs) != len(args):
        raise ValueError("변환지시자 %d 개 vs 인자 %d 개 — 짝이 안 맞는다"
                         % (len(specs), len(args)))
    cursor = 0
    for spec, arg in zip(specs, args):
        at = fmt.index(spec, cursor)
        cursor = at + len(spec)
        if not spec.endswith("f"):
            continue
        head = fmt[:at]
        if not head.endswith("="):
            continue
        name = re.split(r"[;\s]", head[:-1])[-1]
        value = table.get(arg.strip())
        if name and value is not None:
            digits = int(re.search(r"\.(\d+)", spec).group(1))
            found[name] = "%.*f" % (digits, value)
    return found


def firmware_identity_keys():
    """`d0_check` 검사 7 이 보드 출력에서 찾을 `키=값` 목록 (검토 §68.2).

    🔴 **필수 키가 하나라도 format 에 없으면 예외**다. 검사기가 축소된 계약을 조용히
    받아들이는 길을 막는다 — 08-13 밤에 `CONTROL_WHEEL_RADIUS` 를 지우자 앞 판이
    그것을 새 정답으로 삼았고(그때는 의도된 삭제였지만), 같은 경로로 `odom_wheel_base`
    가 사라져도 검사가 통과했을 것이다.

    ⚠ `build` 는 여기서 안 낸다 — 문자열이라 `.ino` 상수에 없다. `d0_check` 가
      별도로 관측한다(정본은 `build` 가 굽힘 판별의 유일한 기준이라고 말한다).
    """
    found = firmware_identity_fields()
    missing = [k for k in REQUIRED_IDENTITY_KEYS if k not in found]
    if missing:
        raise KeyError(
            "`/firmware/info` format 에 필수 정체 키가 없다: %s\n"
            "  🔴 필드를 지웠다면 그것은 **스키마 개정**이다 — "
            "REQUIRED_IDENTITY_KEYS 와 정본을 같이 고쳐라.\n"
            "  지금 있는 키: %s"
            % (", ".join(missing), ", ".join(sorted(found))))
    return ["%s=%s" % (k, found[k]) for k in sorted(found)]
