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
