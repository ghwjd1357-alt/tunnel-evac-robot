#!/usr/bin/env python3
"""`.ino` 의 **생산 함수 원문**을 잘라내 PC 에서 컴파일·실행하는 받침대 (검토 §66.2).

왜 필요한가
-----------
08-13 §65 보완에 붙인 두 회귀는 `.ino` 를 **읽기만** 하고, 실제 산술은 파이썬으로
다시 짠 복제 모형에서 돌았다. 검토 §66.2 가 그 틈으로 두 변이를 통과시켰다:

  ① 제어식에 `+ 0.010` 을 더해도 통과 — 정규식이 식의 **접두부만** 봤다.
  ② epoch 조건을 `==` 에서 `!=` 로 뒤집어도 통과 — 동작 검사가 파이썬 복제본이었다.

이건 §65.3 에서 고친 "도구가 상수를 베껴 자기확인한다" 와 **같은 병**이다. 그때는
상수를 베꼈고 이번엔 로직을 베꼈다. 그래서 이 모듈은 베끼지 않는다 — `.ino` 에서
함수 본문을 **글자 그대로** 떼어다 g++ 로 컴파일한다. 생산 코드가 바뀌면 이 시험이
돌리는 코드도 같이 바뀐다. 그게 유일한 자기확인 차단선이다.

왜 헤더로 분리하지 않았나 (안 A 대신 안 B)
------------------------------------------
검토 §66.2 의 최소 보완 방향은 두 갈래였다 — ⓐ 순수 헤더로 분리 ⓑ 생산 코드에서
추출한 실제 함수를 회귀 입력으로. ⓑ 를 골랐다. 이 묶음(예약 32-d)의 목적은
**제어 경로를 안 건드리는 것**인데, PI 산술을 새 헤더로 옮기는 것 자체가 제어
경로 편집이다 — §65.1 이 금지한 일을 형태만 바꿔 하는 셈이다. ⓑ 는 펌웨어
바이트를 1 도 안 바꾸므로 `b9fb8e3` 의 컴파일·게이트 증거가 그대로 유효하다.

⚠ 이 방식이 증명하지 않는 것: 링크·micro-ROS 콜백 배선·실제 타이밍. 그건
`rearm_gate_host_test.sh` 2단계 구조 검사와 실기 `JETSON_SETUP §7-c-E` 몫이다.

부모판 비교
----------
`load(ref)` 에 git ref 를 주면 그 시점 `.ino` 를 읽는다. 상수 이름이 세대마다
다르므로(부모판 `WHEEL_RADIUS` vs 현행 `ODOM_WHEEL_RADIUS`) 상수는 이름을 열거하지
않고 **전량 자동 추출**한다. 그래야 세대가 바뀌어도 받침대가 안 낡는다.

종료코드 계약: 이 모듈은 예외로만 실패를 알린다. 컴파일 실패는 `ProbeError` 다 —
**"못 돌렸다" 와 "돌렸는데 통과" 를 같은 값으로 적지 않는다** (rearm_gate_host_test
머리말과 같은 규칙).
"""

import os
import re
import shutil
import subprocess
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SKETCH_DIR = os.path.join(ROOT, "firmware", "teensy_integrated_base_v1_4")
INO_REL = "firmware/teensy_integrated_base_v1_4/teensy_integrated_base_v1_4.ino"
INO_PATH = os.path.join(ROOT, INO_REL)


class ProbeError(RuntimeError):
    """추출·컴파일·실행이 불가능하다 = 판정 불능. 통과로 읽으면 안 된다."""


# ── 원문 읽기 ────────────────────────────────────────────────────────────────
def load(ref=None):
    """작업본(ref=None) 또는 특정 커밋의 `.ino` 원문을 돌려준다."""
    if ref is None:
        with open(INO_PATH, encoding="utf-8") as handle:
            return handle.read()

    done = subprocess.run(
        ["git", "-C", ROOT, "show", "%s:%s" % (ref, INO_REL)],
        capture_output=True, text=True)
    if done.returncode != 0:
        raise ProbeError("git show %s:%s 실패\n%s" % (ref, INO_REL, done.stderr))
    return done.stdout


# ── 원문 조각 떼어내기 ───────────────────────────────────────────────────────
def function(source, name):
    """이름으로 함수 정의를 **글자 그대로** 떼어낸다 (중괄호 균형으로 끝을 찾는다)."""
    pattern = r"^[\w:<>,\s\*&]*?\b%s\s*\([^;{]*\)\s*\{" % re.escape(name)
    match = re.search(pattern, source, re.M)
    if match is None:
        raise ProbeError(
            "함수 %s 를 `.ino` 에서 못 찾았다 — 이름이 바뀌었으면 시험을 먼저 고쳐라"
            % name)

    depth = 0
    for index in range(source.index("{", match.start()), len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[match.start():index + 1]
    raise ProbeError("함수 %s 의 중괄호가 안 닫혔다" % name)


def enum(source, name):
    """`enum NAME { ... };` 를 원문 그대로 떼어낸다."""
    match = re.search(r"enum\s+%s\s*\{.*?\};" % re.escape(name), source, re.S)
    if match is None:
        raise ProbeError("enum %s 를 못 찾았다" % name)
    return match.group(0)


#: 초기화식에 이 토큰이 있으면 Arduino 런타임이 필요해 host 로 못 가져온다.
_ARDUINO_ONLY = re.compile(r"\b(HIGH|LOW|STRINGIFY|A\d+)\b")


def constants(source):
    """파일 최상단 `static const` 선언을 **전량** 원문 그대로 모은다.

    이름을 열거하지 않는 이유: 부모판과 현행판은 상수 이름이 다르다
    (`WHEEL_RADIUS` -> `ODOM_WHEEL_RADIUS`). 목록을 손으로 들면 그 목록이 또
    하나의 베낀 사본이 된다.
    """
    kept = []
    for match in re.finditer(r"^static const [^;]*?;", source, re.M | re.S):
        text = match.group(0)
        head, _, initializer = text.partition("=")
        if re.search(r"\bchar\b", head):
            continue                       # 문자열 상수는 산술에 안 쓰인다
        if _ARDUINO_ONLY.search(initializer):
            continue                       # 핀 레벨 등 보드 전용
        kept.append(text)
    if not kept:
        raise ProbeError("상수를 하나도 못 뗐다 — 선언 형식이 바뀌었다")
    return "\n".join(kept)


# ── 컴파일·실행 ──────────────────────────────────────────────────────────────
#: Arduino 가 주던 것 중 떼어낸 조각이 실제로 쓰는 것만 최소로 흉내낸다.
PRELUDE = """\
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <cstring>
using std::abs;
using std::fabs; using std::round; using std::cos; using std::sin;
using std::sqrt; using std::atan2; using std::isfinite;
#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif
template <typename A, typename B>
static inline double arduino_max(A a, B b) {
  return (double)a > (double)b ? (double)a : (double)b;
}
#define max(a, b) arduino_max((a), (b))
"""

CXX_FLAGS = ["-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror",
             "-Wno-unused-function", "-Wno-unused-const-variable"]


def compile_and_run(pieces, source_name="probe.cpp", include_sketch=False):
    """조각들을 한 파일로 붙여 컴파일하고 실행 표준출력을 돌려준다.

    `pieces` 는 문자열 목록이고 순서대로 이어 붙인다. `PRELUDE` 는 자동으로 맨 앞.
    `-ffast-math` 류는 절대 안 켠다 — isfinite 검사가 무력화된다.
    """
    compiler = os.environ.get("CXX", "g++")
    if shutil.which(compiler) is None:
        raise ProbeError("C++ 컴파일러가 없다: %s  (sudo apt install g++)" % compiler)

    workdir = tempfile.mkdtemp(prefix="ino-host-probe.")
    try:
        source_path = os.path.join(workdir, source_name)
        with open(source_path, "w", encoding="utf-8") as handle:
            handle.write(PRELUDE)
            handle.write("\n")
            handle.write("\n\n".join(pieces))
            handle.write("\n")

        binary = os.path.join(workdir, "probe")
        command = [compiler] + CXX_FLAGS
        if include_sketch:
            command += ["-I", SKETCH_DIR]
        command += ["-o", binary, source_path]

        built = subprocess.run(command, capture_output=True, text=True)
        if built.returncode != 0:
            raise ProbeError(
                "추출한 생산 코드가 host 에서 컴파일되지 않는다.\n"
                "  (의존성이 늘었으면 shim 을 먼저 고쳐라 — 조용히 넘기지 않는다)\n"
                + built.stderr)

        ran = subprocess.run([binary], capture_output=True, text=True)
        if ran.returncode != 0:
            raise ProbeError(
                "probe 실행이 비정상 종료했다 (rc=%d)\n%s"
                % (ran.returncode, ran.stderr))
        return ran.stdout
    finally:
        shutil.rmtree(workdir, ignore_errors=True)


# ── 변이 주입 (역회귀 전용) ──────────────────────────────────────────────────
def mutate(source, old, new):
    """원문 한 곳을 바꾼 사본을 만든다. 시험이 **자기 감도**를 증명할 때만 쓴다.

    저장소 파일은 절대 안 건드린다 — 문자열만 돌려준다.
    """
    if source.count(old) != 1:
        raise ProbeError(
            "변이 지점이 %d 곳이다 (정확히 1 이어야 한다): %r"
            % (source.count(old), old))
    return source.replace(old, new)
