#!/usr/bin/env bash
# rearm_gate_host_test.sh — re-arm 상태 전이 harness 를 PC 에서 컴파일·실행한다.
#
# 사용:  bash tools/rearm_gate_host_test.sh
# 종료:  0 = 전이 계약 전량 통과 / 1 = 계약 위반 / 2 = 판정 불능(컴파일러·경로)
#        🔴 2 를 0 처럼 읽지 않는다 — "못 돌렸다"와 "돌렸는데 통과"는 다른 사실이다.
#
# 왜 이게 필요한가 (검토 §54.7): 상태 전이를 실기로만 확인하면 500ms 경계의 앞뒤
# 1ms 나 rclc take 스냅샷 틈 같은 것을 재현할 수 없다. rearm_gate.h 는 Arduino 를
# 안 쓰는 순수 헤더라 g++ 로 그대로 컴파일된다 — 보드 없이 전이를 전수한다.
#
# ⚠ 이 검사가 증명하지 않는 것: `.ino` 의 배선(stopAllMotors 호출·publish 내용).
#    그건 실기 `docs/JETSON_SETUP.md §7-c-E` 가 관측한다.

set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$HERE")"
SRC="$HERE/rearm_gate_host_test.cpp"
HDR="$REPO/firmware/teensy_integrated_base_v1_4/rearm_gate.h"

for f in "$SRC" "$HDR"; do
    if [ ! -f "$f" ]; then
        echo "FAIL 입력 파일이 없다: $f" >&2; exit 2
    fi
done

CXX="${CXX:-g++}"
if ! command -v "$CXX" >/dev/null 2>&1; then
    echo "FAIL C++ 컴파일러가 없다: $CXX  (sudo apt install g++)" >&2; exit 2
fi

TMP="$(mktemp -d -t rearm-gate-test.XXXXXX)" || {
    echo "FAIL 임시 디렉터리를 만들지 못했다" >&2; exit 2; }
trap 'rm -rf "$TMP"' EXIT
BIN="$TMP/rearm_gate_host_test"

# -Wall -Wextra -Werror: 헤더가 Teensy 쪽 컴파일러에서만 조용한 상태로 남지 않게 한다.
# NaN/Inf 케이스가 있으므로 -ffast-math 류는 절대 켜지 않는다 — isfinite 가 무력화된다.
if ! "$CXX" -std=c++17 -O1 -Wall -Wextra -Werror -o "$BIN" "$SRC" 2>"$TMP/cc.log"; then
    echo "FAIL harness 컴파일 실패:" >&2
    cat "$TMP/cc.log" >&2
    exit 2
fi

"$BIN"
rc=$?
exit "$rc"
