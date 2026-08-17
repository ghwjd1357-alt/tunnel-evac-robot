#!/usr/bin/env bash
# rearm_gate_host_test.sh — re-arm 상태 전이 + 정지 배선을 PC 에서 검사한다.
#
# 사용:  bash tools/rearm_gate_host_test.sh
# 종료:  0 = 계약 전량 통과 / 1 = 계약 위반 / 2 = 판정 불능(컴파일러·경로)
#        🔴 2 를 0 처럼 읽지 않는다 — "못 돌렸다"와 "돌렸는데 통과"는 다른 사실이다.
#
# 왜 이게 필요한가 (검토 §54.7): 상태 전이를 실기로만 확인하면 500ms 경계의 앞뒤
# 1ms 나 rclc take 스냅샷 틈 같은 것을 재현할 수 없다. rearm_gate.h·drive_wiring.h 는
# Arduino 를 안 쓰는 순수 헤더라 g++ 로 그대로 컴파일된다 — 보드 없이 전이를 전수한다.
#
# 두 단계인 이유 (검토 §55.2):
#   1단계 **동작 검사** — 헤더의 결정과 정지 부작용을 가짜 모터로 관측한다. 강한 증거.
#   2단계 **구조 검사** — 스케치가 그 헤더 함수들을 실제로 부르는지 텍스트로 본다.
#       약한 증거다(호출 여부만 보지 호출 결과를 안 본다). 약한 줄 알고 쓴다.
#       이 단계가 없으면 .ino 에서 호출 한 줄만 지워도 1단계는 초록이다.
#
# ⚠ 이 검사가 증명하지 않는 것: PWM 파형·publish 내용·응답이 클라이언트에 닿은 시각.
#    그건 실기 `docs/JETSON_SETUP.md §7-c-E` 가 관측한다.

set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$HERE")"
SKETCH="$REPO/firmware/teensy_integrated_base_v1_4"
SRC="$HERE/rearm_gate_host_test.cpp"
INO="$SKETCH/teensy_integrated_base_v1_4.ino"

for f in "$SRC" "$INO" "$SKETCH/rearm_gate.h" "$SKETCH/drive_wiring.h"; do
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

# ── 1단계: 동작 검사 ─────────────────────────────────────────────────────────
# -Wall -Wextra -Werror: 헤더가 Teensy 쪽 컴파일러에서만 조용한 상태로 남지 않게 한다.
# NaN/Inf 케이스가 있으므로 -ffast-math 류는 절대 켜지 않는다 — isfinite 가 무력화된다.
if ! "$CXX" -std=c++17 -O1 -Wall -Wextra -Werror -o "$BIN" "$SRC" 2>"$TMP/cc.log"; then
    echo "FAIL harness 컴파일 실패:" >&2
    cat "$TMP/cc.log" >&2
    exit 2
fi

"$BIN"
behavior_rc=$?
if [ "$behavior_rc" -gt 1 ]; then
    echo "FAIL harness 실행이 비정상 종료했다 (rc=$behavior_rc)" >&2
    exit 2
fi

# ── 2단계: 구조 검사 — 스케치가 헤더를 부르는가 ──────────────────────────────
echo
echo "=== 구조 검사: .ino 가 drive_wiring.h 를 실제로 부르는가 (검토 §55.2) ==="

struct_checks=0
struct_fail=0

# 함수 하나의 본문만 잘라낸다 (서명 줄 ~ 첫 열의 닫는 중괄호).
fn_body() {
    awk -v pat="$1" '
        !inside && $0 ~ pat { inside = 1 }
        inside { print }
        inside && /^}/ { exit }
    ' "$INO"
}

expect_contains() {   # $1 설명 · $2 함수 서명 regex · $3 있어야 할 문자열
    struct_checks=$((struct_checks + 1))
    if ! fn_body "$2" | grep -qF -- "$3"; then
        echo "  FAIL $1 — '$3' 가 없다"
        struct_fail=$((struct_fail + 1))
    fi
}

expect_absent() {     # $1 설명 · $2 함수 서명 regex · $3 없어야 할 문자열
    struct_checks=$((struct_checks + 1))
    if fn_body "$2" | grep -qF -- "$3"; then
        echo "  FAIL $1 — '$3' 가 남아 있다"
        struct_fail=$((struct_fail + 1))
    fi
}

# §54.1 — 두 정지 강제점이 헤더를 거친다
expect_contains "disarmDrive 가 driveDisarm 을 부른다" \
    '^void disarmDrive\(\)' 'driveDisarm(&driveGate, driveSink)'
expect_contains "출력단이 driveOutputAllowed 를 거친다" \
    '^void updateMotorOutputs\(\)' 'driveOutputAllowed(&driveGate'
expect_contains "cmd_vel 이 driveOnCommand 를 거친다" \
    '^void cmdVelCallback' 'driveOnCommand(&driveGate'
expect_contains "서비스가 driveOnServiceRequest 를 거친다" \
    '^void driveEnableCallback' 'driveOnServiceRequest('
expect_contains "phase overrun 이 사유 7 로 풀린다" \
    '^bool recordRuntimePhase' 'disarmDriveWithReason(REARM_DISARM_RUNTIME_OVERRUN)'
expect_contains "publish overrun 이 사유 7 로 풀린다" \
    '^rcl_ret_t publishMeasured' 'disarmDriveWithReason(REARM_DISARM_RUNTIME_OVERRUN)'
expect_contains "spin 응답 실패가 사유 8 로 풀린다" \
    '^void loop\(\)' 'disarmDriveWithReason(REARM_DISARM_SPIN_RESPONSE)'
expect_absent "loop 안에 사유 없는 해제가 없다" \
    '^void loop\(\)' 'disarmDrive();'

# §55.1 — 장벽 시계가 콜백 안에서 시작하지 않는다
expect_absent "서비스 콜백에 시각이 없다" '^void driveEnableCallback' 'millis()'
expect_contains "loop 이 장벽 시계를 시작한다" \
    '^void loop\(\)' 'rearmGateArmBarrierStart(&driveGate, millis())'

# §55.1 — 그 시작이 spin **뒤**여야 한다 (앞이면 응답 전이라 무의미하다)
struct_checks=$((struct_checks + 1))
spin_line="$(fn_body '^void loop\(\)' | grep -n 'rclc_executor_spin_some' | head -1 | cut -d: -f1)"
start_line="$(fn_body '^void loop\(\)' | grep -n 'rearmGateArmBarrierStart' | head -1 | cut -d: -f1)"
if [ -z "$spin_line" ] || [ -z "$start_line" ] || [ "$start_line" -le "$spin_line" ]; then
    echo "  FAIL 장벽 시계가 spin_some 뒤에 있지 않다 (spin=${spin_line:-없음} start=${start_line:-없음})"
    struct_fail=$((struct_fail + 1))
fi

echo "구조 검사 ${struct_checks}건 · 실패 ${struct_fail}건"

if [ "$behavior_rc" -ne 0 ] || [ "$struct_fail" -ne 0 ]; then
    echo "FAIL 굽지 않는다." >&2
    exit 1
fi

echo "OK   동작 + 구조 전량 통과."
exit 0
