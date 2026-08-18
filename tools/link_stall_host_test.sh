#!/usr/bin/env bash
# link_stall_host_test.sh — 예약 41-g 사건 계측을 PC 에서 주입 검사한다.
#
# 사용:  bash tools/link_stall_host_test.sh
# 종료:  0 = 계약 전량 통과 / 1 = 계약 위반 / 2 = 판정 불능(컴파일러·경로)
#        🔴 2 를 0 처럼 읽지 않는다 — "못 돌렸다"와 "돌렸는데 통과"는 다른 사실이다.
#
# 왜 이게 필요한가 (계획 §4 · 검토 §78.4):
#   41-g 완료판정은 **합성 주입이 서로 다른 분류로 갈릴 때**다. 자연 재현 0건은
#   R3 관측이지 41-g 종결이 아니다. 실기로는 loop **안** 300ms 와 판 **사이**
#   300ms 를 따로 만들 수 없으므로 가짜 시계가 유일한 경로다.
#
# 세 단계인 이유:
#   1단계 **MCU 동작 검사** — link_stall_probe.h 의 결정을 가짜 시계로 관측한다.
#   2단계 **호스트 분류 검사** — 주입마다 수신 스트림을 뱉고, 9행 분류표의 유일한
#       구현인 link_stall_classify.py 가 서로 **다른** 분류를 내는지 본다.
#       🔴 여기가 완료선이다. 1단계만 초록인 것은 "사건이 났다" 까지다.
#   3단계 **구조 검사** — 스케치가 그 함수들을 실제로 부르는지 텍스트로 본다.
#       약한 증거다(호출 여부만 보지 결과를 안 본다). 약한 줄 알고 쓴다.
#
# ⚠ 이 검사가 증명하지 않는 것: 실제 USB/agent 가 300ms 서는가(=B3 실기 bag),
#    복귀하지 않는 영구 정지의 원인(=§79.2 가 공개한 한계).

set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$HERE")"
SKETCH="$REPO/firmware/teensy_integrated_base_v1_4"
SRC="$HERE/link_stall_host_test.cpp"
CLASSIFY="$HERE/link_stall_classify.py"
INO="$SKETCH/teensy_integrated_base_v1_4.ino"

for f in "$SRC" "$CLASSIFY" "$INO" "$SKETCH/link_stall_probe.h" \
         "$SKETCH/runtime_guard.h"; do
    if [ ! -f "$f" ]; then
        echo "FAIL 입력 파일이 없다: $f" >&2; exit 2
    fi
done

CXX="${CXX:-g++}"
if ! command -v "$CXX" >/dev/null 2>&1; then
    echo "FAIL C++ 컴파일러가 없다: $CXX  (sudo apt install g++)" >&2; exit 2
fi
PY="${PYTHON:-python3}"
if ! command -v "$PY" >/dev/null 2>&1; then
    echo "FAIL python3 이 없다: $PY" >&2; exit 2
fi

TMP="$(mktemp -d -t link-stall-test.XXXXXX)" || {
    echo "FAIL 임시 디렉터리를 만들지 못했다" >&2; exit 2; }
trap 'rm -rf "$TMP"' EXIT
BIN="$TMP/link_stall_host_test"
STREAMS="$TMP/streams"
mkdir -p "$STREAMS" || { echo "FAIL 스트림 디렉터리를 못 만든다" >&2; exit 2; }

# ── 1단계: MCU 동작 검사 ────────────────────────────────────────────────────
# -Wall -Wextra -Werror: 헤더가 Teensy 쪽 컴파일러에서만 조용한 상태로 남지 않게 한다.
if ! "$CXX" -std=c++17 -O1 -Wall -Wextra -Werror -o "$BIN" "$SRC" 2>"$TMP/cc.log"; then
    echo "FAIL harness 컴파일 실패:" >&2
    cat "$TMP/cc.log" >&2
    exit 2
fi

"$BIN" "$STREAMS"
behavior_rc=$?
if [ "$behavior_rc" -gt 1 ]; then
    echo "FAIL harness 실행이 비정상 종료했다 (rc=$behavior_rc)" >&2
    exit 2
fi

# ── 2단계: 호스트 분류 검사 ─────────────────────────────────────────────────
echo
echo "=== 분류 검사: 주입이 서로 다른 분류로 갈리는가 (41-g 완료판정) ==="

class_checks=0
class_fail=0
declare -A seen_class

shopt -s nullglob
metas=("$STREAMS"/*.meta)
if [ "${#metas[@]}" -eq 0 ]; then
    echo "FAIL 주입 스트림이 하나도 없다 — harness 가 아무것도 안 뱉었다" >&2
    exit 2
fi

for meta in "${metas[@]}"; do
    name="$(basename "$meta" .meta)"
    GAP_START=""; GAP_END=""; EXPECT=""; EXPECT_GROUPS=""
    # shellcheck disable=SC1090
    . "$meta"
    args=(--gap-start "$GAP_START" --gap-end "$GAP_END" --expect "$EXPECT")
    if [ "$EXPECT_GROUPS" != "-1" ]; then
        args+=(--expect-groups "$EXPECT_GROUPS")
    fi

    class_checks=$((class_checks + 1))
    out="$("$PY" "$CLASSIFY" "$STREAMS/$name.jsonl" "${args[@]}" 2>&1)"
    rc=$?
    if [ "$rc" -eq 0 ]; then
        printf '  OK   %-20s → %s\n' "$name" "$EXPECT"
    else
        printf '  FAIL %-20s (기대 %s)\n' "$name" "$EXPECT"
        echo "$out" | sed 's/^/       /'
        class_fail=$((class_fail + 1))
    fi
    seen_class["$EXPECT"]=1
done

# 🔴 "전부 같은 분류로 통과" 를 성공으로 읽지 않는다. 41-g 는 **갈리는 것**이
#    완료판정이므로, 서로 다른 분류가 최소 6종은 나와야 한다
#    (LOOP_INTERNAL · BETWEEN_LOOPS · COMPOUND · PUBLISH_LAYER · HOST_AFTER ·
#     RUN_ENDED · MCU_RESET · UNDECIDABLE_*).
class_checks=$((class_checks + 1))
distinct="${#seen_class[@]}"
if [ "$distinct" -lt 6 ]; then
    echo "  FAIL 서로 다른 분류가 ${distinct}종뿐이다 — 갈리지 않으면 41-g 는 미완이다"
    class_fail=$((class_fail + 1))
else
    echo "  OK   서로 다른 분류 ${distinct}종"
fi

for must in LOOP_INTERNAL BETWEEN_LOOPS COMPOUND PUBLISH_LAYER HOST_AFTER \
            RUN_ENDED MCU_RESET UNDECIDABLE_INSTRUMENT; do
    class_checks=$((class_checks + 1))
    if [ -z "${seen_class[$must]:-}" ]; then
        echo "  FAIL 분류 $must 를 내는 주입이 없다"
        class_fail=$((class_fail + 1))
    fi
done

echo "분류 검사 ${class_checks}건 · 실패 ${class_fail}건"

# ── 3단계: 구조 검사 — 스케치가 헤더를 부르는가 ─────────────────────────────
echo
echo "=== 구조 검사: .ino 가 link_stall_probe.h 를 실제로 부르는가 ==="

struct_checks=0
struct_fail=0

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

expect_before() {     # $1 설명 · $2 함수 regex · $3 앞에 와야 할 것 · $4 뒤
    struct_checks=$((struct_checks + 1))
    local a b
    a="$(fn_body "$2" | grep -n -- "$3" | head -1 | cut -d: -f1)"
    b="$(fn_body "$2" | grep -n -- "$4" | head -1 | cut -d: -f1)"
    if [ -z "$a" ] || [ -z "$b" ] || [ "$a" -ge "$b" ]; then
        echo "  FAIL $1 (앞=${a:-없음} 뒤=${b:-없음})"
        struct_fail=$((struct_fail + 1))
    fi
}

# 시간을 두 조각으로 재는 두 자리가 loop 에 다 있는가
expect_contains "loop 이 판 시작을 찍는다" '^void loop\(\)' 'linkProbeLoopBegin(&linkProbe'
expect_contains "loop 이 판 끝을 찍는다"   '^void loop\(\)' 'linkProbeLoopEnd(&linkProbe'

# 🔴 delay(1) 은 판 **사이**다. loopEnd 가 delay 뒤로 가면 delay 가 exec 에 들어가
#    "loop 안" 으로 오분류된다 — §74.6 이 지적한 바로 그 자리의 재발이다.
expect_before "판 끝이 delay(1) 앞에 있다" '^void loop\(\)' \
    'linkProbeLoopEnd(&linkProbe' 'delay(1);'

# 🔴 판 시작이 안전 점검보다 앞이어야 판 사이가 전부 잡힌다
expect_before "판 시작이 checkSafety 앞에 있다" '^void loop\(\)' \
    'linkProbeLoopBegin(&linkProbe' 'checkSafety();'

# 모든 측정 발행이 사건 계측을 거친다
expect_contains "publishMeasured 가 발행 결과를 계측에 넘긴다" \
    '^rcl_ret_t publishMeasured' 'linkProbePublishResult(&linkProbe'

# 🔴 TIME_SYNC 반환값을 버리지 않는다 (계약 3판 ③)
expect_absent "동기 반환값을 버리지 않는다" '^void periodicTimeSync' \
    '(void)rmw_uros_sync_session'
expect_contains "동기 결과를 계측에 넘긴다" '^void periodicTimeSync' \
    'linkProbeSyncResult(&linkProbe'

# 생존 표본이 주기로 나간다
expect_contains "loop 이 생존 표본을 낸다" '^void loop\(\)' 'publishFirmwarePulse()'
expect_contains "생존 표본이 publishMeasured 를 거친다" \
    '^void publishFirmwarePulse' 'RUNTIME_PUBLISH_PULSE'
expect_contains "생존 표본 발행 결과를 계측에 넘긴다" \
    '^void publishFirmwarePulse' 'linkProbePulseSent(&linkProbe'

# 사건이 배출된다
expect_contains "loop 이 사건을 배출한다" '^void loop\(\)' 'drainLinkEvents()'
expect_contains "사건 배출이 ring 을 꺼낸다" '^void drainLinkEvents' \
    'linkProbeDrain(&linkProbe'

# 🔴 계측이 차량을 세우지 않는다 — 자동 해제 5사유 불변
expect_absent "생존 표본이 해제를 부르지 않는다" '^void publishFirmwarePulse' \
    'disarmDrive'
expect_absent "사건 배출이 해제를 부르지 않는다" '^void drainLinkEvents' \
    'disarmDrive'

echo "구조 검사 ${struct_checks}건 · 실패 ${struct_fail}건"

if [ "$behavior_rc" -ne 0 ] || [ "$class_fail" -ne 0 ] || [ "$struct_fail" -ne 0 ]; then
    echo "FAIL 굽지 않는다." >&2
    exit 1
fi

echo "OK   MCU 동작 + 호스트 분류 + 구조 전량 통과."
exit 0
