#!/usr/bin/env bash
# firmware_precheck.sh — 굽기 직전, **펌웨어 소스가 우리가 아는 그 상태인지**를 종료코드로 판정한다.
#
# 사용:  bash tools/firmware_precheck.sh [--repo PATH] [--baseline REV] [--expect P=A,D]...
# 출력:  판정 근거 여러 줄 + 마지막에 `OK …` / `FAIL …` 한 줄.
# 종료:  0 = 허용된 변경 그대로 / 1 = 오염(굽지 않는다) / 2 = **판정 불능**(사용법·저장소·기준점 오류)
#        🔴 2 를 0 처럼 읽지 않는다 — "못 봤다"와 "봤는데 깨끗하다"는 다른 사실이다.
#
# ── 왜 이 파일이 생겼나 (08-07 검토 §47.1 P1) ──────────────────────────────────
# 구판 정본은 사람이 눈으로 보는 한 줄이었다:
#     git diff --numstat f57d454 HEAD -- 'firmware/*/*.ino'
# `.ino` 에 **커밋하지 않은** 한 줄을 넣고 그대로 실행해도 결과는 여전히 기준값 `8 2` 였다.
# 끝점을 `HEAD` 로 못 박으면 **index 와 작업 트리가 비교에서 통째로 빠지기 때문**이다.
# 즉 "예상 밖 펌웨어 변경이면 업로드 전에 멈춘다"는 유일한 오염 게이트가, 현장에서 가장 흔한
# 상태(고쳤는데 아직 커밋 안 함)를 **조용히 통과**시켰다. `MASTER_PLAN §7` 에 적혀 있던
# "2줄 주입 → 10/3 → 중단" 관측도 그 명령으로는 성립하지 않는다 — 그 관측은 정정됐다.
#   → 끝점을 지운다. 기준점 → **작업 트리** 하나만 판정 입력으로 본다.
#
# ── 눈으로 보는 절차를 왜 그만두나 ───────────────────────────────────────────
# `8 2` 를 눈으로 대조하는 절차는 사람이 "대충 맞네"로 넘길 수 있고, 무엇보다 **다음 세 가지를
# 아예 보여 주지 않았다.** 그래서 판정을 종료코드로 옮긴다:
#   ① **미추적 파일** — `git diff` 는 추적되지 않는 새 파일을 보지 못한다. 그런데
#      `arduino-cli` 는 스케치 폴더의 소스를 함께 컴파일한다. 폴더에 떨궈진 `rogue.ino`
#      한 장은 diff 에 안 보이면서 바이너리에는 들어간다. `.gitignore`·전역 ignore에 걸려도
#      컴파일 입력이라는 사실은 바뀌지 않으므로 `--exclude-standard` 없이 전부 센다.
#   ② **공식 확장자·재귀 경계** — 루트의 `.hh/.tpp/.ipp`도 공식 지원인데 수동 목록에서
#      빠졌고, Git pathspec `*`는 slash까지 잡아 `src/`뿐 아니라 비컴파일 `data/`도 암묵적으로
#      섞었다. 판정 범위는 Arduino CLI sketch specification의 root/`src/**` 규칙이어야 한다.
#   ③ **삭제·이름변경** — 기대 목록과 **양방향으로** 대조해야 "있어야 할 게 없음"도 잡힌다.
#      그래서 `--no-renames` 다. 이름이 바뀐 것은 숨길 일이 아니라 멈출 일이다.
#
# ── 판정에서 일부러 빼는 것 (08-07 검토 §46.2 P2 — 늘 울리는 경보는 무시된다) ──
# `firmware/VENDOR_DROP.md`·`SHA256SUMS.txt` 같은 **소유·기록 문서**는 고쳐도 되는 자리다.
# 구판은 범위를 `-- firmware/` 로 잡아 이 허용된 변경까지 오염으로 셌고, 그래서 정상 작업본에서
# **매번** "멈춰라"가 떴다. 늘 울리는 경보는 사람이 곧 건너뛰고, 그때 진짜 오염이 지나간다.
#   → 문서는 판정에서 빼되 **숨기지 않는다.** 아래 `[참고]` 절에 항상 같이 찍는다.
#
# ── 이 검사가 증명하지 않는 것 (숨기지 않는다) ───────────────────────────────
# - **보드에 올라가 있는 펌웨어**가 이 소스라는 증거가 아니다. 여긴 저장소만 본다.
#   보드 쪽 대조 수단은 지금 없다 (`/firmware/info` 는 v1_3 시절 고정 문자열이라 못 쓴다).
# - 기대 증감(`--expect`)은 **사람이 관리하는 계약**이다. `.ino` 를 정당하게 고치면 이 값도
#   같이 옮겨야 한다 — 옮기지 않으면 FAIL 로 시끄럽게 막힌다(fail-closed, 의도한 방향이다).
# - 외부 라이브러리·보드 core는 이 저장소의 스케치 소유 범위 밖이다. 빌드 환경 지문과 라이브러리
#   해시는 `docs/FIRMWARE_REBUILD.md §2`~`§4`가 별도로 대조한다.
#
# 정본·맥락 = `docs/FIRMWARE_REBUILD.md §4` · 소유 경계 = `firmware/VENDOR_DROP.md §2`

set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

REPO="$(dirname "$HERE")"
BASELINE="f57d454"          # = vendor 수령본을 저장소에 들인 커밋
EXPECT_ARGS=()

while [ $# -gt 0 ]; do
    case "$1" in
        --repo)     REPO="${2:-}";     shift 2 || exit 2 ;;
        --baseline) BASELINE="${2:-}"; shift 2 || exit 2 ;;
        --expect)   EXPECT_ARGS+=("${2:-}"); shift 2 || exit 2 ;;
        -h|--help)  sed -n '2,6p' "${BASH_SOURCE[0]}"; exit 2 ;;
        *) echo "FAIL 모르는 인자: $1  (사용법은 --help)" >&2; exit 2 ;;
    esac
done

# 기대 목록의 기본값 = 2026-08-07 현재 **허용된 변경 딱 한 건**.
#   `ESTOP_ACTIVE_LOW true→false` = `.ino:111` 과 그 위 주석. 되돌리면 `ELECTRICAL_BASELINE §2`-⑧ 이
#   재개방된다. 이 값을 옮길 땐 `docs/FIRMWARE_REBUILD.md §4` 의 숫자도 같이 옮긴다.
if [ ${#EXPECT_ARGS[@]} -eq 0 ]; then
    EXPECT_ARGS=("firmware/teensy_integrated_base_v1_4/teensy_integrated_base_v1_4.ino=8,2")
fi

# Arduino CLI 1.5 sketch specification의 **전수 목록**.
# - sketch root: .ino/.pde/.c/.cpp/.S + header 5종
# - sketch/src/**: 재귀 컴파일. Arduino language(.ino/.pde)는 여기서 지원하지 않는다.
# - data/**: 컴파일하지 않는다.
# 공식 목록에 없는 .cc/.cxx를 임의로 보태지 않는다. 목록과 구현이 갈라지지 않게 이 함수 하나가
# tracked diff·미추적 파일·기대값 검증의 공통 분류기다.
is_sketch_source() {
    local path="$1" rest sketch within
    case "$path" in firmware/*/*) ;; *) return 1 ;; esac
    rest="${path#firmware/}"
    sketch="${rest%%/*}"
    [ -n "$sketch" ] && [ "$rest" != "$sketch" ] || return 1
    within="${rest#*/}"

    case "$within" in
        */*)
            case "$within" in
                src/*)
                    case "$within" in
                        *.c|*.cpp|*.S|*.h|*.hpp|*.hh|*.tpp|*.ipp) return 0 ;;
                    esac
                    ;;
            esac
            ;;
        *)
            case "$within" in
                *.ino|*.pde|*.c|*.cpp|*.S|*.h|*.hpp|*.hh|*.tpp|*.ipp) return 0 ;;
            esac
            ;;
    esac
    return 1
}

# ── 사전 조건: 여기서 못 넘어가면 **판정 불능(2)** 이지 통과가 아니다 ────────────
if [ ! -d "$REPO" ]; then
    echo "FAIL 저장소 경로가 없다: $REPO" >&2; exit 2
fi
if ! git -C "$REPO" rev-parse --git-dir >/dev/null 2>&1; then
    echo "FAIL git 저장소가 아니다: $REPO" >&2; exit 2
fi
BASELINE_OID="$(git -C "$REPO" rev-parse --verify --quiet --end-of-options \
    "${BASELINE}^{commit}" 2>/dev/null)"
if [ -z "$BASELINE_OID" ]; then
    echo "FAIL 기준점 커밋을 못 찾는다: $BASELINE  (얕은 clone 이면 전체 이력이 필요하다)" >&2
    exit 2
fi

TMP="$(mktemp -d -t firmware-precheck.XXXXXX)" || {
    echo "FAIL 임시 디렉터리를 만들지 못했다" >&2; exit 2; }
trap 'rm -rf "$TMP"' EXIT
DIFF_Z="$TMP/diff.z"
UNTRACKED_Z="$TMP/untracked.z"

# 먼저 firmware/ 전량을 Git에서 받고, 아래 공통 분류기로 소스/참고를 나눈다. pathspec을 확장자별로
# 손으로 만들지 않으므로 root 목록과 src/** 재귀 목록이 서로 갈라질 자리가 없다.
if ! git -C "$REPO" diff --numstat --no-renames -z "$BASELINE_OID" -- firmware/ >"$DIFF_Z"; then
    echo "FAIL 기준점→작업 트리 diff를 읽지 못했다" >&2; exit 2
fi
if ! git -C "$REPO" ls-files --others -z -- firmware/ >"$UNTRACKED_Z"; then
    echo "FAIL 미추적 firmware 파일 목록을 읽지 못했다" >&2; exit 2
fi

# ── 판정 입력 ①: 기준점 → **작업 트리** (끝점을 안 준다 = committed+staged+unstaged 전부) ──
declare -A ACTUAL=()
DOC_DIFF=()
while IFS= read -r -d '' rec; do
    [ -n "$rec" ] || continue
    add="${rec%%	*}";  rest="${rec#*	}"
    del="${rest%%	*}"; path="${rest#*	}"
    if is_sketch_source "$path"; then
        ACTUAL["$path"]="$add,$del"
    else
        DOC_DIFF+=("$add" "$del" "$path")
    fi
done <"$DIFF_Z"

# ── 판정 입력 ②: 미추적 소스 (git diff 가 못 보는 자리) ────────────────────────
# ignore 규칙은 저장소 표시 정책이지 Arduino 컴파일 정책이 아니다. 그래서 일부러
# `--exclude-standard`를 쓰지 않는다.
UNTRACKED=()
DOC_NEW=()
while IFS= read -r -d '' path; do
    [ -n "$path" ] || continue
    if is_sketch_source "$path"; then
        UNTRACKED+=("$path")
    else
        DOC_NEW+=("$path")
    fi
done <"$UNTRACKED_Z"

declare -A EXPECT=()
for spec in "${EXPECT_ARGS[@]}"; do
    case "$spec" in
        *=*,*) : ;;
        *) echo "FAIL --expect 형식은 '경로=추가,삭제' 다: $spec" >&2; exit 2 ;;
    esac
    path="${spec%%=*}"; want="${spec#*=}"
    if ! is_sketch_source "$path"; then
        echo "FAIL --expect 경로가 Arduino 스케치 소스 범위가 아니다: $path" >&2; exit 2
    fi
    case "$want" in
        *[!0-9,]*|,*|*,*,*|*','|'')
            echo "FAIL --expect 증감은 음이 아닌 정수 두 개여야 한다: $spec" >&2; exit 2 ;;
    esac
    if [ -n "${EXPECT[$path]+x}" ]; then
        echo "FAIL --expect 경로가 중복됐다: $path" >&2; exit 2
    fi
    EXPECT["$path"]="$want"
done

echo "=== 펌웨어 사전검사 — 굽기 전 오염 판정 ==="
echo "  저장소   : $REPO"
echo "  기준점   : $BASELINE → **작업 트리**(커밋·staged·unstaged 전부. HEAD 로 끊지 않는다)"
echo "  판정 범위: sketch root 지원 확장자 + sketch/src/** 재귀 소스 (ignore 여부 무관)"
echo

FAIL=0
echo "--- 기대한 변경 ---"
for path in "${!EXPECT[@]}"; do
    want="${EXPECT[$path]}"
    got="${ACTUAL[$path]-}"
    if [ -z "$got" ]; then
        echo "  🔴 없음  $path  (기대 ${want} · 실제 변경 없음 — 허용된 수정이 **되돌려졌다**)"
        FAIL=1
    elif [ "$got" != "$want" ]; then
        echo "  🔴 불일치 $path  (기대 ${want} · 실제 ${got})"
        FAIL=1
    else
        echo "  ok     $path  (${got})"
    fi
done

echo "--- 기대 밖의 소스 변경 ---"
FOUND_EXTRA=0
for path in "${!ACTUAL[@]}"; do
    if [ -z "${EXPECT[$path]-}" ]; then
        echo "  🔴 기대에 없는 변경  $path  (${ACTUAL[$path]})"
        FAIL=1; FOUND_EXTRA=1
    fi
done
for path in "${UNTRACKED[@]}"; do
    echo "  🔴 미추적 소스        $path  (git diff 엔 안 보이지만 **함께 컴파일된다**)"
    FAIL=1; FOUND_EXTRA=1
done
[ "$FOUND_EXTRA" -eq 0 ] && echo "  ok     없음"

echo
echo "--- [참고] 판정에 넣지 않은 firmware/ 파일 (소유 문서·data 등 빌드 밖) ---"
if [ ${#DOC_DIFF[@]} -eq 0 ] && [ ${#DOC_NEW[@]} -eq 0 ]; then
    echo "  (없음)"
else
    for ((i=0; i<${#DOC_DIFF[@]}; i+=3)); do
        printf '  %s\t%s\t%s\n' "${DOC_DIFF[i]}" "${DOC_DIFF[i+1]}" "${DOC_DIFF[i+2]}"
    done
    for path in "${DOC_NEW[@]}"; do
        printf '  (미추적) %s\n' "$path"
    done
    echo "  ※ 이 줄들은 판정에 **영향을 주지 않는다** — 여기까지 오염으로 세면 경보가 늘 울린다."
fi
echo

if [ "$FAIL" -ne 0 ]; then
    echo "FAIL 펌웨어 소스가 우리가 아는 상태가 아니다 — **compile·upload 전에 멈추고 사유를 찾는다.**"
    echo "     정당한 변경이었다면 커밋한 뒤 docs/FIRMWARE_REBUILD.md §4 의 기대 증감도 같이 옮긴다."
    exit 1
fi

echo "OK   펌웨어 소스 = 기준점 + 허용된 변경 ${#EXPECT[@]}건 그대로. 굽어도 된다."
echo "     ⚠ 이것은 **저장소**가 깨끗하다는 뜻이다 — 보드에 올라가 있는 펌웨어의 증거가 아니다."
exit 0
