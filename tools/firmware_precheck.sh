#!/usr/bin/env bash
# firmware_precheck.sh — 굽기 직전, **펌웨어 소스가 우리가 아는 그 상태인지**를 종료코드로 판정한다.
#
# 사용:  bash tools/firmware_precheck.sh [--repo PATH] [--baseline REV] [--expect P=A,D,내용SHA256]...
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
# ── 판정은 **내용**을 잰다, git 의 렌더링이 아니다 (08-07 검토 §50.1 P1) ──────
# 직전 판까지 기대 지문은 `git diff … | sha256sum` — 즉 **diff 출력 텍스트**의 해시였다.
# 그 텍스트에는 내용 말고 **이 기계의 git 설정**이 함께 들어간다. 작업 트리를 한 바이트도
# 안 건드리고 `core.abbrev` · `diff.context` · `diff.noprefix` · `diff.mnemonicPrefix` 중
# 아무거나 주면 `rc=1` **"오염"** 이 떴다. 게다가 `index c7cfbd4..764db98` 의 축약 자릿수는
# `core.abbrev=auto` 가 객체 수에서 뽑으므로, **객체가 늘어나는 것만으로 지문이 스스로 만료**된다.
#   → 굽기 직전 유일한 게이트가 진단 불가능하게 멈추면 사람은 게이트를 건너뛴다. 이것은
#     §46.2 로 닫은 "늘 울리는 경보" 클래스의 재발이었다.
#   → 그래서 판정 입력에서 **diff 출력 텍스트를 통째로 뺐다.** 기대 파일의 판정은 작업 트리
#     **파일 내용의 sha256** 하나다. 렌더링 설정에 원리적으로 면역이고, `VENDOR_DROP §2` 가
#     원래 주장하던 "허용 **내용** 일치"와 재는 것이 같아진다.
#   ⚠ **느슨해진 게 아니라 강해졌다 — 논증을 정확히 쓴다.** 내용 해시가 맞으면 작업 트리
#     파일의 **바이트가 기대값으로 완전히 결정**된다. 파일에서 파생되는 모든 성질(증감 포함)이
#     따라서 결정되므로, 파일에 대해 증감이 추가로 말해 줄 수 있는 것은 없다. 반면 patch 해시는
#     hunk 만 고정하면서 **환경까지 같이 고정**했다. 빠진 것은 파일에 대한 제약이 아니라
#     **환경에 대한 제약**이고, 그건 애초에 판정할 대상이 아니었다.
#   ⚠ 정확히 하자면 증감은 `(기준점 blob, 파일 내용, diff 알고리즘)` 의 함수다 — 앞의 둘이
#     고정돼도 **알고리즘이 바뀌면 값이 달라질 수 있다.** 그래서 증감은 판정이 아니라 진단이다.
#   → 그래서 증감(`8,2`)은 **진단 출력으로만** 남긴다.
#
# ── 이 검사가 증명하지 않는 것 (숨기지 않는다) ───────────────────────────────
# - **보드에 올라가 있는 펌웨어**가 이 소스라는 증거가 아니다. 여긴 저장소만 본다.
#   보드 쪽 대조 수단은 지금 없다 (`/firmware/info` 는 v1_3 시절 고정 문자열이라 못 쓴다).
# - 기대 내용(`--expect`)은 **사람이 관리하는 계약**이다. `.ino` 를 정당하게 고치면
#   `docs/FIRMWARE_REBUILD.md §4` 의 값과 여기 기본값을 **같이** 옮겨야 한다.
#   그 값은 `sha256sum <파일>` 한 줄로 언제든 다시 만들 수 있다 — 정본에 64자리 전문이 있다.
# - 🔴 **허용 파일의 판정은 이제 `--baseline` 과 무관하다** (숨기지 않는다). 기준점은
#   *기대 밖 변경*을 찾는 데만 쓰인다. 즉 기준점을 잘못 주면 "허용 파일은 맞다"는 여전히
#   참이지만 **그 기준점 이전에 들어온 다른 변경은 안 보인다.** 기본값을 바꿔 부를 이유는 없다.
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

# 기대 목록의 기본값 = 2026-08-17 §75 조건부 승인 뒤 이관한 **스케치 소스 다섯 개**.
#   ① `.ino` — `ESTOP_ACTIVE_LOW true→false`(`.ino:111`, 되돌리면 `ELECTRICAL_BASELINE §2`-⑧ 재개방)
#      + re-arm 래치 배선(§54→§55 보완).
#   ② `rearm_gate.h` — 상태전이 정본. ③ `drive_wiring.h` — 모터 정지의 관측 가능한 자리.
#   ④ `estop_debounce.h` — E-stop 디바운스. ⑤ `runtime_guard.h` — runtime 계측·복귀 후 fail-closed.
#   🔴 아래 64자리는 **파일 내용의 sha256** 이다(patch 가 아니다). 정본 `docs/FIRMWARE_REBUILD.md §4`
#   가 같은 값을 64자리 전문으로 갖고 있고, 재생성은 그쪽에 적힌 `sha256sum <파일>` 한 줄이다.
#   스케치 소스를 정당하게 고치면 **두 자리를 같이** 옮긴다.
#   ⚠ 이 기본값은 검토 §56 조건부 수용까지 받은 내용이다 — 승인 없이 갱신하면 지문이 새 내용을
#   스스로 승인한다(`REAL_ROBOT_VALUES §1-f` ⓷).
#   🔴 **2026-08-12 에 실제로 그 순서를 뒤집었다** (검토 §60.2 P1). 예약 32 의 행동 변경과
#   같은 커밋에서 이 기본값과 정본 지문을 함께 옮겨, **독립 검토 전인데 `rc=0`** 이 나왔다.
#   유일한 자동 굽기 차단이 미승인 구동 상수를 "굽어도 된다"로 통과시킨 것이다 — 되돌렸다.
#   → 순서는 `코드 수정 → 빌드 → 구판 지문으로 rc=1 → 독립 검토 → **지문 전용 별도 커밋**`.
#   → 지문은 오염 검출기이지 **의미 승인자가 아니다.**
#   ✅ **2026-08-14 — 그 순서대로 다시 옮겼다.** 예약 32-e(구름 반지름 `0.05698` · base
#   `0.829` · CONTROL_* 삭제)를 08-13 밤에 굽고 **구판 지문으로 `rc=1`** 인 채 검토 네 회차를
#   태웠다 — §68 → §69 → §70 → **§71(P0 0)**. 승인 뒤 이 커밋에서만 두 자리를 옮겼다.
#   🔴 §71 은 P1 을 **조건부 수용으로 열어 둔 채** 승인했다(`MASTER_PLAN §7` 32-e ⓖ-2).
#   ⚠ 아래 구판 서술(2026-08-12)은 같은 순서를 처음 밟은 기록으로 남긴다.
#   ✅ **2026-08-12 — 그 순서대로 옮긴 값이 당시의 `.ino` 기본값이었다.** 예약 32 계수
#   `FEEDFORWARD_PWM_PER_MPS_ABOVE_MIN 375→335` 이 3회차 독립 검토
#   (`~/Desktop/개발현황/CODEX 현황/0812검토현황.md §62`)에서
#   **P0 0 · 조건부 수용**을 받은 뒤, 그 판정이 지정한 blob(`47661a8f…`)을 이 커밋에서만 옮겼다.
#   🔴 §62 는 P1 1 · P2 2 를 **열어 둔 채** 승인했다 — `rc=0` 은 "굽어도 되는 상태"지
#   "결함이 없다"가 아니다. 재개방 조건은 `MASTER_PLAN §7` 예약 32-a.
if [ ${#EXPECT_ARGS[@]} -eq 0 ]; then
    EXPECT_ARGS=(
        "firmware/teensy_integrated_base_v1_4/teensy_integrated_base_v1_4.ino=569,44,819180ebc9c27bec6cc41befe606f952174097914c7903d54f8afe5cc0faf872"
        "firmware/teensy_integrated_base_v1_4/rearm_gate.h=308,0,abff1f7b292f766540854b3c6a8493525f5494f3ff177dd290ed74a3aa77eea3"
        "firmware/teensy_integrated_base_v1_4/drive_wiring.h=114,0,f34ba116fbd94a317362754dd1fc846a39ca76a387cd9d1e7a9d43783e08b860"
        "firmware/teensy_integrated_base_v1_4/estop_debounce.h=140,0,126fc729074cbcca170c93c93514c5bddd4545e67d2044d1bbd5734f92380940"
        "firmware/teensy_integrated_base_v1_4/runtime_guard.h=127,0,19332991f027569c48e5231d707e1592efb83f5e0a7b584025e74384da539f01"
    )
fi

# 이 설치본의 실제 권위 = arduino-cli 1.5.2-rc.1 + teensy:avr 1.58.2, Teensy 4.1의
# compile_commands.json 전수 관측(검토 §49.1). 공식 명세보다 실제 toolchain이 넓다.
# - sketch root: .ino/.pde/.c/.cc/.cpp/.cxx/.S + header 5종
# - sketch/src/**: .c/.cc/.cpp/.cxx/.S + header 5종을 재귀 컴파일.
# - data/**: 컴파일하지 않는다.
# 목록과 구현이 갈라지지 않게 이 함수 하나가 tracked diff·미추적 파일·기대값 검증의 공통 분류기다.
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
                        *.c|*.cc|*.cpp|*.cxx|*.S|*.h|*.hpp|*.hh|*.tpp|*.ipp) return 0 ;;
                    esac
                    ;;
            esac
            ;;
        *)
            case "$within" in
                *.ino|*.pde|*.c|*.cc|*.cpp|*.cxx|*.S|*.h|*.hpp|*.hh|*.tpp|*.ipp) return 0 ;;
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
SYMLINK_Z="$TMP/symlink.z"

# 먼저 firmware/ 전량을 Git에서 받고, 아래 공통 분류기로 소스/참고를 나눈다. pathspec을 확장자별로
# 손으로 만들지 않으므로 root 목록과 src/** 재귀 목록이 서로 갈라질 자리가 없다.
if ! git -C "$REPO" diff --numstat --no-renames -z "$BASELINE_OID" -- firmware/ >"$DIFF_Z"; then
    echo "FAIL 기준점→작업 트리 diff를 읽지 못했다" >&2; exit 2
fi
if ! git -C "$REPO" ls-files --others -z -- firmware/ >"$UNTRACKED_Z"; then
    echo "FAIL 미추적 firmware 파일 목록을 읽지 못했다" >&2; exit 2
fi
# Arduino CLI는 디렉터리 symlink를 따라 Git이 열거하지 않은 외부 소스까지 컴파일할 수 있다.
# 링크 목적지를 추정해 일부만 허용하지 않고, 스케치 트리 안 symlink 전부를 fail-closed로 막는다.
if ! find "$REPO/firmware" -mindepth 2 -type l -print0 >"$SYMLINK_Z"; then
    echo "FAIL firmware symlink 목록을 읽지 못했다" >&2; exit 2
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

declare -A EXPECT_COUNT=()
declare -A EXPECT_HASH=()
for spec in "${EXPECT_ARGS[@]}"; do
    case "$spec" in
        *=*,*,*) : ;;
        *) echo "FAIL --expect 형식은 '경로=추가,삭제,내용-SHA256' 이다: $spec" >&2; exit 2 ;;
    esac
    path="${spec%%=*}"; want="${spec#*=}"
    if ! is_sketch_source "$path"; then
        echo "FAIL --expect 경로가 Arduino 스케치 소스 범위가 아니다: $path" >&2; exit 2
    fi
    add="${want%%,*}"; rest="${want#*,}"; del="${rest%%,*}"; digest="${rest#*,}"
    # 증감은 **진단용**이지만 형식은 계속 검증한다 — 계약 문자열이 깨진 채로 굽는 것도 판정 불능이다.
    case "$add,$del" in
        *[!0-9,]*|,*|*,|'')
            echo "FAIL --expect 증감은 음이 아닌 정수 두 개여야 한다: $spec" >&2; exit 2 ;;
    esac
    case "$digest" in
        *[!0-9a-f]*|'') echo "FAIL --expect 내용 SHA256은 소문자 64자리여야 한다: $spec" >&2; exit 2 ;;
    esac
    if [ "${#digest}" -ne 64 ]; then
        echo "FAIL --expect 내용 SHA256은 소문자 64자리여야 한다: $spec" >&2; exit 2
    fi
    if [ -n "${EXPECT_COUNT[$path]+x}" ]; then
        echo "FAIL --expect 경로가 중복됐다: $path" >&2; exit 2
    fi
    EXPECT_COUNT["$path"]="$add,$del"
    EXPECT_HASH["$path"]="$digest"
done

echo "=== 펌웨어 사전검사 — 굽기 전 오염 판정 ==="
echo "  저장소   : $REPO"
echo "  기준점   : $BASELINE → **작업 트리**(커밋·staged·unstaged 전부. HEAD 로 끊지 않는다)"
echo "  판정 범위: Teensy 실제 컴파일 root + sketch/src/** 재귀 소스 (ignore 여부 무관)"
echo "  판정 근거: 허용 파일은 **내용 sha256**(git 렌더링 설정에 면역) · 그 밖은 존재 여부"
echo

FAIL=0
echo "--- 기대한 변경 ---"
# 판정은 **작업 트리 파일 내용의 sha256** 하나다. 증감은 아래에서 진단으로만 찍는다 —
# git 의 diff 알고리즘이 정하는 값을 판정에 쓰면 §50.1 의 거짓 양성이 그대로 돌아온다.
for path in "${!EXPECT_HASH[@]}"; do
    want="${EXPECT_COUNT[$path]}"
    want_hash="${EXPECT_HASH[$path]}"
    got="${ACTUAL[$path]-}"
    file="$REPO/$path"

    if [ -L "$file" ]; then
        echo "  🔴 symlink   $path  (기대 파일이 symlink 다 — 내용을 저장소 밖에서 끌어온다)"
        FAIL=1; continue
    fi
    if [ ! -e "$file" ]; then
        echo "  🔴 사라졌다 $path  (삭제·이름변경 — 있어야 할 파일이 없다)"
        echo "     기대 내용 sha256 $want_hash"
        FAIL=1; continue
    fi
    if [ ! -f "$file" ]; then
        echo "  🔴 정규 파일이 아니다 $path"
        FAIL=1; continue
    fi
    got_hash="$(sha256sum "$file" 2>/dev/null | awk '{print $1}')"
    if [ "${#got_hash}" -ne 64 ]; then
        echo "FAIL 기대 파일의 내용을 읽지 못했다: $path" >&2; exit 2
    fi

    if [ "$got_hash" != "$want_hash" ]; then
        if [ -z "$got" ]; then
            echo "  🔴 되돌려졌다 $path  (기준점 대비 변경이 없다 — 허용된 수정이 사라졌다)"
        else
            echo "  🔴 내용 불일치 $path  (증감이 ${got} 로 같아 보여도 내용이 다르면 오염이다)"
        fi
        echo "     기대 내용 sha256 $want_hash"
        echo "     실제 내용 sha256 $got_hash"
        FAIL=1
        continue
    fi

    echo "  ok     $path  (내용 sha256 $got_hash)"
    if [ -z "$got" ]; then
        echo "         진단: 기준점 대비 증감 없음 (기대 ${want})"
    elif [ "$got" != "$want" ]; then
        echo "         진단: 증감이 기대 ${want} 와 다르다 (실제 ${got}) — **판정은 내용이 했다.**"
        echo "               증감은 diff 알고리즘이 정하는 값이라 판정에 쓰지 않는다."
    else
        echo "         진단: 기준점 대비 증감 ${got}"
    fi
done

echo "--- 기대 밖의 소스 변경 ---"
FOUND_EXTRA=0
for path in "${!ACTUAL[@]}"; do
    if [ -z "${EXPECT_COUNT[$path]-}" ]; then
        echo "  🔴 기대에 없는 변경  $path  (${ACTUAL[$path]})"
        FAIL=1; FOUND_EXTRA=1
    fi
done
for path in "${UNTRACKED[@]}"; do
    echo "  🔴 미추적 소스        $path  (git diff 엔 안 보이지만 **함께 컴파일된다**)"
    FAIL=1; FOUND_EXTRA=1
done
while IFS= read -r -d '' link; do
    rel="${link#"$REPO"/}"
    echo "  🔴 symlink 빌드 경계 이탈  $rel  (Git 밖 소스를 따라갈 수 있어 허용하지 않는다)"
    FAIL=1; FOUND_EXTRA=1
done <"$SYMLINK_Z"
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
    echo "     정당한 변경이었다면 docs/FIRMWARE_REBUILD.md §4 의 기대 **내용 sha256**(64자리)을"
    echo "     \`sha256sum <파일>\` 로 다시 만들어 정본과 이 스크립트 기본값 **두 자리**를 같이 옮긴다."
    exit 1
fi

echo "OK   펌웨어 소스 = 기준점 + 허용된 내용 ${#EXPECT_COUNT[@]}건 그대로. 굽어도 된다."
echo "     ⚠ 이것은 **저장소**가 깨끗하다는 뜻이다 — 보드에 올라가 있는 펌웨어의 증거가 아니다."
exit 0
