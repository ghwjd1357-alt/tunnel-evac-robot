#!/usr/bin/env bash
# doc_check.sh — 문서 동기화 자동 검사 (07-20 신설, 같은 날 Codex 검토 반영 개정)
#
# 목적: "문서는 다음 세션의 입력"이므로 낡은 문서 = 실행 버그다.
#       사람이 기억으로 지키던 동기화를 기계가 대신 검사한다.
#
# 사용:
#   bash tools/doc_check.sh              # 전체 검사 (TEST_GATES §1 게이트 마지막 = 커밋 직전)
#   bash tools/doc_check.sh --after-push # 커밋+push 후 원격 동기만 재확인 (~0.1초)
#   bash tools/doc_check.sh --strict     # 생략(skip)도 실패로 취급
#
# ⚠ 한계: 잡는 것은 '숫자·존재·중복' 같은 기계적 불일치뿐이다.
#    "정책이 서로 모순된다" 같은 의미 수준 불일치는 Codex 독립 검토가 잡는다.
# ⚠ 원격 검사 시점: 이 스크립트를 커밋 직전에 돌리면 HEAD 는 아직 '이전 커밋'이다.
#    새 커밋의 push 누락은 잡을 수 없으므로 **push 후 --after-push 로 한 번 더** 돌린다.

set -uo pipefail
cd "$(dirname "$0")/.." || exit 1

FAIL=0
SKIPPED=0
STRICT=0
MODE="full"
for arg in "$@"; do
    case "$arg" in
        --after-push) MODE="after-push" ;;
        --strict)     STRICT=1 ;;
        *) echo "알 수 없는 인자: $arg"; exit 2 ;;
    esac
done

ok()   { printf '  \033[32mOK\033[0m   %s\n' "$1"; }
bad()  { printf '  \033[31mFAIL\033[0m %s\n' "$1"; FAIL=1; }
skip() { printf '  \033[33m--\033[0m   %s (생략)\n' "$1"; SKIPPED=$((SKIPPED+1));
         [ "$STRICT" = "1" ] && FAIL=1; return 0; }

DESK="$HOME/Desktop/개발현황"

# 참조된 문서명을 실제 경로로 해석 (repo 루트 → docs/ → Desktop 순)
resolve_doc() {
    local p="${1/#\~/$HOME}"
    local c
    for c in "$p" "docs/$p" "$DESK/$p" "$HOME/Desktop/$p"; do
        [ -f "$c" ] && { echo "$c"; return 0; }
    done
    return 1
}

# 문서에 'N' 번 제목이 존재하는가 (## 2. / ### 7.3 / ### 2-B. 등)
heading_exists() {
    local path="$1" num="${2//./\\.}"
    grep -qP "^#{1,6} *\**${num}[.\s\*]" "$path" || grep -qP "^#{1,6} *\**${num}$" "$path"
}

# ── 원격 동기 검사 (ahead/behind/diverged) ───────────────────────────────
remote_check() {
    if ! git rev-parse --abbrev-ref '@{upstream}' >/dev/null 2>&1; then
        skip "원격 동기 검사 — upstream 없음"; return
    fi
    local counts behind ahead
    counts=$(git rev-list --left-right --count '@{upstream}...HEAD' 2>/dev/null) || {
        skip "원격 동기 검사 — rev-list 실패"; return; }
    behind=$(echo "$counts" | cut -f1); ahead=$(echo "$counts" | cut -f2)
    if [ "$ahead" != "0" ] && [ "$behind" != "0" ]; then
        bad "원격과 갈라짐(diverged): ahead $ahead / behind $behind — 정리 후 push"
    elif [ "$ahead" != "0" ]; then
        bad "push 안 된 커밋 $ahead 개 — AGENTS.md §5 '커밋 = push 한 세트'"
    elif [ "$behind" != "0" ]; then
        bad "원격보다 $behind 개 뒤처짐 — pull 후 작업 (다른 기기/세션의 커밋 존재)"
    else
        ok "원격과 동기 (마지막 fetch 기준)"
    fi
}

if [ "$MODE" = "after-push" ]; then
    # 이 모드는 원격 확인이 유일한 목적 → upstream 없음·rev-list 실패도 실패로 본다
    # (확인 못 한 것을 '정상'이라 말하지 않는다).
    echo "=== push 후 원격 동기 확인 ==="
    STRICT=1
    remote_check
    echo
    [ "$FAIL" = "0" ] && echo "✅ 원격 동기 정상" || echo "❌ 위 항목을 정리할 것"
    exit "$FAIL"
fi

echo "=== 문서 동기화 검사 ==="

# ── 1. pytest 기준선: 실제 개수 == TEST_GATES 수치 ────────────────────────
# 낡은 기준선은 회귀 검출력을 조용히 무력화한다(테스트가 사라져도 PASS 로 보임).
ACTUAL=$(python3 -m pytest src/mission_manager/test/ -q 2>/dev/null \
         | grep -oP '\d+(?= passed)' | head -1)
DOC=$(grep -oP 'pytest \*\*\K\d+' docs/TEST_GATES.md | head -1)
if [ -z "$ACTUAL" ]; then
    bad "pytest 실행 실패 — 테스트가 깨졌거나 환경 문제"
elif [ "$ACTUAL" = "$DOC" ]; then
    ok "pytest 기준선 $DOC 개 = 실제"
else
    bad "pytest 기준선 불일치: 문서 $DOC / 실제 $ACTUAL → docs/TEST_GATES.md §1 갱신"
fi

# ── 2. colcon 기준선 (이전 실행 결과가 있을 때만) ────────────────────────
if command -v colcon >/dev/null 2>&1 && [ -d build ]; then
    CA=$(colcon test-result 2>/dev/null | grep -oP '\d+(?= tests)' | tail -1)
    CD=$(grep -oP 'colcon \*\*\K\d+' docs/TEST_GATES.md | head -1)
    if [ -z "$CA" ]; then
        skip "colcon 기준선 — 테스트 결과 없음(colcon test 미실행)"
    elif [ "$CA" = "$CD" ]; then
        ok "colcon 기준선 $CD 개 = 실제"
    else
        bad "colcon 기준선 불일치: 문서 $CD / 실제 $CA → docs/TEST_GATES.md §1 갱신"
    fi
else
    skip "colcon 기준선 — colcon 없음 또는 build/ 없음"
fi

# ── 3. '현재 위치' 단일 출처: MASTER_PLAN 이 현재를 가리키면 안 됨 ────────
# 07-20 실사고 재발 방지 — 현재 단계는 CURRENT_HANDOFF 한 곳에만 존재해야 한다.
if grep -q "◀ *현재" docs/MASTER_PLAN.md; then
    bad "MASTER_PLAN 에 '◀ 현재' 표시 — 현재 위치 정본은 CURRENT_HANDOFF 한 곳뿐"
else
    ok "현재 위치 단일 출처 유지"
fi

# ── 4. CURRENT_HANDOFF 템플릿 필수 절 유지 ───────────────────────────────
MISS=""
for sec in "현재 단계" "이번 한 묶음 목표" "완료조건" "금지 범위" "완료 판정" "근거 문서"; do
    grep -q "$sec" docs/CURRENT_HANDOFF.md || MISS="$MISS '$sec'"
done
[ -z "$MISS" ] && ok "CURRENT_HANDOFF 필수 절 전부 존재" \
               || bad "CURRENT_HANDOFF 필수 절 누락:$MISS"

# ── 5·6. 문서 링크 + § 절 참조 검사 ──────────────────────────────────────
# 참조는 백틱(`…`) 안에 적는 것이 이 저장소의 표기 규칙 → 백틱 단위로 통째 추출한다.
#   구판은 공백 없는 토큰만 봐서 'CODEX 현황/…'(공백)·docs/legacy/…(중첩)를 놓쳤고,
#   산문 표기(`docs/{A, B}.md`, `06xx/…`)를 없는 파일로 오탐했다 (Codex 지적 + 실측).
SRC=(docs/*.md AGENTS.md CLAUDE.md)
BROKEN=""; LINKN=0; SECBAD=""; SECN=0
while IFS= read -r ref; do
    [ -z "$ref" ] && continue
    case "$ref" in *'{'*|*'}'*|*'*'*|*xx*) continue ;; esac   # 산문 자리표시자 제외
    file="${ref%%.md*}.md"
    if ! path=$(resolve_doc "$file"); then
        case "$BROKEN" in *" $file"*) ;; *) BROKEN="$BROKEN $file" ;; esac
        continue
    fi
    LINKN=$((LINKN+1))
    raw=$(printf '%s' "$ref" | grep -oP '§ *\K\S+')
    [ -z "$raw" ] && continue
    raw="${raw%%~*}"                       # 범위 표기(§18.3~18.4)는 앞부분으로 검사
    # 지원 형식: N / N.N / N-X (X = 숫자 또는 영문). 그 외는 조용히 넘기지 않고 FAIL.
    if ! printf '%s' "$raw" | grep -qP '^[0-9]+(\.[0-9]+)*(-[0-9A-Za-z]+)?$'; then
        SECBAD="$SECBAD [$ref ← 지원하지 않는 절 형식]"; continue
    fi
    # §2-B 처럼 하이픈까지가 실제 제목인 경우와, §2-2(=§2 의 2번 항목)처럼
    # 하이픈 앞 번호가 제목인 경우를 모두 인정한다.
    if heading_exists "$path" "$raw" || heading_exists "$path" "${raw%%-*}"; then
        SECN=$((SECN+1))
    else
        SECBAD="$SECBAD [$ref]"
    fi
done < <(grep -rhoP '`[^`]+`' "${SRC[@]}" 2>/dev/null | tr -d '`' \
         | grep -P '\.md( *§ *\S+)? *$' | sort -u)

[ -z "$BROKEN" ] && ok "문서 링크 전부 유효 ($LINKN 건)" || bad "없는 문서 참조:$BROKEN"
# 07-20 실사고: CURRENT_HANDOFF 가 repo MASTER_PLAN 에 없는 '§7.3' 을 가리켰다.
[ -z "$SECBAD" ] && ok "§ 절 참조 전부 유효 ($SECN 건)" \
                 || bad "대상 문서에 없는 절 참조:$SECBAD"

# ── 7. Desktop 역사 문서 참조 (이름 변경·이동 감지) ──────────────────────
MISSD=""
for f in $(grep -rhoP '[0-9]{4}_[가-힣A-Za-z_]+\.md' "${SRC[@]}" 2>/dev/null | sort -u); do
    [ -f "$DESK/$f" ] || MISSD="$MISSD $f"
done
[ -z "$MISSD" ] && ok "Desktop 역사 문서 참조 유효" || bad "Desktop 에 없는 문서:$MISSD"

# ── 9. 영구 증거 문서가 '가변 핸드오프의 내용'을 근거로 삼지 않는가 ──────
# CURRENT_HANDOFF 는 묶음마다 통째로 교체된다. 영구 증거(동결 manifest)가 그 파일의
# 완료조건·금지 범위를 인용하면, 교체되는 순간 같은 문장이 조용히 다른 뜻이 된다.
#   07-24 실사고(Codex 0723검토 §10 P2): manifest 가 "CURRENT_HANDOFF 완료조건 6"을
#   태그 순서의 근거로 걸어 뒀는데, 같은 커밋이 핸드오프를 교체해 그 번호가 무관한
#   항목이 됐다. 위 5·6번 검사는 링크와 절의 '존재'만 보므로 이 의미 드리프트를 PASS 했다.
# 허용 = ① 커밋 고정 형태(git show <hash>:docs/CURRENT_HANDOFF.md)
#        ② 내용 인용 없이 파일명만 언급 (구조 설명 등)
# ⚠ 한계: 검출 대상은 docs/*MANIFEST*.md 다 — 새 영구 증거 문서는 이 이름 규칙을 따를 것.
HSEC='완료조건|금지 범위|허용 파일|완료 판정|이번 한 묶음|현재 단계'
DRIFT=""
for f in docs/*MANIFEST*.md; do
    [ -f "$f" ] || continue
    while IFS= read -r hit; do
        txt="${hit#*:}"
        # 커밋을 고정한 참조는 불변이므로 통과
        printf '%s' "$txt" | grep -qP 'git show +[0-9a-f]{7,40}:' && continue
        printf '%s' "$txt" \
            | grep -qP "(CURRENT_HANDOFF.*($HSEC))|(($HSEC).*CURRENT_HANDOFF)" \
            && DRIFT="$DRIFT $f:${hit%%:*}"
    done < <(grep -n 'CURRENT_HANDOFF' "$f")
done
[ -z "$DRIFT" ] && ok "영구 증거가 가변 핸드오프 내용을 인용하지 않음" \
                || bad "영구 증거가 교체되는 핸드오프 내용을 인용:$DRIFT → 불변 역사 절 또는 커밋 고정(git show <hash>:…)으로 바꿀 것"

# ── 8. 원격 동기 (커밋 직전 기준 — 새 커밋은 --after-push 로 재확인) ─────
remote_check

echo
if [ "$FAIL" != "0" ]; then
    echo "❌ 위 FAIL 을 고친 뒤 커밋할 것 (문서 불일치 = 다음 세션의 실행 버그)"
elif [ "$SKIPPED" != "0" ]; then
    echo "✅ 검사한 항목은 이상 없음 — 단 $SKIPPED 개 생략됨(위 -- 표시). 전수 확인은 --strict"
else
    echo "✅ 문서 동기화 이상 없음"
fi
echo "   ※ 커밋+push 후 'bash tools/doc_check.sh --after-push' 로 원격 동기 재확인"
exit "$FAIL"
