#!/usr/bin/env bash
# doc_check.sh — 문서 동기화 자동 검사 (07-20 신설)
#
# 목적: "문서는 다음 세션의 입력"이므로 낡은 문서 = 실행 버그다.
#       사람이 기억으로 지키던 동기화를 기계가 대신 검사한다.
# 사용: bash tools/doc_check.sh      (TEST_GATES §1 게이트의 마지막 단계)
# 결과: 전부 통과 = exit 0 / 하나라도 어긋나면 exit 1 + 무엇이 왜 틀렸는지 출력
#
# ⚠ 이 스크립트가 잡는 것은 '숫자·존재·중복' 같은 기계적 불일치뿐이다.
#    "정책이 서로 모순된다" 같은 의미 수준 불일치는 Codex 독립 검토가 잡는다.

set -uo pipefail
cd "$(dirname "$0")/.." || exit 1

FAIL=0
ok()   { printf '  \033[32mOK\033[0m   %s\n' "$1"; }
bad()  { printf '  \033[31mFAIL\033[0m %s\n' "$1"; FAIL=1; }
skip() { printf '  --   %s\n' "$1"; }

echo "=== 문서 동기화 검사 ==="

# ── 1. pytest 기준선: 실제 개수 == TEST_GATES 에 적힌 수치 ────────────────
# 낡은 기준선은 회귀 검출력을 조용히 무력화한다(테스트가 사라져도 PASS 로 보임).
ACTUAL=$(python3 -m pytest src/mission_manager/test/ -q 2>/dev/null \
         | grep -oP '\d+(?= passed)' | head -1)
DOC=$(grep -oP 'pytest \*\*\K\d+' docs/TEST_GATES.md | head -1)
if [ -z "$ACTUAL" ]; then
    bad "pytest 실행 실패 — 테스트가 깨졌거나 환경 문제"
elif [ "$ACTUAL" = "$DOC" ]; then
    ok "pytest 기준선 $DOC 개 = 실제"
else
    bad "pytest 기준선 불일치: 문서 $DOC / 실제 $ACTUAL → docs/TEST_GATES.md §1 갱신 필요"
fi

# ── 2. colcon 기준선 (이전 실행 결과가 있을 때만) ────────────────────────
if command -v colcon >/dev/null 2>&1 && [ -d build ]; then
    CA=$(colcon test-result 2>/dev/null | grep -oP '\d+(?= tests)' | tail -1)
    CD=$(grep -oP 'colcon \*\*\K\d+' docs/TEST_GATES.md | head -1)
    if [ -z "$CA" ]; then
        skip "colcon 결과 없음 (colcon test 미실행 — 게이트에서 먼저 실행할 것)"
    elif [ "$CA" = "$CD" ]; then
        ok "colcon 기준선 $CD 개 = 실제"
    else
        bad "colcon 기준선 불일치: 문서 $CD / 실제 $CA → docs/TEST_GATES.md §1 갱신 필요"
    fi
else
    skip "colcon 미검사 (환경에 colcon 없음 또는 build/ 없음)"
fi

# ── 3. '현재 위치' 단일 출처: MASTER_PLAN 이 현재를 가리키면 안 됨 ────────
# 07-20 실사고 재발 방지 — 현재 단계는 CURRENT_HANDOFF 한 곳에만 존재해야 한다.
if grep -q "◀ *현재" docs/MASTER_PLAN.md; then
    bad "MASTER_PLAN 에 '◀ 현재' 표시가 있다 → 현재 위치 정본은 CURRENT_HANDOFF 한 곳뿐"
else
    ok "현재 위치 단일 출처 유지 (MASTER_PLAN 에 중복 없음)"
fi

# ── 4. CURRENT_HANDOFF 템플릿 필수 절 유지 ───────────────────────────────
# 범위·완료판정 없는 인계장은 다음 세션이 제멋대로 하게 만든다.
MISS=""
for sec in "현재 단계" "이번 한 묶음 목표" "완료조건" "금지 범위" "완료 판정" "근거 문서"; do
    grep -q "$sec" docs/CURRENT_HANDOFF.md || MISS="$MISS '$sec'"
done
[ -z "$MISS" ] && ok "CURRENT_HANDOFF 필수 절 전부 존재" \
               || bad "CURRENT_HANDOFF 필수 절 누락:$MISS"

# ── 5. repo 문서 상호참조 깨짐 검사 ──────────────────────────────────────
BROKEN=""
for f in $(grep -rhoP 'docs/[A-Za-z_]+\.md' docs/*.md AGENTS.md CLAUDE.md 2>/dev/null | sort -u); do
    [ -f "$f" ] || BROKEN="$BROKEN $f"
done
[ -z "$BROKEN" ] && ok "repo 문서 링크 전부 유효" || bad "없는 문서 참조:$BROKEN"

# ── 6. Desktop 역사 문서 참조 깨짐 검사 (이름 변경·이동 감지) ────────────
DESK="$HOME/Desktop/개발현황"
MISSD=""
for f in $(grep -rhoP '[0-9]{4}_[가-힣A-Za-z_]+\.md' docs/*.md AGENTS.md CLAUDE.md 2>/dev/null | sort -u); do
    [ -f "$DESK/$f" ] || MISSD="$MISSD $f"
done
[ -z "$MISSD" ] && ok "Desktop 역사 문서 참조 유효" \
               || bad "Desktop 에 없는 문서 참조:$MISSD"

# ── 7. 기준 커밋 표기: 미push 상태 경고 ──────────────────────────────────
if git rev-parse --abbrev-ref '@{upstream}' >/dev/null 2>&1; then
    AHEAD=$(git rev-list --count '@{upstream}..HEAD' 2>/dev/null || echo 0)
    [ "$AHEAD" = "0" ] && ok "원격과 동기 (커밋=push 한 세트 준수)" \
                       || bad "push 안 된 커밋 $AHEAD 개 — AGENTS.md §5 '커밋 = push 한 세트'"
else
    skip "upstream 없음"
fi

echo
if [ "$FAIL" = "0" ]; then
    echo "✅ 문서 동기화 이상 없음"
else
    echo "❌ 위 FAIL 항목을 고친 뒤 커밋할 것 (문서 불일치 = 다음 세션의 실행 버그)"
fi
exit "$FAIL"
