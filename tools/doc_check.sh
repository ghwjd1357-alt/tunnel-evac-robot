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
if [ -z "$ACTUAL" ]; then
    bad "pytest 실행 실패 — 테스트가 깨졌거나 환경 문제"
else
    ok "pytest 실행 $ACTUAL 개 passed (문서 대조는 아래 2b 전수 스윕)"
fi

# ── 2. colcon 실제값 (이전 실행 결과가 있을 때만) ────────────────────────
CA=""
if command -v colcon >/dev/null 2>&1 && [ -d build ]; then
    CA=$(colcon test-result 2>/dev/null | grep -oP '\d+(?= tests)' | tail -1)
    if [ -z "$CA" ]; then
        skip "colcon 실제값 — 테스트 결과 없음(colcon test 미실행)"
    else
        ok "colcon 실제 $CA 개 (문서 대조는 아래 2b 전수 스윕)"
    fi
else
    skip "colcon 실제값 — colcon 없음 또는 build/ 없음"
fi

# ── 2b. 게이트 기준선 4종 — 문서 **전수 스윕** 대조 ──────────────────────
# ★ 08-01 §18.2 P2-①: 직전 구현은 여기에 `grep … | head -1` 캡처 7개를 **손으로 나열**했다.
#   나열은 두 가지로 샌다 — ① 표기가 조금만 달라도 못 본다(`22 검사` 는 잡고 `22검사` 는 놓쳤다)
#   ② 나열 목록과 구현이 갈라진다(착수 때 grep 출력엔 13자리가 다 있었는데 패턴은 7개였다).
#   실측 사각 6자리: TEST_GATES:129·:11·:71 · MASTER_PLAN:197 · CURRENT_HANDOFF 의 pytest·colcon.
#   → 나열을 버리고 **훑는 행위 자체를 검사로** 만든다 (`AGENTS.md §3-10` 커버리지 폐포).
#     규칙·역사 제외·표기 규약 전량 = `tools/gate_baseline_scan.py` 머리말.
#
# 셸 게이트를 세는 법: 각 케이스는 성공 시 `ok "…"` 를 **정확히 한 번** 부른다(`ng` 는 실패
#   분기가 여러 개일 수 있어 케이스 수와 무관). 실행(≈130초·≈163초)을 doc_check 에 넣지 않고
#   정적으로 세어 1초 안에 끝낸다. ⚠ 이 규약은 강제가 아니라 관례다 — 실측 대조 = harness 22,
#   gate_regr 14 로 일치(08-01, 검토 §18.5 가 독립 재확인).
GB_ARGS=()
[ -n "$ACTUAL" ] && GB_ARGS+=(--expect "pytest=$ACTUAL")
[ -n "$CA" ]     && GB_ARGS+=(--expect "colcon=$CA")
GB_ARGS+=(--expect "harness=$(grep -cP '(^|\s)ok "' tools/test_harness_guards.sh)")
GB_ARGS+=(--expect "gate_regression=$(grep -cP '(^|\s)ok "' tools/test_gate_regression.sh)")
GB_OUT=$(python3 tools/gate_baseline_scan.py "${GB_ARGS[@]}" 2>&1)
if [ -z "$GB_OUT" ]; then
    bad "gate_baseline_scan 이 아무 출력도 못 냈다 — 스윕 자체가 죽었다(fail-closed)"
else
    while IFS= read -r gb_line; do
        case "$gb_line" in
            "OK   "*) ok   "${gb_line#OK   }" ;;
            "FAIL "*) bad  "${gb_line#FAIL }" ;;
            *)        bad  "gate_baseline_scan: $gb_line" ;;
        esac
    done <<< "$GB_OUT"
fi

# ── 2c. CURRENT_HANDOFF '지금 하는 일'의 단일성 ──────────────────────────
# ★ 08-01 §16.3 P2 → §17.3 → §18.3: 상단 '현재 단계'·진행표·본문 '이번 한 묶음 목표'가
#   서로 다른 묶음(§15 / §12 / §14)을 가리킨 채 세 회차를 지나갔다. "항상 이 한 묶음만 유지"가
#   이 파일의 계약인데 사람이 손으로 맞추다 갈라졌고, 기계가 안 잡았다.
#   ⚠ §18.3: 직전 구현은 `head -1`/`tail -1` 로 **첫(끝) 하나만** 골랐다. 값을 바꿔치기하면
#     잡지만 **행을 하나 더 추가**하면 뒤로 숨었다 — 계약을 깨는 가장 직접적인 방법이 그것이다.
#     그리고 상단을 파일 전체의 `tail -1` 로 읽어, 아무 데나 표기를 넣으면 현재값이 바뀌었다.
#   → 규칙 전량과 판정 = `tools/handoff_single_check.sh` (픽스처로 회귀를 걸 수 있게 분리했다 —
#     doc_check 안에 두면 진짜 문서 하나로만 검사돼 부정 회귀를 영구화할 수 없다).
HS_OUT=$(bash tools/handoff_single_check.sh docs/CURRENT_HANDOFF.md 2>&1)
if [ -z "$HS_OUT" ]; then
    bad "handoff_single_check 가 아무 출력도 못 냈다 — 검사 자체가 죽었다(fail-closed)"
else
    while IFS= read -r hs_line; do
        case "$hs_line" in
            "OK   "*) ok  "CURRENT_HANDOFF ${hs_line#OK   }" ;;
            "FAIL "*) bad "CURRENT_HANDOFF ${hs_line#FAIL }" ;;
            *)        bad "handoff_single_check: $hs_line" ;;
        esac
    done <<< "$HS_OUT"
fi

# ── 2d. `TODO(D+0/D+1)` 목록·실행 표식 계약 (08-03 · 검토 §30.4·§34.6) ───
# ★ 왜 필요했나: `JETSON_SETUP §9` 의 목록은 10건인데 핸드오프 완료조건 네 자리가 8건이라고
#   적은 채 `--strict` 가 PASS 했다. 현장 작업자가 8건만 채우면 NTP·E-stop 이 통째로 빠진다.
#   개수는 사람이 지키는 약속이 아니라 **기계가 세는 사실**이어야 한다.
#   ⚠ 드리프트 자리 하나는 `TODO(D+0)` 와 `8건` 이 **다른 줄**에 있었다 — 그래서 줄 단위가
#     아니라 공백 정규화 + 근접 창으로 본다(아래 §9 검사가 같은 이유로 쓰는 방식).
#   §34.6: D1은 목록 10건인데 실행 가능한 표식이 7건뿐이었다. 같은 알고리즘을 phase 인자로
#   두 번 실행하며, 목록·개수 주장·실행 표식의 증가/감소를 모두 exact 계약으로 막는다.
#   규칙·검증 상한 전량 = `tools/todo_d0_scan.py` 머리말.
for TD_PHASE in D+0 D+1; do
    TD_OUT=$(python3 tools/todo_d0_scan.py --phase "$TD_PHASE" 2>&1)
    if [ -z "$TD_OUT" ]; then
        bad "todo_d0_scan $TD_PHASE 가 아무 출력도 못 냈다 — 검사 자체가 죽었다(fail-closed)"
    else
        while IFS= read -r td_line; do
            case "$td_line" in
                "OK   "*) ok  "${td_line#OK   }" ;;
                "FAIL "*) bad "${td_line#FAIL }" ;;
                *)        bad "todo_d0_scan $TD_PHASE: $td_line" ;;
            esac
        done <<< "$TD_OUT"
    fi
done

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
    # 지원 형식: N / N.N / N-X / N-X-Y (X,Y = 숫자 또는 영문). 그 외는 FAIL.
    # 🔴 08-13 확장 — 하위 절이 두 단 깊어졌다(`§1-b-1`, `§4-f-4`). 구판 규칙은 대시
    #   **한 번**만 허용해 정상 참조를 "지원하지 않는 절 형식" 으로 떨궜다.
    if ! printf '%s' "$raw" | grep -qP '^[0-9]+(\.[0-9]+)*(-[0-9A-Za-z]+)*$'; then
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
# 🔴 08-13 — 검색 자리가 `개발현황/` 하나뿐이라는 전제가 틀렸다. 날짜별 현황은 거기 있지만
#   정찰·작업가이드 같은 1차 기록은 **Desktop 루트**에 쓰인다(`0813_복도정찰.md`). 한 자리만
#   보면 실재하는 문서를 "없다"고 부른다 — 이름 변경 감지라는 목적에 두 자리 다 필요하다.
for f in $(grep -rhoP '[0-9]{4}_[가-힣A-Za-z_]+\.md' "${SRC[@]}" 2>/dev/null | sort -u); do
    [ -f "$DESK/$f" ] || [ -f "$HOME/Desktop/$f" ] || MISSD="$MISSD $f"
done
[ -z "$MISSD" ] && ok "Desktop 역사 문서 참조 유효" || bad "Desktop 에 없는 문서:$MISSD"

# ── 9. 영구 증거 문서가 '가변 핸드오프의 내용'을 근거로 삼지 않는가 ──────
# CURRENT_HANDOFF 는 묶음마다 통째로 교체된다. 영구 증거(동결 manifest)가 그 파일의
# 완료조건·금지 범위를 인용하면, 교체되는 순간 같은 문장이 조용히 다른 뜻이 된다.
#   07-24 실사고(Codex 0723검토 §10 P2): manifest 가 "CURRENT_HANDOFF 완료조건 6"을
#   태그 순서의 근거로 걸어 뒀는데, 같은 커밋이 핸드오프를 교체해 그 번호가 무관한
#   항목이 됐다. 위 5·6번 검사는 링크와 절의 '존재'만 보므로 이 의미 드리프트를 PASS 했다.
# 허용 = ① 그 핸드오프 경로 자체가 해시로 고정된 참조(git show <hash>:docs/CURRENT_HANDOFF.md)
#        ② 내용 인용 없이 파일명만 언급 (파일 생명주기 설명 등)
#
# ★ 07-24 개정 (Codex 0723검토 §11.2 P2) — 초판은 **줄 단위**로 검사해 두 가지로 우회됐다:
#   ⓐ Markdown 은 폭에 맞춰 줄을 바꾸므로 'CURRENT_HANDOFF' 와 절 이름이 다른 줄에 놓이면 놓쳤다.
#   ⓑ 무관한 `git show <hash>:README.md` 가 같은 줄 앞에 있기만 해도 줄 전체가 예외 처리됐다.
#   → ⓐ 공백을 한 칸으로 정규화해 **줄바꿈 위치와 무관**하게 보고,
#     ⓑ 예외를 '핸드오프 경로 자체가 고정된 참조'로 좁혀 **그 참조만 지운 뒤** 남은 것을 검사한다.
# ⚠ 한계 2가지 (은폐하지 않는다):
#   · 검출 대상은 docs/*MANIFEST*.md — 새 영구 증거 문서는 이 이름 규칙을 따를 것.
#   · 근접 창(HWIN)보다 멀리 떨어뜨려 쓰면 놓친다. 창은 '표현을 바꿔 우회'까지 막지는 못한다.
HSEC='완료조건|금지 범위|허용 파일|완료 판정|이번 한 묶음|현재 단계'
HWIN=120        # 근접 창(정규화 후 문자 수) ≈ 줄바꿈으로 갈라진 인접 1~2줄
DRIFT=""
for f in docs/*MANIFEST*.md; do
    [ -f "$f" ] || continue
    # ① 줄바꿈·들여쓰기를 한 칸으로 정규화  ② 해시 고정된 핸드오프 참조는 통째로 제거
    hit=$(tr -s '[:space:]' ' ' < "$f" \
          | sed -E 's#git show +[0-9a-f]{7,40}:[^ ]*CURRENT_HANDOFF[^ ]*##g' \
          | grep -oP "(CURRENT_HANDOFF.{0,$HWIN}?($HSEC))|(($HSEC).{0,$HWIN}?CURRENT_HANDOFF)" \
          | head -1)
    [ -n "$hit" ] && DRIFT="$DRIFT [$f → \"$hit\"]"
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
