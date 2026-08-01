#!/usr/bin/env bash
# handoff_single_check.sh — CURRENT_HANDOFF 가 **한 묶음만** 가리키는지 검사한다.
#
# 왜 있나 (08-01 · 검토 §16.3 → §17.3 → §18.3):
#   이 파일의 계약은 "항상 이 한 묶음만 유지" 다. 그런데 상단 '현재 단계'·진행표·본문
#   '이번 한 묶음 목표' 가 서로 다른 묶음(§15 / §12 / §14)을 가리킨 채 **세 회차**를 지나갔다.
#   §17 보완은 세 자리의 § 를 뽑아 동일성만 봤는데, `head -1`/`tail -1` 로 **하나만** 골라
#   ① 행을 하나 더 추가하면 뒤로 숨었고(계약을 깨는 가장 직접적인 방법이 그것이다)
#   ② 상단을 파일 전체의 마지막 일치로 읽어 아무 데나 표기를 넣으면 현재값이 바뀌었다.
#
# 그래서 이 검사는 값보다 **카디널리티와 경계**를 먼저 본다:
#   ① `- **현재 단계**` 불릿 = 정확히 1 개
#   ② `| **진행 중 묶음** |` 행 = 정확히 1 개, `## 이번 한 묶음 목표` 제목 = 정확히 1 개
#   ③ `→ **검토 §N**` 표기는 **현재 단계 블록 안**에만 존재 (블록 밖 = 현재 지시가 모호해짐)
#   ④ 그 위에서 세 § 가 같은가 — 상단은 블록 안 화살표 **체인의 꼬리**가 현재다
#   문구 형식은 강제하지 않는다. 역사 절의 옛 § 번호는 애초에 위 세 패턴이 아니다.
#
# 사용:  bash tools/handoff_single_check.sh [파일]      (기본 = docs/CURRENT_HANDOFF.md)
# 출력:  `OK   …` / `FAIL …` 한 줄.  종료코드 0 = 이상 없음 / 1 = 위반 / 2 = 사용법·파일 오류

set -uo pipefail

HO="${1:-docs/CURRENT_HANDOFF.md}"
if [ ! -f "$HO" ]; then
    echo "FAIL 핸드오프 파일이 없다: $HO" >&2
    exit 2
fi

# 블록 경계: `- **현재 단계**` 부터 다음 최상위 불릿(`- **`) 또는 제목(`#`) 직전까지.
ARROW='→ \*\*검토 §\K\d+(?=[^*]*\*\*)'
BLOCK=$(awk '/^- \*\*현재 단계\*\*/{inb=1; print; next}
             inb && /^(- \*\*|#)/{inb=0}
             inb{print}' "$HO")

N_HEAD=$(grep -cP '^- \*\*현재 단계\*\*' "$HO")
N_TBL=$(grep -cP '\|\s*\*\*진행 중 묶음\*\*\s*\|' "$HO")
N_BODY=$(grep -cP '^## 이번 한 묶음 목표' "$HO")
N_ARROW_ALL=$(grep -cP "$ARROW" "$HO")
N_ARROW_IN=$(printf '%s\n' "$BLOCK" | grep -cP "$ARROW")

TOP=$(printf '%s\n' "$BLOCK" | grep -oP "$ARROW" | tail -1)
TBL=$(grep -oP '\|\s*\*\*진행 중 묶음\*\*\s*\|\s*§\K\d+' "$HO" | head -1)
BODY=$(grep -oP '^## 이번 한 묶음 목표.*?§\K\d+' "$HO" | head -1)

if [ "$N_HEAD" != "1" ]; then
    echo "FAIL '현재 단계' 불릿이 $N_HEAD 개 — 정확히 1 개여야 한다 ($HO)"
    exit 1
fi
if [ "$N_TBL" != "1" ] || [ "$N_BODY" != "1" ]; then
    echo "FAIL 현재 지시가 중복/누락: 진행표 행 $N_TBL 개 · 본문 목표 $N_BODY 개 — 각각 정확히 1 개여야 한다('항상 이 한 묶음만 유지')"
    exit 1
fi
if [ "$N_ARROW_IN" != "$N_ARROW_ALL" ]; then
    echo "FAIL 상단 표기 '→ **검토 §N**' 가 '현재 단계' 블록 **밖**에 $((N_ARROW_ALL - N_ARROW_IN)) 개 — 현재 지시가 모호해진다"
    exit 1
fi
if [ -z "$TOP" ] || [ -z "$TBL" ] || [ -z "$BODY" ]; then
    echo "FAIL 현재 묶음 § 를 세 자리(상단·진행표·본문 목표)에서 다 못 찾음: 상단='$TOP' 진행표='$TBL' 본문='$BODY'"
    exit 1
fi
if [ "$TOP" = "$TBL" ] && [ "$TBL" = "$BODY" ]; then
    echo "OK   현재 묶음 단일 (§$TOP — 상단(블록 안 $N_ARROW_IN 개 중 꼬리)·진행표·본문 각 1 개 일치)"
    exit 0
fi
echo "FAIL 서로 다른 묶음을 지시: 상단=§$TOP 진행표=§$TBL 본문=§$BODY — '항상 이 한 묶음만 유지' 위반"
exit 1
