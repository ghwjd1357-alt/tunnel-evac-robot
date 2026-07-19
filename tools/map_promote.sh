#!/usr/bin/env bash
# ============================================================
# map_promote.sh — 지도 정본 승격 transaction 소도구 (G2, 07-19 마스터플랜 §7.5)
#
# 배경: make_map 인라인 승격의 3가지 구멍 (Codex §13.4)
#   ① 정본이 아직 없는 '최초 생성'에선 rollback 이 불가능 (백업이 없어서
#      중간 실패 시 새 정본 파일이 반쪽만 남음 — 축소 재현으로 실증)
#   ② rollback 이 cp 복원만 해서 staging 산출물이 유실 (재시도 불가)
#   ③ 승격 로직이 스크립트에 박혀 있어 격리 테스트 불가
#
# 정책: fail-closed transaction —
#   ① 모든 staging 소스 존재+비어있지 않음 확인 (하나라도 없으면 아무것도 안 함)
#   ② 기존 정본은 .bak_스탬프 백업 (cp 실패 = 즉시 중단, 무손상)
#      없던 정본은 '없었음'을 기록 (최초 생성 대비)
#   ③ 일괄 mv, 중간 실패 시 rollback:
#      - 이미 옮긴 파일은 staging 이름으로 되돌림 (산출물 보존 → 재시도 가능)
#      - 기존 정본이 있었으면 백업에서 복원, 없었으면 '부재' 상태로 복구
#
# 사용: map_promote.sh <MAPDIR> <STAMP> <src:dst> [<src:dst> ...]
# 종료코드: 0 = 전부 승격 / 1 = 실패 (이전 상태로 원복됨)
# 테스트 훅: MAP_PROMOTE_FAIL_AT=n → n번째(0부터) mv 강제 실패 (하네스 전용)
# ============================================================
set -u

[ $# -ge 3 ] || { echo "PROMOTE-FAIL: 인자 부족 (MAPDIR STAMP src:dst ...)"; exit 1; }
MAPDIR="$1"; STAMP="$2"; shift 2
[ -d "$MAPDIR" ] || { echo "PROMOTE-FAIL: MAPDIR 없음 ($MAPDIR)"; exit 1; }

SRCS=(); DSTS=(); BAKS=(); EXISTED=()
for pair in "$@"; do
  SRCS+=("${pair%%:*}")
  DSTS+=("${pair##*:}")
done

# ① staging 소스 전수 확인 — 하나라도 없으면 시작 자체를 안 함 (부분 승격 금지)
for s in "${SRCS[@]}"; do
  [ -s "$MAPDIR/$s" ] || { echo "PROMOTE-FAIL: staging 없음/빈파일 ($s) — 아무것도 안 옮김"; exit 1; }
done

# ② 백업 (확장자 앞에 .bak_스탬프 — 기존 관례 유지: name.bak_YYMMDD_HHMMSS.ext)
for i in "${!DSTS[@]}"; do
  d="${DSTS[$i]}"
  b="${d%.*}.bak_${STAMP}.${d##*.}"
  BAKS+=("$b")
  if [ -f "$MAPDIR/$d" ]; then
    EXISTED+=(1)
    cp "$MAPDIR/$d" "$MAPDIR/$b" \
      || { echo "PROMOTE-FAIL: 백업 실패 ($d) — 정본·staging 무손상 중단"; exit 1; }
  else
    EXISTED+=(0)   # 최초 생성: 이전 상태 = '부재'
  fi
done

rollback() {  # $1 = 실패 인덱스 (그 앞까지는 이동 완료 상태)
  local j
  for j in $(seq 0 $(($1 - 1))); do
    # staging 산출물 보존 (되돌려야 재시도 가능 — cp 복원만 하면 유실)
    mv -f "$MAPDIR/${DSTS[$j]}" "$MAPDIR/${SRCS[$j]}" 2>/dev/null || true
    if [ "${EXISTED[$j]}" = 1 ]; then
      cp "$MAPDIR/${BAKS[$j]}" "$MAPDIR/${DSTS[$j]}"
    else
      rm -f "$MAPDIR/${DSTS[$j]}"   # 최초 생성 rollback = 부재로 복구 (Codex §13.4-3)
    fi
  done
}

do_mv() {  # $1 = 인덱스 — MAP_PROMOTE_FAIL_AT 훅으로 실패 주입 가능
  if [ "${MAP_PROMOTE_FAIL_AT:-}" = "$1" ]; then return 1; fi
  mv "$MAPDIR/${SRCS[$1]}" "$MAPDIR/${DSTS[$1]}"
}

# ③ 일괄 승격
for i in "${!DSTS[@]}"; do
  if ! do_mv "$i"; then
    rollback "$i"
    echo "PROMOTE-FAIL: 승격 실패(${SRCS[$i]}) — 이전 상태로 원복됨 (staging 보존, 재시도 가능)"
    exit 1
  fi
done
echo "PROMOTE-OK: ${#DSTS[@]}개 파일 승격 완료 (백업 스탬프 $STAMP)"
