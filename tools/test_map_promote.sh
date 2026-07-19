#!/usr/bin/env bash
# test_map_promote.sh — map_promote.sh 격리 하네스 (G2 완료판정 — Codex §13.4 공격 시나리오 5종)
# 사용: bash tools/test_map_promote.sh (1초, 실지도 무접촉 — mktemp 격리)
set -u
TOOL=~/ros2_ws/tools/map_promote.sh
PASS=0; FAIL=0
check() { if eval "$2"; then echo "  ✓ $1"; PASS=$((PASS+1)); else echo "  ✗ $1"; FAIL=$((FAIL+1)); fi; }

fresh() {  # $1=with_canon(1|0) — staging 4개 + (옵션) 기존 정본 4개
  D=$(mktemp -d)
  for f in pg.posegraph pg.data m.pgm m.yaml; do echo "NEW-$f" > "$D/s_$f"; done
  if [ "$1" = 1 ]; then for f in pg.posegraph pg.data m.pgm m.yaml; do echo "OLD-$f" > "$D/$f"; done; fi
}
PAIRS=(s_pg.posegraph:pg.posegraph s_pg.data:pg.data s_m.pgm:m.pgm s_m.yaml:m.yaml)

echo "== A. 정상 승격 (기존 정본 있음)"
fresh 1
bash "$TOOL" "$D" T1 "${PAIRS[@]}" > /dev/null
check "4파일 전부 새 내용" '[ "$(cat "$D/pg.posegraph")" = NEW-pg.posegraph ] && [ "$(cat "$D/m.yaml")" = NEW-m.yaml ]'
check "백업 4개 = 구 내용" '[ "$(cat "$D/pg.bak_T1.posegraph")" = OLD-pg.posegraph ] && [ "$(cat "$D/m.bak_T1.yaml")" = OLD-m.yaml ]'
rm -rf "$D"

echo "== B. 중간 실패 (3번째 mv, 기존 정본 있음) — 원복 + staging 보존"
fresh 1
MAP_PROMOTE_FAIL_AT=2 bash "$TOOL" "$D" T2 "${PAIRS[@]}" > /dev/null && echo "  ✗ 성공하면 안 됨" || true
check "정본 4개 전부 구 내용 복원" '[ "$(cat "$D/pg.posegraph")" = OLD-pg.posegraph ] && [ "$(cat "$D/pg.data")" = OLD-pg.data ] && [ "$(cat "$D/m.pgm")" = OLD-m.pgm ]'
check "staging 4개 보존 (재시도 가능)" '[ "$(cat "$D/s_pg.posegraph")" = NEW-pg.posegraph ] && [ -s "$D/s_pg.data" ] && [ -s "$D/s_m.pgm" ] && [ -s "$D/s_m.yaml" ]'
rm -rf "$D"

echo "== C. ★최초 생성 중간 실패 (정본 없음, 2번째 mv 실패) — Codex §13.4-3 재현"
fresh 0
MAP_PROMOTE_FAIL_AT=1 bash "$TOOL" "$D" T3 "${PAIRS[@]}" > /dev/null && echo "  ✗ 성공하면 안 됨" || true
check "새 정본 파일 0개 잔존 (부재로 복구)" '[ ! -f "$D/pg.posegraph" ] && [ ! -f "$D/pg.data" ] && [ ! -f "$D/m.pgm" ] && [ ! -f "$D/m.yaml" ]'
check "staging 4개 보존" '[ -s "$D/s_pg.posegraph" ] && [ -s "$D/s_pg.data" ] && [ -s "$D/s_m.pgm" ] && [ -s "$D/s_m.yaml" ]'
rm -rf "$D"

echo "== D. 최초 생성 정상 승격"
fresh 0
bash "$TOOL" "$D" T4 "${PAIRS[@]}" > /dev/null
check "4파일 승격, 백업 없음(원래 부재)" '[ "$(cat "$D/pg.posegraph")" = NEW-pg.posegraph ] && [ ! -f "$D/pg.bak_T4.posegraph" ]'
rm -rf "$D"

echo "== E. staging 결손 — 아무것도 안 옮겨야 함"
fresh 1
rm "$D/s_m.yaml"
bash "$TOOL" "$D" T5 "${PAIRS[@]}" > /dev/null && echo "  ✗ 성공하면 안 됨" || true
check "정본 4개 무손상 (구 내용 유지)" '[ "$(cat "$D/pg.posegraph")" = OLD-pg.posegraph ] && [ "$(cat "$D/m.pgm")" = OLD-m.pgm ]'
rm -rf "$D"

echo "== 결과: PASS=$PASS FAIL=$FAIL"
[ "$FAIL" = 0 ]
