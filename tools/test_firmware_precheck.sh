#!/usr/bin/env bash
# test_firmware_precheck.sh — `firmware_precheck.sh` 를 **공격한다**.
#
# 사용:  bash tools/test_firmware_precheck.sh
# 종료:  0 = 전 케이스 통과 / 1 = 한 건이라도 실패
#
# ── 왜 이 픽스처가 필요한가 (08-07 검토 §47.1 P1) ────────────────────────────
# 구판 사전검사는 **자기가 통과시킨 오염을 스스로 증명하지 못했다.** `MASTER_PLAN §7` 에는
# "임의 2줄 주입 → 10/3 → 중단" 이라는 관측이 적혀 있었지만, 문서가 제시한 명령
# (`git diff --numstat f57d454 HEAD -- …`)으로는 **그 관측이 재현되지 않는다** — 끝점이
# `HEAD` 라 작업 트리가 비교에서 빠지기 때문이다. 관측을 눈으로 적어 두는 것과 검사가 실제로
# 막는 것은 다른 일이고, 그 차이가 회차를 넘어 살아남았다.
#   → 그래서 판정을 종료코드로 옮기고, **그 종료코드를 픽스처가 공격한다.**
#
# 케이스 1~3 은 §47.1 이 요구한 부정·역회귀 그대로다. 나머지는 같은 자리에서 발견한 이웃
# 구멍(미추적·공식 확장자 전수·명시적 `src/`/`data/` 경계·ignore·삭제·판정 불능)이고, 마지막은
# **진짜 저장소**에 대고 도는 역회귀다.
#
# ⚠ 합성 저장소는 실물 이력이 아니다 — 여기 통과는 실차 통과가 아니라 **판정기의 성질**만
#   증명한다. 실물 대조는 케이스 10 하나뿐이고, 그것도 저장소만 본다(보드는 못 본다).

set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$HERE")"
CHECK="$HERE/firmware_precheck.sh"

PASS=0
FAIL=0
ok()  { PASS=$((PASS + 1)); echo "  PASS  $1"; }
ng()  { FAIL=$((FAIL + 1)); echo "  FAIL  $1"; }

# expect_rc <기대종료코드> <설명> -- <명령...>
expect_rc() {
    local want="$1" desc="$2"; shift 3   # 3번째는 `--`
    local out rc
    out="$("$@" 2>&1)"; rc=$?
    if [ "$rc" -eq "$want" ]; then
        ok "$desc  (rc=$rc)"
    else
        ng "$desc  (기대 rc=$want · 실제 rc=$rc)"
        echo "$out" | sed 's/^/        | /'
    fi
    LAST_OUT="$out"
}

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

# ── 합성 저장소: 실물과 **같은 경로 모양**으로 짓는다 (pathspec 을 그대로 시험하려고) ──
SKETCH="firmware/teensy_integrated_base_v1_4"
INO="$SKETCH/teensy_integrated_base_v1_4.ino"
REPO="$TMP/repo"
mkdir -p "$REPO/$SKETCH"
git -C "$REPO" init -q 2>/dev/null || git init -q "$REPO"
git -C "$REPO" config user.email "fixture@example.invalid"
git -C "$REPO" config user.name  "fixture"

# 기준점 = vendor 수령본. 10줄짜리 가짜 소스.
for i in 1 2 3 4 5 6 7 8 9 10; do echo "// vendor line $i"; done > "$REPO/$INO"
echo "# vendor drop notes" > "$REPO/firmware/VENDOR_DROP.md"
git -C "$REPO" add -A && git -C "$REPO" commit -qm "vendor drop"
BASE="$(git -C "$REPO" rev-parse HEAD)"

# 허용된 변경 = 8 추가 / 2 삭제. 실물의 `ESTOP_ACTIVE_LOW` 한 줄 수정과 같은 모양.
{
    for i in 1 2 3 4 5 6 7 8; do echo "// allowed edit $i"; done
    for i in 3 4 5 6 7 8 9 10; do echo "// vendor line $i"; done
} > "$REPO/$INO"
git -C "$REPO" add -A && git -C "$REPO" commit -qm "allowed firmware edit"

EXPECT="$INO=8,2"
run() { bash "$CHECK" --repo "$REPO" --baseline "$BASE" --expect "$EXPECT"; }

echo "=== firmware_precheck 픽스처 ==="

# ── 1) 역회귀: 허용된 변경만 커밋된 상태는 통과한다 ─────────────────────────
expect_rc 0 "① 허용된 committed 8/2 만 있으면 PASS" -- run

# ── 2) 부정 회귀: unstaged 한 줄 (§47.1 이 재현한 바로 그 상태) ─────────────
echo "// uncommitted probe" >> "$REPO/$INO"
expect_rc 1 "② unstaged 한 줄 추가 → FAIL" -- run

#    같은 상태에서 **구판 명령**은 통과했다는 사실을 함께 박제한다.
OLD="$(git -C "$REPO" diff --numstat "$BASE" HEAD -- "firmware/*/*.ino")"
if [ "$OLD" = "8	2	$INO" ]; then
    ok "②-b 구판(HEAD 끝점) 명령은 같은 오염에서 여전히 '8 2' 를 낸다 — 결함의 증거"
else
    ng "②-b 구판 명령이 '8 2' 가 아니다: $OLD  (결함 재현이 안 되면 이 픽스처의 전제가 바뀐 것)"
fi

# ── 3) 부정 회귀: staged 로 올려도 마찬가지다 ──────────────────────────────
git -C "$REPO" add "$INO"
expect_rc 1 "③ staged 한 줄 추가 → FAIL" -- run
git -C "$REPO" reset -q --hard

# ── 4) 부정 회귀: 허용된 줄을 되돌리면 '변경 없음' 도 오염이다 ─────────────
git -C "$REPO" show "$BASE:$INO" > "$REPO/$INO"
expect_rc 1 "④ 허용된 수정을 되돌리면 FAIL (있어야 할 변경이 사라진 것)" -- run
case "$LAST_OUT" in
    *"되돌려졌다"*) ok "④-b 사유가 '되돌려졌다' 로 구분돼 나온다" ;;
    *) ng "④-b 되돌림 사유가 출력에 없다" ;;
esac
git -C "$REPO" reset -q --hard

# ── 5) 부정 회귀: 미추적 `.ino` — git diff 가 못 보는 자리 ──────────────────
echo "void rogue() {}" > "$REPO/$SKETCH/rogue_patch.ino"
expect_rc 1 "⑤ 미추적 .ino 를 폴더에 떨구면 FAIL (arduino-cli 는 이걸 함께 굽는다)" -- run
NOTSEEN="$(git -C "$REPO" diff --numstat "$BASE" -- "firmware/*/*.ino")"
case "$NOTSEEN" in
    *rogue_patch*) ng "⑤-b git diff 가 미추적 파일을 봤다 — 픽스처 전제가 바뀌었다" ;;
    *) ok "⑤-b git diff 는 그 파일을 못 본다 — 미추적 검출이 없으면 놓치는 자리다" ;;
esac
rm -f "$REPO/$SKETCH/rogue_patch.ino"

# ── 6) 부정 회귀: 커밋된 두 번째 `.ino` ────────────────────────────────────
echo "void rogue() {}" > "$REPO/$SKETCH/rogue_patch.ino"
git -C "$REPO" add -A && git -C "$REPO" commit -qm "rogue ino"
expect_rc 1 "⑥ 기대 목록에 없는 .ino 가 커밋돼 있어도 FAIL" -- run
git -C "$REPO" reset -q --hard HEAD~1

# ── 7) 부정 회귀: `.h` — `.ino` 만 보던 구판이 통째로 놓치던 확장자 ─────────
echo "#define MAX_SPEED 99" > "$REPO/$SKETCH/motor_override.h"
expect_rc 1 "⑦ 스케치 폴더의 미추적 .h 도 FAIL (.ino 만 보면 뚫리는 자리)" -- run
rm -f "$REPO/$SKETCH/motor_override.h"

# ── 7-a) 공식 root 확장자 전수 — 목록과 구현의 항목별 대조 ─────────────────
for ext in pde c cpp S hpp hh tpp ipp; do
    probe="$REPO/$SKETCH/root_probe.$ext"
    echo "// root extension probe" > "$probe"
    expect_rc 1 "⑦-a root .$ext 미추적 소스 → FAIL" -- run
    rm -f "$probe"
done

# ── 7-b) 공식 src/** 확장자 전수 — 재귀 깊이까지 대조 ───────────────────────
mkdir -p "$REPO/$SKETCH/src/deep/nested"
for ext in c cpp S h hpp hh tpp ipp; do
    probe="$REPO/$SKETCH/src/deep/nested/src_probe.$ext"
    echo "// recursive src extension probe" > "$probe"
    expect_rc 1 "⑦-b src/** .$ext 미추적 소스 → FAIL" -- run
    rm -f "$probe"
done

# ── 7-c) ignore는 컴파일 제외 규칙이 아니다 ─────────────────────────────────
echo "*.cpp" > "$REPO/.gitignore"
echo "// ignored root source" > "$REPO/$SKETCH/ignored_root.cpp"
expect_rc 1 "⑦-c .gitignore에 걸린 root .cpp도 FAIL" -- run
rm -f "$REPO/$SKETCH/ignored_root.cpp"
echo "// ignored recursive source" > "$REPO/$SKETCH/src/deep/ignored_recursive.cpp"
expect_rc 1 "⑦-d .gitignore에 걸린 src/** .cpp도 FAIL" -- run
rm -f "$REPO/$SKETCH/src/deep/ignored_recursive.cpp" "$REPO/.gitignore"

# ── 7-d) data/**는 공식적으로 컴파일하지 않는다 — 오탐 역회귀 ──────────────
mkdir -p "$REPO/$SKETCH/data/deep"
echo "// source-looking documentation" > "$REPO/$SKETCH/data/deep/notes.cpp"
expect_rc 0 "⑦-e data/**의 source-looking 파일은 PASS (빌드 밖)" -- run
case "$LAST_OUT" in
    *"data/deep/notes.cpp"*) ok "⑦-f data/** 파일은 [참고]에 숨지 않고 나타난다" ;;
    *) ng "⑦-f data/** 파일이 [참고] 출력에서 사라졌다" ;;
esac
rm -f "$REPO/$SKETCH/data/deep/notes.cpp"

# ── 7-e) committed src/**도 기대 목록 밖이면 잡는다 ─────────────────────────
echo "// committed recursive source" > "$REPO/$SKETCH/src/deep/committed.cpp"
git -C "$REPO" add -A && git -C "$REPO" commit -qm "recursive source attack"
expect_rc 1 "⑦-g committed src/** .cpp도 FAIL" -- run
git -C "$REPO" reset -q --hard HEAD~1

# ── 8) 부정 회귀: 소스 삭제 ────────────────────────────────────────────────
rm -f "$REPO/$INO"
expect_rc 1 "⑧ 소스가 사라져도 FAIL" -- run
git -C "$REPO" reset -q --hard

# ── 9) 역회귀: 소유 문서만 바뀐 상태는 오염이 아니다 — 다만 보여는 준다 ────
echo "extra ownership note" >> "$REPO/firmware/VENDOR_DROP.md"
expect_rc 0 "⑨ VENDOR_DROP.md 단독 변경은 PASS (판정 대상이 아니다)" -- run
case "$LAST_OUT" in
    *"VENDOR_DROP.md"*) ok "⑨-b 그 변경이 [참고] 절에 **숨지 않고** 나타난다" ;;
    *) ng "⑨-b 문서 변경이 출력에서 사라졌다 — 빼는 것과 숨기는 것은 다르다" ;;
esac
git -C "$REPO" reset -q --hard

# ── 10) 판정 불능은 통과가 아니다 ──────────────────────────────────────────
expect_rc 2 "⑩ 기준점 커밋이 없으면 rc=2 (판정 불능)" -- \
    bash "$CHECK" --repo "$REPO" --baseline "0000000000000000000000000000000000000000" --expect "$EXPECT"
expect_rc 2 "⑩-b git 저장소가 아니면 rc=2" -- bash "$CHECK" --repo "$TMP" --baseline "$BASE"
expect_rc 2 "⑩-c --expect 형식이 틀리면 rc=2" -- \
    bash "$CHECK" --repo "$REPO" --baseline "$BASE" --expect "$INO=8"
expect_rc 2 "⑩-d --expect 경로가 소스 범위 밖이면 rc=2" -- \
    bash "$CHECK" --repo "$REPO" --baseline "$BASE" --expect "firmware/VENDOR_DROP.md=1,0"
expect_rc 2 "⑩-e --expect 증감이 숫자가 아니면 rc=2" -- \
    bash "$CHECK" --repo "$REPO" --baseline "$BASE" --expect "$INO=x,2"
expect_rc 2 "⑩-f --expect 경로가 중복되면 rc=2" -- \
    bash "$CHECK" --repo "$REPO" --baseline "$BASE" --expect "$EXPECT" --expect "$EXPECT"

# ── 11) 진짜 저장소 역회귀 (read-only) ─────────────────────────────────────
expect_rc 0 "⑪ 실제 저장소의 현행 상태는 PASS" -- bash "$CHECK" --repo "$ROOT"

echo
echo "PASS $PASS / FAIL $FAIL"
[ "$FAIL" -eq 0 ] || exit 1
exit 0
