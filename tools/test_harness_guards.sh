#!/usr/bin/env bash
# ============================================================
# test_harness_guards.sh — lib_e2e.sh 의 '유한 벽시계 상한' 보장 격리 단위 테스트
#   (07-24 §14 P1: read_param_float 복구 시퀀스 상한 + wait_state 벽시계 deadline)
#
# 배경: 0723검토 §14 가 두 경로를 표적 harness 로 재현해 불승인했다 —
#   ① read_param_float 의 복구용 `ros2 daemon stop/start` 가 무방비라 blocking 시 무한 행.
#   ② wait_state 가 sleep 누적으로만 예산을 재, state timeout 소비분을 빼먹어 벽시계로 상한 초과.
#   이 테스트가 그 두 부정 회귀를 영구히 박제한다. Gazebo·실 ROS 없이 **fake ros2 실행파일 +
#   state 스텁 + 벽시계 측정**만으로 돈다 (≈15초). 검토자도 같은 하네스로 재확인할 수 있다.
#
# ★ 실행파일 stub 을 쓰는 이유: `timeout 8 ros2 …` 의 timeout 은 셸 함수를 실행하지 못하고
#   PATH 상의 실행파일만 부른다. 그래서 ros2 를 함수가 아니라 fake 실행파일로 주입한다.
# ============================================================
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
FAKEBIN=$(mktemp -d /tmp/harness_guard.XXXX)
trap 'rm -rf "$FAKEBIN"' EXIT

# --- fake ros2: 환경변수로 시나리오 제어 --------------------------------------
#   FAKE_PARAM_OUT   : `ros2 param get` 이 낼 문자열 (빈값 = 못 읽음)
#   FAKE_DAEMON_BLOCK: 1 이면 `ros2 daemon stop/start` 가 300초 블록 (무한 행 모사)
cat > "$FAKEBIN/ros2" << 'EOF'
#!/usr/bin/env bash
if [ "${1:-}" = param ] && [ "${2:-}" = get ]; then printf '%s\n' "${FAKE_PARAM_OUT:-}"; exit 0; fi
if [ "${1:-}" = daemon ]; then [ "${FAKE_DAEMON_BLOCK:-0}" = 1 ] && sleep 300; exit 0; fi
exit 0
EOF
chmod +x "$FAKEBIN/ros2"
export PATH="$FAKEBIN:$PATH"

# lib 소싱 (함수 정의 + trap). 실제 프로세스가 없어 cleanup·trap 은 무해한 no-op.
source "$HERE/lib_e2e.sh"

P=0; F=0
ok(){ echo "  ✓ $1"; P=$((P+1)); }
ng(){ echo "  ✗ $1"; F=$((F+1)); }

# ── 케이스 1: read_param_float — 복구용 daemon 이 무한 블록해도 상한 내 빈 결과 ──
#   §14 P1-① 부정 회귀. 구판은 daemon 무방비라 여기서 300초 매달렸다.
echo "== 1: read_param_float — daemon 무한 블록에도 유한 종결"
t0=$SECONDS
out=$(FAKE_PARAM_OUT="" FAKE_DAEMON_BLOCK=1 read_param_float /controller_server X)
el=$((SECONDS-t0))
{ [ -z "$out" ] && [ "$el" -le 30 ]; } \
  && ok "빈 결과 + 벽시계 ${el}s ≤ 상한 26s(+여유) — 무한 행 봉쇄" \
  || ng "out='$out' el=${el}s — 무한 행 미봉쇄 또는 오분류"

# ── 케이스 2: read_param_float — 정상값은 그대로(복구 미진입, 오분류 없음) ──
#   빈 결과를 '값 틀림'으로, 정상값을 '못 읽음'으로 뒤섞지 않는지.
echo "== 2: read_param_float — 정상값 0.12 는 즉시 그대로"
t0=$SECONDS
out=$(FAKE_PARAM_OUT="Double value is: 0.12" FAKE_DAEMON_BLOCK=1 read_param_float /controller_server X)
el=$((SECONDS-t0))
{ [ "$out" = "0.12" ] && [ "$el" -le 2 ]; } \
  && ok "0.12 즉시 반환(${el}s), 복구 미진입" \
  || ng "out='$out' el=${el}s"

# ── 케이스 3: wait_state — state 가 매번 timeout 을 소비해도 벽시계 예산 내 FAIL ──
#   §14 P1-② 부정 회귀. 구판(sleep 누적)이면 예산 3 → 벽시계 9s. 보완판은 벽시계로 재 ~3s.
echo "== 3: wait_state — state 소비형 flake 에도 벽시계 예산 내 FAIL"
t0=$SECONDS
( state(){ sleep "${1:-3}"; echo ""; }; fail(){ exit 1; }; wait_state SEARCH_BACK 3 ) >/dev/null 2>&1
el=$((SECONDS-t0))
[ "$el" -le 5 ] \
  && ok "벽시계 ${el}s 내 FAIL (예산3 + 허용치 — 구판 9s 회귀 아님)" \
  || ng "벽시계 ${el}s — 예산+허용치 초과(sleep 누적 회귀 의심)"

# ── 케이스 4a: wait_state — 예산 안 도달은 PASS ──
echo "== 4a: wait_state — 예산 안 목표 도달은 PASS"
( state(){ echo "TARGET"; }; fail(){ exit 1; }; wait_state TARGET 5 ) >/dev/null 2>&1 \
  && ok "예산 안 도달 → PASS" \
  || ng "예산 안 도달인데 PASS 아님"

# ── 케이스 4b: wait_state — 예산 밖(늦게) 도달은 FAIL, 메시지 모순 없음 ──
#   deadline 직후 전이는 FAIL 이어야 한다(§14 P1-② 지적). s 가 목표여도 벽시계 경과>예산이
#   메시지에 찍혀 옛 ⑧-a 자기모순("타임아웃인데 마지막=목표")이 재발하지 않는다.
echo "== 4b: wait_state — 예산 밖 도달은 FAIL + 경과 명시(모순 없음)"
msg=$( state(){ sleep 2; [ "$SECONDS" -ge "$((BASE+4))" ] && echo "TARGET" || echo ""; }; \
       fail(){ echo "$1"; exit 1; }; BASE=$SECONDS; wait_state TARGET 3 2>&1 )
if echo "$msg" | grep -q "타임아웃" && echo "$msg" | grep -q "경과"; then
  ok "예산 밖 도달 → FAIL, 경과 명시: $(echo "$msg" | tr '\n' ' ')"
else
  ng "예산 밖 도달 처리 오류: $msg"
fi

echo
echo "== 결과: PASS $P / FAIL $F =="
[ "$F" = 0 ]
