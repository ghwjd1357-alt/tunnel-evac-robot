#!/usr/bin/env bash
# ============================================================
# test_harness_guards.sh — lib_e2e.sh 의 '유한 벽시계 상한' 보장 격리 단위 테스트
#   (07-24 §14 P1: read_param_float 복구 시퀀스 상한 + wait_state 벽시계 deadline)
#   (07-24 §15 P1: hard-kill(SIGTERM 무시도 종결) + wait_state daemon 복구의 남은-예산 배분)
#
# 배경: 0723검토 §14·§15 가 두 라운드에 걸쳐 표적 harness 로 부정 회귀를 재현해 불승인했다 —
#   ① read_param_float 의 복구용 `ros2 daemon stop/start` 가 무방비라 blocking 시 무한 행. (§14)
#   ② wait_state 가 sleep 누적으로만 예산을 재, state timeout 소비분을 빼먹어 벽시계 초과.    (§14)
#   ③ 모든 timeout 이 SIGTERM 만 보내, CLI 가 TERM 을 무시(trap '' TERM)하면 안 죽음.        (§15)
#   ④ wait_state 의 daemon kick 이 고정 timeout 5 라, 남은 예산과 무관하게 벽시계 초과.       (§15)
#   ⑤ mission_e2e alarm·stop·follow topic pub 3곳이 일반 timeout 으로 남아 TERM 무시 시 무한 행. (§16)
#   이 테스트가 그 다섯 부정 회귀를 영구히 박제한다. Gazebo·실 ROS 없이 **fake ros2 실행파일 +
#   state 스텁 + 벽시계 측정**만으로 돈다 (≈105초 — TERM-무시 케이스가 실제 hard-kill 유예까지
#   벽시계로 소모하므로 case 6=34s·7=13s·8=30s·9≈9s 가 실시간). 검토자도 같은 하네스로 재확인한다.
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
#   FAKE_PARAM_TRAP  : 1 이면 `param get` 이 SIGTERM 무시(trap '' TERM) + 300초 블록
#   FAKE_DAEMON_BLOCK: 1 이면 `ros2 daemon stop/start` 가 300초 블록 (TERM 응답형 무한 행)
#   FAKE_DAEMON_TRAP : 1 이면 `daemon` 이 SIGTERM 무시(trap '' TERM) + 300초 블록 (TERM 무시형)
#   FAKE_TOPIC_TRAP  : 1 이면 `topic pub` 이 SIGTERM 무시(trap '' TERM) + 300초 블록
cat > "$FAKEBIN/ros2" << 'EOF'
#!/usr/bin/env bash
if [ "${1:-}" = param ] && [ "${2:-}" = get ]; then
  [ "${FAKE_PARAM_TRAP:-0}" = 1 ] && { trap '' TERM; sleep 300; }   # TERM 무시 + 블록
  printf '%s\n' "${FAKE_PARAM_OUT:-}"; exit 0
fi
if [ "${1:-}" = daemon ]; then
  [ "${FAKE_DAEMON_TRAP:-0}"  = 1 ] && { trap '' TERM; sleep 300; } # TERM 무시 + 블록
  [ "${FAKE_DAEMON_BLOCK:-0}" = 1 ] && sleep 300                    # TERM 응답형 블록
  exit 0
fi
if [ "${1:-}" = topic ] && [ "${2:-}" = pub ]; then
  [ "${FAKE_TOPIC_TRAP:-0}" = 1 ] && { trap '' TERM; sleep 300; }   # TERM 무시 + 블록
  exit 0
fi
exit 0
EOF
chmod +x "$FAKEBIN/ros2"
export PATH="$FAKEBIN:$PATH"

# lib 소싱 (함수 정의 + trap). 실제 프로세스가 없어 cleanup·trap 은 무해한 no-op.
source "$HERE/lib_e2e.sh"

P=0; F=0
ok(){ echo "  ✓ $1"; P=$((P+1)); }
ng(){ echo "  ✗ $1"; F=$((F+1)); }

# ── 케이스 1: read_param_float — 복구용 daemon(TERM 응답형)이 블록해도 상한 내 빈 결과 ──
#   §14 P1-① 부정 회귀. 구판은 daemon 무방비라 여기서 300초 매달렸다. TERM 응답형이라
#   각 timeout 이 SIGTERM 으로 즉시 종결 → 정상 복구 경로(≈10s, 상한 26s) 를 확인한다.
echo "== 1: read_param_float — daemon 블록(TERM 응답형)에도 유한 종결"
t0=$SECONDS
out=$(FAKE_PARAM_OUT="" FAKE_DAEMON_BLOCK=1 read_param_float /controller_server X)
el=$((SECONDS-t0))
{ [ -z "$out" ] && [ "$el" -le 30 ]; } \
  && ok "빈 결과 + 벽시계 ${el}s ≤ 정상경로 26s(+여유) — 무한 행 봉쇄" \
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

# ── 케이스 6: read_param_float — SIGTERM 무시 param·daemon 도 함수 자체 상한 내 종결 ──
#   §15 P1-① 부정 회귀. GNU timeout 은 기본 TERM 만 보내, 대상이 trap '' TERM 하면 안 죽는다.
#   상위 hard cutoff 없이 hard_timeout 의 --kill-after(SIGKILL)만으로 종결하는지 검증한다.
#   실제 hard 상한 = (8+g)+(5+g)+(5+g)+(8+g) = 34s (g=2). 외부 45s 는 '회귀 시 테스트가
#   무한 행하지 않게' 하는 안전망일 뿐 — 함수가 정상이면 발동(rc 124/137)하지 않아야 한다.
echo "== 6: read_param_float — SIGTERM 무시 param·daemon 도 함수 자체 hard 상한(≤34s) 내 종결"
export -f read_param_float hard_timeout
export E2E_KILL_GRACE
t0=$SECONDS
out=$(FAKE_PARAM_OUT="" FAKE_PARAM_TRAP=1 FAKE_DAEMON_TRAP=1 \
      timeout --kill-after=2 45 bash -c 'read_param_float /controller_server X')
rc=$?
el=$((SECONDS-t0))
{ [ "$rc" != 124 ] && [ "$rc" != 137 ] && [ -z "$out" ] && [ "$el" -le 37 ]; } \
  && ok "TERM 무시에도 상위 cutoff(45s) 미발동·빈 결과·벽시계 ${el}s ≤ 34s(+스케줄링 여유) — 함수 자체 종결" \
  || ng "rc=$rc out='$out' el=${el}s — 함수 자체 hard 상한 미보장(§15 P1-①)"

# ── 케이스 7: wait_state — 예산 부족 시 daemon kick 을 생략하고 예산 내 FAIL(구 23s 앵커) ──
#   §15 P1 부정 회귀. 즉시 빈 상태 5연속으로 deadline 직전에 kick 을 유도한다. 구판은 고정
#   timeout 5 로 daemon stop/start 를 무조건 불러 예산 13→벽시계 23s. 보완판은 남은 예산(≈1s)이
#   복구 최소(6s)보다 작으면 복구를 아예 생략하고 13s 에 FAIL 한다. FAKE_DAEMON_BLOCK 을 켜
#   'kick 이 불렸다면 블록됐을' 상황을 만들어 두고, 생략 로직이 그걸 막는지 본다.
echo "== 7: wait_state — 예산 부족 시 daemon kick 생략, 예산 내 FAIL(구 23s 회귀 아님)"
t0=$SECONDS
( export FAKE_DAEMON_BLOCK=1
  state(){ echo ""; }            # 즉시 빈 상태 → 5연속으로 kick 유도
  fail(){ echo "$1"; exit 1; }
  wait_state SEARCH_BACK 13 ) >/dev/null 2>&1
el=$((SECONDS-t0))
[ "$el" -le 16 ] \
  && ok "예산 부족분 복구 생략 → 벽시계 ${el}s ≤ 13s(+허용) — 구 23s 회귀 아님" \
  || ng "벽시계 ${el}s — daemon 복구가 예산 밖(§15 P1 회귀, 구 대조 23s)"

# ── 케이스 8: wait_state — 예산 충분 시 kick 이 불려도(+TERM 무시 daemon) 예산 내 수렴 ──
#   §15 P1 역·부정 회귀 결합. rem 이 충분해 kick 이 실제로 불리고, 그 daemon 이 TERM 을
#   무시(FAKE_DAEMON_TRAP)해도 hard_timeout 의 SIGKILL + 남은-예산 배분으로 벽시계 예산을
#   넘기지 않아야 한다. 구판이면 고정 timeout 5 가 TERM 무시에 안 죽어 수백 초로 폭주한다.
echo "== 8: wait_state — 예산 충분 시 kick(+TERM 무시 daemon)도 예산 내 수렴"
t0=$SECONDS
( export FAKE_DAEMON_TRAP=1
  state(){ echo ""; }            # 즉시 빈 상태 → kick 유도(rem 충분)
  fail(){ echo "$1"; exit 1; }
  wait_state SEARCH_BACK 30 ) >/dev/null 2>&1
el=$((SECONDS-t0))
[ "$el" -le 34 ] \
  && ok "kick+TERM 무시 daemon 이어도 벽시계 ${el}s ≤ 30s(+허용) — 복구가 남은 예산 안에서 수렴" \
  || ng "벽시계 ${el}s — kick daemon 복구가 예산 밖(§15 P1 회귀: hard-kill/예산배분 미보장)"

# ── 케이스 9: mission topic pub 3종 — TERM 무시도 hard-timeout 으로 유한 종결 ──
#   §16 P1 부정 회귀. alarm·stop·follow 가 일반 timeout 12 로 남으면 TERM 무시 CLI 를 못 죽인다.
#   격리에선 상한을 1s 로 축소해 각 호출이 1s+유예(2s) 안에 rc=137 로 종결하는지 확인한다.
echo "== 9: mission topic pub 3종 — TERM 무시도 hard-timeout 으로 유한 종결"
topic_hard_ok=1
for topic_case in alarm stop follow; do
  t0=$SECONDS
  FAKE_TOPIC_TRAP=1 hard_timeout 1 ros2 topic pub --times 2 -w 1 \
    "/$topic_case" std_msgs/msg/String "{data: test}" >/dev/null 2>&1
  rc=$?
  el=$((SECONDS-t0))
  { [ "$rc" = 124 ] || [ "$rc" = 137 ]; } && [ "$el" -le 4 ] || topic_hard_ok=0
done
plain_pub=$(grep -cE '^[[:space:]]*timeout 12 ros2 topic pub' "$HERE/mission_e2e.sh" || true)
hard_pub=$(grep -cE '^[[:space:]]*hard_timeout 12 ros2 topic pub' "$HERE/mission_e2e.sh" || true)
{ [ "$topic_hard_ok" = 1 ] && [ "$plain_pub" = 0 ] && [ "$hard_pub" = 3 ]; } \
  && ok "TERM 무시 3종이 각 hard 상한 내 종결 + mission wiring hard_timeout 3/3" \
  || ng "topic_hard_ok=$topic_hard_ok plain_pub=$plain_pub hard_pub=$hard_pub — §16 P1 미종결"

# ── 케이스 10: mission topic pub 정상 경로 — alarm·stop·follow 모두 즉시 반환 ──
echo "== 10: mission topic pub 3종 — 정상 fake CLI 즉시 반환"
t0=$SECONDS
topic_normal_ok=1
for topic_case in alarm stop follow; do
  hard_timeout 1 ros2 topic pub --times 2 -w 1 \
    "/$topic_case" std_msgs/msg/String "{data: test}" >/dev/null 2>&1 || topic_normal_ok=0
done
el=$((SECONDS-t0))
{ [ "$topic_normal_ok" = 1 ] && [ "$el" -le 2 ]; } \
  && ok "정상 topic pub 3종 즉시 반환(${el}s)" \
  || ng "topic_normal_ok=$topic_normal_ok el=${el}s — 정상 발행 역회귀"

echo
echo "== 결과: PASS $P / FAIL $F =="
[ "$F" = 0 ]
