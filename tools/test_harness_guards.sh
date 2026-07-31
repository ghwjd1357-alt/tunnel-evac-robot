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
#   ⑥ (07-30 예약 4) abort_e2e ⑦ 실정지 단언이 깨지면 fail() 이 즉시 cleanup+exit 해
#      /cmd_vel 증거가 사라져 '코드 결함 vs 잔류 명령'을 사람이 손으로 갈라야 했다.
#      → 케이스 11·12 가 (a) 분류가 두 경우를 실제로 가르는지 (b) 수집이 fail 보다
#        먼저 배선돼 있는지를 함께 박제한다.
#   이 테스트가 그 부정 회귀들을 영구히 박제한다. Gazebo·실 ROS 없이 **fake ros2 실행파일 +
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
#   FAKE_TOPIC_INFO  : `ros2 topic info` 가 낼 문자열 (cmd_vel 타입·발행자 근거 검증)
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
if [ "${1:-}" = topic ] && [ "${2:-}" = info ]; then
  printf '%b' "${FAKE_TOPIC_INFO:-}"; exit 0
fi
exit 0
EOF
chmod +x "$FAKEBIN/ros2"

# --- fake gz: ground truth 조회 CLI 스텁 (07-30 예약 4 확대분) ------------------
#   FAKE_GZ_OUT  : `gz model -p` 가 낼 마지막 줄 (빈값 = 못 읽음)
#   FAKE_GZ_TRAP : 1 이면 SIGTERM 무시(trap '' TERM) + 300초 블록 — 실측된 무한 행 재현
cat > "$FAKEBIN/gz" << 'EOF'
#!/usr/bin/env bash
[ "${FAKE_GZ_TRAP:-0}" = 1 ] && { trap '' TERM; sleep 300; }
printf '%s\n' "${FAKE_GZ_OUT:-}"
exit 0
EOF
chmod +x "$FAKEBIN/gz"
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

# ── 케이스 11: 실정지 실패 분류 — /cmd_vel 잔류 '있음' → 코드 결함(취소 경로) ──
#   07-30 예약 4 부정 회귀 ①. 분류 코드를 넣었다는 것과 그 분류가 두 경우를 **가른다**는
#   것은 다른 명제다 — 그래서 가짜 cmdvel 덤프 2종으로 실제 분기를 확인한다.
#   여기선 linear.x=0.26 + angular.z=-0.35 (제자리 회전도 '움직임') = 잔류 2건.
echo "== 11: 실정지 실패 분류 — /cmd_vel 잔류 있음 → '코드 결함(취소 경로)'"
printf '0.26,0.0,0.0,0.0,0.0,-0.35\n' > "$FAKEBIN/cmdvel_residual.log"
n_res=$(cmdvel_nonzero "$FAKEBIN/cmdvel_residual.log")
msg_res=$(classify_stop_failure "$n_res")
{ [ "$n_res" = 2 ] && echo "$msg_res" | grep -q "코드 결함" && echo "$msg_res" | grep -q "2건"; } \
  && ok "잔류 2건 판독 → '$msg_res'" \
  || ng "n_res='$n_res' msg='$msg_res' — 잔류 있음이 코드 결함으로 분류되지 않음"

# ── 케이스 12: 실정지 실패 분류 — /cmd_vel '잠잠' → 잔류 명령/시뮬 특성 + 두 분류 상이 ──
#   07-30 예약 4 부정 회귀 ②. 세 가지를 한꺼번에 요구한다:
#   (a) 전 성분 0 은 0건으로 읽힌다  (b) 그때의 분류 문장이 케이스 11 과 **실제로 다르다**
#   (c) abort_e2e ⑦ 이 수집을 fail() **보다 먼저** 부른다 — 배선이 뒤집히면(수집이 fail 뒤로
#       가면) 함수는 멀쩡한데 증거는 여전히 사라진다. 그 회귀를 줄 번호 순서로 박제한다.
echo "== 12: 실정지 실패 분류 — /cmd_vel 잠잠 → '잔류 명령/시뮬' + 두 분류가 서로 다름 + 배선 순서"
printf '0.0,0.0,0.0,0.0,0.0,0.0\n' > "$FAKEBIN/cmdvel_quiet.log"
n_quiet=$(cmdvel_nonzero "$FAKEBIN/cmdvel_quiet.log")
msg_quiet=$(classify_stop_failure "$n_quiet")
# ⚠ 배선 단언은 `-F`(고정 문자열)로만 — 아래 케이스 15 주석의 07-31 실측 참조.
l_collect=$(grep -nF 'measure_cmdvel_residual "$LOGDIR/cmdvel_stopfail.log"' "$HERE/abort_e2e.sh" | head -1 | cut -d: -f1)
l_fail=$(grep -nF 'abort 후에도 이동 계속' "$HERE/abort_e2e.sh" | head -1 | cut -d: -f1)
if [ "$n_quiet" = 0 ] && echo "$msg_quiet" | grep -q "잔류 명령" \
   && [ "$msg_quiet" != "$msg_res" ] \
   && [ -n "$l_collect" ] && [ -n "$l_fail" ] && [ "$l_collect" -lt "$l_fail" ]; then
  ok "잔류 0건 → '$msg_quiet' (11과 상이) · abort_e2e 수집(L$l_collect) < fail(L$l_fail)"
else
  ng "n_quiet='$n_quiet' msg='$msg_quiet' 상이=$([ "$msg_quiet" != "$msg_res" ] && echo y || echo n) collect=L${l_collect:-없음} fail=L${l_fail:-없음} — 분류 미분기 또는 수집이 fail 뒤(증거 소실 회귀)"
fi

# ── 케이스 13: gz_model_xy — SIGTERM 무시 gz 도 유한 상한 + 정상 조회는 그대로(역회귀) ──
#   07-30 실측 부정 회귀. `gz model -p` 는 무방비면 **무한 행**한다 — 이번 세션
#   mission_e2e ⑪ 에서 약 11분 매달렸고 외부 kill 로만 풀렸다(고아는 20분+ 생존).
#   abort_e2e ④·⑦ 이 같은 호출을 쓰므로 같은 방식으로 영구 정지할 수 있었다.
#   실제 hard 상한 = 8 + E2E_KILL_GRACE(2) = 10s. 역회귀 앵커로 정상 조회도 같이 본다 —
#   상한을 씌우다 멀쩡한 좌표 읽기를 죽이면 그것대로 게이트가 거짓 FAIL 을 낸다.
echo "== 13: gz_model_xy — TERM 무시 gz 도 hard 상한(≤10s) 내 빈 결과 + 정상 조회 역회귀"
t0=$SECONDS
out_hang=$(FAKE_GZ_TRAP=1 gz_model_xy tunnel_robot)
el=$((SECONDS-t0))
out_ok=$(FAKE_GZ_OUT="-11.87 -0.06 0.05 0 0 3.14" gz_model_xy tunnel_robot)
{ [ -z "$out_hang" ] && [ "$el" -le 13 ] && [ "$out_ok" = "-11.87 -0.06" ]; } \
  && ok "TERM 무시 gz → 빈 결과·벽시계 ${el}s ≤ 10s(+여유) · 정상 조회 '$out_ok' 보존" \
  || ng "out_hang='$out_hang' el=${el}s out_ok='$out_ok' — 무한 행 미봉쇄 또는 정상 조회 역회귀"

# ── 케이스 14: cmdvel 판독기 fail-closed — 빈·경고문·필드누락·NaN/Inf 는 전부 '판독 실패' ──
#   07-31 §7.2 P1 부정 회귀 (Codex). 구판은 정규식에 걸린 값이 하나도 없어도 sum([])==0 을
#   찍어 **전부 '잔류 0건'** 으로 둔갑시켰고, ⑧ 에서는 그대로 PASS 였다(구현자 재현 확정).
#   ★ 케이스 11·12 는 완전한 zero/nonzero Twist 두 종만 봤다 — 세 번째 갈래를 아예 안 넣었다.
echo "== 14: cmdvel 판독기 fail-closed — 빈·경고문·필드누락·NaN·Inf 전부 '판독 실패'"
printf ''                                                                   > "$FAKEBIN/cv_empty.log"
printf 'WARNING: topic [/cmd_vel] does not appear to be published yet\n'     > "$FAKEBIN/cv_warn.log"
printf '0.0,0.0\n'                                                        > "$FAKEBIN/cv_short.log"
printf 'nan,0.0,0.0,0.0,0.0,0.0\n'                                      > "$FAKEBIN/cv_nan.log"
printf 'inf,0.0,0.0,0.0,0.0,0.0\n'                                      > "$FAKEBIN/cv_inf.log"
printf 'abc,0.0,0.0,0.0,0.0,0.0\n'                                      > "$FAKEBIN/cv_text.log"
# ★ 07-31 §8.2 P1 (Codex): 구판 케이스 14 는 경고문을 **발행자 인자 없이** 검사하고,
#   발행자 1 과 결합한 손상 입력은 두 종류만 봤다 — **두 축(손상 × 발행자 생존)이 만나는
#   교차 입력**이 통째로 빠져 `경고문 + 발행자 1` 이 '잔류 0건' 으로 승격되는 것을 못 잡았다.
#   → 아래는 손상 입력 전부를 **발행자 {없음, 0, 1} 세 축과 교차**시킨다.
printf 'garbage text that is not yaml at all\n'                              > "$FAKEBIN/cv_junk.log"
fc_ok=1; fc_note=""
for cv in cv_empty cv_warn cv_junk cv_short cv_nan cv_inf cv_text; do
  for pub in "" 0 1; do
    [ "$cv" = cv_empty ] && [ "$pub" = 1 ] && continue   # 진짜 빈 덤프 + 발행자 = 정상(케이스 15)
    n=$(cmdvel_nonzero "$FAKEBIN/$cv.log" $pub)
    m=$(classify_stop_failure "$n")
    { [ -z "$n" ] && echo "$m" | grep -q "분류 불가"; } \
      || { fc_ok=0; fc_note="$fc_note [$cv+pub'$pub'→'$n']"; }
  done
done
n_missing=$(cmdvel_nonzero "$FAKEBIN/does_not_exist.log")     # 수집 자체가 없던 경우
[ -z "$n_missing" ] || { fc_ok=0; fc_note="$fc_note [없는파일→'$n_missing']"; }
# ★ 수집 rc 가 정상이어도 덤프가 비면(발행자 근거 없이) 판독 실패여야 한다 — 판정 주체는 판독기다.
collect_cmdvel "$FAKEBIN/cv_collected.log" 1; coll_rc=$?
n_coll=$(cmdvel_nonzero "$FAKEBIN/cv_collected.log")
[ "$coll_rc" = 0 ] && [ -z "$n_coll" ] || { fc_ok=0; fc_note="$fc_note [수집rc=$coll_rc→'$n_coll']"; }
[ "$fc_ok" = 1 ] \
  && ok "손상 7종 × 발행자 3축 교차 + 없는 파일 + 빈 수집 전부 '판독 실패'→'분류 불가'" \
  || ng "판독기가 손상 입력을 0건으로 승격:$fc_note — §7.2·§8.2 P1 회귀"

# ── 케이스 15: cmdvel 판독기 역회귀 — 정상 CSV 입력 + 관측된 침묵 + ⑧ 배선 ──
#   조이다가 정상 판독을 죽이면 그것대로 게이트가 거짓 FAIL 을 낸다. 그리고 시간상자
#   출력이 YAML 여러 줄에서 **Twist 1개=CSV 1줄**로 바뀐다. PYTHONUNBUFFERED=1과 결합해
#   정상 시간상자 종료는 완전한 줄들만 남기며, 불완전 줄은 정상 꼬리로 승격하지 않고 실패한다.
echo "== 15: cmdvel 판독기 역회귀 — 정상 CSV 0건/nonzero + 관측된 침묵 + ⑧ 배선"
printf '0.0,0.0,0.0,0.0,0.0,0.0\n0.0,0.0,0.0,0.0,0.0,0.0\n' \
  > "$FAKEBIN/cv_zero.log"
printf '0.26,0.0,0.0,0.0,0.0,-0.35\n' > "$FAKEBIN/cv_res.log"
n_zero=$(cmdvel_nonzero "$FAKEBIN/cv_zero.log")
n_res2=$(cmdvel_nonzero "$FAKEBIN/cv_res.log")
# ★★ 07-31 실측 역회귀 — 이 저장소의 실제 '잠잠'은 zero Twist 가 아니라 **완전 침묵**이다
#    (abort 뒤 nav2 가 발행을 멈춘다 — 실덤프 0바이트를 직접 확인했다). 발행자가 살아 있는데
#    아무것도 안 왔으면 그것이 정상 PASS 다. 이걸 '판독 실패'로 두면 abort_e2e 가 영구 거짓 FAIL.
n_sil_ok=$(cmdvel_nonzero "$FAKEBIN/cv_empty.log" 1)
# ⑧ 배선: 판독 실패('')가 PASS 로 새지 않도록 '!= 0' 비교 + 같은 분류 함수를 쓰는가
# ⚠ 반드시 `-F`(고정 문자열). 07-31 실측: `grep -E` 의 `.*` 는 한글·em대시가 섞인 줄을
#   **못 넘어가 조용히 0 을 낸다** (`-F` 는 1, `-E` 는 0, LC_ALL 무관). 배선 단언에 정규식을
#   쓰면 검사가 '배선이 없어서'가 아니라 '패턴이 안 맞아서' 판정된다 — 이 저장소의 코드는
#   주석·메시지가 한글이라 이 함정을 상시 밟는다.
l_cmp=$(grep -cF '"$nonzero" != "0"' "$HERE/abort_e2e.sh" || true)
l_cls=$(grep -cF 'classify_stop_failure "$nonzero"' "$HERE/abort_e2e.sh" || true)
# ⑦·⑧ 이 **같은 단일 계약**(measure_cmdvel_residual)을 쓰는가 — 각자 조합하면 드리프트가 난다
l_uni=$(grep -cF 'measure_cmdvel_residual "$LOGDIR/' "$HERE/abort_e2e.sh" || true)
if [ "$n_zero" = 0 ] && [ "$n_res2" = 2 ] && [ "$n_sil_ok" = 0 ] \
   && [ "$l_cmp" = 1 ] && [ "$l_cls" = 1 ] && [ "$l_uni" = 2 ]; then
  ok "완전 CSV zero=0건 · 잔류=2건 · 관측된 침묵=0건 · ⑦⑧ 단일 계약 2/2 + '!=0' 비교"
else
  ng "n_zero='$n_zero' n_res='$n_res2' 침묵='$n_sil_ok' ⑧비교=$l_cmp ⑧분류=$l_cls 단일계약=$l_uni — 역회귀 또는 배선 이상"
fi

# ── 케이스 16: gz_model_xy 정상화 계약 — 비숫자·NaN/Inf·필드부족은 '좌표 없음' ──
#   07-31 §7.3 P1 부정 회귀 (Codex). 구판은 첫 두 토큰을 검증 없이 흘려보내
#   `model -m`·`nan nan`·`inf inf` 가 인프라 분기를 우회해 **실정지 실패로 오분류**됐다.
#   ⚠ 음수·소수는 정상 world 좌표다(스폰 -12,0) — 거부하면 그것대로 역회귀다.
echo "== 16: gz_model_xy — 비숫자·NaN·Inf·필드부족은 '좌표 없음', 정상 음수 좌표는 보존"
gz_ok=1; gz_note=""
for bad in "model -m" "nan nan" "inf inf" "-inf 0.0" "5.0" ""; do
  o=$(FAKE_GZ_OUT="$bad" gz_model_xy tunnel_robot)
  [ -z "$o" ] || { gz_ok=0; gz_note="$gz_note ['$bad'→'$o']"; }
done
o_neg=$(FAKE_GZ_OUT="-11.87 -0.06 0.05 0 0 3.14" gz_model_xy tunnel_robot)
o_pos=$(FAKE_GZ_OUT="1.40779 0.108277 0.05" gz_model_xy tunnel_robot)
{ [ "$gz_ok" = 1 ] && [ "$o_neg" = "-11.87 -0.06" ] && [ "$o_pos" = "1.40779 0.108277" ]; } \
  && ok "손상 6종 전부 좌표 없음 · 정상 음수 '$o_neg' · 정상 양수 '$o_pos' 보존" \
  || ng "gz_note=$gz_note o_neg='$o_neg' o_pos='$o_pos' — §7.3 P1 회귀 또는 정상 좌표 역회귀"

# ── 케이스 17: cmdvel **고정 CSV 계약** — YAML 자체 파싱 제거 + 타입 근거 ──
#   07-31 §9.2 P1: 미지 `metadata:` 부모 아래 y/z가 직전 angular의 누락 키를 채웠다.
#   근인은 YAML 구조를 줄 상태기로 다시 구현한 것. 수리는 ros2가 Twist를 역직렬화한 뒤 내는
#   고정 6열 CSV만 소비해 부모·들여쓰기·중복 키라는 입력 공간 자체를 제거한다.
echo "== 17: cmdvel 고정 CSV 계약 — 6열·유한값·Twist 타입 근거 외에는 전부 '판독 실패'"
printf 'linear:\n  x: 0.0\n  y: 0.0\n  z: 0.0\nangular:\n  x: 0.0\nmetadata:\n  y: 0.0\n  z: 0.0\n---\n' \
  > "$FAKEBIN/cv_foreign_parent.log"
printf '0.0,0.0,0.0,0.0,0.0\n'                         > "$FAKEBIN/cv_five.log"
printf '0.0,0.0,0.0,0.0,0.0,0.0,0.0\n'                 > "$FAKEBIN/cv_seven.log"
printf '0.0,0.0,0.0,0.0,0.0,0.0\nWARNING: damaged tail\n' > "$FAKEBIN/cv_bad_tail.log"
printf '0.0,0.0,0.0,0.0,0.0,0.0\n\n0.0,0.0,0.0,0.0,0.0,0.0\n' \
  > "$FAKEBIN/cv_blank_mid.log"
st_ok=1; st_note=""
for cv in cv_foreign_parent cv_five cv_seven cv_bad_tail cv_blank_mid; do
  for pub in "" 1; do            # ★ 발행자가 살아 있어도 손상은 손상이다
    n=$(cmdvel_nonzero "$FAKEBIN/$cv.log" $pub)
    m=$(classify_stop_failure "$n")
    { [ -z "$n" ] && echo "$m" | grep -q "분류 불가"; } \
      || { st_ok=0; st_note="$st_note [$cv+pub'$pub'→'$n']"; }
  done
done
# 역회귀: 완전한 CSV 레코드 2개는 그대로 0건
printf '0.0,0.0,0.0,0.0,0.0,0.0\n0.0,0.0,0.0,0.0,0.0,0.0\n' \
  > "$FAKEBIN/cv_two_zero.log"
n_two=$(cmdvel_nonzero "$FAKEBIN/cv_two_zero.log" 1)
# 침묵 근거는 발행자 존재뿐 아니라 토픽 타입까지 Twist여야 한다.
TI_OK='Type: geometry_msgs/msg/Twist\n\nNode name: nav\nTopic type: geometry_msgs/msg/Twist\nEndpoint type: PUBLISHER\nGID: aa.01\n'
TI_BAD='Type: std_msgs/msg/String\n\nNode name: nav\nTopic type: std_msgs/msg/String\nEndpoint type: PUBLISHER\nGID: aa.01\n'
p_twist=$(FAKE_TOPIC_INFO="$TI_OK"  cmdvel_pub_gids 2 0)
p_wrong=$(FAKE_TOPIC_INFO="$TI_BAD" cmdvel_pub_gids 2 0)
[ "$p_twist" = "aa.01" ] && p_twist=1      # 근거가 GID 가 됐다(§11.3 P1-②) — 판정 의미는 동일
csv_wiring=$(grep -cF 'geometry_msgs/msg/Twist --csv --no-lost-messages' "$HERE/lib_e2e.sh" || true)
{ [ "$st_ok" = 1 ] && [ "$n_two" = 0 ] && [ "$p_twist" = 1 ] && [ -z "$p_wrong" ] \
   && [ "$csv_wiring" = 1 ]; } \
  && ok "YAML 부모공격·CSV 열손상·임의꼬리 × 발행자 2축 실패 · 완전 2레코드=0 · Twist 타입 근거 + CSV 배선" \
  || ng "st_note=$st_note 완전2='$n_two' type_ok='$p_twist' type_bad='$p_wrong' wiring=$csv_wiring — §9.2 P1/고정 CSV 계약 회귀"

# ── 케이스 18: 증거 수집이 발행자 조회보다 **먼저** 온다 (§10.5 P2-① 부정 회귀) ──
#   구판은 `cmdvel_publisher_count`(daemon 복구 포함 최악 ≈26~34s)를 수집보다 먼저 불렀다.
#   ⑦ 실정지 단언이 깨진 직후는 잔류 명령이 흐르는 유일한 창인데 거기서 30초를 먼저 쓰면,
#   그 사이 잔류가 멎어 '코드 결함'이 '잔류 명령'으로 오분류된다.
#   ★ 배선이 아니라 **동작**으로 잡는다: `topic info` 가 TERM 을 무시하며 블록해도,
#     덤프에 내용이 있으면 조회를 아예 안 하므로 즉시 끝나야 한다. 구판이면 수십 초 걸린다.
echo "== 18: 증거 수집이 발행자 조회보다 먼저 — 조회가 블록해도 내용 있는 덤프는 즉시 판정"
cat > "$FAKEBIN/ros2_probe" << 'EOF'
#!/usr/bin/env bash
if [ "${1:-}" = topic ] && [ "${2:-}" = echo ]; then
  printf '0.26,0.0,0.0,0.0,0.0,-0.35\n'; exit 0        # 잔류 2건짜리 정상 CSV
fi
if [ "${1:-}" = topic ] && [ "${2:-}" = info ]; then
  trap '' TERM; sleep 300                               # 조회는 TERM 무시하며 블록
fi
if [ "${1:-}" = daemon ]; then trap '' TERM; sleep 300; fi
exit 0
EOF
chmod +x "$FAKEBIN/ros2_probe"
t0=$SECONDS
n_order=$( PATH="$FAKEBIN:$PATH"
           cp "$FAKEBIN/ros2" "$FAKEBIN/ros2.bak"; cp "$FAKEBIN/ros2_probe" "$FAKEBIN/ros2"
           measure_cmdvel_residual "$FAKEBIN/order_probe.log" 1
           cp "$FAKEBIN/ros2.bak" "$FAKEBIN/ros2" )
el=$((SECONDS-t0))
{ [ "$n_order" = 2 ] && [ "$el" -le 8 ]; } \
  && ok "내용 있는 덤프 → 발행자 조회 없이 ${el}s 만에 잔류 2건 판정 (구판이면 조회 블록에 수십 초)" \
  || ng "n='$n_order' el=${el}s — 수집보다 조회가 먼저(§10.5 P2-① 회귀) 또는 판독 이상"

# ── 케이스 19: 예약 17 — 게이트 5파일에 **상한 없는 외부 CLI 호출 0건** (전수 기계 검사) ──
#   07-30 실측: `gz model` 무상한 호출이 mission_e2e ⑪ 에서 11분 행을 만들었고, 사람이
#   죽여서야 `== PASS` 가 났다 — **개입해서 얻은 PASS 는 판정이 아니다.**
#   ★ '전수 점검'을 사람 기억이 아니라 기계가 지키게 한다.
#
#   ★ 07-31 §11.2 P1-① (검토) — 구판 검사기는 **하위 명령 화이트리스트**
#     (daemon|topic|service|param|gz model)였다. 그래서 실제로 남아 있던
#     `regression_negative.sh:44` 의 `timeout "$3" ros2 action send_goal` 을 못 보고
#     19/19 PASS 했다 — **거짓 녹색**. `ros2 lifecycle`·`ros2 action info` 도 같은 사각이었다.
#     → 검사기를 `scan_unbounded_cli.py` 로 분리하고 '모든 foreground ros2·gz' 로 뒤집었다.
#   ★ 07-31 §12.2 P1 (검토) — wrapper **옵션** 뒤가 사각이었다(`xargs -n1`·`env -i`·
#     `command --`·`time -p`). wrapper 문맥에서 첫 미지 토큰을 '다른 명령'으로 보고
#     성공 반환한 탓. → wrapper 뒤는 단위 끝까지 보수적으로 훑도록 바꿨다.
#   ★ 07-31 §13 (검토) — 남은 두 사각을 닫았다:
#     §13.2 P1 = **따옴표 하나면 사라졌다**. `y="$(gz model …)"` 처럼 큰따옴표 안의
#       명령치환은 마스킹에 통째로 지워져 상한 없는 `gz` 가 조용히 통과했다.
#       (게이트 5파일은 `"$( )"` 를 이미 10곳 넘게 쓴다 — 가정이 아니라 실사용 표기다.)
#     §13.3 P2 = wrapper **화이트리스트** 자체가 남은 구멍이었다(`setsid`·`nice`·`taskset`…).
#       → 목록을 지우고 단위 전체를 훑는다. 실측 8개 셸 도구 전량에서 증가 검출 0건.
#   ★ 07-31 §14.2 P1 (검토) — **`&` 라는 글자 하나에 네 가지 뜻이 섞여 있었다.**
#     배경 실행을 검사에서 빼는 근거는 오직 '블록하지 않는다'인데, 구판은 앞의 `>`(`2>&1`)만
#     예외로 두고 **뒤의 `>`** 는 안 봤다. 그래서 Bash 의 foreground 리다이렉션
#     `ros2 … &>/dev/null` · `gz … &>>log` 가 배경 실행으로 분류돼 단위 전체가 사라졌다 —
#     리다이렉션 표기 하나로 무상한 CLI 가 검사기에서 증발한 것이다(양성 20·21).
#     → 역할 판정을 `_amp_role()` 한 곳으로 모았다. 역회귀는 음성 11·12·13.
#   ★ 08-01 §15.2 P1 (검토) — 그 보완도 `>` 쪽만 봤다. **입력 FD 복제 `<&` 를 빠뜨렸다**
#     (양성 23·24·25 — `<&0`·`0<&3`·`<&-`. 셋 다 `bash -n` 유효 foreground 다).
#     같은 자리 **여섯 번째**. 표기를 하나씩 덧대던 것이 근인이고, 더 정확히는 `prev` 탐색이
#     **공백을 건너뛰고 있었다** — 리다이렉션 연산자에는 공백이 낄 수 없으므로 그 탐색 자체가
#     문법에 어긋났다. → **인접 한 글자만** 보게 해 `&` 의 네 자리를 규칙으로 닫았다.
#     ⚠ 배경 예외를 통째로 없애는 안은 **채택하지 않았다** — 실측 게이트 5파일 거짓 양성 7건
#       (전부 `nohup ros2 launch … &`). 예외는 정당하고 틀린 것은 판정 정확도였다.
#   ★ 08-01 §16.2 P1 (검토) — Bash 는 토큰화 전에 line continuation(백슬래시+개행)을
#     **제거**하지만, 검사기는 공백 두 칸으로 치환했다. 그래서 source 에서는 떨어져 있어도
#     논리행에서는 붙는 `<&`·`>&`·`&>`·`&&` 와 `ro`+`s2` 가 다시 사라졌다.
#     → 직접 표기 목록을 늘리지 않고, `_mask()` 가 Bash 와 같은 논리 입력을 만들게 한다.
#     아래 별도 픽스처는 양성 11(연산자 4 + 명령치환/wrapper 4 + 명령명 분할 1 +
#     큰따옴표 안에서 조립된 명령치환 1 + 주석 다음 물리행 1)과 음성 3(실제 background +
#     guarded 연산자 + 분할된 hard_timeout)을 함께 박제한다.
#     ⚠ 주석 안의 줄 끝 백슬래시는 continuation이 아니다 — Bash 실실행에서 다음 줄 `ros2`가
#       RAN을 출력했다. 첫 보완안은 주석까지 이어 가려 새 false-negative를 만들었고 되돌렸다.
#   ★ 검사기 자체도 검사한다. 검사기의 사각이 그대로 게이트의 사각이 된다 —
#     양성 19종·음성 10종 픽스처로 **검사기의 부정 회귀**를 박제한다.
#     ⚠ 음성 7·8·9 가 특히 중요하다: 변수 확장(`"${BASH_SOURCE[0]}"`)까지 열면 게이트
#       5파일이 즉시 거짓 양성으로 무너진다. '더 많이 잡는 것'이 항상 옳지는 않다.
echo "== 19: 예약 17 — 상한 없는 외부 CLI 전수 검사 + 검사기 자체의 부정 회귀"
SCAN="$HERE/scan_unbounded_cli.py"
unguarded=$(python3 "$SCAN" "$HERE"/{lib_e2e,abort_e2e,mission_e2e,regression_negative,regression_3goals}.sh)
cat > "$FAKEBIN/fixture.sh" << 'EOF'
timeout "$3" ros2 action send_goal /navigate_to_pose x        # 양성1: 일반 timeout
if gz model -m robot -p; then :; fi                           # 양성2: if 뒤
out=$(xargs ros2 topic echo /cmd_vel)                         # 양성3: xargs + 명령치환
v=`gz model -m robot`                                         # 양성4: 백틱
cat f | ros2 param get /a b                                   # 양성5: 파이프 뒤
xargs -n1 ros2 topic echo /cmd_vel                            # 양성6: wrapper 옵션
time -p ros2 topic info /cmd_vel                              # 양성7: keyword wrapper 옵션
command -- ros2 daemon stop                                   # 양성8: -- 구분자
env -i ros2 param get /n p                                    # 양성9: env 옵션
nohup -- ros2 topic echo /cmd_vel                             # 양성10: foreground nohup
exec -a probe ros2 topic info /cmd_vel                        # 양성11: exec 옵션+인자
sudo -n ros2 daemon stop                                      # 양성12: sudo 옵션
stdbuf -oL ros2 topic echo /cmd_vel                           # 양성13: stdbuf 옵션
y="$(gz model -m robot -p)"                                   # 양성14: 큰따옴표 안 명령치환
w="`gz model -m robot`"                                       # 양성15: 큰따옴표 안 백틱
hard_timeout 5 ros2 topic pub /t T "x: $(gz model -m r -p)"   # 양성16: 바깥 guarded·안쪽 무상한
setsid ros2 topic echo /cmd_vel                               # 양성17: 목록 밖 wrapper
nice -n 5 ros2 daemon stop                                    # 양성18: 〃 + 옵션
taskset -c 0 gz model -m robot -p                             # 양성19: 〃 + gz
ros2 topic echo /cmd_vel &>/dev/null                          # 양성20: &> 는 foreground 리다이렉션
gz model -m robot -p &>>log                                   # 양성21: &>> 〃
hard_timeout 5 ros2 daemon stop && ros2 topic echo /x         # 양성22: && 뒤는 별도 단위(무상한)
ros2 topic echo /cmd_vel <&0                                  # 양성23: <& 입력 FD 복제
gz model -m robot -p 0<&3                                     # 양성24: 〃 + FD 번호 접두
ros2 topic echo /x <&-                                        # 양성25: 〃 + FD 닫기
hard_timeout 5 ros2 daemon stop                               # 음성1
hard_timeout 5 env -i ros2 param get /n p                     # 음성2: guarded wrapper
until hard_timeout 8 ros2 lifecycle get /n | grep -q x; do :; done   # 음성3
nohup ros2 launch pkg a.launch.py > /dev/null 2>&1 &          # 음성4: 백그라운드
python3 -c 'print("timeout 3 ros2 topic echo /x")'            # 음성5: 인용 안쪽(작은따옴표)
#  timeout 9 ros2 topic echo /x                                 음성6: 주석
source "$(dirname "${BASH_SOURCE[0]}")/lib_e2e.sh"            # 음성7: 변수 확장은 열지 않는다
[ "$(state)" = "PATROL" ] && echo ok                          # 음성8: 명령치환이나 외부 CLI 아님
fail "원인: $(classify_stop_failure "$n")"                     # 음성9: 중첩 인용
hard_timeout "${2:-2}" env PYTHONUNBUFFERED=1 \
  ros2 topic echo /cmd_vel geometry_msgs/msg/Twist --csv \
  > "$1" 2>/dev/null                                          # 음성10: 다중행 guarded
ros2 topic echo /cmd_vel &                                    # 음성11: 진짜 background(블록 안 함)
hard_timeout 5 ros2 topic echo /x &>/dev/null                 # 음성12: guarded + &>
hard_timeout 5 ros2 daemon stop > "$1" 2>&1                   # 음성13: 2>&1 의미 유지
hard_timeout 5 ros2 topic echo /x <&0                         # 음성14: guarded + <&
EOF
fx_lines=$(python3 "$SCAN" "$FAKEBIN/fixture.sh" | cut -d: -f2 | cut -d' ' -f1 | paste -sd, -)
fx_want="1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25"
{ [ -z "$unguarded" ] && [ "$fx_lines" = "$fx_want" ]; } \
  && ok "게이트 5파일 상한 없는 외부 CLI 0건 · 검사기 픽스처 양성25(인용 안 명령치환 3·목록 밖 wrapper 3·리다이렉션 &>/&>>/<& 5 포함)/음성14 정확" \
  || ng "잔존='$unguarded' 픽스처검출='$fx_lines'(기대 $fx_want) — 예약 17 또는 검사기 회귀"

# line continuation은 quoted heredoc 안에서 실제 두 문자(백슬래시+개행)로 보존된다.
# 기대 줄은 외부 명령 토큰의 **원본 시작 줄**이다 — 논리행 결합 뒤에도 진단 위치를 지킨다.
echo "== 19b: Bash 논리행 — line continuation 제거 뒤 역할·토큰·주석을 판정한다"
cat > "$FAKEBIN/fixture_cont.sh" << 'EOF'
ros2 topic echo /cmd_vel <\
&0
gz model -m robot -p >\
&1
ros2 topic echo /cmd_vel &\
>/dev/null
ros2 topic echo /cmd_vel &\
& echo ok
out=$(xargs ros2 topic echo /cmd_vel <\
&0)
out=$(xargs ros2 topic echo /cmd_vel &\
>/dev/null)
env ros2 topic echo /cmd_vel >\
&1
env ros2 topic echo /cmd_vel &\
& echo ok
ro\
s2 topic echo /cmd_vel
x="$\
(ros2 topic echo /cmd_vel)"
# 주석 안의 백슬래시는 리터럴 — 다음 물리행 ros2 는 실행된다 \
ros2 topic echo /not_executed
ros2 topic echo /cmd_vel \
  &
hard_timeout 5 ros2 topic echo /x <\
&0
hard_time\
out 5 ros2 topic echo /x
EOF
cont_lines=$(python3 "$SCAN" "$FAKEBIN/fixture_cont.sh" | cut -d: -f2 | cut -d' ' -f1 | paste -sd, -)
cont_want="1,3,5,7,9,11,13,15,17,20,22"
{ bash -n "$FAKEBIN/fixture_cont.sh" && [ "$cont_lines" = "$cont_want" ]; } \
  && ok "line continuation 양성11/음성3 정확 · 논리행 결합 뒤 원본 시작 줄 보존 · 주석 다음 물리행 활성" \
  || ng "line continuation 계약 위반: 검출='$cont_lines'(기대 $cont_want) — §16.2 P1 회귀"

# ── 케이스 20: 침묵 근거는 **수집 창**에 묶인다 (§11.3 P1-② 부정 회귀) ──────────
#   구판은 수집이 끝난 뒤 발행자 '수'를 셌다. 수집 창엔 발행자가 없고 창 직후에만 생긴
#   경우(Nav2 재기동·세대 전환·DDS discovery 지연)에도 빈 덤프를 `0건`으로 승인했다 —
#   들을 대상이 없었는데 '정상 침묵'이 되는 거짓 PASS. 검토자·구현자가 각각 재현했다.
#   ★ 수가 아니라 GID 로 창 양끝을 브래킷한다. '수 1 → 수 1' 은 세대 전환을 통과시키지만
#     'GID A → GID B' 는 걸린다. 여기서 그 경계를 7종으로 박제한다.
echo "== 20: 침묵 근거의 관측 창 결합 — 창 밖 발행자는 근거로 쓰지 않는다"
#   ⚠ 07-31 §14.3: 구판 스텁은 pre/post 를 `retry` **인자**로 갈랐다(`[ "$2" = 0 ]`). 그건
#     구현의 배선에 결합된 것이라, 두 조회가 모두 retry=1 이 되자 스텁이 통째로 무의미해졌다
#     (항상 post 를 돌려줌). 계약은 '몇 번째 조회인가'지 '어떤 인자로 불렀나'가 아니다.
#     → **호출 순서**로 가른다. 서브셸 경계를 넘으므로 카운터는 파일이어야 한다.
run_bracket() {  # $1=창시작근거 $2=창끝근거 $3=덤프내용 → measure_cmdvel_residual 결과
  ( BR_PRE="$1"; BR_POST="$2"; BR_DUMP="$3"; BR_N="$FAKEBIN/br.n"; printf 0 > "$BR_N"
    collect_cmdvel()  { printf '%b' "$BR_DUMP" > "$1"; return 0; }
    cmdvel_pub_gids() {
      local c; c=$(($(cat "$BR_N" 2>/dev/null || echo 0) + 1)); printf '%s' "$c" > "$BR_N"
      if [ "$c" = 1 ]; then printf '%b' "$BR_PRE"; else printf '%b' "$BR_POST"; fi; }
    measure_cmdvel_residual "$FAKEBIN/br.log" 1 )
}
br_ok=1; br_note=""
br_expect() {  # $1=기대 $2=라벨 $3..=run_bracket 인자
  local want="$1" label="$2"; shift 2
  local got; got=$(run_bracket "$@")
  [ "$got" = "$want" ] || { br_ok=0; br_note="$br_note [$label→'$got'(기대'$want')]"; }
}
br_expect ''  "창중0·창후1"   'NONE'  'aa.01'          ''      # 창엔 없고 직후에만 생김
br_expect ''  "세대전환"      'aa.01' 'bb.02'          ''      # 죽고 새로 태어남
br_expect ''  "창중소실"      'aa.01' 'NONE'           ''      # 창 도중 사라짐
br_expect ''  "창시작조회실패" ''      'aa.01'          ''      # 근거 자체를 못 읽음
br_expect ''  "창끝조회실패"   'aa.01' ''               ''      # 〃
br_expect '0' "역회귀:진짜침묵" 'aa.01' 'aa.01'         ''      # 창 양끝 같은 발행자 = 관측된 침묵
br_expect '0' "역회귀:발행자추가" 'aa.01' 'aa.01\nbb.02' ''     # 새 발행자 추가는 무해(부분집합)
br_expect '2' "역회귀:내용있음"  'NONE'  'NONE' '0.26,0.0,0.0,0.0,0.0,-0.35\n'  # 근거 불필요
[ "$br_ok" = 1 ] \
  && ok "창 밖 발행자·세대 전환·소실·조회 실패 5종 fail-closed · 관측된 침묵/추가/내용 역회귀 3종 보존" \
  || ng "브래킷 계약 위반:$br_note — §11.3 P1-② 회귀"

# ── 케이스 21: 시작 스냅샷은 **승인 대상 창보다 먼저 확정**된다 (§14.3 P1 부정 회귀) ──
#   구판은 창-시작 조회를 수집과 **동시에** 띄웠다. 그건 두 작업이 같이 돈다는 것뿐이고,
#   그 조회가 그래프를 실제로 읽은 시점이 창 시작보다 앞이라는 보장이 아니다. 조회가 늦게
#   읽는 동안 A 가 사라지고 B 가 생기면 pre 도 post 도 B 가 되어 빈 덤프를 `0` 으로 승인했다.
#   ★ 케이스 20 은 이걸 못 잡는다 — 근거를 함수 시작 즉시 확정된 문자열로 주입하므로
#     **시간축 자체가 없다**. 그래서 구판도 20/20 PASS 였다. 여기서 그 축을 만든다.
#   ★ 값이 아니라 **순서**를 잰다. 스텁이 수집·조회의 시작/끝을 트레이스에 남기고,
#     승인 대상 창(C2)이 시작 스냅샷(G1)이 **끝난 뒤에** 열렸는지를 문자열로 단언한다.
#     구판 실측 트레이스: `C1< G1< C1> G1> …` — G1 이 C1 과 겹친다(= 결함).
#     신판 실측 트레이스: `C1< C1> G1< G1> C2< C2> G2< G2>` — 완전 순차.
#   ⚠ 조회 스텁을 수집보다 **느리게**(0.6s vs 0.3s) 둔다. 겹침이 있으면 반드시 드러나도록.
echo "== 21: 침묵 창의 시간 경계 — 시작 GID 스냅샷이 확정된 뒤에 승인 대상 창이 열린다"
TB_TR="$FAKEBIN/tb.trace"; TB_CN="$FAKEBIN/tb.cn"; TB_GN="$FAKEBIN/tb.gn"
tb_next() {  # 서브셸($(...))을 넘나드므로 카운터는 파일이다
  local c; c=$(($(cat "$1" 2>/dev/null || echo 0) + 1)); printf '%s' "$c" > "$1"; printf '%s' "$c"
}
run_tb() {  # $1=GID계획(|구분·조회순) $2=덤프계획(|구분·수집순) $3=복구흉내(1/0) → "결과|트레이스"
  : > "$TB_TR"; printf 0 > "$TB_CN"; printf 0 > "$TB_GN"
  local got
  got=$( TB_GP="$1"; TB_DP="$2"; TB_RECOVER="${3:-0}"
    collect_cmdvel() {
      local k; k=$(tb_next "$TB_CN"); printf 'C%s< ' "$k" >> "$TB_TR"
      sleep 0.3
      printf '%b' "$(printf '%s' "$TB_DP" | cut -d'|' -f"$k")" > "$1"
      printf 'C%s> ' "$k" >> "$TB_TR"; return 0; }
    cmdvel_pub_gids() {
      local k; k=$(tb_next "$TB_GN"); printf 'G%s< ' "$k" >> "$TB_TR"
      sleep 0.6
      printf 'G%s> ' "$k" >> "$TB_TR"
      # daemon 복구(retry=1)를 흉내낸다: 복구가 허용되면 **늦게, 다른 값으로** 성공한다.
      # 시작 스냅샷이 이걸 받아들이면 승인 창이 잔류 창에서 멀어진다 → 아래 'zz' 케이스가 잡는다.
      if [ "$k" = 1 ] && [ "${2:-1}" = 1 ] && [ "$TB_RECOVER" = 1 ]; then
        printf 'zz.99'; return; fi
      printf '%b' "$(printf '%s' "$TB_GP" | cut -d'|' -f"$k")"; }
    measure_cmdvel_residual "$FAKEBIN/tb.log" 0.3 )
  printf '%s|%s' "$got" "$(cat "$TB_TR")"
}
TB_FULL='C1< C1> G1< G1> C2< C2> G2< G2>'   # 침묵 경로: 수집 → 스냅샷 확정 → 검증 창 → 창끝
TB_EARLY='C1< C1> G1< G1>'                  # 시작 근거가 없으면 검증 창을 열 이유도 없다
TB_NOGID='C1< C1>'                          # 내용 있으면 조회 자체를 하지 않는다(케이스 18)
tb_ok=1; tb_note=""
tb_expect() {  # $1=기대결과 $2=기대트레이스 $3=라벨 $4=GID계획 $5=덤프계획 $6=복구흉내
  local want_r="$1" want_t="$2" label="$3" out got_r got_t
  out=$(run_tb "$4" "$5" "${6:-0}"); got_r="${out%%|*}"; got_t="${out#*|}"; got_t="${got_t% }"
  { [ "$got_r" = "$want_r" ] && [ "$got_t" = "$want_t" ]; } \
    || { tb_ok=0; tb_note="$tb_note [$label→'$got_r'/'$got_t'(기대'$want_r'/'$want_t')]"; }
}
CSV='0.26,0.0,0.0,0.0,0.0,-0.35\n'
tb_expect ''  "$TB_FULL"  "검증창중 세대전환"   'aa.01|bb.02'           '|'
tb_expect ''  "$TB_FULL"  "검증창중 소실"       'aa.01|NONE'            '|'
tb_expect ''  "$TB_FULL"  "창끝 조회실패"       'aa.01|'                '|'
tb_expect ''  "$TB_EARLY" "시작 발행자0"        'NONE|aa.01'            '|'
tb_expect ''  "$TB_EARLY" "시작 조회실패"       '|aa.01'                '|'
# ★ 구현자 실측(실제 DDS)이 잡은 **보완 1차안의 회귀**: 시작 스냅샷에 daemon 복구를 허용하면
#   조회가 최대 26s 늦게, 그 사이 새로 생긴 발행자로 성공한다. 논리는 닫히지만 승인 창이
#   잔류 수집 창에서 30초 떨어져 나가 abort **직후**의 침묵을 더 이상 묻지 않게 된다.
#   → 시작 스냅샷은 복구 없이 '지금'을 묻고 못 읽으면 fail-closed 다. 배선이 아니라 동작으로 잡는다.
tb_expect ''  "$TB_EARLY" "시작조회 실패→복구로 늦게 성공 금지" '|zz.99' '|' 1
tb_expect '0' "$TB_FULL"  "역회귀:관측된 침묵"  'aa.01|aa.01'           '|'
tb_expect '0' "$TB_FULL"  "역회귀:발행자 추가"  'aa.01|aa.01\nbb.02'    '|'
tb_expect '2' "$TB_NOGID" "역회귀:내용→조회0회" 'NONE|NONE'             "$CSV|"
tb_expect '2' "$TB_FULL"  "검증창에서 잔류 등장" 'aa.01|aa.01'          "|$CSV"
[ "$tb_ok" = 1 ] \
  && ok "승인 대상 창이 시작 스냅샷 확정 뒤에 열림(순차 트레이스 10종) · 근거 없으면 창 미개방 · 시작 스냅샷 복구 금지 · 내용은 조회 0회" \
  || ng "시간 경계 위반:$tb_note — §14.3 P1 회귀"

echo
echo "== 결과: PASS $P / FAIL $F =="
[ "$F" = 0 ]
