#!/usr/bin/env bash
# ============================================================
# test_gate_regression.sh — readiness_gate '조건 기동' 격리 회귀 (Gazebo·로봇 불필요)
#
# 배경: `tunnel_bringup` 의 readiness_gate 는 실차에서 **무엇이 언제 뜨는지**를 정하는
#   안전 경계인데, 실제로 처음 도는 곳은 로봇 위다. 회귀가 없으면 결함이 R0 현장에서
#   발견된다 — 가장 비싼 자리다. 그래서 토픽·TF·lifecycle 서비스·액션만 가짜로 주입해
#   로봇 없이 게이트를 실물 그대로(rclpy·DDS·런치 배선 포함) 돌린다.
#
# ★ 2층 구조인 이유
#   1층 = `src/tunnel_bringup/test/test_readiness_gate.py` (pytest, ms 단위, 결정적)
#         → 판정 '상태기'를 시간까지 조작해 시험한다. 5초 경계 같은 것은 여기서만
#           결정적으로 넘길 수 있다.
#   2층 = 이 파일 → 그 판정이 **실제 rclpy·DDS·ros2 launch 위에서도 같은지** 본다.
#         1층만 있으면 "가짜가 실물을 잘못 흉내 냈을 가능성"이 남는다.
#
# ★ 재현 대상 ① (2026-07-29 Codex 1차 P1): lifecycle 서버가 ACTIVE 를 **정확히 한 번**
#   답한 뒤 멎어도 게이트가 통과했다 → 반쪽 Nav2 위에서 미션이 기동. 케이스 6 이 그 입력이다.
#   ⚠ "처음부터 응답 없는" 서버(케이스 7)로는 이 결함을 못 잡는다 — 그 경로는 원래 실패한다.
# ★ 재현 대상 ② (2026-07-30 Codex 2차 P1): get_state 서비스가 잠깐 사라졌다 돌아오면
#   단절 전에 보낸 조회가 폐기되지 않아, 그 늦은 ACTIVE 1회 + 복구 후 ACTIVE 1회로
#   통과했다. 케이스 13 이 그 입력이고 14 가 대조군(복구되면 정상 통과)이다.
#   ⚠ 케이스 13·14 는 소실이 **관측됐는지**를 먼저 단언한다 — 관측 안 되면 그 케이스는
#     아무것도 시험하지 않은 것이므로 PASS 가 아니라 인프라 실패로 갈라 낸다.
#
# ⚠ 동시 실행 금지: 정리 단계가 nav2·slam·ekf 프로세스를 전역으로 죽인다(AGENTS.md §4).
#   Gazebo 가 떠 있으면 아예 시작하지 않는다.
#
# 실행: bash tools/test_gate_regression.sh          (≈2~3분)
#       GATE_TEST_DOMAIN=99 bash tools/...          (도메인 바꾸기)
#       GATE_SKIP_LAUNCH=1 bash tools/...           (11·12 런치 체인 생략 — 빠른 확인용)
# ============================================================
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$HERE/.." && pwd)"
FAKES="$WS/src/tunnel_bringup/test/gate_fakes.py"

# ★ 도메인 격리 — 다른 터미널의 ROS 그래프와 서로 안 보이게.
export ROS_DOMAIN_ID="${GATE_TEST_DOMAIN:-77}"

if pgrep -x gzserver >/dev/null 2>&1 || pgrep -x gzclient >/dev/null 2>&1; then
  echo "❌ gzserver/gzclient 실행 중 — E2E 를 끝내고 다시 실행할 것 (동시 실행 금지)."
  exit 2
fi
if [ ! -f "$FAKES" ]; then
  echo "❌ 가짜 조건 발생기를 못 찾음: $FAKES"; exit 2
fi

LOGDIR=$(mktemp -d /tmp/gate_reg.XXXX)
PIDS=()
LAUNCH_PID=""

stop_fakes() {
  if [ "${#PIDS[@]}" -gt 0 ]; then
    for p in "${PIDS[@]}"; do kill -9 "$p" 2>/dev/null; done
    wait 2>/dev/null
  fi
  PIDS=()
}

kill_launch() {
  [ -n "$LAUNCH_PID" ] && kill "$LAUNCH_PID" 2>/dev/null
  sleep 3
  # 브래킷 트릭: 패턴 자체가 자기 자신에 매칭되지 않게 (AGENTS.md §4)
  pkill -9 -f "lib/nav2[_]"            2>/dev/null
  pkill -9 -f "[s]lam_toolbox"         2>/dev/null
  pkill -9 -f "[e]kf_node"             2>/dev/null
  pkill -9 -f "[r]obot_state_publisher" 2>/dev/null
  pkill -9 -f "[r]eadiness_gate"       2>/dev/null
  [ -n "$LAUNCH_PID" ] && wait "$LAUNCH_PID" 2>/dev/null
  LAUNCH_PID=""
  return 0
}

cleanup() { stop_fakes; kill_launch; rm -rf "$LOGDIR"; }
trap cleanup EXIT

P=0; F=0
ok(){ echo "  ✓ $1"; P=$((P+1)); }
ng(){ echo "  ✗ $1"; F=$((F+1)); }

fake(){ python3 "$FAKES" "$@" > "$LOGDIR/fake_$$_${#PIDS[@]}.log" 2>&1 & PIDS+=($!); }

stf(){  # 정적 TF 하나 (부모 자식)
  ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 \
    --frame-id "$1" --child-frame-id "$2" > /dev/null 2>&1 &
  PIDS+=($!)
}

gate(){  # label timeout [추가 --ros-args 파라미터…] → 종료코드를 stdout 으로
  local label=$1 tmo=$2; shift 2
  timeout $(( ${tmo%.*} + 25 )) ros2 run tunnel_bringup readiness_gate --ros-args \
    -p use_sim_time:=false -p label:="$label" -p timeout_sec:="$tmo" \
    -p poll_period_sec:=0.5 "$@" > "$LOGDIR/gate_$label.log" 2>&1
  echo $?
}

wait_log(){  # 파일 패턴 제한초
  local f=$1 pat=$2 dl=$3 t0=$SECONDS
  while [ $((SECONDS - t0)) -lt "$dl" ]; do
    grep -q "$pat" "$f" 2>/dev/null && return 0
    sleep 1
  done
  return 1
}

# 런치 케이스용 가짜 지도 (내용은 비었다 — 여기서 보는 것은 '게이트 배선'이지 위치추정
# 품질이 아니다. slam_toolbox 는 지도 로드에 실패해도 계속 살아서 map→odom 을 발행한다는
# 것이 07-29 실측이고, 그래서 지도 '존재' 검사가 런치 앞단에 따로 있다).
: > "$LOGDIR/fakemap.posegraph"
: > "$LOGDIR/fakemap.data"

echo "== readiness_gate 격리 회귀 (ROS_DOMAIN_ID=$ROS_DOMAIN_ID) =="

# ── 1: 양성 — 조건이 다 차면 0 ───────────────────────────────────────────────
echo "== 1: 양성 — 토픽·TF·lifecycle·액션이 다 차면 종료코드 0"
fake sensors --with-filtered
fake lifecycle --node /controller_server:active --node /planner_server:active
fake action --name /navigate_to_pose
stf base_footprint lidar_link
stf odom base_footprint
sleep 4
rc=$(gate pos 25.0 \
  -p "require_topics:=['/scan','/odom','/imu/data','/odometry/filtered']" \
  -p "require_tf:=['base_footprint:lidar_link','odom:base_footprint']" \
  -p tf_fresh_sec:=0.0 \
  -p "require_lifecycle_active:=['/controller_server','/planner_server']" \
  -p "require_actions:=['/navigate_to_pose']")
[ "$rc" = 0 ] && ok "네 조건 전부 충족 → 0" || ng "rc=$rc (양성인데 통과 못 함)"
stop_fakes

# ── 2: 음성① 요구 토픽 없음 ─────────────────────────────────────────────────
echo "== 2: 음성① 요구 토픽이 없으면 1"
rc=$(gate no_topic 6.0 -p "require_topics:=['/scan']")
{ [ "$rc" = 1 ] && grep -q "수신 0건" "$LOGDIR/gate_no_topic.log"; } \
  && ok "없는 토픽 → 1 (사유: 수신 0건)" || ng "rc=$rc"

# ── 3: 음성②/N4 토픽 1건 뒤 정지 (예약 12 — '한 번 관측 = 통과' 금지) ───────
echo "== 3: 음성②/N4 토픽이 1건만 오고 끊기면 1"
fake sensors --messages 1 --delay 6     # 퍼블리셔 먼저, 발행은 구독 뒤에 딱 1건
rc=$(gate one_msg 14.0 -p "require_topics:=['/scan']")
# ★ '수신 0건이라 실패'와 '1건 받고도 실패'는 완전히 다른 결과다. 후자만 이 케이스의
#   합격이므로, 실제로 받았다는 증거를 사유 문자열로 요구한다 — '흐름 확인 n/2'(확인
#   중) 또는 'N.Ns 전이 마지막'(받았는데 끊김). 둘 다 _last_rx 가 있어야만 나온다.
{ [ "$rc" = 1 ] && grep -qE "흐름 확인|전이 마지막" "$LOGDIR/gate_one_msg.log"; } \
  && ok "1건 수신했지만 통과 못 함 ($(grep -oE '/scan\([^)]*\)' "$LOGDIR/gate_one_msg.log" | tail -1))" \
  || ng "rc=$rc / 사유: $(grep -o '미충족:.*' "$LOGDIR/gate_one_msg.log" | tail -1)"
stop_fakes

# ── 4: 음성③ 정적 TF 에 신선도 요구 (게이트 오사용 검출) ────────────────────
echo "== 4: 음성③ 정적 TF 에 신선도를 요구하면 1"
stf base_footprint lidar_link
sleep 3
rc=$(gate static_fresh 6.0 -p "require_tf:=['base_footprint:lidar_link']" -p tf_fresh_sec:=2.0)
[ "$rc" = 1 ] && ok "정적 TF + 신선도 요구 → 1 (오사용이 조용히 통과하지 않는다)" || ng "rc=$rc"
stop_fakes

# ── 5: 음성④ lifecycle INACTIVE ─────────────────────────────────────────────
echo "== 5: 음성④ lifecycle 이 ACTIVE 가 아니면 1"
fake lifecycle --node /planner_server:inactive
sleep 3
rc=$(gate inactive 8.0 -p "require_lifecycle_active:=['/planner_server']")
{ [ "$rc" = 1 ] && grep -q "inactive" "$LOGDIR/gate_inactive.log"; } \
  && ok "INACTIVE → 1" || ng "rc=$rc"
stop_fakes

# ── 6: 음성⑤/N1 — ACTIVE 1회 뒤 응답 정지 (★ Codex P1 재현 입력) ────────────
echo "== 6: 음성⑤/N1 ACTIVE 를 한 번 답한 뒤 멎으면 1 (경계 전후 모두 미통과)"
fake lifecycle --node /planner_server:active,hang
sleep 3
rc=$(gate hang_after 15.0 -p "require_lifecycle_active:=['/planner_server']")
before=$(grep -c "ACTIVE 확인 1/2" "$LOGDIR/gate_hang_after.log" || true)
after=$(grep -c "상태 응답" "$LOGDIR/gate_hang_after.log" || true)
{ [ "$rc" = 1 ] && [ "$before" -ge 1 ] && [ "$after" -ge 1 ]; } \
  && ok "1회 ACTIVE 로는 통과 못 함 — 경계 전 '확인 1/2' ${before}건 · 경계 후 '응답 지연' ${after}건" \
  || ng "rc=$rc before=$before after=$after (P1 미봉쇄)"
stop_fakes

# ── 7: N3 첫 조회부터 미응답 (기존 경로도 계속 실패해야 한다) ───────────────
echo "== 7: N3 첫 조회부터 응답이 없으면 1"
fake lifecycle --node /planner_server:hang
sleep 3
rc=$(gate never 8.0 -p "require_lifecycle_active:=['/planner_server']")
[ "$rc" = 1 ] && ok "무응답 → 1" || ng "rc=$rc"
stop_fakes

# ── 8: N2 ACTIVE 뒤 INACTIVE — 확인 상태 초기화 ─────────────────────────────
echo "== 8: N2 ACTIVE 뒤 INACTIVE 로 바뀌면 1"
fake lifecycle --node /planner_server:active,inactive
sleep 3
rc=$(gate flip 10.0 -p "require_lifecycle_active:=['/planner_server']")
{ [ "$rc" = 1 ] && grep -q "inactive" "$LOGDIR/gate_flip.log"; } \
  && ok "ACTIVE→INACTIVE → 1 (첫 ACTIVE 가 latch 되지 않는다)" || ng "rc=$rc"
stop_fakes

# ── 9: 액션 서버 없음 ───────────────────────────────────────────────────────
echo "== 9: 액션 서버가 없으면 1"
rc=$(gate no_action 6.0 -p "require_actions:=['/navigate_to_pose']")
{ [ "$rc" = 1 ] && grep -q "서비스 없음" "$LOGDIR/gate_no_action.log"; } \
  && ok "없는 액션 → 1" || ng "rc=$rc"

# ── 10: 런치 오설정 3종 — 노드를 0개 띄우고 종료 ────────────────────────────
echo "== 10: 런치 오설정 3종은 노드를 0개 띄우고 종료"
mis_ok=1; mis_detail=""
run_mis(){  # 라벨 인자…
  local label=$1; shift
  timeout 40 ros2 launch tunnel_bringup real_bringup.launch.py "$@" \
    > "$LOGDIR/mis_$label.log" 2>&1
  local started
  started=$(grep -c "process started" "$LOGDIR/mis_$label.log" || true)
  if [ "$started" != 0 ]; then mis_ok=0; mis_detail="$mis_detail $label=$started"; fi
}
run_mis nomap    map_file:=/nonexistent/no_such_map
run_mis with_ext "map_file:=$LOGDIR/fakemap.posegraph"
run_mis nowp     "map_file:=$LOGDIR/fakemap" mission:=true
[ "$mis_ok" = 1 ] \
  && ok "없는 지도 · 확장자 붙인 지도 · waypoints 없는 mission — 전부 process started 0" \
  || ng "노드가 떴다:$mis_detail"

# ── 13·14: 서비스 소실→복구 세대 경계 (★ Codex 2차 P1 의 경계 가드) ────────
#   ※ 번호는 '추가된 순서'다. 11·12(런치 양성 체인)는 물리적으로 이 뒤에서 돈다 —
#     기존 기록(0729_현황.md)의 케이스 번호를 보존하려고 재번호를 하지 않았다.
#   ※ 런치가 필요 없으므로 GATE_SKIP_LAUNCH=1 에서도 돈다.
#   ★ 정직하게: 이 두 케이스는 2차 P1 의 **검출기가 아니다.** '단절 전에 보낸 조회의 늦은
#     응답'은 서비스를 파괴하는 순간 DDS 에서 함께 소멸하므로 실물 층에서 만들 수 없다.
#     검출은 1층 pytest(test_n5_*, test_service_loss_removes_pending_request)가 하고,
#     여기서는 **실물 DDS 위에서도 소실→복구 경계의 관측 가능한 결과가 같은지**를 지킨다.
echo "== 13: N5 경계 — 서비스가 소실됐다 복구된 뒤 ACTIVE 1회만 답하면 1"
fake lifecycle --node /planner_server:active,active,hang --drop-at 1 --drop-sec 6.0
sleep 3
rc=$(gate gen_break 30.0 -p "require_lifecycle_active:=['/planner_server']")
# ★ 사전조건 단언 2종: 입력이 실제로 걸렸는가(가짜 로그) + 게이트가 실제로 관측했는가
#   (게이트 로그). 하나라도 없으면 이 케이스는 아무것도 시험하지 않은 것이므로
#   PASS 가 아니라 인프라 실패로 갈라 낸다 (조용히 무의미해지는 길 봉쇄).
#   ⚠ 게이트는 미충족 사유를 5초 주기로만 찍는다 → drop-sec 은 5 이상이어야 한다.
dropped=$(grep -c "get_state 파괴" "$LOGDIR"/fake_*.log 2>/dev/null | awk -F: '{s+=$2} END{print s+0}')
lost=$(grep -c "get_state 서비스 없음" "$LOGDIR/gate_gen_break.log" || true)
if [ "$dropped" = 0 ] || [ "$lost" = 0 ]; then
  ng "입력 미성립 (파괴=$dropped · 게이트 관측=$lost) — 케이스가 성립하지 않았다 [인프라 실패]"
else
  [ "$rc" = 1 ] \
    && ok "소실 관측 ${lost}건 → 복구 후 ACTIVE 1회로는 통과 못 함" \
    || ng "rc=$rc — 소실 경계를 넘어 확인이 누적됐다"
fi
stop_fakes

echo "== 14: R3 역회귀 — 소실됐다 복구해 새 ACTIVE 2회를 답하면 0"
fake lifecycle --node /planner_server:active --drop-at 1 --drop-sec 6.0
sleep 3
rc=$(gate gen_recover 35.0 -p "require_lifecycle_active:=['/planner_server']")
lost=$(grep -c "get_state 서비스 없음" "$LOGDIR/gate_gen_recover.log" || true)
if [ "$lost" = 0 ]; then
  ng "소실이 관측되지 않음 — 대조군이 성립하지 않았다 [인프라 실패]"
else
  { [ "$rc" = 0 ] && grep -q "준비 완료" "$LOGDIR/gate_gen_recover.log"; } \
    && ok "소실 관측 ${lost}건 뒤 복구되면 정상 통과 ('한 번 끊기면 영영 못 뜬다'가 아니다)" \
    || ng "rc=$rc — 복구 후에도 통과 못 함 (보완이 과잉 차단)"
fi
stop_fakes

if [ "${GATE_SKIP_LAUNCH:-0}" = 1 ]; then
  echo "(11·12 런치 양성 체인 생략 — GATE_SKIP_LAUNCH=1)"
  echo; echo "== 결과: PASS $P / FAIL $F =="; [ "$F" = 0 ]; exit
fi

# ── 11: R2 역회귀 — real_mapping 양성 체인 ──────────────────────────────────
echo "== 11: R2 역회귀 — real_mapping 게이트 A 통과 후 SLAM 기동"
fake sensors
ros2 launch tunnel_bringup real_mapping.launch.py lidar:=false micro_ros:=false \
  > "$LOGDIR/mapping.log" 2>&1 &
LAUNCH_PID=$!
if wait_log "$LOGDIR/mapping.log" "SLAM 기동" 60; then
  ok "가짜 센서로 게이트 A 통과 → slam_toolbox 기동"
else
  ng "60s 안에 SLAM 기동 로그 없음 — 마지막 미충족: $(grep -o '미충족:.*' "$LOGDIR/mapping.log" | tail -1)"
fi
kill_launch; stop_fakes

# ── 12: R2 역회귀 — real_bringup 전체 체인 (게이트 A→B→C) ───────────────────
echo "== 12: R2 역회귀 — real_bringup 게이트 A→B→C 통과, 다음 단계 정확히 1회"
fake sensors
ros2 launch tunnel_bringup real_bringup.launch.py "map_file:=$LOGDIR/fakemap" \
  lidar:=false micro_ros:=false > "$LOGDIR/bringup.log" 2>&1 &
LAUNCH_PID=$!
if wait_log "$LOGDIR/bringup.log" "Nav2 준비 완료" 150; then
  passed=$(grep -c "✅ 준비 완료" "$LOGDIR/bringup.log" || true)
  once=$(grep -c "Nav2 준비 완료" "$LOGDIR/bringup.log" || true)
  { [ "$passed" = 3 ] && [ "$once" = 1 ]; } \
    && ok "게이트 3개(A·B·C) 통과 + 다음 단계 정확히 1회 기동" \
    || ng "게이트 통과 $passed 건(예상 3) / 다음 단계 $once 회(예상 1)"
else
  ng "150s 안에 Nav2 준비 완료 없음 — 마지막 미충족: $(grep -o '미충족:.*' "$LOGDIR/bringup.log" | tail -1)"
fi
kill_launch; stop_fakes

echo
echo "== 결과: PASS $P / FAIL $F =="
[ "$F" = 0 ]
