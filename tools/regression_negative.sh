#!/usr/bin/env bash
# ============================================================
# regression_negative.sh — 부정(negative) 회귀: "안 돼야 하는 게 안 되는가" (07-19 신설)
#
# 배경: 07-19 외부 AI 검토 반영. 기존 회귀 3종은 전부 '성공 경로'만 검증 —
#   안전 정책(allow_unknown:false + track_unknown_space:true, 벽 관통 차단)이
#   실제로 작동하는지는 아무도 안 물어봤다. 이 스크립트가 그 반대면 담당:
#   ① 지도 밖 goal        → 거부/실패해야 정상 (미탐사 공간 진입 금지)
#   ② 벽 너머 goal        → 거부/실패해야 정상 (벽 관통 경로 금지)
#   ③ 장애물로 막힌 goal  → 성공하면 안 되고, '무한 회복 반복'에도 안 빠져야 함
#      (planner tolerance 0.25 > goal tolerance 0.15 트레이드오프의 감시 지점 —
#       막힌 목표에서 "계획 성공→도달 실패→회복 무한"이 나면 여기서 검거)
#   ④ 이후 정상 goal      → 성공해야 정상 (실패 3연타 뒤에도 시스템 생존 확인.
#      이게 없으면 "다 실패해서 ①~③ 통과"인 고장 상태와 구분 불가)
#
# 사용: bash ~/ros2_ws/tools/regression_negative.sh [추가 런치인자...]
# 판정: 마지막 줄 PASS / FAIL. (약 3~5분 소요)
#   ★ 기본 = localization 운영 모드 (테스트는 운영 구성을 따라간다 원칙).
#
# 노하우 박제:
#   - pkill/pgrep 자기매칭 자살 방지 → 브래킷 트릭 "ros2[ ]launch"
#   - 스폰 서비스는 멱등으로 (already exists = 실패 아님)
#   - 검증은 believed(TF) 아닌 ground truth(gz model) 로
# ============================================================
# ⚠ set -u 는 source 뒤에! (setup.bash 가 미정의 변수 참조 → unbound variable 즉사)
# ⚠ 전용 시뮬 PC 전용 (S2-4, Codex §11.4): cleanup 이 전역 pkill 로 gzserver·
#   nav2·slam 등을 프로세스 이름으로 죽인다 — 다른 ROS 작업이 도는 PC/Jetson
#   에서 실행 금지. Ctrl+C 중단 시에도 trap 이 좀비를 정리한다.
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
set -u

EXTRA_ARGS=("$@")
LOGDIR=$(mktemp -d /tmp/regneg.XXXX)
echo "로그: $LOGDIR"

cleanup() {
  pgrep -f "ros2[ ]launch" | xargs -r kill -9 2>/dev/null
  pkill -9 -x gzserver 2>/dev/null; pkill -9 -x gzclient 2>/dev/null
  pkill -9 -f "slam[_]toolbox" 2>/dev/null
  pkill -9 -f "robot_state[_]publisher" 2>/dev/null
  pkill -9 -f "lib/nav2[_]" 2>/dev/null   # launch 부모 kill -9 의 고아 nav2 노드까지
  pkill -9 -x ekf_node 2>/dev/null
  pkill -9 -f "spawn[_]entity" 2>/dev/null
  sleep 1
}
fail() { echo "== FAIL: $1 (로그: $LOGDIR)"; cleanup; exit 1; }
# ★ S2-4: Ctrl+C/kill 로 끊겨도 좀비(고아 nav2 등)를 안 남기게
# ★ F4 (Codex §12.6): cleanup 후 반드시 exit — 없으면 Ctrl+C 뒤에도 다음 단계 계속 실행
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM

# --- 부정 목표 전송: '성공하면 안 되고, 제한시간 안에 끝나야(무한 회복 금지)' ---
send_goal_expect_fail() {  # $1=x $2=y $3=제한시간(초) $4=라벨
  local out attempt
  for attempt in 1 2; do
    out=$(timeout "$3" ros2 action send_goal /navigate_to_pose \
      nav2_msgs/action/NavigateToPose \
      "{pose: {header: {frame_id: map}, pose: {position: {x: $1, y: $2}, orientation: {w: 1.0}}}}" 2>&1)
    echo "$out" > "$LOGDIR/$4.log"
    # ⚠ 응답 유실 함정(0705 §15.7 ③, 07-19 재발 실측): 오래 뜬 액션서버 + 새 CLI 는
    #   goal 응답을 떨굴 수 있다(bt_navigator "Failed to send goal response" — CLI 는
    #   영구 대기 → timeout). 수락/거부 흔적조차 없으면 '무한 회복'이 아니라
    #   '전송 실패' = 판정 무효 → 1회 재전송. (expect_success 와 동일한 방어)
    if echo "$out" | grep -qE "Goal accepted|rejected|REJECTED"; then
      break
    fi
    echo "  (시도 $attempt: goal 응답 유실 의심 — 재전송)"
  done
  if echo "$out" | grep -q "SUCCEEDED"; then
    fail "$4: 도달 성공 — 안전 정책 구멍! (성공하면 안 되는 목표)"
  fi
  # timeout 에 잘렸다 = 제한시간 내 종결 실패 = 무한 회복 반복 의심
  if ! echo "$out" | grep -qE "ABORTED|REJECTED|rejected|CANCELED"; then
    fail "$4: ${3}초 내 종결 안 됨 — 무한 회복 반복 의심 (막힌/불가 목표에서 헤어나오지 못함)"
  fi
  echo "  ✓ $4: 예상대로 실패 종결 ($(grep -oE 'ABORTED|REJECTED|rejected|CANCELED' <<<"$out" | head -1))"
}

# --- 정상 목표 전송 (regression_3goals 와 동일: 응답유실 대비 1회 재전송) ---
send_goal_expect_success() {  # $1=x $2=y $3=제한시간(초)
  local out
  for attempt in 1 2; do
    out=$(timeout "$3" ros2 action send_goal /navigate_to_pose \
      nav2_msgs/action/NavigateToPose \
      "{pose: {header: {frame_id: map}, pose: {position: {x: $1, y: $2}, orientation: {w: 1.0}}}}" 2>&1 | tail -1)
    if echo "$out" | grep -q SUCCEEDED; then return 0; fi
    echo "  (시도 $attempt 결과: $out — 재전송)"
  done
  return 1
}

echo "== ① 잔여 프로세스 정리 + 기동"
cleanup
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1
nohup ros2 launch tunnel_sim slam_nav2.launch.py gui:=false localization:=true "${EXTRA_ARGS[@]}" \
  > "$LOGDIR/launch.log" 2>&1 &

echo "== ② Nav2 활성화 대기 (최대 90초)"
T0=$SECONDS   # ★ G4 (Codex §13.5): sleep 누적 아닌 실경과시간 상한 (timeout 대기 미산입 구멍 봉쇄)
# ★ F4 (Codex §12.10): CLI 자체 timeout 없으면 hang 시 '최대 90초'가 무효
until timeout 8 ros2 param get /controller_server FollowPath.desired_linear_vel 2>/dev/null | grep -q Double; do
  sleep 3; [ $((SECONDS-T0)) -ge 90 ] && fail "Nav2 기동 타임아웃"
done
# F4: parameter 존재 ≠ lifecycle active — bt_navigator 활성까지 확인 ("inactive" 오매칭 방지 ^앵커)
until timeout 8 ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q "^active"; do
  sleep 3; [ $((SECONDS-T0)) -ge 120 ] && fail "bt_navigator 미활성 (lifecycle bringup 실패 의심 — launch 로그 확인)"
done
# ★ G4 (Codex §13.5): bt_navigator active ≠ action discovery 완료 —
#   navigate_to_pose 서버가 실제 떠 있는지 별도 확인 (goal 전송 전 마지막 관문)
until timeout 8 ros2 action info /navigate_to_pose 2>/dev/null | grep -q "Action servers: [1-9]"; do
  sleep 2; [ $((SECONDS-T0)) -ge 150 ] && fail "navigate_to_pose action server 미준비"
done
sleep 5   # 지도/TF 안정화

echo "== ③ 지도 밖 goal (0,15) — 미탐사 공간, 실패해야 정상"
send_goal_expect_fail 0.0 15.0 60 "offmap"

echo "== ④ 벽 너머 goal (6,6) — 메인복도 북벽(y=3) 너머 바위 속, 실패해야 정상"
send_goal_expect_fail 6.0 6.0 60 "throughwall"

echo "== ⑤ 장애물 스폰 (map 6,0 = world -6,0) — 키 1m 상자 (라이다에 보이는 높이)"
# ⚠ 콘(~0.3m)은 라이다 평면 아래 = 안 보임 (07-05 §8 실증) → 1m 상자 사용
cat > "$LOGDIR/box.sdf" <<'SDF'
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="neg_block_box">
    <static>true</static>
    <link name="link">
      <collision name="col"><geometry><box><size>0.4 0.4 1.0</size></box></geometry></collision>
      <visual name="vis"><geometry><box><size>0.4 0.4 1.0</size></box></geometry></visual>
    </link>
  </model>
</sdf>
SDF
# 멱등: 이미 있으면 성공 취급 (§14.3 함정 — already exists 를 실패로 보면 경고 폭주)
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity \
  "{name: neg_block_box, xml: '$(tr -d '\n' < "$LOGDIR/box.sdf")', initial_pose: {position: {x: -6.0, y: 0.0, z: 0.5}}}" \
  > "$LOGDIR/spawn.log" 2>&1 || true
# ⚠ service call 응답은 Python repr 형식(success=True) — "success: True" 아님 (첫 실행이 검거)
grep -qE "success=True|already exists" "$LOGDIR/spawn.log" || fail "장애물 스폰 실패 ($(tail -1 "$LOGDIR/spawn.log"))"
sleep 4   # costmap 이 /scan 으로 상자를 마킹할 시간

echo "== ⑥ 막힌 goal (6,0) — 상자 정중앙, 성공 금지 + 제한시간 내 종결 필수"
# 제한 240초 근거 (첫 실행 실측): 상자까지 주행 ~21s + BT 회복 1사이클 13~16s ×
# number_of_retries=6 ≈ 100~120s — 120초 제한은 정상 종결도 '무한 의심' 오탐.
# 240초에도 안 끝나면 그때가 진짜 무한 회복.
send_goal_expect_fail 6.0 0.0 240 "blocked"

echo "== ⑦ 정상 goal (3,0) — 실패 3연타 뒤에도 시스템이 살아있는지 (양성 대조군)"
send_goal_expect_success 3.0 0.0 120 || fail "정상 goal 미도달 — 부정 테스트 후 시스템 사망 의심"
read -r gx gy < <(gz model -m tunnel_robot -p 2>/dev/null | tail -1 | awk '{print $1, $2}')
err=$(python3 -c "import math; print(round(math.hypot(($gx+12)-3.0, $gy-0.0), 3))")
echo "  ✓ 정상 goal SUCCEEDED, 실위치 오차 ${err}m"
pass=$(python3 -c "print('yes' if $err <= 0.6 else 'no')")

cleanup
if [ "$pass" = "yes" ]; then
  echo "== PASS: 지도밖/벽너머/막힌 goal 전부 실패 종결 + 정상 goal 성공 (오차 ${err}m)"
else
  echo "== FAIL: 정상 goal 실위치 오차 ${err}m > 0.6m"
  exit 1
fi
