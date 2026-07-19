#!/usr/bin/env bash
# ============================================================
# regression_3goals.sh — Nav2 3목표 회귀 테스트 (§8 수동 절차의 자동화)
#
# 하는 일: 시뮬+SLAM+Nav2 를 헤드리스로 띄우고
#   goal1 분기입구(12,0 북향) → goal2 곁복도(12,9 남향) → goal3 동쪽끝(13.5,0)
#   을 차례로 보내 전부 SUCCEEDED 인지 + 최종 실위치 오차를 판정한다.
#
# 사용: bash ~/ros2_ws/tools/regression_3goals.sh [추가 런치인자...]
# 판정: 마지막 줄 PASS / FAIL. (약 4~6분 소요)
#   ★ 기본 = localization 운영 모드 (07-07: 테스트는 운영 구성을 따라간다 원칙).
#     라이브 SLAM(mapping)으로 검증하려면: bash tools/regression_3goals.sh localization:=false
#
# 노하우 박제 (0705_현황.md 함정들):
#   - pkill/pgrep 자기매칭 자살 방지 → 브래킷 트릭 "ros2[ ]launch"
#   - send_goal 응답 유실(bt_navigator timeout) → 타임아웃+1회 재전송
#   - 검증은 believed(TF) 아닌 ground truth(gz model) 로
# ============================================================
# ⚠ set -u 는 source 뒤에! — ROS setup.bash 내부가 미정의 변수를 참조해서
#   set -u 상태로 source 하면 "AMENT_TRACE_SETUP_FILES: unbound variable" 로 즉사.
# ⚠ 전용 시뮬 PC 전용 (S2-4, Codex §11.4): cleanup 이 전역 pkill 로 gzserver·
#   nav2·slam 등을 프로세스 이름으로 죽인다 — 다른 ROS 작업이 도는 PC/Jetson
#   에서 실행 금지. Ctrl+C 중단 시에도 trap 이 좀비를 정리한다.
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
set -u

EXTRA_ARGS=("$@")           # 인자는 그대로 런치에 전달 (예: localization:=false)
LOGDIR=$(mktemp -d /tmp/reg3goals.XXXX)
echo "로그: $LOGDIR"

cleanup() {
  pgrep -f "ros2[ ]launch" | xargs -r kill -9 2>/dev/null
  pkill -9 -x gzserver 2>/dev/null; pkill -9 -x gzclient 2>/dev/null
  pkill -9 -f "slam[_]toolbox" 2>/dev/null   # async(mapping)·localization 둘 다 매칭
  pkill -9 -f "robot_state[_]publisher" 2>/dev/null
  pkill -9 -f "lib/nav2[_]" 2>/dev/null   # ★ nav2 노드 전체 — launch 부모 kill -9 는 고아(좀비 bt_navigator)를 남김
  pkill -9 -x ekf_node 2>/dev/null
  pkill -9 -f "spawn[_]entity" 2>/dev/null
  sleep 1
}
fail() { echo "== FAIL: $1 (로그: $LOGDIR)"; cleanup; exit 1; }
# ★ S2-4: Ctrl+C/kill 로 끊겨도 좀비(고아 nav2 등)를 안 남기게
# ★ F4 (Codex §12.6): cleanup 후 반드시 exit — 없으면 Ctrl+C 뒤에도 다음 단계 계속 실행
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM

# --- 목표 전송: SUCCEEDED 대기, 응답유실 대비 1회 재전송 ---
send_goal() {  # $1=x $2=y $3=yaw $4=제한시간(초)
  local qz qw out
  qz=$(python3 -c "import math; print(math.sin($3/2))")
  qw=$(python3 -c "import math; print(math.cos($3/2))")
  for attempt in 1 2; do
    out=$(timeout "$4" ros2 action send_goal /navigate_to_pose \
      nav2_msgs/action/NavigateToPose \
      "{pose: {header: {frame_id: map}, pose: {position: {x: $1, y: $2}, orientation: {z: $qz, w: $qw}}}}" 2>&1 | tail -1)
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
t=0
# ★ F4 (Codex §12.10): CLI 자체 timeout 없으면 hang 시 '최대 90초'가 무효
until timeout 8 ros2 param get /controller_server FollowPath.desired_linear_vel 2>/dev/null | grep -q Double; do
  sleep 3; t=$((t+3)); [ $t -ge 90 ] && fail "Nav2 기동 타임아웃"
done
# F4 (Codex §12.10): parameter 존재 ≠ lifecycle active — bt_navigator 활성까지 확인
# ⚠ "inactive" 에 'active' 가 부분 문자열로 포함 → ^앵커 필수
until timeout 8 ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q "^active"; do
  sleep 3; t=$((t+3)); [ $t -ge 120 ] && fail "bt_navigator 미활성 (lifecycle bringup 실패 의심 — launch 로그 확인)"
done
sleep 5   # SLAM 첫 지도/TF 안정화

echo "== ③ goal1 분기입구 (12,0) 북향"
send_goal 12.0 0.0 1.57 180 || fail "goal1 미도달"
echo "  ✓ goal1 SUCCEEDED"

echo "== ④ goal2 곁복도 (12,9) 남향"
send_goal 12.0 9.0 -1.57 180 || fail "goal2 미도달"
echo "  ✓ goal2 SUCCEEDED"

echo "== ⑤ goal3 동쪽끝 (13.5,0)"
send_goal 13.5 0.0 0.0 240 || fail "goal3 미도달"
echo "  ✓ goal3 SUCCEEDED"

echo "== ⑥ ground truth 오차 판정 (goal3 기준)"
# map = world + (12,0) (스폰 world -12,0 이 map 0,0)
read -r gx gy < <(gz model -m tunnel_robot -p 2>/dev/null | tail -1 | awk '{print $1, $2}')
err=$(python3 -c "import math; print(round(math.hypot(($gx+12)-13.5, $gy-0.0), 3))")
echo "  실위치 world($gx,$gy) → 목표와 오차 ${err}m"
pass=$(python3 -c "print('yes' if $err <= 0.3 else 'no')")

cleanup
if [ "$pass" = "yes" ]; then
  echo "== PASS: 3목표 전부 SUCCEEDED, 최종 오차 ${err}m (허용 0.3m — S3-3: tolerance 0.15 의 2배, 07-19 강화)"
else
  echo "== FAIL: 도달은 했으나 실위치 오차 ${err}m > 0.3m (believed vs 실제 어긋남 의심)"
  exit 1
fi
