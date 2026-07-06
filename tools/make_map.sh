#!/usr/bin/env bash
# ============================================================
# make_map.sh — 정본 지도 제작 자동화 (mapping 1회 → localization 운영의 재료)
#
# 하는 일: 라이브 SLAM(mapping)으로 4목표 코스를 돌아 터널 전체를 훑고
#   ① posegraph 저장 (slam_toolbox localization 모드가 읽는 파일) ★ 핵심 산출물
#      → maps/tunnel_localization.{posegraph,data}
#   ② pgm+yaml 저장 (사람 눈 확인·기록용) → maps/tunnel_map_loc.{pgm,yaml}
#
# 사용: bash tools/make_map.sh    (약 6분)
# 지도를 다시 만들 때(월드 변경 등)도 이 스크립트 한 번이면 끝 — 재현 가능.
#
# 노하우 박제: set -u 는 source 뒤 / pkill 브래킷 트릭 / send_goal 재전송 (§15.7)
# ============================================================
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
set -u

MAPDIR=~/ros2_ws/maps
LOGDIR=$(mktemp -d /tmp/makemap.XXXX)
echo "== 지도 제작 시작 — 로그: $LOGDIR"

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

echo "== ① 잔여 프로세스 정리 + 기동 (mapping 모드)"
cleanup
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1
nohup ros2 launch tunnel_sim slam_nav2.launch.py gui:=false \
  > "$LOGDIR/launch.log" 2>&1 &

echo "== ② Nav2 활성화 대기 (최대 90초)"
t=0
until ros2 param get /controller_server FollowPath.desired_linear_vel 2>/dev/null | grep -q Double; do
  sleep 3; t=$((t+3)); [ $t -ge 90 ] && fail "Nav2 기동 타임아웃"
done
sleep 5

echo "== ③ 4목표 커버리지 주행 (동쪽끝·곁복도·서쪽 복귀 = 터널 전체 훑기)"
send_goal 12.0 0.0 1.57 180  || fail "goal1(분기입구) 미도달"
echo "  ✓ goal1 분기입구"
send_goal 12.0 9.0 -1.57 180 || fail "goal2(곁복도) 미도달"
echo "  ✓ goal2 곁복도"
send_goal 13.5 0.0 3.14 240  || fail "goal3(동쪽끝) 미도달"
echo "  ✓ goal3 동쪽끝"
send_goal 1.0 0.0 0.0 240    || fail "goal4(서쪽 복귀) 미도달"
echo "  ✓ goal4 서쪽 복귀 (복도 재훑기 = 지도 다지기)"

echo "== ④ 저장: posegraph (localization 용) + pgm/yaml (기록용)"
mkdir -p "$MAPDIR"
# ⚠ 확장자 없이 — slam_toolbox 가 .posegraph/.data 를 알아서 붙임
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: $HOME/ros2_ws/maps/tunnel_localization}" \
  | grep -q "result=0" || fail "posegraph 저장 실패"
echo "  ✓ posegraph: $MAPDIR/tunnel_localization.{posegraph,data}"
timeout 30 ros2 run nav2_map_server map_saver_cli -f "$MAPDIR/tunnel_map_loc" \
  >> "$LOGDIR/mapsaver.log" 2>&1 || echo "  (pgm 저장 실패 — 기록용이라 계속)"
ls -l "$MAPDIR"/tunnel_localization.* 2>/dev/null || fail "posegraph 파일 없음"

cleanup
echo "== 완료 — localization 모드 재료 준비 끝"
