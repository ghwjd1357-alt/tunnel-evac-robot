#!/usr/bin/env bash
# ============================================================
# mission_e2e.sh — 미션 상태머신 E2E 회귀 테스트 (§14 성공 경로의 자동화)
#
# 시나리오 (0705_현황.md §14 run3 과 동일):
#   순찰 → 🔥알람 → APPROACH → GATHER(8초) → GUIDE(저속 유도)
#   → /follower_cmd stop (놓침 재현) → SEARCH_BACK (역행)
#   → /follower_cmd follow (재접근) → 재발견 → GUIDE 복귀 → ESCAPED
#
# 사용: bash ~/ros2_ws/tools/mission_e2e.sh [추가 런치인자...]      (T자, 약 5~8분, 헤드리스)
#       bash ~/ros2_ws/tools/mission_e2e.sh twin [추가 런치인자...] (쌍굴, 약 10~15분)
#   ★ 기본 = localization 운영 모드 (07-07: 테스트는 운영 구성을 따라간다 원칙).
#     라이브 SLAM(mapping)으로 검증하려면: bash tools/mission_e2e.sh localization:=false
#     (뒤에 준 인자가 기본값을 덮음 — ros2 launch 는 중복 인자 시 마지막 값 적용)
# 판정: 마지막 줄 PASS / FAIL.
#
# 노하우 박제 (0705_현황.md 함정들):
#   - topic pub 은 --once 금지 → -w 1(구독자 매칭 대기) + --times
#   - pkill 자기매칭 자살 방지 → 브래킷 트릭
#   - 자작 노드는 use_sim_time:=true 필수
# ============================================================
# ⚠ set -u 는 source 뒤에! — ROS setup.bash 내부가 미정의 변수를 참조해서
#   set -u 상태로 source 하면 "AMENT_TRACE_SETUP_FILES: unbound variable" 로 즉사.
# ⚠ 전용 시뮬 PC 전용 (S2-4, Codex §11.4): cleanup 이 전역 pkill 로 gzserver·
#   nav2·slam 등을 프로세스 이름으로 죽인다 — 다른 ROS 작업이 도는 PC/Jetson
#   에서 실행 금지. Ctrl+C 중단 시에도 trap 이 좀비를 정리한다.
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
set -u

# --- 월드 모드 (07-07): 첫 인자 twin = 쌍굴, 아니면 T자(기존 그대로) ---
MODE=tunnel
if [ "${1:-}" = "twin" ]; then MODE=twin; shift; fi
if [ "$MODE" = "twin" ]; then
  WORLD_ARGS=(world:=tunnel_twin.world spawn_x:=-17
              localization_params:=slam_params_localization_twin.yaml)
  WAYPOINTS_FILE="$HOME/ros2_ws/install/mission_manager/share/mission_manager/config/waypoints_twin.yaml"
  FIRE_X=30.0; FIRE_Y=0.0        # 1번 굴 동쪽 (map). 집결지 계산 = (22, 0)
  ESCAPE_WORLD="(-17, 0)"        # 탈출구 = 스폰 지점 (world)
  # 쌍굴은 순찰 루프가 길어(굴 2개 순회) 상태 대기 상한을 늘린다
  T_GATHER=420; T_ESCAPED=600
else
  WORLD_ARGS=()
  WAYPOINTS_FILE=""              # 빈값 = mission_node 기본(waypoints.yaml)
  FIRE_X=14.0; FIRE_Y=0.0
  ESCAPE_WORLD="(-12, 0)"
  T_GATHER=240; T_ESCAPED=300
fi

EXTRA_ARGS=("$@")           # 인자는 그대로 런치에 전달 (예: localization:=false)
LOGDIR=$(mktemp -d /tmp/mission_e2e.XXXX)
echo "로그: $LOGDIR (모드: $MODE)"

cleanup() {
  pgrep -f "ros2[ ]launch" | xargs -r kill -9 2>/dev/null
  pkill -9 -f "mission[_]node" 2>/dev/null
  pkill -9 -f "fake[_]follower" 2>/dev/null
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

state() {  # 현재 /mission_state 값 1개 읽기 (2Hz 발행이라 3초면 충분)
  timeout 3 ros2 topic echo /mission_state --once 2>/dev/null \
    | sed -n 's/^data: //p' | head -1
}

wait_state() {  # $1=원하는 상태 $2=제한시간(초) — FAULT 자동재시도는 통과시킴
  # ★ F7 방어 (07-19 실측): CLI echo 가 240초 내내 빈값('')인 flake —
  #   미션 노드 로그는 전 상태 정상 진행 = 코드 아닌 ros2 CLI/데몬 결함.
  #   빈 읽기 5연속(≈15s)이면 daemon 재시작 1회로 자가 복구 시도.
  local t=0 s empty=0 kicked=0
  while [ "$t" -lt "$2" ]; do
    s=$(state)
    if [ "$s" = "$1" ]; then echo "  ✓ $1 도달 (${t}s)"; return 0; fi
    if [ -z "$s" ]; then
      empty=$((empty+1))
      if [ "$empty" -ge 5 ] && [ "$kicked" = 0 ]; then
        echo "  (⚠ /mission_state 빈 읽기 ${empty}연속 — ros2 daemon 재시작으로 자가 복구 시도)"
        ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1
        kicked=1
      fi
    else
      empty=0
    fi
    sleep 3; t=$((t+3))
  done
  fail "$1 대기 타임아웃(${2}s), 마지막 상태='$(state)'"
}

echo "== ① 잔여 프로세스 정리 + 시뮬 기동"
cleanup
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1
nohup ros2 launch tunnel_sim slam_nav2.launch.py gui:=false localization:=true \
  "${WORLD_ARGS[@]}" "${EXTRA_ARGS[@]}" \
  > "$LOGDIR/launch.log" 2>&1 &

echo "== ② Nav2 활성화 대기 (최대 90초)"
T0=$SECONDS   # ★ G4 (Codex §13.5): sleep 누적 아닌 실경과시간 상한 (timeout 대기 미산입 구멍 봉쇄)
# ★ F4 (Codex §12.10): CLI 자체 timeout 없으면 hang 시 '최대 90초'가 무효
until timeout 8 ros2 param get /controller_server FollowPath.desired_linear_vel 2>/dev/null | grep -q Double; do
  sleep 3; [ $((SECONDS-T0)) -ge 90 ] && fail "Nav2 기동 타임아웃"
done
# F4 (Codex §12.10): parameter 존재 ≠ lifecycle active — bt_navigator 활성까지 확인
# ⚠ "inactive" 에 'active' 가 부분 문자열로 포함 → ^앵커 필수
until timeout 8 ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q "^active"; do
  sleep 3; [ $((SECONDS-T0)) -ge 120 ] && fail "bt_navigator 미활성 (lifecycle bringup 실패 의심 — launch 로그 확인)"
done
# ★ G4 (Codex §13.5): bt_navigator active ≠ action discovery 완료 —
#   navigate_to_pose 서버가 실제 떠 있는지 별도 확인 (goal 전송 전 마지막 관문)
until timeout 8 ros2 action info /navigate_to_pose 2>/dev/null | grep -q "Action servers: [1-9]"; do
  sleep 2; [ $((SECONDS-T0)) -ge 150 ] && fail "navigate_to_pose action server 미준비"
done
sleep 5

echo "== ③ 미션 노드 + 가짜 추종자 기동"
MISSION_ARGS=(-p use_sim_time:=true)
[ -n "$WAYPOINTS_FILE" ] && MISSION_ARGS+=(-p "waypoints_file:=$WAYPOINTS_FILE")
nohup ros2 run mission_manager mission_node --ros-args "${MISSION_ARGS[@]}" \
  > "$LOGDIR/mission.log" 2>&1 &
nohup ros2 run tunnel_sim fake_follower --ros-args -p use_sim_time:=true \
  > "$LOGDIR/follower.log" 2>&1 &
wait_state PATROL 30

echo "== ④ 추종자 스폰 확인"
T0=$SECONDS   # ★ G4 (Codex §13.5): sleep 누적 아닌 실경과시간 상한 (timeout 대기 미산입 구멍 봉쇄)
until grep -qE "스폰 완료|모델 접수" "$LOGDIR/follower.log"; do
  sleep 3; [ $((SECONDS-T0)) -ge 45 ] && fail "추종자 스폰 확인 실패"
done
echo "  ✓ 추종자 스폰"
sleep 10   # 순찰 이동 + 추종자 따라붙기 (모니터가 '봤다' 기록을 쌓게)

echo "== ⑤ 🔥 화재 알람 발사 → APPROACH"
# ★ 알람은 상태 전이 확인까지 재시도 (07-07): -w 1 + --times 로도 간헐 유실 실측
#   (mission.log 에 '알람' 0건). 전이 안 됐으면 그냥 한 번 더 쏘면 되는 멱등 신호.
for try in 1 2 3; do
  ros2 topic pub --times 2 -w 1 /alarm geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: $FIRE_X, y: $FIRE_Y}}}" >/dev/null 2>&1
  sleep 4
  [ "$(state)" = "APPROACH" ] && break
  echo "  (알람 시도 $try — 아직 PATROL, 재발사)"
done
wait_state APPROACH 20

echo "== ⑥ 집결(GATHER) → 유도(GUIDE) 대기"
wait_state GATHER "$T_GATHER"
wait_state GUIDE 60

echo "== ⑦ 유도 15초 진행 후 놓침 재현 (follower stop)"
sleep 5
# ★ S1-5 (07-19): 상태 전이만 보지 말고 GUIDE 저속이 '실제로' 적용됐는지 —
#   set_nav_speed 요청이 조용히 실패하면 사람 걸음 배려 없이 0.26 으로 유도하는 구멍
v=$(ros2 param get /controller_server FollowPath.desired_linear_vel 2>/dev/null \
    | grep -oE '[0-9]+\.[0-9]+')
if [ "$v" = "0.12" ]; then
  echo "  ✓ GUIDE 저속 0.12 m/s 실측 확인"
else
  fail "GUIDE 중 desired_linear_vel=$v ≠ 0.12 (속도 변경 미적용 — S1-5)"
fi
sleep 10
ros2 topic pub --times 3 -w 1 /follower_cmd std_msgs/msg/String \
  "{data: stop}" >/dev/null 2>&1
grep -q "stop" "$LOGDIR/follower.log" || echo "  (경고: follower 로그에 stop 미확인)"

echo "== ⑧ SEARCH_BACK 진입 대기 (거리 벌어짐 + 3초 디바운스)"
wait_state SEARCH_BACK 90

echo "== ⑨ 추종 재개 (follow) → 재발견 → GUIDE 복귀"
sleep 3
ros2 topic pub --times 3 -w 1 /follower_cmd std_msgs/msg/String \
  "{data: follow}" >/dev/null 2>&1
wait_state GUIDE 120
grep -q "재발견" "$LOGDIR/mission.log" && echo "  ✓ 미션 로그에 '재발견' 확인"

echo "== ⑩ 같이 탈출 → ESCAPED 대기"
wait_state ESCAPED "$T_ESCAPED"

echo "== ⑪ 최종 위치 (ground truth)"
robot=$(gz model -m tunnel_robot -p 2>/dev/null | tail -1 | awk '{printf "(%.2f, %.2f)", $1, $2}')
fol=$(gz model -m fake_follower -p 2>/dev/null | tail -1 | awk '{printf "(%.2f, %.2f)", $1, $2}')
echo "  로봇 world $robot / 추종자 world $fol  (탈출구 = world $ESCAPE_WORLD)"

cleanup
echo "== PASS: 놓침→역행→재발견→GUIDE 복귀→탈출 전 구간 완주"
