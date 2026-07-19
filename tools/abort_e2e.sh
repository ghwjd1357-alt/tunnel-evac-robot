#!/usr/bin/env bash
# ============================================================
# abort_e2e.sh — 주행 중 abort = '실제 정지' 통합 검증 (07-19 Codex §3.2 반영)
#
# 배경: P0 goal 취소 레이스 수정의 단위테스트는 "취소를 호출했다"까지만 증명.
#   Codex 지적 — "Nav2 가 취소를 수락했고 로봇이 실제로 멈췄다"는 아무도 검증
#   안 함. 이 스크립트가 그 마지막 조각: 진짜 Gazebo+Nav2 에서 주행 중 abort 를
#   쏘고 ① 상태=FAULT ② ground truth 위치가 실제로 정지 ③ /cmd_vel 잠잠
#   ④ mission 로그에 '취소 접수 확인' 을 전부 본다.
#
# 사용: bash ~/ros2_ws/tools/abort_e2e.sh [추가 런치인자...]   (약 2~3분, 헤드리스)
# 판정: 마지막 줄 PASS / FAIL. ★ 기본 = localization 운영 모드.
# ============================================================
# ⚠ set -u 는 source 뒤에! (setup.bash 가 미정의 변수 참조)
# ⚠ 전용 시뮬 PC 전용 (S2-4, Codex §11.4): cleanup 이 전역 pkill 로 gzserver·
#   nav2·slam 등을 프로세스 이름으로 죽인다 — 다른 ROS 작업이 도는 PC/Jetson
#   에서 실행 금지. Ctrl+C 중단 시에도 trap 이 좀비를 정리한다.
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
set -u

EXTRA_ARGS=("$@")
LOGDIR=$(mktemp -d /tmp/abort_e2e.XXXX)
echo "로그: $LOGDIR"

cleanup() {
  pgrep -f "ros2[ ]launch" | xargs -r kill -9 2>/dev/null
  pkill -9 -f "mission[_]node" 2>/dev/null
  pkill -9 -x gzserver 2>/dev/null; pkill -9 -x gzclient 2>/dev/null
  pkill -9 -f "slam[_]toolbox" 2>/dev/null
  pkill -9 -f "robot_state[_]publisher" 2>/dev/null
  pkill -9 -f "lib/nav2[_]" 2>/dev/null   # 고아 nav2(좀비 bt_navigator) 필수 청소
  pkill -9 -x ekf_node 2>/dev/null
  pkill -9 -f "spawn[_]entity" 2>/dev/null
  sleep 1
}
fail() { echo "== FAIL: $1 (로그: $LOGDIR)"; cleanup; exit 1; }
# ★ S2-4: Ctrl+C/kill 로 끊겨도 좀비(고아 nav2 등)를 안 남기게
# ★ F4 (Codex §12.6): cleanup 후 반드시 exit — 없으면 Ctrl+C 뒤에도 다음 단계 계속 실행
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM

state() {
  timeout 3 ros2 topic echo /mission_state --once 2>/dev/null \
    | sed -n 's/^data: //p' | head -1
}

robot_xy() {  # ground truth (believed 아님 — gz 실위치)
  gz model -m tunnel_robot -p 2>/dev/null | tail -1 | awk '{print $1, $2}'
}

echo "== ① 잔여 프로세스 정리 + 시뮬 기동"
cleanup
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1
nohup ros2 launch tunnel_sim slam_nav2.launch.py gui:=false localization:=true \
  "${EXTRA_ARGS[@]}" > "$LOGDIR/launch.log" 2>&1 &

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
sleep 5

echo "== ③ 미션 노드 기동 (추종자 불필요 — 순찰 주행만 쓰면 됨)"
nohup ros2 run mission_manager mission_node --ros-args -p use_sim_time:=true \
  > "$LOGDIR/mission.log" 2>&1 &
t=0
until [ "$(state)" = "PATROL" ]; do
  sleep 3; t=$((t+3)); [ $t -ge 30 ] && fail "PATROL 대기 타임아웃"
done
echo "  ✓ PATROL 진입"

echo "== ④ 주행 확인 대기 — 스폰(-12,0)에서 0.5m 이상 이동해야 '주행 중 abort'"
t=0
while true; do
  read -r rx ry < <(robot_xy)
  moved=$(python3 -c "import math; print(math.hypot($rx+12.0, $ry-0.0))")
  if python3 -c "exit(0 if $moved > 0.5 else 1)"; then
    echo "  ✓ 주행 중 (이동 ${moved}m, world ($rx, $ry))"; break
  fi
  sleep 3; t=$((t+3)); [ $t -ge 120 ] && fail "120초 내 주행 시작 안 함 (이동 ${moved}m)"
done

echo "== ⑤ ★ abort 발사"
ros2 topic pub --times 2 -w 1 /mission_cmd std_msgs/msg/String \
  "{data: abort}" >/dev/null 2>&1

echo "== ⑥ 상태 = FAULT 확인"
t=0
until [ "$(state)" = "FAULT" ]; do
  sleep 2; t=$((t+2)); [ $t -ge 20 ] && fail "abort 후 FAULT 미진입 (상태='$(state)')"
done
echo "  ✓ FAULT 진입"

echo "== ⑦ 실제 정지 확인 — 감속 여유 5초 후, 5초 간격 ground truth 2회 비교"
sleep 5
read -r x1 y1 < <(robot_xy)
sleep 5
read -r x2 y2 < <(robot_xy)
drift=$(python3 -c "import math; print(round(math.hypot($x2-$x1, $y2-$y1), 3))")
echo "  5초간 이동량: ${drift}m (($x1,$y1) → ($x2,$y2))"
python3 -c "exit(0 if $drift <= 0.10 else 1)" \
  || fail "abort 후에도 이동 계속 (${drift}m > 0.10m) — 취소가 실주행을 못 멈춤"
echo "  ✓ 정지 확인"

echo "== ⑧ /cmd_vel 잠잠 확인 (2초 수집 — 0 이 아닌 속도 명령이 없어야)"
timeout 2 ros2 topic echo /cmd_vel > "$LOGDIR/cmdvel.log" 2>/dev/null || true
nonzero=$(python3 - "$LOGDIR/cmdvel.log" <<'EOF'
import re, sys
txt = open(sys.argv[1]).read()
# linear/angular 의 x·y·z 전 성분 — 제자리 회전(angular.z)도 '움직임'이다
vals = [abs(float(v)) for v in re.findall(r'^\s+[xyz]: (-?[\d.eE+-]+)', txt, re.M)]
print(sum(1 for v in vals if v > 0.01))
EOF
)
[ "$nonzero" = "0" ] || fail "abort 후에도 /cmd_vel 에 속도 명령 ${nonzero}건"
echo "  ✓ /cmd_vel 잠잠"

echo "== ⑨ 미션 로그의 취소 감시 확인 (cancel 응답 확인 콜백이 실제로 돌았나)"
# ★ S2-5 (Codex §8.3): PASS 문구는 실제로 확인된 것만 말한다 — '접수 안 됨' 분기로
#   통과했는데 마지막 줄이 "취소 접수 확인"이라고 과장하던 불일치 수정
if grep -q "취소 접수 확인" "$LOGDIR/mission.log"; then
  echo "  ✓ '취소 접수 확인' 로그 존재 (Nav2 가 취소를 수락)"
  CANCEL_NOTE="취소 접수 확인"
elif grep -q "취소 접수 안 됨" "$LOGDIR/mission.log"; then
  # goal 이 abort 직전에 이미 끝난 드문 타이밍 — 정지 자체(⑦)는 이미 검증됨
  echo "  (참고: '취소 접수 안 됨' — goal 이 이미 종결된 타이밍이었을 수 있음)"
  CANCEL_NOTE="취소 접수는 미확인(이미 종결 추정) — 실정지로 판정"
else
  fail "미션 로그에 취소 응답 확인 흔적 없음 (감시 콜백 미동작 의심)"
fi

cleanup
echo "== PASS: 주행 중 abort → FAULT + 실정지(${drift}m) + cmd_vel 잠잠 + ${CANCEL_NOTE}"
