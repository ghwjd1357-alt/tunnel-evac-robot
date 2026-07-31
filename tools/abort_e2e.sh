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
# 공통 함수(cleanup·fail·trap·state·deadline_*·wait_nav2_ready·send_goal) = 같은 폴더의 라이브러리.
source "$(dirname "${BASH_SOURCE[0]}")/lib_e2e.sh"

EXTRA_ARGS=("$@")
LOGDIR=$(mktemp -d /tmp/abort_e2e.XXXX)
echo "로그: $LOGDIR"

robot_xy() {  # ground truth (believed 아님 — gz 실위치). 빈 출력 = 조회 실패(인프라).
  # ★ 07-30: 구판은 `gz model` 을 무방비로 불러 gz CLI 가 매달리면 ④·⑦ 이 상한 없이
  #   영구 정지했다(같은 호출이 mission_e2e ⑪ 에서 11분 행으로 실측). 상한은 lib 로 단일화.
  gz_model_xy tunnel_robot
}

echo "== ① 잔여 프로세스 정리 + 시뮬 기동"
cleanup
hard_timeout 5 ros2 daemon stop  >/dev/null 2>&1
hard_timeout 5 ros2 daemon start >/dev/null 2>&1   # ★ 예약 17: daemon 도 상한 안에서
nohup ros2 launch tunnel_sim slam_nav2.launch.py gui:=false localization:=true \
  "${EXTRA_ARGS[@]}" > "$LOGDIR/launch.log" 2>&1 &

wait_nav2_ready   # ② 3단 관문(param→lifecycle active→action discovery)·"최대 90초" = lib_e2e.sh

echo "== ③ 미션 노드 기동 (추종자 불필요 — 순찰 주행만 쓰면 됨)"
nohup ros2 run mission_manager mission_node --ros-args -p use_sim_time:=true \
  > "$LOGDIR/mission.log" 2>&1 &
deadline_start   # ★ G4: 실경과시간 상한(timeout 미산입 봉쇄) — lib_e2e.sh
until [ "$(state)" = "PATROL" ]; do
  sleep 3; deadline_exceeded 30 && fail "PATROL 대기 타임아웃"
done
echo "  ✓ PATROL 진입"

echo "== ④ 주행 확인 대기 — 스폰(-12,0)에서 0.5m 이상 이동해야 '주행 중 abort'"
deadline_start   # ★ G4: 실경과시간 상한(timeout 미산입 봉쇄) — lib_e2e.sh
while true; do
  read -r rx ry < <(robot_xy)
  if [ -z "${rx:-}" ] || [ -z "${ry:-}" ]; then
    # ★ 못 읽음은 '안 움직였다'가 아니다 (§5 ③ 인프라 vs 코드 결함 불혼동).
    #   같은 deadline 안에서만 재시도하고, 예산이 다하면 '판정 불가'로 끝낸다.
    echo "  (⚠ ground truth 조회 무응답 — gz model 상한 초과, 재시도)"
    sleep 3
    deadline_exceeded 120 && fail "ground truth(gz model) 무응답으로 주행 시작 판정 불가 — 인프라 결함(§5 ③), 주행/정지 어느 쪽도 주장하지 않음"
    continue
  fi
  moved=$(python3 -c "import math; print(math.hypot($rx+12.0, $ry-0.0))")
  if python3 -c "exit(0 if $moved > 0.5 else 1)"; then
    echo "  ✓ 주행 중 (이동 ${moved}m, world ($rx, $ry))"; break
  fi
  sleep 3; deadline_exceeded 120 && fail "120초 내 주행 시작 안 함 (이동 ${moved}m)"
done

echo "== ⑤ ★ abort 발사"
# ★ 예약 17: `-w 1` 은 구독자 매칭까지 **블록**한다 — mission_node 가 죽어 있으면 무한 대기다.
#   mission_e2e 의 alarm·stop·follow 3곳은 이미 hard_timeout 12 인데 여기만 무방비로 남아 있었다.
hard_timeout 12 ros2 topic pub --times 2 -w 1 /mission_cmd std_msgs/msg/String \
  "{data: abort}" >/dev/null 2>&1

echo "== ⑥ 상태 = FAULT 확인"
deadline_start   # ★ G4: 실경과시간 상한(timeout 미산입 봉쇄) — lib_e2e.sh
until [ "$(state)" = "FAULT" ]; do
  sleep 2; deadline_exceeded 20 && fail "abort 후 FAULT 미진입 (상태='$(state)')"
done
echo "  ✓ FAULT 진입"

echo "== ⑦ 실제 정지 확인 — 감속 여유 5초 후, 5초 간격 ground truth 2회 비교"
sleep 5
read -r x1 y1 < <(robot_xy)
sleep 5
read -r x2 y2 < <(robot_xy)
# ★ 좌표를 못 읽었으면 drift 계산이 쓰레기가 된다 — 그대로 두면 '실정지 실패'로 오분류돼
#   ⑦ 아래 원인 분류까지 헛돈다. 판정 전에 '못 읽음(인프라)'을 먼저 갈라낸다.
if [ -z "${x1:-}" ] || [ -z "${y1:-}" ] || [ -z "${x2:-}" ] || [ -z "${y2:-}" ]; then
  fail "정지 판정용 ground truth 조회 실패 (gz model 무응답, 상한 8s) — 인프라 결함(§5 ③). ⚠ '실정지 실패'가 아니다: 정지/미정지 어느 쪽도 주장하지 않는다"
fi
drift=$(python3 -c "import math; print(round(math.hypot($x2-$x1, $y2-$y1), 3))")
echo "  5초간 이동량: ${drift}m (($x1,$y1) → ($x2,$y2))"
if ! python3 -c "exit(0 if $drift <= 0.10 else 1)"; then
  # ★ 예약 4 (07-30): 여기서 fail() 을 바로 부르면 **즉시 cleanup + exit** 이라
  #   ⑧(/cmd_vel 수집)이 영영 실행되지 않고 노드까지 죽어 증거가 사라진다 —
  #   정작 그 증거가 필요한 순간에. 그래서 **cleanup 전에** 먼저 수집·분류해
  #   '코드 결함(취소 경로)' 인지 '잔류 명령/시뮬 특성' 인지를 FAIL 메시지에 담는다.
  #   (구판은 이 구분에 07-24 동결 게이트에서 수동 규명이 필요했다 — 0723_현황.md §11.3)
  echo "  ⚠ 실정지 단언 실패 — cleanup 전에 /cmd_vel 잔류 수집 (원인 자동 분류)"
  stopfail_n=$(measure_cmdvel_residual "$LOGDIR/cmdvel_stopfail.log")
  echo "  잔류 판독 '${stopfail_n}' 건 (빈값=판독 실패, 덤프: $LOGDIR/cmdvel_stopfail.log)"
  fail "abort 후에도 이동 계속 (${drift}m > 0.10m) — 취소가 실주행을 못 멈춤 | 원인 분류: $(classify_stop_failure "$stopfail_n")"
fi
echo "  ✓ 정지 확인"

echo "== ⑧ /cmd_vel 잠잠 확인 (2초 수집 — 0 이 아닌 속도 명령이 없어야)"
# 수집·판독·분류는 ⑦ 실패 경로와 **같은 계약**을 쓴다 (lib_e2e.sh) — 한쪽만 고쳐지는 드리프트 봉쇄.
# ★ 07-31 §7.2 P1: 구판은 판독 실패('')를 "속도 명령 건" 이라는 빈 문구로 흘렸고, 더 나쁘게는
#   빈 덤프가 '0건'으로 판독돼 **그대로 PASS** 였다. 이제 '0건'은 완전한 유한 Twist 를
#   최소 1개 본 경우에만 나온다 — 그 외는 전부 여기서 FAIL 하며 분류 문구를 함께 남긴다.
nonzero=$(measure_cmdvel_residual "$LOGDIR/cmdvel.log")
if [ "$nonzero" != "0" ]; then
  fail "abort 후 /cmd_vel 판정 실패 (덤프: $LOGDIR/cmdvel.log) — $(classify_stop_failure "$nonzero")"
fi
echo "  ✓ /cmd_vel 잠잠 (관측 근거 확보: 발행자 생존 확인 + 완전 표본 또는 관측된 침묵)"

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
