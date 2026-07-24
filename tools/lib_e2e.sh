#!/usr/bin/env bash
# ============================================================
# lib_e2e.sh — 4개 E2E 스크립트 공통 셸 함수 라이브러리 (구조 분리 3/3, 07-24 추출)
#
# 배경: mission_e2e·abort_e2e·regression_negative·regression_3goals 가 프로세스
#   cleanup·Nav2 readiness 대기·goal 전송·deadline 계산을 각자 복붙하고 있었다.
#   중복은 한쪽만 고쳐지는 드리프트의 씨앗이라, 그 공통부를 여기 한 곳으로 모은다.
#   (마스터플랜 §2-3 / §7-2 · Codex §14.5 P2 — readiness "최대 90초" 문구·deadline
#    함수 단일화.)
#
# ★ 순수 리팩터 규약: E2E 판정 기준(PASS 조건·허용 오차·타임아웃)은 한 글자도
#   바꾸지 않는다. 이 파일은 '어떻게 기다리고 어떻게 정리하는가'의 공통 절차만 담고,
#   '무엇을 PASS 로 볼지'는 각 스크립트가 그대로 판정한다.
#
# source 규약 (중요): 각 스크립트가 ROS setup.bash 를 source 하고 `set -u` 를 켠
#   '뒤'에 이 파일을 source 한다. (setup.bash 는 미정의 변수를 참조하므로 set -u
#   상태로 source 하면 즉사 — 그래서 순서가 source→set -u→이 파일이다.)
#   이 파일은 함수 정의와 trap 설치만 하고, LOGDIR 은 각 스크립트가 mktemp 로 잡는다
#   (스크립트마다 접두어가 달라 여기서 만들지 않는다). cleanup 은 LOGDIR 을 쓰지 않아
#   LOGDIR 설정 전에 trap 이 돌아도 안전하다.
#
# ⚠ 전용 시뮬 PC 전용 (S2-4, Codex §11.4): cleanup 이 전역 pkill 로 gzserver·nav2·
#   slam 등을 '프로세스 이름'으로 죽인다 — 다른 ROS 작업이 도는 PC/Jetson 에서 실행 금지.
# ============================================================

# --- 공통 프로세스 정리 ------------------------------------------------------
# 순서 불변조건(AGENTS.md §4): 부모 `ros2 launch` 를 먼저 kill → 그 다음 nav2 노드.
#   부모만 kill -9 하면 고아가 된 좀비 bt_navigator 가 다음 실행의 goal 을 가로챈다.
#   자기매칭 자살 방지 = 브래킷 트릭("ros2[ ]launch", "mission[_]node" …).
#
# cleanup 은 4개 스크립트의 '합집합'이다: mission_node·fake_follower 는 회귀 2종
#   (negative·3goals)에는 없던 대상이지만, 그 스크립트가 해당 프로세스를 안 띄우므로
#   pkill 은 무해한 no-op 이고, 오히려 직전 mission 실행이 남긴 좀비(고아 mission_node
#   가 제 goal 을 nav2 로 계속 쏘는 간섭)를 선제 정리해 더 안전하다. 어느 경우에도
#   PASS 판정에는 영향이 없다(정리 절차일 뿐 판정 기준이 아니다).
cleanup() {
  pgrep -f "ros2[ ]launch" | xargs -r kill -9 2>/dev/null   # ★ 부모 먼저
  pkill -9 -f "mission[_]node" 2>/dev/null
  pkill -9 -f "fake[_]follower" 2>/dev/null
  pkill -9 -x gzserver 2>/dev/null; pkill -9 -x gzclient 2>/dev/null
  pkill -9 -f "slam[_]toolbox" 2>/dev/null   # async(mapping)·localization 둘 다 매칭
  pkill -9 -f "robot_state[_]publisher" 2>/dev/null
  pkill -9 -f "lib/nav2[_]" 2>/dev/null   # ★ nav2 노드 전체 (부모 kill -9 의 고아 청소)
  pkill -9 -x ekf_node 2>/dev/null
  pkill -9 -f "spawn[_]entity" 2>/dev/null
  sleep 1
}

# --- 실패 종결 + 인터럽트 정리 (F4, Codex §12.6) -----------------------------
# fail 은 LOGDIR(각 스크립트 global)을 참조 — 호출 시점엔 이미 mktemp 로 설정돼 있다.
fail() { echo "== FAIL: $1 (로그: $LOGDIR)"; cleanup; exit 1; }
# ★ S2-4: Ctrl+C/kill 로 끊겨도 좀비(고아 nav2 등)를 안 남기게. cleanup 후 반드시 exit —
#   없으면 Ctrl+C 뒤에도 다음 단계가 계속 실행된다.
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM

# --- 경과시간(deadline) 계산 -------------------------------------------------
# ★ G4 (Codex §13.5): 상한은 sleep 누적이 아니라 '실경과시간'으로 잰다. timeout 대기가
#   sleep 에 안 잡히는 구멍을 막는다. deadline_start 로 기준시각을 찍고, 루프 안에서
#   deadline_exceeded N 으로 N 초 초과를 확인한다.
E2E_T0=$SECONDS            # set -u 안전용 초기화 (실사용은 항상 deadline_start 가 먼저 덮음)
deadline_start()    { E2E_T0=$SECONDS; }
deadline_exceeded() { [ $(( SECONDS - E2E_T0 )) -ge "$1" ]; }

# --- 미션 상태 1개 읽기 (mission_e2e·abort_e2e 공용) -------------------------
# /mission_state 는 2Hz 발행이라 3초 timeout 이면 1개 읽기에 충분하다.
state() {
  timeout 3 ros2 topic echo /mission_state --once 2>/dev/null \
    | sed -n 's/^data: //p' | head -1
}

# --- 파라미터 float 1개 읽기 (timeout + daemon 재시작 1회 재시도) -------------
# ★ ⑦ (07-24 e2e-harness-fix): `ros2 param get` 은 CLI/daemon flake(§5 ③) 때
#   무한 행할 수 있다 — 무방비 호출이 쌍굴 3회차에서 13분 27초 매달렸다
#   (FREEZE_MANIFEST §8). readiness 관문(`timeout 8`)·`wait_state`(빈 읽기 시 daemon
#   재시작)가 이미 쓰는 방어를 이 조회에도 단일화한다: 매 조회 `timeout 8`, 빈 결과면
#   `ros2 daemon` 재시작 1회 후 재시도. 두 번 다 비면 '' 를 돌려준다(상한 ≈19s).
#   ⚠ 여기서 보장하는 것은 '유한 시간에 읽거나 포기'뿐이다 — 읽은 값이 옳은지의 판정은
#   호출자 몫이다. '못 읽음(§5 ③ 인프라)'과 '값이 틀림(코드 결함)'을 뒤섞지 않기 위함.
read_param_float() {  # $1=노드 $2=파라미터명 → float 문자열(예: 0.12) 또는 '' (읽기 실패)
  local out
  out=$(timeout 8 ros2 param get "$1" "$2" 2>/dev/null | grep -oE '[0-9]+\.[0-9]+' | head -1)
  if [ -z "$out" ]; then
    ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1
    out=$(timeout 8 ros2 param get "$1" "$2" 2>/dev/null | grep -oE '[0-9]+\.[0-9]+' | head -1)
  fi
  printf '%s' "$out"
}

# --- Nav2 활성화 대기 (4개 스크립트 공통 — "최대 90초" 문구 단일 출처) --------
# 3단 관문: ① controller_server 파라미터 존재 → ② bt_navigator lifecycle active →
#   ③ navigate_to_pose action 서버 discovery. 셋은 '한 번' 찍은 기준시각(deadline_start)
#   을 공유해 누적 상한(90→120→150초)으로 판정한다. CLI 자체 hang 방지로 매 조회에
#   timeout 8 을 건다(F4, Codex §12.10). "inactive" 에 'active' 가 부분포함되므로 ^앵커 필수.
wait_nav2_ready() {
  echo "== ② Nav2 활성화 대기 (최대 90초)"
  deadline_start
  until timeout 8 ros2 param get /controller_server FollowPath.desired_linear_vel 2>/dev/null | grep -q Double; do
    sleep 3; deadline_exceeded 90 && fail "Nav2 기동 타임아웃"
  done
  # F4 (Codex §12.10): parameter 존재 ≠ lifecycle active — bt_navigator 활성까지 확인
  until timeout 8 ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q "^active"; do
    sleep 3; deadline_exceeded 120 && fail "bt_navigator 미활성 (lifecycle bringup 실패 의심 — launch 로그 확인)"
  done
  # ★ G4 (Codex §13.5): bt_navigator active ≠ action discovery 완료 —
  #   navigate_to_pose 서버가 실제 떠 있는지 별도 확인 (goal 전송 전 마지막 관문)
  until timeout 8 ros2 action info /navigate_to_pose 2>/dev/null | grep -q "Action servers: [1-9]"; do
    sleep 2; deadline_exceeded 150 && fail "navigate_to_pose action server 미준비"
  done
  sleep 5   # 지도/TF 안정화
}

# --- 정상 goal 전송: SUCCEEDED 대기, 응답유실 대비 1회 재전송 ----------------
# regression_3goals(yaw 지정)·regression_negative(정상 대조군, yaw=0) 공용.
#   응답 유실 함정(0705 §15.7 ③): 오래 뜬 액션서버 + 새 CLI 는 goal 응답을 떨굴 수
#   있어(bt_navigator "Failed to send goal response") 1회 재전송으로 방어한다.
#   판정은 believed(TF)가 아니라 최종 상태문자열(SUCCEEDED)로만 한다 — 실위치 오차
#   판정은 각 스크립트가 ground truth(gz model)로 별도로 본다.
send_goal() {  # $1=x $2=y $3=yaw(rad) $4=제한시간(초)
  local qz qw out attempt
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
