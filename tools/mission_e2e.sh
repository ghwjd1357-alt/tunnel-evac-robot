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
# 공통 함수(cleanup·fail·trap·state·deadline_*·wait_nav2_ready·send_goal) = 같은 폴더의 라이브러리.
source "$(dirname "${BASH_SOURCE[0]}")/lib_e2e.sh"

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
  T_SEARCHBACK=180   # ⑧-b 재산정(07-24): 관측 최악 ≈90s 의 2배 마진 — 분포·근거 = TEST_GATES §2
else
  WORLD_ARGS=()
  WAYPOINTS_FILE=""              # 빈값 = mission_node 기본(waypoints.yaml)
  FIRE_X=14.0; FIRE_Y=0.0
  ESCAPE_WORLD="(-12, 0)"
  T_GATHER=240; T_ESCAPED=300
  T_SEARCHBACK=180   # ⑧-b 재산정(07-24): 관측 최악 ≈90s 의 2배 마진 — 분포·근거 = TEST_GATES §2
fi

EXTRA_ARGS=("$@")           # 인자는 그대로 런치에 전달 (예: localization:=false)
LOGDIR=$(mktemp -d /tmp/mission_e2e.XXXX)
echo "로그: $LOGDIR (모드: $MODE)"

# wait_state 는 lib_e2e.sh 로 이동했다 (07-24 §14 P1 — 벽시계 deadline 전환 + 격리 단위
#   테스트 가능화). 예산은 여전히 '무엇을 얼마 안에 확인하는가'라는 판정 기준이고, 여기
#   호출부(wait_state PATROL 30 … SEARCH_BACK "$T_SEARCHBACK")가 그 기준을 그대로 정한다.

echo "== ① 잔여 프로세스 정리 + 시뮬 기동"
cleanup
hard_timeout 5 ros2 daemon stop  >/dev/null 2>&1
hard_timeout 5 ros2 daemon start >/dev/null 2>&1   # ★ 예약 17: daemon 도 상한 안에서
nohup ros2 launch tunnel_sim slam_nav2.launch.py gui:=false localization:=true \
  "${WORLD_ARGS[@]}" "${EXTRA_ARGS[@]}" \
  > "$LOGDIR/launch.log" 2>&1 &

wait_nav2_ready   # ② 3단 관문(param→lifecycle active→action discovery)·"최대 90초" = lib_e2e.sh

echo "== ③ 미션 노드 + 가짜 추종자 기동"
MISSION_ARGS=(-p use_sim_time:=true)
[ -n "$WAYPOINTS_FILE" ] && MISSION_ARGS+=(-p "waypoints_file:=$WAYPOINTS_FILE")
nohup ros2 run mission_manager mission_node --ros-args "${MISSION_ARGS[@]}" \
  > "$LOGDIR/mission.log" 2>&1 &
nohup ros2 run tunnel_sim fake_follower --ros-args -p use_sim_time:=true \
  > "$LOGDIR/follower.log" 2>&1 &
wait_state PATROL 30

echo "== ④ 추종자 스폰 확인"
deadline_start   # ★ G4: 실경과시간 상한(timeout 미산입 봉쇄) — lib_e2e.sh
until grep -qE "스폰 완료|모델 접수" "$LOGDIR/follower.log"; do
  sleep 3; deadline_exceeded 45 && fail "추종자 스폰 확인 실패"
done
echo "  ✓ 추종자 스폰"
sleep 10   # 순찰 이동 + 추종자 따라붙기 (모니터가 '봤다' 기록을 쌓게)

echo "== ⑤ 🔥 화재 알람 발사 → APPROACH"
# ★ 알람은 상태 전이 확인까지 재시도 (07-07): -w 1 + --times 로도 간헐 유실 실측
#   (mission.log 에 '알람' 0건). 전이 안 됐으면 그냥 한 번 더 쏘면 되는 멱등 신호.
for try in 1 2 3; do
  # ⑦ 스윕 (07-24): -w 1 은 구독자 매칭까지 블록한다 — 미들웨어 이상 시 무한 대기 가능
  #   (param get 과 같은 실패양식). 구독자(mission_node)는 정상 시 즉시 뜨므로 happy path
  #   영향 0, 병리 시 timeout 으로 상한을 씌운다. --times 로도 이미 유한하지만 이중 방어.
  # ★ 07-24 §16 P1: 일반 timeout 은 TERM 무시 CLI 를 못 죽인다. 공통 hard_timeout 으로
  #   alarm 발행도 12s + SIGKILL 유예 안에 반드시 종결한다.
  hard_timeout 12 ros2 topic pub --times 2 -w 1 /alarm geometry_msgs/msg/PoseStamped \
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
# ★ ⑦ 타임아웃 가드 (07-24 e2e-harness-fix): 구판은 이 param get 이 무방비라 CLI/daemon
#   flake(§5 ③) 때 13분 27초 무한 행이 실측됐다(쌍굴 3회차, FREEZE_MANIFEST §8).
#   read_param_float 가 hard_timeout(param get 8 + daemon 재시작 각 5 + 재시도 8, TERM 무시도
#   SIGKILL)로 상한(정상 ≈26s, TERM 무시 최악 34s)을 씌운다 — 근거 = lib_e2e.sh · TEST_GATES §2.
#   ⚠ '못 읽음(§5 ③ 인프라)'과 '값이 틀림(S1-5 코드 결함)'을 절대 뒤섞지 않는다 —
#   빈 결과는 인프라 결함으로 분류해 FAIL(조용한 통과 없음), 값이 있으면 0.12 비교로 판정.
v=$(read_param_float /controller_server FollowPath.desired_linear_vel)
if [ -z "$v" ]; then
  fail "desired_linear_vel 조회 무응답 — ros2 param CLI/daemon 결함(§5 ③), hard_timeout+daemon 재시작 재시도도 실패(상한 ≤34s)"
elif [ "$v" = "0.12" ]; then
  echo "  ✓ GUIDE 저속 0.12 m/s 실측 확인"
else
  fail "GUIDE 중 desired_linear_vel=$v ≠ 0.12 (속도 변경 미적용 — S1-5)"
fi
sleep 10
hard_timeout 12 ros2 topic pub --times 3 -w 1 /follower_cmd std_msgs/msg/String \
  "{data: stop}" >/dev/null 2>&1   # ⑦ 스윕: -w 1 블록 방지 timeout (구독자=fake_follower)
grep -q "stop" "$LOGDIR/follower.log" || echo "  (경고: follower 로그에 stop 미확인)"

echo "== ⑧ SEARCH_BACK 진입 대기 (거리 벌어짐 + 3초 디바운스)"
wait_state SEARCH_BACK "$T_SEARCHBACK"

echo "== ⑨ 추종 재개 (follow) → 재발견 → GUIDE 복귀"
sleep 3
hard_timeout 12 ros2 topic pub --times 3 -w 1 /follower_cmd std_msgs/msg/String \
  "{data: follow}" >/dev/null 2>&1   # ⑦ 스윕: -w 1 블록 방지 timeout (구독자=fake_follower)
wait_state GUIDE 120
grep -q "재발견" "$LOGDIR/mission.log" && echo "  ✓ 미션 로그에 '재발견' 확인"

echo "== ⑩ 같이 탈출 → ESCAPED 대기"
wait_state ESCAPED "$T_ESCAPED"

echo "== ⑪ 최종 위치 (ground truth)"
# ★ 예약 17: 여기가 07-30 에 **11분 무한 행**으로 실측된 지점이다(고아 gz 는 21분 생존).
#   ⑩까지 전부 통과한 뒤 보고 단계에서 게이트가 멈췄고, 사람이 죽여야 PASS 가 났다 —
#   개입해서 얻은 PASS 는 판정이 아니다. 상한은 gz_model_xy 로 단일화한다.
#   ⚠ 이 단계는 **보고 전용**이라 못 읽어도 FAIL 로 만들지 않는다(판정은 ⑩에서 끝났다).
fmt_xy() { awk '{printf "(%.2f, %.2f)", $1, $2; f=1} END{if(!f) printf "(조회 실패)"}'; }
robot=$(gz_model_xy tunnel_robot  | fmt_xy)
fol=$(gz_model_xy fake_follower | fmt_xy)
echo "  로봇 world $robot / 추종자 world $fol  (탈출구 = world $ESCAPE_WORLD)"

cleanup
echo "== PASS: 놓침→역행→재발견→GUIDE 복귀→탈출 전 구간 완주"
