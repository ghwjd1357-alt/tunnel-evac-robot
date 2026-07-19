#!/usr/bin/env bash
# ============================================================
# make_map.sh — 정본 지도 제작 자동화 (mapping 1회 → localization 운영의 재료)
#
# 하는 일: 라이브 SLAM(mapping)으로 4목표 코스를 돌아 터널 전체를 훑고
#   ① posegraph 저장 (slam_toolbox localization 모드가 읽는 파일) ★ 핵심 산출물
#      → maps/tunnel_localization.{posegraph,data}
#   ② pgm+yaml 저장 (사람 눈 확인·기록용) → maps/tunnel_map_loc.{pgm,yaml}
#
# 사용: bash tools/make_map.sh         (T자 터널, 약 11분 — 승격 전 negative 수락 게이트 포함)
#       bash tools/make_map.sh twin    (쌍굴 터널 → maps/twin_localization.*, 약 12분)
# 흐름(G2·G3): 주행 → staging 저장(4파일, yaml 경로 정정 포함) → 스모크(도달+TF-GT)
#   → [T자] staging 부정 회귀 = 수락 게이트 → map_promote.sh transaction 승격 → manifest
#   전부 fail-closed: 어느 단계가 실패해도 정본 무손상 (staging 보존 = 재시도 가능)
# 지도를 다시 만들 때(월드 변경 등)도 이 스크립트 한 번이면 끝 — 재현 가능.
#
# 노하우 박제: set -u 는 source 뒤 / pkill 브래킷 트릭 / send_goal 재전송 (§15.7)
# ============================================================
# ⚠ 전용 시뮬 PC 전용 (S2-4, Codex §11.4): cleanup 이 전역 pkill 로 gzserver·
#   nav2·slam 등을 프로세스 이름으로 죽인다 — 다른 ROS 작업이 도는 PC/Jetson
#   에서 실행 금지. Ctrl+C 중단 시에도 trap 이 좀비를 정리한다.
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
set -u

# --- 월드 모드 (07-07 → S2-1 엄격화): 인자 없음 = T자 / twin = 쌍굴 / 그 외 = 즉사 ---
# ⚠ 기존엔 "twin 이 아니면 전부 tunnel" — `twim` 오타가 T자 정본 덮어쓰기로 흘렀다 (Codex §11.3)
MODE="${1:-tunnel}"
case "$MODE" in
  tunnel)
    LAUNCH_ARGS=()
    OUT_POSEGRAPH=tunnel_localization
    OUT_PGM=tunnel_map_loc
    LOC_PARAMS=slam_params_localization.yaml
    SMOKE_GOAL="3.0 0.0"               # 스모크: 짧은 정상 goal 1개
    WORLD_FILE=tunnel.world
    SPAWN_X=-12                        # 스모크 TF-GT 오차 계산용 (map=world+offset)
    ;;
  twin)
    LAUNCH_ARGS=(world:=tunnel_twin.world spawn_x:=-17)
    OUT_POSEGRAPH=twin_localization    # slam_params_localization_twin.yaml 이 읽는 이름
    OUT_PGM=twin_map_loc
    LOC_PARAMS=slam_params_localization_twin.yaml
    SMOKE_GOAL="12.0 0.0"
    WORLD_FILE=tunnel_twin.world
    SPAWN_X=-17
    ;;
  *)
    echo "== FAIL: 알 수 없는 모드 '$MODE' (tunnel|twin 만 허용 — 오타가 정본을 덮어쓰지 않게 즉사)"
    exit 1
    ;;
esac

MAPDIR=~/ros2_ws/maps
LOGDIR=$(mktemp -d /tmp/makemap.XXXX)
echo "== 지도 제작 시작 (모드: $MODE) — 로그: $LOGDIR"

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
# ★ S2-4→F4 (Codex §12.6): cleanup 후 반드시 exit — 없으면 Ctrl+C 뒤에도
#   스크립트가 다음 단계를 계속 실행 (재검토 중 실측: 늦은 전역 cleanup 이
#   다음 테스트의 launch 까지 사살)
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM

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
# ★ mapping 탐사 오버레이 (07-19 심야 — S2-1 이 검거한 정책 충돌):
#   운영 안전 정책 track_unknown_space:true(07-19 명시) + allow_unknown:false 는
#   '미지 공간으로 계획 금지'라, 미지 영역으로 goal 을 보내며 지도를 넓히는
#   mapping 세션에선 planner 가 전 goal 을 거부한다 (make_map 첫 실행이 검거 —
#   localization 기반 회귀들은 전부 통과해서 여기서만 드러남).
#   해법 = 07-06·07-07 지도 v3·쌍굴 제작을 성공시킨 '검증된 mapping 구성' 재현:
#   track_unknown_space:false (당시 배포판 기본값 — 미지=자유로 취급).
#   ⚠ allow_unknown:true 우회는 기각 — NavFn "legal potential ... This shouldn't
#   happen" 결함으로 traceback 실패 실측 (두 번째 실행이 검거).
MAP_PARAMS="$LOGDIR/nav2_mapping.yaml"
sed 's/track_unknown_space: true/track_unknown_space: false  # mapping 탐사 한정 오버레이/' \
  ~/ros2_ws/src/tunnel_sim/config/nav2_params.yaml > "$MAP_PARAMS"
grep -q "track_unknown_space: false" "$MAP_PARAMS" || fail "mapping 오버레이 생성 실패"
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1
nohup ros2 launch tunnel_sim slam_nav2.launch.py gui:=false \
  nav2_params:="$MAP_PARAMS" "${LAUNCH_ARGS[@]}" \
  > "$LOGDIR/launch.log" 2>&1 &

echo "== ② Nav2 활성화 대기 (최대 90초)"
# ★ F4 (Codex §12.10): CLI 자체 timeout 없으면 hang 시 '최대 90초'가 무효 +
#   parameter 존재 ≠ lifecycle active → bt_navigator active 까지 확인
T0=$SECONDS   # ★ G4 (Codex §13.5): sleep 누적 아닌 실경과시간 상한 (timeout 대기 미산입 구멍 봉쇄)
until timeout 8 ros2 param get /controller_server FollowPath.desired_linear_vel 2>/dev/null | grep -q Double; do
  sleep 3; [ $((SECONDS-T0)) -ge 90 ] && { grep -iE "lifecycle|Failed|Error" "$LOGDIR/launch.log" | tail -5; fail "Nav2 기동 타임아웃"; }
done
# ⚠ "inactive" 에도 'active' 가 부분 문자열로 들어 있음 → 행 시작 앵커 필수
until timeout 8 ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q "^active"; do
  sleep 3; [ $((SECONDS-T0)) -ge 120 ] && { grep -iE "lifecycle|Failed|Error" "$LOGDIR/launch.log" | tail -5; fail "bt_navigator 미활성"; }
done
# ★ G4 (Codex §13.5): bt_navigator active ≠ action discovery 완료 —
#   navigate_to_pose 서버가 실제 떠 있는지 별도 확인 (goal 전송 전 마지막 관문)
until timeout 8 ros2 action info /navigate_to_pose 2>/dev/null | grep -q "Action servers: [1-9]"; do
  sleep 2; [ $((SECONDS-T0)) -ge 150 ] && fail "navigate_to_pose action server 미준비"
done
sleep 5

if [ "$MODE" = "twin" ]; then
  echo "== ③ 쌍굴 커버리지 주행 (1번 굴 왕복 + 통로 2곳 + 2번 굴 왕복)"
  # map 좌표 치트시트: 1번 굴 y=0 (x -3~37) / 2번 굴 y=10 / 통로 x=7,17,27
  # ★ goal 은 12m 이하 징검다리로 (07-07 실측 함정): 라이브 SLAM 지도는 로봇 주변만
  #   그려진 채 자라므로, 멀리 있는 goal 은 "off the global costmap" 으로 계획 불가.
  #   (T자는 goal 이 다 13.5m 이내라 이 함정이 드러난 적 없음 — 40m 굴에서 첫 검거)
  hop() { send_goal "$1" "$2" "$3" "$4" || fail "hop($1,$2) 미도달"; echo "  ✓ hop ($1, $2)"; }
  hop 12.0 0.0 0.0 150          # 1번 굴 동진 ①
  hop 24.0 0.0 0.0 150          # 1번 굴 동진 ②
  hop 35.0 0.0 1.57 150         # 1번 굴 동쪽 끝
  hop 27.0 0.0 1.57 120         # 동쪽 통로 입구
  hop 27.0 10.0 0.0 180         # 통로 통과 → 2번 굴
  hop 35.0 10.0 3.14 150        # 2번 굴 동쪽 끝
  hop 24.0 10.0 3.14 150        # 2번 굴 서진 ①
  hop 12.0 10.0 3.14 150        # 2번 굴 서진 ②
  hop 2.0 10.0 3.14 150         # 2번 굴 서쪽 끝
  hop 7.0 10.0 -1.57 120        # 서쪽 통로 입구
  hop 7.0 0.0 3.14 180          # 통로 통과 → 1번 굴 복귀
  hop 1.0 0.0 0.0 150           # 원점 복귀 (1번 굴 서쪽 재훑기 = 지도 다지기)
else
  echo "== ③ 4목표 커버리지 주행 (동쪽끝·곁복도·서쪽 복귀 = 터널 전체 훑기)"
  send_goal 12.0 0.0 1.57 180  || fail "goal1(분기입구) 미도달"
  echo "  ✓ goal1 분기입구"
  send_goal 12.0 9.0 -1.57 180 || fail "goal2(곁복도) 미도달"
  echo "  ✓ goal2 곁복도"
  send_goal 13.5 0.0 3.14 240  || fail "goal3(동쪽끝) 미도달"
  echo "  ✓ goal3 동쪽끝"
  send_goal 1.0 0.0 0.0 240    || fail "goal4(서쪽 복귀) 미도달"
  echo "  ✓ goal4 서쪽 복귀 (복도 재훑기 = 지도 다지기)"
fi

echo "== ④ 저장: staging 이름으로 (★ S2-1 — 정본은 스모크 통과 전까지 안 건드림)"
# ⚠ 기존엔 정본에 직행 serialize — 부분 지도·나쁜 런도 result=0 이면 정본을 대체했다
#   (Codex §11.3). 이제: staging 저장 → localization 스모크 → 통과 시에만 승격.
mkdir -p "$MAPDIR"
STAGING="${OUT_POSEGRAPH}_staging"
rm -f "$MAPDIR/$STAGING".*
# ⚠ 확장자 없이 — slam_toolbox 가 .posegraph/.data 를 알아서 붙임
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: $HOME/ros2_ws/maps/$STAGING}" \
  | grep -q "result=0" || fail "posegraph 저장 실패"
ls -l "$MAPDIR/$STAGING".* 2>/dev/null || fail "staging posegraph 파일 없음"
# ★ F3 (Codex §12.5-1): PGM/YAML 도 staging — 기존엔 정본 이름에 직접 써서
#   스모크 실패 런에서도 사람 확인용 지도가 이미 바뀌어 있었다
PGM_STAGING="${OUT_PGM}_staging"
rm -f "$MAPDIR/$PGM_STAGING".*
# ★ G2 (Codex §13.4-1): '기록용이라 계속' 폐지 — pgm 실패도 승격 차단.
#   4파일 transaction 주장이 조건부가 되지 않게 fail-closed 로 통일.
timeout 30 ros2 run nav2_map_server map_saver_cli -f "$MAPDIR/$PGM_STAGING" \
  >> "$LOGDIR/mapsaver.log" 2>&1 || fail "pgm 저장 실패 — 승격 차단 (G2 fail-closed)"
[ -s "$MAPDIR/$PGM_STAGING.pgm" ] && [ -s "$MAPDIR/$PGM_STAGING.yaml" ] \
  || fail "pgm/yaml staging 파일 없음/빈파일 — 승격 차단"
# ★ G2 (Codex §13.4-4): yaml 의 image: 경로를 staging 단계에서 정본 이름으로
#   정정+검증 — 기존엔 승격 '후' 무검증 sed 라 실패 시 yaml 이 이동된 staging
#   pgm 이름을 가리킨 채 남을 수 있었다. 이제 경로 정정도 transaction 안쪽.
sed -i "s|$PGM_STAGING|$OUT_PGM|g" "$MAPDIR/$PGM_STAGING.yaml"
grep -q "image: .*$OUT_PGM.pgm" "$MAPDIR/$PGM_STAGING.yaml" \
  || fail "yaml image 경로 정정 실패 — 승격 차단"

echo "== ⑤ 스모크 검증: 새 지도로 localization 기동 + goal 1개 (약 2분)"
cleanup
# staging 파일을 읽는 임시 localization 파라미터 생성 (경로만 치환).
# launch 의 PathJoinSubstitution 은 os.path.join 의미 — 절대경로를 주면 그대로 쓴다.
SMOKE_YAML="$LOGDIR/smoke_localization.yaml"
sed "s|maps/$OUT_POSEGRAPH|maps/$STAGING|" \
  ~/ros2_ws/src/tunnel_sim/config/$LOC_PARAMS > "$SMOKE_YAML"
grep -q "$STAGING" "$SMOKE_YAML" || fail "스모크 파라미터 생성 실패 (경로 치환 안 됨)"
nohup ros2 launch tunnel_sim slam_nav2.launch.py gui:=false localization:=true \
  localization_params:="$SMOKE_YAML" "${LAUNCH_ARGS[@]}" \
  > "$LOGDIR/smoke.log" 2>&1 &
T0=$SECONDS   # ★ G4 (Codex §13.5): sleep 누적 아닌 실경과시간 상한 (timeout 대기 미산입 구멍 봉쇄)
until timeout 8 ros2 param get /controller_server FollowPath.desired_linear_vel 2>/dev/null | grep -q Double; do
  sleep 3; [ $((SECONDS-T0)) -ge 90 ] && fail "스모크 기동 타임아웃 (새 지도로 localization 불가?)"
done
until timeout 8 ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q "^active"; do
  sleep 3; [ $((SECONDS-T0)) -ge 120 ] && fail "스모크 bt_navigator 미활성"
done
# ★ G4 (Codex §13.5): bt_navigator active ≠ action discovery 완료 —
#   navigate_to_pose 서버가 실제 떠 있는지 별도 확인 (goal 전송 전 마지막 관문)
until timeout 8 ros2 action info /navigate_to_pose 2>/dev/null | grep -q "Action servers: [1-9]"; do
  sleep 2; [ $((SECONDS-T0)) -ge 150 ] && fail "navigate_to_pose action server 미준비"
done
sleep 5
read -r sx sy <<< "$SMOKE_GOAL"
send_goal "$sx" "$sy" 0.0 120 || fail "스모크 goal($sx,$sy) 미도달 — 새 지도 품질 의심, 정본 유지"
# ★ F3 (F1 진단 후속): 스모크는 '도달'만이 아니라 '위치추정 정직성'까지 —
#   TF(believed)와 gz(ground truth)가 벌어진 지도는 goal 판정 전체를 오염시킨다
#   (blocked goal 이 성공으로 둔갑하는 류의 사고 예방)
read -r gx gy < <(gz model -m tunnel_robot -p 2>/dev/null | tail -1 | awk '{print $1, $2}')
tfl=$(timeout 6 ros2 run tf2_ros tf2_echo map base_footprint 2>/dev/null \
      | grep -m1 Translation | sed 's/.*\[//;s/\].*//')
tf_err=$(python3 -c "
import sys
try:
    tx, ty, _ = [float(v) for v in '''$tfl'''.split(',')]
    print(round(abs((($gx)-($SPAWN_X))-tx)+abs(($gy)-ty), 3))
except Exception:
    print('nan')")
if [ "$tf_err" = "nan" ]; then
  # ★ G2 (Codex §13.4-2): 측정 실패 = 통과가 아니라 차단 — F1 사고(위치추정
  #   오차가 tolerance 를 삼켜 blocked goal 오성공)의 검사 게이트가 fail-open
  #   이면 게이트가 아니다.
  fail "TF-GT 오차 측정 실패 — 위치추정 정직성 미확인, 승격 차단 (G2 fail-closed)"
elif python3 -c "exit(0 if $tf_err <= 0.3 else 1)"; then
  echo "  ✓ 스모크 통과 (도달 + TF-GT 오차 ${tf_err}m ≤ 0.3m)"
else
  fail "스모크 위치추정 오차 ${tf_err}m > 0.3m — 새 지도 품질 불량, 정본 유지"
fi

if [ "$MODE" = "tunnel" ]; then
  echo "== ⑤.5 승격 전 수락 게이트: staging 지도로 부정 회귀 (G3 — F1 개정 자동화, 약 5분)"
  # ★ G3 (Codex §13.4-5): "승격 후 negative 를 사람이 돌린다" 안내문은 게이트가
  #   아니다 (잊으면 불량 지도가 정본으로 잔존 — F1 재발 경로). 승격 '전'에
  #   staging 지도로 자동 실행 = 불량 지도는 정본 이름을 얻지 못한다.
  #   (twin 판 negative 는 기존 백로그 유지 — twin 은 스모크+TF 게이트까지)
  cleanup
  bash ~/ros2_ws/tools/regression_negative.sh localization_params:="$SMOKE_YAML" \
    || fail "staging 지도 부정 회귀 FAIL — 승격 금지 (지도 수락 기준 미달, 정본 무손상)"
else
  echo "  (twin: negative twin 판 백로그 — 스모크+TF 게이트로 승격, mission_e2e twin 으로 사후 확인)"
fi

echo "== ⑥ 승격: 4파일 transaction → tools/map_promote.sh (G2 소도구 분리)"
cleanup
# ★ G2 (Codex §13.4-3): 승격 로직을 격리 테스트 가능한 소도구로 —
#   최초 생성(기존 정본 없음) rollback 까지 하네스로 검증됨 (staging 보존 포함).
#   초 단위 스탬프·백업 실패 중단·중간 실패 원복은 F3 정책 계승.
STAMP=$(date +%y%m%d_%H%M%S)
bash ~/ros2_ws/tools/map_promote.sh "$MAPDIR" "$STAMP" \
  "$STAGING.posegraph:$OUT_POSEGRAPH.posegraph" \
  "$STAGING.data:$OUT_POSEGRAPH.data" \
  "$PGM_STAGING.pgm:$OUT_PGM.pgm" \
  "$PGM_STAGING.yaml:$OUT_PGM.yaml" \
  || fail "승격 transaction 실패 — 이전 상태로 원복됨 (staging 보존, 재시도 가능)"
DSTS=("$OUT_POSEGRAPH.posegraph" "$OUT_POSEGRAPH.data" "$OUT_PGM.pgm" "$OUT_PGM.yaml")
# ★ F3 (Codex §12.5-5·6): manifest 에 재현에 필요한 전부 —
#   dirty tree 면 commit 만으로 생성 과정 재현 불가함을 명시
{
  echo "generated: $(date -Iseconds)"
  echo "mode: $MODE"
  echo "git_commit: $(git -C ~/ros2_ws rev-parse HEAD 2>/dev/null || echo unknown)"
  echo "git_dirty_files: $(git -C ~/ros2_ws status --porcelain 2>/dev/null | wc -l)"
  echo "smoke: goal($SMOKE_GOAL) SUCCEEDED, tf_gt_err=${tf_err}m"
  for f in "${DSTS[@]}"; do
    echo "sha256($f): $(sha256sum "$MAPDIR/$f" | cut -d' ' -f1)"
  done
  for f in "src/tunnel_sim/config/nav2_params.yaml" \
           "src/tunnel_sim/config/$LOC_PARAMS" \
           "src/tunnel_sim/config/slam_params.yaml" \
           "src/tunnel_sim/worlds/$WORLD_FILE" \
           "src/tunnel_sim/urdf/robot.urdf"; do
    echo "sha256($f): $(sha256sum "$HOME/ros2_ws/$f" 2>/dev/null | cut -d' ' -f1)"
  done
} > "$MAPDIR/$OUT_POSEGRAPH.manifest.txt"
echo "  ✓ 정본 승격 + manifest: $MAPDIR/$OUT_POSEGRAPH.manifest.txt (백업: .bak_$STAMP)"

# ★ F1→G3 (07-19): 스모크 통과 지도가 부정 회귀에서 검거된 실전 사례 후속 —
#   T자는 negative 가 승격 '전' 자동 게이트로 올라감 (⑤.5). 사람 기억에 의존 안 함.
if [ "$MODE" = "tunnel" ]; then
  echo "== 완료 — 스모크 + 부정 회귀(수락 게이트)까지 통과한 정본 (G3 자동화)"
else
  echo "== 완료 — 스모크 검증된 정본 (twin: negative twin 판 백로그)"
  echo "⚠ 권장: bash tools/mission_e2e.sh twin 으로 사후 확인"
fi
