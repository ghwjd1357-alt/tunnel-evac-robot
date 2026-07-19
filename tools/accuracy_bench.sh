#!/usr/bin/env bash
# ============================================================
# accuracy_bench.sh — SLAM·Nav2 정확도 벤치마크 (연속 오차 + 목표별 끝점 오차)
#
# regression_3goals.sh(합격/불합격 판정용)와 형제 스크립트.
# 차이: 판정이 아니라 '측정' — 3목표 코스를 돌며
#   ① accuracy_sampler.py 로 매초 [실위치 vs SLAM 추정] 오차를 CSV 기록
#   ② 각 goal SUCCEEDED 직후의 실위치 끝점 오차 + 소요시간 기록
#   ③ 통계(평균/p95/최대/최종) + 오차 그래프 PNG 생성
#
# 사용: bash tools/accuracy_bench.sh [라벨] [추가 런치인자...]
#   예: bash tools/accuracy_bench.sh loc_mode localization:=true
#   (라벨 기본 = 날짜시각)
# 출력: ~/ros2_ws/bench_out/<라벨>/ {trace.csv, summary.txt, error.png}
# 비교: python3 tools/accuracy_report.py A/trace.csv B/trace.csv \
#         --labels 조정전 조정후 -o compare.png
#
# 노하우 박제 (0705_현황.md 함정 — regression_3goals.sh 와 동일):
#   - set -u 는 ROS source 뒤에 / pkill 브래킷 트릭 / send_goal 재전송 / gz ground truth
# ============================================================
# ⚠ 전용 시뮬 PC 전용 (S2-4, Codex §11.4): cleanup 이 전역 pkill 로 gzserver·
#   nav2·slam 등을 프로세스 이름으로 죽인다 — 다른 ROS 작업이 도는 PC/Jetson
#   에서 실행 금지. Ctrl+C 중단 시에도 trap 이 좀비를 정리한다.
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
set -u

LABEL=${1:-$(date +%m%d_%H%M)}
shift 2>/dev/null || true
# ★ S2-2 라벨 sanitize (Codex §11.5): 경로문자·상위이동이 섞이면 bench_out 밖을 쓴다
case "$LABEL" in
  */*|*..*|.*) echo "== FAIL: 라벨에 경로 문자 금지 ('$LABEL')"; exit 1 ;;
esac
EXTRA_ARGS=("$@")           # 라벨 뒤 인자는 그대로 런치에 전달
# ★ S2-2 기본 모드 = localization (운영 구성. Codex §11.5 — 기존엔 인자 없으면
#   launch 기본(mapping)으로 돌아 'mapping 수치를 운영 정확도로 오인' 가능했다).
#   mapping 측정은 명시적으로 localization:=false 를 줄 때만.
MODE=localization
for a in "${EXTRA_ARGS[@]}"; do
  case "$a" in localization:=false) MODE=mapping ;; esac
done
if [ "$MODE" = "localization" ]; then
  EXTRA_ARGS=(localization:=true "${EXTRA_ARGS[@]}")   # 뒤 인자가 있으면 그쪽이 이김
fi
OUT=~/ros2_ws/bench_out/$LABEL
mkdir -p "$OUT"
LOGDIR=$(mktemp -d /tmp/accbench.XXXX)
echo "== 벤치 시작 (라벨: $LABEL) — 출력: $OUT"

cleanup() {
  pkill -9 -f "accuracy[_]sampler" 2>/dev/null
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

send_goal() {  # $1=x $2=y $3=yaw $4=제한시간(초) — SUCCEEDED 대기, 유실 대비 1회 재전송
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

# goal 도착 직후 실위치(ground truth)로 끝점 오차 계산 → summary 기록
endpoint() {  # $1=goal이름 $2=목표x $3=목표y $4=소요초
  local gx gy err
  read -r gx gy < <(gz model -m tunnel_robot -p 2>/dev/null | tail -1 | awk '{print $1, $2}')
  err=$(python3 -c "import math; print(round(math.hypot(($gx+12)-$2, $gy-($3)), 3))")
  echo "  ✓ $1 SUCCEEDED — 끝점 오차 ${err}m, 소요 ${4}s"
  echo "$1: endpoint_err=${err}m elapsed=${4}s" >> "$OUT/summary.txt"
}

echo "== ① 잔여 프로세스 정리 + 기동"
cleanup
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1
nohup ros2 launch tunnel_sim slam_nav2.launch.py gui:=false "${EXTRA_ARGS[@]}" \
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
sleep 5   # SLAM 첫 지도/TF 안정화

echo "== ③ 오차 샘플러 기동 (1초 주기 → trace.csv)"
# ★ S2-2 메타 기록 (기존 백로그 'accuracy_bench 메타' 흡수): 나중에 이 수치가
#   '어떤 조건'이었는지 — 특히 시뮬 world-odom 상한이라는 사실 — 을 파일이 증언
{
  echo "# mode=$MODE  world=${EXTRA_ARGS[*]:-T자기본}  date=$(date -Iseconds)"
  echo "# git=$(git -C ~/ros2_ws rev-parse --short HEAD 2>/dev/null || echo unknown)"
  echo "# ⚠ Gazebo world-odom(치트 오돔) 시뮬 수치 — 실차 정확도로 인용 금지 (Codex §9.3)"
} > "$OUT/summary.txt"
nohup python3 ~/ros2_ws/tools/accuracy_sampler.py --ros-args \
  -p use_sim_time:=true -p csv:="$OUT/trace.csv" \
  > "$LOGDIR/sampler.log" 2>&1 &

echo "== ④ 3목표 코스 주행"
for goal in "goal1_분기입구 12.0 0.0 1.57 180" \
            "goal2_곁복도   12.0 9.0 -1.57 180" \
            "goal3_동쪽끝   13.5 0.0 0.0 240"; do
  read -r name x y yaw tmo <<< "$goal"
  echo "-- $name ($x,$y)"
  t0=$(date +%s)
  send_goal "$x" "$y" "$yaw" "$tmo" || fail "$name 미도달"
  endpoint "$name" "$x" "$y" $(( $(date +%s) - t0 ))
done

echo "== ⑤ 통계 + 그래프"
cleanup    # 샘플러 먼저 멈추고 집계 (CSV 는 flush 돼 있어 안전)
python3 ~/ros2_ws/tools/accuracy_report.py "$OUT/trace.csv" \
  --labels "$LABEL" -o "$OUT/error.png" | tee -a "$OUT/summary.txt"

echo "== 완료 — 결과: $OUT/{trace.csv,summary.txt,error.png}"
