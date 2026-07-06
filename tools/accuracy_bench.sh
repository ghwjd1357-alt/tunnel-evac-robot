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
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
set -u

LABEL=${1:-$(date +%m%d_%H%M)}
shift 2>/dev/null || true
EXTRA_ARGS=("$@")           # 라벨 뒤 인자는 그대로 런치에 전달 (예: localization:=true)
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
t=0
until ros2 param get /controller_server FollowPath.desired_linear_vel 2>/dev/null | grep -q Double; do
  sleep 3; t=$((t+3)); [ $t -ge 90 ] && fail "Nav2 기동 타임아웃"
done
sleep 5   # SLAM 첫 지도/TF 안정화

echo "== ③ 오차 샘플러 기동 (1초 주기 → trace.csv)"
: > "$OUT/summary.txt"
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
