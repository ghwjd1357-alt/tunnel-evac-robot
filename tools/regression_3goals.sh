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
# 공통 함수(cleanup·fail·trap·state·deadline_*·wait_nav2_ready·send_goal) = 같은 폴더의 라이브러리.
# ★ set -u 뒤에 source (setup.bash 는 이미 위에서 처리). BASH_SOURCE 로 위치 독립.
source "$(dirname "${BASH_SOURCE[0]}")/lib_e2e.sh"

EXTRA_ARGS=("$@")           # 인자는 그대로 런치에 전달 (예: localization:=false)
LOGDIR=$(mktemp -d /tmp/reg3goals.XXXX)
echo "로그: $LOGDIR"

echo "== ① 잔여 프로세스 정리 + 기동"
cleanup
hard_timeout 5 ros2 daemon stop  >/dev/null 2>&1
hard_timeout 5 ros2 daemon start >/dev/null 2>&1   # ★ 예약 17: daemon 도 상한 안에서
nohup ros2 launch tunnel_sim slam_nav2.launch.py gui:=false localization:=true "${EXTRA_ARGS[@]}" \
  > "$LOGDIR/launch.log" 2>&1 &

wait_nav2_ready   # ② 3단 관문(param→lifecycle active→action discovery)·"최대 90초" = lib_e2e.sh

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
read -r gx gy < <(gz_model_xy tunnel_robot)   # ★ 예약 17: 상한 + 유한값 검증
# ⚠ 못 읽음은 '오차 초과'가 아니다 (§5 ③ 인프라 vs 코드 결함 불혼동).
if [ -z "${gx:-}" ] || [ -z "${gy:-}" ]; then
  fail "정확도 판정용 ground truth 조회 실패 (gz model 무응답, 상한 8s) — 인프라 결함(§5 ③). ⚠ 오차 초과가 아니다: 정확도를 판정하지 않는다"
fi
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
