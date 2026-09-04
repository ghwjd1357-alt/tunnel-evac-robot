#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════
# 관제 콘솔 실행 (2026-09-02 개정 — bag 재생 모드 추가)
#
#   기본      : rosbridge(:9090) + 웹서버(:8000)
#   --bag TAG : 위 둘 + `~/robot_evidence/TAG` 를 재생한다
#               → 로봇 없이 08-23 실차 데이터로 관제 화면 전체가 돈다
#   --at N    : N 초 지점부터 재생 (촬영용 — 원하는 장면으로 바로 간다)
#   --paused  : 멈춘 채로 시작. 터미널에서 space 로 재생/정지
#
#   ── realtake6 장면표 (bag 기록시각) ────────────────────────────
#     12.6 PATROL · 68.6 APPROACH · 89.6 SCAN_AREA · 133.6 GATHER
#    146.1 GUIDE  · 163.0 연결복도 진입 · 188.7 한가운데 · 214.3 이탈
#    247.6 HOLD   · 252.1 SEARCH_BACK · 272.1 GUIDE · 319.1 ESCAPED
#
#   예)  bash console/run_console.sh --bag realtake6
#        bash console/run_console.sh --bag realtake6 --at 180            # 연결복도
#        bash console/run_console.sh --bag realtake6 --at 180 --rate 0.25 # 메뉴 넘기며 캡처
#        bash console/run_console.sh --bag realtake6 --rate 2 --loop
#
#   종료: Ctrl+C (모두 정리)
# ⚠ set -u 는 ROS setup source 뒤에 (테스트/스크립트 함정 ①)
# ═══════════════════════════════════════════════════════════════════
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true
set -eu

BAG=""; RATE="1"; LOOP=""; AT=""; PAUSED=""
while [ $# -gt 0 ]; do
  case "$1" in
    --bag)  BAG="${2:-}"; shift 2 ;;
    --rate) RATE="${2:-1}"; shift 2 ;;
    --loop) LOOP="--loop"; shift ;;
    --at)   AT="--start-offset ${2:-0}"; shift 2 ;;
    --paused) PAUSED="--start-paused"; shift ;;
    *) echo "알 수 없는 옵션: $1"; exit 2 ;;
  esac
done

if ! ros2 pkg list 2>/dev/null | grep -q rosbridge_server; then
  echo "❌ rosbridge 미설치. 먼저:  sudo apt install ros-humble-rosbridge-suite"
  exit 1
fi

CONSOLE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_PID=""; HTTP_PID=""

cleanup() {
  # pkill -f 자기매칭 함정 회피: 브래킷 트릭
  pkill -f "rosbridge[_]websocket" 2>/dev/null || true
  [ -n "$BAG_PID" ]  && kill "$BAG_PID"  2>/dev/null || true
  [ -n "$HTTP_PID" ] && kill "$HTTP_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "▶ rosbridge 시작 (ws://localhost:9090)"
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
sleep 2

if [ -n "$BAG" ]; then
  BAG_PATH="$HOME/robot_evidence/$BAG"
  if [ ! -d "$BAG_PATH" ]; then echo "❌ bag 없음: $BAG_PATH"; exit 1; fi
  echo "▶ bag 재생: $BAG  (rate=$RATE ${LOOP:-단발}${AT:+ ${AT#--start-offset }초부터}${PAUSED:+ · 멈춘 채 시작})"
  [ -n "$PAUSED" ] && echo "  ⏸ 터미널에서 space = 재생/정지"
  echo "  🔴 이것은 08-23 실차 기록의 재생이다. 실시간 주행이 아니다."
  # shellcheck disable=SC2086
  ros2 bag play "$BAG_PATH" --rate "$RATE" $LOOP $AT $PAUSED &
  BAG_PID=$!
fi

echo "▶ 웹서버 시작 → 브라우저에서 http://localhost:8000"
cd "$CONSOLE_DIR"
python3 -m http.server 8000 >/dev/null 2>&1 &
HTTP_PID=$!

wait
