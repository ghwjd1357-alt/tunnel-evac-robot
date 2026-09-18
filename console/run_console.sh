#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════
# 관제 콘솔 실행 (2026-09-02 개정 — bag 재생 모드 추가)
#
#   기본      : rosbridge(:9090) + 웹서버(:8000)
#   --bag TAG : 위 둘 + `~/robot_evidence/TAG` 를 재생한다
#               → 로봇 없이 08-23 실차 데이터로 관제 화면 전체가 돈다
#   --at N    : N 초 지점부터 재생 (촬영용 — 원하는 장면으로 바로 간다)
#   --paused  : 멈춘 채로 시작. 터미널에서 space 로 재생/정지
#   --on-connect : 🎬 촬영용. **브라우저가 관제를 연 순간**에 맞춰 재생을 시작한다.
#               명령 실행 시점에 틀면 주소를 입력하는 사이 로봇이 지나가
#               지도와 카메라 영상이 어긋난다. 이 옵션이 그 어긋남을 없앤다.
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
#        bash console/run_console.sh --bag realtake6 --at 163 --on-connect  # 촬영
#
#   종료: Ctrl+C (모두 정리)
# ⚠ set -u 는 ROS setup source 뒤에 (테스트/스크립트 함정 ①)
# ═══════════════════════════════════════════════════════════════════
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true
set -eu

BAG=""; RATE="1"; LOOP=""; AT=""; PAUSED=""; ONCONN=""
while [ $# -gt 0 ]; do
  case "$1" in
    --bag)  BAG="${2:-}"; shift 2 ;;
    --rate) RATE="${2:-1}"; shift 2 ;;
    --loop) LOOP="--loop"; shift ;;
    --at)   AT="--start-offset ${2:-0}"; shift 2 ;;
    --paused) PAUSED="--start-paused"; shift ;;
    --on-connect) ONCONN=1; shift ;;
    *) echo "알 수 없는 옵션: $1"; exit 2 ;;
  esac
done

if ! ros2 pkg list 2>/dev/null | grep -q rosbridge_server; then
  echo "❌ rosbridge 미설치. 먼저:  sudo apt install ros-humble-rosbridge-suite"
  exit 1
fi

CONSOLE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── 🔴 이전 실행 잔재 정리 (2026-09-04 신설) ────────────────────────
#   trap cleanup 은 스크립트가 정상 종료할 때만 돈다. kill -9 로 죽이거나
#   터미널을 닫으면 자식들이 살아남는다. 그 상태에서 다시 띄우면 **bag 이 둘**
#   돌면서 서로 다른 시점의 /tf 를 번갈아 쏜다 — 지도의 로봇이 순간이동하고
#   시계가 계속 뒤로 감겨 관제가 누적값을 반복해서 초기화한다.
#   09-04 촬영 테이크가 이것 때문에 통째로 날아갔다. 조용히 망가지는 종류라
#   화면만 보고는 원인을 못 찾는다. 그래서 시작할 때 먼저 지운다.
#   ⚠ pkill -f 는 자기 자신도 잡는다(이 스크립트 이름이 명령줄에 있다).
#      그래서 PID 를 골라 자기 자신과 부모를 뺀 뒤 죽인다.
for _pat in "bag play" "rosbridge_websocket" "console/serve.py"; do
  for _pid in $(pgrep -f "$_pat" 2>/dev/null); do
    [ "$_pid" = "$$" ] && continue
    [ "$_pid" = "$PPID" ] && continue
    kill -9 "$_pid" 2>/dev/null || true
  done
done
sleep 1
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

echo "▶ 웹서버 시작 → 브라우저에서 http://localhost:8000"
CONSOLE_DIR_EARLY="$CONSOLE_DIR"
# 🔴 캐시 금지 서버를 쓴다 — 낡은 모듈이 섞이면 화면이 조용히 "연결 대기"로 죽는다
python3 "$CONSOLE_DIR/serve.py" 8000 "$CONSOLE_DIR" >/dev/null 2>&1 &
HTTP_PID=$!
sleep 1

if [ -n "$BAG" ]; then
  BAG_PATH="$HOME/robot_evidence/$BAG"
  if [ ! -d "$BAG_PATH" ]; then echo "❌ bag 없음: $BAG_PATH"; exit 1; fi
  echo "▶ bag 재생: $BAG  (rate=$RATE ${LOOP:-단발}${AT:+ ${AT#--start-offset }초부터}${PAUSED:+ · 멈춘 채 시작})"
  [ -n "$PAUSED" ] && echo "  ⏸ 터미널에서 space = 재생/정지"
  echo "  🔴 이것은 08-23 실차 기록의 재생이다. 실시간 주행이 아니다."
  if [ -n "$ONCONN" ]; then
    # 🎬 촬영용 — 브라우저가 실제로 붙은 순간에 맞춰 재생을 시작한다.
    #    명령 실행 시점에 틀면 주소를 입력하는 사이 로봇이 지나가 버려
    #    지도와 카메라 영상이 어긋난다(09-04 촬영에서 실제로 어긋났다).
    echo "  ⏳ 브라우저 접속 대기 중 — 지금 http://localhost:8000/?tour=3 을 연다"
    python3 "$CONSOLE_DIR_EARLY/wait_client.py" 300 || true
    echo "  ▶ 접속 확인 — 재생 시작"
  fi
  # shellcheck disable=SC2086
  ros2 bag play "$BAG_PATH" --rate "$RATE" $LOOP $AT $PAUSED &
  BAG_PID=$!
fi

wait
