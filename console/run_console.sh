#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════
# 관제 콘솔 실행 스크립트 (0718_관제시스템.md §5)
#   rosbridge(ws:9090) + 정적 웹서버(:8000) 를 한 줄로.
#   종료: Ctrl+C (둘 다 정리)
# ⚠ set -u 는 ROS setup source 뒤에 (테스트/스크립트 함정 ①)
# ═══════════════════════════════════════════════════════════════════
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true
set -eu

# 사전 확인: rosbridge 설치 여부
if ! ros2 pkg list 2>/dev/null | grep -q rosbridge_server; then
  echo "❌ rosbridge 미설치. 먼저 실행하세요:"
  echo "   sudo apt install ros-humble-rosbridge-suite"
  exit 1
fi

CONSOLE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cleanup() {
  # 자식 프로세스 정리 (pkill -f 자기매칭 함정 회피: 브래킷 트릭)
  pkill -f "rosbridge[_]websocket" 2>/dev/null || true
  kill "${HTTP_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "▶ rosbridge 시작 (ws://localhost:9090)"
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &

echo "▶ 웹서버 시작 → 브라우저에서 http://localhost:8000"
cd "$CONSOLE_DIR"
python3 -m http.server 8000 &
HTTP_PID=$!

wait
