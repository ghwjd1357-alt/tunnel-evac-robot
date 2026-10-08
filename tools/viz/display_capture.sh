#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════
# 디스플레이 화면 캡처·녹화 — PPT·보고서용 (2026-10-09)
#
#   노트북에서 가상 화면(Xvfb 1024×600 = 패널과 같은 크기)에 디스플레이 모드를 띄우고,
#   실차 기록(realtake6)의 구간을 재생하며 ① 상태별 PNG ② 상태가 바뀌는 mp4 를 뽑는다.
#   패널(젯슨)에 뜨는 것과 **같은 웹 프로그램·같은 해상도**다.
#
#   사전 1회:  sudo apt install -y xvfb
#   실행:      bash tools/viz/display_capture.sh [출력폴더]
#              (기본 출력 = ~/Desktop/디스플레이_PPT자료_1009)
#
#   🔴 산출물은 **기록 재생**이다 — 실시간 주행이 아니다. 영상 파일명·자막에 배속을 적는다.
#   ⚠ set -u 는 ROS setup source 뒤에 (테스트/스크립트 함정 ①)
# ═══════════════════════════════════════════════════════════════════
source /opt/ros/humble/setup.bash
set -eu

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="${1:-$HOME/Desktop/디스플레이_PPT자료_1009}"
SRC="$HOME/robot_evidence/realtake6"
WORK="$HOME/.cache/display_capture"          # snap 파이어폭스가 /tmp 에 못 쓴다 — 홈 아래
DSP=":99"
mkdir -p "$OUT/screens" "$OUT/video" "$WORK"
command -v Xvfb >/dev/null || { echo "❌ Xvfb 없음:  sudo apt install -y xvfb"; exit 1; }
[ -d "$SRC" ] || { echo "❌ 원본 bag 없음: $SRC"; exit 1; }

XV_PID=""; SV_PID=""; FF_PID=""
cleanup() {
  pkill -f "ffprof_[c]apture" 2>/dev/null || true
  pkill -f "ba[g] play" 2>/dev/null || true
  [ -n "$SV_PID" ] && kill "$SV_PID" 2>/dev/null || true
  pkill -f "rosbridge[_]websocket" 2>/dev/null || true
  [ -n "$XV_PID" ] && kill "$XV_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "▶ 가상 화면 $DSP 1024x600"
Xvfb "$DSP" -screen 0 1024x600x24 -nolisten tcp >/dev/null 2>&1 &
XV_PID=$!; sleep 1

echo "▶ rosbridge + 웹서버 (브라우저 없이)"
bash "$WS/console/run_display.sh" --no-browser >"$WORK/server.log" 2>&1 &
SV_PID=$!; sleep 6

# 파이어폭스 — 매 장면 새로 띄운다(지나온 길·마커가 이전 장면에서 넘어오지 않게).
# 지도는 같은 프로필의 localStorage 캐시가 이어 준다(console/js/map.js 지도 캐시).
# 🔴 10-09 — 우분투 22.04 의 firefox 는 snap 이라 **숨김 폴더(~/.cache 등)를 못 읽는다.**
#    프로필을 거기 두면 조용히 사용자 기본 프로필로 넘어가고, 데스크톱 파이어폭스가 그걸
#    쥐고 있어 "Firefox is already running" 창만 찍혔다 — 15개 산출물이 전부 그 화면이었고
#    스크립트는 rc=0 이었다. 프로필은 **숨김이 아닌** 홈 아래에 둔다.
PROF="$HOME/tmp_shot/ffprof_capture"; mkdir -p "$PROF"
cat > "$PROF/user.js" <<'EOF'
user_pref("browser.sessionstore.resume_from_crash", false);
user_pref("browser.startup.couldRestoreSession.count", -1);
user_pref("browser.shell.checkDefaultBrowser", false);
user_pref("datareporting.policy.dataSubmissionPolicyBypassNotification", true);
user_pref("browser.startup.homepage_override.mstone", "ignore");
EOF
RB_LOG=/tmp/display_rosbridge.log          # run_display.sh 가 rosbridge 출력을 여기 쓴다
# ⚠ `grep -c … || echo 0` 은 쓰지 않는다 — 0건이면 grep 이 "0" 을 찍고 **실패**해서 "0\n0" 이 된다
clients() { local n; n=$(grep -c "Client connected" "$RB_LOG" 2>/dev/null) || true; echo "${n:-0}"; }
browser_up() {
  # $! 는 snap 껍데기의 PID 라 진짜 파이어폭스가 안 죽는다 — 프로필 경로로 찾아 죽인다
  pkill -f "ffprof_[c]apture" 2>/dev/null || true; sleep 1.5
  rm -rf "$PROF/sessionstore-backups" "$PROF/sessionstore.jsonlz4" "$PROF/lock" "$PROF/.parentlock"
  local before now; before=$(clients)
  DISPLAY="$DSP" firefox --kiosk --no-remote --profile "$PROF" --width 1024 --height 600 \
      "http://localhost:8000/?display=1" >/dev/null 2>&1 &
  FF_PID=$!
  # 🔴 페이지가 rosbridge 에 **실제로 붙었는지** 확인한다 — 안 붙으면 오류 창을 찍고도 성공한다
  for _ in $(seq 1 30); do
    now=$(clients); [ "$now" -gt "$before" ] && { sleep 2; return 0; }
    sleep 0.5
  done
  echo "❌ 브라우저가 rosbridge 에 붙지 않았다 — 캡처 중단 (프로필·포트 확인)"; exit 1
}
grab() {   # grab <파일>  — 지금 화면 한 장 (마우스 커서는 그리지 않는다)
  ffmpeg -loglevel error -y -f x11grab -draw_mouse 0 -video_size 1024x600 -i "$DSP" -frames:v 1 "$1"
}
# 🔴 10-09 — `ros2 bag play` 는 시작하자마자 첫 메시지를 내보내 rosbridge 와의 발견(DDS discovery)
#    전에 **앞쪽 /map·/mission_state 가 유실**된다. 구간 안에 /map 재발행이 늦게 있으면 화면이
#    "지도 수신 대기" 로 찍혔다(05_GUIDE). → `-d 1.5` 로 발견이 끝난 뒤에 재생을 시작한다.
PLAY_DELAY=1.5
slice() {  # slice <이름> <구간…> [-- 토픽…]
  local n="$1"; shift
  local args=() topics=()
  while [ $# -gt 0 ]; do
    if [ "$1" = "--" ]; then shift; topics=("$@"); break; fi
    args+=(--window "$1"); shift
  done
  [ ${#topics[@]} -gt 0 ] && args+=(--topics "${topics[@]}")
  python3 "$WS/tools/bag_slice.py" "$SRC" "$WORK/$n" "${args[@]}" >/dev/null
}

# ── ① 상태별 화면 — 실차 기록에서 각 상태의 한가운데 ──────────────────
#   전이 시각(bag 기록시각): PATROL 13.3 · APPROACH 69.3 · SCAN_AREA 90.3 · GATHER 134.3 ·
#   GUIDE 146.8 · HOLD 248.3 · SEARCH_BACK 252.8 · GUIDE 272.8 · ESCAPED 319.8
#   각 구간 4초를 재생하고 3초 지점(상태 문구·로봇 위치가 다 들어온 뒤)에서 찍는다.
SHOTS=(
  "01_PATROL_순찰중:40:44"
  "02_APPROACH_화재확인중:78:82"
  "03_SCAN_AREA_주변을살피는중:110:114"
  "04_GATHER_여기서기다리세요:139:143"
  "05_GUIDE_따라오세요:196:200"
  "06_HOLD_잠시멈춥니다:248.6:252.6"
  "07_SEARCH_BACK_되돌아갑니다:262:266"
  "08_ESCAPED_대피완료:321:325"
)
for s in "${SHOTS[@]}"; do
  IFS=: read -r name a b <<<"$s"
  slice "shot_$name" "$a:$b"
  browser_up
  ros2 bag play "$WORK/shot_$name" -d "$PLAY_DELAY" >/dev/null 2>&1 &
  BP=$!; sleep 4.7
  grab "$OUT/screens/$name.png"; echo "  📷 $name"
  wait "$BP" 2>/dev/null || true
done

# ── ② 기록에 없는 상태 — 문구 예시 ─────────────────────────────────────
#   🔴 실차 기록이 아니라 /mission_state 를 직접 발행해 띄운 화면이다. 파일명에 '예시'.
#   배경(지도·로봇 위치)은 기록에서 **상태 토픽을 뺀** 구간을 반복 재생해 깐다 —
#   지도 캐시에 기대면 새 브라우저에서 빈 지도가 찍혔다(10-09).
slice "bg_scan" "112:116" -- /map /tf /tf_static /scan       # 집결지(SCAN_AREA 자리)
slice "bg_guide" "196:200" -- /map /tf /tf_static /scan /plan # 연결통로(GUIDE 자리)
example() {  # example <배경> <상태|''> <파일> <발행초> <찍는시점>
  local bg="$1" state="$2" name="$3" pub="$4" at="$5"
  browser_up
  ros2 bag play "$WORK/$bg" -d "$PLAY_DELAY" --loop >/dev/null 2>&1 &
  local BP=$!; sleep 2
  [ -n "$state" ] && { timeout "$pub" ros2 topic pub -r 2 /mission_state std_msgs/msg/String \
                       "{data: $state}" >/dev/null 2>&1 & }
  sleep "$at"; grab "$OUT/screens/$name.png"; echo "  📷 $name"
  kill "$BP" 2>/dev/null || true; pkill -f "ba[g] play" 2>/dev/null || true; sleep 1
}
example bg_scan  RESCUE    09_예시_RESCUE_구조가필요합니다 8 4.5
example bg_scan  NO_VICTIM 10_예시_NO_VICTIM_대기중       8 4.5
example bg_scan  FAULT     11_예시_FAULT_정지             8 4.5
#   안전 동작: "따라오세요" 를 3초 내다가 끊는다 → 5초 뒤 화면이 낡은 안내를 지우고 "신호 없음"
example bg_guide GUIDE     12_안전동작_신호없음           3 9.5

# ── ③ 영상 — 끊김 없는 한 구간을 그대로 재생 (편집 없음 · 배속만) ─────────
record() {  # record <파일> <구간> <배속> <초>
  local f="$1" w="$2" r="$3" sec="$4"
  slice "vid" "$w"; browser_up
  ros2 bag play "$WORK/vid" --rate "$r" -d "$PLAY_DELAY" >/dev/null 2>&1 &
  # 🔴 재생 1초 뒤부터 찍는다 — 첫 상태·지도가 화면에 들어온 뒤라 "연결 중" 프레임이 안 섞인다
  #    (10-09 첫 판은 녹화를 먼저 켜서 세 영상 모두 1~3초가 "연결 중" 이었다)
  sleep "$(python3 -c "print($PLAY_DELAY + 1.0)")"
  ffmpeg -loglevel error -y -f x11grab -draw_mouse 0 -framerate 30 -video_size 1024x600 -i "$DSP" \
         -t "$sec" -c:v libx264 -pix_fmt yuv420p -crf 20 "$f"
  pkill -f "ba[g] play" 2>/dev/null || true
  echo "  🎬 $(basename "$f")"
}
#   녹화 시작 = 구간 시작 + 1초×배속. 아래 구간은 그만큼 앞당겨 잡았다.
#   A  집결 → 유도 시작      SCAN_AREA → GATHER → GUIDE   (기록 131~151 s · 1.5배속 · 13.5초)
record "$OUT/video/A_집결에서유도까지_기록재생_1.5배속.mp4" "129.5:152" 1.5 13.5
#   B  놓침 → 재탐색 → 복귀  HOLD → SEARCH_BACK → GUIDE    (기록 246~280 s · 2배속 · 17초)
record "$OUT/video/B_놓침재탐색복귀_기록재생_2배속.mp4" "244:282" 2 17
#   C  대피 완료            GUIDE → ESCAPED              (기록 315~326 s · 1배속 · 11초)
record "$OUT/video/C_대피완료_기록재생_1배속.mp4" "314:327" 1 11

echo "🟢 완료 → $OUT/screens · $OUT/video"
