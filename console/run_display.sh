#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════
# 로봇 몸통 디스플레이 기동 (2026-09-18 신설) — Jetson 에서 실행
#
#   rosbridge(:9090) + 캐시금지 웹서버(:8000) + 브라우저 전체화면(kiosk)
#   → 7인치 패널에 http://localhost:8000/?display=1 이 뜬다
#
#   real_bringup.launch.py 에는 rosbridge 가 없다. 로봇 스택과 별개로
#   이 스크립트가 따로 뜬다 — 죽어도 로봇은 계속 간다(반대도 마찬가지).
#
#   사용:
#     bash ~/ros2_ws/console/run_display.sh                 # 지금 띄운다
#     bash ~/ros2_ws/console/run_display.sh --install-autostart   # 로그인하면 자동으로 뜨게 등록
#     bash ~/ros2_ws/console/run_display.sh --remove-autostart
#     bash ~/ros2_ws/console/run_display.sh --no-browser    # 서버만 (다른 PC 브라우저로 볼 때)
#
#   종료: Ctrl+C (브라우저·서버 모두 정리)
#
# 🔴 run_console.sh 와 같이 띄우지 않는다 — 둘 다 :9090/:8000 을 쓴다.
#    (시작할 때 이전 실행 잔재를 죽이는 것도 run_console.sh 와 같은 이유 = PITFALLS §20-③)
# ⚠ set -u 는 ROS setup source 뒤에 (테스트/스크립트 함정 ①)
# ═══════════════════════════════════════════════════════════════════
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true
set -eu

CONSOLE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
URL="http://localhost:8000/?display=1"
AUTOSTART="$HOME/.config/autostart/tunnel-display.desktop"
NO_BROWSER=""

# ── 자동 시작 등록/해제 ─────────────────────────────────────────────
#   GNOME 데스크톱은 로그인 직후 ~/.config/autostart/*.desktop 을 실행한다.
#   systemd 서비스가 아니라 이걸 쓰는 이유: 브라우저는 그래픽 세션 안에서만
#   뜬다(DISPLAY·Wayland 소켓). 세션 밖 systemd 에서 띄우면 화면 없이 죽는다.
#   🔴 자동 로그인(설정 → 사용자 → 자동 로그인)이 켜져 있어야 부팅만으로 뜬다.
case "${1:-}" in
  --install-autostart)
    mkdir -p "$(dirname "$AUTOSTART")"
    cat > "$AUTOSTART" <<EOF
[Desktop Entry]
Type=Application
Name=Tunnel Robot Display
Comment=로봇 몸통 디스플레이 (rosbridge + 웹서버 + kiosk 브라우저)
Exec=bash -c 'sleep 8; bash $CONSOLE_DIR/run_display.sh'
Terminal=false
X-GNOME-Autostart-enabled=true
EOF
    echo "✅ 등록: $AUTOSTART  (로그인 8초 뒤 자동 실행)"
    echo "   해제: bash $CONSOLE_DIR/run_display.sh --remove-autostart"
    exit 0 ;;
  --remove-autostart)
    rm -f "$AUTOSTART"; echo "✅ 해제: $AUTOSTART"; exit 0 ;;
  --no-browser) NO_BROWSER=1 ;;
  "") ;;
  *) echo "알 수 없는 옵션: $1"; exit 2 ;;
esac

if ! ros2 pkg list 2>/dev/null | grep -q rosbridge_server; then
  echo "❌ rosbridge 미설치. 먼저:  sudo apt install ros-humble-rosbridge-suite"
  exit 1
fi

# ── 브라우저 고르기 — 있는 것 아무거나. kiosk = 주소창·탭·닫기 버튼 없음 ──
BROWSER=""; BROWSER_ARGS=()
for b in chromium chromium-browser google-chrome; do
  if command -v "$b" >/dev/null 2>&1; then
    # 🔴 snap 크로미움은 젯슨에서 실행 자체가 안 된다 (09-18) — 있어도 건너뛴다
    case "$(command -v "$b")" in /snap/*) continue ;; esac
    BROWSER="$b"
    # --incognito : 이전 세션 복원 팝업("비정상 종료") 차단 — 전원을 그냥 끄는 로봇이라 매번 뜬다
    # --kiosk     : 전체화면 + 조작 UI 없음
    # --autoplay-policy : 사용자 클릭 없이도 소리 재생 허용 — 안내 음성·싸이렌(js/audio.js).
    #                     이 인자가 없으면 화면은 뜨고 소리만 조용히 안 난다
    BROWSER_ARGS=(--kiosk --incognito --noerrdialogs --disable-infobars --disable-session-crashed-bubble
                  --no-first-run --autoplay-policy=no-user-gesture-required
                  --window-size=1024,600 --window-position=0,0 "$URL")
    break
  fi
done
if [ -z "$BROWSER" ] && command -v firefox >/dev/null 2>&1; then
  # 🔵 젯슨(JetPack 6)은 snap 크로미움이 안 뜬다(SELinux 검사에서 죽음 · 09-18 실측).
  #    Mozilla 공식 deb 파이어폭스를 쓴다. 인자 대신 **전용 프로필의 user.js** 로 설정한다 —
  #    자동재생 허용(안내 음성·싸이렌) · 비정상종료 복구 팝업 끔 · 첫 실행 안내 끔 · 업데이트 끔.
  #    sudo 없이 되고, 사람이 쓰는 파이어폭스 프로필을 건드리지 않는다.
  BROWSER=firefox
  FF_PROFILE="$HOME/.tunnel-display-firefox"
  mkdir -p "$FF_PROFILE"
  cat > "$FF_PROFILE/user.js" <<'EOF_USERJS'
// 로봇 디스플레이 전용 — run_display.sh 가 매번 다시 쓴다
user_pref("media.autoplay.default", 0);                 // 0 = 소리 있는 자동재생 허용
user_pref("media.autoplay.blocking_policy", 0);
user_pref("browser.sessionstore.resume_from_crash", false);
user_pref("browser.startup.couldRestoreSession.count", -1);   // "Open previous tabs?" 띠 금지 (09-18 실측)
user_pref("browser.startup.page", 0);
user_pref("browser.shell.checkDefaultBrowser", false);
user_pref("browser.startup.homepage_override.mstone", "ignore");
user_pref("datareporting.policy.dataSubmissionPolicyBypassNotification", true);
user_pref("app.update.auto", false);
user_pref("browser.tabs.warnOnClose", false);
user_pref("dom.disable_beforeunload", true);
user_pref("full-screen-api.warning.timeout", 0);
EOF_USERJS
  # 강제 종료 뒤 남는 세션 파일이 "이전 탭 복구?" 띠를 만든다 — 시작 전에 지운다
  rm -rf "$FF_PROFILE/sessionstore-backups" "$FF_PROFILE/sessionstore.jsonlz4" 2>/dev/null || true
  BROWSER_ARGS=(--kiosk --profile "$FF_PROFILE" --no-remote "$URL")
fi
if [ -z "$NO_BROWSER" ] && [ -z "$BROWSER" ]; then
  echo "❌ 브라우저 없음. 먼저:  sudo apt install chromium-browser   (또는 firefox)"
  exit 1
fi

# ── 이전 실행 잔재 정리 (자기 자신·부모는 제외) ──────────────────────
#   🔴 브라우저 패턴은 브라우저 이름과 묶는다. "display=1" 만으로 찾으면 그 문자열이
#      들어간 **아무 명령줄**(curl 시험·다른 터미널)까지 -9 로 죽인다 — 09-18 노트북
#      시험에서 실제로 시험 셸이 죽었다 (AGENTS §4-1 자기매칭 함정의 형제 버전).
for _pat in "rosbridge_websocket" "console/serve.py" "(chromium|chrome|firefox).*display=1"; do
  for _pid in $(pgrep -f "$_pat" 2>/dev/null); do
    [ "$_pid" = "$$" ] && continue
    [ "$_pid" = "$PPID" ] && continue
    kill -9 "$_pid" 2>/dev/null || true
  done
done
sleep 1
HTTP_PID=""; BR_PID=""

cleanup() {
  pkill -f "rosbridge[_]websocket" 2>/dev/null || true
  [ -n "$HTTP_PID" ] && kill "$HTTP_PID" 2>/dev/null || true
  [ -n "$BR_PID" ]   && kill "$BR_PID"   2>/dev/null || true
}
trap cleanup EXIT INT TERM

# ── 터치 입력 끄기 — 대피자가 만져도 화면이 안 움직이게 ────────────────
#   패널 mini-USB 를 꽂으면 터치(@dacai usb touch)가 마우스로 잡힌다. 09-18 책상 검증에서
#   터치가 kiosk 창을 500x120 으로 끌어 놓아 바탕화면이 드러났다. 우리는 터치를 안 쓴다(결정).
if command -v xinput >/dev/null 2>&1; then
  #   ⚠ 이미 꺼진 장치는 이름 앞에 '∼' 가 붙어 나온다 — 그대로 넘기면 실패하고 set -e 가 스크립트를 죽인다 (09-18)
  xinput list --name-only 2>/dev/null | grep -i touch | sed 's/^[∼~] *//' | while read -r dev; do
    xinput disable "$dev" 2>/dev/null && echo "▶ 터치 입력 끔: $dev" || true
  done || true
fi

# ── 바탕화면 아이콘 끄기 — GNOME 확장(ding)이 아이콘을 kiosk 창 위에 그린다 (09-18 실측) ──
if command -v gnome-extensions >/dev/null 2>&1; then
  gnome-extensions disable ding@rastersoft.com 2>/dev/null && echo "▶ 바탕화면 아이콘 끔" || true
fi

# ── 화면 꺼짐 방지 — 디스플레이는 몇 시간이고 켜 둔다 ──────────────────
#   X11 이면 xset, GNOME 이면 gsettings. 둘 다 실패해도 진행한다(없는 환경).
xset s off -dpms 2>/dev/null || true
gsettings set org.gnome.desktop.session idle-delay 0 2>/dev/null || true
gsettings set org.gnome.desktop.screensaver lock-enabled false 2>/dev/null || true

# ── 소리 출력을 패널(DP/HDMI)로 ────────────────────────────────────
#   젯슨 DP 는 영상과 소리를 같이 실어 보낸다. 패널 PCB 앰프 → 스피커 4Ω 2W.
#   기본 출력이 다른 장치(내장 코덱·USB)면 브라우저 소리가 거기로 간다 → HDMI sink 를 기본으로.
#   pactl 이 없거나 sink 가 안 잡히면 그냥 진행한다 (책상 검증에서 `pactl list short sinks` 로 본다).
if command -v pactl >/dev/null 2>&1; then
  _hdmi="$(pactl list short sinks 2>/dev/null | grep -i -m1 -E 'hdmi|dp|displayport' | cut -f2 || true)"
  if [ -n "$_hdmi" ]; then
    pactl set-default-sink "$_hdmi" 2>/dev/null && echo "▶ 소리 출력 → $_hdmi" || true
    pactl set-sink-volume "$_hdmi" 80% 2>/dev/null || true
    pactl set-sink-mute "$_hdmi" 0 2>/dev/null || true
  else
    echo "⚠ HDMI/DP 오디오 sink 를 못 찾았다 — 소리는 안 나도 화면은 뜬다"
  fi
fi

echo "▶ rosbridge 시작 (ws://localhost:9090)"
ros2 launch rosbridge_server rosbridge_websocket_launch.xml >/tmp/display_rosbridge.log 2>&1 &
sleep 2

echo "▶ 웹서버 시작 (:8000, 캐시 금지)"
python3 "$CONSOLE_DIR/serve.py" 8000 "$CONSOLE_DIR" >/dev/null 2>&1 &
HTTP_PID=$!
# 서버가 실제로 응답할 때까지 기다린다 — 브라우저가 먼저 뜨면 "연결할 수 없음" 페이지가 박힌다
for _ in $(seq 1 20); do
  curl -s -o /dev/null "http://localhost:8000/" 2>/dev/null && break
  sleep 0.5
done

if [ -n "$NO_BROWSER" ]; then
  echo "▶ 브라우저는 안 띄움. 다른 PC 에서:  http://<젯슨IP>:8000/?display=1"
else
  echo "▶ 브라우저($BROWSER) kiosk → $URL"
  "$BROWSER" "${BROWSER_ARGS[@]}" >/tmp/display_browser.log 2>&1 &
  BR_PID=$!
  # 🔴 GNOME 은 자동 로그인 직후 "Activities"(앱 목록) 화면을 띄운 채 두고, 나중에 뜬 kiosk
  #    창이 그것을 닫지 못한다 (09-18 패널 캡처). Escape 두 번 = 앱 목록 → 창 미리보기 → 창.
  #    xdotool 이 없어 libXtst 를 직접 부른다 (console/x11_key.py). 창이 뜬 뒤에 보내야 한다.
  sleep 8
  for _ in 1 2; do python3 "$CONSOLE_DIR/x11_key.py" Escape 2>/dev/null || true; sleep 1; done
fi

echo "  (Ctrl+C 로 전부 종료)"
wait
