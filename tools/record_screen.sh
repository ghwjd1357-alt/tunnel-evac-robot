#!/usr/bin/env bash
# record_screen.sh — 촬영용 화면 녹화 (관제·RViz·터미널)
#
# 사용:
#   bash tools/record_screen.sh console     # 이름표를 붙여 파일명에 넣는다
#   bash tools/record_screen.sh rviz 1920x1080+0+0
#   Ctrl+C 로 정지 → 파일이 닫힌다
#
# 🔴 왜 스크립트로 만드나 — 화면 3종 동시 녹화는 **주행과 동시에만** 얻어지고
#   나중에 절대 못 만든다. 매번 손으로 ffmpeg 인자를 치면 테이크마다 설정이
#   갈리고, 한 번 빼먹으면 그 테이크의 관제 화면이 통째로 없다.
#
# ⚠ GNOME 내장 녹화(Ctrl+Alt+Shift+R)를 쓰지 말 것 — Ubuntu 22.04 기본값이
#   **30초에서 자동 정지**한다(`max-screencast-length`). 대본 한 테이크가 7분이라
#   그걸로는 못 찍는다. 굳이 쓰려면 먼저:
#     gsettings set org.gnome.settings-daemon.plugins.media-keys max-screencast-length 0
set -e

LABEL="${1:?이름표를 주십시오 — 예: console / rviz / term}"
GEOM="${2:-}"
OUT="$HOME/Desktop/rec_${LABEL}_$(date +%m%d_%H%M%S).mp4"

if [ -z "$GEOM" ]; then
  # 전체 화면
  GEOM=$(xdpyinfo | awk '/dimensions:/{print $2}')
  OFF="+0,0"
  SIZE="$GEOM"
else
  # "WxH+X+Y" → ffmpeg 는 -video_size WxH -i :0.0+X,Y
  SIZE="${GEOM%%+*}"
  REST="${GEOM#*+}"
  OFF="+${REST%+*},${REST#*+}"
fi

# 🔴 DISPLAY 를 ":0.0" 으로 박지 않는다 — 이 노트북은 ":1" 이다.
#   08-21 자체시험에서 실제로 여기서 실패했고, 실패해도 ffmpeg 가 조용히
#   빈 파일도 안 남겨서 "녹화한 줄 알았는데 없다"가 된다. 촬영에선 치명적이다.
DISP="${DISPLAY:-:0}"
[ "${DISP%.*}" = "$DISP" ] && DISP="${DISP}.0"

echo "녹화 시작 — ${SIZE} at ${OFF} (display ${DISP})"
echo "  → $OUT"
echo "  🔴 Ctrl+C 로 정지. 정지해야 파일이 닫힙니다."
echo

# -draw_mouse 1 : 커서를 남긴다 (관제에서 클릭하는 장면이 보여야 한다)
# crf 20        : 편집용으로 충분한 화질. 파일이 너무 크면 23 으로 올린다
# preset ultrafast : 녹화 중 CPU 를 최소로 — 주행과 같은 노트북에서 돌 수 있다
exec ffmpeg -hide_banner -loglevel warning \
  -f x11grab -framerate 30 -draw_mouse 1 \
  -video_size "$SIZE" -i "${DISP}${OFF}" \
  -c:v libx264 -preset ultrafast -crf 20 -pix_fmt yuv420p \
  "$OUT"
