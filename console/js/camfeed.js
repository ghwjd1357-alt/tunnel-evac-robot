/* ═══════════════════════════════════════════════════════════════════
   camfeed.js — 🎬 DEMO-0904  카메라 자리에 실사 클립을 재생한다
   ═══════════════════════════════════════════════════════════════════

   시연 영상은 역할 B 인지·카메라가 **결합 완료된 기준**으로 찍는다. 실사는 로봇에
   카메라를 붙여 따로 촬영하고, 그 클립을 여기서 재생해 **관제 화면 안에서 한 번에**
   녹화한다 (편집에서 합성하는 것보다 화면이 자연스럽고 한 테이크로 끝난다).

   ── 쓰는 법 ────────────────────────────────────────────────────
   클립을 `console/media/robot_view.webm` (또는 `.mp4`) 로 넣기만 하면 된다.
   둘 다 없으면 자동으로 `CAMERA` 자리표시로 되돌아간다 — 없다고 화면이 깨지지 않는다.

   ── 🔴 webm 을 먼저 두는 이유 ──────────────────────────────────
   우분투 Firefox 는 H.264(mp4) 를 자체 디코더가 아니라 **시스템 gstreamer 코덱**으로
   재생한다. 코덱이 없는 기본 설치에서는 mp4 가 조용히 실패해 카메라 자리가 검게 남는다
   (2026-09-04 실제로 이 상태였다 — 코덱 패키지 0개).
   VP9/webm 은 Firefox·Chrome 모두 **자체 디코더**로 재생하므로 설치에 의존하지 않는다.
   시연 당일 다른 노트북을 쓰게 되어도 이쪽이 안전하다.

   ── 실물이 붙으면 ──────────────────────────────────────────────
   web_video_server 의 MJPEG 스트림으로 바꾼다:
     <img src="http://<로봇>:8080/stream?topic=/camera/color/image_raw">
   그때 이 파일을 지운다.
   ═══════════════════════════════════════════════════════════════ */

// 앞에서부터 시도한다. 하나가 실패하면 다음으로 넘어간다.
const CLIPS = ['media/robot_view.webm', 'media/robot_view.mp4'];

/* 🔵 09-18 실물 결합 — 로봇의 web_video_server(:8080) MJPEG 를 **먼저** 시도한다.
   관제 페이지를 로봇(젯슨)에서 서빙하므로 호스트는 페이지 주소와 같다. 스냅샷 한 장이
   받아지면 살아 있는 것으로 보고 <img> 스트림으로 바꾼다. 안 받아지면(노트북 단독·bag 재생)
   기존 데모 클립 → CAMERA 자리표시 순서 그대로다. 젯슨 기동 = run_display.sh 가 같이 띄운다. */
const LIVE_HOST = window.location.hostname || 'localhost';
const LIVE_TOPIC = '/camera/color/image_raw';
/* 🔵 화질은 원본 유지(사용자 결정 09-18). 핫스팟 와이파이에선 ≈5 Mbps 라 끊길 수 있다 —
   근본 대책은 노트북↔젯슨 유선/전용 공유기(시연장). */
const LIVE_STREAM = `http://${LIVE_HOST}:8080/stream?topic=${LIVE_TOPIC}&type=mjpeg&quality=60`;
const LIVE_PROBE  = `http://${LIVE_HOST}:8080/snapshot?topic=${LIVE_TOPIC}`;

export function setupCamFeed() {
  const probe = new Image();
  probe.onload = () => {
    for (const v of document.querySelectorAll('video.camfeed')) useLive(v);
  };
  probe.onerror = () => {
    for (const v of document.querySelectorAll('video.camfeed')) tryClip(v, 0);
  };
  probe.src = LIVE_PROBE + '&t=' + Date.now();
}

function useLive(v) {
  const box = v.parentElement;
  const img = document.createElement('img');
  img.className = 'camfeed';
  img.alt = '카메라 실시간';
  img.src = LIVE_STREAM;
  img.onerror = () => { img.remove(); box?.classList.remove('live'); tryClip(v, 0); };
  v.replaceWith(img);
  box?.classList.add('live');
  const note = box?.querySelector('.camnote');
  if (note) note.textContent = 'LIVE';
}

function tryClip(v, i) {
  const box = v.parentElement;
  if (i >= CLIPS.length) {
    box?.classList.remove('live');        // 전부 실패 — 자리표시로 남는다
    return;
  }
  v.addEventListener('loadeddata', () => {
    box?.classList.add('live');
    v.play().catch(() => {});             // 자동재생이 막히면 조용히 자리표시로 남는다
  }, { once: true });
  v.addEventListener('error', () => tryClip(v, i + 1), { once: true });
  v.src = CLIPS[i];
  v.load();
}
