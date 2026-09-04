/* ═══════════════════════════════════════════════════════════════════
   camfeed.js — 🎬 DEMO-0904  카메라 자리에 실사 클립을 재생한다
   ═══════════════════════════════════════════════════════════════════

   시연 영상은 역할 B 인지·카메라가 **결합 완료된 기준**으로 찍는다. 실사는 로봇에
   카메라를 붙여 따로 촬영하고, 그 클립을 여기서 재생해 **관제 화면 안에서 한 번에**
   녹화한다 (편집에서 합성하는 것보다 화면이 자연스럽고 한 테이크로 끝난다).

   ── 쓰는 법 ────────────────────────────────────────────────────
   클립을 `console/media/robot_view.mp4` 로 넣기만 하면 된다. 파일이 없으면
   자동으로 `CAMERA` 자리표시로 되돌아간다 — 없다고 화면이 깨지지 않는다.

   ── 실물이 붙으면 ──────────────────────────────────────────────
   web_video_server 의 MJPEG 스트림으로 바꾼다:
     <img src="http://<로봇>:8080/stream?topic=/camera/color/image_raw">
   그때 이 파일을 지운다.
   ═══════════════════════════════════════════════════════════════ */

const CLIP = 'media/robot_view.mp4';

export function setupCamFeed() {
  for (const v of document.querySelectorAll('video.camfeed')) {
    const box = v.parentElement;
    v.addEventListener('loadeddata', () => {
      box.classList.add('live');
      v.play().catch(() => {});            // 자동재생이 막히면 조용히 자리표시로 남는다
    }, { once: true });
    v.addEventListener('error', () => box.classList.remove('live'), { once: true });
    v.src = CLIP;
    v.load();
  }
}
