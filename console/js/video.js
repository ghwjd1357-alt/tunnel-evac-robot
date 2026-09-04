/* ═══════════════════════════════════════════════════════════════════
   video.js — 영상 · 의사소통 화면 (2026-09-04)

   관제 메인의 '로봇 시야'는 작은 확인용이고, 여기는 **크게 보는 자리**다.
   현장을 보면서 그 자리에서 말을 걸 수 있어야 해서 의사소통 칸을 같이 둔다.

   🔴 카메라 영상은 아직 없다 — 예약 66(카메라를 꽂고 주행하면 라이다가 죽는다).
      실물이 붙으면 web_video_server 의 MJPEG 스트림을 아래 IMG 자리에 끼우면 된다:
        <img src="http://<로봇>:8080/stream?topic=/camera/color/image_raw">
   🔴 `/display_msg` 를 받는 쪽도 아직 없다 — 디스플레이는 구동부팀이 설치했고
      연결은 미완이다. 그래서 '보냄'과 '도착함'을 화면에서 구분해 적는다.
   ═══════════════════════════════════════════════════════════════════ */

import { state, onChange } from './state.js';
import { sendDisplay } from './ros.js';
import { hms } from './i18n.js';

/* MOCK — 역할 B 결합 전까지 화면을 채우는 시연용 값 */
const MOCK = [{ cls: 'person_ok', conf: 0.91 }, { cls: 'fire', conf: 0.87 }];

const sent = [];   // [{t, text}] 보낸 문구 이력

export function setupVideo() {
  for (const b of document.querySelectorAll('#comm .quick button')) {
    b.onclick = () => say(b.dataset.say);
  }
  const input = document.getElementById('say-text');
  document.getElementById('say-send').onclick = () => { say(input.value); input.value = ''; };
  input.addEventListener('keydown', e => {
    if (e.key === 'Enter') { say(input.value); input.value = ''; }
  });

  onChange(render);
}

function say(text) {
  const t = String(text || '').trim();
  if (!t) return;
  const ok = sendDisplay(t);
  sent.unshift({ t: Date.now(), text: t, ok });
  if (sent.length > 20) sent.pop();
  renderSaid();
}

function renderSaid() {
  const el = document.getElementById('say-log');
  if (!el) return;
  el.innerHTML = sent.length
    ? sent.map(x => `<div class="said"><span class="t">${hms(x.t)}</span>` +
                    `<span class="m">${esc(x.text)}</span></div>`).join('')
    : '';
  const st = document.getElementById('say-state');
  if (st) {
    st.textContent = sent.length
      ? `${sent.length}건 보냄 — 로봇 디스플레이 연결은 미완입니다 (도착 확인 불가)`
      : '아직 보낸 문구가 없습니다';
  }
}

function render(s) {
  if (s.menu !== 'video') return;

  const live = s.detLastMs > 0;
  const list = live ? s.detections : MOCK;
  txt('v-person', String(list.filter(d => d.cls.startsWith('person')).length));
  txt('v-fire',   String(list.filter(d => d.cls === 'fire' || d.cls === 'smoke').length));

  const src = document.getElementById('v-src');
  if (src) src.textContent = live ? '' : ' · 데모 데이터';

  const age = s.detLastMs ? (Date.now() - s.detLastMs) / 1000 : null;
  set('v-age', age == null ? '수신 없음' : `${age.toFixed(1)}초 전`, age == null ? 'off' : 'ok');
  set('v-adapter', s.adapter || '입력 없음', s.adapter ? '' : 'off');
}

const txt = (id, v) => { const e = document.getElementById(id); if (e) e.textContent = v; };
const set = (id, v, c = '') => { const e = document.getElementById(id); if (!e) return; e.textContent = v; e.className = c; };
const esc = t => String(t).replace(/[<>&]/g, c => ({ '<': '&lt;', '>': '&gt;', '&': '&amp;' }[c]));
