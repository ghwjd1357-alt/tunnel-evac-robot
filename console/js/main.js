/* ═══════════════════════════════════════════════════════════════════
   main.js — 시작점. 모듈을 불러 조립한다 (2026-09-02)
   ═══════════════════════════════════════════════════════════════════ */

import { state, update, onChange, startTicker } from './state.js';
import { connect } from './ros.js';
import { setupMap, draw as drawMap } from './map.js';
import { setupMission } from './mission.js';
import { setupVideo } from './video.js';
import { setupCamFeed } from './camfeed.js';   // 🎬 DEMO-0904
import { maybeStartTour } from './demo.js';    // 🎬 DEMO-0904
import { setupDiag } from './diag.js';
import { setupRecord } from './record.js';
import { setupEmergency } from './emergency.js';
import { evaluateAlerts, healthSummary } from './alert.js';
import { hms, MODE_OF, MODE_KO, DISPLAY_KO } from './i18n.js';

/* ── 메뉴 전환: 화면 4개 중 하나만 보인다 ─────────────────────── */
function setupMenu() {
  for (const btn of document.querySelectorAll('#sidebar button[data-menu]')) {
    btn.onclick = () => showMenu(btn.dataset.menu);
  }
  document.getElementById('health-summary').onclick = () => showMenu('diag');
  document.getElementById('alert-badge').onclick = () => {
    document.getElementById('alert-list').classList.toggle('show');
  };
  // 🎬 DEMO-0904 — `?tour=3` 이면 스스로 넘어간다. 없으면 평소대로 관제부터.
  if (!maybeStartTour(showMenu)) showMenu('main');
}

function showMenu(name) {
  update({ menu: name });
  moveMap(name);
  document.querySelectorAll('#sidebar button[data-menu]')
    .forEach(b => b.classList.toggle('on', b.dataset.menu === name));
  document.querySelectorAll('.screen')
    .forEach(el => el.classList.toggle('on', el.dataset.menu === name));
}

/**
 * 지도 패널을 화면 사이로 옮긴다.
 * 🔴 비상 원격 조종은 **로봇이 어디로 가는지 보면서** 해야 한다 — 안 보이면 위험하다.
 *    캔버스를 하나 더 만들지 않고 같은 노드를 옮긴다. map.js 는 시작할 때 잡아둔
 *    엘리먼트를 계속 쓰므로 참조가 그대로 살아 있고, 크기는 draw() 가 부모에 맞춘다.
 */
function moveMap(name) {
  const panel = document.getElementById('map-panel');
  if (!panel) return;
  if (name === 'display') {
    document.getElementById('display-map').appendChild(panel);
  } else if (name === 'emergency') {
    document.getElementById('emg-map').appendChild(panel);
  } else if (panel.parentElement.id !== 'center') {
    const center = document.getElementById('center');
    center.insertBefore(panel, document.getElementById('right-col'));
  }
}

/* ── 상단바 (모든 화면 공통) ──────────────────────────────────── */
/**
 * 로봇 몸통 디스플레이에 띄우는 문구.
 * 관제가 보낸 문구(/display_msg)가 있으면 그게 우선이다 — 사람이 일부러 보낸 말이니까.
 * 없으면 임무 상태를 대피자가 읽을 말로 바꿔 띄운다.
 */
function renderDisplay(s) {
  if (!document.body?.classList?.contains('display')) return;
  const box = document.getElementById('big-state');
  const said = s.sayText && s.sayAt && (Date.now() - s.sayAt < SAY_HOLD_MS);
  box.classList.toggle('said', !!said);
  if (said) {
    txt('big-main', s.sayText);
    txt('big-sub', '관제에서 보낸 안내입니다');
    return;
  }
  const [main, sub] = DISPLAY_KO[s.mission] || ['연결 중', '관제와 연결하고 있습니다'];
  txt('big-main', main);
  txt('big-sub', sub);
}
const SAY_HOLD_MS = 20000;   // 보낸 문구를 20초 띄운 뒤 상태 문구로 돌아간다
const txt = (id, v) => { const e = document.getElementById(id); if (e) e.textContent = v; };

function renderTop(s) {
  /* 모드는 body 속성 하나로만 알린다 — 색·테두리·강조는 전부 CSS 가 처리한다.
     JS 가 스타일을 직접 만지면 규칙이 두 곳으로 흩어진다. */
  const mode = MODE_OF[s.mission] || 'normal';
  if (document.body.dataset.mode !== mode) document.body.dataset.mode = mode;
  const tag = document.getElementById('mode-tag');
  if (tag) tag.textContent = MODE_KO[mode];

  const h = healthSummary(s);
  document.getElementById('health-dot').className = 'dot ' + h.cls;
  document.getElementById('health-text').textContent = h.text;

  const badge = document.getElementById('alert-badge');
  badge.classList.toggle('show', s.alerts.length > 0);
  document.getElementById('alert-count').textContent = String(s.alerts.length);

  const list = document.getElementById('alert-body');
  list.innerHTML = s.alerts.length
    ? s.alerts.map(a =>
        `<div class="alert-item sev-${a.sev}">` +
          `<div class="what">${a.what}</div><div class="when">${hms(a.when)}</div>` +
          `<div class="why">${a.why}</div>` +
        `</div>`).join('')
    : '<div class="empty">경보 없음</div>';
  if (!s.alerts.length) document.getElementById('alert-list').classList.remove('show');
}

/* ── 기동 ─────────────────────────────────────────────────────── */
window.addEventListener('DOMContentLoaded', () => {
  /* ?display=1 → 로봇 몸통 디스플레이 모드.
     관제 UI 를 전부 숨기고 상태 문구 + 지도만 남긴다 (원안 = feature/display). */
  const isDisplay = new URLSearchParams(location.search).has('display');
  if (isDisplay) document.body.classList.add('display');

  setupMenu();
  if (isDisplay) showMenu('display');
  setupMap();
  setupMission();
  setupVideo();
  setupCamFeed();
  setupDiag();
  setupRecord();
  setupEmergency();
  onChange(renderTop);
  onChange(renderDisplay);

  connect();

  /* 경보 판정은 1초마다 (신선도 기반 규칙이 있어 값 변화만으로는 못 잡는다) */
  setInterval(evaluateAlerts, 1000);

  /* 지도는 상태 갱신과 분리해 따로 돌린다 (무거운 렌더가 화면을 막지 않게).
     🔴 20fps — 5fps 였을 때 제자리 회전(SCAN_AREA)이 뚝뚝 끊겨 보였다.
        지도 그림은 캐시된 오프스크린 캔버스를 한 번 그리는 것이라 fps 를 올려도
        비용은 점군 600점 + 폴리라인 몇 개뿐이다. */
  setInterval(drawMap, 50);

  /* 경과시간·신선도처럼 '시간이 흘러서' 바뀌는 값 때문에 주기 갱신이 필요하다.
     글자는 4Hz 면 충분하다 — 여기를 올리면 DOM 갱신만 늘고 체감은 그대로다. */
  startTicker(4);
});
