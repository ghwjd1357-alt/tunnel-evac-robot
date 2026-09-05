/* ═══════════════════════════════════════════════════════════════════
   record.js — 기록 화면 (2026-09-02)
     임무 요약(자동 집계) + 전체 이벤트 로그
   ═══════════════════════════════════════════════════════════════════ */

import { state, onChange } from './state.js';
import { dur, clock, STATE_KO } from './i18n.js';
import { renderLog } from './log.js';

/* 상태별 체류 시간 — /mission_state 전이 로그에서 스스로 집계한다.
   미션 노드를 안 고쳐도 되는 파생값이다. */
const dwell = {};
let lastState = null, lastT = null;

/** 🎬 DEMO-0904 — 촬영 시작 전 구간의 체류시간을 미리 채운다 (js/demo.js) */
export function seedDwell(pre, curState, curSince) {
  Object.assign(dwell, pre);
  lastState = curState; lastT = curSince;
}

export function tickDwell() {
  const s = state;
  if (s.mission !== lastState) {
    if (lastState && lastT) dwell[lastState] = (dwell[lastState] || 0) + (Date.now() - lastT);
    lastState = s.mission; lastT = Date.now();
  }
}

export function setupRecord() {
  setInterval(tickDwell, 500);
  onChange(render);
}

function render(s) {
  if (s.menu !== 'record') return;

  /* 관제 화면과 같은 기준(현재 구간)으로 맞춘다 — 두 화면이 다른 값을 보이면 안 된다 */
  stat('r-elapsed', clock(s.phaseT0 ? (Date.now() - s.phaseT0) / 1000 : null));
  stat('r-dist', Math.max(0, s.distance - s.phaseDist0).toFixed(1), 'm');
  stat('r-total', clock(s.missionT0 ? (Date.now() - s.missionT0) / 1000 : null));
  stat('r-events', String(s.logs.length));
  stat('r-alerts', String(s.alerts.length));
  stat('r-states', String(Object.keys(dwell).length + (s.mission ? 1 : 0)));

  /* 상태별 체류 시간 표 */
  const live = { ...dwell };
  if (lastState && lastT) live[lastState] = (live[lastState] || 0) + (Date.now() - lastT);
  const rows = Object.entries(live).sort((a, b) => b[1] - a[1]);
  document.getElementById('r-dwell').innerHTML = rows.length
    ? rows.map(([k, ms]) =>
        `<dt>${STATE_KO[k] || k}<span class="en"> ${k}</span></dt><dd>${dur(ms / 1000)}</dd>`).join('')
    : '<dt>아직 집계된 상태가 없습니다</dt><dd>—</dd>';

  renderLog(document.getElementById('record-log-body'), s.logs, 300);
}

function stat(id, v, unit) {
  const el = document.getElementById(id);
  if (!el) return;
  el.innerHTML = v + (unit ? `<span class="unit">${unit}</span>` : '');
}
