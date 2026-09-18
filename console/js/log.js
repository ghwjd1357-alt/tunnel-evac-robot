/* ═══════════════════════════════════════════════════════════════════
   log.js — 이벤트 로그 (2026-09-02)

   ── 형식 ────────────────────────────────────────────────────────
   실제 관제 로그의 모양은 문장이 아니라 **열**이다.
       12:04:12  ALARM   화재 감지            12.50  -0.10
       12:04:31  STATE   APPROACH → GATHER
   시각·태그·내용·값이 열로 정렬되어야 눈이 세로로 훑을 수 있다.
   ═══════════════════════════════════════════════════════════════════ */

import { state, update } from './state.js';
import { hms } from './i18n.js';

const MAX = 500;

/**
 * @param {string} tag  대문자 짧은 태그 (STATE·ALARM·DRIVE…)
 * @param {string} msg  내용
 * @param {string} val  오른쪽에 붙는 수치 (없으면 '')
 * @param {string} cls  'state' | 'alarm' | 'warn' | 'ctrl' | ''
 */
export function pushLog(tag, msg, val = '', cls = '') {
  const logs = [{ t: Date.now(), tag, msg, val, cls }, ...state.logs].slice(0, MAX);
  update({ logs });
}

/**
 * 로그 배열 → DOM. **시간 순(오래된 것 위 → 최신 아래)** 으로 그리고 바닥에 붙는다.
 *
 * 🔴 왜 최신이 아래인가 (2026-09-03 변경)
 *   처음엔 최신을 위로 쌓았다(스크롤 없이 최신이 보인다는 이유). 그런데 사람이
 *   로그를 읽는 방식은 터미널이라 위→아래가 시간 순이라는 기대가 강하다.
 *   `tools/viz` 의 미션 로그 재생도 시간 순으로 흐른다 — 화면 두 곳이 반대로
 *   흐르면 같은 사건을 대조할 때 혼란스럽다.
 *
 * 자동 스크롤은 **사용자가 바닥 근처를 보고 있을 때만** 한다.
 * 위로 올려 과거를 읽는 중인데 새 줄이 왔다고 끌어내리면 읽을 수가 없다.
 */
export function renderLog(el, logs, limit = 200) {
  if (!logs.length) { el.innerHTML = '<div class="empty">기록된 이벤트가 없습니다</div>'; return; }
  const atBottom = el.scrollHeight - el.scrollTop - el.clientHeight < 24;
  el.innerHTML = logs.slice(0, limit).reverse().map(l =>
    `<div class="logline ${l.cls}">` +
      `<span class="ts">${hms(l.t)}</span>` +
      `<span class="tag">${l.tag}</span>` +
      `<span class="msg">${esc(l.msg)}</span>` +
      `<span class="val">${esc(l.val)}</span>` +
    `</div>`).join('');
  if (atBottom) el.scrollTop = el.scrollHeight;   // 바닥을 보고 있었으면 계속 따라간다
}

const esc = s => String(s).replace(/[<>&]/g, c => ({ '<': '&lt;', '>': '&gt;', '&': '&amp;' }[c]));
