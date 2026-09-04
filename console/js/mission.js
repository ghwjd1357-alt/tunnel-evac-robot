/* ═══════════════════════════════════════════════════════════════════
   mission.js — 관제 메인 화면 (2026-09-02)
     ① 진행 막대   ② 로봇 시야(인지)   ③ 제어 버튼
   지도는 map.js 가 그린다.
   ═══════════════════════════════════════════════════════════════════ */

import { state, onChange } from './state.js';
import { STAGES, EXCEPTION_STATES, STATE_KO, PERSON_KO, NAV_KO,
         MODE_OF, ACTION_HINT, FOLLOWER_OF, dur, clock, hms } from './i18n.js';
import { sendCmd, sendAlarm, softStop } from './ros.js';
import { renderLog } from './log.js';
import { demoPerception } from './demo.js';   // 🎬 DEMO-0904

/* 🔴 2026-09-04: 인지 패널의 가짜 탐지 수(사람 1 · 화재 1)를 **삭제했다.**
   고정 상수라 연결복도처럼 아무것도 안 보이는 구간에서도 "1 / 1" 이라고 말했다.
   화면이 장면과 어긋나는 거짓말을 하면 관제가 아니다.
   → 지금은 **실제로 온 것만** 보여준다: /detections(미결합) · /alarm(진짜). */

export function setupMission() {
  buildProgress();

  document.getElementById('btn-reset').onclick = () => sendCmd('reset');
  document.getElementById('btn-abort').onclick = () => sendCmd('abort');
  document.getElementById('btn-estop').onclick = () => softStop();
  document.getElementById('btn-fire').onclick = () => {
    const x = parseFloat(document.getElementById('fire-x').value);
    const y = parseFloat(document.getElementById('fire-y').value);
    if (!isFinite(x) || !isFinite(y)) return;
    sendAlarm(x, y);
  };

  onChange(render);
}

/* ── 진행 막대: 12개 상태를 6단계 흐름으로 ─────────────────────── */
function buildProgress() {
  const el = document.getElementById('progress-steps');
  el.innerHTML = STAGES.map(s => `<div class="step" data-key="${s.key}">${s.label}</div>`).join('');
}

function render(s) {
  /* ── 진행 막대 ── */
  const idx = STAGES.findIndex(st => st.states.includes(s.mission));
  document.querySelectorAll('#progress-steps .step').forEach((el, i) => {
    el.classList.toggle('on', i === idx);
    el.classList.toggle('done', idx >= 0 && i < idx);
  });
  const exEl = document.getElementById('progress-exception');
  const ex = EXCEPTION_STATES[s.mission];
  exEl.classList.toggle('show', !!ex);
  if (ex) exEl.textContent = ex;

  /* ── 현재 상태 크게 ── */
  txt('state-ko', s.mission ? (STATE_KO[s.mission] || s.mission) : '수신 대기');
  txt('state-en', s.mission || '');
  txt('state-since', s.missionSince ? dur((Date.now() - s.missionSince) / 1000) : '—');

  /* 🔴 경과·이동거리는 **현재 구간 기준**이다 (대응 = 화재 감지 시점부터).
     순찰은 무한 루프라 관제 접속 시점부터 재면 아무 의미가 없다. */
  txt('mission-elapsed', clock(s.phaseT0 ? (Date.now() - s.phaseT0) / 1000 : null));
  html('mission-dist', `${Math.max(0, s.distance - s.phaseDist0).toFixed(1)}<span class="u">m</span>`);
  txt('mission-speed', `${s.speed.toFixed(2)} m/s`);

  /* 목표까지 남은 거리 = 계획 경로의 길이. 미션 노드를 안 고쳐도 되는 파생값이다.
     평시에는 '다음 순찰 지점까지', 대응 중에는 '목표까지'로 같은 값을 다르게 부른다. */
  const togo = s.planPts.length > 1 ? `${pathLen(s.planPts).toFixed(1)} m` : '—';
  txt('mission-togo', togo);
  txt('togo-n', togo);

  /* ── 모드에 따라 패널의 얼굴을 바꾼다 ── */
  const mode = MODE_OF[s.mission] || 'normal';
  txt('mission-title', mode === 'normal' ? '순찰' : mode === 'incident' ? '화재 대응' : '조치 필요');
  txt('bs-label', mode === 'normal' ? '순찰 경과' : '대응 경과');

  const box = byId('action-box');
  if (box) box.textContent = ACTION_HINT[s.mission] || '';

  const fp = byId('fire-pos');
  if (fp) {
    fp.textContent = s.fireXY ? `${s.fireXY.x.toFixed(2)}  ${s.fireXY.y.toFixed(2)}` : '—';
    fp.className = s.fireXY ? 'alarm' : 'off';
  }

  const nav = byId('mission-nav');
  nav.textContent = s.navStatus == null ? '—' : (NAV_KO[s.navStatus] || `상태 ${s.navStatus}`);
  nav.className = s.navStatus == null ? 'off'
                : s.navStatus === 6 ? 'alarm'
                : (s.navStatus === 2 || s.navStatus === 4) ? 'ok' : 'warn';

  renderTimeline(s);
  renderDiagMini(s);

  const sirenEl = document.getElementById('siren-v');
  sirenEl.textContent = s.siren ? '작동' : '정지';
  sirenEl.className = s.siren ? 'warn' : 'off';

  /* DEMO-0904 — 대피자 추종 상태를 미션 상태에서 유도한다 (i18n.js FOLLOWER_OF 주석 참조).
     /person_status 는 08-23 bag 에서 상수 스텁이라 못 쓴다. */
  const ps = byId('person-v');
  const [ftxt, fcls] = FOLLOWER_OF[s.mission] || ['—', 'off'];
  ps.textContent = ftxt;
  ps.className = fcls;

  /* ── 로봇 시야 ── 🎬 DEMO-0904 (값 출처 = js/demo.js)
     🔴 이 패널에는 **로봇이 감각한 것만** 둔다. 화재 신고 좌표(/alarm)는 관제·방재
        시스템이 알려준 값이라 로봇의 감각이 아니고, 임무 패널과 지도에 이미 있다. */
  const d = demoPerception(s);
  set('det-src', d.live ? '수신 중' : '미결합', d.live ? 'ok' : 'off');
  set('det-person', d.live ? `${d.person} 명` : '—', d.live ? (d.person ? 'warn' : 'ok') : 'off');
  set('det-fire',   d.live ? `${d.fire} 건`  : '—', d.live ? (d.fire ? 'alarm' : 'ok') : 'off');

  /* ── 버튼 가용성 ── */
  const on = s.connected;
  for (const id of ['btn-reset', 'btn-abort', 'btn-fire']) byId(id).disabled = !on;

  /* ── 로그 ── */
  renderLog(document.getElementById('log-body'), s.logs, 60);
}

/* 로봇 상태 요약 — 상단바 신호등의 내역 네 줄.
   진단 화면(16항목)까지 안 들어가도 "지금 로봇이 명령을 받을 수 있나"가 보여야 한다. */
function renderDiagMini(s) {
  const age = t => t ? (Date.now() - t) / 1000 : null;
  const freshCls = v => v == null ? 'off' : v < 1.5 ? 'ok' : v < 4 ? 'warn' : 'alarm';

  const a = age(s.fresh.scan);
  const rows = [
    ['drive', s.driveEnabled == null ? '—' : (s.driveEnabled ? '무장' : '무장 해제'),
              s.driveEnabled == null ? 'off' : (s.driveEnabled ? 'ok' : 'warn')],
    ['estop', s.estop == null ? '—' : (s.estop ? '눌림' : '해제'),
              s.estop == null ? 'off' : (s.estop ? 'alarm' : 'ok')],
    ['scan',  a == null ? '미수신' : `${a.toFixed(1)}초 전`, freshCls(a)],
    ['rtt',   s.rttMs == null ? '—' : `${s.rttMs} ms`,
              s.rttMs == null ? 'off' : s.rttMs < 100 ? 'ok' : s.rttMs < 500 ? 'warn' : 'alarm'],
  ];
  /* 같은 값이 평시엔 왼쪽 큰 패널(n-*), 대응 중엔 우측 요약(m-*)에 들어간다 */
  for (const [k, v, c] of rows) { set('n-' + k, v, c); set('m-' + k, v, c); }
}

function set(id, text, cls = '') {
  const e = byId(id);
  if (!e) return;
  e.textContent = text;
  e.className = cls;
}

/* 상태 전이 타임라인 — 최신이 위. 각 줄에 그 상태에 머문 시간을 붙인다.
   "지금 몇 단계째이고 각 단계가 얼마나 걸렸나"가 한눈에 보여야 한다. */
function renderTimeline(s) {
  const el = byId('timeline-body');
  if (!el) return;
  if (!s.history.length) { el.innerHTML = '<div class="empty">임무 시작 대기</div>'; return; }
  const rows = [];
  for (let i = s.history.length - 1; i >= 0; i--) {
    const h = s.history[i];
    const end = i + 1 < s.history.length ? s.history[i + 1].at : Date.now();
    const now = i === s.history.length - 1;
    rows.push(`<div class="tl${now ? ' now' : ''}">` +
                `<span class="t">${hms(h.at)}</span>` +
                `<span class="s">${STATE_KO[h.state] || h.state}</span>` +
                `<span class="d">${dur((end - h.at) / 1000)}</span>` +
              `</div>`);
    if (rows.length >= 14) break;
  }
  el.innerHTML = rows.join('');
}

/** 폴리라인 길이 (m) */
function pathLen(pts) {
  let d = 0;
  for (let i = 1; i < pts.length; i++) d += Math.hypot(pts[i][0] - pts[i-1][0], pts[i][1] - pts[i-1][1]);
  return d;
}

const byId = id => document.getElementById(id);
const txt = (id, v) => { const e = byId(id); if (e) e.textContent = v; };
const html = (id, v) => { const e = byId(id); if (e) e.innerHTML = v; };
