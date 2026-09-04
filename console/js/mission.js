/* ═══════════════════════════════════════════════════════════════════
   mission.js — 관제 메인 화면 (2026-09-02)
     ① 진행 막대   ② 로봇 시야(인지)   ③ 제어 버튼
   지도는 map.js 가 그린다.
   ═══════════════════════════════════════════════════════════════════ */

import { state, onChange } from './state.js';
import { STAGES, EXCEPTION_STATES, STATE_KO, PERSON_KO, NAV_KO,
         MODE_OF, ACTION_HINT, dur, clock, hms } from './i18n.js';
import { sendCmd, sendAlarm, softStop } from './ros.js';
import { renderLog } from './log.js';

/* MOCK — 역할 B 결합 전까지 인지 패널을 채우는 시연용 값.
   🔴 실제 /detections 가 한 번이라도 오면 그 즉시 무시된다(아래 참조).
      영상·문서에서 이 값을 "실측"으로 소개하지 않는다. */
const MOCK_DETECTIONS = [{ cls: 'person_ok', conf: 0.91 }, { cls: 'fire', conf: 0.87 }];
export const mock = { vision: true };

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

  const ps = document.getElementById('person-v');
  ps.textContent = s.personStatus ? (PERSON_KO[s.personStatus] || s.personStatus) : '—';
  ps.className = s.personStatus === 'fallen' ? 'alarm'
               : s.personStatus === 'ok' ? 'ok' : 'off';

  /* ── 로봇 시야 (인지) ──
     실제 /detections 가 오면 그것을, 없으면 MOCK 을 보여준다.
     "수신 없음"과 "탐지 0건"은 다른 것이므로 아래 라벨에서 구분한다. */
  const live = s.detLastMs > 0;
  const list = live ? s.detections : (mock.vision ? MOCK_DETECTIONS : []);
  const person = list.filter(d => d.cls.startsWith('person')).length;
  const fire   = list.filter(d => d.cls === 'fire' || d.cls === 'smoke').length;
  txt('det-person', String(person));
  txt('det-fire', String(fire));
  const src = document.getElementById('det-src');
  src.textContent = live ? '카메라 수신 중' : (mock.vision ? '데모 데이터' : '수신 없음');
  src.className = live ? 'ok' : 'off';

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
