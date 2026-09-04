/* ═══════════════════════════════════════════════════════════════════
   alert.js — 경보 판정 (2026-09-02)

   ── 왜 필요한가 ────────────────────────────────────────────────
   FAULT·BLOCKED·E-stop·통신두절이 로그 한 줄로 흘러가면 사람이 놓친다.
   감시 시스템인데 **사람을 부르는 층**이 없으면 감시가 성립하지 않는다.

   특히 `BLOCKED` 는 정본(FREEZE_MANIFEST)이 "탈출구 = 관제 reset 뿐"이라고
   적어둔 상태다. 화면이 조용하면 아무도 누르지 않는다.

   ── 판정 규칙 ──────────────────────────────────────────────────
   조건이 처음 참이 되는 순간 경보를 켜고, 거짓이 되면 끈다(자동 해소).
   같은 경보를 반복해서 쌓지 않는다.
   ═══════════════════════════════════════════════════════════════════ */

import { state, update } from './state.js';
import { pushLog } from './log.js';

const SEV = { ALARM: 'alarm', WARN: 'warn' };

/* 각 규칙: id · 등급 · 제목 · 이유 · 조건함수(s) → boolean */
const RULES = [
  { id: 'blocked',  sev: SEV.ALARM, what: '사람의 판단이 필요합니다',
    why: '안전한 집결지를 만들지 못했습니다. 관제에서 재시작(reset)해야 임무가 이어집니다.',
    when: s => s.mission === 'BLOCKED' },

  { id: 'fault',    sev: SEV.ALARM, what: '주행 실패 — 로봇 정지',
    why: 'Nav2 재시도가 소진되어 로봇이 멈췄습니다. 재시작하거나 원인을 확인하세요.',
    when: s => s.mission === 'FAULT' },

  { id: 'rescue',   sev: SEV.ALARM, what: '쓰러진 대피자 발견',
    why: '로봇이 그 자리에 정지했습니다. 구조 인력이 필요합니다.',
    when: s => s.mission === 'RESCUE' },

  { id: 'estop',    sev: SEV.ALARM, what: '비상정지 눌림',
    why: '하드웨어 비상정지가 활성입니다. 해제 전까지 구동부가 움직이지 않습니다.',
    when: s => s.estop === true },

  { id: 'link',     sev: SEV.ALARM, what: '로봇과 통신 끊김',
    why: 'rosbridge 연결이 끊어졌습니다. 화면의 값은 마지막 수신 시점의 것입니다.',
    /* 🔴 기동 직후(아직 한 번도 못 붙은 상태)는 '두절'이 아니라 '대기'다.
       그대로 두면 페이지를 열 때마다 가짜 경보가 한 번씩 뜬다. */
    when: s => s.connected === false && s.everConnected === true },

  { id: 'disarmed', sev: SEV.WARN,  what: '구동부 무장 해제',
    why: '명령을 보내도 로봇이 움직이지 않습니다.',
    when: s => s.driveEnabled === false },

  { id: 'scan',     sev: SEV.WARN,  what: '라이다 신호 끊김',
    why: '추종 판정과 장애물 회피가 정지합니다.',
    when: s => stale(s, 'scan', 4000) },

  { id: 'odom',     sev: SEV.WARN,  what: '로봇 위치를 알 수 없음',
    why: '지도 위 로봇 위치가 멈춰 있습니다.',
    when: s => stale(s, 'tf', 4000) },

  { id: 'navrej',   sev: SEV.WARN,  what: 'Nav2 목표 거부',
    why: '경로를 만들지 못했습니다. 반복되면 FAULT 로 넘어갑니다.',
    when: s => s.navStatus === 6 },

  { id: 'lost',     sev: SEV.WARN,  what: '대피자를 놓쳤습니다',
    why: '로봇이 되돌아가며 재탐색 중입니다.',
    when: s => s.mission === 'SEARCH_BACK' },
];

/** 마지막 수신이 ms 보다 오래됐나 (한 번도 안 왔으면 경보 대상 아님 — 미결합과 고장은 다르다) */
function stale(s, key, ms) {
  const t = s.fresh[key];
  return t ? (Date.now() - t > ms) : false;
}

export function evaluateAlerts() {
  const now = Date.now();
  const cur = new Map(state.alerts.map(a => [a.id, a]));
  const next = [];
  let changed = false;

  for (const r of RULES) {
    let on = false;
    try { on = !!r.when(state); } catch { on = false; }
    const had = cur.get(r.id);
    if (on) {
      if (had) next.push(had);
      else {
        next.push({ id: r.id, sev: r.sev, what: r.what, why: r.why, when: now });
        pushLog('ALERT', r.what, '', r.sev);
        changed = true;
      }
    } else if (had) {
      pushLog('ALERT', `해소 — ${r.what}`, '', 'state');
      changed = true;
    }
  }
  next.sort((a, b) => (a.sev === b.sev ? b.when - a.when : a.sev === 'alarm' ? -1 : 1));
  if (changed || next.length !== state.alerts.length) update({ alerts: next });
}

/** 신호등 하나로 요약 — 진단 16항목을 심사위원이 읽을 수 있는 한 덩어리로 */
export function healthSummary(s) {
  if (s.alerts.some(a => a.sev === 'alarm')) return { cls: 'alarm', text: '이상' };
  if (s.alerts.length)                        return { cls: 'warn',  text: '주의' };
  if (!s.connected)                           return { cls: '',      text: '연결 대기' };
  return { cls: 'ok', text: '정상' };
}
