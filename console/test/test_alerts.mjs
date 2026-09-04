/* ═══════════════════════════════════════════════════════════════════
   test_alerts.mjs — 경보 규칙 10종 검증 (2026-09-02)

   🔴 왜 따로 필요한가: 본편 bag(realtake6)에는 FAULT·BLOCKED 가 **0건**이다.
      재생만으로는 경보 층이 한 번도 안 켜진다. 경보는 관제의 심장이므로
      상태를 직접 만들어 각 규칙이 켜지고 꺼지는지 확인한다.
   ═══════════════════════════════════════════════════════════════════ */

import { document, window, getComputedStyle } from './dom_stub.mjs';
globalThis.document = document; globalThis.window = window;
globalThis.getComputedStyle = getComputedStyle;

const { state, update } = await import('../js/state.js');
const { evaluateAlerts, healthSummary } = await import('../js/alert.js');

/* 아무 경보도 없는 평상시 상태 */
const CALM = {
  connected: true, everConnected: true, mission: 'GUIDE', estop: false, driveEnabled: true,
  navStatus: 2, fresh: { scan: Date.now(), tf: Date.now() }, alerts: [], logs: [],
};

const CASES = [
  ['평상시 — 경보 없음',            {},                                    [],            'ok',    '정상'],
  ['BLOCKED — 사람 개입 필요',      { mission: 'BLOCKED' },                ['blocked'],   'alarm', '이상'],
  ['FAULT — 주행 실패',             { mission: 'FAULT' },                  ['fault'],     'alarm', '이상'],
  ['RESCUE — 쓰러진 대피자',        { mission: 'RESCUE' },                 ['rescue'],    'alarm', '이상'],
  ['E-stop 눌림',                   { estop: true },                       ['estop'],     'alarm', '이상'],
  ['통신 두절 (붙었다가 끊김)',      { connected: false },                  ['link'],      'alarm', '이상'],
  /* 🔴 회귀 — 2026-09-03 브라우저 첫 확인에서 잡힌 오탐.
     기동 직후(한 번도 못 붙은 상태)를 '두절'로 판정해 페이지를 열 때마다 가짜 경보가 떴다.
     '아직 안 붙음'과 '붙었다가 끊김'은 다른 사건이다. */
  ['기동 직후 — 아직 연결 전',       { connected: false, everConnected: false }, [],       '',      '연결 대기'],
  ['구동부 무장 해제',              { driveEnabled: false },               ['disarmed'],  'warn',  '주의'],
  ['라이다 끊김',                   { fresh: { scan: Date.now() - 9000, tf: Date.now() } }, ['scan'], 'warn', '주의'],
  ['위치추정 끊김',                 { fresh: { scan: Date.now(), tf: Date.now() - 9000 } }, ['odom'], 'warn', '주의'],
  ['Nav2 목표 거부',                { navStatus: 6 },                      ['navrej'],    'warn',  '주의'],
  ['대피자 놓침 (SEARCH_BACK)',     { mission: 'SEARCH_BACK' },            ['lost'],      'warn',  '주의'],
  ['복합 — E-stop + 무장해제',      { estop: true, driveEnabled: false },  ['estop', 'disarmed'], 'alarm', '이상'],
];

let fail = 0;
console.log(`\n${'═'.repeat(74)}\n  경보 규칙 검증\n${'═'.repeat(74)}`);
console.log(`  ${'상황'.padEnd(30)} ${'기대'.padEnd(22)} ${'신호등'.padEnd(8)} 판정`);
console.log('  ' + '─'.repeat(70));

for (const [name, patch, expect, hCls, hText] of CASES) {
  update({ ...CALM, alerts: [], logs: [] });    // 매번 평상시로 되돌린 뒤
  update(patch);                                 // 조건 하나만 바꾼다
  evaluateAlerts();
  const got = state.alerts.map(a => a.id).sort();
  const want = [...expect].sort();
  const h = healthSummary(state);
  const ok = JSON.stringify(got) === JSON.stringify(want) && h.cls === hCls && h.text === hText;
  if (!ok) fail++;
  console.log(`  ${(ok ? '🟢' : '🔴')} ${name.padEnd(28)} ${(want.join(',') || '없음').padEnd(21)} ` +
              `${h.text.padEnd(7)} ${ok ? 'OK' : `실제=[${got.join(',')}] ${h.text}`}`);
}

/* 해소 시험: 켰다가 조건을 없애면 꺼져야 한다 (자동 해소) */
update({ ...CALM, alerts: [], logs: [] });
update({ mission: 'FAULT' }); evaluateAlerts();
const on = state.alerts.length;
update({ mission: 'PATROL' }); evaluateAlerts();
const off = state.alerts.length;
const clearOk = on === 1 && off === 0;
if (!clearOk) fail++;
console.log(`\n  ${clearOk ? '🟢' : '🔴'} 자동 해소 — FAULT 켜짐(${on}) → PATROL 꺼짐(${off})`);

/* 중복 방지: 같은 조건이 계속 참이어도 경보가 쌓이면 안 된다 */
update({ ...CALM, alerts: [], logs: [] });
update({ mission: 'BLOCKED' });
for (let i = 0; i < 20; i++) evaluateAlerts();
const dupOk = state.alerts.length === 1;
if (!dupOk) fail++;
console.log(`  ${dupOk ? '🟢' : '🔴'} 중복 방지 — 20회 평가 후 경보 ${state.alerts.length}건 (기대 1)`);

console.log(`\n${'═'.repeat(74)}`);
console.log(fail ? `🔴 실패 ${fail}건` : `🟢 전부 통과 (${CASES.length + 2} 케이스)`);
process.exit(fail ? 1 : 0);
