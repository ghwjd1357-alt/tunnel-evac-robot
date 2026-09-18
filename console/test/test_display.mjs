/* ═══════════════════════════════════════════════════════════════════
   test_display.mjs — 디스플레이 문구 우선순위 검증 (2026-09-18)

   🔴 왜 필요한가: 디스플레이는 켜두고 아무도 안 만진다. rosbridge 가 죽거나
      미션 노드가 멈춰도 마지막 "따라오세요"가 남아 있으면 대피자는 그 말을
      믿는다. 브라우저에서는 이 두 경우가 **아무 오류 없이** 그대로 보였다.
      (09-18 이전 renderDisplay 는 connected·fresh 를 보지 않았다)

   실행: node console/test/test_display.mjs
   ═══════════════════════════════════════════════════════════════════ */

const { displayText, SAY_HOLD_MS, MISSION_STALE_MS } = await import('../js/display.js');
const { DISPLAY_KO } = await import('../js/i18n.js');

const NOW = 1_700_000_000_000;
const LIVE = { connected: true, everConnected: true, mission: 'GUIDE',
               sayText: null, sayAt: null, fresh: { mission: NOW - 500 } };

/* [이름, 덮어쓸 상태, 기대 main, 기대 cls] */
const CASES = [
  ['정상 — 상태 문구',                {},                                          '따라오세요', ''],
  ['기동 직후 — 아직 한 번도 못 붙음', { connected: false, everConnected: false }, '연결 중',   'offline'],
  ['🔴 연결 끊김 — 마지막 문구를 지운다', { connected: false },                  '연결 끊김', 'offline'],
  ['🔴 미션 노드 침묵 — 상태가 낡음',   { fresh: { mission: NOW - MISSION_STALE_MS - 1 } }, '신호 없음', 'offline'],
  ['경계 — 정확히 임계까지는 살아 있음', { fresh: { mission: NOW - MISSION_STALE_MS } }, '따라오세요', ''],
  ['한 번도 안 온 상태 — 낡음이 아니라 대기', { mission: null, fresh: {} },       '연결 중',   'offline'],
  ['관제 문구 — 상태보다 우선',        { sayText: '이쪽으로 오세요', sayAt: NOW - 1000 }, '이쪽으로 오세요', 'said'],
  ['관제 문구 — 20초 지나면 상태로 복귀', { sayText: '이쪽으로 오세요', sayAt: NOW - SAY_HOLD_MS }, '따라오세요', ''],
  ['🔴 관제 문구가 있어도 끊기면 끊김이 이긴다', { connected: false, sayText: '이쪽으로', sayAt: NOW - 1000 }, '연결 끊김', 'offline'],
  ['🔴 관제 문구가 있어도 미션이 낡으면 문구는 유지(사람이 보낸 말)', { sayText: '대기하세요', sayAt: NOW - 1000, fresh: { mission: NOW - 60000 } }, '대기하세요', 'said'],
  ['모르는 상태값 — 빈 화면이 아니라 대기', { mission: 'WHATEVER' },             '연결 중',   'offline'],
  ['FAULT — 대피자에게는 "정지"',       { mission: 'FAULT' },                       '정지',      ''],
];

let fail = 0;
for (const [name, patch, main, cls] of CASES) {
  const r = displayText({ ...LIVE, ...patch }, NOW);
  const ok = r.main === main && r.cls === cls;
  if (!ok) fail++;
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}` + (ok ? '' : `  → got main="${r.main}" cls="${r.cls}"`));
}

/* 상태 12종 전부가 문구를 가진다 — 하나라도 빠지면 그 상태에서 "연결 중"이 뜬다 */
const STATES = ['PATROL','APPROACH','SCAN_AREA','GATHER','GUIDE','HOLD','SEARCH_BACK',
                'RESCUE','NO_VICTIM','ESCAPED','FAULT','BLOCKED'];
for (const st of STATES) {
  const r = displayText({ ...LIVE, mission: st }, NOW);
  const ok = DISPLAY_KO[st] && r.main === DISPLAY_KO[st][0] && r.cls === '';
  if (!ok) fail++;
  console.log(`${ok ? 'PASS' : 'FAIL'}  상태 문구 ${st} → ${r.main}`);
}

console.log(`\n${CASES.length + STATES.length - fail}/${CASES.length + STATES.length} 통과`);
process.exit(fail ? 1 : 0);
