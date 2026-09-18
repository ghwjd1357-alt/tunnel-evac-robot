/* ═══════════════════════════════════════════════════════════════════
   test_audio.mjs — 디스플레이 소리 결정 검증 (2026-09-18)

   🔴 소리는 화면보다 되돌리기 어렵다 — 잘못 나간 "따라오세요" 는 지울 수 없다.
      그래서 "언제 말하고 언제 침묵하는가" 를 브라우저 없이 잠근다.

   실행: node console/test/test_audio.mjs
   ═══════════════════════════════════════════════════════════════════ */
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const { decideAudio, REPEAT_MS, VOICE_STATES, voiceFile, SIREN_FILE } = await import('../js/audio.js');
const { MISSION_STALE_MS } = await import('../js/display.js');

const NOW = 1_700_000_000_000;
const LIVE = { connected: true, everConnected: true, mission: 'GUIDE', siren: false,
               sayText: null, sayAt: null, fresh: { mission: NOW - 500 } };
const FRESH_MEM = { lastState: null, lastVoiceAt: 0, lastSayAt: null };

let fail = 0;
const check = (name, got, want) => {
  const ok = JSON.stringify(got) === JSON.stringify(want);
  if (!ok) fail++;
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}` + (ok ? '' : `\n        got  ${JSON.stringify(got)}\n        want ${JSON.stringify(want)}`));
};
const pick = d => ({ voice: d.voice, siren: d.siren, say: d.say });

/* ① 전이 1회 */
let d = decideAudio(LIVE, FRESH_MEM, NOW);
check('처음 GUIDE 를 보면 1회 말한다', pick(d), { voice: 'GUIDE', siren: false, say: null });
let m = d.mem;
d = decideAudio(LIVE, m, NOW + 1000);
check('같은 상태 1초 뒤 — 다시 말하지 않는다', pick(d), { voice: null, siren: false, say: null });

/* ② 반복 */
/* 20초 뒤 시점이므로 미션 신선도도 같이 옮긴다 — 안 옮기면 '낡음' 가드가 먼저 침묵시킨다 */
const LATER = t => ({ ...LIVE, fresh: { mission: t - 500 } });
d = decideAudio(LATER(NOW + REPEAT_MS), m, NOW + REPEAT_MS);
check('GUIDE 는 REPEAT_MS 지나면 반복', pick(d), { voice: 'GUIDE', siren: false, say: null });
d = decideAudio(LATER(NOW + REPEAT_MS - 1), m, NOW + REPEAT_MS - 1);
check('경계 — REPEAT_MS 직전은 침묵', pick(d), { voice: null, siren: false, say: null });
d = decideAudio({ ...LIVE, mission: 'PATROL' }, { ...FRESH_MEM, lastState: 'PATROL', lastVoiceAt: NOW - 60000 }, NOW);
check('PATROL 은 반복하지 않는다 (1분 지나도)', pick(d), { voice: null, siren: false, say: null });

/* ④ 침묵 */
d = decideAudio({ ...LIVE, mission: 'FAULT' }, FRESH_MEM, NOW);
check('🔴 FAULT 는 말하지 않는다', pick(d), { voice: null, siren: false, say: null });
d = decideAudio({ ...LIVE, mission: 'BLOCKED', siren: true }, FRESH_MEM, NOW);
check('🔴 BLOCKED 는 말하지 않는다 — 싸이렌은 미션이 켠 대로', pick(d), { voice: null, siren: true, say: null });
d = decideAudio({ ...LIVE, connected: false, siren: true }, { ...FRESH_MEM, lastState: 'GUIDE' }, NOW);
check('🔴 연결 끊김 — 음성·싸이렌 모두 침묵', pick(d), { voice: null, siren: false, say: null });
check('    끊기면 상태 기억을 지운다 (복구 시 다시 1회 말하도록)', d.mem.lastState, null);
d = decideAudio({ ...LIVE, siren: true, fresh: { mission: NOW - MISSION_STALE_MS - 1 } }, { ...FRESH_MEM, lastState: 'GUIDE' }, NOW);
check('🔴 미션 낡음 — 싸이렌도 끈다 (미션 노드가 죽었는데 울리면 안 된다)', pick(d), { voice: null, siren: false, say: null });
d = decideAudio(LIVE, { ...FRESH_MEM, lastState: null }, NOW);
check('끊겼다 복구 — GUIDE 를 다시 1회 말한다', pick(d), { voice: 'GUIDE', siren: false, say: null });

/* ③ 싸이렌 */
d = decideAudio({ ...LIVE, mission: 'APPROACH', siren: true }, FRESH_MEM, NOW);
check('APPROACH + /siren true → 음성 1회 + 싸이렌', pick(d), { voice: 'APPROACH', siren: true, say: null });

/* ⑤ 관제 문구 */
d = decideAudio({ ...LIVE, sayText: '이쪽입니다', sayAt: NOW - 100 }, { ...FRESH_MEM, lastState: 'GUIDE', lastVoiceAt: NOW - 1000 }, NOW);
check('새 관제 문구 → TTS 1회', pick(d), { voice: null, siren: false, say: '이쪽입니다' });
d = decideAudio({ ...LIVE, sayText: '이쪽입니다', sayAt: NOW - 100 }, d.mem, NOW + 500);
check('같은 문구는 다시 읽지 않는다', pick(d), { voice: null, siren: false, say: null });

/* 음성 파일 존재 — 상태 목록과 파일이 갈라지면 그 상태에서 소리만 조용히 빠진다 */
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
for (const st of VOICE_STATES) {
  const ok = fs.existsSync(path.join(root, voiceFile(st)));
  if (!ok) fail++;
  console.log(`${ok ? 'PASS' : 'FAIL'}  음성 파일 ${voiceFile(st)}`);
}
{
  const ok = fs.existsSync(path.join(root, SIREN_FILE));
  if (!ok) fail++;
  console.log(`${ok ? 'PASS' : 'FAIL'}  싸이렌 파일 ${SIREN_FILE}`);
}

console.log(`\n${fail ? `FAIL ${fail}` : '전부 통과'}`);
process.exit(fail ? 1 : 0);
