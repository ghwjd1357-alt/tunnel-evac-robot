/**
 * audio.js — 로봇 몸통 디스플레이의 소리 (2026-09-18 신설)
 *
 * 패널 PCB 에 앰프 + 스피커(4Ω 2W)가 있고 젯슨 DP 가 영상과 소리를 같이 실어 보낸다.
 * 브라우저가 소리를 내면 그대로 패널 스피커로 나간다 — 그래서 이 층도 화면과 같은
 * 자리(관제 콘솔 ?display=1)에 산다.
 *
 * 무엇을 언제 내는가 (decideAudio — 순수 함수, test/test_audio.mjs 가 잠근다):
 *   ① 상태가 바뀐 순간 그 상태의 안내 음성 1회        (media/voice/<STATE>.mp3)
 *   ② GUIDE·GATHER 는 REPEAT_MS 마다 반복              — 5분 유도 중 한 번만 말하면 못 듣는다
 *   ③ /siren true 인 동안 싸이렌 반복                  (media/siren.mp3 · mission_node 가 켜고 끈다)
 *   ④ 🔴 연결 끊김·미션 낡음·FAULT·BLOCKED = 침묵      — 틀린 안내가 나가는 것보다 침묵이 안전하다
 *   ⑤ 관제가 보낸 문구는 브라우저 내장 TTS(있으면)     — 젯슨에 한국어 음성이 없으면 조용히 건너뛴다
 *
 * 🔴 크로미움은 사용자 클릭 없이 소리를 막는다 → run_display.sh 가
 *    --autoplay-policy=no-user-gesture-required 로 띄운다. 그 인자 없이 열면 소리만 없다.
 */
import { onChange } from './state.js';
import { displayText } from './display.js';

export const REPEAT_MS = 20000;
export const REPEAT_STATES = new Set(['GUIDE', 'GATHER']);
/* 음성이 있는 상태. FAULT·BLOCKED 는 일부러 없다 — 화면엔 "정지" 가 뜨지만 소리로
   "정지" 를 외치면 대피자가 멈춰 선다. 그 상황의 지시는 관제(사람)가 한다. */
export const VOICE_STATES = ['PATROL', 'APPROACH', 'SCAN_AREA', 'GATHER', 'GUIDE', 'HOLD',
                             'SEARCH_BACK', 'RESCUE', 'NO_VICTIM', 'ESCAPED'];
export const voiceFile = st => `media/voice/${st}.mp3`;
export const SIREN_FILE = 'media/siren.mp3';

/**
 * 지금 무엇을 재생해야 하나.
 * @param {object} s      state
 * @param {object} mem    { lastState, lastVoiceAt, lastSayAt } — 호출자가 보관하는 기억
 * @param {number} now
 * @returns {{ voice: string|null, siren: boolean, say: string|null, mem: object }}
 *   voice = 이번 tick 에 새로 틀 상태 이름 (없으면 null) · siren = 싸이렌이 울려야 하는가
 *   say   = TTS 로 읽을 관제 문구 (없으면 null)
 */
export function decideAudio(s, mem, now = Date.now()) {
  const { cls } = displayText(s, now);
  const next = { ...mem };

  /* ④ 화면이 offline 이면 소리도 없다. 상태 기억은 지워서 복구 순간 다시 1회 말한다 */
  if (cls === 'offline') {
    next.lastState = null; next.lastVoiceAt = 0;
    return { voice: null, siren: false, say: null, mem: next };
  }

  let voice = null;
  const st = s.mission;
  if (VOICE_STATES.includes(st)) {
    if (st !== mem.lastState) {                                   // ① 전이 1회
      voice = st; next.lastVoiceAt = now;
    } else if (REPEAT_STATES.has(st) && now - mem.lastVoiceAt >= REPEAT_MS) {   // ② 반복
      voice = st; next.lastVoiceAt = now;
    }
  }
  next.lastState = st;

  /* ⑤ 관제 문구 — 새로 온 것만 한 번 */
  let say = null;
  if (s.sayText && s.sayAt && s.sayAt !== mem.lastSayAt) {
    say = s.sayText; next.lastSayAt = s.sayAt;
  }

  return { voice, siren: !!s.siren, say, mem: next };
}

/* ── DOM 쪽 — 위 결정을 실제 소리로 ─────────────────────────────── */
let mem = { lastState: null, lastVoiceAt: 0, lastSayAt: null };
let sirenEl = null;
let voiceEl = null;

function play(el, src) {
  try {
    if (el.getAttribute('src') !== src) el.setAttribute('src', src);
    el.currentTime = 0;
    const p = el.play();
    if (p && p.catch) p.catch(() => { /* 자동재생 차단 — run_display.sh 인자 없이 열었을 때 */ });
  } catch { /* 파일 없음 등 — 소리만 빠진다 */ }
}

function speak(text) {
  const ss = window.speechSynthesis;
  if (!ss) return;
  const ko = (ss.getVoices() || []).find(v => (v.lang || '').toLowerCase().startsWith('ko'));
  if (!ko) return;                       // 한국어 음성이 없는 젯슨 — 조용히 건너뛴다
  const u = new SpeechSynthesisUtterance(text);
  u.voice = ko; u.lang = 'ko-KR'; u.rate = 0.95;
  ss.cancel(); ss.speak(u);
}

export function setupAudio() {
  if (!document.body?.classList?.contains('display')) return;
  sirenEl = new Audio(); sirenEl.loop = true; sirenEl.volume = 0.6;
  voiceEl = new Audio(); voiceEl.volume = 1.0;

  onChange(s => {
    const d = decideAudio(s, mem);
    mem = d.mem;
    if (d.voice) play(voiceEl, voiceFile(d.voice));
    if (d.siren) { if (sirenEl.paused) play(sirenEl, SIREN_FILE); }
    else if (!sirenEl.paused) sirenEl.pause();
    if (d.say) speak(d.say);
  });
}
