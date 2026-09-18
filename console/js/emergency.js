/* ═══════════════════════════════════════════════════════════════════
   emergency.js — 비상 원격 조종 (2026-09-02)

   ── 왜 별도 화면인가 ────────────────────────────────────────────
   평상시에 눌리면 안 되는 버튼이다. 메인 화면에 두면 오조작 위험이 있고,
   메뉴를 한 번 거치는 것 자체가 방어막이 된다.

   ── 4중 안전장치 (사용자 결정 2026-09-02) ──────────────────────
   "비상시에만 쓴다"는 사람의 약속이라 코드가 지켜주지 않는다. 그래서:
     ① abort 로 임무를 멈춘 뒤에만 활성화된다
     ② 버튼을 누르고 있는 동안만 움직인다 (떼면 즉시 0 발행)
     ③ 통신이 끊기면 자동 정지 — Teensy 0.5초 워치독이 이미 담당
     ④ 속도 상한을 자율주행보다 낮게 고정

   🔴 MAX_* 를 올리지 않는다 — ELECTRICAL_BASELINE 전제 재작성(예약 63)이 선행한다.
   ═══════════════════════════════════════════════════════════════════ */

import { state, onChange } from './state.js';
import { STATE_KO } from './i18n.js';
import { sendCmd, sendTwist } from './ros.js';
import { pushLog } from './log.js';

const MAX_LIN = 0.10;   // m/s — 자율주행(0.20)의 절반
const MAX_ANG = 0.35;   // rad/s
const HZ = 10;

let armed = false;      // ①을 만족해 조종이 열린 상태인가
let held = null;        // 지금 누르고 있는 방향
let timer = null;

const DIRS = {
  fwd:   [ MAX_LIN,  0       ],
  back:  [-MAX_LIN,  0       ],
  left:  [ 0,        MAX_ANG ],
  right: [ 0,       -MAX_ANG ],
};

export function setupEmergency() {
  document.getElementById('emg-abort').onclick = () => {
    sendCmd('abort');
    pushLog('EMG', '원격 조종 준비 — 임무 중단(abort) 발행', '', 'ctrl');
  };

  for (const btn of document.querySelectorAll('#dpad button[data-dir]')) {
    const dir = btn.dataset.dir;
    const press = ev => { ev.preventDefault(); if (!armed) return; hold(dir, btn); };
    const release = () => letGo(btn);
    btn.addEventListener('mousedown', press);
    btn.addEventListener('touchstart', press, { passive: false });
    ['mouseup', 'mouseleave', 'touchend', 'touchcancel'].forEach(e => btn.addEventListener(e, release));
  }
  /* 창을 벗어나거나 탭이 가려지면 즉시 놓은 것으로 본다 */
  window.addEventListener('blur', () => letGo());
  document.addEventListener('visibilitychange', () => { if (document.hidden) letGo(); });

  onChange(render);
}

function hold(dir, btn) {
  held = dir;
  btn?.classList.add('pressed');
  if (timer) clearInterval(timer);
  const [lin, ang] = DIRS[dir];
  sendTwist(lin, ang);
  timer = setInterval(() => {                    // ② 누르는 동안만 계속 발행
    if (!held) return letGo();
    sendTwist(lin, ang);
  }, Math.round(1000 / HZ));
  pushLog('EMG', `원격 조종 ${dir}`, `${lin.toFixed(2)} ${ang.toFixed(2)}`, 'ctrl');
}

function letGo(btn) {
  if (timer) { clearInterval(timer); timer = null; }
  document.querySelectorAll('#dpad button').forEach(b => b.classList.remove('pressed'));
  if (held) { held = null; sendTwist(0, 0); sendTwist(0, 0); }   // 정지는 두 번 쏜다
}

function set(id, text, cls = '') {
  const el = document.getElementById(id);
  if (!el) return;
  el.textContent = text;
  el.className = cls;
}

function render(s) {
  /* ① 게이트: 연결됨 + 임무가 멈춘 상태(FAULT/BLOCKED) + 비상정지 해제 */
  const stopped = s.mission === 'FAULT' || s.mission === 'BLOCKED';
  const conds = [
    ['c-link',  s.connected === true,  '관제와 로봇이 연결되어 있다'],
    ['c-abort', stopped,               '임무가 중단되어 있다 (FAULT · BLOCKED)'],
    ['c-estop', s.estop === false,     '하드웨어 비상정지가 해제되어 있다'],
    ['c-drive', s.driveEnabled === true, '구동부가 무장되어 있다'],
  ];
  armed = conds.every(([, ok]) => ok);

  for (const [id, ok] of conds) {
    const el = document.getElementById(id);
    if (!el) continue;
    el.parentElement?.classList.toggle('met', ok);   // 구조가 바뀌어도 렌더가 죽지 않게
    el.className = 'dot ' + (ok ? 'ok' : '');
  }
  if (s.menu !== 'emergency') return;

  document.querySelectorAll('#dpad button[data-dir]').forEach(b => { b.disabled = !armed; });
  const st = document.getElementById('emg-state');
  st.textContent = armed ? '조종 가능' : '잠김 — 아래 조건을 모두 만족해야 열립니다';
  st.className = armed ? 'ok' : 'off';

  document.getElementById('emg-limit').textContent =
    `${MAX_LIN.toFixed(2)} m/s · ${MAX_ANG.toFixed(2)} rad/s`;
  set('emg-speed', `${s.speed.toFixed(2)} m/s`);

  /* 조종 판단에 필요한 로봇 상태 — 여기서 화면을 안 떠나도 보이게 */
  set('e-mission', s.mission ? (STATE_KO[s.mission] || s.mission) : '—', s.mission ? '' : 'off');
  set('e-drive', s.driveEnabled == null ? '—' : (s.driveEnabled ? '무장' : '무장 해제'),
                 s.driveEnabled == null ? 'off' : (s.driveEnabled ? 'ok' : 'warn'));
  set('e-estop', s.estop == null ? '—' : (s.estop ? '눌림' : '해제'),
                 s.estop == null ? 'off' : (s.estop ? 'alarm' : 'ok'));
  set('e-rtt', s.rttMs == null ? '—' : `${s.rttMs} ms`,
               s.rttMs == null ? 'off' : s.rttMs < 100 ? 'ok' : s.rttMs < 500 ? 'warn' : 'alarm');
  set('e-pose', s.robot ? `${s.robot.x.toFixed(2)}  ${s.robot.y.toFixed(2)}` : '—',
                s.robot ? '' : 'off');
}
