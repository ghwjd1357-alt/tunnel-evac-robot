/* ═══════════════════════════════════════════════════════════════════
   diag.js — 진단 화면 (2026-09-02)

   메인 화면의 신호등 하나를 누르면 여기로 온다.
   심사위원은 신호등만 보고, 운용자는 이 16개를 본다 — 같은 데이터의 두 깊이.

   🔴 여기 있는 값 대부분은 펌웨어가 08-23 실차에서 이미 뱉고 있던 것이다.
      화면이 없어서 그동안 터미널로만 볼 수 있었다.
   ═══════════════════════════════════════════════════════════════════ */

import { state, onChange } from './state.js';
import { NAV_KO, DRIVE_STATE_KO, DRIVE_REJECT_KO } from './i18n.js';

/* MOCK — 토픽 자체가 없는 항목. 실물이 생기면 이 블록만 지운다. */
const MOCK = {
  battery: { pct: 78, volt: 24.6 },        // /battery_state 미합의 (구동부팀)
  jetson:  { cpu: 41, gpu: 27, temp: 52 }, // 자원 발행 노드 없음
};

const AGE = (t) => t ? (Date.now() - t) / 1000 : null;

/** 신선도 등급: 🟢 <1.5s / 🟡 <4s / 🔴 그 이상·미수신 (0718 §5.5 규칙 유지) */
function freshCls(sec) { return sec == null ? 'off' : sec < 1.5 ? 'ok' : sec < 4 ? 'warn' : 'alarm'; }
const secTxt = sec => sec == null ? '미수신' : `${sec.toFixed(1)}초 전`;

export function setupDiag() { onChange(render); }

function render(s) {
  if (s.menu !== 'diag') return;      // 안 보는 화면은 그리지 않는다 (부하 절약)

  /* 상단 큰 타일 다섯 — 한눈에 봐야 하는 것 */
  const sa = AGE(s.fresh.scan);
  set('t-drive', s.driveEnabled == null ? '—' : (s.driveEnabled ? '무장' : '해제'),
                 s.driveEnabled == null ? 'off' : (s.driveEnabled ? 'ok' : 'warn'));
  set('t-estop', s.estop == null ? '—' : (s.estop ? '눌림' : '해제'),
                 s.estop == null ? 'off' : (s.estop ? 'alarm' : 'ok'));
  set('t-scan', sa == null ? '미수신' : `${sa.toFixed(1)}s`, freshCls(sa));
  set('t-rtt', s.rttMs == null ? '—' : `${s.rttMs} ms`,
               s.rttMs == null ? 'off' : s.rttMs < 100 ? 'ok' : s.rttMs < 500 ? 'warn' : 'alarm');
  set('t-nav', s.navStatus == null ? '—' : (NAV_KO[s.navStatus] || `상태 ${s.navStatus}`),
               s.navStatus == null ? 'off' : s.navStatus === 6 ? 'alarm'
               : (s.navStatus === 2 || s.navStatus === 4) ? 'ok' : 'warn');

  /* ── 구동부·안전 ──
     🔴 x y z 를 그대로 두면 아무도 못 읽는다. 계약(ino:1456 · rearm_gate.h:59)대로 해독한다. */
  const dd = s.driveDiag;
  const z = dd ? Math.round(dd.z) : null, y = dd ? Math.round(dd.y) : null;
  set('d-dstate', z == null ? '—' : (DRIVE_STATE_KO[z] || `알 수 없음 (${z})`),
                  z == null ? 'off' : z === 2 ? 'ok' : z === 0 ? 'warn' : '');
  set('d-dreject', y == null ? '—' : (DRIVE_REJECT_KO[y] || `알 수 없음 (${y})`),
                   y == null ? 'off' : y === 0 ? 'ok' : 'warn');
  set('d-dcount', dd ? `${Math.round(dd.x)} 회` : '—', dd ? '' : 'off');
  set('d-estop', s.estop == null ? '—' : (s.estop ? '눌림' : '해제'),
                 s.estop == null ? 'off' : (s.estop ? 'alarm' : 'ok'));
  set('d-diag', dd ? `${dd.x}  ${dd.y}  ${dd.z}` : '—', 'raw');
  set('d-diag-age', secTxt(AGE(s.fresh.drive)), freshCls(AGE(s.fresh.drive)));

  /* ── 펌웨어 ──
     /firmware/info 는 `키=값;` 이 40개 넘게 붙은 800자짜리 한 줄이다.
     통째로 넣으면 패널이 깨진다 → 화면엔 버전·빌드만, 전문은 title 로 남긴다. */
  const fw = parseFw(s.fwInfo);
  set('d-fw', fw.short, s.fwInfo ? '' : 'off');
  const fwEl = document.getElementById('d-fw');
  if (fwEl) fwEl.title = s.fwInfo || '';
  set('d-pulse', secTxt(AGE(s.fwPulse)), freshCls(AGE(s.fwPulse)));

  /* ── 센서 신선도 ── */
  for (const [id, key] of [['d-scan', 'scan'], ['d-odom', 'odom'], ['d-imu', 'imu'], ['d-tf', 'tf']]) {
    const a = AGE(s.fresh[key]);
    set(id, secTxt(a), freshCls(a));
  }
  set('d-yaw', s.imuYaw == null ? '—' : `${s.imuYaw.toFixed(1)}°`, s.imuYaw == null ? 'off' : '');

  /* ── 항법·통신 ── */
  set('d-nav', s.navStatus == null ? '—' : (NAV_KO[s.navStatus] || `상태 ${s.navStatus}`),
               s.navStatus == null ? 'off' : (s.navStatus === 6 ? 'alarm'
                                            : (s.navStatus === 2 || s.navStatus === 4) ? 'ok' : 'warn'));
  set('d-conn', s.connected ? '연결됨' : '끊김', s.connected ? 'ok' : 'alarm');
  set('d-rtt', s.rttMs == null ? '—' : `${s.rttMs} ms`,
               s.rttMs == null ? 'off' : s.rttMs < 100 ? 'ok' : s.rttMs < 500 ? 'warn' : 'alarm');

  /* ── 위치·주행 ── */
  set('d-pose', s.robot ? `${s.robot.x.toFixed(2)}  ${s.robot.y.toFixed(2)}  ${(s.robot.yaw * 180 / Math.PI).toFixed(0)}°` : '—',
                s.robot ? '' : 'off');
  set('d-speed', `${s.speed.toFixed(2)} m/s`);
  set('d-dist', `${s.distance.toFixed(1)} m`);

  /* ── 인지 어댑터 ── */
  set('d-adapter', s.adapter || '입력 없음', s.adapter ? '' : 'off');
  set('d-det', s.detLastMs ? `${secTxt(AGE(s.detLastMs))}` : '미결합 (역할 B)',
               s.detLastMs ? 'ok' : 'off');

  /* ── MOCK — 막대로 보여준다 (퍼센트는 숫자보다 막대가 빠르다) ── */
  meter('batt', MOCK.battery.pct, `${MOCK.battery.pct}%  ${MOCK.battery.volt} V`,
        p => p > 40 ? 'ok' : p > 20 ? 'warn' : 'alarm');
  meter('cpu',  MOCK.jetson.cpu,  `${MOCK.jetson.cpu}%`,  p => p < 70 ? 'ok' : p < 90 ? 'warn' : 'alarm');
  meter('gpu',  MOCK.jetson.gpu,  `${MOCK.jetson.gpu}%`,  p => p < 70 ? 'ok' : p < 90 ? 'warn' : 'alarm');
  meter('temp', MOCK.jetson.temp, `${MOCK.jetson.temp} °C`, p => p < 70 ? 'ok' : p < 85 ? 'warn' : 'alarm');
}

/** "version=1.6.3; build=Aug 22 2026 16:17:15; ..." → { short: "1.6.3 · Aug 22 2026" } */
function parseFw(raw) {
  if (!raw) return { short: '—' };
  const kv = {};
  for (const part of raw.split(';')) {
    const i = part.indexOf('=');
    if (i > 0) kv[part.slice(0, i).trim()] = part.slice(i + 1).trim();
  }
  const ver = (kv.version || '').replace(/^.*?([0-9]+\.[0-9]+\.[0-9]+).*$/, '$1') || kv.version || '?';
  const build = (kv.build || '').split(' ').slice(0, 3).join(' ');
  return { short: build ? `${ver} · ${build}` : ver };
}

/** 막대 하나: 값 글자 + 채움 폭 + 등급색 */
function meter(key, pct, text, grade) {
  const v = document.getElementById('mv-' + key);
  const b = document.getElementById('mb-' + key);
  if (v) v.textContent = text;
  if (b) {
    b.style.width = `${Math.max(0, Math.min(100, pct))}%`;
    b.className = grade(pct);
  }
}

function set(id, text, cls = '') {
  const el = document.getElementById(id);
  if (!el) return;
  el.textContent = text;
  el.className = cls;
}
