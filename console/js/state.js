/* ═══════════════════════════════════════════════════════════════════
   state.js — 화면에 보이는 모든 값을 한 곳에 모은다 (2026-09-02)

   ── 왜 이렇게 하나 ──────────────────────────────────────────────
   항목이 70개가 넘어가면 "토픽 콜백에서 곧바로 DOM 을 고치는" 방식은
   무너진다. 같은 값을 쓰는 자리가 여러 곳이면 한 군데를 빠뜨리기 때문.

   그래서 규칙을 하나 둔다:
     ① 토픽 콜백은 update() 로 **값만** 바꾼다. DOM 을 만지지 않는다.
     ② 화면 그리는 쪽은 onChange() 로 등록해두고 값이 바뀌면 자기 자리를 그린다.

   React 가 해주는 일의 핵심이 이 15줄이다. 빌드 도구 없이 같은 효과를 낸다.
   ═══════════════════════════════════════════════════════════════════ */

export const state = {
  /* ── 연결 ── */
  connected: false,
  everConnected: false,   // 한 번이라도 붙은 적 있나 (기동 직후 '두절' 오탐 방지)
  rttMs: null,

  /* ── 미션 ── */
  mission: null,          // 'PATROL' … 'BLOCKED' (12종)
  missionSince: null,     // 현재 상태 진입 시각 (ms)
  missionT0: null,        // 관제 가동 시각 (ms) — 첫 상태 수신 시점
  /* 🔴 구간 기준점 (2026-09-04).
     순찰은 무한 루프라 거기서부터 잰 시간·거리는 의미가 없다.
     대응 구간은 **화재 감지(/alarm) 시점**부터, 순찰 구간은 순찰 진입 시점부터 잰다. */
  phaseT0: null,          // 현재 구간 시작 시각
  phaseDist0: 0,          // 현재 구간 시작 시점의 누적 이동거리
  history: [],            // [{state, at}] 상태 전이 이력 (타임라인·요약이 쓴다)
  siren: false,
  personStatus: null,     // 'ok' | 'fallen' | 'unknown' | 'stale'
  victim: null,           // {x, y}

  /* ── 공간 ── */
  mapInfo: null,          // OccupancyGrid.info
  robot: null,            // {x, y, yaw} — map 기준
  planPts: [],            // 전역 경로
  fireXY: null,           // 화재 좌표
  trail: [],              // 지나온 길 (map 좌표 배열)
  scanPts: [],            // 라이다 점군 (map 좌표로 옮긴 것)
  distance: 0,            // 누적 이동거리 (m)
  speed: 0,               // 현재 속도 (m/s)

  /* ── 하드웨어 ── */
  driveEnabled: null,     // /drive/enabled
  driveDiag: null,        // /drive/diag {x,y,z}
  estop: null,            // /estop/state — true = 눌림
  fwInfo: null,           // /firmware/info
  fwPulse: null,          // 마지막 하트비트 시각 (ms)
  imuYaw: null,

  /* ── Nav2 ── */
  navStatus: null,        // 1 접수 / 2 주행 / 4 도달 / 6 거부

  /* ── 신선도: {토픽id: 마지막 수신 ms} ── */
  fresh: {},

  /* ── 인지 ── */
  detections: [],         // [{cls, conf}]
  detLastMs: 0,
  adapter: null,

  /* 관제 → 로봇 디스플레이 문구 (/display_msg). 디스플레이 모드가 이걸 크게 띄운다 */
  sayText: null,
  sayAt: null,

  /* ── 경보·로그 ── */
  alerts: [],             // [{id, sev, what, why, when}]
  logs: [],               // [{t, tag, msg, val, cls}]

  /* ── 화면 ── */
  menu: 'main',
};

const listeners = [];

/** 값이 바뀌면 알려달라고 등록한다. 등록 즉시 한 번 불러 초기 화면을 그린다. */
export function onChange(fn) {
  listeners.push(fn);
  try { fn(state); } catch (e) { console.error(e); }
}

/** 값을 바꾸는 유일한 통로. 바꾼 뒤 등록된 모두에게 알린다. */
export function update(patch) {
  Object.assign(state, patch);
  for (const fn of listeners) {
    try { fn(state); } catch (e) { console.error(e); }
  }
}

/* 고빈도 토픽(/tf·/scan 는 초당 수십 번)까지 매번 전체 갱신을 돌리면 낭비다.
   → 값만 조용히 바꾸고, 화면 갱신은 아래 tick() 이 일정 주기로 몰아서 한다. */
export function quiet(patch) { Object.assign(state, patch); }

/** 주기 갱신 시작 (기본 5fps — 관제 화면은 이걸로 충분하다) */
export function startTicker(hz = 5) {
  setInterval(() => { for (const fn of listeners) { try { fn(state); } catch (e) { console.error(e); } } },
              Math.round(1000 / hz));
}
