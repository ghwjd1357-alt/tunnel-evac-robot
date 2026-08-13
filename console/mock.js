/* ════════════════════════════════════════════════════════════════
   console/mock.js — L1 목업 모드  (계획서 §4)

   목적 : rosbridge·Gazebo·로봇 없이 브라우저만으로 콘솔 화면을 띄운다.
   원리 : ?mock=1 이면 ROSLIB 라이브러리를 "가짜"로 통째로 바꿔치기한다.
          → index.html 본문 로직(구독·콜백·렌더)은 한 줄도 안 고친다.

   🔴 지도는 합성이 아니다 — maps/twin_map_loc.pgm (실제 SLAM 저장본) 을
      mock_map_twin.js 픽스처로 변환해 그대로 쓴다. 원점·해상도도 yaml 원본값.

   🔴 주행 waypoint 는 목업 전용 임시값이다 (아래 §3 주석).
      쌍굴용 waypoints.yaml 정본이 오면 그 숫자로 교체할 것.

   사용 : console/ 에서  python3 -m http.server 8000
          http://localhost:8000/?mock=1
          ?speed=0.26 → 실제 순항속도로 재생 (기본은 시험 편의상 빠르게)
   ════════════════════════════════════════════════════════════════ */
(function () {
'use strict';

const params = new URLSearchParams(location.search);
if (!params.has('mock')) return;          // 실서비스 경로는 절대 건드리지 않는다

const MAPDEF = window.MOCK_MAP_TWIN;
if (!MAPDEF) {
  console.error('[MOCK] mock_map_twin.js 가 없다. index.html 에서 mock.js 보다 먼저 불러야 한다.');
  return;
}
console.log('[MOCK] 목업 모드 —', MAPDEF.name,
            `${MAPDEF.width}×${MAPDEF.height} @${MAPDEF.resolution}m`, 'origin', MAPDEF.origin);

/* ───────────────────────────────────────────────────────────────
   1. 가짜 ROSLIB — 구독자 명단을 들고 있다가 엔진이 부르면 콜백을 때린다
   ─────────────────────────────────────────────────────────────── */
const subs = {};                                   // { 토픽명: [콜백, …] }
function emit(topic, msg) {
  (subs[topic] || []).forEach(cb => { try { cb(msg); } catch (e) { console.error(e); } });
}

function MockRos() {
  this._h = {};
  // 접속 성공을 흉내 → index.html 의 setupTopics() 가 불려 구독이 등록된다.
  // ★ 엔진은 그 "다음"에 시작해야 한다. 먼저 발행하면 /map 첫 장을 아무도 못 받는다.
  setTimeout(() => { this._fire('connection'); startEngineOnce(); }, 150);
}
MockRos.prototype.on    = function (ev, cb) { (this._h[ev] = this._h[ev] || []).push(cb); };
MockRos.prototype._fire = function (ev, a)  { (this._h[ev] || []).forEach(cb => cb(a)); };
MockRos.prototype.close = function ()       { this._fire('close'); };

function MockTopic(o) { this.name = o.name; }
MockTopic.prototype.subscribe   = function (cb) { (subs[this.name] = subs[this.name] || []).push(cb); };
MockTopic.prototype.unsubscribe = function ()   { delete subs[this.name]; };
MockTopic.prototype.publish     = function (m)  { onPublish(this.name, m); };

function MockService(o) { this.name = o.name; }
MockService.prototype.callService = function (req, ok, fail) {
  if (this.name === '/rosapi/get_time') {          // RTT 측정용 — 8~20ms 흉내
    setTimeout(() => ok({ time: { secs: Math.floor(Date.now() / 1000), nsecs: 0 } }), 8 + Math.random() * 12);
  } else if (this.name === '/slam_toolbox/dynamic_map') {
    setTimeout(() => ok({ map: buildMap() }), 50);
  } else if (fail) { setTimeout(() => fail('mock: 지원하지 않는 서비스'), 50); }
};

function MockMessage(o) { Object.assign(this, o); }

window.ROSLIB = {
  Ros: MockRos, Topic: MockTopic, Service: MockService,
  Message: MockMessage, ServiceRequest: MockMessage,
};

/* ───────────────────────────────────────────────────────────────
   2. 지도 (/map) — 실제 저장 지도를 그대로 발행
   ─────────────────────────────────────────────────────────────── */
let mapCache = null;
function buildMap() {
  if (!mapCache) {
    mapCache = {
      header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
      info: {
        width: MAPDEF.width, height: MAPDEF.height, resolution: MAPDEF.resolution,
        origin: { position: { x: MAPDEF.origin.x, y: MAPDEF.origin.y, z: 0 },
                  orientation: { x: 0, y: 0, z: 0, w: 1 } },
      },
      data: MAPDEF.decode(),
    };
  }
  return mapCache;
}

/* ───────────────────────────────────────────────────────────────
   3. 쌍굴 지형 — 지도에서 실측한 값
        남측 터널 자유폭 y -2.83 ~  2.88  (중심 ≈ 0.0)
        북측 터널 자유폭 y  7.17 ~ 12.88  (중심 ≈ 10.0)
        연결통로 3개    x = 7 / 17 / 27, 폭 2.2m
        x 범위          -2.8 ~ 36.9
      ⚠ y=0 선에는 미탐사(205) 점선이 깔려 있어 주행선은 y=0.6 으로 잡았다.

   🔴 아래 waypoint 는 목업 전용 임시값이다. 쌍굴용 정본이 없어서
      다음 근거로 직접 세웠다:
        · 화재 데모 좌표 (30,0)          — 0718_관제시스템.md §4
        · 집결지 = 화재에서 8m           — waypoints.yaml gather_dist: 8.0
        · 역행 지점 ≥ 화재에서 5m        — waypoints.yaml search_back.min_fire_dist
        · 탈출구 = 서쪽 입구             — waypoints.yaml escape
      정본이 오면 N/GOAL_NODE/PATROL_LOOP 숫자만 갈아끼우면 된다.
   ─────────────────────────────────────────────────────────────── */
const SY = 0.6, NY = 10.0;

const N = {
  SW: { x: -1.5, y: SY }, S7:  { x: 7,    y: SY }, S17: { x: 17, y: SY },
  S22:{ x: 22,   y: SY }, S24: { x: 24,   y: SY }, S27: { x: 27, y: SY },
  SE: { x: 35.5, y: SY },
  NW: { x: -1.5, y: NY }, N7:  { x: 7,    y: NY }, N17: { x: 17, y: NY },
  N27:{ x: 27,   y: NY }, NE:  { x: 35.5, y: NY },
};
const EDGES = [
  ['SW','S7'], ['S7','S17'], ['S17','S22'], ['S22','S24'], ['S24','S27'], ['S27','SE'],
  ['NW','N7'], ['N7','N17'], ['N17','N27'], ['N27','NE'],
  ['S7','N7'], ['S17','N17'], ['S27','N27'],          // 연결통로
];
const ADJ = {};
for (const [a, b] of EDGES) { (ADJ[a] = ADJ[a] || []).push(b); (ADJ[b] = ADJ[b] || []).push(a); }

const FIRE_DEMO = { x: 30, y: 0 };
const GOAL_NODE = {
  APPROACH:    'S22',    // 집결지 = 화재(x30)에서 탈출구 방향 8m
  GUIDE:       'SW',     // 탈출구 = 서쪽 입구
  SEARCH_BACK: 'S24',    // 역행 지점 = 화재에서 6m (min_fire_dist 5.0 준수)
};
const PATROL_LOOP = ['SE', 'NE', 'NW', 'SW'];   // 쌍굴 양쪽 보어를 도는 순찰

const dist = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);
const nearestNode = p => Object.keys(N).reduce((m, k) => (dist(N[k], p) < dist(N[m], p) ? k : m), 'SW');

// 다익스트라 — 격벽을 뚫지 않고 연결통로로 돌아가는 경로가 나온다
function routeNodes(from, to) {
  const D = { [from]: 0 }, prev = {}, seen = {};
  for (;;) {
    let u = null;
    for (const k in D) if (!seen[k] && (u === null || D[k] < D[u])) u = k;
    if (u === null || u === to) break;
    seen[u] = 1;
    for (const v of ADJ[u]) {
      const nd = D[u] + dist(N[u], N[v]);
      if (D[v] === undefined || nd < D[v]) { D[v] = nd; prev[v] = u; }
    }
  }
  const out = []; let c = to;
  while (c) { out.unshift(c); c = prev[c]; }
  return out;
}

function pathTo(goalKey) {
  const pts = routeNodes(nearestNode(S.robot), goalKey).map(k => ({ x: N[k].x, y: N[k].y }));
  // 첫 노드가 뒤쪽이면 버린다 (로봇이 노드 사이에 있을 때 되돌아가는 것 방지)
  if (pts.length > 1 && dist(S.robot, pts[1]) < dist(pts[0], pts[1])) pts.shift();
  return pts;
}

/* ───────────────────────────────────────────────────────────────
   4. 시나리오 엔진
   ─────────────────────────────────────────────────────────────── */
const SPEED = parseFloat(params.get('speed')) || 2.0;   // 기본은 시험 편의상 빠르게
                                                        // 실제값: normal 0.26 / guide 0.12 (waypoints.yaml)
const S = {
  state: 'PATROL',
  robot: { x: 2.0, y: SY, yaw: 0 },
  path: [], goalKey: null,
  patrolIdx: 0,
  tGather: 0,
  moving: true,        // 패널 토글
  planOn: true,        // 패널 토글 (T1 시험용)
  lastArriveAt: 0,
};

const SIREN_ON = ['APPROACH', 'GATHER', 'GUIDE', 'SEARCH_BACK'];

function setGoal(key) {
  S.goalKey = key;
  S.path = key ? pathTo(key) : [];
}

function setState(s) {
  if (S.state === s) return;
  S.state = s;
  if (s === 'PATROL')          { S.patrolIdx = 0; setGoal(PATROL_LOOP[0]); }
  else if (GOAL_NODE[s])         setGoal(GOAL_NODE[s]);
  else                           setGoal(null);          // GATHER · ESCAPED · FAULT
  if (s === 'GATHER') S.tGather = Date.now();
  syncPanel();
}

const DT = 0.1;                                     // 10Hz
function stepRobot() {
  if (!S.path.length || !S.moving) return;
  const speed = (S.state === 'GUIDE') ? SPEED * 0.45 : SPEED;   // GUIDE = 저속 선행 유도
  const tgt = S.path[0];
  const dx = tgt.x - S.robot.x, dy = tgt.y - S.robot.y;
  let e = Math.atan2(dy, dx) - S.robot.yaw;
  while (e >  Math.PI) e -= 2 * Math.PI;
  while (e < -Math.PI) e += 2 * Math.PI;
  S.robot.yaw += Math.max(-2.0 * DT, Math.min(2.0 * DT, e));    // 각속도 제한
  const step = Math.min(speed * DT, Math.hypot(dx, dy));
  S.robot.x += Math.cos(S.robot.yaw) * step;
  S.robot.y += Math.sin(S.robot.yaw) * step;

  if (Math.hypot(tgt.x - S.robot.x, tgt.y - S.robot.y) < 0.3) {
    S.path.shift();
    if (!S.path.length) onArrive();
  }
}

function onArrive() {
  S.lastArriveAt = Date.now();
  if (S.state === 'PATROL') {                       // 다음 순찰 지점으로
    S.patrolIdx = (S.patrolIdx + 1) % PATROL_LOOP.length;
    setGoal(PATROL_LOOP[S.patrolIdx]);
  }
  else if (S.state === 'APPROACH')    setState('GATHER');
  else if (S.state === 'GUIDE')       setState('ESCAPED');
  else if (S.state === 'SEARCH_BACK') setState('GUIDE');
}

function samplePath(pts, step) {
  const out = [];
  for (let i = 0; i < pts.length - 1; i++) {
    const a = pts[i], b = pts[i + 1];
    const d = Math.hypot(b.x - a.x, b.y - a.y), n = Math.max(1, Math.ceil(d / step));
    for (let k = 0; k < n; k++) out.push({ x: a.x + (b.x - a.x) * k / n, y: a.y + (b.y - a.y) * k / n, z: 0 });
  }
  const last = pts[pts.length - 1];
  out.push({ x: last.x, y: last.y, z: 0 });
  return out;
}

const yawToQuat = yaw => ({ x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) });

/* ───────────────────────────────────────────────────────────────
   5. 발행 루프 — 실제 시스템의 주기를 흉내 낸다
   ─────────────────────────────────────────────────────────────── */
// map→odom 을 일부러 0 이 아니게 둔다 → TF 2단 합성이 맞는지 목업에서 검증됨
const MAP_ODOM = { x: 0.30, y: -0.20, yaw: 0.05 };

function tfMsg(parent, child, t) {
  return {
    header: { frame_id: parent, stamp: { sec: 0, nanosec: 0 } },
    child_frame_id: child,
    transform: { translation: { x: t.x, y: t.y, z: 0 }, rotation: yawToQuat(t.yaw) },
  };
}

let engineStarted = false;
function startEngineOnce() {
  if (engineStarted) return;
  engineStarted = true;
  emit('/map', buildMap());
  setGoal(PATROL_LOOP[0]);

  setInterval(() => {                                   // 10Hz: FSM + 이동 + TF
    if (S.state === 'GATHER' && Date.now() - S.tGather > 8000) setState('GUIDE');
    stepRobot();

    const c = Math.cos(-MAP_ODOM.yaw), s = Math.sin(-MAP_ODOM.yaw);
    const rx = S.robot.x - MAP_ODOM.x, ry = S.robot.y - MAP_ODOM.y;
    const ob = { x: c * rx - s * ry, y: s * rx + c * ry, yaw: S.robot.yaw - MAP_ODOM.yaw };

    emit('/tf', { transforms: [
      tfMsg('map', 'odom', MAP_ODOM),
      tfMsg('odom', 'base_footprint', ob),
    ]});
    emit('/odom', {}); emit('/scan', {}); emit('/imu/data', {});
  }, 100);

  setInterval(() => {                                   // 1Hz: Nav2 가 /plan 을 재발행하는 주기
    if (!S.path.length || !S.planOn) return;            // 목표 없음/토글 OFF → 발행 중단 (T1 시험)
    const pts = [{ x: S.robot.x, y: S.robot.y }, ...S.path];
    emit('/plan', { poses: samplePath(pts, 0.25).map(p => ({ pose: { position: p } })) });
  }, 1000);

  setInterval(() => {                                   // 상태·싸이렌·Nav2 상태
    emit('/mission_state', { data: S.state });
    emit('/siren', { data: SIREN_ON.includes(S.state) });
    let st = 2;
    if (S.state === 'FAULT') st = 6;
    else if (!S.path.length) st = (Date.now() - S.lastArriveAt < 4000) ? 4 : 1;
    emit('/navigate_to_pose/_action/status', { status_list: [{ status: st }] });
  }, 500);
}

/* ───────────────────────────────────────────────────────────────
   6. 콘솔이 "발행"한 것 처리 — 버튼이 실제로 동작하는 것처럼 보이게
   ─────────────────────────────────────────────────────────────── */
function onPublish(topic, msg) {
  if (topic === '/alarm') {
    emit('/alarm', msg);                                // 화재 마커 갱신
    if (S.state === 'PATROL' || S.state === 'ESCAPED') setState('APPROACH');
  } else if (topic === '/mission_cmd') {
    if (msg.data === 'reset') setState('PATROL');
    if (msg.data === 'abort') setState('FAULT');
  }
  // /cmd_vel 은 목업에서 할 일 없음
}

/* ───────────────────────────────────────────────────────────────
   7. 목업 조작판
   ─────────────────────────────────────────────────────────────── */
const STATES = ['PATROL', 'APPROACH', 'GATHER', 'GUIDE', 'SEARCH_BACK', 'ESCAPED', 'FAULT'];
let panel, tab;

function togglePanel(show) {
  panel.style.display = show ? 'block' : 'none';
  tab.style.display   = show ? 'none'  : 'block';
}

function buildPanel() {
  tab = document.createElement('button');
  tab.textContent = '🧪';
  tab.title = '목업 조작판 열기';
  tab.style.cssText = 'position:fixed;right:10px;bottom:10px;z-index:9999;display:none;' +
    'width:40px;height:40px;border:0;border-radius:20px;background:#2f4a63cc;color:#fff;' +
    'font-size:18px;cursor:pointer';
  document.body.appendChild(tab);

  panel = document.createElement('div');
  panel.id = 'mockPanel';
  panel.innerHTML = `
    <style>
      #mockPanel{position:fixed;right:12px;bottom:12px;z-index:9999;width:262px;
        background:#101820;border:1px solid #3a4a5c;border-radius:10px;padding:12px;
        font:12px/1.5 'Noto Sans KR',sans-serif;color:#cfe0f0;box-shadow:0 6px 24px #0009}
      #mockPanel h3{font-size:12px;color:#ffb04d;margin-bottom:6px;letter-spacing:.05em}
      #mockPanel button{width:100%;margin-top:6px;padding:7px;border:0;border-radius:6px;
        background:#2f4a63;color:#fff;font-size:12px;font-weight:700;cursor:pointer}
      #mockPanel button.warn{background:#8a3b2f}
      #mockPanel select{width:100%;margin-top:6px;padding:6px;background:#12161c;
        color:#cfe0f0;border:1px solid #2a3340;border-radius:6px}
      #mockPanel .kv{color:#7fc4ff;font-weight:700}
      #mockPanel .hint{color:#6b7c8d;font-size:11px;margin-top:8px}
    </style>
    <h3>🧪 목업 조작판 (?mock=1)</h3>
    <div class="hint" style="margin:0 0 6px">지도: 실제 <b>${MAPDEF.name}</b> ·
      ${MAPDEF.width}×${MAPDEF.height} @${MAPDEF.resolution}m · origin (${MAPDEF.origin.x}, ${MAPDEF.origin.y})</div>
    <div>상태: <span class="kv" id="mkState"></span></div>
    <select id="mkSel">${STATES.map(s => `<option>${s}</option>`).join('')}</select>
    <button id="mkFire">🔥 화재 발생 (${FIRE_DEMO.x}, ${FIRE_DEMO.y})</button>
    <button id="mkMove">⏸ 로봇 이동 정지</button>
    <button id="mkPlan" class="warn">🛑 /plan 발행 중단 (T1 시험)</button>
    <button id="mkHide" style="background:#26313d">✕ 조작판 접기</button>
    <div class="hint">주행선 y=${SY}(남측)·${NY}(북측), 연결통로 x=7·17·27 = 지도 실측값.
      waypoint 는 목업 임시값 — 쌍굴 정본 오면 교체.</div>`;
  document.body.appendChild(panel);

  panel.querySelector('#mkSel').onchange = e => setState(e.target.value);
  panel.querySelector('#mkFire').onclick = () => {
    emit('/alarm', { header: { frame_id: 'map' }, pose: { position: { x: FIRE_DEMO.x, y: FIRE_DEMO.y, z: 0 } } });
    if (S.state !== 'FAULT') setState('APPROACH');
  };
  panel.querySelector('#mkHide').onclick = () => togglePanel(false);
  tab.onclick = () => togglePanel(true);
  // 디스플레이 모드(?display=1)에서는 1024×600 을 가리므로 접어 둔다
  togglePanel(!params.has('display'));
  panel.querySelector('#mkMove').onclick = e => {
    S.moving = !S.moving;
    e.target.textContent = S.moving ? '⏸ 로봇 이동 정지' : '▶ 로봇 이동 재개';
  };
  panel.querySelector('#mkPlan').onclick = e => {
    S.planOn = !S.planOn;
    e.target.textContent = S.planOn ? '🛑 /plan 발행 중단 (T1 시험)' : '▶ /plan 발행 재개';
  };
  syncPanel();
}

function syncPanel() {
  if (!panel) return;
  panel.querySelector('#mkState').textContent = S.state;
  panel.querySelector('#mkSel').value = S.state;
}

if (document.readyState === 'loading') window.addEventListener('DOMContentLoaded', buildPanel);
else buildPanel();
})();
