/* ════════════════════════════════════════════════════════════════
   console/mock.js — L1 목업 모드  (계획서 §4)

   목적 : rosbridge·Gazebo·로봇 없이 브라우저만으로 콘솔 화면을 띄운다.
   원리 : ?mock=1 이면 ROSLIB 라이브러리를 "가짜"로 통째로 바꿔치기한다.
          → index.html 본문 로직(구독·콜백·렌더)은 한 줄도 안 고친다.
          → 목업에서 잘 그려지면 진짜 rosbridge 에서도 같은 코드가 돈다.

   사용 : console/ 에서  python3 -m http.server 8000
          http://localhost:8000/?mock=1
          (?mock 이 없으면 이 파일은 아무 일도 하지 않는다 → 관제 모드 그대로)

   옵션 : ?mock=1&map=big   / &map=tight   지도 경계조건 테스트
   ════════════════════════════════════════════════════════════════ */
(function () {
'use strict';

const params = new URLSearchParams(location.search);
if (!params.has('mock')) return;          // 실서비스 경로는 절대 건드리지 않는다
console.log('[MOCK] 목업 모드로 실행합니다. rosbridge 에 접속하지 않습니다.');

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
   2. 가짜 지도 (/map) — T자 터널을 코드로 합성한다
      실제 캡처가 없어도 된다. 오히려 경계조건 테스트에 낫다(계획서 §4)
   ─────────────────────────────────────────────────────────────── */
const MAPS = {
  base:  { res: 0.05, ox: -2.0,  oy: -6.0,  w: 400, h: 240 },   // 기본
  big:   { res: 0.05, ox: -10.0, oy: -15.0, w: 600, h: 600 },   // 로봇이 화면 중앙이 아님
  tight: { res: 0.05, ox: -0.5,  oy: -1.5,  w: 350, h: 60  },   // 납작·잘림 테스트
};
let mapKey = MAPS[params.get('map')] ? params.get('map') : 'base';

// 터널 형상: 본선(가로) + x=14 지선(세로) = T자
const inMain   = (x, y, m) => x >= -m && x <= 16.5 + m && Math.abs(y) <= 1.2 + m;
const inBranch = (x, y, m) => Math.abs(x - 14) <= 1.2 + m && y >= -5 - m && y <= 5 + m;

function buildMap() {
  const M = MAPS[mapKey];
  const data = new Array(M.w * M.h);
  for (let row = 0; row < M.h; row++) {
    for (let col = 0; col < M.w; col++) {
      const x = M.ox + (col + 0.5) * M.res;
      const y = M.oy + (row + 0.5) * M.res;       // OccupancyGrid: row 0 = y 최소
      const free = inMain(x, y, 0)    || inBranch(x, y, 0);
      const near = inMain(x, y, 0.45) || inBranch(x, y, 0.45);
      data[row * M.w + col] = free ? 0 : (near ? 100 : -1);   // 주행가능 / 벽 / 미탐사
    }
  }
  return {
    header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
    info: {
      width: M.w, height: M.h, resolution: M.res,
      origin: { position: { x: M.ox, y: M.oy, z: 0 },
                orientation: { x: 0, y: 0, z: 0, w: 1 } },
    },
    data,
  };
}

/* ───────────────────────────────────────────────────────────────
   3. 시나리오 엔진 — 미션 FSM + 로봇 이동 + 경로
   ─────────────────────────────────────────────────────────────── */
const WP = {
  patrolA: { x: 1.5,  y: 0 },     // 순찰 서쪽 끝
  patrolB: { x: 16.0, y: 0 },     // 순찰 동쪽 끝
  gather:  { x: 11.0, y: 0 },     // 집결지
  escape:  { x: 0.5,  y: 0 },     // 탈출구(안전구역) — waypoints.yaml escape 에 대응
  back:    { x: 14.0, y: 3.5 },   // 역행 재탐색 지점(지선) = 탈출 반대 방향
};

const S = {
  state: 'PATROL',
  robot: { x: 2.89, y: -0.01, yaw: 0 },   // 검증 기록의 스폰 좌표와 동일
  goal:  WP.patrolB,
  patrolTarget: 'B',
  tGather: 0,
  moving: true,        // 패널 토글
  planOn: true,        // 패널 토글 (T1 시험용)
  navStatus: 2,
  lastArriveAt: 0,
};

const SIREN_ON = ['APPROACH', 'GATHER', 'GUIDE', 'SEARCH_BACK'];
const GOAL_OF  = {
  PATROL:      () => (S.patrolTarget === 'B' ? WP.patrolB : WP.patrolA),
  APPROACH:    () => WP.gather,
  GATHER:      () => null,
  GUIDE:       () => WP.escape,
  SEARCH_BACK: () => WP.back,
  ESCAPED:     () => null,
  FAULT:       () => null,
};

function setState(s) {
  if (S.state === s) return;
  S.state = s;
  S.goal = GOAL_OF[s]();
  if (s === 'GATHER') S.tGather = Date.now();
  if (s === 'FAULT')  S.navStatus = 6;
  syncPanel();
}

const DT = 0.1;                                     // 10Hz
function stepRobot() {
  if (!S.goal || !S.moving) return;
  const speed = (S.state === 'GUIDE') ? 0.45 : 0.9; // GUIDE = 저속 선행 유도
  const way = routeTo(S.goal);                      // 지선 진입 시 (14,0) 경유
  const next = way[1] || S.goal;
  const dx = next.x - S.robot.x, dy = next.y - S.robot.y;
  const d = Math.hypot(dx, dy);
  const want = Math.atan2(dy, dx);
  let e = want - S.robot.yaw;
  while (e > Math.PI) e -= 2 * Math.PI;
  while (e < -Math.PI) e += 2 * Math.PI;
  S.robot.yaw += Math.max(-1.5 * DT, Math.min(1.5 * DT, e));   // 각속도 제한
  const step = Math.min(speed * DT, d);
  S.robot.x += Math.cos(S.robot.yaw) * step;
  S.robot.y += Math.sin(S.robot.yaw) * step;

  if (Math.hypot(S.goal.x - S.robot.x, S.goal.y - S.robot.y) < 0.3) onArrive();
}

function onArrive() {
  S.navStatus = 4;
  S.lastArriveAt = Date.now();
  if (S.state === 'PATROL')      { S.patrolTarget = S.patrolTarget === 'B' ? 'A' : 'B'; S.goal = GOAL_OF.PATROL(); S.navStatus = 2; }
  else if (S.state === 'APPROACH')    setState('GATHER');
  else if (S.state === 'GUIDE')       setState('ESCAPED');
  else if (S.state === 'SEARCH_BACK') setState('GUIDE');
}

// 아주 단순한 라우터: 한쪽이 지선(|y|>1.2)이면 교차점 (14,0) 을 경유한다
function routeTo(goal) {
  const junction = { x: 14, y: 0 };
  const pts = [{ x: S.robot.x, y: S.robot.y }];
  if (Math.abs(S.robot.y) > 1.2 || Math.abs(goal.y) > 1.2) pts.push(junction);
  pts.push(goal);
  return pts;
}

function samplePath(pts, step) {
  const out = [];
  for (let i = 0; i < pts.length - 1; i++) {
    const a = pts[i], b = pts[i + 1];
    const d = Math.hypot(b.x - a.x, b.y - a.y), n = Math.max(1, Math.ceil(d / step));
    for (let k = 0; k < n; k++) out.push({ x: a.x + (b.x - a.x) * k / n, y: a.y + (b.y - a.y) * k / n, z: 0 });
  }
  out.push({ x: pts[pts.length - 1].x, y: pts[pts.length - 1].y, z: 0 });
  return out;
}

const yawToQuat = yaw => ({ x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) });

/* ───────────────────────────────────────────────────────────────
   4. 발행 루프 — 실제 시스템의 주기를 흉내 낸다
   ─────────────────────────────────────────────────────────────── */
// map→odom 을 일부러 0 이 아니게 둔다 → TF 2단 합성이 맞는지 목업에서 검증됨
const MAP_ODOM = { x: 0.30, y: -0.20, yaw: 0.05 };

let engineStarted = false;
function startEngineOnce() {
  if (engineStarted) return;
  engineStarted = true;
  emit('/map', buildMap());

  setInterval(() => {                                   // 10Hz: FSM + 이동 + TF
    if (S.state === 'GATHER' && Date.now() - S.tGather > 8000) setState('GUIDE');
    stepRobot();

    // odom 기준 위치 = map→odom 의 역변환 ∘ map 위치 (합성 검증용으로 역산)
    const c = Math.cos(-MAP_ODOM.yaw), s = Math.sin(-MAP_ODOM.yaw);
    const rx = S.robot.x - MAP_ODOM.x, ry = S.robot.y - MAP_ODOM.y;
    const ob = { x: c * rx - s * ry, y: s * rx + c * ry, yaw: S.robot.yaw - MAP_ODOM.yaw };

    emit('/tf', { transforms: [
      tfMsg('map', 'odom', MAP_ODOM),
      tfMsg('odom', 'base_footprint', ob),
    ]});
    emit('/odom', {});
  }, 100);

  setInterval(() => {                                   // 10Hz 센서 신선도
    emit('/scan', {}); emit('/imu/data', {});
  }, 100);

  setInterval(() => {                                   // 1Hz: Nav2 가 /plan 을 재발행하는 주기
    if (!S.goal || !S.planOn) return;                   // 목표 없음/토글 OFF → 발행 중단 (T1 시험)
    emit('/plan', { poses: samplePath(routeTo(S.goal), 0.25).map(p => ({ pose: { position: p } })) });
  }, 1000);

  setInterval(() => {                                   // 상태·싸이렌·Nav2 상태
    emit('/mission_state', { data: S.state });
    emit('/siren', { data: SIREN_ON.includes(S.state) });
    let st = S.navStatus;
    if (S.state === 'FAULT') st = 6;
    else if (S.goal) st = 2;
    else if (Date.now() - S.lastArriveAt < 4000) st = 4;
    emit('/navigate_to_pose/_action/status', { status_list: [{ status: st }] });
  }, 500);
}

function tfMsg(parent, child, t) {
  return {
    header: { frame_id: parent, stamp: { sec: 0, nanosec: 0 } },
    child_frame_id: child,
    transform: { translation: { x: t.x, y: t.y, z: 0 }, rotation: yawToQuat(t.yaw) },
  };
}

/* ───────────────────────────────────────────────────────────────
   5. 콘솔이 "발행"한 것 처리 — 버튼이 실제로 동작하는 것처럼 보이게
   ─────────────────────────────────────────────────────────────── */
function onPublish(topic, msg) {
  if (topic === '/alarm') {
    emit('/alarm', msg);                                // 화재 마커 갱신
    if (S.state === 'PATROL' || S.state === 'ESCAPED') setState('APPROACH');
  } else if (topic === '/mission_cmd') {
    if (msg.data === 'reset') { S.patrolTarget = 'B'; setState('PATROL'); }
    if (msg.data === 'abort') setState('FAULT');
  }
  // /cmd_vel 은 목업에서 할 일 없음
}

/* ───────────────────────────────────────────────────────────────
   6. 목업 조작 패널 — 시나리오를 손으로 돌려본다
   ─────────────────────────────────────────────────────────────── */
const STATES = ['PATROL', 'APPROACH', 'GATHER', 'GUIDE', 'SEARCH_BACK', 'ESCAPED', 'FAULT'];
let panel;

function buildPanel() {
  panel = document.createElement('div');
  panel.id = 'mockPanel';
  panel.innerHTML = `
    <style>
      #mockPanel{position:fixed;right:12px;bottom:12px;z-index:9999;width:270px;
        background:#101820;border:1px solid #3a4a5c;border-radius:10px;padding:12px;
        font:12px/1.5 'Noto Sans KR',sans-serif;color:#cfe0f0;box-shadow:0 6px 24px #0009}
      #mockPanel h3{font-size:12px;color:#ffb04d;margin-bottom:8px;letter-spacing:.05em}
      #mockPanel button{width:100%;margin-top:6px;padding:7px;border:0;border-radius:6px;
        background:#2f4a63;color:#fff;font-size:12px;font-weight:700;cursor:pointer}
      #mockPanel button.warn{background:#8a3b2f}
      #mockPanel select{width:100%;margin-top:6px;padding:6px;background:#12161c;
        color:#cfe0f0;border:1px solid #2a3340;border-radius:6px}
      #mockPanel .kv{color:#7fc4ff;font-weight:700}
      #mockPanel .hint{color:#6b7c8d;font-size:11px;margin-top:8px}
    </style>
    <h3>🧪 목업 조작판 (?mock=1)</h3>
    <div>상태: <span class="kv" id="mkState"></span></div>
    <select id="mkSel">${STATES.map(s => `<option>${s}</option>`).join('')}</select>
    <button id="mkFire">🔥 화재 발생 (14, 0)</button>
    <button id="mkMove">⏸ 로봇 이동 정지</button>
    <button id="mkPlan" class="warn">🛑 /plan 발행 중단 (T1 시험)</button>
    <select id="mkMap">
      <option value="base">지도: 기본 T자</option>
      <option value="big">지도: 큰 지도(여백)</option>
      <option value="tight">지도: 납작·잘림</option>
    </select>
    <div class="hint">/plan 을 중단했을 때 경로가 사라지는지 = T1 완료 판정.
    지금 코드는 안 사라지는 게 정상(=결함 재현).</div>`;
  document.body.appendChild(panel);

  panel.querySelector('#mkSel').onchange = e => setState(e.target.value);
  panel.querySelector('#mkFire').onclick = () => {
    const m = { header: { frame_id: 'map' }, pose: { position: { x: 14, y: 0, z: 0 } } };
    emit('/alarm', m);
    if (S.state !== 'FAULT') setState('APPROACH');
  };
  panel.querySelector('#mkMove').onclick = e => {
    S.moving = !S.moving;
    e.target.textContent = S.moving ? '⏸ 로봇 이동 정지' : '▶ 로봇 이동 재개';
  };
  panel.querySelector('#mkPlan').onclick = e => {
    S.planOn = !S.planOn;
    e.target.textContent = S.planOn ? '🛑 /plan 발행 중단 (T1 시험)' : '▶ /plan 발행 재개';
  };
  panel.querySelector('#mkMap').value = mapKey;
  panel.querySelector('#mkMap').onchange = e => { mapKey = e.target.value; emit('/map', buildMap()); };
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
