/* ════════════════════════════════════════════════════════════════
   console/mock.js — L1 목업 모드  (계획서 §4)

   목적 : rosbridge·Gazebo·로봇 없이 브라우저만으로 콘솔 화면을 띄운다.
   원리 : ?mock=1 이면 ROSLIB 라이브러리를 "가짜"로 통째로 바꿔치기한다.
          → index.html 본문 로직(구독·콜백·렌더)은 한 줄도 안 고친다.

   🔴 지도는 합성이 아니다 — maps/twin_map_loc.pgm (실제 SLAM 저장본) 을
      mock_map_twin.js 픽스처로 변환해 그대로 쓴다. 원점·해상도도 yaml 원본값.

   ✅ 주행 waypoint 는 src/mission_manager/config/waypoints_twin.yaml 정본을 그대로 옮겼다
      (임시값 시절 좌표는 폐기 — 아래 §3 참조).

   사용 : console/ 에서  python3 -m http.server 8000
          http://localhost:8000/?mock=1
          ?speed=0.26 → 실제 순항속도로 재생 (기본은 시험 편의상 빠르게)
   ════════════════════════════════════════════════════════════════ */
(function () {
'use strict';

const params = new URLSearchParams(location.search);
if (!params.has('mock')) return;          // 실서비스 경로는 절대 건드리지 않는다

/* 🔴 ?mock 과 ?display 동시 사용 차단 (2026-08-17 역할 A PR 2차 검토 §3)
   문제: ?mock=1&display=1 로 열면 7인치 패널에 가짜 지도·경로·상태가 전체화면으로 뜨는데,
        가짜라는 표시가 우하단 34px 회색 'M' 버튼뿐이라 2m 밖에서는 안 보인다.
        디스플레이는 자동 실행이 요건이라, URL 을 한 번 잘못 적으면 그날부터 조용히 가짜가 뜬다.
        (state_marker.py 를 뺀 이유와 같은 실패 유형 — 조용히 틀린 표시는 표시가 없는 것보다 나쁘다)

   조치: 기본 차단. 여기서 return 하면 목업이 안 붙고 진짜 rosbridge 를 찾다 DISCONNECTED 가
        크게 뜬다 → 조용한 실패가 아니라 눈에 보이는 실패가 된다.

   예외: 실패널 판정(가독성·표식 크기·절전·자동 실행)은 시뮬 없이 화면만 켜서 해야 하고,
        그때 띄울 수 있는 것은 목업뿐이다. 그래서 ?labdemo=1 을 함께 명시한 경우에만 허용하고,
        그 경우 화면 상단에 지워지지 않는 경고 배너를 강제한다.
        → 자동 실행 URL 에 플래그 3개가 동시에 잘못 적힐 확률은 사실상 없고,
          설령 그렇게 되어도 배너 때문에 '조용히'가 성립하지 않는다. */
if (params.has('display') && !params.has('labdemo')) {
  console.error('[MOCK] ?display 와 ?mock 은 같이 쓸 수 없다 — 실물 패널에 가짜가 뜬다. ' +
                '패널에서 목업을 봐야 하면 ?labdemo=1 을 함께 붙일 것 (경고 배너 강제).');
  return;
}

const MAPDEF = window.MOCK_MAP_TWIN;
if (!MAPDEF) {
  console.error('[MOCK] mock_map_twin.js 가 없다. index.html 에서 mock.js 보다 먼저 불러야 한다.');
  return;
}
console.log('[MOCK] 목업 모드 —', MAPDEF.name,
            `${MAPDEF.width}×${MAPDEF.height} @${MAPDEF.resolution}m`, 'origin', MAPDEF.origin);

/* 지워지지 않는 목업 경고 배너 — 패널에서 목업을 볼 때(?labdemo=1) 강제.
   화면 상단 전체 폭, 접기 불가. 1초마다 존재를 확인해 없어지면 다시 만든다. */
function mountMockBanner() {
  const ID = 'mockBanner';
  const make = () => {
    if (document.getElementById(ID) || !document.body) return;
    const b = document.createElement('div');
    b.id = ID;
    b.textContent = '목업 — 실제 상황 아님 · MOCK DATA';
    b.style.cssText = 'position:fixed;top:0;left:0;right:0;z-index:2147483647;' +
      'background:#b3261e;color:#fff;text-align:center;letter-spacing:.1em;' +
      'font:800 19px/1.7 "Noto Sans KR",sans-serif;pointer-events:none;' +
      'box-shadow:0 2px 10px #000a';
    document.body.appendChild(b);
  };
  make();
  setInterval(make, 1000);
}
if (params.has('labdemo')) {
  if (document.readyState === 'loading') window.addEventListener('DOMContentLoaded', mountMockBanner);
  else mountMockBanner();
}

/* ───────────────────────────────────────────────────────────────
   1. 가짜 ROSLIB — 구독자 명단을 들고 있다가 엔진이 부르면 콜백을 때린다
   ─────────────────────────────────────────────────────────────── */
const subs = {};                                   // { 토픽명: [콜백, …] }
function emit(topic, msg) {
  (subs[topic] || []).forEach(cb => { try { cb(msg); } catch (e) { console.error(e); } });
}

let lastRos = null;
function MockRos() {
  // 실제 rosbridge 는 소켓이 끊기면 그 위의 구독도 같이 죽는다.
  // 목업도 똑같이 비워야 재접속 시험(T4)에서 '콜백 중복'이 가짜로 보이지 않는다.
  for (const k in subs) delete subs[k];
  lastRos = this;
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
      ⚠ y=0(1번 굴 중심선)에는 미탐사(205) 셀이 77개 산재한다 (x -2.98 ~ 20.77m, image row 261).
        지도 저장 시의 자국이며 실제 장애물은 아니다. 정본 중심선이 y=0 이라 주행 샘플의
        약 6%가 이 위를 지난다. Nav2 는 allow_unknown: false 라 이 셀들을 통행 불가로 보지만,
        연속이 아니라 산재이고 쌍굴은 통로 폭이 넓어 플래너가 살짝 비껴 우회한다(막히지 않음).
        🔴 실차(R5 지도, 최협부 1.65m)에서는 여유가 없어 결과가 다를 수 있다 — 역할 A 가 검사 담당.
   ─────────────────────────────────────────────────────────────── */
const SY = 0.0, NY = 10.0;      // 1번 굴 / 2번 굴 중심선 (정본)

/* ✅ waypoints_twin.yaml 정본 반영 (2026-08-14).
   출처: src/mission_manager/config/waypoints_twin.yaml (07-07 작성)
     · corridor_graph.nodes 를 그대로 옮김 (b1_* = 1번 굴, b2_* = 2번 굴)
     · patrol 5개 지점, gather (22,0), escape (0,0), gather_wait_sec 8.0
     · normal_speed 0.26 / guide_speed 0.12, search_back.min_fire_dist 5.0
   ⚠ 이전 버전의 임시 좌표(y=0.6 주행선 등)는 폐기했다. */
const N = {
  SW: { x: 0,  y: SY }, S5:  { x: 5,  y: SY }, S7:  { x: 7,  y: SY },
  S17:{ x: 17, y: SY }, S22: { x: 22, y: SY }, S24: { x: 24, y: SY },
  S27:{ x: 27, y: SY }, SE:  { x: 35, y: SY },
  NW: { x: 0,  y: NY }, N7:  { x: 7,  y: NY }, N17: { x: 17, y: NY },
  N27:{ x: 27, y: NY }, NE:  { x: 35, y: NY },
};
const EDGES = [                                   // corridor_graph.edges 그대로
  ['SW','S5'], ['S5','S7'], ['S7','S17'], ['S17','S22'], ['S22','S24'], ['S24','S27'], ['S27','SE'],
  ['NW','N7'], ['N7','N17'], ['N17','N27'], ['N27','NE'],
  ['S7','N7'], ['S17','N17'], ['S27','N27'],      // 피난연결통로 x=7 / 17 / 27
];
const ADJ = {};
for (const [a, b] of EDGES) { (ADJ[a] = ADJ[a] || []).push(b); (ADJ[b] = ADJ[b] || []).push(a); }

const FIRE_DEMO = { x: 30, y: 0 };                // 표준 테스트 화재
const GOAL_NODE = {
  APPROACH:    'S22',    // gather (22,0) — 화재 30 에서 gather_dist 8.0
  GUIDE:       'SW',     // escape (0,0) = 스폰 지점
  SEARCH_BACK: 'S24',    // 화재에서 6m (min_fire_dist 5.0 준수)
};
// patrol: 1번 굴 동진 → 동쪽 통로(x=27) → 2번 굴 서진 → 서쪽 통로(x=7) 복귀
const PATROL_LOOP = ['S5', 'S27', 'N27', 'N7', 'S7'];

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
/* 🔴 주행 속도 = waypoints.yaml 정본값 (2026-08-14 수정).
   이전 목업 기본값 2.0 m/s 는 시험을 빨리 돌리려고 임의로 잡은 값이었고 실제보다 약 8배 빨랐다.
   빨리 확인하고 싶을 때만 ?speed=2 처럼 덮어쓴다. */
const SPEED       = parseFloat(params.get('speed')) || 0.26;   // normal
const SPEED_GUIDE = SPEED === 0.26 ? 0.12 : SPEED * 0.46;      // guide (저속 선행 유도)

/* ★ 목업 배속 — 실제 속도는 그대로 두고 '재생 속도'만 올린다.
   시나리오를 빨리 돌려보려는 용도. 화면에 ×N 이 항상 표시되므로
   실제 로봇이 그 속도로 달리는 것으로 오해할 여지가 없다.
   빨리 보려면 조작판 버튼 또는 주소에 ?x=10 */
let speedMult = parseFloat(params.get('x')) || 1;      // 기본 ×1 = 정본 속도 그대로
const MULTS = [1, 3, 6, 10];                          // 버튼으로 순환. T4 2시간 시험은 반드시 ×1
const S = {
  state: 'PATROL',
  robot: { x: 1.0, y: SY, yaw: 0 },   // 스폰 = map 원점 부근
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
  // 배속은 '시간을 빨리 돌리는 것' → 직진뿐 아니라 회전·도착판정도 같이 배속해야
  // 궤적 모양이 ×1 일 때와 똑같이 유지된다. (안 그러면 코너를 잘라 돌아 벽을 스친다)
  const speed = ((S.state === 'GUIDE') ? SPEED_GUIDE : SPEED) * speedMult;
  const tgt = S.path[0];
  const dx = tgt.x - S.robot.x, dy = tgt.y - S.robot.y;
  let e = Math.atan2(dy, dx) - S.robot.yaw;
  while (e >  Math.PI) e -= 2 * Math.PI;
  while (e < -Math.PI) e += 2 * Math.PI;
  const wmax = 2.0 * DT * speedMult;                            // 각속도도 같은 배속
  S.robot.yaw += Math.max(-wmax, Math.min(wmax, e));
  const step = Math.min(speed * DT, Math.hypot(dx, dy));
  S.robot.x += Math.cos(S.robot.yaw) * step;
  S.robot.y += Math.sin(S.robot.yaw) * step;

  const reach = Math.max(0.3, speed * DT * 1.6);                // 한 틱 이동량보다 커야 지나치지 않는다
  if (Math.hypot(tgt.x - S.robot.x, tgt.y - S.robot.y) < reach) {
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
  tab.textContent = 'M';
  tab.title = '목업 조작판 열기';
  tab.style.cssText = 'position:fixed;right:10px;bottom:10px;z-index:9999;display:none;' +
    'width:34px;height:34px;border:1px solid #2f363c;background:#15181b;color:#8b939b;' +
    'font:600 12px ui-monospace,Menlo,monospace;cursor:pointer';
  document.body.appendChild(tab);

  panel = document.createElement('div');
  panel.id = 'mockPanel';
  panel.innerHTML = `
    <style>
      #mockPanel{position:fixed;right:12px;bottom:12px;z-index:9999;width:250px;
        background:#15181b;border:1px solid #2f363c;padding:11px;
        font:12px/1.5 'Noto Sans KR',sans-serif;color:#d7dbdf}
      #mockPanel h3{font-family:ui-monospace,Menlo,monospace;font-size:10px;font-weight:500;
        letter-spacing:.14em;color:#d0a215;margin-bottom:8px;text-transform:uppercase}
      #mockPanel button{width:100%;margin-top:5px;padding:7px;border:1px solid #2f363c;
        background:#1b1f23;color:#d7dbdf;font-size:12px;font-weight:600;cursor:pointer}
      #mockPanel button:hover{background:#22272c}
      #mockPanel button.warn{border-color:#7a3428;color:#e4523c}
      #mockPanel select{width:100%;margin-top:5px;padding:6px;background:#0e1113;color:#d7dbdf;
        border:1px solid #2f363c;font-family:ui-monospace,Menlo,monospace;font-size:11px}
      #mockPanel .kv{font-family:ui-monospace,Menlo,monospace;color:#4f9ee8;font-weight:600}
      #mockPanel .hint{color:#5b636a;font-size:10.5px;line-height:1.5;margin-top:8px}
    </style>
    <h3>MOCK CONTROL · ?mock=1</h3>
    <div class="hint" style="margin:0 0 6px">지도: 실제 <b>${MAPDEF.name}</b> ·
      ${MAPDEF.width}×${MAPDEF.height} @${MAPDEF.resolution}m · origin (${MAPDEF.origin.x}, ${MAPDEF.origin.y})</div>
    <div>상태: <span class="kv" id="mkState"></span></div>
    <select id="mkSel">${STATES.map(s => `<option>${s}</option>`).join('')}</select>
    <button id="mkFire">화재 발생 (${FIRE_DEMO.x}, ${FIRE_DEMO.y})</button>
    <button id="mkMove">로봇 이동 정지</button>
    <button id="mkPlan" class="warn">/plan 발행 중단 (T1 시험)</button>
    <button id="mkDrop" class="warn">연결 끊김 시뮬 (T4 시험)</button>
    <button id="mkSpeed">재생 배속</button>
    <button id="mkHide" style="color:#8b939b">조작판 접기</button>
    <div class="hint">waypoints_twin.yaml 정본 반영 (중심선 y=0·10, 통로 x=7·17·27,
      집결 22, 탈출 0, 0.26/0.12 m/s). 배속은 재생 속도일 뿐 설계값이 아님.</div>`;
  document.body.appendChild(panel);

  panel.querySelector('#mkSel').onchange = e => setState(e.target.value);
  panel.querySelector('#mkFire').onclick = () => {
    emit('/alarm', { header: { frame_id: 'map' }, pose: { position: { x: FIRE_DEMO.x, y: FIRE_DEMO.y, z: 0 } } });
    if (S.state !== 'FAULT') setState('APPROACH');
  };
  const spBtn = panel.querySelector('#mkSpeed');
  const paintSpeed = () => {
    spBtn.textContent = `재생 배속 \u00d7${speedMult}`;
    spBtn.style.color = speedMult > 1 ? '#d0a215' : '';
  };
  spBtn.onclick = () => {
    speedMult = MULTS[(MULTS.indexOf(speedMult) + 1) % MULTS.length] || 1;
    paintSpeed();
  };
  paintSpeed();
  panel.querySelector('#mkDrop').onclick = () => {
    if (lastRos) lastRos.close();          // index.html 이 3초 뒤 자동 재접속한다
  };
  panel.querySelector('#mkHide').onclick = () => togglePanel(false);
  tab.onclick = () => togglePanel(true);
  // 디스플레이 모드(?display=1)에서는 1024×600 을 가리므로 접어 둔다
  togglePanel(!params.has('display'));
  panel.querySelector('#mkMove').onclick = e => {
    S.moving = !S.moving;
    e.target.textContent = S.moving ? '로봇 이동 정지' : '로봇 이동 재개';
  };
  panel.querySelector('#mkPlan').onclick = e => {
    S.planOn = !S.planOn;
    e.target.textContent = S.planOn ? '/plan 발행 중단 (T1 시험)' : '/plan 발행 재개';
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
