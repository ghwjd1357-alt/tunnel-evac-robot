/* ═══════════════════════════════════════════════════════════════════
   ros.js — rosbridge 연결과 토픽 배선 (2026-09-02)

   이 파일의 규칙: **여기서는 DOM 을 만지지 않는다.**
   토픽이 오면 state.update() / state.quiet() 로 값만 바꾼다.
   화면은 각 화면 모듈이 onChange 로 받아 그린다.

   ── 07-19 에 밟은 함정 두 개 (그대로 유지) ────────────────────
   ① rosbridge 서비스의 serviceType 패키지명은 `rosapi` 가 아니라 `rosapi_msgs`.
      틀리면 에러 없이 응답만 조용히 유실된다(콜백이 영영 안 온다).
   ② 재접속마다 setInterval 이 누적된다 — 구독은 소켓과 함께 죽지만
      JS 타이머는 영원히 산다. 새로 걸기 전에 반드시 이전 것을 해제한다.
   ═══════════════════════════════════════════════════════════════════ */

import { state, update, quiet } from './state.js';
import { pushLog } from './log.js';

export let ros = null;
let pubCmd = null, pubAlarm = null, pubCmdVel = null, pubSay = null;
let rttTimer = null;

/* 라이다 점군을 map 으로 옮기려면 base_footprint→laser 오프셋이 필요하다.
   /tf_static 에서 읽고, 없으면 0 으로 둔다(수 cm 라 화면상 차이는 미미). */
let tfBaseLaser = { x: 0, y: 0, yaw: 0 };
let tfMapOdom = null, tfOdomBase = null;

/* ── 자세 이력 (시각 맞추기용) ──────────────────────────────────────
   🔴 왜 필요한가
   /scan 이 도착했을 때 '가장 최근에 받은' 자세를 그냥 쓰면, 그 자세는 최대
   한 주기만큼 낡은 값이다. 제자리 회전(0.63 rad/s)에서 50 ms 지연 = 1.8°,
   3 m 거리에서 9 cm 오차 — 실측 7 cm 와 일치했다(2026-09-04).
   → 로봇 안의 tf2 가 하는 일을 여기서 한다: **스캔의 타임스탬프에 맞는 자세를
     이력에서 찾아 보간**한다. 그래야 회전 중에도 점군이 벽에 붙는다. */
const POSE_HISTORY = [];          // [{ t(ms), x, y, yaw }] 시간 오름차순
const POSE_HISTORY_MAX = 80;      // 40Hz × 2초
const TIME_JUMP_BACK_MS = 1000;   // 이만큼 뒤로 뛰면 '되감기'로 본다

const stampMs = h => (h && h.stamp) ? h.stamp.sec * 1000 + h.stamp.nanosec / 1e6 : null;

/** 각도 보간 — 경계(±π)를 넘어가도 짧은 쪽으로 돈다 */
function lerpYaw(a, b, k) {
  let d = b - a;
  while (d >  Math.PI) d -= 2 * Math.PI;
  while (d < -Math.PI) d += 2 * Math.PI;
  return a + d * k;
}

/** t(ms) 시점의 자세. 이력 밖이면 가장 가까운 끝값을 쓴다. */
function poseAt(t) {
  const H = POSE_HISTORY;
  if (!H.length) return null;
  if (t == null || t <= H[0].t) return H[0];
  if (t >= H[H.length - 1].t) return H[H.length - 1];
  let lo = 0, hi = H.length - 1;
  while (hi - lo > 1) { const mid = (lo + hi) >> 1; (H[mid].t <= t ? lo = mid : hi = mid); }
  const a = H[lo], b = H[hi];
  const span = b.t - a.t;
  if (span <= 0) return b;
  const k = (t - a.t) / span;
  return { x: a.x + (b.x - a.x) * k, y: a.y + (b.y - a.y) * k, yaw: lerpYaw(a.yaw, b.yaw, k) };
}

const quatToYaw = q => Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));

/** 2D 변환 합성 a∘b (a 좌표계에서 본 b) */
/** map ← odom 변환. 로컬 costmap 이 odom 좌표계라 지도에 얹으려면 필요하다. */
export function getMapOdom() { return tfMapOdom; }

export function compose(a, b) {
  const c = Math.cos(a.yaw), s = Math.sin(a.yaw);
  return { x: a.x + c * b.x - s * b.y, y: a.y + s * b.x + c * b.y, yaw: a.yaw + b.yaw };
}

export function connect() {
  ros = new ROSLIB.Ros({ url: 'ws://' + window.location.hostname + ':9090' });
  ros.on('connection', () => {
    update({ connected: true, everConnected: true });
    pushLog('SYS', 'rosbridge 연결됨', '', 'state');
    wire();
  });
  ros.on('close', () => {
    update({ connected: false });
    pushLog('SYS', 'rosbridge 연결 끊김 — 3초 후 재시도', '', 'alarm');
    setTimeout(connect, 3000);
  });
  ros.on('error', () => { /* close 가 뒤따르므로 별도 처리 없음 */ });
}

const sub = (name, type, cb, throttle) =>
  new ROSLIB.Topic({ ros, name, messageType: type,
                     ...(throttle ? { throttle_rate: throttle, queue_length: 1 } : {}) }).subscribe(cb);

function wire() {
  /* ── 미션 ─────────────────────────────────────────────────── */
  sub('/mission_state', 'std_msgs/msg/String', m => {
    quiet({ fresh: { ...state.fresh, mission: Date.now() } });
    if (m.data === state.mission) return;                 // 상시 발행 → 전이 때만 반응
    const now = Date.now();
    if (state.mission !== null) pushLog('STATE', `${state.mission} → ${m.data}`, '', 'state');
    const history = [...state.history, { state: m.data, at: now }].slice(-40);

    /* 🔴 순찰로 되돌아왔다 = 이전 사건이 끝났다 (임무 종결 또는 관제 reset).
       화재·대피자 신고 마커를 지운다. 안 지우면 지난 임무의 화재가 새 임무 화면에
       계속 떠 있다 — bag 되감기에서 '알람 전인데 화재가 표시된다'로 드러났다(09-04).
       ⚠ 이 콜백은 **상태가 바뀔 때만** 실행되므로, 알람 직후 들어오는 PATROL
          중복 발행에 지워질 염려는 없다 (그때는 상태가 그대로라 위에서 빠져나간다). */
    const cleared = (m.data === 'PATROL' && state.mission !== null)
                    ? { fireXY: null, victim: null, phaseT0: now, phaseDist0: state.distance } : {};

    update({ mission: m.data, missionSince: now, history, ...cleared,
             missionT0: state.missionT0 ?? now,
             phaseT0: state.phaseT0 ?? now });
  });

  sub('/siren', 'std_msgs/msg/Bool', m => {
    if (m.data !== state.siren) {
      pushLog('SIREN', m.data ? '싸이렌 ON' : '싸이렌 OFF', '', 'warn');
      update({ siren: m.data });
    }
  });

  sub('/person_status', 'std_msgs/msg/String', m => {
    quiet({ fresh: { ...state.fresh, person: Date.now() } });
    if (m.data !== state.personStatus) update({ personStatus: m.data });
  }, 500);

  sub('/victim', 'geometry_msgs/msg/PoseStamped', m => {
    update({ victim: { x: m.pose.position.x, y: m.pose.position.y } });
    pushLog('VICTIM', '쓰러진 대피자 신고',
            `${m.pose.position.x.toFixed(2)}  ${m.pose.position.y.toFixed(2)}`, 'alarm');
  });

  /* ── 지도·위치 ────────────────────────────────────────────── */
  sub('/map', 'nav_msgs/msg/OccupancyGrid', m => {
    const first = !state.mapInfo;
    window.dispatchEvent(new CustomEvent('map:grid', { detail: m }));   // map.js 가 렌더
    quiet({ mapInfo: m.info });
    if (first) pushLog('MAP', `지도 받음  ${m.info.width}x${m.info.height}`,
                       `${m.info.resolution.toFixed(3)} m/셀`, 'state');
  });

  sub('/tf', 'tf2_msgs/msg/TFMessage', m => {
    let baseStamp = null;
    for (const t of m.transforms) {
      const tr = { x: t.transform.translation.x, y: t.transform.translation.y,
                   yaw: quatToYaw(t.transform.rotation) };
      if (t.header.frame_id === 'map' && t.child_frame_id === 'odom') tfMapOdom = tr;
      if (t.header.frame_id === 'odom' &&
          (t.child_frame_id === 'base_footprint' || t.child_frame_id === 'base_link')) {
        tfOdomBase = tr;
        baseStamp = stampMs(t.header);      // 빠르게 변하는 쪽의 시각을 기준으로 쌓는다
      }
    }
    if (!tfMapOdom || !tfOdomBase) return;
    const r = compose(tfMapOdom, tfOdomBase);

    /* 자세 이력에 시각과 함께 넣는다 (스캔 시각 맞추기용) */
    if (baseStamp != null) {
      const H = POSE_HISTORY;
      const last = H.length ? H[H.length - 1].t : null;

      /* 🔴 시간이 뒤로 뛰면 이력을 통째로 버린다 (2026-09-04).
         이력을 '시각 오름차순'으로만 쌓았더니, bag 이 한 바퀴 돌아 타임스탬프가
         330초 뒤로 점프하면 그 뒤 자세가 전부 '과거'라 하나도 안 쌓였다.
         → 이력이 이전 바퀴 끝(탈출구, 아래 복도)에 얼어붙고 그 자세로 점군을
           변환해서 **점군이 화면 아래에 고정되고 새로고침해야 풀렸다.**
         실차에서도 클럭 리셋·재기동이면 같은 일이 난다. 되감기는 버리는 게 맞다. */
      if (last != null && baseStamp < last - TIME_JUMP_BACK_MS) {
        H.length = 0;
        pushLog('TIME', '시각이 되감김 — 누적값 초기화', '', 'warn');
        update({ trail: [], distance: 0, missionT0: null, history: [],
                 fireXY: null, victim: null, phaseT0: null, phaseDist0: 0 });
      }

      if (!H.length || baseStamp > H[H.length - 1].t) {
        H.push({ t: baseStamp, x: r.x, y: r.y, yaw: r.yaw });
        if (H.length > POSE_HISTORY_MAX) H.shift();
      }
    }
    const prev = state.robot;
    let { distance, trail } = state;
    if (prev) {
      const d = Math.hypot(r.x - prev.x, r.y - prev.y);
      if (d > 0.02) {                       // 노이즈 제거 후에만 누적
        distance += d;
        trail = trail.concat([[r.x, r.y]]);
        if (trail.length > 4000) trail = trail.slice(-4000);
      }
    } else {
      trail = [[r.x, r.y]];
    }
    quiet({ robot: r, distance, trail, fresh: { ...state.fresh, tf: Date.now() } });
    /* 🔴 스로틀 50ms — /scan 을 map 으로 옮길 때 쓰는 자세가 낡으면 점군이 벽에서 밀린다.
       🔴 실측(2026-09-04): 스로틀 25ms 를 줘도 자세 이력은 **9.4Hz** 밖에 안 쌓였다.
          /tf 한 메시지에 map→odom 과 odom→base 가 번갈아 들어오는데, 스로틀이
          그 절반을 버리면 내가 쓰는 odom→base 는 절반만 남기 때문이다.
          → 10ms(=100Hz 상한)로 사실상 스로틀을 푼다. /tf 는 변환 1~2개짜리
            작은 메시지라 50Hz 라도 약 20 KB/s 로 /scan 보다 훨씬 가볍다. */
  }, 10);

  sub('/plan', 'nav_msgs/msg/Path', m => {
    quiet({ planPts: m.poses.map(p => [p.pose.position.x, p.pose.position.y]) });
  }, 500);

  /* /alarm — 🔴 **정상 경로는 로봇의 화재 탐지**다.
     perception_adapter 가 /detections 를 받아 여기로 발행한다
     (`adapter_node.py:521` · `PROJECT_CONTEXT §4.1` — "/detections → /alarm 은 한 줄도
     안 바뀐다"). 관제의 지도 클릭은 검출이 실패했을 때 사람이 메우는 **대체 경로**다
     (`PROJECT_CONTEXT` — "검출은 오퍼레이터가 수동 /alarm 으로 즉시 메운다").
     → 화면 라벨은 `화재 탐지` 다. `신고` 로 쓰면 로봇이 못 찾은 것처럼 읽힌다.

     관례상 2회 발행되고 bag 재생에서도 반복된다. 같은 좌표가 연달아 오면 로그를
     쌓지 않는다 — 같은 사건은 한 줄이어야 한다. */
  let lastAlarmKey = null, lastAlarmT = 0;
  sub('/alarm', 'geometry_msgs/msg/PoseStamped', m => {
    const xy = { x: m.pose.position.x, y: m.pose.position.y };
    const key = `${xy.x.toFixed(2)},${xy.y.toFixed(2)}`;
    const now = Date.now();
    update({ fireXY: xy });
    if (key === lastAlarmKey && now - lastAlarmT < 10000) return;   // 10초 안의 같은 좌표 = 같은 사건
    lastAlarmKey = key; lastAlarmT = now;
    /* 🔴 대응 경과·이동거리는 여기서부터 잰다 (화재 감지 시점) */
    update({ phaseT0: now, phaseDist0: state.distance });
    pushLog('ALARM', '화재 감지', `${xy.x.toFixed(2)}  ${xy.y.toFixed(2)}`, 'alarm');
  });

  /* /scan — 로봇 자세로 map 좌표까지 옮겨서 저장한다.
     🔴 스로틀 100ms = 센서 원래 속도(bag 실측 10.6 Hz). 그 이상 올려도 데이터가 없다.
        예전 200ms 는 절반을 버려서 회전이 끊겨 보였다.
     ⚠ 비용: rosbridge 가 720점을 JSON 으로 바꿔 보낸다 (약 10 KB × 10 Hz = 100 KB/s).
        localhost 는 무시할 수준이고, 젯슨 실차에서 부하가 문제되면 이 값을 200 으로
        되돌리면 된다 (설계 근거 = 0718_관제시스템.md §5.5). */
  sub('/scan', 'sensor_msgs/msg/LaserScan', m => {
    quiet({ fresh: { ...state.fresh, scan: Date.now() } });
    /* 🔴 '지금의 자세'가 아니라 **이 스캔이 찍힌 순간의 자세**를 쓴다 (위 POSE_HISTORY 주석) */
    const at = poseAt(stampMs(m.header)) || state.robot;
    if (!at) return;
    const base = compose(at, tfBaseLaser);
    const c = Math.cos(base.yaw), s = Math.sin(base.yaw);
    const pts = [];
    for (let i = 0; i < m.ranges.length; i++) {
      const r = m.ranges[i];
      if (!isFinite(r) || r < m.range_min || r > m.range_max) continue;
      const a = m.angle_min + i * m.angle_increment;
      const lx = r * Math.cos(a), ly = r * Math.sin(a);
      pts.push([base.x + c * lx - s * ly, base.y + s * lx + c * ly]);
    }
    quiet({ scanPts: pts });
  }, 100);

  sub('/tf_static', 'tf2_msgs/msg/TFMessage', m => {
    for (const t of m.transforms) {
      if (/laser|lidar|scan/i.test(t.child_frame_id)) {
        tfBaseLaser = { x: t.transform.translation.x, y: t.transform.translation.y,
                        yaw: quatToYaw(t.transform.rotation) };
      }
    }
  });

  sub('/local_costmap/costmap', 'nav_msgs/msg/OccupancyGrid', m => {
    window.dispatchEvent(new CustomEvent('map:costmap', { detail: m }));
  }, 1000);

  /* ── 속도 ─────────────────────────────────────────────────── */
  sub('/odometry/filtered', 'nav_msgs/msg/Odometry', m => {
    const v = m.twist.twist.linear;
    quiet({ speed: Math.hypot(v.x, v.y), fresh: { ...state.fresh, odom: Date.now() } });
  }, 200);

  /* ── 하드웨어 (펌웨어가 이미 뱉고 있는데 지금껏 화면이 안 보던 것들) ── */
  sub('/drive/enabled', 'std_msgs/msg/Bool', m => {
    if (m.data !== state.driveEnabled) {
      if (state.driveEnabled !== null) {
        pushLog('DRIVE', m.data ? '구동부 무장' : '구동부 무장 해제', '', m.data ? 'state' : 'warn');
      }
      update({ driveEnabled: m.data });
    }
  });

  sub('/drive/diag', 'geometry_msgs/msg/Vector3', m => {
    const d = { x: m.x, y: m.y, z: m.z }, p = state.driveDiag;
    quiet({ fresh: { ...state.fresh, drive: Date.now() } });
    if (!p || p.x !== d.x || p.y !== d.y || p.z !== d.z) {
      if (p) pushLog('DRIVE', '진단값 변화', `x=${d.x} y=${d.y} z=${d.z}`, 'warn');
      update({ driveDiag: d });
    }
  });

  sub('/estop/state', 'std_msgs/msg/Bool', m => {
    if (m.data !== state.estop) {
      if (state.estop !== null) pushLog('ESTOP', m.data ? '비상정지 눌림' : '비상정지 해제', '',
                                        m.data ? 'alarm' : 'state');
      update({ estop: m.data });
    }
  });

  sub('/firmware/info',  'std_msgs/msg/String', m => { if (m.data !== state.fwInfo) update({ fwInfo: m.data }); });
  sub('/firmware/event', 'std_msgs/msg/String', m => pushLog('FW', m.data, '', 'warn'));
  sub('/firmware/pulse', 'std_msgs/msg/String', () => quiet({ fwPulse: Date.now() }), 500);
  sub('/imu/yaw_deg',    'std_msgs/msg/Float64', m => quiet({ imuYaw: m.data,
                                fresh: { ...state.fresh, imu: Date.now() } }), 500);

  /* ── Nav2 ─────────────────────────────────────────────────── */
  sub('/navigate_to_pose/_action/status', 'action_msgs/msg/GoalStatusArray', m => {
    if (!m.status_list.length) return;
    const st = m.status_list[m.status_list.length - 1].status;
    if (st !== state.navStatus) {
      if (st === 6) pushLog('NAV2', '목표 거부·실패', '', 'alarm');
      update({ navStatus: st });
    }
  });

  /* ── 인지 (역할 B 결합 전까지는 오지 않는다 — 그것도 정보다) ── */
  const DET = ['fire', 'smoke', 'person_ok', 'person_fallen', 'person_unknown'];
  sub('/detections', 'tunnel_interfaces/msg/Detection3DArray', m => {
    const list = (m.detections || [])
      .filter(d => DET.includes(d.class_name))
      .map(d => ({ cls: d.class_name, conf: d.confidence }));
    update({ detections: list, detLastMs: Date.now() });
  }, 200);
  sub('/adapter_status', 'std_msgs/msg/String', m => quiet({ adapter: m.data }), 1000);

  /* 관제가 보낸 문구를 디스플레이 모드가 받아 띄운다. 관제 화면에서도 미리보기로 쓴다.
     ⚠ 자기가 보낸 것도 되돌아온다 — 그래야 '실제로 나갔다'가 화면에서 확인된다. */
  sub('/display_msg', 'std_msgs/msg/String', m => update({ sayText: m.data, sayAt: Date.now() }));

  /* ── 발행 준비 ────────────────────────────────────────────── */
  pubCmd    = new ROSLIB.Topic({ ros, name: '/mission_cmd', messageType: 'std_msgs/msg/String' });
  pubAlarm  = new ROSLIB.Topic({ ros, name: '/alarm',       messageType: 'geometry_msgs/msg/PoseStamped' });
  pubCmdVel = new ROSLIB.Topic({ ros, name: '/cmd_vel',     messageType: 'geometry_msgs/msg/Twist' });
  pubSay    = new ROSLIB.Topic({ ros, name: '/display_msg', messageType: 'std_msgs/msg/String' });

  /* ── 통신 지연 (⚠ rosapi_msgs — rosapi 가 아니다) ─────────── */
  if (rttTimer) clearInterval(rttTimer);        // ⚠ 재접속 시 타이머 누수 방지
  const svc = new ROSLIB.Service({ ros, name: '/rosapi/get_time',
                                   serviceType: 'rosapi_msgs/srv/GetTime' });
  rttTimer = setInterval(() => {
    const t0 = performance.now();
    svc.callService(new ROSLIB.ServiceRequest({}),
      () => quiet({ rttMs: Math.round(performance.now() - t0) }),
      () => quiet({ rttMs: null }));
  }, 2000);
}

/* ═══ 발행 (관제 → 로봇) ═════════════════════════════════════════ */

export function sendCmd(cmd) {
  if (!pubCmd) return false;
  pubCmd.publish(new ROSLIB.Message({ data: cmd }));
  pushLog('CMD', `관제 명령: ${cmd}`, '', 'ctrl');
  return true;
}

export function sendAlarm(x, y) {
  if (!pubAlarm) return false;
  const msg = new ROSLIB.Message({
    header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },   // stamp=0 = 최신 TF
    pose: { position: { x, y, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
  });
  pubAlarm.publish(msg);
  setTimeout(() => pubAlarm.publish(msg), 300);   // 간헐 유실 대비 재발사 (기존 관례)
  pushLog('CMD', '화재 경보 발행', `${x.toFixed(2)}  ${y.toFixed(2)}`, 'ctrl');
  return true;
}

export function sendTwist(lin, ang) {
  if (!pubCmdVel) return false;
  pubCmdVel.publish(new ROSLIB.Message({
    linear:  { x: lin, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: ang },
  }));
  return true;
}

/**
 * 로봇 디스플레이에 띄울 문구를 보낸다 (관제 → 현장 사람).
 * 🔴 `/display_msg` 를 받는 쪽은 아직 없다 — 디스플레이는 구동부팀이 설치했고
 *    연결은 미완이다(MASTER_PLAN §383). 그래서 화면에도 '로봇 미연결'로 정직하게 적는다.
 *    보냈다는 사실과 도착했다는 사실은 다르다.
 */
export function sendDisplay(text) {
  const t = String(text || '').trim().slice(0, 40);
  if (!t || !pubSay) return false;
  pubSay.publish(new ROSLIB.Message({ data: t }));
  pushLog('SAY', '디스플레이 문구 전송', t, 'ctrl');
  return true;
}

/** 소프트 비상정지 = abort + cmd_vel 0 연사.
    하드 정지는 Teensy 0.5초 워치독이 최후 안전장치다. */
export function softStop() {
  sendCmd('abort');
  let n = 0;
  const iv = setInterval(() => { sendTwist(0, 0); if (++n >= 10) clearInterval(iv); }, 100);
  pushLog('CMD', '비상 정지(소프트) — abort + cmd_vel 0 x10', '', 'alarm');
}
