/* ═══════════════════════════════════════════════════════════════════
   demo.js — 🎬 DEMO-0904 시연용 가짜값 **전부** 여기 모여 있다
   ═══════════════════════════════════════════════════════════════════

   🔴 촬영 후 원복: 이 파일을 지우고 import 세 줄을 빼면 끝이다.
      `grep -rn "DEMO-0904" console/` 로 전부 찾을 수 있다.

   ── 왜 있나 ────────────────────────────────────────────────────
   시연 영상은 **역할 B 인지·카메라가 결합 완료된 기준**으로 찍는다. 카메라 실사는
   따로 촬영해 편집에서 얹고, 화면의 수치는 그 장면에 맞는 값이 떠 있어야 한다.

   ── 지켜야 할 것 ────────────────────────────────────────────────
   🔴 **고정 상수를 쓰지 않는다.** 첫 판은 `사람 1 · 화재 1` 을 상수로 뒀는데,
      연결복도처럼 아무것도 안 보이는 구간에서도 그대로 떠서 **화면이 장면과
      어긋났다**(09-04 지적). 가짜값이라도 장면을 따라가야 한다.
   🔴 **실제 토픽이 오면 그쪽이 이긴다** — 역할 B 가 붙는 순간 이 파일은 무시된다.

   ── 근거로 쓰는 실데이터 ────────────────────────────────────────
   미션 상태 · 로봇 위치 · /alarm 좌표 · 구간 경과시간. 전부 bag 의 진짜 값이다.
   가짜인 것은 "그래서 카메라가 무엇을 봤을까" 를 그 위에 얹는 부분뿐이다.
   ═══════════════════════════════════════════════════════════════ */

import { update } from './state.js';
import { seedDwell } from './record.js';

export const DEMO_ON = true;

/* 부드럽게 흔들리는 값 — 난수를 쓰면 매 프레임 튀어서 화면이 지저분해진다 */
const wobble = (seed, periodSec, amp) =>
  Math.sin((Date.now() / 1000 / periodSec) + seed) * amp;

/* 미션 단계별로 "**앞을 보는 카메라**가 무엇을 보고 있을 상황인가" ──────
   🔴 카메라는 전방이고 라이다 추종감시는 **후방 부채꼴(±60°)** 이다
      (`follower_monitor.py` 머리말). 이 둘을 헷갈리면 화면이 거짓이 된다.

     사람 — 유도(GUIDE) 중에는 로봇이 **앞장서고 대피자가 뒤**를 따른다.
            그래서 **카메라에는 안 잡힌다 → 0.** 뒤를 보는 것은 라이다이고
            그 결과는 화면의 `대피자 추종` 에 따로 나온다.
            앞에 사람이 있는 상황은 집결 대기(GATHER)와 쓰러진 사람 발견(RESCUE)뿐이다.
            SEARCH_BACK 은 돌아서서 앞을 보지만 **아직 못 찾은 구간**이라 0 이다.
     화재 — 로봇이 화재 부근(위 복도)을 향하는 단계에서만 보인다.
            유도로 돌아서 연결복도·아래복도로 내려가면 벽에 가려 0.                */
const SCENE = {
  PATROL:      { person: 0, fire: 0 },
  APPROACH:    { person: 0, fire: 1 },   // 화재를 향해 간다
  SCAN_AREA:   { person: 0, fire: 1 },   // 제자리 회전하며 훑는다
  GATHER:      { person: 1, fire: 1 },   // 집결지 — 사람이 앞에 모인다
  GUIDE:       { person: 0, fire: 0 },   // 🔴 사람은 뒤 → 카메라엔 안 잡힌다
  HOLD:        { person: 0, fire: 0 },
  SEARCH_BACK: { person: 0, fire: 0 },   // 돌아섰지만 아직 못 찾았다
  RESCUE:      { person: 1, fire: 0 },   // 쓰러진 사람이 앞에 있다
  NO_VICTIM:   { person: 0, fire: 0 },
  ESCAPED:     { person: 0, fire: 0 },   // 탈출구를 향해 서 있다
  FAULT:       { person: 0, fire: 0 },
  BLOCKED:     { person: 0, fire: 0 },
};

/**
 * 인지 결과 — 실제 /detections 가 오면 그쪽을 그대로 쓴다.
 * @returns {{live:boolean, person:number, fire:number, conf:number, ageSec:number, adapter:string}}
 */
export function demoPerception(s) {
  if (s.detLastMs > 0) {                       // 실물이 이긴다
    return {
      live: true,
      person: s.detections.filter(d => d.cls.startsWith('person')).length,
      fire:   s.detections.filter(d => d.cls === 'fire' || d.cls === 'smoke').length,
      conf: Math.max(0, ...s.detections.map(d => d.conf)),
      ageSec: (Date.now() - s.detLastMs) / 1000,
      adapter: s.adapter || 'OK',
    };
  }
  if (!DEMO_ON || !s.mission) {
    return { live: false, person: 0, fire: 0, conf: 0, ageSec: null, adapter: null };
  }
  const sc = SCENE[s.mission] || { person: 0, fire: 0 };
  return {
    live: true,
    person: sc.person,
    fire: sc.fire,
    conf: 0.88 + wobble(1.7, 9, 0.06),          // 0.82~0.94
    ageSec: 0.1 + Math.abs(wobble(0.4, 3, 0.05)),
    adapter: `OK · 10.0 Hz`,
  };
}

/**
 * 전원·부하 — 실측 토픽이 없다. 임무 경과·주행 속도에 얹어 움직이게 한다.
 * 🔴 배터리는 `/battery_state` 합의 미완, Jetson 부하는 발행 노드가 없다.
 */
export function demoPower(s) {
  const min = s.phaseT0 ? (Date.now() - s.phaseT0) / 60000 : 0;
  const batt = Math.max(15, 82 - min * 0.9);            // 분당 0.9%
  const load = Math.min(1, s.speed / 0.2);              // 주행할수록 바쁘다
  const cpu  = clamp(36 + load * 22 + wobble(2.1, 5, 4), 5, 99);
  const gpu  = clamp(20 + (s.detLastMs || DEMO_ON ? 18 : 0) + wobble(3.3, 7, 5), 3, 99);
  const temp = clamp(46 + cpu * 0.22 + wobble(1.1, 13, 1.5), 30, 95);
  return {
    batt: Math.round(batt),
    volt: (23.0 + batt * 0.028).toFixed(1),             // 25.3V(만충) ~ 23.4V
    cpu: Math.round(cpu), gpu: Math.round(gpu), temp: Math.round(temp),
  };
}

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

/* ═══ 🎬 DEMO-0904 촬영용 자동 순회 ═══════════════════════════════
   `?tour=3` 을 붙이면 3초마다 메뉴를 스스로 넘긴다.

   왜 필요한가 — 촬영 중에 사람이 사이드바를 클릭하면 **마우스 커서와 클릭
   순간의 흔들림이 화면에 남고**, 컷 길이도 매번 달라져 편집이 어려워진다.
   화면이 스스로 넘어가면 커서가 안 나오고 컷 길이가 정확히 일정하다.

   순서는 시연 대본 그대로다: 관제 → 영상 → 진단 → 기록 → 비상.
   마지막 메뉴에서 멈춘다(반복하지 않는다 — 편집에서 뒤를 자르기 쉽게).      */
/* [메뉴, 기본 대기시간에 더할 초] — 관제 화면은 지도·미션·인지를 한 번에 보여주는
   첫 컷이라 읽을 것이 가장 많다. 다른 컷과 같은 길이면 눈이 못 따라간다. */
const TOUR = [
  ['main',      2],
  ['video',     0],
  ['diag',      0],
  ['record',    0],
  ['emergency', 0],
];

export function maybeStartTour(showMenu) {
  const q = new URLSearchParams(location.search).get('tour');
  if (q === null) return false;
  const hold = (parseFloat(q) || 3) * 1000;

  seedRecord();          // 기록 화면이 비어 있으면 "방금 켠 화면" 이 드러난다

  /* 컷마다 길이가 다르므로 setInterval 대신 한 컷씩 예약한다 */
  let i = 0;
  const step = () => {
    showMenu(TOUR[i][0]);
    const wait = hold + TOUR[i][1] * 1000;
    if (++i < TOUR.length) setTimeout(step, wait);
  };
  step();
  return true;
}

/* ═══ 🎬 DEMO-0904 기록 화면 앞부분 채우기 ═══════════════════════
   촬영은 bag 163초(연결통로 진입)부터 시작한다. 그 앞에서 실제로 일어난 일들이
   관제에는 없으니 **기록 화면이 미션 전이 몇 줄뿐**이라 "방금 켠 화면"이 드러난다.

   🔴 여기 값은 지어낸 것이 아니다. `realtake6` bag 의 `/mission_state` · `/siren` ·
      `/alarm` 을 rosbag2 로 직접 읽어 얻은 **실제 사건과 시각**이다.
      문구·태그·중복제거 규칙은 `ros.js` 가 실제로 쓰는 것과 똑같이 맞췄다 —
      다르면 이 줄만 티가 난다.

   🔴 `fireXY` 도 같이 심는다. `/alarm` 은 69초에 3번 발행되고 끝이라, 163초부터
      붙은 관제는 **화재 위치를 영영 모른다.** 지도에 화재 표시가 없는 것은
      버그가 아니라 늦게 붙었기 때문이고, 그 공백을 실제 좌표로 메운다.

   ⚠ `--at` 값을 바꾸면 START_AT 도 같이 바꾼다.                              */
const START_AT = 163.0;                      // run_console.sh --at 163 과 맞춘다
const FIRE_XY = { x: 12.50, y: -0.10 };      // bag /alarm 실측 좌표

/* [bag초, 태그, 문구, 값, 색] — 오래된 것부터 */
const PRE_LOG = [
  [68.5,  'STATE', 'PATROL → APPROACH',   '',                'state'],
  [68.5,  'SIREN', '싸이렌 ON',            '',                'warn'],
  [69.1,  'ALARM', '화재 감지',            '12.50  -0.10',    'alarm'],
  [89.4,  'STATE', 'APPROACH → SCAN_AREA', '',               'state'],
  [133.4, 'STATE', 'SCAN_AREA → GATHER',  '',                'state'],
  [145.9, 'STATE', 'GATHER → GUIDE',      '',                'state'],
];

/* 상태 전이 시각 (체류시간·타임라인이 쓴다) */
const PRE_STATES = [
  [12.4,  'PATROL'], [68.5, 'APPROACH'], [89.4, 'SCAN_AREA'],
  [133.4, 'GATHER'], [145.9, 'GUIDE'],
];

/** 촬영 시작 시점(= bag START_AT 초)에 관제가 갖고 있었을 상태를 만들어 준다. */
export function seedRecord() {
  const base = Date.now() - START_AT * 1000;   // bag 0초에 해당하는 벽시계 시각
  const at = sec => base + sec * 1000;

  const history = PRE_STATES.map(([sec, st]) => ({ state: st, at: at(sec) }));

  /* log.js 의 pushLog 는 최신을 앞에 넣는다 — 같은 순서로 만든다 */
  const logs = PRE_LOG.map(([sec, tag, msg, val, cls]) =>
    ({ t: at(sec), tag, msg, val, cls })).reverse();

  /* 상태별 체류시간 = 다음 전이까지의 실제 간격 */
  const dwell = {};
  for (let i = 0; i < PRE_STATES.length - 1; i++) {
    dwell[PRE_STATES[i][1]] = (PRE_STATES[i + 1][0] - PRE_STATES[i][0]) * 1000;
  }
  const [lastSec, lastState] = PRE_STATES[PRE_STATES.length - 1];
  seedDwell(dwell, lastState, at(lastSec));

  update({
    mission: lastState, missionSince: at(lastSec), history, logs,
    missionT0: at(PRE_STATES[0][0]),
    /* 🔴 ros.js 는 대응 경과를 **화재 감지 시점**부터 잰다. 같은 기준을 쓴다. */
    phaseT0: at(69.1), phaseDist0: 0,
    fireXY: FIRE_XY, siren: true,
  });
}
