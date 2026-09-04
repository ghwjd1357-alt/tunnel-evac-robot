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
