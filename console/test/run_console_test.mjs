/* ═══════════════════════════════════════════════════════════════════
   run_console_test.mjs — 관제 화면 로직을 실데이터로 검증 (2026-09-02)

     ros2 bag play realtake6  →  rosbridge  →  (이 스크립트)  →  가짜 DOM
                                                     ↑
                                        브라우저와 같은 모듈을 그대로 import

   확인하는 것:
     ① 토픽이 state 에 제대로 꽂히는가
     ② 화면 모듈이 그 값을 사람이 읽을 문자열로 바꾸는가 (한글화·단위·정렬)
     ③ 경보 규칙이 올바로 켜지고 꺼지는가
     ④ HTML 에 없는 id 를 건드리지 않는가

   실행: node console/test/run_console_test.mjs [관찰초]
   ═══════════════════════════════════════════════════════════════════ */

import { document, window, elements, missingIds, getComputedStyle } from './dom_stub.mjs';
import { makeROSLIB } from './roslib_shim.mjs';

globalThis.document = document;
globalThis.window = window;
globalThis.getComputedStyle = getComputedStyle;
globalThis.performance = globalThis.performance ?? { now: () => Date.now() };
globalThis.ROSLIB = makeROSLIB();

const { state, update, startTicker } = await import('../js/state.js');
const { hms } = await import('../js/i18n.js');
const { connect } = await import('../js/ros.js');
const { setupMap, draw } = await import('../js/map.js');
const { setupMission } = await import('../js/mission.js');
const { setupDiag } = await import('../js/diag.js');
const { setupRecord } = await import('../js/record.js');
const { setupEmergency } = await import('../js/emergency.js');
const { evaluateAlerts, healthSummary } = await import('../js/alert.js');

setupMap(); setupMission(); setupDiag(); setupRecord(); setupEmergency();
connect();
setInterval(evaluateAlerts, 500);
setInterval(draw, 400);
startTicker(4);

const SECONDS = Number(process.argv[2] || 25);
const seen = { states: new Set(), alerts: new Set() };
const t0 = Date.now();

const tick = setInterval(() => {
  if (state.mission) seen.states.add(state.mission);
  state.alerts.forEach(a => seen.alerts.add(a.id));
}, 200);

setTimeout(() => {
  clearInterval(tick);
  const el = id => elements.get(id);
  const g = id => { const e = el(id); if (!e) return '(없음)';
                    return (e.textContent || (e.innerHTML || '').replace(/<[^>]+>/g, '')) || '(빈값)'; };
  const line = (k, v) => console.log(`  ${k.padEnd(22)} ${v}`);

  console.log(`\n${'═'.repeat(72)}\n  관제 화면 검증 — ${SECONDS}초 관찰 (bag realtake6 재생분)\n${'═'.repeat(72)}`);

  console.log('\n[ 상단바 ]');
  const h = healthSummary(state);
  line('신호등', `${h.cls || '(회색)'}  ${h.text}`);
  line('경보 개수', String(state.alerts.length));

  console.log('\n[ 관제 화면 — 임무 ]');
  line('상태(한글)', g('state-ko'));
  line('상태(코드)', g('state-en'));
  line('이 상태 유지', g('state-since'));
  line('임무 경과', g('mission-elapsed'));
  line('이동 거리', g('mission-dist'));
  line('현재 속도', g('mission-speed'));
  line('싸이렌', g('siren-v'));
  line('대피자 판정', g('person-v'));
  line('목표까지', g('mission-togo'));
  line('항법', g('mission-nav'));

  console.log('\n[ 관제 화면 — 인지 ]');
  line('사람', g('det-person'));
  line('화재·연기', g('det-fire'));
  line('출처', g('det-src'));

  console.log('\n[ 진단 화면 ]');
  update({ menu: 'diag' });   // 진단은 menu 가 diag 일 때만 그린다
  for (const [k, id] of [['구동부 무장', 'd-drive'], ['비상정지', 'd-estop'], ['진단값', 'd-diag'],
                         ['펌웨어', 'd-fw'], ['하트비트', 'd-pulse'], ['라이다', 'd-scan'],
                         ['오도메트리', 'd-odom'], ['IMU', 'd-imu'], ['/tf', 'd-tf'],
                         ['Nav2', 'd-nav'], ['rosbridge', 'd-conn'], ['위치 x y θ', 'd-pose'],
                         ['배터리(MOCK)', 'd-batt'], ['Jetson(MOCK)', 'd-jetson']]) line(k, g(id));

  console.log('\n[ 기록 화면 ]');
  update({ menu: 'record' });
  for (const [k, id] of [['임무 경과', 'r-elapsed'], ['이동 거리', 'r-dist'],
                         ['이벤트', 'r-events'], ['경보', 'r-alerts'], ['임무 단계', 'r-states']])
    line(k, (elements.get(id)?.innerHTML || '').replace(/<[^>]+>/g, ' '));

  console.log('\n[ 지도 데이터 ]');
  line('지도', state.mapInfo ? `${state.mapInfo.width}x${state.mapInfo.height} @ ${state.mapInfo.resolution.toFixed(3)}` : '미수신');
  line('로봇 위치', state.robot ? `${state.robot.x.toFixed(2)}  ${state.robot.y.toFixed(2)}  ${(state.robot.yaw*180/Math.PI).toFixed(0)}°` : '미수신');
  line('라이다 점군', `${state.scanPts.length} 점`);
  line('지나온 길', `${state.trail.length} 점`);
  line('계획 경로', `${state.planPts.length} 점`);
  line('화재 좌표', state.fireXY ? `${state.fireXY.x.toFixed(2)}  ${state.fireXY.y.toFixed(2)}` : '없음');

  console.log('\n[ 관찰 중 거친 상태 ]');
  console.log('  ' + ([...seen.states].join(' → ') || '(없음)'));
  console.log('\n[ 발생한 경보 ]');
  console.log('  ' + ([...seen.alerts].join(', ') || '(없음)'));

  console.log('\n[ 최근 이벤트 로그 ]');
  state.logs.slice(0, 8).forEach(l => {
    console.log(`  ${hms(l.t)}  ${l.tag.padEnd(7)} ${l.msg.padEnd(34)} ${l.val}`);
  });

  const miss = missingIds();
  console.log(`\n${'═'.repeat(72)}`);
  console.log(miss.length ? `🔴 HTML 에 없는 id 를 건드렸다: ${miss.join(', ')}`
                          : '🟢 HTML 에 없는 id 접근 없음');
  console.log(`🟢 수신 토픽 종류: ${Object.keys(state.fresh).length}`);
  process.exit(miss.length ? 1 : 0);
}, SECONDS * 1000);
