/**
 * display.js — 로봇 몸통 디스플레이(?display=1)에 띄울 문구를 고른다 (2026-09-18)
 *
 * 왜 main.js 에서 떼어냈나: 디스플레이는 **켜두고 아무도 안 만지는** 물건이다.
 * rosbridge 가 죽거나 미션 노드가 멈춰도 마지막 문구("따라오세요")가 그대로 남으면
 * 대피자는 그 말을 믿고 따라간다. 그래서 "무엇을 보여줄지"를 순수 함수로 빼서
 * 브라우저 없이 회귀(`test/test_display.mjs`)로 잠근다.
 *
 * 우선순위 (위가 이긴다):
 *   ① 연결 끊김           — 무슨 말이든 이미 낡았다
 *   ② 관제가 보낸 문구     — 사람이 일부러 보낸 말 (SAY_HOLD_MS 동안)
 *   ③ 임무 상태가 낡음     — rosbridge 는 살았는데 미션 노드가 침묵
 *   ④ 임무 상태 문구       — DISPLAY_KO (대피자가 읽는 말)
 */
import { DISPLAY_KO } from './i18n.js';

export const SAY_HOLD_MS = 20000;      // 보낸 문구를 20초 띄운 뒤 상태 문구로 돌아간다
export const MISSION_STALE_MS = 5000;  // 미션은 2 Hz 발행(mission_node.py tick 0.5s) — 10회 연속 결번이면 낡은 것

/**
 * @param {object} s    state.js 의 state
 * @param {number} now  Date.now() — 테스트가 시간을 넣기 위해 인자로 받는다
 * @returns {{main: string, sub: string, cls: ''|'said'|'offline'}}
 *   cls 는 #big-state 에 붙는 클래스. 색은 전부 CSS 가 정한다.
 */
export function displayText(s, now = Date.now()) {
  if (!s.connected) {
    return s.everConnected
      ? { main: '연결 끊김', sub: '로봇과 다시 연결하는 중입니다', cls: 'offline' }
      : { main: '연결 중',   sub: '관제와 연결하고 있습니다',     cls: 'offline' };
  }
  if (s.sayText && s.sayAt && (now - s.sayAt < SAY_HOLD_MS)) {
    return { main: s.sayText, sub: '관제에서 보낸 안내입니다', cls: 'said' };
  }
  const t = s.fresh && s.fresh.mission;
  if (t && now - t > MISSION_STALE_MS) {
    return { main: '신호 없음', sub: '임무 상태를 받지 못하고 있습니다', cls: 'offline' };
  }
  const pair = DISPLAY_KO[s.mission];
  if (!pair) return { main: '연결 중', sub: '임무 상태를 기다리고 있습니다', cls: 'offline' };
  return { main: pair[0], sub: pair[1], cls: '' };
}
