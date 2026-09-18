/* ═══════════════════════════════════════════════════════════════════
   i18n.js — 코드의 말 → 사람의 말 (2026-09-02)

   🔴 심사위원·평가자는 `SEARCH_BACK` 을 모른다. 화면에는 한글을 크게,
      영문(코드값)은 작게 병기한다. 그래야 처음 보는 사람도 읽고,
      개발자는 코드와 대조할 수 있다.
   ═══════════════════════════════════════════════════════════════════ */

/* 미션 상태 12종 — 정본 = src/mission_manager/mission_manager/mission_node.py State */
export const STATE_KO = {
  PATROL:      '순찰 중',
  APPROACH:    '화재 지점으로 출동',
  GATHER:      '대피자 집결 대기',
  SCAN_AREA:   '주변 수색 중',
  GUIDE:       '대피 유도 중',
  HOLD:        '정지 · 대피자 재확인',
  SEARCH_BACK: '놓친 대피자 재탐색',
  RESCUE:      '쓰러진 대피자 발견 — 신고',
  NO_VICTIM:   '대피자 없음 확인',
  ESCAPED:     '대피 완료',
  FAULT:       '주행 실패 — 정지',
  BLOCKED:     '사람의 판단이 필요합니다',
};

/* 진행 막대 6단계 — 12개 상태를 사람이 이해하는 흐름으로 묶는다.
   묶음 밖(예외)인 상태는 막대에 칠하지 않고 별도 경고로 띄운다. */
export const STAGES = [
  { key: 'patrol',  label: '순찰',  states: ['PATROL'] },
  { key: 'go',      label: '출동',  states: ['APPROACH'] },
  { key: 'scan',    label: '수색',  states: ['SCAN_AREA'] },
  { key: 'gather',  label: '집결',  states: ['GATHER'] },
  { key: 'guide',   label: '유도',  states: ['GUIDE', 'HOLD', 'SEARCH_BACK'] },
  { key: 'done',    label: '완료',  states: ['ESCAPED'] },
];

/** 막대 밖의 예외 상태 — 화면 위쪽에 빨갛게 따로 띄운다 */
export const EXCEPTION_STATES = {
  FAULT:     '주행 실패 — 정지했습니다',
  BLOCKED:   '사람의 판단이 필요합니다',
  RESCUE:    '쓰러진 대피자 발견 — 신고',
  NO_VICTIM: '집결지에 대피자가 없습니다',
};

/* 🔴 관제 모드 — 평시와 비상의 화면이 같으면 사람이 상황을 못 읽는다.
   실제 관제 시스템은 상황에 따라 화면 자체가 바뀐다. 상태 12종을 세 모드로 묶는다.
     normal   평시 감시     — 무채색. 조용해야 경보가 눈에 띈다
     incident 화재 대응 중  — 사건 진행. 상단에 경고 띠
     critical 즉시 개입 필요 — 사람이 안 누르면 진행이 멈추는 상태 */
export const MODE_OF = {
  PATROL: 'normal',  ESCAPED: 'normal',
  APPROACH: 'incident', SCAN_AREA: 'incident', GATHER: 'incident',
  GUIDE: 'incident', HOLD: 'incident', SEARCH_BACK: 'incident',
  FAULT: 'critical', BLOCKED: 'critical', RESCUE: 'critical', NO_VICTIM: 'critical',
};

export const MODE_KO = {
  normal:   '평시 감시',
  incident: '화재 대응 중',
  critical: '즉시 개입 필요',
};

/* 즉시 개입이 필요한 상태에서 **사람이 무엇을 해야 하는지** 한 줄로 알려준다.
   상태 이름만 빨갛게 띄우고 끝내면, 처음 보는 사람은 뭘 눌러야 할지 모른다. */
export const ACTION_HINT = {
  BLOCKED:   '안전한 집결지를 만들지 못했습니다. 임무를 재시작해야 진행됩니다.',
  FAULT:     '주행에 실패해 로봇이 멈췄습니다. 로봇 상태를 확인한 뒤 재시작하세요.',
  RESCUE:    '쓰러진 대피자를 발견했습니다. 구조 인력이 필요합니다.',
  NO_VICTIM: '집결지에 대피자가 없습니다. 임무를 종료하거나 재시작하세요.',
};

/* 🔴 로봇 몸통 디스플레이(7인치 1024x600)에 띄우는 문구.
   보는 사람이 **대피자·현장 인원**이라 관제 화면과 말이 달라야 한다.
   관제는 'GUIDE 대피 유도 중'이지만, 대피자에게 필요한 말은 '따라오세요'다.
   원안 = feature/display (팀원 3ddef79) — 그 사상을 그대로 옮겼다. */
export const DISPLAY_KO = {
  PATROL:      ['순찰 중',           '평상시 순찰하고 있습니다'],
  APPROACH:    ['화재 확인 중',      '화재 지점으로 이동합니다'],
  SCAN_AREA:   ['주변을 살피는 중',  '사람이 있는지 확인합니다'],
  GATHER:      ['여기서 기다리세요', '잠시 후 안내를 시작합니다'],
  GUIDE:       ['따라오세요',        '탈출구로 안내합니다'],
  HOLD:        ['잠시 멈춥니다',     '뒤따라오는지 확인합니다'],
  SEARCH_BACK: ['되돌아갑니다',      '놓친 분을 찾고 있습니다'],
  RESCUE:      ['구조가 필요합니다', '구조대에 신고했습니다'],
  NO_VICTIM:   ['대기 중',           '주변에 사람이 없습니다'],
  ESCAPED:     ['대피 완료',         '안전한 곳에 도착했습니다'],
  FAULT:       ['정지',              '관제에 연락했습니다'],
  BLOCKED:     ['정지',              '관제 확인을 기다립니다'],
};

/* /drive/diag (geometry_msgs/Vector3) 해독표.
   계약 정본 = firmware/teensy_integrated_base_v1_4.ino:1456 · rearm_gate.h:59
     x = /drive/enable 서비스 호출 누계
     y = 거절·해제 사유 (DriveReject)
     z = 구동 상태 (DriveState)
   🔴 z=2(ARMED) 에서만 모터가 돈다. 숫자로 두면 아무도 못 읽으므로 말로 바꾼다. */
export const DRIVE_STATE_KO = {
  0: '해제됨',
  1: '대기 (무장 가능)',
  2: '무장 · 모터 동작 가능',
  3: '무장 대기',
  4: '무장 처리 중',
};

export const DRIVE_REJECT_KO = {
  0: '없음',
  1: 'E-stop 이 눌려 있음',
  2: '정지 확인 미충족',
  3: '이미 무장 중',
  4: 'E-stop 이 무장을 해제함',
  5: '잘못된 속도 명령 (비유한값)',
  6: '무장 전에 이동 명령이 들어옴',
  7: '펌웨어 실행시간 초과',
  8: '무장 응답 전송 실패',
};

/* ═══ DEMO-0904 ═══ 촬영 후 원복 검토 대상 (console/README.md '시연용 임시 변경')
   대피자 추종 상태를 **미션 상태에서 유도**한다.

   🔵 근거 — 거짓말이 아니다. SEARCH_BACK 은 추종감시가 lost() 를 선언해서 들어간
      상태이고, HOLD 는 그 직전 '놓침 확정 직후 제자리 재수집'이다. 즉 미션 상태
      자체가 "놓쳤다"의 증거다 (mission_node.py State 주석).
   🔴 그래도 이것은 **센서 토픽이 아니라 상태에서 뒤집어 읽은 값**이다.
      진짜 판정값(FollowerMonitor.visible/lost)은 미션 노드 안에만 있고 토픽으로
      안 나온다 — /mission_status 가 생기면 그걸 직접 쓰고 이 표는 지운다.
   🔴 08-23 bag 의 /person_status 는 1,651건 전부 'ok' 인 **상수 스텁**이라 못 쓴다. */
export const FOLLOWER_OF = {
  GUIDE:       ['감지', 'ok'],
  ESCAPED:     ['감지', 'ok'],
  HOLD:        ['미탐지', 'alarm'],
  SEARCH_BACK: ['미탐지', 'alarm'],
};

export const NAV_KO = {
  1: '목표 접수', 2: '주행 중', 3: '취소 중',
  4: '도달', 5: '취소됨', 6: '거부·실패',
};

export const PERSON_KO = {
  ok: '정상', fallen: '쓰러짐', unknown: '판별 불가', stale: '최근 관측 없음',
};

/**
 * 큰 숫자 칸용 시계 표기 — 항상 `MM:SS` (한 시간 넘으면 `H:MM:SS`).
 * 🔴 "53초" → "1분 03초" 로 글자 수가 늘면 큰 글씨 칸을 넘친다(09-04 확인).
 *    자리수가 고정돼야 값이 바뀌어도 화면이 안 흔들린다.
 */
export function clock(sec) {
  if (sec == null || !isFinite(sec)) return '--:--';
  const s = Math.max(0, Math.floor(sec));
  const p2 = n => String(n).padStart(2, '0');
  return s < 3600 ? `${p2((s / 60) | 0)}:${p2(s % 60)}`
                  : `${(s / 3600) | 0}:${p2(((s % 3600) / 60) | 0)}:${p2(s % 60)}`;
}

/** 초 → "4분 12초" (목록·타임라인처럼 자리가 넉넉한 곳) */
export function dur(sec) {
  if (sec == null || !isFinite(sec)) return '—';
  const s = Math.max(0, Math.floor(sec));
  return s < 60 ? `${s}초` : `${Math.floor(s / 60)}분 ${String(s % 60).padStart(2, '0')}초`;
}

/** 시각(ms) → "12:04:31"
    🔴 toLocaleTimeString('ko-KR') 은 환경에 따라 "17시 18분 15초" 를 낸다.
       로그는 열 정렬이 생명이라 자리수가 흔들리면 안 된다 → 직접 만든다. */
export function hms(ms) {
  const d = new Date(ms), p2 = n => String(n).padStart(2, '0');
  return `${p2(d.getHours())}:${p2(d.getMinutes())}:${p2(d.getSeconds())}`;
}
