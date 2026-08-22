
# 🔴 2026-08-22 — 독립 검토 §90 이 남긴 **미수정** 발견 넷. 사슬은 4회차에서 조건부
#   동결됐고(`CURRENT_HANDOFF` §검토 §87~§90), 아래는 **고쳐진 것이 아니라 실행 경로
#   밖에 둔 것**이다. 재개 조건도 그 절에 있다. 인벤토리 = `tools/ai_known_p0_p1.json`.
#     90.1 P0  취소를 못 보낸 채 FAULT 로 넘어가면 살아 있는 goal 을 끌고 간다
#     90.2 P0  응답 등록이 실패한 goal 은 미지 상태로 남고 다음 goal 이 허용된다
#     90.3 P1  하한·문자열 개수 회귀는 새 진입점 누락을 통과시킨다
#     90.4 P1  IMU 기준 감시도 저발행률과 곡선 전환에서 판정 공백이 남는다

# -*- coding: utf-8 -*-
"""
goal_manager.py — Nav2 goal 전송·응답·취소·최종결과의 비동기 수명주기 전담 (07-23 구조 분리 2/3)
============================================================

[왜 분리했나 — 마스터플랜 §2-2 "최대 수익"]
  goal 하나의 일생은 "전송 → 수락응답 → (주행) → 최종결과", 그리고 중간에
  끼어드는 "취소 → 취소접수 → CANCELED 종결"까지, 전부 **시간차를 두고 따로
  도착하는 비동기 콜백**이다. 여기에 세대(generation) 토큰으로 stale(낡은)
  콜백을 걸러내는 레이스 방어가 얽혀 있어, MissionNode(상태머신) 안에 두면
  상태 전이 코드와 콜백 타이밍 코드가 뒤엉킨다.
  → MissionNode 에는 "어떤 상태에서 어디로 갈지"(정책)와 도착/실패 콜백만 남기고,
    수명주기 전부를 이 Manager 가 소유한다. (SpeedManager 와 같은 구조.)

[핵심 설계 — generation(세대) 토큰]
  send_goal / cancel 마다 세대 번호(_seq)를 +1 한다. 응답·결과 콜백은 자기 세대
  (seq)와 현재 세대(_seq)를 비교해 stale 을 가려낸다:
    - 현재 세대 == : 정상 처리 (수락→핸들 보관, 결과→도착/FAULT)
    - stale(과거 세대) : "무시"가 아니라 **뒤늦게 수락됐으면 즉시 취소**하고
      최종 결과(CANCELED 여야 정상)까지 감시한다. "취소했다"≠"로봇이 멈췄다".

[호출 ≠ 접수 ≠ 종결 ≠ 실효 (AGENTS.md §3-3)]
  cancel 은 cancel_goal_async 를 '부른 것'으로 끝내지 않는다. 취소 응답의
  goals_canceling 을 확인(접수)하고, 최종 status 가 CANCELED 인지까지 관찰(종결)한다.

[★ B: GUIDE 저속 상실 취소의 '종결 직렬화' (07-23 이 묶음 신규 — 핸드오프 완료조건 5)]
  주행 중 저속(0.12)을 끝내 못 지키게 되면(SpeedManager 복구 소진) 상위 정책이
  goal 취소 + FAULT 를 결정한다(§22.3). 이때 로봇은 아직 '옛 목표'로 움직이는
  중일 수 있으므로, **그 취소가 CANCELED 로 최종 종결되기 전에는 저속 0.12 가
  다시 확인돼도 새 goal 을 절대 보내지 않는다**(직렬화 = 먼저 확실히 멈추고 나서
  다음 명령). 취소가 끝내 실패/지연되면(호출·응답 예외, 빈 goals_canceling,
  대기 예산 소진, CANCELED 아닌 종결) FAULT + 재전송 금지로 굳힌다.
    - 진입: cancel_current_goal(intent='guide_stop') → _stop_pending=True. 대상은 수락된
      핸들뿐 아니라 '수락응답 대기 세대'(_response_pending_seq)도 포함 — 서버가 이미 수락해
      주행 중인데 응답만 늦은 goal 을 놓치지 않는다(0723검토 §2 P1 보완).
    - 해제: 그 goal(대상 seq)이 CANCELED 로 종결 관찰 → _stop_pending=False (신규 허용).
      대상이 끝내 미수락(거부)이면 실제 주행 goal 이 없으니 정상 해제.
    - 실패(취소 호출/응답 예외·빈 goals_canceling·비CANCELED 종결·응답/결과 예외·예산
      소진) → on_fault + 봉쇄 유지. 모든 B 콜백은 대상 seq 에 귀속(옛 세대 오판 방지).
    - hard(reset/abort): intent='hard' → 직렬화 강제 해제(운영자 개입이 최우선)

[MissionNode 와의 이음새 — 콜백 3개 + 얇은 위임]
  생성: GoalManager(action_client, logger, on_reached, on_fault, on_active)
    on_reached()  = 목표 도착(SUCCEEDED). MissionNode 가 상태 전이.
    on_fault()    = 거부/실패/예외/취소 종결 실패. MissionNode 가 FAULT.
    on_active(b)  = goal 진행중 플래그를 MissionNode.goal_active 로 미러(정책 입력).
  MissionNode 는 send_goal / cancel_current_goal 을 이 Manager 로 위임한다.
"""

import math
from functools import partial

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


def yaw_to_quat(yaw):
    """yaw(각도 1개) → quaternion. 평면 로봇이라 z,w 만 유효.
    (구 mission_node.py 에 있던 순수 함수 — goal 좌표 조립 전용이라 여기로 이동.)"""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class GoalManager:
    """Nav2 navigate_to_pose goal 의 전송·수락·취소확인·결과감시 전담."""

    # ★ B(유도정지 취소 종결 직렬화)의 '대기 예산' — CANCELED 종결을 기다리며 신규
    #   goal 을 보류할 수 있는 최대 '보내려는 시도' 횟수. MissionNode.tick(0.5초)마다
    #   유도 재개를 시도하므로 60회 ≈ 30초 (SpeedManager 의 30초 timeout 과 같은 감각).
    #   이 예산을 넘도록 취소가 CANCELED 로 안 끝나면 정지 확인 불가 → FAULT 로
    #   가시화한다(무기한 '멈춘 척 유도활성'으로 고장을 숨기지 않기 — §13 교훈).
    #   ⚠ 벽시계 대신 '시도 횟수'로 재는 이유: MissionNode.tick 이 껍데기 speed
    #   테스트 하네스(매니저 미배선)와 공유돼 Manager.tick 을 못 부른다 — 그래서
    #   주기 tick 없이 소비 지점(send_goal)에서만 예산을 센다.
    CANCEL_STOP_MAX_BLOCKS = 60

    def __init__(self, action_client, logger, *,
                 on_reached, on_fault, on_active,
                 on_stop_unconfirmed=None, on_stop_confirmed=None):
        """action_client = NavigateToPose ActionClient (노드가 만들어 줌).
        on_reached() / on_fault() / on_active(bool) = 노드의 정책 콜백.
        (SpeedManager 와 달리 시계가 필요 없다 — 예산이 '시도 횟수' 기반이라
         순수 이벤트/카운터로만 결정된다.)"""
        self._nav = action_client
        self._log = logger
        self._on_reached = on_reached
        self._on_fault = on_fault
        self._on_active = on_active
        # §84.2 — 안전정지 종결 확인 실패 신호 (없으면 FAULT 로 떨어진다).
        self._on_stop_unconfirmed = on_stop_unconfirmed or (lambda reason: None)
        # §84.2 — 안전정지 CANCELED 종결 확인 신호.
        self._on_stop_confirmed = on_stop_confirmed or (lambda: None)

        # --- 세대 토큰 + 현재 goal ---
        self._seq = 0             # send_goal/cancel 마다 +1 (stale 판별 기준)
        self._handle = None       # 현재 수락된 goal 핸들 (취소 대상)
        self._active = False      # goal 이 진행 중인가 (노드 정책 입력 = goal_active)
        # ★ P1 보완(0723검토 §2): 요청은 나갔지만 수락/거부 응답이 아직 안 온 세대.
        #   `_handle is None` 하나로는 "나간 goal 없음"과 "수락응답 대기 중"(서버는
        #   이미 주행 중일 수 있음)을 구분 못 해 B 직렬화가 접수 전환 순간에 새 나갔다.
        self._response_pending_seq = None

        # --- B: 유도정지 취소 종결 직렬화 ---
        self._stop_pending = False   # CANCELED 종결 대기 중 — 신규 goal 전면 보류
        self._stop_seq = None        # 종결을 기다리는 그 goal 의 세대
        self._stop_blocks = 0        # 보류 중 '보내려던 시도' 누적 (예산 소진 판정)
        # 🔴 08-21 §84.2 — 정지 직렬화를 **누가 걸었는지**를 기억한다.
        #   'guide_stop' 실패는 FAULT(자동 재시도 경로)로 가지만,
        #   'safety_stop'(S1-3 안전 거부) 실패는 **재시도할 대상이 없다** —
        #   사람이 판단해야 하므로 별도 신호로 올린다.
        self._stop_intent = None

    # ===========================================================
    # 공개 API — MissionNode 가 위임하는 2개
    # ===========================================================
    @property
    def active(self):
        """goal 이 진행 중인가 — MissionNode.goal_active 미러의 원본(live)."""
        return self._active

    def send_goal(self, wp, tag='', state_name=''):
        """목표 전송. wp = {'x','y','yaw'} (yaw 선택). state_name 은 로그용.

        ★ B: 유도정지 취소가 아직 CANCELED 로 안 끝났으면(_stop_pending) 저속이
        확인됐든 말든 신규 goal 을 보내지 않는다. 대기 예산까지 소진하면 FAULT."""
        if self._stop_pending:
            self._stop_blocks += 1
            if self._stop_blocks >= self.CANCEL_STOP_MAX_BLOCKS:
                self._log.error(
                    f'★ 정지 취소가 {self.CANCEL_STOP_MAX_BLOCKS} tick 내 '
                    f'CANCELED 종결 실패 — 정지 확인 불가 (재전송 금지)')
                # §84.2 — 의도별 귀속. safety_stop 은 FAULT 자동복귀로 안 보낸다.
                self._stop_failed(
                    f'{self.CANCEL_STOP_MAX_BLOCKS} tick 내 종결 실패')
            else:
                self._log.warn(
                    '⚠ 유도정지 취소 종결 대기 — 신규 goal 보류',
                    throttle_duration_sec=5.0)
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        # stamp=0 유지 — "최신 TF 사용" (PITFALLS §6 · 0705_현황 §12.2)
        goal.pose.pose.position.x = float(wp['x'])
        goal.pose.pose.position.y = float(wp['y'])
        _, _, qz, qw = yaw_to_quat(float(wp.get('yaw', 0.0)))
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        if not self._nav.server_is_ready():        # 블로킹 금지 (콜백 싱글스레드)
            self._log.warn('Nav2 액션서버 아직 없음',
                           throttle_duration_sec=5.0)
            return

        self._seq += 1
        seq = self._seq
        self._set(True)
        self._response_pending_seq = seq   # 수락/거부 응답 도착 전까지 '대기'로 표시
        self._log.info(
            f'[{state_name}] 목표전송 {tag} → ({wp["x"]:.1f}, {wp["y"]:.1f})')
        if not self._watch(lambda: self._nav.send_goal_async(goal),
                           partial(self._on_goal_response, seq),
                           f'목표전송(seq={seq})'):
            # 🔴 §89.3 — 등록이 실패했으면 **응답을 영원히 못 듣는다.** 요청이 이미
            #   나갔을 수 있는데 핸들이 없어 취소할 대상도 없다. 미러를 정리하고
            #   그 사실을 크게 남긴다 — 안 그러면 `active=True` 로 굳어 다음 goal 도 막힌다.
            self._clear_response_pending(seq)
            self._set(False)
            self._log.error(
                f'🔴 목표전송(seq={seq}) 응답 등록 실패 — 요청이 이미 나갔다면 '
                f'**추적 불가한 goal 이 남는다.** 물리 정지를 확인할 것')

    def cancel_current_goal(self, intent=None):
        """현재 goal 취소. intent 로 상위 의도를 구분한다:
          None        일반 취소(알람 재지정·재발견 복귀 등) — B 직렬화 상태는 건드리지 않음
          'guide_stop' 저속 상실에 의한 유도정지 — CANCELED 종결까지 신규 goal 봉쇄(B)
          'hard'      운영자 reset/abort — 진행 중 B 직렬화를 강제 해제(개입 최우선)

        세대를 올려(이전 응답을 stale 화) 취소 레이스를 닫고, 핸들이 있으면
        cancel_goal_async 의 접수·종결까지 확인한다. 핸들이 없으면(아직 미수락·
        이미 종결) 조용히 통과하되 goal_active 는 남기지 않는다."""
        stopped_seq = self._seq
        self._seq += 1

        if intent in ('guide_stop', 'safety_stop'):
            # ★ P1 보완(§2): 멈출 대상은 '수락된 핸들'만이 아니다. 요청은 나갔지만
            #   수락 응답만 늦는 goal(_response_pending_seq)도 서버에선 이미 주행
            #   중일 수 있다 — 그 세대도 B 정지 대상으로 무장한다. 요청조차 없고
            #   핸들도 없으면(진짜 멈출 대상 없음) 조용히 통과.
            if (self._handle is not None
                    or self._response_pending_seq == stopped_seq):
                self._stop_pending = True
                self._stop_seq = stopped_seq
                self._stop_blocks = 0
                self._stop_intent = intent
        elif intent == 'hard':
            self._clear_stop()

        if self._handle is not None:
            # 핸들이 있으면 지금 취소. 핸들이 아직 없는(응답 대기) B 대상은 늦은 수락
            # 경로(_on_goal_response stale)에서 취소·감시가 이어진다.
            self._cancel_with_confirm(
                self._handle, f'현재 목표(seq={stopped_seq})',
                stop_seq=(stopped_seq
                          if intent in ('guide_stop', 'safety_stop') else None))
            self._handle = None
        self._set(False)

    # ===========================================================
    # 내부 — 상태 변경 헬퍼
    # ===========================================================
    def _set(self, active):
        """goal_active 원본 갱신 + 노드 미러 콜백. 모든 active 변경은 여기로."""
        self._active = active
        self._on_active(active)

    def _clear_stop(self):
        """B 직렬화 해제 — CANCELED 종결 관찰 또는 hard(reset/abort) 때."""
        self._stop_pending = False
        self._stop_seq = None
        self._stop_blocks = 0
        self._stop_intent = None

    @property
    def stop_pending(self):
        """§84.2 — 정지 종결을 아직 기다리는가 (미션이 읽는다)."""
        return self._stop_pending

    #: 🔴 §85.3 **패턴 수정** — 정지 실패 출구의 **개수를 계약으로 고정**한다.
    #:   §84.2 는 출구 3개만 바꾸고 2개를 놓쳤다. 손으로 세면 또 놓친다.
    #:   `tools/test_stop_exits.py` 가 이 값과 실제 `_stop_failed(` 호출 수를 대조한다.
    STOP_FAILURE_EXITS = 8   # 08-22 §88.2 — _watch 공통 경계가 하나 늘었다

    def _stop_failed(self, reason):
        """🔴 §84.2 — 정지 확인 실패의 귀속처를 **의도로** 가른다.

        `guide_stop` — 저속 상실에 의한 유도정지. FAULT 로 올려 기존 재시도
                       정책에 맡긴다(그 경로는 재시도가 의미 있다).
        `safety_stop` — S1-3 안전 거부. **재시도할 대상이 자체가 없다** —
                       안전한 집결지가 없어서 멈춘 것이므로 다시 보낼 goal 이
                       없다. FAULT 자동복귀와 섞으면 로봇이 스스로 재개한다.
                       → 별도 신호로 사람에게 올린다. `_stop_pending` 은 유지.
        """
        if self._stop_intent == 'safety_stop':
            self._on_stop_unconfirmed(reason)
        else:
            self._on_fault()

    def _clear_response_pending(self, seq):
        """자기 seq 의 '수락응답 대기' 표시만 비운다 — 오래된 응답이 새 요청의
        pending 을 지우지 않게 한다(0723검토 §5 권장 3)."""
        if self._response_pending_seq == seq:
            self._response_pending_seq = None

    def _is_stop_target(self, seq):
        """seq 가 지금 B 직렬화가 종결을 기다리는 그 goal 인가 (live).
        모든 B 콜백을 이 술어로 귀속시켜, hard 해제 뒤 도착한 옛 세대 콜백이
        새 B 세대를 false FAULT 시키지 않게 한다(0723검토 §5 권장 8)."""
        return self._stop_pending and seq == self._stop_seq

    def _judge_stop_terminal(self, seq, status):
        """B 대상 goal 의 최종 status 판정 — B 대상이면 처리하고 True 반환.
        CANCELED 만 직렬화 해제, 나머지는 FAULT + 봉쇄 유지(재전송 금지).
        수락된 핸들 경로(_observe_stale_terminal)와 늦은 수락 경로(_on_stale_result)가
        같은 판정으로 합류한다(0723검토 §5 권장 5)."""
        if not self._is_stop_target(seq):
            return False
        if status == GoalStatus.STATUS_CANCELED:
            self._log.info(
                f'정지 취소 CANCELED 종결 확인(seq={seq}) — 신규 goal 허용')
            safety = self._stop_intent == 'safety_stop'
            self._clear_stop()
            if safety:
                self._on_stop_confirmed()      # §84.2 — 정지 완료를 미션에 알린다
        else:
            self._log.error(
                f'★ 정지 취소가 CANCELED 아닌 status={status} 로 '
                f'종결(seq={seq}) — 정지 확인 불가 (재전송 금지)')
            self._stop_failed(f'status={status} 종결 (seq={seq})')
        return True

    def _stop_target_terminal_lost(self, seq):
        """B 대상 goal 의 최종결과 수신 자체가 실패(Future 예외) — 정지 확인 불가
        → FAULT + 봉쇄 유지. 반환 True = B 대상이라 처리함.

        ★ 0723검토 §6.3 P1: accepted-handle(_on_result 예외)·늦은 수락
        (_on_stale_result 예외) 두 경로가 이 helper 로 같은 정책을 공유한다 —
        성공 경로는 _judge_stop_terminal 로 이미 합쳤는데 예외 경로가 비대칭이었다."""
        if not self._is_stop_target(seq):
            return False
        self._log.error(
            f'★ 정지 대상(seq={seq}) 종결 확인 불가 (재전송 금지)')
        self._stop_failed(f'종결 확인 불가 (seq={seq})')   # _stop_pending 유지
        return True

    # ===========================================================
    # 내부 — 응답/결과 콜백 (세대 토큰으로 stale 구분)
    # ===========================================================
    def _on_goal_response(self, seq, future):
        """send_goal_async 응답 — 수락/거부 + stale(취소 레이스) + B 대상 처리."""
        # ★ future.result() 예외가 콜백 밖으로 새면 executor 로그에만 남고 미션은
        #   goal_active=True 로 영구 대기. 예외 = 응답 자체를 못 받음 → 정리+FAULT.
        try:
            handle = future.result()
        except Exception as e:
            self._log.error(f'★ goal 응답 수신 실패: {e} → FAULT')
            self._clear_response_pending(seq)
            if seq == self._seq:
                self._set(False)
                self._handle = None
                self._on_fault()
            elif self._is_stop_target(seq):
                # ★ P1: B 대상의 응답 자체가 예외 = 수락 여부 불명 → 정지 확인 불가.
                self._log.error(
                    f'★ 정지 대상(seq={seq}) 응답 예외 — 수락 여부 불명 '
                    f'(재전송 금지)')
                self._stop_failed(f'응답 예외 (seq={seq})')   # _stop_pending 유지
            return
        self._clear_response_pending(seq)
        if seq != self._seq:
            # ★ 취소 레이스: cancel 이 응답보다 먼저 실행되면 그 시점엔 핸들이 없어
            #   취소할 게 없었다. 뒤늦게 수락되면 Nav2 는 혼자 주행 계속 —
            #   "abort 했는데 안 멈춤"의 구멍 → stale 수락은 즉시 취소 + 최종결과 감시.
            if handle is not None and handle.accepted:
                self._log.warn(
                    f'뒤늦게 수락된 이전 목표(seq={seq}) 즉시 취소 '
                    f'(현재 seq={self._seq})')
                # 🔴 §88.2 ① — 이 등록이 동기 예외를 내면 구판은 **다음 줄의 취소가
                #   아예 안 불렸다.** Nav2 가 수락해 주행 중인 옛 goal 에 취소를 못
                #   보내는 것이라 직접적인 이동 P0 였다. `_watch` 가 삼키고,
                #   실패해도 아래 취소는 반드시 나간다.
                self._watch(handle.get_result_async,
                            partial(self._on_stale_result, seq),
                            f'늦은 수락 결과감시(seq={seq})',
                            stop_seq=(seq if self._is_stop_target(seq) else None))
                # ★ P1: 이 goal 이 B 정지 대상이면 취소도 B 로 귀속(빈 goals_canceling·
                #   호출 예외를 FAULT+봉쇄로). 아니면 일반 취소(로그만).
                self._cancel_with_confirm(
                    handle, f'이전 목표(seq={seq})',
                    stop_seq=(seq if self._is_stop_target(seq) else None))
            elif self._is_stop_target(seq):
                # ★ P1: B 대상이 거부됨(또는 핸들 없음) — 실제 주행 goal 이 생성되지
                #   않았다. 실패로 취급하면 영구 봉쇄 → 명시 로그 후 직렬화 정상 해제.
                self._log.info(
                    f'유도정지 대상(seq={seq}) 미수락 — 주행 goal 없음, 직렬화 해제')
                self._clear_stop()
            return
        if not handle.accepted:
            self._log.warn('목표 거부됨 → FAULT')
            self._set(False)
            self._on_fault()
            return
        self._handle = handle
        if not self._watch(handle.get_result_async, partial(self._on_result, seq),
                           f'결과감시(seq={seq})',
                           stop_seq=(seq if self._is_stop_target(seq) else None)):
            # 🔴 §89.3 — 구판은 `_on_fault()` 만 부르고 `active=True` · 핸들 보존이
            #   남았다. `enter_fault()` 는 goal 을 취소하지 않으므로 **Nav2 goal 이
            #   FAULT 아래서 계속 달리고 5초 뒤 자동 재개까지 했다** — 불완전한 FAULT.
            #   🔵 핸들을 아니까 **취소할 수 있다.** 취소하고 나서 미러를 내린다.
            self._cancel_with_confirm(
                handle, f'결과감시 실패 목표(seq={seq})',
                stop_seq=(seq if self._is_stop_target(seq) else None))
            self._handle = None
            self._set(False)

    def _on_result(self, seq, future):
        """현재 goal 최종결과 — 성공→on_reached, 실패→on_fault. stale 은 종결 관찰."""
        try:
            status = future.result().status
        except Exception as e:          # 결과 수신 실패도 미션을 멈추면 안 됨
            self._log.error(f'★ goal 결과 수신 실패(seq={seq}): {e}')
            if seq == self._seq:
                self._handle = None
                self._set(False)
                self._on_fault()
            else:
                # ★ 0723검토 §6.3 P1: stale 이라도 이게 B 정지 대상이면 terminal
                #   수신 실패 = 정지 확인 불가 → FAULT + 봉쇄(늦은 수락 경로와 대칭).
                self._stop_target_terminal_lost(seq)
            return
        if seq != self._seq:
            # ★ 취소한(=지난) goal 의 최종 status 관찰. cancel_current_goal 경로는
            #   _on_stale_result 가 없어 여기가 유일한 종결 관찰 지점이다.
            self._observe_stale_terminal(seq, status)
            return
        # 끝난 goal 핸들은 즉시 비움 — 남기면 다음 cancel 이 '이미 끝난 핸들'을
        # 취소하는 헛손질을 하고 stale-취소 경로와 조합 시 혼동 소지.
        self._handle = None
        self._set(False)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._on_reached()
        else:
            self._log.warn(f'목표 실패(status={status}) → FAULT (재시도 판단)')
            self._on_fault()

    def _observe_stale_terminal(self, seq, status):
        """지난 세대 goal 의 최종 status 관찰 (현재 goal 아님).
        B 대상이면 종결 직렬화 판정, 아니면 일반 관찰 로그."""
        if self._judge_stop_terminal(seq, status):
            return
        if status == GoalStatus.STATUS_CANCELED:
            self._log.info(f'지난 목표(seq={seq}) CANCELED 종결 확인')
        else:
            self._log.warn(
                f'지난 목표(seq={seq})가 취소 아닌 status={status} 로 종결 — '
                f'취소 전 이미 끝났거나 취소 실패 (주행 이력 확인 권장)')

    # ===========================================================
    # 내부 — 취소 접수·종결 확인
    # ===========================================================
    def _watch(self, make_future, cb, tag, stop_seq=None):
        """🔴 08-22 §88.2 — 비동기 등록의 **공통 보호 경계**.

        `get_result_async()` 도 `add_done_callback()` 도 **동기 예외를 낼 수 있다.**
        구판은 이 둘을 맨몸으로 불렀고, 재검토가 두 자리에서 예외가 밖으로 새는 것을
        재현했다(`:318` · `:400`). 예외가 새면 그 뒤 줄이 아예 안 돌고,
        `_stop_pending` 만 True 로 남아 **상위는 정지 여부를 영원히 모른다.**

        반환 True/False. 🔴 **False 여도 호출자는 다음 일을 계속해야 한다** —
        특히 결과 감시 등록 실패가 **취소 시도 자체를 건너뛰게 하면 안 된다**
        (그게 §88.2 ①의 직접적인 이동 P0 였다).
        """
        try:
            fut = make_future()
            fut.add_done_callback(cb)
            return True
        except Exception as e:                                   # noqa: BLE001
            self._log.error(f'★ {tag} 비동기 등록 실패: {e} — 종결 관찰 불가')
            # 🔴 §84.2 — **정지 대상 실패는 FAULT 로 보내지 않는다.** FAULT 는 자동
            #   재시도가 있는데, 정지가 확인 안 된 상황에서 재시도는 위험하다.
            #   ⚠ 일반 goal 경로를 **먼저** 처리해 그 분리가 눈에 보이게 둔다
            #     (`test_stop_exits` 가 정지 가드 아래의 직접 FAULT 를 잡는다).
            if stop_seq is None:
                self._on_fault()
                return False
            if self._is_stop_target(stop_seq):
                self._stop_failed(f'{tag} 등록 실패: {e}')
            return False

    def _cancel_with_confirm(self, handle, tag, stop_seq=None):
        """cancel_goal_async 의 응답까지 확인. 재시도는 안 한다(거절의 흔한 원인이
        '이미 종결'이라 무의미) — 진짜 이상은 경고 로그가 관제 대응 경로.

        ★ B: stop_seq(≠None)면 이 취소는 유도정지 대상 — '요청조차 실패'하면 정지
        확인 불가 → FAULT. 콜백엔 bool 이 아니라 대상 seq 를 실어, hard 해제 뒤 도착한
        옛 응답이 새 B 세대를 오판(false FAULT)하지 않게 한다(0723검토 §5 권장 8)."""
        try:                            # 호출 자체의 예외도 미션을 못 세우게
            fut = handle.cancel_goal_async()
        except Exception as e:
            self._log.error(f'★ {tag} 취소 요청 실패: {e} — 정지 미보장')
            if stop_seq is not None and self._is_stop_target(stop_seq):
                self._stop_failed(f'취소 요청 실패: {e}')
            return
        # 🔴 §88.2 ② — 구판은 이 등록이 try 밖이라 예외가 샜고, 그러면
        #   `_stop_failed` 가 안 불려 **상위가 취소 접수 여부를 모른다.**
        self._watch(lambda: fut, partial(self._on_cancel_response, tag, stop_seq),
                    f'{tag} 취소응답', stop_seq=stop_seq)

    def _on_cancel_response(self, tag, stop_seq, future):
        try:
            res = future.result()
        except Exception as e:      # 통신 실패 — 취소 접수 여부 자체를 모름
            self._log.error(f'★ {tag} 취소 응답 수신 실패: {e} — 정지 미보장')
            if stop_seq is not None and self._is_stop_target(stop_seq):
                # 🔴 §85.3 — 여기가 `_on_fault()` 직접 호출로 남아 있었다.
                #   safety_stop 이 FAULT 자동재시도로 새는 두 출구 중 하나다.
                self._stop_failed(f'취소 응답 수신 실패: {e}')
            return
        if res.goals_canceling:
            self._log.info(f'{tag} 취소 접수 확인 (Nav2 정지 진행)')
        else:
            # 빈 응답 = 취소할 goal 이 없다 — 대개 '이미 종결'(무해)이지만 주행이
            # 계속된다면 이 로그가 원인 추적의 첫 단서.
            self._log.warn(f'{tag} 취소 접수 안 됨 (이미 종결됐거나 거절)')
            # ★ B: 유도정지 취소가 접수조차 안 됐고 아직 종결 관찰 전이면(CANCELED
            #   terminal 이 이미 왔다면 _is_stop_target 이 거짓) 정지 확인 불가.
            if stop_seq is not None and self._is_stop_target(stop_seq):
                self._log.error(
                    '★ 정지 취소 접수 안 됨 — 정지 확인 불가 (재전송 금지)')
                # 🔴 §85.3 — 두 번째 누수 출구. 의도별 귀속으로 합친다.
                self._stop_failed('취소 접수 안 됨 (빈 goals_canceling)')

    def _on_stale_result(self, seq, future):
        """stale(뒤늦게 수락돼 즉시 취소한) goal 의 최종결과 감시 — CANCELED 정상.

        ★ P1: 이 goal 이 B 정지 대상(수락응답이 늦어 핸들 없이 무장된 그 세대)이면
        로그로 끝내지 말고 종결 직렬화 판정(_judge_stop_terminal)에 합류한다 —
        CANCELED 만 해제, 나머지는 FAULT+봉쇄. 아니면 기존 일반 관찰 로그."""
        try:
            status = future.result().status
        except Exception as e:
            self._log.error(f'★ 이전 목표(seq={seq}) 결과 수신 실패: {e}')
            self._stop_target_terminal_lost(seq)   # B 대상이면 FAULT+봉쇄(공통 helper)
            return
        if self._judge_stop_terminal(seq, status):
            return
        if status == GoalStatus.STATUS_CANCELED:
            self._log.info(f'이전 목표(seq={seq}) CANCELED 종결 확인')
        else:
            self._log.error(
                f'★ 이전 목표(seq={seq})가 취소되지 않고 status={status} 로 종결 — '
                f'abort/알람 시점에 로봇이 그 목표로 주행했을 수 있음 (위치 확인 필요)')
