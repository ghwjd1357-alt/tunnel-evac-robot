# -*- coding: utf-8 -*-
"""
speed_manager.py — 주행속도 변경의 비동기 수명주기 전담 모듈 (07-20 구조 분리 1/3)
============================================================

[왜 분리했나 — 마스터플랜 §7.3-1]
  속도 변경은 "요청 → 응답 확인 → 실패 시 3회 재시도 → purpose 별 최종처리"라는
  비동기 수명주기를 가진다. 이게 MissionNode(상태머신) 안에 섞여 있으면
  상태 전이 코드와 콜백 타이밍 코드가 뒤엉켜 유령 FAULT(G1) 같은 레이스가 숨는다.
  → MissionNode 에는 "어떤 상태에서 어떤 속도를 원한다"는 정책만 남기고,
    수명주기 전부를 이 Manager 가 소유한다.

[핵심 설계 — generation(세대) 토큰]
  새 요청(guide/sync/restore)마다 세대 번호를 올린다. 응답 콜백은 자기 세대와
  현재 세대를 비교해 stale(낡은) 응답을 성공·실패 모두 구분한다:
    - stale 실패 → 무시 (재시도·FAULT 금지 — G1 유령 FAULT 의 정석 해법)
    - stale 성공 → 무시로 끝내지 않는다. 낡은 요청이 원격 controller 에
      '늦게 적용'됐다는 뜻이므로, 현재 desired 속도와 다르면
      **재조정(reconcile)** 요청을 보낸다 (Codex §14.3 필수 조건).
      reconcile 자체도 현재 세대를 달고 나가므로 "최신 세대만 유효" 규칙이
      똑같이 적용된다 — 낡은 reconcile 이 또 reconcile 을 부르는 핑퐁은
      "값이 desired 와 다를 때만 발사" 조건이 차단한다.

[desired vs applied — 07-20 Codex P1 보완의 핵심]
  원하는 값(desired)과 controller 에 실제로 반영된 값(applied)을 따로 추적한다.
  이전엔 `synced` 플래그 하나가 "한 번이라도 성공했다"는 뜻으로 세팅되고
  "지금 desired 가 반영돼 있다"는 뜻으로 읽혀서, 두 뜻이 어긋나는 순간
  아무도 복구를 책임지지 않는 구멍이 생겼다 (0720_현황.md §20.4).
    - `_settle()` = 매 tick, "desired 미반영인데 남은 요청이 없으면" 재요청.
      범위는 restore/reconcile 뿐 — guide 는 FAULT, sync 는 cooldown 이 담당.
      cooldown + MAX_SETTLE 상한으로 유한 종결(무한 재발사 구조적 불가).

[단일 deadline — "적용 확인까지 총 N초"]
  요청 하나에 예산 하나. 재시도·미준비 대기를 가로질러 유지되며, 요청이 실제로
  나가는 지점(_send)에서 무장한다. 서비스 미준비든 응답 Future 무응답이든
  같은 경로로 포기 처리 — 후자를 방치하면 로봇은 정지라 물리적으론 안전하지만
  고장이 관제 화면에 안 드러나고 무기한 숨는다 (07-20 정책이 막으려던 것).
  포기 시 세대를 올려 늦은 응답을 stale 화 → 기존 규칙이 그대로 처리한다.

[purpose 별 정책 (F2 유지)]
  'guide'     GUIDE 진입 게이트. 성공 확인 → on_guide_confirmed (노드가 전환).
              3회 실패·예외·서비스 미준비 timeout → on_guide_failed (노드가 FAULT).
              서비스 미준비면 즉시 포기하지 않고 unready_timeout_sec 까지
              대기 후 실패 확정 (07-20 사용자 결정: timeout 후 FAULT —
              무기한 GATHER 대기는 관제 화면에서 고장을 은폐하므로).
  'sync'      시작 속도 동기화. 성공 응답에서만 synced=True.
              3회 실패 → cooldown(10 tick=5초) 후 ensure_sync 가 재요청.
  'restore'   평시 복원 (ESCAPED·reset·FAULT 정지). 실패 = 에러 로그만
              (이때 로봇은 정지/종결 국면 — 주행 위험 없음).
  'reconcile' (내부 전용) stale 늦은 적용의 되돌리기. 실패 = 에러 로그만.

[사용법 — MissionNode 쪽]
  매 tick: speed.tick() → speed.ensure_sync(normal)   (cooldown·미준비 대기 구동)
  GATHER 종료: speed.request_guide(guide_speed)
  복원: speed.request_restore(normal)
  reset/abort: speed.cancel_pending(...)  ← 진행 중 요청 전부 stale 화
"""

import time
from functools import partial

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


class SpeedManager:
    """controller_server 속도 파라미터 변경의 요청·확인·재시도·stale 처리 전담."""

    MAX_ATTEMPTS = 3            # 시도 상한 (기존 set_nav_speed 의 3회 재시도 유지)
    SYNC_COOLDOWN_TICKS = 10    # sync 최종 실패 후 재요청 대기 (10 tick = 5초)
    MAX_SETTLE = 3              # 종결 재평가 재요청 상한 (유한 종결 보장)
    SETTLE_COOLDOWN_TICKS = 10  # 재평가 재요청 간격 (10 tick = 5초)

    def __init__(self, param_cli, logger, *,
                 on_guide_confirmed, on_guide_failed,
                 unready_timeout_sec=30.0,
                 param_name='FollowPath.desired_linear_vel',
                 now_fn=time.monotonic):
        """param_cli = SetParameters 서비스 클라이언트 (노드가 만들어 줌).
        on_guide_confirmed() / on_guide_failed(reason) = 노드의 정책 콜백.
        now_fn = 단조 시계 (테스트에서 시간을 손으로 돌리기 위해 주입 가능).
        ⚠ timeout 은 벽시계(monotonic) — 시뮬 use_sim_time 과 무관하게
          '서비스가 실제로 안 뜬 실경과시간'을 재는 게 목적이라 의도된 선택."""
        self._cli = param_cli
        self._log = logger
        self._on_guide_confirmed = on_guide_confirmed
        self._on_guide_failed = on_guide_failed
        self._param_name = param_name
        self._unready_timeout = float(unready_timeout_sec)
        self._now = now_fn

        # --- 세대 토큰 + 현재 desired ---
        self._gen = 0             # 새 요청·cancel_pending 마다 +1
        # ★ origin(상위 정책 출처)과 in-flight purpose(지금 날아가는 요청의 종류)를
        #   분리한다. 한 값에 겸직시켰더니 guide 에서 출발한 내부 reconcile 이
        #   양쪽에서 오분류됐다 (07-20 Codex 재검토 P1 — 0720_현황.md §22.2).
        self._desired = None      # (v, origin) — origin ∈ guide/sync/restore
        self._applied = None      # controller 에 '성공 확인'된 마지막 값
        self._inflight = False    # 현재 세대 요청이 응답 대기 중인가
        self._inflight_purpose = None   # 그 요청의 종류 (reconcile 포함)
        self._guide_confirmed = False   # GUIDE 저속이 한 번이라도 확인됐는가

        # --- 종결 재평가 (0720 P1 — desired 미반영 채로 끝나는 것 금지) ---
        self._settle_cooldown = 0
        self._settle_rounds = 0
        self._settle_gave_up = False

        # --- 요청 단위 단일 deadline (0720 P1 — "적용 확인까지 총 N초") ---
        self._deadline_at = None  # None = 추적 중인 요청·대기 없음

        # --- sync 상태 (기존 _speed_synced/_inflight/_cooldown 이관) ---
        self.synced = False       # ★ 성공 '응답'에서만 True (F2 불변조건 유지)
        self._sync_cooldown = 0

        # --- guide 서비스 미준비 대기 (07-20 timeout→FAULT 정책) ---
        self._wait_since = None   # 대기 시작 시각 (monotonic). None = 대기 아님
        self._wait_v = None
        self._wait_gen = None     # 대기가 어느 세대의 것인지 (reset 이 무효화)

    # ===========================================================
    # 공개 API — MissionNode 의 정책이 부르는 4개 + tick
    # ===========================================================
    def request_guide(self, v):
        """GUIDE 저속 적용 요청. 성공 확인 → on_guide_confirmed 콜백.
        서비스 미준비면 timeout 까지 tick 이 대기·재시도 (기존 '무기한 GATHER
        보류'를 대체 — timeout 소진 시 on_guide_failed 로 FAULT 확정)."""
        self._new_request(float(v), 'guide')

    def ensure_sync(self, v):
        """시작 속도 동기화 — 매 tick 불러도 안전 (멱등).
        이미 동기화됐거나, 요청 진행 중이거나, cooldown 중이거나,
        서비스 미준비면 아무것도 안 한다 (기존 tick 의 sync 블록 이관)."""
        if self.synced or self._inflight or self._sync_cooldown > 0:
            return
        if self._wait_since is not None:      # guide 대기 중 — sync 로 덮지 않음
            return
        if not self._cli.service_is_ready():
            return
        self._new_request(float(v), 'sync')

    def request_restore(self, v):
        """평시 속도 복원 (ESCAPED·reset·FAULT 정지). 미준비면 경고 후 생략
        (기존 동작 유지 — 이 국면의 로봇은 정지/종결이라 주행 위험 없음)."""
        self._new_request(float(v), 'restore')

    def cancel_pending(self, reason=''):
        """reset/abort: 진행 중 요청을 전부 stale 화 — 늦은 성공·실패 응답이
        상태·FAULT 를 덮지 못하게 세대를 올린다. desired 는 그대로 두므로
        '늦은 적용값 == desired' 인 경우 불필요한 reconcile 은 안 나간다."""
        self._gen += 1
        self._inflight = False
        self._inflight_purpose = None
        self._guide_confirmed = False
        self._deadline_at = None
        if self._wait_since is not None:
            self._log.info(f'guide 저속 대기 취소 ({reason})')
            self._wait_since = None
            self._wait_v = None
            self._wait_gen = None

    def tick(self):
        """MissionNode.tick(0.5초)마다 호출 — cooldown 감소 + 미준비 대기 +
        deadline 감시 + 종결 재평가."""
        if self._sync_cooldown > 0:
            self._sync_cooldown -= 1
        if self._settle_cooldown > 0:
            self._settle_cooldown -= 1

        if self._wait_since is not None and self._wait_gen != self._gen:
            self._wait_since = None           # reset 등으로 이미 무효화된 대기
            self._wait_v = None
            self._wait_gen = None

        if self._check_deadline():
            return                            # 이번 tick 은 포기 처리로 종료

        if self._wait_since is not None:
            if self._cli.service_is_ready():  # 서비스가 떴다 — 이제 진짜 요청
                v = self._wait_v
                self._wait_since = None
                self._wait_v = None
                self._wait_gen = None
                self._send(v, 1, 'guide', self._gen)
            else:
                self._log.warn(
                    'controller_server 파라미터 서비스 미준비 — GUIDE 저속 대기 중',
                    throttle_duration_sec=5.0)

        self._settle()

    # ===========================================================
    # 내부 — 요청 생성·전송·응답 처리
    # ===========================================================
    def _new_request(self, v, purpose):
        """새 desired 선언: 세대 +1 (이전 in-flight 응답은 전부 stale 이 된다)."""
        self._gen += 1
        self._inflight = False
        self._desired = (v, purpose)
        # ★ 요청 단위 단일 deadline — 재시도·미준비 대기를 가로질러 유지한다.
        #   (시도마다 재무장하면 3회 × timeout 이 되어 "총 N초" 정책이 깨진다.
        #    §19.4-3 의 '재진입마다 재무장' 위험도 이 한 줄로 함께 닫힌다.)
        self._deadline_at = self._now() + self._unready_timeout
        # 새 정책 결정 = 재평가 예산도 새로 (이전 실패 이력에 발목 잡히지 않게)
        self._settle_cooldown = 0
        self._settle_rounds = 0
        self._settle_gave_up = False
        self._guide_confirmed = False
        # 이전 guide 대기가 있었다면 새 요청이 대체 (세대 불일치로 tick 이 정리)
        if not self._cli.service_is_ready():
            if purpose == 'guide':
                # 즉시 포기 대신 timeout 대기 시작 (tick 이 준비 감시)
                self._wait_since = self._now()
                self._wait_v = v
                self._wait_gen = self._gen
                self._log.warn(
                    f'controller_server 파라미터 서비스 미준비 — GUIDE 저속 '
                    f'{self._unready_timeout}초 내 준비 대기')
            else:
                # sync 는 ensure_sync 가 준비 후 재시도 / restore 는 생략 (기존 동작).
                # 추적할 요청이 없으므로 deadline 을 내리고, desired 미반영 복구는
                # _settle 이 담당한다 (기존엔 여기서 영영 잊혔다 — 0720 P1).
                self._deadline_at = None
                self._log.warn('controller_server 파라미터 서비스 없음 — 속도 변경 생략')
            return
        self._send(v, 1, purpose, self._gen)

    def _send(self, v, attempt, purpose, gen):
        """SetParameters 비동기 전송 (재시도는 같은 세대 유지)."""
        if not self._cli.service_is_ready():
            # 재시도 도중 서비스 소멸 — purpose 별 기존 정책 유지
            self._inflight = False
            self._inflight_purpose = None
            if purpose == 'guide':
                # 다시 대기 모드 — 단, deadline 은 재무장하지 않는다 (총 예산 유지)
                self._wait_since = self._now()
                self._wait_v = v
                self._wait_gen = gen
                self._log.warn('재시도 중 서비스 소멸 — GUIDE 저속 준비 대기 재진입')
            else:
                # ★ 0720 P1(2번째 누출 경로): 여기서 요청이 _final_fail 조차 없이
                #   증발했다. deadline 을 내리고 _settle 에 복구를 넘긴다.
                self._deadline_at = None
                self._log.warn('controller_server 파라미터 서비스 없음 — 속도 변경 생략')
            return
        p = Parameter()
        p.name = self._param_name
        p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                 double_value=v)
        try:
            fut = self._cli.call_async(SetParameters.Request(parameters=[p]))
        except Exception as e:
            # 호출 자체 예외(서비스 소멸 순간 등) — 콜백 밖 전파 금지, 시도 1회 소모
            if attempt < self.MAX_ATTEMPTS:
                self._log.warn(
                    f'속도 변경 호출 예외({e}) — 재시도 {attempt + 1}/{self.MAX_ATTEMPTS}')
                self._send(v, attempt + 1, purpose, gen)
            else:
                self._final_fail(v, purpose, str(e), gen)
            return
        self._inflight = (gen == self._gen)
        self._inflight_purpose = purpose
        # ★ deadline 은 '요청이 실제로 나간 시점'에 무장하되, 이미 무장돼 있으면
        #   손대지 않는다 — 재시도가 예산을 연장하면 "총 N초" 정책이 깨진다.
        #   여기 두는 이유: _new_request 를 거치지 않는 발사 경로(_settle 재평가,
        #   stale reconcile)도 감시 대상이어야 한다. 빠지면 그 요청이 무응답일 때
        #   _inflight=True 로 영구 정지한다 (자기 반증에서 발견).
        if self._deadline_at is None:
            self._deadline_at = self._now() + self._unready_timeout
        fut.add_done_callback(partial(self._on_result, v, attempt, purpose, gen))
        self._log.info(f'주행속도 변경 요청 → {v} m/s (시도 {attempt})')

    def _on_result(self, v, attempt, purpose, gen, future):
        """응답 확인 콜백 — 세대 비교로 stale 성공·실패를 모두 구분 (S1-5+G1 정석)."""
        try:
            res = future.result()
            ok = bool(res.results) and all(r.successful for r in res.results)
            reason = '' if ok else '; '.join(
                r.reason for r in res.results if not r.successful)
        except Exception as e:
            ok, reason = False, str(e)

        if gen != self._gen:
            self._on_stale_result(v, purpose, ok, reason)
            return

        if ok:
            self._inflight = False
            self._inflight_purpose = None
            self._deadline_at = None
            self._applied = v          # ★ controller 에 실제로 반영된 값
            self._log.info(f'주행속도 변경 확인 → {v} m/s')
            # 어떤 요청이든 성공 확인 = controller 에 속도를 성공적으로 선언한 것 —
            # 시작 동기화의 목적(재시작 저속 상속 방지)은 이 시점에 충족된다.
            self.synced = True
            if purpose == 'guide':
                self._guide_confirmed = True   # 이후 실패는 '유지 실패'로 분류
                self._on_guide_confirmed()
        else:
            if attempt < self.MAX_ATTEMPTS:
                self._log.warn(
                    f'속도 변경 실패({reason}) — 재시도 {attempt + 1}/{self.MAX_ATTEMPTS}')
                self._send(v, attempt + 1, purpose, gen)
            else:
                self._final_fail(v, purpose, reason, gen)

    def _on_stale_result(self, v, purpose, ok, reason):
        """낡은 세대 응답 처리 — 실패는 무시, 성공은 reconcile 판단 (Codex §14.3).

        늦은 '성공' = 낡은 속도가 원격 controller 에 방금 적용됐다는 뜻.
        현재 desired 와 다르면 controller 가 잘못된 속도로 남는다 →
        desired 로 재조정 요청. 핑퐁 방지: ① 최신 세대만 유효 규칙은 reconcile
        에도 동일 적용 ② 값이 desired 와 같으면 발사 안 함 ③ 현재 세대 요청이
        이미 응답 대기 중이면 그 요청이 곧 덮으므로 발사 안 함."""
        if not ok:
            self._log.warn(
                f'늦은 {purpose} 속도 실패 응답({v} m/s: {reason}) 무시 — '
                f'세대 변경됨, 재시도·FAULT 안 함')
            return
        self._applied = v              # ★ 늦게라도 '적용'된 값 — 재평가의 기준
        self._log.warn(
            f'늦은 {purpose} 속도 성공 응답 — {v} m/s 가 controller 에 뒤늦게 적용됨')
        if self._desired is None:
            return
        dv = self._desired[0]
        if v == dv:
            return                        # 늦게 적용된 값이 이미 desired — 재조정 불필요
        if self._inflight:
            # 현재 세대 요청이 응답 대기 중 — 그 요청이 desired 를 곧 덮는다.
            # ★ 0720 P1 이전엔 "그 요청이 최종 실패하면 purpose 별 최종처리가
            #   안전을 담당한다"고 봤지만, restore/reconcile 의 최종처리는
            #   '에러 로그만'이라 desired 가 영영 복구되지 않았다. 지금은 이 생략이
            #   안전의 책임자가 아니라 순수 최적화다 — 복구는 _settle 이 보증한다.
            self._log.info(
                f'재조정 생략 — 현재 세대 요청({dv} m/s) 응답 대기 중')
            return
        if self._wait_since is not None:
            return                        # 서비스 미준비 대기 중 — 보낼 수 없음
        self._log.warn(f'★ 재조정(reconcile): 현재 desired {dv} m/s 로 재요청')
        self._send(dv, 1, 'reconcile', self._gen)

    def _final_fail(self, v, purpose, reason, gen):
        """3회 최종 실패 — purpose 별 안전 조치 (기존 _speed_final_fail 이관)."""
        self._inflight = False
        self._inflight_purpose = None
        self._deadline_at = None
        self._log.error(
            f'★ 속도 변경 {self.MAX_ATTEMPTS}회 실패({reason}) — 현재 주행속도가 '
            f'요청값 {v} 와 다를 수 있음 (GUIDE 저속 미적용 위험, 관제 확인 필요)')
        self._terminate(purpose, reason)
        # restore/reconcile: FAULT 는 아니지만(정지/종결 국면) 로그로 끝내지 않는다 —
        # desired 미반영이 남으면 다음 tick 의 _settle 이 복구를 재시도한다.

    # ===========================================================
    # 내부 — deadline 감시 + 종결 재평가 (0720 Codex P1 보완)
    # ===========================================================
    def _check_deadline(self):
        """요청 예산 소진 검사. 소진했으면 포기 처리 후 True.

        ★ 하나의 deadline 이 두 경우를 함께 덮는다:
          ① 서비스가 안 떠서 요청조차 못 나간 경우 (기존 _wait_since 경로)
          ② 서비스는 떴는데 응답 Future 가 영영 완료되지 않는 경우 (0720 신규)
        ②를 방치하면 로봇은 GATHER 정지라 물리적으로는 안전하지만, 고장이
        관제 화면에 드러나지 않고 무기한 숨는다 — 07-20 확정 정책("저속 적용
        확인까지 총 N초")이 막으려던 바로 그 은폐다.

        포기할 때 세대를 올리는 이유: 뒤늦게 도착할 응답이 자동으로 stale 이
        되어 기존 규칙(늦은 성공이면 reconcile, 늦은 실패면 무시)을 그대로 탄다.
        새 방어 코드를 짤 필요가 없다."""
        if self._deadline_at is None or self._now() < self._deadline_at:
            return False
        unready = self._wait_since is not None
        v, origin = self._desired if self._desired else (None, '')
        # ★ '무엇이 만료했나' 는 in-flight purpose, '그래서 어떻게 종결하나' 는
        #   origin. 겸직시켰더니 reconcile 만료를 guide 진입 실패로 오분류해
        #   통보가 통째로 버려졌다 (재검토 §10.3).
        kind = self._inflight_purpose or origin

        self._gen += 1                    # 늦은 응답을 stale 화
        self._inflight = False
        self._inflight_purpose = None
        self._deadline_at = None
        self._wait_since = None
        self._wait_v = None
        self._wait_gen = None

        if unready:
            self._log.error(
                f'★ controller_server 파라미터 서비스 {self._unready_timeout}초 '
                f'미준비 — GUIDE 저속 적용 불가, FAULT 전환')
        else:
            self._log.error(
                f'★ 속도 변경 응답 {self._unready_timeout}초 미도착 '
                f'({v} m/s, {kind}) — 무응답 포기 (controller 확인 필요)')

        self._terminate(
            kind, 'service unready timeout' if unready else 'no response timeout')
        return True

    def _terminate(self, kind, reason):
        """요청 종결 시 상위 정책 조치 — '무엇이 실패했나(kind)'와
        '무엇을 위한 요청이었나(origin)'를 나눠서 판단한다.

        내부 reconcile 은 여기서 끝내지 않는다 — 예산과 최종 종결은 _settle 이
        소유한다. 그래야 "제한 재시도 후 FAULT"(07-20 사용자 확정) 정책이
        reconcile 1회 실패로 앞당겨지지 않는다."""
        if kind == 'reconcile':
            return
        origin = self._desired[1] if self._desired else ''
        if origin == 'guide':
            # ★ '진입 차단'인지 '유도 중단'인지는 MissionNode 가 자기 상태로 판단한다.
            #   Manager 가 _guide_confirmed 로 대신 판단하면, FAULT 자동 재시도로
            #   GUIDE 에 복귀한 순간 두 관점이 어긋난다 (자기반증에서 발견).
            self._on_guide_failed(reason)
        elif origin == 'sync':
            self._sync_cooldown = self.SYNC_COOLDOWN_TICKS

    def _settle(self):
        """종결 재평가 — desired 가 controller 에 반영 안 된 채 아무 요청도
        남지 않았으면 다시 발사한다 (0720 Codex P1 의 근본 수정).

        [왜 필요한가] 기존엔 `synced` 플래그가 "한 번이라도 성공했다"는 뜻으로
        세팅되고 "지금 desired 가 반영돼 있다"는 뜻으로 쓰였다. 두 뜻이 겹치니
        ensure_sync 가 안전망 역할을 구조적으로 못 했다. 그래서 '지금 반영된 값'
        (_applied)을 별도로 추적하고, desired 와 어긋난 채 종결됐는지만 본다.

        [범위 = restore/reconcile 뿐] 이 둘만 최종 실패가 '에러 로그만'이라
        복구 주체가 없다. guide 는 실패하면 FAULT 가, sync 는 cooldown 뒤
        ensure_sync 가 이미 담당하므로 여기서 또 쏘면 이중 발사가 된다.

        [유한 종결] cooldown(5초) + MAX_SETTLE 라운드 상한. 상한을 넘으면 큰
        에러 로그 1회만 남기고 완전히 조용해진다 — 무한 재발사가 구조적으로
        불가능하다 (Codex 0720 §5 가 명시 확인한 조건)."""
        if self._desired is None or self._inflight:
            return
        if self._wait_since is not None:
            return                                  # 미준비 대기 중 — tick 이 담당
        dv, origin = self._desired
        # 범위: restore(복구 주체가 없던 원래 구멍) + 확인된 guide(유지 보장).
        # sync 는 cooldown+ensure_sync 가 이미 담당하므로 제외(이중 발사 금지).
        # ★ 이전엔 origin guide 를 통째로 뺐다가, GUIDE 진입 후 stale 이 덮은
        #   경우를 아무도 안 고쳤다 (재검토 §10.2).
        if not (origin == 'restore'
                or (origin == 'guide' and self._guide_confirmed)):
            return
        if self._applied == dv:
            return                                  # 이미 일치 — 할 일 없음
        if self._settle_cooldown > 0 or self._settle_gave_up:
            return
        if self._settle_rounds >= self.MAX_SETTLE:
            self._settle_gave_up = True
            self._log.error(
                f'★ 재조정 포기 — {dv} m/s 복구를 {self.MAX_SETTLE}회 시도했으나 '
                f'실패, controller 속도가 {self._applied} 로 남음 (관제 확인 필요)')
            if origin == 'guide':
                # 07-20 사용자 확정: 제한 재시도 후에도 저속을 못 지키면
                # 사람을 평시속도로 유도하느니 정지시킨다.
                self._on_guide_failed('guide speed reconcile exhausted')
            return
        if not self._cli.service_is_ready():
            return                                  # 헛발사 금지 — 다음 tick 에 재시도
        self._settle_rounds += 1
        self._settle_cooldown = self.SETTLE_COOLDOWN_TICKS
        self._log.warn(
            f'★ 종결 재평가: desired {dv} m/s 가 미반영(현재 {self._applied}) — '
            f'재요청 {self._settle_rounds}/{self.MAX_SETTLE}')
        self._send(dv, 1, 'reconcile', self._gen)
