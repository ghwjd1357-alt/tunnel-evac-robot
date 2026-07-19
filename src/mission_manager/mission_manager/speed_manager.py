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

    MAX_ATTEMPTS = 3          # 시도 상한 (기존 set_nav_speed 의 3회 재시도 유지)
    SYNC_COOLDOWN_TICKS = 10  # sync 최종 실패 후 재요청 대기 (10 tick = 5초)

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
        self._desired = None      # (v, purpose) — 최신 요청의 속도 (reconcile 기준)
        self._inflight = False    # 현재 세대 요청이 응답 대기 중인가

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
        if self._wait_since is not None:
            self._log.info(f'guide 저속 대기 취소 ({reason})')
            self._wait_since = None
            self._wait_v = None
            self._wait_gen = None

    def tick(self):
        """MissionNode.tick(0.5초)마다 호출 — cooldown 감소 + guide 미준비 대기."""
        if self._sync_cooldown > 0:
            self._sync_cooldown -= 1
        if self._wait_since is None:
            return
        if self._wait_gen != self._gen:       # reset 등으로 이미 무효화된 대기
            self._wait_since = None
            return
        if self._cli.service_is_ready():      # 서비스가 떴다 — 이제 진짜 요청
            v = self._wait_v
            self._wait_since = None
            self._wait_v = None
            self._wait_gen = None
            self._send(v, 1, 'guide', self._gen)
        elif self._now() - self._wait_since >= self._unready_timeout:
            # ★ 07-20 사용자 결정: 무기한 안전대기 대신 timeout 후 FAULT —
            #   관제 화면에 고장이 즉시 가시화되고 reset/abort 복구 경로가 있음.
            self._wait_since = None
            self._wait_v = None
            self._wait_gen = None
            self._log.error(
                f'★ controller_server 파라미터 서비스 {self._unready_timeout}초 '
                f'미준비 — GUIDE 저속 적용 불가, FAULT 전환')
            self._on_guide_failed('service unready timeout')
        else:
            self._log.warn(
                'controller_server 파라미터 서비스 미준비 — GUIDE 저속 대기 중',
                throttle_duration_sec=5.0)

    # ===========================================================
    # 내부 — 요청 생성·전송·응답 처리
    # ===========================================================
    def _new_request(self, v, purpose):
        """새 desired 선언: 세대 +1 (이전 in-flight 응답은 전부 stale 이 된다)."""
        self._gen += 1
        self._inflight = False
        self._desired = (v, purpose)
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
                # sync 는 ensure_sync 가 준비 후 재시도 / restore 는 생략 (기존 동작)
                self._log.warn('controller_server 파라미터 서비스 없음 — 속도 변경 생략')
            return
        self._send(v, 1, purpose, self._gen)

    def _send(self, v, attempt, purpose, gen):
        """SetParameters 비동기 전송 (재시도는 같은 세대 유지)."""
        if not self._cli.service_is_ready():
            # 재시도 도중 서비스 소멸 — purpose 별 기존 정책 유지
            self._inflight = False
            if purpose == 'guide':
                self._wait_since = self._now()   # 다시 대기 모드 (timeout 재무장)
                self._wait_v = v
                self._wait_gen = gen
                self._log.warn('재시도 중 서비스 소멸 — GUIDE 저속 준비 대기 재진입')
            else:
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
            self._log.info(f'주행속도 변경 확인 → {v} m/s')
            # 어떤 요청이든 성공 확인 = controller 에 속도를 성공적으로 선언한 것 —
            # 시작 동기화의 목적(재시작 저속 상속 방지)은 이 시점에 충족된다.
            self.synced = True
            if purpose == 'guide':
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
        self._log.warn(
            f'늦은 {purpose} 속도 성공 응답 — {v} m/s 가 controller 에 뒤늦게 적용됨')
        if self._desired is None:
            return
        dv = self._desired[0]
        if v == dv:
            return                        # 늦게 적용된 값이 이미 desired — 재조정 불필요
        if self._inflight:
            # 현재 세대 요청이 응답 대기 중 — 그 요청이 desired 를 곧 덮는다.
            # (그 요청이 최종 실패하면 purpose 별 최종처리가 이미 안전을 담당)
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
        self._log.error(
            f'★ 속도 변경 {self.MAX_ATTEMPTS}회 실패({reason}) — 현재 주행속도가 '
            f'요청값 {v} 와 다를 수 있음 (GUIDE 저속 미적용 위험, 관제 확인 필요)')
        if purpose == 'guide':
            self._on_guide_failed(reason)
        elif purpose == 'sync':
            self._sync_cooldown = self.SYNC_COOLDOWN_TICKS
        # restore/reconcile: 에러 로그로 충분 (정지/종결 국면 — 주행 위험 없음)
