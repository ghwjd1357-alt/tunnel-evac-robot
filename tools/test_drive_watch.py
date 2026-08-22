#!/usr/bin/env python3
"""`drive_watch.py` 회귀 — 08-22 실측을 그대로 흘린다.

🔴 이 도구는 **테이크를 다시 찍으라고 말하는** 물건이다. 거짓 경고는 멀쩡한 테이크를
버리게 하고, 놓친 경고는 망가진 테이크를 편집까지 들고 간다. 둘 다 재촬영이 없는
일정에서 비싸다. 숫자는 08-21 21:32 / 17:17 bag 실측이다.
"""
import unittest

import drive_watch as W

BROKEN = (0.0772, -0.0580)     # rehearsal2_0821 21:32 (odom 속도, ω)
HEALTHY = (0.0958, +0.0031)    # rehearsalD_0821 17:17


def run(samples, cmd=(0.10, 0.0), hz=20.0, window=2.0):
    w = W.Watch(window)
    hits, t = [], 0.0
    for lin, ow in samples:
        t += 1.0 / hz
        m = w.feed(t, cmd[0], cmd[1], lin, ow)
        if m:
            hits.append(m)
    return w, hits


# 08-21 실측 (odom ω, IMU ω) — 이 네 경우가 판정선의 전부다
STRAIGHT_OK = (+0.0031, +0.0031)     # rehearsalD 17:17 직진
STRAIGHT_BAD = (-0.0580, +0.0058)    # rehearsal2  21:32 직진
TURN_OK = (+0.1283, +0.1262)         # M2 14:00 회전
TURN_BAD = (+0.0965, +0.1456)        # spin180b 21:26 회전


def run(pairs, hz=47.0, window=2.0, lin=0.10):
    """`/odom` 실측은 47.3 Hz 다. 3.8 Hz 는 `/detections` 이고 이 도구와 무관하다."""
    w = W.Watch(window)
    msgs, t = [], 0.0
    for ow, iw in pairs:
        t += 1.0 / hz
        m = w.feed(t, ow, iw, lin)
        if m:
            msgs.append(m)
    return w, msgs


class DiscrepancyTest(unittest.TestCase):
    """🔴 기준은 **자이로**다 — 바퀴와 물리적으로 무관한 유일한 증인."""

    def test_w1_the_real_failures_are_caught(self):
        for tag, (ow, iw) in (('직진', STRAIGHT_BAD), ('회전', TURN_BAD)):
            _, _, bad = W.discrepancy(ow, iw)
            self.assertTrue(bad, f'{tag} 고장을 못 잡았다')

    def test_w2_the_healthy_runs_are_quiet(self):
        for tag, (ow, iw) in (('직진', STRAIGHT_OK), ('회전', TURN_OK)):
            _, _, bad = W.discrepancy(ow, iw)
            self.assertFalse(bad, f'{tag} 정상에 경고 — 멀쩡한 테이크를 버린다')

    def test_w3_a_healthy_curve_is_not_a_fault(self):
        """🔴 **부정 회귀 — 이것이 첫 판을 폐기시킨 결함이다.**

        구판은 `cmd_vel` 이 직진일 때 `odom ω` 가 0 인지를 봤다. Nav2 RPP 는 곡률
        보정을 계속 내므로(전진 명령 |ω| 중앙값 0.0468) **정상 곡선에서 124회
        오경보**가 났다. 자이로 기준에서는 곡률이 있어도 둘이 같이 움직인다.
        """
        for w_ in (0.02, 0.05, 0.10, 0.30):
            _, _, bad = W.discrepancy(w_ * 1.02, w_)   # odom 이 IMU 를 2% 안에서 따라감
            self.assertFalse(bad, f'정상 곡선 ω={w_} 에서 오경보')

    def test_w4_a_rotation_uses_a_proportional_threshold(self):
        """회전에서는 절대 문턱만 쓰면 큰 ω 의 작은 오차에 소리친다."""
        _, th_slow, _ = W.discrepancy(0.0, 0.0)
        _, th_fast, _ = W.discrepancy(0.0, 0.5)
        self.assertGreater(th_fast, th_slow)


class WatchTest(unittest.TestCase):

    def test_w5_a_recurrence_during_a_take_is_reported(self):
        _, msgs = run([STRAIGHT_BAD] * 300)
        self.assertTrue(msgs, '🔴 재발이 났는데 아무 말도 안 했다')
        self.assertIn('오른쪽', msgs[0], '직진 지문인데 쪽을 안 말했다')

    def test_w6_a_healthy_take_produces_no_alert(self):
        for pairs in ([STRAIGHT_OK] * 300, [TURN_OK] * 300):
            _, msgs = run(pairs)
            self.assertEqual([], msgs, '멀쩡한 주행에 경고가 떴다')

    def test_w7_one_event_does_not_spam(self):
        """🔴 §89.6 — 한 사건에 계속 소리치면 로그가 묻힌다. latch 한다."""
        w, msgs = run([STRAIGHT_BAD] * 500)
        self.assertEqual(1, len([m for m in msgs if m.startswith('🔴')]))
        self.assertEqual(1, w.alerts)

    def test_w8_recovery_rearms_the_latch(self):
        """복구를 보면 다시 무장한다 — 간헐 고장이라 두 번째가 온다."""
        w = W.Watch(2.0)
        t = 0.0
        seq = ([STRAIGHT_BAD] * 200) + ([STRAIGHT_OK] * 200) + ([STRAIGHT_BAD] * 200)
        msgs = []
        for ow, iw in seq:
            t += 1.0 / 47.0
            m = w.feed(t, ow, iw, 0.10)
            if m:
                msgs.append(m)
        self.assertEqual(2, w.alerts, f'두 번째 재발을 놓쳤다: {msgs}')
        self.assertTrue(any(m.startswith('🟢') for m in msgs), '복구를 안 알렸다')

    def test_w9_a_short_burst_is_not_enough(self):
        _, msgs = run([STRAIGHT_BAD] * 5)
        self.assertEqual([], msgs)

    def test_w10_being_blind_is_not_the_same_as_being_quiet(self):
        """🔴 §89.6 — 표본이 모자라 판정이 **안 서는 것**을 조용함으로 착각하면 안 된다.

        구판은 낮은 발행률에서 그냥 침묵했다 — 필요할 때 침묵하는 것이 제일 나쁘다.
        """
        w, msgs = run([STRAIGHT_BAD] * 20, hz=2.0)   # 최소 rate 미달
        self.assertEqual([], msgs)
        self.assertGreater(w.blind_for(20.0 / 2.0), 5.0,
                           '판정이 안 서는데 그 사실을 보고하지 않는다')

    def test_w11_wheel_base_constant_is_gone(self):
        """🔵 자이로 기준으로 바꾸면서 윤거 상수 의존이 사라졌다 — 남아 있으면 잔재다."""
        self.assertFalse(hasattr(W, 'ODOM_WHEEL_BASE'),
                         'cmd 기반 잔재가 남았다')


class ScanWatchTest(unittest.TestCase):
    """🔴 라이다는 **에러 없이** 죽는다 (`PITFALLS §17-①`). "안 온다" 를 세는 것뿐이다."""

    def test_w11_a_silent_lidar_is_reported_once(self):
        sw = W.ScanWatch(timeout=1.0)
        for i in range(10):
            self.assertIsNone(sw.on_scan(i * 0.1))
            self.assertIsNone(sw.check(i * 0.1))
        first = sw.check(2.0)
        self.assertIsNotNone(first, '🔴 /scan 이 끊겼는데 아무 말도 안 했다')
        self.assertIn('0.9', first, '마지막 수신 시각이 안 찍혔다')
        self.assertIsNone(sw.check(3.0), '같은 사망에 두 번 소리쳤다')
        self.assertEqual(1, sw.deaths)

    def test_w12_normal_ten_hz_never_trips(self):
        """🔵 역회귀 — 정상 10 Hz 에 소리치면 매 테이크가 버려진다."""
        sw = W.ScanWatch(timeout=1.0)
        for i in range(200):
            t = i * 0.1
            sw.on_scan(t)
            self.assertIsNone(sw.check(t), f'정상 주기 t={t} 에서 오경보')
        self.assertEqual(0, sw.deaths)

    def test_w13_one_dropped_frame_is_not_a_death(self):
        """한 프레임 걸러진 것에 소리치면 안 된다 — 실측에서 흔하다."""
        sw = W.ScanWatch(timeout=1.0)
        sw.on_scan(0.0)
        self.assertIsNone(sw.check(0.5))       # 0.5초 공백은 정상 범위
        sw.on_scan(0.6)
        self.assertEqual(0, sw.deaths)

    def test_w14_recovery_reports_how_long_it_was_out(self):
        """🔵 복구 시각과 지속시간이 있어야 `dmesg` 와 대조할 수 있다."""
        sw = W.ScanWatch(timeout=1.0)
        sw.on_scan(0.0)
        self.assertIsNotNone(sw.check(2.0))
        back = sw.on_scan(5.0)
        self.assertIsNotNone(back, '복구를 안 알렸다')
        self.assertIn('5.0', back)
        self.assertIsNone(sw.dead_since, '복구 후에도 사망 상태가 남았다')

    def test_w15_a_lidar_that_never_started_is_not_called_dead(self):
        """🔴 한 번도 안 온 것과 오다 끊긴 것은 다르다 — 기동 전 오경보 금지."""
        sw = W.ScanWatch(timeout=1.0)
        self.assertIsNone(sw.check(100.0))
        self.assertEqual(0, sw.deaths)


def test_w16_the_default_timeout_has_real_margin_over_10hz():
    """🔴 부정 회귀 — 기본값이 정상 주기에 가까우면 **매 테이크가 오경보로 버려진다.**

    ⚠ 위 시험들이 전부 `timeout=1.0` 을 명시해서 **기본값 자체는 검사 밖**이었다
    (변이로 0.05 를 넣어도 전부 통과했다). 기본값도 잠근다.
    정상 `/scan` 은 10 Hz(0.1s) 다 — 최소 5배 여유를 요구한다.
    """
    assert W.SCAN_TIMEOUT_DEFAULT >= 0.5, \
        f'기본 {W.SCAN_TIMEOUT_DEFAULT}s 는 10 Hz 주기에 너무 가깝다'
    sw = W.ScanWatch()                      # 기본값으로
    for i in range(100):
        t = i * 0.1
        sw.on_scan(t)
        assert sw.check(t) is None, f'기본값이 정상 10 Hz 에 오경보 (t={t})'
    assert sw.deaths == 0
