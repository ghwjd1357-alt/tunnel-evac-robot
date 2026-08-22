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


class VerdictTest(unittest.TestCase):

    def test_w1_the_real_failure_is_caught(self):
        r, weak = W.verdict(*BROKEN)
        self.assertEqual('오른쪽', weak)
        self.assertAlmostEqual(-0.311, r, places=2)

    def test_w2_the_healthy_run_is_quiet(self):
        _, weak = W.verdict(*HEALTHY)
        self.assertIsNone(weak, '정상 시행에 경고가 떴다 — 멀쩡한 테이크를 버린다')

    def test_w3_the_weak_side_is_not_hardcoded(self):
        _, weak = W.verdict(BROKEN[0], -BROKEN[1])
        self.assertEqual('왼쪽', weak)

    def test_w4_a_stopped_robot_is_undecidable(self):
        r, weak = W.verdict(0.001, -0.05)
        self.assertIsNone(r)


class WatchTest(unittest.TestCase):

    def test_w5_a_recurrence_during_a_take_is_reported(self):
        _, hits = run([BROKEN] * 60)
        self.assertTrue(hits, '🔴 재발이 났는데 아무 말도 안 했다')
        self.assertIn('오른쪽', hits[0])

    def test_w6_a_healthy_take_produces_no_alert(self):
        _, hits = run([HEALTHY] * 60)
        self.assertEqual([], hits, '멀쩡한 주행에 경고가 떴다')

    def test_w7_turning_and_stopping_are_skipped(self):
        """🔴 회전·정지에서는 이 식이 성립하지 않는다 — 판정하면 안 된다."""
        _, turn = run([BROKEN] * 60, cmd=(0.0, 0.45))
        self.assertEqual([], turn, '회전 중에 직진 판정을 했다')
        _, stop = run([BROKEN] * 60, cmd=(0.0, 0.0))
        self.assertEqual([], stop, '정지 중에 판정을 했다')

    def test_w8_a_short_burst_is_not_enough(self):
        """창을 못 채운 표본으로 경고하면 노이즈 한 번이 테이크를 버린다."""
        _, hits = run([BROKEN] * 5)
        self.assertEqual([], hits)

    def test_w9_a_turn_in_the_middle_discards_the_window(self):
        """🔴 회전이 섞이면 창을 버려야 한다 — 안 그러면 회전 표본이 직진 판정에 샌다."""
        w = W.Watch(2.0)
        t = 0.0
        for i in range(60):
            t += 0.05
            w.feed(t, 0.10, 0.0, *HEALTHY)
        t += 0.05
        w.feed(t, 0.0, 0.45, *BROKEN)          # 회전 한 번
        self.assertEqual([], w.buf, '회전이 들어왔는데 창이 안 비워졌다')

    def test_w10_wheel_base_matches_the_firmware(self):
        from tools.firmware_constants import firmware_double
        self.assertEqual(firmware_double('ODOM_WHEEL_BASE'), W.ODOM_WHEEL_BASE)


if __name__ == '__main__':
    unittest.main()


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
