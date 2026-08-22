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
