#!/usr/bin/env python3
"""watchdog_report 회귀 — 검토 §52 의 필수 부정 회귀·역회귀.

ROS 없이 돈다(`analyze` 는 순수 함수, `load` 안에서만 rosbag2_py 를 부른다).
    python3 -m unittest tools.test_watchdog_report -v
"""
import io
import math
import unittest
from contextlib import redirect_stdout

from tools import watchdog_report as wr

MS = 1_000_000  # ns


def odom(t_ms, x=0.0, y=0.0, vx=0.0):
    return (int(t_ms * MS), x, y, vx)


def cmd(t_ms, lin=0.0, ang=0.0):
    return (int(t_ms * MS), lin, ang)


def straight(start_ms, n, step_mm, period_ms=20, x0=0.0):
    """`period_ms` 마다 `step_mm` 씩 +x 로 가는 표본열."""
    return [odom(start_ms + i * period_ms, x0 + i * step_mm / 1000.0)
            for i in range(n)]


def settled(start_ms, n, x, period_ms=20):
    return [odom(start_ms + i * period_ms, x) for i in range(n)]


class AnalyzeTest(unittest.TestCase):

    # ── §52.1 zero 개입 ────────────────────────────────────────────────
    def test_01_zero_intervening_is_undecidable_not_a_watchdog_number(self):
        """비영 0ms → zero 100ms → 이동 520ms. 구판은 520.0 을 냈다."""
        cmds = [cmd(0, 0.12), cmd(100)]
        odoms = settled(0, 5, 0.0) + straight(100, 30, 1.0) + settled(700, 200, 0.030)
        verdict = wr.analyze(cmds, odoms)
        self.assertFalse(verdict['ok'])
        # 핵심 = 어떤 수치도 내지 않는다. 구판은 여기서 520.0 을 냈다.
        self.assertNotIn('last_move_ms', verdict)

    def test_02_zero_before_two_second_tail_is_undecidable(self):
        """마지막 이동 뒤 2초가 차기 전에 zero 가 오면 수치를 내지 않는다."""
        cmds = [cmd(0, 0.12), cmd(1500)]
        odoms = straight(0, 26, 2.0) + settled(520, 50, 0.050)
        self.assertFalse(wr.analyze(cmds, odoms)['ok'])

    def test_03_zero_after_full_observation_uses_only_the_clean_window(self):
        """관찰이 충분히 찬 뒤의 정상 종료용 zero 는 앞 구간만 쓰게 한다(역회귀)."""
        cmds = [cmd(0, 0.12), cmd(3000)]
        odoms = straight(0, 27, 2.0) + settled(540, 130, 0.052)
        verdict = wr.analyze(cmds, odoms)
        self.assertTrue(verdict['ok'], verdict)
        self.assertAlmostEqual(520.0, verdict['last_move_ms'], places=1)
        self.assertAlmostEqual(3000.0, verdict['zero_gap_ms'], places=1)

    # ── §52.2 누적 이동 ───────────────────────────────────────────────
    def test_04_cumulative_creep_below_the_step_threshold_is_movement(self):
        """0.4mm × 130 표본 = 52mm. 구판은 전부 버리고 '정지'라고 했다."""
        cmds = [cmd(0, 0.12)]
        odoms = straight(0, 130, 0.4) + settled(2600, 150, 0.4 * 129 / 1000.0)
        verdict = wr.analyze(cmds, odoms)
        self.assertTrue(verdict['ok'], verdict)
        # 마지막 증분이 임계 미만이어도 누적으로 잡혀 2580ms 근처가 나와야 한다.
        self.assertGreater(verdict['last_move_ms'], 2000)

    def test_05_bounded_jitter_below_threshold_stays_stopped(self):
        """0.5mm 안에서 떠는 잡음은 이동이 아니다(역회귀)."""
        cmds = [cmd(0, 0.12)]
        jitter = [odom(540 + i * 20, 0.052 + (0.00005 if i % 2 else -0.00005))
                  for i in range(200)]
        verdict = wr.analyze(cmds, straight(0, 27, 2.0) + jitter)
        self.assertTrue(verdict['ok'], verdict)
        self.assertAlmostEqual(520.0, verdict['last_move_ms'], places=1)

    def test_06_non_finite_pose_is_undecidable_not_stopped(self):
        cmds = [cmd(0, 0.12)]
        for bad in (float('nan'), float('inf')):
            odoms = straight(0, 26, 2.0) + [odom(520, bad)] + settled(540, 130, 0.050)
            verdict = wr.analyze(cmds, odoms)
            self.assertFalse(verdict['ok'], bad)
            self.assertIn('NaN', verdict['reason'])

    # ── 판정 불능 일반 ────────────────────────────────────────────────
    def test_07_empty_topics_are_undecidable(self):
        self.assertFalse(wr.analyze([], [])['ok'])
        self.assertFalse(wr.analyze([cmd(0, 0.12)], [])['ok'])
        self.assertFalse(wr.analyze([], [odom(0)])['ok'])

    def test_08_no_nonzero_command_is_undecidable(self):
        self.assertFalse(wr.analyze([cmd(0), cmd(50)], settled(0, 10, 0.0))['ok'])

    def test_09_zero_duration_is_undecidable(self):
        odoms = [odom(0, 0.0), odom(0, 0.0)]
        self.assertFalse(wr.analyze([cmd(0, 0.12)], odoms)['ok'])

    def test_10_no_movement_in_window_is_undecidable_not_a_pass(self):
        cmds = [cmd(0, 0.12)]
        self.assertFalse(wr.analyze(cmds, settled(0, 200, 0.0))['ok'])

    # ── settle_index 경계 ─────────────────────────────────────────────
    def test_11_diagonal_displacement_counts_both_axes(self):
        """축마다 0.8mm 면 각각은 1.0mm 임계 미만인데 합치면 1.13mm 다."""
        samples = [odom(0, 0.0, 0.0)] + [odom(20 + i * 20, 0.0008, 0.0008)
                                         for i in range(150)]
        self.assertEqual(0, wr.last_motion_index(samples))
        # 축 하나만 0.8mm 벌어지면 임계 미만이라 정지로 남아야 한다 — 역방향.
        flat = [odom(0, 0.0, 0.0)] + [odom(20 + i * 20, 0.0008, 0.0)
                                      for i in range(150)]
        self.assertIsNone(wr.last_motion_index(flat))

    def test_12_last_motion_index_finds_the_transition(self):
        samples = straight(0, 10, 2.0) + settled(200, 10, 0.018)
        self.assertEqual(8, wr.last_motion_index(samples))

    def test_12b_slow_settling_drift_is_not_motion(self):
        """실측 1521 재현 — 정지 뒤 표본당 0.07mm 로 1.3mm 흐르는 것은 이동이 아니다."""
        drift = [odom(2000 + i * 21, 0.05 - i * 0.00007) for i in range(70)]
        samples = straight(0, 26, 2.0) + settled(520, 70, 0.050) + drift
        last = wr.last_motion_index(samples)
        self.assertLess(samples[last][0] / 1e6, 600, '느린 안착을 이동으로 셌다')


class ExitCodeTest(unittest.TestCase):
    """§52.4 — 판정 불능이 종료코드 0 으로 새지 않는다."""

    def _run(self, bags):
        buf = io.StringIO()
        with redirect_stdout(buf):
            rc = wr.main(['watchdog_report.py', *bags.keys()])
        return rc

    def test_13_undecidable_bag_returns_nonzero(self):
        wr.load, saved = (lambda bag: ([], [])), wr.load
        try:
            self.assertEqual(1, self._run({'empty': None}))
        finally:
            wr.load = saved

    def test_14_all_valid_bags_return_zero(self):
        good = ([cmd(0, 0.12)], straight(0, 26, 2.0) + settled(520, 130, 0.050))
        wr.load, saved = (lambda bag: good), wr.load
        try:
            self.assertEqual(0, self._run({'a': None, 'b': None}))
        finally:
            wr.load = saved

    def test_15_mixed_valid_and_invalid_returns_nonzero(self):
        good = ([cmd(0, 0.12)], straight(0, 26, 2.0) + settled(520, 130, 0.050))
        seen = []

        def fake(bag):
            seen.append(bag)
            return good if bag == 'good' else ([], [])

        wr.load, saved = fake, wr.load
        try:
            self.assertEqual(1, self._run({'good': None, 'bad': None}))
        finally:
            wr.load = saved

    def test_16_unreadable_bag_is_undecidable_not_a_crash(self):
        def boom(bag):
            raise RuntimeError('sqlite3 header damaged')

        wr.load, saved = boom, wr.load
        try:
            self.assertEqual(1, self._run({'broken': None}))
        finally:
            wr.load = saved


class ContractTest(unittest.TestCase):

    def test_17_print_window_is_not_the_judgement_window(self):
        """출력 절단(2600ms)이 판정을 자르면 안 된다 — §52.1 이 둘을 분리했다."""
        self.assertEqual((-150, 2600), wr.PRINT_WINDOW_MS)
        cmds = [cmd(0, 0.12)]
        odoms = straight(0, 200, 0.4) + settled(4000, 150, 0.4 * 199 / 1000.0)
        verdict = wr.analyze(cmds, odoms)
        self.assertTrue(verdict['ok'], verdict)
        self.assertGreater(verdict['last_move_ms'], wr.PRINT_WINDOW_MS[1])

    def test_18_constants_match_the_canon(self):
        self.assertEqual(600, wr.WATCHDOG_CONTRACT_MS)
        self.assertEqual(2000, wr.REQUIRED_TAIL_MS)
        self.assertEqual(0.5, wr.MOVE_EPS_MM)          # 잡음 바닥
        self.assertEqual(5.0, wr.MOTION_RATE_MM_S)     # 이동 판정선 — 다른 값이다
        self.assertEqual(200, wr.MOTION_WINDOW_MS)
        self.assertTrue(math.isclose(1.0, wr.MOTION_RATE_MM_S * wr.MOTION_WINDOW_MS / 1000))

    def test_19_sensitivity_band_is_always_reported(self):
        """임계 하나로 수치가 흔들리는 것을 숨기지 않는다."""
        cmds = [cmd(0, 0.12)]
        verdict = wr.analyze(cmds, straight(0, 26, 2.0) + settled(520, 130, 0.050))
        self.assertTrue(verdict['ok'], verdict)
        self.assertEqual(set(wr.SENSITIVITY_RATES_MM_S), set(verdict['sensitivity']))
        self.assertIn(wr.MOTION_RATE_MM_S, verdict['sensitivity'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
