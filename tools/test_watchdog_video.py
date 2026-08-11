#!/usr/bin/env python3
"""watchdog_video 회귀 — 부정 회귀(안 돼야 하는 것)를 먼저 박는다.

cv2·영상 없이 돈다(`analyze` 는 순수 함수, `rotation_series` 안에서만 cv2 를 부른다).
    python3 -m unittest tools.test_watchdog_video -v
"""
import io
import math
import unittest
from contextlib import redirect_stdout

from tools import watchdog_video as wv

FPS = 59.9955
DRIVE = -1.49          # 주행 중 실측 접선 회전 °/프레임 (2026-08-11, 무부하)


def row(n, rot=0.0, rad=0.02, npts=300):
    return (n, rot, rad, npts)


def series(t0=670, drive_from=610, stop_at=698, end=895, rot=DRIVE, tail_noise=0.0):
    """T0 앞뒤로 주행 → 정지 → 관찰 꼬리를 갖는 합성 열."""
    out = []
    for n in range(drive_from, end):
        if n < stop_at:
            out.append(row(n, rot))
        else:
            # 부호가 번갈아 바뀌는 잡음 — 창 안에서 상쇄돼야 정상이다.
            out.append(row(n, tail_noise * (1 if n % 2 else -1)))
    assert out[0][0] <= t0 <= out[-1][0]
    return out


class LowerBoundTest(unittest.TestCase):
    """🔴 이 도구의 핵심 계약 — 영상은 FAIL 만 증명하고 PASS 는 못 만든다."""

    def test_01_under_the_limit_is_undecidable_never_pass(self):
        """28프레임(466.7ms)은 구 판정선 500ms 아래인데도 PASS 가 아니다."""
        v = wv.analyze(series(), 670, FPS)
        self.assertTrue(v['ok'], v)
        self.assertEqual(28, v['n_frames'])
        self.assertEqual('판정 불능', v['legacy_verdict'])
        self.assertEqual('판정 불능', v['proposed_verdict'])
        self.assertNotIn('PASS', (v['legacy_verdict'], v['proposed_verdict']))

    def test_02_over_the_limit_is_a_definite_fail(self):
        """넘으면 확정이다 — 참값은 렌더 지연만큼 더 크기 때문이다."""
        v = wv.analyze(series(stop_at=720), 670, FPS)      # 50 프레임 = 833ms
        self.assertEqual('FAIL', v['legacy_verdict'])
        self.assertEqual('FAIL', v['proposed_verdict'])

    def test_03_proposed_limit_is_larger_than_legacy(self):
        """ⓐ 초안(600ms)과 구 기준(fps×0.5) 사이에서만 판정이 갈린다."""
        v = wv.analyze(series(stop_at=703), 670, FPS)      # 33 프레임 = 550ms
        self.assertEqual('FAIL', v['legacy_verdict'])
        self.assertEqual('판정 불능', v['proposed_verdict'])

    def test_04_main_returns_nonzero_when_no_pass_exists(self):
        buf = io.StringIO()
        with redirect_stdout(buf):
            rc = wv.main(['watchdog_video.py', 'x.mov', '--t0-frame', '670',
                          '--preset', '0807-1522', '--fps', str(FPS)],
                         series_fn=lambda *a, **k: series())
        self.assertEqual(1, rc)
        self.assertIn('하한', buf.getvalue())


class WindowedDetectorTest(unittest.TestCase):
    """§52.2 와 같은 자리 — 프레임당 증분으로 재면 정지를 못 본다."""

    def test_05_per_frame_noise_above_the_line_still_reads_as_stopped(self):
        """🔴 실측 재현: 프레임당 잡음 0.146°가 판정선 0.084°/f 를 넘는데도 정지다."""
        per_frame_line = wv.mm_s_to_deg_per_frame(wv.MOTION_RATE_MM_S, FPS)
        self.assertGreater(0.146, per_frame_line, '전제가 깨졌다 — 이 회귀의 의미가 없어진다')
        v = wv.analyze(series(tail_noise=0.146), 670, FPS)
        self.assertTrue(v['ok'], v)
        self.assertEqual(28, v['n_frames'], '번갈아 부호가 바뀌는 잡음을 이동으로 셌다')

    def test_06_sustained_slow_creep_is_motion(self):
        """반대 방향 — 같은 크기라도 **한 방향으로 지속**되면 이동이다(역회귀)."""
        rows = series()
        creep = [row(n, -0.12) for n in range(760, 820)]   # 0.12°/f ≈ 7.2 mm/s
        rows = [r for r in rows if r[0] < 760] + creep + [
            row(n) for n in range(820, 1000)]
        v = wv.analyze(rows, 670, FPS)
        self.assertTrue(v['ok'], v)
        self.assertGreater(v['stop_frame'], 800, '지속되는 creep 를 놓쳤다')

    def test_07_window_matches_the_bag_tool(self):
        """창이 다르면 두 도구의 수치가 같은 뜻이 아니게 된다."""
        v = wv.analyze(series(), 670, FPS)
        self.assertEqual(12, v['window_frames'])
        self.assertAlmostEqual(wv.MOTION_WINDOW_MS / 1000.0 * FPS, 12, places=0)


class UndecidableTest(unittest.TestCase):

    def test_08_short_tail_is_undecidable_not_a_stop(self):
        """§7-c-0 조건 2 — 마지막 회전 뒤 2초를 못 채우면 수치를 내지 않는다.

        ⚠ 꼬리는 잡음창(`NOISE_TAIL_FRAMES`)보다 길고 2초보다 짧아야 이 검사에 닿는다.
        더 짧으면 잡음창이 주행 구간과 겹쳐 그 앞의 '못 가른다'가 먼저 걸린다 —
        그 순서가 설계이며, 두 상수의 대소가 그것을 보증한다(아래 test_08b).
        """
        v = wv.analyze(series(stop_at=790, end=900), 670, FPS)   # 꼬리 1.83초
        self.assertFalse(v['ok'])
        self.assertIn('조건 2', v['reason'])

    def test_08b_noise_window_is_shorter_than_the_required_tail(self):
        """이 대소가 깨지면 조건 2 검사에 영원히 못 닿는다."""
        required = wv.REQUIRED_TAIL_MS / 1000.0 * FPS
        self.assertLess(wv.NOISE_TAIL_FRAMES, required)

    def test_09_t0_outside_the_range_is_undecidable(self):
        self.assertFalse(wv.analyze(series(), 100, FPS)['ok'])
        self.assertFalse(wv.analyze(series(), 5000, FPS)['ok'])

    def test_10_noise_above_the_decision_line_is_undecidable(self):
        """잡음이 판정선을 덮으면 수치를 내지 않는다 — 못 가른다고 말한다."""
        v = wv.analyze(series(tail_noise=3.0), 670, FPS)
        self.assertFalse(v['ok'])
        self.assertIn('못 가른다', v['reason'])

    def test_11_no_rotation_after_t0_is_undecidable(self):
        rows = [row(n) for n in range(610, 895)]
        v = wv.analyze(rows, 670, FPS)
        self.assertFalse(v['ok'])
        self.assertIn('T0', v['reason'])

    def test_12_too_few_frames_is_undecidable(self):
        self.assertFalse(wv.analyze([row(n, DRIVE) for n in range(50)], 10, FPS)['ok'])

    def test_13_zero_fps_is_undecidable(self):
        self.assertFalse(wv.analyze(series(), 670, 0.0)['ok'])


class CrossCheckTest(unittest.TestCase):
    """bag 과의 교차 — 렌더 지연이 타당 범위 밖이면 T0 오독이나 다른 시행이다."""

    def test_14_plausible_render_lag_is_accepted(self):
        v = wv.analyze(series(), 670, FPS, bag_ms=516.2)
        self.assertAlmostEqual(49.5, v['render_lag_ms'], places=1)
        self.assertTrue(v['render_lag_plausible'])

    def test_15_negative_render_lag_is_flagged(self):
        """bag 이 영상보다 짧으면 물리적으로 불가능하다 — 잡아야 한다."""
        v = wv.analyze(series(), 670, FPS, bag_ms=400.0)
        self.assertLess(v['render_lag_ms'], 0)
        self.assertFalse(v['render_lag_plausible'])

    def test_16_absurdly_large_lag_is_flagged(self):
        v = wv.analyze(series(), 670, FPS, bag_ms=900.0)
        self.assertFalse(v['render_lag_plausible'])


class ContractTest(unittest.TestCase):

    def test_17_constants_match_the_bag_tool_and_the_firmware(self):
        from tools import watchdog_report as wr
        self.assertEqual(wr.MOTION_RATE_MM_S, wv.MOTION_RATE_MM_S)
        self.assertEqual(wr.MOTION_WINDOW_MS, wv.MOTION_WINDOW_MS)
        self.assertEqual(wr.REQUIRED_TAIL_MS, wv.REQUIRED_TAIL_MS)
        self.assertEqual(0.05698, wv.WHEEL_RADIUS_M)      # `.ino:128`
        self.assertEqual(600, wv.PROPOSED_TOTAL_MS)       # 결정 1-ⓐ 초안
        self.assertEqual(0.5, wv.LEGACY_TOTAL_RATIO)      # 구 §7-c-0 조건 1

    def test_18_unit_conversion_round_trips(self):
        for mm_s in (2.0, 5.0, 10.0, 20.0):
            deg = wv.mm_s_to_deg_per_frame(mm_s, FPS)
            self.assertTrue(math.isclose(mm_s, wv.deg_per_frame_to_mm_s(deg, FPS),
                                         rel_tol=1e-9))

    def test_19_sensitivity_band_is_always_reported(self):
        v = wv.analyze(series(), 670, FPS)
        self.assertEqual(set(wv.SENSITIVITY_RATES_MM_S), set(v['sensitivity']))
        self.assertIn(wv.MOTION_RATE_MM_S, v['sensitivity'])

    def test_20_report_prints_the_lower_bound_warning_with_the_number(self):
        """🔴 수치와 경고가 **같은 화면**에 나와야 한다 — 잘라 인용되는 것을 막는다."""
        buf = io.StringIO()
        with redirect_stdout(buf):
            wv.report(wv.analyze(series(), 670, FPS))
        out = buf.getvalue()
        self.assertIn('466.7 ms', out)
        self.assertIn('하한', out)
        self.assertIn('검토자 확인 대기', out)

    def test_21_preset_reproduces_the_recorded_analysis(self):
        """정본에 적힌 수치를 만든 파라미터가 도구 안에 있어야 재현된다."""
        p = wv.PRESETS['0807-1522']
        self.assertEqual((610, 895), p['frame_range'])
        self.assertEqual((112.0, 178.0), p['axes'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
