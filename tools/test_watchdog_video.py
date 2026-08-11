#!/usr/bin/env python3
"""watchdog_video 회귀 — 부정 회귀(안 돼야 하는 것)를 먼저 박는다.

cv2·영상 없이 돈다(`analyze` 는 순수 함수, `rotation_series` 안에서만 cv2 를 부른다).
    python3 -m unittest tools.test_watchdog_video -v
"""
import io
import math
import unittest
from contextlib import redirect_stderr, redirect_stdout

from tools import watchdog_video as wv

FPS = 59.9955
DRIVE = -1.49          # 주행 중 실측 접선 회전 °/프레임 (2026-08-11, 무부하)


NAN = float('nan')

# 🔴 생산자(`rotation_series`)가 추적에 실패하는 **네 경로 전부**와, 각 경로가 남기는 행의
# 모양. 검토 §57.1 이 연 자리 — 소비자가 이 넷을 전부 "판정 불능" 으로 보내야 한다.
PRODUCER_FAILURES = (
    ('특징점 없음(goodFeaturesToTrack None)', NAN, 0),
    ('추적 점 수 부족(len(Q0)<25 or len(P0)<40)', NAN, 300),
    ('배경 아핀 실패(estimateAffine2D None)', NAN, 300),
    ('유효 바퀴점 부족(keep.sum()<30)', NAN, 12),
)
# 판정에 닿는 네 자리. T0 직후 · 감속 경계 · 2초 꼬리 중간 · 꼬리 끝.
INJECTION_POINTS = (671, 697, 800, 894)

# 🔴 위 넷 중 **앞의 셋**은 생산자가 `continue` 로 빠져나가면서 바퀴 중심 갱신까지
# 건너뛴다 — 그래서 그 프레임의 카메라 이동이 중심에서 영구 누락된다(검토 §58.1).
# 넷째(`keep.sum()<30`)는 배경 아핀이 살아 있어 중심은 갱신되지만, **최소 안전선**을
# 택했으므로 소비자는 넷을 구분하지 않는다 — 틀리더라도 보수적인 쪽으로만 틀린다.
CHAIN_BREAKING = PRODUCER_FAILURES[:3]
# 판정에 **안 닿는** 자리들. 구간 시작 · T0 직전 · T0 앞 연속 20프레임.
PRE_T0_INJECTIONS = (('구간 시작', (610,)),
                     ('T0 직전', (669,)),
                     ('T0 앞 연속 20프레임', tuple(range(650, 670))))


def row(n, rot=0.0, rad=0.02, npts=300):
    return (n, rot, rad, npts)


def report_text(v):
    buf = io.StringIO()
    with redirect_stdout(buf):
        wv.report(v)
    return buf.getvalue()


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


class ObservationCompletenessTest(unittest.TestCase):
    """🔴 검토 §57.1 — **못 본 프레임은 정지 증거가 아니다.**

    구판은 추적 실패(`NaN`)를 `0.0` 회전으로 바꿔, T1 뒤 꼬리를 통째로 못 봐도
    `조건 2 충족` 을 냈다. 이 클래스가 그 fail-open 을 전부 막는다.
    """

    def test_22_every_producer_failure_at_every_position_is_undecidable(self):
        for name, rot, npts in PRODUCER_FAILURES:
            for at in INJECTION_POINTS:
                rows = [r if r[0] != at else (at, rot, NAN, npts)
                        for r in series()]
                v = wv.analyze(rows, 670, FPS)
                with self.subTest(cause=name, at=at):
                    self.assertFalse(v['ok'], f'{name} @{at} 를 관측으로 셌다')
                    self.assertIn('관측 실패', v['reason'])

    def test_23_the_whole_tail_going_blind_is_not_a_stop(self):
        """🔴 검토자가 재현한 그 공격 — T1 뒤 전량 NaN 이 `cond2_ok=True` 였다."""
        rows = [r if r[0] < 698 else (r[0], NAN, NAN, 0) for r in series()]
        v = wv.analyze(rows, 670, FPS)
        self.assertFalse(v['ok'])
        self.assertNotIn('cond2_ok', v)

    def test_24_finite_rotation_with_too_few_points_is_still_invalid(self):
        """생산자가 바뀌어 유한값을 내놔도 유효점이 모자라면 관측이 아니다."""
        rows = [r if r[0] != 800 else (800, 0.0, 0.02, wv.MIN_VALID_POINTS - 1)
                for r in series()]
        self.assertFalse(wv.analyze(rows, 670, FPS)['ok'])

    def test_25_one_missing_frame_is_undecidable(self):
        rows = [r for r in series() if r[0] != 800]
        v = wv.analyze(rows, 670, FPS)
        self.assertFalse(v['ok'])
        self.assertIn('연속이 아니다', v['reason'])

    def test_26_a_long_deleted_span_cannot_be_counted_as_observation(self):
        """🔴 구판은 중간을 통째로 지워도 `frames[-1]-stop` 으로 2초를 셌다."""
        rows = [r for r in series() if not 700 <= r[0] < 860]
        v = wv.analyze(rows, 670, FPS)
        self.assertFalse(v['ok'])
        self.assertIn('연속이 아니다', v['reason'])

    def test_27_duplicate_and_reversed_frame_numbers_are_undecidable(self):
        rows = series()
        dup = rows[:100] + [rows[99]] + rows[100:]
        self.assertFalse(wv.analyze(dup, 670, FPS)['ok'])
        rev = rows[:100] + rows[100:120][::-1] + rows[120:]
        self.assertFalse(wv.analyze(rev, 670, FPS)['ok'])

    def test_28_early_eof_against_the_requested_range_is_undecidable(self):
        """요청은 610~894 인데 850 에서 끊겼다 — 남은 꼬리는 관찰한 적이 없다."""
        rows = [r for r in series() if r[0] < 850]
        v = wv.analyze(rows, 670, FPS, expected_range=(610, 895))
        self.assertFalse(v['ok'])
        self.assertIn('조기 EOF', v['reason'])
        # 같은 열이라도 요청 구간을 모르면 그 사실을 주장하지 않는다.
        self.assertTrue(wv.analyze(rows, 670, FPS)['ok'])

    def test_29_invalid_frames_before_t0_block_too(self):
        """🔴 검토 §58.1 로 **뒤집힌** 회귀. 구판은 여기서 `ok=True` 를 냈다.

        "T0 앞은 판정에 안 쓰이니 봐준다" 가 정확히 뒷문이었다 — 중심이 누적 상태라
        T0 앞 실패는 T0 **이후 전부**를 오염시킨다.
        """
        rows = [r if r[0] != 620 else (620, NAN, NAN, 0) for r in series()]
        v = wv.analyze(rows, 670, FPS)
        self.assertFalse(v['ok'], 'T0 앞 실패를 그대로 통과시켰다')
        self.assertIn('T0 앞이라도', v['reason'])

    def test_30_the_real_contiguous_run_still_reproduces_the_record(self):
        """🔴 역회귀 — 실제와 같은 285프레임·NaN 0건은 기록값을 그대로 낸다."""
        rows = series()
        self.assertEqual(285, len(rows))
        self.assertFalse([r for r in rows if not math.isfinite(r[1])])
        v = wv.analyze(rows, wv.RECORDED['t0_frame'], wv.RECORDED['fps'],
                       bag_ms=wv.RECORDED['bag_ms'], expected_range=(610, 895))
        self.assertTrue(v['ok'], v.get('reason'))
        self.assertEqual(wv.RECORDED['stop_frame'], v['stop_frame'])
        self.assertEqual(wv.RECORDED['n_frames'], v['n_frames'])
        self.assertAlmostEqual(wv.RECORDED['measured_ms'], v['measured_ms'], places=1)
        self.assertAlmostEqual(wv.RECORDED['delta_ms'],
                               v['cross_observer_delta_ms'], places=1)


class CenterChainTest(unittest.TestCase):
    """🔴 검토 §58.1 — **"나중 행이 finite" 는 상태가 회복됐다는 증거가 아니다.**

    바퀴 중심은 프레임마다 배경 아핀으로 누적해 옮기는 상태다. 추적 실패는 그 갱신을
    건너뛰므로, T0 **앞**의 실패 한 번이 어긋난 중심을 T0 로 실어 나른다. 검토자의
    실측 공격에서 조건 2 가 `0.5945` → `0.1487 mm/s` 로 4분의 1까지 과소평가됐다
    (안전 반대 방향). 이 클래스가 그 뒷문을 막는다.
    """

    def test_46_chain_breaking_failures_before_t0_are_undecidable(self):
        """세 경로 × 세 자리. 🔴 **뒤 행을 전부 유한하게 둬도** 판정 불능이어야 한다."""
        for name, rot, npts in CHAIN_BREAKING:
            for where, frames in PRE_T0_INJECTIONS:
                hit = set(frames)
                rows = [r if r[0] not in hit else (r[0], rot, NAN, npts)
                        for r in series()]
                # 전제 확인 — T0 이후는 한 프레임도 안 건드렸다.
                self.assertFalse([r for r in rows
                                  if r[0] >= 670 and not math.isfinite(r[1])])
                v = wv.analyze(rows, 670, FPS, expected_range=(610, 895))
                with self.subTest(cause=name, where=where):
                    self.assertFalse(v['ok'], f'{name} @{where} 를 통과시켰다')
                    self.assertIn('관측 실패', v['reason'])
                    # 오염된 상태로 조건 2 를 계산해 내보내면 안 된다.
                    self.assertNotIn('drift_mm_s', v)
                    self.assertNotIn('cond2_ok', v)

    def test_47_the_reason_names_the_center_not_just_the_blind_frame(self):
        """왜 T0 앞까지 막는지가 사유에 남아야 한다 — 다음 사람이 되돌리지 않도록."""
        rows = [r if r[0] != 615 else (615, NAN, NAN, 0) for r in series()]
        self.assertIn('바퀴 중심', wv.analyze(rows, 670, FPS)['reason'])

    def test_48_a_clean_run_prints_the_completeness_line(self):
        """🔴 검토 §58 전제는 "**별도 출력으로** 확인" 이다 — 안 찍히면 못 쓴다."""
        v = wv.analyze(series(), 670, FPS, expected_range=(610, 895))
        self.assertEqual(285, v['observed_frames'])
        self.assertEqual((610, 894), v['observed_span'])
        self.assertTrue(v['range_checked'])
        text = report_text(v)
        self.assertIn('관측 완전성', text)
        self.assertIn('285프레임 전량 연속·유한', text)
        self.assertIn('✅', text)

    def test_49_without_a_requested_range_the_line_says_so(self):
        """`--range` 없이 돌리면 조기 EOF 를 못 본다 — ✅ 로 위장하지 않는다."""
        v = wv.analyze(series(), 670, FPS)
        self.assertFalse(v['range_checked'])
        text = report_text(v)
        self.assertIn('⚠', text)
        self.assertNotIn('전량 연속·유한·유효점≥30 ✅', text)


class CrossObserverTest(unittest.TestCase):
    """검토 §57.2 — bag 과의 차이는 **관측계 차이**일 뿐, 원인을 특정하지 않는다."""

    def test_14_delta_is_reported_without_naming_a_cause(self):
        v = wv.analyze(series(), 670, FPS, bag_ms=wv.RECORDED['bag_ms'])
        self.assertAlmostEqual(wv.RECORDED['delta_ms'],
                               v['cross_observer_delta_ms'], places=1)
        self.assertNotIn('render_lag_ms', v, '옛 이름이 남으면 옛 주장이 남는다')
        self.assertNotIn('render_lag_plausible', v)

    def test_14b_the_same_49_5ms_never_prints_a_render_lag_verdict(self):
        """🔴 같은 입력이 '렌더 지연 확정'·'타당'을 출력하면 안 된다."""
        out = report_text(wv.analyze(series(), 670, FPS,
                                     bag_ms=wv.RECORDED['bag_ms']))
        self.assertIn('관측계 차이', out)
        self.assertIn('한 원인으로 특정하지 않는다', out)
        self.assertNotIn('렌더 지연 +', out)
        self.assertNotIn('타당', out)

    def test_14c_no_bag_value_promotes_a_single_cause(self):
        """저장 지연을 다르게 주입해도(=차이가 아무리 변해도) 원인 판정은 없다."""
        for bag_ms in (470.0, 516.2, 560.0, 900.0, 2000.0):
            out = report_text(wv.analyze(series(), 670, FPS, bag_ms=bag_ms))
            self.assertNotIn('타당', out, bag_ms)
            self.assertNotIn('범위 밖', out, bag_ms)
            self.assertIn('한 원인으로 특정하지 않는다', out, bag_ms)

    def test_15_negative_delta_is_flagged_for_re_checking_not_judged(self):
        """bag 이 영상보다 짧으면 같은 시행인지 되물어야 한다 — 타당/부당 판정은 아니다."""
        v = wv.analyze(series(), 670, FPS, bag_ms=400.0)
        self.assertLess(v['cross_observer_delta_ms'], 0)
        self.assertTrue(v['delta_negative'])
        self.assertIn('부호가 음수다', report_text(v))

    def test_16_a_large_delta_is_not_an_error_and_not_a_verdict(self):
        """상한을 못 정하므로 큰 차이도 '부당'이라 부르지 않는다(0~60ms 판정 폐기)."""
        v = wv.analyze(series(), 670, FPS, bag_ms=900.0)
        self.assertFalse(v['delta_negative'])
        self.assertFalse(hasattr(wv, 'PLAUSIBLE_RENDER_LAG_MS'),
                         '타당 범위 상수가 남으면 §57.2 가 되살아난다')

    def test_16b_the_lower_bound_holds_without_any_bag_value(self):
        """🔴 하한 성질은 교차검사에 의존하지 않는다 — bag 없이도 PASS 는 없다."""
        v = wv.analyze(series(), 670, FPS)
        self.assertIsNone(v['cross_observer_delta_ms'])
        self.assertNotIn('PASS', (v['legacy_verdict'], v['proposed_verdict']))


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


class RecordedFactsTest(unittest.TestCase):
    """검토 §57.3 — 설명·출력·회귀가 **한 출처**(`RECORDED`)에서 나온다."""

    def test_31_the_docstring_is_built_from_recorded_not_typed_by_hand(self):
        doc = wv.__doc__
        self.assertNotIn('{', doc, '포맷이 안 됐다 — 설명이 상수와 끊겼다')
        for key in ('measured_ms', 'bag_ms', 'delta_ms'):
            self.assertIn(str(wv.RECORDED[key]), doc, key)

    def test_32_the_stale_numbers_are_gone_from_the_description(self):
        """🔴 실제는 466.7/49.5 인데 설명만 500.0/16.2 로 남아 있던 자리다."""
        self.assertNotIn('`500.0ms`', wv.__doc__)
        self.assertNotIn('`16.2ms`', wv.__doc__)      # `516.2ms` 와 헷갈리지 않게 백틱까지
        self.assertNotIn('정확히 렌더 지연', wv.__doc__)

    def test_33_recorded_measured_ms_is_derivable_from_frames_and_fps(self):
        """숫자 하나를 바꾸면 여기서 깨진다 — 손으로 적은 값이 못 남는다."""
        derived = wv.RECORDED['n_frames'] / wv.RECORDED['fps'] * 1000.0
        self.assertAlmostEqual(wv.RECORDED['measured_ms'], derived, places=1)
        self.assertAlmostEqual(
            wv.RECORDED['delta_ms'],
            wv.RECORDED['bag_ms'] - wv.RECORDED['measured_ms'], places=1)
        self.assertEqual(wv.RECORDED['n_frames'],
                         wv.RECORDED['stop_frame'] - wv.RECORDED['t0_frame'])

    def test_34_the_recorded_drift_satisfies_condition_2(self):
        """조건 2 는 영상만으로 닫히는 유일한 조건이다 — 그 기록도 판정선 아래여야 한다."""
        self.assertLess(wv.RECORDED['drift_mm_s'], wv.MOTION_RATE_MM_S)


class CliContractTest(unittest.TestCase):
    """검토 §57.4 — 입력 오류는 traceback 이 아니라 원인 + `rc=2` 다."""

    BASE = ['watchdog_video.py', 'x.mov', '--t0-frame', '670',
            '--preset', '0807-1522']

    def run_cli(self, **opts):
        """🔴 `--opt=값` 한 토큰으로 넘긴다 — 음수 값(`-1`)을 argparse 가 옵션으로 읽지
        않게 하려는 것이다. 그 경로는 `test_45` 가 따로 본다."""
        extra = [f"--{k.replace('_', '-')}={v}" for k, v in opts.items()]
        buf, err = io.StringIO(), io.StringIO()
        with redirect_stdout(buf), redirect_stderr(err):
            rc = wv.main(self.BASE + extra, series_fn=lambda *a, **k: series())
        return rc, buf.getvalue() + err.getvalue()

    def test_35_non_finite_numbers_end_with_rc_2_not_a_traceback(self):
        """🔴 `--fps nan` 이 `round(NaN)` 에서 ValueError 로 죽던 자리다."""
        for bad in ('nan', 'NaN', 'inf', '-inf', '', 'abc'):
            with self.subTest(fps=bad):
                rc, out = self.run_cli(fps=bad)
                self.assertEqual(2, rc)
                self.assertIn('입력 오류', out)
                self.assertNotIn('Traceback', out)

    def test_36_non_positive_fps_is_a_usage_error(self):
        for bad in ('0', '-1', '-0.001'):
            self.assertEqual(2, self.run_cli(fps=bad)[0], bad)

    def test_37_pair_arguments_check_the_field_count(self):
        for arg, bad in (('center', '1'), ('center', '1,2,3'),
                         ('axes', '5'), ('axes', '1,2,3'),
                         ('range', '610'), ('range', '610,895,900')):
            with self.subTest(arg=arg, value=bad):
                rc, out = self.run_cli(fps=FPS, **{arg: bad})
                self.assertEqual(2, rc)
                self.assertNotIn('Traceback', out)

    def test_38_non_finite_or_non_numeric_geometry_is_a_usage_error(self):
        for arg, bad in (('center', 'nan,531'), ('center', '439,inf'),
                         ('axes', 'nan,178'), ('range', 'nan,895')):
            self.assertEqual(2, self.run_cli(fps=FPS, **{arg: bad})[0], bad)

    def test_39_zero_or_negative_axes_are_a_usage_error(self):
        """축이 0 이면 정규화에서 0 으로 나눈다 — 계산 전에 막는다."""
        for bad in ('0,178', '112,0', '-112,178'):
            self.assertEqual(2, self.run_cli(fps=FPS, axes=bad)[0], bad)

    def test_40_reversed_or_negative_frame_range_is_a_usage_error(self):
        for bad in ('895,610', '610,610', '-10,895', '610.5,895'):
            self.assertEqual(2, self.run_cli(fps=FPS, range=bad)[0], bad)

    def test_45_argparse_own_rejection_also_exits_with_2(self):
        """`--axes -112,178` 처럼 값이 옵션으로 보이면 argparse 가 먼저 잡는다.

        argparse 는 `-1`·`-0.001` 같은 **순수 음수만** 값으로 받아 준다(그건 우리
        `parse_inputs` 가 잡는다). 나머지는 여기서 끝나며, 그 경로의 종료코드도 2 다.
        """
        with redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as caught:
                wv.main(self.BASE + ['--axes', '-112,178'],
                        series_fn=lambda *a, **k: series())
        self.assertEqual(2, caught.exception.code)

    def test_41_t0_frame_must_be_a_non_negative_integer(self):
        for bad in ('nan', '670.5', '-1', 'abc'):
            with self.subTest(t0=bad):
                buf, err = io.StringIO(), io.StringIO()
                with redirect_stdout(buf), redirect_stderr(err):
                    rc = wv.main(['watchdog_video.py', 'x.mov', '--t0-frame', bad,
                                  '--preset', '0807-1522', '--fps', str(FPS)],
                                 series_fn=lambda *a, **k: series())
                self.assertEqual(2, rc)
                self.assertNotIn('Traceback', buf.getvalue() + err.getvalue())

    def test_42_bag_ms_is_validated_too(self):
        for bad in ('nan', 'inf', 'abc'):
            self.assertEqual(2, self.run_cli(fps=FPS, bag_ms=bad)[0], bad)

    def test_43_the_valid_preset_run_still_reaches_a_verdict(self):
        """🔴 역회귀 — 검증을 넣다가 정상 경로를 막으면 도구가 죽는다."""
        rc, out = self.run_cli(fps=FPS, bag_ms=wv.RECORDED['bag_ms'])
        self.assertEqual(1, rc)                    # 판정 불능은 성공이 아니다
        self.assertIn(f"{wv.RECORDED['measured_ms']} ms", out)
        self.assertIn('관측계 차이', out)

    def test_44_early_eof_from_the_producer_is_undecidable_through_main(self):
        """생산자가 짧게 돌려주면 `main` 이 요청 구간과 대조해 판정 불능으로 보낸다."""
        short = [r for r in series() if r[0] < 860]
        self.assertEqual(1, self.run_cli(fps=FPS)[0], '정상 경로가 먼저 살아 있어야 한다')
        buf, err = io.StringIO(), io.StringIO()
        with redirect_stdout(buf), redirect_stderr(err):
            rc = wv.main(self.BASE + ['--fps', str(FPS)],
                         series_fn=lambda *a, **k: short)
        self.assertEqual(1, rc)
        self.assertIn('조기 EOF', buf.getvalue())


if __name__ == '__main__':
    unittest.main(verbosity=2)
