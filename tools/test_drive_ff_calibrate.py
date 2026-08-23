#!/usr/bin/env python3
"""`drive_ff_calibrate.py` 회귀.

🔴 **이 도구가 막아야 하는 것은 08-12 에 실제로 일어난 두 실수다** (검토 §60.1·§61.1):
  ① 명목 FF 를 실제 PWM 으로 쓰기 → `Kp` 보정이 실제로 값을 바꾸는지 본다
  ② 2점으로 계수를 확정하기 → 점이 모자라면 **산출 불능**이어야 한다

그리고 적분 무시 전제가 깨지는 자리(긴 창)에서 **거부**하는지도 본다.
bag I/O 는 타지 않는다 — `fit()`·`reconstruct_point()` 가 순수 함수다.
"""
import unittest

import drive_ff_calibrate as cal


def pt(cmd, v, window=6.0):
    return cal.reconstruct_point(cmd, v, window)


class ReconstructTest(unittest.TestCase):

    def test_01_feedforward_matches_the_firmware_formula(self):
        """`30 + (v−0.02)×1300`, `145` 에서 포화."""
        self.assertAlmostEqual(69.0, cal.feedforward_pwm(0.05), places=6)
        self.assertAlmostEqual(145.0, cal.feedforward_pwm(0.12), places=6)
        self.assertAlmostEqual(cal.LOW_SPEED_HOLD_PWM, cal.feedforward_pwm(0.02), places=6)

    def test_02_kp_correction_actually_moves_the_point(self):
        """🔴 §60.1 의 핵심 — 명목 FF 와 재구성 PWM 이 달라야 한다."""
        p = pt(0.12, 0.3269)
        self.assertAlmostEqual(145.0, p['ff_pwm'], places=6)
        self.assertLess(p['pwm'], p['ff_pwm'] - 5.0,
                        '과속 시행인데 Kp 보정이 PWM 을 안 내렸다')
        self.assertTrue(p['saturated'])

    def test_03_integral_headroom_is_reported_not_hidden(self):
        """적분을 "무시했다"가 아니라 "이만큼이라 무시한다"로 남긴다."""
        p = pt(0.12, 0.3269, window=9.3)
        self.assertGreater(p['i_possible_pwm'], 0.0)
        self.assertLessEqual(p['i_possible_pwm'], cal.INTEGRAL_PWM_LIMIT)

    def test_04_integral_time_constant_matches_the_review_number(self):
        """§61.1 이 든 71~77초가 이 식에서 나오는지 확인한다."""
        t = cal.integral_seconds(7.0, 0.0197)
        self.assertGreater(t, 60.0)
        self.assertLess(t, 90.0)


class FitTest(unittest.TestCase):

    @staticmethod
    def plant(pwm, slope=0.00282, c=17.97):
        return slope * (pwm - c)

    def three_good_points(self):
        pts = []
        for cmd in (0.04, 0.05, 0.12):
            ff = cal.feedforward_pwm(cmd)
            # 실제 속도는 plant 가 정한다 — Kp 보정 뒤 PWM 에서 일관되게 만든다.
            v = self.plant(ff)
            for _ in range(3):
                v = self.plant(ff + cal.WHEEL_KP * (cmd - v))
            pts.append(pt(cmd, v))
        pts.sort(key=lambda p: p['pwm'])
        return pts

    def test_11_three_points_produce_a_coefficient(self):
        v = cal.fit(self.three_good_points())
        self.assertTrue(v['ok'], v.get('reason'))
        self.assertGreater(v['ff_slope'], 100.0)
        self.assertLess(v['ff_slope'], 900.0)

    def test_12_two_points_are_refused(self):
        """🔴 §60.1 — 2점은 후보까지다. 확정으로 내보내지 않는다."""
        v = cal.fit(self.three_good_points()[:2])
        self.assertFalse(v['ok'])
        self.assertIn('2개', v['reason'])

    def test_13_long_window_is_refused(self):
        """적분 무시 전제가 깨지는 창에서는 산출하지 않는다."""
        pts = self.three_good_points()
        pts[0]['window_s'] = cal.MAX_WINDOW_S + 1.0
        v = cal.fit(pts)
        self.assertFalse(v['ok'])
        self.assertIn('적분', v['reason'])

    def test_14_recovers_the_slope_that_generated_the_data(self):
        """합성 plant 를 그대로 되찾아야 한다 — 못 찾으면 산출식이 틀린 것이다."""
        v = cal.fit(self.three_good_points())
        self.assertAlmostEqual(0.00282, v['slope'], places=4)
        ff, v_pred = cal.predict(v['ff_slope'], v)
        self.assertAlmostEqual(cal.TARGET_MPS, v_pred, places=3)

    def test_15_the_08_12_candidate_375_is_not_what_this_produces(self):
        """🔴 회귀의 본체 — 같은 데이터에서 375 가 아니라 300 대가 나와야 한다.

        375 는 명목 FF 를 실제 PWM 으로 쓴 결과였다(§60.1). Kp 보정을 넣으면
        값이 내려가고, 그 차이가 합격 상단을 가른다.
        """
        v = cal.fit(self.three_good_points())
        self.assertLess(v['ff_slope'], 360.0,
                        'Kp 보정을 넣었는데도 375 근처가 나온다 — 보정이 안 먹었다')

    def test_16_extrapolation_is_flagged(self):
        """목표를 사이에 두지 않으면 외삽이라고 말해야 한다."""
        pts = [pt(0.10, 0.30), pt(0.11, 0.32), pt(0.12, 0.34)]
        v = cal.fit(pts)
        self.assertTrue(v['ok'])
        self.assertFalse(v['brackets_target'])

    def test_17_degenerate_input_is_undecidable_not_a_number(self):
        pts = [pt(0.05, 0.13), pt(0.05, 0.13), pt(0.05, 0.13)]
        v = cal.fit(pts)
        self.assertFalse(v['ok'])

    def test_18_constants_match_the_firmware(self):
        self.assertEqual(30.0, cal.LOW_SPEED_HOLD_PWM)
        self.assertEqual(0.020, cal.MIN_EFFECTIVE_WHEEL_CMD)
        self.assertEqual(145.0, cal.FEEDFORWARD_MAX_PWM)
        self.assertEqual(30.0, cal.WHEEL_KP)
        self.assertEqual(5.0, cal.WHEEL_KI)
        self.assertEqual(20.0, cal.INTEGRAL_PWM_LIMIT)
        self.assertEqual(1300.0, cal.FF_IN_EFFECT)


class ProfileGateTest(unittest.TestCase):
    """🔴 08-23 §91(2회차) P1-2 — 배너는 못 막는다. 실행 시 거부하는지 본다.

    1회차에 *"08-22 이후 bag 이면 산출 무효"* 를 **소스 주석으로만** 적었고, 검토가
    "실행 시 거부나 경고가 없다" 고 지적했다. 도구를 돌리는 사람은 소스를 안 읽는다.
    """

    def test_10_no_profile_is_refused(self):
        """프로필 없이 부르면 rc=2 — 산출을 아예 안 한다."""
        self.assertEqual(2, cal.main(['--point', '0.05:bag:685']))

    def test_11_unknown_profile_is_refused(self):
        """🔴 `post-0822` 는 **아직 상수가 없다.** 있는 척 통과시키면 안 된다."""
        self.assertEqual(2, cal.main(['--profile', 'post-0822',
                                      '--point', '0.05:bag:685']))

    def test_12_profile_flag_without_value_is_refused(self):
        """`--profile` 만 주고 값이 없는 경우도 거부."""
        self.assertEqual(2, cal.main(['--profile']))

    def test_13_known_profile_gets_past_the_gate(self):
        """역회귀 — `pre-0822` 는 게이트를 지난다(점이 없어 usage 로 끝나도 rc=2 는
        같지만, 거부 사유가 프로필이 아니라 입력이라는 것은 상수로 확인한다)."""
        self.assertIn('pre-0822', cal._PROFILES)
        self.assertEqual(0.12, cal._PROFILES['pre-0822']['TARGET_MPS'])
        self.assertEqual(20.0, cal._PROFILES['pre-0822']['INTEGRAL_PWM_LIMIT'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
