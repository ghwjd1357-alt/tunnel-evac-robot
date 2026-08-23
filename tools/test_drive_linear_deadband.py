#!/usr/bin/env python3
"""`drive_linear_deadband.py` 의 `--steps` 입력 검증 회귀 (2026-08-23 §91 2회차 P1-2).

🔴 **왜 필요한가** — `--steps` 값은 곧바로 `/cmd_vel` 로 나가는 **실제 주행 명령**이다.
구판은 `tuple(float(x) for x in text.split(','))` 한 줄이라 검증이 0 이었고,
음수·NaN·Inf·상한 초과·역순이 전부 통과했다(검토가 `0.200001` 과 역순으로 확인).
불감대 산출은 "낮은 쪽부터 올려 처음 움직인 지점" 을 찾는 절차라, 순서가 섞이면
`D_lin` 자체가 의미를 잃는다 — 그래서 오름차순도 계약이다.
"""
import unittest

import drive_linear_deadband as db


class ParseStepsTest(unittest.TestCase):

    def test_01_normal_ascending_passes(self):
        self.assertEqual((0.04, 0.10, 0.20), db.parse_steps('0.04,0.10,0.20'))

    def test_02_boundary_at_the_cap_passes(self):
        """상한 **정확히** 는 통과한다 — 경계는 허용이다."""
        self.assertEqual((db.MAX_LINEAR_CMD,), db.parse_steps(str(db.MAX_LINEAR_CMD)))

    def test_03_just_over_the_cap_is_refused(self):
        """검토가 넣은 그 값."""
        with self.assertRaises(db.UsageError):
            db.parse_steps('0.200001')

    def test_04_negative_is_refused(self):
        with self.assertRaises(db.UsageError):
            db.parse_steps('-0.1,0.2')

    def test_05_zero_is_refused(self):
        """0 은 '정지 명령'이라 계단이 아니다."""
        with self.assertRaises(db.UsageError):
            db.parse_steps('0,0.1')

    def test_06_nan_and_inf_are_refused(self):
        for bad in ('nan', 'inf', '-inf'):
            with self.subTest(bad=bad), self.assertRaises(db.UsageError):
                db.parse_steps(f'0.05,{bad}')

    def test_07_non_numeric_is_refused(self):
        with self.assertRaises(db.UsageError):
            db.parse_steps('0.05,빠르게')

    def test_08_descending_is_refused(self):
        with self.assertRaises(db.UsageError):
            db.parse_steps('0.10,0.04')

    def test_09_duplicate_is_refused(self):
        """중복은 같은 계단을 두 번 타는 것 — 표본만 늘고 판정은 안 는다."""
        with self.assertRaises(db.UsageError):
            db.parse_steps('0.05,0.05')

    def test_10_empty_is_refused(self):
        for bad in ('', ',', '  ,  '):
            with self.subTest(bad=bad), self.assertRaises(db.UsageError):
                db.parse_steps(bad)

    def test_11_the_documented_default_still_parses(self):
        """🔴 도구 머리주석의 예시가 실제로 통과하는지 — 문서와 코드가 갈라지는 자리."""
        self.assertEqual(tuple(db.DEFAULT_STEPS),
                         db.parse_steps(','.join('%.2f' % s for s in db.DEFAULT_STEPS)))


if __name__ == '__main__':
    unittest.main(verbosity=2)
