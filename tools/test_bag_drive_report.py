#!/usr/bin/env python3
"""`bag_drive_report.py` 회귀 — 08-21 실측을 그대로 박는다.

🔴 이 도구가 낸 결론(우측 합성 계측 비대칭)으로 **어느 쪽을 볼지 정한다.** 역산이 틀리면
아침 두 시간이 엉뚱한 곳으로 간다. 숫자는 전부 08-21 21:32 리허설 bag 에서 나온
실측이다 — 지어낸 값으로 통과시키면 도구가 죽어도 초록이다.
"""
import unittest

import bag_drive_report as B


class SolveKrTest(unittest.TestCase):
    """`/odom` 만으로 약한 쪽 계수를 역산한다."""

    # 08-21 실측 (직진 구간 중앙값)
    # 🔴 08-22 에 ODOM_WHEEL_BASE 를 0.829 -> 0.859 로 재교정했다. 아래 픽스처는
    #   **그 이전** bag 이므로 옛 눈금으로 풀어야 한다 — 새 값으로 풀면 0.525 가
    #   0.507 로 어긋나고, 그건 도구가 틀린 게 아니라 우리가 잘못 물은 것이다.
    BASE = B.WHEEL_BASE_PRE_0822
    BROKEN = (0.0772, -0.0580)     # rehearsal2_0821 21:32
    HEALTHY = (0.0958, +0.0031)    # rehearsalD_0821 17:17

    def test_b1_the_real_failure_solves_to_a_half(self):
        """🎯 이 숫자가 진단의 전부다 — ≈0.52 는 **합성** 비율이다(채널이 아니다)."""
        r, weak, k = B.solve_kr(*self.BROKEN, base=self.BASE)
        self.assertEqual('right', weak)
        self.assertAlmostEqual(0.525, k, places=3)
        self.assertAlmostEqual(-0.3114, r, places=4)

    def test_b2_the_healthy_run_is_balanced(self):
        r, weak, k = B.solve_kr(*self.HEALTHY, base=self.BASE)
        self.assertIsNone(weak, f'정상 시행을 고장으로 읽었다 (r={r:+.4f})')

    def test_b3_the_weak_side_is_not_hardcoded(self):
        """🔴 부정 회귀 — 부호를 뒤집으면 **왼쪽**이 나와야 한다.

        실측이 마침 오른쪽이었으니 'right' 를 박아 두어도 위 둘은 통과한다.
        그러면 다음에 왼쪽이 고장 났을 때 도구가 거짓말한다.
        """
        _, weak, k = B.solve_kr(self.BROKEN[0], -self.BROKEN[1], base=self.BASE)
        self.assertEqual('left', weak)
        self.assertAlmostEqual(0.525, k, places=3)

    def test_b4_a_robot_that_did_not_move_is_undecidable(self):
        """🔴 안 움직였는데 판정이 나오면 무장 실패가 진단으로 둔갑한다."""
        r, weak, k = B.solve_kr(0.0, -0.05)
        self.assertIsNone(r)
        self.assertIsNone(k)

    def test_b5_a_symmetric_radius_error_is_not_flagged_as_asymmetry(self):
        """🔴 **이 도구가 존재하는 이유** — 반지름 오차와 결손을 가른다.

        `ODOM_WHEEL_RADIUS` 가 틀리면 모든 바퀴 거리가 같은 계수로 곱해진다.
        직진 배율도 회전 배율도 똑같이 틀리지만 **좌우는 대칭**이라 phantom ω 가
        안 생긴다. 08-22 검토 전까지 `PITFALLS §18` 이 이 구분을 빠뜨리고 있었다.
        """
        # 진짜 0.10 m/s 인데 반지름이 35% 작게 잡혀 0.065 로 읽히는 상황
        r, weak, k = B.solve_kr(0.065, 0.0)
        self.assertIsNone(weak, '대칭 오차를 좌우 비대칭으로 오진했다')

    def test_b6_wheel_base_matches_the_firmware(self):
        """🔴 `.ino` 에서 읽어 대조한다 — 베껴 적으면 자기확인이 된다."""
        from tools.firmware_constants import firmware_double

        self.assertEqual(firmware_double('ODOM_WHEEL_BASE'), B.ODOM_WHEEL_BASE)
        self.assertNotEqual(firmware_double('CMD_WHEEL_BASE'), B.ODOM_WHEEL_BASE)


if __name__ == '__main__':
    unittest.main()
