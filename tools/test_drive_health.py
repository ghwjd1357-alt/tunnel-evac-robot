#!/usr/bin/env python3
"""`drive_health.py` 회귀.

🔴 **회귀의 기준은 "실제로 있었던 고장을 잡는가"** 다. 그래서 여기 들어가는 숫자는
전부 2026-08-21 실차 bag 에서 나온 실측이다 — 지어낸 값으로 통과시키면, 도구가
죽어도 초록이 뜬다.

  ① 21:32 리허설 — 직진 명령만 주는데 `/odom` 이 −0.058 rad/s 로 "휜다"고 했다.
     IMU 는 0 이라 했고, SLAM 은 로봇이 명령대로 곧게 갔다고 했다.
     역산하면 **오른쪽 계수 0.525** — 펌웨어가 한쪽당 엔코더 2개를 평균하므로
     (`deltaRight = 0.5*(dFR+dRR)`) **하나가 0 이면 그 쪽이 절반**이 된다.
  ② 17:17 리허설D — 같은 계산이 0.01 근처를 내놓던 정상 상태.

🔴 이 도구는 08-21 밤에 **회귀 없이** 커밋됐다(`32d2b74`). 이 파일이 그 구멍이다.
"""
import io
import unittest
from contextlib import redirect_stdout

import drive_health as dh


def run(fn, *a, **kw):
    """판정 함수를 돌려 (rc, 출력) 을 준다 — 문구까지 회귀에 넣기 위한 것."""
    buf = io.StringIO()
    with redirect_stdout(buf):
        rc = fn(*a, **kw)
    return rc, buf.getvalue()


class StraightVerdictTest(unittest.TestCase):
    """직진 좌우 판정 — `/odom` 하나로 계수를 역산한다."""

    # 08-21 실측: (bag, odom속도, odom ω, IMU ω)
    HEALTHY = (0.0958, +0.0031, +0.0031)   # rehearsalD_0821 17:17
    BROKEN = (0.0772, -0.0580, +0.0058)    # rehearsal2_0821  21:32
    # 🔴 08-22 에 ODOM_WHEEL_BASE 를 0.829 -> 0.859 로 재교정했다. 위 두 픽스처는
    #   **그 이전** 실측이라 옛 눈금으로 풀어야 0.525 가 나온다. 새 값으로 풀면
    #   0.507 이 되는데, 그건 도구가 틀린 게 아니라 우리가 잘못 물은 것이다.
    BASE = dh.WHEEL_BASE_PRE_0822

    def test_01_healthy_run_passes(self):
        rc, out = run(dh.straight_verdict, *self.HEALTHY)
        self.assertEqual(0, rc, out)
        self.assertIn('🟢', out)

    def test_02_the_real_failure_is_caught(self):
        """🔴 이걸 놓치면 도구가 존재할 이유가 없다."""
        rc, out = run(dh.straight_verdict, *self.BROKEN)
        self.assertEqual(1, rc, out)
        self.assertIn('오른쪽', out)

    def test_03_the_factor_is_a_half_not_just_low(self):
        """계수가 **0.5** 로 나와야 '엔코더 하나' 라는 결론이 선다."""
        rc, out = run(dh.straight_verdict, *self.BROKEN, base=self.BASE)
        r = self.BROKEN[1] * self.BASE / (2 * self.BROKEN[0])
        k = (1 - abs(r)) / (1 + abs(r))
        self.assertAlmostEqual(0.525, k, places=3)
        self.assertIn('0.525', out)
        self.assertIn('선두 가설', out)
        self.assertNotIn('엔코더 2개 중 하나가 안 센다.', out)  # §87.3 단정 금지

    def test_04_the_weak_side_is_not_hardcoded(self):
        """🔴 부정 회귀 — 부호를 뒤집으면 **왼쪽**이 나와야 한다.

        구판을 상상해 보면 쉽다: 실측이 마침 오른쪽이었으니 '오른쪽' 을 박아 두어도
        위 두 시험은 통과한다. 그러면 다음에 왼쪽이 고장 났을 때 도구가 거짓말한다.
        """
        rc, out = run(dh.straight_verdict, self.BROKEN[0], -self.BROKEN[1],
                      self.BROKEN[2])
        self.assertEqual(1, rc)
        self.assertIn('왼쪽', out)
        self.assertNotIn('오른쪽', out)

    def test_05_a_robot_that_did_not_move_is_undecidable_not_pass(self):
        """🔴 안 움직였는데 '정상' 이 뜨면 무장 실패가 초록으로 기록된다."""
        rc, out = run(dh.straight_verdict, 0.001, 0.0, 0.0)
        self.assertEqual(1, rc)
        self.assertIn('판정 불가', out)
        self.assertNotIn('🟢', out)

    def test_06_a_curve_the_imu_also_saw_is_flagged_as_suspect(self):
        """실제로 휘었으면 좌우 판정을 믿으면 안 된다 — 경고가 붙어야 한다."""
        _, out = run(dh.straight_verdict, 0.0772, -0.0580, -0.0550)
        self.assertIn('실제로 휘었다', out)


class TurnVerdictTest(unittest.TestCase):
    """제자리 회전 판정 — 08-21 실측 3벌."""

    def test_07_normal_drivetrain_passes(self):
        # M2 14:00 — odom 0.1283 ≈ IMU 0.1262
        rc, out = run(dh.verdict, 100 * (1 - 0.1283 / 0.1262), -0.0002, +0.45)
        self.assertEqual(0, rc, out)

    def test_08_the_broken_night_run_fails(self):
        # spin180b 21:26 — odom 0.0965 vs IMU 0.1456
        rc, out = run(dh.verdict, 100 * (1 - 0.0965 / 0.1456), -0.0160, +0.45)
        self.assertEqual(1, rc, out)
        self.assertIn('오른쪽', out)

    def test_09_the_weak_side_flips_with_the_turn_direction(self):
        """🔴 부정 회귀 — 같은 편차라도 **회전 방향이 반대면 약한 쪽도 반대**다.

        vl = d − ωL/2 · vr = d + ωL/2 → 약한 쪽을 가르는 것은 `d·ω` 의 부호다.
        `asym` 부호만 보고 정하면 시계방향 시행에서 좌우가 뒤바뀐다.
        """
        _, cw = run(dh.verdict, 0.0, -0.0160, -0.45)
        self.assertIn('왼쪽', cw)
        _, ccw = run(dh.verdict, 0.0, -0.0160, +0.45)
        self.assertIn('오른쪽', ccw)

    def test_10_it_no_longer_tells_you_to_tighten_things(self):
        """🔴 08-21 — 구판은 '미끄러진다, 조임 점검' 이라고 단정해 **로봇을 뜯게** 했다.

        실제 원인은 한쪽 엔코더가 0 을 내는 것이었고, 조여서 고칠 물건이 아니었다.
        """
        _, out = run(dh.verdict, 33.7, -0.0160, +0.45)
        self.assertNotIn('미끄러진다', out)
        self.assertIn('--straight', out)


class ConstantsTest(unittest.TestCase):

    def test_11_wheel_base_matches_the_firmware(self):
        """🔴 `.ino` 에서 읽어 대조한다 — 숫자를 여기 베껴 적으면 자기확인이 된다."""
        from tools.firmware_constants import firmware_double

        self.assertEqual(firmware_double('ODOM_WHEEL_BASE'), dh.ODOM_WHEEL_BASE)
        # 명령 경로 상수는 이 도구가 쓰면 안 되는 값이다.
        self.assertNotEqual(firmware_double('CMD_WHEEL_BASE'), dh.ODOM_WHEEL_BASE)

    def test_12_straight_speed_stays_at_the_proven_value(self):
        """0.10 m/s 는 M1 에서 안전이 증명된 속도다. 올리려면 근거가 따로 필요하다."""
        self.assertEqual(0.10, dh.STRAIGHT_SPEED)


if __name__ == '__main__':
    unittest.main()
