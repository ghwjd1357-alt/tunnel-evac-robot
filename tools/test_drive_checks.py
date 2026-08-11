#!/usr/bin/env python3
"""`drive_encoder_check.py` · `drive_ground_report.py` 회귀.

🔴 **회귀의 기준은 "실제로 있었던 고장을 잡는가"** 다. 2026-08-11 지면 세션에서 나온
두 고장을 합성 열로 재현해 **판정이 실제로 뒤집히는지**를 본다 — 통과만 확인하는
테스트는 도구가 죽어도 초록이다.

  ① 좌전륜 엔코더 부호 반전 — `deltaLeft` 가 0 이 돼 거리 절반·없던 회전 69.5°/5초
  ② 명령 대비 실제 속도 2.6배 — odom 은 줄자와 맞는데 명령과 안 맞는다

bag I/O 는 타지 않는다. 두 도구 모두 `analyze()` 를 순수 함수로 갈라 놨다.
"""
import math
import unittest

import drive_encoder_check as ec
import drive_ground_report as gr

NS = 1_000_000_000


# ── 합성 열 만들기 ──────────────────────────────────────────────────────
def roll_segment(t_start, sign, turns=3.0, n=40, dt=0.05):
    """바퀴 하나를 `turns` 만큼 굴린 구간의 `/odom` 열.

    한 바퀴만 굴리면 Δyaw = ±0.5*d/WHEEL_BASE 이고 Δdist = 0.25*d 다.
    `sign` = 그 바퀴가 **어느 쪽으로 적분되는가**(좌 −, 우 +).
    """
    d = turns * 2 * math.pi * ec.WHEEL_RADIUS_M
    dyaw = sign * 0.5 * d / ec.WHEEL_BASE_M
    rows = []
    for i in range(n + 1):
        f = i / n
        rows.append((int(t_start + i * dt * NS), 0.25 * d * f, 0.0, dyaw * f))
    return rows


def quiet(t_start, x, y, yaw, seconds=3.0, dt=0.05):
    """완전 정지 구간 — 구간을 가르는 것은 이 침묵뿐이다."""
    n = int(seconds / dt)
    return [(int(t_start + i * dt * NS), x, y, yaw) for i in range(n)]


def four_wheel_bag(signs, turns=(3.0, 3.0, 3.0, 3.0), ns=(40, 40, 40, 40)):
    """네 바퀴를 순서대로 굴린 `/odom` 열. `signs` = 각 바퀴가 적분되는 쪽.

    `ns` 를 줄이면 같은 회전량을 더 적은 표본에 담아 **표본당 이동이 커진다** —
    구간은 잡히지만 총량이 작은 "거의 안 도는 바퀴"를 그렇게 만든다.
    """
    rows, t = [], 0
    x = y = yaw = 0.0
    for s, tn, n in zip(signs, turns, ns):
        seg = roll_segment(t, s, turns=tn, n=n)
        # 앞 구간 끝점을 이어 붙인다(누적).
        seg = [(tt, x + sx, y + sy, yaw + syaw) for tt, sx, sy, syaw in seg]
        rows += seg
        x, y, yaw = seg[-1][1], seg[-1][2], seg[-1][3]
        t = seg[-1][0] + int(0.05 * NS)
        q = quiet(t, x, y, yaw)
        rows += q
        t = q[-1][0] + int(0.05 * NS)
    return rows


class EncoderCheckTest(unittest.TestCase):
    """부호 배치 판정."""

    def test_01_normal_wiring_passes(self):
        rows = four_wheel_bag((-1, -1, +1, +1))
        v = ec.analyze(rows, ec.segments(rows))
        self.assertTrue(v['ok'], v.get('reason'))
        self.assertEqual(['ok'] * 4, v['verdicts'])

    def test_02_front_left_flipped_is_caught(self):
        """🔴 2026-08-11 실제 고장 — 좌전륜이 우측으로 적분됐다."""
        rows = four_wheel_bag((+1, -1, +1, +1))
        v = ec.analyze(rows, ec.segments(rows))
        self.assertFalse(v['ok'], '부호 반전을 통과시키면 안 된다')
        self.assertEqual('flipped', v['verdicts'][0])
        self.assertEqual([0], v['bad'])

    def test_03_any_single_wheel_flip_is_caught(self):
        """네 자리 어디서 뒤집혀도 잡아야 한다 — 좌전륜만 보는 도구가 아니다."""
        for i in range(4):
            signs = list(ec.EXPECTED_SIGNS)
            signs[i] = -signs[i]
            rows = four_wheel_bag(tuple(signs))
            v = ec.analyze(rows, ec.segments(rows))
            self.assertFalse(v['ok'], f'{i}번 반전을 놓쳤다')
            self.assertEqual('flipped', v['verdicts'][i])

    def test_04_barely_responding_encoder_is_not_read_as_ok(self):
        """🔴 거의 반응이 없는 것을 "정상"으로 읽지 않는다 — 부호가 맞아도 통과 아니다.

        굴렸는데 `DEAD_DEG` 도 못 넘으면 그 바퀴는 **관측되지 않은 것**이다.
        부호만 보고 통과시키면 반쯤 죽은 엔코더가 초록으로 지나간다.
        """
        rows = four_wheel_bag((-1, -1, +1, +1),
                              turns=(3.0, 3.0, 0.05, 3.0), ns=(40, 40, 5, 40))
        v = ec.analyze(rows, ec.segments(rows))
        self.assertFalse(v['ok'], '거의 안 도는 바퀴를 통과시키면 안 된다')
        self.assertEqual('dead', v['verdicts'][2])
        self.assertEqual([2], v['bad'])

    def test_05_wrong_segment_count_is_undecidable_not_pass(self):
        """구간이 4개가 아니면 **판정 불능**이지 통과가 아니다."""
        rows = four_wheel_bag((-1, -1, +1))
        v = ec.analyze(rows, ec.segments(rows))
        self.assertFalse(v['ok'])
        self.assertIn('굴림 구간', v['reason'])

    def test_06_turn_count_matches_what_was_rolled(self):
        """회전수 환산이 실제로 굴린 양과 맞아야 계수 상수 오류를 잡을 수 있다."""
        rows = four_wheel_bag((-1, -1, +1, +1))
        v = ec.analyze(rows, ec.segments(rows))
        for r in v['rows']:
            self.assertAlmostEqual(3.0, r['turns'], places=2)

    def test_07_constants_match_the_firmware(self):
        """🔴 펌웨어와 같은 값이어야 한다 — 다르면 다른 물건을 재는 것이다."""
        self.assertEqual(0.62, ec.WHEEL_BASE_M)
        self.assertEqual(0.05698, ec.WHEEL_RADIUS_M)
        self.assertEqual(ec.WHEEL_BASE_M, gr.WHEEL_BASE_M)


# ── 지면 주행 리포터 ────────────────────────────────────────────────────
def straight_run(v_true, seconds, n=200, yaw_rate=0.0, t0=10 * NS):
    """등속 직진(또는 일정 회전)의 `/cmd_vel`·`/odom` 열."""
    dt = seconds / n
    cmds = [(int(t0 + i * 0.1 * NS), 0.05, 0.0) for i in range(int(seconds / 0.1))]
    odoms, x, y, yaw = [], 0.0, 0.0, 0.0
    for i in range(n + 1):
        t = int(t0 + i * dt * NS)
        odoms.append((t, x, y, yaw, v_true))
        x += v_true * dt * math.cos(yaw)
        y += v_true * dt * math.sin(yaw)
        yaw += yaw_rate * dt
    return cmds, odoms


class GroundReportTest(unittest.TestCase):
    """지면 실측 판정."""

    def test_11_straight_run_has_small_lateral(self):
        cmds, odoms = straight_run(0.12, 5.0)
        v = gr.analyze(cmds, odoms)
        self.assertTrue(v['ok'], v.get('reason'))
        self.assertLess(abs(v['lat_mm']), 1.0)
        self.assertLess(abs(v['dyaw_deg']), 0.1)

    def test_12_the_arc_from_a_flipped_encoder_shows_up(self):
        """🔴 2026-08-11 R1 1차 — 5초에 69.5° 가 적분됐다."""
        cmds, odoms = straight_run(0.14, 5.0, yaw_rate=0.225)
        v = gr.analyze(cmds, odoms)
        self.assertGreater(abs(v['dyaw_deg']), 60.0)
        self.assertGreater(v['lat_pct'], 20.0)

    def test_13_tape_is_the_judge_of_scale(self):
        """odom 이 줄자와 맞으면 스케일 정상, 어긋나면 잡는다."""
        cmds, odoms = straight_run(0.12, 5.0)
        ok = gr.analyze(cmds, odoms, tape_mm=v_path(odoms))
        self.assertAlmostEqual(1.0, ok['odom_over'], places=2)

        bad = gr.analyze(cmds, odoms, tape_mm=v_path(odoms) / 2.5)
        self.assertGreater(bad['odom_over'], 2.0)

    def test_14_real_speed_comes_from_the_tape_not_the_command(self):
        """🔴 예약 32 — 명령 0.05 인데 실제 0.123 이었다. 배율이 드러나야 한다."""
        cmds, odoms = straight_run(0.123, 5.55)
        v = gr.analyze(cmds, odoms, tape_mm=v_path(odoms))
        ratio = v['true_mps'] / abs(v['cmd_linear'])
        self.assertGreater(ratio, 2.0)

    def test_15_imu_disagreement_is_surfaced(self):
        """두 관측자가 어긋나면 그 사실이 판정에 남아야 한다."""
        cmds, odoms = straight_run(0.12, 5.0, yaw_rate=0.225)
        imu = [(odoms[0][0] - NS, 0.0), (odoms[-1][0] + NS, 2.0)]
        v = gr.analyze(cmds, odoms, imu=imu)
        self.assertIn('imu_dyaw_deg', v)
        self.assertGreater(abs(v['imu_dyaw_deg'] - v['dyaw_deg']), 10.0)

    def test_16_no_nonzero_command_is_undecidable(self):
        """무장이 안 돼 명령이 안 나간 시행을 "정지했다"로 읽지 않는다."""
        _, odoms = straight_run(0.0, 2.0)
        v = gr.analyze([(0, 0.0, 0.0)], odoms)
        self.assertFalse(v['ok'])
        self.assertIn('비영', v['reason'])

    def test_17_too_few_odom_samples_is_undecidable(self):
        cmds, odoms = straight_run(0.12, 5.0)
        v = gr.analyze(cmds, odoms[:1])
        self.assertFalse(v['ok'])


def v_path(odoms):
    """합성 열의 경로장(mm) — 기대값을 손으로 안 적기 위해 같은 방식으로 센다."""
    return sum(math.hypot(odoms[i][1] - odoms[i - 1][1], odoms[i][2] - odoms[i - 1][2])
               for i in range(1, len(odoms))) * 1000.0


if __name__ == '__main__':
    unittest.main(verbosity=2)
