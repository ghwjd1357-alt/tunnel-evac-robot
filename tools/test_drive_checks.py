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
    d = turns * 2 * math.pi * ec.ODOM_WHEEL_RADIUS_M
    dyaw = sign * 0.5 * d / ec.ODOM_WHEEL_BASE_M
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
        """🔴 펌웨어와 같은 값이어야 한다 — 다르면 다른 물건을 재는 것이다.

        🔴 08-13 (검토 §65.3) — 숫자를 여기 적지 않는다. 구판은
        `assertEqual(0.05698, ec.WHEEL_RADIUS_M)` 처럼 도구가 베낀 값을 시험도
        베껴 적어, 펌웨어가 바뀌어도 둘이 서로에게 초록을 주는 자기확인이었다.
        이제 `.ino` 에서 읽어 대조한다 — 펌웨어가 바뀌면 이 시험이 **깨진다**.
        """
        from tools.firmware_constants import firmware_double

        # 이 도구는 `/odom` 을 되짚는 도구다 → odom 계열 상수를 써야 한다.
        self.assertEqual(firmware_double('ODOM_WHEEL_BASE'), ec.ODOM_WHEEL_BASE_M)
        self.assertEqual(firmware_double('ODOM_WHEEL_RADIUS'), ec.ODOM_WHEEL_RADIUS_M)

        # 명령 경로 상수는 **이 도구가 쓰면 안 되는 값**이다. 둘이 서로 달라야 한다.
        self.assertNotEqual(firmware_double('CMD_WHEEL_BASE'),
                            firmware_double('ODOM_WHEEL_BASE'))
        # 🔴 08-13 밤 — `CONTROL_WHEEL_RADIUS` 는 예약 32-e 에서 사라졌다. 반지름이
        #    C10 실측(0.05698)으로 돌아오면서 odom 과 제어가 같은 눈금이 됐기 때문이다.
        #    그 이름이 되살아나면 반지름이 또 갈렸다는 뜻이므로 여기서 잡는다.
        with self.assertRaises(KeyError):
            firmware_double('CONTROL_WHEEL_RADIUS')

        # 판재 이전 profile 은 옛 값 그대로여야 옛 증거가 재현된다.
        self.assertEqual(0.62, ec.PRE_PLATE_WHEEL_BASE_M)
        self.assertEqual(0.05698, ec.PRE_PLATE_WHEEL_RADIUS_M)

        # 🔴 지면 리포터에는 윤거 상수가 없어야 한다 — 안 쓰는데 들고 있던 잔재였다.
        self.assertFalse(hasattr(gr, 'WHEEL_BASE_M'))


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

    # ── 08-12 실측 고장: bag 이 명령 앞부분을 못 받았다 ──────────────────
    def test_18_bag_missing_the_command_start_does_not_shrink_the_window(self):
        """🔴 2026-08-12 실측 — rosbag2 가 발행자를 늦게 발견해 앞 1.45s 가 빠졌다.

        줄자는 **시작 표시부터 최종 정지 위치까지**라 창이 늦게 시작하면 둘이 다른
        구간이 된다. 실측 배율이 `0.753` 으로 나왔고 움직임 구간으로 맞추자 `0.966`.
        여기서 보는 것은 **창이 줄자와 같은 구간을 덮는가**다.
        """
        cmds, odoms = run_with_coast(0.09, 5.0, 1.3, drop_lead_s=1.5)
        tape = v_path(odoms)                      # 줄자 = 움직인 전 구간
        v = gr.analyze(cmds, odoms, tape_mm=tape)
        self.assertTrue(v['ok'], v.get('reason'))
        self.assertAlmostEqual(1.0, v['odom_over'], places=2)
        self.assertGreater(v['cmd_lead_s'], 1.0, 'bag 결손이 판정에 안 남았다')

    def test_19_the_dropped_lead_would_have_broken_the_scale(self):
        """🔴 이 검사가 없으면 위 검사는 "그냥 통과"다 — 결손분이 실제로 크다는 증거.

        창을 기록된 첫 명령에서 잡았을 때의 경로장을 같이 세서, 그것이 줄자와
        **5% 가드를 넘게** 어긋난다는 것을 보인다(= 예전 동작이면 경보가 떴을 자리).
        """
        cmds, odoms = run_with_coast(0.09, 5.0, 1.3, drop_lead_s=1.5)
        tape = v_path(odoms)
        t0 = cmds[0][0]
        late = [o for o in odoms if o[0] >= t0]
        self.assertLess(v_path(late) / tape, 0.95,
                        '결손분이 5% 미만이면 이 회귀는 고장을 재현하지 못한다')

    def test_20_a_normal_run_is_left_alone(self):
        """역회귀 — 결손이 없으면 창도 판정도 예전 그대로여야 한다."""
        cmds, odoms = run_with_coast(0.09, 5.0, 1.3, drop_lead_s=0.0)
        v = gr.analyze(cmds, odoms, tape_mm=v_path(odoms))
        self.assertTrue(v['ok'], v.get('reason'))
        self.assertEqual(0.0, v['cmd_lead_s'])
        self.assertAlmostEqual(1.0, v['odom_over'], places=2)

    def test_21_cruise_excludes_the_coast_tail(self):
        """🔴 관성 꼬리를 정상구간에 넣으면 순항속도가 낮게 나온다(08-12 에 한 실수)."""
        cmds, odoms = run_with_coast(0.09, 5.0, 1.3, drop_lead_s=0.0)
        v = gr.analyze(cmds, odoms)
        self.assertAlmostEqual(0.09, v['cruise_mps'], places=3)

    def test_22_a_recorder_stall_does_not_truncate_the_window(self):
        """🔴 2026-08-12 실측(`r1_0812_1612`) — 주행 도중 기록이 0.343s 멈췄다.

        속도는 공백을 사이에 두고 `0.0513 → 0.0524` 로 이어졌는데도 구판은 거기서
        뒤로 걷기를 멈춰 창이 **2.09s 짧아졌고** 배율이 `0.818` 로 나왔다.
        로봇이 아니라 **기록이** 딸꾹질한 것이고, 창은 줄자와 같은 구간이어야 한다.
        """
        cmds, odoms = run_with_coast(0.09, 5.0, 1.3, drop_lead_s=1.5)
        odoms = drop_odom_window(odoms, at_s=2.0, dur_s=0.35)
        v = gr.analyze(cmds, odoms, tape_mm=v_path(odoms))
        self.assertTrue(v['ok'], v.get('reason'))
        self.assertGreater(v['cmd_lead_s'], 1.4)
        self.assertAlmostEqual(1.0, v['odom_over'], places=2)

    def test_23_a_real_stop_is_still_a_trial_boundary(self):
        """🔴 역회귀 — 딸꾹질 면역이 "정지도 무시한다"가 되면 안 된다.

        앞 시행이 **완전히 서 있다가** 다시 출발한 열에서는, 창이 앞 시행까지
        먹으면 안 된다. 경계를 만드는 것은 시간 공백이 아니라 **정지 표본**이다.
        """
        cmds, odoms = run_with_coast(0.09, 5.0, 1.3, drop_lead_s=1.5)
        prior = [(t - 20 * NS, 0.0, 0.0, 0.0, v) for (t, _, _, _, v) in odoms]
        mi = gr.motion_start_before(prior + odoms, cmds[0][0])
        self.assertIsNotNone(mi)
        self.assertGreater((prior + odoms)[mi][0], prior[-1][0])

    def test_24_a_gap_longer_than_the_hard_limit_still_cuts(self):
        """공백이 `MOTION_GAP_HARD_S` 를 넘으면 속도가 이어져 보여도 끊는다 —
        그 사이에 서 있었는지 알 방법이 없기 때문이다(과잉 신뢰 금지)."""
        # 공백은 반드시 **앵커보다 앞**에 둔다 — 뒤에 두면 뒤로 걷기가 지나가지도 않는다.
        cmds, odoms = run_with_coast(0.09, 6.0, 1.3, drop_lead_s=4.0)
        odoms = drop_odom_window(odoms, at_s=1.5,
                                 dur_s=gr.MOTION_GAP_HARD_S + 0.5)
        mi = gr.motion_start_before(odoms, cmds[0][0])
        self.assertIsNotNone(mi)
        # 창이 공백 **뒤**에서 시작해야 한다 = 알 수 없는 구간을 안 먹었다.
        self.assertGreater((odoms[mi][0] - odoms[0][0]) / NS, 3.0)

    def test_25_tape_anchored_cruise_beats_the_diluted_average(self):
        """🔴 평균속도는 가감속이 섞여 아래로 희석된다 — 짧은 주행일수록 심하다.

        합격선을 평균 하나로만 보면 **멀쩡한 계수가 불합격으로 보인다**(08-12 사용자
        지적). 줄자앵커 순항 = 순항 × (줄자/odom) 은 희석이 없고 스케일도 외부 앵커다.
        여기서는 ① 그 값이 실제 등속에 맞고 ② 평균은 그보다 **낮게** 나온다를 같이 본다.
        """
        cmds, odoms = run_with_coast(0.12, 5.0, 1.3, drop_lead_s=0.0)
        v = gr.analyze(cmds, odoms, tape_mm=v_path(odoms))
        self.assertAlmostEqual(0.12, v['cruise_true_mps'], places=3)
        self.assertLess(v['true_mps'], v['cruise_true_mps'])

    def test_25b_tape_anchor_divides_when_odom_is_inflated(self):
        """🔴 08-13 버그 회귀 — 앵커 방향이 뒤집혀 있었다.

        `cruise_true = 순항 × odom_over` 였는데 `odom_over = odom/줄자` 다.
        곱하면 odom 이 부풀어 있을수록 보정값이 **더** 부푼다 — 상쇄가 아니라 증폭이다.

        구판 시험(위 test_25)은 `tape_mm = odom 경로` 로 줘서 `odom_over == 1.0` 이었고,
        1 을 곱하나 나누나 같아 **버그를 통과시켰다**. 그래서 여기서는 odom 을 일부러
        부풀린다 — 08-13 실측과 같은 1.238 배다.

        실해: 이 자리가 실제 0.0976 m/s 를 0.1495 m/s 로 보고했다. 그 수는 🔴 예약 32-c
        위험 수용의 `0.12 m/s` 상한을 판정하는 데 쓰인다.
        """
        inflation = 1.238                        # 08-13 r2_line_0813_1516 실측
        cmds, odoms = run_with_coast(0.12, 20.0, 1.3, drop_lead_s=0.0)
        true_mm = v_path(odoms) / inflation      # 줄자 = odom 보다 짧다

        v = gr.analyze(cmds, odoms, tape_mm=true_mm)

        self.assertAlmostEqual(inflation, v['odom_over'], places=2)
        # 앵커된 순항은 odom 순항보다 **작아야** 한다. 곱셈 버그면 커진다.
        self.assertLess(v['cruise_true_mps'], v['cruise_mps'])
        self.assertAlmostEqual(v['cruise_mps'] / inflation,
                               v['cruise_true_mps'], places=4)
        # 그리고 줄자에 앵커된 평균속도와 같은 자리에 있어야 한다 (희석 몫만큼만 위).
        self.assertLess(v['true_mps'], v['cruise_true_mps'])
        self.assertLess(v['cruise_true_mps'], v['true_mps'] * 1.2)


def run_with_coast(v_true, cmd_s, coast_s, drop_lead_s=0.0, t0=10 * NS, dt=0.02):
    """명령 구간 등속 → 명령이 끊기면 선형 감속 → 정지.

    `drop_lead_s` = rosbag2 가 발행자를 아직 못 찾아 **bag 에 안 들어온** 앞 구간.
    `/odom` 은 처음부터 기록되므로(구독자가 이미 붙어 있다) 결손은 `/cmd_vel` 에만 난다 —
    08-12 실측이 정확히 이 모양이었다.
    """
    cmds = [(t0 + int(i * 0.1 * NS), 0.05, 0.0)
            for i in range(int(round(cmd_s / 0.1)))
            if i * 0.1 >= drop_lead_s]
    odoms, x = [], 0.0
    for i in range(int(round(0.5 / dt))):        # 출발 전 정지 구간
        odoms.append((t0 - int((0.5 - i * dt) * NS), 0.0, 0.0, 0.0, 0.0))
    for i in range(int(round((cmd_s + coast_s + 1.0) / dt)) + 1):
        el = i * dt
        if el <= cmd_s:
            v = v_true
        elif el <= cmd_s + coast_s:
            v = v_true * (1.0 - (el - cmd_s) / coast_s)
        else:
            v = 0.0
        odoms.append((t0 + int(el * NS), x, 0.0, 0.0, v))
        x += v * dt
    return cmds, odoms


def drop_odom_window(odoms, at_s, dur_s):
    """움직임 도중 `/odom` 표본이 `dur_s` 동안 통째로 빠진 열을 만든다.

    08-12 실측의 모양이다 — 기록이 잠깐 멈췄다가 밀린 표본을 한꺼번에 토했다.
    🔴 **속도는 공백 전후로 이어진다**(로봇은 계속 굴렀다). 지우는 것은 표본뿐이다.
    """
    t0 = odoms[0][0]
    lo = t0 + int(at_s * NS)
    hi = t0 + int((at_s + dur_s) * NS)
    return [o for o in odoms if not (lo < o[0] < hi)]


def v_path(odoms):
    """합성 열의 경로장(mm) — 기대값을 손으로 안 적기 위해 같은 방식으로 센다."""
    return sum(math.hypot(odoms[i][1] - odoms[i - 1][1], odoms[i][2] - odoms[i - 1][2])
               for i in range(1, len(odoms))) * 1000.0


if __name__ == '__main__':
    unittest.main(verbosity=2)
