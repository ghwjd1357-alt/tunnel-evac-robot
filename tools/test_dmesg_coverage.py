#!/usr/bin/env python3
"""`dmesg_coverage_check` 회귀 (검토 §76.2).

🔴 **이 시험의 존재 이유는 부정 A 한 줄이다** — 08-17 밤에 실제로 쓴 조합
(주행 **전** 스냅샷 + 그 뒤에 찍은 bag)이 반드시 FAIL 해야 한다. 그 조합이 통과하면
판정기가 §73.2·§76.2 를 또 못 잡는다.

성공 경로만 보는 시험은 미완성이므로(`AGENTS §3-7`) 경계 양쪽을 같이 고정한다:
로그가 1초 이르게 끝나도 FAIL, 정확히 맞닿으면 통과.
"""

import os
import subprocess
import sys
import tempfile
import time
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOL = os.path.join(ROOT, "tools", "dmesg_coverage_check.py")

#: 기준 시각 — 실제 08-17 soak 과 같은 모양으로 만든다.
BASE = time.mktime(time.strptime("2026-08-17 23:18:43", "%Y-%m-%d %H:%M:%S"))
SOAK_SECONDS = 744.696844165


def write_bag(directory, start_epoch=BASE, duration_s=SOAK_SECONDS):
    os.makedirs(directory, exist_ok=True)
    with open(os.path.join(directory, "metadata.yaml"), "w", encoding="utf-8") as fh:
        fh.write(
            "rosbag2_bagfile_information:\n"
            "  duration:\n"
            "    nanoseconds: %d\n"
            "  starting_time:\n"
            "    nanoseconds_since_epoch: %d\n"
            "  message_count: 110735\n" % (int(duration_s * 1e9), int(start_epoch * 1e9))
        )
    return directory


def write_log(path, first_epoch, last_epoch, extra=()):
    """양 끝 시각을 가진 `dmesg -T` 형식 로그를 만든다."""
    def stamp(epoch, text):
        return "[%s] %s\n" % (
            time.strftime("%a %b %e %H:%M:%S %Y", time.localtime(epoch)), text)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(stamp(first_epoch, "Booting Linux on physical CPU 0x0000000000"))
        for epoch, text in extra:
            fh.write(stamp(epoch, text))
        fh.write(stamp(last_epoch, "cdc_acm 1-2.1:1.0: ttyACM0: USB ACM device"))
    return path


def run(bag_dir, log_path):
    proc = subprocess.run(
        [sys.executable, TOOL, bag_dir, log_path],
        capture_output=True, text=True,
    )
    return proc.returncode, proc.stdout + proc.stderr


class DmesgCoverageTest(unittest.TestCase):

    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix="dmesg_cov_")
        self.bag = write_bag(os.path.join(self.dir, "bag"))
        self.log = os.path.join(self.dir, "dmesg.log")

    def test_negative_a_pre_run_snapshot_fails(self):
        """🔴 08-17 밤에 실제로 쓴 조합 — soak 시작 43초 전에 끝난 로그."""
        write_log(self.log, BASE - 6000, BASE - 43)
        rc, out = run(self.bag, self.log)
        self.assertEqual(1, rc, out)
        self.assertIn("덮지 못한다", out)
        self.assertIn("끝을", out)

    def test_negative_b_one_second_early_end_fails(self):
        """경계 — 1초만 이르게 끝나도 통과시키지 않는다."""
        write_log(self.log, BASE - 60, BASE + SOAK_SECONDS - 1)
        rc, out = run(self.bag, self.log)
        self.assertEqual(1, rc, out)

    def test_negative_late_start_fails(self):
        """반대쪽 경계 — 주행이 시작된 뒤에 뜬 로그도 못 쓴다."""
        write_log(self.log, BASE + 1, BASE + SOAK_SECONDS + 60)
        rc, out = run(self.bag, self.log)
        self.assertEqual(1, rc, out)
        self.assertIn("시작을", out)

    def test_negative_c_disconnect_inside_window_is_warning_not_failure(self):
        """판정(덮었는가)과 관측(무엇이 보이는가)을 한 출구로 합치지 않는다."""
        write_log(
            self.log, BASE - 60, BASE + SOAK_SECONDS + 60,
            extra=[(BASE + 100, "usb 1-2.1: USB disconnect, device number 8")],
        )
        rc, out = run(self.bag, self.log)
        self.assertEqual(0, rc, out)
        self.assertIn("눈여겨볼 줄", out)
        self.assertIn("USB disconnect", out)

    def test_reverse_1_covering_log_passes(self):
        """역회귀 — 주행 종료 뒤 받은 정상 조합은 통과해야 한다."""
        write_log(self.log, BASE - 3600, BASE + SOAK_SECONDS + 120)
        rc, out = run(self.bag, self.log)
        self.assertEqual(0, rc, out)
        self.assertIn("덮었다", out)

    def test_reverse_2_exact_boundary_passes(self):
        """양 끝이 정확히 맞닿으면 통과 — 경계를 과하게 조이지 않았음을 고정한다."""
        write_log(self.log, BASE, BASE + SOAK_SECONDS + 1)
        rc, out = run(self.bag, self.log)
        self.assertEqual(0, rc, out)

    def test_undecidable_when_dmesg_has_no_absolute_time(self):
        """`-T` 없이 저장한 로그는 FAIL 이 아니라 **판정 불능**(rc=2)이다."""
        with open(self.log, "w", encoding="utf-8") as fh:
            fh.write("[    0.000000] Booting Linux\n[ 1234.567890] usb 1-2.1: disconnect\n")
        rc, out = run(self.bag, self.log)
        self.assertEqual(2, rc, out)
        self.assertIn("판정 불능", out)

    def test_missing_metadata_is_undecidable(self):
        rc, out = run(os.path.join(self.dir, "nope"), self.log)
        self.assertEqual(2, rc, out)


if __name__ == "__main__":
    unittest.main()
