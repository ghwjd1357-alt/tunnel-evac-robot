#!/usr/bin/env python3
"""`dmesg_coverage_check` 회귀 (검토 §76.2 · §77.2 · 예약 46).

🔴 **이 시험의 존재 이유는 부정 A 한 줄이다** — 08-17 밤에 실제로 쓴 조합
(주행 **전** 스냅샷 + 그 뒤에 찍은 bag)이 **절대 rc=0 이 되면 안 된다**. 그 조합이
통과하면 판정기가 §73.2·§76.2 를 또 못 잡는다.

§77.2 뒤 불변식이 하나 늘었다 — **약한 증거는 FAIL 은 만들 수 있어도 PASS 는 못 만든다.**
`--trust-mtime` 은 사람이 "원본이다" 라고 주장한 것뿐이라, mtime 이 bag 종료 뒤여도
`덮었다` 가 아니라 판정 불능(rc=2)이다. 반대로 마지막 커널 **사건**이 bag 종료 뒤인 것은
부등식이라 그것만으로 통과다.

성공 경로만 보는 시험은 미완성이므로(`AGENTS §3-7`) 양 끝 경계를 **정수초 fixture** 로
직접 맞춘다: 시작·종료 exact 는 통과, 각각 1초 어긋나면 갈린다.
"""

import json
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

#: 경계 시험 전용 — 정수초라 로그 stamp 와 bag 끝을 **정확히** 맞출 수 있다.
EXACT_SECONDS = 600.0


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
    """양 끝 **사건** 시각을 가진 `dmesg -T` 형식 로그를 만든다."""
    def stamp(epoch, text):
        return "[%s] %s\n" % (
            time.strftime("%a %b %e %H:%M:%S %Y", time.localtime(epoch)), text)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(stamp(first_epoch, "Booting Linux on physical CPU 0x0000000000"))
        for epoch, text in extra:
            fh.write(stamp(epoch, text))
        fh.write(stamp(last_epoch, "cdc_acm 1-2.1:1.0: ttyACM0: USB ACM device"))
    return path


def write_sidecar(log_path, capture_end, capture_start=None, sha=None):
    """`--capture` 가 남기는 취득 경계 sidecar 를 손으로 만든다."""
    import hashlib
    if sha is None:
        digest = hashlib.sha256()
        with open(log_path, "rb") as fh:
            digest.update(fh.read())
        sha = digest.hexdigest()
    info = {
        "tool": "dmesg_coverage_check", "version": 2,
        "capture_start": capture_start if capture_start is not None else capture_end - 1.0,
        "capture_end": capture_end, "sha256": sha,
        "log": os.path.basename(log_path),
    }
    with open(log_path + ".capture.json", "w", encoding="utf-8") as fh:
        json.dump(info, fh)
    return log_path + ".capture.json"


def run(bag_dir, log_path, *flags):
    proc = subprocess.run(
        [sys.executable, TOOL, bag_dir, log_path] + list(flags),
        capture_output=True, text=True,
    )
    return proc.returncode, proc.stdout + proc.stderr


class DmesgCoverageTest(unittest.TestCase):

    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix="dmesg_cov_")
        self.bag = write_bag(os.path.join(self.dir, "bag"))
        self.log = os.path.join(self.dir, "dmesg.log")

    # ── 부정 A — 존재 이유 ────────────────────────────────────────────
    def test_negative_a_pre_run_snapshot_never_passes(self):
        """🔴 08-17 밤에 실제로 쓴 조합. 경계가 없으면 판정 불능, mtime 이면 FAIL."""
        write_log(self.log, BASE - 6000, BASE - 43)
        os.utime(self.log, (BASE - 8.5, BASE - 8.5))         # 실제 파일과 같은 모양
        rc, out = run(self.bag, self.log)
        self.assertEqual(2, rc, out)                          # ⓒ 취득 출처 없음
        self.assertIn("판정 불능", out)
        rc, out = run(self.bag, self.log, "--trust-mtime")
        self.assertEqual(1, rc, out)                          # ⓑ 취득이 주행 전
        self.assertIn("끝을", out)
        self.assertNotIn("덮었다", out)

    # ── 예약 46 완료판정 ⓐ~ⓔ ──────────────────────────────────────────
    def test_a_quiet_log_captured_after_run_passes(self):
        """ⓐ 마지막 **사건**이 주행 전이어도 **취득**이 주행 뒤면 통과한다."""
        write_log(self.log, BASE - 6000, BASE - 43)
        write_sidecar(self.log, BASE + SOAK_SECONDS + 5)
        rc, out = run(self.bag, self.log)
        self.assertEqual(0, rc, out)
        self.assertIn("덮었다", out)

    def test_b_capture_before_run_end_fails(self):
        """ⓑ 취득 자체가 주행 종료 전이면 FAIL."""
        write_log(self.log, BASE - 6000, BASE - 43)
        write_sidecar(self.log, BASE - 8.5)
        rc, out = run(self.bag, self.log)
        self.assertEqual(1, rc, out)
        self.assertIn("끝을", out)

    def test_c_no_capture_source_is_undecidable(self):
        """ⓒ 조용한 로그 + 취득 출처 없음 = 판정 불능. FAIL 로 단정하지 않는다."""
        write_log(self.log, BASE - 6000, BASE + 10)
        rc, out = run(self.bag, self.log)
        self.assertEqual(2, rc, out)
        self.assertIn("취득 경계가 없다", out)

    def test_c2_tampered_sidecar_is_undecidable(self):
        """sidecar 의 sha256 이 이 로그의 것이 아니면 rc=2 — 복사·변조를 통과시키지 않는다."""
        write_log(self.log, BASE - 6000, BASE - 43)
        write_sidecar(self.log, BASE + SOAK_SECONDS + 5, sha="00" * 32)
        rc, out = run(self.bag, self.log)
        self.assertEqual(2, rc, out)
        self.assertIn("sha256", out)

    # ── §78.2 — 재현된 두 구멍 ────────────────────────────────────────
    def test_c4_reversed_capture_interval_is_undecidable(self):
        """🔴 §78.2 재현 A — identity 없이 `start>end` 인 sidecar 가 rc=0 을 냈었다."""
        write_log(self.log, BASE - 6000, BASE - 43)
        write_sidecar(
            self.log, BASE + SOAK_SECONDS + 5,
            capture_start=BASE + SOAK_SECONDS + 500)       # 시작이 종료보다 뒤
        rc, out = run(self.bag, self.log)
        self.assertEqual(2, rc, out)
        self.assertNotIn("✅", out)

    def test_c5_non_finite_capture_time_is_undecidable_not_crash(self):
        """🔴 §78.2 재현 B — NaN 이 `fmt()` 를 터뜨려 traceback + **rc=1** 이 났었다.

        크래시가 rc=1 로 새면 *"못 덮었다"* 라는 판정이 되어 rc=1/rc=2 구분이 죽는다.
        """
        for bad in (float("nan"), float("inf")):
            write_log(self.log, BASE - 6000, BASE - 43)
            write_sidecar(self.log, bad, capture_start=bad)
            rc, out = run(self.bag, self.log)
            self.assertEqual(2, rc, out)
            self.assertNotIn("Traceback", out)

    def test_capture_refuses_to_overwrite_existing_pair(self):
        """🔴 §78.2 — 취득이 실패했을 때 옛 쌍이 이번 판정에 답하면 §73.2 의 재발이다."""
        out_path = os.path.join(self.dir, "captured.log")
        with open(out_path, "w", encoding="utf-8") as fh:
            fh.write("어제 받은 로그\n")
        proc = subprocess.run(
            [sys.executable, TOOL, "--capture", out_path], capture_output=True, text=True)
        self.assertEqual(2, proc.returncode, proc.stdout)
        self.assertIn("취득 거부", proc.stdout)
        self.assertEqual("어제 받은 로그\n", open(out_path, encoding="utf-8").read())

    def test_c3_broken_sidecar_is_undecidable(self):
        write_log(self.log, BASE - 6000, BASE - 43)
        with open(self.log + ".capture.json", "w", encoding="utf-8") as fh:
            fh.write("{not json")
        rc, out = run(self.bag, self.log)
        self.assertEqual(2, rc, out)

    def test_d_disconnect_inside_window_is_warning_not_failure(self):
        """ⓓ 판정(덮었는가)과 관측(무엇이 보이는가)을 한 출구로 합치지 않는다."""
        write_log(
            self.log, BASE - 60, BASE + SOAK_SECONDS + 60,
            extra=[(BASE + 100, "usb 1-2.1: USB disconnect, device number 8")],
        )
        rc, out = run(self.bag, self.log)
        self.assertEqual(0, rc, out)
        self.assertIn("눈여겨볼 줄", out)
        self.assertIn("USB disconnect", out)

    def test_e_no_absolute_time_is_undecidable(self):
        """ⓔ `-T` 없이 저장한 로그는 FAIL 이 아니라 **판정 불능**(rc=2)이다."""
        with open(self.log, "w", encoding="utf-8") as fh:
            fh.write("[    0.000000] Booting Linux\n[ 1234.567890] usb 1-2.1: disconnect\n")
        rc, out = run(self.bag, self.log)
        self.assertEqual(2, rc, out)
        self.assertIn("판정 불능", out)

    # ── §77.2 비대칭 — 약한 증거는 PASS 를 못 만든다 ──────────────────
    def test_trust_mtime_cannot_produce_pass(self):
        """🔴 mtime 이 bag 종료 뒤여도 `덮었다` 가 아니라 rc=2 다."""
        write_log(self.log, BASE - 6000, BASE - 43)
        after = BASE + SOAK_SECONDS + 300
        os.utime(self.log, (after, after))
        rc, out = run(self.bag, self.log, "--trust-mtime")
        self.assertEqual(2, rc, out)
        self.assertNotIn("✅", out)

    def test_trust_mtime_may_produce_fail(self):
        """반대 방향 — 재시험을 요구하는 결론은 약한 증거로도 낸다."""
        write_log(self.log, BASE - 6000, BASE - 43)
        os.utime(self.log, (BASE - 8.5, BASE - 8.5))
        rc, out = run(self.bag, self.log, "--trust-mtime")
        self.assertEqual(1, rc, out)

    def test_sidecar_beats_mtime(self):
        """검증된 경계가 있으면 mtime 주장은 쓰이지 않는다."""
        write_log(self.log, BASE - 6000, BASE - 43)
        write_sidecar(self.log, BASE + SOAK_SECONDS + 5)
        os.utime(self.log, (BASE - 8.5, BASE - 8.5))
        rc, out = run(self.bag, self.log, "--trust-mtime")
        self.assertEqual(0, rc, out)

    # ── 경계 — 정수초 fixture 로 양 끝 exact ±1초 ─────────────────────
    def test_boundary_end_exact_passes(self):
        bag = write_bag(os.path.join(self.dir, "exact"), duration_s=EXACT_SECONDS)
        write_log(self.log, BASE - 60, BASE + EXACT_SECONDS)   # 마지막 사건 = bag 종료
        rc, out = run(bag, self.log)
        self.assertEqual(0, rc, out)

    def test_boundary_end_one_second_early_needs_capture_boundary(self):
        """1초 이른 마지막 **사건**만으로는 못 덮었다고 못 한다 — 경계가 있어야 갈린다."""
        bag = write_bag(os.path.join(self.dir, "exact"), duration_s=EXACT_SECONDS)
        write_log(self.log, BASE - 60, BASE + EXACT_SECONDS - 1)
        rc, out = run(bag, self.log)
        self.assertEqual(2, rc, out)
        write_sidecar(self.log, BASE + EXACT_SECONDS - 1)      # 취득도 1초 이르다
        self.assertEqual(1, run(bag, self.log)[0])
        write_sidecar(self.log, BASE + EXACT_SECONDS)          # 취득이 정확히 끝에 닿는다
        self.assertEqual(0, run(bag, self.log)[0])

    def test_boundary_start_exact_passes(self):
        bag = write_bag(os.path.join(self.dir, "exact"), duration_s=EXACT_SECONDS)
        write_log(self.log, BASE, BASE + EXACT_SECONDS + 60)   # 첫 사건 = bag 시작
        rc, out = run(bag, self.log)
        self.assertEqual(0, rc, out)

    def test_boundary_start_one_second_late_fails(self):
        """반대쪽 경계 — 주행이 시작된 뒤에 뜬 로그는 링 버퍼가 앞을 못 덮는다."""
        bag = write_bag(os.path.join(self.dir, "exact"), duration_s=EXACT_SECONDS)
        write_log(self.log, BASE + 1, BASE + EXACT_SECONDS + 60)
        rc, out = run(bag, self.log)
        self.assertEqual(1, rc, out)
        self.assertIn("시작을", out)

    # ── 역회귀 ────────────────────────────────────────────────────────
    def test_reverse_1_covering_log_passes(self):
        """주행 종료 뒤 사건까지 있는 정상 조합은 sidecar 없이도 통과한다 (부등식)."""
        write_log(self.log, BASE - 3600, BASE + SOAK_SECONDS + 120)
        rc, out = run(self.bag, self.log)
        self.assertEqual(0, rc, out)
        self.assertIn("덮었다", out)

    def test_missing_metadata_is_undecidable(self):
        rc, out = run(os.path.join(self.dir, "nope"), self.log)
        self.assertEqual(2, rc, out)

    # ── 취득 모드 ────────────────────────────────────────────────────
    def test_capture_writes_boundary_sidecar(self):
        """`--capture` 는 로그와 경계를 같이 남긴다. 커널 접근이 막히면 rc=2 다."""
        out_path = os.path.join(self.dir, "captured.log")
        proc = subprocess.run(
            [sys.executable, TOOL, "--capture", out_path], capture_output=True, text=True)
        if proc.returncode == 2:
            self.assertIn("취득 실패", proc.stdout + proc.stderr)
            self.assertFalse(os.path.exists(out_path + ".capture.json"))
            self.skipTest("이 호스트는 `dmesg -T` 를 허용하지 않는다 (취득 실패 경로만 확인)")
        self.assertEqual(0, proc.returncode, proc.stdout + proc.stderr)
        info = json.loads(open(out_path + ".capture.json", encoding="utf-8").read())
        self.assertGreaterEqual(info["capture_end"], info["capture_start"])
        self.assertEqual(64, len(info["sha256"]))


if __name__ == "__main__":
    unittest.main()
