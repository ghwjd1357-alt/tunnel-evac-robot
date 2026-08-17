#!/usr/bin/env python3
"""Negative and inverse-regression tests for todo_d0_scan.py."""

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCANNER = ROOT / "tools/todo_d0_scan.py"


def fixture(phase, items, claim, markers):
    if phase == "D+0":
        filename, start, list_number = "JETSON_SETUP.md", "## 1. start", "9"
    else:
        filename, start, list_number = "D1_FIRST_STEP.md", "## 0. start", "7"
    marker_lines = "\n".join(f"TODO({phase}): 확인 — item {n}" for n in range(markers))
    rows = "\n".join(f"| {n} | item | method | §0 |" for n in range(1, items + 1))
    text = f"""# fixture
{start}
{marker_lines}
## {list_number}. `TODO({phase})` 전량 목록 — **{claim}건**
| # | 무엇 | 확인 방법 | 절 |
|---|---|---|---|
{rows}
## 99. end
"""
    return filename, text


class TodoScanTest(unittest.TestCase):
    def run_scan(self, phase, *, items, claim, markers, overrides=()):
        with tempfile.TemporaryDirectory() as tmp:
            filename, text = fixture(phase, items, claim, markers)
            Path(tmp, filename).write_text(text)
            return subprocess.run(
                [sys.executable, str(SCANNER), "--root", tmp, "--phase", phase,
                 *overrides], text=True, stdout=subprocess.PIPE,
                stderr=subprocess.PIPE, check=False,
            )

    def test_repository_d0_and_d1_contracts_pass(self):
        for phase in ("D+0", "D+1"):
            with self.subTest(phase=phase):
                result = subprocess.run(
                    [sys.executable, str(SCANNER), "--phase", phase], cwd=ROOT,
                    text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    check=False,
                )
                self.assertEqual(0, result.returncode, result.stdout + result.stderr)

    # ⚠ 아래 세 시험은 D+1 등록 계약 N 의 **양쪽 이웃(N±1)** 을 고정한다.
    #   2026-08-17 에 N 이 10 → 9 로 내려갔다 (구 #4 라이다 높이 종결, `773d1a7`).
    #   계약을 바꾸면 여기 경계도 같은 커밋에서 따라 내린다 — 안 내리면 이 시험이
    #   "9 는 실패해야 한다"고 우기며 정상 상태를 FAIL 로 만든다(08-17 에 실제로 그랬다).
    def test_d1_claim_off_by_one_fails(self):
        result = self.run_scan("D+1", items=9, claim=8, markers=9)
        self.assertEqual(1, result.returncode)
        self.assertIn("개수 불일치", result.stdout)

    def test_d1_extra_or_missing_item_fails_both_directions(self):
        for items, claim, markers in ((10, 10, 10), (8, 8, 8)):
            with self.subTest(items=items):
                result = self.run_scan("D+1", items=items, claim=claim, markers=markers)
                self.assertEqual(1, result.returncode)
                self.assertIn("등록 계약", result.stdout)

    def test_action_marker_increase_and_decrease_both_fail(self):
        for markers in (8, 10):
            with self.subTest(markers=markers):
                result = self.run_scan("D+1", items=9, claim=9, markers=markers)
                self.assertEqual(1, result.returncode)
                self.assertIn("실행 표식", result.stdout)

    def test_generic_algorithm_accepts_a_registered_d0_fixture(self):
        result = self.run_scan(
            "D+0", items=11, claim=11, markers=7,
            overrides=("--want-sites", "1"),
        )
        self.assertEqual(0, result.returncode, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main(verbosity=2)
