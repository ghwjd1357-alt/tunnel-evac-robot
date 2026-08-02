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

    def test_d1_claim_off_by_one_fails(self):
        result = self.run_scan("D+1", items=10, claim=9, markers=10)
        self.assertEqual(1, result.returncode)
        self.assertIn("개수 불일치", result.stdout)

    def test_d1_extra_or_missing_item_fails_both_directions(self):
        for items, claim, markers in ((11, 11, 11), (9, 9, 9)):
            with self.subTest(items=items):
                result = self.run_scan("D+1", items=items, claim=claim, markers=markers)
                self.assertEqual(1, result.returncode)
                self.assertIn("등록 계약", result.stdout)

    def test_action_marker_increase_and_decrease_both_fail(self):
        for markers in (9, 11):
            with self.subTest(markers=markers):
                result = self.run_scan("D+1", items=10, claim=10, markers=markers)
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
