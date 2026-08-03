#!/usr/bin/env python3
"""Regression and negative tests for tools/ai_context.py (stdlib only)."""

from __future__ import annotations

import json
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
from collections import Counter
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import ai_context  # noqa: E402


def git(*args: str) -> str:
    return subprocess.run(
        ["git", *args], cwd=ROOT, check=True, text=True,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    ).stdout


class ContextRouterTest(unittest.TestCase):
    maxDiff = None

    def test_01_known_inventory_is_stable_and_unique(self):
        rows = json.loads((ROOT / "tools/ai_known_p0_p1.json").read_text())
        self.assertEqual(61, len(rows))
        self.assertEqual(61, len({row["id"] for row in rows}))

    def test_02_known_p0_p1_routing_recall_is_100_percent(self):
        rows = json.loads((ROOT / "tools/ai_known_p0_p1.json").read_text())
        missed, common_leaks = [], []
        common_refs = (
            *ai_context.COMMON, ai_context.IMPLEMENT_GATE, ai_context.REVIEW_GATE,
        )
        common_text = "\n".join(ai_context.read_ref(ref)[0] for ref in common_refs)
        for row in rows:
            packet = ai_context.build_packet([row["path"]], "implement")
            routed_sources = packet.split("\n## WORKING FILE", 1)[0]
            if row["anchor"] not in routed_sources:
                missed.append(row["id"])
            profiles, unknown = ai_context.classify_paths([row["path"]])
            self.assertEqual([], unknown, row["id"])
            profile_refs = [
                ref for profile in sorted(profiles)
                for ref in ai_context.PROFILE_REFS[profile]
            ]
            profile_text = "\n".join(
                ai_context.read_ref(ref)[0] for ref in profile_refs
            )
            if row["anchor"] not in profile_text:
                missed.append(f'{row["id"]}:profile-only')
            if row["anchor"] in common_text:
                common_leaks.append(row["id"])
        self.assertEqual([], missed, f"known P0/P1 routing misses: {missed}")
        self.assertEqual([], common_leaks, f"anchors masked by common refs: {common_leaks}")

        # Empty profile routing must recover none of the inventory from common refs.
        recovered = [row["id"] for row in rows if row["anchor"] in common_text]
        self.assertEqual([], recovered)

    def test_02b_history_scan_independently_counts_61_primary_findings(self):
        review_dir = Path("/home/minwoo/Desktop/개발현황/CODEX 현황")
        names = (
            "0719검토현황.md", "0720검토현황.md", "0723검토현황.md",
            "0729검토현황.md", "0801검토현황.md",
        )
        if not all((review_dir / name).exists() for name in names):
            self.skipTest("Desktop review history is unavailable on this machine")
        found = []
        for name in names:
            source = (review_dir / name).read_text()
            for line in source.splitlines():
                primary_heading = (
                    re.match(r"^### .*P(?:0|1)(?:/P2|·P2|→P1)? .*—", line)
                    or re.match(r"^### .*P1-[①②] .*—", line)
                    or re.match(r"^## \d+\. P[01] —", line)
                )
                primary_table = re.match(r"^\| P0(?:\([^)]*\))? \|", line)
                if (primary_heading and "충족 확인" not in line) or primary_table:
                    found.append((name, line))
        # Three primary findings use historical formats outside the rule above.
        specials = (
            ("0719검토현황.md", "### 4.1 P0 stale goal 취소 레이스"),
            ("0720검토현황.md", "| P2→P1 — service ready 후 Future 영구 미완료 |"),
            ("0720검토현황.md", "### 10.2 P1 재현 경로"),
        )
        for name, marker in specials:
            self.assertIn(marker, (review_dir / name).read_text())
            found.append((name, marker))
        self.assertEqual(61, len(found))
        history_counts = Counter(name[:4] for name, _line in found)
        inventory = json.loads((ROOT / "tools/ai_known_p0_p1.json").read_text())
        inventory_counts = Counter(row["id"].split("-", 1)[0] for row in inventory)
        self.assertEqual(history_counts, inventory_counts)

    def test_03_unknown_path_falls_back_to_all_safety_docs(self):
        packet = ai_context.build_packet(["new_package/novel.runtime"], "implement")
        self.assertIn("unknown=new_package/novel.runtime", packet)
        for ref in ai_context.FULL_SAFETY_DOCS:
            self.assertIn(f"SOURCE `{ref.path}`", packet)

    def test_04_mixed_known_unknown_still_falls_back(self):
        packet = ai_context.build_packet(
            ["src/mission_manager/mission_manager/mission_node.py", "mystery/file.zz"],
            "implement",
        )
        self.assertIn("profiles=mission", packet)
        self.assertIn("unknown=mystery/file.zz", packet)
        self.assertIn("SOURCE `docs/REAL_ROBOT_VALUES.md`", packet)

    def test_05_multiple_known_surfaces_are_unioned(self):
        packet = ai_context.build_packet(
            ["src/mission_manager/mission_manager/mission_node.py", "tools/lib_e2e.sh"],
            "implement",
        )
        self.assertIn("profiles=e2e,mission", packet)
        self.assertIn("SOURCE `docs/PITFALLS.md §8`", packet)
        self.assertIn("SOURCE `docs/TEST_GATES.md §2`", packet)

    def test_06_platform_and_real_robot_surfaces_keep_freeze_evidence(self):
        for path in (
            "src/mission_manager/mission_manager/mission_node.py",
            "src/tunnel_bringup/launch/real_bringup.launch.py",
            "src/tunnel_bringup/config/nav2_params_real.yaml",
            "maps/real/tunnel.yaml",
        ):
            with self.subTest(path=path):
                packet = ai_context.build_packet([path], "implement")
                self.assertIn("SOURCE `docs/FREEZE_MANIFEST.md`", packet)

    def test_07_real_bringup_keeps_measured_values(self):
        packet = ai_context.build_packet(
            ["src/tunnel_bringup/launch/real_bringup.launch.py"], "implement"
        )
        self.assertIn("SOURCE `docs/REAL_ROBOT_VALUES.md`", packet)
        self.assertIn("실효 0.62", packet)

    def test_08_role_gate_is_not_swapped(self):
        implement = ai_context.build_packet(["docs/AI_CONTEXT.md"], "implement")
        review = ai_context.build_packet(
            ["docs/AI_CONTEXT.md"], "review", base="8fcc1a2", target="ef25ad3"
        )
        self.assertIn("SOURCE `docs/TEST_GATES.md §1`", implement)
        self.assertIn("SOURCE `docs/TEST_GATES.md §7`", review)

    def test_09_review_diff_is_byte_for_byte_complete(self):
        expected = git(
            "diff", "--find-renames", "--find-copies", "--no-ext-diff",
            "8fcc1a2", "ef25ad3", "--",
        )
        paths = ai_context.changed_paths("8fcc1a2", "ef25ad3")
        packet = ai_context.build_packet(
            paths, "review", base="8fcc1a2", target="ef25ad3"
        )
        actual = packet.split("```diff\n", 1)[1].rsplit("\n```\n", 1)[0]
        self.assertEqual(expected.rstrip("\n"), actual.rstrip("\n"))

    def test_10_review_rename_path_is_classified(self):
        profiles, unknown = ai_context.classify_paths(["src/mission_manager/x.py"])
        self.assertEqual({"mission"}, profiles)
        self.assertEqual([], unknown)

    def test_11_section_extraction_stops_at_peer_heading(self):
        text, first, last, _ = ai_context.read_ref(
            ai_context.Ref("docs/PROJECT_CONTEXT.md", "4")
        )
        self.assertIn("## 4.", text)
        self.assertIn("### 4.1", text)
        self.assertNotIn("## 5.", text)
        self.assertLess(first, last)

    def test_12_source_provenance_is_emitted(self):
        packet = ai_context.build_packet(["docs/AI_CONTEXT.md"], "implement")
        self.assertRegex(packet, r"lines \d+-\d+, sha256=[0-9a-f]{12}")

    def test_13_reservation_23_implementation_is_untouched(self):
        old = git("show", "ef25ad3:tools/handoff_single_check.sh")
        now = (ROOT / "tools/handoff_single_check.sh").read_text()
        self.assertEqual(old, now)

        def reservation(text: str, start: str, end: str) -> str:
            return text.split(start, 1)[1].split(end, 1)[0]

        old_master = git("show", "ef25ad3:docs/MASTER_PLAN.md")
        new_master = (ROOT / "docs/MASTER_PLAN.md").read_text()
        self.assertEqual(
            reservation(old_master, "23. **", "\n## 8."),
            reservation(new_master, "23. **", "\n## 8."),
        )
        old_handoff = git("show", "ef25ad3:docs/CURRENT_HANDOFF.md")
        new_handoff = (ROOT / "docs/CURRENT_HANDOFF.md").read_text()
        self.assertEqual(
            reservation(old_handoff, "- **⚠ 미해결 보류", "\n- **⏸ 6단 보류"),
            reservation(new_handoff, "- **⚠ 미해결 보류", "\n- **⏸ 6단 보류"),
        )

    def test_14_cold_start_repository_input_is_at_least_30_percent_smaller(self):
        old = "".join(
            git("show", f"ef25ad3:{path}")
            for path in ("AGENTS.md", "CLAUDE.md", "docs/CURRENT_HANDOFF.md")
        )
        new = "".join(
            (ROOT / path).read_text()
            for path in ("AGENTS.md", "CLAUDE.md", "docs/CURRENT_HANDOFF.md")
        )
        metrics = {
            "utf8_bytes": (len(old.encode()), len(new.encode())),
            "unicode_chars": (len(old), len(new)),
            "whitespace_words": (len(old.split()), len(new.split())),
        }
        failures = {
            name: (before, after, 1 - after / before)
            for name, (before, after) in metrics.items()
            if after > before * 0.70
        }
        self.assertEqual({}, failures, f"less than 30% reduction: {failures}")

    @unittest.skipUnless(
        shutil.which("node")
        and Path("/usr/share/code/resources/app/extensions/copilot/dist/")
        .joinpath("o200k_base.tiktoken").exists(),
        "local o200k tokenizer is not installed",
    )
    def test_15_local_o200k_raw_tokens_are_at_least_30_percent_smaller(self):
        paths = ("AGENTS.md", "CLAUDE.md", "docs/CURRENT_HANDOFF.md")
        old = "".join(git("show", f"ef25ad3:{path}") for path in paths)
        new = "".join((ROOT / path).read_text() for path in paths)

        def count(text: str) -> int:
            result = subprocess.run(
                ["node", "tools/local_token_count.js"], cwd=ROOT, input=text,
                text=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            )
            return int(result.stdout.strip())

        before, after = count(old), count(new)
        self.assertLessEqual(after, before * 0.70, (before, after, 1 - after / before))

    def test_16_missing_local_tokenizer_assets_fail_closed(self):
        result = subprocess.run(
            ["node", "tools/local_token_count.js", "/no/worker", "/no/vocab"],
            cwd=ROOT, input="hello", text=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        )
        self.assertEqual(2, result.returncode)
        self.assertIn("assets not found", result.stderr)

    def test_17_empty_review_range_fails_at_cli_boundary(self):
        result = subprocess.run(
            [sys.executable, "tools/ai_context.py", "review", "--base", "HEAD",
             "--target", "HEAD"],
            cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        )
        self.assertEqual(2, result.returncode)
        self.assertIn("no changed paths", result.stderr)

    def test_18_implementation_packet_includes_existing_target_file(self):
        packet = ai_context.build_packet(["tools/local_token_count.js"], "implement")
        self.assertIn("WORKING FILE `tools/local_token_count.js`", packet)
        self.assertIn("worker.postMessage", packet)

    def test_19_path_traversal_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "inside the repository"):
            ai_context.build_packet(["../outside-secret"], "implement")

    def test_20_full_document_supersedes_duplicate_section_excerpt(self):
        packet = ai_context.build_packet(["new/unknown.file"], "implement")
        self.assertEqual(1, packet.count("SOURCE `docs/TEST_GATES.md`"))
        self.assertNotIn("SOURCE `docs/TEST_GATES.md §1`", packet)

    def test_21_revision_that_looks_like_an_option_is_rejected(self):
        with self.assertRaises(RuntimeError):
            ai_context.changed_paths("--help", "HEAD")

    def test_22_binary_working_file_is_reported_without_decoding(self):
        packet = ai_context.build_packet(["maps/tunnel_map.pgm"], "implement")
        self.assertIn("WORKING FILE `maps/tunnel_map.pgm` — binary", packet)

    def test_23_missing_section_heading_fails_closed(self):
        with self.assertRaisesRegex(ValueError, "heading not found"):
            ai_context.read_ref(ai_context.Ref("docs/PROJECT_CONTEXT.md", "999"))

    def test_24_fenced_headings_do_not_truncate_sections(self):
        fixture = """# title
## 7. target
before
   ```bash
# 7 fake peer
## 8 fake peer
   ```
~~~text
# 9 another fake
~~~
after
## 8. real peer
outside
"""
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "fixture.md").write_text(fixture)
            with mock.patch.object(ai_context, "ROOT", root):
                text, _first, _last, _digest = ai_context.read_ref(
                    ai_context.Ref("fixture.md", "7")
                )
        self.assertIn("# 7 fake peer", text)
        self.assertIn("# 9 another fake", text)
        self.assertIn("after", text)
        self.assertNotIn("## 8. real peer", text)
        self.assertIsNone(ai_context._heading_number("## "))

    def test_25_real_routed_sections_keep_balanced_fences(self):
        refs = (
            ai_context.Ref("docs/TEST_GATES.md", "1"),
            ai_context.Ref("docs/JETSON_SETUP.md", "3"),
            ai_context.Ref("docs/JETSON_SETUP.md", "7"),
            ai_context.Ref("docs/D1_FIRST_STEP.md", "2"),
        )
        for ref in refs:
            with self.subTest(ref=ref):
                text = ai_context.read_ref(ref)[0]
                fences = sum(
                    1 for line in text.splitlines()
                    if re.match(r"^\s*(`{3,}|~{3,})", line)
                )
                self.assertEqual(0, fences % 2, f"unbalanced fences in {ref}")
        packet = ai_context.build_packet(
            ["src/mission_manager/mission_manager/mission_node.py"], "implement"
        )
        self.assertIn("기준선 (**08-02 재갱신 4**", packet)

    def test_26_one_path_unions_every_matching_profile(self):
        profiles, unknown = ai_context.classify_paths(["docs/JETSON_SETUP.md"])
        self.assertEqual({"bringup", "docs"}, profiles)
        self.assertEqual([], unknown)
        packet = ai_context.build_packet(["docs/JETSON_SETUP.md"], "implement")
        self.assertIn("profiles=bringup,docs", packet)
        self.assertIn("SOURCE `docs/PROJECT_CONTEXT.md §8`", packet)
        self.assertIn("SOURCE `docs/TEST_GATES.md §7`", packet)
        for ref in ai_context.PROFILE_REFS["bringup"]:
            suffix = f" §{ref.section}" if ref.section else ""
            self.assertIn(f"SOURCE `{ref.path}{suffix}`", packet)

    def test_27_watchdog_commands_require_an_uncommanded_observation_window(self):
        text = ai_context.read_ref(ai_context.Ref("docs/JETSON_SETUP.md", "7"))[0]
        watchdog = text.split("#### 7-c-0.", 1)[1].split("#### 7-c-R1.", 1)[0]
        blocks = list(re.finditer(r"```bash\n(.*?)```", watchdog, re.S))
        pub_blocks = [match for match in blocks if "ros2 topic pub" in match.group(1)]
        self.assertEqual(2, len(pub_blocks))
        self.assertTrue(all(match.group(1).count("ros2 topic pub") == 1
                            for match in pub_blocks))
        self.assertIn("--times 30", pub_blocks[0].group(1))
        self.assertIn("linear: {x: 0.05", pub_blocks[0].group(1))
        self.assertNotIn("linear: {x: 0.0,", pub_blocks[0].group(1))
        self.assertIn("--times 3", pub_blocks[1].group(1))
        self.assertIn("linear: {x: 0.0,", pub_blocks[1].group(1))
        between = watchdog[pub_blocks[0].end():pub_blocks[1].start()]
        self.assertIn("2초 이상 관찰", between)
        self.assertIn("마친 뒤에만", between)

        r1 = text.split("#### 7-c-R1.", 1)[1].split("#### 7-c-1.", 1)[0]
        r1_blocks = re.findall(r"```bash\n(.*?)```", r1, re.S)
        self.assertTrue(any(block.count("ros2 topic pub") == 2 for block in r1_blocks))

    def test_28_watchdog_evidence_has_one_physical_authority(self):
        d1 = (ROOT / "docs/D1_FIRST_STEP.md").read_text()
        real = (ROOT / "docs/REAL_ROBOT_VALUES.md").read_text()
        freeze = (ROOT / "docs/FREEZE_MANIFEST.md").read_text()
        self.assertNotIn("R0 의 미결 하나를 잊지 말 것", d1)
        self.assertNotIn("조건부 수용의 **정지 조건은 충족**", real)
        self.assertIn("cmd_vel watchdog 회신 참고값", real)
        self.assertIn("확정·재개방의 유일한 근거", real)
        self.assertIn("R0 실측에서 watchdog 이 확인되지 않으면", freeze)


if __name__ == "__main__":
    unittest.main(verbosity=2)
