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


REVIEW_HISTORY_GLOB = "*검토현황.md"
REVIEW_HISTORY_SPECIALS = (
    ("0719검토현황.md", "### 4.1 P0 stale goal 취소 레이스"),
    ("0720검토현황.md", "| P2→P1 — service ready 후 Future 영구 미완료 |"),
    ("0720검토현황.md", "### 10.2 P1 재현 경로"),
)
# ── cold-start 예산 (08-07 사용자 결정 — 비율 폐기, 절대 상한) ─────────────
# 차가운 세션이 강제로 읽는 세 파일. 여기 안 들어가는 사실은 정본으로 보내고 링크한다.
COLD_START_PATHS = ("AGENTS.md", "CLAUDE.md", "docs/CURRENT_HANDOFF.md")
# 🔴 08-11 사용자 결정 — **총량 완화 + 핸드오프 전용 상한 신설**. 이유는 숫자가 아니라
#   재는 대상이었다: 총량은 세 파일을 재는데 `AGENTS §5` 의 "현재 묶음 + 미해결 보류만"
#   규율이 걸리는 것은 **핸드오프 하나뿐**이다. 그래서 `AGENTS.md` 에 규칙 한 줄이 늘면
#   그 값을 무관한 핸드오프가 대신 냈다. 08-07 개정(비율→절대) 뒤 **4일 만에** 여유가
#   30 바이트로 돌아온 것이 그 증거다(08-07 은 41 바이트였다 — 같은 병이 재발한 것).
#   → 압력을 그것이 유효한 자리에만 남긴다. 핸드오프는 자기 상한으로 규율을 그대로 받고,
#     정본(`AGENTS.md`)의 정당한 증가는 더 이상 핸드오프 여유를 훔치지 않는다.
#   ⚠ **총량 42,000→45,000 · 토큰 13,500→14,500 은 완화다 — 숨기지 않는다.**
#   재개방 조건: ① 핸드오프가 **옮길 완료 서사가 없는데** 자기 상한에 반복해 걸릴 때
#   ② 총량 여유가 다시 5% 밑으로 갈 때(그때는 `AGENTS.md` 를 줄일 차례지 총량을 올릴
#   차례가 아니다). 색인 = `MASTER_PLAN §8`.
COLD_START_BYTE_BUDGET = 45_000
COLD_START_TOKEN_BUDGET = 14_500
# 핸드오프 단독 상한 — `AGENTS §5` 를 강제하는 자리는 여기다.
HANDOFF_BYTE_BUDGET = 20_000

WATCHDOG_ORIGIN = re.compile(r"(?:구동부(?:의)?\s*)?(?:\d+차\s*)?회신(?:값|수치)?")
WATCHDOG_AUTHORITY = re.compile(r"(?:충족|통과(?:\s*처리)?|확정|해소)")
WATCHDOG_SCOPE = re.compile(r"(?:watchdog|정지\s*조건|물리\s*증거|조건부\s*수용)", re.I)
WATCHDOG_DOWNGRADE = re.compile(
    r"(?:참고|근거가\s*아니|승격하지|대신하지|충족하지|확정하지|해소하지|"
    r"통과(?:\s*처리)?(?:하지|하면\s*안)|"
    r"만으로(?:는)?[^.!?]*(?:아니|안\s*(?:되|된|돼)))"
)
WATCHDOG_SLOT_START = "<!-- watchdog-evidence-slot:start -->"
WATCHDOG_SLOT_END = "<!-- watchdog-evidence-slot:end -->"
WATCHDOG_RESULT = re.compile(r"(?:^|\s|[|—-])(?:PASS|FAIL)(?:\s|$|[—:.-])")
WATCHDOG_MEASUREMENT = re.compile(r"\d+(?:\.\d+)?\s*(?:fps|프레임|ms|s\b|초)", re.I)
WATCHDOG_REPLY_ONLY = re.compile(
    r"(?:실측|측정)\s*(?:을\s*)?(?:생략|없이)|"
    r"회신(?:값|수치)?(?:만)?\s*(?:으로|만으로)"
)


# 🔴 후속 회차가 앞 회차의 P0/P1 을 **닫았다고 적은 제목**은 새 발견이 아니다 — 제목 형태가
#   발견과 같아서 스캐너가 한 건 더 세고, 그러면 재고(`ai_known_p0_p1.json`)와 갈라진다.
#   0729 §17.2 는 `충족 확인`, 0813 §64.1 은 `P1 닫힘` 으로 같은 뜻을 썼다. 🔴 계약을 맞추려고
#   **검토자 원문 제목을 고치지 않는다** — 스캐너가 두 표기를 다 안다(08-13 · 검토 §64).
# ⚠ `닫힘` 은 제목의 **상태 절(첫 `—` 앞)** 에서만 닫힘으로 읽는다 — 뒤쪽 본문 절에서는 진짜
#   발견이 그 낱말을 **인용**한다. 0801-3 §43.6 이 정확히 그런 P1 이다(*"미착수인데 …
#   `닫힘/완료`로 기록했고"*). 낱말이 어디 있느냐가 뜻을 가른다.
REVIEW_HISTORY_CLOSURE_MARKS = ("충족 확인",)
REVIEW_HISTORY_CLOSURE_STATUS_MARKS = ("닫힘",)


def review_history_findings(review_dir: Path):
    """Return every primary P0/P1 finding from every review-history file."""
    files = sorted(review_dir.glob(REVIEW_HISTORY_GLOB))
    found = []
    for path in files:
        source = path.read_text()
        for line in source.splitlines():
            primary_heading = (
                re.match(r"^### .*P(?:0|1)(?:/P2|·P2|→P1)? .*—", line)
                or re.match(r"^### .*P1-[①②] .*—", line)
                or re.match(r"^## \d+\. P[01] —", line)
            )
            primary_table = re.match(r"^\| P0(?:\([^)]*\))? \|", line)
            status = line.split("—", 1)[0]
            closure = any(m in line for m in REVIEW_HISTORY_CLOSURE_MARKS) or any(
                m in status for m in REVIEW_HISTORY_CLOSURE_STATUS_MARKS
            )
            if (primary_heading and not closure) or primary_table:
                found.append((path.name, line))
    return files, found


def watchdog_authority_paths(freeze_text: str):
    """Read the active-doc census itself so the scanner and inventory cannot drift."""
    census = freeze_text.split("**★ watchdog 활성 문서 전수 열거", 1)[1]
    census = census.split("**회귀 관찰값**", 1)[0]
    return tuple(re.findall(r"^\| `([^`]+\.md)` \|", census, re.M))


def watchdog_authority_documents(freeze_text: str, docs_dir: Path):
    """Load every census path, with a readable fail-closed error for stale paths."""
    documents = {}
    for path in watchdog_authority_paths(freeze_text):
        source_path = docs_dir / path
        if not source_path.is_file():
            raise AssertionError(f"active watchdog document is missing: {source_path}")
        documents[path] = source_path.read_text()
    return documents


def watchdog_record_slot_lines(source: str):
    """Return line indexes inside balanced evidence-slot markers; malformed means none."""
    lines = source.splitlines()
    slot_lines, opened = set(), None
    for index, line in enumerate(lines):
        if WATCHDOG_SLOT_START in line:
            if opened is not None:
                return set()
            opened = index
        if WATCHDOG_SLOT_END in line:
            if opened is None:
                return set()
            slot_lines.update(range(opened + 1, index))
            opened = None
    return set() if opened is not None else slot_lines


def markdown_table_cells(line: str):
    """Split Markdown cells without treating escaped or inline-code pipes as separators."""
    cells, cell, code_ticks = [], [], 0
    index = 0
    while index < len(line):
        char = line[index]
        if char == "\\" and index + 1 < len(line):
            cell.extend((char, line[index + 1]))
            index += 2
            continue
        if char == "`":
            run = 1
            while index + run < len(line) and line[index + run] == "`":
                run += 1
            if code_ticks == 0:
                code_ticks = run
            elif code_ticks == run:
                code_ticks = 0
            cell.extend("`" * run)
            index += run
            continue
        if char == "|" and code_ticks == 0:
            value = "".join(cell).strip()
            if value:
                cells.append(value)
            cell = []
        else:
            cell.append(char)
        index += 1
    value = "".join(cell).strip()
    if value:
        cells.append(value)
    return cells


def markdown_statements(source: str):
    """Return claim-sized text with evidence-slot location, including split claims."""
    statements, paragraph = [], []
    slot_lines = watchdog_record_slot_lines(source)

    def flush():
        if paragraph:
            joined = " ".join(text for text, _in_slot in paragraph)
            in_slot = all(flag for _text, flag in paragraph)
            sentences = [
                part for part in re.split(r"(?<=[.!?])\s+", joined) if part
            ]
            statements.extend((sentence, in_slot) for sentence in sentences)
            for left, right in zip(sentences, sentences[1:]):
                combined = f"{left} {right}"
                left_complete = all((
                    WATCHDOG_ORIGIN.search(left),
                    WATCHDOG_AUTHORITY.search(left),
                    WATCHDOG_SCOPE.search(left),
                ))
                right_complete = all((
                    WATCHDOG_ORIGIN.search(right),
                    WATCHDOG_AUTHORITY.search(right),
                    WATCHDOG_SCOPE.search(right),
                ))
                if not left_complete and not right_complete and all((
                    WATCHDOG_ORIGIN.search(combined),
                    WATCHDOG_AUTHORITY.search(combined),
                    WATCHDOG_SCOPE.search(combined),
                )):
                    statements.append((combined, in_slot))
            paragraph.clear()

    in_fence = False
    for index, raw in enumerate(source.splitlines()):
        line = raw.strip()
        if WATCHDOG_SLOT_START in line or WATCHDOG_SLOT_END in line:
            flush()
            continue
        if re.match(r"^(?:`{3,}|~{3,})", line):
            flush()
            in_fence = not in_fence
            continue
        if in_fence:
            continue
        if not line:
            flush()
        elif line.startswith("|"):
            flush()
            cells = markdown_table_cells(line)
            if cells:
                in_slot = index in slot_lines
                statements.extend((cell, in_slot) for cell in cells)
                statements.extend(
                    (f"{cells[0]} | {cell}", in_slot) for cell in cells[1:]
                )
        else:
            paragraph.append((line, index in slot_lines))
    flush()
    return statements


def measured_watchdog_record(statement: str):
    """Accept measured PASS/FAIL comparisons only inside document-owned record slots."""
    return (
        WATCHDOG_RESULT.search(statement)
        and WATCHDOG_MEASUREMENT.search(statement)
        and not WATCHDOG_REPLY_ONLY.search(statement)
    )


def watchdog_authority_violations(documents):
    """Find reply-only claims that improperly close the physical watchdog gate."""
    violations = []
    for path, source in documents.items():
        for statement, in_record_slot in markdown_statements(source):
            if not all((
                WATCHDOG_ORIGIN.search(statement),
                WATCHDOG_AUTHORITY.search(statement),
                WATCHDOG_SCOPE.search(statement),
            )):
                continue
            if in_record_slot and measured_watchdog_record(statement):
                continue
            if WATCHDOG_DOWNGRADE.search(statement):
                continue
            violations.append((path, statement))
    return violations


def git(*args: str) -> str:
    return subprocess.run(
        ["git", *args], cwd=ROOT, check=True, text=True,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    ).stdout


class ContextRouterTest(unittest.TestCase):
    maxDiff = None

    def test_01_known_inventory_is_stable_and_unique(self):
        rows = json.loads((ROOT / "tools/ai_known_p0_p1.json").read_text())
        self.assertEqual(189, len(rows))
        self.assertEqual(189, len({row["id"] for row in rows}))

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

    def test_02b_history_scan_independently_counts_189_primary_findings(self):
        review_dir = Path("/home/minwoo/Desktop/개발현황/CODEX 현황")
        files, found = review_history_findings(review_dir)
        if not files:
            self.skipTest("Desktop review history is unavailable on this machine")
        # Three primary findings use historical formats outside the rule above.
        for name, marker in REVIEW_HISTORY_SPECIALS:
            self.assertIn(marker, (review_dir / name).read_text())
            found.append((name, marker))
        self.assertEqual(189, len(found))
        history_counts = Counter(name[:4] for name, _line in found)
        inventory = json.loads((ROOT / "tools/ai_known_p0_p1.json").read_text())
        inventory_counts = Counter(row["id"].split("-", 1)[0] for row in inventory)
        self.assertEqual(history_counts, inventory_counts)

    def test_02c_history_scan_discovers_new_files_in_both_directions(self):
        with tempfile.TemporaryDirectory() as tmp:
            review_dir = Path(tmp)
            first = review_dir / "0801검토현황.md"
            first.write_text("### 1.1 P1 original — finding\n")
            files, found = review_history_findings(review_dir)
            self.assertEqual([first], files)
            self.assertEqual(1, len(found))

            extra = review_dir / "arbitrary-new-name-검토현황.md"
            extra.write_text("### 2.1 P1 newly found — finding\n")
            files, found = review_history_findings(review_dir)
            self.assertEqual([first, extra], files)
            self.assertEqual(2, len(found))

            extra.unlink()
            _files, found = review_history_findings(review_dir)
            self.assertEqual(1, len(found))

            first.write_text("### 1.1 P2 documentation only — finding\n")
            _files, found = review_history_findings(review_dir)
            self.assertEqual([], found)
            first.write_text("### 1.1 P1 original — finding\n")
            extra.write_text("### 2.1 P2 documentation only — finding\n")
            _files, found = review_history_findings(review_dir)
            self.assertEqual(1, len(found))

            # 검토 §40.4 — 계수 상한은 `###` **정확히 세 개**다. 같은 P1 을 한 단계 깊게 쓰면
            # 조용히 계약 밖으로 샌다(실제로 §39.6.1 이 `####` 로 쓰여 1건이 안 세어졌다).
            # 상한을 넓히려면 이 회귀를 먼저 바꾼다 — `AI_CONTEXT §3` 이 공개한 그 상한이다.
            first.write_text("#### 1.1 P1 deeper heading — finding\n")
            _files, found = review_history_findings(review_dir)
            self.assertEqual([], found)
            first.write_text("### 1.1 P1 original — finding\n")
            _files, found = review_history_findings(review_dir)
            self.assertEqual(1, len(found))

            outside_contract = review_dir / "0803_review.md"
            outside_contract.write_text("### 3.1 P1 outside suffix contract — finding\n")
            files, found = review_history_findings(review_dir)
            self.assertNotIn(outside_contract, files)
            self.assertEqual(1, len(found))

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
        """08-07: 통째→절 단위로 좁힌 뒤에도 **동결 증거의 실질**이 남는지 본다.

        구판은 `SOURCE …FREEZE_MANIFEST.md\\`` 라는 **표기**를 봤다. 절 단위로 바꾸자
        `…md §10\\`` 가 되어 검사가 깨졌는데, 정작 내용은 그대로였다 — 표기를 보는
        검사는 라우팅 모양이 바뀔 때마다 부서진다. 그래서 **실질**로 바꾼다.
        """
        for path in (
            "src/mission_manager/mission_manager/mission_node.py",
            "src/tunnel_bringup/launch/real_bringup.launch.py",
            "src/tunnel_bringup/config/nav2_params_real.yaml",
            "maps/real/tunnel.yaml",
        ):
            with self.subTest(path=path):
                packet = ai_context.build_packet([path], "implement")
                self.assertIn("SOURCE `docs/FREEZE_MANIFEST.md", packet)
                # 동결 예외 기록의 존재 이유 — 이 문장이 빠지면 증거절이 안 실린 것이다.
                self.assertIn("열 때마다 누가·무엇을·어디까지 열었는지 남긴다", packet)

    def test_07_real_bringup_keeps_measured_values(self):
        packet = ai_context.build_packet(
            ["src/tunnel_bringup/launch/real_bringup.launch.py"], "implement"
        )
        self.assertIn("SOURCE `docs/REAL_ROBOT_VALUES.md", packet)
        # 🔴 08-13 (검토 §65.3) — "실효 0.62" 라는 단일 이름은 폐기됐다. 그 문구가
        #    0.62 를 odom 보정 계수로 읽게 만들어 도구들이 잘못 베껴 갔다.
        #    패킷은 이제 **셋이 갈라져 있다는 사실**을 실어 날라야 한다.
        self.assertIn("명령 0.62", packet)
        self.assertIn("0.670", packet)

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

        def reservation(text: str, start: str, end_pattern: str) -> str:
            tail = text.split(start, 1)[1]
            boundary = re.search(end_pattern, tail, re.MULTILINE)
            self.assertIsNotNone(boundary, start)
            return tail[:boundary.start()]

        old_master = git("show", "ef25ad3:docs/MASTER_PLAN.md")
        new_master = (ROOT / "docs/MASTER_PLAN.md").read_text()
        self.assertEqual(
            reservation(old_master, "23. **", r"^(?:\d+\. \*\*|## 8\.)"),
            reservation(new_master, "23. **", r"^(?:\d+\. \*\*|## 8\.)"),
        )
        old_handoff = git("show", "ef25ad3:docs/CURRENT_HANDOFF.md")
        new_handoff = (ROOT / "docs/CURRENT_HANDOFF.md").read_text()
        self.assertEqual(
            reservation(old_handoff, "- **⚠ 미해결 보류 — 예약 23", r"^- \*\*"),
            reservation(new_handoff, "- **⚠ 미해결 보류 — 예약 23", r"^- \*\*"),
        )

    def test_14_cold_start_repository_input_fits_the_absolute_budget(self):
        """Cold start must fit a fixed budget — not a ratio against a 08-03 snapshot.

        08-07 개정 (사용자 결정). 구판은 `ef25ad3` 대비 **30% 작을 것**을 요구했다.
        기준점이 고정이라 프로젝트가 사실을 쌓을수록 **구조적으로 반드시 실패**하고,
        그때 압력이 "문서를 줄여라"로 오는데 같은 저장소의 다른 게이트는 정반대를
        요구한다 — `doc_check` 는 특정 개수 주장 자리를 **유지**하라 하고,
        `test_13` 은 예약 23 블록을 **바이트 동일**로 얼려 둔다. 실제로 08-07 에
        여유가 **41 바이트**까지 몰려 규칙 개정문 한 줄을 못 쓰는 상태가 됐다.
        → 재는 대상을 바꾼다: 비율이 아니라 **절대 상한**. 우리가 실제로 신경 쓰는 것은
          "차가운 세션이 얼마를 읽고 시작하는가"이지 6월 어느 날과의 비율이 아니다.
        ⚠ 이는 `ef25ad3` 대비 30% → 약 25% 로 **완화**다. 숨기지 않는다 —
          결정 = 사용자(2026-08-07), 색인 = `MASTER_PLAN.md §8`.
        상한을 지탱하는 규약은 `AGENTS.md §5` 의 "핸드오프는 현재 묶음 + 미해결 보류만".
        """
        text = "".join(
            (ROOT / path).read_text() for path in COLD_START_PATHS
        )
        self.assertLessEqual(
            len(text.encode()), COLD_START_BYTE_BUDGET,
            f"cold-start {len(text.encode()):,} bytes > {COLD_START_BYTE_BUDGET:,} — "
            "정본을 줄일 차례다 (총량은 AGENTS.md 증가분까지 포함한다)",
        )

    def test_14b_handoff_alone_fits_its_own_budget(self):
        """🔴 `AGENTS §5` 를 강제하는 자리는 총량이 아니라 **여기**다 (08-11 사용자 결정).

        총량 상한은 세 파일을 재므로 `AGENTS.md` 의 정당한 증가가 핸드오프 여유를
        갉아먹었다 — 서로 무관한 두 관심사가 한 예산을 놓고 다퉜고, 그래서 08-07
        개정 뒤 4일 만에 여유가 30 바이트로 돌아왔다. 압력은 규율이 실제로 걸리는
        파일에만 있어야 의미가 있다: 핸드오프는 **현재 묶음 + 미해결 보류만** 든다.

        ⚠ 이 테스트가 깨지면 상한을 올리기 전에 **옮길 완료 서사가 있는지 먼저 본다.**
        없는데도 반복해 걸리면 그때가 재개방 시점이다(모듈 상단 재개방 조건 ①).
        """
        handoff = (ROOT / "docs/CURRENT_HANDOFF.md").read_text()
        size = len(handoff.encode())
        self.assertLessEqual(
            size, HANDOFF_BYTE_BUDGET,
            f"CURRENT_HANDOFF {size:,} bytes > {HANDOFF_BYTE_BUDGET:,} — "
            "완료된 서사를 정본으로 보내라 (AGENTS.md §5)",
        )

    @unittest.skipUnless(
        shutil.which("node")
        and Path("/usr/share/code/resources/app/extensions/copilot/dist/")
        .joinpath("o200k_base.tiktoken").exists(),
        "local o200k tokenizer is not installed",
    )
    def test_15_local_o200k_raw_tokens_fit_the_absolute_budget(self):
        """토큰도 같은 이유로 절대 상한이다 (test_14 docstring 참조).

        바이트와 토큰을 **둘 다** 재는 이유: 한글은 바이트/토큰 비가 영문과 달라
        한쪽만 재면 다른 쪽이 조용히 넘칠 수 있다. 토큰이 실제 문맥 비용이고,
        바이트는 토크나이저 없이도 도는 값이라 fail-closed 용으로 남긴다.
        """
        text = "".join((ROOT / path).read_text() for path in COLD_START_PATHS)
        result = subprocess.run(
            ["node", "tools/local_token_count.js"], cwd=ROOT, input=text,
            text=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        )
        tokens = int(result.stdout.strip())
        self.assertLessEqual(
            tokens, COLD_START_TOKEN_BUDGET,
            f"cold-start {tokens:,} o200k tokens > {COLD_START_TOKEN_BUDGET:,} — "
            "완료된 서사를 정본으로 보내라 (AGENTS.md §5)",
        )

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
        # The contract is about §7-c-0's own procedure, so the window is that
        # section's body: from its heading to the next `####`, whatever it is.
        # Ending at the literal `#### 7-c-R1.` silently policed every section
        # inserted between the two — §7-c-E (re-arm) tripped the "exactly two
        # pub blocks" count in 08-11 with a command that has nothing to do with
        # the watchdog measurement.
        watchdog = re.split(
            r"\n#### ", text.split("#### 7-c-0.", 1)[1], maxsplit=1
        )[0]
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
        self.assertIn("0.5초를 넘겨 계속 돌면", between)
        self.assertIn("즉시 E-stop", between)
        self.assertIn("2초를 채우지", between)
        self.assertIn("zero Twist 블록은 실행하지 않는다", between)

        r1 = text.split("#### 7-c-R1.", 1)[1].split("#### 7-c-1.", 1)[0]
        r1_blocks = re.findall(r"```bash\n(.*?)```", r1, re.S)
        self.assertTrue(any(block.count("ros2 topic pub") == 2 for block in r1_blocks))
        self.assertNotIn("2초를 채우지", r1)
        self.assertNotIn("zero Twist 블록은 실행하지 않는다", r1)

    def test_28_watchdog_evidence_has_one_physical_authority(self):
        d1 = (ROOT / "docs/D1_FIRST_STEP.md").read_text()
        real = (ROOT / "docs/REAL_ROBOT_VALUES.md").read_text()
        freeze = (ROOT / "docs/FREEZE_MANIFEST.md").read_text()
        self.assertNotIn("R0 의 미결 하나를 잊지 말 것", d1)
        self.assertNotIn("조건부 수용의 **정지 조건은 충족**", real)
        self.assertIn("cmd_vel watchdog 회신 참고값", real)
        self.assertIn("확정·재개방의 유일한 근거", real)
        self.assertIn("R0 실측에서 watchdog 이 확인되지 않으면", freeze)

        paths = watchdog_authority_paths(freeze)
        self.assertEqual(7, len(paths))
        self.assertEqual(7, len(set(paths)))
        documents = watchdog_authority_documents(freeze, ROOT / "docs")
        slot_counts = {
            path: source.count(WATCHDOG_SLOT_START)
            for path, source in documents.items()
            if WATCHDOG_SLOT_START in source
        }
        self.assertEqual({"D1_FIRST_STEP.md": 1, "JETSON_SETUP.md": 1}, slot_counts)
        self.assertTrue(all(
            source.count(WATCHDOG_SLOT_START) == source.count(WATCHDOG_SLOT_END)
            for source in documents.values()
        ))
        for path in slot_counts:
            self.assertTrue(watchdog_record_slot_lines(documents[path]), path)
        self.assertEqual([], watchdog_authority_violations(documents))

    def test_29_watchdog_authority_sweep_rejects_paraphrases_in_all_active_docs(self):
        freeze = (ROOT / "docs/FREEZE_MANIFEST.md").read_text()
        paths = watchdog_authority_paths(freeze)
        documents = watchdog_authority_documents(freeze, ROOT / "docs")
        attacks = (
            "구동부 회신만으로 watchdog 정지 조건은 사실상 충족됐다고 본다.",
            "구동부 3차 회신으로 watchdog 정지 조건은 이미 충족됐다.",
            "R0 watchdog 정지 조건은 구동부 회신으로 통과 처리한다.",
            "구동부 회신으로 watchdog 정지 조건은 충족됐으며 R1은 금지한다.",
            "구동부 회신으로 watchdog 정지 조건은 확정됐고 필요하면 재개방한다.",
            "구동부 회신으로 watchdog 정지 조건을 통과 처리하고 실측을 전제한다.",
        )
        for path in paths:
            for attack in attacks:
                with self.subTest(path=path, attack=attack):
                    injected = {**documents, path: f"{documents[path]}\n\n{attack}\n"}
                    violations = watchdog_authority_violations(injected)
                    self.assertIn((path, attack), violations)

    def test_30_watchdog_authority_sweep_allows_explicitly_downgraded_replies(self):
        allowed = (
            "구동부 회신은 watchdog 판단의 참고값일 뿐 물리 증거가 아니다.",
            "구동부 회신만으로는 watchdog 정지 조건을 통과 처리하면 안 된다.",
            "watchdog 조건부 수용은 구동부 회신으로 확정하지 않고 R0 실측을 전제한다.",
        )
        for statement in allowed:
            with self.subTest(statement=statement):
                self.assertEqual([], watchdog_authority_violations({"fixture.md": statement}))

    def test_31_watchdog_measurements_are_allowed_in_both_document_owned_slots(self):
        freeze = (ROOT / "docs/FREEZE_MANIFEST.md").read_text()
        documents = watchdog_authority_documents(freeze, ROOT / "docs")
        records = (
            "PASS — 60fps 18프레임 = 0.30초, bag d0_watchdog_0803_1420",
            "PASS 0.30초. 구동부 회신 0.010s보다 느리지만 조건 충족",
            "PASS — 정지 조건 충족(0.30초). 구동부 회신값과 자릿수 차이 있음",
            "PASS. 회신 0.010s 대비 0.30초로 느림, 통과 처리",
            "FAIL — 0.8초 활주, E-stop. 구동부 회신과 불일치",
        )

        def replace_slot(source, content):
            before, rest = source.split(WATCHDOG_SLOT_START, 1)
            _old, after = rest.split(WATCHDOG_SLOT_END, 1)
            return (
                f"{before}{WATCHDOG_SLOT_START}\n{content}\n"
                f"{WATCHDOG_SLOT_END}{after}"
            )

        for path in ("D1_FIRST_STEP.md", "JETSON_SETUP.md"):
            for record in records:
                with self.subTest(path=path, record=record):
                    content = (
                        f"| R0 watchdog | 실차 판정 | {record} |"
                        if path == "D1_FIRST_STEP.md"
                        else f"R0 watchdog 실측 기록: {record}"
                    )
                    source = replace_slot(documents[path], content)
                    self.assertEqual([], watchdog_authority_violations({path: source}))

    def test_32_watchdog_record_slot_does_not_exempt_reply_only_or_outside_claims(self):
        outside_records = (
            "PASS 0.30초. 구동부 회신 0.010s보다 느리지만 조건 충족",
            "PASS — 정지 조건 충족(0.30초). 구동부 회신값과 자릿수 차이 있음",
            "PASS. 회신 0.010s 대비 0.30초로 느림, 통과 처리",
        )
        for record in outside_records:
            with self.subTest(outside=record):
                outside = f"R0 watchdog 실측 기록: {record}"
                self.assertTrue(watchdog_authority_violations({"outside.md": outside}))

        reply_only_records = (
            "PASS — 실측 생략, 구동부 회신으로 정지 조건 충족",
            "PASS — 0.010s, 실측 생략, 구동부 회신으로 정지 조건 충족",
        )
        for record in reply_only_records:
            with self.subTest(reply_only=record):
                source = (
                    f"{WATCHDOG_SLOT_START}\nR0 watchdog: {record}\n"
                    f"{WATCHDOG_SLOT_END}\n"
                )
                self.assertTrue(watchdog_authority_violations({"slot.md": source}))

        outside = f"R0 watchdog 실측 기록: {outside_records[0]}"
        marked = f"{WATCHDOG_SLOT_START}\n{outside}\n{WATCHDOG_SLOT_END}\n"
        self.assertEqual([], watchdog_authority_violations({"slot.md": marked}))
        missing_end = marked.replace(WATCHDOG_SLOT_END, "")
        self.assertTrue(watchdog_authority_violations({"slot.md": missing_end}))
        unmarked = marked.replace(WATCHDOG_SLOT_START, "").replace(WATCHDOG_SLOT_END, "")
        self.assertTrue(watchdog_authority_violations({"slot.md": unmarked}))

    def test_33_watchdog_table_cells_and_split_sentences_cannot_hide_claims(self):
        attacks = (
            "| watchdog 회신 | 구동부 회신으로 정지 조건 충족 | 참고: 3차 회신 |",
            "구동부 3차 회신이 도착했다. 이로써 watchdog 정지 조건은 충족됐다.",
        )
        for attack in attacks:
            with self.subTest(attack=attack):
                self.assertTrue(watchdog_authority_violations({"fixture.md": attack}))

    def test_34_missing_watchdog_census_path_names_the_contract_failure(self):
        freeze = (ROOT / "docs/FREEZE_MANIFEST.md").read_text()
        broken = freeze.replace(
            "| `MASTER_PLAN.md` |", "| `MISSING_WATCHDOG.md` |", 1
        )
        with self.assertRaisesRegex(
            AssertionError, "active watchdog document is missing: .*MISSING_WATCHDOG.md"
        ):
            watchdog_authority_documents(broken, ROOT / "docs")


if __name__ == "__main__":
    unittest.main(verbosity=2)
