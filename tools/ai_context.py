#!/usr/bin/env python3
"""Build one lossless, change-surface-aware context packet for an AI turn.

The packet reduces repeated model reads; it never replaces the source documents.
Unknown paths fail closed by including every safety document.  Review packets also
carry the complete diff (no line or byte truncation), so saving context cannot
silently narrow the review surface.
"""

from __future__ import annotations

import argparse
import fnmatch
import hashlib
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class Ref:
    path: str
    section: str | None = None


COMMON = (Ref("docs/CURRENT_HANDOFF.md"),)
REVIEW_GATE = Ref("docs/TEST_GATES.md", "7")
IMPLEMENT_GATE = Ref("docs/TEST_GATES.md", "1")

PROFILE_REFS: dict[str, tuple[Ref, ...]] = {
    "process": (
        Ref("docs/AI_CONTEXT.md"),
        Ref("docs/TEST_GATES.md", "7"),
    ),
    "docs": (
        Ref("docs/PROJECT_CONTEXT.md", "8"),
        Ref("docs/TEST_GATES.md", "7"),
    ),
    "firmware": (
        Ref("docs/FIRMWARE_REBUILD.md"),
        Ref("docs/REAL_ROBOT_VALUES.md", "1"),
        Ref("docs/ELECTRICAL_BASELINE.md", "4"),
        # 검토 §43: E-stop 물리 권위(릴레이 DC 차단·퓨즈 협조)의 전제조건·재개방 조건이
        # §7 에, 그 판정을 되돌린 사유가 §13 에 있다. 펌웨어 상수 MAX_LINEAR_CMD 가
        # §43.2 조건부 수용의 전제라, 펌웨어를 만지는 사람이 이 둘을 못 보면 안 된다.
        Ref("docs/ELECTRICAL_BASELINE.md", "7"),
        Ref("docs/ELECTRICAL_BASELINE.md", "13"),
        Ref("docs/TEST_GATES.md", "7"),
    ),
    "mission": (
        Ref("docs/PROJECT_CONTEXT.md", "4"),
        Ref("docs/PROJECT_CONTEXT.md", "6"),
        Ref("docs/MASTER_PLAN.md", "8"),
        Ref("docs/PITFALLS.md", "8"),
        # 08-07: 통째 로딩을 절 단위로 좁혔다. 앵커 실측(§6·§8·§10)이 판정 근거이며
        # test_02 의 profile-only 회수 100% 가 이 목록의 전수성을 강제한다.
        Ref("docs/FREEZE_MANIFEST.md", "6"),
        Ref("docs/FREEZE_MANIFEST.md", "8"),
        Ref("docs/FREEZE_MANIFEST.md", "10"),
    ),
    "bringup": (
        Ref("docs/PROJECT_CONTEXT.md", "3"),
        Ref("docs/PROJECT_CONTEXT.md", "4"),
        Ref("docs/PROJECT_CONTEXT.md", "5"),
        Ref("docs/PROJECT_CONTEXT.md", "6"),
        Ref("docs/MASTER_PLAN.md", "3"),
        Ref("docs/MASTER_PLAN.md", "8"),
        Ref("docs/PITFALLS.md", "1"),
        Ref("docs/PITFALLS.md", "2"),
        Ref("docs/PITFALLS.md", "3"),
        Ref("docs/PITFALLS.md", "5"),
        Ref("docs/PITFALLS.md", "6"),
        Ref("docs/PITFALLS.md", "7"),
        Ref("docs/REAL_ROBOT_VALUES.md", "1"),
        Ref("docs/REAL_ROBOT_VALUES.md", "2"),
        Ref("docs/REAL_ROBOT_VALUES.md", "4"),
        # 08-07: 통째 로딩을 절 단위로 좁혔다. 앵커 실측(§6·§8·§10)이 판정 근거이며
        # test_02 의 profile-only 회수 100% 가 이 목록의 전수성을 강제한다.
        Ref("docs/FREEZE_MANIFEST.md", "6"),
        Ref("docs/FREEZE_MANIFEST.md", "8"),
        Ref("docs/FREEZE_MANIFEST.md", "10"),
    ),
    "nav2": (
        Ref("docs/PROJECT_CONTEXT.md", "5"),
        Ref("docs/PROJECT_CONTEXT.md", "6"),
        Ref("docs/MASTER_PLAN.md", "3"),
        Ref("docs/MASTER_PLAN.md", "8"),
        Ref("docs/PITFALLS.md", "3"),
        Ref("docs/PITFALLS.md", "5"),
        Ref("docs/PITFALLS.md", "6"),
        Ref("docs/PITFALLS.md", "7"),
        Ref("docs/REAL_ROBOT_VALUES.md", "1"),
        Ref("docs/REAL_ROBOT_VALUES.md", "2"),
        Ref("docs/REAL_ROBOT_VALUES.md", "4"),
        # 08-07: 통째 로딩을 절 단위로 좁혔다. 앵커 실측(§6·§8·§10)이 판정 근거이며
        # test_02 의 profile-only 회수 100% 가 이 목록의 전수성을 강제한다.
        Ref("docs/FREEZE_MANIFEST.md", "6"),
        Ref("docs/FREEZE_MANIFEST.md", "8"),
        Ref("docs/FREEZE_MANIFEST.md", "10"),
    ),
    "e2e": (
        Ref("docs/MASTER_PLAN.md", "8"),
        Ref("docs/TEST_GATES.md", "2"),
        Ref("docs/TEST_GATES.md", "5"),
        Ref("docs/PITFALLS.md", "1"),
        Ref("docs/PITFALLS.md", "2"),
        Ref("docs/PITFALLS.md", "4"),
        Ref("docs/FREEZE_MANIFEST.md", "6"),
        Ref("docs/FREEZE_MANIFEST.md", "7"),
        Ref("docs/FREEZE_MANIFEST.md", "8"),
    ),
    "judgment": (
        Ref("docs/MASTER_PLAN.md", "8"),
        Ref("docs/TEST_GATES.md", "5"),
        Ref("docs/PITFALLS.md", "1"),
        Ref("docs/PITFALLS.md", "2"),
        Ref("docs/FREEZE_MANIFEST.md", "10"),
    ),
    "map": (
        Ref("docs/PROJECT_CONTEXT.md", "6"),
        Ref("docs/MASTER_PLAN.md", "8"),
        Ref("docs/TEST_GATES.md", "2"),
        Ref("docs/TEST_GATES.md", "4"),
        Ref("docs/PITFALLS.md", "4"),
        Ref("docs/PITFALLS.md", "7"),
        # 08-07: 통째 로딩을 절 단위로 좁혔다. 앵커 실측(§6·§8·§10)이 판정 근거이며
        # test_02 의 profile-only 회수 100% 가 이 목록의 전수성을 강제한다.
        Ref("docs/FREEZE_MANIFEST.md", "6"),
        Ref("docs/FREEZE_MANIFEST.md", "8"),
        Ref("docs/FREEZE_MANIFEST.md", "10"),
    ),
    "perception": (
        Ref("docs/PROJECT_CONTEXT.md", "2"),
        Ref("docs/PROJECT_CONTEXT.md", "4"),
        Ref("docs/MASTER_PLAN.md", "1"),
        Ref("docs/MASTER_PLAN.md", "4"),
        Ref("docs/MASTER_PLAN.md", "8"),
    ),
    "accuracy": (
        Ref("docs/PROJECT_CONTEXT.md", "7"),
        Ref("docs/MASTER_PLAN.md", "8"),
        Ref("docs/TEST_GATES.md", "6"),
        Ref("docs/PITFALLS.md", "4"),
        Ref("docs/PITFALLS.md", "6"),
        Ref("docs/PITFALLS.md", "7"),
        # 08-07: 통째 로딩을 절 단위로 좁혔다. 앵커 실측(§6·§8·§10)이 판정 근거이며
        # test_02 의 profile-only 회수 100% 가 이 목록의 전수성을 강제한다.
        Ref("docs/FREEZE_MANIFEST.md", "6"),
        Ref("docs/FREEZE_MANIFEST.md", "8"),
        Ref("docs/FREEZE_MANIFEST.md", "10"),
    ),
}

FULL_SAFETY_DOCS = tuple(
    Ref(path)
    for path in (
        "docs/PROJECT_CONTEXT.md",
        "docs/MASTER_PLAN.md",
        "docs/TEST_GATES.md",
        "docs/PITFALLS.md",
        "docs/FREEZE_MANIFEST.md",
        "docs/REAL_ROBOT_VALUES.md",
    )
)

PATH_PROFILES: tuple[tuple[str, tuple[str, ...]], ...] = (
    ("process", (
        "AGENTS.md", "CLAUDE.md", "docs/CURRENT_HANDOFF.md",
        "docs/AI_CONTEXT.md", "tools/ai_context.py", "tools/test_ai_context.py",
        "tools/ai_known_p0_p1.json", "tools/local_token_count.js",
    )),
    ("mission", ("src/mission_manager/**",)),
    ("bringup", (
        "src/tunnel_bringup/**", "docs/JETSON_SETUP.md", "docs/D1_FIRST_STEP.md",
        "tools/d0_check.sh", "tools/bag_gap_report.py", "tools/todo_d0_scan.py",
        # 검토 §52: R0 watchdog 판정기. 판정 기준은 `JETSON_SETUP §7-c-0` 이 소유하고
        # 이 도구는 그 기준에 넣을 수치를 만든다 — 둘을 같은 프로필에 둔다.
        # 회귀도 같은 자리다 — 08-10 에 판정기만 넣고 그 테스트를 빠뜨렸다.
        "tools/watchdog_report.py", "tools/test_watchdog_report.py",
    )),
    ("nav2", (
        "**/*nav2*.yaml", "**/*.urdf", "**/*.xacro",
        "src/tunnel_sim/launch/nav2.launch.py",
    )),
    ("map", (
        "maps/**", "tools/map_promote.sh", "tools/make_map.sh",
        "tools/test_map_promote.sh",
    )),
    ("accuracy", ("tools/accuracy_*",)),
    ("e2e", (
        "tools/lib_e2e.sh", "tools/*e2e.sh", "tools/regression_*.sh",
        "tools/test_harness_guards.sh", "tools/scan_unbounded_cli.py",
        "src/tunnel_bringup/tunnel_bringup/readiness_gate.py",
        "src/tunnel_bringup/test/test_readiness_gate*.py",
    )),
    ("judgment", (
        "tools/gate_baseline_scan.py", "tools/handoff_single_check.sh",
        "tools/doc_check.sh",
    )),
    ("perception", (
        "src/tunnel_interfaces/**", "**/*detection*", "**/*perception*",
        "console/**",
    )),
    # 검토 §47.1: 굽기 전 오염 판정기는 `firmware/` 밖(`tools/`)에 있지만 계약은
    # `FIRMWARE_REBUILD §4` 가 소유한다 — 기대 증감을 이 둘이 따로 들면 곧 갈라진다.
    # 검토 §54.7: re-arm 상태 전이 harness 도 같은 모양이다 — 파일은 `tools/` 에 있지만
    # 계약은 `REAL_ROBOT_VALUES §1-f` 가 소유하고 대상은 스케치 헤더다.
    ("firmware", (
        "firmware/**", "firmware/*",
        "tools/firmware_precheck.sh", "tools/test_firmware_precheck.sh",
        "tools/rearm_gate_host_test.cpp", "tools/rearm_gate_host_test.sh",
    )),
    ("docs", ("docs/*.md",)),
)


def _git(*args: str, check: bool = True) -> str:
    result = subprocess.run(
        ["git", *args], cwd=ROOT, text=True, stdout=subprocess.PIPE,
        stderr=subprocess.PIPE, check=False,
    )
    if check and result.returncode:
        raise RuntimeError(result.stderr.strip() or f"git {' '.join(args)} failed")
    return result.stdout


def resolve_commit(revision: str) -> str:
    commit = _git(
        "rev-parse", "--verify", "--end-of-options", f"{revision}^{{commit}}"
    ).strip()
    if not re.fullmatch(r"[0-9a-f]{40}", commit):
        raise ValueError(f"not a commit: {revision}")
    return commit


def classify_paths(paths: list[str]) -> tuple[set[str], list[str]]:
    profiles: set[str] = set()
    unknown: list[str] = []
    for path in paths:
        matched = False
        for profile, patterns in PATH_PROFILES:
            if any(fnmatch.fnmatch(path, pattern) for pattern in patterns):
                profiles.add(profile)
                matched = True
        if not matched:
            unknown.append(path)
    return profiles, unknown


def _heading_number(line: str) -> tuple[int, str] | None:
    if not line.startswith("#"):
        return None
    marks = len(line) - len(line.lstrip("#"))
    if marks == 0 or len(line) <= marks or line[marks] != " ":
        return None
    title = line[marks + 1:].strip()
    if not title:
        return None
    token = title.split(maxsplit=1)[0].rstrip(".")
    return marks, token


def _fence_after(line: str, fence: tuple[str, int] | None) -> tuple[str, int] | None:
    """Return Markdown fence state after *line* (backticks and tildes)."""
    stripped = line.lstrip()
    match = re.match(r"^(`{3,}|~{3,})(.*)$", stripped.rstrip("\r\n"))
    if not match:
        return fence
    marks, rest = match.groups()
    if fence is None:
        return marks[0], len(marks)
    char, width = fence
    if marks[0] == char and len(marks) >= width and not rest.strip():
        return None
    return fence


def _headings_outside_fences(lines: list[str]):
    """Yield ``(index, heading)`` only for headings outside fenced code."""
    fence = None
    for index, line in enumerate(lines):
        before = fence
        fence = _fence_after(line, fence)
        if before is None and fence is None:
            heading = _heading_number(line)
            if heading:
                yield index, heading


def read_ref(ref: Ref, *, source_ref: str | None = None) -> tuple[str, int, int, str]:
    if source_ref:
        raw = _git("show", f"{source_ref}:{ref.path}")
    else:
        raw = (ROOT / ref.path).read_text(encoding="utf-8")
    lines = raw.splitlines(keepends=True)
    digest = hashlib.sha256(raw.encode()).hexdigest()[:12]
    if ref.section is None:
        return raw, 1, len(lines), digest

    start = None
    level = None
    headings = list(_headings_outside_fences(lines))
    for index, heading in headings:
        if heading and heading[1] == ref.section:
            start = index
            level = heading[0]
            break
    if start is None or level is None:
        raise ValueError(f"{ref.path}: §{ref.section} heading not found")
    end = len(lines)
    for index, heading in headings:
        if index <= start:
            continue
        if heading and heading[0] <= level:
            end = index
            break
    return "".join(lines[start:end]), start + 1, end, digest


def refs_for(profiles: set[str], unknown: list[str], role: str) -> list[Ref]:
    refs = list(COMMON)
    refs.append(REVIEW_GATE if role == "review" else IMPLEMENT_GATE)
    if unknown:
        refs.extend(FULL_SAFETY_DOCS)
    else:
        for profile in sorted(profiles):
            refs.extend(PROFILE_REFS[profile])
    full_paths = {ref.path for ref in refs if ref.section is None}
    seen: set[Ref] = set()
    return [
        ref for ref in refs
        if not (ref.section is not None and ref.path in full_paths)
        and not (ref in seen or seen.add(ref))
    ]


def safe_worktree_path(path: str) -> Path:
    rel = Path(path)
    if rel.is_absolute() or ".." in rel.parts:
        raise ValueError(f"path must stay inside the repository: {path}")
    resolved = (ROOT / rel).resolve()
    if resolved != ROOT and ROOT not in resolved.parents:
        raise ValueError(f"path escapes the repository: {path}")
    return resolved


def build_packet(paths: list[str], role: str, *, base: str | None = None,
                 target: str = "HEAD", source_ref: str | None = None) -> str:
    profiles, unknown = classify_paths(paths)
    refs = refs_for(profiles, unknown, role)
    out = [
        "# AI CONTEXT PACKET v1\n",
        "> Generated, read-only view. Source documents remain authoritative.\n",
        f"> role={role} profiles={','.join(sorted(profiles)) or 'none'} "
        f"unknown={','.join(unknown) or 'none'}\n",
        "> If unknown is not 'none', full safety docs are intentionally included "
        "(fail-closed).\n\n",
        "## Changed / intended paths\n\n",
        *(f"- `{path}`\n" for path in paths),
    ]
    for ref in refs:
        text, first, last, digest = read_ref(ref, source_ref=source_ref)
        suffix = f" §{ref.section}" if ref.section else ""
        out.extend((
            f"\n## SOURCE `{ref.path}{suffix}` (lines {first}-{last}, sha256={digest})\n\n",
            text,
            "\n" if not text.endswith("\n") else "",
        ))
    if role == "implement":
        routed_paths = {ref.path for ref in refs}
        for path in paths:
            target_path = safe_worktree_path(path)
            if path in routed_paths or not target_path.is_file():
                continue
            try:
                text = target_path.read_text(encoding="utf-8")
            except UnicodeDecodeError:
                out.append(f"\n## WORKING FILE `{path}` — binary; text packet omitted\n")
                continue
            digest = hashlib.sha256(text.encode()).hexdigest()[:12]
            line_count = len(text.splitlines())
            out.extend((
                f"\n## WORKING FILE `{path}` (lines 1-{line_count}, "
                f"sha256={digest})\n\n",
                text,
                "\n" if not text.endswith("\n") else "",
            ))
    if role == "review":
        if not base:
            raise ValueError("review packet requires --base")
        base_commit = resolve_commit(base)
        target_commit = resolve_commit(target)
        diff = _git(
            "diff", "--find-renames", "--find-copies", "--no-ext-diff",
            base_commit, target_commit, "--",
        )
        out.extend((
            "\n## COMPLETE DIFF — NOT TRUNCATED\n\n",
            "```diff\n", diff, "\n" if not diff.endswith("\n") else "", "```\n",
        ))
    return "".join(out)


def changed_paths(base: str, target: str) -> list[str]:
    base_commit = resolve_commit(base)
    target_commit = resolve_commit(target)
    result = subprocess.run(
        ["git", "diff", "--name-only", "-z", "--find-renames", "--find-copies",
         base_commit, target_commit, "--"],
        cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False,
    )
    if result.returncode:
        raise RuntimeError(result.stderr.decode(errors="replace").strip())
    return [item.decode(errors="surrogateescape")
            for item in result.stdout.split(b"\0") if item]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    review = sub.add_parser("review", help="packet for an independent commit review")
    review.add_argument("--base", required=True, help="base commit (exclusive)")
    review.add_argument("--target", default="HEAD", help="target commit (default: HEAD)")
    implement = sub.add_parser("implement", help="packet for an implementation surface")
    implement.add_argument("paths", nargs="+", help="intended files or representative paths")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        if args.command == "review":
            paths = changed_paths(args.base, args.target)
            if not paths:
                raise ValueError("review range has no changed paths")
            packet = build_packet(paths, "review", base=args.base, target=args.target)
        else:
            packet = build_packet(args.paths, "implement")
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"FAIL ai_context: {exc}", file=sys.stderr)
        return 2
    sys.stdout.write(packet)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
