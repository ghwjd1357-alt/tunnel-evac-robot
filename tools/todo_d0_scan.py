#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Check D+0/D+1 TODO list, count claims, and actionable marker contracts.

The first version checked only the D+0 list count.  Review §34 found the same
class next door: D1_FIRST_STEP claimed ten items but exposed only seven
actionable ``TODO(D+1): 확인`` markers.  One phase-specific implementation would
repeat that drift, so this scanner uses one algorithm with explicit contracts.

For each phase it independently locks three boundaries:
  1. numbered list rows are contiguous and equal the registered item count;
  2. every current-document ``N건`` claim near that phase agrees with the list,
     and the number of claim sites is exact (increase and decrease both fail);
  3. actionable markers between the runbook start and its list heading have an
     exact count.  D+1 deliberately requires one marker per listed item.  D+0
     keeps its existing grouped-runbook structure, so its marker count is a
     separate explicit contract rather than pretending it maps 1:1.

History policy is shared with gate_baseline_scan.py: legacy directories,
FREEZE_MANIFEST, and ``직전 완료`` bullet blocks are historical evidence and do
not become current count claims.

Usage:
  python3 tools/todo_d0_scan.py [--phase D+0|D+1] [--list]
  python3 tools/todo_d0_scan.py --root DIR --phase D+1
  python3 tools/todo_d0_scan.py --want-items N --want-sites N --want-markers N

Exit: 0 consistent / 1 contract failure / 2 usage or missing-input failure.
"""

import os
import re
import sys


PHASES = {
    "D+0": {
        "list_file": "JETSON_SETUP.md",
        "list_heading": re.compile(r"^#{2,3} .*TODO\(D\+0\).*전량 목록"),
        "marker_start": re.compile(r"^## 1\. "),
        "anchor": "TODO(D+0)",
        "marker": "TODO(D+0): 확인",
        "want_items": 11,
        "want_sites": 5,
        "want_markers": 7,
    },
    "D+1": {
        "list_file": "D1_FIRST_STEP.md",
        "list_heading": re.compile(r"^#{2,3} .*TODO\(D\+1\).*전량 목록"),
        "marker_start": re.compile(r"^## 0\. "),
        "anchor": "TODO(D+1)",
        "marker": "TODO(D+1): 확인",
        "want_items": 10,
        "want_sites": 1,
        "want_markers": 10,
    },
}

ROW_RE = re.compile(r"^\| *(\d+) *\|")
NEXT_HEADING = re.compile(r"^#{1,3} ")
CLAIM_RE = re.compile(r"(\d+) *건")
WINDOW = 100
EXCLUDE_FILES = {"FREEZE_MANIFEST.md"}
EXCLUDE_DIRS = {"legacy"}
HIST_BULLET = re.compile(r"^- \*\*직전 완료")
BLOCK_END = re.compile(r"^(?:- \*\*|#{1,6} )")


def count_list(path, contract):
    """Return contiguous numbered-row count, or ``(None, reason)``."""
    with open(path, encoding="utf-8") as fh:
        lines = fh.read().splitlines()
    start = None
    for index, line in enumerate(lines):
        if contract["list_heading"].match(line):
            start = index + 1
            break
    if start is None:
        return None, "%s 에서 전량 목록 제목을 못 찾았다" % contract["list_file"]

    numbers = []
    for line in lines[start:]:
        if NEXT_HEADING.match(line):
            break
        match = ROW_RE.match(line)
        if match:
            numbers.append(int(match.group(1)))
    if not numbers:
        return None, "%s 의 목록 표에 행이 하나도 없다" % contract["list_file"]
    expected = list(range(1, len(numbers) + 1))
    if numbers != expected:
        return None, ("%s 목록 번호가 1..%d 연속이 아니다: %s"
                      % (contract["list_file"], len(numbers), numbers))
    return len(numbers), None


def count_action_markers(path, contract):
    """Count actionable markers only in the runbook body, excluding preamble/list."""
    with open(path, encoding="utf-8") as fh:
        lines = fh.read().splitlines()
    started = False
    count = 0
    for line in lines:
        if not started:
            if contract["marker_start"].match(line):
                started = True
            else:
                continue
        if contract["list_heading"].match(line):
            break
        count += line.count(contract["marker"])
    if not started:
        return None, "%s 에서 실행 본문 시작 제목을 못 찾았다" % contract["list_file"]
    return count, None


def scan_claims(root, contract):
    """Return current Markdown ``(file, line, N)`` claims for one phase."""
    claims = []
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [name for name in dirnames if name not in EXCLUDE_DIRS]
        for name in sorted(filenames):
            if not name.endswith(".md") or name in EXCLUDE_FILES:
                continue
            full = os.path.join(dirpath, name)
            rel = os.path.relpath(full, os.path.dirname(root) or ".")
            claims.extend(scan_file(full, rel, contract))
    claims.sort()
    return claims


def scan_file(path, rel, contract):
    """Whitespace-normalize a document while retaining source line numbers."""
    hits = []
    buffer, line_map = [], []
    in_history = False
    with open(path, encoding="utf-8") as fh:
        for lineno, line in enumerate(fh, 1):
            if HIST_BULLET.match(line):
                in_history = True
                continue
            if in_history and BLOCK_END.match(line):
                in_history = False
            if in_history:
                continue
            text = re.sub(r"\s+", " ", line.strip()) + " "
            buffer.append(text)
            line_map.extend([lineno] * len(text))
    flat = "".join(buffer)
    for match in CLAIM_RE.finditer(flat):
        lo = max(0, match.start() - WINDOW)
        hi = min(len(flat), match.end() + WINDOW)
        if contract["anchor"] in flat[lo:hi]:
            hits.append((rel, line_map[match.start()], int(match.group(1))))
    return hits


def _number(value, option):
    if not value.isdigit():
        raise ValueError("%s 는 숫자여야 한다: %r" % (option, value))
    return int(value)


def parse_args(argv):
    phase = "D+0"
    root = None
    show_list = False
    overrides = {}
    iterator = iter(argv)
    for arg in iterator:
        if arg == "--list":
            show_list = True
        elif arg == "--phase":
            phase = next(iterator, "")
        elif arg == "--root":
            root = next(iterator, "")
        elif arg in ("--want-items", "--want-sites", "--want-markers"):
            key = arg[2:].replace("-", "_")
            overrides[key] = _number(next(iterator, ""), arg)
        else:
            raise ValueError("알 수 없는 인자: %r" % arg)
    if phase not in PHASES:
        raise ValueError("--phase 는 D+0 또는 D+1 이어야 한다: %r" % phase)
    if root == "":
        raise ValueError("--root 에 디렉터리가 필요하다")
    return phase, root, show_list, overrides


def main(argv):
    try:
        phase, root, show_list, overrides = parse_args(argv)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    contract = dict(PHASES[phase])
    contract.update(overrides)
    if root is None:
        root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs")
    root = os.path.normpath(root)
    if not os.path.isdir(root):
        print("디렉터리가 없다: %s" % root, file=sys.stderr)
        return 2

    list_path = os.path.join(root, contract["list_file"])
    if not os.path.isfile(list_path):
        print("FAIL %s 이 없다 — 대조할 정본 목록이 없다" % list_path)
        return 1
    actual, error = count_list(list_path, contract)
    if actual is None:
        print("FAIL %s" % error)
        return 1
    markers, error = count_action_markers(list_path, contract)
    if markers is None:
        print("FAIL %s" % error)
        return 1
    claims = scan_claims(root, contract)
    if show_list:
        for rel, lineno, value in claims:
            print("%s:%d:%d건" % (rel, lineno, value))

    failures = []
    if actual != contract["want_items"]:
        failures.append("목록 %d건 — 등록 계약은 정확히 %d건" %
                        (actual, contract["want_items"]))
    for rel, lineno, value in claims:
        if value != actual:
            failures.append("개수 불일치: %s:%d 이 %d건 — 실제 목록은 %d건" %
                            (rel, lineno, value, actual))
    if len(claims) != contract["want_sites"]:
        direction = "자리 증식" if len(claims) > contract["want_sites"] else "자리 소실"
        failures.append("개수 주장 %d자리 — 계약은 정확히 %d (%s)" %
                        (len(claims), contract["want_sites"], direction))
    if markers != contract["want_markers"]:
        direction = "표식 증식" if markers > contract["want_markers"] else "표식 소실"
        failures.append("실행 표식 %d자리 — 계약은 정확히 %d (%s)" %
                        (markers, contract["want_markers"], direction))

    if failures:
        for failure in failures:
            print("FAIL TODO(%s) %s" % (phase, failure))
        return 1
    print("OK   TODO(%s) 목록 %d건 = 문서 %d자리 · 실행 표식 %d자리" %
          (phase, actual, len(claims), markers))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
