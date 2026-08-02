#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""todo_d0_scan.py — `TODO(D+0)` **목록의 실제 개수**와 문서가 말하는 개수를 대조한다.

왜 있나 (08-03 · 검토 §30.4):
  `JETSON_SETUP.md §9` 의 정본 목록은 **10건**인데 `CURRENT_HANDOFF.md` 의 완료조건은
  네 자리에서 **8건**이라고 적었다. 바로 앞 커밋(`0a70218`)이 8→10 드리프트를 고친
  **다음 커밋**에서 다시 갈라졌고, `doc_check --strict` 는 그 상태에서도 PASS 했다.
  현장 작업자가 완료조건의 8건만 채우면 NTP 단조성·E-stop 배선 확인이 통째로 빠진다.
  → 개수는 사람이 지키는 약속이 아니라 **기계가 세는 사실**이어야 한다.

무엇을 하나 (두 질문은 다르다 — 둘 다 묻는다)
  ① **목록이 몇 건인가** — `§9` 표의 행을 직접 센다(번호가 1..N 연속인지도 본다).
  ② **문서가 몇 건이라고 말하는가** — 모든 문서에서 `TODO(D+0)` 근처의 `N건` 을 훑는다.
  둘이 다르면 FAIL. 목록이 늘거나(10→11) 줄면(10→9) ①이 바뀌므로 ②의 모든 자리가 깨진다.

★ 왜 '자리 수'까지 정확한 계약인가 (`AGENTS.md §3-10 ⑤` · 검토 §19 P2-① 의 교훈)
  값만 보면 **표기가 사라지는 쪽**이 안 잡힌다 — 완료조건에서 그 문장을 지워 버리면
  대조할 것이 없어져 조용히 통과한다. 그래서 자리 수를 `WANT_SITES` 에 등록하고
  증감을 **양쪽 다** 잡는다. 자리를 늘리거나 줄이려면 여기 숫자를 같이 고쳐야 한다.

★ 줄바꿈에 속지 않는다: 실제 드리프트 자리 하나(`CURRENT_HANDOFF.md:101-102`)는
  `TODO(D+0)` 와 `8건` 이 **다른 줄**에 있었다. 그래서 줄 단위가 아니라 파일 전체를
  공백 정규화한 뒤 근접 창으로 본다 (`doc_check.sh §9` 검사가 같은 이유로 쓰는 방식).

⚠ 검증 상한 (숨기지 않는다)
  · 대상은 `.md` 뿐이다. 셸·파이썬 주석 안의 개수 주장은 안 본다.
  · `FREEZE_MANIFEST.md` 와 `legacy/` 는 **그 시점 기록**이라 제외한다
    (`gate_baseline_scan.py` 와 같은 규약). 거기 적힌 옛 개수는 정상이다.
  · `- **직전 완료 …**` 불릿 블록도 그 회차의 기록이라 제외한다(같은 규약).
  · 근접 창(±%d자)보다 멀리 떨어뜨려 쓰면 못 본다. 표현을 바꿔 우회하는 것까지는 못 막는다.

사용:
  python3 tools/todo_d0_scan.py                 # 저장소 docs/ 대조
  python3 tools/todo_d0_scan.py --list          # 훑은 자리 전량 출력
  python3 tools/todo_d0_scan.py --root <디렉터리>  # 회귀 픽스처용
  python3 tools/todo_d0_scan.py --want-sites N  # 픽스처에서 자리 수 계약을 바꿔 볼 때

종료코드: 0 = 이상 없음 / 1 = 불일치·자리 수 위반·목록 판독 실패 / 2 = 사용법 오류
"""

import os
import re
import sys

# 정본 목록이 있는 파일과 절 (이 두 개가 이 검사의 기준점이다)
LIST_FILE = "JETSON_SETUP.md"
LIST_HEADING = re.compile(r"^#{2,3} .*TODO\(D\+0\).*전량 목록")
ROW_RE = re.compile(r"^\| *(\d+) *\|")
NEXT_HEADING = re.compile(r"^#{1,3} ")

# 개수 주장 = `TODO(D+0)` 로부터 WINDOW 자 안에 있는 `N건`
CLAIM_RE = re.compile(r"(\d+) *건")
ANCHOR = "TODO(D+0)"
WINDOW = 100

# ★ 자리 수 계약. 늘리거나 줄이려면 이 숫자를 **같이** 고친다 (자동 편입은 없다).
#   08-03 실측 자리 5 = CURRENT_HANDOFF 4 (함정 3 · 할 일 표 8행 · 완료조건 2 · 완료 판정)
#                      + JETSON_SETUP §9 제목 1
WANT_SITES = 5

EXCLUDE_FILES = {"FREEZE_MANIFEST.md"}
EXCLUDE_DIRS = {"legacy"}
HIST_BULLET = re.compile(r"^- \*\*직전 완료")
BLOCK_END = re.compile(r"^(?:- \*\*|#{1,6} )")

__doc__ = __doc__ % WINDOW


def count_list(path):
    """§9 표의 행을 직접 센다 — (개수, 오류문자열). 못 읽으면 (None, 사유)."""
    with open(path, encoding="utf-8") as fh:
        lines = fh.read().splitlines()
    start = None
    for i, line in enumerate(lines):
        if LIST_HEADING.match(line):
            start = i + 1
            break
    if start is None:
        return None, "%s 에서 'TODO(D+0) … 전량 목록' 제목을 못 찾았다" % LIST_FILE

    nums = []
    for line in lines[start:]:
        if NEXT_HEADING.match(line):
            break
        m = ROW_RE.match(line)
        if m:
            nums.append(int(m.group(1)))
    if not nums:
        return None, "%s 의 목록 표에 행이 하나도 없다" % LIST_FILE
    if nums != list(range(1, len(nums) + 1)):
        return None, ("%s 목록 번호가 1..%d 연속이 아니다: %s "
                      "(중복·건너뜀은 '몇 건인지'를 못 세게 만든다)"
                      % (LIST_FILE, len(nums), nums))
    return len(nums), None


def scan_claims(root):
    """docs 전체에서 'TODO(D+0) 근처의 N건' 주장을 훑는다 → [(rel, line, value)]."""
    claims = []
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in EXCLUDE_DIRS]
        for fn in sorted(filenames):
            if not fn.endswith(".md") or fn in EXCLUDE_FILES:
                continue
            full = os.path.join(dirpath, fn)
            rel = os.path.relpath(full, os.path.dirname(root) or ".")
            claims.extend(scan_file(full, rel))
    claims.sort()
    return claims


def scan_file(path, rel):
    """한 파일을 **공백 정규화한 한 줄**로 만들어 훑는다(줄바꿈으로 갈라진 주장을 잡으려고).

    정규화하면서 각 문자가 원래 몇 번째 줄이었는지 map 을 같이 만든다 — 사람이 고칠 수
    있어야 검사가 쓸모 있다. 줄 번호 없이 "어딘가 틀렸다"고만 하면 못 고친다.
    """
    hits = []
    buf, lineno_of = [], []
    in_hist = False
    with open(path, encoding="utf-8") as fh:
        for lineno, line in enumerate(fh, 1):
            if HIST_BULLET.match(line):
                in_hist = True
                continue
            if in_hist and BLOCK_END.match(line):
                in_hist = False
            if in_hist:
                continue
            text = re.sub(r"\s+", " ", line.strip()) + " "
            buf.append(text)
            lineno_of.extend([lineno] * len(text))
    flat = "".join(buf)

    for m in CLAIM_RE.finditer(flat):
        lo = max(0, m.start() - WINDOW)
        hi = min(len(flat), m.end() + WINDOW)
        if ANCHOR in flat[lo:hi]:
            hits.append((rel, lineno_of[m.start()], int(m.group(1))))
    return hits


def main(argv):
    root = None
    do_list = False
    want_sites = WANT_SITES
    it = iter(argv)
    for arg in it:
        if arg == "--list":
            do_list = True
        elif arg == "--root":
            root = next(it, None)
            if root is None:
                print("--root 에 디렉터리가 필요하다", file=sys.stderr)
                return 2
        elif arg == "--want-sites":
            val = next(it, "")
            if not val.isdigit():
                print("--want-sites 는 숫자여야 한다: %r" % val, file=sys.stderr)
                return 2
            want_sites = int(val)
        else:
            print("알 수 없는 인자: %r" % arg, file=sys.stderr)
            return 2

    if root is None:
        root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs")
    root = os.path.normpath(root)
    if not os.path.isdir(root):
        print("디렉터리가 없다: %s" % root, file=sys.stderr)
        return 2

    list_path = os.path.join(root, LIST_FILE)
    if not os.path.isfile(list_path):
        # fail-closed: 정본 목록을 못 찾으면 '이상 없음'이라고 말하지 않는다.
        print("FAIL %s 이 없다 — 대조할 정본 목록이 없다" % list_path)
        return 1
    actual, err = count_list(list_path)
    if actual is None:
        print("FAIL %s" % err)
        return 1

    claims = scan_claims(root)
    if do_list:
        for rel, lineno, val in claims:
            print("%s:%d:%d건" % (rel, lineno, val))

    rc = 0
    bad = [c for c in claims if c[2] != actual]
    for rel, lineno, val in bad:
        rc = 1
        print("FAIL TODO(D+0) 개수 불일치: %s:%d 이 %d건 — 실제 목록은 %d건"
              % (rel, lineno, val, actual))
    if len(claims) != want_sites:
        rc = 1
        kind = ("자리 증식 — 새 자리는 todo_d0_scan.py 의 WANT_SITES 에 같이 올려야 한다"
                if len(claims) > want_sites
                else "자리 소실 — 개수를 말하는 문장이 사라졌다(검사도 같이 증발한다)")
        where = " · ".join("%s:%d" % (c[0], c[1]) for c in claims) or "(한 자리도 없음)"
        print("FAIL TODO(D+0) 개수 주장 %d자리 — 계약은 정확히 %d · %s ↳ %s"
              % (len(claims), want_sites, kind, where))
    if rc == 0:
        print("OK   TODO(D+0) 목록 %d건 = 문서 %d자리 전부" % (actual, len(claims)))
    return rc


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
