#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gate_baseline_scan.py — 문서에 적힌 게이트 기준선 수치를 **전수로 훑는다**.

왜 있나 (08-01 · 검토 §18 P2-①):
  이전 구현은 `doc_check.sh` 안에 `grep … | head -1` 캡처 7개를 **손으로 나열**했다.
  나열은 두 가지 방식으로 샌다.
    ① 표기가 조금만 달라도 못 본다 — `22 검사` 는 잡고 `22검사`(TEST_GATES:129)는 놓쳤다.
    ② 나열 목록과 구현이 사람 손을 거치며 갈라진다 — §17 착수 때 grep 출력에는
       13자리가 다 찍혀 있었는데 패턴은 7개만 썼고, 그 대조를 아무도 하지 않았다.
  → **찾는 행위 자체를 검사로** 만든다. 여기서 훑은 결과가 곧 대조 대상이므로
    '목록'과 '구현'이 갈라질 자리가 없다 (`AGENTS.md §3-10` 커버리지 폐포).

판정 규칙 — 기억이 아니라 표기 사실 하나로:
  같은 줄에 [게이트 별칭] 이 있고 그 **뒤쪽**에 [숫자][단위어] 가 오면 그 게이트의 기준선이다.
  단위어 = passed | tests | 검사 | 케이스.
  한 줄에 숫자가 여럿이면 **각자 자기 앞의 가장 가까운 별칭**에 붙는다
  (`TEST_GATES.md §1` 한 줄에 네 게이트가 같이 적힌다).

역사 수치 제외 — 셋 다 '그 시점 기록'이라 현재값과 달라야 정상이다:
  ① `docs/FREEZE_MANIFEST.md` · `docs/legacy/` — 파일 자체가 동결 스냅샷 / 보관본
  ② `- **직전 완료 …**` 불릿 블록(다음 최상위 불릿·제목 전까지) — 그 회차의 실행 실측
  ③ 줄에 `동결` 이 있음 — 동결 태그 자체의 수치(colcon 165 — `TEST_GATES.md §1`)
  ④ `N/N` 표기(`22/22`·`16/16`) — 실행 결과의 관용 표기

⚠ 문서 쓸 때 지킬 표기 규약:
  기준선이 **아닌** 수를 게이트 줄에 적을 때는 `케이스 12` 처럼 **단위어를 앞에** 쓴다.
  `12케이스` 로 쓰면 기준선으로 읽혀 FAIL 한다 — 놓치는 쪽이 아니라 시끄러운 쪽으로 틀린다.

사용:
  python3 tools/gate_baseline_scan.py --list
  python3 tools/gate_baseline_scan.py --expect pytest=159 --expect harness=22 …
  python3 tools/gate_baseline_scan.py --root <디렉터리> …     # 회귀 픽스처용

종료코드: 0 = 이상 없음 / 1 = 불일치·자리 부족 / 2 = 사용법 오류
"""

import os
import re
import sys

# ── 등록부 ───────────────────────────────────────────────────────────────
# min_hits = 현재형 표기가 있어야 할 **최소 자리 수**. 자리 '목록' 이 아니라 '개수' 라서
#   표기가 바뀌어도 안 부서지고, 자리가 조용히 사라지면(= 검사가 증발하면) FAIL 한다.
#   08-01 실측 자리: pytest 2 · colcon 2 · harness 4 · gate_regression 5 (합 13).
GATES = [
    ("pytest",          r"pytest",                               2),
    ("colcon",          r"colcon(?:\s+test-result)?",            2),
    ("harness",         r"test_harness_guards|harness guards",   4),
    ("gate_regression", r"test_gate_regression|gate regression",  5),
]

UNIT = r"(?:passed|tests|검사|케이스)"
# 숫자 앞: 다른 숫자나 '/' 가 오면 안 된다(`14/14` 의 뒷 14 배제).
# 숫자 뒤: `**` 와 공백 하나까지 허용(`(**14**케이스` · `22 검사` · `22검사`).
#          `/숫자` 가 이어지면 N/N 표기이므로 배제.
NUM_RE = re.compile(r"(?<![\d/])(\d+)(?!/\d)\*{0,2}[ ]?" + UNIT)

HIST_BULLET = re.compile(r"^- \*\*직전 완료")
BLOCK_END = re.compile(r"^(?:- \*\*|#{1,6} )")
FREEZE_WORD = "동결"

EXCLUDE_FILES = {"FREEZE_MANIFEST.md"}
EXCLUDE_DIRS = {"legacy"}


def scan_file(path, rel):
    """한 파일에서 (rel, lineno, gate, value) 를 전부 뽑는다."""
    hits = []
    in_hist = False
    with open(path, encoding="utf-8") as fh:
        for lineno, line in enumerate(fh, 1):
            # ② 직전 완료 블록 = 그 회차 실측 → 제외
            if HIST_BULLET.match(line):
                in_hist = True
                continue
            if in_hist and BLOCK_END.match(line):
                in_hist = False
            if in_hist:
                continue
            # ③ 동결 태그 자체의 수치 → 제외
            if FREEZE_WORD in line:
                continue

            # 별칭 위치를 먼저 모은다 (start 오름차순)
            aliases = []
            for name, alias_re, _ in GATES:
                for m in re.finditer(alias_re, line):
                    aliases.append((m.start(), name))
            if not aliases:
                continue
            aliases.sort()

            for m in NUM_RE.finditer(line):
                owner = None
                for start, name in aliases:
                    if start < m.start():
                        owner = name          # 자기 앞의 '가장 가까운' 별칭
                    else:
                        break
                if owner is not None:
                    hits.append((rel, lineno, owner, int(m.group(1))))
    return hits


def scan(root):
    hits = []
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in EXCLUDE_DIRS]
        for fn in sorted(filenames):
            if not fn.endswith(".md") or fn in EXCLUDE_FILES:
                continue
            full = os.path.join(dirpath, fn)
            hits.extend(scan_file(full, os.path.relpath(full, os.path.dirname(root) or ".")))
    hits.sort(key=lambda h: (h[2], h[0], h[1]))
    return hits


def main(argv):
    root = None
    expect = {}
    do_list = False
    it = iter(argv)
    for arg in it:
        if arg == "--list":
            do_list = True
        elif arg == "--root":
            root = next(it, None)
            if root is None:
                print("--root 에 디렉터리가 필요하다", file=sys.stderr)
                return 2
        elif arg == "--expect":
            spec = next(it, "")
            if "=" not in spec:
                print("--expect 는 이름=값 형태여야 한다: %r" % spec, file=sys.stderr)
                return 2
            name, _, val = spec.partition("=")
            if name not in dict((g[0], g) for g in GATES):
                print("모르는 게이트 이름: %r" % name, file=sys.stderr)
                return 2
            if not val.isdigit():
                print("실제값이 숫자가 아니다: %r (게이트 실행 실패?)" % val, file=sys.stderr)
                return 2
            expect[name] = int(val)
        else:
            print("알 수 없는 인자: %r" % arg, file=sys.stderr)
            return 2

    if root is None:
        root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs")
    root = os.path.normpath(root)
    if not os.path.isdir(root):
        print("디렉터리가 없다: %s" % root, file=sys.stderr)
        return 2

    hits = scan(root)

    if do_list:
        for rel, lineno, gate, val in hits:
            print("%s:%d:%s:%d" % (rel, lineno, gate, val))
        if not expect:
            return 0

    if not expect:
        print("--expect 가 하나도 없다 — 대조할 실제값이 필요하다", file=sys.stderr)
        return 2

    rc = 0
    for name, _alias, min_hits in GATES:
        if name not in expect:
            continue
        actual = expect[name]
        mine = [h for h in hits if h[2] == name]
        wrong = [h for h in mine if h[3] != actual]
        if wrong:
            rc = 1
            for rel, lineno, _g, val in wrong:
                print("FAIL %s 기준선 불일치: %s:%d 이 %d — 실제는 %d"
                      % (name, rel, lineno, val, actual))
        elif len(mine) < min_hits:
            # 자리가 사라지면 검사도 같이 증발한다. 그것 자체가 결함이다.
            rc = 1
            print("FAIL %s 기준선 표기가 %d 자리뿐 — 최소 %d (표기가 사라졌거나 단위어 누락)"
                  % (name, len(mine), min_hits))
        else:
            print("OK   %s 기준선 %d = 문서 %d 자리 전부" % (name, actual, len(mine)))
    return rc


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
