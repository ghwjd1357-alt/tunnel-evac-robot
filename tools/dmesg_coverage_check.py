#!/usr/bin/env python3
"""bag 구간을 커널 로그가 실제로 덮는지 판정한다 (검토 §73.2 · §76.2).

두 번 같은 실수를 했다. 08-17 `dmesg_drive_0817_1320.log` 는 13:25 주행보다 앞선
스냅샷이었는데 정본 3곳이 "그 시각 USB 오류 없음" 을 근거로 USB 를 기각했다(§73.2).
같은 날 밤 `dmesg_fw162_ground_soak_0817_2320.log` 는 soak 시작 **43초 전**에 떠서
사건 구간을 34분 못 미쳤는데, 이번에는 정본이 한 걸음 더 나가 **"전 구간을 덮고
오류 0건"** 이라고 적었다(§76.2).

그때 만든 대책은 런북 문장(*"주행 종료 후 `dmesg -T` 까지 저장한다"*) 하나였고,
문장은 이미 있었는데도 지켜지지 않았다. 🔴 **진짜 클래스는 "절차가 없다" 가 아니라
"절차를 지켰는지 아무도 검사하지 않는다" 이므로, 사람이 지킬 문장 대신 이 판정기를 둔다.**

판정 규칙 — 부재는 관측이 아니다:

* 로그가 bag 구간을 **완전히 덮지 못하면 FAIL**. "오류가 안 보인다" 를 쓸 수 없다.
* 덮는데 `disconnect`/`reset` 이 있으면 **FAIL 이 아니라 경고**로 인쇄한다.
  판정(덮었는가)과 관측(무엇이 보이는가)을 한 출구로 합치지 않는다 — 합치면
  "오류가 있으니 FAIL" 과 "못 봤으니 FAIL" 이 같은 종료코드가 되어 구별이 사라진다.

사용법:
    python3 tools/dmesg_coverage_check.py <bag_dir> <dmesg_log>

종료코드: 0 = 덮었다 · 1 = 못 덮었다(또는 읽기 실패) · 2 = 판정 불능(형식 미상)
"""

import os
import re
import sys
import time

#: `dmesg -T` 한 줄의 머리. 예: `[Mon Aug 17 22:44:41 2026] usb 1-2.1: USB disconnect`
#: ⚠ `dmesg` (T 없음)는 부팅 이후 초 단위라 절대시각이 없다 — 그건 판정 불능으로 뺀다.
DMESG_STAMP = re.compile(r"^\[(\w{3} \w{3} +\d+ \d{2}:\d{2}:\d{2} \d{4})\]")
DMESG_FORMAT = "%a %b %d %H:%M:%S %Y"

#: 덮은 구간에서 눈에 띄면 경고로 인쇄할 것. 판정에는 쓰지 않는다.
NOTABLE = re.compile(r"disconnect|reset high-speed|ttyACM|cp210x|xhci.*error", re.I)


def read_bag_window(bag_dir):
    """bag metadata 에서 (시작 epoch 초, 종료 epoch 초) 를 읽는다."""
    meta = os.path.join(bag_dir, "metadata.yaml")
    if not os.path.exists(meta):
        raise FileNotFoundError(meta)
    text = open(meta, encoding="utf-8").read()
    start = re.search(r"nanoseconds_since_epoch:\s*(\d+)", text)
    dur = re.search(r"duration:\s*\n\s*nanoseconds:\s*(\d+)", text)
    if not start or not dur:
        raise ValueError("metadata.yaml 에서 starting_time/duration 을 못 읽었다")
    t0 = int(start.group(1)) / 1e9
    return t0, t0 + int(dur.group(1)) / 1e9


def read_log_window(log_path):
    """dmesg -T 로그에서 (첫 줄 시각, 마지막 줄 시각, 전체 줄 수) 를 읽는다."""
    first = last = None
    lines = 0
    with open(log_path, encoding="utf-8", errors="ignore") as handle:
        for line in handle:
            lines += 1
            hit = DMESG_STAMP.match(line)
            if hit:
                stamp = time.mktime(time.strptime(hit.group(1), DMESG_FORMAT))
                if first is None:
                    first = stamp
                last = stamp
    return first, last, lines


def notable_lines(log_path, lo, hi):
    """덮은 구간 안에서 눈여겨볼 커널 줄만 뽑는다 (판정 아님)."""
    out = []
    with open(log_path, encoding="utf-8", errors="ignore") as handle:
        for line in handle:
            hit = DMESG_STAMP.match(line)
            if not hit:
                continue
            stamp = time.mktime(time.strptime(hit.group(1), DMESG_FORMAT))
            if lo <= stamp <= hi and NOTABLE.search(line):
                out.append(line.rstrip())
    return out


def fmt(epoch):
    return time.strftime("%m-%d %H:%M:%S", time.localtime(epoch))


def check(bag_dir, log_path):
    print("── 커널 로그 커버리지 판정 (검토 §73.2·§76.2) ────────────────")
    try:
        bag_lo, bag_hi = read_bag_window(bag_dir)
    except Exception as exc:                                 # noqa: BLE001
        print("  판정 불능 — bag: %s" % exc)
        return 2
    log_lo, log_hi, lines = read_log_window(log_path)
    if log_lo is None:
        print("  판정 불능 — `dmesg -T` 절대시각이 없다 (`-T` 없이 저장했다)")
        print("     로그: %s (%d줄)" % (log_path, lines))
        return 2

    print("  bag  : %s ~ %s  (%.1f초)" % (fmt(bag_lo), fmt(bag_hi), bag_hi - bag_lo))
    print("  dmesg: %s ~ %s  (%d줄)" % (fmt(log_lo), fmt(log_hi), lines))

    late_start = log_lo - bag_lo
    early_end = bag_hi - log_hi
    if late_start > 0 or early_end > 0:
        print("  ❌ FAIL — 로그가 bag 구간을 덮지 못한다")
        if late_start > 0:
            print("     시작을 %.1f분 놓쳤다" % (late_start / 60.0))
        if early_end > 0:
            print("     끝을 %.1f분 놓쳤다" % (early_end / 60.0))
        print("     🔴 이 로그로 'USB 오류 없음' 을 쓸 수 없다. 부재는 관측이 아니다.")
        print("     주행 **종료 뒤** `dmesg -T > <경로>` 로 다시 받는다.")
        return 1

    notes = notable_lines(log_path, bag_lo, bag_hi)
    print("  ✅ 덮었다 — 이 구간의 판정에 로그를 쓸 수 있다")
    if notes:
        print("  ⚠ 구간 안 눈여겨볼 줄 %d건 (판정 아님 · 사람이 읽는다):" % len(notes))
        for line in notes[:20]:
            print("     %s" % line)
    else:
        print("  구간 안 USB/ACM 특이 줄 0건")
    return 0


def main(argv):
    if len(argv) != 3:
        print(__doc__)
        return 2
    return check(argv[1], argv[2])


if __name__ == "__main__":
    sys.exit(main(sys.argv))
