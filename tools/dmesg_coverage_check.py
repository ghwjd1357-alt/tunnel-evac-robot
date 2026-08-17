#!/usr/bin/env python3
"""bag 구간을 커널 로그가 실제로 덮는지 판정한다 (검토 §73.2 · §76.2 · §77.2).

세 번 같은 자리에서 틀렸다. 08-17 낮 `dmesg_drive_0817_1320.log` 는 13:25 주행보다
앞선 스냅샷이었는데 정본 3곳이 "그 시각 USB 오류 없음" 을 근거로 USB 를 기각했고(§73.2),
같은 날 밤 `dmesg_fw162_ground_soak_0817_2320.log` 는 정본이 한 걸음 더 나가
**"전 구간을 덮고 오류 0건"** 이라고 적었다(§76.2). 그래서 만든 것이 이 판정기인데,
그 판정기마저 **마지막 커널 사건을 취득 종료로 대용**했다(§77.2).

🔴 **사건 시각과 취득 시각은 같은 경계가 아니다.** `dmesg -T > file` 은 마지막 사건 뒤
한참 지나서도 찍을 수 있다. 조용한 정상 커널에서 주행 **뒤** 제대로 받은 로그를
"마지막 사건이 오래됐다" 는 이유로 거짓 FAIL 내면, 다음 사람은 이 도구를 끄게 된다.

판정 규칙 — **비대칭**이다. 이게 이 파일의 핵심이다:

* **머리**(bag 시작): 로그 첫 줄이 bag 시작보다 늦으면 FAIL. 링 버퍼가 그 앞까지
  닿지 않으므로 그 구간의 사건은 있었어도 이미 잘렸다.
* **꼬리**(bag 종료): 마지막 사건이 bag 종료 **이후**면 취득은 필연적으로 그 뒤다
  → 덮었다. 이건 부등식이라 건전하다.
* 마지막 사건이 bag 종료 **이전**이면 그것만으로는 아무 결론도 못 낸다. 이때만
  취득 경계가 필요하다:
  - `--capture` 가 남긴 sidecar 가 있고 sha256 이 맞으면 그 취득 종료로 판정한다.
  - sidecar 가 변조·불일치면 **rc=2**. 없으면 **rc=2**.
  - `--trust-mtime` 은 "이 파일은 원본" 이라는 **검증되지 않은 사람의 주장**이다.
    그래서 **FAIL(rc=1) 은 낼 수 있어도 PASS 는 못 낸다** — 재시험을 요구하는 방향은
    약한 증거로 가도 안전하지만, 무사고 증명은 약한 증거로 갈 수 없다. mtime 이
    bag 종료 뒤면 rc=2 다.
* 덮은 구간에 `disconnect`/`reset` 이 있으면 **FAIL 이 아니라 경고**다.
  판정(덮었는가)과 관측(무엇이 보이는가)을 한 출구로 합치지 않는다 — 합치면
  "오류가 있으니 FAIL" 과 "못 봤으니 FAIL" 이 같은 종료코드가 되어 구별이 사라진다.

사용법:
    python3 tools/dmesg_coverage_check.py --capture <저장경로>     # 취득 (경계 소유자)
    python3 tools/dmesg_coverage_check.py <bag_dir> <dmesg_log> [--trust-mtime]

종료코드: 0 = 덮었다 · 1 = 못 덮었다 · 2 = 판정 불능(형식 미상 · 취득 경계 없음 ·
읽기 실패). 🔴 **rc=2 를 rc=0 처럼 읽지 않는다.**
"""

import hashlib
import json
import os
import re
import subprocess
import sys
import time

#: `dmesg -T` 한 줄의 머리. 예: `[Mon Aug 17 22:44:41 2026] usb 1-2.1: USB disconnect`
#: ⚠ `dmesg` (T 없음)는 부팅 이후 초 단위라 절대시각이 없다 — 그건 판정 불능으로 뺀다.
DMESG_STAMP = re.compile(r"^\[(\w{3} \w{3} +\d+ \d{2}:\d{2}:\d{2} \d{4})\]")
DMESG_FORMAT = "%a %b %d %H:%M:%S %Y"

#: 덮은 구간에서 눈에 띄면 경고로 인쇄할 것. 판정에는 쓰지 않는다.
NOTABLE = re.compile(r"disconnect|reset high-speed|ttyACM|cp210x|xhci.*error", re.I)

#: sidecar 규약 — 이 도구가 직접 취득했을 때만 쓴다.
SIDECAR_SUFFIX = ".capture.json"
SIDECAR_VERSION = 2
CAPTURE_TIMEOUT_S = 30


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
    """dmesg -T 로그에서 (첫 사건 시각, 마지막 사건 시각, 전체 줄 수) 를 읽는다.

    🔴 반환하는 것은 **사건** 시각이지 취득 시각이 아니다 (§77.2). 이름을 헷갈리면
    다시 같은 실수를 한다.
    """
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


def sha256_of(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sidecar_path(log_path):
    return log_path + SIDECAR_SUFFIX


def read_sidecar(log_path):
    """취득 경계 sidecar 를 읽어 `(상태, 정보)` 를 돌려준다.

    상태: `none`(없음) · `bad`(형식 깨짐) · `mismatch`(이 파일의 것이 아님) · `ok`.
    """
    path = sidecar_path(log_path)
    if not os.path.exists(path):
        return "none", None
    try:
        info = json.loads(open(path, encoding="utf-8").read())
        end = float(info["capture_end"])
        start = float(info["capture_start"])
        recorded = str(info["sha256"])
    except Exception as exc:                                 # noqa: BLE001
        return "bad", str(exc)
    if recorded != sha256_of(log_path):
        return "mismatch", recorded
    return "ok", {"capture_start": start, "capture_end": end}


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


def capture(out_path):
    """`dmesg -T` 를 받고 **취득 시작·종료 시각**을 sidecar 에 같이 남긴다.

    이 도구가 경계의 소유자다 (예약 46). 사람이 기억해서 적는 절차는 08-17 에 두 번
    깨졌으므로, 취득한 쪽이 그 자리에서 기계로 남긴다.
    """
    started = time.time()
    try:
        proc = subprocess.run(
            ["dmesg", "-T"], capture_output=True, text=True, timeout=CAPTURE_TIMEOUT_S)
    except Exception as exc:                                 # noqa: BLE001
        print("  취득 실패 — `dmesg -T` 를 실행하지 못했다: %s" % exc)
        return 2
    ended = time.time()
    if proc.returncode != 0:
        print("  취득 실패 — `dmesg -T` rc=%d: %s" % (proc.returncode, proc.stderr.strip()))
        print("     권한이면 `sudo dmesg -T` 가 되는지부터 본다.")
        return 2
    with open(out_path, "w", encoding="utf-8") as handle:
        handle.write(proc.stdout)
    info = {
        "tool": "dmesg_coverage_check",
        "version": SIDECAR_VERSION,
        "capture_start": started,
        "capture_end": ended,
        "sha256": sha256_of(out_path),
        "log": os.path.basename(out_path),
    }
    with open(sidecar_path(out_path), "w", encoding="utf-8") as handle:
        json.dump(info, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    print("── 커널 로그 취득 (경계 동봉) ──────────────────────────────")
    print("  로그    : %s" % out_path)
    print("  경계    : %s ~ %s" % (fmt(started), fmt(ended)))
    print("  sidecar : %s" % sidecar_path(out_path))
    print("  🔴 이 두 파일을 **같이** 옮긴다. sidecar 가 없으면 판정 불능(rc=2)이다.")
    return 0


def resolve_capture_end(log_path, trust_mtime, bag_hi):
    """취득 **종료** 시각을 결정한다 → `(출처, 시각 또는 None, 설명)`.

    출처 `none` 이면 판정 불능이다. `mtime` 은 통과를 만들 수 없다 — 호출부 계약.
    """
    state, info = read_sidecar(log_path)
    if state == "ok":
        return "sidecar", info["capture_end"], "sidecar 취득 종료"
    if state == "mismatch":
        return "none", None, "sidecar 의 sha256 이 이 로그와 다르다 (복사·변조·짝 어긋남)"
    if state == "bad":
        return "none", None, "sidecar 를 읽을 수 없다: %s" % info
    if trust_mtime:
        mtime = os.path.getmtime(log_path)
        if mtime >= bag_hi:
            return "none", None, (
                "mtime %s 은 bag 종료 뒤지만, **검증되지 않은 mtime 으로 '덮었다' 를 "
                "낼 수 없다**" % fmt(mtime))
        return "mtime", mtime, "파일 mtime (원본성 미검증 — FAIL 방향으로만 쓴다)"
    return "none", None, "취득 경계가 없다 (`--capture` sidecar 도 `--trust-mtime` 도 없음)"


def check(bag_dir, log_path, trust_mtime=False):
    print("── 커널 로그 커버리지 판정 (검토 §73.2·§76.2·§77.2) ──────────")
    try:
        bag_lo, bag_hi = read_bag_window(bag_dir)
    except Exception as exc:                                 # noqa: BLE001
        print("  판정 불능 — bag: %s" % exc)
        return 2
    try:
        log_lo, log_hi, lines = read_log_window(log_path)
    except Exception as exc:                                 # noqa: BLE001
        print("  판정 불능 — 로그를 읽을 수 없다: %s" % exc)
        return 2
    if log_lo is None:
        print("  판정 불능 — `dmesg -T` 절대시각이 없다 (`-T` 없이 저장했다)")
        print("     로그: %s (%d줄)" % (log_path, lines))
        return 2

    print("  bag   : %s ~ %s  (%.1f초)" % (fmt(bag_lo), fmt(bag_hi), bag_hi - bag_lo))
    print("  사건  : %s ~ %s  (%d줄)" % (fmt(log_lo), fmt(log_hi), lines))

    # ① 머리 — 링 버퍼가 bag 시작 앞까지 닿는가. 안 닿으면 그 구간은 이미 잘렸다.
    late_start = log_lo - bag_lo
    if late_start > 0:
        print("  ❌ FAIL — 로그가 bag 시작 앞까지 닿지 않는다")
        print("     시작을 %.1f분 놓쳤다" % (late_start / 60.0))
        print("     🔴 이 로그로 'USB 오류 없음' 을 쓸 수 없다. 부재는 관측이 아니다.")
        return 1

    # ② 꼬리 — 마지막 **사건**이 bag 종료 뒤면 취득은 필연적으로 그 뒤다 (부등식).
    if log_hi >= bag_hi:
        return _pass(log_path, bag_lo, bag_hi, "마지막 커널 사건이 bag 종료 이후")

    # ③ 마지막 사건이 bag 종료 앞이면 그것만으로는 아무 결론도 못 낸다 (§77.2).
    source, cap_end, why = resolve_capture_end(log_path, trust_mtime, bag_hi)
    if source == "none":
        print("  ⚠ 판정 불능 — 마지막 사건 %s 은 bag 종료 %s 앞이다."
              % (fmt(log_hi), fmt(bag_hi)))
        print("     마지막 사건은 취득 종료가 아니므로 이것만으로는 못 덮었다고도 못 한다.")
        print("     사유: %s" % why)
        print("     🔴 다음부터 `python3 tools/dmesg_coverage_check.py --capture <경로>` 로")
        print("        받는다 — 취득 경계가 sidecar 에 같이 남는다.")
        return 2

    print("  취득  : %s  (%s)" % (fmt(cap_end), why))
    if cap_end >= bag_hi:
        if source == "mtime":                                # 계약 — 여기 오면 안 된다
            raise AssertionError("mtime 으로 통과를 만들 수 없다")
        return _pass(log_path, bag_lo, bag_hi, why)

    print("  ❌ FAIL — 취득이 bag 종료 전에 끝났다")
    print("     끝을 %.1f분 놓쳤다" % ((bag_hi - cap_end) / 60.0))
    if source == "mtime":
        print("     (약한 증거인 mtime 이지만 방향이 FAIL 이라 채택한다 — 재시험을 요구한다)")
    print("     주행 **종료 뒤** `--capture` 로 다시 받는다.")
    return 1


def _pass(log_path, bag_lo, bag_hi, why):
    notes = notable_lines(log_path, bag_lo, bag_hi)
    print("  ✅ 덮었다 — 이 구간의 판정에 로그를 쓸 수 있다 (근거: %s)" % why)
    if notes:
        print("  ⚠ 구간 안 눈여겨볼 줄 %d건 (판정 아님 · 사람이 읽는다):" % len(notes))
        for line in notes[:20]:
            print("     %s" % line)
    else:
        print("  구간 안 USB/ACM 특이 줄 0건")
    return 0


def main(argv):
    args = list(argv[1:])
    if len(args) == 2 and args[0] == "--capture":
        return capture(args[1])
    trust = "--trust-mtime" in args
    rest = [a for a in args if not a.startswith("--")]
    if len(rest) != 2 or (len(args) - len(rest)) != (1 if trust else 0):
        print(__doc__)
        return 2
    return check(rest[0], rest[1], trust_mtime=trust)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
