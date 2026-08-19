#!/usr/bin/env python3
"""실차 시행 bag 하나를 **현장에서 즉시** 판정한다 — 노트북으로 옮기지 않는다.

사용 (Jetson 에서 시행 직후):
    python3 tools/nav2_trial_report.py ~/d0_evidence/tune_0820_C1
    python3 tools/nav2_trial_report.py <bag> --deadband 0.38     # 그날 잰 D 로 판정
    python3 tools/nav2_trial_report.py <bag> --window 12.0 90.0  # 무장구간 대신 직접 지정

왜 이 도구가 있나
-----------------
08-19 에 시행마다 bag 을 노트북으로 `scp` 하고 분석해서 판정했다 — **시행당 약 10분**이
그 왕복에 들어갔다. 남은 작업일이 이틀이라 그 왕복을 없앤다. 이 도구는 시행이 끝난
자리에서 *"다음 시행을 어떻게 할 것인가"* 에 필요한 것만 인쇄한다.

무엇을 인쇄하는가
-----------------
1. **무장 구간** — `/drive/diag` 의 `z` 전이(0/1/2/3/4)와 해제 사유 `y`.
2. 🔴 **명령이 불감대를 넘었는가** — 08-19 의 핵심 질문. 제자리 회전 국면에서
   `|angular.z|` 최대와 불감대 초과 표본 수.
3. 🔴 **되먹임 덫이 살아 있는가** — `|명령ω| − |실측ω|` 가 `max_angular_accel · dt` 에
   붙어 있으면 RPP 가 실측 속도에 묶인 것이다(`MASTER_PLAN §7` 예약 40).
4. **이동** — `/odom` 기준 전진·횡 성분과 구간별 속도(정체 구간이 보인다).
5. **torn `/odom`** — `tools/odom_guard.py` 의 `check()` 를 **그대로 import** 해서 센다.
   🔴 규칙을 베껴 오지 않는다 — 두 벌이 되면 갈라진다(`AGENTS §3-10 ★`).
6. **주기 공백** — `/odom`·`/imu/data`·`/scan` 의 최대 수신 간격(41-g 재발 감시).

🔴 이 도구가 하지 않는 것
------------------------
- **goal 성공/실패를 말하지 않는다.** action 결과는 bag 에 없다 — 그건 시행 도구의
  터미널 출력이 정본이다.
- **원인을 말하지 않는다.** 숫자와 "무엇이 관찰됐다"까지다.
- **판정을 대신하지 않는다.** 불감대 `D` 는 `--deadband` 로 **그날 잰 값**을 넣어야 하고,
  안 넣으면 08-18 값(0.32)을 쓰되 그 사실을 인쇄한다.
"""

import argparse
import math
import os
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from odom_guard import check as odom_check  # noqa: E402

#: 08-18 §1-h-1 실측. 🔴 재조립 뒤에는 --deadband 로 그날 값을 준다.
FALLBACK_DEADBAND = 0.32
#: RPP 제어 주기 1/controller_frequency(20Hz).
CONTROL_DT = 0.05
LIN_EPS = 0.01
ANG_EPS = 0.01
Z_NAME = {0: "DISARMED", 1: "READY", 2: "ARMED", 3: "PENDING", 4: "ARMING"}
Y_NAME = {0: "없음", 1: "E-stop", 2: "READY 아님", 3: "이미 무장", 4: "E-stop 해제",
          7: "runtime overrun", 8: "spin 실패"}


def read_bag(path, topics):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = SequentialReader()
    r.open(StorageOptions(uri=path, storage_id="sqlite3"),
           ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = defaultdict(list)
    want = {t for t in topics if t in types}
    while r.has_next():
        topic, raw, ts = r.read_next()
        if topic in want:
            out[topic].append((ts / 1e9, deserialize_message(raw, get_message(types[topic]))))
    return out, set(types)


def fmt_span(rows):
    return "%.1f~%.1fs" % (rows[0][0], rows[-1][0]) if rows else "—"


def main(argv=None):
    ap = argparse.ArgumentParser(description="실차 시행 bag 즉석 판정")
    ap.add_argument("bag")
    ap.add_argument("--deadband", type=float, default=None,
                    help="그날 잰 제자리 불감대 D [rad/s]. 없으면 08-18 값 0.32 를 쓴다")
    ap.add_argument("--accel-dt", type=float, default=CONTROL_DT)
    ap.add_argument("--window", nargs=2, type=float, metavar=("FROM", "TO"),
                    help="무장 구간 자동탐지 대신 직접 지정 [s]")
    args = ap.parse_args(argv)

    D = args.deadband if args.deadband is not None else FALLBACK_DEADBAND
    topics = ["/drive/diag", "/drive/enabled", "/estop/state", "/cmd_vel",
              "/cmd_vel_nav", "/odom", "/imu/data", "/scan"]
    data, present = read_bag(args.bag, topics)
    if not data:
        print("🔴 읽을 토픽이 없다: %s" % args.bag)
        return 2
    t0 = min(v[0][0] for v in data.values() if v)
    rel = lambda t: t - t0                                    # noqa: E731

    print("=" * 72)
    print("  시행 판정 — %s" % os.path.basename(args.bag.rstrip("/")))
    src = ("(--deadband 지정)" if args.deadband is not None
           else "🔴 (08-18 값 — 재조립 뒤면 --deadband 로 그날 값을 줘라)")
    print("  불감대 D = %.3f %s" % (D, src))
    print("=" * 72)

    # ── 1. 무장 구간 ────────────────────────────────────────────────
    diag = data.get("/drive/diag", [])
    print("\n[1] 무장 상태 전이")
    armed_from = armed_to = None
    prev = None
    for t, m in diag:
        cur = (int(m.x), int(m.y), int(m.z))
        if cur != prev:
            print("    +%7.1fs  호출누계 %-4d 사유 %-14s 상태 %s"
                  % (rel(t), cur[0], Y_NAME.get(cur[1], str(cur[1])),
                     Z_NAME.get(cur[2], str(cur[2]))))
            if cur[2] == 2 and armed_from is None:
                armed_from = rel(t)
            if cur[2] != 2 and armed_from is not None and armed_to is None:
                armed_to = rel(t)
            prev = cur
    if armed_to is None and diag:
        armed_to = rel(diag[-1][0])
    if args.window:
        armed_from, armed_to = args.window
    if armed_from is None:
        print("    🔴 ARMED 구간이 없다 — 무장 안 된 시행이다.")
        return 1
    print("    → 판정 구간 = +%.1f ~ +%.1fs (%.1f초)"
          % (armed_from, armed_to, armed_to - armed_from))

    def win(rows):
        return [(rel(t), m) for t, m in rows if armed_from <= rel(t) <= armed_to]

    # ── 2. 명령이 불감대를 넘었는가 ─────────────────────────────────
    print("\n[2] 🔴 명령이 불감대를 넘었는가")
    for topic in ("/cmd_vel_nav", "/cmd_vel"):
        s = win(data.get(topic, []))
        if not s:
            print("    %-14s 표본 없음" % topic)
            continue
        rot = [m for _, m in s if abs(m.linear.x) < LIN_EPS and abs(m.angular.z) > ANG_EPS]
        fwd = [m for _, m in s if abs(m.linear.x) >= LIN_EPS]
        angs = sorted(abs(m.angular.z) for _, m in s if abs(m.angular.z) > ANG_EPS)
        over = [a for a in angs if a > D]
        print("    %-14s 표본 %-5d 회전국면 %5.1f%%  전진 %5.1f%%"
              % (topic, len(s), 100.0 * len(rot) / len(s), 100.0 * len(fwd) / len(s)))
        if angs:
            print("        |ω| 중앙 %.3f  최대 %.3f   🔴 불감대(%.2f) 초과 %d/%d (%.1f%%)"
                  % (angs[len(angs) // 2], angs[-1], D, len(over), len(angs),
                     100.0 * len(over) / len(angs)))
            if topic == "/cmd_vel_nav":
                print("        → %s" % ("🟢 넘었다 — 회전 명령이 살아 있다"
                                        if over else "🔴 한 번도 못 넘었다 — 08-19 와 같은 상태"))

    # ── 3. 되먹임 덫 ────────────────────────────────────────────────
    print("\n[3] 🔴 되먹임 덫 (|명령ω| − |실측ω| 이 상한에 붙어 있나)")
    cmds = win(data.get("/cmd_vel_nav", []))
    odom = win(data.get("/odom", []))
    if cmds and odom:
        ot = [t for t, _ in odom]
        ow = [m.twist.twist.angular.z for _, m in odom]
        import bisect
        diffs = []
        for t, m in cmds:
            if abs(m.linear.x) >= LIN_EPS or abs(m.angular.z) <= ANG_EPS:
                continue
            i = bisect.bisect_left(ot, t) - 1
            if i >= 0:
                diffs.append(abs(m.angular.z) - abs(ow[i]))
        if diffs:
            ds = sorted(diffs)
            med = ds[len(ds) // 2]
            print("    회전국면 표본 %d · 중앙 %.4f · 90%% %.4f"
                  % (len(ds), med, ds[int(0.9 * len(ds))]))
            for a in (2.0, 10.0):
                lim = a * args.accel_dt
                near = sum(1 for d in ds if abs(d - lim) < 0.005)
                if near > 0.5 * len(ds):
                    print("        🔴 %.0f%% 가 %.3f 에 붙어 있다 = max_angular_accel %.1f 로 묶임"
                          % (100.0 * near / len(ds), lim, a))
        else:
            print("    회전국면 표본 없음")
    else:
        print("    /cmd_vel_nav 또는 /odom 이 없다")

    # ── 4. 이동 ────────────────────────────────────────────────────
    print("\n[4] 이동 (/odom · 판정구간 시작 기준)")
    if odom:
        x0 = odom[0][1].pose.pose.position.x
        y0 = odom[0][1].pose.pose.position.y
        pts = [(t, m.pose.pose.position.x - x0, m.pose.pose.position.y - y0) for t, m in odom]
        n = len(pts)
        prev_i = 0
        for k in range(1, 5):
            i = min(int(k * (n - 1) / 4), n - 1)
            dt = pts[i][0] - pts[prev_i][0]
            dd = math.hypot(pts[i][1] - pts[prev_i][1], pts[i][2] - pts[prev_i][2])
            print("    +%6.1fs  dx=%+.3f dy=%+.3f  |이동| %.3f m   구간 %.3f m (%.3f m/s)"
                  % (pts[i][0], pts[i][1], pts[i][2],
                     math.hypot(pts[i][1], pts[i][2]), dd, dd / dt if dt > 0 else 0.0))
            prev_i = i
        print("    🔵 최종 이동 %.3f m  (dy %+.3f — ⚠ 예약 48 로 하한이다)"
              % (math.hypot(pts[-1][1], pts[-1][2]), pts[-1][2]))

    # ── 5. torn /odom ──────────────────────────────────────────────
    print("\n[5] torn /odom (규칙 = tools/odom_guard.py)")
    bad = []
    for t, m in data.get("/odom", []):
        why = odom_check(m.twist.covariance, m.twist.twist.linear.x,
                         m.twist.twist.linear.y, m.twist.twist.angular.z)
        if why:
            bad.append((rel(t), why))
    if bad:
        print("    🔴 %d 건 — 예약 50-3 재개방 조건이다" % len(bad))
        for t, why in bad[:5]:
            print("        +%7.1fs  %s" % (t, why))
    else:
        print("    🟢 0 건 (표본 %d)" % len(data.get("/odom", [])))

    # ── 6. 주기 공백 ───────────────────────────────────────────────
    print("\n[6] 수신 공백 최대 (41-g 재발 감시)")
    for topic, limit in (("/odom", 0.0333), ("/imu/data", 0.0333), ("/scan", 0.150)):
        rows = data.get(topic, [])
        if len(rows) < 2:
            print("    %-12s 표본 부족" % topic)
            continue
        gaps = [rows[i + 1][0] - rows[i][0] for i in range(len(rows) - 1)]
        mx = max(gaps)
        over = sum(1 for g in gaps if g > limit)
        mark = "🔴" if mx > 0.25 else ("🔶" if over else "🟢")
        print("    %-12s 표본 %-6d 최대 %6.1f ms  상한(%.0fms) 초과 %d 건  %s"
              % (topic, len(rows), mx * 1000, limit * 1000, over, mark))

    print("\n" + "=" * 72)
    print("  🔴 이 출력은 관측이지 판정이 아니다. goal 성공/실패는 시행 도구 터미널이 정본.")
    print("=" * 72)
    return 0


if __name__ == "__main__":
    sys.exit(main())
