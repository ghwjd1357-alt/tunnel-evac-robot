#!/usr/bin/env python3
"""여러 지면 주행 bag 을 **한 표**로 세운다 (08-13 밤 신설).

왜 이 도구가 있나
-----------------
08-13 밤에 bag 11 개를 하나씩 열어 손으로 훑어서야 원인이 보였다. 시행 하나만 보면
`odom/줄자 = 1.238` 이 "반지름이 틀렸다" 로 읽히는데, **여러 시행을 나란히 놓자**
오후와 밤의 회전 거동이 완전히 같다는 것이 드러났고 — 그러면 로봇은 안 변했고 —
남는 차이는 줄자 하나뿐이라는 결론이 나왔다.

🔴 **한 시행은 상수를 못 정한다.** 이 도구는 그 사실을 도구 형태로 굳힌 것이다.

무엇을 찍나
----------
  · `odom Δyaw` vs `IMU Δyaw` 와 그 비 — 🔴 **시행 간에 이 비가 흔들리면 거동이 바뀐 것**
  · 경로장 · 순항속도 · 편향 각속도(직진 명령인데 IMU 가 본 회전)
  · 줄자를 주면 **줄자 타당성**(줄자÷시간 vs 명령). 🔴 `odom/줄자` 판정은
    `drive_ground_report.py` 가 한다 — 창 정의가 달라 두 도구가 다른 비를 낸다

읽는 법
------
  · **비가 시행마다 같다** → 로봇은 안 변했다. 어긋남은 상수나 측정 쪽이다
  · **비가 갈린다** → 그 시행에서 로봇이 다르게 굴렀다(슬립·배선·바닥)
  · **편향 각속도 부호가 한쪽으로 몰린다** → 계통 편향(예약 39)

사용
----
    python3 tools/ground_run_table.py <bag> [<bag> ...]
    python3 tools/ground_run_table.py ~/Desktop/d0_evidence/r2_*
    python3 tools/ground_run_table.py <bag>=3065 <bag>=3900     # `=줄자mm` 로 준다

    종료 0 = 전부 읽었다 / 1 = 하나 이상 못 읽었다 / 2 = 사용법

정본 = docs/PITFALLS.md §12 · docs/MASTER_PLAN.md §7 예약 32-e.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def unwrap(series):
    """±π 를 넘어가는 yaw 를 이어 붙인다. 여러 바퀴를 도는 시행에 필수다."""
    out, prev, acc = [], None, 0.0
    for stamp, value in series:
        if prev is not None:
            step = value - prev
            while step > math.pi:
                step -= 2.0 * math.pi
            while step < -math.pi:
                step += 2.0 * math.pi
            acc += step
        prev = value
        out.append((stamp, acc))
    return out


def read_bag(path):
    """bag 에서 /odom · /imu/yaw_deg · /cmd_vel 만 뽑는다."""
    from rclpy.serialization import deserialize_message      # noqa: PLC0415
    from rosbag2_py import (ConverterOptions, SequentialReader,  # noqa: PLC0415
                            StorageOptions)
    from rosidl_runtime_py.utilities import get_message      # noqa: PLC0415

    reader = SequentialReader()
    reader.open(StorageOptions(uri=path, storage_id="sqlite3"),
                ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    odom, imu, cmd = [], [], []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        t = stamp / 1e9
        if topic == "/odom":
            m = deserialize_message(data, get_message(types[topic]))
            p = m.pose.pose.position
            odom.append((t, p.x, p.y, yaw_of(m.pose.pose.orientation)))
        elif topic == "/imu/yaw_deg":
            imu.append((t, deserialize_message(data, get_message(types[topic])).data))
        elif topic == "/cmd_vel":
            m = deserialize_message(data, get_message(types[topic]))
            cmd.append((t, m.linear.x, m.angular.z))
    return odom, imu, cmd


def summarize(path, tape_mm=None):
    odom, imu, cmd = read_bag(path)
    if not odom:
        raise ValueError("/odom 이 없다")
    nonzero = [(t, lx, az) for t, lx, az in cmd if abs(lx) > 1e-9 or abs(az) > 1e-9]
    if not nonzero:
        raise ValueError("비영 /cmd_vel 이 없다")
    t_start, t_end = nonzero[0][0] - 0.3, nonzero[-1][0] + 3.0
    seg = [r for r in odom if t_start <= r[0] <= t_end]
    if len(seg) < 5:
        raise ValueError("창 안의 /odom 표본이 너무 적다")

    path_mm = sum(math.hypot(b[1] - a[1], b[2] - a[2])
                  for a, b in zip(seg, seg[1:])) * 1000.0
    dur = seg[-1][0] - seg[0][0]
    yaws = unwrap([(r[0], r[3]) for r in seg])
    odom_dyaw = math.degrees(yaws[-1][1] - yaws[0][1])

    imu_dyaw = float("nan")
    if imu:
        near = lambda t: min(imu, key=lambda x: abs(x[0] - t))[1]   # noqa: E731
        imu_dyaw = near(seg[-1][0]) - near(seg[0][0])

    return dict(
        name=os.path.basename(path.rstrip("/")),
        cmd_lin=nonzero[0][1], cmd_ang=nonzero[0][2],
        dur=dur, path_mm=path_mm,
        odom_dyaw=odom_dyaw, imu_dyaw=imu_dyaw,
        ratio=(odom_dyaw / imu_dyaw) if abs(imu_dyaw) > 0.5 else float("nan"),
        bias=(math.radians(imu_dyaw) / dur) if dur > 0 else float("nan"),
        odom_mps=(path_mm / 1000.0 / dur) if dur > 0 else float("nan"),
        tape_mm=tape_mm,
        odom_over=(path_mm / tape_mm) if tape_mm else None,
        tape_mps=((tape_mm / 1000.0 / dur) if tape_mm and dur > 0 else None),
    )


def main(argv):
    if len(argv) < 2:
        print(__doc__.split("사용\n----\n", 1)[1].split("\n\n")[0], file=sys.stderr)
        return 2

    rows, failed = [], []
    for raw in argv[1:]:
        path, _, tape = raw.partition("=")
        try:
            rows.append(summarize(os.path.expanduser(path),
                                  float(tape) if tape else None))
        except Exception as exc:                             # noqa: BLE001
            failed.append((os.path.basename(path.rstrip("/")), exc))

    if rows:
        print("=" * 104)
        print("지면 주행 요약 — 🔴 한 시행은 상수를 못 정한다 (PITFALLS §12)")
        print("=" * 104)
        print("%-22s %6s %7s %8s %9s %9s %8s %10s" %
              ("bag", "명령", "시간s", "경로mm", "odomΔyaw", "IMUΔyaw",
               "odom/IMU", "편향rad/s"))
        for r in rows:
            cmd = ("lin %.2f" % r["cmd_lin"]) if abs(r["cmd_lin"]) > 1e-9 \
                else ("ang %.2f" % r["cmd_ang"])
            print("%-22s %6s %7.1f %8.1f %8.2f° %8.2f° %8.4f %10.5f" %
                  (r["name"][:22], cmd, r["dur"], r["path_mm"],
                   r["odom_dyaw"], r["imu_dyaw"], r["ratio"],
                   r["bias"] if abs(r["cmd_ang"]) < 1e-9 else float("nan")))

        taped = [r for r in rows if r["tape_mm"]]
        if taped:
            print("\n%-22s %9s %10s %10s" %
                  ("bag", "줄자mm", "줄자m/s", "명령대비"))
            for r in taped:
                vs = r["tape_mps"] / max(abs(r["cmd_lin"]), 1e-9)
                flag = "  🔴 줄자 의심" if not 0.85 <= vs <= 1.30 else ""
                print("%-22s %9.0f %10.4f %9.2f배%s" %
                      (r["name"][:22], r["tape_mm"], r["tape_mps"], vs, flag))
            # 🔴 `odom/줄자` 는 **여기서 안 찍는다.** 이 도구의 창은
            #    `비영 명령 첫 발행 - 0.3s ~ 마지막 + 3.0s` 로 거칠게 잡은 것이고,
            #    `drive_ground_report.py` 는 창의 시작을 **움직이기 시작한 시점**으로
            #    되돌려 잡는다 — 줄자가 재는 구간과 맞추기 위해서다. 두 창이 다르므로
            #    두 도구가 같은 bag 에 다른 비를 낸다(실측 0.754 vs 0.807).
            #    ⚠ 그런 자리는 하나만 권위를 가져야 한다. R2 판정의 정본은
            #      `drive_ground_report.py` 다. 이 표는 **시행 간 비교** 전용이다.
            print("  ⚠ `odom/줄자` 는 이 표가 안 낸다 — 창 정의가 달라 값이 갈린다."
                  " 판정 정본 = tools/drive_ground_report.py")

        # 🔴 제자리 회전과 완만한 곡선은 **다른 기동**이다 — ICR 이 선회 반경에 따라
        #    옮겨가므로 두 무리를 섞어 폭을 재면 의미 없는 큰 수가 나온다
        #    (실측: 제자리 0.991 vs 곡선 0.838 — REAL_ROBOT_VALUES §1-c).
        for label, group in (("제자리 회전", [r for r in rows if abs(r["cmd_ang"]) > 1e-9]),
                             ("직진·곡선", [r for r in rows if abs(r["cmd_ang"]) < 1e-9])):
            got = [r["ratio"] for r in group if r["ratio"] == r["ratio"]]
            if len(got) >= 2:
                lo, hi = min(got), max(got)
                print("\n  [%s] odom/IMU = %.4f ~ %.4f (폭 %.1f%%)"
                      % (label, lo, hi, (hi - lo) / max(abs(lo), 1e-9) * 100))
                print("     🔴 같은 무리 안에서 이 비가 흔들리면 **로봇이 다르게 굴렀다**는"
                      " 뜻이고, 고르면 어긋남은 상수·측정 쪽이다")

        # 🔴 회전 명령 시행의 '편향' 은 그냥 회전 속도다 — 직진 명령만 센다.
        signs = [r["bias"] for r in rows
                 if r["bias"] == r["bias"] and abs(r["cmd_ang"]) < 1e-9]
        if len(signs) >= 3:
            left = sum(1 for b in signs if b > 0)
            print("  편향 부호: 좌 %d / 우 %d — 한쪽으로 몰리면 계통 편향(예약 39)"
                  % (left, len(signs) - left))

    for name, exc in failed:
        print("  \033[31m건너뜀\033[0m %s — %s" % (name, exc))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
