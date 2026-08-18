#!/usr/bin/env python3
"""rosbag2 → 예약 41-g 판독. `/firmware/pulse`·`/firmware/event` 를 뽑아 분류표에 먹인다.

사용:
    python3 tools/link_stall_bag_extract.py <bag 디렉터리> [옵션]
      --emit <파일>      수신 스트림을 JSONL 로 쓴다 (link_stall_classify.py 입력)
      --gap-topic <토픽> 수신 공백을 찾을 토픽 (기본 /odom, 여러 번 줄 수 있다)
      --gap-ms <ms>      이 값을 넘는 수신 공백만 본다 (기본 100)

왜 이 도구가 필요한가
---------------------
`link_stall_classify.py` 는 9 행 분류표의 유일한 구현이고 입력이 JSONL 이다. 실기 증거는
rosbag2 라 그 사이를 잇는 것이 없으면 **bag 을 받아도 판독을 못 한다.** 이 파일이 그 다리다.
분류 규칙은 여기에 **한 줄도 두지 않는다** — 두면 표가 두 곳으로 갈라진다
(`AGENTS.md` §3-10 ②).

두 시계가 왜 비교 가능한가
--------------------------
- bag 수신시각 = Jetson 벽시계 (Unix epoch ns)
- pulse `epoch_ms` = `rmw_uros_epoch_millis()` = **agent 와 동기된 같은 Unix 벽시계**

그래서 수신 공백 [t0, t1] 을 그대로 `epoch_ms` 축에 놓을 수 있다. 🔴 다만 그것을
가정으로 두지 않는다 — pulse 마다 (수신시각 - epoch_ms) 를 재서 **중앙값 offset 과
산포를 인쇄**한다. offset 이 크면 그 시행은 시각축이 어긋난 것이므로 판독 전에 안다.

🔴 이 도구가 하지 않는 것
------------------------
- 원인을 적는 것. 분류는 `link_stall_classify.py` 가 하고, 이 파일은 재료만 만든다.
- 깨진 전문을 조용히 버리는 것. 파싱 실패는 **세어서 보고**한다 — 버리면 `evt_seq`
  회계가 맞아 보여 계측 결함이 정상으로 둔갑한다.
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import link_stall_classify as lsc  # noqa: E402

try:
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rosidl_runtime_py.utilities import get_message
except ImportError as exc:  # pragma: no cover
    sys.exit("ROS 2 모듈을 못 찾았다 (%s) — source /opt/ros/humble/setup.bash" % exc)

PULSE_TOPIC = "/firmware/pulse"
EVENT_TOPIC = "/firmware/event"
STORAGE = "sqlite3"


def parse_pulse(text):
    """`P,boot,seq,epoch,sync,age,evt,total,delta,fail` → dict. 실패하면 None."""
    parts = text.strip().split(",")
    if len(parts) != 10 or parts[0] != "P":
        return None
    try:
        return {
            "t": "pulse",
            "boot_id": int(parts[1]),
            "sample_seq": int(parts[2]),
            "epoch_ms": int(parts[3]),
            "sync_ok": parts[4] == "1",
            "sync_age_ms": int(parts[5]),
            "evt_seq": int(parts[6]),
            "evt_dropped_total": int(parts[7]),
            "evt_dropped_delta": int(parts[8]),
            "pulse_fail": int(parts[9]),
        }
    except ValueError:
        return None


def parse_event(text):
    """`E,code,phase,slot,burst,first,last,exec,idle,count` → dict. 실패하면 None."""
    parts = text.strip().split(",")
    if len(parts) != 10 or parts[0] != "E":
        return None
    try:
        return {
            "t": "event",
            "code": int(parts[1]),
            "phase": int(parts[2]),
            "slot": int(parts[3]),
            "burst_id": int(parts[4]),
            "first_epoch_ms": int(parts[5]),
            "last_epoch_ms": int(parts[6]),
            "exec_us_max": int(parts[7]),
            "idle_us_max": int(parts[8]),
            "count": int(parts[9]),
        }
    except ValueError:
        return None


def read_bag(bag, gap_topics):
    """bag 을 한 번 훑는다. (pulse, event, 깨진 전문, 토픽별 수신시각[ms])."""
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id=STORAGE), ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    wanted = {PULSE_TOPIC, EVENT_TOPIC} | set(gap_topics)
    missing = [t for t in wanted if t not in types]

    classes = {}
    for name in wanted & set(types):
        classes[name] = get_message(types[name])

    pulses, events, broken = [], [], []
    recv = {t: [] for t in gap_topics}

    while reader.has_next():
        name, data, t_ns = reader.read_next()
        if name not in classes:
            continue
        if name in recv:
            recv[name].append(t_ns / 1e6)
        if name in (PULSE_TOPIC, EVENT_TOPIC):
            msg = deserialize_message(data, classes[name])
            text = msg.data
            rec = parse_pulse(text) if name == PULSE_TOPIC else parse_event(text)
            if rec is None:
                broken.append((name, t_ns / 1e6, text[:80]))
                continue
            rec["_recv_ms"] = t_ns / 1e6
            (pulses if name == PULSE_TOPIC else events).append(rec)

    return pulses, events, broken, recv, missing


def clock_offset(pulses):
    """수신시각 - epoch_ms 의 중앙값과 산포. 시각축이 맞물리는지 본다."""
    if not pulses:
        return None, None
    deltas = sorted(p["_recv_ms"] - p["epoch_ms"] for p in pulses)
    mid = len(deltas) // 2
    median = deltas[mid] if len(deltas) % 2 else (deltas[mid - 1] + deltas[mid]) / 2.0
    return median, deltas[-1] - deltas[0]


def find_gaps(times_ms, limit_ms):
    """연속 수신 사이 간격이 limit 을 넘는 구간 [t0, t1] 목록."""
    gaps = []
    for prev, cur in zip(times_ms, times_ms[1:]):
        if cur - prev > limit_ms:
            gaps.append((prev, cur, cur - prev))
    return gaps


def main(argv=None):
    ap = argparse.ArgumentParser(description="예약 41-g bag 판독")
    ap.add_argument("bag")
    ap.add_argument("--emit")
    ap.add_argument("--gap-topic", action="append", default=None)
    ap.add_argument("--gap-ms", type=float, default=100.0)
    args = ap.parse_args(argv)

    gap_topics = args.gap_topic or ["/odom", "/imu/data"]

    pulses, events, broken, recv, missing = read_bag(args.bag, gap_topics)

    print("=" * 74)
    print("예약 41-g bag 판독 —", args.bag)
    print("=" * 74)
    if missing:
        print("🔴 bag 에 없는 토픽: %s" % ", ".join(missing))
        print("   → 없는 축은 판정에 쓰지 않는다.")
    print("  pulse %d개 · event %d개 · 깨진 전문 %d개"
          % (len(pulses), len(events), len(broken)))

    if broken:
        print("  🔴 깨진 전문이 있다 — 계측을 못 믿는 구간이 있다는 뜻이다:")
        for name, ms, text in broken[:5]:
            print("     %s  t=%.3f  %r" % (name, ms, text))

    if not pulses:
        print("\n🔴 생존 표본이 0건이다 — 41-g 판독 불가. 계측부터 고친다.")
        return 2

    median, spread = clock_offset(pulses)
    print("  시각축: 수신 - epoch 중앙값 %.1fms · 산포 %.1fms" % (median, spread))
    if abs(median) > 1000.0:
        print("  🔴 offset 이 1초를 넘는다 — 수신 공백을 epoch 축에 놓을 수 없다.")
        return 2

    period = lsc.observed_pulse_period_ms(_stream(pulses, events))
    print("  실측 pulse 주기 %.2fms (%.2fHz)" % (period, 1000.0 / period))

    last = pulses[-1]
    print("  MCU 계수: evt_seq=%d · dropped_total=%d · pulse_fail=%d · sync_ok=%s"
          % (last["evt_seq"], last["evt_dropped_total"], last["pulse_fail"],
             last["sync_ok"]))

    stream = _stream(pulses, events)
    groups = lsc.group_events(stream)
    print("  사건 묶음 %d개 (접는 키 = code·slot·phase·burst_id)" % len(groups))
    for key in sorted(groups, key=lambda k: groups[k]["first_epoch_ms"]):
        g = groups[key]
        print("     %-14s slot=%-3s phase=%s burst=%-4d count=%-4d "
              "%.3f~%.3f  exec_max=%dus idle_max=%dus"
              % (lsc.CODE_NAMES.get(g["code"], "?"), g["slot"], g["phase"],
                 g["burst_id"], g["count"],
                 g["first_epoch_ms"] / 1000.0, g["last_epoch_ms"] / 1000.0,
                 g["exec_us_max"], g["idle_us_max"]))

    if args.emit:
        with open(args.emit, "w", encoding="utf-8") as fh:
            for rec in sorted(pulses + events,
                              key=lambda r: r.get("epoch_ms", r.get("first_epoch_ms"))):
                out = {k: v for k, v in rec.items() if not k.startswith("_")}
                fh.write(json.dumps(out) + "\n")
        print("  스트림 → %s" % args.emit)

    rc = 0
    for topic in gap_topics:
        times = recv.get(topic) or []
        if not times:
            continue
        gaps = find_gaps(times, args.gap_ms)
        print("\n--- %s 수신 공백 %.0fms 초과: %d건 (표본 %d) ---"
              % (topic, args.gap_ms, len(gaps), len(times)))
        for t0, t1, width in gaps:
            # 🔴 공백 경계를 epoch 축으로 옮긴다. offset 은 위에서 실측한 값이다.
            e0 = int(round(t0 - median))
            e1 = int(round(t1 - median))
            verdict = lsc.classify(stream, e0, e1)
            print("  %.3f → %.3f  (%.1fms)  %s" % (t0 / 1000.0, t1 / 1000.0, width,
                                                   verdict))
            if verdict.name.startswith("UNDECIDABLE"):
                rc = 1
    return rc


def _stream(pulses, events):
    s = lsc.Stream()
    s.pulses = sorted(pulses, key=lambda r: r["epoch_ms"])
    s.events = sorted(events, key=lambda r: r["first_epoch_ms"])
    return s


if __name__ == "__main__":
    sys.exit(main())
