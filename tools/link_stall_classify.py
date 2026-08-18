#!/usr/bin/env python3
"""link_stall_classify.py — 예약 41-g 분류표 3판 (9행 전칭·상호배타).

정본 = ``docs/MASTER_PLAN.md`` §7 예약 41-g "원인군이 갈리는 방식".

이 파일이 그 표의 **유일한 구현**이다. 같은 표를 C++ harness 와 파이썬에 두 벌
쓰면 다음 세션에 둘이 갈라진다 (``AGENTS.md`` §3-10 ② — 열거를 검사기 안으로 옮긴다).
그래서 host harness 도 B4 판독도 이 파일을 부른다.

입력 = 호스트가 **실제로 수신한** 스트림(JSONL) + 수신 공백 하나의 [t0, t1].
출력 = 9행 중 정확히 하나.

🔴 상호배타는 주석이 아니라 구조로 보장한다 — 위에서부터 처음 걸리는 행에서
   즉시 반환한다. 전칭은 9행이 무조건 참인 catch-all 인 것으로 보장한다.

🔴 이 도구가 하지 않는 것: 원인을 적는 것. ``UNDECIDABLE_*`` 는 실패가 아니라
   **분류 거부**다. 9행이 걸리면 그것은 데이터가 아니라 **계약의 결함 보고**다.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field
from typing import Iterable, Optional

# 계약 상수 — firmware/teensy_integrated_base_v1_4/link_stall_probe.h 와 같은 값.
PULSE_PERIOD_MS = 100
SYNC_AGE_LIMIT_MS = 60000
SYNC_AGE_NEVER = 0xFFFFFFFF

CODE_LOOP_INTERNAL = 0
CODE_BETWEEN_LOOPS = 1
CODE_PUBLISH_FAIL = 2

CODE_NAMES = {
    CODE_LOOP_INTERNAL: "LOOP_INTERNAL",
    CODE_BETWEEN_LOOPS: "BETWEEN_LOOPS",
    CODE_PUBLISH_FAIL: "PUBLISH_FAIL",
}

# ── 9행 ─────────────────────────────────────────────────────────────────────
ROWS = (
    (1, "UNDECIDABLE_INSTRUMENT", "판정 불능 — 계측부터 고친다"),
    (2, "MCU_RESET", "MCU reset — 이전 구간과 잇지 않는다"),
    (3, "RUN_ENDED", "시행 종료 — 원인 분류를 만들지 않는다"),
    (4, "LOOP_INTERNAL", "loop 안"),
    (5, "BETWEEN_LOOPS", "판 사이"),
    (6, "COMPOUND", "복합 — 2종 이상이면 하나로 강제하지 않는다"),
    (7, "PUBLISH_LAYER", "발행 층"),
    (8, "HOST_AFTER", "host/agent/recorder 이후"),
    (9, "UNDECIDABLE_UNCOVERED", "판정 불능 — 표가 못 덮는다"),
)
ROW_BY_NAME = {name: (num, why) for num, name, why in ROWS}


@dataclass
class Verdict:
    row: int
    name: str
    why: str
    detail: str = ""

    def __str__(self) -> str:
        tail = f" ({self.detail})" if self.detail else ""
        return f"{self.row}행 {self.name} — {self.why}{tail}"


@dataclass
class Stream:
    pulses: list = field(default_factory=list)
    events: list = field(default_factory=list)

    @classmethod
    def load(cls, path: str) -> "Stream":
        s = cls()
        with open(path, "r", encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                rec = json.loads(line)
                if rec.get("t") == "pulse":
                    s.pulses.append(rec)
                elif rec.get("t") == "event":
                    s.events.append(rec)
                else:
                    raise ValueError(f"알 수 없는 레코드 종류: {rec.get('t')!r}")
        s.pulses.sort(key=lambda r: r["epoch_ms"])
        s.events.sort(key=lambda r: r["first_epoch_ms"])
        return s


def observed_pulse_period_ms(stream: Stream) -> float:
    """스트림에서 **실측** pulse 주기를 뽑는다.

    🔴 명목 100ms 를 그대로 쓰면 안 된다. 펌웨어는 `nowMs - lastPulseMs >= 100` 으로
    발화하는데 loop 이 1~7ms 주기라 실제 주기는 늘 100ms 를 조금 넘긴다 —
    08-18 실기 실측 **약 103.9ms**(9.67~9.78Hz). 명목으로 대조하면 46.6초 구간에서
    기대 466 · 실제 449 로 17이 벌어져 **멀쩡한 구간이 판정 불능**으로 떨어진다.

    ⚠ 펌웨어를 100ms 에 맞추려고 `lastPulseMs += 100`(catch-up)으로 고치면 안 된다.
      그러면 loop 이 300ms 섰다 깨어날 때도 +3 이 되어 *"loop 이 섰다"* 와
      *"host 가 잃었다"* 가 같은 값이 된다 — 판정 자체가 무너진다. 호스트가 잰다.

    유실이 있어도 된다: 이웃한 두 표본의 `epoch` 차를 `sample_seq` 차로 나누면
    표본당 주기가 나온다. 중앙값을 쓰므로 튀는 구간에 안 끌려간다.
    """
    samples = []
    for prev, cur in zip(stream.pulses, stream.pulses[1:]):
        if prev["boot_id"] != cur["boot_id"]:
            continue
        ticks = cur["sample_seq"] - prev["sample_seq"]
        elapsed = cur["epoch_ms"] - prev["epoch_ms"]
        # 정지 구간(ticks 가 시간에 비해 턱없이 적다)은 주기 추정에서 뺀다 —
        # 그 구간을 넣으면 정지가 주기를 늘려 자기 자신을 정상으로 보이게 만든다.
        if ticks <= 0 or elapsed <= 0 or ticks > 64:
            continue
        per = elapsed / ticks
        if per < PULSE_PERIOD_MS * 0.5 or per > PULSE_PERIOD_MS * 2.0:
            continue
        samples.append(per)

    if len(samples) < 5:
        return float(PULSE_PERIOD_MS)
    samples.sort()
    mid = len(samples) // 2
    if len(samples) % 2:
        return samples[mid]
    return (samples[mid - 1] + samples[mid]) / 2.0


def _last_pulse_before(stream: Stream, t0: int) -> Optional[dict]:
    hits = [p for p in stream.pulses if p["epoch_ms"] <= t0]
    return hits[-1] if hits else None


def _first_pulse_after(stream: Stream, t1: int) -> Optional[dict]:
    for p in stream.pulses:
        if p["epoch_ms"] >= t1:
            return p
    return None


def _events_in_window(stream: Stream, t0: int, t1: int) -> list:
    """공백 [t0, t1] 과 겹치는 사건. 접힌 칸은 [first, last] 구간을 가진다."""
    return [
        e
        for e in stream.events
        if e["last_epoch_ms"] >= t0 and e["first_epoch_ms"] <= t1
    ]


def _instrument_faults(
    stream: Stream, t0: int, t1: int, before: Optional[dict], after: Optional[dict]
) -> list:
    """1행 — 계측 자체를 못 믿는 사유 전량. 하나라도 있으면 판정 불능이다."""
    faults = []

    # pulse 무표본 — 기준이 될 표본이 아예 없으면 증가량을 계산할 수 없다.
    # ⚠ "공백 뒤에 복귀 pulse 가 없다" 는 3행(시행 종료)이지 여기가 아니다.
    if before is None:
        faults.append("pulse 무표본(공백 이전 기준 표본 없음)")

    for label, p in (("복귀", after), ("직전", before)):
        if p is None:
            continue
        if not p["sync_ok"]:
            faults.append(f"{label} pulse sync_ok=false")
        age = p["sync_age_ms"]
        if age == SYNC_AGE_NEVER:
            faults.append(f"{label} pulse 동기 이력 없음")
        elif age > SYNC_AGE_LIMIT_MS:
            faults.append(f"{label} pulse sync_age_ms={age} 초과")

    if after is not None and after["evt_dropped_delta"] > 0:
        faults.append(f"복귀 pulse evt_dropped_delta={after['evt_dropped_delta']}")

    if before is not None and after is not None:
        # sample_seq 정지 / 역행. boot_id 가 바뀐 경우는 2행이 가져가므로 여기서 뺀다.
        if after["boot_id"] == before["boot_id"]:
            inc = after["sample_seq"] - before["sample_seq"]
            if inc == 0:
                faults.append("sample_seq 정지")
            elif inc < 0:
                faults.append(f"sample_seq 역행({inc})")

    # 사건 publisher 자체의 publish 실패 — evt_seq 회계로만 보인다.
    # MCU 가 센 발생 건수에서 버린 건수를 빼면 host 가 받았어야 할 건수다.
    #
    # 🔴 구간이 아니라 **스트림 전체**로 센다. 구간으로 자르면 경계에서 갈린다:
    #    LOOP_INTERNAL 은 loop **끝**에 기록되는데 그 판의 pulse 는 그 앞에서
    #    만들어지므로, 사건과 pulse 가 같은 ms 를 갖고도 evt_seq 에는 아직 안 실린다.
    #    그 한 칸 어긋남이 멀쩡한 구간을 판정 불능으로 만든다(실측: harness ⓐ).
    #
    # 🔴 한 방향으로만 본다. received < expected = MCU 가 센 사건이 host 에 안 왔다
    #    → 계측 결함. 반대(received > expected)는 마지막 pulse **뒤**에 난 사건이
    #    정상적으로 있을 수 있으므로 결함이 아니다.
    #
    # ⚠ 공개하는 한계: 이 검사는 사건 유실을 **스트림 단위**로 잡으므로, 어느 구간에서
    #    잃었는지는 못 가른다. 한 번 걸리면 그 시행 전체가 사건 기반 판정에서 빠진다.
    # 🔴 **증가량**으로 센다. 절대값으로 비교하면 안 된다 — bag 은 펌웨어보다 늦게
    #    시작하므로 첫 표본의 `evt_seq` 가 이미 0 이 아니다. 08-18 실기에서 굽기 40분
    #    뒤에 기록을 시작해 첫 표본이 12 였고, 절대값 비교(47 vs 수신 35)가 멀쩡한
    #    시행을 "12건 유실" 로 판정했다. 실제 유실은 0 이었다.
    # ⚠ 같은 이유로 `evt_dropped_total` 도 증가량으로 뺀다.
    if len(stream.pulses) >= 2:
        first = stream.pulses[0]
        last = stream.pulses[-1]
        if first["boot_id"] == last["boot_id"]:
            expected_total = (last["evt_seq"] - first["evt_seq"]) - (
                last["evt_dropped_total"] - first["evt_dropped_total"]
            )
            received_total = sum(e["count"] for e in stream.events)
            if received_total < expected_total:
                faults.append(
                    f"사건 유실(MCU 증가 {expected_total} · 수신 {received_total})"
                )

    return faults


def classify(stream: Stream, t0: int, t1: int) -> Verdict:
    """수신 공백 하나를 9행 중 하나로 보낸다. **위에서부터** 본다."""
    before = _last_pulse_before(stream, t0)
    after = _first_pulse_after(stream, t1)

    # ── 1행 ────────────────────────────────────────────────────────────────
    # 🔴 맨 위인 것이 핵심이다. 계측이 못 미더운 구간을 원인 분류로 흘려보내면
    #    아래가 전부 오염된다.
    faults = _instrument_faults(stream, t0, t1, before, after)
    if faults:
        return _verdict("UNDECIDABLE_INSTRUMENT", " · ".join(faults))

    # ── 2행 ────────────────────────────────────────────────────────────────
    if after is not None and before is not None and after["boot_id"] != before["boot_id"]:
        return _verdict(
            "MCU_RESET", f"boot_id {before['boot_id']} → {after['boot_id']}"
        )

    # ── 3행 ────────────────────────────────────────────────────────────────
    if after is None:
        return _verdict("RUN_ENDED", "bag 끝까지 복귀 pulse 없음")

    # ── 4·5·7행 (단독) → 6행 (복합) ───────────────────────────────────────
    #
    # 🔴 08-18 개정 (사용자 결정). 구판 6행은 `LOOP_INTERNAL + BETWEEN_LOOPS` 만
    #    복합으로 봤다. 그러면 `{LOOP_INTERNAL, PUBLISH_FAIL}` 과
    #    `{BETWEEN_LOOPS, PUBLISH_FAIL}` 이 어느 줄에도 안 걸려 9행으로 떨어진다.
    #    08-17 의 7사건은 `publish_failures +70` 이 300ms 공백과 겹친 모양이라,
    #    그 조합이 실제로 나오면 계측이 다 옳게 기록해도 **표가 답을 못 낸다**.
    #    (재현 = host harness ⓧ. 계약 스스로 "9행이 걸리면 데이터가 아니라 계약의
    #     결함 보고"라고 썼고, 그 보고가 실제로 나왔다.)
    #
    #    개정: **단독 3행을 먼저 보고, 2종 이상이면 전부 복합**이다. 이러면 code
    #    조합 8가지(공집합 포함)가 전부 덮여 표가 전칭이 된다.
    # ⚠ 이 네 줄은 code 집합의 공간을 **분할**한다: 단독 셋(집합 상등) + 나머지 전부.
    #   그래서 넷의 순서를 바꿔도 결과가 같고, 공집합만 아래 8행으로 내려간다.
    #   구판이 무너진 이유는 순서가 아니라 6행이 **부분집합 검사**
    #   (`LI in codes and BL in codes`)여서 분할이 아니었기 때문이다 — 분할이
    #   아니면 어떤 조합은 어느 줄에도 안 걸린다. 그게 {LI,PF}·{BL,PF} 였다.
    # ⚠ 여기까지 내려왔다는 것은 codes 가 단독도 공집합도 아니라는 뜻이라 이 조건은
    #   이 자리에서 항상 참이다. 그래도 명시해 두는 이유: 위 셋을 나중에 건드렸을 때
    #   이 줄이 **조용히 catch-all 이 되는 것**을 읽는 사람이 알아채게 하려는 것이다.
    events = _events_in_window(stream, t0, t1)
    codes = {e["code"] for e in events}

    if codes == {CODE_LOOP_INTERNAL}:
        return _verdict("LOOP_INTERNAL", _codes_detail(events))
    if codes == {CODE_BETWEEN_LOOPS}:
        return _verdict("BETWEEN_LOOPS", _codes_detail(events))
    if codes == {CODE_PUBLISH_FAIL}:
        return _verdict("PUBLISH_LAYER", _codes_detail(events))
    if len(codes) >= 2:
        return _verdict("COMPOUND", _codes_detail(events))

    # ── 8행 ────────────────────────────────────────────────────────────────
    if not codes:
        elapsed = after["epoch_ms"] - before["epoch_ms"]
        period = observed_pulse_period_ms(stream)
        expected = elapsed / period
        inc = after["sample_seq"] - before["sample_seq"]
        # 🔴 판정 단위는 "pulse 1개가 왔는가" 가 아니라 "복귀 후 첫 pulse 의
        #    seq 증가량" 이다. BEST_EFFORT 로 k 개를 잃어도 증가량은 살아 돌아온다.
        #
        # 허용폭 = max(1, 2%). 1 은 tick 위상이 공백 경계에 걸치는 몫이고, 2% 는 주기
        # **지터**가 누적되는 몫이다.
        # 🔴 주기 **편향**은 여기서 덮지 않는다 — 그건 위 observed_pulse_period_ms 가
        #    실측으로 없앤다. 편향까지 덮을 만큼 폭을 넓히면(초판 5%) 추정기가 있으나
        #    마나가 되고, 실제로 그렇게 뒀더니 "명목 100ms 로 되돌리는" 변이가 회귀를
        #    그대로 통과했다(08-18 실측). 폭은 추정기가 못 없애는 것만 덮는다.
        #    지터 근거: 표본당 ±2ms 가 n 회 누적돼도 상대오차는 1/√n 로 줄어
        #    449 tick 에서 0.1% 수준이다. 2% 는 그 20배 여유다.
        # 🔴 이 폭이 "loop 이 섰다"를 삼키지 않는다: 300ms 공백이면 기대 약 2.9 에
        #    허용 1 이라, 정지 시 증가량 1 은 |1-2.9|=1.9 로 **밖에 남는다**.
        tolerance = max(1.0, 0.02 * expected)
        detail = (
            f"sample_seq +{inc} (경과 {elapsed}ms · 실측주기 {period:.1f}ms · "
            f"기대 {expected:.1f} ± {tolerance:.1f})"
        )
        if abs(inc - expected) <= tolerance:
            return _verdict("HOST_AFTER", detail)
        return _verdict(
            "UNDECIDABLE_UNCOVERED", "사건 0건인데 " + detail + " 과 안 맞는다")

    # ── 9행 ────────────────────────────────────────────────────────────────
    # 🔴 여기 걸리면 데이터가 아니라 **계약의 결함 보고**다. 가장 가까운 줄로
    #    반올림하지 않는다 — 그게 §79.3 이 지적한 "복합이 단일로 숨는" 자리다.
    #
    # ⚠ 08-18 개정으로 **code 조합**은 8/8 이 위에서 다 덮인다. 그래서 이 줄에
    #   남은 도달 경로는 사건이 0건인데 `sample_seq` 증가량이 경과 시간과 안 맞는
    #   경우뿐이다(위 8행 else). 예: 공백 중 TIME_SYNC 가 offset 을 옮겨 epoch 경과와
    #   tick 수가 어긋난 구간. 🔴 그때도 "가장 그럴듯한 원인"을 적지 않는다.
    return _verdict(
        "UNDECIDABLE_UNCOVERED",
        "표가 못 덮는 조합: " + "+".join(sorted(CODE_NAMES[c] for c in codes)),
    )


def _codes_detail(events: Iterable[dict]) -> str:
    parts = []
    for e in events:
        parts.append(
            f"{CODE_NAMES[e['code']]}"
            f"(slot={e['slot']} burst={e['burst_id']} count={e['count']})"
        )
    return " · ".join(parts)


def _verdict(name: str, detail: str = "") -> Verdict:
    num, why = ROW_BY_NAME[name]
    return Verdict(num, name, why, detail)


def group_events(stream: Stream) -> dict:
    """접는 키 (code, slot, phase, burst_id) 로 묶는다.

    ring 이 중간에 배출되면 같은 burst 가 여러 칸으로 나뉘어 나온다 — 그건 정상이다.
    §79.3 이 요구한 "7개로 복원" 은 **이 묶음의 개수**로 판정한다.
    """
    groups: dict = {}
    for e in stream.events:
        key = (e["code"], e["slot"], e["phase"], e["burst_id"])
        g = groups.get(key)
        if g is None:
            groups[key] = {
                "code": e["code"],
                "slot": e["slot"],
                "phase": e["phase"],
                "burst_id": e["burst_id"],
                "first_epoch_ms": e["first_epoch_ms"],
                "last_epoch_ms": e["last_epoch_ms"],
                "count": e["count"],
                "exec_us_max": e["exec_us_max"],
                "idle_us_max": e["idle_us_max"],
            }
            continue
        g["first_epoch_ms"] = min(g["first_epoch_ms"], e["first_epoch_ms"])
        g["last_epoch_ms"] = max(g["last_epoch_ms"], e["last_epoch_ms"])
        g["count"] += e["count"]
        g["exec_us_max"] = max(g["exec_us_max"], e["exec_us_max"])
        g["idle_us_max"] = max(g["idle_us_max"], e["idle_us_max"])
    return groups


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="예약 41-g 수신 공백 분류 (3판 9행)")
    ap.add_argument("stream", help="수신 스트림 JSONL")
    ap.add_argument("--gap-start", type=int, required=True, help="공백 시작 epoch_ms")
    ap.add_argument("--gap-end", type=int, required=True, help="공백 끝 epoch_ms")
    ap.add_argument("--expect", help="기대 분류 이름. 다르면 rc=1")
    ap.add_argument(
        "--expect-groups",
        type=int,
        help="접는 키로 묶었을 때 기대 묶음 수. 다르면 rc=1",
    )
    args = ap.parse_args(argv)

    if args.expect is not None and args.expect not in ROW_BY_NAME:
        print(f"FAIL 알 수 없는 기대 분류: {args.expect}", file=sys.stderr)
        return 2

    try:
        stream = Stream.load(args.stream)
    except (OSError, ValueError) as exc:
        print(f"FAIL 스트림을 읽지 못했다: {exc}", file=sys.stderr)
        return 2

    verdict = classify(stream, args.gap_start, args.gap_end)
    groups = group_events(stream)

    rc = 0
    print(f"판정: {verdict}")
    print(f"사건 묶음: {len(groups)}개")

    if args.expect is not None and verdict.name != args.expect:
        print(f"FAIL 기대 {args.expect} · 실제 {verdict.name}", file=sys.stderr)
        rc = 1
    if args.expect_groups is not None and len(groups) != args.expect_groups:
        print(
            f"FAIL 묶음 기대 {args.expect_groups} · 실제 {len(groups)}",
            file=sys.stderr,
        )
        rc = 1
    return rc


if __name__ == "__main__":
    sys.exit(main())
