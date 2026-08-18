#!/usr/bin/env python3
"""예약 41-g 분류표 3판(9행)의 전칭·상호배타·경계 회귀.

왜 이 시험이 따로 있나 — host harness(`link_stall_host_test.sh`)는 **주입 10여 개**가
서로 다른 분류를 내는지 본다. 그건 표본이지 전칭이 아니다. 계약은 *"가능한 모든
사건 조합이 … 정확히 하나로 간다"* 라고 전칭으로 썼으므로, 조합 공간을 **전수로
돌려** 그 전칭을 확인한다 (``AGENTS.md`` §3-10 ②: 열거를 검사기 안으로 옮긴다).

🔴 경계는 양쪽을 잠근다 (§3-10 ⑤). 개수·시간 경계마다 넘는 쪽과 못 넘는 쪽을
   같이 박는다 — 부등호 하나로 끝냈다면 반대 방향을 아직 안 물은 것이다.
"""

import itertools
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import link_stall_classify as lsc  # noqa: E402


T0 = 2000
T1 = 2300


def pulse(epoch_ms, **kw):
    rec = {
        "t": "pulse",
        "boot_id": 1,
        "sample_seq": 0,
        "epoch_ms": epoch_ms,
        "sync_ok": True,
        "sync_age_ms": 1000,
        "evt_seq": 0,
        "evt_dropped_total": 0,
        "evt_dropped_delta": 0,
        "pulse_fail": 0,
    }
    rec.update(kw)
    return rec


def event(code, epoch_ms=T0 + 10, **kw):
    rec = {
        "t": "event",
        "code": code,
        "phase": 6,
        "slot": 255,
        "burst_id": 0,
        "first_epoch_ms": epoch_ms,
        "last_epoch_ms": epoch_ms,
        "exec_us_max": 0,
        "idle_us_max": 0,
        "count": 1,
    }
    rec.update(kw)
    return rec


def make(records):
    stream = lsc.Stream()
    for rec in records:
        if rec["t"] == "pulse":
            stream.pulses.append(rec)
        else:
            stream.events.append(rec)
    stream.pulses.sort(key=lambda r: r["epoch_ms"])
    stream.events.sort(key=lambda r: r["first_epoch_ms"])
    return stream


def clean(events=(), before=None, after=None):
    """계측이 건강한 스트림. evt_seq 회계는 사건 수에 맞춘다."""
    total = sum(e["count"] for e in events)
    b = pulse(T0 - 100, sample_seq=10, evt_seq=0)
    a = pulse(T1 + 100, sample_seq=14, evt_seq=total)
    if before:
        b.update(before)
    if after:
        a.update(after)
    return make([b, a] + list(events))


class TotalityTest(unittest.TestCase):
    """9행이 전칭인가 — 조합 공간 전수."""

    def test_every_code_subset_lands_in_exactly_one_row(self):
        codes = (
            lsc.CODE_LOOP_INTERNAL,
            lsc.CODE_BETWEEN_LOOPS,
            lsc.CODE_PUBLISH_FAIL,
        )
        seen = {}
        for size in range(len(codes) + 1):
            for subset in itertools.combinations(codes, size):
                events = [event(c) for c in subset]
                verdict = lsc.classify(clean(events), T0, T1)
                self.assertIn(verdict.name, lsc.ROW_BY_NAME, subset)
                seen[subset] = verdict.name

        # 08-18 개정 기대값 (사용자 결정): 단독 3행이 먼저, 2종 이상은 전부 복합.
        # 🔴 구판은 (0,2)·(1,2) 를 9행으로 흘렸다. 08-17 의 7사건이 바로 그 모양이라
        #    계측이 옳아도 표가 답을 못 냈다.
        self.assertEqual(seen[()], "HOST_AFTER")
        self.assertEqual(seen[(0,)], "LOOP_INTERNAL")
        self.assertEqual(seen[(1,)], "BETWEEN_LOOPS")
        self.assertEqual(seen[(2,)], "PUBLISH_LAYER")
        self.assertEqual(seen[(0, 1)], "COMPOUND")
        self.assertEqual(seen[(0, 2)], "COMPOUND")
        self.assertEqual(seen[(1, 2)], "COMPOUND")
        self.assertEqual(seen[(0, 1, 2)], "COMPOUND")

    def test_no_code_combination_falls_through_to_row_nine(self):
        """🔴 code 조합에 대해 표가 전칭인가 — 못 덮는 조합은 **0개**여야 한다.

        양방향 계약이다 (AGENTS §3-10 ⑤): 구멍이 다시 생기면 개수가 0 을 넘어
        깨지고, 반대로 9행을 아예 없애 버려도 아래 도달성 시험이 깨진다.
        """
        codes = (
            lsc.CODE_LOOP_INTERNAL,
            lsc.CODE_BETWEEN_LOOPS,
            lsc.CODE_PUBLISH_FAIL,
        )
        uncovered = []
        for size in range(1, len(codes) + 1):
            for subset in itertools.combinations(codes, size):
                verdict = lsc.classify(clean([event(c) for c in subset]), T0, T1)
                if verdict.name == "UNDECIDABLE_UNCOVERED":
                    uncovered.append(subset)
        self.assertEqual(uncovered, [])

    def test_repeated_same_code_stays_singleton(self):
        """복합을 정하는 것은 **code 종류 수**지 사건 건수가 아니다.

        08-17 은 한 공백에 실패가 수십 건인 데이터다. 건수로 복합을 정하면 그게
        전부 "복합"으로 뭉개져 4·5·7행이 실질적으로 죽는다.
        """
        self.assertEqual(
            lsc.classify(clean([event(0), event(0, T0 + 20)]), T0, T1).name,
            "LOOP_INTERNAL")
        self.assertEqual(
            lsc.classify(clean([event(2), event(2, T0 + 20)]), T0, T1).name,
            "PUBLISH_LAYER")

    def test_every_row_name_is_reachable(self):
        """9행 중 도달 불가능한 행이 없어야 한다 — 죽은 줄은 계약의 거짓말이다."""
        reached = set()
        reached.add(lsc.classify(clean([event(0)]), T0, T1).name)
        reached.add(lsc.classify(clean([event(1)]), T0, T1).name)
        reached.add(lsc.classify(clean([event(2)]), T0, T1).name)
        reached.add(lsc.classify(clean([event(0), event(1)]), T0, T1).name)
        reached.add(lsc.classify(clean(), T0, T1).name)
        # 🔴 9행의 남은 도달 경로: 사건 0건인데 sample_seq 증가량이 경과와 안 맞는다.
        #    (공백 중 TIME_SYNC 가 offset 을 옮긴 구간 등)
        reached.add(lsc.classify(
            make([pulse(T0, sample_seq=10), pulse(T0 + 500, sample_seq=10)
                  | {"sample_seq": 30}]), T0, T0 + 500).name)
        reached.add(
            lsc.classify(clean(after={"sync_ok": False}), T0, T1).name)
        reached.add(
            lsc.classify(clean(after={"boot_id": 2}), T0, T1).name)
        reached.add(lsc.classify(make([pulse(T0 - 100)]), T0, T1).name)
        self.assertEqual(reached, {name for _, name, _ in lsc.ROWS})


class PrecedenceTest(unittest.TestCase):
    """위에서부터 본다 — 위 행이 아래를 반드시 이긴다."""

    def test_instrument_fault_beats_every_cause(self):
        for events in ([event(0)], [event(1)], [event(2)],
                       [event(0), event(1)], []):
            verdict = lsc.classify(
                clean(events, after={"evt_dropped_delta": 1}), T0, T1)
            self.assertEqual(verdict.name, "UNDECIDABLE_INSTRUMENT", events)

    def test_instrument_fault_beats_reset(self):
        verdict = lsc.classify(
            clean(after={"boot_id": 2, "sync_ok": False}), T0, T1)
        self.assertEqual(verdict.name, "UNDECIDABLE_INSTRUMENT")

    def test_reset_beats_cause(self):
        # reset 이면 sample_seq 가 0 부터라 그 자체로는 회계가 안 맞을 수 있다.
        verdict = lsc.classify(
            clean([event(0)], after={"boot_id": 2, "sample_seq": 3}), T0, T1)
        self.assertEqual(verdict.name, "MCU_RESET")

    def test_run_ended_beats_cause(self):
        stream = make([pulse(T0 - 100, sample_seq=10, evt_seq=1), event(0)])
        self.assertEqual(lsc.classify(stream, T0, T1).name, "RUN_ENDED")

    def test_no_baseline_pulse_is_instrument_fault_not_run_ended(self):
        """🔴 '기준 표본이 없다'와 '복귀 표본이 없다'는 다른 사실이다."""
        stream = make([pulse(T1 + 100, sample_seq=3)])
        self.assertEqual(
            lsc.classify(stream, T0, T1).name, "UNDECIDABLE_INSTRUMENT")


class BoundaryTest(unittest.TestCase):
    """🔴 경계는 양쪽을 잠근다 (AGENTS §3-10 ⑤)."""

    def test_sync_age_limit_both_directions(self):
        ok = clean(after={"sync_age_ms": lsc.SYNC_AGE_LIMIT_MS})
        self.assertEqual(lsc.classify(ok, T0, T1).name, "HOST_AFTER")
        bad = clean(after={"sync_age_ms": lsc.SYNC_AGE_LIMIT_MS + 1})
        self.assertEqual(
            lsc.classify(bad, T0, T1).name, "UNDECIDABLE_INSTRUMENT")

    def test_sync_never_sentinel_is_fault(self):
        bad = clean(after={"sync_age_ms": lsc.SYNC_AGE_NEVER})
        self.assertEqual(
            lsc.classify(bad, T0, T1).name, "UNDECIDABLE_INSTRUMENT")

    def test_dropped_delta_both_directions(self):
        ok = clean(after={"evt_dropped_delta": 0})
        self.assertEqual(lsc.classify(ok, T0, T1).name, "HOST_AFTER")
        bad = clean(after={"evt_dropped_delta": 1})
        self.assertEqual(
            lsc.classify(bad, T0, T1).name, "UNDECIDABLE_INSTRUMENT")

    def test_sample_seq_increment_tolerance_both_edges(self):
        # 경과 500ms → 기대 증가량 5. ±1 은 tick 위상 몫이라 허용한다.
        b = {"sample_seq": 10, "epoch_ms": T0}
        for inc, want in ((4, "HOST_AFTER"), (5, "HOST_AFTER"),
                          (6, "HOST_AFTER"), (3, "UNDECIDABLE_UNCOVERED"),
                          (7, "UNDECIDABLE_UNCOVERED")):
            stream = make([
                pulse(T0, sample_seq=10),
                pulse(T0 + 500, sample_seq=10 + inc),
            ])
            self.assertEqual(
                lsc.classify(stream, T0, T0 + 500).name, want, (inc, b))

    def test_stalled_loop_does_not_read_as_host_after(self):
        """🔴 이게 계약의 요점이다 — loop 이 섰으면 증가량이 1 이라 8행에 안 걸린다."""
        stream = make([
            pulse(T0, sample_seq=10),
            pulse(T0 + 300, sample_seq=11),
        ])
        self.assertNotEqual(
            lsc.classify(stream, T0, T0 + 300).name, "HOST_AFTER")

    def test_sample_seq_frozen_and_rollback_are_faults(self):
        frozen = make([pulse(T0, sample_seq=10), pulse(T1, sample_seq=10)])
        self.assertEqual(
            lsc.classify(frozen, T0, T1).name, "UNDECIDABLE_INSTRUMENT")
        rollback = make([pulse(T0, sample_seq=10), pulse(T1, sample_seq=4)])
        self.assertEqual(
            lsc.classify(rollback, T0, T1).name, "UNDECIDABLE_INSTRUMENT")

    def test_event_loss_is_fault_but_surplus_is_not(self):
        # MCU 가 3건을 셌는데 2건만 왔다 → 계측 결함.
        lost = make([
            pulse(T0 - 100, sample_seq=10, evt_seq=0),
            pulse(T1 + 100, sample_seq=14, evt_seq=3),
            event(2, T0 + 10), event(2, T0 + 20),
        ])
        self.assertEqual(
            lsc.classify(lost, T0, T1).name, "UNDECIDABLE_INSTRUMENT")
        # 마지막 pulse 뒤에 난 사건이 더 있는 것은 정상이다.
        surplus = make([
            pulse(T0 - 100, sample_seq=10, evt_seq=0),
            pulse(T1 + 100, sample_seq=14, evt_seq=1),
            event(2, T0 + 10), event(2, T1 + 200),
        ])
        self.assertEqual(lsc.classify(surplus, T0, T1).name, "PUBLISH_LAYER")

    def test_bag_starting_mid_run_is_not_read_as_loss(self):
        """🔴 08-18 실기 재현 — bag 은 펌웨어보다 늦게 시작한다.

        굽기 40분 뒤에 기록을 시작해 첫 표본의 `evt_seq` 가 이미 **12** 였다.
        절대값으로 비교하면(끝 47 vs 수신 35) 멀쩡한 시행이 "12건 유실"로 판정되고,
        그 시행의 **모든** 수신 공백이 1행 판정 불능으로 오염된다. 실제 유실은 0 이었다.
        """
        stream = make([
            pulse(T0 - 100, sample_seq=10, evt_seq=12),
            pulse(T1 + 100, sample_seq=14, evt_seq=12 + 11),
        ] + [event(2, T0 + i) for i in range(11)])
        verdict = lsc.classify(stream, T0, T1)
        self.assertEqual(verdict.name, "PUBLISH_LAYER", verdict.detail)

    def test_real_loss_after_offset_start_is_still_caught(self):
        """양방향 — 시작 offset 을 빼도 **진짜** 유실은 여전히 잡아야 한다."""
        stream = make([
            pulse(T0 - 100, sample_seq=10, evt_seq=12),
            pulse(T1 + 100, sample_seq=14, evt_seq=12 + 11),
        ] + [event(2, T0 + i) for i in range(9)])
        self.assertEqual(
            lsc.classify(stream, T0, T1).name, "UNDECIDABLE_INSTRUMENT")

    def test_dropped_events_are_not_counted_as_loss(self):
        """버려진 사건은 host 에 올 수 없다 — 그걸 유실로 또 세면 이중 계상이다."""
        stream = make([
            pulse(T0 - 100, sample_seq=10, evt_seq=0),
            pulse(T1 + 100, sample_seq=14, evt_seq=5,
                  evt_dropped_total=3, evt_dropped_delta=0),
            event(2, T0 + 10), event(2, T0 + 20),
        ])
        self.assertEqual(lsc.classify(stream, T0, T1).name, "PUBLISH_LAYER")


class FieldPeriodTest(unittest.TestCase):
    """🔴 08-18 실기 재현 — pulse 주기는 명목 100ms 가 아니라 약 104ms 다.

    보드 실측 두 표본:
        P,11529985,407,1787024132328,1,11892,0,0,0,0
        P,11529985,856,1787024178990,1,28554,0,0,0,0
    449 tick / 46,662 ms = **103.92 ms**  (`ros2 topic hz` 9.67~9.78Hz 와 일치)

    명목 100ms 로 대조하면 기대 466 · 실제 449 라 17 이 벌어져 **아무 일도 없던
    46.6초 구간이 판정 불능**이 된다. 원인은 펌웨어가 `nowMs - lastPulseMs >= 100`
    으로 발화하는데 loop 이 1~7ms 주기라 늘 100ms 를 조금 넘겨 발화하기 때문이다.
    """

    PERIOD = 103.92

    def _field_stream(self, ticks=500, start_seq=407, start_epoch=1787024132328):
        pulses = []
        for i in range(ticks):
            pulses.append(pulse(
                int(start_epoch + round(i * self.PERIOD)),
                boot_id=11529985,
                sample_seq=start_seq + i,
                sync_age_ms=(i * 104) % 30000,
            ))
        return make(pulses)

    def test_observed_period_matches_field_measurement(self):
        got = lsc.observed_pulse_period_ms(self._field_stream())
        self.assertAlmostEqual(got, self.PERIOD, delta=0.6)

    def test_long_quiet_span_is_host_after_not_undecidable(self):
        """🔴 재현 후 수정 확인 — 46.6초 조용한 구간이 8행으로 간다."""
        stream = self._field_stream()
        t0 = stream.pulses[0]["epoch_ms"]
        t1 = stream.pulses[449]["epoch_ms"]
        verdict = lsc.classify(stream, t0, t1)
        self.assertEqual(verdict.name, "HOST_AFTER", verdict.detail)

    def test_stall_still_detected_at_field_period(self):
        """🔴 허용폭이 "loop 이 섰다"를 삼키지 않는가 — 실측 주기에서도 확인한다."""
        stream = self._field_stream(ticks=200)
        last = stream.pulses[-1]
        # 300ms 가 흘렀는데 tick 은 1 번밖에 안 올랐다 = loop 이 섰다.
        stalled = pulse(last["epoch_ms"] + 300, boot_id=11529985,
                        sample_seq=last["sample_seq"] + 1)
        stream.pulses.append(stalled)
        verdict = lsc.classify(stream, last["epoch_ms"], stalled["epoch_ms"])
        self.assertNotEqual(verdict.name, "HOST_AFTER", verdict.detail)

    def test_stalled_span_is_excluded_from_period_estimate(self):
        """정지 구간이 주기 추정을 늘려 자기 자신을 정상으로 만들면 안 된다."""
        stream = self._field_stream(ticks=50)
        last = stream.pulses[-1]
        stream.pulses.append(pulse(last["epoch_ms"] + 9000, boot_id=11529985,
                                   sample_seq=last["sample_seq"] + 1))
        self.assertAlmostEqual(
            lsc.observed_pulse_period_ms(stream), self.PERIOD, delta=0.6)


class GroupingTest(unittest.TestCase):
    """§79.3 — 접어도 7 burst 가 7개로 복원되는가."""

    def test_seven_bursts_restore_to_seven_groups(self):
        events = []
        for burst in range(7):
            for i in range(10):
                events.append(event(2, T0 + burst * 100 + i, slot=0,
                                    burst_id=burst * 2))
        groups = lsc.group_events(make(events))
        self.assertEqual(len(groups), 7)
        for group in groups.values():
            self.assertEqual(group["count"], 10)
            self.assertEqual(group["slot"], 0)
            self.assertLess(group["first_epoch_ms"], group["last_epoch_ms"])

    def test_slot_is_part_of_the_key(self):
        """🔴 slot 을 키에서 빼면 두 토픽의 공백이 한 칸으로 합쳐진다."""
        events = [event(2, T0 + i, slot=i % 2) for i in range(12)]
        self.assertEqual(len(lsc.group_events(make(events))), 2)

    def test_phase_is_part_of_the_key(self):
        events = [event(2, T0 + i, slot=0, phase=i % 3) for i in range(12)]
        self.assertEqual(len(lsc.group_events(make(events))), 3)


class CliTest(unittest.TestCase):
    def test_expect_mismatch_returns_one(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "s.jsonl")
            with open(path, "w", encoding="utf-8") as fh:
                for rec in [pulse(T0 - 100, sample_seq=10),
                            pulse(T1 + 100, sample_seq=14),
                            event(0)]:
                    fh.write(json.dumps(rec) + "\n")
            argv = [path, "--gap-start", str(T0), "--gap-end", str(T1)]
            self.assertEqual(lsc.main(argv + ["--expect", "LOOP_INTERNAL"]), 0)
            self.assertEqual(lsc.main(argv + ["--expect", "HOST_AFTER"]), 1)
            self.assertEqual(
                lsc.main(argv + ["--expect", "LOOP_INTERNAL",
                                 "--expect-groups", "2"]), 1)

    def test_unknown_expect_is_undecidable_not_pass(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "s.jsonl")
            open(path, "w", encoding="utf-8").close()
            self.assertEqual(
                lsc.main([path, "--gap-start", "0", "--gap-end", "1",
                          "--expect", "NOPE"]), 2)


if __name__ == "__main__":
    unittest.main()
