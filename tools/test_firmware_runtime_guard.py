#!/usr/bin/env python3
"""D0-FW 주기 정지 보완의 생산 코드·소비자 폐포 회귀.

08-17 bag에서는 `/odom`·`/imu/data`만이 아니라 같은 Teensy loop의 상태
토픽까지 동시에 비었다. 펌웨어의 주기 RELIABLE publish가 각각 최대 1초 ACK를
기다릴 수 있었고, 단일 loop 안에서 그 대기가 연쇄될 수 있었다. 1.6.1에서 8개를
모두 BEST_EFFORT로 바꿨지만 512-byte MTU보다 큰 `/odom`·`/firmware/info`가 아예
발행되지 않는 회귀를 실차에서 재현했다. 두 대형 표본만 RELIABLE+20ms 상한으로
되돌리고 나머지 6개는 BEST_EFFORT로 유지한다.

이 시험은 지목된 `/odom` 한 자리만 보지 않는다. 생산 `.ino`에서 주기 publisher
8개와 실제 publish 8개를 전수하고, 현장 도구 구독 12곳과 bag override 8곳까지
항목별로 대조한다. 개수의 증가·감소도 모두 실패시켜 새 자리가 검사 밖으로
생기는 것을 막는다(AGENTS §3-10).
"""

import os
import re
import shutil
import subprocess
import sys
import tempfile
import unittest


sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import firmware_constants as fc  # noqa: E402
import firmware_info_length_check as fil  # noqa: E402


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SKETCH_DIR = os.path.join(ROOT, "firmware", "teensy_integrated_base_v1_4")
INO_PATH = os.path.join(SKETCH_DIR, "teensy_integrated_base_v1_4.ino")
GUARD_PATH = os.path.join(SKETCH_DIR, "runtime_guard.h")
BAG_QOS_PATH = os.path.join(ROOT, "tools", "bag_qos_overrides.yaml")

PUBLISHERS = {
    "odomPublisher": "RUNTIME_PUBLISH_ODOM",
    "imuPublisher": "RUNTIME_PUBLISH_IMU",
    "imuYawPublisher": "RUNTIME_PUBLISH_IMU_YAW",
    "gyroBiasPublisher": "RUNTIME_PUBLISH_GYRO_BIAS",
    "estopStatePublisher": "RUNTIME_PUBLISH_ESTOP",
    "firmwareInfoPublisher": "RUNTIME_PUBLISH_FIRMWARE_INFO",
    "driveEnabledPublisher": "RUNTIME_PUBLISH_DRIVE_ENABLED",
    "driveDiagPublisher": "RUNTIME_PUBLISH_DRIVE_DIAG",
    # 🔴 08-18 1.6.3 예약 41-g 3판 — 생존 표본이 주기 publisher 를 8 → **9** 로
    # 만든다. 이 줄이 8 로 남으면 pulse 가 계측 밖이라는 뜻이라 FAIL 이다
    # (MASTER_PLAN §7 41-g "그 증가 자체를 기대값으로 회귀에 넣는다").
    "firmwarePulsePublisher": "RUNTIME_PUBLISH_PULSE",
}

# 사건 publisher 는 **비주기**라 위 전수에 넣지 않고 따로 센다. 넣으면 계약의
# "8 → 9" 가 10 이 된다. 발행 실패는 pulse 의 evt_seq 회계로 host 가 본다.
EVENT_PUBLISHERS = {
    "firmwareEventPublisher": "best_effort",
}

PUBLISHER_QOS = {
    "odomPublisher": "default",
    "imuPublisher": "best_effort",
    "imuYawPublisher": "best_effort",
    "gyroBiasPublisher": "best_effort",
    "estopStatePublisher": "best_effort",
    "firmwareInfoPublisher": "default",
    "driveEnabledPublisher": "best_effort",
    "driveDiagPublisher": "best_effort",
    # 🔴 BEST_EFFORT 여야 한다. RELIABLE 이면 감시 loop 에 ACK 대기를 다시 넣어
    # 1.6.1 → 1.6.2 회귀를 되풀이한다.
    "firmwarePulsePublisher": "best_effort",
}

LARGE_PUBLISHERS = {
    "odomPublisher",
    "firmwareInfoPublisher",
}

TELEMETRY_TOPICS = {
    "/odom",
    "/imu/data",
    "/imu/yaw_deg",
    "/imu/gyro_bias",
    "/estop/state",
    "/firmware/info",
    "/drive/enabled",
    "/drive/diag",
    # 08-18 1.6.3 — 관측되지 않으면 계약이 아니다. bag override 전수도 8 → 9.
    "/firmware/pulse",
}

# 사건 스트림도 기록해야 하지만 주기 telemetry 전수와는 따로 센다.
EVENT_TOPICS = {
    "/firmware/event",
}

FIELD_CONSUMERS = {
    "tools/rearm_field_regress.py": 3,
    "tools/rearm_field_disarm.py": 3,
    "tools/rearm_neg6_field.py": 2,
    "tools/rearm_field_wiring.py": 3,
    "tools/estop_toggle_check.py": 1,
}


def read(path):
    with open(path, encoding="utf-8") as handle:
        return handle.read()


def publisher_contract(source):
    matches = re.findall(
        r"rclc_publisher_init_(default|best_effort)\s*\(\s*"
        r"&([A-Za-z][A-Za-z0-9]*Publisher)\s*,",
        source,
        re.S,
    )
    return matches


def publish_contract(source):
    matches = re.findall(
        r"publishMeasured\s*\(\s*&([A-Za-z][A-Za-z0-9]*Publisher)\s*,"
        r"[\s\S]{0,180}?\b(RUNTIME_PUBLISH_[A-Z_]+)\s*\)",
        source,
    )
    return dict(matches), matches


def assert_publisher_contract(testcase, source):
    initializers = publisher_contract(source)
    # 주기 publisher 와 비주기 사건 publisher 를 **갈라서** 센다. 합쳐 세면
    # 계약의 "주기 8 → 9" 가 10 으로 보여 무엇이 늘었는지 알 수 없게 된다.
    periodic = [(k, n) for k, n in initializers if n not in EVENT_PUBLISHERS]
    events = [(k, n) for k, n in initializers if n in EVENT_PUBLISHERS]

    testcase.assertEqual(len(periodic), len(PUBLISHERS), initializers)
    testcase.assertEqual(
        {name for _, name in periodic}, set(PUBLISHERS), initializers)
    testcase.assertEqual(
        {name: kind for kind, name in periodic}, PUBLISHER_QOS,
        initializers)
    testcase.assertEqual(len(events), len(EVENT_PUBLISHERS), initializers)
    testcase.assertEqual(
        {name: kind for kind, name in events}, EVENT_PUBLISHERS, initializers)

    timeout_match = re.search(
        r"static\s+const\s+int\s+LARGE_PUBLISH_TIMEOUT_MS\s*=\s*(\d+)\s*;",
        source,
    )
    testcase.assertIsNotNone(timeout_match)
    testcase.assertEqual(int(timeout_match.group(1)), 20)
    timeout_publishers = re.findall(
        r"rmw_uros_set_publisher_session_timeout\s*\(\s*"
        r"rcl_publisher_get_rmw_handle\s*\(\s*&([A-Za-z][A-Za-z0-9]*Publisher)"
        r"\s*\)\s*,\s*LARGE_PUBLISH_TIMEOUT_MS\s*\)",
        source,
        re.S,
    )
    testcase.assertEqual(len(timeout_publishers), len(LARGE_PUBLISHERS))
    testcase.assertEqual(set(timeout_publishers), LARGE_PUBLISHERS)

    measured, raw_matches = publish_contract(source)
    testcase.assertEqual(len(raw_matches), len(PUBLISHERS), raw_matches)
    testcase.assertEqual(measured, PUBLISHERS)

    # 측정되지 않는 rcl_publish 우회가 없어야 한다. 🔴 08-18 부터 허용되는 자리는
    # **정확히 둘**이다: publishMeasured 안의 한 호출 + drainLinkEvents 의 사건 발행.
    # 개수만 2 로 늘리면 아무 데나 세 번째가 생겨도 안 잡히므로 자리까지 못박는다.
    testcase.assertEqual(source.count("rcl_publish("), 2)
    wrapper = extract_function(source, "publishMeasured")
    drain = extract_function(source, "drainLinkEvents")
    testcase.assertEqual(wrapper.count("rcl_publish("), 1)
    testcase.assertEqual(drain.count("rcl_publish("), 1)
    testcase.assertIn("&firmwareEventPublisher", drain)
    # 사건 발행은 publishMeasured 를 거치지 않는다 — 거치면 slot 이 9 → 10 이 된다.
    testcase.assertNotIn("publishMeasured(", drain)


def assert_spin_response_contract(testcase, source):
    loop = extract_function(source, "loop")
    testcase.assertLess(
        loop.index("runtimeGuardBeginLoop(&runtimeGuard)"),
        loop.index("const uint32_t loopStartedUs"),
    )
    testcase.assertRegex(loop, r"const\s+rcl_ret_t\s+spinRc\s*=")
    testcase.assertRegex(
        loop,
        r"if\s*\(spinRc\s*!=\s*RCL_RET_OK\s*&&\s*"
        r"driveGate\.state\s*==\s*DRIVE_ARMING\)\s*\{\s*"
        r"disarmDriveWithReason\(REARM_DISARM_SPIN_RESPONSE\);",
    )
    testcase.assertNotIn("disarmDrive();", loop)
    testcase.assertRegex(
        loop,
        r"if\s*\(spinRc\s*==\s*RCL_RET_OK\)\s*\{\s*"
        r"rearmGateArmBarrierStart\(&driveGate,\s*millis\(\)\);",
    )


def assert_diagnostics_refresh_contract(testcase, source):
    diagnostics = extract_function(source, "publishDiagnostics")
    gyro_publish = diagnostics.index("&gyroBiasPublisher")
    estop_publish = diagnostics.index("&estopStatePublisher")
    enabled_refresh = diagnostics.index("driveEnabledMessage.data =")
    enabled_publish = diagnostics.index("&driveEnabledPublisher")
    diag_refresh = diagnostics.index("driveDiagMessage.x =")
    diag_publish = diagnostics.index("&driveDiagPublisher")
    testcase.assertLess(gyro_publish, estop_publish)
    testcase.assertLess(estop_publish, enabled_refresh)
    testcase.assertLess(enabled_refresh, enabled_publish)
    testcase.assertLess(enabled_publish, diag_refresh)
    testcase.assertLess(diag_refresh, diag_publish)
    testcase.assertEqual(diagnostics.count("driveEnabledMessage.data ="), 1)
    testcase.assertEqual(diagnostics.count("driveDiagMessage.x ="), 1)
    testcase.assertEqual(diagnostics.count("driveDiagMessage.y ="), 1)
    testcase.assertEqual(diagnostics.count("driveDiagMessage.z ="), 1)


def assert_runtime_disarm_reason_contract(testcase, source):
    for name in ("recordRuntimePhase", "publishMeasured"):
        body = extract_function(source, name)
        testcase.assertIn(
            "disarmDriveWithReason(REARM_DISARM_RUNTIME_OVERRUN)", body)
        testcase.assertNotIn("disarmDrive();", body)


def header_count(header, name):
    match = re.search(
        r"static\s+const\s+uint8_t\s+" + re.escape(name)
        + r"\s*=\s*(\d+)U\s*;",
        header,
    )
    if match is None:
        raise AssertionError("runtime count not found: " + name)
    return int(match.group(1))


def enum_members(header, enum_name, prefix):
    match = re.search(
        r"enum\s+" + re.escape(enum_name) + r"\s*\{([^}]*)\}",
        header,
        re.S,
    )
    if match is None:
        raise AssertionError("runtime enum not found: " + enum_name)
    return re.findall(
        r"(?m)^\s*(" + re.escape(prefix) + r"[A-Z0-9_]+)\s*=\s*\d+",
        match.group(1),
    )


def firmware_info_fields(source):
    _buffer_size, fmt, args = fil.extract(source)
    specs = list(fil.SPEC.finditer(fmt))
    fields = {}
    for match in re.finditer(
            r"(?:^|;\s+)([a-z][a-z0-9_]*)=([^;]*)", fmt):
        key = match.group(1)
        value = match.group(2)
        start = sum(spec.start() < match.start(2) for spec in specs)
        arity = len(fil.SPEC.findall(value))
        if key in fields:
            raise AssertionError("duplicate firmware info field: " + key)
        fields[key] = (value, args[start:start + arity])
    return fields


def runtime_field_expected_args(header):
    phases = enum_members(header, "RuntimePhase", "RUNTIME_PHASE_")
    publishes = enum_members(
        header, "RuntimePublishSite", "RUNTIME_PUBLISH_")
    if len(phases) != header_count(header, "RUNTIME_PHASE_COUNT"):
        raise AssertionError("phase enum/count mismatch: " + repr(phases))
    if len(publishes) != header_count(header, "RUNTIME_PUBLISH_COUNT"):
        raise AssertionError("publish enum/count mismatch: " + repr(publishes))
    cast = "static_cast<unsigned long>({})".format
    return {
        "disarm_runtime": [cast("driveGate.disarmRuntimeCount")],
        "disarm_spin": [cast("driveGate.disarmSpinCount")],
        "runtime_overruns": [cast("runtimeGuard.overrunCount")],
        "runtime_last": [
            cast("runtimeGuard.lastOverrunCode"),
            cast("runtimeGuard.lastOverrunUs"),
        ],
        "publish_failures": [cast("runtimeGuard.publishFailureCount")],
        "phase_max_us": [
            cast("runtimeGuard.phaseMaxUs[{}]".format(member))
            for member in phases
        ],
        "publish_max_us": [
            cast("runtimeGuard.publishMaxUs[{}]".format(member))
            for member in publishes
        ],
    }


def assert_firmware_info_runtime_contract(testcase, source, header):
    fields = firmware_info_fields(source)
    expected = runtime_field_expected_args(header)
    testcase.assertEqual(set(expected), set(fc.RUNTIME_GUARD_FIELDS))
    for key in fc.RUNTIME_GUARD_FIELDS:
        testcase.assertIn(key, fields)
        value, actual_args = fields[key]
        testcase.assertEqual(
            len(fil.SPEC.findall(value)), len(expected[key]), key)
        testcase.assertEqual(actual_args, expected[key], key)


def remove_firmware_info_field(source, header, key):
    fields = firmware_info_fields(source)
    value, _actual_args = fields[key]
    expected_args = runtime_field_expected_args(header)[key]
    fragment = key + "=" + value + "; "
    mutated = source.replace(fragment, "", 1)
    if mutated == source:
        raise AssertionError("format fragment not removed: " + fragment)
    for arg in expected_args:
        old = "      " + arg + ",\n"
        changed = mutated.replace(old, "", 1)
        if changed == mutated:
            raise AssertionError("argument not removed: " + arg)
        mutated = changed
    return mutated


def swap_once(source, first, second):
    sentinel = "__RUNTIME_CONTRACT_SWAP_SENTINEL__"
    if source.count(first) != 1 or source.count(second) != 1:
        raise AssertionError("swap inputs must be unique")
    return source.replace(first, sentinel).replace(
        second, first).replace(sentinel, second)


def extract_function(source, name):
    match = re.search(
        r"^[\w:<>,\s\*&]*?\b" + re.escape(name) + r"\s*\([^;{]*\)\s*\{",
        source,
        re.M,
    )
    if match is None:
        raise AssertionError("production function not found: " + name)
    depth = 0
    for index in range(match.end() - 1, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[match.start():index + 1]
    raise AssertionError("unbalanced production function: " + name)


def compile_guard(header_text, main_text):
    compiler = shutil.which("g++")
    if compiler is None:
        raise unittest.SkipTest("g++ is not installed")
    with tempfile.TemporaryDirectory(prefix="runtime_guard_") as directory:
        header = os.path.join(directory, "runtime_guard.h")
        main = os.path.join(directory, "main.cpp")
        binary = os.path.join(directory, "guard_test")
        with open(header, "w", encoding="utf-8") as handle:
            handle.write(header_text)
        with open(main, "w", encoding="utf-8") as handle:
            handle.write(main_text)
        built = subprocess.run(
            [compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror",
             "-I", directory, main, "-o", binary],
            capture_output=True,
            text=True,
        )
        if built.returncode != 0:
            return built.returncode, built.stdout + built.stderr
        ran = subprocess.run([binary], capture_output=True, text=True)
        return ran.returncode, ran.stdout + ran.stderr


GUARD_MAIN = r'''
#include "runtime_guard.h"

int main()
{
  RuntimeGuard guard;
  runtimeGuardInit(&guard);
  if (RUNTIME_STALL_LIMIT_US != 400000U) return 18;
  if (!runtimeGuardRecordPhase(&guard, RUNTIME_PHASE_SPIN, 399999U)) return 1;
  if (runtimeGuardRecordPhase(&guard, RUNTIME_PHASE_SPIN, 400000U)) return 2;
  if (guard.phaseMaxUs[RUNTIME_PHASE_SPIN] != 400000U) return 3;
  if (guard.overrunCount != 1U) return 4;
  if (guard.lastOverrunUs != 400000U) return 5;
  if (guard.lastOverrunCode != RUNTIME_PHASE_SPIN) return 6;
  // Enclosing phase/loop observations must not erase the first concrete cause.
  if (runtimeGuardRecordPhase(&guard, RUNTIME_PHASE_LOOP, 450000U)) return 14;
  if (guard.overrunCount != 1U) return 15;
  if (guard.lastOverrunCode != RUNTIME_PHASE_SPIN) return 16;
  runtimeGuardBeginLoop(&guard);
  if (!runtimeGuardRecordPublish(
          &guard, RUNTIME_PUBLISH_ODOM, 10U, false)) return 7;
  if (guard.publishFailureCount != 1U) return 8;
  if (guard.publishMaxUs[RUNTIME_PUBLISH_ODOM] != 10U) return 9;
  if (runtimeGuardRecordPublish(
          &guard, RUNTIME_PUBLISH_IMU, 450000U, true)) return 10;
  if (guard.overrunCount != 2U) return 17;
  if (guard.lastOverrunCode !=
      RUNTIME_PUBLISH_CODE_BASE + RUNTIME_PUBLISH_IMU) return 11;
  if (runtimeGuardRecordPhase(&guard, RUNTIME_PHASE_COUNT, 1U)) return 12;
  if (runtimeGuardRecordPublish(
          &guard, RUNTIME_PUBLISH_COUNT, 1U, true)) return 13;
  return 0;
}
'''


class FirmwareRuntimeGuardTest(unittest.TestCase):

    def setUp(self):
        self.ino = read(INO_PATH)

    def test_periodic_publishers_match_mtu_qos_and_are_measured(self):
        assert_publisher_contract(self, self.ino)

    def test_publisher_enumeration_rejects_qos_timeout_and_count_mutations(
            self):
        removed = self.ino.replace(
            "rclc_publisher_init_default(\n      &odomPublisher,",
            "rclc_publisher_init_default(\n      &notTelemetry,",
            1,
        )
        with self.assertRaises(AssertionError):
            assert_publisher_contract(self, removed)

        added = self.ino.replace(
            "  // BEST_EFFORT XRCE output cannot fragment",
            "  RCCHECK(rclc_publisher_init_best_effort(\n"
            "      &extraPublisher, &node, type_support, \"extra\"));\n\n"
            "  // BEST_EFFORT XRCE output cannot fragment",
            1,
        )
        with self.assertRaises(AssertionError):
            assert_publisher_contract(self, added)

        mutations = (
            # Large samples must not return to unfragmented BEST_EFFORT.
            self.ino.replace(
                "rclc_publisher_init_default(\n      &odomPublisher,",
                "rclc_publisher_init_best_effort(\n      &odomPublisher,", 1),
            self.ino.replace(
                "rclc_publisher_init_default(\n      &firmwareInfoPublisher,",
                "rclc_publisher_init_best_effort(\n"
                "      &firmwareInfoPublisher,", 1),
            # Small telemetry must not regain the default 1000ms ACK wait.
            self.ino.replace(
                "rclc_publisher_init_best_effort(\n      &imuPublisher,",
                "rclc_publisher_init_default(\n      &imuPublisher,", 1),
            # Both bounded reliable publishers and the exact bench candidate
            # timeout are part of the safety contract.
            self.ino.replace(
                "rcl_publisher_get_rmw_handle(&odomPublisher),",
                "rcl_publisher_get_rmw_handle(&imuPublisher),", 1),
            self.ino.replace(
                "LARGE_PUBLISH_TIMEOUT_MS = 20;",
                "LARGE_PUBLISH_TIMEOUT_MS = 1000;", 1),
        )
        for mutated in mutations:
            self.assertNotEqual(mutated, self.ino)
            with self.assertRaises(AssertionError):
                assert_publisher_contract(self, mutated)

    def test_runtime_guard_production_boundary_and_observability(self):
        rc, output = compile_guard(read(GUARD_PATH), GUARD_MAIN)
        self.assertEqual(rc, 0, output)

    def test_runtime_guard_boundary_mutation_is_detected(self):
        source = read(GUARD_PATH)
        mutated = source.replace(
            "elapsedUs >= RUNTIME_STALL_LIMIT_US",
            "elapsedUs > RUNTIME_STALL_LIMIT_US",
            1,
        )
        self.assertNotEqual(mutated, source)
        rc, _ = compile_guard(mutated, GUARD_MAIN)
        self.assertNotEqual(rc, 0)

    def test_failed_spin_cannot_start_arming_barrier(self):
        assert_spin_response_contract(self, self.ino)
        for old, new in (
            ("spinRc != RCL_RET_OK", "spinRc == RCL_RET_OK"),
            ("if (spinRc == RCL_RET_OK) {", "if (spinRc != RCL_RET_OK) {"),
            ("disarmDriveWithReason(REARM_DISARM_SPIN_RESPONSE)",
             "disarmDrive()"),
        ):
            mutated = self.ino.replace(old, new, 1)
            with self.assertRaises(AssertionError):
                assert_spin_response_contract(self, mutated)

    def test_runtime_overrun_disarms_with_reason(self):
        assert_runtime_disarm_reason_contract(self, self.ino)
        mutated = self.ino.replace(
            "disarmDriveWithReason(REARM_DISARM_RUNTIME_OVERRUN)",
            "disarmDrive()",
            1,
        )
        with self.assertRaises(AssertionError):
            assert_runtime_disarm_reason_contract(self, mutated)

    def test_firmware_info_exposes_runtime_contract(self):
        header = read(GUARD_PATH)
        assert_firmware_info_runtime_contract(self, self.ino, header)

        # 필드와 인자를 같이 지우면 길이 검사는 통과한다.
        # 여기서 7필드 각각을 계약 소실로 거부한다(§74.3).
        for key in fc.RUNTIME_GUARD_FIELDS:
            mutated = remove_firmware_info_field(self.ino, header, key)
            with self.assertRaises(AssertionError, msg=key):
                assert_firmware_info_runtime_contract(self, mutated, header)

        phase_members = enum_members(
            header, "RuntimePhase", "RUNTIME_PHASE_")
        publish_members = enum_members(
            header, "RuntimePublishSite", "RUNTIME_PUBLISH_")

        # 배열 크기는 시험에 숫자를 복사하지 않고 헤더에서 읽는다.
        phase_value, phase_args = firmware_info_fields(
            self.ino)["phase_max_us"]
        mutated = self.ino.replace(
            "phase_max_us=" + phase_value,
            "phase_max_us=" + phase_value.replace(",%lu", "", 1),
            1,
        )
        mutated = mutated.replace("      " + phase_args[-1] + ",\n", "", 1)
        with self.assertRaises(AssertionError):
            assert_firmware_info_runtime_contract(self, mutated, header)

        publish_value, publish_args = firmware_info_fields(
            self.ino)["publish_max_us"]
        mutated = self.ino.replace(
            "publish_max_us=" + publish_value,
            "publish_max_us=" + publish_value.replace(",%lu", "", 1),
            1,
        )
        mutated = mutated.replace("      " + publish_args[-1] + ",\n", "", 1)
        with self.assertRaises(AssertionError):
            assert_firmware_info_runtime_contract(self, mutated, header)

        # 개수가 같아도 enum 해독표와 인자 순서가 갈라지면 실패한다.
        phase_first = "runtimeGuard.phaseMaxUs[{}]".format(phase_members[0])
        phase_last = "runtimeGuard.phaseMaxUs[{}]".format(phase_members[-1])
        mutated = swap_once(self.ino, phase_first, phase_last)
        with self.assertRaises(AssertionError):
            assert_firmware_info_runtime_contract(self, mutated, header)

        publish_first = "runtimeGuard.publishMaxUs[{}]".format(
            publish_members[0])
        publish_last = "runtimeGuard.publishMaxUs[{}]".format(
            publish_members[-1])
        mutated = swap_once(self.ino, publish_first, publish_last)
        with self.assertRaises(AssertionError):
            assert_firmware_info_runtime_contract(self, mutated, header)

    def test_state_messages_refresh_after_preceding_publish_side_effects(self):
        assert_diagnostics_refresh_contract(self, self.ino)
        diagnostics = extract_function(self.ino, "publishDiagnostics")
        assignment = (
            "  driveEnabledMessage.data = "
            "(driveGate.state == DRIVE_ARMED);\n")
        mutated_diagnostics = diagnostics.replace(assignment, "", 1)
        mutated_diagnostics = mutated_diagnostics.replace(
            "  RCSOFTCHECK(publishMeasured(\n"
            "      &gyroBiasPublisher, &gyroBiasMessage, "
            "RUNTIME_PUBLISH_GYRO_BIAS));",
            assignment
            + "  RCSOFTCHECK(publishMeasured(\n"
            "      &gyroBiasPublisher, &gyroBiasMessage, "
            "RUNTIME_PUBLISH_GYRO_BIAS));",
            1,
        )
        mutated = self.ino.replace(diagnostics, mutated_diagnostics, 1)
        with self.assertRaises(AssertionError):
            assert_diagnostics_refresh_contract(self, mutated)

    def test_field_consumers_are_exhaustively_best_effort(self):
        total = 0
        for relative, expected_count in FIELD_CONSUMERS.items():
            source = read(os.path.join(ROOT, relative))
            calls = re.findall(
                r"create_subscription\s*\([\s\S]{0,180}?\)", source)
            self.assertEqual(len(calls), expected_count, relative)
            self.assertTrue(
                all("qos_profile_sensor_data" in call for call in calls),
                (relative, calls),
            )
            total += len(calls)
        self.assertEqual(total, 12)

    def test_field_consumer_qos_mutation_is_detected(self):
        relative = "tools/rearm_field_regress.py"
        source = read(os.path.join(ROOT, relative))
        mutated = source.replace("qos_profile_sensor_data)", "10)", 1)
        calls = re.findall(
            r"create_subscription\s*\([\s\S]{0,180}?\)", mutated)
        self.assertFalse(
            len(calls) == FIELD_CONSUMERS[relative]
            and all("qos_profile_sensor_data" in call for call in calls))

    def test_bag_qos_has_exact_telemetry_closure(self):
        source = read(BAG_QOS_PATH)
        blocks = re.findall(
            r"(?m)^(/[^:]+):\n((?:^[ \t]+[^\n]*\n?)+)", source)
        telemetry = {
            topic: body for topic, body in blocks if topic in TELEMETRY_TOPICS}
        self.assertEqual(set(telemetry), TELEMETRY_TOPICS)
        self.assertTrue(
            all("reliability: best_effort" in body
                and "durability: volatile" in body
                for body in telemetry.values()), telemetry)
        # `/cmd_vel`은 별도 RELIABLE 명령 계약이며 telemetry 전수에 섞지 않는다.
        self.assertIn("/cmd_vel:", source)
        self.assertIn("reliability: reliable", source)


if __name__ == "__main__":
    unittest.main()
