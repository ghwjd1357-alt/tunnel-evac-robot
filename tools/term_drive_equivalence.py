#!/usr/bin/env python3
"""원본 펌웨어(오라클) ↔ 클래스 리팩터 펌웨어(term_drive_v2) **동작 동치** 검사.

무엇을 증명하나
---------------
`firmware/term_drive_v2/` 는 `firmware/teensy_integrated_base_v1_4/` 의 구동 경로를
바퀴 배열·자유 함수에서 `DriveBase`/`Wheel`/`WheelSpeedController`/`PeriodicTimer`
클래스로 옮긴 판이다. 이 도구는 **원본을 정답(오라클)** 으로 놓고, 같은 입력을 준
두 프로그램의 관측값이 **비트 단위로** 같은지 본다. 기대값을 새 코드에서 만들지 않는다.

어떻게
------
1. 두 `.ino` 에서 구동 경로 함수·전역·상수·객체 정의를 **글자 그대로** 떼어낸다
   (`ino_host_probe` 와 같은 생각 — 베끼지 않는다). 새 판은 헤더를 `-I` 로 그대로 쓴다.
   ⚠ `struct TeensyDriveSink { void stopAllMotors() { ::stopAllMotors(); } }` 가 진짜
   `void stopAllMotors()` 보다 앞에 있다. 그래서 함수는 **0열에서 시작하는 정의**만,
   그리고 정확히 1개일 때만 뗀다 (2개 이상/0개면 판정 불능).
2. 하드웨어는 두 프로그램에 **같은 글자**의 가짜를 붙인다 — uint32 millis/micros 시계
   (랩어라운드 가능), analogWrite/digitalWrite/엔코더 read·write 를 전부 기록, E-stop 핀,
   rearm_gate.h/drive_wiring.h 는 실물을 그대로 include.
3. 공용 시나리오 구동기(두 프로그램에 같은 글자)가 loop() 순서를 흉내 낸다.
     direct 모드: [명령 → applySkidSteerCommand] → updateAppliedPwmEpoch → updateMotorOutputs → updateOdometry
     loop   모드: checkSafety → [cmdVelCallback/driveEnableCallback] → rearmGateArmBarrierStart
                  → checkSafety → updateAppliedPwmEpoch → updateMotorOutputs → updateOdometry
   닫힌 루프 플랜트(정수 연산)가 마지막 PWM·DIR 로 엔코더를 굴린다 → 어긋남이 전파된다.
4. 매 tick 한 줄: 하드웨어 호출 순서(핀,값), 오도메트리 5개, applied_pwm_max/epoch,
   바퀴별 내부 상태 10개(원본 배열 ↔ 새 getter), 타이머 마지막 시각. double 은 %a(16진).
   이름 붙은 시나리오는 전문(全文)을 줄 단위로 비교, 퍼즈는 FNV-1a 해시 비교 후
   불일치면 그 seed 를 전문으로 다시 돌려 첫 차이 tick·필드를 찾는다.
5. 플랫폼: host g++ -O1 (기본), host g++ -O2 -mfma -ffp-contract=fast (CPU 가 FMA 지원 시),
   그리고 **Teensy 실제 툴체인**(arm-none-eabi-g++ 11.3.1, -Os -mcpu=cortex-m7 -mfpu=fpv5-d16)
   으로 빌드해 qemu-arm-static -cpu cortex-m7 에서 실행 (둘 다 있을 때). 플랫폼 간 비교는
   하지 않는다 — 같은 플랫폼 안에서 원본 ↔ 새 판만 비교한다.
6. PeriodicTimer: 구동 경로 밖 4개(diag 1000ms, fw info 5000ms, time sync 30000ms, IMU 20000us)
   는 원본 함수 머리(`if (now - last < P) return; last = now;`)를 원문에서 떼어 새 판 머리와
   같은 시계로 나란히 돌린다 (+ 순수 패턴 ↔ 클래스 비교, restart 포함, 랩어라운드 포함).
7. 안전 헤더 5개 sha256 이 두 폴더에서 같은지.
8. 구조 검사: 두 `.ino` 의 0열 함수 정의를 모두 비교해, 바뀐 함수 집합이 이 시험이 돌리는
   집합(+정규화로 증명되는 치환)과 정확히 같은지. 예상 밖 변경 = 실패.
9. `--mutate`: 임시 사본에 단일 변이를 하나씩 넣고 이 시험이 **실패하는지** 확인.

종료코드: 0 = 전부 일치 / 1 = 불일치(또는 변이 미검출이 '반드시 잡혀야 함' 항목) /
          2 = 판정 불능(추출·컴파일·실행 실패). "못 돌렸다" 를 "통과" 로 적지 않는다.

사용
----
    python3 tools/term_drive_equivalence.py                # 전체 (host + ARM/qemu 가능 시)
    python3 tools/term_drive_equivalence.py --no-arm       # host 만
    python3 tools/term_drive_equivalence.py --mutate       # 기본 검사 + 변이 검사
    python3 tools/term_drive_equivalence.py --mutate-only  # 변이 검사만
    python3 tools/term_drive_equivalence.py --seeds 400 --ticks 3000
"""

import argparse
import concurrent.futures
import hashlib
import os
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ORIG_DIR = os.path.join(ROOT, "firmware", "teensy_integrated_base_v1_4")
NEW_DIR = os.path.join(ROOT, "firmware", "term_drive_v2")
ORIG_INO = os.path.join(ORIG_DIR, "teensy_integrated_base_v1_4.ino")
NEW_INO_NAME = "term_drive_v2.ino"

SAFETY_HEADERS = ["rearm_gate.h", "drive_wiring.h", "estop_debounce.h",
                  "runtime_guard.h", "link_stall_probe.h"]

TEENSY_TOOLCHAIN_GLOB = os.path.expanduser(
    "~/.arduino15/packages/teensy/tools/teensy-compile")


class ProbeError(RuntimeError):
    """추출·컴파일·실행 불가 = 판정 불능 (종료코드 2)."""


# ════════════════════════════════════════════════════════════════════════════
# 1. 원문 떼어내기 — 전부 글자 그대로
# ════════════════════════════════════════════════════════════════════════════
def read(path):
    with open(path, encoding="utf-8") as handle:
        return handle.read()


def _brace_end(src, open_index):
    depth = 0
    for index in range(open_index, len(src)):
        char = src[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return index
    raise ProbeError("중괄호가 안 닫혔다 (위치 %d)" % open_index)


def top_function(src, name):
    """0열에서 시작하는 `name(...) {` 정의를 정확히 1개 찾아 원문 그대로 돌려준다.

    들여쓴 정의(예: TeensyDriveSink 안의 한 줄 메서드 stopAllMotors)는 제외된다.
    """
    pattern = re.compile(
        r"^(?=[A-Za-z_])[\w:<>,\*&]+(?:[ \t]+[\w:<>,\*&]+)*[ \t]+\**%s[ \t]*\([^;{}]*\)\s*\{"
        % re.escape(name), re.M)
    matches = list(pattern.finditer(src))
    if len(matches) != 1:
        raise ProbeError("0열 함수 정의 %s 가 %d 개다 (정확히 1 이어야 한다)"
                         % (name, len(matches)))
    match = matches[0]
    end = _brace_end(src, match.end() - 1)
    return src[match.start():end + 1]


def all_top_functions(src):
    """0열 함수 정의 전부 {이름: 원문}. 이름 중복은 판정 불능."""
    pattern = re.compile(
        r"^(?=[A-Za-z_])[\w:<>,\*&]+(?:[ \t]+[\w:<>,\*&]+)*[ \t]+\**(\w+)[ \t]*\([^;{}]*\)\s*\{",
        re.M)
    result = {}
    for match in pattern.finditer(src):
        name = match.group(1)
        if name in ("if", "while", "for", "switch"):
            continue
        end = _brace_end(src, match.end() - 1)
        if name in result:
            raise ProbeError("0열 함수 %s 가 두 번 정의됐다" % name)
        result[name] = src[match.start():end + 1]
    return result


def unique(src, regex, what, flags=re.M):
    matches = list(re.finditer(regex, src, flags))
    if len(matches) != 1:
        raise ProbeError("%s 를 정확히 1곳에서 찾지 못했다 (%d 곳)" % (what, len(matches)))
    return matches[0]


def struct_block(src, name):
    """`struct NAME {` ... `};` 를 원문 그대로."""
    match = unique(src, r"^struct %s \{" % re.escape(name), "struct " + name)
    end = _brace_end(src, match.end() - 1)
    if src[end + 1] != ";":
        raise ProbeError("struct %s 뒤에 ; 가 없다" % name)
    return src[match.start():end + 2]


def constants(src):
    """최상단 `static const` 전량 (문자열 상수만 제외). HIGH 는 shim 이 정의하므로 포함."""
    kept, names = [], set()
    for match in re.finditer(r"^static const [^;]*?;", src, re.M | re.S):
        text = match.group(0)
        head = text.partition("=")[0]
        if re.search(r"\bchar\b", head):
            continue
        if "STRINGIFY" in text:
            continue
        name = re.search(r"(\w+)\s*(\[[^\]]*\])?\s*$", head.strip()).group(1)
        if name in names:
            raise ProbeError("상수 %s 가 두 번 나온다" % name)
        names.add(name)
        kept.append(text)
    if not kept:
        raise ProbeError("상수를 하나도 못 뗐다")
    return "\n".join(kept)


def quoted_includes(src):
    lines = re.findall(r'^#include "[^"]+"[ \t]*$', src, re.M)
    if not lines:
        raise ProbeError("#include \"...\" 가 없다")
    return "\n".join(lines)


def runtime_state_region(src):
    """`// Runtime state` 머리 뒤부터 `uint64_t lastPublishedTimestampNs = 0;` 까지."""
    start = unique(src, r"^// Runtime state\n// =+\n", "Runtime state 머리")
    end = unique(src, r"^uint64_t lastPublishedTimestampNs = 0;$", "lastPublishedTimestampNs")
    if end.start() < start.end():
        raise ProbeError("Runtime state 구간 순서가 이상하다")
    return src[start.end():end.end()]


def forward_decl_region(src):
    start = unique(src, r"^void errorLoop\(\);$", "전방선언 시작")
    end = unique(src, r"^\s+double accelZ\);$", "전방선언 끝")
    return src[start.start():end.end()]


def extract_drive_pieces(src, kind):
    """kind = 'orig' | 'new'. 구동 경로 원문 조각들을 순서대로."""
    pieces = []
    pieces.append(quoted_includes(src))
    pieces.append(constants(src))
    pieces.append(unique(src, r"^enum MotorIndex \{.*?\};", "enum MotorIndex",
                         re.M | re.S).group(0))
    encoder_lines = re.findall(r"^Encoder encoder\w+\(\d+, \d+\);$", src, re.M)
    if len(encoder_lines) != 4:
        raise ProbeError("Encoder 객체가 4개가 아니다")
    pieces.append("\n".join(encoder_lines))
    pieces.append(unique(src, r"^Encoder\* const encoders\[4\] = \{.*?\};",
                         "encoders 배열", re.M | re.S).group(0))
    pieces.append(runtime_state_region(src))
    pieces.append(forward_decl_region(src))
    pieces.append(struct_block(src, "TeensyDriveSink"))
    pieces.append(unique(src, r"^TeensyDriveSink driveSink;$", "driveSink").group(0))
    pieces.append(unique(src, r"^struct EstopDebounce estopFilter;$", "estopFilter").group(0))

    if kind == "orig":
        names = ["clampDouble", "normalizeAngle", "updateEstopFilter", "isEstopActive",
                 "getMonotonicTimestampNs", "writeMotorPwm", "resetWheelControllerState",
                 "resetAllWheelControllers", "stopAllMotors",
                 "wheelVelocityToFeedforwardPwm", "setTargetWheelVelocity",
                 "applySkidSteerCommand", "updateWheelControllers", "movePwmTowardTarget",
                 "updateAppliedPwmEpoch", "updateMotorOutputs", "disarmDrive",
                 "disarmDriveWithReason", "checkSafety", "readEncoderCount",
                 "resetOdometry", "updateOdometry", "cmdVelCallback",
                 "driveEnableCallback"]
    else:
        names = ["normalizeAngle", "updateEstopFilter", "isEstopActive",
                 "getMonotonicTimestampNs", "writeMotorPwm", "stopAllMotors",
                 "applySkidSteerCommand", "updateWheelControllers",
                 "updateAppliedPwmEpoch", "updateMotorOutputs", "disarmDrive",
                 "disarmDriveWithReason", "checkSafety", "readEncoderCount",
                 "resetOdometry", "updateOdometry", "cmdVelCallback",
                 "driveEnableCallback"]
    for name in names:
        body = top_function(src, name)
        if name == "stopAllMotors" and "for (int motor" not in body:
            raise ProbeError("stopAllMotors 추출이 TeensyDriveSink 한 줄 메서드를 집었다")
        pieces.append(body)
    return pieces


# ════════════════════════════════════════════════════════════════════════════
# 2. C++ 공용 텍스트 (두 프로그램에 같은 글자)
# ════════════════════════════════════════════════════════════════════════════
PRELUDE = r"""
// ── 시스템 헤더 (Arduino.h 가 먼저 끌어오는 것들) ──
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <stdbool.h>
#include <utility>

// ── Teensy 코어 teensy4/wiring.h · core_pins.h 의 정의를 그대로 ──
#define HIGH 1
#define LOW 0
#ifdef PI
#undef PI
#endif
#define PI 3.1415926535897932384626433832795
#if __cplusplus >= 201103L && defined(__STRICT_ANSI__)
#define typeof(a) decltype(a)
#endif
template<class A, class B>
constexpr auto min(A&& a, B&& b) -> decltype(a < b ? std::forward<A>(a) : std::forward<B>(b)) {
  return a < b ? std::forward<A>(a) : std::forward<B>(b);
}
template<class A, class B>
constexpr auto max(A&& a, B&& b) -> decltype(a < b ? std::forward<A>(a) : std::forward<B>(b)) {
  return a >= b ? std::forward<A>(a) : std::forward<B>(b);
}
#ifdef abs
#undef abs
#endif
#define abs(x) ({ \
  typeof(x) _x = (x); \
  (_x > 0) ? _x : -_x; \
})
#define constrain(amt, low, high) ({ \
  typeof(amt) _amt = (amt); \
  typeof(low) _low = (low); \
  typeof(high) _high = (high); \
  (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); \
})
#define round(x) ((long) __builtin_round(x))
"""

STUBS = r"""
// ════ 가짜 하드웨어 (두 프로그램 공통 글자) ════
static uint32_t g_us = 0;     // micros()
static uint32_t g_ms = 0;     // millis()
static uint32_t g_sub = 0;    // ms 미만 누적 [us]
uint32_t millis() { return g_ms; }
uint32_t micros() { return g_us; }
static void advance_clock(uint32_t d) {
  g_us += d;
  g_sub += d;
  g_ms += g_sub / 1000u;
  g_sub %= 1000u;
}

static char g_hw[65536];
static size_t g_hwLen = 0;
static int g_hwOverflow = 0;
static void hwlog(const char* fmt, ...) {
  if (g_hwLen + 256 >= sizeof(g_hw)) { g_hwOverflow = 1; return; }
  if (g_hwLen > 0) g_hw[g_hwLen++] = ',';
  va_list ap; va_start(ap, fmt);
  int n = vsnprintf(g_hw + g_hwLen, sizeof(g_hw) - g_hwLen, fmt, ap);
  va_end(ap);
  if (n < 0) { g_hwOverflow = 1; return; }
  g_hwLen += (size_t)n;
}

// 플랜트(테스트 고정물): 핀 → 바퀴 번호. 새 판이 핀을 틀리게 쓰면 플랜트가 다르게 굴러 드러난다.
static const uint8_t PLANT_PWM_PINS[4] = {2, 4, 6, 8};
static const uint8_t PLANT_DIR_PINS[4] = {3, 5, 7, 9};
static const int64_t PLANT_GAIN[4] = {20, 19, 21, 20};   // [counts/s per PWM]
static const int PLANT_DEADBAND_PWM = 12;
struct Plant {
  int pwm[4];
  int dirLevel[4];
  int64_t speed[4];   // [counts/s]
  int64_t acc[4];     // [micro-counts]
  int32_t noise[4];
  bool frozen[4];
  bool pushed[4];      // 외부 힘으로 바퀴 속도를 강제 (내리막·끌림) — PWM 과 무관
  int64_t pushSpeed[4];
  int noiseAmp;
};
static Plant g_plant;

void analogWrite(uint8_t pin, int val) {
  hwlog("A%u:%d", (unsigned)pin, val);
  for (int i = 0; i < 4; ++i) if (pin == PLANT_PWM_PINS[i]) g_plant.pwm[i] = val;
}
void digitalWrite(uint8_t pin, uint8_t val) {
  hwlog("D%u:%u", (unsigned)pin, (unsigned)val);
  for (int i = 0; i < 4; ++i) if (pin == PLANT_DIR_PINS[i]) g_plant.dirLevel[i] = val;
}
static int g_estopPinLevel = LOW;
static uint8_t g_estopPinNumber = 255;
uint8_t digitalRead(uint8_t pin) {
  return (pin == g_estopPinNumber) ? (uint8_t)g_estopPinLevel : (uint8_t)LOW;
}
void noInterrupts() { hwlog("nI"); }
void interrupts() { hwlog("I"); }
uint64_t rmw_uros_epoch_nanos() { return 0; }

static int64_t floor_div(int64_t a, int64_t b) {
  int64_t q = a / b;
  if ((a % b != 0) && ((a < 0) != (b < 0))) --q;
  return q;
}
static int g_encoderCount = 0;
class Encoder {
public:
  Encoder(int pinA, int pinB) : index(g_encoderCount++) { (void)pinA; (void)pinB; }
  int32_t read() {
    const int32_t value = (int32_t)(floor_div(g_plant.acc[index], 1000000) + g_plant.noise[index]);
    hwlog("R%d:%ld", index, (long)value);
    return value;
  }
  void write(int32_t p) {
    hwlog("W%d:%ld", index, (long)p);
    g_plant.acc[index] = (int64_t)p * 1000000;
    g_plant.noise[index] = 0;
  }
private:
  int index;
};

// micro-ROS 메시지 형의 최소 가짜 (콜백 원문이 쓰는 필드만)
struct FakeVector3 { double x; double y; double z; };
struct geometry_msgs__msg__Twist { FakeVector3 linear; FakeVector3 angular; };
struct std_srvs__srv__SetBool_Request { bool data; };
struct std_srvs__srv__SetBool_Response { bool success; };
"""

# 선언만 되고 원문을 안 가져오는 함수 (발행) — 값만 기록
TAIL_STUBS = r"""
void publishOdometry(uint64_t timestampNs) {
  hwlog("P%llu", (unsigned long long)timestampNs);
}
"""

ADAPTER_ORIG = r"""
// ════ 관측 어댑터 — 원본: 전역 배열을 읽는다 ════
struct WheelView {
  double target, meas, filt; int desired, applied; uint32_t boost; long prevCount;
  double integ, prevErr, dfilt; unsigned pwmPin, dirPin;
};
static void adapter_view(int i, WheelView& v) {
  v.target = targetWheelVelocity[i];
  v.meas = measuredWheelVelocity[i];
  v.filt = filteredWheelVelocity[i];
  v.desired = desiredPwm[i];
  v.applied = appliedPwm[i];
  v.boost = startBoostUntilMs[i];
  v.prevCount = previousCount[i];
  v.integ = wheelIntegralError[i];
  v.prevErr = wheelPreviousError[i];
  v.dfilt = wheelFilteredDerivative[i];
  v.pwmPin = PWM_PIN[i];
  v.dirPin = DIR_PIN[i];
}
static uint32_t adapter_pwm_last() { return lastPwmUpdateMs; }
static uint32_t adapter_odom_last() { return previousOdomUs; }
"""

ADAPTER_NEW = r"""
// ════ 관측 어댑터 — 새 판: Wheel 의 공개 getter 만 쓴다 ════
struct WheelView {
  double target, meas, filt; int desired, applied; uint32_t boost; long prevCount;
  double integ, prevErr, dfilt; unsigned pwmPin, dirPin;
};
static void adapter_view(int i, WheelView& v) {
  const Wheel& w = drive.wheel(i);
  v.target = w.getTarget();
  v.meas = w.getMeasuredVelocity();
  v.filt = w.getFilteredVelocity();
  v.desired = w.getDesiredPwm();
  v.applied = w.getAppliedPwm();
  v.boost = w.getStartBoostUntilMs();
  v.prevCount = w.getPreviousCount();
  v.integ = w.getController().getIntegralError();
  v.prevErr = w.getController().getPreviousError();
  v.dfilt = w.getController().getFilteredDerivative();
  // 핀 번호는 새 판에서도 Wheel 이 아니라 .ino 의 상수 표에 있다 (검토 P2 반영 후).
  v.pwmPin = PWM_PIN[i];
  v.dirPin = DIR_PIN[i];
}
static uint32_t adapter_pwm_last() { return pwmRampTimer.getLastTime(); }
static uint32_t adapter_odom_last() { return odomTimer.getLastTime(); }
"""

DRIVER = r"""
// ════════════════════════════════════════════════════════════════════════════
// 공용 시나리오 구동기 (두 프로그램에 같은 글자)
// ════════════════════════════════════════════════════════════════════════════
enum { MODE_DIRECT = 0, MODE_LOOP = 1 };
enum Op {
  OP_CMD,        // a=v b=w : 스트림 명령을 바꾸고 즉시 1회 송신
  OP_STREAM,     // i=0/1 스트림 끄기/켜기 (끄면 loop 모드 워치독이 작동)
  OP_PERIOD,     // i=스트림 주기 [ms]
  OP_ESTOP,      // i=0/1
  OP_CMDRX,      // i=0/1 cmdVelReceived 강제
  OP_GATE,       // i=0/1 출력 허용 (direct: 상태 강제, loop: enable 서비스/자동 재무장)
  OP_RESET_ODOM, // resetOdometry()
  OP_JUMP_US,    // i=시계 점프 [us] (다음 진행에 더한다)
  OP_FREEZE,     // i=바퀴, a=0/1
  OP_NOISE,      // i=진폭 [counts]
  OP_ZERO_DT,    // i=시계가 안 가는 tick 수
  OP_FIXED_DT,   // i=고정 tick 간격 [us] (0=지터)
  OP_WC_DIRECT,  // a=dt : updateWheelControllers(dt) 직접 호출
  OP_ONCE,       // a=v b=w : 스트림을 안 바꾸고 1회만 송신
  OP_UNIT,       // 단위 호출 묶음 (normalizeAngle · writeMotorPwm · 극단 명령)
  OP_PUSH,       // i=바퀴, a=강제 속도 [counts/s], b=1 켜기 / 0 끄기
  OP_END
};
struct Step { uint32_t atMs; int op; int i; double a; double b; };

struct Sim {
  int mode;
  uint64_t rng;
  int tick;
  uint64_t simUs;       // 랩어라운드 없는 경과 시간
  uint64_t t0Us;        // 시나리오 시각 0 (loop 모드는 첫 무장 시각)
  bool t0Set;
  double cmdV, cmdW;
  bool streamOn;
  uint32_t streamPeriodUs;
  uint64_t lastSendUs;
  bool sendNow;
  bool onceP; double onceV, onceW;
  bool autoArm;
  bool estop;
  uint32_t jumpUs;
  int zeroDtTicks;
  uint32_t fixedDt;
  uint32_t jitMin, jitMax;
  bool fuzz;
  const Step* steps;
  int stepIndex;
  // 관측 커버리지 (원본·새 판 같은 글자로 같은 값 → 같아야 함)
  unsigned long long cov[16];
  int prevApplied[4];
  double prevTarget[4];
};

static uint64_t rng_next(uint64_t& s) {
  s ^= s >> 12; s ^= s << 25; s ^= s >> 27;
  return s * 2685821657736338717ULL;
}
static uint32_t rng_range(uint64_t& s, uint32_t lo, uint32_t hi) {  // [lo, hi]
  if (hi <= lo) return lo;
  return lo + (uint32_t)(rng_next(s) % (uint64_t)(hi - lo + 1));
}
static double rng_unit(uint64_t& s) {
  return (double)(rng_next(s) >> 11) * (1.0 / 9007199254740992.0);
}
static double rng_double(uint64_t& s, double lo, double hi) {
  const double u = rng_unit(s);
  return lo + (hi - lo) * u;
}

static void send_cmd(Sim& s, double v, double w) {
  hwlog("cmd(%a;%a)", v, w);
  if (s.mode == MODE_DIRECT) {
    applySkidSteerCommand(v, w);
  } else {
    geometry_msgs__msg__Twist m;
    memset(&m, 0, sizeof(m));
    m.linear.x = v;
    m.angular.z = w;
    cmdVelCallback(&m);
  }
}
static void send_enable(bool on) {
  std_srvs__srv__SetBool_Request req; std_srvs__srv__SetBool_Response resp;
  req.data = on; resp.success = false;
  driveEnableCallback(&req, &resp);
  hwlog("enable(%d)=%d", on ? 1 : 0, resp.success ? 1 : 0);
}

static void unit_calls() {
  // normalizeAngle — 같은 원문이지만 목록에 있으므로 값으로도 본다
  const double angles[] = {0.0, PI, -PI, 3.2, -3.2, 7.0, -7.0, 100.0, -100.0, 1e6,
                           -0.0, 2.0 * PI, -2.0 * PI, 3.0 * PI + 1e-9};
  for (size_t k = 0; k < sizeof(angles) / sizeof(angles[0]); ++k) {
    hwlog("nA(%a)=%a", angles[k], normalizeAngle(angles[k]));
  }
  // writeMotorPwm — 범위·부호·E-stop
  const int pw[] = {-300, -256, -255, -100, -1, 0, 1, 2, 100, 255, 256, 300};
  for (int m = 0; m < 4; ++m) {
    for (size_t k = 0; k < sizeof(pw) / sizeof(pw[0]); ++k) writeMotorPwm(m, pw[k]);
  }
  estopFilter.stable = true;
  for (int m = 0; m < 4; ++m) writeMotorPwm(m, 50);
  estopFilter.stable = false;
  stopAllMotors();
}

static void apply_step(Sim& s, const Step& st) {
  switch (st.op) {
    case OP_CMD: s.cmdV = st.a; s.cmdW = st.b; s.sendNow = true; break;
    case OP_STREAM: s.streamOn = st.i != 0; hwlog("stream(%d)", st.i); break;
    case OP_PERIOD: s.streamPeriodUs = (uint32_t)st.i * 1000u; break;
    case OP_ESTOP:
      s.estop = st.i != 0; hwlog("estop(%d)", st.i);
      if (s.mode == MODE_DIRECT) estopFilter.stable = s.estop;
      break;
    case OP_CMDRX: cmdVelReceived = st.i != 0; hwlog("rx(%d)", st.i); break;
    case OP_GATE:
      hwlog("gate(%d)", st.i);
      if (s.mode == MODE_DIRECT) {
        driveGate.state = st.i ? DRIVE_ARMED : DRIVE_DISARMED;
        if (st.i) cmdVelReceived = true;
      } else {
        if (st.i) { s.autoArm = true; } else { s.autoArm = false; send_enable(false); }
      }
      break;
    case OP_RESET_ODOM: hwlog("resetOdom"); resetOdometry(); break;
    case OP_JUMP_US: s.jumpUs += (uint32_t)st.i; break;
    case OP_FREEZE: g_plant.frozen[st.i] = st.a != 0.0; hwlog("freeze(%d;%d)", st.i, st.a != 0.0); break;
    case OP_NOISE: g_plant.noiseAmp = st.i; break;
    case OP_ZERO_DT: s.zeroDtTicks += st.i; break;
    case OP_FIXED_DT: s.fixedDt = (uint32_t)st.i; break;
    case OP_WC_DIRECT: hwlog("wc(%a)", st.a); updateWheelControllers(st.a); break;
    case OP_ONCE: s.onceP = true; s.onceV = st.a; s.onceW = st.b; break;
    case OP_UNIT: unit_calls(); break;
    case OP_PUSH:
      g_plant.pushed[st.i] = st.b != 0.0; g_plant.pushSpeed[st.i] = (int64_t)st.a;
      hwlog("push(%d;%ld;%d)", st.i, (long)st.a, st.b != 0.0);
      break;
    default: break;
  }
}

static void fuzz_events(Sim& s) {
  uint64_t& r = s.rng;
  const uint32_t p = rng_range(r, 0, 99999);   // 1e-5 단위 확률
  if (p < 2000) {                                // 2% 새 명령
    double v = 0, w = 0;
    switch (rng_range(r, 0, 11)) {
      case 0: v = 0; w = 0; break;
      case 1: v = rng_double(r, -0.25, 0.25); w = rng_double(r, -1.0, 1.0); break;
      case 2: v = rng_double(r, -0.004, 0.004); w = 0; break;
      case 3: v = rng_double(r, -0.025, 0.025); w = rng_double(r, -0.02, 0.02); break;
      case 4: v = 0; w = rng_double(r, -1.0, 1.0); break;
      case 5: v = (rng_range(r, 0, 1) ? 0.2 : -0.2); w = (rng_range(r, 0, 1) ? 0.83 : -0.83); break;
      case 6: v = rng_double(r, -5, 5); w = rng_double(r, -50, 50); break;
      case 7: v = 0; w = rng_double(r, -0.02, 0.02); break;
      case 8: v = rng_double(r, 0.0, 0.2); w = rng_double(r, -0.3, 0.3); break;
      case 9: v = -rng_double(r, 0.0, 0.2); w = rng_double(r, -0.3, 0.3); break;
      case 10: v = (rng_range(r, 0, 1) ? 0.002 : -0.002); w = 0; break;
      default: v = rng_double(r, -0.2, 0.2); w = 0; break;
    }
    s.cmdV = v; s.cmdW = w; s.sendNow = true;
  } else if (p < 2020) {
    s.onceP = true; s.onceV = rng_range(r, 0, 1) ? NAN : INFINITY; s.onceW = 0.0;
  } else if (p < 2220) {
    s.streamOn = !s.streamOn; hwlog("stream(%d)", s.streamOn ? 1 : 0);
  } else if (p < 2320) {
    s.streamPeriodUs = rng_range(r, 10, 700) * 1000u;
  } else if (p < 2400) {
    Step st = {0, OP_ESTOP, s.estop ? 0 : 1, 0, 0}; apply_step(s, st);
  } else if (p < 2500) {
    Step st = {0, OP_GATE, 0, 0, 0}; apply_step(s, st);
  } else if (p < 3500) {
    if (driveGate.state != DRIVE_ARMED && !s.autoArm) { Step st = {0, OP_GATE, 1, 0, 0}; apply_step(s, st); }
  } else if (p < 3700) {
    Step st = {0, OP_CMDRX, (int)rng_range(r, 0, 1), 0, 0}; apply_step(s, st);
  } else if (p < 3800) {
    Step st = {0, OP_RESET_ODOM, 0, 0, 0}; apply_step(s, st);
  } else if (p < 4500) {
    const int w = (int)rng_range(r, 0, 3);
    Step st = {0, OP_FREEZE, w, g_plant.frozen[w] ? 0.0 : 1.0, 0}; apply_step(s, st);
  } else if (p < 4700) {
    g_plant.noiseAmp = (int)rng_range(r, 0, 3);
  } else if (p < 4800) {
    static const uint32_t jumps[] = {19999, 20000, 480000, 499999, 500000, 500001,
                                     520000, 2000000, 30000000};
    s.jumpUs += jumps[rng_range(r, 0, 8)];
  } else if (p < 5000) {
    s.zeroDtTicks += (int)rng_range(r, 1, 8);
  } else if (p < 5050) {
    static const double dts[] = {0.0, -0.001, 1e-9, 0.02, 0.5, 0.50000001, 0.6};
    Step st = {0, OP_WC_DIRECT, 0, dts[rng_range(r, 0, 6)], 0}; apply_step(s, st);
  } else if (p < 5150) {
    s.fixedDt = rng_range(r, 0, 1) ? 0u : rng_range(r, 500, 3000);
  } else if (p < 5450) {
    const int w = (int)rng_range(r, 0, 3);
    Step st = {0, OP_PUSH, w, (double)((int32_t)rng_range(r, 0, 10000) - 5000), g_plant.pushed[w] ? 0.0 : 1.0};
    apply_step(s, st);
  }
}

static void deliver_inputs(Sim& s) {
  if (s.onceP) { s.onceP = false; send_cmd(s, s.onceV, s.onceW); }
  if (s.mode == MODE_LOOP && s.autoArm) {
    if (driveGate.state == DRIVE_ARMED) {
      s.autoArm = false;
      s.sendNow = true;
    } else {
      if (s.simUs - s.lastSendUs >= 50000u || s.sendNow) {
        s.lastSendUs = s.simUs; s.sendNow = false;
        send_cmd(s, 0.0, 0.0);
      }
      if (driveGate.state == DRIVE_READY && !isEstopActive()) send_enable(true);
      return;
    }
  }
  if (s.streamOn && (s.sendNow || s.simUs - s.lastSendUs >= s.streamPeriodUs)) {
    s.lastSendUs = s.simUs; s.sendNow = false;
    send_cmd(s, s.cmdV, s.cmdW);
  }
}

static void plant_step(Sim& s, uint32_t d) {
  for (int i = 0; i < 4; ++i) {
    const int p = g_plant.pwm[i];
    int64_t target = (p > PLANT_DEADBAND_PWM) ? (int64_t)(p - PLANT_DEADBAND_PWM) * PLANT_GAIN[i] : 0;
    if (g_plant.dirLevel[i] == LOW) target = -target;
    if (g_plant.frozen[i]) { g_plant.speed[i] = 0; }
    else if (g_plant.pushed[i]) {
      g_plant.speed[i] = g_plant.pushSpeed[i];
      g_plant.acc[i] += g_plant.speed[i] * (int64_t)d;
    } else {
      g_plant.speed[i] += (target - g_plant.speed[i]) / 4;
      g_plant.acc[i] += g_plant.speed[i] * (int64_t)d;
    }
    if (g_plant.noiseAmp > 0) {
      g_plant.noise[i] = (int32_t)rng_range(s.rng, 0, (uint32_t)(2 * g_plant.noiseAmp)) - g_plant.noiseAmp;
    }
  }
}

static char g_line[131072];
static size_t g_lineLen = 0;
static void L(const char* fmt, ...) {
  va_list ap; va_start(ap, fmt);
  int n = vsnprintf(g_line + g_lineLen, sizeof(g_line) - g_lineLen, fmt, ap);
  va_end(ap);
  if (n < 0 || (size_t)n >= sizeof(g_line) - g_lineLen) { fprintf(stderr, "line overflow\n"); exit(3); }
  g_lineLen += (size_t)n;
}
static uint64_t g_hash = 1469598103934665603ULL;
static bool g_trace = false;
static void flush_line() {
  for (size_t k = 0; k < g_lineLen; ++k) { g_hash ^= (unsigned char)g_line[k]; g_hash *= 1099511628211ULL; }
  g_hash ^= '\n'; g_hash *= 1099511628211ULL;
  if (g_trace) { fwrite(g_line, 1, g_lineLen, stdout); fputc('\n', stdout); }
  g_lineLen = 0;
}

static void emit(Sim& s) {
  if (g_hwOverflow) { fprintf(stderr, "hw log overflow\n"); exit(3); }
  g_hw[g_hwLen] = 0;
  L("t=%d us=%lu ms=%lu hw=[%s]", s.tick, (unsigned long)g_us, (unsigned long)g_ms, g_hw);
  L(" odomX=%a odomY=%a odomYaw=%a odomV=%a odomW=%a", odomX, odomY, odomYaw,
    odomLinearVelocity, odomAngularVelocity);
  L(" pwmMax=%d epoch=%lu rx=%d lastCmd=%lu gate=%u pubTs=%llu",
    appliedPwmMaxMagnitude, (unsigned long)appliedPwmEpoch, cmdVelReceived ? 1 : 0,
    (unsigned long)lastCmdVelMs, (unsigned)driveGate.state,
    (unsigned long long)lastPublishedTimestampNs);
  L(" tm.pwm=%lu tm.odom=%lu", (unsigned long)adapter_pwm_last(), (unsigned long)adapter_odom_last());
  for (int i = 0; i < 4; ++i) {
    WheelView v; adapter_view(i, v);
    L(" w%d.target=%a w%d.meas=%a w%d.filt=%a w%d.desired=%d w%d.applied=%d w%d.boost=%lu"
      " w%d.prevCount=%ld w%d.integ=%a w%d.prevErr=%a w%d.dfilt=%a",
      i, v.target, i, v.meas, i, v.filt, i, v.desired, i, v.applied, i, (unsigned long)v.boost,
      i, (long)v.prevCount, i, v.integ, i, v.prevErr, i, v.dfilt);
    // 커버리지 (관측값으로만)
    if (abs(v.applied) == START_BOOST_PWM[i] && (int32_t)(v.boost - millis()) > 0) s.cov[0]++;
    if (abs(v.desired) == MAX_CONTROL_PWM) s.cov[1]++;
    if (v.desired != 0 && abs(v.desired) == MIN_RUNNING_PWM[i]) s.cov[2]++;
    if (fabs(v.integ) == INTEGRAL_PWM_LIMIT / WHEEL_KI) s.cov[3]++;
    if (v.integ < -3.0) s.cov[4]++;
    if (v.applied - s.prevApplied[i] == PWM_RAMP_STEP || s.prevApplied[i] - v.applied == PWM_RAMP_STEP) s.cov[5]++;
    if (fabs(v.target) == MIN_EFFECTIVE_WHEEL_CMD) s.cov[6]++;
    if (fabs(v.target) == MAX_WHEEL_CMD) s.cov[7]++;
    if (v.target != v.target) s.cov[8]++;
    if ((v.target > 0 && s.prevTarget[i] < 0) || (v.target < 0 && s.prevTarget[i] > 0)) s.cov[9]++;
    if (v.applied != 0 && v.applied * v.desired < 0) s.cov[10]++;
    s.prevApplied[i] = v.applied; s.prevTarget[i] = v.target;
  }
  if (fabs(odomYaw) > 3.0) s.cov[11]++;
  flush_line();
  g_hwLen = 0;
}

static void run_tick(Sim& s) {
  // 시나리오 단계 (loop 모드는 첫 무장 이후 시각 기준)
  if (s.t0Set && s.steps) {
    while (s.steps[s.stepIndex].op != OP_END &&
           (uint64_t)s.steps[s.stepIndex].atMs * 1000u <= s.simUs - s.t0Us) {
      apply_step(s, s.steps[s.stepIndex]);
      s.stepIndex++;
    }
  }
  if (s.fuzz && s.t0Set) fuzz_events(s);

  g_estopPinLevel = s.estop ? HIGH : LOW;   // ESTOP_ACTIVE_LOW=false 기준; 아래에서 확인
  if (s.mode == MODE_LOOP) {
    checkSafety();
    deliver_inputs(s);                       // spin_some 안의 콜백들
    rearmGateArmBarrierStart(&driveGate, millis());
    checkSafety();
  } else {
    deliver_inputs(s);
  }
  updateAppliedPwmEpoch();
  updateMotorOutputs();
  updateOdometry();

  if (!s.t0Set && driveGate.state == DRIVE_ARMED) { s.t0Set = true; s.t0Us = s.simUs; }
  emit(s);

  uint32_t d;
  if (s.zeroDtTicks > 0) { s.zeroDtTicks--; d = 0; }
  else if (s.fixedDt) d = s.fixedDt;
  else d = rng_range(s.rng, s.jitMin, s.jitMax);
  plant_step(s, d);
  d += s.jumpUs; s.jumpUs = 0;
  advance_clock(d);
  s.simUs += d;
  s.tick++;
}

// ── 이름 붙은 시나리오 ──────────────────────────────────────────────────────
#define S(t, op, i, a, b) {t, op, i, a, b}
#define END {0, OP_END, 0, 0, 0}
static const Step SC_STANDSTILL[] = { S(0, OP_CMD, 0, 0.0, 0.0), END };
static const Step SC_STRAIGHT[] = { S(100, OP_CMD, 0, 0.15, 0.0), END };
static const Step SC_STOP[] = { S(0, OP_CMD, 0, 0.15, 0.0), S(2000, OP_CMD, 0, 0.0, 0.0), S(2600, OP_CMD, 0, 0.1, 0.0), S(2800, OP_CMD, 0, 0.0, 0.0), END };
static const Step SC_SPIN[] = { S(0, OP_CMD, 0, 0.0, 0.83), S(6000, OP_CMD, 0, 0.0, -0.83), END };
static const Step SC_CURVE[] = { S(0, OP_CMD, 0, 0.12, 0.3), S(3000, OP_CMD, 0, 0.08, -0.4), S(5000, OP_CMD, 0, -0.1, 0.25), END };
static const Step SC_LIMIT[] = { S(0, OP_CMD, 0, 0.20, 0.83), S(3000, OP_CMD, 0, -0.2, -0.83), S(5000, OP_CMD, 0, 0.2, -0.83), S(7000, OP_CMD, 0, 0.19, 0.5), END };
static const Step SC_REVERSAL[] = { S(0, OP_CMD, 0, 0.15, 0.0), S(2000, OP_CMD, 0, -0.15, 0.0), S(4000, OP_CMD, 0, 0.1, 0.0), S(4030, OP_CMD, 0, -0.1, 0.0), S(4070, OP_CMD, 0, 0.1, 0.0), S(5000, OP_CMD, 0, -0.05, 0.2), END };
static const Step SC_SUBMIN[] = { S(0, OP_CMD, 0, 0.01, 0.0), S(2000, OP_CMD, 0, -0.005, 0.0), S(3500, OP_CMD, 0, 0.0199, 0.0), S(4500, OP_CMD, 0, 0.02, 0.0), S(5500, OP_CMD, 0, 0.0, 0.03), S(6500, OP_CMD, 0, 0.012, 0.03), END };
static const Step SC_DEADBAND[] = { S(0, OP_CMD, 0, 0.002, 0.0), S(1000, OP_CMD, 0, 0.0019999, 0.0), S(2000, OP_CMD, 0, 0.0020001, 0.0), S(3000, OP_CMD, 0, -0.002, 0.0), S(4000, OP_CMD, 0, 0.001, 0.006), S(5000, OP_CMD, 0, 0.0, 0.00646), S(6000, OP_CMD, 0, -0.0, 0.0), S(6500, OP_CMD, 0, 0.0021, 0.0), END };
static const Step SC_STALL[] = { S(0, OP_CMD, 0, 0.2, 0.0), S(1000, OP_FREEZE, 2, 1.0, 0), S(3000, OP_CMD, 0, 0.2, 0.83), S(3500, OP_FREEZE, 3, 1.0, 0), S(7000, OP_FREEZE, 2, 0.0, 0), S(7500, OP_FREEZE, 3, 0.0, 0), S(9000, OP_FREEZE, 0, 1.0, 0), S(9000, OP_CMD, 0, 0.05, 0.0), S(11000, OP_FREEZE, 0, 0.0, 0), END };
static const Step SC_NOISE[] = { S(0, OP_NOISE, 2, 0, 0), S(500, OP_CMD, 0, 0.05, 0.0), S(2500, OP_CMD, 0, 0.0, 0.2), S(4000, OP_NOISE, 3, 0, 0), S(4000, OP_CMD, 0, 0.0, 0.0), END };
static const Step SC_ESTOP[] = { S(0, OP_CMD, 0, 0.15, 0.2), S(1500, OP_ESTOP, 1, 0, 0), S(2500, OP_ESTOP, 0, 0, 0), S(2600, OP_GATE, 1, 0, 0), S(2600, OP_CMD, 0, 0.12, 0.0), S(5000, OP_ESTOP, 1, 0, 0), S(5010, OP_ESTOP, 0, 0, 0), END };
static const Step SC_CMDRX[] = { S(0, OP_CMD, 0, 0.15, 0.0), S(1500, OP_CMDRX, 0, 0, 0), S(2500, OP_CMDRX, 1, 0, 0), S(3000, OP_STREAM, 0, 0, 0), S(4000, OP_STREAM, 1, 0, 0), S(4000, OP_CMDRX, 1, 0, 0), END };
static const Step SC_GATE[] = { S(0, OP_CMD, 0, 0.12, 0.0), S(1500, OP_GATE, 0, 0, 0), S(1500, OP_CMD, 0, 0.0, 0.0), S(2500, OP_GATE, 1, 0, 0), S(2500, OP_CMD, 0, 0.12, 0.1), S(4500, OP_GATE, 0, 0, 0), S(4500, OP_CMD, 0, 0.0, 0.0), S(4520, OP_GATE, 1, 0, 0), S(4520, OP_CMD, 0, -0.12, 0.0), END };
static const Step SC_ABNORMAL_DT[] = { S(0, OP_CMD, 0, 0.15, 0.0), S(1500, OP_JUMP_US, 600000, 0, 0), S(2500, OP_JUMP_US, 480000, 0, 0), S(3500, OP_ZERO_DT, 12, 0, 0), S(4000, OP_JUMP_US, 499990, 0, 0), S(5000, OP_JUMP_US, 2000000, 0, 0), S(8000, OP_FIXED_DT, 20000, 0, 0), S(8300, OP_FIXED_DT, 0, 0, 0), S(8500, OP_STREAM, 0, 0, 0), S(9300, OP_STREAM, 1, 0, 0), S(9300, OP_CMDRX, 1, 0, 0), END };
static const Step SC_DT_GUARD[] = { S(0, OP_CMD, 0, 0.15, 0.1), S(1000, OP_WC_DIRECT, 0, 0.0, 0), S(1100, OP_CMDRX, 1, 0, 0), S(1100, OP_CMD, 0, 0.15, 0.1), S(1500, OP_WC_DIRECT, 0, -0.001, 0), S(1600, OP_CMDRX, 1, 0, 0), S(1600, OP_CMD, 0, 0.15, 0.1), S(2000, OP_WC_DIRECT, 0, 0.5, 0), S(2100, OP_WC_DIRECT, 0, 0.50000001, 0), S(2200, OP_CMDRX, 1, 0, 0), S(2200, OP_CMD, 0, 0.15, 0.1), S(2500, OP_WC_DIRECT, 0, 1e-9, 0), S(2600, OP_WC_DIRECT, 0, NAN, 0), S(3000, OP_CMD, 0, 0.1, 0.0), END };
static const Step SC_RESET_ODOM[] = { S(0, OP_CMD, 0, 0.15, 0.3), S(2000, OP_RESET_ODOM, 0, 0, 0), S(2100, OP_CMDRX, 1, 0, 0), S(2100, OP_CMD, 0, 0.1, 0.0), S(4000, OP_RESET_ODOM, 0, 0, 0), S(4001, OP_CMDRX, 1, 0, 0), S(4001, OP_CMD, 0, 0.1, -0.2), END };
static const Step SC_WRAP[] = { S(0, OP_CMD, 0, 0.15, 0.0), S(300, OP_CMD, 0, -0.15, 0.0), S(600, OP_CMD, 0, 0.15, 0.1), S(900, OP_CMD, 0, -0.1, 0.0), S(1200, OP_CMD, 0, 0.1, 0.0), S(1500, OP_CMD, 0, -0.1, 0.0), S(1800, OP_CMD, 0, 0.1, 0.0), S(2100, OP_CMD, 0, -0.1, 0.0), S(2400, OP_CMD, 0, 0.1, 0.0), S(2700, OP_CMD, 0, -0.1, 0.3), S(3000, OP_CMD, 0, 0.1, 0.0), END };
static const Step SC_FIXED_1MS[] = { S(0, OP_FIXED_DT, 1000, 0, 0), S(0, OP_CMD, 0, 0.1, 0.0), S(1500, OP_CMD, 0, -0.1, 0.2), S(3000, OP_FIXED_DT, 2000, 0, 0), S(3000, OP_CMD, 0, 0.05, 0.0), END };
static const Step SC_EXTREME[] = { S(0, OP_CMD, 0, 5.0, 50.0), S(800, OP_CMD, 0, -5.0, -50.0), S(1600, OP_CMD, 0, 1e-300, 0.0), S(2000, OP_CMD, 0, 0.1, 0.0), S(2400, OP_ONCE, 0, NAN, 0.0), S(2500, OP_GATE, 1, 0, 0), S(2500, OP_CMD, 0, 0.1, 0.0), S(3000, OP_ONCE, 0, INFINITY, 0.0), S(3100, OP_GATE, 1, 0, 0), S(3100, OP_CMD, 0, -0.1, -INFINITY), S(3500, OP_GATE, 1, 0, 0), S(3500, OP_CMD, 0, 0.1, 0.0), END };
// 적분 클램프(±8)·unwindHigh·unwindLow·최소구동 바닥(15)을 실제로 밟는 시나리오.
//  FR 을 뒤로 끌어(오차 +0.5) 적분 +8 까지, RR 을 앞으로 밀어(오차 −0.4) 적분 −8 까지 쌓은 뒤
//  ① 목표를 0.3 으로 올리고 FR 을 풀면 PWM 160 으로 0.3 을 넘는 순간 candidate>160 & 오차<0 (unwindHigh)
//  ② 목표를 0.02 로 낮추고 RR 을 풀면 FF 30 + I(−40) < 15 & 오차>0 (unwindLow, 바닥 15)
static const Step SC_WINDUP[] = { S(0, OP_CMD, 0, 0.2, 0.0), S(500, OP_PUSH, 2, -2213, 1), S(500, OP_PUSH, 3, 4426, 1),
  S(21000, OP_CMD, 0, 0.2, 0.83), S(21500, OP_PUSH, 2, 0, 0), S(23000, OP_CMD, 0, 0.02, 0.0), S(23000, OP_PUSH, 3, 0, 0),
  S(25000, OP_PUSH, 0, 4426, 1), S(25000, OP_CMD, 0, 0.3, 0.0), S(27000, OP_CMD, 0, 0.02, 0.0), S(29000, OP_PUSH, 0, 0, 0), END };
// 방향 전환 직후(제어기가 desired 를 갱신하기 전) 시계가 0.5 s 넘게 뛰면, 부스트가 끝난 뒤
// 램프가 "반대 부호 applied ↔ 옛 부호 desired" 를 만난다 (rampToward 의 부호 반대 분기).
//  순서: 점프(T) → 다음 tick 에 반대 명령 + 또 점프 (그 tick 의 odom 은 dt>0.5 로 제어기를 건너뜀)
static const Step SC_REV_JUMP[] = { S(0, OP_CMD, 0, 0.15, 0.0),
  S(1000, OP_JUMP_US, 600000, 0, 0), S(1600, OP_CMD, 0, -0.15, 0.0), S(1600, OP_JUMP_US, 600000, 0, 0),
  S(3500, OP_JUMP_US, 600000, 0, 0), S(4100, OP_CMD, 0, 0.15, 0.0), S(4100, OP_JUMP_US, 600000, 0, 0),
  S(6000, OP_JUMP_US, 700000, 0, 0), S(6700, OP_CMD, 0, -0.1, 0.3), S(6700, OP_JUMP_US, 520000, 0, 0),
  S(9000, OP_CMD, 0, 0.0, 0.0), END };
static const Step SC_UNIT[] = { S(0, OP_UNIT, 0, 0, 0), S(100, OP_CMD, 0, 0.1, 0.0), S(500, OP_UNIT, 0, 0, 0), END };

struct Scenario { const char* name; const Step* steps; uint32_t durationMs; uint32_t us0; uint32_t ms0; uint32_t jitMin; uint32_t jitMax; };
static const Scenario SCENARIOS[] = {
  {"standstill", SC_STANDSTILL, 3000, 0, 0, 1000, 3000},
  {"straight_start", SC_STRAIGHT, 4000, 0, 0, 1000, 3000},
  {"stop", SC_STOP, 3500, 0, 0, 1000, 3000},
  {"spin", SC_SPIN, 10000, 0, 0, 1000, 3000},
  {"curve", SC_CURVE, 7000, 0, 0, 1000, 3000},
  {"wheel_limit_scale", SC_LIMIT, 9000, 0, 0, 1000, 3000},
  {"reversal", SC_REVERSAL, 7000, 0, 0, 1000, 3000},
  {"sub_min_raise", SC_SUBMIN, 8000, 0, 0, 1000, 3000},
  {"near_deadband", SC_DEADBAND, 7500, 0, 0, 1000, 3000},
  {"stalled_wheel", SC_STALL, 12000, 0, 0, 1000, 3000},
  {"encoder_noise", SC_NOISE, 6000, 0, 0, 1000, 3000},
  {"estop_mid_run", SC_ESTOP, 7000, 0, 0, 1000, 3000},
  {"cmdvel_received_false", SC_CMDRX, 6000, 0, 0, 1000, 3000},
  {"output_not_allowed", SC_GATE, 7000, 0, 0, 1000, 3000},
  {"abnormal_dt", SC_ABNORMAL_DT, 11000, 0, 0, 1000, 3000},
  {"controller_dt_guard", SC_DT_GUARD, 4000, 0, 0, 1000, 3000},
  {"reset_odometry", SC_RESET_ODOM, 6000, 0, 0, 1000, 3000},
  {"wrap_micros", SC_WRAP, 4000, 4294967295u - 2500000u, 5000, 1000, 3000},
  {"wrap_millis", SC_WRAP, 4000, 0, 4294967295u - 2600u, 1000, 3000},
  {"wrap_both", SC_WRAP, 4000, 4294967295u - 2700000u, 4294967295u - 2700u, 1000, 3000},
  {"fixed_dt_1ms", SC_FIXED_1MS, 5000, 0, 0, 1000, 1000},
  {"extreme_and_nonfinite", SC_EXTREME, 4500, 0, 0, 1000, 3000},
  {"unit_calls", SC_UNIT, 1000, 0, 0, 1000, 3000},
  {"windup_paths", SC_WINDUP, 31000, 0, 0, 3000, 5000},
  {"reversal_then_clock_jump", SC_REV_JUMP, 10000, 0, 0, 1000, 3000},
};

static void sim_init(Sim& s, int mode, uint64_t seed, uint32_t us0, uint32_t ms0) {
  memset(&s, 0, sizeof(s));
  memset(&g_plant, 0, sizeof(g_plant));
  for (int i = 0; i < 4; ++i) g_plant.dirLevel[i] = HIGH;
  s.mode = mode;
  s.rng = seed * 0x9E3779B97F4A7C15ULL + 0x1234567ULL;
  if (s.rng == 0) s.rng = 1;
  g_us = us0; g_ms = ms0; g_sub = 0;
  g_estopPinNumber = ESTOP_PIN;
  s.streamOn = true; s.streamPeriodUs = 50000; s.jitMin = 1000; s.jitMax = 3000;
  // setup() 과 같은 초기화 (두 판 같은 글자)
  estopDebounceInit(&estopFilter, false, millis());
  rearmGateInit(&driveGate);
  resetOdometry();
  if (mode == MODE_DIRECT) {
    driveGate.state = DRIVE_ARMED;
    cmdVelReceived = true;
    s.t0Set = true; s.t0Us = 0;
  } else {
    s.autoArm = true;
  }
}

static void print_cov(const Sim& s) {
  printf("cov boost=%llu satHigh=%llu floor=%llu integLimit=%llu integNeg=%llu ramp=%llu minRaise=%llu"
         " wheelCap=%llu nanTarget=%llu signFlip=%llu appliedVsDesiredOpp=%llu yawNearPi=%llu\n",
         s.cov[0], s.cov[1], s.cov[2], s.cov[3], s.cov[4], s.cov[5], s.cov[6], s.cov[7],
         s.cov[8], s.cov[9], s.cov[10], s.cov[11]);
}

// 인자는 표준입력 한 줄로 받는다 — qemu-arm 세미호스팅에서는 argv 가 안 넘어온다.
static int read_args(char** out, int maxArgs) {
  static char buf[512];
  int n = 1; out[0] = (char*)"prog";
  if (!fgets(buf, sizeof(buf), stdin)) return n;
  char* p = buf;
  while (*p && n < maxArgs) {
    while (*p == ' ' || *p == '\n' || *p == '\r' || *p == '\t') *p++ = 0;
    if (!*p) break;
    out[n++] = p;
    while (*p && *p != ' ' && *p != '\n' && *p != '\r' && *p != '\t') ++p;
  }
  return n;
}

int main(int, char**) {
  char* argv[16];
  const int argc = read_args(argv, 16);
  if (ESTOP_ACTIVE_LOW) { fprintf(stderr, "fixture assumes ESTOP_ACTIVE_LOW=false\n"); return 3; }
  if (argc >= 2 && strcmp(argv[1], "list") == 0) {
    for (size_t k = 0; k < sizeof(SCENARIOS) / sizeof(SCENARIOS[0]); ++k) printf("%s\n", SCENARIOS[k].name);
    return 0;
  }
  if (argc < 5) { fprintf(stderr, "usage: prog direct|loop <scenario|fuzz> <seed> trace|hash [ticks]\n"); return 64; }
  const int mode = strcmp(argv[1], "loop") == 0 ? MODE_LOOP : MODE_DIRECT;
  const char* name = argv[2];
  const uint64_t seed = strtoull(argv[3], 0, 10);
  g_trace = strcmp(argv[4], "trace") == 0;
  Sim s;
  // pins (한 번)
  for (int i = 0; i < 4; ++i) { WheelView v; adapter_view(i, v); L("%spin%d=%u/%u", i ? " " : "pins ", i, v.pwmPin, v.dirPin); }
  flush_line();

  if (strcmp(name, "fuzz") == 0) {
    const int ticks = argc > 5 ? atoi(argv[5]) : 2000;
    uint64_t r = seed * 0xD1B54A32D192ED03ULL + 7;
    if (r == 0) r = 1;
    uint32_t us0 = (uint32_t)rng_next(r), ms0 = (uint32_t)rng_next(r);
    const uint32_t kind = rng_range(r, 0, 9);
    if (kind < 3) { us0 = 4294967295u - rng_range(r, 0, 5000000); }
    else if (kind < 5) { ms0 = 4294967295u - rng_range(r, 0, 5000); }
    else if (kind < 6) { us0 = 4294967295u - rng_range(r, 0, 3000000); ms0 = 4294967295u - rng_range(r, 0, 3000); }
    else if (kind < 8) { us0 = rng_range(r, 0, 100000); ms0 = rng_range(r, 0, 100); }
    sim_init(s, mode, seed, us0, ms0);
    s.fuzz = true;
    s.jitMin = rng_range(r, 0, 2000);
    s.jitMax = s.jitMin + rng_range(r, 0, 4000);
    for (int t = 0; t < ticks; ++t) run_tick(s);
  } else {
    const Scenario* sc = 0;
    for (size_t k = 0; k < sizeof(SCENARIOS) / sizeof(SCENARIOS[0]); ++k) if (strcmp(SCENARIOS[k].name, name) == 0) sc = &SCENARIOS[k];
    if (!sc) { fprintf(stderr, "unknown scenario %s\n", name); return 65; }
    sim_init(s, mode, seed, sc->us0, sc->ms0);
    s.steps = sc->steps; s.jitMin = sc->jitMin; s.jitMax = sc->jitMax;
    const int maxTicks = 2000000;
    while (!(s.t0Set && s.simUs - s.t0Us >= (uint64_t)sc->durationMs * 1000u)) {
      run_tick(s);
      if (s.tick > maxTicks) { fprintf(stderr, "never armed / too long\n"); return 4; }
    }
  }
  if (!g_trace) printf("ticks=%d hash=%016llx\n", s.tick, (unsigned long long)g_hash);
  else printf("ticks=%d\n", s.tick);
  print_cov(s);
  return 0;
}
"""


def build_source(kind, ino_text):
    pieces = extract_drive_pieces(ino_text, kind)
    parts = [PRELUDE, STUBS]
    parts.extend(pieces)
    parts.append(TAIL_STUBS)
    parts.append(ADAPTER_ORIG if kind == "orig" else ADAPTER_NEW)
    parts.append(DRIVER)
    return "\n\n".join(parts) + "\n"


# ════════════════════════════════════════════════════════════════════════════
# 3. 빌드·실행 (host / ARM-qemu)
# ════════════════════════════════════════════════════════════════════════════
HOST_FLAGS = ["-std=gnu++14", "-Wall", "-Wextra", "-Werror",
              "-Wno-unused-function", "-Wno-unused-variable",
              "-Wno-unused-const-variable", "-Wno-unused-but-set-variable"]


def teensy_gxx():
    if not os.path.isdir(TEENSY_TOOLCHAIN_GLOB):
        return None
    for entry in sorted(os.listdir(TEENSY_TOOLCHAIN_GLOB)):
        candidate = os.path.join(TEENSY_TOOLCHAIN_GLOB, entry, "arm", "bin",
                                 "arm-none-eabi-g++")
        if os.path.isfile(candidate):
            return candidate
    return None


def host_has_fma():
    try:
        with open("/proc/cpuinfo") as handle:
            return re.search(r"\bfma\b", handle.read()) is not None
    except OSError:
        return False


class Platform:
    def __init__(self, name, compile_cmd, run_prefix):
        self.name = name
        self.compile_cmd = compile_cmd     # list, -o / src 는 붙인다
        self.run_prefix = run_prefix       # list


def platforms(want_arm, want_fma):
    result = []
    if shutil.which("g++") is None:
        raise ProbeError("g++ 가 없다 (sudo apt install g++)")
    result.append(Platform("host-O1", ["g++", "-O1"] + HOST_FLAGS, []))
    if want_fma and host_has_fma():
        result.append(Platform("host-O2-fma",
                               ["g++", "-O2", "-mfma", "-ffp-contract=fast"] + HOST_FLAGS, []))
    if want_arm:
        gxx = teensy_gxx()
        qemu = shutil.which("qemu-arm-static") or shutil.which("qemu-arm")
        if gxx and qemu:
            result.append(Platform(
                "teensy-gcc-Os-cortex-m7(qemu)",
                [gxx, "-Os", "-std=gnu++14", "-mthumb", "-mcpu=cortex-m7",
                 "-mfloat-abi=hard", "-mfpu=fpv5-d16", "-fno-exceptions", "-fno-rtti",
                 "-fno-threadsafe-statics", "-felide-constructors", "-fpermissive",
                 "-Wall", "--specs=rdimon.specs"],
                [qemu, "-cpu", "cortex-m7"]))
        else:
            print("  [알림] ARM 검사 생략 — Teensy 툴체인 또는 qemu-arm 이 없다")
    return result


def fused_multiply_add_count(platform, binary):
    """실행 파일 안의 FMA 명령 수 (정보용 — 축약이 실제로 일어났는지)."""
    compiler = platform.compile_cmd[0]
    objdump = (compiler[:-len("g++")] + "objdump") if compiler.endswith("g++") else "objdump"
    if compiler == "g++":
        objdump = "objdump"
    if shutil.which(objdump) is None and not os.path.isfile(objdump):
        return None
    done = subprocess.run([objdump, "-d", binary], capture_output=True, text=True)
    if done.returncode != 0:
        return None
    return len(re.findall(r"\b(vfn?m[as]\.f64|vfn?madd\w*sd|vfn?msub\w*sd)\b", done.stdout))


def compile_program(platform, source, include_dir, workdir, tag, extra=()):
    src_path = os.path.join(workdir, tag + ".cpp")
    with open(src_path, "w", encoding="utf-8") as handle:
        handle.write(source)
    binary = os.path.join(workdir, tag + ".bin")
    command = list(platform.compile_cmd) + list(extra) + ["-I", include_dir, "-o", binary, src_path]
    built = subprocess.run(command, capture_output=True, text=True)
    if built.returncode != 0:
        raise ProbeError("[%s] %s 컴파일 실패\n%s" % (platform.name, tag, built.stderr[-6000:]))
    return binary


def run_program(platform, binary, args, timeout=600):
    done = subprocess.run(list(platform.run_prefix) + [binary],
                          input=" ".join(args) + "\n",
                          capture_output=True, text=True, timeout=timeout)
    if done.returncode != 0:
        raise ProbeError("[%s] %s %s 비정상 종료 rc=%d\n%s"
                         % (platform.name, os.path.basename(binary), " ".join(args),
                            done.returncode, done.stderr[-3000:]))
    return done.stdout


def first_difference(text_a, text_b):
    """두 trace 의 첫 차이 (tick, 필드, 원본값, 새값)."""
    lines_a, lines_b = text_a.splitlines(), text_b.splitlines()
    for index in range(max(len(lines_a), len(lines_b))):
        line_a = lines_a[index] if index < len(lines_a) else "<없음>"
        line_b = lines_b[index] if index < len(lines_b) else "<없음>"
        if line_a == line_b:
            continue
        tick = re.match(r"t=(\d+)", line_a)
        tick = tick.group(1) if tick else "?(line %d)" % index
        tokens_a = dict(_tokens(line_a))
        tokens_b = dict(_tokens(line_b))
        for key in [k for k, _ in _tokens(line_a)] + [k for k, _ in _tokens(line_b)]:
            if tokens_a.get(key) != tokens_b.get(key):
                return {"tick": tick, "field": key,
                        "orig": _short(tokens_a.get(key)), "new": _short(tokens_b.get(key))}
        return {"tick": tick, "field": "(line)", "orig": _short(line_a), "new": _short(line_b)}
    return None


def _tokens(line):
    result = []
    for token in re.findall(r"(\S+?)=(\[[^\]]*\]|\S+)", line):
        result.append(token)
    return result


def _short(value, limit=300):
    if value is None:
        return "<없음>"
    return value if len(value) <= limit else value[:limit] + "..."


def hw_first_diff(orig_hw, new_hw):
    a = orig_hw.strip("[]").split(",")
    b = new_hw.strip("[]").split(",")
    for index in range(max(len(a), len(b))):
        x = a[index] if index < len(a) else "<없음>"
        y = b[index] if index < len(b) else "<없음>"
        if x != y:
            return "hw 호출 #%d: 원본 %s / 새 판 %s" % (index, x, y)
    return ""


# ════════════════════════════════════════════════════════════════════════════
# 4. 동치 검사 본체
# ════════════════════════════════════════════════════════════════════════════
class Suite:
    def __init__(self, platform, orig_bin, new_bin, seeds, ticks, jobs):
        self.platform = platform
        self.orig_bin = orig_bin
        self.new_bin = new_bin
        self.seeds = seeds
        self.ticks = ticks
        self.jobs = jobs

    def scenario_names(self):
        names_orig = run_program(self.platform, self.orig_bin, ["list"]).split()
        names_new = run_program(self.platform, self.new_bin, ["list"]).split()
        if names_orig != names_new:
            raise ProbeError("두 프로그램의 시나리오 목록이 다르다 (구동기 글자 불일치)")
        return names_orig

    def compare_named(self, mode, name, stop_on_fail=False):
        out_o = run_program(self.platform, self.orig_bin, [mode, name, "1", "trace"])
        out_n = run_program(self.platform, self.new_bin, [mode, name, "1", "trace"])
        ticks = _ticks_of(out_o)
        digest = hashlib.sha1(out_o.encode()).hexdigest()
        if out_o == out_n:
            return {"ok": True, "ticks": ticks, "cov": _cov_of(out_o), "digest": digest}
        diff = first_difference(out_o, out_n)
        if diff and diff["field"] == "hw":
            diff["detail"] = hw_first_diff(diff["orig"], diff["new"])
        return {"ok": False, "ticks": ticks, "diff": diff, "cov": _cov_of(out_o), "digest": digest}

    def compare_fuzz_seed(self, mode, seed):
        args = [mode, "fuzz", str(seed), "hash", str(self.ticks)]
        out_o = run_program(self.platform, self.orig_bin, args)
        out_n = run_program(self.platform, self.new_bin, args)
        if out_o == out_n:
            return {"ok": True, "ticks": _ticks_of(out_o), "cov": _cov_of(out_o)}
        targs = [mode, "fuzz", str(seed), "trace", str(self.ticks)]
        trace_o = run_program(self.platform, self.orig_bin, targs)
        trace_n = run_program(self.platform, self.new_bin, targs)
        diff = first_difference(trace_o, trace_n)
        if diff and diff["field"] == "hw":
            diff["detail"] = hw_first_diff(diff["orig"], diff["new"])
        return {"ok": False, "ticks": _ticks_of(out_o), "diff": diff, "seed": seed}

    def run_all(self, stop_on_first=False, verbose=True):
        """반환: (불일치 목록, 요약 행 목록, 총 tick)."""
        failures, rows, total_ticks = [], [], 0
        self.digests = {}
        names = self.scenario_names()
        jobs = [(mode, name) for name in names for mode in ("direct", "loop")]
        with concurrent.futures.ThreadPoolExecutor(self.jobs) as pool:
            futures = {pool.submit(self.compare_named, mode, name): (mode, name)
                       for mode, name in jobs}
            results = {}
            for future in concurrent.futures.as_completed(futures):
                results[futures[future]] = future.result()
        for mode, name in jobs:
            res = results[(mode, name)]
            self.digests[(mode, name)] = res.get("digest")
            total_ticks += res["ticks"]
            rows.append((name, mode, res["ticks"], 0 if res["ok"] else 1,
                         res.get("diff"), res.get("cov")))
            if not res["ok"]:
                failures.append(("%s/%s" % (name, mode), res["diff"]))
        if stop_on_first and failures:
            return failures, rows, total_ticks

        fuzz_jobs = [(mode, seed) for mode in ("direct", "loop")
                     for seed in range(1, self.seeds + 1)]
        fuzz_fail = {"direct": 0, "loop": 0}
        fuzz_ticks = {"direct": 0, "loop": 0}
        fuzz_cov = {"direct": None, "loop": None}
        with concurrent.futures.ThreadPoolExecutor(self.jobs) as pool:
            futures = {pool.submit(self.compare_fuzz_seed, mode, seed): (mode, seed)
                       for mode, seed in fuzz_jobs}
            for future in concurrent.futures.as_completed(futures):
                mode, seed = futures[future]
                res = future.result()
                fuzz_ticks[mode] += res["ticks"]
                if res.get("cov"):
                    fuzz_cov[mode] = _cov_add(fuzz_cov[mode], res["cov"])
                if not res["ok"]:
                    fuzz_fail[mode] += 1
                    failures.append(("fuzz/%s/seed%d" % (mode, seed), res["diff"]))
                    if stop_on_first:
                        for other in futures:
                            other.cancel()
                        break
        for mode in ("direct", "loop"):
            total_ticks += fuzz_ticks[mode]
            rows.append(("fuzz x%d seeds" % self.seeds, mode, fuzz_ticks[mode],
                         fuzz_fail[mode], None, fuzz_cov[mode]))
        return failures, rows, total_ticks


def _ticks_of(output):
    match = re.search(r"^ticks=(\d+)", output, re.M)
    if match is None:
        raise ProbeError("출력에 ticks= 가 없다")
    return int(match.group(1))


def _cov_of(output):
    match = re.search(r"^cov (.*)$", output, re.M)
    if match is None:
        return None
    return {k: int(v) for k, v in re.findall(r"(\w+)=(\d+)", match.group(1))}


def _cov_add(total, cov):
    if total is None:
        return dict(cov)
    return {k: total.get(k, 0) + cov.get(k, 0) for k in cov}


# ════════════════════════════════════════════════════════════════════════════
# 5. PeriodicTimer 검사 (구동 경로 밖 4개 + 순수 패턴)
# ════════════════════════════════════════════════════════════════════════════
def _head_until(body, end_regex, what):
    """함수 원문에서 여는 `{` 다음부터 end_regex 에 맞는 줄 끝까지 (원문 그대로)."""
    open_index = body.index("{") + 1
    match = re.search(end_regex, body[open_index:], re.M)
    if match is None:
        raise ProbeError("%s: 타이머 머리 끝(%s)을 못 찾았다" % (what, end_regex))
    snippet = body[open_index:open_index + match.end()]
    return snippet


def _wrap_head(name, snippet, tail=""):
    if snippet.count("return;") < 1:
        raise ProbeError("%s 머리에 return; 이 없다" % name)
    return ("bool %s() {%s\n%s  return true;\n}\n"
            % (name, snippet.replace("return;", "return false;"), tail))


def timer_program(orig_src, new_src):
    orig_funcs = {n: top_function(orig_src, n) for n in
                  ("publishDiagnostics", "publishFirmwareInfo", "periodicTimeSync",
                   "updateImu", "resetImuYaw")}
    new_funcs = {n: top_function(new_src, n) for n in
                 ("publishDiagnostics", "publishFirmwareInfo", "periodicTimeSync",
                  "updateImu", "resetImuYaw")}

    pieces = [PRELUDE, r"""
static uint32_t g_us = 0, g_ms = 0;
uint32_t millis() { return g_ms; }
uint32_t micros() { return g_us; }
bool imuInitialized = true;
bool gyroBiasCalibrated = true;
static double g_dt_o = -1, g_dt_n = -1;
"""]
    pieces.append(constants(orig_src))
    pieces.append(quoted_includes(new_src).replace('#include "rearm_gate.h"', "")
                  .replace('#include "drive_wiring.h"', "")
                  .replace('#include "estop_debounce.h"', "")
                  .replace('#include "runtime_guard.h"', "")
                  .replace('#include "link_stall_probe.h"', "")
                  .replace('#include "pid_controller.h"', "")
                  .replace('#include "wheel.h"', "")
                  .replace('#include "drive_base.h"', ""))
    # 원본 전역 (원문 줄)
    for name in ("lastDiagnosticMs", "lastFirmwareInfoMs", "lastTimeSyncMs", "previousImuUs"):
        pieces.append(unique(orig_src, r"^uint32_t %s = 0;$" % name, name).group(0))
    pieces.append(unique(orig_src, r"^double imuYawUnwrappedRad = 0\.0;$", "imuYaw").group(0))
    # 새 판 타이머 선언 (원문 줄)
    timer_lines = re.findall(r"^PeriodicTimer \w+\(\w+\);.*$", new_src, re.M)
    if len(timer_lines) != 6:
        raise ProbeError("PeriodicTimer 선언이 6개가 아니다 (%d)" % len(timer_lines))
    pieces.append("\n".join(timer_lines))

    # 원본 머리: 마지막 시각 대입 줄까지
    pieces.append(_wrap_head("o_diag", _head_until(orig_funcs["publishDiagnostics"],
                                                    r"^\s*lastDiagnosticMs = nowMs;$", "diag")))
    pieces.append(_wrap_head("o_info", _head_until(orig_funcs["publishFirmwareInfo"],
                                                    r"^\s*lastFirmwareInfoMs = nowMs;$", "info")))
    pieces.append(_wrap_head("o_sync", _head_until(orig_funcs["periodicTimeSync"],
                                                    r"^\s*lastTimeSyncMs = nowMs;$", "sync")))
    pieces.append(_wrap_head("o_imu", _head_until(orig_funcs["updateImu"],
                                                   r"^\s*previousImuUs = currentUs;$", "imu"),
                             "  g_dt_o = dt;\n"))
    # 새 판 머리: isDue 블록 닫는 } 까지 (IMU 는 dt 계산 줄까지)
    pieces.append(_wrap_head("n_diag", _head_until(new_funcs["publishDiagnostics"],
                                                    r"^\s*if \(!diagnosticTimer\.isDue\(nowMs\)\) \{\n\s*return;\n\s*\}$", "diag")))
    pieces.append(_wrap_head("n_info", _head_until(new_funcs["publishFirmwareInfo"],
                                                    r"^\s*if \(!firmwareInfoTimer\.isDue\(nowMs\)\) \{\n\s*return;\n\s*\}$", "info")))
    pieces.append(_wrap_head("n_sync", _head_until(new_funcs["periodicTimeSync"],
                                                    r"^\s*if \(!timeSyncTimer\.isDue\(nowMs\)\) \{\n\s*return;\n\s*\}$", "sync")))
    pieces.append(_wrap_head("n_imu", _head_until(new_funcs["updateImu"],
                                                   r"^\s*const double dt = static_cast<double>\(elapsedUs\) / 1000000\.0;$", "imu"),
                             "  g_dt_n = dt;\n"))
    # restart 경로: resetImuYaw 원문 (이름 바꿔 둘 다 넣는다) + setup 의 부팅 동기 줄
    pieces.append(orig_funcs["resetImuYaw"].replace("void resetImuYaw()", "void o_resetImuYaw()", 1))
    pieces.append(new_funcs["resetImuYaw"].replace("void resetImuYaw()", "void n_resetImuYaw()", 1))
    boot_o = unique(orig_src, r"^\s*lastTimeSyncMs = millis\(\);$", "부팅 lastTimeSyncMs").group(0)
    boot_n = unique(new_src, r"^\s*timeSyncTimer\.restart\(millis\(\)\);$", "부팅 restart").group(0)
    pieces.append("void o_boot() {\n%s\n}\nvoid n_boot() {\n%s\n}\n" % (boot_o, boot_n))
    pieces.append(r"""
static uint64_t rs = 88172645463325252ULL;
static uint64_t rnext() { rs ^= rs << 13; rs ^= rs >> 7; rs ^= rs << 17; return rs; }
static uint32_t rstep(uint32_t period) {
  switch (rnext() % 12) {
    case 0: return 0;
    case 1: return 1;
    case 2: return period - 1;
    case 3: return period;
    case 4: return period + 1;
    case 5: return (uint32_t)(rnext() % 50);
    case 6: return (uint32_t)(rnext() % (uint64_t)(period * 3u + 1u));
    case 7: return (uint32_t)rnext();                 // 거대 점프 (2^31 이상 포함)
    case 8: return 0x80000000u;
    default: return (uint32_t)(rnext() % (uint64_t)(period / 4u + 2u));
  }
}
// 순수 패턴 (원본 6곳 공통형) ↔ 클래스
static bool pattern(uint32_t& last, uint32_t period, uint32_t now) {
  if (now - last < period) return false;
  last = now; return true;
}
int main() {
  char buf[64] = "";
  const long iters = fgets(buf, sizeof(buf), stdin) ? atol(buf) : 0;
  if (iters <= 0) { fprintf(stderr, "iters?\n"); return 64; }
  unsigned long long mism = 0, fired_o = 0, fired_n = 0, restarts = 0, wraps = 0, steps = 0;
  long firstBad = -1; char firstWhat[64] = "";
  for (int run = 0; run < 64; ++run) {
    // 각 run 은 시작점을 달리한다 (0 근처 / 무작위 / 랩 직전)
    const uint32_t startKind = (uint32_t)(rnext() % 3);
    g_ms = startKind == 0 ? 0u : (startKind == 1 ? (uint32_t)rnext() : 4294967295u - (uint32_t)(rnext() % 100000));
    g_us = startKind == 0 ? 0u : (startKind == 1 ? (uint32_t)rnext() : 4294967295u - (uint32_t)(rnext() % 100000000));
    // 상태 초기화: 원본 전역 0, 새 판 타이머 0 (생성 직후와 같게)
    lastDiagnosticMs = lastFirmwareInfoMs = lastTimeSyncMs = previousImuUs = 0;
    diagnosticTimer.restart(0); firmwareInfoTimer.restart(0); timeSyncTimer.restart(0); imuTimer.restart(0);
    if (rnext() % 2) { o_boot(); n_boot(); }
    for (long k = 0; k < iters / 64; ++k) {
      const uint32_t sel = (uint32_t)(rnext() % 4);
      const uint32_t periods[4] = {DIAGNOSTIC_PERIOD_MS, FW_INFO_PERIOD_MS, TIME_SYNC_PERIOD_MS, IMU_PERIOD_US};
      const uint32_t before_ms = g_ms, before_us = g_us;
      if (sel == 3) g_us += rstep(periods[3]); else g_ms += rstep(periods[sel]);
      if (g_ms < before_ms || g_us < before_us) wraps++;
      steps++;
      if (rnext() % 997 == 0) { o_resetImuYaw(); n_resetImuYaw(); restarts++; }
      if (rnext() % 991 == 0) { o_boot(); n_boot(); restarts++; }
      bool a, b; const char* what;
      g_dt_o = g_dt_n = -1;
      switch (rnext() % 4) {
        case 0: a = o_diag(); b = n_diag(); what = "diag"; break;
        case 1: a = o_info(); b = n_info(); what = "info"; break;
        case 2: a = o_sync(); b = n_sync(); what = "sync"; break;
        default: a = o_imu(); b = n_imu(); what = "imu"; break;
      }
      fired_o += a; fired_n += b;
      const bool same = (a == b) && (g_dt_o == g_dt_n || (g_dt_o != g_dt_o && g_dt_n != g_dt_n)) &&
          lastDiagnosticMs == diagnosticTimer.getLastTime() &&
          lastFirmwareInfoMs == firmwareInfoTimer.getLastTime() &&
          lastTimeSyncMs == timeSyncTimer.getLastTime() &&
          previousImuUs == imuTimer.getLastTime();
      if (!same) { if (firstBad < 0) { firstBad = (long)steps; snprintf(firstWhat, sizeof firstWhat, "%s", what); } mism++; }
    }
  }
  // 순수 패턴 ↔ 클래스: 극단 주기 포함
  unsigned long long pmism = 0, pfire = 0;
  const uint32_t pers[] = {0u, 1u, 2u, 20u, 1000u, 20000u, 0x7FFFFFFFu, 0x80000000u, 0xFFFFFFFFu};
  for (size_t pi = 0; pi < sizeof(pers) / sizeof(pers[0]); ++pi) {
    PeriodicTimer t(pers[pi]); uint32_t last = 0; uint32_t now = (uint32_t)rnext();
    for (long k = 0; k < 200000; ++k) {
      now += rstep(pers[pi] ? pers[pi] : 3u);
      if (rnext() % 1000 == 0) { t.restart(now); last = now; }
      const uint32_t el_o = now - last, el_n = t.elapsed(now);
      const bool a = pattern(last, pers[pi], now), b = t.isDue(now);
      pfire += a;
      if (a != b || el_o != el_n || last != t.getLastTime()) pmism++;
    }
  }
  printf("timers steps=%llu fired_orig=%llu fired_new=%llu restarts=%llu wraps=%llu mismatches=%llu first_bad_step=%ld first_bad_site=%s\n",
         steps, fired_o, fired_n, restarts, wraps, mism, firstBad, firstWhat);
  printf("pattern mismatches=%llu fired=%llu\n", pmism, pfire);
  return 0;
}
""")
    return "\n\n".join(pieces) + "\n"


def run_timer_check(platform, orig_src, new_src, include_dir, workdir):
    source = timer_program(orig_src, new_src)
    binary = compile_program(platform, source, include_dir, workdir, "timers")
    out = run_program(platform, binary, ["2000000"])
    m1 = re.search(r"mismatches=(\d+)", out.splitlines()[0])
    m2 = re.search(r"mismatches=(\d+)", out.splitlines()[1])
    if not m1 or not m2:
        raise ProbeError("타이머 검사 출력 해석 실패:\n" + out)
    ok = int(m1.group(1)) == 0 and int(m2.group(1)) == 0
    return ok, out.strip()


# ════════════════════════════════════════════════════════════════════════════
# 6. 헤더 해시 · 구조 검사
# ════════════════════════════════════════════════════════════════════════════
def sha256(path):
    with open(path, "rb") as handle:
        return hashlib.sha256(handle.read()).hexdigest()


def header_hash_check(new_dir):
    rows, ok = [], True
    for name in SAFETY_HEADERS:
        a = sha256(os.path.join(ORIG_DIR, name))
        b = sha256(os.path.join(new_dir, name))
        rows.append((name, a, b, a == b))
        ok = ok and a == b
    return ok, rows


def _strip_comments(text):
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    text = re.sub(r"//[^\n]*", "", text)
    return re.sub(r"\s+", " ", text).strip()


#: 이 시험이 실행으로 직접 비교하는 함수 (바뀌어도 된다 — 동작으로 증명)
EXECUTED_CHANGED = {"stopAllMotors", "applySkidSteerCommand", "updateWheelControllers",
                    "updateMotorOutputs", "resetOdometry", "updateOdometry"}
#: 타이머 검사가 머리를 실행 비교하는 함수 + 정규화로 증명하는 함수
TIMER_CHANGED = {"publishDiagnostics", "publishFirmwareInfo", "periodicTimeSync",
                 "updateImu", "resetImuYaw", "setup"}
#: 이 중 하나라도 0 이면 시나리오가 그 경로를 못 밟은 것 = 시험 실패
REQUIRED_COVERAGE = ["boost", "satHigh", "floor", "integLimit", "integNeg", "ramp",
                     "minRaise", "wheelCap", "nanTarget", "signFlip",
                     "appliedVsDesiredOpp", "yawNearPi"]
REMOVED_OK = {"clampDouble", "resetWheelControllerState", "resetAllWheelControllers",
              "wheelVelocityToFeedforwardPwm", "setTargetWheelVelocity",
              "movePwmTowardTarget"}


def _normalize_orig_timer_sites(body):
    """원본 타이머 머리 3곳 + setup 줄을 새 판 표기로 기계 치환 (정규화 비교용)."""
    mapping = {"lastDiagnosticMs": "diagnosticTimer", "lastFirmwareInfoMs": "firmwareInfoTimer",
               "lastTimeSyncMs": "timeSyncTimer"}
    for var, timer in mapping.items():
        body = re.sub(r"if \(nowMs - %s < \w+\) \{\s*return;\s*\}\s*%s = nowMs;" % (var, var),
                      "if (!%s.isDue(nowMs)) { return; }" % timer, body)
    body = body.replace("lastTimeSyncMs = millis();", "timeSyncTimer.restart(millis());")
    body = body.replace("bootSyncRc == RMW_RET_OK, lastTimeSyncMs)",
                        "bootSyncRc == RMW_RET_OK, timeSyncTimer.getLastTime())")
    for wheel in ("FL", "RL", "FR", "RR"):
        body = body.replace("appliedPwm[%s]" % wheel, "drive.wheel(%s).getAppliedPwm()" % wheel)
    body = body.replace("previousImuUs = micros();", "imuTimer.restart(micros());")
    return body


def structure_check(orig_src, new_src):
    """두 .ino 의 0열 함수 전부를 비교. 예상 밖 변경이 있으면 실패."""
    fo, fn = all_top_functions(orig_src), all_top_functions(new_src)
    removed = set(fo) - set(fn)
    added = set(fn) - set(fo)
    changed = {n for n in set(fo) & set(fn)
               if _strip_comments(fo[n]) != _strip_comments(fn[n])}
    problems = []
    if removed != REMOVED_OK:
        problems.append("삭제된 함수 집합이 예상과 다르다: %s" % sorted(removed ^ REMOVED_OK))
    if added:
        problems.append("새로 생긴 0열 함수: %s" % sorted(added))
    unexpected = changed - EXECUTED_CHANGED - TIMER_CHANGED
    if unexpected:
        problems.append("이 시험이 다루지 않는 함수가 바뀌었다: %s" % sorted(unexpected))
    normalized = {}
    for name in ("publishDiagnostics", "publishFirmwareInfo", "periodicTimeSync",
                 "setup", "resetImuYaw"):
        if name in changed:
            same = _strip_comments(_normalize_orig_timer_sites(fo[name])) == _strip_comments(fn[name])
            normalized[name] = same
            if not same:
                problems.append("%s: 정규화(타이머·getter 치환) 뒤에도 원본과 다르다" % name)
    # 상수 블록은 같아야 한다 (두 프로그램이 같은 숫자로 도는지)
    if constants(orig_src) != constants(new_src):
        problems.append("static const 상수 블록이 두 .ino 에서 다르다")
    return problems, {"removed": sorted(removed), "added": sorted(added),
                      "changed": sorted(changed), "normalized": normalized,
                      "functions_compared": len(set(fo) & set(fn))}


# ════════════════════════════════════════════════════════════════════════════
# 7. 변이 검사
# ════════════════════════════════════════════════════════════════════════════
#: (id, 파일, 바꿀 원문, 바꿀 글자, 설명, 기대) — 기대 'kill' 은 반드시 잡혀야 한다.
#: 'equiv' 는 의미상 등가 변이(어떤 시험으로도 못 잡는 것)로 예상 — 정직 보고용.
MUTATIONS = [
    ("M01", "wheel.h", "    currentPwm += pwmRampStep;", "    currentPwm -= pwmRampStep;",
     "램프 증가 방향 반전 (rampToward)", "kill"),
    ("M02", "wheel.h", "if (!targetIsZero && fabs(target) < minEffectiveWheelCmd) {",
     "if (false && !targetIsZero && fabs(target) < minEffectiveWheelCmd) {",
     "최소 유효속도 올림 제거 (Wheel::setTarget)", "kill"),
    ("M03", "pid_controller.h", "  if (ki > 0.0) {", "  if (ki < 0.0) {",
     "`if (ki > 0.0)` 적분 클램프 분기 반전", "kill"),
    ("M04", "pid_controller.h",
     "candidate = driveClamp(candidate, -integralErrorLimit, integralErrorLimit);",
     "candidate = driveClamp(candidate, -1e9, 1e9);",
     "적분 클램프 한계 제거(±1e9)", "kill"),
    ("M05", "pid_controller.h", "candidateMagnitude > maxControlPwm && error < 0.0",
     "candidateMagnitude > maxControlPwm && error > 0.0", "unwindHigh 조건 오차 부호 반전", "kill"),
    ("M06", "pid_controller.h", "candidateMagnitude < minimumRunningPwm && error > 0.0",
     "candidateMagnitude < minimumRunningPwm && error < 0.0", "unwindLow 조건 오차 부호 반전", "kill"),
    ("M07", "wheel.h", "static_cast<int32_t>(startBoostUntilMs - nowMs) > 0;",
     "static_cast<int32_t>(startBoostUntilMs - nowMs) >= 0;", "기동 부스트 비교 `> 0` → `>= 0`", "kill"),
    ("M08", "drive_base.h", "linearX - angularZ * cmdWheelBase * 0.5;",
     "linearX - angularZ * cmdWheelBase * 0.25;", "왼쪽 바퀴 `cmdWheelBase * 0.5` → `0.25`", "kill"),
    ("M09", "wheel.h",
     "filteredVelocity += velocityFilterAlpha * (measuredVelocity - filteredVelocity);",
     "filteredVelocity = velocityFilterAlpha * (measuredVelocity - filteredVelocity);",
     "속도 필터 갱신 `+=` → `=`", "kill"),
    ("M10", "periodic_timer.h", "if (now - lastTime < period) {", "if (now - lastTime <= period) {",
     "PeriodicTimer `<` → `<=`", "kill"),
    ("M11", "wheel.h", "((oldTarget > 0.0) != (target > 0.0));",
     "((oldTarget > 0.0) == (target > 0.0));", "방향 전환 판정 `!=` → `==`", "kill"),
    ("M12", NEW_INO_NAME,
     "  const uint32_t elapsedUs = odomTimer.elapsed(currentUs);\n\n  if (!odomTimer.isDue(currentUs)) {\n    return;\n  }",
     "  if (!odomTimer.isDue(currentUs)) {\n    return;\n  }\n  const uint32_t elapsedUs = odomTimer.elapsed(currentUs);",
     ".ino updateOdometry: elapsed 를 isDue 뒤에 계산 (dt=0)", "kill"),
    ("M13", NEW_INO_NAME, "  drive.stopAll();\n", "",
     ".ino stopAllMotors 에서 drive.stopAll() 제거", "kill"),
    ("M14", "wheel.h", "  controller.reset();\n  startBoostUntilMs = 0;\n}",
     "  controller.reset();\n}", "Wheel::stop 에서 부스트 시각 리셋 제거", "kill"),
    ("M15", "pid_controller.h", "(error - previousError) / dt", "(error + previousError) / dt",
     "미분 원값 부호 (D 꺼짐 — 내부 상태만 달라짐)", "kill"),
    ("M16", "wheel.h", "  filteredVelocity = 0.0;\n  targetVelocity = 0.0;",
     "  targetVelocity = 0.0;", "resetMeasurement 에서 필터 속도 리셋 제거", "kill"),
    ("M17", NEW_INO_NAME, "    analogWrite(PWM_PIN[motor], 0);\n  }\n}",
     "    analogWrite(DIR_PIN[motor], 0);\n  }\n}", ".ino stopAllMotors 핀 오배선 (DIR 핀에 0)", "kill"),
    ("M18", "periodic_timer.h", "  return now - lastTime;\n", "  return now - lastTime + 1;\n",
     "PeriodicTimer::elapsed +1", "kill"),
    ("M19", "pid_controller.h", "fabs(candidateMagnitude - saturatedCandidate) < 0.0001;",
     "fabs(candidateMagnitude - saturatedCandidate) < 0.01;",
     "포화 판정 허용오차 0.0001 → 0.01 (미세)", "kill?"),
    ("M20", "drive_base.h", "if (maximumMagnitude > maxWheelCmd) {",
     "if (maximumMagnitude >= maxWheelCmd) {",
     "비율 축소 `>` → `>=` (같을 때 scale=1.0 → 결과 동일)", "equiv"),
    ("M21", "pid_controller.h", "driveMax(0.0, (magnitude - minEffectiveWheelCmd) * feedforwardSlope)",
     "((magnitude - minEffectiveWheelCmd) * feedforwardSlope)",
     "FF 의 max(0, ·) 제거 (목표는 항상 ≥ 최소유효속도라 음수가 안 나옴)", "equiv"),
]


def run_mutations(platform, orig_bin, workdir, seeds, ticks, jobs, orig_src):
    rows = []
    for mid, filename, old, new, desc, expect in MUTATIONS:
        mdir = tempfile.mkdtemp(prefix="mut_%s_" % mid, dir=workdir)
        shutil.rmtree(mdir)
        shutil.copytree(NEW_DIR, mdir)
        path = os.path.join(mdir, filename)
        text = read(path)
        if text.count(old) != 1:
            raise ProbeError("%s: 변이 지점이 %d 곳 (정확히 1 이어야)" % (mid, text.count(old)))
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(text.replace(old, new))
        new_src = read(os.path.join(mdir, NEW_INO_NAME))
        detected_by = None
        detail = None
        try:
            new_bin = compile_program(platform, build_source("new", new_src), mdir, workdir,
                                      "mut_" + mid)
        except ProbeError as error:
            detected_by = "컴파일 실패"
            detail = str(error).splitlines()[0]
            new_bin = None
        if new_bin:
            suite = Suite(platform, orig_bin, new_bin, seeds, ticks, jobs)
            failures, _, _ = suite.run_all(stop_on_first=True, verbose=False)
            if failures:
                where, diff = failures[0]
                detected_by = where
                detail = diff
        timer_detect = None
        if filename == "periodic_timer.h":
            ok, _ = run_timer_check(platform, orig_src, new_src, mdir, workdir)
            timer_detect = not ok
        rows.append((mid, desc, expect, detected_by, detail, timer_detect))
        shutil.rmtree(mdir, ignore_errors=True)
    return rows


# ════════════════════════════════════════════════════════════════════════════
# 8. gcov — 새 헤더 줄 커버리지 (보조 근거)
# ════════════════════════════════════════════════════════════════════════════
def coverage_report(new_src, workdir, seeds, ticks):
    if shutil.which("gcov") is None:
        return None
    cdir = os.path.join(workdir, "cov")
    os.makedirs(cdir)
    plat = Platform("host-cov", ["g++", "-O0", "--coverage"] + HOST_FLAGS, [])
    src_path = os.path.join(cdir, "covnew.cpp")
    with open(src_path, "w", encoding="utf-8") as handle:
        handle.write(build_source("new", new_src))
    binary = os.path.join(cdir, "covnew")
    built = subprocess.run(plat.compile_cmd + ["-I", NEW_DIR, "-o", binary, src_path],
                           capture_output=True, text=True, cwd=cdir)
    if built.returncode != 0:
        return None
    names = run_program(plat, binary, ["list"]).split()
    for name in names:
        for mode in ("direct", "loop"):
            run_program(plat, binary, [mode, name, "1", "hash"])
    for seed in range(1, seeds + 1):
        for mode in ("direct", "loop"):
            run_program(plat, binary, [mode, "fuzz", str(seed), "hash", str(ticks)])
    done = subprocess.run(["gcov", "-o", cdir, src_path], capture_output=True, text=True, cwd=cdir)
    result = {}
    current = None
    for line in done.stdout.splitlines():
        m = re.match(r"File '(.*)'", line)
        if m:
            current = os.path.basename(m.group(1))
            continue
        m = re.match(r"Lines executed:([\d.]+)% of (\d+)", line)
        if m and current in ("drive_math.h", "periodic_timer.h", "pid_controller.h",
                             "wheel.h", "drive_base.h"):
            result[current] = (float(m.group(1)), int(m.group(2)))
    unexecuted = {}
    for header in result:
        gcov_file = os.path.join(cdir, header + ".gcov")
        if os.path.exists(gcov_file):
            lines = []
            for row in read(gcov_file).splitlines():
                parts = row.split(":", 2)
                if len(parts) == 3 and parts[0].strip() == "#####":
                    lines.append("%s: %s" % (parts[1].strip(), parts[2].strip()))
            unexecuted[header] = lines
    return result, unexecuted


# ════════════════════════════════════════════════════════════════════════════
# main
# ════════════════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--seeds", type=int, default=200)
    parser.add_argument("--ticks", type=int, default=2000)
    parser.add_argument("--arm-seeds", type=int, default=None,
                        help="ARM(qemu) 퍼즈 seed 수 (기본 = --seeds)")
    parser.add_argument("--no-arm", action="store_true")
    parser.add_argument("--no-fma", action="store_true")
    parser.add_argument("--no-coverage", action="store_true")
    parser.add_argument("--mutate", action="store_true", help="기본 검사 뒤 변이 검사")
    parser.add_argument("--mutate-only", action="store_true")
    parser.add_argument("--jobs", type=int, default=max(2, (os.cpu_count() or 2)))
    parser.add_argument("--keep", action="store_true", help="임시 폴더 보존")
    args = parser.parse_args()

    fail = False
    summary = []
    workdir = tempfile.mkdtemp(prefix="term-drive-equiv.")
    try:
        orig_src = read(ORIG_INO)
        new_src = read(os.path.join(NEW_DIR, NEW_INO_NAME))
        print("== 대상 ==")
        print("  원본(오라클) %s  sha256=%s" % (os.path.relpath(ORIG_INO, ROOT), sha256(ORIG_INO)))
        print("  새 판        %s  sha256=%s" % (os.path.relpath(os.path.join(NEW_DIR, NEW_INO_NAME), ROOT),
                                              sha256(os.path.join(NEW_DIR, NEW_INO_NAME))))
        for header in ("drive_math.h", "periodic_timer.h", "pid_controller.h", "wheel.h", "drive_base.h"):
            print("    %-18s sha256=%s" % (header, sha256(os.path.join(NEW_DIR, header))))

        plats = platforms(want_arm=not args.no_arm and not args.mutate_only,
                          want_fma=not args.no_fma and not args.mutate_only)
        src_orig = build_source("orig", orig_src)
        src_new = build_source("new", new_src)

        if not args.mutate_only:
            # ── 헤더 해시 ──
            print("\n== 안전 헤더 5개 sha256 (원본 폴더 ↔ 새 폴더) ==")
            ok, rows = header_hash_check(NEW_DIR)
            for name, a, b, same in rows:
                print("  %-20s %s  %s" % (name, "같음" if same else "다름!", a))
            fail |= not ok
            summary.append("안전 헤더 sha256: %s" % ("5개 모두 같음" if ok else "다름"))

            # ── 구조 검사 ──
            print("\n== 구조 검사 (두 .ino 의 0열 함수 전부, 주석 제외 비교) ==")
            problems, info = structure_check(orig_src, new_src)
            print("  비교한 공통 함수 %d 개 / 삭제 %s" % (info["functions_compared"], info["removed"]))
            print("  바뀐 함수: %s" % info["changed"])
            for name, same in info["normalized"].items():
                print("  정규화 비교 %-20s %s" % (name, "동일" if same else "다름!"))
            for problem in problems:
                print("  [실패] " + problem)
            fail |= bool(problems)
            summary.append("구조 검사: %s" % ("예상 밖 변경 없음" if not problems else "문제 %d건" % len(problems)))

            # ── 플랫폼별 동치 ──
            baseline_digests = None
            for plat in plats:
                seeds = args.seeds
                if plat.run_prefix and args.arm_seeds is not None:
                    seeds = args.arm_seeds
                print("\n== 동치 검사 [%s]  퍼즈 %d seed × %d tick × 2 모드 ==" % (plat.name, seeds, args.ticks))
                orig_bin = compile_program(plat, src_orig, ORIG_DIR, workdir, "orig_" + str(id(plat)))
                new_bin = compile_program(plat, src_new, NEW_DIR, workdir, "new_" + str(id(plat)))
                suite = Suite(plat, orig_bin, new_bin, seeds, args.ticks, args.jobs)
                failures, rows, total = suite.run_all()
                fma_o = fused_multiply_add_count(plat, orig_bin)
                fma_n = fused_multiply_add_count(plat, new_bin)
                print("  FMA 명령 수(정보): 원본 %s / 새 판 %s" % (fma_o, fma_n))
                if baseline_digests is None:
                    baseline_digests = suite.digests
                else:
                    differ = sum(1 for key, value in suite.digests.items()
                                 if baseline_digests.get(key) != value)
                    print("  (정보) 이 플랫폼의 원본 trace 가 host-O1 원본 trace 와 다른 이름 시나리오: %d / %d"
                          " — 0 이 아니면 부동소수 결과가 플랫폼마다 실제로 다르다는 뜻 (그래도 원본↔새 판은 같아야 한다)"
                          % (differ, len(suite.digests)))
                print("  %-24s %-6s %9s %6s" % ("시나리오", "모드", "tick", "불일치"))
                for name, mode, ticks, nfail, diff, cov in rows:
                    print("  %-24s %-6s %9d %6d" % (name, mode, ticks, nfail))
                print("  총 tick %d, 불일치 %d" % (total, len(failures)))
                cov_total = None
                for row in rows:
                    if row[5]:
                        cov_total = _cov_add(cov_total, row[5])
                if cov_total:
                    print("  관측 커버리지(원본 출력 기준 합): " + " ".join("%s=%d" % kv for kv in cov_total.items()))
                    # 시험이 이빨을 잃지 않았는지: 핵심 경로를 한 번도 안 밟았으면 실패
                    missing = [k for k in REQUIRED_COVERAGE if cov_total.get(k, 0) == 0]
                    if missing:
                        print("  [실패] 시나리오가 이 경로를 한 번도 안 밟았다: %s" % missing)
                        fail = True
                        summary.append("[%s] 커버리지 부족 %s" % (plat.name, missing))
                for where, diff in failures[:10]:
                    print("  [불일치] %s 첫 차이: %s" % (where, diff))
                fail |= bool(failures)
                summary.append("[%s] 동치: tick %d, 불일치 %d" % (plat.name, total, len(failures)))

                # ── 타이머 ──
                ok, out = run_timer_check(plat, orig_src, new_src, NEW_DIR, workdir)
                print("  PeriodicTimer 검사: %s" % ("일치" if ok else "불일치!"))
                summary.append("[%s] PeriodicTimer: %s" % (plat.name, "일치" if ok else "불일치"))
                for line in out.splitlines():
                    print("    " + line)
                fail |= not ok

            # ── gcov ──
            if not args.no_coverage:
                cov = coverage_report(new_src, workdir, min(args.seeds, 50), args.ticks)
                if cov:
                    result, unexecuted = cov
                    print("\n== 새 헤더 줄 커버리지 (gcov, host -O0, 이름 시나리오 전부 + 퍼즈 %d seed) =="
                          % min(args.seeds, 50))
                    for header, (pct, n) in sorted(result.items()):
                        print("  %-18s %6.2f%% of %d" % (header, pct, n))
                        for line in unexecuted.get(header, [])[:12]:
                            print("      미실행 " + line)

        # ── 변이 ──
        if args.mutate or args.mutate_only:
            plat = Platform("host-O1", ["g++", "-O1"] + HOST_FLAGS, [])
            print("\n== 변이 검사 [host-O1] (임시 사본, 한 번에 하나) ==")
            orig_bin = compile_program(plat, src_orig, ORIG_DIR, workdir, "orig_mut")
            rows = run_mutations(plat, orig_bin, workdir, args.seeds, args.ticks, args.jobs, orig_src)
            for mid, desc, expect, detected_by, detail, timer_detect in rows:
                status = "검출" if detected_by else "미검출"
                print("  %s %-6s 기대=%-5s %s" % (mid, status, expect, desc))
                if detected_by:
                    print("        첫 검출: %s  %s" % (detected_by, detail))
                if timer_detect is not None:
                    print("        타이머 검사: %s" % ("검출" if timer_detect else "미검출"))
                if expect == "kill" and not detected_by:
                    fail = True
                    print("        [실패] 반드시 잡혀야 하는 변이가 통과했다")
            killed = sum(1 for row in rows if row[3])
            must = [row for row in rows if row[2] == "kill"]
            summary.append("변이: %d/%d 검출 (반드시 잡혀야 하는 %d개 중 미검출 %d)"
                           % (killed, len(rows), len(must), sum(1 for row in must if not row[3])))
    except ProbeError as error:
        print("\n[판정 불능] " + str(error))
        return 2
    except subprocess.TimeoutExpired as error:
        print("\n[판정 불능] 시간 초과: %s" % error)
        return 2
    except Exception as error:  # noqa: BLE001 — 예상 못 한 예외도 '통과/불일치' 로 읽히면 안 된다
        print("\n[판정 불능] 예외: %r" % (error,))
        return 2
    finally:
        if args.keep:
            print("임시 폴더 보존: " + workdir)
        else:
            shutil.rmtree(workdir, ignore_errors=True)

    print("\n== 요약 ==")
    for line in summary:
        print("  " + line)
    print("== 결과: %s ==" % ("실패 (위 [실패]/[불일치] 참고)" if fail else "통과"))
    return 1 if fail else 0


if __name__ == "__main__":
    sys.exit(main())
