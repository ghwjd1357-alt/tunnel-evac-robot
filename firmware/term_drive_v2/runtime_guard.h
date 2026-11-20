#ifndef RUNTIME_GUARD_H
#define RUNTIME_GUARD_H

// Main-loop latency guard for the Teensy drive firmware.
//
// This header is deliberately Arduino-free so the exact boundary logic can be
// compiled and mutation-tested on the laptop.  It does not stop motors itself;
// the sketch owns that physical side and must disarm whenever a record function
// returns false.

#include <stdbool.h>
#include <stdint.h>

// §73.3: first-bench provisional value, not a measured final threshold.
// The old 320 ms estimate counted the USB CDC ceiling only once even though up
// to eight measured publishes can coincide in one loop.  Therefore 400 ms is
// not claimed to sit above a proven normal-loop ceiling.  Keep it provisional
// and replace it only after phase_max_us/publish_max_us are measured on-board.
static const uint32_t RUNTIME_STALL_LIMIT_US = 400000U;
static const uint8_t RUNTIME_PHASE_COUNT = 7U;
// 🔴 예약 41-g 계약 3판: 생존 표본 /firmware/pulse 가 publishMeasured 를 거치므로
// publish slot 은 8 → **9** 다. 8 로 남으면 pulse 가 계측 밖이라는 뜻이라 FAIL 이다
// (MASTER_PLAN §7 41-g "그 증가 자체를 기대값으로 회귀에 넣는다").
static const uint8_t RUNTIME_PUBLISH_COUNT = 9U;
static const uint8_t RUNTIME_PUBLISH_CODE_BASE = 16U;

enum RuntimePhase {
  RUNTIME_PHASE_SPIN = 0,
  RUNTIME_PHASE_ODOM = 1,
  RUNTIME_PHASE_IMU = 2,
  RUNTIME_PHASE_DIAGNOSTICS = 3,
  RUNTIME_PHASE_FIRMWARE_INFO = 4,
  RUNTIME_PHASE_TIME_SYNC = 5,
  RUNTIME_PHASE_LOOP = 6
};

enum RuntimePublishSite {
  RUNTIME_PUBLISH_ODOM = 0,
  RUNTIME_PUBLISH_IMU = 1,
  RUNTIME_PUBLISH_IMU_YAW = 2,
  RUNTIME_PUBLISH_GYRO_BIAS = 3,
  RUNTIME_PUBLISH_ESTOP = 4,
  RUNTIME_PUBLISH_DRIVE_ENABLED = 5,
  RUNTIME_PUBLISH_DRIVE_DIAG = 6,
  RUNTIME_PUBLISH_FIRMWARE_INFO = 7,
  // 예약 41-g 3판 — 생존 표본. 이 publish 자체의 최악 시간도 runtime guard 에
  // 잡혀야 한다. 🔴 사건 publisher(/firmware/event)는 **비주기**라 이 배열에
  // 넣지 않고 따로 센다 — 넣으면 계약의 "8 → 9" 가 10 이 된다.
  RUNTIME_PUBLISH_PULSE = 8
};

struct RuntimeGuard {
  uint32_t phaseMaxUs[RUNTIME_PHASE_COUNT];
  uint32_t publishMaxUs[RUNTIME_PUBLISH_COUNT];
  uint32_t overrunCount;
  uint32_t publishFailureCount;
  uint32_t lastOverrunUs;
  uint8_t lastOverrunCode;
  bool overrunLatchedThisLoop;
};

inline void runtimeGuardInit(struct RuntimeGuard* guard)
{
  for (uint8_t i = 0; i < RUNTIME_PHASE_COUNT; ++i) {
    guard->phaseMaxUs[i] = 0U;
  }
  for (uint8_t i = 0; i < RUNTIME_PUBLISH_COUNT; ++i) {
    guard->publishMaxUs[i] = 0U;
  }
  guard->overrunCount = 0U;
  guard->publishFailureCount = 0U;
  guard->lastOverrunUs = 0U;
  guard->lastOverrunCode = 0U;
  guard->overrunLatchedThisLoop = false;
}

inline void runtimeGuardBeginLoop(struct RuntimeGuard* guard)
{
  guard->overrunLatchedThisLoop = false;
}

inline bool runtimeGuardRecord(struct RuntimeGuard* guard,
                               uint8_t code,
                               uint32_t elapsedUs,
                               uint32_t* maximum)
{
  if (elapsedUs > *maximum) {
    *maximum = elapsedUs;
  }

  if (elapsedUs >= RUNTIME_STALL_LIMIT_US) {
    // A measured publish sits inside a phase, and every phase sits inside the
    // full loop.  Keep the first (therefore most specific) overrun in one loop;
    // otherwise the enclosing phase/loop immediately overwrites its cause.
    if (!guard->overrunLatchedThisLoop) {
      ++guard->overrunCount;
      guard->lastOverrunUs = elapsedUs;
      guard->lastOverrunCode = code;
      guard->overrunLatchedThisLoop = true;
    }
    return false;
  }
  return true;
}

inline bool runtimeGuardRecordPhase(struct RuntimeGuard* guard,
                                    uint8_t phase,
                                    uint32_t elapsedUs)
{
  if (phase >= RUNTIME_PHASE_COUNT) {
    return false;
  }
  return runtimeGuardRecord(guard, phase, elapsedUs, &guard->phaseMaxUs[phase]);
}

inline bool runtimeGuardRecordPublish(struct RuntimeGuard* guard,
                                      uint8_t site,
                                      uint32_t elapsedUs,
                                      bool publishOk)
{
  if (site >= RUNTIME_PUBLISH_COUNT) {
    return false;
  }
  if (!publishOk) {
    ++guard->publishFailureCount;
  }
  return runtimeGuardRecord(
      guard,
      static_cast<uint8_t>(RUNTIME_PUBLISH_CODE_BASE + site),
      elapsedUs,
      &guard->publishMaxUs[site]);
}

#endif  // RUNTIME_GUARD_H
