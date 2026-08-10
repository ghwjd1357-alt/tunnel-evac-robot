#ifndef REARM_GATE_H
#define REARM_GATE_H

// ============================================================================
// rearm_gate.h — re-arm 래치의 상태 전이 전체 (2026-08-11, 검토 §54 보완)
// 계약 정본 = docs/REAL_ROBOT_VALUES.md §1-f
//
// 왜 이 파일이 생겼나 (검토 §54.7):
//   §54 는 상태 전이의 불변조건 3개가 각각 깨진 것을 P1 로 냈다 —
//     §54.1 DISARMED 진입이 모터를 안 멈춘다
//     §54.2 서비스 응답 이전에 도착한 명령을 구분할 장벽이 없다
//     §54.3 NaN/Inf 가 zero-hold 를 안 끊는다
//   셋 다 "전이가 스케치 여기저기에 흩어져 있어 한 자리를 고치면 다른 자리가 남는다"는
//   같은 뿌리였다. 그래서 개별 분기를 고치지 않고 **전이 전체를 이 파일 하나로 옮겼다.**
//
// 이 파일은 **순수 함수만** 담는다 — Arduino·micro-ROS·millis()·전역을 쓰지 않는다.
//   → 그래서 PC 에서 g++ 로 그대로 컴파일된다: tools/rearm_gate_host_test.cpp
//   → 그것이 §54.7 이 요구한 "host-side 결정론 harness" 다. 시각은 인자로 받으므로
//     실제 대기 없이 전이 경계를 정확히 재현한다.
//
// 모터를 실제로 멈추는 것은 호출자다. 이 파일은 "굴려도 되는가"만 답한다.
// 🔴 반환값이 DRIVE_EFFECT_DRIVE 가 **아니면 호출자는 무조건 정지**한다 — 기본이 정지다.
// ============================================================================

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

// ── 시간 계약 ────────────────────────────────────────────────────────────────
// ① zero-hold: 무장 이전에 발행자가 실제로 멈췄다는 증거. 끊김 없는 zero 가 이 시간만큼.
static const uint32_t REARM_ZERO_HOLD_MS = 500;

// ② post-response quiet: 서비스 성공 응답 **전에** 큐에 있던 명령을 배제하는 장벽 (§54.2).
//    rclc 는 static-LET 이라 한 spin 에서 ready handle 을 전부 take 한 뒤 처리한다
//    (local rclc 2.0.8-humble `executor.h:742~751`). 그래서 take 스냅샷 뒤·응답 전에
//    도착한 /cmd_vel 은 **다음 spin** 에서 처리되며, 그때 상태가 이미 ARMED 면 구동에
//    들어간다. Twist 에는 세대·stamp 가 없어 받은 쪽에서 그 명령을 식별할 수단이 없다.
//    → 서비스 성공은 ARMED 가 아니라 PENDING 으로 보내고, 응답 뒤 이 시간만큼
//      **비영 명령이 하나도 없어야** ARMED 가 된다. 잔류 비영 명령은 장벽 안에서
//      도착해 DISARMED 로 떨어뜨린다 = fail-closed.
//    ⚠ 이 장벽이 닫는 것은 "장벽 시간 안에 도착하는 잔류 명령"이다. 그보다 더 늦게
//      도착하는 명령까지 배제하려면 명령 인터페이스에 세대·stamp 가 있어야 한다.
//      전제조건·재개방 조건 = docs/REAL_ROBOT_VALUES.md §1-f.
static const uint32_t REARM_POST_ARM_QUIET_MS = 500;

// ── 상태 ─────────────────────────────────────────────────────────────────────
// /drive/diag 의 z 로 그대로 발행된다.
enum DriveState : uint8_t {
  DRIVE_DISARMED = 0,  // 부팅 / E-stop / 계약 위반. 모터 정지가 불변조건이다.
  DRIVE_READY = 1,     // E-stop 해제 + zero-hold 충족. 서비스 대기.
  DRIVE_ARMED = 2,     // 명령 수용. **여기서만 모터가 돈다.**
  DRIVE_PENDING = 3,   // 서비스 성공 응답 뒤 quiet 장벽 진행 중 (§54.2).
};

// 가장 최근 /drive/enable 이 거절된 사유. /drive/diag 의 y.
enum DriveReject : uint8_t {
  REARM_OK = 0,
  REARM_REJECT_ESTOP = 1,        // E-stop 활성
  REARM_REJECT_ZERO_HOLD = 2,    // zero-hold 미충족 (= 아직 READY 가 아니다)
  REARM_REJECT_ALREADY = 3,      // 이미 ARMED 이거나 PENDING (장벽 진행 중)
};

// 호출자가 이번 명령으로 무엇을 해야 하는가. 기본이 정지다.
enum DriveEffect : uint8_t {
  DRIVE_EFFECT_HOLD = 0,   // 🔴 모터를 멈춘다. cmdVelReceived 도 내린다.
  DRIVE_EFFECT_DRIVE = 1,  // 이 명령을 적용해도 된다.
};

struct RearmGate {
  uint8_t state;
  bool zeroHolding;         // zeroSinceMs 의 유효 플래그. millis() 는 0 이 정상값이라
  uint32_t zeroSinceMs;     // 센티널로 0 을 못 쓴다.
  uint32_t quietSinceMs;    // PENDING 진입 시각
  uint32_t serviceCalls;    // 도달한 서비스 호출 누계 (거절 포함)
  uint8_t rejectReason;
};

// ── 전이표 ───────────────────────────────────────────────────────────────────
//
//  현재 상태 │ zero 명령        │ 비영 명령   │ NaN/Inf │ enable(true)       │ enable(false) │ tick
//  ─────────┼─────────────────┼────────────┼────────┼───────────────────┼──────────────┼──────────
//  DISARMED │ hold 누적→READY  │ DISARMED   │DISARMED│ 거절 ZERO_HOLD     │ DISARMED     │ −
//  READY    │ hold 유지        │ DISARMED   │DISARMED│ **PENDING** (성공)  │ DISARMED     │ −
//  PENDING  │ 장벽 유지(무해)   │ DISARMED   │DISARMED│ 거절 ALREADY       │ DISARMED     │ 500ms→ARMED
//  ARMED    │ **구동**(zero)   │ **구동**    │DISARMED│ 거절 ALREADY       │ DISARMED     │ −
//
//  E-stop 활성: 어느 상태에서든 DISARMED (호출자가 rearmGateDisarm 을 부른다).
//  🔴 DISARMED 로 가는 모든 칸은 호출자 쪽에서 **그 자리에서** 모터를 멈춘다 (§54.1).

static inline void rearmGateInit(struct RearmGate* g)
{
  g->state = DRIVE_DISARMED;
  g->zeroHolding = false;
  g->zeroSinceMs = 0;
  g->quietSinceMs = 0;
  g->serviceCalls = 0;
  g->rejectReason = REARM_OK;
}

// DISARMED 로 가는 유일한 문. 타이머를 전부 지운다 — 남겨 두면 다음 hold 가
// 옛날 시작점을 물려받아 §54.3 과 같은 "끊겼는데 이어진" 판정이 생긴다.
static inline void rearmGateDisarm(struct RearmGate* g)
{
  g->state = DRIVE_DISARMED;
  g->zeroHolding = false;
  g->zeroSinceMs = 0;
  g->quietSinceMs = 0;
}

// /cmd_vel 한 건. 반환이 DRIVE 가 아니면 호출자는 정지한다.
static inline enum DriveEffect rearmGateOnCommand(struct RearmGate* g,
                                                  double linearX,
                                                  double angularZ,
                                                  uint32_t nowMs)
{
  // §54.3 — 유한하지 않은 입력은 **명령이 아니라 고장 신호**다. 어느 상태에서 오든
  // fail-closed 로 DISARMED 로 간다(ARMED 포함 = 사용자 결정 ⓐ, 08-11). 구판은
  // 여기서 return 만 해서 zero-hold 타이머가 살아남았고, NaN 을 사이에 끼운
  // zero@0 → NaN@250 → zero@500 이 "zero 0.5초 연속"으로 승인됐다.
  if (!isfinite(linearX) || !isfinite(angularZ)) {
    rearmGateDisarm(g);
    return DRIVE_EFFECT_HOLD;
  }

  const bool isZero = (linearX == 0.0 && angularZ == 0.0);

  if (g->state == DRIVE_ARMED) {
    return DRIVE_EFFECT_DRIVE;
  }

  if (!isZero) {
    // 멈춘 적 없는 발행자는 READY 에도 ARMED 에도 못 간다. PENDING 에서는
    // 이것이 곧 "응답 전에 큐에 있던 잔류 명령"이므로 장벽을 깨고 떨어뜨린다.
    rearmGateDisarm(g);
    return DRIVE_EFFECT_HOLD;
  }

  if (g->state == DRIVE_PENDING) {
    // 장벽 중의 zero 는 무해하다 — 타이머를 건드리지 않는다. 승격은 tick 이 한다
    // (발행이 완전히 끊긴 경우에도 장벽이 완주해야 하기 때문이다).
    return DRIVE_EFFECT_HOLD;
  }

  // DISARMED · READY
  if (!g->zeroHolding) {
    g->zeroHolding = true;
    g->zeroSinceMs = nowMs;
  } else if (nowMs - g->zeroSinceMs >= REARM_ZERO_HOLD_MS) {
    g->state = DRIVE_READY;
  }
  return DRIVE_EFFECT_HOLD;
}

// 주기 호출. PENDING 의 quiet 장벽만 여기서 완주한다 — 발행이 아예 끊겨도
// (= cmd_vel 콜백이 안 불려도) 장벽이 끝나야 하기 때문이다.
static inline void rearmGateTick(struct RearmGate* g, uint32_t nowMs)
{
  if (g->state == DRIVE_PENDING &&
      nowMs - g->quietSinceMs >= REARM_POST_ARM_QUIET_MS) {
    g->state = DRIVE_ARMED;
  }
}

// /drive/enable 한 건. 반환 = 서비스 success 값.
// 🔴 호출자는 이 함수가 false 를 돌려주든 true 를 돌려주든 **응답을 보내기 전에**
//    상태가 ARMED 가 아니면 모터를 멈춘다 (§54.1: 응답은 정리 뒤에 보낸다).
static inline bool rearmGateOnService(struct RearmGate* g,
                                      bool enable,
                                      bool estopActive,
                                      uint32_t nowMs)
{
  // 모든 분기 이전에 센다. 호출자가 이 값이 안 변한 것을 보면 "요청이 보드에
  // 도달하지 못했다"를 "로직이 거절했다"와 구분할 수 있다 — 굽기 1회 롤아웃에서
  // 그것을 구분할 다른 수단이 없다.
  g->serviceCalls++;

  if (!enable) {
    rearmGateDisarm(g);
    g->rejectReason = REARM_OK;
    return true;
  }

  if (estopActive) {
    rearmGateDisarm(g);
    g->rejectReason = REARM_REJECT_ESTOP;
    return false;
  }

  if (g->state == DRIVE_ARMED || g->state == DRIVE_PENDING) {
    g->rejectReason = REARM_REJECT_ALREADY;
    return false;
  }

  if (g->state != DRIVE_READY) {
    g->rejectReason = REARM_REJECT_ZERO_HOLD;
    return false;
  }

  // 🔴 여기서 ARMED 로 가지 않는다 (§54.2). 응답 전에 큐에 있던 명령과 응답 뒤
  //    새로 만든 명령을 구분할 수단이 없으므로, 장벽을 한 번 더 세운다.
  g->state = DRIVE_PENDING;
  g->quietSinceMs = nowMs;
  g->rejectReason = REARM_OK;
  return true;
}

#endif  // REARM_GATE_H
