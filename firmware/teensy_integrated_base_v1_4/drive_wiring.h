#ifndef DRIVE_WIRING_H
#define DRIVE_WIRING_H

// ============================================================================
// drive_wiring.h — 전이와 **모터 정지**를 한 동작으로 묶는다 (2026-08-11, 검토 §55.2)
// 계약 정본 = docs/REAL_ROBOT_VALUES.md §1-f
//
// 왜 이 파일이 생겼나 (검토 §55.2):
//   §54 보완은 전이를 rearm_gate.h 로 옮겼지만, "DISARMED 로 갈 때 모터를 세운다"는
//   §54.1 의 본체는 .ino 에 남아 있었다. harness 는 rearm_gate.h 만 include 하므로
//   그 배선을 구판으로 되돌려도 **412/412 로 통과했다.** 즉 안전 출력 경로의 자동
//   방어가 0 이었고, 정본은 그것을 "세 P1 변이 전부 FAIL" 이라고 잘못 적고 있었다.
//   (같은 종류의 과거 실패 = docs/TEST_GATES.md §21 P2-① "채점자가 자기 답을 채점".)
//
// 그래서 정지를 **호출자의 기억력에서 떼어냈다.** 전이와 부작용이 한 함수 안에 있고,
// 스케치는 그 함수만 부른다. 정지를 빼려면 이 헤더를 고쳐야 하고, 이 헤더는 PC 에서
// 가짜 모터(FakeSink)를 붙여 그대로 돌아간다 → 변이가 harness 에서 죽는다.
//
// ── Sink 계약 (스케치는 TeensyDriveSink, harness 는 FakeDriveSink) ────────────
//   void stopAllMotors();               // PWM·목표속도·적분기·부스트를 전부 0 으로
//   void setCmdVelReceived(bool);       // 수신 플래그
//   void noteCommandAccepted(uint32_t); // lastCmdVelMs = now; 수신 플래그 = true
//
// 🔴 이 헤더가 증명하지 못하는 것 (숨기지 않는다): 스케치가 **이 함수들을 부르는가**.
//    그건 텍스트 구조 검사(tools/rearm_gate_host_test.sh 2단계)가 본다. 동작 검사보다
//    약한 증거이고, 약한 줄 알고 쓴다.
// ============================================================================

#include <stdbool.h>
#include <stdint.h>

#include "rearm_gate.h"

// ── ① DISARMED 진입 = 전이 + 정지 + 수신 플래그 해제, 한 동작 (§54.1) ────────
// 구판은 이 셋이 호출자마다 흩어져 있었고, /drive/enable 의 false 분기가 정지를
// 빼먹어 상태 토픽은 false 인데 모터는 워치독까지 돌았다.
template <typename Sink>
inline void driveDisarm(struct RearmGate* g, Sink& sink)
{
  rearmGateDisarm(g);
  sink.stopAllMotors();
  sink.setCmdVelReceived(false);
}

// ── ①-b 사유를 남기는 해제 (§63.1) ──────────────────────────────────────────
// ① 과 같은 동작에 "누가 풀었는지"만 얹는다. 사유는 **전이했을 때만** 기록된다.
// 반환 = 이번 호출이 실제로 풀었는가. 사유 기록을 스케치 쪽 규율에 맡기면
// 08-13 이전처럼 정지와 기록이 다시 갈라지므로 여기서 한 동작으로 묶는다.
template <typename Sink>
inline bool driveDisarmWithReason(struct RearmGate* g, uint8_t reason, Sink& sink)
{
  const bool transitioned = rearmGateDisarmWithReason(g, reason);
  sink.stopAllMotors();
  sink.setCmdVelReceived(false);
  return transitioned;
}

// ── ② 출력단 가드 (§54.1) ────────────────────────────────────────────────────
// 반환 false = 이번 주기에 PWM 을 쓰면 안 된다. 정지는 이미 여기서 끝냈다.
// 전이 경로 하나가 정지를 잊어도 모터가 못 도는 **두 번째 겹**이다: ARMED 가 아니면
// 출력 자체가 0 이다.
template <typename Sink>
inline bool driveOutputAllowed(const struct RearmGate* g, bool estopActive, Sink& sink)
{
  if (estopActive || g->state != DRIVE_ARMED) {
    sink.stopAllMotors();
    sink.setCmdVelReceived(false);
    return false;
  }
  return true;
}

// ── ③ /cmd_vel 한 건의 배선 ──────────────────────────────────────────────────
// 반환 true = 호출자가 이 명령을 적용한다. false 면 정지까지 이미 끝났다.
template <typename Sink>
inline bool driveOnCommand(struct RearmGate* g,
                           double linearX,
                           double angularZ,
                           uint32_t nowMs,
                           Sink& sink)
{
  if (rearmGateOnCommand(g, linearX, angularZ, nowMs) != DRIVE_EFFECT_DRIVE) {
    sink.stopAllMotors();
    sink.setCmdVelReceived(false);
    return false;
  }
  sink.noteCommandAccepted(nowMs);
  return true;
}

// ── ④ /drive/enable 한 건의 배선 ─────────────────────────────────────────────
// 반환 = 응답의 success 값.
// 🔴 정리가 **응답보다 먼저** 끝나야 한다: 운용자가 disable 응답에서 success:true 를
//    읽었을 때 "모터는 이미 0" 을 믿을 수 있어야 한다. 그 순서를 호출자에게 맡기지
//    않으려고 여기서 정리까지 하고 값을 돌려준다 — 호출자는 이 반환값을 응답에
//    넣는 것 말고 할 일이 없다.
// 🔴 성공해도 ARMED 가 아니라 ARMING 이므로(§54.2·§55.1) 아래 정리는 enable 성공
//    경로에서도 실행된다. 그것이 정상이다: 장벽이 끝나기 전에는 못 굴러야 한다.
template <typename Sink>
inline bool driveOnServiceRequest(struct RearmGate* g,
                                  bool enable,
                                  bool estopActive,
                                  Sink& sink)
{
  const bool success = rearmGateOnService(g, enable, estopActive);

  if (g->state != DRIVE_ARMED) {
    sink.stopAllMotors();
    sink.setCmdVelReceived(false);
  }
  return success;
}

#endif  // DRIVE_WIRING_H
