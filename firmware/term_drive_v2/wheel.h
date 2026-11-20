#ifndef WHEEL_H
#define WHEEL_H

// ============================================================================
// wheel.h — 바퀴 하나를 객체 하나로
//
// 원본 .ino 는 바퀴 4개의 상태를 **배열 여러 개에 같은 번호로** 흩어 두었다:
//     targetWheelVelocity[4], measuredWheelVelocity[4], filteredWheelVelocity[4],
//     desiredPwm[4], appliedPwm[4], startBoostUntilMs[4], previousCount[4],
//     wheelIntegralError[4], wheelPreviousError[4], wheelFilteredDerivative[4]
// 여기서는 "바퀴 하나" 에 속한 값을 한 클래스에 모으고(강의 07 배열 → 09 클래스),
// 바깥에서 함부로 못 바꾸게 private 으로 숨긴다(캡슐화). 바퀴 4개는 이 클래스의
// 객체 4개다 — 강의 09 의 Box1·Box2·Box3 와 같은 생각이다.
//
// 이 클래스가 하는 일 (모두 계산만)
//   setTarget()        목표 속도 받기 (원본 setTargetWheelVelocity)
//   measure()          엔코더 카운트 → 바퀴 속도 → 필터 (원본 updateOdometry 의 바퀴 몫)
//   updateController() 피드포워드 + PI 로 원하는 PWM 계산 (원본 updateWheelControllers 의 바퀴 몫)
//   updateOutput()     기동 부스트·램프로 실제 낼 PWM 결정 (원본 updateMotorOutputs 의 바퀴 몫)
//   stop()             정지 상태로 (원본 stopAllMotors 의 바퀴 몫)
//
// 이 클래스가 하지 **않는** 일
//   analogWrite · digitalWrite · 엔코더 읽기 · millis() — 하드웨어는 .ino 가 맡는다.
//   핀 번호도 여기 두지 않는다 — .ino 의 PWM_PIN·DIR_PIN 한 곳에만 있다.
//   그래서 이 파일은 Arduino 없이 노트북 g++ 로 컴파일·시험할 수 있다.
//   시각(nowMs)과 엔코더 카운트는 인자로 받는다.
//
// 🔴 각 함수의 계산 순서는 원본(1.6.3)과 같다. 원본과의 동작 일치는
//    tools/term_drive_equivalence.* 가 노트북에서 비교하고, 보드에서의 연산 방식(FMA)은
//    보드용 바이너리를 원본과 비교해 확인했다 (pid_controller.h 머리말).
// ============================================================================

#include <math.h>
#include <stdint.h>

#include "drive_math.h"
#include "wheel_config.h"
#include "pid_controller.h"

class Wheel
{
public:
  // 매개변수 생성자 (강의 10): 바퀴마다 다를 수 있는 것(부스트·저속유지·최소구동 PWM)과
  // 공통 설정을 받아 바퀴 객체를 만든다.
  Wheel(int startBoostPwmValue, int lowSpeedHoldPwmValue, int minRunningPwmValue,
        const WheelControlConfig& controlConfig,
        const WheelMotionConfig& motionConfig);

  // 목표 속도를 정한다. nowMs = 지금 시각(밀리초) — 출발·방향 전환 시 부스트 끝 시각 계산용.
  void setTarget(double target, uint32_t nowMs);

  // 엔코더 누적 카운트로 속도를 갱신한다. 이번 구간에 굴러간 거리[m]를 돌려준다(오도메트리용).
  double measure(long currentCount, double dt);

  // 제어 계산: 원하는 PWM 을 갱신한다.
  void updateController(double dt);

  // 실제로 낼 PWM 을 갱신하고 돌려준다 (부스트·램프 적용). 0 이면 모터를 끄면 된다.
  int updateOutput(uint32_t nowMs);

  // 정지: 목표·PWM·부스트·제어기 기억을 0 으로. (필터 속도와 엔코더 기준값은 그대로 —
  // 원본 stopAllMotors 와 같다.)
  void stop();

  // 제어기 기억만 0 으로.
  void resetController();

  // 측정값을 0 으로 (오도메트리 리셋 때). 엔코더 기준값·측정 속도·필터 속도·목표.
  void resetMeasurement();

  // ── 읽기 전용 (get 함수, 강의 09 의 getVolume 과 같은 방식) ──
  double getTarget() const;
  double getMeasuredVelocity() const;
  double getFilteredVelocity() const;
  int getDesiredPwm() const;
  int getAppliedPwm() const;
  uint32_t getStartBoostUntilMs() const;
  long getPreviousCount() const;
  const WheelSpeedController& getController() const;

private:
  // 지금 PWM 에서 목표 PWM 쪽으로 한 걸음(램프). 원본 movePwmTowardTarget.
  int rampToward(int currentPwm, int targetPwm) const;

  // ── 바퀴마다 다를 수 있는 고정값 ──
  int startBoostPwm;

  // ── 공통 설정 ──
  double maxWheelCmd;
  double commandDeadband;
  double minEffectiveWheelCmd;
  uint32_t startBoostDurationMs;
  int pwmRampStep;
  double velocityFilterAlpha;
  double distancePerCount;

  // ── 계속 바뀌는 상태 (원본에서는 전역 배열의 한 칸이었다) ──
  double targetVelocity;      // 원본 targetWheelVelocity[m]
  double measuredVelocity;    // 원본 measuredWheelVelocity[m]
  double filteredVelocity;    // 원본 filteredWheelVelocity[m]
  int desiredPwm;             // 원본 desiredPwm[m]
  int appliedPwm;             // 원본 appliedPwm[m]
  uint32_t startBoostUntilMs; // 원본 startBoostUntilMs[m]
  long previousCount;         // 원본 previousCount[m]

  // 바퀴 객체 안에 제어기 객체가 들어 있다 (객체 안의 객체).
  WheelSpeedController controller;
};

// ── 멤버함수 정의 ─────────────────────────────────────────────────────────

inline Wheel::Wheel(int startBoostPwmValue, int lowSpeedHoldPwmValue, int minRunningPwmValue,
                    const WheelControlConfig& controlConfig,
                    const WheelMotionConfig& motionConfig)
    : startBoostPwm(startBoostPwmValue),
      maxWheelCmd(motionConfig.maxWheelCmd),
      commandDeadband(motionConfig.commandDeadband),
      minEffectiveWheelCmd(motionConfig.minEffectiveWheelCmd),
      startBoostDurationMs(motionConfig.startBoostDurationMs),
      pwmRampStep(motionConfig.pwmRampStep),
      velocityFilterAlpha(motionConfig.velocityFilterAlpha),
      distancePerCount(motionConfig.distancePerCount),
      targetVelocity(0.0),
      measuredVelocity(0.0),
      filteredVelocity(0.0),
      desiredPwm(0),
      appliedPwm(0),
      startBoostUntilMs(0),
      previousCount(0),
      controller(controlConfig, motionConfig, lowSpeedHoldPwmValue, minRunningPwmValue)
{
}

// 원본 setTargetWheelVelocity 와 같은 순서.
inline void Wheel::setTarget(double target, uint32_t nowMs)
{
  target = driveClamp(target, -maxWheelCmd, maxWheelCmd);

  const double oldTarget = targetVelocity;
  const bool targetIsZero = fabs(target) < commandDeadband;

  // 너무 느린 명령은 최소 유효 속도로 올린다 (그보다 느리면 모터가 못 돈다).
  if (!targetIsZero && fabs(target) < minEffectiveWheelCmd) {
    target = (target >= 0.0) ? minEffectiveWheelCmd : -minEffectiveWheelCmd;
  }

  const bool oldWasZero = fabs(oldTarget) < commandDeadband;
  const bool directionChanged =
      !oldWasZero && !targetIsZero &&
      ((oldTarget > 0.0) != (target > 0.0));

  if (targetIsZero) {
    targetVelocity = 0.0;
    desiredPwm = 0;
    appliedPwm = 0;
    startBoostUntilMs = 0;
    controller.reset();
    return;
  }

  // 출발하거나 방향을 바꾸면: 기억을 지우고 기동 부스트를 예약한다.
  if (oldWasZero || directionChanged) {
    controller.reset();
    appliedPwm = 0;
    startBoostUntilMs = nowMs + startBoostDurationMs;
  }

  targetVelocity = target;
}

// 원본 updateOdometry 의 바퀴 하나 몫과 같은 순서.
inline double Wheel::measure(long currentCount, double dt)
{
  const long deltaCount = currentCount - previousCount;
  previousCount = currentCount;

  const double deltaDistance = static_cast<double>(deltaCount) * distancePerCount;

  measuredVelocity = deltaDistance / dt;
  filteredVelocity += velocityFilterAlpha * (measuredVelocity - filteredVelocity);

  return deltaDistance;
}

inline void Wheel::updateController(double dt)
{
  desiredPwm = controller.compute(targetVelocity, filteredVelocity, dt);
}

// 원본 updateMotorOutputs 의 바퀴 하나 몫 (하드웨어 쓰기는 .ino 가 한다).
inline int Wheel::updateOutput(uint32_t nowMs)
{
  if (fabs(targetVelocity) < commandDeadband || desiredPwm == 0) {
    appliedPwm = 0;
    return 0;
  }

  const int direction = (targetVelocity >= 0.0) ? 1 : -1;

  // 부스트 끝 시각이 아직 미래인가 — 시계가 0 으로 돌아가도 안전한 비교(부호 있는 뺄셈).
  const bool startBoostActive =
      static_cast<int32_t>(startBoostUntilMs - nowMs) > 0;

  if (startBoostActive) {
    // 출발 직후 한 번만 세게 민다 (정지마찰 극복). 주기적으로 반복하지 않는다.
    appliedPwm = direction * startBoostPwm;
  } else {
    appliedPwm = rampToward(appliedPwm, desiredPwm);
  }

  return appliedPwm;
}

inline void Wheel::stop()
{
  targetVelocity = 0.0;
  desiredPwm = 0;
  appliedPwm = 0;
  controller.reset();
  startBoostUntilMs = 0;
}

inline void Wheel::resetController()
{
  controller.reset();
}

inline void Wheel::resetMeasurement()
{
  previousCount = 0;
  measuredVelocity = 0.0;
  filteredVelocity = 0.0;
  targetVelocity = 0.0;
}

// 원본 movePwmTowardTarget 과 같은 규칙: 멈춤은 즉시, 방향 전환은 0 을 거치고,
// 멈춰 있었으면 바로 목표로, 그 밖에는 한 번에 pwmRampStep 씩.
inline int Wheel::rampToward(int currentPwm, int targetPwm) const
{
  if (targetPwm == 0) {
    return 0;
  }

  if (currentPwm != 0 && ((currentPwm > 0) != (targetPwm > 0))) {
    return 0;
  }

  if (currentPwm == 0) {
    return targetPwm;
  }

  if (currentPwm < targetPwm) {
    currentPwm += pwmRampStep;
    if (currentPwm > targetPwm) currentPwm = targetPwm;
  } else if (currentPwm > targetPwm) {
    currentPwm -= pwmRampStep;
    if (currentPwm < targetPwm) currentPwm = targetPwm;
  }

  return currentPwm;
}

inline double Wheel::getTarget() const { return targetVelocity; }
inline double Wheel::getMeasuredVelocity() const { return measuredVelocity; }
inline double Wheel::getFilteredVelocity() const { return filteredVelocity; }
inline int Wheel::getDesiredPwm() const { return desiredPwm; }
inline int Wheel::getAppliedPwm() const { return appliedPwm; }
inline uint32_t Wheel::getStartBoostUntilMs() const { return startBoostUntilMs; }
inline long Wheel::getPreviousCount() const { return previousCount; }
inline const WheelSpeedController& Wheel::getController() const { return controller; }

#endif  // WHEEL_H
