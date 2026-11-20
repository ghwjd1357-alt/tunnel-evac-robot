#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

// ============================================================================
// pid_controller.h — 바퀴 속도 제어기: PIDController(기본) → WheelSpeedController(파생)
//
// 구조 (강의 10 상속)
//   PIDController         : PID 의 게인(Kp·Ki·Kd)과 기억(적분값·이전 오차·미분 필터값),
//                           그리고 PID 의 각 단계(미분 갱신·적분 후보·각 항 계산·적분 확정)
//          ▲  상속
//   WheelSpeedController  : 위의 것을 물려받고 **피드포워드** 와 **출력 제한·와인드업 방지** 를
//                           더해 한 번의 제어 계산(compute)을 완성한다
//   강의의 Box → Cube(색깔 추가) 와 같은 모양이다. "PID 제어기에 피드포워드를 더한
//   제어기" 라는 실제 제어 구조가 그대로 클래스 관계가 된다.
//
// D 항
//   **구현은 되어 있고 꺼져 있다** (useDerivative = false, kd = 0). 그래서 실제 동작은
//   피드포워드 + PI 다. 미분 필터값은 꺼져 있어도 계속 갱신한다(원본과 같은 상태 유지).
//
// 🔴 계산 순서에 대해 — 왜 문장 순서까지 신경 쓰나
//   Teensy 컴파일러는 "곱하고 더하기" a*b + c 를 명령 하나(FMA, vfma)로 합쳐 반올림을 한 번만
//   하기도 한다(-ffp-contract=fast). 어느 곱셈을 합치는지는 **소스의 모양** 에 따라 달라지고,
//   합치느냐에 따라 결과의 마지막 비트가 달라진다. 노트북(x86-64)은 기본으로 합치지 않으므로
//   노트북 시험으로는 이 차이를 볼 수 없다.
//   그래서 이 파일의 compute() 는 원본 펌웨어(1.6.3)를 보드용으로 빌드했을 때와 **같은 연산이
//   같은 방식으로 합쳐지도록** 문장 배치를 맞췄고, 보드용 바이너리의 명령어를 원본과 직접 비교해
//   확인했다(arm-none-eabi-objdump). 문장 순서를 바꾸면 이 비교를 다시 해야 한다.
//   ⚠ 피드포워드도 같은 이유로 "max(0, 초과속도) × 기울기" 대신 "max(0, 초과속도 × 기울기)" 로
//     쓴다. 기울기가 양수이면 두 식은 모든 입력에서 비트까지 같은 값이고, 원본 컴파일러가 실제로
//     만든 계산 순서가 뒤의 것이다.
//
// Arduino 를 쓰지 않는다 → 노트북 g++ 로 컴파일·시험할 수 있다.
// ============================================================================

#include <math.h>
#include <stdint.h>

#include "drive_math.h"
#include "wheel_config.h"

// ============================================================================
// 기본 클래스: PIDController
// ============================================================================
class PIDController
{
public:
  PIDController(double kpValue, double kiValue, double kdValue,
                bool useDerivativeValue, double integralPwmLimit,
                double derivativeFilterAlphaValue);

  // 기억(적분값·이전 오차·미분 필터값)을 모두 0 으로. 정지·출발·방향 전환 때 부른다.
  void reset();

  // 관측용 (시험·진단에서 읽기만 한다)
  double getIntegralError() const;
  double getPreviousError() const;
  double getFilteredDerivative() const;

protected:
  // ── PID 한 번의 계산을 이루는 단계들. 파생 클래스가 순서대로 부른다. ──
  // 미분: (이번 오차 − 지난 오차) / dt 를 구해 저역통과 필터에 넣는다.
  void updateDerivative(double error, double dt);
  // D 항 출력 (꺼져 있으면 0).
  double derivativeTerm() const;
  // 적분 후보: 지금 적분값 + 오차 × dt, 상한으로 자른 값. 아직 확정하지 않는다.
  double integralCandidate(double error, double dt) const;
  // P 항 출력.
  double proportionalTerm(double error) const;
  // I 항 출력 (주어진 적분값으로).
  double integralTerm(double integralValue) const;
  // 적분값을 확정한다 (와인드업 판단을 통과했을 때만 부른다).
  void commitIntegral(double integralValue);
  // 이번 오차를 다음 계산을 위해 기억한다.
  void rememberError(double error);

private:
  // 게인과 설정
  double kp;
  double ki;
  double kd;
  bool useDerivative;
  double integralErrorLimit;     // 적분값 자체의 상한 = integralPwmLimit / ki
  double derivativeFilterAlpha;

  // 기억
  double integralError;          // 오차를 시간에 따라 쌓은 값 [m]
  double previousError;          // 지난 계산의 오차 [m/s]
  double filteredDerivative;     // 필터를 거친 오차 변화율
};

// ============================================================================
// 파생 클래스: WheelSpeedController — 피드포워드 + PID + 출력 제한
// ============================================================================
class WheelSpeedController : public PIDController
{
public:
  // lowSpeedHoldPwm · minRunningPwm 은 바퀴마다 다를 수 있어서 따로 받는다.
  WheelSpeedController(const WheelControlConfig& controlConfig,
                       const WheelMotionConfig& motionConfig,
                       int lowSpeedHoldPwmValue, int minRunningPwmValue);

  // 목표 속도만 보고 미리 계산하는 PWM (부호 포함).
  int feedforwardPwm(double wheelVelocity) const;

  // 한 번의 제어 계산 (20 ms 마다). 원하는 PWM(부호 포함)을 돌려준다.
  int compute(double signedTarget, double filteredVelocity, double dt);

private:
  double commandDeadband;
  double minEffectiveWheelCmd;
  double feedforwardSlope;
  int feedforwardMaxPwm;
  int maxControlPwm;
  int lowSpeedHoldPwm;
  int minRunningPwm;
};

// ── PIDController 멤버함수 정의 (강의 09: 클래스 밖에서 :: 로 정의) ──────────────

inline PIDController::PIDController(double kpValue, double kiValue, double kdValue,
                                    bool useDerivativeValue, double integralPwmLimit,
                                    double derivativeFilterAlphaValue)
    : kp(kpValue),
      ki(kiValue),
      kd(kdValue),
      useDerivative(useDerivativeValue),
      integralErrorLimit((kiValue > 0.0) ? (integralPwmLimit / kiValue) : 0.0),
      derivativeFilterAlpha(derivativeFilterAlphaValue),
      integralError(0.0),
      previousError(0.0),
      filteredDerivative(0.0)
{
}

inline void PIDController::reset()
{
  integralError = 0.0;
  previousError = 0.0;
  filteredDerivative = 0.0;
}

inline double PIDController::getIntegralError() const { return integralError; }
inline double PIDController::getPreviousError() const { return previousError; }
inline double PIDController::getFilteredDerivative() const { return filteredDerivative; }

inline void PIDController::updateDerivative(double error, double dt)
{
  const double rawDerivative = (error - previousError) / dt;
  filteredDerivative += derivativeFilterAlpha * (rawDerivative - filteredDerivative);
}

inline double PIDController::derivativeTerm() const
{
  return useDerivative ? kd * filteredDerivative : 0.0;
}

inline double PIDController::integralCandidate(double error, double dt) const
{
  double candidate = integralError + error * dt;
  if (ki > 0.0) {
    candidate = driveClamp(candidate, -integralErrorLimit, integralErrorLimit);
  } else {
    candidate = 0.0;
  }
  return candidate;
}

inline double PIDController::proportionalTerm(double error) const
{
  return kp * error;
}

inline double PIDController::integralTerm(double integralValue) const
{
  return ki * integralValue;
}

inline void PIDController::commitIntegral(double integralValue)
{
  integralError = integralValue;
}

inline void PIDController::rememberError(double error)
{
  previousError = error;
}

// ── WheelSpeedController 멤버함수 정의 ─────────────────────────────────────

inline WheelSpeedController::WheelSpeedController(const WheelControlConfig& controlConfig,
                                                  const WheelMotionConfig& motionConfig,
                                                  int lowSpeedHoldPwmValue,
                                                  int minRunningPwmValue)
    : PIDController(controlConfig.kp, controlConfig.ki, controlConfig.kd,
                    controlConfig.useDerivative, controlConfig.integralPwmLimit,
                    controlConfig.derivativeFilterAlpha),
      commandDeadband(motionConfig.commandDeadband),
      minEffectiveWheelCmd(motionConfig.minEffectiveWheelCmd),
      feedforwardSlope(controlConfig.feedforwardSlope),
      feedforwardMaxPwm(controlConfig.feedforwardMaxPwm),
      maxControlPwm(controlConfig.maxControlPwm),
      lowSpeedHoldPwm(lowSpeedHoldPwmValue),
      minRunningPwm(minRunningPwmValue)
{
}

// 피드포워드: PWM = 저속유지PWM + max(0, (|목표| − 최소유효속도) × 기울기),
//             범위 [최소구동PWM, FF상한]
inline int WheelSpeedController::feedforwardPwm(double wheelVelocity) const
{
  const double magnitude = fabs(wheelVelocity);

  if (magnitude < commandDeadband) {
    return 0;
  }

  // 기울기(feedforwardSlope)는 양수다 → max(0, x) × 기울기 == max(0, x × 기울기) (머리말 ⚠)
  const double excessPwm =
      driveMax(0.0, (magnitude - minEffectiveWheelCmd) * feedforwardSlope);

  const double pwmMagnitude = driveClamp(
      static_cast<double>(lowSpeedHoldPwm) + excessPwm,
      static_cast<double>(minRunningPwm),
      static_cast<double>(feedforwardMaxPwm));

  const int pwm = static_cast<int>(driveRound(pwmMagnitude));
  return (wheelVelocity >= 0.0) ? pwm : -pwm;
}

// 제어 계산 한 번. 🔴 문장 순서를 바꾸지 않는다 (머리말 "계산 순서에 대해").
inline int WheelSpeedController::compute(double signedTarget,
                                         double filteredVelocity,
                                         double dt)
{
  // ① 목표가 사실상 0 이면 출력 0, 기억도 지운다.
  if (fabs(signedTarget) < commandDeadband) {
    reset();
    return 0;
  }

  // ② 방향은 떼어 두고 크기로 계산한다 (뒤로 갈 때도 같은 식). 부호는 마지막에 붙인다.
  const double direction = (signedTarget >= 0.0) ? 1.0 : -1.0;
  const double targetMagnitude = fabs(signedTarget);
  const double measuredAlongCommand = direction * filteredVelocity;
  const double error = targetMagnitude - measuredAlongCommand;

  // ③ 미분 (꺼져 있어도 필터값은 계속 갱신) → D 항
  updateDerivative(error, dt);
  const double derivativeOutput = derivativeTerm();

  // ④ 적분 후보 (아직 확정하지 않는다 — 아래 와인드업 판단 뒤에 확정) → I 항 후보
  const double candidateIntegral = integralCandidate(error, dt);
  const double candidateIntegralOutput = integralTerm(candidateIntegral);

  // ⑤ 피드포워드 + P → 네 조각의 합(후보)
  const double feedforwardMagnitude =
      fabs(static_cast<double>(feedforwardPwm(signedTarget)));
  const double proportionalOutput = proportionalTerm(error);

  const double candidateMagnitude =
      feedforwardMagnitude +
      proportionalOutput +
      candidateIntegralOutput +
      derivativeOutput;

  // ⑥ 와인드업 방지(조건부 적분): 출력이 포화되지 않았거나, 포화를 풀어 주는
  //    방향일 때만 적분을 실제로 쌓는다.
  const double minimumRunningPwm = static_cast<double>(minRunningPwm);
  const double saturatedCandidate = driveClamp(
      candidateMagnitude,
      minimumRunningPwm,
      static_cast<double>(maxControlPwm));

  const bool outputNotSaturated =
      fabs(candidateMagnitude - saturatedCandidate) < 0.0001;
  const bool unwindHigh = candidateMagnitude > maxControlPwm && error < 0.0;
  const bool unwindLow = candidateMagnitude < minimumRunningPwm && error > 0.0;

  if (outputNotSaturated || unwindHigh || unwindLow) {
    commitIntegral(candidateIntegral);
  }

  // ⑦ 확정된 적분값으로 다시 합치고 [최소구동, 상한] 으로 자른다.
  const double controllerMagnitude = driveClamp(
      feedforwardMagnitude +
      proportionalOutput +
      integralTerm(getIntegralError()) +
      derivativeOutput,
      minimumRunningPwm,
      static_cast<double>(maxControlPwm));

  rememberError(error);

  // ⑧ 정수 PWM 으로 반올림하고 방향 부호를 붙인다.
  return static_cast<int>(driveRound(direction * controllerMagnitude));
}

#endif  // PID_CONTROLLER_H
