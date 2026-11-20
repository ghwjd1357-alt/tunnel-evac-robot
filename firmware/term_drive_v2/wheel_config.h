#ifndef WHEEL_CONFIG_H
#define WHEEL_CONFIG_H

// ============================================================================
// wheel_config.h — 구동 제어 설정값 묶음 (강의 08 구조체)
//
// 설정값은 세 묶음으로 나눈다. **값 하나는 정확히 한 묶음에만 있다** — 같은 값을 두 곳에
// 두면 한 곳만 고쳤을 때 둘이 갈라진다.
//   WheelControlConfig : 속도 제어기(PID + 피드포워드)의 게인과 한계
//   WheelMotionConfig  : 바퀴 하나의 움직임 규칙(목표 상한·데드밴드·부스트·램프·측정)
//   DriveConfig        : 로봇 몸체(직진·회전 명령 상한, 명령용 윤거)
// 값 자체와 그 근거·측정 이력 주석은 term_drive_v2.ino 의 상수 옆에 있다. .ino 가 그 상수로
// 이 구조체들을 채운다.
// ============================================================================

#include <stdint.h>

struct WheelControlConfig {
  double kp;                     // [PWM / (m/s)]   WHEEL_KP
  double ki;                     // [PWM / m]       WHEEL_KI
  double kd;                     // [PWM*s / (m/s)] WHEEL_KD
  bool useDerivative;            // USE_PID_D_TERM
  double integralPwmLimit;       // 적분항이 낼 수 있는 최대 PWM  INTEGRAL_PWM_LIMIT
  double derivativeFilterAlpha;  // 미분 필터 계수  DERIVATIVE_FILTER_ALPHA
  double feedforwardSlope;       // 피드포워드 기울기 [PWM/(m/s)] (양수)  FEEDFORWARD_PWM_PER_MPS_ABOVE_MIN
  int feedforwardMaxPwm;         // 피드포워드 상한  FEEDFORWARD_MAX_PWM
  int maxControlPwm;             // 제어 출력 상한  MAX_CONTROL_PWM
};

struct WheelMotionConfig {
  double maxWheelCmd;            // 바퀴 목표 속도 상한 [m/s]  MAX_WHEEL_CMD
  double commandDeadband;        // 이보다 작은 목표는 0  COMMAND_DEADBAND
  double minEffectiveWheelCmd;   // 0 이 아닌 목표의 최소값 [m/s]  MIN_EFFECTIVE_WHEEL_CMD
  uint32_t startBoostDurationMs; // 기동 부스트 지속 시간  START_BOOST_DURATION_MS
  int pwmRampStep;               // 램프 한 번에 바꾸는 PWM  PWM_RAMP_STEP
  double velocityFilterAlpha;    // 속도 필터 계수  VELOCITY_FILTER_ALPHA
  double distancePerCount;       // 엔코더 1 카운트의 지면 거리 [m]  DISTANCE_PER_COUNT
};

struct DriveConfig {
  double maxLinearCmd;   // 직진 명령 상한 [m/s]  MAX_LINEAR_CMD
  double maxAngularCmd;  // 회전 명령 상한 [rad/s]  MAX_ANGULAR_CMD
  double cmdWheelBase;   // 명령용 유효 윤거 [m]  CMD_WHEEL_BASE (실제 0.49 보다 큼 — 스키드 미끄러짐)
};

#endif  // WHEEL_CONFIG_H
