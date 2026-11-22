#ifndef SIM_CONFIG_H
#define SIM_CONFIG_H

// ============================================================================
// sim_config.h — 시뮬에서 쓰는 펌웨어 설정값
//
// 펌웨어의 설정값은 .ino 파일 안에 있다. .ino 는 Arduino·micro-ROS 를 쓰므로 노트북에서
// #include 할 수 없다 → 값만 여기에 옮겨 적었다. 이름은 .ino 와 똑같이 두었다.
// 🔴 .ino 값을 바꾸면 여기도 바꿔야 한다. 줄 번호 = firmware/term_drive_v2/term_drive_v2.ino.
// ============================================================================

#include <stdint.h>

#include "wheel_config.h"   // 설정 묶음 구조체 (펌웨어 헤더 그대로)

#ifndef PI
#define PI 3.14159265358979323846   // Arduino 에는 있지만 노트북 g++ 에는 없다
#endif

// ── 엔코더 · 바퀴 ───────────────────────────────────────────────────────────
static const double TOTAL_PPR = 2641.1;            // .ino:178  바퀴 1바퀴 = 엔코더 칸 수
static const double ODOM_WHEEL_RADIUS = 0.05698;   // .ino:207  굴림 반지름 [m]
static const double DISTANCE_PER_COUNT =           // .ino:245  엔코더 1칸 = 0.1356 mm
    (2.0 * PI * ODOM_WHEEL_RADIUS) / TOTAL_PPR;

// ── 바퀴 하나의 PWM 값 (바퀴 4개 모두 같다) ──────────────────────────────────
static const int START_BOOST_PWM = 75;             // .ino:289  출발 부스트
static const int LOW_SPEED_HOLD_PWM = 30;          // .ino:291  FF 의 기본값
static const int MIN_RUNNING_PWM = 15;             // .ino:292  최소 구동 PWM

// ── 제어 설정 (WheelControlConfig) ─────────────────────────────────────────
static const double WHEEL_KP = 60.0;               // .ino:342
static const double WHEEL_KI = 5.0;                // .ino:343
static const double WHEEL_KD = 0.0;                // .ino:344  (D 항 꺼짐)
static const bool USE_PID_D_TERM = false;          // .ino:331
static const double INTEGRAL_PWM_LIMIT = 40.0;     // .ino:349
static const double DERIVATIVE_FILTER_ALPHA = 0.10;// .ino:352
static const double FEEDFORWARD_PWM_PER_MPS_ABOVE_MIN = 335.0;  // .ino:322
static const int FEEDFORWARD_MAX_PWM = 145;        // .ino:323
static const int MAX_CONTROL_PWM = 160;            // .ino:326

// ── 바퀴 움직임 설정 (WheelMotionConfig) ───────────────────────────────────
static const double MAX_WHEEL_CMD = 0.30;          // .ino:276
static const double COMMAND_DEADBAND = 0.002;      // .ino:277
static const double MIN_EFFECTIVE_WHEEL_CMD = 0.020;  // .ino:286
static const uint32_t START_BOOST_DURATION_MS = 80;   // .ino:290
static const int PWM_RAMP_STEP = 2;                // .ino:355
static const double VELOCITY_FILTER_ALPHA = 0.10;  // .ino:351

// ── 주기 ────────────────────────────────────────────────────────────────────
static const uint32_t CONTROL_PERIOD_MS = 20;      // .ino:356 (PWM 램프) · :361 (엔코더·제어 20000 us)

// 설정 묶음 — .ino:493~514 와 같은 순서로 채운다 (순서가 밀리면 값이 조용히 바뀐다).
// 실험에서 바꿔 보는 값(kp · 적분 상한)은 인자로 받는다. 펌웨어 값 = WHEEL_KP · INTEGRAL_PWM_LIMIT.
inline WheelControlConfig makeControlConfig(double kp, double integralPwmLimit)
{
  const WheelControlConfig config = {
    kp,                                 // kp
    WHEEL_KI,                           // ki
    WHEEL_KD,                           // kd
    USE_PID_D_TERM,                     // useDerivative
    integralPwmLimit,                   // integralPwmLimit
    DERIVATIVE_FILTER_ALPHA,            // derivativeFilterAlpha
    FEEDFORWARD_PWM_PER_MPS_ABOVE_MIN,  // feedforwardSlope
    FEEDFORWARD_MAX_PWM,                // feedforwardMaxPwm
    MAX_CONTROL_PWM                     // maxControlPwm
  };
  return config;
}

// 실험에서 바꿔 보는 값(속도 필터 α)은 인자로 받는다. 펌웨어 값 = VELOCITY_FILTER_ALPHA.
inline WheelMotionConfig makeMotionConfig(double velocityFilterAlpha)
{
  const WheelMotionConfig config = {
    MAX_WHEEL_CMD,            // maxWheelCmd
    COMMAND_DEADBAND,         // commandDeadband
    MIN_EFFECTIVE_WHEEL_CMD,  // minEffectiveWheelCmd
    START_BOOST_DURATION_MS,  // startBoostDurationMs
    PWM_RAMP_STEP,            // pwmRampStep
    velocityFilterAlpha,      // velocityFilterAlpha
    DISTANCE_PER_COUNT        // distancePerCount
  };
  return config;
}

#endif  // SIM_CONFIG_H
