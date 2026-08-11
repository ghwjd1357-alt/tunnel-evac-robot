#include <Arduino.h>
#include <Encoder.h>
#include <Wire.h>
#include <math.h>
#include <stdio.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/string.h>
#include <std_srvs/srv/set_bool.h>
#include <rosidl_runtime_c/string_functions.h>

#include "rearm_gate.h"
#include "drive_wiring.h"

// ============================================================================
// Firmware identity
// Replace the zero SHA values with the real Git values before handover.
//   git rev-parse HEAD
//   git rev-parse --short=8 HEAD
// ============================================================================

#define STRINGIFY_INNER(x) #x
#define STRINGIFY(x) STRINGIFY_INNER(x)

// 🔴 2026-08-12 — 정체 문자열 정정 (docs/FIRMWARE_REBUILD.md §4 항목 1 의 부채).
// 08-06 실물 관측에서 version·source·git_sha 셋 다 실제와 달랐고, 그래서 정체 판별에
// 쓸 수 있는 필드가 build(컴파일 시각)와 매크로 2개뿐이었다. 아래 둘을 사실로 맞춘다.
static const char FW_VERSION[] = "rearm-latch-pi-continuous-low-speed-1.4.0";
// ⚠ git_sha 는 아직 0 이다 — 소스에 자기 커밋 해시를 적으면 그 편집이 다시 해시를 바꾸는
// 순환이라, 채우려면 빌드 시 주입(-DFW_GIT_SHA=...)이 필요하다. 이번 묶음의 최소 변경
// 범위 밖이므로 손대지 않고, 이 주석이 "왜 0 인지"를 대신 기록한다.
static const char FW_GIT_SHA[] = "0000000000000000000000000000000000000000";
static const char FW_GIT_SHA_SHORT[] = "00000000";
static const char FW_SOURCE_PATH[] = "firmware/teensy_integrated_base_v1_4/teensy_integrated_base_v1_4.ino";
static const char FW_LIBRARY_LIST[] =
    "micro_ros_arduino, Encoder, Adafruit_BNO055, Adafruit_Unified_Sensor, Adafruit_BusIO";

#ifdef ARDUINO
static const char FW_ARDUINO_VERSION[] = STRINGIFY(ARDUINO);
#else
static const char FW_ARDUINO_VERSION[] = "unknown";
#endif

#ifdef TEENSYDUINO
static const char FW_TEENSYDUINO_VERSION[] = STRINGIFY(TEENSYDUINO);
#else
static const char FW_TEENSYDUINO_VERSION[] = "unknown";
#endif

// ============================================================================
// micro-ROS error handling
// ============================================================================

#define RCCHECK(fn)                                   \
  do {                                                \
    const rcl_ret_t rc = (fn);                        \
    if (rc != RCL_RET_OK) {                           \
      errorLoop();                                    \
    }                                                 \
  } while (0)

#define RCSOFTCHECK(fn)                               \
  do {                                                \
    const rcl_ret_t rc = (fn);                        \
    (void)rc;                                         \
  } while (0)

// ============================================================================
// Motor and encoder mapping confirmed by previous tests
// ============================================================================

enum MotorIndex {
  FL = 0,
  RL = 1,
  FR = 2,
  RR = 3
};

static const uint8_t PWM_PIN[4] = {2, 4, 6, 8};
static const uint8_t DIR_PIN[4] = {3, 5, 7, 9};
static const uint8_t FORWARD_DIR_LEVEL[4] = {HIGH, HIGH, HIGH, HIGH};

Encoder encoderFL(10, 11);
Encoder encoderRL(12, 20);
Encoder encoderFR(14, 15);
Encoder encoderRR(16, 17);

Encoder* const encoders[4] = {
  &encoderFL,
  &encoderRL,
  &encoderFR,
  &encoderRR
};

// Forward travel must produce positive counts on all four wheels.
static const int8_t ENCODER_POLARITY[4] = {1, 1, 1, 1};

// ============================================================================
// E-stop monitor
// Active-HIGH since 2026-08-06 (ESTOP_ACTIVE_LOW = false). The pin reads the
// normally-open contact of signal relay 3: pin 21 -> relay3 [87]/[30] -> GND.
//   coil energised (button released) -> contact closed -> pin pulled to GND  -> LOW  -> running
//   button pressed / wire broken / coil supply lost -> contact open -> pull-up -> HIGH -> stopped
// Every failure therefore falls to the "stopped" side (fail-safe).
// Do NOT set this back to true: the polarity would invert and the robot would
// read "E-stop pressed" whenever it is actually running.
// This software input supplements, but does not replace, the hardwired motor
// power cutoff.
// ============================================================================

static const uint8_t ESTOP_PIN = 21;
static const bool ESTOP_ACTIVE_LOW = false;

// ============================================================================
// Robot parameters
// ============================================================================

static const double TOTAL_PPR = 2641.1;
static const double WHEEL_RADIUS = 0.05698;  // corrected rolling radius [m]
static const double WHEEL_BASE = 0.62;       // left-right wheel-center distance [m]

static const double DISTANCE_PER_COUNT =
    (2.0 * PI * WHEEL_RADIUS) / TOTAL_PPR;

// cmd_vel and wheel-speed limits.
static const double MAX_LINEAR_CMD = 0.12;   // [m/s]
static const double MAX_ANGULAR_CMD = 0.50;  // [rad/s]
static const double MAX_WHEEL_CMD = 0.15;    // [m/s]
static const double COMMAND_DEADBAND = 0.002;

// Low-speed continuous-drive model.
// The old code repeatedly switched PWM 90 ON/OFF, which caused the chassis to
// jump and stop. This version never uses periodic burst control.
//
// A non-zero command below MIN_EFFECTIVE_WHEEL_CMD is raised to that practical
// minimum. At each new start or direction change, one short start boost is
// applied, then the motor remains on with a continuous hold/feed-forward PWM.
static const double MIN_EFFECTIVE_WHEEL_CMD = 0.020;  // [m/s]

// Initial values for all four motors. Tune each wheel separately after testing.
static const int START_BOOST_PWM[4] = {75, 75, 75, 75};
static const uint32_t START_BOOST_DURATION_MS = 80;
static const int LOW_SPEED_HOLD_PWM[4] = {30, 30, 30, 30};
static const int MIN_RUNNING_PWM[4] = {15, 15, 15, 15};

// Feed-forward above 0.02 m/s:
// 0.02 m/s -> approximately HOLD_PWM
// 0.03 m/s -> approximately HOLD_PWM + 13
// 0.05 m/s -> approximately HOLD_PWM + 39
// 🔴 2026-08-12 교정 (예약 32). 구값 1300.0 은 지면 실측 대비 약 3.5배였다 — 명령 0.12 에서
// 곧바로 FEEDFORWARD_MAX_PWM(145) 로 포화했고, 그 PWM 의 실제 속도가 0.33 m/s 였다.
// PI 가 되돌릴 수 있는 폭은 WHEEL_KP*오차 + INTEGRAL_PWM_LIMIT = 약 26 PWM 뿐이라
// 119~145 PWM 에 갇혔다 = 그 구간에서 속도 제어가 사실상 열려 있었다.
// 🔴 이 값은 "교정값"이 아니라 **후보**다 (검토 §60.1). 회귀에 쓴 69/145 는 명령에서 계산한
// **명목 feedforward 값**이지 관측된 appliedPwm 이 아니다 — 실제 출력은 FF + Kp*error +
// Ki*integral 을 거치고 보드가 그 값을 발행하지 않는다. 검토자의 PI 포함 근사는 0.12 m/s 의
// 실제 출력을 ~60.5 PWM(기울기 ~305)로 본다.
// ⚠ 다만 고쳐야 할 것은 기울기의 정확도가 아니라 **포화**다: 구값에서 필요한 보정이 약
// -75 PWM 으로 PI 권한(INTEGRAL_PWM_LIMIT 기준 +-20)을 넘었다. 375 는 그 보정을 0.12 에서
// -7 PWM, 0.05 에서 -5.6 PWM 으로 만들어 **적분이 오차를 0 으로 몰 수 있는 범위**에 넣는다.
// 🔴 그래도 확정은 굽고 지면에서 0.12 = 0.12 +-10% 를 재는 것이다.
// 근거 bag = r1_ground_0811_2127, d0_drive_0811_2146 · 정본 = docs/MASTER_PLAN.md §7 예약 32.
static const double FEEDFORWARD_PWM_PER_MPS_ABOVE_MIN = 375.0;
static const int FEEDFORWARD_MAX_PWM = 145;

// Closed-loop output safety limit. Raise only after ground-speed verification.
static const int MAX_CONTROL_PWM = 160;

// Wheel-speed controller.
// false: PI control (recommended initial mode)
// true : PID control using the filtered derivative term
static const bool USE_PID_D_TERM = false;

// Conservative gains: feed-forward performs most of the drive, PI only trims.
// Units:
//   KP: PWM / (m/s)
//   KI: PWM / m
//   KD: PWM*s / (m/s)
static const double WHEEL_KP = 30.0;
static const double WHEEL_KI = 5.0;
static const double WHEEL_KD = 0.0;

// Integral and derivative protection.
static const double INTEGRAL_PWM_LIMIT = 20.0;
static const double VELOCITY_FILTER_ALPHA = 0.10;
static const double DERIVATIVE_FILTER_ALPHA = 0.10;

// Slow output slew prevents abrupt chassis jolts after the one-time start boost.
static const int PWM_RAMP_STEP = 2;
static const uint32_t PWM_RAMP_INTERVAL_MS = 20;

static const uint32_t WATCHDOG_TIMEOUT_MS = 500;

// Encoder odometry/control target period. Actual dt is measured by micros().
static const uint32_t ODOM_PERIOD_US = 20000;

// ============================================================================
// Re-arm latch (2026-08-11) — contract: docs/REAL_ROBOT_VALUES.md §1-f
//
// Clearing the E-stop must not by itself put the robot back under command. A
// publisher that never stopped would otherwise have its next message accepted
// as a fresh command, which is the hazard this latch exists for. Arming needs
// three things in order: the E-stop released, zero commands held continuously
// for REARM_ZERO_HOLD_MS, and an explicit /drive/enable service call.
//
// The latch lives in the firmware rather than in a ROS node because the Teensy
// subscribes to /cmd_vel directly — a node-side gate is bypassed by anyone
// publishing to that topic.
//
// It does NOT replace the watchdog. WATCHDOG_TIMEOUT_MS still stops the motors
// on command dropout while armed; see checkSafety().
//
// 🔴 The transition table itself is NOT here — it is rearm_gate.h, a pure header
// with no Arduino dependency, so tools/rearm_gate_host_test.cpp can drive every
// transition on a PC with no board and no waiting. What lives in this file is
// only the wiring: read the clock, ask the gate, and act on the answer.
// (2026-08-11, review §54.7 — states/rejects/timings all moved into that header.)
//
// 🔴 The motor-stop side of each transition is not here either — it is
// drive_wiring.h (2026-08-11, review §55.2). Keeping it in this file meant the
// host test could not see it, so reverting the safety wiring left the gate green.
// Everything below asks drive_wiring.h and does what it is told; the only thing
// this file still owns is what a stop physically means on this board.
// ============================================================================

// ============================================================================
// BNO055 IMU configuration
// Teensy 4.1 Wire: SDA=18, SCL=19
// ============================================================================

static const uint8_t IMU_SDA_PIN = 18;
static const uint8_t IMU_SCL_PIN = 19;

Adafruit_BNO055 bnoAddress28(55, 0x28, &Wire);
Adafruit_BNO055 bnoAddress29(56, 0x29, &Wire);
Adafruit_BNO055* bno = nullptr;

// Verified sign: counter-clockwise/left turn -> positive angular_velocity.z.
static const double IMU_X_SIGN = 1.0;
static const double IMU_Y_SIGN = 1.0;
static const double IMU_Z_SIGN = 1.0;

// Setting is 20 ms; measured output was approximately 41.63 Hz in the current build.
static const uint32_t IMU_PERIOD_US = 20000;
static const int GYRO_BIAS_SAMPLE_COUNT = 500;
static const uint32_t GYRO_BIAS_SAMPLE_DELAY_MS = 10;
static const double GYRO_ZERO_THRESHOLD_RAD_S = 0.005;

// Diagnostic and time-sync periods
static const uint32_t DIAGNOSTIC_PERIOD_MS = 1000;
static const uint32_t FW_INFO_PERIOD_MS = 5000;
static const uint32_t TIME_SYNC_PERIOD_MS = 30000;

// ============================================================================
// micro-ROS objects
// No TF publisher is created in this firmware.
// ============================================================================

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

rcl_subscription_t cmdVelSubscriber;
rcl_subscription_t resetOdomSubscriber;
rcl_subscription_t resetYawSubscriber;

rcl_publisher_t odomPublisher;
rcl_publisher_t imuPublisher;
rcl_publisher_t imuYawPublisher;
rcl_publisher_t gyroBiasPublisher;
rcl_publisher_t estopStatePublisher;
rcl_publisher_t firmwareInfoPublisher;
rcl_publisher_t driveEnabledPublisher;
rcl_publisher_t driveDiagPublisher;

// Only one service slot exists (RMW_UXRCE_MAX_SERVICES is exactly 1), so the
// diagnostic counters ride a publisher instead of a second service.
rcl_service_t driveEnableService;
std_srvs__srv__SetBool_Request driveEnableRequest;
std_srvs__srv__SetBool_Response driveEnableResponse;

geometry_msgs__msg__Twist cmdVelMessage;
std_msgs__msg__Bool resetOdomMessage;
std_msgs__msg__Bool resetYawMessage;

nav_msgs__msg__Odometry odomMessage;
sensor_msgs__msg__Imu imuMessage;
std_msgs__msg__Float64 imuYawMessage;
geometry_msgs__msg__Vector3 gyroBiasMessage;
std_msgs__msg__Bool estopStateMessage;
std_msgs__msg__String firmwareInfoMessage;
std_msgs__msg__Bool driveEnabledMessage;
geometry_msgs__msg__Vector3 driveDiagMessage;

// ============================================================================
// Runtime state
// ============================================================================

int desiredPwm[4] = {0, 0, 0, 0};
int appliedPwm[4] = {0, 0, 0, 0};

double targetWheelVelocity[4] = {0.0, 0.0, 0.0, 0.0};
double measuredWheelVelocity[4] = {0.0, 0.0, 0.0, 0.0};
double filteredWheelVelocity[4] = {0.0, 0.0, 0.0, 0.0};

double wheelIntegralError[4] = {0.0, 0.0, 0.0, 0.0};
double wheelPreviousError[4] = {0.0, 0.0, 0.0, 0.0};
double wheelFilteredDerivative[4] = {0.0, 0.0, 0.0, 0.0};

bool cmdVelReceived = false;
bool imuInitialized = false;
bool gyroBiasCalibrated = false;

// Re-arm latch state. One struct, so there is exactly one place a transition
// can be written and exactly one place the host test has to construct.
RearmGate driveGate;

uint32_t lastCmdVelMs = 0;
uint32_t lastPwmUpdateMs = 0;
uint32_t lastDiagnosticMs = 0;
uint32_t lastFirmwareInfoMs = 0;
uint32_t lastTimeSyncMs = 0;
uint32_t startBoostUntilMs[4] = {0, 0, 0, 0};

long previousCount[4] = {0, 0, 0, 0};
uint32_t previousOdomUs = 0;
uint32_t previousImuUs = 0;

double odomX = 0.0;
double odomY = 0.0;
double odomYaw = 0.0;
double odomLinearVelocity = 0.0;
double odomAngularVelocity = 0.0;

double gyroBiasX = 0.0;
double gyroBiasY = 0.0;
double gyroBiasZ = 0.0;
double imuYawUnwrappedRad = 0.0;

uint64_t lastPublishedTimestampNs = 0;

// ============================================================================
// Forward declarations
// ============================================================================

void errorLoop();
void stopAllMotors();
void resetWheelControllerState(int motor);
void resetAllWheelControllers();
void updateWheelControllers(double dt);
void resetOdometry();
void resetImuYaw();
void publishOdometry(uint64_t timestampNs);
void publishImu(uint64_t timestampNs,
                double gyroX,
                double gyroY,
                double gyroZ,
                double accelX,
                double accelY,
                double accelZ);

// ============================================================================
// Drive sink — what a stop means on this board (review §55.2)
// ============================================================================
//
// drive_wiring.h decides *when* to stop; this decides *what* stopping is. The
// host test substitutes FakeDriveSink for this struct and gets the same
// decisions, so reverting either enforcement point now fails on a PC.
//
// It is three one-line forwarders on purpose: anything with a branch in here
// would be logic that the host test cannot see again.
struct TeensyDriveSink {
  void stopAllMotors() { ::stopAllMotors(); }
  void setCmdVelReceived(bool value) { cmdVelReceived = value; }
  void noteCommandAccepted(uint32_t nowMs)
  {
    lastCmdVelMs = nowMs;
    cmdVelReceived = true;
  }
};

TeensyDriveSink driveSink;

// ============================================================================
// Utilities
// ============================================================================

double clampDouble(double value, double minimum, double maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

double normalizeAngle(double angle)
{
  while (angle > PI) angle -= 2.0 * PI;
  while (angle < -PI) angle += 2.0 * PI;
  return angle;
}

double applyGyroDeadband(double value)
{
  return (fabs(value) < GYRO_ZERO_THRESHOLD_RAD_S) ? 0.0 : value;
}

bool isEstopActive()
{
  const bool rawHigh = digitalRead(ESTOP_PIN) == HIGH;
  return ESTOP_ACTIVE_LOW ? !rawHigh : rawHigh;
}

uint64_t getMonotonicTimestampNs()
{
  uint64_t nowNs = rmw_uros_epoch_nanos();

  if (nowNs == 0) {
    nowNs = static_cast<uint64_t>(millis()) * 1000000ULL;
  }

  if (nowNs <= lastPublishedTimestampNs) {
    nowNs = lastPublishedTimestampNs + 1ULL;
  }

  lastPublishedTimestampNs = nowNs;
  return nowNs;
}

void setRosTime(builtin_interfaces__msg__Time* stamp, uint64_t timestampNs)
{
  stamp->sec = static_cast<int32_t>(timestampNs / 1000000000ULL);
  stamp->nanosec = static_cast<uint32_t>(timestampNs % 1000000000ULL);
}

// ============================================================================
// Motor control
// ============================================================================

void writeMotorPwm(int motor, int signedPwm)
{
  signedPwm = constrain(signedPwm, -255, 255);

  if (signedPwm == 0 || isEstopActive()) {
    analogWrite(PWM_PIN[motor], 0);
    return;
  }

  const bool forward = signedPwm > 0;
  const uint8_t directionLevel =
      forward ? FORWARD_DIR_LEVEL[motor] : !FORWARD_DIR_LEVEL[motor];

  digitalWrite(DIR_PIN[motor], directionLevel);
  analogWrite(PWM_PIN[motor], abs(signedPwm));
}

void resetWheelControllerState(int motor)
{
  wheelIntegralError[motor] = 0.0;
  wheelPreviousError[motor] = 0.0;
  wheelFilteredDerivative[motor] = 0.0;
}

void resetAllWheelControllers()
{
  for (int motor = 0; motor < 4; ++motor) {
    resetWheelControllerState(motor);
  }
}

void stopAllMotors()
{
  for (int motor = 0; motor < 4; ++motor) {
    targetWheelVelocity[motor] = 0.0;
    desiredPwm[motor] = 0;
    appliedPwm[motor] = 0;
    resetWheelControllerState(motor);
    startBoostUntilMs[motor] = 0;
    analogWrite(PWM_PIN[motor], 0);
  }
}

int wheelVelocityToFeedforwardPwm(int motor, double wheelVelocity)
{
  const double magnitude = fabs(wheelVelocity);

  if (magnitude < COMMAND_DEADBAND) {
    return 0;
  }

  const double excessSpeed = max(
      0.0,
      magnitude - MIN_EFFECTIVE_WHEEL_CMD);

  const double pwmMagnitude = clampDouble(
      static_cast<double>(LOW_SPEED_HOLD_PWM[motor]) +
      excessSpeed * FEEDFORWARD_PWM_PER_MPS_ABOVE_MIN,
      static_cast<double>(MIN_RUNNING_PWM[motor]),
      static_cast<double>(FEEDFORWARD_MAX_PWM));

  const int pwm = static_cast<int>(round(pwmMagnitude));
  return (wheelVelocity >= 0.0) ? pwm : -pwm;
}

void setTargetWheelVelocity(int motor, double target)
{
  target = clampDouble(target, -MAX_WHEEL_CMD, MAX_WHEEL_CMD);

  const double oldTarget = targetWheelVelocity[motor];
  const bool targetIsZero = fabs(target) < COMMAND_DEADBAND;

  if (!targetIsZero && fabs(target) < MIN_EFFECTIVE_WHEEL_CMD) {
    target = (target >= 0.0)
             ? MIN_EFFECTIVE_WHEEL_CMD
             : -MIN_EFFECTIVE_WHEEL_CMD;
  }

  const bool oldWasZero = fabs(oldTarget) < COMMAND_DEADBAND;
  const bool directionChanged =
      !oldWasZero && !targetIsZero &&
      ((oldTarget > 0.0) != (target > 0.0));

  if (targetIsZero) {
    targetWheelVelocity[motor] = 0.0;
    desiredPwm[motor] = 0;
    appliedPwm[motor] = 0;
    startBoostUntilMs[motor] = 0;
    resetWheelControllerState(motor);
    return;
  }

  if (oldWasZero || directionChanged) {
    resetWheelControllerState(motor);
    appliedPwm[motor] = 0;
    startBoostUntilMs[motor] = millis() + START_BOOST_DURATION_MS;
  }

  targetWheelVelocity[motor] = target;
}

void applySkidSteerCommand(double linearX, double angularZ)
{
  linearX = clampDouble(linearX, -MAX_LINEAR_CMD, MAX_LINEAR_CMD);
  angularZ = clampDouble(angularZ, -MAX_ANGULAR_CMD, MAX_ANGULAR_CMD);

  double leftWheelCommand = linearX - angularZ * WHEEL_BASE * 0.5;
  double rightWheelCommand = linearX + angularZ * WHEEL_BASE * 0.5;

  const double maximumMagnitude =
      max(fabs(leftWheelCommand), fabs(rightWheelCommand));

  if (maximumMagnitude > MAX_WHEEL_CMD) {
    const double scale = MAX_WHEEL_CMD / maximumMagnitude;
    leftWheelCommand *= scale;
    rightWheelCommand *= scale;
  }

  setTargetWheelVelocity(FL, leftWheelCommand);
  setTargetWheelVelocity(RL, leftWheelCommand);
  setTargetWheelVelocity(FR, rightWheelCommand);
  setTargetWheelVelocity(RR, rightWheelCommand);
}

void updateWheelControllers(double dt)
{
  if (dt <= 0.0 || dt > 0.5 || isEstopActive() || !cmdVelReceived) {
    stopAllMotors();
    return;
  }

  const double integralErrorLimit =
      (WHEEL_KI > 0.0)
      ? (INTEGRAL_PWM_LIMIT / WHEEL_KI)
      : 0.0;

  for (int motor = 0; motor < 4; ++motor) {
    const double signedTarget = targetWheelVelocity[motor];

    if (fabs(signedTarget) < COMMAND_DEADBAND) {
      desiredPwm[motor] = 0;
      resetWheelControllerState(motor);
      continue;
    }

    // Perform the controller calculation in the commanded direction.
    // This prevents a large overspeed correction from commanding reverse PWM.
    const double direction = (signedTarget >= 0.0) ? 1.0 : -1.0;
    const double targetMagnitude = fabs(signedTarget);
    const double measuredAlongCommand =
        direction * filteredWheelVelocity[motor];
    const double error = targetMagnitude - measuredAlongCommand;

    const double rawDerivative =
        (error - wheelPreviousError[motor]) / dt;

    wheelFilteredDerivative[motor] +=
        DERIVATIVE_FILTER_ALPHA *
        (rawDerivative - wheelFilteredDerivative[motor]);

    double candidateIntegral =
        wheelIntegralError[motor] + error * dt;

    if (WHEEL_KI > 0.0) {
      candidateIntegral = clampDouble(
          candidateIntegral,
          -integralErrorLimit,
          integralErrorLimit);
    } else {
      candidateIntegral = 0.0;
    }

    const double feedforwardMagnitude =
        fabs(static_cast<double>(
            wheelVelocityToFeedforwardPwm(motor, signedTarget)));

    const double proportionalOutput = WHEEL_KP * error;
    const double derivativeOutput =
        USE_PID_D_TERM
        ? WHEEL_KD * wheelFilteredDerivative[motor]
        : 0.0;
    const double candidateIntegralOutput =
        WHEEL_KI * candidateIntegral;

    const double candidateMagnitude =
        feedforwardMagnitude +
        proportionalOutput +
        candidateIntegralOutput +
        derivativeOutput;

    const double minimumRunningPwm =
        static_cast<double>(MIN_RUNNING_PWM[motor]);

    const double saturatedCandidate = clampDouble(
        candidateMagnitude,
        minimumRunningPwm,
        static_cast<double>(MAX_CONTROL_PWM));

    // Conditional-integration anti-windup with a continuous-running floor.
    const bool outputNotSaturated =
        fabs(candidateMagnitude - saturatedCandidate) < 0.0001;
    const bool unwindHigh =
        candidateMagnitude > MAX_CONTROL_PWM && error < 0.0;
    const bool unwindLow =
        candidateMagnitude < minimumRunningPwm && error > 0.0;

    if (outputNotSaturated || unwindHigh || unwindLow) {
      wheelIntegralError[motor] = candidateIntegral;
    }

    const double controllerMagnitude = clampDouble(
        feedforwardMagnitude +
        proportionalOutput +
        WHEEL_KI * wheelIntegralError[motor] +
        derivativeOutput,
        minimumRunningPwm,
        static_cast<double>(MAX_CONTROL_PWM));

    desiredPwm[motor] = static_cast<int>(
        round(direction * controllerMagnitude));

    wheelPreviousError[motor] = error;
  }
}

int movePwmTowardTarget(int currentPwm, int targetPwm)
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
    currentPwm += PWM_RAMP_STEP;
    if (currentPwm > targetPwm) currentPwm = targetPwm;
  } else if (currentPwm > targetPwm) {
    currentPwm -= PWM_RAMP_STEP;
    if (currentPwm < targetPwm) currentPwm = targetPwm;
  }

  return currentPwm;
}

void updateMotorOutputs()
{
  // 🔴 Review §54.1 — the invariant is enforced at the output stage, not only on
  // the paths that enter DISARMED. Every earlier version relied on each caller
  // remembering to stop, and the one caller that forgot (the false branch of
  // /drive/enable) let the motors keep their PWM until the watchdog expired.
  // Motion is now structurally impossible outside DRIVE_ARMED: even a path that
  // forgets to stop cannot produce output, because this is the only writer.
  // 🔴 Review §55.2 — the guard itself lives in drive_wiring.h so that deleting
  // it is a host-test failure and not a silent one.
  if (!driveOutputAllowed(&driveGate, isEstopActive(), driveSink)) {
    return;
  }

  const uint32_t nowMs = millis();
  if (nowMs - lastPwmUpdateMs < PWM_RAMP_INTERVAL_MS) {
    return;
  }
  lastPwmUpdateMs = nowMs;

  for (int motor = 0; motor < 4; ++motor) {
    const double target = targetWheelVelocity[motor];

    if (fabs(target) < COMMAND_DEADBAND || desiredPwm[motor] == 0) {
      appliedPwm[motor] = 0;
      writeMotorPwm(motor, 0);
      continue;
    }

    const int direction = (target >= 0.0) ? 1 : -1;
    const bool startBoostActive =
        static_cast<int32_t>(startBoostUntilMs[motor] - nowMs) > 0;

    if (startBoostActive) {
      // One start boost only. It is not periodically repeated.
      appliedPwm[motor] = direction * START_BOOST_PWM[motor];
    } else {
      appliedPwm[motor] = movePwmTowardTarget(
          appliedPwm[motor], desiredPwm[motor]);
    }

    writeMotorPwm(motor, appliedPwm[motor]);
  }
}

// 🔴 Review §54.1 — stopping is part of entering DISARMED, not a separate step
// the caller has to remember. Callers that skipped it left the motors running to
// the watchdog while the state topic already read false. Both effects now happen
// in one function, so all call sites get the invariant whether they knew it or not.
// 🔴 Review §55.2 — that function is driveDisarm() in drive_wiring.h, not this
// one. This is a name the sketch already used; the invariant it carries is now
// somewhere the host test can revert and observe.
void disarmDrive()
{
  driveDisarm(&driveGate, driveSink);
}

void checkSafety()
{
  if (isEstopActive()) {
    // The E-stop latches the drive off. Releasing the button does not undo
    // this by itself — the zero hold, the service, and the quiet barrier do.
    // disarmDrive() stops the motors as part of the transition.
    disarmDrive();
    return;
  }

  // The post-response quiet barrier has to finish even when the publisher went
  // completely silent, so it cannot live in the cmd_vel callback (§54.2).
  rearmGateTick(&driveGate, millis());

  if (!cmdVelReceived) {
    stopAllMotors();
    return;
  }

  if (millis() - lastCmdVelMs >= WATCHDOG_TIMEOUT_MS) {
    stopAllMotors();
    cmdVelReceived = false;
  }
}

// ============================================================================
// Encoder odometry using measured micros() dt
// ============================================================================

long readEncoderCount(int motor)
{
  return encoders[motor]->read() * ENCODER_POLARITY[motor];
}

void resetOdometry()
{
  stopAllMotors();

  noInterrupts();
  for (int motor = 0; motor < 4; ++motor) {
    encoders[motor]->write(0);
  }
  interrupts();

  for (int motor = 0; motor < 4; ++motor) {
    previousCount[motor] = 0;
    measuredWheelVelocity[motor] = 0.0;
    filteredWheelVelocity[motor] = 0.0;
    targetWheelVelocity[motor] = 0.0;
  }

  resetAllWheelControllers();

  odomX = 0.0;
  odomY = 0.0;
  odomYaw = 0.0;
  odomLinearVelocity = 0.0;
  odomAngularVelocity = 0.0;
  previousOdomUs = micros();
  cmdVelReceived = false;
}

void updateOdometry()
{
  const uint32_t currentUs = micros();
  const uint32_t elapsedUs = currentUs - previousOdomUs;

  if (elapsedUs < ODOM_PERIOD_US) {
    return;
  }

  // Required handover method: measured dt, never a fixed 0.02 s.
  const double dt = static_cast<double>(elapsedUs) / 1000000.0;
  previousOdomUs = currentUs;

  if (dt <= 0.0 || dt > 0.5) {
    odomLinearVelocity = 0.0;
    odomAngularVelocity = 0.0;
    return;
  }

  long currentCount[4];
  long deltaCount[4];

  for (int motor = 0; motor < 4; ++motor) {
    currentCount[motor] = readEncoderCount(motor);
    deltaCount[motor] = currentCount[motor] - previousCount[motor];
    previousCount[motor] = currentCount[motor];
  }

  const double deltaFL = static_cast<double>(deltaCount[FL]) * DISTANCE_PER_COUNT;
  const double deltaRL = static_cast<double>(deltaCount[RL]) * DISTANCE_PER_COUNT;
  const double deltaFR = static_cast<double>(deltaCount[FR]) * DISTANCE_PER_COUNT;
  const double deltaRR = static_cast<double>(deltaCount[RR]) * DISTANCE_PER_COUNT;

  const double wheelDeltaDistance[4] = {
    deltaFL,
    deltaRL,
    deltaFR,
    deltaRR
  };

  for (int motor = 0; motor < 4; ++motor) {
    measuredWheelVelocity[motor] =
        wheelDeltaDistance[motor] / dt;

    filteredWheelVelocity[motor] +=
        VELOCITY_FILTER_ALPHA *
        (measuredWheelVelocity[motor] -
         filteredWheelVelocity[motor]);
  }

  const double deltaLeft = 0.5 * (deltaFL + deltaRL);
  const double deltaRight = 0.5 * (deltaFR + deltaRR);
  const double deltaDistance = 0.5 * (deltaLeft + deltaRight);
  const double deltaYaw = (deltaRight - deltaLeft) / WHEEL_BASE;

  const double midpointYaw = odomYaw + 0.5 * deltaYaw;
  odomX += deltaDistance * cos(midpointYaw);
  odomY += deltaDistance * sin(midpointYaw);
  odomYaw = normalizeAngle(odomYaw + deltaYaw);

  const double filteredLeftVelocity = 0.5 * (
      filteredWheelVelocity[FL] + filteredWheelVelocity[RL]);
  const double filteredRightVelocity = 0.5 * (
      filteredWheelVelocity[FR] + filteredWheelVelocity[RR]);

  // Pose integration uses raw encoder increments. Published twist is filtered
  // to reduce encoder quantization noise at low continuous wheel speeds.
  odomLinearVelocity =
      0.5 * (filteredLeftVelocity + filteredRightVelocity);
  odomAngularVelocity =
      (filteredRightVelocity - filteredLeftVelocity) / WHEEL_BASE;

  // PI/PID control uses the same measured micros() dt as odometry.
  updateWheelControllers(dt);

  publishOdometry(getMonotonicTimestampNs());
}

void publishOdometry(uint64_t timestampNs)
{
  setRosTime(&odomMessage.header.stamp, timestampNs);

  odomMessage.pose.pose.position.x = odomX;
  odomMessage.pose.pose.position.y = odomY;
  odomMessage.pose.pose.position.z = 0.0;

  odomMessage.pose.pose.orientation.x = 0.0;
  odomMessage.pose.pose.orientation.y = 0.0;
  odomMessage.pose.pose.orientation.z = sin(odomYaw * 0.5);
  odomMessage.pose.pose.orientation.w = cos(odomYaw * 0.5);

  odomMessage.twist.twist.linear.x = odomLinearVelocity;
  odomMessage.twist.twist.linear.y = 0.0;
  odomMessage.twist.twist.linear.z = 0.0;
  odomMessage.twist.twist.angular.x = 0.0;
  odomMessage.twist.twist.angular.y = 0.0;
  odomMessage.twist.twist.angular.z = odomAngularVelocity;

  RCSOFTCHECK(rcl_publish(&odomPublisher, &odomMessage, nullptr));
}

// ============================================================================
// IMU initialization, gyro bias, and publishing
// ============================================================================

void initializeImu()
{
  Wire.setSDA(IMU_SDA_PIN);
  Wire.setSCL(IMU_SCL_PIN);
  Wire.begin();
  Wire.setClock(100000);
  delay(700);

  if (bnoAddress28.begin(OPERATION_MODE_AMG)) {
    bno = &bnoAddress28;
    imuInitialized = true;
  } else if (bnoAddress29.begin(OPERATION_MODE_AMG)) {
    bno = &bnoAddress29;
    imuInitialized = true;
  } else {
    imuInitialized = false;
    errorLoop();
  }

  delay(1000);
}

void calibrateGyroBias()
{
  stopAllMotors();
  gyroBiasCalibrated = false;

  double sumX = 0.0;
  double sumY = 0.0;
  double sumZ = 0.0;
  int validSamples = 0;

  sensors_event_t gyroEvent;

  for (int sample = 0; sample < GYRO_BIAS_SAMPLE_COUNT; ++sample) {
    if (bno->getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE)) {
      const double gx = IMU_X_SIGN * gyroEvent.gyro.x;
      const double gy = IMU_Y_SIGN * gyroEvent.gyro.y;
      const double gz = IMU_Z_SIGN * gyroEvent.gyro.z;

      if (isfinite(gx) && isfinite(gy) && isfinite(gz)) {
        sumX += gx;
        sumY += gy;
        sumZ += gz;
        ++validSamples;
      }
    }

    if ((sample % 25) == 0) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }

    delay(GYRO_BIAS_SAMPLE_DELAY_MS);
  }

  digitalWrite(LED_BUILTIN, LOW);

  if (validSamples < 100) {
    errorLoop();
  }

  gyroBiasX = sumX / static_cast<double>(validSamples);
  gyroBiasY = sumY / static_cast<double>(validSamples);
  gyroBiasZ = sumZ / static_cast<double>(validSamples);
  gyroBiasCalibrated = true;
  resetImuYaw();
}

void resetImuYaw()
{
  imuYawUnwrappedRad = 0.0;
  previousImuUs = micros();
}

void updateImu()
{
  if (!imuInitialized || !gyroBiasCalibrated) {
    return;
  }

  const uint32_t currentUs = micros();
  const uint32_t elapsedUs = currentUs - previousImuUs;

  if (elapsedUs < IMU_PERIOD_US) {
    return;
  }

  const double dt = static_cast<double>(elapsedUs) / 1000000.0;
  previousImuUs = currentUs;

  if (dt <= 0.0 || dt > 0.5) {
    return;
  }

  sensors_event_t gyroEvent;
  sensors_event_t accelEvent;

  const bool gyroOk =
      bno->getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);
  const bool accelOk =
      bno->getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  if (!gyroOk || !accelOk) {
    stopAllMotors();
    return;
  }

  double gyroX = IMU_X_SIGN * gyroEvent.gyro.x - gyroBiasX;
  double gyroY = IMU_Y_SIGN * gyroEvent.gyro.y - gyroBiasY;
  double gyroZ = IMU_Z_SIGN * gyroEvent.gyro.z - gyroBiasZ;

  gyroX = applyGyroDeadband(gyroX);
  gyroY = applyGyroDeadband(gyroY);
  gyroZ = applyGyroDeadband(gyroZ);

  const double accelX = IMU_X_SIGN * accelEvent.acceleration.x;
  const double accelY = IMU_Y_SIGN * accelEvent.acceleration.y;
  const double accelZ = IMU_Z_SIGN * accelEvent.acceleration.z;

  imuYawUnwrappedRad += gyroZ * dt;

  publishImu(
      getMonotonicTimestampNs(),
      gyroX,
      gyroY,
      gyroZ,
      accelX,
      accelY,
      accelZ);
}

void publishImu(uint64_t timestampNs,
                double gyroX,
                double gyroY,
                double gyroZ,
                double accelX,
                double accelY,
                double accelZ)
{
  setRosTime(&imuMessage.header.stamp, timestampNs);

  // Orientation is intentionally marked unavailable. EKF should fuse the
  // measured angular velocity. Relative yaw is published separately for tests.
  imuMessage.orientation.x = 0.0;
  imuMessage.orientation.y = 0.0;
  imuMessage.orientation.z = 0.0;
  imuMessage.orientation.w = 1.0;

  imuMessage.angular_velocity.x = gyroX;
  imuMessage.angular_velocity.y = gyroY;
  imuMessage.angular_velocity.z = gyroZ;

  imuMessage.linear_acceleration.x = accelX;
  imuMessage.linear_acceleration.y = accelY;
  imuMessage.linear_acceleration.z = accelZ;

  imuYawMessage.data = imuYawUnwrappedRad * 180.0 / PI;

  RCSOFTCHECK(rcl_publish(&imuPublisher, &imuMessage, nullptr));
  RCSOFTCHECK(rcl_publish(&imuYawPublisher, &imuYawMessage, nullptr));
}

// ============================================================================
// ROS callbacks
// ============================================================================

void cmdVelCallback(const void* messageInput)
{
  const auto* message =
      static_cast<const geometry_msgs__msg__Twist*>(messageInput);

  if (isEstopActive()) {
    disarmDrive();
    return;
  }

  const double linearX = message->linear.x;
  const double angularZ = message->angular.z;
  const uint32_t nowMs = millis();

  // The whole decision — including the non-finite case, the zero hold, and the
  // post-response quiet barrier — is rearm_gate.h, and the stop that goes with a
  // rejected command is drive_wiring.h. Anything but true has already stopped.
  if (!driveOnCommand(&driveGate, linearX, angularZ, nowMs, driveSink)) {
    return;
  }

  applySkidSteerCommand(linearX, angularZ);
}

// /drive/enable — the explicit arming step. std_srvs/SetBool: data=true enters
// ARMING (it does NOT arm on its own — see §54.2 and §55.1 in rearm_gate.h),
// data=false disarms. The response message string is intentionally left empty:
// assigning it would allocate on every call, and the reason code is published on
// /drive/diag where it can be watched without polling the service.
//
// 🔴 Review §55.1 — there is no millis() in this function any more. rclc calls
// rcl_send_response only after this callback returns, so any clock started here
// runs before the response goes out and shortens the barrier by that much. The
// barrier clock starts in loop(), after spin_some() has done the sending.
void driveEnableCallback(const void* requestInput, void* responseOutput)
{
  const auto* request =
      static_cast<const std_srvs__srv__SetBool_Request*>(requestInput);
  auto* response =
      static_cast<std_srvs__srv__SetBool_Response*>(responseOutput);

  // 🔴 Review §54.1 — the cleanup happens before the response is written, not
  // after. An operator who reads success:true on a disable has to be able to
  // treat it as "the motors are already at zero", and the only way to promise
  // that is to do the stopping first. driveOnServiceRequest() owns that order
  // (review §55.2) so this file cannot get it wrong later.
  const bool success = driveOnServiceRequest(
      &driveGate, request->data, isEstopActive(), driveSink);

  response->success = success;
}

void resetOdomCallback(const void* messageInput)
{
  const auto* message =
      static_cast<const std_msgs__msg__Bool*>(messageInput);

  if (message->data) {
    resetOdometry();
  }
}

void resetYawCallback(const void* messageInput)
{
  const auto* message =
      static_cast<const std_msgs__msg__Bool*>(messageInput);

  if (message->data) {
    resetImuYaw();
  }
}

// ============================================================================
// Diagnostics, E-stop state, firmware information, and time sync
// ============================================================================

void publishDiagnostics()
{
  const uint32_t nowMs = millis();

  if (nowMs - lastDiagnosticMs < DIAGNOSTIC_PERIOD_MS) {
    return;
  }
  lastDiagnosticMs = nowMs;

  gyroBiasMessage.x = gyroBiasX;
  gyroBiasMessage.y = gyroBiasY;
  gyroBiasMessage.z = gyroBiasZ;
  estopStateMessage.data = isEstopActive();

  // Wire contract (canon = docs/REAL_ROBOT_VALUES.md §1-f):
  //   /drive/enabled  std_msgs/Bool        data = (state == ARMED)
  //   /drive/diag     geometry_msgs/Vector3  x = service call count
  //                                          y = DriveReject
  //                                          z = DriveState (0/1/2/3)
  // Two topics rather than one: Bool cannot carry the counters, and the counter
  // is what separates "the request never arrived" from "the logic refused it".
  driveEnabledMessage.data = (driveGate.state == DRIVE_ARMED);
  driveDiagMessage.x = static_cast<double>(driveGate.serviceCalls);
  driveDiagMessage.y = static_cast<double>(driveGate.rejectReason);
  driveDiagMessage.z = static_cast<double>(driveGate.state);

  RCSOFTCHECK(rcl_publish(&gyroBiasPublisher, &gyroBiasMessage, nullptr));
  RCSOFTCHECK(rcl_publish(&estopStatePublisher, &estopStateMessage, nullptr));
  RCSOFTCHECK(rcl_publish(&driveEnabledPublisher, &driveEnabledMessage, nullptr));
  RCSOFTCHECK(rcl_publish(&driveDiagPublisher, &driveDiagMessage, nullptr));
}

void publishFirmwareInfo()
{
  const uint32_t nowMs = millis();

  if (nowMs - lastFirmwareInfoMs < FW_INFO_PERIOD_MS) {
    return;
  }
  lastFirmwareInfoMs = nowMs;

  char infoBuffer[1024];
  snprintf(
      infoBuffer,
      sizeof(infoBuffer),
      "version=%s; git_sha=%s; git_short=%s; build=%s %s; source=%s; "
      "arduino_macro=%s; teensyduino_macro=%s; transport=serial; baud=115200; "
      "wheel_radius=%.5f; control=%s; kp=%.3f; ki=%.3f; kd=%.3f; "
      "low_speed_mode=continuous_start_boost; min_speed=%.3f; "
      "start_boost_ms=%lu; hold_pwm=%d,%d,%d,%d; encoder_polarity=%d,%d,%d,%d; "
      "libraries=%s",
      FW_VERSION,
      FW_GIT_SHA,
      FW_GIT_SHA_SHORT,
      __DATE__,
      __TIME__,
      FW_SOURCE_PATH,
      FW_ARDUINO_VERSION,
      FW_TEENSYDUINO_VERSION,
      WHEEL_RADIUS,
      USE_PID_D_TERM ? "PID" : "PI",
      WHEEL_KP,
      WHEEL_KI,
      WHEEL_KD,
      MIN_EFFECTIVE_WHEEL_CMD,
      static_cast<unsigned long>(START_BOOST_DURATION_MS),
      LOW_SPEED_HOLD_PWM[FL],
      LOW_SPEED_HOLD_PWM[RL],
      LOW_SPEED_HOLD_PWM[FR],
      LOW_SPEED_HOLD_PWM[RR],
      ENCODER_POLARITY[FL],
      ENCODER_POLARITY[RL],
      ENCODER_POLARITY[FR],
      ENCODER_POLARITY[RR],
      FW_LIBRARY_LIST);

  rosidl_runtime_c__String__assign(&firmwareInfoMessage.data, infoBuffer);
  RCSOFTCHECK(rcl_publish(&firmwareInfoPublisher, &firmwareInfoMessage, nullptr));
}

void periodicTimeSync()
{
  const uint32_t nowMs = millis();

  if (nowMs - lastTimeSyncMs < TIME_SYNC_PERIOD_MS) {
    return;
  }
  lastTimeSyncMs = nowMs;

  // Maximum blocking time is 100 ms, below the 500 ms motor watchdog.
  (void)rmw_uros_sync_session(100);
}

// ============================================================================
// Fatal error loop
// ============================================================================

void errorLoop()
{
  stopAllMotors();
  pinMode(LED_BUILTIN, OUTPUT);

  while (true) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}

// ============================================================================
// Setup
// ============================================================================

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ESTOP_PIN, INPUT_PULLUP);

  analogWriteResolution(8);

  for (int motor = 0; motor < 4; ++motor) {
    pinMode(PWM_PIN[motor], OUTPUT);
    pinMode(DIR_PIN[motor], OUTPUT);
    analogWrite(PWM_PIN[motor], 0);
    digitalWrite(DIR_PIN[motor], LOW);
  }

  stopAllMotors();

  // Boot is a DISARMED entry like any other. Arming after a reset takes the
  // same zero hold, service call, and quiet barrier as arming after an E-stop.
  rearmGateInit(&driveGate);

  // Keep the robot completely still during this startup calibration.
  initializeImu();
  calibrateGyroBias();

  // USB serial transport to the laptop/Jetson micro-ROS agent.
  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, nullptr, &allocator));
  RCCHECK(rclc_node_init_default(
      &node,
      "teensy_integrated_base",
      "",
      &support));

  // Subscribers
  RCCHECK(rclc_subscription_init_default(
      &cmdVelSubscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"));

  RCCHECK(rclc_subscription_init_default(
      &resetOdomSubscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "reset_odom"));

  RCCHECK(rclc_subscription_init_default(
      &resetYawSubscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "imu/reset_yaw"));

  // BEST_EFFORT / VOLATILE sensor publishers
  RCCHECK(rclc_publisher_init_default(
      &odomPublisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "odom"));

  RCCHECK(rclc_publisher_init_best_effort(
      &imuPublisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "imu/data"));

  RCCHECK(rclc_publisher_init_best_effort(
      &imuYawPublisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
      "imu/yaw_deg"));

  RCCHECK(rclc_publisher_init_best_effort(
      &gyroBiasPublisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
      "imu/gyro_bias"));

  // Reliable status publishers
  RCCHECK(rclc_publisher_init_default(
      &estopStatePublisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "estop/state"));

  RCCHECK(rclc_publisher_init_default(
      &firmwareInfoPublisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "firmware/info"));

  RCCHECK(rclc_publisher_init_default(
      &driveEnabledPublisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "drive/enabled"));

  // x = service call count, y = last reject reason, z = latch state.
  RCCHECK(rclc_publisher_init_default(
      &driveDiagPublisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
      "drive/diag"));

  RCCHECK(rclc_service_init_default(
      &driveEnableService,
      &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
      "drive/enable"));

  // Message initialization
  geometry_msgs__msg__Twist__init(&cmdVelMessage);
  std_msgs__msg__Bool__init(&resetOdomMessage);
  std_msgs__msg__Bool__init(&resetYawMessage);

  nav_msgs__msg__Odometry__init(&odomMessage);
  sensor_msgs__msg__Imu__init(&imuMessage);
  std_msgs__msg__Float64__init(&imuYawMessage);
  geometry_msgs__msg__Vector3__init(&gyroBiasMessage);
  std_msgs__msg__Bool__init(&estopStateMessage);
  std_msgs__msg__String__init(&firmwareInfoMessage);
  std_msgs__msg__Bool__init(&driveEnabledMessage);
  geometry_msgs__msg__Vector3__init(&driveDiagMessage);
  std_srvs__srv__SetBool_Request__init(&driveEnableRequest);
  std_srvs__srv__SetBool_Response__init(&driveEnableResponse);

  rosidl_runtime_c__String__assign(&odomMessage.header.frame_id, "odom");
  rosidl_runtime_c__String__assign(&odomMessage.child_frame_id, "base_footprint");
  rosidl_runtime_c__String__assign(&imuMessage.header.frame_id, "imu_link");

  // Odom covariance: provisional values; replace with measured values later.
  for (int i = 0; i < 36; ++i) {
    odomMessage.pose.covariance[i] = 0.0;
    odomMessage.twist.covariance[i] = 0.0;
  }
  odomMessage.pose.covariance[0] = 0.01;
  odomMessage.pose.covariance[7] = 0.01;
  odomMessage.pose.covariance[35] = 0.05;
  odomMessage.twist.covariance[0] = 0.02;
  odomMessage.twist.covariance[7] = 0.02;
  odomMessage.twist.covariance[35] = 0.10;

  // IMU orientation is unavailable in /imu/data; angular velocity is valid.
  for (int i = 0; i < 9; ++i) {
    imuMessage.orientation_covariance[i] = 0.0;
    imuMessage.angular_velocity_covariance[i] = 0.0;
    imuMessage.linear_acceleration_covariance[i] = 0.0;
  }
  imuMessage.orientation_covariance[0] = -1.0;
  imuMessage.angular_velocity_covariance[0] = 0.0025;
  imuMessage.angular_velocity_covariance[4] = 0.0025;
  imuMessage.angular_velocity_covariance[8] = 0.0025;
  imuMessage.linear_acceleration_covariance[0] = 0.04;
  imuMessage.linear_acceleration_covariance[4] = 0.04;
  imuMessage.linear_acceleration_covariance[8] = 0.04;

  // 3 subscriptions + 1 service. Undersizing this does not fail quietly: the
  // handle array fills, rclc_executor_add_service() returns RCL_RET_ERROR and
  // RCCHECK drops into errorLoop() — a halted board, not a silent no-op.
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));

  RCCHECK(rclc_executor_add_subscription(
      &executor,
      &cmdVelSubscriber,
      &cmdVelMessage,
      &cmdVelCallback,
      ON_NEW_DATA));

  RCCHECK(rclc_executor_add_subscription(
      &executor,
      &resetOdomSubscriber,
      &resetOdomMessage,
      &resetOdomCallback,
      ON_NEW_DATA));

  RCCHECK(rclc_executor_add_subscription(
      &executor,
      &resetYawSubscriber,
      &resetYawMessage,
      &resetYawCallback,
      ON_NEW_DATA));

  RCCHECK(rclc_executor_add_service(
      &executor,
      &driveEnableService,
      &driveEnableRequest,
      &driveEnableResponse,
      &driveEnableCallback));

  (void)rmw_uros_sync_session(1000);
  lastTimeSyncMs = millis();

  resetOdometry();
  resetImuYaw();
}

// ============================================================================
// Main loop
// ============================================================================

void loop()
{
  // Safety checks are intentionally performed before and after ROS work.
  checkSafety();

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2)));

  // 🔴 Review §55.1 — the quiet barrier clock starts HERE, not in the service
  // callback. rclc runs the callback and only then calls rcl_send_response, both
  // inside the spin above; by the time this line runs the response has been
  // sent. Calling it every loop is intentional — it is a no-op unless the gate
  // is in ARMING, and a gate left in ARMING never arms (fail-closed).
  rearmGateArmBarrierStart(&driveGate, millis());

  checkSafety();
  updateMotorOutputs();
  updateOdometry();
  updateImu();
  publishDiagnostics();
  publishFirmwareInfo();
  periodicTimeSync();

  delay(1);
}