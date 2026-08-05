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
#include <rosidl_runtime_c/string_functions.h>

// ============================================================================
// Firmware identity
// Replace the zero SHA values with the real Git values before handover.
//   git rev-parse HEAD
//   git rev-parse --short=8 HEAD
// ============================================================================

#define STRINGIFY_INNER(x) #x
#define STRINGIFY(x) STRINGIFY_INNER(x)

static const char FW_VERSION[] = "handover-integrated-pi-continuous-low-speed-1.3.0";
static const char FW_GIT_SHA[] = "0000000000000000000000000000000000000000";
static const char FW_GIT_SHA_SHORT[] = "00000000";
static const char FW_SOURCE_PATH[] = "/home/park/robot_firmware/teensy_integrated_pi_continuous_low_speed_v1_3.ino";
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
// Active-low auxiliary contact: pin 21 -> E-stop contact -> GND.
// This software input supplements, but does not replace, the hardwired motor
// power cutoff.
// ============================================================================

static const uint8_t ESTOP_PIN = 21;
static const bool ESTOP_ACTIVE_LOW = true;

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
static const double FEEDFORWARD_PWM_PER_MPS_ABOVE_MIN = 1300.0;
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

geometry_msgs__msg__Twist cmdVelMessage;
std_msgs__msg__Bool resetOdomMessage;
std_msgs__msg__Bool resetYawMessage;

nav_msgs__msg__Odometry odomMessage;
sensor_msgs__msg__Imu imuMessage;
std_msgs__msg__Float64 imuYawMessage;
geometry_msgs__msg__Vector3 gyroBiasMessage;
std_msgs__msg__Bool estopStateMessage;
std_msgs__msg__String firmwareInfoMessage;

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
  if (isEstopActive()) {
    stopAllMotors();
    cmdVelReceived = false;
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

void checkSafety()
{
  if (isEstopActive()) {
    stopAllMotors();
    cmdVelReceived = false;
    return;
  }

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
    stopAllMotors();
    cmdVelReceived = false;
    return;
  }

  const double linearX = message->linear.x;
  const double angularZ = message->angular.z;

  if (!isfinite(linearX) || !isfinite(angularZ)) {
    stopAllMotors();
    cmdVelReceived = false;
    return;
  }

  lastCmdVelMs = millis();
  cmdVelReceived = true;
  applySkidSteerCommand(linearX, angularZ);
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

  RCSOFTCHECK(rcl_publish(&gyroBiasPublisher, &gyroBiasMessage, nullptr));
  RCSOFTCHECK(rcl_publish(&estopStatePublisher, &estopStateMessage, nullptr));
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

  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));

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

  checkSafety();
  updateMotorOutputs();
  updateOdometry();
  updateImu();
  publishDiagnostics();
  publishFirmwareInfo();
  periodicTimeSync();

  delay(1);
}