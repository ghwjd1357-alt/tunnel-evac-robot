#ifndef DRIVE_BASE_H
#define DRIVE_BASE_H

// ============================================================================
// drive_base.h — 로봇 몸체: 바퀴 4개를 가진 객체
//
// 젯슨은 로봇 중심의 "직진 속도 v, 회전 속도 ω" 두 숫자만 보낸다(/cmd_vel).
// 이 클래스가 그것을 차동구동(스키드 스티어) 기구학으로 좌우 바퀴 목표로 바꾼다:
//     왼쪽  = v − ω × b / 2
//     오른쪽 = v + ω × b / 2        (b = 명령용 유효 윤거 cmdWheelBase)
// 한쪽이 바퀴 상한을 넘으면 두 바퀴를 **같은 비율** 로 줄여 회전 반경을 유지한다.
// 왼쪽 값은 왼앞·왼뒤, 오른쪽 값은 오른앞·오른뒤 바퀴에 똑같이 준다.
//
// 구조 (객체 안의 객체, 객체 배열)
//   DriveBase ── wheels[4] : Wheel ── controller : WheelSpeedController ← PIDController
//                 [0] 왼앞  [1] 왼뒤  [2] 오른앞  [3] 오른뒤
//   강의 07 의 배열을 객체로 확장한 "객체 배열" 이다. 실제 로봇의 "몸체 하나에 바퀴 넷" 이
//   코드 구조와 같아진다.
//
// 원본 .ino 대응: applySkidSteerCommand → setCommand,
//                 updateWheelControllers 의 반복문 → updateControllers,
//                 stopAllMotors 의 상태 부분 → stopAll.
// 하드웨어(analogWrite 등)는 다루지 않는다 → 노트북 g++ 로 시험할 수 있다.
// ============================================================================

#include <math.h>
#include <stdint.h>

#include "drive_math.h"
#include "wheel_config.h"
#include "wheel.h"

class DriveBase
{
public:
  // 바퀴 번호. 순서는 .ino 의 MotorIndex(FL=0, RL=1, FR=2, RR=3)와 같다
  // (.ino 의 static_assert 가 둘이 어긋나면 컴파일을 막는다).
  enum WheelIndex { FRONT_LEFT = 0, REAR_LEFT = 1, FRONT_RIGHT = 2, REAR_RIGHT = 3, WHEEL_COUNT = 4 };

  DriveBase(const Wheel& frontLeft, const Wheel& rearLeft,
            const Wheel& frontRight, const Wheel& rearRight,
            const DriveConfig& driveConfig,
            const WheelMotionConfig& motionConfig);

  // 젯슨 명령 (v, ω) → 바퀴 4개의 목표 속도. 원본 applySkidSteerCommand.
  void setCommand(double linearX, double angularZ, uint32_t nowMs);

  // 바퀴 4개의 제어 계산. 원본 updateWheelControllers 의 반복문.
  void updateControllers(double dt);

  // 바퀴 4개를 정지 상태로 (하드웨어 PWM 0 쓰기는 .ino 가 한다).
  void stopAll();

  // 바퀴 4개의 제어기 기억만 0 으로.
  void resetControllers();

  // 바퀴 4개의 측정값을 0 으로 (오도메트리 리셋).
  void resetMeasurements();

  // 번호로 바퀴 꺼내기. 0~3 = FL, RL, FR, RR. (배열이므로 범위 밖 번호는 넘기지 않는다.)
  Wheel& wheel(int index);
  const Wheel& wheel(int index) const;

private:
  double maxLinearCmd;
  double maxAngularCmd;
  double maxWheelCmd;
  double cmdWheelBase;

  Wheel wheels[WHEEL_COUNT];   // 객체 배열
};

// ── 멤버함수 정의 ─────────────────────────────────────────────────────────

inline DriveBase::DriveBase(const Wheel& frontLeft, const Wheel& rearLeft,
                            const Wheel& frontRight, const Wheel& rearRight,
                            const DriveConfig& driveConfig,
                            const WheelMotionConfig& motionConfig)
    : maxLinearCmd(driveConfig.maxLinearCmd),
      maxAngularCmd(driveConfig.maxAngularCmd),
      maxWheelCmd(motionConfig.maxWheelCmd),
      cmdWheelBase(driveConfig.cmdWheelBase),
      wheels{frontLeft, rearLeft, frontRight, rearRight}
{
}

// 원본 applySkidSteerCommand 와 같은 순서.
inline void DriveBase::setCommand(double linearX, double angularZ, uint32_t nowMs)
{
  linearX = driveClamp(linearX, -maxLinearCmd, maxLinearCmd);
  angularZ = driveClamp(angularZ, -maxAngularCmd, maxAngularCmd);

  double leftWheelCommand = linearX - angularZ * cmdWheelBase * 0.5;
  double rightWheelCommand = linearX + angularZ * cmdWheelBase * 0.5;

  const double maximumMagnitude =
      driveMax(fabs(leftWheelCommand), fabs(rightWheelCommand));

  if (maximumMagnitude > maxWheelCmd) {
    const double scale = maxWheelCmd / maximumMagnitude;
    leftWheelCommand *= scale;
    rightWheelCommand *= scale;
  }

  wheels[FRONT_LEFT].setTarget(leftWheelCommand, nowMs);
  wheels[REAR_LEFT].setTarget(leftWheelCommand, nowMs);
  wheels[FRONT_RIGHT].setTarget(rightWheelCommand, nowMs);
  wheels[REAR_RIGHT].setTarget(rightWheelCommand, nowMs);
}

inline void DriveBase::updateControllers(double dt)
{
  for (int index = 0; index < WHEEL_COUNT; ++index) {
    wheels[index].updateController(dt);
  }
}

inline void DriveBase::stopAll()
{
  for (int index = 0; index < WHEEL_COUNT; ++index) {
    wheels[index].stop();
  }
}

inline void DriveBase::resetControllers()
{
  for (int index = 0; index < WHEEL_COUNT; ++index) {
    wheels[index].resetController();
  }
}

inline void DriveBase::resetMeasurements()
{
  for (int index = 0; index < WHEEL_COUNT; ++index) {
    wheels[index].resetMeasurement();
  }
}

inline Wheel& DriveBase::wheel(int index)
{
  return wheels[index];
}

inline const Wheel& DriveBase::wheel(int index) const
{
  return wheels[index];
}

#endif  // DRIVE_BASE_H
