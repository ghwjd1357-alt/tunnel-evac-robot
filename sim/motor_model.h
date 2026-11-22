#ifndef MOTOR_MODEL_H
#define MOTOR_MODEL_H

// ============================================================================
// motor_model.h — 가짜 모터·바퀴·엔코더 (플랜트 모델)
//
// 실물이 없으니 "PWM 을 주면 바퀴가 이렇게 돈다" 를 수식으로 흉내 낸다.
// 🔴 이 파일은 **가정**이다. 펌웨어 코드가 아니다. 숫자마다 출처를 적는다.
//
// 모델 두 줄:
//   ① 정상상태 속도  v_ss = K × (|PWM| − PWM0)   (|PWM| ≤ PWM0 이면 0 = 불감대)
//   ② 1차 지연      v 는 v_ss 쪽으로 시정수 τ 만큼 천천히 다가간다
//
//   K    = 0.003691 m/s per PWM ┐ 지면 실측 3점 직선 맞춤 (docs/MASTER_PLAN.md, main 브랜치
//   PWM0 = 30.99               ┘ "plant: v = 0.003691 × (PWM − 30.99)")
//        ⚠ 맞춘 구간 = PWM 54.6~66.5 (속도 0.09~0.13 m/s). 145 PWM 에서는 0.42 를 예측하지만
//          실측은 0.33 → **구간 밖(빠른 속도)에서는 이 직선이 과대**다.
//   τ    = 가정값. 기존 bag 은 /drive/diag 가 1 Hz 라 측정 불가 → 실차에서 재서 바꾼다.
// ============================================================================

#include <math.h>

class MotorModel
{
public:
  MotorModel(double gainMpsPerPwm, double deadbandPwm, double timeConstantS,
             double distancePerCount)
      : gain(gainMpsPerPwm),
        deadband(deadbandPwm),
        tau(timeConstantS),
        metersPerCount(distancePerCount),
        loadPwm(0.0),
        blocked(false),
        velocity(0.0),
        position(0.0)
  {
  }

  // 실험용 조건 (실물로는 위험하거나 재현이 어려운 상황)
  //  · 부하: 바퀴를 움직이는 데 PWM 이 loadPwm 만큼 더 든다 (예: 걸림·경사). 값은 실험마다 (예시).
  //  · 막힘: 바퀴가 물리적으로 붙잡혀 속도 0.
  void setLoadPwm(double extraPwm) { loadPwm = extraPwm; }
  void setBlocked(bool isBlocked) { blocked = isBlocked; }

  // PWM 을 dt 초 동안 줬을 때 바퀴 속도·위치를 한 칸 진행한다.
  void step(int pwm, double dt)
  {
    // 모델 ① : 정상상태 속도. |PWM| 이 불감대(+부하) 이하면 0, 그 위로는 직선.
    const double magnitude = fabs(static_cast<double>(pwm));
    const double threshold = deadband + loadPwm;
    double steadyVelocity = 0.0;
    if (magnitude > threshold) {
      steadyVelocity = gain * (magnitude - threshold);
    }
    if (pwm < 0) {
      steadyVelocity = -steadyVelocity;   // 뒤로
    }

    // 모델 ② : 1차 지연. 속도 필터와 같은 모양 — α 자리에 dt / τ.
    velocity += (dt / tau) * (steadyVelocity - velocity);

    if (blocked) {
      velocity = 0.0;   // 붙잡힌 바퀴
    }

    // 바퀴가 굴러간 거리를 쌓는다.
    position += velocity * dt;
  }

  // 엔코더 누적 칸 수. 실제 엔코더처럼 **정수**만 나온다.
  long encoderCount() const
  {
    return static_cast<long>(floor(position / metersPerCount));
  }

  double getVelocity() const { return velocity; }

private:
  double gain;
  double deadband;
  double tau;
  double metersPerCount;
  double loadPwm;
  bool blocked;

  double velocity;   // 실제(참) 바퀴 속도 [m/s] — 실물에서는 아무도 직접 못 본다
  double position;   // 굴러간 거리 [m]
};

#endif  // MOTOR_MODEL_H
