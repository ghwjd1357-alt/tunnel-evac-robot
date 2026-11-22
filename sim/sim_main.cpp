// ============================================================================
// sim_main.cpp — 바퀴 하나 속도 제어 SIL 시뮬레이션
//
// 펌웨어의 Wheel 클래스(firmware/term_drive_v2/wheel.h)를 **고치지 않고** 그대로 쓰고,
// 모터만 motor_model.h 의 가짜 모델로 대신한다. 20 ms 마다 한 칸씩 시간을 넘기며
// 한 줄씩 CSV 로 출력한다.
//
// 빌드 · 실행 (sim 폴더에서):
//   g++ -std=c++11 -Wall -Wextra -I ../firmware/term_drive_v2 sim_main.cpp -o build/sim
//   ./build/sim > build/step.csv
// ============================================================================

#include <stdint.h>
#include <stdio.h>

#include "sim_config.h"
#include "motor_model.h"
#include "wheel.h"   // 펌웨어 그대로 — Wheel 안에 WheelSpeedController · PIDController 가 들어 있다

int main()
{
  // ── 실험 조건 ───────────────────────────────────────────────────────────
  const double targetSpeed = 0.12;   // 목표 속도 [m/s]
  const double runSeconds = 2.0;     // 몇 초 돌릴까
  const double kp = WHEEL_KP;        // 펌웨어 값 60. 4단계에서 30 으로 바꿔 본다

  // ── 가짜 모터 (출처 = motor_model.h 머리말) ─────────────────────────────
  const double motorTau = 0.10;      // 🔴 가정값 [s] — 실차에서 재서 바꾼다
  MotorModel motor(0.003691, 30.99, motorTau, DISTANCE_PER_COUNT);

  // ── 펌웨어 바퀴 객체 하나 (.ino:533 과 같은 생성자) ─────────────────────
  Wheel wheel(START_BOOST_PWM, LOW_SPEED_HOLD_PWM, MIN_RUNNING_PWM,
              makeControlConfig(kp), makeMotionConfig());

  const double dt = CONTROL_PERIOD_MS / 1000.0;   // 0.02 s
  const uint32_t steps = static_cast<uint32_t>(runSeconds / dt);

  // 시각 0 에 젯슨 명령이 들어왔다고 친다 (.ino 의 cmdVelCallback → setTarget).
  wheel.setTarget(targetSpeed, 0);

  // CSV 머리줄
  printf("t_ms,target,applied_pwm,true_velocity,encoder_count,measured,filtered,desired_pwm\n");

  for (uint32_t k = 0; k <= steps; ++k) {
    const uint32_t nowMs = k * CONTROL_PERIOD_MS;

    // ✏️ 빈칸 C — 20 ms 한 칸에 일어나는 일 4가지를 **순서대로** 쓴다.
    //    펌웨어 loop() 순서 = updateMotorOutputs() 먼저, 그다음 updateOdometry()
    //    (updateOdometry 안에서 measure 와 제어 계산을 차례로 부른다 — firmware README "한 번의 제어 주기")
    //
    //    쓸 수 있는 함수:
    //      int  wheel.updateOutput(nowMs)        → 이번에 모터에 줄 PWM
    //      void motor.step(pwm, dt)              → 가짜 모터를 20 ms 진행
    //      long motor.encoderCount()             → 엔코더 누적 칸 수
    //      double wheel.measure(count, dt)       → 칸 수로 속도 재기 + 필터
    //      void wheel.updateController(dt)       → FF + PI 로 다음 PWM 계산
    //
    //    질문: 이 다섯 개를 어떤 순서로 불러야 실제 로봇과 같아질까?
    int appliedPwm = 0;   // ← 빈칸 C 에서 updateOutput 의 결과를 여기에 담는다

    // 한 줄 출력
    printf("%u,%.3f,%d,%.4f,%ld,%.4f,%.4f,%d\n",
           nowMs,
           wheel.getTarget(),
           appliedPwm,
           motor.getVelocity(),
           motor.encoderCount(),
           wheel.getMeasuredVelocity(),
           wheel.getFilteredVelocity(),
           wheel.getDesiredPwm());
  }

  return 0;
}
