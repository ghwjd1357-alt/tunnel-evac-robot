// ============================================================================
// sim_main.cpp — 바퀴 하나 속도 제어 SIL 시뮬레이션
//
// 펌웨어의 Wheel 클래스(firmware/term_drive_v2/wheel.h)를 **고치지 않고** 그대로 쓰고,
// 모터만 motor_model.h 의 가짜 모델로 대신한다. 20 ms 마다 한 칸씩 시간을 넘기며
// 한 줄씩 CSV 로 출력한다. (빈칸 연습판 = 커밋 3c30c9c)
//
// 빌드 · 실행 (sim 폴더에서):
//   g++ -std=c++11 -Wall -Wextra -I ../firmware/term_drive_v2 sim_main.cpp -o build/sim
//   ./build/sim                                   # 펌웨어 값 그대로, 0.12 m/s 스텝 2초
//   ./build/sim target=0.30 seconds=60 block_from=0 block_to=30   # 예: 30초 막혔다 풀림
//
// 인자 (key=value, 모두 선택):
//   target      목표 속도 [m/s]                기본 0.12
//   seconds     실행 시간 [s]                  기본 2
//   kp          비례 게인                      기본 WHEEL_KP (60)
//   ilimit      적분항 상한 [PWM]              기본 INTEGRAL_PWM_LIMIT (40)
//   alpha       속도 필터 α                    기본 VELOCITY_FILTER_ALPHA (0.10)
//   tau         가짜 모터 시정수 [s]           기본 0.10 (🔴 가정값)
//   load        부하 [PWM] — 움직이는 데 더 드는 PWM (예시 값)   기본 0
//   block_from  바퀴를 붙잡기 시작 [s]          기본 -1 (안 붙잡음)
//   block_to    바퀴를 놓는 시각 [s]            기본 -1
// ============================================================================

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sim_config.h"
#include "motor_model.h"
#include "wheel.h"   // 펌웨어 그대로 — Wheel 안에 WheelSpeedController · PIDController 가 들어 있다

// "key=value" 인자 하나를 읽는다. key 가 맞으면 value 를 out 에 넣고 true.
static bool readArg(const char* arg, const char* key, double* out)
{
  const size_t keyLength = strlen(key);
  if (strncmp(arg, key, keyLength) == 0 && arg[keyLength] == '=') {
    *out = atof(arg + keyLength + 1);
    return true;
  }
  return false;
}

int main(int argc, char** argv)
{
  // ── 실험 조건 (기본값 = 펌웨어 값) ─────────────────────────────────────
  double targetSpeed = 0.12;
  double runSeconds = 2.0;
  double kp = WHEEL_KP;
  double integralLimit = INTEGRAL_PWM_LIMIT;
  double filterAlpha = VELOCITY_FILTER_ALPHA;
  double motorTau = 0.10;   // 🔴 가정값 — 실차에서 재서 바꾼다
  double loadPwm = 0.0;
  double blockFrom = -1.0;
  double blockTo = -1.0;

  for (int i = 1; i < argc; ++i) {
    const bool known =
        readArg(argv[i], "target", &targetSpeed) ||
        readArg(argv[i], "seconds", &runSeconds) ||
        readArg(argv[i], "kp", &kp) ||
        readArg(argv[i], "ilimit", &integralLimit) ||
        readArg(argv[i], "alpha", &filterAlpha) ||
        readArg(argv[i], "tau", &motorTau) ||
        readArg(argv[i], "load", &loadPwm) ||
        readArg(argv[i], "block_from", &blockFrom) ||
        readArg(argv[i], "block_to", &blockTo);
    if (!known) {
      fprintf(stderr, "모르는 인자: %s\n", argv[i]);
      return 1;
    }
  }

  // ── 가짜 모터 (출처 = motor_model.h 머리말) ─────────────────────────────
  MotorModel motor(0.003691, 30.99, motorTau, DISTANCE_PER_COUNT);
  motor.setLoadPwm(loadPwm);

  // ── 펌웨어 바퀴 객체 하나 (.ino:533 과 같은 생성자) ─────────────────────
  Wheel wheel(START_BOOST_PWM, LOW_SPEED_HOLD_PWM, MIN_RUNNING_PWM,
              makeControlConfig(kp, integralLimit), makeMotionConfig(filterAlpha));

  // 관측용 "거울" 제어기. Wheel 안의 제어기는 private 이라 적분값을 밖에서 읽을 수 없다.
  // 같은 설정·같은 입력으로 하나 더 돌려서 읽는다. 매 칸 출력이 Wheel 과 같은지 검사한다
  // (다르면 거울이 틀린 것 → 즉시 멈춘다). 펌웨어 헤더는 고치지 않는다.
  WheelSpeedController mirror(makeControlConfig(kp, integralLimit), makeMotionConfig(filterAlpha),
                              LOW_SPEED_HOLD_PWM, MIN_RUNNING_PWM);

  const double dt = CONTROL_PERIOD_MS / 1000.0;   // 0.02 s
  const uint32_t steps = static_cast<uint32_t>(runSeconds / dt + 0.5);

  // 시각 0 에 젯슨 명령이 들어왔다고 친다 (.ino 의 cmdVelCallback → setTarget).
  wheel.setTarget(targetSpeed, 0);

  printf("t_s,target,applied_pwm,true_velocity,encoder_count,measured,filtered,"
         "desired_pwm,ff_pwm,p_term,i_term,blocked\n");

  for (uint32_t k = 0; k <= steps; ++k) {
    const uint32_t nowMs = k * CONTROL_PERIOD_MS;
    const double nowS = nowMs / 1000.0;

    const bool blocked = (blockFrom >= 0.0) && (nowS >= blockFrom) && (nowS < blockTo);
    motor.setBlocked(blocked);

    // 20 ms 한 칸 — 펌웨어 loop() 순서: updateMotorOutputs() → updateOdometry()
    // ① PWM 출력 (부스트·램프) — 지난 칸에 계산한 원하는 PWM 을 향해
    const int appliedPwm = wheel.updateOutput(nowMs);
    // ② 그 PWM 으로 모터가 20 ms 동안 돈다 (실물에서는 물리가 하는 일)
    motor.step(appliedPwm, dt);
    // ③ 엔코더 읽기 → 속도 재기 + 필터
    wheel.measure(motor.encoderCount(), dt);
    // ④ FF + PI 로 다음 칸의 원하는 PWM 계산
    wheel.updateController(dt);

    // 관측용 분해 — 거울 제어기에 같은 입력을 넣는다.
    const int mirrorPwm = mirror.compute(wheel.getTarget(), wheel.getFilteredVelocity(), dt);
    if (mirrorPwm != wheel.getDesiredPwm()) {
      fprintf(stderr, "거울 불일치 t=%.2f: wheel=%d mirror=%d\n",
              nowS, wheel.getDesiredPwm(), mirrorPwm);
      return 2;
    }
    const double ff = mirror.feedforwardPwm(wheel.getTarget());
    const double pTerm = kp * (wheel.getTarget() - wheel.getFilteredVelocity());   // 목표 양수 기준
    const double iTerm = WHEEL_KI * mirror.getIntegralError();

    printf("%.2f,%.3f,%d,%.5f,%ld,%.5f,%.5f,%d,%.0f,%.3f,%.3f,%d\n",
           nowS, wheel.getTarget(), appliedPwm, motor.getVelocity(), motor.encoderCount(),
           wheel.getMeasuredVelocity(), wheel.getFilteredVelocity(), wheel.getDesiredPwm(),
           ff, pTerm, iTerm, blocked ? 1 : 0);
  }

  return 0;
}
