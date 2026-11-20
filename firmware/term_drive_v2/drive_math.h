#ifndef DRIVE_MATH_H
#define DRIVE_MATH_H

// ============================================================================
// drive_math.h — 구동부 클래스들이 같이 쓰는 작은 계산 도우미
//
// 왜 따로 두나
//   wheel.h · pid_controller.h · drive_base.h 가 모두 "범위로 자르기", "큰 값 고르기",
//   "반올림" 을 쓴다. 한 곳에만 두어야 세 파일이 같은 규칙으로 계산한다.
//
// 왜 Arduino 의 max()·round() 를 그대로 안 쓰나
//   이 헤더들은 노트북 g++ 로도 컴파일해서 시험한다(tools/term_drive_equivalence.*).
//   그래서 Arduino.h 에 기대지 않는다. 대신 Teensy 코어(teensy4/wiring.h)의 정의를
//   **글자 그대로 따라** 보드에서 원본과 같은 값이 나오게 한다:
//     max(a, b)   →  a >= b ? a : b
//     round(x)    →  (long) __builtin_round(x)
//   ⚠ 이 헤더 안에서는 `max(` · `round(` 라는 이름을 쓰지 않는다. Arduino 는 이 둘을
//     매크로/템플릿으로 정의하므로 이름이 겹치면 보드 빌드에서 엉뚱하게 펼쳐진다.
// ============================================================================

#include <math.h>
#include <stdint.h>

// 값을 [minimum, maximum] 범위 안으로 자른다. 원본 .ino 의 clampDouble 과 같은 순서로 비교한다.
static inline double driveClamp(double value, double minimum, double maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

// 둘 중 큰 값. Teensy 의 max 템플릿과 같은 비교(>=)를 쓴다.
static inline double driveMax(double a, double b)
{
  return (a >= b) ? a : b;
}

// 가장 가까운 정수로 반올림(0.5 는 0 에서 먼 쪽). Teensy 의 round 매크로와 같은 정의.
static inline long driveRound(double x)
{
  return static_cast<long>(__builtin_round(x));
}

#endif  // DRIVE_MATH_H
