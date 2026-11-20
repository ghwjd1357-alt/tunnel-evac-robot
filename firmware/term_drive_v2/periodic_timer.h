#ifndef PERIODIC_TIMER_H
#define PERIODIC_TIMER_H

// ============================================================================
// periodic_timer.h — "주기가 지났을 때만 일한다" 를 객체 하나로
//
// 원본 .ino 에는 같은 모양의 코드가 여섯 번 있었다:
//     if (now - lastXxx < 주기) return;
//     lastXxx = now;
// 그리고 마지막 실행 시각을 담는 전역 변수도 여섯 개였다(lastPwmUpdateMs, lastDiagnosticMs,
// lastFirmwareInfoMs, lastTimeSyncMs, previousOdomUs, previousImuUs).
// 이 클래스는 "주기" 와 "마지막 실행 시각" 을 한 객체에 묶는다 (강의 09 클래스와 객체).
// 주기는 객체를 만들 때 정한다 (강의 10 매개변수 생성자):
//     PeriodicTimer diagnosticTimer(1000);   // 1초마다
//
// 단위는 정하지 않는다 — 밀리초(millis())로 쓰면 밀리초, 마이크로초(micros())로 쓰면
// 마이크로초다. 한 객체 안에서만 같은 단위를 쓰면 된다.
//
// 🔴 시간 비교는 반드시 **뺄셈** 으로 한다 (now - last < period).
//    uint32_t 시계는 micros() 기준 약 71.6분, millis() 기준 약 49.7일마다 0 으로 돌아가는데,
//    부호 없는 뺄셈은 그때도 흐른 시간을 정확히 준다. `now > last + period` 로 쓰면 깨진다.
// ============================================================================

#include <stdint.h>

class PeriodicTimer
{
public:
  // 매개변수 생성자: 주기를 정한다. 마지막 실행 시각은 0 에서 시작한다
  // (원본 전역 변수들의 초기값 0 과 같다).
  explicit PeriodicTimer(uint32_t periodValue);

  // 주기가 지났으면 마지막 실행 시각을 now 로 바꾸고 true, 아직이면 false.
  bool isDue(uint32_t now);

  // 마지막 실행 후 흐른 시간. 오도메트리·IMU 처럼 실제 dt 가 필요한 곳에서 쓴다.
  uint32_t elapsed(uint32_t now) const;

  // 마지막 실행 시각을 now 로 다시 맞춘다 (리셋·부팅 동기 직후).
  void restart(uint32_t now);

  // 마지막 실행 시각 (부팅 시각 동기 결과를 기록할 때 쓴다).
  uint32_t getLastTime() const;

private:
  uint32_t period;    // 주기
  uint32_t lastTime;  // 마지막으로 실행한 시각
};

// ── 멤버함수 정의 (강의 09: 클래스 밖에서 :: 범위지정연산자로 정의) ──────────────

inline PeriodicTimer::PeriodicTimer(uint32_t periodValue)
    : period(periodValue), lastTime(0)
{
}

inline bool PeriodicTimer::isDue(uint32_t now)
{
  if (now - lastTime < period) {
    return false;
  }
  lastTime = now;
  return true;
}

inline uint32_t PeriodicTimer::elapsed(uint32_t now) const
{
  return now - lastTime;
}

inline void PeriodicTimer::restart(uint32_t now)
{
  lastTime = now;
}

inline uint32_t PeriodicTimer::getLastTime() const
{
  return lastTime;
}

#endif  // PERIODIC_TIMER_H
