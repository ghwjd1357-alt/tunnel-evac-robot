#ifndef ESTOP_DEBOUNCE_H
#define ESTOP_DEBOUNCE_H

// ============================================================================
// E-stop 입력 디바운스 + 글리치 관측 (2026-08-13 신설)
//
// 🔴 왜 필요한가 (08-12 실측 · ELECTRICAL_BASELINE §4-f~§4-f-3)
//   주행 중 아무도 버튼을 안 눌렀는데 PIN21 이 순간 HIGH 로 뜬다. 펌웨어는 그것을
//   "E-stop 눌림"으로 읽고 무장을 푼다. 08-12 하루에 12회 이상 났고, `0.05`(40 PWM)
//   에서는 평균 99초, `0.12`(63.5 PWM)에서는 평균 6초 — **전류에 비례**한다.
//   `/estop/state` 가 주행 중 True 를 찍은 표본을 직접 관측해 이 경로가 확정됐다.
//   유력 원인은 접지 바운스(모터 귀환 1~3A 가 신호 GND 도체를 공유해 I×R 이 생김)다.
//
// 🔴 무엇을 가르는가 — **시간**이다.
//   사람이 버튼을 누르면 최소 수백 ms 유지된다. 접촉/전기 글리치는 수 ms~수십 ms 다.
//   그래서 "연속 ESTOP_DEBOUNCE_MS 이상 같은 값일 때만 판정을 옮긴다"로 가른다.
//
// 🔴 안전이 나빠지지 않는 근거 (검토가 반드시 볼 자리)
//   ① 진짜 차단은 **하드웨어**가 한다. 버튼을 누르면 릴레이1·2 가 모터 전원 자체를
//      끊는다 — `ELECTRICAL_BASELINE §5-G8` 부하 차단 10회 실측 완료(실패 0).
//      이 핀은 그 뒤에 오는 **소프트웨어 래치**이지 차단 수단이 아니다.
//   ② 30ms 늦게 알아도 모터는 이미 전기가 없다. `0.12 m/s` 에서 30ms = **3.6mm**.
//   ③ 해제 방향도 같이 늦어지는데, 그건 **정지 상태를 30ms 더 유지**하는 것이라
//      안전한 방향이다.
//   ④ 필터가 없으면 로봇이 6초마다 선다. 반복 재무장은 그 자체로 운영 위험이고
//      우회 유혹을 만든다.
//   ⚠ 그래도 이것은 **증상 완화**다. 원인(접지 바운스 추정)은 배선 쪽에서 따로 닫는다.
//      🔴 원인 판정을 이 필터보다 **먼저** 끝내야 한다 — 필터가 증상을 덮으면
//      무엇이 고쳤는지 영원히 못 가린다.
//
// 🔴 관측 필드를 같이 둔다 (08-12 에 6시간을 태운 이유)
//   `rawEdges`·`maxHighMs` 는 **필터가 먹어치운 글리치**를 밖으로 내보낸다.
//   이게 없으면 "필터가 잘 듣는지"도, "30ms 가 충분한지"도 알 수 없다.
//   → `/firmware/info` 로 발행한다.
//
// 이 파일은 Arduino 헤더에 의존하지 않는다 — `tools/rearm_gate_host_test.cpp` 가
// PC 에서 그대로 include 해서 결정론적으로 시험한다.
// ============================================================================

#include <stdbool.h>
#include <stdint.h>

// 연속 이 시간 이상 같은 원시값이어야 판정을 옮긴다.
// 🔴 값 근거: 08-12 관측 글리치는 `/estop/state` 1Hz 표본 **한 칸 안**에 들어갔으므로
//    1초보다 짧다. 정확한 길이는 미측정이다 — 그래서 `maxHighMs` 를 같이 발행해
//    다음 주행에서 이 값이 충분한지 **실측으로** 확인한다.
//    ⚠ 글리치가 이보다 길면 필터는 듣지 않는다. 그때는 원인을 고쳐야지 이 값을
//    키우면 안 된다 — 키울수록 진짜 E-stop 인지도 늦게 안다.
static const uint32_t ESTOP_DEBOUNCE_MS = 30;

struct EstopDebounce {
  bool stable;           // 디바운스된 판정. 펌웨어는 이 값만 쓴다.
  bool lastRaw;          // 직전 원시 읽기
  uint32_t lastEdgeMs;   // 원시값이 마지막으로 바뀐 시각
  uint32_t highSinceMs;  // 내부 — 현재 HIGH 구간의 시작
  uint32_t rawEdges;     // [관측] 원시 전이 누계
  uint32_t maxHighMs;    // [관측] 원시 HIGH 최대 지속
};

// 🔴 부팅 시에는 읽은 값을 **그대로 믿는다.** 필터를 태우지 않는다 —
//    켰을 때 눌려 있으면 즉시 눌린 것으로 읽는 쪽이 fail-closed 다.
static inline void estopDebounceInit(struct EstopDebounce* d,
                                     bool rawActive,
                                     uint32_t nowMs)
{
  d->stable = rawActive;
  d->lastRaw = rawActive;
  d->lastEdgeMs = nowMs;
  d->highSinceMs = nowMs;
  d->rawEdges = 0;
  d->maxHighMs = 0;
}

// 매 루프 호출. 반환 = 디바운스된 판정.
static inline bool estopDebounceUpdate(struct EstopDebounce* d,
                                       bool rawActive,
                                       uint32_t nowMs)
{
  if (rawActive != d->lastRaw) {
    d->lastRaw = rawActive;
    d->lastEdgeMs = nowMs;
    d->rawEdges++;
    if (rawActive) {
      d->highSinceMs = nowMs;
    }
  }

  // HIGH 가 유지되는 동안 최대 지속을 갱신한다. 전이에서만 재면 아직 안 끝난
  // HIGH 를 놓치고, 그러면 "필터를 넘긴 글리치"가 기록에서 사라진다.
  if (rawActive) {
    const uint32_t held = nowMs - d->highSinceMs;
    if (held > d->maxHighMs) {
      d->maxHighMs = held;
    }
  }

  if (rawActive != d->stable &&
      (nowMs - d->lastEdgeMs) >= ESTOP_DEBOUNCE_MS) {
    d->stable = rawActive;
  }
  return d->stable;
}

#endif  // ESTOP_DEBOUNCE_H
