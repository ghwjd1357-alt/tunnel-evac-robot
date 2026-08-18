#ifndef LINK_STALL_PROBE_H
#define LINK_STALL_PROBE_H

// ============================================================================
// link_stall_probe.h — 예약 41-g 사건별 링크 정지 계측 (계약 3판 구현)
//
// 계약 정본 = docs/MASTER_PLAN.md §7 예약 41-g "08-18 확정 계측 계약" (3판).
// 이 헤더는 runtime_guard.h 와 같은 이유로 **Arduino 를 쓰지 않는다** — 보드 없이
// g++ 로 컴파일해 가짜 시계로 경계를 주입할 수 있어야 하기 때문이다.
// 시각·epoch 는 전부 인자로 받는다. 이 파일은 micros()/millis() 를 부르지 않는다.
//
// 이 헤더가 하는 일 / 하지 않는 일:
//   한다   — 시간을 exec/idle 두 조각으로 쪼개고, 사건을 ring 에 접어 담고,
//            생존 표본(pulse)의 계수를 관리한다.
//   안 한다 — 분류. 9행 분류표는 **호스트 쪽** tools/link_stall_classify.py 가 소유한다.
//            MCU 는 관측만 하고 원인을 적지 않는다 (§7 41-g "분류 전에 원인을 적지 않는다").
//
// 🔴 이 헤더가 증명하지 않는 것 (숨기지 않는다):
//   - 스케치가 이 함수들을 실제로 부르는가. 그 대조는 link_stall_host_test.sh 의
//     2단계 구조 검사(텍스트)가 하며 동작 검사보다 약한 증거다.
//   - 복귀하지 않는 영구 정지. 생존 표본도 loop() 에 매달려 있으므로 downstream
//     영구 정지와 갈리지 않는다 (§79.2 가 공개한 한계). "시행 종료"로만 적는다.
// ============================================================================

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "runtime_guard.h"

// ── 문턱값 (계약 3판 초안값) ────────────────────────────────────────────────
// exec/idle 문턱이 같은 100ms 인 것은 우연이 아니다 — 둘은 같은 하나의 공백을
// 서로 배타적인 두 조각으로 나눈 것이므로 같은 자로 재야 한다.
static const uint32_t LINK_EXEC_EVENT_US = 100000UL;   // loop 안
static const uint32_t LINK_IDLE_EVENT_US = 100000UL;   // 판 사이

// burst 를 끊는 무사건 간격. 08-17 최소 공백 257.51ms 의 1/5, 정상 주기 22ms 의
// 2배 이상 — 한 공백 **안**을 쪼개지 않으면서 서로 다른 공백은 확실히 끊는다.
static const uint32_t LINK_BURST_GAP_US = 50000UL;

// 생존 표본 주기. 사건 문턱이 100ms 이므로 300ms 공백이면 표본 3개가 빈다.
static const uint32_t LINK_PULSE_PERIOD_MS = 100UL;

// ring 물리 상한. 실제 사용 깊이는 linkProbeInit 의 capacity 로 정한다 —
// FLASH/RAM 여유가 줄면 8 로 낮추는 것이 계약이고(계측이 안전 여유를 먹지 않는다),
// 그때 harness 가 "8 에 9" 넘침도 같은 코드로 재현할 수 있어야 한다.
#define LINK_EVENT_RING_MAX 16

// sync 를 신뢰하는 최대 나이. TIME_SYNC 주기 30초의 2배 = 2회 연속 실패까지 허용.
static const uint32_t LINK_SYNC_AGE_LIMIT_MS = 60000UL;

// 한 판에 배출하는 사건 수 상한. 16칸을 한 판에 다 쏟으면 String 16개 약 2KB 를
// 한 loop 에서 밀어야 해서 921600bps 로도 20ms 를 먹는다 — 계측이 지연을 만든다.
// 4 면 약 5ms 이고, 1ms 주기 loop 에서 초당 4000건까지 뺄 수 있어 남는다.
static const uint8_t LINK_DRAIN_PER_LOOP = 4U;

// 전문 상한. 🔴 pulse 는 계약이 **≤128B** 로 못박았다 (MTU 512B 미만 → 단편화 없음).
// 이 상한은 주석이 아니라 회귀로 고정한다 — host harness 가 최악값으로 재본다.
#define LINK_PULSE_TEXT_MAX 128
#define LINK_EVENT_TEXT_MAX 160

// 한 번도 동기에 성공한 적이 없을 때의 sentinel. 호스트는 이 값을 보면
// 나이 비교를 하지 않고 곧바로 판정 불능으로 간다.
static const uint32_t LINK_SYNC_AGE_NEVER = 0xFFFFFFFFUL;

// publish 사건이 아님을 뜻하는 slot sentinel.
static const uint8_t LINK_SLOT_NONE = 0xFFU;

// ── 사건 코드 ───────────────────────────────────────────────────────────────
// 세 종이 최소이자 전부다. 이 셋이 상호 배타인 근거는 exec/idle 을 따로 재는 것
// (아래 linkProbeLoopBegin/End) 이지 코드 이름이 아니다.
enum LinkEventCode {
  LINK_EVENT_LOOP_INTERNAL = 0,   // exec_us 초과 — loop 안
  LINK_EVENT_BETWEEN_LOOPS = 1,   // idle_us 초과 — 판 사이
  LINK_EVENT_PUBLISH_FAIL  = 2    // rcl_publish 실패 — 발행 층
};

// burst_id 는 lane 마다 따로 센다. lane = publish slot 하나씩 + 비-publish 사건 둘.
// 🔴 전역 하나로 두면 안 되는 이유: 한 odom 공백 안에서도 imu 는 성공 발행하므로
//    전역 카운터는 그 성공에 반응해 **한 공백을 여러 조각으로** 쪼갠다.
//    반대로 slot 을 키에서 빼면 §79.3 이 지적한 "7 공백이 count=70 한 칸" 이 된다.
#define LINK_LANE_LOOP_INTERNAL (RUNTIME_PUBLISH_COUNT)
#define LINK_LANE_BETWEEN_LOOPS (RUNTIME_PUBLISH_COUNT + 1U)
#define LINK_LANE_COUNT         (RUNTIME_PUBLISH_COUNT + 2U)

// ── ring 한 칸 ──────────────────────────────────────────────────────────────
// 접어도 first/last epoch·count·publish_slot 은 남는다 (계약 3판 "보존 의무").
struct LinkEvent {
  uint64_t firstEpochMs;
  uint64_t lastEpochMs;
  uint32_t execUsMax;
  uint32_t idleUsMax;
  uint32_t burstId;
  uint32_t count;
  uint8_t  phase;
  uint8_t  publishSlot;   // LINK_SLOT_NONE = publish 사건이 아니다
  uint8_t  code;
};

// ── 생존 표본 ───────────────────────────────────────────────────────────────
// 🔴 이 구조체의 존재 이유가 §79.2 다. MCU 안에서 세기만 하면 관측이 아니다 —
//    조용한 구간에 host 로 갈 메시지가 있어야 "그 구간에 loop 가 돌았다"가 증거가 된다.
struct LinkPulse {
  uint32_t bootId;
  uint32_t sampleSeq;
  uint64_t epochMs;
  uint32_t syncAgeMs;
  uint32_t evtSeq;
  uint32_t evtDroppedTotal;
  uint32_t evtDroppedDelta;
  uint32_t pulseFail;
  bool     syncOk;
};

struct LinkStallProbe {
  struct LinkEvent ring[LINK_EVENT_RING_MAX];
  uint32_t burstId[LINK_LANE_COUNT];

  // 누적 최대 — 역회귀용. 사건이 이것을 대체하는 게 아니라 **옆에** 선다
  // (§7 41-g "누적 최대는 역회귀용으로 남기고, 그 옆에 사건을 세운다").
  uint32_t execMaxUs;
  uint32_t idleMaxUs;

  uint64_t lastLoopEndUs;
  uint64_t loopStartUs;
  uint64_t lastEventUs;
  uint32_t lastPulseMs;

  uint32_t evtSeq;
  uint32_t evtDroppedTotal;
  uint32_t evtDroppedDelta;
  uint32_t pendingDroppedDelta;   // 이번 pulse 에 실어 보낸 delta 스냅샷
  uint32_t sampleSeq;
  uint32_t pulseFail;
  uint32_t bootId;

  uint32_t lastSyncMs;
  uint32_t nowMsAtSyncQuery;

  uint8_t capacity;
  uint8_t head;    // 다음에 꺼낼 칸
  uint8_t used;    // 담긴 칸 수

  bool haveLastLoopEnd;
  bool haveLastEvent;
  bool havePulseSent;
  bool inLoop;
  bool syncEverOk;
};

// ── 32비트 micros() 뒤집힘 확장 ─────────────────────────────────────────────
// 🔴 Teensy 의 micros() 는 uint32 라 약 **71.6분마다 0 으로 되돌아간다**.
//    그 값을 그대로 uint64 자리에 넣으면 `nowUs - lastLoopEndUs` 가 약 4295초로
//    보인다 — 아무 일도 없었는데 BETWEEN_LOOPS 사건이 나고, 그 시각의 진짜 원인은
//    영영 못 찾는다. 계측이 스스로 가짜 사건을 만드는 것이 최악이다.
//
// ⚠ 전제: **뒤집힘 주기(71.6분)마다 최소 한 번은 불려야** 한다. loop 은 1~47ms
//   주기이므로 성립한다. 이 전제가 깨지는 자리(예: 긴 blocking 구간)를 새로 만들면
//   이 확장기는 뒤집힘을 놓친다 — 그때는 여기가 아니라 그 blocking 을 고친다.
struct LinkClock {
  uint64_t high;
  uint32_t lastRaw;
  bool started;
};

inline void linkClockInit(struct LinkClock* c)
{
  c->high = 0U;
  c->lastRaw = 0U;
  c->started = false;
}

inline uint64_t linkClockExtend(struct LinkClock* c, uint32_t rawUs)
{
  if (!c->started) {
    c->started = true;
    c->lastRaw = rawUs;
    return (uint64_t)rawUs;
  }
  if (rawUs < c->lastRaw) {
    c->high += 0x100000000ULL;   // 한 바퀴 돌았다
  }
  c->lastRaw = rawUs;
  return c->high + (uint64_t)rawUs;
}

// ── 초기화 ──────────────────────────────────────────────────────────────────
// capacity 는 1..LINK_EVENT_RING_MAX. 범위 밖이면 물리 상한으로 잠근다 —
// 0 을 받아 "담을 곳이 없는데 조용한" 계측이 되는 것이 최악이다 (fail-closed).
inline void linkProbeInit(struct LinkStallProbe* p, uint8_t capacity, uint32_t bootId)
{
  for (uint8_t i = 0; i < LINK_EVENT_RING_MAX; ++i) {
    p->ring[i].firstEpochMs = 0U;
    p->ring[i].lastEpochMs = 0U;
    p->ring[i].execUsMax = 0U;
    p->ring[i].idleUsMax = 0U;
    p->ring[i].burstId = 0U;
    p->ring[i].count = 0U;
    p->ring[i].phase = 0U;
    p->ring[i].publishSlot = LINK_SLOT_NONE;
    p->ring[i].code = 0U;
  }
  for (uint8_t i = 0; i < LINK_LANE_COUNT; ++i) {
    p->burstId[i] = 0U;
  }

  p->execMaxUs = 0U;
  p->idleMaxUs = 0U;
  p->lastLoopEndUs = 0U;
  p->loopStartUs = 0U;
  p->lastEventUs = 0U;
  p->lastPulseMs = 0U;

  p->evtSeq = 0U;
  p->evtDroppedTotal = 0U;
  p->evtDroppedDelta = 0U;
  p->pendingDroppedDelta = 0U;
  p->sampleSeq = 0U;
  p->pulseFail = 0U;
  p->bootId = bootId;

  p->lastSyncMs = 0U;
  p->nowMsAtSyncQuery = 0U;

  p->capacity = (capacity == 0U || capacity > LINK_EVENT_RING_MAX)
                    ? (uint8_t)LINK_EVENT_RING_MAX
                    : capacity;
  p->head = 0U;
  p->used = 0U;

  p->haveLastLoopEnd = false;
  p->haveLastEvent = false;
  p->havePulseSent = false;
  p->inLoop = false;
  p->syncEverOk = false;
}

// ── 내부: burst 경계 ────────────────────────────────────────────────────────
// 사건이 하나도 없이 BURST_GAP_US 가 지나면 모든 lane 의 burst 가 끝난 것이다.
// 게으르게(사건이 올 때) 판정한다 — burst_id 는 사건 시각에만 쓰이므로 별도 tick 이
// 필요 없고, tick 을 안 불러서 계측이 조용히 틀어지는 자리도 안 생긴다.
inline void linkProbeBreakBurstsIfIdle(struct LinkStallProbe* p, uint64_t nowUs)
{
  if (!p->haveLastEvent) {
    return;
  }
  if (nowUs - p->lastEventUs > (uint64_t)LINK_BURST_GAP_US) {
    for (uint8_t i = 0; i < LINK_LANE_COUNT; ++i) {
      ++p->burstId[i];
    }
  }
}

// ── 내부: 사건 한 건 기록 ───────────────────────────────────────────────────
// 접는 키 = (code, publish_slot, phase, burst_id) 넷 **전부**.
//
// 🔴 접기 후보는 ring 안의 **모든** 칸이다. "가장 최근 칸 하나" 로 두면 안 된다 —
//    08-17 의 7사건은 /odom 과 /imu 가 **같이** 멈춘 것이라 실패가 odom·imu·odom·imu
//    로 번갈아 온다. 그러면 새 사건의 키는 tail(다른 slot)과 절대 안 맞아 접기가
//    한 번도 일어나지 않고, 349ms 공백 하나가 ring 칸 32개를 요구한다.
//    깊이 16 은 즉시 넘치고 evt_dropped_delta>0 → 그 구간은 **전부 판정 불능**이 된다.
//    즉 tail-only 접기는 계약이 겨눈 바로 그 데이터에서 계측을 무효화한다.
//    (이 결함은 host harness ⓘ 가 잡았다 — 12건이 12칸이 됐다.)
//
//    전량 스캔 비용은 최대 16회 비교다. Teensy 4.1(600MHz)에서 사건 발생 시에만
//    도는 경로이므로 무시할 수 있고, 넘쳐서 판정을 잃는 비용과 비교 대상이 아니다.
// ⚠ 오래된 칸에 접히면 ring 의 칸 순서가 더 이상 시간 순이 아니다. 그래도 되는
//   이유: 호스트는 **접는 키로 다시 묶고** first/last epoch 로 시각을 복원하지,
//   칸 순서를 시간축으로 쓰지 않는다 (link_stall_classify.py group_events).
inline void linkProbeRecordEvent(struct LinkStallProbe* p,
                                 uint8_t code,
                                 uint8_t slot,
                                 uint8_t phase,
                                 uint32_t execUs,
                                 uint32_t idleUs,
                                 uint64_t nowUs,
                                 uint64_t epochMs)
{
  linkProbeBreakBurstsIfIdle(p, nowUs);

  const uint8_t lane = (slot == LINK_SLOT_NONE)
                           ? (uint8_t)((code == LINK_EVENT_LOOP_INTERNAL)
                                           ? LINK_LANE_LOOP_INTERNAL
                                           : LINK_LANE_BETWEEN_LOOPS)
                           : slot;
  const uint32_t burst = p->burstId[lane];

  // 발생 자체는 언제나 센다. 담지 못한 것은 evt_dropped 가 따로 센다 —
  // 그래야 host 가 evt_seq - dropped 와 실제 수신 건수를 대조해 **사건 publisher
  // 자체의 발행 실패**를 볼 수 있다 (분류표 1행). 이 대조가 유일한 관측 경로다.
  ++p->evtSeq;
  p->haveLastEvent = true;
  p->lastEventUs = nowUs;

  for (uint8_t i = 0; i < p->used; ++i) {
    const uint8_t idx = (uint8_t)((p->head + i) % p->capacity);
    struct LinkEvent* e = &p->ring[idx];
    if (e->code == code && e->publishSlot == slot && e->phase == phase &&
        e->burstId == burst) {
      if (epochMs > e->lastEpochMs) {
        e->lastEpochMs = epochMs;
      }
      if (epochMs < e->firstEpochMs) {
        e->firstEpochMs = epochMs;
      }
      if (execUs > e->execUsMax) {
        e->execUsMax = execUs;
      }
      if (idleUs > e->idleUsMax) {
        e->idleUsMax = idleUs;
      }
      ++e->count;
      return;
    }
  }

  if (p->used >= p->capacity) {
    // 🔴 넘치면 **버리고 센다**. 오래된 칸을 덮어쓰지 않는 이유: 덮어쓰면 공백의
    //    시작이 사라져 first_epoch 보존 의무가 깨지고, 무엇을 잃었는지도 모른다.
    ++p->evtDroppedTotal;
    ++p->evtDroppedDelta;
    return;
  }

  const uint8_t idx = (uint8_t)((p->head + p->used) % p->capacity);
  struct LinkEvent* e = &p->ring[idx];
  e->firstEpochMs = epochMs;
  e->lastEpochMs = epochMs;
  e->execUsMax = execUs;
  e->idleUsMax = idleUs;
  e->burstId = burst;
  e->count = 1U;
  e->phase = phase;
  e->publishSlot = slot;
  e->code = code;
  ++p->used;
}

// ── loop 시작 — 판 사이(idle) 를 여기서 닫는다 ──────────────────────────────
// 🔴 배타성의 근거가 이 함수와 linkProbeLoopEnd 의 분리다. 초판이 쓴 "판 꼭대기
//    간격"은 직전 loop 의 실행시간을 포함하므로, loop **안** 300ms 주입이
//    다음 꼭대기 간격을 넘겨 "판 사이"로 오분류됐다 (§78.3).
inline void linkProbeLoopBegin(struct LinkStallProbe* p, uint64_t nowUs, uint64_t epochMs)
{
  if (p->haveLastLoopEnd) {
    const uint64_t idle64 = nowUs - p->lastLoopEndUs;
    const uint32_t idleUs =
        (idle64 > 0xFFFFFFFFULL) ? 0xFFFFFFFFUL : (uint32_t)idle64;
    if (idleUs > p->idleMaxUs) {
      p->idleMaxUs = idleUs;
    }
    if (idleUs > LINK_IDLE_EVENT_US) {
      linkProbeRecordEvent(p, LINK_EVENT_BETWEEN_LOOPS, LINK_SLOT_NONE,
                           RUNTIME_PHASE_LOOP, 0U, idleUs, nowUs, epochMs);
    }
  }
  p->loopStartUs = nowUs;
  p->inLoop = true;
}

// ── loop 끝 — loop 안(exec) 을 여기서 닫는다 ────────────────────────────────
inline void linkProbeLoopEnd(struct LinkStallProbe* p, uint64_t nowUs, uint64_t epochMs)
{
  if (p->inLoop) {
    const uint64_t exec64 = nowUs - p->loopStartUs;
    const uint32_t execUs =
        (exec64 > 0xFFFFFFFFULL) ? 0xFFFFFFFFUL : (uint32_t)exec64;
    if (execUs > p->execMaxUs) {
      p->execMaxUs = execUs;
    }
    if (execUs > LINK_EXEC_EVENT_US) {
      linkProbeRecordEvent(p, LINK_EVENT_LOOP_INTERNAL, LINK_SLOT_NONE,
                           RUNTIME_PHASE_LOOP, execUs, 0U, nowUs, epochMs);
    }
  }
  p->lastLoopEndUs = nowUs;
  p->haveLastLoopEnd = true;
  p->inLoop = false;
}

// ── publish 결과 ────────────────────────────────────────────────────────────
// 성공하면 그 slot 의 burst 를 끊는다 (계약 3판 "같은 slot 의 성공 publish 1회").
// 실패하면 발행 층 사건 한 건.
//
// 🔴 pulse slot 은 여기서 **헤더가 막는다** — 호출자에게 맡기지 않는다.
//    pulse 실패가 사건을 낳고 그 사건 발행이 또 실패하면 무한 재귀다.
//    "부르는 쪽이 조심한다"는 계약은 다음 세션에 깨진다 (AGENTS §3-10 ②:
//    열거를 검사기 안으로 옮긴다).
inline void linkProbePublishResult(struct LinkStallProbe* p,
                                   uint8_t slot,
                                   bool publishOk,
                                   uint8_t phase,
                                   uint64_t nowUs,
                                   uint64_t epochMs)
{
  if (slot >= RUNTIME_PUBLISH_COUNT) {
    return;
  }
  if (slot == RUNTIME_PUBLISH_PULSE) {
    if (!publishOk) {
      ++p->pulseFail;
    } else {
      ++p->burstId[slot];
    }
    return;
  }
  if (publishOk) {
    ++p->burstId[slot];
    return;
  }
  linkProbeRecordEvent(p, LINK_EVENT_PUBLISH_FAIL, slot, phase, 0U, 0U, nowUs, epochMs);
}

// ── TIME_SYNC 결과 ──────────────────────────────────────────────────────────
// rmw_uros_sync_session() 의 반환값을 **버리지 않는다** (계약 3판 ③).
// 동기가 실패했거나 낡으면 epoch_ms 는 uptime fallback 이거나 낡은 offset 이라
// bag 수신 시각과 대응할 수 없다 — 그 구간은 판정 불능이지 원인이 아니다.
inline void linkProbeSyncResult(struct LinkStallProbe* p, bool ok, uint32_t nowMs)
{
  if (ok) {
    p->syncEverOk = true;
    p->lastSyncMs = nowMs;
  }
}

inline uint32_t linkProbeSyncAgeMs(const struct LinkStallProbe* p, uint32_t nowMs)
{
  if (!p->syncEverOk) {
    return LINK_SYNC_AGE_NEVER;
  }
  return nowMs - p->lastSyncMs;
}

inline bool linkProbeSyncUsable(const struct LinkStallProbe* p, uint32_t nowMs)
{
  if (!p->syncEverOk) {
    return false;
  }
  return linkProbeSyncAgeMs(p, nowMs) <= LINK_SYNC_AGE_LIMIT_MS;
}

// ── 생존 표본 ───────────────────────────────────────────────────────────────
// 🔴 밀린 만큼 몰아서 올리지 않는다 (catch-up 금지). lastPulseMs = nowMs 다.
//    몰아 올리면 loop 가 300ms 섰다 깨어날 때도 sample_seq 가 +3 이 되어
//    "loop 가 섰다"와 "host 가 잃었다"가 **같은 값**이 된다 — 판정이 무너진다.
inline bool linkProbePulseDue(const struct LinkStallProbe* p, uint32_t nowMs)
{
  if (!p->havePulseSent) {
    return true;
  }
  return (uint32_t)(nowMs - p->lastPulseMs) >= LINK_PULSE_PERIOD_MS;
}

// sample_seq 는 pulse tick 마다 증가한다.
//
// ⚠ 계약 3판 ④ 는 "매 판 증가"라고 쓰지만 ⑤ 와 분류표 8행은 "증가량이 공백
//   길이(÷100ms)와 맞는가"로 판정한다. 두 문장은 같이 성립하지 않는다 — loop 주기가
//   1~47ms 로 흔들려서, 매 판 증가면 300ms 공백의 기대 증가량이 범위가 되고 8행이
//   다시 평가 불가가 된다(§79.2 가 닫은 실패 모양). 판정을 내는 ⑤ 를 채택한다.
//   🔴 이 선택은 구현자 단독 판단이며 계약과 함께 검토에 올린다.
inline void linkProbeBuildPulse(struct LinkStallProbe* p,
                                uint32_t nowMs,
                                uint64_t epochMs,
                                struct LinkPulse* out)
{
  ++p->sampleSeq;
  p->lastPulseMs = nowMs;
  p->havePulseSent = true;

  p->pendingDroppedDelta = p->evtDroppedDelta;

  out->bootId = p->bootId;
  out->sampleSeq = p->sampleSeq;
  out->epochMs = epochMs;
  out->syncOk = linkProbeSyncUsable(p, nowMs);
  out->syncAgeMs = linkProbeSyncAgeMs(p, nowMs);
  out->evtSeq = p->evtSeq;
  out->evtDroppedTotal = p->evtDroppedTotal;
  out->evtDroppedDelta = p->evtDroppedDelta;
  out->pulseFail = p->pulseFail;
}

// 발행이 **성공했을 때만** delta 를 턴다.
// 🔴 만들 때 털면 그 pulse 가 안 나갔을 때 delta 가 영원히 사라져, 판정 불능이어야
//    할 구간이 조용히 유효로 보인다. 실패하면 계속 쌓아 다음 표본에 실어 보낸다
//    (fail-closed). 빼기로 터는 이유는 만든 뒤 보낸 사이에 늘어난 몫을 안 잃기 위해서다.
// ⚠ 여기서 pulseFail 을 세지 않는다 — 실패 계수는 linkProbePublishResult 한 곳이
//    소유한다. 두 곳에서 세면 같은 실패가 2 로 보이고, 그 수를 근거로 판정하는
//    사람이 계측을 못 믿게 된다. 서식이 잘려 발행조차 못 한 경우도 호출자가
//    linkProbePublishResult(..., false, ...) 를 거쳐 한 곳에서 세게 한다.
inline void linkProbePulseSent(struct LinkStallProbe* p, bool publishOk)
{
  if (!publishOk) {
    p->pendingDroppedDelta = 0U;
    return;
  }
  p->evtDroppedDelta -= p->pendingDroppedDelta;
  p->pendingDroppedDelta = 0U;
}

// ── 사건 배출 ───────────────────────────────────────────────────────────────
// 가장 오래된 칸부터 꺼낸다. 꺼내도 evt_seq 는 줄지 않는다 — host 가 수신 건수와
// 대조할 기준이 evt_seq 이기 때문이다.
inline bool linkProbeDrain(struct LinkStallProbe* p, struct LinkEvent* out)
{
  if (p->used == 0U) {
    return false;
  }
  *out = p->ring[p->head];
  p->head = (uint8_t)((p->head + 1U) % p->capacity);
  --p->used;
  return true;
}

inline uint8_t linkProbePending(const struct LinkStallProbe* p)
{
  return p->used;
}

// ── 전문 서식 ───────────────────────────────────────────────────────────────
// 서식을 헤더에 두는 이유: .ino 와 host harness 가 **같은 함수**를 쓰게 해서
// "펌웨어가 보내는 글자"와 "harness 가 검사한 글자"가 갈라질 자리를 없앤다.
// std_msgs/String 이라 `ros2 topic echo` 로 사람이 그대로 읽을 수 있다 —
// 16:00 결정점에서 도구 없이 눈으로 봐야 하는 값이다.
//
// 반환값 = snprintf 반환값. cap 이상이면 **잘렸다는 뜻**이고 호출자는 그 표본을
// 버려야 한다. 조용히 자르고 성공한 척하지 않는다.
inline int linkPulseFormat(const struct LinkPulse* p, char* buf, int cap)
{
  return snprintf(buf, (size_t)cap, "P,%lu,%lu,%llu,%d,%lu,%lu,%lu,%lu,%lu",
                  (unsigned long)p->bootId, (unsigned long)p->sampleSeq,
                  (unsigned long long)p->epochMs, p->syncOk ? 1 : 0,
                  (unsigned long)p->syncAgeMs, (unsigned long)p->evtSeq,
                  (unsigned long)p->evtDroppedTotal,
                  (unsigned long)p->evtDroppedDelta,
                  (unsigned long)p->pulseFail);
}

inline int linkEventFormat(const struct LinkEvent* e, char* buf, int cap)
{
  return snprintf(buf, (size_t)cap, "E,%u,%u,%u,%lu,%llu,%llu,%lu,%lu,%lu",
                  (unsigned)e->code, (unsigned)e->phase, (unsigned)e->publishSlot,
                  (unsigned long)e->burstId, (unsigned long long)e->firstEpochMs,
                  (unsigned long long)e->lastEpochMs, (unsigned long)e->execUsMax,
                  (unsigned long)e->idleUsMax, (unsigned long)e->count);
}

#endif  // LINK_STALL_PROBE_H
