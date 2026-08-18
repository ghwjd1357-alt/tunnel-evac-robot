// ============================================================================
// link_stall_host_test.cpp — 예약 41-g 사건 계측 주입 harness (PC 전용)
//
// 실행:  bash tools/link_stall_host_test.sh
// 대상:  firmware/teensy_integrated_base_v1_4/link_stall_probe.h
//        firmware/teensy_integrated_base_v1_4/runtime_guard.h
//
// 왜 이 파일이 있나 (계획 §4 · 검토 §78.4):
//   "B1 의 산출물은 헤더가 아니라 harness 다. compile 성공은 착수선이지 완료선이
//    아니다." 41-g 완료판정은 **합성 주입이 서로 다른 분류로 갈릴 때**이고,
//   자연 재현 0건은 R3 관측이지 41-g 종결이 아니다.
//
// 시계가 가짜인 것이 요점이다 — 300ms 를 실제로 기다리지 않고 정확히 찍는다.
// 실기로는 loop **안** 300ms 와 판 **사이** 300ms 를 따로 만들 수가 없다.
//
// 이 harness 는 두 겹으로 판정한다:
//   1) MCU 쪽 불변식 — 여기 C++ 에서 직접 본다 (사건 개수·코드·burst·drop).
//   2) 호스트 쪽 분류 — 수신 스트림을 JSONL 로 뱉고, 9행 분류표의 유일한 구현인
//      tools/link_stall_classify.py 가 판정한다. 표를 두 벌 쓰지 않는다.
//
// 🔴 이 harness 가 증명하지 않는 것 (숨기지 않는다):
//   - 스케치가 이 함수들을 부르는가 → 실행 스크립트 2단계 구조 검사(텍스트, 약한 증거)
//   - 실제 USB/agent 가 300ms 서는가 → 그건 B3 실기 bag 이 관측한다
//   - 복귀하지 않는 영구 정지의 원인 → §79.2 가 공개한 한계. "시행 종료" 로만 적는다
// ============================================================================

#include "../firmware/teensy_integrated_base_v1_4/link_stall_probe.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

static int g_checks = 0;
static int g_failures = 0;
static std::string g_outDir;

static void expectU32(const char* what, uint32_t got, uint32_t want)
{
    ++g_checks;
    if (got != want) {
        std::printf("  FAIL %-62s = %u (기대 %u)\n", what, (unsigned)got, (unsigned)want);
        ++g_failures;
    }
}

static void expectU64(const char* what, uint64_t got, uint64_t want)
{
    ++g_checks;
    if (got != want) {
        std::printf("  FAIL %-62s = %llu (기대 %llu)\n", what,
                    (unsigned long long)got, (unsigned long long)want);
        ++g_failures;
    }
}

static void expectBool(const char* what, bool got, bool want)
{
    ++g_checks;
    if (got != want) {
        std::printf("  FAIL %-62s = %s (기대 %s)\n", what,
                    got ? "true" : "false", want ? "true" : "false");
        ++g_failures;
    }
}

// pulse 전문 되파싱 — 서식이 실제로 읽히는 글자인지 확인한다.
static bool parsePulse(const char* text, LinkPulse& out)
{
    unsigned long boot = 0, seq = 0, age = 0, evt = 0, dt = 0, dd = 0, pf = 0;
    unsigned long long epoch = 0;
    int ok = 0;
    const int n = std::sscanf(text, "P,%lu,%lu,%llu,%d,%lu,%lu,%lu,%lu,%lu",
                              &boot, &seq, &epoch, &ok, &age, &evt, &dt, &dd, &pf);
    if (n != 9) {
        return false;
    }
    out.bootId = (uint32_t)boot;
    out.sampleSeq = (uint32_t)seq;
    out.epochMs = (uint64_t)epoch;
    out.syncOk = (ok != 0);
    out.syncAgeMs = (uint32_t)age;
    out.evtSeq = (uint32_t)evt;
    out.evtDroppedTotal = (uint32_t)dt;
    out.evtDroppedDelta = (uint32_t)dd;
    out.pulseFail = (uint32_t)pf;
    return true;
}

// ── 보드 모형 ───────────────────────────────────────────────────────────────
// .ino 의 loop() 순서를 그대로 흉내낸다. 실제 호출 여부는 구조 검사가 본다.
struct Board {
    LinkStallProbe probe;
    uint64_t nowUs = 0;
    uint64_t epochBaseMs = 1000;   // 동기된 벽시계의 출발점 (읽기 쉬운 작은 값)
    bool hostRecording = true;     // host recorder/agent 가 살아 있는가
    std::FILE* out = nullptr;

    // 수신 스트림에 실제로 실린 것만 센다 — MCU 계수와 대조하기 위해서다.
    uint32_t emittedEvents = 0;
    uint32_t emittedPulses = 0;

    // 🔴 ring 은 매 판 배출되므로 "지금 ring 에 몇 건" 을 세면 항상 0 이다.
    //    사건이 실제로 몇 건 났는지는 **배출분 + 잔량** 으로만 알 수 있다.
    uint32_t drainedCount[3] = {0, 0, 0};
    uint32_t drainedEntries[3] = {0, 0, 0};
    LinkEvent lastDrained{};

    // 사건이 배출보다 빨리 쌓이는 구간을 만들기 위한 스위치 (ⓙ 깊이 초과).
    bool drainEnabled = true;

    uint32_t nowMs() const { return (uint32_t)(nowUs / 1000ULL); }
    uint64_t epochMs() const { return epochBaseMs + nowUs / 1000ULL; }

    void openStream(const std::string& name)
    {
        const std::string path = g_outDir + "/" + name + ".jsonl";
        out = std::fopen(path.c_str(), "w");
        if (out == nullptr) {
            std::printf("  FAIL 스트림 파일을 못 연다: %s\n", path.c_str());
            ++g_failures;
        }
    }

    void closeStream()
    {
        if (out != nullptr) {
            std::fclose(out);
            out = nullptr;
        }
    }

    void advance(uint32_t us) { nowUs += us; }

    void syncOk() { linkProbeSyncResult(&probe, true, nowMs()); }

    // 🔴 헤더의 서식 함수를 **거쳐서** 텍스트를 만들고, 그 텍스트를 되파싱해
    //    JSON 을 뱉는다. 그래야 "펌웨어가 보내는 글자"가 실제로 분류 가능한지
    //    검사된다 — JSON 을 직접 만들면 서식 버그가 harness 를 통과한다.
    void emitPulse(const LinkPulse& p)
    {
        char text[LINK_PULSE_TEXT_MAX];
        const int n = linkPulseFormat(&p, text, (int)sizeof(text));
        ++g_checks;
        if (n < 0 || n >= (int)sizeof(text)) {
            std::printf("  FAIL pulse 전문이 %dB 로 %uB 상한을 넘었다\n", n,
                        (unsigned)sizeof(text));
            ++g_failures;
            return;
        }
        LinkPulse rt{};
        if (!parsePulse(text, rt)) {
            ++g_checks;
            std::printf("  FAIL pulse 전문을 되파싱하지 못했다: %s\n", text);
            ++g_failures;
            return;
        }
        if (out == nullptr || !hostRecording) {
            return;
        }
        (void)rt;
        std::fprintf(out,
                     "{\"t\":\"pulse\",\"boot_id\":%u,\"sample_seq\":%u,"
                     "\"epoch_ms\":%llu,\"sync_ok\":%s,\"sync_age_ms\":%u,"
                     "\"evt_seq\":%u,\"evt_dropped_total\":%u,"
                     "\"evt_dropped_delta\":%u,\"pulse_fail\":%u}\n",
                     (unsigned)p.bootId, (unsigned)p.sampleSeq,
                     (unsigned long long)p.epochMs, p.syncOk ? "true" : "false",
                     (unsigned)p.syncAgeMs, (unsigned)p.evtSeq,
                     (unsigned)p.evtDroppedTotal, (unsigned)p.evtDroppedDelta,
                     (unsigned)p.pulseFail);
        ++emittedPulses;
    }

    void emitEvent(const LinkEvent& e)
    {
        char text[LINK_EVENT_TEXT_MAX];
        const int n = linkEventFormat(&e, text, (int)sizeof(text));
        ++g_checks;
        if (n < 0 || n >= (int)sizeof(text)) {
            std::printf("  FAIL event 전문이 %dB 로 %uB 상한을 넘었다\n", n,
                        (unsigned)sizeof(text));
            ++g_failures;
            return;
        }
        if (out == nullptr || !hostRecording) {
            return;
        }
        std::fprintf(out,
                     "{\"t\":\"event\",\"code\":%u,\"phase\":%u,\"slot\":%u,"
                     "\"burst_id\":%u,\"first_epoch_ms\":%llu,"
                     "\"last_epoch_ms\":%llu,\"exec_us_max\":%u,"
                     "\"idle_us_max\":%u,\"count\":%u}\n",
                     (unsigned)e.code, (unsigned)e.phase, (unsigned)e.publishSlot,
                     (unsigned)e.burstId, (unsigned long long)e.firstEpochMs,
                     (unsigned long long)e.lastEpochMs, (unsigned)e.execUsMax,
                     (unsigned)e.idleUsMax, (unsigned)e.count);
        ++emittedEvents;
    }

    // .ino loop() 꼬리에서 하는 일: 생존 표본 → 사건 배출.
    // pulse 를 먼저 보내는 이유 — 사건이 밀려도 생존 표본은 주기를 지켜야 한다.
    void servicePulseAndDrain(bool pulsePublishOk = true, bool eventPublishOk = true)
    {
        if (linkProbePulseDue(&probe, nowMs())) {
            LinkPulse pulse;
            linkProbeBuildPulse(&probe, nowMs(), epochMs(), &pulse);
            // publishMeasured 경유 = slot 8. 실패해도 사건을 만들지 않는다.
            linkProbePublishResult(&probe, RUNTIME_PUBLISH_PULSE, pulsePublishOk,
                                   RUNTIME_PHASE_LOOP, nowUs, epochMs());
            linkProbePulseSent(&probe, pulsePublishOk);
            if (pulsePublishOk) {
                emitPulse(pulse);
            }
        }

        if (!drainEnabled) {
            return;
        }
        LinkEvent e;
        uint8_t drained = 0;
        while (drained < LINK_DRAIN_PER_LOOP && linkProbeDrain(&probe, &e)) {
            ++drained;
            if (e.code < 3U) {
                drainedCount[e.code] += e.count;
                ++drainedEntries[e.code];
            }
            lastDrained = e;
            if (eventPublishOk) {
                emitEvent(e);
            }
            // 🔴 사건 발행 실패는 사건을 또 만들지 않는다. host 는 evt_seq 회계로 본다.
        }
    }
};

// 한 판. execUs = 이 loop 이 안에서 쓴 시간. publishes = (slot, ok) 목록.
struct Pub {
    uint8_t slot;
    bool ok;
};

static void runLoop(Board& b, uint32_t execUs, const std::vector<Pub>& pubs,
                    bool pulseOk = true, bool eventOk = true)
{
    linkProbeLoopBegin(&b.probe, b.nowUs, b.epochMs());

    // publish 들은 loop 앞쪽에서 일어난다. 남은 실행시간은 뒤에 붙인다.
    for (const Pub& p : pubs) {
        b.advance(50);
        linkProbePublishResult(&b.probe, p.slot, p.ok, RUNTIME_PHASE_ODOM,
                               b.nowUs, b.epochMs());
    }

    const uint32_t used = (uint32_t)(50U * pubs.size());
    if (execUs > used) {
        b.advance(execUs - used);
    }

    b.servicePulseAndDrain(pulseOk, eventOk);
    linkProbeLoopEnd(&b.probe, b.nowUs, b.epochMs());
}

// 판 사이 시간 (기본 = delay(1) 상당).
static void runIdle(Board& b, uint32_t idleUs) { b.advance(idleUs); }

// 정상 주행 n 판. 22ms 주기를 흉내낸다.
static void runQuiet(Board& b, int loops, uint32_t execUs = 2000,
                     uint32_t idleUs = 20000)
{
    // 🔴 idle 을 **먼저** 둔다. loop 를 먼저 돌리면 runQuiet 이 idle 로 끝나서,
    //    뒤이어 주입하는 300ms 판 사이가 320ms 가 된다 — 주입값과 관측값이 갈린다.
    for (int i = 0; i < loops; ++i) {
        runIdle(b, idleUs);
        runLoop(b, execUs, {});
    }
}

static void bootBoard(Board& b, const std::string& stream, uint8_t capacity = 16,
                      uint32_t bootId = 1)
{
    linkProbeInit(&b.probe, capacity, bootId);
    b.nowUs = 0;
    b.hostRecording = true;
    b.emittedEvents = 0;
    b.emittedPulses = 0;
    b.openStream(stream);
    b.syncOk();
}

// meta 파일 — 실행 스크립트가 이걸 읽어 분류기를 호출한다.
static void writeMeta(const std::string& name, uint64_t gapStart, uint64_t gapEnd,
                      const char* expect, int expectGroups)
{
    const std::string path = g_outDir + "/" + name + ".meta";
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (f == nullptr) {
        std::printf("  FAIL meta 를 못 쓴다: %s\n", path.c_str());
        ++g_failures;
        return;
    }
    std::fprintf(f, "GAP_START=%llu\nGAP_END=%llu\nEXPECT=%s\nEXPECT_GROUPS=%d\n",
                 (unsigned long long)gapStart, (unsigned long long)gapEnd, expect,
                 expectGroups);
    std::fclose(f);
}

// 특정 code 의 **발생 건수** = 이미 배출된 몫 + ring 잔량.
static uint32_t countCode(const Board& b, uint8_t code)
{
    uint32_t n = (code < 3U) ? b.drainedCount[code] : 0U;
    const LinkStallProbe& p = b.probe;
    for (uint8_t i = 0; i < p.used; ++i) {
        const uint8_t idx = (uint8_t)((p.head + i) % p.capacity);
        if (p.ring[idx].code == code) {
            n += p.ring[idx].count;
        }
    }
    return n;
}

// 특정 code 의 **칸 수** = 접기가 몇 칸으로 남겼는가.
static uint32_t countEntries(const Board& b, uint8_t code)
{
    uint32_t n = (code < 3U) ? b.drainedEntries[code] : 0U;
    const LinkStallProbe& p = b.probe;
    for (uint8_t i = 0; i < p.used; ++i) {
        const uint8_t idx = (uint8_t)((p.head + i) % p.capacity);
        if (p.ring[idx].code == code) {
            ++n;
        }
    }
    return n;
}

// ═══════════════════════════════════════════════════════════════════════════
// ⓐ loop **안** 300ms → LOOP_INTERNAL 하나만.
// 🔴 BETWEEN_LOOPS 가 같이 뜨면 FAIL — 초판이 정확히 여기서 무너졌다 (§78.3).
//    "판 꼭대기 간격" 하나로 재면 이 주입이 다음 꼭대기 간격을 300ms 넘겨
//    "판 사이" 로도 잡힌다. exec/idle 을 따로 재는 것이 그 오분류를 막는 유일한 근거다.
// ═══════════════════════════════════════════════════════════════════════════
static void a_loop_internal()
{
    std::printf("\nⓐ loop 안 300ms\n");
    Board b;
    bootBoard(b, "a_loop_internal");

    runQuiet(b, 20);
    const uint64_t gapStart = b.epochMs();

    runIdle(b, 20000);               // 정상 판 사이
    runLoop(b, 300000, {});          // ← loop 안에서 300ms
    const uint64_t gapEnd = b.epochMs();

    runQuiet(b, 20);

    expectU32("ⓐ LOOP_INTERNAL 발생", countCode(b, LINK_EVENT_LOOP_INTERNAL), 1);
    expectU32("ⓐ 🔴 BETWEEN_LOOPS 는 0 이어야 한다",
              countCode(b, LINK_EVENT_BETWEEN_LOOPS), 0);
    expectU32("ⓐ PUBLISH_FAIL 0", countCode(b, LINK_EVENT_PUBLISH_FAIL), 0);
    expectU32("ⓐ evt_dropped 0", b.probe.evtDroppedTotal, 0);
    // 누적 최대는 역회귀용으로 살아 있다.
    expectU32("ⓐ 누적 exec 최대", b.probe.execMaxUs, 300000);
    // 배출된 칸에도 first/last epoch 이 남는다 (보존 의무).
    expectU64("ⓐ 사건 first_epoch = 공백 끝", b.lastDrained.firstEpochMs, gapEnd);
    expectU64("ⓐ 사건 last_epoch = 공백 끝", b.lastDrained.lastEpochMs, gapEnd);
    expectU32("ⓐ 배출된 칸의 code", b.lastDrained.code, LINK_EVENT_LOOP_INTERNAL);

    b.servicePulseAndDrain();
    runQuiet(b, 5);
    b.closeStream();
    writeMeta("a_loop_internal", gapStart, gapEnd, "LOOP_INTERNAL", 1);
}

// ═══════════════════════════════════════════════════════════════════════════
// ⓑ 끝→시작 300ms → BETWEEN_LOOPS 만.
// ═══════════════════════════════════════════════════════════════════════════
static void b_between_loops()
{
    std::printf("\nⓑ 판 사이 300ms\n");
    Board b;
    bootBoard(b, "b_between_loops");

    runQuiet(b, 20);
    const uint64_t gapStart = b.epochMs();

    runIdle(b, 300000);              // ← 판 사이에서 300ms
    runLoop(b, 2000, {});
    const uint64_t gapEnd = b.epochMs();

    runQuiet(b, 20);

    expectU32("ⓑ BETWEEN_LOOPS 발생", countCode(b, LINK_EVENT_BETWEEN_LOOPS), 1);
    expectU32("ⓑ 🔴 LOOP_INTERNAL 은 0 이어야 한다",
              countCode(b, LINK_EVENT_LOOP_INTERNAL), 0);
    expectU32("ⓑ 누적 idle 최대", b.probe.idleMaxUs, 300000);

    b.servicePulseAndDrain();
    runQuiet(b, 5);
    b.closeStream();
    writeMeta("b_between_loops", gapStart, gapEnd, "BETWEEN_LOOPS", 1);
}

// ═══════════════════════════════════════════════════════════════════════════
// ⓒ host recorder 300ms 정지 → MCU 사건 0건. pulse 는 계속 나가지만 host 가 못 받고,
//   복귀 후 첫 pulse 의 sample_seq 증가량이 공백 길이(÷100ms)와 맞는다.
// 🔴 이 줄이 08-17 의 답을 내는 줄이고, 2판은 여기가 비어 있었다 (§79.2).
// ═══════════════════════════════════════════════════════════════════════════
static void c_host_after()
{
    std::printf("\nⓒ host recorder 300ms 정지\n");
    Board b;
    bootBoard(b, "c_host_after");

    runQuiet(b, 30);
    const uint32_t seqBefore = b.probe.sampleSeq;
    const uint64_t gapStart = b.epochMs();

    // MCU 는 아무 일 없이 계속 돈다. 다만 host 가 못 받는다.
    b.hostRecording = false;
    runQuiet(b, 14, 2000, 20000);    // 약 308ms
    b.hostRecording = true;

    const uint64_t gapEnd = b.epochMs();
    runQuiet(b, 30);

    expectU32("ⓒ 사건 0건", b.probe.evtSeq, 0);
    expectU32("ⓒ evt_dropped 0", b.probe.evtDroppedTotal, 0);
    expectBool("ⓒ sample_seq 는 공백 동안에도 올라갔다",
               b.probe.sampleSeq - seqBefore >= 3, true);

    b.closeStream();
    writeMeta("c_host_after", gapStart, gapEnd, "HOST_AFTER", 0);
}

// ═══════════════════════════════════════════════════════════════════════════
// ⓓ loop 무복귀 → "시행 종료". 🔴 원인 분류를 만들지 않는다.
//   생존 표본도 loop() 에 매달려 있으므로 downstream 영구 정지와 갈리지 않는다.
//   이건 결함이 아니라 §79.2 가 **공개한 한계**다.
// ═══════════════════════════════════════════════════════════════════════════
static void d_run_ended()
{
    std::printf("\nⓓ loop 무복귀\n");
    Board b;
    bootBoard(b, "d_run_ended");

    runQuiet(b, 30);
    const uint64_t gapStart = b.epochMs();
    // 여기서 loop 이 돌아오지 않는다 — 더 이상 아무것도 하지 않는다.
    const uint64_t gapEnd = gapStart + 5000;

    expectU32("ⓓ 사건 0건", b.probe.evtSeq, 0);
    b.closeStream();
    writeMeta("d_run_ended", gapStart, gapEnd, "RUN_ENDED", 0);
}

// ═══════════════════════════════════════════════════════════════════════════
// ⓔ MCU reset → boot_id 교체 + sample_seq 0 부터. 이전 구간과 잇지 않는다.
// ═══════════════════════════════════════════════════════════════════════════
static void e_mcu_reset()
{
    std::printf("\nⓔ MCU reset\n");
    Board b;
    bootBoard(b, "e_mcu_reset");

    runQuiet(b, 30);
    const uint64_t gapStart = b.epochMs();

    // 리셋: 시각은 계속 흐르지만 probe 는 처음부터다.
    const uint64_t keepUs = b.nowUs + 400000ULL;
    linkProbeInit(&b.probe, 16, 2);
    b.nowUs = keepUs;
    b.syncOk();

    expectU32("ⓔ boot_id 교체", b.probe.bootId, 2);
    expectU32("ⓔ sample_seq 0 부터", b.probe.sampleSeq, 0);

    runQuiet(b, 30);
    const uint64_t gapEnd = gapStart + 400;

    b.closeStream();
    writeMeta("e_mcu_reset", gapStart, gapEnd, "MCU_RESET", 0);
}

// ═══════════════════════════════════════════════════════════════════════════
// ⓕ PUBLISH_FAIL 단독 → 발행 층. loop 안도 판 사이도 아니다.
// ═══════════════════════════════════════════════════════════════════════════
static void f_publish_layer()
{
    std::printf("\nⓕ PUBLISH_FAIL 단독\n");
    Board b;
    bootBoard(b, "f_publish_layer");

    runQuiet(b, 20);
    const uint64_t gapStart = b.epochMs();

    // loop 은 정상 주기로 돌지만 odom publish 만 실패한다.
    for (int i = 0; i < 10; ++i) {
        runLoop(b, 2000, {{RUNTIME_PUBLISH_ODOM, false}});
        runIdle(b, 20000);
    }
    const uint64_t gapEnd = b.epochMs();

    runQuiet(b, 20);

    expectU32("ⓕ PUBLISH_FAIL 발생 10", b.probe.evtSeq, 10);
    expectU32("ⓕ LOOP_INTERNAL 0", countCode(b, LINK_EVENT_LOOP_INTERNAL), 0);
    // 🔴 배출은 매 판 일어나므로 칸은 10개로 **나뉘어** 나간다. 그래도 접는 키가
    //    같으므로 host 가 묶으면 1 burst 로 복원된다 — 그 복원은 분류기가 본다.
    expectU32("ⓕ 배출 칸 10개", countEntries(b, LINK_EVENT_PUBLISH_FAIL), 10);
    expectU32("ⓕ BETWEEN_LOOPS 0", countCode(b, LINK_EVENT_BETWEEN_LOOPS), 0);

    b.servicePulseAndDrain();
    runQuiet(b, 5);
    b.closeStream();
    // 22ms 주기 · BURST_GAP 50ms 미만이라 한 burst 로 접힌다.
    writeMeta("f_publish_layer", gapStart, gapEnd, "PUBLISH_LAYER", 1);
}

// ═══════════════════════════════════════════════════════════════════════════
// ⓖ loop 안 + 판 사이 동시 → 복합. 🔴 하나로 강제하면 FAIL.
// ═══════════════════════════════════════════════════════════════════════════
static void g_compound()
{
    std::printf("\nⓖ loop 안 + 판 사이 동시\n");
    Board b;
    bootBoard(b, "g_compound");

    runQuiet(b, 20);
    const uint64_t gapStart = b.epochMs();

    runIdle(b, 20000);
    runLoop(b, 200000, {});          // loop 안 200ms
    runIdle(b, 200000);              // 이어서 판 사이 200ms
    runLoop(b, 2000, {});
    const uint64_t gapEnd = b.epochMs();

    runQuiet(b, 20);

    expectU32("ⓖ LOOP_INTERNAL 1", countCode(b, LINK_EVENT_LOOP_INTERNAL), 1);
    expectU32("ⓖ BETWEEN_LOOPS 1", countCode(b, LINK_EVENT_BETWEEN_LOOPS), 1);

    b.servicePulseAndDrain();
    runQuiet(b, 5);
    b.closeStream();
    writeMeta("g_compound", gapStart, gapEnd, "COMPOUND", 2);
}

// ═══════════════════════════════════════════════════════════════════════════
// ⓗ 성공 publish 로 분리된 7 burst → 7개로 복원.
// 🔴 한 칸 count=70 이면 FAIL — 2판이 정확히 여기서 무너졌다 (§79.3).
//    성공 publish 는 ring 사건이 **아니므로** 떨어진 7개 공백의 PUBLISH_FAIL 70건이
//    event stream 에서는 연속이다. code 하나로 접으면 7개가 1개가 된다.
// ═══════════════════════════════════════════════════════════════════════════
static void h_seven_bursts()
{
    std::printf("\nⓗ 성공 publish 로 분리된 7 burst\n");
    Board b;
    bootBoard(b, "h_seven_bursts");

    runQuiet(b, 10);
    const uint64_t gapStart = b.epochMs();

    for (int burst = 0; burst < 7; ++burst) {
        // 한 공백 안에서 odom 10회 실패
        for (int i = 0; i < 10; ++i) {
            runLoop(b, 2000, {{RUNTIME_PUBLISH_ODOM, false}});
            runIdle(b, 20000);
            b.servicePulseAndDrain();
        }
        // 공백이 끝나고 성공 발행 1회 → 같은 slot 의 burst 를 끊는다
        runLoop(b, 2000, {{RUNTIME_PUBLISH_ODOM, true}});
        runIdle(b, 20000);
        b.servicePulseAndDrain();
        // 다음 공백까지 정상 구간
        runQuiet(b, 10);
        b.servicePulseAndDrain();
    }
    const uint64_t gapEnd = b.epochMs();

    expectU32("ⓗ 발생 70건", b.probe.evtSeq, 70);
    expectU32("ⓗ 버린 사건 0", b.probe.evtDroppedTotal, 0);

    b.servicePulseAndDrain();
    runQuiet(b, 5);
    b.closeStream();
    // 🔴 묶음 7 — 이 숫자가 §79.3 의 완료판정이다.
    writeMeta("h_seven_bursts", gapStart, gapEnd, "PUBLISH_LAYER", 7);
}

// ═══════════════════════════════════════════════════════════════════════════
// ⓘ 서로 다른 slot 교대 burst → slot 별로 갈린다.
// ═══════════════════════════════════════════════════════════════════════════
static void i_slot_split()
{
    std::printf("\nⓘ slot 교대 burst\n");
    Board b;
    bootBoard(b, "i_slot_split");

    runQuiet(b, 10);
    const uint64_t gapStart = b.epochMs();

    // 같은 burst 창 안에서 odom 과 imu 가 번갈아 실패한다.
    // 🔴 배출을 잠깐 막는다 — 매 판 비우면 접기가 일어난 적이 없어져서
    //    "slot 이 접는 키에 들어 있는가" 를 ring 으로 볼 수 없다.
    b.drainEnabled = false;
    for (int i = 0; i < 6; ++i) {
        runIdle(b, 20000);
        runLoop(b, 2000, {{RUNTIME_PUBLISH_ODOM, false}, {RUNTIME_PUBLISH_IMU, false}});
    }
    const uint64_t gapEnd = b.epochMs();

    expectU32("ⓘ 발생 12건", b.probe.evtSeq, 12);
    // slot 이 키에 들어가므로 12건이 두 칸(odom · imu)으로 접힌다.
    // 🔴 slot 을 키에서 빼면 한 칸 count=12 가 되어 slot 이 사라진다.
    expectU32("ⓘ 🔴 ring 칸 2개 (slot 별)", linkProbePending(&b.probe), 2);
    b.drainEnabled = true;

    b.servicePulseAndDrain();
    runQuiet(b, 5);
    b.closeStream();
    writeMeta("i_slot_split", gapStart, gapEnd, "PUBLISH_LAYER", 2);
}

// ═══════════════════════════════════════════════════════════════════════════
// ⓙ 깊이 초과 → **그 구간만** 판정 불능. drain 뒤 구간은 다시 유효하다.
//   🔴 누계만 두면 한 번 넘친 뒤 모든 미래 구간이 영구 판정 불능이 된다 —
//      계측이 스스로를 영구히 무효화한다. delta 를 따로 두는 이유가 이것이다.
// ═══════════════════════════════════════════════════════════════════════════
static void j_overflow(uint8_t capacity, int distinct, const char* name)
{
    std::printf("\nⓙ 깊이 %u 에 %d 건\n", (unsigned)capacity, distinct);
    Board b;
    bootBoard(b, name, capacity);

    runQuiet(b, 10);
    const uint64_t gapStart = b.epochMs();

    // 🔴 배출을 막는다 — 사건이 배출보다 빨리 쌓이는 구간을 만드는 것이
    //    깊이 초과의 유일한 현실 경로다. 매 판 비우면 깊이는 영원히 안 찬다.
    b.drainEnabled = false;

    // 서로 다른 slot/burst 를 만들어 접히지 않게 한다.
    for (int i = 0; i < distinct; ++i) {
        const uint8_t slot = (uint8_t)(i % (RUNTIME_PUBLISH_COUNT - 1));
        runLoop(b, 2000, {{slot, false}});
        // 접히지 않도록 BURST_GAP 을 넘겨 burst 를 끊는다
        runIdle(b, 60000);
        linkProbeLoopBegin(&b.probe, b.nowUs, b.epochMs());
        linkProbeLoopEnd(&b.probe, b.nowUs, b.epochMs());
    }
    const uint64_t gapEnd = b.epochMs();

    const uint32_t over = (uint32_t)distinct - capacity;
    expectU32("ⓙ 버린 사건 누계", b.probe.evtDroppedTotal, over);
    expectU32("ⓙ 버린 사건 구간 delta", b.probe.evtDroppedDelta, over);

    // 배출하고 pulse 를 한 번 성공시키면 delta 가 0 으로 돌아온다.
    b.drainEnabled = true;
    b.servicePulseAndDrain();
    b.advance(200000);
    b.servicePulseAndDrain();
    expectU32("ⓙ drain 뒤 delta 0", b.probe.evtDroppedDelta, 0);
    expectU32("ⓙ 🔴 누계는 남는다 (역회귀용)", b.probe.evtDroppedTotal, over);

    runQuiet(b, 20);
    b.closeStream();
    writeMeta(name, gapStart, gapEnd, "UNDECIDABLE_INSTRUMENT", -1);
}

// ═══════════════════════════════════════════════════════════════════════════
// 판정 불능이 원인 분류로 새지 않는가 — 계약이 "전부 판정 불능으로 고정" 하라고
// 명시한 사유들을 하나씩 만든다.
// ═══════════════════════════════════════════════════════════════════════════
static void k_undecidable_sync()
{
    std::printf("\nⓚ sync 실패 구간\n");
    Board b;
    bootBoard(b, "k_undecidable_sync");

    // 한 번도 동기에 성공하지 않은 상태로 시작한다.
    linkProbeInit(&b.probe, 16, 1);
    expectBool("ⓚ 동기 이력 없으면 sync_ok=false",
               linkProbeSyncUsable(&b.probe, b.nowMs()), false);
    expectU32("ⓚ 동기 이력 없으면 age = sentinel",
              linkProbeSyncAgeMs(&b.probe, b.nowMs()), LINK_SYNC_AGE_NEVER);

    runQuiet(b, 30);
    const uint64_t gapStart = b.epochMs();
    runIdle(b, 300000);
    runQuiet(b, 30);
    const uint64_t gapEnd = gapStart + 300;

    b.closeStream();
    writeMeta("k_undecidable_sync", gapStart, gapEnd, "UNDECIDABLE_INSTRUMENT", -1);
}

// sync 가 낡으면 (30초 주기 2회 실패 초과) 판정 불능이다.
static void k_sync_age()
{
    std::printf("\nⓚ' sync 나이 초과\n");
    Board b;
    bootBoard(b, "k_sync_age");
    b.closeStream();

    linkProbeInit(&b.probe, 16, 1);
    linkProbeSyncResult(&b.probe, true, 1000);
    expectBool("ⓚ' 60초 경계 안", linkProbeSyncUsable(&b.probe, 1000 + 60000), true);
    expectBool("ⓚ' 60초 경계 밖", linkProbeSyncUsable(&b.probe, 1000 + 60001), false);
    expectU32("ⓚ' 나이 계산", linkProbeSyncAgeMs(&b.probe, 1000 + 12345), 12345);
}

// ═══════════════════════════════════════════════════════════════════════════
// 🔴 재귀 금지 — pulse publish 실패는 PUBLISH_FAIL 사건을 만들지 않는다.
//    실패가 사건을 낳고 그 사건 발행이 또 실패하면 무한 재귀다.
//    이 검사는 헤더가 막는지 본다 — 호출자 규율에 맡기면 다음 세션에 깨진다.
// ═══════════════════════════════════════════════════════════════════════════
static void l_pulse_no_recursion()
{
    std::printf("\nⓛ pulse 실패는 사건을 만들지 않는다\n");
    Board b;
    bootBoard(b, "l_pulse_no_recursion");
    b.closeStream();

    linkProbeInit(&b.probe, 16, 1);
    for (int i = 0; i < 50; ++i) {
        linkProbePublishResult(&b.probe, RUNTIME_PUBLISH_PULSE, false,
                               RUNTIME_PHASE_LOOP, (uint64_t)i * 1000ULL, 1000 + i);
    }
    expectU32("ⓛ pulse 실패 50회에도 사건 0", b.probe.evtSeq, 0);
    expectU32("ⓛ pulse_fail 누계 50", b.probe.pulseFail, 50);
    expectU32("ⓛ ring 은 비어 있다", linkProbePending(&b.probe), 0);

    // 다른 slot 은 정상적으로 사건이 된다 — 막은 것은 pulse 뿐이다.
    linkProbePublishResult(&b.probe, RUNTIME_PUBLISH_ODOM, false, RUNTIME_PHASE_ODOM,
                           100000, 1100);
    expectU32("ⓛ odom 실패는 사건이 된다", b.probe.evtSeq, 1);

    // 범위 밖 slot 은 조용히 무시한다 (배열 밖 쓰기 금지).
    linkProbePublishResult(&b.probe, RUNTIME_PUBLISH_COUNT, false, RUNTIME_PHASE_ODOM,
                           200000, 1200);
    expectU32("ⓛ 범위 밖 slot 은 사건을 만들지 않는다", b.probe.evtSeq, 1);
}

// ═══════════════════════════════════════════════════════════════════════════
// 🔴 pulse 가 안 나가면 delta 를 털지 않는다 (fail-closed).
//    만들 때 털면 그 pulse 가 유실됐을 때 판정 불능이어야 할 구간이 유효로 보인다.
// ═══════════════════════════════════════════════════════════════════════════
static void m_delta_fail_closed()
{
    std::printf("\nⓜ pulse 실패 시 delta 보존\n");
    Board b;
    bootBoard(b, "m_delta_fail_closed");
    b.closeStream();

    linkProbeInit(&b.probe, 1, 1);
    linkProbeSyncResult(&b.probe, true, 0);

    // capacity 1 을 넘겨 delta 를 만든다.
    linkProbeRecordEvent(&b.probe, LINK_EVENT_PUBLISH_FAIL, RUNTIME_PUBLISH_ODOM,
                         RUNTIME_PHASE_ODOM, 0, 0, 0, 1000);
    linkProbeRecordEvent(&b.probe, LINK_EVENT_PUBLISH_FAIL, RUNTIME_PUBLISH_IMU,
                         RUNTIME_PHASE_IMU, 0, 0, 1000, 1001);
    expectU32("ⓜ delta 1", b.probe.evtDroppedDelta, 1);

    LinkPulse p;
    linkProbeBuildPulse(&b.probe, 100, 1100, &p);
    expectU32("ⓜ pulse 가 delta 를 싣는다", p.evtDroppedDelta, 1);
    // 스케치와 같은 순서: publishMeasured(→publishResult) 다음에 pulseSent.
    linkProbePublishResult(&b.probe, RUNTIME_PUBLISH_PULSE, false, RUNTIME_PHASE_LOOP,
                           5000, 1100);
    linkProbePulseSent(&b.probe, false);          // ← 발행 실패
    expectU32("ⓜ 🔴 실패했으므로 delta 는 남는다", b.probe.evtDroppedDelta, 1);
    expectU32("ⓜ pulse_fail 은 한 곳에서만 센다", b.probe.pulseFail, 1);

    linkProbeBuildPulse(&b.probe, 200, 1200, &p);
    linkProbePulseSent(&b.probe, true);           // ← 이번엔 성공
    expectU32("ⓜ 성공하면 delta 를 턴다", b.probe.evtDroppedDelta, 0);

    // 만든 뒤 보낸 사이에 늘어난 몫은 잃지 않는다 (빼기로 털기 때문).
    linkProbeRecordEvent(&b.probe, LINK_EVENT_PUBLISH_FAIL, RUNTIME_PUBLISH_ODOM,
                         RUNTIME_PHASE_ODOM, 0, 0, 2000, 1300);
    linkProbeRecordEvent(&b.probe, LINK_EVENT_PUBLISH_FAIL, RUNTIME_PUBLISH_IMU,
                         RUNTIME_PHASE_IMU, 0, 0, 3000, 1301);
    linkProbeBuildPulse(&b.probe, 300, 1300, &p);
    linkProbeRecordEvent(&b.probe, LINK_EVENT_PUBLISH_FAIL, RUNTIME_PUBLISH_ESTOP,
                         RUNTIME_PHASE_LOOP, 0, 0, 4000, 1302);
    const uint32_t deltaBefore = b.probe.evtDroppedDelta;
    linkProbePulseSent(&b.probe, true);
    expectU32("ⓜ 표본 사이에 늘어난 몫은 남는다",
              b.probe.evtDroppedDelta, deltaBefore - p.evtDroppedDelta);
}

// ═══════════════════════════════════════════════════════════════════════════
// 🔴 catch-up 금지 — loop 이 300ms 섰다가 깨어나도 sample_seq 는 +1 이다.
//    몰아서 올리면 "loop 가 섰다" 와 "host 가 잃었다" 가 같은 값이 되어
//    분류표 8행이 무너진다.
// ═══════════════════════════════════════════════════════════════════════════
static void n_no_catchup()
{
    std::printf("\nⓝ pulse tick 은 밀린 만큼 몰아 올리지 않는다\n");
    Board b;
    bootBoard(b, "n_no_catchup");
    b.closeStream();

    linkProbeInit(&b.probe, 16, 1);
    linkProbeSyncResult(&b.probe, true, 0);

    LinkPulse p;
    linkProbeBuildPulse(&b.probe, 0, 1000, &p);
    linkProbePulseSent(&b.probe, true);
    expectU32("ⓝ 첫 표본 seq 1", b.probe.sampleSeq, 1);

    expectBool("ⓝ 99ms 에는 아직 아니다", linkProbePulseDue(&b.probe, 99), false);
    expectBool("ⓝ 100ms 경계에서 due", linkProbePulseDue(&b.probe, 100), true);

    // 300ms 정지 뒤 깨어남 — 한 판이므로 tick 도 한 번이다.
    linkProbeBuildPulse(&b.probe, 400, 1400, &p);
    linkProbePulseSent(&b.probe, true);
    expectU32("ⓝ 🔴 300ms 를 건너뛰어도 +1", b.probe.sampleSeq, 2);

    // 정상 주행이면 100ms 마다 한 번씩 올라간다.
    for (uint32_t t = 500; t <= 700; t += 100) {
        if (linkProbePulseDue(&b.probe, t)) {
            linkProbeBuildPulse(&b.probe, t, 1000 + t, &p);
            linkProbePulseSent(&b.probe, true);
        }
    }
    expectU32("ⓝ 300ms 정상 주행이면 +3", b.probe.sampleSeq, 5);
}

// ═══════════════════════════════════════════════════════════════════════════
// 🔴 전문 크기 상한 — 계약이 "pulse ≤128B" 라고 못박았다. 최악값으로 잰다.
//    손으로 센 길이는 §65.5 에서 이미 한 번 틀렸다(845 라고 적고 실제 1407).
//    그래서 여기서는 세지 않고 **만들어 본다**.
// ═══════════════════════════════════════════════════════════════════════════
static void p_text_bounds()
{
    std::printf("\nⓟ 전문 크기 상한\n");
    LinkPulse worst{};
    worst.bootId = 0xFFFFFFFFUL;
    worst.sampleSeq = 0xFFFFFFFFUL;
    worst.epochMs = 4102444800000ULL;   // 2100-01-01, epoch_ms 13자리
    worst.syncOk = false;
    worst.syncAgeMs = 0xFFFFFFFFUL;
    worst.evtSeq = 0xFFFFFFFFUL;
    worst.evtDroppedTotal = 0xFFFFFFFFUL;
    worst.evtDroppedDelta = 0xFFFFFFFFUL;
    worst.pulseFail = 0xFFFFFFFFUL;

    char buf[512];
    const int n = linkPulseFormat(&worst, buf, (int)sizeof(buf));
    std::printf("  최악 pulse 전문 %dB: %s\n", n, buf);
    ++g_checks;
    if (n < 0 || n >= LINK_PULSE_TEXT_MAX) {
        std::printf("  FAIL 🔴 최악 pulse 전문 %dB 가 계약 상한 %dB 를 넘는다\n", n,
                    LINK_PULSE_TEXT_MAX);
        ++g_failures;
    }

    LinkEvent we{};
    we.firstEpochMs = 4102444800000ULL;
    we.lastEpochMs = 4102444800000ULL;
    we.execUsMax = 0xFFFFFFFFUL;
    we.idleUsMax = 0xFFFFFFFFUL;
    we.burstId = 0xFFFFFFFFUL;
    we.count = 0xFFFFFFFFUL;
    we.phase = 255;
    we.publishSlot = 255;
    we.code = 255;
    const int m = linkEventFormat(&we, buf, (int)sizeof(buf));
    std::printf("  최악 event 전문 %dB: %s\n", m, buf);
    ++g_checks;
    if (m < 0 || m >= LINK_EVENT_TEXT_MAX) {
        std::printf("  FAIL 최악 event 전문 %dB 가 상한 %dB 를 넘는다\n", m,
                    LINK_EVENT_TEXT_MAX);
        ++g_failures;
    }

    // 되파싱까지 왕복되는가
    LinkPulse rt{};
    (void)linkPulseFormat(&worst, buf, (int)sizeof(buf));
    expectBool("ⓟ 최악 전문도 되파싱된다", parsePulse(buf, rt), true);
    expectU32("ⓟ 왕복 sample_seq", rt.sampleSeq, worst.sampleSeq);
    expectU64("ⓟ 왕복 epoch_ms", rt.epochMs, worst.epochMs);
    expectBool("ⓟ 왕복 sync_ok", rt.syncOk, worst.syncOk);
}

// ═══════════════════════════════════════════════════════════════════════════
// 🔴 micros() 뒤집힘 — 71.6분마다 0 으로 돌아가는 32비트 시계가 가짜 사건을
//    만들지 않는가. 이건 주입 목록 ⓐ~ⓙ 에 없지만, 없으면 12분 시행 다음에
//    나오는 긴 시행에서 계측이 스스로 무너진다.
// ═══════════════════════════════════════════════════════════════════════════
static void o_clock_wrap()
{
    std::printf("\nⓞ micros() 뒤집힘\n");
    LinkClock c;
    linkClockInit(&c);

    expectU64("ⓞ 첫 표본", linkClockExtend(&c, 1000), 1000);
    expectU64("ⓞ 단조 증가", linkClockExtend(&c, 2000), 2000);
    expectU64("ⓞ 상한 직전", linkClockExtend(&c, 0xFFFFFF00UL), 0xFFFFFF00ULL);
    // 뒤집힘: raw 가 줄었다 → 한 바퀴를 더한다.
    expectU64("ⓞ 🔴 뒤집힘 뒤에도 단조", linkClockExtend(&c, 0x100),
              0x100000000ULL + 0x100ULL);
    expectU64("ⓞ 두 바퀴째", linkClockExtend(&c, 0xFFFFFF00UL),
              0x100000000ULL + 0xFFFFFF00ULL);
    expectU64("ⓞ 세 바퀴째", linkClockExtend(&c, 0x50),
              0x200000000ULL + 0x50ULL);

    // 확장기를 쓰면 뒤집힘이 사건을 만들지 않는다.
    LinkStallProbe p;
    linkProbeInit(&p, 16, 1);
    LinkClock c2;
    linkClockInit(&c2);
    linkProbeLoopBegin(&p, linkClockExtend(&c2, 0xFFFFFF00UL), 1000);
    linkProbeLoopEnd(&p, linkClockExtend(&c2, 0xFFFFFFC0UL), 1000);
    linkProbeLoopBegin(&p, linkClockExtend(&c2, 0x40), 1001);
    expectU32("ⓞ 🔴 뒤집힘이 가짜 BETWEEN_LOOPS 를 만들지 않는다", p.evtSeq, 0);

    // 확장기 없이 raw 를 그대로 넣으면 어떻게 되는지 — 이 검사가 지키는 것을 보인다.
    LinkStallProbe bad;
    linkProbeInit(&bad, 16, 1);
    linkProbeLoopBegin(&bad, 0xFFFFFF00ULL, 1000);
    linkProbeLoopEnd(&bad, 0xFFFFFFC0ULL, 1000);
    linkProbeLoopBegin(&bad, 0x40ULL, 1001);
    expectU32("ⓞ (대조) raw 를 그대로 넣으면 가짜 사건이 난다", bad.evtSeq, 1);
}

// ═══════════════════════════════════════════════════════════════════════════
// ⓧ loop 안 + 발행 실패 → **복합**.
//
// 🔴 이 주입이 08-18 에 계약을 한 번 고쳤다. 구판 6행은 `LOOP_INTERNAL +
//    BETWEEN_LOOPS` 만 복합으로 봐서 이 조합이 9행(판정 불능)으로 떨어졌다.
//    08-17 의 7사건은 `publish_failures +70` 이 300ms 공백과 겹친 모양이라,
//    그대로 뒀으면 계측이 다 옳게 기록해도 **표가 답을 못 내는** 상태였다.
//    개정 뒤: 단독 3행을 먼저 보고 2종 이상이면 전부 복합 (사용자 결정).
//    ⚠ 이 주입을 지우지 마라 — 구멍이 되돌아오면 여기가 먼저 깨진다.
// ═══════════════════════════════════════════════════════════════════════════
static void x_compound_with_publish()
{
    std::printf("\nⓧ loop 안 + 발행 실패 → 복합\n");
    Board b;
    bootBoard(b, "x_compound_with_publish");

    runQuiet(b, 20);
    const uint64_t gapStart = b.epochMs();

    // loop 이 300ms 서고, 그 앞뒤로 odom publish 가 실패한다 — 08-17 에 실제로
    // 같이 나올 법한 조합이다 (publish_failures +70 이 300ms 공백과 겹쳤다).
    runLoop(b, 300000, {{RUNTIME_PUBLISH_ODOM, false}});
    runIdle(b, 20000);
    runLoop(b, 2000, {{RUNTIME_PUBLISH_ODOM, false}});
    const uint64_t gapEnd = b.epochMs();

    runQuiet(b, 20);

    expectU32("ⓧ LOOP_INTERNAL 1", countCode(b, LINK_EVENT_LOOP_INTERNAL), 1);
    expectU32("ⓧ PUBLISH_FAIL 2", countCode(b, LINK_EVENT_PUBLISH_FAIL), 2);
    expectU32("ⓧ BETWEEN_LOOPS 0", countCode(b, LINK_EVENT_BETWEEN_LOOPS), 0);

    b.servicePulseAndDrain();
    runQuiet(b, 5);
    b.closeStream();
    writeMeta("x_compound_with_publish", gapStart, gapEnd, "COMPOUND", -1);
}

// ═══════════════════════════════════════════════════════════════════════════
// ⓨ 공백 중 TIME_SYNC 가 offset 을 옮긴다 → 🔴 **판정 불능(9행)**.
//
// 왜 이 주입이 필요한가: 08-18 개정으로 code 조합 8가지가 전부 위에서 덮이므로,
// 9행이 **도달 불가능한 죽은 줄**이 될 수 있다. 죽은 줄은 계약의 거짓말이다.
// 실제 남은 도달 경로가 이것이다 — 사건은 0건인데 epoch 경과와 sample_seq 증가량이
// 안 맞는 구간. 동기가 offset 을 옮기면 벽시계는 뛰고 tick 수는 안 뛴다.
//
// 🔴 이때 "사건이 0건이니 host 이후겠지" 로 반올림하지 않는 것이 요점이다.
//    그 반올림이 §79.3 이 지적한 "복합이 단일로 숨는" 것과 같은 종류의 잘못이다.
// ═══════════════════════════════════════════════════════════════════════════
static void y_clock_shift_undecidable()
{
    std::printf("\nⓨ 공백 중 동기 offset 이동 → 판정 불능\n");
    Board b;
    bootBoard(b, "y_clock_shift");

    runQuiet(b, 30);
    const uint64_t gapStart = b.epochMs();

    // 실제로는 약 200ms 만 흘렀는데 동기가 벽시계를 2초 앞으로 옮겼다.
    runQuiet(b, 9);
    b.epochBaseMs += 2000;
    b.syncOk();
    // 공백 창은 이동을 **가로질러야** 한다 — 이동 전 표본과 이동 후 표본을 각각
    // 기준·복귀로 잡아야 벽시계 경과와 tick 수의 어긋남이 보인다.
    const uint64_t gapEnd = b.epochMs();
    runQuiet(b, 30);

    expectU32("ⓨ 사건 0건", b.probe.evtSeq, 0);
    b.closeStream();
    writeMeta("y_clock_shift", gapStart, gapEnd, "UNDECIDABLE_UNCOVERED", -1);
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::printf("사용: link_stall_host_test <출력 디렉터리>\n");
        return 2;
    }
    g_outDir = argv[1];

    std::printf("=== 예약 41-g 사건 계측 주입 harness (계약 3판) ===\n");
    std::printf("가짜 시계로 주입한다 — 300ms 를 실제로 기다리지 않는다.\n");

    a_loop_internal();
    b_between_loops();
    c_host_after();
    d_run_ended();
    e_mcu_reset();
    f_publish_layer();
    g_compound();
    h_seven_bursts();
    i_slot_split();
    j_overflow(16, 17, "j_overflow_16");
    j_overflow(8, 9, "j_overflow_8");
    k_undecidable_sync();
    k_sync_age();
    l_pulse_no_recursion();
    m_delta_fail_closed();
    n_no_catchup();
    o_clock_wrap();
    p_text_bounds();
    x_compound_with_publish();
    y_clock_shift_undecidable();

    std::printf("\nMCU 쪽 검사 %d건 · 실패 %d건\n", g_checks, g_failures);
    if (g_failures != 0) {
        std::printf("FAIL 계측이 계약대로 갈리지 않는다.\n");
        return 1;
    }
    std::printf("OK   MCU 쪽 불변식 전량 통과.\n");
    std::printf("     ⚠ 분류가 실제로 갈리는지는 link_stall_classify.py 가 본다.\n");
    return 0;
}
