// ============================================================================
// rearm_gate_host_test.cpp — re-arm 래치 상태 전이 결정론 harness (PC 전용)
//
// 실행:  bash tools/rearm_gate_host_test.sh
// 대상:  firmware/teensy_integrated_base_v1_4/rearm_gate.h  (그 파일을 직접 include)
//
// 왜 이 파일이 있나 (검토 §54.7):
//   "첫 diff 에서 상태 전이 클래스 전체를 닫는다 … 하나의 상태 전이표와 host-side
//    결정론 harness 로 고정한다."
//   보드 없이·대기 없이 전이 경계를 재현한다. 시각을 인자로 주므로 500ms 경계의
//   앞뒤 1ms 를 정확히 찍을 수 있다 — 실기 시험으로는 못 하는 일이다.
//
// 🔴 이 harness 가 증명하지 않는 것 (숨기지 않는다):
//   - 스케치의 **배선**은 여기서 안 본다. rearm_gate.h 의 전이만 본다.
//     stopAllMotors() 호출 여부·publish 내용은 .ino 쪽이고, 그것은 실기
//     JETSON_SETUP §7-c-E 가 관측한다.
//   - millis() 32bit 랩어라운드는 부호 없는 뺄셈이라 원리적으로 안전하지만,
//     그 성질을 여기서 별도 케이스로 확인한다 (T21).
// ============================================================================

#include "../firmware/teensy_integrated_base_v1_4/rearm_gate.h"

#include <cstdio>
#include <cstring>
#include <string>

static int g_failures = 0;
static int g_checks = 0;

static const char* stateName(uint8_t s)
{
    switch (s) {
        case DRIVE_DISARMED: return "DISARMED";
        case DRIVE_READY:    return "READY";
        case DRIVE_ARMED:    return "ARMED";
        case DRIVE_PENDING:  return "PENDING";
        default:             return "???";
    }
}

static void expectState(const char* what, const RearmGate& g, uint8_t want)
{
    ++g_checks;
    if (g.state != want) {
        std::printf("  FAIL %-58s state=%s (기대 %s)\n",
                    what, stateName(g.state), stateName(want));
        ++g_failures;
    }
}

static void expectEffect(const char* what, DriveEffect got, DriveEffect want)
{
    ++g_checks;
    if (got != want) {
        std::printf("  FAIL %-58s effect=%d (기대 %d)\n", what, (int)got, (int)want);
        ++g_failures;
    }
}

static void expectBool(const char* what, bool got, bool want)
{
    ++g_checks;
    if (got != want) {
        std::printf("  FAIL %-58s = %s (기대 %s)\n",
                    what, got ? "true" : "false", want ? "true" : "false");
        ++g_failures;
    }
}

static void expectU32(const char* what, uint32_t got, uint32_t want)
{
    ++g_checks;
    if (got != want) {
        std::printf("  FAIL %-58s = %u (기대 %u)\n", what, got, want);
        ++g_failures;
    }
}

// 관용 조작 — zero 를 dt 간격으로 흘려 hold 를 채운다.
static void feedZeros(RearmGate& g, uint32_t startMs, uint32_t untilMs, uint32_t stepMs)
{
    for (uint32_t t = startMs; t <= untilMs; t += stepMs) {
        rearmGateOnCommand(&g, 0.0, 0.0, t);
    }
}

// DISARMED → READY → PENDING → ARMED 전 과정. 반환 = ARMED 가 된 시각.
static uint32_t armFully(RearmGate& g, uint32_t t0)
{
    feedZeros(g, t0, t0 + REARM_ZERO_HOLD_MS, 50);
    rearmGateOnService(&g, true, false, t0 + REARM_ZERO_HOLD_MS);
    const uint32_t armedAt = t0 + REARM_ZERO_HOLD_MS + REARM_POST_ARM_QUIET_MS;
    rearmGateTick(&g, armedAt);
    return armedAt;
}

// ── T01 부팅 직후는 DISARMED 이고 명령은 안 먹는다 ───────────────────────────
static void t01_boot_is_disarmed()
{
    RearmGate g;
    rearmGateInit(&g);
    expectState("T01 부팅 상태", g, DRIVE_DISARMED);
    expectEffect("T01 부팅 직후 비영 명령",
                 rearmGateOnCommand(&g, 0.05, 0.0, 10), DRIVE_EFFECT_HOLD);
    expectState("T01 비영 뒤에도", g, DRIVE_DISARMED);
    expectU32("T01 서비스 호출 누계", g.serviceCalls, 0);
}

// ── T02 zero 를 0.5초 채우면 READY. 그 전엔 아니다 ──────────────────────────
static void t02_zero_hold_boundary()
{
    RearmGate g;
    rearmGateInit(&g);
    rearmGateOnCommand(&g, 0.0, 0.0, 1000);          // 타이머 시작
    rearmGateOnCommand(&g, 0.0, 0.0, 1499);          // 499ms — 아직
    expectState("T02 499ms", g, DRIVE_DISARMED);
    rearmGateOnCommand(&g, 0.0, 0.0, 1500);          // 정확히 500ms
    expectState("T02 500ms 경계", g, DRIVE_READY);
}

// ── T03 멈춘 적 없는 발행자는 절대 READY 가 못 된다 ─────────────────────────
static void t03_never_stopping_publisher()
{
    RearmGate g;
    rearmGateInit(&g);
    for (uint32_t t = 0; t <= 5000; t += 20) {
        expectEffect("T03 계속 비영",
                     rearmGateOnCommand(&g, 0.1, 0.0, t), DRIVE_EFFECT_HOLD);
    }
    expectState("T03 5초 뒤에도", g, DRIVE_DISARMED);
    expectBool("T03 서비스 거절", rearmGateOnService(&g, true, false, 5000), false);
    expectU32("T03 사유 = ZERO_HOLD", g.rejectReason, REARM_REJECT_ZERO_HOLD);
}

// ── T04 hold 중 비영 하나가 타이머를 처음부터 다시 돌린다 ──────────────────
static void t04_nonzero_breaks_hold()
{
    RearmGate g;
    rearmGateInit(&g);
    rearmGateOnCommand(&g, 0.0, 0.0, 0);
    rearmGateOnCommand(&g, 0.0, 0.0, 400);
    rearmGateOnCommand(&g, 0.05, 0.0, 450);          // 파괴
    expectState("T04 파괴 직후", g, DRIVE_DISARMED);
    expectBool("T04 hold 플래그 해제", g.zeroHolding, false);
    rearmGateOnCommand(&g, 0.0, 0.0, 500);           // 새 타이머
    rearmGateOnCommand(&g, 0.0, 0.0, 999);
    expectState("T04 새 타이머 499ms", g, DRIVE_DISARMED);
    rearmGateOnCommand(&g, 0.0, 0.0, 1000);
    expectState("T04 새 타이머 500ms", g, DRIVE_READY);
}

// ── T05 §54.3 P1 — NaN 이 hold 를 끊는다 (구판은 안 끊었다) ────────────────
static void t05_nan_breaks_hold()
{
    const double kNan = NAN;
    const double kInf = INFINITY;
    const double kNegInf = -INFINITY;
    const double bad[3] = {kNan, kInf, kNegInf};
    const char* names[3] = {"NaN", "+Inf", "-Inf"};

    for (int axis = 0; axis < 2; ++axis) {
        for (int i = 0; i < 3; ++i) {
            RearmGate g;
            rearmGateInit(&g);
            rearmGateOnCommand(&g, 0.0, 0.0, 0);       // 타이머 시작
            const double lin = (axis == 0) ? bad[i] : 0.0;
            const double ang = (axis == 0) ? 0.0 : bad[i];
            expectEffect("T05 비유한 입력은 HOLD",
                         rearmGateOnCommand(&g, lin, ang, 250), DRIVE_EFFECT_HOLD);

            char label[96];
            std::snprintf(label, sizeof(label), "T05 %s(%s) 뒤 500ms 의 zero",
                          names[i], axis == 0 ? "linear.x" : "angular.z");
            rearmGateOnCommand(&g, 0.0, 0.0, 500);
            // 🔴 구판은 여기서 READY 였다. 이제는 250ms 에 타이머가 지워졌으므로
            //    500ms 의 zero 는 **새 타이머의 시작**일 뿐이다.
            expectState(label, g, DRIVE_DISARMED);

            expectBool("T05 그 시점 서비스는 거절",
                       rearmGateOnService(&g, true, false, 500), false);
            expectU32("T05 사유 = ZERO_HOLD", g.rejectReason, REARM_REJECT_ZERO_HOLD);
        }
    }
}

// ── T06 비유한 입력 뒤 새 zero 500ms 를 완주하면 정상적으로 READY ───────────
static void t06_recovery_after_nan()
{
    RearmGate g;
    rearmGateInit(&g);
    rearmGateOnCommand(&g, 0.0, 0.0, 0);
    rearmGateOnCommand(&g, NAN, 0.0, 250);
    feedZeros(g, 500, 1000, 50);
    expectState("T06 비유한 뒤 새 500ms", g, DRIVE_READY);
}

// ── T07 §54.3 결정 ⓐ — ARMED 중 비유한 입력은 재무장을 요구한다 ────────────
static void t07_nan_while_armed_disarms()
{
    RearmGate g;
    rearmGateInit(&g);
    const uint32_t armedAt = armFully(g, 0);
    expectState("T07 무장 완료", g, DRIVE_ARMED);
    expectEffect("T07 ARMED 에서 비유한 입력",
                 rearmGateOnCommand(&g, 0.05, NAN, armedAt + 10), DRIVE_EFFECT_HOLD);
    expectState("T07 → DISARMED (사용자 결정 ⓐ)", g, DRIVE_DISARMED);
    expectBool("T07 즉시 재무장 시도는 거절",
               rearmGateOnService(&g, true, false, armedAt + 20), false);
    expectU32("T07 사유 = ZERO_HOLD", g.rejectReason, REARM_REJECT_ZERO_HOLD);
}

// ── T08 §54.2 P1 — 서비스 성공이 곧바로 ARMED 가 아니다 ────────────────────
static void t08_service_does_not_arm_directly()
{
    RearmGate g;
    rearmGateInit(&g);
    feedZeros(g, 0, 500, 50);
    expectState("T08 서비스 직전", g, DRIVE_READY);
    expectBool("T08 서비스 성공", rearmGateOnService(&g, true, false, 500), true);
    // 🔴 구판은 여기서 이미 ARMED 였고, 그래서 응답 전에 큐에 있던 명령이
    //    다음 spin 에서 그대로 구동에 들어갔다.
    expectState("T08 성공 응답 시점", g, DRIVE_PENDING);
    expectU32("T08 사유 = OK", g.rejectReason, REARM_OK);
}

// ── T09 §54.2 부정회귀 ① — take 스냅샷 뒤·응답 전 비영 도착 ────────────────
// ② 서비스 콜백 중 도착, ③ 응답 직전 큐 적재 — 셋 다 "PENDING 중 도착하는
// 비영 명령"으로 귀결된다. 도착 시각을 장벽 안 전 구간에 걸쳐 훑는다.
static void t09_pre_response_command_cannot_drive()
{
    for (uint32_t delay = 0; delay < REARM_POST_ARM_QUIET_MS; delay += 25) {
        RearmGate g;
        rearmGateInit(&g);
        feedZeros(g, 0, 500, 50);
        rearmGateOnService(&g, true, false, 500);

        char label[96];
        std::snprintf(label, sizeof(label), "T09 장벽 +%ums 잔류 비영 명령", delay);
        expectEffect(label,
                     rearmGateOnCommand(&g, 0.05, 0.0, 500 + delay),
                     DRIVE_EFFECT_HOLD);
        expectState("T09 → DISARMED (fail-closed)", g, DRIVE_DISARMED);

        // 장벽이 깨졌으므로 tick 이 와도 절대 ARMED 가 되면 안 된다.
        rearmGateTick(&g, 500 + REARM_POST_ARM_QUIET_MS + 1000);
        expectState("T09 tick 뒤에도 DISARMED", g, DRIVE_DISARMED);
    }
}

// ── T10 장벽 중의 zero 는 무해하다 (장벽을 늦추지도 앞당기지도 않는다) ─────
static void t10_zero_during_barrier_is_harmless()
{
    RearmGate g;
    rearmGateInit(&g);
    feedZeros(g, 0, 500, 50);
    rearmGateOnService(&g, true, false, 500);
    feedZeros(g, 510, 990, 20);                      // 장벽 내내 zero 발행
    rearmGateTick(&g, 999);
    expectState("T10 장벽 999ms", g, DRIVE_PENDING);
    rearmGateTick(&g, 1000);
    expectState("T10 장벽 1000ms(=500+500)", g, DRIVE_ARMED);
}

// ── T11 발행이 완전히 끊겨도 장벽은 완주한다 ───────────────────────────────
static void t11_silence_completes_barrier()
{
    RearmGate g;
    rearmGateInit(&g);
    feedZeros(g, 0, 500, 50);
    rearmGateOnService(&g, true, false, 500);
    // cmd_vel 이 한 건도 안 온다 → tick 만으로 승격돼야 한다.
    rearmGateTick(&g, 999);
    expectState("T11 침묵 999ms", g, DRIVE_PENDING);
    rearmGateTick(&g, 1000);
    expectState("T11 침묵 1000ms", g, DRIVE_ARMED);
}

// ── T12 응답 뒤 새 비영 명령은 정상 주행한다 (역회귀) ──────────────────────
static void t12_post_response_command_drives()
{
    RearmGate g;
    rearmGateInit(&g);
    const uint32_t armedAt = armFully(g, 0);
    expectEffect("T12 무장 뒤 첫 비영 명령",
                 rearmGateOnCommand(&g, 0.05, 0.0, armedAt + 1), DRIVE_EFFECT_DRIVE);
    expectEffect("T12 무장 뒤 zero 명령도 적용",
                 rearmGateOnCommand(&g, 0.0, 0.0, armedAt + 2), DRIVE_EFFECT_DRIVE);
    expectState("T12 zero 를 줘도 ARMED 유지", g, DRIVE_ARMED);
}

// ── T13 §54.1 — disable 은 어느 상태에서든 DISARMED 로 간다 ────────────────
static void t13_disable_from_every_state()
{
    const uint8_t targets[3] = {DRIVE_READY, DRIVE_PENDING, DRIVE_ARMED};
    for (int i = 0; i < 3; ++i) {
        RearmGate g;
        rearmGateInit(&g);
        feedZeros(g, 0, 500, 50);
        if (targets[i] != DRIVE_READY) {
            rearmGateOnService(&g, true, false, 500);
        }
        if (targets[i] == DRIVE_ARMED) {
            rearmGateTick(&g, 1000);
        }
        expectState("T13 사전 상태", g, targets[i]);

        expectBool("T13 disable 은 항상 성공",
                   rearmGateOnService(&g, false, false, 1100), true);
        expectState("T13 disable 뒤", g, DRIVE_DISARMED);
        expectU32("T13 사유 = OK", g.rejectReason, REARM_OK);
        expectBool("T13 hold 플래그도 해제", g.zeroHolding, false);
    }
}

// ── T14 disable 은 E-stop 이 눌려 있어도 성공한다 ──────────────────────────
static void t14_disable_succeeds_under_estop()
{
    RearmGate g;
    rearmGateInit(&g);
    const uint32_t armedAt = armFully(g, 0);
    expectBool("T14 E-stop 중 disable", rearmGateOnService(&g, false, true, armedAt + 5), true);
    expectState("T14 → DISARMED", g, DRIVE_DISARMED);
}

// ── T15 E-stop 중 enable 은 거절 + DISARMED ────────────────────────────────
static void t15_enable_under_estop()
{
    RearmGate g;
    rearmGateInit(&g);
    feedZeros(g, 0, 500, 50);
    expectState("T15 서비스 직전", g, DRIVE_READY);
    expectBool("T15 E-stop 중 enable", rearmGateOnService(&g, true, true, 500), false);
    expectU32("T15 사유 = ESTOP", g.rejectReason, REARM_REJECT_ESTOP);
    expectState("T15 READY 도 잃는다", g, DRIVE_DISARMED);
}

// ── T16 중복 enable 은 ALREADY 로 거절하고 상태를 안 흔든다 ────────────────
static void t16_duplicate_enable()
{
    RearmGate g;
    rearmGateInit(&g);
    feedZeros(g, 0, 500, 50);
    rearmGateOnService(&g, true, false, 500);
    expectBool("T16 PENDING 중 재호출", rearmGateOnService(&g, true, false, 600), false);
    expectU32("T16 사유 = ALREADY", g.rejectReason, REARM_REJECT_ALREADY);
    expectState("T16 PENDING 유지", g, DRIVE_PENDING);
    // 🔴 거절이 장벽을 리셋하면 안 된다 — 원래 시각 기준으로 승격돼야 한다.
    rearmGateTick(&g, 1000);
    expectState("T16 원래 시각 기준 승격", g, DRIVE_ARMED);

    expectBool("T16 ARMED 중 재호출", rearmGateOnService(&g, true, false, 1100), false);
    expectU32("T16 사유 = ALREADY", g.rejectReason, REARM_REJECT_ALREADY);
    expectState("T16 ARMED 유지", g, DRIVE_ARMED);
}

// ── T17 서비스 호출 누계는 거절까지 전부 센다 ──────────────────────────────
static void t17_service_counter_counts_rejections()
{
    RearmGate g;
    rearmGateInit(&g);
    rearmGateOnService(&g, true, false, 10);         // 1 거절 ZERO_HOLD
    rearmGateOnService(&g, true, true, 20);          // 2 거절 ESTOP
    rearmGateOnService(&g, false, false, 30);        // 3 성공 disable
    feedZeros(g, 100, 600, 50);
    rearmGateOnService(&g, true, false, 600);        // 4 성공 → PENDING
    rearmGateOnService(&g, true, false, 610);        // 5 거절 ALREADY
    expectU32("T17 누계", g.serviceCalls, 5);
}

// ── T18 READY 유지 — hold 를 넘겨 계속 zero 를 줘도 READY 에 머문다 ────────
static void t18_ready_is_stable()
{
    RearmGate g;
    rearmGateInit(&g);
    feedZeros(g, 0, 5000, 50);
    expectState("T18 zero 5초", g, DRIVE_READY);
    expectBool("T18 서비스 성공", rearmGateOnService(&g, true, false, 5000), true);
}

// ── T19 READY 에서 비영이 오면 READY 를 잃는다 ─────────────────────────────
static void t19_ready_lost_on_nonzero()
{
    RearmGate g;
    rearmGateInit(&g);
    feedZeros(g, 0, 500, 50);
    expectState("T19 READY", g, DRIVE_READY);
    rearmGateOnCommand(&g, 0.0, 0.05, 550);          // angular 만 비영
    expectState("T19 angular 비영 뒤", g, DRIVE_DISARMED);
}

// ── T20 아주 작은 비영도 비영이다 (deadband 는 여기 관심사가 아니다) ───────
static void t20_tiny_nonzero_is_nonzero()
{
    RearmGate g;
    rearmGateInit(&g);
    rearmGateOnCommand(&g, 0.0, 0.0, 0);
    rearmGateOnCommand(&g, 1e-12, 0.0, 100);
    expectState("T20 1e-12 도 파괴한다", g, DRIVE_DISARMED);
    expectBool("T20 hold 해제", g.zeroHolding, false);
}

// ── T21 millis() 32bit 랩어라운드를 넘어가도 경계가 같다 ───────────────────
static void t21_millis_wraparound()
{
    const uint32_t nearMax = 0xFFFFFF00u;            // 랩까지 256ms
    RearmGate g;
    rearmGateInit(&g);
    rearmGateOnCommand(&g, 0.0, 0.0, nearMax);
    rearmGateOnCommand(&g, 0.0, 0.0, (uint32_t)(nearMax + 499));
    expectState("T21 랩 넘어 499ms", g, DRIVE_DISARMED);
    rearmGateOnCommand(&g, 0.0, 0.0, (uint32_t)(nearMax + 500));
    expectState("T21 랩 넘어 500ms", g, DRIVE_READY);

    rearmGateOnService(&g, true, false, (uint32_t)(nearMax + 500));
    rearmGateTick(&g, (uint32_t)(nearMax + 999));
    expectState("T21 랩 넘어 장벽 499ms", g, DRIVE_PENDING);
    rearmGateTick(&g, (uint32_t)(nearMax + 1000));
    expectState("T21 랩 넘어 장벽 500ms", g, DRIVE_ARMED);
}

// ── T22 전체 왕복 시나리오 (현장 절차 그대로) ──────────────────────────────
static void t22_field_procedure()
{
    RearmGate g;
    rearmGateInit(&g);

    // ① E-stop 을 눌렀다 뗀다 → 호출자가 disarm 을 부른 상태
    rearmGateDisarm(&g);
    expectState("T22 ① E-stop 해제 직후", g, DRIVE_DISARMED);

    // ② zero 0.5초
    feedZeros(g, 1000, 1500, 100);
    expectState("T22 ② zero 0.5초", g, DRIVE_READY);

    // ③ 서비스
    expectBool("T22 ③ enable", rearmGateOnService(&g, true, false, 1500), true);
    expectState("T22 ③ 응답 시점", g, DRIVE_PENDING);

    // ④ 장벽 0.5초 (zero 를 계속 줘도 되고 안 줘도 된다)
    feedZeros(g, 1600, 1900, 100);
    rearmGateTick(&g, 2000);
    expectState("T22 ④ 장벽 완주", g, DRIVE_ARMED);

    // ⑤ 주행
    expectEffect("T22 ⑤ 주행 명령", rearmGateOnCommand(&g, 0.05, 0.0, 2050),
                 DRIVE_EFFECT_DRIVE);

    // ⑥ E-stop 재차 → 다시 처음부터
    rearmGateDisarm(&g);
    expectState("T22 ⑥ 재차 E-stop", g, DRIVE_DISARMED);
    expectEffect("T22 ⑥ 직후 주행 명령은 거부",
                 rearmGateOnCommand(&g, 0.05, 0.0, 2100), DRIVE_EFFECT_HOLD);
}

int main()
{
    std::printf("=== re-arm 래치 상태 전이 harness (검토 §54 보완) ===\n");
    std::printf("  zero-hold %ums · post-response quiet %ums\n\n",
                REARM_ZERO_HOLD_MS, REARM_POST_ARM_QUIET_MS);

    t01_boot_is_disarmed();
    t02_zero_hold_boundary();
    t03_never_stopping_publisher();
    t04_nonzero_breaks_hold();
    t05_nan_breaks_hold();
    t06_recovery_after_nan();
    t07_nan_while_armed_disarms();
    t08_service_does_not_arm_directly();
    t09_pre_response_command_cannot_drive();
    t10_zero_during_barrier_is_harmless();
    t11_silence_completes_barrier();
    t12_post_response_command_drives();
    t13_disable_from_every_state();
    t14_disable_succeeds_under_estop();
    t15_enable_under_estop();
    t16_duplicate_enable();
    t17_service_counter_counts_rejections();
    t18_ready_is_stable();
    t19_ready_lost_on_nonzero();
    t20_tiny_nonzero_is_nonzero();
    t21_millis_wraparound();
    t22_field_procedure();

    std::printf("\n검사 %d건 · 실패 %d건\n", g_checks, g_failures);
    if (g_failures != 0) {
        std::printf("FAIL 상태 전이 계약이 깨졌다 — 굽지 않는다.\n");
        return 1;
    }
    std::printf("OK   전이표 전량 통과. ⚠ 이것은 rearm_gate.h 의 증거이지\n");
    std::printf("     .ino 배선·모터 출력의 증거가 아니다 (JETSON_SETUP §7-c-E).\n");
    return 0;
}
