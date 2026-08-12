// ============================================================================
// rearm_gate_host_test.cpp — re-arm 래치 상태 전이 + 정지 배선 결정론 harness (PC 전용)
//
// 실행:  bash tools/rearm_gate_host_test.sh
// 대상:  firmware/teensy_integrated_base_v1_4/rearm_gate.h    (전이)
//        firmware/teensy_integrated_base_v1_4/drive_wiring.h  (전이 + 정지)
//
// 왜 이 파일이 있나 (검토 §54.7):
//   "첫 diff 에서 상태 전이 클래스 전체를 닫는다 … 하나의 상태 전이표와 host-side
//    결정론 harness 로 고정한다."
//   보드 없이·대기 없이 전이 경계를 재현한다. 시각을 인자로 주므로 500ms 경계의
//   앞뒤 1ms 를 정확히 찍을 수 있다 — 실기 시험으로는 못 하는 일이다.
//
// 왜 대상이 두 파일이 됐나 (검토 §55.2):
//   초판은 rearm_gate.h 만 봤다. 그래서 §54.1 의 **진짜** 구판 —
//   .ino 의 disarmDrive() 에서 stopAllMotors() 를 빼고 출력단 가드를 지우는 것 —
//   을 되돌려도 412/412 로 통과했다. 안전 출력 경로의 자동 방어가 0 이었는데
//   정본은 "세 P1 변이 전부 FAIL" 이라고 적고 있었다. 정지를 drive_wiring.h 로
//   옮기고, 여기에 가짜 모터를 붙여 그 변이가 죽게 했다.
//
// 🔴 이 harness 가 증명하지 않는 것 (숨기지 않는다):
//   - 스케치가 drive_wiring.h 의 함수들을 **부르는가**. 아래 SketchModel 은 .ino 의
//     네 호출 지점을 모사한 것이지 .ino 자신이 아니다. 그 대조는 실행 스크립트의
//     2단계 **구조 검사**(텍스트)가 하며, 동작 검사보다 약한 증거다.
//   - stopAllMotors() 가 실제로 PWM 을 0 으로 쓰는가, publish 내용이 맞는가.
//     그건 실기 JETSON_SETUP §7-c-E 가 관측한다.
//   - 응답 바이트가 **클라이언트에 도달한** 시각. 펌웨어가 관측할 수 없는 값이다
//     (§55.1 의 남은 전제 = agent→client 전송, 미실측).
// ============================================================================

#include "../firmware/teensy_integrated_base_v1_4/rearm_gate.h"
#include "../firmware/teensy_integrated_base_v1_4/drive_wiring.h"
#include "../firmware/teensy_integrated_base_v1_4/estop_debounce.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

static int g_failures = 0;
static int g_checks = 0;

static const char* stateName(uint8_t s)
{
    switch (s) {
        case DRIVE_DISARMED: return "DISARMED";
        case DRIVE_READY:    return "READY";
        case DRIVE_ARMED:    return "ARMED";
        case DRIVE_PENDING:  return "PENDING";
        case DRIVE_ARMING:   return "ARMING";
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

// ── 가짜 모터 (검토 §55.2) ───────────────────────────────────────────────────
// .ino 의 TeensyDriveSink 와 같은 자리에 꽂힌다. stopAllMotors() 가 실제로 불렸는지,
// 그 순간 수신 플래그가 내려갔는지를 host 에서 관측한다.
struct FakeDriveSink {
    int stopCalls = 0;
    int acceptCalls = 0;
    bool cmdVelReceived = false;
    uint32_t lastCmdVelMs = 0;
    // 실물 PWM 대응물. applySkidSteerCommand() 가 목표를 넣으면 true, 정지하면 false.
    bool motorsCommanded = false;

    void stopAllMotors()
    {
        ++stopCalls;
        motorsCommanded = false;
    }
    void setCmdVelReceived(bool value) { cmdVelReceived = value; }
    void noteCommandAccepted(uint32_t nowMs)
    {
        ++acceptCalls;
        lastCmdVelMs = nowMs;
        cmdVelReceived = true;
    }
};

// ── .ino 네 호출 지점의 모사 (검토 §55.2) ────────────────────────────────────
// 🔴 이것은 .ino 가 아니다. 스케치가 실제로 이 순서대로 부르는지는 실행 스크립트의
//    구조 검사가 텍스트로 본다. 여기서 얻는 것은 "이 배선이면 어떤 순서로도 모터가
//    ARMED 밖에서 못 돈다"는 성질이다.
struct SketchModel {
    RearmGate gate;
    FakeDriveSink sink;
    bool estop = false;

    SketchModel() { rearmGateInit(&gate); }

    // cmdVelCallback()
    void onCmdVel(double lin, double ang, uint32_t nowMs)
    {
        if (estop) {
            driveDisarmWithReason(&gate, REARM_DISARM_ESTOP, sink);
            return;
        }
        if (!driveOnCommand(&gate, lin, ang, nowMs, sink)) {
            return;
        }
        // applySkidSteerCommand() 대응물
        sink.motorsCommanded = (lin != 0.0 || ang != 0.0);
    }

    // driveEnableCallback()
    bool onService(bool enable) { return driveOnServiceRequest(&gate, enable, estop, sink); }

    // loop(): spin 이 응답 전송까지 마치고 돌아온 뒤
    void afterSpin(uint32_t nowMs) { rearmGateArmBarrierStart(&gate, nowMs); }

    // checkSafety()
    void safetyTick(uint32_t nowMs)
    {
        if (estop) {
            driveDisarmWithReason(&gate, REARM_DISARM_ESTOP, sink);
            return;
        }
        rearmGateTick(&gate, nowMs);
    }

    // updateMotorOutputs()
    void updateOutputs()
    {
        if (!driveOutputAllowed(&gate, estop, sink)) {
            return;
        }
        // ARMED 이고 명령이 있으면 그대로 나간다.
    }

    // 🔴 불변조건: ARMED 가 아닌데 모터가 돌면 안 된다.
    void expectInvariant(const char* what)
    {
        ++g_checks;
        if (gate.state != DRIVE_ARMED && sink.motorsCommanded) {
            std::printf("  FAIL %-58s state=%s 인데 모터가 돈다\n",
                        what, stateName(gate.state));
            ++g_failures;
        }
    }
};

// 관용 조작 — zero 를 dt 간격으로 흘려 hold 를 채운다.
static void feedZeros(RearmGate& g, uint32_t startMs, uint32_t untilMs, uint32_t stepMs)
{
    for (uint32_t t = startMs; t <= untilMs; t += stepMs) {
        rearmGateOnCommand(&g, 0.0, 0.0, t);
    }
}

// DISARMED → READY → ARMING → PENDING → ARMED 전 과정. 반환 = ARMED 가 된 시각.
// 🔴 응답 전송 시각(respAt)과 콜백 시각을 여기서는 같게 둔다. 둘을 **분리**해서
//    보는 것이 T24 의 일이다.
static uint32_t armFully(RearmGate& g, uint32_t t0)
{
    feedZeros(g, t0, t0 + REARM_ZERO_HOLD_MS, 50);
    rearmGateOnService(&g, true, false);
    const uint32_t respAt = t0 + REARM_ZERO_HOLD_MS;
    rearmGateArmBarrierStart(&g, respAt);
    const uint32_t armedAt = respAt + REARM_POST_ARM_QUIET_MS;
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
    expectBool("T03 서비스 거절", rearmGateOnService(&g, true, false), false);
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
                       rearmGateOnService(&g, true, false), false);
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
               rearmGateOnService(&g, true, false), false);
    expectU32("T07 사유 = ZERO_HOLD", g.rejectReason, REARM_REJECT_ZERO_HOLD);
}

// ── T08 §54.2 P1 — 서비스 성공이 곧바로 ARMED 가 아니다 ────────────────────
static void t08_service_does_not_arm_directly()
{
    RearmGate g;
    rearmGateInit(&g);
    feedZeros(g, 0, 500, 50);
    expectState("T08 서비스 직전", g, DRIVE_READY);
    expectBool("T08 서비스 성공", rearmGateOnService(&g, true, false), true);
    // 🔴 구판(§54)은 여기서 이미 ARMED 였다. 그 다음 판(§55.1 이전)은 PENDING —
    //    즉 장벽 시계가 응답보다 이르게 돌기 시작했다. 이제는 ARMING 이다:
    //    시계 자체가 아직 없다.
    expectState("T08 콜백 반환 시점", g, DRIVE_ARMING);
    expectU32("T08 사유 = OK", g.rejectReason, REARM_OK);
}

// ── T09 §54.2 부정회귀 ① — take 스냅샷 뒤·응답 전 비영 도착 ────────────────
// ② 서비스 콜백 중 도착, ③ 응답 직전 큐 적재 — 셋 다 "장벽 중 도착하는
// 비영 명령"으로 귀결된다. 도착 시각을 장벽 안 전 구간에 걸쳐 훑는다.
static void t09_pre_response_command_cannot_drive()
{
    for (uint32_t delay = 0; delay < REARM_POST_ARM_QUIET_MS; delay += 25) {
        RearmGate g;
        rearmGateInit(&g);
        feedZeros(g, 0, 500, 50);
        rearmGateOnService(&g, true, false);
        rearmGateArmBarrierStart(&g, 500);

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
    rearmGateOnService(&g, true, false);
    rearmGateArmBarrierStart(&g, 500);
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
    rearmGateOnService(&g, true, false);
    rearmGateArmBarrierStart(&g, 500);
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
    const uint8_t targets[4] = {DRIVE_READY, DRIVE_ARMING, DRIVE_PENDING, DRIVE_ARMED};
    for (int i = 0; i < 4; ++i) {
        RearmGate g;
        rearmGateInit(&g);
        feedZeros(g, 0, 500, 50);
        if (targets[i] != DRIVE_READY) {
            rearmGateOnService(&g, true, false);
        }
        if (targets[i] == DRIVE_PENDING || targets[i] == DRIVE_ARMED) {
            rearmGateArmBarrierStart(&g, 500);
        }
        if (targets[i] == DRIVE_ARMED) {
            rearmGateTick(&g, 1000);
        }
        expectState("T13 사전 상태", g, targets[i]);

        expectBool("T13 disable 은 항상 성공",
                   rearmGateOnService(&g, false, false), true);
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
    armFully(g, 0);
    expectBool("T14 E-stop 중 disable", rearmGateOnService(&g, false, true), true);
    expectState("T14 → DISARMED", g, DRIVE_DISARMED);
}

// ── T15 E-stop 중 enable 은 거절 + DISARMED ────────────────────────────────
static void t15_enable_under_estop()
{
    RearmGate g;
    rearmGateInit(&g);
    feedZeros(g, 0, 500, 50);
    expectState("T15 서비스 직전", g, DRIVE_READY);
    expectBool("T15 E-stop 중 enable", rearmGateOnService(&g, true, true), false);
    expectU32("T15 사유 = ESTOP", g.rejectReason, REARM_REJECT_ESTOP);
    expectState("T15 READY 도 잃는다", g, DRIVE_DISARMED);
}

// ── T16 중복 enable 은 ALREADY 로 거절하고 상태를 안 흔든다 ────────────────
static void t16_duplicate_enable()
{
    RearmGate g;
    rearmGateInit(&g);
    feedZeros(g, 0, 500, 50);
    rearmGateOnService(&g, true, false);
    rearmGateArmBarrierStart(&g, 500);
    expectBool("T16 PENDING 중 재호출", rearmGateOnService(&g, true, false), false);
    expectU32("T16 사유 = ALREADY", g.rejectReason, REARM_REJECT_ALREADY);
    expectState("T16 PENDING 유지", g, DRIVE_PENDING);
    // 🔴 거절이 장벽을 리셋하면 안 된다 — 원래 시각 기준으로 승격돼야 한다.
    rearmGateTick(&g, 1000);
    expectState("T16 원래 시각 기준 승격", g, DRIVE_ARMED);

    expectBool("T16 ARMED 중 재호출", rearmGateOnService(&g, true, false), false);
    expectU32("T16 사유 = ALREADY", g.rejectReason, REARM_REJECT_ALREADY);
    expectState("T16 ARMED 유지", g, DRIVE_ARMED);
}

// ── T17 서비스 호출 누계는 거절까지 전부 센다 ──────────────────────────────
static void t17_service_counter_counts_rejections()
{
    RearmGate g;
    rearmGateInit(&g);
    rearmGateOnService(&g, true, false);             // 1 거절 ZERO_HOLD
    rearmGateOnService(&g, true, true);              // 2 거절 ESTOP
    rearmGateOnService(&g, false, false);            // 3 성공 disable
    feedZeros(g, 100, 600, 50);
    rearmGateOnService(&g, true, false);             // 4 성공 → ARMING
    rearmGateOnService(&g, true, false);             // 5 거절 ALREADY
    expectU32("T17 누계", g.serviceCalls, 5);
}

// ── T18 READY 유지 — hold 를 넘겨 계속 zero 를 줘도 READY 에 머문다 ────────
static void t18_ready_is_stable()
{
    RearmGate g;
    rearmGateInit(&g);
    feedZeros(g, 0, 5000, 50);
    expectState("T18 zero 5초", g, DRIVE_READY);
    expectBool("T18 서비스 성공", rearmGateOnService(&g, true, false), true);
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

    rearmGateOnService(&g, true, false);
    rearmGateArmBarrierStart(&g, (uint32_t)(nearMax + 500));
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

    // ③ 서비스 (콜백 반환 = ARMING, 아직 응답 전)
    expectBool("T22 ③ enable", rearmGateOnService(&g, true, false), true);
    expectState("T22 ③ 콜백 반환 시점", g, DRIVE_ARMING);

    // ④ 응답 전송 뒤 장벽 시작 → 0.5초
    rearmGateArmBarrierStart(&g, 1502);
    expectState("T22 ④ 응답 전송 직후", g, DRIVE_PENDING);
    feedZeros(g, 1600, 1900, 100);
    rearmGateTick(&g, 2001);
    expectState("T22 ④ 응답+499ms 는 아직", g, DRIVE_PENDING);
    rearmGateTick(&g, 2002);                         // 1502 + 500
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

// ============================================================================
// §55.1 — 장벽 시계가 서비스 응답 **전송 뒤**에 시작한다
// ============================================================================

// ── T23 콜백만으로는 시계가 없다. tick 을 아무리 줘도 무장되지 않는다 ──────
static void t23_arming_has_no_clock()
{
    RearmGate g;
    rearmGateInit(&g);
    feedZeros(g, 0, 500, 50);
    expectBool("T23 서비스 성공", rearmGateOnService(&g, true, false), true);
    expectState("T23 콜백 반환 = ARMING", g, DRIVE_ARMING);

    // 🔴 구판(§55.1 이전)은 여기서 이미 PENDING 이라 아래 tick 이 무장시켰다.
    for (uint32_t t = 500; t <= 60000; t += 500) {
        rearmGateTick(&g, t);
    }
    expectState("T23 tick 60초 — 여전히 ARMING", g, DRIVE_ARMING);

    // 응답이 나가야 비로소 시계가 돈다.
    rearmGateArmBarrierStart(&g, 60000);
    expectState("T23 응답 전송 직후", g, DRIVE_PENDING);
    rearmGateTick(&g, 60499);
    expectState("T23 장벽 499ms", g, DRIVE_PENDING);
    rearmGateTick(&g, 60500);
    expectState("T23 장벽 500ms", g, DRIVE_ARMED);
}

// ── T24 §55.1 P1 — 콜백↔응답 지연 δ 를 주입해도 장벽은 **응답 기준** 500ms ─
// 검토가 요구한 필수 부정 회귀: δ 를 1ms·경계값·큰 값으로 주입하고, 각 경우
// 응답 기준 +499ms 비영은 전부 거절 · +500ms 뒤 새 비영은 주행이어야 한다.
static void t24_barrier_is_measured_from_response()
{
    const uint32_t deltas[7] = {0, 1, 2, 50, 499, 500, 5000};
    const uint32_t tCallback = 500;

    for (int i = 0; i < 7; ++i) {
        const uint32_t delta = deltas[i];
        const uint32_t tResponse = tCallback + delta;
        char label[128];

        // ── ① 응답 기준 +499ms 의 잔류 비영은 반드시 거절 ──
        {
            RearmGate g;
            rearmGateInit(&g);
            feedZeros(g, 0, tCallback, 50);
            rearmGateOnService(&g, true, false);
            rearmGateArmBarrierStart(&g, tResponse);

            // 구판은 시계가 tCallback 부터라 여기서 이미 ARMED 가 될 수 있었다.
            rearmGateTick(&g, tResponse + REARM_POST_ARM_QUIET_MS - 1);
            std::snprintf(label, sizeof(label),
                          "T24 δ=%ums · 응답+499ms 는 아직 장벽 안", delta);
            expectState(label, g, DRIVE_PENDING);

            std::snprintf(label, sizeof(label),
                          "T24 δ=%ums · 응답+499ms 잔류 비영은 거절", delta);
            expectEffect(label,
                         rearmGateOnCommand(&g, 0.05, 0.0,
                                            tResponse + REARM_POST_ARM_QUIET_MS - 1),
                         DRIVE_EFFECT_HOLD);
            expectState("T24 → DISARMED", g, DRIVE_DISARMED);
        }

        // ── ② 응답 기준 +500ms 뒤 새 비영은 주행 (역회귀) ──
        {
            RearmGate g;
            rearmGateInit(&g);
            feedZeros(g, 0, tCallback, 50);
            rearmGateOnService(&g, true, false);
            rearmGateArmBarrierStart(&g, tResponse);
            rearmGateTick(&g, tResponse + REARM_POST_ARM_QUIET_MS);

            std::snprintf(label, sizeof(label),
                          "T24 δ=%ums · 응답+500ms 에 무장", delta);
            expectState(label, g, DRIVE_ARMED);

            std::snprintf(label, sizeof(label),
                          "T24 δ=%ums · 그 뒤 새 비영은 주행", delta);
            expectEffect(label,
                         rearmGateOnCommand(&g, 0.05, 0.0,
                                            tResponse + REARM_POST_ARM_QUIET_MS + 1),
                         DRIVE_EFFECT_DRIVE);
        }

        // ── ③ 장벽 전 구간(응답 기준 0~499ms) 훑기 ──
        for (uint32_t at = 0; at < REARM_POST_ARM_QUIET_MS; at += 50) {
            RearmGate g;
            rearmGateInit(&g);
            feedZeros(g, 0, tCallback, 50);
            rearmGateOnService(&g, true, false);
            rearmGateArmBarrierStart(&g, tResponse);

            std::snprintf(label, sizeof(label),
                          "T24 δ=%ums · 응답+%ums 비영", delta, at);
            expectEffect(label,
                         rearmGateOnCommand(&g, 0.05, 0.0, tResponse + at),
                         DRIVE_EFFECT_HOLD);
            expectState("T24 → DISARMED", g, DRIVE_DISARMED);
        }
    }
}

// ── T25 콜백↔응답 **사이**에 도착한 비영은 장벽을 시작조차 못 하게 한다 ────
static void t25_command_between_callback_and_response()
{
    RearmGate g;
    rearmGateInit(&g);
    feedZeros(g, 0, 500, 50);
    rearmGateOnService(&g, true, false);
    expectState("T25 ARMING", g, DRIVE_ARMING);

    expectEffect("T25 응답 전 잔류 비영",
                 rearmGateOnCommand(&g, 0.05, 0.0, 501), DRIVE_EFFECT_HOLD);
    expectState("T25 → DISARMED", g, DRIVE_DISARMED);

    // 🔴 그 뒤 응답이 나가도 되살아나면 안 된다.
    rearmGateArmBarrierStart(&g, 502);
    expectState("T25 응답 전송해도 DISARMED", g, DRIVE_DISARMED);
    rearmGateTick(&g, 10000);
    expectState("T25 tick 을 줘도 DISARMED", g, DRIVE_DISARMED);
}

// ── T26 ARMING 중 zero 는 무해하고, 장벽은 여전히 응답 시각 기준 ───────────
static void t26_zero_during_arming_is_harmless()
{
    RearmGate g;
    rearmGateInit(&g);
    feedZeros(g, 0, 500, 50);
    rearmGateOnService(&g, true, false);
    feedZeros(g, 501, 600, 10);                      // 응답 전 zero 폭격
    expectState("T26 ARMING 유지", g, DRIVE_ARMING);

    rearmGateArmBarrierStart(&g, 600);
    rearmGateTick(&g, 1099);
    expectState("T26 응답+499ms", g, DRIVE_PENDING);
    rearmGateTick(&g, 1100);
    expectState("T26 응답+500ms", g, DRIVE_ARMED);
}

// ── T27 barrierStart 는 멱등하다 — 매 루프 불러도 장벽이 안 늘어난다 ───────
static void t27_barrier_start_is_idempotent()
{
    RearmGate g;
    rearmGateInit(&g);
    feedZeros(g, 0, 500, 50);
    rearmGateOnService(&g, true, false);
    rearmGateArmBarrierStart(&g, 500);
    expectU32("T27 장벽 시작 시각", g.quietSinceMs, 500);

    // 스케치는 매 루프 부른다. 두 번째부터는 아무 일도 없어야 한다 —
    // 그렇지 않으면 장벽이 영원히 뒤로 밀려 무장이 안 된다.
    for (uint32_t t = 501; t <= 999; ++t) {
        rearmGateArmBarrierStart(&g, t);
    }
    expectU32("T27 반복 호출 뒤에도 같은 시각", g.quietSinceMs, 500);
    rearmGateTick(&g, 1000);
    expectState("T27 원래 시각 기준 승격", g, DRIVE_ARMED);

    // ARMED 에서 불려도 상태를 안 흔든다.
    rearmGateArmBarrierStart(&g, 2000);
    expectState("T27 ARMED 에서 호출", g, DRIVE_ARMED);
}

// ── T28 ARMING 중 E-stop·disable·중복 enable ───────────────────────────────
static void t28_arming_interruptions()
{
    {   // disable
        RearmGate g;
        rearmGateInit(&g);
        feedZeros(g, 0, 500, 50);
        rearmGateOnService(&g, true, false);
        expectBool("T28 ARMING 중 disable", rearmGateOnService(&g, false, false), true);
        expectState("T28 → DISARMED", g, DRIVE_DISARMED);
        rearmGateArmBarrierStart(&g, 510);
        expectState("T28 그 뒤 응답 전송해도 DISARMED", g, DRIVE_DISARMED);
    }
    {   // E-stop (호출자가 disarm 을 부른다)
        RearmGate g;
        rearmGateInit(&g);
        feedZeros(g, 0, 500, 50);
        rearmGateOnService(&g, true, false);
        rearmGateDisarm(&g);
        expectState("T28 ARMING 중 E-stop", g, DRIVE_DISARMED);
        rearmGateArmBarrierStart(&g, 510);
        expectState("T28 E-stop 뒤 응답 전송해도 DISARMED", g, DRIVE_DISARMED);
    }
    {   // 중복 enable
        RearmGate g;
        rearmGateInit(&g);
        feedZeros(g, 0, 500, 50);
        rearmGateOnService(&g, true, false);
        expectBool("T28 ARMING 중 재호출", rearmGateOnService(&g, true, false), false);
        expectU32("T28 사유 = ALREADY", g.rejectReason, REARM_REJECT_ALREADY);
        expectState("T28 ARMING 유지", g, DRIVE_ARMING);
    }
}

// ── T29 ARMING 상태에서는 어떤 명령도 구동이 아니다 ────────────────────────
static void t29_arming_never_drives()
{
    for (uint32_t at = 500; at <= 600; at += 10) {
        RearmGate g;
        rearmGateInit(&g);
        feedZeros(g, 0, 500, 50);
        rearmGateOnService(&g, true, false);
        expectEffect("T29 ARMING 중 비영은 HOLD",
                     rearmGateOnCommand(&g, 0.05, 0.0, at), DRIVE_EFFECT_HOLD);
    }
}

// ============================================================================
// §55.2 — 정지 배선 (drive_wiring.h) · 가짜 모터로 관측
// ============================================================================

// ── T30 driveDisarm 은 전이 + 정지 + 플래그 해제를 한 번에 한다 ────────────
static void t30_disarm_stops_motors()
{
    const uint8_t targets[5] = {DRIVE_DISARMED, DRIVE_READY, DRIVE_ARMING,
                                DRIVE_PENDING, DRIVE_ARMED};
    for (int i = 0; i < 5; ++i) {
        RearmGate g;
        FakeDriveSink sink;
        rearmGateInit(&g);
        if (targets[i] != DRIVE_DISARMED) {
            feedZeros(g, 0, 500, 50);
        }
        if (targets[i] == DRIVE_ARMING || targets[i] == DRIVE_PENDING ||
            targets[i] == DRIVE_ARMED) {
            rearmGateOnService(&g, true, false);
        }
        if (targets[i] == DRIVE_PENDING || targets[i] == DRIVE_ARMED) {
            rearmGateArmBarrierStart(&g, 500);
        }
        if (targets[i] == DRIVE_ARMED) {
            rearmGateTick(&g, 1000);
        }
        expectState("T30 사전 상태", g, targets[i]);

        sink.cmdVelReceived = true;
        sink.motorsCommanded = true;
        const int before = sink.stopCalls;

        driveDisarm(&g, sink);

        // 🔴 §54.1 의 진짜 구판은 이 세 줄이 안 나는 것이었다.
        expectState("T30 disarm 뒤 상태", g, DRIVE_DISARMED);
        expectU32("T30 stopAllMotors 가 불렸다",
                  (uint32_t)(sink.stopCalls - before), 1);
        expectBool("T30 수신 플래그 해제", sink.cmdVelReceived, false);
        expectBool("T30 모터 정지", sink.motorsCommanded, false);
    }
}

// ── T31 출력단 가드는 ARMED + E-stop 해제일 때만 통과한다 ──────────────────
static void t31_output_guard()
{
    const uint8_t states[5] = {DRIVE_DISARMED, DRIVE_READY, DRIVE_ARMING,
                               DRIVE_PENDING, DRIVE_ARMED};
    for (int i = 0; i < 5; ++i) {
        for (int estop = 0; estop < 2; ++estop) {
            RearmGate g;
            FakeDriveSink sink;
            rearmGateInit(&g);
            g.state = states[i];
            sink.cmdVelReceived = true;
            sink.motorsCommanded = true;

            const bool want = (states[i] == DRIVE_ARMED) && (estop == 0);
            char label[128];
            std::snprintf(label, sizeof(label), "T31 %s · estop=%d 출력 허용",
                          stateName(states[i]), estop);
            const bool got = driveOutputAllowed(&g, estop != 0, sink);
            expectBool(label, got, want);

            if (want) {
                expectU32("T31 허용이면 정지를 안 부른다",
                          (uint32_t)sink.stopCalls, 0);
            } else {
                // 🔴 §54.1 의 두 번째 겹. 이 분기를 지우는 것이 검토자의 변이 ②였다.
                expectU32("T31 불허면 그 자리에서 정지", (uint32_t)sink.stopCalls, 1);
                expectBool("T31 불허면 수신 플래그 해제", sink.cmdVelReceived, false);
                expectBool("T31 불허면 모터 정지", sink.motorsCommanded, false);
            }
        }
    }
}

// ── T32 거절된 /cmd_vel 은 그 자리에서 정지한다 ────────────────────────────
static void t32_rejected_command_stops()
{
    {   // 비유한 입력
        RearmGate g;
        FakeDriveSink sink;
        rearmGateInit(&g);
        armFully(g, 0);
        sink.motorsCommanded = true;
        sink.cmdVelReceived = true;
        expectBool("T32 NaN 은 거절", driveOnCommand(&g, NAN, 0.0, 1100, sink), false);
        expectU32("T32 정지 호출", (uint32_t)sink.stopCalls, 1);
        expectBool("T32 수신 플래그 해제", sink.cmdVelReceived, false);
        expectBool("T32 모터 정지", sink.motorsCommanded, false);
    }
    {   // 무장 전 비영
        RearmGate g;
        FakeDriveSink sink;
        rearmGateInit(&g);
        expectBool("T32 DISARMED 에서 비영은 거절",
                   driveOnCommand(&g, 0.05, 0.0, 10, sink), false);
        expectU32("T32 정지 호출", (uint32_t)sink.stopCalls, 1);
    }
    {   // 정상 주행 (역회귀)
        RearmGate g;
        FakeDriveSink sink;
        rearmGateInit(&g);
        const uint32_t armedAt = armFully(g, 0);
        expectBool("T32 ARMED 에서 비영은 수용",
                   driveOnCommand(&g, 0.05, 0.0, armedAt + 1, sink), true);
        expectU32("T32 정지를 안 부른다", (uint32_t)sink.stopCalls, 0);
        expectBool("T32 수신 플래그 설정", sink.cmdVelReceived, true);
        expectU32("T32 마지막 명령 시각", sink.lastCmdVelMs, armedAt + 1);
    }
}

// ── T33 서비스 응답 **전에** 정지가 끝나 있다 ──────────────────────────────
static void t33_service_stops_before_response()
{
    {   // disable — 검토 §55.2 완료판정의 그 자리
        RearmGate g;
        FakeDriveSink sink;
        rearmGateInit(&g);
        armFully(g, 0);
        sink.motorsCommanded = true;
        sink.cmdVelReceived = true;

        const bool success = driveOnServiceRequest(&g, false, false, sink);
        // 반환값을 응답에 넣기 **전에** 이미 관측되는 사실들:
        expectBool("T33 disable 성공", success, true);
        expectState("T33 → DISARMED", g, DRIVE_DISARMED);
        expectU32("T33 응답 전 정지 호출", (uint32_t)sink.stopCalls, 1);
        expectBool("T33 응답 전 모터 0", sink.motorsCommanded, false);
        expectBool("T33 응답 전 수신 플래그 0", sink.cmdVelReceived, false);
    }
    {   // enable 성공 — ARMING 이므로 여전히 정지 상태여야 한다
        RearmGate g;
        FakeDriveSink sink;
        rearmGateInit(&g);
        feedZeros(g, 0, 500, 50);
        const bool success = driveOnServiceRequest(&g, true, false, sink);
        expectBool("T33 enable 성공", success, true);
        expectState("T33 → ARMING", g, DRIVE_ARMING);
        expectU32("T33 무장 성공해도 응답 전 정지", (uint32_t)sink.stopCalls, 1);
        expectBool("T33 모터 0 유지", sink.motorsCommanded, false);
    }
    {   // enable 거절
        RearmGate g;
        FakeDriveSink sink;
        rearmGateInit(&g);
        const bool success = driveOnServiceRequest(&g, true, false, sink);
        expectBool("T33 zero-hold 없이 enable 은 거절", success, false);
        expectU32("T33 거절도 정지", (uint32_t)sink.stopCalls, 1);
    }
}

// ── T34 스케치 모사 — 전 과정에서 ARMED 밖에서는 모터가 안 돈다 ────────────
// 🔴 이것이 검토 §55.2 가 요구한 "두 정지 강제점을 되돌리면 FAIL" 의 본체다.
//    disarmDrive 의 정지를 빼면 ⑤·⑧ 에서, 출력단 가드를 빼면 ④·⑥ 에서 죽는다.
static void t34_sketch_model_invariant()
{
    SketchModel m;

    // ① 부팅 — 명령을 아무리 줘도 안 돈다
    for (uint32_t t = 0; t < 300; t += 20) {
        m.onCmdVel(0.2, 0.1, t);
        m.updateOutputs();
        m.expectInvariant("T34 ① 부팅 직후");
    }

    // ② zero 0.5초 → READY
    for (uint32_t t = 1000; t <= 1500; t += 50) {
        m.onCmdVel(0.0, 0.0, t);
        m.updateOutputs();
        m.expectInvariant("T34 ② zero-hold 중");
    }
    expectState("T34 ② READY", m.gate, DRIVE_READY);

    // ③ enable — 콜백 반환은 ARMING, 응답 전이라 아직 못 돈다
    expectBool("T34 ③ enable 성공", m.onService(true), true);
    expectState("T34 ③ ARMING", m.gate, DRIVE_ARMING);
    m.updateOutputs();
    m.expectInvariant("T34 ③ 응답 전");

    // ④ 응답 전송 전에 도착한 잔류 비영 → 떨어진다
    m.onCmdVel(0.3, 0.0, 1501);
    m.updateOutputs();
    expectState("T34 ④ 잔류 명령 → DISARMED", m.gate, DRIVE_DISARMED);
    m.expectInvariant("T34 ④ 잔류 명령 뒤");
    expectBool("T34 ④ 모터 0", m.sink.motorsCommanded, false);

    // ⑤ 다시 처음부터 — zero → enable → 응답 → 장벽 완주
    for (uint32_t t = 2000; t <= 2500; t += 50) {
        m.onCmdVel(0.0, 0.0, t);
        m.updateOutputs();
        m.expectInvariant("T34 ⑤ 재시도 zero-hold");
    }
    expectBool("T34 ⑤ enable", m.onService(true), true);
    m.afterSpin(2502);
    expectState("T34 ⑤ 응답 뒤 PENDING", m.gate, DRIVE_PENDING);
    for (uint32_t t = 2510; t < 3002; t += 10) {
        m.onCmdVel(0.0, 0.0, t);
        m.safetyTick(t);
        m.updateOutputs();
        m.expectInvariant("T34 ⑤ 장벽 중");
    }
    expectState("T34 ⑤ 응답+499ms 는 아직", m.gate, DRIVE_PENDING);
    m.safetyTick(3002);                              // 2502 + 500
    expectState("T34 ⑤ 장벽 완주", m.gate, DRIVE_ARMED);

    // ⑥ 주행 — 여기서만 돈다
    m.onCmdVel(0.2, 0.0, 3010);
    m.updateOutputs();
    expectBool("T34 ⑥ ARMED 에서 모터가 돈다", m.sink.motorsCommanded, true);
    m.expectInvariant("T34 ⑥ 주행 중");

    // ⑦ E-stop → 그 자리에서 0
    m.estop = true;
    m.safetyTick(3020);
    expectState("T34 ⑦ E-stop → DISARMED", m.gate, DRIVE_DISARMED);
    expectBool("T34 ⑦ 모터 0", m.sink.motorsCommanded, false);
    m.updateOutputs();
    m.expectInvariant("T34 ⑦ E-stop 뒤");

    // ⑧ E-stop 해제 — 자동 재가동은 없다
    m.estop = false;
    for (uint32_t t = 3100; t < 5000; t += 20) {
        m.onCmdVel(0.2, 0.0, t);       // 계속 비영을 쏜다
        m.safetyTick(t);
        m.updateOutputs();
        m.expectInvariant("T34 ⑧ E-stop 해제 뒤 비영 폭격");
    }
    expectState("T34 ⑧ 여전히 DISARMED", m.gate, DRIVE_DISARMED);
    expectBool("T34 ⑧ 모터 0", m.sink.motorsCommanded, false);
}

// ── T35 주행 중 disable — 응답 전에 모터가 0 이다 ──────────────────────────
static void t35_disable_while_driving()
{
    SketchModel m;
    for (uint32_t t = 0; t <= 500; t += 50) {
        m.onCmdVel(0.0, 0.0, t);
    }
    m.onService(true);
    m.afterSpin(500);
    m.safetyTick(1000);
    expectState("T35 무장", m.gate, DRIVE_ARMED);

    m.onCmdVel(0.2, 0.0, 1010);
    m.updateOutputs();
    expectBool("T35 주행 중", m.sink.motorsCommanded, true);

    const int before = m.sink.stopCalls;
    expectBool("T35 disable 성공", m.onService(false), true);
    expectU32("T35 응답 전 정지 호출", (uint32_t)(m.sink.stopCalls - before), 1);
    expectBool("T35 응답 전 모터 0", m.sink.motorsCommanded, false);
    expectBool("T35 응답 전 수신 플래그 0", m.sink.cmdVelReceived, false);
    m.expectInvariant("T35 disable 뒤");
}


// ============================================================================
// 2026-08-13 신설 — E-stop 디바운스 (estop_debounce.h)
//
// 왜: 08-12 에 주행 중 PIN21 이 스스로 떠서 무장이 12회 이상 풀렸다. 사람 누름은
//     수백 ms, 글리치는 수 ms~수십 ms 이므로 **시간**으로 가른다.
// 🔴 여기서 보는 것은 두 가지다 — ① 짧은 글리치를 먹는가 ② **진짜 누름을 놓치지
//     않는가**. ②가 깨지면 이 필터는 안전 기능을 망가뜨린 것이다.
// ============================================================================

static void t36_debounce_boot_trusts_the_pin()
{
    // 🔴 부팅 시엔 필터를 태우지 않는다. 켰을 때 눌려 있으면 즉시 눌린 것이다.
    EstopDebounce d;
    estopDebounceInit(&d, true, 1000);
    expectBool("T36 부팅 시 눌림은 즉시 눌림 (fail-closed)", d.stable, true);

    EstopDebounce e;
    estopDebounceInit(&e, false, 1000);
    expectBool("T36 부팅 시 해제는 해제", e.stable, false);
}

static void t37_short_glitch_is_swallowed()
{
    EstopDebounce d;
    estopDebounceInit(&d, false, 0);
    // 0~20ms 동안 HIGH 로 떴다가 돌아온다 (디바운스 30ms 미만)
    bool tripped = false;
    for (uint32_t t = 1; t <= 20; ++t) {
        if (estopDebounceUpdate(&d, true, t)) { tripped = true; }
    }
    for (uint32_t t = 21; t <= 60; ++t) {
        if (estopDebounceUpdate(&d, false, t)) { tripped = true; }
    }
    expectBool("T37 20ms 글리치는 E-stop 으로 승격되지 않는다", tripped, false);
    expectBool("T37 글리치는 계수에 남는다 (rawEdges>=2)", d.rawEdges >= 2, true);
    expectBool("T37 최대 HIGH 지속이 기록된다 (>=19ms)", d.maxHighMs >= 19, true);
}

static void t38_real_press_is_not_missed()
{
    // 🔴 이게 안전 계약이다 — 진짜 누름은 반드시 잡혀야 한다.
    EstopDebounce d;
    estopDebounceInit(&d, false, 0);
    bool tripped = false;
    for (uint32_t t = 1; t <= 500; ++t) {
        if (estopDebounceUpdate(&d, true, t)) { tripped = true; }
    }
    expectBool("T38 500ms 누름은 반드시 잡힌다", tripped, true);
    expectBool("T38 누름 중에는 판정이 유지된다", d.stable, true);
}

static void t39_threshold_boundary()
{
    // 🔴 경계는 **전이 시각 기준**이다. 아래 열은 t=1 에서 LOW→HIGH 로 바뀌므로
    //    승격은 t=31(= 전이 + 30ms)이고, t=30 은 아직 29ms 라 안 된다.
    EstopDebounce a;
    estopDebounceInit(&a, false, 0);
    bool tripped_before = false;
    for (uint32_t t = 1; t <= 30; ++t) {
        if (estopDebounceUpdate(&a, true, t)) { tripped_before = true; }
    }
    expectBool("T39 전이 후 29ms 에서는 아직 아니다", tripped_before, false);

    EstopDebounce b;
    estopDebounceInit(&b, false, 0);
    bool tripped_at = false;
    for (uint32_t t = 1; t <= 31; ++t) {
        if (estopDebounceUpdate(&b, true, t)) { tripped_at = true; }
    }
    expectBool("T39 전이 후 30ms 에서 승격된다", tripped_at, true);
}

static void t40_release_is_also_filtered()
{
    // 해제 방향도 같은 시간을 쓴다. 늦어지는 것은 **정지 유지**라 안전한 방향이다.
    EstopDebounce d;
    estopDebounceInit(&d, true, 0);
    bool released_early = false;
    for (uint32_t t = 1; t <= 20; ++t) {
        if (!estopDebounceUpdate(&d, false, t)) { released_early = true; }
    }
    expectBool("T40 20ms 해제는 아직 해제가 아니다 (정지를 더 유지)", released_early, false);
    for (uint32_t t = 21; t <= 40; ++t) { estopDebounceUpdate(&d, false, t); }
    expectBool("T40 30ms 넘으면 해제된다", d.stable, false);
}

static void t41_chatter_never_arms_the_gate()
{
    // 🔴 08-12 실물 모양 — 계속 채터링하는 접점. 필터가 있으면 판정이 안 흔들려야 한다.
    EstopDebounce d;
    estopDebounceInit(&d, false, 0);
    bool tripped = false;
    uint32_t t = 0;
    for (int cycle = 0; cycle < 200; ++cycle) {
        for (int i = 0; i < 5; ++i)  { if (estopDebounceUpdate(&d, true,  ++t)) tripped = true; }
        for (int i = 0; i < 15; ++i) { if (estopDebounceUpdate(&d, false, ++t)) tripped = true; }
    }
    expectBool("T41 5ms HIGH 채터링 200회로도 안 뜬다", tripped, false);
    expectU32("T41 그래도 400 전이가 전부 계수된다", d.rawEdges, 400);
    expectBool("T41 최대 HIGH 지속은 5ms 이하로 기록", d.maxHighMs <= 5, true);
}

static void t42_long_glitch_is_not_hidden()
{
    // 🔴 필터가 못 막는 경우를 시험한다. 30ms 를 넘는 글리치는 **통과한다** —
    //    그때는 값을 키울 게 아니라 원인을 고쳐야 한다. 계수가 그 사실을 남긴다.
    EstopDebounce d;
    estopDebounceInit(&d, false, 0);
    bool tripped = false;
    for (uint32_t t = 1; t <= 50; ++t) {
        if (estopDebounceUpdate(&d, true, t)) { tripped = true; }
    }
    expectBool("T42 50ms 글리치는 필터를 넘는다 (숨기지 않는다)", tripped, true);
    expectBool("T42 그 지속시간이 계수에 남는다 (>=49ms)", d.maxHighMs >= 49, true);
}

// ════════════════════════════════════════════════════════════════════════════
// §63.1 2회차 — 필터·상태기계·게시 주기를 **한 시계열**에 올린다
//
// 🔴 1회차 검토가 초록을 뚫은 이유가 이것이다. T36~T42 는 필터만, T01~T35 는
//    상태기계만 봤다. 실제 반례는 셋이 겹칠 때만 나온다:
//    E-stop 이 풀었는데(4) → 주행 중이던 10Hz 비영이 계속 오고(6) → 진단은 1Hz.
//    각각은 옳은데 합치면 원인이 지워진다. 그래서 아래는 ms 단위로 같이 돈다.
// ════════════════════════════════════════════════════════════════════════════

// .ino 의 loop() 한 바퀴를 1ms 로 모사한다. 스케치 순서 그대로:
//   updateEstopFilter() → checkSafety() → (10Hz) cmdVelCallback() → (1Hz) diag 발행
struct TimelineModel {
    RearmGate gate;
    FakeDriveSink sink;
    EstopDebounce filter;

    // 발행된 /drive/diag 표본 (t, y)
    std::vector<std::pair<uint32_t, uint8_t>> diag;

    TimelineModel(uint32_t t0 = 0)
    {
        rearmGateInit(&gate);
        estopDebounceInit(&filter, false, t0);
    }

    // rawHigh = 그 순간 PIN21 의 원시 읽기. cmd = 이번 ms 에 보낼 /cmd_vel (없으면 nullptr).
    void step(uint32_t t, bool rawHigh, const double* cmd, bool publishDiag)
    {
        estopDebounceUpdate(&filter, rawHigh, t);      // updateEstopFilter()
        if (filter.stable) {                            // checkSafety()
            driveDisarmWithReason(&gate, REARM_DISARM_ESTOP, sink);
        } else {
            rearmGateTick(&gate, t);
        }
        if (cmd != nullptr) {                           // cmdVelCallback()
            if (filter.stable) {
                driveDisarmWithReason(&gate, REARM_DISARM_ESTOP, sink);
            } else {
                driveOnCommand(&gate, cmd[0], cmd[1], t, sink);
            }
        }
        if (publishDiag) {
            diag.emplace_back(t, gate.rejectReason);
        }
    }

    // 정상 무장 4단계를 t0 까지 끝내고 ARMED 로 만든다.
    void armBy(uint32_t& t)
    {
        const double zero[2] = {0.0, 0.0};
        for (; t <= 600; t += 10) { step(t, false, zero, false); }
        driveOnServiceRequest(&gate, true, false, sink);
        rearmGateArmBarrierStart(&gate, t);
        for (uint32_t end = t + REARM_POST_ARM_QUIET_MS + 20; t <= end; t += 10) {
            step(t, false, zero, false);
        }
    }
};

// ── T43 🔴 반례 그대로 — E-stop 이 푼 사유가 비영 홍수에 안 덮인다 ───────────
static void t43_estop_reason_survives_the_command_flood()
{
    TimelineModel m;
    uint32_t t = 0;
    m.armBy(t);
    expectU32("T43 준비: ARMED", m.gate.state, DRIVE_ARMED);

    const double go[2] = {0.10, 0.0};
    const uint32_t highFrom = t + 100;
    const uint32_t highTo = highFrom + 49;   // 50ms HIGH — 필터를 넘긴다

    // 2초 동안: 10Hz 비영 명령 · 1Hz 진단 · 그 사이 50ms HIGH 한 번
    for (uint32_t k = 0; k <= 2000; ++k) {
        const uint32_t now = t + k;
        const bool rawHigh = (now >= highFrom && now <= highTo);
        const bool sendCmd = (k % 100 == 0);
        const bool pubDiag = (k % 1000 == 0 && k > 0);
        m.step(now, rawHigh, sendCmd ? go : nullptr, pubDiag);
    }

    expectU32("T43 무장은 풀렸다", m.gate.state, DRIVE_DISARMED);
    expectBool("T43 진단이 실제로 나갔다", m.diag.size() >= 2, true);
    // 🔴 구판은 여기가 6 이었다. 그래서 E-stop 이 밖에서 안 보였다.
    expectU32("T43 첫 진단이 E-stop 사유를 담는다", m.diag.front().second,
              REARM_DISARM_ESTOP);
    expectU32("T43 마지막 진단까지 유지된다", m.diag.back().second,
              REARM_DISARM_ESTOP);
    expectU32("T43 E-stop 누계는 눌린 횟수만큼만 (루프마다 아니다)",
              m.gate.disarmEstopCount, 1u);
    expectU32("T43 비영 누계는 0 — 이미 풀린 뒤의 명령은 푼 것이 아니다",
              m.gate.disarmNonzeroCount, 0u);
}

// ── T44 NaN 사유도 뒤이은 비영에 안 덮인다 ──────────────────────────────────
static void t44_nonfinite_reason_survives()
{
    TimelineModel m;
    uint32_t t = 0;
    m.armBy(t);

    const double nan2[2] = {NAN, 0.0};
    m.step(t + 1, false, nan2, false);
    expectU32("T44 NaN 이 풀었다", m.gate.rejectReason, REARM_DISARM_NONFINITE);

    const double go[2] = {0.10, 0.0};
    for (uint32_t k = 1; k <= 20; ++k) {
        m.step(t + 1 + k * 100, false, go, false);
    }
    expectU32("T44 비영 20발 뒤에도 사유 5 유지", m.gate.rejectReason,
              REARM_DISARM_NONFINITE);
    expectU32("T44 NaN 누계 1", m.gate.disarmNonfiniteCount, 1u);
}

// ── T45 역회귀 — 진짜 장벽 위반은 여전히 6 이다 ─────────────────────────────
static void t45_real_barrier_violation_still_reports_nonzero()
{
    // READY 에서 온 첫 비영
    {
        RearmGate g;
        rearmGateInit(&g);
        rearmGateOnCommand(&g, 0.0, 0.0, 0);
        rearmGateOnCommand(&g, 0.0, 0.0, REARM_ZERO_HOLD_MS + 10);
        expectU32("T45 준비: READY", g.state, DRIVE_READY);
        rearmGateOnCommand(&g, 0.05, 0.0, REARM_ZERO_HOLD_MS + 20);
        expectU32("T45 READY 의 첫 비영 = 사유 6", g.rejectReason,
                  REARM_DISARM_NONZERO);
        expectU32("T45 그리고 누계 1", g.disarmNonzeroCount, 1u);
    }
    // ARMING 에서 온 잔류 명령
    {
        RearmGate g;
        rearmGateInit(&g);
        rearmGateOnCommand(&g, 0.0, 0.0, 0);
        rearmGateOnCommand(&g, 0.0, 0.0, REARM_ZERO_HOLD_MS + 10);
        rearmGateOnService(&g, true, false);
        expectU32("T45 준비: ARMING", g.state, DRIVE_ARMING);
        rearmGateOnCommand(&g, 0.05, 0.0, REARM_ZERO_HOLD_MS + 20);
        expectU32("T45 ARMING 의 잔류 비영 = 사유 6", g.rejectReason,
                  REARM_DISARM_NONZERO);
    }
    // PENDING 에서 온 잔류 명령
    {
        RearmGate g;
        rearmGateInit(&g);
        rearmGateOnCommand(&g, 0.0, 0.0, 0);
        rearmGateOnCommand(&g, 0.0, 0.0, REARM_ZERO_HOLD_MS + 10);
        rearmGateOnService(&g, true, false);
        rearmGateArmBarrierStart(&g, REARM_ZERO_HOLD_MS + 15);
        expectU32("T45 준비: PENDING", g.state, DRIVE_PENDING);
        rearmGateOnCommand(&g, 0.05, 0.0, REARM_ZERO_HOLD_MS + 20);
        expectU32("T45 PENDING 의 잔류 비영 = 사유 6", g.rejectReason,
                  REARM_DISARM_NONZERO);
    }
    // ARMED 에서 NaN → 6 이 아니라 5
    {
        RearmGate g;
        rearmGateInit(&g);
        rearmGateOnCommand(&g, 0.0, 0.0, 0);
        rearmGateOnCommand(&g, 0.0, 0.0, REARM_ZERO_HOLD_MS + 10);
        rearmGateOnService(&g, true, false);
        rearmGateArmBarrierStart(&g, REARM_ZERO_HOLD_MS + 15);
        rearmGateTick(&g, REARM_ZERO_HOLD_MS + 15 + REARM_POST_ARM_QUIET_MS + 1);
        expectU32("T45 준비: ARMED", g.state, DRIVE_ARMED);
        rearmGateOnCommand(&g, INFINITY, 0.0, REARM_ZERO_HOLD_MS + 800);
        expectU32("T45 ARMED 의 Inf = 사유 5", g.rejectReason,
                  REARM_DISARM_NONFINITE);
    }
}

// ── T46 🔴 §5-G6 진짜 누름 10회가 글리치 자를 오염시키지 않는다 ─────────────
static void t46_real_presses_do_not_pollute_the_glitch_ruler()
{
    EstopDebounce d;
    estopDebounceInit(&d, false, 0);
    uint32_t t = 1;

    // 필수 토글 절차 모사 — 500ms 누름 · 500ms 해제, 10회
    for (int i = 0; i < 10; ++i) {
        for (uint32_t k = 0; k < 500; ++k, ++t) { estopDebounceUpdate(&d, true, t); }
        for (uint32_t k = 0; k < 500; ++k, ++t) { estopDebounceUpdate(&d, false, t); }
    }
    expectU32("T46 진짜 누름은 글리치로 안 센다", d.rejectedHighCount, 0u);
    expectU32("T46 그래서 글리치 자는 아직 0", d.maxRejectedHighMs, 0u);
    expectBool("T46 (전체 max 는 예상대로 오염된다 — 그래서 칸을 나눴다)",
               d.maxHighMs >= 499, true);

    // 이제 주행 중 20ms 글리치 하나
    for (uint32_t k = 0; k < 20; ++k, ++t) { estopDebounceUpdate(&d, true, t); }
    for (uint32_t k = 0; k < 50; ++k, ++t) { estopDebounceUpdate(&d, false, t); }

    expectU32("T46 글리치 1회가 잡힌다", d.rejectedHighCount, 1u);
    expectBool("T46 그 길이가 20ms 근처로 읽힌다 (18~22)",
               d.maxRejectedHighMs >= 18 && d.maxRejectedHighMs <= 22, true);
    // 🔴 이것이 "30ms 가 충분한가"에 답할 수 있는 유일한 값이다.
}

// ── T47 필터를 넘은 글리치는 rejected 에 안 들어간다 (T42 의 짝) ────────────
static void t47_promoted_high_is_not_a_rejected_glitch()
{
    EstopDebounce d;
    estopDebounceInit(&d, false, 0);
    uint32_t t = 1;
    for (uint32_t k = 0; k < 50; ++k, ++t) { estopDebounceUpdate(&d, true, t); }
    for (uint32_t k = 0; k < 50; ++k, ++t) { estopDebounceUpdate(&d, false, t); }
    expectU32("T47 50ms 는 승격됐으므로 글리치 계수에 없다", d.rejectedHighCount, 0u);
    expectBool("T47 대신 전체 max 에는 남는다", d.maxHighMs >= 49, true);
    // 🔴 판독 규약: rejected=0 인데 무장이 풀렸다면 "필터를 넘었다" = 원인을 고칠 때다.
}

// ── T48 역회귀 — millis() 랩어라운드에서도 두 자가 다 옳다 ──────────────────
static void t48_glitch_ruler_survives_wraparound()
{
    const uint32_t base = 0xFFFFFFF0u;   // 16ms 뒤 랩
    EstopDebounce d;
    estopDebounceInit(&d, false, base);

    uint32_t t = base + 1;
    for (uint32_t k = 0; k < 20; ++k, ++t) { estopDebounceUpdate(&d, true, t); }
    for (uint32_t k = 0; k < 40; ++k, ++t) { estopDebounceUpdate(&d, false, t); }
    expectU32("T48 랩을 가로지른 20ms 글리치도 1회", d.rejectedHighCount, 1u);
    expectBool("T48 길이도 옳다 (18~22)",
               d.maxRejectedHighMs >= 18 && d.maxRejectedHighMs <= 22, true);

    bool tripped = false;
    for (uint32_t k = 0; k < 500; ++k, ++t) {
        if (estopDebounceUpdate(&d, true, t)) { tripped = true; }
    }
    expectBool("T48 랩 이후에도 500ms 진짜 누름은 잡힌다", tripped, true);
}

int main()
{
    std::printf("=== re-arm 래치 상태 전이 + 정지 배선 harness (검토 §54·§55) ===\n");
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
    // §55.1 — 장벽 시계의 기준점
    t23_arming_has_no_clock();
    t24_barrier_is_measured_from_response();
    t25_command_between_callback_and_response();
    t26_zero_during_arming_is_harmless();
    t27_barrier_start_is_idempotent();
    t28_arming_interruptions();
    t29_arming_never_drives();
    // §55.2 — 정지 배선
    t30_disarm_stops_motors();
    t31_output_guard();
    t32_rejected_command_stops();
    t33_service_stops_before_response();
    t34_sketch_model_invariant();
    t35_disable_while_driving();
    // 2026-08-13 — E-stop 디바운스
    t36_debounce_boot_trusts_the_pin();
    t37_short_glitch_is_swallowed();
    t38_real_press_is_not_missed();
    t39_threshold_boundary();
    t40_release_is_also_filtered();
    t41_chatter_never_arms_the_gate();
    t42_long_glitch_is_not_hidden();
    // §63.1 2회차 — 필터 × 상태기계 × 게시 주기를 한 시계열에서
    t43_estop_reason_survives_the_command_flood();
    t44_nonfinite_reason_survives();
    t45_real_barrier_violation_still_reports_nonzero();
    t46_real_presses_do_not_pollute_the_glitch_ruler();
    t47_promoted_high_is_not_a_rejected_glitch();
    t48_glitch_ruler_survives_wraparound();

    std::printf("\n검사 %d건 · 실패 %d건\n", g_checks, g_failures);
    if (g_failures != 0) {
        std::printf("FAIL 상태 전이·정지 배선 계약이 깨졌다 — 굽지 않는다.\n");
        return 1;
    }
    std::printf("OK   전이표 + 정지 배선 전량 통과.\n");
    std::printf("     ⚠ 스케치가 이 함수들을 부르는지는 2단계 구조 검사가 본다.\n");
    std::printf("     ⚠ PWM 파형·publish 내용은 실기 JETSON_SETUP §7-c-E 가 본다.\n");
    return 0;
}
