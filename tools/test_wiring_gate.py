#!/usr/bin/env python3
"""`§7-c-E` 6 행 축소 관문이 **생략한 7 행의 폐포 전체**를 잠그는지 본다 (검토 §69.3).

왜 이 시험이 있나
-----------------
`rearm_field_wiring.py` 는 실기와 통신하므로 저장소 회귀가 없다(예약 21 과 같은 자리).
그래서 **거부 경로만은 여기서 기계로 친다** — 실기 없이 돌아가는 부분이다.

08-13 밤에 두 번 뚫렸다:
  1차(§68.3) 두 헤더만 잠갔다 → `.ino` 통합 배선을 바꿔도 통과
  2차(§69.3) 일곱 함수로 늘렸는데 **생략 행이 판정하는 값을 만드는 곳**이 빠졌다.
             `driveDiagMessage.y`(= 부정 2·3·5 가 보는 거절 사유)를 `0.0` 으로 바꿔도,
             `setup()` 의 서비스 binding 을 다른 콜백으로 바꿔도 **ACCEPT** 했다.
🔴 6 행은 `y` 를 안 보므로, 그런 blob 은 **실기 6 행까지 통과하면서** 생략한 세 행의
   계약을 깬다. 축소의 fail-closed 전제가 성립하지 않는 것이다.

무엇을 검사하나
--------------
  ① 현행 `.ino`·헤더에서 관문이 **통과**한다 (상수-only diff 는 축소를 허용한다)
  ② 🔴 생략 행의 폐포를 흔드는 변이가 **전부 거부**된다
     — 상태 전이(`loop` 의 tick·장벽 순서)
     — 판정 대상 값(`driveDiagMessage.x/y/z` · `driveEnabledMessage.data`)
     — 콜백 결선(`setup` 의 service/subscription binding)
     — 정지 부작용(`disarmDrive`)
  ③ 두 게이트 헤더 변경도 여전히 거부된다
  ④ 지문 표가 실제 `.ino` 함수와 1:1 이다 (이름이 바뀌면 조용히 안 잠긴다)

사용
----
    python3 tools/test_wiring_gate.py
    echo $?      # 0 = 통과 / 1 = 계약 위반 / 2 = 판정 불능

정본 = docs/JETSON_SETUP.md §7-c-E2 · 검토현황 §68.3 · §69.3.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ino_host_probe as probe                       # noqa: E402
import rearm_field_wiring as wiring                  # noqa: E402

#: 🔴 생략 7 행의 폐포를 흔드는 변이. 전부 거부되어야 한다.
#:   (원문, 바꿀 것, 무엇을 깨는가)
MUTATIONS = (
    ("loop 의 tick 시각", "  rearmGateTick(&driveGate, millis());",
     "  rearmGateTick(&driveGate, millis() + 1);", "전환 2·3 장벽"),
    ("진단 y = 거절사유",
     "driveDiagMessage.y = static_cast<double>(driveGate.rejectReason);",
     "driveDiagMessage.y = 0.0;", "부정 2·3·5 가 보는 값"),
    ("진단 x = 서비스 호출수",
     "driveDiagMessage.x = static_cast<double>(driveGate.serviceCalls);",
     "driveDiagMessage.x = 0.0;", "서비스 도달 확인"),
    ("진단 z = 상태",
     "driveDiagMessage.z = static_cast<double>(driveGate.state);",
     "driveDiagMessage.z = 2.0;", "전환 전량"),
    ("enabled 발행",
     "driveEnabledMessage.data = (driveGate.state == DRIVE_ARMED);",
     "driveEnabledMessage.data = true;", "전환 3 · 부정 4"),
    ("setup 서비스 binding", "      &driveEnableCallback));",
     "      &resetOdomCallback));", "부정 2·3·5 · 해제"),
    ("setup cmd_vel binding", "      &cmdVelCallback,",
     "      &resetYawCallback,", "전환 1"),
    ("setup executor 등록", "  RCCHECK(rclc_executor_add_service(",
     "  RCSOFTCHECK(rclc_executor_add_service(", "서비스 도달"),
    ("해제의 정지 부작용", "  driveDisarm(&driveGate, driveSink);",
     "  (void)driveGate;", "해제"),
)

FAILURES = []


def check(label, ok, detail=""):
    if ok:
        print("  \033[32mOK\033[0m  " + label)
    else:
        print("  \033[31mNG\033[0m  " + label + ("\n      " + detail if detail else ""))
        FAILURES.append(label)


def with_ino(mutate):
    """`.ino` 를 문자열만 바꿔 읽히게 하고 관문을 돌린다. 저장소는 안 건드린다."""
    original = probe.load
    probe.load = lambda ref=None: mutate(original(ref))
    try:
        return wiring.gate_headers_unchanged()
    finally:
        probe.load = original


def main():
    print("축소 관문 폐포 회귀 (검토 §68.3 · §69.3)")
    print("  잠그는 대상: 헤더 %d + `.ino` 함수 %d"
          % (len(wiring.GATE_SHA256), len(wiring.INO_WIRING_SHA256)))

    print("\n[1] 현행 — 상수-only diff 에서 축소가 허용되는가")
    bad = wiring.gate_headers_unchanged()
    check("① 현행 트리에서 관문 통과", not bad,
          "축소 전제가 안 맞으면 6 행을 못 쓴다: %s" % (bad[0] if bad else ""))

    print("\n[2] 🔴 생략 7 행의 폐포를 흔들면 전부 거부되어야 한다")
    source = probe.load()
    for label, old, new, breaks in MUTATIONS:
        if source.count(old) < 1:
            check("② %s" % label, False,
                  "변이 지점이 `.ino` 에 없다 — 코드가 바뀌었으면 이 표를 먼저 고쳐라")
            continue
        refused = bool(with_ino(lambda s, o=old, n=new: s.replace(o, n, 1)))
        check("② %-22s 거부 (깨는 것: %s)" % (label, breaks), refused,
              "이 변이가 통과하면 6 행 축소의 fail-closed 가 거짓이다")

    print("\n[3] 게이트 헤더 변경도 여전히 거부되는가")
    saved = dict(wiring.GATE_SHA256)
    try:
        wiring.GATE_SHA256["rearm_gate.h"] = "0" * 64
        check("③ rearm_gate.h 변경 -> 거부",
              bool(wiring.gate_headers_unchanged()))
    finally:
        wiring.GATE_SHA256.clear()
        wiring.GATE_SHA256.update(saved)

    print("\n[4] 지문 표가 실제 `.ino` 함수와 1:1 인가")
    for name in wiring.INO_WIRING_SHA256:
        try:
            probe.function(source, name)
            check("④ %s 를 `.ino` 에서 뗄 수 있다" % name, True)
        except probe.ProbeError as error:
            check("④ %s 를 `.ino` 에서 뗄 수 있다" % name, False, str(error))

    print()
    if FAILURES:
        print("\033[31m%d 건 위반\033[0m" % len(FAILURES))
        return 1
    print("\033[32m전량 통과\033[0m — 축소 관문이 생략 행의 폐포를 잠근다")
    return 0


if __name__ == "__main__":
    sys.exit(main())
