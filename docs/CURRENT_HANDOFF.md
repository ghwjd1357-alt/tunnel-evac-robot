# CURRENT_HANDOFF.md — 지금 할 한 묶음 (매 세션 시작점)

> 작업 완료 시 구현자가 갱신하고 검토자가 사실관계·테스트 결과를 확인한다.
> 이전 묶음 내용은 날짜별 현황으로 보내고, 이 파일에는 현재 작업과 직전 주장만 유지한다.

- **동결 기준점**: `212885a8292d1e86677d5beeb9f5358d76fe9b40` = tag
  **`platform-core-freeze-260724`**. 증거 = `docs/FREEZE_MANIFEST.md`.
- **현재 기준 커밋**: 이 파일과 같은 커밋(main).
- **현재 단계**: 마스터플랜 5단 완료 → 역할 B V1 합의(6단)는 회의 대기 보류
  → 실차 S6 준비·펌웨어 반영·검토 §29~§32 보완 완료
  → **검토 §32 승인 + 프로세스 보완 완료**(검토 §33 불승인은 예약 23으로 분리 보류)
  → **실차 D+0 인수·연결** **(지금 — 2026-08-03)**
- **⚠ 미해결 보류 — 예약 23 / 검토 §33 (`8fcc1a2`)**: P0 0 · **P1 1** · P2 1.
  `handoff_single_check.sh` 는 `§` 없는 새 작업 꼬리를 붙여도 `OK` 를 내고, 숫자 계약 전수
  증거는 기록 6자리와 독립 실행 21자리가 불일치한다. **승인 완료로 읽지 말 것.** 런타임
  `src/**` 변경·회귀는 없으므로 현재 D+0 실행은 막지 않되, 보완은 다른 묶음에 섞지 않고
  `MASTER_PLAN.md §7` 예약 23 의 완료판정과 부정 회귀로 별도 구현한다. 전문 =
  `~/Desktop/개발현황/CODEX 현황/0801검토현황.md §33`.
- **⏸ 6단 보류**: 역할 B 회의가 잡히면 커밋 `30c5e87`의 핸드오프 원문을 그대로 복원한다.

```bash
git show 30c5e87:docs/CURRENT_HANDOFF.md
```

## 직전 완료 — 검토 §34 보완 (구현자 = Codex, 독립 검토자 = Claude 예정)

사용자가 역할 교대를 명시하고 R0 watchdog **실측안**을 선택한 묶음이다. §34의 P1 2건·P2
4건을 사례가 아니라 같은 결함 클래스로 전수해 보완했다.

- 착수 때 Markdown fenced code 안에서 제목처럼 보이는 줄은 활성 문서 **17곳**이었다. watchdog
  명령 설명의 주석 1곳이 추가돼 최종 corpus는 **18곳**(`TEST_GATES` 1 + `JETSON_SETUP` 14 +
  `D1_FIRST_STEP` 3)이며, 스윕형 파서가 전부 자동으로 덮는다. 백틱·물결표·들여쓰기 fence를
  추적해 섹션 절단에서 제외하고, 빈 제목도 예외 없이 fail-safe 처리한다.
- 한 파일이 여러 프로필에 맞으면 전부 합친다. 알려진 P0/P1 앵커는 검토가 지목한 6개만
  아니라 공통 구현·검토 게이트에서 우연히 잡히던 같은 클래스 **10개**를 profile-only로
  바꿨다. 공통 문서만 남기면 **0/59**, 정상 프로필은 **59/59**다.
- `JETSON_SETUP §7-c`에 zero Twist 없이 publisher를 끝내는 R0 watchdog 실측을 추가했다.
  60fps 영상 30프레임 이하 + raw `/odom.pose` 정지를 증거로 쓰며 EMA twist 0 도달은 판정에서
  제외한다. PASS 기록이 없으면 R1 금지다. D+0 목록은 **11건**으로 갱신됐다.
- D+1 10개 항목의 실행 표식은 7→10으로 닫았고, 한 검사기가 D+0/D+1의 목록·개수 주장·표식
  증감 양쪽을 막는다. frozen `gate_fakes` 주기 값·실행은 불변이고, 3차 회신 뒤 R3까지
  의도적으로 동결했다는 출처만 정정했다.

구현자 회귀: AI context **27/27**, TODO scanner **5/5**, pytest **182/182**, gate-fakes
**34/34**, harness **24/24**, colcon **245/245**(0 failure, 3 skip), build·flake8·py_compile·
`doc_check` PASS. readiness 통합은 **13/14**로, 기존 공개 간헐 case 12가 150초 내 Nav2 준비를
못 했다. 값·분기 변경은 0이고 결정적 1층 20건과 나머지 13경로가 통과했으므로 기존 방침대로
PASS로 바꾸지 않고 판정 근거에서 제외한다. 예약 22·23은 미접촉이다.

**이 구현 diff는 Claude가 독립 재검토해 완료판정과 회귀 관찰값으로 승인해야 닫힌다.**

## ★ 이번 회차 최우선 확인 사항 — 현장 실행

1. **D+0 착수 전에** `JETSON_SETUP.md §3`의 HTTPS+PAT private clone을 실제 Jetson에서 통과한다.
   노트북 clone 성공은 Jetson 인증 증거가 아니다. 실패하면 USB 소스 복사 경로를 준비한다.
2. 사용자가 `docs/JETSON_SETUP.md §1`부터 순서대로 실행하고 출력을 붙여넣는다. AI는 결과를
   해석하고 코드/인프라를 분류한다. Jetson SSH 비밀번호는 AI가 입력하지 않는다.
3. `TODO(D+0)`를 만나면 넘기지 말고 실측값을 그 자리에 기록한다. 전량 목록은
   `JETSON_SETUP.md §9`의 11건이다.
4. EKF를 먼저 띄운 뒤 구동부가 있는 자리에서 `bash tools/d0_check.sh`를 실행한다.
5. 런북 순서·NTP·agent 먼저 기동 후 Teensy reset·부팅 뒤 8.7초 정지 조건을 지킨다.
6. 네트워크, `micro_ros_arduino` 라이브러리 폴더, 전원·충전, E-stop 인계를 현장에서 확인한다.

## 사용자 결정 — 유지

- 동결 예외 4회는 전부 소진됐다. `src/**`를 다시 열려면 사용자 승인이 필요하다
  (`FREEZE_MANIFEST.md §10`).
- 순서는 D+0 연결 → D+1 R3 rosbag 분석 → R4~R8이다(`MASTER_PLAN.md §3`).
- never-seen은 다른 놓침과 같이 역행 2회 → 보고 → 단독 탈출이다. 실차 R3 뒤 재평가한다.
- 예약 20·21·22·23과 역할 B 6단은 각각 정해진 트리거 전까지 별건으로 유지한다
  (`MASTER_PLAN.md §7`).

| 트랙 | 범위 | 상태 |
|---|---|---|
| **진행 중 묶음** | §32 후속: **실차 D+0 인수·연결** (`JETSON_SETUP.md`) | **착수** |
| AI 입력 최적화 | `ef25ad3` 기준, 품질 불변 + raw 입력 30% 절감 | §34 보완 완료·Claude 재검토 대기 |
| 실차 전 마무리 | clone 게이트·D0 번호·D0/D1 소스 발견·QoS 설명·정본 감사 | 구현 완료·Claude 검토 대기 |
| 예약 23 | 핸드오프 ID·숫자 계약 전수 폐포 | ❌ 별도 보류 |
| 역할 B V1 합의 | 커밋 `30c5e87` 원문 | ⏸ 회의 대기 |

## 이번 한 묶음 목표 — §32 후속: 실차 D+0 인수·연결

`JETSON_SETUP.md §1`→`§7`을 실제 Jetson·Teensy에서 실행하고, 확인한 `TODO(D+0)` 11건의 값을
`§9`에 기록한다. 새 절차를 설계하는 회차가 아니라 준비된 런북을 실행해 현실과 다른 곳을
고치는 회차다.

## 완료조건

1. Jetson에서 `bash tools/d0_check.sh`가 **종료 0**이다. 종료 2(불완전)는 통과가 아니다.
2. `JETSON_SETUP.md §9`의 `TODO(D+0)` 11건이 값과 함께 해소된다. 미해결은 이유와 재확인
   시점을 적는다.
3. 실제 장비와 다른 런북 문장은 그 자리에서 고친다.
4. 변경 표면에 맞는 `TEST_GATES.md §7` 검토 라우팅과 구현 회귀를 통과한다. 문서·셸만 바뀌면
   `doc_check.sh`+`bash -n`, 판정 도구를 바꾸면 합성 입력 부정·역회귀까지 직접 실행한다.
5. 현재 회귀 정본은 pytest **182 passed** / colcon **245 tests** / harness guards **24 검사** /
   gate regression **14 케이스**다. `colcon test-result --verbose`로 판정한다.

## 보존해야 할 안전 불변조건

- 설정은 대상 소프트웨어가 실제로 읽는지 `param list`·`topic info -v`로 확인한다.
- N초 관측은 같은 관측자가 종료 상태와 벽시계 경과시간으로 창을 완주해야 한다.
- 토픽별 계약의 단일 출처는 `tools/bag_gap_report.py`의 `TOPIC_POLICY`다.
- `/odom`은 RELIABLE, `/imu/*`는 BEST_EFFORT다. EKF 소비자 QoS까지 함께 본다.
- Teensy 재연결 로직 없음, IMU 치명 실패 시 노드 자체가 안 뜸, 시각 역행 시 stamp가 사실상
  멈춘다는 펌웨어 전제를 유지한다(`REAL_ROBOT_VALUES.md §1-d`).
- 실차 없이 확인한 합성 bag·가짜 `ros2` 결과를 실차 통과로 승격하지 않는다.

## 금지 범위

- 사용자 추가 승인 없는 `src/**` 변경.
- 예약 20·21·22·23, 역할 B V1, 관제 통보 표면, 깜빡임 원인 규명을 이 묶음에 혼합.
- `make_map.sh` 실런, 지도 자산 변경, Gazebo E2E 동시 실행.
- TODO를 추측값으로 닫기, `d0_check.sh` 종료 2를 PASS로 기록하기.
- Codex가 구현한 AI 입력 최적화 diff를 Codex가 최종 승인하기.

## 완료 판정

**Jetson에서 `d0_check.sh`가 종료 0이고 `TODO(D+0)` 11건이 값과 함께 기록되며, 실제와 달랐던
런북이 그 자리에서 수정됐으면 D+0 완료다.**

## 완료 후 다음 단계

1. `docs/D1_FIRST_STEP.md`의 D+1 R3 rosbag 분석.
2. R0 watchdog 실측으로 `FREEZE_MANIFEST.md §6` 조건부 수용 재판정.
3. D+0 속도 실측 뒤 예약 22와 R4~R8 진행.

## 근거 문서

- 실행 순서·TODO: `docs/JETSON_SETUP.md §1`, `§7`, `§9`
- 실차 값·펌웨어 전제: `docs/REAL_ROBOT_VALUES.md §1`, `§4`, `§5`
- 사다리·보류: `docs/MASTER_PLAN.md §3`, `§7`
- 변경 표면별 검토: `docs/TEST_GATES.md §7`
- AI 입력 최적화 계약: `docs/AI_CONTEXT.md §2`, `§3`, `§4`
