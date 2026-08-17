# CURRENT_HANDOFF.md — 지금 할 한 묶음 (매 세션 시작점)

> 현재 묶음 + 미해결 보류만 유지한다. 이 파일의 완료 주장은 구현자의 주장이고,
> Tier A 최종판정은 반대쪽 AI가 한다(`AGENTS.md §5`).

- **동결 기준점**: `212885a8292d1e86677d5beeb9f5358d76fe9b40` =
  `platform-core-freeze-260724` (`FREEZE_MANIFEST`).
- **현재 기준**: main 작업 트리. 펌웨어는 아직 미굽기이며, §75 최종 독립 재검토의
  조건부 승인 뒤 정본·승인 지문·clean compile을 닫고 커밋하는 단계다.
- **현재 단계**: 5단 완료 → R0~R2·A-10·R5 지도 완료 → EKF NaN 가드 완료 →
  08-17 발현 주행에서 새 **전 telemetry 6.85초 침묵** 확인
  → **Claude 검토 §73 P0 0·P1 4·P2 3, 바퀴 공중 bench 조건부 승인**
  → **Claude 재검토 §74 P0 0·P1 2·P2 3, bench 조건부 승인 유지**
  → **§74.2·§74.3 보완 구현**
  → **Codex 별도 세션 최종 재검토 §75 P0 0·P1 2·P2 1, Tier A diff 커밋 가능**.
- **역할 교대**: 08-17 로봇 실험에서 발견되는 보완 구현 전반은 **Codex=구현자**,
  §73·§74는 **Claude=독립검토자**, §75 최종판정은 **구현자와 다른 Codex 세션=독립검토자**다.
  모델명이 아니라 구현자와 최종 승인 세션이 다르다는 불변조건을 지켰다. 현재 Codex 구현
  세션이 정본·커밋을 마무리하며, 역할 원위치는 오늘 로봇 실험·보완 사슬을 사용자가 종료한
  묶음 경계에서 한다.

## 이번 한 묶음 목표 — 검토 §75 D0-FW 조건부 승인 이관·커밋

08-17 구 15시간 계획의 H′·I·J와 R4는 일시중단했다. 순서 변경 정본은
`~/Desktop/0817_15시간_작업리스트.md` 맨 앞의 **08-17 14:55 긴급 순서 변경**이다.
오늘 전체 시간순 서사·원자료 위치·판단 변화 = `~/Desktop/개발현황/0817_현황.md`.

### 재현 증거

- bag: `~/Desktop/d0_evidence/drive_0817_1325` — **953.42초**, db3 189MB.
- kernel: `~/Desktop/d0_evidence/dmesg_drive_0817_1320.log`.
- 최대 공백: `/odom` **6.837961초**, `/imu/data` **6.851182초**.
  같은 사건에서 `/estop/state`·`/drive/enabled`·`/drive/diag`도 최대 **7.737초** 비었고
  `/scan`은 계속 왔다.
- ROS 수신시각 공백과 header stamp 공백이 수 ms 안에서 일치한다. 버퍼에 쌓였다 늦게 온
  것이 아니라 Teensy 주기 메시지가 그 시간 동안 생성되지 않았다.
- 🔴 kernel log는 13:21에 끝난 **주행 전 스냅샷**이다. 사건(13:30:22~29)
  시각은 커널 기록이 없어 USB reset·disconnect·`ttyACM` 오류 유무를 판정할 수 없다.
- 사용자는 바퀴 재장착·몸체 기울임이 그 사건 시각에는 없었다고 확인했다.

### 원인 분류

| 순위 | 후보 | 현재 판정 |
|---|---|---|
| 1 | micro-ROS **RELIABLE publish ACK 대기 연쇄** | 강함. timeout 1000ms인 호출들이 하나의 motor `loop()`에서 직렬 실행되고 관측 6.85초와 크기가 맞음 |
| 2 | loop가 ROS·IMU·안전표본·PWM을 모두 소유 | 확정된 안전 구조 결함. 한 호출이 늦으면 watchdog·E-stop 표본도 같이 늦음 |
| 3 | executor spin 반환값 무시(기존 검토 §56.1) | 확정. 응답 실패 뒤에도 ARMING 장벽을 시작할 수 있었음 |
| 4 | publish 실패·단계 지연 관측 부재 | 확정. 어느 호출이 늦거나 실패했는지 보드 밖에서 구분 불가 |
| 5 | BNO055 I2C 물림 | 보조 후보. Wire 한 전송 상한 50ms라 홀로 6.85초를 설명하지 못함. 예약 36 증상과 오늘 전 토픽 침묵은 모양도 다름 |
| 6 | time sync·USB CDC write | 보조 후보. 각각 100ms·120ms 상한이라 단독 원인 근거 없음 |
| 7 | USB 케이블·PCB·접점 | **미판정.** 08-17 dmesg가 사건 시각을 보지 못했다. Teensy 내부 정지와 USB/agent 링크 정지를 현 증거로 구분할 수 없음 |
| 8 | 바퀴 탈락·몸체 기울임 | 오늘 공백과 시각 불일치로 기각. 별도 기구 정비 이력일 뿐 |

🔴 **두 사건을 합치지 않는다.** 이번 보완은 08-17의 *전 telemetry 침묵*을 겨냥한다.
08-14의 *수신된 `/odom` 내용 손상·stamp 역행*이 이것으로 해결됐다고 주장하지 않는다.
R3와 예약 41의 물리 후보는 실차 재검증 전까지 열린 채다.

🔴 정지 직전 `/drive/diag x=2,y=0,z=2`(**ARMED**), E-stop=false였고 정지
구간 내내 `/cmd_vel wz=-1.0`이 계속 발행됐다. 6.84초 동안 watchdog·E-stop 표본이
돌지 않아 마지막 PWM을 유지할 수 있었다. 시급성의 핵심은 **감시 loop 전체 정지**다.

### Claude 재검토 §74 수용·보완 경계

- P0 0. 바퀴 공중·모터 전력 0V·물리 E-stop 담당자·clean compile 전제의
  bench만 조건부 승인했다. 실차 결함 종결은 불승인이다.
- P1 73.2: USB 기각 문장을 사건 시각 미관측으로 정정했다.
- P1 73.3: 250ms 강제값을 **미실측 임시 400ms**로 올렸다. 첫 bench 분포
  전에 최종 임계로 승격하지 않는다.
- P1 73.4: overrun 해제는 `/drive/diag y=7`·`disarm_runtime`을 남긴다.
- P1 73.5/P2-3: `REAL_ROBOT_VALUES §1-g`·`JETSON_SETUP §5-d`에 runtime 해독표와
  `runtime_overruns=overrun이 있었던 loop 수`를 고정했다.
- P2-1/P2-2: `FREEZE_MANIFEST §10.28`을 신설했고 state-bearing 진단은 선행
  publish의 해제 부작용 뒤에 다시 채운다.
- P1 74.2: spin 응답 실패 해제도 사유 **8**·`disarm_spin`을 남기고,
  `loop()` 안 사유 없는 `disarmDrive()` 호출을 0으로 고정했다.
- P1 74.3: runtime 관측 필드 **7개**의 존재·개수·enum 순서를 생산 소스에서
  직접 읽는 변이 회귀로 강제했다.
- P2 74.4: "320ms 상한 위"라는 과잉 주장을 취소했다. 생산 코드상 한 판에
  **최대 8 publish**가 겹칠 수 있으므로 400ms는 첫 bench 전까지 임시 후보다.
- P2 74.5·74.6: `src/tunnel_bringup` 3파일의 주석·docstring 전용 접촉을
  동결 예외 대장에 적고, `phase_max_us[6]`을 delay 제외 작업 시간으로 정정했다.

### Codex 별도 세션 최종 재검토 §75 조건부 수용

- **P0 0, 현재 Tier A diff 최종 승인·커밋 가능**. 승인 범위는 모터 전력 0V·바퀴 공중·
  물리 E-stop 담당자·승인 지문·clean compile 전제의 bench까지다.
- **P1 75.2 조건부 수용**: 실패한 executor spin이 400ms 이상이면 runtime 해제 사유 7이
  먼저 상태를 풀어 사유 8·`disarm_spin`이 남지 않을 수 있다. 모터는 DISARMED로 정지한다.
  enable 호출과 겹친 `y=7/disarm_runtime`은 spin 응답 실패를 배제하는 증거로 쓰지 않는다.
- **P1 75.3 조건부 수용**: 현행 enum 숫자 0~6·16~23은 생산 소스와 문서가 일치하지만,
  숫자 자체를 바꾸는 변이를 현재 회귀가 잡지 못한다. 자동 판독을 붙이기 전에는 header 숫자를
  수동 대조한다.
- **P2 75.4 닫힘**: 지속 `z=4`를 정상 fail-closed로 읽던 활성 런북 문장을 제거했다.
  현행 1.6.1의 spin 실패는 `z=0/y=8`이고, 지속 `z=4` 또는 진단 정지는 loop 정체 확인 대상이다.
- 같은 결함 사슬 3회차이므로 `AGENTS.md §6`에 따라 두 P1을 조건부 수용하고 동결한다.
  재개방은 enable 실패/timeout과 400ms overrun 동시 관측, 문서와 header 코드 불일치,
  또는 이 두 전제를 자동 판독에 쓰기 시작하는 시점이다. 임계를 올려 우회하지 않는다.

- **⚠ 미해결 보류 — 예약 23 / 검토 §33 (`8fcc1a2`)**: P0 0 · **P1 1** · P2 1.
  `handoff_single_check.sh` 는 `§` 없는 새 작업 꼬리를 붙여도 `OK` 를 내고, 숫자 계약 전수
  증거는 기록 6자리와 독립 실행 21자리가 불일치한다. **승인 완료로 읽지 말 것.** 런타임
  `src/**` 변경·회귀는 없으므로 현재 D+0 실행은 막지 않되, 보완은 다른 묶음에 섞지 않고
  `MASTER_PLAN.md §7` 예약 23 의 완료판정과 부정 회귀로 별도 구현한다. 전문 =
  `~/Desktop/개발현황/CODEX 현황/0801검토현황.md §33`.
- **예약 36·40·42·43** — IMU I2C 물림 · 각속도 전달률 · 아크릴 하중 · R3와 cmd_vel 현장
  확인은 이번 D0-FW 완료로 승격하지 않는다.

## 구현자가 주장하는 완료판정

> 주행 중 주기 발행 8개는 모두 BEST_EFFORT여서 1초 ACK 대기가 제어 loop에 연쇄되지 않고,
> 모든 발행과 loop 단계의 실행시간·실패가 관측되며, 한 단계 또는 전체 loop가 **임시
> 400ms 이상** 걸리면 복귀 즉시 `y=7`·DISARMED로 래치되어 새 무장 없이는 재가동하지
> 않는다. enable 서비스 응답을 처리한 executor spin이 실패하면 ARMING 장벽을 시작하지
> 않고 DISARMED로 끝난다. 단, 그 spin이 임시 경계 400ms도 함께 넘으면 현행 우선순위상
> `y=7`이 먼저 남아 `y=8`·`disarm_spin`이 보존되지 않을 수 있다. 기존 정상 re-arm·E-stop·
> watchdog·EKF 소비 경로는 유지된다.

⚠ 호출이 **영원히 반환하지 않는 동안**에는 소프트웨어가 E-stop 핀을 다시 읽을 수 없다.
이번 400ms guard는 복귀 직후 fail-closed이며 **첫 bench 계측용 임시값**이다.
하드웨어 watchdog 도입은 별도 bench 검증 없이
섞지 않는다.

## §3-10 전수 열거와 폐포

기억으로 고르지 않고 생산 코드와 소비자 파일을 기계 검색했다.

| 클래스 | 전수 | 구현 대조 |
|---|---:|---|
| 주기 publisher 초기화 | **8** | 전부 `rclc_publisher_init_best_effort` |
| 실제 `rcl_publish` 경로 | **8** | 전부 `publishMeasured` 한 경로, 우회 0 |
| executor spin | **1** | 시간 측정 + 실패 시 ARMING 해제 + 성공일 때만 장벽 시작 |
| 자동 해제 사유 | **5** | 4~8 전부 `disarmDriveWithReason`, spin=8·runtime=7 누계 분리 |
| runtime BNO055 `getEvent` | **2** | IMU 단계 시간에 포함 |
| 주기 time sync | **1** | 별도 단계 시간에 포함 |
| loop 단계 | **7** | spin·odom·IMU·diagnostics·firmware_info·time_sync·전체 loop |
| 직접 영향받는 현장 도구 구독 | **12** | 5파일 전부 `qos_profile_sensor_data` |
| bag telemetry override | **8** | 전부 BEST_EFFORT/VOLATILE |
| runtime info 계약 필드 | **7** | 존재·scalar/2개·phase 7·publish 8·enum 인자 순서 |

`tools/test_firmware_runtime_guard.py`는 다음 반례를 실제로 실패시킨다:

- publisher 한 자리 누락·새 자리 추가·BEST_EFFORT→RELIABLE;
- publish 측정 우회 또는 site 매핑 이탈;
- 임시 400ms 경계 `>=`→`>`;
- spin 성공/실패 조건 역전;
- spin 실패 사유 8→무사유 해제;
- runtime info 7필드 각각 삭제·배열 감소·phase/publish 순서 교환;
- 현장 구독 한 자리 QoS 역전;
- bag 8토픽 추가·누락/계약 불일치.

## 작업 트리 구현 범위

- `firmware/teensy_integrated_base_v1_4/runtime_guard.h` — Arduino 비의존 경계 로직.
- 같은 `.ino` — 1.6.1, telemetry 8개 BEST_EFFORT, publish/phase 계측(한 loop에서는
  가장 먼저 넘은 구체 원인만 `runtime_last`에 래치해 바깥 phase가 덮지 않음),
  임시 400ms 복귀 후 사유 7 disarm, spin 실패 사유 8 disarm,
  `/firmware/info` runtime 계수 7필드.
- `tools/test_firmware_runtime_guard.py` — 생산 코드·경계 변이·소비자 폐포 회귀. enum 선언
  순서는 검사하지만 숫자 wire code 변이는 §75.3 조건부 수용으로 남는다.
- 현장 스크립트 5파일·`d0_check.sh`·`bag_qos_overrides.yaml` — 소비 QoS 동기화.
- `readiness_gate.py`·`gate_fakes.py`·`ekf_real.yaml` — 이미 BEST_EFFORT인 런타임 계약 주석 정정.
- 정본: 이 파일, `MASTER_PLAN §7 41-e`, `REAL_ROBOT_VALUES §1-d`, `JETSON_SETUP §5-d·§7`.

🔴 사용자 선행 수정 4파일은 되돌리지 않는다:
`docs/D1_FIRST_STEP.md`, `docs/JETSON_SETUP.md`, `tools/todo_d0_scan.py`,
`tools/test_todo_scan.py`. `JETSON_SETUP`에는 겹치지 않는 절에 이 묶음을 얹었다.

## 현재 회귀와 남은 검증

완료된 호스트 검사:

- runtime-guard 생산 코드·변이 회귀 — **11/11 성공**.
- `python3 -m pytest tools/ -q` 전체 회귀 — §74 보완 후 **174/174 성공**.
- `firmware_info_length_check.py` — format/인자 **61/61**, 현재 소스 상한 **1407자**,
  1536 버퍼 여유 **128자**, overflow 변이 검출.
- re-arm 생산 경로 **989+11**, PWM epoch 해시는 host 989/0 후 갱신. 현장
  축소 6행 해시는 갱신하지 않아 굽기 후 13행 전량을 강제함.
- 변경 Python flake8(ament 99), `bash -n`, `py_compile`, `git diff --check` — rc=0.
- colcon 직접 실행 뒤 `test-result --verbose` — **0 errors · 0 failures · 3 skipped**.
  첫 실행의 11 실패는 sandbox가 `.ros/log` 쓰기를 거부한 인프라 실패였고,
  `ROS_LOG_DIR=/tmp/d0_fw_ros_logs`로 원인을 제거한 재실행에서 닫혔다.
- `test_firmware_precheck.sh` — §75 이관 뒤 **73/73 성공**. 실제 `firmware_precheck`도
  승인 소스 5개 내용 지문 일치·기대 밖 0건으로 **rc=0**이다.
- gate regression 첫 실행은 sandbox의 DDS 로컬 통신 차단으로 양성 입력부터 성립하지 않아
  **인프라 실패 3/14**로 분류했다. 격리 밖 단독 재실행은 **14/14 성공**했고,
  유한 상한 harness는 **24/24 성공**했다.
- `bash tools/doc_check.sh` — §74 시점 **rc=0**. §75 코퍼스 142 이관 뒤 재실행한다.

§75 승인 전에는 precheck가 새 `.ino`·`rearm_gate.h`·`runtime_guard.h`를 막아 rc=1이었다.
§75 승인 뒤 다섯 소스의 내용 지문을 이관했고 clean compile이 성공했다:
FLASH code/data/headers **295,072/86,500/8,568**, RAM1 variables/code **62,784/160,056**,
RAM2 **12,448**, 환경 지문 **10607/158**, build 기대값 `Aug 17 2026 20:35:06`.
보드에는 아직 업로드하지 않았다.

## 완료조건

호스트 구현 완료 기준은 아래와 같다.

1. `bash tools/doc_check.sh` rc=0.
2. `python3 -m pytest src/mission_manager/test/ -q` **184 passed** 기준선 유지.
3. `python3 -m pytest tools/ -q` **174개 전부 성공** 새 기준선 유지
   (mission pytest 184와 범위가 다른 `tools/` 전용 수치).
4. colcon **247 tests** 기준선 유지(`colcon test-result --verbose`로 판정).
5. re-arm harness **989+11** · harness guards **24 검사** · gate regression **14 케이스** 유지.
6. `§7-c-E` **13행 전량 수행** 계약은 독립검토 뒤 실차 사다리에서 수행하며, 구현 완료로
   미리 승격하지 않는다.
7. `JETSON_SETUP §9` `TODO(D+0)` **11건**은 종결 상태를 유지한다.

독립검토 §75는 다음을 직접 재산출했다.

1. bag SQL 재산출로 동시 공백을 재현하되, dmesg는 사건 시각 미관측임을 유지한다.
2. BEST_EFFORT 변경이 서비스 응답 QoS까지 잘못 약화하지 않았다.
3. 8 publish·7 phase·12 field subscription·8 bag topic·7 runtime info 폐포가 실제 생산 코드와 맞다.
4. `spin_some` 실패에서 ARMING이 남거나 장벽이 시작되는 경로가 없고,
   실제 해제는 `y=8`·`disarm_spin`을 남긴다.
5. 임시 400ms 경계·`micros()` wrap 산술·중복 disarm이 안전 상태를 역전하지 않는다.
6. `/firmware/info`가 잘리지 않고 새 계수의 순서·뜻이 맞다.
7. 정상 re-arm·watchdog·E-stop·EKF 역회귀가 유지된다.

§75의 `P0 0, 현재 Tier A diff 최종 승인·커밋 가능` 판정을 받았다. 두 P1은 위 전제·재개방
조건으로 수용하며, 사람/하드웨어 위험의 P0만 진행을 멈춘다는 규칙을 적용한다.

## 완료 판정

Codex 구현자의 완료 주장은 **보완 diff와 호스트 게이트가 끝났다는 것**까지이며,
§75 독립검토는 bench 한정 승인이지 실차 결함 종결이 아니다. `§7-c-E`
**13행 전량 수행**과 아래 사다리까지 통과하고,
장시간 bag에서 전 telemetry 수 초 동시 공백이 0이며 runtime 계수로 지연 원인을 설명할 수
있어야 08-17 침묵 사건을 닫는다. `TODO(D+0)` **11건** 계약은 그대로다.

## 완료 후 다음 단계 — §75 승인 뒤 실차 사다리 (업로드·실차는 아직 실행 금지)

1. clean build와 `FIRMWARE_REBUILD` precheck, 승인 blob 지문 이관.
2. 모터 전력 0V·바퀴 공중·E-stop 담당자 상태에서 업로드.
3. `/firmware/info`의 `build`·`version=…runtime-guard-1.6.1`·runtime 계수 확인.
4. E-stop 10/10, re-arm 배선/게이트 전 행, watchdog·무장 복귀 회귀.
5. 짧은 공중 저속 → 짧은 지면 저속 → 같은 bag 장시간 주행. 커널 로그는
   주행 종료 후 `dmesg -T`까지 저장해 사건 시각을 반드시 덮는다.
6. 첫 공중 무장 관측은 **최소 30초** 유지해 diagnostics·firmware_info·time sync
   주기가 겹치는 판을 포함하고, `runtime_overruns`·`runtime_last`·`publish_failures`·
   `disarm_runtime`·`disarm_spin`·phase/publish max를 시행 전후 대조.
7. 별도로 `bag_gap_report` R3와 08-14 message 손상/stamp 역행을 재판정.

`TODO(D+0)` **11건**은 다시 열지 않는다. 실제 모터 시험 명령을 줄 때는 사용자 요청문에
지정된 무장 확인 세 줄을 항상 같은 답변에 붙인다.

## 금지 범위

- self-approval·승인받은 Tier A 소스의 추가 변경·사용자 확인 없는 굽기 금지.
  §75 승인 뒤 지문 이관·clean compile·커밋 담당은 Codex.
- `CMD_WHEEL_BASE`, FF/게인, 지도 자산, `make_map.sh`, `rotate_to_heading`, 아크릴 장착 금지.
- R3 전 R6 진입 금지. 구 H′·I·J/R4는 D0-FW 독립검토와 실차 게이트 뒤 재배치.
- §72.2는 `drive_0817_1325` t=306.02의 `wz=-5.552` 통과로 **재개방**. 3×3
  가드 전수 보완은 D0-FW와 섞지 않고 별도 묶음으로 닫기 전 R4/R6 진입 금지.
- watchdog은 명령 단절 때 PWM만 0으로 만들고 ARMED를 유지한다 — H′(odom 가드 liveness)와
  다른 조건부 설계다. 자율 `/cmd_vel` 발행자가 붙기 전에 §56.1과 함께 다시 판단한다.
- 예약 36 IMU I2C 물림, 예약 40 각속도 전달률, 예약 42 아크릴 하중, 예약 43 R3·cmd_vel 현장
  확인, 예약 23·24·26·28~34는 여전히 보류다.
- `TODO(D+0)` **11건**을 추측값으로 다시 열거나 닫지 않는다.
- 08-14 라이다 z=`0.779` 동결. URDF를 열지 않는다. 라이다 TF 높이 관측값은 `0.832`.
- E-stop은 `Ctrl+C`가 아니다. 배선 작업 전 XT90 분리, 업로드는 모터 전력 0V·바퀴 공중.

## 진행표

| 트랙 | 상태 |
|---|---|
| **진행 중 묶음** | §75 D0-FW 조건부 승인 이관·커밋 | ✅ 최종 승인 · 정본/지문/clean compile/커밋 진행 |
| **D0-FW 독립검토** | ✅ §73 → §74 → §75(P0 0 · 조건부 승인 · 커밋 가능) |
| clean build·지문·굽기 | ✅ 지문 5개·clean build / 굽기 ⏸ 사용자 물리 승인 뒤 |
| 실차 사다리·장시간 재현 | ⏸ 굽기 뒤 |
| 08-14 stamp 손상/R3 | 🔴 D0-FW와 별도 판정 유지 |
| 구 15시간 H′·I·J/R4 | ⏸ D0-FW 뒤 재배치 |

## 근거 문서

- 증거·원인·예약 갱신 = `MASTER_PLAN §7` 예약 41-e.
- 펌웨어·소비 QoS 계약 = `REAL_ROBOT_VALUES §1-d` · `JETSON_SETUP §5-d`,`§7`.
- 재빌드·업로드 안전선 = `FIRMWARE_REBUILD §5-b` · `ELECTRICAL_BASELINE §7`,`§13`.
- loop·USB·I2C 함정 = `PITFALLS §11`,`§12`.
- 오늘 전체 서사·증거 = `~/Desktop/개발현황/0817_현황.md`.
