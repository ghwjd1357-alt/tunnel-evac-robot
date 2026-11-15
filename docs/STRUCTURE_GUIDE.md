# 코드 구조 가이드 — 어디에 무엇이 있나

> 공학프로그래밍 텀프로젝트 분석용 지도 (2026-11-15 작성, 기준 = 태그 `hanium-final`).
> **이 문서는 사실만 적는다.** 어디를 중점으로 설명하고 무엇을 뺄지는 적지 않았다 —
> 각자 분석한 뒤 만나서 정한다. 줄 수·줄 번호는 `hanium-final` 기준이다.

---

## 0. 이 문서를 읽는 법

- **GitHub 만으로 무엇을 볼 수 있고 무엇은 없는지 = §10.**
- 처음이면 **§1 그림 → §2 폴더 지도** 순서로 읽고, 그다음 맡은 쪽(§3 펌웨어 / §4 젯슨)으로 간다.
- 두 쪽이 만나는 지점은 **§5** 에 따로 모았다.
- 외부 코드·원작자 구분은 **§7** 에 있다. 과목 공지가 "외부 코드를 쓰면 시연 때 출처와
  아이디어가 들어간 부분을 밝히라"고 했으므로 그 확인용이다.
- 표의 `파일:줄` 은 그 줄로 바로 가면 된다. VS Code 라면 `Ctrl+P` 로 파일을 열고 `:줄번호`.

### 용어 다섯 개 (이것만 알면 나머지 문서가 읽힌다)

| 용어 | 뜻 |
|---|---|
| **노드 (node)** | ROS2 에서 실행되는 프로그램 하나. 예: 미션 노드, 라이다 드라이버 노드 |
| **토픽 (topic)** | 노드끼리 메시지를 주고받는 이름 붙은 통로. 예: `/cmd_vel` = 속도 명령 |
| **발행 / 구독** | 토픽에 메시지를 내보내는 쪽 / 받아 보는 쪽 |
| **런치 파일 (launch)** | 여러 노드를 한 번에 순서대로 켜는 파이썬 스크립트 |
| **micro-ROS** | 마이크로컨트롤러(Teensy)가 ROS2 토픽을 직접 주고받게 해 주는 라이브러리 |

---

## 1. 한 장 그림 — 로봇 한 대 안의 구성

```
                       ┌──────────────────────── Jetson Orin Nano (리눅스 + ROS2 Humble) ────────────────────────┐
 RPLIDAR C1 ──USB──▶   │ sllidar_node ──/scan──┬──▶ slam_toolbox(위치추정) ──/map, /tf──┐                          │
                       │                       │                                        ▼                          │
 Orbbec 카메라 ─USB─▶  │ (역할 B 인지) ──/detections──▶ adapter_node ──/person_status,/victim,/alarm──▶ mission_node │
                       │                                                                    │  (상태머신 12개)     │
                       │                                         Nav2 (경로계획·추종) ◀──goal(navigate_to_pose)─┘  │
                       │                                            │                                              │
                       │   robot_localization EKF ◀──/odom,/imu/data─┼──────────────┐                              │
                       │        └─/odometry/filtered─▶ (위치추정에 사용)│              │                              │
                       │                                            │/cmd_vel      │                              │
                       │                              micro_ros_agent (USB 시리얼 중계)                              │
                       └───────────────────────────────────────────┼──────────────▲──────────────────────────────┘
                                                                   ▼              │ /odom, /imu/data, /drive/diag …
                       ┌──────────────── Teensy 4.1 (펌웨어, C++/Arduino) ──────────────────────────────┐
                       │ cmd_vel → 바퀴별 목표속도 → PID → PWM ─▶ 모터드라이버 ─▶ DC 모터 ×4          │
                       │ 엔코더 ×4 ─▶ 오도메트리 계산 ─▶ /odom      BNO055 IMU ─▶ /imu/data           │
                       │ E-stop 버튼(21번 핀) · 0.5초 명령 끊김 워치독 · re-arm(무장) 게이트           │
                       └────────────────────────────────────────────────────────────────────────────────┘
 노트북/폰 브라우저 ◀──rosbridge(:9090)──▶ console/ (관제 웹: 지도·상태·진단 표시, 명령 전송)
```

- **젯슨**은 "어디로 갈지"를 정한다(지도·위치·경로·미션 판단). 결과는 속도 명령 `/cmd_vel` 하나다.
- **Teensy**는 "어떻게 바퀴를 돌릴지"를 정한다(속도→PWM, 엔코더→위치). 결과는 `/odom`·`/imu/data` 다.
- 둘은 USB 케이블 하나로 연결되고, 그 위를 micro-ROS 가 토픽으로 오간다(§5).
- 하드웨어 수치(바퀴 간격·지름·엔코더 PPR 등) 정본 = `docs/REAL_ROBOT_VALUES.md`.

---

## 2. 저장소 폴더 지도

| 폴더 | 들어 있는 것 | 언어 | 규모 | 만든 쪽 (§7) |
|---|---|---|---|---|
| `firmware/teensy_integrated_base_v1_4/` | Teensy 펌웨어 | **C++** (Arduino) | 6파일 · 약 3,300줄 | 구동부 원본 + 역할 A 수정 |
| `src/mission_manager/` | 미션 상태머신 (노드 1개 + 보조 모듈 3개) | Python | 약 3,200줄 | 역할 A |
| `src/perception_adapter/` | 인지 결과 → 미션 이벤트 변환 | Python | 약 1,500줄 | 역할 A |
| `src/tunnel_bringup/` | **실차** 런치·설정(yaml)·URDF·기동 게이트 | Python · YAML | 약 1,300줄 + 설정 7개 | 역할 A |
| `src/tunnel_interfaces/` | 커스텀 메시지 정의 2개 (`.msg`) | ROS IDL | 실행 코드 없음 | 역할 A·B 합의 |
| `src/tunnel_sim/` | **시뮬레이션**(Gazebo) 런치·월드·설정 | Python · XML | 약 900줄 | 역할 A |
| `src/sllidar_ros2/` | 라이다 드라이버 — ⚠ **GitHub 에는 없다**(§10) | **C++** | 69개 C++ 파일 | **외부** (Slamtec, BSD) |
| `src/my_first_pkg/` | ROS2 입문 연습 (talker/listener) | Python | 약 250줄 | 역할 A (학습용) |
| `console/` | 관제 웹 화면 + 로봇 디스플레이 화면 | JavaScript · Python | JS 17개 | 역할 A |
| `tools/` | 검사·측정·분석 스크립트 (테스트 36개 포함) | Python · Shell · C++ 2개 | 약 100개 | 역할 A |
| `maps/` | SLAM 으로 만든 지도 파일 | 데이터 | — | 실측 |
| `docs/` | 설계·측정·함정 기록 | Markdown | — | 역할 A |
| `docs/handover/0904_perception/` | 역할 B 인지 노드 **사본** + 측정 로그 | Python | — | 역할 B 원본 |

- `build/` · `install/` · `log/` 는 빌드 산출물이다(코드 아님, git 에 없음).
- 저장소에 **없는** 외부 패키지(설치해서 씀): Nav2, slam_toolbox, robot_localization(EKF),
  micro_ros_agent, rosbridge, OrbbecSDK_ROS2. 저장소에는 이들의 **설정 파일**만 있다(§4-6).

---

## 3. 펌웨어 지도 (Teensy 4.1 · C++)

### 3-1. 파일

| 파일 | 줄 | 하는 일 |
|---|---|---|
| `teensy_integrated_base_v1_4.ino` | 2,076 | 본체. 핀·상수 정의, `setup()`, `loop()`, 모터·오도메트리·IMU·통신 전부 |
| `rearm_gate.h` | 308 | 무장(re-arm) 상태 전이 — 꺼짐/무장 중/무장됨. 서비스 요청으로만 무장 |
| `drive_wiring.h` | 114 | "무장 해제로 바뀌면 모터를 세운다"를 한 동작으로 묶은 것 |
| `estop_debounce.h` | 140 | E-stop 입력 떨림(글리치) 걸러내기 + 관측 카운터 |
| `runtime_guard.h` | 134 | `loop()` 한 바퀴가 너무 오래 걸리면 무장 해제 |
| `link_stall_probe.h` | 548 | USB 링크 멈춤 사건을 재고 `/firmware/event` 로 보고 |
| `SHA256SUMS.txt` | 1 | 원본 파일 해시(원본과 같은지 대조용) |

- `.h` 다섯 개는 **Arduino 함수를 쓰지 않게** 짜여 있어서 노트북에서 `g++` 로 따로 컴파일해 시험한다(§3-6).
- 빌드 환경(Arduino IDE/Teensyduino 버전·라이브러리·패치) = `docs/FIRMWARE_REBUILD.md`.

### 3-2. 실행 흐름

**`setup()`** — `.ino:1739` (전원을 켜면 한 번)

1. 핀 설정: LED, E-stop(21번, 풀업 입력), PWM 해상도 8비트, 모터 핀
2. 모터 정지 + 무장 게이트 초기화(`rearmGateInit`) — 켜자마자 바퀴가 돌지 않는다
3. IMU 초기화(`initializeImu`) + 자이로 바이어스 보정(`calibrateGyroBias`)
4. micro-ROS 연결(`set_microros_transports`) → 노드 `teensy_integrated_base` 생성
5. 구독 3 · 발행 11 · 서비스 1 등록 (§3-4), 메시지 초기화(frame_id, 공분산)
6. 실행기(executor) 준비

**`loop()`** — `.ino:1991` (계속 반복, 끝에 `delay(1)`)

```
checkSafety()                 안전 점검 ①
rclc_executor_spin_some(2ms)  들어온 토픽·서비스 처리 → 여기서 cmdVelCallback 등이 불린다
(무장 장벽 처리)               rearm 관련
checkSafety()                 안전 점검 ②
updateAppliedPwmEpoch()
updateMotorOutputs()          목표 PWM 쪽으로 천천히 이동(램프) 후 출력
updateOdometry()              엔코더 → 위치·속도 → /odom (20 ms 주기)
updateImu()                   BNO055 → /imu/data (20 ms 주기)
publishDiagnostics()          /drive/diag (1초 주기)
publishFirmwareInfo()         /firmware/info (5초 주기)
periodicTimeSync()            젯슨과 시계 맞추기 (30초 주기)
publishFirmwarePulse() · drainLinkEvents()   생존 신호 · 링크 사건 보고
```

### 3-3. 함수 지도 (`.ino`, 기능별)

| 묶음 | 함수 (줄) |
|---|---|
| **속도 명령 → 바퀴** | `cmdVelCallback`(1364) → `applySkidSteerCommand`(739, 직진·회전 → 좌우 바퀴 속도) → `setTargetWheelVelocity`(703) |
| **바퀴 속도 제어 (PID)** | `updateWheelControllers`(762) · `wheelVelocityToFeedforwardPwm`(681) · `resetWheelControllerState`(655) |
| **PWM 출력** | `updateMotorOutputs`(907) · `movePwmTowardTarget`(866, 램프) · `writeMotorPwm`(638) · `stopAllMotors`(669) |
| **오도메트리** | `readEncoderCount`(1056) · `updateOdometry`(1089) · `publishOdometry`(1168) · `resetOdometry`(1061) |
| **IMU** | `initializeImu`(1196) · `calibrateGyroBias`(1218) · `updateImu`(1270) · `publishImu`(1327) · `applyGyroDeadband`(588) |
| **안전** | `checkSafety`(1022) · `isEstopActive`(607) · `updateEstopFilter`(597) · `disarmDrive`(965) · `disarmDriveWithReason`(971) · `driveEnableCallback`(1398, 무장 서비스) |
| **진단·시간** | `publishDiagnostics`(1440) · `publishFirmwareInfo`(1483) · `publishFirmwarePulse`(1631) · `drainLinkEvents`(1672) · `periodicTimeSync`(1700) · `recordRuntimePhase`(976) |
| **기타** | `resetOdomCallback`(1416) · `resetYawCallback`(1426) · `clampDouble`(574) · `normalizeAngle`(581) · `errorLoop`(1722) |

### 3-4. 펌웨어가 주고받는 토픽

| 방향 | 이름 | 타입 | 원본에 있었나 |
|---|---|---|---|
| 구독 | `cmd_vel` | Twist | ✅ |
| 구독 | `reset_odom` · `imu/reset_yaw` | Bool | ✅ |
| 발행 | `odom` | Odometry | ✅ |
| 발행 | `imu/data` · `imu/yaw_deg` · `imu/gyro_bias` | Imu · Float64 · Vector3 | ✅ |
| 발행 | `estop/state` | Bool | ✅ |
| 발행 | `firmware/info` | String | ✅ |
| 발행 | `drive/enabled` · `drive/diag` | Bool · Vector3 | 추가 |
| 발행 | `firmware/pulse` · `firmware/event` | String | 추가 |
| 서비스 | `drive/enable` | SetBool | 추가 |

### 3-5. 주요 상수 (`.ino`)

| 상수 | 값 | 줄 |
|---|---|---|
| `PWM_PIN` / `DIR_PIN` | {2,4,6,8} / {3,5,7,9} | 104–105 |
| `ESTOP_PIN` | 21 | 136 |
| `IMU_SDA_PIN` / `IMU_SCL_PIN` | 18 / 19 | 369–370 |
| `TOTAL_PPR` (바퀴 한 바퀴당 엔코더 펄스) | 2641.1 | 150 |
| `ODOM_WHEEL_RADIUS` | 0.05698 m | 179 |
| `MAX_LINEAR_CMD` / `MAX_ANGULAR_CMD` / `MAX_WHEEL_CMD` | 0.20 m/s / 0.83 rad/s / 0.30 m/s | 246–248 |
| `WHEEL_KP` / `KI` / `KD` | 60.0 / 5.0 / 0.0 | 314–316 |
| `FEEDFORWARD_MAX_PWM` / `MAX_CONTROL_PWM` | 145 / 160 | 295, 298 |
| `WATCHDOG_TIMEOUT_MS` (명령 끊기면 정지) | 500 | 330 |
| `ODOM_PERIOD_US` / `IMU_PERIOD_US` | 20,000 (= 50 Hz) | 333, 382 |

🔴 속도 상한(`MAX_*_CMD`)은 전기 안전 검증과 묶여 있어 올리지 않는다(`docs/CURRENT_HANDOFF.md` 금지 범위).

### 3-6. 보드 없이 시험하는 곳 (`tools/`)

| 파일 | 하는 일 |
|---|---|
| `tools/rearm_gate_host_test.cpp` + `.sh` | `rearm_gate.h` 를 노트북 `g++` 로 컴파일해 상태 전이 시험 |
| `tools/link_stall_host_test.cpp` + `.sh` | `link_stall_probe.h` 를 같은 방식으로 시험 |
| `tools/test_firmware_runtime_guard.py` | `runtime_guard.h` 경계 시험 |
| `tools/firmware_constants.py` | `.ino` 에서 상수를 읽어 오는 도구(다른 도구들이 씀) |
| `tools/firmware_precheck.sh` | 굽기 전 점검(원본 해시·변경 범위) |

### 3-7. 원본과 이후 수정 구분

- 원본 = 구동부 파트가 만든 `.ino` 한 파일(**1,307줄**). 2026-08-05 커밋 `f57d454` 로 저장소에 들어왔다.
- 이후 `hanium-final` 까지: `.ino` 가 **2,076줄**이 됐고 `.h` 다섯 개(합계 1,244줄)가 새로 생겼다.
- 원본에 없던 함수: `disarmDrive` · `disarmDriveWithReason` · `driveEnableCallback` · `updateEstopFilter` ·
  `updateAppliedPwmEpoch` · `recordRuntimePhase` · `currentEpochMs` · `publishFirmwarePulse` · `drainLinkEvents`.
- 정확히 무엇이 바뀌었는지 보는 명령:

```bash
git show f57d454:firmware/teensy_integrated_base_v1_4/teensy_integrated_base_v1_4.ino > /tmp/original.ino
git diff f57d454 hanium-final -- firmware/teensy_integrated_base_v1_4/
```

- 수령 경위·소유 경계 = `firmware/VENDOR_DROP.md`.

---

## 4. 젯슨 지도 (Jetson Orin Nano · ROS2 Humble)

### 4-1. 실차에서 실제로 켜지는 노드

`ros2 launch tunnel_bringup real_bringup.launch.py` 한 줄이 아래를 켠다(`src/tunnel_bringup/launch/real_bringup.launch.py`).

| 노드 | 패키지 | 우리 코드? | 하는 일 |
|---|---|---|---|
| `robot_state_publisher` | 외부 (ROS 기본) | 설정만 (`urdf/robot_real.urdf`) | 로봇 부품 위치 관계(TF) 발행 |
| `sllidar_node` | `src/sllidar_ros2` | ❌ 외부 (Slamtec) | 라이다 → `/scan` |
| `micro_ros_agent` | 외부 | ❌ | USB 로 Teensy 와 토픽 중계 |
| `ekf_node` | 외부 (robot_localization) | 설정만 (`ekf_real.yaml`) | `/odom` + `/imu/data` 융합 |
| `localization_slam_toolbox_node` | 외부 (slam_toolbox) | 설정만 | 저장된 지도 위에서 위치 추정 |
| Nav2 일체 | 외부 (Nav2) | 설정만 (`nav2_params_real*.yaml`) | 경로 계획·추종 → `/cmd_vel` |
| `readiness_gate` | `src/tunnel_bringup` | ✅ | 센서·위치추정이 준비됐는지 확인 후 다음 단계 기동 |
| `mission_node` | `src/mission_manager` | ✅ | 미션 상태머신 |
| `adapter_node` | `src/perception_adapter` | ✅ | 인지 결과 해석 (별도 런치 `adapter.launch.py`) |

- 기동 순서는 `readiness_gate` 가 단계별로 확인하며 진행한다(센서 → 카메라(선택) → 위치추정 → Nav2 → 미션).
  `camera:=true` 인자를 주면 카메라 depth 스트림 확인이 단계에 들어간다.
- 지도 제작은 다른 런치: `real_mapping.launch.py`.

### 4-2. `mission_manager` — 미션 상태머신

| 파일 | 줄 | 하는 일 |
|---|---|---|
| `mission_node.py` | 1,907 | 노드 본체. 상태머신·콜백·타이머 |
| `goal_manager.py` | 522 | Nav2 목표 전송·취소의 수명주기 관리 |
| `speed_manager.py` | 530 | 상태별 주행 속도를 Nav2 파라미터로 바꾸기 |
| `follower_monitor.py` | 259 | 뒤따라오는 사람을 라이다로 감시(놓침·재발견 판정) |
| `config/waypoints*.yaml` | — | 경유점·집결지·속도 등 시나리오 값 (실차 `_real_H*`, 시뮬 `waypoints.yaml`) |

**상태 12개** — `mission_node.py:75` `class State`

```
PATROL(순찰) → APPROACH(집결지로 이동) → SCAN_AREA(제자리 회전 탐색) → GATHER(집결 대기)
 → GUIDE(선두 유도) ⇄ HOLD(놓침 직후 정지) → SEARCH_BACK(뒤로 돌아가 재탐색) → ESCAPED(탈출 완료)
 그 밖: RESCUE(쓰러진 사람 발견 → 정지·신고) · NO_VICTIM(아무도 없음) · FAULT(Nav2 실패) · BLOCKED(안전 정지)
```

**주요 함수** (`mission_node.py`)

| 묶음 | 함수 (줄) |
|---|---|
| 시작·타이머 | `__init__`(561) — `tick` 0.5초(2 Hz) · `safety_watchdog` 0.5초 |
| 상태머신 본체 | `tick`(784) — 상태마다 무엇을 할지 |
| 상태 진입 | `enter_scan_area`(1352) · `enter_rescue`(1438) · `enter_no_victim`(1443) · `enter_hold`(1448) · `enter_search_back`(1467) · `enter_fault`(1828) · `_enter_guide_lowspeed`(1257) |
| Nav2 목표 | `send_goal`(730) · `cancel_current_goal`(735) · `on_reached`(746) · `scan_goal`(1363) |
| 입력 콜백 | `on_scan`(1559) · `on_alarm`(1563) · `on_person_status`(1269) · `on_victim`(1273) · `on_cmd`(1735) |
| 사람 판정 | `person_verdict`(1291) · `fresh_person_status`(1281) · `record_last_seen`(1532) |
| 안전 정지 | `_confirmed_stop`(1376) · `on_safety_stop_confirmed`(1720) · `safety_watchdog`(1801) |
| 설정 검증 | `need` · `need_num` · `need_int` · `need_bool` (374~439) — yaml 값 검사 |

**토픽**: 구독 `/scan` `/alarm` `/victim` `/person_status` `/mission_cmd` · 발행 `/mission_state` `/siren` ·
액션 클라이언트 `navigate_to_pose`(Nav2).

### 4-3. `perception_adapter` — 인지 → 미션 연결

| 파일 | 줄 | 하는 일 |
|---|---|---|
| `adapter_node.py` | 1,195 | `/detections` 를 받아 검증·누적 판정 → `/alarm`(화재) · `/person_status` · `/victim` 발행 |
| `fake_detections.py` | 203 | 카메라 없이 가짜 `/detections` 를 내는 시험 노드 |
| `launch/adapter.launch.py` | 113 | 어댑터 기동 인자 |

주요 구성: `ConfirmTracker`(124, N번 연속 확인) · `validate_params`(224) · `pick_best`(294) ·
`PerceptionAdapter.on_detections`(991) · `_person_frame_verdict`(809) · `_person_to_map`(769, 카메라 좌표 → 지도 좌표).
판정 규칙·상수의 근거 = `docs/PROJECT_CONTEXT.md` §4.1-b.

### 4-4. `tunnel_interfaces` — 메시지 정의

- `Detection3D.msg`: `class_name`(사람 자세·화재 구분도 이 이름으로) · `confidence` · `bbox` · `position`
- `Detection3DArray.msg`: `header` + `Detection3D[]`
- 역할 B(인지)와 역할 A(어댑터) 사이 약속. 계약 전문 = `docs/PROJECT_CONTEXT.md` §4.1.

### 4-5. `tunnel_bringup` · `tunnel_sim`

| 위치 | 내용 |
|---|---|
| `tunnel_bringup/launch/real_bringup.launch.py` (433줄) | 실차 운행 전체 기동 |
| `tunnel_bringup/launch/real_mapping.launch.py` (194줄) | 실차 지도 제작 |
| `tunnel_bringup/tunnel_bringup/readiness_gate.py` (565줄) | 단계별 준비 확인 노드 |
| `tunnel_bringup/urdf/robot_real.urdf` | 실차 치수·센서 위치 |
| `tunnel_sim/worlds/*.world` | Gazebo 터널 월드 |
| `tunnel_sim/launch/*.launch.py` (7개) | 시뮬 로봇·SLAM·Nav2·미션 기동 |
| `tunnel_sim/tunnel_sim/fake_follower.py` | 시뮬에서 따라오는 사람 흉내 |

### 4-6. 설정 파일 (외부 패키지의 동작을 정하는 곳)

| 파일 (`src/tunnel_bringup/config/`) | 대상 |
|---|---|
| `nav2_params_real.yaml` · `nav2_params_real_db.yaml` | Nav2 (속도·허용오차·장애물 여유 등) |
| `ekf_real.yaml` · `ekf_real_db.yaml` | EKF (어떤 센서 값을 융합할지) |
| `slam_real_mapping.yaml` · `slam_real_localization.yaml` | slam_toolbox |
| `lidar_standalone.yaml` | 라이다 단독 시험 |

시뮬 쪽 같은 역할의 파일 = `src/tunnel_sim/config/`.

---

## 5. 두 영역의 경계 — 젯슨 ↔ Teensy

| 방향 | 토픽 | 젯슨 쪽 상대 | 펌웨어 쪽 함수 |
|---|---|---|---|
| 젯슨 → Teensy | `/cmd_vel` | Nav2 (velocity_smoother) | `cmdVelCallback` |
| 젯슨 → Teensy | `/drive/enable` (서비스) | 사람이 `ros2 service call` 로 호출 (현장 시험 도구 `tools/rearm_*` 도) | `driveEnableCallback` |
| 젯슨 → Teensy | `/reset_odom` · `/imu/reset_yaw` | 사람이 명령으로 | `resetOdomCallback` · `resetYawCallback` |
| Teensy → 젯슨 | `/odom` · `/imu/data` | EKF | `publishOdometry` · `publishImu` |
| Teensy → 젯슨 | `/estop/state` · `/drive/enabled` · `/drive/diag` | 관제 화면 | `publishDiagnostics` 등 |
| Teensy → 젯슨 | `/firmware/info` · `/firmware/pulse` · `/firmware/event` | 관제·분석 도구 | `publishFirmwareInfo` 등 |

- 연결: USB 시리얼 → 젯슨의 `micro_ros_agent` 가 받아서 일반 ROS2 토픽으로 풀어 준다.
- 정지 경로는 두 겹이다: 젯슨이 0 속도를 보내는 것과, Teensy 가 **0.5초 동안 명령이 안 오면 스스로 멈추는 것**(`WATCHDOG_TIMEOUT_MS`).

---

## 6. 그 밖의 폴더

### `console/` — 관제 웹

- 실행: `console/run_console.sh` → rosbridge(포트 9090) + 웹서버(포트 8000). 사용법 = `console/README.md`.
- `index.html` + `js/` 17개: `ros.js`(토픽 연결) · `map.js`(지도) · `mission.js`(상태) · `diag.js`(펌웨어 진단 해독) ·
  `emergency.js`(비상) · `display.js`(로봇 디스플레이 모드) 등.
- 구독하는 토픽 대부분이 §4·§5 의 토픽이다(`/map` `/scan` `/mission_state` `/drive/diag` …).
- ⚠ 화면 일부는 시연용 MOCK(가짜 값)이다 — 카메라·탐지목록·배터리·Jetson 부하. 화면에 `데모 데이터` 로 표시된다.

### `tools/` — 검사·측정 스크립트

| 이름 앞부분 | 개수 | 대략 |
|---|---|---|
| `test_*` | 36 | 자동 테스트 |
| `drive_*` | 7 | 구동부 측정·보고 |
| `rearm_*` | 6 | 무장 게이트 현장 시험 |
| `link_*` · `bag_*` | 4 · 4 | 링크 멈춤·rosbag 분석 |
| `viz/` | — | rosbag → 영상 렌더러 (`tools/viz/README.md`) |

### 테스트 위치

| 위치 | 대상 |
|---|---|
| `src/mission_manager/test/` (8파일) | 미션 상태머신 |
| `src/perception_adapter/test/` (5파일) | 어댑터 |
| `src/tunnel_bringup/test/` (7파일) · `src/tunnel_sim/test/` (3파일) | 런치·설정 |
| `tools/test_*` | 도구·펌웨어 헤더 |

실행법 = `docs/TEST_GATES.md`.

---

## 7. 출처·작성자 구분 (사실 기록)

| 대상 | 작성 | 근거 |
|---|---|---|
| 펌웨어 원본 (`.ino` 1,307줄) | 구동부 파트 | `firmware/VENDOR_DROP.md`, 커밋 `f57d454` |
| 펌웨어 이후 수정 (`.ino` 증가분 + `.h` 5개) | 역할 A | `git log -- firmware/` |
| `src/` 의 mission·adapter·bringup·sim·interfaces | 역할 A (interfaces 는 역할 B 와 합의) | `git log` |
| 인지 노드 (RF-DETR 화재 · YOLO 자세 + depth) | 역할 B — 저장소에는 사본만 | `docs/handover/0904_perception/` |
| `src/sllidar_ros2` | Slamtec (외부, BSD 라이선스) | https://github.com/Slamtec/sllidar_ros2 (커밋 `3430009`) |
| Nav2 · slam_toolbox · robot_localization · micro-ROS · rosbridge | 외부 오픈소스 (설치형) | 저장소에는 설정만 |
| 관제 웹의 외부 라이브러리 | 외부 | `console/vendor/` |
| AI 도구 사용 | 커밋 378개 중 344개에 `Co-Authored-By: Claude` 표기, 검토에 Codex 사용 | `git log --grep='Co-Authored-By'` |

---

## 8. 분석할 때 쓰는 명령

```bash
# 어떤 함수·토픽이 어디서 쓰이나
grep -rn "cmd_vel" src/ firmware/

# 이 파일은 언제 왜 바뀌었나 (커밋 메시지에 이유가 적혀 있다)
git log --oneline -- src/mission_manager/mission_manager/mission_node.py

# 이 줄을 누가 언제 바꿨나
git blame -L 75,110 src/mission_manager/mission_manager/mission_node.py

# 한이음 최종본 그대로 보기 (나중에 코드가 바뀌어도)
git show hanium-final:firmware/teensy_integrated_base_v1_4/teensy_integrated_base_v1_4.ino
```

로봇(또는 시뮬)이 켜져 있을 때:

```bash
ros2 node list           # 지금 실행 중인 노드
ros2 topic list          # 지금 있는 토픽
ros2 topic echo /mission_state   # 토픽 내용 실시간 보기
rqt_graph                # 노드·토픽 연결을 그림으로
```

---

## 9. 더 깊이 볼 때

| 궁금한 것 | 문서 |
|---|---|
| 시스템 구조·토픽 약속·하드웨어 값 | `docs/PROJECT_CONTEXT.md` |
| 실측값 (바퀴·엔코더·IMU·속도) | `docs/REAL_ROBOT_VALUES.md` |
| 전기계통·E-stop 배선 | `docs/ELECTRICAL_BASELINE.md` |
| 펌웨어 빌드·굽기 | `docs/FIRMWARE_REBUILD.md` · `firmware/VENDOR_DROP.md` |
| 젯슨 설치·환경 | `docs/JETSON_SETUP.md` |
| 영역별 함정 | `docs/PITFALLS.md` |
| 한이음 진행 기록·결정 이유 | `docs/MASTER_PLAN.md` (읽기 전용) |
| 관제 웹 | `console/README.md` |

---

## 10. GitHub 에 있는 것 / 없는 것

**코드를 읽고 분석하는 데는 GitHub 저장소만으로 충분하다** — 우리가 쓴 코드·펌웨어·설정·문서·커밋 이력이
전부 있고, 저장소는 공개라 로그인 없이 열린다. 아래는 **저장소에 없는 것**과 구하는 곳이다.

| 없는 것 | 왜 없나 | 구하는 곳 |
|---|---|---|
| 라이다 드라이버 `src/sllidar_ros2/` | 외부 코드라 `.gitignore` 로 제외 | `git clone https://github.com/Slamtec/sllidar_ros2.git` → `git checkout 3430009` (우리가 쓴 판, 2024-06-17) |
| 펌웨어가 쓰는 라이브러리 | 외부 코드 | `micro_ros_arduino` 2.0.8-humble · `Adafruit_BNO055` 1.6.4 · `Adafruit_BusIO` 1.17.4 · `Adafruit_Unified_Sensor` 1.1.15 (각 이름으로 GitHub 검색) · `Encoder` 1.4.3 = Teensyduino 1.58.2 동봉. 굽기까지 하려면 `docs/FIRMWARE_REBUILD.md` |
| Nav2 · slam_toolbox · robot_localization · micro_ros_agent · rosbridge | 설치형 외부 패키지 | 각 프로젝트 GitHub (`ros-navigation/navigation2`, `SteveMacenski/slam_toolbox`, `cra-ros-pkg/robot_localization` 등). 저장소엔 우리 **설정 파일**만 |
| 역할 B 인지 코드의 **원본** | 역할 B 소유 | 저장소엔 **사본** `docs/handover/0904_perception/perception_node_0904.py` 만. 원본은 역할 B |
| 젯슨에서만 고친 내용 | 젯슨 로컬 변경 | 목록·원복법 = `docs/handover/0904_perception/README.md` |
| 실차 운행 지도 | 젯슨에만 있음 | 젯슨 `~/Desktop/maps_0820/`. 저장소 `maps/` 는 **시뮬** 지도 |
| 주행 기록(rosbag)·시연 영상 | 용량(약 2.6 GB) | 사용자 노트북 `~/robot_evidence/` |
| 관제 화면 데모 영상 클립 | 용량 | 사용자 노트북 `console/media/` |
| 날짜별 개발 기록·검토문 | 다른 저장소 | 문서 곳곳의 `~/Desktop/…` 경로 = 사용자 노트북의 기록. **결정 이유는 커밋 메시지에도 있다** (§8 `git log`) |

**실행까지 해 보려면**(읽기만이면 필요 없음): Ubuntu 22.04 + ROS2 Humble + Gazebo Classic 11 이 있어야 하고,
위 외부 패키지를 설치한 뒤 `colcon build --symlink-install`. 실차 없이 돌리는 방법 = `docs/TEST_GATES.md`.

