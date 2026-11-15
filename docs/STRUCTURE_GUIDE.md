# 코드 구조 가이드 — 어디에 무엇이 있나

> 공학프로그래밍 텀프로젝트 분석용 지도 (2026-11-15 작성, 기준 = 태그 `hanium-final`).
> **이 문서는 사실만 적는다.** 어디를 중점으로 설명하고 무엇을 뺄지는 이 문서의 범위 밖이다.
> 줄 수·줄 번호는 `hanium-final` 기준이다.

---

## 0. 이 문서를 읽는 법

- 절 구성: §1 전체 그림 · §2 폴더 지도 · §3 펌웨어 · §4 젯슨 · §5 두 영역의 경계 ·
  §6 관제·도구·테스트 · §7 출처 · §8 명령 · §9 관련 문서 · §10 GitHub 에 있는 것/없는 것.
- 표의 `파일:줄` 은 그 파일의 그 줄이다.
  - **GitHub 웹**: 파일을 열고 주소 끝에 `#L123` 을 붙이면 123번째 줄로 간다(범위는 `#L75-L110`).
  - **clone 해서 VS Code**: `Ctrl+P` 로 파일을 열고 `:123`.
- 과목 공지가 "외부 코드를 쓰면 시연 때 출처와 아이디어가 들어간 부분을 밝히라"고 했다. 원작자·외부 코드 구분은 §7 에 있다.

### 팀 안에서 쓰던 이름

| 이름 | 뜻 |
|---|---|
| **역할 A** | 젯슨 쪽 담당(SLAM·내비·미션·관제) |
| **역할 B** | 카메라 인지 담당(사람·화재 탐지) |
| **구동부 파트** | Teensy 펌웨어·모터·배선 담당 |

### 용어

| 용어 | 뜻 |
|---|---|
| **노드 (node)** | ROS2 에서 실행되는 프로그램 하나. 예: 미션 노드, 라이다 드라이버 노드 |
| **토픽 (topic)** | 노드끼리 메시지를 흘려보내는 이름 붙은 통로. 예: `/cmd_vel` = 속도 명령. 보내는 쪽 = 발행, 받는 쪽 = 구독 |
| **서비스 (service)** | 요청 한 번 → 응답 한 번. 예: `/drive/enable` (무장 요청) |
| **액션 (action)** | 오래 걸리는 요청 + 진행 상황 + 취소. 예: `navigate_to_pose` (Nav2 에 "여기로 가라") |
| **TF** | 좌표계 사이의 위치 관계. 예: `map → odom → base_footprint`(로봇 바닥 중심) |
| **런치 파일 (launch)** | 여러 노드를 한 번에 켜는 파이썬 스크립트 |
| **micro-ROS** | 마이크로컨트롤러(Teensy)가 ROS2 토픽·서비스를 직접 쓰게 해 주는 라이브러리 |
| **EKF** | 확장 칼만 필터. 바퀴 오도메트리와 IMU 를 합쳐 위치를 추정한다(`robot_localization` 패키지) |
| **SLAM / 위치추정** | 라이다로 지도를 만들거나(SLAM), 만든 지도 위에서 자기 위치를 찾는 것 |
| **Nav2** | ROS2 표준 내비게이션 스택. 목표를 받으면 경로를 계획하고 속도 명령을 낸다 |

- 펌웨어 코드에는 토픽 이름이 `cmd_vel` 처럼 **앞 슬래시 없이** 적혀 있다. ROS2 에서는 `/cmd_vel` 과 같은 토픽이다.

---

## 1. 전체 그림

### 하드웨어

```
[RPLIDAR C1] --USB--+
[Orbbec camera]-USB-+--> [Jetson Orin Nano]  Ubuntu 22.04 + ROS2 Humble
                    |          |
                    |          +--USB serial (micro-ROS)--> [Teensy 4.1] --PWM/DIR--> [motor driver] --> [DC motor x4]
                    |                                          ^   ^
[browser] --Wi-Fi (rosbridge :9090)--> Jetson                  |   +-- [encoder x4]
                                                               +------ [BNO055 IMU] (I2C)
[E-stop button] --> relay 1, 2 (cut motor power)  +  relay 3 --> Teensy pin 21 (software latch)
```

### 토픽 흐름 (실차, `real_bringup.launch.py` 기준)

```
sllidar_node   --/scan-----------------> slam_toolbox, Nav2 costmaps, mission_node, console
slam_toolbox   --TF map->odom----------> Nav2, mission_node
Teensy         --/odom-----------------> odom_guard --/odom_guarded--> EKF
Teensy         --/odom-----------------> Nav2 bt_navigator, controller_server
Teensy         --/imu/data-------------> EKF
EKF            --TF odom->base_footprint--> slam_toolbox, Nav2, mission_node
(role B node)  --/detections-----------> adapter_node
adapter_node   --/alarm, /person_status, /victim--> mission_node
console        --/alarm, /mission_cmd--> mission_node
mission_node   --navigate_to_pose (action)--> Nav2
Nav2 (velocity_smoother) --/cmd_vel----> micro_ros_agent --USB--> Teensy
Nav2 (behavior_server: spin/backup/wait) --/cmd_vel--> micro_ros_agent --USB--> Teensy
console        --/cmd_vel (manual drive, soft stop)--> micro_ros_agent --USB--> Teensy
```

- 젯슨이 Teensy 에 보내는 것은 **속도 명령 `/cmd_vel`** 과 서비스·리셋 몇 가지다(§5).
  `/cmd_vel` 을 내는 쪽은 Nav2 와 관제 웹 두 곳이다. Nav2 안에서는 `velocity_smoother`(평소 주행)와
  `behavior_server`(회복 동작 spin·backup·wait, smoother 를 거치지 않음 — Humble `navigation_launch.py` 에 리맵 없음)가 낸다.
- Teensy 가 젯슨에 보내는 것은 **`/odom`·`/imu/data`** 와 진단 토픽이다(§5).
- `/odometry/filtered`(EKF 출력 토픽)는 기동 게이트와 관제 화면이 받는다. 위치추정에 쓰이는 것은 EKF 가 내는 **TF** 쪽이다.
- 하드웨어 수치(바퀴 간격·지름·엔코더 PPR 등) = `docs/REAL_ROBOT_VALUES.md`.

---

## 2. 저장소 폴더 지도

| 폴더 | 들어 있는 것 | 언어 | 규모 | 만든 쪽 (§7) |
|---|---|---|---|---|
| `firmware/teensy_integrated_base_v1_4/` | Teensy 펌웨어 | **C++** (Arduino) | 코드 6파일 + 해시 1 · 3,321줄 | 구동부 원본 + 역할 A 수정 |
| `src/mission_manager/` | 미션 상태머신 (노드 1개 + 보조 모듈 3개) | Python | 약 3,200줄 | 역할 A |
| `src/perception_adapter/` | 인지 결과 → 미션 이벤트 변환 | Python | 약 1,500줄 | 역할 A |
| `src/tunnel_bringup/` | **실차** 런치·설정(yaml)·URDF·기동 게이트 | Python · YAML | 약 1,300줄 + 설정 7개 | 역할 A |
| `src/tunnel_interfaces/` | 커스텀 메시지 정의 2개 (`.msg`) | ROS IDL | 실행 코드 없음 | 역할 A·B 합의 |
| `src/tunnel_sim/` | **시뮬레이션**(Gazebo) 런치·월드·설정 | Python · XML | 약 900줄 | 역할 A |
| `src/sllidar_ros2/` | 라이다 드라이버 — ⚠ **GitHub 에는 없다**(§10) | **C++** | 69개 C++ 파일 | **외부** (Slamtec, BSD) |
| `src/my_first_pkg/` | ROS2 입문 연습 (talker/listener) | Python | 약 250줄 | 역할 A (학습용) |
| `console/` | 관제 웹 화면 + 로봇 디스플레이 화면 | JavaScript · Python | `js/` 16개 + 외부 라이브러리 | 역할 A |
| `tools/` | 검사·측정·분석 스크립트 (테스트 36개 포함) | Python · Shell · C++ 2개 | 125개 (하위 폴더 포함) | 역할 A |
| `maps/` | **시뮬** 터널 지도 파일 | 데이터 | — | SLAM 결과 |
| `docs/` | 설계·측정·함정 기록 | Markdown | — | 역할 A |
| `docs/handover/0904_perception/` | 역할 B 인지 노드 **사본** + 측정 로그 | Python | — | 역할 B 원본 |

- `src/` 패키지의 줄 수는 **테스트·`setup.py` 를 뺀** 코드 기준이다(예: `mission_manager` 테스트만 약 5,400줄).
- `build/` · `install/` · `log/` 는 빌드 산출물이다(코드 아님, git 에 없음).
- 저장소에 **없는** 외부 패키지(설치해서 씀): Nav2, slam_toolbox, robot_localization(EKF),
  micro_ros_agent, rosbridge, OrbbecSDK_ROS2. 저장소에는 이들의 **설정 파일**만 있다(§4-6).

---

## 3. 펌웨어 지도 (Teensy 4.1 · C++)

### 3-1. 파일

| 파일 | 줄 | 하는 일 |
|---|---|---|
| `teensy_integrated_base_v1_4.ino` | 2,076 | 본체. 핀·상수 정의, `setup()`, `loop()`, 모터·오도메트리·IMU·통신 |
| `rearm_gate.h` | 308 | 무장(re-arm) 상태 전이. 상태 5개 = `DRIVE_DISARMED`·`DRIVE_READY`·`DRIVE_ARMED`·`DRIVE_PENDING`·`DRIVE_ARMING` (`rearm_gate.h:60`). 모터는 `DRIVE_ARMED` 에서만 돈다 |
| `drive_wiring.h` | 114 | "무장 해제로 바뀌면 모터를 세운다"를 한 동작으로 묶은 것 |
| `estop_debounce.h` | 140 | E-stop 입력 떨림(글리치) 걸러내기 + 관측 카운터 |
| `runtime_guard.h` | 134 | `loop()` 한 바퀴가 너무 오래 걸리면 무장 해제 |
| `link_stall_probe.h` | 548 | USB 링크 멈춤 사건을 재고 `/firmware/event` 로 보고 |
| `SHA256SUMS.txt` | 1 | **구동부 원본** `.ino` 의 해시. 원본에서 수정했으므로 지금 `sha256sum -c` 를 돌리면 **실패하는 것이 정상**이다(`firmware/VENDOR_DROP.md`) |

- `.h` 다섯 개는 **Arduino 함수를 쓰지 않게** 짜여 있어서 노트북에서 `g++` 로 따로 컴파일해 시험한다(§3-6).
- 빌드 환경(Teensyduino 버전·라이브러리·패치) = `docs/FIRMWARE_REBUILD.md`.

### 3-2. 실행 흐름

**`setup()`** — `.ino:1739` (전원을 켜면 한 번)

1. 핀 설정: LED, E-stop(21번 입력), PWM 해상도 8비트, 모터 핀 · E-stop 필터 초기화(`estopDebounceInit`, 1747)
2. 모터 정지 + 무장 게이트 초기화(`rearmGateInit`) — 켜자마자 바퀴가 돌지 않는다
3. IMU 초기화(`initializeImu`) + 자이로 바이어스 보정(`calibrateGyroBias`)
4. micro-ROS 연결(`set_microros_transports`) → 노드 `teensy_integrated_base` 생성
5. **구독 3 · 발행 10 · 서비스 1** 등록(§3-4), 메시지 초기화(frame_id, 공분산), 실행기(executor) 준비
6. 젯슨과 시계 맞추기(`rmw_uros_sync_session`, 1967) · `resetOdometry`(1970) · `resetImuYaw`(1971) · `runtimeGuardInit`(1972) · 링크 계측 초기화(1982–1984)

**`loop()`** — `.ino:1991` (계속 반복, 끝에 `delay(1)`)

```
runtimeGuardBeginLoop() · linkProbeLoopBegin()   한 바퀴 시간 재기 시작
checkSafety()                 안전 점검 ①  (E-stop · 무장 게이트 · 0.5초 명령 워치독)
rclc_executor_spin_some(2ms)  들어온 토픽·서비스 처리 → 여기서 cmdVelCallback 등이 불린다
(무장 장벽 처리)               rearm 관련
checkSafety()                 안전 점검 ②
updateAppliedPwmEpoch()
updateMotorOutputs()          목표 PWM 쪽으로 천천히 이동(램프) 후 출력
updateOdometry()              엔코더 → 위치·속도 → /odom (20 ms 주기)
                              └ 이 안에서 updateWheelControllers() = 바퀴 PID 가 돈다 (.ino:1163)
updateImu()                   BNO055 → /imu/data (20 ms 주기)
publishDiagnostics()          /imu/gyro_bias · /estop/state · /drive/enabled · /drive/diag (1초 주기)
publishFirmwareInfo()         /firmware/info (5초 주기)
periodicTimeSync()            젯슨과 시계 맞추기 (30초 주기)
publishFirmwarePulse() · drainLinkEvents()   생존 신호 · 링크 사건 보고
linkProbeLoopEnd()            한 바퀴 시간 재기 끝
```

### 3-3. 함수 지도 (`.ino`, 기능별)

| 묶음 | 함수 (줄) |
|---|---|
| **속도 명령 → 바퀴** | `cmdVelCallback`(1364) → `applySkidSteerCommand`(739, 직진·회전 → 좌우 바퀴 속도) → `setTargetWheelVelocity`(703) |
| **바퀴 속도 제어 (PID)** | `updateWheelControllers`(762, `updateOdometry` 안에서 호출) · `wheelVelocityToFeedforwardPwm`(681) · `resetWheelControllerState`(655) |
| **PWM 출력** | `updateMotorOutputs`(907) · `movePwmTowardTarget`(866, 램프) · `writeMotorPwm`(638) · `stopAllMotors`(669) |
| **오도메트리** | `readEncoderCount`(1056) · `updateOdometry`(1089) · `publishOdometry`(1168) · `resetOdometry`(1061) |
| **IMU** | `initializeImu`(1196) · `calibrateGyroBias`(1218) · `updateImu`(1270) · `publishImu`(1327) · `applyGyroDeadband`(588) |
| **안전** | `checkSafety`(1022) · `isEstopActive`(607) · `updateEstopFilter`(597) · `disarmDrive`(965) · `disarmDriveWithReason`(971) · `driveEnableCallback`(1398, 무장 서비스) |
| **진단·시간** | `publishDiagnostics`(1440) · `publishFirmwareInfo`(1483) · `publishFirmwarePulse`(1631) · `drainLinkEvents`(1672) · `periodicTimeSync`(1700) · `recordRuntimePhase`(976) · `publishMeasured`(1001, 발행 결과를 링크 계측에 넘김) |
| **기타** | `resetOdomCallback`(1416) · `resetYawCallback`(1426) · `clampDouble`(574) · `normalizeAngle`(581) · `errorLoop`(1722) |

### 3-4. 펌웨어가 주고받는 토픽 (구독 3 · 발행 10 · 서비스 1)

| 방향 | 이름 | 타입 | 구동부 원본에 있었나 |
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

- `MAX_*_CMD` 는 펌웨어가 받아들이는 **상한**이다. 실제 주행 속도는 Nav2 설정이 정한다:
  기본 `nav2_params_real.yaml` = 0.12 m/s · 0.50 rad/s(`velocity_smoother`, 163–164행),
  시험 사본 `nav2_params_real_db.yaml` = 0.20 m/s · 0.83 rad/s.
- `MAX_*_CMD` 상향은 `docs/CURRENT_HANDOFF.md` 금지 범위에 들어 있다(전기 안전 검증과 묶임).

### 3-6. 보드 없이 시험하는 곳 (`tools/`)

| 파일 | 하는 일 |
|---|---|
| `tools/rearm_gate_host_test.cpp` + `.sh` | `rearm_gate.h` · `drive_wiring.h` · `estop_debounce.h` 를 노트북 `g++` 로 컴파일해 시험 |
| `tools/link_stall_host_test.cpp` + `.sh` | `link_stall_probe.h` 를 같은 방식으로 시험 |
| `tools/test_firmware_runtime_guard.py` | `runtime_guard.h` 경계 시험 |
| `tools/firmware_constants.py` | `.ino` 에서 상수를 읽어 오는 도구(다른 도구들이 씀) |
| `tools/firmware_precheck.sh` | 굽기 전 점검 — 원본 기준점(`f57d454`)에서 승인된 변경 내용의 sha256 을 대조. 08-22 에 승인 밖 변경을 구웠기 때문에 지금 실행하면 **FAIL** 이 나온다(`docs/CURRENT_HANDOFF.md` 열린 위험 · 예약 62) |

### 3-7. 구동부 원본과 이후 수정 구분

- 원본 = 구동부 파트가 만든 `.ino` 한 파일(**1,308줄** — 마지막 줄바꿈이 없어 `wc -l` 은 1,307 로 센다). 2026-08-05 커밋 `f57d454` 로 저장소에 들어왔다.
- `hanium-final` 까지의 변경: `.ino` **+817 / −49 줄**(2,076줄이 됨) + `.h` 다섯 개 신규(합계 1,244줄).
- 원본에 없던 함수 10개: `disarmDrive` · `disarmDriveWithReason` · `driveEnableCallback` · `updateEstopFilter` ·
  `updateAppliedPwmEpoch` · `recordRuntimePhase` · `currentEpochMs` · `publishMeasured` · `publishFirmwarePulse` · `drainLinkEvents`.
- 값이 바뀐 상수:

| 상수 | 원본 | `hanium-final` |
|---|---|---|
| `MAX_LINEAR_CMD` / `MAX_ANGULAR_CMD` / `MAX_WHEEL_CMD` | 0.12 / 0.50 / 0.15 | 0.20 / 0.83 / 0.30 |
| `WHEEL_KP` | 30.0 | 60.0 |
| `INTEGRAL_PWM_LIMIT` | 20.0 | 40.0 |
| `FEEDFORWARD_PWM_PER_MPS_ABOVE_MIN` | 1300.0 | 335.0 |
| `ESTOP_ACTIVE_LOW` | true | false |
| `WHEEL_BASE` 0.62 | → | `CMD_WHEEL_BASE` 0.62 + `ODOM_WHEEL_BASE` 0.859 로 분리 |
| `WHEEL_RADIUS` 0.05698 | → | `ODOM_WHEEL_RADIUS` 0.05698 (이름 변경) |
| `FW_VERSION` | `handover-integrated-pi-continuous-low-speed-1.3.0` | `rearm-latch-pi-link-probe-1.6.3` |
| `FW_SOURCE_PATH` | `/home/park/robot_firmware/teensy_integrated_pi_continuous_low_speed_v1_3.ino` | `firmware/teensy_integrated_base_v1_4/teensy_integrated_base_v1_4.ino` |
| `LARGE_PUBLISH_TIMEOUT_MS` | (없음) | 20 |

- 원본 전체를 보거나 변경을 확인하는 명령:

```bash
git show f57d454:firmware/teensy_integrated_base_v1_4/teensy_integrated_base_v1_4.ino > /tmp/original.ino
git diff f57d454 hanium-final -- firmware/teensy_integrated_base_v1_4/
```

- 수령 경위·소유 경계 = `firmware/VENDOR_DROP.md`.

---

## 4. 젯슨 지도 (Jetson Orin Nano · ROS2 Humble)

### 4-1. 실차 기동 — `real_bringup.launch.py`

```bash
ros2 launch tunnel_bringup real_bringup.launch.py map_file:=<지도 경로>                       # 미션 없이 Nav2 까지
ros2 launch tunnel_bringup real_bringup.launch.py map_file:=<지도 경로> mission:=true waypoints_file:=<yaml>   # 미션까지
```

**인자** (`src/tunnel_bringup/launch/real_bringup.launch.py` 아래쪽)

| 인자 | 기본값 | 뜻 |
|---|---|---|
| `map_file` | **없음(필수)** | 저장된 SLAM 지도(posegraph) 경로 |
| `mission` | `false` | `true` 여야 미션 노드가 켜진다 |
| `waypoints_file` | 빈 값 | 미션 시나리오 yaml |
| `camera` | `false` | `true` 면 카메라 depth 스트림 확인이 기동 단계에 들어간다 |
| `nav2_params` / `ekf_params` / `slam_params` | `nav2_params_real.yaml` / `ekf_real.yaml` / `slam_real_localization.yaml` | 설정 파일 선택(§4-6) |
| `lidar` · `micro_ros` | `true` | 라이다 · micro-ROS agent 를 켤지 |

**켜지는 것** (`ros2 node list` 로 보이는 이름 기준)

| 이름 | 실행 파일 · 패키지 | 우리 코드? | 하는 일 |
|---|---|---|---|
| `robot_state_publisher` | 외부 (ROS 기본) | 설정만 (`urdf/robot_real.urdf`) | 로봇 부품 위치 관계(TF) 발행 |
| `sllidar_node` | `sllidar_ros2` | ❌ 외부 (Slamtec) | 라이다 → `/scan` |
| `micro_ros_agent` | 외부 | ❌ | USB 로 Teensy 와 토픽 중계 |
| `odom_guard` | `tools/odom_guard.py` (rclpy 노드. 런치에서는 `Node` 가 아니라 `ExecuteProcess` 로 띄운다) | ✅ | 깨진 `/odom` 메시지를 걸러 `/odom_guarded` 로 다시 냄 |
| `ekf_filter_node` | `ekf_node` · robot_localization | 설정만 (`ekf_real.yaml`) | `/odom_guarded` + `/imu/data` 융합 → TF `odom→base_footprint` |
| `slam_toolbox` | `localization_slam_toolbox_node` | 설정만 | 저장된 지도 위에서 위치 추정 → TF `map→odom` |
| Nav2 노드들 (`controller_server` · `planner_server` · `bt_navigator` · `behavior_server` · `velocity_smoother` 등) | Nav2 | 설정만 (`nav2_params_real*.yaml`) | 경로 계획·추종 → `velocity_smoother` 가 `/cmd_vel` · 회복 동작은 `behavior_server` 가 직접 `/cmd_vel` |
| `readiness_gate_sensors` · `_camera`(선택) · `_localized` · `_nav2` | `readiness_gate` · `tunnel_bringup` | ✅ | 단계별 준비 확인 |
| `mission_manager` | `mission_node` · `mission_manager` | ✅ | 미션 상태머신 (`mission:=true` 일 때만) |

- 기동 순서: 센서 확인 → (카메라 확인) → 위치추정 확인 → Nav2 확인 → 미션(또는 "Nav2 준비 완료" 메시지).
  한 단계라도 시간 안에 준비되지 않으면 다음 단계를 켜지 않고 **런치 전체를 종료**한다(`src/tunnel_bringup/tunnel_bringup/launch_util.py` `when_ready`).
- **어댑터(`adapter_node`)는 이 런치에 없다** — 별도 런치 `src/perception_adapter/launch/adapter.launch.py`.
- 지도 제작은 다른 런치: `real_mapping.launch.py`.

### 4-2. `mission_manager` — 미션 상태머신

| 파일 | 줄 | 하는 일 |
|---|---|---|
| `mission_node.py` | 1,907 | 노드 본체. 상태머신·콜백·타이머 |
| `goal_manager.py` | 522 | Nav2 목표 전송·취소의 수명주기 관리 |
| `speed_manager.py` | 530 | 상태별 주행 속도를 Nav2 파라미터(`FollowPath.desired_linear_vel`)로 바꾸기 |
| `follower_monitor.py` | 259 | 뒤따라오는 사람을 라이다로 감시(놓침·재발견 판정) |
| `config/waypoints*.yaml` | — | 경유점·집결지·속도 등 시나리오 값 (실차 `_real_H*`, 시뮬 `waypoints.yaml`·`_twin`) |

**상태 12개짜리 상태머신 하나** — `mission_node.py:75` `class State`

```
PATROL ─화재 경보─▶ APPROACH ─도착─┬─(person_gate: true)──▶ SCAN_AREA ─┬─▶ GATHER
                                   │                                    └─(쓰러진 사람 확정)─▶ RESCUE
                                   └─(person_gate: false)─▶ GATHER
GATHER ─┬─▶ GUIDE
        ├─(쓰러진 사람 확정)─▶ RESCUE
        └─(아무도 없음 확정)─▶ NO_VICTIM
GUIDE ─(놓침)─▶ HOLD ─(재발견)─▶ GUIDE
                HOLD ─(시간 초과)─▶ SEARCH_BACK ─┬─(재발견)─▶ GUIDE
                                                 └─(쓰러진 사람 확정)─▶ RESCUE
GUIDE ─(출구 도착)─▶ ESCAPED
FAULT ─(자동 재시도)─▶ 실패 직전 상태 (`mission_node.py:1233` `resume_state`)
화재 경보 시 안전한 집결지 계산 실패 ─▶ BLOCKED
```

| 상태 | 뜻 (`mission_node.py:76~108` 주석 기준) |
|---|---|
| `PATROL` | 평시 순찰 |
| `APPROACH` | 화재 경보 → 집결지로 이동 |
| `SCAN_AREA` | 집결지에서 제자리 회전하며 사람 탐색 |
| `GATHER` | 정해진 시간 동안 집결 대기 |
| `GUIDE` | 저속으로 앞서가며 유도 + 뒤따르는 사람 감시 |
| `HOLD` | 사람을 놓친 직후 제자리 대기 |
| `SEARCH_BACK` | 마지막 목격 지점 쪽으로 돌아가 재탐색 |
| `RESCUE` | 쓰러진 사람 확정 → 신고하고 정지 |
| `NO_VICTIM` | 집결지에 아무도 없음 확정 → 신고하고 정지 |
| `ESCAPED` | 탈출 완료 |
| `FAULT` | Nav2 실패 → 자동 재시도 → 소진 시 정지. 관제 `abort` 명령으로도 들어간다(`on_cmd`) |
| `BLOCKED` | 안전한 집결지를 계산하지 못해(예: 화재와 너무 가까움) 사람에게 넘긴 상태. 재시도 없이 정지 |

- `person_gate` 값: `waypoints_real_H.yaml` = `true`, 나머지 waypoints 파일 = `false`.

**주요 함수** (`mission_node.py`)

| 묶음 | 함수 (줄) |
|---|---|
| 시작·타이머 | `__init__`(561) — `tick` 0.5초(2 Hz) · `safety_watchdog` 0.5초 |
| 상태머신 본체 | `tick`(784) — 상태마다 무엇을 할지 |
| 상태 진입 | `enter_scan_area`(1352) · `enter_rescue`(1438) · `enter_no_victim`(1443) · `enter_hold`(1448) · `enter_search_back`(1467) · `enter_fault`(1828) · `_enter_guide_lowspeed`(1257) |
| Nav2 목표 | `send_goal`(730) · `cancel_current_goal`(735) · `on_reached`(746) · `scan_goal`(1363) |
| 입력 콜백 | `on_scan`(1559) · `on_alarm`(1563) · `on_person_status`(1269) · `on_victim`(1273) · `on_cmd`(1735, `reset`·`abort`) |
| 사람 판정 | `person_verdict`(1291) · `fresh_person_status`(1281) · `record_last_seen`(1532) |
| 정지 확인 | `_confirmed_stop`(1376) · `on_safety_stop_confirmed`(1720) · `safety_watchdog`(1801) |
| 설정 검증 | `need` · `need_num` · `need_int` · `need_bool` (374~439) — yaml 값 검사 |

**토픽**: 구독 `/scan`(614) `/alarm` `/victim` `/person_status` `/mission_cmd` · 발행 `/mission_state` `/siren` ·
액션 클라이언트 `navigate_to_pose`(Nav2). 위치는 TF 로 얻는다(`/map` 토픽은 구독하지 않는다).

### 4-3. `perception_adapter` — 인지 → 미션 연결

| 파일 | 줄 | 하는 일 |
|---|---|---|
| `adapter_node.py` | 1,195 | `/detections` 를 받아 검증·누적 판정 → `/alarm`(화재) · `/person_status` · `/victim` · `/adapter_status` 발행. `/mission_state` · `/adapter_cmd` 도 구독 |
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
| `tunnel_bringup/launch/real_bringup.launch.py` (433줄) | 실차 운행 기동(§4-1) |
| `tunnel_bringup/launch/real_mapping.launch.py` (194줄) | 실차 지도 제작 |
| `tunnel_bringup/tunnel_bringup/readiness_gate.py` (565줄) | 단계별 준비 확인 노드 |
| `tunnel_bringup/tunnel_bringup/launch_util.py` | 게이트를 만들고 순서대로 잇는 도우미 |
| `tunnel_bringup/urdf/robot_real.urdf` | 실차 치수·센서 위치 |
| `tunnel_sim/worlds/*.world` | Gazebo 터널 월드 |
| `tunnel_sim/launch/*.launch.py` (7개) | 시뮬 로봇·SLAM·Nav2·미션 기동 |
| `tunnel_sim/tunnel_sim/fake_follower.py` | 시뮬에서 따라오는 사람 흉내 |

### 4-6. 설정 파일 (외부 패키지의 동작을 정하는 곳)

| 파일 (`src/tunnel_bringup/config/`) | 대상 |
|---|---|
| `nav2_params_real.yaml` | Nav2 기본 설정 (속도·허용오차·장애물 여유 등) |
| `nav2_params_real_db.yaml` | Nav2 **시험용 사본** — 구동부 불감대 대응으로 속도를 올린 판. `nav2_params:=` 인자로 고른다 |
| `ekf_real.yaml` | EKF 기본 설정 (어떤 센서 값을 융합할지) |
| `ekf_real_db.yaml` | EKF **고장 우회용 사본** (한쪽 엔코더 계측 이상 대응) |
| `slam_real_mapping.yaml` · `slam_real_localization.yaml` | slam_toolbox (지도 제작용 / 위치추정용) |
| `lidar_standalone.yaml` | 라이다 노드만 따로 되살릴 때 쓰는 파라미터 |

시뮬 쪽 같은 역할의 파일 = `src/tunnel_sim/config/`.

---

## 5. 두 영역의 경계 — 젯슨 ↔ Teensy

| 방향 | 토픽 | 젯슨 쪽 상대 | 펌웨어 쪽 함수 |
|---|---|---|---|
| 젯슨 → Teensy | `/cmd_vel` | Nav2 `velocity_smoother` · Nav2 `behavior_server`(회복 동작) · 관제 웹(수동 조종·소프트 정지) | `cmdVelCallback` |
| 젯슨 → Teensy | `/drive/enable` (서비스) | 사람이 `ros2 service call` 로 호출 (현장 시험 도구 `tools/rearm_field_wiring.py` · `rearm_field_regress.py` · `rearm_field_disarm.py` · `rearm_neg6_field.py` 도) | `driveEnableCallback` |
| 젯슨 → Teensy | `/reset_odom` · `/imu/reset_yaw` | 사람이 명령으로 | `resetOdomCallback` · `resetYawCallback` |
| Teensy → 젯슨 | `/odom` | `odom_guard`(→ EKF) · Nav2 `bt_navigator` · Nav2 `controller_server` · 기동 게이트 | `publishOdometry` |
| Teensy → 젯슨 | `/imu/data` | EKF · 기동 게이트 | `publishImu` |
| Teensy → 젯슨 | `/estop/state` · `/drive/enabled` · `/drive/diag` · `/imu/gyro_bias` | 관제 화면 | `publishDiagnostics` |
| Teensy → 젯슨 | `/firmware/info` · `/firmware/pulse` · `/firmware/event` | 관제·분석 도구 | `publishFirmwareInfo` 등 |

- 연결: USB 시리얼 → 젯슨의 `micro_ros_agent` 가 받아서 일반 ROS2 토픽으로 풀어 준다.

**로봇을 멈추는 경로**

| 경로 | 어디서 | 위치 |
|---|---|---|
| 하드웨어 E-stop | 버튼 → 릴레이 1·2 가 **모터 전원 자체를 끊는다** | `docs/ELECTRICAL_BASELINE.md` §4 |
| 펌웨어 E-stop 래치 | 21번 핀 입력 → `checkSafety` 가 무장 해제 | `.ino:1022` · `estop_debounce.h` |
| 명령 워치독 | `/cmd_vel` 이 0.5초 끊기면 모터 정지 | `.ino:1022` `checkSafety` · `WATCHDOG_TIMEOUT_MS` |
| 무장 게이트 | `DRIVE_ARMED` 상태가 아니면 모터가 돌지 않는다 · 재무장은 `/drive/enable` 서비스로만 | `rearm_gate.h` · `drive_wiring.h` |
| 루프 지연 감시 | `loop()` 한 바퀴가 너무 길면 무장 해제 | `runtime_guard.h` |
| 0 속도 명령 | Nav2 가 멈추거나, 관제 소프트 정지(`abort` + `/cmd_vel` 0 ×10). 기록된 한계: `abort` 는 한 번만 발행된다(예약 81) · 관제의 `/cmd_vel` 0 은 `velocity_smoother` 의 20 Hz 출력에 덮일 수 있다(예약 82) — `docs/MASTER_PLAN.md` §7 | `console/js/ros.js:400` |
| 미션 정지 | 관제 `abort` → `FAULT` | `mission_node.py` `on_cmd` |

---

## 6. 그 밖의 폴더

### `console/` — 관제 웹

- 실행: `console/run_console.sh` → rosbridge(포트 9090) + 웹서버(포트 8000). 사용법 = `console/README.md`.
- `index.html` + `js/` 16개: `ros.js`(토픽 연결) · `map.js`(지도) · `mission.js`(상태) · `diag.js`(펌웨어 진단 해독) ·
  `emergency.js`(수동 조종·비상) · `display.js`(로봇 디스플레이 모드) 등. 외부 라이브러리 = `console/vendor/`.
- 구독: §4·§5 의 토픽 대부분(`/map` `/scan` `/mission_state` `/drive/diag` `/detections` …).
- **발행**: `/mission_cmd`(`reset`·`abort`) · `/alarm`(수동 화재 지정) · `/cmd_vel`(수동 조종·소프트 정지) · `/display_msg` — `console/js/ros.js:337~340`.
- 카메라 패널은 로봇의 `web_video_server`(:8080) 실영상을 먼저 시도하고, 안 되면 데모 클립 → 자리표시 순서다(`console/js/camfeed.js`).
- 화면에 `데모 데이터` 라벨이 붙은 곳 = **전원·부하 패널**(`console/index.html:293`).
- 인지 수치는 `/detections` 를 구독하지만, **`/detections` 가 한 번도 오지 않으면** `console/js/demo.js`(`DEMO_ON = true`)가
  미션 상태별 표에서 만든 값(사람·화재 수, 신뢰도, 어댑터 상태)을 **라벨 없이** 띄운다 — 인지 패널(`diag.js`)·로봇 시야(`mission.js`)·영상 메뉴(`video.js`).
  실물 `/detections` 가 한 번이라도 오면 실물 값이 쓰인다(`demo.js` `demoPerception`).
- `대피자 추종` 표시는 센서 값이 아니라 미션 상태에서 만든 값이다(`console/js/mission.js` · `i18n.js` `FOLLOWER_OF`).

### `tools/` — 검사·측정 스크립트

| 이름 앞부분 | 개수 | 대략 |
|---|---|---|
| `test_*` | 36 | 자동 테스트 |
| `drive_*` | 7 | 구동부 측정·보고 |
| `rearm_*` | 6 | 무장 게이트 — 보드 없는 시험 2개(`rearm_gate_host_test.*`) + 현장 시험 4개(`rearm_field_*` 3 · `rearm_neg6_field.py`) |
| `link_*` · `bag_*` | 4 · 4 | 링크 멈춤·rosbag 분석 |
| `odom_guard.py` | 1 | 실차 런치가 실행하는 `/odom` 필터(§4-1) |
| `viz/` | — | rosbag → 영상 렌더러 (`tools/viz/README.md`) |

### 테스트 위치

| 위치 | 대상 |
|---|---|
| `src/mission_manager/test/` (8파일) | 미션 상태머신 |
| `src/perception_adapter/test/` (5파일) | 어댑터 |
| `src/tunnel_bringup/test/` (7파일) | 기동 게이트·라이다 설정 시험 3개 + 시험용 가짜 모듈 `gate_fakes.py` + 코드 스타일 검사 3개 |
| `src/tunnel_sim/test/` (3파일) | 코드 스타일 검사(copyright·flake8·pep257)만 |
| `src/my_first_pkg/test/` (3파일) | 코드 스타일 검사만 |
| `console/test/` (7파일) | 관제 웹 — 배선 대조 `check_wiring.py` + 자바스크립트 시험(`.mjs`) |
| `tools/test_*` | 도구·펌웨어 헤더 |

실행법 = `docs/TEST_GATES.md`.

---

## 7. 출처·작성자 구분 (사실 기록)

| 대상 | 작성 | 근거 |
|---|---|---|
| 펌웨어 원본 (`.ino` 1,308줄) | 구동부 파트 | `firmware/VENDOR_DROP.md`, 커밋 `f57d454` |
| 펌웨어 이후 수정 (`.ino` +817/−49줄 · 상수 변경 · `.h` 5개 신규) | 역할 A | §3-7 · `git log -- firmware/` |
| `src/` 의 mission·adapter·bringup·sim·interfaces | 역할 A (interfaces 는 역할 B 와 합의) | `git log` |
| 인지 노드 (RF-DETR 화재 · YOLO 자세 + depth) | 역할 B — 저장소에는 사본만 | `docs/handover/0904_perception/` |
| `src/sllidar_ros2` | Slamtec (외부, BSD 라이선스) | https://github.com/Slamtec/sllidar_ros2 (커밋 `3430009`) |
| Nav2 · slam_toolbox · robot_localization · micro-ROS · rosbridge | 외부 오픈소스 (설치형) | 저장소에는 설정만 |
| 관제 웹의 외부 라이브러리 | 외부 | `console/vendor/` |
| AI 도구 사용 | `hanium-final` 기준 커밋 378개 중 344개에 `Co-Authored-By: Claude` 표기 · 검토에 Codex 사용 | `git log hanium-final --grep='Co-Authored-By: Claude' --oneline \| wc -l` |

---

## 8. 분석할 때 쓰는 명령

```bash
# 어떤 함수·토픽이 어디서 쓰이나
grep -rn "cmd_vel" src/ firmware/ console/js/

# 이 파일은 언제 왜 바뀌었나 (커밋 메시지에 이유가 적혀 있다)
git log --oneline -- src/mission_manager/mission_manager/mission_node.py

# 이 줄을 누가 언제 바꿨나
git blame -L 75,110 src/mission_manager/mission_manager/mission_node.py

# 한이음 최종본 그대로 보기 (나중에 코드가 바뀌어도)
git show hanium-final:firmware/teensy_integrated_base_v1_4/teensy_integrated_base_v1_4.ino
```

로봇(또는 시뮬)이 켜져 있을 때:

```bash
ros2 node list                   # 지금 실행 중인 노드
ros2 topic list                  # 지금 있는 토픽
ros2 topic echo /mission_state   # 토픽 내용 실시간 보기
ros2 topic info /cmd_vel -v      # 이 토픽을 누가 내고 누가 받나
rqt_graph                        # 노드·토픽 연결을 그림으로
```

---

## 9. 관련 문서

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

- 일부 문서(예: `PROJECT_CONTEXT.md` 의 토픽 표)는 작성 시점 이후 코드가 바뀐 곳이 있다. 문서와 코드가 다르면 코드가 실제 동작이다.

---

## 10. GitHub 에 있는 것 / 없는 것

우리가 쓴 코드·펌웨어·설정·문서·커밋 이력은 GitHub 저장소에 있고, 저장소는 공개라 로그인 없이 열린다.
아래는 **저장소에 없는 것**과 구하는 곳이다.

| 없는 것 | 왜 없나 | 구하는 곳 |
|---|---|---|
| 라이다 드라이버 `src/sllidar_ros2/` | 외부 코드라 `.gitignore` 로 제외 | `git clone https://github.com/Slamtec/sllidar_ros2.git` → `git checkout 3430009` (사용한 판, 2024-06-17) |
| 펌웨어가 쓰는 라이브러리 | 외부 코드 | `micro_ros_arduino` 2.0.8-humble · `Adafruit_BNO055` 1.6.4 · `Adafruit_BusIO` 1.17.4 · `Adafruit_Unified_Sensor` 1.1.15 (각 이름으로 GitHub 검색) · `Encoder` 1.4.3 = Teensyduino 1.58.2 동봉. 굽기까지 하려면 `docs/FIRMWARE_REBUILD.md` |
| Nav2 · slam_toolbox · robot_localization · micro_ros_agent · rosbridge | 설치형 외부 패키지 | 각 프로젝트 GitHub (`ros-navigation/navigation2`, `SteveMacenski/slam_toolbox`, `cra-ros-pkg/robot_localization` 등) |
| 역할 B 인지 코드의 **원본** | 역할 B 소유 | 저장소엔 **사본** `docs/handover/0904_perception/perception_node_0904.py` 만 |
| 젯슨에서만 고친 내용 | 젯슨 로컬 변경 | 목록·원복법 = `docs/handover/0904_perception/README.md` |
| 실차 운행 지도 | 젯슨에만 있음 | 젯슨 `~/Desktop/maps_0820/`. 저장소 `maps/` 는 **시뮬** 지도 |
| 주행 기록(rosbag)·시연 영상 | 용량(폴더 전체 약 14 GB) | 사용자 노트북 `~/robot_evidence/` |
| 관제 화면 데모 영상 클립 | 용량 | 사용자 노트북 `console/media/` |
| 날짜별 개발 기록·검토문 | 다른 저장소 | **공개 저장소** https://github.com/ghwjd1357-alt/tunnel-evac-docs — 마크다운 문서만 있다. 문서 곳곳의 `~/Desktop/…` 경로 중 마크다운 파일은 이 저장소에 있고, 라이브러리 폴더·지도 같은 그 밖의 경로는 없다 |

**실행 환경**: Ubuntu 22.04 + ROS2 Humble + Gazebo Classic 11 이 있어야 하고,
위 외부 패키지를 설치한 뒤 `colcon build --symlink-install`. 실차 없이 돌리는 방법 = `docs/TEST_GATES.md`.
