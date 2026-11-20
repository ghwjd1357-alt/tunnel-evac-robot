# term_drive_v2 — 클래스 구조의 Teensy 4.1 구동 펌웨어

지하터널 대피 로봇의 구동 펌웨어다. 젯슨이 보낸 속도 명령(`/cmd_vel`)대로 바퀴 4개를 돌리고,
엔코더로 잰 움직임(`/odom`)을 젯슨에 돌려준다. 구동 제어부를 **클래스**로 구성했다.

## 파일

| 파일 | 내용 |
|---|---|
| `term_drive_v2.ino` | 메인. `setup()`·`loop()`, micro-ROS 통신, 하드웨어 입출력(PWM 핀·엔코더·IMU) |
| `drive_base.h` | `DriveBase` — 로봇 몸체. (v, ω) → 바퀴 4개 목표 (차동구동 기구학) |
| `wheel.h` | `Wheel` — 바퀴 하나. 속도 측정·필터·기동 부스트·램프·PWM 계산 |
| `pid_controller.h` | `PIDController` → `WheelSpeedController` (상속). 피드포워드 + PID(D 꺼짐) + 출력 제한 |
| `periodic_timer.h` | `PeriodicTimer` — "주기가 지났을 때만 일한다" |
| `wheel_config.h` | 설정값 묶음 구조체 3개 (`WheelControlConfig` · `WheelMotionConfig` · `DriveConfig`) — 값 하나는 한 묶음에만 |
| `drive_math.h` | 범위 자르기·큰 값·반올림 도우미 |
| `rearm_gate.h` · `drive_wiring.h` · `estop_debounce.h` · `runtime_guard.h` · `link_stall_probe.h` | 안전 모듈 (구조체·열거형). 무장 게이트·E-stop 디바운스·루프 감시·링크 계측 |

## 클래스 구조

```
DriveBase ── wheels[4] : Wheel ── controller : WheelSpeedController
              [0] 왼앞                                  │ 상속
              [1] 왼뒤                                  ▼
              [2] 오른앞                          PIDController
              [3] 오른뒤

PeriodicTimer × 6   pwmRampTimer · diagnosticTimer · firmwareInfoTimer ·
                    timeSyncTimer · odomTimer · imuTimer

설정값 묶음(struct)  WheelControlConfig · WheelMotionConfig · DriveConfig
```

- **클래스는 계산만 한다.** `analogWrite`·`digitalWrite`·엔코더 읽기·`millis()` 같은 하드웨어 호출은
  `.ino` 에 있다. 시각과 엔코더 값은 인자로 받는다. 그래서 헤더들은 Arduino 없이 노트북 `g++` 로
  컴파일해 시험할 수 있다.
- 설정값(게인·상한·주기)은 `.ino` 의 상수에 있고, 숫자의 근거와 측정 이력 주석도 그 옆에 있다.
- 핀 번호는 `.ino` 의 `PWM_PIN`·`DIR_PIN` 한 곳에만 있다(정지 경로가 객체가 아니라 상수 표를 직접 쓴다).

## 한 번의 제어 주기 (20 ms)

```
젯슨 /cmd_vel ─▶ cmdVelCallback ─▶ DriveBase::setCommand ─▶ Wheel::setTarget ×4
엔코더 ─▶ updateOdometry ─▶ Wheel::measure ×4 (속도·필터) ─▶ 오도메트리 ─▶ /odom
                       └─▶ DriveBase::updateControllers ─▶ WheelSpeedController::compute ×4
updateMotorOutputs ─▶ Wheel::updateOutput ×4 (부스트·램프) ─▶ writeMotorPwm (핀)
```

## 공학프로그래밍 강의와의 대응

| 강의 | 개념 | 이 펌웨어에서 |
|---|---|---|
| 07 배열 | 배열 | 핀 번호 `PWM_PIN[4]` · `DIR_PIN[4]`, 바퀴별 설정 `START_BOOST_PWM[4]` 등, **객체 배열** `Wheel wheels[4]` |
| 08 구조체·열거형 | `struct`, `enum` | 설정 묶음 `WheelControlConfig` 등, 무장 상태 `enum DriveState`, `struct RearmGate`, `enum MotorIndex` |
| 09 클래스와 객체 | 멤버변수·멤버함수, 객체 여러 개, `private`, `::` | `Wheel` 객체 4개, `PeriodicTimer` 객체 6개, 상태는 `private`, 멤버함수는 클래스 밖에서 `Wheel::setTarget` 처럼 정의 |
| 10 생성자·상속 | 매개변수 생성자, 상속 | `Wheel(부스트PWM, 저속유지PWM, 최소구동PWM, 설정…)`, `PeriodicTimer(1000)`, `WheelSpeedController : public PIDController` — 기본 클래스가 PID 단계(미분·적분 후보·각 항·적분 확정)를, 파생 클래스가 피드포워드·출력 제한·와인드업 방지를 맡는다 |

## 빌드

```bash
arduino-cli compile -b teensy:avr:teensy41 firmware/term_drive_v2
```
라이브러리·보드 버전·필수 패치 = `docs/FIRMWARE_REBUILD.md`. 보드에 굽는 절차와 안전 확인(바퀴 띄우기·E-stop)은
같은 문서와 `docs/ELECTRICAL_BASELINE.md` 를 따른다.

## 검증

- **동일성 (노트북)**: 원본(`firmware/teensy_integrated_base_v1_4/`)과 같은 입력을 넣었을 때 출력·내부 상태가 같은지
  `python3 tools/term_drive_equivalence.py`
- **안전 모듈 (노트북)**: 헤더 5개는 원본과 바이트 단위로 같다. 이 폴더의 `.ino` 배선까지 검사하려면 폴더를 지정한다
  ```bash
  SKETCH=firmware/term_drive_v2 bash tools/rearm_gate_host_test.sh
  SKETCH=firmware/term_drive_v2 bash tools/link_stall_host_test.sh
  ```
- **보드에서의 계산 방식**: Teensy 컴파일러는 곱셈과 덧셈을 명령 하나(FMA)로 합쳐 반올림을 한 번만 하기도 한다.
  어느 연산을 합치는지는 소스 모양에 따라 달라지고, 노트북(x86)은 합치지 않아 노트북 시험으로는 이 차이가 안 보인다.
  그래서 보드용 바이너리를 원본과 `arm-none-eabi-objdump` 로 비교해, 제어기·오도메트리·PWM 출력의 부동소수 연산이
  원본과 같은 방식으로 계산됨을 확인했다. 차동구동 계산의 `× 0.5` 하나는 합쳐지는 방식이 다르지만, 0.5 곱셈은
  오차가 없어서 값은 같다. `pid_controller.h` 의 문장 순서를 바꾸면 이 비교를 다시 해야 한다.

## 출처

- 제어 로직(피드포워드·PI·부스트·램프·오도메트리)의 바탕: 한이음 프로젝트 구동부 펌웨어 `teensy_integrated_base_v1_4`
  1.6.3 판 (`.ino` 마지막 변경 커밋 `60bb3c2`). 최초 수령본은 커밋 `f57d454` 이다.
- 외부 라이브러리: micro_ros_arduino 2.0.8-humble · Encoder 1.4.3 · Adafruit BNO055 1.6.4 · Adafruit BusIO 1.17.4 · Adafruit Unified Sensor 1.1.15
