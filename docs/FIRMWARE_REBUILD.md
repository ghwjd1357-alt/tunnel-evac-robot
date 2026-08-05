# FIRMWARE_REBUILD.md — Teensy 펌웨어 재빌드 환경 (2026-08-05 신설·실증)

> **왜 이 문서가 있나**: `re-arm` 래치·`ESTOP_ACTIVE_LOW`·(뒤에) `EFFECTIVE_WHEEL_BASE` 를
> 바꾸려면 펌웨어를 다시 구워야 한다. 그런데 **micro-ROS 는 보드 `platform.txt` 를 패치하지
> 않으면 링크에서만 죽는다** — 컴파일은 멀쩡히 통과하기 때문에 원인을 코드에서 찾게 된다.
> 그 함정과 실제로 통한 절차를 기록한다.
>
> **정본 경계** — 이 문서 = *빌드 환경 구성과 검증*. 펌웨어가 가진 값·전제 = `REAL_ROBOT_VALUES.md §1`.
> E-stop 전기 설계 = `ELECTRICAL_BASELINE.md §4`. 라이브러리 수령 기록 = `JETSON_SETUP.md §5-d`.

## 0. 어디서 빌드하나 — **노트북**

| | 노트북 (`minwoo`, x86_64) | Jetson (`hanhan`, aarch64) |
|---|---|---|
| 판정 | ✅ **여기서 한다** | ⏸ 지금은 안 한다 |

이유:

1. `micro_ros_arduino` 는 **사전컴파일 정적 라이브러리**(`libmicroros.a`)를 링크한다. 타깃은
   Teensy 의 Cortex-M7 이므로 **호스트 아키텍처와 결과물이 무관**하다 → 어려운 쪽을 고를 이유가 없다.
2. Teensyduino 는 x86_64 리눅스가 1순위 지원이다.
3. 업로드는 Teensy USB 케이블을 잠깐 노트북에 꽂으면 된다.
4. Jetson 은 실차 런타임(agent·SLAM·Nav2) 담당이다. 자원 여유(`JETSON_SETUP.md §11-f` 실측)를
   IDE·툴체인으로 흔들지 않는다.

⚠ **나중에 Jetson 이 필요해질 수 있다** — 터널 현장에서 노트북 없이 `EFFECTIVE_WHEEL_BASE` 를
재조정해야 하는 시나리오가 `REAL_ROBOT_VALUES.md §5` 에 있다. 그때 이 문서를 aarch64 로 확장한다.

## 1. 환경 지문 — 이 두 숫자가 "맞는 환경"을 판정한다

펌웨어 소스가 컴파일러 매크로를 **그대로 문자열로 굽는다**:

```c
static const char FW_ARDUINO_VERSION[]     = STRINGIFY(ARDUINO);       // 실차 관측 10607
static const char FW_TEENSYDUINO_VERSION[] = STRINGIFY(TEENSYDUINO);   // 실차 관측 158
```

그래서 `/firmware/info` 가 방송하는 `arduino_macro`·`teensyduino_macro` 가 **환경 일치의 지문**이다.

| 지문 | 실차 관측 (D+0) | 뜻 |
|---|---|---|
| `arduino_macro` | **10607** | Arduino IDE 2.x · `arduino-cli` 계열 (IDE 1.8.x 면 10819 가 나온다) |
| `teensyduino_macro` | **158** | Teensyduino **1.58.x** |

## 2. 구성 절차 (2026-08-05 실행 그대로)

`arduino-cli` 를 쓴다 — IDE 2.x 의 빌드 백엔드와 같은 것이라 위 지문이 그대로 나오고,
GUI 없이 검증·재현할 수 있다.

```bash
# ① arduino-cli (sudo 불필요 — 사용자 홈에 설치)
curl -sSL -o /tmp/arduino-cli.tar.gz \
  https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Linux_64bit.tar.gz
tar xzf /tmp/arduino-cli.tar.gz -C /tmp && mkdir -p ~/.local/bin && cp /tmp/arduino-cli ~/.local/bin/
export PATH="$HOME/.local/bin:$PATH"

# ② PJRC 보드 인덱스 등록
arduino-cli config init
arduino-cli config add board_manager.additional_urls \
  https://www.pjrc.com/teensy/package_teensy_index.json
arduino-cli core update-index

# ③ ★ 버전을 고정한다 — 최신(1.62.0)을 깔면 지문이 안 맞는다
arduino-cli core install teensy:avr@1.58.2

# ④ 라이브러리 배치 (수령본 = JETSON_SETUP.md §5-d)
mkdir -p ~/Arduino/libraries
cp -r ~/Desktop/teensy_required_libraries_v1_4/sketchbook_libraries/* ~/Arduino/libraries/
```

★ **`Encoder` 는 복사하지 않는다.** 코어 1.58.2 가 번들한 것이 **1.4.3 으로 수령본과 동일**함을
확인했다(2026-08-05). 같은 것을 두 곳에 두면 어느 쪽이 쓰였는지 모르게 된다.

## 3. 🔴 필수 패치 — 이거 없으면 **링크에서만** 죽는다

`micro_ros_arduino` 는 사전컴파일 `.a` 를 쓰는데, **Teensy 보드 정의의 링크 레시피에 라이브러리
링커 플래그를 넣는 자리가 없다.** 그래서 다음 증상이 난다:

```
undefined reference to `rclc_executor_init'
undefined reference to `rmw_uros_sync_session'
```

⚠ **컴파일은 전부 통과하고 마지막 링크에서만 터진다.** 헤더는 찾아지므로 코드 문제로
오인하기 쉽다 — 실제로 이 저장소가 그 경로를 밟았다(08-05).

라이브러리가 패치본을 동봉한다: `micro_ros_arduino/extras/patching_boards/platform_teensy.txt`.
1.58.2 원본과의 실제 차이는 **딱 2줄**이므로, 벤더 파일을 덮어쓰지 말고 **로컬 오버라이드**로 넣는다.

```bash
CORE=~/.arduino15/packages/teensy/hardware/avr/1.58.2
cp -n $CORE/platform.txt $CORE/platform.txt.orig_backup     # 원본 백업

cat > $CORE/platform.local.txt << 'EOF'
compiler.libraries.ldflags=
recipe.c.combine.pattern="{compiler.path}{build.toolchain}{build.command.linker}" {build.flags.optimize} {build.flags.ld} {build.flags.ldspecs} {build.flags.cpu} -o "{build.path}/{build.project_name}.elf" {object_files} {compiler.libraries.ldflags} "{build.path}/{archive_file}" "-L{build.path}" {build.flags.libs}
EOF
```

바뀌는 것은 ① `compiler.libraries.ldflags` 를 정의하고 ② 링크 레시피에 그것을 끼워 넣는 것뿐이다.

⚠ **코어를 재설치·업그레이드하면 이 파일이 사라진다.** 링크 오류가 다시 나면 여기부터 본다.

## 4. ★ 무변경 재빌드로 재현성부터 검증한다 (업로드 금지)

**소스를 한 글자도 고치지 않고** 빌드해서 환경이 맞는지 먼저 본다.

> **왜 이 순서인가**: 나중에 `re-arm` 을 구현하고 문제가 생겼을 때 **"내 코드 탓인가 환경
> 탓인가"** 를 갈라야 한다(`AGENTS.md §3-6`). 기준점을 안 만들면 그 분류를 영영 못 한다.

★ **작업본은 저장소 `firmware/teensy_integrated_base/` 다** (08-05 신설). Desktop 수령본은
독립 대조군으로 남겨 두고 건드리지 않는다 — 소유 경계·수정 규칙 = `firmware/VENDOR_DROP.md`.

```bash
cd ~/ros2_ws/firmware/teensy_integrated_base_v1_4 && sha256sum -c SHA256SUMS.txt  # 무엇이 바뀌었는지 먼저
cd ~/ros2_ws/firmware
arduino-cli compile -b teensy:avr:teensy41 --output-dir /tmp/fwout teensy_integrated_base_v1_4
```

⚠ 산출물(`--output-dir`)은 **저장소 밖**으로 뺀다. 빌드물은 소스에서 다시 만들어지므로 커밋하지 않는다.

★ **폴더 이름은 `.ino` 파일 이름과 정확히 같아야 한다** — Arduino 의 규칙이다.
`teensy_integrated_base_v1_4/teensy_integrated_base_v1_4.ino` 처럼 짝이 맞지 않으면
*"Error: no valid sketch found"* 로 **컴파일 시작도 못 한다.** 폴더를 예쁘게 줄여 부르고 싶어도
바꾸지 않는다(08-05 에 실제로 이 자리를 밟아 폴더를 되돌렸다).

**2026-08-05 관측값 (기준점)**

| 항목 | 값 |
|---|---|
| 소스 해시 | `13f929cb551ce3aa75d69bb615e04de5a0794c5259501684aae626eec2412106` (빌드 전후 불변) |
| 재현 확인 | 임시 사본 빌드와 저장소 `firmware/` 빌드가 **바이트 수까지 동일**했다(08-05, 2회) |
| FLASH | code **291,100** · data **84,452** · headers **8,440** |
| RAM1 | variables **60,096** · code **156,088** · padding **7,752** |
| RAM2 | variables **12,448** |
| 바이너리에 구워진 `ARDUINO` | **10607** ← 실차 관측과 일치 |
| 바이너리에 구워진 `TEENSYDUINO` | **158** ← 실차 관측과 일치 |
| `FW_VERSION` | `handover-integrated-pi-continuous-low-speed-1.3.0` |

지문 확인 명령 (⚠ `strings` 기본 최소길이가 4라 세 글자 `158` 이 안 잡힌다 — `-n 3` 필수):

```bash
strings -n 3 /tmp/fwout/*.elf | grep -xE "158|10607"
```

⚠ **검증 상한 (과대 주장 금지)**: 확인한 것은 **환경 지문 일치·링크 성공·소스 해시 불변**이다.
**실차에 올라가 있는 바이너리와 바이트 단위로 같은지는 확인하지 않았다** — Teensy 플래시를
되읽지 않았기 때문이다. "동일 바이너리"라고 쓰지 않는다.

## 5. 업로드 안전 규칙 (아직 실행하지 않았다)

- 🔴 **모터 전력 0V 에서만 업로드한다** — XT90 분리 또는 메인 스위치 OFF. 리셋·업로드 순간
  핀 상태가 뜨면서 모터가 튈 수 있다. 바퀴는 공중에 띄운다.
- Teensy 4.1 보드 버튼은 **Program 진입**이지 reset 이 아니다. 함부로 누르지 않는다
  (`PITFALLS.md §1` 계열 — `/dev/ttyACM*` 이 사라지는 HID 전이를 D+0 에서 실제로 겪었다).
- 업로드 뒤 agent 재기동 → **전체 필드로** 정체를 확인한다. 기본 `topic echo` 는 **128자에서
  잘라** 정상 펌웨어를 불일치로 오판한다(D+0 실측):

```bash
timeout --kill-after=2s 10s ros2 topic echo /firmware/info --field data --full-length --once
```

## 6. 남은 확인 항목

| # | 항목 | 소유자·트리거 |
|---|---|---|
| 1 | 🔴 폴더명 `v1_4` 와 소스 상수 `FW_VERSION "…-1.3.0"` 불일치. `/firmware/info` 가 방송하는 것은 후자다 | 역할 A · 구동부와 다음 대면 |
| 2 | 구동부가 실제로 쓴 Teensyduino 패치 버전(1.58.**0/1/2**). 매크로는 셋 다 `158` 이라 구분되지 않는다 | 역할 A · 위와 같이 확인 |
| 3 | `re-arm` 래치 구현 주체(역할 A / 구동부)와 부정·전환 시험 설계 | `CURRENT_HANDOFF` 결정 대기 |
| 4 | aarch64(Jetson) 빌드 환경 필요 여부 | 터널 현장 재조정 시나리오가 실제로 오면 |

## 근거 문서

`REAL_ROBOT_VALUES.md §1` · `REAL_ROBOT_VALUES.md §5` · `JETSON_SETUP.md §5-d` ·
`ELECTRICAL_BASELINE.md §4` · `AGENTS.md §3` · `PITFALLS.md §1`
