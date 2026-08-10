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

★ **작업본은 저장소 `firmware/teensy_integrated_base_v1_4/` 다** (08-05 신설). Desktop 수령본은
독립 대조군으로 남겨 두고 건드리지 않는다 — 소유 경계·수정 규칙 = `firmware/VENDOR_DROP.md`.

```bash
cd ~/ros2_ws/firmware/teensy_integrated_base_v1_4 && sha256sum -c SHA256SUMS.txt  # 무엇이 바뀌었는지 먼저
cd ~/ros2_ws/firmware
arduino-cli compile -b teensy:avr:teensy41 --output-dir /tmp/fwout teensy_integrated_base_v1_4
```

🔴 **2026-08-06 이후 이 `sha256sum -c` 는 `실패` 를 낸다 — 그게 정상이다.** `SHA256SUMS.txt` 는
**수령 원본**의 해시라, 우리가 `ESTOP_ACTIVE_LOW` 한 줄을 고친 지금은 안 맞는 것이 맞다.
**"실패" 를 오염으로 읽지 말고 무엇이 달라졌는지로 읽는다** — 판정은 허용 diff 내용 지문이다:

```bash
bash ~/ros2_ws/tools/firmware_precheck.sh      # 종료 0 = 굽어도 된다 / 1 = 멈춘다 / 2 = 판정 불능
```

**2026-08-07 기준 허용된 변경은 딱 한 건이다** — `ESTOP_ACTIVE_LOW true→false`
(`.ino:111` 과 그 위 주석 · 1307→1313줄 · 37,965→38,458 bytes · 기준점 대비 증감 `8 2`).

🔴 **판정하는 값은 이것 하나다 — `.ino` 파일 **내용**의 sha256** (patch 가 아니다):

```
1db24326ff1f4d8100e5a1fd99f77803a5f02e8c28a0aa0c0609d6d817a90bd8
  firmware/teensy_integrated_base_v1_4/teensy_integrated_base_v1_4.ino
```

```bash
sha256sum firmware/teensy_integrated_base_v1_4/teensy_integrated_base_v1_4.ino  # 값을 다시 만드는 법
```

`tools/firmware_precheck.sh` 의 `--expect` 기본값이 **같은 64자리**를 갖고 있다.
🔴 **`.ino` 를 정당하게 고치면 위 명령으로 새 값을 만들어 두 자리(이 절 · 그 스크립트)를 같이
옮긴다.** 증감 `8 2` 는 **진단 출력일 뿐 판정에 쓰지 않는다** — git 의 diff 알고리즘이 정하는
값이라 판정에 쓰면 §50.1 의 거짓 양성이 돌아온다. 같은 `8/2` 라도 내용이 다르면 FAIL 인 것은
그대로이며, 이제 그 근거가 **내용 자체**다.

🔴 **08-07 검토 §47.1 P1 정정 — 사람이 눈으로 대조하던 절차를 종료코드로 옮겼다.** 구판 정본은
`git diff --numstat f57d454 HEAD -- 'firmware/*/*.ino'` 한 줄이었는데, **끝점을 `HEAD` 로 못
박으면 index 와 작업 트리가 비교에서 통째로 빠진다.** 실제로 `.ino` 에 커밋하지 않은 한 줄을
넣고 그대로 실행해도 결과는 여전히 기준값 `8 2` 였다(staged 로 올려도 같다). 현장에서 가장 흔한
상태 — **고쳤는데 아직 커밋 안 함** — 이 유일한 오염 게이트를 조용히 통과한 것이다.
🔴 **비교 끝점을 HEAD 로 박으면, 아직 커밋하지 않은 오염은 판정에서 통째로 빠진다.** 같은 자리에서
셋을 더 놓치고 있었다: **① 미추적 파일**은 `git diff` 에 안 보이고, **② `.h`·`.cpp`** 도
빌드에 들어가며, **③ 삭제·이름변경**은 기대 목록과 양방향으로 대조해야 잡힌다. → 판정 입력은
**기준점 → 작업 트리** + **미추적 소스**, 판정은 사람 눈이 아니라 **종료코드**다.

🔴 **08-07 검토 §48.1 P1 — ignore 규칙은 컴파일 제외 규칙이 아니다 — 숨은 소스도 판정 입력이다.**
§47 보완은 `--exclude-standard` 때문에 ignore된 `.cpp`를 `rc=0`으로 통과시켰고,
수동 확장자 목록에서 Arduino 공식 `.hh/.tpp/.ipp`도 빠졌다. 그러나 §49 재측정으로
**실제 Teensy toolchain은 공식 sketch 명세보다 넓다**는 것이 확인됐다. 설치본
`arduino-cli 1.5.2-rc.1` + `teensy:avr 1.58.2`, Teensy 4.1의 `compile_commands.json`에서
root와 `src/**`의 `.cc/.cxx`도 실제 컴파일됐다. 따라서 root 12종·`src/**` 10종을 공통 분류기로
판정하며 `data/**`·비-`src` 중첩·대문자 `.INO`·`src/*.ino`는 빌드 밖 참고로 고정한다.
스케치 트리 symlink는 Git 밖 입력을 따라갈 수 있으므로 전부 거부한다.
🔴 **`2`를 `0`처럼 읽지 않는다** — 못 본 것과 깨끗함은 다르다.

🔴 **08-07 검토 §46.2 P2 — 소유 문서를 판정에서 빼는 이유(그대로 유지).** 구판은 범위를
`-- firmware/` 로 잡아 **허용된 소유 문서 변경(`firmware/VENDOR_DROP.md` 3+/2-)까지 끌어왔다.**
그래서 정상 작업본에서도 매번 "한 줄보다 크다"가 되어, **멈추라는 지시가 항상 발동**했다.
늘 울리는 경보는 사람이 곧 무시한다 — 그러면 진짜 펌웨어 오염을 놓친다. 판정에서 빼되
**숨기지는 않는다** — 검사기가 `[참고]` 절에 **항상 같이 찍는다**(회귀 케이스 ⑨).

🔴 **08-07 검토 §50.1 P1 — 지문이 "내용"이 아니라 "이 기계의 git 렌더링"을 고정하고 있었다.**
§49 보완의 기대 지문은 `git diff … | sha256sum`, 즉 **diff 출력 텍스트**의 해시였다. 그래서
작업 트리를 **한 바이트도 안 건드리고** `core.abbrev` · `diff.context` · `diff.noprefix` ·
`diff.mnemonicPrefix` 중 아무거나 주면 `rc=1` **"오염"** 이 떴다. 게다가 지문에 들어가는
`index c7cfbd4..764db98` 의 축약 자릿수는 `core.abbrev=auto` 가 **저장소 객체 수에서 뽑으므로**,
펌웨어를 한 줄도 안 고쳐도 객체가 늘면 지문이 **스스로 만료**된다. 굽기 직전 유일한 게이트가
진단 불가능하게 멈추면 사람은 게이트를 건너뛴다 — §46.2 로 닫은 "늘 울리는 경보"의 재발이었다.
→ 판정 입력에서 **diff 출력 텍스트를 통째로 뺐다.** 허용 파일의 판정은 위 **내용 sha256** 하나다.
⚠ 느슨해진 게 아니라 **강해졌다 — 논증을 정확히 쓴다.** 내용 해시가 맞으면 작업 트리 파일의
**바이트가 기대값으로 완전히 결정**되고, 파일에서 파생되는 모든 성질(증감 포함)도 따라서 결정된다.
즉 파일에 대해 증감이 추가로 말해 줄 것이 없다. patch 해시가 더 갖고 있던 것은 파일에 대한 제약이
아니라 **환경에 대한 제약**이었고, 그건 애초에 판정할 대상이 아니었다. ⚠ 정확히 하자면 증감은
`(기준점 blob, 파일 내용, diff 알고리즘)` 의 함수라 **앞의 둘이 고정돼도 알고리즘이 바뀌면
달라질 수 있다** — 그래서 판정이 아니라 진단이다. 이 변경은 `firmware/VENDOR_DROP.md §2` 가
원래 주장하던 "허용 **내용** 일치"와 재는 것을 일치시킨다.
⚠ **허용 파일 판정이 `--baseline` 과 무관해졌다는 것도 숨기지 않는다** — 기준점은 이제
*기대 밖 변경* 탐지 전용이다(스크립트 머리말 마지막 항).

🔴 **2026-08-10 검토 §51.1 (P1) — `rc=0` 을 "조상 경로 symlink 까지 닫혔다"로 인용하지 않는다.**
**조상 디렉터리 symlink 는 사전검사가 보지 않는다**: 스케치 디렉터리(`firmware/` 의 직접 자식)를
저장소 밖 같은 이름 디렉터리로 옮기고 그 자리에 symlink 를 걸면, 기대 `.ino` 내용 해시가 맞는 한
**밖에 `rogue.cpp` 를 더 놔도 `rc=0`** 이 나온다. Codex 가 실물 toolchain 의
`compile_commands.json` 에서 그 파일이 **실제로 컴파일되는 것**을 확인했다. 원인은 세 자리다 —
`find … -mindepth 2 -type l` 이 깊이 1 을 안 보고, `[ -L "$file" ]` 이 마지막 경로 요소만 보며,
삭제 증감은 진단으로만 인쇄된다. **완료판정** = 기대 컴파일 root 까지의 **조상 경로 요소**와 그
트리 안 symlink 전량을 fail-closed 로 거부하고, 밖에 같은 `.ino` 와 추가 소스가 있어도 `rc=0` 을
내지 않으면 완료.
⚠ **이번 묶음에서 고치지 않았다** — `AGENTS.md §6` 회차 상한(이 사슬 §47→§51 = **5회차**)이
"3회차부터 P0 만"을 지시한다. 🟢 실무 노출은 낮다(스케치 폴더를 symlink 로 바꿀 이유가 없다).
**대신 굽기 전에 `firmware/` 아래를 눈으로 한 번 본다.**

이 검사가 증명하지 않는 것: **보드에 올라가 있는 펌웨어**가 이 소스라는 증거가 아니다. 저장소만
본다. 회귀 = `bash tools/test_firmware_precheck.sh` (**73 검사** — 실제 root 12종·`src/**` 10종,
같은 증감의 다른 내용·symlink 부정 회귀, 빌드 밖 네 경계·실제 저장소 역회귀, **git 렌더링 7종을
흔들어도 깨끗한 트리는 PASS·진짜 오염은 여전히 FAIL 이라는 양방향 축**, 사례 ID 1:1 강제 포함).
소유 경계·되돌리면 안 되는 이유 = `firmware/VENDOR_DROP.md §2`·`§4`.

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

## 5. 업로드 안전 규칙 (✅ 2026-08-06 실물 1회 실행)

- 🔴 **모터 전력 0V 에서만 업로드한다** — 리셋·업로드 순간 핀 상태가 뜨면서 모터가 튈 수 있다.
  바퀴는 공중에 띄운다. **0V 를 만드는 방법은 두 가지이고 전제가 다르다 → `§5-b`.**

### 5-0. 🔴 선행 — udev 규칙 (2026-08-06 실측 신설)

**이게 없으면 첫 업로드가 반드시 실패한다.** 08-06 에 실제로 실패했고, 규칙 부재가 원인이었다.

```
Teensy did not respond to a USB-based request to enter program mode.
  Cause may be missing 00-teensy.rules UDEV rules in /etc/udev/rules.d
Failed uploading: uploading error: exit status 1
```

🟢 **인터넷에서 받을 필요 없다 — 규칙 파일은 teensy-tools 안에 이미 있다.**

```bash
sudo cp ~/.arduino15/packages/teensy/tools/teensy-tools/1.58.2/00-teensy.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
# USB 를 뽑았다 다시 꽂는 것이 가장 확실하다
```

### 5-a. 업로드 절차 (✅ 2026-08-06 실측 반영)

1. **포트 확인** — `arduino-cli board list`.
   🔴 **판정 기준 정정(08-06 실측)** — 구판은 "`/dev/ttyACM0` **같은 줄**에 `Teensy`" 라고
   했으나 **실제 출력은 두 줄로 갈린다.** 그대로 읽으면 정상 상태를 중단으로 오판한다.
   ```
   /dev/ttyACM0  serial  Serial Port (USB)  Unknown
   usb3/3-2      teensy  Teensy Ports       Teensy 4.1  teensy:avr:teensy41   ← 품번은 여기만
   ```
   → **`Teensy` 문자열이 어느 줄에든 보이면 통과.** 업로드 포트는 **`usb3/3-2`(teensy 프로토콜)**
   를 쓴다. Program 버튼을 누르면 `/dev/ttyACM*` 은 사라지지만 **이 포트는 남기** 때문이다.
   🔴 **아무 줄에도 안 보이면 멈춘다** — 이미 Program 모드이거나 보드가 꺼져 있다.
2. **포트 점유 확인** — agent 가 떠 있으면 업로드가 포트를 못 연다.
   ```bash
   ps aux | grep micro_ros_agent    # 떠 있으면 먼저 종료
   ```
3. **업로드**
   ```bash
   cd ~/ros2_ws/firmware
   arduino-cli upload -b teensy:avr:teensy41 -p usb3/3-2 \
     --input-dir /tmp/fwout teensy_integrated_base_v1_4
   ```
4. **성공 관찰** — Teensy Loader 진행바 완주 → 보드 자동 재부팅 → 터미널이 오류 없이 종료.
   자동 재부팅이 없으면 보드의 **Program 버튼을 한 번** 누른다(reset 이 아니라 Program 진입).
5. **크기 대조** — `§4` 무변경 빌드와 비교. 🔴 **Teensy 4.1 은 `Sketch uses …` 를 찍지 않는다**
   (08-06 실측). 실제 형식은 아래이고, 이 값들을 기준점으로 적는다.
   ```
   FLASH: code:291100, data:84452, headers:8440
   RAM1: variables:60096, code:156088, padding:7752    RAM2: variables:12448
   ```
   `bool` 상수 한 개 변경은 **차이 0** 이었다(08-06 실측). **1KB 이상 차이나면 중단**한다.

🟢 **검증 상태 — 2026-08-06 실물 업로드 1회 성공.** 위 명령·포트·출력은 실측이다.
🟡 **다만 udev 규칙 설치 후에도 사용자가 PROGRAM 버튼을 눌렀다.**
**따라서 "udev 만으로 버튼 없이 구워진다"는 확인되지 않았다.**
- **재개방/승격 조건**: 다음 업로드에서 **버튼을 누르지 않고** 성공하면 그때 "버튼 불필요"로 올린다.
- 🔴 Teensy 4.1 보드 버튼은 **Program 진입**이지 reset 이 아니다. 브레드보드 작업 중 실수로
  눌리면 보드가 **HalfKay 부트로더(`16c0:0478`)** 로 올라와 **`/dev/ttyACM*` 이 아예 안 생긴다.**
  08-06 에 실제로 겪었다. 진단·복구 = `PITFALLS.md §1`.

### 5-b. 🔴 `VUSB-VIN` 트레이스 절단 후의 업로드 (2026-08-06 신설)

**트레이스를 자르면 USB 만으로는 Teensy 가 켜지지 않는다**(`ELECTRICAL_BASELINE.md §2`-⑪).
즉 굽는 동안에도 **5V DCDC 가 살아 있어야** 하고, 그건 **XT90 연결**을 뜻한다 —
위 "모터 전력 0V" 규칙과 정면으로 부딪힌다.

| 상황 | 0V 를 만드는 방법 |
|---|---|
| 트레이스 **절단 전** | **XT90 분리.** Teensy 는 USB 로 산다 |
| 트레이스 **절단 후** | 🔴 **E-stop 을 누른 채 XT90 연결.** 모터 가지만 0V 이고 로직은 산다 |

🔴 **아래 조건이 성립할 때만 이 우회를 쓴다.** 하나라도 아니면 쓰지 않는다.
1. **`§5-G8` 부하 차단 10회를 통과했다** (E-stop 이 실제로 끊는다는 실측 근거) — 2026-08-07 통과
2. 업로드 직전 **모터 단자 0V 를 직접 잰다** (E-stop 을 눌렀다는 사실로 대신하지 않는다)
3. 바퀴는 공중
- 업로드 뒤 agent 재기동 → **전체 필드로** 정체를 확인한다. 기본 `topic echo` 는 **128자에서
  잘라** 정상 펌웨어를 불일치로 오판한다(D+0 실측):

```bash
timeout --kill-after=2s 10s ros2 topic echo /firmware/info --field data --full-length --once
```

## 6. 남은 확인 항목

| # | 항목 | 소유자·트리거 |
|---|---|---|
| 1 | 🔴 폴더명 `v1_4` 와 소스 상수 `FW_VERSION "…-1.3.0"` 불일치. `/firmware/info` 가 방송하는 것은 후자다. **08-06 실물에서 확인**: `version` · `source=/home/park/…v1_3.ino` · `git_sha=000…` **셋 다 실제와 다르다**. 🔴 **정체 판별에 쓸 수 있는 필드는 `build`(컴파일 시각)와 매크로 2개뿐이다** — 08-06 업로드 확정도 `build=Aug 6 2026 22:17:38` 로 했다 | 역할 A · **펌웨어를 다음에 여는 묶음에서 상수 3개를 갱신** (구동부는 08-06 합의로 펌웨어에 관여하지 않는다) |
| 2 | 구동부가 실제로 쓴 Teensyduino 패치 버전(1.58.**0/1/2**). 매크로는 셋 다 `158` 이라 구분되지 않는다 | 역할 A · 위와 같이 확인 |
| ✅ 3 | ~~`re-arm` 래치 구현 주체~~ — **08-11 사용자 결정: 펌웨어 구현 주체는 이제부터 항상 역할 A** 다. 부정·전환 시험 설계 = `REAL_ROBOT_VALUES §1-f`⓹ | 완료 (08-11) |
| 4 | aarch64(Jetson) 빌드 환경 필요 여부 | 터널 현장 재조정 시나리오가 실제로 오면 |

## 근거 문서

`REAL_ROBOT_VALUES.md §1` · `REAL_ROBOT_VALUES.md §5` · `JETSON_SETUP.md §5-d` ·
`ELECTRICAL_BASELINE.md §4` · `AGENTS.md §3` · `PITFALLS.md §1`
