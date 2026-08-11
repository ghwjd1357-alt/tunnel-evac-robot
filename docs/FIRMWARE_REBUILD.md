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

🔴 **판정하는 값은 파일 **내용**의 sha256 다** (patch 가 아니다). **2026-08-11 지문 이관 후
허용된 변경은 세 건**이고, 아래 64자리 세 개가 **이 절의 정본**이다:

🔴 **2026-08-12 — `.ino` 는 예약 32 교정으로 이미 바뀌었고, 그래서 이 지문은 지금 `rc=1` 을
낸다. 그게 의도다** (검토 §60.2 P1). `REAL_ROBOT_VALUES §1-f` ⓷ 의 순서는
`코드 수정 → 빌드 → **구판 지문으로 FAIL** → 독립 검토 → 지문 갱신` 이다. 08-12 에 한 번
6·7 을 뒤집어 검토 전에 지문을 옮겼고, 그 결과 **유일한 자동 굽기 차단이 미승인 구동 상수를
"굽어도 된다"로 통과시켰다** — 되돌렸다. 🔴 **지문은 오염 검출기이지 의미 승인자가 아니다.**
🔴 **미승인 상태의 새 지문·증감·줄수를 여기에 적지 않는다** (검토 §61.2 P2). 계수가 아직
확정 전이라 `.ino` 는 더 바뀔 수 있고, 부모 커밋의 blob 을 "새 지문"으로 적어 두면 승인 뒤
그 값을 옮겼을 때 **정상 소스인데도 계속 `rc=1`** 이 된다. 이관은 승인 시점에 `sha256sum` 을
**한 번 새로 계산**해 여기와 검사기 두 자리에만 넣는다.

```
aa8e75ec2d5884bf12ee3110b7140fc9b75ab3368865a739b558ab867f334d02
  firmware/teensy_integrated_base_v1_4/teensy_integrated_base_v1_4.ino     (기준점 대비 199,15 · 1,491줄 / 47,808 bytes · 🔴 08-12 현재 소스는 이 값이 아니다 = 의도된 rc=1)
7b3a04621f590cfe51e4f96721000c39d128dafa3dceefc8a06e5132c5de6978
  firmware/teensy_integrated_base_v1_4/rearm_gate.h        (신규 · 247,0 · 247줄 / 13,693 bytes)
f4b6d65e88fb375dfce70eec36c38b1aac0c426157338a7806f14a30f23f5663
  firmware/teensy_integrated_base_v1_4/drive_wiring.h      (신규 · 101,0 · 101줄 / 5,196 bytes)
```

```bash
sha256sum firmware/teensy_integrated_base_v1_4/{teensy_integrated_base_v1_4.ino,rearm_gate.h,drive_wiring.h}
```

허용된 세 건이 담고 있는 것: ① `ESTOP_ACTIVE_LOW true→false`(`.ino:111` 과 그 위 주석 —
되돌리면 `ELECTRICAL_BASELINE §2`-⑧ 이 재개방된다) ② re-arm 래치 배선 ③ 상태전이 정본
(`rearm_gate.h`)과 **모터 정지의 관측 가능한 자리**(`drive_wiring.h`).

⚠ **구판 기록** — 08-07 기준 허용 변경은 `ESTOP_ACTIVE_LOW` **한 건**이었고 그때의 내용
sha256 은 `1db24326…`(1307→1313줄 · 37,965→38,458 bytes · 증감 `8 2`)였다. 이 값은 **더 이상
판정에 쓰지 않는다** — 위 세 개가 대체했다. 이관 경위는 `§4-a`.

🔴 **이 세 지문은 검토 §56 조건부 수용까지 받은 내용을 가리킨다.** `§56.1` **P1 은 열린 채**
동결됐다(전제조건·재개방·완료판정 = `REAL_ROBOT_VALUES §1-f` ⓵). **지문이 `rc=0` 이라는 것은
"굽어도 되는 상태"라는 뜻이지 "결함이 없다"는 뜻이 아니다.**

### 4-a. 2026-08-11 — 지문 이관 **완료**. `rc=0` 이고, **굽기 차단이 풀렸다**

re-arm 래치(§54 보완 → §55 보완)가 스케치를 고쳤고 **파일이 둘 늘었다**. 지문은 **독립 검토
승인 뒤에만** 옮긴다는 규칙(`REAL_ROBOT_VALUES §1-f` ⓷ — 6 을 5 보다 먼저 하면 지문이 새 내용을
스스로 승인한다)에 따라 **§54→§55→§56 세 회차를 다 태운 뒤** 옮겼다.

| 언제 | 무엇 |
|---|---|
| `d0bf9f8` ~ `a7d1483` | 구현·보완. 지문 **미이관** · `firmware_precheck` `rc=1` 이 굽기를 막음 |
| `20b2a3a` | §56 결과 정본 이관(문서·인벤토리 전용). 🔴 **이 묶음에서도 지문은 안 옮겼다** |
| **이 커밋** | **지문 3개 이관 → `rc=0`.** 값은 `§4` 가 정본이고 여기서 복제하지 않는다 |

⚠ **root `.h` 도 판정 대상이다** — `arduino-cli` 는 스케치 root 의 헤더를 함께 컴파일하고
`firmware_precheck.sh` 의 `is_sketch_source()` 가 root `.h` 를 소스로 분류한다. 이관 전에는
두 헤더가 **미추적 소스**로 잡혀 `rc=1` 이었고, 그것이 의도된 방어였다.

🔴 **이관으로 사라진 것이 무엇인지 정확히 쓴다.** `rc=1` 은 *"굽지 마라"* 를 **자동으로**
강제하던 유일한 게이트였다. 지금부터 그 자리는 비어 있고, 굽기 전 확인은 **사람의 절차**
(`§5` 업로드 안전 규칙 · `JETSON_SETUP §7-c-E`)로만 남는다. **소스를 또 고치면 `rc=1` 이
돌아오는 것이 정상이고, 그때 승인 없이 지문을 다시 옮기면 안 된다.**

🔴 **`rc=0` 은 "결함 없음"이 아니다.** `§56.1` P1(응답 전송이 실패해도 보드만 무장)은
**열린 채 동결**됐다 — 전제조건·재개방·완료판정은 `REAL_ROBOT_VALUES §1-f` ⓵ 이고,
코드 보완 기한은 **R1 통과 직후·자율 발행 전**이다.

재판정이 필요하면 기본값 없이도 같은 판정을 만들 수 있다:
```bash
bash tools/firmware_precheck.sh \
  --expect firmware/teensy_integrated_base_v1_4/teensy_integrated_base_v1_4.ino=199,15,aa8e75ec... \
  --expect firmware/teensy_integrated_base_v1_4/rearm_gate.h=247,0,7b3a0462... \
  --expect firmware/teensy_integrated_base_v1_4/drive_wiring.h=101,0,f4b6d65e...
```

**2026-08-11 빌드 실측** (`arduino-cli compile -b teensy:avr:teensy41`, 링크 성공):

| | 08-11 re-arm 구현 | 08-11 §54 보완 | 08-11 §55 보완 | **08-12 예약 32(현재)** |
|---|---|---|---|---|
| FLASH code | 294,112 | 294,112 | 294,176 | **294,176** (0) |
| FLASH headers | 8,504 | 8,504 | 8,440 | **8,440** (0) |
| RAM1 variables | 62,624 | 62,656 | 62,656 | **62,656** (0) |
| RAM1 free for locals | 297,824 | 297,792 | 297,792 | **297,792** (0) |
| RAM2 variables | 12,448 | 12,448 | 12,448 | **12,448** (0) |
| `.ino` 줄수 / bytes | 1,487 / — | 1,459 / 45,638 | 1,491 / 47,808 | **1,504 / 49,154** |
| `rearm_gate.h` | — | 207 / 10,367 | **247 / 13,693** | 247 / 13,693 (무변경) |
| `drive_wiring.h` | — | — | **101 / 5,196** | 101 / 5,196 (무변경) |

⚠ **08-12 는 코드·RAM 이 전부 0 증감이다** — 바뀐 것이 `double` 상수 하나와 문자열 리터럴
둘(그리고 주석)뿐이라 그래야 맞다. 🔴 **증감이 0 이라는 사실을 "안 바뀌었다"로 읽지 않는다**
— 내용 sha256 이 `aa8e75ec…` 에서 벗어났고, 그게 판정 근거다.
⚠ **그 새 값은 여기 적지 않는다** — 계수 확정 전이라 아직 바뀐다(검토 §61.2).

🔴 **RAM 증감이 0 인 것이 §55 보완의 성질을 그대로 보여준다.** `drive_wiring.h` 는 템플릿이고
`TeensyDriveSink` 는 멤버가 없는 빈 구조체라 **런타임 비용도 데이터도 0** 이다 — 함수 포인터
대신 템플릿을 쓴 이유가 이것이다. FLASH code +64 는 `DRIVE_ARMING` 분기 몇 개다.

### 4-b. 지문 갱신 규칙 (상시)

`tools/firmware_precheck.sh` 의 `--expect` 기본값이 **같은 64자리**를 갖고 있다.
🔴 **스케치 소스를 정당하게 고치면 위 명령으로 새 값을 만들어 두 자리(이 절 · 그 스크립트)를 같이
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

### 4-c. 2026-08-11 — 🔴 **실제로 구웠다.** 굽힘 확정값은 `build` 하나다

지문 이관(`§4-a`) 뒤 같은 날 업로드까지 갔다. 서사 = `FREEZE_MANIFEST §10.25`.

| 항목 | 값 | 판정 |
|---|---|---|
| 빌드 크기 | FLASH code 294,176 · headers 8,440 · RAM1 var 62,656 · 여유 297,792 · RAM2 12,448 | ✅ `§4` 표와 전항목 일치 |
| 환경 지문 | `ARDUINO=10607` · `TEENSYDUINO=158` | ✅ |
| `firmware_precheck` | `rc=0` (지문 3개 ok · 기대 밖 0건) | ✅ |
| 🔴 **`build`** | **`Aug 11 2026 15:13:20`** | **보드 정체 판별값** |
| `.hex` sha256 | `5f89ff84975983bae50e7d48c73fa7db861714ce5cb02f9e9cb75e0d76663a7d` | 참고 |

🔴 **`--clean` 을 쓴 이유를 기록한다.** 처음 빌드는 오전 세션의 오브젝트 캐시를 재사용해
`build` 가 **`07:30:34`** 로 나왔다. 크기는 같았지만 **정체 판별에 쓰는 유일한 필드**(§6-1)가
굽는 시각과 어긋나면 나중에 "보드에 있는 게 이건가"를 못 가린다 → `--clean` 재빌드로 대체했다.
🟢 **부수 소득**: 캐시본과 clean 본의 크기가 **전항목 동일**했다 — 캐시 오염이 없었다는
관측이 하나 늘었다(재현성 관측은 08-05 2회 + 이번 1회).

🔴 **굽힘 확정은 `/firmware/info` 의 `build` 로 했다.** 업로드 뒤 실측:
`build=Aug 11 2026 15:13:20` 이 clean 빌드본과 **초 단위 일치**했고, 구판(`Aug 6 2026 22:17:38`)과
명확히 갈렸다. `/drive/enabled`(`std_msgs/msg/Bool`)·`/drive/diag`(`geometry_msgs/msg/Vector3`)가
**둘 다 존재**하는 것도 새 펌웨어의 증거다(구판에 없는 토픽).
⚠ `version`·`git_sha`·`source` 셋은 **여전히 거짓**이다(§6-1). 이번에 `.ino` 를 열지 않았으므로
의도된 결과다 — 주석 한 줄이라도 고치면 sha256 이 바뀌어 굽는 물건이 검토받은 물건과 달라진다.

**2026-08-11 관측값 (re-arm 래치 · 🔴 아직 굽지 않았다 — ⚠ 이 표는 굽기 *전* 기록이다. 굽힘 = `§4-c`)**

기준점과 같은 환경(`arduino-cli` + `teensy:avr 1.58.2` + `platform.local.txt`)에서 잰 값이다.
⚠ **이 표는 측정이지 승인이 아니다** — 판정하는 값은 여전히 **내용 sha256** 하나고, 그것은
독립 검토 뒤에 옮긴다(`REAL_ROBOT_VALUES §1-f`⓷).

| 항목 | 08-05 기준점 | 08-11 re-arm | 증분 |
|---|---|---|---|
| FLASH code | 291,100 | **294,112** | +3,012 |
| FLASH data | 84,452 | **86,500** | +2,048 |
| FLASH headers | 8,440 | **8,504** | +64 |
| RAM1 variables | 60,096 | **62,624** | +2,528 |
| RAM1 code | 156,088 | **159,096** | +3,008 |
| RAM2 variables | 12,448 | **12,448** | 0 |
| RAM1 여유(지역변수) | — | **297,824** | — |
| `ARDUINO` / `TEENSYDUINO` | 10607 / 158 | **10607 / 158** | 불변 ✅ |

소스 = 1,313 → **1,487 줄**. 새 내용 sha256(**검토 통과 전까지 옮기지 않는다**) =
`e6e15b5f55324c730524e8f706fcc90bcc9b52a071a965b39899ac438670e840`.

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
   🔴 **08-11 신설 — `arduino-cli upload` 의 종료 0 을 "구워졌다"로 읽지 않는다.** 08-11 실측에서
   출력은 `Opening Teensy Loader...` **두 줄뿐**이었고 즉시 `exit 0` 이었다. **arduino-cli 는
   Teensy Loader 를 띄우기만 하고 그 종료를 기다리지 않는다** — 위 "진행바 완주" 관찰이
   CLI 경로에서는 아예 불가능하다. 종료 0 은 *로더를 띄웠다*는 뜻이다.
   → **굽힘 판정은 `§4-c` 의 `build` 문자열로 한다.** 중간 정황으로는 `/dev/ttyACM0` **노드
   생성 시각**이 쓸 만하다(재열거 = 보드 재부팅). 08-11 에는 업로드 20초 뒤로 찍혔다:
   ```bash
   ls -l --time-style=full-iso /dev/ttyACM0     # 생성 시각이 방금이면 재부팅한 것
   ```
5. **크기 대조** — `§4` 무변경 빌드와 비교. 🔴 **Teensy 4.1 은 `Sketch uses …` 를 찍지 않는다**
   (08-06 실측). 실제 형식은 아래이고, 이 값들을 기준점으로 적는다.
   ```
   FLASH: code:291100, data:84452, headers:8440
   RAM1: variables:60096, code:156088, padding:7752    RAM2: variables:12448
   ```
   `bool` 상수 한 개 변경은 **차이 0** 이었다(08-06 실측). **1KB 이상 차이나면 중단**한다.

🟢 **검증 상태 — 실물 업로드 2회 성공**(2026-08-06 · **2026-08-11**). 명령·포트·출력은 실측이고
08-11 에도 `board list` 두 줄이 08-06 기록과 **글자 그대로 같았다**(포트 `usb3/3-2` 포함).
🟡 **다만 두 번 다 사용자가 PROGRAM 버튼을 눌렀다.**
**따라서 "udev 만으로 버튼 없이 구워진다"는 여전히 확인되지 않았다.**
- **재개방/승격 조건**: 다음 업로드에서 **버튼을 누르지 않고** 성공하면 그때 "버튼 불필요"로 올린다.
- 🔴 **08-11 정정 — 2회 연속 버튼을 눌러서 이 조건이 안 닫혔다.** 버튼을 누르면 *자동 진입이
  됐는데 버튼이 군더더기였을 가능성*도 함께 못 가른다. **관측 설계를 여기 박아 둔다**:
  다음 굽기 때 **버튼을 누르지 말고 10초 기다렸다가, 타임아웃이 날 때만 누른다.**
  성공하면 승격이고 실패해도 잃는 것이 없다 — 비용 0 의 시험을 두 번 놓쳤다.
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
| 1 | 🔴 폴더명 `v1_4` 와 소스 상수 `FW_VERSION "…-1.3.0"` 불일치. `/firmware/info` 가 방송하는 것은 후자다. **08-06 실물에서 확인**: `version` · `source=/home/park/…v1_3.ino` · `git_sha=000…` **셋 다 실제와 다르다**. 🔴 **정체 판별에 쓸 수 있는 필드는 `build`(컴파일 시각)와 매크로 2개뿐이었다** — 08-06 업로드 확정도 `build=Aug 6 2026 22:17:38` 로 했다 | ⚠ **08-12 부분 종결** — `FW_VERSION` = `rearm-latch-pi-continuous-low-speed-1.4.0` · `FW_SOURCE_PATH` = 저장소 실제 경로로 정정했다(예약 32 굽기와 같은 묶음). 🔴 **`FW_GIT_SHA` 는 `0` 그대로다** — 소스에 자기 커밋 해시를 적으면 그 편집이 해시를 다시 바꾸는 순환이라 **빌드 시 주입(`-DFW_GIT_SHA=…`)이 필요**하고, 그건 이번 최소 변경 범위 밖이다. 🔴 **그때까지 정체 판별의 정본은 여전히 `build` 다** |
| 2 | 구동부가 실제로 쓴 Teensyduino 패치 버전(1.58.**0/1/2**). 매크로는 셋 다 `158` 이라 구분되지 않는다 | 역할 A · 위와 같이 확인 |
| ✅ 3 | ~~`re-arm` 래치 구현 주체~~ — **08-11 사용자 결정: 펌웨어 구현 주체는 이제부터 항상 역할 A** 다. 부정·전환 시험 설계 = `REAL_ROBOT_VALUES §1-f`⓹ | 완료 (08-11) |
| 4 | aarch64(Jetson) 빌드 환경 필요 여부 | 터널 현장 재조정 시나리오가 실제로 오면 |

## 근거 문서

`REAL_ROBOT_VALUES.md §1` · `REAL_ROBOT_VALUES.md §5` · `JETSON_SETUP.md §5-d` ·
`ELECTRICAL_BASELINE.md §4` · `AGENTS.md §3` · `PITFALLS.md §1`
