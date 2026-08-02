# JETSON_SETUP.md — 인수 당일(D+0) Jetson 셋업 런북 (2026-08-02 신설, S6-1·S6-2)

> **목표**: Jetson 을 **처음 켜는 사람이** 이 문서만 들고 apt 설치 → 소스 빌드 →
> `micro_ros_agent` 연결 → `/dev/teensy_drive` 인식 → `tools/d0_check.sh` 통과까지
> **검색 없이** 가는 것. 그다음 날이 `docs/D1_FIRST_STEP.md`(R3 rosbag)다.
>
> 왜 하루 안에 끝나야 하나: 로봇·Jetson 을 **인수일에 일괄 수령**(B안)하므로
> 사전 접근이 없다. D+0 하루가 무너지면 D+1 첫 스텝이 통째로 밀린다
> (`MASTER_PLAN.md §3` S6 · `REAL_ROBOT_VALUES.md §5`).

## 0. 먼저 알아야 할 것 — 이 문서의 한계 (읽고 시작한다)

⚠ **이 런북은 장비 없이 썼다.** 2026-08-02 작성 시점에 Jetson 도 Teensy 도 손에 없었다.
그래서 아래 규칙을 지켰다. **"완벽한 런북"이 아니라 "막힐 자리를 미리 드러낸 런북"이다.**

- 노트북에서 **확인할 수 있는 것은 전부 실제로 확인**했다(패키지 존재·명령 출력 형식·
  아키텍처 지원 여부). 그 근거를 각 절에 적어 뒀다.
- 확인할 수 없는 것은 **`TODO(D+0): 확인`** 으로 남기고 **확인 방법을 같이** 적었다.
  그럴듯한 값으로 칸을 채우지 않았다 (`AGENTS.md §5` 미실측 값 금지).
- ⚠ 그래서 **`TODO(D+0)` 를 만나면 멈추고 확인**해야 한다. 넘어가면 나중에
  "왜 안 되지"의 원인이 이 문서 안에 숨는다.

**누가 실행하나**: 명령은 준비돼 있지만 **실행은 사용자(역할 A)** 다.
Claude 는 SSH 비밀번호를 칠 수 없다 — 명령을 주고 결과를 해석하는 데까지가 Claude 몫이다
(`CLAUDE.md §2`). 결과를 붙여넣으면 다음 판단을 이어서 한다.

**계정**: Jetson 은 **`hanhan`**, 노트북은 `minwoo`. 헷갈리면 경로가 전부 어긋난다.

## 1. 접속과 사전 확인 (5분)

```bash
ssh hanhan@jetson.local
```

접속되면 **먼저 네 가지를 확인**한다. 이 네 줄이 뒤의 모든 선택을 바꾼다.

```bash
lsb_release -a                 # Ubuntu 22.04 (jammy) 인가
dpkg --print-architecture      # arm64 여야 한다 (노트북은 amd64)
ls /opt/ros                    # humble 이 이미 있나? 없으면 §2 로
df -h ~                        # 여유 공간 (빌드+apt 에 최소 10GB 권장)
```

| 확인 | 기대값 | 아니면 |
|---|---|---|
| OS | **Ubuntu 22.04 (jammy)** | 20.04 면 Humble apt 경로가 없다 → Docker/소스빌드로 며칠. **07-31 에 22.04 로 확인됨**(`MASTER_PLAN.md §6`) |
| 아키텍처 | **arm64** | — |
| ROS | `humble` 디렉터리 존재 | 없으면 §2 |
| 여유 공간 | 10GB+ | 부족하면 `sudo apt clean` · 불필요 JetPack 샘플 삭제 |

⚠ **인터넷 연결을 지금 확인한다.** 아래 §2·§4·§5 가 **전부 네트워크를 쓴다.**

```bash
ping -c 2 packages.ros.org
```

★ **D+0 최대 위험 1**: Jetson 에 인터넷이 없으면 apt·git clone·agent 빌드가 **전부** 막힌다.
인수일 **전에** 테더링이든 유선이든 경로를 확보해 둘 것. 확보가 안 되면 §5 안 B(Docker)도
막히므로(이미지 pull 도 네트워크다) **오프라인 대비 = §8 를 미리 읽는다.**

### 1-b. ★ 시각 동기 — **08-02 소스로 새로 발견. 안 잡으면 EKF 가 멈춘다**

```bash
timedatectl                              # NTPSynchronized=yes 인가
sudo timedatectl set-ntp true            # 아니면 켠다
timedatectl                              # 다시 확인 (수십 초 걸릴 수 있다)
```

**왜 이게 EKF 문제인가** — Teensy 는 stamp 를 이렇게 만든다:

```c
uint64_t nowNs = rmw_uros_epoch_nanos();          // agent(=Jetson) 시각을 따라간다
if (nowNs <= lastPublishedTimestampNs) {
    nowNs = lastPublishedTimestampNs + 1ULL;      // ★ 강제 단조증가
}
```

그리고 **30초마다 `rmw_uros_sync_session()` 으로 Jetson 시각에 다시 맞춘다.**

> Jetson 이 부팅 직후 엉뚱한 시각으로 시작했다가 NTP 가 붙으면서 **시각을 뒤로 되돌리면**,
> Teensy 의 stamp 는 뒤로 갈 수 없으므로 **1 ns 씩만 증가한다.**
> EKF 는 `dt ≈ 0` 을 계속 받고 **시간이 멈춘 것으로 인식** → 공분산이 발산한다.

증상은 "EKF 출력이 이상하다" 로만 보이고 원인이 시계라는 단서가 없다.
**부팅 후 NTP 가 안정된 다음에 agent 를 띄우는 것**이 가장 안전하다.

TODO(D+0): 확인 — 인터넷이 없어 NTP 를 못 잡는 경우, `sudo date -s` 로 노트북 시각과
**대충이라도 맞춘 뒤 그 이후로는 건드리지 않는다.** 정확도보다 **단조성**이 중요하다.

## 2. ROS 2 Humble 설치 (이미 있으면 건너뛴다)

```bash
sudo apt update && sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions python3-rosdep
```

`ros-humble-desktop` 이 아니라 **`ros-humble-ros-base`** 를 쓰는 이유: Jetson 에서 RViz·
Gazebo 를 돌릴 이유가 없다. 용량과 설치 시간을 아낀다. TF 트리 확인은 GUI 없이 되는
`tf2_tools` 로 한다(§4 에서 rosdep 이 같이 깔아 준다. 안 깔리면 `sudo apt install -y ros-humble-tf2-tools`).

매 터미널에서 자동으로 잡히게:

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## 3. 소스 가져오기 — ★ 함정이 두 개 있다

### 3-a. 노트북 빌드물은 못 쓴다

노트북은 x86_64, Jetson 은 aarch64 다. `build/`·`install/` 을 복사하면
**실행 시점에 알 수 없는 오류로 죽는다.** Jetson 은 **소스를 받아 직접 빌드**한다.
(반대로 **소스 자체는 아키텍처와 무관**하다 — 오프라인이면 §8 의 USB 복사가 유효하다.)

### 3-b. ★★ `sllidar_ros2` 는 저장소에 **없다** (08-02 발견)

`.gitignore` 가 `src/sllidar_ros2/` 를 제외하고 있다(남의 저장소라 우리 스냅샷에 안 넣는
관례). 노트북에는 예전에 clone 해 둔 것이 있어서 보이지만, **새로 clone 한 Jetson 에는
없다.** 그런데 `tunnel_bringup/package.xml` 이 `sllidar_ros2` 를 의존으로 선언하므로
**그대로 빌드하면 실패한다.**

> ⚠ `MASTER_PLAN.md §3` 의 포터빌리티 표에 "sllidar_ros2 = 이미 src/ 에 있음, clone 불필요"
> 라고 적혀 있던 것은 **노트북 기준**이었다. Jetson 에는 해당되지 않는다.

### 3-c. ⚠ **디렉터리를 미리 만들지 않는다** (08-02 검토 §29.2 로 고침)

구판은 이렇게 적혀 있었고, **확정적으로 실패했다**:

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws
git clone <url> .        # ← fatal: 대상 경로가('.') 이미 있고 빈 디렉터리가 아닙니다 (rc 128)
```

`git clone <url> .` 은 목적지가 **비어 있을 때만** 허용되는데, 바로 앞 줄이 `src/` 를
만들어 놓아 절대 비어 있지 않다. 런북의 정상 첫 경로가 100% 막히는 상태였다.
**`~/ros2_ws` 는 git 이 만들게 둔다.**

```bash
# ① 우리 저장소 (private — 인증이 필요하다. 아래 ⚠ 참조)
#    ★ 목적지를 인자로 준다. 미리 mkdir 하지 않는다.
cd ~
git clone https://github.com/ghwjd1357-alt/tunnel-evac-robot.git ~/ros2_ws

# ② 라이다 드라이버 (별도 저장소 — 위 3-b)
git clone https://github.com/Slamtec/sllidar_ros2.git ~/ros2_ws/src/sllidar_ros2
```

**이미 `~/ros2_ws` 가 있는 경우**(두 번째 시도·부분 실패 후 재개)는 새 설치와 섞지 않는다.
`.git` 이 있는지로 가른다 — 있으면 갱신, 없으면 **멈추고 사람이 판단**한다:

```bash
if [ -d ~/ros2_ws/.git ]; then
  cd ~/ros2_ws && git pull                    # 이미 clone 돼 있다 → 갱신
elif [ -e ~/ros2_ws ]; then
  echo "~/ros2_ws 가 있는데 git 저장소가 아니다 — 내용을 확인하고 직접 정리할 것"
else
  git clone https://github.com/ghwjd1357-alt/tunnel-evac-robot.git ~/ros2_ws
fi
# 라이다는 없을 때만 받는다
[ -d ~/ros2_ws/src/sllidar_ros2/.git ] \
  || git clone https://github.com/Slamtec/sllidar_ros2.git ~/ros2_ws/src/sllidar_ros2
```

확인 — 두 패키지가 **함께** 있어야 다음 절(rosdep)이 성립한다:

```bash
ls -d ~/ros2_ws/src/tunnel_bringup ~/ros2_ws/src/sllidar_ros2
```

⚠ **private 저장소 인증**: `git clone` 이 아이디·비밀번호를 물으면 GitHub 계정 비밀번호는
안 통한다(2021년에 막혔다). 셋 중 하나로 한다 — **인수 전에 정해 둘 것**:

| 방법 | 준비 | 비고 |
|---|---|---|
| **Personal Access Token** | GitHub → Settings → Developer settings → PAT(classic, `repo` 권한) | 비밀번호 자리에 토큰을 붙여넣는다. 가장 빠르다 |
| **SSH 키** | Jetson 에서 `ssh-keygen` → 공개키를 GitHub 에 등록 | `git@github.com:…` 주소로 clone |
| **USB 복사** | 노트북에서 소스만 복사 (§8) | 네트워크 없을 때의 유일한 길 |

TODO(D+0): 확인 — 위 셋 중 무엇을 쓸지. 확인 방법 = 인수 전에 실제로 한 번 clone 해 본다.

## 4. 의존성 설치 → COLCON_IGNORE → 빌드

### 4-a. ★ 빌드 전에 반드시 — `tunnel_sim` 을 제외한다

```bash
touch ~/ros2_ws/src/tunnel_sim/COLCON_IGNORE
```

**이 한 줄을 모르면 오전을 통째로 쓴다.** `tunnel_sim` 은 `gazebo_ros`·`gazebo_msgs` 를
의존하는데 **Jetson 에 Gazebo 를 깔 이유가 없다.** 그대로 두면 워크스페이스 전체
`colcon build` 가 여기서 멈춘다. 시뮬 자산은 노트북에만 있으면 된다.
(`COLCON_IGNORE` = "이 디렉터리는 빌드 대상이 아니다"라고 colcon 에게 알리는 빈 파일)

### 4-b. 의존성

```bash
sudo rosdep init          # 이미 했으면 "already exists" 가 뜬다 — 무시해도 된다
rosdep update
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

이 명령이 각 `package.xml` 을 읽어 필요한 apt 패키지를 자동으로 깐다. 우리 워크스페이스가
요구하는 굵직한 것들(참고용 — 손으로 깔 필요는 없다):

- `ros-humble-nav2-bringup` (Nav2 일습) · `ros-humble-slam-toolbox`
- `ros-humble-robot-localization` (EKF) · `ros-humble-robot-state-publisher`
- `ros-humble-tf2-ros-py` · `ros-humble-lifecycle-msgs` · `python3-yaml`

⚠ `rosdep` 이 **`micro_ros_agent` 는 못 깐다** — apt 에 없다(§5). 별개로 처리한다.
⚠ 실패하면 `-r` 덕분에 계속 진행하고 마지막에 목록을 보여 준다. **그 목록을 반드시 읽는다.**

### 4-c. 빌드

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

`tunnel_bringup`·`mission_manager` 는 **순수 Python** 이라 컴파일이 없어 수 초면 끝난다.
`sllidar_ros2` 는 C++ 이라 조금 걸린다(Jetson 에서 수 분 예상 — TODO(D+0): 확인, 실제 시간을 적어 둘 것).

확인:

```bash
ros2 pkg list | grep -E "tunnel_bringup|mission_manager|sllidar"
ros2 launch tunnel_bringup real_bringup.launch.py --show-args | head -30
```

`--show-args` 는 **스택을 띄우지 않고** 인자 목록만 보여 준다. 여기서
`serial_baud` 기본값이 **115200** 으로 보이면 08-02 확정값이 제대로 들어간 것이다.

## 5. `micro_ros_agent` 확보 — 2안 (S6-2)

### 5-a. 왜 이 절이 따로 있나

**apt 바이너리가 없다.** 이건 추측이 아니라 확인한 사실이다 (2026-08-02, 노트북 apt 색인):

```
apt-cache policy ros-humble-micro-ros-agent   → 아무것도 안 나온다 (패키지 없음)
apt-cache policy ros-humble-micro-ros-msgs    → 후보 1.0.0-3jammy 있음  ← 대조군
apt-cache search xrce                          → 0건
```

대조군이 나온다는 건 **색인은 살아 있는데 그 패키지만 없다**는 뜻이다. 그래서
`sudo apt install ros-humble-micro-ros-agent` 는 D+0 에 확정적으로 실패한다.

⚠ **agent 가 없으면 `/odom`·`/imu/data` 가 한 건도 안 온다.** D+0 최대 시간 소모 구간이라
**두 가지 길을 다 적어 둔다** — 하나가 막혔을 때 그 자리에서 갈아탈 수 있게.

### 5-b. 안 A — 소스 빌드 (권장: 되면 이쪽이 깔끔하다)

`micro_ros_setup` 이 agent 워크스페이스를 만들어 주고 빌드까지 해 준다.
(2026-08-02 확인: `humble` 브랜치 존재, `micro_ros_setup` 3.1.3, `create_agent_ws.sh`·
`build_agent.sh` 둘 다 있음)

```bash
# 우리 워크스페이스와 섞지 않는다 — 별도 워크스페이스에 만든다
mkdir -p ~/uros_ws/src && cd ~/uros_ws
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

sudo apt install -y python3-vcstool build-essential cmake \
                    flex bison libncurses-dev usbutils curl
rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash

ros2 run micro_ros_setup create_agent_ws.sh     # 소스 내려받기 (네트워크 필요)
ros2 run micro_ros_setup build_agent.sh         # 빌드 (수 분 ~ 십수 분)
source install/local_setup.bash
```

⚠ `create_agent_ws.sh` 는 **네트워크에서 여러 저장소를 내려받는다.** 여기서 막히면 안 B.
⚠ 이 워크스페이스를 `~/ros2_ws` 안에 만들지 말 것 — 우리 `colcon build` 가 같이 빌드하려 든다.
TODO(D+0): 확인 — 빌드 소요 시간과 실제로 성공했는지. 실패 메시지는 그대로 기록해 둔다.

### 5-c. 안 B — Docker (백업: 소스 빌드가 막히면)

2026-08-02 확인: `microros/micro-ros-agent:humble` 이미지는 **멀티아키이고 `linux/arm64`
를 포함한다**(Docker Hub manifest 목록에서 `linux/amd64`·`linux/arm64` 확인). Jetson 에서 쓸 수 있다.

```bash
sudo apt install -y docker.io
sudo usermod -aG docker $USER      # ★ 로그아웃/로그인 해야 반영된다
docker pull microros/micro-ros-agent:humble
```

⚠ Docker 로 띄우면 **네트워크 네임스페이스가 갈린다** — 그냥 띄우면 Jetson 의 ROS 노드가
agent 가 만든 토픽을 못 본다. `--net=host` 가 **필수**다. 장치도 넘겨줘야 한다.

### 5-d. agent 기동 (두 안 공통 — 아래 둘 중 하나)

```bash
# 안 A (소스 빌드) 로 깔았을 때 — 런치가 이 형태로 부른다
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/teensy_drive -b 115200

# 안 B (Docker) 로 갈 때
docker run -it --rm --net=host --device=/dev/teensy_drive \
  microros/micro-ros-agent:humble serial --dev /dev/teensy_drive -b 115200
```

★ **`-b 115200`** 이 08-02 확정값이다(구동부 3차 회신 §1. 구값 921600 은 폐기).
런치의 `serial_baud` 기본값도 115200 으로 맞춰 뒀다.

✅ **[08-02 소스로 종결] 이 숫자는 아무 의미가 없다.** 펌웨어가 쓰는 함수는

```c
set_microros_transports();      // ← 인자가 없다. baudrate 를 받지 않는다
```

**Teensy USB CDC 를 그대로 쓰므로 `-b` 값은 리눅스 tty 계층에만 전달되고 실제 전송률과
무관하다.** 아래 "대역폭이 모자란다"는 산술은 애초에 성립하지 않는 걱정이었다.
→ **안 붙는 원인 후보에서 baudrate 를 지운다.** 다른 값을 시도할 필요도 없다.

★★ **버전 정합 — 절반 해소, 절반 남음 (D+0 위험 2)**: agent 와 Teensy 안의 micro-ROS
클라이언트는 **버전이 맞아야 세션이 열린다.**

- ✅ **라이브러리 목록은 받았다** (펌웨어 소스 v1.4 헤더):
  `micro_ros_arduino` · `Encoder` · `Adafruit_BNO055` · `Adafruit_Unified_Sensor` · `Adafruit_BusIO`
- ❌ **버전 번호는 여전히 없다.** 소스에도 안 적혀 있고, 구동부도 *"소스만으로는 확정할 수
  없으므로 인수 시 개발환경에서 함께 확인한다"* 고 회신했다(최종 회신 §1).
- **그래서 안 붙으면 여전히 여기가 1순위 용의자다.**

**TODO(D+0): 확인 — 버전 번호를 받아적지 말고 `~/Arduino/libraries/` 폴더를 통째로 복사받는다.**
번호는 나중에 재현할 때 또 틀리지만, 폴더는 그 자체가 재현 수단이다. USB 하나면 된다.

**펌웨어 정체 확인 (붙은 뒤 30초)** — 소스를 받았으므로 이제 대조가 가능하다:

```bash
ros2 topic echo /firmware/info --once     # 5초 주기라 최대 5초 기다린다
```

⚠ **`version=…-1.3.0` 이라고 나오는 것이 정상이다.** 소스는 v1.4 인데 `FW_VERSION` 문자열이
1.3.0 그대로이고 `FW_GIT_SHA` 는 0 으로 채워져 있다 — **이 필드로 버전을 판별할 수 없다.**
대신 `wheel_radius=0.05698` · `kp=30.000` · `ki=5.000` 이 소스와 일치하는지 본다
(`d0_check.sh` 검사 [6] 이 이걸 자동으로 한다).

### 5-e. ★★ 기동 순서와 부팅 대기 — **08-02 소스로 확정. 순서를 바꾸면 안 붙는다**

펌웨어 소스를 읽고서야 알게 된 두 가지다. **둘 다 D+0 에 바로 걸린다.**

**① 순서: agent 를 먼저 띄우고, 그 다음 Teensy 를 리셋한다**

```
1. agent 실행 (위 5-d)
2. Teensy 리셋 버튼 (또는 USB 재삽입)
3. LED 를 보며 8.7초 기다린다 (아래 ②)
4. ros2 topic list
```

**이유**: 소스에 **재연결 로직이 아예 없다.** `setup()` 에서 micro-ROS 를 1회 초기화할
뿐이고 세션이 끊겼을 때 복구하는 경로가 없다. **agent 를 나중에 띄우거나 재시작하면
Teensy 는 그 사실을 모른 채 계속 돌기만 한다** → 토픽이 영영 안 뜬다.
→ **agent 를 재시작할 때마다 Teensy 도 리셋한다.** 이건 버그가 아니라 이 펌웨어의 사용법이다.

**② 부팅 후 약 8.7초 동안 로봇을 절대 건드리지 않는다**

```
initializeImu()        delay(700) + delay(1000)      ≈ 1.7초
calibrateGyroBias()    500샘플 × 10ms                 = 5.0초  ★ 이 동안 흔들면 오염된다
set_microros_transports() + delay(2000)              = 2.0초
────────────────────────────────────────────────────────────
micro-ROS 노드는 그 뒤에야 생성된다                     ≈ 8.7초
```

자이로 **바이어스를 이 5초 동안 측정**한다. 흔들거나 밀면 그 오차가 **그날 내내
`angular_velocity.z` 에 상수로 얹힌다** → EKF yaw 가 계속 한쪽으로 돈다.
바이어스는 `/imu/gyro_bias` 로 방송되니 의심되면 확인하고, 이상하면 **리셋해서 다시 잰다.**

**③ LED 로 상태를 읽는다 — 이게 유일한 진단 수단이다**

| Teensy LED | 의미 | 할 일 |
|---|---|---|
| **느리게 깜빡임** (250ms 주기, 약 5초) | ✅ 자이로 바이어스 측정 중 | **기다린다. 건드리지 않는다** |
| **꺼짐 / 안정** | ✅ 정상 기동 완료 | 다음 단계 |
| **빠르게 깜빡임** (100ms 주기, 무한) | 🔴 `errorLoop()` | 아래 ★ |

★ **빠른 깜빡임 = IMU 초기화 실패 또는 rcl 초기화 실패.** 소스에서 `initializeImu()` 가
**micro-ROS 초기화보다 먼저** 실행되므로, **IMU 가 죽으면 micro-ROS 노드가 아예 안 뜬다.**

> ⚠ **이때 증상이 "통신 문제"처럼 보인다** — agent 는 정상적으로 뜨고 포트도 열려 있는데
> `ros2 topic list` 만 비어 있다. **원인은 I2C 에 물린 BNO055 다.**
> 케이블·baudrate·agent 버전을 몇 시간 뒤지게 만드는 자리이므로, **토픽이 하나도 없으면
> 가장 먼저 LED 를 본다.**

**붙었는지 보는 법**: agent 터미널에 세션 수립 로그가 뜨고, 다른 터미널에서

```bash
ros2 topic list | grep -E "odom|imu"
```

에 `/odom`·`/imu/data` 가 보이면 성공이다. 안 보이면 **위 ③ LED 를 먼저 본 뒤** §8.

## 6. udev 규칙 — `/dev/teensy_drive`

장치 번호(`/dev/ttyACM0`)는 꽂는 순서에 따라 바뀐다. 고정 이름을 붙인다.
규칙 파일과 **실측 방법**은 `tools/udev/99-teensy-drive.rules` 안에 다 적어 뒀다.

```bash
# ① Teensy 를 꽂고 VID/PID 를 실측한다
udevadm info -q property -n /dev/ttyACM0 | grep -E '^ID_VENDOR_ID=|^ID_MODEL_ID=|^ID_SERIAL_SHORT='

# ② 그 값을 규칙 파일의 XXXX / YYYY 자리에 넣는다 (편집기로)
nano ~/ros2_ws/tools/udev/99-teensy-drive.rules

# ③ 설치하고 다시 읽힌다
sudo cp ~/ros2_ws/tools/udev/99-teensy-drive.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
# USB 를 뽑았다 다시 꽂는 것이 가장 확실하다

# ④ 확인
ls -l /dev/teensy_drive
```

TODO(D+0): 확인 — `idVendor`/`idProduct` 실측값. **추측으로 채우지 않는다**(위 ① 이 확인 방법).

## 7. 연결 판정 — `tools/d0_check.sh`

여기까지 왔으면 **판정**한다. 사람이 눈으로 보는 것으로 끝내지 않는다.

### 7-a. ★ 먼저 EKF 를 띄운다 (08-02 검토 §29.3 로 신설)

**별도 터미널**에서 — 그대로 켜 둔 채 다음 절로 간다:

```bash
ros2 run robot_localization ekf_node --ros-args \
  --params-file ~/ros2_ws/src/tunnel_bringup/config/ekf_real.yaml
```

**왜 지금 띄우나**: `d0_check.sh` 의 QoS 검사는 *"구독자 쪽 QoS 가 발행자와 맞물리는가"* 를
본다. 그런데 구판 런북은 agent 만 띄운 상태로 검사를 돌리게 짜여 있어 **구독자가 0개**였고,
검사는 그 0개를 "전부 매칭됨"으로 통과시켰다. **아무도 안 보는 것을 보고 "봤다"고 말한**
셈이다 — 이제 EKF 엔드포인트가 없으면 FAIL 한다.

⚠ **노드 이름을 바꾸지 말 것.** yaml 최상단 키가 `ekf_filter_node:` 라, `-r __node:=…` 로
이름을 바꾸면 파라미터가 **에러 없이** 하나도 안 붙는다.
⚠ EKF 는 모터를 돌리지 않는다 — `/odom`·`/imu/data` 를 받아 TF 를 내보낼 뿐이다. D+0 에
띄워도 안전하다.

### 7-b. 판정

```bash
cd ~/ros2_ws
bash tools/d0_check.sh
```

무엇을 보는가 — **8 검사** (자세한 근거는 스크립트 머리말):

| # | 검사 | 통과 기준 |
|---|---|---|
| 1 | 시리얼 장치 | `/dev/teensy_drive` 존재 |
| 2·3 | `/odom`·`/imu/data` 주기 | **관측 창을 같은 관측자가 끝까지 채움** → 평균 ≥ 하한 · **최대 간격 ≤ EKF 한 주기** · **표본 수**가 창을 채움 · **창 끝에도 수신** |
| 4 | `/odom` QoS | 발행자 **RELIABLE**(소스 v1.4) · `ekf_filter_node` 가 **실제로 구독 중** · 조합 호환 |
| 5 | `/imu/data` QoS | 발행자 **BEST_EFFORT**(소스 v1.4) · 위와 동일 |
| 6 | 전진 부호 | 바퀴를 손으로 앞으로 굴리면 `linear.x > 0` |
| 7 | 펌웨어 정체 | `/firmware/info` 의 `wheel_radius=0.05698` · `kp=30 ki=5` |
| 8 | E-stop 배선 | 버튼을 **누르면** `/estop/state` 가 `true` 로 바뀐다 |
| — | 재확인 | 종료 직전에 `ekf_filter_node` 가 **아직 살아 있는가**(마지막 `topic info` 가 **rc 0 으로 완주**했을 때만 판독) |

★ **숫자 계약은 이 표에 적지 않는다** (08-03 검토 §31.2·§32.2). 하한 `HZ_MIN`·상한
`GAP_MAX_MS` 의 **정본은 `tools/d0_check.sh` 머리말의 상수 한 곳**이고, 스크립트가 판정할 때마다
`… (EKF 한 주기 33.33ms 이내)` 처럼 **자기 계약을 출력에 함께 찍는다.** 문서에 같은 숫자를
복사해 두면 R3 실측으로 계약을 갱신할 때 **런북만 옛 숫자로 남는다** — 실제로 활성 런북
두 곳에서 그 드리프트가 났다. 현장에서는 **화면에 찍힌 값**을 근거로 읽는다.

⚠ **발행자 QoS 는 토픽마다 다르다** — `/odom` 은 RELIABLE, `/imu/*` 만 BEST_EFFORT 다
(펌웨어 소스 v1.4 의 `rclc_publisher_init_default` vs `_best_effort`). 예전 표에 적혀 있던
"발행자 BEST_EFFORT" 단일값은 **틀렸다** — 그대로 뒀으면 정상 로봇을 계약 위반으로 읽었다.

- 종료 코드 **0 = 전량 통과** · 1 = 실패 · **2 = 불완전**(건너뛴 검사 있음).
  2 를 통과로 기록하지 않는다.
- 검사 6·8 은 **사람이 손을 써야** 한다. 스크립트는 **모터에 명령을 보내지 않는다** —
  D+0 의 로봇 상태는 R0(바퀴 공중)이고, 검증 안 된 스택이 모터를 돌리는 것은 순서가 뒤집힌 것이다.
  각각 `--no-sign`·`--no-estop` 으로 따로 끌 수 있고, 끄면 종료 2 다.
- **E-stop 을 지금 누를 수 없으면 `s` + Enter** 로 건너뛴다. 그러면 "배선 없음"이 아니라
  **"확인 못 함"** 으로 기록된다 — 안 눌러 본 것을 결함으로 적으면 그 기록이 다음 판단을 오염시킨다.
- ⚠ 통과해도 **주기의 상한이 증명된 것은 아니다**(관측 창이 짧다). 상한 판정은 R3 rosbag 이다.

★ **구동부가 자리에 있을 때 돌린다.** 최종합의서에도 "인수 당일 함께 확인" 으로 넣어 뒀다.
실패를 그 자리에서 보면 10분이고, 돌아간 뒤에 보면 며칠이다.

## 8. 막힐 자리와 대처 (미리 읽어 둘 것)

| 증상 | 먼저 볼 것 | 대처 |
|---|---|---|
| `git clone` 이 인증을 요구한다 | private 저장소다 | §3 의 PAT/SSH/USB 중 하나 |
| `colcon build` 가 gazebo 에서 실패 | `COLCON_IGNORE` 를 안 만들었다 | §4-a |
| `sllidar_ros2` 없다고 실패 | 저장소에 포함돼 있지 않다 | §3-b 의 별도 clone |
| agent 가 안 붙는다 | 장치·권한·버전 | `ls -l /dev/teensy_drive` → `groups`(dialout) → **micro_ros_arduino 버전**(§5-d) |
| `topic list` 에 `/odom` 이 없다 | agent 세션이 안 열렸다 | agent 터미널 로그를 그대로 읽는다. 케이블·전원부터 |
| `topic hz` 가 아무것도 안 찍는다 | **QoS 탓이 아니다** | 08-02 실측: `topic hz` 는 기본 인자로 BEST_EFFORT 를 본다. 정말 안 오는 것이다 |
| 노드는 뜨는데 EKF 만 조용하다 | QoS 불일치 | `d0_check.sh` 검사 4·5 가 잡는다 — 단 **EKF 를 띄운 상태**여야 한다(§7-a) |
| `d0_check` 이 "구독자가 하나도 없다" FAIL | EKF 를 안 띄웠다 | §7-a 를 먼저 한다. 이건 로봇 고장이 아니라 순서 문제다 |
| `d0_check` 이 "표본이 N개뿐" FAIL | 창의 일부만 살아 있었다 | 평균이 정상이어도 믿지 않는다. agent 로그에서 재연결 흔적을 본다 |
| `d0_check` 이 "관측자가 rc=N 으로 먼저 끝났다" FAIL | **8초 창이 성립하지 않았다** (08-03 검토 §30.3) | `ros2 topic hz` 자체가 죽은 것이다 — DDS·daemon·agent 를 본다(`ros2 daemon stop` 후 재시도). 요약이 정상 모양이어도 그 표본은 창의 일부다 |
| 인터넷이 없다 | apt·clone·agent 빌드가 전부 막힌다 | ↓ 아래 오프라인 대비 |

**오프라인 대비 (네트워크가 없을 때)** — 소스는 아키텍처와 무관하므로 복사가 통한다:

```bash
# 노트북에서 (build/install/log 를 빼고 소스만)
rsync -av --exclude build --exclude install --exclude log --exclude .git \
      ~/ros2_ws/ /media/USB/ros2_ws/
```

⚠ 그래도 **apt 패키지와 agent 는 복사로 해결되지 않는다.** 그래서 §1 에서 네트워크를
먼저 확인하라고 한 것이다. 인수 전에 확보하는 것이 유일한 진짜 대비다.

## 9. `TODO(D+0)` 전량 목록 — **10건** (이 문서에서 확인해야 할 것)

착수 전에 이 목록을 한 번 읽고, 확인할 때마다 결과를 **이 문서에 적어** 다음 사람에게 남긴다.

| # | 무엇 | 확인 방법 | 절 |
|---|---|---|---|
| 1 | ROS 2 Humble 설치 여부 | `ls /opt/ros` | §1 |
| 2 | 인터넷 연결 | `ping -c 2 packages.ros.org` | §1 |
| 3 | private 저장소 인증 수단 | 인수 전에 실제로 한 번 clone | §3 |
| 4 | `colcon build` 소요 시간 | 실제로 재고 적는다 | §4-c |
| 5 | agent 확보 성공 여부(A안/B안) | §5-d 의 `topic list` | §5 |
| 6 | **`micro_ros_arduino` 버전** | ★ 번호를 묻지 말고 `~/Arduino/libraries/` **폴더를 통째로 복사**받는다 | §5-d |
| 7 | Teensy `idVendor`/`idProduct` | `udevadm info -q property …` | §6 |
| 8 | `robot_localization` 버전과 구독 QoS | `d0_check.sh` 검사 4·5 — **EKF 를 띄운 뒤**(§7-a)여야 판정이 성립한다 | §7 |
| 9 | **NTP 동기 여부** ★08-02 신설 | `timedatectl` → `NTPSynchronized=yes` | §1-b |
| 10 | **E-stop 배선 여부** ★08-02 신설 | `d0_check.sh` **검사 8** (버튼을 눌러야 한다. 못 누르면 `s` = 확인 못 함) | §7 |

## 10. 다음 단계

`d0_check.sh` 가 **종료 0** 이면 D+0 는 끝이다. 다음은 **`docs/D1_FIRST_STEP.md`** —
agent → TF 트리 → EKF → **R3 rosbag** 순서로 간다.

## 근거 문서

`MASTER_PLAN.md §3` · `MASTER_PLAN.md §6` · `MASTER_PLAN.md §7` ·
`REAL_ROBOT_VALUES.md §1` · `REAL_ROBOT_VALUES.md §4` · `REAL_ROBOT_VALUES.md §5` ·
`CLAUDE.md §2` · `AGENTS.md §5` · `D1_FIRST_STEP.md §1`
