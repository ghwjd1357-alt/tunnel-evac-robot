# JETSON_SETUP.md — 인수 당일(D+0) Jetson 셋업 런북 (2026-08-02 신설, S6-1·S6-2)

> **목표**: Jetson 을 **처음 켜는 사람이** 이 문서만 들고 apt 설치 → 소스 빌드 →
> `micro_ros_agent` 연결 → `/dev/teensy_drive` 인식 → `tools/d0_check.sh` 통과까지
> **검색 없이** 가는 것. 그다음 날이 `docs/D1_FIRST_STEP.md`(R3 rosbag)다.
>
> 왜 하루 안에 끝나야 하나: 로봇·Jetson 을 **인수일에 일괄 수령**(B안)하므로
> 사전 접근이 없다. D+0 하루가 무너지면 D+1 첫 스텝이 통째로 밀린다
> (`MASTER_PLAN.md §3` S6 · `REAL_ROBOT_VALUES.md §5`).

## 0. 먼저 알아야 할 것 — 이 문서의 한계 (읽고 시작한다)

⚠ **이 런북은 장비 없이 썼고(2026-08-02), 2026-08-03 에 실차에서 §1~§7 을 1회 완주했다.**
그래서 **절마다 신뢰도가 다르다** — 아래 표를 먼저 보고 어느 절이 아직 종이 위인지 안다.

| 신뢰도 | 절 | 근거 |
|---|---|---|
| ✅ **실차 1회 완주** | §1 · §3 · §4 · §5 · §6 · §7-b | 08-03 Jetson·Teensy 실행. 결과는 `§9` 표 |
| ⚠ **부분만 실행** | §11-d · §11-e · §11-f | 라이다 없이 odom·IMU·EKF 만. `/scan` 포함 판정은 미실행 |
| ❌ **실차에서 한 번도 안 돌았다** | **§7 검사 8**(E-stop) · **§7-c R0~R2** · §11-b · §11-c(라이다) | 물리 E-stop 미설치·라이다 미장착 |

❌ 행은 **종이 위의 절차**다. 여기서 막히면 그것은 장비 이상이 아니라 **런북이 틀린 것일 수
있다** — 그 자리에서 문서를 고친다. 아래 작성 규칙은 그대로 유지한다.

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

🔴 **인수 전 선행 게이트 — private 저장소 인증**: `git clone` 이 아이디·비밀번호를 물으면
GitHub 계정 비밀번호는 안 통한다(2021년에 막혔다). **1차 경로는 HTTPS + PAT, 비상 경로는 USB
소스 복사**로 확정한다. SSH 키는 PAT를 쓰지 않기로 바꿀 때의 대안이다.

★ **08-03 실제로 통과한 것은 fine-grained PAT 다** (`§9` 표 3번). classic 과 권한 모델이
달라서 "classic 을 만들라"는 안내로는 재현이 안 된다 — **실측 경로를 1차로 적는다.**

| 방법 | 준비 | 비고 |
|---|---|---|
| **PAT — fine-grained (1차 · 08-03 실측 통과)** | GitHub → Settings → Developer settings → Personal access tokens → **Fine-grained** → **Repository access = 이 저장소 선택** + **Permissions → Contents: Read** 이상 | 비밀번호 프롬프트에 토큰을 붙여넣는다. 권한을 이 저장소로만 좁힐 수 있어 안전하다 |
| **PAT — classic (대안)** | GitHub → Settings → Developer settings → PAT(classic, `repo` 권한) | `repo` 는 계정의 **모든** private 저장소를 연다. fine-grained 가 막힐 때만 |
| **SSH 키** | Jetson 에서 `ssh-keygen` → 공개키를 GitHub 에 등록 | `git@github.com:…` 주소로 clone |
| **USB 복사 — 비상** | 노트북에서 소스만 복사 (§8) | 네트워크 없을 때의 유일한 길 |

⚠ PAT를 URL(`https://TOKEN@…`)이나 명령 인자에 넣지 않는다. shell history·로그·문서에 남는다.
위의 평범한 HTTPS `git clone`을 실행하고, Username에는 GitHub ID, Password 프롬프트에는 PAT를
붙여넣는다(화면에 표시되지 않는 것이 정상). `gh auth status`와 Git HTTPS 자격증명은 별개이므로
`gh` 로그인 여부로 clone 성공을 대신 판정하지 않는다.

TODO(D+0): 확인 — **실제 Jetson에서** 위 HTTPS 명령으로 private 저장소를 한 번 clone하고
`git -C ~/ros2_ws rev-parse HEAD`가 원격 `main`과 같은 40자 SHA를 출력해야 이 게이트가 닫힌다.
인수 전에 못 하면 D+0 첫 작업으로 넘기는 것이 아니라 **D+0 착수 차단 상태**로 기록하고 USB
비상 경로를 준비한다. 2026-08-03 노트북에서는 HTTPS private clone을 실제 통과했지만, 노트북의
자격증명이 Jetson에 이전되는 것은 아니므로 Jetson 통과 증거로 승격하지 않는다.

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
TODO(D+0): 확인 완료 (2026-08-03) — **4패키지 전체 28초, 종료 0**. `sllidar_ros2` 가 C++ 이라
가장 오래 걸리지만 "수 분"은 아니었다. ★ **28초가 기준선이다** — 다음에 몇 분이 걸리면
정상이 아니라 **이상 신호**로 보고 원인을 찾는다(스왑·전원 모드·디스크). 관측된 경고는
`sllidar_ros2` 외부 SDK 의 C++ 경고뿐이다.

확인:

```bash
ros2 pkg list | grep -E "tunnel_bringup|mission_manager|sllidar"
D0_SHOW_ARGS=/tmp/d0_real_bringup_show_args.txt
ros2 launch tunnel_bringup real_bringup.launch.py --show-args >"$D0_SHOW_ARGS"
grep -A3 -B1 "'serial_baud':" "$D0_SHOW_ARGS"
```

`--show-args` 는 **스택을 띄우지 않고** 인자 목록만 보여 준다. 출력을 파일에 완주한 뒤
필요한 절만 읽는다(`head` 로 파이프를 먼저 닫으면 정상 `ros2` 가 `BrokenPipeError` 를 낸다). 여기서
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
TODO(D+0): 확인 완료 (2026-08-03) — `micro_ros_setup` HEAD
`af209288676e5f02ac7c6d419b8ad157d3bed14e`에서 agent 소스 생성 종료 0,
agent 2패키지 빌드 종료 0·98초. `/home/hanhan/uros_ws/install/micro_ros_agent`의
실행 파일을 확인했고, agent 먼저 기동 → USB 전원 재인가 뒤 8개 펌웨어 토픽을 수신했다.

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

🔴 **선행 — `source` 를 빼먹으면 "패키지를 못 찾는다"에서 멈춘다** (2026-08-11 실측 신설).
agent 는 **`~/uros_ws`** 에 있고 우리 워크스페이스가 아니다(§5-b 가 일부러 갈라 놨다 —
`~/ros2_ws` 안에 두면 우리 `colcon build` 가 같이 빌드하려 든다). 그런데 Jetson `~/.bashrc` 는
**humble + `~/ros2_ws/install` 만** 자동 source 한다. 즉 **agent 워크스페이스는 매번 손으로
source 해야 한다.** 이 한 줄이 구판 문서에 없어서 08-11 현장에서 실제로 막혔다.

```bash
source ~/uros_ws/install/local_setup.bash      # 🔴 이거 먼저. 없으면 아래가 "package not found"
```

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

**TODO(D+0): 확인 완료 (2026-08-05) — 폴더 전체 수령.**
받은 곳 = 노트북 `~/Desktop/teensy_required_libraries_v1_4/` (1946 파일 · 117MB).
번호를 받아적지 않고 폴더를 통째로 받았다 — 폴더 그 자체가 재현 수단이다.

| 라이브러리 | 버전 | 소스가 요구하는 헤더 |
|---|---|---|
| `micro_ros_arduino` | **2.0.8-humble** | `micro_ros_arduino.h`·`rclc`·`rmw_microros` |
| `Encoder` (보드 라이브러리) | **1.4.3** | `Encoder.h` |
| `Adafruit_BNO055` | **1.6.4** | `Adafruit_BNO055.h` |
| `Adafruit_Unified_Sensor` | **1.1.15** | `Adafruit_Sensor.h` |
| `Adafruit_BusIO` | **1.17.4** | (BNO055 의존성 — 직접 include 없음) |

`Wire.h`·`Arduino.h` 는 Teensyduino 내장이라 별도 수령 대상이 아니다.
`.ino` 의 `#include` 전수와 대조해 **5개가 요구를 정확히 덮는 것**을 확인했다.

★ **가장 중요한 확인 — 사전컴파일 정적 라이브러리가 살아 있다**:
`micro_ros_arduino/src/imxrt1062/fpv5-d16-hard/libmicroros.a` **8,360,694 bytes**
(imxrt1062 = Teensy 4.x). 이 파일이 없으면 링크 단계에서 죽는다. 자주 누락되는 자리다.

무결성: 심링크 0 · 0바이트 1건뿐이며 그 1건(`uxr/client/core/session/time_sync.h`)은
**저장소 전체에서 아무 것도 include 하지 않는다**(실제 사용되는 것은
`rmw_microros/time_sync.h` 2052 bytes). 상류 배포판의 빈 placeholder이지 전송 손상이 아니다.

보존 해시 (`§11-g` 규정):
```text
라이브러리 전체 매니페스트 sha256 = 1f349c4474e46180200857bf1377fabe0d390097ebaab65dafdfadce39e1cb78
  (생성: find . -type f -exec sha256sum {} + | sort -k2)
펌웨어 소스 sha256                = 13f929cb551ce3aa75d69bb615e04de5a0794c5259501684aae626eec2412106
  (~/Desktop/teensy_integrated_base_v1_4/SHA256SUMS.txt 대조 성공)
```

🔴 **미해결 — 버전 문자열이 폴더명과 다르다**: 폴더는 `v1_4` 인데 소스 상수는
`FW_VERSION[] = "handover-integrated-pi-continuous-low-speed-1.3.0"` 이고, `/firmware/info` 가
방송하는 값이 이것이다. **어느 쪽이 정본인지 구동부에 확인한다** — 버전으로 판정하는 자리에서
갈린다. 소유자 = 역할 A · 트리거 = 구동부와 다음 대면.

★ **환경 지문**: 소스가 `STRINGIFY(ARDUINO)`·`STRINGIFY(TEENSYDUINO)` 를 그대로 굽는다.
실차 관측값 `arduino_macro=10607`(= Arduino IDE 2.x·arduino-cli 계열) ·
`teensyduino_macro=158`(= Teensyduino 1.58). **재빌드 환경이 맞는지는 이 두 숫자로 판정한다.**

**펌웨어 정체 확인 (붙은 뒤 30초)** — 소스를 받았으므로 이제 대조가 가능하다:

```bash
timeout --kill-after=2s 10s ros2 topic echo /firmware/info --field data --full-length --once
# 5초 주기라 최대 5초 기다린다. --full-length 없이는 긴 문자열이 128자에서 잘린다.
```

**D+0 실측(2026-08-03)** — 수신 종료 0. `build=Aug 2 2026 06:46:55`,
`source=/home/park/robot_firmware/teensy_integrated_pi_continuous_low_speed_v1_3.ino`,
`transport=serial`, `baud=115200`, `wheel_radius=0.05698`, `control=PI`,
`kp=30.000`, `ki=5.000`, `kd=0.000`, 라이브러리 목록 5개가 소스 전제와 일치했다.

⚠ **`version=…-1.3.0` 이라고 나오는 것이 정상이다.** 소스는 v1.4 인데 `FW_VERSION` 문자열이
1.3.0 그대로이고 `FW_GIT_SHA` 는 0 으로 채워져 있다 — **이 필드로 버전을 판별할 수 없다.**
대신 `wheel_radius=0.05698` · `kp=30.000` · `ki=5.000` 이 소스와 일치하는지 본다
(`d0_check.sh` 검사 [7] 이 이걸 자동으로 한다).

### 5-e. ★★ 기동 순서와 부팅 대기 — **08-02 소스로 확정. 순서를 바꾸면 안 붙는다**

펌웨어 소스를 읽고서야 알게 된 두 가지다. **둘 다 D+0 에 바로 걸린다.**

**① 순서: agent 를 먼저 띄우고, 그 다음 Teensy USB 전원을 재인가한다**

```
1. agent 실행 (위 5-d)
2. Teensy USB 를 뺐다가 다시 꽂아 전원을 재인가한다
3. LED 를 보며 8.7초 기다린다 (아래 ②)
4. ros2 topic list
```

🔴 **Teensy 4.1의 보드 버튼을 누르지 않는다.** 그 버튼은 reset이 아니라 **Program
pushbutton**이다. 누르면 사용자 펌웨어가 멈추고 HalfKay bootloader(`16c0:0478` HID)로
들어가 `/dev/ttyACM*`가 사라진다. 2026-08-03 D+0에서 실제로 `16c0:0483` Serial →
`16c0:0478` HID 전이를 재현했다. **13~17초 길게 누르면 플래시 전체 삭제와 기본 blink
복원까지 실행될 수 있으므로 절대 시도하지 않는다.** Teensy 4.1은 하드웨어 reset 신호도
외부에 제공하지 않는다. 애플리케이션 재시작은 이 런북에서 **USB 전원 재인가**로만 한다.

**이유**: 소스에 **재연결 로직이 아예 없다.** `setup()` 에서 micro-ROS 를 1회 초기화할
뿐이고 세션이 끊겼을 때 복구하는 경로가 없다. **agent 를 나중에 띄우거나 재시작하면
Teensy 는 그 사실을 모른 채 계속 돌기만 한다** → 토픽이 영영 안 뜬다.
→ **agent 를 재시작할 때마다 Teensy USB 전원도 재인가한다.** 이건 버그가 아니라 이
펌웨어의 사용법이다.

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

# ② 실측값이 규칙 파일의 값과 같은지 대조한다
nano ~/ros2_ws/tools/udev/99-teensy-drive.rules

# ③ 설치하고 다시 읽힌다
sudo cp ~/ros2_ws/tools/udev/99-teensy-drive.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
# USB 를 뽑았다 다시 꽂는 것이 가장 확실하다

# ④ 확인
ls -l /dev/teensy_drive
```

TODO(D+0): 확인 완료 (2026-08-03) — `/dev/ttyACM0`, `idVendor=16c0`,
`idProduct=0483`, `ID_SERIAL_SHORT=20379630`. 위 ①의 실제 Jetson 출력으로 확정했고,
재삽입 뒤 `/dev/teensy_drive -> ttyACM0` 및 `DEVLINKS` 반영을 확인했다.

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
- **D+0 현장 사실(2026-08-03)**: 이번에는 "누를 수 없음"이 아니라 **물리 버튼 자체가 없음**을
  사용자가 확인했다. 구동부는 연결된 노트북/Jetson에서 `Ctrl+C`로 정지하라고 안내했고,
  사용자는 E-stop 항목을 임시 생략해 연결 검사를 계속하기로 결정했다. 그러나 `Ctrl+C`는
  ROS 프로세스의 정상 종료 요청일 뿐 독립 전원 차단이 아니므로 **검사 8 PASS로 승격하지
  않는다**. `--no-estop` 종료 2로 남기고, 모터 명령 시험 전 물리 차단 수단을 다시 확인한다.
- ⚠ 통과해도 **주기의 상한이 증명된 것은 아니다**(관측 창이 짧다). 상한 판정은 R3 rosbag 이다.

**D+0 1차 실행(2026-08-03 13:48)** — 장치·두 토픽 주기·최대 간격·QoS·전진 부호와
종료 시점 EKF 구독 유지가 모두 통과했다. `/firmware/info`는 별도 전체 필드 조회에서
`wheel_radius=0.05698`, `kp=30.000`, `ki=5.000`이 확인됐지만, 당시 판정기가 기본 128자
축약 출력의 `...` 뒤 값을 찾다가 거짓 FAIL을 냈다. 판정 명령에 `--field data
--full-length`를 추가했으며, 이 실행은 수정판 재실행 전이므로 최종 통과로 기록하지 않는다.

**D+0 2차 실행 후 종료 확인 분류** — `/odom --once` 격리 5회는
`rc=0,124,0,0,0`, 성공 지연도 1.996~2.315초였다. 같은 실행의 8초 주기창·후속 전진 부호와
종료 시 EKF 구독은 정상이어서 센서 중단으로 단정할 수 없다. 종료 확인의 기존 3초 상한이
DDS 발견 지연과 너무 가까운 판정기 결함으로 분류됐고, 주 측정창과 같은 8초 유한 상한으로
보완했다. 수정판 재실행 전이므로 이 결과도 최종 통과로 승격하지 않는다.

**D+0 수정판 재실행 결과** — `--no-manual`로 실행해 자동 검사 전부 통과, 종료 2였다.
이 실행은 전진 부호와 E-stop 두 항목을 생략했지만 전진 부호는 직전 두 실행에서 각각
`linear.x=+0.1502`, `+0.1459m/s`로 이미 확인됐다. 따라서 장치·주기·간격·QoS·전진 부호·
펌웨어 정체·종료 시 EKF 생존까지는 확인 완료다. 물리 E-stop은 여전히 없으므로 검사 8과
D+0 전량 통과는 미완료로 유지한다.

★ **구동부가 자리에 있을 때 돌린다.** 최종합의서에도 "인수 당일 함께 확인" 으로 넣어 뒀다.
실패를 그 자리에서 보면 10분이고, 돌아간 뒤에 보면 며칠이다.

### 7-c. ★ `d0_check` 통과 뒤 R1·R2 사전 실측 — R3보다 먼저

`d0_check.sh` 종료 0은 **연결·입력 안전 게이트**다. `MASTER_PLAN.md §3`의 순서는
R0→R1→R2→R3이므로, 다음 날 R3 런북으로 넘어가기 전에 아래 세 실측을 닫는다.

**주행 허용 전제 — 하나라도 아니면 모터 명령 금지**:

- 검사 8에서 E-stop `false→true` 전환을 실제로 확인했다.
- `REAL_ROBOT_VALUES.md §1-e`의 공통 B+ 물리 차단을 R0에서 확인했다. `d0_check` 검사 8은
  GPIO 상태 입력만 보며 컨택터 출력 0V나 자동 재가동 방어를 대신 증명하지 않는다.
- E-stop 해제 뒤 계속 발행 중인 비영 명령으로 재가동하지 않고, 새 zero 0.5초 확인과 명시적
  ROS enable 뒤 새 명령만 허용하는 부정·전환 시험을 통과했다.
  ✅ **충족 — 08-11 굽기로 re-arm 이 보드에 올라갔고 `§7-c-E` 13행 전항목을 통과했다.**
  🔴 배선을 만지거나 펌웨어를 다시 구우면 이 조건은 **13행을 다시 하기 전까지 회수**된다.
- R0에서 `/cmd_vel` 단절 뒤 watchdog 정지를 확인했다.
  🔴 **2026-08-07 관측(08-10 재산출)은 516.2~532.0ms 로 구 기준(총 500ms)을 넘었고, 그 기준
  자체가 펌웨어 구조상 달성 불가임이 드러났다.** 08-11 에 **1차 증거(영상)를 분석**하고
  **결정 1 = 안 ⓐ(기준 재정의)** 가 내려졌다 — 🔴 **그래도 이 항목은 아직 미충족**이다.
  ✅ **2026-08-11 19:41 현행 펌웨어 재측정으로 충족됐다** — bag `516.0 ms`(`1-b ≤600ms`) ·
  조건 2 = 정지 후 37,130ms `pose` 고정 · 영상 `관측 완전성 ✅`. **검토 §59 가 정식화와
  증거를 독립 재현·확인**했고(P0 0 · P1 0) `#11` 이 종결됐다(§7-c-0).
  🔴 **재개방 조건은 살아 있다** — 속도 상한 `0.12 m/s` 상향 · 펌웨어 재굽기나 안전 경로
  배선 변경 · 자율 발행자가 `/cmd_vel` 에 붙을 때는 이 판정을 다시 세운다.
- 🔴 **모터 명령 전에 네 바퀴가 모두 같은 방향으로 도는지 눈으로 본다** ★08-07 신설.
  `d0_check` 검사 6(전진 부호)은 **바퀴를 손으로 굴려 엔코더 부호**를 보는 것이라
  **모터 구동 방향을 검사하지 않는다** — D+0 검사 8개 어디에도 없던 구멍이다.
  2026-08-07 에 실제로 좌전륜 DIR 핀이 빠져 **혼자 역회전**했고 발행 도중에야 발견됐다.
  브레드보드라 **통전할 때마다** 본다 (`PITFALLS.md`).
- 안전요원 한 명이 E-stop을 잡고 있고, 평평한 바닥의 **양옆 1m 이상**을 비웠다.
- 펌웨어는 역 PWM 제동을 의도적으로 쓰지 않으므로 **경사·내리막에서는 시험하지 않는다**.

#### 7-c-0. ★ R0 watchdog — 명령 단절 0.5초 안에 실제 정지하는가

이 검사는 zero Twist를 보내서 멈추는 시험이 아니다. publisher가 끝난 뒤 **아무 명령도 없는
상태**에서 펌웨어 watchdog만으로 정지해야 한다. 바퀴를 공중에 띄우고 E-stop 담당자가 버튼을
잡는다. 60fps 이상 영상 한 화면에 **publisher 터미널의 마지막 `publishing #N` 표시와 바퀴**가
함께 보이게 한다. 별도 터미널에서는 교차 증거를 녹화한다.

```bash
ros2 bag record /cmd_vel /odom /estop/state -o d0_watchdog_$(date +%m%d_%H%M)
```

🔴 **이 절에는 무장 절차가 빠져 있었다 (2026-08-11 재측정에서 발견).** 위 본문은 re-arm
래치를 굽기 **전에** 쓰였다. 현행 펌웨어(`build=Aug 11 2026 15:13:20`)에서는 아래
`topic pub` 만 쏘면 **바퀴가 아예 안 돈다** — DISARMED 라 명령이 무시된다. 그러면
"watchdog 이 세웠다"가 아니라 "애초에 안 돌았다"가 되어 시행이 무효다. 그래서 주행
블록 앞에 `§7-c-E` 의 무장 4단계(`zero 0.5초 → /drive/enable true → 0.5초 장벽 → 주행`)
를 먼저 밟는다. 🔴 **무장에 쓴 zero 발행기는 주행 블록 전에 반드시 끈다** — zero 도
명령이라 계속 나가면 `lastCmdVelMs` 가 살아 있어 watchdog 이 영영 안 깨어나고, 재려는
대상 자체가 사라진다. ⚠ watchdog 정지는 무장을 풀지 않으므로 방향 확인 예행 뒤에도
`z=2` 는 유지된다(08-11 실측).

🔴 **재측정 촬영 규격 (2026-08-11 신설 — 이걸 어기면 영상이 통째로 무효다).**

- **한 화면에 publisher 터미널 글자 + 바퀴**. `T0` 는 *마지막 `publishing #30` 이 뜬 프레임*
  이고 **사람이 눈으로 읽는 값**이라, 글자가 안 보이면 도구가 시작점을 못 잡는다.
- **60fps 이상 · 슬로모션 아님**. 판정 창이 `200ms` 라 30fps 면 창이 6프레임밖에 안 된다.
- **바퀴 하나가 크게 · 림 무늬가 보이게.** 특징점이 안 잡히면 §58 관문에서 판정 불능이다.
- **촬영 중 줌·급격한 팬 금지.** 중심 추적이 배경 아핀이라 화면이 크게 튀면 체인이 끊긴다.
- 🔴 **폰을 고정한다 — 손각대는 판정선을 넘긴다** (2026-08-11 실측 신설). 손으로 들면
  배경 아핀 평행이동이 **프레임당 2~3px** 씩 튀고, 그 잔차가 잡음 바닥을 판정선
  (`5 mm/s` = 창당 `1.006°`) 위로 올려 도구가 **fail-closed 로 판정을 거부**한다.
  `IMG_3483.mov` 이 실제로 그랬다 — 전 구간(413~790)은 판정 불능이고 흔들림 전
  238프레임만 판정이 섰다. **삼각대·상자에 기대 놓기 어느 쪽이든 놓고 찍는다.**
- ⚠ **정지 뒤 최소 4초는 더 찍는다.** 조건 2 의 관찰 창이 `2.0초` 인데, 끝부분은
  촬영을 멈추려는 손 때문에 대개 못 쓴다. 여유가 없으면 그 시행의 조건 2 가 날아간다.
- ⚠ **`--preset 0807-1522` 을 새 영상에 쓰지 않는다.** 프리셋은 *그 영상의* 바퀴 타원
  좌표(`center`·`axes`·`frame_range`, 4K 원본 좌표계)다. 새 영상은 프레임을 뽑아
  **중심·반축·분석 구간·`T0` 를 새로 재고** `PRESETS` 에 항목을 추가한 뒤 판정한다.
  그 전까지는 `--center/--axes/--range/--t0-frame` 을 직접 준다.

아래 명령이 30건을 다 보내고 **자연 종료**하게 둔다. 종료 직후 zero Twist를 보내지 말고 2초
이상 그대로 관찰한다. 그 2초가 끝난 뒤에만 zero Twist 3회를 보내 시험을 안전하게 닫는다.

```bash
timeout --signal=INT --kill-after=2s 10s \
  ros2 topic pub --times 30 -r 10 -w 1 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
# 이 블록이 끝나도 아래 zero Twist 블록은 아직 실행하지 않는다.
```

첫 블록이 끝나면 **아무 명령도 보내지 않고 2초 이상 관찰한다.** 영상·pose 관찰을 마친 뒤에만
아래 블록을 따로 복사해 실행한다. 두 블록을 한꺼번에 붙여넣으면 이 시험은 무효다.
**바퀴가 0.5초를 넘겨 계속 돌면 2초를 채우지 말고 즉시 E-stop을 누른다.** 그 시점까지의
영상이 FAIL 증거이며, 아래 zero Twist 블록은 실행하지 않는다. R1 지면 명령을 금지하고
`FREEZE_MANIFEST.md §6` 조건부 수용을 재개방한다.

```bash
timeout --kill-after=2s 12s \
  ros2 topic pub --times 3 -w 1 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

**PASS는 세 조건을 모두 만족해야 한다.**

> 🔴 **아래는 구 기준이다** — 조건 1 은 펌웨어 구조상 달성 불가임이 밝혀져 **결정 1 = 안 ⓐ**
> 로 재정의됐다(2026-08-11 사용자 결정 · 🔴 **구현자 정식화는 검토 §57 에서 확인 보류**).
> 새 정식화 = 이 절 아래 "결정 1" 표. 확인이 끝날 때까지 **둘 다 만족되지 않은 것으로 읽는다.**
> ⚠ 조건 1 을 **영상만으로 PASS 판정하지 않는다** — 영상은 하한만 준다(같은 절).

1. 마지막 발행 표시 프레임부터 마지막 바퀴 회전 프레임까지 **30프레임 이하**다(60fps 기준
   0.5초, 더 높은 fps면 `fps×0.5` 프레임 이하).
2. 그 뒤 2초 동안 바퀴가 다시 돌지 않고, bag의 `/odom.pose` raw encoder 적분값도 더 변하지 않는다.
3. 영상 fps·센 프레임 수·계산한 초·bag 경로를 아래 D+1 인계표에 기록한다.

`/odom.twist`는 EMA(`α=0.10`)라 PWM이 이미 0이어도 늦게 감쇠한다. 따라서 **twist가 0이 되는
시각은 합격·불합격 기준이 아니고 보조 관측일 뿐**이다. 반대로 pose가 계속 변하거나 영상이
0.5초를 넘으면 실제 활주이므로 FAIL이다. FAIL·측정 불능이면 R1의 0.05m/s 지면 명령부터 금지하고
`FREEZE_MANIFEST.md §6` 조건부 수용을 재개방한다.

<!-- watchdog-evidence-slot:start -->
TODO(D+0): 확인 — 영상 fps·정지 프레임 수·환산 시간·bag 경로와 PASS/FAIL을
`D1_FIRST_STEP.md §0-a`에 기록한다. 빈칸이면 바로 아래 R1 대조군으로 진행하지 않는다.

**2026-08-07 실측 — 🔴 관측은 했고 이 항목은 닫히지 않았다.**
bag 3회(`~/Desktop/d0_evidence/d0_watchdog_0807_15{19,21,22}`, 노트북 보존). 마지막 비영
`/cmd_vel` → `/odom.pose` 마지막 이동 = 🔴 **519.9 / 532.0 / 516.2 ms**(3회 편차 16ms · 08-10 재산출).
⚠ **구판 값 `519.9 / 532.0 / 537.1` 은 폐기했다** — 검토 §52 가 그 도구의 P1 2건을 재현했다
(zero 개입 구간 미배제 · 증분 기반 정지 판정). 앞의 둘이 같은 것은 새 방법이 구판을 승인한 것이
**아니라**, 틀린 방법이 그 두 bag 에서 우연히 근처를 짚었던 것이다. 🔴 **판정선 민감도도 함께 본다** —
`2mm/s` 로 낮추면 1521·1522 가 10초대로 튄다(정지 11초 뒤의 기구 안착을 이동으로 세기 때문).
정본 판정선 = **5 mm/s / 200ms 창**(`tools/watchdog_report.py` `MOTION_RATE_MM_S`), 관측 주행속도
약 0.1 m/s 의 5% 다. **흔들리는 구간에서는 영상이 1차 증거다.**
정지 후 관찰 창 2.0초 이상에서 재기동 0 · `pose` 완전 고정 → **조건 2 는 충족**.
재계산 = `python3 tools/watchdog_report.py <bag>`.

- 🔴 **조건 1 미충족** — 세 값이 전부 0.5초를 넘는다.
- 🔴 **그런데 조건 1 은 이 펌웨어에서 달성이 불가능하다.** `WATCHDOG_TIMEOUT_MS = 500`
  (`.ino:184`)이 500ms 에 정지를 **시작**하므로(`.ino:669`), 마지막 회전은 반드시
  `500ms + 전송지연 + 기계관성` 이다. 관측 초과분 19.9~37.1ms 가 정확히 그 예산이다.
  🔴 **기준 재정의는 사용자 결정 + 검토자 확인 사항이며 구현자가 고치지 않는다** —
  시험에 떨어진 쪽이 기준을 바꾸는 모양이 되기 때문이다.
**2026-08-11 — 1차 증거(60fps 영상) 분석 완료. 🔴 그래도 조건 1 은 안 닫혔다.**

영상 = `~/Desktop/d0_evidence/video/IMG_3461.mov` (iPhone 14 Pro · 3840×2160 HEVC ·
**실측 `59.9955 fps`**(`r_frame_rate=60/1`, 슬로모션 아님) · 1327 프레임 / 22.118초).
🔴 **촬영 시각 `2026-08-07 15:22:58 KST` 라 bag `d0_watchdog_0807_1522` 와 같은 시행**이고,
화면 로그에도 그 bag 경로가 찍혀 있다. 재계산 = `python3 tools/watchdog_video.py <영상>
--t0-frame 670 --preset 0807-1522 --bag-ms 516.2` (그래프 = 같은 디렉터리 `wd_result.png`).

| 항목 | 값 |
|---|---|
| T0 = 마지막 `publishing #30` 화면 표시 | **n=670** (t=11.1675s) · 669=`#29`→670=`#30` 한 프레임 전환 |
| T1 = 마지막 바퀴 회전 | **n=698** · 정본 판정선 `5 mm/s` / 창 `200ms` |
| 간격 | **28 프레임 = 466.7 ms** |
| 판정선 민감도 | `2mm/s→220f`(불안정) · **`5·10·20 mm/s→28f`**(일치) |

🔴 **이 값을 "조건 1 PASS" 로 읽지 않는다 — 영상은 원리적으로 PASS 를 못 만든다.**
T0 는 *글자가 화면에 뜬 시각*이지 *메시지가 나간 시각*이 아니다. 터미널 렌더는 항상 늦고,
T1 은 판정선 `5 mm/s` 를 밑도는 순간 잡히므로 참값보다 이르다. **양쪽 모두 구간을 짧게
만들어 측정값은 참값의 하한**이 된다 — 판정선 아래로 보이는 것은 편향의 결과일 수 있다.
이는 `§7-c-E` 의 `464.0ms` 를 기각한 것과 **같은 종류의 편향**이다. 총 정지 시간의 정본
측정은 **bag** 이다(사람 눈과 화면 렌더가 끼지 않는다).

🔴 **같은 시행 bag `516.2ms` 와의 차이 `49.5ms` 는 "렌더 지연"이 아니라 `관측계 차이` 다**
(독립 검토 §57.2 P1 — 초판이 이것을 렌더 지연으로 특정했다). bag 의 시작점은 `Twist` 가
*발행된* 시각이 아니라 rosbag 이 **저장한** 시각이고, 끝점도 `/odom` 이 저장된 시각이다.
영상은 *화면 표시* → *회전 검출* 이다. 그래서 `49.5ms` 안에는 **렌더 지연 · 두 토픽의
DDS/record 저장 지연 · odom 발행 주기 · 두 T1 검출기의 차이**가 함께 들어 있고, 별도
동기화나 검증된 상한 없이 **어느 한 성분으로 특정하지 않는다.** 도구도 이 차이에
"타당/부당" 판정을 붙이지 않는다.

✅ **영상이 실제로 닫은 것 — 조건 2 에 펌웨어 독립 증거가 생겼다.** 지금까지 조건 2 는
`/odom.pose`(펌웨어가 자기 엔코더를 적분한 값)에만 기대고 있었다. 영상 실측 =
**정지 후 3267ms 누적 회전 `+1.963°` = `0.595 mm/s`** (판정선 `5 mm/s` 의 1/8.4).
잔차를 접선(회전)·반경(잡음)으로 분해하면 접선/반경 비가 주행 `1.6` → 정지 `0.01` 이라,
**남은 흔들림이 회전이 아님**이 같이 증명된다.

🔴 **단, 이 `0.5945 mm/s` 가 유효한 것은 285프레임에 결측이 0건이기 때문이다**(검토 §58.1).
판정기의 바퀴 중심은 프레임마다 배경 아핀으로 **누적**되므로, 추적이 한 번 끊기면 그 뒤
전부가 어긋난 자리에서 측정된다 — 검토자가 이 영상의 T0 **앞** 20프레임만 눈멀게 하자
같은 조건 2 가 `0.1487 mm/s`(4분의 1)로 떨어졌다. **덜 미끄러진 것처럼 보이는** 방향이라
위험하다. 그래서 판정기는 이제 **요청 구간 전체**(T0 앞 포함)가 연속·유한·유효점 `≥30`
일 때만 수치를 내고, 그 사실을 `관측 완전성:` 줄로 **따로 찍는다**. 이 영상은 재실행에서도
`610~894 285프레임 … ✅` 를 그대로 냈고 위 수치는 변하지 않았다.

**정지 예산이 좁혀졌다 — 다만 "정체가 특정됐다"고는 말하지 않는다.** 영상이 실제로 준 것은
**기계 관성의 상한**이다: 바퀴가 전속(`1.49°/f`)에서 정지까지 가는 데 **1~2 프레임
(17~33ms)** 뿐이라 관성의 몫이 작다.

    516.2 ms(bag 실측) = 500 ms(`WATCHDOG_TIMEOUT_MS`, `.ino:184`) + ~16 ms(잔여 예산)

🔴 잔여 `~16ms` 는 **전송 + loop + 관성 + bag 저장 지연이 섞인 덩어리**이고, 위 관성 상한을
빼고 나면 나머지를 성분별로 가르지 못한다(§57.2). 말할 수 있는 것은 딱 둘이다 —
**상수를 안 건드리면 500ms 미만은 원리적으로 불가능**하고, 거꾸로 **상수를 낮추면 예산상
반드시 통과**한다. 이것이 아래 결정의 입력이 됐다.

**2026-08-11 19:41 — 🔴 현행 펌웨어 재측정 완료. `#11` 의 남은 항목은 검토자 재확인 하나다.**

승계 불인정(§57.5)에 따라 `build=Aug 11 2026 15:13:20` 으로 **새로 찍었다.** 바퀴 공중 ·
영상 + bag 동시 · 굽기 없음. 무장은 `§7-c-E` 4단계(`z=0→1→(3)→2`)로 했고 주행 전
zero 발행기를 껐다.

| 항목 | 값 | 관측계 |
|---|---|---|
| bag | `~/robot_evidence/d0_watchdog_0811_1938` (노트북 · 젯슨 원본 보존) | **정본** |
| 마지막 비영 `/cmd_vel` → 마지막 `pose` 이동 | **516.0 ms** (계약 500ms 대비 **+16.0 ms**) | bag |
| 판정선 민감도 | `2mm/s→536.7` · **`5→516.0`** · `10→495.0` · `20→473.8` ms — 08-07 처럼 10초대로 튀지 않는다 | bag |
| 조건 2 | 정지 후 **37,130 ms** 관찰 · `pose` 가 `0.2620 / 0.3978` 로 **완전 고정** · 재기동 0 | bag |
| 영상 | `~/Desktop/d0_evidence/video/IMG_3483.mov` · 3840×2160 HEVC · **59.9925 fps** · 804프레임/13.4초 | 교차 |
| `T0` → `T1` | n**473**(`publishing #30` 표시) → n**500** = **27 프레임 = 450.1 ms** | 교차 |
| 관측 완전성 | **`413~650 238프레임 전량 연속·유한·유효점≥30 ✅`** | 교차 |
| 조건 2 (펌웨어 독립) | 정지 후 2500ms 누적 `-1.270°` = **0.5018 mm/s** (판정선의 1/10) | 교차 |
| 교차 차이 | bag `516.0` − 영상 `450.1` = **+65.9 ms** | — |

재계산 = `python3 tools/watchdog_video.py <영상> --t0-frame 473 --preset 0811-1938
--bag-ms 516.0` · `python3 tools/watchdog_report.py <bag>`.

- ✅ **1-b(`≤600ms`) 실측 충족** — `516.0 ms`. 08-07 의 `516.2 ms` 와 **0.2ms** 차이라,
  안전 경로 배선이 바뀌어도 총 정지가 같은 자리에 있음을 **이 시행이 직접** 보였다.
  🔴 그래도 이것은 **승계가 아니라 재측정**이고, 08-07 값은 여전히 인용하지 않는다.
- 🔴 **여전히 영상으로 조건 1 을 PASS 시키지 않는다** — `450.1 ms` 는 하한이다.
- 🔴 **영상 분석 구간을 `413~650` 으로 끊었다. 이 사실을 판정과 함께 읽는다.**
  `651` 이후는 손각대 흔들림(배경 평행이동 프레임당 2~3px) 때문에 **끝 범위를 어디로
  잡아도 fail-closed 로 끝난다** — 사유가 둘이라 범위마다 다르게 나오므로 명령과 같이
  적는다(검토 §59.3): `--range 413,790` → **꼬리 부족**(겉보기 회전이 흔들림에 늦게
  잡혀 정지 뒤 관찰이 `1550ms` < 2000ms) · `--range 413,680` → **잡음 최대 `1.089°`** ·
  `--range 413,741` → **`1.574°`** (둘 다 판정선 `1.006°` 초과 = 흔들림을 끝 잡음창에
  넣은 경우). 정본 `--preset 0811-1938`(=`413~651`)만 `0.924° < 1.006°` 로 판정이 선다. 끊은 구간이
  조건 2 요구치(2.0초)보다 긴 **2.5초**를 담는다. `690~700` 의 신호가 바퀴 회전이
  **아니라는** 근거 셋 — ① 배경 아핀 평행이동 급변과 프레임 단위로 일치 ② 접선/반경
  비 `0.45`(실주행은 `4.0`) ③ **같은 구간 bag `pose` 가 완전 고정**(2° 만 돌아도 림에서
  2mm 라 엔코더가 본다). ⚠ **다음 촬영은 폰을 고정해서 이 단서를 없앤다.**
- ✅ **검토 §59 가 이 증거를 독립 재현하고 `#11` 종결 · R1 진입 가능으로 판정했다**
  (2026-08-11 · P0 0 · P1 0 · P2 2). ⚠ **승인의 형태를 정확히 읽는다** — `516.0ms` 단독이
  아니라 `1-a` 구조 · 조건 2 · 조건 3 · **사용자 위험 수용 문장** · 기존 R1 운용 전제를
  **함께** 쓰는 승인이다. 🔴 **재개방 조건은 그대로다**(속도 상한 상향 · 재굽기나 안전
  경로 배선 변경 · 자율 발행자 연결).

#### 🔴 결정 1 — `§7-c-0` 합격 기준 재정의 = **안 ⓐ** (2026-08-11 사용자 결정)

**사용자 결정 = ⓐ 기준 재정의** (ⓑ 구조 개선 / ⓒ 보류를 물리치고). 색인 = `MASTER_PLAN §8`.
✅ **닫혔다 — 검토 §59 가 아래 정식화를 확인했다** (2026-08-11 · P0 0 · P1 0 · P2 2).
계약이 *"사용자 결정 **+** 검토자 확인"* 이었고 둘 다 채워졌다. §57 의 확인 보류 사유
(P1 2건 + 비어 있는 사용자 근거)는 `(c)` 문장과 08-11 재측정으로 해소됐다.
⚠ **승인의 형태를 정확히 읽는다** — `516.0ms` 라는 숫자 하나가 아니라 `1-a` 구조 ·
조건 2 · 조건 3 · **사용자 위험 수용 문장** · 기존 R1 운용 전제를 **함께** 쓰는 승인이다.

아래가 검토자가 확인한 정식화다:

| # | 조건 | 판정 방법 |
|---|---|---|
| **1-a** | `WATCHDOG_TIMEOUT_MS ≤ 500` | **구조** — 소스 상수. `firmware_precheck.sh` 지문이 보증 |
| **1-b** | 마지막 비영 `/cmd_vel` → 바퀴 완전 정지 **≤ 600 ms** | **실측** — 정본 = bag(`watchdog_report.py`), 교차 = 영상(`watchdog_video.py`) |
| 2 | 정지 후 2초 재기동 0 · pose 고정 | 기존과 같음 |
| 3 | fps·프레임·초·bag 경로 기록 | `D1_FIRST_STEP §0-a` |

🔴 **`600ms` 는 최악 지연의 증명이 아니라 사용자가 수용할 운용 상한이다** (독립 검토 §57.2).
초판은 `500`(상수) + `100`(전송 + loop + 관성) 을 *계측 근거*처럼 적었는데, 위에서 보듯
잔여 예산을 성분별로 못 가르므로 그 논거는 성립하지 않는다. 이 값을 유지하려면 근거는
**셋뿐**이다 — ① bag 3회 관측 범위 `519.9 / 532.0 / 516.2 ms` ② 적용 속도에서 허용
정지거리(`0.05 m/s` 에서 **0.81mm**, 상한 `0.12 m/s` 에서 **1.94mm**) ③ **현행 펌웨어
재측정 결과**. ✅ **③ = `516.0 ms`**(08-11 19:41 · 위 표 · 검토 §59 독립 재현).

✅ **`(c) 왜 그 값인가` — 사용자 결정 (2026-08-11).** 🔴 이 문장은 **사용자 소유**이고
구현자가 고쳐 쓰지 않는다:

> **"적용 속도 상한 `0.12 m/s` 에서 명령이 끊긴 뒤 약 7 cm 더 나아가는 것까지는
> 받아들인다. 안전요원이 E-stop 을 들고 양옆 1 m 를 비운 상태라 그 거리가 물리적
> 위험을 만들지 않는다."**

산출 = 명령 단절 뒤 `WATCHDOG_TIMEOUT_MS = 500ms` 동안은 **원래 속도 그대로 간다**
(watchdog 이 아직 안 깨어난다). 그 뒤 정지까지는 1~2 프레임뿐이므로 미끄러지는 거리
`≈ v × 0.6 s` 다 — `0.05 m/s` 에서 **약 30 mm**, 상한 `0.12 m/s` 에서 **약 72 mm**.
🔴 **이것은 최악 지연의 증명이 아니라 "그 거리를 수용한다"는 판단**이다(형식 = 검토 §57.2).
🔴 **재개방 조건** — 속도 상한을 `0.12 m/s` 위로 올리면 이 문장의 전제가 깨지므로
`600ms` 를 다시 판정한다(`FREEZE_MANIFEST §6` 재개방 ⓸ 와 같은 자리).

> 🔴🔴 **2026-08-11 21:46 — 이 문장의 전제가 실측으로 깨졌다. 재개방된 상태다.**
> 위 문장은 **사용자 소유**이므로 구현자가 고쳐 쓰지 않고, 그대로 두되 **효력이 정지된
> 것으로 읽는다.** 사유: 명령 `0.12` 의 실제 속도가 **`0.3265 m/s`**(예약 32)라서
> 정지거리가 `0.33 × 0.6s ≈` **약 20cm** 다 — 문장이 수용한 **7cm 의 약 3배**다.
> 🔴 **상한을 올려서 깨진 것이 아니라, 상한이 물리 속도를 묶은 적이 없다는 것이 관측된
> 것**이다. `MAX_LINEAR_CMD` 는 명령 클램프다.
>
> ⚠ **같은 날 21:30 경 갱신안(`0.13 m/s` / 약 8cm)이 사용자 승인을 받았으나, 30분 뒤
> `§7-c-1` 3m 실측이 그 근거(`0.13` 이 물리 하한이라는 관측 1회)를 뒤집어 **채택되지
> 않았다.** 그 갱신안을 유효한 수용으로 인용하지 않는다.
>
> **다시 쓰는 조건** = 예약 32 교정 후 **실측 정지거리**를 얻고, 그 거리로 사용자가 문장을
> 새로 쓴다. 그때까지 **R1·R2 지면 주행 금지**.

✅ **`#11` 종결 — §57.6 이 그은 경계 셋이 모두 닫혔다** (2026-08-11).
① ✅ **사용자 근거 한 줄 — 채워졌다**(바로 위 `(c)`, 2026-08-11 사용자 결정).
② ✅ **현행 펌웨어 재측정 — 2026-08-11 19:41 완료** (`d0_watchdog_0811_1938` +
`IMG_3483.mov` · `516.0 ms` · `관측 완전성 ✅` · 전문 = 바로 위 표). 아래는 그 시행이
따라야 했던 전제이며, 다음 재측정에도 그대로 적용된다 — 🔴 **재측정에는 전제가 하나 붙었다**
(검토 §58 조건부 수용): 영상 도구가 찍는 **`관측 완전성:` 줄이 `✅` 여야** 그 시행의 조건 2
값을 쓸 수 있다. 요청 구간 **전체**(T0 앞 포함)가 연속·유한·유효점 `≥30` 이라는 뜻이다.
`⚠` 이거나 판정 불능이면 **그 시행은 R1 허가 근거로 쓰지 않고 다시 찍는다** — 바퀴가
공중이라 재촬영은 싸다. 이유는 §58.1: 바퀴 중심이 프레임마다 배경 아핀으로 **누적되는
상태**라, T0 앞에서 추적이 한 번 끊기면 어긋난 중심이 T0 이후 전 구간을 오염시키고
조건 2 가 **실제보다 작게**(안전 반대 방향) 나온다. 오늘 분석한 영상은 **굽기 전** 물건이고
R1 이 달릴 물건은 `build=Aug 11 2026 15:13:20` 이다. 🔴 **승계는 불인정됐다**: 08-07 이후
`f57d454..a7d1483` 에서 `cmdVelCallback` · `updateMotorOutputs` · `checkSafety` ·
`lastCmdVelMs` 배선이 바뀌었으므로, watchdog 상수 `500ms` 가 그대로여도 새 바이너리의
**물리 총정지**를 08-07 증거가 대신하지 못한다.
③ ✅ **검토자 재확인 — 검토 §59 완료**(2026-08-11 · P0 0 · P1 0 · P2 2). `1-a` 구조 ·
`1-b 516.0ms ≤ 600ms` · 조건 2 · 조건 3 이 모두 관찰됐고 정식화가 확인됐다.
**R1 지면 주행 금지는 해제된다.** 🔴 다만 `§7-a` 의 나머지 주행 허용 전제는 그대로다 —
안전요원의 E-stop · 양옆 1m · 경사 금지 · **네 바퀴 방향 육안 확인**.
- ⚠ 무부하 공중이라 `/odom.twist.x` 가 명령 `0.05` 의 약 2배(0.10)로 관측됐다. PI 가
  `MIN_RUNNING_PWM` 바닥에 걸린 것으로 보이나 오도메트리 스케일 오차와 구분되지 않는다 —
  **구분은 R2(3m ±3%) 몫**이다. 🔴 이 값이 사실이면 `§5-G8` 의 "`0.05 m/s`" 라벨도
  공중 시험분에 한해 실제보다 낮게 적힌 것이 된다.
<!-- watchdog-evidence-slot:end -->

#### 7-c-E. 🔴 re-arm 래치 부정·전환 시험 — **R1 진입 전 필수** (2026-08-11 신설)

계약 = `REAL_ROBOT_VALUES.md §1-f`. 이 절을 통과하기 전에는 `§7-a` 의 "주행 허용 전제"
셋째 줄이 미충족이므로 **R1·R2 지면 주행을 금지**한다.

⚠ **전량 바퀴를 띄운 채로 한다.** 통과한 뒤에 내린다.

🔴 **08-11 개정 — 상태가 5개가 됐고 무장에 단계가 하나 늘었다** (검토 §54·§55).
**전이 자체는 이 절에서 판정하지 않는다** — 그건 PC 에서 `bash tools/rearm_gate_host_test.sh`
가 결정론적으로 끝냈다(동작 922건 + 구조 7건). **이 절이 보는 것은 오직 배선이다**: 상태에 맞게
*모터가 실제로 서는가*, *토픽이 실제로 나오는가*. 실기로 500ms 경계를 재려 들지 않는다.

**준비** — 터미널 3개
```bash
# T1  상태 관측 (이 창을 보면서 진행한다)
ros2 topic echo /drive/enabled          # std_msgs/Bool — true 면 ARMED
# T2  진단 — geometry_msgs/Vector3
#     x = 서비스 호출수(거절 포함) · y = 거절사유 · z = 상태
#     🔴 z: 0 DISARMED / 1 READY / 2 ARMED / 3 PENDING / 4 ARMING (08-11 신설)
ros2 topic echo /drive/diag
# T3  명령·서비스
```

⚠ **`z=4`(ARMING) 는 눈에 안 띄는 것이 정상이다** — 서비스 콜백 반환과 같은 `loop()` 안에서
`z=3` 이 되므로 그 사이에 발행이 없다. 🔴 **반대로 `z=4` 가 계속 붙어 있으면** `loop()` 의
`rearmGateArmBarrierStart()` 가 안 불리는 것이고, 그 보드는 **영원히 무장되지 않는다**
(fail-closed 라 위험하진 않다). 그때는 굽힌 펌웨어가 이 묶음이 맞는지부터 본다.

🔴 **08-11 절차 정정 ① — 눈으로 재면 안 되는 행이 셋 있다. 도구를 쓴다.**
`/drive/diag`·`/drive/enabled` 는 **둘 다 `publishDiagnostics()` 안에서 1초에 한 번**만 나간다
(`DIAGNOSTIC_PERIOD_MS = 1000`). 그런데 `PENDING` 은 **500ms** 만 유지된다 —
**샘플링 주기가 관측 대상보다 길다.** 그래서 아래 세 행은 사람 손·눈으로 판정할 수 없다:

| 행 | 왜 못 하나 | 쓰는 도구 |
|---|---|---|
| **전환 2** | *"`z=2` 가 **바로** 떴다"* 와 *"0.5초 뒤에 떴다"* 를 1Hz 로는 **원리적으로 못 가린다** | — (보조 관측으로 내린다) |
| **부정 6** | `ros2 topic pub` 은 노드 생성·DDS 탐색에 **0.5~1.5초**가 걸려 명령이 **장벽이 끝난 뒤** 도착한다 → 바퀴가 도는 것을 "장벽 미배선"으로 오판(**거짓 FAIL**) | `tools/rearm_neg6_field.py` |
| **부정 7** | 해제 뒤 발행을 끊으면 **watchdog 이 대신 세워** "해제가 세운 것"과 구분 불가 | `tools/rearm_field_disarm.py` |

🔴 **판정 무게를 옮긴다 — 장벽 배선의 정본 증거는 전환 2 가 아니라 부정 6 이다.**
부정 6 은 행동 시험이라 샘플링 주기와 무관하다. 도구는 **주입 지연을 함께 인쇄**하므로
"명령이 늦게 도착해서 통과한 것 아니냐"는 반론까지 같이 닫는다(08-11 실측 **0.3ms**).
⚠ 전환 2 에서 `z=3` 이 보이면 좋지만 **안 보여도 실패가 아니다** — 관측 한계이지 결함이 아니다.
반복 시도로 잡히는 것은 절차가 아니라 **확률**이다(08-11 에 실제로 그렇게 잡혔다).

🔴 **부정 7 도구의 설계를 적어 둔다 — 해제 뒤에도 비영을 계속 발행한다.** 그래야 watchdog 이
발동하지 않고, 바퀴가 서는 이유가 `driveDisarm()` 정지 + `driveOutputAllowed()` 0 덮어쓰기
(§54.1 두 겹)로 **유일하게 좁혀진다.** DISARMED 에서 비영은 어차피 무시되므로 안전하다.

⚠ **정지 판정선은 `tools/watchdog_report.py` 정본과 같은 5 mm/s · 200ms 창**을 쓴다.
새 기준을 만들면 R0 기록과 비교가 깨진다.

🔴 **먼저 볼 것 — 새 펌웨어가 실제로 올라갔는가.** 두 단계다:
```bash
ros2 topic type /drive/enabled    # → std_msgs/msg/Bool           없으면 구판이다. 멈춘다
ros2 topic type /drive/diag       # → geometry_msgs/msg/Vector3
```

🔴 **무장 절차가 4단계다** (`REAL_ROBOT_VALUES §1-f` ⓵):
```
① zero 를 0.5초 이상 발행    → z=1 (READY)
② /drive/enable true 호출     → success:true, 🔴 z=3 (PENDING) — 아직 무장 아니다
     (내부적으로 z=4 ARMING 을 거치지만 응답 전송과 같은 loop 이라 안 보인다)
③ 0.5초 더 기다린다           → z=2 (ARMED), /drive/enabled=true
     (그동안 zero 를 계속 줘도 되고 아예 안 줘도 된다. 비영을 주면 처음으로 돌아간다)
④ 주행 명령
```
⚠ **②에서 `z=2` 가 바로 뜨면 그건 구판이다** — 장벽이 없는 펌웨어이므로 멈추고 §5 를 본다.
⚠ **③의 0.5초는 응답을 받은 시각부터 센다** (검토 §55.1). 08-11 이전 판은 서비스 콜백 진입
시각부터 재서 실제 장벽이 그만큼 짧았다. 실기에서 그 차이는 눈으로 못 가르므로 **여기서
판정하지 않는다** — PC harness T24 가 δ 를 주입해 닫았다.

| # | 넣는 것 (T3) | 기대 | 실패 시 뜻 |
|---|---|---|---|
| **부정 1** 🔴 | E-stop 누름 → `0.05` 연속 발행을 **켜 둔 채** E-stop 해제 | 바퀴 **안 돎**. `z` 가 0 에서 안 올라감 | 🔴 **이 래치의 존재 이유가 무너졌다. 즉시 중단** |
| **부정 2** | 위 상태에서 `ros2 service call /drive/enable std_srvs/srv/SetBool "{data: true}"` | `success: false` · `y=2` | `x` 가 안 오르면 **서비스가 안 온 것**(로직 문제 아님) |
| **부정 3** | E-stop 누른 채 같은 서비스 호출 | `success: false` · `y=1` · `z=0` | — |
| **전환 1** | 발행 중지 → zero 를 0.5초 이상 | `z=1` (READY) | zero 가 안 들어가고 있다 |
| **전환 2** 🔴 | READY 에서 서비스 호출 | `success: true` · **`z=3`(PENDING)** · `/drive/enabled` = **아직 false** | `z=2` 면 **장벽이 없는 구판**이다 |
| **전환 3** 🔴 | 아무것도 안 하고 0.5초 기다림 | `z=2` · `/drive/enabled` = **true** | 장벽이 안 끝난다 |
| **부정 6** 🔴 | 다시 처음부터: READY → 서비스 → **곧바로(0.5초 안에) `0.05` 발행** | 바퀴 **안 돎** · `z=0` 으로 떨어짐 · 그 뒤 기다려도 ARMED 안 됨 | 🔴 **§54.2 장벽이 배선되지 않았다** |
| **역회귀 1** | 정상 4단계로 ARMED 만든 뒤 `0.05` 발행 | 주행 | — |
| **역회귀 2** 🔴 | ARMED 주행 중 **발행을 끊는다** | **0.5초 안에 정지**(watchdog 이 산다) | 🔴 **re-arm 이 watchdog 을 가렸다 — R0 가 다시 깨진다** |
| **부정 4** | 다시 ARMED 로 만든 뒤 주행 중 **E-stop 누름** | 즉시 정지 · `z=0` · `/drive/enabled` = false | — |
| **부정 5** | ARMED 상태에서 서비스 재호출 | `success: false` · `y=3` | 멱등이 아니라 거절이다(설계 그대로) |
| **부정 7** 🔴 | **ARMED 로 `0.05` 주행 중** `"{data: false}"` 호출 (§54.1) | 🔴 **응답이 오는 순간 이미 바퀴가 서 있다** · `z=0` | 🔴 **응답 뒤에도 도는 시간이 있으면 §54.1 이 안 고쳐졌다** |
| **부정 8** 🔴 | ARMED 주행 중 `linear.x: .nan` 발행 (§54.3) | 즉시 정지 · `z=0` · **재무장 필요** | 결정 ⓐ 대로다. `z` 가 2 로 남으면 안 고쳐졌다 |
| **해제** | 어느 상태에서든 `"{data: false}"` | `success: true` · `z=0` | 명시적 해제는 언제나 허용 |

> **부정 8 의 NaN 발행법** — YAML 은 `.nan` 이다:
> ```bash
> ros2 topic pub --times 3 -w 1 /cmd_vel geometry_msgs/msg/Twist \
>   '{linear: {x: .nan, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
> ```

⚠ **`x`(서비스 호출수)를 매번 본다.** 굽기를 1회로 합친 대가를 이 숫자가 갚는다 —
호출했는데 `x` 가 그대로면 **로직이 아니라 서비스 전달**이 문제다(`§5` 버전 정합).

#### 7-c-E 결과 — 🔴 **2026-08-11 13행 전항목 통과** (펌웨어 `build=Aug 11 2026 15:13:20`)

| 행 | 결과 | 관측값 |
|---|---|---|
| 부정 1 | ✅ | E-stop 해제 시 잔류 `0.05` 로 **안 돎** · `z=0` 유지 |
| 부정 2 | ✅ | `success:false` · `y=2` · **`x` 증가**(서비스 도달 확인) |
| 부정 3 | ✅ | `success:false` · `y=1` · `z=0` |
| 전환 1 | ✅ | zero 지속 → `z=1` |
| 전환 2 | ✅ | `success:true` · **`z=3` 실제 관측**(⚠ 확률로 잡힌 것 — 위 정정 ①) |
| 전환 3 | ✅ | `z=2` · `/drive/enabled=true` |
| **부정 6** | 🔴✅ | **주입지연 0.3ms** · `z` 1→**0** · `enabled=true` **0건** · 바퀴 안 돎(육안) |
| 역회귀 1 | ✅ | ARMED 에서 `0.05` 주행, 바퀴 돎(육안) |
| 역회귀 2 | ✅ | 발행 끊자 정지(육안) · 마지막발행→마지막이동 **464.0ms** |
| 부정 4 | ✅ | 주행 중 E-stop → 즉시 정지 · `z=0` · `enabled=false` |
| 부정 5 | ✅ | `success:false` · `y=3` · `z=2` 유지 |
| **부정 7** | ✅ | 응답 이후 **이동 0** (해제 뒤에도 `0.05` 계속 발행 = watchdog 배제) · `z=0` |
| **부정 8** | ✅ | NaN 이후 **이동 0** · `z=0` · 뒤이은 `0.05` 2초에도 안 돎(**재무장 필요 확인**) |
| 해제 | ✅ | 매 회차 `success:true` · `z=0` |

🔴 **464.0ms 를 "watchdog 이 500ms 계약을 만족한다"로 읽지 않는다.** 이 계측은 낮은 쪽으로
편향돼 있다 — 시작점이 *발행 루프 종료 시각*(실제 마지막 메시지보다 최대 ~50ms 늦다)이고
끝점도 `/odom` **수신** 시각이다. 08-07 R0 값(519.9/532.0/516.2ms)은 **bag 타임스탬프라 자가
다르다.** → **08-07 범위와 모순되지 않는 값**일 뿐이고 `#11` 은 그대로 열려 있다.
이 행이 판정한 것은 하나다 — **re-arm 이 watchdog 을 가리지 않았다.**

🔴 **08-11 신설 관측 — watchdog 정지는 무장을 풀지 않는다.** 역회귀 2 구간에서 `z=2`(ARMED)가
watchdog 정지를 사이에 두고 **끊기지 않고 유지**됐다. 설계상 틀린 것은 아니다(래치가 막는 것은
*E-stop 해제 순간의 잔류 명령*이지 통신 끊김이 아니다). 🔴 **다만 함의는 분명하다 — 발행자가
멈췄다 살아나면 재무장 없이 바퀴가 다시 돈다.** 수동 텔레옵인 R1 에서는 조작자가 그 자리에
있으므로 절차로 덮이지만, **자율 발행자가 `/cmd_vel` 에 붙는 시점에 재평가한다** — 그 시점은
`REAL_ROBOT_VALUES §1-f` §56.1 재개방 1순위와 **같은 자리**다.

⚠ `/drive/enabled` 는 `z` 의 파생값(`data = (state == ARMED)`)이고 같은 1Hz 스냅샷에서 나간다.
**진단 정보량은 `z` 가 더 크다**(5상태 vs 2상태). 둘이 어긋나면 그 자체가 펌웨어 버그다.

🔴 **통과 뒤 곧바로 `§5-G6` E-stop 개폐 10회를 다시 한다** — 업로드로 `/estop/state`
신뢰가 회수됐다(`ELECTRICAL_BASELINE §7` 표). 약 30분.
✅ **2026-08-11 재수행 `10/10` 통과** — `tools/estop_toggle_check.py` 로 전이를 자동 계수했다
(10회 중 9회는 통과가 아니다 — `§5-G6` 규칙). 이어 **G7 `d0_check.sh` 통과**(검사 8개).
⚠ 그 스크립트가 찍는 초는 **스위치 반응 속도가 아니다** — `/estop/state` 도 1Hz 발행이고
누르는 시점도 사람 자유다. 판정하는 것은 **전이의 유무**다.

#### 7-c-R1. R1 0.05m/s 대조군

🔴 **2026-08-11 20:47 1차 시행 = 실패. 좌전륜 엔코더 부호 반전을 찾았다** (`r1_ground_0811_2047`).

육안으로는 **직진 완벽 · 진동/소음 없음 · 릴레이 발열 없음**이었는데 `/odom` 이 5초 내내
일정한 `+0.225 rad/s` 를 냈다 — 반지름 31cm 원호, 5초에 `69.5°`. 둘 다 참일 수 없어서
바퀴별로 갈랐다(**바퀴 띄우고 무장 해제 · 한 번에 한 바퀴씩 굴리며 3초 정지** ·
`enc_check_0811_2102`):

| 굴린 바퀴 | `Δyaw` | 계산되는 쪽 | |
|---|---|---|---|
| 좌전륜 | **+70.65°** | **우측** | 🔴 **부호 반전** |
| 좌후륜 | −57.70° | 좌측 | ✅ |
| 우전륜 | +59.33° | 우측 | ✅ |
| 우후륜 | +49.10° | 우측 | ✅ |

`deltaLeft = 0.5×(dFL + dRL)` 에서 **FL 이 RL 을 지워 `deltaLeft = 0`** 이 되므로
거리는 절반, 회전은 없던 것이 생긴다. 관측 `0.225` 와 모델 `0.226` 이 셋째 자리까지 맞는다.

🔴 **오도메트리만의 문제가 아니다** — PI 가 `direction × filteredWheelVelocity` 로 오차를
만들어서 부호가 반대면 적분기가 감기고 **FL 만 PWM 천장**으로 간다. 그래서 실제 속도가
명령을 넘는다: odom 이 실제의 절반을 세므로 **실제 이동 ≈ 750mm / 4.9s ≈ `0.15 m/s`**
(명령 `0.05` 의 3배 · 🔴 **안전 상한 `0.12` 초과**). ⚠ **미확정** — 줄자 눈대중은 `289mm`
였고(시작점 미표시) odom 모델과 안 맞는다. **고친 뒤 표시하고 다시 잰다.**

✅ **2026-08-11 21:27 2차 시행 = 부호 수정 확인.** 좌전륜 엔코더 **A/B 배선 교환**(펌웨어
무변경) 뒤 다시 쟀다(`enc_check_0811_2120` → 좌 2개 음수 · 우 2개 양수 · `r1_ground_0811_2127`).

| 항목 | 값 | |
|---|---|---|
| 횡편차 | **12.4mm / 676mm = 1.8%** | ✅ (1차 201mm) |
| yaw — 엔코더 | **2.00°** | ✅ (1차 69.5°) |
| yaw — **IMU(독립 관측자)** | **1.70°** | ✅ 두 관측자 **0.3° 이내 일치** |
| 거리 — odom vs **줄자** | **676 vs 685 mm** | ✅ **오도메트리 스케일 정상**(오차 1.5%) |

🔴 **줄자가 08-07부터 열려 있던 질문을 닫았다.** `PITFALLS §11` 이 *"명령보다 빠른 것과
오도메트리 스케일 2배 오차는 이 데이터로 구분되지 않는다 — 가르는 것은 R2"* 라고 미뤄 뒀는데,
**odom 거리가 줄자와 맞으므로 스케일이 아니라 실제로 빠른 것**이다. → 예약 32.

**고치는 자리는 배선이다** — 🔴 `ENCODER_POLARITY[FL]` 를 고쳐 다시 구우면 `§7-c-E` 13행과
`§5-G6` 10회를 **다시** 해야 한다. **좌전륜 엔코더 A/B 두 가닥을 서로 바꿔 꽂는 것**이
같은 효과에 펌웨어 무변경이다. 순서 = `Jetson shutdown → XT90 분리 + 퓨즈 제거 → A/B 교환
→ 기동 순서 → E-stop 개폐 10회 재검증(배선을 만졌으므로 신뢰 회수) → 위 4구간 엔코더 시험
재실행(①② 음수 · ③④ 양수) → R1 재시행(시작·끝 표시 + 줄자)`.

⚠ **`#11` 은 영향받지 않는다** — R0 는 *언제* 멈췄는지만 쟀고 거리·방향을 안 썼다. FL 이
전속이었다면 관성이 더 컸으므로 `516.0 ms` 는 오히려 **보수적**이다. 🔴 다만 `600ms` 수용
근거가 **"`0.12 m/s` 에서 약 7cm"** 를 전제로 쓰였으므로, **실제 속도가 상한을 넘는지**는
고친 뒤 실측으로 확인한다.

먼저 R1 대조군으로 0.05m/s를 약 5초만 보낸다. `timeout`과 메시지 수가 이 명령의 상한이고,
끝난 뒤 zero Twist를 3회 보낸다. 움직임·진동·편향이 이상하면 0.12 시험으로 올라가지 않는다.

```bash
timeout --signal=INT --kill-after=2s 15s \
  ros2 topic pub --times 50 -r 10 -w 1 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout --kill-after=2s 12s \
  ros2 topic pub --times 3 -w 1 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

#### 7-c-1. 0.12m/s가 실제로 나오는가 — 3m 지면 실측

펌웨어의 `FEEDFORWARD_MAX_PWM=145`·`MAX_CONTROL_PWM=160` 천장 때문에 명령 0.12m/s에
도달하지 못할 수 있다. 바닥에 시작·종료선을 정확히 3m 간격으로 표시하고 별도 터미널에서
증거를 녹화한다.

```bash
ros2 bag record /odom /imu/yaw_deg /cmd_vel /estop/state -o d0_drive_$(date +%m%d_%H%M)
```

시작선에서 `/odom.header.stamp`를 기록한 뒤 0.12m/s를 보낸다. 로봇 선단의 같은 기준점이
종료선에 닿으면 `Ctrl+C`로 publisher를 끝내고 즉시 zero Twist를 보낸 뒤 끝 stamp를 기록한다.
35초 상한이 먼저 끝나도 시험은 실패가 아니라 **0.12 도달 불확실**로 기록하고 원인을 본다.

```bash
timeout --kill-after=2s 10s ros2 topic echo /odom --field header.stamp --once
timeout --signal=INT --kill-after=2s 35s \
  ros2 topic pub -r 10 -w 1 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.12, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout --kill-after=2s 12s \
  ros2 topic pub --times 3 -w 1 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout --kill-after=2s 10s ros2 topic echo /odom --field header.stamp --once
```

🔴 **2026-08-11 21:46 실측 = `0.3265 m/s`. 명령 `0.12` 의 2.7배이고, 이 시험이 예약 32 를 열었다.**

`d0_drive_0811_2146` — 명령 `linear.x = 0.12` 를 9.30초 발행, `/odom` 경로장 **3.116 m**,
**3.000 m 도달 = 9.47초 → `0.3169 m/s`**(정상구간 `twist.x` 평균 `0.3265`, 범위 `0.3156~0.3405`).
같은 시행 yaw 변화는 엔코더 `4.74°` · IMU `2.77°` 로 직진성 자체는 유지됐다.
⚠ **회전 시험(`§7-c-2`)은 실시하지 못했다** — 붙여넣기에서 줄 연결이 깨져 명령이 안 나갔고
(`IMU yaw` 앞뒤가 `0.3329` 로 동일), 속도 계통 교정 전 데이터는 어차피 다시 찍어야 한다.

🔴 **이 값은 `0.12 도달 불확실`이 아니라 `0.12 초과 확정`이다.** 문서가 예상했던 실패 방향
(PWM 천장 때문에 **못 미친다**)과 **반대**다. 전문·기구·교정값 = `MASTER_PLAN §7` 예약 32.
🔴 **교정 전까지 지면 주행을 하지 않는다.**

`elapsed = (끝 sec−시작 sec) + (끝 nanosec−시작 nanosec)/1e9`,
`실측 평균속도 = 3.0/elapsed`로 계산한다. **벽시계 대신 같은 `/odom`의 header stamp 두 개**를
쓰므로 NTP 역행·stamp 정체도 함께 드러난다. 결과와 bag 경로를 `REAL_ROBOT_VALUES.md §4`에
기록한다. 0.12m/s 미달이면 예약 22의 `max_vel_x`·`desired_linear_vel` 하향 판단이 열린다.

#### 7-c-2. 우회전 명령 오기인가 실제 과속인가

최종 회신은 `angular.z=-0.12`인데 360°/32.13초는 실제 약 −0.196rad/s다. E-stop 안전요원이
있는 같은 평지에서 10초만 재현한다. 시작·끝 `/imu/yaw_deg`의 wrap을 보정한 변화가 약 −69°면
회신 기록 오기, 약 −112°면 실제 우회전 과속이다.

```bash
timeout --kill-after=2s 10s ros2 topic echo /imu/yaw_deg --field data --once
timeout --signal=INT --kill-after=2s 20s \
  ros2 topic pub --times 100 -r 10 -w 1 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.12}}'
timeout --kill-after=2s 12s \
  ros2 topic pub --times 3 -w 1 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
timeout --kill-after=2s 10s ros2 topic echo /imu/yaw_deg --field data --once
```

두 실측은 `D1_FIRST_STEP.md §0-a`의 시작 대조표에 결과를 옮긴다. 추측값이나 구동부 회신값으로
칸을 닫지 않는다.

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
| **펌웨어 정체가 소스와 다르다고 나온다** (08-03 실현) | **판정기 오경보를 먼저 의심한다** | 기본 `topic echo` 는 장문을 **128자에서 자른다**. `--full-length` 로 전체 필드를 다시 본다. 반지름·게인이 그제서야 맞았다 — 자른 문자열로 펌웨어 불일치를 선언하지 않는다 |
| **`/odom --once` 가 "못 받았다"로 FAIL 한다** (08-03 실현) | **정상 발행인데 상한이 짧았다** | 격리 5회가 `0·124·0·0·0`, 성공 지연 약 2.0~2.3초였다. 3초 단발 상한이 정상을 거짓 FAIL 시킨 것이라 8초로 넓혔다. FAIL 한 번으로 센서 두절을 선언하지 말고 주기창·후속 부호를 같이 본다 |
| 인터넷이 없다 | apt·clone·agent 빌드가 전부 막힌다 | ↓ 아래 오프라인 대비 |

**오프라인 대비 (네트워크가 없을 때)** — 소스는 아키텍처와 무관하므로 복사가 통한다:

```bash
# 노트북에서 (build/install/log 를 빼고 소스만)
rsync -av --exclude build --exclude install --exclude log --exclude .git \
      ~/ros2_ws/ /media/USB/ros2_ws/
```

⚠ 그래도 **apt 패키지와 agent 는 복사로 해결되지 않는다.** 그래서 §1 에서 네트워크를
먼저 확인하라고 한 것이다. 인수 전에 확보하는 것이 유일한 진짜 대비다.

## 9. `TODO(D+0)` 전량 목록 — **11건** (이 문서에서 확인해야 할 것)

착수 전에 이 목록을 한 번 읽고, 확인할 때마다 결과를 **이 문서에 적어** 다음 사람에게 남긴다.

| # | 무엇 | 확인 방법 | 절 | D+0 결과 (2026-08-03) |
|---|---|---|---|---|
| 1 | ROS 2 Humble 설치 여부 | `ls /opt/ros` | §1 | ✅ Ubuntu 22.04.5 Jammy·arm64, `/opt/ros/humble` 존재. 🔴 L4T R36.5.0(JetPack 6.2.2 계열)이며 `nvidia-jetpack` 메타패키지·`nvcc`는 **없음** — D+0 비차단이지만 **CUDA 없이는 YOLO 추론이 CPU로 떨어져 역할 B 성능 전제가 통째로 바뀐다.** 소유자·트리거·완료판정은 `MASTER_PLAN.md §7` **예약 27** |
| 2 | 인터넷 연결 | `ping -c 2 packages.ros.org` | §1 | ✅ IPv6 2/2 수신·손실 0%, 평균 175ms |
| 3 | private 저장소 인증 수단 — **D+0 착수 전 게이트** | Jetson에서 실제 clone + 40자 HEAD 대조 | §3 | ✅ HTTPS+fine-grained PAT clone, HEAD `ff0555f899fcc86ff342a3a9ed30742dd1e8b5cf` |
| 4 | `colcon build` 소요 시간 | 실제로 재고 적는다 | §4-c | ✅ Jetson에서 4패키지 종료 0, 28초. `sllidar_ros2` 외부 SDK의 C++ 경고뿐이며 `show-args` 종료 0·`serial_baud=115200` 확인 |
| 5 | agent 확보 성공 여부(A안/B안) | §5-d 의 `topic list` | §5 | ✅ 안 A 소스 빌드·실행, agent 엔티티 생성 및 펌웨어 토픽 8개 확인 |
| 6 | **`micro_ros_arduino` 버전** | ★ 번호를 묻지 말고 `~/Arduino/libraries/` **폴더를 통째로 복사**받는다 | §5-d | ✅ **08-05 수령 완료** — 노트북 `~/Desktop/teensy_required_libraries_v1_4/`, 1946 파일. `micro_ros_arduino` **2.0.8-humble** · Encoder 1.4.3 · BNO055 1.6.4 · Unified Sensor 1.1.15 · BusIO 1.17.4. Teensy 4.x 사전컴파일 `libmicroros.a` 존재 확인. 해시·전수 대조 = §5-d. 🔴 잔여: `FW_VERSION` 문자열(1.3.0)과 폴더명(v1_4) 불일치를 구동부에 확인 |
| 7 | Teensy `idVendor`/`idProduct` | `udevadm info -q property …` | §6 | ✅ `/dev/ttyACM0`, `16c0:0483`, serial `20379630`; `/dev/teensy_drive -> ttyACM0` |
| 8 | `robot_localization` 버전과 구독 QoS | `d0_check.sh` 검사 4·5 — **EKF 를 띄운 뒤**(§7-a)여야 판정이 성립한다 | §7 | ✅ `ekf_filter_node` frequency 30.0; `/odom` RELIABLE→BEST_EFFORT 및 `/imu/data` BEST_EFFORT→BEST_EFFORT 호환·구독 유지 확인 |
| 9 | **NTP 동기 여부** ★08-02 신설 | `timedatectl` → `NTPSynchronized=yes` | §1-b | ✅ 시계 동기화 yes·NTP active·Asia/Seoul |
| 10 | **E-stop 배선 여부** ★08-02 신설 | `d0_check.sh` **검사 8** (버튼을 눌러야 한다. 못 누르면 `s` = 확인 못 함) | §7 | ✅ **2026-08-07 통과** — 전기 시공 완주(`ELECTRICAL_BASELINE.md §14`). `평상시 false` · `누름 → true 전환 확인 — 배선 정상`. 같은 실행에서 **검사 1~8 전항목 OK**. ⚠ 08-03 의 `⚠ 물리 버튼 없음·임시 생략` 은 그때의 사실이고 지금은 아니다. 🔴 **이 통과는 신호 경로에 대한 것이다** — 릴레이 DC 차단 정격은 여전히 미증명(`§14-b`) |
| 11 | **R0 watchdog 실제 정지** ★08-03 §34 보완 | 영상(1차) + `/odom.pose`(정본 측정) 교차 확인 | §7-c-0 | ✅ **종결 — 2026-08-11 (검토 §59 확인).** 🔴 **08-10 재산출 = `519.9 / 532.0 / 516.2 ms`**(구 기준 500ms 초과 16.2~32.0ms). ~~537.1~~ 은 검토 §52 가 결함을 재현한 구판 도구의 값이라 폐기했다. ✅ **08-11 영상 분석 완료** — `28프레임=466.7ms`(bag 1522 와 같은 시행) · 🔴 **영상은 하한이라 PASS 를 못 만든다**(bag 과의 차이 49.5ms 는 **관측계 차이**이지 렌더 지연 확정이 아니다 — 검토 §57.2) · ✅ **조건 2 는 펌웨어 독립 증거로 충족**(정지 후 0.595 mm/s). ✅ **결정 1 = ⓐ 기준 재정의**(08-11 사용자) — 🔴 **구현자 정식화는 검토 §57 에서 확인 보류**. ✅ **사용자 근거 한 줄 = 08-11 결정**(`0.12 m/s` 에서 약 7cm 수용 · 재개방 = 상한 상향). ✅ **08-11 19:41 현행 펌웨어 재측정 완료** — `d0_watchdog_0811_1938` **`516.0 ms`**(1-b `≤600ms` 충족) · 조건 2 = 정지 후 37,130ms `pose` 완전 고정 · 영상 `IMG_3483.mov` 교차 `450.1ms`(하한) · 🔴 **`관측 완전성 413~650 238프레임 ✅`**(전 구간은 손각대 흔들림으로 판정 불능 — 끊은 사실을 판정과 같이 읽는다). ✅ **검토 §59 = 증거 승인 · 정식화 확인 · `#11` 종결 · R1 진입 가능**(P0 0 · P1 0 · P2 2). 🔴 **재개방 = 속도 상한 상향 · 재굽기/안전 배선 변경 · 자율 발행자 연결.** 전문 = §7-c-0 |

## 10. 다음 단계

🔴 **현재 상태부터 읽는다 (2026-08-05 기준) — 여기서 R1 로 바로 갈 수 없다.**
`d0_check.sh` 는 아직 **종료 2(불완전)** 이고 **검사 8(E-stop `false→true`)은 확인된 적이
없다.** 물리 E-stop 이 설치 중이며, 게다가 이번 회차 전장은 **PIN 21 미연결**이라
`/estop/state` 는 눌러도 `false` 를 발행한다(`ELECTRICAL_BASELINE.md §4-b`·`§4-c`).
→ **지금의 복귀점은 §11-g 다.** 비구동 선행 작업은 `§11`, 그 완료 표기와 복귀점은 `§11-g`.

주행 허용 전제는 여기에 다시 적지 않는다 — **`§7-c` 첫 줄의 전제 목록이 유일한 정본**이고,
그 첫 항목이 *"검사 8에서 E-stop `false→true` 전환을 실제로 확인했다"* 이다. 하나라도
아니면 **모터 명령 금지**다. 이 문서에 같은 전제를 두 번 적어 두 판본이 갈리는 것을 막는다.

**순서 (전제가 닫힌 뒤에만)**: `d0_check.sh` **종료 0** → §7-c 의 R0 watchdog → R1 → R2 를
순서대로 실측하고 결과를 **`docs/D1_FIRST_STEP.md §0-a`** 에 옮긴다. 그다음
agent → TF 트리 → EKF → **R3 rosbag** 이다. R0→R1→R2를 건너뛰고 R3로 가지 않는다.

⚠ 이 절은 2026-08-04 검토 §39.6.1(P1)로 고쳤다. 구판은 *"종료 0 이면 통과"* 만 적어
**§9 표의 ✅ 9건을 보고 내려온 사람이 E-stop 없이 R1(0.05m/s 지면 명령)로 갈 수 있었다.**
방어는 §7-c 에 있었지만 **두 절 아래**였고 이 문장은 통과시켰다.

## 11. 물리 E-stop 설치 대기 중 비구동 선행 작업 — `pre-R3 diagnostic`

> **적용 조건**: 2026-08-03처럼 물리 E-stop이 아직 없지만 라이다·TF·센서·Jetson 상태를
> 먼저 확인하려는 경우에만 쓴다. 이 절은 §7의 검사 8, §7-c의 R0~R2 또는
> `D1_FIRST_STEP.md`의 정식 R3를 대체하지 않는다.

### 11-a. 시작 조건과 금지선

아래 네 조건을 **전부** 만족해야 §11-b로 간다.

1. 전기 작업 담당자가 배터리에서 두 MDD10A로 가는 **모터 전력 계통을 물리적으로 분리**하고,
   두 드라이버의 모터 공급 전압이 0V임을 확인했다. Jetson·Teensy·라이다의 제어 전원은
   유지해도 되지만, 활선 상태에서 배선·단자를 만지지 않는다.
2. 바퀴는 공중에 띄우거나 구름 방지 조치를 하고, 작업 반경 안에 사람이 들어가지 않는다.
3. 아래 확인에서 Nav2·미션·수동 조종기처럼 `/cmd_vel`을 낼 수 있는 노드가 보이면 중단하고
   그 프로세스를 정상 종료한 뒤 다시 확인한다.
4. 이 절에서는 **`/cmd_vel`을 한 번도 발행하지 않는다.** `real_bringup` 전체 런치,
   R0 watchdog, R1·R2, `make_map.sh`, Nav2 goal·미션 명령도 실행하지 않는다.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 node list
ros2 topic info /cmd_vel -v
```

⚠ `Ctrl+C`는 여기서 띄운 **관측 프로세스를 종료하는 수단**일 뿐 E-stop이 아니다. 모터 전력이
분리됐다는 1번 조건을 대신하지 않는다. 1번을 확인할 수 없으면 이 절도 시작하지 않는다.

### 11-b. 펌웨어 재현 환경 수령

`§5-d`의 미완료 항목을 먼저 닫는다. 구동부 개발 PC 또는 인계 USB에서
`~/Arduino/libraries/`를 **폴더 전체로** 받아 별도 보관하고, 원본을 수정하지 않은 상태에서
목록과 해시를 남긴다. 인계 매체가 아직 없으면 `미수령`으로 기록하고 다음 비구동 항목은
계속할 수 있지만 D+0 완료로 승격하지 않는다.

```bash
find ~/Arduino/libraries -mindepth 1 -maxdepth 1 -printf '%f\n' | sort
find ~/Arduino/libraries -type f -print0 | sort -z | xargs -0 sha256sum > ~/Arduino/libraries.sha256
wc -l ~/Arduino/libraries.sha256
```

### 11-c. 라이다 장착·높이·포트 확인

기계 장착과 높이 계산은 `D1_FIRST_STEP.md §2`의 제약을 그대로 따른다. 스캔 평면을 모든
차체 구조물보다 위, 임시 기준으로 몸통 최상면보다 0.05m 이상 높이고 x/y는 정중앙에 둔다.

**현장 상태(2026-08-03)** — 라이다 작업은 다음 날로 미뤘다. 장착·스캔면 높이·시리얼
포트·USB 정체·`/scan`은 전부 미확인이다. 따라서 오늘은 §11-d에서 라이다 TF를 제외한
agent·odom·IMU·EKF만 확인하고, `/scan`이 필요한 §11-e 정식 사전 bag 판정과 §11-f의
4토픽 동시부하 판정은 실행하지 않는다. 라이다를 장착한 뒤 이 절 처음으로 돌아온다.

1. 바닥에서 라이다 스캔 평면까지 높이를 m 단위로 잰다.
2. 계산값 `lidar_joint z = 측정 높이 - 0.053m`를 기록한다.
3. 현재 묶음은 `src/**` 동결 중이므로 **측정값만 기록하고 URDF는 아직 수정하지 않는다.**
4. 포트와 USB 정체를 확인한다. 번호가 재삽입 때 바뀌면 라이다 전용 udev 별칭을 별도 구현한다.

```bash
ls -l /dev/ttyUSB*
udevadm info -q property -n /dev/ttyUSB0 | grep -E '^ID_VENDOR_ID=|^ID_MODEL_ID=|^ID_SERIAL_SHORT='
```

터미널 C에서 드라이버만 기동한다. `/dev/ttyUSB0`가 아니면 위에서 확인한 실제 포트를 넣는다.

```bash
ros2 run sllidar_ros2 sllidar_node --ros-args \
  -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=460800 \
  -p frame_id:=lidar_link -p angle_compensate:=true -p scan_mode:=Standard
```

다른 터미널에서 유한 시간으로 확인한다.

```bash
timeout --signal=INT --kill-after=2s 12s ros2 topic hz /scan
timeout --kill-after=2s 10s ros2 topic echo /scan --field header.frame_id --once
```

### 11-d. agent·정적 TF·EKF 비구동 확인

터미널 A에서 `§5-d`의 agent를 계속 띄우고, Teensy USB 전원을 재인가한 뒤 `§5-e`의 8.7초
정지 조건을 지킨다. 터미널 D에는 robot_state_publisher, 터미널 E에는 EKF만 띄운다.

```bash
# 터미널 D
ros2 run robot_state_publisher robot_state_publisher \
  ~/ros2_ws/src/tunnel_bringup/urdf/robot_real.urdf
```

```bash
# 터미널 E
ros2 run robot_localization ekf_node --ros-args \
  --params-file ~/ros2_ws/src/tunnel_bringup/config/ekf_real.yaml
```

관측 터미널에서 다음을 확인한다. 현재 URDF의 라이다 z는 미실측 표시인 0이므로
`base_footprint→lidar_link`는 값 확정이 아니라 **연결 여부만** 본다.
라이다를 아직 장착하지 않은 2026-08-03 실행에서는 해당 명령만 건너뛰고 나머지를 수행한다.

```bash
timeout --kill-after=2s 10s ros2 run tf2_ros tf2_echo base_footprint imu_link
timeout --kill-after=2s 10s ros2 run tf2_ros tf2_echo base_footprint lidar_link
timeout --signal=INT --kill-after=2s 12s ros2 topic hz /odometry/filtered

timeout --kill-after=2s 10s ros2 topic echo /odom --field header.frame_id --once
timeout --kill-after=2s 10s ros2 topic echo /odom --field child_frame_id --once
timeout --kill-after=2s 10s ros2 topic echo /imu/data --field header.frame_id --once
timeout --kill-after=2s 10s ros2 topic echo /odom --field twist.covariance --once
timeout --kill-after=2s 10s ros2 topic echo /imu/data --field angular_velocity_covariance --once
```

기대 frame은 차례로 `odom`, `base_footprint`, `imu_link`다. IMU TF 기대값은 바닥 기준
z=0.392m, yaw=-90°다. 다르면 remap으로 덮지 않고 관측값을 기록한다.

**현장 결과(2026-08-03)** — agent·robot_state_publisher·`ekf_filter_node`가 함께 생존했고
`/odometry/filtered`는 12초 창에서 평균 29.999~30.003Hz, 간격 0.033~0.034초였다.
frame_id 3종은 `odom`·`base_footprint`·`imu_link`로 전부 일치했다. IMU TF는
translation `[0,0,0.392]`, yaw `-1.571rad(-90°)`로 기대값과 일치했다. `tf2_echo`의 최초
1회 `frame does not exist`는 디스커버리 대기 뒤 같은 변환이 반복 수신돼 실패가 아니다.
`/odom twist.covariance`는 x·y `0.02`, yaw `0.1`; `/imu/data angular_velocity_covariance`는
x·y·z `0.0025`로 확인돼 전부 0인 미기입 값이 아니다. 라이다 관련 값만 §11-c 미결로 남는다.

### 11-e. 비구동 진단 bag과 판정

이 녹화는 정식 R3가 아니다. 디렉터리와 문서 어디에서도 `r3_PASS` 같은 이름을 쓰지 않고
반드시 `pre_r3_no_estop`을 사용한다. 모터 전력이 분리된 상태에서 정지 60초 → 바퀴를 손으로
앞뒤로 30초 → 좌우 바퀴를 손으로 반대 방향으로 돌려 회전 부호를 30초 → 정지 30초로 기록한다.

**라이다 미장착일의 부분 녹화** — `/scan`이 없으면 아래 명령으로 odom·IMU만 먼저
수집할 수 있다. 출력 이름에 `partial_odom_imu`를 반드시 넣고, 분석도 두 입력만 한다.
정상이더라도 3토픽 사전 판정이나 R3 PASS가 아니며 라이다 장착 뒤 아래 본 녹화를 새로 한다.
⚠ 이 부분 녹화에는 `/tf`·`/tf_static`을 섞지 않는다. 2026-08-03 실측에서 TF를 같이 담으면
TF 계열이 bag 시작점을 먼저 만들고 센서 구독 DDS 매칭 전 약 0.81초가 두 센서의 앞 공백으로
잡혔다. TF까지 필요한 본 녹화의 시작점 계약은 정식 R3 전에 판정기 트랙에서 별도로 해결한다.

```bash
mkdir -p ~/pre_r3_bags
cd ~/pre_r3_bags
timeout --signal=INT --kill-after=5s 180s \
  ros2 bag record /odom /imu/data \
  -o pre_r3_no_estop_partial_odom_imu_$(date +%m%d_%H%M)
```

```bash
cd ~/ros2_ws
python3 tools/bag_gap_report.py \
  ~/pre_r3_bags/pre_r3_no_estop_partial_odom_imu_MMDD_HHMM /odom /imu/data
```

**현장 결과(2026-08-03)** — 최초 149.6초 bag은 odom 7084개·IMU 7085개, 내부 stamp 최대
간격 22.00ms·25.68ms와 엄격 단조를 확인했지만, `/tf`·`/tf_static`을 함께 담아 두 센서 모두
앞 공백 약 0.81초로 RC 1이었다. 원인 분리용 센서 전용 44.3초 대조 bag은 odom 2111개·IMU
2110개, 평균 47.64·47.62Hz, 수신 최대 간격 22.55·25.00ms, stamp 최대 간격 22.00·24.87ms,
양끝 포함 계약 초과 0건·엄격 단조로 RC 0이었다. 따라서 센서 두절로 승격하지 않는다.
🔴 **다만 원인은 "TF 포함 녹화의 관측 시작점" 이라는 가설이며 아직 미확정이다** (08-04 검토
§39.3 P2-3). 두 bag 은 **TF 유무와 길이(149.6초 vs 44.3초)가 동시에 다르다** — 앞 공백은
길이에도 민감한 양이라 이 대조로는 TF 를 원인으로 **분리하지 못한다**. 단일변수 통제는
**같은 길이로 TF 유/무 두 번**을 찍는 것이고, 그 판정은 정식 R3 전 판정기 트랙이 소유한다.
그때까지 "TF 문제로 규명됐다"고 인용하지 않는다.
bag 경로는 각각
`~/pre_r3_bags/pre_r3_no_estop_partial_odom_imu_0803_1504`와
`~/pre_r3_bags/pre_r3_no_estop_endpoint_control_0803_1511`이며,
**이 두 bag 이 위 대조군이므로 지우지 않는다**(보존 규정 = `§11-g`).

아래는 라이다 장착 뒤 `/scan`까지 포함하는 **본 사전 녹화**다.

```bash
mkdir -p ~/pre_r3_bags
cd ~/pre_r3_bags
timeout --signal=INT --kill-after=5s 180s \
  ros2 bag record /odom /imu/data /scan /tf /tf_static \
  -o pre_r3_no_estop_$(date +%m%d_%H%M)
```

생성된 실제 경로를 넣어 간격·bag 양끝 공백·`header.stamp` 중복/역행을 판정한다.
토픽별 숫자 계약은 문서에 복사하지 않고 도구의 `TOPIC_POLICY` 출력만 사용한다.

```bash
cd ~/ros2_ws
python3 tools/bag_gap_report.py \
  ~/pre_r3_bags/pre_r3_no_estop_MMDD_HHMM /odom /imu/data /scan
```

판정 결과는 **사전 진단값**이다. 정상이어도 `D1_FIRST_STEP.md §5`의 R3 PASS 칸이나
`REAL_ROBOT_VALUES.md`의 확정값을 닫지 않는다. 비정상이면 bag 경로·도구 전체 출력·NTP 상태를
보존해 E-stop 설치 뒤 정식 R3에서 재현 여부를 본다.

### 11-f. Jetson 10분 동시부하 관찰

agent·라이다·robot_state_publisher·EKF만 띄운 상태에서 실행한다. Nav2·SLAM·미션은 포함하지
않으므로 이 결과를 전체 스택 성능으로 부르지 않는다.

라이다가 없는 날에는 agent·robot_state_publisher·EKF만으로 아래 부분 부하를 먼저 볼 수 있다.
파일명과 판정에 `partial_odom_imu`를 남기며, `/scan`을 포함한 4토픽 동시부하를 대신하지 않는다.

```bash
mkdir -p ~/pre_r3_logs
timeout --signal=INT --kill-after=5s 600s tegrastats --interval 1000 \
  | tee ~/pre_r3_logs/tegrastats_no_estop_partial_odom_imu_$(date +%m%d_%H%M).log
```

```bash
ros2 node list
timeout --signal=INT --kill-after=2s 12s ros2 topic hz /odom
timeout --signal=INT --kill-after=2s 12s ros2 topic hz /imu/data
timeout --signal=INT --kill-after=2s 12s ros2 topic hz /odometry/filtered
```

**현장 결과(2026-08-03)** — 부분 부하 로그
`~/pre_r3_logs/tegrastats_no_estop_partial_odom_imu_0803_1516.log`에 596개 1초 표본을 남겼다.
최고 온도는 tj·gpu·soc1 49.375°C, RAM 최고 1075/7607MB(약 14.1%), swap 0MB였다.
10분 뒤에도 agent·robot_state_publisher·EKF와 transform listener가 생존했다. 후속 12초 창은
odom 47.611Hz·최대 22ms, IMU 47.657Hz·최대 25ms, EKF 29.999Hz·최대 34ms였다.
`/odometry/filtered`의 최초 `does not appear` 1회는 DDS 발견 뒤 30Hz가 연속 수신돼 장애가 아니다.
따라서 **라이다 없는 부분 soak는 통과**했지만 `/scan` 포함 4토픽·전체 Nav2 스택 성능으로
승격하지 않는다.

아래는 라이다 장착 뒤 수행하는 **4토픽 본 부하 관찰**이다.

```bash
mkdir -p ~/pre_r3_logs
timeout --signal=INT --kill-after=5s 600s tegrastats --interval 1000 \
  | tee ~/pre_r3_logs/tegrastats_no_estop_$(date +%m%d_%H%M).log
```

끝난 뒤 입력·출력 노드가 그대로 있는지 확인한다.

```bash
ros2 node list
timeout --signal=INT --kill-after=2s 12s ros2 topic hz /odom
timeout --signal=INT --kill-after=2s 12s ros2 topic hz /imu/data
timeout --signal=INT --kill-after=2s 12s ros2 topic hz /scan
timeout --signal=INT --kill-after=2s 12s ros2 topic hz /odometry/filtered
```

### 11-g. 이 절의 완료 표기와 복귀점

다음을 한 묶음으로 남기면 **`pre-R3 diagnostic 완료`**라고만 기록한다.

- 모터 전력 분리 확인자·시각·확인 방법
- Arduino 라이브러리 수령 여부와 해시 파일
- 라이다 포트·USB 정체·스캔면 실측 높이·계산한 URDF z
- frame_id 3종·IMU/라이다 TF·covariance 출력
- bag 경로와 `bag_gap_report.py` 전체 결과
- tegrastats 로그 경로와 전후 네 토픽 생존 결과

🔴 **산출물 보존 규정 (08-04 검토 §39.6.2 P2-D — 경로만 적는 것으로는 부족하다)**

원본 bag·로그는 지금 **Jetson 로컬에만** 있고, 저장소에는 요약과 판정만 있다. 그런데
`D1_FIRST_STEP.md` 는 D+0 산출물을 *"다시 판정하지 말고 그대로 소비한다"* 로 의존하고,
`§11-e` 의 TF 가설도 이 bag 들이 대조군이다. **원본이 사라지면 소비할 것이 요약문뿐이다.**

- **보존 위치**: Jetson 밖 **최소 한 곳** — 노트북 `~/robot_evidence/` (1차) + USB(2차).
- **복제 수단**: `scp -r hanhan@jetson.local:~/pre_r3_bags ~/robot_evidence/` ·
  `scp -r hanhan@jetson.local:~/pre_r3_logs ~/robot_evidence/`
- **보존 기간**: **R3 정식 통과 + 예약 25 종결 시점까지**. 그 전에는 지우지 않는다.
- ⚠ **재플래시·SD 교체는 재개방 시점이 아니라 소실 시점이다.** Jetson 이 살아 있는 지금만
  유효한 기회이므로, 다음 현장 세션의 **첫 명령**으로 복제를 수행한다.
- 대상 3건: `pre_r3_no_estop_partial_odom_imu_0803_1504` ·
  `pre_r3_no_estop_endpoint_control_0803_1511` ·
  `~/pre_r3_logs/tegrastats_no_estop_partial_odom_imu_0803_1516.log`
- ✅ **2026-08-07 복제 완료** — 노트북 `~/robot_evidence/`(`pre_r3_bags` 13MB ·
  `pre_r3_logs` 168KB). 무결성 확인: 두 bag 이 `ros2 bag info` 로 열리고 메시지 수가
  **18,659건 / 4,221건**으로 온전하다. 🔴 **USB 2차 사본은 아직 없다** — 1차만 있다.
  ⚠ 같은 날 Jetson 을 세 번 전원 순환했고 매번이 소실 기회였다. **첫 명령 규정은 유효하다.**

**2026-08-03 부분 진행 상태** — agent·odom·IMU·EKF·frame/covariance, 센서 전용 부분 bag,
라이다 없는 10분 부분 soak까지 완료했다. Arduino 라이브러리 원본, 라이다 전 항목,
`/scan` 포함 3토픽 bag·4토픽 soak, E-stop·R0~R2는 미완료다. 따라서 §11 전체 완료나
D+0/R3 통과가 아니다.

물리 E-stop 설치가 끝나면 **§7 검사 8로 돌아가** `false→true`를 확인한다. 그 뒤에만
§7-c R0 watchdog → R1 → R2를 순서대로 실행하고, 모두 통과한 다음
`D1_FIRST_STEP.md`의 정식 R3를 새 bag으로 다시 수행한다. 이 절의 bag을 정식 R3로 이름만
바꿔 재사용하지 않는다.

## 근거 문서

`MASTER_PLAN.md §3` · `MASTER_PLAN.md §6` · `MASTER_PLAN.md §7` ·
`REAL_ROBOT_VALUES.md §1` · `REAL_ROBOT_VALUES.md §4` · `REAL_ROBOT_VALUES.md §5` ·
`CLAUDE.md §2` · `AGENTS.md §5` · `D1_FIRST_STEP.md §1`
