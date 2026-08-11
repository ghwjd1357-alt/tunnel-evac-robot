# D1_FIRST_STEP.md — 인수 다음 날(D+1) 첫 스텝 런북: R3 rosbag (2026-08-02 신설, S6-5)

> **시작 조건**: `bash tools/d0_check.sh` 가 **종료 0**(전량 통과)이고,
> `JETSON_SETUP.md §7-c`의 R0 watchdog과 R1·R2 사전 실측 결과가 아래 §0-a에 기록돼 있다. 종료 2(불완전)는
> 시작 조건이 아니다 — 건너뛴 검사를 먼저 채운다. 셋업 전체는 `JETSON_SETUP.md`.
>
> **끝 조건**: R3 통과 = *"odom/imu/scan 의 주기·timestamp 단조·covariance 가 EKF 재료로
> 쓸 만한가"* 를 **녹화된 데이터로** 판정했다 (`MASTER_PLAN.md §3`).
>
> ⚠ 이 문서도 **장비 없이 썼다.** 확인 못 한 것은 `TODO(D+1): 확인` 으로 남기고 확인
> 방법을 같이 적었다. 다만 아래 **§5 의 분석 도구는 노트북에서 실제 bag 으로 검증**했다.

## 0. 오늘 하는 일 한 장 요약

```
① agent 기동      → /odom · /imu/data 가 흐른다
② 라이다 기동      → /scan 이 흐른다          ★ 장착·측정이 선행 (§2)
③ TF 트리 확인     → base_footprint → lidar_link · imu_link 가 이어진다
④ EKF 기동        → odom → base_footprint TF 를 EKF 가 만든다
⑤ R3 rosbag 녹화  → 그리고 **분석해서 판정**한다 (녹화만 하는 것이 아니다)
```

★ **순서를 바꾸지 않는다.** 각 단계는 앞 단계가 만든 것을 소비한다. 예를 들어 EKF 를
먼저 띄우면 입력이 없어 "EKF 가 이상하다"로 보이는데, 실제 원인은 agent 다.

★ **R3 녹화 중에는 로봇을 모터로 주행시키지 않는다.** R1·R2의 지면 주행은 R3보다 먼저
`JETSON_SETUP.md §7-c`에서 끝낸다. R6는 *최초 주행*이 아니라 **최초 Nav2 자율주행**이다.
R0→R1→R2→R3 순서를 건너뛰면 무엇이 틀렸는지 못 가른다.

### 0-a. ★ 펌웨어 소스 발견 5건 — R3 시작 전 인계 대조

아래는 참고 메모가 아니라 **R3 입력의 전제**다. 결과가 비어 있으면 §1로 가지 말고
`JETSON_SETUP.md §7-c`로 돌아간다.

| 항목 | 실차 전제·판정 | D0/R1·R2 결과 |
|---|---|---|
| 첫 직진 안전 공간 | 오픈루프 횡편차를 모르므로 평지에서 **양옆 1m 이상** 확보 | `TODO(D+1): 확인` |
| 최고속 0.12m/s | 3m 실주행의 `/odom.header.stamp` 차로 평균속도 계산. PWM 천장 때문에 미달 가능 | `TODO(D+1): 확인` |
| 우회전 각속도 | `angular.z=-0.12` 10초 뒤 `/imu/yaw_deg` 변화: 약 −69°=회신 오기, 약 −112°=실제 과속 | `TODO(D+1): 확인` |
| odom 의미 | pose는 raw encoder 적분, twist는 EMA(`α=0.10`, 시정수 약 0.2초). EKF는 **twist** 소비 | R3 가감속·covariance 판정에 반영 |
| 감속 능력 | 역 PWM을 의도적으로 쓰지 않아 능동 제동 없음. 경사 하강 자유주행 가능 | R6 경사 시험 전 별도 안전판정 |
<!-- watchdog-evidence-slot:start -->
| R0 watchdog | 명령 단절 뒤 실제 바퀴 정지. EMA twist가 아니라 영상(1차)+raw pose(정본 측정)로 판정 | ✅ **충족 — `516.0 ms` · 검토 §59 확인 · `#11` 종결** (`JETSON_SETUP §7-c-0`) |

**🔴 조건 3 기록 — 현행 펌웨어 재측정 (2026-08-11 19:41 · `build=Aug 11 2026 15:13:20`).**
**R1 허가 근거로 쓰는 시행은 이쪽이다** — 08-07 증거는 승계가 불인정됐다(§57.5).

| | 값 |
|---|---|
| 영상 | `~/Desktop/d0_evidence/video/IMG_3483.mov` (3840×2160 HEVC) |
| fps | **59.9925**(실측) · 1 프레임 = **16.669 ms** · 804 프레임 / 13.402초 |
| 센 프레임 수 | **27** (T0 = n**473** = `publishing #30` 표시 → T1 = n**500** = 마지막 회전) |
| 환산 시간 | **450.1 ms** 🔴 **하한이다** · bag 과의 **관측계 차이 65.9 ms** — 🔴 한 원인으로 특정하지 않는다 |
| bag 경로 | `~/robot_evidence/d0_watchdog_0811_1938` (젯슨 원본 보존 · **같은 시행**, 주행 `19:41:14.909~17.639`) |
| bag 실측 | **516.0 ms** (계약 500ms 대비 +16.0ms) · 민감도 `2mm/s→536.7` `5→516.0` `10→495.0` `20→473.8` |
| PASS/FAIL | ✅ **PASS (결정 1-ⓐ 기준 · 검토 §59 확인)** — `1-a` 구조 + `1-b 516.0ms ≤ 600ms` + 조건 2 + 조건 3. 🔴 **영상 단독으로는 여전히 PASS 를 못 만든다**(하한) |

- ✅ **조건 2 충족(2중)** — bag: 정지 후 **37,130 ms** 관찰에 `pose` 가 `0.2620 / 0.3978` 로
  완전 고정. 영상(펌웨어 독립): 정지 후 2500ms 누적 `-1.270°` = **0.5018 mm/s**.
  ✅ **관측 완전성 = `413~650 238프레임 전량 연속·유한·유효점≥30`**.
- 🔴 **영상 분석 구간을 `413~650` 으로 끊었다** — `651` 이후는 손각대 흔들림이 잡음 바닥을
  올려 **끝 범위를 어디로 잡아도 fail-closed** 다(사유 2종·재현 명령 = `§7-c-0`). 끊은 구간이
  조건 2 요구치(2.0초)보다 긴 2.5초를 담는다. 흔들림 구간이 바퀴 회전이 아니라는 근거는
  `JETSON_SETUP §7-c-0` 에 셋으로 적혀 있다(그중 하나가 같은 구간 bag `pose` 고정이다).
- 재계산 = `python3 tools/watchdog_video.py <영상> --t0-frame 473 --preset 0811-1938 --bag-ms 516.0`
- ✅ **`#11` 종결** — 검토 §59 가 위 값을 독립 재현하고 정식화를 확인했다(P0 0 · P1 0).
  🔴 **재개방 조건은 살아 있다** — 속도 상한 상향 · 재굽기나 안전 경로 배선 변경 ·
  자율 발행자가 `/cmd_vel` 에 붙을 때.

**참고 기록 (2026-08-07 시행 · 🔴 승계 불인정 — 현행 펌웨어 결과로 쓰지 않는다).**

(아래는 `§7-c-0` 이 요구한 다섯 항목을 **08-07 시행**에 대해 채운 기록이다.)

| | 값 |
|---|---|
| 영상 | `~/Desktop/d0_evidence/video/IMG_3461.mov` (iPhone 14 Pro · 3840×2160 HEVC) |
| fps | **59.9955**(실측 `avg_frame_rate`) · 1 프레임 = **16.668 ms** · 1327 프레임 / 22.118초 |
| 센 프레임 수 | **28** (T0 = n**670** = `publishing #30` 표시 → T1 = n**698** = 마지막 회전) |
| 환산 시간 | **466.7 ms** 🔴 **하한이다**(T0 는 늦게 뜨고 T1 은 이르게 잡힌다) · bag 과의 **관측계 차이 49.5 ms** — 🔴 렌더 지연으로 특정하지 않는다(검토 §57.2) |
| bag 경로 | `~/Desktop/d0_evidence/d0_watchdog_0807_1522` (**같은 시행** — 촬영 15:22:58) |
| PASS/FAIL | 🔴 **판정 불능**. 구 조건 1 은 구조상 달성 불가 · 영상은 PASS 를 못 만든다 |

- 재계산 = `python3 tools/watchdog_video.py <영상> --t0-frame 670 --preset 0807-1522 --bag-ms 516.2`
- ✅ **조건 2 는 충족** — 정지 후 3267ms 누적 회전 `+1.963°` = **0.595 mm/s**(판정선 5 mm/s).
  🔴 이 값만 **펌웨어와 독립된 관측**이다(나머지는 `/odom.pose` 파생).
  ✅ **관측 완전성 = `610~894 285프레임 전량 연속·유한·유효점≥30`** — 이 줄이 `✅` 여야 조건 2
  값을 쓸 수 있다(검토 §58 조건부 수용 전제). 바퀴 중심이 누적 상태라 `T0` **앞** 결측도
  조건 2 를 오염시킨다 — 실영상 공격에서 `0.5945` → `0.1487 mm/s`(안전 반대 방향).
- 🔴 **`#11` 은 열려 있다.** 결정 1 = 안 ⓐ(기준 재정의)가 08-11 에 내려졌으나 구현자 정식화는
  검토 §57 에서 **확인 보류**됐다. 위 실측은 **굽기 전(08-07)** 펌웨어의 값이고 🔴 **현행
  펌웨어로의 승계는 불인정**됐다(`f57d454..a7d1483` 에서 안전 경로 배선이 바뀌었다) —
  ✅ 사용자 근거 한 줄 = 08-11 결정(`0.12 m/s` 에서 약 7cm 수용 · 재개방 = 속도 상한 상향).
  ✅ **재측정은 08-11 19:41 에 끝났고 검토 §59 가 확인했다**(위 블록) — `#11` 종결.
<!-- watchdog-evidence-slot:end -->

⚠ 3m 거리로 확인한 pose 정확도를 곧바로 EKF twist 정확도로 승격하지 않는다. 등속 평균은
보존되지만 가감속 구간에는 약 0.2초 지연이 있으므로 §5의 bag과 R4 잔차에서 따로 본다.

## 1. agent 기동 (터미널 A — 계속 띄워 둔다)

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/teensy_drive -b 115200
```

Docker 로 갔다면 `JETSON_SETUP.md §5-d` 의 두 번째 명령을 쓴다.
확인(터미널 B):

```bash
ros2 topic list | grep -E "odom|imu"
```

## 2. ★ 라이다 — 장착과 측정이 **먼저**다 (오늘의 가장 큰 미결)

⚠ **라이다 스캔면 높이는 아직 없다.** 5번 요청해도 안 온 이유가 **라이다 미장착**으로
확인됐고, 08-02 에 **장착·측정을 역할 A 가 흡수**하기로 방침이 바뀌었다
(`REAL_ROBOT_VALUES.md §4`). 즉 **오늘 우리가 달고 우리가 잰다.**

**지켜야 할 제약** (`PITFALLS.md §7` · 합의사항 §6.1):

- 스캔 평면은 **몸통 최상면과 모든 고정 구조물보다 위**여야 한다. 같은 높이면 레이저가
  자기 몸통 모서리를 상시 타격해(시뮬에서 ±90° 0.20m 고정 히트 실측) 지나간 자리마다
  **유령 장애물**을 그리고 경로가 막힌다. 임시 기준 = 몸통 최상면 **+0.05 m 이상**.
- 회전 중심(정중앙 x=0, y=0)에 맞춘다 — x/y 는 이미 정중앙으로 받았다.

**측정과 반영**:

```bash
# ① 바닥에서 스캔 평면까지 줄자로 잰다 (단위 m)
# ② URDF 의 lidar_joint 에 넣을 값 = (잰 높이) − 0.053    ← 0.053 = 바퀴축 높이
#    ⚠ base_link 의 부모가 바퀴축이라 기준면을 옮겨야 한다. IMU 때와 같은 뺄셈이다.
nano ~/ros2_ws/src/tunnel_bringup/urdf/robot_real.urdf     # lidar_joint 의 origin xyz
cd ~/ros2_ws && colcon build --symlink-install --packages-select tunnel_bringup
```

⚠ **`z=0` 인 채로 R5(지도 제작)를 하면 지도가 통째로 못 쓰게 된다.** 지금 URDF 의 `0` 은
측정값이 아니라 **"아직 안 쟀다"는 표시**다(그럴듯한 숫자를 넣지 않은 이유가 이것이다).
R3 녹화 자체는 z 가 틀려도 되지만(스캔 원본은 그대로다), **잰 김에 오늘 채운다.**

TODO(D+1): 확인 — 잰 높이와 URDF 에 넣은 값을 `REAL_ROBOT_VALUES.md §2` 3-a 에 기록한다.

라이다 기동(터미널 C) — 런치 전체를 띄우기 전에 드라이버만 먼저 본다:

```bash
ros2 run sllidar_ros2 sllidar_node --ros-args \
  -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=460800 \
  -p frame_id:=lidar_link -p angle_compensate:=true -p scan_mode:=Standard
```

```bash
ros2 topic hz /scan          # 확인
```

TODO(D+1): 확인 — 라이다 포트가 `/dev/ttyUSB0` 이 맞는지(`ls /dev/ttyUSB*`).
번호가 바뀌면 Teensy 때와 같은 문제다 → udev 규칙을 하나 더 만든다.

## 3. TF 트리 확인 — "스캔이 로봇 어디에 붙어 있는가"

```bash
ros2 run robot_state_publisher robot_state_publisher \
  ~/ros2_ws/src/tunnel_bringup/urdf/robot_real.urdf
```

```bash
ros2 run tf2_tools view_frames      # frames.pdf 를 만든다
ros2 run tf2_ros tf2_echo base_footprint lidar_link
ros2 run tf2_ros tf2_echo base_footprint imu_link
```

**기대값** (08-02 노트북에서 URDF 로 실제 확인한 값):

| 변환 | 기대 | 근거 |
|---|---|---|
| `base_footprint → base_link` | z = **0.053** | 3차 회신 §10 실측 축높이 |
| `base_footprint → imu_link` | z = **0.392**, yaw **−90°** | 3차 회신 §10 (바닥기준) — 0.339 + 0.053 |
| `base_footprint → lidar_link` | z = **§2 에서 잰 값** | 오늘 측정 |

★ `base_footprint → imu_link` 가 **0.392** 로 나오는지 꼭 본다. 이 값이 구동부가 준
바닥 기준 실측과 **같아야** 뺄셈이 맞은 것이다. 다르면 URDF 의 두 자리(축높이·IMU) 중
하나가 틀렸다.

⚠ 이 시점에 `odom → base_footprint` 는 **아직 없다.** 그건 EKF 가 만든다(다음 절).
"TF 트리가 끊겼다"는 경고가 보이면 정상이다.

## 4. EKF 기동

```bash
ros2 run robot_localization ekf_node --ros-args \
  --params-file ~/ros2_ws/src/tunnel_bringup/config/ekf_real.yaml
```

⚠ **노드 이름을 바꾸지 않는다.** yaml 최상단 키가 `ekf_filter_node:` 라서, `-r __node:=…`
로 이름을 바꾸면 **파라미터가 하나도 안 붙는다**(에러 없이 조용히 기본값으로 뜬다).
08-02 에 노트북에서 실제로 밟은 함정이다.

확인:

```bash
ros2 topic hz /odometry/filtered            # EKF 출력이 흐르는가 (= 융합 중)
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic info /odom -v | grep -A1 "Endpoint type"   # 구독자 QoS 확인
```

★ `/odom`(입력)이 아니라 **`/odometry/filtered`(출력)** 를 보는 이유: 입력만 보면
EKF 가 죽어 있어도 통과한다. 출력이 흐르면 EKF 가 살아서 융합 중이라는 뜻이다.

⚠ EKF 가 조용하면 QoS 순서를 이렇게 본다 — **어제 `d0_check` 을 EKF 를 띄운 채로
돌렸다면** 검사 4·5 가 이미 그 조합을 봤으므로 QoS 는 뒤로 미룬다
(`JETSON_SETUP.md §7-a`. 08-02 검토 §29.3 이전 판은 EKF 없이 돌아 **아무 구독자도 못 봤다** —
그때의 "QoS 는 확인됨" 은 근거가 없었다).
그다음 `ekf_real.yaml` 머리말의 QoS 절을 읽고, `frame_id` 를 본다.

★★ **`frame_id` 3종은 아직 미확인이다** (`REAL_ROBOT_VALUES.md §4`):
`/odom` 의 `header.frame_id`·`child_frame_id`, `/imu/data` 의 `header.frame_id`.
**틀리면 오류 없이 조용히 실패한다** — TF 가 안 이어져 스캔이 지도에 안 붙는다.

```bash
timeout --kill-after=2s 10s ros2 topic echo /odom --field header.frame_id --once        # 기대: odom
timeout --kill-after=2s 10s ros2 topic echo /odom --field child_frame_id --once         # 기대: base_footprint
timeout --kill-after=2s 10s ros2 topic echo /imu/data --field header.frame_id --once    # 기대: imu_link
```

TODO(D+1): 확인 — 셋 중 하나라도 다르면 **구동부 펌웨어 쪽을 고쳐야 한다.**
우리 쪽에서 remap 으로 덮으면 나중에 두 곳이 어긋난 채로 굳는다. 결과를
`REAL_ROBOT_VALUES.md §4` 에 기록한다.

## 5. R3 rosbag — 녹화하고 **판정한다**

### 5-a. 무엇을 녹화하나

```bash
mkdir -p ~/r3_bags && cd ~/r3_bags
ros2 bag record /odom /imu/data /scan /tf /tf_static -o r3_$(date +%m%d_%H%M)
```

**녹화 대본** (한 판에 다 담는다 — 나중에 나눠 뜨면 조건이 달라진다):

| 구간 | 시간 | 무엇을 보려고 |
|---|---|---|
| 정지 | **60초** | 드리프트 — 안 움직이는데 EKF 가 움직인다고 하면 그게 드리프트다 |
| 손으로 앞뒤 굴리기 | 30초 | 속도 부호·스케일 |
| 손으로 좌우 회전 | 30초 | yaw 부호·스케일 (좌회전 `angular_velocity.z` 양수) |
| 정지 | 30초 | 다시 잠잠해지는가 |

⚠ **모터로 주행하지 않는다** — 오늘은 R3 다. 바퀴는 공중(R0 상태)에서 손으로 돌린다.

⚠ **`/tf` 를 녹화했으므로 재생할 때 주의**: bag 안의 `/tf` 를 그대로 재생하면 EKF 가
발행하는 `odom→base_footprint` 와 **충돌한다**(같은 변환을 둘이 쏜다 = 위치 널뜀).
R4 에서 재생할 때는 `--topics` 로 골라 재생하거나 EKF 의 `publish_tf` 를 끈다
(`config/ekf_real.yaml` 머리말에 같은 경고가 있다).

### 5-b. 판정 ① — 주기·간격 분포 (★ 이게 오늘의 핵심 숫자다)

```bash
cd ~/ros2_ws
python3 tools/bag_gap_report.py ~/r3_bags/r3_XXXX /odom /imu/data /scan
```

**왜 평균이 아니라 최대 간격인가**: EKF 입력의 계약과 `/scan` liveness 계약은
`tools/bag_gap_report.py`의 `TOPIC_POLICY`가 정한다. 입력 간격이 계약을 넘으면
그 토픽의 결측 또는 재개방 조건으로 분류되며, 평균만으로는 간헐적 끊김을 놓친다.

★ **bag 양끝도 간격으로 센다** (08-02 검토 §29.4). 구판은 각 토픽의 *첫~마지막 사이만* 봐서,
150초 bag 중 토픽이 10초만 살아 있어도 "전 구간 정상" 으로 통과했다. 이제 bag 시작→첫 수신,
마지막 수신→bag 종료를 간격에 포함하고, 초과하면 **위치(앞 공백/내부/뒤 공백)까지** 찍는다.

★★ **토픽마다 계약이 다르다** (08-03 검토 §30.2). EKF 입력과 `/scan`을 하나의 상수로
판정하면 정상 라이다를 IMU 결함으로 읽을 수 있으므로, 계약은 도구의 정책 표에서만 읽는다.

| 토픽 | 무엇을 보는가 | 한계 | 근거 |
|---|---|---|---|
| `/odom`·`/imu/data` | **EKF 입력 계약** | 도구 출력 참조 | `TOPIC_POLICY` |
| `/scan` | **liveness·stamp** | 도구 출력 참조 | `TOPIC_POLICY` |

★ 계약의 **단일 출처는 `tools/bag_gap_report.py` 의 `TOPIC_POLICY` 표 하나**다. 도구가
실행할 때 각 토픽의 계약과 근거를 한 줄로 출력하므로, 이 문서에 숫자를 복사하지 않는다.
정책을 바꾸면 도구 출력과 판정 회귀를 함께 갱신한다.

★ liveness의 양끝 처리와 허용 창의 근거도 `TOPIC_POLICY` 주석과 실행 출력에 남긴다.
bag의 시작·끝은 토픽 발행 위상과 맞지 않으므로, 도구는 그 정책에 따라 양끝 공백을 함께
판정한다.

| 결과 | 뜻 | 다음 행동 |
|---|---|---|
| 전 토픽 **자기 계약 안** | 현재 조건에서는 EKF 재료로 충분 | R4 로 진행 |
| `/odom`·`/imu/data` 가 **도구의 EKF 입력 계약 초과** | **IMU 주기 재개방 조건이 걸렸다** | `REAL_ROBOT_VALUES.md §1` 과 `src/tunnel_bringup/test/gate_fakes.py` 의 주기 정본을 **함께** 다시 판단 |
| `/scan` 이 **liveness 위반** | 재개방 조건이 **아니다** — 그 구간에 스캔이 없었다 | 라이다 드라이버가 늦게 떴는지·도중에 죽었는지 본다. 녹화를 다시 하면 된다 |

★ 구동부 회신의 "20~23ms" 를 우리가 상한으로 쓰지 않은 이유가 여기서 해소된다 —
회신에는 관측 창 크기·표본 수가 없었고, 이 bag 에는 있다.
⚠ 그래도 **이 판정은 이 녹화 구간에 대한 사실**이다. 더 길게·다른 부하에서 다시 볼 수 있다.
우리가 구동부의 짧은 창을 비판했으므로 우리 창의 한계도 같이 적어 둔다.

TODO(D+1): 확인 — 세 토픽의 계약 판정 결과와 bag 경로를 `REAL_ROBOT_VALUES.md §1`에 기록한다.

### 5-c. 판정 ② — `header.stamp` 단조성 (★ 위 명령이 함께 판정한다)

시간이 뒤로 가거나 멈추면 TF·EKF 가 통째로 무너진다. EKF 는 `dt<=0` 을 받으면 시간이
멈춘 것으로 보고 공분산이 발산한다.

★ **별도 명령이 필요 없다** — `bag_gap_report.py` 가 §5-b 에서 이미 검사한다.
08-02 검토 §29.4 이전 판은 **수신 시각만** 읽고 메시지 본문을 버려서 이 검사를
*원리상 할 수 없었다*(동일 stamp·역행 stamp 를 넣어도 녹색이었다). 이제 메시지를
역직렬화해 `header.stamp` 를 직접 본다.

- `✅ stamp 엄격 단조` — 통과
- `❌ stamp 중복 N건` — `dt=0`. 펌웨어가 같은 시각을 두 번 찍었다
- `❌ stamp 역행 N건` — 시각이 뒤로 갔다

⚠ **이건 우리 yaml 로 못 고친다 — 펌웨어(또는 Jetson 시계) 쪽이다.**
특히 `JETSON_SETUP.md §1-b` 의 NTP 되돌림이 정확히 이 증상을 만든다:
Jetson 시각이 뒤로 가면 Teensy stamp 가 **1ns 씩만** 증가해 `dt≈0` 이 된다.
→ 중복이 대량으로 나오면 **시계를 먼저 의심**하고, 그다음 펌웨어를 본다.

TODO(D+1): 확인 — 토픽별 중복·역행 건수와 NTP 상태를 `REAL_ROBOT_VALUES.md §1`에 기록한다.

### 5-d. 판정 ③ — covariance 가 '의미값'인가

```bash
timeout --kill-after=2s 10s ros2 topic echo /odom --field twist.covariance --once
timeout --kill-after=2s 10s ros2 topic echo /imu/data --field angular_velocity_covariance --once
```

**보는 법**: 전부 `0.0` 이면 "불확실성이 0" 이라는 뜻이 되어 EKF 가 그 값을 **절대 신뢰**한다
— 실제로는 "구동부가 안 채웠다"는 뜻일 가능성이 높다. 전부 `-1` 이면 "이 값 없음" 규약이다.

TODO(D+1): 확인 — 결과를 `REAL_ROBOT_VALUES.md §4` 에 기록하고, 0 으로 차 있으면
구동부에 확인한다. 이건 우리가 yaml 로 못 고친다(메시지 안의 값이다).

## 6. 오늘 끝나면

1. **bag 을 노트북으로 백업**한다. R4(EKF 단독 검증)는 이 bag 을 재생해서 한다 —
   로봇이 없어도 되는 단계라 노트북에서 돌릴 수 있다.
2. 아래를 **문서에 기록**한다 (다음 사람이 아니라 **일주일 뒤의 나**를 위해서다):
   - `REAL_ROBOT_VALUES.md §2` — 라이다 z 실측값
   - `REAL_ROBOT_VALUES.md §4` — 0.12m/s·우회전·횡편차 결과 · frame_id 3종 · covariance 실태
   - `REAL_ROBOT_VALUES.md §1` — 간격 분포 결과(재개방 조건이 걸렸는지)
   - `CURRENT_HANDOFF.md` — 다음 묶음
3. **D+0 결과를 다시 판정하지 말고 그대로 소비한다**: §0-a의 `JETSON_SETUP §7-c-0`
   실측 기록이 PASS면 `FREEZE_MANIFEST.md §6`의 조건부 수용을 확정하고, FAIL·측정 불능·빈칸이면
   즉시 재개방한 뒤 이 D+1 런북을 중단한다. 구동부 회신의 0.010/0.002/0.134초는 **실측 전
   참고값**일 뿐 확정 근거가 아니다. 소스상 새 명령 없는 자동 재가동은 차단돼 있지만 물리 확인은
   아래 TODO로 남는다.

TODO(D+1): 확인 — 재연결 뒤 새 `/cmd_vel` 없이 모터가 다시 돌지 않는지 결과를 기록한다.

## 7. `TODO(D+1)` 전량 목록 — **10건**

| # | 무엇 | 확인 방법 | 절 |
|---|---|---|---|
| 1 | 첫 직진 안전 공간 | 평지·양옆 1m 이상을 눈으로 확인 | §0-a · `JETSON_SETUP §7-c` |
| 2 | 0.12m/s 3m 실측 | `/odom.header.stamp` 두 값으로 평균속도 계산 | §0-a · `JETSON_SETUP §7-c-1` |
| 3 | 우회전 각속도 | `angular.z=-0.12` 10초와 `/imu/yaw_deg` 변화 | §0-a · `JETSON_SETUP §7-c-2` |
| 4 | 라이다 스캔면 높이 | 줄자 실측 → URDF `lidar_joint` | §2 |
| 5 | 라이다 시리얼 포트 | `ls /dev/ttyUSB*` | §2 |
| 6 | `frame_id` 3종 | `topic echo --field … --once` | §4 |
| 7 | 간격 분포(토픽별 `TOPIC_POLICY` 계약) | `tools/bag_gap_report.py` | §5-b |
| 8 | `header.stamp` 단조성 | `tools/bag_gap_report.py` 가 함께 판정 (중복·역행) | §5-c |
| 9 | covariance 실태 | `topic echo --field …covariance` | §5-d |
| 10 | 재연결 후 자동 재가동 금지 | R0 항목 — 구동부와 함께 | §6 |

## 근거 문서

`MASTER_PLAN.md §3` · `REAL_ROBOT_VALUES.md §1` · `REAL_ROBOT_VALUES.md §2` ·
`REAL_ROBOT_VALUES.md §4` · `JETSON_SETUP.md §5` · `JETSON_SETUP.md §7` ·
`FREEZE_MANIFEST.md §6` · `PITFALLS.md §7` · `TEST_GATES.md §2`
