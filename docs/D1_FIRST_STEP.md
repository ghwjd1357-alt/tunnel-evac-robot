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
| R0 watchdog | 명령 단절 뒤 실제 바퀴 정지 ≤0.5초. EMA twist가 아니라 60fps 영상+raw pose로 판정 | `TODO(D+0): 확인` — fps·프레임·초·bag·PASS/FAIL |
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
ros2 topic echo /odom --field header.frame_id --once        # 기대: odom
ros2 topic echo /odom --field child_frame_id --once         # 기대: base_footprint
ros2 topic echo /imu/data --field header.frame_id --once    # 기대: imu_link
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
ros2 topic echo /odom --field twist.covariance --once
ros2 topic echo /imu/data --field angular_velocity_covariance --once
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
