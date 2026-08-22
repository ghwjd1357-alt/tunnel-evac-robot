# PROJECT_CONTEXT.md — 잘 바뀌지 않는 목적·경계·구조

> 정본. 현재 작업·완료 이력·테스트 수치는 여기 없음 (→ `CURRENT_HANDOFF.md` / 날짜별 현황).

## 1. 프로젝트 정의

자율주행과 멀티모달 센서 퓨전 AI를 활용한 **지하터널 전용 자동화 대피 유도 로봇** (2026.4~10).
GPS 불가 지하터널에서 재난(화재·연기·붕괴) 시, 로봇이 위치 파악 → 출동 → 대피자 집결 → 안전구역 선두 유도.

## 2. 시스템 구조 & 팀 경계

| 계층 | 담당 | 역할 |
|---|---|---|
| **Jetson Orin Nano** | **역할 A (손민우)** | SLAM · Nav2 · EKF · 미션 로직 · Perception Adapter · 관제 |
| **Teensy 4.1** | 구동부 팀 (3명) | 모터 PID · 엔코더 · micro-ROS |
| (카메라 인지) | 역할 B | YOLO 탐지 + depth 결합 → `/detections` |

- 역할 A → Teensy: `/cmd_vel` 발행 (**내 범위의 끝단**) / Teensy → 역할 A: `/odom`·`/imu/data` 구독.
- micro-ROS agent 실행은 역할 A 가 띄운다 (시점은 구동부 진행에 종속 — 후순위).

## 3. 버전 고정값 ★

| 계층 | 고정값 |
|---|---|
| OS | Ubuntu 22.04 LTS (노트북) / **JetPack 6.2.2** (Jetson, 22.04 기반) |
| 미들웨어 | **ROS2 Humble Hawksbill** (모든 노드·펌웨어·agent 통일) |
| 빌드 | colcon / `~/ros2_ws` / 항상 `--symlink-install` |
| SLAM / 내비 | slam_toolbox / Nav2 (**differential 모션모델**) |
| 위치 융합 | robot_localization EKF (odom + IMU) |
| MCU 통신 | micro-ROS (distro Humble 일치 필수) |
| LiDAR | sllidar_ros2 (RPLIDAR C1) |
| 카메라 | OrbbecSDK_ROS2 (Orbbec Gemini 2) |
| 시뮬 | Gazebo Classic 11 (`gz sim`/Ignition 자료와 문법 다름 — 주의) |

## 4. 토픽 입출력 계약

| 방향 | 토픽 | 타입 | 비고 |
|---|---|---|---|
| 입력 | `/scan` | LaserScan | RPLIDAR C1 |
| 입력 | `/odom` | Odometry | 구동부 Teensy (frame `odom`, child `base_footprint` 제안) |
| 입력 | `/imu/data` | Imu | 구동부 Teensy (BNO055 — **실측 46.4Hz** 08-02 정정. 분포·전제조건 = `REAL_ROBOT_VALUES.md §1`) |
| 입력 | `/detections` | 커스텀 (`tunnel_interfaces` 제안) | 역할 B. 아래 §4.1 |
| 출력 | `/cmd_vel` | Twist | Nav2 → Teensy (`linear.x`·`angular.z`만) |
| 출력 | `/map`, `/tf` | — | SLAM |

### 4.1 `/detections` V1 계약 (08-17 재합의 4왕복 반영 — schema·실패 표현·자세 판정 확정)

🔴 **상태가 바뀌었다.** 08-13 재합의 요청 → 역할 B 1·2·3차 회신 · 역할 A 2·3·4차 회신으로
**schema·실패 표현·class 열거·자세 판정까지 확정**됐다. 남은 것은 역할 B 회신 대기 4건과
실측 종속 항목뿐이다. 계약 정본 = `~/Desktop/YOLO_탐지연동_합의사항.md §15`
(구 `역할B_detection_토픽계약_전달.md` 는 대체됨).

- **책임 경계(변동 없음)**: YOLO(역할 B) 측이 depth 결합까지 담당해
  **camera-frame 3D position 제공**. 역할 A는 그것을 검증해 map 으로 옮긴다.
- **`.msg` 2종 확정** (`tunnel_interfaces`) — 07-19 골격 그대로다. 필드 추가 = 타입 변경 =
  양측 동시 리빌드이므로, 확장은 V2 별도 메시지로 한다.

```text
Detection3D      : string class_name / float32 confidence /
                   sensor_msgs/RegionOfInterest bbox / geometry_msgs/Point position
Detection3DArray : std_msgs/Header header / Detection3D[] detections
```

- **`class_name` = 닫힌 열거 5종 · 소문자 고정** — `person_fallen` · `person_ok` ·
  `person_unknown` · `fire` · `smoke`. header stamp = color 촬영시각, frame = camera optical
  frame. QoS = RELIABLE / VOLATILE / KEEP_LAST 5, 10Hz, 0.5초 stale.
- 🔴 **`car`·`human` 은 열거 검사 *이전에* drop 하고 `/diagnostics` 카운터로만 남긴다.**
  검사에 태우면 정상 상황(터널의 차·사람)이 매번 위반 경보가 되고, 그러면 사람이 경보를
  꺼서 **열거 검사 자체가 없는 것과 같아진다.**
- 🔴 **부분 실패 금지** — 인지 모델이 2개(Fire-Smoke + pose)여도 **한 프레임의 결과는
  하나**다. 한쪽만 성공한 프레임은 발행하지 않고 `/diagnostics` ERROR 로 남긴다.
  빈 배열(= 정상 미탐지)과 미발행(= 실패)을 섞지 않는다.
- 🔴 **중력 보정은 V1 계약에서 제외** — 자세 판정 기준축은 **이미지 y축**이다.
  근거·재개방 조건 = `PITFALLS §13`. 이 결정을 뒤집으려면 `ekf_real.yaml` 이 먼저 바뀐다.
- 자세 판정 규칙(관찰창 10프레임/3.0초 · `valid>=4` · 임계 60° · 히스테리시스 0.3/0.6)의
  정본 블록은 역할 B 3차 회신 부록이다. **Perception Adapter 는 그 블록을 그대로 읽어 만든다.**
- map 좌표 생성·검증(timestamp·frame·반복관측·오탐 억제)은 역할 A **Perception Adapter** 책임.
  수신은 **funnel 구조** (콜백 1개 → 내부 dict) — 필드 추가 시 콜백 한 곳만 수정.
- 🔶 **역할 A가 병목이다** — `tunnel_interfaces` 패키지 커밋·태그·40자 SHA 미전달이
  역할 B의 깡통 퍼블리셔 연결시험을 막고 있다 (`MASTER_PLAN §7` 예약 45).
  🟢 **08-19 노트북 빌드까지 완료** · 🔴 Jetson 확인·태그·전달은 남았다. 최신 상태 = **§4.2**.
- **역할 B 회신 대기 4건** (🔴 08-19 에 **6건**으로 늘었다 — §4.2 U1·U2) — T1 `confirmed` 상태 수명(`valid<4` 는 리셋 사유 아님) ·
  T2 지속 실패 시 `/diagnostics` STALE 승격 · T3 G3 실측표 정본화 ·
  T4 `human_dropped` 와 `person_*` 발행 수 동반 노출.

### 4.1-b 🆕 `/person_status`·`/victim` — 어댑터→미션 **내부** 계약 (2026-08-22 신설)

🔴 **`§4.1` 과 다른 물건이다.** `§4.1`(`/detections`)은 **역할 B 와의 계약**이라 바꾸면
양쪽이 같이 리빌드한다. 여기 둘은 **역할 A 안에서만** 쓰는 내부 배선이라, 우리가
혼자 바꿀 수 있고 역할 B 는 알 필요가 없다. 🔵 그래서 새 `.msg` 를 만들지 않았다 —
`tunnel_interfaces` 는 계약 전용 패키지이고 거기 손대면 양쪽 리빌드가 된다.

```
/person_status   std_msgs/String            10 Hz 상시   ok | fallen | unknown | none | stale
/victim          geometry_msgs/PoseStamped  fallen 확정 시 1회   map 좌표
```

QoS 는 **`/alarm` 과 같은 기본값**(RELIABLE·VOLATILE·KEEP_LAST 10)을 쓴다. 이 프로젝트는
BEST_EFFORT/RELIABLE 불일치로 이미 두 번 당했다(`PITFALLS §17`) — 같은 계열끼리는
같은 값으로 맞춘다.

#### 왜 두 개인가

미션이 필요로 하는 것이 **성격이 다른 둘**이다.

| | 쓰는 곳 | 성격 |
|---|---|---|
| `/person_status` | `SCAN_AREA` 의 분기(유도/신고/사람없음) | **상시 신호** — "지금 어떤 상태인가" |
| `/victim` | `RESCUE` 가 관제에 신고할 좌표 | **사건** — `/alarm` 과 같은 모양 |

`/alarm` 하나로 끝나는 화재와 달리, 사람은 *"아직 판정 중"* 과 *"사람이 없다"* 를
구분해야 한다. 사건만으로는 **"없다"가 영원히 안 온다.**

#### 🔴 디바운스는 **어댑터**가 한다 (미션이 아니다)

이미 화재가 그렇게 돼 있고(`confirm_frames 5` · `confirm_window_sec 3.0` ·
`confirm_assoc_radius_m 1.0`), 그 기계와 회귀가 검증돼 있다. 미션은 상태머신이지
신호처리기가 아니다. `부분 실패 금지`·`stale` 판정도 이미 어댑터 몫이다.

#### 판정 규칙 — 🔴 비대칭 방향이 직관과 반대다

| 오판 | 결과 |
|---|---|
| 쓰러졌는데 `ok` | 로봇이 유도를 시작하고 떠난다 → 🔴 **쓰러진 사람을 버린다** |
| 서 있는데 `fallen` | 관제에 신고가 뜬다 → 사람이 확인 → 시간 손실뿐 |

→ **`ok`(= 유도 시작) 가 비싼 판정이다.** 그러니 `fallen` 은 빠르게, `ok` 는 신중하게.

```
person_fallen  confirm_sec_fallen 연속  →  status = fallen  (+ /victim 1회)
person_ok      confirm_sec_ok     연속  →  status = ok
person_unknown                          →  🔴 둘 다 안 센다 → status = unknown
탐지 0건(빈 배열)                        →  status = none    (정상 미탐지)
/detections 가 person_stale_sec 동안 없음 →  status = stale  (🔴 판정 불가 ≠ 사람 없음)
유효 프레임 < person_min_frames          →  status = unknown
```

🔴 **`stale` 과 `none` 을 절대 섞지 않는다.** `none` 은 "봤는데 없다", `stale` 은
"못 봤다" 다. 미션은 `stale` 에서 **아무 분기도 하지 않고 정지 상태를 유지한다.**
이건 `§4.1` 의 *"빈 배열과 미발행을 섞지 않는다"* 를 우리 쪽에서 이어받은 것이다.

#### 🔴 판정 상수 — 역할 B 실측(08-22 인수인계 §8-a·§9)으로 확정

```
person_confirm_sec_fallen  1.5      person_min_frames      4   ← 6 에서 내림
person_confirm_sec_leave   4.0      person_min_confidence  0.50
person_stale_sec           1.5      ← 계약값 0.5 를 못 쓴다
min_confidence (fire)      0.60     ← 0.40 에서 올림
```

🔴 **`/detections` 는 계약 10 Hz 가 아니라 실측 3.8 Hz 다**(정지 상태 · 회전 중 미측정).
그러면 `fallen 1.5s` 창에 **5.7 프레임**만 들어온다 — 구값 `min_frames=6` 은 그것을
넘어 **확정을 영원히 막았다.** 쓰러진 사람을 보고도 신고가 안 나간다.

🔴 **`person_stale_sec` 는 계약값 0.5 를 쓸 수 없다.** 실측 프레임 간격이
min 0.000 ~ **max 0.729 s** 라 0.5 를 자주 넘고, 그러면 정상 동작 중에 `stale` 이 떠
판정이 계속 튕긴다. ⚠ **계약(§4.1)은 여전히 0.5 이고 구현이 그것을 못 지키는 것이다** —
우리 쪽에서 흡수했지만 계약 위반 사실은 남는다.

🔴 **fire 문턱을 0.40 → 0.60 으로 올렸다.** 역할 B §9: **불이 없는데 fire 가
0.45~0.58 로 뜬다**(원인 미특정). 0.40 이면 그대로 통과하고 3초 창에 11.4 프레임이
들어오므로 `confirm_frames 5` 도 쉽게 채운다 → **본편 원테이크 중 거짓 알람.**
⚠ 근거 있는 문턱이 아니라 **응급 조치**다 — 진짜 화재의 confidence 분포를 아직
아무도 안 쟀다. 🔵 그래도 올리는 쪽이 맞다: 거짓 알람은 테이크를 버리고, 놓친 자동
검출은 오퍼레이터가 **수동 `/alarm`** 으로 즉시 메운다. 되돌릴 수 있는 쪽을 고른다.
⏸ 되돌리는 조건 = 진짜 화재 confidence 실측.

🔵 **전부 런치 파라미터라 코드를 안 고치고 값만 바꿨다** — 08-22 새벽에 그렇게
설계한 이유가 이것이다. 회귀(`test_p17`~`p20`)가 **실측값과 상수의 관계**를 잠근다.

#### 🔴 어댑터 기동 인자 — 기본값으로 띄우면 TF 트리가 깨진다

역할 B 인수인계 §5: **Orbbec 드라이버가 이미 전체 TF 체인을 발행한다**
(`camera_link → … → camera_color_optical_frame`). 우리 기본값
(`camera_frame:=camera_color_optical_frame optical:=true`)으로 띄우면 그 프레임에
**부모가 둘**이 되어 트리가 깨진다.

```
ros2 launch perception_adapter adapter.launch.py \
    cam_x:=0.25 cam_z:=0.55 camera_frame:=camera_link optical:=false
```

⚠ `cam_x 0.25` · `cam_z 0.55` 는 **카메라 본체 기준 실측**이다. 브래킷 포함 최상단이
`base_link` 기준 0.779 m 미만인지는 **아직 아무도 안 쟀다**(예약 43 전제).

#### 🟢 08-22 구현 완료 — 무엇이 어디에 있나

| | 어디 | 키 |
|---|---|---|
| 발행 (어댑터) | `adapter_node._update_person` · `_person_tick`(10 Hz) | `person_confirm_sec_fallen 1.5` · `person_confirm_sec_leave 4.0` · `person_min_frames 6` · `person_min_confidence 0.50` · `person_stale_sec 0.5` |
| 소비 (미션) | `mission_node.person_verdict` — `GATHER` 분기 | `person_gate` **false** · `person_decide_timeout_sec 10.0` · `person_status_timeout_sec 1.0` |
| 훑기 | `SCAN_AREA` — 같은 (x,y) 에 yaw 만 돌린 Nav2 goal ×4 | `scan_steps 4` · `scan_dwell_sec 2.0` |
| 역행 병행 | 예약 61 — 라이다에 **OR 로만** | `search_back.camera_refind` **false** |
| 가짜 입력 | `fake_detections.py` 시나리오 6종 | `person_ok`·`person_fallen`·`person_none`·`person_unknown`·`person_flicker`·`person_far_fallen` |

🔵 **로봇도 카메라도 역할 B 도 없이 전구간이 굴러간다** — 가짜 탐지 → 어댑터 →
`/person_status` → 미션. 회귀 = adapter `test_person_path.py` · mission `test_person_gate.py`.

🔴 **미션도 자기 눈으로 신선도를 본다** (`person_status_timeout_sec`). 어댑터는 자기
입력이 끊기면 `stale` 을 말해 주지만, **어댑터 자체가 죽으면 아무 말도 못 한다.**
그때 마지막 `ok` 를 붙들면 미션은 빈 복도를 유도한다.

🔴 **두 게이트가 다 기본 꺼짐이다** — 본편 테이크(`camera:=false`)에는 어댑터가 없어
`/person_status` 가 한 건도 안 온다. 켜 두면 그 침묵이 `stale` 로 굳어 사람이 서
있는데도 `RESCUE` 로 빠진다. 증거는 회귀 숫자다 — 게이트를 더한 커밋에서 기존
**209 가 209 그대로**였다(`PITFALLS §19-③`).

#### 🔴 화재 경로는 손대지 않는다

어댑터는 검토 다섯 회차로 🧊 동결한 사슬이다(`AGENTS §6` 상한 초과). 사람 경로는
**순수 추가**이고, `/detections → /alarm` 은 한 줄도 안 바뀐다.
🔴 **"안 바뀌었다" 를 회귀로 잠근다** — 안 그러면 다음 사람이 그 말을 믿을 근거가 없다.
⚠ 이 추가는 **독립 검토를 안 받았다.** 동결을 푼 것이 아니라 새 변경을 얹은 것이다.

### 4.2 08-18~19 왕복 — 역할 B 실측·회신과 역할 A 5~7차 회신 (2026-08-19 신설 · 08-19 오후 갱신)

🔴 **원문 보관 위치가 생겼다.** 그전까지 역할 B 회신은 **원문이 한 건도 저장돼 있지 않았고**
`YOLO_탐지연동_합의사항.md` 요약본만 남아 있었다. 그래서 §4.1 이 P3-b(메모리 완화책)를
통째로 누락한 것을 08-19 에야 발견했다. **수신 원문 8건을 전량 보관한다.**

| # | 파일 (`~/Desktop/`) | 수령 |
|---|---|---|
| 1 | `YOLO_탐지연동_역할B_원안회신_0720.md` | 07-20 |
| 2 | `YOLO_탐지연동_역할B_1차회신_0817.md` | 08-17 |
| 3 | `YOLO_탐지연동_역할B_2차회신_0817.md` (N1~N11) | 08-17 |
| 4 | `YOLO_탐지연동_역할B_3차회신_0817.md` (P1~P5) | 08-17 |
| 5 | `YOLO_탐지연동_역할B_실측보고_0818오전.md` | 08-18 |
| 6 | `YOLO_탐지연동_역할B_추가보고_0818오후.md` | 08-18 |
| 7 | `YOLO_탐지연동_역할B_정정철회_0819.md` (M6 철회) | 08-19 |
| 8 | `YOLO_탐지연동_역할B_4차회신_0819오후.md` (V1~V4 · M7 실측) | 08-19 오후 |

역할 A 회신 3건 = `YOLO_탐지연동_역할A_5차회신_0819.md` ·
`YOLO_탐지연동_역할A_6차회신_0819.md`(M7 신설) ·
`YOLO_탐지연동_역할A_7차회신_0819오후.md`(W1 조건부 닫힘 · W2 시나리오 결정).

**이 왕복이 확정한 것**

| 항목 | 값 |
|---|---|
| color optical frame | **`camera_color_optical_frame`** (C1 실측 완료) |
| aligned depth 토픽 | **`/camera/depth/image_raw`** · `ALIGN_D2C_HW_MODE` 기본 활성 → **A8 충족** |
| Orbbec 기동 인자 | `enable_ir:=false` · 🔴 `depth_width`/`depth_height` **지정 금지**(프로파일 없음) |
| Fire-Smoke V1 모델 | **v9 확정** (v10 은 Recall −4.0%p · 실환경 오탐 4 vs 21 로 기각) |
| `angle_max` | **120° 이상 → `person_unknown` + WARN** · 카운터 `implausible_angle_frames`·`implausible_now` |
| N9 확정 지연 | 0.4초 → 0.20초 → 0.48초 → **≈1.12초**(08-19 · `min_valid 4 ÷ /detections 3.58Hz`). 🔴 **어느 값도 계약값이 아니다 · 정본 공란 유지** |
| 🔴 N9 분모 정정 | 앞 세 값은 **`color` 주기**를 분모로 썼는데 틀렸다 — 판정은 **추론 완료마다** 돈다. ⚠ 1.12초도 **하한**이다(`unknown` 이 섞이면 늘고, Jetson 동시 부하에서 커진다) |
| 🔴 **M7** `/detections` stale | 🟢 **조건부 닫힘**(08-19) — 사람 1명 조건 max **0.429초** < 0.5초. **재개방 3종** = ①Jetson 이관 후 ②TensorRT 전환 후 ③**촬영 길이(120초) 이상 연속 측정**에서 max ≥ 0.5초 |
| 🔴 M7 이 닫힌 **이유** | *"끊기지 않아서"* 가 아니다 — `color` 는 여전히 max **1.402초**이고, **추론이 병목(3.58Hz)이라 그 요철을 삼키고 있을 뿐**이다. 🔴 **성능 개선이 계약 위반을 만들어내는 구조** |
| 🔴 `fire` v9 오탐 | 실내 무화재에서 `fire_detected`·`person_near_fire` 발화 확인(08-19). **닫힌 항목이 아니다** — v11(9월)이 근본. 시연은 §4.2-a 로 우회 |
| B1 `person_fallen` | ✅ **08-19 확정 실측 성공** — `thr` 70 · `min_valid` 4 · `confirm_ratio` 0.6 · 3분기 전부 관측. **시연 인지 요구 충족** |
| `CRITICAL` 경로 | 🟢 C2-b 재배선 뒤 **처음 동작 확인**(08-19). `class_name == 'human'` 을 기다리다 영구히 죽어 있던 경로다 |
| P3-b 메모리 완화책 | ① **TensorRT 엔진 먼저 로드** ② `PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:128` ③ 기동 워밍업 1회 |
| P3-c 안전 논리 | 런타임 OOM → 미발행 + ERROR. **순손실은 가용성이지 안전성이 아니다** |

**역할 A가 내린 판정 2건 (5차 회신)**

- 🔴 **M2 중력 보정 — 재상정하지 않는다.** Q1 실측 `10° 초과 0.080%`(표본 31,330)로
  08-17 재상정 기준 미달. 전량 = `REAL_ROBOT_VALUES §1-i`. 역할 B §1 증거(각도 172~178°)는
  **중력이 아니라 카메라 장착 방향** 문제다 — 같은 표에서 정방향은 6~7° 로 정상이었다.
- 🔴 **M6 depth 실패 — ①도 ②도 아니다.** color/depth 동기화 실패는 *"개별 객체 depth
  실패"*(§15-c 3행)가 아니라 **"처리 실패"**(2행)이고, 계약은 그 경우 **미발행 + ERROR**
  라고 이미 적어 두었다. 빈 배열 발행은 *"빈 배열과 미발행을 섞지 않는다"* 위반이다.
  미발행으로 바꾸면 수신 측 0.5초 stale 이 자동으로 걸려, **역할 A가 `depth_failed` 를
  감시해야 성립하던 의존이 사라진다.**

🔴 **역할 A 미제출 3건** (둘은 신규)

| # | 항목 | 상태 |
|---|---|---|
| ~~**A10**~~ | `tunnel_interfaces` tag + 40자 SHA | ✅ **08-19 종결** — 양쪽 빌드·지문 일치·전달 완료. 역할 B도 지문 2건 일치 확인(`MASTER_PLAN §7` 예약 45) |
| **A11** | 🔴 **URDF 카메라 링크 + `base_link↔camera` static TF** | 🔴 **신규** — `robot_real.urdf` 에 카메라 링크가 **0개**라 역할 B의 E3 질문에 답할 수 없다. 카메라 물리 장착 시 |
| **A12** | Jetson 잔여 자원 측정 (YOLO 제외 스택) | 🔴 08-20 — 🔵 **M7 재개방 조건 ①과 같은 시험이다** |
| **A13** | 🔴 **Perception Adapter** | 🔴 **신규 · 미착수.** 역할 B가 깡통 퍼블리셔를 보냈는데 **받을 노드가 없다.** 지금 가능한 것은 `topic echo` 로 필드·QoS·stale 확인까지다. 착수 08-21 이후 |

### 4.2-a 🔵 08-19 사용자 결정 — **시연 시나리오에 화재를 넣는다**

`fire` v9 오탐이 실내에서 `CRITICAL`(`fire_detected`·`person_near_fire`)을 띄웠고,
2분 원테이크라 편집으로 못 가린다. 네 안 중 **⑤(신규)** 를 택했다.

| 안 | 판정 |
|---|---|
| ① 촬영 환경 조정(흰 반사면·강한 조명 제거) | 🟢 병행 |
| ② `fire_conf` 0.38→0.5 상향 | 🔴 **하지 않는다** — 계약값을 시연 편의로 바꾸는 것이고 **실화재 미탐 위험**이 생긴다 |
| ③ 관제 화면에서 fire 계열 숨김 | 🔴 **하지 않는다** — ②보다 낫지 않다. 판정은 살아 있는데 **사람이 못 보게** 하는 것이고 되돌리는 걸 잊기 쉽다 |
| ④ v11 재학습 | 9월 — 시연에 못 씀 |
| **⑤ 시나리오에 화재 포함** | 🔵 **채택** |

**근거** — 이 로봇의 정의가 `AGENTS.md §0` 의 *"재난 발생 시 … 대피 유도"* 다.
대본을 *"화재 발생 → 쓰러진 대피자 발견 → 안전구역 유도"* 로 두면 `fire_detected` 가
**시나리오상 정답**이 된다. 계약값 불변 · 정보 은폐 없음 · 실화재 미탐 위험 없음.

🔴 **⑤ 도 오탐을 고치는 것이 아니다** — 오탐이 화면에서 오탐으로 안 보이게 할 뿐이고
그 점에서 ③과 본질이 같다. 차이는 **③은 정보를 숨기고 ⑤는 정보를 맞게 만든다**는 것뿐이다.
그러므로 **`fire` v9 오탐은 닫힌 항목이 아니며 v11 이 근본이다.**

⚠ **역할 B 회신 대기 7건** — T1~T4(기존) · **W1-③** 재개방 조건 ③ 동의 여부 ·
**X1** `사람 있음` 조건에서 `/detections`·`color` **동시 120초** 측정 ·
**X2** 크래시 원인 가르기(overlay 중복 확인 + 종료 외 발생 여부).
🔴 **일곱 건 모두 시연을 막지 않는다.** ⚠ U1·U2 는 08-19 로 종결됐다.

### 4.2-b 🔴 `perception_node` 크래시 — 원인이 둘이고 하나는 우리가 경고한 그것이다

역할 B가 `Ctrl+C` 시점에 1회 관측했다(재현 미확인):
`rclpy/executors.py:400 _take_subscription → RuntimeError: Unable to convert call argument`.

| | 원인 | 성격 | 언제 |
|---|---|---|---|
| ⓐ | 종료 경합 — context 파괴 중 executor 가 take 시도 | 🟢 무해 | 종료 시에만 |
| ⓑ | 🔴 **구독 타입과 wire 타입 불일치** | 🔴 위험 | **운용 중에도** |

🔴 **ⓑ 를 의심하는 이유** — 역할 B 문서에 *"기존 `~/ros2_ws` **사본**과도 같아서"* 가 있다.
`PITFALLS §13` 이 금지한 **인터페이스 패키지 두 벌** 상태를 시사한다.
⚠ 다만 그 함정의 문서화된 증상은 *"런타임 에러가 아니라 값이 틀린 수신"* 이라 **이 예외와
정확히 맞지는 않는다** — 단정하지 않고 **X2 로 확인만 요청**했다.
가르는 법 = `ros2 pkg prefix tunnel_interfaces` + `AMENT_PREFIX_PATH` 전수 · 종료 외 발생 여부.

## 5. 하드웨어 확정값

- **DC 모터 ×4 + 강체 4륜 차동구동(differential) 확정** — 앞바퀴 조향 없음, 제자리 회전 가능.
  4륜 스키드 회전 슬립 → **IMU yaw 융합(EKF) 필수** (EKF는 Jetson 위 노드 = 역할 A).
- ★ 실측값 4종 수령 완료 (07-08~18, 정본 = **`docs/REAL_ROBOT_VALUES.md`** — 반영 지점·잔여 합의 포함):
  wheel_separation **0.49m** / 바퀴 지름 **0.13m** / footprint **0.55×0.57m**(외접반경 ≈0.40m) /
  encoder 26PPR → 바퀴축 TOTAL_PPR **2641.1** (08-02 확정 · quadrature ×4). IMU = BNO055 **실측 46.4Hz** (08-02 정정 — 구 41.63Hz·'50Hz 확정' 폐기).
  ⚠ 🔴 **08-13 재교정** — 바퀴 반지름은 **4종 분리**(공칭 0.065 / 실측 축높이 **0.053**(URDF) / odom 계수 **0.04603** / 제어 피드백 계수 0.05698) · 좌우 간격은 **3종**(물리 0.49(URDF) / 명령 0.62 / odom **0.670**) — `REAL_ROBOT_VALUES.md §1-b-1`·`§1-c`. 숫자를 베끼지 말고 `tools/firmware_constants.py` 로 `.ino` 에서 읽는다.
- 배선 핀맵·구동부 검증 절차 = `0718_구동부_배선맵핑_검증절차.md`. Teensy 계약 정본 = `~/Desktop/TEENSY_실차연동_최종합의서_0802.md` (07-24 합의사항을 대체).
- 잔여: `frame_id` 3종 확인 + 펌웨어 소스 + 물리 E-stop 장착 + 전원 인계 + 속도 검증 — `REAL_ROBOT_VALUES.md §4`.
- 코드 이식 트리거 = **구동부 "3m 직진 오차 3% 이내" 통과 선언.**

## 6. 아키텍처 구분 (동결 단위의 근거)

- **platform-core** (모든 시나리오 공통 메커니즘): Nav2 goal 사슬·속도 검증·scan watchdog·알람 신뢰경계·그래프 투영·설정 검증·지도 생명주기·테스트 인프라·관제.
  ★ **07-24 동결됨** — tag `platform-core-freeze-260724`. 이후 이 영역은 **실차 이슈 대응 외 변경 금지**이며, 변경이 필요하면 근거를 `docs/FREEZE_MANIFEST.md` 기준으로 남긴다.
- **mission policy** (시나리오 종속): 상태 전이 규칙·대기시간·SEARCH_BACK 정책·E2E 대본.
- **Perception Adapter** (역할 A 신설 예정): YOLO 원시 관측 → 검증 → FireEvent/PersonEvent → FSM 은 출처 무관 이벤트만 소비. 관제 수동 화재 지정도 별도 출처의 동일 이벤트 (영구 보존).
- **동결 3단**: ✅ `platform-core-freeze`(구조 분리 후 — **07-24 완료**, `212885a`, 증거 `docs/FREEZE_MANIFEST.md`) → `mission-logic-RC`(최종 시나리오 확정 후) → `mission-v1-freeze`(Orbbec·YOLO 실측 통합+통제환경 시험 후). 인지 주입만 통과한 상태를 freeze 라 부르지 않는다.
- **가상 시나리오 = 제품 사양이 아니라 검증 대본.** 최종 시나리오는 실차 R0~R8 경험 후 확정.

## 7. ★ 통찰 (왜 시뮬 먼저 하나)

Gazebo 플러그인의 가짜 `/scan`·`/odom`·`/imu`는 실물과 **타입·형식 동일** → 시뮬로 짠 SLAM·Nav2·미션 코드를 실물에 거의 그대로 이식 (동일 소스·동일 인터페이스, 단 Jetson 은 aarch64 별도 빌드). 실물 라이다 단독으론 odom 이 없어 SLAM 불가 → 구동부 odom 수령 전까지 메인 트랙 = 시뮬.

## 8. 문서 / 파일 위치 맵

**repo 정본 (`~/ros2_ws/`)**: `AGENTS.md`(공통 규칙) · `CLAUDE.md`(Claude 진입점) ·
`docs/{PROJECT_CONTEXT, MASTER_PLAN, CURRENT_HANDOFF, TEST_GATES, PITFALLS, REAL_ROBOT_VALUES}.md` ·
`docs/AI_CONTEXT.md`(AI 입력 라우팅·품질/토큰 측정 계약) ·
**`docs/FREEZE_MANIFEST.md`**(동결 증거 — 해시·게이트 수치·조건부 위험·검토 결과. 07-24 신설) ·
**`docs/ELECTRICAL_BASELINE.md`**(전기계통 현황·시험환경 최소 안전선·E-stop 재설계 근거. 08-04 신설) ·
**`docs/FIRMWARE_REBUILD.md`**(Teensy 펌웨어 재빌드 환경·platform.txt 패치·재현성 기준점. 08-05 신설) ·
`docs/legacy/CLAUDE_pre-restructure_0720.md`(개편 전 CLAUDE.md 백업, git `8042464`와 동일).

**Desktop 역사·근거 (`~/Desktop/개발현황/`)**: 날짜별 `06xx/07xx_현황.md`(상세 학습기록) ·
`0719_현황.md`(외부 검토 교차검증 정본 — 07-19 시뮬 마감·G 묶음까지, §18.2 에서 끝) ·
**`0720_현황.md`**(SpeedManager 전용 서사 — §18.3 인계 / §19 구현 / §20 Codex 검토·P1 보완.
07-20 에 0719 에서 이관, **절 번호 보존**) · `CODEX 현황/0719검토현황.md`(Codex 원문, §14=최종 승인) ·
`CODEX 현황/0720검토현황.md`(SpeedManager 추출 독립 검토 — P1 1건, GoalManager HOLD) ·
`CODEX 현황/0723검토현황.md`(GoalManager 2/3 = §7 통과 / E2E 하네스 3/3 = §8 통과 — 구조 분리 종결 /
**§9 = platform-core-freeze 동결 판정 통과, P0·P1·P2 0건**) ·
`0719_실차전환_마스터플랜.md`(계획 근거·개정 이력 — 실행 정본은 repo MASTER_PLAN) ·
`0705_실차전_전략.md` · `0707_로드맵_통합계획.md`(구판, 마스터플랜이 개정) · `0718_관제시스템.md`(관제 설계 정본) ·
`0718_구동부_배선맵핑_검증절차.md` · `실차값_수령체크리스트.md`(구판, `docs/REAL_ROBOT_VALUES.md` 로 이관) ·
`학습로드맵.md`+`학습진도.md`(학습 트랙 — 이 폴더에서 세션을 열면 학습 튜터용 `CLAUDE.md` 가 로드된다) ·
`역할B_detection_토픽계약_전달.md`(구판, 대체됨) · `0720_AI공통개발환경_개편_Claude확인요청.md`(개편안+검토, 완료 후 삭제 예정) ·
**`0720_AI공동개발환경_사용설명서.md`**(이 문서 체계를 쓰는 법 — 구조·동기화·자동화 3층·AI 플로우).

### 8-b. 🔴 핸드오프는 **한 묶음만** 가리킨다 (08-13 신설 — 검토 §65.4)

`CURRENT_HANDOFF.md` 안에서 **현재 단계 · 본문 목표 · 진행표 · 완료조건 · 완료 판정**은
전부 **같은 하나의 묶음**을 가리켜야 한다. 08-13 에 이것이 깨졌다: 본문 목표는 예약 32-d
(펌웨어)인데 진행표와 완료조건은 예약 32-b(전기계통)를 지시했다. 다음 세션이 **어느 것을
실행할지 갈린다.**

🔴 **`doc_check --strict` 와 `handoff_single_check` 는 이 모순에 OK 를 냈다.** 그 도구들은
꼬리 링크 형식과 절 존재를 보지 **의미의 일관성을 못 본다**(예약 23 에 이미 기록된 검사 상한).
초록을 반증으로 쓰지 않는다 — **사람이 다섯 자리를 눈으로 맞춘다.**

⚠ 묶음이 끝나면 그 서사를 **정본으로 보내고 핸드오프에는 링크만 남긴다.** 상한(`20,000 B`)에
걸리면 **상한을 올리지 말고 옮길 완료 서사를 먼저 찾는다** — 그것이 상한의 목적이다.

### 8-c. 🔴 문서의 개수 주장은 기계가 세는 값과 같아야 한다 (08-13 밤 · 검토 §69.4)

`TEST_GATES` 가 알려진 P0/P1 을 `114건` 이라 쓰고 실행 정본은 `117` 이어도 `doc_check
--strict` 가 통과했다. 같은 회차에 `CURRENT_HANDOFF` 는 완료조건에 **배선 6행**, 완료
판정에 **13/13** 을 적어 **한 문서 안에서 안전 게이트 계약이 갈렸다** — 물리 굽기 직전에.

🔴 **사람이 손으로 맞추는 숫자는 반드시 갈린다.** 그래서 둘 다 기계 대조를 붙였다:
- 알려진 P0/P1 수 ↔ `tools/ai_known_p0_p1.json` 항목 수 (문서 주장 **전수** 대조)
- `§7-c-E` 행 수 단일성 — 완료조건과 완료 판정이 같은 계약을 말하는가
  ⚠ 역사 서술(`08-12 에 닫은 것 — §7-c-E 13/13`)은 오탐이 아니므로 **현재 묶음 계약을
  적는 두 절만** 본다.

🔴 **재개방 조건에는 관측자와 시점을 함께 적는다** (08-14 · 검토 §71.4).
*"한 번이라도 또 갈리면"* 처럼 **누가 언제 보는지가 없는 재개방 조건은 발동하지 않는다** —
조건부 수용이 형식만 남고 실제로는 영구 면제가 된다. 관측 지점은 셋 중 하나 이상이어야
한다: ① 매 커밋 도는 기계 검사 ② 세션 시작 필수 읽기(`CLAUDE.md §0`) ③ 현장 절차서.

⚠ 이 규율은 개수에만 있는 것이 아니다 — **보완 뒤에도 정본이 보완 전 주장을 유지하면
그 보완은 인계되지 않는다.** 도구를 고쳤으면 가장 먼저 읽는 문서를 같이 고친다.

### 8-a. 검토 서사 파일 규약 (08-07 신설 — `AGENTS.md §5` 가 여기를 가리킨다)

검토 결과는 `CODEX 현황/*검토현황.md` 에 **절을 이어서** 쓴다. **절 번호는 리셋하지 않는다** —
`0719`§1 부터 지금까지 한 줄기이고, 인용(`§33`, `§50`)이 그 번호에 걸려 있다.

🔴 **08-07 개정 — "새 파일 금지"를 철회했다.** 그 규칙은 서사를 잇게 하려던 것이었는데,
`CODEX 현황/0801검토현황.md` 가 **331KB · 4,335줄 · 27절**이 되어 열기도 검색하기도 어려워졌다. 서사를
잇는 것은 한 파일이 아니라 **색인**이다. 새 규약:

- 파일이 커지면 **절 경계에서** 나눈다. 본문은 한 글자도 바꾸지 않는다(이동이지 편집이 아니다).
- 조각 파일명은 **앞 4자를 원본과 같게**(`0801…`) 하고 **`검토현황.md` 로 끝낸다.**
  🔴 이건 취향이 아니라 계약이다 — `tools/test_ai_context.py` 의 `review_history_findings()`
  가 `*검토현황.md` 로 훑고 `Counter(name[:4])` 를 `ai_known_p0_p1.json` 의 id 접두사와 대조한다.
- **각 파일 머리에 형제 색인**을 둔다(어느 절이 어디 있는지 + 앞 서사 포인터).
- 저장소가 `§N` 을 인용하던 자리를 **그 절이 실제로 있는 파일로 옮긴다** —
  `doc_check.sh` 가 대상 문서에 그 절이 있는지 본다.

실적: `CODEX 현황/0801검토현황.md` 331KB → `0801`(§26~§33) · `0801-2`(§34~§38) · `0801-3`(§39~§43) ·
`0801-4`(§39-R·§44~§50) 4장. 본문 189,208자 **바이트 동일** 검증 후 분할했다.

**Desktop 팀 공유물 (`~/Desktop/`)**: `미션노드_개발현황_및_실차이후계획.md` · `TEENSY_실차연동_합의사항.md` · `YOLO_탐지연동_합의사항.md`.

**`firmware/`**(Teensy 펌웨어 작업본 — vendor drop + 역할 A 수정. 소유 경계·수정 규칙 = `firmware/VENDOR_DROP.md`, 빌드 환경 = `docs/FIRMWARE_REBUILD.md`. 08-05 신설. ⚠ `src/**` 동결과 무관한 별개 최상위 폴더) ·

**기타**: `~/setup-tasks/`(GPU 복구 브리프·스피커 SOF 로그). `~/ros2_ws/console/`(관제 웹 — ROS 패키지 아님).
패키지: `tunnel_sim`(시뮬 자산) / `mission_manager`(미션 로직) / `perception_adapter`(08-21 신설 — `/detections`→`/alarm`. **기존 코드 무변경 별도 노드**) / `my_first_pkg`(학습용).
