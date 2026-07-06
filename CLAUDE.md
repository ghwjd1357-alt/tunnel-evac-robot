# CLAUDE.md — 한이음 지하터널 대피로봇 / 역할 A (손민우)

> Claude Code가 매 세션 자동으로 읽는 프로젝트 규칙서. 위치: `~/ros2_ws/CLAUDE.md`
> **이 파일 = 규칙 + 현재상태만.** 그날그날 상세 학습기록은 `~/Desktop/개발현황/`의 날짜별 현황.md 참조 (§8 위치 맵).
> **Phase 번호는 이 파일이 정본.** 마스터 컨텍스트는 방향 점검 시 동기화하는 상위 문서.

---

## 0. 작업 규칙 (매 세션 적용)

- **기술 스택:** ROS2 + C++ / Python.
- **눈높이:** 나는 ROS·프로그래밍 **초보자** (파이썬·XML 모두 첫 입문). 코드만 던지지 말고, 원리·구조·핵심 명령어를 **초보자 눈높이로 상세히** 설명할 것.
- **호환성 최우선 ★:** 모든 코드·패키지는 **Ubuntu 22.04 + ROS2 Humble** 호환을 먼저 확인.
- **종속성 명시:** 새 패키지 제안 시 의존성 + 설치 명령어를 항상 함께 제시.
- **드라이버 경고:** 센서·드라이버 작업 시 호환 문제를 미리 경고하고 대안 제시.
- **단계별 진행:** 빌드 → 테스트 → 디버그 순으로 쪼개어 하나씩 검증. 에러 로그 입력 시 원인·해결책을 분류해 설명.
- **모듈화 + 주석:** 모든 코드는 기능별 모듈화 + 상세 주석.

---

## 1. 프로젝트 한 줄 정의

자율주행과 멀티모달 센서 퓨전 AI를 활용한 **지하터널 전용 자동화 대피 유도 로봇.**
GPS 불가 지하터널에서 재난(화재·연기·붕괴) 시, 로봇이 위치 파악 → 출동 → 대피자 집결 → 안전구역 선두 유도. (2026.4 ~ 2026.10)

---

## 2. 시스템 구조 & 팀 경계

| 계층 | 담당 | 역할 |
|---|---|---|
| **Jetson Orin Nano** | **나 (역할 A)** | SLAM · Nav2 · YOLO · 관제 |
| **Teensy 4.1** | 구동부 팀 (3명) | 모터 PID · 엔코더 · micro-ROS |

- 나 → Teensy : `/cmd_vel` 발행 **(내 범위의 끝단)**
- Teensy → 나 : `/odom`, `/imu/data` 구독만

---

## 3. 버전 고정값 ★

| 계층 | 고정값 |
|---|---|
| OS | Ubuntu 22.04 LTS (노트북) / **JetPack 6.2.2** (Jetson, 22.04 기반) |
| 미들웨어 | **ROS2 Humble Hawksbill** (모든 노드 통일) |
| 빌드 | colcon / 워크스페이스 `~/ros2_ws` / 항상 `--symlink-install` |
| SLAM / 내비 | slam_toolbox / Nav2 (**differential 모션모델**) |
| 위치 융합 | robot_localization EKF (odom + IMU) |
| MCU 통신 | micro-ROS (**agent · 펌웨어 distro 반드시 Humble 일치**) |
| LiDAR | sllidar_ros2 (RPLIDAR C1) |
| 카메라 | OrbbecSDK_ROS2 (Orbbec Gemini 2) |
| 시뮬 | Gazebo Classic 11 |

---

## 4. 토픽 입출력 계약

| 방향 | 토픽 | 타입 | 출처 |
|---|---|---|---|
| 입력 | `/scan` | LaserScan | RPLIDAR C1 |
| 입력 | `/odom` | Odometry | 구동부 Teensy |
| 입력 | `/imu/data` | Imu | 구동부 Teensy |
| 입력 | `/detections` | 커스텀 (역할 B와 합의) | 역할 B (YOLO). ★ 계약 시 "필드 **추가** 가능(기존 필드 변경 금지)" 원칙 선합의 — 후보: `mobility`(관절추정 거동판별). 내 쪽은 **funnel 구조**(콜백 1개→내부 dict, `mobility:'unknown'` 자리 예약)로 수신 (→ 0705_실차전_전략.md §1) |
| 출력 | `/cmd_vel` | Twist | Nav2 → Teensy |
| 출력 | `/map`, `/tf` | — | SLAM |

micro-ROS 에이전트 실행 (브리지 1줄) 은 내가 띄운다. (단 시점은 후순위 — §6)

---

## 5. 하드웨어 핵심 (구동방식 확정 ✅)

- **DC 모터 ×4** (BLDC 아님), 10A 단순 DC 드라이버 (VESC 제거).
- **강체 4륜 + 차동구동(differential) 확정** — **앞바퀴 조향 없음.** 좌/우 2개씩 묶어 속도차로만 방향 전환, 제자리 회전 가능.
- → Nav2 모션모델 = **differential**, URDF = 차동 4륜, 내 발행은 `/cmd_vel`의 `linear.x`·`angular.z`까지.
- 4륜이라 회전 시 슬립 발생 → **IMU yaw 융합(robot_localization EKF)으로 보정 필수.** (EKF는 Jetson 위 노드 = 역할 A 담당)

---

## 6. 현재 진행 상태 (2026-07-06 갱신 — ★ 미션 로직 ①~③ 전체 완료, 시나리오 상태머신 시뮬 검증 ✅)

### 상태 한눈에
| 항목 | 상태 |
|---|---|
| 노트북 Ubuntu 22.04 듀얼부팅 (커널 6.17.12) | ✅ |
| 노트북 NVIDIA 580.159.03-open (Production) | ✅ 복구완료 (상세: `~/setup-tasks` 브리프) |
| Jetson JetPack 6.2.2 (NVMe) + 무선 SSH + VSCode Remote | ✅ |
| ROS2 Humble (노트북 + Jetson, talker 검증) | ✅ |
| RPLIDAR C1 sllidar_ros2 (노트북 + Jetson, /scan 검증) | ✅ 2026-06-23 (상세: `0623_현황.md`) |
| URDF 로봇(differential) + Gazebo spawn + 운전 | ✅ 2026-06-26 (2D, 상세: `0626_현황.md`) |
| slam_toolbox + Nav2 + robot_localization (노트북 apt) | ✅ 2026-06-26 설치 |
| SLAM 터널 지도 생성·저장 (`maps/tunnel_map.*`) | ✅ 2026-06-26 (2E, 헤드리스 자동주행) |
| Nav2 자율주행 | ✅ **완전검증 (2026-07-05):** EKF+SLAM튜닝 후 12m 목표 SUCCEEDED, **실제 위치 목표 0.24m 이내** (이전 ~9m 어긋남 해소) |
| **미션 로직 (`mission_manager`)** | ✅ **①~③ 완료 (2026-07-06):** PATROL→APPROACH→GATHER→GUIDE⇄SEARCH_BACK→ESCAPED+FAULT 전체 시뮬 E2E 검증. 후방 추종감지(라이다)·놓침 역행·안전장치 2종·GUIDE 저속·가짜 추종자 포함 (상세: 0705_현황.md §12~14) |
| EKF (robot_localization, odom+IMU 융합) | ✅ 2026-07-05: 시뮬 IMU 추가 + `config/ekf.yaml` + slam 파라미터 튜닝. 12.5m 주행 시 SLAM 위치오차 **9m → 0.17m** (상세: `0705_현황.md` §7) |
| Orbbec 카메라 (OrbbecSDK_ROS2) | ⬜ **의도적 보류** (시뮬 트랙엔 불필요, 역할 B·실차 단계에서) |
| micro-ROS agent | ⏸ **후순위** — 구동부 Teensy 진행에 맞춰 별도 시점 진행 |
| JetPack SDK (CUDA/cuDNN/TensorRT) | ⏸ 추후 `sudo apt install nvidia-jetpack` |

### 로드맵 & 현재 위치
**2A** 클코 학습 ✅ → **2B** ROS2 기초 ✅ → **2C Gazebo ✅** → **2D URDF(differential) ✅** → **2E SLAM+Nav2+EKF ✅** → **2F 미션 로직 ①뼈대 ✅ ②GATHER/GUIDE ✅ ③후방감지+SEARCH_BACK ✅** (2026-07-06: 시나리오 전체 상태머신 시뮬 E2E — 실패·성공 경로 모두 실증) ◀ 현재 = 다음 선정 (후보: 집결지 계산 모듈 / 역할 B `.msg` 계약 / 중간보고서 / micro-ROS)
- **micro-ROS = 후순위** (구동부 진행에 맞춰, 위 로드맵과 별개 시점).
- ★ **메인 트랙은 Gazebo 시뮬.** 실물 라이다는 `/scan`만 나오고 `/odom`(움직임)이 없어 단독 SLAM 불가 → odom은 구동부가 줄 때까지 **시뮬로 SLAM·Nav2 개발.**

### 2D URDF + 2E SLAM 진척 (2026-06-26, 헤드리스 무인 작업 — 상세: `0626_현황.md`)
- ✅ **2D URDF 로봇 완성** — `urdf/robot.urdf`. base_footprint→base_link→바퀴/라이다 TF, diff_drive(`/cmd_vel`→이동, `/odom`+TF 발행), 라이다(`/scan`). 확정치수 반영(몸통 0.5×0.4×0.6, 라이다 지면 0.60m). world의 static box_robot 제거 → `robot.launch.py`가 spawn.
- ✅ **★ 시뮬 구동모델 = 2 구동휠+앞뒤 캐스터** (실물은 4륜). **4륜 강구동은 Gazebo에서 과구속(over-constrained)으로 제자리회전 불가** → 헤드리스 실측 끝에 2휠+캐스터(이상적 differential)로 전환. 토픽·TF·footprint 동일하므로 SLAM/Nav2 코드는 실물 4륜에 그대로 이식. 회전 0.5rad/s·전진 0.3m/s 실측 검증.
- ✅ **2E SLAM 지도 생성·저장** — `slam_toolbox`(async) + 자동주행으로 T자 터널 전체 매핑. `maps/tunnel_map.pgm/.yaml`(occupancy) + `maps/tunnel_posegraph.*`(localization용). RViz 없이 헤드리스로 검증.
- 🔶 **2E Nav2 부분검증 (2026-06-27)** — `config/nav2_params.yaml`(RPP 컨트롤러·rolling global costmap) + `launch/slam_nav2.launch.py`(라이브SLAM+Nav2). 목표 전송 → 전역경로+RPP로 **13m 자율주행 실증.** 단 **긴 복도 SLAM 길이방향 드리프트(corridor problem)로 목표 정밀도달 미흡** (믿는위치 vs 실제 ~9m 어긋남). 상세·디버깅: `0626_현황.md` §6.5.
- ✅ **EKF 완료 (2026-07-05, 상세: `0705_현황.md` §7)** — ① URDF에 imu_link+IMU 플러그인(`/imu/data` 50Hz) ② diff_drive `publish_odom_tf` **false**(TF는 EKF 단독 발행) ③ `config/ekf.yaml`(odom0=vx·vy·vyaw 속도융합, imu0=yaw·vyaw) ④ ekf_node는 `robot.launch.py`에 상주(모든 상위 런치 TF 보장). **EKF 융합 odom 자체는 오차 ~0 실증.**
- ✅ **★ SLAM 튜닝이 마무리 열쇠였음** — EKF만으론 부족: slam_toolbox가 정확한 odom을 받고도 복도에서 scan-match("안 움직였다" 피크)로 덮어씀. `slam_params.yaml`에 **`distance_variance_penalty: 0.02`**(⚠ 분모라 **작을수록** odom 신뢰↑, 반대방향 함정), `correlation_search_space_dimension: 0.2`, `minimum_distance_penalty: 0.05` → 12.5m 주행 시 위치오차 **9m → 0.17m**, Nav2 12m 목표 **실제 0.24m 이내 도달(SUCCEEDED)**. 전략판단("EKF 먼저→재평가→SLAM 튜닝")이 정확히 그 순서로 실현됨.
- ✅ **회귀진단 완료 + 3목표 전부 통과 (07-05 심야, 상세: 0705_현황.md §8)** — 발견·수정 4건: ① SLAM 회전탈출(튜닝 부작용) ② 라이다 자기타격 ③ 안 보이는 콘 ④ **★ 벽 관통 경로**(진범: obstacle_layer만+allow_unknown:true → planner가 옅은 벽 뚫고 터널 밖으로 경로. `/plan` 덤프로 검거 → **static_layer 복귀+allow_unknown:false**로 해결). **최종: 분기입구→곁복도끝→동쪽끝 3목표 SUCCEEDED, 오차 0.28m/believed 0.01m. 지도 v3 교과서급.** 변경: slam_params·robot.urdf·nav2_params·tunnel.world.
- ✅ **오돔 오차 주입 테스트 완료 (07-05 3차, 상세: 0705_현황.md §10)** — 실차 리허설 성공. ① **★ 대발견: diff_drive `odometry_source` 기본=world(치트 오돔 — Gazebo 실위치 복사)** = 시뮬 odom이 늘 완벽했던 진짜 이유·/odom에 spawn좌표 포함되던 이유. `encoder`로 바꿔야 실차 방식(바퀴 적분, (0,0) 시작) ② 신설: `urdf/robot_odomerr.urdf`(거리 +5%·yaw −9% 거짓 주입) + `robot.launch.py urdf:=`·`slam.launch.py slam_params:=` 인자 ③ 실측: **회전 28% 거짓 → EKF(IMU)가 완벽 교정(0.001rad)** / 거리 거짓은 EKF 못 잡음(절대기준 없음) → SLAM 몫 ④ **penalty 0.02 실차 부적합 실증**(+5% 거짓을 통째 흡수: 12.9m에 +0.62m) → **실차 시작값 = `config/slam_params_realodom.yaml`**(0.1/0.1/0.3): 거짓 odom 0.156m·정상 odom 0.113m 양쪽 건강. 시뮬 정본(0.02)은 Nav2 완전검증치라 교체 안 함.
- ✅ **실차 전 전략 확정 (07-05, 정본: `0705_실차전_전략.md`)** — 가짜 detection(품질 흉내) 금지 / 역할 B와 `.msg` 계약(위치=로봇기준 3D 권장)만 선합의 + 상태머신 트리거용 깡통 퍼블리셔는 OK(단위테스트). 실차 순서: micro-ROS 브리지 → teleop+odom 검증(★최대 관문: 3m 직진·1바퀴 실측) → EKF·SLAM 재튜닝 → SLAM 지도 → Nav2(3목표 회귀 절차 재사용).
- ✅ **미션 로직 ①뼈대 완성 + E2E 통과 (07-05 4차, 상세: 0705_현황.md §12)** — ① 팀 시나리오 **잠정** 합의(⚠ **확정 아님 — 유동적.** 변경 후보: 카메라 관절추정으로 구조자 거동가능 판별 → 거동불능자 분기 로직 등. 뼈대/살 분리가 그 대비책)(순찰→화재→싸이렌+집결→T초→선행유도→라이다 후방추종확인→놓치면 역행→탈출): 상태도 PATROL→APPROACH→GATHER→GUIDE⇄SEARCH_BACK→ESCAPED. 집결완료=시간기반, 후방감지=라이다 상시(360°라 회전 불필요, 카메라는 놓침확인 1회만), SEARCH_BACK엔 재시도 제한+화재 안전하한 필수 ② **신설 패키지 `mission_manager`**(rclpy 상태머신, 실차 이식 대상): waypoints.yaml 분리·/mission_state·/siren·FAULT 자동재시도 2회. 실행 `ros2 run mission_manager mission_node --ros-args -p use_sim_time:=true` ③ **E2E 실측**: 순찰 3지점→알람→goal 취소·전환→집결(싸이렌 ON 실측)→8초→탈출구 0.28m 도달, 빈 경로 0회 ④ **★ Nav2 근본결함 해결**: rolling global costmap+static layer가 라이브 SLAM 리사이즈와 충돌해 간헐 '성공+빈경로(0 poses)'→ABORTED (이동 중에만, §8이 통과했던 건 운) → **`rolling_window: false`**(nav2_params, SLAM+Nav2 표준구성)로 종결.
- ✅ **②단계 완료 (07-05 5차, 상세: 0705_현황.md §13)** — ① 상태 정식 승격: PATROL→APPROACH→GATHER→GUIDE→ESCAPED(+FAULT), Sub 폐지 ② **GUIDE 저속**: /controller_server/set_parameters 비동기 호출로 0.26→0.12 동적 변경(cmd_vel 0.12 실측)·ESCAPED 복원 ③ **가짜 추종자**: `ros2 run tunnel_sim fake_follower` — tunnel.world 에 libgazebo_ros_state.so 추가, 원기둥이 로봇 뒤 1.2m 추적(걸음 0.8m/s), **/follower_cmd "stop"=놓침 재현 버튼** ④ E2E: 알람→집결→8초→저속유도→**추종자와 같이 탈출**(robot map 0.27 / follower 1.44), APPROACH 전환 레이스 FAULT 를 자동재시도가 흡수 ⑤ 발견: transform_tolerance 기본 0.1s 가 부하 시 간헐 abort → RPP 에 0.5 명시 / 추종자가 라이다에 찍혀 SLAM 오차 0.17→0.56m 열화(=현실적, ③단계 배경제거 재료). funnel 원칙 확정(§12.5) + on_alarm funnel 수정 + 낡은 rolling 함정 폐기 표기.
- ✅ **③단계 완료 (07-06, 상세: 0705_현황.md §14)** — ① **`follower_monitor.py`**: 후방 부채꼴(±60°·2.5m 문턱=폭6m 터널 기하 근거) + **디바운스 비대칭**(놓침 3초/재발견 1초) + **이중 구역**(rear=GUIDE용 / any=SEARCH_BACK 재발견용 — 역행 중엔 사람이 '앞'이라 후방만 보면 영원히 못 찾는 설계 구멍 수정). visible()/lost() 두 답만 노출 = 교체가능 모듈 ② **SEARCH_BACK**: 마지막 목격 지점(TF 상시 기록) 역행 + **안전장치① 시도 2회 제한**(소진→관제 보고+단독 탈출, run1 실증) + **안전장치② 화재 안전하한 5m 클램프**(수식 검산 통과, 실발동은 추후) + 재발견 시 rear 타이머 리셋(즉시 재-놓침 방지) ③ **E2E 실증**: 실패 경로(놓침→역행→소진→보고+단독 탈출) + 성공 경로(놓침→역행→**재발견→GUIDE 복귀**→같이 탈출, 간격 1.20m) 모두 완주. **= 시나리오 7단계 상태머신 전체가 시뮬에서 돌아감.**
- ▶ **다음 후보:** ⓐ 집결지 계산 모듈(화재 좌표 기반 — `self.fire['pos']` 준비됨) ⓑ 역할 B `.msg` 계약 합의(+funnel 연결) ⓒ 중간보고서 초안(재료 풍부) ⓓ RPP 속도저하 규명(이월) ⓔ micro-ROS(구동부 진행 시). + 학습: 세션 첫 30분 = **URDF 코드리딩** (로드맵: §9).

### 새 런치/실행법 (2D~2E)
- `ros2 launch tunnel_sim robot.launch.py` — Gazebo+터널+로봇 (gui:=false 면 헤드리스)
- `ros2 launch tunnel_sim slam.launch.py` — 위 + slam_toolbox (지도작성). 다 돌면 `ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/tunnel_map`
- `ros2 launch tunnel_sim nav2.launch.py` — 저장맵+amcl+Nav2
- `ros2 launch tunnel_sim slam_nav2.launch.py` — ★ 라이브SLAM+Nav2(amcl·저장맵 불필요, 검증에 사용). 목표: `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: ..., y: ...}, orientation: {w: 1.0}}}}"`
- 키보드 운전: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
- **미션 로직 (2F):** slam_nav2 켠 뒤 → `ros2 run mission_manager mission_node --ros-args -p use_sim_time:=true` (두뇌) + `ros2 run tunnel_sim fake_follower --ros-args -p use_sim_time:=true` (가짜 추종자). 화재: `ros2 topic pub --times 2 -w 1 /alarm geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 14.0, y: 0.0}}}"`. 놓침 재현: `ros2 topic pub --times 3 -w 1 /follower_cmd std_msgs/msg/String "{data: stop}"` (재개는 `follow`). 관찰: `ros2 topic echo /mission_state`. ⚠ pub 은 `--once` 대신 `-w 1` (§함정).

### 2C 진척 & 남은 일 (→ 끝나면 2D)
- ✅ **① GUI 조작 + 모델 배치 워크플로우 정착** (2026-06-23) — 마우스 조작 3종 익힘. GUI Insert/저장이 불안정해 **"마우스로 시각 배치 → `gz model -m 이름 -p`로 좌표 읽기 → `tunnel.world`의 `<include>`에 박기 → colcon build"** 워크플로우 확립. tunnel.world에 cone_1·barrel_1·victim_1(person_standing) 배치 완료. (상세: `0623_현황.md` §8)
- ✅ **② tunnel.world 보강** (2026-06-23) — ㄷ자 → **T자 분기 터널** (메인 30m×폭6m×높이2m + 곁복도 12m). 벽 8개, 분기=북벽 2조각으로 입구 뚫음. 곁복도 끝에 victim. (상세: `0623_현황.md` §9)
- ✅ **③ 센서 플러그인** (2026-06-23) — box_robot에 `<sensor type="ray">` + `libgazebo_ros_ray_sensor.so` → **시뮬 `/scan` 발행 확인** (10Hz·360도, 실물 C1과 동일). RViz 시각화까지(static TF map→base_link 필요). **→ 2C 전체 완료.** (상세: `0623_현황.md` §10)

### 매 세션 운영 규칙 (항상 적용)
- **자동 source됨:** `~/.bashrc`에 `/opt/ros/humble` + `~/ros2_ws/install` setup.bash 등록 → 새 터미널 source 불필요. 단 `colcon build`는 수동, 항상 `--symlink-install`.
- **Jetson 작업:** 계정은 **`hanhan`** (노트북은 `minwoo`). `ssh hanhan@jetson.local` (핫스팟+mDNS). **Claude는 SSH 비번 못 침** → 사용자가 접속·붙여넣기, Claude는 명령 제공·결과 해석(협업 패턴). **노트북 빌드물은 ARM(aarch64)에서 못 씀 → Jetson에선 소스 새로 colcon build.**
- **Gazebo 실행:** `sudo prime-select on-demand`(재부팅) 후 `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia ros2 launch ...`. 종료는 **Ctrl+C** 또는 `pkill -9 gzserver`(창 X버튼만으론 gzserver 잔존). 평소엔 `sudo prime-select intel`(배터리 절약).
- **Gazebo = Classic 11** (Humble 표준). 신버전 `gz sim`/Ignition과 명령·문법 다름 — 인터넷 자료 볼 때 Classic용인지 확인.
- **패키지 분리:** `my_first_pkg`(학습용 talker/listener) / `tunnel_sim`(시뮬 전용 — worlds·urdf·maps 모임).

### 자주 무는 함정 (요약 — 상세는 현황.md)
- **런치/월드 만들면 `setup.py`의 `data_files`에 등록 필수** (안 하면 `ros2 launch`가 파일 못 찾음). 상단에 `import os`, `from glob import glob`도 함께. → 0621_현황.md
- **라이다 디버깅 순서:** ttyUSB 인식(`lsusb`→`ls /dev/ttyUSB*`) → dialout 권한(`usermod -aG dialout` 영구 / `chmod 666` 임시) → 소스빌드 → `/scan` 검증. ttyUSB 안 보이면 윗단계 전부 무의미. → 0623_현황.md
- **RViz 런치(`view_*`)는 드라이버를 자기가 다시 켬** → 기존 드라이버 `pkill -9 sllidar_node` 먼저.
- **런치 심화:** `parameters` 키는 `declare_parameter` 이름과 정확히 일치 / remap은 발행·구독 양쪽 동시에 / `get_parameter('이름').value`의 `.value` 빠뜨리면 객체 나와 에러. → 0621_현황.md
- **SDF/XML:** `<pose>`는 물체 **중심점**(벽 높이 1m면 `z=0.5`) / 각도는 **라디안** / **visual만 있고 collision 빠지면 LiDAR가 통과**(벽 안 찍힘) → 벽엔 visual+collision 항상 같이. → 0621_현황.md §14
- **센서 플러그인:** 가상 LiDAR = 몸체(link)+`<sensor type="ray">`+`<plugin libgazebo_ros_ray_sensor.so>` 3층(플러그인=시뮬의 드라이버). ⚠️ 플러그인은 **/scan만 발행, TF 안 줌** → RViz 보려면 `ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id base_link ...` 따로. RViz LaserScan **QoS=Best Effort**. → 0623_현황.md §10
- **Gazebo GUI 함정 5종:** ① Insert 모델은 **`models.gazebosim.org`만, Fuel 금지**(Classic 아님) ② **무거운 모델**(자동차·heightmap·walking person actor) 넣으면 `gzclient exit -9` 크래시 → **가벼운 것만**(cone/barrel/standing person) ③ gzclient 죽어도 **gzserver 잔존** → `pkill -9 gzserver` ④ **Save World As 다이얼로그 안 뜸**(PRIME 오프로드) → 저장은 "마우스 배치 → `gz model -m 이름 -p` 좌표 읽어 `<include>`에 박기"로 우회 ⑤ Insert 온라인 `Connecting` hang. `.world` 수정 후 **colcon build 필수**. → 0623_현황.md §8
- **URDF/diff_drive 함정 (2D, → 0626_현황.md):** ① **belly drag** — 몸통 박스 바닥이 지면(z=0)에 닿으면 바퀴 굴러도 몸통이 끌려 거의 안 감 → 박스 바닥에 클리어런스(z=0.05). ② **4륜 강구동 = 과구속** — Gazebo에서 앞/뒤 바퀴가 싸워 제자리회전 불가. 시뮬은 **2 구동휠+캐스터**로. ③ 모든 link에 `<inertial>` 필수(없으면 안 움직임). ④ Gazebo 바퀴 마찰은 `<gazebo reference>`의 `mu1/mu2`로, `kp` 낮추면 바퀴 바닥에 파묻혀 stuck. ⑤ **diff_drive `/odom`은 spawn 위치 포함한 world 좌표로 나옴**(0,0 아님) → 웨이포인트 좌표 주의.
- **헤드리스/프로세스 함정 (2E, → 0626_현황.md):** ① **GUI 없이 검증** — `robot.launch.py gui:=false`로 gzserver만, `gz model -m tunnel_robot -p`(ground truth)·`ros2 topic echo /odom`로 검증(눈보다 정확). ② 백그라운드 런치는 **수동 `setsid nohup &` 불안정** → Claude는 Bash `run_in_background` 사용. ③ **`pgrep -f 패턴`은 자기 명령줄도 매칭**해 가짜 카운트 → `pgrep -x 프로세스명` 사용. ④ 재시작 시 좀비 누적(특히 robot_state_publisher) + `TF_OLD_DATA` 경고 폭주 → `pkill -9 -x` 전부 + `ros2 daemon stop/start`. ⑤ slam 지도 저장 `ros2 run nav2_map_server map_saver_cli -f 경로`, localization용 posegraph는 `/slam_toolbox/serialize_map` 서비스.
- **Nav2 함정 (2E 자율주행, 2026-06-27, → 0626_현황.md):** ① **`ros2 launch`(백그라운드) 프로세스가 안 죽으면** 자식 노드(planner·bt_navigator·gzserver) 계속 살아 좀비·중복 라이프사이클 → 반드시 `pgrep -f "ros2 launch" | xargs kill -9` 부모부터. **robot_state_publisher는 comm이 `robot_state_pub`(15자)** 라 `pkill -x robot_state_publisher` 안 먹음 → `pkill -f`. ② **map_server `yaml_filename not initialized`** → nav2_params에 `map_server: {yaml_filename: ""}` 섹션 필수(bringup이 map인자로 치환). ③ ~~라이브SLAM+Nav2 시작 시 /map이 작아 global_costmap 10×8 → rolling_window:true로 해결~~ **← 폐기(07-05 4차): rolling+static_layer 조합이 간헐 빈경로 결함의 원인으로 판명 → 현재 정답 = `rolling_window: false`** (미션노드/Nav2 함정 ③ 참조). ④ **DWB "No valid trajectories"** → RPP 컨트롤러로 교체가 강건. RPP "collision ahead 오탐"은 `use_collision_detection:false`. ⑤ **★ 긴 복도 = lidar SLAM 길이방향 드리프트(corridor problem)** — 특징 없는 직선복도서 scan-matching이 전후위치 못잡아 ~9m 어긋남 → **EKF(odom+IMU) 또는 AMCL+저장맵**으로 보강 필요(odom은 길이방향 정확). ⑥ **/odom·map = world 좌표**(spawn 오프셋 포함) → 목표좌표 줄 때 `ros2 run tf2_ros tf2_echo map base_footprint`로 실제 좌표 먼저 확인.
- **EKF/SLAM튜닝 함정 (2E 마무리, 2026-07-05, → 0705_현황.md §7):** ① **TF 이중발행 금지** — EKF 켜면 diff_drive `publish_odom_tf` 반드시 false(둘 다 쏘면 위치 널뜀). ② **EKF엔 위치 말고 속도 융합** — odom0_config는 vx·vy·vyaw(위치 x·y를 믿으면 드리프트까지 통째로 흡수). ③ **`distance_variance_penalty`는 이름과 달리 '분모(분산)'** — 키우면 벌점 약해짐! odom 신뢰 올리려면 **작게**(0.02). 50으로 키웠다가 무효과 실측. ④ 복도 드리프트의 실체 = 매처가 "연속 스캔이 똑같음 → 안 움직였다"를 적극 선호 → `correlation_search_space_dimension` 축소(0.2) + 벌점 강화로 odom이 이기게. ⑤ **EKF만으론 안 끝남** — SLAM이 odom을 덮어쓰므로 slam_params 튜닝까지가 한 세트. ⑥ 시뮬 IMU에 노이즈 일부러 부여(covariance=0이면 EKF가 신뢰도 판단 불가). ⑦ 실차 이식 시: 실물 odom은 슬립으로 부정확 → penalty 0.02가 너무 강할 수 있음, 재튜닝 항목 (**→ §10 오돔주입 실험으로 실증 완료, 실차 시작값 = slam_params_realodom.yaml**).
- **오돔주입/프로세스 함정 (07-05 3차, → 0705_현황.md §10):** ① **★ diff_drive `odometry_source` 기본값 = `world` = 치트 오돔**(Gazebo 실위치 복사, 바퀴 계산식 무시) — 오돔 실험·실차 근접 시뮬엔 반드시 `encoder` 명시(그래야 파라미터 오차가 odom에 반영, (0,0) 시작). ② **`pkill/pgrep -f`는 자기 명령줄도 매칭해 자살** — 명령 문자열에 프로세스명이 들어가면 자기 셸 kill(2회 실측). 브래킷 트릭 필수: `pgrep -f "ros2[ ]launch"`. ③ **`pkill -x`는 comm 15자 잘림** — `async_slam_toolbox_node`의 comm은 `async_slam_tool`(15자)라 16자 이상 패턴은 조용히 실패 → `pkill -f "async_slam[_]toolbox_node"`.
- **토픽 통신/테스트 자동화 함정 (07-06 ③단계, → 0705_현황.md §14.3):** ① **★ `ros2 topic pub --once` 는 디스커버리 매칭 전에 쏘고 죽을 수 있음** — 구독자가 버젓이 있어도 메시지 유실(테스트 사이클 통째 무효 실측). → **`-w 1`(구독자 매칭 대기) + `--times 3`** 조합 사용. ② 스폰류 서비스는 **멱등으로**: "already exists"를 실패 취급하면 재시도 경고 폭주 → 기존 모델 '접수'(재사용+텔레포트). ③ 자동화 테스트에선 명령이 닿았는지 **상대 노드 로그로 수신 확인** 후 다음 단계로.
- **미션노드/Nav2 프로그래밍 함정 (07-05 4차, → 0705_현황.md §12.2):** ① **goal 의 header.stamp 는 비워라(=0)** — '지금시각' 찍으면 planner 재계획 때마다 그 시각 TF 요구, 버퍼(~27s) 지나면 extrapolation 에러→회복스핀 무한. stamp 0 = "최신 TF 사용"(CLI 가 잘 됐던 이유). ② **콜백 안 블로킹 금지** — rclpy 싱글스레드라 wait_for_server(2.0) 하나가 노드 전체(구독·발행) 동결 → server_is_ready() 즉답+다음 tick 재시도 패턴. ③ **★ rolling global costmap + static layer 금지 (라이브 SLAM)** — 지도 성장 리사이즈와 충돌, 간헐 '성공+빈경로'로 goal ABORTED(이동 중에만 발생해 재현 어려움) → `rolling_window: false` 가 SLAM+Nav2 표준. ④ 상태성 신호(/siren 등)는 전환 1회 발행이 아니라 **매 tick 반복 발행**(늦은 구독자 대비). ⑤ 시뮬에서 자작 노드는 `--ros-args -p use_sim_time:=true` 필수(빼면 stamp·타이머가 현실시간).
- **회귀진단 함정 3종 (07-05 심야, → 0705_현황.md §8):** ① **거리만 잠그면 매처가 '회전'으로 탈출** — 지도가 통째로 기울어짐 → `angle_variance_penalty 0.05`+`minimum_angle_penalty 0.1`(기본 0.9=무벌점!)+`coarse_search_angle_offset 0.1`로 회전도 잠가야 한 세트. ② **라이다 평면 = 몸통 윗면 높이면 상시 자기타격** → 지도에 유령 상자 행렬. 라이다는 몸통 최상면보다 위에(시뮬 z+0.05, **실물도 동일 — 하드웨어팀 전달**). ③ **낮은 장애물(콘 ~0.3m)은 라이다 사각지대** → costmap에 안 보여 로봇이 밀고 다니다 회전 시 끼어 쐐기. "매번 같은 좌표에서 멈춤"=물리 덫 신호, cmd_vel vs gz pose 동시 관측으로 잡음. **= Orbbec 깊이카메라 필수의 실증 근거.** + Nav2: `inflation_radius ≥ 로봇 외접반경(0.32)` 필수(0.9로 중앙 선호), progress_checker 회전 고려 완화, Humble 기본 BT엔 BackUp 회복 없음.

### ★ 통찰 (왜 시뮬 먼저 하나)
Gazebo 플러그인이 만드는 "가짜 `/scan`·`/odom`·`/imu`"가 실물과 **타입·형식 동일** → 시뮬로 짠 SLAM·Nav2 코드를 실물에 거의 그대로 이식. (06-23 라이다 실물검증으로 `/scan` 동일함 실증 완료.)

---

## 7. Phase 3 진입 전 구동부로부터 수령할 항목

> 구동방식(differential·앞바퀴 조향 없음)은 **확정 완료**. 아래 실측값 + micro-ROS 토픽 합의가 남음.

| 파라미터 | 설명 | 사용처 |
|---|---|---|
| `wheel_separation` | 좌우 바퀴 중심 간격 (m) | URDF · 오도메트리 |
| `wheel_radius` | 바퀴 반지름 (m) | URDF · 오도메트리 |
| `robot_footprint` | 로봇 외형 크기 | Nav2 costmap inflation |
| `encoder_ppr` | 엔코더 1회전당 펄스 수 | Teensy 오도메트리 계산 |

- ※ 수령 전까지 URDF는 **예상값**으로 작성, 수령 후 수치만 교체.
- micro-ROS 토픽 합의(후순위): `/odom`·`/imu/data`·`/cmd_vel`의 frame_id(`odom`/`base_link`)·발행주기·단위(m/s, rad/s).

---

## 8. 문서 / 파일 위치 맵

| 문서 | 위치 | 역할 |
|---|---|---|
| **CLAUDE.md** (이 파일) | `~/ros2_ws/CLAUDE.md` | 규칙 + 현재상태 (매 세션 자동로드, Phase 번호 정본) |
| 마스터 컨텍스트 | (프로젝트/노션) | 큰 그림. 방향 점검 시 동기화하는 상위 문서 |
| `0620_현황.md` | `~/Desktop/개발현황/` | Phase 1 환경세팅 전 과정 (구 한이음_환경세팅_작업기록) |
| `0621_현황.md` | `~/Desktop/개발현황/` | 2B ROS2 기초 + 2C Gazebo 개념 (§9 파이썬 / §10 런치 / §13~14 Gazebo·SDF) |
| `0623_현황.md` | `~/Desktop/개발현황/` | RPLIDAR C1 실물테스트 전과정 (디버깅순서·포트권한·LaserScan 구조·RViz) |
| `0626_현황.md` | `~/Desktop/개발현황/` | 2D URDF(differential) + 2E SLAM 지도 전과정 (URDF구조·diff_drive튜닝·2휠전환·헤드리스SLAM·지도저장) |
| `0705_현황.md` | `~/Desktop/개발현황/` | §7 EKF 완료(9m→0.17m) · §8 회귀진단 · §10 오돔주입 · §11~12 미션설계+뼈대 · §13 ②단계 · **§14 ③단계(후방감지+SEARCH_BACK) — 미션 상태머신 전체 기록** |
| `0705_실차전_전략.md` | `~/Desktop/개발현황/` | ★ 실차 전 전략 정본: 가짜 detection 판단(계약+깡통만) + 실차 결합 5단계 로드맵 |
| `NVIDIA_GPU_복구_작업브리프.md` | `~/setup-tasks/` | GPU 복구 완료 + 재빌드 레시피(`nv_rebuild_recipe.sh` 동봉) |
| `스피커_SOF_복구_진행로그.md` | `~/setup-tasks/` | 내장 스피커 SOF 펌웨어 복구 기록 |

> ⚠️ `setup-tasks` 폴더명 확인 필요: 하이픈(`setup-tasks`)인지 언더스코어(`setup_tasks`)인지 `ls ~ | grep setup`으로 1회 확인해 통일할 것. (NVIDIA 브리프 내부 경로는 하이픈 기준으로 기록돼 있음)
