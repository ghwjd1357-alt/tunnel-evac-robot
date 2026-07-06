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

## 6. 현재 진행 상태 (2026-07-06 2차 갱신 — ★ 미션 상태머신 완료 + 감지 2차 + 테스트 인프라. §6은 압축본, 상세는 현황.md)

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
| Nav2 자율주행 | ✅ **완전검증 (2026-07-06 갱신):** SLAM 궤적오차 평균 0.027m, tolerance 강화 후 **끝점오차 0.12~0.17m** (07-05 의 0.24m 에서 개선 — 병목이 goal tolerance 였음, §17) |
| **미션 로직 (`mission_manager`)** | ✅ **①~③ + 감지 2차 완료 (2026-07-06):** PATROL→APPROACH→GATHER→GUIDE⇄SEARCH_BACK→ESCAPED+FAULT 전체 시뮬 E2E 검증. 추종감지 = 클러스터 크기 판별(GUIDE 판정 any존), 안전장치 2종, `mission.launch.py` 한 줄 실행 (상세: 0705_현황.md §12~15) |
| **테스트·회귀 인프라** | ✅ 2026-07-06: git 저장소(작업 단위 커밋) + pytest 13개 + `tools/regression_3goals.sh` + `tools/mission_e2e.sh` — **변경 후 이 3종 실행이 회귀 검증** (상세: §15.5) |
| EKF (robot_localization, odom+IMU 융합) | ✅ 2026-07-05: 시뮬 IMU 추가 + `config/ekf.yaml` + slam 파라미터 튜닝. 12.5m 주행 시 SLAM 위치오차 **9m → 0.17m** (상세: `0705_현황.md` §7) |
| Orbbec 카메라 (OrbbecSDK_ROS2) | ⬜ **의도적 보류** (시뮬 트랙엔 불필요, 역할 B·실차 단계에서) |
| micro-ROS agent | ⏸ **후순위** — 구동부 Teensy 진행에 맞춰 별도 시점 진행 |
| JetPack SDK (CUDA/cuDNN/TensorRT) | ⏸ 추후 `sudo apt install nvidia-jetpack` |

### 로드맵 & 현재 위치
**2A** 클코 학습 ✅ → **2B** ROS2 기초 ✅ → **2C Gazebo ✅** → **2D URDF(differential) ✅** → **2E SLAM+Nav2+EKF ✅** → **2F 미션 로직 ①뼈대 ✅ ②GATHER/GUIDE ✅ ③후방감지+SEARCH_BACK ✅** (2026-07-06: 시나리오 전체 상태머신 시뮬 E2E — 실패·성공 경로 모두 실증) ◀ 현재 = 다음 선정 (후보: 집결지 계산 모듈 / 역할 B `.msg` 계약 / 중간보고서 / micro-ROS)
- **micro-ROS = 후순위** (구동부 진행에 맞춰, 위 로드맵과 별개 시점).
- ★ **메인 트랙은 Gazebo 시뮬.** 실물 라이다는 `/scan`만 나오고 `/odom`(움직임)이 없어 단독 SLAM 불가 → odom은 구동부가 줄 때까지 **시뮬로 SLAM·Nav2 개발.**

### 완료 이력 (한 줄 요약 — ★ 상세는 날짜별 현황.md 해당 절, 여기엔 다시 안 적음)
- ✅ **2D URDF 로봇** — diff_drive. 시뮬 구동모델 = 2구동휠+캐스터(4륜 강구동은 Gazebo 과구속으로 회전 불가). 토픽·TF 동일 → 실물 이식 무관. → `0626_현황.md`
- ✅ **2E SLAM 지도 + Nav2 부분검증** — T자 지도 저장, 13m 자율주행. 복도 길이방향 드리프트(~9m) 발견 → EKF 과제화. → `0626_현황.md`
- ✅ **EKF + SLAM 튜닝 (07-05)** — 위치오차 **9m→0.17m**, Nav2 12m 목표 0.24m 도달. ⚠ penalty 류는 '분모' — 작을수록 odom 신뢰↑. → `0705_현황.md §7`
- ✅ **회귀진단 4건 (07-05 심야)** — 회전탈출·라이다 자기타격·안 보이는 콘·★벽 관통 경로(→ static_layer 복귀 + `allow_unknown:false`). 3목표 SUCCEEDED, 지도 v3. → `§8`
- ✅ **오돔 오차 주입 (07-05 3차)** — ★ diff_drive `odometry_source` 기본=world는 치트 오돔. 회전 거짓은 EKF(IMU)가 교정, 거리 거짓은 SLAM 몫 → 실차 시작값 `slam_params_realodom.yaml`. → `§10`
- ✅ **실차 전 전략** — 가짜 detection 금지, `.msg` 계약+깡통 퍼블리셔만. 실차 순서: 브리지→teleop/odom(★최대 관문)→EKF→SLAM→Nav2. → `0705_실차전_전략.md`
- ✅ **미션 ①뼈대 (07-05 4차)** — `mission_manager` 신설(상태머신+waypoints.yaml). goal stamp=0·콜백 비블로킹·★`rolling_window:false` 종결. 시나리오는 잠정(유동적 — 뼈대/살 분리로 대비). → `§12`
- ✅ **미션 ② (07-05 5차)** — GATHER/GUIDE 정식 승격, GUIDE 저속 0.12 동적 변경, fake_follower(+`/follower_cmd` stop=놓침 재현). → `§13`
- ✅ **미션 ③ (07-06)** — follower_monitor(디바운스 비대칭·이중 구역) + SEARCH_BACK(안전장치 2종: 시도 2회 제한·화재하한 5m 클램프). 실패·성공 경로 E2E 완주 = 시나리오 상태머신 전체 가동. → `§14`
- ✅ **품질 세션 (07-06 오후)** — **git 개시**(.gitignore+커밋, 이후 작업 단위마다 커밋) · **RPP 속도저하 숙제 종결**(재현 불가 — rolling 결함 시절 부작용. cost regulation 명시 OFF + scaling 2.5 일치) · **감지 2차 = 클러스터 크기 판별**(벽 호 배제·랩 병합·슬라이버 배제) · **★GUIDE 판정 rear→any 존 전환**(집결 180° 회전 후 추종자가 '앞' — rear만 보면 가짜 놓침으로 역행예산 전소, E2E 자동화가 첫 실행에 검거) · **pytest 13개 + 회귀 스크립트 2종 + mission.launch.py**. → `§15`
- ✅ **ⓐ 집결지 계산 (07-06 저녁)** — 고정 좌표 → **화재→탈출구 방향선 위 gather_dist(8m) 지점 계산**(`compute_gather_point` 순수 함수, yaw=탈출구 방향). 클램프·fallback(yaml gather) 포함, gather_dist=8.0 은 표준 화재(14,0)에서 기존 검증값 (6,0) 재현. pytest 18개 + E2E PASS. 한계: 직선 수식(곁복도 화재는 그래프 경유지 과제). → `§16`
- ✅ **정확도 벤치 + tolerance 강화 (07-06 밤)** — `tools/accuracy_{sampler,bench,report}` 신설(매초 ground truth vs SLAM 오차 → CSV·그래프). ★ 실측: SLAM 궤적오차 **평균 0.027m** vs 끝점오차 0.26~0.30m = **병목은 xy_goal_tolerance(0.3)** → xy 0.15·yaw 0.25·planner 0.25 로 조임 → 끝점오차 **절반(0.12~0.17m)**, +2~3s/goal 뿐(맴돎 없음). 남은 후보: localization 모드 전환·BT BackUp 회복·velocity_smoother(실차). → `§17`
- ▶ **다음 후보:** ⓑ 역할 B `.msg` 계약 합의(+funnel 연결) ⓒ 중간보고서 초안(재료 풍부) ⓔ micro-ROS(구동부 진행 시) + 감지 3차(지도 배경제거). 변경 후 회귀 = `bash tools/regression_3goals.sh` + `bash tools/mission_e2e.sh` + `pytest src/mission_manager/test/`. + 학습: 세션 첫 30분 = **URDF 코드리딩** (로드맵: §9).

### 런치/실행법
- **★ 미션 전체 (2F):** `ros2 launch tunnel_sim mission.launch.py` — 시뮬+SLAM+Nav2+미션노드+가짜추종자 전부 한 줄 (`gui:=false` 헤드리스 / `follower:=false` 추종자 제외 / use_sim_time 은 런치가 챙김).
  - 화재: `ros2 topic pub --times 2 -w 1 /alarm geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 14.0, y: 0.0}}}"`
  - 놓침 재현: `ros2 topic pub --times 3 -w 1 /follower_cmd std_msgs/msg/String "{data: stop}"` (재개 `follow`) / 관찰: `ros2 topic echo /mission_state`. ⚠ pub 은 `--once` 금지, `-w 1` (§함정).
- `ros2 launch tunnel_sim slam_nav2.launch.py` — 라이브SLAM+Nav2 만 (미션노드 없이 목표 실험용). 목표: `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: ..., y: ...}, orientation: {w: 1.0}}}}"`
- `ros2 launch tunnel_sim robot.launch.py` — Gazebo+로봇만 / `slam.launch.py` — +지도작성(저장: `map_saver_cli -f ~/ros2_ws/maps/tunnel_map`) / `nav2.launch.py` — 저장맵+amcl / 키보드: `teleop_twist_keyboard`
- **★ 변경 후 회귀 3종:** `python3 -m pytest src/mission_manager/test/ -q`(0.2초) + `bash tools/regression_3goals.sh`(~4분) + `bash tools/mission_e2e.sh`(~3분)
- **정확도 벤치 (SLAM·Nav2 튜닝 전/후 측정):** `bash tools/accuracy_bench.sh 라벨` → `bench_out/라벨/{trace.csv,summary.txt,error.png}` / 비교: `python3 tools/accuracy_report.py A/trace.csv B/trace.csv --labels 전 후 -o compare.png` (§17)

### 2C 완료 (2026-06-23) — GUI 배치 워크플로우("마우스 배치→`gz model -m 이름 -p` 좌표→world `<include>`→build") · T자 터널(메인 30m×폭6m + 곁복도 12m) · 가상 라이다 `/scan` 검증. → `0623_현황.md §8~10`

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
- **테스트/스크립트 함정 (07-06 품질세션, → 0705_현황.md §15.7):** ① **셸 스크립트 `set -u` 는 ROS setup.bash source 뒤에**(setup.bash 가 미정의 변수 참조 → unbound variable 즉사) ② **테스트 가짜 시계는 정수 ns 누적**(float 0.1×11=1.09999…로 디바운스 경계 1.0초를 못 넘는 가짜 실패) ③ **오래 뜬 액션 서버 + 새 CLI = goal 응답 유실 가능**(bt_navigator "Failed to send goal response" 실측, send_goal 영구 대기) → 자동화엔 타임아웃+1회 재전송.
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
| `0705_현황.md` | `~/Desktop/개발현황/` | §7 EKF(9m→0.17m) · §8 회귀진단 · §10 오돔주입 · §11~12 미션설계+뼈대 · §13 ② · §14 ③(후방감지+SEARCH_BACK) · **§15 품질세션(git·RPP종결·클러스터 감지·any존·테스트 인프라·mission.launch)** · §16 집결지 계산 · §17 정확도 벤치+tolerance 강화 |
| `0705_실차전_전략.md` | `~/Desktop/개발현황/` | ★ 실차 전 전략 정본: 가짜 detection 판단(계약+깡통만) + 실차 결합 5단계 로드맵 |
| `NVIDIA_GPU_복구_작업브리프.md` | `~/setup-tasks/` | GPU 복구 완료 + 재빌드 레시피(`nv_rebuild_recipe.sh` 동봉) |
| `스피커_SOF_복구_진행로그.md` | `~/setup-tasks/` | 내장 스피커 SOF 펌웨어 복구 기록 |

> ✅ `setup-tasks` 폴더명 = 하이픈 확정 (07-06 `ls ~` 실측 — 브리프 내부 경로와 일치).
