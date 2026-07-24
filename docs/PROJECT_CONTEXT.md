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
| 입력 | `/imu/data` | Imu | 구동부 Teensy (BNO055, 100Hz 예정) |
| 입력 | `/detections` | 커스텀 (`tunnel_interfaces` 제안) | 역할 B. 아래 §4.1 |
| 출력 | `/cmd_vel` | Twist | Nav2 → Teensy (`linear.x`·`angular.z`만) |
| 출력 | `/map`, `/tf` | — | SLAM |

### 4.1 `/detections` V1 계약 (★ 07-20 책임경계 (b) 확정)

- **YOLO(역할 B) 측이 depth 결합까지 담당 — camera-frame 3D position 제공.**
  V1 = header(stamp=촬영시각, frame=camera optical frame) · class_name · confidence · bbox · position(m).
- `.msg` 필드 추가 = **타입 변경 = 양측 동시 리빌드 필요** → V1 최소 계약 고정, 확장은 V2 별도 메시지 (07-19 개정).
- map 좌표 생성·검증(timestamp·frame·반복관측·오탐 억제)은 역할 A **Perception Adapter** 책임.
- 계약 정본 = `~/Desktop/YOLO_탐지연동_합의사항.md` (구 `역할B_detection_토픽계약_전달.md` 는 대체됨).
- 수신은 **funnel 구조** (콜백 1개 → 내부 dict) — 필드 추가 시 콜백 한 곳만 수정.

## 5. 하드웨어 확정값

- **DC 모터 ×4 + 강체 4륜 차동구동(differential) 확정** — 앞바퀴 조향 없음, 제자리 회전 가능.
  4륜 스키드 회전 슬립 → **IMU yaw 융합(EKF) 필수** (EKF는 Jetson 위 노드 = 역할 A).
- ★ 실측값 4종 수령 완료 (07-08~18, 정본 = **`docs/REAL_ROBOT_VALUES.md`** — 반영 지점·잔여 합의 포함):
  wheel_separation **0.49m** / 바퀴 지름 **0.13m** / footprint **0.55×0.57m**(외접반경 ≈0.40m) /
  encoder 26PPR → 바퀴축 TOTAL_PPR **2644.3** (실측). IMU = BNO055 100Hz.
- 배선 핀맵·구동부 검증 절차 = `0718_구동부_배선맵핑_검증절차.md`. Teensy 합의표 = `~/Desktop/TEENSY_실차연동_합의사항.md`.
- 잔여: micro-ROS 토픽 세부 합의 + 라이다 장착높이(몸통 최상면보다 위) 확인.
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
**`docs/FREEZE_MANIFEST.md`**(동결 증거 — 해시·게이트 수치·조건부 위험·검토 결과. 07-24 신설) ·
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

**Desktop 팀 공유물 (`~/Desktop/`)**: `미션노드_개발현황_및_실차이후계획.md` · `TEENSY_실차연동_합의사항.md` · `YOLO_탐지연동_합의사항.md`.

**기타**: `~/setup-tasks/`(GPU 복구 브리프·스피커 SOF 로그). `~/ros2_ws/console/`(관제 웹 — ROS 패키지 아님).
패키지: `tunnel_sim`(시뮬 자산) / `mission_manager`(미션 로직) / `my_first_pkg`(학습용).
