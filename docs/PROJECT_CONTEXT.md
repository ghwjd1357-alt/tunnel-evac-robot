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

### 4.1 `/detections` V1 계약 초안 (★ 역할 B 공동 합의 전)

- **역할 A 내부안**: YOLO(역할 B) 측이 depth 결합까지 담당해 camera-frame 3D position 제공.
  역할 B의 확인·동의는 아직 받지 않았으며 V1 공동 합의에서 확정한다.
- V1 필드 골격 초안 = header(stamp=촬영시각, frame=camera optical frame) · class_name · confidence ·
  bbox · position(m). 세부 타입·주기·QoS·실패 표현과 함께 **전부 역할 B 확인 전**이다.
- `.msg` 필드 추가 = **타입 변경 = 양측 동시 리빌드 필요** → 공동 합의 뒤 V1 최소 계약을 고정하고,
  확장은 V2 별도 메시지로 한다 (07-19 개정).
- map 좌표 생성·검증(timestamp·frame·반복관측·오탐 억제)은 역할 A **Perception Adapter** 책임.
- 계약 정본 = `~/Desktop/YOLO_탐지연동_합의사항.md` (구 `역할B_detection_토픽계약_전달.md` 는 대체됨).
- 수신은 **funnel 구조** (콜백 1개 → 내부 dict) — 필드 추가 시 콜백 한 곳만 수정.

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
패키지: `tunnel_sim`(시뮬 자산) / `mission_manager`(미션 로직) / `my_first_pkg`(학습용).
