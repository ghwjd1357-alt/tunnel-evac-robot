# CLAUDE.md — Claude 전용 진입점 / 한이음 지하터널 대피로봇 · 역할 A (손민우)

> 매 세션 자동 로드되는 최소 진입점 (07-20 개편 — 구판 백업 = `docs/legacy/CLAUDE_pre-restructure_0720.md`, git `8042464`).
> 프로젝트 사실·계획·함정의 정본은 `AGENTS.md` + `docs/`. 이 파일엔 Claude 전용 규칙만.

## 0. 필수 읽기 (명령형 — 건너뛰지 않는다)

1. **`AGENTS.md`** — 공통 규칙·사고 프로토콜·세션 파괴급 함정 5종·운영 규칙. **먼저 읽는다.**
2. **`docs/CURRENT_HANDOFF.md`** — 이번 한 묶음의 범위·완료조건·게이트. **작업 시작 전 반드시 읽는다.**
3. 원문을 여러 번 따로 열기 전에 `tools/ai_context.py`로 구현/검토 문맥 패킷을 **한 번** 만든다
   (`docs/AI_CONTEXT.md`). 실패·미지 경로면 패킷 출력을 신뢰하지 말고 아래 수동 순서로 복귀한다.
4. 작업에 필요한 절만: `docs/PROJECT_CONTEXT.md`(구조·계약·문서 맵) / `docs/MASTER_PLAN.md`(순서·사다리·유효 결정 색인) / `docs/TEST_GATES.md`(테스트·런치 실행법) / `docs/PITFALLS.md`(영역별 함정 — 해당 영역 작업 전 그 절) / `docs/FREEZE_MANIFEST.md`(동결 증거·해시·조건부 위험 — **platform-core 를 건드리기 전**) / `docs/REAL_ROBOT_VALUES.md`(구동부 실측값·실차 반영 지점 — 실차 작업 시) / `docs/ELECTRICAL_BASELINE.md`(전기계통 사실·위험·최소 안전선 — **E-stop·전원·배선·퓨즈를 건드리기 전**) / `docs/FIRMWARE_REBUILD.md`(펌웨어 재빌드 환경·platform.txt 패치 — **펌웨어를 다시 굽기 전**).

정본 우선순위·충돌 규칙은 `AGENTS.md §2`. 링크는 항상 `파일명 §번호`로 정확히 따라간다.
Desktop 날짜별 현황(`~/Desktop/개발현황/`)은 역사·근거 — 필요 절만 선택해 읽는다.

## 1. Claude 작업 규칙 (매 세션)

- **기술 스택**: ROS2 Humble + C++/Python. **호환성 최우선** — Ubuntu 22.04 + Humble 확인 먼저.
- **눈높이 ★**: 사용자는 ROS·프로그래밍 **초보자** (파이썬·XML 첫 입문). 코드만 던지지 말고 원리·구조·핵심 명령어를 초보자 눈높이로 상세히 설명한다.
- **종속성 명시**: 새 패키지 제안 시 의존성 + 설치 명령어 함께. 센서·드라이버는 호환 문제 사전 경고 + 대안.
- **단계별 진행**: 빌드 → 테스트 → 디버그로 쪼개어 하나씩 검증. 에러 로그는 원인·해결책 분류 설명.
- **모듈화 + 상세 주석.** 한 세션 = 한 묶음, 변경 직후 회귀 (`AGENTS.md §3-9`, 게이트 = `docs/TEST_GATES.md`).
- **커밋 = push 한 세트** (`AGENTS.md §5`). 빌드는 수동 `colcon build --symlink-install`.

## 2. 환경·협업 패턴 (Claude 전용)

- `~/.bashrc` 에 humble + `~/ros2_ws/install` setup.bash 자동 source — 새 터미널 source 불필요.
- **Jetson**: 계정 **`hanhan`** (노트북은 `minwoo`), `ssh hanhan@jetson.local`. **Claude 는 SSH 비번 못 침** → 사용자가 접속·붙여넣기, Claude 는 명령 제공·결과 해석. **노트북 빌드물은 aarch64 에서 못 씀** → Jetson 은 repo clone 후 소스 colcon build.
- 백그라운드 실행은 Bash `run_in_background` 사용 (수동 `setsid nohup` 불안정).
- Gazebo GUI 실행법·E2E 실행법 = `docs/TEST_GATES.md §3`. Gazebo = **Classic 11** (Ignition 자료 주의).
- 패키지: `tunnel_sim`(시뮬 자산) / `tunnel_bringup`(실차 런치·설정) / `mission_manager`(미션) / `perception_adapter`(인지→미션 어댑터 · 08-21 신설) / `tunnel_interfaces`(역할 B 계약 `.msg` 2종 — **실행 코드 0**, 인터페이스 전용) / `my_first_pkg`(학습용) / `console/`(관제 웹, ROS 패키지 아님).
