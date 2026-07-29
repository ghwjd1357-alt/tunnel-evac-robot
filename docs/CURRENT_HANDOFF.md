# CURRENT_HANDOFF.md — 지금 할 한 묶음 (매 세션 시작점)

> 작업 완료 시 구현자가 갱신하고 검토자가 사실관계·테스트 결과를 확인한다.
> 이전 묶음 내용은 날짜별 현황으로 보낸다. 항상 이 한 묶음만 유지.

- **동결 기준점**: `212885a8292d1e86677d5beeb9f5358d76fe9b40` = tag **`platform-core-freeze-260724`**
  (annotated, 원격 push 완료). 증거 전량 = `docs/FREEZE_MANIFEST.md`.
  ★ 회귀가 나오면 `git diff platform-core-freeze-260724 --stat` 이 1차 용의선상이다.
- **현재 기준 커밋**: 이 파일과 같은 커밋 (main)
- **현재 단계**: 마스터플랜 5단 완료 → **6단(역할 B V1 합의)은 ⏸ 보류**(역할 B 회의 일정 대기 —
  `MASTER_PLAN.md §1` 6) → 순서 밖 소묶음 **tunnel-bringup-s4 착수**(`MASTER_PLAN.md §7` 예약 9).
  이 묶음은 실차 7단의 **산출물이 아니라 입력물**을 만든다.
- **⏸ 6단 보류 기록**: 준비된 YOLO V1 핸드오프 원문(목표·함정·완료조건·허용 범위 전량)은
  **커밋 `30c5e87` 에 고정 보존**했다. 역할 B 회의가 잡히면 **그 원문을 그대로 되살려 재개**한다
  — 새로 쓰지 말 것. 6단은 7단과 병렬이라 이 보류가 실차 트랙을 막지 않는다. 복원 명령:

```bash
git show 30c5e87:docs/CURRENT_HANDOFF.md            # 원문 확인
git show 30c5e87:docs/CURRENT_HANDOFF.md > docs/CURRENT_HANDOFF.md   # 그대로 되살리기
```
- **직전 완료**: 문서 동기화 점검 + 드리프트 3건 정정 `30c5e87` — ① `TEST_GATES.md §2` 관측 분포가
  벽시계 전환 이전(sleep 누적) 수치로 남아 있던 것을 갱신 + 옛 9s 계열 재사용 금지 명기,
  ② 진입점 라우터에 `docs/FREEZE_MANIFEST.md`·`docs/REAL_ROBOT_VALUES.md` 등재,
  ③ `FREEZE_MANIFEST.md §9` 에 §8.1~§8.3 앞방향 포인터. **런타임·판정 기준 무변경.**
  같은 점검의 실측: doc_check 9/9 OK · 동결 해시 **16/16 MATCH** · sllidar upstream 일치 ·
  `test_harness_guards` **10/10** · pytest **159** · colcon **165·0f·2s**.

## 이번 한 묶음 목표 — `tunnel_bringup` S4 골격

`MASTER_PLAN.md §3` 사전준비(S4)의 실차 bringup 패키지를 **지금** 만든다. 명세 정본 =
`~/Desktop/개발현황/0719_실차전환_마스터플랜.md §3.1`.

**왜 지금인가**: 착수 조건이 *"트리거 임박 시 / 트리거 2주 전부터"* 였으나, 트리거는 구동부의
"3m 직진 3% 이내" 선언이라 **역할 A 가 그 시점을 알 수 없다 — 만족 불가능한 조건**이었다
(`MASTER_PLAN.md §7` 예약 9). 지금은 실차·Jetson·역할 B 가 **모두 대기**라 역할 A 가 단독으로
진행할 수 있는 거의 유일한 항목이고, **시뮬 자산을 한 글자도 안 건드리므로 동결 위반이 아니다.**

**왜 필요한가** — 사다리 실행 열이 이 파일들을 **전제**로 한다:
R4 = `ekf_real` + bag replay / R5 = `real_mapping.launch` / R6 = `real_bringup`(mission 제외).
파일이 없으면 그 단계를 **시작할 수 없다.**

**만드는 범위 = 3층 중 1층뿐이다** (이 구분을 흐리면 "완성했다"는 거짓 보고가 된다):

| 층 | 내용 | 이 묶음 |
|---|---|---|
| ① 구조·확정값 | 런치 배선·`use_sim_time false`·절대경로 제거·실측 치수·footprint·collision ON | **✅ 이번** |
| ② 실물 접근 시 | 라이다·IMU 장착 오프셋 TF (줄자 실측) | ❌ TODO 표시만 |
| ③ R 사다리에서 | EKF covariance(R3~R4) · SLAM penalty·지도(R5) · RPP·정지거리(R6) | ❌ TODO 표시만 |

## ★ 함정 (먼저 읽을 것)

1. **`TimerAction` 금지 — 이게 이 묶음의 진짜 작업이다.** 시뮬 런치는 `period=12.0` 으로
   *"Nav2 가 자리 잡을 시간을 준 뒤 기동"* 한다(`mission.launch.py:67`·`88`, `slam_nav2.launch.py:55`).
   실차는 Jetson 부팅·USB 인식·micro-ROS 연결 타이밍이 매번 달라 **고정 시간이 성립하지 않는다.**
   → **lifecycle active · TF 연결 · `/scan`·`/odom` freshness 조건 기동**으로 재설계한다
   (`0719_실차전환_마스터플랜.md §3.1`). 복붙이 아니라 새로 짜는 부분이다.
2. **`setup.py` `data_files` 등록 누락** — `launch/`·`config/`·`urdf/` 를 등록하지 않으면 빌드는
   되는데 런타임에 파일을 못 찾는다. `+import os`, `from glob import glob` 까지 한 세트
   (`PITFALLS.md §3`).
3. **절대경로 금지** — 시뮬 config 3곳이 `/home/minwoo/…` 인데 Jetson 계정은 `hanhan` 이라
   **그 경로가 없다**(`MASTER_PLAN.md §7` 예약 8). 신규 파일은 처음부터
   `get_package_share_directory` 기준으로만 쓴다.
4. **바퀴 반경이 0.10 → 0.065 로 작아진다** → 차고(belly clearance) 재계산 필수. 몸통 바닥이
   지면에 닿으면 안 간다(`PITFALLS.md §5`). 실측 URDF 는 **시뮬 URDF 복사가 아니라 새로 작성**이다.
5. **Gazebo 플러그인 태그 전부 제거** — `robot_real.urdf` 에 `libgazebo_ros_diff_drive`·ray sensor
   플러그인이 남으면 실차에서 의미 없는 노드가 뜨거나 TF 를 이중 발행한다(`PITFALLS.md §7`).
6. **추정치로 빈칸을 메우지 말 것** — 미실측 값(장착 오프셋·covariance·penalty)은 **반드시
   `TODO: R? 실측 후 확정` 주석**으로 남긴다. 그럴듯한 숫자를 넣으면 R 단계에서 "튜닝 문제"와
   "애초에 가짜였던 값"을 구분할 수 없게 된다 — 가짜 detection 금지(`MASTER_PLAN.md §8`)와 같은 원칙.

## 완료조건

1. **패키지 생성 + 빌드** — `src/tunnel_bringup/` 신설, `colcon build --symlink-install` 성공.
   `package.xml` 의존성 명시, `setup.py` 에 `launch/`·`config/`·`urdf/` `data_files` 등록.
2. **명세 6파일 존재** (`0719_실차전환_마스터플랜.md §3.1` 그대로):
   `launch/real_bringup.launch.py` · `launch/real_mapping.launch.py` ·
   `config/nav2_params_real.yaml` · `config/ekf_real.yaml` ·
   `config/slam_real_{mapping,localization}.yaml` · `urdf/robot_real.urdf`.
3. **기계적으로 검사 가능한 3가지** (실물 없이 확인되는 완료조건):
   - `grep -rn "/home/\|/media/" src/tunnel_bringup/` → **0건**
   - `grep -rn "TimerAction" src/tunnel_bringup/` → **0건**
   - `grep -rn "use_sim_time.*[Tt]rue" src/tunnel_bringup/` → **0건**
4. **실측값 반영** (`docs/REAL_ROBOT_VALUES.md §1`·`§3`): separation **0.49** · 반지름 **0.065** ·
   footprint 꼭짓점 `[[0.285,0.275],[0.285,-0.275],[-0.285,-0.275],[-0.285,0.275]]` ·
   `inflation_radius ≥ 0.40` 확인 · **collision detection ON**.
5. **미확정값은 TODO** — 장착 오프셋·BNO055 축·covariance·penalty·RPP 튜닝값에 `TODO: R? 실측 후 확정`.
   ★ **추정치 금지** (함정 6).
6. **시뮬 회귀 무영향 실측** — 패키지 추가가 워크스페이스 빌드를 바꾸므로 기준선이 그대로인지 본다:
   `python3 -m pytest src/mission_manager/test/ -q` (159) + `colcon test` + `colcon test-result --verbose`
   (165·0f·2s) + `bash tools/mission_e2e.sh` (T자 1회 PASS).
7. 문서 동기화 → **한 커밋 + push** → `bash tools/doc_check.sh --after-push` → Codex 검토
   (범위 = `TEST_GATES.md §7` — 신규 패키지·launch 는 표에 없는 런타임 추가라 **fail-closed**,
   검토자가 관련 테스트를 직접 선정한다).

## 허용 파일/범위

- **신규** `src/tunnel_bringup/**` (전부)
- `docs/CURRENT_HANDOFF.md` · `docs/MASTER_PLAN.md`(§7 예약 9 ✅) · `docs/REAL_ROBOT_VALUES.md §2`
  (반영 지점 표를 실제 파일명으로 확정)
- `~/Desktop/개발현황/` 다음 절

## 금지 범위

- **`src/tunnel_sim/**` 변경 절대 금지** — 동결 대상(`docs/FREEZE_MANIFEST.md §3` 해시 고정)이자
  T자·쌍굴 회귀의 **살아있는 기준선**이다. 실차 값은 전부 신규 패키지 안에서만
  (`docs/REAL_ROBOT_VALUES.md §2` 5번).
- **`src/mission_manager/**` 변경 금지** — 미션 로직은 이 묶음이 아니다. 실차용 파라미터가
  필요하면 `tunnel_bringup` 의 yaml 로 주입한다.
- **추정치로 미실측 값 채우기 금지** (함정 6).
- **Jetson 실행·빌드 시도 금지** — 장비가 없다. 이 묶음은 **노트북 빌드까지만** 검증한다.
  Jetson 실검증은 R0 에서 처음 이뤄지며 그 위험은 `MASTER_PLAN.md §6` 에 등록돼 있다.
- `make_map.sh` 실런 금지. Gazebo E2E 동시 실행 금지.
- 실패를 원인 한 줄 분류 없이 재실행 금지 (`AGENTS.md §3-6` · `TEST_GATES.md §5`).

## 보존해야 할 안전 불변조건

- **★ R0 watchdog 리마인더 (잊으면 위험이 조용히 남는다)**: 실차 7단 R0 에서 **cmd_vel watchdog(단절
  0.5s 내 정지)** 실측 결과를 받는 즉시 `FREEZE_MANIFEST.md §6` 의 잔류 cmd_vel 활주 **조건부 수용을
  확정 또는 재개방**한다. 이 묶음이 R0 보다 먼저 끝나도 이 항목은 계속 열려 있다.
- **★ 08-15 플랜 B 판정일** (`MASTER_PLAN.md §6`) — 구동부 진척 주 1회 확인, 그날까지 R2 통과 선언이
  없으면 '구동부 지연' 행을 발동한다. **판정을 미루지 않는다.**
- E2E cleanup 순서 불변조건(부모 `ros2 launch` 먼저 kill → `pkill -9 -f "lib/nav2[_]"`, 브래킷 트릭 —
  `AGENTS.md §4`)은 회귀 실행 시 그대로 지킨다.

## 완료 판정 + 필수 테스트

**한 문장 완료판정**: "`tunnel_bringup` 이 워크스페이스에서 빌드되고, 명세 6파일이
**절대경로 0 · `TimerAction` 0 · `use_sim_time true` 0** 으로 존재하며, 미실측 값은 전부 `TODO` 로
표시돼 추정치가 섞이지 않았고, 시뮬 회귀(pytest 159 · colcon 165 · T자 mission PASS)가
무영향임이 실측된다."

⚠ **이 묶음은 "실차 준비 완료"가 아니다.** 실제 동작 검증은 로봇·Jetson 이 있어야 가능하며,
값의 절반은 R3~R6 에서 채워진다. 목표는 **트리거 도착 시 며칠을 절약하는 것**이다.

실행 순서 = 완료조건 6 그대로. 쌍굴·negative·3goals·abort 는 이 묶음이 시뮬 자산과 미션 로직을
건드리지 않으므로 불필요(건드리면 그 자체가 별도 묶음).

## 완료 후 다음 단계

**둘 중 사용자가 트랙을 고른다** (서로 독립):
- 역할 B 회의가 잡히면 → **6단 재개**: 위 '⏸ 6단 보류 기록' 의 복원 명령으로 원문을 되살린다
- 구동부 트리거가 오면 → **7단 실차 R0~R8** (`MASTER_PLAN.md §3`). R0 실측에서 **cmd_vel watchdog**
  결과를 받는 즉시 `FREEZE_MANIFEST.md §6` 조건부 수용을 확정/재개방.

## 근거 문서

`docs/MASTER_PLAN.md §1` · `docs/MASTER_PLAN.md §3` · `docs/MASTER_PLAN.md §6` ·
`docs/MASTER_PLAN.md §7` · `docs/MASTER_PLAN.md §8` · `docs/REAL_ROBOT_VALUES.md §1` ·
`docs/REAL_ROBOT_VALUES.md §2` · `docs/REAL_ROBOT_VALUES.md §3` · `docs/REAL_ROBOT_VALUES.md §4` ·
`docs/PROJECT_CONTEXT.md §3` · `docs/PROJECT_CONTEXT.md §6` · `docs/PITFALLS.md §3` ·
`docs/PITFALLS.md §5` · `docs/PITFALLS.md §7` · `docs/TEST_GATES.md §1` · `docs/TEST_GATES.md §5` ·
`docs/TEST_GATES.md §7` · `docs/FREEZE_MANIFEST.md §3` · `docs/FREEZE_MANIFEST.md §6` ·
`~/Desktop/개발현황/0719_실차전환_마스터플랜.md §3.1` · `~/Desktop/TEENSY_실차연동_합의사항.md`
