# CURRENT_HANDOFF.md — 지금 할 한 묶음 (매 세션 시작점)

> 작업 완료 시 구현자가 갱신하고 검토자가 사실관계·테스트 결과를 확인한다.
> 이전 묶음 내용은 날짜별 현황으로 보낸다. 항상 이 한 묶음만 유지.

- **동결 기준점**: `212885a8292d1e86677d5beeb9f5358d76fe9b40` = tag **`platform-core-freeze-260724`**
  (annotated, 원격 push 완료). 증거 전량 = `docs/FREEZE_MANIFEST.md`.
  ★ 회귀가 나오면 `git diff platform-core-freeze-260724 --stat` 이 1차 용의선상이다.
- **현재 기준 커밋**: 이 파일과 같은 커밋 (main)
- **현재 단계**: 마스터플랜 5단 완료 → **6단(역할 B V1 합의)은 ⏸ 보류**(역할 B 회의 일정 대기 —
  `MASTER_PLAN.md §1` 6) → 순서 밖 소묶음 **tunnel-bringup-s4 완료**(`MASTER_PLAN.md §7` 예약 9 ✅)
  → 그 묶음이 남긴 갭을 닫는 **후속 소묶음**(예약 10)이 이번 차례다.
- **⏸ 6단 보류 기록**: 준비된 YOLO V1 핸드오프 원문(목표·함정·완료조건·허용 범위 전량)은
  **커밋 `30c5e87` 에 고정 보존**했다. 역할 B 회의가 잡히면 **그 원문을 그대로 되살려 재개**한다
  — 새로 쓰지 말 것. 6단은 7단과 병렬이라 이 보류가 실차 트랙을 막지 않는다. 복원 명령:

```bash
git show 30c5e87:docs/CURRENT_HANDOFF.md            # 원문 확인
git show 30c5e87:docs/CURRENT_HANDOFF.md > docs/CURRENT_HANDOFF.md   # 그대로 되살리기
```
- **직전 완료**: `tunnel_bringup` S4 골격 — 실차 bringup 패키지 신설(명세 6파일 + 조건 기동
  게이트 `readiness_gate`). 고정 시간 지연을 **lifecycle·TF·토픽 신선도 조건 기동**으로 대체하고,
  게이트 실패 시 런치 전체를 내린다(fail-closed). 시뮬 자산 무변경.
  서사·실측 전량 = `0729_현황.md §1`. 반영 상태표 = `docs/REAL_ROBOT_VALUES.md §2`.
  같은 묶음의 실측: pytest **159** · colcon **165·0f·2s** · T자 mission **PASS**(GUIDE 0.12 실측,
  ESCAPED 24s) · 기계 검사 3종 **0건** · flake8 **0건**.
  ★ 실측이 뒤집은 가정 1건 = `0729_현황.md §1.5` (지도 로드 실패해도 slam_toolbox 는 살아서
  map→odom 을 계속 발행 → TF 게이트로는 못 잡는다 → 기동 전 파일 검사 추가).

## 이번 한 묶음 목표 — `tunnel_bringup` 게이트 회귀 하네스 + lint 편입

`MASTER_PLAN.md §7` 예약 10. 직전 묶음이 **의도적으로 남긴 갭을 닫는다.**

**왜 필요한가**: `readiness_gate` 는 실차에서 "무엇이 언제 뜨는지"를 결정하는 **신규 안전
로직**인데 지금 **자동 회귀가 하나도 없다.** 07-29 검증은 전부 사람이 손으로 돌린 것이라,
다음 세션이 이 파일을 고치면 아무도 깨진 것을 모른다. 게다가 이 코드는 **실차에서만
처음 실행**되므로(노트북엔 로봇이 없다) 회귀가 없으면 결함이 R0 현장에서 발견된다 —
가장 비싼 자리다.

**왜 직전 묶음에서 안 했나** (숨기지 않는다): 직전 묶음의 완료조건이 "colcon 기준선 165 무변동"
이었다. 테스트를 추가하면 그 수치가 바뀌므로 같은 묶음에 섞으면 "무엇이 무영향인지"가 흐려진다
(`TEST_GATES.md §7` 의 "동결 커밋에는 수정을 섞지 않는다"와 같은 원칙).

## ★ 함정 (먼저 읽을 것)

1. **Gazebo 없이 도는 하네스여야 한다.** 기존 `tools/test_harness_guards.sh` 가 선례다
   (10케이스, ~105초, Gazebo 불필요). 게이트 검증에 Gazebo 를 끌어들이면 실행 시간이
   분 단위로 늘고 E2E 동시 실행 금지 제약까지 따라붙는다.
2. **가짜 조건을 '주입'해야 한다.** 게이트가 보는 것은 토픽·TF·lifecycle 서비스·액션이다.
   실물 없이 검증하려면 그것들을 흉내 내는 최소 발행자/서버가 필요하다
   (07-29 검증에 쓴 임시 스크래치가 참고 — 계약대로 /odom 50Hz·/imu 100Hz·/scan 10Hz,
   covariance 비-0. 그 파일은 repo 에 없으므로 다시 쓴다).
3. **부정 회귀가 본체다** (`AGENTS.md §3-7`). "조건이 차면 통과한다"보다 **"안 차면 통과 못 한다"**가
   이 게이트의 존재 이유다. 아래 완료조건 2의 음성 케이스를 빼면 하네스가 무의미하다.
4. **live 술어를 지키는 회귀를 반드시 넣을 것.** lifecycle 판정이 latch 로 되돌아가면
   '반쪽 Nav2' 위에서 주행이 시작된다. 07-29 에 초안이 실제로 latch 였다가 자기 반증으로
   잡혔다(`0729_현황.md §1.6`). **응답이 멎은 lifecycle 서버**를 주입해 통과하지 못하는지 본다.
5. **`colcon test` 대상에 넣으면 기준선 수치가 바뀐다** → `docs/TEST_GATES.md §1` 의
   pytest·colcon 수치를 **같은 커밋에서** 갱신한다. 안 하면 `doc_check.sh` 가 FAIL 한다
   (그게 정상 동작이다 — 기억이 아니라 기계가 지키는 구조).
6. **`setup.py` `data_files` 함정 재발 주의** — 테스트를 추가해도 `launch/`·`config/`·`urdf/`
   등록은 그대로 유지할 것 (`PITFALLS.md §3`).

## 완료조건

1. **격리 하네스 신설** — Gazebo 없이 도는 `readiness_gate` 회귀. 위치·이름은 구현자 판단
   (`tools/` 셸 하네스 또는 `src/tunnel_bringup/test/` pytest 중 택1, 근거를 커밋 메시지에).
2. **최소 검증 케이스** (양성 1 + **음성 5** — 음성이 본체):
   - 양성: 토픽·TF·lifecycle·액션 조건이 다 차면 종료코드 **0**
   - 음성 ①: 요구 토픽이 없으면 제한시간 후 **1**
   - 음성 ②: 요구 토픽이 **한 번 오고 끊기면**(신선도 초과) **1**
   - 음성 ③: 정적 TF 에 신선도를 요구하면 **1** (게이트 오사용 검출)
   - 음성 ④: lifecycle 노드가 ACTIVE 가 아니면 **1**
   - 음성 ⑤: **lifecycle 응답이 멎으면** 과거 ACTIVE 결과로 통과하지 **않음** (live 술어 — 함정 4)
3. **런치 레벨 오설정 차단 회귀** — 없는 지도 / 확장자 붙인 지도 / `mission:=true` + waypoints 없음
   3종이 **노드를 0개 띄우고** 종료하는지 (`grep -c "process started"` = 0).
4. **`tunnel_bringup` lint 편입** — ament_copyright·flake8·pep257 테스트 추가 후
   `colcon test --packages-select mission_manager tunnel_sim tunnel_bringup` 로 전량 통과.
5. **기준선 갱신** — `docs/TEST_GATES.md §1` 의 pytest·colcon 수치와 §1 명령줄을 새 값으로.
   `docs/TEST_GATES.md §2` 표에 새 하네스 행 추가(목적·PASS 기준).
6. **시뮬 회귀 무영향 실측** — `python3 -m pytest src/mission_manager/test/ -q` (159 유지) +
   `bash tools/test_harness_guards.sh` (10/10) + `bash tools/mission_e2e.sh` (T자 1회 PASS).
   ⚠ 미션 로직을 안 건드리므로 negative·3goals·abort·쌍굴은 불필요.
7. 문서 동기화 → **한 커밋 + push** → `bash tools/doc_check.sh --after-push` → Codex 검토
   (범위 = `TEST_GATES.md §7` — 신규 테스트 인프라는 표에 없는 변경이라 **fail-closed**,
   검토자가 관련 테스트를 직접 선정한다).

## 허용 파일/범위

- `src/tunnel_bringup/**` (테스트 추가·게이트 보완)
- `tools/` 신규 하네스 1개 (기존 하네스 수정은 이 묶음이 아니다)
- `docs/CURRENT_HANDOFF.md` · `docs/TEST_GATES.md §1`·`§2` · `docs/MASTER_PLAN.md`(§7 예약 10 ✅)
- `~/Desktop/개발현황/0729_현황.md` 다음 절

## 금지 범위

- **`src/tunnel_sim/**` 변경 절대 금지** — 동결 대상(`docs/FREEZE_MANIFEST.md §3` 해시 고정)이자
  T자·쌍굴 회귀의 **살아있는 기준선**이다.
- **`src/mission_manager/**` 변경 금지** — 미션 로직은 이 묶음이 아니다.
- **미실측 값을 채우는 것 금지** — 장착 오프셋·covariance·penalty·RPP 튜닝값은 여전히
  `TODO:` 로 남긴다 (전수 확인 `grep -rn "TODO: " src/tunnel_bringup/`). 하네스가 통과하도록
  숫자를 넣고 싶어지는 자리가 나오면, 그건 하네스를 잘못 설계한 것이다.
- **Jetson 실행·빌드 시도 금지** — 장비가 없다. 노트북 검증까지만.
- `make_map.sh` 실런 금지. Gazebo E2E 동시 실행 금지.
- 실패를 원인 한 줄 분류 없이 재실행 금지 (`AGENTS.md §3-6` · `TEST_GATES.md §5`).

## 보존해야 할 안전 불변조건

- **★ R0 watchdog 리마인더 (잊으면 위험이 조용히 남는다)**: 실차 7단 R0 에서 **cmd_vel watchdog(단절
  0.5s 내 정지)** 실측 결과를 받는 즉시 `FREEZE_MANIFEST.md §6` 의 잔류 cmd_vel 활주 **조건부 수용을
  확정 또는 재개방**한다. 이 항목은 계속 열려 있다.
- **★ 08-15 플랜 B 판정일** (`MASTER_PLAN.md §6`) — 구동부 진척 주 1회 확인, 그날까지 R2 통과 선언이
  없으면 '구동부 지연' 행을 발동한다. **판정을 미루지 않는다.**
- **★ 라이다 장착 높이 미수령** (`MASTER_PLAN.md §7` 예약 11 · `REAL_ROBOT_VALUES.md §4`) —
  기구·센서 담당에게 **재요청이 필요한 사용자 액션**이다. 이것 없이는 R5 지도 제작이 무의미하다
  (스캔이 로봇 엉뚱한 자리에 붙는다). 역할 A 가 코드로 우회할 수 없다.
- E2E cleanup 순서 불변조건(부모 `ros2 launch` 먼저 kill → `pkill -9 -f "lib/nav2[_]"`, 브래킷 트릭 —
  `AGENTS.md §4`)은 회귀 실행 시 그대로 지킨다.

## 완료 판정 + 필수 테스트

**한 문장 완료판정**: "`readiness_gate` 의 통과·미통과가 **Gazebo 없이 자동으로** 검증되고
(양성 1 + 음성 5 + 런치 오설정 3), `tunnel_bringup` 이 `colcon test` 대상에 편입돼
`TEST_GATES.md §1` 기준선이 새 수치로 갱신되며, 시뮬 회귀(pytest 159 · harness 10/10 ·
T자 mission PASS)가 무영향임이 실측된다."

실행 순서 = 완료조건 6 그대로.

## 완료 후 다음 단계

**아래 두 트랙은 외부 대기이며, 풀리는 즉시 이 소묶음보다 우선한다** (서로 독립):
- 역할 B 회의가 잡히면 → **6단 재개**: 위 '⏸ 6단 보류 기록' 의 복원 명령으로 원문을 되살린다
- 구동부 트리거(R2 "3m 직진 3%" 선언)가 오면 → **7단 실차 R0~R8** (`MASTER_PLAN.md §3`).
  R0 실측에서 **cmd_vel watchdog** 결과를 받는 즉시 `FREEZE_MANIFEST.md §6` 조건부 수용을
  확정/재개방. R5 이전에 **라이다 장착 오프셋 실측 반영**이 선행돼야 한다.

## 근거 문서

`docs/MASTER_PLAN.md §3` · `docs/MASTER_PLAN.md §6` · `docs/MASTER_PLAN.md §7` ·
`docs/MASTER_PLAN.md §8` · `docs/REAL_ROBOT_VALUES.md §2` · `docs/REAL_ROBOT_VALUES.md §4` ·
`docs/PITFALLS.md §3` · `docs/TEST_GATES.md §1` · `docs/TEST_GATES.md §2` ·
`docs/TEST_GATES.md §5` · `docs/TEST_GATES.md §7` · `docs/FREEZE_MANIFEST.md §3` ·
`docs/FREEZE_MANIFEST.md §6` · `0729_현황.md §1` ·
`~/Desktop/개발현황/0719_실차전환_마스터플랜.md §3.1` · `~/Desktop/TEENSY_실차연동_합의사항.md`
