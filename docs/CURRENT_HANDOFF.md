# CURRENT_HANDOFF.md — 지금 할 한 묶음 (매 세션 시작점)

> 작업 완료 시 구현자가 갱신하고 검토자가 사실관계·테스트 결과를 확인한다.
> 이전 묶음 내용은 날짜별 현황으로 보낸다. 항상 이 한 묶음만 유지.

- **동결 기준점**: `212885a8292d1e86677d5beeb9f5358d76fe9b40` = tag **`platform-core-freeze-260724`**
  (annotated, 원격 push 완료). 증거 전량 = `docs/FREEZE_MANIFEST.md`.
  ★ 회귀가 나오면 `git diff platform-core-freeze-260724 --stat` 이 1차 용의선상이다.
- **현재 기준 커밋**: 이 파일과 같은 커밋 (main)
- **현재 단계**: 마스터플랜 5단 완료 + 순서 밖 소묶음 **e2e-harness-fix 완료** → 다음 = 마스터플랜
  **6단 역할 B V1 계약 세부 확정** (`MASTER_PLAN.md §1` 6, 실차 7단과 **병렬 가능**).
- **직전 완료**: **e2e-harness-fix** (마스터플랜 §7 예약 6·7) — 쌍굴 하네스 결함 2건(⑦ `ros2 param get`
  무한 행 · ⑧ 폴링 race + 90s 예산) 수리. **런타임 코드 무변경, 셸 하네스만**. 검토에서 나온 경계를
  재현·보완했다:
  1차 `0eb285c` → Codex `§14 P1` 불승인(유한 상한 벽시계 미보장) → 벽시계 deadline 보완 `d70bbaf`
  → Codex `§15 P1` **재불승인**(SIGTERM 무시 시 무한 행 · daemon kick 예산 밖) → **hard-kill 재보완**.
  `853ea7a` → Codex `§16 P1` 불승인(mission alarm·stop·follow 3곳이 일반 timeout으로 남음) →
  **사용자 명시 예외로 이번 세션 Codex가 최소 보완**. 최종 핵심: mission E2E의 유한 대기 전부를 공통
  `hard_timeout`(`timeout --kill-after=2`, TERM 무시도 SIGKILL)으로 통일 · `wait_state` daemon kick은
  **남은 예산 배분**(부족 시 복구 생략, deadline FAIL). `read_param_float` hard 상한 = **34s**(정상 ≈26s).
  격리 테스트 `tools/test_harness_guards.sh` **10케이스**(§16 topic-pub TERM 무시·정상 역회귀 포함).
  이번 최종 실측: 격리 **10/10** · pytest **159** · colcon **165·0f·2s** · T자 PASS(GATHER 15s,
  SEARCH_BACK 14s, ESCAPED 22s) · 쌍굴 PASS(GATHER 76s, SEARCH_BACK 14s, ESCAPED 164s).
  SEARCH_BACK 예산 **90s→180s(벽시계)** 재산정(근거 = `TEST_GATES.md §2`, 판정기준 변경 =
  `FREEZE_MANIFEST.md §8.1`~`§8.3`). 구현·검증 기록 = `0723_현황.md §15.7`.
  ⚠ 구현자=검토자 예외는 **이번 세션 사용자 명시 승인에 한정**한다. §16이 허용된 연장 검토였으므로
  추가 Codex 검토 루프는 열지 않고 사용자 최종 승인으로 닫는다.

## 이번 한 묶음 목표 — 역할 B `/detections` V1 최소 계약 세부 확정

`PROJECT_CONTEXT.md §4.1` 이 책임경계((b) camera-frame 3D)와 V1 필드 골격은 확정했다. 남은 것은
**세부 3종**이다: ① 각 필드의 정확한 타입·단위·좌표계(특히 `position` 의 optical frame 축 방향과
`stamp` 기준), ② **발행 주기·QoS**(누락·지연 허용치), ③ **실패·불확실 표현**(탐지 없음 / 저신뢰 /
depth 실패를 어떻게 신호하는가 — 빈 배열 vs 명시 플래그). 이 셋을 계약 정본
(`~/Desktop/YOLO_탐지연동_합의사항.md`)에 못 박고, `.msg` 로 고정한 뒤, **깡통 퍼블리셔 + 수신 funnel
스켈레톤**으로 왕복을 검증한다.

★ **가짜 detection 금지 원칙**(`MASTER_PLAN.md §8`): 계약과 **깡통 퍼블리셔만** 만든다. map 좌표 생성·
검증(Perception Adapter)의 실제 로직은 이 묶음이 아니다 — 계약 왕복이 되는지까지만.

★ **병렬성**: 이 묶음은 실차 7단(R0~R8)과 독립이다. 순서 강제 없음 — 사용자가 트랙을 고른다.

## ★ 함정 (먼저 읽을 것)

1. **`.msg` 필드 = 타입 = 양측 동시 리빌드**(`PROJECT_CONTEXT.md §4.1`). V1 은 **최소로 고정**하고
   확장은 V2 별도 메시지다. "혹시 몰라 필드 추가"가 가장 비싼 실수다.
2. **역할 B 협의 의존** — 주기·실패 표현은 역할 B 측 YOLO 파이프라인 능력에 달려 있다. 구현자가
   단독으로 확정할 수 없는 항목은 **가정을 명시**하고 계약 문서에 "역할 B 확인 필요"로 남긴다.
3. **frame·stamp 규약** — `stamp=촬영시각`, `frame=camera optical frame`. map 변환은 Adapter 책임이라
   이 계약은 camera frame 까지만 책임진다(`PROJECT_CONTEXT.md §4.1`).

## 완료조건

1. 계약 세부 3종(필드 타입·단위·좌표계 / 주기·QoS / 실패·불확실 표현)이
   `~/Desktop/YOLO_탐지연동_합의사항.md` 에 확정 기록된다. 구현자 단독 확정 불가 항목은 **가정 + "역할
   B 확인 필요"** 로 명시(빈칸 금지).
2. `.msg` 정의가 계약과 일치하게 고정되고 빌드된다(`colcon build`). 필드 추가는 V2 로 미룬다.
3. **깡통 퍼블리셔 + 수신 funnel 스켈레톤**으로 왕복 검증 — 발행→수신 콜백 1개→내부 dict 까지.
   가짜 map 좌표·가짜 인식 로직은 넣지 않는다(계약 왕복만).
4. 회귀: `python3 -m pytest src/mission_manager/test/ -q` + `colcon test` + `colcon test-result --verbose`.
   새 msg·수신 funnel 의 단위 테스트(파싱·빈 배열·저신뢰 분기) 추가. E2E 는 이 묶음이 미션 상태머신을
   바꾸지 않으므로 해당 없음(바꾼다면 그 자체가 별도 묶음).
5. 문서 동기화 → **한 커밋 + push** → `bash tools/doc_check.sh --after-push` → Codex 검토
   (범위 = `TEST_GATES.md §7` — msg·wiring 은 표에 없는 런타임 변경이라 **fail-closed**, 검토자가 관련
   테스트를 직접 선정한다).

## 허용 파일/범위

- 새 `.msg` 패키지/정의 · 수신 funnel(Perception Adapter 스켈레톤) · 깡통 퍼블리셔 · 그 단위 테스트
- 계약 정본 `~/Desktop/YOLO_탐지연동_합의사항.md`
- `docs/PROJECT_CONTEXT.md §4.1`(계약 세부 반영) · `docs/CURRENT_HANDOFF.md` · `docs/MASTER_PLAN.md`(§1 6단 ✅)
- `~/Desktop/개발현황/` 다음 절

## 금지 범위

- **동결된 런타임 변경 금지** — `src/mission_manager/` 상태머신 로직 · `src/tunnel_sim/` · 설정 yaml ·
  URDF · Nav2 파라미터 · 지도 자산 (동결 이후 platform-core 변경은 실차 이슈 대응 한정 —
  `PROJECT_CONTEXT.md §6`). 미션 상태머신에 손대야 하면 그 자체가 별도 묶음(9단 FSM 순수화).
- **가짜 detection/인식 로직 금지** — 계약 + 깡통 퍼블리셔만(`MASTER_PLAN.md §8`).
- `make_map.sh` 실런 금지. Gazebo E2E 동시 실행 금지.
- 실패를 원인 한 줄 분류 없이 재실행 금지 (`AGENTS.md §3-6` · `TEST_GATES.md §5`).

## 보존해야 할 안전 불변조건

- **★ R0 watchdog 리마인더 (잊으면 위험이 조용히 남는다)**: 실차 7단 R0 에서 **cmd_vel watchdog(단절
  0.5s 내 정지)** 실측 결과를 받는 즉시 `FREEZE_MANIFEST.md §6` 의 잔류 cmd_vel 활주 **조건부 수용을
  확정 또는 재개방**한다. 이 묶음(6단)이 R0 보다 먼저 끝나도 이 항목은 계속 열려 있다.
- E2E cleanup 순서 불변조건(부모 `ros2 launch` 먼저 kill → `pkill -9 -f "lib/nav2[_]"`, 브래킷 트릭 —
  `AGENTS.md §4`)은 이 묶음이 E2E 를 건드리지 않아도 회귀 실행 시 그대로 지킨다.

## 완료 판정 + 필수 테스트

**한 문장 완료판정**: "V1 계약 세부 3종이 정본에 확정되고 `.msg` 로 고정되며, 깡통 퍼블리셔→수신
funnel 왕복이 단위 테스트로 검증되고, pytest·colcon 전량 통과한다."

실행 순서 = 완료조건 4 그대로. 이 묶음은 미션 E2E 를 바꾸지 않으므로 Gazebo E2E 는 불필요(바꾸면 별도 묶음).

## 완료 후 다음 단계

7단 **실차 R0~R8**(`MASTER_PLAN.md §3`, 트리거 = 구동부 "3m 직진 오차 3% 이내" 선언) — 6단과 병렬.
R0 실측에서 위 **cmd_vel watchdog** 결과를 받는 즉시 `FREEZE_MANIFEST.md §6` 조건부 수용을 확정/재개방.

## 근거 문서

`docs/PROJECT_CONTEXT.md §4.1` · `docs/PROJECT_CONTEXT.md §6` · `docs/MASTER_PLAN.md §1` ·
`docs/MASTER_PLAN.md §3` · `docs/MASTER_PLAN.md §8` · `docs/TEST_GATES.md §2` ·
`docs/TEST_GATES.md §5` · `docs/TEST_GATES.md §7` · `docs/FREEZE_MANIFEST.md §6` ·
`docs/FREEZE_MANIFEST.md §8.1` · `~/Desktop/개발현황/0723_현황.md §15` ·
`~/Desktop/YOLO_탐지연동_합의사항.md`
