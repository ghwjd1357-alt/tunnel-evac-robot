# CURRENT_HANDOFF.md — 지금 할 한 묶음 (매 세션 시작점)

> 작업 완료 시 구현자가 갱신하고 검토자가 사실관계·테스트 결과를 확인한다.
> 이전 묶음 내용은 날짜별 현황으로 보낸다. 항상 이 한 묶음만 유지.

- **동결 기준점**: `212885a8292d1e86677d5beeb9f5358d76fe9b40` = tag **`platform-core-freeze-260724`**
  (annotated, 원격 push 완료). 증거 전량 = `docs/FREEZE_MANIFEST.md`.
  ★ 앞으로 회귀가 나오면 `git diff platform-core-freeze-260724 --stat` 이 1차 용의선상이다.
- **현재 기준 커밋**: 이 파일과 같은 커밋 (main)
- **현재 단계**: 마스터플랜 5단 **완료** → 순서 밖 소묶음 **e2e-harness-fix**
  (`MASTER_PLAN.md §7` 예약 **6·7**). 6단(역할 B V1 계약)·7단(실차 R0~R8)은 이 묶음 뒤.
- **직전 완료**: platform-core-freeze — 동결 게이트 전량 PASS + hash manifest + **Codex 동결 판정
  통과**(`CODEX 현황/0723검토현황.md §9`, P0/P1/P2 0건) + 태그 부착.
  구현 기록 = `0723_현황.md §11`·`0723_현황.md §12`.

## 이번 한 묶음 목표 — e2e-harness-fix (쌍굴 4회 중 2회를 깨뜨린 하네스 결함 2건)

동결 게이트에서 **쌍굴 mission 이 4회 중 2회 실패**했는데, **두 실패 모두 미션 코드가 아니라
E2E 하네스 결함**이었다 (`FREEZE_MANIFEST.md §8` 에 전량 공개 기록). 그 2건을 고친다.

| # | 결함 | 실측 | 근거 |
|---|---|---|---|
| ⑦ | `mission_e2e.sh` 의 `ros2 param get` 에 **타임아웃 가드 없음** | **13분 27초 무한 행**(쌍굴 3회차) | `0723_현황.md §11.5.2` |
| ⑧ | SEARCH_BACK **90초 예산**이 실측 분포 대비 빠듯 + `wait_state` 폴링 race | ≈3초 차 경계 실패(쌍굴 2회차) | `0723_현황.md §11.5.1` |

★ **이 두 결함은 쌍굴 전용이 아니다** — `mission_e2e.sh` 는 T자와 **같은 파일**이다.
T자가 통과해 온 것은 경로가 짧아 여유가 있었을 뿐이다.

★ **왜 지금인가**: 게이트가 흔들리면 다음 트랙(실차 R0~R8·mission-logic-RC)에서 실패가 나올
때마다 "코드 회귀인가 하네스인가"를 매번 손으로 가려야 한다. 게이트의 신뢰도는 그 뒤 모든
판단의 전제다.

## ★ 동결과의 관계 (먼저 읽을 것 — 이 묶음의 유일한 함정)

`PROJECT_CONTEXT.md §6` 의 platform-core 정의에는 **"테스트 인프라"가 포함**된다. 그러니
"동결 후 platform-core 는 손대지 않는다"와 이 묶음은 형식상 충돌하는 것처럼 보인다. 정리:

1. **이 수리는 동결 위반이 아니다** — 동결 증거 자체(`FREEZE_MANIFEST.md §8`)가 이 2건을
   "동결 범위 밖 후속 = `MASTER_PLAN.md §7` 예약 6·7"로 **미리 분리 선언**했고, Codex 도
   §9.5 에서 "수리 완료로 승격하지 않는다"며 예약 상태를 확인했다. 예정된 후속이다.
2. **그러나 ⑧ 은 판정 기준(PASS 조건) 변경이다.** 태그 `platform-core-freeze-260724` 가 담은
   게이트 수치는 **옛 기준으로 얻은 것**이므로, 기준을 바꾸면 그 사실을 동결 증거에 남기지 않는
   한 "동결 시점 수치"와 "현재 기준"이 조용히 어긋난다 → 완료조건 5.
3. **동결된 런타임 코드는 여전히 못 건드린다** — `src/mission_manager/` · `src/tunnel_sim/` ·
   설정 · 지도 (금지 범위).

## 완료조건

1. **⑦ 타임아웃 가드** — `mission_e2e.sh` 의 `ros2 param get /controller_server
   FollowPath.desired_linear_vel` 이 유한 시간에 끝난다. 같은 파일의 `wait_state`(빈 읽기 5연속 시
   daemon 재시작)·readiness(`timeout 8`)가 이미 쓰는 방어를 **이 줄에도 적용**한다.
   ★ 한 문장 완료판정: "`ros2 param get` 이 응답하지 않아도 스크립트는 **정해진 상한 안에**
   `TEST_GATES.md §5 ③` 계열로 **분류된 메시지**를 남기고 끝난다 (무한 대기 0)."
   ⚠ 이건 `lib_e2e.sh` 로 뽑을지 `mission_e2e.sh` 안에서 처리할지 구현자 선택 — 단, 같은 위험을
   가진 다른 `ros2` CLI 호출이 더 있는지 **한 번 훑고** 결과를 기록에 남긴다(있으면 같이, 없으면 없다고).
2. **⑧-a 폴링 race 제거 (구조 결함 — 기준 완화 아님)** — `wait_state` 는 3초 간격 폴링이라
   마지막 확인이 t=87s 이고, 루프 종료 후 `fail` 이 상태를 **한 번 더** 읽는다. 그래서
   "타임아웃인데 마지막 상태는 목표 상태"라는 자기모순 메시지가 나왔다.
   ★ 완료판정: "예산 내에 목표 상태에 도달했으면 **폴링 격자와 무관하게 PASS**, 도달하지
   못했으면 FAIL — 두 경우의 메시지가 서로 모순되지 않는다."
3. **⑧-b 90초 예산 재산정 (판정 기준 — 근거 수치 필수)** — 현재 실측 표본은
   SEARCH_BACK 도달 **9s / ≈90s**, GATHER 도달 **48s / 126s / 48s** 뿐이다(`FREEZE_MANIFEST.md §8`).
   ⚠ **표본이 부족한 채로 숫자만 올리면 그건 기준 완화다.** 새 예산은 (a) 추가 실측 분포와
   (b) "무엇을 놓치지 않기 위한 상한인가"라는 근거 한 문장을 함께 남겨야 한다.
   ★ 완료판정: "새 예산값과 그 근거(관측 분포 + 상한의 의미)가 `TEST_GATES.md §2` 에 적혀 있고,
   **도달 실패 시 여전히 FAIL 한다**는 것이 부정 회귀로 관찰됐다."
4. **회귀 — 하네스를 고쳤으므로 하네스로 증명한다**:
   - `python3 -m pytest src/mission_manager/test/ -q` + `colcon test` + `colcon test-result --verbose` (무조건)
   - `bash tools/mission_e2e.sh` (T자) — **같은 파일을 고쳤으므로 필수**
   - `bash tools/mission_e2e.sh twin` (쌍굴) — ⚠ **최소 2회.** 1회 PASS 로 판단 금지
     (변동폭이 큰 것이 이 결함의 본질이다 — `FREEZE_MANIFEST.md §8`)
   - `bash -n` (셸 문법) + `bash tools/doc_check.sh --strict`
   - ⚠ **역회귀 앵커** (`AGENTS.md §3-5`): T자·쌍굴 두 실배포 설정이 **둘 다** 통과해야 한다.
     한쪽만 맞춘 예산은 다른 쪽을 깨뜨린 적이 있다.
5. **동결 증거 정합성 갱신** — `FREEZE_MANIFEST.md §8` 에 "예약 6·7 수리 완료" + **바뀐 판정
   기준(옛 90s → 새 값)** 을 명시한다. 동결 태그의 수치는 **옛 기준으로 얻은 것**임을 한 줄로 남겨
   나중에 두 기준이 헷갈리지 않게 한다. `MASTER_PLAN.md §7` 예약 6·7 에는 ✅ 표시.
6. 문서 동기화 → **한 커밋 + push** → `bash tools/doc_check.sh --after-push` → **Codex 독립 검토**.
   검토 범위 = `TEST_GATES.md §7` **"셸 도구" 행**이지만, 이번엔 **판정 기준 자체가 바뀌므로**
   검토자가 T자·쌍굴을 직접 돌리는 것이 타당하다 — 그 요청을 기록에 남긴다.

## 허용 파일/범위

- `tools/mission_e2e.sh` · `tools/lib_e2e.sh` (필요 시)
- `docs/TEST_GATES.md` · `docs/MASTER_PLAN.md` · `docs/FREEZE_MANIFEST.md` · `docs/CURRENT_HANDOFF.md`
- `~/Desktop/개발현황/0723_현황.md` 다음 절

## 금지 범위

- **동결된 런타임 변경 금지** — `src/mission_manager/` · `src/tunnel_sim/` · 설정 yaml · URDF ·
  Nav2 파라미터 · 지도 자산. 미션 코드에 손대야 한다는 판단이 서면 **그 자체가 별도 묶음**이다
  (동결 이후 platform-core 변경은 실차 이슈 대응 한정 — `PROJECT_CONTEXT.md §6`).
- **다른 E2E 의 판정 기준 변경 금지** — `abort_e2e` 실정지 ≤0.10m · `regression_3goals` ≤0.3m ·
  `regression_negative` 금지 3종 ABORTED 는 이 묶음에서 건드리지 않는다.
- **통과시키려고 예산만 올리는 것 금지** (완료조건 3 — 근거 없는 상향은 기준 완화다).
- `make_map.sh` 실런 금지. Gazebo E2E 동시 실행 금지.
- 실패를 원인 한 줄 분류 없이 재실행 금지 (`AGENTS.md §3-6` · `TEST_GATES.md §5`).

## 보존해야 할 안전 불변조건

- E2E cleanup 은 부모(`ros2 launch`)부터 kill → `pkill -9 -f "lib/nav2[_]"`, 브래킷 트릭
  (`AGENTS.md §4`). 좀비 bt_navigator 가 goal 을 가로채면 게이트 결과 자체가 무효다.
- `mission_e2e` 의 PASS 조건 본체(GUIDE 0.12 실측 → SEARCH_BACK → 재발견 → ESCAPED)는 불변.
  바꾸는 것은 **⑧ 의 제한시간과 폴링 방식뿐**이며, "무엇을 확인하는가"는 그대로다.
- 타임아웃 가드를 넣을 때 **실패를 성공으로 삼키지 않는다** — 가드에 걸리면 명확히 FAIL 하거나
  분류된 재시도를 하고, 조용한 통과는 만들지 않는다.

## 완료 판정 + 필수 테스트

**한 문장 완료판정**: "`ros2 param get` 이 무한 대기하지 않고, SEARCH_BACK 판정이 폴링 격자에
좌우되지 않으며, 새 예산의 근거가 문서에 남은 상태에서 **T자 1회 + 쌍굴 2회가 연속 PASS** 하고
부정 회귀(도달 실패 시 FAIL)가 관찰된다."

실행 순서 = 완료조건 4 그대로. E2E 는 **동시 실행 금지** — 한 스크립트 완전 종료 후 다음.

## 완료 후 다음 단계

6단 **역할 B V1 최소 계약 세부 확정**(필드·주기·실패 표현 — `PROJECT_CONTEXT.md §4.1`, 병렬 가능)
→ 7단 **실차 R0~R8**(`MASTER_PLAN.md §3`, 트리거 = 구동부 "3m 직진 오차 3% 이내" 선언).
★ R0 에서 **cmd_vel watchdog(단절 0.5s 내 정지)** 실측 결과를 받는 즉시
`FREEZE_MANIFEST.md §6` 의 조건부 수용을 **확정 또는 재개방**한다 — 잊으면 위험이 조용히 남는다.

## 근거 문서

`docs/MASTER_PLAN.md §7` · `docs/MASTER_PLAN.md §3` · `docs/TEST_GATES.md §2` ·
`docs/TEST_GATES.md §5` · `docs/TEST_GATES.md §7` · `docs/FREEZE_MANIFEST.md §8` ·
`docs/FREEZE_MANIFEST.md §6` · `docs/PROJECT_CONTEXT.md §6` ·
`~/Desktop/개발현황/0723_현황.md §11.5` · `~/Desktop/개발현황/0723_현황.md §12` ·
`~/Desktop/개발현황/CODEX 현황/0723검토현황.md §9`
