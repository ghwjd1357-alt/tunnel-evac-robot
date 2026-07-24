# CURRENT_HANDOFF.md — 지금 할 한 묶음 (매 세션 시작점)

> 작업 완료 시 구현자가 갱신하고 검토자가 사실관계·테스트 결과를 확인한다.
> 이전 묶음 내용은 날짜별 현황으로 보낸다. 항상 이 한 묶음만 유지.

- **현재 기준 커밋**: 이 파일과 같은 커밋 (main — 구조 분리 3종 전부 독립 검토 통과, 07-24)
- **현재 단계**: 마스터플랜 12단계 중 **5단 platform-core-freeze 착수** (동결 게이트 + release tag).
  4단 구조 분리 3종은 전부 Codex 독립 검토 통과로 **종료**됐다.
- **직전 완료**: E2E 공통 하네스 3/3 (`4fe060d`) — **0723검토 §8 에서 P0/P1/P2 0건 기술 통과**
  (`CODEX 현황/0723검토현황.md §8`: 셸 5파일 `bash -n`, 공통 함수 표적 프로브, pytest 159,
  colcon 165, `doc_check --strict`/`--after-push` 전부 PASS). 구조 분리 3종 종결 =
  SpeedManager `f94da44`(+보완 6회) / GoalManager `9a03d1f`(+P1 보완 2회, §7 통과) /
  하네스 `4fe060d`(§8 통과). 구현 기록 = `0723_현황.md §10`.

- **진행 상황 (07-24)**: 완료조건 **1·2·3 완료** — 동결 게이트 전량 PASS(pytest 159 / colcon 165 /
  negative·3goals 0.142m·mission T자·abort 0.0m·**쌍굴** 전부 PASS) + 지도 hash MATCH +
  `docs/FREEZE_MANIFEST.md` 작성. 증거 = `0723_현황.md §11`.
  ★ `abort_e2e` 최초 1회 실패는 **잔류 cmd_vel 활주**로 근인 규명(미션 코드 무결) — 알려진 시뮬
  한계로 공개 기록, **R0 watchdog 실측 통과 전제의 조건부 수용** (`FREEZE_MANIFEST.md §6`).
  **남은 것 = 완료조건 5(Codex 동결 판정) → 6(태그).**

## 이번 한 묶음 목표 — platform-core-freeze (동결 게이트 + release tag)

"주행·정지·목표·속도·지도·고장 처리의 **기반**이 완성됐다"를 선언하고, 실차에서 문제가 났을 때
**되돌아갈 기준점**을 못 박는 단계다 (`MASTER_PLAN.md §1` 5단 · 정의 `PROJECT_CONTEXT.md §6` ·
근거 `0719_실차전환_마스터플랜.md §7.1`).

★ 이 묶음은 **코드를 더 쓰는 묶음이 아니라 증거를 모아 태그로 고정하는 묶음**이다.
동결 후 platform-core 코드는 **실차 이슈 대응 외에는 손대지 않는다** (미션 시나리오는 별도 트랙).

## 완료조건

1. **동결 게이트 전량 PASS** — `TEST_GATES.md §1` 전량(pytest · colcon test · colcon
   test-result · E2E 4종) **+ 쌍굴 mission**(`bash tools/mission_e2e.sh twin`) **+ 지도 승격
   evidence**. 이 3종 묶음이 `TEST_GATES.md §7` 표의 **동결 게이트** 행이다.
   실패 시 `TEST_GATES.md §5` 로 **원인 한 줄 분류 전 재실행 금지**.
2. **지도 evidence 는 실런 없이 재확인** — 현재 정본 `maps/tunnel_localization.posegraph` ·
   `maps/tunnel_localization.data` 의 sha256 이 `maps/tunnel_localization.manifest.txt` 기록과
   일치하는지 대조하고, 그 대조 결과를 evidence 로 남긴다(불일치 = **동결 중단**, 원인 규명 먼저).
   ⚠ `make_map.sh` 실런은 이 묶음에 없다 — 새 지도 제작이 필요하다는 판단이 서면 **사용자 명시
   승인**을 받아 별도 묶음으로 분리한다 (`AGENTS.md §5` · `TEST_GATES.md §4`).
3. ✅ **hash manifest 신설** = `docs/FREEZE_MANIFEST.md` — 최소 항목:
   태그 대상 커밋 해시 / 지도 정본 sha256(§2 대조값 + `tunnel_map_loc`·`twin_*` 포함) /
   설정·URDF sha256(`src/tunnel_sim/config/*.yaml` · `src/tunnel_sim/urdf/robot.urdf`) /
   **`src/sllidar_ros2` upstream commit** / 게이트 결과 수치.
   ★ `src/sllidar_ros2` 는 `.gitignore` 대상이라 **git 스냅샷에 없다** — 해시를 손으로 남기지
   않으면 동결 시점 구성을 재현할 수 없다 (0719 종료 게이트가 "sllidar commit"을 명시한 이유).
4. 기록·문서 동기화 → **한 커밋 + push** (= 검토 대상 커밋): `0723_현황.md` 다음 절에 게이트
   수치 전량·남은 위험·구현자 주장, `MASTER_PLAN.md §1` 5단 ✅, `TEST_GATES.md §1` 기준선,
   `bash tools/doc_check.sh --strict` → 커밋+push → `bash tools/doc_check.sh --after-push`.
5. **Codex 독립 검토(동결 판정) 통과** — 검토 범위는 `TEST_GATES.md §7` 동결 게이트 행(전량 +
   쌍굴 + 지도 승격 evidence). 이 묶음은 검토 통과로만 닫힌다.
6. 통과 후 **annotated tag** `platform-core-freeze-YYMMDD`(실제 태그일) 를 그 커밋에 붙이고
   `git push --tags`. 태그 메시지 = 게이트 결과 요약 한 화면.
   ★ 태그는 **검토 통과 뒤에** 붙인다 — 불승인 커밋에 동결 태그가 남으면 기준점이 오염된다.

## 허용 파일/범위

- `docs/FREEZE_MANIFEST.md`(신설) · `docs/MASTER_PLAN.md` · `docs/TEST_GATES.md` ·
  `docs/CURRENT_HANDOFF.md` · `docs/PROJECT_CONTEXT.md`(동결 표기) · `~/Desktop/개발현황/0723_현황.md`
- git tag 생성/push (코드 변경 없음)

## 금지 범위

- **코드·설정 동작 변경 금지** — 게이트 중 결함이 나오면 **별도 묶음**으로 분리해 고친 뒤 다시
  게이트를 돌린다. 동결 커밋에 수정을 섞지 않는다(동결의 의미가 사라진다).
- `make_map.sh` 실런 금지(완료조건 2), Nav2 파라미터·costmap·planner·지도 자산 변경 금지.
- 게이트 실패를 원인 분류 없이 재실행 금지(flake 은폐), Gazebo E2E 동시 실행 금지.

## 보존해야 할 안전 불변조건

- E2E cleanup 은 부모(`ros2 launch`)부터 kill → `pkill -9 -f "lib/nav2[_]"`, 브래킷 트릭
  (`AGENTS.md §4`). 좀비 bt_navigator 가 goal 을 가로채면 게이트 결과 자체가 무효다.
- 각 E2E 판정 기준(abort 실정지 ≤0.10m · 3goals ≤0.3m · negative 금지 3종 ABORTED ·
  mission ESCAPED)은 동결 게이트에서도 그대로다 — 통과시키려고 완화하지 않는다.
- 지도 정본은 `map_promote.sh` fail-closed transaction 밖에서 손으로 교체하지 않는다.

## 완료 판정 + 필수 테스트

**한 문장 완료판정**: "`TEST_GATES.md §1` 전량 + 쌍굴 mission + 지도 hash 대조가 전부 PASS 한
커밋에, hash manifest 와 `platform-core-freeze-*` annotated tag 가 붙어 원격에 push 되어 있다."

실행 순서 = `TEST_GATES.md §1` 그대로 + `bash tools/mission_e2e.sh twin` + 지도 sha256 대조.
E2E 는 **동시 실행 금지** — 한 스크립트 완전 종료 후 다음.

## 완료 후 다음 단계

6단 **역할 B V1 최소 계약 세부 확정**(필드·주기·실패 표현 — 병렬 진행 가능, `PROJECT_CONTEXT.md §4`)
→ 7단 **실차 R0~R8**(`MASTER_PLAN.md §3`, 트리거 = 구동부 "3m 직진 오차 3% 이내" 선언).
동결 후 트랙이 갈리므로, 동결 커밋 해시를 다음 핸드오프 첫 줄에 남긴다.

## 근거 문서

`docs/MASTER_PLAN.md §1` · `docs/MASTER_PLAN.md §2` · `docs/TEST_GATES.md §1` ·
`docs/TEST_GATES.md §7` · `docs/TEST_GATES.md §4` · `docs/PROJECT_CONTEXT.md §6` ·
`~/Desktop/개발현황/0719_실차전환_마스터플랜.md §7.1` ·
`~/Desktop/개발현황/CODEX 현황/0723검토현황.md §8` · `~/Desktop/개발현황/0723_현황.md §10`
