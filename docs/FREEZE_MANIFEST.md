# FREEZE_MANIFEST.md — platform-core-freeze 동결 증거

> 동결의 **의미** = `PROJECT_CONTEXT.md §6` (동결 3단) · **절차** = `MASTER_PLAN.md §1` 5단.
> 이 파일은 "무엇을 어떤 상태로 얼어붙였는가"의 기계적 증거다. 서사·판단 근거는
> `~/Desktop/개발현황/0723_현황.md §11`, 독립 검토는 `CODEX 현황/0723검토현황.md`.

## 1. 동결 대상

| 항목 | 값 |
|---|---|
| 게이트 실행 기준 커밋 | `a7b9da825b168327a88acca844618bb82a4acfb0` (작업트리 깨끗 — 게이트 중 코드 변경 0) |
| **동결 기준점 커밋** ★ | `212885a8292d1e86677d5beeb9f5358d76fe9b40` (이 manifest 최종본 = Codex 승인 대상) |
| release tag | ✅ **`platform-core-freeze-260724`** — annotated, `212885a` 에 부착 + 원격 push 완료 (2026-07-24) |
| 게이트 실행일 | 2026-07-24 |
| 실행 환경 | Ubuntu 22.04 · ROS2 Humble · Gazebo Classic 11.10.2 · 전용 시뮬 PC(노트북 `minwoo`) |

★ **태그는 검토 통과 뒤에 붙인다** — 불승인 커밋에 동결 태그가 남으면 기준점이 오염되기
때문이다. 실제 순서 = 동결 게이트 전량 PASS → Codex 동결 판정(§9) → 태그 부착 → 이 문서 갱신.
경위 = `~/Desktop/개발현황/0723_현황.md §12.3` (불변 역사).

⚠ **이 문서는 영구 증거다** — 묶음마다 통째로 교체되는 작업 정본의 내용을 근거로 인용하지
않는다. 당시 원문이 필요하면 아래처럼 **커밋을 고정**해 읽는다 (07-24 Codex §10 P2).

**되돌아가는 법** (실차에서 문제가 났을 때의 기준점 — 이게 동결의 존재 이유다):

```bash
git show platform-core-freeze-260724          # 게이트 결과 요약 + 조건부 위험이 태그 메시지에 있다
git diff platform-core-freeze-260724 --stat   # 동결 이후 무엇이 바뀌었나
git checkout platform-core-freeze-260724      # 그 시점 소스로 복귀 (detached HEAD — 작업은 브랜치 파서)
git show 212885a:docs/CURRENT_HANDOFF.md      # 동결 당시의 완료조건·금지 범위 원문 (커밋 고정)
```

⚠ `src/sllidar_ros2` 는 git 밖이라 checkout 으로 안 돌아온다 → §4 의 재획득 명령을 같이 쓴다.

## 2. 지도 정본 sha256

`make_map.sh` 는 실행하지 않았다. 정본 2파일은 `maps/tunnel_localization.manifest.txt` 의
기록값과 **일치 확인**(MATCH)했으며, git 작업트리상 변경도 0이다.

| 파일 | sha256 |
|---|---|
| `maps/tunnel_localization.posegraph` | `a1010ff47ed5931871284f5d47ac1864e61729e574d4e0043af467a8756054cc` |
| `maps/tunnel_localization.data` | `e22f8513355d2ea63eeca64a1a47da9bfa2073889ed0b0bd44dac3fba4f0557d` |
| `maps/tunnel_map_loc.pgm` | `8228eea8b77ecabededc1ddd53e77746e35d8158d716bcea4510911f627762a9` |
| `maps/tunnel_map_loc.yaml` | `ae3e0768175d4efb8796b118f2e1d87d4d482c1acdfdc70ed6e0250a40bb696a` |
| `maps/twin_localization.posegraph` | `6e4a96f2c96c1bbea2fc95027bd8e0fdf85a0c597e4d88a656d10ff52be28f96` |
| `maps/twin_localization.data` | `5108e6645d21a256d7aba3d752ba874dd0f66ec4956d61dccca481699e8b6d44` |
| `maps/twin_map_loc.pgm` | `d3009251ac8c29efe1cb3f6aaface396300066ff844c94144048b97877b92ce9` |
| `maps/twin_map_loc.yaml` | `ea2c9eccd537dc919e8280555eafb12ef5bf02875321b8841cf12cdb759def56` |

## 3. 설정·URDF sha256

| 파일 | sha256 |
|---|---|
| `src/tunnel_sim/config/nav2_params.yaml` | `73a78a4397d80a36ac0957f0ebc708a9a471832a96b3129eff681fd89ad040bd` |
| `src/tunnel_sim/config/bt_nav_to_pose_backup_first.xml` | `fcdd13a0ff6148780784c7b1925d5b789a58627e94a352470fd5a4a53233546c` |
| `src/tunnel_sim/config/slam_params.yaml` | `6abf1decb6761a3ab2228a0a1aa6aa724722e048737ebd20e2bed8af644b12a7` |
| `src/tunnel_sim/config/slam_params_localization.yaml` | `1583773ae1a912ecb3edc6187378aa644d74c88d3618cbc07aafbf63afaac0b0` |
| `src/tunnel_sim/config/slam_params_localization_twin.yaml` | `b389d6d1e54b92ffd8a4be40725be9db8852137a2e92c22a427182b43a1ba687` |
| `src/tunnel_sim/config/slam_params_realodom.yaml` | `8a55b4f28ef2b6b522effd5bd8df60dd59c2beaab96f2f7eddfa9b78b0c482b8` |
| `src/tunnel_sim/config/ekf.yaml` | `4684e0eaca24df4eab8e49e00f5156e5a392bb212e3f8180c16cb23c11f3dd1d` |
| `src/tunnel_sim/urdf/robot.urdf` | `253b42607b18a6a21e51184be511ed9cbcec0246ff130ee830f89d47468f73b5` |

## 4. 외부 의존 — sllidar_ros2 upstream commit

★ `src/sllidar_ros2/` 는 `.gitignore` 대상이라 **우리 git 스냅샷에 없다.** 아래 해시를 남기지
않으면 동결 시점 구성을 재현할 수 없다.

| 항목 | 값 |
|---|---|
| repo | `https://github.com/Slamtec/sllidar_ros2.git` |
| commit | `34300099fadfc772965962dec837bf436706188f` (2024-06-17, `main`) |

재획득: `git clone https://github.com/Slamtec/sllidar_ros2.git src/sllidar_ros2 && git -C src/sllidar_ros2 checkout 34300099`

## 5. 동결 게이트 결과 (2026-07-24)

범위 = `TEST_GATES.md §7` **동결 게이트** 행 = `TEST_GATES.md §1` 전량 + 쌍굴 + 지도 승격 evidence.
상세·로그는 `~/Desktop/개발현황/0723_현황.md §11`.

| 항목 | 결과 |
|---|---|
| pytest | **159 passed** |
| colcon test-result | **165 tests, 0 errors, 0 failures, 2 skipped** |
| `regression_negative` | **PASS** — 금지 3종 ABORTED, 양성 대조군 0.085m |
| `regression_3goals` | **PASS** — 3목표 SUCCEEDED, 오차 **0.142m** (허용 0.3m) |
| `mission_e2e` (T자) | **PASS** — GUIDE 0.12 → SEARCH_BACK → 재발견 → ESCAPED |
| `abort_e2e` | **PASS** — 실정지 **0.0m**, cmd_vel 잠잠, '취소 접수 확인'. ★ 최초 1회 실패 = §6 |
| `mission_e2e twin` (쌍굴) | **PASS** — ESCAPED 111s, 로봇 world(-16.88, 0.08) / 탈출구 (-17, 0). ★ 4회 중 2회 하네스 실패 = §8 |
| 지도 hash 대조 | **MATCH** — §2 |
| `doc_check.sh --strict` | **PASS** |

## 6. ★ 알려진 시뮬 한계 — 동결과 함께 명시적으로 안고 가는 것

> 숨기지 않고 남긴다. 동결이 "결함 0"을 뜻하지 않으며, 이 한계를 아는 상태로 얼렸다는 뜻이다.

**현상**: `abort_e2e` 최초 실행에서 안전 단언 실패 — abort 후 5초간 **0.27m 이동**(> 허용 0.10m).

**근인** (`0723_현황.md §11.3` 시계열 증거):
1. 기동 직후 TF 지연으로 planner·controller 연속 실패 → BT 회복 사슬 진입.
2. 커스텀 BT 가 **BackUp 1순위**(`backup_dist 0.30 / backup_speed 0.05`) → 로봇 후진.
3. `backup failed` 로 회복이 **중단** — 중단된 회복은 **정지 명령(0)을 내보내지 않는다.**
4. `libgazebo_ros_diff_drive` 에 **command timeout 이 없다** → **마지막 cmd_vel 을 무한 유지.**
5. 로봇이 stale 한 backup 속도로 활주: 실측 **0.054 m/s** ≈ 설정 `backup_speed 0.05`.

**미션 코드는 무결**: abort → `취소 접수 확인`(2ms) → bt_navigator `Goal canceled`(60ms) →
`CANCELED 종결 확인`. 미션 노드는 `cmd_vel` 을 발행하지 않으므로 잔류 명령을 멈출 수단이 없다.

**수용 조건 (★ 오해 금지)**: 이 위험은 **"이미 해소됨"이 아니라**, 실차 구동부(Teensy)의
**cmd_vel watchdog(단절 0.5s 내 정지)이 R0 에서 실측 통과하는 것을 전제로 한 비차단 수용**이다.
그 watchdog 은 `MASTER_PLAN.md §3` 의 R0 **통과조건으로 계획되어 있을 뿐 아직 실측되지 않았다.**
**R0 실측에서 watchdog 이 확인되지 않으면 이 항목은 다시 열린다.**

**후속** = `MASTER_PLAN.md §7` 예약 항목 4·5 (abort 진단 강화 / 시뮬-실차 watchdog 정합성).

## 7. 환경 이탈 기록 (게이트 실행 당시)

- 실행 PC 의 GLX 가 깨져(`X_GLXCreateContext BadValue`) gzserver 가 기동 중 사망 →
  `env -u DISPLAY` 로 실행했다. 라이다가 `type="ray"`(CPU) 라 렌더링은 판정과 무관하며,
  RTF 를 **1.01**(`gz stats`)·**1.015**(`/clock` 대 벽시계)로 실측해 시간 축 왜곡이 없음을 확인했다.
- **원인**: 밤사이 **apt 자동 업데이트 실패로 그래픽 스택이 깨진 것** (07-24 사용자 확인·해소).
  ⚠ 구현자가 처음 지목한 `prime-select on-demand` 는 **오진**이었다 — 복구 후에도 `on-demand`
  그대로인 채 GLX 가 정상이다 (`0723_현황.md §11.2`).
- **해소 (07-24)**: GLX 복구 뒤 이탈 상태로 얻었던 2종을 **표준 환경에서 재실행**했다.
  `abort_e2e` = **PASS**(실정지 0.0m, GLX 에러 0건) → 이탈 없는 증거 확보.
  쌍굴 mission = §8 참조. 나머지 6종은 애초에 GLX 정상 시점에 통과한 것이다.
  재확인 상세·해시 16개 전량 재대조 = `0723_현황.md §11.5`.

## 8. ★ 알려진 하네스 취약점 — 쌍굴 4회 중 2회 실패 (전량 공개)

> §5 표의 쌍굴 PASS 는 **4회차**다. 통과한 회차만 인용하지 않기 위해 전 회차를 남긴다.

| 회차 | 환경 | 결과 | 원인 |
|---|---|---|---|
| 1 | `env -u DISPLAY` | **PASS** — ESCAPED 114s | — |
| 2 | 표준 | FAIL ⑧ | SEARCH_BACK 이 90초 예산을 ≈3초 차로 스침 |
| 3 | 표준 | FAIL ⑦ | `ros2 param get` 무한 행 (13분 27초, 타임아웃 가드 없음) |
| 4 | 표준 | **PASS** — ESCAPED 111s, SEARCH_BACK 9s | — |

**두 실패 모두 미션 코드를 지목하지 않는다** — E2E 하네스 결함이다.
- 2회차: 미션 로직은 놓침을 정확히 확정하고 역행 목표를 보냈으며, 그 구간 Nav2 는 건강했다
  (`Passing new path to controller` 1초 간격, 회복행동 0건). `wait_state` 가 t=87s 마지막 폴링 후
  `fail` 이 상태를 한 번 더 읽는 구조라 "타임아웃인데 마지막 상태는 목표 상태"가 나왔다.
- 3회차: 스크립트가 멈춘 동안 **미션은 독립적으로 쌍굴 탈출을 완주**했다(ESCAPED) — 시나리오
  자체의 건강함을 보이는 부수 증거다.

**변동성 기록**: GATHER 도달 **48s / 126s / 48s**, SEARCH_BACK 도달 **9s ~ ≈90s**.
쌍굴은 T자보다 경로가 길어 타이밍 분산이 크다.

⚠ 이 두 결함은 **T자 `mission_e2e.sh` 와 같은 파일**이다. 쌍굴 전용 문제가 아니며,
T자가 통과해 온 것은 여유가 있었을 뿐이다.

**동결 범위 밖으로 분리한 이유**: 둘 다 **판정 기준·도구 변경**이라, 동결 묶음이 스스로 금지한
"게이트 중 결함이 나오면 고치지 말고 별도 묶음으로 분리한다"에 걸린다 — 동결 커밋에 수정을
섞는 순간 "무엇을 얼렸는가"가 흐려지기 때문이다 (그 규칙의 원문은 §1 의 커밋 고정 명령으로 읽는다).
결정 경위 = `~/Desktop/개발현황/0723_현황.md §11.5` (불변 역사).
후속 = `MASTER_PLAN.md §7` 예약 항목 **6·7**.

### 8.1 ✅ 예약 6·7 수리 (07-24 e2e-harness-fix)

두 하네스 결함을 후속 묶음에서 고쳤다. **동결 런타임 코드는 무변경** — `tools/mission_e2e.sh`·
`tools/lib_e2e.sh`(테스트 하네스) + 신설 `tools/test_harness_guards.sh`(격리 단위)만 손댔고,
동결 기준점·태그는 재개방하지 않는다.

**이력 (정직하게)**: 1차 커밋 `0eb285c` 는 방향은 맞았으나 Codex `§14 P1` 로 **불승인**됐다 —
"유한 상한"·"180s 상한"이 sleep 누적·무방비 복구 명령 탓에 **벽시계로 보장되지 않았다**. 그 지적을
받아 벽시계(`SECONDS`) deadline 으로 재수리했다(아래 표의 '보완' 열).

| 결함 | 1차(0eb285c) | 보완 (§14 P1 반영) | 검증 |
|---|---|---|---|
| ⑦ `param get` 무한 행 | `read_param_float`(param get 만 `timeout 8`) | 복구용 `daemon stop/start` **각각 `timeout 5`** → 복구 시퀀스 전체 상한 **≈26s**. `-w 1` pub 3종도 timeout 예방 가드 | 격리 테스트 케이스 1·2 (daemon 무한 블록에도 유한 종결) |
| ⑧-a 폴링 race | 판정·보고 같은 읽기로 통일 | (유지) + 벽시계라 "예산 밖 늦은 도달"은 `경과>예산` 명시로 FAIL — 모순 없음 | 격리 테스트 케이스 4a·4b |
| ⑧-b 90s 예산 | **180s 재산정** (관측 최악 ≈90s 2배 마진) | 예산 **집행을 벽시계로** — `wait_state` 가 `state` timeout 소비분까지 산입, 읽기·대기 timeout 을 남은 예산으로 제한 | 격리 테스트 케이스 3 |

★ **판정 기준 변경 고지 (오해 금지)**: ⑧-b 는 **PASS 조건(SEARCH_BACK 제한시간)을 90s→180s 로 바꿨다.**
그러므로 위 §5·§8·§9 표의 게이트 수치와 태그 `platform-core-freeze-260724` 가 담은 수치는 **전부
옛 90s 기준**으로 얻은 것이다. 수리 후 현재 기준은 **180s(벽시계)** 이며, 두 기준이 조용히 섞이지
않도록 여기 못 박는다. **동결 기준점·태그는 재개방하지 않는다** — 태그는 "그 시점 옛 기준으로 이런
수치였다"는 불변 기록이고, 새 기준은 이 별도 묶음부터 적용된다.

**재검증 실측** (07-24, 표준환경, 새 180s 벽시계 기준):
- `tools/test_harness_guards.sh` **5/5 PASS** (daemon 무한 블록 → 벽시계 10s 종결 / state 소비형 flake
  → 벽시계 3s FAIL / 예산 안 PASS·예산 밖 FAIL).
- pytest **159 passed** / colcon **165 tests, 0 errors, 0 failures, 2 skipped** (무변동 — 셸 도구만 변경).
- `mission_e2e` (T자) **PASS**, `mission_e2e twin` (쌍굴) **PASS ×2 연속**, `abort_e2e` **PASS**(실정지 0.0m).
  벽시계 관측: SEARCH_BACK 13~15s, GATHER 71~143s, ESCAPED 28~167s — 모두 예산 내(옛 sleep 누적보다
  정직하게 큼).
- ★ 쌍굴 재실행 중 1회 `escape status=6 ABORTED` 관측 — **미션·하네스 무결**(planner 1초 간격 path 전송,
  GLX·좀비 0), 시뮬 주행 변동으로 분류 후 재실행 완주. 하네스 스코프 밖(미션/Nav2)이며, 오히려 벽시계
  `wait_state` 가 도달 실패를 정직하게 타임아웃 FAIL 보고함을 부수 확인.

상세·서사 = `~/Desktop/개발현황/0723_현황.md §15`.

### 8.2 ✅ §14 P1 보완의 재불승인 → hard-kill 재보완 (07-24 §15 P1)

**이력 (정직하게, 2라운드)**: §8.1 의 보완 커밋 `d70bbaf` 도 Codex `§15 P1` 로 **불승인 유지**됐다.
벽시계 전환과 판정·보고 일관성은 인정됐으나, "유한 상한"이 여전히 **두 경계에서 벽시계로 미보장**
이었다. 그 지적을 재현·수용해 hard-kill 로 재보완했다(커밋 `d70bbaf`→이번 보완).

| §15 P1 지적 | 재현(Codex) | 재보완 | 검증 |
|---|---|---|---|
| **① SIGTERM 무시 무한 행** — GNU `timeout N` 은 기본 TERM 만 보냄. CLI 가 `trap '' TERM` 이면 안 죽어 원 결함(daemon flake) 표면 잔존 | `timeout 5 ros2 daemon`(TERM 무시 fake)이 5초에 미종결, 외부 `--kill-after` 로 9초 SIGKILL 이라야 끝남 | `lib_e2e.sh` 소유 대기를 공통 `hard_timeout`=`timeout --kill-after=2` 로 단일화 → TERM 뒤 2초 유예 후 SIGKILL 보장. `read_param_float` 실제 hard 상한 = **34s**(정상 ≈26s). mission topic-pub 누락은 §8.3에서 종결 | 격리 케이스 **6** (TERM 무시 param·daemon 이어도 상위 cutoff 없이 함수 자체가 34s 내 종결) |
| **② daemon kick 예산 밖** — `wait_state` 의 daemon 재시작이 남은 예산과 무관하게 고정 `timeout 5`×2 | `wait_state SEARCH_BACK 13` 이 예산 13초가 아니라 **경과 23초**에 FAIL | daemon kick 도 **남은 예산 배분**(각 5s 상한 + 유예까지 rem 안에 수렴). 남은 예산 < 6s 면 복구 **생략**하고 deadline FAIL | 격리 케이스 **7** (예산 부족→생략, 13s FAIL — 구 23s 회귀 아님) · **8** (예산 충분→kick+TERM 무시 daemon 도 30s 내 수렴) |

★ **hard 상한 수치 (판정 기준)**: `read_param_float` = (8+2)+(5+2)+(5+2)+(8+2) = **34s**(TERM 무시 최악),
정상 TERM 응답 시 ≈26s. `wait_state N` 은 읽기·daemon 복구·sleep 전부가 같은 남은 예산을 소비해
벽시계 **N초(+마지막 주기 ≈1s 스케줄링 허용치)** 안에 같은 마지막 샘플로 PASS/FAIL 한다.

**재검증 실측** (07-24, 표준환경):
- `tools/test_harness_guards.sh` **8/8 PASS** (기존 5 + §15 신설 3: case 6=34s·7=13s·8=30s, 전부 벽시계 실측).
- pytest **159 passed** / colcon **165 tests, 0 errors, 0 failures, 2 skipped** (무변동 — 셸 도구만 변경).
- `mission_e2e` (T자)·`mission_e2e twin` (쌍굴)·`abort_e2e` 재검증 = §8.1 과 같은 게이트 재실행
  (결과는 커밋 메시지·`0723_현황.md §15.6` 에 실측 기록).

상세·서사 = `~/Desktop/개발현황/0723_현황.md §15.6`.

### 8.3 ✅ §16 P1 — mission topic-pub hard-timeout 누락 최종 종결

**이력 (검토 루프 마지막)**: `853ea7a`는 §15의 직접 두 공격을 닫았지만, Codex 연장 검토
`~/Desktop/개발현황/CODEX 현황/0723검토현황.md §16`에서 같은 `mission_e2e.sh`의
alarm·stop·follow 세 호출이 여전히
일반 `timeout 12 ros2 topic pub`임이 재현됐다. fake CLI가 SIGTERM을 무시하자 내부 timeout은
종결하지 못했고 외부 hard cutoff가 16초에 SIGKILL해야 끝났다. 구현 기록의 “모든 ros2 CLI”
주장은 이 세 wiring을 빠뜨려 불완전했다.

사용자가 **이번 세션에 한해 Codex 직접 보완을 명시 승인**해 세 호출을 이미 source된 공통
`hard_timeout 12`로 통일했다. 격리 테스트에 두 케이스를 추가했다:

- case 9: alarm·stop·follow TERM 무시 fake가 각각 축소 hard 상한 안에 종결하고,
  `mission_e2e.sh` 실제 wiring이 일반 timeout 0 / hard-timeout 3인지 확인.
- case 10: 정상 fake topic-pub 3종이 즉시 반환하는 역회귀.

**최종 실측** (07-24, 표준환경):
- `tools/test_harness_guards.sh` **10/10 PASS**.
- pytest **159 passed** / colcon **165 tests, 0 errors, 0 failures, 2 skipped**.
- T자 mission PASS: GATHER 15s · SEARCH_BACK 14s · ESCAPED 22s · 최종 로봇
  `(-11.87, -0.04)`(탈출구 `(-12,0)`).
- 쌍굴 mission PASS: GATHER 76s · SEARCH_BACK 14s · ESCAPED 164s · 최종 로봇
  `(-16.88, 0.08)`(탈출구 `(-17,0)`).

런타임 코드·동결 기준점·태그는 변경하지 않았다. 상세 구현·검증 서사는
`~/Desktop/개발현황/0723_현황.md §15.7`, 검토 발견은 `~/Desktop/개발현황/CODEX 현황/0723검토현황.md §16`.
이번이 허용된 연장 검토였으므로 추가 검토 루프는 열지 않고 사용자 최종 승인으로 닫는다.

## 9. ✅ 독립 검토 결과 (Codex 동결 판정 — 2026-07-24)

정본 = `~/Desktop/개발현황/CODEX 현황/0723검토현황.md §9`. **P0/P1/P2 0건 — 기술 통과.**
검토 범위 = `TEST_GATES.md §7` 동결 게이트 행(§1 전량 + 쌍굴 + 지도 승격 evidence),
대상 = `47bc440`(manifest 최초) + `212885a`(GLX 복구 후 재확인·정정).

검토자는 **구현자 수치를 재독해한 것이 아니라 전부 다시 실행**했다 (아래는 검토자 독립 실행값):

| 항목 | 구현자 (§5) | 검토자 독립 재현 (§9.3) |
|---|---|---|
| pytest | 159 passed | **159 passed** |
| colcon test-result | 165 / 0 fail / 2 skip | **165 / 0 fail / 2 skip** |
| `regression_negative` | PASS, 대조군 0.085m | **PASS**, 대조군 0.079m |
| `regression_3goals` | PASS, 0.142m | **PASS**, 0.137m |
| `mission_e2e` (T자) | PASS | **PASS** — ESCAPED 15s |
| `abort_e2e` | PASS, 실정지 0.0m | **PASS**, 실정지 **0.0m** |
| `mission_e2e twin` (쌍굴) | PASS, ESCAPED 111s | **PASS**, ESCAPED 114s, world (-16.88, 0.08) |
| hash evidence | 16/16 MATCH | **16/16 일치** + sllidar commit 일치 |

- 변경 표면 정적 검토: `a7b9da8..212885a` 는 문서 4개뿐, **코드·설정·지도 변경 0건** 확인.
- 검토자의 Gazebo 5종 로그에도 `X_GLXCreateContext`·`Service /spawn_entity unavailable` **0건**
  → §7 환경 이탈이 실제로 해소됐음을 제3자가 독립 확인.
- 검토 중 실패 2건(`regression_negative` 샌드박스 read-only / `abort_e2e` lifecycle 응답 유실)은
  **원인 한 줄 분류 후에만 재실행**됐고, 대상 커밋의 결함으로 분류할 근거가 없었다
  (`AGENTS.md §3-6` 준수 — `CODEX 현황/0723검토현황.md §9.4`).

**★ 검토자가 명시적으로 못 박은 것** (`CODEX 현황/0723검토현황.md §9.5`) — 이 문서의 §6·§8 과 동일 취지:

1. §6 잔류 cmd_vel 활주는 "해소"가 아니라 **R0 watchdog 실측 통과 전제의 조건부 수용**이며,
   **이번 abort PASS 가 그 실차 조건을 대신 증명하지 않는다.** R0 미통과 시 재개방한다.
2. §8 하네스 결함 2건은 이번 독립 실행에서 재현되지 않았지만 **"수리 완료"로 승격하지 않는다**
   (`MASTER_PLAN.md §7` 예약 6·7 유지).
   → **이후 경과 (07-29 추가)**: 위 판정은 **07-24 동결 시점의 불변 기록**이다. 예약 6·7 은 그 뒤
   별도 묶음 **e2e-harness-fix** 에서 수리·종결됐다 (§8.1~§8.3 · `MASTER_PLAN.md §7` 6·7 = ✅).
   **§9 를 현재 상태로 읽지 말 것** — 현재 하네스 기준은 §8.3 이다.

즉 **동결 = "결함 0"이 아니라 "무엇을 안고 얼렸는지 양쪽이 같은 문장으로 합의한 상태"**다.

## 10. 동결 예외 사용 기록 (열린 것만 · 이어서 추가)

> 동결은 "영원히 안 건드린다"가 아니라 **"열 때마다 누가·무엇을·어디까지 열었는지 남긴다"** 다.
> 이 절이 없으면 `git diff platform-core-freeze-260724 --stat` 이 1차 용의선상이라는 규칙이
> 무의미해진다 — 의도된 변경과 사고를 구별할 수 없기 때문이다. **한 줄도 빠짐없이 여기 적는다.**

### 10.1 예약 16 — `mission_node.py` 의 `last_seen` 기록 경로 (2026-08-01)

| 항목 | 값 |
|---|---|
| 승인 | 2026-07-31 **사용자(역할 A) 명시 승인** — 원문 '결정 A' 는 커밋 `c5b4fd3` 에 고정 (아래 명령) |
| 승인 범위 | `①` 진단 로그 + `②′` 기록 조건·안전망. **`mission_node.py` 의 `last_seen` 기록 경로에 한정** |
| 명시적 제외 | ~~`③` `visible` 술어 완화~~ (구현자 반대 → 사용자 철회) · `follower_monitor.py` 술어 자체 · 다른 상태 전이 |
| 열린 이유 | 게이트 신뢰성 문제가 아니라 **미션 안전 결함**으로 재분류됐다 — 추종자를 놓친 것을 판정하고도 역행·보고 없이 탈출 계속 (`MASTER_PLAN.md §7` 예약 16) |
| 실제 변경 | `src/mission_manager/mission_manager/mission_node.py` **1파일 · 3곳** (기록 조건 · `last_seen is None` 가드 · `record_last_seen` 의 `except`). 기본 `git diff` 기준 **3 hunk**, `-U0` 로 쪼개면 5 — 기록 조건 한 곳이 3조각이라 그렇다 |
| 변경 안 함 | `follower_monitor.py` · `speed_manager.py` · `goal_manager.py` · `config/waypoints.yaml` · `src/tunnel_sim/**` — **전부 무변경** |
| 회귀 | 신규 `src/mission_manager/test/test_search_back_entry.py` 11건 (**보완 전 5건 FAIL 관측** + 역회귀 앵커 6). 역회귀 앵커 = T자·쌍굴 `mission_e2e` + `abort_e2e` |
| 서사·증거 | `0801_현황.md §1`~`§7` |

승인 원문을 그때 그대로 꺼내는 법 (핸드오프는 묶음마다 교체되므로 **커밋을 고정해** 읽는다):

```bash
git show c5b4fd3:docs/CURRENT_HANDOFF.md     # 07-31 승인 원문 '결정 A' + 그때의 허용 범위
```

★ **범위 준수는 주장이 아니라 diff 로 보인다** — `git diff` 가 위 3곳 밖으로 나가지
않았음을 커밋 전에 확인했다. 나갔다면 그 시점에 승인을 다시 받는 것이 규칙이다.
