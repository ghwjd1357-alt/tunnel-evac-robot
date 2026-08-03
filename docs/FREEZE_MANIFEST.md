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

**D+0 실행 절차** = `JETSON_SETUP.md §7-c`의 7-c-0. 바퀴를 공중에 띄우고 zero Twist 없이
publisher를 끝낸 뒤, 60fps 동일 화면 영상에서 마지막 발행 표시→마지막 바퀴 회전이 30프레임
이하인지 본다. `/odom.pose` raw encoder 적분 정지는 교차 증거이고 EMA `/odom.twist`의 0 도달
시각은 판정에서 제외한다. **PASS 기록이 `D1_FIRST_STEP.md §0-a`에 없으면 R1로 진행하지 않는다.**

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

### 10.2 검토 §26 P1·P2 — GUIDE 진입 관측 세대 (2026-08-01, 두 번째 예외)

| 항목 | 값 |
|---|---|
| 승인 | 2026-08-01 **사용자(역할 A) 명시 승인** — §10.1 예외는 이미 소진됐으므로 **새로 받았다.** 범위 질의에 '`mission_node.py` + `test/` 만'을 명시 선택 |
| 승인 범위 | `src/mission_manager/mission_manager/mission_node.py` + `src/mission_manager/test/**`. **검토 §26 의 P1(GUIDE 진입 세대) · P2(주석 정정)에 한정** |
| 명시적 제외 | `follower_monitor.py` — **또 열지 않았다**(diff 0). 모니터 안에 '세대' 개념을 넣는 선택지는 사용자가 배제했다. `visible`/`lost` 술어 자체도 여전히 금지 |
| 열린 이유 | §10.1 보완(`e0fac77`)이 **불승인**됐다 — 회귀가 상태를 손으로 `GUIDE` 에 꽂아 **진입 경계를 한 번도 실행하지 않았고**, 그 경계에 안전 결함 2개가 살아 있었다 (`~/Desktop/개발현황/CODEX 현황/0801검토현황.md §26.2`) |
| ★ 사용자 정책 결정 | **never-seen(한 프레임도 못 본 추종자)은 다른 놓침과 동일 취급**한다 — 역행 `max_attempts` 회 → '추종자 확인 불가' 보고 → 단독 탈출. 근거: 실차에서는 **카메라가 사람을 확인한 뒤 GATHER 로 넘어가므로**(역할 B) never-seen = '확인된 사람을 후방 라이다만 못 봄' → 되돌아가 확인하지 않고 나가는 것은 확인된 사람을 버리는 것 |
| 실제 변경 | `mission_node.py` **1파일 · 5자리** (`-U0` 기준, 아래 목록). **동작이 바뀌는 자리는 2곳뿐**이고 3곳은 주석·독스트링이다 |
| 변경 안 함 | `follower_monitor.py` · `speed_manager.py` · `goal_manager.py` · `config/waypoints.yaml` · `src/tunnel_sim/**` · `src/tunnel_bringup/**` — **전부 무변경** |
| 회귀 | `test_search_back_entry.py` **11 → 19건**. 신규 8건 중 **부정 회귀 4건**(sb12·13·14·15)은 **보완 전 FAIL 을 격리 사본에서 관측**했고, **역회귀 앵커 4건**(sb16·17·18·19)은 구판·신판 양쪽 통과. `test_speed_manager.py` 는 껍데기 노드 필드 1줄 추가뿐 |
| 서사·증거 | `0801_현황.md §10`~`§14` |

`mission_node.py` 의 5자리 (**개수가 아니라 자리로** 남긴다 — §10.1 검토가 요구한 형식):

| # | 자리 | 성격 |
|---|---|---|
| 1 | `__init__` — `_prev_tick_state` 초기화 | **동작** (신규 상태 필드) |
| 2 | `__init__` — `last_seen` 필드 규약 주석 | 주석 (P2) |
| 3 | `tick()` — GUIDE 진입 초크포인트 (`monitor.reset('any')` + 스냅샷) | **동작 — 이번 수리의 전부** |
| 4 | `enter_search_back()` — ②′ 안전망 논증 주석 재작성 | 주석 (구판 논증이 §26 에서 반증됐다) |
| 5 | `record_last_seen()` — 독스트링 | 주석 (P2) |

★ **왜 진입 경로마다가 아니라 초크포인트 1곳인가**: `grep` 전수로 GUIDE 진입은
`mission_node.py` 의 4자리(SEARCH_BACK 재발견 · SEARCH_BACK 대기실패 · GATHER 저속확인
콜백 · FAULT `resume_state` 복귀)이고 그중 재무장하던 곳은 **2자리뿐**이었다. 상태 전이
감지로 걸면 남은 2자리와 **앞으로 생길 경로**가 자동으로 덮인다 — GUIDE 저속 fail-closed
게이트가 07-20 에 세운 것과 같은 논증이다(경로를 세지 말고 '시작하는 지점'을 막는다).

### 10.3 검토 §27 P1·P2 — 센서 관측 세대 (2026-08-02, 세 번째 예외)

| 항목 | 값 |
|---|---|
| 승인 | 2026-08-02 **사용자(역할 A) 명시 승인** — §10.2 예외는 이미 소진됐으므로 **새로 받았다.** 두 방안(A: 모니터에서 규칙 수정 / B: 노드에서 재무장 보류)을 제시했고 사용자가 **A안**을 선택 |
| 승인 범위 | `src/mission_manager/mission_manager/follower_monitor.py` + `src/mission_manager/test/**` (+ `mission_node.py` **주석만**) |
| ★ 처음으로 열린 파일 | `follower_monitor.py` — §10.1·§10.2 에서 **두 회차 연속 배제**했던 파일이다. 다만 열린 것은 `update()` 의 **단절 복구 경계** 한 곳이고, **안전 술어 `visible`/`lost` 는 이번에도 무변경**이다 |
| 열린 이유 | §10.2 보완(`539f434`)이 **불승인**됐다 — GUIDE 진입 시 놓침 타이머의 기산점을 세우는데, 그 시점에 `/scan` 이 **한 장도 온 적이 없으면**(`_last_scan_t is None`) 모니터의 단절 복구 보호가 발동하지 않아, 첫 유효 빈 프레임 한 장에 즉시 `lost` → 역행 0회로 예산만 태우고 단독 탈출 (`~/Desktop/개발현황/CODEX 현황/0801검토현황.md §27.2`) |
| 실제 변경 | `follower_monitor.py` **2자리** — ① `update()` 단절 복구 가드 `is not None and` → `is None or` (**동작 · 이번 수리의 전부**) ② `reset()` 독스트링 (P2). `mission_node.py` **4자리는 전부 주석**이라 실행 코드 변경 0 |
| 변경 안 함 | `speed_manager.py` · `goal_manager.py` · `config/waypoints.yaml` · `src/tunnel_sim/**` · `src/tunnel_bringup/**` — **전부 무변경** |
| 회귀 | `test_search_back_entry.py` **19 → 23건**. 부정 회귀 3건(sb20·sb21·sb23)은 **보완 전 FAIL 을 격리 사본에서 관측**했고, 역회귀 앵커 1건(sb22 첫 유효 PERSON 스캔)은 구판·신판 양쪽 통과 |
| 서사·증거 | `0801_현황.md §15`~`§18` |

★ **왜 모니터 쪽인가 (A안 채택 근거)**: 깨진 계약이 *"센서 데이터가 존재하지 않았던
시간은 debounce 에 포함하지 않는다"* 이고, 그 계약은 **모니터가 소유**한다(07-19 watchdog).
호출자 쪽에서 막으면 지금 뚫린 **한 자리**만 닫히고 새 호출자마다 같은 구멍이 다시 열린다
— §26→§27 이 **연속 두 회차** 그 패턴이었다. 모니터에서 막으면 **클래스**가 닫힌다.
동작 확대는 안쪽 가드(`_last_seen_t is not None` 인 zone 만)가 막는다: 첫 스캔 이전에
그 값이 non-None 인 경우는 **`reset()` 이 불린 경우뿐**이므로(검출은 스캔 없이 불가능)
그 외 모든 입력에서 **no-op** 이다.

★ **P2 는 기계 검사로 닫았다**: `reset()` 호출 자리마다 `[reset-role] <이름>` 을 달고
독스트링의 역할 목록과 대조한다(`test_sb23_…`). 호출 자리를 추가하고 설명을 안 고치면
FAIL 한다 — 사람이 목록을 보고 옮겨 적는 자리를 없앤 것이다 (`AGENTS.md §3-10 ②`).
개수 대신 역할을 적은 이유는 숫자가 다음 회차에 또 갈라지기 때문이다.
⚠ **단, "자리 소실 양방향 게이트"라는 커밋의 전칭 주장은 틀렸다** (§28 P2-1 · 구현자 재현
확인). 이 검사는 소스를 **줄 문자열**로 읽으므로 **실행 호출을 주석 처리하고 `pass` 를 넣으면
통과한다.** 잡는 것 = 태그 오타·태그 없는 새 줄·텍스트 삭제. 못 잡는 것 = **주석 잔존 소실**.
런타임 결함은 아니지만(생산 호출 3곳은 실제 실행문이고 sb12~sb17 이 동작으로 검증) 미래
유지보수의 fail-open 이다 → `ast` 기반 교체 = `MASTER_PLAN.md §7` **예약 20**.

### 10.3.1 ✅ 독립 검토 결과 — **§28 승인** (2026-08-02)

| 항목 | 결과 |
|---|---|
| 판정 | **승인 · P0 0 · P1 0 · P2 2** — 예약 16 검토 루프 **종결**, 정본을 실차 S6 로 전진 |
| §27 P1 닫힘 확인 | 검토자 독립 실행: `/scan` 0장 GATHER→GUIDE 전이 뒤 첫 유효 EMPTY 에서 즉시 `lost`·예산 소모 **없음** · `lost_sec` 초과 뒤에만 **좌표가 있는** SEARCH_BACK · 같은 지연에서 실제 `search_back` goal **2회** 뒤 보고·단독 탈출 |
| 구판/신판 | 격리 사본 **3 failed / 20 passed**(sb20·21·23) → 신판 **23 passed** (구현자 주장과 일치) |
| 게이트 | follower monitor 역회귀 **59 passed** · pytest **182** · colcon **245**(0e·0f·3s) · `doc_check --strict`/`--after-push` PASS · `mission_e2e` T자 PASS(GATHER 14s·SEARCH_BACK 14s·ESCAPED 23s) · 쌍굴 PASS(GATHER 73s·SEARCH_BACK 15s·ESCAPED 164s) · `abort_e2e` PASS(실정지 0.0m) |
| 범위 | 승인 자리를 벗어나지 않음 — 제외 파일 diff 0 확인 |
| 남은 P2 | **P2-1** = 위 `[reset-role]` 검사의 주석 잔존 사각 (→ 예약 20) · **P2-2** = 당시 핸드오프 활성 문장이 옛 대상 해시를 가리킴 (→ 이 전진 갱신에서 해소) |
| 검토자 인프라 실패 1건 | 최초 `colcon` 이 sandbox 의 `~/.ros/log` 쓰기 거부로 연쇄 실패 → **원인 분류 후** sandbox 밖 재실행하여 245 확보 (`AGENTS.md §3-6` 준수) |

전문 = `~/Desktop/개발현황/CODEX 현황/0801검토현황.md §28`.

### 10.4 예약 19 — `src/tunnel_bringup/**` 구동부 확정값 코드 반영 (2026-08-02, **네 번째 예외**)

★ **처음으로 `mission_manager` 밖이 열린 예외다.** §10.1~§10.3 은 전부 미션 층이었다.

| 항목 | 값 |
|---|---|
| 승인 | 2026-08-02 **사용자(역할 A) 명시 승인** |
| 승인 범위 | `src/tunnel_bringup/config/**` · `src/tunnel_bringup/urdf/robot_real.urdf` · `src/tunnel_bringup/launch/**` — **대상 5건에 한정** (아래) |
| 열린 이유 | 구동부 3차 회신(08-01) 확정값을 **문서 정본에는 반영했으나**(`f4bfa74`) 코드가 구값이라 **문서와 코드가 갈라진 상태**였다. 그중 2건은 **인수 당일(2026-08-03) 즉발**이다 |
| 착수 순서 | **S6 작성순 1번(맨 앞)** — `MASTER_PLAN.md §3` S6 표. `launch` 기본값이 바뀌어 S6-1·S6-5 런북 내용을 좌우하므로 런북보다 먼저 한다 |
| **변경 안 함 (경계)** | `src/tunnel_sim/**` · `src/mission_manager/**` · `src/tunnel_bringup/tunnel_bringup/**`(노드 코드) · `src/tunnel_bringup/test/**` — **전부 무변경이어야 한다** |
| ⚠ **특히 `gate_fakes.py` 무변경** | IMU 주기 정본은 **§25 결정 B 로 동결**된 트랙이다. 재개방 조건(R3 rosbag)이 아직 안 걸렸고, 지금 고치면 **요약통계를 다른 요약통계로** 바꾸는 것이라 R3 때 두 번 일한다. 전제·교체 시점 = `TEST_GATES.md §2` 검증 상한 5 ⓒ |

**대상 5건** (정본 = `MASTER_PLAN.md §7` 예약 19)

| # | 파일 | 현재 → 반영 | 급 |
|---|---|---|:--:|
| ① | `config/ekf_real.yaml` | QoS 오버라이드 없음(RELIABLE) → `/odom`·`/imu/data` **BEST_EFFORT** | 🔴 |
| ② | `config/nav2_params_real.yaml` | `max_velocity: [0.30, 0.0, 0.80]` → **`[0.12, 0.0, 0.50]`** | 🔴 |
| ③ | `urdf/robot_real.urdf` | `base_footprint→base_link` `z=0.065` → **`z=0.053`** | 🔴 |
| ④ | `urdf/robot_real.urdf` | `imu_link` TODO → **`xyz="0 0 0.339" rpy="0 0 -1.5708"`** | 🟠 |
| ⑤ | `config/ekf_real.yaml` 주석 · `launch/*.launch.py` | 41.63Hz·`serial_baud 921600` → **46.4Hz·115200** | 🟡 |

**⚠ 판정 근거에서 제외한 게이트 — 의도된 예외**

`test_gate_regression` 은 이 묶음의 **판정 근거가 아니다.** 4회 중 3회 `13/14` 로 갈리는
간헐 실패(§24.5a)가 **미규명**이라, 실패해도 **내 변경 탓인지 원래 flake 인지 분리할 수 없다**
(`AGENTS.md §3-6`). 원래 규칙은 "다음 `tunnel_bringup` 묶음 착수 **전에** 규명"이었으나
**인수일이 2026-08-03** 이라 기다리면 🔴 즉발 2건을 못 고친 채 인수를 맞는다.

- **판정**: `pytest` · `colcon` · `doc_check` 로 한다
- **실행은 하되** 결과는 **참고 기록**으로만 남긴다
- **재개방 조건**: 인수 후 flake 를 규명하면 **그때 이 변경을 게이트로 재판정**한다.
  규명 전까지 "게이트가 통과했으니 안전하다"고 **쓰지 않는다**

#### 10.4.1 실제 변경 기록 (2026-08-02 구현 완료)

| 항목 | 값 |
|---|---|
| 실제 변경 | **5파일** — `config/ekf_real.yaml` · `config/nav2_params_real.yaml` · `urdf/robot_real.urdf` · `launch/real_bringup.launch.py` · `launch/real_mapping.launch.py` |
| 변경 안 함 (확인) | `src/tunnel_sim/**` · `src/mission_manager/**` · `src/tunnel_bringup/tunnel_bringup/**` · `src/tunnel_bringup/test/**` → `git diff --stat` **0줄**. **`gate_fakes.py` 무변경** |
| 회귀 | pytest **182 passed** · colcon **245 tests, 0 errors, 0 failures, 3 skipped** — 기준선 유지 |
| 참고 기록 (판정 근거 아님) | `test_gate_regression` — 위 '판정 근거에서 제외' 항목대로 실행하되 결과를 판정에 쓰지 않았다 |

**자리별 기록** (개수가 아니라 **자리**로 남긴다 — §10.1 검토가 요구한 형식)

| # | 자리 | 성격 |
|---|---|---|
| 1 | `nav2_params_real.yaml` — `velocity_smoother.max_velocity`/`min_velocity` | **동작** (0.30/0.80 → 0.12/0.50, 음수 쪽 포함) |
| 2 | `nav2_params_real.yaml` — RPP `desired_linear_vel`·`rotate_to_heading_angular_vel` 주석 | 주석 (근거 갱신 + 여유 소실 경고) |
| 3 | `robot_real.urdf` — `base_joint` origin z | **동작** (0.065 → 0.053) |
| 4 | `robot_real.urdf` — `imu_joint` origin | **동작** (`0 0 0`/`0 0 0` → `0 0 0.339`/`0 0 -1.5708`) |
| 5 | `robot_real.urdf` — 머리말·차고·바퀴 그림 주석 | 주석 (3종 분리 · 지상고 20mm · TODO 목록 정리) |
| 6 | `real_bringup.launch.py`·`real_mapping.launch.py` — `serial_baud` 기본값 | **동작** (921600 → 115200) |
| 7 | `ekf_real.yaml` — `frequency` 주석 | 주석 (41.63Hz/30ms → 46.4Hz/20~23ms, 여유 10.3ms) |
| 8 | `ekf_real.yaml` — QoS 절 **신설** | 주석 (①의 실측 결과와 재확인 방법) |

★ **①은 코드 변경이 없다 — 전제가 실측으로 뒤집혔다.** 상세 = `MASTER_PLAN.md §7` 예약 19.
요약: EKF 구독은 **이미 BEST_EFFORT** 였고, `robot_localization` 3.5.4 는 **구독 QoS
오버라이드 파라미터 자체를 제공하지 않는다**(퍼블리셔 3개만). 계획대로 yaml 에 한 줄을
넣었다면 **아무 효과 없이 무시되면서 "막았다"는 기록만** 남았을 것이다.
→ 조치 대신 **검사**로 닫았다: `tools/d0_check.sh` 가 *"BEST_EFFORT 발행 + RELIABLE 구독"* 을
토픽 전체를 훑어 잡는다(목록을 손으로 적지 않았다). Jetson 의 버전 차이도 이 검사가 D+0 에 재확인한다.

★ **관측으로 닫은 것** (구현자 주장이 아니라 실행 결과):
`robot_state_publisher` + `tf2_echo` 실행에서 **`base_footprint→imu_link` = 0.392 m · yaw −90°** —
구동부가 준 **바닥 기준 실측 392mm 와 일치**했다. 기준면 뺄셈(0.392 − 0.053)이 왕복으로 닫혔다.
`base_footprint→base_link` = **0.053**.

⚠ **공개하는 잔여 위험**: ② 반영으로 `desired_linear_vel`(0.12)·`rotate_to_heading_angular_vel`(0.5)이
**펌웨어 상한과 같아졌다.** 곡선에서 두 성분이 동시에 걸리면 바깥 바퀴가 상한을 넘어 잘리고,
**잘림은 Nav2 에 보고되지 않는다.** 실측이 없어 값을 임의로 낮추지 않았다 — R6 판정 항목이다.


### 10.5 검토 §29 보완 — D+0/R3 판정기 (2026-08-02, **동결 예외 아님**)

★ **이 절은 동결 예외 기록이 아니다.** `src/**` 를 한 글자도 열지 않았다
(`tools/`·`docs/` 전용). 여기 두는 이유는 §10.4 의 산출물을 **검토가 불승인했고**
그 보완 근거를 같은 자리에서 따라갈 수 있어야 하기 때문이다.

**대상 판정** = `483c856` 독립 검토 **불승인 (P0 0 · P1 4 · P2 1)**
(전문 = `~/Desktop/개발현황/CODEX 현황/0801검토현황.md §29`).
그중 P1 1건(`/odom` publisher 계약)은 후속 `eafa0ed` 가 펌웨어 소스 v1.4 로 이미 정정했다.

| 지적 | 무엇이 틀렸나 | 보완 |
|---|---|---|
| §29.2 P1 | 런북 clone 이 **rc 128 확정 실패** (`mkdir -p src` 뒤 `clone .`) | 목적지를 인자로 주고, 기존 워크스페이스는 `.git` 유무로 분기 |
| §29.3 P1 | QoS 검사가 **구독자 0개를 "전부 매칭"으로 승인** + 런북 순서상 소비자가 없었다 | `nsub=0` FAIL · **EKF 를 이름으로 요구** · 런북에 EKF 선기동(§7-a) · 종료 직전 **생존 재확인** |
| §29.4 P1 | `max: N/A`→`0.00ms` · 표본 수/창 지속시간 미검증 · bag 이 **양끝과 `header.stamp` 를 안 봄** | 유한 숫자 검증 · **표본 수 하한**(실측 보정) · **창 끝 수신 프로브** · bag 양끝 결합 · stamp 엄격 단조 |
| §29.5 P1→P2 | 런북 §7 표가 "발행자 BEST_EFFORT" 단일값 | 토픽별 계약(`/odom` RELIABLE · `/imu/*` BEST_EFFORT)으로 정정 |
| §29.6 P2 | `--secs` 값 누락이 **무한 루프**(rc 124) | `shift` 전 개수 확인 · 상한 120 |

**★ 검토가 준 목록과 구현의 대조** (`AGENTS.md §3-10 ④` — 남이 준 열거가 더 위험하다)

가짜 `ros2` 를 PATH 에 놓고 **17 시나리오**, 합성 bag **6종**, clone **4경로**,
`--secs` **경계 6종**을 실행해 요구 목록을 한 줄씩 대조했다. 전부 일치(불일치 0).

| 검토가 요구한 것 | 시나리오 | 결과 |
|---|---|---|
| pub 만 있고 sub 0개 → FAIL | `nosub` | ✅ |
| BEST_EFFORT pub + RELIABLE EKF sub → FAIL | `sub_rel` | ✅ |
| DDS 호환 3조합 → PASS | `sub_be`·`sub_rel_odom`·`normal` | ✅ |
| EKF 가 검사 도중 죽으면 PASS 금지 | `ekf_dies` | ✅ |
| pub 기대값 반대 주입 2건 → FAIL | `pub_swapped`·`pub_be_all` | ✅ |
| 정상 1초 뒤 정지 · `max: N/A` · 표본 1개 · 창 끝 소실 → FAIL | `hz_die`·`hz_na`·`hz_one`·`tail_dead` | ✅ |
| bag: 늦은 시작·이른 종료·동일 stamp·역행 stamp·내부 40ms → 각각 FAIL | 합성 bag 5종 | ✅ |
| bag 양끝 공백을 **따로** 주입해 둘 다 검출 | `late_start`·`early_end` | ✅ |
| clone: 구판 rc 128 · 보완 성공 · 재실행 · 최종 트리 | 임시 HOME 4경로 | ✅ |
| `--secs` 값 누락·0·2·음수·문자·상한+1 → 3 / 3·기본·상한 → 실행 | 경계 8건 | ✅ |

**⚠ 보완 중에 내가 새로 넣었다가 관측으로 잡은 결함 (숨기지 않는다)**

검사 번호 중복을 없애려고 `next_idx()` 를 넣었는데 **`$(next_idx)` 로 호출**했다 —
명령치환은 서브셸이라 증가가 부모로 전파되지 않아 **여덟 검사가 전부 `[1]`** 로 찍혔다.
손으로 적어 중복됐던 것보다 나빠진 상태였고, **출력을 눈으로 보고서야** 알았다.
→ 그날 아침 내가 핸드오프에 올린 *'설정 실효 불변조건'*("조치했다 ≠ 조치가 실효했다")을
**같은 날 내가 어겼다.** 고친 뒤 번호 `[1]`~`[8]` 과 `검사 8 개 수행` 을 출력으로 확인했다.

**게이트**: pytest **182** · colcon **245**(0e·0f·3s) · `doc_check --strict` PASS ·
`bash -n` PASS · `scan_unbounded_cli` 0건 · ~~flake8 PASS~~. `src/**` diff **0**.

⚠ **08-03 정정 (검토 §30.5 P2)** — 위의 `flake8 PASS` 는 **독립 실행으로 재현되지 않았다.**
검토자가 `python3 -m flake8 tools/bag_gap_report.py` 를 돌리자 E501 **9건**으로 종료 1이었다.
명령을 안 적고 결과만 적었기 때문에 무엇을 돌렸는지 다시 알 수 없다 — **취소선으로 남기고**
지운다. 이 저장소의 실제 lint 계약과 그 정확한 명령은 아래 §10.6 에 적었다.
★ 교훈: **증거에는 결과가 아니라 명령을 적는다.** 결과만 적힌 증거는 다음 사람이 재현할 수
없고, 재현할 수 없는 증거는 없는 것과 같다(그 자리에서 이번 P1 하나가 실제로 숨었다).

⚠ **여전히 실차에서 못 돌려 봤다.** 위 검증은 전부 가짜 입력이다 — 실제 Jetson·Teensy·
rosbag 으로는 D+0/D+1 에 처음 돌아간다. 회귀를 저장소에 박는 일은 **예약 21** 이다.

### 10.6 검토 §30 보완 — 토픽별 계약·관측 창 완주 (2026-08-03, **동결 예외 아님**)

★ **이 절도 동결 예외 기록이 아니다.** `src/**` 를 한 글자도 열지 않았다(`tools/`·`docs/` 전용).
§10.5 의 보완물을 검토가 다시 불승인했으므로 그 근거를 같은 자리에서 이어 간다.

**대상 판정** = `6dacd19` 독립 검토 **불승인 (P0 0 · P1 2 · P2 2)**
(전문 = `~/Desktop/개발현황/CODEX 현황/0801검토현황.md §30`).
§29 에서 닫힌 clone·QoS·bag 양끝·stamp·`--secs` 경계는 **수용**됐고, 다시 설계하지 않았다.

| 지적 | 무엇이 틀렸나 | 보완 |
|---|---|---|
| §30.2 P1 | `bag_gap_report.py` 가 **33.33ms 하나를 모든 토픽에** 적용 → 사양 10Hz 인 정상 `/scan` 이 반드시 FAIL (검토자 실측: 초과 10건·최대 100.00ms). 멀쩡한 라이다를 IMU 결함으로 읽고 **동결된 주기 정본을 헛되이 재개방**시킬 뻔했다 | 토픽별 계약을 `TOPIC_POLICY` **한 표**로 두고 판정기·런북이 같은 표를 소비. EKF 입력(`/odom`·`/imu/data`)만 33.33ms 계약, `/scan` 은 **liveness(사양 100ms×2)+stamp** 만 판정하고 분포는 관측값으로 보고 |
| §30.3 P1 | `check_hz` 가 `topic hz` 의 **종료 상태를 안 봤다** → 4초 만에 rc 42 로 죽어도 죽기 전 요약이 "8초 창"으로 승격 (검토자 실측: `rc=42 / check_fail=0`) | **종료 상태 + 벽시계 경과시간**을 함께 판정. 상한 발동(124/137)만 완전한 창으로 인정하고, 그 판정을 **표본·간격 판정보다 먼저** 두어 실패 시 조기 반환. 별도 `echo` 는 **보조 증거로 격하**(승격 근거 아님을 코드·주석에 명시) |
| §30.4 P2 | `JETSON_SETUP §9` 는 10건인데 핸드오프 네 자리가 **8건** — `doc_check --strict` 가 PASS | 네 자리를 10 으로 정정 + **`tools/todo_d0_scan.py` 신설**: 목록 행을 직접 세어 모든 개수 주장과 대조하고 `doc_check` 에 편입 |
| §30.5 P2 | `flake8 PASS` 가 **독립 실행으로 재현 불가**(E501 9건) | 위 §10.5 의 주장을 취소선으로 정정하고, 이 저장소의 실제 계약과 **정확한 명령**을 아래에 남긴다 |

**★ 클래스 전수 열거 → 항목별 대조** (`AGENTS.md §3-10` — 지적받은 그 자리만 고치면 다음
회차에 바로 옆이 지적된다. 근거는 기억이 아니라 `grep` 이다)

| 클래스 | 열거 명령 | 나온 자리 | 각 자리가 덮이는가 |
|---|---|---|---|
| A. 판정 상수를 여러 토픽에 일괄 적용 | `grep -rnE "GAP_LIMIT_MS\|GAP_MAX_MS\|HZ_MIN" tools/*.py tools/*.sh` | `bag_gap_report.py` 7행 · `d0_check.sh` 8행 | ✅ 전자는 토픽별 표로 분리. **후자는 결함이 아니다** — 대상이 EKF 입력 2종뿐이라 33.33ms/30Hz 가 둘 다 맞다. `/scan` 을 나중에 넣으면 같은 함정이므로 **그 사실을 주석에 못 박았다** |
| B. 외부 CLI 종료 상태를 안 묻는 자리 | `grep -n "hard_timeout " tools/d0_check.sh` | 호출 **8자리** | ✅ 8/8 대조표를 `d0_check.sh` 머리말에 넣었다. 지적은 1자리였는데 **같은 클래스에서 2자리(`topic info`·부호 관측)가 더 나왔다.** ~~⚠ 마지막 1자리(재확인)는 파이프라인이라 rc 를 직접 안 보고 fail-closed 방향으로만 떨어진다~~ → **정정 (08-03 §31.3)**: fail-closed 가 아니었다. 잘린 지점이 목표 EKF 행 **뒤**면 `grep` 이 성공해 rc 124 가 덮인다 — 아래 §10.7 에서 닫았다 |
| C. 개수 계약이 문서에 손으로 적힌 자리 | `grep -rn "TODO(D+0)" docs/*.md` + 공백 정규화 근접 창 | **5자리** (핸드오프 4 · 런북 1) | ✅ 5/5 를 기계가 훑는다. ⚠ 줄 단위 grep 은 그중 **한 자리를 못 봤다**(`TODO(D+0)` 와 `8건` 이 다른 줄) — 그래서 검사기는 파일을 정규화해서 본다 |
| D. 명령 없이 결과만 적은 증거 | `grep -rn "flake8\|bash -n\|scan_unbounded_cli" docs/*MANIFEST*.md` | 1자리 | ✅ 그 자리를 정정하고, 이 절의 증거는 **전부 명령을 그대로** 적는다 |

★ **B 클래스에서 배운 것 — 같은 `rc` 인데 정상값이 정반대다.** 관측 창(`topic hz` 를 N초
돌리는 것)은 **우리가 건 상한이 발동해야**(124/137) 정상이고, 스냅샷(`topic info`·`echo --once`)은
**rc 0** 이 정상이다. 한쪽 규칙으로 다른 쪽을 재면 조용히 뚫린다. 이 구분을 `window_completed()`
라는 이름으로 코드에 박아 두 용도가 섞이지 않게 했다.

**★ 검토가 준 완료판정·회귀 목록과 구현의 대조** (`AGENTS.md §3-10 ④`)

| 검토가 요구한 것 | 어떻게 관측했나 | 결과 |
|---|---|---|
| 역회귀: bag 전 구간 10Hz `/scan` + 정상 odom·imu → PASS | 합성 bag `normal` | ✅ 종료 0 (구판은 종료 1) |
| 부정: `/scan` 늦은 시작·이른 종료 → FAIL | `scan_late`(앞 250ms) · `scan_early_end`(뒤 250ms) | ✅ 둘 다 FAIL |
| 부정: `/scan` 동일·역행 stamp → FAIL | `scan_dup_stamp` · `scan_back_stamp` | ✅ 둘 다 FAIL |
| 부정: odom 또는 imu 내부 40ms → `/scan` 정상 여부와 무관하게 FAIL | `odom_gap40` · `imu_gap40` (scan 정상) | ✅ 둘 다 FAIL |
| 경계: scan 계약과 odom·imu 33.33ms 의 **바로 안/밖** | `odom_edge_in/out`(33.32/33.34ms) · `scan_edge_in/out`(199.9/200.1ms) | ✅ 안 2/2 PASS · 밖 2/2 FAIL |
| 부정: 정상 모양 요약 뒤 rc 0·rc 42 조기 종료 → 둘 다 FAIL | 가짜 `ros2` `early0`·`early42` | ✅ 둘 다 FAIL |
| 전환: 충분한 표본 뒤 관측자가 죽고 별도 echo 때만 복구 → FAIL | `early42` (echo 는 항상 성공하게 둠) | ✅ FAIL |
| 부정: 의도한 timeout 이 아닌 신호/CLI 오류 종료 → FAIL | `signal`(rc 130) | ✅ FAIL |
| 역회귀: 동일 관측자가 요구 시간 전체를 채움 → PASS | `full`(rc 124, 3002ms) | ✅ PASS |
| 부정: TODO 목록 10→9 · 10→11 → 문서 검사 실패 | 픽스처 6종 | ✅ 6/6 FAIL, 현 10건은 PASS |
| flake8: 기록한 **정확한 명령**이 종료 0 | 아래 게이트 줄 | ✅ rc 0 |

★ **검토가 안 준 경계를 스스로 하나 더 걸었다**: rc 만 보면 **즉시 124 를 뱉는 상대**에게
속는다(`insta124` — rc 는 상한 발동인데 벽시계 0ms). 그래서 rc 와 경과시간을 **함께** 본다.
그리고 `kill9`(TERM 을 씹어 SIGKILL 까지 간 rc 137)도 완전한 창으로 인정되는지 따로 확인했다.

★★ **이 보완이 실패하는 단 하나의 방식을 먼저 반증했다** (`AGENTS.md §3-4`).
"상한 발동 = 124" 가 틀리면 **D+0 에서 매번 거짓 FAIL** 이 난다 — `ros2` CLI 는 파이썬이라
SIGTERM 을 잡아 **정상 종료(exit 0)** 할 수 있고, 그러면 rc 가 0 으로 보일 수 있다.
가짜가 아니라 **GNU `timeout` 규약과 진짜 CLI** 에 직접 물었다:

```text
timeout --kill-after=2 2 bash -c 'trap "exit 0" TERM; sleep 30'   → rc 124   (자식이 0 이어도 124)
timeout --kill-after=2 2 bash -c 'trap "exit 7" TERM; sleep 30'   → rc 124
timeout --kill-after=2 3 ros2 topic hz /__nonexistent_probe       → rc 124 · 3223ms  ← 진짜 CLI
```

`--preserve-status` 를 쓰지 않는 한 timeout 은 **자식의 종료 상태와 무관하게 124** 를 준다.
마지막 줄이 핵심이다 — 이 판정의 전제를 **가짜 `ros2` 가 아니라 실제 `ros2 topic hz`** 로 확인했다.

**★ 실행 증거 — 명령 그대로** (결과만 적지 않는다. 이번 P2-2 가 정확히 그 실패였다)

```bash
python3 -m flake8 --max-line-length=99 tools/bag_gap_report.py tools/todo_d0_scan.py   # rc 0
bash -n tools/d0_check.sh && bash -n tools/doc_check.sh                                # rc 0
python3 -m py_compile tools/bag_gap_report.py tools/todo_d0_scan.py                    # rc 0
python3 tools/scan_unbounded_cli.py tools/d0_check.sh                                  # 0건 · rc 0
python3 tools/todo_d0_scan.py                                                          # 목록 10건 = 5자리 · rc 0
python3 -m pytest src/mission_manager/test/ -q                                         # 182 passed
colcon test && colcon test-result            # 245 tests, 0 errors, 0 failures, 3 skipped
bash tools/doc_check.sh --strict                                                       # PASS
```

⚠ **`--max-line-length=99` 는 골대를 옮긴 것이 아니다.** 이 저장소의 실제 lint 게이트는
`colcon test` 안의 `ament_flake8` 이고, 그 설정이 99열이다
(`/opt/ros/humble/lib/python3.10/site-packages/ament_flake8/configuration/ament_flake8.ini:4`
→ `max-line-length = 99`). **기본 79열로 재면 지금도 15건이 남는다** — 그 사실을 여기 적어 둔다.
그리고 이 명령은 **이번에 만진 2파일만** 검사한다. 저장소 전체는 99열에서도 통과하지 않는다
(`tools/scan_unbounded_cli.py` E702·W605 등). **전 저장소 lint 를 통과했다고 주장하지 않는다.**

**게이트**: pytest **182** · colcon **245**(0e·0f·3s) · `doc_check --strict` PASS ·
`bash -n` PASS · `scan_unbounded_cli` 0건 · flake8(99열, 2파일) rc 0. `src/**` diff **0**.

⚠ **검증 상한 (닫지 않고 공개한다)**
- **여전히 실차가 없다.** bag 은 우리가 만든 합성 sqlite3 이고 `ros2` 는 가짜다.
  실제 RPLIDAR C1 의 `/scan` 주기 분포는 **R3 에서 처음 본다** — 그래서 `/scan` 에 간격
  '계약'을 두지 않고 liveness 만 걸었다. **없는 실측을 숫자로 발명하지 않기 위해서다.**
- `/scan` liveness 한계 **200ms = 사양 100ms × 2** 의 근거는 *"녹화 양끝은 발행 위상과
  안 맞으므로 건강한 토픽도 한 주기까지는 빈다"* 이다. **R3 실측이 오면 이 값과 함께
  간격 계약 자체를 다시 정한다** (`TOPIC_POLICY` 한 곳만 고치면 된다).
- 이번 회귀는 **저장소에 박히지 않았다**(합성 bag·가짜 `ros2` 는 스크래치패드에서 실행).
  하네스 편입은 **예약 21** 이고 인수 후 한 묶음에서 기준선을 한 번만 흔들며 처리한다.
- **경과시간은 벽시계(`date +%s%N`)다 — 단조 시계가 아니다.** 관측 창 도중 NTP 가 시각을
  **뒤로** 되돌리면 경과시간이 모자라 보여 거짓 FAIL 이 날 수 있다. 방향은 fail-closed 이고
  메시지도 *"시스템 시각 또는 CLI 를 의심한다"* 로 안내한다 — 그리고 그 상황은 애초에
  `JETSON_SETUP.md §1-b` 가 경고하는 **EKF 를 멈추는 조건**이라 FAIL 이 맞다. 그래도
  *"이 검사는 시계가 단조라는 가정 위에 있다"* 는 사실은 여기 적어 둔다.
- `todo_d0_scan.py` 는 `docs/*.md` 만 본다. `FREEZE_MANIFEST.md`·`legacy/`·'직전 완료' 블록은
  그 시점 기록이라 제외한다(`gate_baseline_scan.py` 와 같은 규약). 근접 창(±100자)보다
  멀리 떨어뜨려 쓰면 못 본다 — **표현을 바꿔 우회하는 것까지 막지는 못한다.**

### 10.7 검토 §31 P2 보완(`66db5c5`) 과 그 독립 검토 §32 (2026-08-03, **동결 예외 아님**)

★ **이 절도 동결 예외 기록이 아니다.** `src/**` 를 한 글자도 열지 않았다(`tools/`·`docs/` 전용).

★★ **이 묶음에서 역할이 한 번 뒤바뀌었다.** 사용자 승인 아래 **검토자(Codex)가 예외적으로
구현자**가 되어 §31 의 P2 2건을 직접 고쳤고(`66db5c5`), 그 diff 를 **Claude 가 독립 검토**했다
(=§32). "구현자 ≠ 최종 승인자"는 유지됐다. 규칙은 `AGENTS.md §5` '역할 교대 예외'로 명문화했다.

**앞선 판정** = `0d9108a` 독립 검토 **승인 (P0 0 · P1 0 · P2 2)**
(전문 = `~/Desktop/개발현황/CODEX 현황/0801검토현황.md §31`). §30 의 두 P1 은 닫혔다.

| 지적·발견 | 무엇이 틀렸나 | 보완 | 누가 |
|---|---|---|---|
| §31.2 P2 | 활성 런북이 `TOPIC_POLICY` 를 "단일 출처"라 **선언하면서 같은 숫자를 다시** 적었다. 값이 우연히 같아 판정은 맞았지만 R3 가 계약을 갱신하면 런북만 옛 숫자로 남는다 | `D1_FIRST_STEP §5-b` 의 숫자 표를 **의미 표**로 줄이고 한계는 도구 출력·`TOPIC_POLICY` 를 가리키게 | Codex |
| §31.3 P2 | D+0 **마지막 EKF 재확인**이 `topic info \| awk \| grep` 파이프라인이고 `pipefail` 이 없어 `$?` 가 마지막 `grep` 것이었다 → **완성된 EKF 행을 찍은 뒤 매달려 rc 124** 여도 통과 | 출력을 파일로 받고 **rc 를 즉시 포획**, `rc != 0` 이면 EKF 행이 보여도 **판독 실패**로 FAIL. rc 0 일 때만 awk/grep 판독 | Codex |
| **§32.2 P1(프로세스)** | `66db5c5` 가 '현재 단계' 체인에 칸을 하나 더 붙여 **상단 §31 · 진행표 §30 · 본문 §30** 이 됐다. 그런데 `handoff_single_check.sh` 는 `→ **검토 §N**` **한 문구만** 찾아 그 꼬리를 **못 봤고 `OK`** 를 냈다 — 계약이 깨진 상태를 **게이트가 초록으로 승인** | 검사 패턴을 낱말이 아니라 **구조**(화살표 뒤 굵은 글 **안**의 `§N`)로 넓히고, 핸드오프 세 자리를 §32 로 일치시켰다 | Claude |
| **§32.3 P2(잔여 클래스)** | §31.2 는 **지목된 런북 한 곳만** 닫혔다. 같은 클래스가 **활성 D+0 런북**에 남아 있었다 — `JETSON_SETUP.md` 판정표의 `최대 간격 ≤ 33.33ms`(= `d0_check.sh` `GAP_MAX_MS` 의 복사본) | 그 표에서 숫자를 빼고 "도구가 판정할 때 자기 계약을 함께 찍는다"로 바꿨다. `CURRENT_HANDOFF` 재개방 조건·`REAL_ROBOT_VALUES §1` 도 **EKF 입력 2종에만** 걸리도록 토픽을 명시 | Claude |

**★ 클래스 전수 열거 → 항목별 대조** (`AGENTS.md §3-10`)

| 클래스 | 열거 명령 | 나온 자리 | 각 자리가 덮이는가 |
|---|---|---|---|
| D. 활성 문서가 판정 도구의 숫자 계약을 **재저장** | `grep -rn "33\.33" docs/*.md` (legacy 제외) | **6자리** | ✅ `D1_FIRST_STEP`(Codex 가 제거) · `JETSON_SETUP:462`(제거) · `CURRENT_HANDOFF` 재개방 조건(토픽 명시 + 도구 지시로 교체) · `REAL_ROBOT_VALUES:44`(도구가 판정한다고 명시) / **무해 2자리** = `CURRENT_HANDOFF` 의 *"구판이 33.33 하나를 모든 토픽에 적용했다"* 는 **과거 사실 서술**이고 `REAL_ROBOT_VALUES:33` 은 여유 산술의 **근거 상수**다 |
| E. `hard_timeout` 뒤 종료 상태를 안 묻는 자리 | `grep -n "hard_timeout " tools/d0_check.sh` | 호출 **8자리** | ✅ 8/8. §10.6 에서 7자리, **마지막 1자리를 `66db5c5` 가 닫았다.** 이제 파이프라인으로 rc 를 가리는 자리가 **0개**다 (전 자리가 파일 리다이렉트 또는 `if` 직접 사용) |
| F. 핸드오프 '한 묶음' 계약을 보는 게이트가 **문구에 의존** | `grep -n "ARROW=" tools/handoff_single_check.sh` | 패턴 **1자리** | ✅ 구조 기반으로 교체. 넓힌 패턴이 **블록 밖을 오탐하지 않는지** 전수 확인(파일 전체 8건이 모두 '현재 단계' 블록 안) |
| G. `d0_check` **밖의** 셸이 CLI 를 파이프에 물려 rc 를 가리는가 | `grep -rn "ros2 \|hard_timeout " tools/*.sh \| grep "\|"` | `lib_e2e`·`make_map`·`accuracy_bench` **14자리** | ✅ **결함 아님** — 전부 `until … \| grep -q …; do` 형태의 **준비 대기 술어**이고 "아직 안 맞음 = 계속 기다림"이 의도다(상한은 바깥 deadline). 값 추출 2자리(`lib_e2e:102·106`)도 빈 문자열이면 실패로 떨어진다. **판정 rc 로 쓰는 자리는 `d0_check.sh` 뿐**이라 클래스가 다르다 — 세고 나서 그 사실을 여기 적어 둔다 |

**회귀 관찰값** — 명령과 관찰값을 함께 남긴다(`§10.6` 에서 배운 규칙)

| 무엇 | 명령 | 관찰값 |
|---|---|---|
| 재확인 **역회귀** | `MODE=full_ok bash recheck_harness.sh` | 두 토픽 `OK 구독 유지` · rc **0** |
| 재확인 부정 — 완성 행 뒤 hang | `MODE=hang_after_row …` | 두 토픽 `rc=124 … 판독 실패` · rc **1** (경과 40초 = 20초×2) |
| 재확인 부정 — TERM 무시 | `MODE=sigkill_137 …` | `rc=137 … 판독 실패` · rc **1** (경과 44초) |
| 재확인 부정 — 완성 행 뒤 rc 42 | `MODE=rc42_after_row …` | `rc=42 … 판독 실패` · rc **1** |
| 재확인 **전환** — rc 0 인데 EKF 없음 | `MODE=rc0_no_ekf …` | `구독하던 … 가 사라졌다` · rc **1** |
| 재확인 부정 — rc 0 인데 **잘린 행** | `MODE=rc0_truncated …` | 위와 같음 · rc **1** |
| 재확인 부정 — EKF 가 **PUBLISHER 로만** | `MODE=rc0_ekf_pub_only …` | 위와 같음 · rc **1** (구독이 아니면 유지가 아니다) |
| 재확인 — 토픽별 **혼합** | `MODE=mixed …` | `/odom` OK · `/imu/data` FAIL · rc **1** (토픽마다 따로 판정) |
| **구판 대조 (`0d9108a`)** | 같은 `hang_after_row` 를 보완 전 블록에 | 두 토픽 **`OK 구독 유지` · FAIL=0** — 검토 §31.3 의 false PASS 를 독립 재현 |
| 문서 드리프트 **시연** | `EKF_PERIOD_MS`·`GAP_MAX_MS` 를 25.00 으로 바꾸고 `doc_check --strict` | **PASS(초록)** 인데 `JETSON_SETUP:462` 는 `33.33ms` 유지 → 잔여 클래스 확증 (원복 완료) |
| 게이트 우회 **재현·차단** | 구판 패턴으로 `handoff_single_check.sh` / 새 패턴으로 | 구판 `OK (§30)` ↔ 새 패턴 `FAIL 상단=§31 진행표=§30 본문=§30` |
| 도구의 **자기 보고** | `python3 tools/bag_gap_report.py <bag> /odom /imu/data /scan` | `계약 간격 상한 33.33ms — EKF 입력 …` / `계약 liveness 한계 200.00ms — …` 를 토픽마다 출력 → 런북이 숫자를 안 적어도 현장에서 읽힌다 |

**게이트**: pytest **182** (`python3 -m pytest src/mission_manager/test/ -q`) ·
colcon **245**(0e·0f·3s) (`colcon test` → `colcon test-result`) · `doc_check --strict` PASS ·
`bash -n tools/d0_check.sh tools/handoff_single_check.sh` PASS ·
`python3 tools/scan_unbounded_cli.py tools/d0_check.sh` 0건. `src/**` diff **0**.

⚠ **검증 상한 (닫지 않고 공개한다)**
- 재확인 공격은 **가짜 `ros2`** 로 했다. 실제 ROS CLI 가 `topic info -v` 에서 rc 0 이 아닌
  값을 어떤 조건에 내는지는 **D+0 에 처음 본다**. 방향은 fail-closed 라 **거짓 통과**는 없고,
  대신 **CLI/daemon 이상일 때 D+0 이 FAIL 로 멈춘다** — 그게 의도다.
- 문서 드리프트는 **기계가 막지 않는다.** 숫자를 지웠을 뿐이고, 누가 다시 적어 넣으면
  `doc_check` 는 모른다. `TOPIC_POLICY`↔문서 값을 대조하는 검사는 **만들지 않았다**
  (활성 런북에 숫자가 없어야 성립하는 계약이라 대조할 짝이 없다). 다시 적히면 그때 만든다.
- 넓힌 `handoff_single_check` 패턴은 **화살표 + 굵은 글**이라는 표기 관습에 여전히 의존한다.
  표기 자체를 버리면 못 본다 — 문구 의존을 **한 단계 줄인 것**이지 없앤 것이 아니다.
- 넓힌 대가로 **새 계약이 하나 생겼다**: 체인 칸의 굵은 글 안 `§N` 은 **검토 번호 전용**이다.
  거기에 예약 번호나 다른 문서의 절을 굵게 적으면 그게 꼬리로 읽혀 **FAIL** 한다. 자기 반증
  결과 이 오탐 위험은 실재하지만 **방향이 fail-closed**(조용한 통과가 아니라 시끄러운 차단)
  라 받아들였다. 현재 체인의 `→ **예약 18 = §25 …**` 는 꼬리가 아니라 무해하다.
- 이번 회귀도 **저장소에 안 박혔다**(스크래치패드 실행). 편입은 **예약 21**.

### 10.8 검토 §34 보완 — AI 문맥·R0 watchdog·TODO 폐포 (2026-08-03, **동결 예외 아님**)

사용자 명시 아래 **Codex가 구현자**, Claude가 독립 재검토자다. `src/**`에서 바뀐 것은
`gate_fakes.py`의 **출처 주석·문자열**과 그 표기를 잠그는 기존 테스트 단언뿐이다. 센서 주기
값·타이머·발행·게이트 분기는 0건 변경이며 예약 19의 R3 전 동결을 유지한다.

| §34 발견 클래스 | 전수·완료판정 | 보완 |
|---|---|---|
| fenced code의 `#`를 제목으로 오인 | 착수 **17곳**, watchdog 주석 추가 뒤 최종 **18곳**; `TEST_GATES §1` 끝 문단 포함·fence 짝수 | backtick/tilde·들여쓰기 fence 상태 추적, 빈 제목 안전 처리 |
| common gate가 inventory anchor를 대신 충족 | 구현·검토 공통 ref까지 비우면 **0/59**, 정상 profile **59/59** | 지목 6개를 포함해 같은 클래스 **10 anchor**를 profile-only로 교체 |
| 한 경로의 복수 profile 유실 | tracked 복수 일치 경로를 합집합 처리 | 첫 일치 `break` 제거, JETSON 단독에서 bringup+docs 보장 |
| R0 watchdog 절차 부재 | D+0 PASS 증거 없으면 R1 금지 | `JETSON_SETUP §7-c`, `D1_FIRST_STEP §0-a`, 이 문서 §6 정합 |
| D1 목록 10건 ↔ 실행 표식 7건 | D+0 11건·D+1 10건의 목록/주장/표식 exact 계약 | phase 공통 scanner + 증감 양방향 합성 회귀 5건 |
| gate fake 출처가 구 회신을 현행처럼 표기 | 주기 값·통계 34건 무변동 | 3차 회신 뒤에도 R3 시간열까지 의도적 동결임을 명시 |

검증 = AI context **27/27** · TODO scanner **5/5** · gate-fakes **34/34** · pytest
**182/182** · harness **24/24** · colcon **245/245**(0 failure, 3 skip) · build·문서/정적 검사
PASS. readiness 통합 **13/14**는 이미 `TEST_GATES §1`에 공개된 case 12 간헐성 그대로이며,
실행 값·분기 diff 0이라 승인 근거에서 제외하고 실패 사실을 보존한다.

### 10.9 검토 §35 P2 보완 — watchdog 권위·현장 관측 창·61건 corpus (2026-08-03, **동결 예외 아님**)

Claude의 §35 독립 검토는 `b856609`를 **승인(P0 0 · P1 0 · P2 3)**했다. 사용자가 P2 세 건도
전부 구현하도록 명시해 **Codex가 구현자**, Claude가 이 후속 diff의 독립 재검토자다. 변경은
`docs/`·`tools/`뿐이며 `src/**` diff는 0이다. 예약 22·23도 열지 않았다. 바로 위 §10.8의
**59/59·27/27은 `b856609` 당시 역사적 스냅샷**이므로 덮어쓰지 않고, 현행값은 이 절에 둔다.

| §35 P2 | 같은 클래스 전수·완료판정 | 보완 |
|---|---|---|
| watchdog 문서 권위 충돌 | 활성 7문서의 역할을 모두 대조. 구동부 회신은 참고, D0 실측만 물리 권위, D1은 결과 소비자여야 한다 | `REAL_ROBOT_VALUES`의 회신을 참고값으로 내리고, `D1_FIRST_STEP §6`은 D0 FAIL·미실측·빈 기록이면 재개방·중단하도록 정정 |
| 두 명령을 한꺼번에 복사 | R0는 비영 publish 종료와 zero publish 사이 **명령 없는 2초 관측**이 구조적으로 존재해야 한다. R1 대조군은 반대로 즉시 zero를 유지해야 한다 | R0 publish를 두 fenced block으로 분리하고 관측 지시를 블록 사이에 배치. R1은 한 블록 두 publish인 역회귀로 잠금 |
| 검토 역사가 §34 앞에서 영구 절단 | Desktop 검토문을 끝까지 세어 inventory와 exact 일치, 새 P0/P1은 증가 방향도 FAIL해야 한다 | `split("\\n## 34.")` 제거, §34 P1 두 행 추가, 현행 **61/61**·common-only **0/61**. 이후 새 제목은 history/inventory 불일치로 자동 FAIL |

**★ watchdog 활성 문서 전수 열거 → 항목별 대조** (`AGENTS.md §3-10`)

열거 명령은 `rg -n -i 'cmd_vel watchdog|R0 watchdog|0\.5초.*watchdog|watchdog.*0\.5초|0\.5s.*watchdog' docs/*.md`다.
scan liveness watchdog만 말하는 `PROJECT_CONTEXT`·`PITFALLS`는 이 물리 계약 클래스가 아니어서
제외했다. 아래 7자리는 모두 서로 다른 역할 하나로 덮인다.

| 활성 문서 | 맡은 역할 | 항목별 대조 |
|---|---|---|
| `MASTER_PLAN.md` | R0 계약·순서 | ✅ 단절 0.5초 내 정지를 통과조건으로만 선언 |
| `JETSON_SETUP.md` | **유일한 물리 측정·증거 권위** | ✅ zero 없는 관측 창·영상+raw pose·PASS/FAIL 기록 |
| `D1_FIRST_STEP.md` | D0 결과 소비 | ✅ PASS만 진행, FAIL·미실측·빈값은 FREEZE 재개방+D+1 중단 |
| `REAL_ROBOT_VALUES.md` | 구동부 회신·소스 참고 | ✅ 회신 수치를 실차 증거로 승격하지 않음 |
| `FREEZE_MANIFEST.md` | 조건부 수용·재개방 상태 | ✅ R0 실측 전 미해결, 불일치 시 §6 재개방 |
| `TEST_GATES.md` | 자동 검증 상한 | ✅ CLI 관측은 물리 연속 생존·손실을 못 증명하며 R0 실측 몫임을 공개 |
| `CURRENT_HANDOFF.md` | 현행 전환·상태 | ✅ D0 실측 전제와 독립 재검토 대기를 한 묶음으로 지시 |

**회귀 관찰값**

| 무엇 | 명령 | 관찰값 |
|---|---|---|
| AI routing·history·R0/R1 구조 | `python3 tools/test_ai_context.py` | **29/29 PASS**; R0 두 block+사이 관측, R1 한 block 두 publish, 61/61·0/61 |
| D0/D1 TODO exact 계약 | `python3 tools/test_todo_scan.py` | **5/5 PASS** |
| 미션 단위 회귀 | `python3 -m pytest src/mission_manager/test/ -q` | **182/182 PASS** |
| 공격 하네스 | `bash tools/test_harness_guards.sh` | **24/24 PASS** |
| readiness 부정·역회귀 | `ROS_LOG_DIR=/tmp/codex_ros_logs bash tools/test_gate_regression.sh` | **14/14 PASS** |
| 전체 빌드 | `colcon build --symlink-install` | **5 packages PASS** |
| 전체 테스트 | `ROS_LOG_DIR=/tmp/codex_ros_logs colcon test` → `colcon test-result --verbose` | **245 tests, 0 errors, 0 failures, 3 skipped** |
| 문서·핸드오프 정합 | `bash tools/doc_check.sh --strict` | **PASS** |

⚠ **검증 상한**
- watchdog은 아직 실제 바퀴로 측정하지 않았다. 이 보완은 **측정 절차와 증거 권위 충돌**을
  닫았을 뿐 `FREEZE_MANIFEST §6`의 조건부 위험을 통과로 바꾸지 않는다.
- 검토 역사 재계수는 Desktop archive가 있는 정본 개발 PC에서만 강제된다. archive 없는
  이식 환경에서는 그 테스트를 skip하므로, portable CI가 새 발견 편입을 증명한다고 주장하지 않는다.
- fenced block 분리는 copy/paste 실수를 구조적으로 줄이지만 사용자가 두 블록을 일부러 연속
  실행하는 것까지 셸이 차단하지는 않는다. 현장 증거는 영상과 raw pose가 실제 무명령 창을 보여야 한다.
- 첫 `colcon test`는 샌드박스의 읽기 전용 `~/.ros/log`에 쓰지 못해 첫 timer test가 실패했고,
  뒤 10건이 `Context.init()` 중복으로 연쇄 실패했다(**인프라 분류**). 원인 기록 뒤 로그 경로를
  `/tmp`로 지정한 동일 245건이 위와 같이 전량 통과했다. 재실행 성공만 남기지 않고 최초 실패와
  분류를 함께 보존한다.
