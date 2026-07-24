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

즉 **동결 = "결함 0"이 아니라 "무엇을 안고 얼렸는지 양쪽이 같은 문장으로 합의한 상태"**다.
