# TEST_GATES.md — 테스트·실행 정본 (게이트 + 런치 실행법)

> 모든 E2E 는 **전용 시뮬 PC 전용 (Jetson 실행 금지)**. Gazebo E2E 는 전역 프로세스 cleanup 을
> 하므로 **동시 실행 절대 금지** — 각 스크립트 완전 종료 후 다음 실행.

## 1. platform-core 구조 분리 전체 게이트 (변경 묶음 완료 시 순서대로 전량)

```bash
python3 -m pytest src/mission_manager/test/ -q          # ~0.6s
bash tools/test_harness_guards.sh                        # ~15초 — E2E 하네스 유한 상한 단위(§14 P1)
colcon test --packages-select mission_manager tunnel_sim
colcon test-result --verbose                             # ⚠ 종료코드 말고 이걸로 판정
bash tools/regression_negative.sh                        # ~6분
bash tools/regression_3goals.sh                          # ~4분
bash tools/mission_e2e.sh                                # ~3분
bash tools/abort_e2e.sh                                  # ~3분
bash tools/doc_check.sh                                  # ~1초 — 문서 동기화 (커밋 직전)
#   … 커밋 + push …
bash tools/doc_check.sh --after-push                     # 원격 동기 재확인 (필수)
```

⚠ **`--after-push` 를 빼면 안 된다.** 커밋 직전 실행 시점의 HEAD 는 아직 *이전* 커밋이라,
새 커밋을 만들고 push 를 잊어도 앞 단계에서는 잡히지 않는다 (Codex 07-20 지적).
`--strict` 를 붙이면 생략된 검사(colcon 결과 없음 등)도 실패로 취급한다.

기준선 (07-24, **platform-core-freeze 동결 기준점** — tag `platform-core-freeze-260724` @ `212885a`): pytest **159 passed** / colcon **165 tests, 0 fail, 2 skip** / E2E 4종 PASS **+ 쌍굴 PASS**.
(구조 분리 3/3 완료 시점과 같은 수치 — 하네스 추출은 순수 리팩터라 개수 무변동.
0723검토 **§8·§9 가 같은 수치를 두 번 독립 재현**했다.)
★ 이 수치는 이제 단순 기준선이 아니라 **동결 기준점**이다. 앞으로 회귀가 나오면
`git diff platform-core-freeze-260724 --stat` 이 1차 용의선상 — 증거 전량 = `docs/FREEZE_MANIFEST.md`.
⚠ 이 수치는 묶음 완료 때마다 갱신한다 (테스트가 늘었는데 기준선이 옛 수치면 회귀 검출력이 조용히 떨어진다).
**갱신을 잊어도 `doc_check.sh` 가 실제 개수와 대조해 잡는다** — 기억이 아니라 기계가 지키는 구조.
`make_map.sh` 는 이 게이트에 포함하지 않는다 (지도 자산 변경 — 명시 승인 시에만).

## 2. 각 테스트의 목적과 PASS 기준

| 검증 | 목적 | PASS 기준 |
|---|---|---|
| pytest | 단위·경계조건 (알람/그래프/디바운스/취소 레이스/validator) | 전부 passed |
| test_harness_guards | E2E 하네스 유한 상한 (`read_param_float` 복구 상한·`wait_state` 벽시계 deadline) — Gazebo 불필요 | 5 케이스 전부 ✓ (§14 P1) |
| colcon test | 워크스페이스 lint+단위 | test-result 0 errors/failures |
| regression_negative | **안 돼야 하는 게 안 되는가** — 지도밖/벽너머/막힌 goal 실패 종결 + 정상 goal 양성 대조군 | 불가 3종 ABORTED + 정상 SUCCEEDED (막힌 goal 은 BT 재시도 소진까지 ~2분 정상) |
| regression_3goals | 주행 정확도 회귀 | 3종 SUCCEEDED, 최종 오차 **≤0.3m** |
| mission_e2e | 미션 전체 흐름 | GUIDE 0.12 실측 → SEARCH_BACK → 재발견 → ESCAPED |
| abort_e2e | "취소 호출"≠"실제 정지" 검증 | FAULT + 5초 이동 ≤0.10m + nonzero cmd_vel 0 (angular.z 포함) + 취소 접수 로그 |

★ **mission_e2e SEARCH_BACK 예산 = 180s** (`T_SEARCHBACK`, T자·쌍굴 공통 — 07-24 e2e-harness-fix
재산정, **옛 90s 대체**). 근거는 아래 둘을 함께 남긴다(둘 중 하나만으론 '숫자만 올린 기준 완화'다):
- **관측 도달 분포**: 표준환경 신규 7회(T자 3·쌍굴 4) 전부 **9s** 로 수렴. 역사 표본은 **9s ~ ≈90s**
  (daemon flake·Nav2 플래닝 지연 환경 — `FREEZE_MANIFEST.md §8`). GATHER 도달은 T자 9s·쌍굴 45~48s
  (역사 48/126/48). 변동성은 지형이 아니라 인프라 상태에 좌우돼 강제 재현이 어렵고, 역사 outlier 가
  최악을 실증한다.
- **상한이 보호하는 것**: "미션은 건강한데 팔로워 간격 벌어짐(≥(2.5−1.2)/0.12 ≈ 11s) + `lost_sec` 3s
  디바운스 + 재플래닝·goal 재전송 지연 + detection flicker 로 놓침 확정이 늦어지는 최악". 관측 최악
  ≈90s(옛 예산 경계에서 스쳐 실패)의 **2배 마진**. 이 상한을 넘으면 '건강한 지연'이 아니라 놓침이
  구조적으로 확정 안 되는 이상(follower `stop` 미수신·`lost` 미발화)이므로 **여전히 FAIL** 해야 한다.
  실측 부정 회귀: 도달 불가 예산에서 `FAIL: … 대기 타임아웃(예산 Ns, 경과 Ms), 마지막 상태='GUIDE'` —
  판정과 '마지막 상태' 보고가 같은 읽기라 자기모순이 없다(옛 폴링 race 제거).
- ★ **상한 집행은 벽시계(`SECONDS`)로 한다** (07-24 §14 P1 보완): `wait_state` 예산과 `read_param_float`
  복구 시퀀스(param get + daemon 재시작 각 timeout, 상한 ≈26s)를 sleep 누적이 아니라 실경과시간으로
  지킨다. 예산 밖에서 늦게 도달하면 s 가 목표여도 `경과>예산`이 메시지에 찍혀 FAIL(모순 없음). 그
  부정 회귀는 `tools/test_harness_guards.sh` 가 Gazebo 없이 격리 검증한다.
- ⚠ 이 예산은 **판정 기준**이라 동결 태그 `platform-core-freeze-260724` 의 게이트 수치는 **옛 90s
  기준**으로 얻은 것이다 — 두 기준의 구분은 `FREEZE_MANIFEST.md §8`.

변경 영역별 최소 조합: 코드 변경 = pytest 무조건 + 해당 영역 E2E / goal·취소·미션 명령 = +abort_e2e /
안전 정책·Nav2 설정·정본 지도 수동 교체 = +negative / 구조 분리 묶음 완료 = §1 전량.

## 3. 런치/실행법

- **미션 전체**: `ros2 launch tunnel_sim mission.launch.py` (`gui:=false` 헤드리스 / `follower:=false`).
  **기본 = localization 운영 모드** (저장 posegraph). 라이브 SLAM 은 `localization:=false`.
- **쌍굴**: `ros2 launch tunnel_sim mission_twin.launch.py` — 화재 `/alarm` x:30,y:0. 좌표: map=world+(17,0), 통로 x=7·17·27. E2E `bash tools/mission_e2e.sh twin`.
- 화재 주입: `ros2 topic pub --times 2 -w 1 /alarm geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 14.0, y: 0.0}}}"`
- 놓침 재현: `ros2 topic pub --times 3 -w 1 /follower_cmd std_msgs/msg/String "{data: stop}"` (재개 `follow`) / 관찰 `ros2 topic echo /mission_state` (⚠ `--once` 금지)
- 관제 명령: `/mission_cmd` 에 `reset`(→PATROL) / `abort`(FAULT 유지)
- **관제 데스크톱**: `bash ~/ros2_ws/console/run_console.sh` (rosbridge:9090 + 웹 :8000) — 시뮬과 병행. 설계 = `0718_관제시스템.md`
- 실험용: `slam_nav2.launch.py`(라이브SLAM+Nav2만) / `robot.launch.py`(Gazebo만) / goal 단발: `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: …, y: …}, orientation: {w: 1.0}}}}"`
- **Gazebo GUI**: `sudo prime-select on-demand`(재부팅) 후 `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia ros2 launch …`. 종료 Ctrl+C 또는 `pkill -9 gzserver`. 평소 `sudo prime-select intel`.

## 4. 지도 생명주기 (★ 실런은 명시 승인 + 정기 지도 제작 때만)

`bash tools/make_map.sh [twin]` — staging 저장 → localization 스모크(TF-GT ≤0.3m) →
(T자는 staging 지도로 negative **자동** 수락 게이트) → `tools/map_promote.sh` fail-closed 4파일 transaction
(백업+manifest, 중간 실패 rollback). 격리 하네스 `test_map_promote.sh` 8검사.
★ 수락 기준 = 스모크 통과 ≠ 수락, **negative 통과까지** (F1 개정 — 자산 교체도 부정 회귀 대상).

## 5. 인프라 실패 분류법 (E2E 실패 시 — 원인 한 줄 기록 전 재실행 금지)

기지 함정과 로그 물증으로 대조: ① goal 응답 유실 (bt_navigator "Failed to send goal response" — 타임아웃+1회 재전송이 표준) ② **좀비 bt_navigator 가로채기** (자기 로그 "Begin navigating" 0건이 판별법 → `pkill -9 -f "lib/nav2[_]"`) ③ ros2 daemon 오류 (`ros2 daemon stop/start`) ④ `/alarm` 유실 (상태 확인 후 재발사) ⑤ 샌드박스 UDP 차단 (Gazebo "Unable to get local interface addresses"). 인프라면 재시도+스크립트 방어 추가, 코드면 수정.

**⑥ GLX 깨짐으로 gzserver 사망** (07-24 신규): 증상은 "Nav2 기동 타임아웃"이지만 진짜 원인은 **로봇 미스폰**이다. 판별 = launch.log 에 `X Error … X_GLXCreateContext BadValue` + `Service /spawn_entity unavailable`. **`glxinfo -B` 단독으로 같은 에러가 나면 우리 코드와 무관**(PC 그래픽 상태). 즉시 회피 = `env -u DISPLAY bash tools/…`(라이다가 CPU `type="ray"` 라 렌더링 불필요). 근본 복구는 **그래픽 스택 자체를 고치는 것** — 07-24 실사례의 원인은 밤사이 **apt 자동 업데이트 실패**였다. ⚠ `prime-select` 값(`on-demand` 등)을 원인으로 단정하지 말 것: 07-24 에 그 오진이 있었고, 복구 후에도 `on-demand` 인 채 GLX 는 정상이었다.

**⑦ 잔류 cmd_vel 활주** (07-24 신규): `abort_e2e` 의 "실정지"가 깨졌는데 미션 로그의 취소 사슬은 정상 종결(≤100ms)인 경우. `libgazebo_ros_diff_drive` 는 command timeout 이 없어 **마지막 cmd_vel 을 무한 유지**하므로, 중단된 Nav2 회복행동(BackUp 0.05m/s)이 남긴 속도로 로봇이 계속 미끄러진다. 판별 = 이동 속도가 `backup_speed` 와 일치 + launch.log 에 `backup failed`. 이건 미션 코드 결함이 아니다 — 상세 `docs/FREEZE_MANIFEST.md §6`.

## 6. 정확도 벤치 (SLAM·Nav2 튜닝 전/후)

`bash tools/accuracy_bench.sh 라벨` → `bench_out/라벨/` / 비교 `python3 tools/accuracy_report.py A/trace.csv B/trace.csv --labels 전 후 -o compare.png`.
★ 수치는 Gazebo world-odom **시뮬 상한** — 실차 정확도로 인용 금지. 실차는 rosbag+줄자 기준점으로 별도 측정.

## 7. 검토자(Codex) 실행 범위 — 변경 표면 라우팅 (07-20 신설)

> §1 은 **구현자** 기준(전량)이다. 검토자가 그대로 따라 하면 같은 커밋에 E2E 를 두 번
> 태우게 된다. 근거: 07-20 검토 2회에서 E2E 9회가 전부 PASS 했고, **P1 3건은 전부
> 단위 harness 가 잡았다** — E2E 재실행의 결함 검출 기여는 0건이었다.
> 반면 `regression_negative`·`regression_3goals` 는 `mission_node` 를 **띄우지도 않는다**
> (`grep -c "mission_manager mission_node" tools/*.sh` = 0) — 미션 로직 변경과 인과가 없다.

**항상 (싸고 위조·환경의존 검출력 있음)**:

```bash
python3 -m pytest src/mission_manager/test/ -q
colcon test --packages-select mission_manager tunnel_sim   # ⚠ 반드시 직접 실행
colcon test-result --verbose                                # 판정만 — 재실행 아님
bash tools/doc_check.sh --strict                            # 문서 검사 본체
bash tools/doc_check.sh --after-push                        # 원격 ahead/behind 만
```

⚠ **`colcon test-result` 는 테스트를 돌리지 않는다** — 기존 `build/` 산출물을 읽을 뿐이라,
`colcon test` 없이 이것만 보면 *구현자가 남긴 결과를 재독해*하는 것이고 위조·환경의존은
검출되지 않는다 (Codex 07-20 §11.5 지적). 검토자는 `colcon test` 를 직접 실행한다.
⚠ `--after-push` 도 **문서 검사가 아니라 원격 동기 확인 전용**이다. 문서 검사 본체는
`doc_check.sh` (엄격 모드 `--strict`).

시간 배분의 중심은 **공격 harness** — 검토자의 실제 가치다.

**E2E 는 변경 표면으로 고른다:**

| 변경 표면 | 검토자 필수 E2E |
|---|---|
| `mission_node.py` · `speed_manager.py` · `goal_manager.py` | `mission_e2e` · `abort_e2e` |
| Nav2 파라미터 · costmap · planner · URDF | `regression_negative` · `regression_3goals` |
| 지도 자산 | `regression_negative` + 승격 evidence |
| 셸 도구 · 문서 전용 | 없음 (`doc_check.sh` + `bash -n`) |
| **동결 게이트** (platform-core-freeze · mission-logic-RC · mission-v1-freeze) | **전량 + 쌍굴 + 지도 승격 evidence** |
| **위에 없는 런타임 변경** (`follower_monitor.py` · waypoints yaml · launch/wiring 등) | ★ **fail-closed — 검토자가 관련 E2E 를 직접 선정**하고 그 근거를 검토본에 남긴다. "표에 없으니 생략"은 금지 |

**조기 판정**: P0/P1 을 재현했으면 **그 시점에 판정하고 남은 게이트는 생략**한다.
불승인이 확정된 커밋에 E2E 를 더 태울 이유가 없다 — 보완 후 어차피 다시 돈다.

**쌍굴 mission** 은 §1 전체 게이트에 포함되지 않는다 (동결 게이트 전용).
실패 시 재실행 규칙은 §5 그대로 — **원인 한 줄 분류 전 재실행 금지.**

★ **동결 게이트 행의 첫 실적 (07-24)**: `platform-core-freeze` 가 이 행 그대로 수행돼
Codex 가 전량 + 쌍굴 + hash evidence 를 독립 재현했다 (`docs/FREEZE_MANIFEST.md §9`).
그때 배운 것 2가지를 다음 동결(`mission-logic-RC`)에 미리 반영한다:
1. **쌍굴은 타이밍 분산이 커서 1회 PASS 로 판단하면 안 된다** — 실측 GATHER 48~126s,
   SEARCH_BACK 9~≈90s. 4회 중 2회가 하네스 경계에서 깨졌다 (`FREEZE_MANIFEST.md §8`).
   ⚠ 그 하네스 결함 2건은 **아직 미수리**(`MASTER_PLAN.md §7` 예약 6·7) — 다음 동결 전에 고친다.
2. **동결 커밋에는 수정을 섞지 않는다** — 게이트 중 결함이 나오면 예약으로 분리한다.
   섞는 순간 "무엇을 얼렸는가"가 흐려진다.
