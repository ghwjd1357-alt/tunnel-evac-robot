# TEST_GATES.md — 테스트·실행 정본 (게이트 + 런치 실행법)

> 모든 E2E 는 **전용 시뮬 PC 전용 (Jetson 실행 금지)**. Gazebo E2E 는 전역 프로세스 cleanup 을
> 하므로 **동시 실행 절대 금지** — 각 스크립트 완전 종료 후 다음 실행.

## 1. platform-core 구조 분리 전체 게이트 (변경 묶음 완료 시 순서대로 전량)

```bash
python3 -m pytest src/mission_manager/test/ -q          # ~0.6s
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

기준선 (07-20, SpeedManager 재검토 보완 후): pytest **122 passed** / colcon **128 tests, 0 fail, 2 skip** / E2E 4종 PASS.
⚠ 이 수치는 묶음 완료 때마다 갱신한다 (테스트가 늘었는데 기준선이 옛 수치면 회귀 검출력이 조용히 떨어진다).
**갱신을 잊어도 `doc_check.sh` 가 실제 개수와 대조해 잡는다** — 기억이 아니라 기계가 지키는 구조.
`make_map.sh` 는 이 게이트에 포함하지 않는다 (지도 자산 변경 — 명시 승인 시에만).

## 2. 각 테스트의 목적과 PASS 기준

| 검증 | 목적 | PASS 기준 |
|---|---|---|
| pytest | 단위·경계조건 (알람/그래프/디바운스/취소 레이스/validator) | 전부 passed |
| colcon test | 워크스페이스 lint+단위 | test-result 0 errors/failures |
| regression_negative | **안 돼야 하는 게 안 되는가** — 지도밖/벽너머/막힌 goal 실패 종결 + 정상 goal 양성 대조군 | 불가 3종 ABORTED + 정상 SUCCEEDED (막힌 goal 은 BT 재시도 소진까지 ~2분 정상) |
| regression_3goals | 주행 정확도 회귀 | 3종 SUCCEEDED, 최종 오차 **≤0.3m** |
| mission_e2e | 미션 전체 흐름 | GUIDE 0.12 실측 → SEARCH_BACK → 재발견 → ESCAPED |
| abort_e2e | "취소 호출"≠"실제 정지" 검증 | FAULT + 5초 이동 ≤0.10m + nonzero cmd_vel 0 (angular.z 포함) + 취소 접수 로그 |

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
