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
```

기준선 (07-20, SpeedManager 추출 후 `f94da44`): pytest **108 passed** / colcon **114 tests, 0 fail, 2 skip** / E2E 4종 PASS.
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
