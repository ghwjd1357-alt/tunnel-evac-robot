# PITFALLS.md — 영역별 함정 전체 목록 (해당 영역 작업 전에 그 절만 읽기)

> 세션 파괴급 5종 축약본은 `AGENTS.md §4` (매 세션 자동 적용). 여기는 전체 정본.
> 각 항목 끝 화살표 = 발견 경위·상세가 있는 날짜별 현황.

## 1. 프로세스/셸

- **`pkill/pgrep -f` 자기 매칭 자살** — 명령 문자열에 프로세스명이 들어가면 자기 셸 kill(2회 실측) → 브래킷 트릭 `pgrep -f "ros2[ ]launch"`. **`pkill -x` 는 comm 15자 잘림** — `robot_state_publisher`→`robot_state_pub`, `async_slam_toolbox_node`→`async_slam_tool` → 긴 이름은 `pkill -f "…[_]…"`. → 0705_현황 §10
- **`ros2 launch` 백그라운드 부모부터 kill** — 부모 생존 시 자식(planner·bt_navigator·gzserver) 좀비·중복 라이프사이클. **부모 kill -9 는 고아 nav2** → 좀비 bt_navigator 가 goal 가로챔(증상 매번 다름) → cleanup `pkill -9 -f "lib/nav2[_]"` 필수. → 0705_현황 §18
- 재시작 시 좀비 누적 + `TF_OLD_DATA` 폭주 → `pkill -9` 전부 + `ros2 daemon stop/start`. 백그라운드 런치는 수동 `setsid nohup` 불안정 → Claude 는 Bash `run_in_background`. → 0626_현황
- **셸 `set -u` 는 ROS setup.bash source 뒤에** (setup.bash 가 미정의 변수 참조 → 즉사). trap 은 `INT TERM` + `exit 130/143` (중단 후 계속 실행 봉쇄). → 0705_현황 §15.7
- `service call` 응답 파싱은 repr 형식 `success=True` (`success: True` 아님). → 0719_현황 §8

## 2. ROS 토픽·통신·테스트 자동화

- **`topic pub --once` 유실** — 디스커버리 매칭 전에 쏘고 죽음 → `-w 1 --times 2~3`. → 0705_현황 §14.3
- 상태성 신호(/siren 등)는 전환 1회가 아니라 **매 tick 반복 발행** (늦은 구독자 대비).
- 자동화에선 명령 도달을 **상대 노드 로그로 수신 확인** 후 다음 단계. 스폰류 서비스는 멱등으로.
- **오래 뜬 액션 서버 + 새 CLI = goal 응답 유실 가능** (send_goal 영구 대기) → 타임아웃+1회 재전송. → 0705_현황 §15.7
- 테스트 가짜 시계는 **정수 ns 누적** (float 0.1×11=1.0999… 로 디바운스 경계 미달 가짜 실패).
- rosbridge 서비스 타입은 `rosapi_msgs/srv/…`. → 0718_관제시스템

## 3. 런치/setup.py

- 런치·월드 파일은 **`setup.py` `data_files` 등록 필수** (+`import os`, `from glob import glob`). → 0621_현황
- `parameters` 키는 `declare_parameter` 이름과 정확히 일치 / remap 은 발행·구독 양쪽 / `get_parameter().value` 의 `.value` 누락 시 객체 반환 에러.
- **시뮬 자작 노드 `use_sim_time:=true` 필수.** 정본 설정은 운영 기준 — 특수 세션(mapping 등)은 명시 오버레이로 (회귀에 없는 모드가 정책 변경의 사각지대). → 0719_현황 §14

## 4. Gazebo/SDF/GUI

- `<pose>` 는 물체 **중심점** (벽 1m 면 z=0.5) / 각도 라디안 / **collision 없으면 LiDAR 통과** → visual+collision 세트. `.world` 수정 후 colcon build 필수. → 0621_현황 §14
- 가상 LiDAR = link+`<sensor type="ray">`+plugin 3층. 플러그인은 **/scan 만 발행, TF 안 줌**. RViz LaserScan QoS=Best Effort. → 0623_현황 §10
- GUI 5종: Insert 는 `models.gazebosim.org` 만(Fuel 금지) / 무거운 모델 = gzclient 크래시 / gzclient 죽어도 gzserver 잔존 / Save World As 안 뜸 → `gz model -m 이름 -p` 좌표를 `<include>` 에 / Insert 온라인 hang. → 0623_현황 §8
- 월드 생성은 생성기 스크립트 방식 (`tools/gen_twin_world.py`) — 손편집 금지.

## 5. URDF/diff_drive

- **belly drag** — 몸통 바닥이 지면에 닿으면 안 감 → 클리어런스(z=0.05). 바퀴 반경 교체 시(0.10→0.065) 차고 재조정 재발 주의.
- **4륜 강구동 = Gazebo 과구속** (제자리회전 불가) → 시뮬은 2구동휠+캐스터. 모든 link `<inertial>` 필수. 마찰은 `mu1/mu2`, `kp` 낮추면 파묻힘.
- **diff_drive `/odom` 은 spawn 위치 포함 world 좌표** (0,0 아님) → 목표 좌표는 `tf2_echo map base_footprint` 로 확인.
- ★ **`odometry_source` 기본 = world = 치트 오돔** (실위치 복사) — 오돔 실험·실차 근접 시뮬엔 `encoder` 명시. → 0705_현황 §10

## 6. Nav2

- map_server `yaml_filename not initialized` → nav2_params 에 `map_server: {yaml_filename: ""}` 섹션.
- **rolling global costmap + static layer 금지 (라이브 SLAM)** — 간헐 '성공+빈경로' ABORTED → `rolling_window: false` 가 표준. (구 "rolling:true 해결"은 폐기)
- DWB "No valid trajectories" → RPP 교체가 강건. RPP collision 오탐은 시뮬 한정 `use_collision_detection:false` (실차는 ON 복귀).
- **goal header.stamp = 0** (지금시각 찍으면 TF extrapolation → 회복스핀 무한). **콜백 블로킹 금지** (rclpy 싱글스레드 — server_is_ready 즉답+재시도 패턴).
- ★ **라이브 SLAM 중 goal 은 12m 이내 징검다리** (먼 goal = "off the global costmap" — localization 모드는 무관). → 0707_현황
- `inflation_radius ≥ 로봇 외접반경` (현 0.9). BackUp 은 Humble 기본 BT 의 RoundRobin 4번째 → 커스텀 BT 로 순서 당김(`bt_nav_to_pose_backup_first.xml`).
- 막힌 goal 은 BT 재시도 소진까지 ~2분 후 ABORTED — 무한 아님.

## 7. SLAM/EKF

- **TF 이중발행 금지** — EKF 켜면 diff_drive `publish_odom_tf:false`. Teensy 도 odom TF 발행 금지(실차 — EKF 단독).
- **EKF 엔 위치 말고 속도 융합** (odom0_config = vx·vy·vyaw). 시뮬 IMU 에 노이즈 부여 (covariance=0 이면 신뢰도 판단 불가).
- **`*_variance_penalty` 는 '분모'** — 키우면 벌점 약화! odom 신뢰 ↑ = 작게(0.02). ★ 긴 복도 드리프트 = 매처가 "안 움직였다" 선호 → `correlation_search_space_dimension` 축소 + 벌점 강화 + **회전도 잠가야 한 세트** (`angle_variance_penalty 0.05`·`minimum_angle_penalty 0.1`).
- **EKF 만으론 안 끝남** — SLAM 이 odom 을 덮어쓰므로 slam_params 튜닝까지 한 세트. 실차 시작값 = `slam_params_realodom.yaml` (penalty 0.02 는 실 odom 에 과신).
- **라이다 평면은 몸통 최상면보다 위** (자기타격 = 유령 상자 행렬 — 실물 동일, 하드웨어팀 전달). 낮은 장애물(콘 ~0.3m)은 라이다 사각 → Orbbec depth 필수의 실증 근거.
- 이동 중 gz vs tf2_echo 나이브 비교 = 샘플링 시차가 가짜 드리프트 — **정지 스냅샷만 신뢰.** → 0719_현황 §16

## 8. 미션노드 프로그래밍 (비동기 안전)

- **비동기는 '호출'이 아니라 '완결' 검증** — goal: 응답 수락 확인+stale 은 즉시 cancel+최종 CANCELED 관찰 / cancel: `goals_canceling` 확인 / 속도: `successful` 확인+재시도+**늦은 응답의 상태 덮어쓰기 가드**. "취소를 호출했다"≠"로봇이 멈췄다".
- **콜백 예외 방어** — `future.result()` 전파 시 goal_active 영구 대기 → try + FAULT 정리.
- **scan watchdog** — /scan 끊기면 visible/lost **판단 보류** (라이다 사망='추종 양호' 오독 방지). 복구 순간 디바운스 타이머 재무장 (단절 시간 산입 금지). `header.stamp` 단조성 확인.
- **외부 입력 신뢰경계** — /alarm: 유한값·frame=map·그래프 투영거리 상한. waypoints: validator (숫자·유한·부호·정수·상한·상호관계 — ★bool 은 int 하위 타입). 그래프 실패 시 직선 fallback 금지 → yaml 고정 집결지.
- ★ **플래그 하나가 두 뜻을 겸직하면 안전망이 조용히 죽는다** — `synced` 가 "한 번이라도 성공"으로 세팅되고 "지금 desired 가 반영됨"으로 읽혀, 값이 어긋난 채 종결돼도 아무도 복구를 안 했다. **원하는 값(desired)과 반영된 값(applied)은 따로 추적한다.** → `0720_현황.md §20.4`
- ★ **"안전 조치가 담당한다"고 적은 비차단 위험은 그 조치가 실제로 복구하는지까지 확인** — `_final_fail` 은 로그만 남겼고, 미뤄둔 위험이 다음 검토에서 P1 으로 돌아왔다. → `0720_현황.md §20.2`
- **비동기 요청엔 요청 단위 deadline** — '서비스 미준비'만 재면 "서비스는 떴는데 응답이 영영 안 옴"을 못 잡는다(로봇은 정지라 안전하지만 고장이 관제에서 은폐). 재시도가 예산을 연장하지 않게 무장은 1회. → `0720_현황.md §21.2`
- 단위테스트는 `MissionNode.__new__` 기법으로 rclpy 없이 콜백 직접 구동. → 0719_현황 §12~13
- ★ **"A 는 B 가 담당한다"고 범위를 나눌 땐 B 가 그 경로 전부를 덮는지 확인** — "guide 실패는 FAULT 가 담당"이 맞은 건 *진입* 실패뿐이었고, guide 성공 **후** stale 이 덮은 경우는 아무도 안 맡아 GUIDE 중 과속이 남았다. → `0720_현황.md §22.2`
- ★ **상태 판단은 그 상태를 소유한 쪽이 한다** — Manager 가 `_guide_confirmed` 로 노드 상태를 짐작하면 FAULT 자동복귀 같은 순간에 둘이 어긋난다. 콜백은 하나로 주고 분기는 state 소유자(MissionNode)가. → `0720_현황.md §22.4`
- **저속·모드 보장은 상태와 함께 자동 복귀하지 않는다** — FAULT 재시도로 GUIDE 에 돌아갈 때 controller 값은 그대로다. 복귀 시 다시 확인받을 것. → `0720_현황.md §22.4`
- ★★ **소비 지점 fail-closed 게이트라도 그 술어가 latch(과거)면 뚫린다** — "위험한 건 소비 지점에서 막자"(§23)로 위치는 옳게 옮겼는데, 게이트가 읽는 값이 `guide_confirmed`(과거 1회 성공)라 늦은 sync 가 실제 속도를 덮어도 True 로 남아 과속 goal 이 나갔다. **게이트 술어는 반드시 live(지금 실효값)여야 한다** — `guide_speed_applied` = `_applied == desired guide값`. public live 와 private latch 는 이름을 뚜렷이 분리(`guide_speed_applied` vs `_guide_was_confirmed`)해 다음 검토의 겸직 오독을 막는다. → `0720_현황.md §24.2`
- ★★ **실패 '결정'을 콜백 이벤트 한 번에만 맡기면 상태 전환 중 유실된다** — 저속 복구 소진(`_settle`)이 콜백(`_on_guide_speed_fail`)으로 딱 한 번 알리는데, 그 순간 상태가 GUIDE 가 아니라 SEARCH_BACK 이면 콜백 else(기본값)가 '늦은 통보'로 **무시** → GUIDE 복귀 후 live 게이트가 신규 goal 은 막아도 FAULT 없이 **고장 은폐 영구정지**. §24 와 같은 클래스(fail-open 기본값). 봉합 = 소비 지점(tick)에서 **매 tick live 술어**(`guide_speed_recovery_exhausted`)를 확인하는 fail-closed 가드 — 통보 유실과 무관하게 유도활성(GUIDE/SEARCH_BACK) 상태가 종결 실패를 한 tick 이상 못 이고 간다. 콜백은 즉시성, 가드는 backstop. **가드는 `ensure_sync` 앞**에 둔다(sync 요청 `_inflight` 이 술어를 가림). FAULT→SEARCH_BACK 재시도 복귀도 `request_guide` 재무장(안 하면 소진 술어 잔존→즉시 재-FAULT). → `0720_현황.md §25.2`
- 껍데기 노드 테스트는 **매니저만 찌르지 말고 노드의 진짜 진입점(`on_cmd` 등)을 통과시킬 것** — 호출 누락·순서 뒤바뀜은 매니저 단위테스트가 못 잡는다. → `0720_현황.md §21.2`

## 9. 실물 라이다 (RPLIDAR C1)

- 디버깅 순서: `lsusb`→`ls /dev/ttyUSB*` → dialout 권한 → 소스빌드 → `/scan`. ttyUSB 안 보이면 윗단계 무의미. → 0623_현황
- RViz 런치(`view_*`)는 드라이버를 자기가 다시 켬 → 기존 `pkill -9 sllidar_node` 먼저.
