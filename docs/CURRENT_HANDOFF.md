# CURRENT_HANDOFF.md — 지금 할 한 묶음

> 현재 묶음 + 미해결 보류만 유지한다. 구현자의 완료 주장은 독립검토 판정이 아니다.

- **동결 기준점**: `212885a8292d1e86677d5beeb9f5358d76fe9b40` =
  `platform-core-freeze-260724`.
- **저장소 기준**: `main`의 이 인계 커밋. 부모 `081591b`(1.6.2) 위에 **검토 §76과
  그 보완**을 한 묶음으로 남긴다.
- **실제 보드**: 현장 시험 후보 `rearm-latch-pi-runtime-guard-1.6.2`,
  `build=Aug 17 2026 21:49:19`. 승인 지문은 여전히 1.6.1이라 둘을 섞지 않는다.
- **현재 단계**: D0-FW 1.6.2 현장 사다리 완료 → **검토 §76 독립검토 완료
  (P0 0 · P1 3 · P2 3 · 조건부 승인)** → §76 보완 구현 완료 → **Codex 재검토 대기**.
- 🔴 **§76이 두 가지를 뒤집었다.** ① 커널 로그는 이번에도 사건 구간을 **못 덮었다**
  (시작 기준 34.0분 · 종료 기준 46.5분). ② 7회 약 300ms 정지는 `/odom`만의 것이
  아니다 — BEST_EFFORT `/imu/data`가 같은 7회를 **시각차 2.2~18.9ms 안에서 같이**
  겪었으므로 원인은 odom QoS가 아니라 **상류 링크**다(예약 41-g 신설).

## 이번 한 묶음 목표 — §76 보완 diff의 독립 검토

**역할이 한 번 더 바뀌었다.** 1.6.2(`081591b`)의 구현자는 Codex, 그 독립검토(§76)는
Claude가 했다. 이 커밋의 **보완 구현자는 Claude**이므로 자기 diff를 자기가 승인하지
않는다 — 다음 묶음은 **Codex가 이 diff를 독립 검토**하는 것이다.

### §76 판정 (전문 = `~/Desktop/개발현황/CODEX 현황/0813검토현황.md` §76)

**P0 0 · P1 3 · P2 3 · 조건부 승인.** 승인 범위를 셋으로 갈랐다 —
✅ 커밋 `081591b` 유지 · ✅ 현 보드(1.6.2) 계속 운용 ·
🔴 **승인 지문의 1.6.1 → 1.6.2 이관은 보류**.

| | 발견 | 이번 커밋의 보완 |
|---|---|---|
| P1 §76.2 | dmesg가 soak를 34.0분 못 덮는데 정본이 "전 구간을 덮고 오류 0건"이라 적었다 (§73.2 재발) | 정본 4곳 정정 + **`tools/dmesg_coverage_check.py` 신설**(기계 판정) |
| P1 §76.3 | 7회 300ms 정지 원인 미분류 · 그것을 볼 계기가 구조적으로 없다 | 원인 문장 정정 + **예약 41-g 신설**. 계기(`loop_gap_max_us`)는 다음 굽기 묶음 |
| P1 §76.4 | `LARGE_PUBLISH_TIMEOUT_MS=20`이 미실측 | §1-g에 임시값·재개방 조건 명시 |
| P2 ① | 판정 임계 변경과 그 임계로 통과한 측정이 같은 커밋 | 관행 지적만 기록(600ms 정본은 08-11부터라 소급 아님) |
| P2 ② | `phase/publish_max_us`가 누적 최대라 사건별 값을 못 본다 | 41-g 완료조건에 포함 |
| P2 ③ | LR 회전수 `4/7` 해석 약점이 예약으로 안 올라갔다 | **예약 42 재개방 조건**으로 승격 |

🔴 **지문 이관 조건 2개** — ⓐ §76.2 정정 + 커버리지 기계 검사(이 커밋에서 닫힘)
ⓑ §76.3의 `loop_gap_max_us`로 300ms가 loop 안인지 밖인지 확정(**미닫힘**).
ⓑ 전까지 *"수 초 감시-loop 정지 재발 0"* 은 계측이 아니라 **미검출**이다.

### 왜 1.6.2가 생겼나

1.6.1은 08-17의 6.85초 감시-loop 정지를 막기 위해 주기 telemetry 8개를 전부
BEST_EFFORT로 바꿨다. 그러나 첫 비무장 bench에서 publisher 엔티티는 존재하는데
`/odom`·`/firmware/info`가 7초 동안 0건이었다. 설치
`micro_ros_arduino 2.0.8-humble`의 custom-transport MTU는 512B이고 두 표본은 이를
넘는다. BEST_EFFORT XRCE stream으로 큰 표본을 보낼 수 없다는 새 근거가 §75의 1.6.1
승인 범위를 반증했다.

1.6.2는 큰 두 publisher만 RELIABLE로 복구하고 각각 ACK 대기 상한을 **20ms**로 제한한다.
나머지 6개 telemetry, 400ms runtime guard, re-arm·E-stop·PWM 배선은 그대로다.

## 구현자가 주장하는 완료판정

> 512B보다 큰 `/odom`·`/firmware/info`는 RELIABLE+20ms라 실제 수신되고, 나머지 6개는
> BEST_EFFORT다. 744.697초 지면 soak에서 한 loop의 감시 기능이 수 초 멈추는 결함은
> 재발하지 않았고 runtime overrun·자동 해제도 0이다. 단, RELIABLE odom publish 실패로
> 최대 349ms 수신 공백이 남았으므로 R3 주기 품질까지 해결됐다고 주장하지 않는다.

이 문장은 **구현자 주장**이다. 1.6.2의 독립검토·승인 지문 이관은 아직 없다.

## §3-10 전수 열거와 구현 대조

생산 소스와 소비자를 기계 검색해 다음 클래스를 고정했다.

| 클래스 | 전수 | 1.6.2 대조 |
|---|---:|---|
| 주기 publisher 초기화 | 8 | 큰 2 RELIABLE · 작은 6 BEST_EFFORT |
| RELIABLE session timeout | 2 | odom·firmware info 각각 20ms |
| 실제 `rcl_publish` 경로 | 8 | 전부 `publishMeasured` 경유 |
| runtime phase | 7 | spin·odom·IMU·diagnostics·info·sync·loop |
| runtime publish slot | 8 | info 배열과 enum 순서 일치 |
| executor spin | 1 | 실패한 ARMING은 사유 8로 해제 |
| 자동 해제 사유 | 5 | E-stop·nonfinite·nonzero·runtime·spin |
| 현장 직접 구독 | 12 | sensor-data/BEST_EFFORT 소비 유지 |
| bag override telemetry | 8 | 구독은 BEST_EFFORT라 양쪽 발행 QoS와 호환 |
| runtime info 계약 필드 | 7 | 존재·개수·배열 순서 변이 회귀 |

`tools/test_firmware_runtime_guard.py`는 publisher 증감 양쪽, 대형/소형 QoS 교환,
timeout 대상 교환·20→1000ms, publish 우회, 400ms `>=` 경계, spin 성공/실패 역전,
runtime 필드 소실·배열 교환, 현장 구독·bag topic 증감 변이를 실패시킨다.

## 08-17 현장 사다리 결과

### 펌웨어·통신

| 시행 | 결과 |
|---|---|
| 비무장 bench | `/odom`·전체 `/firmware/info` 수신, version/build 확인 |
| 5초 공중 저속 | 네 바퀴 방향·동작 이상 없음 |
| 비무장 QoS bag `fw162_qos_0817_2220` | 59.4초, odom/IMU 최대 22.52/22.29ms, stamp 엄격 단조 |
| 공중 부하 `fw162_air_0817_2230` | 409.4초, 최대 24.03/23.03ms, runtime 해제 0 |
| E-stop | `/estop/state` 개폐 **10/10** |
| 지면 soak `fw162_ground_soak_0817_2320` | 744.697초, 비영 명령 341.7초, odom 경로 22.256m |

지면 soak 최종 계수:

- `runtime_overruns=0`, `runtime_last=0,0`, `disarm_runtime=0`, `disarm_spin=0`.
- `phase_max_us=4487,41997,4255,20,1304,798,47165`.
- `publish_max_us=41993,23,7,6,5,5,6,1212`.
- `publish_failures=13→83(+70)`.
- E-stop 계수 `raw_edges=39→67`, `rejected=5→19`, rejected max **12ms**,
  `disarm_estop=3→3`. 30ms 미만 HIGH 펌스 14건은 전부 걸러졌지만 짧은 잡음은 남아 있다.
- `/odom`·`/imu/data` 수신 최대 공백 **346.35/349.41ms**, 33.33ms 초과 각 7회.
- `/odom` header stamp는 엄격 단조지만 7회 **210~297ms** 건너뜀.
- 🔴 **커널 층은 이번에도 미관측이다** (검토 §76.2 정정). `dmesg_fw162_ground_soak_0817_2320.log`는
  마지막 줄이 **22:44:41**이고 파일 mtime이 **23:18**인데 soak는 **23:18:43~23:31:08**이라
  **34.0분**(soak 종료 기준 **46.5분**)을 못 미친다. `python3 tools/dmesg_coverage_check.py <bag> <log>` 가 이제 이 조합을 rc=1 로 거부한다. 앞 판의 "전 구간을 덮고 오류 0건"은 취소한다. 게다가 로그가
  덮는 구간에는 Teensy 포트(`1-2.1`·`ttyACM0`)가 **6회 재열거**돼 있고 그중
  `22:15:06→22:17:11`은 **2분 5초 단절**이라 굽기로 설명되지 않는다. USB 재열거는
  기각된 것이 아니라 **후보로 남는다**.
- `/scan`은 라이다 미연결로 0건이므로 독립 비교축은 없음.

7개 공백은 전체 loop 정지가 아니다. 각 공백 직후 odom stamp가 건너뛰고
`publish_failures`가 증가했지만 loop 최대는 47.165ms였다. 결론은 **원래 수 초 안전감시
정지는 현 시험에서 재발하지 않음 · EKF 입력 단기 결측은 남음**이다.

### 외장 판재 후 예약 42 재시험

- 축 높이 LF/LR/RF/RR `43.5/43.5/44.0/43.5mm`, 최저 차고 **19mm**.
- 실제 라이다 스캔 평면 **831mm**, 바닥 기준 TF 높이 **832mm**.
  URDF `lidar_joint z=0.779`는 유지했고 URDF를 수정하지 않았다.
- R1 `panel_r1_fw162_0817`: 경로 122.9mm, 횡 0.8mm(0.67%), yaw 0.60°.
- R2 직진 `panel_r2_line_fw162_0817_retry1`: 줄자 2735mm, odom 2751.1mm,
  `odom/줄자=1.006` 통과. 물리 좌향 273mm(**10.0%**)는 예약 39로 유지.
- C10 평균 57.64mm(08-14의 57.31mm 대비 +0.6%)라 반지름·윤거·FF/게인 무변경.
  🔴 **이 값은 우전륜(FR) 유격이 있는 차량에서 나왔다 — 예약 44.** 스페어 4짝 교체·
  재시행 전까지 확정값으로 인용하지 않는다.
  🔴 LR 회전수 표기 `4/7`은 문맥상 `7 4/7`로 해석했다 — **이 해석 위에 C10 평균이
  서 있고 그 값이 "반지름·윤거 무변경" 판정의 근거**이므로, 검토 §76.5-③에 따라
  **예약 42의 재개방 조건**으로 올린다: 원장부를 다시 읽어 `7 4/7`이 아니면 C10과
  무변경 판정을 같이 다시 세운다.
- R2 회전 retry: IMU yaw 391.60°, gyro 적분 391.54°, odom 389.73°,
  odom/IMU=0.995 통과. 육안 324°/396° 중 IMU는 396°를 지지한다.
- watchdog 총 정지 578.5ms로 현행 운용 수용선 ≤600ms 통과.

예약 42의 하중 재시험은 닫았지만, 19mm 차고는 평탄 바닥 전제이고 물리 좌향 10.0%는
예약 39를 닫지 않는다.

## 근거 문서

- D0-FW 원인·field 판정: `MASTER_PLAN §7` 41-e·41-f.
- runtime 숫자 해독: `REAL_ROBOT_VALUES §1-g`.
- 승인/미승인 경계: `FREEZE_MANIFEST §10.28·§10.28-a`.
- clean compile·보드 이력: `FIRMWARE_REBUILD §4-c-3·§4-c-4`.
- 판재 재시험: `MASTER_PLAN §7` 예약 42.
- R3·cmd_vel 기록: 같은 파일 예약 43.
- 전체 시간순 역사: `~/Desktop/개발현황/0817_현황.md`.

## 완료조건

다음 세션은 아래를 직접 재산출한다.

1. 512B MTU에서 큰 두 표본은 RELIABLE이 필요하고 20ms 상한이 1000ms 연쇄를 되살리지 않는다.
2. publisher 8·timeout 2·publish 8·phase 7·runtime info 7·현장 구독 12·bag 8 폐포가 맞다.
3. 1.6.2 변이 회귀가 QoS 대상 교환·timeout 증가·자리 증감 양쪽에서 실제 실패한다.
4. soak의 7개 공백, stamp 결측, `publish_failures +70`, loop max 47.165ms를 bag에서 재산출한다.
5. runtime/spin 해제 0과 E-stop 10/10·watchdog ≤600ms 역회귀, E-stop 짧은
   HIGH 14건·최대 12ms·`disarm_estop` 증가 0을 확인한다.
6. 1.6.2가 수 초 loop stall을 해결했다는 주장과 R3를 해결하지 못했다는 경계를 유지한다.
7. 재굽기 안전 배선 계약인 `§7-c-E` **13행 전량 수행** 상태를 확인한다.
8. `TODO(D+0)` **11건** 종결 계약을 유지한다.
9. `pytest` **184 passed**, `colcon` **247 tests**, `harness guards` **24 검사**,
   `gate regression` **14 케이스** 기준선을 유지한다.

승인되면 그때만 `FIRMWARE_REBUILD §4`의 `.ino` 허용 지문을 1.6.2로 이관한다.

## 완료 판정

현 상태는 **구현·현장 시험 완료 주장, §76 독립검토 미완료**다. 구현자는 1.6.2를
승인하지 않는다. `§7-c-E` **13행 전량 수행**의 기존 계약과 `TODO(D+0)` **11건**은
축소하지 않는다. 수 초 감시-loop 정지의 재발 0은 현장 근거지만, 349ms 단기 결측 때문에
R3 완료판정은 FAIL이다.

## 완료 후 다음 단계

1. **1.6.2 독립검토**와 승인 지문 이관 여부 결정.
2. 별도 Tier B/Jetson 묶음으로 §72.2 odom guard 3×3(`vx,vy,wz` + covariance) 폐포.
3. H′ odom guard liveness를 구현·검증해 가드 사망을 fail-closed로 만든다.
4. R3를 `/odom /imu/data /scan /cmd_vel` 모두 담아 재시험한다.
5. R3 통과 전 R4/R6로 가지 않는다. 기존 지도는 보존한다.

## 금지 범위

- 구현자가 1.6.2를 자기 승인하거나 미승인 source hash를 precheck 허용값으로 옮기지 않는다.
- 새 펌웨어 업로드·CMD_WHEEL_BASE·FF/게인·지도 재생성·URDF 수정은 하지 않는다.
- §72.2 보완과 H′를 D0-FW 독립검토 diff에 섞지 않는다.
- 08-14 torn odom 원인, 예약 36 IMU I2C, 예약 39 좌향, 예약 40 각속도 전달률,
  예약 43 R3는 열린 상태다.
- `TODO(D+0)` **11건**을 추측으로 다시 열거나 닫지 않는다.

- **⚠ 미해결 보류 — 예약 23 / 검토 §33 (`8fcc1a2`)**: P0 0 · **P1 1** · P2 1.
  `handoff_single_check.sh` 는 `§` 없는 새 작업 꼬리를 붙여도 `OK` 를 내고, 숫자 계약 전수
  증거는 기록 6자리와 독립 실행 21자리가 불일치한다. **승인 완료로 읽지 말 것.** 런타임
  `src/**` 변경·회귀는 없으므로 현재 D+0 실행은 막지 않되, 보완은 다른 묶음에 섞지 않고
  `MASTER_PLAN.md §7` 예약 23 의 완료판정과 부정 회귀로 별도 구현한다. 전문 =
  `~/Desktop/개발현황/CODEX 현황/0801검토현황.md §33`.
- **현재 미해결 보류**: 예약 36·39·40·43·**44**와 §72.2·H′는 위 순서를 유지한다.
- 🔴 **예약 44 전제 (스페어 도착 전까지 매 시행에 적용)** — 우전륜(FR) 유격이 큰 상태다.
  ⓐ `ODOM_WHEEL_RADIUS`·윤거·FF/게인을 08-17 숫자로 바꾸지 않는다 ⓑ 예약 39(좌향)·
  40(각속도 전달률)을 08-17 데이터로 판정하지 않는다 ⓒ C10 **57.64mm**를 확정값으로
  인용하지 않는다 ⓓ 추가 지면 주행은 바퀴 공중 우선·물리 E-stop 담당자 상시·저속.

## 진행표

| 트랙 | 상태 |
|---|---|
| **진행 중 묶음** | §76 1.6.2 독립검토 | ⏸ 구현자와 다른 세션 대기 |
| 1.6.2 현장 사다리 | ✅ 완료 · 구현자 주장 |
| 승인 지문 이관 | 🔴 §76 전 금지 |
| 외장 판재 예약 42 | ✅ 재시험 완료 |
| R3 | 🔴 349ms 결측·scan 무표본으로 보류 |
| `TODO(D+0)` **11건** | ✅ 종결 계약 유지 |

## 최종 회귀

- `bash tools/doc_check.sh` rc=0.
- tools pytest **182/182**(§76.2 커버리지 회귀 8건 신설), mission pytest **184/184**.
- runtime/watchdog 묶음 **31/31**, firmware-info **61/61**(1407/1535).
- firmware precheck 픽스처 **74/74**; 실제 저장소는 승인 지문 미이관으로 rc=1이 정상.
- re-arm host **989/989 + 구조 11/11**.
- harness guards **24/24**, gate regression **14/14**.
- colcon **247/247**, `0 errors · 0 failures · 3 skipped` (`test-result --verbose`).
- 변경 Python flake8(99), `py_compile`, `git diff --check` rc=0.
- 알려진 P0/P1 코퍼스 **146/146**(§76 4자리 이관 완료 — 이 커밋에서 같이 옮겼다).

## 역할 교대

08-17 로봇 실험·1.6.2 구현자 = **Codex**, 그 독립검토(§76) = **Claude**.
이 커밋(§76 보완)의 구현자 = **Claude**이므로 최종 승인은 **Codex**가 쓴다.
같은 세션이 자기 diff를 승인하지 않는다는 불변식은 양쪽 모두 지켰다.
