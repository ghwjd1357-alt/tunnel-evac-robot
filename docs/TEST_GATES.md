# TEST_GATES.md — 테스트·실행 정본 (게이트 + 런치 실행법)

> 모든 E2E 는 **전용 시뮬 PC 전용 (Jetson 실행 금지)**. Gazebo E2E 는 전역 프로세스 cleanup 을
> 하므로 **동시 실행 절대 금지** — 각 스크립트 완전 종료 후 다음 실행.

## 1. platform-core 구조 분리 전체 게이트 (변경 묶음 완료 시 순서대로 전량)

```bash
python3 -m pytest src/mission_manager/test/ -q          # ~0.6s
bash tools/test_harness_guards.sh                        # ~130초 — 하네스 유한 상한 + 외부 증거 파서 fail-closed + 외부 CLI 전수 상한(§14~§16 · 예약 4 · §7~§10 · 예약 17)
bash tools/test_gate_regression.sh                       # ~3분 — readiness_gate 조건 기동 14케이스(Gazebo 불필요)
colcon test --packages-select mission_manager tunnel_sim tunnel_bringup
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

기준선 (**07-30 3차 갱신** — E2E 하네스 신뢰성 묶음 반영): pytest **159 passed** / colcon **188 tests, 0 fail, 3 skip** / **test_harness_guards 20 케이스** / test_gate_regression 14 / E2E 4종 PASS **+ 쌍굴 PASS**.
★ 하네스 증분 = **10 → 13**(07-30: 예약 4 분류 2 + `gz model` 상한 1) → **13 → 16**
  (07-31 Codex §7 P1 보완: cmdvel 판독기 fail-closed 2 + ground truth 정상화 1)
  → **16 → 17** (07-31 Codex §8 P1 보완: cmdvel 판독기 구조 검증 1)
  → **17 유지·케이스 17 계약 교체** (07-31 Codex §9 P1 보완: YAML 자체 파싱 제거 +
  명시 Twist 타입의 고정 6열 CSV + 타입이 맞는 발행자 근거)
  → **17 → 19** (07-31 **예약 17 봉쇄**: 증거 수집 순서(§10.5 P2-①) 1 +
  게이트 스크립트의 **상한 없는 외부 CLI 호출 전수 검사** 1)
  → **19 → 20** (07-31 Codex §11 P1 보완: 침묵 근거의 **관측 창 결합** 1.
  케이스 19 는 개수가 아니라 **검사기 자체를 교체** — 화이트리스트 정규식 → 인용 인식 파서
  `tools/scan_unbounded_cli.py`. §12 P1 보완으로 wrapper 옵션 사각을 fail-closed로 바꾸고
  검사기 부정 회귀를 **양성 13(옵션 wrapper 8종 포함)·음성 6**으로 확대).
  pytest·colcon 은 **무변동** — 이 묶음의 변경 표면이 `tools/` 뿐이라 그래야 맞다.
★ 07-30 증분 = `tunnel_bringup` **+23** (게이트 판정 단위 **20** + lint 3, 그중 copyright 1건 skip).
  1차 보완이 181(단위 13), 2차 보완이 **188**(단위 20 — lifecycle 세대 3 + TF 갱신 4 추가).
pytest 수치는 무변동 — 새 단위테스트는 `src/tunnel_bringup/test/` 에 있어 `colcon test` 로만 돈다
(doc_check 의 pytest 대조 대상은 `src/mission_manager/test/` 하나다).
⚠ **동결 태그 `platform-core-freeze-260724` @ `212885a` 자체의 수치는 pytest 159 / colcon
165·0f·2s 로 불변**이다 (07-24 · 0723검토 §8·§9 가 두 번 독립 재현). 위 181 은 그 뒤 증가분이며,
동결본과 대조할 때는 165 를 쓴다 — 두 수치를 섞지 말 것.
★ 이 수치는 이제 단순 기준선이 아니라 **동결 기준점**이다. 앞으로 회귀가 나오면
`git diff platform-core-freeze-260724 --stat` 이 1차 용의선상 — 증거 전량 = `docs/FREEZE_MANIFEST.md`.
⚠ 이 수치는 묶음 완료 때마다 갱신한다 (테스트가 늘었는데 기준선이 옛 수치면 회귀 검출력이 조용히 떨어진다).
**갱신을 잊어도 `doc_check.sh` 가 실제 개수와 대조해 잡는다** — 기억이 아니라 기계가 지키는 구조.
`make_map.sh` 는 이 게이트에 포함하지 않는다 (지도 자산 변경 — 명시 승인 시에만).

## 2. 각 테스트의 목적과 PASS 기준

| 검증 | 목적 | PASS 기준 |
|---|---|---|
| pytest | 단위·경계조건 (알람/그래프/디바운스/취소 레이스/validator) | 전부 passed |
| test_harness_guards | E2E 하네스 유한 상한 (`read_param_float` 복구 상한·`wait_state` 벽시계 deadline·**SIGTERM 무시도 hard-kill**·daemon kick 남은-예산 배분·mission topic-pub wiring) + **실정지 실패 원인 분류**(케이스 11·12)와 **`gz model` 유한 상한**(케이스 13) — Gazebo 불필요 | **20 케이스** 전부 ✓ (§14~§16 P1 · 07-30 예약 4 · 07-31 §7~§9 P1 · §10 P2 · 예약 17 · §11 P1) |
| ↳ **케이스 11·12 가 요구하는 것** | "분류 코드를 넣었다"와 "그 분류가 두 경우를 **구분한다**"는 다른 명제다. 가짜 `/cmd_vel` 덤프 2종으로 분기를 확인하고, 나아가 `abort_e2e.sh` 안에서 **수집 줄 번호 < `fail()` 줄 번호**까지 단언한다 — 함수가 멀쩡해도 배선이 뒤집히면 증거는 여전히 사라진다 | 두 분류 문자열이 실제로 상이 + 수집이 `fail()` 앞 |
| **test_gate_regression** | 실차 조건 기동 게이트(`readiness_gate`)의 **미통과**가 실제로 지켜지는가 — 토픽·TF·lifecycle·액션을 가짜로 주입해 로봇·Gazebo 없이 검증. 음성이 본체(양성 4 + 음성 10). ★ 핵심은 **"한 번 관측 = 통과" 금지**: lifecycle 이 ACTIVE 를 1회 답한 뒤 멎는 입력(케이스 6)과 토픽이 1건만 오고 죽는 입력(케이스 3). 케이스 13·14 = 서비스 소실→복구 경계 가드 | 14 케이스 전부 ✓ (~163초 실측). 런치 양성 체인 생략 = `GATE_SKIP_LAUNCH=1` (12케이스). ⚠ Gazebo 실행 중이면 자체 거부(정리 단계가 nav2 를 전역 kill) |
| ↳ **검출력 경계 (정직하게)** | 이 셸 층은 **2차 P1(소실 경계 in-flight 조회 폐기)을 검출하지 못한다** — 실측: 보완을 되돌려도 12/12 PASS. 소실 순간의 늦은 응답은 서비스 파괴와 함께 DDS 에서 소멸해 실물 층에서 만들 수 없다. 검출은 `src/tunnel_bringup/test/` 단위층 3케이스가 한다 | 층 분담을 흐리지 말 것 — 셸이 PASS 라고 세대 결함이 없는 것이 아니다 |
| colcon test | 워크스페이스 lint+단위 | test-result 0 errors/failures |
| regression_negative | **안 돼야 하는 게 안 되는가** — 지도밖/벽너머/막힌 goal 실패 종결 + 정상 goal 양성 대조군 | 불가 3종 ABORTED + 정상 SUCCEEDED (막힌 goal 은 BT 재시도 소진까지 ~2분 정상) |
| regression_3goals | 주행 정확도 회귀 | 3종 SUCCEEDED, 최종 오차 **≤0.3m** |
| mission_e2e | 미션 전체 흐름 | GUIDE 0.12 실측 → SEARCH_BACK → 재발견 → ESCAPED |
| abort_e2e | "취소 호출"≠"실제 정지" 검증 | FAULT + 5초 이동 ≤0.10m + nonzero cmd_vel 0 (angular.z 포함) + 취소 접수 로그 |
| ↳ **실패 시 원인 자동 분류** (07-30 예약 4) | 실정지 단언이 깨지면 `fail()` **전에** `/cmd_vel` 을 수집해 **코드 결함(취소 경로) vs 잔류 명령/시뮬 특성**을 갈라 FAIL 메시지에 담는다. 구판은 `fail()` 이 즉시 cleanup+exit 해 증거가 사라졌다 | 잔류 있음/없음/판독실패 3갈래가 서로 다른 문구 — '판독 실패'를 '잠잠(0건)'으로 뭉개지 않는다 |
| ↳ **판독기 fail-closed 계약** (07-31 §7.2 P1) | 07-30 판은 정규식에 걸린 값이 **하나도 없어도 `0건`** 을 찍어 빈·경고문뿐·필드누락·NaN/Inf 덤프가 전부 '잔류 없음'으로 둔갑했고 ⑧ 에서 **그대로 PASS** 였다. 이제 **완전한 Twist 표본(6성분 전부 유한)** 을 최소 1개 확인해야 정수를 반환한다. ⑦·⑧ 은 **단일 계약** `measure_cmdvel_residual` 만 소비한다 | 손상 입력은 전부 '판독 실패'. 시간상자 출력은 한 메시지=CSV 한 줄·unbuffered로 바꿔, 불완전 줄도 정상 꼬리로 추정하지 않는다 |
| ↳ **고정 CSV 계약** (07-31 §8.2·§9.2 P1) | §8 보완은 줄 개수 대신 키 집합을 봤지만, 미지 `metadata:` 부모 아래 `y/z`를 직전 `angular`의 키로 오귀속했다. YAML 부모·들여쓰기·중복·꼬리를 계속 자체 구현하는 것이 근인이므로 YAML 파서를 폐기했다. 수집 시 타입을 `geometry_msgs/msg/Twist`로 명시하고 Humble `--csv`로 평탄화한 **고정 6열**만 판독한다 | 모든 비어 있지 않은 줄이 정확히 6개 유한 실수여야 함. YAML·경고문·열 부족/초과·중간 빈 줄·임의 꼬리 = 판독 실패. 실제 Humble 출력 `0.12,0,0,0,0,…`와 abort E2E 보존 |
| ↳ ★ **'관측된 침묵'** (07-31 실측) | 이 시스템에서 abort 뒤의 '잠잠'은 zero Twist 가 아니라 **완전한 침묵**이다 — nav2 가 취소 후 발행 자체를 멈춰 **실덤프가 0바이트**다. "빈 덤프=무조건 판독 실패"로 두면 `abort_e2e` 가 **영구 거짓 FAIL** 이 된다. 그래서 침묵은 **관측 근거**(수집이 시간상자 정상 소진 + `/cmd_vel` 타입이 **Twist** + 발행자 생존)가 있을 때만 0건으로 인정한다 | 빈 덤프+근거 = **0건(PASS)** / 타입 불일치·발행자 없음·조회실패 = **판독 실패** / 내용은 왔는데 완전한 6열 표본 0개 = **판독 실패**. ⚠ `ros2 topic info` 는 daemon 의존 → 복구 1회 후에도 못 읽으면 fail-closed |
| ↳ ★ **침묵 근거의 '관측 창' 결합** (07-31 §11.3 P1-②) | 근거를 **발행자 수**로 세면 창을 못 묶는다. 수집 창에는 발행자가 없고 **창 직후에만** 생긴 경우(Nav2 재기동·세대 전환·DDS discovery 지연)에도 그 수를 근거로 빈 덤프가 `0건` PASS 였다 — 들을 대상이 아예 없었는데 '정상 침묵'이 되는 거짓 PASS(검토자·구현자 각각 재현). 선-조회(창 도중 죽은 발행자를 승인)와 후-조회(창 뒤에 생긴 발행자를 승인)는 **방향만 다른 같은 크기의 구멍**이다 | 근거를 수 → **GID**(엔드포인트마다 유일한 DDS GUID)로 바꾸고 **창 양끝을 브래킷**한다. ① 창-시작 조회는 수집과 **동시에** 띄우고 상한을 창 길이로 묶는다(실측 조회 0.17~0.18s vs 창 2s = **11배 여유**) ② 덤프에 내용이 있으면 조회를 **기다리지 않는다**(§10.5 P2-① 유지) ③ 창 시작 GID 가 **전부** 창 끝에도 살아 있을 때만 침묵 승인 — 창 뒤에 새로 생긴 발행자는 근거로 재사용하지 않는다. 실 그래프 실증: A→B 세대 전환 = 판독 실패 / 침묵 발행자 생존 = 0건 |
| ↳ ⚠ **검증 상한 — 브래킷은 '연속 생존'을 증명하지 않는다** (07-31) | GID 가 창 양끝에 있으면 그 사이 **죽었다 다시 태어난 것은 아니다**(재생성은 새 GID 를 받는다). 그러나 ⓐ 그 발행자가 창 내내 살아 있었는지, ⓑ 발행할 **의도**가 있었는지는 이 층에서 증명하지 않는다 | **닫지 않는다** — 위의 메시지 손실 상한과 같은 자리에 둔다. 실차 R0 의 cmd_vel watchdog 실측이 물리적으로 덮을 몫(`FREEZE_MANIFEST.md §6`) |
| ↳ ⚠ **검증 상한 — 메시지 손실은 침묵과 구별되지 않는다** (07-31 §10.6 P2-②) | 수집 명령이 `--no-lost-messages`(손실 보고 억제)를 쓴다. 켜지 않으면 손실 보고문이 CSV 를 오염시켜 fail-closed FAIL 이 되므로 켜는 선택 자체는 합리적이지만, **그 대가로 '표본이 불완전했다'는 유일한 신호가 사라진다.** 잔류 명령이 실제로 있었는데 2초 창의 표본이 전부 손실되면 빈 덤프 + Twist 발행자 근거로 **'관측된 침묵' PASS** 가 될 수 있다 | **이 층에서는 닫지 않는다** — 실차 R0 의 cmd_vel watchdog 실측이 같은 위험을 물리적으로 덮는다(`FREEZE_MANIFEST.md §6`). 그때 이 상한을 재평가한다 |
| ↳ **상한 없는 외부 CLI 전수 봉쇄** (07-31 예약 17) | `gz model` 무상한 호출이 `mission_e2e ⑪` 에서 **11분 행**을 만들었고 사람이 죽여서야 `== PASS` 가 났다 — **개입해서 얻은 PASS 는 판정이 아니다.** 게이트 5파일(`lib_e2e`·`abort_e2e`·`mission_e2e`·`regression_negative`·`regression_3goals`)의 모든 foreground `ros2`·`gz` 호출에 `hard_timeout` 을 씌웠다(**10곳 + §11.2 의 1곳**). 판정에 쓰이는 좌표는 '못 읽음=인프라'로 따로 분류한다 | 케이스 19 가 **전수 기계 검사**한다. ⚠ 제외 대상은 `MASTER_PLAN §7` 예약 17 에 **줄 번호까지 등록** — 검사기 출력이 그 목록의 정본이다 |
| ↳ ⚠ **'전수'라는 말이 두 번 전수가 아니었다** (07-31 §11.2·§12 P1) | ① 구판 검사기는 **하위 명령 화이트리스트**라 `ros2 action send_goal`을 놓친 채 19/19 PASS했다. ② 전체 `ros2/gz`로 뒤집은 검사기도 wrapper 이름 하나만 건너뛴 뒤 옵션을 "다른 명령"으로 오인해 `xargs -n1`·`time -p`·`command --`·`env -i` 뒤의 무상한 호출 4종을 전부 출력 0건으로 승인했다 | 인용·주석·구분자를 가르는 `tools/scan_unbounded_cli.py`가 명령 단위로 `hard_timeout`을 결합한다. wrapper를 본 단위는 옵션 문법을 불완전하게 추정하지 않고 끝까지 보수적으로 훑어 unquoted `ros2/gz`를 fail-closed로 잡는다. 케이스 19가 **양성 13(옵션 wrapper 8종)·음성 6**으로 검사기 자체를 검증하며, guarded `hard_timeout N env … ros2 …`는 보존한다 |
| ↳ **ground truth 조회 상한** (07-30) | `gz model -m … -p` 는 무방비면 **무한 행**한다(실측: `mission_e2e ⑪` 11분, 고아 21분). `gz_model_xy` 가 hard 상한 10s 를 씌우고, 못 읽으면 **'인프라 실패'로 따로 분류**해 '실정지 실패'와 섞지 않는다 | 상한 내 종결 + 정상 좌표 조회는 보존(역회귀) |
| ↳ **ground truth 정상화** (07-31 §7.3 P1) | 07-30 판은 마지막 줄의 첫 두 토큰을 **검증 없이** 흘려보내 `model -m`·`nan nan`·`inf inf` 가 "빈 문자열이 아니다"라는 이유로 인프라 분기를 **우회**했고, ⑦ 에서 실정지 실패로 오분류돼 원인 분류까지 틀린 전제 위에서 돌았다. 이제 **두 개의 유한 실수**만 통과한다 | timeout·빈값·필드부족·비숫자·NaN/Inf 가 전부 같은 '좌표 없음'. ⚠ **음수·소수는 정상 world 좌표**(스폰 -12,0) — 거부하면 역회귀 |

★ **mission_e2e SEARCH_BACK 예산 = 180s** (`T_SEARCHBACK`, T자·쌍굴 공통 — 07-24 e2e-harness-fix
재산정, **옛 90s 대체**). 근거는 아래 둘을 함께 남긴다(둘 중 하나만으론 '숫자만 올린 기준 완화'다):
- **관측 도달 분포** (★ 07-29 갱신 — **측정 방식이 다른 수치를 섞어 쓰지 않는다**):
  - **현행 기준 = 벽시계 실측.** 최종 회차(`FREEZE_MANIFEST.md §8.3`): T자 GATHER **15s** ·
    SEARCH_BACK **14s** · ESCAPED **22s** / 쌍굴 GATHER **76s** · SEARCH_BACK **14s** · ESCAPED **164s**.
    직전 재검증 구간(`FREEZE_MANIFEST.md §8.1`): SEARCH_BACK **13~15s** · GATHER **71~143s** ·
    ESCAPED **28~167s** — 전부 예산 내.
  - ⚠ **옛 수치 재사용 금지**: 예산 재산정 당시 적었던 "표준환경 신규 7회 전부 **9s** 수렴 /
    GATHER T자 9s·쌍굴 45~48s"는 **sleep 누적 측정값**이다 (`FREEZE_MANIFEST.md §8.1` 원문:
    "옛 sleep 누적보다 정직하게 큼"). 벽시계보다 작게 나오므로 **이 9s 계열 숫자를 근거로
    예산을 다시 조이지 말 것.**
  - **역사 표본**(옛 하네스 · daemon flake · Nav2 플래닝 지연 환경): SEARCH_BACK **9s ~ ≈90s**,
    GATHER 48/126/48 (`FREEZE_MANIFEST.md §8`). 변동성은 지형이 아니라 인프라 상태에 좌우돼
    강제 재현이 어렵고, 이 역사 outlier 가 최악을 실증한다 — **180s 의 2배 마진이 여기에 걸려 있다.**
- **상한이 보호하는 것**: "미션은 건강한데 팔로워 간격 벌어짐(≥(2.5−1.2)/0.12 ≈ 11s) + `lost_sec` 3s
  디바운스 + 재플래닝·goal 재전송 지연 + detection flicker 로 놓침 확정이 늦어지는 최악". 관측 최악
  ≈90s(옛 예산 경계에서 스쳐 실패)의 **2배 마진**. 이 상한을 넘으면 '건강한 지연'이 아니라 놓침이
  구조적으로 확정 안 되는 이상(follower `stop` 미수신·`lost` 미발화)이므로 **여전히 FAIL** 해야 한다.
  실측 부정 회귀: 도달 불가 예산에서 `FAIL: … 대기 타임아웃(예산 Ns, 경과 Ms), 마지막 상태='GUIDE'` —
  판정과 '마지막 상태' 보고가 같은 읽기라 자기모순이 없다(옛 폴링 race 제거).
- ★ **상한 집행은 벽시계(`SECONDS`)로, hard-kill 로 실제 보장한다** (07-24 §14·§15 P1 보완):
  `wait_state` 예산과 `read_param_float` 복구 시퀀스를 sleep 누적이 아니라 실경과시간으로 지킨다.
  예산 밖에서 늦게 도달하면 s 가 목표여도 `경과>예산`이 메시지에 찍혀 FAIL(모순 없음).
  - **§15 P1 추가**: `lib_e2e.sh`가 소유한 ros2 CLI 대기를 공통 `hard_timeout`
    (= `timeout --kill-after=2`)으로 단일화 —
    GNU `timeout` 은 기본 SIGTERM 만 보내 CLI 가 TERM 을 무시하면 안 죽는다. `--kill-after` 로 TERM 뒤
    2초 유예 후 SIGKILL 을 보장한다. `read_param_float` 실제 hard 상한 = (8+2)+(5+2)+(5+2)+(8+2) = **34s**
    (정상 TERM 응답 시 ≈26s). `wait_state` 의 daemon kick 도 고정 5 가 아니라 **남은 예산 배분**(각 5s
    상한 + 유예까지 rem 안에 수렴)이며, 남은 예산 < 6s 면 복구를 생략하고 deadline FAIL 한다 → daemon
    복구까지 전부 예산 안에서 소모돼 벽시계 상한이 N 을 넘지 않는다.
  - **§16 P1 추가**: `mission_e2e.sh`의 alarm·stop·follow `topic pub` 3곳도 공통
    `hard_timeout 12`로 통일한다. TERM 무시 fake CLI가 상위 cutoff 없이 각 12s+유예 안에
    종결하고, 정상 fake CLI는 즉시 반환하며, 실제 wiring이 hard-timeout 3/3인지 함께 검사한다.
  - 이 부정·역회귀는 `tools/test_harness_guards.sh` **20케이스**가 Gazebo 없이 격리 검증한다
    (case 6=34s·case 7=13s·case 8=30s·case 9=TERM 무시 topic 3종·case 10=정상 topic 3종·
    **case 11·12=실정지 실패 분류 + 수집/`fail()` 배선 순서·case 13=`gz model` hard 상한 10s·
    case 14·15=cmdvel 판독기 fail-closed 계약 + '관측된 침묵' 역회귀·case 16=ground truth 정상화·
    **case 18=증거 수집이 발행자 조회보다 먼저(§10.5 P2-①)·case 19=게이트 5파일의 상한 없는
    외부 CLI 호출 전수 검사 + 검사기 자체의 양성13/음성6 부정 회귀(예약 17·§11.2·§12 P1)·
    case 20=침묵 근거의 관측 창 결합 — 창 밖 발행자·세대 전환·소실 fail-closed(§11.3 P1-②)**·
    case 17=cmdvel **고정 CSV 6열 + Twist 타입 근거** 계약).
    ⚠ 배선 단언은 반드시 `-F`(고정 문자열) — 07-31 실측: `grep -E` 의 `.*` 는 한글·em대시가 섞인
    줄을 **못 넘어 조용히 0** 을 낸다(`-F` 는 1, LC_ALL 무관). 긍정 단언에 쓰면 검사가 조용히 통과한다.
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
colcon test --packages-select mission_manager tunnel_sim tunnel_bringup   # ⚠ 반드시 직접 실행
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
| **`tunnel_bringup/**`** (게이트 판정·런치 배선·실차 파라미터) | `test_gate_regression` (Gazebo 불필요 — 실차 코드라 시뮬 E2E 로는 검출이 안 된다). 실차 자산 자체는 노트북에서 실행 불가이므로 **여기까지가 검증 상한**임을 검토본에 명시할 것 |
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
   당시 하네스 결함 2건은 후속 **e2e-harness-fix**에서 수리 완료됐다
   (`FREEZE_MANIFEST.md §8.1`~`§8.3`): 현재 SEARCH_BACK 상한은 180s(벽시계), 유한 대기는
   hard-kill로 보장하며 격리 회귀 **20케이스**(07-31 갱신)를 유지한다. 동결 태그의 옛 90s 기준 기록은 불변이다.
2. **동결 커밋에는 수정을 섞지 않는다** — 게이트 중 결함이 나오면 예약으로 분리한다.
   섞는 순간 "무엇을 얼렸는가"가 흐려진다.
