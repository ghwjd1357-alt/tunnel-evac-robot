#!/usr/bin/env bash
# ============================================================================
# d0_check.sh — 인수 당일(D+0) "구동부 연결이 진짜 됐는가" 판정 (2026-08-02 신설, S6-4)
#
# 무엇을 판정하는가:
#   Teensy(micro-ROS) → Jetson 사이가 **EKF 를 돌릴 수 있을 만큼** 연결됐는지.
#   통과하면 그날의 인계가 끝나고, 다음 날 D+1 첫 스텝(R3 rosbag)으로 넘어간다
#   (docs/D1_FIRST_STEP.md). 실패하면 **그 자리에서 구동부와 함께** 원인을 본다 —
#   구동부가 돌아간 뒤에 발견하면 며칠이 든다.
#
# 언제 돌리는가:
#   ① micro-ROS agent 가 떠 있고  ② 로봇 전원이 켜져 있을 때.
#   agent 기동 = docs/JETSON_SETUP.md §5 (micro_ros_agent 확보 2안 + 기동 명령).
#
# ★★ 실행 전제 (08-02 검토 §29.3 로 신설) — **EKF 를 먼저 띄워 놓아야 한다.**
#   이 스크립트의 QoS 검사는 "구독자 쪽 QoS 가 발행자와 맞물리는가"를 본다. 그런데 구판은
#   agent 만 띄운 상태에서 돌려 **구독자가 0개**였고, 그 0개를 "전부 매칭됨"으로 통과시켰다
#   (검토자 재현: `npub=1 nsub=0 => target_check_passes=yes`). 아무도 안 보는 것을 보고
#   "봤다"고 말한 셈이다. 이제 EKF 엔드포인트가 실제로 있어야 통과한다.
#     별도 터미널:  ros2 run robot_localization ekf_node --ros-args \
#                     --params-file ~/ros2_ws/src/tunnel_bringup/config/ekf_real.yaml
#   ⚠ 노드 이름을 바꾸지 말 것 — yaml 최상단 키가 `ekf_filter_node:` 라 이름이 다르면
#     파라미터가 **에러 없이** 하나도 안 붙는다.
#
# 사용:
#   bash tools/d0_check.sh              # 전량 검사 (검사 6·8 은 사람이 손을 써야 한다)
#   bash tools/d0_check.sh --no-sign    # 검사 6(바퀴 부호)만 생략
#   bash tools/d0_check.sh --no-estop   # 검사 8(E-stop)만 생략
#   bash tools/d0_check.sh --no-manual  # 사람이 필요한 6·8 을 함께 생략
#   bash tools/d0_check.sh --secs 15    # **관측 창** 길이(기본 8초, 허용 3~120)
#   bash tools/d0_check.sh --expect-build 'Aug 14 2026 09:12:33'
#       🔴 굽기 직후 컴파일 기록의 build 문자열. 주면 stale build 를 기계로 잡는다.
#       안 주면 검사 7 의 build 행은 **미판정**이다 (검토 §69.2).
#   ⚠ 무엇을 생략하든 '전량 통과'가 아니다 — 종료 2.
#   ⚠ `--secs` 는 **관측 창만** 바꾼다. 스냅샷(`echo --once`·`topic info`) 상한은 별도
#     상수(SNAP_ECHO_SECS·SNAP_INFO_SECS·FW_INFO_SECS)이고 이유는 그 정의부에 있다.
#     실제 시간 예산은 머리말이 그 변수들로 **계산해서** 찍는다 (08-05 검토 §39.3 P2-1).
#
# 검사 목록 (8개):
#   [1] 시리얼 장치   [2] /odom 주기   [3] /imu/data 주기
#   [4] /odom QoS     [5] /imu/data QoS                      — 여기까지 자동
#   [6] 전진 부호(바퀴를 굴린다)  [7] 펌웨어 정체  [8] E-stop 배선(버튼을 누른다)
#   ★ [7][8] 은 **펌웨어 소스를 받은 08-02 에 신설**됐다. 소스가 없으면 쓸 수 없는 검사다.
#   ⚠ 번호는 손으로 적지 않는다 — `next_idx()` 가 실행 순서대로 매긴다. 손으로 적었더니
#     신설 2건이 [6] 을 중복해서 쓰고 문서는 "7검사"라고 적는 드리프트가 실제로 났다.
#
# 종료 코드:  0 = 전량 통과 · 1 = 실패 · 2 = 불완전(건너뛴 검사 있음) · 3 = 사용법 오류
#   ★ 2 를 0 과 구분하는 이유: "검사를 안 한 것"과 "검사해서 통과한 것"은 다르다.
#     이 구분이 없으면 나중에 "D+0 에 통과했었다"는 기록이 실제로는 아무 의미가 없어진다.
#
# ⚠⚠ 이 스크립트는 **작성 시점에 실행 검증이 불가능했다.** 2026-08-02 현재 노트북에는
#    Jetson 도 Teensy 도 없다. 그래서 아래 원칙으로 썼다:
#      · 추측한 명령을 쓰지 않는다 — 여기 쓰인 `ros2` 명령의 **출력 형식은 전부 노트북에서
#        가짜 BEST_EFFORT 퍼블리셔로 실측**했다(아래 [실측으로 확인한 전제]).
#      · 확인 못 한 것은 `TODO(D+0): 확인` 으로 남기고 확인 방법을 같이 적는다.
#      · 판독에 실패하면 **통과시키지 않는다**(fail-closed). "조용한 통과"는 이 저장소가
#        07-31 에 실제로 당한 사고다 (MASTER_PLAN.md §7 예약 4).
#
# [실측으로 확인한 전제 — 2026-08-02, 노트북 x86_64 / ROS 2 Humble]
#   ⓐ `ros2 topic hz` 는 **기본 인자 그대로 BEST_EFFORT 퍼블리셔를 관측한다.**
#      (가짜 46.5Hz BEST_EFFORT 퍼블리셔로 확인 → `average rate: 46.500`)
#      → D+0 에 "hz 가 아무것도 안 나온다"면 그건 QoS 탓이 아니라 **정말 안 오는 것**이다.
#   ⓑ `ros2 topic hz` 에는 `--qos-reliability` 옵션이 **없다**(unrecognized arguments).
#      혹시 어디선가 그 플래그를 본다면 그건 `topic echo` 쪽이다. 여기서 쓰지 않는다.
#   ⓒ `ros2 topic echo --field twist.twist.linear.x --once` 도 기본 인자로 관측된다.
#   ⓓ `ros2 topic info -v` 는 퍼블리셔·구독자 **각각의 Reliability** 를 찍어 준다.
#
# [08-05 검토 §39.2 보완 — `hard_timeout` 8자리 전수 대조 2회차]
#   08-03(§30.3)에 이 표를 처음 썼을 때는 **종료 상태를 묻는가**만 물었다. 08-04 검토 §39.2 가
#   같은 8자리에서 **다른 축**을 찾았다: 그 자리가 *"몇 초 기다렸다"* 를 말할 자격이 있는가.
#   두 자리(구 `300`·`552`)는 벽시계를 한 번도 안 재고 `"8초 안에"`·`"12초 대기했다"` 를
#   **사실로 단정**했다 — 검토자 실측 실패 지연은 **0.408초**였다.
#
#   ★ 이번에는 검토가 준 목록(2자리)을 그대로 따라가지 않고 `grep` 으로 전수를 다시 셌다
#     (`AGENTS.md §3-10 ★★④` — 남이 준 열거가 더 위험하다). 그랬더니 **원인 문자열을
#     못 보여주는 자리가 4개**였다(검토 지목 2 + E-stop 2). 재확인 자리는 stderr 를
#     `2>/dev/null` 로 아예 버리고 있어 캡처 대상조차 아니었다.
#
#     자리(현재 줄)      | 종류    | 정상 rc | 상한 출처        | 실측 경과 | 원인 문자열
#     -------------------|---------|---------|------------------|-----------|-------------
#     topic hz           | 관측 창 | 124/137 | $HZ_SECS(--secs) | ✅ 표시    | ✅ 앞 3줄
#     echo --once(창 끝) | 스냅샷  | 0       | $SNAP_ECHO_SECS  | ✅ 표시    | ✅ (구: 폐기)
#     topic info -v      | 스냅샷  | 0       | $SNAP_INFO_SECS  | ✅ 표시    | ✅ 앞 3줄
#     echo --field(부호) | 관측 창 | 124/137 | $HZ_SECS(--secs) | ✅ 표시    | ✅ 앞 3줄
#     echo /firmware     | 스냅샷  | 0       | $FW_INFO_SECS    | ✅ 표시    | ✅ (구: 캡처만)
#     echo /estop(평상시)| 스냅샷  | 0       | $SNAP_ECHO_SECS  | ✅ 표시    | ✅ (구: 캡처만)
#     echo /estop(누름)  | 스냅샷  | 0       | $SNAP_ECHO_SECS  | ✅ 표시    | ✅ (구: 캡처만)
#     topic info(재확인) | 스냅샷  | 0       | $SNAP_INFO_SECS  | ✅ 표시    | ✅ (구: stderr 폐기)
#
#   ★ 표와 구현이 갈라질 자리를 없앴다 (`§3-10 ★②`): 8자리 전부 **같은 두 함수**
#     (`clock_begin`/`clock_end`)로 재고, 실패 문장도 `why_snapshot()` 한 곳에서만 쓴다.
#     자리마다 손으로 초를 적던 것이 정확히 이 결함의 근인이었다.
#
# [08-07 검토 §39-R.1 보완 — 같은 8자리에 **세 번째 축**을 준다: rc 를 무엇의 근거로 쓰는가]
#   08-05 판은 '재는가'·'원인을 남기는가'까지 닫고도 **'무엇으로 판정하는가'** 를 안 물었다.
#   `why_snapshot()` 은 실측 경과를 **찍기만** 하고 분류에는 안 썼다 — rc 가 124/137 이면
#   무조건 *"상한이 실제로 발동했다"*. 자식이 즉시 `exit 124` 하는 입력에서 스냅샷 6자리가
#   **3~4ms** 만에 그 문장을 냈다. 관측 창 3자리는 §30.3 의 벽시계 덕분에 같은 입력을 거부했다.
#
#     자리(현재 줄)      | 판정 근거(구판) | 08-07 이후
#     -------------------|-----------------|------------------------------------------
#     topic hz ⓪ rc      | rc              | 유지 (다음 줄이 벽시계를 본다)
#     topic hz ⓪ 벽시계  | 경과            | `clock_reached_limit()` 로 규칙 일원화
#     echo --field 부호  | rc **만**       | `limit_actually_fired()` — 짧은 124 도 경고
#     부호 STILL/READ    | rc + 경과       | 같은 함수로 통일 (부등식 손으로 안 적는다)
#     why_snapshot 6자리 | rc **만**       | rc + 경과. 짧은 124 는 '상한 상태값 + 시각 이상'
#
#   ⚠ 이웃 클래스도 세어 봤다(숨기고 '전수'라고 쓰지 않는다): `tools/lib_e2e.sh:267` 은
#     124/137 을 **시간상자의 정상 종료**로 받아들일 뿐 시간을 주장하지 않고, 판정은 별도
#     판독기가 한다. `tools/test_harness_guards.sh:185` 는 이미 rc 와 경과를 **함께** 본다.
#     그래서 이 결함은 `d0_check.sh` 안 2자리로 좁혀졌다.
#   마지막 자리도 원 명령 rc를 먼저 확인한다. rc 0이 아닌 잘린 스냅샷은 EKF 행이 보여도
#   유지 판정에 사용하지 않는다.
#
# ⚠ 이 스크립트는 tools/lib_e2e.sh 를 **일부러 source 하지 않는다.**
#   그쪽 `cleanup()` 은 nav2·slam·gzserver 를 **이름으로 전역 kill** 한다 — 전용 시뮬 PC
#   전용이고, 실차 Jetson 에서 돌면 살아 있는 스택을 통째로 죽인다. 여기서 필요한 것은
#   `hard_timeout` 하나뿐이라 아래에 4줄로 다시 정의한다 (중복이지만 안전 경계가 우선).
# ============================================================================
set -u

# 🔴 08-13 (검토 §65.3) — 펌웨어 정체 검사가 기대값을 `.ino` 에서 읽으려면 저장소
#   뿌리를 알아야 한다. 실행 위치와 무관하게 스크립트 자기 위치에서 잡는다.
D0_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
D0_EXPECT_BUILD="${D0_EXPECT_BUILD:-}"

ODOM_TOPIC="/odom"
IMU_TOPIC="/imu/data"
TEENSY_DEV="/dev/teensy_drive"     # udev 규칙 = tools/udev/99-teensy-drive.rules (S6-3)

HZ_SECS=8                          # 주기 측정 시간(초)
HZ_MIN=30.0                        # ★ 통과 하한. 근거 = EKF 발행 주기 30Hz
                                   #   (config/ekf_real.yaml `frequency`). 이보다 느리면
                                   #   EKF 한 주기가 입력 없이 지나간다.
HZ_EXPECT_ODOM=46.5                # 구동부 3차 회신 §5 실측 (REAL_ROBOT_VALUES.md §1)
HZ_EXPECT_IMU=46.4
GAP_MAX_MS=33.33                   # 최대 간격 상한(ms) = EKF 한 주기(1/30s)
SIGN_MIN=0.005                     # 바퀴를 굴렸을 때 '움직였다'고 볼 최소 속도(m/s)

D0_KILL_GRACE=2                    # SIGTERM 뒤 SIGKILL 까지 유예(초)

# --- 스냅샷 상한 (★ 08-05 검토 §39.3 P2-1 — `--secs` 와 **다른 축**이다) -------------
# 구판은 이 세 값을 호출 자리에 숫자로 박아 두고, 주석에는 *"주 측정창과 같은 8초를 주되"*
# 라고 적었다. 그런데 `--secs` 는 3~120 을 받으므로 `--secs 3` 이면 창 3초 · 스냅샷 8초로
# **갈라진다.** 주석이 거짓이 된 것이지 값이 틀린 게 아니다 → 이름을 붙여 한 곳에서 정하고,
# 머리말은 이 변수들로 예산을 **계산해서** 찍는다(숫자를 손으로 다시 적지 않는다).
#
# ⚠ `--secs` 에 묶지 **않는** 이유(권고를 받았으나 채택하지 않았고, 그 근거를 남긴다):
#   08-03 D+0 실측에서 `--once` 성공까지 **1.996~2.315초**가 걸렸다(DDS 발견 지연).
#   `--secs 3` 을 그대로 물리면 정상 발행을 거짓 FAIL 내는 회귀가 된다 — 그게 정확히
#   구판 3초가 현장에서 낸 오경보다. 관측 창은 '표본을 얼마나 모으나'이고 스냅샷 상한은
#   'DDS 가 답하는 데 얼마나 걸리나'라, 사용자가 조절할 축이 아니다.
SNAP_ECHO_SECS=8                   # `topic echo --once` — 실측 2.0~2.3초의 3.5~4배 여유
SNAP_INFO_SECS=20                  # `topic info -v` — daemon 응답 대기
FW_INFO_SECS=12                    # /firmware/info 는 5초 주기 VOLATILE → 2주기 + 여유

# --- 외부 CLI 상한 불변조건 (FREEZE_MANIFEST.md §10 · 07-30 신설) --------------
# `ros2` CLI 는 daemon flake 로 **무한 행**할 수 있다(실측 13분 27초 매달린 적이 있다).
# 그냥 `timeout N` 은 SIGTERM 만 보내서, TERM 을 무시하는 상대에겐 상한이 아니다.
# --kill-after 로 SIGKILL 까지 보장해야 '진짜' 벽시계 상한이 된다.
hard_timeout() {  # $1=상한(초) $2..=실행할 명령
  local dur=$1; shift
  timeout --kill-after="$D0_KILL_GRACE" "$dur" "$@"
}

# --- ★ 08-03 검토 §30.3 — 상한을 씌우는 것과 **결과를 묻는 것**은 다른 일이다 ---------
# 구판은 `hard_timeout` 으로 상한만 씌우고 **종료 상태를 한 번도 보지 않았다.** 그래서
# `ros2 topic hz` 가 4초 만에 rc 42 로 죽어도, 죽기 전에 찍어 둔 요약 한 덩어리가 남아
# "8초 창을 관측했다"로 승격됐다(검토자 실측: `simulated_topic_hz_rc=42 / check_fail=0`).
# 8초를 한 번도 기다리지 않고 8초 판정을 낸 것이다.
#
# ⚠ 여기서 rc 의 **정상값이 두 종류**라는 점이 함정이다. 용도에 따라 정반대다:
#   ① 관측 창(`topic hz`·`topic echo` 를 N초 동안 돌리는 것) — 대상은 스스로 끝나지 않는다.
#      → **우리가 건 상한이 발동해서 끝나는 것이 정상**이다. rc 124(SIGTERM) 또는
#        137(=128+9, TERM 을 씹어 SIGKILL 까지 간 경우). **rc 0 은 비정상**이다
#        ("일찍 끝났다" = 창을 안 채웠다).
#   ② 스냅샷(`topic info -v`·`echo --once`) — 대상이 스스로 끝난다.
#      → **rc 0 이 정상**이고 124/137 은 매달렸다는 뜻이다.
# GNU timeout 규약: 상한 발동 시 124, SIGKILL 까지 갔으면 128+9=137 (`--preserve-status` 미사용).
window_completed() {  # $1=rc — 관측 창이 '상한 발동으로' 끝났는가
  [ "$1" = "124" ] || [ "$1" = "137" ]
}

# --- ★ 08-05 검토 §39.2 — "기다렸다"고 쓰려면 **잰 사람**이 써야 한다 -----------------
# 구판의 스냅샷 실패 문장 두 자리는 상한 초를 **사실로 단정**했다:
#   `"종료 확인 8초 안에 못 받았다"` (검토자 실측 지연 0.408초) · `"5초 주기 × 12초 대기했다"`.
# 근인은 값이 아니라 **구조**다 — 재는 일(t0/t1)과 말하는 일(문장)이 자리마다 흩어져 있으면
# 반드시 갈라진다. 관측 창 3자리는 §30.3 에서 `elapsed_ms` 를 받았는데 스냅샷은 못 받은 것이
# 정확히 그 갈라짐이다. → 재기·판정 근거·설명을 **아래 세 함수 한 곳**에 모은다.
# 호출자는 상한을 고르고 결과를 묻기만 하며, 초를 문장에 손으로 적지 않는다.
#
# ⚠ `hard_timeout <초> ros2 …` 인접은 **호출 자리에 그대로 남긴다.** 명령까지 함수로 감싸면
#   `tools/scan_unbounded_cli.py` 가 위반으로 잡는다 — 그 검사기는 wrapper 화이트리스트를
#   **일부러** 갖지 않기 때문이다(§13.3: 목록이 있으면 목록 밖 wrapper 로 조용히 뚫린다).
#   그래서 상한을 씌우는 일만 호출자가 하고, 재는 일·말하는 일은 여기가 한다.
CLK_T0=0; CLK_RC=0; CLK_MS=0; CLK_LIMIT=0; CLK_OUT=""
clock_begin() { CLK_T0=$(date +%s%N); }
clock_end() {   # $1=rc(바로 다음 줄에서 $? 를 넘긴다) $2=상한(초) $3=출력파일("" 가능)
  CLK_RC="$1"; CLK_LIMIT="$2"; CLK_OUT="${3:-}"
  CLK_MS=$(( ($(date +%s%N) - CLK_T0) / 1000000 ))
}

# --- ★ 08-07 검토 §39-R.1 — **'상한 상태값'은 '상한이 발동한 사실'이 아니다** ------------
# rc 124/137 은 timeout 이 발동했을 때 *나오는 값*이지 발동했다는 *증거*가 아니다.
# 자식이 스스로 `exit 124` 해도 똑같은 값이 온다 — 검토자 재현에서 스냅샷 6자리가 **3~4ms**
# 만에 끝나고도 *"상한이 실제로 발동했다"* 를 출력했다(관측 창 3자리는 §30.3 에서 이미 벽시계를
# 받았기 때문에 같은 입력에서 정상적으로 거부했다). 그래서 근거는 **둘**이어야 한다:
#   ① 종료 상태가 상한값인가(124/137)  ② 벽시계가 계약 상한을 실제로 채웠는가.
#
# ⚠ **허용 오차는 0 이다.** `t0` 은 `timeout` 을 부르기 **전에** 찍히므로 진짜 발동은 언제나
#   계약 상한 이상으로 측정된다(프로세스 기동 몫이 여유로 들어간다 — 실측 20002ms/12002ms).
#   오차를 두는 순간 이 검토가 잡은 '짧은 124' 가 그 폭만큼 다시 통과한다.
# ⚠ 비교 규칙을 여기 **한 곳**에만 둔다. 자리마다 부등식을 손으로 적으면 여섯 호출자가 서로
#   다른 판정을 쓰게 된다 — 그 갈라짐이 §39.2·§39-R.1 두 번의 근인이었다.
clock_reached_limit() {  # $1=실측 경과ms $2=상한(초) — 벽시계가 계약 상한을 채웠는가
  [ "$1" -ge "$(( $2 * 1000 ))" ]
}
limit_actually_fired() { # $1=rc $2=실측 경과ms $3=상한(초) — 상한이 **정말** 발동했는가
  window_completed "$1" && clock_reached_limit "$2" "$3"
}

# 스냅샷 실패 분기의 **유일한** 설명자. 여기서 말하는 것은 전부 관측된 사실이다:
#   실측 rc · 실측 경과 · 그리고 상한은 '주장'이 아니라 **계약값**으로만 표기한다.
# 원인 문자열을 반드시 남긴다 — rc 124(상한 발동 = DDS 지연)와 rc 1(타입 판별 실패 =
# 토픽 부재·노드 미기동)은 **원인도 조치도 다른데** 구판은 한 문장으로 뭉갰다.
why_snapshot() {
  ng "  → rc=$CLK_RC · 실측 경과 ${CLK_MS}ms · 이 검사의 상한 계약 ${CLK_LIMIT}초"
  case "$CLK_RC" in
    124|137)
      # ★ 여기서 rc 하나로 단정하던 것이 검토 §39-R.1 이다. 두 근거를 함께 본다.
      if clock_reached_limit "$CLK_MS" "$CLK_LIMIT"; then
        ng "     rc $CLK_RC + 경과가 계약 상한 이상 = 상한이 **실제로 발동**했다 — 상대가 끝까지 응답하지 않았다"
      else
        ng "     rc $CLK_RC 는 상한 **상태값**이지만 벽시계가 모자라다 — 상한은 **발동하지 않았다**"
        ng "     → 자식이 스스로 그 값으로 끝났거나 시각이 이상하다. DDS 장기 응답 정지로 기록하지 않는다"
        ng "     → 아래 원문과 CLI 오류·토픽 부재부터 본다 (원인도 조치도 다르다)"
      fi ;;
    0)       ng "     rc 0 = 명령 자체는 성공했다 — 실패 원인은 아래 출력 내용 쪽이다" ;;
    *)       ng "     rc $CLK_RC = 상한 전에 **스스로** 끝났다 — 토픽 부재·타입 판별 실패·CLI 오류" ;;
  esac
  if [ -n "$CLK_OUT" ] && [ -s "$CLK_OUT" ]; then
    ng "     원문 앞 3줄:"
    head -3 "$CLK_OUT" | sed 's/^/       /'
  else
    ng "     (원문 출력이 비어 있다 — 명령이 아무 말도 남기지 않았다)"
  fi
}

EKF_NODE="ekf_filter_node"         # ★ ekf_real.yaml 최상단 키와 같은 이름이어야 한다
HZ_SECS_MAX=120                    # 상한. 현장 판정은 유한 시간에 끝나야 한다

SKIP_SIGN=0
SKIP_ESTOP=0
# ★ 인자 파서 (08-02 검토 §29.6) — 구판은 `--secs` 뒤에 값이 없어도 `shift 2` 를 불러
#   shift 실패 → 위치 인자가 그대로 → **같은 옵션을 영원히 다시 처리**했다(무한 루프).
#   검토자 실측: 종료 3 이 아니라 외부 timeout rc 124. 그래서 shift 전에 개수를 먼저 본다.
while [ $# -gt 0 ]; do
  case "$1" in
    --no-sign)   SKIP_SIGN=1;  shift ;;
    --no-estop)  SKIP_ESTOP=1; shift ;;
    --no-manual) SKIP_SIGN=1; SKIP_ESTOP=1; shift ;;
    --secs)
      [ $# -ge 2 ] || { echo "--secs 에 값이 없다 (예: --secs 8)"; exit 3; }
      HZ_SECS="$2"; shift 2 ;;
    # 🔴 검토 §69.2 — 굽기 직후 컴파일 기록의 build 문자열을 넘겨 **기계로 대조**한다.
    #   없으면 검사 7 의 build 행이 `ok` 가 아니라 **미판정(warn)** 으로 나간다.
    --expect-build)
      [ $# -ge 2 ] || { echo "--expect-build 에 값이 없다 (예: 'Aug 14 2026 09:12:33')"; exit 3; }
      D0_EXPECT_BUILD="$2"; shift 2 ;;
    -h|--help) sed -n '2,60p' "$0"; exit 3 ;;
    *) echo "알 수 없는 인자: $1  (사용법은 --help)"; exit 3 ;;
  esac
done
# 음수(`-`)·소수점·문자는 전부 여기서 걸린다. 빈 값도 마찬가지다.
case "$HZ_SECS" in ''|*[!0-9]*) echo "--secs 는 0 이상의 정수여야 한다: '$HZ_SECS'"; exit 3 ;; esac
[ "$HZ_SECS" -lt 3 ] && { echo "--secs 는 3 이상이어야 한다(표본이 너무 적다): $HZ_SECS"; exit 3; }
[ "$HZ_SECS" -gt "$HZ_SECS_MAX" ] && {
  echo "--secs 는 $HZ_SECS_MAX 이하여야 한다(현장 판정이 끝나지 않는다): $HZ_SECS"; exit 3; }

FAIL=0
SKIPPED=0
TMP=$(mktemp -d -t d0check.XXXXXX) || { echo "임시 디렉터리 생성 실패"; exit 1; }
trap 'rm -rf "$TMP"' EXIT

ok()   { printf '  \033[32mOK\033[0m   %s\n' "$1"; }
ng()   { printf '  \033[31mFAIL\033[0m %s\n' "$1"; FAIL=1; }
warn() { printf '  \033[33m⚠\033[0m    %s\n' "$1"; }
skip() { printf '  \033[33m--\033[0m   %s (생략)\n' "$1"; SKIPPED=$((SKIPPED+1)); }

# --- 검사 번호는 손으로 적지 않는다 (08-02) ----------------------------------
# 신설 2건이 손으로 `[6]` 을 적어 **번호가 중복**됐고("검사 6 FAIL" 이 어느 쪽인지 알 수 없다),
# 문서는 "7검사"라고 적었는데 실제로는 8개였다. 순서를 세는 일을 사람에게서 뺏는다.
# ⚠ 명령치환 `$(next_idx)` 로 쓰면 **서브셸**이라 증가가 부모로 전파되지 않는다
#   (08-02 에 실제로 그렇게 짰다가 여덟 검사가 전부 `[1]` 로 찍혔다 — 손으로 적던 중복보다
#    나빠졌다). 그래서 값을 돌려주지 않고 **전역 IDX 를 올리기만** 하고, 호출자가 $IDX 를 쓴다.
IDX=0
next_idx() { IDX=$((IDX + 1)); }

# ★ 08-05 검토 §39.3 P2-1 — 구판 머리말은 `"주기 ${HZ_SECS}초 × 2회"` 만 말해서 실제 예산을
#   **축소 표기**했다(스냅샷 상한 9자리가 빠져 있었다). 숫자를 손으로 적지 않고 실제 상한
#   변수로 계산한다 — 그래야 `--secs` 를 바꾸든 상수를 바꾸든 문장이 사실과 갈라지지 않는다.
# ★★ 08-07 검토 §39-R.2 P2-A — **명목 상한 합계는 최악이 아니다.** `hard_timeout` 은
#   `timeout --kill-after="$D0_KILL_GRACE"` 라, TERM 을 무시하는 자식은 상한 **+ 유예**까지 산다
#   (검토자 실측: `--kill-after=2s 1s` → rc=137 · **3002ms**). 그래서 유예를 호출 수만큼 더한다.
#   ⚠ 호출 수도 손으로 적지 않는다 — **도달 가능한 호출을 목록으로 한 번 적고** 합계·개수를
#     그 목록에서 센다. 수를 따로 적으면 목록과 갈라진다(`AGENTS.md §3-10 ★①`).
# ★★★ 08-07 검토 §46.1 P2 — 유예를 더해도 **그 값은 여전히 "최악"이 아니다.** 호출마다
#   프로세스 기동·파이썬 임포트·DDS discovery·파싱 몫이 더 붙는데, 그건 `timeout` 이 세는
#   시간 밖이라 우리가 상한을 걸지 않았다. 상한을 못 거는 시간을 포함해 놓고 "최악"이라
#   부르면, 그 이름을 믿은 사람이 실제로 더 오래 걸릴 때 이상으로 읽는다.
#   🔴 **상한을 못 걸면 이름을 정직하게 바꾼다** — `최악` → `timeout+유예 명목 예산`.
#   "최악"이라는 낱말을 다시 쓰려면 **기동 지연까지 상한 안에 넣은 뒤에만** 쓴다.
plan_calls() {   # 이 실행에서 실제로 도달하는 hard_timeout 호출의 상한(초)을 한 줄씩
  printf '%s\n' "$HZ_SECS" "$HZ_SECS"                # [2][3] 관측 창
  printf '%s\n' "$SNAP_ECHO_SECS" "$SNAP_ECHO_SECS"  # [2][3] 창 끝 수신 확인
  printf '%s\n' "$SNAP_INFO_SECS" "$SNAP_INFO_SECS"  # [4][5] QoS
  [ "$SKIP_SIGN" = "1" ]  || printf '%s\n' "$HZ_SECS"                          # [6] 부호
  printf '%s\n' "$FW_INFO_SECS"                                                # [7] 펌웨어
  [ "$SKIP_ESTOP" = "1" ] || printf '%s\n' "$SNAP_ECHO_SECS" "$SNAP_ECHO_SECS" # [8] 평상시·누름
  printf '%s\n' "$SNAP_INFO_SECS" "$SNAP_INFO_SECS"  # [재확인] 두 토픽
}
BUDGET=0; CALLS=0
while read -r _dur; do
  CALLS=$(( CALLS + 1 )); BUDGET=$(( BUDGET + _dur ))
done < <(plan_calls)
BUDGET_NOMINAL=$(( BUDGET + CALLS * D0_KILL_GRACE ))

echo "=== D+0 연결 판정 ($(date '+%Y-%m-%d %H:%M:%S')) ==="
echo "    관측 창: ${HZ_SECS}초 (--secs) × 2회"
echo "    스냅샷 상한: echo ${SNAP_ECHO_SECS}초 · info ${SNAP_INFO_SECS}초 · 펌웨어 ${FW_INFO_SECS}초"
echo "    → 사람이 손을 쓰는 시간을 뺀 **timeout+유예 명목 예산 ${BUDGET_NOMINAL}초**"
echo "       = 명목 상한 합 ${BUDGET}초 + TERM 무시 유예 ${D0_KILL_GRACE}초 × 도달 가능 호출 ${CALLS}회"
echo "       🔴 이것은 **벽시계 최악 상한이 아니다** — 프로세스 기동·임포트·discovery·파싱이"
echo "          호출 ${CALLS}회마다 더 붙는다. 그 몫엔 상한을 걸지 않았다"
echo "    실측 경과는 관측 창·창 끝 수신과 **실패한** 스냅샷이 ms 로 찍는다"
echo "       (성공한 QoS·펌웨어·E-stop·재확인은 ms 를 찍지 않는다)"
echo

# ── [1] 시리얼 장치 ─────────────────────────────────────────────────────────
# 여기서 막히면 뒤의 검사는 전부 "안 온다"로만 나와 원인이 안 보인다. 먼저 가른다.
next_idx; echo "[$IDX] Teensy 시리얼 장치"
if [ -e "$TEENSY_DEV" ]; then
  ok "$TEENSY_DEV 존재 (udev 규칙이 먹었다)"
elif [ -e /dev/ttyACM0 ]; then
  warn "$TEENSY_DEV 는 없는데 /dev/ttyACM0 은 있다 — udev 규칙 미적용"
  warn "  → tools/udev/99-teensy-drive.rules 를 설치하고 규칙을 다시 읽힌다"
  warn "  → 그 전까지는 agent 를 /dev/ttyACM0 으로 띄워도 진행은 된다"
  ng "$TEENSY_DEV 없음 (장치 번호가 매번 바뀌는 상태 — 인수 전에 고친다)"
else
  ng "시리얼 장치를 못 찾았다 ($TEENSY_DEV · /dev/ttyACM0 둘 다 없음)"
  ng "  → USB 케이블 · 로봇 전원 · dmesg | tail 을 본다"
fi
echo

# ── [2][3] 발행 주기 ────────────────────────────────────────────────────────
# `ros2 topic hz` 는 평균뿐 아니라 min/max/std dev/window 까지 찍어 준다.
# 평균만 보면 "가끔 한 번씩 길게 끊기는" 상태를 놓친다 — EKF 를 죽이는 건 평균이 아니라
# **최대 간격**이다. 그래서 최대 간격도 함께 본다.
# ⚠ 단, 여기 통과는 상한을 **증명하지 않는다.** 관측 창이 몇 초짜리 하나뿐이기 때문이다
#   (구동부 회신의 '최대 30ms'·'20~23ms' 가 정확히 그 한계였다 — REAL_ROBOT_VALUES.md §1).
#   진짜 판정은 R3 rosbag 의 간격 히스토그램이다 (docs/D1_FIRST_STEP.md).

# 숫자 판정에 쓸 값이 **정말 유한한 십진수인가**. `N/A`·`nan`·`inf`·빈 값은 전부 거짓이다.
# ★ 08-02 검토 §29.4: 구판은 이걸 안 물어서 `max: N/A` 가 awk 곱셈에서 **0.00ms** 가 됐다
#   — 판독 실패가 '완벽한 간격'으로 둔갑했다. 숫자로 쓸 것은 숫자인지 먼저 묻는다.
is_finite_num() { printf '%s' "${1:-}" | grep -qE '^[0-9]+(\.[0-9]+)?$'; }

check_hz() {  # $1=토픽 $2=기대 주기(참고용) $3=검사 번호
  local topic="$1" expect="$2" idx="$3" out="$TMP/hz$3.txt" tail_out="$TMP/tail$3.txt"
  local parsed rate gapms win need
  local hz_rc elapsed_ms need_ms
  echo "[$idx] $topic 발행 주기 (기대 약 ${expect}Hz)"
  clock_begin
  hard_timeout "$HZ_SECS" ros2 topic hz "$topic" >"$out" 2>&1
  clock_end $? "$HZ_SECS" "$out"             # ★ rc 는 바로 다음 줄에서 받는다(뒤로 미루면 덮인다)
  hz_rc=$CLK_RC
  elapsed_ms=$CLK_MS
  need_ms=$(( HZ_SECS * 1000 ))

  # ── ⓪ 같은 관측자가 창을 **끝까지** 살아서 채웠는가 (★ 08-03 검토 §30.3) ─
  # 이 검사가 ⓐ~ⓒ 보다 **먼저** 와야 한다. 창이 성립하지 않았는데 그 안의 표본·간격을
  # 판정하면, 과거 세대의 요약을 현재 세대의 근거로 승격시키게 된다.
  #   근거 2개를 **함께** 본다 — 하나만으로는 속는다:
  #     · 종료 상태: 우리가 건 상한이 발동했는가(124/137). rc 0·rc 42 는 조기 종료다.
  #     · 벽시계   : 정말 그 시간을 썼는가. rc 만 보면 즉시 124 를 뱉는 상대에게 속는다.
  if ! window_completed "$hz_rc"; then
    ng "$topic 관측자가 ${elapsed_ms}ms 만에 rc=$hz_rc 로 **먼저 끝났다** — 창이 성립하지 않았다"
    ng "  → 요약이 정상 모양이어도 쓰지 않는다. 그 표본은 창의 일부에서만 모인 것이다"
    ng "  → 상한 발동(rc 124/137)만 완전한 창이다. rc 0 은 CLI 가 스스로 끝난 것,"
    ng "     그 밖의 값은 CLI 오류·신호 종료다. 원문 앞 3줄:"
    head -3 "$out" | sed 's/^/       /'
    echo; return
  fi
  if ! clock_reached_limit "$elapsed_ms" "$HZ_SECS"; then
    ng "$topic 관측이 ${elapsed_ms}ms 만에 끝났다 (요구 ${need_ms}ms) — **창을 다 안 썼다**"
    ng "  → 종료 상태는 상한 발동인데 벽시계가 모자라다. 시스템 시각 또는 CLI 를 의심한다"
    echo; return
  fi
  ok "$topic 관측 창 ${elapsed_ms}ms 완주 (${HZ_SECS}초 상한이 발동 rc=$hz_rc)"

  # 파서는 fail-closed: 'average rate' 를 한 번도 못 봤으면 통과시키지 않는다.
  #   빈 출력 · 경고문만 · 형식 변경이 전부 여기서 걸린다.
  #   ★ window(표본 수)도 같이 뽑는다 — 아래 '관측 창' 검사의 근거다.
  parsed=$(awk '
      /average rate:/ { rate=$3 }
      /max:/ { for (i = 1; i <= NF; i++) if ($i == "max:") { m=$(i+1) } }
      /window:/ { for (i = 1; i <= NF; i++) if ($i == "window:") { w=$(i+1) } }
      END {
        if (rate == "" || m == "" || w == "") { exit 3 }
        sub(/s$/, "", m)
        printf "%s %s %s", rate, m, w
      }' "$out")
  if [ -z "$parsed" ]; then
    ng "$topic 주기를 판독하지 못했다 — '측정값 0' 이 아니라 **판독 실패**다"
    ng "  → 원문 ${HZ_SECS}초 출력의 앞 3줄:"
    head -3 "$out" | sed 's/^/       /'
    ng "  → 아무것도 안 찍혔다면 발행이 정말 없는 것이다(QoS 탓이 아니다 — 위 전제 ⓐ)"
    echo; return
  fi
  rate=$(printf '%s' "$parsed" | cut -d' ' -f1)
  gapms=$(printf '%s' "$parsed" | cut -d' ' -f2)
  win=$(printf '%s' "$parsed" | cut -d' ' -f3)

  # ── ⓐ 숫자 형식 (판독 실패를 값으로 착각하지 않는다) ────────────────────
  if ! is_finite_num "$rate" || ! is_finite_num "$gapms" || ! is_finite_num "$win"; then
    ng "$topic 판독값이 숫자가 아니다 (rate='$rate' max='$gapms' window='$win')"
    ng "  → **판독 실패**다. 0 으로 취급해 통과시키지 않는다"
    echo; return
  fi
  gapms=$(awk "BEGIN{printf \"%.2f\", $gapms * 1000}")

  # ── ⓑ 관측 창이 실제로 채워졌는가 (★ 08-02 검토 §29.4) ──────────────────
  # 구판은 마지막 요약만 읽어서, **초반 1초만 정상이고 나머지를 죽어 있어도** 그 옛 요약이
  # 남아 통과했다. 실측(노트북): 퍼블리셔가 3초에 죽으면 `ros2 topic hz` 는 그 뒤로
  # **아무것도 안 찍고** 마지막 블록은 `46.475Hz · window 95` 로 멀쩡해 보였다.
  # → 평균이 아니라 **표본 수**를 본다. 하한 = 하한주기 × (관측시간 − 2초).
  #   (−2 는 디스커버리 지연 몫. 실측 보정: secs=3→window 48 · 8→330 이라 경계에서도 여유가 있다)
  need=$(awk "BEGIN{printf \"%d\", $HZ_MIN * ($HZ_SECS - 2)}")
  if [ "$win" -ge "$need" ]; then
    ok "$topic 표본 ${win}개 (${HZ_SECS}초 창의 최소 요구 ${need}개 이상)"
  else
    ng "$topic 표본이 ${win}개뿐이다 (최소 요구 ${need}개) — **창의 일부만 살아 있었다**"
    ng "  → 평균값이 정상이어도 믿지 않는다. 센서가 중간에 멎었을 때 정확히 이 모양이 된다"
  fi

  # ── ⓒ 창 끝에도 살아 있는가 (표본 수와 다른 질문이다) ────────────────────
  # 표본 수는 '얼마나 많이 왔나'이고, 이건 '지금도 오나'다. 창 **끝 직전**에 죽으면
  # 표본 수는 통과할 수 있으므로 따로 묻는다.
  # ⚠ 08-03 검토 §30.3 — 이건 **생존 보조 증거일 뿐**이다. 이 echo 가 성공한다고 해서
  #   앞의 요약이 완전한 창이 되지는 않는다(그 승격을 막는 것은 위 ⓪ 하나뿐이다).
  #   관측자가 죽은 뒤 센서가 이 echo 때만 복구돼도 ⓪ 에서 이미 FAIL 이라 여기 오지 않는다.
  # ⚠ 여기 rc 는 위 ⓪ 과 **정상값이 반대**다 — `--once` 는 스스로 끝나므로 rc 0 이 정상이고,
  #   상한 발동(124/137)은 '상한 안에 한 건도 확인하지 못했다'는 뜻이라 실패다.
  # ★ 08-03 D+0 실측: `/odom --once` 5회가 0·124·0·0·0, 성공도 1.996~2.315초였다.
  #   3초는 47Hz 데이터가 아니라 DDS 발견 지연에 너무 가까워 정상 발행을 거짓 FAIL 냈다.
  #   그래서 상한을 $SNAP_ECHO_SECS 로 두되 hard_timeout 으로 유한성은 유지한다. 실패하더라도
  #   이 한 번으로 '센서가 끊겼다'고 단정하지 않고 토픽/DDS 발견 경로를 함께 지목한다.
  # ★ 08-05 검토 §39.2: 구판은 출력을 `/dev/null` 로 버리고 `"8초 안에 못 받았다"` 라고
  #   **기다린 적 없는 시간**을 단정했다. 이제 재고, 원인 문자열을 남긴다.
  clock_begin
  hard_timeout "$SNAP_ECHO_SECS" ros2 topic echo "$topic" --once >"$tail_out" 2>&1
  clock_end $? "$SNAP_ECHO_SECS" "$tail_out"
  if [ "$CLK_RC" = "0" ]; then
    ok "$topic 관측 창 종료 시점에도 수신됨 (${CLK_MS}ms)"
  else
    ng "$topic 을 관측 창 종료 뒤 한 건도 못 받았다 — 발행 중단 또는 DDS 발견 실패"
    why_snapshot
  fi

  if awk "BEGIN{exit !($rate >= $HZ_MIN)}"; then
    ok "$topic 평균 ${rate}Hz (하한 ${HZ_MIN}Hz 이상)"
    if awk "BEGIN{exit !($rate < $expect * 0.9)}"; then
      warn "  실측 기대치 ${expect}Hz 보다 10% 넘게 낮다 — 구동부에게 지금 확인할 것"
    fi
  else
    ng "$topic 평균 ${rate}Hz — EKF 주기 하한 ${HZ_MIN}Hz 미만이다"
  fi

  if awk "BEGIN{exit !($gapms <= $GAP_MAX_MS)}"; then
    ok "$topic 최대 간격 ${gapms}ms (EKF 한 주기 ${GAP_MAX_MS}ms 이내)"
  else
    ng "$topic 최대 간격 ${gapms}ms — EKF 한 주기(${GAP_MAX_MS}ms)를 넘겼다"
    ng "  → EKF 가 입력 없이 지나가는 주기가 생긴다. 구동부와 지금 본다"
  fi
  echo
}
next_idx; check_hz "$ODOM_TOPIC" "$HZ_EXPECT_ODOM" "$IDX"
next_idx; check_hz "$IMU_TOPIC"  "$HZ_EXPECT_IMU"  "$IDX"

# ── [4] QoS 정합 ────────────────────────────────────────────────────────────
# ★ 이 검사가 이 스크립트에서 가장 중요하다. 여기서 잡는 고장은 **증상이 없다**:
#   BEST_EFFORT 퍼블리셔 + RELIABLE 구독자는 DDS 가 아예 매칭하지 않아 에러도 경고도
#   없이 그냥 0건이 된다. `ros2 topic hz` 는 정상인데 그 노드만 조용히 굶는다.
#
# ★ '아는 자리를 확인한다'가 아니라 **'전부 훑다가 걸리는 것을 잡는다'** 로 짰다
#   (AGENTS.md §3-10 ②). 그래서 새 노드가 나중에 붙어도 이 검사가 안 부서진다.
#   구독자 목록을 손으로 적었다면 목록과 실제가 갈라지는 순간 조용히 뚫린다.
#
# [08-02 실측] 그 목록을 지금 적을 필요가 없다는 근거이자, 예약 19 ①의 전제가 틀렸다는 근거:
#   노트북에서 실차 소비자를 전부 띄워 재어 보니 /odom 구독자 3종(ekf_filter_node ·
#   bt_navigator · controller_server)이 **이미 전부 BEST_EFFORT** 였고, velocity_smoother 는
#   (feedback: OPEN_LOOP 라) /odom 을 아예 구독하지 않았다. /imu/data 는 EKF 하나뿐이다.
#   robot_localization 3.5.4 는 **구독 QoS 오버라이드 파라미터를 아예 제공하지 않으므로**
#   yaml 로 고칠 수도 없다(퍼블리셔 3개만 연다). 그래서 조치가 아니라 **검사**로 닫는다.
#   ⚠ TODO(D+0): 확인 — Jetson 의 apt 버전이 다르면 그 결론이 바뀔 수 있다. 이 검사가
#     바로 그 재확인이다. 여기서 FAIL 이 나면 config/ekf_real.yaml 의 QoS 절을 다시 읽는다.
#
# ★★ [08-02 정정 — 펌웨어 소스 수령 후] 이 검사의 ⓐ 가 **틀린 계약을 강제하고 있었다.**
#   회신 PDF §7 은 Durability 만 적고 Reliability 를 비워 뒀고, 우리는 2차 회신의
#   "IMU QoS = BEST_EFFORT" 를 /odom 까지 확장해 읽었다. **소스가 그 확장을 부정했다:**
#
#     rclc_publisher_init_default    (&odomPublisher, ...)        // RELIABLE
#     rclc_publisher_init_best_effort(&imuPublisher, ...)         // BEST_EFFORT
#     rclc_publisher_init_default    (&estopStatePublisher, ...)  // RELIABLE
#     rclc_publisher_init_default    (&firmwareInfoPublisher, ...)// RELIABLE
#
#   (소스 주석은 이 블록을 "BEST_EFFORT / VOLATILE sensor publishers" 라고 적어 두었으나
#    `init_default` 는 rclc 에서 RELIABLE 이다. **주석이 아니라 함수 이름이 사실이다.**)
#
#   고치지 않았다면 D+0 에 `/odom 발행자가 계약과 다르다` **오경보**가 떴을 것이고,
#   런북은 "구동부에게 지금 확인한다"고 지시한다 — 인수 현장에서 가장 비싼 자원(담당자의
#   현장 시간)을 **없는 고장에** 쓰게 만든다. 오경보는 침묵보다 싸지 않다.
#
# ★ 그리고 ⓑ 의 판정 근거도 **발행자에 의존한다** — 이것도 틀려 있었다:
#     pub RELIABLE    + sub RELIABLE     → 매칭 ✅   ← RELIABLE 구독이 항상 고장은 아니다
#     pub RELIABLE    + sub BEST_EFFORT  → 매칭 ✅
#     pub BEST_EFFORT + sub BEST_EFFORT  → 매칭 ✅
#     pub BEST_EFFORT + sub RELIABLE     → 매칭 ❌   ← 이 조합만 고장이다
#   그래서 ⓑ 를 **발행자가 BEST_EFFORT 일 때만** 발동하도록 조건화한다.
#   지금 구독자는 전부 BEST_EFFORT 라 당장 오탐은 없지만, 근거가 틀린 검사는 다음에 틀린다.
check_qos() {  # $1=토픽 $2=검사 번호 $3=소스로 확정된 기대 Reliability
  local topic="$1" idx="$2" expect="${3:-}" out="$TMP/qos$2.txt"
  local rows npub nsub off_pub be_pub bad_pair ekf_rel pub_rel info_rc
  echo "[$idx] $topic QoS 정합"
  clock_begin
  hard_timeout "$SNAP_INFO_SECS" ros2 topic info "$topic" -v >"$out" 2>&1
  clock_end $? "$SNAP_INFO_SECS" "$out"
  info_rc=$CLK_RC
  # ★ 08-03 검토 §30.3 계열 — 여기는 **스냅샷**이라 rc 0 이 정상이다(위 window_completed 와 반대).
  #   상한이 발동했다면 daemon 이 매달린 것이고, 그때 남은 출력은 **잘린 목록**이다.
  #   잘린 목록으로 "RELIABLE 구독자 없음" 을 말하면 그건 안 본 것을 봤다고 하는 것이다.
  # ★ 08-05 검토 §39.2 계열 — 구판 문장은 rc 가 무엇이든 `"(20초 상한 발동 = 124)"` 를 달아
  #   rc 1(daemon 오류·토픽 부재)까지 '매달렸다'로 읽히게 했다. 원인은 why_snapshot 이 가른다.
  if [ "$info_rc" != "0" ]; then
    ng "$topic 의 topic info 가 끝까지 성공하지 못했다 — **판독 실패**"
    why_snapshot
    ng "  → 엔드포인트 목록이 잘렸을 수 있다. 잘린 목록으로 QoS 를 판정하지 않는다"
    echo; return
  fi

  # 한 엔드포인트 = "종류 노드이름 신뢰성" 한 줄로 접는다.
  #   출력 순서가 (Node name → Endpoint type → QoS profile → Reliability) 이라
  #   상태를 세 줄만 기억하면 된다.
  rows=$(awk '
      /^Node name:/       { node=$3; next }
      /^Endpoint type:/   { ep=$3;   next }
      /Reliability:/      { if (ep != "") { print ep, node, $2; ep="" } }' "$out")

  if [ -z "$rows" ]; then
    ng "$topic 의 엔드포인트를 판독하지 못했다 — **판독 실패**(형식 변경/빈 출력)"
    head -3 "$out" | sed 's/^/       /'
    echo; return
  fi

  npub=$(printf '%s\n' "$rows" | grep -c '^PUBLISHER ')
  nsub=$(printf '%s\n' "$rows" | grep -c '^SUBSCRIPTION ')
  if [ "$npub" -eq 0 ]; then
    ng "$topic 에 발행자가 없다 — agent 가 안 붙었거나 토픽 이름이 다르다"
    echo; return
  fi

  # ── ★ 소비자가 실제로 있는가 (08-02 검토 §29.3) ─────────────────────────
  # 구판은 구독자가 **0개여도** "RELIABLE 구독자 없음 = 전부 매칭됨"으로 통과시켰다.
  # 게다가 런북이 agent 만 띄운 상태에서 이 검사를 돌리게 짜여 있어, 실제로 소비자가
  # 0개인 것이 정상 경로였다 — **아무도 안 보는 것을 보고 "봤다"고 말한** 셈이다.
  # 그래서 ① 구독자 0개를 실패로 세우고 ② EKF 를 **이름으로** 요구한다.
  #   (Nav2 는 D+0 에 띄우지 않으므로 여기서 책임지지 않는다 — 그건 실제 기동 게이트 몫이다)
  if [ "$nsub" -eq 0 ]; then
    ng "$topic 에 구독자가 하나도 없다 — QoS 정합을 **판정할 대상이 없다**"
    ng "  → EKF 를 먼저 띄우고 다시 돌린다 (JETSON_SETUP.md §7):"
    ng "     ros2 run robot_localization ekf_node --ros-args \\"
    ng "       --params-file ~/ros2_ws/src/tunnel_bringup/config/ekf_real.yaml"
    echo; return
  fi
  ekf_rel=$(printf '%s\n' "$rows" \
            | awk -v n="$EKF_NODE" '$1 == "SUBSCRIPTION" && $2 == n { print $3; exit }')
  if [ -z "$ekf_rel" ]; then
    ng "$topic 을 구독하는 $EKF_NODE 가 없다 (구독자 ${nsub}개는 전부 다른 노드다)"
    ng "  → 이 검사의 목적은 **EKF 가 받을 수 있는가**다. EKF 없이는 판정이 성립하지 않는다"
    ng "  → 노드 이름을 바꿔 띄웠다면 그것도 원인이다 (yaml 키가 $EKF_NODE 다)"
    echo; return
  fi
  # DDS 호환 규칙은 하나뿐이다: **발행 BEST_EFFORT + 구독 RELIABLE 만 매칭 실패.**
  pub_rel=$(printf '%s\n' "$rows" | awk '$1 == "PUBLISHER" { print $3; exit }')
  if [ "$pub_rel" = "BEST_EFFORT" ] && [ "$ekf_rel" = "RELIABLE" ]; then
    ng "$EKF_NODE 가 $topic 을 RELIABLE 로 구독한다 — 발행이 BEST_EFFORT 라 **한 건도 못 받는다**"
    ng "  → 에러도 경고도 없이 조용히 0건이 되는 그 고장이다"
  else
    ok "$EKF_NODE 가 $topic 구독 중 ($ekf_rel) — 발행($pub_rel)과 호환"
  fi

  # ⓐ 계약 대조: 기대값은 **펌웨어 소스에서 읽은 값**이다(토픽마다 다르다).
  #   다르면 '지금 당장 고장'은 아니지만 **우리가 읽은 소스와 굽힌 펌웨어가 다르다**는 뜻이라
  #   그 자체로 큰 신호다 → FAIL 로 세운다. 같으면 조용히 통과시킨다(오경보를 만들지 않는다).
  off_pub=$(printf '%s\n' "$rows" | awk -v e="$expect" '$1 == "PUBLISHER" && $3 != e { print $2 " (" $3 ")" }')
  if [ -z "$expect" ]; then
    warn "$topic 발행자 ${npub}개 — 기대 Reliability 미지정, 대조 생략"
  elif [ -n "$off_pub" ]; then
    ng "$topic 발행자가 소스와 다르다 (기대 $expect): $(echo "$off_pub" | tr '\n' ' ')"
    ng "  → 인수받은 펌웨어가 우리가 읽은 v1.4 소스가 아닐 수 있다."
    ng "     ros2 topic echo /firmware/info --field data --full-length --once  로 정체를 먼저 확인한다"
  else
    ok "$topic 발행자 ${npub}개 전부 $expect (소스 v1.4 와 일치)"
  fi

  # ⓑ 진짜 고장: **BEST_EFFORT 발행** + RELIABLE 구독 = 매칭 안 됨(조용한 0건)
  #   발행자가 RELIABLE 이면 RELIABLE 구독도 정상 매칭이므로 여기서 걸면 오탐이다.
  be_pub=$(printf '%s\n' "$rows" | awk '$1 == "PUBLISHER" && $3 == "BEST_EFFORT" { print }')
  bad_pair=$(printf '%s\n' "$rows" | awk '$1 == "SUBSCRIPTION" && $3 == "RELIABLE" { print $2 }')
  if [ -n "$be_pub" ] && [ -n "$bad_pair" ]; then
    ng "$topic 에 RELIABLE 구독자가 있다: $(echo "$bad_pair" | tr '\n' ' ')"
    ng "  → 발행자가 BEST_EFFORT 라 이 노드들은 **에러 없이 한 건도 못 받는다.**"
    ng "  → 해당 노드의 구독 QoS 를 BEST_EFFORT 로 바꿔야 한다"
  elif [ -n "$bad_pair" ]; then
    ok "$topic 구독자 중 RELIABLE 있음 — 그러나 발행자도 RELIABLE 이라 **매칭된다**"
  else
    ok "$topic 구독자 ${nsub}개 중 RELIABLE 없음 (전부 매칭됨)"
  fi
  echo
}
# ★ 기대값 근거 = 펌웨어 소스 v1.4 (sha256 13f929cb…2106) 의 publisher 초기화 함수.
next_idx; check_qos "$ODOM_TOPIC" "$IDX" RELIABLE
next_idx; check_qos "$IMU_TOPIC"  "$IDX" BEST_EFFORT

# ── [6] 전진 부호 ───────────────────────────────────────────────────────────
# ★ 이 스크립트는 **로봇에 명령을 보내지 않는다.** 사람이 바퀴를 손으로 굴린다.
#   이유: D+0 의 로봇 상태는 R0(바퀴 공중)이고, 검증 안 된 스택이 모터를 돌리게 하는 것은
#   순서가 뒤집힌 것이다. 부호는 명령 없이도 확인된다 — 굴리면 /odom 이 반응한다.
# 왜 부호를 보는가: 반대면 EKF·SLAM·Nav2 가 전부 **거울처럼 뒤집힌 세계**에서 동작한다.
#   증상은 "지도가 이상하다"로만 나타나서 원인 찾기가 오래 걸린다.
next_idx; echo "[$IDX] 전진 시 twist.twist.linear.x 부호"
if [ "$SKIP_SIGN" = "1" ]; then
  skip "전진 부호 — --no-sign"
else
  echo "  ★ 지금 할 일: **바퀴를 손으로 '앞으로' 굴려 주세요** (바퀴가 공중에 뜬 상태 R0)."
  echo "     준비되면 Enter — 그때부터 ${HZ_SECS}초 동안 /odom 을 봅니다."
  read -r _ || true
  clock_begin
  hard_timeout "$HZ_SECS" ros2 topic echo "$ODOM_TOPIC" \
      --field twist.twist.linear.x >"$TMP/sign.txt" 2>&1
  clock_end $? "$HZ_SECS" "$TMP/sign.txt"
  SIGN_RC=$CLK_RC
  SIGN_MS=$CLK_MS

  # 숫자로 읽히는 줄만 채택한다(`---` 구분선·경고문 제외). 한 줄도 없으면 판독 실패.
  SIGN=$(awk -v lo="$SIGN_MIN" '
      $0 ~ /^-?[0-9]+(\.[0-9]+)?([eE][-+]?[0-9]+)?$/ {
        n++; if (n == 1 || $1 > mx) mx = $1; if (n == 1 || $1 < mn) mn = $1
      }
      END {
        if (n == 0) { print "READFAIL" }
        else if (mx >= lo)      { printf "OK %.4f %d", mx, n }
        else if (mn <= -lo)     { printf "REVERSED %.4f %d", mn, n }
        else                    { printf "STILL %.4f %d", mx, n }
      }' "$TMP/sign.txt")
  # ★ 08-03 검토 §30.3 계열 — 여기도 **관측 창**이라 상한 발동(124/137)이 정상 종료다.
  #   ⚠ 단 결론의 방향에 따라 조기 종료의 의미가 다르다. 그래서 일괄로 죽이지 않는다:
  #     · '움직였다'(OK/REVERSED) = **관측된 사실**이라 창이 짧아도 여전히 참이다.
  #     · '안 움직였다'(STILL/READFAIL) = **못 본 것**이다. 창이 안 채워졌으면 그건
  #       "안 움직였다"가 아니라 **판독 실패**다. 안 본 것을 결함으로 적으면 그 기록이
  #       다음 판단을 오염시킨다(§29 의 E-stop 과 같은 종류의 잘못이다).
  case "$SIGN" in
    OK*)
      ok "앞으로 굴릴 때 linear.x 최대 $(echo "$SIGN" | cut -d' ' -f2) m/s > 0 (부호 정상)"
      # ★ 08-07 §39-R.1 — 여기도 rc 만 보면 즉시 124 를 뱉는 상대에게 속아 경고가 안 뜬다.
      limit_actually_fired "$SIGN_RC" "$SIGN_MS" "$HZ_SECS" \
        || warn "  ⚠ 관측자는 ${SIGN_MS}ms 만에 rc=$SIGN_RC 로 일찍 끝났다 — 부호는 관측된 사실이라 유효하다" ;;
    REVERSED*)
      ng "부호가 **반대**다 — 앞으로 굴렸는데 linear.x 최소 $(echo "$SIGN" | cut -d' ' -f2) m/s"
      ng "  → 펌웨어의 좌우/전후 부호를 구동부와 지금 맞춘다. URDF 로 덮지 말 것" ;;
    READFAIL|STILL*)
      if ! window_completed "$SIGN_RC"; then
        ng "/odom 관측자가 ${SIGN_MS}ms 만에 rc=$SIGN_RC 로 **먼저 끝났다** — 판독 실패다"
        ng "  → '안 움직였다'가 아니라 **못 봤다**. 다시 돌린다(agent 연결부터 확인)"
        head -3 "$TMP/sign.txt" | sed 's/^/       /'
      # ★ 08-05 — 관측 창 클래스의 나머지 한 자리에도 §30.3 의 **벽시계** 규칙을 준다.
      #   `topic hz` 자리는 rc 와 벽시계를 둘 다 보는데 여기는 rc 만 봤다. 결론이
      #   '안 움직였다'(= 못 본 것)일 때는 창이 실제로 채워졌는지가 판정의 전제다.
      elif ! clock_reached_limit "$SIGN_MS" "$HZ_SECS"; then
        ng "/odom 관측이 ${SIGN_MS}ms 만에 끝났다 (요구 $(( HZ_SECS * 1000 ))ms) — **창을 다 안 썼다**"
        ng "  → 종료 상태는 상한 발동인데 벽시계가 모자라다. 시스템 시각 또는 CLI 를 의심한다"
      elif [ "$SIGN" = "READFAIL" ]; then
        ng "/odom 값을 한 줄도 못 읽었다 — **판독 실패**(값이 0 이었다는 뜻이 아니다)"
        head -3 "$TMP/sign.txt" | sed 's/^/       /'
      else
        ng "움직임이 관측되지 않았다 (최댓값 $(echo "$SIGN" | cut -d' ' -f2) m/s)"
        ng "  → 바퀴를 굴리는 동안 측정됐는지, 엔코더가 붙어 있는지 확인한다"
      fi ;;
    *)
      ng "부호 판정 자체가 실패했다: $SIGN" ;;
  esac
fi
echo

# ── [7] 펌웨어 정체 ─────────────────────────────────────────────────────────
# [08-02 신설] 펌웨어 소스를 받고 나서야 가능해진 검사다.
# `/firmware/info` 는 버전·게인·바퀴 반지름·라이브러리 목록을 한 줄로 방송한다.
# ⚠ 이 값으로 **버전을 판별할 수는 없다** — 소스 v1.4 인데 FW_VERSION 은 "1.3.0",
#   FW_SOURCE_PATH 는 v1_3, FW_GIT_SHA 는 0 으로 채워져 있다(소스 36~39행).
#   그래도 **바퀴 반지름·게인·baud 가 우리가 읽은 소스와 같은지**는 여기서 갈린다.
# ⚠ 발행 주기가 5초(FW_INFO_PERIOD_MS)라 --once 는 최대 5초를 기다린다. VOLATILE 이라
#   지나간 것은 못 받는다 — 타임아웃을 넉넉히 준다.
# ★ 08-03 D+0 실측: Humble `topic echo` 는 긴 문자열을 기본 128자에서 `...` 로 잘랐다.
#   wheel_radius·게인은 그 뒤에 있어 정상 펌웨어가 불일치로 오판됐다. 이 스크립트의
#   `topic echo --once` 전수를 대조하면, 장문을 판정에 쓰는 곳은 여기 하나뿐이다
#   (나머지는 짧은 bool·숫자 또는 값을 버리는 생존 확인). `--full-length` 를 빼지 않는다.
next_idx; echo "[$IDX] 펌웨어 정체 (/firmware/info · 5초 주기)"
FWOUT="$TMP/fw.txt"
clock_begin
hard_timeout "$FW_INFO_SECS" ros2 topic echo /firmware/info --field data --full-length --once \
   >"$FWOUT" 2>&1
clock_end $? "$FW_INFO_SECS" "$FWOUT"
if [ "$CLK_RC" = "0" ] && [ -s "$FWOUT" ]; then
  sed 's/^/       /' "$FWOUT" | head -6
  # 🔴 08-13 (검토 §65.3) — 기대값을 여기 적지 않고 **`.ino` 에서 읽어** 대조한다.
  #   구판은 `wheel_radius=0.05698` 을 스크립트 안에 박아 두어, 08-13 재교정 뒤
  #   정상 펌웨어를 "다른 펌웨어" 로 거절하게 돼 있었다. 정체 검사가 소스를 거절하면
  #   그건 검사가 아니라 장애물이다.
  #   ⚠ 굽기 **전**에는 이 검사가 NG 로 나오는 것이 정상이다 — 보드에 아직 옛 펌웨어가
  #     들어 있다는 사실을 정확히 말하는 것이다.
  # 🔴 08-13 밤 2차 — 기대 **키 목록**도 손으로 안 든다. 앞 판은 네 키를 튜플로 박아
  #   두었는데, 같은 날 `CONTROL_WHEEL_RADIUS` 를 지우자(예약 32-e) 목록이 `.ino` 와
  #   어긋나 검사가 통째로 멈췄다 — 값을 베낀 것(§65.3)과 같은 병이 **목록**에서 재발.
  #   이제 `/firmware/info` 의 format·인자에서 `이름=%.Nf` 짝을 직접 읽는다.
  FW_KEYS=$(PYTHONPATH="$D0_ROOT/tools" python3 -c \
    'import sys; from firmware_constants import firmware_identity_keys
try:
    print("\n".join(firmware_identity_keys()))
except Exception as exc:
    print(exc, file=sys.stderr); sys.exit(1)' 2>/dev/null)
  if [ -z "$FW_KEYS" ]; then
    ng '.ino 에서 기대 상수를 못 읽었다 — 정체 검사를 못 한다 (도구를 먼저 고쳐라)'
  else
    # 🔴 잘린 표본을 정상으로 받지 않는다 (검토 §65.5 의 |TRUNCATED 표식).
    if grep -q "|TRUNCATED" "$FWOUT"; then
      ng "/firmware/info 가 버퍼를 넘어 **잘렸다** — 이 표본으로는 아무것도 판정 못 한다"
    fi
    while IFS= read -r expect; do
      [ -z "$expect" ] && continue
      if grep -qF "$expect" "$FWOUT"; then
        ok "$expect — 소스와 일치"
      else
        case "$expect" in
          kp=*) warn "제어 게인이 소스($expect)와 다르다 — 시험 데이터의 전제가 달라진다" ;;
          *)    ng "$expect 가 아니다 — **다른 펌웨어가 구워져 있다**" ;;
        esac
      fi
    done <<< "$FW_KEYS"

    # 🔴 검토 §68.2 — 정본은 **`build` 문자열이 굽힘 판별의 유일한 기준**이라고 말하는데
    #   앞 판 검사 7 은 실수 필드만 보고 `build` 를 아예 안 봤다. 화면을 사람이 읽는
    #   절차와 자동 검사를 같은 "정체 검사" 로 부르면 안 된다.
    #   ⚠ 여기서 "오늘 굽었나" 는 판정하지 않는다 — 이 스크립트는 언제 구웠는지 모른다.
    #     관측한 사실(`build` 값)을 **출력에 남겨** 사람이 절차서와 대조하게 한다.
    FW_BUILD=$(grep -o 'build=[A-Za-z]* *[0-9]* [0-9]* [0-9:]*' "$FWOUT" | head -1)
    if [ -z "$FW_BUILD" ]; then
      ng "/firmware/info 에 build= 가 없다 — **굽힘 판별의 정본이 없는 표본**이다"
    elif [ -n "$D0_EXPECT_BUILD" ]; then
      # 🔴 검토 §69.2 — 굽기 직후 컴파일 기록의 기대 문자열과 **기계로 대조**한다.
      if [ "$FW_BUILD" = "build=$D0_EXPECT_BUILD" ]; then
        ok "$FW_BUILD — 기대와 일치"
      else
        ng "$FW_BUILD 가 기대 build=$D0_EXPECT_BUILD 와 다르다 — **구판이 올라가 있다**"
      fi
    else
      # 🔴 존재만 보고 `ok` 를 내면 구판 build 나 `Foo 99 99:99:99` 도 초록이 된다.
      #   기대값이 없으면 **미판정**이다 — 자동 완료조건에서 뺀다 (검토 §69.2).
      warn "$FW_BUILD — 🔴 **미판정**. 기대값 없이는 stale build 를 못 가른다"
      warn "     굽기 직후라면 --expect-build '<컴파일 시각>' 으로 기계 대조한다"
    fi
  fi
else
  # ★ 08-05 검토 §39.2 — 구판은 `"(5초 주기 × 12초 대기했다)"` 라고 적고 실제로는 0.4초 만에
  #   끝난 적이 있다. 바로 아랫줄이 *"노드가 없다면"* 을 맞게 짚어 놓고 윗줄이 그 경우와
  #   모순되는 사실을 주장했다. 이제 왜 끝났는지는 why_snapshot 이 rc 로 가른다.
  ng "/firmware/info 를 한 건도 못 읽었다 (발행 주기 5초 · VOLATILE)"
  why_snapshot
  ng "  → agent 는 붙었는데 노드가 없다면 **IMU 초기화 실패**를 먼저 의심한다"
  ng "     (소스: IMU 실패 시 errorLoop() → micro-ROS 노드 자체가 안 뜬다."
  ng "      Teensy LED 가 100ms 주기로 빠르게 깜빡이면 그 상태다)"
fi
echo

# ── [8] E-stop 배선 ─────────────────────────────────────────────────────────
# [08-02 신설] 최종 회신 PDF 8쪽에는 E-stop 이 한 번도 안 나왔지만 **소스에는 있다**
#   (ESTOP_PIN=21, INPUT_PULLUP, 차단 5지점).
# 🔴 [08-07 검토 §45.3 P2 정정] **극성은 active-HIGH 다.** 구판 주석은 `active-low` 라고
#   적어 실제 펌웨어와 **반대**였다 — `.ino:117` 이 `ESTOP_ACTIVE_LOW = false` 이고
#   (2026-08-06 PIN21 2단계에서 뒤집었다 · `ELECTRICAL_BASELINE.md §4-c`), 판정은 `.ino:337`:
#     rawHigh = (digitalRead(ESTOP_PIN)==HIGH) ; pressed = ESTOP_ACTIVE_LOW ? !rawHigh : rawHigh
# ★ 이 검사가 필요한 이유 — **극성이 뒤집히면서 이유도 뒤집혔다**: 핀에 아무것도 안 물려
#   있으면 INPUT_PULLUP 이 HIGH 로 띄우는데, active-high 에서 그 HIGH 는 **"눌림 = 정지"** 다.
#   즉 미배선은 08-06 부터 "안 눌림"이 아니라 **항상 정지**로 굳는다(안전 쪽 실패라 정상이며,
#   `ELECTRICAL_BASELINE.md §7` 이 이를 상태 ③ 으로 적어 둔 그 값이다).
#   ⚠ 어느 극성이든 **'토픽이 온다'로는 배선이 증명되지 않는다.** 평상시 false 를 먼저 보고
#   눌러서 **false→true 로 실제 바뀌는지**만이 증거다 — 아래가 그 순서다.
# ⚠ 발행 주기 1Hz(DIAGNOSTIC_PERIOD_MS) — 합의서의 10Hz 가 아니다. 최대 1초 기다린다.
next_idx; echo "[$IDX] E-stop 배선 (/estop/state · 1Hz)"
if [ "$SKIP_ESTOP" = "1" ]; then
  skip "E-stop 배선 — 사람이 버튼을 눌러야 한다 (--no-estop)"
else
  ESOUT="$TMP/estop_idle.txt"
  clock_begin
  hard_timeout "$SNAP_ECHO_SECS" ros2 topic echo /estop/state --field data --once >"$ESOUT" 2>&1
  clock_end $? "$SNAP_ECHO_SECS" "$ESOUT"
  if [ "$CLK_RC" = "0" ] && [ -s "$ESOUT" ]; then
    if grep -qi "false" "$ESOUT"; then
      ok "평상시 /estop/state = false"
      # ★ 08-02 검토 §29 계열 정정 — 구판은 "누르지 않았으면 그냥 Enter" 라고 안내해 놓고
      #   안 누르면 **"배선돼 있지 않다"** 로 FAIL 하고 주행 금지까지 지시했다.
      #   "안 눌렀다"와 "눌렀는데 안 바뀐다"는 **다른 사실**이다 — 관측하지 않은 것을
      #   결함으로 기록하면 그 기록이 다음 판단을 오염시킨다. 그래서 사람에게 직접 묻는다.
      echo "     ▶ E-stop 버튼을 **누른 채로** Enter 를 누르세요."
      echo "       버튼이 없거나 지금 누를 수 없으면  s  + Enter (건너뜁니다)"
      read -r ES_ANS || ES_ANS=""
      case "$ES_ANS" in
        s|S)
          skip "E-stop 배선 — 사람이 누르지 못했다 (배선 결함과 **구분해서** 기록한다)"
          warn "  → 눌러 보기 전에는 배선 여부를 알 수 없다. 인수 전에 반드시 확인할 것" ;;
        *)
          ESOUT2="$TMP/estop_press.txt"
          clock_begin
          hard_timeout "$SNAP_ECHO_SECS" ros2 topic echo /estop/state --field data --once \
             >"$ESOUT2" 2>&1
          clock_end $? "$SNAP_ECHO_SECS" "$ESOUT2"
          if [ "$CLK_RC" = "0" ] && [ -s "$ESOUT2" ]; then
            if grep -qi "true" "$ESOUT2"; then
              ok "누름 → true 전환 확인 — **배선 정상**"
            else
              ng "**눌렀다고 했는데** false 그대로다 — E-stop 이 실차에 배선돼 있지 않다"
              ng "  → 소프트웨어 정지 수단은 /cmd_vel 0 과 watchdog 뿐이다(둘 다 소프트웨어 경로)"
              ng "  → 배선 전까지 **R1 이상 지면 주행을 하지 않는다** (인수 후 별도 일정)"
            fi
          else
            ng "누른 뒤 /estop/state 를 못 읽었다 — **판독 실패**(배선 없음과 다른 사실이다)"
            why_snapshot
            ng "  → agent 연결이 끊겼는지 먼저 본다. 끊겼다면 배선 판정을 하지 않는다"
          fi ;;
      esac
    else
      ng "평상시부터 /estop/state = true — 버튼이 눌린 채이거나 배선이 반대다"
      ng "  → 이 상태에서는 모터가 전혀 돌지 않는다(차단 5지점 전부 발동)"
    fi
  else
    ng "/estop/state 를 못 읽었다 — 토픽이 없다면 굽힌 펌웨어가 v1.4 가 아니다"
    why_snapshot
  fi
fi
echo

# ── 종료 직전 재확인: EKF 가 아직도 살아 있는가 ─────────────────────────────
# ★ 08-02 검토 §29.3 '전환' 회귀 — *"EKF 가 검사 도중 죽어 endpoint 가 사라지면 최종 판정은
#   PASS 금지."* QoS 검사는 **순간 스냅샷**이라, 그 순간만 살아 있으면 통과한다.
#   판정의 전제가 판정 뒤에 사라지면 그 판정은 이미 무효다 → 끝에서 한 번 더 묻는다.
echo "[재확인] $EKF_NODE 가 아직 두 토픽을 구독 중인가"
for t in "$ODOM_TOPIC" "$IMU_TOPIC"; do
  RECHECK_OUT="$TMP/recheck${t//\//_}.txt"
  # ★ 08-05 — 구판은 stderr 를 `2>/dev/null` 로 버려 실패 원인을 **캡처조차 못 했다**.
  #   검토가 지목한 두 자리 밖이지만 같은 클래스라 여기서 함께 닫는다(§3-10 전수).
  clock_begin
  hard_timeout "$SNAP_INFO_SECS" ros2 topic info "$t" -v >"$RECHECK_OUT" 2>&1
  clock_end $? "$SNAP_INFO_SECS" "$RECHECK_OUT"
  RECHECK_RC=$CLK_RC
  if [ "$RECHECK_RC" != "0" ]; then
    ng "$t 의 최종 topic info 가 끝까지 성공하지 못했다 — **판독 실패**"
    why_snapshot
    ng "  → EKF 행이 일부 출력됐어도 스냅샷이 완주하지 않았으므로 유지 판정하지 않는다"
    continue
  fi
  if awk '/^Node name:/ { node=$3 } /^Endpoint type:/ { ep=$3 }
            /Reliability:/ { if (ep == "SUBSCRIPTION") print node; ep="" }' \
       "$RECHECK_OUT" | grep -qx "$EKF_NODE"; then
    ok "$t — $EKF_NODE 구독 유지"
  else
    ng "$t 을 구독하던 $EKF_NODE 가 사라졌다 — 검사 도중 죽었다"
    ng "  → 앞선 QoS 통과는 **더 이상 근거가 아니다.** EKF 로그를 보고 다시 돌린다"
  fi
done
echo

# ── 종합 ────────────────────────────────────────────────────────────────────
echo "=========================================="
echo "검사 $IDX 개 수행 (문서의 검사 수와 다르면 문서가 낡은 것이다)"
if [ "$FAIL" != "0" ]; then
  echo "❌ D+0 판정 실패 — 구동부가 자리에 있을 때 위 FAIL 을 함께 본다."
  echo "   (돌아간 뒤에 발견하면 같은 문제에 며칠이 든다)"
  exit 1
elif [ "$SKIPPED" != "0" ]; then
  echo "🟨 검사한 것은 전부 통과했으나 $SKIPPED 개를 건너뛰었다 — **전량 통과가 아니다.**"
  echo "   건너뛴 검사를 채우기 전에는 'D+0 통과' 라고 기록하지 않는다."
  exit 2
else
  # 🔴 08-13 — 구판은 여기서 "다음은 D1_FIRST_STEP.md (R3 rosbag)" 이라고 못 박았다.
  #   이 스크립트는 **연결이 성립하는가**만 본다. 사다리 어디에 서 있는지는 모른다.
  #   실제로 08-13 저녁, 굽기 직후의 이 줄이 §7-c-E·§5-G6·R2 셋을 건너뛰고 R3 로 가라고
  #   가리켰다 — 그 시점 계약은 정반대였다(굽기 → R2 → R3). 도구가 모르는 것을 단정하면
  #   그 단정이 사람의 순서를 이긴다. → **다음 단계는 핸드오프가 말한다.**
  echo "✅ D+0 판정 통과 — **연결·정체·QoS·배선**이 성립한다."
  echo "   🔴 다음 단계는 이 스크립트가 정하지 않는다 — docs/CURRENT_HANDOFF.md 의"
  echo "      완료조건이 정본이다. 사다리 순서(R0→R1→R2→R3)를 건너뛰지 않는다."
  echo "      🔴 펌웨어를 새로 구웠다면 §7-c-E 13행 · §5-G6 10/10 이 R1·R2 앞에 있다."
  echo "   ⚠ 여기 통과는 **주기의 상한을 증명하지 않는다**(관측 창이 짧다)."
  echo "     최대 간격의 정본 판정은 R3 rosbag 의 간격 히스토그램이다"
  echo "     (docs/D1_FIRST_STEP.md — 사다리가 거기까지 왔을 때)."
  exit 0
fi
