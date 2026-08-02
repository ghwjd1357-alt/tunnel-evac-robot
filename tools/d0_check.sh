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
#   bash tools/d0_check.sh --secs 15    # 주기 측정 시간(기본 8초, 허용 3~120)
#   ⚠ 무엇을 생략하든 '전량 통과'가 아니다 — 종료 2.
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
# [08-03 검토 §30.3 보완 — `hard_timeout` 8자리 전수 대조]
#   지적받은 곳은 `topic hz` **한 자리**였지만, 같은 클래스(외부 CLI 의 종료 상태를 묻는가)를
#   `grep -n "hard_timeout " tools/d0_check.sh` 로 전수 열거하고 **한 줄씩** 대조했다
#   (`AGENTS.md §3-10 ①`). 열거를 세는 데서 끝내지 않고 각 자리가 덮이는지 적어 둔다:
#
#     자리            | 종류     | 정상 rc  | 구판          | 지금
#     ----------------|----------|----------|---------------|--------------------------------
#     topic hz        | 관측 창  | 124/137  | rc 무시 ★결함 | rc + 벽시계 둘 다 (검토 §30.3)
#     echo --once(ⓒ)  | 스냅샷   | 0        | if 로 사용 ✅ | 유지 (보조 증거로 격하 명시)
#     topic info -v   | 스냅샷   | 0        | rc 무시       | rc 0 아니면 판독 실패
#     echo --field(부호)| 관측 창 | 124/137  | rc 무시       | 결론 방향별로 분기(아래 [6] 주석)
#     echo /firmware  | 스냅샷   | 0        | if 로 사용 ✅ | 유지
#     echo /estop ×2  | 스냅샷   | 0        | if 로 사용 ✅ | 유지
#     topic info(재확인)| 스냅샷 | 0        | 파이프 끝 grep| rc 0 확인 후 판독 (08-03 P2 보완)
#
#   마지막 자리도 원 명령 rc를 먼저 확인한다. rc 0이 아닌 잘린 스냅샷은 EKF 행이 보여도
#   유지 판정에 사용하지 않는다.
#
# ⚠ 이 스크립트는 tools/lib_e2e.sh 를 **일부러 source 하지 않는다.**
#   그쪽 `cleanup()` 은 nav2·slam·gzserver 를 **이름으로 전역 kill** 한다 — 전용 시뮬 PC
#   전용이고, 실차 Jetson 에서 돌면 살아 있는 스택을 통째로 죽인다. 여기서 필요한 것은
#   `hard_timeout` 하나뿐이라 아래에 4줄로 다시 정의한다 (중복이지만 안전 경계가 우선).
# ============================================================================
set -u

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

echo "=== D+0 연결 판정 ($(date '+%Y-%m-%d %H:%M:%S')) ==="
echo "    측정 시간: 주기 ${HZ_SECS}초 × 2회"
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
  local topic="$1" expect="$2" idx="$3" out="$TMP/hz$3.txt"
  local parsed rate gapms win need
  local hz_rc t0 t1 elapsed_ms need_ms
  echo "[$idx] $topic 발행 주기 (기대 약 ${expect}Hz)"
  t0=$(date +%s%N)
  hard_timeout "$HZ_SECS" ros2 topic hz "$topic" >"$out" 2>&1
  hz_rc=$?                                   # ★ 바로 다음 줄에서 받는다(뒤로 미루면 덮인다)
  t1=$(date +%s%N)
  elapsed_ms=$(( (t1 - t0) / 1000000 ))
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
  if [ "$elapsed_ms" -lt "$need_ms" ]; then
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
  #   상한 발동(124/137)은 '3초 안에 한 건도 안 왔다'는 뜻이라 실패다.
  if hard_timeout 3 ros2 topic echo "$topic" --once >/dev/null 2>&1; then
    ok "$topic 관측 창 종료 시점에도 수신됨"
  else
    ng "$topic 이 창 종료 시점에는 오지 않는다 — 측정 도중 끊겼다"
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
  hard_timeout 20 ros2 topic info "$topic" -v >"$out" 2>&1
  info_rc=$?
  # ★ 08-03 검토 §30.3 계열 — 여기는 **스냅샷**이라 rc 0 이 정상이다(위 window_completed 와 반대).
  #   상한이 발동했다면 daemon 이 매달린 것이고, 그때 남은 출력은 **잘린 목록**이다.
  #   잘린 목록으로 "RELIABLE 구독자 없음" 을 말하면 그건 안 본 것을 봤다고 하는 것이다.
  if [ "$info_rc" != "0" ]; then
    ng "$topic 의 topic info 가 rc=$info_rc 로 끝났다 (20초 상한 발동 = 124) — **판독 실패**"
    ng "  → 엔드포인트 목록이 잘렸을 수 있다. 잘린 목록으로 QoS 를 판정하지 않는다"
    head -3 "$out" | sed 's/^/       /'
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
    ng "     ros2 topic echo /firmware/info --once  로 정체를 먼저 확인한다"
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
  hard_timeout "$HZ_SECS" ros2 topic echo "$ODOM_TOPIC" \
      --field twist.twist.linear.x >"$TMP/sign.txt" 2>&1
  SIGN_RC=$?

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
      window_completed "$SIGN_RC" \
        || warn "  ⚠ 관측자는 rc=$SIGN_RC 로 일찍 끝났다 — 부호는 관측된 사실이라 유효하다" ;;
    REVERSED*)
      ng "부호가 **반대**다 — 앞으로 굴렸는데 linear.x 최소 $(echo "$SIGN" | cut -d' ' -f2) m/s"
      ng "  → 펌웨어의 좌우/전후 부호를 구동부와 지금 맞춘다. URDF 로 덮지 말 것" ;;
    READFAIL|STILL*)
      if ! window_completed "$SIGN_RC"; then
        ng "/odom 관측자가 rc=$SIGN_RC 로 **먼저 끝났다** — 판독 실패다"
        ng "  → '안 움직였다'가 아니라 **못 봤다**. 다시 돌린다(agent 연결부터 확인)"
        head -3 "$TMP/sign.txt" | sed 's/^/       /'
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
next_idx; echo "[$IDX] 펌웨어 정체 (/firmware/info · 5초 주기)"
FWOUT="$TMP/fw.txt"
if hard_timeout 12 ros2 topic echo /firmware/info --once >"$FWOUT" 2>&1 && [ -s "$FWOUT" ]; then
  sed 's/^/       /' "$FWOUT" | head -6
  if grep -q "wheel_radius=0.05698" "$FWOUT"; then
    ok "wheel_radius=0.05698 — 소스 v1.4 와 일치"
  else
    ng "wheel_radius 가 소스(0.05698)와 다르다 — **다른 펌웨어가 구워져 있다**"
  fi
  if grep -q "kp=30.000; ki=5.000" "$FWOUT"; then
    ok "제어 게인 Kp=30 · Ki=5 — 소스 v1.4 와 일치"
  else
    warn "제어 게인이 소스(Kp=30, Ki=5)와 다르다 — 시험 데이터의 전제가 달라진다"
  fi
else
  ng "/firmware/info 를 못 읽었다 (5초 주기 × 12초 대기했다)"
  ng "  → agent 는 붙었는데 노드가 없다면 **IMU 초기화 실패**를 먼저 의심한다"
  ng "     (소스: IMU 실패 시 errorLoop() → micro-ROS 노드 자체가 안 뜬다."
  ng "      Teensy LED 가 100ms 주기로 빠르게 깜빡이면 그 상태다)"
fi
echo

# ── [8] E-stop 배선 ─────────────────────────────────────────────────────────
# [08-02 신설] 최종 회신 PDF 8쪽에는 E-stop 이 한 번도 안 나왔지만 **소스에는 있다**
#   (ESTOP_PIN=21, INPUT_PULLUP, active-low, 차단 5지점).
# ★ 이 검사가 필요한 이유: 핀에 아무것도 안 물려 있으면 풀업이 HIGH 로 띄워서
#   **"안 눌림"으로 읽힌다.** 배선이 없어도 소프트웨어는 아무 이상을 보고하지 않는다.
#   그래서 '토픽이 온다'로는 아무것도 증명되지 않고, **눌러서 바뀌는지**만이 증거다.
# ⚠ 발행 주기 1Hz(DIAGNOSTIC_PERIOD_MS) — 합의서의 10Hz 가 아니다. 최대 1초 기다린다.
next_idx; echo "[$IDX] E-stop 배선 (/estop/state · 1Hz)"
if [ "$SKIP_ESTOP" = "1" ]; then
  skip "E-stop 배선 — 사람이 버튼을 눌러야 한다 (--no-estop)"
else
  ESOUT="$TMP/estop_idle.txt"
  if hard_timeout 8 ros2 topic echo /estop/state --field data --once >"$ESOUT" 2>&1 && [ -s "$ESOUT" ]; then
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
          if hard_timeout 8 ros2 topic echo /estop/state --field data --once >"$ESOUT2" 2>&1 \
             && [ -s "$ESOUT2" ]; then
            if grep -qi "true" "$ESOUT2"; then
              ok "누름 → true 전환 확인 — **배선 정상**"
            else
              ng "**눌렀다고 했는데** false 그대로다 — E-stop 이 실차에 배선돼 있지 않다"
              ng "  → 소프트웨어 정지 수단은 /cmd_vel 0 과 watchdog 뿐이다(둘 다 소프트웨어 경로)"
              ng "  → 배선 전까지 **R1 이상 지면 주행을 하지 않는다** (인수 후 별도 일정)"
            fi
          else
            ng "누른 뒤 /estop/state 를 못 읽었다 — **판독 실패**(배선 없음과 다른 사실이다)"
            ng "  → agent 연결이 끊겼는지 먼저 본다. 끊겼다면 배선 판정을 하지 않는다"
          fi ;;
      esac
    else
      ng "평상시부터 /estop/state = true — 버튼이 눌린 채이거나 배선이 반대다"
      ng "  → 이 상태에서는 모터가 전혀 돌지 않는다(차단 5지점 전부 발동)"
    fi
  else
    ng "/estop/state 를 못 읽었다 — 토픽이 없다면 굽힌 펌웨어가 v1.4 가 아니다"
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
  hard_timeout 20 ros2 topic info "$t" -v >"$RECHECK_OUT" 2>/dev/null
  RECHECK_RC=$?
  if [ "$RECHECK_RC" != "0" ]; then
    ng "$t 의 최종 topic info 가 rc=$RECHECK_RC 로 끝났다 — **판독 실패**"
    ng "  → EKF 행이 일부 출력됐어도 스냅샷이 완주하지 않았으므로 유지 판정하지 않는다"
    head -3 "$RECHECK_OUT" | sed 's/^/       /'
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
  echo "✅ D+0 판정 통과 — 다음은 docs/D1_FIRST_STEP.md (R3 rosbag)."
  echo "   ⚠ 단, 여기 통과는 **주기의 상한을 증명하지 않는다**(관측 창이 짧다)."
  echo "     최대 간격의 정본 판정은 R3 rosbag 의 간격 히스토그램이다."
  exit 0
fi
