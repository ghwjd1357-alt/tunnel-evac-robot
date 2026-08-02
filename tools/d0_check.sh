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
# 사용:
#   bash tools/d0_check.sh              # 전량 검사 (검사 5·7 은 사람이 손을 써야 한다)
#   bash tools/d0_check.sh --no-sign    # 검사 5·7 생략 (⚠ 그러면 '전량 통과'가 아니다 — 종료 2)
#
# 검사 목록: [1] 시리얼 [2][3] 발행주기 [4][5] QoS  — 여기까지 자동
#            [5] 전진 부호(바퀴를 굴린다) [6] 펌웨어 정체 [7] E-stop 배선(버튼을 누른다)
#   ★ [6][7] 은 **펌웨어 소스를 받은 08-02 에 신설**됐다. 소스가 없으면 쓸 수 없는 검사다.
#   bash tools/d0_check.sh --secs 15    # 주기 측정 시간(기본 8초)
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

SKIP_SIGN=0
while [ $# -gt 0 ]; do
  case "$1" in
    --no-sign) SKIP_SIGN=1; shift ;;
    --secs)    HZ_SECS="${2:-}"; shift 2 ;;
    -h|--help) sed -n '2,45p' "$0"; exit 3 ;;
    *) echo "알 수 없는 인자: $1  (사용법은 --help)"; exit 3 ;;
  esac
done
case "$HZ_SECS" in ''|*[!0-9]*) echo "--secs 는 정수여야 한다: $HZ_SECS"; exit 3 ;; esac
[ "$HZ_SECS" -lt 3 ] && { echo "--secs 는 3 이상이어야 한다(표본이 너무 적다)"; exit 3; }

FAIL=0
SKIPPED=0
TMP=$(mktemp -d -t d0check.XXXXXX) || { echo "임시 디렉터리 생성 실패"; exit 1; }
trap 'rm -rf "$TMP"' EXIT

ok()   { printf '  \033[32mOK\033[0m   %s\n' "$1"; }
ng()   { printf '  \033[31mFAIL\033[0m %s\n' "$1"; FAIL=1; }
warn() { printf '  \033[33m⚠\033[0m    %s\n' "$1"; }
skip() { printf '  \033[33m--\033[0m   %s (생략)\n' "$1"; SKIPPED=$((SKIPPED+1)); }

echo "=== D+0 연결 판정 ($(date '+%Y-%m-%d %H:%M:%S')) ==="
echo "    측정 시간: 주기 ${HZ_SECS}초 × 2회"
echo

# ── [1] 시리얼 장치 ─────────────────────────────────────────────────────────
# 여기서 막히면 뒤의 검사는 전부 "안 온다"로만 나와 원인이 안 보인다. 먼저 가른다.
echo "[1] Teensy 시리얼 장치"
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
check_hz() {  # $1=토픽 $2=기대 주기(참고용) $3=검사 번호
  local topic="$1" expect="$2" idx="$3" out="$TMP/hz$3.txt" parsed rate gapms
  echo "[$idx] $topic 발행 주기 (기대 약 ${expect}Hz)"
  hard_timeout "$HZ_SECS" ros2 topic hz "$topic" >"$out" 2>&1

  # 파서는 fail-closed: 'average rate' 를 한 번도 못 봤으면 통과시키지 않는다.
  #   빈 출력 · 경고문만 · 형식 변경이 전부 여기서 걸린다.
  parsed=$(awk '
      /average rate:/ { rate=$3 }
      /max:/ { for (i = 1; i <= NF; i++) if ($i == "max:") { m=$(i+1) } }
      END {
        if (rate == "" || m == "") { exit 3 }
        sub(/s$/, "", m)
        printf "%s %.2f", rate, m * 1000
      }' "$out")
  if [ -z "$parsed" ]; then
    ng "$topic 주기를 판독하지 못했다 — '측정값 0' 이 아니라 **판독 실패**다"
    ng "  → 원문 ${HZ_SECS}초 출력의 앞 3줄:"
    head -3 "$out" | sed 's/^/       /'
    ng "  → 아무것도 안 찍혔다면 발행이 정말 없는 것이다(QoS 탓이 아니다 — 위 전제 ⓐ)"
    echo; return
  fi
  rate=${parsed% *}
  gapms=${parsed#* }

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
check_hz "$ODOM_TOPIC" "$HZ_EXPECT_ODOM" 2
check_hz "$IMU_TOPIC"  "$HZ_EXPECT_IMU"  3

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
  local rows npub nsub off_pub be_pub bad_pair
  echo "[$idx] $topic QoS 정합"
  hard_timeout 20 ros2 topic info "$topic" -v >"$out" 2>&1

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
check_qos "$ODOM_TOPIC" 4 RELIABLE
check_qos "$IMU_TOPIC"  5 BEST_EFFORT

# ── [5] 전진 부호 ───────────────────────────────────────────────────────────
# ★ 이 스크립트는 **로봇에 명령을 보내지 않는다.** 사람이 바퀴를 손으로 굴린다.
#   이유: D+0 의 로봇 상태는 R0(바퀴 공중)이고, 검증 안 된 스택이 모터를 돌리게 하는 것은
#   순서가 뒤집힌 것이다. 부호는 명령 없이도 확인된다 — 굴리면 /odom 이 반응한다.
# 왜 부호를 보는가: 반대면 EKF·SLAM·Nav2 가 전부 **거울처럼 뒤집힌 세계**에서 동작한다.
#   증상은 "지도가 이상하다"로만 나타나서 원인 찾기가 오래 걸린다.
echo "[6] 전진 시 twist.twist.linear.x 부호"
if [ "$SKIP_SIGN" = "1" ]; then
  skip "전진 부호 — --no-sign"
else
  echo "  ★ 지금 할 일: **바퀴를 손으로 '앞으로' 굴려 주세요** (바퀴가 공중에 뜬 상태 R0)."
  echo "     준비되면 Enter — 그때부터 ${HZ_SECS}초 동안 /odom 을 봅니다."
  read -r _ || true
  hard_timeout "$HZ_SECS" ros2 topic echo "$ODOM_TOPIC" \
      --field twist.twist.linear.x >"$TMP/sign.txt" 2>&1

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
  case "$SIGN" in
    READFAIL)
      ng "/odom 값을 한 줄도 못 읽었다 — **판독 실패**(값이 0 이었다는 뜻이 아니다)"
      head -3 "$TMP/sign.txt" | sed 's/^/       /' ;;
    OK*)
      ok "앞으로 굴릴 때 linear.x 최대 $(echo "$SIGN" | cut -d' ' -f2) m/s > 0 (부호 정상)" ;;
    REVERSED*)
      ng "부호가 **반대**다 — 앞으로 굴렸는데 linear.x 최소 $(echo "$SIGN" | cut -d' ' -f2) m/s"
      ng "  → 펌웨어의 좌우/전후 부호를 구동부와 지금 맞춘다. URDF 로 덮지 말 것" ;;
    STILL*)
      ng "움직임이 관측되지 않았다 (최댓값 $(echo "$SIGN" | cut -d' ' -f2) m/s)"
      ng "  → 바퀴를 굴리는 동안 측정됐는지, 엔코더가 붙어 있는지 확인한다" ;;
    *)
      ng "부호 판정 자체가 실패했다: $SIGN" ;;
  esac
fi
echo

# ── [6] 펌웨어 정체 ─────────────────────────────────────────────────────────
# [08-02 신설] 펌웨어 소스를 받고 나서야 가능해진 검사다.
# `/firmware/info` 는 버전·게인·바퀴 반지름·라이브러리 목록을 한 줄로 방송한다.
# ⚠ 이 값으로 **버전을 판별할 수는 없다** — 소스 v1.4 인데 FW_VERSION 은 "1.3.0",
#   FW_SOURCE_PATH 는 v1_3, FW_GIT_SHA 는 0 으로 채워져 있다(소스 36~39행).
#   그래도 **바퀴 반지름·게인·baud 가 우리가 읽은 소스와 같은지**는 여기서 갈린다.
# ⚠ 발행 주기가 5초(FW_INFO_PERIOD_MS)라 --once 는 최대 5초를 기다린다. VOLATILE 이라
#   지나간 것은 못 받는다 — 타임아웃을 넉넉히 준다.
echo "[6] 펌웨어 정체 (/firmware/info · 5초 주기)"
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

# ── [7] E-stop 배선 ─────────────────────────────────────────────────────────
# [08-02 신설] 최종 회신 PDF 8쪽에는 E-stop 이 한 번도 안 나왔지만 **소스에는 있다**
#   (ESTOP_PIN=21, INPUT_PULLUP, active-low, 차단 5지점).
# ★ 이 검사가 필요한 이유: 핀에 아무것도 안 물려 있으면 풀업이 HIGH 로 띄워서
#   **"안 눌림"으로 읽힌다.** 배선이 없어도 소프트웨어는 아무 이상을 보고하지 않는다.
#   그래서 '토픽이 온다'로는 아무것도 증명되지 않고, **눌러서 바뀌는지**만이 증거다.
# ⚠ 발행 주기 1Hz(DIAGNOSTIC_PERIOD_MS) — 합의서의 10Hz 가 아니다. 최대 1초 기다린다.
echo "[7] E-stop 배선 (/estop/state · 1Hz)"
if [ "$SKIP_SIGN" = "1" ]; then
  skip "E-stop 확인 — 사람이 버튼을 눌러야 한다"
else
  ESOUT="$TMP/estop_idle.txt"
  if hard_timeout 8 ros2 topic echo /estop/state --field data --once >"$ESOUT" 2>&1 && [ -s "$ESOUT" ]; then
    if grep -qi "false" "$ESOUT"; then
      ok "평상시 /estop/state = false"
      echo "     ▶ 이제 **E-stop 버튼을 누른 채로** Enter 를 누르세요 (누르지 않았으면 그냥 Enter)"
      read -r _ || true
      ESOUT2="$TMP/estop_press.txt"
      if hard_timeout 8 ros2 topic echo /estop/state --field data --once >"$ESOUT2" 2>&1 \
         && grep -qi "true" "$ESOUT2"; then
        ok "누름 → true 전환 확인 — **배선 정상**"
      else
        ng "눌러도 false 그대로다 — **E-stop 이 실차에 배선돼 있지 않다**"
        ng "  → 소프트웨어 정지 수단은 /cmd_vel 0 과 watchdog 뿐이다(둘 다 소프트웨어 경로)"
        ng "  → 배선 전까지 **R1 이상 지면 주행을 하지 않는다** (인수 후 별도 일정)"
      fi
    else
      ng "평상시부터 /estop/state = true — 버튼이 눌린 채이거나 배선이 반대다"
      ng "  → 이 상태에서는 모터가 전혀 돌지 않는다(차단 5지점 전부 발동)"
    fi
  else
    ng "/estop/state 를 못 읽었다 — 토픽이 없다면 굽힌 펌웨어가 v1.4 가 아니다"
  fi
fi
echo

# ── 종합 ────────────────────────────────────────────────────────────────────
echo "=========================================="
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
