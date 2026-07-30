#!/usr/bin/env bash
# ============================================================
# lib_e2e.sh — 4개 E2E 스크립트 공통 셸 함수 라이브러리 (구조 분리 3/3, 07-24 추출)
#
# 배경: mission_e2e·abort_e2e·regression_negative·regression_3goals 가 프로세스
#   cleanup·Nav2 readiness 대기·goal 전송·deadline 계산을 각자 복붙하고 있었다.
#   중복은 한쪽만 고쳐지는 드리프트의 씨앗이라, 그 공통부를 여기 한 곳으로 모은다.
#   (마스터플랜 §2-3 / §7-2 · Codex §14.5 P2 — readiness "최대 90초" 문구·deadline
#    함수 단일화.)
#
# ★ 순수 리팩터 규약: E2E 판정 기준(PASS 조건·허용 오차·타임아웃)은 한 글자도
#   바꾸지 않는다. 이 파일은 '어떻게 기다리고 어떻게 정리하는가'의 공통 절차만 담고,
#   '무엇을 PASS 로 볼지'는 각 스크립트가 그대로 판정한다.
#
# source 규약 (중요): 각 스크립트가 ROS setup.bash 를 source 하고 `set -u` 를 켠
#   '뒤'에 이 파일을 source 한다. (setup.bash 는 미정의 변수를 참조하므로 set -u
#   상태로 source 하면 즉사 — 그래서 순서가 source→set -u→이 파일이다.)
#   이 파일은 함수 정의와 trap 설치만 하고, LOGDIR 은 각 스크립트가 mktemp 로 잡는다
#   (스크립트마다 접두어가 달라 여기서 만들지 않는다). cleanup 은 LOGDIR 을 쓰지 않아
#   LOGDIR 설정 전에 trap 이 돌아도 안전하다.
#
# ⚠ 전용 시뮬 PC 전용 (S2-4, Codex §11.4): cleanup 이 전역 pkill 로 gzserver·nav2·
#   slam 등을 '프로세스 이름'으로 죽인다 — 다른 ROS 작업이 도는 PC/Jetson 에서 실행 금지.
# ============================================================

# --- 공통 프로세스 정리 ------------------------------------------------------
# 순서 불변조건(AGENTS.md §4): 부모 `ros2 launch` 를 먼저 kill → 그 다음 nav2 노드.
#   부모만 kill -9 하면 고아가 된 좀비 bt_navigator 가 다음 실행의 goal 을 가로챈다.
#   자기매칭 자살 방지 = 브래킷 트릭("ros2[ ]launch", "mission[_]node" …).
#
# cleanup 은 4개 스크립트의 '합집합'이다: mission_node·fake_follower 는 회귀 2종
#   (negative·3goals)에는 없던 대상이지만, 그 스크립트가 해당 프로세스를 안 띄우므로
#   pkill 은 무해한 no-op 이고, 오히려 직전 mission 실행이 남긴 좀비(고아 mission_node
#   가 제 goal 을 nav2 로 계속 쏘는 간섭)를 선제 정리해 더 안전하다. 어느 경우에도
#   PASS 판정에는 영향이 없다(정리 절차일 뿐 판정 기준이 아니다).
cleanup() {
  pgrep -f "ros2[ ]launch" | xargs -r kill -9 2>/dev/null   # ★ 부모 먼저
  pkill -9 -f "mission[_]node" 2>/dev/null
  pkill -9 -f "fake[_]follower" 2>/dev/null
  pkill -9 -x gzserver 2>/dev/null; pkill -9 -x gzclient 2>/dev/null
  pkill -9 -f "slam[_]toolbox" 2>/dev/null   # async(mapping)·localization 둘 다 매칭
  pkill -9 -f "robot_state[_]publisher" 2>/dev/null
  pkill -9 -f "lib/nav2[_]" 2>/dev/null   # ★ nav2 노드 전체 (부모 kill -9 의 고아 청소)
  pkill -9 -x ekf_node 2>/dev/null
  pkill -9 -f "spawn[_]entity" 2>/dev/null
  sleep 1
}

# --- 실패 종결 + 인터럽트 정리 (F4, Codex §12.6) -----------------------------
# fail 은 LOGDIR(각 스크립트 global)을 참조 — 호출 시점엔 이미 mktemp 로 설정돼 있다.
fail() { echo "== FAIL: $1 (로그: $LOGDIR)"; cleanup; exit 1; }
# ★ S2-4: Ctrl+C/kill 로 끊겨도 좀비(고아 nav2 등)를 안 남기게. cleanup 후 반드시 exit —
#   없으면 Ctrl+C 뒤에도 다음 단계가 계속 실행된다.
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM

# --- 경과시간(deadline) 계산 -------------------------------------------------
# ★ G4 (Codex §13.5): 상한은 sleep 누적이 아니라 '실경과시간'으로 잰다. timeout 대기가
#   sleep 에 안 잡히는 구멍을 막는다. deadline_start 로 기준시각을 찍고, 루프 안에서
#   deadline_exceeded N 으로 N 초 초과를 확인한다.
E2E_T0=$SECONDS            # set -u 안전용 초기화 (실사용은 항상 deadline_start 가 먼저 덮음)
deadline_start()    { E2E_T0=$SECONDS; }
deadline_exceeded() { [ $(( SECONDS - E2E_T0 )) -ge "$1" ]; }

# --- 공통 hard-timeout 헬퍼 (SIGTERM 무시도 상한 안에 강제 종결) ---------------
# ★ 07-24 §15 P1: GNU `timeout N` 은 기본 SIGTERM 만 보낸다 — 대상이 TERM 을 무시하거나
#   (trap '' TERM) 처리 못 하면 N 초에도 안 죽어 무한 행이 그대로 남는다(원 CLI/daemon flake
#   재현). `--kill-after=유예` 로 TERM 뒤 짧은 유예 후 SIGKILL(catch 불가)을 보장해 '진짜'
#   벽시계 상한을 만든다. 이 라이브러리와 mission_e2e 가 유한 상한을 요구하는 ros2 CLI 대기를
#   이 helper 하나로 단일화한다.
#   실제 상한 = 인자 dur + E2E_KILL_GRACE (TERM 무시 최악). 반환 rc 는 timeout 그대로
#   (124=TERM 만료, 137=SIGKILL). 정상 TERM 응답 시엔 dur 안에 끝나 유예가 발동 안 한다.
E2E_KILL_GRACE=2           # hard-timeout: SIGTERM 뒤 SIGKILL 까지 유예(초)
E2E_MIN_RECOVER=6          # wait_state daemon 복구 최소 잔여예산(초) — 미만이면 복구 생략·deadline FAIL
hard_timeout() {  # $1=상한(초) $2..=실행할 명령
  local dur=$1; shift
  timeout --kill-after="$E2E_KILL_GRACE" "$dur" "$@"
}

# --- 미션 상태 1개 읽기 (mission_e2e·abort_e2e 공용) -------------------------
# /mission_state 는 2Hz 발행이라 3초 timeout 이면 1개 읽기에 충분하다.
# ★ 07-24 §14 P1: timeout 을 인자로 받는다(기본 3). wait_state 가 '남은 예산'만큼만
#   읽어 벽시계 deadline 을 넘기지 않게 하려는 것 — 인자 없이 부르는 abort_e2e 는 기본 3 유지.
# ★ 07-24 §15 P1: hard_timeout 으로 TERM 무시 시에도 (인자+유예) 안에 강제 종결.
state() {  # $1=timeout 초 (기본 3)
  hard_timeout "${1:-3}" ros2 topic echo /mission_state --once 2>/dev/null \
    | sed -n 's/^data: //p' | head -1
}

# --- 파라미터 float 1개 읽기 (복구 시퀀스 전체에 유한 상한) --------------------
# ★ ⑦ (07-24 e2e-harness-fix): `ros2 param get` 은 CLI/daemon flake(§5 ③) 때
#   무한 행할 수 있다 — 무방비 호출이 쌍굴 3회차에서 13분 27초 매달렸다 (FREEZE_MANIFEST §8).
# ★ 07-24 §14 P1 보완: 복구용 `ros2 daemon stop/start` **자체도** 같은 무한 행 표면이었다
#   (원 결함이 daemon flake인데 복구 명령을 무방비로 부르면 도로아미타불).
# ★ 07-24 §15 P1 보완: 그 timeout 들이 SIGTERM 만 보내 CLI 가 TERM 을 무시하면 여전히 안 죽었다.
#   4개 호출 전부 hard_timeout(=TERM 뒤 유예 후 SIGKILL)으로 바꿔 '진짜' 상한을 만든다.
#   실제 hard 상한 = (8+g)+(5+g)+(5+g)+(8+g) = 26 + 4g = **34s**(g=E2E_KILL_GRACE=2, TERM
#   무시 최악). 정상 TERM 응답 시엔 ≈26s. ⚠ 여기서 보장하는 것은 '유한 시간에 읽거나 포기'뿐 —
#   읽은 값이 옳은지의 판정은 호출자 몫이다('못 읽음=§5 ③ 인프라'과 '값 틀림=코드 결함' 불혼동).
read_param_float() {  # $1=노드 $2=파라미터명 → float 문자열(예: 0.12) 또는 '' (읽기 실패)
  local out
  out=$(hard_timeout 8 ros2 param get "$1" "$2" 2>/dev/null | grep -oE '[0-9]+\.[0-9]+' | head -1)
  if [ -z "$out" ]; then
    hard_timeout 5 ros2 daemon stop  >/dev/null 2>&1   # ★ 복구 명령도 hard-kill 상한
    hard_timeout 5 ros2 daemon start >/dev/null 2>&1
    out=$(hard_timeout 8 ros2 param get "$1" "$2" 2>/dev/null | grep -oE '[0-9]+\.[0-9]+' | head -1)
  fi
  printf '%s' "$out"
}

# --- 미션 상태가 목표에 도달할 때까지 대기 (벽시계 deadline — mission_e2e 전용) ---
# ★ 07-24 §14 P1 보완: 예산 $2 를 sleep 누적이 아니라 '벽시계 실경과'(SECONDS-t0)로 잰다.
#   구판은 `t += sleep 3` 만 세고 state()가 timeout 을 소비한 시간을 빼먹어, state 가 매번
#   3초 걸리는 flake 에서 선언 3초 예산이 벽시계 9초까지 늘었다(재현됨). 게다가 빈 읽기 시
#   daemon 재시작이 무방비라 걸리면 무한대였다. 불변식:
#   ① 예산 안(el<$2)에 목표면 PASS, el≥$2 면 FAIL — 판정과 '마지막 상태' 보고는 **같은 s**.
#   ② FAIL 메시지에 벽시계 경과(el)를 함께 찍는다. el≥예산이 FAIL 근거라, s 가 목표여도
#      "예산 밖에서 늦게 도달"이 명시돼 옛 ⑧-a 자기모순(타임아웃인데 마지막=목표)이 재발 안 한다.
#   ③ 읽기·대기 timeout 을 매번 '남은 예산'으로 제한 → 예산을 크게 넘겨 반환하지 않는다.
#      잔여 오차 상한 = 마지막 read/sleep 한 주기(≈1s) 수준의 스케줄링 허용치.
#   ④ 내부 daemon 재시작(F7)도 §15 P1 보완: (a) 각 호출을 hard_timeout(TERM 무시도 SIGKILL),
#      (b) 상한을 고정 5 가 아니라 '남은 예산'에서 배분(각 5s 상한 + 유예까지 rem 안에 수렴),
#      (c) 남은 예산 < E2E_MIN_RECOVER 면 복구를 아예 시작하지 않고 다음 루프의 deadline FAIL
#      에 맡긴다. 이렇게 daemon 복구까지 전부 같은 rem 을 소비해 벽시계 상한이 N 을 안 넘는다.
#   ⚠ mission 전용(F7 daemon-kick 포함)이지만 격리 단위 테스트를 위해 라이브러리에 둔다.
wait_state() {  # $1=원하는 상태 $2=예산(벽시계 초)
  local t0=$SECONDS s empty=0 kicked=0 el rem d
  while :; do
    rem=$(( $2 - (SECONDS - t0) ))                          # 남은 예산으로 읽기 timeout 제한
    if [ "$rem" -gt 3 ]; then rem=3; elif [ "$rem" -lt 1 ]; then rem=1; fi
    s=$(state "$rem")
    el=$(( SECONDS - t0 ))
    if [ "$el" -lt "$2" ] && [ "$s" = "$1" ]; then          # 예산 안에서 목표 도달
      echo "  ✓ $1 도달 (${el}s)"; return 0
    fi
    if [ "$el" -ge "$2" ]; then                             # 예산 소진 — 판정=보고 동일 샘플
      fail "$1 대기 타임아웃(예산 ${2}s, 경과 ${el}s), 마지막 상태='$s'"
    fi
    if [ -z "$s" ]; then                                    # F7: 빈 읽기 5연속 시 daemon 재시작
      empty=$((empty+1))
      if [ "$empty" -ge 5 ] && [ "$kicked" = 0 ]; then
        rem=$(( $2 - (SECONDS - t0) ))                      # 복구 직전 실측 잔여예산
        if [ "$rem" -ge "$E2E_MIN_RECOVER" ]; then          # 예산 충분 → 남은 예산 안에서 복구
          d=$(( (rem - 2*E2E_KILL_GRACE) / 2 ))             # stop/start 각 상한(+유예 2회 = rem)
          if [ "$d" -gt 5 ]; then d=5; elif [ "$d" -lt 1 ]; then d=1; fi
          echo "  (⚠ /mission_state 빈 읽기 ${empty}연속 — 남은 ${rem}s 내 ros2 daemon 재시작 자가 복구)"
          hard_timeout "$d" ros2 daemon stop  >/dev/null 2>&1
          hard_timeout "$d" ros2 daemon start >/dev/null 2>&1
        else                                                # 예산 부족 → 복구 생략, deadline FAIL 에 맡김
          echo "  (⚠ 빈 읽기 ${empty}연속 — 남은 예산 ${rem}s < 복구 최소 ${E2E_MIN_RECOVER}s: 복구 생략, deadline FAIL)"
        fi
        kicked=1
      fi
    else
      empty=0
    fi
    rem=$(( $2 - (SECONDS - t0) ))                          # 다음 폴링 대기도 예산으로 제한
    if [ "$rem" -gt 3 ]; then rem=3; elif [ "$rem" -lt 1 ]; then rem=1; fi
    sleep "$rem"
  done
}

# --- Nav2 활성화 대기 (4개 스크립트 공통 — "최대 90초" 문구 단일 출처) --------
# 3단 관문: ① controller_server 파라미터 존재 → ② bt_navigator lifecycle active →
#   ③ navigate_to_pose action 서버 discovery. 셋은 '한 번' 찍은 기준시각(deadline_start)
#   을 공유해 누적 상한(90→120→150초)으로 판정한다. CLI 자체 hang 방지로 매 조회에
#   timeout 8 을 건다(F4, Codex §12.10). "inactive" 에 'active' 가 부분포함되므로 ^앵커 필수.
wait_nav2_ready() {
  echo "== ② Nav2 활성화 대기 (최대 90초)"
  deadline_start
  until hard_timeout 8 ros2 param get /controller_server FollowPath.desired_linear_vel 2>/dev/null | grep -q Double; do
    sleep 3; deadline_exceeded 90 && fail "Nav2 기동 타임아웃"
  done
  # F4 (Codex §12.10): parameter 존재 ≠ lifecycle active — bt_navigator 활성까지 확인
  until hard_timeout 8 ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q "^active"; do
    sleep 3; deadline_exceeded 120 && fail "bt_navigator 미활성 (lifecycle bringup 실패 의심 — launch 로그 확인)"
  done
  # ★ G4 (Codex §13.5): bt_navigator active ≠ action discovery 완료 —
  #   navigate_to_pose 서버가 실제 떠 있는지 별도 확인 (goal 전송 전 마지막 관문)
  until hard_timeout 8 ros2 action info /navigate_to_pose 2>/dev/null | grep -q "Action servers: [1-9]"; do
    sleep 2; deadline_exceeded 150 && fail "navigate_to_pose action server 미준비"
  done
  sleep 5   # 지도/TF 안정화
}

# --- 정상 goal 전송: SUCCEEDED 대기, 응답유실 대비 1회 재전송 ----------------
# regression_3goals(yaw 지정)·regression_negative(정상 대조군, yaw=0) 공용.
#   응답 유실 함정(0705 §15.7 ③): 오래 뜬 액션서버 + 새 CLI 는 goal 응답을 떨굴 수
#   있어(bt_navigator "Failed to send goal response") 1회 재전송으로 방어한다.
#   판정은 believed(TF)가 아니라 최종 상태문자열(SUCCEEDED)로만 한다 — 실위치 오차
#   판정은 각 스크립트가 ground truth(gz model)로 별도로 본다.
send_goal() {  # $1=x $2=y $3=yaw(rad) $4=제한시간(초)
  local qz qw out attempt
  qz=$(python3 -c "import math; print(math.sin($3/2))")
  qw=$(python3 -c "import math; print(math.cos($3/2))")
  for attempt in 1 2; do
    out=$(hard_timeout "$4" ros2 action send_goal /navigate_to_pose \
      nav2_msgs/action/NavigateToPose \
      "{pose: {header: {frame_id: map}, pose: {position: {x: $1, y: $2}, orientation: {z: $qz, w: $qw}}}}" 2>&1 | tail -1)
    if echo "$out" | grep -q SUCCEEDED; then return 0; fi
    echo "  (시도 $attempt 결과: $out — 재전송)"
  done
  return 1
}

# --- ground truth(gz) 좌표 조회 — 유한 상한 (07-30 실측 반영) -----------------
# ★ 07-30 실측: `gz model -m … -p` 는 상한이 없으면 **무한 행**한다. 이번 세션
#   mission_e2e ⑪ 에서 그대로 재현됐다 — ⑩ ESCAPED 까지 다 통과한 뒤 ⑪ 에서 약 11분
#   매달렸고, 외부에서 그 PID 를 죽여야만 풀렸다(고아 gz 프로세스는 20분 넘게 생존).
#   ros2 param get(§14)·topic pub(§16)과 **같은 실패 양식**인데 gz CLI 만 무방비였다.
# ⚠ 빈 출력 = '못 읽음(인프라)'. 이 함수는 그것을 0 이나 임의값으로 메우지 않는다 —
#   '못 읽음'과 '값이 틀림(코드 결함)'을 뒤섞지 않는 것이 이 저장소의 판정 규칙이다.
#   빈 출력을 어떻게 다룰지는 호출자가 명시적으로 분류한다.
# ★ 07-31 §7.3 P1 (Codex): 구판은 마지막 줄의 첫 두 토큰을 **검증 없이** 흘려보냈다.
#   그래서 `model -m`(비숫자)·`nan nan`·`inf inf` 가 "빈 문자열이 아니다"라는 이유로
#   인프라 분기를 우회해, ⑦ 에서 **실정지 실패로 오분류**되고 /cmd_vel 원인 분류까지
#   틀린 전제 위에서 돌았다 (구현자 재현 확정).
#   계약: **정확히 두 개의 유한 실수만** 통과시킨다. timeout·빈값·필드 부족·비숫자·
#   NaN/Inf 는 전부 같은 '좌표 없음'(빈 출력)으로 수렴한다 — 호출자는 그 하나만 소비한다.
#   ⚠ 음수·소수는 이 프로젝트의 **정상 world 좌표**다 (스폰 -12,0). 거부하면 안 된다.
gz_model_xy() {  # $1=모델명 → "x y"(유한 실수 2개) 또는 '' (조회 실패·형식 불량)
  hard_timeout 8 gz model -m "$1" -p 2>/dev/null | python3 -c '
import math, sys
last = ""
for line in sys.stdin:
    if line.strip():
        last = line
f = last.split()
if len(f) < 2:
    sys.exit(1)                      # 빈 출력·필드 부족 = 좌표 없음
try:
    x, y = float(f[0]), float(f[1])
except ValueError:
    sys.exit(1)                      # 비숫자 = 좌표 없음
if not (math.isfinite(x) and math.isfinite(y)):
    sys.exit(1)                      # NaN/Inf = 좌표 없음
print(x, y)
'
}

# --- /cmd_vel 잔류 판독 + 실정지 실패 원인 분류 (07-30 예약 4) -----------------
# ★ 왜 라이브러리로 올렸나: 구판은 abort_e2e ⑦(실정지 단언)이 깨지면 `fail()` 이 **즉시
#   cleanup + exit** 해서, 바로 다음 단계인 ⑧(/cmd_vel 수집)이 영영 실행되지 않았다.
#   노드까지 죽으니 /cmd_vel 은 이미 없다 — **정작 그 증거가 필요한 순간에 증거가 사라진다.**
#   07-24 동결 게이트에서 이 구분에 수동 규명이 필요했다(0723_현황.md §11.3).
#   그래서 ① 수집·판독·분류를 fail() 보다 '먼저' 부를 수 있는 함수로 분리하고,
#   ② 판정 로직을 Gazebo 없이 단위 테스트할 수 있게 여기(라이브러리)에 둔다.
#
# ⚠ 판정 기준은 그대로다: '0 이 아닌 속도 성분' 문턱 0.01 · 수집 2초 — ⑧ 이 쓰던 값 그대로.
#   이 절은 '무엇을 PASS 로 볼지'를 바꾸지 않고, 실패했을 때 '왜'를 덧붙일 뿐이다.

# 지정한 시간 동안 /cmd_vel 을 덤프한다 (수집 실패해도 호출자를 죽이지 않는다).
# ★ hard_timeout: 일반 timeout 은 TERM 무시 CLI 를 못 죽인다(07-24 §16 P1)— 여기도 같은 표면.
# ★ 07-31 §7.2 P1 (Codex): 구판은 `|| true` 로 수집 상태를 지웠다. 그런데 이 수집은
#   **시간상자**라 정상 종료가 곧 124(TERM)/137(KILL)이다 — rc 를 그대로 실패로 읽으면
#   정상까지 실패가 된다. 그래서 '정상 종료 집합'을 명시하고 그 밖만 실패로 남긴다.
#   ⚠ 판정의 주체는 어디까지나 아래 판독기다. 이 rc 는 호출자가 로그에 남길 **표식**이다.
collect_cmdvel() {  # $1=출력 파일 $2=수집 시간(초, 기본 2) → rc 0=수집 정상 / 1=수집 실패
  local rc
  hard_timeout "${2:-2}" ros2 topic echo /cmd_vel > "$1" 2>/dev/null
  rc=$?
  case "$rc" in 0|124|137) return 0 ;; *) return 1 ;; esac
}

# --- /cmd_vel 발행자 수 — '침묵'을 '관측된 침묵'으로 승격시키는 근거 -----------
# ★ 07-31 실측(가장 중요한 발견): 이 시스템에서 abort 뒤의 '잠잠'은 zero Twist 가 아니라
#   **완전한 침묵**이다 — 실제 덤프가 **0바이트**였다(nav2 가 취소 후 발행 자체를 멈춘다).
#   그래서 "빈 덤프 = 무조건 판독 실패"로 두면 **정상 경로가 영구 거짓 FAIL** 이 된다
#   (검토자가 같이 요구한 역회귀 '정상 abort_e2e 의 정지·잠잠 PASS 보존'과 충돌).
#   → 침묵을 인정하되 **공짜로는 안 준다**: 그 순간 /cmd_vel 에 발행자가 실제로 있었는지
#     확인해, '살아있는 토픽이 조용했다'와 '아무것도 안 듣고 있었다'를 가른다.
#   ⚠ ros2 topic 계열은 daemon 에 의존한다(§5 ③ flake 표면) → read_param_float 와 같은
#     복구 절차(daemon 재시작 1회)를 붙이고, 그래도 못 읽으면 **fail-closed**(빈 결과)다.
cmdvel_publisher_count() {  # → 발행자 수(정수) 또는 '' (조회 실패)
  local out
  out=$(hard_timeout 8 ros2 topic info /cmd_vel 2>/dev/null \
        | grep -oE 'Publisher count: [0-9]+' | grep -oE '[0-9]+' | head -1)
  if [ -z "$out" ]; then
    hard_timeout 5 ros2 daemon stop  >/dev/null 2>&1
    hard_timeout 5 ros2 daemon start >/dev/null 2>&1
    out=$(hard_timeout 8 ros2 topic info /cmd_vel 2>/dev/null \
          | grep -oE 'Publisher count: [0-9]+' | grep -oE '[0-9]+' | head -1)
  fi
  printf '%s' "$out"
}

# --- ⑦·⑧ 공용 단일 계약: 수집 → 관측 근거 확보 → 판독 --------------------------
# 한 곳에서만 조합한다 — ⑦ 와 ⑧ 이 각자 조합하면 한쪽만 고쳐지는 드리프트가 생긴다.
measure_cmdvel_residual() {  # $1=덤프 파일 $2=수집 시간(초, 기본 2) → 개수 또는 '' (판독 실패)
  local pub
  pub=$(cmdvel_publisher_count)          # 판정 시점의 그래프 생존 근거
  if ! collect_cmdvel "$1" "${2:-2}"; then printf ''; return; fi
  cmdvel_nonzero "$1" "$pub"
}

# 덤프에서 '0 이 아닌 속도 성분' 개수를 센다. linear/angular 의 x·y·z 전 성분 —
# 제자리 회전(angular.z)도 '움직임'이다.
#
# ★ 07-31 §8 P1 (Codex) — **줄 세기를 버리고 구조 파서로 다시 썼다.**
#   세 라운드 연속으로 이 함수 하나에서 P1 이 나왔다(§7.2 → §8.2). 그것은 "패치가
#   모자랐다"가 아니라 **처음부터 구조를 봤어야 했다**는 신호다. 구판이 뚫린 두 지점:
#   ① `seen_lines == 0`(정규식에 걸린 '값 줄' 수)을 '파일이 비었다'로 읽었다 → 경고문·
#      쓰레기 텍스트처럼 **내용은 있는데 값 줄이 0개**인 덤프가 '완전 침묵'으로 분류돼
#      발행자 근거를 얻어 `0건` 으로 승격됐다. 주석엔 "내용은 왔는데 온전한 표본 0개 =
#      판독 실패"라고 써 놓고 코드는 다르게 굴었다.
#   ② "6성분"을 **"6줄"** 로 셌다 → `angular.{x,y,y}`(z 누락 + y 중복)·`linear` 안의 6줄·
#      섹션 밖 6줄이 전부 '완전한 Twist' 로 통과했다.
#
#   계약(fail-closed) — 아래 넷을 **전부** 만족해야 정수를 반환한다:
#   (a) `linear` 와 `angular` **각각**에 `x·y·z` 가 **정확히 한 번씩** 있고 전부 유한한
#       표본이 최소 1개 있을 것 (개수가 아니라 **키 집합**으로 확인한다)
#   (b) 손상 신호가 하나도 없을 것 — 섹션 중복 · 섹션 밖 x/y/z · 키 중복 · 비숫자 · NaN/Inf
#   (c) 불완전 레코드는 **마지막 하나만** 허용 — 시간상자 수집이라 잘림은 원리상 꼬리에만
#       생긴다. 중간 레코드가 불완전하면 그것은 잘림이 아니라 손상이다
#   (d) '관측된 침묵'($2 = 발행자 수 ≥ 1 → 0건) 예외는 **파일이 정말로 빈 경우에만**
#       (`txt.strip() == ""`). 내용이 있으면 발행자가 살아 있어도 구조 검증을 통과해야 한다
#   ★ (d)가 07-31 실측의 핵심이다: abort 뒤 nav2 가 발행을 멈춰 **실덤프가 0바이트**다.
#     빈 덤프를 무조건 실패로 두면 abort_e2e 가 영구 거짓 FAIL 이 된다.
cmdvel_nonzero() {  # $1=덤프 파일 $2=발행자 수(선택) → 개수 또는 '' (판독 실패)
  python3 -c '
import math, re, sys
try:
    txt = open(sys.argv[1]).read()
except OSError:
    sys.exit(1)                          # 파일 없음 = 판독 실패
pub = sys.argv[2] if len(sys.argv) > 2 else ""

# (d) 침묵 예외는 **진짜 빈 파일**에만. 내용이 있으면 아래 구조 검증을 반드시 통과해야 한다.
if not txt.strip():
    if pub.isdigit() and int(pub) >= 1:
        print(0); sys.exit(0)            # 살아있는 토픽이 조용했다 = 관측된 침묵
    sys.exit(1)                          # 근거 없는 빈 덤프 = 판독 실패

SECS = ("linear", "angular")
recs = [r for r in txt.split("---") if r.strip()]
complete = nonzero = 0
for i, rec in enumerate(recs):
    sec, cur = {}, None
    for line in rec.splitlines():
        m = re.match(r"^(linear|angular):\s*$", line)
        if m:
            cur = m.group(1)
            if cur in sec:
                sys.exit(1)              # (b) 섹션 중복 = 손상
            sec[cur] = {}
            continue
        m = re.match(r"^\s+([xyz]):\s*(\S+)\s*$", line)
        if not m:
            continue                     # 그 밖의 줄은 무시
        if cur is None:
            sys.exit(1)                  # (b) 섹션 밖 x/y/z = 손상
        k, v = m.group(1), m.group(2)
        if k in sec[cur]:
            sys.exit(1)                  # (b) 키 중복 = 손상
        try:
            f = float(v)
        except ValueError:
            sys.exit(1)                  # (b) 비숫자 = 손상
        if not math.isfinite(f):
            sys.exit(1)                  # (b) NaN/Inf = 손상
        sec[cur][k] = abs(f)
    if all(set(sec.get(s, {})) == {"x", "y", "z"} for s in SECS):
        complete += 1                    # (a) 키 집합으로 확인 — 줄 개수가 아니다
        nonzero += sum(1 for s in SECS for v in sec[s].values() if v > 0.01)
    elif i != len(recs) - 1:
        sys.exit(1)                      # (c) 중간 레코드 불완전 = 잘림이 아니라 손상
if complete == 0:
    sys.exit(1)                          # 온전한 표본 0개 = 판독 실패
print(nonzero)
' "$1" "${2-}" 2>/dev/null
}

# 실정지 단언(⑦)이 깨졌을 때 잔류 개수로 원인을 가른다.
# ★ 이 두 갈래는 '고칠 곳'이 서로 다르다 — 그래서 자동 분류에 값어치가 있다:
#   있음 = 취소 경로가 새 명령을 계속 내보낸다 → mission/nav2 코드 결함.
#   없음 = 명령은 끊겼는데 로봇이 움직인다 → 잔류 명령. libgazebo_ros_diff_drive 는
#          command timeout 이 없어 마지막 속도를 무한 유지한다(MASTER_PLAN §7 예약 5) —
#          그래서 이 분기는 실제로 일어난다. 실차에선 cmd_vel watchdog 이 덮을 몫.
classify_stop_failure() {  # $1=cmdvel_nonzero 결과 → 사람이 읽는 원인 분류 한 줄
  case "$1" in
    ''|*[!0-9]*)
      printf '%s' "분류 불가 — /cmd_vel 판독 실패(수집 실패·빈 덤프·경고문뿐·필드 누락·비숫자·NaN/Inf 중 하나). 정지/미정지 어느 쪽도 주장하지 않는다. 로그의 cmdvel 덤프 확인" ;;
    0)
      printf '%s' "잔류 명령/시뮬 특성 — 새 속도 명령은 끊겼는데(잔류 0건) 로봇이 계속 움직임. diff_drive 가 마지막 속도를 유지(예약 5), 실차는 cmd_vel watchdog 소관" ;;
    *)
      printf '%s' "코드 결함(취소 경로) — 취소 접수 후에도 새 속도 명령이 ${1}건 계속 나감" ;;
  esac
}
