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
  # ★ 07-31 §9 P1: YAML 텍스트를 다시 구조 파싱하지 않는다. 메시지 타입을 Twist로 고정한 뒤
  #   ros2가 역직렬화한 6성분을 **한 메시지=CSV 한 줄**로 받는다. PYTHONUNBUFFERED는 timeout이
  #   여러 줄짜리 YAML 중간을 자르던 정상 꼬리 형상을 없애고 완전한 CSV 줄 단위로 남긴다.
  hard_timeout "${2:-2}" env PYTHONUNBUFFERED=1 \
    ros2 topic echo /cmd_vel geometry_msgs/msg/Twist --csv --no-lost-messages \
    > "$1" 2>/dev/null
  rc=$?
  case "$rc" in 0|124|137) return 0 ;; *) return 1 ;; esac
}

# --- /cmd_vel 발행자 **엔드포인트 GID 집합** — 침묵을 '관측된 침묵'으로 올리는 근거 ------
# ★ 07-31 실측(가장 중요한 발견): 이 시스템에서 abort 뒤의 '잠잠'은 zero Twist 가 아니라
#   **완전한 침묵**이다 — 실제 덤프가 **0바이트**였다(nav2 가 취소 후 발행 자체를 멈춘다).
#   그래서 "빈 덤프 = 무조건 판독 실패"로 두면 **정상 경로가 영구 거짓 FAIL** 이 된다
#   (검토자가 같이 요구한 역회귀 '정상 abort_e2e 의 정지·잠잠 PASS 보존'과 충돌).
#   → 침묵을 인정하되 **공짜로는 안 준다**: 그 순간 /cmd_vel 에 발행자가 실제로 있었는지
#     확인해, '살아있는 토픽이 조용했다'와 '아무것도 안 듣고 있었다'를 가른다.
#
# ★ 07-31 §11.3 P1-② (검토) — **발행자 '수'로는 관측 창을 묶지 못한다.**
#   구판은 수집이 끝난 뒤 발행자 수를 셌다. 그런데 수집 창에는 발행자가 없고 창 **직후에만**
#   생긴 경우(Nav2 재기동·발행자 세대 전환·DDS discovery 지연)에도 그 수를 근거로 빈 덤프를
#   `0건`으로 승인했다 — 들을 대상이 아예 없었는데 '정상 침묵'이 되는 거짓 PASS 다.
#   독립 재현(검토자·구현자 모두): `수집 중 발행자 0 · 수집 직후 1 → 결과 0`.
#   → 수 대신 **GID**(엔드포인트 인스턴스마다 유일한 DDS GUID)를 본다. 같은 GID 가 창
#     양끝에 있으면 그 사이 죽었다 다시 태어난 것이 아니다 — 재생성은 새 GID 를 받는다.
#     '수 1 → 수 1' 은 세대 전환을 통과시키지만 'GID A → GID B' 는 걸린다.
#   ⚠ ros2 topic 계열은 daemon 에 의존한다(§5 ③ flake 표면) → read_param_float 와 같은
#     복구 절차(daemon 재시작 1회)를 붙이고, 그래도 못 읽으면 **fail-closed**(빈 결과)다.
_cmdvel_gids_from() {  # $1=`ros2 topic info -v` 원문 → GID 줄들 / 'NONE'(발행자 0) / ''(실패)
  printf '%s\n' "$1" | python3 -c '
import sys
lines = sys.stdin.read().splitlines()
# 최상단 Type 이 Twist 라는 근거가 없으면 조회 실패로 본다(fail-closed).
if not any(l.strip() == "Type: geometry_msgs/msg/Twist" for l in lines):
    sys.exit(1)
gids, cur = [], None
for l in lines:
    s = l.strip()
    if s.startswith("Node name:"):       # 엔드포인트 블록 시작
        cur = {}
        continue
    if cur is None:
        continue
    for key in ("Topic type:", "Endpoint type:", "GID:"):
        if s.startswith(key):
            cur[key] = s[len(key):].strip()
    if cur.get("Endpoint type:") == "PUBLISHER" and "GID:" in cur:
        if cur.get("Topic type:") != "geometry_msgs/msg/Twist":
            sys.exit(1)                  # 같은 토픽에 다른 타입 발행자 = 근거로 쓰지 않는다
        gids.append(cur["GID:"])
        cur = None
print("\n".join(sorted(gids)) if gids else "NONE")
' 2>/dev/null
}

cmdvel_pub_gids() {  # $1=상한(초, 기본 8) $2=daemon 복구 재시도(1/0, 기본 1)
                     # → GID 한 줄씩 / 'NONE'(조회 성공·발행자 0) / '' (조회 실패)
  local budget="${1:-8}" retry="${2:-1}" raw out
  raw=$(hard_timeout "$budget" ros2 topic info -v /cmd_vel 2>/dev/null)
  out=$(_cmdvel_gids_from "$raw")
  # ⚠ 재시도 허용 기준은 '창 밖이냐'가 아니라 **'늦어져도 근거 자격을 잃지 않느냐'** 다.
  #   복구는 stop+start+재조회로 최대 ≈26s 를 쓴다. 그동안 그래프가 바뀔 수 있으므로,
  #   '이 시점의 상태'를 물어야 하는 조회(= 창 시작 스냅샷)에는 붙이면 안 된다 — 늦게
  #   성공한 값은 물어본 시점의 답이 아니다. 창 **끝** 조회는 늦어도 판정을 보수적으로만
  #   만들므로(measure_cmdvel_residual 의 비대칭 참조) 복구를 허용한다.
  if [ -z "$out" ] && [ "$retry" = 1 ]; then
    hard_timeout 5 ros2 daemon stop  >/dev/null 2>&1
    hard_timeout 5 ros2 daemon start >/dev/null 2>&1
    raw=$(hard_timeout "$budget" ros2 topic info -v /cmd_vel 2>/dev/null)
    out=$(_cmdvel_gids_from "$raw")
  fi
  printf '%s' "$out"
}

# 창 양끝 근거가 **같은 발행 세대**인지 판정한다.
# 승인 조건: 창 시작에 발행자가 있었고(비어있지 않음), 그 GID 가 **전부** 창 끝에도 살아 있다.
#   - 창 시작 0 → 들을 대상이 없었다        → 근거 없음
#   - 창 끝 0   → 창 도중 사라졌다          → 근거 없음
#   - A → B     → 세대 전환(죽고 새로 생김) → 근거 없음
#   - 창 끝에 **새로 생긴** 발행자는 근거로 재사용하지 않는다(부분집합만 본다).
_cmdvel_gid_survivors() {  # $1=창 시작 근거 $2=창 끝 근거 → 생존 발행자 수 또는 '' (근거 없음)
  python3 -c '
import sys
pre, post = sys.argv[1], sys.argv[2]
if not pre or not post or pre == "NONE" or post == "NONE":
    sys.exit(1)
a, b = set(pre.split()), set(post.split())
if not a or not a <= b:
    sys.exit(1)
print(len(a))
' "$1" "$2" 2>/dev/null
}

# --- ⑦·⑧ 공용 단일 계약: 수집 → 관측 근거 확보 → 판독 --------------------------
# 한 곳에서만 조합한다 — ⑦ 와 ⑧ 이 각자 조합하면 한쪽만 고쳐지는 드리프트가 생긴다.
# ★ 07-31 §10.5 P2-① (검토): 구판은 발행자·타입 조회를 **수집보다 먼저** 불렀다. 그 조회는
#   daemon 복구를 포함해 최악 ≈26~34s 다 — ⑦ 실정지 단언이 깨진 직후는 **잔류 명령이 아직
#   흐르고 있을 수 있는 유일한 창**인데, 거기서 30초를 먼저 써 버리면 그 사이 잔류가 멎어
#   '코드 결함(취소 경로)' 이 '잔류 명령/시뮬 특성' 으로 오분류될 수 있었다.
#   → **증거를 먼저 확보하고 근거는 뒤에 붙인다.** 근거(발행자 생존)는 덤프가 비었을 때만
#     필요하므로 그때만 조회한다 — 내용이 있으면 구조만으로 판정되기 때문이다.
#
# ★ 07-31 §11.3 P1-② (검토) — 구판의 '후-조회가 더 강하다'는 논증은 **틀렸다.**
#   선-조회는 '창 도중 죽은 발행자'를 근거로 잘못 내주고, 후-조회는 '창 직후에야 생긴
#   발행자'를 근거로 잘못 내준다. 어느 한쪽 시점만으로는 창을 묶을 수 없다 — 방향만 다른
#   같은 크기의 구멍이었다. 재현으로 확인했다(§11.3 · 구현자 독립 재현 동일).
#   → **창 양끝을 브래킷한다.**
#
# ★ 07-31 §14.3 P1 (검토) — '수집과 **동시에** 띄운' 조회는 창 시작을 고정하지 못했다.
#   구판은 `cmdvel_pub_gids "$dur" 0 &` 로 조회를 수집과 나란히 띄웠다. 그런데 그건 두 작업이
#   같이 **돈다**는 것뿐이고, 그 조회가 그래프를 실제로 **읽은 시점**이 창 시작보다 앞이라는
#   보장은 어디에도 없다. 상한(`dur`)은 늦은 조회를 유한하게 만들 뿐 시작 스냅샷으로
#   만들어주지 않는다. 그래서 조회가 늦게 읽는 동안 A 가 사라지고 B 가 생기면 pre 도 B,
#   post 도 B 가 되어 **빈 덤프가 `0` 으로 승인**된다 — §11.3 이 막으려던 A→B 거짓 PASS 가
#   그대로 재개방된다. 구현자 재현(시간축은 실제 벽시계, GID 만 스텁):
#     같은 DDS 진실(A 가 창 도중 소멸·B 생성) · 조회 지연 0.1s → `''`(정상)
#                                              · 조회 지연 1.2s → `0` (거짓 승인)
#   판정이 **DDS 그래프가 아니라 조회 속도**를 따라가고 있었다. 빠른 환경에서만 닫힌 구멍이다.
#
#   → 필요한 건 '조회를 띄웠다'가 아니라 **시작 스냅샷이 확정된 시점**이다. 창을 그 뒤에 연다.
#
#   ⚠ 여기서 pre 와 post 는 **대칭이 아니다** — 이 비대칭이 이 함수의 핵심이다:
#     · pre 가 늦으면 **치명적**이다. 늦게 읽은 pre 는 이미 새 세대(B)를 보므로, 창이 옛
#       세대(A)로 시작했다는 사실 자체가 증거에서 사라진다 → 거짓 승인.
#     · post 가 늦는 건 **안전하다**. GID 는 엔드포인트 인스턴스마다 유일해서 재생성되면 새
#       값을 받는다. 창 시작에 있던 A 가 창 끝보다 더 뒤에도 여전히 보인다면, 그 A 는 그
#       사이 죽었다 태어난 것이 아니라 **계속 살아 있던 같은 인스턴스**다. 늦은 post 는
#       판정을 보수적으로 만들 뿐(사이에 죽었으면 A 가 안 보여 fail-closed) 느슨하게 못 만든다.
#     그래서 pre 에만 '창보다 먼저 확정' 이라는 순서 제약을 건다.
#
#   P2-① 의 목적(잔류 수집을 지연시키지 않는다)은 그대로 지킨다:
#     ① **잔류 수집을 맨 먼저** 한다 — 조회를 한 번도 하지 않고 즉시 연다.
#     ② 덤프에 내용이 있으면 근거가 필요 없으므로 조회 없이 즉시 판정한다(케이스 18).
#     ③ 비었을 때만 시작 스냅샷을 **확정**하고, 그 **뒤에** 침묵 검증 창을 새로 연다.
#        빈 덤프를 승인하는 창은 이 검증 창이지 ① 의 수집 창이 아니다.
#     ④ 창 끝 스냅샷을 조회해 시작 GID 가 **전부** 살아남았는지 본다.
#   대가: 침묵 경로에서만 창 하나(기본 2s)와 조회 한 번이 더 든다. 잔류 경로는 그대로다.
measure_cmdvel_residual() {  # $1=덤프 파일 $2=수집 시간(초, 기본 2) → 개수 또는 '' (판독 실패)
  local dump="$1" dur="${2:-2}" pre post
  # ① 잔류 수집 — 지연 없이 맨 먼저. 여기가 잔류 명령이 흐르는 유일한 창이다(§10.5 P2-①).
  collect_cmdvel "$dump" "$dur" || { printf ''; return; }
  # 파서와 **같은 기준**으로 비었는지 본다(공백만 있는 파일도 '빔'). 판정 의미는 불변.
  if [ -n "$(tr -d '[:space:]' < "$dump" 2>/dev/null)" ]; then
    cmdvel_nonzero "$dump"               # ② 내용이 있으면 근거 없이 구조로만·지연 없이 판정
    return
  fi
  # ③ 이 창은 침묵 승인에 쓰지 않는다 — 시작 스냅샷이 확정된 적이 없기 때문이다.
  #    스냅샷을 **먼저 확정**한다. 비었거나(NONE) 못 읽으면 검증 창을 열 이유도 없다.
  #
  #    ⚠ retry=0 이다 — 시작 스냅샷에 daemon 복구를 허용하면 안 된다. 구현자 실측(실제 DDS):
  #      발행자가 0 이면 `topic info` 에 Type 줄이 없어 조회가 `''`(실패)로 관측된다. 여기서
  #      복구를 돌리면 stop+start+재조회로 **최대 26s** 가 흐르고, 그 사이에 새 발행자가
  #      생기면 그것이 '시작 스냅샷'이 된다. 논리는 닫히지만(창을 그 뒤에 여니까) 승인 창이
  #      잔류 수집 창에서 30초나 떨어져 나간다 — abort **직후**의 침묵이 아니라 30초 뒤의
  #      침묵을 승인하는 것이라 ⑦·⑧ 이 물으려던 질문에 더 이상 답하지 못한다.
  #      실측: retry=1 로 두자 '발행자 없음 → 창 도중 생성' 이 `0` 으로 승인됐다(§14.3 부정회귀②).
  #      → pre 는 **지금 이 순간**을 짧게 묻고, 못 읽으면 fail-closed 한다.
  #      post 는 retry=1 을 유지한다 — 늦게 끝나도 판정을 보수적으로만 만든다(위 비대칭).
  pre=$(cmdvel_pub_gids 8 0)
  case "$pre" in '' | NONE) printf ''; return ;; esac
  # ④ 확정된 스냅샷 **뒤에** 침묵 검증 창을 연다. 이 창의 빈 덤프만 승인 대상이다.
  collect_cmdvel "$dump" "$dur" || { printf ''; return; }
  post=$(cmdvel_pub_gids 8 1)            # 창 끝 근거(늦게 끝나도 안전 — 위 비대칭 참조)
  cmdvel_nonzero "$dump" "$(_cmdvel_gid_survivors "$pre" "$post")"
}

# 덤프에서 '0 이 아닌 속도 성분' 개수를 센다. CSV 열 순서는 Twist 타입이 고정한다:
# linear.x, linear.y, linear.z, angular.x, angular.y, angular.z.
# 제자리 회전(마지막 열)도 '움직임'이다.
#
# ★ 07-31 §9 P1 (Codex) — **YAML 자체 파서를 폐기했다.**
#   §8 보완은 키 집합을 봤지만 들여쓰기/부모를 보지 않아, `angular.x` 뒤 별도
#   `metadata.{y,z}`가 오면 그 y/z를 angular의 누락 키로 오귀속했다. 줄 상태기로 YAML을
#   복원하는 한 부모·중복·꼬리 문법을 계속 재구현하게 된다.
#   → 수집 경계에서 메시지 타입을 `geometry_msgs/msg/Twist`로 명시하고 Humble의 `--csv`를
#     사용한다. ROS가 이미 타입 역직렬화를 끝낸 뒤 내는 **고정 6열**만 판독한다.
#
#   계약(fail-closed):
#   (a) 비어 있지 않은 모든 줄이 정확히 6열이며, 여섯 값이 전부 유한 실수일 것
#   (b) 빈 줄·열 부족/초과·비숫자·NaN/Inf·YAML/경고문/임의 꼬리가 하나라도 있으면 실패
#   (c) 정상 표본을 최소 1개 확인한 뒤에만 정수를 반환
#   (d) 진짜 빈 덤프는 **Twist 타입 + 발행자 ≥1** 근거가 있을 때만 관측된 침묵(0건)
#   CSV는 한 메시지가 한 줄이고 PYTHONUNBUFFERED로 수집하므로, 구 YAML의 다중행 꼬리 잘림
#   예외는 제거한다. 불완전 CSV 한 줄은 정상으로 추정하지 않고 fail-closed 한다.
cmdvel_nonzero() {  # $1=덤프 파일 $2=발행자 수(선택) → 개수 또는 '' (판독 실패)
  python3 -c '
import math, sys
try:
    txt = open(sys.argv[1]).read()
except OSError:
    sys.exit(1)                          # 파일 없음 = 판독 실패
pub = sys.argv[2] if len(sys.argv) > 2 else ""

# (d) 침묵 예외는 진짜 빈 파일에만. pub은 호출자가 Twist 타입까지 확인한 발행자 수다.
if not txt.strip():
    if pub.isdigit() and int(pub) >= 1:
        print(0); sys.exit(0)            # 살아있는 토픽이 조용했다 = 관측된 침묵
    sys.exit(1)                          # 근거 없는 빈 덤프 = 판독 실패

nonzero = complete = 0
for line in txt.splitlines():
    if not line.strip():
        sys.exit(1)                       # (b) 중간 빈 줄 = 손상
    vals = line.split(",")
    if len(vals) != 6:
        sys.exit(1)                       # (a,b) 고정 6열 외에는 손상
    nums = []
    for v in vals:
        if not v or v != v.strip():
            sys.exit(1)                   # ROS CSV가 내지 않는 빈값/공백 = 손상
        try:
            f = float(v)
        except ValueError:
            sys.exit(1)                   # (b) 비숫자 = 손상
        if not math.isfinite(f):
            sys.exit(1)                  # (b) NaN/Inf = 손상
        nums.append(abs(f))
    complete += 1
    nonzero += sum(1 for v in nums if v > 0.01)
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
