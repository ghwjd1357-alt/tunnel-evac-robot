# -*- coding: utf-8 -*-
"""
test_search_back_entry.py — SEARCH_BACK 진입 봉인 (예약 16 ①+②′, 08-01)
============================================================
[무엇을 잡나 — 미션 안전 결함, 근거 = 0730_현황.md §2]
  GUIDE 중 추종감시의 **쓰는 술어와 읽는 술어가 달랐다.**

    if  monitor.visible('any'):  record_last_seen()   # 엄격(1초 연속 검출)이 쓰고
    elif monitor.lost('any'):    enter_search_back()  # 관대(3초 미검출)가 읽는다

  검출이 깜빡이면 `visible` 은 한 번도 참이 안 되는데 `lost` 는 참이 된다.
  그때 `last_seen is None` → `enter_search_back()` 이 **조용히 return** 하고,
  시도 횟수를 소모하지 않으므로 `give_up`(= '관제 보고: 추종자 확인 불가')에
  **영영 도달하지 못한다.** 그동안 `/mission_state` 는 계속 GUIDE 를 발행하고
  escape goal 은 살아 있다 —
  **"대피자를 놓친 걸 알면서 되돌아가지도, 알리지도 않고 혼자 나간다."**

[불변조건 (②′)]
  *"기록은 그 기록을 소비하는 술어와 같은 타이머를 써야 한다."*
  안전 술어(`visible`)는 그대로 두고 **기록 조건만** 소비 쪽(`lost` 가 보는
  타이머)에 맞춘다. `last_seen` 은 로봇 자기 좌표 저장일 뿐 안전 판단이 아니다.

[★ 이 파일이 게이트인 이유 — 핸드오프 함정 8]
  "테스트가 생산 코드를 흉내 내면 그 테스트는 게이트가 아니다"(08-01 §21 P2-①).
  → 판정 술어는 **진짜 `FollowerMonitor`** 가 계산하고, GUIDE 분기·
    `enter_search_back()`·`record_last_seen()` 은 **진짜 `MissionNode`** 것을 부른다.
    파라미터도 **진짜 `config/waypoints.yaml`** 에서 읽는다(값 드리프트 봉쇄).
    가짜로 대체한 것은 이 결함과 무관한 바깥 배선뿐 —
    TF·Nav2 goal·SpeedManager·발행자.

[★ 보완 전 실패를 관측했다 — AGENTS.md §3-7]
  이 파일을 먼저 쓰고 **수정 전 코드에서 5건 FAIL**(sb1·sb2·sb3·sb4·sb10) 하는
  것을 눈으로 본 뒤 ①·②′ 를 넣었다. 관측 기록 = 0801_현황.md §4.
  "고쳤더니 통과한다"는 증거가 아니다.
  나머지 6건(sb5~sb9·sb11)은 **보완 전에도 통과했다** — 역회귀 앵커다.
  보완 후에도 그대로 통과해야 "안 돼야 하는 게 여전히 안 된다"가 증명된다.

[실행]
  cd ~/ros2_ws && python3 -m pytest src/mission_manager/test/ -q
"""

import os
import types

import yaml

from mission_manager.mission_node import MissionNode, State
from mission_manager.follower_monitor import FollowerMonitor

# 이미 20여 개 테스트가 검증한 가짜 부품을 재사용한다 (정의를 두 곳에 두면 갈라진다).
from test_follower_monitor import FakeClock, make_scan


# ============================================================
# 진짜 설정값 — 코드와 테스트가 갈라지지 않게 yaml 에서 직접 읽는다
# ============================================================
WP_PATH = os.path.join(os.path.dirname(__file__), '..', 'config', 'waypoints.yaml')

PERSON = make_scan([(180.0, 10, 1.5)])   # 정후방 1.5m 사람 크기 덩어리
EMPTY = make_scan([])                    # 배경(10m)뿐 — 문턱 2.5m 안에 점 0개


def load_wp():
    with open(WP_PATH, 'r') as f:
        return yaml.safe_load(f)


# ============================================================
# 가짜 바깥 배선 (결함과 무관한 것만)
# ============================================================
class FakeTF:
    """tf_buffer 흉내 — record_last_seen 이 읽는 필드만. fail=True 면 예외."""

    def __init__(self, pose=(1.0, 2.0)):
        self.pose = pose
        self.fail = False
        self.calls = 0

    def lookup_transform(self, target, source, when):
        self.calls += 1
        if self.fail:
            raise RuntimeError('map→base_footprint 미준비')
        x, y = self.pose
        return types.SimpleNamespace(
            transform=types.SimpleNamespace(
                translation=types.SimpleNamespace(x=x, y=y)))


def make_env(tf_ok=True, state=State.GUIDE):
    """껍데기 MissionNode + ★진짜 FollowerMonitor★ + 진짜 waypoints.yaml.

    반환 env:
      env.node  = MissionNode (기본 state=GUIDE)
      env.clock = FakeClock (정수 ns 누적 — 디바운스 경계 오차 방지)
      env.tf    = FakeTF (pose 이동·fail 토글)
      env.logs  = [(메시지, kwargs)] — throttle 유무까지 단언 가능
      env.goals = [(tag, wp)] · env.cancels = 취소 횟수
      env.step  = 0.1s 스텝 카운터 (5스텝마다 tick — 실제 2Hz)

    ★ `state` 인자의 의미 (08-01 검토 §26 P1 이 지적한 바로 그 구멍):
      기본값 `State.GUIDE` 는 **"이미 GUIDE 로 유도 중"** 을 뜻하며 `_prev_tick_state`
      도 같이 GUIDE 로 둔다 — 진입 전이가 **아니다.** 진입 경계를 검사하는 회귀는
      `state=State.GATHER` 로 만들어 **진짜 `_on_guide_speed_ok()`** 가 전환하게 한다
      (`speed.request_guide` 를 그 콜백에 배선해 뒀다). 상태를 손으로 GUIDE 에 꽂는
      env 만으로는 진입 경계가 영원히 녹색이다 — 그게 §26 불승인 사유였다.
    """
    node = MissionNode.__new__(MissionNode)
    clock = FakeClock()
    wp = load_wp()
    sb = wp['search_back']

    # ★ 생산 배선과 같은 인자로 진짜 모니터를 만든다 (mission_node.py:478~).
    node.monitor = FollowerMonitor(
        clock,
        cone_half_deg=float(sb['cone_half_deg']),
        max_range=float(sb['detect_range']),
        lost_sec=float(sb['lost_sec']),
        seen_sec=float(sb['seen_sec']),
        max_cluster_width=float(sb['cluster_max_width']),
        min_points=int(sb['min_points']),
        range_jump=float(sb['range_jump']),
        edge_margin=float(sb['edge_margin']),
        scan_timeout=float(sb['scan_timeout']))

    logs = []
    node.get_logger = lambda: types.SimpleNamespace(
        warn=lambda m, **kw: logs.append((m, kw)),
        info=lambda m, **kw: logs.append((m, kw)),
        error=lambda m, **kw: logs.append((m, kw)))

    node.wp = wp
    node.state = state
    node._prev_tick_state = state    # "이 상태로 이미 진행 중" — 진입 전이가 아니다
    node.siren_on = True
    node.fire = None
    node.goal_active = False
    node.give_up = False
    node.search_attempts = 0
    node.last_seen = None
    node.search_goal = None
    node.refind_since = None
    # --- 상태머신의 나머지 소유 필드 (진짜 전이 경로가 읽고 쓴다) ---
    node.patrol_idx = 0
    node.gather_wp = None
    node.gather_since = clock.now()
    node._guide_pending = False
    node._escaped_logged = False
    node._cancel_intent = None
    node.fault_retries = 0
    node.fault_since = None
    node.resume_state = None
    node.MAX_RETRIES = 2
    node.RETRY_WAIT = 5.0

    tf = FakeTF()
    tf.fail = not tf_ok
    node.tf_buffer = tf

    goals, cancels = [], []
    node.send_goal = lambda w, tag='': (goals.append((tag, w)),
                                        setattr(node, 'goal_active', True))[0]
    node.cancel_current_goal = lambda: (cancels.append(1),
                                        setattr(node, 'goal_active', False))[0]
    node.state_pub = types.SimpleNamespace(publish=lambda m: None)
    node.siren_pub = types.SimpleNamespace(publish=lambda m: None)
    node.get_clock = lambda: clock
    # 속도 수명주기는 이 결함과 무관 — 저속이 '적용 확인된' 정상 상태로 고정.
    # ★ request_guide 는 진짜 성공 콜백에 배선한다 — GATHER→GUIDE 전환을
    #   테스트가 흉내내지 않고 **생산 코드 `_on_guide_speed_ok()` 가** 하게.
    #   (SpeedManager 의 비동기 수명주기만 가짜다. 그건 이 결함과 무관하다.)
    node.speed = types.SimpleNamespace(
        tick=lambda: None,
        ensure_sync=lambda v: None,
        guide_speed_recovery_exhausted=False,
        guide_speed_applied=True,
        request_guide=lambda v: node._on_guide_speed_ok(),
        request_restore=lambda v: None,
        cancel_pending=lambda why: None)

    return types.SimpleNamespace(node=node, clock=clock, tf=tf, logs=logs,
                                 goals=goals, cancels=cancels, step=0)


def run(env, seconds, scan, dt=0.1):
    """스캔 10Hz · tick 2Hz 로 실제 주기대로 섞어 돌린다.
    ⚠ int() 는 11.999→11 로 한 스텝 깎아먹는다 → round (구판 실측 함정)."""
    for _ in range(round(seconds / dt)):
        env.clock.advance(dt)
        env.node.monitor.update(scan)
        env.step += 1
        if env.step % 5 == 0:
            env.node.tick()


def run_with_nav(env, seconds, scan, dt=0.1):
    """run() + "보낸 goal 은 Nav2 가 도착시켜 준다" 시늉.

    가짜인 것은 **'언제 도착했나'뿐**이고 도착 처리는 진짜 `on_reached()` 가 한다
    (SEARCH_BACK → refind_since 세팅). goal_active 를 내리는 것도 실제 GoalManager
    가 결과 콜백에서 하는 일이다. 이게 없으면 역행이 영원히 '주행 중'이라
    max_attempts 소진 경로를 끝까지 못 돌린다."""
    for _ in range(round(seconds / dt)):
        env.clock.advance(dt)
        env.node.monitor.update(scan)
        env.step += 1
        if env.step % 5 == 0:
            env.node.tick()
            if (env.node.state == State.SEARCH_BACK
                    and env.node.goal_active
                    and env.node.refind_since is None):
                env.node.goal_active = False
                env.node.on_reached()          # 역행 지점 도착


def msgs(env):
    return [m for m, _ in env.logs]


# ============================================================
# ★ 부정 회귀 5단계 (0730_현황.md §2.6) — 보완 전 FAIL 을 관측한 검사
# ============================================================
def test_sb1_flicker_loss_actually_reaches_search_back():
    """① 검출 1장 → ② 미검출 → ③ lost_sec 경과 → ④ GUIDE tick → ⑤ 반복.

    보완 전: `visible` 이 한 번도 참이 아니라 last_seen 이 None →
             enter_search_back 이 조용히 return → search_attempts 0 고정 ·
             give_up False 고정 · 영원히 GUIDE. (이 단언 3개가 전부 FAIL 했다)
    """
    env = make_env()
    run(env, 0.1, PERSON)          # ① 검출 1장 (visible 은 seen_sec 1.0 미달로 False)
    assert not env.node.monitor.visible('any')
    run(env, 3.6, EMPTY)           # ②③④⑤

    assert env.node.last_seen is not None, \
        '소비 술어(lost)와 같은 타이머로 기록되지 않았다 — ②′ 불변조건 위반'
    assert env.node.state == State.SEARCH_BACK, \
        '놓침을 판정하고도 역행하지 않았다 (사람을 두고 계속 탈출)'
    assert env.node.search_attempts == 1, \
        '역행 시도 횟수가 소모되지 않았다 — give_up 보고에 영영 도달 못 한다'
    assert env.node.search_goal == {'x': 1.0, 'y': 2.0, 'yaw': 0.0}


def test_sb2_never_silently_stuck_after_loss():
    """갇힘 = '놓침을 판정했는데 역행도 보고도 없이 GUIDE 를 무한 반복'.
    10초를 더 돌려도 그 상태가 남아 있으면 FAIL."""
    env = make_env()
    run(env, 0.1, PERSON)
    run(env, 13.0, EMPTY)
    assert env.node.search_attempts > 0 or env.node.give_up, \
        '갇힘: 역행도 보고도 없이 GUIDE 만 반복했다'


def test_sb3_tf_dead_reaches_report_instead_of_silent_loop():
    """②′ 안전망 — TF 가 끝내 안 열려 last_seen 이 None 인 채 놓침이 확정되면,
    조용히 도는 대신 예산을 소모하고 **관제 보고 경로**로 빠져야 한다.

    (last_seen 은 한 번 기록되면 관제 reset 전엔 None 으로 안 돌아간다 →
     이 자리의 None 은 '순간 딸꾹질'이 아니라 GUIDE 내내 TF 가 한 번도
     안 풀렸다는 뜻이다. 그 상태는 Nav2 주행 자체가 불가능한 상태다.)
    보완 전: give_up False 고정 → FAIL."""
    env = make_env(tf_ok=False)
    run(env, 0.1, PERSON)
    run(env, 6.0, EMPTY)

    assert env.node.last_seen is None            # TF 가 죽었으니 기록은 여전히 불가
    assert env.node.give_up, \
        '역행 불가가 무한 반복 — 관제 보고 경로에 도달하지 못했다'
    assert any('추종자 확인 불가' in m for m in msgs(env))
    assert env.node.state == State.GUIDE          # 보고 후 단독 탈출은 기존 정책 그대로


# ============================================================
# ① 진단 로그 — 동작 변경 0
# ============================================================
def test_sb4_record_failure_is_logged_with_throttle():
    """`except Exception: pass` 가 삼키던 TF 실패를 로그로 남긴다.
    ⚠ TF 미준비면 매 tick 발생 → throttle 없으면 2Hz 로그 폭주."""
    env = make_env(tf_ok=False)
    env.node.record_last_seen()

    hits = [(m, kw) for m, kw in env.logs if '기록 실패' in m]
    assert hits, 'TF 예외가 조용히 삼켜졌다 — 다음 재현 때 원인을 밝힐 재료가 없다'
    assert all('throttle_duration_sec' in kw for _, kw in hits), \
        'throttle 없음 — 2Hz 로그 폭주 (기존 코드 관례 위반)'


def test_sb5_record_failure_does_not_change_behavior():
    """① 은 진단이다 — 실패해도 예외를 밖으로 내지 않고, 기존 기록도 지우지 않는다."""
    env = make_env()
    env.node.record_last_seen()
    assert env.node.last_seen == (1.0, 2.0)

    env.tf.fail = True
    env.node.record_last_seen()            # 예외 → 로그만 (raise 금지)
    assert env.node.last_seen == (1.0, 2.0), '기록 실패가 기존 목격 지점을 지웠다'


# ============================================================
# 역회귀 앵커 — "안 돼야 하는 게 여전히 안 되는가" (AGENTS.md §3-5·§3-7)
# ============================================================
def test_sb6_visible_predicate_not_relaxed():
    """③(`visible` 무관용 리셋 완화)은 열지 않았다.
    안전 술어를 풀면 실물 라이다 노이즈에서 가짜 '따라오는 중'이 생긴다."""
    env = make_env()
    run(env, 0.1, PERSON)
    assert not env.node.monitor.visible('any')     # 1장으로는 아직
    run(env, 0.8, PERSON)                          # 누적 0.9s < seen_sec 1.0
    assert not env.node.monitor.visible('any')
    run(env, 0.3, PERSON)                          # 1.2s ≥ 1.0
    assert env.node.monitor.visible('any')


def test_sb7_continuous_follower_never_enters_search_back():
    """정상 추종 중에는 역행이 절대 안 열린다 (오작동 방향 봉인)."""
    env = make_env()
    run(env, 10.0, PERSON)
    assert env.node.state == State.GUIDE
    assert env.node.search_attempts == 0
    assert not env.node.give_up
    assert env.node.last_seen == (1.0, 2.0)
    assert [tag for tag, _ in env.goals] == ['escape']


def test_sb8_short_flicker_below_lost_sec_does_not_reverse():
    """경계 양방향 (AGENTS.md §3-10 ⑤) — lost_sec 미만이면 안 열리고,
    넘기면 열린다. 부등호 한쪽만 박으면 반대 방향을 아직 안 물은 것이다."""
    env = make_env()
    run(env, 2.0, PERSON)
    run(env, 2.5, EMPTY)                   # 미검출 2.5s < lost_sec 3.0
    assert env.node.state == State.GUIDE, '깜빡임만으로 비싼 역행이 발동했다'
    assert env.node.search_attempts == 0

    run(env, 1.0, EMPTY)                   # 넘김 → 이제는 열려야 한다
    assert env.node.state == State.SEARCH_BACK
    assert env.node.search_attempts == 1


def test_sb9_no_record_while_scan_stale():
    """/scan 끊김 중엔 기록도 보류한다 — 소비 술어(lost)가 판단을 보류하는 구간이다.
    ⚠ 이 앵커가 없으면 ②′ 를 `not lost` 만으로 구현했을 때
      라이다가 죽은 동안 목격 지점이 로봇을 따라 계속 전진한다
      (`lost` 는 stale 중 False 를 돌려주므로)."""
    env = make_env()
    run(env, 2.0, PERSON)
    assert env.node.last_seen == (1.0, 2.0)

    env.tf.pose = (9.0, 9.0)               # 로봇이 계속 전진했다고 치자
    env.clock.advance(2.0)                 # /scan 두절 (scan_timeout 1.0 초과)
    assert env.node.monitor.scan_stale()
    env.node.tick()
    assert env.node.last_seen == (1.0, 2.0), \
        '라이다가 죽은 동안 마지막 목격 지점이 로봇을 따라 전진했다'


def test_sb10_record_stops_at_loss_declaration():
    """기록은 놓침이 확정되는 순간 멈춘다 — 그 tick 에서 또 갱신하면
    역행 목표가 '지금 로봇 자리'가 되어 역행이 무의미해진다."""
    env = make_env()
    run(env, 0.1, PERSON)
    run(env, 2.9, EMPTY)                   # t=3.0 — gap 2.9 < 3.0, 아직 놓침 아님
    assert env.node.state == State.GUIDE
    assert env.node.last_seen == (1.0, 2.0)

    env.tf.pose = (7.0, 7.0)
    run(env, 0.6, EMPTY)                   # t=3.6 — 3.5 tick 에서 놓침 확정
    assert env.node.state == State.SEARCH_BACK
    assert env.node.last_seen == (1.0, 2.0), '놓침 확정 tick 에서 기록이 또 갱신됐다'
    assert env.node.search_goal == {'x': 1.0, 'y': 2.0, 'yaw': 0.0}


def test_sb11_give_up_still_stops_all_follower_monitoring():
    """give_up 이후엔 추종감시 블록 전체가 꺼진다 (기존 단독 탈출 정책 불변).
    ②′ 안전망이 이 정책을 우회해 계속 기록/역행하면 안 된다."""
    env = make_env()
    env.node.give_up = True
    run(env, 0.1, PERSON)
    run(env, 5.0, EMPTY)
    assert env.node.last_seen is None
    assert env.node.search_attempts == 0
    assert env.node.state == State.GUIDE


# ============================================================
# ★ GUIDE 진입 세대 (08-01 검토 §26 P1) — 진짜 전이를 태우는 부정 회귀
#   ─────────────────────────────────────────────────────────
#   §26 불승인 사유: 위 sb1~sb11 은 전부 `state = GUIDE` 를 손으로 꽂은
#   env 라 **GUIDE 진입 경계가 한 번도 실행되지 않았다.** 아래는 진짜
#   `_on_guide_speed_ok()` · `on_reached()` · `on_cmd('reset')` · FAULT 재시도로
#   전이시킨다. grep 전수로 센 GUIDE 진입 4경로 + 관제 reset 잔재를 각각 덮는다.
# ============================================================
def test_sb12_prelost_gather_to_guide_does_not_burn_budget_without_search_back():
    """① GATHER 중 1프레임만 보고 놓침 타이머가 만료된 채 GUIDE 로 들어간다.

    보완 전(§26.2 재현): 진입 첫 tick 이 lost=True 로 시작해 record_last_seen()
    호출 기회가 **0회**(tf_calls=0) → last_seen=None → '역행 불가'로 예산 2회를
    1.5초에 태우고 give_up. **TF 는 멀쩡한데 SEARCH_BACK 을 한 번도 안 하고
    사람을 버린다.**
    보완 후: 진입이 관측 세대를 재무장 → 정상 기록 → 진짜 좌표로 역행 1회.
    """
    env = make_env(state=State.GATHER)
    run(env, 0.1, PERSON)          # GATHER 중 1프레임 검출
    run(env, 3.6, EMPTY)           # lost_sec(3.0) 초과 — 아직 GATHER 다
    assert env.node.monitor.lost('any'), '전제 불성립: 전환 전에 이미 놓침이어야 한다'
    assert env.node.state == State.GATHER

    run(env, 5.0, EMPTY)           # gather_wait_sec(8.0) 충족 → 진짜 GUIDE 전환
    assert env.node.state == State.GUIDE, '전제 불성립: _on_guide_speed_ok 가 안 걸렸다'
    assert env.node.search_attempts == 0, \
        '진입하자마자 예산이 깎였다 — 역행 한 번 없이 give_up 으로 간다 (§26 P1)'
    assert env.tf.calls > 0, 'TF 가 멀쩡한데 기록 기회가 0회였다 (§26 P1 의 핵심)'
    assert env.node.last_seen == (1.0, 2.0)
    assert not any('역행 불가' in m for m in msgs(env)), \
        'TF 실패가 아닌데 "역행 불가"로 예산을 태웠다'

    run(env, 3.5, EMPTY)           # 이제 이 세대에서 lost_sec 를 채운다
    assert env.node.state == State.SEARCH_BACK
    assert env.node.search_attempts == 1
    assert env.node.search_goal == {'x': 1.0, 'y': 2.0, 'yaw': 0.0}
    assert not env.node.give_up


def test_sb13_never_seen_follower_ends_finitely_with_report():
    """② GATHER 부터 GUIDE 까지 **0프레임** 검출 — 사용자 정책 결정(08-01).

    보완 전: `_last_seen_t` 가 None 이라 lost 가 **영원히 False** →
    역행도 보고도 안 열리고 escape 만 계속 = 사람을 두고 조용히 나간다.
    정책 A(다른 놓침과 동일 취급): 역행 max_attempts 회 → '추종자 확인 불가'
    보고 → 단독 탈출. **유한하고 관측 가능하게** 끝나야 한다.
    """
    env = make_env(state=State.GATHER)
    run_with_nav(env, 9.0, EMPTY)          # 검출 0건인 채 GATHER 8초 → GUIDE
    assert env.node.state == State.GUIDE
    assert env.node.monitor._last_seen_t['any'] is not None, \
        'GUIDE 진입이 관측 세대를 열지 않았다 — lost 가 영원히 False 로 남는다'

    max_attempts = int(env.node.wp['search_back']['max_attempts'])
    run_with_nav(env, 60.0, EMPTY)

    assert env.node.give_up, '무한 GUIDE — 역행도 보고도 열리지 않았다'
    assert env.node.search_attempts == max_attempts
    assert any('추종자 확인 불가' in m for m in msgs(env)), '보고 경로에 도달 못 함'
    assert [tag for tag, _ in env.goals].count('search_back') == max_attempts, \
        '예산만 태우고 실제 역행은 안 했다'
    assert not any('역행 불가' in m for m in msgs(env)), \
        'TF 가 멀쩡한데 좌표 없이 예산만 소모했다'


def test_sb14_fault_resume_to_guide_rearms_generation():
    """③ FAULT → resume_state 복귀. FAULT 로 멈춰 있던 시간이 놓침 시간으로
    계산되면 복귀 즉시 역행이 터진다 (로봇은 서 있었는데 사람을 '놓쳤다'고 판정).

    ★ 경계 양방향 (AGENTS.md §3-10 ⑤): 복귀 후 lost_sec 미만이면 안 열리고,
      넘기면 열려야 한다. 한쪽만 박으면 반대 방향을 안 물은 것이다."""
    env = make_env()
    run(env, 2.0, PERSON)                  # 정상 추종 중
    assert env.node.last_seen == (1.0, 2.0)

    env.node.enter_fault()                 # 진짜 FAULT 진입 (resume_state=GUIDE)
    assert env.node.resume_state == State.GUIDE
    run(env, 6.0, EMPTY)                   # RETRY_WAIT(5.0) 경과 → 진짜 GUIDE 복귀
    assert env.node.state == State.GUIDE, '전제 불성립: FAULT 재시도가 안 걸렸다'
    assert env.node.search_attempts == 0, \
        'FAULT 로 정지해 있던 시간이 놓침으로 계산돼 복귀 즉시 역행했다'

    run(env, 2.0, EMPTY)                   # 복귀 후 2.5s < lost_sec 3.0
    assert env.node.state == State.GUIDE
    assert env.node.search_attempts == 0
    run(env, 2.0, EMPTY)                   # 넘김 → 이제는 열려야 한다
    assert env.node.state == State.SEARCH_BACK
    assert env.node.search_attempts == 1


def test_sb15_control_reset_does_not_leak_old_generation_into_next_mission():
    """④ 관제 reset 은 last_seen 만 비우고 **모니터는 안 건드린다**(생산 사실).
    그 잔재가 다음 임무의 GUIDE 로 새면, 진입하자마자 옛 타이머로 놓침 판정 +
    last_seen 은 None → '역행 불가'로 예산이 즉시 소진된다.

    ⚠ 상태를 손으로 꽂는 것은 APPROACH 하나뿐이다(주행 배선은 이 결함과 무관).
      APPROACH→GATHER 는 진짜 `on_reached()`, GATHER→GUIDE 는 진짜 콜백이 한다."""
    env = make_env()
    run(env, 0.1, PERSON)
    run(env, 5.0, EMPTY)                   # 놓침 → 역행 (이전 임무)
    assert env.node.search_attempts == 1

    env.node.on_cmd(types.SimpleNamespace(data='reset'))
    assert env.node.state == State.PATROL
    assert env.node.last_seen is None and env.node.search_attempts == 0
    assert env.node.monitor._last_seen_t['any'] is not None, \
        '전제 불성립: 관제 reset 이 모니터 잔재를 남긴다는 사실이 바뀌었다'

    env.node.state = State.APPROACH        # 다음 임무 — 집결지로 주행 중
    env.node.on_reached()                  # 진짜 전이: APPROACH → GATHER
    assert env.node.state == State.GATHER
    run(env, 9.0, EMPTY)                   # 진짜 전이: GATHER → GUIDE
    assert env.node.state == State.GUIDE
    assert env.node.search_attempts == 0, \
        '옛 임무의 목격 시각으로 새 임무 진입 즉시 놓침 판정했다'

    run(env, 2.5, EMPTY)
    assert env.node.state == State.GUIDE, '새 세대의 lost_sec 를 안 채우고 열렸다'
    run(env, 1.0, EMPTY)
    assert env.node.state == State.SEARCH_BACK   # 반대 방향 경계
    assert env.node.search_attempts == 1


def test_sb16_refind_return_to_guide_does_not_relose_immediately():
    """역회귀 — SEARCH_BACK 재발견 복귀(이미 reset('any') 하던 경로)가
    초크포인트 추가로 깨지지 않는다."""
    env = make_env(state=State.SEARCH_BACK)
    env.node.last_seen = (1.0, 2.0)
    env.node.search_goal = {'x': 1.0, 'y': 2.0, 'yaw': 0.0}
    env.node.search_attempts = 1

    run(env, 1.5, PERSON)                  # seen_sec(1.0) 충족 → 재발견 복귀
    assert env.node.state == State.GUIDE
    run(env, 2.5, PERSON)                  # 계속 보이는 동안은 재놓침 없음
    assert env.node.state == State.GUIDE
    assert env.node.search_attempts == 1

    run(env, 3.5, EMPTY)                   # 진짜로 다시 놓치면 2회차는 열린다
    assert env.node.state == State.SEARCH_BACK
    assert env.node.search_attempts == 2


def test_sb17_refind_timeout_return_to_guide_does_not_relose_immediately():
    """역회귀 — SEARCH_BACK 재탐색 실패 복귀(07-07 의 두 번째 reset('any') 경로).
    "같은 곳 두 번"으로 예산이 소진되던 07-07 결함이 되살아나면 안 된다."""
    env = make_env(state=State.SEARCH_BACK)
    env.node.last_seen = (1.0, 2.0)
    env.node.search_goal = {'x': 1.0, 'y': 2.0, 'yaw': 0.0}
    env.node.search_attempts = 1
    env.node.refind_since = env.clock.now()

    run(env, 11.0, EMPTY)                  # refind_wait_sec(10.0) 만료 → GUIDE 복귀
    assert env.node.state == State.GUIDE
    assert env.node.search_attempts == 1, '복귀 즉시 같은 지점으로 2차 역행이 나갔다'

    # ⚠ 경계에 딱 붙이지 않는다 — 이 검사는 **역회귀 앵커**다(구판도 통과해야
    #   "안 깨졌다"가 증명된다). 초크포인트는 이 경로의 재무장을 0.5초(1 tick)
    #   늦추는데, 그 차이에 단언을 걸면 앵커가 부정 회귀로 둔갑한다.
    run(env, 1.5, EMPTY)                   # 복귀 후 ~2s < lost_sec 3.0
    assert env.node.state == State.GUIDE
    run(env, 2.0, EMPTY)                   # 넘김 → 양쪽 다 열려야 한다
    assert env.node.state == State.SEARCH_BACK
    assert env.node.search_attempts == 2


# ============================================================
# ★ P2 (§26.3) — last_seen 의 실제 의미를 코드로 못박는다
#   주석만 고치면 다음 회차에 또 갈라진다. "무엇이 저장되는가"를 단언한다.
# ============================================================
def test_sb18_last_seen_is_robot_pose_before_loss_not_the_detection_moment():
    """검출이 끊긴 뒤에도 놓침 확정 직전까지 갱신된다 —
    즉 저장값은 '목격한 순간의 좌표'가 아니라 **그 직전 로봇 좌표**다."""
    env = make_env()
    run(env, 1.0, PERSON)
    assert env.node.last_seen == (1.0, 2.0)

    env.tf.pose = (5.0, 6.0)               # 검출이 끊긴 뒤에도 로봇은 전진한다
    run(env, 2.0, EMPTY)                   # 2.5s < lost_sec 3.0 — 아직 놓침 아님
    assert env.node.state == State.GUIDE
    assert env.node.last_seen == (5.0, 6.0), \
        'last_seen 은 "마지막 목격 좌표"가 아니라 놓침 확정 직전의 로봇 좌표다'


def test_sb19_last_seen_updates_after_rearm_without_any_detection():
    """세대 재무장 뒤에는 **실제 재검출 0건**이어도 갱신된다.
    이 값을 '사람을 봤다'는 관측 증거로 읽으면 안 된다는 규약의 근거."""
    env = make_env(state=State.GATHER)
    run(env, 9.0, EMPTY)                   # 검출 0건으로 GUIDE 진입
    assert env.node.state == State.GUIDE

    env.tf.pose = (7.0, 8.0)
    run(env, 1.0, EMPTY)
    assert env.node.last_seen == (7.0, 8.0), \
        '재무장 뒤 무검출 구간에서도 갱신된다 — 검출 증거가 아니다'
