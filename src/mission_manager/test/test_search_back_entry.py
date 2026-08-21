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

🔴 **08-22 — 놓침이 역행까지 가는 시간이 늘었다.** `GUIDE → SEARCH_BACK` 사이에
`HOLD`(제자리 재수집, `search_back.hold_sec` 기본 4.0s)가 들어갔기 때문이다.
그래서 아래 회귀 10곳의 EMPTY 구간에 **각각 4.0초를 더했다** — 기대 동작을 바꾼
것이 아니라, 같은 결론에 도달하는 데 필요한 시간이 `lost_sec(3.0) + hold_sec(4.0)`
로 늘어난 것이다. HOLD 자체의 회귀는 `test_hold.py` 가 따로 본다.
"""

import ast
import os
import re
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


def tick_only(env, seconds, dt=0.1):
    """시간과 tick 만 진행하고 **/scan 은 한 장도 넣지 않는다** (08-02 §27 P1 재료).

    run() 과의 차이가 이 검사의 전부다: run() 은 매 스텝 monitor.update() 를 부르므로
    `_last_scan_t` 가 곧바로 non-None 이 되어 '센서가 아직 한 번도 안 살아난' 구간을
    영원히 재현하지 못한다. 라이다 드라이버·DDS discovery 가 미션 노드보다 늦게 뜨는
    것은 실차의 **정상 기동 순서**다."""
    for _ in range(round(seconds / dt)):
        env.clock.advance(dt)
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
    run(env, 7.6, EMPTY)           # ②③④⑤

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
    # 🔴 08-22 — 여기는 단순히 4초를 더한 자리가 아니다. 이 시험은 역행이 **매번
    #   거부되는**(TF 사망 → last_seen None) 경로가 max_attempts 를 소진해 관제
    #   보고까지 가는지를 본다. HOLD 가 들어가면서 **시도 한 번의 값이
    #   lost_sec(3.0) + hold_sec(4.0) 로 2.3배가 됐다.**
    #   ⚠ 이건 실차에서도 그렇다 — 라이다로 못 찾는 상황에서 "추종자 확인 불가"
    #     보고가 그만큼 늦어진다. 그 대가로 흔한 경우의 22초 역행을 아낀다.
    #     보고가 늦는 동안 로봇은 **서 있다**(HOLD 는 goal 을 안 낸다) — 위험이
    #     늘어나는 방향은 아니다.
    #   실측 전개(08-22): 3.6 HOLD → 7.6 attempts=1 → 11.1 HOLD → 15.1 attempts=2
    #                      → 18.6 HOLD → **22.6 give_up**.
    #   시도 한 번이 7.5s 인 이유는 hold_sec(4.0) 만이 아니다 — `HOLD → GUIDE`
    #   복귀가 `[reset-role] guide-entry` 를 타서 놓침 타이머가 재무장되므로
    #   lost_sec(3.5) 를 매번 다시 채운다. 🔵 그 자리를 특례로 빼지 않는다:
    #   복귀 뒤 3.5초는 "사람이 돌아왔나" 를 다시 보는 시간이고, 그동안 사람이
    #   보이면 record_last_seen 이 살아나 역행 자체가 필요 없어진다.
    #   ⚠ 숫자를 넉넉히가 아니라 **실측+여유**로 박는다 — 이 값이 늘어나면
    #     보고가 늦어진 것이므로 그때 이 시험이 깨져서 알려주는 편이 낫다.
    run(env, 25.0, EMPTY)

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

    run(env, 5.0, EMPTY)                   # 넘김 → 이제는 열려야 한다
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
    run(env, 4.6, EMPTY)                   # t=3.6 — 3.5 tick 에서 놓침 확정
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

    run(env, 7.5, EMPTY)           # 이제 이 세대에서 lost_sec 를 채운다
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
    run(env, 6.0, EMPTY)                   # 넘김 → 이제는 열려야 한다
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
    run(env, 9.0, EMPTY)                   # 놓침 → 역행 (이전 임무)
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
    run(env, 5.0, EMPTY)                   # +4.0 = hold_sec
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

    run(env, 7.5, EMPTY)                   # 진짜로 다시 놓치면 2회차는 열린다
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
    run(env, 6.0, EMPTY)                   # 넘김 → 양쪽 다 열려야 한다
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


# ============================================================
# ★ 센서 관측 세대 (08-02 검토 §27 P1) — '아직 한 번도 안 살아난 라이다'
#   ─────────────────────────────────────────────────────────
#   §27 불승인 사유: §26 보완이 GUIDE 진입 시 놓침 타이머의 기산점을 세우는데,
#   그 시점에 **/scan 이 한 장도 온 적이 없으면**(`_last_scan_t is None`)
#   모니터의 '단절 복구' 보호가 발동하지 않는다 — 그 가드가 "이전에 scan 을
#   받은 적이 있을 때"만 열리기 때문이다. 그래서 첫 유효 빈 프레임 한 장이
#   도착하는 순간 그동안 흐른 벽시계 시간이 통째로 '미검출 시간'으로 계산돼
#   즉시 lost 가 되고, 그 구간엔 기록도 못 했으므로(stale) 좌표가 없어
#   **역행 0회로 예산만 태우고 단독 탈출** — §26 P1 과 결과가 같다.
#   ⚠ 라이다 드라이버·DDS discovery 가 미션 노드보다 늦게 뜨는 것은 실차의
#     정상 기동 순서다. 시뮬에선 거의 안 나지만 실물에선 흔하다.
# ============================================================
def test_sb20_first_valid_scan_after_dead_start_does_not_declare_loss():
    """부정 — scan 이력 0 인 채 GUIDE 진입 → lost_sec 초과 → 첫 EMPTY 1장.

    이 한 장으로 예산이 깎이거나 역행이 열리면 FAIL.
    ★ 경계 양방향(`AGENTS.md §3-10 ⑤`): 첫 유효 scan 뒤 fresh 누적이 lost_sec
      미만이면 계속 보류하고, 넘긴 뒤에만 **진짜 좌표로** 역행 1회.
    """
    env = make_env(state=State.GATHER)
    tick_only(env, 9.0)                    # scan 0장인 채 GATHER 8초 → 진짜 GUIDE 전이
    assert env.node.state == State.GUIDE
    assert env.node.monitor._last_scan_t is None, '전제 불성립: scan 이 들어와 버렸다'

    tick_only(env, 5.0)                    # 여전히 0장 — lost_sec(3.0)을 훌쩍 넘긴다
    assert env.node.monitor.scan_stale()
    assert env.node.search_attempts == 0    # stale 중엔 판정 보류 (기존 계약)
    assert env.tf.calls == 0                # stale 중엔 기록도 보류 (sb9 와 같은 계약)

    env.clock.advance(0.1)
    env.node.monitor.update(EMPTY)         # ★ 첫 유효 스캔 한 장 (사람 없음)
    assert not env.node.monitor.lost('any'), \
        '센서가 죽어 있던 시간이 미검출 시간으로 계산됐다 — watchdog 계약 위반'

    env.node.tick()
    assert env.node.search_attempts == 0, '첫 유효 프레임 한 장이 예산을 깎았다'
    assert not env.node.give_up
    assert env.node.state == State.GUIDE
    assert env.node.last_seen == (1.0, 2.0), 'scan 이 살아났는데 기록이 안 됐다'

    run(env, 2.5, EMPTY)                   # fresh 누적 2.5s < lost_sec 3.0
    assert env.node.state == State.GUIDE, '새 세대의 lost_sec 를 안 채우고 열렸다'
    assert env.node.search_attempts == 0

    run(env, 5.0, EMPTY)                   # 넘김 → 이제는 열려야 한다
    assert env.node.state == State.SEARCH_BACK
    assert env.node.search_attempts == 1
    assert env.node.search_goal == {'x': 1.0, 'y': 2.0, 'yaw': 0.0}, \
        '좌표 없이 예산만 태우는 경로로 갔다'
    assert not any('역행 불가' in m for m in msgs(env))


def test_sb21_dead_start_still_ends_with_real_search_backs_then_report():
    """정책 종결 — 결정 B 는 '**실제 역행** max_attempts 회 → 보고 → 단독 탈출'이다.
    센서가 늦게 살아난 경우에도 역행 0회로 예산만 태우면 그 결정을 어긴 것이다."""
    env = make_env(state=State.GATHER)
    tick_only(env, 9.0)
    tick_only(env, 5.0)
    env.clock.advance(0.1)
    env.node.monitor.update(EMPTY)         # 첫 유효 스캔

    max_attempts = int(env.node.wp['search_back']['max_attempts'])
    run_with_nav(env, 60.0, EMPTY)

    assert env.node.give_up
    assert env.node.search_attempts == max_attempts
    assert [tag for tag, _ in env.goals].count('search_back') == max_attempts, \
        '역행 goal 이 안 나갔다 — 예산만 태우고 보고했다(§27 P1 재발)'
    assert any('추종자 확인 불가' in m for m in msgs(env))
    assert not any('역행 불가' in m for m in msgs(env))


def test_sb22_first_valid_scan_with_person_keeps_normal_following():
    """역회귀 — 첫 유효 스캔이 **사람**이면 기존 의미 그대로 정상 추종이다.
    §27 보완이 '첫 스캔은 무조건 재무장'으로 과하게 가도 이건 안 깨져야 한다."""
    env = make_env(state=State.GATHER)
    tick_only(env, 9.0)
    tick_only(env, 5.0)

    env.clock.advance(0.1)
    env.node.monitor.update(PERSON)        # 첫 유효 스캔 = 사람
    assert not env.node.monitor.lost('any')

    run(env, 5.0, PERSON)
    assert env.node.state == State.GUIDE
    assert env.node.search_attempts == 0
    assert not env.node.give_up
    assert env.node.last_seen == (1.0, 2.0)


# ============================================================
# ★ P2 (§27.3) — reset() 의 호출 계약을 기계가 대조한다
#   사람이 목록을 보고 설명으로 옮겨 적는 순간 둘은 갈라진다(`AGENTS.md §3-10 ②`).
#   → 호출 자리마다 `[reset-role] <이름>` 을 달고, 그 집합과 독스트링의 집합을
#     **양방향**으로 대조한다. 자리를 추가하고 설명을 안 고치면 여기서 FAIL 한다.
# ============================================================
RESET_ROLE_RE = re.compile(r'\[reset-role\]\s*([a-z0-9-]+)')


def _production_src(name):
    p = os.path.join(os.path.dirname(__file__), '..', 'mission_manager', name)
    with open(p, encoding='utf-8') as f:
        return f.read()


def _reset_call_lines(src):
    """`self.monitor.reset(...)` **실행 호출**의 줄 번호를 `ast` 로 찾는다 (예약 20).

    🔴 **문자열 검색으로 세면 주석 처리된 호출도 호출로 센다.** 08-02 검토 §28 P2-1 이
    격리 사본에서 실증했다 — 실행 호출을 주석으로 덮고 `pass` 를 넣어도 이 검사가
    `1 passed` 였다. 즉 "자리 소실 양방향 게이트"라는 주장이 그때는 성립하지 않았다.

    `ast` 는 주석을 파싱하지 않으므로 여기서 세는 것은 **실제로 실행되는 호출뿐**이다.
    역할 태그(`[reset-role] …`)는 주석에 있어 AST 에 안 남으므로, 호출의 줄 번호로
    원본 줄을 되짚어 읽는다 — **호출이 사라지면 그 줄도 같이 사라진다**.
    """
    tree = ast.parse(src)
    lines = src.splitlines()
    found = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        f = node.func
        if not (isinstance(f, ast.Attribute) and f.attr == 'reset'):
            continue
        owner = f.value
        if not (isinstance(owner, ast.Attribute) and owner.attr == 'monitor'):
            continue
        if not (isinstance(owner.value, ast.Name) and owner.value.id == 'self'):
            continue
        # 여러 줄 호출이면 태그가 끝 줄에 붙을 수 있으므로 호출이 걸친 줄을 다 본다.
        end = getattr(node, 'end_lineno', node.lineno)
        found.append('\n'.join(lines[node.lineno - 1:end]))
    return found


def test_sb23_reset_docstring_covers_every_production_caller():
    node_src = _production_src('mission_node.py')
    call_lines = _reset_call_lines(node_src)
    assert call_lines, '전제 불성립: 생산 코드에서 reset 호출을 못 찾았다'

    tagged = [role for ln in call_lines for role in RESET_ROLE_RE.findall(ln)]
    assert len(tagged) == len(call_lines), \
        (f'reset() 호출 자리 {len(call_lines)} 곳 중 [reset-role] 태그가 붙은 것은 '
         f'{len(tagged)} 곳뿐이다 — 태그 없는 자리는 대조에서 조용히 빠진다')

    documented = set(RESET_ROLE_RE.findall(FollowerMonitor.reset.__doc__ or ''))
    called = set(tagged)
    assert called <= documented, \
        f'reset() 설명이 빠뜨린 호출 역할: {sorted(called - documented)}'
    assert documented <= called, \
        f'설명에는 있는데 호출 자리가 사라진 역할: {sorted(documented - called)}'


def test_sb23b_commented_out_reset_is_not_counted_as_a_caller():
    """🔴 예약 20 부정 회귀 — 주석 처리된 호출을 호출로 세면 안 된다.

    이 검사가 등록된 사유가 바로 이것이다(08-02 검토 §28 P2-1): 구판은
    `mission_node.py` 를 줄 단위 문자열로 읽어 `'self.monitor.reset('` 이 든 줄을
    호출자로 셌고, **실행 호출을 주석으로 덮고 `pass` 를 넣어도 통과**했다.
    ⚠ 문자열 검색으로 돌아가면 이 테스트가 죽는다.
    """
    live = 'class N:\n    def f(self):\n        self.monitor.reset("any")  # [reset-role] a\n'
    assert len(_reset_call_lines(live)) == 1

    commented = ('class N:\n    def f(self):\n'
                 '        # self.monitor.reset("any")  # [reset-role] a\n'
                 '        pass\n')
    assert _reset_call_lines(commented) == [], \
        '주석 처리된 호출을 실행 호출로 셌다 — 예약 20 의 결함이 되살아났다'


def test_sb23c_other_objects_reset_is_not_miscounted():
    """`self.monitor.reset` 만 센다 — 이름이 비슷한 다른 호출을 끌어오지 않는다."""
    src = ('class N:\n    def f(self):\n'
           '        self.other.reset("any")\n'
           '        self.monitor.clear("any")\n'
           '        monitor.reset("any")\n')
    assert _reset_call_lines(src) == []
