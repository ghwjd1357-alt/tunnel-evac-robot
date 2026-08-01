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


def make_env(tf_ok=True):
    """껍데기 MissionNode + ★진짜 FollowerMonitor★ + 진짜 waypoints.yaml.

    반환 env:
      env.node  = MissionNode (state=GUIDE)
      env.clock = FakeClock (정수 ns 누적 — 디바운스 경계 오차 방지)
      env.tf    = FakeTF (pose 이동·fail 토글)
      env.logs  = [(메시지, kwargs)] — throttle 유무까지 단언 가능
      env.goals = [(tag, wp)] · env.cancels = 취소 횟수
      env.step  = 0.1s 스텝 카운터 (5스텝마다 tick — 실제 2Hz)
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
    node.state = State.GUIDE
    node.siren_on = True
    node.fire = None
    node.goal_active = False
    node.give_up = False
    node.search_attempts = 0
    node.last_seen = None
    node.search_goal = None
    node.refind_since = None

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
    node.speed = types.SimpleNamespace(
        tick=lambda: None,
        ensure_sync=lambda v: None,
        guide_speed_recovery_exhausted=False,
        guide_speed_applied=True)

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
