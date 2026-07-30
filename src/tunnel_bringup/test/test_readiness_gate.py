# -*- coding: utf-8 -*-
"""
test_readiness_gate.py — 조건 기동 게이트 판정 로직 단위 회귀.

============================================================
[왜 단위테스트인가]
  `readiness_gate` 는 실차에서 "무엇이 언제 뜨는지"를 정하는 안전 경계다. 그런데
  이 코드가 실제로 처음 도는 곳은 로봇 위라, 회귀가 없으면 결함이 R0 현장에서
  발견된다 — 가장 비싼 자리다. 그래서 판정 로직만 떼어 Gazebo·DDS·로봇 없이,
  몇 ms 만에, 결정적으로 돌린다.
  (mission_manager 단위테스트와 같은 방식 = '가짜 clock 주입 + 순수 로직'.)

[무엇을 시험하나 — 이 게이트의 존재 이유는 '통과'가 아니라 '미통과'다]
  이 파일의 대부분이 **음성 케이스**다. 조건이 차면 통과하는 것보다,
  안 차면 통과 못 하는 것이 게이트의 값어치다 (AGENTS.md §3-7 부정 회귀).

[재현 대상 결함 — 2026-07-29 Codex 검토 P1]
  `_lifecycle_ok()` 가 **첫 ACTIVE 응답을 수확한 바로 그 폴링에서** 성공했다.
  게이트는 성공하면 즉시 종료하므로 다음 폴링이 없고, '5초 유효기간' 방어는
  실행될 기회 자체가 없었다 → 실효 술어가 "지금 ACTIVE 인가"가 아니라
  "한 번이라도 ACTIVE 를 답한 적 있는가"(=latch)로 퇴화.
  ⚠ **응답이 처음부터 안 오는 서버로는 이 결함을 못 잡는다**(그 경로는 원래 실패).
    한 번 답하고 멎는 서버가 유일한 재현 입력이다 — test_n1_* 가 그것이다.

[실행]
  cd ~/ros2_ws && python3 -m pytest src/tunnel_bringup/test/test_readiness_gate.py -q
  (colcon test --packages-select tunnel_bringup 로도 돈다)
"""

import tf2_ros

from tunnel_bringup import readiness_gate as rg


# ============================================================
# 가짜 부품 — 게이트가 실제로 만지는 표면만 흉내 낸다
# ============================================================
class FakeClock:
    """주입식 단조시계. 정수 ns 로 누적해 '5초 경계'를 오차 없이 넘긴다."""

    def __init__(self):
        self.ns = 0

    def advance(self, sec):
        """시간을 sec 초 흘린다."""
        self.ns += int(round(sec * 1e9))

    def now(self):
        """monotonic() 자리에 꽂히는 함수."""
        return self.ns / 1e9


class FakeState:
    """lifecycle_msgs/msg/State 흉내 (게이트는 .id 와 .label 만 읽는다)."""

    def __init__(self, state_id, label):
        self.id = state_id
        self.label = label


class FakeResponse:
    """GetState.Response 흉내."""

    def __init__(self, state):
        self.current_state = state


class FakeFuture:
    """rclpy Future 흉내 — done()/result() 만. 완료시키지 않으면 영원히 미완이다."""

    def __init__(self):
        self._result = None
        self._done = False

    def complete(self, result):
        """응답이 도착한 것으로 만든다."""
        self._result = result
        self._done = True

    def done(self):
        """완료 여부."""
        return self._done

    def result(self):
        """완료됐다면 응답."""
        return self._result


class FakeGetStateClient:
    """
    `<노드>/get_state` 서비스 클라이언트 흉내 — 응답 시나리오를 대본으로 준다.

    script 의 각 항목 = 그 순번 요청에 대한 서버 행동.
      'active'   : ACTIVE(3) 로 즉시 응답 (다음 폴링에 수확된다)
      'inactive' : INACTIVE(2) 로 즉시 응답
      'hang'     : 영원히 응답 없음 (서비스 엔드포인트는 그래프에 그대로 남는다)
      'defer'    : 지금은 미완 — 시험이 `finish(i, ...)` 로 **나중에** 완료시킨다.
                   서비스가 사라진 사이에 늦게 도착하는 응답을 흉내내는 항목이다.
    대본이 떨어지면 **마지막 항목을 계속 반복**한다.
    """

    def __init__(self, script, ready=True):
        self.script = list(script)
        self.ready = ready
        self.requests = 0
        self.removed = 0
        self.issued = []      # 발행한 future 전부 ('defer' 를 나중에 완료시키려고 붙잡아 둔다)
        self.removed_futs = []

    def service_is_ready(self):
        """그래프에 서비스가 있는가 (노드 사망 흉내는 ready=False)."""
        return self.ready

    def call_async(self, _request):
        """대본대로 완료/미완 future 를 돌려준다."""
        behavior = self.script[min(self.requests, len(self.script) - 1)]
        self.requests += 1
        fut = FakeFuture()
        if behavior == 'active':
            fut.complete(FakeResponse(FakeState(rg._STATE_ACTIVE, 'active')))
        elif behavior == 'inactive':
            fut.complete(FakeResponse(FakeState(2, 'inactive')))
        elif behavior not in ('hang', 'defer'):
            raise ValueError(f'알 수 없는 대본 항목: {behavior}')
        self.issued.append(fut)
        return fut

    def finish(self, index, state_id=rg._STATE_ACTIVE, label='active'):
        """해당 순번으로 발행한 조회가 **지금** 응답한 것으로 만든다(늦은 응답)."""
        self.issued[index].complete(FakeResponse(FakeState(state_id, label)))

    def remove_pending_request(self, future):
        """멎은 조회를 버릴 때 게이트가 부른다."""
        self.removed += 1
        self.removed_futs.append(future)


class FakeStamp:
    """builtin_interfaces/Time 흉내 (초 float 를 sec+nanosec 로 쪼갠다)."""

    def __init__(self, sec_float):
        self.sec = int(sec_float)
        self.nanosec = int(round((sec_float - self.sec) * 1e9))


class FakeTransform:
    """geometry_msgs/TransformStamped 흉내 — 게이트는 header.stamp 만 읽는다."""

    def __init__(self, sec_float):
        self.header = type('H', (), {'stamp': FakeStamp(sec_float)})()


class FakeTfBuffer:
    """
    tf2_ros.Buffer 흉내 — `stamp` 를 바꿔 가며 '갱신되는 TF' 를 만든다.

    stamp = None 이면 아직 연결 안 됨(실제 Buffer 처럼 예외를 던진다).
    """

    def __init__(self, stamp=None):
        self.stamp = stamp
        self.lookups = 0

    def lookup_transform(self, _parent, _child, _time):
        """게이트가 부르는 유일한 메서드."""
        self.lookups += 1
        if self.stamp is None:
            raise tf2_ros.LookupException('아직 연결 안 됨')
        return FakeTransform(self.stamp)


# ============================================================
# 조립 도우미
# ============================================================
def make_gate(**overrides):
    """
    ROS 자원 없이 판정 상태만 가진 게이트를 만든다.

    `Node.__init__` 을 부르지 않는다 — DDS 참가자도, 그래프도, 파라미터도 없다.
    판정 메서드(`_topics_ok`·`_lifecycle_ok`)는 인스턴스 상태만 읽으므로 그대로 돈다.
    """
    gate = rg.ReadinessGate.__new__(rg.ReadinessGate)
    clock = FakeClock()
    kwargs = dict(label='t', topics=[], latched=[], tf=[], lifecycle=[], actions=[],
                  topic_fresh=2.0, tf_fresh=0.0, timeout=30.0, poll=0.5,
                  stamp_skew_max=0.0)
    kwargs.update(overrides)
    # ROS 시계도 같은 가짜 시계를 쓴다 — TF stamp 를 초 단위로 다루기 쉽게.
    gate.init_conditions(now_fn=clock.now, ros_now_fn=clock.now, **kwargs)
    return gate, clock


def poll_lifecycle(gate, clock, times, dt=0.5):
    """폴링을 times 회 돌리고 매회의 '미충족 목록'을 모아 준다."""
    out = []
    for _ in range(times):
        out.append(gate._lifecycle_ok())
        clock.advance(dt)
    return out


# ============================================================
# lifecycle — 검토자 지정 부정 회귀 N1~N3 + 역회귀 R1
# ============================================================
def test_n1_active_once_then_hang_never_passes():
    """
    N1: ACTIVE 를 정확히 한 번 답한 뒤 멎으면 끝까지 통과하지 못한다.

    ★ 이것이 Codex P1 의 재현 입력이다. 보완 전 코드는 2회차 폴링에서
      미충족 0건이 되어 게이트가 종료코드 0 으로 끝났다.
    """
    gate, clock = make_gate(lifecycle=['/planner_server'])
    client = FakeGetStateClient(['active', 'hang'])
    gate._lc_clients = {'/planner_server': client}

    # 5초 유효기간 경계를 넉넉히 넘긴다 (0.5s × 40 = 20s)
    results = poll_lifecycle(gate, clock, 40)

    passed = [i for i, missing in enumerate(results) if not missing]
    assert passed == [], (
        f'첫 ACTIVE 1회만으로 통과했다 (통과한 폴링 회차: {passed}) — live 술어 위반')
    # 응답이 멎은 뒤에도 계속 재조회를 시도해야 한다 (한 번 묻고 포기하지 않는다)
    assert client.requests >= 3, f'재조회를 멈췄다 (요청 {client.requests}회)'
    assert client.removed >= 1, '멎은 조회를 버리지 않았다 (유효기간 방어 미동작)'


def test_n1_boundary_before_and_after_stale():
    """N1 보강: 5초 유효기간 경계 '전'과 '후' 양쪽에서 성공 0건."""
    gate, clock = make_gate(lifecycle=['/bt_navigator'])
    gate._lc_clients = {'/bt_navigator': FakeGetStateClient(['active', 'hang'])}

    before = poll_lifecycle(gate, clock, 8)      # 0 ~ 4.0s (경계 전)
    after = poll_lifecycle(gate, clock, 20)      # 4.0 ~ 14.0s (경계 후)

    assert all(before), f'경계 전에 통과 발생: {before}'
    assert all(after), f'경계 후에 통과 발생: {after}'


def test_n2_active_then_inactive_resets_confirmation():
    """N2: 첫 ACTIVE 뒤 응답이 INACTIVE 로 바뀌면 확인이 초기화되고 통과하지 않는다."""
    gate, clock = make_gate(lifecycle=['/controller_server'])
    gate._lc_clients = {'/controller_server': FakeGetStateClient(['active', 'inactive'])}

    results = poll_lifecycle(gate, clock, 12)

    assert all(results), f'ACTIVE→INACTIVE 인데 통과했다: {results}'
    assert any('inactive' in ' '.join(m) for m in results), \
        f'INACTIVE 라는 사실이 미충족 사유에 안 나온다: {results}'


def test_n3_never_answers_keeps_failing():
    """N3: 서비스는 그래프에 있으나 첫 조회부터 미완이면 계속 실패한다(기존 경로)."""
    gate, clock = make_gate(lifecycle=['/behavior_server'])
    gate._lc_clients = {'/behavior_server': FakeGetStateClient(['hang'])}

    results = poll_lifecycle(gate, clock, 30)

    assert all(results), f'응답이 한 번도 없었는데 통과했다: {results}'


def test_r1_steady_active_passes_after_two_replies():
    """
    R1(역회귀): 계속 응답하는 ACTIVE 서버에서는 통과한다 — 단 응답 2회 뒤에.

    '보완 = 더 엄격'이 '보완 = 영원히 통과 못 함'이 되지 않았는지 보는 대조군이다.
    """
    gate, clock = make_gate(lifecycle=['/velocity_smoother'])
    gate._lc_clients = {'/velocity_smoother': FakeGetStateClient(['active'])}

    results = poll_lifecycle(gate, clock, 6)
    passed = [i for i, missing in enumerate(results) if not missing]

    assert passed, f'정상 ACTIVE 서버인데 끝까지 통과 못 했다: {results}'
    assert passed[0] == rg._CONFIRM_MIN, (
        f'첫 통과가 {passed[0]}회차 — 완료된 ACTIVE 응답 '
        f'{rg._CONFIRM_MIN}회 뒤에 통과해야 한다')


def test_service_disappearing_resets_confirmation():
    """노드가 죽어 서비스가 사라지면(그래프에서 소실) 확인 상태를 버린다."""
    gate, clock = make_gate(lifecycle=['/planner_server'])
    client = FakeGetStateClient(['active'])
    gate._lc_clients = {'/planner_server': client}

    poll_lifecycle(gate, clock, 2)               # ACTIVE 1회 확인까지 진행
    client.ready = False                          # 노드 사망
    missing = gate._lifecycle_ok()

    assert missing and '서비스 없음' in missing[0], missing
    assert gate._lc_confirm.get('/planner_server', 0) == 0, '확인 상태가 남아 있다'


# ============================================================
# lifecycle — 2026-07-30 Codex 2차 P1: "증거는 세대에 속한다"
# ------------------------------------------------------------
# [무엇이 결함이었나]
#   서비스 소실 분기가 **확인 카운터만** 0 으로 되돌리고, 그때 날아가 있던 조회
#   (`_lc_pending` 의 future)는 그대로 뒀다. 단절 전에 보낸 그 조회가 복구 뒤 늦게
#   ACTIVE 로 완료되면 **새 세대의 확인 1회**로 수확된다. 복구된 서버가 딱 한 번만
#   답해도 1+1=2 가 되어 게이트가 통과 → _CONFIRM_MIN 이 실질 1회로 퇴화.
#   ⚠ 카운터를 0 으로 만드는 수정으로는 절대 안 잡힌다 — 0 으로 만든 그 카운터에
#     지난 세대 증거가 **나중에 다시 들어오는** 것이 결함의 본체다.
# ============================================================
def test_n5_pending_before_service_loss_is_not_reused():
    """
    N5: 단절 전 늦은 ACTIVE + 복구 후 ACTIVE 1회 뒤 멎으면 끝까지 통과하지 못한다.

    검토자 지정 시퀀스(`0729검토현황.md §5.2`)를 그대로 주입한다.
    """
    gate, clock = make_gate(lifecycle=['/planner_server'])
    # 1번째 조회 = 늦게 오는 응답, 2번째 = 복구 후 1회 ACTIVE, 그 뒤로는 영원히 멎음
    client = FakeGetStateClient(['defer', 'active', 'hang'])
    gate._lc_clients = {'/planner_server': client}
    results = []

    results.append(gate._lifecycle_ok())         # 폴링 0: 첫 조회(미완)
    clock.advance(0.5)

    client.ready = False                          # 폴링 1: 서비스 소실 관측
    results.append(gate._lifecycle_ok())
    clock.advance(0.5)

    client.ready = True                           # 폴링 2: 복구 + 단절 전 응답이 늦게 도착
    client.finish(0)
    results.append(gate._lifecycle_ok())
    clock.advance(0.5)

    # 폴링 3 이후: 복구된 서버는 ACTIVE 를 한 번만 답고 멎는다
    results += poll_lifecycle(gate, clock, 30)

    passed = [i for i, missing in enumerate(results) if not missing]
    assert passed == [], (
        f'단절 전 세대의 응답이 새 확인으로 재사용됐다 (통과한 폴링 회차: {passed}) — '
        '복구된 서버는 ACTIVE 를 1회밖에 답하지 않았다')


def test_service_loss_removes_pending_request():
    """
    상태 회귀: 소실을 관측한 순간 in-flight 조회가 **실제로** 폐기된다.

    검토자 요구는 "카운터가 0" 이 아니라 "pending request 가 실제로 제거되고,
    복구 뒤 확인 수는 0 부터 시작" 이다. 카운터만 보는 단언은 이 결함을 못 잡는다.
    """
    gate, clock = make_gate(lifecycle=['/bt_navigator'])
    client = FakeGetStateClient(['defer', 'active'])
    gate._lc_clients = {'/bt_navigator': client}

    gate._lifecycle_ok()                          # 조회 1건 발행 (미완)
    assert '/bt_navigator' in gate._lc_pending, '사전조건: 미완 조회가 있어야 한다'
    first = client.issued[0]
    clock.advance(0.5)

    client.ready = False
    gate._lifecycle_ok()                          # 소실 관측

    assert '/bt_navigator' not in gate._lc_pending, \
        '소실을 관측했는데 미완 조회가 남아 있다 (늦은 응답이 나중에 수확된다)'
    assert first in client.removed_futs, \
        'remove_pending_request 로 실제 폐기하지 않았다 (클라이언트에 응답이 남는다)'
    assert gate._lc_confirm.get('/bt_navigator', 0) == 0, '확인 수가 0 이 아니다'
    assert gate._lc_state.get('/bt_navigator') is None, '지난 세대 결과가 남아 있다'


def test_r3_loss_then_two_new_actives_passes():
    """
    R3(역회귀): 단절 후 복구해 **새** ACTIVE 2회를 완료하면 통과한다.

    '보완 = 더 엄격'이 '한 번 끊기면 영영 못 뜬다'가 되지 않았는지 보는 대조군이다.
    실차에서 Nav2 기동 중 DDS 재발견은 실제로 일어난다.
    """
    gate, clock = make_gate(lifecycle=['/controller_server'])
    client = FakeGetStateClient(['defer', 'active'])
    gate._lc_clients = {'/controller_server': client}

    gate._lifecycle_ok()                          # 폴링 0: 미완 조회
    clock.advance(0.5)
    client.ready = False
    gate._lifecycle_ok()                          # 폴링 1: 소실
    clock.advance(0.5)
    client.ready = True
    client.finish(0)                              # 폴링 2: 복구 + 지난 세대 늦은 응답

    results = poll_lifecycle(gate, clock, 6)
    passed = [i for i, missing in enumerate(results) if not missing]

    assert passed, f'복구 후 계속 ACTIVE 인데 통과 못 했다: {results}'
    assert passed[0] == rg._CONFIRM_MIN, (
        f'첫 통과가 복구 후 {passed[0]}회차 — 지난 세대 응답을 세지 말고 '
        f'{rg._CONFIRM_MIN}회를 새로 채워야 한다')


def test_multiple_nodes_one_hung_blocks_the_gate():
    """5종 중 하나만 멎어도 게이트는 통과하지 않는다 ('반쪽 Nav2' 방지)."""
    names = ['/controller_server', '/planner_server', '/bt_navigator',
             '/behavior_server', '/velocity_smoother']
    gate, clock = make_gate(lifecycle=names)
    gate._lc_clients = {n: FakeGetStateClient(['active']) for n in names}
    gate._lc_clients['/bt_navigator'] = FakeGetStateClient(['active', 'hang'])

    results = poll_lifecycle(gate, clock, 30)

    assert all(results), f'한 노드가 멎었는데 통과했다: {results}'
    assert all(len(m) == 1 and '/bt_navigator' in m[0] for m in results[2:]), \
        f'멎은 노드만 지목해야 한다: {results[2:5]}'


# ============================================================
# 토픽 — 같은 결함 계열 (예약 12 합류분)
# ============================================================
def test_n4_topic_single_message_then_death_never_passes():
    """
    N4: 토픽이 1건만 오고 죽으면, 신선도 창(2s) 안이라도 통과하지 않는다.

    실차 재현 상황: USB 드롭아웃으로 라이다가 첫 스캔 직후 끊긴다.
    보완 전에는 '마지막 수신이 2초 안'이라는 이유만으로 통과했다.
    """
    gate, clock = make_gate(topics=['/scan'])
    gate._last_rx['/scan'] = clock.now()          # 딱 1건 도착

    results = []
    for _ in range(6):                            # 0 ~ 3.0s (신선도 창 안팎)
        results.append(gate._topics_ok())
        clock.advance(0.5)

    assert all(results), f'1건만 온 토픽으로 통과했다: {results}'


def test_topic_burst_then_death_never_passes():
    """
    폴링 사이에 여러 건이 몰려 와도(버스트) 그 뒤 끊기면 통과하지 않는다.

    '수신 건수 2회'로 셌다면 버스트가 뚫는다 — 그래서 **서로 다른 폴링 구간**으로 센다.
    """
    gate, clock = make_gate(topics=['/odom'])
    gate._last_rx['/odom'] = clock.now()
    clock.advance(0.001)
    gate._last_rx['/odom'] = clock.now()          # 같은 폴링 구간에 2건째

    results = []
    for _ in range(6):
        results.append(gate._topics_ok())
        clock.advance(0.5)

    assert all(results), f'버스트 2건으로 통과했다: {results}'


def test_topic_streaming_passes():
    """역회귀: 계속 흐르는 토픽은 통과한다 (폴링 2회 뒤)."""
    gate, clock = make_gate(topics=['/imu/data'])

    passed_at = None
    for i in range(6):
        gate._last_rx['/imu/data'] = clock.now()   # 매 폴링 구간마다 새 메시지
        if not gate._topics_ok():
            passed_at = i
            break
        clock.advance(0.5)

    assert passed_at is not None, '정상적으로 흐르는 토픽인데 통과 못 했다'
    assert passed_at == rg._CONFIRM_MIN - 1, f'첫 통과가 {passed_at}회차 (예상 1회차)'


def test_topic_stale_after_flow_stops():
    """흐르다 끊기면 다시 미충족이 된다 (한 번 통과했다고 latch 되지 않는다)."""
    gate, clock = make_gate(topics=['/odom'])
    for _ in range(3):
        gate._last_rx['/odom'] = clock.now()
        gate._topics_ok()
        clock.advance(0.5)
    assert not gate._topics_ok(), '준비 단계에서 이미 미충족'

    clock.advance(3.0)                             # 발행이 멎고 신선도 초과
    missing = gate._topics_ok()

    assert missing and '마지막' in missing[0], missing
    assert gate._rx_confirm.get('/odom', 0) == 0, '끊겼는데 확인 상태가 남아 있다'


def test_latched_topic_single_message_is_enough():
    """
    래치 토픽(/map 등)은 1건이면 충족 — 설계상 한 번만 발행되기 때문이다.

    여기에 '두 폴링 구간' 규칙을 적용하면 영원히 통과하지 못한다.
    이 예외가 살아 있는지 지키는 회귀다.
    """
    gate, clock = make_gate(topics=[], latched=['/map'])
    gate._last_rx['/map'] = clock.now()

    clock.advance(60.0)                            # 아무리 오래돼도
    assert gate._topics_ok() == [], '래치 토픽이 신선도 때문에 막혔다'


def test_topic_never_received():
    """음성 ①: 요구 토픽이 아예 안 오면 당연히 미충족."""
    gate, _clock = make_gate(topics=['/scan'])
    missing = gate._topics_ok()
    assert missing and '수신 0건' in missing[0], missing


# ============================================================
# TF — 같은 결함 계열 (2026-07-30 사용자 판단 확대분)
# ------------------------------------------------------------
# [왜 여기까지 봐야 하나]
#   real_bringup 2단계 게이트가 `tf=['map:odom'], tf_fresh=3.0` 이다. 이 게이트가
#   0 으로 끝나면 **Nav2 가 기동한다**. 그런데 신선도만 보면, slam_toolbox 가
#   map→odom 을 한 번 발행하고 멎어도 3초 안에 판정되는 첫 폴링에서 통과한다
#   (게이트는 통과하는 순간 죽으므로 '3초 뒤 다시 본다'가 없다).
#   → lifecycle·토픽과 같은 규칙: **서로 다른 폴링에서 stamp 가 바뀐 것을 2회** 본다.
#   ⚠ tf_fresh == 0(정적 TF)은 예외다. URDF 고정 joint 는 설계상 한 번만 발행되므로
#     갱신을 요구하면 영원히 통과 못 한다 — 래치 토픽과 같은 자리다.
# ============================================================
def poll_tf(gate, clock, buf, times, stamp_step=0.0, dt=0.5):
    """폴링을 times 회 돌린다. stamp_step > 0 이면 매회 TF stamp 를 그만큼 전진시킨다."""
    out = []
    for _ in range(times):
        out.append(gate._tf_ok())
        clock.advance(dt)
        if stamp_step and buf.stamp is not None:
            buf.stamp += stamp_step
    return out


def test_n6_tf_single_update_then_frozen_never_passes():
    """
    N6: map→odom 이 딱 한 번 갱신되고 멎으면 끝까지 통과하지 못한다.

    ★ 보완 전 코드는 첫 폴링에서 미충족 0건 → Nav2 가 굳은 위치추정 위에서 기동했다.
      신선도 초과(3초) '전'에 통과해 버리므로, tf_fresh 를 줄이는 것으로는 안 고쳐진다.
    """
    gate, clock = make_gate(tf=['map:odom'], tf_fresh=3.0)
    buf = FakeTfBuffer(stamp=clock.now())          # 지금 막 1회 발행됨
    gate._tf_buffer = buf

    results = poll_tf(gate, clock, buf, 20)        # stamp 고정 = 발행이 멎음

    passed = [i for i, missing in enumerate(results) if not missing]
    assert passed == [], (
        f'1회 갱신 뒤 멎었는데 통과했다 (통과한 폴링 회차: {passed}) — '
        'Nav2 가 굳은 map→odom 위에서 기동한다')
    # '갱신 확인 부족'과 '신선도 초과' 둘 다 나와야 정상 (경계 전/후)
    joined = [' '.join(m) for m in results]
    assert any('확인' in m for m in joined[:2]), f'경계 전 사유가 갱신 확인이 아니다: {joined[:2]}'
    assert any('마지막 갱신' in m for m in joined[8:]), f'경계 후 사유가 신선도가 아니다: {joined[8:12]}'


def test_r4_tf_streaming_passes_after_two_updates():
    """R4(역회귀): 계속 갱신되는 TF 는 통과한다 — 단 서로 다른 stamp 2회 뒤에."""
    gate, clock = make_gate(tf=['map:odom'], tf_fresh=3.0)
    buf = FakeTfBuffer(stamp=clock.now())
    gate._tf_buffer = buf

    results = poll_tf(gate, clock, buf, 6, stamp_step=0.5)   # 폴링마다 갱신
    passed = [i for i, missing in enumerate(results) if not missing]

    assert passed, f'정상 갱신되는 TF 인데 끝까지 통과 못 했다: {results}'
    assert passed[0] == rg._CONFIRM_MIN - 1, (
        f'첫 통과가 {passed[0]}회차 — 서로 다른 stamp {rg._CONFIRM_MIN}회 뒤여야 한다')


def test_static_tf_single_lookup_is_enough():
    """
    tf_fresh == 0(정적 TF)은 1회 조회로 충족 — URDF 고정 joint 는 갱신되지 않는다.

    여기에 갱신 규칙을 적용하면 base_footprint→lidar_link 게이트가 영원히 안 뜬다.
    이 예외가 살아 있는지 지키는 회귀다.
    """
    gate, clock = make_gate(tf=['base_footprint:lidar_link'], tf_fresh=0.0)
    buf = FakeTfBuffer(stamp=clock.now())
    gate._tf_buffer = buf

    clock.advance(600.0)                           # 10분이 지나도
    assert gate._tf_ok() == [], '정적 TF 가 갱신 규칙에 막혔다'


def test_tf_disconnect_resets_confirmation():
    """
    TF 가 끊기면(lookup 예외) 지금까지의 갱신 확인을 버린다.

    끊긴 뒤 한 번 돌아왔다고 바로 통과하면 lifecycle 과 같은 fail-open 이 된다.
    """
    gate, clock = make_gate(tf=['map:odom'], tf_fresh=3.0)
    buf = FakeTfBuffer(stamp=clock.now())
    gate._tf_buffer = buf

    poll_tf(gate, clock, buf, 1, stamp_step=0.5)   # 갱신 1회 확인
    assert gate._tf_confirm.get('map:odom', 0) > 0, '사전조건: 확인이 1 이상이어야 한다'

    buf.stamp = None                               # TF 트리 끊김
    missing = gate._tf_ok()

    assert missing and 'LookupException' in missing[0], missing
    assert gate._tf_confirm.get('map:odom', 0) == 0, '끊겼는데 갱신 확인이 남아 있다'
