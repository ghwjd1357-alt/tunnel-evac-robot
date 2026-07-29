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
    대본이 떨어지면 **마지막 항목을 계속 반복**한다.
    """

    def __init__(self, script, ready=True):
        self.script = list(script)
        self.ready = ready
        self.requests = 0
        self.removed = 0

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
        elif behavior != 'hang':
            raise ValueError(f'알 수 없는 대본 항목: {behavior}')
        return fut

    def remove_pending_request(self, _future):
        """멎은 조회를 버릴 때 게이트가 부른다."""
        self.removed += 1


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
    gate.init_conditions(now_fn=clock.now, **kwargs)
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
