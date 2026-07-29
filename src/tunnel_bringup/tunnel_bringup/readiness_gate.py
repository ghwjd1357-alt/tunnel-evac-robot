#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""readiness_gate — "앞 단계가 진짜 살아났는가"를 확인하고 종료하는 1회성 노드.

[왜 이게 이 패키지의 본체인가]
  시뮬 런치는 다음 단계를 **고정 시간 지연**으로 띄운다 (예: "Nav2 는 8초 뒤, 미션은
  12초 뒤"). 시뮬은 매번 같은 PC 에서 같은 순서로 뜨니 그 숫자가 맞아떨어진다.
  실차는 아니다 — Jetson 부팅 상태·USB 인식 순서·micro-ROS 연결·라이다 스핀업이
  매번 다르다. 고정 시간은 두 가지로 동시에 틀린다:
    ① 너무 짧으면  → 아직 없는 TF/센서 위에서 Nav2·미션이 기동해 초기 goal 이 거부되고,
                     최악에는 TF 없는 상태로 costmap 이 굳는다.
    ② 너무 길면    → 매 기동마다 쓸데없이 기다리고, "언제 준비됐는지"를 아무도 모른다.
  → 시간이 아니라 **관측 가능한 조건**으로 기동한다. 이 노드가 그 조건을 확인한다.

[동작]
  파라미터로 받은 조건이 **전부** 만족되면 종료코드 0 으로 죽는다.
  제한시간(timeout_sec) 안에 못 만족하면 종료코드 1 로 죽는다.
  런치는 `OnProcessExit` 로 이 종료를 받아, 0 이면 다음 단계를 띄우고
  0 이 아니면 **런치 전체를 내린다**(fail-closed — 반쪽짜리 스택으로 주행 금지).

[확인할 수 있는 조건 4종]
  require_topics            : 그 토픽이 topic_fresh_sec 안에 실제로 도착했나
                              (존재만이 아니라 '흐르고 있나' — 죽은 퍼블리셔 검출)
  transient_local_topics    : /map 처럼 한 번 발행하고 마는 래치 토픽 (수신 1건 = 충족)
  require_tf                : "부모:자식" TF 가 연결됐나 (+선택적으로 신선한가)
  require_lifecycle_active  : lifecycle 노드가 ACTIVE(3) 인가 (Nav2 활성화 확인)
  require_actions           : 액션 서버가 그래프에 올라왔나 (goal 을 받을 수 있나)

[이 게이트가 확인하지 '않는' 것 — 정직하게 적어 둔다]
  · 값의 품질(오도메트리 정확도·IMU 축 부호·covariance 의미)은 못 본다.
    그건 R2~R4 의 rosbag 이 판정한다. 여기는 '살아 있는가'까지다.
  · 액션 확인은 그래프에 서버가 올라왔는가까지다 (goal 왕복 handshake 아님).
    rcl 의 액션 가용성 판정과 같은 기준이며, 실제 수락 여부는 goal 을 보내 봐야 안다.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

import tf2_ros

from lifecycle_msgs.srv import GetState

# lifecycle 상태 ID (lifecycle_msgs/msg/State) — 3 = ACTIVE
_STATE_ACTIVE = 3


class ReadinessGate(Node):
    """조건이 전부 만족될 때까지 폴링하는 1회성 노드."""

    def __init__(self):
        super().__init__('readiness_gate')

        # ── 파라미터 선언 ───────────────────────────────────────────────
        # ★ use_sim_time 은 런치가 명시적으로 false 를 준다 (실차엔 /clock 이 없다).
        self.declare_parameter('label', 'gate')
        self.declare_parameter('require_topics', [''])
        # ★ 래치(latched) 토픽 목록 — /map 처럼 "한 번 발행하고 마는" 토픽용.
        #   기본 구독은 VOLATILE 이라 구독 시점 이후에 온 것만 받는다. /map 을 그냥
        #   require_topics 에 넣으면 이미 발행이 끝난 뒤라 영원히 못 받고 게이트가 매달린다.
        #   여기 적은 토픽만 TRANSIENT_LOCAL 로 구독해 과거 1건을 받아 온다.
        self.declare_parameter('transient_local_topics', [''])
        self.declare_parameter('require_tf', [''])          # "parent:child" 형식
        self.declare_parameter('require_lifecycle_active', [''])
        self.declare_parameter('require_actions', [''])
        self.declare_parameter('topic_fresh_sec', 2.0)
        self.declare_parameter('tf_fresh_sec', 0.0)         # 0 = 신선도 미검사(정적 TF 용)
        self.declare_parameter('timeout_sec', 120.0)
        self.declare_parameter('poll_period_sec', 0.5)
        # TODO: R3 실측 후 확정 — micro-ROS 시간동기 실패(측정시각 대신 수신시각) 검출용.
        #   0.0 = 비활성. R3 rosbag 에서 odom/imu/scan 의 실제 지연을 재고 나서 켠다.
        #   ★ 추정치로 미리 채우지 않는다: 지금 아무 숫자나 넣으면 R3 에서 "튜닝 문제"와
        #     "애초에 가짜였던 값"을 구분할 수 없게 된다.
        self.declare_parameter('stamp_skew_max_sec', 0.0)

        self.label = self.get_parameter('label').value
        self.req_topics = _clean(self.get_parameter('require_topics').value)
        self.latched = set(_clean(self.get_parameter('transient_local_topics').value))
        # 래치 토픽도 검사 대상이다 — 따로 적어도 require_topics 에 없으면 합쳐 준다.
        for topic in self.latched:
            if topic not in self.req_topics:
                self.req_topics.append(topic)
        self.req_tf = _clean(self.get_parameter('require_tf').value)
        self.req_lifecycle = _clean(self.get_parameter('require_lifecycle_active').value)
        self.req_actions = _clean(self.get_parameter('require_actions').value)
        self.topic_fresh = float(self.get_parameter('topic_fresh_sec').value)
        self.tf_fresh = float(self.get_parameter('tf_fresh_sec').value)
        self.timeout = float(self.get_parameter('timeout_sec').value)
        self.poll = float(self.get_parameter('poll_period_sec').value)
        self.stamp_skew_max = float(self.get_parameter('stamp_skew_max_sec').value)

        # ── 토픽 신선도 추적 ────────────────────────────────────────────
        # 도착 시각은 ROS 시계가 아니라 monotonic 벽시계로 잰다.
        # 이유: 시계 종류(sim/real)나 시각 점프에 판정이 흔들리면 안 된다.
        self._subs = {}          # topic -> Subscription (타입을 알아낸 뒤에 생성)
        self._last_rx = {}       # topic -> monotonic 도착시각
        self._last_stamp_skew = {}   # topic -> 마지막 메시지의 stamp 오차(초)

        # ── TF ──────────────────────────────────────────────────────────
        if self.req_tf:
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        else:
            self._tf_buffer = None

        # ── lifecycle: 노드마다 get_state 클라이언트 1개 ────────────────
        self._lc_clients = {}
        self._lc_pending = {}    # node_name -> (진행 중 future, 보낸 시각)
        self._lc_state = {}      # node_name -> (가장 최근 완료 결과, 받은 시각)
        # 조회 결과·미완 조회의 유효기간. 이보다 오래되면 '모른다'로 되돌린다.
        # 폴링 몇 번은 여유를 주되(느린 Jetson), 무한정 믿지는 않는 값.
        self._lc_stale = max(5.0, 4.0 * self.poll)
        for name in self.req_lifecycle:
            srv = name if name.endswith('/get_state') else f'{name}/get_state'
            self._lc_clients[name] = self.create_client(GetState, srv)

        self._t0 = time.monotonic()
        self._last_report = 0.0

    # ── 토픽 ────────────────────────────────────────────────────────────
    def _ensure_subscriptions(self):
        """아직 구독 못 한 토픽은 그래프에서 타입을 찾아 구독한다.

        타입을 미리 코드에 박지 않는 이유: 이 게이트는 /scan·/odom 뿐 아니라
        어떤 토픽에도 쓸 수 있어야 하고, 타입을 박으면 그 메시지 패키지에
        의존성이 묶인다.
        """
        if len(self._subs) == len(self.req_topics):
            return
        graph = dict(self.get_topic_names_and_types())
        for topic in self.req_topics:
            if topic in self._subs:
                continue
            types = graph.get(topic)
            if not types:
                continue                      # 아직 퍼블리셔가 안 떴다 — 다음 폴링에 재시도
            try:
                msg_type = get_message(types[0])
            except (ImportError, ValueError, AttributeError) as exc:
                self.get_logger().warn(f'[{self.label}] {topic} 타입 로드 실패({types[0]}): {exc}')
                continue
            # ★ QoS 는 BEST_EFFORT 로 구독한다.
            #   RELIABLE 퍼블리셔  + BEST_EFFORT 구독자 = 매칭됨
            #   BEST_EFFORT 퍼블리셔 + RELIABLE 구독자  = 매칭 안 됨(조용히 0Hz)
            #   실차 /odom·/imu/data 는 micro-ROS 쪽이 BEST_EFFORT 예정이라
            #   (TEENSY 합의사항 §4.5) 게이트가 RELIABLE 을 요구하면 '센서가 죽은 것처럼'
            #   보인다. 넓게 받는 쪽으로 고정한다.
            if topic in self.latched:
                # 래치 토픽(/map 등)은 과거 1건을 받아 와야 하므로 신뢰성도 함께 올린다
                # (TRANSIENT_LOCAL 은 RELIABLE 과 짝으로 쓰는 것이 관례이고,
                #  래치 퍼블리셔는 실제로 RELIABLE 로 발행한다).
                qos = QoSProfile(
                    reliability=QoSReliabilityPolicy.RELIABLE,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=1,
                )
            else:
                qos = QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    durability=QoSDurabilityPolicy.VOLATILE,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=1,
                )
            self._subs[topic] = self.create_subscription(
                msg_type, topic, lambda msg, t=topic: self._on_msg(t, msg), qos)
            self.get_logger().info(f'[{self.label}] 구독 시작: {topic} ({types[0]})')

    def _on_msg(self, topic, msg):
        self._last_rx[topic] = time.monotonic()
        # header 가 있는 메시지면 stamp 와 현재 시각의 차이도 기록해 둔다.
        # (검사 자체는 stamp_skew_max_sec > 0 일 때만 — 기본은 비활성, 위 TODO 참조)
        header = getattr(msg, 'header', None)
        if header is not None:
            stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
            if stamp > 0.0:
                now = self.get_clock().now().nanoseconds * 1e-9
                self._last_stamp_skew[topic] = now - stamp

    def _topics_ok(self):
        missing = []
        now = time.monotonic()
        for topic in self.req_topics:
            rx = self._last_rx.get(topic)
            if rx is None:
                missing.append(f'{topic}(수신 0건)')
            elif topic in self.latched:
                # 래치 토픽은 '한 번 왔는가'가 조건이다. 신선도를 요구하면 발행이
                # 끝난 뒤라 시간이 흐르는 것만으로 조건이 다시 깨진다.
                continue
            elif now - rx > self.topic_fresh:
                missing.append(f'{topic}({now - rx:.1f}s 전이 마지막)')
            elif self.stamp_skew_max > 0.0:
                skew = self._last_stamp_skew.get(topic)
                if skew is not None and abs(skew) > self.stamp_skew_max:
                    missing.append(f'{topic}(stamp 오차 {skew:+.2f}s)')
        return missing

    # ── TF ──────────────────────────────────────────────────────────────
    def _tf_ok(self):
        missing = []
        for pair in self.req_tf:
            if ':' not in pair:
                missing.append(f'{pair}(형식 오류 — "부모:자식")')
                continue
            parent, child = pair.split(':', 1)
            try:
                tr = self._tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
            except tf2_ros.TransformException as exc:
                missing.append(f'{parent}->{child}({type(exc).__name__})')
                continue
            # tf_fresh_sec > 0 이면 '연결됐다'가 아니라 '지금도 갱신되고 있다'까지 본다.
            # 정적 TF(URDF 고정 joint)는 한 번만 발행되므로 반드시 0 으로 둘 것.
            if self.tf_fresh > 0.0:
                stamp = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                age = self.get_clock().now().nanoseconds * 1e-9 - stamp
                if age > self.tf_fresh:
                    missing.append(f'{parent}->{child}({age:.1f}s 전이 마지막 갱신)')
        return missing

    # ── lifecycle ───────────────────────────────────────────────────────
    def _lifecycle_ok(self):
        """lifecycle 노드가 **지금** ACTIVE 인지 본다.

        ★ "한 번 ACTIVE 였다"를 기억해 두고 통과시키지 않는다(= latch 금지).
          이 저장소가 이미 내린 결정이다 — 게이트가 읽는 술어는 과거 1회 성공이 아니라
          지금의 실효값이어야 한다 (MASTER_PLAN.md §8 '게이트 술어 = live').
          여기서 latch 를 쓰면, 먼저 활성화된 노드가 뒤에 죽어도 게이트가 통과해
          '반쪽 Nav2' 위에서 주행이 시작된다.
        판정 근거는 항상 '가장 최근에 완료된 조회'이고, 조회는 매 폴링마다 다시 띄운다
        (한 번에 노드당 최대 1건만 in-flight — 응답이 안 오면 그 자체가 미충족).
        """
        missing = []
        now = time.monotonic()
        for name, client in self._lc_clients.items():
            # ① 지난 폴링에 띄운 조회가 끝났으면 결과를 갱신한다.
            fut, sent_at = self._lc_pending.get(name, (None, 0.0))
            if fut is not None and fut.done():
                self._lc_pending.pop(name, None)
                result = fut.result()
                state = result.current_state if result is not None else None
                self._lc_state[name] = (state, now)
                fut = None
            elif fut is not None and now - sent_at > self._lc_stale:
                # ★ 응답이 안 오는 조회를 그냥 두면, 과거 결과가 영원히 '최신'인 척한다
                #   (= 뒷문으로 들어온 latch). 버리고 다시 묻는다.
                self._lc_pending.pop(name, None)
                try:
                    client.remove_pending_request(fut)
                except (AttributeError, KeyError, ValueError):
                    pass
                fut = None

            # ② 서비스가 사라졌으면(노드 사망) 과거 결과를 즉시 버린다.
            if not client.service_is_ready():
                self._lc_state.pop(name, None)
                missing.append(f'{name}(get_state 서비스 없음)')
                continue

            # ③ 다음 조회를 띄워 둔다 — 판정이 계속 갱신되게.
            if fut is None:
                self._lc_pending[name] = (client.call_async(GetState.Request()), now)

            state, obtained_at = self._lc_state.get(name, (None, 0.0))
            if state is None:
                missing.append(f'{name}(상태 조회 중)')
            elif now - obtained_at > self._lc_stale:
                # ④ 결과에 유효기간을 둔다 — 노드가 살아 있는 척 멈춰도 통과 못 하게.
                missing.append(f'{name}(상태 응답 {now - obtained_at:.0f}s 지연)')
            elif state.id != _STATE_ACTIVE:
                missing.append(f'{name}({state.label})')
        return missing

    # ── 액션 ────────────────────────────────────────────────────────────
    def _actions_ok(self):
        """액션 서버가 그래프에 올라왔는지 — 서비스 3종의 존재로 판정한다.

        액션 하나는 내부적으로 서비스 3개(send_goal·cancel_goal·get_result) +
        토픽 2개(feedback·status)로 구현된다. rcl 의 가용성 판정도 같은 구성요소를
        본다. '서버가 있다'까지이고 '이 goal 을 수락한다'는 아니다(위 docstring).
        """
        missing = []
        services = dict(self.get_service_names_and_types())
        for action in self.req_actions:
            need = [f'{action}/_action/{s}' for s in ('send_goal', 'cancel_goal', 'get_result')]
            absent = [s for s in need if s not in services]
            if absent:
                missing.append(f'{action}({len(absent)}/3 서비스 없음)')
        return missing

    # ── 종합 판정 ───────────────────────────────────────────────────────
    def check(self):
        """아직 못 만족한 조건 목록을 돌려준다. 빈 리스트 = 준비 완료."""
        self._ensure_subscriptions()
        missing = []
        missing += self._topics_ok()
        if self._tf_buffer is not None:
            missing += self._tf_ok()
        missing += self._lifecycle_ok()
        missing += self._actions_ok()
        return missing

    def describe(self):
        parts = []
        if self.req_topics:
            parts.append(f'토픽 {self.req_topics}(신선도 {self.topic_fresh}s)')
        if self.req_tf:
            parts.append(f'TF {self.req_tf}')
        if self.req_lifecycle:
            parts.append(f'lifecycle ACTIVE {self.req_lifecycle}')
        if self.req_actions:
            parts.append(f'액션 {self.req_actions}')
        return ' · '.join(parts) if parts else '(조건 없음)'


def _clean(values):
    """빈 문자열 자리표시자를 걷어낸다.

    rclpy 는 빈 리스트만으로 파라미터 타입을 정할 수 없어 기본값을 [''] 로 둬야 한다.
    (`declare_parameter('x', [])` 는 타입 추론 실패로 에러)
    """
    if not values:
        return []
    return [v for v in values if v]


def main(args=None):
    rclpy.init(args=args)
    node = ReadinessGate()
    label = node.label
    log = node.get_logger()
    log.info(f'[{label}] 조건 기동 대기 시작 — {node.describe()} / 제한 {node.timeout}s')

    rc = 1
    next_check = 0.0
    try:
        while rclpy.ok():
            # 콜백(구독·서비스 응답·TF)은 쉬지 않고 처리한다.
            rclpy.spin_once(node, timeout_sec=0.1)

            # ★ 조건 '평가'는 poll_period_sec 마다만 한다.
            #   spin_once 는 콜백이 하나라도 있으면 즉시 돌아오므로, 여기서 매번
            #   판정까지 하면 센서 주기(IMU 100Hz 등)만큼 그래프 조회·TF 조회가 돌아
            #   Jetson 기동 중에 쓸데없이 CPU 를 먹는다 (Nav2 활성화를 오히려 늦춘다).
            now = time.monotonic()
            if now < next_check:
                continue
            next_check = now + node.poll

            missing = node.check()
            elapsed = time.monotonic() - node._t0
            if not missing:
                log.info(f'[{label}] ✅ 준비 완료 ({elapsed:.1f}s) — 다음 단계 기동')
                rc = 0
                break
            if elapsed > node.timeout:
                log.error(f'[{label}] ❌ 제한시간 {node.timeout}s 초과 ({elapsed:.1f}s). '
                          f'미충족: {", ".join(missing)}')
                log.error(f'[{label}] 다음 단계를 기동하지 않는다 (fail-closed). '
                          f'센서·micro-ROS·Nav2 활성화를 먼저 확인할 것.')
                rc = 1
                break
            # 진행 상황을 5초마다 한 줄 — "왜 안 뜨지"를 로그만 보고 알 수 있게.
            if elapsed - node._last_report >= 5.0:
                node._last_report = elapsed
                log.warn(f'[{label}] 대기 {elapsed:.0f}/{node.timeout:.0f}s — '
                         f'미충족: {", ".join(missing)}')
    except KeyboardInterrupt:
        rc = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
