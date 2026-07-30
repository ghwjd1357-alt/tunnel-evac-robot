#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
readiness_gate — "앞 단계가 진짜 살아났는가"를 확인하고 종료하는 1회성 노드.

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

[★ 공통 규칙 ① — "한 번 관측 = 통과" 금지 (_CONFIRM_MIN)]
  토픽·lifecycle·TF(tf_fresh>0) 는 **서로 다른 폴링에서 2회** 관측돼야 충족이다.
  게이트는 조건이 차는 순간 죽으므로, 첫 관측으로 통과시키면 그 뒤의 방어(신선도·
  유효기간)가 실행될 기회 자체가 없다 — 술어가 조용히 latch 로 퇴화한다.
  대가는 통과가 폴링 1회만큼 늦는 것뿐.
  예외는 **설계상 1회만 발행되는 것** 둘이다: 래치 토픽(/map)과 정적 TF(tf_fresh=0).
  이들에 2회를 요구하면 영원히 통과하지 못한다.

[★ 공통 규칙 ② — 증거는 '세대'에 속한다]
  연결이 끊긴 것을 관측하면(get_state 서비스 소실 · TF lookup 실패 · 토픽 신선도 초과)
  **그 경계 이전의 증거를 전부 버린다** — 이미 센 카운터뿐 아니라 **아직 응답이 안 온
  진행 중 조회까지.** 남겨 두면 지난 세대의 늦은 응답이 새 세대의 확인으로 수확되어
  ①의 '2회' 가 실질 1회로 되돌아간다 (2026-07-30 Codex 2차 P1 이 정확히 이것이었다).

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

# ★ "한 번 관측 = 통과" 금지 — 조건 하나를 충족으로 인정하려면 **서로 다른 폴링에서
#   완료된 관측이 이만큼** 있어야 한다.
#   왜 1 이면 안 되는가 (2026-07-29 Codex P1 실측): 게이트는 조건이 다 차는 순간
#   종료코드 0 으로 죽는다. 그래서 '첫 관측을 수확한 그 폴링'에서 성공해 버리면
#   그 뒤 폴링이 아예 없고, 결과 유효기간(_lc_stale)·신선도(topic_fresh) 같은
#   '나중에 확인하는' 방어는 실행될 기회 자체가 없다. 결국 실효 술어가
#   "지금 살아 있는가"가 아니라 "한 번이라도 살아 있던 적 있는가"(=latch)로 퇴화한다
#   — 이 저장소가 금지한 것이다 (MASTER_PLAN.md §8 '게이트 술어 = live').
#   2 로 두면 조건 충족까지 폴링 1회(=0.5s)가 더 걸린다. 그 지연이 정상 동작이다.
_CONFIRM_MIN = 2


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

        self.init_conditions(
            label=self.get_parameter('label').value,
            topics=_clean(self.get_parameter('require_topics').value),
            latched=_clean(self.get_parameter('transient_local_topics').value),
            tf=_clean(self.get_parameter('require_tf').value),
            lifecycle=_clean(self.get_parameter('require_lifecycle_active').value),
            actions=_clean(self.get_parameter('require_actions').value),
            topic_fresh=float(self.get_parameter('topic_fresh_sec').value),
            tf_fresh=float(self.get_parameter('tf_fresh_sec').value),
            timeout=float(self.get_parameter('timeout_sec').value),
            poll=float(self.get_parameter('poll_period_sec').value),
            stamp_skew_max=float(self.get_parameter('stamp_skew_max_sec').value),
        )

        # ── 여기서부터가 ROS 자원 (단위테스트는 여기를 만들지 않는다) ──
        if self.req_tf:
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # lifecycle: 노드마다 get_state 클라이언트 1개
        for name in self.req_lifecycle:
            srv = name if name.endswith('/get_state') else f'{name}/get_state'
            self._lc_clients[name] = self.create_client(GetState, srv)

    def init_conditions(self, label, topics, latched, tf, lifecycle, actions,
                        topic_fresh, tf_fresh, timeout, poll, stamp_skew_max,
                        now_fn=time.monotonic, ros_now_fn=None):
        """
        판정에 쓰는 상태를 전부 세팅한다 — ROS 자원은 하나도 만들지 않는다.

        ★ __init__ 에서 굳이 떼어 놓은 이유: 이 게이트의 판정 로직(`_topics_ok` ·
          `_lifecycle_ok`)은 실차 안전 경계인데, DDS·그래프 없이는 시험할 수 없다면
          회귀를 짤 수 없다. 여기까지만 부르면 노드 없이 판정만 골라 시험할 수 있다
          (`test/test_readiness_gate.py` 가 그렇게 쓴다 — mission_manager 단위테스트의
          '가짜 clock 주입 + 순수 로직' 방식과 같은 결).
        now_fn 도 같은 이유로 주입식이다. 실동작은 monotonic 벽시계 그대로이고,
        시험에서만 가짜 시계를 꽂아 '5초 유효기간' 경계를 결정적으로 넘긴다.

        ★ 시계가 **두 개**인 것에 주의: now_fn 은 monotonic(도착시각·유효기간용),
          ros_now_fn 은 ROS 시계(TF stamp 와 비교하는 용도)다. 섞으면 안 된다 —
          stamp 는 ROS 시계로 찍혀 오므로 monotonic 과 빼면 의미 없는 수가 나온다.
          기본값 None 이면 실제 노드 시계를 쓴다(= 실동작). 시험에서만 주입한다.
        """
        self.label = label
        self.req_topics = list(topics)
        self.latched = set(latched)
        # 래치 토픽도 검사 대상이다 — 따로 적어도 require_topics 에 없으면 합쳐 준다.
        for topic in sorted(self.latched):
            if topic not in self.req_topics:
                self.req_topics.append(topic)
        self.req_tf = list(tf)
        self.req_lifecycle = list(lifecycle)
        self.req_actions = list(actions)
        self.topic_fresh = topic_fresh
        self.tf_fresh = tf_fresh
        self.timeout = timeout
        self.poll = poll
        self.stamp_skew_max = stamp_skew_max
        self._now = now_fn
        self._ros_now = ros_now_fn or self._node_clock_sec

        # ── 토픽 신선도 추적 ────────────────────────────────────────────
        # 도착 시각은 ROS 시계가 아니라 monotonic 벽시계로 잰다.
        # 이유: 시계 종류(sim/real)나 시각 점프에 판정이 흔들리면 안 된다.
        self._subs = {}              # topic -> Subscription (타입을 알아낸 뒤에 생성)
        self._last_rx = {}           # topic -> monotonic 도착시각
        self._last_stamp_skew = {}   # topic -> 마지막 메시지의 stamp 오차(초)
        self._rx_confirm = {}        # topic -> 서로 다른 폴링에서 받은 횟수 (_CONFIRM_MIN 까지)
        self._rx_seen = {}           # topic -> 직전 폴링에서 본 도착시각 (새 메시지 판별용)

        # ── TF 갱신 추적 ────────────────────────────────────────────────
        self._tf_buffer = None
        self._tf_confirm = {}        # "부모:자식" -> 서로 다른 stamp 를 본 횟수
        self._tf_seen = {}           # "부모:자식" -> 직전 폴링에서 본 stamp

        # ── lifecycle ───────────────────────────────────────────────────
        self._lc_clients = {}
        self._lc_pending = {}    # node_name -> (진행 중 future, 보낸 시각)
        self._lc_state = {}      # node_name -> (가장 최근 완료 결과, 받은 시각)
        self._lc_confirm = {}    # node_name -> 연속으로 '완료된' ACTIVE 응답 수
        # 조회 결과·미완 조회의 유효기간. 이보다 오래되면 '모른다'로 되돌린다.
        # 폴링 몇 번은 여유를 주되(느린 Jetson), 무한정 믿지는 않는 값.
        self._lc_stale = max(5.0, 4.0 * self.poll)

        self._t0 = self._now()
        self._last_report = 0.0

    def _node_clock_sec(self):
        """ROS 시계의 '지금'을 초 단위 float 로 (실동작용 기본 ros_now_fn)."""
        return self.get_clock().now().nanoseconds * 1e-9

    # ── 토픽 ────────────────────────────────────────────────────────────
    def _ensure_subscriptions(self):
        """
        아직 구독 못 한 토픽은 그래프에서 타입을 찾아 구독한다.

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
        self._last_rx[topic] = self._now()
        # header 가 있는 메시지면 stamp 와 현재 시각의 차이도 기록해 둔다.
        # (검사 자체는 stamp_skew_max_sec > 0 일 때만 — 기본은 비활성, 위 TODO 참조)
        header = getattr(msg, 'header', None)
        if header is not None:
            stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
            if stamp > 0.0:
                now = self.get_clock().now().nanoseconds * 1e-9
                self._last_stamp_skew[topic] = now - stamp

    def _topics_ok(self):
        """
        토픽이 **지금 흐르고 있는가**. '마지막 수신이 신선하다'만으로는 부족하다.

        ★ 왜 부족한가 (_CONFIRM_MIN 머리말과 같은 결함): 퍼블리셔가 메시지를 딱 1건
          보내고 죽어도, 그 뒤 topic_fresh(기본 2s) 안에 판정되면 통과한다. 실차에서
          이건 흔한 입력이다 — USB 드롭아웃으로 라이다가 첫 스캔 직후 끊기는 경우.
          그래서 **서로 다른 폴링 구간에서** _CONFIRM_MIN 회 수신돼야 충족으로 본다.
          '수신 건수'가 아니라 '수신된 폴링 수'로 세는 이유: 버스트로 여러 건이
          한꺼번에 왔다가 끊기는 경우를 건수로 세면 그대로 뚫린다.
        """
        missing = []
        now = self._now()
        for topic in self.req_topics:
            rx = self._last_rx.get(topic)
            if rx is None:
                missing.append(f'{topic}(수신 0건)')
                continue
            if topic in self.latched:
                # 래치 토픽은 '한 번 왔는가'가 조건이다. 신선도도, 반복 관측도 요구하지
                # 않는다 — /map 은 설계상 한 번만 발행되므로 요구하면 영원히 못 통과한다.
                continue
            if now - rx > self.topic_fresh:
                # 흐름이 끊겼다 → 지금까지의 관측을 버린다(한 번 찼다고 latch 되지 않게).
                self._rx_confirm[topic] = 0
                missing.append(f'{topic}({now - rx:.1f}s 전이 마지막)')
                continue
            if rx != self._rx_seen.get(topic):
                # 직전 폴링 이후 새 메시지가 왔다 = 관측 1회 추가.
                self._rx_seen[topic] = rx
                seen = self._rx_confirm.get(topic, 0)
                if seen < _CONFIRM_MIN:
                    self._rx_confirm[topic] = seen + 1
            seen = self._rx_confirm.get(topic, 0)
            if seen < _CONFIRM_MIN:
                missing.append(f'{topic}(흐름 확인 {seen}/{_CONFIRM_MIN})')
                continue
            if self.stamp_skew_max > 0.0:
                skew = self._last_stamp_skew.get(topic)
                if skew is not None and abs(skew) > self.stamp_skew_max:
                    missing.append(f'{topic}(stamp 오차 {skew:+.2f}s)')
        return missing

    # ── TF ──────────────────────────────────────────────────────────────
    def _tf_ok(self):
        """
        요구된 TF 가 연결됐는가 — tf_fresh > 0 이면 **지금도 갱신되고 있는가**까지.

        ★★ 2026-07-30 확대분 (lifecycle·토픽과 같은 결함 계열).
          신선도만 보면 뚫린다: `map:odom` 을 한 번 발행하고 slam_toolbox 가 멎어도,
          tf_fresh(3초) 안에 판정되는 **첫 폴링**에서 통과한다. 게이트는 통과하는
          순간 죽으므로 '3초 뒤 다시 본다'가 없다 → real_bringup 2단계 게이트가
          굳은 위치추정 위에서 Nav2 를 띄운다.
          → 서로 다른 폴링에서 **stamp 가 바뀐 것을 _CONFIRM_MIN 회** 봐야 충족.
          ⚠ tf_fresh 를 줄이는 것으로는 안 고쳐진다 — 첫 폴링의 나이는 0 에 가깝다.

        ⚠ tf_fresh == 0(정적 TF)은 예외다. URDF 고정 joint 는 설계상 한 번만
          발행되므로 갱신을 요구하면 영원히 통과 못 한다 — 래치 토픽과 같은 자리다.
        """
        missing = []
        now = self._ros_now()
        for pair in self.req_tf:
            if ':' not in pair:
                missing.append(f'{pair}(형식 오류 — "부모:자식")')
                continue
            parent, child = pair.split(':', 1)
            try:
                tr = self._tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
            except tf2_ros.TransformException as exc:
                # 끊긴 순간 지금까지의 갱신 확인을 버린다(돌아왔다고 바로 통과 금지).
                self._tf_confirm[pair] = 0
                missing.append(f'{parent}->{child}({type(exc).__name__})')
                continue
            if self.tf_fresh <= 0.0:
                continue                       # 정적 TF — '연결됐다'까지가 조건
            stamp = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            age = now - stamp
            if age > self.tf_fresh:
                self._tf_confirm[pair] = 0
                missing.append(f'{parent}->{child}({age:.1f}s 전이 마지막 갱신)')
                continue
            if stamp != self._tf_seen.get(pair):
                # 직전 폴링 이후 새로 발행됐다 = 갱신 관측 1회 추가.
                self._tf_seen[pair] = stamp
                seen = self._tf_confirm.get(pair, 0)
                if seen < _CONFIRM_MIN:
                    self._tf_confirm[pair] = seen + 1
            seen = self._tf_confirm.get(pair, 0)
            if seen < _CONFIRM_MIN:
                missing.append(f'{parent}->{child}(갱신 확인 {seen}/{_CONFIRM_MIN})')
        return missing

    # ── lifecycle ───────────────────────────────────────────────────────
    def _drop_lc_pending(self, name, client):
        """
        이 노드의 진행 중 조회를 '지난 세대의 증거'로 보고 버린다.

        ★★ 2026-07-30 Codex 2차 P1 보완의 핵심. 버리는 곳이 두 군데(응답 정지 /
          서비스 소실)인데 한쪽만 고쳐져 있었던 것이 그 결함의 직접 원인이라,
          **버리는 코드를 여기 하나로 모은다.** 새 초기화 지점이 생기면 이 함수를
          부르는 것이 규칙이다.

        왜 pop 이 본질인가: `_lc_pending` 에서 빼는 순간 그 future 는 다시 읽히지
        않는다(= 늦은 응답이 확인 수를 올릴 수 없다). `remove_pending_request` 는
        클라이언트 내부에 남는 요청 슬롯 정리용이며, 실패해도 판정은 안전하다.
        """
        fut, _sent_at = self._lc_pending.pop(name, (None, 0.0))
        if fut is None:
            return
        try:
            client.remove_pending_request(fut)
        except (AttributeError, KeyError, ValueError):
            pass

    def _lifecycle_ok(self):
        """
        요구된 lifecycle 노드가 **지금** ACTIVE 인지 본다.

        ★ "한 번 ACTIVE 였다"를 기억해 두고 통과시키지 않는다(= latch 금지).
          이 저장소가 이미 내린 결정이다 — 게이트가 읽는 술어는 과거 1회 성공이 아니라
          지금의 실효값이어야 한다 (MASTER_PLAN.md §8 '게이트 술어 = live').
          여기서 latch 를 쓰면, 먼저 활성화된 노드가 뒤에 죽어도 게이트가 통과해
          '반쪽 Nav2' 위에서 주행이 시작된다.

        ★★ 2026-07-29 Codex P1 보완 — **첫 ACTIVE 응답만으로는 충족이 아니다.**
          그전 판정은 "가장 최근에 완료된 조회가 5초 안쪽의 ACTIVE 인가"였는데,
          첫 응답을 수확한 바로 그 폴링에서 그 조건이 참이 된다(나이 0초). 게이트는
          충족되면 즉시 종료하므로 다음 폴링이 없고, 유효기간 방어는 영영 실행되지
          않는다 → "한 번 답했으면 통과"라는 latch 와 같아진다. 실제 위험 입력은
          '노드가 ACTIVE 를 한 번 답한 직후 executor 가 멎는' 경우로, 서비스 이름은
          그래프에 남아 있어 아무 신호도 나오지 않는다.
          → 노드마다 **서로 다른 두 번의 '완료된' ACTIVE 응답**(_CONFIRM_MIN)을 요구한다.
            노드당 in-flight 는 항상 1건이므로 확인은 폴링당 최대 1씩만 오르고,
            따라서 2회는 반드시 서로 다른 왕복이다.
          ⚠ 유효기간(_lc_stale)을 줄이는 것으로는 이 결함이 안 고쳐진다 — 첫 응답의
            나이는 0초라 아무리 조여도 그 폴링에서 통과한다.

        조회는 매 폴링마다 다시 띄운다(노드당 최대 1건 in-flight). 서비스 소실·응답
        정지(stale)·non-ACTIVE 응답에서는 확인 상태를 **0 으로 되돌린다.**
        """
        missing = []
        now = self._now()
        for name, client in self._lc_clients.items():
            # ① 지난 폴링에 띄운 조회가 끝났으면 결과를 갱신한다.
            fut, sent_at = self._lc_pending.get(name, (None, 0.0))
            if fut is not None and fut.done():
                self._lc_pending.pop(name, None)
                result = fut.result()
                state = result.current_state if result is not None else None
                self._lc_state[name] = (state, now)
                if state is not None and state.id == _STATE_ACTIVE:
                    seen = self._lc_confirm.get(name, 0)
                    if seen < _CONFIRM_MIN:
                        self._lc_confirm[name] = seen + 1
                else:
                    # ACTIVE 가 아닌 응답이 하나라도 오면 지금까지의 확인은 무효다.
                    self._lc_confirm[name] = 0
                fut = None
            elif fut is not None and now - sent_at > self._lc_stale:
                # ★ 응답이 안 오는 조회를 그냥 두면, 과거 결과가 영원히 '최신'인 척한다
                #   (= 뒷문으로 들어온 latch). 버리고 다시 묻는다.
                self._drop_lc_pending(name, client)
                self._lc_confirm[name] = 0
                fut = None

            # ② 서비스가 사라졌으면(노드 사망) 지난 세대의 증거를 **전부** 버린다.
            #   ★★ 2026-07-30 Codex 2차 P1: 여기서 카운터(_lc_confirm)만 0 으로 되돌리고
            #     진행 중 조회(_lc_pending)를 남겨 두면, 단절 전에 보낸 그 조회가 복구 뒤
            #     늦게 ACTIVE 로 완료돼 ①에서 **새 세대의 확인 1회**로 수확된다. 복구된
            #     서버가 딱 한 번만 답해도 1+1=2 가 되어 게이트가 통과한다 → _CONFIRM_MIN
            #     이 실질 1회로 퇴화. 서비스 소실은 **이전 서버의 liveness 증거를 무효화하는
            #     경계**이므로, 그 경계를 넘는 증거는 완료 여부와 무관하게 폐기한다.
            #   ⚠ 알려진 한계: DDS 가 알아채기 전에(<1폴링) 서버가 재시작하면 소실이
            #     관측되지 않아 두 세대를 구분할 수 없다. GetState 응답에 세대 식별자가
            #     없어 이 계층에서는 못 닫는다 (CURRENT_HANDOFF '남은 검증 상한').
            if not client.service_is_ready():
                self._drop_lc_pending(name, client)
                self._lc_state.pop(name, None)
                self._lc_confirm[name] = 0
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
                self._lc_confirm[name] = 0
                missing.append(f'{name}(상태 응답 {now - obtained_at:.0f}s 지연)')
            elif state.id != _STATE_ACTIVE:
                missing.append(f'{name}({state.label})')
            elif self._lc_confirm.get(name, 0) < _CONFIRM_MIN:
                # ⑤ ACTIVE 를 한 번 봤을 뿐 — 다음 조회의 응답이 실제로 완료돼야 통과.
                missing.append(
                    f'{name}(ACTIVE 확인 {self._lc_confirm.get(name, 0)}/{_CONFIRM_MIN})')
        return missing

    # ── 액션 ────────────────────────────────────────────────────────────
    def _actions_ok(self):
        """
        액션 서버가 그래프에 올라왔는지 — 서비스 3종의 존재로 판정한다.

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
    """
    빈 문자열 자리표시자를 걷어낸다.

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
