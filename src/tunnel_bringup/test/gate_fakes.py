#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gate_fakes.py — `readiness_gate` 회귀용 '가짜 조건' 발생기 (로봇·Gazebo 불필요).

============================================================
[왜 필요한가]
  게이트가 보는 것은 프로세스가 아니라 **토픽·TF·lifecycle 서비스·액션**이다.
  그래서 실물이 없어도 그 네 가지만 흉내 내면 게이트를 그대로 시험할 수 있다.
  (이 성질은 원래 설계 의도이기도 하다 — 그래서 `ros2 bag play` 로도 상위 스택이 돈다.)

[이 파일이 만드는 것 — 모드별]
  sensors    : /scan · /odom · /imu/data (+옵션 /odometry/filtered) 를 micro-ROS 토픽
               계약대로 발행. **발행 주기와 그 근거는 아래 '주기 정본' 블록 한 곳에만**
               적는다 — 이 독스트링에 숫자를 다시 쓰지 않는다(갈라질 자리를 없앤다).
               --messages N 으로 **N건만 보내고 멈추는** 죽은 퍼블리셔도 흉내 낸다.
  lifecycle  : `<노드>/get_state` 서비스를 대본대로 응답. 대본 항목:
                 active   = ACTIVE(3) 응답
                 inactive = INACTIVE(2) 응답
                 hang     = **응답하지 않음** (서비스 이름은 그래프에 그대로 남는다)
               ★ 'active,hang' 이 2026-07-29 Codex P1 의 재현 입력이다 —
                 "한 번 ACTIVE 를 답한 직후 executor 가 멎은 노드".
               ★★ --drop-at N --drop-sec S = 서비스 **소실→복구** (2026-07-30 2차 P1).
                 'active,active,hang' + --drop-at 1 이면 "단절 전 1회 → 소실 → 복구 →
                 ACTIVE 1회 → 멎음". 게이트가 소실 경계에서 지난 세대 증거를 버리지
                 않으면 이 입력에서 통과해 버린다.
                 ⚠ 이 층에서 **재현할 수 없는 것**: '단절 전에 보낸 요청의 늦은 응답이
                   복구 뒤 실제로 도착하는' 순간. 서비스를 파괴하면 그 요청은 DDS 상에서
                   소멸한다. 그 순간은 1층(pytest, FakeFuture)이 증명하고, 여기서는
                   **관측 가능한 결과**(소실→복구→ACTIVE 1회 ⇒ rc 1)를 증명한다.
  action     : 액션 서버를 그래프에 올려 둔다 (goal 은 전부 거절 — 존재만 흉내).

[QoS 를 일부러 BEST_EFFORT 로 발행하는 이유]
  실차 /odom·/imu/data 는 micro-ROS 쪽이 BEST_EFFORT 예정이다(TEENSY 합의사항 §4.5).
  게이트가 RELIABLE 로 구독하면 매칭이 안 돼 '센서가 죽은 것처럼' 보인다.
  여기서 BEST_EFFORT 로 쏘면, 게이트 구독 QoS 가 잘못 바뀌는 순간 회귀가 실패한다.

[실행 — 하네스(tools/test_gate_regression.sh)가 부른다]
  python3 gate_fakes.py sensors --messages 0
  python3 gate_fakes.py lifecycle --node /planner_server:active,hang
  python3 gate_fakes.py action --name /navigate_to_pose
"""

import argparse
import math
import sys
import threading
import time

from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState

from nav2_msgs.action import NavigateToPose

from nav_msgs.msg import Odometry

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import Imu, LaserScan


BEST_EFFORT = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)


# ============================================================
# --- 주기 정본 시작 ---
#
# ★ 여기가 '가짜 센서가 몇 ms 마다 발행하는가'의 **유일한 자리**다.
#   주기 수치를 이 블록 밖(주석·독스트링·타이머 인자)에 다시 적지 않는다.
#   같은 사실을 두 곳에 적으면 언젠가 갈라지고, 실제로 갈라졌던 것이 이 묶음의 발단이다:
#   코드 50Hz(타이머) / 독스트링 100Hz / 구동부 실측 41.63Hz — **셋이 전부 달랐다.**
#   회귀(`test_gate_fakes_periods.py`)가 "이 블록 밖에 <숫자>Hz 표기가 0건"임을 기계로 지킨다.
#
# ★ 규약 — 무엇이 이 표의 대상인가: **가짜 '센서 입력'의 발행 주기만** 넣는다.
#   하네스 자신의 제어 타이머(FakeLifecycle 의 소실/복구 tick 등)는 실물이 만드는 입력이
#   아니라 시험 대본이므로 대상이 아니다. (열거를 좁히는 판단은 규약으로 고정한다.)
# ============================================================

# [/imu/data] BNO055 — 구동부 2차 회신 §11 **실측** (window 295 표본):
#   평균 24.02ms · min 18ms · max 30ms · σ 0.55ms  (= 41.63Hz)
#   구동부 코드 설정은 20000us(50Hz)인데 실제 루프가 18~30ms 로 도는 상태다.
#   ⚠ 우리가 받은 것은 이 **5개 요약통계뿐**이고 원자료(타임스탬프 열)는 못 받았다.
#     따라서 아래 시퀀스는 '그 5통계를 만족하는 한 대표'이지 실차의 실제 시간열이 아니다.
#     지터의 세부 파형은 **미확보**이며, 원자료를 받으면 이 시퀀스를 통째로 교체한다.
#   ⚠ 30ms 는 '상한'이 아니라 **7초 남짓한 창에서 관측된 최대**다. 더 긴 창에서 더 큰
#     간격이 나올 수 있다 — 재개방 조건은 `docs/REAL_ROBOT_VALUES.md §1`.
IMU_WINDOW = 295          # 실측 표본 수 — 시퀀스 길이를 여기 맞춘다
IMU_MEAN_US = 24_020      # 평균 24.02ms
IMU_MIN_US = 18_000       # 관측 최소
IMU_MAX_US = 30_000       # 관측 최대 ★ EKF 한 주기에 가장 가까이 붙는 값
IMU_SIGMA_US = 550        # 표준편차 0.55ms

# [왜 이런 모양이 되는가 — 추측이 아니라 위 4수치에서 나오는 산술적 귀결]
#   창 295 · σ 0.55 → 편차제곱합 = 295 x 0.55^2 = 89.24 ms^2.
#   그런데 극단 2개(18·30)만으로 6.02^2 + 5.98^2 = 72.00 ms^2 를 이미 써 버린다.
#   → 나머지 293 표본이 나눠 쓸 수 있는 것은 17.24 ms^2, 즉 rms 0.2425ms 뿐이다.
#   **즉 18·30 은 자주 나오는 값이 아니라 창당 한 번꼴의 이탈이고, 나머지는 잔잔하다.**
#   그래서 시퀀스는 '±0.242ms 지터 + 창당 1회의 18·30' 모양이다. 이 유도가 틀렸다면
#   회귀의 통계 대조가 즉시 깨진다(값을 손으로 맞춰 넣은 것이 아니다).
_IMU_JITTER_US = 242      # 기저 지터 진폭 — 위 유도의 rms 를 만든다
_IMU_TRIM_US = 40         # 표본 수가 홀수라 남는 한 칸 — 평균을 24.020ms 에 정확히 맞춘다
_IMU_MAX_AT = 98          # 극단값 위치. 창을 대략 3등분한 고정 좌표(근거 없는 임의값 = 규약)
_IMU_MIN_AT = 197

# [/scan] RPLIDAR C1 — 사양 10Hz. **실측 미확보** (우리 쪽 센서라 구동부 회신 대상이 아니었다).
SCAN_PERIOD_US = 100_000
SCAN_UNMEASURED = '실측 미확보 — RPLIDAR C1 사양값. R3 rosbag 에서 실측 후 교체'

# [/odom] 구동부 계약 50Hz (`TEENSY_실차연동_합의사항.md §4.5`). **실측 미확보** —
#   2차 회신 §10.3 토픽 목록에 `/odom` 이 아직 없어서 주기를 잴 대상 자체가 없었다.
ODOM_PERIOD_US = 20_000
ODOM_UNMEASURED = '실측 미확보 — 합의서 계약값. 인수 항목 1(통합 펌웨어) 수령 후 실측'

# [/odometry/filtered] EKF 출력 자리 — 게이트 단독 케이스에서 EKF 를 대신한다.
#   근거는 실측이 아니라 **우리 설정값**이다: `config/ekf_real.yaml` 의 frequency.
#   회귀가 그 YAML 을 직접 읽어 아래 값과 대조한다(두 자리가 갈라지지 않게).
EKF_FREQUENCY_HZ = 30.0
EKF_CONFIG_REL = 'config/ekf_real.yaml'


def _build_imu_periods():
    """실측 5통계를 재현하는 결정적 주기 열(us)을 만든다 — 난수 없음, 시드도 없음."""
    pairs = (IMU_WINDOW - 3) // 2          # 극단 2개 + 보정 1칸을 뺀 나머지를 ± 로 채운다
    jitter = [_IMU_JITTER_US, -_IMU_JITTER_US] * pairs + [_IMU_TRIM_US]
    rest = [IMU_MEAN_US + d for d in jitter]
    return tuple(
        rest[:_IMU_MAX_AT] + [IMU_MAX_US]
        + rest[_IMU_MAX_AT:_IMU_MIN_AT - 1] + [IMU_MIN_US]
        + rest[_IMU_MIN_AT - 1:]
    )


class PeriodPlan:
    """가짜 센서 한 종의 발행 주기 열 + 그 주기의 출처. **불변** (커서는 쓰는 쪽이 갖는다)."""

    def __init__(self, topic, periods_us, basis, unmeasured=None):
        self.topic = topic
        self.periods_us = tuple(periods_us)
        self.basis = basis              # 이 주기가 어디서 왔는가 (실측/계약/설정)
        self.unmeasured = unmeasured    # None 이 아니면 '실측 미확보' + 그 사유

    @property
    def initial_s(self):
        """타이머를 만들 때 줄 첫 주기(초)."""
        return self.periods_us[0] / 1e6

    @property
    def varies(self):
        """주기가 매번 바뀌는가 (= 콜백마다 타이머에 갈아 끼워야 하는가)."""
        return len(set(self.periods_us)) > 1

    def period_ns(self, index):
        """시퀀스의 index 번째 주기(ns). 끝나면 처음으로 돌아온다."""
        return self.periods_us[index % len(self.periods_us)] * 1000


SENSOR_PERIODS = {
    'scan': PeriodPlan('/scan', (SCAN_PERIOD_US,),
                       basis='RPLIDAR C1 사양', unmeasured=SCAN_UNMEASURED),
    'odom': PeriodPlan('/odom', (ODOM_PERIOD_US,),
                       basis='합의사항 §4.5 계약값', unmeasured=ODOM_UNMEASURED),
    'imu': PeriodPlan('/imu/data', _build_imu_periods(),
                      basis='구동부 2차 회신 §11 실측 (window 295)'),
    'filtered': PeriodPlan('/odometry/filtered', (int(round(1e6 / EKF_FREQUENCY_HZ)),),
                           basis=f'{EKF_CONFIG_REL} 의 frequency (실측 아님 — 우리 설정값)'),
}
# --- 주기 정본 끝 ---


class FakeSensors(Node):
    """계약대로 도는 가짜 센서. --messages N 이면 N건 뒤 발행을 멈춘다(죽은 퍼블리셔)."""

    def __init__(self, messages, with_filtered, delay=0.0):
        super().__init__('gate_fake_sensors')
        self.limit = messages          # 0 = 무제한
        self.sent = {}
        # ★ 퍼블리셔는 지금 만들고 **발행만 늦춘다.** 게이트는 그래프에서 타입을 찾아
        #   구독하므로 퍼블리셔가 먼저 있어야 하고, "1건만 보내고 죽는" 케이스는 그
        #   1건이 구독 뒤에 와야 한다. 안 그러면 구독 전에 흘려보내 '수신 0건'이 되고,
        #   시험하려던 것(1건 받고도 통과 못 함)이 아니라 엉뚱한 이유로 실패한다.
        self._go_at = time.monotonic() + delay
        self._imu_step = 0
        self.scan_pub = self.create_publisher(LaserScan, '/scan', BEST_EFFORT)
        self.odom_pub = self.create_publisher(Odometry, '/odom', BEST_EFFORT)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', BEST_EFFORT)
        # ★ 주기 리터럴을 여기 쓰지 않는다 — 전부 '주기 정본' 표에서 꺼낸다.
        self.create_timer(SENSOR_PERIODS['scan'].initial_s, self._scan)
        self.create_timer(SENSOR_PERIODS['odom'].initial_s, self._odom)
        self.imu_timer = self.create_timer(SENSOR_PERIODS['imu'].initial_s, self._imu)
        if with_filtered:
            # EKF 를 띄우지 않는 게이트 단독 케이스용 — EKF 출력 자리를 대신 채운다.
            self.filtered_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
            self.create_timer(SENSOR_PERIODS['filtered'].initial_s, self._filtered)

    def _budget(self, key):
        """이 토픽이 아직 발행해도 되는지 (한도 소진 = 죽은 퍼블리셔 흉내)."""
        if time.monotonic() < self._go_at:
            return False
        sent = self.sent.get(key, 0)
        if self.limit and sent >= self.limit:
            return False
        self.sent[key] = sent + 1
        return True

    def _stamp(self, msg, frame):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame
        return msg

    def _scan(self):
        """RPLIDAR C1 흉내 — 360빔, 전방 5m 벽."""
        if not self._budget('scan'):
            return
        msg = self._stamp(LaserScan(), 'lidar_link')
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = 2.0 * math.pi / 360.0
        msg.range_min = 0.1
        msg.range_max = 12.0
        msg.ranges = [5.0] * 360
        self.scan_pub.publish(msg)

    def _odom(self):
        """구동부 /odom 흉내 — covariance 는 0 이 아니어야 EKF 가 받아들인다."""
        if not self._budget('odom'):
            return
        msg = self._stamp(Odometry(), 'odom')
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.orientation.w = 1.0
        for i in (0, 7, 14, 21, 28, 35):
            msg.pose.covariance[i] = 0.05
            msg.twist.covariance[i] = 0.05
        self.odom_pub.publish(msg)

    def _advance_imu_period(self):
        """
        다음 발행 간격을 시퀀스에서 꺼내 타이머에 갈아 끼운다.

        ★ 한 칸 늦게 반영되는 것이 **정상이고, 그래서 결과가 맞는다.**
          rcl 은 콜백을 부르기 **전에** `next_call_time += 현재주기` 로 다음 만기를
          이미 확정한다. 그래서 k 번째 콜백에서 심은 값은 k+1 번째 간격을 지배한다.
          초기 주기를 seq[0] 으로 두고 k 번째 콜백이 seq[k] 를 심으면, 실제 발행
          간격 열이 정확히 seq[0], seq[1], seq[2] … 가 된다.
          이 순서는 추론이 아니라 **실측으로 확정**했고(rclpy Humble), 회귀는
          실제 `FakeSensors` 를 rclpy executor 로 돌려 **콜백 간격을 직접 관측**한다
          — 누가 rclpy 를 올려 순서가 바뀌면 그 회귀가 깨진다.
          (초판 회귀는 이 순서를 테스트 안 시뮬레이터로 재구현했다가 08-01 검토
           §21 에서 불승인됐다. 생산 배선을 통과하지 않는 검사는 게이트가 아니다.)
        ★ 예산(`_budget`) 검사보다 **먼저** 부른다. --delay·--messages 로 발행이
          멈춰도 스케줄 자체는 흐르게 해서, 주기 열이 발행 여부에 좌우되지 않게 한다.
        """
        self._imu_step += 1
        self.imu_timer.timer_period_ns = SENSOR_PERIODS['imu'].period_ns(self._imu_step)

    def _imu(self):
        """BNO055 흉내 — 주기는 실측 분포다 ('주기 정본' 블록의 IMU 항목)."""
        self._advance_imu_period()
        if not self._budget('imu'):
            return
        msg = self._stamp(Imu(), 'imu_link')
        msg.orientation.w = 1.0
        for i in (0, 4, 8):
            msg.orientation_covariance[i] = 0.01
            msg.angular_velocity_covariance[i] = 0.01
            msg.linear_acceleration_covariance[i] = 0.05
        self.imu_pub.publish(msg)

    def _filtered(self):
        """EKF 출력 자리."""
        if not self._budget('filtered'):
            return
        msg = self._stamp(Odometry(), 'odom')
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.orientation.w = 1.0
        self.filtered_pub.publish(msg)


class FakeLifecycle(Node):
    """
    `<노드>/get_state` 를 대본대로 응답한다 (대본이 떨어지면 마지막 항목 반복).

    ★ --drop-at / --drop-sec = **서비스 소실→복구** 흉내 (2026-07-30 Codex 2차 P1).
      모든 노드가 drop_at 회 답한 뒤 get_state 서비스를 **파괴**하고, drop_sec 초 뒤
      다시 만든다. 대본 순번은 이어진다 — 즉 `active,active,hang` + drop_at=1 이면
      "단절 전 1회 ACTIVE → 소실 → 복구 → ACTIVE 1회 → 이후 멎음" 이 된다.
      ⚠ drop_sec 은 **5초 이상** 줄 것. 게이트는 미충족 사유를 5초 주기로만 찍으므로,
        더 짧으면 소실이 로그에 안 남아 하네스가 "정말 관측됐는가"를 확인할 수 없다
        (확인 못 하는 케이스는 조용히 아무것도 시험하지 않는 케이스가 된다).
      ⚠ 이 층에서 **재현되지 않는 것**: 소실 순간에 날아가 있던 조회. 대본상 직전 응답이
        이미 수확된 뒤 서비스가 파괴되므로 in-flight 조회가 없고, 설령 있어도 파괴와 함께
        DDS 에서 소멸해 '늦은 응답'이 도착하지 않는다. 즉 이 두 케이스는 P1 **검출기가
        아니라 경계 가드**다 — 검출은 1층(pytest, FakeFuture)이 한다.
    """

    def __init__(self, specs, drop_at=0, drop_sec=0.0):
        super().__init__('gate_fake_lifecycle')
        self._group = ReentrantCallbackGroup()
        self._counts = {}
        self._stop = threading.Event()
        self._scripts = dict(specs)
        self._srv_handles = {}
        for name, script in specs:
            self._create_service_for(name)
            self.get_logger().info(f'가짜 lifecycle {name}/get_state — 대본 {script}')

        self._drop_at = drop_at
        self._drop_sec = drop_sec
        self._dropped = False
        self._restore_at = None
        if drop_at > 0:
            self.create_timer(0.1, self._drop_tick, callback_group=self._group)

    def _create_service_for(self, name):
        """노드 하나의 get_state 서비스를 만든다(복구 때도 이 경로로 재생성)."""
        srv = name if name.endswith('/get_state') else f'{name}/get_state'
        self._srv_handles[name] = self.create_service(
            GetState, srv,
            self._make_cb(name, self._scripts[name]), callback_group=self._group)

    def _drop_tick(self):
        """모든 노드가 drop_at 회 답하면 서비스를 파괴하고, drop_sec 뒤 되살린다."""
        now = time.monotonic()
        if not self._dropped:
            if all(self._counts.get(n, 0) >= self._drop_at for n in self._scripts):
                for name, srv in self._srv_handles.items():
                    self.destroy_service(srv)
                    self.get_logger().warn(f'{name}: get_state 파괴 — 서비스 소실 흉내')
                self._srv_handles = {}
                self._dropped = True
                self._restore_at = now + self._drop_sec
        elif self._restore_at is not None and now >= self._restore_at:
            for name in self._scripts:
                self._create_service_for(name)
                self.get_logger().info(f'{name}: get_state 재생성 — 서비스 복구')
            self._restore_at = None

    def _make_cb(self, name, script):
        """노드 하나의 응답 콜백을 만든다."""
        def _cb(_request, response):
            index = self._counts.get(name, 0)
            self._counts[name] = index + 1
            behavior = script[min(index, len(script) - 1)]
            if behavior == 'hang':
                # ★ 응답하지 않는다. 서비스 이름은 그래프에 그대로 남으므로,
                #   게이트 입장에서는 "엔드포인트는 있는데 답이 없다"가 된다.
                self.get_logger().warn(f'{name}: 요청 {index + 1} — 응답 정지(멎은 노드)')
                self._stop.wait()
                return response
            if behavior == 'active':
                response.current_state.id = State.PRIMARY_STATE_ACTIVE
                response.current_state.label = 'active'
            else:
                response.current_state.id = State.PRIMARY_STATE_INACTIVE
                response.current_state.label = 'inactive'
            self.get_logger().info(f'{name}: 요청 {index + 1} → {behavior}')
            return response
        return _cb


class FakeAction(Node):
    """액션 서버를 그래프에 올려 두기만 한다 (goal 은 전부 거절)."""

    def __init__(self, name):
        super().__init__('gate_fake_action')
        self._server = ActionServer(self, NavigateToPose, name, self._execute)
        self.get_logger().info(f'가짜 액션 서버 {name}')

    def _execute(self, goal_handle):
        """존재만 흉내 — 실제 주행은 하지 않는다."""
        goal_handle.abort()
        return NavigateToPose.Result()


def parse_spec(text):
    """`/planner_server:active,hang` → ('/planner_server', ['active', 'hang'])."""
    if ':' in text:
        name, script = text.split(':', 1)
    else:
        name, script = text, 'active'
    behaviors = [b.strip() for b in script.split(',') if b.strip()]
    for behavior in behaviors:
        if behavior not in ('active', 'inactive', 'hang'):
            raise ValueError(f'알 수 없는 대본 항목: {behavior}')
    return name, behaviors


def build_parser():
    """CLI 정의."""
    parser = argparse.ArgumentParser(description='readiness_gate 회귀용 가짜 조건 발생기')
    sub = parser.add_subparsers(dest='mode', required=True)

    sensors = sub.add_parser('sensors', help='/scan·/odom·/imu/data 발행')
    sensors.add_argument('--messages', type=int, default=0,
                         help='토픽마다 N건만 보내고 멈춘다 (0=무제한)')
    sensors.add_argument('--with-filtered', action='store_true',
                         help='/odometry/filtered 도 발행 (EKF 를 안 띄우는 케이스용)')
    sensors.add_argument('--delay', type=float, default=0.0,
                         help='퍼블리셔는 즉시 만들되 발행은 N초 뒤부터 (구독 뒤에 흘리기)')

    lifecycle = sub.add_parser('lifecycle', help='<노드>/get_state 서비스')
    lifecycle.add_argument('--node', action='append', required=True,
                           help='이름[:대본] (예: /planner_server:active,hang)')
    lifecycle.add_argument('--drop-at', type=int, default=0,
                           help='N회 답한 뒤 get_state 서비스를 파괴한다 (0=안 함)')
    lifecycle.add_argument('--drop-sec', type=float, default=6.0,
                           help='파괴 후 몇 초 뒤 재생성할지 (5초 이상 — 위 클래스 독스트링)')

    action = sub.add_parser('action', help='액션 서버 존재만 흉내')
    action.add_argument('--name', default='/navigate_to_pose')
    return parser


def main(argv=None):
    """모드에 맞는 가짜 노드를 띄우고 kill 될 때까지 돈다."""
    args = build_parser().parse_args(argv)
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=8)
    if args.mode == 'sensors':
        node = FakeSensors(args.messages, args.with_filtered, args.delay)
    elif args.mode == 'lifecycle':
        node = FakeLifecycle([parse_spec(spec) for spec in args.node],
                             args.drop_at, args.drop_sec)
    else:
        node = FakeAction(args.name)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 멎은 콜백이 스레드를 붙들고 있어도 프로세스는 확실히 끝나야 한다
        # (하네스가 다음 케이스로 못 넘어가는 것을 막는다).
        time.sleep(0.1)
        sys.stdout.flush()
    return 0


if __name__ == '__main__':
    sys.exit(main())
