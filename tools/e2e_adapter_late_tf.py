#!/usr/bin/env python3
"""늦은 TF 전구간 검사 — **실제 entrypoint 형상**으로 돈다 (08-21 §85.2).

실행:
    python3 tools/e2e_adapter_late_tf.py

왜 이 파일이 따로 있나
----------------------
🔴 §84.1 을 고치면서 `spin_thread=True` 를 넣고 **41 ms 성공**이라고 커밋에 적었다.
그 검산은 `rclpy.spin(node)` 를 **안 돌린** 구성에서 바깥 스레드로 조회한 값이었다.
실제 `main()` 에서는 `rclpy.spin(node)` 가 노드를 리스너 executor 에서 도로 가져가
(`nodes=1 → 0`) 대기가 실효를 잃는다. **존재하지 않는 형상을 시험한 것이다**(§85.2).

그래서 이 검사는 형상을 흉내내지 않는다:

    · 어댑터를 **실제 `main()` 처럼** `rclpy.spin` 하는 별도 스레드에서 돌린다
    · `/detections` 를 **토픽으로** 넣는다 (`on_detections()` 직접 호출 금지)
    · TF 를 detection **뒤에** 보낸다 — 그것이 정상 비동기 도착 순서다

⚠ 이 검사가 증명하지 않는 것: Jetson 에서의 실제 지연 분포. `tf_wait_sec` 값의
  근거는 여전히 약하다(§84.1 완료판정 중 미이행 — 정본에 공개돼 있다).
"""

import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import TransformBroadcaster

from perception_adapter.adapter_node import PerceptionAdapter
from perception_adapter.fake_detections import mk

from tunnel_interfaces.msg import Detection3DArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=5)

CAM_X, CAM_Z = 10.0, 0.3
OPTICAL_Q = (-0.5, 0.5, -0.5, 0.5)


class Rig(Node):
    """탐지 발행 + TF 발행 + /alarm 수신 — 어댑터 **밖**에서 돈다."""

    def __init__(self):
        super().__init__('late_tf_rig')
        self.det = self.create_publisher(Detection3DArray, '/detections', QOS)
        self.tf = TransformBroadcaster(self)
        self.got = None
        self.create_subscription(PoseStamped, '/alarm', self._cb, 10)

    def _cb(self, m):
        if self.got is None:
            self.got = (m.pose.position.x, m.pose.position.y)

    def send_detection(self, stamp):
        msg = Detection3DArray()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.detections = [mk('fire', 0.9, 0.0, 0.0, 2.0)]
        self.det.publish(msg)

    def send_tf(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'camera_color_optical_frame'
        t.transform.translation.x = CAM_X
        t.transform.translation.z = CAM_Z
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = OPTICAL_Q
        self.tf.sendTransform(t)


def run_case(label, delay_sec, expect_fire, overrides=(), frames=6, budget=6.0):
    """detection 을 먼저 보내고 `delay_sec` 뒤에 같은 stamp TF 를 보낸다."""
    adapter = PerceptionAdapter(parameter_overrides=[
        Parameter('confirm_frames', value=3),
        Parameter('confirm_window_sec', value=10.0),
        *overrides])
    rig = Rig()
    # 🔴 실제 main() 과 같은 형상 — 어댑터를 **자기 executor 로 spin** 한다.
    #   ⚠ `rclpy.spin(node)` 은 기본으로 **전역 executor** 를 쓴다. 두 노드를
    #     각각 `rclpy.spin` 하면 같은 전역 executor 를 두 스레드가 물고,
    #     rig 의 콜백이 안 도는 일이 생긴다(첫 구현이 여기서 전부 FAIL 했다).
    #     노드마다 executor 를 명시해야 형상이 실제와 같아진다.
    ad_ex = SingleThreadedExecutor()
    ad_ex.add_node(adapter)
    rig_ex = SingleThreadedExecutor()
    rig_ex.add_node(rig)
    threading.Thread(target=ad_ex.spin, daemon=True).start()
    threading.Thread(target=rig_ex.spin, daemon=True).start()
    time.sleep(1.0)                       # discovery

    ok = False
    try:
        for i in range(frames):
            stamp = rig.get_clock().now().to_msg()
            rig.send_detection(stamp)
            if delay_sec is not None:
                time.sleep(delay_sec)
                rig.send_tf(stamp)
            time.sleep(0.12)
        end = time.time() + budget
        while time.time() < end and rig.got is None:
            time.sleep(0.02)
        fired = rig.got is not None
        ok = (fired == expect_fire)
        detail = f'{rig.got}' if fired else '발행 없음'
        print(f'  {"OK " if ok else "XX "} {label:34s} {detail}')
    finally:
        ad_ex.shutdown()
        rig_ex.shutdown()
        adapter.destroy_node()
        rig.destroy_node()
    return ok


def main():
    rclpy.init()
    ok = bad = 0
    try:
        cases = [
            # 🔴 핵심 — detection 뒤 30 ms 에 도착하는 정상 비동기 순서
            ('늦은 TF 30ms → 발행돼야 한다', 0.030, True, ()),
            ('늦은 TF 90ms → 발행돼야 한다', 0.090, True, ()),
            # tf_wait_sec 밖 — 후퇴 금지가 기본이므로 발행 0
            ('늦은 TF 300ms → 발행 0 (대기 상한 밖)', 0.300, False, ()),
            ('TF 영구 미도착 → 발행 0', None, False, ()),
            # 명시적 격하 모드에서만 최신 TF 후퇴
            ('TF 300ms + 후퇴 허용 → 발행', 0.300, True,
             (Parameter('allow_latest_tf_fallback', value=True),)),
        ]
        for label, delay, expect, ov in cases:
            if run_case(label, delay, expect, ov):
                ok += 1
            else:
                bad += 1
    finally:
        rclpy.shutdown()
    print(f'\n{ok}/{ok + bad} 통과')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
