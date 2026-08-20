#!/usr/bin/env python3
"""어댑터 전구간 검증 — `/detections`(가짜) → 어댑터 → `/alarm`.

실행:
    python3 tools/e2e_adapter.py

무엇을 판정하는가:
    합의사항 §9 **2단계**가 요구한 계약 왕복 전량이다 —
    정상 단일·다중 / 빈 배열 / 저신뢰 / NaN / 계약 밖 class / 오래된 stamp /
    미래 stamp / 빈 frame / 없는 frame / 다른 클래스 / 거리 클램프 / 격하 모드.

🔴 이 도구가 증명하지 않는 것:
    "불이 잘 보인다" 는 **증명하지 않는다.** 인식 성능은 역할 B의 실물로만
    판정한다. 여기서 보는 것은 계약 왕복과 우리 쪽 거부 경로뿐이다.

🔴 하네스가 세 번 깨졌던 자리 — 셸로 만들지 말 것:
    ① `/alarm` 은 VOLATILE 이라 늦게 붙은 구독자가 못 받는다
    ② `ros2 topic echo > file` 은 블록 버퍼링이라 kill 하면 버퍼가 날아간다
    ③ pkill 정리가 상위 셸까지 말려들었다 (exit 144)
    → 한 프로세스 안에서 rclpy 로 돌리면 셋이 동시에 사라진다.
"""

import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import StaticTransformBroadcaster

from perception_adapter.adapter_node import PerceptionAdapter
from perception_adapter.fake_detections import FakeDetections

# 카메라를 map 의 (10, 0, 0.3) 에 정면(+x) 으로 둔다.
#   → 카메라 앞 2 m 화재 = map (12, 0). 이 값이 판정 기준이다.
CAM_X, CAM_Z = 10.0, 0.3


class TfPub(Node):
    """map → camera_color_optical_frame 정적 TF.

    실차에선 SLAM(map→odom) + EKF(odom→base_footprint) + URDF(base→camera) 가
    이 사슬을 만든다. 여기선 사슬 전체를 한 변환으로 줄여 **어댑터만** 시험한다.
    """

    def __init__(self):
        super().__init__('e2e_tf')
        self._b = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'camera_color_optical_frame'
        t.transform.translation.x = CAM_X
        t.transform.translation.z = CAM_Z
        t.transform.rotation.w = 1.0
        self._b.sendTransform(t)


class AlarmSpy(Node):
    def __init__(self):
        super().__init__('e2e_spy')
        self.got = None
        self.create_subscription(PoseStamped, '/alarm', self._cb, 10)

    def _cb(self, m):
        if self.got is None:
            self.got = (m.pose.position.x, m.pose.position.y, m.header.frame_id)


def run_case(scenario, overrides, settle=1.5, budget=5.0):
    nodes = []
    try:
        tf = TfPub(); nodes.append(tf)
        ov = [Parameter('confirm_frames', value=3),
              Parameter('confirm_window_sec', value=3.0)] + overrides
        adapter = PerceptionAdapter(parameter_overrides=ov); nodes.append(adapter)
        spy = AlarmSpy(); nodes.append(spy)

        ex = SingleThreadedExecutor()
        for n in nodes:
            ex.add_node(n)

        # 🔴 구독 연결이 서기를 기다린다 — 이걸 안 하면 어댑터가 발행해도
        #    spy 가 아직 안 붙어 있어 "발행 안 됨"으로 잘못 읽는다.
        end = time.time() + settle
        while time.time() < end:
            ex.spin_once(timeout_sec=0.02)

        fake = FakeDetections(
            parameter_overrides=[Parameter('scenario', value=scenario)])
        nodes.append(fake)
        ex.add_node(fake)

        end = time.time() + budget
        while time.time() < end and spy.got is None:
            ex.spin_once(timeout_sec=0.02)
        return spy.got
    finally:
        for n in nodes:
            try:
                n.destroy_node()
            except Exception:
                pass


CASES = [
    # (시나리오, 기대 발행?, 파라미터, 기대 좌표 or None)
    ('fire',        True,  [], (12.0, 0.0)),
    ('multi',       True,  [], (12.6, 0.1)),      # conf 0.77 쪽을 골라야 한다
    ('fire_far',    True,  [], (15.0, 0.0)),      # 20m → max_range 5m 클램프
    ('empty',       False, [], None),
    ('low_conf',    False, [], None),
    ('nan',         False, [], None),
    ('bad_class',   False, [], None),
    ('stale',       False, [], None),
    ('future',      False, [], None),
    ('no_frame',    False, [], None),
    ('wrong_frame', False, [], None),
    ('smoke',       False, [], None),
    ('fire',        True,  [Parameter('use_fixed_range', value=True),
                            Parameter('fixed_range', value=2.0)], (12.0, 0.0)),
]


def main():
    rclpy.init()
    ok = bad = 0
    try:
        for i, (scen, expect, ov, want_xy) in enumerate(CASES):
            got = run_case(scen, ov)
            fired = got is not None
            tag = scen + (' [격하]' if ov else '')
            if fired != expect:
                print(f'  XX  {tag:16s} 기대={expect} 실제={fired}  <-- 불일치')
                bad += 1
                continue
            if fired and want_xy is not None:
                dx = abs(got[0] - want_xy[0]); dy = abs(got[1] - want_xy[1])
                if dx > 0.05 or dy > 0.05 or got[2] != 'map':
                    print(f'  XX  {tag:16s} 좌표 {got} != 기대 {want_xy}/map')
                    bad += 1
                    continue
            detail = f'({got[0]:.2f}, {got[1]:.2f}) {got[2]}' if fired else '발행 없음'
            print(f'  OK  {tag:16s} {detail}')
            ok += 1
    finally:
        rclpy.shutdown()
    print(f'\n{ok}/{ok + bad} 통과')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
