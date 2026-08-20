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

# REP-103 optical → base 회전 rpy(-π/2, 0, -π/2) 의 사원수 (x, y, z, w).
#   launch/adapter.launch.py 의 static_transform_publisher 인자와 같은 회전이다.
#   검산은 tools/test_optical_frame.py 가 한다 (합성 좌표 3방향).
OPTICAL_Q = (-0.5, 0.5, -0.5, 0.5)


class TfPub(Node):
    """map → camera_color_optical_frame 정적 TF + map → base_link.

    실차에선 SLAM(map→odom) + EKF(odom→base_footprint) + URDF(base→camera) 가
    이 사슬을 만든다. 여기선 사슬 전체를 한 변환으로 줄여 **어댑터만** 시험한다.

    🔴 08-21 §82.3 — 구판은 회전을 **단위 사원수**로 뒀다. 그래서 좌표 13/13 이
    통과해도 optical 축(x=오른쪽·y=아래·z=앞)을 한 번도 시험하지 않았다.
    이제 REP-103 optical 회전(rpy = -π/2, 0, -π/2)을 실제로 건다:

        optical (x_r, y_d, z_f)  →  map ( z_f, -x_r, -y_d ) + 카메라 위치

    그리고 `base_link` 도 트리에 올린다 — **존재하지만 계약이 아닌** frame 이
    거부되는지 보려면 변환 가능해야 하기 때문이다(없으면 TF 실패와 구별 불가).
    """

    def __init__(self):
        super().__init__('e2e_tf')
        self._b = StaticTransformBroadcaster(self)
        now = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'map'
        t.child_frame_id = 'camera_color_optical_frame'
        t.transform.translation.x = CAM_X
        t.transform.translation.z = CAM_Z
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = OPTICAL_Q

        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'map'
        t2.child_frame_id = 'base_link'
        t2.transform.translation.x = CAM_X
        t2.transform.rotation.w = 1.0

        self._b.sendTransform([t, t2])


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
    # ── 08-21 §82.3 — optical 축을 실제로 시험한다 ──────────────────────
    ('fire_left',   True,  [], (12.0, 1.0)),      # optical x=-1 → map +y
    ('fire_right',  True,  [], (12.0, -1.0)),     # optical x=+1 → map -y
    # 🔴 TF 에 **있는** 엉뚱한 frame — 구판은 조용히 변환해 (12,0) 을 냈다
    ('resolvable_frame', False, [], None),
    # 🔵 검사를 끄면(비권장) 같은 입력이 통과한다 — 잠금이 실제로 일하는지 대조.
    #   🔴 그리고 **답이 틀린다**: base_link 는 회전이 없으므로 optical 규약의
    #   z=2(앞) 가 여기서는 '위쪽 2m' 로 읽혀 map (10,0) 이 나온다. 화재는 2m
    #   앞인데 좌표는 로봇 발밑이다. 이것이 frame 잠금이 필요한 이유 그 자체다.
    ('resolvable_frame', True,
     [Parameter('expected_source_frame', value='')], (10.0, 0.0)),
]


def main():
    rclpy.init()
    ok = bad = 0
    try:
        for i, (scen, expect, ov, want_xy) in enumerate(CASES):
            got = run_case(scen, ov)
            fired = got is not None
            names = [q.name for q in ov]
            if 'use_fixed_range' in names:
                tag = scen + ' [격하]'
            elif 'expected_source_frame' in names:
                tag = scen + ' [잠금끔]'
            else:
                tag = scen
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
