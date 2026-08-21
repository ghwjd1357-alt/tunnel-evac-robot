#!/usr/bin/env python3
"""가짜 `/detections` 퍼블리셔 — 역할 B 없이 어댑터를 끝까지 돌려보는 도구.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
왜 만드는가
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
합의사항 §9 **2단계**가 요구하는 것이 정확히 이것이다 —
*"가짜 인식 성능이나 가짜 map 좌표를 만들지 않는다. 다음 계약 왕복만 시험한다."*

    정상 단일·다중 탐지 / 같은 stamp·frame 의 빈 배열 / 저신뢰 탐지 제외 /
    빈 frame · 잘못된 optical frame / NaN·Inf·범위 밖 / 오래된 stamp / 발행 중단

🔴 **이 도구는 "불이 잘 보인다" 를 증명하지 않는다.** 계약 왕복만 본다.
   인식 성능은 역할 B의 실물로만 판정한다. 둘을 섞지 않는다.

역할 B도 깡통 퍼블리셔를 갖고 있다(9차 회신 §3-e 로 지참 요청). 합류 때
**양쪽 것을 다 돌린다** — 우리 것만 통과하면 우리 가정에 맞춘 것일 수 있다.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
쓰는 법
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    ros2 run perception_adapter fake_detections --ros-args -p scenario:=fire
    ros2 run perception_adapter fake_detections --ros-args -p scenario:=nan

시나리오 목록은 아래 SCENARIOS 주석을 볼 것. 런타임에 바꿀 수도 있다:
    ros2 param set /fake_detections scenario empty
"""


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from tunnel_interfaces.msg import Detection3D, Detection3DArray


# 시나리오 — 어댑터의 **거부 경로 하나씩**을 겨눈다.
SCENARIOS = (
    'fire',          # 정상. 확정되면 /alarm 이 나가야 한다
    'fire_far',      # 거리 20m — max_range 클램프가 걸려야 한다
    'multi',         # fire + person_ok 동시. 높은 conf 의 fire 를 골라야 한다
    'empty',         # 빈 배열 = 정상 미탐지. 🔴 절대 안 쏴야 한다
    'low_conf',      # conf 0.10 — min_confidence 밑. 안 쏴야 한다
    'nan',           # position NaN. 안 쏴야 한다
    'bad_class',     # 계약 열거 밖("human"). 경고 + 무시
    'stale',         # stamp 가 5초 과거. 안 쏴야 한다
    'future',        # stamp 가 5초 미래. 안 쏴야 한다
    'no_frame',      # frame_id 빈 문자열. 안 쏴야 한다
    'wrong_frame',   # TF 에 없는 frame. 변환 실패로 안 쏴야 한다
    'smoke',         # fire 가 아닌 클래스만. 안 쏴야 한다
    # ── 08-21 §82.3 추가: optical 축과 frame 잠금을 실제로 시험한다 ──
    'fire_left',        # optical x=-1 → map 에서 왼쪽(+y) 으로 나와야 한다
    'fire_right',       # optical x=+1 → map 에서 오른쪽(-y)
    'resolvable_frame',  # TF 에 **있는** 엉뚱한 frame(base_link). 거부해야 한다
    # ── 🆕 08-22: 사람 경로(`PROJECT_CONTEXT §4.1-b`). 로봇도 카메라도 역할 B 도
    #    없이 `/person_status`·`/victim` 전구간을 굴리기 위한 것이다. ──
    'person_ok',        # 서 있는 사람 → status=ok → 미션은 유도로 간다
    'person_fallen',    # 쓰러진 사람 → status=fallen + /victim → 미션은 신고로 간다
    'person_none',      # 사람 없음(빈 배열) → status=none. 🔴 stale 과 섞이면 안 된다
    'person_unknown',   # 자세 판정 실패 → status=unknown. 🔴 "괜찮다"로 접으면 안 된다
    'person_flicker',   # ok/fallen 이 프레임마다 뒤집힌다 — 디바운스가 살아 있는지
    'person_far_fallen',  # 쓰러진 사람이 8m 밖. 거리 문턱이 걸리는지
)


def mk(class_name, conf, x, y, z):
    d = Detection3D()
    d.class_name = class_name
    d.confidence = float(conf)
    d.position.x = float(x)
    d.position.y = float(y)
    d.position.z = float(z)
    # bbox 는 어댑터가 안 쓴다(우리는 3D position 만 소비한다). 형식만 채운다.
    d.bbox.x_offset = 100
    d.bbox.y_offset = 100
    d.bbox.width = 40
    d.bbox.height = 40
    return d


class FakeDetections(Node):

    def __init__(self, **kwargs):
        # ⚠ **kwargs 를 통과시키는 이유 — 검증 하네스가 한 프로세스 안에서
        #   `parameter_overrides=` 로 노드를 여러 번 만든다. 그게 없으면
        #   시나리오마다 프로세스를 따로 띄워야 하고, 그 순간 신호·버퍼링
        #   문제가 다시 생긴다(실제로 세 번 깨졌다 — tools/e2e_adapter.py 머리말).
        super().__init__('fake_detections', **kwargs)
        self.declare_parameter('scenario', 'fire')
        self.declare_parameter('topic', '/detections')
        self.declare_parameter('rate_hz', 10.0)
        # 🔴 실제 optical frame 이름은 역할 B 확인 사항이다(합의사항 §4.1 미확정).
        #    합류 때 실물 이름으로 바꾼다. 그때까지 우리 static TF 와 같은 이름을 쓴다.
        self.declare_parameter('frame_id', 'camera_color_optical_frame')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.pub = self.create_publisher(
            Detection3DArray, self.get_parameter('topic').value, qos)
        hz = self.get_parameter('rate_hz').value
        self.create_timer(1.0 / hz, self.tick)
        self.get_logger().info(
            f'가짜 퍼블리셔 기동 — scenario={self.get_parameter("scenario").value} '
            f'@ {hz} Hz. 🔴 계약 왕복 시험 전용이다(인식 성능 아님).')

    def tick(self):
        s = self.get_parameter('scenario').value
        frame = self.get_parameter('frame_id').value
        now = self.get_clock().now()

        msg = Detection3DArray()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = frame

        if s == 'fire':
            msg.detections = [mk('fire', 0.82, 0.0, 0.0, 2.0)]
        elif s == 'fire_far':
            msg.detections = [mk('fire', 0.82, 0.0, 0.0, 20.0)]
        elif s == 'multi':
            msg.detections = [mk('person_ok', 0.91, 0.0, 0.0, 1.2),
                              mk('fire', 0.55, -0.1, 0.0, 2.4),
                              mk('fire', 0.77, -0.1, 0.0, 2.6)]
        elif s == 'empty':
            msg.detections = []
        elif s == 'low_conf':
            msg.detections = [mk('fire', 0.10, 0.0, 0.0, 2.0)]
        elif s == 'nan':
            msg.detections = [mk('fire', 0.90, float('nan'), 0.0, 2.0)]
        elif s == 'bad_class':
            msg.detections = [mk('human', 0.90, 0.0, 0.0, 2.0)]
        elif s == 'stale':
            msg.detections = [mk('fire', 0.90, 0.0, 0.0, 2.0)]
            msg.header.stamp.sec = max(0, msg.header.stamp.sec - 5)
        elif s == 'future':
            msg.detections = [mk('fire', 0.90, 0.0, 0.0, 2.0)]
            msg.header.stamp.sec = msg.header.stamp.sec + 5
        elif s == 'no_frame':
            msg.header.frame_id = ''
            msg.detections = [mk('fire', 0.90, 0.0, 0.0, 2.0)]
        elif s == 'wrong_frame':
            msg.header.frame_id = 'frame_that_does_not_exist'
            msg.detections = [mk('fire', 0.90, 0.0, 0.0, 2.0)]
        # ── 🆕 08-22 사람 경로 ──────────────────────────────────────────
        elif s == 'person_ok':
            msg.detections = [mk('person_ok', 0.88, 0.0, 0.0, 2.2)]
        elif s == 'person_fallen':
            msg.detections = [mk('person_fallen', 0.86, 0.2, 0.0, 2.2)]
        elif s == 'person_none':
            # 🔴 `empty` 와 바이트로는 같다. 그런데 **이름이 다른 이유가 있다** —
            #   사람 경로에서 빈 배열은 "봤는데 없다"(none)이고, 발행이 끊긴 것은
            #   "못 봤다"(stale)다. 둘을 섞으면 아무도 없는 자리에서 유도가 시작되거나
            #   (none 을 stale 로 읽음) 센서가 죽은 채로 신고가 나간다(그 반대).
            #   시나리오 이름을 갈라 두면 시험이 어느 쪽을 겨눴는지 읽는 사람이 안다.
            msg.detections = []
        elif s == 'person_unknown':
            msg.detections = [mk('person_unknown', 0.71, 0.0, 0.0, 2.2)]
        elif s == 'person_flicker':
            # 🔴 한 프레임씩 뒤집는다. 디바운스가 없으면 미션이 유도↔신고를 왕복한다.
            self._flip = not getattr(self, '_flip', False)
            msg.detections = [mk('person_fallen' if self._flip else 'person_ok',
                                 0.90, 0.0, 0.0, 2.2)]
        elif s == 'person_far_fallen':
            msg.detections = [mk('person_fallen', 0.86, 0.0, 0.0, 8.0)]
        elif s == 'fire_left':
            # optical x=오른쪽 이므로 왼쪽은 음수. map 에서는 +y 로 나와야 한다.
            msg.detections = [mk('fire', 0.90, -1.0, 0.0, 2.0)]
        elif s == 'fire_right':
            msg.detections = [mk('fire', 0.90, 1.0, 0.0, 2.0)]
        elif s == 'resolvable_frame':
            # 🔴 §82.3 — TF 트리에 **존재하는** 엉뚱한 frame. 구판은 이걸 조용히
            #   map 으로 변환했다. 이제 expected_source_frame 이 거부해야 한다.
            msg.header.frame_id = 'base_link'
            msg.detections = [mk('fire', 0.90, 0.0, 0.0, 2.0)]
        elif s == 'smoke':
            msg.detections = [mk('smoke', 0.95, 0.0, 0.0, 2.0)]
        else:
            self.get_logger().error(
                f'알 수 없는 scenario "{s}". 가능한 값 = {SCENARIOS}',
                throttle_duration_sec=5.0)
            return

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeDetections()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
