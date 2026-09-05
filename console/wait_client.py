"""브라우저가 rosbridge 에 붙을 때까지 기다린다.

촬영용이다. bag 을 명령 실행 시점에 틀면, 주소를 입력하고 페이지가 뜨는 사이에
로봇이 이미 지나가 버려 **지도와 카메라 영상이 어긋난다**. rosbridge 는 클라이언트
수를 `/client_count` 로 알려주므로, 그 값이 1 이상이 되는 순간 = 관제 화면이 실제로
열린 순간에 맞춰 bag 을 시작할 수 있다.
"""
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class Wait(Node):
    """접속 수가 **늘어나는** 순간을 기다린다.

    처음엔 `> 0` 으로 판정했는데, 직전 테이크의 관제 탭이 남아 있으면 그것이
    이미 1 로 잡혀 **bag 이 곧바로 시작**됐다(09-04). 남은 탭을 매번 닫게 하는
    것은 촬영 중에 잊기 쉬우므로, 기준선을 잡고 그보다 늘어날 때만 반응한다.
    새로고침은 끊겼다 붙으므로 이 방식으로도 정상 감지된다.
    """

    def __init__(self):
        super().__init__('wait_client')
        self.hit = False
        self.base = None
        self.create_subscription(Int32, '/client_count', self.cb, 10)

    def cb(self, msg):
        if self.base is None:
            self.base = msg.data          # 남아 있던 탭 수 = 기준선
            return
        if msg.data > self.base:
            self.hit = True
        elif msg.data < self.base:
            self.base = msg.data          # 탭이 닫혔으면 기준선을 내린다


def main():
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
    rclpy.init()
    node = Wait()
    t0 = node.get_clock().now().nanoseconds / 1e9
    while rclpy.ok() and not node.hit:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.get_clock().now().nanoseconds / 1e9 - t0 > timeout:
            print('TIMEOUT', flush=True)
            sys.exit(1)
    print('CONNECTED', flush=True)


main()
