#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
listener.py — 메시지를 '구독(subscribe)'하는 노드.

[이 노드가 하는 일]
  talker가 'chatter' 토픽에 방송하는 문자열을 듣고, 받을 때마다 출력한다.
  라디오를 그 채널에 맞춰놓은 청취자라고 생각하면 된다.

[핵심 ROS2 개념 대응]
  - Subscribe       : create_subscription(...)
  - Callback        : 메시지가 도착할 때마다 자동 호출되는 함수(listener_callback)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """Node를 상속받아 '구독 기능'을 가진 노드를 정의한다."""

    def __init__(self):
        super().__init__('minimal_subscriber')

        # 구독기(subscription) 생성:
        #   String              타입 메시지를
        #   'chatter'           토픽에서 듣고
        #   self.listener_callback  메시지 도착 시 이 함수를 호출
        #   10                  QoS 큐 크기
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        # (사용하지 않는 변수 경고 방지용 — 구독 객체는 살아있어야 함)
        self.subscription

    def listener_callback(self, msg):
        """'chatter' 토픽에 메시지가 도착할 때마다 자동 호출됨."""
        self.get_logger().info(f'수신: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    try:
        rclpy.spin(node)               # 메시지를 계속 기다림
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
