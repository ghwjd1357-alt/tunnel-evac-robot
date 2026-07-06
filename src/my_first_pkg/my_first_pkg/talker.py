#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
talker.py — 메시지를 '발행(publish)'하는 노드.

[이 노드가 하는 일]
  'chatter' 라는 토픽 채널에 0.5초마다 "Hello ROS2: N" 문자열을 방송한다.
  라디오 방송국이라고 생각하면 된다. 듣는 사람(listener)이 없어도 계속 방송한다.

[핵심 ROS2 개념 대응]
  - Node      : MinimalPublisher 클래스 자체
  - Topic     : 'chatter'
  - Message   : std_msgs/String (문자열 양식)
  - Publish   : self.publisher_.publish(msg)
"""

import rclpy                       # ROS2 파이썬 핵심 라이브러리
from rclpy.node import Node        # 모든 노드의 부모가 되는 Node 클래스
from std_msgs.msg import String    # 우리가 보낼 메시지 타입(문자열)


class MinimalPublisher(Node):
    """Node를 상속받아 '발행 기능'을 가진 나만의 노드를 정의한다."""

    def __init__(self):
        # 부모(Node)에게 이 노드의 이름을 'minimal_publisher'로 등록
        super().__init__('minimal_publisher')

        # ── 파라미터 선언 ─────────────────────────────────────────
        #   declare_parameter('이름', 기본값)
        #   → 밖(런치파일/명령줄)에서 값을 안 주면 이 기본값이 쓰인다.
        self.declare_parameter('publish_period', 0.5)        # 발행 주기(초)
        self.declare_parameter('message_prefix', 'Hello ROS2')  # 메시지 앞글자
        # ──────────────────────────────────────────────────────────

        # 발행기(publisher) 생성:
        #   String  타입 메시지를
        #   'chatter' 토픽으로
        #   10       = QoS(큐 크기). 처리 못한 메시지를 최대 10개까지 쌓아둠
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # 파라미터 값을 읽어와 타이머 주기로 사용
        #   get_parameter('이름').value  →  실제 값 꺼내기
        timer_period = self.get_parameter('publish_period').value  # 초
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # 메시지 번호 카운터
        self.count = 0

    def timer_callback(self):
        """타이머가 0.5초마다 호출하는 함수 — 여기서 실제 발행이 일어남."""
        prefix = self.get_parameter('message_prefix').value  # 파라미터 값 읽기
        msg = String()                              # 빈 메시지 양식 생성
        msg.data = f'{prefix}: {self.count}'        # 앞글자 + 번호로 내용 채움
        self.publisher_.publish(msg)                # 토픽에 발행(방송)!

        # 터미널에 로그 출력 (print 대신 ROS2 표준 로거 사용 권장)
        self.get_logger().info(f'발행: "{msg.data}"')
        self.count += 1


def main(args=None):
    """노드 실행 진입점."""
    rclpy.init(args=args)              # ROS2 통신 시스템 초기화
    node = MinimalPublisher()          # 노드 객체 생성
    try:
        rclpy.spin(node)               # 노드를 계속 살려둠(타이머가 콜백 반복 호출)
    except KeyboardInterrupt:
        pass                           # Ctrl+C 누르면 조용히 종료
    finally:
        node.destroy_node()            # 노드 정리
        rclpy.shutdown()               # ROS2 시스템 종료


if __name__ == '__main__':
    main()
