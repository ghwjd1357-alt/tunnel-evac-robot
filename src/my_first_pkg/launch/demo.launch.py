#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
demo.launch.py — talker 와 listener 노드를 '한 번에' 실행하는 런치파일.

[이 런치파일이 하는 일]
  터미널 한 개에서 'ros2 launch my_first_pkg demo.launch.py' 하면
  talker(발행) + listener(구독) 두 노드가 동시에 켜진다.
  → 예전처럼 터미널을 2개 열 필요가 없다.

[핵심 규칙]
  ros2 launch 는 이 파일에서 'generate_launch_description' 라는
  이름의 함수를 찾아 호출한다. 이 함수는 반드시
  LaunchDescription 객체를 return 해야 한다.
"""

from launch import LaunchDescription          # 실행 목록을 담는 '장바구니'
from launch_ros.actions import Node           # '노드 하나 실행' 주문서


def generate_launch_description():
    """ros2 launch 가 찾는 약속된 함수. 실행할 노드 목록을 돌려준다."""

    # talker 노드 실행 주문서
    talker_node = Node(
        package='my_first_pkg',   # 어느 패키지에서
        executable='talker',      # setup.py entry_points 에 등록한 실행이름
        name='my_talker',         # (선택) 노드 이름을 새로 지정
        output='screen',          # 노드 로그를 이 터미널에 출력
    )

    # listener 노드 실행 주문서
    listener_node = Node(
        package='my_first_pkg',
        executable='listener',
        name='my_listener',
        output='screen',
    )

    # 장바구니에 두 노드를 담아서 돌려준다
    return LaunchDescription([
        talker_node,
        listener_node,
    ])
