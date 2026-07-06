#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
demo_advanced.launch.py — 런치 '심화' 3종 세트 데모.

  ① parameters : talker 에 발행주기/메시지앞글자를 주입
  ② remappings : 'chatter' 토픽 이름을 'my_topic' 으로 갈아끼움
  ③ argument   : 실행할 때 명령줄에서 값을 받음
      예) ros2 launch my_first_pkg demo_advanced.launch.py period:=1.0 prefix:=안녕

[핵심 규칙(복습)]
  ros2 launch 는 generate_launch_description() 를 찾아 호출하고,
  이 함수는 LaunchDescription 객체를 return 해야 한다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument      # ③ 명령줄 인자 '선언'
from launch.substitutions import LaunchConfiguration  # ③ 그 인자 '값 가져오기'
from launch_ros.actions import Node


def generate_launch_description():

    # ───────────────────────────────────────────────────────────
    # ③ launch argument 선언
    #    DeclareLaunchArgument('이름', default_value='기본값')
    #    → 'ros2 launch ... period:=1.0' 처럼 밖에서 받을 수 있게 등록
    # ───────────────────────────────────────────────────────────
    period_arg = DeclareLaunchArgument(
        'period', default_value='0.5',
        description='발행 주기(초)')
    prefix_arg = DeclareLaunchArgument(
        'prefix', default_value='Hello ROS2',
        description='메시지 앞글자')

    # 선언한 인자의 '현재 값'을 꺼내오는 핸들
    period = LaunchConfiguration('period')
    prefix = LaunchConfiguration('prefix')

    # ───────────────────────────────────────────────────────────
    # talker 노드 : ① 파라미터 주입 + ② 토픽 remap
    # ───────────────────────────────────────────────────────────
    talker_node = Node(
        package='my_first_pkg',
        executable='talker',
        name='my_talker',
        output='screen',
        # ① parameters: 노드 파라미터에 위에서 받은 argument 값을 연결
        parameters=[{
            'publish_period': period,   # talker.py 의 declare_parameter 이름과 일치!
            'message_prefix': prefix,
        }],
        # ② remappings: ('원래이름', '새이름') 튜플 리스트
        #    talker 가 'chatter' 로 발행 → 실제로는 'my_topic' 으로 나감
        remappings=[('chatter', 'my_topic')],
    )

    # listener 도 똑같이 remap 해줘야 같은 채널에서 들을 수 있다
    listener_node = Node(
        package='my_first_pkg',
        executable='listener',
        name='my_listener',
        output='screen',
        remappings=[('chatter', 'my_topic')],
    )

    return LaunchDescription([
        period_arg,    # ③ 선언한 인자들도 리스트에 담아야 동작
        prefix_arg,
        talker_node,
        listener_node,
    ])
