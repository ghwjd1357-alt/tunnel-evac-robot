#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tunnel.launch.py — Gazebo + 터널 월드를 '한 줄'로 실행하는 런치파일.

실행:  ros2 launch tunnel_sim tunnel.launch.py

[핵심 아이디어]
  Gazebo 실행은 이미 gazebo_ros 패키지의 'gazebo.launch.py' 가 다 해준다.
  우리는 그걸 '불러와서(include)' world 파일 경로만 인자로 넘긴다.
  → 직접 Node 를 짜는 대신, 남이 만든 런치를 재활용 = IncludeLaunchDescription.
"""

import os
from ament_index_python.packages import get_package_share_directory  # 설치된 패키지 경로 찾기
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription  # 인자 선언 + 런치 포함
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution  # 런치 인자 값 참조
from launch.launch_description_sources import PythonLaunchDescriptionSource  # .py 런치 읽는 해석기


def generate_launch_description():

    # gui 인자: true=창 띄움(평소), false=헤드리스(창 없이 서버만, 무인검증·서버용).
    gui = LaunchConfiguration('gui')

    # ① 내 패키지(tunnel_sim)가 '설치된' 경로를 찾는다.
    #    colcon build 하면 worlds/*.world 가 install/.../share/tunnel_sim/worlds/ 로 복사됨.
    pkg_share = get_package_share_directory('tunnel_sim')
    # world 인자 (07-07): worlds/ 안의 파일명. 기본 = 기존 T자 터널 (동작 불변).
    #   쌍굴은 world:=tunnel_twin.world (mission_twin.launch.py 가 세팅해줌).
    #   경로 조립은 실행 시점 값이 필요하므로 os.path.join 이 아니라 substitution 으로.
    world_path = PathJoinSubstitution([pkg_share, 'worlds', LaunchConfiguration('world')])

    # ② gazebo_ros 패키지가 제공하는 표준 Gazebo 런치파일의 경로를 찾는다.
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    gazebo_launch_path = os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')

    # ③ 그 런치파일을 '포함'하면서, world 인자로 우리 터널 경로를 넘긴다.
    #    launch_arguments 는 '문자열 쌍'이어야 해서 값도 문자열로 준다.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={
            'world': world_path,   # ← gazebo.launch.py 가 받는 'world' 인자에 우리 월드 주입
            'verbose': 'true',     # 자세한 로그
            'gui': gui,            # ← true=gzclient(창) 켜짐 / false=헤드리스
        }.items(),                 # dict → (키,값) 쌍 목록으로 변환 (요구 형식)
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='true=GUI 창, false=헤드리스(서버만)'),
        DeclareLaunchArgument('world', default_value='tunnel.world',
                              description='worlds/ 안의 월드 파일명 (tunnel_twin.world=쌍굴)'),
        gazebo,
    ])
