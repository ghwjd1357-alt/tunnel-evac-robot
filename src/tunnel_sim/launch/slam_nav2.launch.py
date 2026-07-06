#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
slam_nav2.launch.py — 라이브 SLAM + Nav2 동시 (2E 자율주행 검증용).

실행:  ros2 launch tunnel_sim slam_nav2.launch.py            (GUI)
       ros2 launch tunnel_sim slam_nav2.launch.py gui:=false (헤드리스)

[저장맵+amcl 대신 이 방식을 쓰는 이유]
  - slam_toolbox 가 실시간으로 지도(/map) + map→odom TF 를 발행 → 로봇 위치가 항상 정확
    (amcl 초기위치 수동설정·저장맵 좌표 불일치 문제 없음).
  - Nav2 는 'navigation_launch'(planner·controller·bt_navigator만, map_server·amcl 제외)로 띄움.
  - 좌표가 명확: 시작 시 map(0,0)=spawn, map+x=로봇 정면(메인복도). 목표 주기 쉬움.

[켜는 것]
  ① slam.launch.py        : Gazebo + 터널 + 로봇 + slam_toolbox
  ② navigation_launch.py  : Nav2 네비게이션 스택 (저장맵 불필요)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_share = get_package_share_directory('tunnel_sim')
    nav2_share = get_package_share_directory('nav2_bringup')
    gui = LaunchConfiguration('gui')
    localization = LaunchConfiguration('localization')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # ① 로봇 + 월드 + slam_toolbox (localization:=true 면 저장 지도로 위치추정만 — §18)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'slam.launch.py')
        ),
        launch_arguments={'gui': gui, 'localization': localization}.items(),
    )

    # ② Nav2 네비게이션 스택 (map_server·amcl 없이). slam 이 map→odom 줄 시간 벌려고 8초 지연.
    navigation = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_share, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'params_file': nav2_params,
                    'use_sim_time': 'true',
                }.items(),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='true=GUI, false=헤드리스'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='true=저장 지도로 위치추정만 (운영), false=지도작성'),
        slam,
        navigation,
    ])
