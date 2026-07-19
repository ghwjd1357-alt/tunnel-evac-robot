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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_share = get_package_share_directory('tunnel_sim')
    nav2_share = get_package_share_directory('nav2_bringup')
    gui = LaunchConfiguration('gui')
    localization = LaunchConfiguration('localization')
    # ★ S2-1 보완 (07-19): nav2 파라미터도 인자화 — make_map(mapping 탐사)이
    #   allow_unknown 오버레이를 절대경로로 넘길 수 있게. 기본값 = 기존 동작 동일.
    #   PathJoinSubstitution 은 os.path.join 의미 — 절대경로 인자가 오면 그대로 쓴다.
    nav2_params = PathJoinSubstitution(
        [pkg_share, 'config', LaunchConfiguration('nav2_params')])

    # ① 로봇 + 월드 + slam_toolbox (localization:=true 면 저장 지도로 위치추정만 — §18)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'slam.launch.py')
        ),
        launch_arguments={'gui': gui, 'localization': localization,
                          # 쌍굴 지원 (07-07) — 기본값이면 기존 T자 그대로
                          'world': LaunchConfiguration('world'),
                          'spawn_x': LaunchConfiguration('spawn_x'),
                          'spawn_y': LaunchConfiguration('spawn_y'),
                          'localization_params':
                              LaunchConfiguration('localization_params')}.items(),
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
        # --- 쌍굴 지원 인자 (07-07). 기본값 = 기존 T자 동작과 동일 ---
        DeclareLaunchArgument('world', default_value='tunnel.world',
                              description='worlds/ 안의 월드 파일명'),
        DeclareLaunchArgument('spawn_x', default_value='-12',
                              description='로봇 스폰 world x'),
        DeclareLaunchArgument('spawn_y', default_value='0',
                              description='로봇 스폰 world y'),
        DeclareLaunchArgument('nav2_params', default_value='nav2_params.yaml',
                              description='config/ 안의 Nav2 파라미터 파일명 또는 절대경로'),
        DeclareLaunchArgument('localization_params',
                              default_value='slam_params_localization.yaml',
                              description='config/ 안의 localization 파라미터 파일명'),
        slam,
        navigation,
    ])
