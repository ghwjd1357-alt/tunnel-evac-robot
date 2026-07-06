#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
nav2.launch.py — 저장된 지도 + Nav2 자율주행 (2E, 스테이징 — 내일 실행 검증 예정).

전제: maps/tunnel_map.yaml 가 이미 저장돼 있어야 함 (slam.launch.py 로 매핑 후 map_saver).

실행(GUI):  ros2 launch tunnel_sim nav2.launch.py
실행(헤드):  ros2 launch tunnel_sim nav2.launch.py gui:=false

[켜는 것]
  ① robot.launch.py        : Gazebo + 터널 + URDF 로봇
  ② nav2_bringup(bringup_launch) : map_server+amcl+planner+controller+bt_navigator 일괄
  → RViz 에서 '2D Goal Pose' 찍으면 로봇이 경로 짜고 자율주행 (/cmd_vel 발행).

[주의] 아직 헤드리스 자율주행 검증은 안 함(범위='SLAM 지도까지'). 파일만 준비.
       localization=amcl 은 저장맵 기준. SLAM 동시(localization) 모드로 바꾸려면 별도 설정.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_share = get_package_share_directory('tunnel_sim')
    nav2_share = get_package_share_directory('nav2_bringup')
    gui = LaunchConfiguration('gui')

    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    map_yaml = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'maps', 'tunnel_map.yaml')

    # ① 로봇 + 월드
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'robot.launch.py')
        ),
        launch_arguments={'gui': gui}.items(),
    )

    # ② Nav2 일괄 bringup (저장맵 + 파라미터 주입)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'params_file': nav2_params,
            'use_sim_time': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='true=GUI, false=헤드리스'),
        robot,
        nav2,
    ])
