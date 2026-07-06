#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
slam.launch.py — 터널 월드 + 로봇 + slam_toolbox 를 한 번에 (2E SLAM).

실행(GUI):       ros2 launch tunnel_sim slam.launch.py
실행(헤드리스):  ros2 launch tunnel_sim slam.launch.py gui:=false

[켜는 것]
  ① robot.launch.py  : Gazebo + 터널 + URDF 로봇 (재활용)
  ② slam_toolbox     : /scan + odom TF → 지도(/map) 작성 + map→odom TF

[지도 만드는 법]
  이 런치 켠 뒤, teleop 또는 자동주행 스크립트로 로봇을 터널 구석구석 운전.
  라이다가 벽을 훑는 만큼 지도가 채워짐. 다 돌면:
    ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/tunnel_map
  로 .pgm(그림) + .yaml(메타) 저장.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('tunnel_sim')
    gui = LaunchConfiguration('gui')
    # slam_params 인자 (07-05): 실험용 파라미터 파일 교체 가능.
    #   기본 = slam_params.yaml (시뮬 정본, odom 과신 튜닝)
    #   실차 리허설 = slam_params:=slam_params_realodom.yaml (오돔오차 대비 완화 튜닝)
    slam_params = PathJoinSubstitution([pkg_share, 'config', LaunchConfiguration('slam_params')])

    # ① 로봇 + 월드 (gui 인자 전달)
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'robot.launch.py')
        ),
        launch_arguments={'gui': gui}.items(),
    )

    # ② slam_toolbox 비동기 노드 (online async = 실시간 지도작성)
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='true=GUI, false=헤드리스'),
        DeclareLaunchArgument('slam_params', default_value='slam_params.yaml',
                              description='config/ 안의 slam 파라미터 파일명'),
        robot,
        slam,
    ])
