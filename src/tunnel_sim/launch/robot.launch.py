#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robot.launch.py — 터널 월드 + URDF 로봇을 '한 줄'로 띄우는 런치파일 (2D).

실행:  ros2 launch tunnel_sim robot.launch.py

[이 런치가 켜는 4개 노드]
  ① gazebo            : 터널 월드(tunnel.world) 띄움  (tunnel.launch.py 재활용)
  ② robot_state_publisher : URDF 읽어 link/joint → TF 발행 (바퀴·라이다 좌표관계)
  ③ spawn_entity      : URDF 로봇을 Gazebo 월드 안에 '소환(spawn)'
  ④ ekf_node (2E)     : /odom + /imu/data 융합 → odom→base_footprint TF 발행
                        ★ diff_drive 의 TF 발행을 껐으므로(URDF), EKF 없인 TF 체인이
                          끊긴다 → 로봇과 한 몸으로 여기서 항상 같이 켠다.

[기존 tunnel.launch.py 와 차이]
  tunnel.launch.py = 월드만 (static box_robot 시절). 보존용으로 남겨둠.
  robot.launch.py  = 월드 + 움직이는 URDF 로봇. ★ 2D부터 이걸 쓴다.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_share = get_package_share_directory('tunnel_sim')

    # gui 인자(기본 true). 헤드리스 검증/서버용으로 false 가능: ... robot.launch.py gui:=false
    gui = LaunchConfiguration('gui')

    # --- urdf 인자 (2026-07-05): 어떤 URDF 로 로봇을 띄울지 선택.
    #     기본 = robot.urdf (정상). 오돔 오차 주입 실험 = urdf:=robot_odomerr.urdf
    #     상위 런치(slam_nav2 등)에서 ros2 launch 인자로 주면 여기까지 그대로 전달된다. ---
    urdf_arg = DeclareLaunchArgument(
        'urdf', default_value='robot.urdf',
        description='urdf/ 폴더 안의 URDF 파일명 (예: robot_odomerr.urdf = 오돔오차 실험용)')

    # --- world / spawn 인자 (07-07 쌍굴 추가): 기본값 = 기존 T자 동작 그대로 ---
    #     쌍굴은 world:=tunnel_twin.world spawn_x:=-17 (스폰 = 1번 굴 서쪽 끝 = map(0,0))
    world_arg = DeclareLaunchArgument(
        'world', default_value='tunnel.world',
        description='worlds/ 안의 월드 파일명 (tunnel_twin.world=쌍굴)')
    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x', default_value='-12',
        description='로봇 스폰 world x (이 지점이 map 원점이 됨)')
    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y', default_value='0',
        description='로봇 스폰 world y')

    # --- URDF 를 '실행 시점'에 읽기 ---
    # 예전: 파이썬 open()으로 즉시 읽음 → 런치파일 파싱 시점에 파일명이 고정돼 인자 사용 불가.
    # 지금: Command('cat 경로') substitution → ros2 launch 가 실제 실행될 때 cat 으로 읽음
    #       → urdf:= 인자가 반영된다. ParameterValue(value_type=str)는 "이 출력은 YAML 로
    #       해석하지 말고 통짜 문자열로 취급하라"는 표시 (XML 을 YAML 파싱하다 깨지는 것 방지).
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', LaunchConfiguration('urdf')])
    robot_description = ParameterValue(Command(['cat ', urdf_path]), value_type=str)

    # --- ① Gazebo + 터널 월드 : 기존 tunnel.launch.py 를 그대로 포함(재활용) ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'tunnel.launch.py')
        ),
        launch_arguments={'gui': gui,
                          'world': LaunchConfiguration('world')}.items(),
    )

    # --- ② robot_state_publisher : URDF → TF.
    #     'robot_description' 파라미터에 URDF 문자열을 넣어주면, 이 노드가
    #     base_footprint→base_link→바퀴/라이다 의 좌표 관계를 /tf 로 계속 발행. ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,   # ★ 시뮬 시계 사용 (Gazebo /clock 과 동기화)
        }],
    )

    # --- ③ spawn_entity : 위 robot_description 토픽을 읽어 Gazebo 안에 로봇 생성.
    #     -x -y -z : 처음 놓을 위치 (메인 복도 서쪽 입구, 2C box_robot 자리와 동일 x=-12). ---
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_tunnel_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',   # robot_state_publisher 가 올린 토픽에서 읽음
            '-entity', 'tunnel_robot',       # Gazebo 안에서 부를 이름
            # 스폰 위치 = spawn_x/spawn_y 인자 (기본 -12,0 = T자 서쪽 입구, 기존 그대로).
            # z 는 살짝 띄워 바닥 뚫림 방지.
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', '0.15',
        ],
    )

    # --- ④ EKF (robot_localization) : 휠 /odom + /imu/data 융합 (2E 복도 드리프트 보강).
    #     출력 = /odometry/filtered + odom→base_footprint TF.
    #     설정 상세는 config/ekf.yaml 주석 참조. ---
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',       # ekf.yaml 최상단 키와 이름 일치해야 파라미터가 붙는다
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml')],
    )

    return LaunchDescription([
        urdf_arg,
        world_arg,
        spawn_x_arg,
        spawn_y_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        ekf,
    ])
