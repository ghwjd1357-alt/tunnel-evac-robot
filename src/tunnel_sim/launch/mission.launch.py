#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mission.launch.py — 미션 전체 스택을 '한 줄'로 (07-06 신설).

실행:  ros2 launch tunnel_sim mission.launch.py             (GUI)
       ros2 launch tunnel_sim mission.launch.py gui:=false  (헤드리스)
       ros2 launch tunnel_sim mission.launch.py follower:=false  (추종자 없이)

[왜 만들었나]
  기존엔 slam_nav2 런치 + mission_node + fake_follower 를 터미널 3개에서
  각각 켜야 했고, 자작 노드마다 `--ros-args -p use_sim_time:=true` 를
  손으로 붙여야 했다 (빼먹으면 stamp·타이머가 현실시간 — 단골 함정).
  → 이 런치가 전부 켜고 use_sim_time 도 항상 챙긴다. 함정이 구조적으로 소멸.

[왜 tunnel_sim 에 있나 (mission_manager 아님)]
  이 런치는 Gazebo 월드(시뮬)까지 켜는 시뮬 전용 오케스트레이션.
  mission_manager 는 실차 이식 대상이라 시뮬 의존성을 넣지 않는다 —
  실차 미션 런치는 실차 브링업에 맞춰 따로 만든다.

[켜는 것]
  ① slam_nav2.launch.py : Gazebo+터널+로봇+EKF+SLAM+Nav2 (기존 스택 그대로)
  ② mission_node        : 임무 상태머신 (12초 지연 — Nav2 활성화 전에 goal 을
                          보내면 거부→FAULT 로 시작하는 §13.3 레이스 회피.
                          거부돼도 자동재시도가 흡수하지만 로그가 지저분해짐)
  ③ fake_follower       : 가짜 추종자 (follower:=false 로 끌 수 있음)

화재 알람·놓침 재현 명령은 CLAUDE.md '미션 로직 (2F)' 절 참조.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('tunnel_sim')
    mission_share = get_package_share_directory('mission_manager')
    gui = LaunchConfiguration('gui')

    # ① 시뮬 + SLAM + Nav2 전체 스택 (검증된 기존 런치 재사용)
    #    ★ 미션은 localization 기본 true (§18): "지도는 미리 있고 그 위에서 운영"
    #      = 실제 시나리오와 동일 + 실행마다 지도 품질 달라지는 비결정성 제거.
    #      지도 재제작이 필요하면: bash tools/make_map.sh (mapping 주행+posegraph 저장)
    #      옛 방식(라이브 SLAM)으로 돌리려면: localization:=false
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'slam_nav2.launch.py')
        ),
        launch_arguments={'gui': gui,
                          'localization': LaunchConfiguration('localization'),
                          # 쌍굴 지원 (07-07) — 기본값이면 기존 T자 그대로
                          'world': LaunchConfiguration('world'),
                          'spawn_x': LaunchConfiguration('spawn_x'),
                          'spawn_y': LaunchConfiguration('spawn_y'),
                          'localization_params':
                              LaunchConfiguration('localization_params')}.items(),
    )

    # ② 미션 상태머신 (두뇌) — Nav2 가 자리 잡을 시간을 준 뒤 기동
    mission = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='mission_manager',
                executable='mission_node',
                name='mission_manager',
                output='screen',
                parameters=[{
                    'use_sim_time': True,   # ★ 런치가 항상 챙김
                    # 미션 좌표 파일 — 월드마다 다름 (waypoints 인자, 07-07).
                    # 기본 waypoints.yaml = T자. 쌍굴 = waypoints_twin.yaml
                    'waypoints_file': PathJoinSubstitution(
                        [mission_share, 'config',
                         LaunchConfiguration('waypoints')]),
                }],
            )
        ],
    )

    # ③ 가짜 추종자 (시뮬 시험 도구 — follower:=false 로 제외 가능)
    follower = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='tunnel_sim',
                executable='fake_follower',
                name='fake_follower',
                output='screen',
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(LaunchConfiguration('follower')),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='true=GUI, false=헤드리스'),
        DeclareLaunchArgument('follower', default_value='true',
                              description='false 면 가짜 추종자 없이 (단독 탈출 시나리오 등)'),
        DeclareLaunchArgument('localization', default_value='true',
                              description='true=저장 지도로 운영(표준), false=라이브 SLAM'),
        # --- 쌍굴 지원 인자 (07-07). 기본값 = 기존 T자 동작과 동일.
        #     쌍굴 세트는 mission_twin.launch.py 가 한 번에 세팅해줌 ---
        DeclareLaunchArgument('world', default_value='tunnel.world',
                              description='worlds/ 안의 월드 파일명'),
        DeclareLaunchArgument('spawn_x', default_value='-12',
                              description='로봇 스폰 world x'),
        DeclareLaunchArgument('spawn_y', default_value='0',
                              description='로봇 스폰 world y'),
        DeclareLaunchArgument('localization_params',
                              default_value='slam_params_localization.yaml',
                              description='config/ 안의 localization 파라미터 파일명'),
        DeclareLaunchArgument('waypoints', default_value='waypoints.yaml',
                              description='mission_manager config/ 안의 좌표 파일명'),
        stack,
        mission,
        follower,
    ])
