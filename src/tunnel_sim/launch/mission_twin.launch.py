#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mission_twin.launch.py — ★ 쌍굴(twin-bore) 터널 미션 전체를 '한 줄'로 (07-07 신설).

실행:  ros2 launch tunnel_sim mission_twin.launch.py             (GUI — 천장 슬롯으로 내려다보기)
       ros2 launch tunnel_sim mission_twin.launch.py gui:=false  (헤드리스)

[정체]
  mission.launch.py(정본 오케스트레이션)를 그대로 포함하되, 쌍굴 세트 4개
  (월드·스폰위치·localization 파라미터·waypoints)만 미리 채워주는 얇은 래퍼.
  → 인자 4개를 매번 손으로 맞출 필요 없음 + 빼먹거나 짝이 안 맞는 사고 방지.
  기존 T자 미션은 그대로: ros2 launch tunnel_sim mission.launch.py

[쌍굴 좌표 치트시트 (map 프레임 = 스폰 world(-17,0) 기준, map = world + 17)]
  1번 굴 중심선 y=0 (x: -3~37) / 2번 굴 y=10 / 피난통로 x = 7, 17, 27 (폭 2.5m)
  화재 테스트:  ros2 topic pub --times 2 -w 1 /alarm geometry_msgs/msg/PoseStamped \
                  "{header: {frame_id: map}, pose: {position: {x: 30.0, y: 0.0}}}"
  놓침/재개·관제(/mission_cmd)는 CLAUDE.md '미션 로직' 절과 동일.

[지도 재제작]  bash tools/make_map.sh twin   (maps/twin_localization.* 생성)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_share = get_package_share_directory('tunnel_sim')

    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'mission.launch.py')
        ),
        launch_arguments={
            # ↓ 사용자가 계속 고를 수 있는 것 3개는 그대로 통과
            'gui': LaunchConfiguration('gui'),
            'follower': LaunchConfiguration('follower'),
            'localization': LaunchConfiguration('localization'),
            # ↓ 쌍굴 세트 4개 (이 파일의 존재 이유)
            'world': 'tunnel_twin.world',
            'spawn_x': '-17',      # 1번 굴 서쪽 끝 → 여기가 map(0,0)
            'spawn_y': '0',
            'localization_params': 'slam_params_localization_twin.yaml',
            'waypoints': 'waypoints_twin.yaml',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='true=GUI, false=헤드리스'),
        DeclareLaunchArgument('follower', default_value='true',
                              description='false 면 가짜 추종자 없이'),
        DeclareLaunchArgument('localization', default_value='true',
                              description='true=저장 쌍굴 지도로 운영(표준), false=라이브 SLAM'),
        mission,
    ])
