#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
real_mapping.launch.py — 실터널 지도 제작 전용 (R5).

실행:
  ros2 launch tunnel_bringup real_mapping.launch.py
  # 다른 터미널에서 사람이 직접 운전한다 (저속!)
  ros2 run teleop_twist_keyboard teleop_twist_keyboard

  # 다 돌고 나서 posegraph 저장 (확장자 없이 준다)
  ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializeMap \
      "{filename: '<저장할 경로>/tunnel_real_localization'}"

[real_bringup 과 무엇이 다른가]
  · slam_toolbox 를 **mapping** 모드로 띄운다 (빈 지도에서 그려 나간다).
  · Nav2 와 미션을 띄우지 않는다. 지도 만들 때 자율주행은 오히려 방해가 된다
    (아직 지도가 없으니 계획기가 믿을 근거가 없다).
  · 그래서 지도 파일 인자도 없다.

[teleop 을 런치에 넣지 않은 이유]
  teleop_twist_keyboard 는 키보드 입력(stdin)을 받아야 하는데, 런치가 띄운
  프로세스는 터미널을 직접 잡지 못해 키가 먹지 않는다. 사람이 별도 터미널에서 켠다.

[주행 요령 — R5 통과 판정: 벽이 직선으로 서고 루프가 닫힐 것]
  · 아주 느리게. 급회전 금지 (스캔매칭이 따라오지 못하면 지도가 접힌다).
  · 갈림길·모서리에서는 잠깐 멈춰 스캔이 쌓이게 한다.
  · 왕복해서 같은 곳을 두 번 지난다 (루프 닫힘 = 누적 오차 교정의 기회).
  · ★ 출발 지점을 바닥에 마킹해 둘 것. 운영(R6~)에서 이 지점이 지도 원점이 되며,
    같은 자리에서 기동해야 slam_real_localization.yaml 의 map_start_pose 전제가 성립한다.

⚠⚠ 주행 전 확인: urdf/robot_real.urdf 의 라이다 장착 오프셋이 미실측이면
   스캔이 로봇 엉뚱한 자리에 붙은 채로 지도가 그려진다 — **지도 자체가 못 쓰게 된다.**
   확인: grep -rn "TODO: " src/tunnel_bringup/
"""

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from tunnel_bringup.launch_util import make_gate, when_ready


def generate_launch_description():

    pkg_share = get_package_share_directory('tunnel_bringup')

    args = [
        DeclareLaunchArgument(
            'urdf', default_value='robot_real.urdf',
            description='urdf/ 안의 URDF 파일명'),
        DeclareLaunchArgument(
            'slam_params', default_value='slam_real_mapping.yaml',
            description='config/ 안의 slam_toolbox 파라미터 파일명 (mapping 용)'),
        DeclareLaunchArgument(
            'lidar', default_value='true',
            description='false = 라이다 드라이버 미기동 (bag 으로 /scan 을 흘릴 때)'),
        DeclareLaunchArgument(
            'lidar_port', default_value='/dev/ttyUSB0',
            description='RPLIDAR 시리얼 포트. TODO: R0 에서 udev 고정 링크로 교체'),
        DeclareLaunchArgument(
            'lidar_baud', default_value='460800',
            description='RPLIDAR C1 보레이트 (sllidar_ros2 C1 기본값)'),
        DeclareLaunchArgument(
            'micro_ros', default_value='true',
            description='false = micro-ROS agent 미기동 (bag 으로 /odom·/imu 를 흘릴 때)'),
        DeclareLaunchArgument(
            'serial_dev', default_value='/dev/teensy_drive',
            description='Teensy 시리얼 장치 (TEENSY 합의사항 §4.5 udev 링크)'),
        DeclareLaunchArgument(
            'serial_baud', default_value='115200',
            description='micro-ROS 전송 속도. 08-02 확정 = 115200 (구동부 3차 회신 §1, '
                        '구값 921600 은 07-24 임시 기준이었다). Teensy 4.x USB CDC 는 '
                        '이 값이 명목값이고 실제 속도는 USB 가 정한다 — 산술상 115200bps '
                        '(≈11.5KB/s)로는 실측 46.5Hz 발행량(≈51KB/s)이 나올 수 없다'),
    ]

    banner = LogInfo(
        msg='⚠ 지도 제작 전 확인: robot_real.urdf 의 라이다 장착 오프셋이 미실측이면 '
            '지도 자체를 못 쓰게 된다 (grep -rn "TODO: " src/tunnel_bringup/).')

    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', LaunchConfiguration('urdf')])
    robot_description = ParameterValue(Command(['cat ', urdf_path]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
    )

    lidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('lidar')),
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': ParameterValue(LaunchConfiguration('lidar_baud'), value_type=int),
            'frame_id': 'lidar_link',      # URDF 의 link 이름과 정확히 일치
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'use_sim_time': False,
        }],
    )

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        condition=IfCondition(LaunchConfiguration('micro_ros')),
        arguments=['serial', '--dev', LaunchConfiguration('serial_dev'),
                   '-b', LaunchConfiguration('serial_baud')],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[PathJoinSubstitution([pkg_share, 'config', 'ekf_real.yaml'])],
    )

    # 게이트: 센서가 흐르고 EKF 가 TF 를 세운 뒤에야 SLAM 을 띄운다.
    # SLAM 을 먼저 띄우면 odom TF 가 없는 동안 스캔이 버려지고, 그 구간이
    # 지도 앞부분에 구멍으로 남는다.
    gate_sensors = make_gate(
        'sensors', timeout=180.0,
        topics=['/scan', '/odom', '/imu/data', '/odometry/filtered'],
        tf=['base_footprint:lidar_link', 'odom:base_footprint'],
        tf_fresh=0.0,
    )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',   # 지도 제작 = async 실행파일
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', LaunchConfiguration('slam_params')]),
            {'use_sim_time': False},
        ],
    )

    ready = LogInfo(msg='✅ SLAM 기동 — 이제 별도 터미널에서 teleop 으로 저속 주행할 것.')

    return LaunchDescription([
        *args,
        banner,
        robot_state_publisher,
        lidar,
        micro_ros_agent,
        ekf,
        gate_sensors,
        when_ready(gate_sensors, [slam, ready], '지도 제작(slam_toolbox)'),
    ])
