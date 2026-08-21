#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
real_bringup.launch.py — 실차 전체 스택을 한 줄로 (R6~R7).

실행:
  # R6 (자율주행 최초 — 미션 없이 goal 하나만 줘 본다)
  ros2 launch tunnel_bringup real_bringup.launch.py map_file:=<R5에서 만든 posegraph 경로>

  # R7 (미션 상태머신까지)
  ros2 launch tunnel_bringup real_bringup.launch.py map_file:=<...> \
      mission:=true waypoints_file:=<실터널 좌표 yaml 경로>

  ⚠ `map_file` 은 기본값이 없다(필수 인자). 지도 없이 자율주행을 시작하는 경로를
    아예 만들지 않기 위해서다 — 빠뜨리면 런치가 즉시 거부한다.
    확장자는 빼고 준다 (slam_toolbox 가 .posegraph/.data 를 알아서 붙인다).

[켜는 순서 — ★ 이 파일의 존재 이유]
  ┌ 0단계  기동 전 오설정 검사 (지도 파일·미션 좌표) — 하나라도 걸리면 아무것도 안 뜬다
  ├ 1단계  robot_state_publisher · 라이다 · micro-ROS agent · EKF
  │        └ 게이트 A: /scan·/odom·/imu/data·/odometry/filtered 가 흐르고
  │                    base_footprint→lidar_link, odom→base_footprint TF 가 섰나
  ├ 2단계  slam_toolbox (localization — 저장 지도로 위치추정)
  │        └ 게이트 B: map→odom TF 가 '지금도 갱신되고' 있나
  ├ 3단계  Nav2 (navigation_launch)
  │        └ 게이트 C: lifecycle 노드 5종이 ACTIVE 이고 /navigate_to_pose 가 떴나
  └ 4단계  mission_node (mission:=true 일 때만)

  ★ 각 단계는 **앞 단계가 실제로 살아난 것이 관측된 뒤에만** 뜬다.
    시뮬 런치처럼 "8초 뒤 Nav2, 12초 뒤 미션" 같은 고정 시간 지연을 쓰지 않는다.
    실차는 Jetson 부팅 상태·USB 인식 순서·micro-ROS 연결·라이다 스핀업이 매번 달라
    고정 시간이 성립하지 않기 때문이다. 짧으면 TF 없는 상태로 Nav2 가 굳고,
    길면 매번 헛기다린다. 조건 판정은 tunnel_bringup/readiness_gate.py 가 한다.

  ★ 게이트가 제한시간 안에 조건을 못 채우면 **런치 전체를 내린다**(fail-closed).
    반쪽짜리 스택(예: 라이다 없이 Nav2)으로 주행을 시작하지 않기 위해서다.

[게이트가 프로세스가 아니라 '토픽·TF'를 보는 이유]
  누가 발행하든 상관없이 조건만 보므로, 센서 대신 `ros2 bag play` 로 같은 토픽을
  흘려도 그대로 동작한다 (R3 bag 으로 상위 스택을 미리 돌려볼 수 있다).
  그때는 micro_ros:=false lidar:=false 로 실물 드라이버만 끄면 된다.

[이 런치가 하지 않는 것]
  · 지도 제작 → real_mapping.launch.py (R5)
  · EKF 단독 검증 → config/ekf_real.yaml 머리말의 R4 명령 (bag replay)

⚠⚠ 주행 전 확인: urdf/robot_real.urdf 의 라이다·IMU **장착 오프셋은 아직 미실측**이다.
   그대로 R5 이상을 돌리면 스캔이 로봇 엉뚱한 자리에 붙은 채로 지도가 그려진다.
   확인: grep -rn "TODO: " src/tunnel_bringup/
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, Shutdown)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from tunnel_bringup.launch_util import make_gate, when_ready


def check_misconfig(context):
    """
    기동 **전에** 잡을 수 있는 오설정을 본다. 문제가 있으면 종료 액션을 돌려준다.

    ★ 이 검사가 통과해야만 launch_setup 이 노드를 하나라도 만든다. "일단 띄우고 곧 내린다"가
      아니라 **아무것도 안 뜬다** — 실차에서 micro-ROS agent 나 라이다가 잠깐이라도
      뜨는 것은 불필요한 위험이다.

    검사 ① 지도 파일 존재
      ★ 추측이 아니라 **실측으로 필요성이 드러나서** 넣었다 (2026-07-29, 없는 경로 실행):
        slam_toolbox 는 지도 로드에 실패해도 ERROR 두 줄만 남기고 **계속 살아서
        map→odom TF 를 발행한다.** 즉 아래 게이트 B(map→odom 신선도)는 그대로 통과한다.
        결과적으로 경로 오타 하나가 "아무것도 없는 빈 지도 위에서 위치추정 중"이라는
        상태를 조용히 만든다 — 로봇은 자기가 어디 있는지 안다고 믿고 주행한다.
        TF 로는 이 실패를 구분할 수 없으므로 파일 존재를 직접 본다.
        slam_toolbox 는 `<map_file>.posegraph` 와 `<map_file>.data` 를 함께 읽는다.

    검사 ② mission:=true 인데 좌표 파일이 없거나 잘못됨
      기본 좌표 파일로 조용히 대체되면 실차가 **시뮬 터널 좌표**로 달린다 —
      벽을 향해 출발하는 것과 같다. 자동 대체 없이 거부한다.
    """
    raw = LaunchConfiguration('map_file').perform(context)
    if raw.endswith('.posegraph') or raw.endswith('.data'):
        return [
            LogInfo(msg=f'❌ map_file 은 확장자를 빼고 준다 (받은 값: {raw}). '
                        f'slam_toolbox 가 .posegraph/.data 를 알아서 붙인다.'),
            Shutdown(reason='map_file must be given without extension'),
        ]
    absent = [p for p in (raw + '.posegraph', raw + '.data') if not os.path.isfile(p)]
    if absent:
        return [
            LogInfo(msg=f'❌ 지도 파일이 없다: {", ".join(absent)} — 기동하지 않는다. '
                        f'R5 에서 만든 posegraph 경로를 확인할 것 '
                        f'(slam_toolbox 는 이 실패를 경고만 하고 빈 지도로 계속 돈다).'),
            Shutdown(reason='map file not found'),
        ]

    if _mission_on(context):
        waypoints = LaunchConfiguration('waypoints_file').perform(context)
        if not waypoints:
            return [
                LogInfo(msg='❌ mission:=true 인데 waypoints_file 이 비었다. '
                            '실터널 좌표 yaml 의 전체 경로를 주고 다시 실행할 것 '
                            '(기본 파일로 대체하지 않는다 — 시뮬 좌표 주행 사고 방지).'),
                Shutdown(reason='mission:=true requires waypoints_file'),
            ]
        if not os.path.isfile(waypoints):
            return [
                LogInfo(msg=f'❌ waypoints_file 이 없다: {waypoints}'),
                Shutdown(reason='waypoints_file not found'),
            ]
    return []


def _mission_on(context):
    return LaunchConfiguration('mission').perform(context).lower() == 'true'


def launch_setup(context, *_):
    """오설정 검사를 통과했을 때만 실제 스택을 구성한다."""
    blocked = check_misconfig(context)
    if blocked:
        return blocked

    pkg_share = get_package_share_directory('tunnel_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')

    banner = LogInfo(
        msg='⚠ robot_real.urdf 의 라이다·IMU 장착 오프셋은 미실측 TODO 다. '
            '실측 반영 전에는 지도·주행 결과를 신뢰하지 말 것 '
            '(확인: grep -rn "TODO: " src/tunnel_bringup/).')

    # ══════════════════════════════════════════════════════════════════
    # 1단계 — 몸(TF) · 센서 · 구동부 · EKF
    # ══════════════════════════════════════════════════════════════════
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', LaunchConfiguration('urdf')])
    # URDF 를 '실행 시점'에 읽는다(cat). 파이썬 open() 으로 읽으면 런치 파싱 시점에
    # 파일명이 고정돼 urdf:= 인자가 무시된다. value_type=str 은 "이 출력은 YAML 로
    # 해석하지 말고 통짜 문자열로 취급하라"는 표시 (XML 을 YAML 로 파싱하다 깨지는 것 방지).
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
            # ★ int 로 변환해서 넘긴다. 런치 인자는 전부 문자열이라 그대로 주면
            #   드라이버가 선언한 int 파라미터와 타입이 안 맞아 기동에 실패한다.
            'serial_baudrate': ParameterValue(LaunchConfiguration('lidar_baud'), value_type=int),
            # ★ URDF 의 lidar_link 와 이름이 정확히 같아야 한다. 어긋나면 costmap 이
            #   스캔을 "TF 없는 프레임"으로 버리고, 증상은 "장애물이 안 보인다"로 나온다.
            'frame_id': 'lidar_link',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'use_sim_time': False,
        }],
    )

    # micro-ROS agent = Teensy(모터·엔코더·BNO055)와 ROS2 를 잇는 다리.
    # 이게 떠야 /odom·/imu/data 가 들어오고 /cmd_vel 이 나간다.
    # ⚠ micro_ros_agent 는 Jetson 에서 소스 빌드해 둬야 한다 (R0 준비물).
    #   설치돼 있지 않으면 런치가 "package not found" 로 실패한다.
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        condition=IfCondition(LaunchConfiguration('micro_ros')),
        arguments=['serial', '--dev', LaunchConfiguration('serial_dev'),
                   '-b', LaunchConfiguration('serial_baud')],
    )

    # 🔴 /odom → (가드) → /odom_guarded → EKF. 가드가 EKF 보다 먼저 선다.
    #   `/odom` 이 가끔 부분만 쓰인 메시지를 내고 그것 하나가 EKF 를 영구 NaN 으로
    #   만든다(08-14 지도 두 세션이 같은 이유로 무너졌다). 근거·전제조건·재개방 조건
    #   = `MASTER_PLAN §7` 예약 41. ⚠ 끄는 스위치를 두지 않는다 — 안 뜨면
    #   `/odom_guarded` 가 조용히 비어 EKF 가 odom 없이 돈다.
    odom_guard = ExecuteProcess(
        cmd=['python3', '-u', LaunchConfiguration('odom_guard')],
        name='odom_guard',
        output='screen',
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',        # ekf_real.yaml 최상단 키와 일치해야 파라미터가 붙는다
        output='screen',
        # 🔵 08-21 — 파일명을 인자로 뺐다. 그전엔 여기 하드코딩이라 EKF 를 바꿔 보려면
        #   정본 파일을 직접 고치는 수밖에 없었다 — 되돌리기가 없는 방식이다.
        #   `nav2_params` 와 같은 방식이다: 기본값은 그대로이니 부르는 쪽은 안 바뀐다.
        parameters=[PathJoinSubstitution(
            [pkg_share, 'config', LaunchConfiguration('ekf_params')])],
    )

    # ── 게이트 A: 센서가 '흐르고' EKF 가 TF 를 세웠는가 ────────────────
    #   · /odometry/filtered 를 요구하는 이유: EKF 의 **출력**이라 이게 흐르면 EKF 가
    #     살아서 융합 중이라는 뜻이다 (입력 /odom 만 보면 EKF 가 죽어도 통과한다).
    #   · tf_fresh=0 인 이유: base_footprint→lidar_link 는 URDF 고정 joint 라 한 번만
    #     발행되는 정적 TF 다. 신선도를 요구하면 영원히 통과 못 한다.
    #     EKF 의 살아있음은 위 토픽이 이미 보장한다.
    #   · 제한 180s: Jetson 부팅 직후 USB 인식 + micro-ROS 세션 수립 + 라이다 스핀업.
    #     TODO: R0 실측 후 확정 (실제 소요를 재고 조인다).
    gate_sensors = make_gate(
        'sensors', timeout=180.0,
        topics=['/scan', '/odom', '/imu/data', '/odometry/filtered'],
        tf=['base_footprint:lidar_link', 'odom:base_footprint'],
        tf_fresh=0.0,
    )

    # ── 카메라 depth 게이트 (08-21 신설 · `camera:=true` 일 때만) ─────────────
    # 🔴 왜 gate_sensors 에 토픽을 더하지 않고 게이트를 따로 두는가:
    #   gate_sensors 는 **미션이 성립하기 위한 최소 집합**이다. 카메라는 거기
    #   안 든다 — 카메라가 없어도 순찰·유도·추종은 전부 돈다. 최소 집합에
    #   섞으면 카메라 하나로 주행 전체가 못 뜬다.
    #   같은 이유로 `/thermal/*` 은 **어느 게이트에도 넣지 않는다**(역할 B 6차
    #   §2-b 요청 수용 — "열화상이 없다고 로봇 전체가 안 뜨면 안 된다").
    # ⚠ 실패하면 런치 전체를 내린다(when_ready 의 fail-closed). 카메라를 켜겠다고
    #   선언했는데 depth 가 안 오는 상태는 **반쪽 인지**라, 그대로 촬영하면
    #   "찍었는데 아무것도 안 나온" 테이크가 된다.
    # 🔴 08-21 §82.2 재현 반영 (동결 예외 아홉 번째) — 게이트는 **직렬**이어야 한다
    #   구판은 `camera_gate` 를 `when_ready(gate_sensors, [slam,...])` 와 **형제**로
    #   뒀다. 형제는 동시에 진행한다. 그래서 core 센서가 먼저 준비되면
    #   slam→Nav2→미션이 depth 보다 **먼저** 떴고, depth 가 끝내 없으면 90초 뒤에야
    #   전체가 내려갔다. 종국 fail-closed 는 돌지만 그 90초 동안 반쪽 인지 상태로
    #   미션이 goal 을 받을 수 있었다. 그리고 런북은 "게이트에서 멈춘다" 라고
    #   적혀 있어 촬영자가 준비됐다고 오판한다 — 문장으로는 못 막는 종류다.
    #
    #   그래서 camera:=true 면 사슬을 하나로 잇는다:
    #       gate_sensors ─▶ gate_camera ─▶ slam ─▶ gate_localized ─▶ Nav2 ─▶ 미션
    #   camera:=false 면 기존 사슬 그대로다 (gate_camera 자체를 안 만든다).
    gate_camera = make_gate('camera', timeout=90.0,
                            topics=['/camera/depth/image_raw'])

    # ══════════════════════════════════════════════════════════════════
    # 2단계 — 위치추정 (저장 지도)
    # ══════════════════════════════════════════════════════════════════
    slam = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',   # 운영 = localization 전용 실행파일
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', LaunchConfiguration('slam_params')]),
            # yaml 뒤에 오는 값이 이긴다 — 지도 경로만 런치 인자로 덮어쓴다.
            # (yaml 에 절대경로를 박으면 계정이 다른 Jetson 에서 확정적으로 실패한다)
            {'use_sim_time': False, 'map_file_name': LaunchConfiguration('map_file')},
        ],
    )

    # ── 게이트 B: map→odom 이 '지금도 갱신되고' 있는가 ─────────────────
    #   tf_fresh 를 켠다. map→odom 은 slam_toolbox 가 계속 발행하는 동적 TF 라
    #   "한 번 섰다"가 아니라 "살아 있다"를 봐야 한다 (프로세스가 떠 있어도 위치추정이
    #   멎으면 Nav2 는 옛 좌표 위에서 계획한다).
    #   ⚠ 이 게이트는 **지도 내용이 옳은지는 못 본다.** 실측(07-29): 지도 파일이 없어도
    #     slam_toolbox 는 계속 살아서 이 TF 를 발행하므로 여기는 통과한다. 그 실패는
    #     위 check_misconfig 가 기동 전에 잡는다 — 역할이 다른 두 검사다.
    gate_localized = make_gate('localized', timeout=90.0, tf=['map:odom'], tf_fresh=3.0)

    # ══════════════════════════════════════════════════════════════════
    # 3단계 — Nav2
    # ══════════════════════════════════════════════════════════════════
    #   map_server·amcl 없는 navigation_launch 를 쓴다 (위치추정은 slam_toolbox 담당).
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': PathJoinSubstitution(
                [pkg_share, 'config', LaunchConfiguration('nav2_params')]),
            'use_sim_time': 'false',
        }.items(),
    )

    # ── 게이트 C: Nav2 가 진짜로 goal 을 받을 수 있는가 ────────────────
    #   lifecycle 노드는 '프로세스가 떴다'와 'ACTIVE 다'가 다르다. 활성화 전에 goal 을
    #   보내면 거부되고, 미션은 그걸 FAULT 로 읽는다.
    #   waypoint_follower·smoother_server 를 목록에 넣지 않은 이유: 우리가 쓰지 않고,
    #   lifecycle_manager 는 하나라도 활성화에 실패하면 시퀀스를 멈추므로 아래 5개가
    #   ACTIVE 면 나머지도 ACTIVE 다.
    gate_nav2 = make_gate(
        'nav2', timeout=120.0,
        lifecycle=['/controller_server', '/planner_server', '/bt_navigator',
                   '/behavior_server', '/velocity_smoother'],
        actions=['/navigate_to_pose'],
    )

    # ══════════════════════════════════════════════════════════════════
    # 4단계 — 미션 상태머신 (R7). mission:=false 면 여기서 끝난다(R6).
    # ══════════════════════════════════════════════════════════════════
    if _mission_on(context):
        after_nav2 = [Node(
            package='mission_manager',
            executable='mission_node',
            name='mission_manager',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'waypoints_file': LaunchConfiguration('waypoints_file'),
            }],
        )]
        what = '미션 상태머신'
    else:
        after_nav2 = [LogInfo(
            msg='✅ Nav2 준비 완료 — 이제 goal 을 보내도 된다 (R6). '
                '미션까지 켜려면 mission:=true waypoints_file:=<경로>.')]
        what = '(미션 없음 — R6)'

    return [
        banner,
        robot_state_publisher,
        lidar,
        micro_ros_agent,
        odom_guard,
        ekf,
        gate_sensors,
        # camera:=true — depth 확인이 **선행 조건**이다 (§82.2)
        GroupAction(
            actions=[
                when_ready(gate_sensors, [gate_camera], '카메라 depth 스트림 확인'),
                when_ready(gate_camera, [slam, gate_localized],
                           '위치추정(slam_toolbox)'),
            ],
            condition=IfCondition(LaunchConfiguration('camera')),
        ),
        # camera:=false — 08-20 까지와 **같은 순서**다 (거동 불변)
        GroupAction(
            actions=[
                when_ready(gate_sensors, [slam, gate_localized],
                           '위치추정(slam_toolbox)'),
            ],
            condition=UnlessCondition(LaunchConfiguration('camera')),
        ),
        when_ready(gate_localized, [navigation, gate_nav2], 'Nav2'),
        when_ready(gate_nav2, after_nav2, what),
    ]


def generate_launch_description():
    # 인자 선언은 밖에 둔다 — `--show-args` 가 스택 구성 없이 목록을 보여줄 수 있게.
    args = [
        # ★ 기본값 없음 = 필수. 지도 없이 기동하는 경로를 아예 만들지 않는다.
        DeclareLaunchArgument(
            'map_file',
            description='R5 에서 만든 posegraph 경로 (확장자 제외). 필수 인자.'),
        DeclareLaunchArgument(
            'mission', default_value='false',
            description='true=미션 상태머신까지 기동(R7). R6 은 false 로 goal 만 준다.'),
        DeclareLaunchArgument(
            'waypoints_file', default_value='',
            description='실터널 좌표 yaml 의 전체 경로. mission:=true 면 필수.'),
        DeclareLaunchArgument(
            'urdf', default_value='robot_real.urdf',
            description='urdf/ 안의 URDF 파일명'),
        DeclareLaunchArgument(
            'nav2_params', default_value='nav2_params_real.yaml',
            description='config/ 안의 Nav2 파라미터 파일명'),
        DeclareLaunchArgument(
            'slam_params', default_value='slam_real_localization.yaml',
            description='config/ 안의 slam_toolbox 파라미터 파일명'),
        DeclareLaunchArgument(
            'ekf_params', default_value='ekf_real.yaml',
            description='config/ 안의 EKF 파라미터 파일명'),
        DeclareLaunchArgument(
            'lidar', default_value='true',
            description='false = 라이다 드라이버 미기동 (bag 으로 /scan 을 흘릴 때)'),
        DeclareLaunchArgument(
            'lidar_port', default_value='/dev/rplidar',
            # 🔴 08-21 동결 예외 **여덟 번째**로 `/dev/ttyUSB0` 에서 바꿨다.
            #   이유 = 열화상 ESP32 가 라이다와 **같은 CP2102(10c4:ea60)** 라
            #   `ttyUSB` 번호를 서로 뺏는다(역할 B 6차 §6). 번호가 바뀌면
            #   라이다 노드가 ESP32 를 열고, `/scan` 이 **에러 없이** 안 온다.
            #   그 상태는 지도·위치추정뿐 아니라 **사람 추종까지** 죽인다
            #   (SEARCH_BACK 판정이 전부 /scan 클러스터다).
            # ⚠ udev 규칙이 있어야 이 경로가 생긴다 — `tools/udev/99-rplidar.rules`
            #   (serial `78824f27…` 로 잠금 · docs/JETSON_SETUP.md §6-b).
            #   링크가 없으면 런치가 **크게 실패**한다 — 엉뚱한 장치를 조용히
            #   여는 것보다 낫다. 규칙이 유실됐으면 재설치하거나, 급하면
            #   `lidar_port:=/dev/ttyUSB0` 으로 되돌린다(그때는 번호를 눈으로 확인).
            description='RPLIDAR 시리얼 포트 (udev 고정 링크). '
                        '규칙 = tools/udev/99-rplidar.rules'),
        DeclareLaunchArgument(
            'camera', default_value='false',
            # 🔴 08-21 신설 (동결 예외 **여덟 번째**). true 면 depth 스트림을 **기동 게이트**에
            #   넣는다 — 역할 B 6차 §2-b 요구. 역할 B의 depth FATAL 이 미구현이라
            #   (6차 §2-c: "08-22 촬영 전까지 들어간다고 약속하지 않겠습니다")
            #   depth 가 죽었을 때 잡는 **자동 방어가 이 게이트 하나뿐**이다.
            # ⚠ 기본값이 false 인 이유 = 카메라는 08-21 오후에 붙는다. true 를
            #   기본으로 하면 그전까지 런치가 안 뜬다.
            # ⚠ 게이트는 **기동 시점만** 본다. 촬영 중 사망은 못 잡는다 —
            #   그건 역할 B 6차 §2-a 체크리스트(매 테이크 확인)가 담당한다.
            description='true = /camera/depth/image_raw 를 기동 게이트에 편입'),
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
            'odom_guard', default_value=os.path.expanduser(
                '~/ros2_ws/tools/odom_guard.py'),
            description='odom 가드 스크립트 경로 (저장소 위치가 다르면 넘겨서 바꾼다)'),
        DeclareLaunchArgument(
            'serial_baud', default_value='115200',
            description='micro-ROS 전송 속도. 08-02 확정 = 115200 (구동부 3차 회신 §1, '
                        '구값 921600 은 07-24 임시 기준이었다). Teensy 4.x USB CDC 는 '
                        '이 값이 명목값이고 실제 속도는 USB 가 정한다 — 산술상 115200bps '
                        '(≈11.5KB/s)로는 실측 46.5Hz 발행량(≈51KB/s)이 나올 수 없다'),
    ]
    return LaunchDescription([*args, OpaqueFunction(function=launch_setup)])
