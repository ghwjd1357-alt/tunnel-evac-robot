"""어댑터 + 카메라 static TF 를 함께 띄운다.

🔴 왜 `tunnel_bringup` 이 아니라 여기 있나
   `src/tunnel_bringup/**` 은 동결 대상이라 파일을 새로 넣으려면 동결 예외를
   써야 한다. 어댑터는 **신규 패키지**라 동결 밖이다. 그래서 카메라 TF 도
   여기서 붙인다 — 이러면 `URDF 수정 금지` 항목도 건드리지 않는다.

🔴 카메라 위치는 **아직 안 쟀다**
   08-21 오후에 물리 장착한 뒤 줄자로 재서 아래 인자로 넘긴다. 기본값은
   "대충 앞쪽 위" 라는 뜻일 뿐 실측이 아니다. **기본값 그대로 쓰지 말 것.**

    ros2 launch perception_adapter adapter.launch.py \
        cam_x:=0.18 cam_z:=0.35 cam_yaw:=0.0

⚠ optical frame 규약 — ROS 의 카메라 optical frame 은 **z 가 앞**, x 가 오른쪽,
  y 가 아래다(REP-103). 로봇 base_link 는 **x 가 앞**이다. 그래서 두 프레임
  사이에는 고정 회전이 들어간다. 아래 `optical` 인자가 그것이다.
  🔴 이 회전을 빼먹으면 화재가 로봇 **옆이나 위**에 찍힌다 — 그리고 그 좌표는
  복도 밖이라 `mission_node` 가 조용히 거부한다. 08-21 합류 때 실물로 확인한다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('parent_frame', default_value='base_link',
                              description='카메라를 매다는 로봇 프레임'),
        DeclareLaunchArgument('camera_frame',
                              default_value='camera_color_optical_frame',
                              description='🔴 역할 B 확인 필요 — 실제 optical frame 이름'),
        DeclareLaunchArgument('cam_x', default_value='0.18',
                              description='🔴 미실측. 장착 후 줄자로 잰다 (m)'),
        DeclareLaunchArgument('cam_y', default_value='0.0'),
        DeclareLaunchArgument('cam_z', default_value='0.35',
                              description='🔴 미실측. 장착 후 줄자로 잰다 (m)'),
        DeclareLaunchArgument('cam_yaw', default_value='0.0',
                              description='카메라가 정면이 아니면 여기 (rad)'),
        DeclareLaunchArgument(
            'optical', default_value='true',
            description='true = REP-103 optical 회전(-90°,0,-90°)을 넣는다. '
                        '역할 B가 이미 광학 프레임을 발행하면 false'),
        DeclareLaunchArgument('trigger_class', default_value='fire'),
        DeclareLaunchArgument('min_confidence', default_value='0.40'),
        DeclareLaunchArgument('confirm_frames', default_value='5'),
        DeclareLaunchArgument('max_range', default_value='5.0'),
        DeclareLaunchArgument(
            'use_fixed_range', default_value='false',
            description='🔴 격하 모드 — depth 를 못 믿을 때만. 쓰면 기록에 남긴다'),
        DeclareLaunchArgument('fixed_range', default_value='2.0'),
    ]

    # base_link → camera optical frame.
    #   `--roll -1.5708 --pitch 0 --yaw -1.5708` 가 REP-103 optical 회전이다
    #   (x 앞 → z 앞). optical:=false 면 회전 없이 yaw 만 준다.
    tf_optical = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='camera_static_tf',
        arguments=[
            '--x', LaunchConfiguration('cam_x'),
            '--y', LaunchConfiguration('cam_y'),
            '--z', LaunchConfiguration('cam_z'),
            # 🔴 08-21 §82.3 — 구판은 여기 yaw 를 상수로 박아서 `cam_yaw` 가
            #   optical=true(기본값)에서 **아무 효과가 없었다.** 운용자가 값을
            #   넘겨도 좌표가 안 변한다 — 조용한 무시가 가장 나쁜 종류다.
            #   합성: Rz(θ)·Rz(-π/2)·Rx(-π/2) = Rz(θ-π/2)·Rx(-π/2)
            #   → roll 은 그대로 -π/2, yaw 만 θ-π/2 가 된다.
            '--roll', '-1.5707963', '--pitch', '0',
            '--yaw', PythonExpression(
                ['str(float("', LaunchConfiguration('cam_yaw'), '") - 1.5707963)']),
            '--frame-id', LaunchConfiguration('parent_frame'),
            '--child-frame-id', LaunchConfiguration('camera_frame'),
        ],
        condition=IfCondition(LaunchConfiguration('optical')),
    )
    tf_plain = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='camera_static_tf',
        arguments=[
            '--x', LaunchConfiguration('cam_x'),
            '--y', LaunchConfiguration('cam_y'),
            '--z', LaunchConfiguration('cam_z'),
            '--roll', '0', '--pitch', '0',
            '--yaw', LaunchConfiguration('cam_yaw'),
            '--frame-id', LaunchConfiguration('parent_frame'),
            '--child-frame-id', LaunchConfiguration('camera_frame'),
        ],
        condition=UnlessCondition(LaunchConfiguration('optical')),
    )

    adapter = Node(
        package='perception_adapter', executable='adapter_node',
        name='perception_adapter', output='screen',
        parameters=[{
            'trigger_class': LaunchConfiguration('trigger_class'),
            'min_confidence': LaunchConfiguration('min_confidence'),
            'confirm_frames': LaunchConfiguration('confirm_frames'),
            'max_range': LaunchConfiguration('max_range'),
            'use_fixed_range': LaunchConfiguration('use_fixed_range'),
            'fixed_range': LaunchConfiguration('fixed_range'),
            'target_frame': 'map',
        }],
    )

    return LaunchDescription(args + [tf_optical, tf_plain, adapter])
