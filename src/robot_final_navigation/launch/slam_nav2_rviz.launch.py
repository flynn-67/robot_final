from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # === Launch arguments ===
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')

    # nav2_bringup 패키지 기준 경로
    bringup_dir = get_package_share_directory('nav2_bringup')
    slam_nav2_launch = os.path.join(bringup_dir, 'launch', 'slam_launch.py')
    nav2_default_rviz = os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz')

    # nav2_bringup의 slam_launch.py는 slam_toolbox + nav2를 같이 띄워줌:contentReference[oaicite:0]{index=0}
    slam_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_nav2_launch),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            # nav2 params_file은 일단 기본값 사용, 필요하면 나중에 네가 커스텀 yaml 연결
            'params_file': params_file,
            'autostart': 'True',
            'use_respawn': 'False',
            'log_level': 'info',
        }.items(),
    )

    # RViz2 실행 (Nav2 기본 RViz 설정 사용)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', nav2_default_rviz],
        output='screen',
    )

    # === LaunchDescription ===
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'namespace',
                default_value='',
                description='Top-level namespace',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='False',  # 실제 LIMO 쓸 때는 False, 시뮬이면 True로
                description='Use simulation (Gazebo) clock if true',
            ),
            DeclareLaunchArgument(
                'params_file',
                default_value=os.path.join(
                    bringup_dir, 'params', 'nav2_params.yaml'
                ),
                description=(
                    'Full path to the ROS2 parameters file to use for all launched nodes'
                ),
            ),
            slam_nav2,
            rviz_node,
        ]
    )
