# src/safety_bringup/launch/safety_chain.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ns = LaunchConfiguration('ns')
    use_ns = DeclareLaunchArgument('ns', default_value='', description='namespace')
    log_level = DeclareLaunchArgument('log_level', default_value='info', description='log level')

    # yaml 경로
    pkg_share = get_package_share_directory('safety_bringup')
    mux_yaml   = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    smooth_yaml= os.path.join(pkg_share, 'config', 'velocity_smoother.yaml')
    cm_yaml    = os.path.join(pkg_share, 'config', 'collision_monitor.yaml')

    # 1. Twist Mux (일반 노드)
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        namespace=ns,
        name='twist_mux',
        parameters=[mux_yaml],
        remappings=[('cmd_vel_out', 'cmd_vel_mux')],
        output='screen',
        arguments=[
            '--ros-args',
            '--log-level', LaunchConfiguration('log_level')
        ]
    )

    # 2. Velocity Smoother (Lifecycle 노드)
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        namespace=ns,
        name='velocity_smoother',
        parameters=[smooth_yaml],
        remappings=[
            ('cmd_vel', 'cmd_vel_mux'), # 입력: Twist Mux의 출력
            ('cmd_vel_smoothed', 'cmd_vel_smooth') # 출력: Monitor로 전달
        ],
        output='screen',
        arguments=[
            '--ros-args', 
            '--log-level', LaunchConfiguration('log_level')
        ]
    )

    # 3. Collision Monitor (Lifecycle 노드)
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        namespace=ns,
        name='collision_monitor',
        parameters=[cm_yaml],
        output='screen',
        arguments=[
            '--ros-args',
            '--log-level', LaunchConfiguration('log_level')
        ]
    )

    # 4. Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_safety',
        namespace=ns,
        output='screen',
        parameters=[
            {'autostart': True}, # 실행 시 자동으로 Active 상태로 전환
            {'node_names': ['velocity_smoother', 'collision_monitor']}
        ]
    )

    return LaunchDescription([use_ns, log_level, twist_mux, velocity_smoother, collision_monitor, lifecycle_manager])
