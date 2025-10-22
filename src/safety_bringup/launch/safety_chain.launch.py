# src/safety_bringup/launch/safety_chain.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ns = LaunchConfiguration('ns')
    use_ns = DeclareLaunchArgument('ns', default_value='', description='namespace')
    log_level = DeclareLaunchArgument('log_level', default_value='info', description='log level')

    # yaml 경로
    from ament_index_python.packages import get_package_share_directory
    import os
    pkg_share = get_package_share_directory('safety_bringup')
    mux_yaml   = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    smooth_yaml= os.path.join(pkg_share, 'config', 'velocity_smoother.yaml')
    cm_yaml    = os.path.join(pkg_share, 'config', 'collision_monitor.yaml')

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

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        namespace=ns,
        name='velocity_smoother',
        parameters=[smooth_yaml],
        remappings=[
            ('cmd_vel', 'cmd_vel_mux'),
            ('cmd_vel_smoothed', 'cmd_vel_smooth')
        ],
        output='screen',
        arguments=[
            '--ros-args', 
            '--log-level', LaunchConfiguration('log_level')
        ]
    )

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

    return LaunchDescription([use_ns, log_level, twist_mux, velocity_smoother, collision_monitor])
