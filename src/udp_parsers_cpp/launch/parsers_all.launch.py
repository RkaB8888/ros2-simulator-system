# udp_parsers_cpp/launch/parsers_all.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def n(exec_name, name=None):
    return Node(
        package='udp_parsers_cpp',
        executable=exec_name,
        name=name or exec_name,
        namespace=LaunchConfiguration('ns'),   # ← 네임스페이스 주입 (멀티 인스턴스/로봇 대비)
        output='screen',
        arguments=[       # <-- 로그 레벨 설정
            '--ros-args', 
            '--log-level', LaunchConfiguration('log_level')
        ],
        respawn=LaunchConfiguration('respawn')
    )

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value=''), # 기본은 루트 네임스페이스
        DeclareLaunchArgument('respawn', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),

        # 상태류
        n('ego_status_parser'),
        n('env_status_parser'),
        n('iot_status_parser'),
        n('custom_object_parser'),

        # 센서류
        n('imu_parser'),
        n('lidar_parser'),
        n('camera_jpeg_parser'),
    ])
