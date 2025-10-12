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
        output='screen',
        respawn=LaunchConfiguration('respawn')
    )

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('respawn', default_value='true'),

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
