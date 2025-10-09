# src/udp_raw_bridge/launch/raw_6ports.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def n(port, name):
    return Node(
        package='udp_raw_bridge',
        executable='udp_raw_node',
        name=f'udp_raw_{port}',
        parameters=[{'listen_port': port, 'topic_name': name}],
        output='screen'
    )

def generate_launch_description():
    return LaunchDescription([
        n(7802, 'env_info_raw'),
        n(8002, 'iot_status_raw'),
        n(8202, 'ego_status_raw'),
        n(8302, 'object_info_raw'),
        n(9092, 'imu_raw'),
        n(1232, 'camera_jpeg_raw'),
        n(9094, 'lidar_raw'),
    ])
