# raw_4ports.launch.py
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
        n(8002, 'object_info_raw'),
        n(8202, 'ego_status_raw'),
        n(8302, 'stuff_info_raw'),
    ])
