# udp_tx_bridge/launch/tx_all.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def n(exec_name, name):
    return Node(
        package='udp_tx_bridge',
        executable=exec_name,
        name=name,
        output='screen',
        respawn=LaunchConfiguration('respawn'),
        parameters=[LaunchConfiguration('config_file')]
    )


def generate_launch_description():
    # 기본 config 파일: 통합 bringup 패키지 경로
    default_cfg = os.path.join(
        get_package_share_directory('bridge_bringup'),
        'config', 'system.wsl.yaml'  # 통합 config 참조
    )

    return LaunchDescription([
        DeclareLaunchArgument('respawn',     default_value='true'),
        DeclareLaunchArgument('config_file', default_value=default_cfg),

        n('udp_tx_cmd_vel_node',      'udp_tx_cmd_vel'),
        n('udp_tx_hand_control_node', 'udp_tx_hand_control'),
        n('udp_tx_iot_control_node',  'udp_tx_iot_control'),
    ])
