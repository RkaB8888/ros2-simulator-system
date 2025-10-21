# udp_raw_bridge/launch/raw_6ports.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def n(id_hint: str):
    # 노드 이름은 YAML 키와 동일(udp_rx_*)
    return Node(
        package='udp_raw_bridge',
        executable='udp_raw_node',
        name=f'udp_rx_{id_hint}',
        namespace=LaunchConfiguration('ns'),  # ← 멀티 인스턴스 대비
        output='screen',
        arguments=[
            '--ros-args', 
            '--log-level', LaunchConfiguration('log_level')
        ],
        respawn=LaunchConfiguration('respawn'),
        parameters=[LaunchConfiguration('config_file')]
    )

def generate_launch_description():
    default_cfg = os.path.join(
        get_package_share_directory('bridge_bringup'),
        'config', 'system.wsl.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value=''),         # 기본 루트
        DeclareLaunchArgument('respawn',     default_value='false'), # RX는 바인딩 충돌 방지 위해 기본 false 권장
        DeclareLaunchArgument('config_file', default_value=default_cfg),
        DeclareLaunchArgument('log_level', default_value='info'),
        
        LogInfo(msg=['[udp_raw_bridge] using config: ', LaunchConfiguration('config_file')]),

        # YAML의 키(udp_rx_env, udp_rx_iot, ...)와 반드시 일치
        n('env'),
        n('iot'),
        n('ego'),
        n('object'),
        n('imu'),
        n('camera'),
        n('lidar'),
    ])
