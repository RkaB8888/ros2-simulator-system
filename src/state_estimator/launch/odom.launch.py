# src/state_estimator/launch/odom.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('state_estimator')

    # 1. 파일 경로 설정
    # 바퀴 오도메트리 설정
    odom_params_file = os.path.join(pkg_share, 'params', 'odom_publisher.yaml')
    # EKF 설정
    ekf_params_file = os.path.join(pkg_share, 'params', 'ekf.yaml')

    # 리맵 훅: 지금은 동일 이름으로 두고, 필요 시 실행 인자로 바꿈
    ego_in_arg  = DeclareLaunchArgument('ego_in',  default_value='ego_status')
    
    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value=''),
        DeclareLaunchArgument('log_level', default_value='info'),
        # 입력 토픽 설정
        DeclareLaunchArgument('ego_in', default_value='ego_status'),

        # ---------------------------------------------------------
        # 1. Wheel Odometry Node (재료 생산)
        # ---------------------------------------------------------
        Node(
            package='state_estimator',
            executable='odom_publisher',
            name='odom_publisher',
            namespace=LaunchConfiguration('ns'),          # ← 네임스페이스 주입
            output='screen',
            arguments=[       # <-- 로그 레벨 설정
            '--ros-args', 
            '--log-level', LaunchConfiguration('log_level')
            ],
            parameters=[odom_params_file],
            remappings=[
                ('ego_status', LaunchConfiguration('ego_in')),
                ('odom',       'wheel/odom')
            ]
        ),

        # ---------------------------------------------------------
        # 2. Robot Localization EKF Node (완성품 생산)
        # ---------------------------------------------------------
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            namespace=LaunchConfiguration('ns'),
            output='screen',
            parameters=[ekf_params_file],
            remappings=[
                # EKF는 'odometry/filtered'로 내보내지만, 우리는 표준인 'odom'을 원함
                ('odometry/filtered', 'odom')
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        ),
    ])
