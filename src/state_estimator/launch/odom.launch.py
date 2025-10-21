# src/state_estimator/launch/odom.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('state_estimator')
    default_params = os.path.join(pkg, 'params', 'odom_publisher.yaml')

    # 기본은 루트 네임스페이스(빈 문자열이 루트)
    ns_arg      = DeclareLaunchArgument('ns', default_value='')
    params_arg  = DeclareLaunchArgument('params', default_value=default_params)

    # 리맵 훅: 지금은 동일 이름으로 두고, 필요 시 실행 인자로 바꿈
    ego_in_arg  = DeclareLaunchArgument('ego_in',  default_value='ego_status')
    odom_out_arg= DeclareLaunchArgument('odom_out', default_value='odom')

    # log 레벨 인자 추가
    log_level = DeclareLaunchArgument('log_level', default_value='info')
    
    return LaunchDescription([
        ns_arg, params_arg, ego_in_arg, odom_out_arg, log_level,
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
            parameters=[LaunchConfiguration('params')],
            remappings=[
                ('ego_status', LaunchConfiguration('ego_in')), # 입력 훅
                ('odom',       LaunchConfiguration('odom_out')) # 출력 훅
            ]
        )
    ])
