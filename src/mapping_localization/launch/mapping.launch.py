# src/mapping_localization/launch/mapping.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 패키지 경로 설정
    mapping_pkg = get_package_share_directory('mapping_localization')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox') # 공식 패키지 경로
    
    # 2. 파라미터 파일 경로
    slam_params_file = os.path.join(mapping_pkg, 'config', 'mapper_params_online_async.yaml')

    # 3. SLAM Toolbox 공식 런치 파일 호출
    # 공식 파일 내부의 복잡한 Lifecycle 로직이 자동으로 수행
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox_pkg, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': 'false' # 브릿지 환경에 맞게 false 설정
        }.items()
    )

    return LaunchDescription([
        slam_launch
    ])